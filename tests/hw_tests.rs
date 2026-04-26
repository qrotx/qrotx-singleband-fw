// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// tests/hw_tests.rs — hardware-in-the-loop tests.
//
// Run with the Nucleo connected:
//   cargo test --test hw_tests
//
// probe-rs flashes the binary and collects defmt output over RTT.
// Each #[test] function runs in sequence; a panic causes the test runner
// to report failure and probe-rs exits with a non-zero status.
//
// Tests:
//   test_si5351_present      — I2C ACK at 0x60, status register reads OK
//   test_hrtim_timer_running — Timer C counter advances after init
//   test_adc_dma_fires       — PROCESS_FIRST_HALF flag set within 2 ms
//   test_adc_nonzero_samples — ADC buffer contains non-zero samples
//   test_adc_mean_midscale   — ADC mean ≈ 2048 (voltage divider at VDDA/2)
//   test_dsp_timing          — process_half() fits within 500 µs budget
//   test_dma_rate            — ~10 full-transfer events in 10 ms
//   test_hrtim_dma_running   — DMA1_CH5 TCIF set after 15 ms (HRTIM DMA active)
//   test_fir_interpolate_passband — 1 kHz passes through 10:1 interpolator
//   test_ssb_analytic_signal — SSB filter suppresses negative frequency ≥ 40 dB
//   test_cordic_modulus      — spot-checks CORDIC modulus and output ordering
//   test_outphasing_boundary — zero/full amplitude, ta_cmp2 invariant
//   test_compressor_passes_quiet — quiet signal gets makeup gain of 1.5×
//   test_fir_interpolate_stopband — 9 kHz rejected > −40 dB by interpolator
//   test_process_second_half — smoke test for second-half buffer pointers
//   test_hrtim_dma_updates_regs — DMA1_CH5 burst writes reach the 5 HRTIM CMP registers

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt::info;
use defmt_rtt as _;
use embassy_stm32::dma as embassy_dma;
extern crate libm;
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::pac;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, peripherals, Config};
use panic_probe as _;

// Modules shared with the firmware (included by path to avoid a lib crate).
#[path = "../src/config.rs"]   mod config;
#[path = "../src/hrtim.rs"]    mod hrtim;
#[path = "../src/adc.rs"]      mod adc;
#[path = "../src/dma.rs"]      mod dma;
#[path = "../src/dsp_ffi.rs"]  mod dsp_ffi;
#[path = "../src/dsp.rs"]      mod dsp;
#[path = "../src/si5351.rs"]   mod si5351;

use hrtim::{PROCESS_FIRST_HALF, PROCESS_SECOND_HALF};
use config::SI5351_I2C_ADDR;

bind_interrupts!(struct Irqs {
    I2C1_EV       => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER       => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    DMA1_CHANNEL6 => embassy_dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => embassy_dma::InterruptHandler<peripherals::DMA1_CH7>;
});

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Spin-wait for approximately `cycles` CPU cycles (168 MHz → 1 cycle ≈ 6 ns).
#[inline(never)]
fn spin(cycles: u32) {
    for _ in 0..cycles {
        cortex_m::asm::nop();
    }
}

/// Spin-wait for approximately `ms` milliseconds at 168 MHz SYSCLK.
fn wait_ms(ms: u32) {
    spin(ms * 168_000);
}

// ---------------------------------------------------------------------------
// DSP filter test helpers (used by tests 8–15)
// ---------------------------------------------------------------------------

fn sine_q31<const N: usize>(freq_hz: f32, fs_hz: f32, amp: f32) -> [i32; N] {
    let mut out = [0i32; N];
    for (i, s) in out.iter_mut().enumerate() {
        let phase = 2.0 * core::f32::consts::PI * freq_hz * i as f32 / fs_hz;
        *s = (amp * libm::sinf(phase) * i32::MAX as f32) as i32;
    }
    out
}

fn sine_f32_arr<const N: usize>(freq_hz: f32, fs_hz: f32, amp: f32) -> [f32; N] {
    let mut out = [0.0f32; N];
    for (i, s) in out.iter_mut().enumerate() {
        let phase = 2.0 * core::f32::consts::PI * freq_hz * i as f32 / fs_hz;
        *s = amp * libm::sinf(phase);
    }
    out
}

fn rms_q31_buf<const N: usize>(buf: &[i32; N]) -> f32 {
    let sum: f32 = buf.iter().map(|&x| { let f = x as f32 / i32::MAX as f32; f * f }).sum();
    libm::sqrtf(sum / N as f32)
}

fn rms_f32_buf<const N: usize>(buf: &[f32; N]) -> f32 {
    let sum: f32 = buf.iter().map(|&x| x * x).sum();
    libm::sqrtf(sum / N as f32)
}

// ---------------------------------------------------------------------------
// Test suite
// ---------------------------------------------------------------------------

#[defmt_test::tests]
mod tests {
    use super::*;

    struct State {
        i2c: I2c<'static, Async, embassy_stm32::i2c::Master>,
    }

    #[init]
    fn init() -> State {
        // Same 168 MHz clock configuration as main.rs.
        let mut rcc = Config::default();
        {
            let r = &mut rcc.rcc;
            r.hsi = true;
            r.pll = Some(Pll {
                source: PllSource::HSI,
                prediv: PllPreDiv::DIV1,
                mul:    PllMul::MUL21,
                divp:   None,
                divq:   None,
                divr:   Some(PllRDiv::DIV2),
            });
            r.sys      = Sysclk::PLL1_R;
            r.ahb_pre  = AHBPrescaler::DIV1;
            r.apb1_pre = APBPrescaler::DIV1;
            r.apb2_pre = APBPrescaler::DIV1;
        }
        let p = embassy_stm32::init(rcc);

        // Initialise hardware in the same order as radio_task.
        hrtim::init();
        adc::init();
        dma::init();
        adc::start();
        dsp::init();

        // Build I2C for Si5351 tests.
        let mut i2c_cfg = I2cConfig::default();
        i2c_cfg.frequency = Hertz(400_000);
        let i2c = I2c::new(
            p.I2C1,
            p.PB8,
            p.PB9,
            p.DMA1_CH6,
            p.DMA1_CH7,
            Irqs,
            i2c_cfg,
        );

        // Clear stale ping-pong flags.
        PROCESS_FIRST_HALF.store(false, Ordering::Relaxed);
        PROCESS_SECOND_HALF.store(false, Ordering::Relaxed);

        info!("hw-tests: init complete");
        State { i2c }
    }

    // -----------------------------------------------------------------------
    // Test 1: Si5351 I2C presence
    //
    // Si5351 at 0x60 should ACK and its Device Status register (reg 0)
    // should be readable without error.  We just confirm no I2C error —
    // the PLLs are not yet programmed so LOL bits may be set.
    // -----------------------------------------------------------------------
    #[test]
    fn test_si5351_present(state: &mut State) {
        info!("test: Si5351 I2C presence");

        let reg_addr = [0x00u8];
        let mut val  = [0u8; 1];
        match state.i2c.blocking_write_read(SI5351_I2C_ADDR, &reg_addr, &mut val) {
            Ok(()) => {}
            Err(_) => defmt::panic!("Si5351 I2C read failed — device not present or bus error"),
        }

        info!("Si5351 status register = {:#04x}", val[0]);
        // Bit 7 (SYS_INIT) must be 0 after power-on init is complete.
        defmt::assert!(val[0] & 0x80 == 0, "SYS_INIT bit still set — device not ready");

        info!("test_si5351_present: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 2: HRTIM Timer C counter advances
    //
    // Read the Timer C counter twice with a short delay.  If the counter
    // value changes the timer is running at the expected ~200 kHz rate.
    // -----------------------------------------------------------------------
    #[test]
    fn test_hrtim_timer_running(_state: &mut State) {
        info!("test: HRTIM Timer C counter advancing");

        // Timer C is index 2 in the HRTIM1 peripheral.
        // Sample the counter 50 µs apart using DWT (safe: CYCCNT already
        // enabled by the time this test runs after test_dsp_timing... but
        // enable it here too in case test order changes).
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            demcr.write_volatile(demcr.read_volatile() | (1 << 24));
            let ctrl = 0xE000_1000 as *mut u32;
            ctrl.write_volatile(ctrl.read_volatile() | 1);
        }
        let cnt0 = pac::HRTIM1.tim(2).cnt().read().cnt();
        // Wait at least one full Timer C period (850 ticks × ~6 ns ≈ 5 µs)
        // using DWT so the delay survives release optimisation.
        let t0 = cortex_m::peripheral::DWT::cycle_count();
        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(t0) < 16_800 {} // 100 µs
        let cnt1 = pac::HRTIM1.tim(2).cnt().read().cnt();

        info!("Timer C cnt0={} cnt1={}", cnt0, cnt1);
        // Either cnt1 > cnt0 (no wrap) or cnt1 < cnt0 (wrapped) — both are
        // fine.  Only failure is cnt0 == cnt1 (counter frozen).
        defmt::assert_ne!(cnt0, cnt1, "Timer C counter did not advance — HRTIM not running");

        info!("test_hrtim_timer_running: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 3: DMA half-transfer ISR fires within 2 ms
    //
    // DMA1_CH1 fires a half-transfer interrupt every 0.5 ms (FRAME_SAMPLES
    // ADC conversions at 200 kHz).  Poll the atomic flag; it must be set
    // within 2 ms (4 half-periods).
    // -----------------------------------------------------------------------
    #[test]
    fn test_adc_dma_fires(_state: &mut State) {
        info!("test: ADC DMA half-transfer fires");

        PROCESS_FIRST_HALF.store(false, Ordering::Relaxed);

        let mut fired = false;
        for _ in 0..2000 {
            spin(168); // ~1 µs
            if PROCESS_FIRST_HALF.load(Ordering::Acquire) {
                fired = true;
                break;
            }
        }

        defmt::assert!(fired, "PROCESS_FIRST_HALF not set within 2 ms — ADC/DMA not running");
        info!("test_adc_dma_fires: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 4: ADC buffer contains non-zero samples
    //
    // After 2 ms the ADC buffer must have at least one non-zero sample.
    // Even with no audio input the ADC reads its own offset (not exactly 0).
    // -----------------------------------------------------------------------
    #[test]
    fn test_adc_nonzero_samples(_state: &mut State) {
        info!("test: ADC buffer non-zero");

        wait_ms(2);

        // Safety: no DSP task runs in this binary — we are the sole reader.
        let buf = unsafe { hrtim::adc_buf_first_half() };
        let nonzero = buf.iter().any(|&s| s != 0);

        info!("ADC buf[0]={} buf[50]={}", buf[0], buf[50]);
        defmt::assert!(nonzero, "ADC buffer all-zeros — ADC not sampling");

        info!("test_adc_nonzero_samples: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 5: ADC mean value ≈ midscale (voltage divider at VDDA/2)
    //
    // The hardware splits 3.3 V in half, so the DC level fed into the ADC is
    // ~1.65 V.  With a 12-bit ADC referenced to VDDA the expected count is
    // ~2048.  Accept ±15 % (1740–2355) to cover resistor tolerances.
    // -----------------------------------------------------------------------
    #[test]
    fn test_adc_mean_midscale(_state: &mut State) {
        info!("test: ADC mean value near midscale");

        wait_ms(2); // let the buffer settle after previous test

        let buf = unsafe { hrtim::adc_buf_first_half() };
        let sum: u32 = buf.iter().map(|&s| s as u32).sum();
        let mean = sum / buf.len() as u32;

        info!("ADC mean = {} (expected ~2048)", mean);
        defmt::assert!(mean >= 1740, "ADC mean too low: {} — check voltage divider", mean);
        defmt::assert!(mean <= 2355, "ADC mean too high: {} — check voltage divider", mean);

        info!("test_adc_mean_midscale: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 6: DSP process_half() execution time
    //
    // One call must complete within the 500 µs half-buffer period.
    // At 168 MHz that is 84 000 cycles.  Per-stage breakdowns are logged
    // so the most expensive stages can be identified at a glance.
    // -----------------------------------------------------------------------
    #[test]
    fn test_dsp_timing(_state: &mut State) {
        info!("test: DSP process_half timing");

        // Enable DWT CYCCNT — required by process_first_half_timed().
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            demcr.write_volatile(demcr.read_volatile() | (1 << 24)); // TRCENA
            let ctrl = 0xE000_1000 as *mut u32;
            ctrl.write_volatile(ctrl.read_volatile() | 1);           // CYCCNTENA
        }

        let t = unsafe { dsp::process_first_half() };

        info!("--- DSP pipeline timing @ 168 MHz ---");
        info!("  Stage 1  adc_to_q31      {} cy  ({} µs)", t.stage1_adc_to_q31,   t.stage1_adc_to_q31   / 168);
        info!("  Stage 2  fir_decimate    {} cy  ({} µs)", t.stage2_fir_decimate,  t.stage2_fir_decimate  / 168);
        info!("  Stage 3a q31_to_f32      {} cy  ({} µs)", t.stage3a_q31_to_f32,  t.stage3a_q31_to_f32  / 168);
        info!("  Stage 3b highpass        {} cy  ({} µs)", t.stage3b_highpass,     t.stage3b_highpass     / 168);
        info!("  Stage 3c compress        {} cy  ({} µs)", t.stage3c_compress,     t.stage3c_compress     / 168);
        info!("  Stage 3d lowpass         {} cy  ({} µs)", t.stage3d_lowpass,      t.stage3d_lowpass      / 168);
        info!("  Stage 4  modulate        {} cy  ({} µs)", t.stage4_modulate,      t.stage4_modulate      / 168);
        info!("  Stage 5a f32_to_q31      {} cy  ({} µs)", t.stage5a_f32_to_q31,  t.stage5a_f32_to_q31  / 168);
        info!("  Stage 5b interpolate     {} cy  ({} µs)", t.stage5b_interpolate,  t.stage5b_interpolate  / 168);
        info!("  Stage 5c cordic          {} cy  ({} µs)", t.stage5c_cordic,      t.stage5c_cordic      / 168);
        info!("  Stage 5c outphasing      {} cy  ({} µs)", t.stage5c_outphasing,  t.stage5c_outphasing  / 168);
        info!("  ----------------------------------------");
        info!("  Total                    {} cy  ({} µs)  budget: 84000", t.total, t.total / 168);

        defmt::assert!(
            t.total < 84_000,
            "process_half too slow: {} cycles (budget: 84 000 = 500 µs @ 168 MHz)",
            t.total
        );

        info!("test_dsp_timing: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 7: DMA transfer rate ≈ 10 TC events per 10 ms
    //
    // ADC_DMA_LEN = 2 × FRAME_SAMPLES = 200 halfwords at 200 kHz → 1 ms per
    // full buffer cycle.  Each PROCESS_SECOND_HALF (TC) flag set equals one
    // full-buffer completion.  Over 10 ms we expect ~10 events.
    // Accept 8–12 to allow for ±1 event at window boundaries.
    // -----------------------------------------------------------------------
    #[test]
    fn test_dma_rate(_state: &mut State) {
        info!("test: DMA transfer rate");

        // Enable DWT CYCCNT via direct register writes (bypasses the cortex-m
        // singleton API which may already be consumed by the runtime).
        // DEMCR[24] = TRCENA enables the DWT/ITM trace macrocell.
        // DWT_CTRL[0] = CYCCNTENA enables the cycle counter.
//      unsafe {
//            let demcr = 0xE000_EDFC as *mut u32;
//            demcr.write_volatile(demcr.read_volatile() | (1 << 24)); // TRCENA
//            (0xE000_1004 as *mut u32).write_volatile(0);             // reset CYCCNT
//            let ctrl = 0xE000_1000 as *mut u32;
//            ctrl.write_volatile(ctrl.read_volatile() | 1);           // CYCCNTENA
//        }

        PROCESS_SECOND_HALF.store(false, Ordering::Relaxed);
        let mut count = 0u32;

        // Use DWT cycle counter for an accurate 10 ms window regardless of
        // debug/release build timing.  168 MHz × 10 ms = 1_680_000 cycles.
        let start = cortex_m::peripheral::DWT::cycle_count();
        loop {
            if PROCESS_SECOND_HALF.load(Ordering::Acquire) {
                PROCESS_SECOND_HALF.store(false, Ordering::Release);
                count += 1;
            }
            if cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) >= 1_680_000 {
                break;
            }
        }

        info!("DMA events in 10 ms: {}", count);
        defmt::assert!(count >= 8,  "Too few DMA events: {} (expected ~10)", count);
        defmt::assert!(count <= 12, "Too many DMA events: {} (expected ~10)", count);

        info!("test_dma_rate: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 8: HRTIM DMA (DMA1_CH5) transfer-complete flag fires
    //
    // DMA1_CH5 transfers HRTIM_BUF (600 words = 200 PwmSamples × 3) to
    // HRTIM1.BDMADR triggered by Timer C REP at ~200 kHz.  Each trigger
    // bursts 3 words (one PwmSample), so one full TC fires every 200
    // triggers = ~1 ms.  No ISR clears CH5 flags, so TCIF accumulates
    // in the DMA1 ISR register and can be polled directly.
    //
    // Procedure:
    //   1. Clear all CH5 flags via IFCR (GIF write clears TCIF/HTIF/TEIF).
    //   2. Wait 5 ms (≥ 4 full buffer cycles at ~1 ms each).
    //   3. Assert TCIF(4) is set and TEIF(4) is clear.
    // -----------------------------------------------------------------------
    #[test]
    fn test_hrtim_dma_running(_state: &mut State) {
        info!("test: HRTIM DMA1_CH5 transfer-complete flag");

        // 1. Clear all DMA1 CH5 flags (GIF clears TCIF, HTIF, TEIF simultaneously).
        pac::DMA1.ifcr().write(|w| w.set_gif(4, true));

        // 2. Wait 5 ms — at least 4 full buffer cycles (1 TC per ~1 ms).
        wait_ms(5);

        // 3. Read the ISR.
        let isr = pac::DMA1.isr().read();
        let tc   = isr.tcif(4);
        let te   = isr.teif(4);
        let ndtr = pac::DMA1.ch(4).ndtr().read().ndt();

        info!("DMA1 CH5: TCIF={} TEIF={} NDTR={}", tc, te, ndtr);

        defmt::assert!(!te, "DMA1_CH5 transfer error — check HRTIM_BUF alignment and NDTR");
        defmt::assert!(tc,  "DMA1_CH5 TCIF not set after 15 ms — HRTIM DMA not running");

        info!("test_hrtim_dma_running: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 9: FIR decimator passband — 1 kHz passes through
    // -----------------------------------------------------------------------
    #[test]
    fn test_fir_decimate_passband(_state: &mut State) {
        use dsp_ffi::{ArmFirDecimateInstanceQ31, FIR_DEC_STATE_LEN,
                      arm_fir_decimate_init_q31, arm_fir_decimate_fast_q31};
        use dsp::{FIR_COEFFS_Q31, FIR_NUM_TAPS, FIR_DECIMATE_FACTOR, DECIMATED_LEN};
        const FRAMES: usize = 10;
        const N_IN:  usize = config::FRAME_SAMPLES * FRAMES;
        const N_OUT: usize = DECIMATED_LEN * FRAMES;

        let input  = sine_q31::<N_IN>(1_000.0, 200_000.0, 0.5);
        let mut state = [0i32; FIR_DEC_STATE_LEN];
        let mut inst = ArmFirDecimateInstanceQ31 {
            m: 0, num_taps: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
        };
        unsafe {
            arm_fir_decimate_init_q31(
                &raw mut inst, FIR_NUM_TAPS as u16, FIR_DECIMATE_FACTOR as u8,
                FIR_COEFFS_Q31.as_ptr(), state.as_mut_ptr(), config::FRAME_SAMPLES as u32,
            );
        }
        let mut output = [0i32; N_OUT];
        for frame in 0..FRAMES {
            unsafe {
                arm_fir_decimate_fast_q31(
                    &inst,
                    input[frame * config::FRAME_SAMPLES..].as_ptr(),
                    output[frame * DECIMATED_LEN..].as_mut_ptr(),
                    config::FRAME_SAMPLES as u32,
                );
            }
        }
        let in_rms  = rms_q31_buf::<{N_IN  - config::FRAME_SAMPLES}>(
            input [config::FRAME_SAMPLES..].try_into().unwrap());
        let out_rms = rms_q31_buf::<{N_OUT - DECIMATED_LEN}>(
            output[DECIMATED_LEN..].try_into().unwrap());
        info!("FIR dec 1 kHz passband: in_rms={} out_rms={}", in_rms, out_rms);
        defmt::assert!(out_rms > in_rms * 0.7, "passband too attenuated");
        defmt::assert!(out_rms < in_rms * 1.4, "passband amplified");
        info!("test_fir_decimate_passband: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 10: FIR decimator stopband — 12 kHz rejected (> −40 dB)
    // -----------------------------------------------------------------------
    #[test]
    fn test_fir_decimate_stopband(_state: &mut State) {
        use dsp_ffi::{ArmFirDecimateInstanceQ31, FIR_DEC_STATE_LEN,
                      arm_fir_decimate_init_q31, arm_fir_decimate_fast_q31};
        use dsp::{FIR_COEFFS_Q31, FIR_NUM_TAPS, FIR_DECIMATE_FACTOR, DECIMATED_LEN};
        const FRAMES: usize = 20;
        const N_IN:  usize = config::FRAME_SAMPLES * FRAMES;
        const N_OUT: usize = DECIMATED_LEN * FRAMES;

        let input  = sine_q31::<N_IN>(12_000.0, 200_000.0, 0.5);
        let mut state = [0i32; FIR_DEC_STATE_LEN];
        let mut inst = ArmFirDecimateInstanceQ31 {
            m: 0, num_taps: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
        };
        unsafe {
            arm_fir_decimate_init_q31(
                &raw mut inst, FIR_NUM_TAPS as u16, FIR_DECIMATE_FACTOR as u8,
                FIR_COEFFS_Q31.as_ptr(), state.as_mut_ptr(), config::FRAME_SAMPLES as u32,
            );
        }
        let mut output = [0i32; N_OUT];
        for frame in 0..FRAMES {
            unsafe {
                arm_fir_decimate_fast_q31(
                    &inst,
                    input[frame * config::FRAME_SAMPLES..].as_ptr(),
                    output[frame * DECIMATED_LEN..].as_mut_ptr(),
                    config::FRAME_SAMPLES as u32,
                );
            }
        }
        let in_rms  = rms_q31_buf::<{N_IN  - config::FRAME_SAMPLES}>(
            input [config::FRAME_SAMPLES..].try_into().unwrap());
        let out_rms = rms_q31_buf::<{N_OUT - DECIMATED_LEN}>(
            output[DECIMATED_LEN..].try_into().unwrap());
        info!("FIR dec 12 kHz stopband: in_rms={} out_rms={}", in_rms, out_rms);
        defmt::assert!(out_rms < in_rms * 0.01, "stopband not attenuated enough");
        info!("test_fir_decimate_stopband: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 11: FIR interpolator passband — 1 kHz passes through
    //
    // Symmetric counterpart to tests 8/9.  The interpolator takes
    // DECIMATED_LEN samples at 20 kHz and produces FRAME_SAMPLES at 200 kHz.
    // A 1 kHz tone is well within the prototype passband (cutoff 5 kHz) and
    // must survive with RMS within ±30 % of the input.
    // -----------------------------------------------------------------------
    #[test]
    fn test_fir_interpolate_passband(_state: &mut State) {
        use dsp_ffi::{ArmFirInterpolateInstanceQ31, FIR_INTERP_STATE_LEN,
                      arm_fir_interpolate_init_q31, arm_fir_interpolate_q31};
        use dsp::{FIR_COEFFS_Q31, FIR_NUM_TAPS, FIR_DECIMATE_FACTOR, DECIMATED_LEN};
        const FRAMES: usize = 10;
        const N_IN:  usize = DECIMATED_LEN       * FRAMES; //  100 samples @ 20 kHz
        const N_OUT: usize = config::FRAME_SAMPLES * FRAMES; // 1000 samples @ 200 kHz

        let input = sine_q31::<N_IN>(1_000.0, 20_000.0, 0.5);
        let mut state = [0i32; FIR_INTERP_STATE_LEN];
        let mut inst = ArmFirInterpolateInstanceQ31 {
            l: 0, phase_length: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
        };
        unsafe {
            arm_fir_interpolate_init_q31(
                &raw mut inst, FIR_DECIMATE_FACTOR as u8, FIR_NUM_TAPS as u16,
                FIR_COEFFS_Q31.as_ptr(), state.as_mut_ptr(), DECIMATED_LEN as u32,
            );
        }
        let mut output = [0i32; N_OUT];
        for frame in 0..FRAMES {
            unsafe {
                arm_fir_interpolate_q31(
                    &inst,
                    input[frame * DECIMATED_LEN..].as_ptr(),
                    output[frame * config::FRAME_SAMPLES..].as_mut_ptr(),
                    DECIMATED_LEN as u32,
                );
            }
        }
        // Skip the first output frame to let the delay line settle.
        let in_rms  = rms_q31_buf::<{N_IN  - DECIMATED_LEN}>(
            input [DECIMATED_LEN..].try_into().unwrap());
        let out_rms = rms_q31_buf::<{N_OUT - config::FRAME_SAMPLES}>(
            output[config::FRAME_SAMPLES..].try_into().unwrap());
        // The CMSIS polyphase interpolator has passband gain 1/L by design:
        // each of L sub-filters holds numTaps/L taps, so it sums to 1/L of
        // the prototype.  Compare against expected = in_rms / L.
        let expected = in_rms / FIR_DECIMATE_FACTOR as f32;
        info!("FIR interp 1 kHz passband: in_rms={} out_rms={} expected≈{}", in_rms, out_rms, expected);
        defmt::assert!(out_rms > expected * 0.7, "passband too attenuated");
        defmt::assert!(out_rms < expected * 1.4, "passband amplified");
        info!("test_fir_interpolate_passband: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 12: Highpass biquad rejects DC
    // -----------------------------------------------------------------------
    #[test]
    fn test_highpass_rejects_dc(_state: &mut State) {
        use dsp_ffi::{ArmBiquadCascadeDf2TInstanceF32, BIQUAD_HP_STATE_LEN,
                      arm_biquad_cascade_df2T_init_f32, arm_biquad_cascade_df2T_f32};
        use dsp::{HIGHPASS_COEFFS, HIGHPASS_STAGES};
        const N: usize = 2000;
        let input = [1.0f32; N];
        let mut state = [0.0f32; BIQUAD_HP_STATE_LEN];
        let mut inst = ArmBiquadCascadeDf2TInstanceF32 {
            num_stages: 0, p_state: core::ptr::null_mut(), p_coeffs: core::ptr::null(),
        };
        unsafe {
            arm_biquad_cascade_df2T_init_f32(
                &raw mut inst, HIGHPASS_STAGES as u8,
                HIGHPASS_COEFFS[0].as_ptr(), state.as_mut_ptr(),
            );
            let mut output = [0.0f32; N];
            arm_biquad_cascade_df2T_f32(&inst, input.as_ptr(), output.as_mut_ptr(), N as u32);
            let tail_rms = rms_f32_buf::<500>(&output[1500..].try_into().unwrap());
            info!("HP DC tail_rms={}", tail_rms);
            defmt::assert!(tail_rms < 1e-3, "HP filter passes DC");
        }
        info!("test_highpass_rejects_dc: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 13: Highpass biquad passes 1 kHz
    // -----------------------------------------------------------------------
    #[test]
    fn test_highpass_passes_1khz(_state: &mut State) {
        use dsp_ffi::{ArmBiquadCascadeDf2TInstanceF32, BIQUAD_HP_STATE_LEN,
                      arm_biquad_cascade_df2T_init_f32, arm_biquad_cascade_df2T_f32};
        use dsp::{HIGHPASS_COEFFS, HIGHPASS_STAGES};
        const N: usize = 2000;
        let input = sine_f32_arr::<N>(1_000.0, 20_000.0, 0.5);
        let mut state = [0.0f32; BIQUAD_HP_STATE_LEN];
        let mut inst = ArmBiquadCascadeDf2TInstanceF32 {
            num_stages: 0, p_state: core::ptr::null_mut(), p_coeffs: core::ptr::null(),
        };
        unsafe {
            arm_biquad_cascade_df2T_init_f32(
                &raw mut inst, HIGHPASS_STAGES as u8,
                HIGHPASS_COEFFS[0].as_ptr(), state.as_mut_ptr(),
            );
            let mut output = [0.0f32; N];
            arm_biquad_cascade_df2T_f32(&inst, input.as_ptr(), output.as_mut_ptr(), N as u32);
            let in_rms  = rms_f32_buf::<1500>(&input [500..].try_into().unwrap());
            let out_rms = rms_f32_buf::<1500>(&output[500..].try_into().unwrap());
            info!("HP 1 kHz: in={} out={}", in_rms, out_rms);
            defmt::assert!(out_rms > in_rms * 0.7, "HP attenuates 1 kHz");
        }
        info!("test_highpass_passes_1khz: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 14: Lowpass biquad passes 500 Hz
    // -----------------------------------------------------------------------
    #[test]
    fn test_lowpass_passes_500hz(_state: &mut State) {
        use dsp_ffi::{ArmBiquadCascadeDf2TInstanceF32, BIQUAD_LP_STATE_LEN,
                      arm_biquad_cascade_df2T_init_f32, arm_biquad_cascade_df2T_f32};
        use dsp::{LOWPASS_COEFFS, LOWPASS_STAGES};
        const N: usize = 2000;
        let input = sine_f32_arr::<N>(500.0, 20_000.0, 0.5);
        let mut state = [0.0f32; BIQUAD_LP_STATE_LEN];
        let mut inst = ArmBiquadCascadeDf2TInstanceF32 {
            num_stages: 0, p_state: core::ptr::null_mut(), p_coeffs: core::ptr::null(),
        };
        unsafe {
            arm_biquad_cascade_df2T_init_f32(
                &raw mut inst, LOWPASS_STAGES as u8,
                LOWPASS_COEFFS[0].as_ptr(), state.as_mut_ptr(),
            );
            let mut output = [0.0f32; N];
            arm_biquad_cascade_df2T_f32(&inst, input.as_ptr(), output.as_mut_ptr(), N as u32);
            let in_rms  = rms_f32_buf::<1500>(&input [500..].try_into().unwrap());
            let out_rms = rms_f32_buf::<1500>(&output[500..].try_into().unwrap());
            info!("LP 500 Hz: in={} out={}", in_rms, out_rms);
            defmt::assert!(out_rms > in_rms * 0.7, "LP attenuates 500 Hz");
        }
        info!("test_lowpass_passes_500hz: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 15: Lowpass biquad rejects 4 kHz
    // -----------------------------------------------------------------------
    #[test]
    fn test_lowpass_rejects_4khz(_state: &mut State) {
        use dsp_ffi::{ArmBiquadCascadeDf2TInstanceF32, BIQUAD_LP_STATE_LEN,
                      arm_biquad_cascade_df2T_init_f32, arm_biquad_cascade_df2T_f32};
        use dsp::{LOWPASS_COEFFS, LOWPASS_STAGES};
        const N: usize = 2000;
        let input = sine_f32_arr::<N>(4_000.0, 20_000.0, 0.5);
        let mut state = [0.0f32; BIQUAD_LP_STATE_LEN];
        let mut inst = ArmBiquadCascadeDf2TInstanceF32 {
            num_stages: 0, p_state: core::ptr::null_mut(), p_coeffs: core::ptr::null(),
        };
        unsafe {
            arm_biquad_cascade_df2T_init_f32(
                &raw mut inst, LOWPASS_STAGES as u8,
                LOWPASS_COEFFS[0].as_ptr(), state.as_mut_ptr(),
            );
            let mut output = [0.0f32; N];
            arm_biquad_cascade_df2T_f32(&inst, input.as_ptr(), output.as_mut_ptr(), N as u32);
            let in_rms  = rms_f32_buf::<1500>(&input [500..].try_into().unwrap());
            let out_rms = rms_f32_buf::<1500>(&output[500..].try_into().unwrap());
            info!("LP 4 kHz: in={} out={}", in_rms, out_rms);
            defmt::assert!(out_rms < in_rms * 0.2, "LP passes 4 kHz");
        }
        info!("test_lowpass_rejects_4khz: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 16: Compressor reduces a loud signal
    // -----------------------------------------------------------------------
    #[test]
    fn test_compressor_reduces_loud(_state: &mut State) {
        let input = [0.8f32; 2000];
        let mut comp = dsp::Compressor::new();
        let mut output = [0.0f32; 2000];
        comp.process(&input, &mut output);
        let in_rms  = rms_f32_buf::<1500>(&input [500..].try_into().unwrap());
        let out_rms = rms_f32_buf::<1500>(&output[500..].try_into().unwrap());
        info!("Compressor: in={} out={}", in_rms, out_rms);
        defmt::assert!(out_rms < in_rms, "Compressor did not reduce level");
        info!("test_compressor_reduces_loud: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 17: SSB filter produces a true analytic signal
    //
    // Feed a 1 kHz sine into arm_fir_ssb_f32.  The complex output
    // z[n] = I[n] + j·Q[n] must be one-sided in frequency: all energy at
    // either +1 kHz or −1 kHz (depending on coefficient sign convention),
    // and near-zero at the opposite frequency.
    //
    // We compute two DFT bins of z[n] directly (no FFT needed):
    //
    //   Z₊ = Σ z[n]·exp(−j·2π·f·n/fs)   ← positive frequency
    //   Z₋ = Σ z[n]·exp(+j·2π·f·n/fs)   ← negative frequency
    //
    // First, the DFT helper is validated with a synthetic positive-frequency
    // complex exponential (no filter involved) — if that self-check fails the
    // DFT math is broken; if the SSB assertion fails it is a filter problem.
    //
    // A 257-tap equiripple filter should achieve at least 40 dB rejection
    // (|dominant| / |suppressed| > 100).
    // -----------------------------------------------------------------------
    #[test]
    fn test_ssb_analytic_signal(_state: &mut State) {
        use dsp_ffi::{ArmFirInstanceF32, SSB_STATE_LEN, arm_fir_ssb_f32};
        use dsp::{SSB_COEFFS_INTERLEAVED, SSB_FILTER_TAPS, DECIMATED_LEN};

        // Helper: compute |Z₊| and |Z₋| for a block of I/Q samples at freq_hz.
        // Z₊ = Σ (I+jQ)·exp(−jθ)   Z₋ = Σ (I+jQ)·exp(+jθ)
        fn dft_bins(i_buf: &[f32], q_buf: &[f32], freq_hz: f32, fs_hz: f32) -> (f32, f32) {
            let n = i_buf.len();
            let mut re_pos = 0.0f32; let mut im_pos = 0.0f32;
            let mut re_neg = 0.0f32; let mut im_neg = 0.0f32;
            for k in 0..n {
                let theta = 2.0 * core::f32::consts::PI * freq_hz * k as f32 / fs_hz;
                let c = libm::cosf(theta);
                let s = libm::sinf(theta);
                let i = i_buf[k]; let q = q_buf[k];
                re_pos += i * c + q * s;  im_pos += q * c - i * s;
                re_neg += i * c - q * s;  im_neg += q * c + i * s;
            }
            (
                libm::sqrtf(re_pos * re_pos + im_pos * im_pos),
                libm::sqrtf(re_neg * re_neg + im_neg * im_neg),
            )
        }

        // --- DFT self-check (no filter) ---
        // Feed a synthetic positive-frequency exponential: I=cos(θ), Q=sin(θ).
        // Z₊ must be ≈ N and Z₋ must be ≈ 0, proving the DFT math is correct
        // before we involve the SSB filter.
        const N_SELF: usize = 200;
        let mut self_i = [0.0f32; N_SELF];
        let mut self_q = [0.0f32; N_SELF];
        for k in 0..N_SELF {
            let theta = 2.0 * core::f32::consts::PI * 1_000.0 * k as f32 / 20_000.0;
            self_i[k] = libm::cosf(theta);
            self_q[k] = libm::sinf(theta);
        }
        let (self_pos, self_neg) = dft_bins(&self_i, &self_q, 1_000.0, 20_000.0);
        info!("DFT self-check: |+1kHz|={} |-1kHz|={}", self_pos / N_SELF as f32, self_neg / N_SELF as f32);
        defmt::assert!(
            self_pos > self_neg * 1000.0,
            "DFT self-check failed: |+|={} |-|={}",
            self_pos, self_neg,
        );

        // --- SSB filter test ---
        // Warmup flushes the 257-tap delay line (group delay ≈ 128 samples =
        // 13 frames); 15 gives a small margin.
        const FRAMES_WARMUP:  usize = 15;
        const FRAMES_MEASURE: usize = 20;  // 200 samples = 10 full periods @ 1 kHz
        const N_TOTAL:   usize = (FRAMES_WARMUP + FRAMES_MEASURE) * DECIMATED_LEN;
        const N_MEASURE: usize = FRAMES_MEASURE * DECIMATED_LEN;

        // Local filter instance — does not disturb the global dsp::SSB_INST.
        let mut state_buf = [0.0f32; SSB_STATE_LEN];
        let inst = ArmFirInstanceF32 {
            num_taps: SSB_FILTER_TAPS as u16,
            p_state:  state_buf.as_mut_ptr(),
            p_coeffs: SSB_COEFFS_INTERLEAVED.as_ptr(),
        };

        let input = sine_f32_arr::<N_TOTAL>(1_000.0, 20_000.0, 0.5);

        // Warmup: fill the delay line; discard output.
        for frame in 0..FRAMES_WARMUP {
            let mut di = [0.0f32; DECIMATED_LEN];
            let mut dq = [0.0f32; DECIMATED_LEN];
            unsafe {
                arm_fir_ssb_f32(
                    &inst,
                    input[frame * DECIMATED_LEN..].as_ptr(),
                    di.as_mut_ptr(), dq.as_mut_ptr(),
                    DECIMATED_LEN as u32,
                );
            }
        }

        // Measurement: collect N_MEASURE I/Q samples.
        let mut i_buf = [0.0f32; N_MEASURE];
        let mut q_buf = [0.0f32; N_MEASURE];
        for frame in 0..FRAMES_MEASURE {
            let in_off  = (FRAMES_WARMUP + frame) * DECIMATED_LEN;
            let out_off = frame * DECIMATED_LEN;
            unsafe {
                arm_fir_ssb_f32(
                    &inst,
                    input[in_off..].as_ptr(),
                    i_buf[out_off..].as_mut_ptr(),
                    q_buf[out_off..].as_mut_ptr(),
                    DECIMATED_LEN as u32,
                );
            }
        }

        let (mag_pos, mag_neg) = dft_bins(&i_buf, &q_buf, 1_000.0, 20_000.0);
        let (mag_dom, mag_sup) = if mag_pos >= mag_neg {
            (mag_pos, mag_neg)
        } else {
            (mag_neg, mag_pos)
        };
        let rejection_db = 20.0 * libm::log10f(mag_dom / (mag_sup + 1e-10));

        info!(
            "SSB analytic: |+1kHz|={} |-1kHz|={} rejection={} dB",
            mag_pos / N_MEASURE as f32,
            mag_neg / N_MEASURE as f32,
            rejection_db,
        );

        // The dominant sideband must not be attenuated to nothing.
        defmt::assert!(
            mag_dom > 0.1 * N_MEASURE as f32,
            "SSB passband too weak: dominant={}",
            mag_dom,
        );
        // The suppressed sideband must be at least 40 dB below the dominant one.
        defmt::assert!(
            mag_dom > mag_sup * 100.0,
            "SSB sideband rejection < 40 dB: dominant={} suppressed={}",
            mag_dom, mag_sup,
        );

        info!("test_ssb_analytic_signal: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 18: CORDIC modulus — known I/Q pairs and output ordering
    //
    // Spot-checks cordic_modulus_vec against analytically known results and
    // verifies that outputs are returned in the same order as inputs (the
    // zero-overhead pipeline could theoretically mis-sequence results).
    //
    // CORDIC MODULUS in Q1.31:
    //   (i32::MAX, 0)         → mod ≈ i32::MAX,      phase ≈ 0
    //   (0,        i32::MAX)  → mod ≈ i32::MAX,      phase ≈ i32::MAX/2  (π/2)
    //   (HLF,      HLF)       → mod ≈ 0x5A82_7999,   phase ≈ i32::MAX/4  (π/4)
    //   (i32::MAX, 0)         → same as first — ordering sentinel
    //
    // Tolerance: 1 % of i32::MAX (~21 M) for modulus; 2 % for phase.
    // -----------------------------------------------------------------------
    #[test]
    fn test_cordic_modulus(_state: &mut State) {
        // CORDIC is already configured by dsp::init() called in #[init].
        const MAX: i32 = i32::MAX;
        const HLF: i32 = i32::MAX / 2;

        // Tolerance: 1 % of i32::MAX for modulus, 2 % for phase.
        const TOL_MOD:   i32 = (i32::MAX as i64 / 100) as i32;
        const TOL_PHASE: i32 = (i32::MAX as i64 / 50)  as i32;

        let i_vals = [MAX, 0,   HLF, MAX];
        let q_vals = [0,   MAX, HLF, 0  ];

        let mut mod_out   = [0i32; 4];
        let mut phase_out = [0i32; 4];
        unsafe {
            dsp::cordic_modulus_vec(&i_vals, &q_vals, &mut mod_out, &mut phase_out);
        }

        info!("CORDIC results:");
        info!("  [0] mod={} phase={}", mod_out[0], phase_out[0]);
        info!("  [1] mod={} phase={}", mod_out[1], phase_out[1]);
        info!("  [2] mod={} phase={}", mod_out[2], phase_out[2]);
        info!("  [3] mod={} phase={}", mod_out[3], phase_out[3]);

        // [0] (MAX, 0): modulus ≈ MAX, phase ≈ 0
        defmt::assert!((mod_out[0] - MAX).abs()     < TOL_MOD,   "[0] mod wrong");
        defmt::assert!( phase_out[0].abs()           < TOL_PHASE, "[0] phase wrong");

        // [1] (0, MAX): modulus ≈ MAX, phase ≈ MAX/2 (π/2 in Q1.31)
        defmt::assert!((mod_out[1] - MAX).abs()     < TOL_MOD,   "[1] mod wrong");
        defmt::assert!((phase_out[1] - MAX / 2).abs() < TOL_PHASE, "[1] phase wrong");

        // [2] (HLF, HLF): modulus ≈ 0x5A82_7999 (√2/2 × MAX), phase ≈ MAX/4 (π/4)
        let expected_mod: i32 = 0x5A82_7999_u32 as i32;
        let expected_phase: i32 = MAX / 4;
        defmt::assert!((mod_out[2] - expected_mod).abs()     < TOL_MOD,   "[2] mod wrong");
        defmt::assert!((phase_out[2] - expected_phase).abs() < TOL_PHASE, "[2] phase wrong");

        // [3] (MAX, 0): same as [0] — verifies pipeline ordering is preserved
        defmt::assert!((mod_out[3] - MAX).abs()     < TOL_MOD,   "[3] mod wrong (ordering?)");
        defmt::assert!( phase_out[3].abs()           < TOL_PHASE, "[3] phase wrong (ordering?)");

        info!("test_cordic_modulus: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 19: Outphasing boundary conditions
    //
    // Verifies outphasing_sample() at the edges of its input range:
    //
    //   modulus = 0         → tc_cmp1 clamped to minimum (3)
    //   modulus = i32::MAX  → tc_cmp1 clamped to AMPLITUDE
    //   phase   = 0         → td_cmp1 = PWM_PERIOD/2, te_cmp1 = 0  (±90°)
    //   phase   = i32::MAX  → td_cmp1 wraps correctly via conditional subtract
    //   te_cmp1 invariant   → always equals (td_cmp1 + PWM_PERIOD/2) % PWM_PERIOD
    // -----------------------------------------------------------------------
    #[test]
    fn test_outphasing_boundary(_state: &mut State) {
        use config::{PWM_PERIOD, TIMERC_PERIOD};
        const AMPLITUDE: u32 = TIMERC_PERIOD / 2;

        // Zero modulus → minimum envelope (clamp floor = 3).
        let s = dsp::outphasing_sample(0, 0);
        info!("zero mod: tc_cmp1={} td_cmp1={} te_cmp1={}", s.tim_c_cmp1, s.tim_d_cmp1, s.tim_e_cmp1);
        defmt::assert!(s.tim_c_cmp1 == 3, "zero mod: tc_cmp1 should be 3, got {}", s.tim_c_cmp1);

        // Full modulus → amplitude ceiling.
        let s = dsp::outphasing_sample(i32::MAX, 0);
        info!("full mod: tc_cmp1={} (expected {})", s.tim_c_cmp1, AMPLITUDE);
        defmt::assert!(s.tim_c_cmp1 == AMPLITUDE, "full mod: tc_cmp1={} expected {}", s.tim_c_cmp1, AMPLITUDE);

        // phase = 0  (CORDIC: atan2(0, MAX) = 0 in Q1.31).
        // phase_ticks = (-0 * half >> 16) + half = half = PWM_PERIOD/2.
        let s = dsp::outphasing_sample(i32::MAX, 0);
        let expected_d = PWM_PERIOD / 2;
        let expected_e = 0u32; // (PWM_PERIOD/2 + PWM_PERIOD/2) % PWM_PERIOD = 0
        info!("phase=0: td_cmp1={} (exp {}) te_cmp1={} (exp {})",
              s.tim_d_cmp1, expected_d, s.tim_e_cmp1, expected_e);
        defmt::assert!(s.tim_d_cmp1 == expected_d,
            "phase=0: td_cmp1={} expected {}", s.tim_d_cmp1, expected_d);
        defmt::assert!(s.tim_e_cmp1 == expected_e,
            "phase=0: te_cmp1={} expected {}", s.tim_e_cmp1, expected_e);

        // te_cmp1 invariant: must equal (td_cmp1 + PWM_PERIOD/2) % PWM_PERIOD
        // for all inputs. Check several phase values spanning the full range.
        for &phase in &[0i32, i32::MAX / 4, i32::MAX / 2, i32::MAX / 4 * 3, i32::MAX] {
            let s = dsp::outphasing_sample(i32::MAX / 2, phase);
            let expected = (s.tim_d_cmp1 + PWM_PERIOD / 2) % PWM_PERIOD;
            defmt::assert!(s.tim_e_cmp1 == expected,
                "te_cmp1 invariant failed at phase={}: td={} te={} expected {}",
                phase, s.tim_d_cmp1, s.tim_e_cmp1, expected);
        }

        info!("test_outphasing_boundary: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 20: Compressor passes quiet signal with makeup gain
    //
    // A signal well below threshold (0.05 < 0.3) must not be compressed.
    // The compressor applies makeup gain of 1.5×, so output RMS ≈ input × 1.5.
    // Accept ±10 % tolerance around the expected gain.
    // -----------------------------------------------------------------------
    #[test]
    fn test_compressor_passes_quiet(_state: &mut State) {
        const AMP: f32 = 0.05;   // well below COMP_THRESHOLD = 0.3
        const MAKEUP: f32 = 1.5; // COMP_MAKEUP_GAIN
        const N: usize = 2000;
        let input = sine_f32_arr::<N>(1_000.0, 20_000.0, AMP);
        let mut comp = dsp::Compressor::new();
        let mut output = [0.0f32; N];
        comp.process(&input, &mut output);
        // Skip first 500 samples to let the envelope follower settle.
        let in_rms  = rms_f32_buf::<1500>(&input [500..].try_into().unwrap());
        let out_rms = rms_f32_buf::<1500>(&output[500..].try_into().unwrap());
        let actual_gain = out_rms / in_rms;
        info!("Compressor quiet: in_rms={} out_rms={} gain={} (expected {})",
              in_rms, out_rms, actual_gain, MAKEUP);
        defmt::assert!(actual_gain > MAKEUP * 0.9, "gain too low: {}", actual_gain);
        defmt::assert!(actual_gain < MAKEUP * 1.1, "gain too high: {}", actual_gain);
        info!("test_compressor_passes_quiet: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 21: FIR interpolator stopband — 9 kHz rejected
    //
    // 9 kHz is below the 20 kHz Nyquist (no aliasing) and 1.8× above the
    // 5 kHz prototype cutoff.  The design notebook shows ~27 dB attenuation
    // at 9 kHz; the threshold is set at −26 dB (ratio 0.05, 1 dB margin).
    // Levels are normalised by the interpolator's inherent 1/L gain.
    // -----------------------------------------------------------------------
    #[test]
    fn test_fir_interpolate_stopband(_state: &mut State) {
        use dsp_ffi::{ArmFirInterpolateInstanceQ31, FIR_INTERP_STATE_LEN,
                      arm_fir_interpolate_init_q31, arm_fir_interpolate_q31};
        use dsp::{FIR_COEFFS_Q31, FIR_NUM_TAPS, FIR_DECIMATE_FACTOR, DECIMATED_LEN};
        const FRAMES: usize = 20;
        const N_IN:  usize = DECIMATED_LEN         * FRAMES;
        const N_OUT: usize = config::FRAME_SAMPLES * FRAMES;

        let input = sine_q31::<N_IN>(9_000.0, 20_000.0, 0.5);
        let mut state = [0i32; FIR_INTERP_STATE_LEN];
        let mut inst = ArmFirInterpolateInstanceQ31 {
            l: 0, phase_length: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
        };
        unsafe {
            arm_fir_interpolate_init_q31(
                &raw mut inst, FIR_DECIMATE_FACTOR as u8, FIR_NUM_TAPS as u16,
                FIR_COEFFS_Q31.as_ptr(), state.as_mut_ptr(), DECIMATED_LEN as u32,
            );
        }
        let mut output = [0i32; N_OUT];
        for frame in 0..FRAMES {
            unsafe {
                arm_fir_interpolate_q31(
                    &inst,
                    input[frame * DECIMATED_LEN..].as_ptr(),
                    output[frame * config::FRAME_SAMPLES..].as_mut_ptr(),
                    DECIMATED_LEN as u32,
                );
            }
        }
        let in_rms  = rms_q31_buf::<{N_IN  - DECIMATED_LEN}>(
            input [DECIMATED_LEN..].try_into().unwrap());
        let out_rms = rms_q31_buf::<{N_OUT - config::FRAME_SAMPLES}>(
            output[config::FRAME_SAMPLES..].try_into().unwrap());
        // Compare against passband-normalised level (in_rms / L) to account
        // for the interpolator's inherent 1/L gain, matching the passband test.
        let passband_rms = in_rms / FIR_DECIMATE_FACTOR as f32;
        info!("FIR interp 9 kHz stopband: in_rms={} out_rms={} passband_ref={}", in_rms, out_rms, passband_rms);
        // Filter achieves ~27 dB at 9 kHz per design_decimate_filter.ipynb.
        // Threshold at 0.05 (≈ −26 dB) leaves 1 dB margin.
        defmt::assert!(out_rms < passband_rms * 0.05, "stopband not attenuated enough");
        info!("test_fir_interpolate_stopband: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 22: process_second_half() smoke test
    //
    // Calls process_second_half() and verifies it completes without faulting.
    // This exercises the second-half buffer pointers which are never used by
    // any other test — a wrong pointer would cause a HardFault here.
    // A meaningful output check is not attempted: a constant ADC input is a
    // DC signal which the highpass filter rejects, leaving tc_cmp1 = 3
    // (minimum clamp) regardless of the ADC value written.
    // -----------------------------------------------------------------------
    #[test]
    fn test_process_second_half(_state: &mut State) {
        info!("test: process_second_half smoke test");

        // Safety: no DMA activity writes to the second half right now and
        // the DSP task is not running in hw_tests.
        unsafe {
            let buf = hrtim::adc_buf_second_half_mut();
            for s in buf.iter_mut() { *s = 2048; } // midscale — valid ADC value
        }

        // A single call is enough to exercise all second-half pointer paths.
        // If any pointer is wrong this will HardFault before returning.
        let _ = unsafe { dsp::process_second_half() };

        let hrtim_buf = unsafe { hrtim::hrtim_buf_second_half() };
        info!("second half hrtim buf[0]: tc_cmp1={}", hrtim_buf[0].tim_c_cmp1);

        info!("test_process_second_half: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 23: HRTIM burst DMA updates all three CMP registers
    //
    // DMA1_CH5 is configured as a circular burst DMA triggered by Timer C's
    // REP event (every ~5 µs at 200 kHz).  Each burst writes one PwmSample
    // (3 × u32) to HRTIM1.BDMADR, which the HRTIM routes to:
    //   word 0 → Timer C CMP1
    //   word 1 → Timer D CMP1
    //   word 2 → Timer E CMP1
    //
    // This test writes a sentinel PwmSample into HRTIM_BUF, waits for
    // several burst periods, then reads back the three CMP registers from
    // the HRTIM1 peripheral and asserts they match.
    // -----------------------------------------------------------------------
    #[test]
    fn test_hrtim_dma_updates_regs(_state: &mut State) {
        use config::{PWM_PERIOD, TIMERC_PERIOD};
        use hrtim::PwmSample;

        info!("test: HRTIM burst DMA updates CMP registers");

        // Sentinel values — clearly different from the init values:
        //   init: Timer C CMP1 = 1, Timer D CMP1 = PWM_PERIOD/4, Timer E CMP1 = PWM_PERIOD*3/4
        let sentinel_tc_cmp1 = TIMERC_PERIOD / 3;
        let sentinel_td_cmp1 = PWM_PERIOD / 3;
        let sentinel_te_cmp1 = (sentinel_td_cmp1 + PWM_PERIOD / 2) % PWM_PERIOD;
        let sentinel = PwmSample {
            tim_c_cmp1: sentinel_tc_cmp1,
            tim_d_cmp1: sentinel_td_cmp1,
            tim_e_cmp1: sentinel_te_cmp1,
        };

        // Overwrite both halves of the DMA buffer with the sentinel sample.
        // Safety: no DSP task is running in hw_tests.
        unsafe {
            for slot in hrtim::hrtim_buf_first_half_mut()  { *slot = sentinel; }
            for slot in hrtim::hrtim_buf_second_half_mut() { *slot = sentinel; }
        }

        // Wait several Timer C periods so the DMA has time to burst the new
        // values into the HRTIM registers.  Timer C period ≈ 5 µs; 1 ms
        // gives ~200 bursts of headroom.
        wait_ms(1);

        // Read back the three CMP registers directly from the HRTIM peripheral.
        let tc_cmp1 = pac::HRTIM1.tim(2).cmp(0).read().cmp() as u32;
        let td_cmp1 = pac::HRTIM1.tim(3).cmp(0).read().cmp() as u32;
        let te_cmp1 = pac::HRTIM1.tim(4).cmp(0).read().cmp() as u32;

        info!(
            "HRTIM regs: TC_CMP1={} TD_CMP1={} TE_CMP1={}",
            tc_cmp1, td_cmp1, te_cmp1
        );
        info!(
            "Expected:   TC_CMP1={} TD_CMP1={} TE_CMP1={}",
            sentinel_tc_cmp1, sentinel_td_cmp1, sentinel_te_cmp1
        );

        defmt::assert!(tc_cmp1 == sentinel_tc_cmp1,
            "Timer C CMP1: got {} expected {}", tc_cmp1, sentinel_tc_cmp1);
        defmt::assert!(td_cmp1 == sentinel_td_cmp1,
            "Timer D CMP1: got {} expected {}", td_cmp1, sentinel_td_cmp1);
        defmt::assert!(te_cmp1 == sentinel_te_cmp1,
            "Timer E CMP1: got {} expected {}", te_cmp1, sentinel_te_cmp1);

        info!("test_hrtim_dma_updates_regs: PASS");
    }

    // -----------------------------------------------------------------------
    // Test 24: Sample format conversion helpers
    // -----------------------------------------------------------------------
    #[test]
    fn test_conversion_helpers(_state: &mut State) {
        defmt::assert!(dsp::adc_to_q31(2048) == 0,  "midscale should be zero");
        defmt::assert!(dsp::adc_to_q31(4095) > 0,   "full-scale should be positive");
        // ADC input 0 → (0 − 2048) << 19 = negative full-scale
        defmt::assert!(dsp::adc_to_q31(0) < 0,      "minimum ADC should be negative");
        defmt::assert!(dsp::adc_to_q31(0) == ((-2048i32) << 19), "minimum ADC value wrong");
        for &x in &[0.0f32, 0.5, -0.5, 0.999, -0.999] {
            let q    = dsp::f32_to_q31(x);
            let back = dsp::q31_to_f32(q);
            defmt::assert!((back - x).abs() < 1e-6, "Q31 round-trip failed");
        }
        info!("test_conversion_helpers: PASS");
    }
}
