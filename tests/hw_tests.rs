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

#![no_std]
#![no_main]

use core::sync::atomic::Ordering;

use defmt::info;
use defmt_rtt as _;
use embassy_stm32::dma as embassy_dma;
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
#[path = "../src/config.rs"]  mod config;
#[path = "../src/hrtim.rs"]   mod hrtim;
#[path = "../src/adc.rs"]     mod adc;
#[path = "../src/dma.rs"]     mod dma;
#[path = "../src/dsp.rs"]     mod dsp;
#[path = "../src/si5351.rs"]  mod si5351;

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
        let cnt0 = pac::HRTIM1.tim(2).cnt().read().cnt();
        spin(10_000); // ~60 µs at 168 MHz → ~12 Timer-C periods
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
    // One call to process_first_half() must complete within the 500 µs
    // half-buffer period.  At 168 MHz that is 84 000 cycles.
    // -----------------------------------------------------------------------
    #[test]
    fn test_dsp_timing(_state: &mut State) {
        info!("test: DSP process_half timing");

        // Ensure DWT CYCCNT is running.
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            demcr.write_volatile(demcr.read_volatile() | (1 << 24)); // TRCENA
            let ctrl = 0xE000_1000 as *mut u32;
            ctrl.write_volatile(ctrl.read_volatile() | 1);           // CYCCNTENA
        }

        let start = cortex_m::peripheral::DWT::cycle_count();
        unsafe { dsp::process_first_half() };
        let elapsed = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start);

        info!("process_half: {} cycles ({} µs)", elapsed, elapsed / 168);
        defmt::assert!(
            elapsed < 84_000,
            "process_half too slow: {} cycles (budget: 84 000 = 500 µs @ 168 MHz)",
            elapsed
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
        unsafe {
            let demcr = 0xE000_EDFC as *mut u32;
            demcr.write_volatile(demcr.read_volatile() | (1 << 24)); // TRCENA
            (0xE000_1004 as *mut u32).write_volatile(0);             // reset CYCCNT
            let ctrl = 0xE000_1000 as *mut u32;
            ctrl.write_volatile(ctrl.read_volatile() | 1);           // CYCCNTENA
        }

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
}
