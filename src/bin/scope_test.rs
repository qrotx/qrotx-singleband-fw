// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// scope_test — Standalone binary for oscilloscope-based PWM output verification.
//
// Cycles through test states, printing RTT markers when each state is stable.
// The host-side tests/scope_tests.py script coordinates scope measurements.
//
// Build:  cargo build --bin scope_test --release
// Flash:  run via scope_tests.py (it builds and flashes automatically)
//
// States (each held for HOLD_MS milliseconds):
//   1  Nominal: ta_cmp1 = PWM_PERIOD/2, tc_cmp1 = AMPLITUDE
//   2  Low amplitude: ta_cmp1 = PWM_PERIOD/2, tc_cmp1 = AMPLITUDE/4
//   3  Full amplitude: ta_cmp1 = PWM_PERIOD/2, tc_cmp1 = AMPLITUDE
//   4  Phase A: ta_cmp1 = 0
//   5  Phase B: ta_cmp1 = PWM_PERIOD/4
//
// Probe connections (documented here and in scope_tests.py):
//   CH1 → PA8  (Timer A output 1 — phase-modulated RF, positive)
//   CH2 → PA9  (Timer A output 2 — dead-time complement of PA8)
//   CH3 → PB12 (Timer C output 1 — buck converter / amplitude envelope)
//   CH4 → PB13 (Timer C output 2 — dead-time complement of PB12)
//   GND → any board GND pin

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk,
};
use embassy_stm32::Config;

#[path = "../config.rs"]  mod config;
#[path = "../hrtim.rs"]   mod hrtim;
#[path = "../dma.rs"]     mod dma;

use config::{PWM_PERIOD, TIMERC_PERIOD, FRAME_SAMPLES};
use hrtim::PwmSample;

const AMPLITUDE: u32 = TIMERC_PERIOD / 2;

// Time the scope has to acquire and measure each state (milliseconds).
// Budget: scope *RST (~2.5 s) + settle (1.5 s) + measurements (~4 s)
// plus user inspection time when running with --step.
// Increase this if measurements time out or if you need more time to inspect
// the scope screen; decrease it (minimum ~10 s) for unattended runs.
const HOLD_MS: u32 = 30_000;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn spin_ms(ms: u32) {
    // cortex_m::asm::delay(n) loops for n iterations.  Empirically measured at
    // ~2 cycles/iteration on this board (Cortex-M4 @ 168 MHz with flash cache),
    // giving iterations_per_ms = 168_000_000 / (1000 × 2) = 84_000.
    // Measured correction: observed time was 0.5× expected with 56_000, so
    // the effective value is 112_000 (= 56_000 × 2) to match real time.
    // Chunk to keep the u32 argument within range:
    //   10_000 ms × 112_000 = 1_120_000_000 < 2^32.
    // Guard rem > 0: asm::delay(0) wraps to 2^32 iterations.
    const CHUNK_MS: u32 = 10_000;
    const ITERS_PER_MS: u32 = 112_000;
    let mut rem = ms;
    while rem >= CHUNK_MS {
        cortex_m::asm::delay(CHUNK_MS * ITERS_PER_MS);
        rem -= CHUNK_MS;
    }
    if rem > 0 {
        cortex_m::asm::delay(rem * ITERS_PER_MS);
    }
}

/// Fill both halves of the HRTIM DMA buffer with the same PwmSample.
/// The DMA continuously plays the buffer in circular mode, so writing here
/// takes effect within one PWM period (~5 µs).
unsafe fn fill_hrtim_buf(s: PwmSample) {
    for slot in hrtim::hrtim_buf_first_half_mut()  { *slot = s; }
    for slot in hrtim::hrtim_buf_second_half_mut() { *slot = s; }
}

/// Build a PwmSample from ta_cmp1 and tc_cmp1; derive ta_cmp2 = ta_cmp1 + half.
fn make_sample(ta_cmp1: u32, tc_cmp1: u32) -> PwmSample {
    let t = ta_cmp1 + PWM_PERIOD / 2;
    let ta_cmp2 = if t >= PWM_PERIOD { t - PWM_PERIOD } else { t };
    PwmSample {
        tim_a_cmp1: ta_cmp1,
        tim_a_cmp2: ta_cmp2,
        tim_b_cmp1: ta_cmp1,
        tim_b_cmp2: ta_cmp2,
        tim_c_cmp1: tc_cmp1,
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[cortex_m_rt::entry]
fn main() -> ! {
    // 168 MHz via HSI PLL — same as hw_tests and main firmware idle clock.
    let mut rcc = Config::default();
    {
        let r = &mut rcc.rcc;
        r.hsi     = true;
        r.pll     = Some(Pll {
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
    let _p = embassy_stm32::init(rcc);

    hrtim::init();
    dma::init();
    // ADC is deliberately NOT started — scope_test only needs HRTIM outputs.

    // Send compile-time constants so the Python script can compute expected
    // values without hard-coding them.
    info!(
        "SCOPE CONSTANTS PWM_PERIOD={} TIMERC_PERIOD={} AMPLITUDE={} FRAME_SAMPLES={}",
        PWM_PERIOD, TIMERC_PERIOD, AMPLITUDE, FRAME_SAMPLES
    );
    spin_ms(200);

    // -------------------------------------------------------------------
    // State 1 — Nominal: mid-phase, full amplitude
    // Scope test: frequency on all 4 channels; Vpp on all channels.
    // -------------------------------------------------------------------
    unsafe { fill_hrtim_buf(make_sample(PWM_PERIOD / 2, AMPLITUDE)); }
    hrtim::enable_outputs();
    spin_ms(200); // outputs settle within one PWM period; 200 ms is generous
    info!("SCOPE STATE 1 READY ta_cmp1={} tc_cmp1={}", PWM_PERIOD / 2, AMPLITUDE);
    spin_ms(HOLD_MS);

    // -------------------------------------------------------------------
    // State 2 — Low amplitude: tc_cmp1 = AMPLITUDE/4
    // Scope test: PB12 duty cycle ≈ AMPLITUDE/4 / TIMERC_PERIOD × 100 %.
    // -------------------------------------------------------------------
    unsafe { fill_hrtim_buf(make_sample(PWM_PERIOD / 2, AMPLITUDE / 4)); }
    spin_ms(50);
    info!("SCOPE STATE 2 READY ta_cmp1={} tc_cmp1={}", PWM_PERIOD / 2, AMPLITUDE / 4);
    spin_ms(HOLD_MS);

    // -------------------------------------------------------------------
    // State 3 — Full amplitude: tc_cmp1 = AMPLITUDE
    // Scope test: PB12 duty cycle ≈ AMPLITUDE / TIMERC_PERIOD × 100 %.
    // -------------------------------------------------------------------
    unsafe { fill_hrtim_buf(make_sample(PWM_PERIOD / 2, AMPLITUDE)); }
    spin_ms(50);
    info!("SCOPE STATE 3 READY ta_cmp1={} tc_cmp1={}", PWM_PERIOD / 2, AMPLITUDE);
    spin_ms(HOLD_MS);

    // -------------------------------------------------------------------
    // State 4 — Phase A: ta_cmp1 = 0
    // Scope test: record PA8 duty cycle as phase reference.
    // -------------------------------------------------------------------
    unsafe { fill_hrtim_buf(make_sample(0, AMPLITUDE)); }
    spin_ms(50);
    info!("SCOPE STATE 4 READY ta_cmp1={} tc_cmp1={}", 0u32, AMPLITUDE);
    spin_ms(HOLD_MS);

    // -------------------------------------------------------------------
    // State 5 — Phase B: ta_cmp1 = PWM_PERIOD/4
    // Scope test: PA8 phase shifts by 90° relative to State 4.
    // -------------------------------------------------------------------
    unsafe { fill_hrtim_buf(make_sample(PWM_PERIOD / 4, AMPLITUDE)); }
    spin_ms(50);
    info!("SCOPE STATE 5 READY ta_cmp1={} tc_cmp1={}", PWM_PERIOD / 4, AMPLITUDE);
    spin_ms(HOLD_MS);

    // -------------------------------------------------------------------
    hrtim::disable_outputs();
    info!("SCOPE DONE");

    // Signal probe-rs to exit cleanly.
    cortex_m::asm::bkpt();
    loop {}
}
