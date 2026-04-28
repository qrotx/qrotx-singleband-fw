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
// States (each held for HOLD_MS / PHASE_HOLD_MS milliseconds):
//   1  Nominal: ta_cmp1 = PWM_PERIOD/2, tc_cmp1 = AMPLITUDE        (HOLD_MS)
//   2  Low amplitude: ta_cmp1 = PWM_PERIOD/2, tc_cmp1 = AMPLITUDE/4 (HOLD_MS)
//   3  Full amplitude: ta_cmp1 = PWM_PERIOD/2, tc_cmp1 = AMPLITUDE  (HOLD_MS)
//   4  Phase A: ta_cmp1 = 0                                          (HOLD_MS)
//   5  Phase B: ta_cmp1 = PWM_PERIOD/4                               (HOLD_MS)
//
// After state 5, Timer C is reconfigured as a 50 % phase reference driven by
// the master timer (SET = master period, RESET = master CMP1).  CH3/CH4 then
// run at the RF frequency and serve as a fixed-phase reference for the phase
// modulation sweep in states 6–13.
//
// Phase sweep states (held for PHASE_HOLD_MS):
//   6  td_cmp1 = 0               — critical: Timer D CMP register = 0
//   7  td_cmp1 = 1               — just above 0 for comparison
//   8  td_cmp1 = PWM_PERIOD/4-1  — just below 180° phase boundary
//   9  td_cmp1 = PWM_PERIOD/4    — at 180° phase boundary
//  10  td_cmp1 = PWM_PERIOD/4+1  — just above 180° phase boundary
//  11  td_cmp1 = PWM_PERIOD/2-1  — just below critical (te_cmp1 wraps near 0)
//  12  td_cmp1 = PWM_PERIOD/2    — critical: Timer E CMP register = 0 (te_cmp1 = 0)
//  13  td_cmp1 = PWM_PERIOD/2+1  — just above critical
//
// Probe connections (documented here and in scope_tests.py):
//   CH1 → PA8  (Timer A output 1 — phase-modulated RF, positive)
//   CH2 → PA9  (Timer A output 2 — dead-time complement of PA8)
//   CH3 → PB12 (Timer C output 1 — buck converter, then phase reference in states 6-13)
//   CH4 → PB13 (Timer C output 2 — complement of CH3)
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
// Budget for State 1 (the largest): frequency measurements (~14 s) +
// dead-time verification (~12 s) + step-mode pauses (user-dependent).
// States 2–5 need ~10 s each (one duty/phase measurement + timebase settle).
// Increase this if measurements time out or if you need more time to inspect
// the scope screen during --step runs.
const HOLD_MS: u32 = 45_000;

// Hold time for phase-sweep states 6–13.  Each state requires one timebase
// setup (~5 s, done once before the sweep) plus one RPHase measurement (~2 s).
const PHASE_HOLD_MS: u32 = 20_000;

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

/// Reconfigure Timer C as a 50 % phase-reference square wave locked to the master timer.
///
/// After this call:
///   TC1 (PB12): SET on master period event (phase = 0°)
///               RESET on master CMP1 event (at PWM_PERIOD/2 = 180°)
///   TC2 (PB13): complement — SET on master CMP1, RESET on master period
///
/// Dead-time is disabled so both outputs are independently controlled.
/// Timer C's own counter keeps running; only the output crossbar is changed.
/// The master events (MSTPER / MSTCMP1) operate independently of Timer C's
/// prescaler, so TC1/TC2 switch at precisely the master timer edges.
unsafe fn configure_timer_c_as_phase_ref() {
    use embassy_stm32::pac;
    let hrtim = pac::HRTIM1;
    // RM0440 §28.3.4 Deadtime caution: DTEN must not be changed while the
    // timer is enabled (TxCEN set) or while outputs are active.  Both must
    // be disabled first, or behavior is unpredictable.
    hrtim.odisr().modify(|w| {
        w.set_t1odis(2, true); // disable TC1 (PB12)
        w.set_t2odis(2, true); // disable TC2 (PB13)
    });
    hrtim.mcr().modify(|w| w.set_tcen(2, false)); // stop Timer C counter
    // Now safe to clear DTEN.
    hrtim.tim(2).outr().modify(|w| w.set_dten(false));
    // TC1: rising edge at master period, falling edge at master CMP1 (50 % duty).
    hrtim.tim(2).setr(0).write(|w| w.set_mstper(true));
    hrtim.tim(2).rstr(0).write(|w| w.set_mstcmp(0, true));
    // TC2: complement — rising edge at master CMP1, falling edge at master period.
    hrtim.tim(2).setr(1).write(|w| w.set_mstcmpx(0, true));
    hrtim.tim(2).rstr(1).write(|w| w.set_mstper(true));
    // Re-enable timer and outputs.  TC1/TC2 will reach their correct state at
    // the next master period / master CMP1 event (~143 ns after restart).
    hrtim.mcr().modify(|w| w.set_tcen(2, true));
    hrtim.oenr().modify(|w| {
        w.set_t1oen(2, true); // TC1
        w.set_t2oen(2, true); // TC2
    });
}

/// Build a PwmSample from td_cmp1 and tc_cmp1; derive te_cmp1 = td_cmp1 + PWM_PERIOD/2.
fn make_sample(td_cmp1: u32, tc_cmp1: u32) -> PwmSample {
    let t = td_cmp1 + PWM_PERIOD / 2;
    let te_cmp1 = if t >= PWM_PERIOD { t - PWM_PERIOD } else { t };
    PwmSample {
        tim_c_cmp1: tc_cmp1,
        tim_d_cmp1: td_cmp1,
        tim_e_cmp1: te_cmp1,
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
    // Phase modulation sweep (States 6–13)
    //
    // Reconfigure Timer C as a 50 % reference locked to the master timer.
    // TC1 (CH3) is then a stable phase reference at the RF frequency.
    //
    // Timer A's phase relative to the reference is:
    //   phase = -(td_cmp1 + PWM_PERIOD/4) / PWM_PERIOD × 360°
    // because TA1 rises at (td_cmp1 + PWM_PERIOD/4) ticks after the master
    // period event that also sets TC1 high.
    //
    // Critical cases under test:
    //   td_cmp1 = 0              — Timer D CMP register = 0 (may not fire)
    //   td_cmp1 = PWM_PERIOD/2   — te_cmp1 wraps to 0 (Timer E CMP = 0)
    // -------------------------------------------------------------------
    unsafe { configure_timer_c_as_phase_ref(); }
    spin_ms(50); // allow TC1/TC2 to settle to new crossbar sources

    macro_rules! phase_state {
        ($n:expr, $td:expr) => {{
            unsafe { fill_hrtim_buf(make_sample($td, AMPLITUDE)); }
            spin_ms(50);
            info!("SCOPE STATE {} READY td_cmp1={}", $n, $td);
            spin_ms(PHASE_HOLD_MS);
        }};
    }

    // -------------------------------------------------------------------
    // States 6–7: near td_cmp1 = 0 (Timer D CMP register = 0)
    // Expected phase: -(PWM_PERIOD/4) / PWM_PERIOD × 360° = −90°
    // -------------------------------------------------------------------
    phase_state!(6, 0);
    phase_state!(7, 1);

    // -------------------------------------------------------------------
    // States 8–10: around 180° phase boundary (td_cmp1 = PWM_PERIOD/4)
    // Expected phase: -(PWM_PERIOD/2) / PWM_PERIOD × 360° = −180° / +180°
    // -------------------------------------------------------------------
    phase_state!(8,  PWM_PERIOD / 4 - 1);
    phase_state!(9,  PWM_PERIOD / 4);
    phase_state!(10, PWM_PERIOD / 4 + 1);

    // -------------------------------------------------------------------
    // States 11–13: near td_cmp1 = PWM_PERIOD/2 (te_cmp1 wraps to 0)
    // Expected phase: -(3×PWM_PERIOD/4) / PWM_PERIOD × 360° = −270° = +90°
    // -------------------------------------------------------------------
    phase_state!(11, PWM_PERIOD / 2 - 1);
    phase_state!(12, PWM_PERIOD / 2);
    phase_state!(13, PWM_PERIOD / 2 + 1);

    // -------------------------------------------------------------------
    hrtim::disable_outputs();
    info!("SCOPE DONE");

    // Signal probe-rs to exit cleanly.
    cortex_m::asm::bkpt();
    loop {}
}
