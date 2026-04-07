// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// hrtim.rs — HRTIM1 initialisation and DMA buffer management.
//
// Hardware topology (from CubeMX / .ioc):
//
//   Master timer  period = PWM_PERIOD          CMP1 = PWM_PERIOD/2
//   Timer A       period = PWM_PERIOD          PA8 (CHA1), PA9 (CHA2)
//                 CMP1   = PWM_PERIOD/4        ← outphasing branch 1
//                 CMP2   = PWM_PERIOD*3/4
//                 Dead-time enabled (3 ticks rising+falling, negative sign)
//                 BurstDMA: [CMP1, CMP2]  ← phase angle updated by DMA
//   Timer B       period = PWM_PERIOD          (no output, sync helper for TB→TA cross-coupling)
//                 CMP1   = PWM_PERIOD/4
//                 CMP2   = PWM_PERIOD*3/4
//                 BurstDMA: [CMP1, CMP2]
//   Timer C       period = TIMERC_PERIOD(850)  PB12 (CHC1), PB13 (CHC2)
//                 Prescaler = DIV1             → ~200 kHz (ADC-trigger source)
//                 CMP1   = 200 (initial)
//                 DMA: DMA1_CH5, memory→periph, word, circular
//                 DMA request: REP (repetition event every period)
//                 → continuously loads new CMP1 values at 200 kHz rate
//   Timer D       period = PWM_PERIOD          PB14 (CHD1), PB15 (CHD2)
//                 CMP1   = PWM_PERIOD/8        (gate / driver-enable shape)
//
// DMA buffers
// -----------
// Two circular double-buffers are allocated in CCMRAM (tightly-coupled RAM
// for lowest-latency DMA access):
//
//   ADC_BUF  [2 × FRAME_SAMPLES] u16  — audio input, DMA1_CH1
//   HRTIM_BUF[2 × FRAME_SAMPLES] u32  — HRTIM Timer-C CMP1 updates, DMA1_CH5
//
// Each "half" of the buffer is one 0.5 ms frame (FRAME_SAMPLES = 100 samples).
// The DMA half-transfer interrupt fires at the midpoint; the ISR processes the
// completed half while DMA fills the other half ("ping-pong").
//
// IMPORTANT: full HRTIM register initialisation is NOT yet implemented.
// The register-level setup (HRTIM1_MASTER, HRTIM1_TIMxCR, dead-time, output
// polarity, DMA burst addresses, …) must be written in a subsequent sprint,
// either via the PAC or by including the STM32G4 HAL C library.  The stubs
// here establish the correct data structures and DMA buffer layout.

use core::sync::atomic::AtomicBool;
use defmt::{debug, info};

use crate::config::{FRAME_SAMPLES, PWM_PERIOD};

// ---------------------------------------------------------------------------
// DMA buffers — placed in CCMRAM for deterministic DMA latency
// ---------------------------------------------------------------------------

/// Total DMA buffer length = 2 × frame (double-buffer / ping-pong).
const DMA_BUF_LEN: usize = FRAME_SAMPLES * 2;

/// Audio ADC samples (u16, 12-bit right-aligned).
/// DMA1_CH1: ADC1 → this buffer, circular, halfword.
#[link_section = ".ccmram"]
static mut ADC_BUF: [u16; DMA_BUF_LEN] = [0u16; DMA_BUF_LEN];

/// HRTIM Timer-C CMP1 update values (u32).
/// DMA1_CH5: this buffer → HRTIM CMP1 register, circular, word.
/// Filled by the DSP pipeline; each word is loaded into Timer-C CMP1 at
/// the next Timer-C repetition event (every ~5 µs at 200 kHz).
#[link_section = ".ccmram"]
// Initial value is 0 (NOLOAD section — not copied from Flash at startup).
// hrtim::init() must fill the buffer with a safe default (TIMERC_PERIOD/2)
// before the DMA is started.
static mut HRTIM_BUF: [u32; DMA_BUF_LEN] = [0u32; DMA_BUF_LEN];

// ---------------------------------------------------------------------------
// Flags set in DMA half/full-transfer ISRs to signal the DSP task
// ---------------------------------------------------------------------------

/// True while the DSP task is working on the first half of the buffer.
pub static PROCESS_FIRST_HALF: AtomicBool = AtomicBool::new(false);
/// True while the DSP task is working on the second half of the buffer.
pub static PROCESS_SECOND_HALF: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// Buffer access — safe wrappers around the static muts
// ---------------------------------------------------------------------------

/// Borrow the first half of the ADC double-buffer (samples 0..FRAME_SAMPLES).
///
/// # Safety
/// Must only be called from the context that owns the first half (i.e. after
/// a half-transfer event and before the next full-transfer event).
pub unsafe fn adc_buf_first_half() -> &'static [u16] {
    &ADC_BUF[..FRAME_SAMPLES]
}

/// Borrow the second half of the ADC double-buffer.
pub unsafe fn adc_buf_second_half() -> &'static [u16] {
    &ADC_BUF[FRAME_SAMPLES..]
}

/// Borrow the first half of the HRTIM output double-buffer (mutable).
pub unsafe fn hrtim_buf_first_half_mut() -> &'static mut [u32] {
    &mut HRTIM_BUF[..FRAME_SAMPLES]
}

/// Borrow the second half of the HRTIM output double-buffer (mutable).
pub unsafe fn hrtim_buf_second_half_mut() -> &'static mut [u32] {
    &mut HRTIM_BUF[FRAME_SAMPLES..]
}

// ---------------------------------------------------------------------------
// HRTIM initialisation stubs
//
// These functions document the intended configuration.  Full implementation
// requires PAC-level register writes (or the STM32G4 HAL).
// ---------------------------------------------------------------------------

/// Initialise the HRTIM peripheral.
///
/// Must be called after the system clock is stable (either internal 168 MHz
/// or external-derived ~169.5 MHz).
///
/// Steps (to be implemented):
///   1. Enable HRTIM1 peripheral clock (RCC).
///   2. Start the HRTIM DLL calibration and wait for lock.
///   3. Configure Master timer: period=PWM_PERIOD, CMP1=PWM_PERIOD/2.
///   4. Configure Timer A: period, CMP1/2 initial values, dead-time.
///   5. Configure Timer B: period, CMP1/2 (sync source for A).
///   6. Configure Timer C: prescaler=DIV1, period=TIMERC_PERIOD, CMP1=200,
///      preload enable, repetition counter=0, REP-DMA enable.
///   7. Configure Timer D: period, CMP1=PWM_PERIOD/8.
///   8. Configure DMA1_CH5 for Timer-C CMP1 burst at 200 kHz.
///   9. Configure DMA1_CH1 for ADC1.
///  10. Set all outputs to inactive (MOSFETs off).
///  11. Start master + sub-timers (outputs still disabled).
pub fn init() {
    // TODO: PAC register writes for steps 1–11.
    //
    // Example skeleton (using embassy_stm32::pac):
    //
    // let rcc  = embassy_stm32::pac::RCC;
    // let hrtim = embassy_stm32::pac::HRTIM1;
    //
    // // 1. Enable HRTIM clock on APB2.
    // rcc.apb2enr().modify(|w| w.set_hrtim1en(true));
    //
    // // 2. DLL calibration.
    // hrtim.dllcr().write(|w| { w.set_cal(true); w.set_calen(true); });
    // while !hrtim.isr().read().dllrdy() { cortex_m::asm::nop(); }
    //
    // // 3. Master timer.
    // hrtim.mcr().modify(|w| { w.set_intlvd(0); w.set_preen(true); });
    // hrtim.mper().write_value(PWM_PERIOD);
    // hrtim.mcmp1r().write_value(PWM_PERIOD / 2);
    //
    // ... (Timers A–D, DMA, outputs)

    info!("hrtim: init stub — full register programming TODO");
}

/// Enable RF outputs (call when clock is stable and Si5351 is locked).
///
/// Enables the HRTIM output cells for Timer A (PA8/PA9), Timer C (PB12/PB13),
/// and Timer D (PB14/PB15).  Timer B has no outputs.
///
/// The TX_ENABLE GPIO (PB2) is asserted separately by the transmitter task.
pub fn enable_outputs() {
    // TODO: HRTIM1_COMMON.OENR |= (TA1|TA2|TC1|TC2|TD1|TD2)
    //
    // let hrtim = embassy_stm32::pac::HRTIM1;
    // hrtim.oenr().write(|w| {
    //     w.set_ta1oen(true); w.set_ta2oen(true);
    //     w.set_tc1oen(true); w.set_tc2oen(true);
    //     w.set_td1oen(true); w.set_td2oen(true);
    // });
    debug!("hrtim: outputs enabled (stub)");
}

/// Disable all RF outputs and force them to the inactive level.
///
/// Call this before cutting power to the MOSFETs or when going idle.
/// All six outputs are set to inactive (low or high depending on polarity
/// config) via the HRTIM output disable register (ODISR).
pub fn disable_outputs() {
    // TODO: HRTIM1_COMMON.ODISR |= (TA1|TA2|TC1|TC2|TD1|TD2)
    debug!("hrtim: outputs disabled (stub)");
}

/// Update Timer-A burst DMA buffer entry at `index` with new [CMP1, CMP2].
///
/// Called by the DSP pipeline to write pre-computed phase values.  The DMA
/// will pick them up at the next Timer-C period boundary.
#[inline]
pub fn write_hrtim_sample(index: usize, cmp1: u32, cmp2: u32) {
    // Timer C only uses one word (CMP1) per the .ioc DMA size setting.
    // Timer A / B burst DMA uses two words [CMP1, CMP2] but is on a separate
    // DMA path that will be wired up in a later sprint.
    let _ = cmp2; // will be used for Timer-A burst DMA
    unsafe {
        HRTIM_BUF[index] = cmp1;
    }
}

// ---------------------------------------------------------------------------
// Compile-time layout verification
// ---------------------------------------------------------------------------
const _: () = {
    assert!(FRAME_SAMPLES > 0);
    assert!(PWM_PERIOD % 4 == 0);
    // Note: TIMERC_PERIOD (850 SYSCLK ticks, no high-res prescaler) and
    // PWM_PERIOD (high-res ticks, ×32 scale) are in different units — no
    // size comparison between them is meaningful.
};
