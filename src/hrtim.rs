// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// hrtim.rs — HRTIM1 full register initialisation and DMA buffer management.
//
// Hardware topology:
//
//   Master timer  period = PWM_PERIOD  prescaler = MUL32 (CKPSC=0)  continuous
//                 CMP1   = PWM_PERIOD/2  → resets Timer E at 180°
//
//   Timer A       period = PWM_PERIOD  prescaler = MUL32 (CKPSC=0)  continuous
//                 Fixed CMP1 = PWM_PERIOD/4  (SET  TA1)
//                 Fixed CMP2 = 3×PWM_PERIOD/4  (RESET TA1)
//                 Reset by: Timer D CMP1 event (0° reference)
//                           Timer E CMP1 event (180° reference)
//                 Dead-time: negative sign (overlap), MUL8 prescaler, 3 ticks R+F
//                 TA1 (PA8): Set on CMP1;  Reset on CMP2
//                 TA2 (PA9): Set/Reset = NONE → driven entirely by dead-time unit
//
//   Timer D       period = PWM_PERIOD  prescaler = MUL32 (CKPSC=0)  continuous
//                 Reset by: Master period (→ 0° phase reference)
//                 CMP1 updated by DMA burst → modulates Timer A reset phase (0° side)
//                 No outputs.
//
//   Timer E       period = PWM_PERIOD  prescaler = MUL32 (CKPSC=0)  continuous
//                 Reset by: Master CMP1 (→ 180° phase reference)
//                 CMP1 updated by DMA burst → modulates Timer A reset phase (180° side)
//                 Initial CMP1 = PWM_PERIOD×3/4 (= PWM_PERIOD/4 + PWM_PERIOD/2)
//                 No outputs.
//
//   Timer C       period = TIMERC_PERIOD (850)  prescaler = DIV1 (CKPSC=5) → ~200 kHz
//                 Runs freely (no reset from master)
//                 CMP1 updated by DMA burst → controls buck converter duty cycle
//                 Dead-time: positive sign, MUL8 prescaler, 10 ticks R+F
//                 TC1 (PB12): Set on period; Reset on CMP1
//                 TC2 (PB13): Set/Reset = NONE → driven by dead-time unit
//                 Preload + RepUpdate enabled; DIER.REPDE = 1 (DMA on REP)
//
// DMA buffer layout
// -----------------
// One double-buffer in SRAM1 (ping-pong, half-transfer ISR):
//
//   HRTIM_BUF  [2 × FRAME_SAMPLES] PwmSample  — DMA1_CH5 → HRTIM BDMADR
//   ADC_BUF    [2 × FRAME_SAMPLES] u16         — ADC1 → DMA1_CH1 (configured in adc.rs)
//
// Each PwmSample is 3 × u32 = 12 bytes written to BDMADR in one DMA burst per
// Timer-C period (every ~5 µs at 200 kHz).  The HRTIM burst DMA routing (BDTxUPR)
// distributes the 3 words to:
//   Word 0  → Timer C CMP1
//   Word 1  → Timer D CMP1
//   Word 2  → Timer E CMP1
//
// DMA1_CH5 configuration (performed in adc.rs / DMA sprint):
//   Source : HRTIM_BUF (memory, word, address increment)
//   Dest   : HRTIM1.BDMADR (peripheral, word, no increment)
//   Length : DMA_BUF_LEN × 3  words  (whole double-buffer)
//   Mode   : circular
//   Request: Timer C REP event (DIER.REPDE enabled here)

use core::sync::atomic::AtomicBool;
use defmt::{debug, info};

use crate::config::{FRAME_SAMPLES, PWM_PERIOD, TIMERC_PERIOD};

// ---------------------------------------------------------------------------
// DMA buffer element: one 200 kHz tick worth of HRTIM compare values.
// Must be repr(C) and word-aligned so that the DMA can stream it directly
// into HRTIM1.BDMADR as three consecutive 32-bit words.
// ---------------------------------------------------------------------------

/// Three HRTIM compare values written per Timer-C period via burst DMA.
///
/// Layout must match the BDTxUPR configuration exactly:
///   bdtupr(2): CMP1 of Timer C  → word 0
///   bdtupr(3): CMP1 of Timer D  → word 1
///   bdtupr(4): CMP1 of Timer E  → word 2
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PwmSample {
    pub tim_c_cmp1: u32, // Timer C CMP1 — buck converter duty cycle
    pub tim_d_cmp1: u32, // Timer D CMP1 — phase reset trigger (0° reference)
    pub tim_e_cmp1: u32, // Timer E CMP1 — phase reset trigger (180° reference, = tim_d_cmp1 + PWM_PERIOD/2)
}

// ---------------------------------------------------------------------------
// Total DMA buffer length = 2 × FRAME_SAMPLES (double-buffer / ping-pong)
// ---------------------------------------------------------------------------
const DMA_BUF_LEN: usize = FRAME_SAMPLES * 2;

/// HRTIM burst-DMA double-buffer in SRAM1.
/// Each element is one PwmSample (3 × u32 = 12 bytes).
/// Loaded to HRTIM1.BDMADR at 200 kHz by DMA1_CH5 (configured in adc.rs).
static mut HRTIM_BUF: [PwmSample; DMA_BUF_LEN] = [PwmSample {
    tim_c_cmp1: 0,
    tim_d_cmp1: 0,
    tim_e_cmp1: 0,
}; DMA_BUF_LEN];

/// Audio ADC samples (u16, 12-bit right-aligned).
/// DMA1_CH1: ADC1 → this buffer, circular, halfword (configured in adc.rs).
static mut ADC_BUF: [u16; DMA_BUF_LEN] = [0u16; DMA_BUF_LEN];

// ---------------------------------------------------------------------------
// Flags set in DMA half/full-transfer ISRs to signal the DSP task
// ---------------------------------------------------------------------------

/// Set by DMA half-transfer ISR: first half of ADC_BUF is ready for processing.
pub static PROCESS_FIRST_HALF: AtomicBool = AtomicBool::new(false);
/// Set by DMA full-transfer ISR: second half of ADC_BUF is ready for processing.
pub static PROCESS_SECOND_HALF: AtomicBool = AtomicBool::new(false);

// ---------------------------------------------------------------------------
// Buffer access — safe wrappers around the static muts
// ---------------------------------------------------------------------------

/// Borrow the first half of the ADC double-buffer (samples 0..FRAME_SAMPLES).
/// # Safety: call only from the half-transfer context that owns this half.
pub unsafe fn adc_buf_first_half() -> &'static [u16] {
    &ADC_BUF[..FRAME_SAMPLES]
}

/// Borrow the second half of the ADC double-buffer.
pub unsafe fn adc_buf_second_half() -> &'static [u16] {
    &ADC_BUF[FRAME_SAMPLES..]
}

/// Borrow the second half of the ADC double-buffer (mutable, for test setup).
#[allow(dead_code)]
pub unsafe fn adc_buf_second_half_mut() -> &'static mut [u16] {
    &mut ADC_BUF[FRAME_SAMPLES..]
}

/// Borrow the second half of the HRTIM output double-buffer (read-only).
#[allow(dead_code)]
pub unsafe fn hrtim_buf_second_half() -> &'static [PwmSample] {
    &HRTIM_BUF[FRAME_SAMPLES..]
}

/// Borrow the first half of the HRTIM output double-buffer (mutable).
pub unsafe fn hrtim_buf_first_half_mut() -> &'static mut [PwmSample] {
    &mut HRTIM_BUF[..FRAME_SAMPLES]
}

/// Borrow the second half of the HRTIM output double-buffer (mutable).
pub unsafe fn hrtim_buf_second_half_mut() -> &'static mut [PwmSample] {
    &mut HRTIM_BUF[FRAME_SAMPLES..]
}

/// Raw pointer to the start of ADC_BUF for DMA configuration.
/// Safe to call; dereferencing the returned pointer requires unsafe.
pub fn adc_buf_ptr() -> *mut u16 {
    core::ptr::addr_of_mut!(ADC_BUF) as *mut u16
}

/// Raw pointer to the start of HRTIM_BUF for DMA configuration.
/// The buffer is repr(C) PwmSample (3 × u32); cast to *mut u32 for DMA word transfers.
pub fn hrtim_buf_ptr() -> *mut u32 {
    core::ptr::addr_of_mut!(HRTIM_BUF) as *mut u32
}

// ---------------------------------------------------------------------------
// HRTIM initialisation
// ---------------------------------------------------------------------------

/// Initialise the HRTIM peripheral, configure all timers, burst DMA routing,
/// and set HRTIM output GPIO pins to AF13.
///
/// Outputs remain disabled after this call — they are enabled by
/// `enable_outputs()` after the Si5351 PLL has settled and the MCU clock
/// has been switched to the external reference.
///
/// Steps performed:
///   1. Enable HRTIM1 APB2 clock.
///   2. Configure output GPIO pins as AF13 (PA8/PA9 = Timer A, PB12/PB13 = Timer C).
///   3. Run DLL calibration and wait for lock.
///   4. ADC trigger 1: Timer C period event, update source = Timer C.
///   5. Burst DMA routing (BDTxUPR): C[CMP1], D[CMP1], E[CMP1].
///   6. Master timer: period = PWM_PERIOD, CMP1 = PWM_PERIOD/2, MUL32, continuous.
///   7. Timer A: fixed CMP1/CMP2, reset from Timer D and E CMP1 events, negative dead-time.
///   8. Timer D: period, CMP1, reset from Master period (0° reference), no outputs.
///   9. Timer E: period, CMP1, reset from Master CMP1 (180° reference), no outputs.
///  10. Timer C: period = TIMERC_PERIOD, DIV1, preload + REP DMA, positive dead-time.
///  11. Fill HRTIM_BUF with safe initial values.
///  12. Start Master + Timer A/C/D/E (outputs still disabled).
pub fn init() {
    use embassy_stm32::pac;
    use embassy_stm32::pac::gpio::vals::{Moder, Ospeedr};
    use embassy_stm32::pac::hrtim::vals::Sdt;

    let rcc   = pac::RCC;
    let hrtim = pac::HRTIM1;
    let gpioa = pac::GPIOA;
    let gpiob = pac::GPIOB;

    // ------------------------------------------------------------------
    // 1. Enable HRTIM1 clock on APB2.
    // ------------------------------------------------------------------
    rcc.apb2enr().modify(|w| w.set_hrtim1en(true));
    // Dummy read-back to flush the write through the bus matrix.
    let _ = rcc.apb2enr().read();

    // ------------------------------------------------------------------
    // 2. GPIO output pins → AF13 (HRTIM1 alternate function on STM32G4).
    //    PA8 = TA1, PA9 = TA2, PB12 = TC1, PB13 = TC2.
    //    Note: GPIOA / GPIOB clocks are already enabled by embassy_stm32::init().
    // ------------------------------------------------------------------
    // PA8, PA9 → AF13, very-high speed, push-pull, no pull.
    gpioa.moder().modify(|w| {
        w.set_moder(8, Moder::ALTERNATE);
        w.set_moder(9, Moder::ALTERNATE);
    });
    gpioa.afr(1).modify(|w| {
        w.set_afr(0, 13); // PA8 is index 0 in AFRH (pin 8 − 8 = 0)
        w.set_afr(1, 13); // PA9
    });
    gpioa.ospeedr().modify(|w| {
        w.set_ospeedr(8, Ospeedr::VERY_HIGH_SPEED);
        w.set_ospeedr(9, Ospeedr::VERY_HIGH_SPEED);
    });

    // PB12, PB13 → AF13, very-high speed.
    gpiob.moder().modify(|w| {
        w.set_moder(12, Moder::ALTERNATE);
        w.set_moder(13, Moder::ALTERNATE);
    });
    gpiob.afr(1).modify(|w| {
        w.set_afr(4, 13); // PB12 is index 4 in AFRH (12 − 8 = 4)
        w.set_afr(5, 13); // PB13
    });
    gpiob.ospeedr().modify(|w| {
        w.set_ospeedr(12, Ospeedr::VERY_HIGH_SPEED);
        w.set_ospeedr(13, Ospeedr::VERY_HIGH_SPEED);
    });

    // ------------------------------------------------------------------
    // 3. DLL calibration — CALIBRATIONRATE_3 (calrte=3), one-shot (calen=0).
    //    cal=1 starts calibration.  Poll dllrdy in HRTIM_ISR.
    // ------------------------------------------------------------------
    hrtim.dllcr().write(|w| {
        w.set_calrte(3);  // calibration rate = 3 (fastest single-shot)
        w.set_cal(true);  // start calibration
        w.set_calen(false); // one-shot (not periodic)
    });
    // Busy-wait; calibration takes << 100 µs at 170 MHz.
    let mut timeout = 200_000u32;
    while !hrtim.isr().read().dllrdy() {
        cortex_m::asm::nop();
        timeout -= 1;
        if timeout == 0 {
            defmt::error!("hrtim: DLL calibration timeout");
            break;
        }
    }

    // ------------------------------------------------------------------
    // 4. ADC Trigger 1 (adc1r(0)):  fire on Timer C period event.
    //    Update source (CR1 ADUSRC[0]): Timer C = value 3.
    // ------------------------------------------------------------------
    hrtim.adc1r(0).write(|w| w.set_adctper(2, true)); // adctper(2) = Timer C period
    hrtim.cr1().modify(|w| w.set_adusrc(0, 3));        // trigger 1 updated by Timer C

    // ------------------------------------------------------------------
    // 5. Burst DMA update register: which timer CMP registers are written
    //    per burst word.  The DMA transfers PwmSample words in order:
    //    [timC_cmp1, timD_cmp1, timE_cmp1].
    // ------------------------------------------------------------------
    hrtim.bdtupr(2).write(|w|   w.set_cmp(0, true));        // Timer C: CMP1
    hrtim.bdtupr(3).write(|w|   w.set_cmp(0, true));        // Timer D: CMP1
    hrtim.bdtupr(4).write(|w|   w.set_cmp(0, true));        // Timer E: CMP1

    // ------------------------------------------------------------------
    // 6. Master timer: MUL32 (CKPSC=0), continuous, period = PWM_PERIOD.
    //    CMP1 = PWM_PERIOD/2 → resets Timer E at 180°.
    //
    //    CKPSC encoding (STM32G4 HAL / RM0440):
    //      0 = MUL32  → fCOUNT = fHRTIM × 32 (high-resolution, ≈ 5.376 GHz at 168 MHz)
    //      5 = DIV1   → fCOUNT = fHRTIM       (same as SYSCLK, no high-res multiplication)
    //    The period/compare registers hold values in fCOUNT ticks.
    // ------------------------------------------------------------------
    hrtim.mcr().write(|w| {
        w.set_ckpsc(0);   // MUL32: fCOUNT = fHRTIM × 32 ≈ 5.376 GHz at 168 MHz
        w.set_cont(true); // continuous counting
    });
    hrtim.mper().write(|w| w.set_mper(PWM_PERIOD as u16));
    hrtim.mcmp(0).write(|w| w.set_mcmp((PWM_PERIOD / 2) as u16));

    // ------------------------------------------------------------------
    // 7. Timer A (index 0): same clock/period as Master.
    //    Fixed CMP1 = PWM_PERIOD/4 (SET TA1), fixed CMP2 = 3×PWM_PERIOD/4 (RESET TA1).
    //    Counter reset by Timer D CMP1 (timevnt[4] = Timer A Event 5)
    //                 and Timer E CMP1 (timevnt[6] = Timer A Event 7).
    // ------------------------------------------------------------------
    hrtim.tim(0).cr().write(|w| {
        w.set_ckpsc(0);   // MUL32: fCOUNT = fHRTIM × 32
        w.set_cont(true);
    });
    hrtim.tim(0).per().write(|w| w.set_per(PWM_PERIOD as u16));
    hrtim.tim(0).cmp(0).write(|w| w.set_cmp((PWM_PERIOD / 4) as u16));
    hrtim.tim(0).cmp(1).write(|w| w.set_cmp((PWM_PERIOD * 3 / 4) as u16));

    // Counter reset: Timer D CMP1 (tcmp1[2]) + Timer E CMP1 (tcmp1[3]).
    // For Timer A's rst register, tcmp1 indices map to: 0=TB, 1=TC, 2=TD, 3=TE.
    hrtim.tim(0).rst().write(|w| {
        w.set_tcmp1(2, true); // Timer D CMP1
        w.set_tcmp1(3, true); // Timer E CMP1
    });

    // Negative dead-time (overlap): both MOSFETs on simultaneously for
    // zero-voltage switching in the current-mode class-D bridge.
    // DTPRSC=0 (×8 DT resolution), rising = 3 ticks, falling = 3 ticks.
    hrtim.tim(0).dt().write(|w| {
        w.set_dtprsc(0);
        w.set_dtr(3);
        w.set_sdtr(Sdt::NEGATIVE);
        w.set_dtf(3);
        w.set_sdtf(Sdt::NEGATIVE);
    });

    // TA1 (PA8): set on own CMP1, reset on own CMP2
    hrtim.tim(0).setr(0).write(|w| w.set_cmp(0, true));  // SET on own CMP1
    hrtim.tim(0).rstr(0).write(|w| w.set_cmp(1, true));  // RESET on own CMP2
    // TA2 (PA9): no set/reset sources — waveform generated by dead-time unit.
    hrtim.tim(0).setr(1).write(|_w| {});
    hrtim.tim(0).rstr(1).write(|_w| {});

    // Enable dead-time insertion for Timer A.
    hrtim.tim(0).outr().modify(|w| w.set_dten(true));

    // ------------------------------------------------------------------
    // 8. Timer D (index 3): phase reset reference at 0°.
    //    No outputs. CMP1 updated by DMA burst to modulate Timer A reset phase.
    // ------------------------------------------------------------------
    hrtim.tim(3).cr().write(|w| {
        w.set_ckpsc(0);   // MUL32: fCOUNT = fHRTIM × 32
        w.set_cont(true);
    });
    hrtim.tim(3).per().write(|w| w.set_per(PWM_PERIOD as u16));
    hrtim.tim(3).cmp(0).write(|w| w.set_cmp((PWM_PERIOD / 4) as u16));
    hrtim.tim(3).rst().write(|w| w.set_mstper(true)); // 0° — reset from Master period

    // ------------------------------------------------------------------
    // 9. Timer E (index 4): phase reset reference at 180°.
    //    No outputs. CMP1 updated by DMA burst (= Timer D CMP1 + PWM_PERIOD/2).
    // ------------------------------------------------------------------
    hrtim.tim(4).cr().write(|w| {
        w.set_ckpsc(0);   // MUL32
        w.set_cont(true);
    });
    hrtim.tim(4).per().write(|w| w.set_per(PWM_PERIOD as u16));
    hrtim.tim(4).cmp(0).write(|w| w.set_cmp((PWM_PERIOD * 3 / 4) as u16));
    hrtim.tim(4).rst().write(|w| w.set_mstcmp(0, true)); // 180° — reset from Master CMP1

    // ------------------------------------------------------------------
    // 10. Timer C (index 2): independent 200 kHz buck-converter PWM.
    //    DIV1 (CKPSC=5) → fCOUNT = fHRTIM ≈ 168 MHz, period = 850 ticks.
    //    Preload enabled; CMP1 register is double-buffered and updated on REP.
    //    REP counter = 0 → REP event every period.
    //    DIER.REPDE = 1 → DMA request on REP.
    // ------------------------------------------------------------------
    hrtim.tim(2).cr().write(|w| {
        w.set_ckpsc(5);    // DIV1: fCOUNT = fHRTIM (SYSCLK, no high-res multiplication)
        w.set_cont(true);
        w.set_preen(true); // preload enable
        w.set_repu(true);  // update preloaded registers on repetition event
    });
    hrtim.tim(2).per().write(|w| w.set_per(TIMERC_PERIOD as u16));
    hrtim.tim(2).cmp(0).write(|w| w.set_cmp(1u16)); // near-zero initial duty
    hrtim.tim(2).rep().write(|w| w.set_rep(0));        // REP every period

    // Enable DMA request on Timer C repetition event (triggers DMA1_CH5 burst).
    hrtim.tim(2).dier().modify(|w| w.set_repde(true));

    // Positive dead-time (normal: both outputs inactive during transition).
    // DTPRSC=0 (×8 DT resolution), rising = 10 ticks, falling = 10 ticks.
    hrtim.tim(2).dt().write(|w| {
        w.set_dtprsc(0);
        w.set_dtr(10);
        w.set_sdtr(Sdt::POSITIVE);
        w.set_dtf(10);
        w.set_sdtf(Sdt::POSITIVE);
    });

    // TC1 (PB12): Set on period, reset on CMP1 → 0 % … 100 % duty via CMP1.
    hrtim.tim(2).setr(0).write(|w| w.set_per(true));
    hrtim.tim(2).rstr(0).write(|w| w.set_cmp(0, true)); // CMP1 index 0
    // TC2 (PB13): driven by dead-time unit.
    hrtim.tim(2).setr(1).write(|_w| {});
    hrtim.tim(2).rstr(1).write(|_w| {});

    // Enable dead-time insertion for Timer C.
    hrtim.tim(2).outr().modify(|w| w.set_dten(true));

    // ------------------------------------------------------------------
    // 11. Fill HRTIM_BUF with safe initial values (no RF output, near-zero
    //     duty on Timer C) before DMA starts streaming.
    // ------------------------------------------------------------------
    let safe = PwmSample {
        tim_c_cmp1: 1,
        tim_d_cmp1: (PWM_PERIOD / 4) as u32,
        tim_e_cmp1: (PWM_PERIOD * 3 / 4) as u32,
    };
    #[allow(static_mut_refs)]
    unsafe {
        for slot in HRTIM_BUF.iter_mut() {
            *slot = safe;
        }
    }

    // ------------------------------------------------------------------
    // 12. Start all configured timers (outputs remain disabled until
    //     enable_outputs() is called after clock lock).
    // ------------------------------------------------------------------
    hrtim.mcr().modify(|w| {
        w.set_mcen(true);    // Master counter
        w.set_tcen(0, true); // Timer A
        w.set_tcen(2, true); // Timer C
        w.set_tcen(3, true); // Timer D
        w.set_tcen(4, true); // Timer E
    });

    info!("hrtim: init done — timers running, outputs disabled");
}

// ---------------------------------------------------------------------------
// Output enable / disable
// ---------------------------------------------------------------------------

/// Enable the HRTIM output cells for Timer A (PA8/PA9) and Timer C (PB12/PB13).
/// Timer B has no outputs.
///
/// Call only after the Si5351 PLL has settled and the MCU clock has been
/// switched to the external reference (stable frequency for the RF bridge).
pub fn enable_outputs() {
    use embassy_stm32::pac;
    let hrtim = pac::HRTIM1;
    // t1oen(n) / t2oen(n): output 1 / output 2 of timer n (0=A … 4=E).
    hrtim.oenr().modify(|w| {
        w.set_t1oen(0, true); // TA1 (PA8)
        w.set_t2oen(0, true); // TA2 (PA9)
        w.set_t1oen(2, true); // TC1 (PB12)
        w.set_t2oen(2, true); // TC2 (PB13)
    });
    debug!("hrtim: outputs enabled");
}

/// Force all HRTIM outputs to the inactive level via ODISR.
///
/// Call this before cutting power to the MOSFETs or when going idle.
pub fn disable_outputs() {
    use embassy_stm32::pac;
    let hrtim = pac::HRTIM1;
    hrtim.odisr().modify(|w| {
        w.set_t1odis(0, true); // TA1
        w.set_t2odis(0, true); // TA2
        w.set_t1odis(2, true); // TC1
        w.set_t2odis(2, true); // TC2
    });
    debug!("hrtim: outputs disabled");
}

// ---------------------------------------------------------------------------
// Compile-time layout verification
// ---------------------------------------------------------------------------
const _: () = {
    assert!(FRAME_SAMPLES > 0);
    assert!(PWM_PERIOD % 4 == 0);
    // PwmSample must be exactly 3 × 4 = 12 bytes.
    assert!(core::mem::size_of::<PwmSample>() == 12);
};
