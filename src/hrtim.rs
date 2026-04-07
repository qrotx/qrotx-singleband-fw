// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// hrtim.rs — HRTIM1 full register initialisation and DMA buffer management.
//
// Hardware topology:
//
//   Master timer  period = PWM_PERIOD  prescaler = MUL32 (CKPSC=5)  continuous
//                 CMP1   = PWM_PERIOD/2  → resets Timer B at 180°
//
//   Timer A       period = PWM_PERIOD  prescaler = MUL32  continuous
//                 Reset by: Master period (→ 0° phase)
//                 CMP1/CMP2 updated by DMA burst (phase control for class-D bridge)
//                 Dead-time: negative sign (overlap), MUL8 prescaler, 3 ticks R+F
//                 TA1 (PA8): Set on CMP1 | TimerB-CMP2;  Reset on CMP2 | TimerB-CMP1
//                 TA2 (PA9): Set/Reset = NONE → driven entirely by dead-time unit
//
//   Timer B       period = PWM_PERIOD  prescaler = MUL32  continuous
//                 Reset by: Master CMP1 (→ 180° phase, offset from Timer A)
//                 CMP1/CMP2 updated by DMA burst
//                 No outputs.  Provides cross-coupling trigger events to Timer A.
//
//   Timer C       period = TIMERC_PERIOD (850)  prescaler = DIV1 (CKPSC=0) → ~200 kHz
//                 Runs freely (no reset from master)
//                 CMP1 updated by DMA burst → controls buck converter duty cycle
//                 Dead-time: positive sign, MUL8 prescaler, 10 ticks R+F
//                 TC1 (PB12): Set on period; Reset on CMP1
//                 TC2 (PB13): Set/Reset = NONE → driven by dead-time unit
//                 Preload + RepUpdate enabled; DIER.REPDE = 1 (DMA on REP)
//
//   Timer D       removed — not required.
//
// DMA buffer layout
// -----------------
// One double-buffer in CCMRAM (ping-pong, half-transfer ISR):
//
//   HRTIM_BUF  [2 × FRAME_SAMPLES] PwmSample  — DMA1_CH5 → HRTIM BDMADR
//   ADC_BUF    [2 × FRAME_SAMPLES] u16         — ADC1 → DMA1_CH1 (configured in adc.rs)
//
// Each PwmSample is 5 × u32 = 20 bytes written to BDMADR in one DMA burst per
// Timer-C period (every ~5 µs at 200 kHz).  The HRTIM burst DMA routing (BDTxUPR)
// distributes the 5 words to:
//   Word 0  → Timer A CMP1
//   Word 1  → Timer A CMP2
//   Word 2  → Timer B CMP1
//   Word 3  → Timer B CMP2
//   Word 4  → Timer C CMP1
//
// DMA1_CH5 configuration (performed in adc.rs / DMA sprint):
//   Source : HRTIM_BUF (memory, word, address increment)
//   Dest   : HRTIM1.BDMADR (peripheral, word, no increment)
//   Length : DMA_BUF_LEN × 5  words  (whole double-buffer)
//   Mode   : circular
//   Request: Timer C REP event (DIER.REPDE enabled here)

use core::sync::atomic::AtomicBool;
use defmt::{debug, info};

use crate::config::{FRAME_SAMPLES, PWM_PERIOD, TIMERC_PERIOD};

// ---------------------------------------------------------------------------
// DMA buffer element: one 200 kHz tick worth of HRTIM compare values.
// Must be repr(C) and word-aligned so that the DMA can stream it directly
// into HRTIM1.BDMADR as five consecutive 32-bit words.
// ---------------------------------------------------------------------------

/// Five HRTIM compare values written per Timer-C period via burst DMA.
///
/// Layout must match the BDTxUPR configuration exactly:
///   bdtupr(0): CMP1+CMP2 of Timer A   → words 0, 1
///   bdtupr(1): CMP1+CMP2 of Timer B   → words 2, 3
///   bdtupr(2): CMP1       of Timer C  → word 4
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PwmSample {
    pub tim_a_cmp1: u32, // Timer A CMP1 — leading edge phase (0°+φ)
    pub tim_a_cmp2: u32, // Timer A CMP2 — trailing edge phase (180°+φ)
    pub tim_b_cmp1: u32, // Timer B CMP1 — leading edge phase (0°+φ, B is 180° shifted)
    pub tim_b_cmp2: u32, // Timer B CMP2 — trailing edge phase (180°+φ, B frame)
    pub tim_c_cmp1: u32, // Timer C CMP1 — buck converter duty cycle (0 = 0 %, TIMERC_PERIOD = 100 %)
}

// ---------------------------------------------------------------------------
// Total DMA buffer length = 2 × FRAME_SAMPLES (double-buffer / ping-pong)
// ---------------------------------------------------------------------------
const DMA_BUF_LEN: usize = FRAME_SAMPLES * 2;

/// HRTIM burst-DMA double-buffer in CCMRAM (tightly-coupled for lowest DMA latency).
/// Each element is one PwmSample (5 × u32 = 20 bytes).
/// Loaded to HRTIM1.BDMADR at 200 kHz by DMA1_CH5 (configured in adc.rs).
#[link_section = ".ccmram"]
static mut HRTIM_BUF: [PwmSample; DMA_BUF_LEN] = [PwmSample {
    tim_a_cmp1: 0,
    tim_a_cmp2: 0,
    tim_b_cmp1: 0,
    tim_b_cmp2: 0,
    tim_c_cmp1: 0,
}; DMA_BUF_LEN];

/// Audio ADC samples (u16, 12-bit right-aligned).
/// DMA1_CH1: ADC1 → this buffer, circular, halfword (configured in adc.rs).
#[link_section = ".ccmram"]
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

/// Borrow the first half of the HRTIM output double-buffer (mutable).
pub unsafe fn hrtim_buf_first_half_mut() -> &'static mut [PwmSample] {
    &mut HRTIM_BUF[..FRAME_SAMPLES]
}

/// Borrow the second half of the HRTIM output double-buffer (mutable).
pub unsafe fn hrtim_buf_second_half_mut() -> &'static mut [PwmSample] {
    &mut HRTIM_BUF[FRAME_SAMPLES..]
}

/// Write a complete PwmSample at `index` (0..DMA_BUF_LEN) in the HRTIM buffer.
/// Called by the DSP pipeline to fill the buffer.
#[inline]
pub unsafe fn write_hrtim_sample(index: usize, sample: PwmSample) {
    HRTIM_BUF[index] = sample;
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
///   5. Burst DMA routing (BDTxUPR): A[CMP1,CMP2], B[CMP1,CMP2], C[CMP1].
///   6. Master timer: period = PWM_PERIOD, CMP1 = PWM_PERIOD/2, MUL32, continuous.
///   7. Timer A: period, CMP1/2, reset from Master period (0°), negative dead-time.
///      TA1 set/reset sources with Timer B cross-coupling.
///   8. Timer B: period, CMP1/2, reset from Master CMP1 (180°), no dead-time.
///   9. Timer C: period = TIMERC_PERIOD, DIV1, preload + REP DMA, positive dead-time.
///  10. Fill HRTIM_BUF with safe initial values.
///  11. Start Master + Timer A/B/C (outputs still disabled).
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
    //    [timA_cmp1, timA_cmp2, timB_cmp1, timB_cmp2, timC_cmp1].
    // ------------------------------------------------------------------
    hrtim.bdtupr(0).write(|w| { w.set_cmp(0, true); w.set_cmp(1, true); }); // Timer A: CMP1+CMP2
    hrtim.bdtupr(1).write(|w| { w.set_cmp(0, true); w.set_cmp(1, true); }); // Timer B: CMP1+CMP2
    hrtim.bdtupr(2).write(|w|   w.set_cmp(0, true));                         // Timer C: CMP1 only

    // ------------------------------------------------------------------
    // 6. Master timer: MUL32 (CKPSC=5), continuous, period = PWM_PERIOD.
    //    CMP1 = PWM_PERIOD/2 → resets Timer B at 180°.
    // ------------------------------------------------------------------
    hrtim.mcr().write(|w| {
        w.set_ckpsc(5);   // MUL32 (32× high-res, fCOUNT = 32 × fHRCLK ≈ 5.4 GHz)
        w.set_cont(true); // continuous counting
    });
    hrtim.mper().write(|w| w.set_mper(PWM_PERIOD as u16));
    hrtim.mcmp(0).write(|w| w.set_mcmp((PWM_PERIOD / 2) as u16));

    // ------------------------------------------------------------------
    // 7. Timer A (index 0): same clock/period as Master.
    //    Reset from Master period → Timer A starts at 0° each master cycle.
    //    Cross-coupling with Timer B via timevnt events in setr/rstr:
    //      timevnt[0] = Timer A Event 1 = Timer B CMP1 (RM0440 Table 215)
    //      timevnt[1] = Timer A Event 2 = Timer B CMP2
    // ------------------------------------------------------------------
    hrtim.tim(0).cr().write(|w| {
        w.set_ckpsc(5);
        w.set_cont(true);
    });
    hrtim.tim(0).per().write(|w| w.set_per(PWM_PERIOD as u16));
    // Initial CMP values: half duty at 90° / 270° — will be updated by DMA.
    hrtim.tim(0).cmp(0).write(|w| w.set_cmp((PWM_PERIOD / 4) as u16));
    hrtim.tim(0).cmp(1).write(|w| w.set_cmp((PWM_PERIOD * 3 / 4) as u16));

    // Timer A reset trigger: Master period event.
    hrtim.tim(0).rst().write(|w| w.set_mstper(true));

    // Negative dead-time (overlap): both MOSFETs on simultaneously for
    // zero-voltage switching in the current-mode class-D bridge.
    // DTPRSC=3 (×8 DT resolution), rising = 3 ticks, falling = 3 ticks.
    hrtim.tim(0).dt().write(|w| {
        w.set_dtprsc(3);
        w.set_dtr(3);
        w.set_sdtr(Sdt::NEGATIVE);
        w.set_dtf(3);
        w.set_sdtf(Sdt::NEGATIVE);
    });

    // TA1 (PA8) set/reset: CMP1 | TimerB-CMP2  /  CMP2 | TimerB-CMP1.
    hrtim.tim(0).setr(0).write(|w| {
        w.set_cmp(0, true);     // own CMP1
        w.set_timevnt(1, true); // Timer A event 2 = Timer B CMP2
    });
    hrtim.tim(0).rstr(0).write(|w| {
        w.set_cmp(1, true);     // own CMP2
        w.set_timevnt(0, true); // Timer A event 1 = Timer B CMP1
    });
    // TA2 (PA9): no set/reset sources — waveform generated by dead-time unit.
    hrtim.tim(0).setr(1).write(|_w| {});
    hrtim.tim(0).rstr(1).write(|_w| {});

    // Enable dead-time insertion for Timer A.
    hrtim.tim(0).outr().modify(|w| w.set_dten(true));

    // ------------------------------------------------------------------
    // 8. Timer B (index 1): sync reference for Timer A cross-coupling.
    //    Reset from Master CMP1 → 180° offset from Timer A.
    //    No outputs.
    // ------------------------------------------------------------------
    hrtim.tim(1).cr().write(|w| {
        w.set_ckpsc(5);
        w.set_cont(true);
    });
    hrtim.tim(1).per().write(|w| w.set_per(PWM_PERIOD as u16));
    hrtim.tim(1).cmp(0).write(|w| w.set_cmp((PWM_PERIOD / 4) as u16));
    hrtim.tim(1).cmp(1).write(|w| w.set_cmp((PWM_PERIOD * 3 / 4) as u16));

    // Timer B reset trigger: Master CMP1 event (mstcmp index 0 = CMP1).
    hrtim.tim(1).rst().write(|w| w.set_mstcmp(0, true));

    // ------------------------------------------------------------------
    // 9. Timer C (index 2): independent 200 kHz buck-converter PWM.
    //    DIV1 (CKPSC=0) → fCOUNT = fHRCLK ≈ 169.5 MHz, period = 850 ticks.
    //    Preload enabled; CMP1 register is double-buffered and updated on REP.
    //    REP counter = 0 → REP event every period.
    //    DIER.REPDE = 1 → DMA request on REP.
    // ------------------------------------------------------------------
    hrtim.tim(2).cr().write(|w| {
        w.set_ckpsc(0);    // DIV1: fCOUNT = fHRCLK (no ×32 multiplier)
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
    // DTPRSC=3 (×8), rising = 10 ticks, falling = 10 ticks.
    hrtim.tim(2).dt().write(|w| {
        w.set_dtprsc(3);
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
    // 10. Fill HRTIM_BUF with safe initial values (no RF output, near-zero
    //     duty on Timer C) before DMA starts streaming.
    // ------------------------------------------------------------------
    let safe = PwmSample {
        tim_a_cmp1: (PWM_PERIOD / 4) as u32,
        tim_a_cmp2: (PWM_PERIOD * 3 / 4) as u32,
        tim_b_cmp1: (PWM_PERIOD / 4) as u32,
        tim_b_cmp2: (PWM_PERIOD * 3 / 4) as u32,
        tim_c_cmp1: 1,   // near-zero duty; 0 risks CMP=period alias on some HRTIM revisions
    };
    #[allow(static_mut_refs)]
    unsafe {
        for slot in HRTIM_BUF.iter_mut() {
            *slot = safe;
        }
    }

    // ------------------------------------------------------------------
    // 11. Start all configured timers (outputs remain disabled until
    //     enable_outputs() is called after clock lock).
    // ------------------------------------------------------------------
    hrtim.mcr().modify(|w| {
        w.set_mcen(true);    // Master counter
        w.set_tcen(0, true); // Timer A
        w.set_tcen(1, true); // Timer B
        w.set_tcen(2, true); // Timer C
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
    // PwmSample must be exactly 5 × 4 = 20 bytes.
    assert!(core::mem::size_of::<PwmSample>() == 20);
};
