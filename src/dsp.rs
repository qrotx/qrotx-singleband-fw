// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// dsp.rs — Digital signal processing pipeline.
//
// Called from the DSP Embassy task whenever the DMA half/full-transfer ISR
// sets PROCESS_FIRST_HALF or PROCESS_SECOND_HALF.
//
// Pipeline per half-buffer (FRAME_SAMPLES = 100 ADC samples at 200 kHz):
//
//   adc_to_q31()         100 × u16  →  100 × i32 (Q31, DC-centred)
//   fir_decimate()       100 × i32  →   10 × i32 (Q31, 20 kHz)
//   q31_to_f32()          10 × i32  →   10 × f32
//   iir_highpass()        10 × f32  →   10 × f32 (Chebyshev I HP, 200 Hz)
//   compress()            10 × f32  →   10 × f32 (envelope follower, 4:1 ratio)
//   iir_lowpass()         10 × f32  →   10 × f32 (Chebyshev I LP, 2800 Hz)
//   ssb_filter()          10 × f32  →  (10 × f32 I, 10 × f32 Q) analytic signal
//   f32_to_q31()         (10 × I, 10 × Q) f32  →  (10 × I, 10 × Q) Q31
//   fir_interpolate()    (10 × Q31 I, 10 × Q31 Q)  →  (100 × Q31 I, 100 × Q31 Q)
//   cordic + outphasing  (100 × I, 100 × Q) Q31  →  100 × PwmSample
//   write HRTIM ping-pong buffer half

use embassy_stm32::pac;

use crate::config::{FRAME_SAMPLES, PWM_PERIOD, TIMERC_PERIOD};
use crate::hrtim::{
    adc_buf_first_half, adc_buf_second_half,
    hrtim_buf_first_half_mut, hrtim_buf_second_half_mut,
    PwmSample,
};
use dsp_core::{
    DECIMATED_LEN,
    HIGHPASS_STAGES, HIGHPASS_COEFFS,
    LOWPASS_STAGES,  LOWPASS_COEFFS,
    FirDecimate, FirInterpolate, BiquadState, Compressor, SsbFilter,
    adc_to_q31, q31_to_f32, f32_to_q31,
};

// ---------------------------------------------------------------------------
// CORDIC outphasing constants
// ---------------------------------------------------------------------------

/// Transmission amplitude — sets the PA supply voltage via the buck converter.
/// Expressed in Timer C ticks (TIMERC_PERIOD = 850).  Higher = more TX power.
const AMPLITUDE: u32 = TIMERC_PERIOD / 2; // = 425 ticks

// ---------------------------------------------------------------------------
// Static filter instances — accessed only from the DSP task (single writer).
// ---------------------------------------------------------------------------
static mut DECIMATOR:      FirDecimate                  = FirDecimate::new();
static mut HP_FILTER:      BiquadState<HIGHPASS_STAGES> = BiquadState::new();
static mut COMPRESSOR:     Compressor                   = Compressor::new();
static mut LP_FILTER:      BiquadState<LOWPASS_STAGES>  = BiquadState::new();
static mut SSB_FILTER:     SsbFilter                    = SsbFilter::new();
static mut INTERP_I:       FirInterpolate               = FirInterpolate::new();
static mut INTERP_Q:       FirInterpolate               = FirInterpolate::new();

// ---------------------------------------------------------------------------
// CORDIC initialisation and per-sample calculation
// ---------------------------------------------------------------------------

/// Initialise the CORDIC peripheral for MODULUS function (Q1.31, 32-bit, 2 args / 2 results).
///
/// Must be called once during hardware init, before `process_first_half` /
/// `process_second_half`.  Uses PAC directly, consistent with HRTIM/ADC/DMA.
///
/// CORDIC config (matching C HAL `CORDIC_FUNCTION_MODULUS`):
///   Function  : MODULUS  (ARG1=I, ARG2=Q → RES1=|Z|, RES2=∠Z)
///   Scale     : 0        (no input/output scaling)
///   Precision : 15       (15 × 4 = 60 iterations, ≥ 19-bit accuracy)
///   ArgSize   : 32-bit   (Q1.31)
///   ResSize   : 32-bit   (Q1.31)
///   NbWrite   : 2        (one ARG1 + ARG2 write pair triggers a calculation)
///   NbRead    : 2        (read modulus then phase)
pub fn init() {
    // Enable CORDIC clock on AHB1.
    pac::RCC.ahb1enr().modify(|w| w.set_cordicen(true));
    // Brief reset to put CORDIC in a known state.
    pac::RCC.ahb1rstr().modify(|w| w.set_cordicrst(true));
    pac::RCC.ahb1rstr().modify(|w| w.set_cordicrst(false));

    pac::CORDIC.csr().write(|w| {
        w.set_func(pac::cordic::vals::Func::from_bits(3)); // MODULUS
        w.set_precision(pac::cordic::vals::Precision::from_bits(15)); // 60 iterations
        w.set_scale(pac::cordic::vals::Scale::from_bits(0)); // no scaling
        w.set_nargs(pac::cordic::vals::Num::NUM2); // write I then Q
        w.set_nres(pac::cordic::vals::Num::NUM2);  // read modulus then phase
        w.set_argsize(pac::cordic::vals::Size::BITS32);
        w.set_ressize(pac::cordic::vals::Size::BITS32);
    });
}

/// Blocking CORDIC MODULUS calculation: (I, Q) → (modulus, phase), all Q1.31.
///
/// Writes ARG1=I then ARG2=Q (ARG2 write triggers the calculation), then
/// reads modulus and phase once RRDY is set.
/// Typical latency: 60 CORDIC iterations ≈ 60 / SYSCLK ≈ 0.35 µs at 170 MHz.
#[inline]
fn cordic_modulus(i: i32, q: i32) -> (i32, i32) {
    pac::CORDIC.wdata().write_value(i as u32); // ARG1 = I (preloaded)
    pac::CORDIC.wdata().write_value(q as u32); // ARG2 = Q (triggers calc)
    while !pac::CORDIC.csr().read().rrdy() {}   // wait for result ready
    let modulus = pac::CORDIC.rdata().read() as i32; // primary result
    let phase   = pac::CORDIC.rdata().read() as i32; // secondary result
    (modulus, phase)
}

// ---------------------------------------------------------------------------
// Public pipeline entry points — called from the DSP Embassy task
// ---------------------------------------------------------------------------

/// Process the first half of the ADC ping-pong buffer (samples 0..FRAME_SAMPLES).
/// Writes results to the first half of the HRTIM ping-pong buffer.
///
/// # Safety
/// Must be called only from the context that owns the first buffer half
/// (i.e. after the DMA half-transfer flag has been set).
pub unsafe fn process_first_half() {
    let adc = adc_buf_first_half();
    let hrtim_out = hrtim_buf_first_half_mut();
    process_half(adc, hrtim_out);
}

/// Process the second half of the ADC ping-pong buffer (samples FRAME_SAMPLES..2×FRAME_SAMPLES).
/// Writes results to the second half of the HRTIM ping-pong buffer.
///
/// # Safety
/// Must be called only after the DMA transfer-complete flag has been set.
pub unsafe fn process_second_half() {
    let adc = adc_buf_second_half();
    let hrtim_out = hrtim_buf_second_half_mut();
    process_half(adc, hrtim_out);
}

// ---------------------------------------------------------------------------
// Internal pipeline implementation
// ---------------------------------------------------------------------------

/// Core pipeline: process one half-buffer of ADC samples into PwmSamples.
///
/// `adc_in`   : FRAME_SAMPLES raw ADC u16 samples (200 kHz)
/// `hrtim_out`: FRAME_SAMPLES PwmSample destination (200 kHz)
// Safety: all static filter instances are owned exclusively by the single DSP task;
// no other task or interrupt creates a reference to them while this runs.
#[allow(static_mut_refs)]
fn process_half(adc_in: &[u16], hrtim_out: &mut [PwmSample]) {
    debug_assert_eq!(adc_in.len(),    FRAME_SAMPLES);
    debug_assert_eq!(hrtim_out.len(), FRAME_SAMPLES);

    // --- Stage 1: convert ADC u16 → Q31 ---
    let mut q31_in = [0i32; FRAME_SAMPLES];
    for (dst, &src) in q31_in.iter_mut().zip(adc_in.iter()) {
        *dst = adc_to_q31(src);
    }

    // --- Stage 2: 10:1 FIR decimation → 10 Q31 samples at 20 kHz ---
    // Safety: static filters are only accessed from this function,
    // which is called exclusively from the single DSP task.
    let q31_in_arr: &[i32; FRAME_SAMPLES] =
        (&q31_in as &[i32]).try_into().unwrap();
    let decimated: [i32; DECIMATED_LEN] =
        unsafe { DECIMATOR.process(q31_in_arr) };

    // --- Stage 3a: Q31 → f32 ---
    let mut audio = [0.0f32; DECIMATED_LEN];
    for (dst, &src) in audio.iter_mut().zip(decimated.iter()) {
        *dst = q31_to_f32(src);
    }

    // --- Stage 3b: high-pass IIR (Chebyshev I, 200 Hz) ---
    let mut hp_out = [0.0f32; DECIMATED_LEN];
    unsafe { HP_FILTER.process(&HIGHPASS_COEFFS, &audio, &mut hp_out) };

    // --- Stage 3c: compression ---
    let mut compressed = [0.0f32; DECIMATED_LEN];
    unsafe { COMPRESSOR.process(&hp_out, &mut compressed) };

    // --- Stage 3d: low-pass IIR (Chebyshev I, 2800 Hz) ---
    let mut lp_out = [0.0f32; DECIMATED_LEN];
    unsafe { LP_FILTER.process(&LOWPASS_COEFFS, &compressed, &mut lp_out) };

    // `lp_out` holds 10 band-limited audio samples in f32 at 20 kHz.

    // --- Stage 4: SSB analytic FIR → I and Q paths ---
    let mut ssb_i = [0.0f32; DECIMATED_LEN];
    let mut ssb_q = [0.0f32; DECIMATED_LEN];
    unsafe { SSB_FILTER.process(&lp_out, &mut ssb_i, &mut ssb_q) };

    // --- Stage 5a: f32 → Q31 (before interpolation, avoid conversion at 200 kHz) ---
    let mut q31_i = [0i32; DECIMATED_LEN];
    let mut q31_q = [0i32; DECIMATED_LEN];
    for (dst, &src) in q31_i.iter_mut().zip(ssb_i.iter()) {
        *dst = f32_to_q31(src);
    }
    for (dst, &src) in q31_q.iter_mut().zip(ssb_q.iter()) {
        *dst = f32_to_q31(src);
    }

    // --- Stage 5b: 10:1 Q31 FIR interpolation → 100 Q31 samples at 200 kHz ---
    let mut interp_i = [0i32; FRAME_SAMPLES];
    let mut interp_q = [0i32; FRAME_SAMPLES];
    unsafe {
        INTERP_I.process(&q31_i, &mut interp_i);
        INTERP_Q.process(&q31_q, &mut interp_q);
    }

    // `interp_i` and `interp_q` are Q31 at 200 kHz, ready for CORDIC.

    // --- Stage 5c: CORDIC polar conversion + outphasing → HRTIM compare values ---
    //
    // CORDIC MODULUS maps (I, Q) → (modulus, phase), both Q1.31.
    //   modulus : magnitude of the analytic signal, range [0, 1)
    //   phase   : phase angle, range [-1, +1) representing [-π, +π)
    //
    // Timer C CMP1 (buck converter duty → PA supply voltage):
    //   Scales modulus Q1.31 to [3, AMPLITUDE] TIMERC ticks.
    //   (modulus >> 15) gives ~Q1.15; × AMPLITUDE >> 16 gives integer ticks.
    //   Hard minimum of 3: HRTIM does not allow triggers in the first 3 ticks.
    //
    // Timer A/B CMP1 (outphasing angle):
    //   Maps phase [-1, +1] (= [-π, +π]) to a tick offset in [0, PWM_PERIOD).
    //   Phase is negated: a positive phase means an earlier crossover, so the
    //   negation maps the signal phase correctly to the HRTIM timing.
    //   CMP2 = (CMP1 + PWM_PERIOD/2) % PWM_PERIOD maintains 180° complement.
    for (i, slot) in hrtim_out.iter_mut().enumerate() {
        let (modulus, phase) = cordic_modulus(interp_i[i], interp_q[i]);

        // Timer C CMP1: modulus → PA bias ticks.
        let mag = (((modulus as i64) >> 15) * AMPLITUDE as i64) >> 16;
        let tc_cmp1 = (mag as u32).clamp(3, AMPLITUDE);

        // Timer A/B CMP1/CMP2: phase → outphasing angle in PWM ticks.
        let phase_ticks =
            (((-(phase as i64)) >> 15) * (PWM_PERIOD as i64 / 2) >> 16)
            + PWM_PERIOD as i64 / 2;
        let ta_cmp1 = (phase_ticks as u32) % PWM_PERIOD;
        let ta_cmp2 = (ta_cmp1 + PWM_PERIOD / 2) % PWM_PERIOD;

        *slot = PwmSample {
            tim_a_cmp1: ta_cmp1,
            tim_a_cmp2: ta_cmp2,
            tim_b_cmp1: ta_cmp1,
            tim_b_cmp2: ta_cmp2,
            tim_c_cmp1: tc_cmp1,
        };
    }
}
