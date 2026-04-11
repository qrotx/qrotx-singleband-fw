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
//   arm_fir_decimate     100 × i32  →   10 × i32 (Q31, 20 kHz) [CMSIS-DSP]
//   q31_to_f32()          10 × i32  →   10 × f32
//   arm_biquad HP         10 × f32  →   10 × f32 (Chebyshev I HP, 200 Hz) [CMSIS-DSP]
//   compress()            10 × f32  →   10 × f32 (envelope follower, 4:1 ratio)
//   arm_biquad LP         10 × f32  →   10 × f32 (Chebyshev I LP, 2800 Hz) [CMSIS-DSP]
//   ssb_filter()          10 × f32  →  (10 × f32 I, 10 × f32 Q) analytic signal [TODO: C]
//   f32_to_q31()         (10 × I, 10 × Q) f32  →  (10 × I, 10 × Q) Q31
//   arm_fir_interpolate  (10 × Q31 I, 10 × Q31 Q)  →  (100 × Q31 I, 100 × Q31 Q) [CMSIS-DSP]
//   cordic + outphasing  (100 × I, 100 × Q) Q31  →  100 × PwmSample
//   write HRTIM ping-pong buffer half

use embassy_stm32::pac;

use crate::config::{FRAME_SAMPLES, PWM_PERIOD, TIMERC_PERIOD};
use crate::dsp_ffi::{
    ArmBiquadCascadeDf2TInstanceF32, ArmFirDecimateInstanceQ31, ArmFirInterpolateInstanceQ31,
    BIQUAD_HP_STATE_LEN, BIQUAD_LP_STATE_LEN, FIR_DEC_STATE_LEN, FIR_INTERP_STATE_LEN,
    arm_biquad_cascade_df2T_f32, arm_biquad_cascade_df2T_init_f32,
    arm_fir_decimate_init_q31, arm_fir_decimate_q31,
    arm_fir_interpolate_init_q31, arm_fir_interpolate_q31,
};
use crate::hrtim::{
    adc_buf_first_half, adc_buf_second_half,
    hrtim_buf_first_half_mut, hrtim_buf_second_half_mut,
    PwmSample,
};
use dsp_core::{
    DECIMATED_LEN, FIR_COEFFS_Q31, FIR_DECIMATE_FACTOR, FIR_NUM_TAPS,
    HIGHPASS_COEFFS, HIGHPASS_STAGES, LOWPASS_COEFFS, LOWPASS_STAGES,
    Compressor, SsbFilter,
    adc_to_q31, q31_to_f32, f32_to_q31,
};

// ---------------------------------------------------------------------------
// CORDIC outphasing constants
// ---------------------------------------------------------------------------

/// Transmission amplitude — sets the PA supply voltage via the buck converter.
/// Expressed in Timer C ticks (TIMERC_PERIOD = 850).  Higher = more TX power.
const AMPLITUDE: u32 = TIMERC_PERIOD / 2; // = 425 ticks

// ---------------------------------------------------------------------------
// CMSIS-DSP filter state buffers
// ---------------------------------------------------------------------------

static mut FIR_DEC_STATE:      [i32; FIR_DEC_STATE_LEN]    = [0; FIR_DEC_STATE_LEN];
static mut FIR_INTERP_I_STATE: [i32; FIR_INTERP_STATE_LEN] = [0; FIR_INTERP_STATE_LEN];
static mut FIR_INTERP_Q_STATE: [i32; FIR_INTERP_STATE_LEN] = [0; FIR_INTERP_STATE_LEN];
static mut BIQUAD_HP_STATE:    [f32; BIQUAD_HP_STATE_LEN]   = [0.0; BIQUAD_HP_STATE_LEN];
static mut BIQUAD_LP_STATE:    [f32; BIQUAD_LP_STATE_LEN]   = [0.0; BIQUAD_LP_STATE_LEN];

// ---------------------------------------------------------------------------
// CMSIS-DSP filter instances (null-initialised; filled by init())
// ---------------------------------------------------------------------------

static mut FIR_DEC_INST: ArmFirDecimateInstanceQ31 = ArmFirDecimateInstanceQ31 {
    m: 0, num_taps: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
};
static mut FIR_INTERP_I_INST: ArmFirInterpolateInstanceQ31 = ArmFirInterpolateInstanceQ31 {
    l: 0, phase_length: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
};
static mut FIR_INTERP_Q_INST: ArmFirInterpolateInstanceQ31 = ArmFirInterpolateInstanceQ31 {
    l: 0, phase_length: 0, p_coeffs: core::ptr::null(), p_state: core::ptr::null_mut(),
};
static mut BIQUAD_HP_INST: ArmBiquadCascadeDf2TInstanceF32 = ArmBiquadCascadeDf2TInstanceF32 {
    num_stages: 0, p_state: core::ptr::null_mut(), p_coeffs: core::ptr::null(),
};
static mut BIQUAD_LP_INST: ArmBiquadCascadeDf2TInstanceF32 = ArmBiquadCascadeDf2TInstanceF32 {
    num_stages: 0, p_state: core::ptr::null_mut(), p_coeffs: core::ptr::null(),
};

// ---------------------------------------------------------------------------
// Pure-Rust filter instances (compressor; SSB filter pending C replacement)
// ---------------------------------------------------------------------------

static mut COMPRESSOR: Compressor = Compressor::new();

// TODO: replace SsbFilter with the C implementation once c_src/ssb_filter.c
//       is integrated and the API is known.
static mut SSB_FILTER: SsbFilter = SsbFilter::new();

// ---------------------------------------------------------------------------
// CORDIC initialisation and per-sample calculation
// ---------------------------------------------------------------------------

/// Initialise the CORDIC peripheral and all CMSIS-DSP filter instances.
///
/// Must be called once during hardware init, before any process_*_half call.
pub fn init() {
    // --- CORDIC: MODULUS function, Q1.31, 60 iterations ---
    pac::RCC.ahb1enr().modify(|w| w.set_cordicen(true));
    pac::RCC.ahb1rstr().modify(|w| w.set_cordicrst(true));
    pac::RCC.ahb1rstr().modify(|w| w.set_cordicrst(false));

    pac::CORDIC.csr().write(|w| {
        w.set_func(pac::cordic::vals::Func::from_bits(3));           // MODULUS
        w.set_precision(pac::cordic::vals::Precision::from_bits(15));// 60 iterations
        w.set_scale(pac::cordic::vals::Scale::from_bits(0));
        w.set_nargs(pac::cordic::vals::Num::NUM2);
        w.set_nres(pac::cordic::vals::Num::NUM2);
        w.set_argsize(pac::cordic::vals::Size::BITS32);
        w.set_ressize(pac::cordic::vals::Size::BITS32);
    });

    // --- CMSIS-DSP filter initialisation ---
    unsafe {
        // FIR decimator: 60-tap lowpass, factor 10, input block = 100 samples.
        arm_fir_decimate_init_q31(
            &raw mut FIR_DEC_INST,
            FIR_NUM_TAPS as u16,
            FIR_DECIMATE_FACTOR as u8,
            FIR_COEFFS_Q31.as_ptr(),
            core::ptr::addr_of_mut!(FIR_DEC_STATE[0]),
            FRAME_SAMPLES as u32,
        );

        // Biquad highpass (2-stage Chebyshev I, 200 Hz, fs = 20 kHz).
        // HIGHPASS_COEFFS is [[f32;5];2]; memory layout is the same as [f32;10].
        arm_biquad_cascade_df2T_init_f32(
            &raw mut BIQUAD_HP_INST,
            HIGHPASS_STAGES as u8,
            HIGHPASS_COEFFS[0].as_ptr(),
            core::ptr::addr_of_mut!(BIQUAD_HP_STATE[0]),
        );

        // Biquad lowpass (2-stage Chebyshev I, 2800 Hz, fs = 20 kHz).
        arm_biquad_cascade_df2T_init_f32(
            &raw mut BIQUAD_LP_INST,
            LOWPASS_STAGES as u8,
            LOWPASS_COEFFS[0].as_ptr(),
            core::ptr::addr_of_mut!(BIQUAD_LP_STATE[0]),
        );

        // FIR interpolators (I and Q paths): factor 10, input block = 10 samples.
        arm_fir_interpolate_init_q31(
            &raw mut FIR_INTERP_I_INST,
            FIR_DECIMATE_FACTOR as u8,
            FIR_NUM_TAPS as u16,
            FIR_COEFFS_Q31.as_ptr(),
            core::ptr::addr_of_mut!(FIR_INTERP_I_STATE[0]),
            DECIMATED_LEN as u32,
        );
        arm_fir_interpolate_init_q31(
            &raw mut FIR_INTERP_Q_INST,
            FIR_DECIMATE_FACTOR as u8,
            FIR_NUM_TAPS as u16,
            FIR_COEFFS_Q31.as_ptr(),
            core::ptr::addr_of_mut!(FIR_INTERP_Q_STATE[0]),
            DECIMATED_LEN as u32,
        );
    }
}

/// Blocking CORDIC MODULUS calculation: (I, Q) → (modulus, phase), all Q1.31.
#[inline]
fn cordic_modulus(i: i32, q: i32) -> (i32, i32) {
    pac::CORDIC.wdata().write_value(i as u32);
    pac::CORDIC.wdata().write_value(q as u32);
    while !pac::CORDIC.csr().read().rrdy() {}
    let modulus = pac::CORDIC.rdata().read() as i32;
    let phase   = pac::CORDIC.rdata().read() as i32;
    (modulus, phase)
}

// ---------------------------------------------------------------------------
// Public pipeline entry points — called from the DSP Embassy task
// ---------------------------------------------------------------------------

/// Process the first half of the ADC ping-pong buffer (samples 0..FRAME_SAMPLES).
///
/// # Safety
/// Must be called only from the context that owns the first buffer half.
pub unsafe fn process_first_half() {
    let adc = adc_buf_first_half();
    let hrtim_out = hrtim_buf_first_half_mut();
    process_half(adc, hrtim_out);
}

/// Process the second half of the ADC ping-pong buffer.
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

#[allow(static_mut_refs)]
unsafe fn process_half(adc_in: &[u16], hrtim_out: &mut [PwmSample]) {
    debug_assert_eq!(adc_in.len(),    FRAME_SAMPLES);
    debug_assert_eq!(hrtim_out.len(), FRAME_SAMPLES);

    // --- Stage 1: ADC u16 → Q31 (DC centred) ---
    let mut q31_in = [0i32; FRAME_SAMPLES];
    for (dst, &src) in q31_in.iter_mut().zip(adc_in.iter()) {
        *dst = adc_to_q31(src);
    }

    // --- Stage 2: 10:1 FIR decimation → 10 Q31 samples at 20 kHz ---
    let mut decimated = [0i32; DECIMATED_LEN];
    arm_fir_decimate_q31(
        &FIR_DEC_INST,
        q31_in.as_ptr(),
        decimated.as_mut_ptr(),
        FRAME_SAMPLES as u32,
    );

    // --- Stage 3a: Q31 → f32 ---
    let mut audio = [0.0f32; DECIMATED_LEN];
    for (dst, &src) in audio.iter_mut().zip(decimated.iter()) {
        *dst = q31_to_f32(src);
    }

    // --- Stage 3b: highpass IIR (Chebyshev I, 200 Hz) ---
    let mut hp_out = [0.0f32; DECIMATED_LEN];
    arm_biquad_cascade_df2T_f32(
        &BIQUAD_HP_INST,
        audio.as_ptr(),
        hp_out.as_mut_ptr(),
        DECIMATED_LEN as u32,
    );

    // --- Stage 3c: compression ---
    let mut compressed = [0.0f32; DECIMATED_LEN];
    COMPRESSOR.process(&hp_out, &mut compressed);

    // --- Stage 3d: lowpass IIR (Chebyshev I, 2800 Hz) ---
    let mut lp_out = [0.0f32; DECIMATED_LEN];
    arm_biquad_cascade_df2T_f32(
        &BIQUAD_LP_INST,
        compressed.as_ptr(),
        lp_out.as_mut_ptr(),
        DECIMATED_LEN as u32,
    );

    // --- Stage 4: SSB analytic FIR → I and Q paths ---
    // TODO: replace with C implementation once c_src/ssb_filter.c is integrated.
    let mut ssb_i = [0.0f32; DECIMATED_LEN];
    let mut ssb_q = [0.0f32; DECIMATED_LEN];
    SSB_FILTER.process(&lp_out, &mut ssb_i, &mut ssb_q);

    // --- Stage 5a: f32 → Q31 ---
    let mut q31_i = [0i32; DECIMATED_LEN];
    let mut q31_q = [0i32; DECIMATED_LEN];
    for (dst, &src) in q31_i.iter_mut().zip(ssb_i.iter()) {
        *dst = f32_to_q31(src);
    }
    for (dst, &src) in q31_q.iter_mut().zip(ssb_q.iter()) {
        *dst = f32_to_q31(src);
    }

    // --- Stage 5b: 10:1 FIR interpolation → 100 Q31 samples at 200 kHz ---
    let mut interp_i = [0i32; FRAME_SAMPLES];
    let mut interp_q = [0i32; FRAME_SAMPLES];
    arm_fir_interpolate_q31(
        &FIR_INTERP_I_INST,
        q31_i.as_ptr(),
        interp_i.as_mut_ptr(),
        DECIMATED_LEN as u32,
    );
    arm_fir_interpolate_q31(
        &FIR_INTERP_Q_INST,
        q31_q.as_ptr(),
        interp_q.as_mut_ptr(),
        DECIMATED_LEN as u32,
    );

    // --- Stage 5c: CORDIC polar conversion + outphasing → HRTIM compare values ---
    for (i, slot) in hrtim_out.iter_mut().enumerate() {
        let (modulus, phase) = cordic_modulus(interp_i[i], interp_q[i]);

        let mag = (((modulus as i64) >> 15) * AMPLITUDE as i64) >> 16;
        let tc_cmp1 = (mag as u32).clamp(3, AMPLITUDE);

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
