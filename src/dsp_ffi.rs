// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// dsp_ffi.rs — FFI bindings for CMSIS-DSP filtering functions.
//
// The C objects are compiled from c_src/CMSIS-DSP/Source/FilteringFunctions/
// by build.rs and linked as the static library `cmsis_dsp`.
//
// Struct layouts must match the C definitions in arm_math.h exactly.
// All structs are repr(C) and contain raw pointers; they live in static mut
// statics in dsp.rs and are accessed exclusively from the single DSP task.

use crate::config::FRAME_SAMPLES;
use crate::dsp::{
    DECIMATED_LEN, FIR_DECIMATE_FACTOR, FIR_NUM_TAPS,
    HIGHPASS_STAGES, LOWPASS_STAGES, SSB_FILTER_TAPS,
};

// ---------------------------------------------------------------------------
// State buffer sizes (used to declare static arrays in dsp.rs)
// ---------------------------------------------------------------------------

/// FIR decimator state: numTaps + blockSize − 1 = 60 + 100 − 1 = 159.
pub const FIR_DEC_STATE_LEN: usize = FIR_NUM_TAPS + FRAME_SAMPLES - 1;

/// FIR interpolator state per channel: (numTaps/L) + blockSize − 1 = 6 + 10 − 1 = 15.
pub const FIR_INTERP_STATE_LEN: usize = FIR_NUM_TAPS / FIR_DECIMATE_FACTOR + DECIMATED_LEN - 1;

/// Biquad DF2T state per instance: 2 × numStages floats.
pub const BIQUAD_HP_STATE_LEN: usize = 2 * HIGHPASS_STAGES; // 4
pub const BIQUAD_LP_STATE_LEN: usize = 2 * LOWPASS_STAGES;  // 4

/// SSB FIR state: numTaps + blockSize − 1 = 257 + 10 − 1 = 266.
pub const SSB_STATE_LEN: usize = SSB_FILTER_TAPS + DECIMATED_LEN - 1;

// ---------------------------------------------------------------------------
// Instance structs — layout mirrors the C typedef in arm_math.h.
//
// arm_fir_decimate_instance_q31 / arm_fir_interpolate_instance_q31:
//   { uint8_t factor, [1-byte pad], uint16_t numTaps, const q31_t*, q31_t* }
//   = 12 bytes on a 32-bit target with repr(C) padding rules.
//
// arm_biquad_cascade_df2T_instance_f32:
//   { uint32_t numStages, float32_t* pState, const float32_t* pCoeffs }
//   = 12 bytes.
//
// arm_fir_instance_f32 (reused for arm_fir_ssb_f32):
//   { uint16_t numTaps, [2-byte pad], float32_t* pState, const float32_t* pCoeffs }
//   = 12 bytes.
// ---------------------------------------------------------------------------

#[repr(C)]
pub struct ArmFirDecimateInstanceQ31 {
    pub m:        u8,         // decimation factor M
    pub num_taps: u16,        // total number of FIR taps
    pub p_coeffs: *const i32, // const q31_t* — prototype filter coefficients
    pub p_state:  *mut i32,   // q31_t*       — delay line (FIR_DEC_STATE_LEN)
}

#[repr(C)]
pub struct ArmFirInterpolateInstanceQ31 {
    pub l:            u8,         // upsample factor L
    pub phase_length: u16,        // taps per polyphase sub-filter = numTaps/L
    pub p_coeffs:     *const i32, // const q31_t* — prototype filter coefficients
    pub p_state:      *mut i32,   // q31_t*       — delay line (FIR_INTERP_STATE_LEN)
}

#[repr(C)]
pub struct ArmFirInstanceF32 {
    pub num_taps: u16,        // number of filter taps
    pub p_state:  *mut f32,   // float32_t* — state buffer (numTaps + blockSize − 1)
    pub p_coeffs: *const f32, // const float32_t* — interleaved {real, imag} coefficients
}

#[repr(C)]
pub struct ArmBiquadCascadeDf2TInstanceF32 {
    pub num_stages: u32,        // number of second-order sections
    pub p_state:    *mut f32,   // float32_t* — state buffer (2 × numStages)
    pub p_coeffs:   *const f32, // const float32_t* — {b0,b1,b2,a1,a2} per stage
}

// ---------------------------------------------------------------------------
// extern "C" declarations — linked from cmsis_dsp static library
// ---------------------------------------------------------------------------

extern "C" {
    /// Initialise the FIR decimation instance.
    /// blockSize: number of INPUT samples per call (must be a multiple of M).
    pub fn arm_fir_decimate_init_q31(
        s:          *mut ArmFirDecimateInstanceQ31,
        num_taps:   u16,
        m:          u8,
        p_coeffs:   *const i32,
        p_state:    *mut i32,
        block_size: u32,
    ) -> i32; // arm_status: 0 = ARM_MATH_SUCCESS

    /// Run the FIR decimation filter on one block of input samples.
    /// blockSize: number of INPUT samples; output length = blockSize / M.
    pub fn arm_fir_decimate_q31(
        s:          *const ArmFirDecimateInstanceQ31,
        p_src:      *const i32,
        p_dst:      *mut i32,
        block_size: u32,
    );

    /// Initialise the FIR interpolation instance.
    /// blockSize: number of INPUT samples per call.
    pub fn arm_fir_interpolate_init_q31(
        s:          *mut ArmFirInterpolateInstanceQ31,
        l:          u8,
        num_taps:   u16,
        p_coeffs:   *const i32,
        p_state:    *mut i32,
        block_size: u32,
    ) -> i32;

    /// Run the FIR interpolation filter on one block of input samples.
    /// blockSize: number of INPUT samples; output length = blockSize × L.
    pub fn arm_fir_interpolate_q31(
        s:          *const ArmFirInterpolateInstanceQ31,
        p_src:      *const i32,
        p_dst:      *mut i32,
        block_size: u32,
    );

    /// Initialise a cascaded biquad Direct Form II Transposed instance.
    /// Coefficient layout per stage: {b0, b1, b2, a1, a2}
    /// where a1, a2 are the negated denominator coefficients
    /// (same sign convention as dsp_core HIGHPASS_COEFFS / LOWPASS_COEFFS).
    pub fn arm_biquad_cascade_df2T_init_f32(
        s:          *mut ArmBiquadCascadeDf2TInstanceF32,
        num_stages: u8,
        p_coeffs:   *const f32,
        p_state:    *mut f32,
    );

    /// Run the cascaded biquad filter on one block of samples.
    pub fn arm_biquad_cascade_df2T_f32(
        s:          *const ArmBiquadCascadeDf2TInstanceF32,
        p_src:      *const f32,
        p_dst:      *mut f32,
        block_size: u32,
    );

    /// Run the SSB analytic FIR filter, producing I and Q outputs simultaneously.
    /// Uses arm_fir_instance_f32 with interleaved {real, imag} coefficients.
    pub fn arm_fir_ssb_f32(
        s:          *const ArmFirInstanceF32,
        p_src:      *const f32,
        p_dst_i:    *mut f32,
        p_dst_q:    *mut f32,
        block_size: u32,
    );
}
