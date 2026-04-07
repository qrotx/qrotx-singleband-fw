// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// config.rs — compile-time hardware constants and frequency calculations.
//
// All frequency maths are done at compile time via `const fn` so that
// zero-cost constants end up directly in the generated machine code.

// ---------------------------------------------------------------------------
// Radio configuration
// ---------------------------------------------------------------------------

/// Transmit frequency in Hz (change here to retune everything at compile time).
pub const TX_FREQ_HZ: u32 = 7_290_000;

// ---------------------------------------------------------------------------
// HRTIM / clock constants
//
// The MCU must run at exactly `SYSCLK_HZ` for the HRTIM PWM period to produce
// the correct RF frequency.  `SYSCLK_HZ` is derived from TX_FREQ_HZ so that
// the PWM period is always a clean multiple-of-4 count.
//
// When running from the internal oscillator (startup / idle) the PLL is
// locked to HSI → 168 MHz.  For transmission the Si5351 provides an external
// reference that brings the PLL to `SYSCLK_HZ`.
// ---------------------------------------------------------------------------

/// Base HRTIM clock used in the PWM-period calculation.
/// The HRTIM counter runs at SYSCLK; high-resolution mode multiplies this by
/// `HRTIM_HR_FACTOR` to get the effective tick rate.
const HRTIM_CALC_BASE_HZ: u64 = 170_000_000;

/// HRTIM high-resolution factor (DLL locked, ×32 in STM32G474).
pub const HRTIM_HR_FACTOR: u32 = 32;

/// PWM period in HRTIM high-resolution ticks for `TX_FREQ_HZ`.
/// Rounded DOWN to the nearest multiple of 4 (HRTIM requirement).
pub const PWM_PERIOD: u32 = {
    let tmp = (HRTIM_CALC_BASE_HZ * HRTIM_HR_FACTOR as u64) / TX_FREQ_HZ as u64;
    let p = tmp as u32;
    p - (p % 4)
};

/// Required system clock in Hz for `TX_FREQ_HZ` and `PWM_PERIOD`.
/// HRTIM will produce exactly TX_FREQ_HZ when the MCU runs at this clock.
pub const SYSCLK_TX_HZ: u32 =
    ((TX_FREQ_HZ as u64 * PWM_PERIOD as u64) / HRTIM_HR_FACTOR as u64) as u32;

/// STM32 PLL net multiplier (PLLN / PLLR) for the external clock.
/// Si5351 output = SYSCLK_TX_HZ / PLL_NET_MUL.
/// PLLM=1, so VCO_in = Si5351 output directly.
pub const PLL_NET_MUL: u32 = SYSCLK_TX_HZ / 8_000_000;

/// Si5351 output frequency required (Hz).
/// The STM32 PLL (×PLL_NET_MUL) brings this up to SYSCLK_TX_HZ.
pub const SI5351_OUTPUT_HZ: u32 = SYSCLK_TX_HZ / PLL_NET_MUL;

// The STM32G474 PLL requires PLLR ∈ {2,4,6,8}.  We use PLLR=2, so:
//   PLLN = PLL_NET_MUL × PLLR / 1 (PLLM = 1)
/// STM32 PLLN value for transmission mode (PLLM=1, PLLR=2).
pub const PLL_N_TX: u32 = PLL_NET_MUL * 2;

// ---------------------------------------------------------------------------
// Internal-clock (idle) PLL: HSI (16 MHz) × 21 / 2 = 168 MHz
// ---------------------------------------------------------------------------
pub const SYSCLK_IDLE_HZ: u32 = 168_000_000;
pub const PLL_N_IDLE: u32 = 21; // PLLM=1, PLLN=21, PLLR=2 → 168 MHz

// ---------------------------------------------------------------------------
// Si5351 clock generator
// ---------------------------------------------------------------------------

/// Si5351A I2C address (default for Adafruit breakout, ADDR pin low).
pub const SI5351_I2C_ADDR: u8 = 0x60;

/// Reference crystal on the Adafruit Si5351 board.
pub const SI5351_XTAL_HZ: u32 = 25_000_000;

// ---------------------------------------------------------------------------
// Audio / DSP pipeline
// ---------------------------------------------------------------------------

/// ADC sampling rate (driven by HRTIM Timer-C at ~200 kHz).
pub const AUDIO_RATE_HZ: u32 = 200_000;

/// DSP processing rate after 10× decimation.
pub const DSP_RATE_HZ: u32 = 20_000;

/// Decimation factor.
pub const DECIMATE: u32 = AUDIO_RATE_HZ / DSP_RATE_HZ;

/// DMA frame duration (µs).  Half-transfer interrupt fires every FRAME_US/2.
pub const FRAME_US: u32 = 500; // 0.5 ms

/// Number of audio samples per DMA frame (= one half of the double buffer).
pub const FRAME_SAMPLES: usize = (AUDIO_RATE_HZ / (1_000_000 / FRAME_US)) as usize; // 100

/// HRTIM Timer-C period (ticks) for the ADC-trigger at ~200 kHz.
/// Configured in CubeMX as 850; recalculated here for documentation.
/// At SYSCLK ≈ 169.5 MHz: 169_492_500 / 850 ≈ 199_403 Hz ≈ 200 kHz.
pub const TIMERC_PERIOD: u32 = 850;

// ---------------------------------------------------------------------------
// Sanity-check assertions (evaluated at compile time)
// ---------------------------------------------------------------------------
const _: () = {
    assert!(PWM_PERIOD % 4 == 0, "PWM_PERIOD must be a multiple of 4");
    assert!(FRAME_SAMPLES > 0, "FRAME_SAMPLES must be positive");
    assert!(PLL_N_TX >= 16, "PLLN must be >= 8 for the STM32G474 PLL");
    assert!(PLL_N_TX <= 127, "PLLN must be <= 127 for the STM32G474 PLL");
};
