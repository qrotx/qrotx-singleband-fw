// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// adc.rs — ADC1 initialisation for audio acquisition.
//
// Hardware (from CubeMX):
//   ADC1, channel 1 (PA0), 12-bit, right-aligned.
//   External trigger: HRTIM_TRG1 (Timer-C period event at ~200 kHz).
//   DMA: DMA1_CH1, peripheral→memory, halfword (u16), circular.
//   Overrun policy: overwrite (ADC_OVR_DATA_OVERWRITTEN).
//   Sample time: 92.5 cycles (adequate for the audio band-limited signal).
//
// The DMA double-buffer lives in hrtim.rs (ADC_BUF) because both ADC and
// HRTIM share the same CCMRAM block and processing context.  This module
// owns the peripheral-side setup only.
//
// Processing pipeline overview:
//
//   ADC1 → DMA1_CH1 → ADC_BUF[200] (double-buffer in CCMRAM)
//                              │
//              half-transfer ISR │ full-transfer ISR
//              (process [0..100])│ (process [100..200])
//                              ▼
//                     Downsample 10:1 (200 kHz → 20 kHz)
//                     (FIR anti-alias, then decimate)
//                              ▼
//                     SSB modulator (DSP task, 20 kHz rate)
//                     Uses CORDIC for sin/cos, FMAC for FIR
//                              ▼
//                     Upsample 10:1 (20 kHz → 200 kHz)
//                              ▼
//                     HRTIM_BUF → DMA1_CH5 → HRTIM Timer-C CMP1

use defmt::{debug, info};

// ---------------------------------------------------------------------------
// ADC1 initialisation stub
// ---------------------------------------------------------------------------

/// Initialise ADC1 for continuous triggered acquisition.
///
/// Steps (to be implemented via PAC writes or STM32G4 HAL):
///   1. Enable ADC1 clock on AHB2.
///   2. Calibrate ADC (single-ended).
///   3. Configure channel 1 (PA0): 92.5 cycles sample time.
///   4. Set external trigger: HRTIM_TRG1, rising edge.
///   5. Set DMA: continuous requests enabled.
///   6. Enable ADC.
///   7. Configure DMA1_CH1: ADC1_DR → ADC_BUF, u16, circular, length=200.
///   8. Enable DMA half-transfer and transfer-complete interrupts.
///   9. Start ADC conversion (ADSTART).
pub fn init() {
    // TODO: PAC register writes.
    //
    // Outline using embassy_stm32::pac:
    //
    // let rcc  = embassy_stm32::pac::RCC;
    // let adc1 = embassy_stm32::pac::ADC1;
    // let dma1 = embassy_stm32::pac::DMA1;
    //
    // // 1. Enable clocks.
    // rcc.ahb2enr().modify(|w| w.set_adc12en(true));
    //
    // // 2. Exit deep-power-down and enable internal regulator.
    // adc1.cr().modify(|w| { w.set_deeppwd(false); w.set_advregen(true); });
    // // Wait TADCVREG_STUP (20 µs).
    //
    // // 3. Calibrate.
    // adc1.cr().modify(|w| w.set_adcal(true));
    // while adc1.cr().read().adcal() { cortex_m::asm::nop(); }
    //
    // // 4–6. Configure and enable.
    // adc1.cfgr().write(|w| {
    //     w.set_res(0b00);       // 12-bit
    //     w.set_exten(0b01);     // rising edge trigger
    //     w.set_extsel(0b01100); // HRTIM_TRG1 (check RM0440 Table 161)
    //     w.set_dmaen(true);
    //     w.set_dmacfg(true);    // circular DMA
    //     w.set_ovrmod(true);    // overwrite on overrun
    // });
    // // Set sample time for channel 1 (SQ1 = 1, SQR1.L = 0).
    // adc1.sqr1().write(|w| { w.set_sq1(1); w.set_l(0); });
    // adc1.smpr1().modify(|w| w.set_smp1(0b110)); // 92.5 cycles
    //
    // // 7. DMA1_CH1 for ADC.
    // // (Embassy HAL or direct DMAMUX + DMA register writes.)
    //
    // adc1.cr().modify(|w| w.set_aden(true));
    // while !adc1.isr().read().adrdy() { cortex_m::asm::nop(); }
    // adc1.cr().modify(|w| w.set_adstart(true));

    info!("adc: init stub — full register programming TODO");
}

/// Stop ADC conversions and DMA (call before switching clocks or going idle).
pub fn stop() {
    // TODO: ADSTP, disable DMA, disable ADC.
    debug!("adc: stopped (stub)");
}

// ---------------------------------------------------------------------------
// DSP pipeline helpers (stubs for the 200 kHz → 20 kHz → 200 kHz chain)
// ---------------------------------------------------------------------------

/// Process one 100-sample frame of raw ADC data.
///
/// Called from the DMA half/full-transfer handler.
/// `raw` : slice of `FRAME_SAMPLES` u16 ADC samples (200 kHz rate).
/// Returns: nothing yet — output will be written to the HRTIM buffer.
///
/// Future implementation:
///   1. Apply DC-blocking IIR filter.
///   2. Decimate 10:1 with FIR anti-alias (→ 20 kHz, 10 samples).
///   3. Compute SSB I/Q via Hilbert transform (FMAC or software).
///   4. CORDIC sin/cos for phase → outphasing CMP1/CMP2 values.
///   5. Interpolate 10:1 back to 200 kHz (→ 100 samples).
///   6. Write to HRTIM DMA buffer half.
pub fn process_frame(_raw: &[u16]) {
    // TODO: DSP chain.
}
