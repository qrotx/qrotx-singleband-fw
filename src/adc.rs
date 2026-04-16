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

use defmt::info;

// ---------------------------------------------------------------------------
// ADC1 initialisation
// ---------------------------------------------------------------------------

/// Initialise ADC1 for triggered acquisition at 200 kHz.
///
/// Steps per RM0440 §21.4.7:
///   1. Enable ADC12 clock on AHB2; set synchronous clock = HCLK/4.
///   2. Enable voltage regulator (ADVREGEN); wait T_ADCVREG_STUP = 20 µs.
///   3. Run single-ended calibration; poll until complete.
///   4. Configure CFGR: 12-bit right-aligned, HRTIM_TRG1 rising edge,
///      DMA circular, overwrite on overrun.
///   5. Set channel 1 sample time = 92.5 cycles.
///   6. Set conversion sequence: 1 conversion, channel 1 (PA0).
///   7. Enable ADC; wait ADRDY.
///
/// DMA1_CH1 (ADC1_DR → ADC_BUF) is configured separately.
/// Call `start()` after DMA is running to begin conversions.
pub fn init() {
    use embassy_stm32::pac;
    use embassy_stm32::pac::adc::vals::{Dmacfg, Dmaen, Exten, Ovrmod, Res, SampleTime};
    use embassy_stm32::pac::adccommon::vals::Ckmode;
    use embassy_stm32::pac::gpio::vals::Moder;

    let rcc  = pac::RCC;
    let adc1 = pac::ADC1;

    // ------------------------------------------------------------------
    // 0. PA0 → ANALOG mode (ADC1_IN1).
    //    GPIOA clock already enabled by embassy_stm32::init().
    // ------------------------------------------------------------------
    pac::GPIOA.moder().modify(|w| w.set_moder(0, Moder::ANALOG));

    // ------------------------------------------------------------------
    // 1. Enable ADC12 peripheral clock on AHB2.
    //    Synchronous clock mode HCLK/4 must be set before ADEN=1.
    // ------------------------------------------------------------------
    rcc.ahb2enr().modify(|w| w.set_adc12en(true));
    let _ = rcc.ahb2enr().read(); // flush

    pac::ADC12_COMMON.ccr().modify(|w| w.set_ckmode(Ckmode::SYNC_DIV4));

    // ------------------------------------------------------------------
    // 2. Exit deep power-down, enable voltage regulator.
    //    T_ADCVREG_STUP = 20 µs; at 170 MHz → 3400 cycles.
    // ------------------------------------------------------------------
    adc1.cr().modify(|w| {
        w.set_deeppwd(false);
        w.set_advregen(true);
    });
    cortex_m::asm::delay(3_400);

    // ------------------------------------------------------------------
    // 3. Single-ended calibration.  Must start with ADEN=0, ADCALDIF=0.
    // ------------------------------------------------------------------
    adc1.cr().modify(|w| w.set_adcal(true));
    let mut timeout = 200_000u32;
    while adc1.cr().read().adcal() {
        cortex_m::asm::nop();
        timeout -= 1;
        if timeout == 0 {
            defmt::error!("adc: calibration timeout");
            break;
        }
    }

    // ------------------------------------------------------------------
    // 4. Configuration register.
    //    EXTSEL = 24 (0x18) = HRTIM1_ADCTRG1  (RM0440 Table 161).
    // ------------------------------------------------------------------
    adc1.cfgr().write(|w| {
        w.set_res(Res::BITS12);
        w.set_align(false);         // right-aligned
        w.set_extsel(21);           // HRTIM1_ADCTRG1
        w.set_exten(Exten::RISING_EDGE);
        w.set_dmaen(Dmaen::ENABLE);
        w.set_dmacfg(Dmacfg::CIRCULAR);
        w.set_ovrmod(Ovrmod::OVERWRITE);
        w.set_cont(false);          // triggered, not continuous
        w.set_discen(false);
    });

    // ------------------------------------------------------------------
    // 5. Sample time: channel 1 = 92.5 ADC clock cycles.
    //    smpr() covers channels 0-9; index = channel number.
    // ------------------------------------------------------------------
    adc1.smpr().modify(|w| w.set_smp(1, SampleTime::CYCLES92_5));

    // ------------------------------------------------------------------
    // 6. Conversion sequence: 1 conversion, rank 1 = channel 1 (PA0).
    // ------------------------------------------------------------------
    adc1.sqr1().write(|w| {
        w.set_l(0);    // 1 conversion (L = number_of_conversions - 1)
        w.set_sq(0, 1); // rank 1 = channel 1
    });

    // ------------------------------------------------------------------
    // 7. Enable ADC; wait for ADRDY.
    // ------------------------------------------------------------------
    adc1.cr().modify(|w| w.set_aden(true));
    let mut timeout = 200_000u32;
    while !adc1.isr().read().adrdy() {
        cortex_m::asm::nop();
        timeout -= 1;
        if timeout == 0 {
            defmt::error!("adc: ADRDY timeout");
            break;
        }
    }

    info!("adc: init done — waiting for DMA before ADSTART");
}

/// Start ADC conversions (ADSTART).  Call after DMA1_CH1 is configured
/// and running, so the first trigger does not overflow the FIFO.
pub fn start() {
    embassy_stm32::pac::ADC1.cr().modify(|w| w.set_adstart(true));
    defmt::debug!("adc: conversions started");
}


