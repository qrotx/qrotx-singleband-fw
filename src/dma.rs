// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// dma.rs — DMA1 channel configuration for ADC and HRTIM burst DMA.
//
// DMA1_CH1  ADC1_DR → ADC_BUF        periph→mem  halfword  circular  HT+TC ISR
// DMA1_CH5  HRTIM_BUF → HRTIM1.BDMADR  mem→periph  word      circular  no ISR
//
// DMAMUX1 request IDs (RM0440 Table 91):
//   ADC1        = 5
//   HRTIM1_TIMC = 96  (Timer C REP event; DIER.REPDE=1 enabled in hrtim::init)
//
// Call order:
//   hrtim::init() → adc::init() → dma::init() → adc::start()

use core::sync::atomic::Ordering;

use embassy_stm32::pac;
use embassy_stm32::pac::bdma::vals::{Dir, Pl, Size};

use crate::config::FRAME_SAMPLES;
use crate::hrtim::{adc_buf_ptr, hrtim_buf_ptr, PROCESS_FIRST_HALF, PROCESS_SECOND_HALF};

/// Total number of halfwords in the ADC double-buffer.
const ADC_DMA_LEN: u16 = (FRAME_SAMPLES * 2) as u16; // 200

/// Total number of words in the HRTIM double-buffer (200 PwmSamples × 5 words).
const HRTIM_DMA_LEN: u16 = (FRAME_SAMPLES * 2 * 5) as u16; // 1000

/// Configure and enable DMA1_CH1 (ADC) and DMA1_CH5 (HRTIM burst DMA).
///
/// Must be called after `hrtim::init()` and `adc::init()`, and before
/// `adc::start()` (ADSTART).
pub fn init() {
    let dma   = pac::DMA1;
    let dmamux = pac::DMAMUX1;

    // ------------------------------------------------------------------
    // DMAMUX1 routing.
    //   Channel 0 → DMA1_CH1: ADC1 request (ID 5).
    //   Channel 4 → DMA1_CH5: HRTIM1 Timer C request (ID 98).
    // ------------------------------------------------------------------
    dmamux.ccr(0).write(|w| w.set_dmareq_id(5));
    dmamux.ccr(4).write(|w| w.set_dmareq_id(98));

    // ------------------------------------------------------------------
    // DMA1_CH1 — ADC1_DR → ADC_BUF
    //   Direction : peripheral → memory
    //   Size      : halfword (u16) on both sides
    //   Circular  : yes (continuous ping-pong)
    //   HTIE/TCIE : yes (half-transfer and transfer-complete ISRs)
    //   TEIE      : yes (transfer error → logged in ISR)
    //   Priority  : high
    // ------------------------------------------------------------------
    let ch1 = dma.ch(0);

    // Addresses and count must be set before enabling.
    // Safety: PAC register writes are unsafe only in that they affect hardware;
    // the addresses are valid static buffers and the ADC DR address is fixed.
    ch1.par().write_value(pac::ADC1.dr().as_ptr() as u32);
    ch1.mar().write_value(adc_buf_ptr() as u32);
    ch1.ndtr().write(|w| w.set_ndt(ADC_DMA_LEN));

    ch1.cr().write(|w| {
        w.set_dir(Dir::FROM_PERIPHERAL);
        w.set_circ(true);
        w.set_pinc(false);
        w.set_minc(true);
        w.set_psize(Size::BITS16);
        w.set_msize(Size::BITS16);
        w.set_htie(true);
        w.set_tcie(true);
        w.set_teie(true);
        w.set_pl(Pl::HIGH);
        w.set_en(true);
    });

    // ------------------------------------------------------------------
    // DMA1_CH5 — HRTIM_BUF → HRTIM1.BDMADR
    //   Direction : memory → peripheral
    //   Size      : word (u32) on both sides
    //   Circular  : yes
    //   No ISR    : timing driven by the ADC side (CH1 ISR)
    //   Priority  : high
    // ------------------------------------------------------------------
    let ch5 = dma.ch(4);

    ch5.par().write_value(pac::HRTIM1.bdmadr().as_ptr() as u32);
    ch5.mar().write_value(hrtim_buf_ptr() as u32);
    ch5.ndtr().write(|w| w.set_ndt(HRTIM_DMA_LEN));

    ch5.cr().write(|w| {
        w.set_dir(Dir::FROM_MEMORY);
        w.set_circ(true);
        w.set_pinc(false);
        w.set_minc(true);
        w.set_psize(Size::BITS32);
        w.set_msize(Size::BITS32);
        w.set_htie(false);
        w.set_tcie(false);
        w.set_teie(false);
        w.set_pl(Pl::HIGH);
        w.set_en(true);
    });

    defmt::info!("dma: CH1 (ADC) and CH5 (HRTIM) enabled");
}

// ---------------------------------------------------------------------------
// DMA1_CH1 interrupt — half-transfer and transfer-complete ping-pong signals
// ---------------------------------------------------------------------------

/// DMA1 Channel 1 ISR: fires on half-transfer (HT) and transfer-complete (TC).
///
/// Clears all CH1 flags via the global interrupt clear (GIF) then sets the
/// appropriate atomic flag for the DSP task to consume.
/// DMA1 Channel 1 interrupt handler.
///
/// Overrides `PROVIDE(DMA1_CHANNEL1 = DefaultHandler)` in device.x.
/// Using `#[no_mangle] extern "C"` rather than `#[cortex_m_rt::interrupt]`
/// because stm32-metapac does not expose the `interrupt` module in a form
/// that the cortex_m_rt proc-macro can resolve at compile time.
#[no_mangle]
pub unsafe extern "C" fn DMA1_CHANNEL1() {
    let isr = pac::DMA1.isr().read();
    // Clear all CH1 flags (GIF clears GIF, TCIF, HTIF, TEIF simultaneously).
    pac::DMA1.ifcr().write(|w| w.set_gif(0, true));

    if isr.teif(0) {
        defmt::error!("dma: ADC DMA transfer error — check buffer alignment and NDTR");
    }
    if isr.htif(0) {
        // First half of ADC_BUF complete; first half of HRTIM_BUF ready to fill.
        PROCESS_FIRST_HALF.store(true, Ordering::Release);
    }
    if isr.tcif(0) {
        // Second half of ADC_BUF complete; second half of HRTIM_BUF ready to fill.
        PROCESS_SECOND_HALF.store(true, Ordering::Release);
    }
}
