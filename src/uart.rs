// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// uart.rs — LPUART1 initialisation and blocking TX for audio debug streaming.
//
// Hardware (Nucleo-G474RE):
//   PA2 → LPUART1_TX (AF12) — connected to the ST-Link VCP.
//   PA3 ← LPUART1_RX (AF12) — not used; only TX is initialised here.
//
// Baud rate: 4 Mbaud (8N1).
//   BRR = PCLK1 × 256 / baud = 168 000 000 × 256 / 4 000 000 = 10 752.
//   PCLK1 = SYSCLK = 168 MHz (APB1 prescaler = DIV1).
//
// Call init() once from main() when DEBUG_UART_AUDIO is true, before the
// DSP task starts.  transmit() is then safe to call from any context.

use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::{Moder, Ospeedr};

/// LPUART1 baud rate register value for 4 Mbaud at PCLK1 = 168 MHz.
/// BRR = PCLK1 × 256 / baud = 168_000_000 × 256 / 4_000_000 = 10_752.
const BRR: u16 = 10_752;

/// Initialise LPUART1 for 4 Mbaud 8N1 TX-only operation.
///
/// Enables the LPUART1 APB1 clock, configures PA2 as LPUART1_TX (AF12),
/// and enables the transmitter.  Call once before the first transmit().
pub fn init() {
    let rcc   = pac::RCC;
    let gpioa = pac::GPIOA;
    let uart  = pac::LPUART1;

    // Enable LPUART1 clock (APB1ENR2 bit 0).
    rcc.apb1enr2().modify(|w| w.set_lpuart1en(true));
    let _ = rcc.apb1enr2().read(); // flush write through bus matrix

    // PA2 → LPUART1_TX: AF12, very-high speed, push-pull, no pull.
    gpioa.moder().modify(|w| w.set_moder(2, Moder::ALTERNATE));
    gpioa.afr(0).modify(|w| w.set_afr(2, 12)); // AFRL, pin 2
    gpioa.ospeedr().modify(|w| w.set_ospeedr(2, Ospeedr::VERY_HIGH_SPEED));

    // 4 Mbaud, 8N1, TX only.
    uart.brr().write(|w| w.set_brr(BRR));
    uart.cr1().write(|w| {
        w.set_ue(true); // UART enable
        w.set_te(true); // TX enable
    });
}

/// Blocking byte-at-a-time transmit.
///
/// Spins on TXE for each byte; returns after the last byte has been
/// accepted into the shift register (not necessarily shifted out).
/// Safe to call from any context including interrupt handlers.
///
/// # Safety
/// `init()` must have been called before the first call to `transmit()`.
pub unsafe fn transmit(data: &[u8]) {
    let uart = pac::LPUART1;
    for &byte in data {
        while !uart.isr().read().txe() {}
        uart.tdr().write(|w| w.set_dr(byte as u16));
    }
}
