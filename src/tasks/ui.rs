// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// tasks/ui.rs — user-interface task.
//
// Currently handles:
//   • PTT button (PB0, active-low with internal pull-up).
//   • User LED (PA5) as TX indicator.
//
// Future additions (not yet wired up):
//   • Display driver (SPI/I2C LCD or OLED).
//   • Rotary encoder for frequency selection.
//   • Band/mode switch inputs.

use defmt::{debug, info};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

use crate::config::TX_FREQ_HZ;
use crate::transmitter::{TxCommand, TX_CMD};

/// Embassy task: handles PTT, LED, and (future) display/controls.
#[embassy_executor::task]
pub async fn ui_task(
    mut ptt: ExtiInput<'static, Async>,
    mut led: Output<'static>,
) {
    info!("ui: task started");

    loop {
        // ---- Wait for PTT press (active-low: wait for falling edge) --------
        ptt.wait_for_falling_edge().await;
        debug!("ui: PTT pressed");

        // Debounce
        Timer::after(Duration::from_millis(20)).await;
        if ptt.is_high() {
            // Bounced — ignore.
            continue;
        }

        // Send StartTx command to the radio task.
        led.set_high(); // LED on = transmitting
        TX_CMD
            .send(TxCommand::StartTx { freq_hz: TX_FREQ_HZ })
            .await;

        // ---- Wait for PTT release (rising edge) ----------------------------
        ptt.wait_for_rising_edge().await;
        debug!("ui: PTT released");

        // Debounce
        Timer::after(Duration::from_millis(20)).await;

        // Send StopTx.
        TX_CMD.send(TxCommand::StopTx).await;
        led.set_low();

        // Brief pause before accepting the next PTT press.
        Timer::after(Duration::from_millis(50)).await;
    }
}
