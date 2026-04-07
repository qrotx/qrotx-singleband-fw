// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// tasks/radio.rs — radio task: manages the Si5351, clock switching, HRTIM,
//                  ADC, and the DSP pipeline.

use defmt::{error, info, warn};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

use crate::clock;
use crate::config::{SI5351_OUTPUT_HZ, SI5351_XTAL_HZ, TX_FREQ_HZ};
use crate::hrtim;
use crate::si5351::{ClkOut, Pll, Si5351};
use crate::transmitter::{TxCommand, TxState, TX_CMD};

/// Embassy task: owns the I2C bus, Si5351, HRTIM, and ADC.
/// Receives `TxCommand` from the UI task via `TX_CMD`.
#[embassy_executor::task]
pub async fn radio_task(
    i2c: I2c<'static, Async, Master>,
    tx_enable: Output<'static>,
) {
    info!("radio: task started");

    let mut si5351 = Si5351::new(i2c, SI5351_XTAL_HZ);
    let mut tx_enable = tx_enable;

    // Initialise Si5351 (outputs disabled, crystal load cap set).
    if let Err(_e) = si5351.init().await {
        error!("radio: Si5351 init failed");
    }

    // Initialise HRTIM (outputs disabled, timers configured but not running RF).
    hrtim::init();

    loop {
        // Wait for a command from the UI task.
        let cmd = TX_CMD.receive().await;
        info!("radio: received command {:?}", cmd);

        match cmd {
            TxCommand::StartTx { freq_hz } => {
                start_transmit(&mut si5351, &mut tx_enable, freq_hz).await;
            }
            TxCommand::StopTx => {
                stop_transmit(&mut si5351, &mut tx_enable).await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Sequence: start transmission
// ---------------------------------------------------------------------------

async fn start_transmit(
    si5351: &mut Si5351<'_>,
    tx_enable: &mut Output<'_>,
    freq_hz: u32,
) {
    // Use config constants; warn if a different frequency is requested.
    if freq_hz != TX_FREQ_HZ {
        warn!(
            "radio: requested {} Hz but compiled for {} Hz — using compiled value",
            freq_hz, TX_FREQ_HZ
        );
    }

    // ----- Step 1: Configure Si5351 ----------------------------------------
    TxState::set(TxState::ConfiguringClock);
    info!(
        "radio: configuring Si5351 for {} Hz (Si5351 output = {} Hz)",
        TX_FREQ_HZ, SI5351_OUTPUT_HZ
    );

    if let Err(_e) = si5351
        .set_frequency(ClkOut::Clk0, Pll::A, SI5351_OUTPUT_HZ)
        .await
    {
        error!("radio: Si5351 frequency set failed");
        TxState::set(TxState::Idle);
        return;
    }

    // ----- Step 2: Enable Si5351 output and wait for PLL lock ---------------
    TxState::set(TxState::WaitingForLock);
    if let Err(_e) = si5351.enable_output(ClkOut::Clk0).await {
        error!("radio: Si5351 output enable failed");
        TxState::set(TxState::Idle);
        return;
    }

    // Allow the Si5351 PLL to settle (~10 ms is more than enough).
    Timer::after(Duration::from_millis(10)).await;
    info!("radio: Si5351 PLL settled");

    // ----- Step 3: Switch MCU clock to external (Si5351) --------------------
    TxState::set(TxState::SwitchingClock);
    match clock::switch_to_external_clock().await {
        Ok(()) => info!("radio: MCU running on external clock"),
        Err(e) => {
            error!("radio: clock switch failed: {:?}", e);
            // Abort: disable Si5351 output and go back to idle.
            let _ = si5351.disable_output(ClkOut::Clk0).await;
            TxState::set(TxState::Idle);
            return;
        }
    }

    // ----- Step 4: Enable RF outputs ----------------------------------------
    hrtim::enable_outputs();
    tx_enable.set_high(); // assert TX_ENABLE (PB2) to enable PA bias etc.

    TxState::set(TxState::Transmitting);
    info!("radio: transmitting on {} Hz", TX_FREQ_HZ);
}

// ---------------------------------------------------------------------------
// Sequence: stop transmission
// ---------------------------------------------------------------------------

async fn stop_transmit(si5351: &mut Si5351<'_>, tx_enable: &mut Output<'_>) {
    TxState::set(TxState::ShuttingDown);
    info!("radio: stopping transmission");

    // 1. Disable RF outputs immediately (MOSFETs off).
    tx_enable.set_low();
    hrtim::disable_outputs();

    // 2. Switch MCU back to internal clock.
    if let Err(e) = clock::switch_to_internal_clock().await {
        error!("radio: clock revert failed: {:?}", e);
    }

    // 3. Disable Si5351 output (saves power).
    if let Err(_e) = si5351.disable_output(ClkOut::Clk0).await {
        error!("radio: Si5351 disable failed");
    }

    TxState::set(TxState::Idle);
    info!("radio: idle");
}
