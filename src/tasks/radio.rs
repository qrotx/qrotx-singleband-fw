// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// tasks/radio.rs — radio task: manages the Si5351, clock switching, HRTIM,
//                  ADC, DMA, and the DSP pipeline.
//
// Startup sequence (one-time, at task launch):
//   hrtim::init()  — configure timers, burst DMA routing
//   adc::init()    — calibrate ADC1, configure channel 1
//   dma::init()    — enable DMA1_CH1 (ADC) and DMA1_CH5 (HRTIM burst)
//   adc::start()   — ADSTART: ADC conversions begin at 200 kHz
//   si5351::init() — set crystal load cap, leave outputs disabled
//
// TX start sequence (on every PTT press):
//   1. Configure Si5351 PLL for SI5351_OUTPUT_HZ.
//   2. Enable Si5351 CLK0 output; wait ~10 ms for PLL lock.
//   3. Switch MCU PLL to external clock (Si5351 reference → ~169.5 MHz SYSCLK).
//   4. Assert TX_ENABLE (relay coil) and wait RELAY_SETTLE_MS for contacts to close.
//   5. Enable HRTIM RF outputs → RF appears at the antenna port.
//
// TX stop sequence (on PTT release):
//   1. Disable HRTIM RF outputs immediately (MOSFETs off, no more RF).
//   2. Wait RELAY_RELEASE_MS to let the PA drain before relay opens.
//   3. De-assert TX_ENABLE (relay releases).
//   4. Switch MCU PLL back to internal (HSI).
//   5. Disable Si5351 CLK0 output.

use defmt::{error, info, warn};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{I2c, Master};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Timer};

use crate::adc;
use crate::clock;
use crate::config::{SI5351_OUTPUT_HZ, SI5351_XTAL_HZ, TX_FREQ_HZ};
use crate::dma;
use crate::hrtim;
use crate::si5351::{ClkOut, Pll, Si5351};
use crate::transmitter::{TxCommand, TxState, TX_CMD};

/// Mechanical relay settling time after asserting TX_ENABLE.
/// The relay must have fully closed before RF is allowed onto the antenna.
const RELAY_SETTLE_MS: u64 = 25;

/// Delay between disabling RF outputs and releasing the relay.
/// Gives the PA time to drain before the relay contacts open under load.
const RELAY_RELEASE_MS: u64 = 10;

/// Embassy task: owns the I2C bus, Si5351, HRTIM, ADC, and DMA.
/// Receives `TxCommand` from the UI task via `TX_CMD`.
#[embassy_executor::task]
pub async fn radio_task(
    i2c: I2c<'static, Async, Master>,
    tx_enable: Output<'static>,
) {
    info!("radio: task started");

    let mut si5351 = Si5351::new(i2c, SI5351_XTAL_HZ);
    let mut tx_enable = tx_enable;

    // ------------------------------------------------------------------
    // One-time hardware initialisation.
    // Order matters: HRTIM timers must be running before DMA starts,
    // and DMA must be enabled before ADC conversions begin.
    // ------------------------------------------------------------------
    hrtim::init();
    adc::init();
    dma::init();
    crate::dsp::init();
    adc::start();

    // Si5351: set crystal load cap, leave all outputs disabled for now.
    if let Err(_e) = si5351.init().await {
        error!("radio: Si5351 init failed");
    }

    info!("radio: hardware init complete — waiting for PTT");

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
    if freq_hz != TX_FREQ_HZ {
        warn!(
            "radio: requested {} Hz but compiled for {} Hz — using compiled value",
            freq_hz, TX_FREQ_HZ
        );
    }

    // ----- Step 1: Configure Si5351 PLL ------------------------------------
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
    Timer::after(Duration::from_millis(10)).await;
    info!("radio: Si5351 PLL settled");

    // ----- Step 3: Switch MCU clock to external (Si5351 reference) ----------
    TxState::set(TxState::SwitchingClock);
    match clock::switch_to_external_clock().await {
        Ok(()) => info!("radio: MCU running on external clock"),
        Err(e) => {
            error!("radio: clock switch failed: {:?}", e);
            let _ = si5351.disable_output(ClkOut::Clk0).await;
            TxState::set(TxState::Idle);
            return;
        }
    }

    // ----- Step 4: Assert relay, wait for contacts to close ----------------
    // RF must NOT be enabled until the relay has fully closed.
    tx_enable.set_high();
    Timer::after(Duration::from_millis(RELAY_SETTLE_MS)).await;
    info!("radio: relay closed");

    // ----- Step 5: Enable HRTIM RF outputs ----------------------------------
    hrtim::enable_outputs();

    TxState::set(TxState::Transmitting);
    info!("radio: transmitting on {} Hz", TX_FREQ_HZ);
}

// ---------------------------------------------------------------------------
// Sequence: stop transmission
// ---------------------------------------------------------------------------

async fn stop_transmit(si5351: &mut Si5351<'_>, tx_enable: &mut Output<'_>) {
    TxState::set(TxState::ShuttingDown);
    info!("radio: stopping transmission");

    // 1. Kill RF immediately — HRTIM outputs off, MOSFETs go inactive.
    hrtim::disable_outputs();

    // 2. Brief wait: let the PA drain before relay contacts open under load.
    Timer::after(Duration::from_millis(RELAY_RELEASE_MS)).await;

    // 3. Release relay.
    tx_enable.set_low();
    info!("radio: relay released");

    // 4. Switch MCU back to internal clock.
    if let Err(e) = clock::switch_to_internal_clock().await {
        error!("radio: clock revert failed: {:?}", e);
    }

    // 5. Disable Si5351 output (saves power).
    if let Err(_e) = si5351.disable_output(ClkOut::Clk0).await {
        error!("radio: Si5351 disable failed");
    }

    TxState::set(TxState::Idle);
    info!("radio: idle");
}
