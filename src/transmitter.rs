// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// transmitter.rs — transmitter state machine and inter-task command channel.
//
// State diagram:
//
//   Idle ──(StartTx)──► ConfiguringClock
//                              │
//                    Si5351 programmed & output enabled
//                              │
//                       WaitingForLock
//                              │
//                    MCU PLL switched to external clock
//                              │
//                       SwitchingClock
//                              │
//                    HRTIM outputs enabled, ADC running
//                              │
//                          Transmitting ◄────────────────────┐
//                              │                             │
//                         (StopTx)                    (continuous)
//                              │
//                        ShuttingDown
//                              │
//                    HRTIM disabled, ADC stopped,
//                    MCU clock back to internal, Si5351 off
//                              │
//                            Idle

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// Commands that can be sent to the radio task.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum TxCommand {
    /// Start transmitting on `freq_hz`.
    StartTx { freq_hz: u32 },
    /// Stop transmitting and return to idle.
    StopTx,
}

/// Capacity of the command channel (a few slots is plenty).
const CMD_CHANNEL_DEPTH: usize = 4;

/// Channel for sending `TxCommand` from the UI task (or main) to the radio task.
pub static TX_CMD: Channel<ThreadModeRawMutex, TxCommand, CMD_CHANNEL_DEPTH> =
    Channel::new();

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TxState {
    Idle,
    ConfiguringClock,
    WaitingForLock,
    SwitchingClock,
    Transmitting,
    ShuttingDown,
}

/// Shared transmitter state (readable by the UI task for display purposes).
///
/// Written only by the radio task.  Read by any task.
/// Protected by an atomic so no mutex is needed for simple reads.
static TX_STATE: core::sync::atomic::AtomicU8 =
    core::sync::atomic::AtomicU8::new(TxState::Idle as u8);

impl TxState {
    /// Map the raw atomic byte back to the enum.
    pub fn current() -> Self {
        match core::sync::atomic::AtomicU8::load(
            &TX_STATE,
            core::sync::atomic::Ordering::Relaxed,
        ) {
            0 => TxState::Idle,
            1 => TxState::ConfiguringClock,
            2 => TxState::WaitingForLock,
            3 => TxState::SwitchingClock,
            4 => TxState::Transmitting,
            5 => TxState::ShuttingDown,
            _ => TxState::Idle,
        }
    }

    pub(crate) fn set(s: TxState) {
        core::sync::atomic::AtomicU8::store(
            &TX_STATE,
            s as u8,
            core::sync::atomic::Ordering::Relaxed,
        );
    }
}
