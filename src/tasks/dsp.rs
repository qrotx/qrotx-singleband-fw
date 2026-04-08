// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// tasks/dsp.rs — DSP task: consumes ping-pong DMA flags and runs the
//                signal processing pipeline for each half-buffer.
//
// The DMA1_CHANNEL1 ISR (in dma.rs) sets PROCESS_FIRST_HALF on half-transfer
// and PROCESS_SECOND_HALF on transfer-complete.  This task spins on those
// atomics and calls the appropriate dsp:: entry point.
//
// Priority note: this task must complete processing one half-buffer before
// the DMA wraps around and overwrites it.  At 200 kHz / 100 samples per half
// that budget is 500 µs.

use core::sync::atomic::Ordering;

use crate::dsp;
use crate::hrtim::{PROCESS_FIRST_HALF, PROCESS_SECOND_HALF};

/// Embassy task: runs the DSP pipeline in response to DMA ping-pong flags.
///
/// Loops forever, polling both atomic flags.  When a flag is set the
/// corresponding half-buffer is processed and the flag is cleared.
#[embassy_executor::task]
pub async fn dsp_task() {
    defmt::info!("dsp: task started");

    loop {
        // Yield to the executor so other tasks (radio, ui) can run between
        // processing calls.  embassy_futures::yield_now() releases the
        // executor for one scheduling round without blocking.
        embassy_futures::yield_now().await;

        if PROCESS_FIRST_HALF.load(Ordering::Acquire) {
            PROCESS_FIRST_HALF.store(false, Ordering::Release);
            // Safety: called only from this task; DMA is writing the second
            // half while we process the first.
            unsafe { dsp::process_first_half() };
        }

        if PROCESS_SECOND_HALF.load(Ordering::Acquire) {
            PROCESS_SECOND_HALF.store(false, Ordering::Release);
            // Safety: called only from this task; DMA is writing the first
            // half while we process the second.
            unsafe { dsp::process_second_half() };
        }
    }
}
