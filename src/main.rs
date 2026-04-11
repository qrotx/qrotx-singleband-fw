// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// main.rs — qrotx single-band shortwave transmitter firmware.
//
// Processor : STM32G474RETx on Nucleo-G474RE
// Target    : thumbv7em-none-eabihf  (Cortex-M4F, hard float)
// Toolchain : stable Rust + probe-rs (flash/debug via on-board STLINK-V3)
//
// Clock plan
// ──────────
//   Idle : HSI 16 MHz → PLL (M=1, N=21, R=2) → SYSCLK 168 MHz
//   TX   : Si5351 CLK0 (~8 MHz) → PLL (M=1, N=42, R=2) → SYSCLK ~169.5 MHz
//          Exact SYSCLK derived in config.rs from TX_FREQ_HZ so that
//          HRTIM PWM period × HRTIM_HR_FACTOR = SYSCLK / TX_FREQ_HZ exactly.
//
// Uses embassy_executor::Executor (requires executor-thread + platform-cortex-m
// features) which provides a run() → ! that WFE-sleeps between tasks.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;   // RTT transport for defmt
use panic_probe as _; // defmt panic handler

use embassy_executor::Executor;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Config as I2cConfig, I2c};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Pll, PllMul, PllPreDiv, PllRDiv, PllSource, Sysclk,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::dma as embassy_dma;
use embassy_stm32::{bind_interrupts, exti, i2c, interrupt, peripherals, Config};
use static_cell::StaticCell;

mod adc;
mod clock;
mod config;
mod dma;
mod dsp;
mod dsp_ffi;
mod hrtim;
mod si5351;
mod tasks;
mod transmitter;

use tasks::dsp::dsp_task;
use tasks::radio::radio_task;
use tasks::ui::ui_task;

// ---------------------------------------------------------------------------
// Interrupt bindings — maps IRQ lines to embassy async handlers.
// ---------------------------------------------------------------------------
bind_interrupts!(struct Irqs {
    // I2C1 event and error interrupts (async I2C for Si5351).
    I2C1_EV      => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER      => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    // DMA1 channel interrupts for I2C1 TX (CH6) and RX (CH7).
    // On STM32G474 the interrupt typelevel names use _CHANNEL suffix.
    DMA1_CHANNEL6 => embassy_dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => embassy_dma::InterruptHandler<peripherals::DMA1_CH7>;
    // EXTI0 for PTT button (PB0).
    EXTI0        => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
});

// ---------------------------------------------------------------------------
// Static executor — lives for the duration of the program.
// ---------------------------------------------------------------------------
static EXECUTOR: StaticCell<Executor> = StaticCell::new();

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[cortex_m_rt::entry]
fn main() -> ! {
    // -----------------------------------------------------------------------
    // 1. Clock configuration — HSI → PLL → 168 MHz (idle / startup)
    //
    //    HSI (16 MHz) / M=1 = 16 MHz VCO input.
    //    × N=21 = 336 MHz VCO.
    //    / R=2  = 168 MHz SYSCLK.
    //
    //    Before transmitting, clock.rs switches to the Si5351 external
    //    reference with PLLN=42, PLLR=2 to achieve ~169.5 MHz SYSCLK.
    // -----------------------------------------------------------------------
    let mut rcc = Config::default();
    {
        let r = &mut rcc.rcc;
        r.hsi = true;
        r.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,    // PLLM = 1
            mul:    PllMul::MUL21,      // PLLN = 21
            divp:   None,
            divq:   None,
            divr:   Some(PllRDiv::DIV2), // PLLR = 2  →  168 MHz
        });
        r.sys      = Sysclk::PLL1_R;
        r.ahb_pre  = AHBPrescaler::DIV1;
        r.apb1_pre = APBPrescaler::DIV1;
        r.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(rcc);

    info!("qrotx singleband transmitter — firmware starting");
    info!("TX freq  : {} Hz",      config::TX_FREQ_HZ);
    info!("PWM per  : {} ticks",   config::PWM_PERIOD);
    info!("SYSCLK TX: {} Hz",      config::SYSCLK_TX_HZ);
    info!("Si5351   : {} Hz",      config::SI5351_OUTPUT_HZ);

    // -----------------------------------------------------------------------
    // 2. GPIO
    //   PA5 — User LED (LD2, active-high)
    //   PB0 — PTT button (active-low, pull-up)
    //   PB2 — TX_ENABLE output (active-high, drives PA bias / relay)
    // -----------------------------------------------------------------------
    let led       = Output::new(p.PA5, Level::Low, Speed::Low);
    let tx_enable = Output::new(p.PB2, Level::Low, Speed::Low);
    let ptt_pin   = ExtiInput::new(p.PB0, p.EXTI0, Pull::Up, Irqs);

    // -----------------------------------------------------------------------
    // 3. I2C1 — Si5351 clock generator (PB8 = SCL, PB9 = SDA, 400 kHz)
    //
    //   I2c::new() argument order: peri, scl, sda, tx_dma, rx_dma, irq, config.
    //   Config is #[non_exhaustive] — use Default then set frequency field.
    // -----------------------------------------------------------------------
    let mut i2c_cfg = I2cConfig::default();
    i2c_cfg.frequency = Hertz(400_000);

    let i2c = I2c::new(
        p.I2C1,
        p.PB8,       // SCL (AF4)
        p.PB9,       // SDA (AF4)
        p.DMA1_CH6,  // I2C1 TX DMA
        p.DMA1_CH7,  // I2C1 RX DMA
        Irqs,
        i2c_cfg,
    );

    // -----------------------------------------------------------------------
    // 4. Start the Embassy executor and spawn tasks.
    //    Executor::run() never returns (loops with WFE) → satisfies fn -> !
    // -----------------------------------------------------------------------
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(dsp_task().unwrap());
        spawner.spawn(radio_task(i2c, tx_enable).unwrap());
        spawner.spawn(ui_task(ptt_pin, led).unwrap());
    });
}
