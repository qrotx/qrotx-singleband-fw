// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// clock.rs — runtime clock switching between internal HSI and external Si5351.
//
// Embassy configures the clocks once at startup (internal HSI → 168 MHz).
// Before transmitting we need to:
//   1. Configure Si5351 to output SI5351_OUTPUT_HZ on CLK0.
//   2. Switch the STM32 PLL source to HSE-bypass (PF0, Si5351 clock).
//   3. Adjust PLLN so SYSCLK = SI5351_OUTPUT_HZ × PLL_NET_MUL.
// After transmitting the reverse takes place.
//
// All register writes go through the raw PAC (embassy_stm32::pac) because
// Embassy does not support runtime clock source changes via its HAL.
//
// PAC register access is safe in stm32-metapac ≥ 21 — no unsafe{} needed.

use defmt::{debug, error};
use embassy_stm32::pac;
use embassy_time::{Duration, Instant, Timer};

use crate::config::{PLL_N_IDLE, PLL_N_TX};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Switch the system clock to use the external Si5351 reference.
/// Must be called after Si5351 output is enabled and PLL is settled.
pub async fn switch_to_external_clock() -> Result<(), ClockError> {
    debug!("clock: switching to external (Si5351) reference");

    select_sysclk_source(SysclkSrc::Hsi)?;
    disable_pll()?;
    enable_hse_bypass()?;
    configure_pll(PllSource::Hse, 1, PLL_N_TX as u8, 2)?;
    enable_pll_and_wait().await?;
    select_sysclk_source(SysclkSrc::Pll)?;

    debug!("clock: external clock active");
    Ok(())
}

/// Switch back to the internal HSI reference (call after stopping TX).
pub async fn switch_to_internal_clock() -> Result<(), ClockError> {
    debug!("clock: switching back to internal HSI");

    select_sysclk_source(SysclkSrc::Hsi)?;
    disable_pll()?;
    disable_hse()?;
    configure_pll(PllSource::Hsi, 1, PLL_N_IDLE as u8, 2)?;
    enable_pll_and_wait().await?;
    select_sysclk_source(SysclkSrc::Pll)?;

    debug!("clock: internal clock active");
    Ok(())
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

#[derive(Debug, defmt::Format)]
pub enum ClockError {
    Timeout,
    InvalidConfig,
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

enum SysclkSrc { Hsi, Pll }
enum PllSource  { Hsi, Hse }

/// Move SYSCLK to `src` and wait for hardware confirmation.
fn select_sysclk_source(src: SysclkSrc) -> Result<(), ClockError> {
    // SW[1:0] in RCC_CFGR: 01=HSI16, 11=PLLRCLK  (STM32G4 RM0440 §6.4.3)
    let sw_val: u8 = match src {
        SysclkSrc::Hsi => 0x01,
        SysclkSrc::Pll => 0x03,
    };

    let rcc = pac::RCC;
    rcc.cfgr().modify(|w| w.set_sw(sw_val.into()));

    // Poll SWS until it mirrors SW.
    let deadline = Instant::now() + Duration::from_millis(10);
    loop {
        let sws = rcc.cfgr().read().sws();
        if sws == sw_val.into() {
            return Ok(());
        }
        if Instant::now() > deadline {
            error!("clock: SYSCLK switch timeout");
            return Err(ClockError::Timeout);
        }
        cortex_m::asm::nop();
    }
}

fn disable_pll() -> Result<(), ClockError> {
    let rcc = pac::RCC;
    rcc.cr().modify(|w| w.set_pllon(false));

    let deadline = Instant::now() + Duration::from_millis(5);
    loop {
        if !rcc.cr().read().pllrdy() {
            return Ok(());
        }
        if Instant::now() > deadline {
            return Err(ClockError::Timeout);
        }
        cortex_m::asm::nop();
    }
}

/// Enable HSE in bypass mode (PF0 receives an external clock signal, not a crystal).
fn enable_hse_bypass() -> Result<(), ClockError> {
    let rcc = pac::RCC;
    rcc.cr().modify(|w| {
        w.set_hsebyp(true);
        w.set_hseon(true);
    });

    let deadline = Instant::now() + Duration::from_millis(50);
    loop {
        if rcc.cr().read().hserdy() {
            return Ok(());
        }
        if Instant::now() > deadline {
            error!("clock: HSE ready timeout");
            return Err(ClockError::Timeout);
        }
        cortex_m::asm::nop();
    }
}

fn disable_hse() -> Result<(), ClockError> {
    let rcc = pac::RCC;
    rcc.cr().modify(|w| {
        w.set_hseon(false);
        w.set_hsebyp(false);
    });
    Ok(())
}

/// Configure PLL registers (PLL must be disabled before calling).
///
/// STM32G474 PLL constraints (RM0440 §6.4):
///   PLLM: 1–16     VCO_in = PLL_source / PLLM  ∈ [2.66, 16] MHz
///   PLLN: 8–127    VCO    = VCO_in × PLLN       ∈ [96, 344]  MHz
///   PLLR: 2,4,6,8  SYSCLK = VCO / PLLR          ≤ 170 MHz
///
/// Register encoding:
///   PLLM field = M − 1      (0 means divide by 1)
///   PLLN field = N           (direct value)
///   PLLR field = R/2 − 1    (0=÷2, 1=÷4, 2=÷6, 3=÷8)
///   PLLSRC     = 2 (HSI16), 3 (HSE)
fn configure_pll(src: PllSource, pllm: u8, plln: u8, pllr: u8) -> Result<(), ClockError> {
    if pllm < 1 || plln < 8 || plln > 127 {
        return Err(ClockError::InvalidConfig);
    }
    if pllr != 2 && pllr != 4 && pllr != 6 && pllr != 8 {
        return Err(ClockError::InvalidConfig);
    }

    // PLLSRC: binary 10 = HSI16, binary 11 = HSE
    let pllsrc_val: u8 = match src {
        PllSource::Hsi => 0x02,
        PllSource::Hse => 0x03,
    };

    let rcc = pac::RCC;
    rcc.pllcfgr().write(|w| {
        w.set_pllsrc(pllsrc_val.into());
        w.set_pllm((pllm - 1).into());           // register = M − 1
        w.set_plln(plln.into());                  // register = N directly
        w.set_pllren(true);                       // enable PLLR output
        w.set_pllr(((pllr / 2) - 1).into());     // 0=÷2, 1=÷4, 2=÷6, 3=÷8
    });
    Ok(())
}

async fn enable_pll_and_wait() -> Result<(), ClockError> {
    let rcc = pac::RCC;
    rcc.cr().modify(|w| w.set_pllon(true));

    let deadline = Instant::now() + Duration::from_millis(10);
    loop {
        if rcc.cr().read().pllrdy() {
            return Ok(());
        }
        if Instant::now() > deadline {
            error!("clock: PLL lock timeout");
            return Err(ClockError::Timeout);
        }
        Timer::after(Duration::from_micros(10)).await;
    }
}
