// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
//
// si5351.rs — async driver for the Silicon Labs Si5351A clock generator.
//
// The Adafruit Si5351 breakout uses a 25 MHz crystal reference and exposes
// CLK0–CLK2 on the I2C bus at address 0x60 (ADDR pin low).
//
// References:
//   • Si5351A/B/C Data Sheet (Silicon Labs)
//   • AN619 – Manually Generating an Si5351 Register Map

use defmt::{debug, error, warn};
use embassy_stm32::i2c::{Error as I2cError, I2c, Master};
use embassy_stm32::mode::Async;

use crate::config::SI5351_I2C_ADDR;

// ---------------------------------------------------------------------------
// Register map
// ---------------------------------------------------------------------------
mod reg {
    pub const DEVICE_STATUS:    u8 = 0;
    pub const OUTPUT_ENABLE:    u8 = 3;   // 0 = enabled, 1 = disabled (per bit)
    pub const CLK0_CTRL:        u8 = 16;
    pub const CLK1_CTRL:        u8 = 17;
    pub const CLK2_CTRL:        u8 = 18;
    pub const MSNA_BASE:        u8 = 26;  // PLL A  — 8 bytes
    pub const MSNB_BASE:        u8 = 34;  // PLL B  — 8 bytes
    pub const MS0_BASE:         u8 = 42;  // CLK0 MultiSynth — 8 bytes
    pub const MS1_BASE:         u8 = 50;  // CLK1 MultiSynth
    pub const MS2_BASE:         u8 = 58;  // CLK2 MultiSynth
    pub const PLL_RESET:        u8 = 177;
    pub const XTAL_LOAD:        u8 = 183;
}

// CLK_CTRL bit fields (CLKx_CTRL register)
const CLK_POWERDOWN: u8  = 1 << 7;
const CLK_INTEGER:   u8  = 1 << 6;
const CLK_PLL_B:     u8  = 1 << 5; // 0 = PLL A
const CLK_SRC_MS:    u8  = 0b11;   // bits [1:0] = MultiSynth (local)
const CLK_DRV_8MA:   u8  = 0b11 << 0; // 8 mA drive (max)

// Crystal load capacitance: 10 pF (Adafruit default, bits [7:6] = 11).
const XTAL_LOAD_10PF: u8 = 0b11 << 6;

// PLL_RESET bit masks
const PLLA_RESET: u8 = 1 << 5;
const PLLB_RESET: u8 = 1 << 7;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Pll {
    A,
    B,
}

#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum ClkOut {
    Clk0,
    Clk1,
    Clk2,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Si5351<'d> {
    i2c:     I2c<'d, Async, Master>,
    xtal_hz: u32,
}

impl<'d> Si5351<'d> {
    pub fn new(i2c: I2c<'d, Async, Master>, xtal_hz: u32) -> Self {
        Self { i2c, xtal_hz }
    }

    // -----------------------------------------------------------------------
    // Initialisation
    // -----------------------------------------------------------------------

    /// Initialise the chip: disable all outputs, configure crystal load cap.
    pub async fn init(&mut self) -> Result<(), I2cError> {
        // Wait for SYS_INIT bit to clear (device ready after power-on).
        let deadline = embassy_time::Instant::now()
            + embassy_time::Duration::from_millis(100);
        loop {
            let status = self.read_reg(reg::DEVICE_STATUS).await?;
            if status & 0x80 == 0 {
                break;
            }
            if embassy_time::Instant::now() > deadline {
                error!("si5351: device not ready after 100 ms");
                break; // continue anyway
            }
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }

        // Disable all clock outputs (bit = 1 means disabled).
        self.write_reg(reg::OUTPUT_ENABLE, 0xFF).await?;

        // Power down all CLK drivers.
        for &r in &[reg::CLK0_CTRL, reg::CLK1_CTRL, reg::CLK2_CTRL] {
            self.write_reg(r, CLK_POWERDOWN).await?;
        }

        // Set crystal load capacitance.
        self.write_reg(reg::XTAL_LOAD, XTAL_LOAD_10PF).await?;

        debug!("si5351: initialised");
        Ok(())
    }

    // -----------------------------------------------------------------------
    // High-level frequency setter
    // -----------------------------------------------------------------------

    /// Set `output` to `target_hz` using `pll` as the VCO source.
    ///
    /// Chooses the smallest integer MultiSynth divider whose VCO falls in
    /// [600, 900] MHz; uses a fractional PLL to hit the exact target.
    pub async fn set_frequency(
        &mut self,
        output: ClkOut,
        pll: Pll,
        target_hz: u32,
    ) -> Result<(), I2cError> {
        const VCO_MIN: u64 = 600_000_000;
        const VCO_MAX: u64 = 900_000_000;

        // Find the smallest *even* integer MultiSynth divider with VCO in range.
        // Si5351 MS divider must be even (≥ 6) for integer mode.
        let ms_div_min = (VCO_MIN.div_ceil(target_hz as u64) as u32).max(6);
        let ms_div_max = (VCO_MAX / target_hz as u64) as u32;

        if ms_div_min > ms_div_max {
            error!(
                "si5351: no valid MS divider for {} Hz (min={}, max={})",
                target_hz, ms_div_min, ms_div_max
            );
            // Fall back to closest value.
        }

        // Pick the smallest even divider ≥ ms_div_min.
        let ms_div = if ms_div_min % 2 == 0 {
            ms_div_min
        } else {
            ms_div_min + 1
        };

        let vco_hz = target_hz as u64 * ms_div as u64;

        debug!(
            "si5351: target={} Hz, ms_div={}, vco={} Hz",
            target_hz, ms_div, vco_hz
        );

        // Program the PLL VCO and output divider.
        self.set_pll_vco(pll, vco_hz as u32).await?;
        self.set_ms_integer(output, pll, ms_div).await?;

        Ok(())
    }

    /// Enable a clock output (clears the disable bit in the OEB register).
    pub async fn enable_output(&mut self, output: ClkOut) -> Result<(), I2cError> {
        let bit = output as u8;
        let val = self.read_reg(reg::OUTPUT_ENABLE).await?;
        self.write_reg(reg::OUTPUT_ENABLE, val & !(1 << bit)).await
    }

    /// Disable a clock output.
    pub async fn disable_output(&mut self, output: ClkOut) -> Result<(), I2cError> {
        let bit = output as u8;
        let val = self.read_reg(reg::OUTPUT_ENABLE).await?;
        self.write_reg(reg::OUTPUT_ENABLE, val | (1 << bit)).await
    }

    // -----------------------------------------------------------------------
    // PLL programming
    // -----------------------------------------------------------------------

    /// Program a PLL to produce `vco_hz` (must be in [600, 900] MHz).
    ///
    /// Uses a fractional multiplier M = vco_hz / xtal_hz = a + b/c,
    /// with c = 1_000_000 for ≈1 Hz resolution.
    pub async fn set_pll_vco(&mut self, pll: Pll, vco_hz: u32) -> Result<(), I2cError> {
        let a = vco_hz / self.xtal_hz;
        let remainder = vco_hz - a * self.xtal_hz;
        // Fractional part: b/c ≈ remainder / xtal_hz, c = 1_000_000
        let c: u32 = 1_000_000;
        let b: u32 = (remainder as u64 * c as u64 / self.xtal_hz as u64) as u32;

        debug!(
            "si5351: PLL {:?}: a={}, b={}, c={} → vco≈{} Hz",
            pll, a, b, c, vco_hz
        );

        let regs = ms_params_to_bytes(a, b, c);
        let base = match pll {
            Pll::A => reg::MSNA_BASE,
            Pll::B => reg::MSNB_BASE,
        };
        self.write_burst(base, &regs).await?;

        // Reset the PLL after reconfiguration.
        let reset_mask = match pll {
            Pll::A => PLLA_RESET,
            Pll::B => PLLB_RESET,
        };
        self.write_reg(reg::PLL_RESET, reset_mask).await
    }

    // -----------------------------------------------------------------------
    // MultiSynth (output divider) programming — integer mode
    // -----------------------------------------------------------------------

    /// Program a MultiSynth output divider in integer mode and configure the
    /// associated CLK control register.
    ///
    /// Integer mode (b=0) produces the cleanest output spectrum.
    /// `div` must be even and ≥ 6 (Si5351 constraint).
    async fn set_ms_integer(
        &mut self,
        output: ClkOut,
        pll: Pll,
        div: u32,
    ) -> Result<(), I2cError> {
        if div < 6 || div % 2 != 0 {
            warn!("si5351: MS divider {} is invalid (must be even ≥ 6)", div);
        }

        let regs = ms_params_to_bytes(div, 0, 1);
        let ms_base = match output {
            ClkOut::Clk0 => reg::MS0_BASE,
            ClkOut::Clk1 => reg::MS1_BASE,
            ClkOut::Clk2 => reg::MS2_BASE,
        };
        self.write_burst(ms_base, &regs).await?;

        // CLKx_CTRL: integer mode | PLL source | MS driver source | 8 mA drive
        let pll_bit = match pll {
            Pll::A => 0,
            Pll::B => CLK_PLL_B,
        };
        let ctrl = CLK_INTEGER | pll_bit | (CLK_SRC_MS << 2) | CLK_DRV_8MA;
        let ctrl_reg = match output {
            ClkOut::Clk0 => reg::CLK0_CTRL,
            ClkOut::Clk1 => reg::CLK1_CTRL,
            ClkOut::Clk2 => reg::CLK2_CTRL,
        };
        self.write_reg(ctrl_reg, ctrl).await
    }

    // -----------------------------------------------------------------------
    // Low-level I2C helpers
    // -----------------------------------------------------------------------

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), I2cError> {
        self.i2c.write(SI5351_I2C_ADDR, &[reg, val]).await
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, I2cError> {
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(SI5351_I2C_ADDR, &[reg], &mut buf)
            .await?;
        Ok(buf[0])
    }

    /// Burst-write `data` starting at `start_reg` (Si5351 auto-increments).
    /// Maximum 8 bytes per call (Si5351 MultiSynth block size).
    async fn write_burst(&mut self, start_reg: u8, data: &[u8]) -> Result<(), I2cError> {
        debug_assert!(data.len() <= 8);
        // Build a single I2C frame: [reg_addr, d0, d1, ..., d7]
        let mut buf = [0u8; 9];
        buf[0] = start_reg;
        buf[1..=data.len()].copy_from_slice(data);
        self.i2c
            .write(SI5351_I2C_ADDR, &buf[..=data.len()])
            .await
    }
}

// ---------------------------------------------------------------------------
// Si5351 MultiSynth parameter encoding (AN619)
//
// For a rational multiplier  M = a + b/c  (c up to 2²⁰−1):
//
//   floor_bc = floor(128 × b / c)
//   P1 = 128 × a + floor_bc − 512
//   P2 = 128 × b − c × floor_bc
//   P3 = c
//
// The eight register bytes are:
//   [0] P3[15:8]
//   [1] P3[7:0]
//   [2] P1[17:16]  (bits [7:2] reserved/zero for MSx, bit [6] = INT_MODE)
//   [3] P1[15:8]
//   [4] P1[7:0]
//   [5] P3[19:16] | P2[19:16]
//   [6] P2[15:8]
//   [7] P2[7:0]
// ---------------------------------------------------------------------------
fn ms_params_to_bytes(a: u32, b: u32, c: u32) -> [u8; 8] {
    let floor_bc = (128 * b) / c;
    let p1: u32 = 128 * a + floor_bc - 512;
    let p2: u32 = 128 * b - c * floor_bc;
    let p3: u32 = c;

    [
        ((p3 >> 8) & 0xFF) as u8,
        (p3 & 0xFF) as u8,
        ((p1 >> 16) & 0x03) as u8,
        ((p1 >> 8) & 0xFF) as u8,
        (p1 & 0xFF) as u8,
        (((p3 >> 12) & 0xF0) | ((p2 >> 16) & 0x0F)) as u8,
        ((p2 >> 8) & 0xFF) as u8,
        (p2 & 0xFF) as u8,
    ]
}
