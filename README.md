# qrotx-singleband-fw

Firmware for the qrotx single-band shortwave SSB transmitter, targeting the
**STM32G474RETx** Cortex-M4F microcontroller on a Nucleo-G474RE development
board.

## License

This program is free software: you can redistribute it and/or modify it under
the terms of the **GNU General Public License version 3 or later** (GPL-3.0-or-later),
as published by the Free Software Foundation.

See [LICENSE](LICENSE) or <https://www.gnu.org/licenses/> for the full text.

---

## Hardware overview

| Component | Part | Notes |
|---|---|---|
| MCU | STM32G474RETx | Cortex-M4F, 170 MHz, HRTIM, CORDIC |
| Development board | Nucleo-G474RE | On-board STLINK-V3 for flashing/debug |
| Clock generator | Si5351A (Adafruit breakout) | 25 MHz crystal, I2C at 0x60 |
| PTT button | Active-low on PB0 | Internal pull-up, EXTI interrupt |
| TX enable | PB2 | Drives PA bias relay (active-high) |
| RF outputs | PA8, PA9 (Timer A), PB12, PB13 (Timer C) | Class-D bridge via HRTIM |
| Audio input | PA0 (ADC1 channel 1) | Microphone or line input |

---

## Firmware architecture

### Clock plan

| Mode | Source | SYSCLK |
|---|---|---|
| Idle / startup | HSI 16 MHz → PLL (M=1, N=21, R=2) | 168 MHz |
| Transmitting | Si5351 CLK0 → PLL (M=1, N=42, R=2) | ~169.5 MHz |

The exact TX SYSCLK is derived from `TX_FREQ_HZ` at compile time so that the
HRTIM PWM period is always a clean multiple-of-4 tick count — see
[`src/config.rs`](src/config.rs).

### Tasks (Embassy async)

The firmware runs three Embassy tasks on a single-threaded executor:

| Task | File | Responsibility |
|---|---|---|
| `dsp_task` | `src/tasks/dsp.rs` | Polls ping-pong DMA flags and calls the DSP pipeline |
| `radio_task` | `src/tasks/radio.rs` | Owns I2C/Si5351, manages TX start/stop sequence |
| `ui_task` | `src/tasks/ui.rs` | Debounces PTT button, drives LED, sends `TxCommand` |

### TX start/stop sequence

**PTT press:**
1. Configure Si5351 PLL for the required output frequency.
2. Enable Si5351 CLK0; wait ~10 ms for PLL lock.
3. Switch MCU PLL to the Si5351 reference (SYSCLK → ~169.5 MHz).
4. Assert TX_ENABLE (relay); wait 25 ms for contacts to close.
5. Enable HRTIM RF outputs — RF appears at the antenna port.

**PTT release:**
1. Disable HRTIM RF outputs immediately.
2. Wait 10 ms for the PA to drain.
3. Release TX_ENABLE relay.
4. Switch MCU PLL back to HSI.
5. Disable Si5351 CLK0.

### DSP pipeline

The ADC samples at 200 kHz, DMA-transferred into a double-buffer.  On every
half-transfer interrupt the DSP task processes one 100-sample frame:

```
ADC u16 (200 kHz)
  │
  ▼  Stage 1  adc_to_q31()
Q31 × 100  (200 kHz)
  │
  ▼  Stage 2  FIR decimation 10:1  (60-tap Hamming lowpass, 5 kHz cutoff)
Q31 × 10   (20 kHz)
  │
  ▼  Stage 3a  Q31 → f32
f32 × 10
  │
  ▼  Stage 3b  Chebyshev I highpass  (200 Hz, order 4, 0.3 dB ripple)
  │
  ▼  Stage 3c  Voice compressor  (4:1, attack 0.2 ms, release 100 ms)
  │
  ▼  Stage 3d  Chebyshev I lowpass  (2800 Hz, order 4, 0.3 dB ripple)
  │
  ▼  Stage 4   SSB analytic FIR  (257-tap Remez, 530–9470 Hz)
f32 I × 10,  f32 Q × 10   (LSB analytic signal)
  │
  ▼  Stage 5a  f32 → Q31
Q31 I × 10,  Q31 Q × 10
  │
  ▼  Stage 5b  FIR interpolation 10:1  (same 60-tap prototype, polyphase)
Q31 I × 100,  Q31 Q × 100   (200 kHz)
  │
  ▼  Stage 5c  CORDIC MODULUS  (I, Q) → (modulus, phase)  Q1.31
  │            Outphasing: modulus → Timer C CMP1 (PA supply)
  │                        phase   → Timer A/B CMP1/CMP2 (RF phase)
  ▼
PwmSample × 100  →  HRTIM DMA buffer  →  RF output
```

The SSB filter uses a compact coefficient representation: the 257-tap analytic
filter has structurally-zero entries at alternating positions, so only 129 real
and 128 imaginary coefficients are stored.  Both paths share a single 257-sample
shift register with stride-2 access.

CORDIC and outphasing math use the STM32G474 hardware CORDIC peripheral (PAC
access, MODULUS function, 60 iterations, Q1.31).

### Module map

```
src/
  main.rs          Clock init, GPIO, I2C, task spawning
  config.rs        Compile-time frequency and timing constants
  clock.rs         Runtime PLL switching (HSI ↔ Si5351 external)
  hrtim.rs         HRTIM1 initialisation, DMA buffers, ping-pong flags
  adc.rs           ADC1 calibration and start
  dma.rs           DMA1_CH1 (ADC) and DMA1_CH5 (HRTIM) setup + ISR
  si5351.rs        Async I2C driver for the Si5351A clock generator
  transmitter.rs   TxCommand enum, TxState atomic
  dsp.rs           Firmware wrapper: CORDIC init, static filter instances
  tasks/
    dsp.rs         Embassy task: polls DMA flags, calls dsp pipeline
    radio.rs       Embassy task: TX start/stop sequencing
    ui.rs          Embassy task: PTT debounce, LED, TxCommand dispatch

dsp-core/          Pure-math DSP library (no_std, no HAL)
  src/lib.rs       FirDecimate, FirInterpolate, BiquadState, Compressor,
                   SsbFilter, coefficients, Q31/f32 converters, unit tests

tests/
  hw_tests.rs      Hardware-in-the-loop tests (defmt-test + probe-rs)

tools/
  design_decimate_filter.ipynb   FIR lowpass prototype design → FIR_COEFFS_Q31
  design_audio_filters.ipynb     Chebyshev I HP/LP design → biquad coefficients
  design_ssb_filter.ipynb        Remez SSB analytic filter design
  test_audio_chain.ipynb         WAV-file audio chain test (Python simulation)
```

---

## Building and flashing

### Prerequisites

- Rust stable toolchain with the `thumbv7em-none-eabihf` target:
  ```
  rustup target add thumbv7em-none-eabihf
  ```
- [probe-rs](https://probe.rs/) for flashing and RTT logging:
  ```
  cargo install probe-rs-tools
  ```
- Nucleo-G474RE connected via USB (STLINK-V3 on-board).

### Build

```
cargo build --release
```

### Flash and run

```
cargo run --release
```

probe-rs flashes the binary and opens an RTT console showing defmt log output.

### Change the transmit frequency

Edit `TX_FREQ_HZ` in [`src/config.rs`](src/config.rs).  All clocks, PWM periods,
and Si5351 dividers are recalculated at compile time — no other changes are needed.

---

## Testing

### Host unit tests (DSP math, no hardware required)

```
cargo test -p dsp-core --target x86_64-pc-windows-msvc
```

(Replace the target triple with your host triple on Linux/macOS:
`x86_64-unknown-linux-gnu` or `aarch64-apple-darwin`.)

13 tests cover FIR passband/stopband behaviour, IIR filter responses,
compressor gain reduction, interpolation round-trip energy, and Q31/f32
conversion accuracy.

### Hardware-in-the-loop tests (Nucleo connected)

```
cargo test --test hw_tests
```

Five tests run on the target via probe-rs:

| Test | What it verifies |
|---|---|
| `test_si5351_present` | Si5351 I2C ACK, SYS_INIT bit clear |
| `test_hrtim_timer_running` | Timer C counter advances after init |
| `test_adc_dma_fires` | DMA half-transfer ISR fires within 2 ms |
| `test_adc_nonzero_samples` | ADC buffer contains non-zero samples |
| `test_dma_rate` | ~20 DMA events per 10 ms (±5 tolerance) |

### Audio chain simulation (Python, no hardware required)

Open [`tools/test_audio_chain.ipynb`](tools/test_audio_chain.ipynb) in
Jupyter, set `INPUT_WAV` to a mono or stereo WAV file, and run all cells.
The notebook resamples to 20 kHz, runs the audio chain (HP → compressor → LP)
using the exact same coefficients as the firmware, writes `output.wav`, and
displays spectrograms for each stage.

---

## Repository

<https://github.com/qrotx/qrotx-singleband-fw>
