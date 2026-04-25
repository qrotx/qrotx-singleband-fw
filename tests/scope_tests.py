#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
#
# scope_tests.py — Host-side PWM output verification via Rigol DS1054Z.
#
# Flashes src/bin/scope_test.rs onto the Nucleo, then coordinates scope
# measurements at each firmware state marker printed over RTT/defmt.
#
# Requirements:
#   pip install pyvisa pyvisa-py
#   (On Windows, also install libusb via Zadig for the scope USB interface.)
#
# Usage:
#   python tests/scope_tests.py             # build, flash, run
#   python tests/scope_tests.py --no-build  # skip cargo build
#   python tests/scope_tests.py --scope "USB0::0x1AB1::0x04CE::DS1ZA..."
#
# Probe connections:
#   CH1 → PA8  (Timer A output 1 — phase-modulated RF, positive)
#   CH2 → PA9  (Timer A output 2 — dead-time complement of PA8)
#   CH3 → PB12 (Timer C output 1 — buck converter / amplitude envelope)
#   CH4 → PB13 (Timer C output 2 — dead-time complement of PB12)
#   GND → any board GND pin

import argparse
import queue
import re
import subprocess
import sys
import threading
import time

try:
    import pyvisa
except ImportError:
    print("ERROR: pyvisa not installed.  Run:  pip install pyvisa pyvisa-py")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CHIP            = "STM32G474RETx"
BINARY          = "target/thumbv7em-none-eabihf/release/scope_test"
SYSCLK_HZ       = 168_000_000   # idle clock (HSI PLL, scope_test always uses this)
HRTIM_HR_FACTOR = 32            # HRTIM DLL high-resolution multiplier (×32 on STM32G474)
# Timer A (MUL32 prescaler, CKPSC=0): fCOUNT = SYSCLK × 32 → RF frequency ≈ TX_FREQ_HZ
# Timer C (DIV1  prescaler, CKPSC=5): fCOUNT = SYSCLK      → ~200 kHz buck / ADC trigger

FREQ_TOL    = 0.005          # ±0.5 % frequency tolerance
VPP_MIN     = 2.8            # V  — minimum acceptable logic high swing
VPP_MAX     = 3.6            # V  — maximum (3.3 V + headroom)
PHASE_TOL   = 10.0           # degrees
DUTY_TOL    = 3.0            # percent

# ---------------------------------------------------------------------------
# Scope wrapper
# ---------------------------------------------------------------------------

class Scope:
    """Rigol DS1054Z SCPI interface via pyvisa."""

    def __init__(self, resource_str=None):
        rm = pyvisa.ResourceManager()

        if resource_str is None:
            resources = rm.list_resources()
            # Look for Rigol by USB vendor ID (0x1AB1) or name fragments.
            candidates = [
                r for r in resources
                if "0x1AB1" in r or "DS1Z" in r or "RIGOL" in r.upper()
            ]
            if not candidates:
                usb = [r for r in resources if r.startswith("USB")]
                candidates = usb[:1]
            if not candidates:
                print(f"Available VISA resources: {resources}")
                raise RuntimeError(
                    "Rigol DS1054Z not found.  Connect scope via USB and "
                    "ensure the USBTMC driver is installed (see Zadig on Windows)."
                )
            resource_str = candidates[0]

        print(f"Connecting to scope: {resource_str}")
        self.inst = rm.open_resource(resource_str)
        self.inst.timeout = 8_000  # ms
        idn = self.inst.query("*IDN?").strip()
        print(f"Scope IDN: {idn}")

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------

    def prepare(self):
        """
        Reset the instrument and configure all four channels and trigger mode.
        Does NOT set the timebase — call set_timebase() once the firmware has
        broadcast its frequency constants.

        Call this before flashing the firmware so the ~3 s RST + *OPC? overlaps
        with the probe-rs flash time rather than eating into the hold window.
        """
        self.inst.write("*RST")
        self.inst.query("*OPC?")  # block until RST is truly complete
        self.inst.write("*CLS")

        # Channels 1–4: DC coupling, 10x probe, 1 V/div.
        # The signal swings 0–3.3 V; offset −1.65 V centres it in the display.
        for ch in range(1, 5):
            self.inst.write(f":CHANnel{ch}:DISPlay ON")
            self.inst.write(f":CHANnel{ch}:COUPling DC")
            self.inst.write(f":CHANnel{ch}:PROBe 1")
            self.inst.write(f":CHANnel{ch}:SCALe 5")        # 1 V/div
            self.inst.write(f":CHANnel{ch}:OFFSet {ch * 5 - 15}")   # arrange channels
            self.inst.write(f":CHANnel{ch}:BWLimit OFF")     # full bandwidth
            self.inst.write(f":CHANnel{ch}:INVert OFF")

        # Trigger mode: source and level are set in set_timebase() because they
        # depend on which timer / channel is being measured.
        self.inst.write(":TRIGger:MODE EDGE")
        self.inst.write(":TRIGger:EDGe:SLOPe POSitive")
        self.inst.write(":TRIGger:SWEep NORMal")
        self.inst.query("*OPC?")
        time.sleep(5)

        print("Scope prepared: channels configured, awaiting timebase")

    def set_timebase(self, freq_hz: float, trigger_ch: int = 1):
        """
        Set timebase and trigger source, then start acquisition.
        Uses *OPC? to block until the scope is fully settled before returning.

        Call once after prepare() with the Timer A frequency from the firmware
        constants.  Call again between test stages when switching between
        Timer A (~7 MHz RF, CH1/CH2) and Timer C (~200 kHz buck, CH3/CH4).
        """
        period_s    = 1.0 / freq_hz
        # Timebase: show approximately 8 complete periods.
        # 8 × T / 12 horizontal divs → T/1.5 per div.
        time_per_div = period_s / 1.5
        self.inst.write(f":TIMebase:SCALe {time_per_div:.3e}")
        self.inst.write(":TIMebase:OFFSet 0")
        self.inst.write(":TIMebase:MODE MAIN")
        self.inst.write(f":TRIGger:EDGe:SOURce CHANnel{trigger_ch}")
        self.inst.write(":TRIGger:EDGe:LEVel 1.65")
        self.inst.write(":RUN")
        self.inst.query("*OPC?")
        time.sleep(5)
        print(f"  Scope timebase → {freq_hz/1e3:.2f} kHz, "
              f"{time_per_div*1e6:.3f} µs/div, trigger CH{trigger_ch}")

    # ------------------------------------------------------------------
    # Measurements
    # ------------------------------------------------------------------

    def _query_float(self, cmd) -> float | None:
        try:
            val = float(self.inst.query(cmd).strip())
            # DS1054Z returns 9.9E+37 for "no valid measurement"
            return None if val > 1e30 else val
        except (ValueError, pyvisa.VisaIOError):
            return None

    def frequency(self, channel: int) -> float | None:
        """Frequency in Hz, measured by the hardware frequency counter.
        The counter returns 0.0 when disabled or no valid signal is present."""
        self.inst.write(f":MEASure:COUNter:SOURce CHANnel{channel}")
        time.sleep(0.5)   # let the counter accumulate at least one gate window
        val = self._query_float(":MEASure:COUNter:VALue?")
        return val if (val is not None and val > 0) else None

    def vpp(self, channel: int) -> float | None:
        """Peak-to-peak voltage in V."""
        return self._query_float(f":MEASure:ITEM? VPP,CHANnel{channel}")

    def duty_cycle(self, channel: int) -> float | None:
        """Positive duty cycle in %."""
        return self._query_float(f":MEASure:ITEM? PDUTy,CHANnel{channel}")

    def phase(self, source: int, ref: int) -> float | None:
        """Phase of *source* relative to *ref* in degrees (DS1054Z range: −180 … +180)."""
        return self._query_float(f":MEASure:ITEM? RPHase,CHANnel{source},CHANnel{ref}")

    def wait_triggered(self, timeout_s: float = 5.0) -> bool:
        """Return True once the scope reports a triggered (TD) state."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                status = self.inst.query(":TRIGger:STATus?").strip().upper()
                if "TD" in status:
                    return True
            except pyvisa.VisaIOError:
                pass
            time.sleep(0.2)
        return False


# ---------------------------------------------------------------------------
# Firmware runner — launches probe-rs and streams RTT output
# ---------------------------------------------------------------------------

class FirmwareRunner:
    """Flashes scope_test.rs and captures defmt/RTT output via probe-rs."""

    def __init__(self, binary: str = BINARY, chip: str = CHIP):
        self.binary  = binary
        self.chip    = chip
        self.proc    = None
        self._q      = queue.Queue()
        self._thread = None
        self.constants: dict = {}

    def build(self):
        print("Building scope_test binary...")
        r = subprocess.run(
            ["cargo", "build", "--bin", "scope_test", "--release"],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            print("Build FAILED:")
            print(r.stderr)
            sys.exit(1)
        print("Build OK")

    def start(self):
        print(f"Flashing {self.binary} …")
        self.proc = subprocess.Popen(
            ["probe-rs", "run", "--chip", self.chip, self.binary],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,   # merge so we see all output
            stdin=subprocess.DEVNULL,   # don't let probe-rs steal keystrokes
            text=True,
            bufsize=1,
            start_new_session=True,     # detach from controlling terminal so
                                        # probe-rs cannot open /dev/tty
        )
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        for raw_line in self.proc.stdout:
            line = raw_line.rstrip()
            print(f"  MCU: {line}")
            self._q.put(line)

            # Parse compile-time constants broadcast by the firmware.
            m = re.search(
                r"SCOPE CONSTANTS "
                r"PWM_PERIOD=(\d+) TIMERC_PERIOD=(\d+) "
                r"AMPLITUDE=(\d+) FRAME_SAMPLES=(\d+)",
                line,
            )
            if m:
                self.constants = {
                    "PWM_PERIOD":    int(m.group(1)),
                    "TIMERC_PERIOD": int(m.group(2)),
                    "AMPLITUDE":     int(m.group(3)),
                    "FRAME_SAMPLES": int(m.group(4)),
                }

        self._q.put(None)   # sentinel: process has exited

    def wait_state(self, n: int, timeout_s: float = 35.0) -> dict | None:
        """
        Block until 'SCOPE STATE N READY …' appears in RTT output.
        Returns a dict of parsed parameters (ta_cmp1, tc_cmp1) or None on timeout.
        """
        marker = f"SCOPE STATE {n} READY"
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                line = self._q.get(timeout=1.0)
            except queue.Empty:
                continue
            if line is None:
                return None     # firmware exited unexpectedly
            if marker in line:
                params = {}
                for key in ("ta_cmp1", "tc_cmp1"):
                    m = re.search(rf"{key}=(\d+)", line)
                    if m:
                        params[key] = int(m.group(1))
                return params
        return None

    def wait_done(self, timeout_s: float = 8.0):
        """Drain until 'SCOPE DONE' or process exit."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                line = self._q.get(timeout=1.0)
            except queue.Empty:
                continue
            if line is None or (line and "SCOPE DONE" in line):
                return

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()


# ---------------------------------------------------------------------------
# Test results
# ---------------------------------------------------------------------------

_results: list[tuple[str, bool, str]] = []

PASS_STR = "\033[32mPASS\033[0m"
FAIL_STR = "\033[31mFAIL\033[0m"


def check(name: str, ok: bool, detail: str = "") -> bool:
    tag = PASS_STR if ok else FAIL_STR
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")
    _results.append((name, ok, detail))
    return ok


def _step_pause(step: bool, label: str = ""):
    """If step mode is active, print a prompt and wait for Enter.
    Reads from /dev/tty directly so that stdin redirection or probe-rs
    terminal manipulation cannot cause a spurious early return."""
    if step:
        suffix = f" ({label})" if label else ""
        sys.stdout.write(f"  [step]{suffix} Press Enter to measure ...")
        sys.stdout.flush()
        with open("/dev/tty") as tty:
            tty.readline()


# ---------------------------------------------------------------------------
# Individual test stages
# ---------------------------------------------------------------------------

def test_state1_frequency(scope: Scope, fw: FirmwareRunner,
                           timer_a_hz: float, timerc_hz: float,
                           step: bool = False):
    """State 1 — all outputs at nominal settings.
    Check frequency on all four channels.
    CH1/CH2 (Timer A, RF ~7 MHz) and CH3/CH4 (Timer C, ~200 kHz buck) need
    separate scope timebase settings."""
    print("\n=== State 1: frequency on all channels ===")
    params = fw.wait_state(1)
    if params is None:
        check("State 1 reached", False, "timeout or firmware exit")
        return
    check("State 1 reached", True)

    # Wait until the scope has captured at least one triggered waveform before
    # querying measurements.  A fixed sleep is unreliable: the DS1054Z returns
    # 9.9E+37 for any measurement query until a triggered acquisition completes.
    if not scope.wait_triggered(timeout_s=5.0):
        print("  WARNING: scope did not trigger on CH1 within 5 s")
    time.sleep(0.5)     # let the measurement algorithm run one more cycle
    _step_pause(step, "CH1/CH2 Timer A ~7 MHz")

    # CH1/CH2 — Timer A (RF, high-resolution prescaler MUL32)
    for ch in (1, 2):
        freq = scope.frequency(ch)
        if freq is not None:
            err = abs(freq - timer_a_hz) / timer_a_hz
            check(f"CH{ch} frequency",
                  err < FREQ_TOL,
                  f"{freq:.0f} Hz  expected {timer_a_hz:.0f} Hz  err={err*100:.2f}%")
        else:
            check(f"CH{ch} frequency", False, "no measurement — check probe connection")

    # Reconfigure scope for Timer C (DIV1 prescaler, ~200 kHz buck converter)
    scope.set_timebase(timerc_hz, trigger_ch=3)
    if not scope.wait_triggered(timeout_s=5.0):
        print("  WARNING: scope did not trigger on CH3 within 5 s")
    time.sleep(0.5)
    _step_pause(step, "CH3/CH4 Timer C ~200 kHz")

    # CH3/CH4 — Timer C (buck converter / ADC trigger, normal prescaler DIV1)
    for ch in (3, 4):
        freq = scope.frequency(ch)
        if freq is not None:
            err = abs(freq - timerc_hz) / timerc_hz
            check(f"CH{ch} frequency",
                  err < FREQ_TOL,
                  f"{freq:.0f} Hz  expected {timerc_hz:.0f} Hz  err={err*100:.2f}%")
        else:
            check(f"CH{ch} frequency", False, "no measurement — check probe connection")


def test_state2_tc_low_duty(scope: Scope, fw: FirmwareRunner,
                             timerc_period: int, timerc_hz: float,
                             step: bool = False):
    """State 2 — Timer C at low amplitude (AMPLITUDE/4).
    Check PB12 (CH3) duty cycle ≈ tc_cmp1 / TIMERC_PERIOD × 100 %."""
    print("\n=== State 2: Timer C low amplitude duty cycle ===")
    params = fw.wait_state(2)
    if params is None:
        check("State 2 reached", False, "timeout")
        return
    check("State 2 reached", True)
    scope.set_timebase(timerc_hz, trigger_ch=3)
    time.sleep(1.0)
    _step_pause(step, "CH3 Timer C low duty")

    tc_cmp1 = params.get("tc_cmp1", 0)
    expected_duty = tc_cmp1 / timerc_period * 100.0
    duty = scope.duty_cycle(3)   # CH3 = PB12 = Timer C output 1
    if duty is not None:
        check(f"CH3 (PB12) duty cycle ≈ {expected_duty:.1f} %",
              abs(duty - expected_duty) < DUTY_TOL,
              f"{duty:.1f} %  expected {expected_duty:.1f} %")
    else:
        check("CH3 duty cycle", False, "no measurement")


def test_state3_tc_full_duty(scope: Scope, fw: FirmwareRunner,
                              timerc_period: int, timerc_hz: float,
                              step: bool = False):
    """State 3 — Timer C at full amplitude (AMPLITUDE).
    Duty cycle should be roughly double that of State 2."""
    print("\n=== State 3: Timer C full amplitude duty cycle ===")
    params = fw.wait_state(3)
    if params is None:
        check("State 3 reached", False, "timeout")
        return
    check("State 3 reached", True)
    scope.set_timebase(timerc_hz, trigger_ch=3)
    time.sleep(1.0)
    _step_pause(step, "CH3 Timer C full duty")

    tc_cmp1 = params.get("tc_cmp1", 0)
    expected_duty = tc_cmp1 / timerc_period * 100.0
    duty = scope.duty_cycle(3)
    if duty is not None:
        check(f"CH3 (PB12) duty cycle ≈ {expected_duty:.1f} %",
              abs(duty - expected_duty) < DUTY_TOL,
              f"{duty:.1f} %  expected {expected_duty:.1f} %")
    else:
        check("CH3 duty cycle", False, "no measurement")

    # PA8/PA9 (CH1/CH2) must still be complementary throughout.
    phase_12 = scope.phase(1, 2)
    if phase_12 is not None:
        check("CH1/CH2 complementary (|phase| ≈ 180°)",
              abs(abs(phase_12) - 180.0) < PHASE_TOL,
              f"{phase_12:.1f}°")
    else:
        check("CH1/CH2 complementary", False, "no measurement")

    # Timer A (CH1) and Timer C (CH3) run at different frequencies
    # (PWM_PERIOD vs TIMERC_PERIOD), so we do not check phase between them.


def test_state4_phase_reference(scope: Scope, fw: FirmwareRunner,
                                 timer_a_hz: float,
                                 step: bool = False) -> float | None:
    """State 4 — ta_cmp1 = 0.  Record CH1 duty cycle as phase reference
    and check CH2 is complementary."""
    print("\n=== State 4: phase reference (ta_cmp1 = 0) ===")
    params = fw.wait_state(4)
    if params is None:
        check("State 4 reached", False, "timeout")
        return None
    check("State 4 reached", True)
    scope.set_timebase(timer_a_hz, trigger_ch=1)
    time.sleep(1.0)
    _step_pause(step, "CH1/CH2 phase reference ta_cmp1=0")

    phase_12 = scope.phase(1, 2)
    if phase_12 is not None:
        check("CH1/CH2 complementary (|phase| ≈ 180°)",
              abs(abs(phase_12) - 180.0) < PHASE_TOL,
              f"{phase_12:.1f}°")
    else:
        check("CH1/CH2 complementary", False, "no measurement")

    duty4 = scope.duty_cycle(1)
    if duty4 is not None:
        print(f"  CH1 duty cycle at ta_cmp1=0: {duty4:.1f}%  (reference for State 5)")
    return duty4


def test_state5_phase_step(scope: Scope, fw: FirmwareRunner,
                            pwm_period: int, duty4: float | None,
                            timer_a_hz: float, step: bool = False):
    """State 5 — ta_cmp1 = PWM_PERIOD/4.
    PA8 rising edge should shift by 90° (= 25 % of PWM period) vs State 4."""
    print("\n=== State 5: phase step (ta_cmp1 = PWM_PERIOD/4) ===")
    params = fw.wait_state(5)
    if params is None:
        check("State 5 reached", False, "timeout")
        return
    check("State 5 reached", True)
    scope.set_timebase(timer_a_hz, trigger_ch=1)
    time.sleep(1.0)
    _step_pause(step, "CH1/CH2 phase step ta_cmp1=PWM_PERIOD/4")

    phase_12 = scope.phase(1, 2)
    if phase_12 is not None:
        check("CH1/CH2 complementary (|phase| ≈ 180°)",
              abs(abs(phase_12) - 180.0) < PHASE_TOL,
              f"{phase_12:.1f}°")
    else:
        check("CH1/CH2 complementary", False, "no measurement")

    duty5 = scope.duty_cycle(1)
    if duty5 is not None:
        print(f"  CH1 duty cycle at ta_cmp1=PWM_PERIOD/4: {duty5:.1f}%")

    if duty4 is not None and duty5 is not None:
        # ta_cmp1 shifts by PWM_PERIOD/4 → duty cycle shifts by 25 %
        expected_shift = 25.0
        actual_shift   = duty5 - duty4
        check("PA8 phase step ≈ +25 % duty cycle (90°)",
              abs(actual_shift - expected_shift) < DUTY_TOL,
              f"Δduty = {actual_shift:.1f} %  expected {expected_shift:.1f} %")
    elif duty4 is None:
        check("PA8 phase step", False, "State 4 duty cycle not available")
    else:
        check("PA8 phase step", False, "State 5 duty cycle not measured")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Scope-based PWM output tests for qrotx-singleband-fw"
    )
    parser.add_argument("--no-build", action="store_true",
                        help="Skip cargo build (use existing binary)")
    parser.add_argument("--scope", metavar="RESOURCE",
                        help="VISA resource string (auto-detect if omitted)")
    parser.add_argument("--step", action="store_true",
                        help="Pause before each measurement so you can inspect "
                             "the scope screen; press Enter to continue")
    args = parser.parse_args()

    fw = FirmwareRunner()
    if not args.no_build:
        fw.build()

    scope = Scope(args.scope)

    # Reset scope and configure channels before flashing — the ~3 s RST + *OPC?
    # overlaps with the probe-rs flash time so it is done by the time the
    # firmware starts broadcasting its state markers.
    scope.prepare()

    fw.start()

    # Wait for the firmware to broadcast its compile-time constants.
    deadline = time.time() + 10.0
    while not fw.constants and time.time() < deadline:
        time.sleep(0.1)
    if not fw.constants:
        print("ERROR: firmware constants not received — is RTT working?")
        fw.stop()
        sys.exit(1)

    c             = fw.constants
    pwm_period    = c["PWM_PERIOD"]
    timerc_period = c["TIMERC_PERIOD"]
    # Timer A: MUL32 prescaler (CKPSC=0) → fCOUNT = SYSCLK × HRTIM_HR_FACTOR
    timer_a_hz    = SYSCLK_HZ * HRTIM_HR_FACTOR / pwm_period
    # Timer C: DIV1 prescaler (CKPSC=5) → fCOUNT = SYSCLK (no high-res factor)
    timerc_hz     = SYSCLK_HZ / timerc_period

    print(f"\nFirmware constants: PWM_PERIOD={pwm_period}  "
          f"TIMERC_PERIOD={timerc_period}  AMPLITUDE={c['AMPLITUDE']}")
    print(f"Expected Timer A frequency: {timer_a_hz:.0f} Hz "
          f"({timer_a_hz/1e3:.2f} kHz)  [SYSCLK × {HRTIM_HR_FACTOR} / PWM_PERIOD]")
    print(f"Expected Timer C frequency: {timerc_hz:.0f} Hz "
          f"({timerc_hz/1e3:.2f} kHz)  [SYSCLK / TIMERC_PERIOD]")

    # Set the timebase now that we know the Timer A frequency.
    # Channels are already configured from prepare() above.
    scope.set_timebase(timer_a_hz)

    try:
        test_state1_frequency(scope, fw, timer_a_hz, timerc_hz, args.step)
        test_state2_tc_low_duty(scope, fw, timerc_period, timerc_hz, args.step)
        test_state3_tc_full_duty(scope, fw, timerc_period, timerc_hz, args.step)
        duty4 = test_state4_phase_reference(scope, fw, timer_a_hz, args.step)
        test_state5_phase_step(scope, fw, pwm_period, duty4, timer_a_hz, args.step)
        fw.wait_done()
    finally:
        fw.stop()

    # Summary
    print(f"\n{'=' * 55}")
    passed = sum(1 for _, ok, _ in _results if ok)
    total  = len(_results)
    print(f"Results: {passed}/{total} passed")
    for name, ok, detail in _results:
        mark = "✓" if ok else "✗"
        suffix = f"  ({detail})" if detail else ""
        print(f"  {mark} {name}{suffix}")

    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
