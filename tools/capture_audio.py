#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2025-2026 Christian Riesch, christian@riesch.at
#
# capture_audio.py — Capture raw audio frames from the firmware via LPUART1.
#
# The firmware must be built with DEBUG_UART_AUDIO = true in src/config.rs.
# Each 500 µs frame produces 10 signed 16-bit little-endian samples (20 bytes)
# representing the audio signal after 10:1 FIR downsampling to 20 kHz.
#
# Hardware: PA2 = LPUART1_TX on Nucleo-G474RE (ST-Link VCP), 4 Mbaud 8N1.
#
# Usage:
#   python tools/capture_audio.py COM3             # Windows
#   python tools/capture_audio.py /dev/ttyACM0     # Linux
#   python tools/capture_audio.py COM3 -o my.bin   # custom output file
#
# Load the captured file in Python/NumPy:
#   import numpy as np
#   samples = np.fromfile('audio.bin', dtype='<i2')   # signed 16-bit LE

import argparse
import sys
import threading

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed.  Run:  pip install pyserial")
    sys.exit(1)

_stop_event = threading.Event()


def _esc_listener():
    """Background thread: set stop flag on ESC key press (Windows only)."""
    try:
        import msvcrt
        while not _stop_event.is_set():
            if msvcrt.kbhit() and msvcrt.getch() == b'\x1b':
                print("\nESC pressed. Stopping capture...")
                _stop_event.set()
    except ImportError:
        pass  # Not Windows; rely on Ctrl+C (KeyboardInterrupt)


def main():
    parser = argparse.ArgumentParser(
        description="Capture raw audio frames from the firmware via LPUART1."
    )
    parser.add_argument("port",
                        help="Serial port (e.g. COM3 on Windows, /dev/ttyACM0 on Linux)")
    parser.add_argument("-o", "--output", default="audio.bin",
                        help="Output binary file (default: audio.bin)")
    parser.add_argument("-b", "--baud", type=int, default=4_000_000,
                        help="Baud rate (default: 4000000)")
    args = parser.parse_args()

    threading.Thread(target=_esc_listener, daemon=True).start()

    ser = None
    try:
        ser = serial.Serial(
            args.port, args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        print(f"Opened {args.port} at {args.baud} baud (8N1)")
        print(f"Writing data to {args.output}")
        print("Press Ctrl+C (or ESC on Windows) to stop...")

        bytes_written = 0
        with open(args.output, "wb") as f:
            while not _stop_event.is_set():
                n = ser.in_waiting
                if n > 0:
                    data = ser.read(n)
                    f.write(data)
                    f.flush()
                    bytes_written += len(data)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        _stop_event.set()
    finally:
        if ser is not None and ser.is_open:
            ser.close()

    samples = bytes_written // 2
    duration_s = samples / 20_000
    print(f"Captured {bytes_written} bytes ({samples} samples, {duration_s:.1f} s at 20 kHz)")
    print(f"Data saved to {args.output}")
    print(f"Load with: numpy.fromfile('{args.output}', dtype='<i2')")


if __name__ == "__main__":
    main()
