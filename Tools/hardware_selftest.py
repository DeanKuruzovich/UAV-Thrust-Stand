#!/usr/bin/env python3
"""
hardware_selftest.py — host-side reader for the PCB self-test firmware.

This is the laptop side of the wiring check. It opens the ESP32's USB-serial
port and streams the diagnostic report produced by `src/selftest.cpp`, with
PASS/WARN/FAIL lines colour-highlighted in the terminal.

It does NOT talk to any sensor directly — a host PC has no access to the
board's I2C/analog pins. The ESP32 reads every component and reports over USB;
this script just displays (and optionally logs) that report.

Usage:
    # 1) flash the self-test firmware to the board:
    pio run -e selftest -t upload
    # 2) then run this reader (auto-detects the port):
    python3 Tools/hardware_selftest.py
    python3 Tools/hardware_selftest.py --port /dev/tty.usbmodem1101
    python3 Tools/hardware_selftest.py --save report.txt
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not installed. Run:  pip install -r Tools/requirements.txt")

BAUD = 921600

# ANSI colours for highlighting tags the firmware emits as plain text.
RESET = "\033[0m"
COLORS = {
    "PASS": "\033[32m",   # green
    "WARN": "\033[33m",   # yellow
    "FAIL": "\033[31m",   # red
}


def find_port(explicit):
    """Return the serial device path, auto-detecting an ESP32 if not given."""
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    # Prefer the ESP32-S3 native USB CDC (Espressif VID 0x303A).
    for p in ports:
        if p.vid == 0x303A:
            return p.device
    # Otherwise match obvious USB-serial adapters, but never grab the ADALM-2000
    # (Analog Devices VID 0x0456), which also enumerates as a usbmodem.
    keywords = ("usbmodem", "usbserial", "wchusbserial", "slab", "cp210", "esp")
    for p in ports:
        if p.vid == 0x0456:
            continue
        hay = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(k in hay for k in keywords):
            return p.device
    for p in ports:
        if p.vid != 0x0456:
            return p.device
    return None


def colorize(line):
    """Wrap a recognised [TAG] in colour; pass other lines through unchanged."""
    for tag, col in COLORS.items():
        token = f"[{tag}]"
        if token in line:
            return line.replace(token, f"[{col}{tag}{RESET}]")
    return line


def main():
    ap = argparse.ArgumentParser(description="Stream the ESP32 PCB self-test report.")
    ap.add_argument("--port", help="serial device (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=BAUD, help=f"baud rate (default {BAUD})")
    ap.add_argument("--save", metavar="FILE", help="also write raw output to FILE")
    args = ap.parse_args()

    port = find_port(args.port)
    if not port:
        sys.exit("No serial ports found. Is the board plugged in? Use --port to set one.")

    print(f"Connecting to {port} @ {args.baud} ...  (Ctrl-C to quit)")
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}")

    # Toggling DTR/RTS resets many ESP32 boards so we catch the boot banner.
    try:
        ser.dtr = False
        ser.rts = False
        time.sleep(0.1)
        ser.reset_input_buffer()
    except Exception:
        pass

    logf = open(args.save, "w") if args.save else None
    counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            for tag in counts:
                if f"[{tag}]" in line:
                    counts[tag] += 1
            print(colorize(line))
            if logf:
                logf.write(line + "\n")
                logf.flush()
    except KeyboardInterrupt:
        print(f"\n\nSummary so far:  "
              f"{COLORS['PASS']}{counts['PASS']} PASS{RESET}  "
              f"{COLORS['WARN']}{counts['WARN']} WARN{RESET}  "
              f"{COLORS['FAIL']}{counts['FAIL']} FAIL{RESET}")
    finally:
        ser.close()
        if logf:
            logf.close()


if __name__ == "__main__":
    main()
