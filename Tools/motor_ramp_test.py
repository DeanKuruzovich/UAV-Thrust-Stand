"""
motor_ramp_test.py — Simple motor ramp-up test
===============================================
Ramps throttle from 0 → 100% in 10% steps (2 s each),
holds at 100% for 10 s, then cuts to idle.

Press Ctrl+C at any time for an emergency stop.

ESC CALIBRATION (run once on a new ESC):
    python Tools/motor_ramp_test.py --calibrate

    1. Disconnect the battery from the ESC.
    2. Run this script with --calibrate.
    3. When prompted, connect the battery — ESC will beep to confirm max throttle.
    4. Press Enter — ESC drops to min, beeps to confirm, and is now calibrated.
"""

import serial
import serial.tools.list_ports
import time
import sys

BAUD = 921600


def find_port():
    ports = list(serial.tools.list_ports.comports())
    # Prefer the ESP32-S3 native USB CDC (Espressif VID 0x303A).
    for p in ports:
        if p.vid == 0x303A:
            return p.device
    # Fall back to common USB-UART bridges, but never grab the ADALM-2000
    # (Analog Devices VID 0x0456), which also enumerates as a usbmodem.
    for p in ports:
        if p.vid == 0x0456:
            continue
        if any(x in (p.description + (p.hwid or "")) for x in ["USB", "ESP32", "UART", "CP210", "CH340", "FTDI"]):
            return p.device
    return None


def send_throttle(ser, pct):
    pwm = 1000 + (pct * 10)
    ser.write(f"{pwm}\n".encode())
    print(f"  Throttle: {pct:3d}%  (PWM {pwm} µs)")


def calibrate(ser):
    """
    ESC throttle-range calibration sequence.
    ESC must be powered on AFTER max throttle is sent so it latches the high point.
    """
    print("\n=== ESC CALIBRATION ===")
    print("Step 1: Sending MAX throttle (2000 µs) — keep battery DISCONNECTED.")
    ser.write(b"2000\n")
    time.sleep(0.5)

    input("Step 2: NOW connect the battery. Wait for the ESC beeps, then press Enter...")

    print("Step 3: Sending MIN throttle (1000 µs)...")
    ser.write(b"1000\n")
    time.sleep(2)

    print("Calibration done. ESC should have beeped to confirm the low point.")
    print("You can now run the ramp test without --calibrate.\n")


def main():
    port = find_port()
    if not port:
        print("ERROR: No ESP32 found. Connect the device and retry.")
        sys.exit(1)

    print(f"Connecting to {port} at {BAUD} baud...")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(1.5)  # let ESC arm

    try:
        if "--calibrate" in sys.argv:
            calibrate(ser)
            return

        print("\n--- Ramp phase: 0 → 100% (10% every 2 s) ---")
        for pct in range(0, 110, 10):
            send_throttle(ser, pct)
            time.sleep(2)

        print("\n--- Hold phase: 100% for 10 s ---")
        send_throttle(ser, 100)
        time.sleep(10)

    except KeyboardInterrupt:
        print("\n[!] Ctrl+C — emergency stop")

    finally:
        print("\n--- Stopping motor (idle) ---")
        ser.write(b"1000\n")
        time.sleep(0.2)
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
