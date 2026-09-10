# UAV Thrust Stand — RPI DBF Propulsion Test Bench

A self-contained motor/propeller test stand for the **RPI Design-Build-Fly (DBF)** team.  An **ESP32-S3** acquires data from three HX711 load cells, an INA228 power monitor, and an RPM sensor, then streams live telemetry to a PC GUI over USB at 50 Hz.

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [ESP32-S3 Pinout](#esp32-s3-pinout)
- [Wiring Diagram Description](#wiring-diagram-description)
- [Calibration](#calibration)
- [Firmware (PlatformIO)](#firmware-platformio)
- [PC Logger GUI](#pc-logger-gui)
- [Serial Protocol](#serial-protocol)
- [Automated Test Programs](#automated-test-programs)
- [CSV Output Format](#csv-output-format)
- [Safety Features](#safety-features)
- [Repository Structure](#repository-structure)

---

## Hardware Overview

| Component | Part / Notes |
|-----------|-------------|
| Microcontroller | ESP32-S3 DevKitC-1 |
| Thrust load cell | Single-point load cell + HX711 ADC |
| Torque load cells | 2× beam load cells (push + pull) + shared HX711 SCK |
| Power monitor | Texas Instruments INA228 (I²C, 20-bit) |
| RPM sensor | Hall-effect or optical sensor → rising-edge interrupt |
| ESC interface | Standard RC ESC: 50 Hz PWM, 1000–2000 µs pulse |
| USB interface | Native USB CDC (no FTDI chip needed on S3) |

The torque arm length is **63.18 mm** from the motor shaft centerline to each load cell contact point.  Both push (CW rotation reaction) and pull (CCW) load cells are measured independently and reported as signed torque values in Newton-metres.

---

## ESP32-S3 Pinout

| GPIO | Function | Connected To | Notes |
|------|----------|-------------|-------|
| **4** | `PIN_LC_THRUST` | HX711 DOUT | Axial thrust load cell |
| **8** | `PIN_I2C_SDA` | INA228 SDA | I²C data line |
| **9** | `PIN_I2C_SCL` | INA228 SCL | I²C clock line |
| **12** | `PIN_HX_SCK` | HX711 SCK | Shared clock — all 3 HX711 modules |
| **16** | `PIN_LC_PUSH` | HX711 DOUT | Torque *push* (CW reaction) load cell |
| **17** | `PIN_LC_PULL` | HX711 DOUT | Torque *pull* (CCW reaction) load cell |
| **18** | `PIN_ESC` | ESC signal wire | 50 Hz PWM output (LEDC channel 0) |
| **35** | `PIN_RPM` | RPM sensor output | Rising-edge interrupt, internal pull-up enabled |

**INA228 I²C address:** `0x45` (ADDR pin tied to VCC).  
**Motor pole pairs:** 7 (14-pole motor).  Change `MOTOR_POLE_PAIRS` in `main.cpp` if your motor differs.

---

## Wiring Diagram Description

```
Motor shaft
   │
   ├─── Propeller
   │
   ├─── [Thrust load cell] ──────── HX711 ── DOUT → GPIO 4
   │                                        SCK  → GPIO 12
   │
   ├─── [Torque push cell] ──────── HX711 ── DOUT → GPIO 16
   │    (CW reaction arm)                   SCK  → GPIO 12 (shared)
   │
   ├─── [Torque pull cell] ──────── HX711 ── DOUT → GPIO 17
   │    (CCW reaction arm)                  SCK  → GPIO 12 (shared)
   │
   ├─── [ESC] ── Signal → GPIO 18  (PWM 50 Hz, 1000–2000 µs)
   │         └── Battery leads → [INA228 shunt] ── I²C → GPIO 8/9
   │
   └─── [RPM sensor] → GPIO 35   (rising edge per magnetic pole pass)

ESP32-S3 USB-C → PC (921600 baud, USB CDC)
```

---

## Calibration

Calibration divisors live at the top of `src/main.cpp`:

```cpp
const float CAL_THRUST     = 73.77;
const float CAL_PUSH_LEFT  = 104.680247642;
const float CAL_PULL_RIGHT = 110;
```

**To recalibrate a load cell:**

1. Flash the firmware and open the serial monitor.
2. Remove all load from the cell and send `Z` to tare.
3. Place a known mass on the cell.
4. Read the raw `get_units(10)` value and divide by the known mass in grams.
5. Update the corresponding `CAL_*` constant and reflash.

**INA228 shunt resistor:** The firmware assumes a **0.2 mΩ shunt** (`0.0002 Ω`).  Update the divisor in `loop()` if your PCB uses a different value:

```cpp
a = ((readReg24(0x04) >> 4) * 0.0000003125f) / 0.0002f;  // ← change 0.0002 to your shunt value
```

---

## Firmware (PlatformIO)

### Prerequisites

- [PlatformIO IDE](https://platformio.org/) (VS Code extension or CLI)
- No additional drivers needed — ESP32-S3 uses native USB CDC

### Build & Flash

```bash
# From the project root
pio run --target upload

# Open serial monitor
pio device monitor
```

### Key Build Flags (`platformio.ini`)

| Flag | Purpose |
|------|---------|
| `ARDUINO_USB_MODE=1` | Enable internal USB PHY |
| `ARDUINO_USB_CDC_ON_BOOT=1` | Start USB CDC Serial before `setup()` |

### Library Dependencies

| Library | Version |
|---------|---------|
| `bogde/HX711` | `^0.7.5` |

---

## PC Logger GUI

### Installation

```bash
cd Tools
pip install -r requirements.txt
```

### Run

```bash
python TESTSTANDGUI.py
```

The GUI auto-detects the ESP32 on startup by scanning COM / tty ports for USB, CP210x, CH340, and FTDI identifiers.  If not found it retries every 2 seconds.

### Interface Overview

| Panel | Description |
|-------|-------------|
| **Telemetry** (top-left) | Live thrust (g), torque (Nm), RPM, voltage/current/power |
| **Throttle Slider** | Manual throttle 0–100 % (disabled during automated tests) |
| **Diagnostics** | Raw bus readings, serial buffer latency indicator |
| **Automated Program** | Select and run a named throttle sequence; logs to CSV |
| **Config & Hardware** | Tare button, prop size, battery ID, temperature, humidity |
| **KILL MOTOR** | Emergency stop — always visible, keyboard shortcut `K` |
| **Plot area** | Four real-time charts: throttle, thrust, torque push, torque pull |

---

## Serial Protocol

### ESP32 → PC (50 Hz CSV stream)

```
Time_ms,Thrust_g,TorquePush_Nm,TorquePull_Nm,Throttle_PWM,Push_g,Pull_g,Volts,Amps,RPM
```

Example row:
```
12345,423.50,0.0821,-0.0654,1450,8.36,-6.67,22.18,14.320,8742
```

### PC → ESP32 (text commands, newline-terminated)

| Command | Effect |
|---------|--------|
| `Z` | Tare (zero) all three load cells |
| `1000` – `2000` | Set ESC throttle in microseconds |

---

## Automated Test Programs

Defined in `TESTSTANDGUI.py` as `(throttle_%, duration_s)` step lists:

| Program | Steps | Total Duration |
|---------|-------|---------------|
| **Quick Ramp Short** | 0→30→40→50→60→70→80→90→100→0 % (5 s each) | 50 s |
| **Quick Ramp** | Same steps, 10 s each | 100 s |
| **Step Test** | 0→20→40→60→80→100→0 % | 32 s |
| **Burst** | 0→100→0 % (2 s idle, 2 s burst, 2 s idle) | 9 s |
| **Manual Mode** | No steps; use the throttle slider | — |

A **5-second arming countdown** runs before each automated test.  The test can be aborted at any time with the **KILL MOTOR** button or `K` key.

---

## CSV Output Format

Files are written to the working directory with the naming pattern:

```
<run_name>_HHMMSS.csv
```

Row 1 — metadata:
```
# Prop, 10x4.5, # Batt, Pack_01, # Temp, 20, # Humi, 50
```

Row 2 — column headers:
```
PC_Time, ESP_ms, Thrust_g, TorquePush_Nm, TorquePull_Nm, Throttle_PWM, Push_g, Pull_g, Volts, Amps, RPM
```

Subsequent rows — 50 Hz data.

---

## Safety Features

| Feature | Detail |
|---------|--------|
| **LiPo under-voltage cutoff** | Emergency stop triggered if `Volts < 26.4 V` during a test |
| **RPM zero-timeout** | RPM forced to 0 if no pulse is received for 200 ms |
| **ESC idle on boot** | Firmware outputs 1000 µs immediately on startup |
| **ESC idle on close** | GUI sends `1000` and flushes serial before closing the port |
| **`K` key kill** | Emergency stop bound to keyboard at all times |
| **Thread-safe ISR** | `noInterrupts()`/`interrupts()` guard used when reading volatile RPM state |

---

## Repository Structure

```
UAV-Thrust-Stand/
├── src/
│   └── main.cpp            # ESP32-S3 firmware (PlatformIO / Arduino)
├── TESTSTANDGUI.py         # PC GUI — launcher for the propeller and hardware tests
├── Tools/
│   └── requirements.txt    # Python dependencies (pyserial, matplotlib)
├── include/                # PlatformIO include directory
├── lib/                    # PlatformIO local libraries
├── test/                   # PlatformIO test directory
├── platformio.ini          # Build configuration
├── Propeller Tests/        # Auto-named recorded runs (prop_test_<date>_#<n>_<label>.csv)
└── DBF_Test_Run_*.csv      # Example recorded test runs
```
