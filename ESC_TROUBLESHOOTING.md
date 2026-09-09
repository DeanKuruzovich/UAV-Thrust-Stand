# ESC / Motor Won't Spin — Diagnose BEFORE You Buy Anything

**Rule #1: Never conclude the ESC (or motor) is dead until every step below passes.**
An ESC is rarely the actual fault. The usual culprits are signal, arming, ground,
or power — all free to check. Do not spend money on hardware until 1–5 are green.

These checks go cheapest/fastest first. Stop as soon as one fails — that's your problem.

---

## 0. What does NOT prove anything (don't be misled)
- **A raw HIGH/LOW digital toggle on the signal pin is NOT an ESC signal.** An ESC
  needs a 50 Hz pulse train, pulse width 1000–2000 µs. A pin going on/off at 1 Hz
  will *never* arm or spin a motor — it tells you nothing about the ESC.
- **A DC multimeter on the signal pin tells you almost nothing.** PWM averages to a
  tiny voltage (~0.17 V at idle, ~0.33 V at full). Seeing "barely any change" is
  normal and is NOT evidence of a fault.
- **Never short the signal pin to ground to "measure" it.** GPIO8 is push-pull;
  shorting it fights the driver, gives garbage readings, and can damage the MCU pin.

## 1. Is the SIGNAL valid? (scope it — this is the #1 skipped step)
- Probe **GPIO8** with the **ADALM-2000** (`/dev/cu.usbmodem1204`) as a scope, or a
  multimeter in **frequency (Hz)** / **duty (%)** mode.
- Expect: **~50 Hz**, pulse width **1000 µs (idle) → 2000 µs (full)**, swinging
  fully 0 ↔ 3.3 V.
- No signal at all → firmware/LEDC/pin problem, not the ESC. (Flash `main`, send a
  throttle value, re-scope.)

## 2. Does the ESC ARM? (listen)
- On power-up the ESC must see **idle (1000 µs) held ~2–4 s**; it should beep its
  arming sequence (cell count, then ready tones).
- **No beeps** → power or ground or signal problem (steps 3–4), NOT a dead ESC.
- Beeps but no spin on throttle → continue.

## 3. Is there a COMMON GROUND?
- ESP32 GND **must** be tied to the ESC GND. Without a shared ground the ESC cannot
  read the pulse at all, even if the scope shows a perfect signal at the board.
- This is the single most common "my ESC is dead" false alarm.

## 4. Is the ESC POWERED correctly?
- Battery connected to the ESC's **power leads** at the correct voltage (check cell
  count). The signal wire alone does nothing.
- Confirm the bench supply / battery isn't in current limit or sagging.

## 5. Is the MOTOR wired to the ESC?
- All **three phase wires** connected. (Swapping any two just reverses direction —
  fine — but a loose/missing phase = no spin or stutter.)

---

## Only now suspect the ESC itself
If 1–5 all pass and it still won't spin:
- **Swap-test the signal source**, not the ESC: drive the same ESC from a cheap
  servo tester or an RC receiver. If it spins there, the problem is upstream (our
  firmware/wiring), not the ESC.
- If it won't spin from a known-good source either, *then* the ESC is a candidate.
- Likewise, test a suspect motor on a known-good ESC before condemning it.

## Quick known-good signal from this rig
Flash the logging firmware and command throttle over serial — no extra hardware:
```
~/.platformio/penv/bin/pio run -e esp32-s3-devkitc-1 -t upload
.venv/bin/python Tools/motor_ramp_test.py        # ramps 0→100%
# or the GUI:
.venv/bin/python Tools/motor_logger.py
```
Firmware streams 50 Hz CSV; the ESC line is GPIO8 the whole time.
