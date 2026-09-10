"""
TESTSTANDGUI.py — UAV Thrust Stand control suite
================================================

Single entry point for the thrust stand. The launcher window shows two tiles:

    Run Test         automated throttle programs with a full telemetry readout
                     and live plots, recorded to
                     "Propeller Tests/prop_test_<date>_#<n>_<label>.csv"
    Debug Menu       read-only bring-up screen: every telemetry field printed
                     plainly, a console of link and sensor faults, and an
                     arm-gated throttle slider. Records nothing.

Usage
-----
    python3 TESTSTANDGUI.py

The ESP32-S3 is auto-detected on any COM / tty port at 921600 baud (must match
the firmware). Press K anywhere in the app for an emergency stop.

Dependencies
------------
    pip install pyserial matplotlib      (see Tools/requirements.txt)
"""

import csv
import math
import os
import re
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# --------------------------------------------------------------------------
# CONFIG
# --------------------------------------------------------------------------

BAUD = 921600
MAX_PLOT_POINTS = 150
VOLTAGE_CUTOFF = 26.4          # LiPo cell-sag abort threshold, volts
STALE_AFTER = 1.5              # seconds without a frame before the link reads stale
REPO_ROOT = Path(__file__).resolve().parent
TEST_DIR = REPO_ROOT / "Propeller Tests"

PROGRAMS = {
    "Quick Ramp Short": [(0, 5), (30, 5), (40, 5), (50, 5), (60, 5), (70, 5), (80, 5), (90, 5), (100, 5), (0, 5)],
    "Quick Ramp":       [(0, 10), (30, 10), (40, 10), (50, 10), (60, 10), (70, 10), (80, 10), (90, 10), (100, 10), (0, 10)],
    "Step Test":        [(0, 2), (20, 5), (40, 5), (60, 5), (80, 5), (100, 5), (0, 2)],
    "Burst":            [(0, 5), (100, 2), (0, 2)],
}

# --------------------------------------------------------------------------
# THEME — greyscale on black, one accent, red reserved for danger and faults.
# --------------------------------------------------------------------------

BG      = "#0e0f10"   # window
PANEL   = "#171819"   # cards
FIELD   = "#202224"   # inputs, slider track
LINE    = "#2c2f32"   # borders, grid
TEXT    = "#e9eaec"   # primary text
DIM     = "#8b9197"   # secondary text
MUTED   = "#5a5f64"   # placeholder / disabled text
ACCENT  = "#dcdee1"   # highlight / live values — deliberately not a hue
DANGER  = "#d05353"   # the only colour in the app: kill button and faults

F_TITLE = ("Helvetica", 15, "bold")
F_HEAD  = ("Helvetica", 10, "bold")
F_BODY  = ("Helvetica", 11)
F_SMALL = ("Helvetica", 9)
F_MONO  = ("Menlo", 11)
F_MONO_S = ("Menlo", 10)
F_BIG   = ("Helvetica", 40, "bold")


# --------------------------------------------------------------------------
# WIDGETS
#
# tk.Button ignores `bg` on macOS Aqua, which is why the old GUI rendered
# unreadable dark-on-grey buttons. These are Frame+Label composites so the
# colours are ours on every platform.
# --------------------------------------------------------------------------

class FlatButton(tk.Frame):
    """A flat, fully themed button that renders identically on macOS and Linux."""

    KINDS = {
        # kind:      (bg,      fg,     hover bg)
        "normal":    (FIELD,   TEXT,   "#2a2d30"),
        "accent":    (ACCENT,  "#101112", "#f2f4f6"),
        "danger":    (DANGER,  "#150a0a", "#e06767"),
        "ghost":     (PANEL,   DIM,    FIELD),
    }

    def __init__(self, master, text, command, kind="normal",
                 font=F_HEAD, pady=9, padx=14, **kw):
        bg, fg, hover = self.KINDS.get(kind, self.KINDS["normal"])
        super().__init__(master, bg=bg, highlightthickness=1,
                         highlightbackground=LINE, highlightcolor=LINE, **kw)
        self._bg, self._fg, self._hover = bg, fg, hover
        self._command = command
        self._enabled = True
        self.label = tk.Label(self, text=text, bg=bg, fg=fg, font=font,
                              pady=pady, padx=padx)
        self.label.pack(fill="both", expand=True)
        for w in (self, self.label):
            w.bind("<Button-1>", self._press)
            w.bind("<ButtonRelease-1>", self._release)
            w.bind("<Enter>", self._enter)
            w.bind("<Leave>", self._leave)

    # -- state ------------------------------------------------------------
    def set_enabled(self, on):
        self._enabled = bool(on)
        self._paint(self._bg if self._enabled else PANEL)
        self.label.config(fg=self._fg if self._enabled else MUTED)

    def set_text(self, text):
        self.label.config(text=text)

    def set_kind(self, kind):
        bg, fg, hover = self.KINDS.get(kind, self.KINDS["normal"])
        self._bg, self._fg, self._hover = bg, fg, hover
        self.set_enabled(self._enabled)

    # -- events -----------------------------------------------------------
    def _paint(self, colour):
        self.config(bg=colour)
        self.label.config(bg=colour)

    def _enter(self, _):
        if self._enabled:
            self._paint(self._hover)

    def _leave(self, _):
        self._paint(self._bg if self._enabled else PANEL)

    def _press(self, _):
        if self._enabled:
            self._paint(self._bg)

    def _release(self, _):
        if self._enabled:
            self._paint(self._hover)
            self._command()


class VSlider(tk.Canvas):
    """
    Vertical 0-100 throttle slider drawn on a canvas.

    Native tk.Scale is unstyleable on macOS; this keeps the throttle control
    legible and gives it an unmistakable disabled state, which matters when the
    hardware screen gates it behind an arm button.
    """

    W = 58
    KNOB_H = 16

    def __init__(self, master, command=None, height=260, enabled=True):
        super().__init__(master, width=self.W, height=height, bg=PANEL,
                         highlightthickness=0, bd=0)
        self._command = command
        self._value = 0
        self._enabled = enabled
        self.bind("<Configure>", lambda e: self._redraw())
        self.bind("<Button-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)

    # -- geometry ---------------------------------------------------------
    def _bounds(self):
        h = self.winfo_height() or int(self["height"])
        top = self.KNOB_H // 2 + 2
        bot = h - self.KNOB_H // 2 - 2
        return top, bot

    def _y_for(self, value):
        top, bot = self._bounds()
        return bot - (value / 100.0) * (bot - top)

    def _value_for(self, y):
        top, bot = self._bounds()
        return max(0, min(100, round((bot - y) / max(1, bot - top) * 100)))

    # -- api --------------------------------------------------------------
    def get(self):
        return self._value

    def set(self, value, notify=False):
        self._value = max(0, min(100, int(value)))
        self._redraw()
        if notify and self._command:
            self._command(self._value)

    def set_enabled(self, on):
        self._enabled = bool(on)
        self._redraw()

    # -- drawing ----------------------------------------------------------
    def _redraw(self):
        self.delete("all")
        top, bot = self._bounds()
        cx = (self.winfo_width() or self.W) / 2
        track_w = 12
        fill = ACCENT if self._enabled else MUTED
        track = FIELD if self._enabled else "#191b1c"

        self.create_rectangle(cx - track_w / 2, top, cx + track_w / 2, bot,
                              fill=track, outline=LINE)
        y = self._y_for(self._value)
        if self._value > 0:
            self.create_rectangle(cx - track_w / 2, y, cx + track_w / 2, bot,
                                  fill=fill, outline="")
        self.create_rectangle(cx - 20, y - self.KNOB_H / 2, cx + 20, y + self.KNOB_H / 2,
                              fill=fill, outline=BG, width=2)

    def _drag(self, event):
        if not self._enabled:
            return
        self.set(self._value_for(event.y), notify=True)


class ScrollFrame(tk.Frame):
    """
    Vertically scrollable container.

    Sidebar content is taller than a laptop screen, and plain pack() silently
    drops whatever does not fit — which is how the KILL button used to vanish.
    Scrolling means content is never lost, only reachable.
    """

    def __init__(self, master, **kw):
        super().__init__(master, bg=BG, **kw)
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0, bd=0)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.body = tk.Frame(self.canvas, bg=BG)
        self._window = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind("<Configure>", lambda _e: self.canvas.configure(
            scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfig(
            self._window, width=e.width))

    def bind_wheel(self):
        """Bind the wheel across every child; call once the content is built."""
        def wheel(event):
            first, last = self.canvas.yview()
            if first <= 0.0 and last >= 1.0:      # nothing to scroll
                return
            self.canvas.yview_scroll(-1 if event.delta > 0 else 1, "units")

        def walk(widget):
            widget.bind("<MouseWheel>", wheel)
            for child in widget.winfo_children():
                walk(child)
        walk(self)


class Dropdown(tk.Frame):
    """
    A themed select control.

    A ttk.Combobox renders as unreadable white-on-white in its readonly state
    under the macOS Aqua theme, so the closed control is drawn by us and the
    open list is a native tk.Menu (which the system always renders legibly).
    """

    def __init__(self, master, values, font=F_BODY):
        super().__init__(master, bg=FIELD, highlightthickness=1,
                         highlightbackground=LINE, highlightcolor=LINE)
        self.values = list(values)
        self._value = self.values[0] if self.values else ""
        self.label = tk.Label(self, text=self._value, bg=FIELD, fg=TEXT,
                              font=font, anchor="w", padx=9, pady=6)
        self.label.pack(side="left", fill="x", expand=True)
        self.caret = tk.Label(self, text="\u25be", bg=FIELD, fg=DIM, font=font, padx=9)
        self.caret.pack(side="right")

        self.menu = tk.Menu(self, tearoff=0)
        for value in self.values:
            self.menu.add_command(label=value,
                                  command=lambda v=value: self.set(v))
        for widget in (self, self.label, self.caret):
            widget.bind("<Button-1>", self._open)

    def _open(self, _event):
        self.menu.post(self.winfo_rootx(), self.winfo_rooty() + self.winfo_height())

    def get(self):
        return self._value

    def set(self, value):
        self._value = value
        self.label.config(text=value)


def card(master, title=None, center=False):
    """A titled panel. Returns the body frame that callers pack into."""
    outer = tk.Frame(master, bg=PANEL, highlightthickness=1,
                     highlightbackground=LINE, highlightcolor=LINE)
    if title:
        tk.Label(outer, text=title.upper(), bg=PANEL, fg=DIM, font=F_SMALL,
                 anchor="center" if center else "w").pack(fill="x", padx=12, pady=(8, 0))
    body = tk.Frame(outer, bg=PANEL)
    body.pack(fill="both", expand=True, padx=12, pady=9)
    outer.body = body
    return outer


def labelled_entry(master, label, default=""):
    """A stacked caption + dark entry field. Returns the Entry."""
    tk.Label(master, text=label, bg=PANEL, fg=DIM, font=F_SMALL,
             anchor="w").pack(fill="x", pady=(5, 2))
    ent = tk.Entry(master, bg=FIELD, fg=TEXT, insertbackground=ACCENT,
                   font=F_BODY, relief="flat", highlightthickness=1,
                   highlightbackground=LINE, highlightcolor=ACCENT)
    ent.insert(0, default)
    ent.pack(fill="x", ipady=5)
    return ent


# --------------------------------------------------------------------------
# SERIAL LINK
# --------------------------------------------------------------------------

# Field order emitted by src/main.cpp at 50 Hz.
CORE_FIELDS = ["Time_ms", "Thrust_g", "TorquePush_Nm", "TorquePull_Nm", "Throttle_PWM",
               "Push_g", "Pull_g", "Volts", "Amps", "RPM"]
AUX_FIELDS = ["AmbTemp_C", "Humidity_pct", "Pressure_hPa", "IRTemp_C", "Pitot_mV",
              "GPS_Lat", "GPS_Lon", "GPS_Speed_mps", "GPS_Fix", "GPS_Sats"]

# Pitot front-end: DIY Drones Airspeed Sensor V2.0 (MPXV7002DP), ratiometric to
# 5 V with ~2.5 V at zero flow, behind a 10k-10k divider on the ADC line.
PITOT_DIVIDER = 2.0
PITOT_VSUPPLY = 5.0
AIR_DENSITY = 1.225


def pitot_airspeed(mv):
    """
    Approximate airspeed (m/s) from raw pitot ADC millivolts.

    Undoes the 10k-10k divider, applies the MPXV7002DP transfer
    (Vout = Vs*(0.2*kPa + 0.5)) for differential pressure, then v = sqrt(2*dP/rho).
    Uncalibrated — good enough for a bench "does it respond" check.
    """
    try:
        vout = (mv * PITOT_DIVIDER) / 1000.0
        dp_pa = (vout / PITOT_VSUPPLY - 0.5) / 0.2 * 1000.0
        return (2.0 * dp_pa / AIR_DENSITY) ** 0.5 if dp_pa > 0 else 0.0
    except (TypeError, ValueError):
        return 0.0


class SerialLink:
    """
    Owns the ESP32 connection for the whole app.

    A daemon thread drains the port, keeps only the newest complete frame per
    cycle (so the GUI never falls behind the 50 Hz stream), and publishes it as
    a plain dict under a lock. Views poll `snapshot()` on their own timer rather
    than taking callbacks on the reader thread.

    It also keeps a rolling event log — port scans, opens, drops, malformed
    frames — which the Hardware Test screen renders verbatim.
    """

    def __init__(self):
        self.ser = None
        self.port_name = None
        self.running = True
        self.frame = None
        self.frame_time = 0.0
        self.frames_buffered = 0
        self.frame_count = 0
        self._lock = threading.Lock()

        self.events = deque(maxlen=200)
        self._events_lock = threading.Lock()
        self._last_event = None

        # Set by the propeller test while a run is recording; the reader thread
        # writes every row so no samples are dropped between GUI ticks.
        self.csv_writer = None
        self.recording = False

        # Raised by the reader when the bus sags mid-run; the owning view acts on it.
        self.voltage_fault = None

        self._scanning = False

    # -- events -----------------------------------------------------------
    def log(self, level, message):
        """Record a link event. Consecutive identical messages are collapsed."""
        if self._last_event == (level, message):
            return
        self._last_event = (level, message)
        with self._events_lock:
            self.events.append((datetime.now().strftime("%H:%M:%S"), level, message))

    def event_lines(self):
        with self._events_lock:
            return list(self.events)

    # -- connection -------------------------------------------------------
    def start(self, root):
        """Begin auto-connect attempts on the Tk event loop."""
        self.root = root
        self._scan()

    def _scan(self):
        if not self.running:
            return
        if self.ser and self.ser.is_open:
            self.root.after(2000, self._scan)
            return

        ports = list(serial.tools.list_ports.comports())
        if not ports:
            self.log("FAIL", "No serial ports found — is the board plugged in?")
        # Prefer the ESP32-S3 native USB CDC (Espressif VID 0x303A); never grab
        # the ADALM-2000 (Analog Devices VID 0x0456), which also enumerates as a
        # usbmodem and would match the generic "USB" keyword below.
        ports.sort(key=lambda p: 0 if p.vid == 0x303A else 1)
        for p in ports:
            if p.vid == 0x0456:
                continue
            hay = p.description + (p.hwid or "")
            if p.vid == 0x303A or any(k in hay for k in ("USB", "ESP32", "UART", "CP210", "CH340", "FTDI")):
                try:
                    self.ser = serial.Serial(p.device, BAUD, timeout=0.1)
                    self.port_name = p.device
                    self._last_event = None
                    self.log("OK", f"Connected on {p.device} @ {BAUD} baud")
                    threading.Thread(target=self._read_loop, daemon=True).start()
                    self.root.after(2000, self._scan)
                    return
                except serial.SerialException as e:
                    self.log("FAIL", f"Could not open {p.device}: {e}")
        self.root.after(2000, self._scan)

    def is_connected(self):
        return bool(self.ser and self.ser.is_open)

    def is_stale(self):
        return self.frame is None or (time.time() - self.frame_time) > STALE_AFTER

    def write(self, text):
        """Send a command line to the firmware. Returns True if it went out."""
        if not self.is_connected():
            return False
        try:
            self.ser.write(f"{text}\n".encode())
            return True
        except serial.SerialException as e:
            self.log("FAIL", f"Write failed: {e}")
            return False

    def set_throttle_pct(self, pct):
        """Throttle as 0-100 %, mapped to the firmware's 1000-2000 us window."""
        return self.write(str(1000 + int(pct) * 10))

    def idle(self):
        return self.write("1000")

    def tare(self):
        return self.write("Z")

    def snapshot(self):
        with self._lock:
            return self.frame, self.frames_buffered

    # -- reader -----------------------------------------------------------
    def _read_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = None
                buffered = 0
                while self.ser.in_waiting > 0 and self.running:
                    raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if raw and "," in raw:
                        line = raw
                        buffered += 1
                if line is None:
                    time.sleep(0.005)
                    continue

                parts = line.split(",")
                if len(parts) < 10:
                    continue
                try:
                    core = [float(v) for v in parts[:10]]
                except ValueError:
                    self.log("WARN", f"Malformed frame: {line[:60]}")
                    continue

                frame = dict(zip(CORE_FIELDS, core))
                frame["Throttle_pct"] = (frame["Throttle_PWM"] - 1000) / 10.0
                frame["Watts"] = frame["Volts"] * frame["Amps"]

                if len(parts) >= 20:
                    try:
                        aux = [float(v) for v in parts[10:20]]
                        frame.update(zip(AUX_FIELDS, aux))
                        frame["GPS_Fix"] = int(frame["GPS_Fix"])
                        frame["GPS_Sats"] = int(frame["GPS_Sats"])
                        frame["Airspeed_mps"] = pitot_airspeed(frame["Pitot_mV"])
                    except ValueError:
                        self.log("WARN", "Aux sensor fields unreadable in frame")

                if self.recording and 1.0 < frame["Volts"] < VOLTAGE_CUTOFF:
                    self.voltage_fault = frame["Volts"]

                with self._lock:
                    self.frame = frame
                    self.frame_time = time.time()
                    self.frames_buffered = buffered
                    self.frame_count += 1
                    writer = self.csv_writer if self.recording else None

                if writer:
                    writer.writerow([datetime.now().strftime("%H:%M:%S.%f")] + parts[:10])

            except (serial.SerialException, OSError) as e:
                self.log("FAIL", f"Link lost on {self.port_name}: {e}")
                break
            except Exception as e:                      # keep the GUI alive
                self.log("FAIL", f"Reader error: {e}")
                break

        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        if self.running:
            self.log("WARN", "Disconnected — rescanning for the board")

    def shutdown(self):
        self.running = False
        self.recording = False
        if self.is_connected():
            try:
                self.ser.write(b"1000\n")
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass


# --------------------------------------------------------------------------
# RUN NAMING
# --------------------------------------------------------------------------

def next_run_path(label):
    """
    Build the auto-named CSV path for the next propeller run.

        Propeller Tests/prop_test_09-09-26_#3_APC10x45.csv

    The date uses dashes (a slash would be a path separator) and the run number
    restarts at 1 each day, picking up after the highest existing number so a
    reopened app never overwrites an earlier run.
    """
    TEST_DIR.mkdir(exist_ok=True)
    prefix = f"prop_test_{datetime.now().strftime('%m-%d-%y')}_#"
    highest = 0
    for existing in TEST_DIR.glob(f"{prefix}*.csv"):
        m = re.match(re.escape(prefix) + r"(\d+)", existing.name)
        if m:
            highest = max(highest, int(m.group(1)))
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", label.strip()).strip("_")
    name = f"{prefix}{highest + 1}" + (f"_{safe}" if safe else "")
    return TEST_DIR / f"{name}.csv"


# --------------------------------------------------------------------------
# LAUNCHER
# --------------------------------------------------------------------------

class LauncherView(tk.Frame):
    """Home screen: two app tiles centred in the window, nothing else."""

    def __init__(self, master, app):
        super().__init__(master, bg=BG)
        self.app = app

        tiles = tk.Frame(self, bg=BG)
        tiles.pack(expand=True)
        self._tile(tiles, "Run Test", self.app.open_propeller).pack(side="left", padx=18)
        self._tile(tiles, "Test Stand Debug Menu", self.app.open_hardware).pack(side="left", padx=18)

    def _tile(self, master, title, command):
        """A square, clickable app tile."""
        tile = tk.Frame(master, bg=PANEL, width=250, height=250,
                        highlightthickness=1, highlightbackground=LINE)
        tile.pack_propagate(False)
        name = tk.Label(tile, text=title, bg=PANEL, fg=TEXT, font=F_TITLE,
                        wraplength=210, justify="center")
        name.place(relx=0.5, rely=0.5, anchor="center")

        def enter(_):
            tile.config(bg=FIELD, highlightbackground=ACCENT)
            name.config(bg=FIELD)

        def leave(_):
            tile.config(bg=PANEL, highlightbackground=LINE)
            name.config(bg=PANEL)

        for widget in (tile, name):
            widget.bind("<Enter>", enter)
            widget.bind("<Leave>", leave)
            widget.bind("<Button-1>", lambda _e: command())
        return tile

    def close(self):
        pass


# --------------------------------------------------------------------------
# HARDWARE TEST
# --------------------------------------------------------------------------

class HardwareTestView(tk.Frame):
    """
    Bring-up screen. Prints every telemetry field, logs link and sensor faults,
    and offers arm-gated direct throttle control. Records nothing to disk.
    """

    # (section, key, caption, format, unit)
    ROWS = [
        ("Load cells", "Thrust_g",      "Thrust",        "{:8.2f}", "g"),
        ("Load cells", "Push_g",        "Push cell",     "{:8.2f}", "g"),
        ("Load cells", "Pull_g",        "Pull cell",     "{:8.2f}", "g"),
        ("Load cells", "TorquePush_Nm", "Torque push",   "{:8.4f}", "Nm"),
        ("Load cells", "TorquePull_Nm", "Torque pull",   "{:8.4f}", "Nm"),
        ("Motor",      "Throttle_PWM",  "Throttle PWM",  "{:8.0f}", "us"),
        ("Motor",      "Throttle_pct",  "Throttle",      "{:8.1f}", "%"),
        ("Motor",      "RPM",           "RPM",           "{:8.0f}", "rpm"),
        ("Power",      "Volts",         "Bus voltage",   "{:8.2f}", "V"),
        ("Power",      "Amps",          "Bus current",   "{:8.2f}", "A"),
        ("Power",      "Watts",         "Power",         "{:8.1f}", "W"),
        ("Environment", "AmbTemp_C",    "Ambient temp",  "{:8.1f}", "C"),
        ("Environment", "IRTemp_C",     "IR / laser temp", "{:8.1f}", "C"),
        ("Environment", "Humidity_pct", "Humidity",      "{:8.1f}", "%"),
        ("Environment", "Pressure_hPa", "Pressure",      "{:8.1f}", "hPa"),
        ("Airspeed",   "Pitot_mV",      "Pitot raw",     "{:8.0f}", "mV"),
        ("Airspeed",   "Airspeed_mps",  "Airspeed",      "{:8.1f}", "m/s"),
        ("GPS",        "GPS_Fix",       "Fix",           "{:8.0f}", ""),
        ("GPS",        "GPS_Sats",      "Satellites",    "{:8.0f}", ""),
        ("GPS",        "GPS_Lat",       "Latitude",      "{:8.5f}", "deg"),
        ("GPS",        "GPS_Lon",       "Longitude",     "{:8.5f}", "deg"),
        ("GPS",        "GPS_Speed_mps", "Ground speed",  "{:8.1f}", "m/s"),
    ]

    def __init__(self, master, app):
        super().__init__(master, bg=BG)
        self.app = app
        self.link = app.link
        self.armed = False
        self.value_labels = {}

        app.build_topbar(self, "Test Stand Debug Menu")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self._build_values(body)
        self._build_faults(body)
        self._build_control(body)

        self._tick()

    # -- layout -----------------------------------------------------------
    def _build_values(self, master):
        panel = card(master, "Live values")
        panel.pack(side="left", fill="both", expand=True)
        grid = panel.body

        section = None
        row = 0
        for sect, key, caption, fmt, unit in self.ROWS:
            if sect != section:
                section = sect
                pad = (0, 4) if row == 0 else (12, 4)
                tk.Label(grid, text=sect.upper(), bg=PANEL, fg=ACCENT,
                         font=F_SMALL, anchor="w").grid(row=row, column=0, columnspan=3,
                                                        sticky="w", pady=pad)
                row += 1
            tk.Label(grid, text=caption, bg=PANEL, fg=DIM, font=F_MONO_S,
                     anchor="w").grid(row=row, column=0, sticky="w", padx=(8, 14))
            val = tk.Label(grid, text="  ------", bg=PANEL, fg=MUTED, font=F_MONO,
                           anchor="e")
            val.grid(row=row, column=1, sticky="e")
            tk.Label(grid, text=unit, bg=PANEL, fg=DIM, font=F_MONO_S,
                     anchor="w").grid(row=row, column=2, sticky="w", padx=(6, 0))
            self.value_labels[key] = (val, fmt)
            row += 1
        grid.columnconfigure(1, weight=1)

    def _build_faults(self, master):
        wrap = tk.Frame(master, bg=BG)
        wrap.pack(side="left", fill="both", expand=True, padx=(14, 14))

        status = card(wrap, "Link")
        status.pack(fill="x")
        self.link_lbl = tk.Label(status.body, text="searching...", bg=PANEL,
                                 fg=DIM, font=F_MONO, anchor="w")
        self.link_lbl.pack(fill="x")
        self.rate_lbl = tk.Label(status.body, text="no telemetry", bg=PANEL,
                                 fg=MUTED, font=F_MONO_S, anchor="w")
        self.rate_lbl.pack(fill="x", pady=(4, 0))

        faults = card(wrap, "Console")
        faults.pack(fill="both", expand=True, pady=(14, 0))
        self.log_text = tk.Text(faults.body, bg=BG, fg=DIM, font=F_MONO_S,
                                relief="flat", highlightthickness=1, wrap="word",
                                highlightbackground=LINE, insertbackground=ACCENT)
        self.log_text.pack(fill="both", expand=True)
        self.log_text.tag_config("FAIL", foreground=DANGER)
        self.log_text.tag_config("WARN", foreground=ACCENT)
        self.log_text.tag_config("OK", foreground=TEXT)
        self.log_text.config(state="disabled")
        self._log_shown = 0

    def _build_control(self, master):
        panel = card(master, "Direct motor control", center=True)
        panel.pack(side="left", fill="y")
        body = panel.body

        # Reserved before the rest of the column, for the same reason as above.
        FlatButton(body, "KILL MOTOR (K)", self.app.emergency_stop, kind="danger",
                   font=F_HEAD).pack(side="bottom", fill="x", pady=(8, 0))
        FlatButton(body, "TARE LOAD CELLS", self.link.tare, kind="normal",
                   font=F_SMALL).pack(side="bottom", fill="x", pady=(14, 0))

        self.throttle_lbl = tk.Label(body, text="0%", bg=PANEL, fg=MUTED, font=F_BIG)
        self.throttle_lbl.pack(pady=(0, 8))

        self.slider = VSlider(body, command=self._on_slider, height=280, enabled=False)
        self.slider.pack(pady=6)

        self.arm_btn = FlatButton(body, "ARM MOTOR DIRECT CONTROL", self._toggle_arm,
                                  kind="accent", font=F_SMALL)
        self.arm_btn.pack(fill="x", pady=(14, 0))

    # -- control ----------------------------------------------------------
    def _toggle_arm(self):
        if self.armed:
            self.disarm()
            return
        if not self.link.is_connected():
            self.link.log("FAIL", "Cannot arm — no board connected")
            return
        self.armed = True
        self.link.idle()
        self.slider.set(0)
        self.slider.set_enabled(True)
        self.arm_btn.set_text("DISARM")
        self.arm_btn.set_kind("danger")
        self.throttle_lbl.config(fg=ACCENT)
        self.link.log("WARN", "Direct control ARMED — throttle slider is live")

    def disarm(self):
        was_armed = self.armed
        self.armed = False
        self.slider.set(0)
        self.slider.set_enabled(False)
        self.link.idle()
        self.arm_btn.set_text("ARM MOTOR DIRECT CONTROL")
        self.arm_btn.set_kind("accent")
        self.throttle_lbl.config(text="0%", fg=MUTED)
        if was_armed:
            self.link.log("OK", "Direct control disarmed — throttle at idle")

    def _on_slider(self, pct):
        if not self.armed:
            return
        self.throttle_lbl.config(text=f"{pct}%")
        if not self.link.set_throttle_pct(pct):
            self.link.log("FAIL", "Throttle command not sent — link is down")

    def emergency_stop(self):
        self.disarm()

    # -- refresh ----------------------------------------------------------
    def _tick(self):
        if not self.winfo_exists():
            return
        frame, buffered = self.link.snapshot()
        stale = self.link.is_stale()

        if not self.link.is_connected():
            self.link_lbl.config(text="DISCONNECTED", fg=DANGER)
            self.rate_lbl.config(text="scanning ports every 2 s", fg=MUTED)
            if self.armed:
                self.disarm()
        elif stale:
            self.link_lbl.config(text=f"{self.link.port_name}  NO DATA", fg=DANGER)
            self.rate_lbl.config(text="port is open but the board is not streaming", fg=DANGER)
        else:
            self.link_lbl.config(text=f"{self.link.port_name}  streaming", fg=ACCENT)
            self.rate_lbl.config(text=f"{self.link.frame_count} frames  |  "
                                      f"{buffered} buffered per tick", fg=DIM)

        for key, (widget, fmt) in self.value_labels.items():
            value = frame.get(key) if frame else None
            if value is None or (isinstance(value, float) and math.isnan(value)):
                widget.config(text="  ------", fg=MUTED)
            elif stale:
                widget.config(text=fmt.format(value), fg=MUTED)
            else:
                widget.config(text=fmt.format(value), fg=TEXT)

        self._flush_log()
        self.after(100, self._tick)

    def _flush_log(self):
        lines = self.link.event_lines()
        if len(lines) == self._log_shown:
            return
        self.log_text.config(state="normal")
        for stamp, level, message in lines[self._log_shown:]:
            self.log_text.insert("end", f"{stamp}  [{level}] {message}\n", level)
        self.log_text.see("end")
        self.log_text.config(state="disabled")
        self._log_shown = len(lines)

    def close(self):
        self.disarm()


# --------------------------------------------------------------------------
# PROPELLER TEST
# --------------------------------------------------------------------------

class PropellerTestView(tk.Frame):
    """
    The recorded test screen: manual throttle, automated programs, live plots,
    and auto-named CSV output under "Propeller Tests/".
    """

    def __init__(self, master, app):
        super().__init__(master, bg=BG)
        self.app = app
        self.link = app.link

        self.test_active = False
        self.counting_down = False
        self.show_all = False
        self.run_path = None
        self._status_hold = 0.0     # keeps result messages up against the 20 Hz tick

        self.plot_thrust = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_tq_push = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_tq_pull = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_throttle = deque(maxlen=MAX_PLOT_POINTS)
        self.hist_thrust, self.hist_tq_push = [], []
        self.hist_tq_pull, self.hist_throttle = [], []
        self._last_frame_count = 0

        self.value_labels = {}
        self.conn_lbl = app.build_topbar(self, "Run Test", status=True)

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=16, pady=(0, 16))

        self._build_sidebar(body)
        self._build_plots(body)
        self._refresh_name_preview()
        self._tick()

    # -- layout -----------------------------------------------------------
    # Everything the firmware reports, two readings per row, so a whole run can
    # be sanity-checked at a glance without leaving the test screen.
    READOUT = [
        ("Thrust",      "Thrust_g",      "{:.1f}", "g"),
        ("RPM",         "RPM",           "{:.0f}", ""),
        ("Torque push", "TorquePush_Nm", "{:.3f}", "Nm"),
        ("Torque pull", "TorquePull_Nm", "{:.3f}", "Nm"),
        ("Throttle",    "Throttle_pct",  "{:.0f}", "%"),
        ("PWM",         "Throttle_PWM",  "{:.0f}", "us"),
        ("Volts",       "Volts",         "{:.2f}", "V"),
        ("Amps",        "Amps",          "{:.2f}", "A"),
        ("Power",       "Watts",         "{:.0f}", "W"),
        ("Push cell",   "Push_g",        "{:.1f}", "g"),
        ("Pull cell",   "Pull_g",        "{:.1f}", "g"),
        ("Ambient",     "AmbTemp_C",     "{:.1f}", "C"),
        ("IR temp",     "IRTemp_C",      "{:.1f}", "C"),
        ("Humidity",    "Humidity_pct",  "{:.0f}", "%"),
        ("Pressure",    "Pressure_hPa",  "{:.0f}", "hPa"),
        ("Airspeed",    "Airspeed_mps",  "{:.1f}", "m/s"),
        ("Pitot",       "Pitot_mV",      "{:.0f}", "mV"),
        ("GPS sats",    "GPS_Sats",      "{:.0f}", ""),
    ]

    def _build_sidebar(self, master):
        bar = tk.Frame(master, bg=BG, width=400)
        bar.pack(side="left", fill="y")
        bar.pack_propagate(False)

        # Safety controls are packed against the bottom FIRST, so their space is
        # reserved before anything else competes for it — KILL can never be
        # pushed off-screen on a short display.
        FlatButton(bar, "KILL MOTOR (K)", self.app.emergency_stop, kind="danger",
                   font=F_TITLE).pack(side="bottom", fill="x", pady=(8, 0))
        FlatButton(bar, "TARE LOAD CELLS", self.link.tare, kind="normal",
                   font=F_SMALL).pack(side="bottom", fill="x", pady=(10, 0))

        self.scroll = ScrollFrame(bar)
        self.scroll.pack(side="top", fill="both", expand=True)
        col = self.scroll.body

        self._build_readout(col)
        self._build_run_setup(col)
        self.scroll.bind_wheel()

    def _build_readout(self, col):
        """Every telemetry field, two readings per row, one line each."""
        panel = card(col)
        panel.pack(fill="x")
        grid = panel.body

        for index, (caption, key, fmt, unit) in enumerate(self.READOUT):
            row, side = divmod(index, 2)
            base = side * 2
            big = index < 2                     # thrust and RPM lead the table
            font = ("Menlo", 14, "bold") if big else F_MONO_S
            tk.Label(grid, text=caption, bg=PANEL, fg=DIM, font=F_SMALL,
                     anchor="w").grid(row=row, column=base, sticky="w",
                                      padx=(2, 6), pady=(3 if big else 1))
            value = tk.Label(grid, text="---", bg=PANEL, fg=MUTED, font=font, anchor="w")
            value.grid(row=row, column=base + 1, sticky="w", padx=(0, 14))
            self.value_labels[key] = (value, fmt, unit)

        grid.columnconfigure(1, weight=1)
        grid.columnconfigure(3, weight=1)

    def _build_run_setup(self, col):
        run = card(col)
        run.pack(fill="x", pady=(9, 0))

        self.label_ent = labelled_entry(run.body, "Test note", "")
        self.label_ent.bind("<KeyRelease>", lambda _e: self._refresh_name_preview())

        tk.Label(run.body, text="Program", bg=PANEL, fg=DIM, font=F_SMALL,
                 anchor="w").pack(fill="x", pady=(10, 2))
        self.prog_sel = Dropdown(run.body, list(PROGRAMS.keys()))
        self.prog_sel.pack(fill="x")

        meta = tk.Frame(run.body, bg=PANEL)
        meta.pack(fill="x")
        left = tk.Frame(meta, bg=PANEL)
        left.pack(side="left", fill="x", expand=True, padx=(0, 5))
        right = tk.Frame(meta, bg=PANEL)
        right.pack(side="left", fill="x", expand=True, padx=(5, 0))
        self.prop_ent = labelled_entry(left, "Prop size", "10x4.5")
        self.batt_ent = labelled_entry(right, "Battery ID", "Pack_01")

        self.name_lbl = tk.Label(run.body, text="", bg=PANEL, fg=TEXT, font=F_BODY,
                                 anchor="w", wraplength=360, justify="left")
        self.name_lbl.pack(fill="x", pady=(10, 0))
        self.saved_lbl = tk.Label(run.body, text="", bg=PANEL, fg=DIM, font=F_SMALL,
                                  anchor="w", wraplength=360, justify="left")
        self.saved_lbl.pack(fill="x")

        self.status_lbl = tk.Label(run.body, text="", bg=PANEL, fg=DIM, font=F_MONO_S,
                                   anchor="w")
        self.status_lbl.pack(fill="x", pady=(8, 0))

        self.start_btn = FlatButton(run.body, "ARM & START TEST", self._arm_test,
                                    kind="accent", font=F_HEAD)
        self.start_btn.pack(fill="x", pady=(6, 0))

    def _build_plots(self, master):
        panel = tk.Frame(master, bg=BG)
        panel.pack(side="left", fill="both", expand=True, padx=(14, 0))

        bar = tk.Frame(panel, bg=BG)
        bar.pack(fill="x", pady=(0, 8))
        self.toggle_btn = FlatButton(bar, "VIEW: ROLLING WINDOW", self._toggle_history,
                                     kind="normal", font=F_SMALL)
        self.toggle_btn.pack(side="right")

        self.fig, axes = plt.subplots(4, 1, figsize=(6, 9), facecolor=BG, sharex=True)
        self.axes = axes
        titles = ["Throttle  %", "Thrust  g", "Torque push  Nm", "Torque pull  Nm"]
        self.lines = []
        for ax, title in zip(axes, titles):
            line, = ax.plot([], [], color=ACCENT, linewidth=1.4)
            self.lines.append(line)
            ax.set_facecolor(PANEL)
            ax.set_title(title, color=DIM, fontsize=9, loc="left", pad=6)
            ax.tick_params(colors=DIM, labelsize=8, length=0)
            ax.grid(color=LINE, linewidth=0.7)
            for spine in ax.spines.values():
                spine.set_color(LINE)
        self.fig.tight_layout(pad=1.6)

        self.canvas = FigureCanvasTkAgg(self.fig, master=panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _set_status(self, text, colour, hold=0.0):
        """Set the status line, optionally pinning it against the refresh tick."""
        self._status_hold = time.time() + hold
        self.status_lbl.config(text=text, fg=colour)

    # -- naming -----------------------------------------------------------
    def _refresh_name_preview(self):
        self.name_lbl.config(text=f"saves as  {next_run_path(self.label_ent.get()).name}")

    # -- controls ---------------------------------------------------------
    def _toggle_history(self):
        self.show_all = not self.show_all
        self.toggle_btn.set_text("VIEW: FULL HISTORY" if self.show_all
                                 else "VIEW: ROLLING WINDOW")

    def _arm_test(self):
        if not self.link.is_connected():
            self._set_status("NO BOARD CONNECTED", DANGER, hold=4)
            return
        self.start_btn.set_enabled(False)
        threading.Thread(target=self._countdown, daemon=True).start()

    def _countdown(self):
        self.counting_down = True
        for i in range(5, 0, -1):
            if not self.app.running or not self.winfo_exists():
                self.counting_down = False
                return
            self.after(0, self.status_lbl.config, {"text": f"ARMING  {i}s", "fg": DANGER})
            time.sleep(1)
        self.counting_down = False
        self._run_program()

    def _run_program(self):
        """
        Walk the selected program's (throttle %, duration s) steps, recording every
        frame the reader thread sees into the auto-named CSV.
        """
        self.test_active = True
        self.run_path = next_run_path(self.label_ent.get())
        self.after(0, self.status_lbl.config, {"text": "TEST ACTIVE", "fg": TEXT})

        try:
            with open(self.run_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["# Prop", self.prop_ent.get(),
                                 "# Batt", self.batt_ent.get(),
                                 "# Testing", self.label_ent.get(),
                                 "# Program", self.prog_sel.get(),
                                 "# Started", datetime.now().isoformat(timespec="seconds")])
                writer.writerow(["PC_Time"] + CORE_FIELDS)

                self.link.voltage_fault = None
                self.link.csv_writer = writer
                self.link.recording = True

                for pct, duration in PROGRAMS[self.prog_sel.get()]:
                    if not self.test_active or not self.app.running:
                        break
                    self.link.set_throttle_pct(pct)
                    end = time.time() + duration
                    while time.time() < end and self.test_active and self.app.running:
                        if self.link.voltage_fault:
                            volts = self.link.voltage_fault
                            self.after(0, self._voltage_abort, volts)
                            break
                        time.sleep(0.05)
                    if self.link.voltage_fault:
                        break
        except OSError as e:
            self.after(0, self._set_status, f"LOG ERROR: {e}", DANGER, 15)
        finally:
            self.link.recording = False
            self.link.csv_writer = None

        self.emergency_stop()
        if self.winfo_exists():
            self.after(0, self.start_btn.set_enabled, True)
            self.after(0, self._refresh_name_preview)
            if not self.link.voltage_fault and self.run_path:
                rows = self.run_path.exists() and self.run_path.stat().st_size > 0
                self.after(0, self._set_status,
                           "RUN COMPLETE" if rows else "RUN COMPLETE (no data)",
                           ACCENT if rows else DANGER, 8)
                self.after(0, self.saved_lbl.config,
                           {"text": f"last saved  {self.run_path.name}"})

    def _voltage_abort(self, volts):
        self.test_active = False
        self._set_status(f"ABORT — BUS SAG {volts:.1f} V", DANGER, hold=20)
        self.app.emergency_stop()

    def emergency_stop(self):
        self.test_active = False
        self.link.recording = False
        self.link.csv_writer = None

    # -- refresh ----------------------------------------------------------
    def _tick(self):
        if not self.winfo_exists():
            return
        frame, _buffered = self.link.snapshot()
        stale = self.link.is_stale()
        live = bool(frame) and not stale

        if not self.link.is_connected():
            self.conn_lbl.config(text="DISCONNECTED", fg=DANGER)
        elif stale:
            self.conn_lbl.config(text=f"NO TELEMETRY   {self.link.port_name}", fg=DANGER)
        else:
            self.conn_lbl.config(text=f"CONNECTED   {self.link.port_name}", fg=TEXT)

        for key, (widget, fmt, unit) in self.value_labels.items():
            value = frame.get(key) if frame else None
            if value is None or (isinstance(value, float) and math.isnan(value)):
                widget.config(text="---", fg=MUTED)
            else:
                text = fmt.format(value) + (f" {unit}" if unit else "")
                widget.config(text=text, fg=TEXT if live else MUTED)

        if live:
            # Only sample the plots when the reader has actually produced new
            # frames, so a stalled link stops drawing instead of flat-lining.
            if self.link.frame_count != self._last_frame_count:
                self._last_frame_count = self.link.frame_count
                for buf, hist, key in (
                    (self.plot_throttle, self.hist_throttle, "Throttle_pct"),
                    (self.plot_thrust, self.hist_thrust, "Thrust_g"),
                    (self.plot_tq_push, self.hist_tq_push, "TorquePush_Nm"),
                    (self.plot_tq_pull, self.hist_tq_pull, "TorquePull_Nm"),
                ):
                    buf.append(frame[key])
                    hist.append(frame[key])
                self._draw()

        if (not self.test_active and not self.counting_down
                and time.time() >= self._status_hold):
            self.status_lbl.config(text="", fg=DIM)

        self.after(50, self._tick)

    def _draw(self):
        sources = ((self.hist_throttle, self.plot_throttle),
                   (self.hist_thrust, self.plot_thrust),
                   (self.hist_tq_push, self.plot_tq_push),
                   (self.hist_tq_pull, self.plot_tq_pull))
        for line, ax, (hist, rolling) in zip(self.lines, self.axes, sources):
            data = hist if self.show_all else list(rolling)
            if len(data) > 1:
                line.set_data(range(len(data)), data)
                ax.relim()
                ax.autoscale_view()
        self.canvas.draw_idle()

    def close(self):
        self.test_active = False
        self.link.recording = False
        self.link.csv_writer = None
        plt.close(self.fig)


# --------------------------------------------------------------------------
# APP SHELL
# --------------------------------------------------------------------------

class App:
    """Owns the window, the serial link, and the swap between the two tools."""

    def __init__(self, root):
        self.root = root
        self.running = True
        self.view = None

        root.title("UAV Thrust Stand")
        root.configure(bg=BG)
        self._center(980, 660)

        self.link = SerialLink()
        self.link.start(root)

        root.bind("<k>", lambda _e: self.emergency_stop())
        root.bind("<K>", lambda _e: self.emergency_stop())
        root.protocol("WM_DELETE_WINDOW", self.shutdown)

        self.show(LauncherView)

    # -- window -----------------------------------------------------------
    def _center(self, width, height):
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        width, height = min(width, sw - 40), min(height, sh - 80)
        self.root.geometry(f"{width}x{height}+{(sw - width) // 2}+{max(20, (sh - height) // 3)}")

    def build_topbar(self, parent, title, status=False):
        """
        Shared header with the back-to-launcher control.

        With status=True it also builds the large connection readout on the
        right and returns that label for the view to keep updated.
        """
        bar = tk.Frame(parent, bg=BG)
        bar.pack(fill="x", padx=16, pady=14)
        FlatButton(bar, "BACK", self.open_launcher, kind="ghost",
                   font=F_SMALL).pack(side="left", padx=(0, 14))
        tk.Label(bar, text=title, bg=BG, fg=TEXT, font=F_TITLE).pack(side="left")

        readout = None
        if status:
            readout = tk.Label(bar, text="SEARCHING...", bg=BG, fg=DIM,
                               font=("Helvetica", 16, "bold"))
            readout.pack(side="right")
        tk.Frame(parent, bg=LINE, height=1).pack(fill="x")
        return readout

    # -- navigation -------------------------------------------------------
    def show(self, view_cls, size=None):
        if self.view is not None:
            self.view.close()
            self.view.destroy()
        if size:
            self._center(*size)
        self.view = view_cls(self.root, self)
        self.view.pack(fill="both", expand=True)

    def open_launcher(self):
        self.emergency_stop()
        self.show(LauncherView, (980, 660))

    def open_propeller(self):
        self.show(PropellerTestView, (1360, 900))

    def open_hardware(self):
        self.show(HardwareTestView, (1360, 820))

    # -- safety -----------------------------------------------------------
    def emergency_stop(self):
        """Idle the ESC and stand every view down. Bound to K everywhere."""
        self.link.idle()
        if self.view is not None and hasattr(self.view, "emergency_stop"):
            self.view.emergency_stop()

    def shutdown(self):
        self.running = False
        if self.view is not None:
            self.view.close()
        self.link.shutdown()
        self.root.quit()
        self.root.destroy()
        os._exit(0)


if __name__ == "__main__":
    App(tk.Tk()).root.mainloop()
