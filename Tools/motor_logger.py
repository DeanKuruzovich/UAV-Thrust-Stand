"""
motor_logger.py — RPI DBF Propulsion Test Hub (GUI)
=====================================================

A desktop GUI for the UAV Thrust Stand.  Connects to the ESP32-S3 over USB
CDC Serial, streams live telemetry (thrust, torque, RPM, voltage, current),
controls the ESC throttle, and records test runs to timestamped CSV files.

Usage
-----
    python motor_logger.py

The script auto-detects the ESP32 on any available COM / tty port.  Baud rate
must match the firmware (921 600 bps).

CSV output format (one row per 50 Hz sample)
--------------------------------------------
    PC_Time, ESP_ms, Thrust_g, TorquePush_Nm, TorquePull_Nm, Throttle_PWM,
    Push_g, Pull_g, Volts, Amps, RPM

Keyboard shortcut
-----------------
    K — emergency stop (sets throttle to 1000 µs / idle)

Dependencies
------------
    pip install pyserial matplotlib   (see requirements.txt)
"""

import serial
import serial.tools.list_ports
import time
import csv
import threading
import os
import math
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

# --- CONFIG ---
BAUD = 921600
MAX_PLOT_POINTS = 150
PROGRAMS = {
    "Quick Ramp Short": [(0, 5), (30, 5), (40, 5), (50, 5), (60, 5), (70, 5),  (80, 5), (90, 5), (100, 5), (0, 5)],
    "Quick Ramp": [(0, 10), (30, 10), (40, 10), (50, 10), (60, 10), (70, 10),  (80, 10), (90, 10), (100, 10), (0, 10)],
    "Step Test":  [(0, 2), (20, 5), (40, 5), (60, 5), (80, 5), (100, 5), (0, 2)],
    "Burst":      [(0, 5), (100, 2), (0, 2)],
    "Manual Mode": []
}

class MotorLabGUI:
    """
    Main application window for the UAV Thrust Stand.

    Responsibilities:
    - Auto-discovers and opens the ESP32 serial port.
    - Spawns a background thread that continuously reads CSV lines from the
      firmware and updates shared state protected by a lock.
    - Renders a live Tkinter GUI with four real-time Matplotlib plots
      (throttle %, thrust, torque push, torque pull).
    - Provides manual throttle control (slider), automated test programs,
      TARE command, and an emergency-stop button / keyboard shortcut.
    - Logs test runs to CSV with metadata header (prop, battery, humidity).
    - Shows live auxiliary sensor diagnostics (ambient temp, IR/laser temp, pitot
      airspeed, GPS heading/fix/sats) parsed from the firmware's extra CSV fields;
      these are displayed only and are not yet written to the recorded log.
    """

    def __init__(self, root):
        self.root = root
        self.root.title("RPI DBF - Propulsion Test Hub (v2.6)")
        # Clamp to the actual screen so the bottom of the sidebar (KILL button)
        # never falls below the display on smaller laptop screens.
        sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
        self.root.geometry(f"{min(1400, sw - 40)}x{min(950, sh - 80)}")
        self.root.configure(bg="#1e272e")

        self.ser = None
        self.running = True
        self.logging = False
        self.test_active = False
        self.counting_down = False

        self.latest = None
        self.latest_extra = None   # (amb_C, humi_pct, press_hPa, ir_C, pitot_mV, gps_lat, gps_lon, gps_speed_mps, gps_fix, gps_sats)
        self.latest_lock = threading.Lock()

        # Plot buffers (rolling window)
        self.plot_thrust      = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_torque_push = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_torque_pull = deque(maxlen=MAX_PLOT_POINTS)
        self.plot_throttle    = deque(maxlen=MAX_PLOT_POINTS)

        # Full history buffers
        self.hist_thrust      = []
        self.hist_torque_push = []
        self.hist_torque_pull = []
        self.hist_throttle    = []

        self.show_all = False

        self.setup_ui()
        self.find_serial()

    def setup_ui(self):
        """Build all Tkinter widgets: sidebar telemetry, controls, and plot panel."""
        # --- LEFT SIDEBAR ---
        self.sidebar = tk.Frame(self.root, bg="#2f3640", width=350)
        self.sidebar.pack(side="left", fill="y", padx=5, pady=5)

        # KILL button is packed FIRST so its space at the bottom of the sidebar is
        # reserved before everything else — it can never get pushed off-screen.
        # Dark bold text stays legible on macOS (which ignores Button bg) and on
        # the red fill elsewhere.
        self.kill_btn = tk.Button(self.sidebar, text="KILL MOTOR (K)", command=self.emergency_stop,
                                  bg="#ff5e57", fg="#1e272e", activeforeground="#1e272e",
                                  font=("Arial", 14, "bold"), height=2)
        self.kill_btn.pack(side="bottom", fill="x", padx=10, pady=10)

        tk.Label(self.sidebar, text="TELEMETRY", font=("Arial", 12, "bold"),
                 bg="#2f3640", fg="white").pack(pady=5)

        self.thrust_lbl = tk.Label(self.sidebar, text="0.0 g",
                                   font=("Arial", 36, "bold"), bg="#2f3640", fg="#4cd137")
        self.thrust_lbl.pack()

        self.torque_lbl = tk.Label(self.sidebar, text="0.000 Nm",
                                   font=("Arial", 18, "bold"), bg="#2f3640", fg="#00a8ff")
        self.torque_lbl.pack()

        self.rpm_lbl = tk.Label(self.sidebar, text="0 RPM",
                                font=("Arial", 22, "bold"), bg="#2f3640", fg="#f1c40f")
        self.rpm_lbl.pack()

        self.status_bar = tk.Label(self.sidebar, text="DISCONNECTED",
                                   font=("Arial", 12, "bold"), bg="#2f3640", fg="#e74c3c")
        self.status_bar.pack(pady=5)

        # THROTTLE SLIDER
        man_frame = tk.LabelFrame(self.sidebar, text=" Throttle Control ",
                                  bg="#2f3640", fg="white", padx=10, pady=5)
        man_frame.pack(fill="x", padx=10, pady=5)
        self.throt_val = tk.IntVar(value=0)
        self.slider = tk.Scale(man_frame, from_=100, to=0, orient="vertical", length=150,
                               variable=self.throt_val, command=self.update_manual_throttle,
                               bg="#34495e", fg="white", highlightthickness=0, troughcolor="#2c3e50")
        self.slider.pack(side="left", padx=20)

        # BUS DIAGNOSTICS
        diag_f = tk.LabelFrame(self.sidebar, text=" Diagnostics ", bg="#2f3640", fg="white")
        diag_f.pack(fill="x", padx=10, pady=5)
        self.pwr_raw  = tk.Label(diag_f, text="0.0V | 0.0A | 0.0W", bg="#2f3640", fg="#bdc3c7", font=("Courier", 10))
        self.pwr_raw.pack(anchor="w", padx=10)
        self.latency_lbl = tk.Label(diag_f, text="Buffer: 0 frames", bg="#2f3640", fg="#7f8c8d", font=("Courier", 9))
        self.latency_lbl.pack(anchor="w", padx=10)

        # AUX SENSORS (shown live, NOT recorded to CSV yet)
        tk.Label(diag_f, text="— aux sensors —", bg="#2f3640", fg="#576574",
                 font=("Courier", 8)).pack(anchor="w", padx=10, pady=(4, 0))
        self.rpm_diag_lbl = tk.Label(diag_f, text="RPM sensor:  --- rpm", bg="#2f3640", fg="#f1c40f", font=("Courier", 9))
        self.rpm_diag_lbl.pack(anchor="w", padx=10)
        self.temp_diag_lbl = tk.Label(diag_f, text="Ambient: --.- C | IR/laser: --.- C", bg="#2f3640", fg="#74b9ff", font=("Courier", 9))
        self.temp_diag_lbl.pack(anchor="w", padx=10)
        self.env_diag_lbl = tk.Label(diag_f, text="Humidity: --.- % | Press: ---- hPa", bg="#2f3640", fg="#74b9ff", font=("Courier", 9))
        self.env_diag_lbl.pack(anchor="w", padx=10)
        self.air_diag_lbl = tk.Label(diag_f, text="Pitot: --.- m/s (---- mV)", bg="#2f3640", fg="#55efc4", font=("Courier", 9))
        self.air_diag_lbl.pack(anchor="w", padx=10)
        self.gps_diag_lbl = tk.Label(diag_f, text="GPS: nofix | 0 sats | --.- m/s", bg="#2f3640", fg="#a29bfe", font=("Courier", 9))
        self.gps_diag_lbl.pack(anchor="w", padx=10, pady=(0, 4))

        # AUTOMATED TEST
        prog_f = tk.LabelFrame(self.sidebar, text=" Automated Program ", bg="#2f3640", fg="white")
        prog_f.pack(fill="x", padx=10, pady=5)
        self.file_ent = tk.Entry(prog_f)
        self.file_ent.insert(0, "DBF_Test_Run")
        self.file_ent.pack(fill="x", padx=10, pady=5)
        self.prog_sel = ttk.Combobox(prog_f, values=list(PROGRAMS.keys()), state="readonly")
        self.prog_sel.current(0)
        self.prog_sel.pack(fill="x", padx=10, pady=2)
        self.start_btn = tk.Button(prog_f, text="ARM & START TEST", command=self.arm_test,
                                   bg="#e1b12c", fg="#1e272e", activeforeground="#1e272e",
                                   font=("Arial", 10, "bold"))
        self.start_btn.pack(fill="x", pady=10, padx=10)

        # SYSTEM & METADATA
        sys_f = tk.LabelFrame(self.sidebar, text=" Config & Hardware ", bg="#2f3640", fg="white")
        sys_f.pack(fill="x", padx=10, pady=5)
        
        tk.Button(sys_f, text="TARE (ZERO)", command=self.send_zero,
                  bg="#dcdde1", fg="#1e272e", activeforeground="#1e272e",
                  font=("Arial", 10, "bold")).pack(fill="x", pady=5, padx=5)

        def make_meta_entry(label_text, default_val):
            frame = tk.Frame(sys_f, bg="#2f3640")
            frame.pack(fill="x", padx=5, pady=2)
            tk.Label(frame, text=label_text, bg="#2f3640", fg="#bdc3c7", font=("Arial", 8)).pack(anchor="w")
            ent = tk.Entry(frame, bg="#34495e", fg="white", insertbackground="white", borderwidth=0)
            ent.insert(0, default_val)
            ent.pack(fill="x")
            return ent

        self.prop_ent = make_meta_entry("Prop Size:", "10x4.5")
        self.batt_ent = make_meta_entry("Battery ID:", "Pack_01")
        self.humi_ent = make_meta_entry("Humidity (%):", "50")

        # PLOT AREA
        self.plot_panel = tk.Frame(self.root, bg="#1e272e")
        self.plot_panel.pack(side="right", fill="both", expand=True)
        self.setup_graphs()

    def setup_graphs(self):
        """Create the four-panel Matplotlib figure embedded in the plot panel."""
        self.fig, (self.ax_p, self.ax_f, self.ax_qp, self.ax_ql) = plt.subplots(4, 1, figsize=(6, 10), facecolor='#1e272e')
        self.fig.tight_layout(pad=3.0)
        self.ln_p,  = self.ax_p.plot([], [], color="#f1c40f", label="Throttle %")
        self.ln_f,  = self.ax_f.plot([], [], color="#4cd137", label="Thrust (g)")
        self.ln_qp, = self.ax_qp.plot([], [], color="#e74c3c", label="Torque Push (Nm)")
        self.ln_ql, = self.ax_ql.plot([], [], color="#9b59b6", label="Torque Pull (Nm)")

        for ax in [self.ax_p, self.ax_f, self.ax_qp, self.ax_ql]:
            ax.set_facecolor('#2f3640')
            ax.tick_params(colors='white', labelsize=8)
            ax.grid(color='#7f8c8d', linestyle='--', alpha=0.1)
            ax.legend(loc="upper right", fontsize=7)

        btn_bar = tk.Frame(self.plot_panel, bg="#1e272e")
        btn_bar.pack(fill="x", padx=5, pady=2)
        self.toggle_btn = tk.Button(
            btn_bar, text="SHOW: ROLLING WINDOW",
            command=self.toggle_history_mode,
            bg="#a4b0be", fg="#1e272e", activeforeground="#1e272e",
            font=("Arial", 9, "bold")
        )
        self.toggle_btn.pack(side="right", padx=5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_panel)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def read_thread(self):
        """
        Background daemon thread: drains the serial buffer and parses CSV lines.

        Keeps only the most recent complete frame per GUI cycle to avoid latency
        build-up.  Triggers an emergency stop if bus voltage drops below
        VOLTAGE_CUTOFF while a test is active (LiPo cell-sag safety).
        """
        VOLTAGE_CUTOFF = 26.4
        while self.running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                line = None
                frames_this_cycle = 0
                while self.ser.in_waiting > 0 and self.running:
                    candidate = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if candidate and ',' in candidate:
                        line = candidate
                        frames_this_cycle += 1

                if line:
                    data = line.split(",")
                    if len(data) >= 10:
                        t, qp, ql, pwm, p, l, v, a, r = map(float, data[1:10])

                        if self.test_active and v < VOLTAGE_CUTOFF and v > 1.0:
                            self.root.after(0, self.emergency_stop)
                            self.root.after(0, messagebox.showwarning, "Safety", f"Voltage Drop: {v}V")

                        # Auxiliary diagnostic fields (amb temp, humidity, pressure,
                        # IR temp, pitot mV, GPS lat/lon/speed/fix/sats) — shown live
                        # but NOT recorded to CSV.
                        extra = None
                        if len(data) >= 20:
                            try:
                                extra = (float(data[10]), float(data[11]), float(data[12]),
                                         float(data[13]), float(data[14]), float(data[15]),
                                         float(data[16]), float(data[17]),
                                         int(float(data[18])), int(float(data[19])))
                            except ValueError:
                                extra = None

                        with self.latest_lock:
                            self.latest = (t, qp, ql, r, v, a, (pwm-1000)/10.0, frames_this_cycle)
                            self.latest_extra = extra

                        self.plot_thrust.append(t)
                        self.plot_torque_push.append(qp)
                        self.plot_torque_pull.append(ql)
                        self.plot_throttle.append((pwm-1000)/10.0)

                        self.hist_thrust.append(t)
                        self.hist_torque_push.append(qp)
                        self.hist_torque_pull.append(ql)
                        self.hist_throttle.append((pwm-1000)/10.0)

                        if self.logging and self.logging_writer:
                            self.logging_writer.writerow([datetime.now().strftime("%H:%M:%S.%f")] + data[:10])
            except: break

    def update_gui(self):
        """
        Periodic Tkinter callback (every 50 ms / ~20 Hz) that refreshes labels
        and redraws the live plots from the latest telemetry snapshot.
        """
        if not self.running: return
        with self.latest_lock:
            snap = self.latest
            snap_extra = self.latest_extra
        if snap:
            t, qp, ql, r, v, a, throt, frames = snap
            self.thrust_lbl.config(text=f"{t:.1f} g")
            self.torque_lbl.config(text=f"Push: {qp:.3f} | Pull: {ql:.3f} Nm")
            self.rpm_lbl.config(text=f"{int(r)} RPM")
            self.pwr_raw.config(text=f"{v:.2f}V | {a:.2f}A | {v*a:.1f}W")
            self.latency_lbl.config(text=f"Buffer: {frames} frames", fg="#2ecc71" if frames <= 2 else "#e74c3c")

            # Auxiliary sensor diagnostics (firmware appends these after RPM)
            if snap_extra:
                amb, humi, press, ir, pitot_mv, _lat, _lon, speed, fix, sats = snap_extra
                self.rpm_diag_lbl.config(text=f"RPM sensor:  {int(r):>5d} rpm")
                self.temp_diag_lbl.config(text=f"Ambient: {self._fmt_temp(amb)} | IR/laser: {self._fmt_temp(ir)}")
                humi_s = "--.-" if math.isnan(humi) else f"{humi:4.1f}"
                press_s = "----" if math.isnan(press) else f"{press:6.1f}"
                self.env_diag_lbl.config(text=f"Humidity: {humi_s} % | Press: {press_s} hPa")
                air = self.pitot_airspeed(pitot_mv)
                self.air_diag_lbl.config(text=f"Pitot: {air:4.1f} m/s ({int(pitot_mv):>4d} mV)")
                vel_s = "--.-" if math.isnan(speed) else f"{speed:4.1f}"
                self.gps_diag_lbl.config(text=f"GPS: {'FIX  ' if fix else 'nofix'}| {sats} sats | {vel_s} m/s")

            buf_thrust  = self.hist_thrust      if self.show_all else list(self.plot_thrust)
            buf_tqpush  = self.hist_torque_push  if self.show_all else list(self.plot_torque_push)
            buf_tqpull  = self.hist_torque_pull  if self.show_all else list(self.plot_torque_pull)
            buf_throttle= self.hist_throttle     if self.show_all else list(self.plot_throttle)
            if len(buf_thrust) > 1:
                x = range(len(buf_thrust))
                self.ln_p.set_data(x, buf_throttle)
                self.ln_f.set_data(x, buf_thrust)
                self.ln_qp.set_data(x, buf_tqpush)
                self.ln_ql.set_data(x, buf_tqpull)
                for ax in [self.ax_p, self.ax_f, self.ax_qp, self.ax_ql]:
                    ax.relim(); ax.autoscale_view()
                self.canvas.draw_idle()
        self.root.after(50, self.update_gui)

    def toggle_history_mode(self):
        """Switch the plots between a rolling 150-sample window and full-run history."""
        self.show_all = not self.show_all
        if self.show_all:
            self.toggle_btn.config(text="SHOW: FULL HISTORY", bg="#7ed6df", fg="#0a3d62")
        else:
            self.toggle_btn.config(text="SHOW: ROLLING WINDOW", bg="#a4b0be", fg="#1e272e")

    def update_manual_throttle(self, val):
        """
        Slider callback: sends a PWM value (1000–2000 µs) to the ESP32.
        Disabled during automated test runs and countdown sequences.
        """
        if self.ser and not self.test_active and not self.counting_down:
            pwm = 1000 + (int(val) * 10)
            self.ser.write(f"{pwm}\n".encode())

    def arm_test(self):
        """
        Kick off the 5-second arming countdown in a daemon thread, then run
        the selected automated test program.
        """
        self.start_btn.config(state="disabled")
        threading.Thread(target=self.countdown_proc, daemon=True).start()

    def countdown_proc(self):
        self.counting_down = True
        for i in range(5, 0, -1):
            if not self.running: return
            self.root.after(0, self.status_bar.config, {"text": f"ARMING: {i}s", "fg": "#e74c3c"})
            time.sleep(1)
        self.counting_down = False
        self.run_auto_test()

    def run_auto_test(self):
        """
        Execute the selected PROGRAMS sequence.

        For each (throttle_percent, duration_s) step:
        1. Sends the corresponding PWM value to the ESP32.
        2. Waits for the step duration while logging incoming CSV rows.
        Writes a metadata header row and a data header row at the top of the
        output file before logging begins.
        """
        self.test_active = True
        self.root.after(0, self.status_bar.config, {"text": "TEST ACTIVE", "fg": "#2ecc71"})
        fname = f"{self.file_ent.get()}_{datetime.now().strftime('%H%M%S')}.csv"
        
        meta = {"Prop": self.prop_ent.get(), "Batt": self.batt_ent.get(), "Humi": self.humi_ent.get()}

        try:
            with open(fname, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["# Prop", meta["Prop"], "# Batt", meta["Batt"], "# Humi", meta["Humi"]])
                writer.writerow(["PC_Time", "ESP_ms", "Thrust_g", "TorquePush_Nm", "TorquePull_Nm", "Throttle_PWM", "Push_g", "Pull_g", "Volts", "Amps", "RPM"])
                self.logging = True
                self.logging_writer = writer
                
                for thr_pct, dur in PROGRAMS[self.prog_sel.get()]:
                    if not self.test_active or not self.running: break
                    if self.ser: self.ser.write(f"{1000 + (thr_pct * 10)}\n".encode())
                    
                    end_t = time.time() + dur
                    while time.time() < end_t and self.test_active and self.running:
                        time.sleep(0.1)
        except Exception as e: print(f"Log Error: {e}")

        self.emergency_stop()
        self.root.after(0, self.start_btn.config, {"state": "normal"})
        self.root.after(0, self.status_bar.config, {"text": "SYSTEM READY", "fg": "#f1c40f"})

    def find_serial(self):
        """
        Scan available serial ports for any device matching common ESP32 / USB-UART
        identifiers.  Retries every 2 seconds until a connection is established.
        """
        ports = list(serial.tools.list_ports.comports())
        # Prefer the ESP32-S3 native USB CDC (Espressif VID 0x303A); never grab
        # the ADALM-2000 (Analog Devices VID 0x0456), which also shows up as a
        # usbmodem and would otherwise match the generic "USB" keyword below.
        ports.sort(key=lambda p: 0 if p.vid == 0x303A else 1)
        for p in ports:
            if p.vid == 0x0456:
                continue
            if p.vid == 0x303A or any(x in (p.description + (p.hwid or "")) for x in ["USB", "ESP32", "UART", "CP210", "CH340", "FTDI"]):
                try:
                    self.ser = serial.Serial(p.device, BAUD, timeout=0.1)
                    self.status_bar.config(text=f"CONNECTED: {p.device}", fg="#2ecc71")
                    threading.Thread(target=self.read_thread, daemon=True).start()
                    self.root.after(100, self.update_gui)
                    return
                except: continue
        self.root.after(2000, self.find_serial)

    @staticmethod
    def _fmt_temp(c):
        """Format a Celsius reading, showing dashes for NaN (sensor offline)."""
        return "--.- C" if (c is None or math.isnan(c)) else f"{c:5.1f} C"

    # Pitot front-end: DIY Drones Airspeed Sensor V2.0 (MPXV7002DP), ratiometric
    # to 5 V with ~2.5 V at zero flow, behind a 10k-10k divider on the ADC line.
    PITOT_DIVIDER = 2.0   # 10k-10k divider halves the signal -> true Vout = 2 x ADC
    PITOT_VSUPPLY = 5.0   # MPXV7002DP supply voltage
    AIR_DENSITY   = 1.225 # kg/m^3 (sea-level, 15 C)

    @classmethod
    def pitot_airspeed(cls, mv):
        """
        Approximate airspeed (m/s) from the raw pitot ADC millivolts.

        Reconstructs the true MPXV7002DP output by undoing the 10k-10k divider,
        applies the sensor transfer (Vout = Vs*(0.2*kPa + 0.5)) to get differential
        pressure, then v = sqrt(2*dP/rho).  Returns 0 at/below the zero-flow point.
        Best-effort / uncalibrated — fine for a bench "does it respond" check.
        """
        try:
            vout = (mv * cls.PITOT_DIVIDER) / 1000.0             # true sensor output, volts
            dp_pa = (vout / cls.PITOT_VSUPPLY - 0.5) / 0.2 * 1000.0   # differential pressure, Pa
            if dp_pa <= 0:
                return 0.0
            return (2.0 * dp_pa / cls.AIR_DENSITY) ** 0.5        # v = sqrt(2*dP/rho)
        except (TypeError, ValueError):
            return 0.0

    def send_zero(self):
        """Send the 'Z' tare command to zero all three load cells on the ESP32."""
        if self.ser: self.ser.write(b"Z\n")

    def emergency_stop(self):
        """Immediately set throttle to idle (1000 µs) and halt logging."""
        
        self.test_active = False
        self.logging = False
        self.throt_val.set(0)
        if self.ser: self.ser.write(b"1000\n")

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorLabGUI(root)
    root.bind('<k>', lambda e: app.emergency_stop())

    def on_closing():
        app.running = False
        app.test_active = False
        if app.ser and app.ser.is_open:
            try:
                app.ser.write(b"1000\n")
                time.sleep(0.1)
                app.ser.close()
            except: pass
        root.quit()
        root.destroy()
        os._exit(0)

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()