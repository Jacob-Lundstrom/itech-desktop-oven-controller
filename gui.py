#!/usr/bin/env python3
"""
Reflow oven GUI: live temperature chart, profile runner, manual control and CSV recording.

    python gui.py            (Windows: double-click gui.py, or `py gui.py`)
    python gui.py --sim      start with the simulator selected

Needs only Python 3 (tkinter is included) + pyserial.
"""
from __future__ import annotations

import csv
import datetime as dt
import json
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from calibration import Calibration
from oven import OUT_35, OUT_36, Oven, OvenError
from reflow import DEFAULT_PROFILE, LeadPI, interp

try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

PERIOD = 0.5  # s, same as the vendor app

CSV_FIELDS = ["timestamp", "t_s", "mode", "phase", "setpoint_C", "temp_oven_C", "temp_board_est_C",
              "predicted_C", "heater_pct", "fan_speed_pct", "fan_enable"]


# =========================================================================== worker
class SimClock:
    """Virtual time for the simulator, running `speed` x faster than real time."""

    def __init__(self, speed=1.0):
        self.t = 0.0
        self.speed = speed

    def now(self):
        return self.t

    def sleep(self, d):
        time.sleep(d / self.speed)
        self.t += d


class RealClock:
    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


class Worker(threading.Thread):
    """Owns the serial port. The GUI only changes `self.cmd` (under `self.lock`)
    and reads samples from `self.q`."""

    def __init__(self, port, sim, sim_speed, q):
        super().__init__(daemon=True)
        self.port, self.sim, self.sim_speed, self.q = port, sim, sim_speed, q
        self.lock = threading.Lock()
        self.cmd = dict(mode="monitor", heater=0, o35=0, o36=0,
                        profile=None, gains=None, run_o35=100, run_o36=1,
                        max_temp=260, cool_to=80, start_run=False,
                        cal=None, cal_apply=False, hold_sp=150)
        self.stop_flag = threading.Event()

    def set(self, **kw):
        with self.lock:
            self.cmd.update(kw)

    def emit(self, kind, **data):
        self.q.put((kind, data))

    def run(self):
        try:
            if self.sim:
                from sim import FakeOvenSerial
                clock = SimClock(self.sim_speed)
                ov = Oven(FakeOvenSerial(clock))
            else:
                clock = RealClock()
                ov = Oven(self.port)
            ov.link()
            ov.safe_off()
        except Exception as e:
            self.emit("error", msg=f"Could not connect: {e}")
            self.emit("disconnected")
            return
        self.emit("connected")

        t0 = clock.now()
        run_t0 = None
        phase = "-"
        ctrl = None
        errors = 0
        last_out = (None, None, None)
        try:
            while not self.stop_flag.is_set():
                loop_start = clock.now()
                with self.lock:
                    c = dict(self.cmd)
                    if c["start_run"]:
                        self.cmd["start_run"] = False
                    if c.get("reset_t0"):
                        self.cmd["reset_t0"] = False
                if c.get("reset_t0"):
                    t0 = clock.now()
                if c["start_run"]:
                    run_t0 = clock.now()
                    k = c["gains"]
                    ctrl = LeadPI(k["kp"], k["ki"], k["kff"], k["lead"])
                    phase = "hold" if c["mode"] == "hold" else "profile"
                    last_out = (None, None, None)

                try:
                    T = ov.read_temp()
                    errors = 0
                except OvenError as e:
                    errors += 1
                    self.emit("warn", msg=str(e))
                    if errors >= 3:
                        raise
                    ov.heater(0)
                    clock.sleep(PERIOD)
                    continue

                t = clock.now() - t0
                sp = pred = None
                mode = c["mode"]
                cal = c["cal"]
                Tb = cal.board(T) if cal else None     # estimated board temperature
                Tc = Tb if (Tb is not None and c["cal_apply"]) else T   # what the profile controller regulates

                if max(T, Tb if Tb is not None else T) >= c["max_temp"] and mode != "monitor":
                    self.set(mode="monitor")
                    mode = "monitor"
                    phase = "-"
                    self.emit("error", msg=f"Over-temperature ({T} C >= {c['max_temp']} C). Heater off.")
                    last_out = (None, None, None)

                if mode == "monitor":
                    heat, o35, o36 = 0, 0, 0
                    phase = "-"
                elif mode == "manual":
                    heat, o35, o36 = c["heater"], c["o35"], c["o36"]
                    phase = "manual"
                elif mode == "hold":  # closed-loop hold on the RAW oven reading (for calibration)
                    sp = c["hold_sp"]
                    heat, pred = ctrl.step(clock.now() - run_t0, PERIOD, sp, 0.0, T)
                    o35, o36 = c["run_o35"], c["run_o36"]
                    phase = "hold"
                else:  # profile (in board temperature when a calibration is loaded)
                    prof = c["profile"]
                    tr = clock.now() - run_t0
                    if phase == "profile" and tr > prof[-1][0]:
                        phase = "cool"
                    if phase == "profile":
                        sp = interp(prof, tr)
                        slope = (interp(prof, tr + 1) - interp(prof, tr - 1)) / 2
                        heat, pred = ctrl.step(tr, PERIOD, sp, slope, Tc)
                        o35, o36 = c["run_o35"], c["run_o36"]
                    else:
                        sp = c["cool_to"]
                        heat, o35, o36 = 0, 100, 1
                        if Tc <= c["cool_to"]:
                            self.set(mode="monitor")
                            self.emit("run_done")
                            phase = "done"

                heat = round(heat)
                # only send outputs that changed (heater every loop, as a keep-alive)
                ov.heater(heat)
                if o35 != last_out[1]:
                    ov.set_output(OUT_35, o35)
                if o36 != last_out[2]:
                    ov.set_output(OUT_36, o36)
                last_out = (heat, o35, o36)

                self.emit("sample", wall=dt.datetime.now().isoformat(timespec="milliseconds"),
                          t=t, run_t=(clock.now() - run_t0) if mode == "profile" and run_t0 is not None else None,
                          mode=mode, phase=phase, sp=sp, temp=T, board=Tb, pred=pred,
                          heater=heat, o35=o35, o36=o36)

                rest = PERIOD - (clock.now() - loop_start)
                clock.sleep(max(0.05, rest))
        except Exception as e:
            self.emit("error", msg=f"Connection lost: {e}")
        finally:
            ov.safe_off()
            try:
                ov.unlink()
            except Exception:
                pass
            ov.close()
            self.emit("disconnected")


# =========================================================================== chart
class Chart(tk.Canvas):
    PAD_L, PAD_R, PAD_T, PAD_B = 52, 52, 16, 34

    def __init__(self, master, **kw):
        super().__init__(master, background="#ffffff", highlightthickness=0, **kw)
        self.samples = []         # dicts from the worker
        self.profile = None       # [[t, C], ...] plotted from run start
        self.run_offset = None    # chart time at which the run started
        self.bind("<Configure>", lambda e: self.redraw())

    def clear(self):
        self.samples.clear()
        self.run_offset = None
        self.redraw()

    def redraw(self):
        self.delete("all")
        W, H = self.winfo_width(), self.winfo_height()
        if W < 100 or H < 100:
            return
        L, R, T, B = self.PAD_L, W - self.PAD_R, self.PAD_T, H - self.PAD_B

        t_max = 300.0
        if self.samples:
            t_max = max(t_max, self.samples[-1]["t"] + 30)
        if self.profile:
            off = self.run_offset if self.run_offset is not None else (self.samples[-1]["t"] if self.samples else 0)
            t_max = max(t_max, off + self.profile[-1][0] + 60)
        t_min = 0.0
        y_max = 260.0
        if self.samples:
            y_max = max(y_max, max(s["temp"] for s in self.samples) + 10)

        def X(t):
            return L + (t - t_min) / (t_max - t_min) * (R - L)

        def Y(c):
            return B - c / y_max * (B - T)

        def Yp(p):
            return B - p / 100.0 * (B - T)

        # grid
        step_t = 30 if t_max <= 600 else 60 if t_max <= 1200 else 300
        for tt in range(0, int(t_max) + 1, step_t):
            x = X(tt)
            self.create_line(x, T, x, B, fill="#eeeeee")
            self.create_text(x, B + 12, text=f"{tt}", fill="#666", font=("Segoe UI", 8))
        for c in range(0, int(y_max) + 1, 50):
            y = Y(c)
            self.create_line(L, y, R, y, fill="#eeeeee")
            self.create_text(L - 8, y, text=f"{c}", anchor="e", fill="#c0392b", font=("Segoe UI", 8))
        for p in range(0, 101, 25):
            self.create_text(R + 8, Yp(p), text=f"{p}%", anchor="w", fill="#2471a3", font=("Segoe UI", 8))
        self.create_rectangle(L, T, R, B, outline="#999")
        self.create_text((L + R) / 2, H - 6, text="time (s)", fill="#666", font=("Segoe UI", 8))

        # profile preview
        if self.profile:
            off = self.run_offset if self.run_offset is not None else (self.samples[-1]["t"] if self.samples else 0)
            pts = []
            # clip to t >= 0 (a run can start before the chart was cleared)
            if off + self.profile[-1][0] > 0:
                if off < 0:
                    pts += [X(0), Y(interp(self.profile, -off))]
                for tt, cc in self.profile:
                    if off + tt >= 0:
                        pts += [X(off + tt), Y(cc)]
            if len(pts) >= 4:
                self.create_line(*pts, fill="#bbbbbb", width=6, capstyle="round", joinstyle="round")

        if len(self.samples) >= 2:
            s = self.samples
            self._line(s, "heater", X, Yp, "#f39c12", 1.5)
            self._line(s, "o35", X, Yp, "#2471a3", 1.5, dash=(4, 3))
            self._line(s, "sp", X, Y, "#7f8c8d", 1.5, dash=(6, 3))
            self._line(s, "temp", X, Y, "#c0392b", 2.5)
            self._line(s, "board", X, Y, "#7d3c98", 2.5)

        # legend
        items = [("Oven sensor", "#c0392b", None)]
        if any(d.get("board") is not None for d in self.samples):
            items.append(("Board (cal.)", "#7d3c98", None))
        items += [("Setpoint", "#7f8c8d", (6, 3)),
                 ("Profile", "#bbbbbb", None), ("Heater %", "#f39c12", None), ("Exhaust fan %", "#2471a3", (4, 3))]
        x = L + 10
        for name, col, dash in items:
            self.create_line(x, T + 10, x + 18, T + 10, fill=col, width=3 if name == "Profile" else 2, dash=dash)
            tid = self.create_text(x + 22, T + 10, text=name, anchor="w", font=("Segoe UI", 9))
            x = self.bbox(tid)[2] + 14

    def _line(self, s, key, X, Y, col, w, dash=None):
        seg = []
        for d in s:
            v = d.get(key)
            if v is None:
                if len(seg) >= 4:
                    self.create_line(*seg, fill=col, width=w, dash=dash)
                seg = []
                continue
            seg += [X(d["t"]), Y(v)]
        if len(seg) >= 4:
            self.create_line(*seg, fill=col, width=w, dash=dash)


# =========================================================================== app
class App(tk.Tk):
    def __init__(self, sim_default=False):
        super().__init__()
        self.title("Reflow Oven Controller")
        self.geometry("1150x720")
        self.minsize(900, 600)
        self.q = queue.Queue()
        self.worker = None
        self.profile = [list(p) for p in DEFAULT_PROFILE]
        self.profile_name = "built-in SAC305"
        self.rec_file = None
        self.rec_writer = None
        self.rec_count = 0
        self._build(sim_default)
        self.update_preview()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.poll)

    # ------------------------------------------------------------------ layout
    def _build(self, sim_default):
        style = ttk.Style(self)
        try:
            style.theme_use("vista" if sys.platform == "win32" else "clam")
        except tk.TclError:
            pass

        side = ttk.Frame(self, padding=10)
        side.pack(side="left", fill="y")
        main = ttk.Frame(self, padding=(0, 10, 10, 10))
        main.pack(side="left", fill="both", expand=True)

        # connection
        f = ttk.LabelFrame(side, text="Connection", padding=8)
        f.pack(fill="x", pady=(0, 8))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(f, textvariable=self.port_var, width=14)
        self.port_box.grid(row=0, column=0, sticky="ew")
        ttk.Button(f, text="↻", width=3, command=self.refresh_ports).grid(row=0, column=1, padx=(4, 0))
        self.sim_var = tk.BooleanVar(value=sim_default)
        ttk.Checkbutton(f, text="Simulator", variable=self.sim_var).grid(row=1, column=0, sticky="w", pady=4)
        self.speed_var = tk.StringVar(value="5")
        sp = ttk.Frame(f)
        sp.grid(row=1, column=1, sticky="e")
        ttk.Label(sp, text="×").pack(side="left")
        ttk.Spinbox(sp, from_=1, to=20, width=3, textvariable=self.speed_var).pack(side="left")
        self.conn_btn = ttk.Button(f, text="Connect", command=self.toggle_connect)
        self.conn_btn.grid(row=2, column=0, columnspan=2, sticky="ew")
        self.refresh_ports()

        nb = self.nb = ttk.Notebook(side)
        nb.pack(fill="x", pady=(0, 8))
        nb.bind("<<NotebookTabChanged>>", lambda e: self.update_preview())
        tab_run = ttk.Frame(nb, padding=8)
        tab_man = ttk.Frame(nb, padding=8)
        tab_cal = ttk.Frame(nb, padding=8)
        nb.add(tab_run, text="Profile")
        nb.add(tab_man, text="Manual")
        nb.add(tab_cal, text="Calibrate")

        # profile
        f = tab_run
        self.prof_label = ttk.Label(f, text=self.profile_name, width=24)
        self.prof_label.grid(row=0, column=0, columnspan=2, sticky="w")
        pf = ttk.Frame(f)
        pf.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 6))
        ttk.Button(pf, text="Load profile…", command=self.load_profile).pack(side="left", expand=True, fill="x")
        self.preview_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pf, text="Preview profile", variable=self.preview_var,
                        command=self.update_preview).pack(side="left", padx=(6, 0))
        self.run_o36 = tk.IntVar(value=1)
        self.run_o35 = tk.StringVar(value="100")
        ttk.Checkbutton(f, text="Convection fan on during run", variable=self.run_o36).grid(row=2, column=0, columnspan=2, sticky="w")
        ttk.Label(f, text="Exhaust fan % during run").grid(row=3, column=0, sticky="w")
        ttk.Spinbox(f, from_=0, to=100, increment=10, width=5, textvariable=self.run_o35).grid(row=3, column=1, sticky="e")
        self.gain_vars = {}
        for i, (k, v, lbl) in enumerate([("kp", 3.0, "Kp  %/°C"), ("ki", 0.05, "Ki  %/°C·s"),
                                         ("kff", 20.0, "Kff % per °C/s"), ("lead", 6.0, "Lead  s"),
                                         ("max_temp", 260, "Max temp °C"), ("cool_to", 80, "Cool to °C")]):
            ttk.Label(f, text=lbl).grid(row=4 + i, column=0, sticky="w")
            var = tk.StringVar(value=str(v))
            ttk.Entry(f, textvariable=var, width=7).grid(row=4 + i, column=1, sticky="e")
            self.gain_vars[k] = var
        self.run_btn = ttk.Button(f, text="▶  Start profile", command=self.start_run, state="disabled")
        self.run_btn.grid(row=10, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        # manual
        f = tab_man
        self.man_heat = tk.DoubleVar(value=0)
        self.man_o35 = tk.DoubleVar(value=0)
        self.man_o36 = tk.IntVar(value=0)
        ttk.Label(f, text="Heater %").grid(row=0, column=0, sticky="w")
        self.man_heat_lbl = ttk.Label(f, text="0", width=4)
        self.man_heat_lbl.grid(row=0, column=2)
        ttk.Scale(f, from_=0, to=100, variable=self.man_heat, command=lambda v: self.manual_changed()).grid(row=0, column=1, sticky="ew")
        ttk.Label(f, text="Exhaust fan %").grid(row=1, column=0, sticky="w")
        self.man_o35_lbl = ttk.Label(f, text="0", width=4)
        self.man_o35_lbl.grid(row=1, column=2)
        ttk.Scale(f, from_=0, to=100, variable=self.man_o35, command=lambda v: self.manual_changed()).grid(row=1, column=1, sticky="ew")
        ttk.Checkbutton(f, text="Convection fan", variable=self.man_o36, command=self.manual_changed).grid(row=2, column=0, columnspan=3, sticky="w")
        f.columnconfigure(1, weight=1)
        self.man_btn = ttk.Button(f, text="Enable manual", command=self.start_manual, state="disabled")
        self.man_btn.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 0))

        # calibration
        f = tab_cal
        self.cal = Calibration()
        self.cal_apply = tk.BooleanVar(value=False)
        ttk.Button(f, text="Load calibration…", command=self.load_cal).grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Checkbutton(f, text="Apply to profile runs", variable=self.cal_apply,
                        command=self.push_cal).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        self.cal_lbl = ttk.Label(f, text="points: none", wraplength=210, foreground="#666")
        self.cal_lbl.grid(row=2, column=0, columnspan=2, sticky="w", pady=(2, 8))
        ttk.Separator(f).grid(row=3, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Label(f, text="1. Hold oven at °C").grid(row=4, column=0, sticky="w")
        self.hold_var = tk.StringVar(value="150")
        ttk.Entry(f, textvariable=self.hold_var, width=7).grid(row=4, column=1, sticky="e")
        self.hold_btn = ttk.Button(f, text="Hold", command=self.start_hold, state="disabled")
        self.hold_btn.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(2, 4))
        self.steady_lbl = ttk.Label(f, text="2. wait for: —", foreground="#666")
        self.steady_lbl.grid(row=6, column=0, columnspan=2, sticky="w")
        ttk.Label(f, text="3. Board TC reads °C").grid(row=7, column=0, sticky="w", pady=(4, 0))
        self.board_var = tk.StringVar()
        ttk.Entry(f, textvariable=self.board_var, width=7).grid(row=7, column=1, sticky="e", pady=(4, 0))
        ttk.Button(f, text="Add point", command=self.add_cal_point).grid(row=8, column=0, columnspan=2, sticky="ew", pady=(2, 0))
        bf = ttk.Frame(f)
        bf.grid(row=9, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Button(bf, text="Save…", command=self.save_cal).pack(side="left", expand=True, fill="x")
        ttk.Button(bf, text="Clear", command=self.clear_cal).pack(side="left", expand=True, fill="x", padx=(4, 0))

        # stop
        self.stop_btn = tk.Button(side, text="■  STOP / HEATER OFF", bg="#c0392b", fg="white",
                                  activebackground="#922b21", activeforeground="white",
                                  font=("Segoe UI", 11, "bold"), relief="flat", command=self.stop_all, state="disabled")
        self.stop_btn.pack(fill="x", ipady=6, pady=(0, 8))

        # recording
        f = ttk.LabelFrame(side, text="CSV recording", padding=8)
        f.pack(fill="x")
        self.rec_btn = ttk.Button(f, text="● Start recording…", command=self.toggle_record)
        self.rec_btn.pack(fill="x")
        self.rec_lbl = ttk.Label(f, text="not recording", foreground="#666", wraplength=200)
        self.rec_lbl.pack(fill="x", pady=(4, 0))
        ttk.Button(f, text="Clear chart", command=self.clear_chart).pack(fill="x", pady=(6, 0))

        # readouts
        top = ttk.Frame(main)
        top.pack(fill="x")
        self.readouts = {}
        for key, title in [("temp", "Oven sensor °C"), ("board", "Board est. °C"), ("sp", "Setpoint °C"), ("heater", "Heater %"),
                           ("o35", "Exhaust fan %"), ("o36", "Convection fan"), ("phase", "Phase"), ("time", "Run time")]:
            box = ttk.Frame(top, padding=(10, 0))
            box.pack(side="left")
            ttk.Label(box, text=title, foreground="#666").pack(anchor="w")
            lbl = ttk.Label(box, text="—", font=("Segoe UI", 20 if key in ("temp", "board") else 14, "bold"))
            lbl.pack(anchor="w")
            self.readouts[key] = lbl

        self.chart = Chart(main)
        self.chart.pack(fill="both", expand=True, pady=(8, 0))
        self.status = ttk.Label(main, text="Not connected", foreground="#666")
        self.status.pack(fill="x", pady=(4, 0))

    # ------------------------------------------------------------------ actions
    def refresh_ports(self):
        ports = []
        if list_ports:
            ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connect(self):
        if self.worker:
            self.worker.stop_flag.set()
            self.conn_btn.config(state="disabled")
            return
        sim = self.sim_var.get()
        if not sim and not self.port_var.get():
            messagebox.showerror("No port", "Choose a COM port, or tick Simulator.")
            return
        try:
            speed = max(1.0, float(self.speed_var.get()))
        except ValueError:
            speed = 1.0
        self.worker = Worker(self.port_var.get(), sim, speed, self.q)
        self.worker.start()
        self.conn_btn.config(text="Connecting…", state="disabled")
        self.status.config(text="Connecting… (close the vendor app if it has the port open)")

    def load_profile(self):
        path = filedialog.askopenfilename(filetypes=[("Profile JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            prof = json.load(open(path))
            assert len(prof) >= 2 and all(len(p) == 2 for p in prof)
            assert all(prof[i][0] <= prof[i + 1][0] for i in range(len(prof) - 1))
        except Exception as e:
            messagebox.showerror("Bad profile", f"Expected a JSON list of [seconds, °C] pairs in time order.\n\n{e}")
            return
        self.profile = prof
        self.profile_name = path.replace("\\", "/").split("/")[-1]
        self.prof_label.config(text=self.profile_name)
        self.update_preview()

    def _num(self, key):
        return float(self.gain_vars[key].get())

    def start_run(self):
        if not self.worker:
            return
        try:
            gains = {k: self._num(k) for k in ("kp", "ki", "kff", "lead")}
            max_temp, cool_to = self._num("max_temp"), self._num("cool_to")
            o35 = int(float(self.run_o35.get()))
        except ValueError:
            messagebox.showerror("Bad value", "Controller settings must be numbers.")
            return
        if max(p[1] for p in self.profile) >= max_temp:
            messagebox.showerror("Profile too hot", "Profile peak is at or above Max temp.")
            return
        if self.chart.samples:
            now = self.chart.samples[-1]
            now_t = now["board"] if (now["board"] is not None and self.cal_apply.get()) else now["temp"]
            if now_t > self.profile[0][1] + 15 and not messagebox.askyesno(
                    "Oven is hot", f"The oven is at {now_t:.0f} °C but the profile starts at {self.profile[0][1]} °C. "
                    "Let it cool first for a proper profile. Start anyway?"):
                return
        if not self.rec_writer and messagebox.askyesno("Record?", "You are not recording. Start a CSV recording for this run?"):
            self.toggle_record()
        self.chart.run_offset = self.chart.samples[-1]["t"] if self.chart.samples else 0
        self.update_preview()
        self.worker.set(mode="profile", profile=self.profile, gains=gains, max_temp=max_temp,
                        cool_to=cool_to, run_o35=o35, run_o36=int(self.run_o36.get()), start_run=True)
        self.status.config(text=f"Running profile '{self.profile_name}'")

    def update_preview(self):
        """Profile overlay only when 'Preview profile' is ticked AND the Profile tab is open."""
        on = bool(self.preview_var.get()) and self.nb.index("current") == 0
        self.chart.profile = self.profile if on else None
        self.chart.redraw()

    def clear_chart(self):
        """Wipe the chart and restart the time axis at 0 s."""
        last_t = self.chart.samples[-1]["t"] if self.chart.samples else 0.0
        if hasattr(self, "hold_t0"):
            self.hold_t0 -= last_t          # keep the hold timer counting
        self.chart.clear()
        if self.worker:
            self.worker.set(reset_t0=True)

    # ------------------------------------------------------------------ calibration
    def push_cal(self):
        n = len(self.cal.points)
        txt = "points: " + self.cal.describe()
        if n == 1:
            txt += "  (offset only)"
        self.cal_lbl.config(text=txt)
        if self.worker:
            self.worker.set(cal=self.cal if self.cal else None, cal_apply=bool(self.cal_apply.get()))

    def load_cal(self):
        path = filedialog.askopenfilename(filetypes=[("Calibration JSON", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            self.cal = Calibration.load(path)
        except Exception as e:
            messagebox.showerror("Bad calibration file", f'Expected {{"points": [[oven_C, board_C], ...]}}\n\n{e}')
            return
        self.cal_apply.set(True)
        self.push_cal()

    def save_cal(self):
        if not self.cal:
            messagebox.showinfo("Calibration", "No points to save yet.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json", initialfile="calibration.json",
                                            filetypes=[("Calibration JSON", "*.json")])
        if path:
            self.cal.save(path)
            self.status.config(text=f"Calibration saved ({len(self.cal.points)} points)")

    def clear_cal(self):
        self.cal = Calibration()
        self.cal_apply.set(False)
        self.push_cal()

    def _recent_raw(self, seconds):
        s = self.chart.samples
        if not s:
            return []
        t_end = s[-1]["t"]
        return [d["temp"] for d in s if d["t"] >= t_end - seconds]

    def add_cal_point(self):
        try:
            board = float(self.board_var.get())
        except ValueError:
            messagebox.showerror("Calibration", "Type the board thermocouple reading first.")
            return
        recent = self._recent_raw(30)
        if not recent:
            messagebox.showerror("Calibration", "No oven readings yet - connect first.")
            return
        oven = sum(recent) / len(recent)
        self.cal = Calibration(self.cal.points + [(oven, board)])
        self.board_var.set("")
        self.push_cal()
        self.status.config(text=f"Added calibration point: oven {oven:.1f} °C → board {board:.1f} °C")

    def start_hold(self):
        if not self.worker:
            return
        try:
            sp = float(self.hold_var.get())
            gains = {k: self._num(k) for k in ("kp", "ki", "kff", "lead")}
            max_temp = self._num("max_temp")
            o35 = int(float(self.run_o35.get()))
        except ValueError:
            messagebox.showerror("Bad value", "Hold temperature and controller settings must be numbers.")
            return
        if sp >= max_temp:
            messagebox.showerror("Too hot", "Hold temperature is at or above Max temp.")
            return
        self.chart.run_offset = None
        self.hold_t0 = self.chart.samples[-1]["t"] if self.chart.samples else 0
        self.worker.set(mode="hold", hold_sp=sp, gains=gains, max_temp=max_temp,
                        run_o35=o35, run_o36=int(self.run_o36.get()), start_run=True)
        self.status.config(text=f"Holding oven sensor at {sp:.0f} °C - wait for 'steady', then read the board thermocouple")

    def start_manual(self):
        if not self.worker:
            return
        if not messagebox.askokcancel("Manual control", "The heater will follow the slider. "
                                      "Over-temperature cut-off still applies. Continue?"):
            return
        self.chart.run_offset = None
        self.worker.set(mode="manual", max_temp=self._num("max_temp"))
        self.manual_changed()
        self.status.config(text="Manual control")

    def manual_changed(self):
        h, o = round(self.man_heat.get()), round(self.man_o35.get())
        self.man_heat_lbl.config(text=str(h))
        self.man_o35_lbl.config(text=str(o))
        if self.worker:
            self.worker.set(heater=h, o35=o, o36=int(self.man_o36.get()))

    def stop_all(self):
        if self.worker:
            self.worker.set(mode="monitor")
        self.man_heat.set(0)
        self.manual_changed()
        self.status.config(text="Stopped - heater off, monitoring temperature")

    def toggle_record(self):
        if self.rec_writer:
            self.rec_file.close()
            self.rec_writer = self.rec_file = None
            self.rec_btn.config(text="● Start recording…")
            self.rec_lbl.config(text=f"saved {self.rec_count} rows", foreground="#666")
            return
        default = dt.datetime.now().strftime("reflow_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=default,
                                            filetypes=[("CSV", "*.csv")])
        if not path:
            return
        self.rec_file = open(path, "w", newline="")
        self.rec_writer = csv.writer(self.rec_file)
        self.rec_writer.writerow(CSV_FIELDS)
        self.rec_count = 0
        self.rec_btn.config(text="■ Stop recording")
        self.rec_lbl.config(text=f"recording → {path.replace(chr(92), '/').split('/')[-1]}", foreground="#c0392b")

    # ------------------------------------------------------------------ worker events
    def poll(self):
        redraw = False
        try:
            while True:
                kind, d = self.q.get_nowait()
                if kind == "sample":
                    self.on_sample(d)
                    redraw = True
                elif kind == "connected":
                    self.conn_btn.config(text="Disconnect", state="normal")
                    for b in (self.run_btn, self.man_btn, self.stop_btn, self.hold_btn):
                        b.config(state="normal")
                    self.push_cal()
                    self.status.config(text="Connected - monitoring (heater off)")
                elif kind == "disconnected":
                    self.worker = None
                    self.conn_btn.config(text="Connect", state="normal")
                    for b in (self.run_btn, self.man_btn, self.stop_btn, self.hold_btn):
                        b.config(state="disabled")
                    self.status.config(text="Not connected")
                elif kind == "run_done":
                    self.status.config(text="Profile complete - cooled down. Monitoring.")
                elif kind == "warn":
                    self.status.config(text=f"Warning: {d['msg']}")
                elif kind == "error":
                    self.status.config(text=d["msg"])
                    messagebox.showerror("Oven", d["msg"])
        except queue.Empty:
            pass
        if redraw:
            self.chart.redraw()
        self.after(100, self.poll)

    def on_sample(self, d):
        if self.chart.samples and d["t"] < self.chart.samples[-1]["t"]:
            self.chart.samples.clear()      # time base was reset; drop samples from before
        self.chart.samples.append(d)
        if d["run_t"] is not None:          # keep the profile overlay aligned with the run
            self.chart.run_offset = d["t"] - d["run_t"]
        r = self.readouts
        r["temp"].config(text=f"{d['temp']}")
        r["board"].config(text="—" if d["board"] is None else f"{d['board']:.0f}")
        if d["mode"] == "hold":
            held = d["t"] - getattr(self, "hold_t0", d["t"])
            win = self._recent_raw(60)
            span = max(win) - min(win)
            avg = sum(self._recent_raw(30)) / len(self._recent_raw(30))
            if held >= 120 and span <= 1:
                self.steady_lbl.config(text=f"2. STEADY ✓  oven avg {avg:.1f} °C", foreground="#1e8449")
            else:
                self.steady_lbl.config(text=f"2. settling… {held:.0f}s, ±{span / 2:.1f} °C", foreground="#b9770e")
        else:
            self.steady_lbl.config(text="2. wait for: —", foreground="#666")
        r["sp"].config(text="—" if d["sp"] is None else f"{d['sp']:.0f}")
        r["heater"].config(text=f"{d['heater']}")
        r["o35"].config(text=f"{d['o35']}")
        r["o36"].config(text=f"{d['o36']}")
        r["phase"].config(text=d["phase"] if d["mode"] != "monitor" else "monitor")
        r["time"].config(text="—" if d["run_t"] is None else f"{d['run_t']:.0f} s")
        if self.rec_writer:
            self.rec_writer.writerow([
                d["wall"], f"{d['t']:.2f}", d["mode"], d["phase"],
                "" if d["sp"] is None else f"{d['sp']:.1f}", d["temp"],
                "" if d["board"] is None else f"{d['board']:.1f}",
                "" if d["pred"] is None else f"{d['pred']:.1f}",
                d["heater"], d["o35"], d["o36"]])
            self.rec_count += 1
            if self.rec_count % 10 == 0:
                self.rec_file.flush()

    def on_close(self):
        if self.worker:
            self.worker.set(mode="monitor")
            self.worker.stop_flag.set()
            self.worker.join(timeout=3)
        if self.rec_file:
            self.rec_file.close()
        self.destroy()


if __name__ == "__main__":
    App(sim_default="--sim" in sys.argv).mainloop()
