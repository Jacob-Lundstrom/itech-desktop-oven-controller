#!/usr/bin/env python3
"""
Replacement controller for Huaqi Zhengbang / iTECH RF-A series reflow ovens.

  python reflow.py temp     --port COM3              # link and print temperature once a second
  python reflow.py identify --port COM3              # step through the fan outputs so you can see/hear which is which
  python reflow.py step     --port COM3 --power 40 --seconds 240   # open-loop step test for tuning (logs CSV)
  python reflow.py run      --port COM3 --profile sac305.json      # run a profile with the new controller
  python reflow.py run      --sim                    # same, against the simulated oven
  python reflow.py vendor   --sim                    # replica of the vendor app's algorithm, for A/B comparison

Every mode turns the heater off on exit, Ctrl-C, or any error.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import deque

from calibration import Calibration
from oven import OUT_35, OUT_36, Oven, OvenError

# Generic SAC305 ramp-soak-spike profile (time s, oven-sensor temp C).
# Replace with your paste's datasheet profile.
DEFAULT_PROFILE = [
    [0, 30], [90, 150], [180, 180], [225, 245], [235, 245], [240, 240],
]


# --------------------------------------------------------------------------- utils
class RealClock:
    now = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


def interp(profile, t):
    if t <= profile[0][0]:
        return profile[0][1]
    for (t0, y0), (t1, y1) in zip(profile, profile[1:]):
        if t <= t1:
            return y0 + (y1 - y0) * (t - t0) / (t1 - t0) if t1 > t0 else y1
    return profile[-1][1]


class Slope:
    """Least-squares dT/dt over a sliding window (sensor is quantised to 1 C)."""

    def __init__(self, window_s=4.0):
        self.w = window_s
        self.buf = deque()

    def add(self, t, y):
        self.buf.append((t, y))
        while self.buf and t - self.buf[0][0] > self.w:
            self.buf.popleft()

    def value(self):
        n = len(self.buf)
        if n < 3:
            return 0.0
        mt = sum(p[0] for p in self.buf) / n
        my = sum(p[1] for p in self.buf) / n
        den = sum((p[0] - mt) ** 2 for p in self.buf)
        return 0.0 if den == 0 else sum((p[0] - mt) * (p[1] - my) for p in self.buf) / den


def open_oven(args, verbose=False):
    log = (lambda s: print("   ", s)) if verbose else None
    if args.sim:
        from sim import FakeOvenSerial, VirtualClock
        clock = VirtualClock()
        ov = Oven(FakeOvenSerial(clock), log=log)
        ov.sim = ov.ser
        return ov, clock
    if not args.port:
        sys.exit("--port is required (or use --sim)")
    return Oven(args.port, log=log), RealClock()


class CsvLog:
    def __init__(self, path, fields):
        self.f = open(path, "w", newline="") if path else None
        self.w = csv.writer(self.f) if self.f else None
        if self.w:
            self.w.writerow(fields)

    def row(self, *vals):
        if self.w:
            self.w.writerow([f"{v:.2f}" if isinstance(v, float) else v for v in vals])
            self.f.flush()

    def close(self):
        if self.f:
            self.f.close()


# --------------------------------------------------------------------------- controllers
class LeadPI:
    """
    PI on a *predicted* temperature plus setpoint-slope feed-forward.

      T_pred = T + lead * dT/dt          (compensates heater/sensor lag; acts like
                                          derivative-on-measurement, no setpoint kick)
      u = Kff * dSP/dt + Kp*(SP - T_pred) + I
      I += Ki*(SP - T)*dt                 only while u is not saturated in the same direction

    Units: Kp [%/C], Ki [%/(C*s)], Kff [% per C/s], lead [s].
    """

    def __init__(self, kp=3.0, ki=0.05, kff=20.0, lead=6.0, bias=0.0):
        self.kp, self.ki, self.kff, self.lead = kp, ki, kff, lead
        self.i = bias
        self.slope = Slope()

    def step(self, t, dt, sp, sp_slope, temp):
        self.slope.add(t, temp)
        tpred = temp + self.lead * self.slope.value()
        u = self.kff * sp_slope + self.kp * (sp - tpred) + self.i
        e = sp - temp
        if not ((u >= 100 and e > 0) or (u <= 0 and e < 0)):
            self.i = max(-50.0, min(100.0, self.i + self.ki * e * dt))
            u = self.kff * sp_slope + self.kp * (sp - tpred) + self.i
        return max(0.0, min(100.0, u)), tpred


class VendorPID:
    """Exact replica of the vendor app (Main.exe @0x407260), gains from config.ini."""

    def __init__(self, kp=35.0, ki=0.5, kd=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.prev = 0.0

    def step(self, temp, sp):
        e = sp - temp
        p = self.kp * e
        if p > 100 or p < -100:
            self.i = 0.0
        else:
            self.i = max(-100.0, min(100.0, self.i + self.ki * e))
        d = self.kd * (e - self.prev)
        self.prev = e
        return int(max(0, min(100, p + self.i + d)))  # caller truncates & clamps 0..100


# --------------------------------------------------------------------------- modes
def mode_temp(ov, clock, args):
    ov.link()
    print("Linked. Ctrl-C to stop.")
    while True:
        print(f"{ov.read_temp()} C")
        clock.sleep(1.0)


def mode_identify(ov, clock, args):
    ov.link()
    ov.heater(0)
    steps = [
        ("all outputs off", []),
        ("fan enable = 1, speed 0", [(OUT_36, 1)]),
        ("fan enable = 0, speed 50", [(OUT_36, 0), (OUT_35, 50)]),
        ("fan enable = 0, speed 100", [(OUT_35, 100)]),
        ("fan enable = 1, speed 100", [(OUT_36, 1)]),
    ]
    for name, cmds in steps:
        if not cmds:
            ov.set_output(OUT_35, 0)
            ov.set_output(OUT_36, 0)
        for c, v in cmds:
            ov.set_output(c, v)
        input(f"[{name}]  Heater is OFF. Note which fan(s) run and how fast, then press Enter...")
    print("Done - turning everything off.")


def mode_step(ov, clock, args):
    ov.link()
    log = CsvLog(args.csv or "step.csv", ["t_s", "temp_C", "heater_pct"])
    ov.set_output(OUT_36, args.o36)
    ov.set_output(OUT_35, args.o35)
    t0 = clock.now()
    while (t := clock.now() - t0) < args.seconds:
        T = ov.read_temp()
        if T >= args.max_temp:
            print(f"max temp {args.max_temp} reached, stopping")
            break
        ov.heater(args.power)
        log.row(t, T, args.power)
        print(f"{t:6.1f}s  {T:4d} C  heater {args.power}%", end="\r")
        clock.sleep(args.period)
    ov.heater(0)
    print("\nlogging cool-down for 120 s...")
    t1 = clock.now()
    while clock.now() - t1 < 120:
        log.row(clock.now() - t0, ov.read_temp(), 0)
        clock.sleep(args.period)
    log.close()


def run_profile(ov, clock, args, controller):
    profile = json.load(open(args.profile)) if args.profile else DEFAULT_PROFILE
    t_end = profile[-1][0]
    cal = Calibration.load(args.cal) if args.cal else Calibration()
    if cal:
        print(f"Calibration: {cal.describe()}  (profile is in board temperature)")
    ov.link()
    log = CsvLog(args.csv, ["t_s", "setpoint_C", "temp_oven_C", "temp_board_est_C", "heater_pct", "fan_speed_pct", "fan_enable", "phase", "pred_C"])
    ov.set_output(OUT_36, args.o36)
    ov.set_output(OUT_35, args.o35)
    t0 = last = clock.now()
    peak = 0
    errors = 0
    while True:
        t = clock.now() - t0
        dt, last = clock.now() - last, clock.now()
        try:
            T_raw = ov.read_temp()
            T = cal.board(T_raw) if cal else T_raw
            errors = 0
        except OvenError as e:
            errors += 1
            print("\n", e)
            if errors >= 3:
                raise
            ov.heater(0)
            clock.sleep(args.period)
            continue
        peak = max(peak, T)
        if max(T, T_raw) >= args.max_temp:
            raise OvenError(f"Over-temperature: {T} C >= {args.max_temp} C")
        if t <= t_end:
            sp = interp(profile, t)
            sp_slope = (interp(profile, t + 1.0) - interp(profile, t - 1.0)) / 2.0
            u, pred = controller(t, max(dt, 1e-3), sp, sp_slope, T)
            ov.heater(u)
            phase = "profile"
        else:
            sp, u, pred = args.cool_to, 0.0, T
            ov.heater(0)
            ov.set_output(OUT_36, 1)
            ov.set_output(OUT_35, 100)
            phase = "cool"
            if T <= args.cool_to:
                break
        log.row(t, float(sp), T_raw, float(T), float(u), 100 if phase == "cool" else args.o35, 1 if phase == "cool" else args.o36, phase, float(pred))
        if not args.quiet:
            print(f"{t:6.1f}s  SP {sp:5.1f}  T {T:5.1f}  heat {u:5.1f}%  [{phase}]   ", end="\r")
        clock.sleep(args.period)
    log.close()
    print(f"\nDone. Peak controlled temperature {peak:.0f} C (profile peak {max(p[1] for p in profile)} C).")
    return peak


def mode_run(ov, clock, args):
    c = LeadPI(args.kp, args.ki, args.kff, args.lead)
    return run_profile(ov, clock, args, lambda t, dt, sp, s, T: c.step(t, dt, sp, s, T))


def mode_vendor(ov, clock, args):
    """Vendor stage logic: 4 stages of (target, hold s), step setpoints,
    0.5 s loop, convection fan on; cooling fan speed 0 while heating, 50 when T > SP+5, 100 to cool."""
    stages = [(100, 20), (150, 30), (240, 30), (240, 30)]  # approx. from your config.ini
    cool_to = args.cool_to
    pid = VendorPID()
    ov.link()
    log = CsvLog(args.csv, ["t_s", "setpoint_C", "temp_C", "heater_pct", "fan_speed_pct", "phase"])
    ov.heater(0); ov.set_output(OUT_35, 0); ov.set_output(OUT_36, 1)
    t0 = clock.now(); stage = 0; hold_start = None; peak = 0
    while True:
        t = clock.now() - t0
        T = ov.read_temp(); peak = max(peak, T)
        if stage < 4:
            sp, hold = stages[stage]
            reached = T >= sp if stage < 3 else T <= sp   # vendor quirk on the last stage
            if reached and hold_start is None:
                hold_start = t
            if hold_start is not None and t - hold_start >= hold:
                stage += 1; hold_start = None
                continue
            u = pid.step(T, sp)
            ov.heater(u)
            o35 = 50 if T - sp > 5 else 0
            ov.set_output(OUT_35, o35)
            log.row(t, sp, T, u, o35, f"stage{stage}")
        else:
            ov.heater(0); ov.set_output(OUT_35, 100)
            log.row(t, cool_to, T, 0, 100, "cool")
            if T <= cool_to:
                break
        clock.sleep(0.5)
    log.close()
    print(f"Vendor algorithm: peak {peak} C vs. target 240 C")
    return peak


# --------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["temp", "identify", "step", "run", "vendor"])
    ap.add_argument("--port")
    ap.add_argument("--sim", action="store_true", help="use the simulated oven")
    ap.add_argument("--verbose", action="store_true", help="print every TX/RX frame")
    ap.add_argument("--csv", help="log file")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--period", type=float, default=0.5, help="loop period s (vendor: 0.5)")
    ap.add_argument("--max-temp", type=int, default=260, help="hard cut-off C")
    ap.add_argument("--cool-to", type=int, default=80)
    ap.add_argument("--fan-enable", "--o36", dest="o36", type=int, default=1, help="convection fan enable during run (0/1)")
    ap.add_argument("--fan-speed", "--o35", dest="o35", type=int, default=100, help="cooling fan speed %% during run (vendor app: 0)")
    ap.add_argument("--profile", help="JSON list of [t_s, temp_C] points")
    ap.add_argument("--cal", help='calibration JSON {"points": [[oven_C, board_C], ...]}')
    ap.add_argument("--kp", type=float, default=3.0)
    ap.add_argument("--ki", type=float, default=0.05)
    ap.add_argument("--kff", type=float, default=20.0)
    ap.add_argument("--lead", type=float, default=6.0)
    ap.add_argument("--power", type=float, default=40.0, help="step mode heater %%")
    ap.add_argument("--seconds", type=float, default=240.0, help="step mode duration")
    args = ap.parse_args(argv)

    ov, clock = open_oven(args, args.verbose)
    try:
        return {"temp": mode_temp, "identify": mode_identify, "step": mode_step,
                "run": mode_run, "vendor": mode_vendor}[args.mode](ov, clock, args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        ov.safe_off()
        if ov.linked:
            try:
                ov.unlink()
            except Exception:
                pass
        ov.close()
        print("Heater off.")


if __name__ == "__main__":
    main()
