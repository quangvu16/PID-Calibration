#!/usr/bin/env python3
"""Live PID tuning with graphs for the BalancingRobotTuner firmware over USB serial (115200).

The PC counterpart of the app's Tune PID screen: scrolling charts of everything the firmware
reports, each control-loop term on its own trace so you can see which one does the work
(the idea of farrellf's Balancing_Robot_Firmware, which streams set point, error and the
P/I/D components to a telemetry plotter).

  tilt / angle    tilt = angle + angle0, the error the angle loop drives to zero (deg)
  gyro            angular rate around the wheel axle (deg/s), what Kd multiplies
  loop terms      angle term (Kp, Kd), speed term (Kp_speed, Ki_speed), turn term
  PWM             motor commands left/right; +-255 is saturation
  wheel speed     encoder pulses per telemetry frame, left/right

Left panel: parameter editors (Enter or Set applies at once, -/+ step 10 %), Get/Save
(EEPROM)/Load/Defaults, motors on/off, telemetry rate, a forward-back nudge to test recovery,
statistics of the visible window, "angle0 from mean" trim, and CSV recording.

Auto-tune: tunes the ticked gains (and angle0, if ticked: searched within +-3 deg) while the
robot balances on the floor (ml_tune.py), with the method picked in the list (METHODS below):
  ML (Bayesian)   a Gaussian process learns the cost from every trial and picks the most
                  promising next one (fewest trials, explores the whole box)
  Neural          the same with a deep ensemble of small neural networks as the model
  RL              policy gradient (PGPE) over the gains, one trial per episode
  CMA-ES, DE, PSO, Genetic   evolutionary / swarm search, populations of trials
  Nelder-Mead, SPSA          local direct search / noisy gradient estimate
  Twiddle         autotune.py's coordinate descent: +-step on one gain at a time, a win is
                  re-measured before it is kept (simple, stays close to the current gains)
  Random          Latin-hypercube baseline
A second window shows every trial's cost and a Gaussian-process fit of the cost over the
gains (for all but ML only a picture of the trials, it does not steer the search).

  python3 tuner_gui.py                 # /dev/ttyUSB0
  python3 tuner_gui.py /dev/ttyACM0 --window 20
"""
import argparse
import csv
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
import numpy as np
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.ticker import FuncFormatter, LogFormatter  # noqa: E402

from autotune import (CMD_DEFAULTS, CMD_GET_PARAMS, CMD_LOAD, CMD_MOTORS, CMD_SAVE,  # noqa: E402
                      CMD_TELEM_RATE, NAMES, P_ANGLE0, P_MAX, P_MIN, PWM_SAT, Robot)
import ml_tune  # noqa: E402
from ml_tune import TUNABLE, from_y, read_history  # noqa: E402
from serial_monitor import TELEM_FIELDS  # noqa: E402

LABELS = ["Kp (angle)", "Kd (angle)", "Kp speed", "Ki speed", "Kp turn", "Kd turn",
          "angle0 (deg)", "drive speed"]

# (subplot title, unit, [(series key, label, colour)])
PLOTS = [
    ("Tilt / angle", "deg", [("tilt", "tilt", "C0"), ("angle", "angle (raw)", "C7")]),
    ("Gyro", "deg/s", [("gyro_x", "gyro", "C1")]),
    ("Loop terms", "", [("angle_term", "angle", "C2"), ("speed_term", "speed", "C3"),
                        ("turn_term", "turn", "C4")]),
    ("PWM", "", [("pwm_l", "left", "C5"), ("pwm_r", "right", "C9")]),
    ("Wheel speed", "pulses/frame", [("pulses_l", "left", "C5"), ("pulses_r", "right", "C9")]),
]

HERE = os.path.dirname(os.path.abspath(__file__))
ML_LOG = os.path.join(HERE, "ml_tune_log.csv")      # both methods' trials (autotune.py format)
# method: (tuner class, default Range x, hint). The searches cover current / range .. x range;
# twiddle steps inside autotune.py's /3..x3.
METHODS = {
    "ML (Bayesian)": (ml_tune.BayesTuner, 2.0,
                      "Gaussian process + expected improvement. Fewest trials; the default."),
    "Neural (deep ensemble)": (ml_tune.NeuralTuner, 2.0,
                               "5 small neural nets learn the cost; tries the lowest confidence bound."),
    "RL (policy gradient)": (ml_tune.PolicyGradientTuner, 2.0,
                             "PGPE: a Gaussian policy over the gains, 2 trials per update."),
    "CMA-ES": (ml_tune.CmaesTuner, 2.0,
               "Evolution strategy that learns the step size and gain correlations."),
    "Differential evolution": (ml_tune.DifferentialEvolutionTuner, 2.0,
                               "Population search, DE/rand/1/bin. Needs 30+ trials."),
    "Particle swarm (PSO)": (ml_tune.PsoTuner, 2.0, "Swarm search. Needs 30+ trials."),
    "Genetic algorithm": (ml_tune.GeneticTuner, 2.0,
                          "Selection, crossover, mutation, elitism. Needs 30+ trials."),
    "Nelder-Mead": (ml_tune.NelderMeadTuner, 2.0,
                    "Downhill simplex: local, no model, sensitive to noise."),
    "SPSA": (ml_tune.SpsaTuner, 2.0,
             "Noisy gradient from 2 trials per step (extremum-seeking style). Local."),
    "Twiddle": (ml_tune.TwiddleTuner, 3.0,
                "One gain at a time, wins re-measured. Simple, stays close to the start."),
    "Random (baseline)": (ml_tune.RandomTuner, 2.0, "Latin-hypercube samples: what the others must beat."),
}
HISTORY_LOGS = [ML_LOG, os.path.join(HERE, "autotune_log.csv")]


class ModelWindow:
    """Trial costs and the GP's predicted cost over the tuned gains."""

    def __init__(self, root):
        self.top = tk.Toplevel(root)
        self.top.title("Auto-tune model")
        self.top.protocol("WM_DELETE_WINDOW", self.close)
        self.fig = Figure(figsize=(10, 4), layout="constrained")
        self.ax_cost, self.ax_map = self.fig.subplots(1, 2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.top)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def close(self):
        self.top.destroy()
        self.top = None

    def update(self, d):
        ax = self.ax_cost
        ax.clear()
        costs = from_y(d["Y"][[i for i, o in enumerate(d["own"]) if o]])
        fell = [f for f, o in zip(d["fell_mask"], d["own"]) if o]
        n = range(1, len(costs) + 1)
        ax.plot(n, costs, color="C7", lw=0.8)
        for k, c, f in zip(n, costs, fell):
            ax.plot(k, c, "x" if f else "o", color="C3" if f else "C0", ms=5)
        ok = [c if not f else math.inf for c, f in zip(costs, fell)]
        run_min = [min(ok[:k]) for k in n]
        ax.step(n, run_min, where="post", color="C2", lw=1.2, label="best trial so far")
        if d["best_pred"] is not None:
            ax.axhline(d["best_pred"], color="C2", ls="--", lw=0.8,
                       label="best (mean of 2)" if d["method"] == "twiddle" else "model's best")
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.3g}"))
        ax.yaxis.set_minor_formatter(LogFormatter(minor_thresholds=(1, 0.4)))  # only when narrow
        ax.set_xlabel("trial", fontsize=8)
        ax.set_ylabel("cost (x = fell)", fontsize=8)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)
        self.draw_map(d)
        self.canvas.draw_idle()

    def draw_map(self, d):
        ax, gp, space, X = self.ax_map, d["gp"], d["space"], d["X"]
        ax.clear()
        if gp is None:
            return
        ax.tick_params(labelsize=7)
        base = d["best_x"] if d["best_x"] is not None else X[-1]
        u = np.linspace(0, 1, 60)
        colours = ["C3" if f else ("C0" if o else "C7") for f, o in zip(d["fell_mask"], d["own"])]
        if len(space.ids) == 1:
            mean, std = gp.predict(u[:, None])
            px = space.value(0, u)
            ax.plot(px, from_y(mean), color="C0")
            ax.fill_between(px, from_y(mean - 2 * std), from_y(mean + 2 * std), color="C0", alpha=0.2)
            ax.scatter(space.value(0, X[:, 0]), from_y(d["Y"]), c=colours, s=14, zorder=3)
            ax.set_yscale("log")
            ax.set_xlabel(space.names[0], fontsize=8)
            ax.set_ylabel("predicted cost (band = 2 std)", fontsize=8)
        else:
            # first two tuned values; any others held at the current best
            gx, gy = np.meshgrid(u, u)
            pts = np.tile(base, (gx.size, 1))
            pts[:, 0], pts[:, 1] = gx.ravel(), gy.ravel()
            mean = from_y(gp.predict(pts)[0]).reshape(gx.shape)
            ax.contourf(space.value(0, gx), space.value(1, gy), np.log10(np.clip(mean, 1, None)),
                        levels=20, cmap="viridis")
            ax.scatter(space.value(0, X[:, 0]), space.value(1, X[:, 1]), c=colours, s=14,
                       edgecolors="w", linewidths=0.5)
            if d["best_x"] is not None:
                ax.plot(space.value(0, base[0]), space.value(1, base[1]), "*", color="w", ms=12,
                        markeredgecolor="k")
            ax.set_xlabel(space.names[0], fontsize=8)
            ax.set_ylabel(space.names[1], fontsize=8)
            extra = ", others at best" if len(space.ids) > 2 else ""
            ax.set_title(f"predicted cost (dark = better{extra}); red = fell, grey = past runs",
                         fontsize=8)
        axes = []                               # gains on log axes, angle0 linear
        if space.log[0]:
            ax.set_xscale("log")
            axes.append(ax.xaxis)
        if len(space.ids) > 1 and space.log[1]:
            ax.set_yscale("log")
            axes.append(ax.yaxis)
        plain = FuncFormatter(lambda v, _: f"{v:.3g}")
        for a in axes:
            a.set_major_formatter(plain)
            a.set_minor_formatter(plain)


class TunerGui:
    def __init__(self, root, robot, window_s):
        self.root = root
        self.robot = robot
        self.window = tk.DoubleVar(value=window_s)
        self.paused = False
        self.rec = None                     # (file, writer, host time of the last row written)
        self.params = [None] * len(NAMES)
        self.ml_thread = None
        self.ml_stop = threading.Event()
        self.ml_queue = queue.Queue()
        self.model_win = None
        self.ml_last = None                 # latest "trial" report, for the model window
        root.title("Balance Tuner")
        root.protocol("WM_DELETE_WINDOW", self.close)

        side = ttk.Frame(root, padding=6)
        side.pack(side=tk.LEFT, fill=tk.Y)
        self._build_params(side)
        self._build_controls(side)
        self._build_autotune(side)
        self.stats = tk.StringVar(value="")
        ttk.Label(side, textvariable=self.stats, font=("TkFixedFont", 9), justify=tk.LEFT
                  ).pack(anchor=tk.W, pady=(8, 0))
        self.status = tk.StringVar(value="connecting...")
        ttk.Label(side, textvariable=self.status, foreground="#a33", wraplength=260
                  ).pack(side=tk.BOTTOM, anchor=tk.W)

        self.fig = Figure(figsize=(9, 8), layout="constrained")
        self.axes = self.fig.subplots(len(PLOTS), 1, sharex=True)
        self.lines = {}
        for ax, (title, unit, series) in zip(self.axes, PLOTS):
            ax.set_ylabel(f"{title}\n{unit}" if unit else title, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
            for key, label, colour in series:
                self.lines[key], = ax.plot([], [], color=colour, lw=1.1, label=label)
            ax.legend(loc="upper left", fontsize=7, ncol=len(series))
        self.axes[0].axhline(0, color="k", lw=0.6)
        for y in (PWM_SAT, -PWM_SAT):
            self.axes[3].axhline(y, color="r", lw=0.6, ls="--")
        self.axes[-1].set_xlabel("s", fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        root.after(2500, self.start)        # opening the port resets the Nano

    # ---------------------------------------------------------------- widgets
    def _build_params(self, parent):
        box = ttk.LabelFrame(parent, text="Parameters", padding=4)
        box.pack(fill=tk.X)
        self.entries = []
        for i, label in enumerate(LABELS):
            ttk.Label(box, text=label).grid(row=i, column=0, sticky=tk.W)
            var = tk.StringVar()
            e = ttk.Entry(box, textvariable=var, width=9)
            e.grid(row=i, column=1, padx=2)
            e.bind("<Return>", lambda _e, i=i: self.set_param(i))
            ttk.Button(box, text="-", width=2, command=lambda i=i: self.step(i, -1)).grid(row=i, column=2)
            ttk.Button(box, text="+", width=2, command=lambda i=i: self.step(i, +1)).grid(row=i, column=3)
            ttk.Button(box, text="Set", width=4, command=lambda i=i: self.set_param(i)).grid(row=i, column=4)
            self.entries.append(var)
        row = ttk.Frame(box)
        row.grid(row=len(LABELS), column=0, columnspan=5, pady=(4, 0))
        for text, cmd in (("Get", lambda: self.command(CMD_GET_PARAMS, then_get=True)),
                          ("Save", self.save), ("Load", lambda: self.command(CMD_LOAD, then_get=True)),
                          ("Defaults", lambda: self.command(CMD_DEFAULTS, then_get=True))):
            ttk.Button(row, text=text, width=8, command=cmd).pack(side=tk.LEFT, padx=1)

    def _build_controls(self, parent):
        box = ttk.LabelFrame(parent, text="Robot", padding=4)
        box.pack(fill=tk.X, pady=(6, 0))
        self.motors = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="Motors on", variable=self.motors,
                        command=lambda: self.command(CMD_MOTORS, int(self.motors.get()))
                        ).grid(row=0, column=0, sticky=tk.W)
        ttk.Button(box, text="Nudge", command=self.nudge).grid(row=0, column=1, columnspan=2)
        ttk.Label(box, text="Telemetry Hz").grid(row=1, column=0, sticky=tk.W)
        self.rate = tk.IntVar(value=25)
        sp = ttk.Spinbox(box, from_=1, to=25, width=4, textvariable=self.rate,
                         command=lambda: self.command(CMD_TELEM_RATE, self.rate.get()))
        sp.bind("<Return>", lambda _e: self.command(CMD_TELEM_RATE, self.rate.get()))
        sp.grid(row=1, column=1, sticky=tk.W)
        ttk.Label(box, text="Window s").grid(row=2, column=0, sticky=tk.W)
        ttk.Spinbox(box, from_=2, to=40, width=4, textvariable=self.window).grid(row=2, column=1, sticky=tk.W)
        self.pause_btn = ttk.Button(box, text="Pause", command=self.toggle_pause)
        self.pause_btn.grid(row=3, column=0, sticky=tk.W, pady=(4, 0))
        self.rec_btn = ttk.Button(box, text="Record CSV", command=self.toggle_record)
        self.rec_btn.grid(row=3, column=1, columnspan=2, pady=(4, 0))
        ttk.Button(box, text="angle0 from mean", command=self.trim_angle0
                   ).grid(row=4, column=0, columnspan=3, sticky=tk.W, pady=(4, 0))

    def _build_autotune(self, parent):
        box = ttk.LabelFrame(parent, text="Auto-tune", padding=4)
        box.pack(fill=tk.X, pady=(6, 0))
        self.ml_method = tk.StringVar(value=next(iter(METHODS)))
        methods = ttk.Combobox(box, textvariable=self.ml_method, values=list(METHODS),
                               state="readonly", width=24)
        methods.grid(row=0, column=0, columnspan=4, sticky=tk.W)

        def picked(_event=None):
            self.ml_range.set(METHODS[self.ml_method.get()][1])
            self.ml_info.set(METHODS[self.ml_method.get()][2])
        methods.bind("<<ComboboxSelected>>", picked)
        self.ml_use = []
        for n, i in enumerate(TUNABLE):
            var = tk.BooleanVar(value=i in TUNABLE[:2])
            ttk.Checkbutton(box, text=LABELS[i], variable=var).grid(row=1 + n // 2, column=n % 2 * 2,
                                                                    columnspan=2, sticky=tk.W)
            self.ml_use.append((i, var))
        self.ml_trim = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="trim angle0 first", variable=self.ml_trim
                        ).grid(row=4, column=0, columnspan=4, sticky=tk.W)
        self.ml_history = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="learn from past trials (logs)", variable=self.ml_history
                        ).grid(row=5, column=0, columnspan=4, sticky=tk.W)
        self.ml_trials = tk.IntVar(value=20)
        self.ml_trial_s = tk.DoubleVar(value=8.0)
        self.ml_nudge = tk.IntVar(value=90)
        self.ml_range = tk.DoubleVar(value=2.0)
        for n, (text, var, lo, hi, inc) in enumerate((
                ("Trials", self.ml_trials, 5, 80, 1), ("Trial s", self.ml_trial_s, 4, 20, 1),
                ("Nudge", self.ml_nudge, 0, 127, 10), ("Range x", self.ml_range, 1.2, 5, 0.5))):
            r, c = 6 + n // 2, n % 2 * 2
            ttk.Label(box, text=text).grid(row=r, column=c, sticky=tk.W)
            ttk.Spinbox(box, from_=lo, to=hi, increment=inc, width=5, textvariable=var
                        ).grid(row=r, column=c + 1, sticky=tk.W)
        self.ml_btn = ttk.Button(box, text="Start auto-tune", command=self.toggle_autotune)
        self.ml_btn.grid(row=8, column=0, columnspan=2, sticky=tk.W, pady=(4, 0))
        ttk.Button(box, text="Model", command=self.show_model).grid(row=8, column=2, columnspan=2,
                                                                     pady=(4, 0))
        self.ml_info = tk.StringVar(value=METHODS[self.ml_method.get()][2])
        ttk.Label(box, textvariable=self.ml_info, wraplength=260, justify=tk.LEFT
                  ).grid(row=9, column=0, columnspan=4, sticky=tk.W)

    # ---------------------------------------------------------------- robot I/O
    def start(self):
        try:
            self.robot.s.write(b"b")        # leave text debug mode
            time.sleep(0.2)
            self.robot.command(CMD_TELEM_RATE, self.rate.get())
            self.refresh_params()
            self.status.set("")
        except (TimeoutError, OSError) as e:
            self.status.set(f"no reply from the robot ({e}); check the port, then press Get")
        self.root.after(100, self.redraw)

    def run(self, what, fn):
        try:
            return fn()
        except (TimeoutError, OSError) as e:
            self.status.set(f"{what}: {e}")
            return None

    def refresh_params(self):
        params = self.robot.get_params()
        for i, v in enumerate(params):
            self.params[i] = v
            self.entries[i].set(f"{v:.4g}")

    def command(self, cmd, arg=0, then_get=False):
        if self.run(f"command {cmd}", lambda: self.robot.command(cmd, arg)) and then_get:
            self.run("get parameters", self.refresh_params)

    def set_param(self, i, value=None):
        if value is None:
            try:
                value = float(self.entries[i].get())
            except ValueError:
                self.status.set(f"{LABELS[i]}: not a number")
                return
        value = max(P_MIN[i], min(P_MAX[i], value))
        applied = self.run(f"set {NAMES[i]}", lambda: self.robot.set_param(i, value))
        if applied is not None:
            self.params[i] = applied
            self.entries[i].set(f"{applied:.4g}")
            self.status.set("")

    def step(self, i, sign):
        try:
            v = float(self.entries[i].get())
        except ValueError:
            return
        if i == P_ANGLE0:
            v += 0.1 * sign             # an offset, not a gain: fixed step
        else:
            v = v * (1 + 0.1 * sign) if v else 0.1 * max(sign, 0)
        self.set_param(i, round(v, 4))

    def save(self):
        if messagebox.askyesno("Save", "Write the current parameters to EEPROM?"):
            self.command(CMD_SAVE)

    def nudge(self, y=90):
        """Forward then back, as autotune does, so you can watch the recovery."""
        def go():
            for sy, dur in ((y, 0.4), (-y, 0.4)):
                end = time.monotonic() + dur
                while time.monotonic() < end:
                    self.robot.drive(sy)
                    time.sleep(0.05)
            for _ in range(3):
                self.robot.drive(0)
                time.sleep(0.05)
        threading.Thread(target=go, daemon=True).start()

    def trim_angle0(self):
        frames = self.visible()
        if len(frames) < 20:
            return
        mean = sum(v["angle"] for _, v in frames) / len(frames)
        if messagebox.askyesno("angle0", f"Mean raw angle over the window is {mean:.2f} deg.\n"
                               f"Set angle0 to {-mean:.2f}? (robot should be balancing in place)"):
            self.set_param(P_ANGLE0, -mean)

    # ---------------------------------------------------------------- auto-tune (ML or twiddle)
    def toggle_autotune(self):
        if self.ml_thread and self.ml_thread.is_alive():
            self.ml_stop.set()
            self.ml_info.set("stopping after this trial...")
            return
        ids = [i for i, var in self.ml_use if var.get()]
        if not ids:
            self.ml_info.set("tick at least one gain to tune")
            return
        try:
            trials, trial_s = int(self.ml_trials.get()), float(self.ml_trial_s.get())
            nudge, factor = int(self.ml_nudge.get()), float(self.ml_range.get())
        except (tk.TclError, ValueError):
            self.ml_info.set("trials / trial s / nudge / range: not a number")
            return
        if not messagebox.askyesno(
                "Auto-tune", "The robot will try parameter sets on its own for about "
                f"{trials * (trial_s + 3) / 60:.0f} min.\n\nRobot on a flat floor with ~1 m free, "
                "USB cable slack. Stand it up again whenever it falls.\n\nStart?"):
            return
        self.motors.set(True)
        self.command(CMD_MOTORS, 1)
        self.rate.set(25)                       # the cost's wheel speed is per telemetry frame
        self.command(CMD_TELEM_RATE, 25)
        self.ml_stop = threading.Event()
        tuner_class = METHODS[self.ml_method.get()][0]
        tuner = tuner_class(self.robot, ids, trials, trial_s, max(0, min(127, nudge)), max(1.1, factor),
                            self.ml_trim.get(),
                            read_history(HISTORY_LOGS) if self.ml_history.get() else [],
                            ML_LOG, self.ml_stop, lambda kind, data: self.ml_queue.put((kind, data)))
        self.ml_thread = threading.Thread(target=tuner.run, daemon=True)
        self.ml_thread.start()
        self.ml_btn.config(text="Stop auto-tune")
        self.ml_info.set(f"starting ({self.ml_method.get()})")

    def show_model(self):
        if self.model_win is None or self.model_win.top is None:
            self.model_win = ModelWindow(self.root)
            if self.ml_last:
                self.model_win.update(self.ml_last)
        else:
            self.model_win.top.lift()

    def poll_autotune(self):
        while True:
            try:
                kind, d = self.ml_queue.get_nowait()
            except queue.Empty:
                return
            if kind == "status":
                self.ml_info.set(d)
            elif kind == "params":
                for i, v in enumerate(d):
                    self.params[i] = v
                    self.entries[i].set(f"{v:.4g}")
            elif kind == "trial":
                self.ml_last = d
                tried = " ".join(f"{k}={v:.3g}" for k, v in d["tried"].items())
                best = " ".join(f"{k}={v:.3g}" for k, v in d["best"].items())
                pred = "" if d["best_pred"] is None else f" (predicted cost {d['best_pred']:.1f})"
                result = "FELL" if d["fell"] else f"cost {d['cost']:.1f}"
                self.ml_info.set(f"trial {d['n']}/{d['total']}: {tried} -> {result}\n"
                                 f"best: {best}{pred}")
                self.show_model()
                self.model_win.update(d)
            elif kind == "done":
                self.ml_btn.config(text="Start auto-tune")
                changed = [f"{n}: {a:.4g} -> {b:.4g}" for n, a, b in zip(NAMES, d["start"] or [], d["result"] or [])
                           if abs(a - b) > 1e-6]
                self.ml_info.set(f"{d['reason']}. " + ("; ".join(changed) if changed else "no change"))
                if changed and messagebox.askyesno(
                        "Auto-tune " + d["reason"].split(":")[0],
                        "Applied:\n  " + "\n  ".join(changed) + "\n\nWrite them to EEPROM?"):
                    self.command(CMD_SAVE)

    # ---------------------------------------------------------------- recording
    def toggle_record(self):
        if self.rec:
            self.rec[0].close()
            self.rec = None
            self.rec_btn.config(text="Record CSV")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="tuner_log.csv")
        if not path:
            return
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["host_s"] + TELEM_FIELDS + NAMES)
        self.rec = [f, w, time.monotonic()]
        self.rec_btn.config(text="Stop recording")

    def write_rows(self):
        f, w, last = self.rec
        with self.robot.lock:
            new = [(t, v) for t, v in self.robot.telem if t > last]
        for t, v in new:
            w.writerow([f"{t:.3f}"] + [v[k] for k in TELEM_FIELDS]
                       + ["" if p is None else f"{p:.5g}" for p in self.params])
        if new:
            self.rec[2] = new[-1][0]

    # ---------------------------------------------------------------- plotting
    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    def span(self):
        try:
            return max(1.0, float(self.window.get()))
        except (tk.TclError, ValueError):
            return 10.0

    def visible(self):
        t0 = time.monotonic() - self.span()
        with self.robot.lock:
            return [(t, v) for t, v in self.robot.telem if t >= t0]

    def redraw(self):
        self.poll_autotune()
        if self.rec:
            self.write_rows()
        frames = self.visible()
        if not self.paused and frames:
            now = time.monotonic()
            xs = [t - now for t, _ in frames]
            a0 = self.params[P_ANGLE0] or 0.0
            for key, line in self.lines.items():
                if key == "tilt":
                    line.set_data(xs, [v["angle"] + a0 for _, v in frames])
                else:
                    line.set_data(xs, [v[key] for _, v in frames])
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view(scalex=False)
            self.axes[0].set_xlim(-self.span(), 0)
            self.canvas.draw_idle()
            self.update_stats(frames, a0)
        elif not frames:
            self.status.set("no telemetry")
        self.root.after(100, self.redraw)

    def update_stats(self, frames, a0):
        n = len(frames)
        vs = [v for _, v in frames]
        tilt = [v["angle"] + a0 for v in vs]
        mean = sum(tilt) / n
        std = math.sqrt(sum((x - mean) ** 2 for x in tilt) / n)
        gyro = math.sqrt(sum(v["gyro_x"] ** 2 for v in vs) / n)
        sat = sum(max(abs(v["pwm_l"]), abs(v["pwm_r"])) >= PWM_SAT for v in vs) / n
        spd = math.sqrt(sum(((v["pulses_l"] + v["pulses_r"]) / 2) ** 2 for v in vs) / n)
        dt = frames[-1][0] - frames[0][0]
        last = vs[-1]
        state = ("FALLEN" if last["flags"] & 2 else "motors on" if last["flags"] & 1 else "motors off")
        self.stats.set(f"{state}   {n / dt if dt > 0 else 0:4.1f} Hz\n"
                       f"tilt mean {mean:6.2f}  std {std:5.2f}\n"
                       f"gyro rms  {gyro:6.1f}\n"
                       f"speed rms {spd:6.1f}\n"
                       f"PWM sat   {sat:6.0%}\n"
                       f"dist      {last['dist_cm']:4d} cm")

    def close(self):
        self.ml_stop.set()
        if self.rec:
            self.rec[0].close()
        try:
            self.robot.drive(0)
            self.robot.s.write(b"d")        # back to readable lines for the Serial Monitor
        except OSError:
            pass
        self.robot.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="seconds shown (default 10)")
    args = ap.parse_args()
    robot = Robot(args.port, args.baud)
    root = tk.Tk()
    TunerGui(root, robot, args.window)
    root.mainloop()


if __name__ == "__main__":
    main()
