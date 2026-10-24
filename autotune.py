#!/usr/bin/env python3
"""Automatic PID tuning for the BalancingRobotTuner firmware over USB serial (115200 baud).

The robot balances on the floor while this script tries parameter sets, measures each one
from the robot's own telemetry (IMU angle/gyro, encoder pulses, motor PWM) and keeps what
scores better. Stages, in the default order:

  angle0   balance-point trim: a robot standing still averages its true balance angle,
           so angle0 = -mean(angle). Needs no search, just one steady trial.
  angle    Kp, Kd of the angle loop (stiffness and damping against tilt)
  speed    Kp, Ki of the speed loop (stops the robot wandering and rocking)
  angle0   trim again with the new gains

Each trial: apply the parameters, settle, optionally nudge the robot (a short drive forward
then back, so it has to recover from a disturbance), record telemetry and compute a cost:

  cost = angle std + 0.1 x gyro RMS + 0.2 x wheel speed RMS + 0.005 x max wheel travel
         + 20 x fraction of time the PWM is saturated     (a fall: 100 + time lost)

Lower is better; the breakdown is printed for every trial. The search is coordinate descent
with multiplicative steps ("twiddle"); a candidate that beats the best is re-measured and
kept only if the average still wins, so one lucky trial does not stick.

Setup: robot on a flat floor with ~1 m of free space, USB cable slack (or tied up so it does
not pull), Arduino Serial Monitor closed. When the robot falls (the firmware cuts the motors
past 30 deg) the script reverts to the best parameters and waits for you to stand it up.
Ctrl+C at any time: the best parameters so far stay applied (not saved).

  python3 autotune.py                        # all stages, /dev/ttyUSB0
  python3 autotune.py --stages angle         # only the angle loop
  python3 autotune.py --no-nudge --trial 6   # quieter trials (can't move the robot)
  python3 autotune.py --save                 # write the result to EEPROM without asking
"""
import argparse
import csv
import math
import struct
import sys
import threading
import time

import serial

from serial_monitor import parse

# Parameter ids, limits and defaults: must match PARAM_* in BalancingRobotTuner.ino
P_KP, P_KD, P_KP_SPEED, P_KI_SPEED, P_KP_TURN, P_KD_TURN, P_ANGLE0, P_DRIVE_SPEED = range(8)
NAMES = ["kp", "kd", "kp_speed", "ki_speed", "kp_turn", "kd_turn", "angle0", "drive_speed"]
P_MIN = [0, 0, 0, 0, 0, 0, -15, 0]
P_MAX = [100, 10, 50, 2, 50, 5, 15, 100]

CMD_GET_PARAMS, CMD_SAVE, CMD_DEFAULTS, CMD_MOTORS, CMD_TELEM_RATE, CMD_LOAD = 1, 2, 3, 4, 5, 6
FALL_DEG = 30                       # firmware cuts the motors past this raw angle
PWM_SAT = 250                       # |pwm| >= this counts as saturated (firmware clamps at 255)

STAGE_PARAMS = {"angle": [P_KP, P_KD], "speed": [P_KP_SPEED, P_KI_SPEED]}


class Robot:
    """Binary-protocol client; a reader thread keeps the latest telemetry and replies."""

    def __init__(self, port, baud):
        for attempt in range(5):                    # this CH340 often fails the first open (EIO)
            try:
                self.s = serial.Serial(port, baud, timeout=0.05)
                break
            except serial.SerialException:
                if attempt == 4:
                    raise
                time.sleep(1)
        self.lock = threading.Lock()
        self.telem = []                     # (host time, frame dict), trimmed to the last 60 s
        self.params = {}
        self.acks = {}
        self.running = True
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        buf = bytearray()
        while self.running:
            try:
                data = self.s.read(256)
            except serial.SerialException:
                break
            if data:
                buf = parse(buf + data, self._on_frame)

    def _on_frame(self, kind, v):
        with self.lock:
            if kind == "telem":
                self.telem.append((time.monotonic(), v))
                if len(self.telem) > 1500:
                    del self.telem[:500]
            elif kind == "param":
                self.params[v["id"]] = v["value"]
            elif kind == "ack":
                self.acks[v["cmd"]] = v["ok"]

    def close(self):
        self.running = False
        self.thread.join(1)
        self.s.close()

    def _send(self, header, body):
        self.s.write(bytes([header]) + body + bytes([sum(body) & 0xFF, 0x5A]))

    def command(self, cmd, arg=0, timeout=1.0):
        with self.lock:
            self.acks.pop(cmd, None)
        self._send(0xA7, bytes([cmd, arg]))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if cmd in self.acks:
                    return self.acks[cmd]
            time.sleep(0.01)
        raise TimeoutError(f"no ack for command {cmd}")

    def set_param(self, pid, value):
        """Send and wait for the echo; returns the value the firmware applied (clamped)."""
        for _ in range(3):
            with self.lock:
                self.params.pop(pid, None)
            self._send(0xA6, bytes([pid]) + struct.pack("<f", value))
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                with self.lock:
                    if pid in self.params:
                        return self.params[pid]
                time.sleep(0.01)
        raise TimeoutError(f"no echo for parameter {NAMES[pid]}")

    def get_params(self):
        with self.lock:
            self.params.clear()
        self.command(CMD_GET_PARAMS)
        time.sleep(0.1)
        with self.lock:
            if len(self.params) < len(NAMES):
                raise TimeoutError("parameter dump incomplete")
            return [self.params[i] for i in range(len(NAMES))]

    def drive(self, y):
        """Kit control frame in mode 1: y > 30 forward, < -30 back, 0 stop."""
        self._send(0xA5, bytes([0, y & 0xFF, 0, 0, 0, 1, 0, 0]))

    def since(self, t0):
        with self.lock:
            return [v for t, v in self.telem if t >= t0]


def fallen(v):
    return bool(v["flags"] & 2) or abs(v["angle"]) > FALL_DEG


def wait_upright(robot, quiet=False):
    """Block until the robot has balanced (within 15 deg, not fallen) for 1.5 s. Raw angle,
    not tilt, so a badly wrong angle0 cannot make this wait forever."""
    if not quiet:
        print("  >> stand the robot up (it balances by itself once upright)", flush=True)
    while True:
        time.sleep(0.25)
        frames = robot.since(time.monotonic() - 1.5)
        if len(frames) >= 25 and all(abs(v["angle"]) < 15 and not fallen(v) for v in frames):
            return


def measure(robot, trial_s, nudge):
    """Run one trial with params already applied; returns (cost, metrics)."""
    time.sleep(1.5)                                     # settle after the parameter change
    t0 = time.monotonic()
    if nudge:
        time.sleep(1.0)
        for y, dur in ((nudge, 0.4), (-nudge, 0.4)):
            end = time.monotonic() + dur
            while time.monotonic() < end:
                robot.drive(y)
                time.sleep(0.05)
        for _ in range(3):
            robot.drive(0)
            time.sleep(0.05)
    while time.monotonic() - t0 < trial_s:
        time.sleep(0.1)
        if any(fallen(v) for v in robot.since(t0)):
            break
    frames = robot.since(t0)
    if not frames:
        raise RuntimeError("no telemetry received")

    fall_at = next((i for i, v in enumerate(frames) if fallen(v)), None)
    m = {"fell": fall_at is not None, "n": len(frames)}
    ok = frames[:fall_at] if fall_at is not None else frames
    if len(ok) < 5:
        ok = frames[:5]
    ang = [v["angle"] for v in ok]
    mean_a = sum(ang) / len(ang)
    m["angle_mean"] = mean_a
    m["angle_std"] = math.sqrt(sum((a - mean_a) ** 2 for a in ang) / len(ang))
    m["gyro_rms"] = math.sqrt(sum(v["gyro_x"] ** 2 for v in ok) / len(ok))
    vel = [(v["pulses_l"] + v["pulses_r"]) / 2 for v in ok]
    m["speed_rms"] = math.sqrt(sum(x * x for x in vel) / len(vel))
    pos, travel = 0.0, 0.0
    for x in vel:
        pos += x
        travel = max(travel, abs(pos))
    m["travel"] = travel
    m["sat"] = sum(max(abs(v["pwm_l"]), abs(v["pwm_r"])) >= PWM_SAT for v in ok) / len(ok)

    if m["fell"]:
        survived = fall_at / max(len(frames), 1) * trial_s
        cost = 100 + 10 * (trial_s - survived)
    else:
        cost = (m["angle_std"] + 0.1 * m["gyro_rms"] + 0.2 * m["speed_rms"]
                + 0.005 * m["travel"] + 20 * m["sat"])
    return cost, m


class Tuner:
    def __init__(self, robot, args, log):
        self.robot = robot
        self.args = args
        self.log = log
        self.n = 0
        self.params = robot.get_params()
        self.start = list(self.params)
        self.best = list(self.params)               # what Ctrl+C falls back to

    def apply(self, params):
        for i, (old, new) in enumerate(zip(self.params, params)):
            if abs(old - new) > 1e-6:
                self.params[i] = self.robot.set_param(i, new)

    def trial(self, stage, params, nudge=None):
        nudge = self.args.nudge if nudge is None else nudge
        wait_upright(self.robot, quiet=True)        # with the last (good) parameters, then switch
        self.apply(params)
        cost, m = measure(self.robot, self.args.trial, nudge)
        self.n += 1
        tuned = "  ".join(f"{NAMES[i]}={self.params[i]:.4g}" for i in (P_KP, P_KD, P_KP_SPEED, P_KI_SPEED, P_ANGLE0))
        detail = ("FELL" if m["fell"] else
                  f"ang_std={m['angle_std']:.2f} gyro={m['gyro_rms']:.1f} spd={m['speed_rms']:.1f} "
                  f"travel={m['travel']:.0f} sat={m['sat']:.0%}")
        print(f"  #{self.n:<3d} {tuned}  ->  cost {cost:7.2f}  ({detail})", flush=True)
        self.log.writerow([self.n, stage] + [f"{p:.5g}" for p in self.params] + [f"{cost:.3f}"]
                          + [f"{m[k]:.4g}" if isinstance(m[k], float) else m[k]
                             for k in ("fell", "angle_mean", "angle_std", "gyro_rms", "speed_rms", "travel", "sat")])
        if m["fell"]:
            print("  !! fell", flush=True)
        return cost, m

    def recover(self, best):
        """After a fall: back to the best parameters, then wait for the robot to be stood up."""
        self.apply(best)
        wait_upright(self.robot)

    def trim_angle0(self):
        print("\n== angle0: balance-point trim (robot left alone, no nudge)")
        for _ in range(2):
            p = list(self.params)
            cost, m = self.trial("angle0", p, nudge=0)
            if m["fell"]:
                self.recover(p)
                continue
            new = max(P_MIN[P_ANGLE0], min(P_MAX[P_ANGLE0], -m["angle_mean"]))
            if m["travel"] > 400:
                print(f"  (robot travelled {m['travel']:.0f} pulses: estimate is rough)")
            old = p[P_ANGLE0]
            print(f"  angle0 {old:.2f} -> {new:.2f}")
            p[P_ANGLE0] = new
            self.apply(p)
            self.best = list(self.params)
            if abs(new - old) < 0.3:
                break

    def twiddle(self, stage):
        ids = STAGE_PARAMS[stage]
        print(f"\n== {stage}: tuning {', '.join(NAMES[i] for i in ids)}")
        best = list(self.params)
        lo = {i: max(P_MIN[i], best[i] / 3) for i in ids}
        hi = {i: min(P_MAX[i], best[i] * 3) for i in ids}
        costs = [self.trial(stage, best)[0] for _ in range(2)]
        best_cost = sum(costs) / 2
        step = {i: 0.25 for i in ids}
        budget = self.args.max_trials
        start_n = self.n

        while self.n - start_n < budget and max(step.values()) > 0.04:
            for i in ids:
                improved = False
                for sign in (+1, -1):
                    cand = list(best)
                    cand[i] = min(hi[i], max(lo[i], best[i] * (1 + sign * step[i])))
                    if abs(cand[i] - best[i]) < 1e-9:
                        continue
                    c, m = self.trial(stage, cand)
                    if m["fell"]:
                        self.recover(best)
                    elif c < best_cost:
                        c2, m2 = self.trial(stage, cand)        # confirm: noise must not win
                        if m2["fell"]:
                            self.recover(best)
                        elif (c + c2) / 2 < best_cost:
                            best, best_cost, improved = cand, (c + c2) / 2, True
                            self.best = list(best)
                            print(f"  ++ {NAMES[i]} = {best[i]:.4g}  (best cost {best_cost:.2f})", flush=True)
                            break
                step[i] *= 1.3 if improved else 0.6
                if self.n - start_n >= budget:
                    break
        self.apply(best)
        print(f"== {stage} done: " + ", ".join(f"{NAMES[i]}={best[i]:.4g}" for i in ids)
              + f"  (cost {best_cost:.2f})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--stages", default="angle0,angle,speed,angle0",
                    help="comma list of angle0, angle, speed (default: %(default)s)")
    ap.add_argument("--trial", type=float, default=8.0, help="seconds measured per trial (default 8)")
    ap.add_argument("--max-trials", type=int, default=30, help="trial budget per search stage (default 30)")
    ap.add_argument("--nudge", type=int, default=90,
                    help="stick value (31..127) for the forward/back disturbance; 0 = none (default 90)")
    ap.add_argument("--no-nudge", dest="nudge", action="store_const", const=0)
    ap.add_argument("--log", default="autotune_log.csv", help="CSV of every trial (default %(default)s)")
    save = ap.add_mutually_exclusive_group()
    save.add_argument("--save", action="store_true", help="save the result to EEPROM without asking")
    save.add_argument("--no-save", action="store_true", help="never save to EEPROM")
    args = ap.parse_args()
    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    for s in stages:
        if s not in ("angle0", "angle", "speed"):
            ap.error(f"unknown stage {s!r}")

    robot = Robot(args.port, args.baud)
    print(f"{args.port} @ {args.baud}: waiting for the Nano to boot...", flush=True)
    time.sleep(2.5)                                     # opening the port resets the Nano
    robot.s.write(b"b")                                 # leave text debug mode
    time.sleep(0.2)
    robot.command(CMD_TELEM_RATE, 25)
    robot.command(CMD_MOTORS, 1)

    logf = open(args.log, "w", newline="")
    log = csv.writer(logf)
    log.writerow(["trial", "stage"] + NAMES + ["cost", "fell", "angle_mean", "angle_std",
                                               "gyro_rms", "speed_rms", "travel", "sat"])
    tuner = Tuner(robot, args, log)
    print("start: " + "  ".join(f"{n}={v:.4g}" for n, v in zip(NAMES, tuner.start)))
    print(f"stages: {' -> '.join(stages)}; {args.trial:g} s per trial; nudge {args.nudge or 'off'}")

    try:
        wait_upright(robot)
        for s in stages:
            if s == "angle0":
                tuner.trim_angle0()
            else:
                tuner.twiddle(s)
    except KeyboardInterrupt:
        print("\nstopped: keeping the best parameters found so far")
        tuner.apply(tuner.best)
    finally:
        robot.drive(0)
        logf.close()

    print("\nresult:")
    for i, n in enumerate(NAMES):
        mark = "" if abs(tuner.params[i] - tuner.start[i]) < 1e-6 else f"   (was {tuner.start[i]:.4g})"
        print(f"  {n:12s} {tuner.params[i]:.4g}{mark}")
    print(f"trial log: {args.log}")

    if args.save:
        ok = True
    elif args.no_save:
        ok = False
    else:
        try:
            ok = input("save to EEPROM? [y/N] ").strip().lower() == "y"
        except (EOFError, KeyboardInterrupt):
            ok = False
    if ok:
        robot.command(CMD_SAVE)
        print("saved (survives power-off)")
    else:
        print("not saved: lost at the next power-off or reset")
    robot.s.write(b"d")                                 # back to readable lines for the Serial Monitor
    robot.close()


if __name__ == "__main__":
    main()
