#!/usr/bin/env python3
"""Auto-tuning methods for tuner_gui.py: Bayesian optimisation with a Gaussian process (ML),
and autotune.py's twiddle search, both run from the GUI (TwiddleTuner, see its docstring).

autotune.py walks one parameter at a time (twiddle) and re-measures every candidate that wins.
Here a Gaussian process (GP) learns the whole cost surface cost(Kp, Kd, ...) from every trial
so far, falls and earlier runs' logs included, and the next parameters tried are the ones
with the highest expected improvement (EI) over the best: it trades off trying where the
model predicts a low cost against where it is still unsure. Expect ~15-25 trials for two
parameters, ~30-40 for four.

  search space  each chosen gain between current / range and current x range, on a log
                scale (gains act multiplicatively); angle0 (an offset) linearly within
                current +- 3 deg; normalised to the unit cube
  trial, cost   the same as autotune.py (measure()); the GP learns log(1 + cost), so a fall
                (cost >= 100) is "much worse" without flattening the differences between
                good trials
  GP            RBF kernel; length scale and noise picked by marginal likelihood on a small
                grid, so trial-to-trial noise is modelled instead of chased
  history       rows of ml_tune_log.csv / autotune_log.csv whose other gains match the
                current ones (within 10 %, angle0 within 1 deg) seed the model
  result        the tried parameter set with the lowest *predicted* cost, not the luckiest
                single trial

numpy only (no scikit-learn). BayesTuner/TwiddleTuner.run() are meant for a thread; it reports through
post(kind, data): "status" text, "params" list, "trial" dict, "error" text, "done" dict.
"""
import csv
import math
import time

import numpy as np

from autotune import (NAMES, P_ANGLE0, P_KD, P_KI_SPEED, P_KP, P_KP_SPEED, P_MAX, P_MIN,
                      fallen, measure)

TUNABLE = [P_KP, P_KD, P_KP_SPEED, P_KI_SPEED, P_ANGLE0]
GAINS = TUNABLE[:4]                 # multiplicative: log scale; angle0 is an offset in degrees
ANGLE0_SPAN = 3.0                   # angle0 is searched within current +- this (deg), linearly
LOG_FIELDS = ["trial", "stage"] + NAMES + ["cost", "fell", "angle_mean", "angle_std",
                                           "gyro_rms", "speed_rms", "travel", "sat"]
COST_CAP = 300                      # falls score 100 + 10 x seconds lost; cap before the log


class Stopped(Exception):
    pass


def to_y(cost):
    return math.log1p(min(cost, COST_CAP))


def from_y(y):
    return np.expm1(y)


def norm_cdf(z):
    return 0.5 * (1 + np.vectorize(math.erf)(z / math.sqrt(2)))


def norm_pdf(z):
    return np.exp(-0.5 * z * z) / math.sqrt(2 * math.pi)


def sqdist(a, b):
    return ((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)


class GaussianProcess:
    """Zero-mean GP on standardised targets, RBF kernel shared over the unit-cube inputs."""
    LENGTHS = (0.08, 0.15, 0.25, 0.4, 0.7, 1.2)
    NOISES = (0.01, 0.05, 0.15, 0.4)            # variance, in units of the target variance

    def __init__(self, X, y):
        self.X = np.asarray(X, float)
        y = np.asarray(y, float)
        self.mu = y.mean()
        self.sd = y.std() if len(y) > 1 and y.std() > 1e-9 else 1.0
        z = (y - self.mu) / self.sd
        d2 = sqdist(self.X, self.X)
        best = -math.inf
        for ls in self.LENGTHS:
            for noise in self.NOISES:
                K = np.exp(-0.5 * d2 / ls ** 2) + noise * np.eye(len(z))
                L = np.linalg.cholesky(K)
                a = np.linalg.solve(L.T, np.linalg.solve(L, z))
                lml = -0.5 * z @ a - np.log(np.diag(L)).sum()
                if lml > best:
                    best, self.ls, self.noise, self.L, self.a = lml, ls, noise, L, a

    def predict(self, Xs):
        """Posterior mean and std of the latent cost (in y units) at the rows of Xs."""
        Xs = np.atleast_2d(np.asarray(Xs, float))
        Ks = np.exp(-0.5 * sqdist(Xs, self.X) / self.ls ** 2)
        v = np.linalg.solve(self.L, Ks.T)
        var = np.clip(1 - (v ** 2).sum(0), 1e-12, None)
        return self.mu + self.sd * (Ks @ self.a), self.sd * np.sqrt(var)

    def expected_improvement(self, Xs, y_best, xi=0.01):
        mean, std = self.predict(Xs)
        imp = y_best - mean - xi
        z = imp / std
        return imp * norm_cdf(z) + std * norm_pdf(z)


class Space:
    """Box around the starting values, mapped to [0, 1]^d: gains on a log scale
    (current / factor .. x factor), angle0 linear (current +- ANGLE0_SPAN deg)."""

    def __init__(self, ids, centre, factor):
        self.ids = list(ids)
        self.names = [NAMES[i] for i in ids]
        self.log = [i != P_ANGLE0 for i in ids]
        self.lo, self.hi = [], []
        for i in ids:
            if i == P_ANGLE0:
                self.lo.append(max(P_MIN[i], centre[i] - ANGLE0_SPAN))
                self.hi.append(min(P_MAX[i], centre[i] + ANGLE0_SPAN))
                continue
            c = centre[i] if centre[i] > 0 else 0.05 * P_MAX[i]
            self.lo.append(math.log(max(c / factor, P_MIN[i], 1e-3 * P_MAX[i])))
            self.hi.append(math.log(min(c * factor, P_MAX[i])))

    def value(self, k, u):
        v = self.lo[k] + np.asarray(u) * (self.hi[k] - self.lo[k])
        return np.exp(v) if self.log[k] else v

    def to_params(self, x):
        return [float(self.value(k, u)) for k, u in enumerate(x)]

    def to_unit(self, values):
        """None when a value lies outside the box (or a gain is not positive)."""
        x = []
        for k, v in enumerate(values):
            if self.log[k] and v <= 0:
                return None
            u = ((math.log(v) if self.log[k] else v) - self.lo[k]) / (self.hi[k] - self.lo[k])
            if not -1e-6 <= u <= 1 + 1e-6:
                return None
            x.append(min(1.0, max(0.0, u)))
        return np.array(x)


def latin_hypercube(n, d, rng):
    return np.stack([(rng.permutation(n) + rng.random(n)) / n for _ in range(d)], axis=1)


def read_history(paths):
    """Trial rows (dicts of floats plus "fell") from autotune-format CSV logs that exist."""
    rows = []
    for path in paths:
        try:
            with open(path, newline="") as f:
                for r in csv.DictReader(f):
                    try:
                        row = {n: float(r[n]) for n in NAMES + ["cost"]}
                    except (KeyError, TypeError, ValueError):
                        continue
                    row["fell"] = r.get("fell") == "True"
                    rows.append(row)
        except OSError:
            pass
    return rows


class GuiTuner:
    """What both GUI auto-tune methods share: a stoppable trial loop with fall recovery,
    angle0 trim, the trial log, and a GP fitted over the trials for the model window.
    Subclasses implement search() and choose_best()."""
    method = ""

    def __init__(self, robot, ids, trials, trial_s, nudge, factor, trim, history, log_path,
                 stop, post, seed=None):
        self.robot = robot
        self.ids = list(ids)
        self.trials = trials
        self.trial_s = trial_s
        self.nudge = nudge
        self.factor = factor
        self.trim = trim
        self.history = history
        self.log_path = log_path
        self.stop = stop                        # threading.Event
        self.post = post
        self.rng = np.random.default_rng(seed)
        self.n = 0
        self.params = self.start = self.best = None
        self.best_x = self.best_pred = self.gp = None

    # ---------------------------------------------------------------- robot
    def check_stop(self):
        if self.stop.is_set():
            raise Stopped

    def wait_upright(self, prompt=False):
        """As autotune.wait_upright, but gives up when stop is set."""
        if prompt:
            self.post("status", "fell: stand the robot up (it balances by itself once upright)")
        while True:
            self.check_stop()
            time.sleep(0.25)
            frames = self.robot.since(time.monotonic() - 1.5)
            if len(frames) >= 25 and all(abs(v["angle"]) < 15 and not fallen(v) for v in frames):
                return

    def apply(self, params):
        for i, (old, new) in enumerate(zip(self.params, params)):
            if abs(old - new) > 1e-6:
                self.params[i] = self.robot.set_param(i, new)
        self.post("params", list(self.params))

    def evaluate(self, stage, params, nudge=None):
        """One trial on the robot: wait upright with the last parameters, switch, measure."""
        self.check_stop()
        self.wait_upright()
        self.apply(params)
        self.tried = list(self.params)          # before a fall puts the best values back
        cost, m = measure(self.robot, self.trial_s, self.nudge if nudge is None else nudge)
        self.n += 1
        self.log.writerow([self.n, stage] + [f"{p:.5g}" for p in self.params] + [f"{cost:.3f}"]
                          + [f"{m[k]:.4g}" if isinstance(m[k], float) else m[k]
                             for k in LOG_FIELDS[-7:]])
        self.logf.flush()
        if m["fell"]:
            self.apply(self.best)
            self.wait_upright(prompt=True)
            self.post("status", "running")
        return cost, m

    # ---------------------------------------------------------------- stages
    def trim_angle0(self):
        """Balance-point trim as in autotune.py: angle0 = -mean(angle) standing still."""
        for _ in range(2):
            self.post("status", "trimming angle0 (robot left alone)")
            p = list(self.params)
            _, m = self.evaluate("angle0", p, nudge=0)
            if m["fell"]:
                continue
            old = p[P_ANGLE0]
            p[P_ANGLE0] = max(P_MIN[P_ANGLE0], min(P_MAX[P_ANGLE0], -m["angle_mean"]))
            self.apply(p)
            self.best = list(self.params)
            self.post("status", f"angle0 {old:.2f} -> {self.params[P_ANGLE0]:.2f}")
            if abs(self.params[P_ANGLE0] - old) < 0.3:
                break

    def seed(self, space):
        """Past trials taken with the same other gains, as unit-cube points."""
        X, Y = [], []
        others = [i for i in GAINS if i not in self.ids]
        for r in self.history:
            if any(abs(r[NAMES[i]] - self.params[i]) > 0.1 * max(abs(self.params[i]), 1e-6)
                   for i in others):
                continue
            if P_ANGLE0 not in self.ids and abs(r["angle0"] - self.params[P_ANGLE0]) > 1.0:
                continue
            x = space.to_unit([r[NAMES[i]] for i in self.ids])
            if x is not None:
                X.append(x)
                Y.append(to_y(r["cost"]))
        return X, Y

    def begin(self, factor):
        """Search box around the current gains, seeded with matching past trials."""
        self.space = Space(self.ids, self.params, factor)
        self.stage = self.method + ":" + "+".join(self.space.names)
        self.X, self.Y = self.seed(self.space)
        self.own, self.fell = [False] * len(self.X), [False] * len(self.X)
        self.k = 0
        if self.X:
            self.post("status", f"learned from {len(self.X)} past trials")

    def trial(self, p, x=None):
        """evaluate() plus the model bookkeeping and the "trial" report."""
        cost, m = self.evaluate(self.stage, p)
        self.k += 1
        # the firmware may have clamped/rounded the values: learn what was really applied
        tried = self.space.to_unit([self.tried[i] for i in self.ids])
        if tried is None:
            tried = x
        if tried is not None:
            self.X.append(tried)
            self.Y.append(to_y(cost))
            self.own.append(True)
            self.fell.append(m["fell"])
        self.gp = GaussianProcess(self.X, self.Y) if self.X else None
        self.choose_best()
        self.post("trial", {
            "method": self.method, "n": self.k, "total": self.trials, "cost": cost,
            "fell": m["fell"], "tried": {NAMES[i]: self.tried[i] for i in self.ids},
            "best": {NAMES[i]: self.best[i] for i in self.ids}, "best_pred": self.best_pred,
            "gp": self.gp, "space": self.space, "X": np.array(self.X), "Y": np.array(self.Y),
            "own": list(self.own), "fell_mask": list(self.fell), "best_x": self.best_x})
        return cost, m

    def search(self):
        raise NotImplementedError

    def choose_best(self):
        raise NotImplementedError

    def run(self):
        reason = "finished"
        try:
            self.params = self.robot.get_params()
            self.start = list(self.params)
            self.best = list(self.params)
            with open(self.log_path, "a", newline="") as self.logf:
                self.log = csv.writer(self.logf)
                if self.logf.tell() == 0:
                    self.log.writerow(LOG_FIELDS)
                self.post("status", "waiting for the robot to balance")
                self.wait_upright()
                self.post("status", "running")
                if self.trim:
                    self.trim_angle0()
                self.search()
        except Stopped:
            reason = "stopped"
        except (TimeoutError, OSError, RuntimeError) as e:
            reason = f"error: {e}"
        finally:
            if self.best is not None:
                try:
                    self.apply(self.best)
                except (TimeoutError, OSError) as e:
                    reason += f" (could not apply the best parameters: {e})"
            self.post("done", {"reason": reason, "start": self.start, "result": self.params})


class BayesTuner(GuiTuner):
    """Bayesian optimisation: the next trial maximises the GP's expected improvement."""
    method = "ml"

    def search(self):
        self.begin(self.factor)
        d = len(self.ids)
        x0 = self.space.to_unit([self.params[i] for i in self.ids])
        n_rand = max(0, d + 2 - len(self.X))
        init = [x0] + list(latin_hypercube(n_rand, d, self.rng))
        while self.k < self.trials:
            if self.k < len(init):
                x = init[self.k]
            else:
                y_ref = float(self.gp.predict(np.array(self.X))[0].min())
                cand = self.rng.random((2000, d))
                if self.best_x is not None:
                    local = self.best_x + self.rng.normal(0, 0.07, (600, d))
                    cand = np.vstack([cand, np.clip(local, 0, 1)])
                x = cand[int(np.argmax(self.gp.expected_improvement(cand, y_ref)))]
            p = list(self.best)
            for i, v in zip(self.ids, self.space.to_params(x)):
                p[i] = v
            self.trial(p, x)

    def choose_best(self):
        """The tried set (this run, did not fall) with the lowest predicted cost."""
        mean = self.gp.predict(np.array(self.X))[0]
        good = [j for j in range(len(self.X)) if self.own[j] and not self.fell[j]]
        if good:
            j = min(good, key=lambda j: mean[j])
            self.best_x, self.best_pred = self.X[j], float(from_y(mean[j]))
            for i, v in zip(self.ids, self.space.to_params(self.best_x)):
                self.best[i] = v


class TwiddleTuner(GuiTuner):
    """autotune.py's coordinate descent ("twiddle") over the chosen gains: multiplicative
    steps (start 25 %, x1.3 after a win, x0.6 otherwise) inside current / range .. x range;
    angle0 steps additively (step x ANGLE0_SPAN deg, so +-0.75 deg first) within +-ANGLE0_SPAN;
    a candidate that beats the best is re-measured and kept only if the average still wins.
    The GP here only draws the model window; it does not steer the search."""
    method = "twiddle"

    def search(self):
        self.begin(self.factor)
        ids = self.ids
        best = list(self.best)
        lo = {i: max(P_MIN[i], best[i] / self.factor) for i in ids}
        hi = {i: min(P_MAX[i], best[i] * self.factor) for i in ids}
        if P_ANGLE0 in ids:                     # an offset: additive steps of step x span deg
            lo[P_ANGLE0] = max(P_MIN[P_ANGLE0], best[P_ANGLE0] - ANGLE0_SPAN)
            hi[P_ANGLE0] = min(P_MAX[P_ANGLE0], best[P_ANGLE0] + ANGLE0_SPAN)
        self.post("status", "measuring the starting gains (2 trials)")
        self.best_cost = None
        costs = [self.trial(best)[0] for _ in range(2)]
        self.best_cost = sum(costs) / 2
        step = {i: 0.25 for i in ids}

        while self.k < self.trials and max(step.values()) > 0.04:
            for i in ids:
                improved = False
                for sign in (+1, -1):
                    if self.k >= self.trials:
                        break
                    cand = list(best)
                    v = (best[i] + sign * step[i] * ANGLE0_SPAN if i == P_ANGLE0
                         else best[i] * (1 + sign * step[i]))
                    cand[i] = min(hi[i], max(lo[i], v))
                    if abs(cand[i] - best[i]) < 1e-9:
                        continue
                    c, m = self.trial(cand)
                    if m["fell"] or c >= self.best_cost or self.k >= self.trials:
                        continue
                    c2, m2 = self.trial(cand)       # confirm: noise must not win
                    if not m2["fell"] and (c + c2) / 2 < self.best_cost:
                        best, self.best_cost, improved = cand, (c + c2) / 2, True
                        self.best = list(best)
                        self.post("status", f"++ {NAMES[i]} = {best[i]:.4g} "
                                            f"(best cost {self.best_cost:.2f})")
                        break
                step[i] *= 1.3 if improved else 0.6
                if self.k >= self.trials:
                    break
        self.post("status", "steps below 4 %: converged" if self.k < self.trials else "trial budget used")

    def choose_best(self):
        self.best_x = self.space.to_unit([self.best[i] for i in self.ids])
        self.best_pred = self.best_cost
