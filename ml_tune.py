#!/usr/bin/env python3
"""Auto-tuning methods for tuner_gui.py: Bayesian optimisation with a Gaussian process (ML),
autotune.py's twiddle search (TwiddleTuner, see its docstring), and, at the end of this file,
a neural-network surrogate, policy-gradient RL, CMA-ES, differential evolution, PSO, a genetic
algorithm, Nelder-Mead, SPSA and a random baseline, all sharing GuiTuner's trial loop.

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

numpy only (no scikit-learn). Each tuner's run() is meant for a thread; it reports through
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


# ==================================================================== more search methods
# All of them search the same unit cube as BayesTuner, score a trial as to_y(cost) (a fall is
# simply a very high score) and report the result as BayesTuner does: the tried set with the
# lowest *predicted* cost of the GP that is fitted for the model window anyway, so a lucky
# trial never wins on its own. Each optimise() is written as an endless loop; f() raises
# Budget when the trials are used up.

class Budget(Exception):
    pass


class SearchTuner(GuiTuner):
    choose_best = BayesTuner.choose_best

    def search(self):
        self.begin(self.factor)
        self.d = len(self.ids)
        x0 = self.space.to_unit([self.params[i] for i in self.ids])
        self.x0 = np.full(self.d, 0.5) if x0 is None else x0
        try:
            self.optimise()
        except Budget:
            pass

    def f(self, x):
        """One trial at unit-cube point x; returns its score to_y(cost)."""
        if self.k >= self.trials:
            raise Budget
        x = np.clip(np.asarray(x, float), 0, 1)
        p = list(self.best)
        for i, v in zip(self.ids, self.space.to_params(x)):
            p[i] = v
        return to_y(self.trial(p, x)[0])

    def left(self):
        return self.trials - self.k

    def pop_size(self):
        """Population for the evolutionary methods: 4 + 2 per gain, but at least 3 generations."""
        return max(4, min(4 + 2 * self.d, self.left() // 3))

    def first_population(self, n):
        return np.vstack([self.x0, latin_hypercube(n - 1, self.d, self.rng)])

    def optimise(self):
        raise NotImplementedError


class RandomTuner(SearchTuner):
    """Baseline: Latin-hypercube random search. Any method worth using should beat it."""
    method = "random"

    def optimise(self):
        for x in self.first_population(max(2, self.left())):
            self.f(x)


class NelderMeadTuner(SearchTuner):
    """Nelder-Mead downhill simplex (1965): d+1 points; reflect the worst through the others'
    centroid, expand on success, contract or shrink on failure. No gradient, no model; it
    stays local and can be fooled by noise, so the best vertex is re-measured on a shrink."""
    method = "nelder-mead"

    def optimise(self):
        d = self.d
        pts = [self.x0]
        for j in range(d):
            x = self.x0.copy()
            x[j] += 0.25 if x[j] <= 0.75 else -0.25
            pts.append(x)
        S = [(self.f(x), x) for x in pts]
        while True:
            S.sort(key=lambda t: t[0])
            c = np.mean([x for _, x in S[:-1]], axis=0)
            fw, xw = S[-1]
            xr = np.clip(c + (c - xw), 0, 1)
            fr = self.f(xr)
            if fr < S[0][0]:
                xe = np.clip(c + 2 * (c - xw), 0, 1)
                fe = self.f(xe)
                S[-1] = (fe, xe) if fe < fr else (fr, xr)
            elif fr < S[-2][0]:
                S[-1] = (fr, xr)
            else:
                xc = c + 0.5 * ((xr if fr < fw else xw) - c)
                fc = self.f(xc)
                if fc < min(fr, fw):
                    S[-1] = (fc, xc)
                else:                               # shrink towards the best, re-measured
                    b = S[0][1]
                    S = [((S[0][0] + self.f(b)) / 2, b)] + [
                        (self.f(b + 0.5 * (x - b)), b + 0.5 * (x - b)) for _, x in S[1:]]


class SpsaTuner(SearchTuner):
    """SPSA (Spall 1992), the stochastic-approximation cousin of extremum seeking: every
    iteration perturbs all gains at once by +-c, two trials give a gradient estimate, and a
    gain-scheduled step goes downhill. Built for noisy costs; 2 trials per step."""
    method = "spsa"

    def optimise(self):
        x, a, A = self.x0.copy(), None, 2
        for k in range(10 ** 6):
            ck = 0.12 / (k + 1) ** 0.101
            delta = self.rng.choice([-1.0, 1.0], self.d)
            g = (self.f(x + ck * delta) - self.f(x - ck * delta)) / (2 * ck) * delta
            if a is None:                           # first step moves ~0.1 of the box
                a = 0.1 * (A + 1) ** 0.602 / max(np.abs(g).max(), 1e-6)
            step = a / (k + 1 + A) ** 0.602 * g
            x = np.clip(x - np.clip(step, -0.15, 0.15), 0, 1)


class GeneticTuner(SearchTuner):
    """Real-coded genetic algorithm: tournament selection, BLX-0.3 blend crossover, Gaussian
    mutation, the best individual carried over (elitism)."""
    method = "genetic"

    def optimise(self):
        n = self.pop_size()
        P = self.first_population(n)
        F = np.array([self.f(x) for x in P])
        while True:
            def pick():
                a, b = self.rng.integers(n, size=2)
                return P[a] if F[a] < F[b] else P[b]
            kids = [P[np.argmin(F)]]
            while len(kids) < n:
                p1, p2 = pick(), pick()
                lo, hi = np.minimum(p1, p2), np.maximum(p1, p2)
                span = hi - lo
                child = self.rng.uniform(lo - 0.3 * span, hi + 0.3 * span)
                mutate = self.rng.random(self.d) < max(1 / self.d, 0.3)
                child = child + mutate * self.rng.normal(0, 0.1, self.d)
                kids.append(np.clip(child, 0, 1))
            elite_f = F.min()
            P = np.array(kids)
            F = np.array([elite_f] + [self.f(x) for x in P[1:]])


class PsoTuner(SearchTuner):
    """Particle swarm optimisation (Kennedy & Eberhart 1995): each particle is pulled towards
    its own best and the swarm's best (inertia 0.6, c1 = c2 = 1.5, speed <= 0.2 per step)."""
    method = "pso"

    def optimise(self):
        n = self.pop_size()
        X = self.first_population(n)
        V = self.rng.normal(0, 0.05, X.shape)
        pb, pf = X.copy(), np.full(n, math.inf)
        while True:
            for j in range(n):
                fx = self.f(X[j])
                if fx < pf[j]:
                    pb[j], pf[j] = X[j].copy(), fx
            g = pb[np.argmin(pf)]
            r1, r2 = self.rng.random(X.shape), self.rng.random(X.shape)
            V = np.clip(0.6 * V + 1.5 * r1 * (pb - X) + 1.5 * r2 * (g - X), -0.2, 0.2)
            X = np.clip(X + V, 0, 1)


class DifferentialEvolutionTuner(SearchTuner):
    """Differential evolution (Storn & Price 1997), DE/rand/1/bin, F 0.7, CR 0.9: a trial
    vector replaces its parent only if it scores better."""
    method = "de"

    def optimise(self):
        n = self.pop_size()
        P = self.first_population(n)
        F = np.array([self.f(x) for x in P])
        while True:
            for j in range(n):
                a, b, c = P[self.rng.choice([i for i in range(n) if i != j], 3, replace=False)]
                cross = self.rng.random(self.d) < 0.9
                cross[self.rng.integers(self.d)] = True
                u = np.clip(np.where(cross, a + 0.7 * (b - c), P[j]), 0, 1)
                fu = self.f(u)
                if fu < F[j]:
                    P[j], F[j] = u, fu


class CmaesTuner(SearchTuner):
    """CMA-ES (Hansen), the usual (mu/mu_w, lambda) form: samples from a Gaussian whose mean,
    step size and covariance adapt to the successful steps. sigma0 0.2 of the box."""
    method = "cma-es"

    def optimise(self):
        d = self.d
        lam = 4 + int(3 * math.log(d))
        mu = lam // 2
        w = np.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
        w /= w.sum()
        mueff = 1 / (w ** 2).sum()
        cc = (4 + mueff / d) / (d + 4 + 2 * mueff / d)
        cs = (mueff + 2) / (d + mueff + 5)
        c1 = 2 / ((d + 1.3) ** 2 + mueff)
        cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((d + 2) ** 2 + mueff))
        damps = 1 + 2 * max(0, math.sqrt((mueff - 1) / (d + 1)) - 1) + cs
        chi = math.sqrt(d) * (1 - 1 / (4 * d) + 1 / (21 * d * d))
        m, sigma = self.x0.copy(), 0.2
        C, pc, ps = np.eye(d), np.zeros(d), np.zeros(d)
        for gen in range(1, 10 ** 6):
            vals, B = np.linalg.eigh(C)
            D = np.sqrt(np.clip(vals, 1e-12, None))
            Z = self.rng.standard_normal((lam, d))
            Y = Z @ (B * D).T
            X = np.clip(m + sigma * Y, 0, 1)
            Y = (X - m) / sigma                     # the clipped steps are the ones learned from
            order = np.argsort([self.f(x) for x in X])[:mu]
            yw = w @ Y[order]
            m = m + sigma * yw
            invsqrt = B @ np.diag(1 / D) @ B.T
            ps = (1 - cs) * ps + math.sqrt(cs * (2 - cs) * mueff) * invsqrt @ yw
            hs = np.linalg.norm(ps) / math.sqrt(1 - (1 - cs) ** (2 * gen)) < (1.4 + 2 / (d + 1)) * chi
            pc = (1 - cc) * pc + hs * math.sqrt(cc * (2 - cc) * mueff) * yw
            C = ((1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (not hs) * cc * (2 - cc) * C)
                 + cmu * (Y[order].T * w) @ Y[order])
            sigma = min(0.5, sigma * math.exp(cs / damps * (np.linalg.norm(ps) / chi - 1)))


class MlpEnsemble:
    """Deep ensemble (Lakshminarayanan et al. 2017) of small neural networks, d -> 32 -> 32 -> 1
    (tanh), each from its own random start, trained with Adam and weight decay in numpy. The
    spread between members stands in for the GP's uncertainty."""
    MEMBERS, HIDDEN, STEPS, LR, DECAY = 5, 32, 500, 0.01, 1e-3

    def __init__(self, X, y, rng):
        X, y = np.asarray(X, float), np.asarray(y, float)
        self.mu, self.sd = y.mean(), (y.std() if y.std() > 1e-9 else 1.0)
        z = (y - self.mu) / self.sd
        self.nets = [self.train(X, z, rng) for _ in range(self.MEMBERS)]

    def train(self, X, z, rng):
        d, h = X.shape[1], self.HIDDEN
        W = [rng.normal(0, 1 / math.sqrt(d), (d, h)), np.zeros(h),
             rng.normal(0, 1 / math.sqrt(h), (h, h)), np.zeros(h),
             rng.normal(0, 1 / math.sqrt(h), (h, 1)), np.zeros(1)]
        m1, m2 = [np.zeros_like(p) for p in W], [np.zeros_like(p) for p in W]
        for t in range(1, self.STEPS + 1):
            h1 = np.tanh(X @ W[0] + W[1])
            h2 = np.tanh(h1 @ W[2] + W[3])
            err = (h2 @ W[4] + W[5])[:, 0] - z
            g_out = 2 * err[:, None] / len(z)
            g = [None] * 6
            g[4], g[5] = h2.T @ g_out, g_out.sum(0)
            g_h2 = (g_out @ W[4].T) * (1 - h2 ** 2)
            g[2], g[3] = h1.T @ g_h2, g_h2.sum(0)
            g_h1 = (g_h2 @ W[2].T) * (1 - h1 ** 2)
            g[0], g[1] = X.T @ g_h1, g_h1.sum(0)
            for j in range(6):
                gj = g[j] + self.DECAY * W[j]
                m1[j] = 0.9 * m1[j] + 0.1 * gj
                m2[j] = 0.999 * m2[j] + 0.001 * gj * gj
                W[j] -= self.LR * (m1[j] / (1 - 0.9 ** t)) / (np.sqrt(m2[j] / (1 - 0.999 ** t)) + 1e-8)
        return W

    def predict(self, Xs):
        out = []
        for W in self.nets:
            h = np.tanh(np.tanh(Xs @ W[0] + W[1]) @ W[2] + W[3])
            out.append((h @ W[4] + W[5])[:, 0])
        out = np.array(out)
        return self.mu + self.sd * out.mean(0), self.sd * out.std(0)


class NeuralTuner(SearchTuner):
    """Deep-learning surrogate: like BayesTuner, but the cost model is the MlpEnsemble and the
    next trial minimises its lower confidence bound (mean - 1.5 x member spread)."""
    method = "neural"

    def optimise(self):
        d = self.d
        for x in self.first_population(max(1, d + 3 - len(self.X))):
            self.f(x)
        while True:
            net = MlpEnsemble(self.X, self.Y, self.rng)
            cand = self.rng.random((3000, d))
            if self.best_x is not None:
                cand = np.vstack([cand, np.clip(self.best_x + self.rng.normal(0, 0.07, (600, d)), 0, 1)])
            mean, std = net.predict(cand)
            self.f(cand[int(np.argmin(mean - 1.5 * std))])


class PolicyGradientTuner(SearchTuner):
    """Reinforcement learning, episodic: PGPE (Sehnke et al. 2010), policy-gradient over the
    gains themselves. The policy is a Gaussian over the unit cube; each step plays a
    symmetric pair mean +- eps (one trial each, reward = -score) and moves the mean towards
    the better one and the widths towards what paid off. Online RL inside the 200 Hz loop
    (actor-critic gain scheduling) would need firmware changes; this is the trial-level form."""
    method = "rl"

    def optimise(self):
        mu, sig = self.x0.copy(), np.full(self.d, 0.15)
        rewards = []
        while True:
            eps = self.rng.normal(0, sig)
            r_plus, r_minus = -self.f(mu + eps), -self.f(mu - eps)
            rewards += [r_plus, r_minus]
            scale = max(np.std(rewards), 0.05)
            base = np.mean(rewards[:-2]) if len(rewards) > 2 else (r_plus + r_minus) / 2
            mu = np.clip(mu + np.clip(0.5 * eps * (r_plus - r_minus) / (2 * scale), -0.15, 0.15), 0, 1)
            sig = np.clip(sig + 0.1 * ((r_plus + r_minus) / 2 - base) / scale * (eps ** 2 - sig ** 2) / sig,
                          0.03, 0.3)
