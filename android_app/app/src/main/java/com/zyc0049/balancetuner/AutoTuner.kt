package com.zyc0049.balancetuner

import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import java.io.File
import java.io.FileWriter
import java.io.PrintWriter
import java.util.Random
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.expm1
import kotlin.math.ln
import kotlin.math.ln1p
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Machine-learning auto-tune: the phone port of ml_tune.py (Bayesian optimisation with a
 * Gaussian process), run on the robot while it balances on the floor.
 *
 * Every trial (wait upright, apply the gains, settle, nudge forward/back, measure) gets the
 * same cost as autotune.py: angle std + 0.1 gyro RMS + 0.2 wheel speed RMS + 0.005 travel
 * + 20 x PWM-saturated fraction, a fall 100 + 10 x seconds lost. A GP learns log(1 + cost)
 * over the chosen gains (log-scaled box current / range .. current x range, normalised to
 * the unit cube); the next trial is where the expected improvement is highest. Trials from
 * earlier runs in the same log, taken with the same other gains, seed the model. The result
 * is the tried set with the lowest *predicted* cost, so a lucky trial does not win.
 *
 * [Method.TWIDDLE] is autotune.py's coordinate descent instead: +-step on one gain at a time
 * (start 25 %, x1.3 after a win, x0.6 otherwise, until below 4 %), a win re-measured and kept
 * only if the average still wins. The GP is then fitted only to draw the model view.
 *
 * The balance angle (angle0) can be tuned too: an offset, so it is searched linearly within
 * current +- 3 deg, and twiddle steps it additively (+-0.75 deg first).
 *
 * Runs on its own thread; [onTelemetry]/[onParam] are fed by the activity and the
 * [Listener] callbacks run on the main thread. The forward/back nudge goes out through
 * [nudgeY], which the activity's control frame uses while it is non-zero.
 */
class AutoTuner(
    private val cfg: Config,
    private val send: (ByteArray) -> Unit,
    private val logFile: File,
    private val listener: Listener,
) {
    enum class Method(val label: String, val defaultRange: Double) {
        ML("ML (Bayesian)", 2.0),
        TWIDDLE("Twiddle", 3.0),                // autotune.py's box: current / 3 .. x 3
    }

    class Config(
        val method: Method, val ids: List<Int>, val trials: Int, val trialS: Double, val nudge: Int,
        val range: Double, val trimAngle0: Boolean, val learn: Boolean,
    )

    interface Listener {
        fun onStatus(text: String)
        fun onTrial(r: Report)
        fun onDone(reason: String, start: FloatArray?, result: FloatArray?)
    }

    /** What the model looks like after a trial; [grid] is predicted cost (see [ModelView]). */
    class Report(
        val method: Method, val n: Int, val total: Int, val cost: Double, val fell: Boolean,
        val names: List<String>, val tried: FloatArray, val best: FloatArray, val bestPred: Double?,
        val space: Space, val xs: List<DoubleArray>, val costs: DoubleArray,
        val own: BooleanArray, val fellMask: BooleanArray, val bestX: DoubleArray?,
        val gridN: Int, val grid: DoubleArray, val gridLo: DoubleArray?, val gridHi: DoubleArray?,
    )

    @Volatile var nudgeY = 0
        private set
    @Volatile private var stopped = false
    private var thread: Thread? = null
    private val main = Handler(Looper.getMainLooper())
    private val telem = ArrayDeque<Pair<Long, Telemetry>>()
    private val echo = HashMap<Int, Float>()
    private val rng = Random()
    private var n = 0
    private var params = FloatArray(0)
    private var tried = FloatArray(0)               // what the last trial ran with
    private var start: FloatArray? = null
    private var best: FloatArray? = null
    private var log: PrintWriter? = null

    val running get() = thread?.isAlive == true

    fun start() { thread = Thread(::run, "autotune").apply { start() } }

    /** Takes effect after the current trial. */
    fun stop() { stopped = true }

    fun onTelemetry(t: Telemetry) = synchronized(telem) {
        telem.addLast(SystemClock.elapsedRealtime() to t)
        while (telem.size > 1500) telem.removeFirst()
    }

    fun onParam(id: Int, value: Float) = synchronized(echo) { echo[id] = value }

    private class Stopped : Exception()
    private class TuneError(msg: String) : Exception(msg)

    // ---------------------------------------------------------------- robot

    private fun now() = SystemClock.elapsedRealtime()

    private fun since(t0: Long) = synchronized(telem) { telem.filter { it.first >= t0 }.map { it.second } }

    private fun checkStop() { if (stopped) throw Stopped() }

    private fun post(f: () -> Unit) { main.post(f) }

    private fun status(s: String) = post { listener.onStatus(s) }

    private fun setParam(id: Int, value: Float): Float {
        repeat(3) {
            synchronized(echo) { echo.remove(id) }
            send(Protocol.setParam(id, value))
            val end = now() + 500
            while (now() < end) {
                synchronized(echo) { echo[id] }?.let { return it }
                Thread.sleep(10)
            }
        }
        throw TuneError("no echo for ${PARAMS[id].name}")
    }

    private fun getParams(): FloatArray {
        synchronized(echo) { echo.clear() }
        send(Protocol.command(Protocol.CMD_GET_PARAMS))
        val end = now() + 1500
        while (now() < end) {
            synchronized(echo) { if (echo.size >= PARAMS.size) return FloatArray(PARAMS.size) { echo[it]!! } }
            Thread.sleep(20)
        }
        throw TuneError("no parameter dump from the robot")
    }

    private fun apply(p: FloatArray) {
        for (i in p.indices) if (abs(params[i] - p[i]) > 1e-6f) params[i] = setParam(i, p[i])
    }

    /** Balanced (within 15 deg raw, not fallen) for 1.5 s. */
    private fun waitUpright(prompt: Boolean) {
        if (prompt) status("Fell: stand the robot up (it balances by itself once upright)")
        while (true) {
            checkStop()
            Thread.sleep(250)
            val f = since(now() - 1500)
            if (f.size >= 25 && f.all { abs(it.angle) < 15 && !fallen(it) }) return
        }
    }

    private fun drive(y: Int, ms: Long) {
        nudgeY = y
        Thread.sleep(ms)
    }

    class Metrics(
        val fell: Boolean, val angleMean: Double, val angleStd: Double, val gyroRms: Double,
        val speedRms: Double, val travel: Double, val sat: Double,
    )

    /** One trial with the parameters already applied; autotune.measure(). */
    private fun measure(nudge: Int): Pair<Double, Metrics> {
        Thread.sleep(1500)                                      // settle after the change
        val t0 = now()
        if (nudge != 0) {
            Thread.sleep(1000)
            try { drive(nudge, 400); drive(-nudge, 400) } finally { nudgeY = 0 }
            Thread.sleep(150)
        }
        val trialMs = (cfg.trialS * 1000).toLong()
        while (now() - t0 < trialMs) {
            Thread.sleep(100)
            if (since(t0).any(::fallen)) break
        }
        val frames = since(t0)
        if (frames.isEmpty()) throw TuneError("no telemetry received")
        val fallAt = frames.indexOfFirst(::fallen).takeIf { it >= 0 }
        var ok = if (fallAt != null) frames.subList(0, fallAt) else frames
        if (ok.size < 5) ok = frames.take(5)
        val ang = ok.map { it.angle.toDouble() }
        val mean = ang.average()
        val vel = ok.map { (it.countL + it.countR) / 2.0 }
        var pos = 0.0
        var travel = 0.0
        for (v in vel) { pos += v; travel = max(travel, abs(pos)) }
        val m = Metrics(
            fell = fallAt != null, angleMean = mean,
            angleStd = sqrt(ang.sumOf { (it - mean) * (it - mean) } / ang.size),
            gyroRms = sqrt(ok.sumOf { it.gyro.toDouble() * it.gyro } / ok.size),
            speedRms = sqrt(vel.sumOf { it * it } / vel.size), travel = travel,
            sat = ok.count { max(abs(it.pwmL), abs(it.pwmR)) >= PWM_SAT }.toDouble() / ok.size,
        )
        val cost = if (fallAt != null) {
            val survived = fallAt.toDouble() / frames.size * cfg.trialS
            100 + 10 * (cfg.trialS - survived)
        } else {
            m.angleStd + 0.1 * m.gyroRms + 0.2 * m.speedRms + 0.005 * m.travel + 20 * m.sat
        }
        return cost to m
    }

    /** Wait upright with the last parameters, switch to [p], measure, log. */
    private fun evaluate(stage: String, p: FloatArray, nudge: Int = cfg.nudge): Pair<Double, Metrics> {
        checkStop()
        waitUpright(false)
        apply(p)
        tried = params.copyOf()                     // before a fall puts the best values back
        val (cost, m) = measure(nudge)
        n++
        log?.apply {
            println((listOf("$n", stage) + params.map { "%.5g".format(it) } + listOf(
                "%.3f".format(cost), if (m.fell) "True" else "False", "%.4g".format(m.angleMean),
                "%.4g".format(m.angleStd), "%.4g".format(m.gyroRms), "%.4g".format(m.speedRms),
                "%.4g".format(m.travel), "%.4g".format(m.sat),
            )).joinToString(","))
            flush()
        }
        if (m.fell) {
            apply(best!!)
            waitUpright(true)
            status("Running")
        }
        return cost to m
    }

    // ---------------------------------------------------------------- stages

    private fun trimAngle0() {
        repeat(2) {
            status("Trimming the balance angle (robot left alone)")
            val p = params.copyOf()
            val (_, m) = evaluate("angle0", p, nudge = 0)
            if (m.fell) return@repeat
            val old = p[P_ANGLE0]
            p[P_ANGLE0] = (-m.angleMean).toFloat().coerceIn(-15f, 15f)
            apply(p)
            best = params.copyOf()
            status("Balance angle %.2f -> %.2f".format(old, params[P_ANGLE0]))
            if (abs(params[P_ANGLE0] - old) < 0.3f) return
        }
    }

    /** Past trials with the same other gains, as unit-cube points and y = log(1 + cost). */
    private fun seed(space: Space): Pair<MutableList<DoubleArray>, MutableList<Double>> {
        val xs = mutableListOf<DoubleArray>()
        val ys = mutableListOf<Double>()
        if (!cfg.learn) return xs to ys
        val others = GAINS.filter { it !in cfg.ids }
        for (r in readHistory(logFile)) {
            if (others.any { abs(r[it] - params[it]) > 0.1 * max(abs(params[it].toDouble()), 1e-6) }) continue
            if (P_ANGLE0 !in cfg.ids && abs(r[P_ANGLE0] - params[P_ANGLE0]) > 1.0) continue
            val x = space.toUnit(cfg.ids.map { r[it] }) ?: continue
            xs += x
            ys += toY(r[PARAMS.size])
        }
        return xs to ys
    }

    // model bookkeeping shared by both methods (see begin/trial)
    private lateinit var space: Space
    private lateinit var stage: String
    private val xs = mutableListOf<DoubleArray>()
    private val ys = mutableListOf<Double>()
    private val own = mutableListOf<Boolean>()
    private val fellMask = mutableListOf<Boolean>()
    private var k = 0
    private var gp: GaussianProcess? = null
    private var bestX: DoubleArray? = null
    private var bestPred: Double? = null
    private var bestCost: Double? = null           // twiddle: mean of the two trials that won

    /** Search box around the current gains, seeded with matching past trials. */
    private fun begin(factor: Double) {
        space = Space(cfg.ids, params, factor)
        stage = (if (cfg.method == Method.ML) "ml:" else "twiddle:") + space.names.joinToString("+")
        val (sx, sy) = seed(space)
        xs += sx; ys += sy
        repeat(sx.size) { own += false; fellMask += false }
        if (sx.isNotEmpty()) status("Learned from ${sx.size} past trials")
    }

    /** evaluate() plus the model bookkeeping and the report. */
    private fun trial(p: FloatArray, x: DoubleArray? = null): Pair<Double, Metrics> {
        val (cost, m) = evaluate(stage, p)
        k++
        // the firmware may clamp/round: learn what was really applied
        (space.toUnit(cfg.ids.map { tried[it].toDouble() }) ?: x)?.let {
            xs += it; ys += toY(cost); own += true; fellMask += m.fell
        }
        val model = if (xs.isEmpty()) null else GaussianProcess(xs, ys.toDoubleArray())
        gp = model
        if (cfg.method == Method.ML) chooseBestMl(model!!) else {
            bestX = space.toUnit(cfg.ids.map { best!![it].toDouble() })
            bestPred = bestCost
        }
        val r = model?.let { report(cost, m.fell, it) }
        if (r != null) post { listener.onTrial(r) }
        return cost to m
    }

    /** The tried set (this run, did not fall) with the lowest predicted cost. */
    private fun chooseBestMl(model: GaussianProcess) {
        val means = xs.map { model.predict(it).first }
        xs.indices.filter { own[it] && !fellMask[it] }.minByOrNull { means[it] }?.let { j ->
            bestX = xs[j]
            bestPred = expm1(means[j])
            cfg.ids.forEachIndexed { q, i -> best!![i] = space.value(q, xs[j][q]).toFloat() }
        }
    }

    private fun searchMl() {
        begin(cfg.range)
        val ids = cfg.ids
        val d = ids.size
        val init = mutableListOf(space.toUnit(ids.map { params[it].toDouble() })!!)
        init += latinHypercube(max(0, d + 2 - xs.size), d)
        while (k < cfg.trials) {
            val x = if (k < init.size) init[k] else {
                val g = gp!!
                val yRef = xs.minOf { g.predict(it).first }
                val cand = List(2000) { DoubleArray(d) { rng.nextDouble() } } +
                    (bestX?.let { b -> List(600) { DoubleArray(d) { j -> (b[j] + 0.07 * rng.nextGaussian()).coerceIn(0.0, 1.0) } } }
                        ?: emptyList())
                cand.maxByOrNull { g.expectedImprovement(it, yRef) }!!
            }
            val p = best!!.copyOf()
            ids.forEachIndexed { j, i -> p[i] = space.value(j, x[j]).toFloat() }
            trial(p, x)
        }
    }

    /** autotune.py's Tuner.twiddle over the ticked gains. */
    private fun searchTwiddle() {
        begin(cfg.range)
        val ids = cfg.ids
        var bestP = best!!.copyOf()
        // angle0 is an offset: additive steps of step x ANGLE0_SPAN deg within +-ANGLE0_SPAN
        val lo = ids.associateWith {
            if (it == P_ANGLE0) max(P_MIN[it], bestP[it] - ANGLE0_SPAN) else max(P_MIN[it], bestP[it] / cfg.range)
        }
        val hi = ids.associateWith {
            if (it == P_ANGLE0) min(P_MAX[it], bestP[it] + ANGLE0_SPAN) else min(P_MAX[it], bestP[it] * cfg.range)
        }
        status("Measuring the starting gains (2 trials)")
        val c0 = trial(bestP).first
        bestCost = (c0 + trial(bestP).first) / 2
        val step = ids.associateWith { 0.25 }.toMutableMap()

        while (k < cfg.trials && step.values.max() > 0.04) {
            for (i in ids) {
                var improved = false
                for (sign in intArrayOf(1, -1)) {
                    if (k >= cfg.trials) break
                    val cand = bestP.copyOf()
                    val v = if (i == P_ANGLE0) bestP[i] + sign * step[i]!! * ANGLE0_SPAN
                    else bestP[i] * (1 + sign * step[i]!!)
                    cand[i] = v.coerceIn(lo[i]!!, hi[i]!!).toFloat()
                    if (abs(cand[i] - bestP[i]) < 1e-9f) continue
                    val (c, m) = trial(cand)
                    if (m.fell || c >= bestCost!! || k >= cfg.trials) continue
                    val (c2, m2) = trial(cand)          // confirm: noise must not win
                    if (!m2.fell && (c + c2) / 2 < bestCost!!) {
                        bestP = cand
                        bestCost = (c + c2) / 2
                        best = bestP.copyOf()
                        improved = true
                        status("++ ${LOG_NAMES[i]} = %.4g (best cost %.2f)".format(bestP[i], bestCost))
                        break
                    }
                }
                step[i] = step[i]!! * if (improved) 1.3 else 0.6
                if (k >= cfg.trials) break
            }
        }
        status(if (k < cfg.trials) "Steps below 4 %: converged" else "Trial budget used")
    }

    private fun report(cost: Double, fell: Boolean, gp: GaussianProcess): Report {
        val d = space.ids.size
        val base = bestX ?: xs.last()
        val gn = if (d == 1) 80 else 32
        var lo: DoubleArray? = null
        var hi: DoubleArray? = null
        val grid = if (d == 1) {
            lo = DoubleArray(gn); hi = DoubleArray(gn)
            DoubleArray(gn) { i ->
                val (mean, std) = gp.predict(doubleArrayOf(i / (gn - 1.0)))
                lo[i] = expm1(mean - 2 * std); hi[i] = expm1(mean + 2 * std)
                expm1(mean)
            }
        } else {
            // first two gains; any others held at the current best
            DoubleArray(gn * gn) { c ->
                val x = base.copyOf()
                x[0] = (c % gn) / (gn - 1.0); x[1] = (c / gn) / (gn - 1.0)
                expm1(gp.predict(x).first)
            }
        }
        return Report(
            cfg.method, k, cfg.trials, cost, fell, space.names, FloatArray(d) { tried[space.ids[it]] },
            FloatArray(d) { best!![space.ids[it]] }, bestPred, space, xs.map { it.copyOf() },
            ys.map { expm1(it) }.toDoubleArray(), own.toBooleanArray(), fellMask.toBooleanArray(),
            bestX?.copyOf(), gn, grid, lo, hi,
        )
    }

    private fun run() {
        var reason = "finished"
        try {
            params = getParams()
            start = params.copyOf()
            best = params.copyOf()
            val fresh = !logFile.exists() || logFile.length() == 0L
            log = PrintWriter(FileWriter(logFile, true)).also { if (fresh) it.println(LOG_HEADER) }
            status("Waiting for the robot to balance")
            waitUpright(false)
            status("Running")
            if (cfg.trimAngle0) trimAngle0()
            if (cfg.method == Method.ML) searchMl() else searchTwiddle()
        } catch (_: Stopped) {
            reason = "stopped"
        } catch (_: InterruptedException) {
            reason = "stopped"
        } catch (e: TuneError) {
            reason = "error: ${e.message}"
        } catch (e: java.io.IOException) {
            reason = "error: ${e.message}"
        } finally {
            nudgeY = 0
            log?.close()
            best?.let {
                try { apply(it) } catch (e: TuneError) { reason += " (could not apply the best values: ${e.message})" }
            }
            val s = start
            val r = params.copyOf()
            post { listener.onDone(reason, s, if (s == null) null else r) }
        }
    }

    private fun latinHypercube(count: Int, d: Int): List<DoubleArray> {
        if (count == 0) return emptyList()
        val cols = List(d) { (0 until count).shuffled(rng).map { (it + rng.nextDouble()) / count } }
        return List(count) { i -> DoubleArray(d) { j -> cols[j][i] } }
    }

    /**
     * Box around the starting values, mapped to [0, 1]^d: gains on a log scale
     * (current / factor .. x factor), angle0 linear (current +- ANGLE0_SPAN deg).
     */
    class Space(val ids: List<Int>, centre: FloatArray, factor: Double) {
        val names = ids.map { LOG_NAMES[it] }
        val log = BooleanArray(ids.size) { ids[it] != P_ANGLE0 }
        val lo = DoubleArray(ids.size)
        val hi = DoubleArray(ids.size)

        init {
            ids.forEachIndexed { k, i ->
                if (i == P_ANGLE0) {
                    lo[k] = max(P_MIN[i], centre[i] - ANGLE0_SPAN)
                    hi[k] = min(P_MAX[i], centre[i] + ANGLE0_SPAN)
                    return@forEachIndexed
                }
                val c = if (centre[i] > 0) centre[i].toDouble() else 0.05 * P_MAX[i]
                lo[k] = ln(max(c / factor, 1e-3 * P_MAX[i]))
                hi[k] = ln(min(c * factor, P_MAX[i]))
            }
        }

        fun value(k: Int, u: Double) = (lo[k] + u * (hi[k] - lo[k])).let { if (log[k]) exp(it) else it }

        /** null when a value lies outside the box (or a gain is not positive). */
        fun toUnit(values: List<Double>): DoubleArray? {
            val x = DoubleArray(values.size)
            for (k in values.indices) {
                if (log[k] && values[k] <= 0) return null
                val u = ((if (log[k]) ln(values[k]) else values[k]) - lo[k]) / (hi[k] - lo[k])
                if (u < -1e-6 || u > 1 + 1e-6) return null
                x[k] = u.coerceIn(0.0, 1.0)
            }
            return x
        }
    }

    companion object {
        const val P_ANGLE0 = 6
        val TUNABLE = listOf(0, 1, 2, 3, P_ANGLE0)  // angle Kp, Kd, speed Kp, Ki, balance angle
        private val GAINS = TUNABLE.take(4)         // multiplicative: log scale; angle0 is an offset
        private const val ANGLE0_SPAN = 3.0         // angle0 searched within current +- this (deg)
        private const val FALL_DEG = 30f
        private const val PWM_SAT = 250f
        private const val COST_CAP = 300.0
        // limits of the firmware (P_MIN/P_MAX in autotune.py)
        private val P_MIN = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -15.0, 0.0)
        private val P_MAX = doubleArrayOf(100.0, 10.0, 50.0, 2.0, 50.0, 5.0, 15.0, 100.0)
        val LOG_NAMES = listOf("kp", "kd", "kp_speed", "ki_speed", "kp_turn", "kd_turn", "angle0", "drive_speed")
        val LOG_HEADER = (listOf("trial", "stage") + LOG_NAMES + listOf(
            "cost", "fell", "angle_mean", "angle_std", "gyro_rms", "speed_rms", "travel", "sat")).joinToString(",")

        fun fallen(t: Telemetry) = t.fallen || abs(t.angle) > FALL_DEG

        fun toY(cost: Double) = ln1p(min(cost, COST_CAP))

        /** Rows of an autotune-format CSV: the 8 parameters, then the cost. */
        fun readHistory(f: File): List<DoubleArray> {
            if (!f.exists()) return emptyList()
            val lines = f.readLines()
            val head = lines.firstOrNull()?.split(",") ?: return emptyList()
            val cols = (LOG_NAMES + "cost").map { head.indexOf(it) }
            if (cols.any { it < 0 }) return emptyList()
            return lines.drop(1).mapNotNull { line ->
                val c = line.split(",")
                val v = cols.map { c.getOrNull(it)?.toDoubleOrNull() }
                if (v.any { it == null }) null else DoubleArray(v.size) { v[it]!! }
            }
        }
    }
}

/** Zero-mean GP on standardised targets, RBF kernel shared over unit-cube inputs (ml_tune.py). */
class GaussianProcess(private val xs: List<DoubleArray>, y: DoubleArray) {
    private val mu = y.average()
    private val sd: Double
    private var ls = 0.0
    private lateinit var chol: Array<DoubleArray>
    private lateinit var alpha: DoubleArray

    init {
        val sdev = sqrt(y.sumOf { (it - mu) * (it - mu) } / y.size)
        sd = if (y.size > 1 && sdev > 1e-9) sdev else 1.0
        val z = DoubleArray(y.size) { (y[it] - mu) / sd }
        var best = Double.NEGATIVE_INFINITY
        for (l in LENGTHS) for (noise in NOISES) {
            val n = xs.size
            val k = Array(n) { i -> DoubleArray(n) { j -> rbf(xs[i], xs[j], l) + if (i == j) noise else 0.0 } }
            val c = cholesky(k) ?: continue
            val a = backSolve(c, forwardSolve(c, z))
            var lml = 0.0
            for (i in 0 until n) lml += -0.5 * z[i] * a[i] - ln(c[i][i])
            if (lml > best) { best = lml; ls = l; chol = c; alpha = a }
        }
    }

    /** Posterior mean and std of the latent y at [x]. */
    fun predict(x: DoubleArray): Pair<Double, Double> {
        val ks = DoubleArray(xs.size) { rbf(x, xs[it], ls) }
        var mean = 0.0
        for (i in ks.indices) mean += ks[i] * alpha[i]
        val v = forwardSolve(chol, ks)
        val variance = max(1e-12, 1 - v.sumOf { it * it })
        return mu + sd * mean to sd * sqrt(variance)
    }

    fun expectedImprovement(x: DoubleArray, yBest: Double, xi: Double = 0.01): Double {
        val (mean, std) = predict(x)
        val imp = yBest - mean - xi
        val z = imp / std
        return imp * normCdf(z) + std * exp(-0.5 * z * z) / sqrt(2 * Math.PI)
    }

    private fun rbf(a: DoubleArray, b: DoubleArray, l: Double): Double {
        var s = 0.0
        for (i in a.indices) { val d = a[i] - b[i]; s += d * d }
        return exp(-0.5 * s / (l * l))
    }

    private companion object {
        val LENGTHS = doubleArrayOf(0.08, 0.15, 0.25, 0.4, 0.7, 1.2)
        val NOISES = doubleArrayOf(0.01, 0.05, 0.15, 0.4)

        fun cholesky(a: Array<DoubleArray>): Array<DoubleArray>? {
            val n = a.size
            val l = Array(n) { DoubleArray(n) }
            for (i in 0 until n) for (j in 0..i) {
                var s = a[i][j]
                for (k in 0 until j) s -= l[i][k] * l[j][k]
                if (i == j) { if (s <= 0) return null; l[i][i] = sqrt(s) } else l[i][j] = s / l[j][j]
            }
            return l
        }

        /** L x = b */
        fun forwardSolve(l: Array<DoubleArray>, b: DoubleArray) = DoubleArray(b.size).also { x ->
            for (i in b.indices) {
                var s = b[i]
                for (k in 0 until i) s -= l[i][k] * x[k]
                x[i] = s / l[i][i]
            }
        }

        /** L^T x = b */
        fun backSolve(l: Array<DoubleArray>, b: DoubleArray) = DoubleArray(b.size).also { x ->
            for (i in b.indices.reversed()) {
                var s = b[i]
                for (k in i + 1 until b.size) s -= l[k][i] * x[k]
                x[i] = s / l[i][i]
            }
        }

        /** Abramowitz-Stegun 7.1.26 erf, |error| < 1.5e-7. */
        fun normCdf(z: Double): Double {
            val x = abs(z) / sqrt(2.0)
            val t = 1 / (1 + 0.3275911 * x)
            val erf = 1 - t * (0.254829592 + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429)))) * exp(-x * x)
            return 0.5 * (1 + if (z >= 0) erf else -erf)
        }
    }
}
