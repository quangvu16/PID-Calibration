package com.zyc0049.balancetuner

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Rect
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View
import kotlin.math.log10
import kotlin.math.max
import kotlin.math.min

/**
 * The auto-tune model after the latest trial (AutoTuner.Report): on top every trial's cost
 * (log scale; x = fell, green = best trial so far, dashed = the model's best), below the
 * Gaussian process' predicted cost over the tuned gains (log axes): a map for two or more
 * gains (others held at the best, dark = better), a curve with a 2-std band for one.
 * With twiddle the map is only a picture of the trials; it does not steer that search.
 * Dots: blue = this run, grey = earlier runs, red = fell; star = current best.
 */
class ModelView @JvmOverloads constructor(context: Context, attrs: AttributeSet? = null) :
    View(context, attrs) {

    var report: AutoTuner.Report? = null
        set(v) { field = v; bitmap = null; invalidate() }

    private val density = resources.displayMetrics.density
    private val c = { id: Int -> context.getColor(id) }
    private val bg = Paint().apply { color = c(R.color.surface) }
    private val grid = Paint().apply { color = c(R.color.line); strokeWidth = density }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 11 * density; color = c(R.color.muted) }
    private val stroke = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 1.6f * density }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG)
    private val dashed = Paint(stroke).apply {
        color = c(R.color.ok); strokeWidth = density
        pathEffect = android.graphics.DashPathEffect(floatArrayOf(4 * density, 4 * density), 0f)
    }
    private val path = Path()
    private var bitmap: Bitmap? = null

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        setMeasuredDimension(w, resolveSize((w * 1.15f).toInt(), heightMeasureSpec))
    }

    override fun onDraw(canvas: Canvas) {
        canvas.drawRect(0f, 0f, width.toFloat(), height.toFloat(), bg)
        val r = report ?: return
        val left = 40 * density
        val right = width - 10 * density
        val split = height * 0.36f
        drawCosts(canvas, r, RectF(left, 18 * density, right, split - 18 * density))
        drawModel(canvas, r, RectF(left, split + 16 * density, right, height - 28 * density))
    }

    private fun drawCosts(canvas: Canvas, r: AutoTuner.Report, box: RectF) {
        val costs = r.costs.indices.filter { r.own[it] }.map { r.costs[it] }
        val fell = r.fellMask.indices.filter { r.own[it] }.map { r.fellMask[it] }
        val dash = if (r.method == AutoTuner.Method.TWIDDLE) "best, mean of 2" else "model's best"
        canvas.drawText("Cost per trial (log; x = fell; dashed = $dash)", box.left, box.top - 6 * density, text)
        if (costs.isEmpty()) return
        val lo = log10(max(0.5, costs.min()))
        val hi = log10(max(costs.max(), 10.0)) + 0.05
        fun y(v: Double) = box.bottom - ((log10(max(v, 0.5)) - lo) / (hi - lo)).toFloat() * box.height()
        fun x(i: Int) = box.left + (if (costs.size == 1) 0.5f else i / (costs.size - 1f)) * box.width()
        axes(canvas, box, lo, hi)
        stroke.color = c(R.color.line)
        path.reset()
        costs.forEachIndexed { i, v -> if (i == 0) path.moveTo(x(i), y(v)) else path.lineTo(x(i), y(v)) }
        canvas.drawPath(path, stroke)
        // best trial so far (falls do not count)
        stroke.color = c(R.color.ok)
        path.reset()
        var runMin = Double.POSITIVE_INFINITY
        var prevY: Float? = null
        costs.forEachIndexed { i, v ->
            if (!fell[i]) runMin = min(runMin, v)
            if (!runMin.isFinite()) return@forEachIndexed
            val yy = y(runMin)
            prevY.let { if (it == null) path.moveTo(x(i), yy) else { path.lineTo(x(i), it); path.lineTo(x(i), yy) } }
            prevY = yy
        }
        canvas.drawPath(path, stroke)
        r.bestPred?.let { canvas.drawLine(box.left, y(it), box.right, y(it), dashed) }
        costs.forEachIndexed { i, v -> dot(canvas, x(i), y(v), if (fell[i]) c(R.color.bad) else c(R.color.accent), fell[i]) }
        canvas.drawText("trial ${costs.size}", box.right - text.measureText("trial ${costs.size}"),
            box.bottom + 14 * density, text)
    }

    private fun drawModel(canvas: Canvas, r: AutoTuner.Report, box: RectF) {
        val s = r.space
        var title = if (r.names.size == 1) "Predicted cost vs ${r.names[0]} (band = 2 std)"
        else "Predicted cost, dark = better" + if (r.names.size > 2) " (others at best)" else ""
        if (r.method == AutoTuner.Method.TWIDDLE) title += " - fit for display only"
        else if (r.method != AutoTuner.Method.ML) title += " - fit picks the result only"
        canvas.drawText(title, box.left, box.top - 6 * density, text)
        fun px(u: Double) = box.left + u.toFloat() * box.width()
        fun py(u: Double) = box.bottom - u.toFloat() * box.height()
        val colours = r.xs.indices.map {
            if (r.fellMask[it]) c(R.color.bad) else if (r.own[it]) c(R.color.accent) else c(R.color.s_raw)
        }
        if (r.names.size == 1) {
            val g = r.grid
            val bandLo = r.gridLo!!
            val bandHi = r.gridHi!!
            val lo = log10(max(0.5, min(g.min(), bandLo.min())))
            val hi = log10(maxOf(g.max(), bandHi.max(), 10.0)) + 0.05
            fun cy(v: Double) = box.bottom - ((log10(max(v, 0.5)) - lo) / (hi - lo)).toFloat() * box.height()
            axes(canvas, box, lo, hi)
            fun gx(i: Int) = px(i / (r.gridN - 1.0))
            path.reset()
            for (i in 0 until r.gridN) if (i == 0) path.moveTo(gx(i), cy(bandHi[i])) else path.lineTo(gx(i), cy(bandHi[i]))
            for (i in r.gridN - 1 downTo 0) path.lineTo(gx(i), cy(bandLo[i]))
            path.close()
            fill.color = c(R.color.accent_soft)
            canvas.drawPath(path, fill)
            path.reset()
            for (i in 0 until r.gridN) if (i == 0) path.moveTo(gx(i), cy(g[i])) else path.lineTo(gx(i), cy(g[i]))
            stroke.color = c(R.color.accent)
            canvas.drawPath(path, stroke)
            r.xs.forEachIndexed { i, x -> dot(canvas, px(x[0]), cy(r.costs[i]), colours[i], r.fellMask[i]) }
            r.bestX?.let { star(canvas, px(it[0]), cy(r.bestPred ?: r.grid.min())) }
        } else {
            val bmp = bitmap ?: heatmap(r).also { bitmap = it }
            canvas.drawBitmap(bmp, Rect(0, 0, r.gridN, r.gridN), box, Paint(Paint.FILTER_BITMAP_FLAG))
            r.xs.forEachIndexed { i, x -> dot(canvas, px(x[0]), py(x[1]), colours[i], r.fellMask[i], outline = true) }
            r.bestX?.let { star(canvas, px(it[0]), py(it[1])) }
            // y axis: second gain
            for (u in doubleArrayOf(0.0, 0.5, 1.0)) {
                val label = "%.3g".format(s.value(1, u))
                canvas.drawText(label, box.left - text.measureText(label) - 4 * density, py(u) + 4 * density, text)
            }
            canvas.save()
            canvas.rotate(-90f, 10 * density, box.centerY())
            canvas.drawText(r.names[1], 10 * density - text.measureText(r.names[1]) / 2, box.centerY(), text)
            canvas.restore()
        }
        // x axis: first gain (log scale)
        for (u in doubleArrayOf(0.0, 0.5, 1.0)) {
            val label = "%.3g".format(s.value(0, u))
            val w = text.measureText(label)
            canvas.drawText(label, (px(u) - w / 2).coerceIn(box.left, box.right - w), box.bottom + 13 * density, text)
        }
        canvas.drawText(r.names[0], box.centerX() - text.measureText(r.names[0]) / 2, box.bottom + 25 * density, text)
    }

    /** Log-cost axis: horizontal grid lines at 1, 10, 100 and labels. */
    private fun axes(canvas: Canvas, box: RectF, lo: Double, hi: Double) {
        for (e in -1..3) {
            val v = Math.pow(10.0, e.toDouble())
            val lv = log10(v)
            if (lv < lo || lv > hi) continue
            val y = box.bottom - ((lv - lo) / (hi - lo)).toFloat() * box.height()
            canvas.drawLine(box.left, y, box.right, y, grid)
            val label = "%.3g".format(v)
            canvas.drawText(label, box.left - text.measureText(label) - 4 * density, y + 4 * density, text)
        }
    }

    private fun heatmap(r: AutoTuner.Report): Bitmap {
        val n = r.gridN
        val logs = r.grid.map { log10(max(it, 1.0)) }
        val lo = logs.min()
        val hi = max(logs.max(), lo + 1e-6)
        val px = IntArray(n * n) { i ->
            val gx = i % n
            val gy = n - 1 - i / n                      // bitmap row 0 is the top
            viridis((logs[gy * n + gx] - lo) / (hi - lo))
        }
        return Bitmap.createBitmap(px, n, n, Bitmap.Config.ARGB_8888)
    }

    private fun dot(canvas: Canvas, x: Float, y: Float, colour: Int, cross: Boolean, outline: Boolean = false) {
        val d = 3.5f * density
        if (cross) {
            stroke.color = colour
            canvas.drawLine(x - d, y - d, x + d, y + d, stroke)
            canvas.drawLine(x - d, y + d, x + d, y - d, stroke)
            return
        }
        fill.color = colour
        canvas.drawCircle(x, y, d, fill)
        if (outline) { stroke.color = Color.WHITE; stroke.strokeWidth = density; canvas.drawCircle(x, y, d, stroke); stroke.strokeWidth = 1.6f * density }
    }

    private fun star(canvas: Canvas, x: Float, y: Float) {
        val outer = 8 * density
        val inner = outer * 0.45f
        path.reset()
        for (k in 0 until 10) {
            val a = Math.PI / 2 + k * Math.PI / 5
            val rad = if (k % 2 == 0) outer else inner
            val px = x + (rad * Math.cos(a)).toFloat()
            val py = y - (rad * Math.sin(a)).toFloat()
            if (k == 0) path.moveTo(px, py) else path.lineTo(px, py)
        }
        path.close()
        fill.color = Color.WHITE
        canvas.drawPath(path, fill)
        stroke.color = Color.BLACK; stroke.strokeWidth = density
        canvas.drawPath(path, stroke)
        stroke.strokeWidth = 1.6f * density
    }

    private companion object {
        val STOPS = intArrayOf(0x440154, 0x3B528B, 0x21918C, 0x5EC962, 0xFDE725)

        /** Matplotlib's viridis from five stops; 0 = dark purple (low cost). */
        fun viridis(t: Double): Int {
            val f = t.coerceIn(0.0, 1.0) * (STOPS.size - 1)
            val i = min(f.toInt(), STOPS.size - 2)
            val w = f - i
            fun ch(shift: Int) = ((STOPS[i] shr shift and 0xFF) * (1 - w) + (STOPS[i + 1] shr shift and 0xFF) * w).toInt()
            return Color.rgb(ch(16), ch(8), ch(0))
        }
    }
}
