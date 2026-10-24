package com.zyc0049.balancetuner

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import kotlin.math.abs
import kotlin.math.max

/**
 * Scrolling strip chart. Every series is scaled on its own, symmetric around zero, so a
 * small angle and a +-255 PWM are both readable; the legend gives each series' range.
 * With [sharedScale] all series use one range instead, so they compare directly (the
 * per-term panels). The zero line is the middle of the plot.
 */
class ChartView @JvmOverloads constructor(context: Context, attrs: AttributeSet? = null) :
    View(context, attrs) {

    class Series(val name: String, val unit: String, val colorRes: Int, val minRange: Float, var enabled: Boolean)

    var series: List<Series> = emptyList()
        set(v) { field = v; data = Array(v.size) { FloatArray(CAPACITY) }; count = 0; head = 0; invalidate() }

    var paused = false
    var title: String? = null
    var sharedScale = false
    var unit = ""                   // shown once in the legend when sharedScale
    var markers = floatArrayOf()    // dashed horizontal lines at these values, e.g. PWM saturation

    private var data = arrayOf<FloatArray>()
    private var head = 0            // index the next sample goes to
    private var count = 0

    private val density = resources.displayMetrics.density
    private val c = { id: Int -> context.getColor(id) }
    private val bg = Paint().apply { color = c(R.color.surface) }
    private val grid = Paint().apply { color = c(R.color.line); strokeWidth = density }
    private val line = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 1.6f * density; strokeJoin = Paint.Join.ROUND
    }
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { textSize = 11 * density; typeface = android.graphics.Typeface.MONOSPACE }
    private val muted = Paint(text).apply { color = c(R.color.muted) }
    private val dashed = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = c(R.color.bad); strokeWidth = density
        pathEffect = android.graphics.DashPathEffect(floatArrayOf(4 * density, 4 * density), 0f)
    }
    private val path = Path()

    fun add(values: FloatArray) {
        if (paused) return
        for (i in data.indices) data[i][head] = values[i]
        head = (head + 1) % CAPACITY
        if (count < CAPACITY) count++
        invalidate()
    }

    fun clear() { count = 0; head = 0; invalidate() }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        setMeasuredDimension(w, resolveSize((240 * density).toInt(), heightMeasureSpec))
    }

    override fun onDraw(canvas: Canvas) {
        val w = width.toFloat()
        val h = height.toFloat()
        canvas.drawRect(0f, 0f, w, h, bg)
        val top = (if (sharedScale) 20 else 34) * density
        val bottom = h - 6 * density
        val mid = (top + bottom) / 2
        val half = (bottom - top) / 2
        for (k in -2..2) {
            val y = mid + k * half / 2
            grid.alpha = if (k == 0) 255 else 90
            canvas.drawLine(0f, y, w, y, grid)
        }
        // 1 s ticks at 20 Hz
        grid.alpha = 60
        var s = 0
        while (s < CAPACITY) { val x = w - s * w / (CAPACITY - 1); canvas.drawLine(x, top, x, bottom, grid); s += 20 }

        var lx = 8 * density
        var ly = 14 * density
        var shared = 0f
        if (sharedScale) {
            series.forEachIndexed { i, ser ->
                if (!ser.enabled) return@forEachIndexed
                shared = max(shared, ser.minRange)
                for (k in 0 until count) shared = max(shared, abs(at(i, k)))
            }
            for (m in markers) if (abs(m) <= shared) {
                val y = mid - m / shared * half
                canvas.drawLine(0f, y, w, y, dashed)
            }
            val head = (title?.let { "$it " } ?: "") + "±${fmt(shared)}$unit"
            canvas.drawText(head, lx, ly, muted)
            lx += muted.measureText(head) + 12 * density
        }
        series.forEachIndexed { i, ser ->
            if (!ser.enabled) return@forEachIndexed
            val color = c(ser.colorRes)
            var range = ser.minRange
            if (sharedScale) range = shared
            else for (k in 0 until count) range = max(range, abs(at(i, k)))
            if (count > 1) {
                path.reset()
                for (k in 0 until count) {
                    // newest sample at the right edge
                    val x = w - (count - 1 - k) * w / (CAPACITY - 1)
                    val y = mid - at(i, k) / range * half
                    if (k == 0) path.moveTo(x, y) else path.lineTo(x, y)
                }
                line.color = color
                canvas.drawPath(path, line)
            }
            text.color = color
            val label = if (sharedScale) ser.name else "${ser.name} ±${fmt(range)}${ser.unit}"
            val tw = text.measureText(label) + 12 * density
            if (lx + tw > w) { lx = 8 * density; ly += 14 * density }
            canvas.drawText(label, lx, ly, text)
            lx += tw
        }
        if (count == 0) {
            val msg = "Waiting for telemetry"
            canvas.drawText(msg, (w - muted.measureText(msg)) / 2, mid - 6 * density, muted)
        }
        if (paused) canvas.drawText("PAUSED", w - muted.measureText("PAUSED") - 8 * density, bottom - 6 * density, muted)
    }

    /** k-th oldest stored sample of series i. */
    private fun at(i: Int, k: Int) = data[i][(head - count + k + CAPACITY) % CAPACITY]

    private fun fmt(v: Float) = when {
        v >= 100 -> "%.0f".format(v)
        v >= 10 -> "%.1f".format(v)
        else -> "%.2f".format(v)
    }

    companion object {
        const val CAPACITY = 300    // 15 s at 20 Hz
    }
}
