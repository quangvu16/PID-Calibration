package com.zyc0049.balancetuner

import android.content.Context
import android.graphics.Canvas
import android.graphics.Paint
import android.util.AttributeSet
import android.view.HapticFeedbackConstants
import android.view.MotionEvent
import android.view.View
import kotlin.math.hypot
import kotlin.math.min

/**
 * Round drive stick. Reports the knob as x (right +) and y (forward +), both
 * -1..1 and clipped to the unit circle; (0, 0) inside the dead zone and on
 * release. Only the finger that touched first steers.
 */
class JoystickView @JvmOverloads constructor(context: Context, attrs: AttributeSet? = null) :
    View(context, attrs) {

    /** Called on every change, on the UI thread. */
    var onStick: ((x: Float, y: Float) -> Unit)? = null

    var stickX = 0f
        private set
    var stickY = 0f
        private set
    private var knobX = 0f      // drawn position, -1..1, before the dead zone
    private var knobY = 0f
    private var pointer = -1
    val held get() = pointer != -1

    private val density = resources.displayMetrics.density
    private val c = { id: Int -> context.getColor(id) }
    private val pad = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL; color = c(R.color.sunk) }
    private val rim = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = density; color = c(R.color.line)
    }
    private val knob = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL; color = c(R.color.accent)
        setShadowLayer(4 * density, 0f, 2 * density, 0x40000000)
    }
    private val label = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = c(R.color.muted); textSize = 11 * density; textAlign = Paint.Align.CENTER
        isFakeBoldText = true; letterSpacing = 0.06f
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val want = (220 * density).toInt()
        val w = resolveSize(want, widthMeasureSpec)
        val size = min(w, want)
        setMeasuredDimension(size, size)
    }

    override fun setEnabled(enabled: Boolean) {
        super.setEnabled(enabled)
        alpha = if (enabled) 1f else 0.45f
        if (!enabled) release()
    }

    /** Snap back to the centre and report (0, 0). */
    fun release() {
        pointer = -1
        knobX = 0f; knobY = 0f
        set(0f, 0f)
        invalidate()
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (!isEnabled) return false
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                // Keep the ScrollView from stealing a vertical drag.
                parent?.requestDisallowInterceptTouchEvent(true)
                pointer = e.getPointerId(0)
                performHapticFeedback(HapticFeedbackConstants.VIRTUAL_KEY)
                move(e.getX(0), e.getY(0))
            }
            MotionEvent.ACTION_MOVE -> {
                val i = e.findPointerIndex(pointer)
                if (i >= 0) move(e.getX(i), e.getY(i))
            }
            MotionEvent.ACTION_POINTER_UP ->
                if (e.getPointerId(e.actionIndex) == pointer) release()
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                parent?.requestDisallowInterceptTouchEvent(false)
                release()
                if (e.actionMasked == MotionEvent.ACTION_UP) performClick()
            }
        }
        return true
    }

    override fun performClick(): Boolean = super.performClick()

    private fun move(px: Float, py: Float) {
        val half = width / 2f
        var nx = (px - half) / half
        var ny = -(py - half) / half
        val m = hypot(nx, ny)
        if (m > 1f) { nx /= m; ny /= m }
        knobX = nx; knobY = ny
        if (hypot(nx, ny) < DEADZONE) set(0f, 0f) else set(nx, ny)
        invalidate()
    }

    private fun set(nx: Float, ny: Float) {
        if (nx == stickX && ny == stickY) return
        stickX = nx; stickY = ny
        onStick?.invoke(nx, ny)
    }

    override fun onDraw(canvas: Canvas) {
        val half = width / 2f
        val r = half - density
        canvas.drawCircle(half, half, r, pad)
        canvas.drawCircle(half, half, r, rim)
        val inset = r * 0.76f
        canvas.drawLine(half, half - inset, half, half + inset, rim)
        canvas.drawLine(half - inset, half, half + inset, half, rim)
        val t = label.textSize
        canvas.drawText("FWD", half, half - r + t * 1.6f, label)
        canvas.drawText("BACK", half, half + r - t * 0.9f, label)
        canvas.drawText("LEFT", half - r + t * 2.2f, half + t * 0.35f, label)
        canvas.drawText("RIGHT", half + r - t * 2.6f, half + t * 0.35f, label)
        val travel = r * 0.7f      // knob centre stays inside the pad
        canvas.drawCircle(half + knobX * travel, half - knobY * travel, r * 0.3f, knob)
    }

    companion object {
        const val DEADZONE = 0.12f
    }
}
