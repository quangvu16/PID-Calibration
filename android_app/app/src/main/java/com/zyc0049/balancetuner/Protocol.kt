package com.zyc0049.balancetuner

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Wire format of BalancingRobotTuner.ino (see the comment at the top of that sketch).
 * chk = low byte of the sum of the bytes between the header and chk; values little-endian.
 */
object Protocol {
    const val HDR_CONTROL = 0xA5
    const val HDR_SET_PARAM = 0xA6
    const val HDR_COMMAND = 0xA7
    const val HDR_PARAM = 0xB6
    const val HDR_TELEMETRY = 0xB7
    const val HDR_ACK = 0xB8
    const val TAIL = 0x5A

    const val CMD_GET_PARAMS = 1
    const val CMD_SAVE = 2
    const val CMD_DEFAULTS = 3
    const val CMD_MOTORS = 4
    const val CMD_TELEM_RATE = 5
    const val CMD_LOAD = 6

    /** x right +, y forward +, both -127..127; mode 1..3 asks for a mode change, 0 keeps it. */
    fun control(x: Int, y: Int, horn: Boolean, ccw: Boolean, cw: Boolean, mode: Int): ByteArray {
        val b = ByteArray(11)
        b[0] = HDR_CONTROL.toByte()
        b[1] = x.coerceIn(-127, 127).toByte()
        b[2] = y.coerceIn(-127, 127).toByte()
        b[3] = if (horn) 1 else 0
        b[4] = if (ccw) 1 else 0
        b[5] = if (cw) 1 else 0
        if (mode in 1..3) b[5 + mode] = 1
        b[9] = sum(b, 1, 9)
        b[10] = TAIL.toByte()
        return b
    }

    fun setParam(id: Int, value: Float): ByteArray {
        val b = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN)
        b.put(HDR_SET_PARAM.toByte()).put(id.toByte()).putFloat(value).put(0).put(TAIL.toByte())
        val a = b.array()
        a[6] = sum(a, 1, 6)
        return a
    }

    fun command(cmd: Int, arg: Int = 0): ByteArray =
        byteArrayOf(HDR_COMMAND.toByte(), cmd.toByte(), arg.toByte(), (cmd + arg).toByte(), TAIL.toByte())

    private fun sum(b: ByteArray, from: Int, to: Int): Byte {
        var s = 0
        for (i in from until to) s += b[i].toInt() and 0xFF
        return s.toByte()
    }
}

/** One telemetry frame. Terms are in PWM units (-255..255 after the sum is clamped). */
data class Telemetry(
    val tMs: Int,           // robot millis(), low 16 bits
    val angle: Float,       // Kalman angle, degrees (fallen beyond +-30)
    val gyro: Float,        // gyro x, deg/s
    val angleTerm: Float,   // kp*(angle+angle0) + kd*gyro
    val speedTerm: Float,   // speed loop PI output
    val turnTerm: Float,    // turn loop PD output
    val pwmL: Float,
    val pwmR: Float,
    val countL: Int,        // wheel encoder pulses since the previous frame
    val countR: Int,
    val distanceCm: Int,
    val motorsOn: Boolean,
    val fallen: Boolean,
    val mode: Int,          // 1 remote, 2 avoid, 3 follow
)

sealed class Frame {
    class Telem(val t: Telemetry) : Frame()
    class Param(val id: Int, val value: Float) : Frame()
    class Ack(val cmd: Int, val ok: Boolean) : Frame()
    class Distance(val cm: Int) : Frame()
}

/**
 * Byte-stream frame splitter. Headers can also occur inside payloads, so a candidate
 * frame must pass its tail and checksum; otherwise one byte is skipped and the search
 * resumes there.
 */
class FrameParser {
    private val buf = ByteArray(512)
    private var len = 0
    var badFrames = 0
        private set

    private companion object {
        const val TELEMETRY_LEN = 24    // payload bytes of Send_Telemetry() in the sketch
    }

    fun feed(data: ByteArray, n: Int, out: (Frame) -> Unit) {
        for (i in 0 until n) {
            if (len == buf.size) drop(1)
            buf[len++] = data[i]
        }
        var start = 0
        while (start < len) {
            val need = frameLength(start)
            if (need < 0) { start++; continue }          // not a header
            if (need == 0 || start + need > len) break   // wait for more bytes
            val f = decode(start, need)
            if (f == null) { badFrames++; start++; continue }
            out(f)
            start += need
        }
        drop(start)
    }

    private fun u8(i: Int) = buf[i].toInt() and 0xFF

    /** -1 not a header, 0 length not yet known, else total frame length. */
    private fun frameLength(i: Int): Int = when (u8(i)) {
        Protocol.HDR_CONTROL -> 4
        Protocol.HDR_PARAM -> 8
        Protocol.HDR_ACK -> 5
        Protocol.HDR_TELEMETRY -> when {
            i + 1 >= len -> 0
            // exact length, so a stray 0xB7 can't stall the parser waiting for a long frame
            u8(i + 1) == TELEMETRY_LEN -> TELEMETRY_LEN + 4
            else -> -1
        }
        else -> -1
    }

    private fun decode(i: Int, n: Int): Frame? {
        if (u8(i + n - 1) != Protocol.TAIL) return null
        val hdr = u8(i)
        if (hdr == Protocol.HDR_CONTROL) {
            return if (u8(i + 1) == u8(i + 2)) Frame.Distance(u8(i + 1)) else null
        }
        val first = if (hdr == Protocol.HDR_TELEMETRY) i + 2 else i + 1
        var s = 0
        for (k in first until i + n - 2) s += u8(k)
        if ((s and 0xFF) != u8(i + n - 2)) return null
        val bb = ByteBuffer.wrap(buf, i, n).order(ByteOrder.LITTLE_ENDIAN)
        return when (hdr) {
            Protocol.HDR_PARAM -> Frame.Param(u8(i + 1), bb.getFloat(i + 2))
            Protocol.HDR_ACK -> Frame.Ack(u8(i + 1), u8(i + 2) != 0)
            else -> {
                val p = i + 2
                fun s16(k: Int) = bb.getShort(p + 2 * k).toInt()
                val flags = s16(11)
                Frame.Telem(Telemetry(
                    tMs = s16(0) and 0xFFFF,
                    angle = s16(1) / 100f,
                    gyro = s16(2) / 10f,
                    angleTerm = s16(3).toFloat(),
                    speedTerm = s16(4).toFloat(),
                    turnTerm = s16(5) / 10f,
                    pwmL = s16(6).toFloat(),
                    pwmR = s16(7).toFloat(),
                    countL = s16(8),
                    countR = s16(9),
                    distanceCm = s16(10),
                    motorsOn = flags and 1 != 0,
                    fallen = flags and 2 != 0,
                    mode = (flags shr 2) and 3,
                ))
            }
        }
    }

    private fun drop(n: Int) {
        if (n <= 0) return
        System.arraycopy(buf, n, buf, 0, len - n)
        len -= n
    }
}

/** A tunable value; [id] and order match PARAM_ID in the sketch. */
data class Param(
    val id: Int,
    val name: String,
    val hint: String,
    val step: Float,
    val decimals: Int,
    val default: Float,
)

val PARAMS = listOf(
    Param(0, "Angle Kp", "Stiffness against tilt. Too low: falls over; too high: fast buzzing oscillation.", 0.5f, 2, 23f),
    Param(1, "Angle Kd", "Damping from the gyro. Raise to calm the wobble; too high makes it jitter.", 0.02f, 3, 0.48f),
    Param(2, "Speed Kp", "Holds the wheels still. Too low: drifts away; too high: slow rocking.", 0.2f, 2, 5.52f),
    Param(3, "Speed Ki", "Pulls back to the start position. The kit uses about Speed Kp / 50.", 0.005f, 4, 0.1098f),
    Param(4, "Turn Kp", "Turning strength when steering.", 0.5f, 2, 10f),
    Param(5, "Turn Kd", "Resists unwanted spinning (gyro z).", 0.01f, 3, 0.09f),
    Param(6, "Balance angle", "Mechanical offset in degrees. Wrong value: the robot keeps creeping one way.", 0.05f, 2, 6.31f),
    Param(7, "Drive speed", "Forward/back command at full stick (kit used 30).", 2f, 0, 30f),
)
