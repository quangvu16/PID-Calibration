package com.zyc0049.balancetuner

import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Handler
import android.os.Looper
import java.io.IOException
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger

/**
 * The link to the robot over any [LinkPort]: the USB cable to the Nano or ESP32-S3 (115200
 * baud), or Bluetooth LE / a WebSocket to the ESP32-S3. One thread opens the port and reads,
 * one writes: queued frames go out as they arrive and a control frame (from [controlFrame])
 * every 50 ms, which also feeds the sketch's 1 s link watchdog. The reader owns the port and
 * closes it once both threads have stopped. A port's read throws when its link is gone.
 * Callbacks run on the main thread.
 */
class SerialLink(private val usb: UsbManager, private val listener: Listener) {

    interface Listener {
        fun onLinkState(state: State, detail: String)
        fun onFrame(frame: Frame)
    }

    enum class State { DISCONNECTED, CONNECTING, CONNECTED }

    /** Built on the writer thread every 50 ms. */
    @Volatile var controlFrame: () -> ByteArray = { Protocol.control(0, 0, false, false, false, 0) }

    @Volatile var bytesIn = 0L
        private set
    @Volatile var badFrames = 0
        private set

    private val main = Handler(Looper.getMainLooper())
    private val queue = LinkedBlockingQueue<ByteArray>()
    @Volatile private var port: LinkPort? = null
    private val session = AtomicInteger()

    /** The caller must already hold USB permission for [device]. */
    fun connect(device: UsbDevice) = connect("USB", device.productName ?: device.deviceName) {
        val p = UsbSerialPort.open(usb, device, BAUD)
        // bulkTransfer returns -1 for a timeout too; only a vanished device means the cable is out.
        object : LinkPort by p {
            override fun read(buf: ByteArray, timeoutMs: Int): Int {
                val n = p.read(buf, timeoutMs)
                if (n < 0 && device.deviceName !in usb.deviceList) throw IOException("USB cable unplugged")
                return n
            }
        }
    }

    /** [open] runs on the reader thread and may block; it throws IOException with a readable message. */
    fun connect(kind: String, name: String, open: () -> LinkPort) {
        disconnect()
        val my = session.incrementAndGet()
        post(my, State.CONNECTING, name)
        Thread({ run(my, kind, open) }, "link-read").start()
    }

    fun disconnect() {
        session.incrementAndGet()
        port = null
        queue.clear()
    }

    fun send(frame: ByteArray) {
        if (port != null) queue.offer(frame)
    }

    private fun run(my: Int, kind: String, open: () -> LinkPort) {
        val p = try {
            open()
        } catch (e: IOException) {
            fail(my, "Could not connect over $kind: ${e.message}")
            return
        } catch (e: RuntimeException) {                     // SecurityException without permission, etc.
            fail(my, "Could not connect over $kind: ${e.message}")
            return
        }
        if (my != session.get()) { p.close(); return }
        port = p
        queue.clear()
        post(my, State.CONNECTED, p.label)
        val writer = Thread({ writeLoop(my, p) }, "link-write").apply { start() }

        val parser = FrameParser()
        val buf = ByteArray(256)
        while (my == session.get()) {
            val n = try {
                p.read(buf, READ_TIMEOUT_MS)
            } catch (e: IOException) {
                fail(my, e.message ?: "Link lost")
                break
            }
            if (n <= 0) continue
            bytesIn += n
            parser.feed(buf, n) { f -> main.post { if (my == session.get()) listener.onFrame(f) } }
            badFrames = parser.badFrames
        }
        writer.join()
        if (port === p) port = null
        p.close()
    }

    private fun writeLoop(my: Int, p: LinkPort) {
        try {
            var nextControl = System.nanoTime()
            while (my == session.get()) {
                val wait = nextControl - System.nanoTime()
                val f = if (wait > 0) queue.poll(wait, TimeUnit.NANOSECONDS) else null
                if (f != null) {
                    p.write(f)
                } else {
                    p.write(controlFrame())
                    nextControl = System.nanoTime() + CONTROL_PERIOD_NS
                }
            }
        } catch (_: IOException) {
            fail(my, "Link lost")
        } catch (_: InterruptedException) {
        }
    }

    private fun post(my: Int, state: State, detail: String) {
        main.post { if (my == session.get()) listener.onLinkState(state, detail) }
    }

    /** Ends session [my] (if still current) and reports why, unless the user reconnected meanwhile. */
    private fun fail(my: Int, detail: String) {
        if (session.compareAndSet(my, my + 1))
            main.post { if (session.get() == my + 1) listener.onLinkState(State.DISCONNECTED, detail) }
    }

    companion object {
        const val BAUD = 115200          // must match Serial.begin() in the sketch
        private const val READ_TIMEOUT_MS = 100
        private const val CONTROL_PERIOD_NS = 50_000_000L
    }
}
