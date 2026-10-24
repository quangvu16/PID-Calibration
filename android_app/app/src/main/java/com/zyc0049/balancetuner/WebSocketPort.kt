package com.zyc0049.balancetuner

import android.util.Base64
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.net.SocketTimeoutException
import java.security.MessageDigest
import java.security.SecureRandom
import javax.net.SocketFactory

/**
 * Just enough of a WebSocket client (RFC 6455) for the ESP32-S3 firmware's /ws endpoint: the
 * robot's frames travel as binary messages. No library: an HTTP upgrade on a plain socket,
 * masked binary frames out, frames in decoded from a byte buffer so a read timeout can never
 * cut one in half. The socket comes from [factory], so the caller can bind it to the Wi-Fi
 * network (the robot's own network has no internet, and Android would otherwise route
 * around it).
 */
class WebSocketPort private constructor(
    private val socket: Socket,
    private val input: InputStream,
    private val output: OutputStream,
    override val label: String,
) : LinkPort {

    private val raw = ByteArray(4096)
    private var pending = ByteArray(8192)              // received, not yet decoded
    private var pendingLen = 0
    private var payload = ByteArray(0)                 // decoded binary bytes not yet handed out
    private var payloadPos = 0
    private var lastRx = System.nanoTime()
    private val rng = SecureRandom()

    override fun read(buf: ByteArray, timeoutMs: Int): Int {
        if (payloadPos < payload.size) return take(buf)
        socket.soTimeout = timeoutMs
        val n = try {
            input.read(raw)
        } catch (_: SocketTimeoutException) {
            if (System.nanoTime() - lastRx > SILENCE_NS) throw IOException("Wi-Fi link lost")
            return -1
        }
        if (n < 0) throw IOException("the robot closed the connection")
        lastRx = System.nanoTime()
        append(raw, n)
        decode()
        return if (payloadPos < payload.size) take(buf) else 0
    }

    private fun take(buf: ByteArray): Int {
        val n = minOf(buf.size, payload.size - payloadPos)
        System.arraycopy(payload, payloadPos, buf, 0, n)
        payloadPos += n
        return n
    }

    private fun append(b: ByteArray, n: Int) {
        if (pendingLen + n > pending.size) pending = pending.copyOf(maxOf(pending.size * 2, pendingLen + n))
        System.arraycopy(b, 0, pending, pendingLen, n)
        pendingLen += n
    }

    /** Takes every complete frame out of [pending]: binary payloads to [payload], answers pings. */
    private fun decode() {
        var out = ByteArray(0)
        var pos = 0
        while (true) {
            if (pendingLen - pos < 2) break
            val b0 = pending[pos].toInt() and 0xFF
            val b1 = pending[pos + 1].toInt() and 0xFF
            val masked = b1 and 0x80 != 0
            var len = (b1 and 0x7F).toLong()
            var head = 2
            if (len == 126L) {
                if (pendingLen - pos < 4) break
                len = ((pending[pos + 2].toInt() and 0xFF) shl 8 or (pending[pos + 3].toInt() and 0xFF)).toLong()
                head = 4
            } else if (len == 127L) {
                if (pendingLen - pos < 10) break
                len = 0
                for (i in 2 until 10) len = len shl 8 or (pending[pos + i].toLong() and 0xFF)
                head = 10
            }
            if (len > MAX_MESSAGE) throw IOException("message too long")
            val maskAt = pos + head
            if (masked) head += 4
            if (pendingLen - pos < head + len) break
            val data = pending.copyOfRange(pos + head, pos + head + len.toInt())
            if (masked) for (i in data.indices) data[i] = (data[i].toInt() xor pending[maskAt + i % 4].toInt()).toByte()
            pos += head + len.toInt()
            when (b0 and 0x0F) {
                0x0, 0x2 -> out += data                          // binary, or its continuation
                0x8 -> throw IOException("the robot closed the connection")
                0x9 -> sendFrame(0xA, data)                      // ping -> pong
            }                                                    // text and pong: ignored
        }
        System.arraycopy(pending, pos, pending, 0, pendingLen - pos)
        pendingLen -= pos
        payload = if (payloadPos < payload.size) payload.copyOfRange(payloadPos, payload.size) + out else out
        payloadPos = 0
    }

    override fun write(data: ByteArray) = sendFrame(0x2, data)

    @Synchronized
    private fun sendFrame(opcode: Int, data: ByteArray) {
        val head = if (data.size < 126) 2 else 4
        val f = ByteArray(head + 4 + data.size)
        f[0] = (0x80 or opcode).toByte()                         // FIN + opcode
        if (head == 2) f[1] = (0x80 or data.size).toByte()      // client frames are masked
        else { f[1] = (0x80 or 126).toByte(); f[2] = (data.size shr 8).toByte(); f[3] = data.size.toByte() }
        val mask = ByteArray(4).also { rng.nextBytes(it) }
        System.arraycopy(mask, 0, f, head, 4)
        for (i in data.indices) f[head + 4 + i] = (data[i].toInt() xor mask[i % 4].toInt()).toByte()
        try {
            output.write(f)
            output.flush()
        } catch (e: IOException) {
            throw IOException("Wi-Fi write failed: ${e.message}")
        }
    }

    override fun close() {
        runCatching { sendFrame(0x8, ByteArray(0)) }
        runCatching { socket.close() }
    }

    companion object {
        const val DEFAULT_HOST = "192.168.4.1"                 // the robot's own network
        private const val PATH = "/ws"
        private const val CONNECT_TIMEOUT_MS = 4000
        private const val SILENCE_NS = 3_000_000_000L          // the robot sends 10+ frames/s
        private const val MAX_MESSAGE = 65536L

        /** [address] is "host" or "host:port". Throws IOException with a user-readable message. */
        fun open(factory: SocketFactory, address: String): WebSocketPort {
            val host = address.substringBefore(':').trim()
            val port = address.substringAfter(':', "80").trim().toIntOrNull() ?: 80
            val socket = factory.createSocket()
            try {
                socket.connect(InetSocketAddress(host, port), CONNECT_TIMEOUT_MS)
                socket.tcpNoDelay = true
                socket.soTimeout = CONNECT_TIMEOUT_MS
                val key = Base64.encodeToString(ByteArray(16).also { SecureRandom().nextBytes(it) }, Base64.NO_WRAP)
                val out = socket.getOutputStream()
                out.write(("GET $PATH HTTP/1.1\r\nHost: $host\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" +
                    "Sec-WebSocket-Key: $key\r\nSec-WebSocket-Version: 13\r\n\r\n").toByteArray())
                out.flush()
                val input = socket.getInputStream()
                val head = StringBuilder()
                while (!head.endsWith("\r\n\r\n")) {
                    val c = input.read()
                    if (c < 0 || head.length > 4096) throw IOException("no WebSocket at $host")
                    head.append(c.toChar())
                }
                val status = head.lineSequence().first()
                if (!status.contains(" 101")) throw IOException("$host answered \"$status\": not the robot's firmware?")
                val accept = Base64.encodeToString(MessageDigest.getInstance("SHA-1")
                    .digest((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").toByteArray()), Base64.NO_WRAP)
                if (!head.contains(accept)) throw IOException("bad WebSocket handshake from $host")
                return WebSocketPort(socket, input, out, "Wi-Fi $host")
            } catch (e: IOException) {
                runCatching { socket.close() }
                throw if (e is SocketTimeoutException) IOException("no answer from $host (on the robot's Wi-Fi?)") else e
            }
        }
    }
}
