package com.zyc0049.balancetuner

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.content.Context
import android.os.Build
import java.io.IOException
import java.util.UUID
import java.util.concurrent.CountDownLatch
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.Semaphore
import java.util.concurrent.TimeUnit

/**
 * Bluetooth LE link to the ESP32-S3 firmware's Nordic UART Service: frames are written to RX
 * (without response) and arrive as notifications on TX. [open] connects, asks for a 185-byte
 * MTU so a telemetry frame fits in one notification, finds the service and turns notifications
 * on. Android allows one GATT operation at a time, so writes wait for the previous one's
 * callback. The caller must hold BLUETOOTH_CONNECT (Android 12+).
 */
@SuppressLint("MissingPermission")
class BleUartPort private constructor(override val label: String) : LinkPort {

    private var gatt: BluetoothGatt? = null
    private var rx: BluetoothGattCharacteristic? = null
    private val incoming = LinkedBlockingQueue<ByteArray>()
    private var chunk = ByteArray(0)                    // the notification being handed out
    private var chunkPos = 0
    @Volatile private var mtu = 23
    @Volatile private var gone: String? = null          // why the link ended
    @Volatile private var setupError: String? = null
    private val ready = CountDownLatch(1)
    private val writeDone = Semaphore(1)
    @Volatile private var lastRx = System.nanoTime()

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
                if (!g.requestMtu(WANT_MTU)) g.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                gone = "Bluetooth disconnected"
                fail("could not connect (status $status)")
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, value: Int, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) mtu = value
            g.discoverServices()
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            val service = g.getService(SERVICE) ?: return fail("not the robot (no Nordic UART service)")
            val tx = service.getCharacteristic(TX) ?: return fail("no TX characteristic")
            rx = service.getCharacteristic(RX) ?: return fail("no RX characteristic")
            val cccd = tx.getDescriptor(CCCD) ?: return fail("TX cannot notify")
            g.setCharacteristicNotification(tx, true)
            val value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            val ok = if (Build.VERSION.SDK_INT >= 33) g.writeDescriptor(cccd, value) == BluetoothStatusCodes.SUCCESS
            else @Suppress("DEPRECATION") run { cccd.value = value; g.writeDescriptor(cccd) }
            if (!ok) fail("could not turn notifications on")
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) ready.countDown() else fail("could not turn notifications on")
        }

        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            writeDone.release()
        }

        // Android 13+
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            received(value)
        }

        @Deprecated("Android 12 and older")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            @Suppress("DEPRECATION") received(c.value ?: return)
        }
    }

    private fun received(value: ByteArray) {
        if (value.isEmpty()) return
        lastRx = System.nanoTime()
        incoming.offer(value.copyOf())
    }

    private fun fail(why: String) {
        if (ready.count > 0) { setupError = why; ready.countDown() }
    }

    override fun read(buf: ByteArray, timeoutMs: Int): Int {
        if (chunkPos >= chunk.size) {
            gone?.let { throw IOException(it) }
            val next = incoming.poll(timeoutMs.toLong(), TimeUnit.MILLISECONDS)
            if (next == null) {
                if (System.nanoTime() - lastRx > SILENCE_NS) throw IOException("Bluetooth link lost")
                return -1
            }
            chunk = next
            chunkPos = 0
        }
        val n = minOf(buf.size, chunk.size - chunkPos)
        System.arraycopy(chunk, chunkPos, buf, 0, n)
        chunkPos += n
        return n
    }

    override fun write(data: ByteArray) {
        val g = gatt ?: throw IOException("Bluetooth closed")
        val c = rx ?: throw IOException("Bluetooth closed")
        val size = maxOf(20, mtu - 3)
        var i = 0
        while (i < data.size) {
            gone?.let { throw IOException(it) }
            if (!writeDone.tryAcquire(WRITE_TIMEOUT_MS, TimeUnit.MILLISECONDS)) throw IOException("Bluetooth write timed out")
            val part = data.copyOfRange(i, minOf(data.size, i + size))
            val ok = if (Build.VERSION.SDK_INT >= 33)
                g.writeCharacteristic(c, part, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE) == BluetoothStatusCodes.SUCCESS
            else @Suppress("DEPRECATION") run {
                c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
                c.value = part
                g.writeCharacteristic(c)
            }
            if (!ok) { writeDone.release(); throw IOException("Bluetooth write failed") }
            i += part.size
        }
    }

    override fun close() {
        gone = "closed"
        gatt?.let { runCatching { it.disconnect() }; runCatching { it.close() } }
        gatt = null
    }

    companion object {
        val SERVICE: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        private val RX: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        private val TX: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
        private val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        private const val WANT_MTU = 185                    // as BLEDevice::setMTU in the sketch
        private const val CONNECT_TIMEOUT_MS = 12_000L
        private const val WRITE_TIMEOUT_MS = 1000L
        private const val SILENCE_NS = 3_000_000_000L       // the robot sends 10+ frames/s

        /** Blocks until the link is ready. Throws IOException with a user-readable message. */
        fun open(context: Context, device: BluetoothDevice, name: String): BleUartPort {
            val port = BleUartPort("BLE $name")
            port.gatt = device.connectGatt(context, false, port.callback, BluetoothDevice.TRANSPORT_LE)
                ?: throw IOException("Bluetooth is not available")
            val done = port.ready.await(CONNECT_TIMEOUT_MS, TimeUnit.MILLISECONDS)
            val error = if (!done) "no answer from $name" else port.setupError
            if (error != null) { port.close(); throw IOException(error) }
            port.lastRx = System.nanoTime()
            return port
        }
    }
}
