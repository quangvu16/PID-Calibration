package com.zyc0049.balancetuner

import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbDeviceConnection
import android.hardware.usb.UsbEndpoint
import android.hardware.usb.UsbInterface
import android.hardware.usb.UsbManager
import java.io.IOException

/**
 * Just enough of a USB-serial driver for the chips found on Arduino Nanos (CH340 on this
 * kit's clone, CP210x, FT232R, CDC-ACM): opens the port at a fixed baud rate, 8N1, with
 * DTR/RTS low so opening it does not reset the board. Register sequences follow
 * usb-serial-for-android and the Linux drivers.
 */
class UsbSerialPort private constructor(
    private val conn: UsbDeviceConnection,
    private val ifaces: List<UsbInterface>,
    private val inEp: UsbEndpoint,
    private val outEp: UsbEndpoint,
    val chip: Chip,
) {
    enum class Chip(val label: String) { CH34X("CH340"), CP210X("CP210x"), FTDI("FTDI"), CDC("CDC-ACM") }

    private val raw = ByteArray(maxOf(inEp.maxPacketSize, 64) * 4)

    /** Bytes read into [buf], 0 if nothing arrived, -1 on timeout or error (the API can't tell them apart). */
    fun read(buf: ByteArray, timeoutMs: Int): Int {
        if (chip != Chip.FTDI) return conn.bulkTransfer(inEp, buf, buf.size, timeoutMs)
        // FTDI prefixes every packet with two modem status bytes.
        val n = conn.bulkTransfer(inEp, raw, minOf(raw.size, buf.size), timeoutMs)
        if (n < 0) return n
        val packet = inEp.maxPacketSize
        var out = 0
        var i = 0
        while (i < n) {
            val len = minOf(packet, n - i) - 2
            if (len > 0) { System.arraycopy(raw, i + 2, buf, out, len); out += len }
            i += packet
        }
        return out
    }

    fun write(data: ByteArray) {
        if (conn.bulkTransfer(outEp, data, data.size, WRITE_TIMEOUT_MS) != data.size) throw IOException("write failed")
    }

    fun close() {
        ifaces.forEach { runCatching { conn.releaseInterface(it) } }
        conn.close()
    }

    private fun init(baud: Int) = when (chip) {
        Chip.CH34X -> {
            vendorIn(0x5f, 0, 0)
            vendorOut(0xa1, 0, 0)
            ch34xBaud(baud)
            vendorIn(0x95, 0x2518, 0)
            vendorOut(0x9a, 0x2518, 0xc3)               // LCR: enable rx/tx, 8 bits, no parity, 1 stop
            vendorIn(0x95, 0x0706, 0)
            vendorOut(0xa1, 0x501f, 0xd90a)
            ch34xBaud(baud)
            vendorOut(0xa4, 0xffff, 0)                  // modem control: DTR and RTS off (active low)
            vendorIn(0x95, 0x0706, 0)
        }
        Chip.CP210X -> {
            val i = ifaces[0].id
            control(0x41, 0x00, 0x0001, i)              // IFC_ENABLE
            control(0x41, 0x1e, 0, i, byteArrayOf(baud.toByte(), (baud shr 8).toByte(), (baud shr 16).toByte(), (baud shr 24).toByte()))
            control(0x41, 0x03, 0x0800, i)              // SET_LINE_CTL 8N1
            control(0x41, 0x07, 0x0300, i)              // SET_MHS: DTR and RTS off
        }
        Chip.FTDI -> {
            vendorOut(0x00, 0, 1)                       // reset
            vendorOut(0x01, 0x0300, 1)                  // modem control: DTR and RTS off
            vendorOut(0x02, 0, 1)                       // no flow control
            vendorOut(0x04, 0x0008, 1)                  // 8N1
            // FT232R / FT-X: 3 MHz base clock, divisor in eighths with a scrambled fraction code.
            val div8 = ((3_000_000L * 8 + baud / 2) / baud).toInt()
            val code = intArrayOf(0, 3, 2, 4, 1, 5, 6, 7)[div8 and 7]
            vendorOut(0x03, (div8 shr 3) or ((code and 3) shl 14), code shr 2)
            vendorOut(0x09, 2, 1)                       // 2 ms latency timer instead of 16
        }
        Chip.CDC -> {
            val comm = ifaces.first().id
            control(0x21, 0x20, 0, comm, byteArrayOf(baud.toByte(), (baud shr 8).toByte(), (baud shr 16).toByte(), (baud shr 24).toByte(), 0, 0, 8))
            control(0x21, 0x22, 0, comm)                // SET_CONTROL_LINE_STATE: DTR and RTS off
        }
    }

    private fun ch34xBaud(baud: Int) {
        var factor = 1_532_620_800L / baud
        var divisor = 3
        while (factor > 0xfff0 && divisor > 0) { factor = factor shr 3; divisor-- }
        factor = 0x10000 - factor
        vendorOut(0x9a, 0x1312, (factor and 0xff00).toInt() or divisor or 0x80)
        vendorOut(0x9a, 0x0f2c, (factor and 0xff).toInt())
    }

    private fun vendorOut(request: Int, value: Int, index: Int) = control(0x40, request, value, index)

    private fun vendorIn(request: Int, value: Int, index: Int) {
        if (conn.controlTransfer(0xc0, request, value, index, ByteArray(2), 2, CONTROL_TIMEOUT_MS) < 0)
            throw IOException("${chip.label} did not answer")
    }

    private fun control(type: Int, request: Int, value: Int, index: Int, data: ByteArray? = null) {
        if (conn.controlTransfer(type, request, value, index, data, data?.size ?: 0, CONTROL_TIMEOUT_MS) < 0)
            throw IOException("${chip.label} setup failed")
    }

    companion object {
        private const val CONTROL_TIMEOUT_MS = 500
        private const val WRITE_TIMEOUT_MS = 500

        /** The driver for [d], or null if it is not a serial adapter we know. */
        fun chipOf(d: UsbDevice): Chip? = when {
            d.vendorId == 0x1a86 && d.productId in setOf(0x7523, 0x5523, 0x7522) -> Chip.CH34X
            d.vendorId == 0x10c4 && d.productId in setOf(0xea60, 0xea70, 0xea71) -> Chip.CP210X
            d.vendorId == 0x0403 && d.productId in setOf(0x6001, 0x6015) -> Chip.FTDI
            (0 until d.interfaceCount).any { d.getInterface(it).interfaceClass == UsbConstants.USB_CLASS_CDC_DATA } -> Chip.CDC
            else -> null
        }

        /** Needs USB permission for [d]. Throws IOException with a user-readable message. */
        fun open(usb: UsbManager, d: UsbDevice, baud: Int): UsbSerialPort {
            val chip = chipOf(d) ?: throw IOException("not a supported USB-serial adapter")
            val all = (0 until d.interfaceCount).map { d.getInterface(it) }
            // CDC: the control interface first (it takes the line-coding requests), then data.
            val ifaces = if (chip == Chip.CDC)
                listOfNotNull(all.firstOrNull { it.interfaceClass == UsbConstants.USB_CLASS_COMM },
                    all.first { it.interfaceClass == UsbConstants.USB_CLASS_CDC_DATA })
            else listOf(all.first())
            val data = ifaces.last()
            val eps = (0 until data.endpointCount).map { data.getEndpoint(it) }
                .filter { it.type == UsbConstants.USB_ENDPOINT_XFER_BULK }
            val inEp = eps.firstOrNull { it.direction == UsbConstants.USB_DIR_IN }
            val outEp = eps.firstOrNull { it.direction == UsbConstants.USB_DIR_OUT }
            if (inEp == null || outEp == null) throw IOException("${chip.label}: no bulk endpoints")

            val conn = usb.openDevice(d) ?: throw IOException("could not open the USB device")
            for (i in ifaces) {
                if (!conn.claimInterface(i, true)) { conn.close(); throw IOException("USB interface is busy") }
            }
            val port = UsbSerialPort(conn, ifaces, inEp, outEp, chip)
            try {
                port.init(baud)
            } catch (e: IOException) {
                port.close()
                throw e
            }
            return port
        }
    }
}
