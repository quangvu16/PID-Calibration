package com.zyc0049.balancetuner

import java.io.IOException

/**
 * A byte-stream connection to the robot that [SerialLink] reads and writes: the USB cable
 * ([UsbSerialPort]), Bluetooth LE ([BleUartPort]) or a WebSocket over Wi-Fi ([WebSocketPort]).
 * The robot's frames are the same on all of them.
 */
interface LinkPort {
    /** Shown in the status pill, e.g. "USB CH340". */
    val label: String

    /** Bytes read into [buf]; 0 or -1 when nothing arrived within [timeoutMs]. */
    @Throws(IOException::class)
    fun read(buf: ByteArray, timeoutMs: Int): Int

    @Throws(IOException::class)
    fun write(data: ByteArray)

    fun close()
}
