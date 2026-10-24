package com.zyc0049.balancetuner

import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.res.ColorStateList
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.text.InputType
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.view.WindowManager
import android.view.inputmethod.EditorInfo
import android.widget.Button
import android.widget.CheckBox
import android.widget.EditText
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.LinearLayout
import android.widget.Spinner
import android.widget.Switch
import android.widget.TextView
import android.widget.Toast
import java.io.File
import java.io.PrintWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt
import kotlin.math.sqrt

class MainActivity : Activity(), SerialLink.Listener {

    private val usb by lazy { getSystemService(UsbManager::class.java) }
    private val link by lazy { SerialLink(usb, this) }
    private val ui = Handler(Looper.getMainLooper())

    private lateinit var btnConnect: Button
    private lateinit var tvLink: TextView
    private lateinit var tvMode: TextView
    private lateinit var tvAngle: TextView
    private lateinit var tvDistance: TextView
    private lateinit var tvRate: TextView
    private lateinit var swMotors: Switch
    private lateinit var chart: ChartView
    private lateinit var panels: List<ChartView>     // per-term charts: tilt, gyro, loop terms, PWM, wheels
    private lateinit var tvStats: TextView
    private lateinit var tvAdvice: TextView
    private lateinit var btnRecord: Button
    private lateinit var btnPause: Button
    private lateinit var modeButtons: List<Button>
    private lateinit var tabButtons: List<Button>

    private var state = SerialLink.State.DISCONNECTED

    // Read by the SerialLink writer thread every 50 ms.
    @Volatile private var stickX = 0f
    @Volatile private var stickY = 0f
    @Volatile private var horn = false
    @Volatile private var spinCcw = false
    @Volatile private var spinCw = false
    @Volatile private var modeRequest = 0
    @Volatile private var modeFrames = 0

    // Parameter rows
    private val robotValue = arrayOfNulls<Float>(PARAMS.size)
    private val edits = arrayOfNulls<EditText>(PARAMS.size)
    private val robotLabels = arrayOfNulls<TextView>(PARAMS.size)

    // Telemetry history for the stats (5 s at 20 Hz)
    private val hist = ArrayDeque<Telemetry>()
    private val histSpeed = ArrayDeque<Float>()
    private var lastT: Telemetry? = null
    private var framesThisSecond = 0
    private var csv: PrintWriter? = null
    private var csvFile: File? = null
    private var updatingMotorsSwitch = false

    // ML auto-tune (AutoTuner); read by the SerialLink writer thread for the nudge
    @Volatile private var tuner: AutoTuner? = null
    private val autoGains = mutableListOf<Pair<Int, CheckBox>>()
    private lateinit var autoTrials: EditText
    private lateinit var autoTrialS: EditText
    private lateinit var autoNudge: EditText
    private lateinit var autoRange: EditText
    private lateinit var btnAutoTune: Button
    private lateinit var tvAutoTune: TextView
    private lateinit var modelView: ModelView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        btnConnect = findViewById(R.id.btnConnect)
        tvLink = findViewById(R.id.tvLink)
        tvMode = findViewById(R.id.tvMode)
        tvAngle = findViewById(R.id.tvAngle)
        tvDistance = findViewById(R.id.tvDistance)
        tvRate = findViewById(R.id.tvRate)
        swMotors = findViewById(R.id.swMotors)
        chart = findViewById(R.id.chart)
        panels = listOf(R.id.panelTilt, R.id.panelGyro, R.id.panelTerms, R.id.panelPwm, R.id.panelWheels)
            .map { findViewById(it) }
        tvStats = findViewById(R.id.tvStats)
        tvAdvice = findViewById(R.id.tvAdvice)
        btnRecord = findViewById(R.id.btnRecord)
        btnPause = findViewById(R.id.btnPause)

        btnConnect.setOnClickListener {
            if (state == SerialLink.State.DISCONNECTED) connectClicked() else disconnect("Disconnected")
        }
        swMotors.setOnCheckedChangeListener { _, on ->
            if (!updatingMotorsSwitch) link.send(Protocol.command(Protocol.CMD_MOTORS, if (on) 1 else 0))
        }

        // Tabs
        val paneDrive = findViewById<View>(R.id.paneDrive)
        val paneTune = findViewById<View>(R.id.paneTune)
        tabButtons = listOf(findViewById(R.id.btnTabDrive), findViewById(R.id.btnTabTune))
        fun showTab(i: Int) {
            paneDrive.visibility = if (i == 0) View.VISIBLE else View.GONE
            paneTune.visibility = if (i == 1) View.VISIBLE else View.GONE
            tabButtons.forEachIndexed { k, b -> highlight(b, k == i) }
            if (i == 1) findViewById<JoystickView>(R.id.joystick).release()
        }
        tabButtons.forEachIndexed { i, b -> b.setOnClickListener { showTab(i) } }
        showTab(0)

        setupDrive()
        setupChart()
        setupParams()
        setupAutoTune()
        link.controlFrame = ::buildControl
        ui.post(rateTick)
        setLinkUi(SerialLink.State.DISCONNECTED, "offline")
        val filter = IntentFilter(ACTION_USB_PERMISSION)
        if (Build.VERSION.SDK_INT >= 33) registerReceiver(usbPermission, filter, RECEIVER_NOT_EXPORTED)
        else registerReceiver(usbPermission, filter)
        handleUsbIntent(intent)
    }

    override fun onDestroy() {
        super.onDestroy()
        ui.removeCallbacksAndMessages(null)
        tuner?.stop()
        unregisterReceiver(usbPermission)
        link.disconnect()
        stopRecording()
    }

    // ---------------------------------------------------------------- USB serial

    private fun connectClicked() {
        val devices = usb.deviceList.values.filter { UsbSerialPort.chipOf(it) != null }
        when (devices.size) {
            0 -> toast("Robot not found: plug the Nano's USB cable into the phone (USB OTG adapter)")
            1 -> connect(devices[0])
            else -> AlertDialog.Builder(this)
                .setTitle("USB serial device")
                .setItems(devices.map { "${it.productName ?: "?"}  (${UsbSerialPort.chipOf(it)!!.label})" }.toTypedArray()) { _, i ->
                    connect(devices[i])
                }
                .show()
        }
    }

    private fun connect(device: UsbDevice) {
        if (usb.hasPermission(device)) { link.connect(device); return }
        val pi = PendingIntent.getBroadcast(this, 0, Intent(ACTION_USB_PERMISSION).setPackage(packageName),
            if (Build.VERSION.SDK_INT >= 31) PendingIntent.FLAG_MUTABLE else 0)
        usb.requestPermission(device, pi)
    }

    private val usbPermission = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            val device = intent.usbDevice() ?: return
            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) link.connect(device)
            else toast("USB permission is needed to talk to the robot")
        }
    }

    /** Started (or brought back) by plugging the robot in; Android grants permission with that. */
    private fun handleUsbIntent(intent: Intent?) {
        if (intent?.action != UsbManager.ACTION_USB_DEVICE_ATTACHED) return
        val device = intent.usbDevice() ?: return
        if (state == SerialLink.State.DISCONNECTED && UsbSerialPort.chipOf(device) != null) connect(device)
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        handleUsbIntent(intent)
    }

    private fun Intent.usbDevice(): UsbDevice? =
        if (Build.VERSION.SDK_INT >= 33) getParcelableExtra(UsbManager.EXTRA_DEVICE, UsbDevice::class.java)
        else @Suppress("DEPRECATION") getParcelableExtra(UsbManager.EXTRA_DEVICE)

    private fun disconnect(why: String) {
        tuner?.stop()
        link.disconnect()
        setLinkUi(SerialLink.State.DISCONNECTED, why)
    }

    override fun onLinkState(state: SerialLink.State, detail: String) {
        setLinkUi(state, detail)
        if (state == SerialLink.State.CONNECTED) {
            link.send(Protocol.command(Protocol.CMD_TELEM_RATE, 20))
            link.send(Protocol.command(Protocol.CMD_GET_PARAMS))
        } else if (state == SerialLink.State.DISCONNECTED && detail.isNotEmpty()) {
            tuner?.stop()
            toast(detail)
        }
    }

    private fun setLinkUi(s: SerialLink.State, detail: String) {
        state = s
        btnConnect.text = if (s == SerialLink.State.DISCONNECTED) "Connect" else "Disconnect"
        tvLink.text = when (s) {
            SerialLink.State.CONNECTED -> "● $detail"
            SerialLink.State.CONNECTING -> "connecting…"
            SerialLink.State.DISCONNECTED -> "offline"
        }
        pill(tvLink, when (s) {
            SerialLink.State.CONNECTED -> R.color.ok_soft to R.color.ok
            SerialLink.State.CONNECTING -> R.color.warn_soft to R.color.warn
            SerialLink.State.DISCONNECTED -> R.color.sunk to R.color.muted
        })
        if (s == SerialLink.State.CONNECTED) window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        else window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
    }

    override fun onFrame(frame: Frame) {
        when (frame) {
            is Frame.Telem -> onTelemetry(frame.t)
            is Frame.Param -> onParam(frame.id, frame.value)
            is Frame.Distance -> tvDistance.text = "dist ${frame.cm} cm"
            is Frame.Ack -> when (frame.cmd) {
                Protocol.CMD_SAVE -> toast(if (frame.ok) "Saved to the robot's EEPROM" else "Save failed")
                Protocol.CMD_LOAD -> toast(if (frame.ok) "Reloaded saved values" else "Nothing saved on the robot yet")
                Protocol.CMD_DEFAULTS -> toast("Kit defaults applied (not saved)")
            }
        }
    }

    // ---------------------------------------------------------------- Drive

    @SuppressLint("ClickableViewAccessibility")
    private fun setupDrive() {
        findViewById<JoystickView>(R.id.joystick).onStick = { x, y -> stickX = x; stickY = y }
        fun hold(id: Int, set: (Boolean) -> Unit) {
            findViewById<View>(id).setOnTouchListener { v, e ->
                when (e.actionMasked) {
                    MotionEvent.ACTION_DOWN -> { set(true); v.isPressed = true }
                    MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> { set(false); v.isPressed = false }
                }
                true
            }
        }
        hold(R.id.btnHorn) { horn = it }
        hold(R.id.btnSpinCcw) { spinCcw = it }
        hold(R.id.btnSpinCw) { spinCw = it }

        modeButtons = listOf(findViewById(R.id.btnModeRemote), findViewById(R.id.btnModeAvoid), findViewById(R.id.btnModeFollow))
        modeButtons.forEachIndexed { i, b ->
            b.setOnClickListener { modeRequest = i + 1; modeFrames = 3 }
        }
    }

    /** Runs on the SerialLink writer thread. */
    private fun buildControl(): ByteArray {
        val nudge = tuner?.nudgeY ?: 0
        if (nudge != 0) return Protocol.control(0, nudge, false, false, false, 1)   // remote mode, as autotune.py
        var mode = 0
        if (modeFrames > 0) { modeFrames--; mode = modeRequest }
        return Protocol.control(
            (stickX * 127).roundToInt(), (stickY * 127).roundToInt(),
            horn, spinCcw, spinCw, mode,
        )
    }

    // ---------------------------------------------------------------- Chart + stats

    private fun setupChart() {
        chart.series = listOf(
            ChartView.Series("Tilt", "°", R.color.s_angle, 1f, true),
            ChartView.Series("Gyro", "°/s", R.color.s_gyro, 20f, false),
            ChartView.Series("Angle term", "", R.color.s_aterm, 20f, true),
            ChartView.Series("Speed term", "", R.color.s_sterm, 20f, true),
            ChartView.Series("Turn term", "", R.color.s_turn, 5f, false),
            ChartView.Series("PWM L", "", R.color.s_pwml, 20f, true),
            ChartView.Series("PWM R", "", R.color.s_pwmr, 20f, false),
            ChartView.Series("Wheel speed", "/s", R.color.s_speed, 50f, false),
        )
        fun panel(i: Int, title: String, unit: String, vararg s: ChartView.Series) = panels[i].also {
            it.title = title; it.unit = unit; it.sharedScale = true; it.series = s.toList()
        }
        panel(0, "Tilt", "°", ChartView.Series("tilt", "", R.color.s_angle, 1f, true),
            ChartView.Series("raw angle", "", R.color.s_raw, 1f, true))
        panel(1, "Gyro", "°/s", ChartView.Series("gyro", "", R.color.s_gyro, 20f, true))
        panel(2, "Loop terms", "", ChartView.Series("angle", "", R.color.s_aterm, 20f, true),
            ChartView.Series("speed", "", R.color.s_sterm, 20f, true),
            ChartView.Series("turn", "", R.color.s_turn, 5f, true))
        panel(3, "PWM", "", ChartView.Series("L", "", R.color.s_pwml, 20f, true),
            ChartView.Series("R", "", R.color.s_pwmr, 20f, true)).markers = floatArrayOf(250f, -250f)
        panel(4, "Wheel speed", "/s", ChartView.Series("L", "", R.color.s_pwml, 50f, true),
            ChartView.Series("R", "", R.color.s_pwmr, 50f, true))
        val panelBox = findViewById<View>(R.id.panels)
        val seriesScroll = findViewById<View>(R.id.seriesScroll)
        findViewById<Button>(R.id.btnView).setOnClickListener { b ->
            val split = panelBox.visibility != View.VISIBLE
            panelBox.visibility = if (split) View.VISIBLE else View.GONE
            chart.visibility = if (split) View.GONE else View.VISIBLE
            seriesScroll.visibility = chart.visibility
            (b as Button).text = if (split) "Overlay" else "Split"
        }

        val row = findViewById<LinearLayout>(R.id.seriesRow)
        chart.series.forEach { s ->
            row.addView(CheckBox(this).apply {
                text = s.name
                isChecked = s.enabled
                textSize = 13f
                setTextColor(getColor(R.color.fg))
                buttonTintList = ColorStateList.valueOf(getColor(s.colorRes))
                setPadding(0, 0, dp(10), 0)
                setOnCheckedChangeListener { _, on -> s.enabled = on; chart.invalidate() }
            })
        }
        btnPause.setOnClickListener {
            val paused = !chart.paused
            (panels + chart).forEach { it.paused = paused; it.invalidate() }
            btnPause.text = if (paused) "Resume" else "Pause"
        }
        findViewById<Button>(R.id.btnClear).setOnClickListener {
            (panels + chart).forEach { it.clear() }; hist.clear(); histSpeed.clear()
        }
        btnRecord.setOnClickListener { if (csv == null) startRecording() else stopRecording() }
        findViewById<Button>(R.id.btnBalancePoint).setOnClickListener { suggestBalanceAngle() }
    }

    private fun onTelemetry(t: Telemetry) {
        tuner?.onTelemetry(t)
        framesThisSecond++
        val prev = lastT
        lastT = t
        var dtMs = if (prev == null) 50 else (t.tMs - prev.tMs + 65536) % 65536
        if (dtMs <= 0 || dtMs > 1000) dtMs = 50
        val wheelSpeed = (t.countL + t.countR) / 2f * 1000f / dtMs
        val angle0 = robotValue[6] ?: 0f
        val tilt = t.angle + angle0

        chart.add(floatArrayOf(tilt, t.gyro, t.angleTerm, t.speedTerm, t.turnTerm, t.pwmL, t.pwmR, wheelSpeed))
        val perS = 1000f / dtMs
        panels[0].add(floatArrayOf(tilt, t.angle))
        panels[1].add(floatArrayOf(t.gyro))
        panels[2].add(floatArrayOf(t.angleTerm, t.speedTerm, t.turnTerm))
        panels[3].add(floatArrayOf(t.pwmL, t.pwmR))
        panels[4].add(floatArrayOf(t.countL * perS, t.countR * perS))
        hist.addLast(t); histSpeed.addLast(wheelSpeed)
        while (hist.size > STATS_SAMPLES) { hist.removeFirst(); histSpeed.removeFirst() }

        tvAngle.text = "tilt %+.2f°".format(tilt)
        pill(tvAngle, if (t.fallen) R.color.bad_soft to R.color.bad else R.color.sunk to R.color.muted)
        tvDistance.text = "dist ${t.distanceCm} cm"
        tvMode.text = when (t.mode) { 1 -> "remote"; 2 -> "avoid"; 3 -> "follow"; else -> "idle" }
        modeButtons.forEachIndexed { i, b -> highlight(b, t.mode == i + 1) }
        if (swMotors.isChecked != t.motorsOn) {
            updatingMotorsSwitch = true
            swMotors.isChecked = t.motorsOn
            updatingMotorsSwitch = false
        }

        csv?.println(listOf(
            System.currentTimeMillis(), t.tMs, t.angle, tilt, t.gyro, t.angleTerm, t.speedTerm, t.turnTerm,
            t.pwmL, t.pwmR, t.countL, t.countR, wheelSpeed, t.distanceCm, if (t.motorsOn) 1 else 0,
            if (t.fallen) 1 else 0, t.mode, PARAMS.joinToString(";") { "${robotValue[it.id] ?: ""}" },
        ).joinToString(","))

        if (hist.size % 5 == 0) updateStats(angle0)
    }

    private fun updateStats(angle0: Float) {
        val n = hist.size
        if (n < 10) return
        val tilt = FloatArray(n) { hist[it].angle + angle0 }
        val mean = tilt.average().toFloat()
        val std = sqrt(tilt.map { (it - mean) * (it - mean) }.average()).toFloat()
        val pp = tilt.max() - tilt.min()
        // Oscillation: sign changes of the de-meaned tilt, ignoring a noise band.
        val band = maxOf(0.05f, 0.25f * std)
        var crossings = 0
        var side = 0
        for (v in tilt) {
            val d = v - mean
            val s = if (d > band) 1 else if (d < -band) -1 else 0
            if (s != 0 && side != 0 && s != side) crossings++
            if (s != 0) side = s
        }
        val seconds = n / 20f
        val freq = crossings / 2f / seconds
        val pwm = hist.map { (abs(it.pwmL) + abs(it.pwmR)) / 2 }
        val sat = hist.count { abs(it.pwmL) >= 250 || abs(it.pwmR) >= 250 } * 100f / n
        val speedMean = histSpeed.average().toFloat()
        val fallen = hist.last().fallen
        val motorsOn = hist.last().motorsOn

        tvStats.text = buildString {
            append("tilt  mean %+6.2f°   std %5.2f°\n".format(mean, std))
            append("      p-p  %6.2f°   osc %4.1f Hz\n".format(pp, freq))
            append("pwm   mean |%5.0f|    sat %4.1f %%\n".format(pwm.average(), sat))
            append("wheel mean %+6.0f pulses/s".format(speedMean))
        }
        tvAdvice.text = when {
            fallen -> "Fallen over (|angle| > 30°): the firmware cuts the motors until it is stood up."
            !motorsOn -> "Motors off: the plots show what the controller would output. Good moment to set the balance angle."
            sat > 5 -> "PWM is at its limit %.0f%% of the time: gains are too high or it is close to falling.".format(sat)
            freq > 4 && std > 0.3f -> "Fast oscillation (~%.1f Hz): lower Angle Kp or raise Angle Kd.".format(freq)
            freq in 0.2f..1.5f && pp > 3 -> "Slow rocking (~%.1f Hz): lower Speed Kp / Speed Ki.".format(freq)
            abs(speedMean) > 150 -> "Wheels keep turning one way: check the balance angle, or raise Speed Kp."
            else -> "Looks steady."
        }
    }

    private fun suggestBalanceAngle() {
        if (hist.size < 40) { toast("Need a few seconds of telemetry first"); return }
        if (hist.any { it.fallen }) { toast("The robot was fallen in the last 5 s; hold it upright and retry"); return }
        // angleTerm = kp*(angle + angle0) + ...: zero at the balance point means angle0 = -angle.
        val suggested = -hist.map { it.angle }.average().toFloat()
        AlertDialog.Builder(this)
            .setTitle("Set balance angle")
            .setMessage(
                "Mean raw angle over the last %.0f s is %+.2f°.\n\nSet Balance angle to %+.2f°? (now %s)\n\n".format(
                    hist.size / 20f, -suggested, suggested, robotValue[6]?.let { "%+.2f°".format(it) } ?: "unknown") +
                    "Only meaningful if the robot was held still at its natural balance point, or was balancing in place."
            )
            .setPositiveButton("Set") { _, _ -> sendParam(6, suggested) }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private val rateTick = object : Runnable {
        override fun run() {
            tvRate.text = if (state == SerialLink.State.CONNECTED)
                "$framesThisSecond Hz · ${link.badFrames} bad" else "– Hz"
            framesThisSecond = 0
            ui.postDelayed(this, 1000)
        }
    }

    private fun startRecording() {
        val dir = getExternalFilesDir(null) ?: filesDir
        val f = File(dir, "balance_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + ".csv")
        csv = PrintWriter(f.bufferedWriter()).also {
            it.println("phone_ms,robot_ms,angle,tilt,gyro,angle_term,speed_term,turn_term,pwm_l,pwm_r," +
                "count_l,count_r,wheel_speed,distance_cm,motors_on,fallen,mode," +
                PARAMS.joinToString(";") { p -> p.name })
        }
        csvFile = f
        btnRecord.text = "Stop recording"
        highlight(btnRecord, true)
    }

    private fun stopRecording() {
        val w = csv ?: return
        w.close()
        csv = null
        btnRecord.text = "Record CSV"
        highlight(btnRecord, false)
        toast("Saved ${csvFile?.absolutePath}")
    }

    // ---------------------------------------------------------------- Parameters

    private fun setupParams() {
        val list = findViewById<LinearLayout>(R.id.paramList)
        for (p in PARAMS) {
            val card = LinearLayout(this).apply {
                orientation = LinearLayout.VERTICAL
                setBackgroundResource(R.drawable.rounded)
                backgroundTintList = ColorStateList.valueOf(getColor(R.color.surface))
                setPadding(dp(12), dp(10), dp(12), dp(10))
                layoutParams = LinearLayout.LayoutParams(-1, -2).apply { bottomMargin = dp(8) }
            }
            val head = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
            head.addView(TextView(this).apply {
                text = p.name; textSize = 15f; setTextColor(getColor(R.color.fg))
                setTypeface(typeface, android.graphics.Typeface.BOLD)
                layoutParams = LinearLayout.LayoutParams(0, -2, 1f)
            })
            val robot = TextView(this).apply {
                text = "robot: –"; textSize = 12f; typeface = android.graphics.Typeface.MONOSPACE
                setTextColor(getColor(R.color.muted))
            }
            head.addView(robot)
            card.addView(head)
            card.addView(TextView(this).apply {
                text = p.hint; textSize = 12f; setTextColor(getColor(R.color.muted))
                setPadding(0, dp(2), 0, dp(6))
            })

            val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
            val edit = EditText(this).apply {
                setText(fmt(p, p.default))
                inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
                imeOptions = EditorInfo.IME_ACTION_SEND
                textSize = 16f; typeface = android.graphics.Typeface.MONOSPACE
                gravity = Gravity.CENTER
                layoutParams = LinearLayout.LayoutParams(0, -2, 1f)
                setOnEditorActionListener { v, _, _ ->
                    val value = parse(v.text.toString())
                    if (value == null) toast("Not a number") else sendParam(p.id, value)
                    true
                }
            }
            fun stepButton(label: String, sign: Int) = Button(this, null, 0, R.style.ToolButton).apply {
                text = label; textSize = 18f
                layoutParams = LinearLayout.LayoutParams(dp(52), dp(44)).apply { marginEnd = dp(4); marginStart = dp(4) }
                setOnClickListener {
                    val cur = parse(edit.text.toString()) ?: robotValue[p.id] ?: p.default
                    val next = cur + sign * p.step
                    edit.setText(fmt(p, next))
                    sendParam(p.id, next)
                }
            }
            row.addView(stepButton("−", -1))
            row.addView(edit)
            row.addView(stepButton("+", 1))
            card.addView(row)
            list.addView(card)
            edits[p.id] = edit
            robotLabels[p.id] = robot
        }

        findViewById<Button>(R.id.btnRead).setOnClickListener { cmd(Protocol.CMD_GET_PARAMS) }
        findViewById<Button>(R.id.btnSave).setOnClickListener {
            AlertDialog.Builder(this)
                .setMessage("Save the current values to the robot's EEPROM? They will be used after every reset.")
                .setPositiveButton("Save") { _, _ -> cmd(Protocol.CMD_SAVE) }
                .setNegativeButton("Cancel", null)
                .show()
        }
        findViewById<Button>(R.id.btnLoad).setOnClickListener { cmd(Protocol.CMD_LOAD) }
        findViewById<Button>(R.id.btnDefaults).setOnClickListener { cmd(Protocol.CMD_DEFAULTS) }
    }

    private fun cmd(c: Int) {
        if (state != SerialLink.State.CONNECTED) { toast("Not connected"); return }
        link.send(Protocol.command(c))
    }

    private fun sendParam(id: Int, value: Float) {
        if (state != SerialLink.State.CONNECTED) { toast("Not connected"); return }
        robotLabels[id]?.text = "robot: sending…"
        link.send(Protocol.setParam(id, value))
    }

    /** The robot echoes every applied value (clamped to its limits). */
    private fun onParam(id: Int, value: Float) {
        tuner?.onParam(id, value)
        val p = PARAMS.getOrNull(id) ?: return
        robotValue[id] = value
        robotLabels[id]?.text = "robot: ${fmt(p, value)}"
        val e = edits[id] ?: return
        val shown = parse(e.text.toString())
        if (!e.hasFocus() || shown == null || abs(shown - value) > 1e-4f) e.setText(fmt(p, value))
    }

    // ---------------------------------------------------------------- ML auto-tune

    private fun setupAutoTune() {
        btnAutoTune = findViewById(R.id.btnAutoTune)
        tvAutoTune = findViewById(R.id.tvAutoTune)
        modelView = findViewById(R.id.modelView)
        val gains = findViewById<LinearLayout>(R.id.autoGains)
        AutoTuner.TUNABLE.chunked(2).forEach { pair ->
            val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
            for (id in pair) {
                val cb = CheckBox(this).apply {
                    text = PARAMS[id].name; isChecked = id <= 1; textSize = 14f
                    setTextColor(getColor(R.color.fg))
                    layoutParams = LinearLayout.LayoutParams(0, -2, 1f)
                }
                row.addView(cb)
                autoGains += id to cb
            }
            gains.addView(row)
        }
        val settings = findViewById<LinearLayout>(R.id.autoSettings)
        fun field(row: LinearLayout, label: String, value: String) = EditText(this).apply {
            row.addView(TextView(this@MainActivity).apply {
                text = label; textSize = 13f; setTextColor(getColor(R.color.muted))
                layoutParams = LinearLayout.LayoutParams(0, -2, 1f)
            })
            setText(value)
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL
            textSize = 15f; typeface = android.graphics.Typeface.MONOSPACE; gravity = Gravity.CENTER
            layoutParams = LinearLayout.LayoutParams(dp(64), -2)
            row.addView(this)
        }
        fun row() = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL; settings.addView(this)
        }
        row().let { autoTrials = field(it, "Trials", "20"); autoTrialS = field(it, "  Trial s", "8") }
        row().let { autoNudge = field(it, "Nudge", "90"); autoRange = field(it, "  Range ×", "2") }
        val methodHint = findViewById<TextView>(R.id.tvAutoMethodHint)
        findViewById<Spinner>(R.id.spAutoMethod).apply {
            adapter = ArrayAdapter(this@MainActivity, android.R.layout.simple_spinner_item,
                AutoTuner.Method.entries.map { it.label }).also {
                it.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
            }
            onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
                override fun onItemSelected(parent: AdapterView<*>?, view: View?, pos: Int, id: Long) {
                    val m = AutoTuner.Method.entries[pos]
                    autoRange.setText("%.0f".format(m.defaultRange))
                    methodHint.text = m.hint
                }
                override fun onNothingSelected(parent: AdapterView<*>?) {}
            }
        }
        btnAutoTune.setOnClickListener { if (tuner?.running == true) stopAutoTune() else startAutoTune() }
    }

    private fun autoMethod() = AutoTuner.Method.entries[findViewById<Spinner>(R.id.spAutoMethod).selectedItemPosition]

    private fun autoLog() = File(getExternalFilesDir(null) ?: filesDir, "ml_tune_log.csv")

    private fun startAutoTune() {
        if (state != SerialLink.State.CONNECTED) { toast("Not connected"); return }
        val ids = autoGains.filter { it.second.isChecked }.map { it.first }
        if (ids.isEmpty()) { toast("Tick at least one gain to tune"); return }
        val trials = autoTrials.text.toString().toIntOrNull()
        val trialS = parse(autoTrialS.text.toString())
        val nudge = autoNudge.text.toString().toIntOrNull()
        val range = parse(autoRange.text.toString())
        if (trials == null || trialS == null || nudge == null || range == null) { toast("Trials / Trial s / Nudge / Range: not a number"); return }
        val cfg = AutoTuner.Config(
            autoMethod(), ids, trials.coerceIn(3, 100), trialS.toDouble().coerceIn(4.0, 30.0), nudge.coerceIn(0, 127),
            range.toDouble().coerceIn(1.1, 5.0), findViewById<CheckBox>(R.id.cbAutoTrim).isChecked,
            findViewById<CheckBox>(R.id.cbAutoLearn).isChecked,
        )
        AlertDialog.Builder(this)
            .setTitle("Auto-tune: ${cfg.method.label}")
            .setMessage("The robot will try gains on its own for about %.0f min.\n\nRobot on a flat floor with ~1 m free, USB cable slack. Stand it up again whenever it falls.".format(
                cfg.trials * (cfg.trialS + 3) / 60))
            .setPositiveButton("Start") { _, _ ->
                updatingMotorsSwitch = true; swMotors.isChecked = true; updatingMotorsSwitch = false
                link.send(Protocol.command(Protocol.CMD_MOTORS, 1))
                link.send(Protocol.command(Protocol.CMD_TELEM_RATE, 25))    // the cost is per frame at 25 Hz, as on the PC
                tuner = AutoTuner(cfg, link::send, autoLog(), autoListener).also { it.start() }
                btnAutoTune.text = "Stop auto-tune"
                highlight(btnAutoTune, true)
                tvAutoTune.text = "Starting (${cfg.method.label})"
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun stopAutoTune() {
        tuner?.stop()
        tvAutoTune.text = "Stopping after this trial…"
    }

    private val autoListener = object : AutoTuner.Listener {
        override fun onStatus(text: String) { tvAutoTune.text = text }

        override fun onTrial(r: AutoTuner.Report) {
            fun gains(v: FloatArray) = r.names.indices.joinToString(" ") { "${r.names[it]}=%.3g".format(v[it]) }
            tvAutoTune.text = "Trial ${r.n}/${r.total}: ${gains(r.tried)}\n  -> " +
                (if (r.fell) "FELL" else "cost %.1f".format(r.cost)) +
                "\nBest: ${gains(r.best)}" + (r.bestPred?.let { " (predicted %.1f)".format(it) } ?: "")
            modelView.visibility = View.VISIBLE
            modelView.report = r
        }

        override fun onDone(reason: String, start: FloatArray?, result: FloatArray?) {
            btnAutoTune.text = "Start auto-tune"
            highlight(btnAutoTune, false)
            if (state == SerialLink.State.CONNECTED) link.send(Protocol.command(Protocol.CMD_TELEM_RATE, 20))
            val changed = if (start == null || result == null) emptyList() else PARAMS.filter { abs(start[it.id] - result[it.id]) > 1e-6f }
                .map { "${it.name}: ${fmt(it, start[it.id])} → ${fmt(it, result[it.id])}" }
            tvAutoTune.text = "Auto-tune $reason.\n" + if (changed.isEmpty()) "No change." else changed.joinToString("\n")
            if (changed.isNotEmpty() && state == SerialLink.State.CONNECTED) {
                AlertDialog.Builder(this@MainActivity)
                    .setTitle("Auto-tune ${reason.substringBefore(':')}")
                    .setMessage("Applied:\n" + changed.joinToString("\n") + "\n\nSave them to the robot's EEPROM?")
                    .setPositiveButton("Save") { _, _ -> cmd(Protocol.CMD_SAVE) }
                    .setNegativeButton("Not now", null)
                    .show()
            }
        }
    }

    // ---------------------------------------------------------------- helpers

    private fun fmt(p: Param, v: Float) = "%.${p.decimals}f".format(Locale.US, v)

    private fun parse(s: String) = s.trim().replace(',', '.').toFloatOrNull()

    private fun highlight(b: Button, on: Boolean) {
        b.backgroundTintList = ColorStateList.valueOf(getColor(if (on) R.color.accent_soft else R.color.surface))
        b.setTextColor(getColor(if (on) R.color.accent else R.color.fg))
    }

    private fun pill(v: TextView, colors: Pair<Int, Int>) {
        v.backgroundTintList = ColorStateList.valueOf(getColor(colors.first))
        v.setTextColor(getColor(colors.second))
    }

    private fun dp(v: Int) = (v * resources.displayMetrics.density).roundToInt()

    private fun toast(s: String) = Toast.makeText(this, s, Toast.LENGTH_SHORT).show()

    companion object {
        private const val ACTION_USB_PERMISSION = "com.zyc0049.balancetuner.USB_PERMISSION"
        private const val STATS_SAMPLES = 100
    }
}
