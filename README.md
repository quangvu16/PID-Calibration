# BalanceTuner

Live PID tuning and remote control for the ZYC0049 (PBOT 3.0) balancing robot over a USB
cable from the phone (USB OTG) to the Nano. The HC-06 Bluetooth module is not used: unplug
it, it shares D0/D1 with the Nano's USB chip and would talk over it.

- `BalancingRobotTuner/` – firmware: the kit's `BalancingRobot` sketch plus telemetry,
  runtime PID parameters saved in EEPROM, and a motor on/off switch. The wire protocol is
  documented at the top of the `.ino`. The kit's 11-byte control packet still works, so the
  kit's btcontroller app could drive it at 9600 baud (the firmware now runs at 115200).
- `android_app/` – the Balance Tuner app (Kotlin, no libraries; its own small USB-serial
  driver for CH340, CP210x, FT232R and CDC-ACM in `UsbSerialPort.kt`). APK: `~/BalanceTuner.apk`.
- `serial_monitor.py` – decodes the same stream on a PC (`/dev/ttyUSB0`).

## Flash the firmware

This Nano's auto-reset does not work, so tap RESET while the upload is trying to connect.

```bash
cd "~/Arduino/ZYC0049 Self Balance/BalanceTuner/BalancingRobotTuner"
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 .
arduino-cli upload  --fqbn arduino:avr:nano:cpu=atmega328 -p /dev/ttyUSB0 .
```

## Build the app

```bash
cd "~/Arduino/ZYC0049 Self Balance/BalanceTuner/android_app"
JAVA_HOME=~/Android/jdk ANDROID_HOME=~/Android/Sdk \
  JAVA_TOOL_OPTIONS=-Djava.net.preferIPv6Addresses=true ./gradlew --no-daemon assembleRelease
cp app/build/outputs/apk/release/app-release.apk ~/BalanceTuner.apk
```

## Use

Connect the Nano's USB port to the phone with an OTG adapter/cable. Android offers to open
Balance Tuner and connects straight away; otherwise open the app, tap Connect and allow USB
access. The phone powers the Nano over the cable; the motors still run from the battery.
**Drive** has the joystick, spin and horn buttons, and the Remote/Avoid/Follow modes.
**Tune PID** has live charts: by default one panel per term, as in `tuner_gui.py` (tilt and
raw angle, gyro, angle/speed/turn loop terms on one shared scale, PWM L/R with ±250
saturation lines, wheel speed L/R); Overlay switches to the single chart with every series
scaled on its own and per-series checkboxes. Below them,
statistics for the last 5 s with hints, editors for each parameter (applied immediately),
Save to robot (EEPROM), and Record CSV. The app's "How to tune" section gives the procedure.

## Debug from a PC

With the Nano on the PC's USB, the Arduino Serial Monitor/Plotter at 115200 (or
`python3 serial_monitor.py --text`) shows readable lines, the default at power-on (`d` brings
them back after binary),
`tilt:0.12 angle:0.29 gyro:-1.3 A:4 S:-12 T:0.0 pwmL:-8 pwmR:-8 pulL:0 pulR:0 dist:42 motors:1 fallen:0 mode:1`,
after a line with the current parameters. `b`, or the app connecting, switches back to
binary. Without `--text`, `serial_monitor.py` decodes the binary frames.

## Tune with live graphs from a PC

`python3 tuner_gui.py [/dev/ttyUSB0]` (Tkinter + matplotlib + pyserial) is the PC version of
the app's Tune PID screen: scrolling charts of tilt/raw angle, gyro, the angle/speed/turn loop
terms, PWM (±250 saturation marked) and wheel speed, each loop term on its own trace as in
farrellf's Balancing_Robot_Firmware. Beside them: editors for every parameter (Enter/Set
applies at once, -/+ = ±10 %, angle0 ±0.1°), Get/Save/Load/Defaults, motors on/off, telemetry
rate, a Nudge (forward then back) to watch the recovery, window statistics, "angle0 from
mean" trim, and Record CSV (telemetry plus the parameters in force). Close the Arduino
Serial Monitor first; closing the window switches the robot back to text mode.

## Auto-tune from a PC

`python3 autotune.py` tunes the parameters on its own while the robot balances on the floor
(~1 m free, USB cable slack, Serial Monitor closed). It trims `angle0` from the mean angle
the robot settles at, then searches angle Kp/Kd and speed Kp/Ki, scoring each 8 s trial from
the telemetry: angle spread, gyro RMS, wheel speed and travel from the encoders, PWM
saturation; a fall scores worst. Each trial nudges the robot forward and back so it must
recover (`--no-nudge` to stay in place). After a fall it restores the best parameters and
waits for you to stand the robot up. It asks before saving to EEPROM and logs every trial
to `autotune_log.csv`. `--stages angle`, `--trial`, `--max-trials`: see `--help`.

### Auto-tune in the GUI: eleven methods

`tuner_gui.py` has an **Auto-tune** panel (`ml_tune.py`, numpy only) that runs the method
picked in its list on the ticked gains, with the same trials, cost, fall recovery, Stop, trial
log, Model window and save prompt for all of them. Twiddle and ML (Bayesian) are described
below; the others:

| Method | Family | Notes |
| --- | --- | --- |
| Neural (deep ensemble) | deep learning surrogate | 5 small neural nets (32-32, tanh, Adam, numpy) learn the cost; next trial = lowest mean − 1.5 × spread |
| RL (policy gradient) | reinforcement learning | PGPE: Gaussian policy over the gains, symmetric pairs of trials as episodes |
| CMA-ES | evolution strategy | adapts step size and gain correlations; ~6–8 trials per generation |
| Differential evolution | evolutionary | DE/rand/1/bin; give it 30+ trials |
| Particle swarm (PSO) | swarm | give it 30+ trials |
| Genetic algorithm | evolutionary | tournament, blend crossover, mutation, elitism; 30+ trials |
| Nelder-Mead | direct search | local simplex, no model; noise can fool it |
| SPSA | stochastic gradient | 2 trials per gradient estimate, extremum-seeking style; local |
| Random (baseline) | – | Latin-hypercube samples; what a method must beat to be worth it |

All but Twiddle report the tried set with the lowest cost *predicted* by the Model window's
Gaussian process, as ML does, so a single lucky trial does not win. On a simulated noisy cost
all of them moved Kp/Kd from the start to near the optimum in 25 trials; on the robot, ML
(Bayesian) still needs the fewest. Not included: Ziegler–Nichols, Cohen–Coon and relay
(Åström–Hägglund) tuning need the open-loop plant to be stable or a sustained oscillation,
and a balancing robot falls under both; online RL or fuzzy gain scheduling inside the 200 Hz
loop would need firmware changes. The app (`AutoTuner.kt`) still has only ML and Twiddle.

- **Twiddle** is `autotune.py`'s search run from the GUI. It steps one gain at a time
  (start ±25 %), and a win is re-measured before it is kept. Range defaults to ×3 as in
  `autotune.py`. Ticking all four gains tunes them together, not in stages. Here the Model
  window's map only pictures the trials; it does not steer the search.
- **ML (Bayesian)**, the default, is described below.

Besides the four gains, **angle0** (balance angle) can be ticked. It is an offset, not a gain,
so it is searched linearly within the current value ±3°, and Twiddle steps it in degrees
(±0.75° first). A wrong angle0 shows up in the cost as wheel drift and travel. The one-shot
"trim angle0 first" (angle0 = −mean angle) is still offered as a start. All trials go to
`ml_tune_log.csv`. `autotune.py` still works on its own from the command line.

**ML (Bayesian)** uses Bayesian optimisation: a Gaussian process learns the cost surface over the ticked gains (angle Kp/Kd by
default, speed Kp/Ki optional) from every trial. That includes falls, and also past trials
from `ml_tune_log.csv`/`autotune_log.csv` when the other gains match. Each next trial goes
where the expected improvement is highest. Trials and the cost are the same as
`autotune.py`. The search box runs from current ÷ Range to current × Range, on a log scale.
The result is the tried set with the lowest *predicted* cost, so one lucky trial does not
win. Expect 15–25 trials for two gains. The **Model** window plots the cost of every trial
and the model's predicted-cost map (falls in red, past runs in grey). Stop keeps the best
found so far. At the end the GUI asks before saving to EEPROM.

The app has the same feature in a native Kotlin port (`AutoTuner.kt`, `ModelView.kt`, no
libraries) under **Tune PID → Auto-tune**. It offers the same two methods (ML (Bayesian) or
Twiddle), the same settings and cost, and a chart of the cost per trial and the predicted-cost map. It switches
telemetry to 25 Hz while it runs, so its costs compare with the PC's, then back to 20 Hz.
Trials are logged to `ml_tune_log.csv` in Android/data/com.zyc0049.balancetuner/files. That
log is separate from the PC's, so each one learns only from its own past trials.

The link runs at 115200 baud (firmware `Serial.begin`, app `SerialLink.BAUD`, and
`serial_monitor.py -b` must agree). 20 Hz telemetry uses about 5% of it. The kit's
btcontroller app and an HC-06 at its default 9600 baud no longer work with this firmware.
