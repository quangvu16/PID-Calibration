# ESP32-S3 version: wiring, Wi-Fi and Bluetooth

*English · [Tiếng Việt](ESP32S3.vi.md) · [README](README.md) · [Nano pinout](PINOUT.md)*

`BalancingRobotTunerESP32S3/` runs the ZYC0049 robot from an **ESP32-S3-DevKitC-1 N16R8**
instead of the Arduino Nano. It balances the same way, with the same parameters, telemetry and
Remote/Avoid/Follow modes, and speaks the same protocol on three links:

| Link | How to connect | Who uses it |
| --- | --- | --- |
| **USB** | cable to the board's "USB" port | the Android app, `tuner_gui.py`, `autotune.py`, `serial_monitor.py` |
| **Wi-Fi** | the robot's network, then `http://192.168.4.1/` | the Android app (1.2+), any browser (control page), or your own WebSocket client |
| **Bluetooth LE** | device `BalanceTuner-XXXX`, Nordic UART Service | the Android app (1.2+), BLE serial apps, or your own BLE client |

The PC tools connect over USB only. In the Android app, tap **Connect** and pick the link (see
[the app guide](ANDROID_APP.md#connect)).

## Wiring

![ESP32-S3-DevKitC-1 pinout: which GPIO goes to which part of the robot](pinout_esp32s3.svg)

| Signal | GPIO | Nano pin | Note |
| --- | --- | --- | --- |
| TB6612 PWMA (left speed) | 4 | D10 | |
| TB6612 AIN1 / AIN2 (left dir) | 5 / 6 | D12 / D13 | |
| TB6612 PWMB (right speed) | 7 | D9 | |
| TB6612 BIN1 / BIN2 (right dir) | 15 / 16 | D7 / D6 | |
| TB6612 STBY | 17 | D8 | LOW = motors off |
| Encoder left / right | 1 / 2 | D2 / D4 | 3.3 V max: divide a 5 V encoder output |
| MPU6050 SDA / SCL | 8 / 9 | A4 / A5 | pull-ups must go to 3.3 V |
| HC-SR04 TRIG | 10 | D3 | a 3.3 V trigger works on most modules |
| HC-SR04 ECHO | 11 | D5 | 5 V output: divider, e.g. 1 kΩ in series, 2 kΩ to GND |
| Buzzer | 12 | D11 | |
| RGB LED R / G / B | 13 / 14 / 21 | A0 / A1 / A2 | active low |
| GND | GND | GND | a common ground with the robot is required |

**The ESP32-S3's pins are 3.3 V and not 5 V tolerant.** Check every signal that comes *from* the
robot (encoders, sonar echo, I2C pull-ups) before connecting it. The TB6612's inputs accept 3.3 V
logic. A common-anode RGB LED on 5 V may glow faintly when "off", because 3.3 V does not reach
its anode voltage. The chosen pins avoid the strapping pins (0, 3, 45, 46), native USB (19/20),
the flash and octal PSRAM (26–37) and the on-board RGB LED (38 or 48).

## Build and flash

```bash
cd "~/Arduino/ZYC0049 Self Balance/BalanceTuner/BalancingRobotTunerESP32S3"
FQBN=esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB
arduino-cli compile --fqbn $FQBN .
arduino-cli upload  --fqbn $FQBN -p /dev/ttyACM0 .
```

- With Wi-Fi and Bluetooth the firmware is about 1.4 MB, so it needs the **3 MB app** partition
  (`app3M_fat9M_16MB`); the default 1.2 MB partition is too small.
- `CDCOnBoot=cdc` puts the serial link on the **USB** port (`/dev/ttyACM0`). With
  `CDCOnBoot=default` it is on the **UART** port instead (`/dev/ttyUSB0`, 115200 baud).
- If an upload does not start: hold BOOT, tap RST, release BOOT.
- Libraries: ESPAsyncWebSrv and AsyncTCP (Wi-Fi), BalanceCar and KalmanFilter, all in
  `~/Arduino/libraries`; Bluetooth comes with the ESP32 core (2.0.10 here).

At power-on the serial port is in text mode, and its parameter line ends with the wireless
details, for example `# wireless: network BalanceTuner-3F2A (password balance123),
http://192.168.4.1/, BLE BalanceTuner-3F2A`. `XXXX` is the end of the board's MAC address.

## USB

Plug the phone (USB OTG) or PC into the **USB** port and use the app or the PC tools as with the
Nano. With app 1.2 or later Android offers to open the app when the board is plugged in; with
1.1, open the app and tap **Connect**.

## Wi-Fi: control page and WebSocket

**In the Android app:** join the robot's network, then **Connect** → **Wi-Fi (ESP32-S3)**, address
**192.168.4.1**.

**In a browser:**

1. Join the Wi-Fi network **BalanceTuner-XXXX**, password **balance123**.
2. Open **http://192.168.4.1/** (or `http://balancetuner.local/` where mDNS works).
3. The page connects by itself and shows ● Wi-Fi.

The page has:

- **Status**: tilt, distance, mode, PWM left/right and the telemetry rate, and a chart of the tilt
  over the last 10 seconds.
- **DRIVE**: Remote / Avoid / Follow, the **Motors** switch, a joystick, and hold-buttons for
  **⟲ Spin**, **Horn** and **Spin ⟳**.
- **PID PARAMETERS**: the eight parameters with the value the robot is using; type a value and
  tap **Set**. **Read**, **Save to robot**, **Reload saved** and **Kit defaults** work as in the
  app.

While it is open the page sends a control frame 20 times a second. If it closes or the Wi-Fi
drops, the robot stops driving after 1 second.

**Joining your own network instead:** set `WIFI_STA_SSID` and `WIFI_STA_PASSWORD` at the top of
the sketch and flash it again. The robot tries for 10 seconds, then falls back to its own network.
The address it got is in the serial parameter line. **Change `WIFI_AP_PASSWORD`** if other people
can reach the robot's network: anyone who joins it can drive the robot.

**Your own WebSocket client:** connect to `ws://<robot>/ws` and exchange **binary** messages
holding the protocol's frames, exactly as on USB (the frame formats are at the top of the
sketch). Send `A7 05 14 19 5A` to start telemetry at 20 Hz and `A7 01 00 01 5A` to read every
parameter; send a control frame at least once a second while driving. Up to 2 clients can be
connected at once; replies go to every connected client.

## Bluetooth LE (Nordic UART Service)

The ESP32-S3 has Bluetooth LE only (no classic Bluetooth or SPP), so the robot offers the common
BLE serial profile. **In the Android app:** **Connect** → **Bluetooth (ESP32-S3)**, then tap
**BalanceTuner-XXXX**. Other BLE apps:

| | UUID |
| --- | --- |
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX (write frames here) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX (notifications: telemetry, replies) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

1. Scan for **BalanceTuner-XXXX** with a BLE app (for example nRF Connect, or Serial Bluetooth
   Terminal in BLE mode) and connect.
2. Turn on notifications for TX.
3. Write frames to RX as **hex bytes**, for example `A7 05 14 19 5A` (telemetry at 20 Hz) or
   `A7 01 00 01 5A` (read the parameters).

The protocol is a byte stream, so a frame may arrive split across notifications; a client must
join them before parsing. The robot asks for a 185-byte MTU so that a telemetry frame fits in one
notification. One BLE client at a time; after it disconnects the robot advertises again.

## Safety

- All three links share the 1-second control timeout: when no control frame arrives from any
  link, the joystick, spin and horn go back to zero.
- Wi-Fi and Bluetooth stay on alongside USB; the robot obeys whichever link sends a control
  frame last.
- Saving to the robot writes the ESP32's flash. It is fine to save after tuning, but the flash
  wears out after many thousands of writes, so do not save in a loop.

## If something is wrong

- **No `BalanceTuner-XXXX` network**: check the serial parameter line; if it says `joined …`,
  the robot is on your network instead.
- **The page loads but stays offline**: reload it; only 2 WebSocket clients can connect at once.
- **The robot reboots every few seconds**: the brown-out detector is resetting it. Power the board
  from a USB source that can supply about 500 mA; Wi-Fi draws current peaks.
- **No distance (always 0)**: no echo within 25 ms; check the sonar wiring and its divider.
- **Upload fails**: hold BOOT, tap RST, release BOOT, then upload again.
