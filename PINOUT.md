# ZYC0049 pinout

*English · [Tiếng Việt](PINOUT.vi.md) · [Interactive version](https://claude.ai/artifact/8strAkTf6Lz7dgaNkb2ueQ) (private until shared)*

Phone app for this robot: [download BalanceTuner-v1.2.apk](releases/BalanceTuner-v1.2.apk).

![ZYC0049 pinout: Arduino Nano pins and what each one connects to](pinout.svg)

Every Arduino Nano (ATmega328P) pin and what the `BalancingRobotTuner` firmware connects to it.
Taken from the pin definitions, `Pin_Config()` and `setup()` in
`BalancingRobotTuner/BalancingRobotTuner.ino`; left/right for the motors comes from the
BalanceCar library's `pwma()`, which drives TB6612 channel A as the left motor and channel B as
the right.

Pins are listed from the USB end, as in the standard Nano top view. Check against the silkscreen
on your board before rewiring.

```
                         ┌──[ USB ]──┐
   AIN2 · left dir   D13 │●         ●│ D12   AIN1 · left dir
              3.3 V  3V3 │●         ●│ D11   buzzer
               AREF AREF │●         ●│ D10   PWMA · left speed
            LED red   A0 │●         ●│ D9    PWMB · right speed
          LED green   A1 │●         ●│ D8    STBY · motors on
           LED blue   A2 │●         ●│ D7    BIN1 · right dir
               free   A3 │●  NANO   ●│ D6    BIN2 · right dir
      SDA · MPU6050   A4 │●         ●│ D5    ECHO · sonar
      SCL · MPU6050   A5 │●         ●│ D4    encoder right
   free · analog in   A6 │●         ●│ D3    TRIG · sonar
   free · analog in   A7 │●         ●│ D2    encoder left
                5 V   5V │●         ●│ GND   ground
              reset  RST │●         ●│ RST   reset
             ground  GND │●         ●│ D0    RX · USB / HC-06
                VIN  VIN │●         ●│ D1    TX · USB / HC-06
                         └───────────┘
```

## By part

**Motor driver (TB6612).** Channel A drives the left motor, channel B the right. PWM sets the
speed, IN1/IN2 the direction: a positive PWM sets IN1 high and IN2 low, a negative one swaps them.

| Pin | Firmware name | Direction | Note |
| --- | --- | --- | --- |
| D6 | `TB6612_BIN2` | out | Right motor direction, input 2 |
| D7 | `TB6612_BIN1` | out | Right motor direction, input 1 |
| D8 | `TB6612_STBY` | out | LOW = standby (the app's Motors off) |
| D9 | `TB6612_PWMB` | PWM | Right motor speed, Timer1 |
| D10 | `TB6612_PWMA` | PWM | Left motor speed, Timer1 |
| D12 | `TB6612_AIN1` | out | Left motor direction, input 1 |
| D13 | `TB6612_AIN2` | out | Left motor direction, input 2 |

**Wheel encoders.** Pulses counted in interrupts and read every 5 ms as wheel speed.

| Pin | Firmware name | Direction | Note |
| --- | --- | --- | --- |
| D2 | `MOTOR1` | in | Left wheel; INT0 on every edge (CHANGE) |
| D4 | `MOTOR2` | in | Right wheel; pin-change interrupt PCINT20, pull-up on |

**IMU (MPU6050).** Accelerometer and gyro over I²C; a Kalman filter turns them into the tilt angle.

| Pin | Firmware name | Direction | Note |
| --- | --- | --- | --- |
| A4 | SDA | I²C | Data (Wire) |
| A5 | SCL | I²C | Clock (Wire) |

**Ultrasonic (HC-SR04).** Distance for the Avoid and Follow modes, measured every 20 ms.

| Pin | Firmware name | Direction | Note |
| --- | --- | --- | --- |
| D3 | `TRIG` | out | Trigger pulse; digital only |
| D5 | `ECHO` | in | Echo pulse, read with `pulseIn` |

**Buzzer, RGB LED and serial.**

| Pin | Firmware name | Direction | Note |
| --- | --- | --- | --- |
| D11 | `BUZZER` | out | The horn; digital only |
| A0 | `RLED` | out | LED red, active low |
| A1 | `GLED` | out | LED green, active low |
| A2 | `BLED` | out | LED blue, active low |
| D0 | RX | in | USB serial, 115200 baud; shared with the HC-06 socket |
| D1 | TX | out | USB serial, 115200 baud; shared with the HC-06 socket |

**Free:** A3 (digital or analog), A6 and A7 (analog input only on the Nano).

**Power and reset** (3V3, AREF, 5V, VIN, GND, RST) are not set by the firmware; check the board's
wiring for what feeds them.

## Things to know

- **Timer2 runs the balance loop.** `MsTimer2` fires every 5 ms, so `analogWrite` no longer
  works on D3 and D11. Both are used as plain digital pins, so nothing is lost.
- **D0/D1 are the USB serial port.** The HC-06 Bluetooth socket shares them, so unplug the HC-06
  when using the cable.
- **The encoders count edges, not direction.** The sign of each count comes from the PWM
  direction.
- **The RGB LED is active low**: LOW lights a colour, HIGH turns it off.
- **The MPU6050 is polled** with `getMotion6` in the 5 ms loop; no interrupt pin is used.
