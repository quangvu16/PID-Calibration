#!/usr/bin/env python3
"""Read and decode the BalancingRobot serial stream (USB, 115200 baud).

Understands both firmwares:
  kit sketch            A5 dist dist 5A                 ultrasonic cm
  BalancingRobotTuner   B6 id float32 chk 5A            parameter value
                        B7 len payload[len] chk 5A      telemetry
                        B8 cmd result chk 5A            command ack

  python3 serial_monitor.py                    # decoded frames from /dev/ttyUSB0
  python3 serial_monitor.py --csv log.csv      # also save telemetry to CSV
  python3 serial_monitor.py --raw              # plain hex dump, no decoding
  python3 serial_monitor.py --text             # robot's readable debug lines ('d'; 'b' on exit)

Without --text it sends 'b' first, so a robot left in text debug mode goes back to binary.
"""
import argparse
import struct
import sys
import time

import serial

TELEM_FIELDS = ["t_ms", "angle", "gyro_x", "angle_term", "speed_term", "turn_term",
                "pwm_l", "pwm_r", "pulses_l", "pulses_r", "dist_cm", "flags"]
TELEM_SCALE = {"angle": 100.0, "gyro_x": 10.0, "turn_term": 10.0}


def decode_telemetry(payload):
    v = dict(zip(TELEM_FIELDS, struct.unpack("<12h", payload[:24])))
    for k, s in TELEM_SCALE.items():
        v[k] /= s
    v["t_ms"] &= 0xFFFF
    return v


def parse(buf, on_frame):
    """Consume complete frames from buf (bytearray); return the unconsumed tail."""
    i = 0
    while i < len(buf):
        h = buf[i]
        if h == 0xA5:
            n = 4
            if len(buf) - i < n:
                break
            f = buf[i:i + n]
            if f[3] == 0x5A and f[1] == f[2]:
                on_frame("dist", {"dist_cm": f[1]})
                i += n
                continue
        elif h in (0xB6, 0xB8):
            n = 8 if h == 0xB6 else 5
            if len(buf) - i < n:
                break
            f = buf[i:i + n]
            body = f[1:n - 2]
            if f[-1] == 0x5A and sum(body) & 0xFF == f[-2]:
                if h == 0xB6:
                    on_frame("param", {"id": body[0], "value": struct.unpack("<f", body[1:5])[0]})
                else:
                    on_frame("ack", {"cmd": body[0], "ok": body[1] == 1})
                i += n
                continue
        elif h == 0xB7:
            if len(buf) - i < 2:
                break
            ln = buf[i + 1]
            n = ln + 4
            if len(buf) - i < n:
                break
            f = buf[i:i + n]
            payload = f[2:2 + ln]
            if ln >= 24 and f[-1] == 0x5A and sum(payload) & 0xFF == f[-2]:
                on_frame("telem", decode_telemetry(payload))
                i += n
                continue
        i += 1                                          # not a valid frame start: resync
    return buf[i:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("--raw", action="store_true", help="hex dump instead of decoding")
    ap.add_argument("--csv", help="write telemetry frames to this CSV file")
    ap.add_argument("--text", action="store_true", help="switch the robot to text debug lines and print them")
    args = ap.parse_args()

    # Opening the port toggles DTR, which resets the Nano: expect ~2 s of silence first.
    s = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"reading {args.port} @ {args.baud}, Ctrl+C to stop", file=sys.stderr)
    s.write(b"d" if args.text else b"b")

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        csv.write(",".join(TELEM_FIELDS) + "\n")

    def on_frame(kind, v):
        if kind == "telem":
            print("telem  angle={angle:7.2f}  gyro={gyro_x:7.1f}  A={angle_term:6d}  S={speed_term:6d}  "
                  "T={turn_term:6.1f}  pwm={pwm_l:5d}/{pwm_r:<5d}  pulses={pulses_l:4d}/{pulses_r:<4d}  "
                  "dist={dist_cm:3d}cm  flags={flags:#04x}".format(**v))
            if csv:
                csv.write(",".join(str(v[k]) for k in TELEM_FIELDS) + "\n")
        else:
            print(f"{kind:6s} {v}")

    buf = bytearray()
    try:
        while True:
            data = s.read(256)
            if not data:
                continue
            if args.raw:
                print(f"{time.time():.3f}  {data.hex(' ')}")
                continue
            if args.text:
                sys.stdout.write(data.decode("ascii", "replace"))
                sys.stdout.flush()
                continue
            buf = parse(buf + data, on_frame)
    except KeyboardInterrupt:
        pass
    finally:
        if args.text:
            s.write(b"b")
        if csv:
            csv.close()


if __name__ == "__main__":
    main()
