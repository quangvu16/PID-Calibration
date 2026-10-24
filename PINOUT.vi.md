# Sơ đồ chân ZYC0049

*[English](PINOUT.md) · Tiếng Việt · [Bản tương tác](https://claude.ai/artifact/8strAkTf6Lz7dgaNkb2ueQ) (riêng tư cho tới khi được chia sẻ)*

App điện thoại cho robot này: [tải BalanceTuner-v1.1.apk](releases/BalanceTuner-v1.1.apk).

![Sơ đồ chân ZYC0049: các chân của Arduino Nano và bộ phận nối vào từng chân (hình bằng tiếng Anh)](pinout.svg)

Mọi chân của Arduino Nano (ATmega328P) và bộ phận mà firmware `BalancingRobotTuner` nối vào
chân đó. Lấy từ phần khai báo chân, `Pin_Config()` và `setup()` trong
`BalancingRobotTuner/BalancingRobotTuner.ino`; trái/phải của động cơ lấy từ hàm `pwma()` của
thư viện BalanceCar, hàm này dùng kênh A của TB6612 cho động cơ trái và kênh B cho động cơ phải.

Các chân được liệt kê từ đầu cổng USB, theo hình nhìn từ trên xuống thường gặp của Nano. Hãy
đối chiếu với chữ in trên board của bạn trước khi đấu lại dây.

```
                          ┌──[ USB ]──┐
   AIN2 · hướng trái  D13 │●         ●│ D12   AIN1 · hướng trái
              3,3 V   3V3 │●         ●│ D11   còi
               AREF  AREF │●         ●│ D10   PWMA · tốc độ trái
              LED đỏ   A0 │●         ●│ D9    PWMB · tốc độ phải
        LED xanh lá    A1 │●         ●│ D8    STBY · bật động cơ
     LED xanh dương    A2 │●         ●│ D7    BIN1 · hướng phải
               trống   A3 │●  NANO   ●│ D6    BIN2 · hướng phải
       SDA · MPU6050   A4 │●         ●│ D5    ECHO · siêu âm
       SCL · MPU6050   A5 │●         ●│ D4    encoder phải
    trống · analog in  A6 │●         ●│ D3    TRIG · siêu âm
    trống · analog in  A7 │●         ●│ D2    encoder trái
                 5 V   5V │●         ●│ GND   mass
               reset  RST │●         ●│ RST   reset
                mass  GND │●         ●│ D0    RX · USB / HC-06
                 VIN  VIN │●         ●│ D1    TX · USB / HC-06
                          └───────────┘
```

## Theo bộ phận

**Driver động cơ (TB6612).** Kênh A chạy động cơ trái, kênh B chạy động cơ phải. PWM đặt tốc
độ, IN1/IN2 đặt hướng: PWM dương đặt IN1 mức cao và IN2 mức thấp, PWM âm thì đảo lại.

| Chân | Tên trong firmware | Chiều | Ghi chú |
| --- | --- | --- | --- |
| D6 | `TB6612_BIN2` | ra | Hướng động cơ phải, ngõ vào 2 |
| D7 | `TB6612_BIN1` | ra | Hướng động cơ phải, ngõ vào 1 |
| D8 | `TB6612_STBY` | ra | LOW = chờ (nút Motors off trong app) |
| D9 | `TB6612_PWMB` | PWM | Tốc độ động cơ phải, Timer1 |
| D10 | `TB6612_PWMA` | PWM | Tốc độ động cơ trái, Timer1 |
| D12 | `TB6612_AIN1` | ra | Hướng động cơ trái, ngõ vào 1 |
| D13 | `TB6612_AIN2` | ra | Hướng động cơ trái, ngõ vào 2 |

**Encoder bánh xe.** Xung được đếm trong ngắt và đọc mỗi 5 ms thành tốc độ bánh.

| Chân | Tên trong firmware | Chiều | Ghi chú |
| --- | --- | --- | --- |
| D2 | `MOTOR1` | vào | Bánh trái; ngắt ngoài INT0 ở mọi cạnh (CHANGE) |
| D4 | `MOTOR2` | vào | Bánh phải; ngắt đổi chân PCINT20, bật điện trở kéo lên |

**IMU (MPU6050).** Gia tốc kế và con quay qua I²C; bộ lọc Kalman tính ra góc nghiêng.

| Chân | Tên trong firmware | Chiều | Ghi chú |
| --- | --- | --- | --- |
| A4 | SDA | I²C | Dữ liệu (Wire) |
| A5 | SCL | I²C | Xung nhịp (Wire) |

**Cảm biến siêu âm (HC-SR04).** Đo khoảng cách cho chế độ Avoid (tránh vật cản) và Follow (đi
theo), mỗi 20 ms.

| Chân | Tên trong firmware | Chiều | Ghi chú |
| --- | --- | --- | --- |
| D3 | `TRIG` | ra | Xung kích; chỉ dùng digital |
| D5 | `ECHO` | vào | Xung phản hồi, đọc bằng `pulseIn` |

**Còi, LED RGB và cổng serial.**

| Chân | Tên trong firmware | Chiều | Ghi chú |
| --- | --- | --- | --- |
| D11 | `BUZZER` | ra | Còi; chỉ dùng digital |
| A0 | `RLED` | ra | LED đỏ, tích cực mức thấp |
| A1 | `GLED` | ra | LED xanh lá, tích cực mức thấp |
| A2 | `BLED` | ra | LED xanh dương, tích cực mức thấp |
| D0 | RX | vào | Serial USB, 115200 baud; dùng chung với chân cắm HC-06 |
| D1 | TX | ra | Serial USB, 115200 baud; dùng chung với chân cắm HC-06 |

**Còn trống:** A3 (digital hoặc analog), A6 và A7 (trên Nano chỉ là ngõ vào analog).

**Nguồn và reset** (3V3, AREF, 5V, VIN, GND, RST) không do firmware điều khiển; hãy xem cách đấu
dây trên board để biết chúng được cấp từ đâu.

## Cần biết

- **Timer2 chạy vòng cân bằng.** `MsTimer2` kích mỗi 5 ms, nên `analogWrite` không còn dùng được
  trên D3 và D11. Cả hai chỉ dùng làm chân digital thường, nên không mất gì.
- **D0/D1 là cổng serial USB.** Chân cắm Bluetooth HC-06 dùng chung hai chân này, nên hãy rút
  HC-06 ra khi dùng cáp.
- **Encoder chỉ đếm cạnh xung, không biết chiều quay.** Dấu của mỗi lần đếm lấy theo chiều PWM.
- **LED RGB tích cực mức thấp**: LOW làm sáng một màu, HIGH tắt nó.
- **MPU6050 được đọc định kỳ** bằng `getMotion6` trong vòng 5 ms; không dùng chân ngắt.
