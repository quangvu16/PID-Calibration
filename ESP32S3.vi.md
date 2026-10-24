# Bản ESP32-S3: đấu dây, Wi-Fi và Bluetooth

*[English](ESP32S3.md) · Tiếng Việt · [README tiếng Việt](README.vi.md) · [Sơ đồ chân Nano](PINOUT.vi.md)*

`BalancingRobotTunerESP32S3/` điều khiển robot ZYC0049 bằng **ESP32-S3-DevKitC-1 N16R8** thay cho
Arduino Nano. Robot cân bằng y như cũ, cùng tham số, telemetry và các chế độ Remote/Avoid/Follow,
và nói cùng một giao thức trên ba đường kết nối:

| Đường | Cách kết nối | Ai dùng |
| --- | --- | --- |
| **USB** | cáp vào cổng "USB" của board | app Android, `tuner_gui.py`, `autotune.py`, `serial_monitor.py` |
| **Wi-Fi** | mạng của robot, rồi mở `http://192.168.4.1/` | app Android (1.2+), trình duyệt bất kỳ (trang điều khiển), hoặc client WebSocket của bạn |
| **Bluetooth LE** | thiết bị `BalanceTuner-XXXX`, Nordic UART Service | app Android (1.2+), app serial BLE, hoặc client BLE của bạn |

Các công cụ trên máy tính chỉ kết nối qua USB. Trong app Android, bấm **Connect** rồi chọn đường kết
nối (xem [hướng dẫn app](ANDROID_APP.vi.md#kết-nối)).

## Đấu dây

![Sơ đồ chân ESP32-S3-DevKitC-1: GPIO nào nối với bộ phận nào của robot (hình bằng tiếng Anh)](pinout_esp32s3.svg)

| Tín hiệu | GPIO | Chân Nano | Ghi chú |
| --- | --- | --- | --- |
| TB6612 PWMA (tốc độ trái) | 4 | D10 | |
| TB6612 AIN1 / AIN2 (hướng trái) | 5 / 6 | D12 / D13 | |
| TB6612 PWMB (tốc độ phải) | 7 | D9 | |
| TB6612 BIN1 / BIN2 (hướng phải) | 15 / 16 | D7 / D6 | |
| TB6612 STBY | 17 | D8 | LOW = tắt động cơ |
| Encoder trái / phải | 1 / 2 | D2 / D4 | tối đa 3,3 V: chia áp nếu encoder ra 5 V |
| MPU6050 SDA / SCL | 8 / 9 | A4 / A5 | điện trở kéo lên phải về 3,3 V |
| HC-SR04 TRIG | 10 | D3 | xung kích 3,3 V chạy được với đa số module |
| HC-SR04 ECHO | 11 | D5 | ra 5 V: chia áp, ví dụ 1 kΩ nối tiếp, 2 kΩ xuống GND |
| Còi | 12 | D11 | |
| LED RGB R / G / B | 13 / 14 / 21 | A0 / A1 / A2 | tích cực mức thấp |
| GND | GND | GND | bắt buộc nối chung mass với robot |

**Các chân của ESP32-S3 là 3,3 V và không chịu được 5 V.** Hãy kiểm tra mọi tín hiệu đi *từ* robot
vào (encoder, echo của siêu âm, điện trở kéo lên I2C) trước khi nối. Ngõ vào của TB6612 nhận được
logic 3,3 V. LED RGB anode chung nối 5 V có thể sáng mờ khi "tắt", vì 3,3 V không đủ tới điện áp
anode. Các chân đã chọn tránh chân strapping (0, 3, 45, 46), USB gốc (19/20), flash và PSRAM octal
(26–37) và LED RGB trên board (38 hoặc 48).

## Build và nạp

```bash
cd "~/Arduino/ZYC0049 Self Balance/BalanceTuner/BalancingRobotTunerESP32S3"
FQBN=esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB
arduino-cli compile --fqbn $FQBN .
arduino-cli upload  --fqbn $FQBN -p /dev/ttyACM0 .
```

- Có Wi-Fi và Bluetooth, firmware khoảng 1,4 MB, nên cần phân vùng **app 3 MB**
  (`app3M_fat9M_16MB`); phân vùng mặc định 1,2 MB không đủ.
- `CDCOnBoot=cdc` đặt đường serial ở cổng **USB** (`/dev/ttyACM0`). Với `CDCOnBoot=default` thì
  nó ở cổng **UART** (`/dev/ttyUSB0`, 115200 baud).
- Nếu không nạp được: giữ BOOT, bấm RST, rồi thả BOOT.
- Thư viện: ESPAsyncWebSrv và AsyncTCP (Wi-Fi), BalanceCar và KalmanFilter, đều trong
  `~/Arduino/libraries`; Bluetooth có sẵn trong core ESP32 (ở đây là 2.0.10).

Khi bật nguồn, cổng serial ở chế độ chữ, và dòng tham số kết thúc bằng thông tin không dây, ví dụ
`# wireless: network BalanceTuner-3F2A (password balance123), http://192.168.4.1/, BLE
BalanceTuner-3F2A`. `XXXX` là phần cuối địa chỉ MAC của board.

## USB

Cắm điện thoại (USB OTG) hoặc máy tính vào cổng **USB** và dùng app hoặc các công cụ trên máy tính
như với Nano. Với app 1.2 trở lên, Android đề nghị mở app khi cắm board; với bản 1.1, hãy mở app và
bấm **Connect**.

## Wi-Fi: trang điều khiển và WebSocket

![Trang điều khiển trong trình duyệt điện thoại: trạng thái, biểu đồ góc nghiêng, điều khiển lái với joystick, và các tham số PID (hình bằng tiếng Anh)](web_control.svg)

*Trang thật, chụp trong Firefox khi đang kết nối với một robot giả lập.*

**Trong app Android:** vào mạng của robot, rồi **Connect** → **Wi-Fi (ESP32-S3)**, địa chỉ
**192.168.4.1**.

**Trong trình duyệt:**

1. Kết nối vào mạng Wi-Fi **BalanceTuner-XXXX**, mật khẩu **balance123**.
2. Mở **http://192.168.4.1/** (hoặc `http://balancetuner.local/` nếu mDNS chạy được).
3. Trang tự kết nối và hiện ● Wi-Fi.

Trang có:

- **Trạng thái**: góc nghiêng, khoảng cách, chế độ, PWM trái/phải và tần số telemetry, cùng biểu
  đồ góc nghiêng 10 giây gần nhất.
- **DRIVE**: Remote / Avoid / Follow, công tắc **Motors**, joystick, và các nút giữ **⟲ Spin**,
  **Horn**, **Spin ⟳**.
- **PID PARAMETERS**: tám tham số cùng giá trị robot đang dùng; gõ giá trị rồi bấm **Set**.
  **Read**, **Save to robot**, **Reload saved** và **Kit defaults** hoạt động như trong app.

Khi trang đang mở, nó gửi lệnh điều khiển 20 lần mỗi giây. Nếu đóng trang hoặc mất Wi-Fi, robot
dừng chạy sau 1 giây.

**Kết nối vào mạng của bạn:** đặt `WIFI_STA_SSID` và `WIFI_STA_PASSWORD` ở đầu sketch rồi nạp lại.
Robot thử trong 10 giây, rồi quay về mạng riêng của nó. Địa chỉ nhận được hiện trong dòng tham số
trên serial. **Hãy đổi `WIFI_AP_PASSWORD`** nếu người khác có thể vào mạng của robot: ai vào được
mạng đó đều lái được robot.

**Client WebSocket của bạn:** kết nối tới `ws://<robot>/ws` và trao đổi các message **nhị phân**
chứa các khung của giao thức, y như qua USB (định dạng khung ở đầu sketch). Gửi `A7 05 14 19 5A`
để bật telemetry 20 Hz và `A7 01 00 01 5A` để đọc mọi tham số; khi lái, gửi lệnh điều khiển ít
nhất mỗi giây một lần. Tối đa 2 client cùng lúc; phản hồi được gửi tới mọi client đang kết nối.

## Bluetooth LE (Nordic UART Service)

ESP32-S3 chỉ có Bluetooth LE (không có Bluetooth classic hay SPP), nên robot dùng profile serial
BLE thông dụng. **Trong app Android:** **Connect** → **Bluetooth (ESP32-S3)**, rồi bấm
**BalanceTuner-XXXX**. Các app BLE khác:

| | UUID |
| --- | --- |
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX (ghi khung vào đây) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX (notification: telemetry, phản hồi) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

1. Quét tìm **BalanceTuner-XXXX** bằng một app BLE (ví dụ nRF Connect, hoặc Serial Bluetooth
   Terminal ở chế độ BLE) rồi kết nối.
2. Bật notification cho TX.
3. Ghi khung vào RX dưới dạng **byte hex**, ví dụ `A7 05 14 19 5A` (telemetry 20 Hz) hoặc
   `A7 01 00 01 5A` (đọc tham số).

Giao thức là một luồng byte, nên một khung có thể bị tách qua nhiều notification; client phải
ghép lại trước khi phân tích. Robot xin MTU 185 byte để một khung telemetry nằm gọn trong một
notification. Mỗi lúc một client BLE; sau khi client ngắt, robot quảng bá lại.

## An toàn

- Cả ba đường dùng chung thời gian chờ điều khiển 1 giây: khi không có lệnh điều khiển nào từ bất
  kỳ đường nào, joystick, xoay và còi trở về 0.
- Wi-Fi và Bluetooth luôn bật cùng với USB; robot làm theo đường nào gửi lệnh điều khiển sau cùng.
- Lưu vào robot là ghi flash của ESP32. Lưu sau khi chỉnh là bình thường, nhưng flash mòn sau
  nhiều nghìn lần ghi, nên đừng lưu liên tục.

## Khi có sự cố

- **Không thấy mạng `BalanceTuner-XXXX`**: xem dòng tham số trên serial; nếu ghi `joined …` thì
  robot đang ở mạng của bạn.
- **Trang tải được nhưng vẫn offline**: tải lại trang; chỉ 2 client WebSocket kết nối được cùng lúc.
- **Robot khởi động lại liên tục**: bộ phát hiện sụt áp đang reset nó. Cấp nguồn board từ nguồn USB
  chịu được khoảng 500 mA; Wi-Fi có các đỉnh dòng.
- **Không có khoảng cách (luôn 0)**: không có tiếng vọng trong 25 ms; kiểm tra dây siêu âm và mạch
  chia áp.
- **Nạp không được**: giữ BOOT, bấm RST, thả BOOT, rồi nạp lại.
