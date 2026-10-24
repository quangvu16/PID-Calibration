# Hướng dẫn app Balance Tuner (Android)

*[English](ANDROID_APP.md) · Tiếng Việt · [README tiếng Việt](README.vi.md) · [Sơ đồ chân](PINOUT.vi.md) · [Tải APK](releases/BalanceTuner-v1.2.apk)*

Balance Tuner điều khiển robot tự cân bằng ZYC0049 (PBOT 3.0) và chỉnh PID trực tiếp từ điện
thoại: qua cáp USB nối vào Arduino Nano hoặc ESP32-S3, hoặc không dây (Bluetooth LE hoặc Wi-Fi) với
[bản ESP32-S3](ESP32S3.vi.md).

Tên nút và tên màn hình trong app là tiếng Anh; hướng dẫn này giữ nguyên các tên đó (in đậm) để
bạn dễ tìm.

![App Balance Tuner: tab Drive với các nút chế độ và joystick, tab Tune PID với biểu đồ trực tiếp của góc nghiêng, gyro, các thành phần vòng điều khiển, PWM và tốc độ bánh, tab Tune PID cuộn xuống phần cài đặt auto-tune và ô chỉnh tham số, và hộp thoại Connect với các lựa chọn cáp USB, Bluetooth và Wi-Fi](android_app.svg)

*Vẽ từ layout, màu và mã vẽ biểu đồ của app với telemetry giả lập, không phải ảnh chụp màn hình điện thoại.*

## Cần chuẩn bị

- **Điện thoại** Android 8.0 trở lên.
- **Với USB: cáp hoặc đầu chuyển OTG** để cắm cáp USB của robot vào điện thoại, và điện thoại có
  hỗ trợ USB OTG (USB host). Bluetooth và Wi-Fi cần bản ESP32-S3 của robot.
- **Firmware `BalancingRobotTuner`** đã nạp vào Nano (xem README). App không làm việc với
  sketch gốc của kit.
- **Rút module Bluetooth HC-06** ra khỏi robot: nó dùng chung chân D0/D1 với chip USB của Nano
  và sẽ tranh đường truyền.
- **Pin của robot** đã sạc: điện thoại chỉ cấp nguồn cho Nano qua cáp, còn động cơ vẫn chạy bằng
  pin.

App nhận các chip USB-serial thường gặp trên Nano: CH340/CH341, CP2102, FT232R/FT-X và Arduino
chính hãng (CDC-ACM).

Sơ đồ chân của robot, để kiểm tra dây nối (D0/D1 là cổng USB dùng chung với HC-06):

[![Sơ đồ chân ZYC0049: các chân của Arduino Nano và bộ phận nối vào từng chân (hình bằng tiếng Anh)](pinout.svg)](PINOUT.vi.md)

## Cài đặt

1. Tải [`BalanceTuner-v1.2.apk`](releases/BalanceTuner-v1.2.apk) về điện thoại.
2. Mở file. Nếu Android hỏi, cho phép cài app từ nguồn này.
3. Bản 1.2 cài đè lên bản 1.0 và 1.1 mà không mất dữ liệu (log, file CSV).

## Kết nối

Bấm **Connect** rồi chọn cách kết nối với robot: **USB cable**, **Bluetooth (ESP32-S3)** hoặc
**Wi-Fi (ESP32-S3)**.

**Cáp USB** (Nano hoặc ESP32-S3):

1. Bật robot, rồi cắm cáp USB của robot vào điện thoại qua đầu OTG. Với ESP32-S3, dùng cổng ghi
   **USB** trên board.
2. Android hỏi có mở **Balance Tuner** không: chọn mở. App kết nối luôn.
3. Nếu không thấy câu hỏi: mở app, bấm **Connect** → **USB cable**, rồi cho phép truy cập USB.

**Bluetooth** (chỉ ESP32-S3; Bluetooth LE):

1. Bật robot và bật Bluetooth của điện thoại.
2. Bấm **Connect** → **Bluetooth (ESP32-S3)**. Lần đầu, cho phép **Thiết bị ở gần** (Android 12 trở
   lên) hoặc **Vị trí** (Android 11 trở xuống, cần quyền này để quét Bluetooth).
3. Robot hiện tên **BalanceTuner-XXXX** kèm cường độ tín hiệu; bấm vào robot của bạn.

**Wi-Fi** (chỉ ESP32-S3):

1. Kết nối vào mạng Wi-Fi của robot **BalanceTuner-XXXX**, mật khẩu **balance123**. Android có thể
   báo mạng không có internet: cứ giữ kết nối.
2. Bấm **Connect** → **Wi-Fi (ESP32-S3)**. Địa chỉ là **192.168.4.1**, trừ khi robot đã vào mạng
   của bạn (robot in địa chỉ ra cổng serial USB); app nhớ địa chỉ này.
3. Bấm **Connect**. App nói chuyện với robot qua Wi-Fi kể cả khi đang bật dữ liệu di động.

Khi đã kết nối, nhãn trạng thái chuyển sang màu xanh với dấu ● và tên đường kết nối (ví dụ **USB
CH340**, **BLE BalanceTuner-3F2A** hoặc **Wi-Fi 192.168.4.1**). App đọc các tham số hiện tại từ robot
và nhận telemetry 20 lần mỗi giây. Màn hình được giữ sáng trong lúc kết nối. Nếu mất kết nối (rút
cáp, ra ngoài tầm Bluetooth hoặc Wi-Fi), app báo lý do và robot dừng chạy trong vòng 1 giây.

Thanh trên cùng luôn hiển thị: chế độ hiện tại (**mode**), góc nghiêng (**tilt**), khoảng cách
siêu âm (**dist**), tần số nhận dữ liệu (**Hz**), và công tắc **Motors** để bật/tắt động cơ.

Các thông báo thường gặp:

| Thông báo | Ý nghĩa và cách xử lý |
| --- | --- |
| Robot not found: plug the Nano's USB cable… | Chưa thấy robot qua USB: kiểm tra cáp, đầu OTG, và robot đã bật chưa |
| USB permission is needed… | Bạn đã từ chối quyền USB; bấm **Connect** lại và chọn cho phép |
| USB serial device (danh sách) | Có nhiều thiết bị USB-serial; chọn đúng robot |
| No robot found yet… (Bluetooth) | Không có ESP32-S3 nào đang quảng bá gần đó: robot đã bật và nạp firmware ESP32-S3 chưa? Android 11 trở xuống phải bật Vị trí |
| Could not connect over Wi-Fi: no answer from … | Điện thoại chưa vào mạng của robot, hoặc sai địa chỉ |
| Bluetooth link lost / Wi-Fi link lost | 3 giây không nhận được gì: ra ngoài tầm, hoặc robot đã tắt |
| Not connected | Lệnh cần kết nối; bấm **Connect** trước |

## Tab Drive: lái robot

- **Joystick**: kéo lên/xuống để đi tới/lùi, sang trái/phải để rẽ. Thả tay thì robot dừng.
- **⟲ Spin** / **Spin ⟳**: giữ nút để robot xoay tại chỗ ngược / thuận chiều kim đồng hồ.
- **Horn**: giữ nút để kêu còi.
- **MODE**:
  - **Remote**: robot làm theo joystick.
  - **Avoid** (tránh vật cản): đường trống thì robot chạy tới (LED xanh lá); vật cản gần hơn
    15 cm thì rẽ phải (LED xanh dương); gần hơn 8 cm thì lùi lại (LED đỏ).
  - **Follow** (đi theo): vật ở cách 20–35 cm thì robot tiến theo (LED xanh lá); gần hơn 20 cm
    thì lùi ra (LED xanh dương); xa hơn 35 cm hoặc không thấy vật thì đứng yên (LED đỏ).

Nếu không có lệnh điều khiển nào tới trong 1 giây (ví dụ rút cáp), firmware tự đưa joystick và
còi về 0.

## Tab Tune PID: chỉnh tham số

### Biểu đồ

Mặc định mỗi đại lượng có một khung riêng:

| Khung | Đường | Cần để ý |
| --- | --- | --- |
| **Tilt** (°) | tilt (đã cộng góc cân bằng), raw angle (góc thô) | tilt nên quanh 0 |
| **Gyro** (°/s) | tốc độ quay quanh trục bánh | gai nhọn = robot giật |
| **Loop terms** | angle, speed, turn (cùng một thang) | vòng nào đang làm việc nhiều nhất |
| **PWM** | L, R; vạch ở ±250 | chạm ±250 là động cơ đã hết sức |
| **Wheel speed** (/s) | L, R | bánh cứ quay về một phía là robot đang trôi |

- **Overlay**: gộp tất cả vào một biểu đồ, mỗi đường có thang riêng, với ô chọn để ẩn/hiện từng
  đường. Bấm **Split** để quay lại các khung riêng.
- **Pause**: đóng băng biểu đồ để xem kỹ.
- **Clear**: xóa dữ liệu trên biểu đồ.
- **Record CSV**: ghi telemetry ra file `balance_<ngày>_<giờ>.csv`, gồm mọi khung và các tham số
  đang dùng. Bấm **Stop recording** để dừng; app báo đường dẫn file. File nằm trong
  `Android/data/com.zyc0049.balancetuner/files` trên điện thoại.

### LAST 5 SECONDS: thống kê và gợi ý

Thống kê 5 giây gần nhất: tilt trung bình, độ lệch (std), biên độ đỉnh-đỉnh (p-p), tần số dao
động (osc), PWM trung bình, tỉ lệ bão hòa (sat), và tốc độ bánh trung bình. Dòng gợi ý bên dưới
đọc các con số đó:

| Gợi ý | Nên làm gì |
| --- | --- |
| Fallen over (\|angle\| > 30°)… | Robot đã ngã, firmware cắt động cơ; dựng robot lên |
| Motors off… | Động cơ đang tắt; đây là lúc tốt để đặt góc cân bằng |
| PWM is at its limit…% of the time | Hệ số quá cao hoặc robot sắp ngã |
| Fast oscillation (~… Hz) | Giảm Angle Kp hoặc tăng Angle Kd |
| Slow rocking (~… Hz) | Giảm Speed Kp / Speed Ki |
| Wheels keep turning one way | Kiểm tra góc cân bằng, hoặc tăng Speed Kp |
| Looks steady. | Robot đứng ổn |

**Use mean angle as balance angle**: lấy góc thô trung bình 5 giây gần nhất làm góc cân bằng.
Chỉ có ý nghĩa khi robot được giữ yên đúng ở điểm nó tự cân bằng, hoặc đang tự đứng tại chỗ. App
từ chối nếu chưa đủ vài giây dữ liệu, hoặc nếu robot đã ngã trong 5 giây đó.

### PID PARAMETERS

Mỗi tham số có một thẻ: tên, giá trị robot đang dùng (**robot: …**), một dòng gợi ý, ô nhập, và
hai nút **−** / **+**.

| Tham số | Bước −/+ | Mặc định của kit | Tác dụng |
| --- | --- | --- | --- |
| Angle Kp | 0,5 | 40 | Độ cứng chống nghiêng. Thấp: ngã; cao: rung nhanh |
| Angle Kd | 0,02 | 1,23 | Giảm chấn từ gyro. Tăng để bớt lắc; quá cao thì giật |
| Speed Kp | 0,2 | 6,7 | Giữ bánh đứng yên. Thấp: trôi đi; cao: lắc chậm tới lui |
| Speed Ki | 0,005 | 0,199 | Kéo về vị trí ban đầu. Kit dùng khoảng Speed Kp / 50 |
| Turn Kp | 0,5 | 10 | Lực rẽ khi lái |
| Turn Kd | 0,01 | 0,09 | Chống xoay ngoài ý muốn (gyro z) |
| Balance angle | 0,05 | 6,31 | Độ lệch cơ khí (độ). Sai thì robot cứ bò về một phía |
| Drive speed | 2 | 30 | Lệnh tới/lùi khi đẩy joystick hết cỡ |

- Gõ giá trị rồi bấm nút gửi trên bàn phím, hoặc bấm **−** / **+**: giá trị được gửi ngay và có
  hiệu lực ngay. Dòng **robot: …** hiện giá trị robot thực sự áp dụng.
- Thay đổi sẽ **mất khi reset** nếu chưa lưu.
- **Read**: đọc lại các giá trị từ robot.
- **Save to robot**: ghi vào EEPROM (có hỏi trước); robot dùng các giá trị đó sau mỗi lần reset.
- **Reload saved**: quay về các giá trị đã lưu trong EEPROM.
- **Kit defaults**: đặt lại giá trị mặc định của kit (chưa lưu cho tới khi bấm Save).

### HOW TO TUNE: chỉnh bằng tay

Đây là quy trình trong mục **HOW TO TUNE** của app:

1. **Góc cân bằng**: tắt **Motors**, giữ robot đứng thẳng ở điểm nó tự cân bằng, rồi bấm **Use
   mean angle as balance angle**. Sai giá trị này thì robot cứ bò về một phía.
2. **Vòng góc**: đặt Speed Kp và Speed Ki về 0. Tăng Angle Kp tới khi robot đứng được nhưng rung
   hoặc lắc nhanh, rồi giảm khoảng 40%. Tăng Angle Kd tới khi hết lắc; Kd quá cao sẽ thấy giật
   (Gyro và PWM có nhiều gai).
3. **Vòng tốc độ**: đặt lại Speed Kp của kit và Speed Ki = Speed Kp / 50. Robot trôi đi thì tăng
   Speed Kp; lắc chậm tới lui thì giảm.
4. **Theo dõi tỉ lệ bão hòa PWM** trong thống kê: trên vài % nghĩa là động cơ đã ở giới hạn và
   robot sắp ngã.

Khi đã ưng ý, bấm **Save to robot**.

### AUTO-TUNE: tự chỉnh

Robot tự thử các bộ hệ số trong lúc cân bằng trên sàn. Mỗi lượt được chấm điểm bằng một hàm chi
phí (độ lắc, gyro, độ trôi và quãng đi của bánh, bão hòa PWM; ngã là tệ nhất).

**Chuẩn bị**: sàn phẳng, trống khoảng 1 m, cáp USB để chùng. Robot ngã thì dựng lên lại; nó tự
cân bằng khi đã đứng thẳng.

1. **Chọn phương pháp** trong danh sách; dòng gợi ý bên dưới nói phương pháp đó làm gì:

   | Phương pháp | Khi nào dùng |
   | --- | --- |
   | ML (Bayesian) | Mặc định, cần ít lượt nhất (15–25 lượt cho hai hệ số) |
   | Neural (deep ensemble) | Mạng nơ-ron học hàm chi phí thay cho Gaussian process |
   | RL (policy gradient) | Học tăng cường, 2 lượt cho mỗi lần cập nhật |
   | CMA-ES | Chiến lược tiến hóa, tự học bước nhảy và tương quan giữa các hệ số |
   | Differential evolution, Particle swarm (PSO), Genetic algorithm | Tìm theo quần thể; cần 30+ lượt |
   | Nelder-Mead, SPSA | Tìm cục bộ quanh giá trị hiện tại |
   | Twiddle | Từng hệ số một, thắng thì đo lại; đơn giản, ở gần điểm đầu |
   | Random (baseline) | Mẫu ngẫu nhiên, để so sánh |

2. **Đánh dấu các hệ số cần chỉnh** (mặc định Angle Kp và Angle Kd). Có thể chọn thêm Speed Kp,
   Speed Ki và Balance angle (tìm trong khoảng ±3°).
3. **Trim the balance angle first**: chạy một lượt đứng yên trước để chỉnh góc cân bằng.
4. **Learn from past trials**: dùng các lượt cũ trong log của điện thoại khi các hệ số khác khớp.
5. **Cài đặt**:
   - **Trials**: số lượt (mặc định 20).
   - **Trial s**: số giây mỗi lượt (mặc định 8).
   - **Nudge**: lực đẩy tới/lùi trong mỗi lượt (mặc định 90; 0 = không đẩy).
   - **Range ×**: vùng tìm kiếm, từ giá trị hiện tại ÷ Range tới × Range. Chọn phương pháp sẽ đặt
     Range mặc định của nó.
6. Bấm **Start auto-tune**. App hỏi lại kèm thời gian dự kiến, rồi bật động cơ và chuyển telemetry
   lên 25 Hz.
7. **Trong lúc chạy**: dòng trạng thái hiện từng lượt (giá trị đã thử → chi phí hoặc ngã) và bộ
   tốt nhất. Biểu đồ bên dưới vẽ chi phí từng lượt (thang log, dấu × là ngã) và bản đồ chi phí dự
   đoán theo các hệ số (càng tối càng tốt).
8. **Stop auto-tune**: dừng sau lượt hiện tại; bộ tốt nhất tìm được vẫn được giữ.
9. **Khi kết thúc**, app liệt kê các thay đổi và hỏi có lưu vào EEPROM không: **Save** hoặc
   **Not now**. Telemetry trở về 20 Hz.

Các lượt được ghi vào `ml_tune_log.csv`, cùng thư mục với các file CSV. Log này tách riêng với log
của máy tính.

## Khi có sự cố

- **Kết nối rồi mà không có dữ liệu (– Hz)**: kiểm tra đã nạp firmware `BalancingRobotTuner` và
  đã rút HC-06.
- **Wi-Fi hay bị rớt**: một số điện thoại tự rời mạng không có internet sau một lúc; tắt tùy chọn
  "chuyển sang dữ liệu di động" (hoặc tương tự) trong cài đặt Wi-Fi của điện thoại.
- **Robot ngã ngay khi bật Motors**: bấm **Kit defaults**, rồi đặt lại góc cân bằng như bước 1
  của HOW TO TUNE.
- **Auto-tune làm robot ngã ở hầu hết các lượt**: bắt đầu từ bộ đã cân bằng được, giảm Range,
  hoặc đặt Nudge = 0.
- **Thay đổi mất sau khi tắt robot**: chưa bấm **Save to robot**.
