# BalanceTuner

*[English](README.md) · Tiếng Việt*

Chỉnh PID trực tiếp và điều khiển từ xa cho robot tự cân bằng ZYC0049 (PBOT 3.0), qua cáp USB
nối từ điện thoại (USB OTG) tới Arduino Nano. Không dùng module Bluetooth HC-06: hãy rút nó ra,
vì nó dùng chung chân D0/D1 với chip USB của Nano và sẽ tranh đường truyền.

- `BalancingRobotTuner/` – firmware: sketch `BalancingRobot` của bộ kit, thêm telemetry, tham
  số PID chỉnh được lúc chạy và lưu trong EEPROM, và công tắc bật/tắt động cơ. Giao thức truyền
  được mô tả ở đầu file `.ino`. Gói điều khiển 11 byte của kit vẫn dùng được, nên app
  btcontroller của kit có thể lái robot ở 9600 baud (firmware hiện chạy ở 115200).
- `android_app/` – app Balance Tuner (Kotlin, không dùng thư viện ngoài; có driver USB-serial
  riêng cho CH340, CP210x, FT232R và CDC-ACM trong `UsbSerialPort.kt`). APK: `~/BalanceTuner.apk`;
  bản build sẵn để tải: [`releases/BalanceTuner-v1.1.apk`](releases/BalanceTuner-v1.1.apk).
- `serial_monitor.py` – giải mã cùng luồng dữ liệu đó trên máy tính (`/dev/ttyUSB0`).
- [`PINOUT.vi.md`](PINOUT.vi.md) – sơ đồ chân: chân nào của Nano nối với bộ phận nào (driver động cơ, encoder, IMU, siêu âm, LED, còi).

## Nạp firmware

Chức năng tự reset của con Nano này không hoạt động, nên hãy bấm nút RESET trong lúc lệnh nạp
đang cố kết nối.

```bash
cd "~/Arduino/ZYC0049 Self Balance/BalanceTuner/BalancingRobotTuner"
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 .
arduino-cli upload  --fqbn arduino:avr:nano:cpu=atmega328 -p /dev/ttyUSB0 .
```

## Build app

```bash
cd "~/Arduino/ZYC0049 Self Balance/BalanceTuner/android_app"
JAVA_HOME=~/Android/jdk ANDROID_HOME=~/Android/Sdk ~/Android/gradle-8.7/bin/gradle --no-daemon assembleRelease
cp app/build/outputs/apk/release/app-release.apk ~/BalanceTuner.apk
```

Lệnh này dùng Gradle 8.7 đã cài sẵn trong `~/Android/gradle-8.7`. `./gradlew` cũng dùng được,
nhưng trước tiên nó tải Gradle từ mirror Tencent, và trên máy này việc tải đó bị đứt giữa chừng.
Lần build đầu tiên cũng tải các thư viện build của Android, nên cần có mạng.

## Sử dụng

Nối cổng USB của Nano với điện thoại bằng cáp/đầu chuyển OTG. Android sẽ đề nghị mở Balance
Tuner và kết nối ngay; nếu không, hãy mở app, bấm Connect và cho phép truy cập USB. Điện thoại
cấp nguồn cho Nano qua cáp; động cơ vẫn chạy bằng pin của robot.
**Drive** có joystick, nút xoay tại chỗ, còi, và các chế độ Remote/Avoid/Follow (điều khiển /
tránh vật cản / đi theo).
**Tune PID** có biểu đồ trực tiếp: mặc định mỗi đại lượng một khung, giống `tuner_gui.py` (góc
nghiêng và góc thô, gyro, các thành phần vòng góc/tốc độ/rẽ trên cùng một thang, PWM trái/phải
với vạch bão hòa ±250, tốc độ bánh trái/phải); Overlay chuyển sang một biểu đồ duy nhất, mỗi
đường có thang riêng và ô chọn riêng. Bên dưới là thống kê 5 s gần nhất kèm gợi ý, ô chỉnh từng
tham số (áp dụng ngay), Save to robot (lưu EEPROM) và Record CSV. Mục "How to tune" trong app
hướng dẫn cách chỉnh.

## Gỡ lỗi từ máy tính

Khi Nano cắm vào USB của máy tính, Arduino Serial Monitor/Plotter ở 115200 (hoặc
`python3 serial_monitor.py --text`) hiển thị các dòng chữ đọc được. Đây là chế độ mặc định khi
bật nguồn (`d` đưa về chế độ này sau khi đã sang nhị phân):
`tilt:0.12 angle:0.29 gyro:-1.3 A:4 S:-12 T:0.0 pwmL:-8 pwmR:-8 pulL:0 pulR:0 dist:42 motors:1 fallen:0 mode:1`,
sau một dòng liệt kê tham số hiện tại. Gửi `b`, hoặc khi app kết nối, robot chuyển lại sang
nhị phân. Không có `--text`, `serial_monitor.py` giải mã các khung nhị phân.

## Chỉnh PID với biểu đồ trực tiếp trên máy tính

`python3 tuner_gui.py [/dev/ttyUSB0]` (Tkinter + matplotlib + pyserial) là phiên bản máy tính
của màn hình Tune PID trong app: biểu đồ cuộn của góc nghiêng/góc thô, gyro, các thành phần
vòng góc/tốc độ/rẽ, PWM (có vạch bão hòa ±250) và tốc độ bánh. Mỗi thành phần vòng điều khiển
có đường riêng, như trong Balancing_Robot_Firmware của farrellf. Bên cạnh là: ô chỉnh mọi tham
số (Enter/Set áp dụng ngay, -/+ = ±10 %, angle0 ±0,1°), Get/Save/Load/Defaults, bật/tắt động
cơ, tần số telemetry, nút Nudge (đẩy tới rồi lùi) để xem robot hồi phục, thống kê cửa sổ, nút
"angle0 from mean", và Record CSV (telemetry cùng các tham số đang dùng). Hãy đóng Arduino
Serial Monitor trước; đóng cửa sổ sẽ đưa robot về chế độ chữ.

### `tuner_gui.py` chi tiết

**Khởi động.** `python3 tuner_gui.py [cổng] [-b baud] [--window s]` (mặc định `/dev/ttyUSB0`,
115200, 10 s). Cần `python3-tk`, matplotlib, numpy và pyserial. Mở cổng sẽ reset Nano, nên GUI
đợi 2,5 s, rồi chuyển robot sang telemetry nhị phân (`b`), đặt tần số telemetry và đọc các tham
số. Nếu không được, dòng trạng thái màu đỏ ở dưới cùng sẽ báo; hãy kiểm tra cổng rồi bấm
**Get**. Đóng cửa sổ sẽ dừng bánh và đưa robot về chế độ chữ (`d`) cho Serial Monitor.

**Bố cục.** Bên trái: các khung Parameters, Robot và Auto-tune, phần thống kê và dòng trạng
thái. Bên phải: năm biểu đồ chung một trục thời gian, hiển thị *Window s* giây gần nhất (0 =
hiện tại), vẽ lại mỗi 100 ms:

| Biểu đồ | Đường | Cần để ý |
| --- | --- | --- |
| Tilt / angle (độ) | tilt = angle + angle0, góc thô; vạch đen ở 0 | tilt nên ở gần 0. Rung nhanh: Kp quá cao. Lắc chậm: tăng Kd |
| Gyro (độ/s) | tốc độ quay quanh trục bánh (thứ mà Kd nhân vào) | gai nhọn = giật; RMS nằm trong phần thống kê |
| Loop terms | thành phần góc (Kp, Kd), tốc độ (Kp, Ki speed), rẽ | vòng nào đang làm việc; thành phần tốc độ lớn chống lại thành phần góc là lắc chậm |
| PWM | lệnh động cơ trái / phải; vạch đỏ đứt ở ±250 | chạm ±250 = bão hòa: động cơ hết sức để hồi phục |
| Wheel speed | số xung encoder mỗi khung, trái / phải | trôi đi xa: Kp speed quá thấp; tới lui chậm: Kp speed quá cao |

**Khung Parameters.** Mỗi tham số một hàng: Kp/Kd (góc), Kp/Ki speed, Kp/Kd turn, angle0 (độ)
và drive speed. Gõ giá trị rồi bấm Enter hoặc **Set**: giá trị được giới hạn trong khoảng của
firmware (Kp 0–100, Kd 0–10, Kp speed 0–50, Ki speed 0–2, Kp turn 0–50, Kd turn 0–5, angle0
±15°, drive speed 0–100). Robot gửi lại giá trị nó thực sự áp dụng, và ô hiển thị giá trị đó.
**−/+** đổi một hệ số 10 %, và đổi angle0 0,1°. Thay đổi có hiệu lực ngay nhưng mất khi tắt
nguồn nếu chưa lưu. **Get** đọc lại từ robot, **Save** ghi vào EEPROM (có hỏi trước), **Load**
quay về giá trị trong EEPROM, **Defaults** về giá trị mặc định của firmware.

**Khung Robot.**
- **Motors on**: bật hoặc tắt động cơ.
- **Nudge**: chạy tới 0,4 s rồi lùi 0,4 s, để xem robot hồi phục.
- **Telemetry Hz**: 1–25 khung mỗi giây (mặc định 25).
- **Window s**: số giây biểu đồ hiển thị (2–40).
- **Pause**: đóng băng biểu đồ. Telemetry vẫn tiếp tục về, và việc ghi vẫn chạy.
- **Record CSV**: ghi mọi khung (`host_s`, các trường telemetry) cùng 8 tham số đang dùng lúc
  đó.
- **angle0 from mean**: khi robot đang tự cân bằng tại chỗ, đề nghị angle0 = −(góc thô trung
  bình) trong cửa sổ đang hiển thị. Cần ít nhất 20 khung.

**Thống kê** (trong cửa sổ đang hiển thị): trạng thái (motors on / motors off / FALLEN), tần số
khung nhận được, trung bình và độ lệch của tilt, gyro RMS, tốc độ bánh RMS, tỉ lệ khung PWM bão
hòa, và khoảng cách siêu âm.

**Khung Auto-tune.**
- **Danh sách phương pháp**: chọn một phương pháp; dòng gợi ý bên dưới nói nó làm gì. Chọn
  phương pháp cũng đặt luôn Range của nó.
- **Hệ số cần chỉnh**: đánh dấu chọn (mặc định Kp và Kd góc; Kp speed, Ki speed và angle0 tùy
  chọn).
- **trim angle0 first**: chạy một lượt đứng yên trước, rồi đặt angle0 = −góc trung bình.
- **learn from past trials**: log của các lần chạy trước được đưa vào mô hình, khi các hệ số
  khác của chúng khớp.
- **Cài đặt**: Trials 5–80 (mặc định 20), Trial s 4–20 (8), Nudge 0–127 (90; 0 = không đẩy),
  Range × 1,2–5 (vùng tìm kiếm từ giá trị hiện tại ÷ Range tới hiện tại × Range).
- **Start auto-tune**: hỏi trước, kèm thời gian dự kiến (trials × (trial s + 3) ÷ 60 phút). Sau
  đó bật động cơ và đặt telemetry 25 Hz, vì tốc độ bánh trong hàm chi phí tính theo khung.
- **Trong lúc chạy**: dòng thông tin hiện giá trị của từng lượt, rồi chi phí hoặc FELL (ngã),
  và bộ tốt nhất đến lúc đó cùng chi phí dự đoán của nó.
- **Stop auto-tune**: dừng sau lượt hiện tại, và bộ tốt nhất tìm được vẫn được giữ.
- **Khi kết thúc**: liệt kê những gì đã thay đổi và hỏi có ghi vào EEPROM không.

**Một lượt thử**, giống nhau cho mọi phương pháp và trong `autotune.py`:
1. Đợi robot đứng được 1,5 s với bộ tham số trước đó: ít nhất 25 khung, góc thô trong ±15°,
   không ngã.
2. Áp dụng bộ mới, rồi đợi 1,5 s cho ổn định.
3. Sau 1 s, đẩy robot tới 0,4 s và lùi 0,4 s (trừ khi Nudge = 0).
4. Đo cho tới hết *Trial s*, hoặc tới khi ngã. Ngã là khi firmware báo cờ fallen hoặc góc thô
   vượt quá ±30°.

Chi phí là

  độ lệch góc + 0,1 × gyro RMS + 0,2 × tốc độ bánh RMS + 0,005 × quãng đi + 20 × tỉ lệ bão hòa

trong đó quãng đi là khoảng cách xa nhất tính từ điểm đầu, theo xung encoder. Một lần ngã tính
100 + 10 × số giây còn thiếu. Sau khi ngã, bộ tốt nhất được đặt lại, và chương trình đợi bạn
dựng robot lên; robot tự cân bằng khi đã đứng thẳng.

**Cửa sổ Model** (nút **Model**; tự mở ở lượt đầu tiên):
- **Biểu đồ trái**: chi phí của từng lượt, thang log. Dấu × đỏ là ngã; đường bậc thang xanh lá
  là lượt tốt nhất tới lúc đó; đường đứt là bộ tốt nhất theo dự đoán của mô hình (với Twiddle
  là trung bình hai lượt xác nhận).
- **Biểu đồ phải, chọn một hệ số**: chi phí dự đoán của mô hình theo hệ số đó, kèm dải ±2 độ
  lệch chuẩn.
- **Biểu đồ phải, từ hai hệ số trở lên**: bản đồ màu theo hai hệ số đầu, càng tối càng tốt, các
  hệ số còn lại giữ ở giá trị tốt nhất. Ngôi sao trắng là bộ tốt nhất; chấm đỏ là ngã, chấm xám
  là các lần chạy trước. Các hệ số dùng trục log, angle0 dùng trục tuyến tính.

**File.** Mỗi lượt được ghi thêm vào `ml_tune_log.csv` cạnh script: số lượt, giai đoạn
(`phương pháp:hệ số`), 8 tham số, chi phí, ngã hay không, trung bình và độ lệch góc, gyro RMS,
tốc độ RMS, quãng đi và tỉ lệ bão hòa. Log này và `autotune_log.csv` là thứ mà "learn from past
trials" đọc.

**Khi có sự cố.**
- **"no telemetry" hoặc "no reply from the robot"**: có chương trình khác đang giữ cổng (đóng
  Arduino Serial Monitor), hoặc sai cổng (thử `/dev/ttyACM0`).
- **Permission denied khi mở cổng**: thêm tài khoản của bạn vào nhóm `dialout` rồi đăng nhập
  lại.
- **Robot ngã ở hầu hết các lượt**: bắt đầu từ bộ tham số đã cân bằng được, giảm Range, hoặc
  đặt Nudge = 0.

## Tự chỉnh từ máy tính

`python3 autotune.py` tự chỉnh các tham số trong lúc robot cân bằng trên sàn (trống ~1 m, cáp
USB để chùng, đã đóng Serial Monitor). Nó chỉnh `angle0` theo góc trung bình mà robot đứng
được, rồi tìm Kp/Kd góc và Kp/Ki tốc độ, chấm điểm mỗi lượt 8 s từ telemetry: độ lệch góc, gyro
RMS, tốc độ bánh và quãng đi từ encoder, độ bão hòa PWM; ngã bị tính điểm tệ nhất. Mỗi lượt đẩy
robot tới rồi lùi để buộc nó phải hồi phục (`--no-nudge` để đứng yên). Sau khi ngã, nó đặt lại
bộ tham số tốt nhất và đợi bạn dựng robot lên. Nó hỏi trước khi lưu vào EEPROM và ghi mọi lượt
vào `autotune_log.csv`. `--stages angle`, `--trial`, `--max-trials`: xem `--help`.

### Auto-tune trong GUI: mười một phương pháp

`tuner_gui.py` có khung **Auto-tune** (`ml_tune.py`, chỉ cần numpy) chạy phương pháp được chọn
trong danh sách trên các hệ số đã đánh dấu. Mọi phương pháp dùng chung cách thử, hàm chi phí,
cách xử lý khi ngã, nút Stop, log, cửa sổ Model và câu hỏi lưu EEPROM. Twiddle và ML (Bayesian)
được mô tả bên dưới; các phương pháp còn lại:

| Phương pháp | Nhóm | Ghi chú |
| --- | --- | --- |
| Neural (deep ensemble) | mô hình thay thế bằng học sâu | 5 mạng nơ-ron nhỏ (32-32, tanh, Adam, numpy) học hàm chi phí; lượt kế tiếp = trung bình − 1,5 × độ phân tán thấp nhất |
| RL (policy gradient) | học tăng cường | PGPE: chính sách Gauss trên các hệ số, mỗi cặp lượt đối xứng là một episode |
| CMA-ES | chiến lược tiến hóa | tự thích nghi bước nhảy và tương quan giữa các hệ số; ~6–8 lượt mỗi thế hệ |
| Differential evolution | tiến hóa | DE/rand/1/bin; nên cho 30+ lượt |
| Particle swarm (PSO) | bầy đàn | nên cho 30+ lượt |
| Genetic algorithm | di truyền | chọn lọc đấu loại, lai ghép pha trộn, đột biến, giữ cá thể tốt nhất; 30+ lượt |
| Nelder-Mead | tìm kiếm trực tiếp | đơn hình cục bộ, không có mô hình; dễ bị nhiễu đánh lừa |
| SPSA | gradient ngẫu nhiên | 2 lượt cho mỗi ước lượng gradient, kiểu extremum seeking; cục bộ |
| Random (baseline) | – | mẫu Latin hypercube; mốc mà phương pháp khác phải vượt qua mới đáng dùng |

Trừ Twiddle, mọi phương pháp báo kết quả là bộ đã thử có chi phí *dự đoán* thấp nhất theo
Gaussian process của cửa sổ Model, giống ML, nên một lượt may mắn đơn lẻ không thắng được. Trên
một hàm chi phí giả lập có nhiễu, tất cả đều đưa Kp/Kd từ điểm đầu về gần điểm tối ưu trong 25
lượt; trên robot thật, ML (Bayesian) vẫn cần ít lượt nhất. Không đưa vào: Ziegler–Nichols,
Cohen–Coon và relay (Åström–Hägglund) cần hệ hở ổn định hoặc một dao động duy trì, mà robot cân
bằng thì ngã trong cả hai trường hợp; RL trực tuyến hoặc lập lịch hệ số mờ (fuzzy) trong vòng
200 Hz thì cần sửa firmware. App (`AutoTuner.kt`) cũng có đủ cả mười một phương pháp.

- **Twiddle** là cách tìm của `autotune.py`, chạy từ GUI. Nó bước từng hệ số một (bắt đầu
  ±25 %), và một lần thắng được đo lại trước khi giữ. Range mặc định ×3 như trong
  `autotune.py`. Đánh dấu cả bốn hệ số thì chúng được chỉnh cùng nhau, không theo từng giai
  đoạn. Ở đây bản đồ của cửa sổ Model chỉ minh họa các lượt; nó không điều khiển việc tìm.
- **ML (Bayesian)**, mặc định, được mô tả bên dưới.

Ngoài bốn hệ số, có thể đánh dấu **angle0** (góc cân bằng). Đây là độ lệch, không phải hệ số
nhân, nên được tìm tuyến tính trong khoảng giá trị hiện tại ±3°, và Twiddle bước nó theo độ
(±0,75° lúc đầu). angle0 sai thể hiện trong chi phí qua việc bánh trôi và quãng đi. Lựa chọn
chạy một lần "trim angle0 first" (angle0 = −góc trung bình) vẫn có để làm điểm khởi đầu. Mọi
lượt được ghi vào `ml_tune_log.csv`. `autotune.py` vẫn chạy riêng được từ dòng lệnh.

**ML (Bayesian)** dùng tối ưu Bayes: một Gaussian process học bề mặt chi phí theo các hệ số đã
chọn (mặc định Kp/Kd góc, tùy chọn Kp/Ki tốc độ) từ mọi lượt thử. Kể cả các lần ngã, và cả các
lượt cũ trong `ml_tune_log.csv`/`autotune_log.csv` khi các hệ số khác khớp. Mỗi lượt kế tiếp
được đặt ở nơi có kỳ vọng cải thiện (expected improvement) cao nhất. Cách thử và hàm chi phí
giống `autotune.py`. Vùng tìm kiếm từ giá trị hiện tại ÷ Range tới hiện tại × Range, theo thang
log. Kết quả là bộ đã thử có chi phí *dự đoán* thấp nhất, nên một lượt may mắn không thắng được.
Hai hệ số thường cần 15–25 lượt. Cửa sổ **Model** vẽ chi phí của mọi lượt và bản đồ chi phí dự
đoán của mô hình (ngã màu đỏ, các lần chạy trước màu xám). Stop giữ lại bộ tốt nhất tìm được
tới lúc đó. Khi kết thúc, GUI hỏi trước khi lưu vào EEPROM.

App có cùng chức năng, viết lại bằng Kotlin (`AutoTuner.kt`, `ModelView.kt`, không thư viện
ngoài), ở **Tune PID → Auto-tune**. App có cùng mười một phương pháp trong một danh sách, cùng
cài đặt và hàm chi phí, cùng biểu đồ chi phí từng lượt và bản đồ chi phí dự đoán. Trong lúc chạy
app chuyển telemetry sang 25 Hz để chi phí so sánh được với máy tính, rồi trả về 20 Hz. Các lượt
được ghi vào `ml_tune_log.csv` trong Android/data/com.zyc0049.balancetuner/files. Log đó tách
riêng với log của máy tính, nên mỗi bên chỉ học từ các lượt cũ của chính nó.

Đường truyền chạy ở 115200 baud (`Serial.begin` trong firmware, `SerialLink.BAUD` trong app, và
`serial_monitor.py -b` phải khớp nhau). Telemetry 20 Hz dùng khoảng 5% băng thông đó. App
btcontroller của kit và HC-06 ở mặc định 9600 baud không còn dùng được với firmware này.
