# Báo cáo 15/10/2026

## 1. Giới thiệu chung
Triển khai chạy thử nghiệm thực tế hệ thống điều khiển viễn thao trên cánh tay robot công nghiệp Universal Robots UR3e (IP `10.42.0.242`) thông qua kính thực tế ảo Meta Quest 3S (WebXR). Tối ưu hóa toàn diện vòng lặp điều khiển RTDE ServoJ 20Hz, đưa chuyển động về vận tốc manual jog đầm chắc, mượt mà (0.5 – 0.6 rad/s). Chuyển đổi cơ chế điều hướng sang Mặt Phẳng Cánh Tay (Arm-Plane Navigation) kết hợp bộ lọc Khóa trục thông minh (Soft Axis Snapping). Tách luồng camera đa nhiệm độc lập (`CameraStreamPipeline`) triệt tiêu hoàn toàn timing jitter. Nâng cấp tính năng Go Home an toàn một chạm trực tiếp từ VR và trực quan hóa mô hình 3D tay cầm Meta Quest 3S đồng bộ cùng Gizmo trên giao diện Viser 3D.

**Nhận xét:** Hệ thống đã vượt qua các bài kiểm thử cơ bản trên robot UR3e thật, thiết lập thành công luồng điều khiển khép kín từ kính Meta Quest 3S qua GPU CuRobo tới cánh tay robot với độ trễ thấp và chuyển động ổn định. Các hạn chế về hướng nhìn và trục chiếu không gian được ghi nhận đầy đủ làm tiền đề cho việc chuẩn hóa toàn diện góc điều khiển trực quan.

---

## 2. Kiểm Thử Thực Tế Robot UR3e & Tối Ưu Hóa Bộ Thực Thi (ServoJ & Arm-Plane)

### 2.1. Triển Khai Thực Nghiệm Trên Robot Thật
- Thiết lập kết nối cáp mạng LAN trực tiếp với UR Control Box (`10.42.0.242`), cấp quyền Remote Control trên Teach Pendant PolyScope, nhả phanh an toàn và khởi chạy thành công vòng lặp RTDE 20Hz.
- Hệ thống tự động gửi lệnh `stop\n` tới Dashboard Server (Port 29999) trước khi kết nối RTDE, giải phóng triệt để các thanh ghi cũ và loại bỏ hoàn toàn lỗi xung đột `One of the RTDE input registers are already in use!`.

### 2.2. Tinh Chỉnh Tham Số ServoJ & Scaled Vector Rate Limiter
- **Thông số ServoJ chuẩn công nghiệp:** Thiết lập chu kỳ điều khiển $\Delta t = 0.05\text{s}$ (20Hz), thời gian đón trước `lookahead_time = 0.12`s và hệ số khuếch đại `gain = 250`.
- **Cơ chế Scaled Vector Rate Limiter:**
  - Kẹp vận tốc góc khớp tối đa ở mức `MAX_JOINT_VEL = 0.5 - 0.6` rad/s ($\approx 34^\circ$/s), tương đương tốc độ manual jog an toàn trên tay dạy PolyScope.
  - Khi vận tốc góc vượt ngưỡng, thuật toán co giãn đồng bộ cả 6 khớp theo cùng một hệ số tỷ lệ $\alpha = v_{\text{max}} / \max(|\Delta q_i|)$, bảo toàn 100% hướng vector vận tốc trong không gian Descartes, triệt tiêu hoàn toàn hiện tượng méo quỹ đạo và rung giật cơ học.

### 2.3. Điều Hướng Mặt Phẳng Cánh Tay (Arm-Plane / Cylindrical Navigation)
- Thay thế cơ chế điều khiển Cartesian tĩnh bằng hệ tọa độ hình trụ bám theo mặt phẳng gập duỗi tự nhiên của cánh tay robot:
  $$\vec{v}_{\text{forward}} = [\cos(q_0), \sin(q_0), 0.0], \quad \vec{v}_{\text{lateral}} = [\sin(q_0), -\cos(q_0), 0.0]$$
- Triệt tiêu hoàn toàn góc lệch cơ khí khớp vai (13.1cm), giúp cánh tay duỗi thẳng ra xa chân đế khi người vận hành đẩy tay về phía trước mà khớp đế $q_0$ không bị lắc đảo.

### 2.4. Bộ Lọc Khóa Trục Thông Minh (Soft Axis Snapping)
- Áp dụng bộ lọc đường cong mềm tỷ lệ ba vùng trên cử động tay người:
  - **Độ lệch trục phụ $< 25\%$:** Triệt tiêu hoàn toàn chuyển động phụ về 0, khóa chuyển động thẳng tắp trên trục chiếm ưu thế (loại bỏ vi rung sinh học khi bóp giữ phím).
  - **Độ lệch trục phụ $25\% - 40\%$:** Làm mềm chuyển tiếp tuyến tính, không gây giật tọa độ.
  - **Độ lệch trục phụ $> 40\%$:** Cho phép tịnh tiến và xoay tự do đa chiều khi người vận hành chủ động vung tay phối hợp.

---

## 3. Kiến Trúc Đa Luồng Camera & Trực Quan Hóa 3D (Viser & Cockpit HUD)

### 3.1. Tách Luồng Đa Nhiệm Camera (`CameraStreamPipeline`)
- Chuyển toàn bộ tác vụ đọc ảnh, nén JPEG, vẽ watermark OSD và phát stream camera cổ tay sang tiến trình ngầm độc lập ở tần số 20 FPS.
- Giải phóng hoàn toàn vòng lặp điều khiển RTDE 20Hz (50ms) khỏi độ trễ kết xuất đồ họa (render latency), triệt tiêu 100% hiện tượng timing jitter (dao động chu kỳ lấy mẫu $\Delta t$).

### 3.2. Đồng Bộ Telemetry Thời Gian Thực & Nâng Cấp Cockpit HUD
- Thiết lập kênh truyền telemetry hai chiều: Vị trí và góc xoay thực tế của robot ($X, Y, Z, Q_w, Q_x, Q_y, Q_z$) cùng trạng thái phím tay cầm VR từ RTDE 20Hz được cập nhật liên tục lên giao diện Cockpit HUD trong kính Meta Quest và Dashboard Viser qua WebSocket/HTTP polling.
- **Phản hồi xúc giác (Haptic Feedback):** Tích hợp lệnh `send_haptic` kích hoạt xung rung xác nhận trên tay cầm khi ngàm kẹp đóng/mở hoặc khi hoàn thành thao tác.
- **Cơ chế Toggle Gripper:** Người vận hành chỉ cần nhấn nhả nút A (tay phải) hoặc X (tay trái) 1 lần để kẹp/hút, và nhấn lại lần nữa để thả, giảm mỏi ngón tay tối đa trong các phiên làm việc kéo dài.

### 3.3. Trực Quan Hóa Mô Hình Tay Cầm Meta Quest 3/3S Trên Viser 3D
- Tích hợp bản sao kỹ thuật số 3D (Digital Twin) của tay cầm Meta Quest 3S vào không gian Viser.
- Đặt đệm an toàn động $8.5\text{cm}$ phía trước điểm TCP và lệch phải $7.5\text{cm}$ theo mặt phẳng cánh tay, giữ khoảng cách thoáng đãng và không che khuất trục tọa độ Gizmo.
- Đồng bộ góc quay 1:1 với đầu tay máy robot và hỗ trợ đầy đủ 8 trạng thái màu sắc phản hồi trực quan độc lập cho từng phím bấm & chức năng:
  - *Nghỉ (Idle):* Trắng xám tự nhiên (độ trong suốt 60%).
  - *Bóp Cò (Trigger):* Sáng xanh lá (Tịnh tiến).
  - *Bóp Grip:* Sáng vàng cam (Xoay 3 trục).
  - *Bóp cả 2 phím (Trigger + Grip):* Sáng xanh dương ngọc (Chuyển động 6-DOF).
  - *Nút A / X:* Sáng tím hồng / Magenta (Đóng/mở ngàm kẹp).
  - *Nút B / Y:* Sáng vàng kim / Gold (Chuyển đổi Drive Mode).
  - *Xoay / gạt Joystick:* Sáng tím Violet (Điều hướng analog).
  - *Nhấn Joystick / Go Home:* Sáng xanh dương đậm Royal Blue (Lệnh và quá trình đưa robot về Home an toàn).

---

## 4. An Toàn Hệ Thống, Lệnh Go Home VR & Tái Cấu Trúc Mã Nguồn

### 4.1. Tính Năng Go Home Một Chạm Từ Kính VR (One-Touch VR Go Home)
- Bổ sung lệnh quy hoạch Home an toàn trực tiếp từ tay cầm VR: Nhấn cần gạt Thumbstick (R3/L3), nhấn giữ phím B/Y trong 3 giây, hoặc nhấp nút *Go to Home* trên WebXR Cockpit HUD.
- Khắc phục triệt để lỗi xung đột đa luồng gây crash/Segmentation fault bằng cách tích hợp khóa luồng `threading.Lock()` cho `ServoJExecutor` và đồng bộ cờ ngắt `is_navigating_home` trên vòng lặp chính.
- Sau khi hoàn thành quy hoạch Home, hệ thống tự động tái kích hoạt kết nối RTDE và đặt lại mỏ neo tọa độ (`Zero-Click Auto Reconnect`) mà người vận hành không cần thao tác bật lại thủ công.

### 4.2. Tái Cấu Trúc Mã Nguồn Theo Nguyên Lý Đơn Nhiệm
- `bootstrap.py`: Chuyên trách kiểm tra trạng thái URSim / Robot thật, tự động kích hoạt tiến trình ngầm và giải phóng tài nguyên.
- `camera_pipeline.py`: Quản lý chuyên biệt luồng đa nhiệm thu thập, xử lý và phân phối hình ảnh camera cổ tay.
- `setup_quest_adb.sh`: Kịch bản shell tự động thiết lập Developer Mode và reverse toàn bộ cổng mạng (`8443`, `8444`, `8445`, `8080`) qua cáp USB Type-C.

---

## 5. Kết Quả Đo Kiểm & Kế Hoạch Tiếp Theo

### 5.1. Bảng Tổng Hợp Chỉ Số Đo Kiểm Trên Robot Thật (15/10/2026)

| Tiêu chí đo kiểm | Giá trị đạt được | Đánh giá kỹ thuật |
| :--- | :--- | :--- |
| **Tần số vòng lặp RTDE** | $20.0 \pm 0.3\text{ Hz}$ | Chu kỳ 50ms ổn định, triệt tiêu hoàn toàn jitter timing |
| **Thời gian giải CuRobo IK (GPU)** | $3.5 - 5.8\text{ ms}$ | Đáp ứng xuất sắc thời gian thực (nhỏ hơn nhiều chu kỳ 50ms) |
| **Vận tốc góc khớp cực đại** | $0.5 - 0.6\text{ rad/s}$ | Chuyển động đầm chắc, chuẩn manual jog an toàn |
| **Độ trễ truyền hình ảnh Camera** | $\le 45\text{ ms}$ | Duy trì ổn định 20 FPS trên WebXR HUD và Viser |
| **Trạng thái bảo vệ kỳ dị** | $\sigma_{\text{min}} \ge 0.08 - 0.15$ | Vượt xa ngưỡng nguy hiểm $0.035$, không phát sinh lỗi khóa |
| **Kết nối mạng VR WebXR** | 100% qua HTTPS / WSS | Không mất gói tin, chuyển đổi linh hoạt Drive/Edit Mode |

### 5.2. Đánh Giá & Kế Hoạch Tiếp Theo
- **Đánh giá hoàn thiện:** Toàn bộ logic điều khiển 6-DOF (Tịnh tiến theo Mặt Phẳng Cánh Tay $q_0$ và Xoay Decoupled 3-Axis độc lập) đã được chuẩn hóa toán học chuẩn xác, triệt tiêu góc lệch cơ khí vai 13.1cm, sẵn sàng 100% cho các bài kiểm thử thao tác gắp đặt thực tế ngoài bàn máy.
- **Kế hoạch tiếp theo:**
  - Hoàn thiện trực quan hóa Digital Twin mô hình tay cầm Meta Quest 3S đồng bộ vị trí, góc quay và phản hồi phím bấm thời gian thực trên Viser 3D.
  - Chuẩn bị kịch bản và môi trường thu thập dữ liệu trình diễn chuẩn HDF5 phục vụ huấn luyện mô hình Robot AI (Imitation Learning / Diffusion Policy).
