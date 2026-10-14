# UR3e Teleoperation qua Meta Quest 3S, Viser 3D & CuRobo GPU IK

Hệ thống điều khiển viễn thao thời gian thực (Real-time Teleoperation) cho cánh tay robot công nghiệp Universal Robots UR3e sử dụng kính thực tế ảo Meta Quest 3S (WebXR) và chuột trên giao diện Web 3D Viser. Hệ thống giải bài toán động học nghịch (Inverse Kinematics) và kiểm tra va chạm hoàn toàn trên GPU bằng NVIDIA CuRobo, đồng thời hỗ trợ thu thập dữ liệu trình diễn (Demonstrations) chuẩn HDF5 phục vụ huấn luyện mô hình Robot AI (Imitation Learning, Diffusion Policy, ACT).

---

## 1. Tính Năng Nổi Bật

1. **Điều khiển Viễn Thao Thời Gian Thực (20Hz RTDE servoJ):**
   - Đồng bộ chuyển động 6DoF từ tay cầm Meta Quest 3S hoặc chuột máy tính (Viser 3D Transform Controls Target Widget) sang robot UR3e với độ trễ cực thấp.
   - Bộ thực thi `ServoJExecutor` sử dụng tham số tối ưu cho chu kỳ 20Hz: `dt = 0.05`s, `lookahead_time = 0.04`s, `gain = 800`.
   - Chế độ điều khiển VR phân tách chức năng:
     - **Chỉ bóp Cò (Trigger):** Tịnh tiến không gian (World-Centric mapping, khóa góc xoay).
     - **Chỉ bóp Grip (Nút hông):** Xoay đầu hút/gắp theo hệ tọa độ đầu công cụ (Tool-Centric mapping, khóa tịnh tiến).
     - **Bóp cả Trigger + Grip:** Chuyển động tự do 6DoF đồng thời.
     - **Cơ chế Smart Clutch:** Tự động chốt mỏ neo mới (Reset Anchor) ngay khi thay đổi nút bấm, triệt tiêu hoàn toàn lỗi nhảy vọt tọa độ (teleport).
   - Chế độ điều khiển Chuột trên PC: Tương tác trực tiếp bằng cách kéo các trục tịnh tiến hoặc vòng xoay góc của khối Target Widget trên trình duyệt Viser.

2. **Động Học Nghịch Tránh Va Chạm Trên GPU (NVIDIA CuRobo 0.8) & Tối Ưu Không Gian Khớp:**
   - Giải bài toán IK thời gian thực trên GPU CUDA (3–6ms), tự động kiểm tra va chạm với bàn làm việc, vách ngăn và cụm camera/ngàm kẹp.
   - Vùng an toàn tinh gọn: `collision_sphere_buffer: 0.005` (5mm) và các quả cầu ngàm kẹp/camera bán kính 2.2cm.
   - Chuẩn hóa giới hạn góc khớp: Khớp khuỷu (`elbow_joint`) được mở rộng an toàn lên $[-2.80, 2.80]$ rad ($\approx \pm 160^\circ$) và cổ tay duy trì dải quay $\pm 2\pi$, loại bỏ triệt để hiện tượng kẹp góc sai lệch ép thuật toán đảo nhánh IK gây lỗi vặn xoắn khớp khi hạ thấp gắp cà chua.
   - Cơ chế chống đảo nhánh IK (Branch Consistency Penalty): Bổ sung hệ số phạt lệch nhánh (`branch_penalty = 50.0` khi lệch $> 0.8$ rad) kết hợp 64 hạt giống (seeds) và 16 nghiệm trả về, giữ quỹ đạo luôn mượt mà và ổn định.

3. **Luồng Stream Camera Cổ Tay Trực Tiếp (Real-time 20 FPS) & Driver Hợp Nhất:**
   - Khung "CAMERA STREAM" trên giao diện WebXR Cockpit và sidebar Viser hiển thị trực tiếp góc nhìn từ camera cổ tay robot ở tần số thời gian thực 20 FPS.
   - Driver camera hợp nhất (`camera_driver.py`) hỗ trợ chuyển đổi linh hoạt qua cờ dòng lệnh CLI:
     - **Camera mô phỏng 3D (`SyntheticWristCamera`):** Kết xuất đồ họa màu RGB 640x480, góc nhìn Eye-in-Hand từ đầu tay máy quan sát quả cà chua và khay chứa.
     - **Camera vật lý thật (`RealWristCamera`):** Đọc trực tiếp luồng hình ảnh từ camera USB hoặc Intel RealSense qua OpenCV (`--camera real --camera-device 0`).
   - Tự động tích hợp thông tin trạng thái OSD (trạng thái ngàm kẹp Gripper) và truyền đồng thời qua kênh WebSocket và MJPEG `/video_feed`.

4. **Mô Phỏng Gắp/Thả 3D (Interactive Grasp Simulator):**
   - Mô phỏng cơ chế hút dính vật thể quả cà chua 3D khi ngàm tiếp cận đúng vị trí (khoảng cách 3D $< 9.5$cm hoặc lệch XY $< 5.5$cm kèm độ cao Z $< 13$cm).
   - Tự động nhận diện khi đưa vật thể vào khay chứa và ghi nhận sự kiện hoàn thành nhiệm vụ (`PLACE_SUCCESS`).
   - Nút bấm Gripper độc lập (nút A/X trên tay cầm VR hoặc checkbox `Gripper (Bóp Kẹp)` trên Web UI) không bị xung đột với nút Grip xoay cổ tay.

5. **Cơ Chế Tự Động Thu Hồi Tọa Độ XYZ (Auto-Snap on Stuck) & Reset Seed Thoát Kẹt:**
   - Khi robot gặp tình trạng kẹt tư thế (vặn xoắn khớp) hoặc vướng vật cản kéo dài quá 1.2 giây (`snap_threshold = 1.2`), hệ thống tự động rung phản hồi Haptic tới tay cầm và giật tọa độ Widget 3D về đúng đầu tay máy robot thật (`fk_res`).
   - Tự động gán lại hạt giống giải IK `teleop._last_valid_q = list(q_real)`, giúp thuật toán lập tức thoát kẹt và sẵn sàng nhận chuyển động mới từ người vận hành.

6. **Tự Động Dọn Dẹp Kịch Bản Ngầm & Giải Phóng Thanh Ghi RTDE:**
   - Bộ thực thi `ServoJExecutor` và controller `CuRoboURSim` tích hợp cơ chế tự động gửi lệnh `stop\n` tới Dashboard Server (Port 29999) trước khi kết nối RTDE.
   - Loại bỏ hoàn toàn lỗi xung đột thanh ghi (`One of the RTDE input registers are already in use!`) hoặc treo tiến trình khi khởi động lại sau các lần thoát đột ngột.
   - Tự động fallback sang dải thanh ghi trên (`FLAG_UPPER_RANGE_REGISTERS`) nếu kết nối trên robot thật có cấu hình fieldbus (Profinet/EtherNet-IP).

7. **Quy Hoạch Trở Về Home An Toàn Không Giật & Chống Rung Lắc Hình Ảnh:**
   - Chuyển đổi phương thức thực thi quy hoạch Home từ vòng lặp `servoJ` rời rạc sang bộ sinh quỹ đạo phần cứng `movej` chạy ở tần số 500Hz trực tiếp trong bộ điều khiển Universal Robots qua cổng Secondary Port 30002.
   - Tự động hòa trộn góc bo cung (`blend radius = 0.025`m) khi bám theo các waypoints né vật cản từ CuRobo `MotionPlanner`, triệt tiêu 100% hiện tượng giật cục cơ học do thiếu hụt buffer servoJ.
   - Loại bỏ xung đột đa luồng cập nhật Viser 3D, xóa bỏ triệt để hiện tượng glitch hình ảnh chập chờn.

8. **Tự Động Tái Kích Hoạt Teleop Sau Khi Về Home (Zero-Click Auto Reconnect):**
   - Tích hợp cờ giám sát `_was_navigating_home`, tự động tái kết nối RTDE `executor.connect()`, đặt lại mỏ neo tay cầm VR `pose_source.reset_anchor()` và đưa hệ thống vào trạng thái sẵn sàng điều khiển ngay khi robot về tới Home mà người dùng không cần phải click tắt/bật lại checkbox thủ công.

9. **Giao Diện Viser 3D Cockpit Tối Giản (Bố Cục Lưới 2x2):**
   - Gom cụm 4 thao tác điều khiển cốt lõi thành ma trận 2x2 tiện dụng ngay trên đầu bảng điều khiển:
     - Hàng 1: `Bật Teleop` (Checkbox) & `Gripper (Bóp Kẹp)` (Checkbox).
     - Hàng 2: `Record AI Episode` (Checkbox) & `Hủy Episode (Discard)` (Checkbox dạng tự nhả sau 200ms).
   - Tối ưu không gian hiển thị, giúp tập trung tối đa vào không gian làm việc 3D và khung camera stream.

10. **Thu Thập & Quản Lý Dữ Liệu AI Chuẩn HDF5:**
    - Ghi lại chuỗi thao tác đầy đủ: góc khớp (`qpos`), vận tốc (`qvel`), lệnh điều khiển (`action`), trạng thái ngàm kẹp (`gripper`), ảnh camera cổ tay (`images/wrist_camera`), timestamp ở tần số 20Hz.
    - Tự động đánh dấu kết quả thành công/thất bại (`success=True/False`).

11. **Bộ Công Cụ Kiểm Định & Tái Lập Quỹ Đạo (Data Sanity & Replay):**
    - `verify_dataset.py`: Quét toàn bộ file HDF5 kiểm tra giá trị NaN/Inf, độ lệch chu kỳ lấy mẫu (dt jitter), kích thước tensor hình ảnh.
    - `replay_episode.py`: Tái hiện lại quỹ đạo trình diễn ở chế độ Offline Dry-Run hoặc Live Replay gửi lệnh trực tiếp sang robot.

12. **Các Lớp Bảo Vệ An Toàn (Safety Gate & Singularity Guard):**
    - **Giám sát điểm kỳ dị (Singularity Guard):** Đánh giá ma trận Jacobian thời gian thực, tự động chặn lệnh nếu giá trị suy biến nhỏ nhất $\sigma_{min} < 0.035$.
    - **Chống chúi cắm bắp tay:** Giới hạn góc nâng vai `shoulder_lift_joint` $\le +0.05$ rad để ngăn ngừa rủi ro đâm bắp tay xuống mặt bàn.
    - **Giới hạn tốc độ và gia tốc (Rate Limiter):** Kẹp vận tốc góc khớp tối đa $\le 2.0$ rad/s (tối đa 0.1 rad mỗi bước 50ms) để bảo vệ động cơ robot vật lý.
    - **Phát hiện lật nhánh IK:** Chặn các bước nhảy góc khớp đột ngột (`max_jump > 1.5` rad).
    - **Giám sát mất kết nối (Timeout Watchdog):** Tự động dừng khẩn cấp robot qua `servoStop` nếu mất tín hiệu VR quá thời gian timeout.

---

## 2. Kiến Trúc & Luồng Hoạt Động Của Hệ Thống

Toàn bộ hệ thống điều khiển viễn thao vận hành theo luồng khép kín thời gian thực ở tần số 20Hz (chu kỳ 50ms), kết nối đồng bộ giữa giao diện điều khiển (Chuột trên PC / Kính Meta Quest 3S), máy chủ giải thuật GPU CuRobo và bộ điều khiển robot Universal Robots UR3e.

### 2.1. Sơ Đồ Khối Luồng Dữ Liệu

```text
[Tay cầm Meta Quest 3S]               [Chuột PC trên Viser 3D]
  (Trigger/Grip/Nút A/X/B/Y)             (Transform Controls Widget)
          │                                        │
          ▼ (WebSocket WSS 8444)                   │
[QuestServer & DeltaPoseSource]                    │
  (World-Centric, Lọc EMA, Smart Clutch)           │
          │                                        │
          └───────────────┬────────────────────────┘
                          ▼
              [Target Widget 3D (Viser)]
           (Tọa độ đích 6DoF: X, Y, Z, Qw, Qx, Qy, Qz)
                          │
                          ▼
              [CuRobo IK Solver (GPU CUDA)]
           (Batched seeds, tránh va chạm 3-5ms -> q_target)
                          │
                          ▼
              [SafetyGate & SingularityGuard]
           (Jacobian sigma_min >= 0.035, Rate Limiter 2.0 rad/s)
                          │
                          ▼
              [ServoJExecutor (RTDE Cổng 30004)]
           (servoJ với lookahead_time = 0.04s, gain = 800)
                          │
                          ▼
              [Robot UR3e (URSim / Robot thật)]
                          │
         ┌────────────────┴────────────────┐
         ▼                                 ▼
[DataLogger HDF5]             [Camera Cổ Tay (Ảo/Thật)]
(q, qd, tcp, action, nhãn)    (20 FPS, OSD Watermark, MJPEG & WS)
```

### 2.2. Cơ Chế Máy Trạng Thái (Edit Mode vs. Drive Mode)

Hệ thống thiết lập 2 chế độ vận hành độc lập, hỗ trợ đồng bộ hai chiều tức thì giữa kính thực tế ảo và màn hình máy tính:

1. **Chế độ Chỉnh sửa (Edit Mode):**
   - **Mục đích:** Thao tác cài đặt thông số, quan sát góc nhìn 3D và chuẩn bị nhiệm vụ.
   - **Tương tác:** Màn hình giao diện HUD nổi lơ lửng trong không gian AR nhận tia laser từ tay cầm (`pointer-events: auto`), cho phép người dùng click chuột ảo vào các nút bấm, chọn tùy chọn và tương tác với khung Viser 3D.
   - **An toàn:** Cánh tay robot được khóa trạng thái (`teleop.enabled = False`). Mọi thao tác bóp cò hoặc vung tay trong chế độ này đều không phát lệnh di chuyển robot.

2. **Chế độ Cầm lái (Drive Mode):**
   - **Cách kích hoạt:** Bấm phím cứng **B** (tay phải) hoặc **Y** (tay trái) trên tay cầm Quest, hoặc bấm nút *Lock UI* trên thanh menu HUD, hoặc tích chọn checkbox *Bật Điều Khiển (Teleop)* trên màn hình máy tính.
   - **Phản hồi:** Tay cầm Quest phát xung rung xúc giác (Haptic pulse 120ms) xác nhận chuyển chế độ, thẻ trạng thái chuyển sang màu xanh lá (`Drive Mode`) và Viser hiển thị `Tracking`.
   - **Tương tác:** Giao diện HUD tự động chuyển sang chế độ xuyên thấu (`pointer-events: none`). Tia laser từ tay cầm đi xuyên qua bảng điều khiển, loại bỏ hoàn toàn rủi ro bấm nhầm nút trên web khi đang lái.
   - **Vận hành:** Cánh tay robot lập tức bám theo chuyển động tay người hoặc chuột máy tính với độ trễ thấp. Bấm lại phím **B/Y** hoặc bỏ chọn checkbox bất kỳ lúc nào để ngắt kết nối an toàn và quay về Edit Mode.

### 2.3. Quy Trình Vòng Lặp Điều Khiển Thời Gian Thực (Chu Kỳ 20Hz - 50ms)

Trong mỗi chu kỳ 50ms của `teleop_quest/main_quest.py`, hệ thống thực thi tuần tự 6 công đoạn:

1. **Bước 1: Thu thập & Lọc tín hiệu điều khiển (`DeltaPoseSource` / `WidgetPoseSource`):**
   - Khi dùng kính Quest: Cò trỏ (Trigger) phụ trách tịnh tiến XYZ (World-Centric), cò báng (Grip) phụ trách xoay hướng công cụ (Tool-Centric), nút A/X đóng mở ngàm kẹp độc lập. Cơ chế Smart Clutch gán mỏ neo mới (`reset_anchor()`) khi đổi trạng thái nút. Bộ lọc Soft Deadband 5mm và EMA ($\alpha = 0.35$) làm mịn quỹ đạo.
   - Khi dùng chuột: Tọa độ đích được cập nhật trực tiếp từ khối Target Widget trên không gian 3D của Viser.
   - Giới hạn Safe Box kẹp chặt tọa độ mục tiêu trong tầm với an toàn của UR3e ($X, Y \in [-0.5, 0.5]$m, $Z \in [-0.02, 0.60]$m).

2. **Bước 2: Giải ngược động học và tránh va chạm trên GPU (`CuRoboIKSolver`):**
   - Nhận tọa độ mục tiêu 6DoF từ Widget 3D, giải bài toán IK phi tuyến kết hợp kiểm tra va chạm trên GPU NVIDIA trong khoảng 3–5ms.
   - Sử dụng 64 hạt giống song song (seeds) và hệ số phạt lệch nhánh (`branch_penalty = 50.0`), đảm bảo nghiệm góc khớp trả về liên tục và không bị lật khớp cổ tay.

3. **Bước 3: Kiểm định an toàn và điểm kỳ dị (`SafetyGate` & `SingularityGuard`):**
   - Đánh giá ma trận Jacobian của cấu hình góc khớp mục tiêu, đo lường giá trị suy biến nhỏ nhất $\sigma_{min}$.
   - Nếu $\sigma_{min} < 0.035$ (tư thế tiến sát điểm kỳ dị), cổng an toàn lập tức chặn lệnh, phát xung rung cảnh báo về tay cầm VR và giữ robot đứng yên.
   - Khối Rate Limiter kiểm tra và kẹp biến thiên vận tốc khớp tối đa $\le 2.0$ rad/s để triệt tiêu hiện tượng rung giật động cơ.

4. **Bước 4: Thực thi lệnh chuyển động thời gian thực (`ServoJExecutor`):**
   - Đảm bảo giải phóng thanh ghi RTDE thông qua lệnh Dashboard `stop\n`.
   - Truyền gói dữ liệu 6 góc khớp mục tiêu $q_{target}$ qua giao thức RTDE (Cổng 30004) bằng lệnh `servoJ` với `lookahead_time = 0.04`s và `gain = 800`.
   - Robot phản hồi bám sát tức thì với độ trễ chuyển động thực tế dưới 10ms.

5. **Bước 5: Mô phỏng gắp nhặt & Ghi tập dữ liệu AI (`GraspSimulator` & `DataLogger`):**
   - Kiểm tra tương tác hình học giữa đầu ngàm kẹp và quả cà chua mục tiêu. Khi tiếp cận đúng phạm vi, vật thể được gắn dính vào đầu kẹp và gửi phản hồi Haptic thành công.
   - Khi cờ ghi hình kích hoạt (`ui.cb_record`), module `DataLogger` lưu trữ đồng bộ trạng thái góc khớp thực tế, vận tốc, tọa độ TCP, lệnh điều khiển, trạng thái kẹp và ảnh camera vào tệp HDF5 theo chu kỳ cố định 20Hz.

6. **Bước 6: Thu nhận hình ảnh camera cổ tay và truyền phát trực tiếp (`create_wrist_camera`):**
   - Thu nhận luồng hình ảnh thời gian thực 20 FPS từ camera ảo 3D hoặc camera vật lý thật gắn trên cổ tay (RGB 640x480).
   - Tích hợp lớp phủ OSD hiển thị trạng thái ngàm kẹp, nén định dạng JPEG chất lượng cao và phát trực tiếp tới cả sidebar Viser và WebXR Cockpit qua WebSocket và MJPEG.

---

## 3. Danh Sách Thư Viện & Môi Trường Cài Đặt

### Yêu Cầu Hệ Thống
- **Hệ điều hành:** Ubuntu 20.04 hoặc 22.04 LTS (x86_64).
- **Phần cứng:** NVIDIA GPU hỗ trợ CUDA 12.0+ (khuyến nghị RTX 3060 trở lên).
- **Bộ điều khiển Robot:** Trình giả lập URSim trên Docker hoặc Cánh tay UR3e thật (Cổng 29999 Dashboard, 30004 RTDE).

### Bảng Thư Viện Python

| Thư viện | Phiên bản | Vai trò trong hệ thống |
| :--- | :--- | :--- |
| `torch`, `torchvision` | 2.1.0+ (CUDA) | Tính toán ma trận Tensor GPU cho CuRobo và Jacobian singularity metric. |
| `curobo` | 0.8.0+ | Bộ giải IK và quy hoạch tránh va chạm trên GPU NVIDIA. |
| `viser` | 0.2.0+ | Máy chủ WebGL 3D Visualization, Gizmo tương tác và Web UI. |
| `ur_rtde` | 1.5.0+ | Giao tiếp thời gian thực RTDE với bộ điều khiển UR qua cổng 30004. |
| `opencv-python` | 4.8.0+ | Render và thu nhận ảnh camera cổ tay (640x480), nén MJPEG và overlay OSD. |
| `transforms3d` | 0.4.0+ | Chuyển đổi giữa Axis-Angle, Quaternion và ma trận quay. |
| `trimesh` | 4.0.0+ | Tải và phân tích mô hình lưới 3D robot và vật cản. |
| `h5py` | 3.9.0+ | Ghi và đọc tập dữ liệu AI Demonstration chuẩn HDF5. |
| `pyyaml` | 6.0+ | Đọc file cấu hình robot (`ur3e_custom.yml`) và cảnh vật cản (`obstacle_scene.yml`). |
| `websockets` | 12.0+ | Máy chủ truyền nhận dữ liệu 6DoF từ tay cầm WebXR với độ trễ thấp. |
| `cryptography` | 41.0+ | Tự động sinh chứng chỉ SSL/TLS cho máy chủ HTTPS WebXR. |
| `numpy`, `scipy` | 1.24+ | Tính toán đại số tuyến tính, phép quay SE(3) và bộ lọc làm mượt tín hiệu. |

### Lệnh Cài Đặt

```bash
# Tạo và kích hoạt môi trường ảo
python3 -m venv ~/curobo/.venv
source ~/curobo/.venv/bin/activate

# Cài đặt PyTorch với hỗ trợ CUDA (Ví dụ CUDA 12.1)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# Cài đặt các thư viện phụ trợ
pip install viser ur_rtde opencv-python transforms3d trimesh h5py pyyaml websockets cryptography numpy scipy

# Cài đặt NVIDIA CuRobo
pip install curobo --extra-index-url https://pypi.nvidia.com
```

---

## 4. Cấu Trúc Thư Mục Dự Án

```text
ur_ws/curobo_ursim/
├── main.py                          # Điểm khởi chạy chính của hệ thống
├── README.md                        # Tài liệu hướng dẫn sử dụng hệ thống
├── assets/
│   ├── ur3e_custom.urdf             # URDF UR3e mô tả link, khớp và tool0
│   ├── ur3e_custom.yml              # Cấu hình CuRobo kinematics, collision spheres và giới hạn khớp
│   └── obstacle_scene.yml           # Cấu hình mặt bàn và vách ngăn vật cản
├── config/
│   └── singularity.yaml             # Ngưỡng an toàn Jacobian singularity (sigma_min)
├── core/
│   ├── curobo_ursim_control.py      # Bộ giải CuRoboIK (GPU batched seeds, dashboard auto-stop)
│   ├── data_logger.py               # Module ghi Demonstration HDF5 ở tần số 20Hz
│   ├── verify_dataset.py            # Công cụ kiểm tra chất lượng tập dữ liệu HDF5
│   └── replay_episode.py            # Công cụ tái hiện quỹ đạo thao tác
├── safety/
│   ├── __init__.py                  # Export SingularityGuard
│   └── singularity_guard/
│       ├── guard_core.py            # Giám sát điểm kỳ dị thời gian thực
│       ├── jacobian_metrics.py      # Tính toán sigma_min từ ma trận Jacobian
│       └── calibration.py           # Cân chỉnh ngưỡng kỳ dị theo không gian làm việc
├── teleop/
│   └── teleoperation.py             # TeleoperationController, ServoJExecutor (Dashboard clean), SafetyGate
├── teleop_quest/
│   ├── main_quest.py                # Vòng lặp chính 20Hz tích hợp VR, Viser, Camera và GraspSim
│   ├── camera_driver.py             # Driver camera cổ tay hợp nhất: Camera ảo 3D & Camera vật lý thật
│   ├── synthetic_camera.py          # Render camera ảo cổ tay 640x480 (Eye-in-Hand)
│   ├── viser_ui.py                  # Giao diện Viser 3D Cockpit và khung xem trước camera
│   ├── grasp_simulator.py           # Mô phỏng gắp/thả quả cà chua 3D vào giỏ
│   ├── quest_server.py              # Server HTTPS (8443), WSS (8444) và MJPEG stream
│   ├── network_utils.py             # Dò tìm IP LAN và tự động sinh chứng chỉ SSL
│   ├── pose_sources.py              # WidgetPoseSource (đọc tọa độ Widget kèm bộ lọc EMA)
│   ├── delta_pose_source.py         # Chuyển đổi vi phân Quest sang UR3e base_link
│   ├── timeout_watchdog.py          # E-Stop và tự động thu hồi tọa độ khi mất tín hiệu
│   ├── home_planner.py              # Đưa robot về vị trí Home an toàn
│   └── static/
│       ├── index.html               # Giao diện WebXR Cockpit trên kính Quest
│       ├── main.js                  # Xử lý gamepad WebXR, luồng camera và rung Haptic
│       ├── style.css                # Style hiển thị phong cách HUD hiện đại
│       └── wrist_camera.jpg         # Bộ đệm khung hình camera phục vụ truyền phát
└── tests/
    ├── test_camera.py               # Kiểm thử render camera ảo ở các góc nhìn & giao diện GUI
    ├── test_simulated_pick_and_place.py # Kiểm thử tự động pick-place cà chua và xuất HDF5
    └── test_singularity_guard.py    # Kiểm thử tính toán Jacobian singularity metric (sigma_min)
```

---

## 5. Hướng Dẫn Vận Hành

### Bước 1: Khởi động URSim hoặc Robot thật
Nếu sử dụng trình giả lập URSim:
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```
*(Nếu sử dụng robot UR3e thật, xem chi tiết cấu hình tại Mục 6)*

### Bước 2: Chạy chương trình chính
Kích hoạt môi trường và khởi chạy với các tham số tương ứng:

```bash
cd /home/nguyen/ur_ws/curobo_ursim
source ~/curobo/.venv/bin/activate

# 1. Chạy với URSim mô phỏng (Camera ảo 3D - Mặc định):
python3 main.py

# 2. Chạy với Robot UR3e thật (Chỉ định IP robot):
python3 main.py --robot-ip 192.168.1.100

# 3. Chạy với Camera vật lý thật gắn trên cổ tay (USB Camera / RealSense):
python3 main.py --robot-ip 192.168.1.100 --camera real --camera-device 0
```

Các tham số dòng lệnh được hỗ trợ:
- `--robot-ip`: Địa chỉ IP của Robot thật hoặc URSim (mặc định: `127.0.0.1`).
- `--camera`: Nguồn camera cổ tay: `synthetic` (mô phỏng) hoặc `real` (camera thật) (mặc định: `synthetic`).
- `--camera-device`: Chỉ số Device ID của camera thật (mặc định: `0`).

Hệ thống sẽ hiển thị các cổng truy cập:
- Viser Dashboard: `http://localhost:8080`
- WebXR Quest: `https://<IP_LAN>:8443` (Data WebSocket tại cổng `8444`, WS Local tại cổng `8445`, luồng video MJPEG tại `/video_feed`)

### Bước 3: Điều khiển bằng Viser Web UI (Máy tính)
1. Mở trình duyệt web trên máy tính và truy cập `http://localhost:8080`.
2. Kiểm tra mô hình robot 3D hiển thị khớp đúng với trạng thái thực tế của robot.
3. **Thao tác trên Bảng điều khiển Lưới 2x2:**
   - **Bật Điều Khiển (Teleop):** Tích chọn để kích hoạt cánh tay robot bám theo mục tiêu (Nhãn `Status` chuyển sang `Tracking`).
   - **Gripper (Bóp Kẹp):** Tích chọn để đóng ngàm kẹp gắp quả cà chua (hoặc bỏ chọn để nhả vật thể).
   - **Record AI Episode:** Tích chọn để bắt đầu ghi chuỗi thao tác phục vụ huấn luyện mô hình Robot AI.
   - **Hủy Episode (Discard):** Tích chọn khi thực hiện thao tác hỏng; hệ thống sẽ hủy bỏ dữ liệu tập hiện tại và tự động nhả checkbox sau 200ms.
4. **Điều khiển chuyển động:** Dùng chuột nhấp và kéo các trục mũi tên hoặc vòng xoay của khối Target Widget 3D: Cánh tay robot sẽ chuyển động bám theo tọa độ đích của chuột trong thời gian thực.
5. **Trở về vị trí an toàn:** Bấm nút **Go to Home** khi cần đưa robot về vị trí mặc định an toàn. Quỹ đạo chuyển động được điều phối mượt mà không rung giật (MoveJ 500Hz), không glitch hình ảnh và tự động kích hoạt lại chế độ Teleop sau khi robot về đích.

### Bước 4: Điều khiển bằng Kính Meta Quest 3S (WebXR)
1. Đảm bảo kính Meta Quest 3S và máy tính kết nối cùng mạng Wi-Fi LAN.
2. Trên trình duyệt Meta Quest Browser:
   - Truy cập `https://<IP_LAN>:8444` -> Chọn **Advanced** -> **Proceed to unsafe** (chấp nhận chứng chỉ SSL tự sinh cho kênh WebSocket).
   - Truy cập `https://<IP_LAN>:8443` -> Chọn **Advanced** -> **Proceed to unsafe** (chấp nhận chứng chỉ SSL cho kênh WebXR).
3. Bấm **START IMMERSIVE VR** trên màn hình chào mừng.
4. **Các thao tác tay cầm:**
   - **Bóp Cò (Trigger):** Tịnh tiến đầu robot trong không gian (trái/phải, tiến/lùi, lên/xuống).
   - **Bóp Grip (Nút hông):** Xoay cổ tay robot theo hướng tay cầm.
   - **Nút B / Y trên tay cầm:** Phím nóng chuyển đổi tức thì giữa Edit Mode (tương tác UI) và Drive Mode (lái robot) kèm phản hồi rung Haptic.
   - **Nút A / X trên tay cầm:** Bật/tắt ngàm kẹp Gripper để gắp hoặc thả quả cà chua.
   - **Khung Camera Stream:** Hiển thị góc nhìn trực tiếp từ đầu tay máy robot theo thời gian thực.

---

## 6. Hướng Dẫn Triển Khai Trên Cánh Tay Robot UR3e Thực Tế

### 6.1. Kết Nối Phần Cứng & Cấu Hình Mạng LAN
1. **Cắm cáp mạng LAN:** Kết nối trực tiếp từ cổng mạng của máy tính (PC chạy code) vào cổng mạng trên hộp điều khiển UR Control Box (hoặc qua Switch mạng nội bộ).
2. **Cài đặt IP tĩnh trên Robot (qua màn hình Teach Pendant - PolyScope):**
   - Vào menu: **Settings** -> **System** -> **Network**.
   - Chọn **Static Address**:
     - IP Address: ví dụ `192.168.1.100`
     - Subnet Mask: `255.255.255.0`
     - Default Gateway: `192.168.1.1`
   - Bấm **Apply** và xác nhận trạng thái mạng hiển thị *Network is connected*.
3. **Cài đặt IP trên máy tính điều khiển (PC):**
   - Cài đặt IP máy tính cùng dải mạng: ví dụ `192.168.1.50`, Subnet Mask `255.255.255.0`.
   - Kiểm tra kết nối mạng từ terminal máy tính:
     ```bash
     ping -c 3 192.168.1.100
     ```

### 6.2. Thiết Lập Trên Tay Dạy PolyScope (Teach Pendant)
1. **Bật chế độ Remote Control:**
   - Vào **Settings** -> **System** -> **Remote Control** -> Chọn **Enable**.
   - Ở góc trên cùng bên phải màn hình PolyScope, chuyển chế độ từ **Local** sang **Remote Control**.
2. **Cài đặt Tải trọng Đầu Gắp (Payload & Center of Gravity - CoG):**
   - Vào **Installation** -> **General** -> **Payload**.
   - Cài đặt chính xác khối lượng ngàm kẹp + camera (ví dụ Robotiq 2F-85 + RealSense $\approx 1.1$kg) và tọa độ trọng tâm CoG.
   - *Lưu ý:* Khai báo đúng tải trọng giúp thuật toán `servoJ` vận hành êm ái, bám quỹ đạo chuẩn và tránh lỗi quá dòng động cơ.
3. **Bật nguồn và Nhả phanh:**
   - Kích hoạt nguồn (Power On) và bấm **Release Brakes** để robot vào trạng thái sẵn sàng hoạt động (màn hình hiển thị màu xanh lá).
4. **Hạ thanh trượt Tốc độ (Speed Slider):**
   - Trong lần đầu vận hành, kéo thanh trượt tốc độ ở góc dưới màn hình PolyScope xuống mức **10% – 20%** để đảm bảo an toàn quan sát.
5. **Cầm sẵn nút dừng khẩn cấp (Emergency Stop):**
   - Luôn cầm Teach Pendant trên tay và sẵn sàng ấn nút E-Stop màu đỏ nếu robot có hiện tượng di chuyển sai hướng.

### 6.3. Cấu Hình Địa Chỉ IP Trong Mã Nguồn
Hệ thống hỗ trợ cấu hình IP linh hoạt không cần chỉnh sửa code:

- **Cách 1: Sử dụng tham số dòng lệnh CLI (Khuyến nghị hàng đầu):**
  ```bash
  python3 main.py --robot-ip 192.168.1.100
  ```
- **Cách 2: Sử dụng biến môi trường:**
  ```bash
  export ROBOT_IP=192.168.1.100
  python3 main.py
  ```
- **Cách 3: Cập nhật mặc định trong file mã nguồn:**
  - `core/curobo_ursim_control.py`: Cập nhật `ROBOT_IP = "192.168.1.100"`.
  - `teleop/teleoperation.py`: Cập nhật `ROBOT_IP = "192.168.1.100"` trong `ServoJExecutor`.
  - `config/singularity.yaml`: Cập nhật `rtde_ip: 192.168.1.100`.

### 6.4. Khai Báo Không Gian Vật Cản Thực Tế (Collision Scene)
File `assets/obstacle_scene.yml` định nghĩa ranh giới an toàn cho thuật toán GPU CuRobo:
1. Đo khoảng cách thực tế từ gốc robot (`base_link`) tới mặt bàn làm việc (thường $Z = 0.0$ hoặc $-0.02$m).
2. Nếu có chướng ngại vật cố định xung quanh bàn, đo kích thước `[dx, dy, dz]` và tọa độ tâm `[x, y, z]`, sau đó khai báo vào `assets/obstacle_scene.yml` để CuRobo tự động né tránh va chạm trong thời gian thực.

### 6.5. Tích Hợp Ngàm Kẹp Vật Lý & Camera Thật
1. **Ngàm kẹp thật (Ví dụ: Robotiq 2F-85 / 2F-140 / OnRobot):**
   - Kết nối qua giao tiếp RS-485 / Modbus RTU tại cổng Tool Flange của UR3e hoặc cổng USB máy tính.
   - Trong `teleop_quest/main_quest.py`, tích hợp điều khiển driver ngàm kẹp thật mỗi khi trạng thái nút `is_grip_pressed` thay đổi.
2. **Camera thật (Ví dụ: Intel RealSense D435i / D405 / USB Camera gắn cổ tay):**
   - Kết nối camera qua cáp USB vào máy tính.
   - Chạy với cờ `--camera real --camera-device 0`. Driver `RealWristCamera` trong `camera_driver.py` sẽ tự động đọc luồng ảnh màu RGB 640x480 thực tế, truyền về WebXR Cockpit và lưu vào tập dữ liệu HDF5.

### 6.6. Quy Trình Vận Hành Thử Nghiệm An Toàn Lần Đầu
1. **Khởi chạy chương trình:**
   ```bash
   cd /home/nguyen/ur_ws/curobo_ursim
   source ~/curobo/.venv/bin/activate
   python3 main.py --robot-ip 192.168.1.100
   ```
2. **Kiểm tra trạng thái kết nối:**
   - Đảm bảo terminal hiển thị `[Setup] Connected` và `[ServoJ] RTDEControl connected: True`.
   - Mở giao diện `http://localhost:8080`, kiểm tra góc khớp của mô hình 3D trên màn hình khớp hoàn toàn với vị trí robot thực tế.
3. **Thử nghiệm chuyển động biên độ nhỏ:**
   - Giữ nguyên tốc độ PolyScope ở mức **10%**.
   - Tích chọn ô **Bật Điều Khiển (Teleop)**.
   - Sử dụng chuột kéo nhẹ Target Widget trên Viser hoặc dùng tay cầm Meta Quest dịch chuyển nhẹ khoảng **1 – 2 cm** theo trục Z.
   - Xác nhận cánh tay robot thật di chuyển đúng hướng, êm ái và không bị giật cục trước khi tăng dần thanh tốc độ lên mức bình thường.

---

## 7. Kiểm Thử Hệ Thống & Quản Lý Dữ Liệu

### 1. Kiểm thử tự động Gắp Cà chua & Xuất HDF5
Chạy kiểm thử toàn bộ chu trình gắp thả và lưu dữ liệu:
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_simulated_pick_and_place.py
```
Kết quả kiểm tra:
- Tập tin Demonstration HDF5 được tạo tại: `logs/ai_training_data/test_grasp_sim/episode_X.hdf5`.
- Ảnh chụp camera ảo cổ tay tại thời điểm hoàn thành: `logs/simulated_wrist_camera_view.jpg`.

### 2. Kiểm thử Camera ảo Cổ tay (Synthetic Wrist Camera)
Kiểm tra khả năng render camera cổ tay ở 4 góc nhìn khác nhau:
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_camera.py
```
Các file ảnh kết xuất sẽ được lưu vào thư mục `logs/` (`test_cam_center.jpg`, `test_cam_left.jpg`,...).

Nếu muốn kiểm tra trực quan bằng thanh trượt trên giao diện Web Viser:
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_camera.py --gui
```
Sau đó mở trình duyệt tại `http://localhost:8089` để kéo tọa độ quả cà chua trực tiếp.

### 3. Kiểm thử Lớp Bảo Vệ Điểm Kỳ Dị (Singularity Guard)
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_singularity_guard.py
```
Kiểm tra tính toán ma trận Jacobian metric ($\sigma_{min}$) trên GPU ở cả tư thế an toàn lẫn tư thế duỗi thẳng cổ tay ($q_5 \approx 0$).

### 4. Kiểm định chất lượng Tập Dữ Liệu (Sanity Check)
```bash
/home/nguyen/curobo/.venv/bin/python core/verify_dataset.py
```
Kiểm tra toàn vẹn các episode: xác minh không có giá trị `NaN`/`Inf`, kiểm tra độ lệch chu kỳ lấy mẫu `dt`, kiểm tra số frame và độ phân giải ảnh camera.

### 5. Tái lập Quỹ đạo Demonstration (Replay)
- **Offline Dry-Run (kiểm tra quỹ đạo an toàn trên Viser, không gửi lệnh sang robot):**
  ```bash
  /home/nguyen/curobo/.venv/bin/python core/replay_episode.py --file logs/ai_training_data/test_grasp_sim/episode_0.hdf5
  ```
- **Live Replay (gửi lệnh điều khiển trực tiếp tới URSim / Robot thật):**
  ```bash
  /home/nguyen/curobo/.venv/bin/python core/replay_episode.py --file logs/ai_training_data/test_grasp_sim/episode_0.hdf5 --execute --ip 127.0.0.1
  ```

---

## 8. Thông Số Kỹ Thuật Điều Khiển Chính

| Tham số | Giá trị | Giải thích kỹ thuật |
| :--- | :--- | :--- |
| **Tần số vòng lặp chính** | 20 Hz (`dt = 0.05`s) | Đồng bộ giữa WebXR, Viser, CuRobo và RTDE servoJ. |
| **Tần số Camera Stream** | 20 FPS | Truyền phát luồng hình ảnh thời gian thực tới Viser và WebXR Cockpit. |
| **ServoJ Lookahead Time** | `0.04` s | Nội suy chuyển động mượt mà, loại bỏ độ trễ và độ rơ giật. |
| **ServoJ Gain** | `800` | Độ cứng bám quỹ đạo cao, giúp đầu robot bám sát vị trí tay người. |
| **Vận tốc khớp tối đa (Rate Limiter)** | `2.0` rad/s | Kẹp gia tốc và vận tốc khớp để chống lỗi Protective Stop trên UR. |
| **Ngưỡng phát hiện vặn xoắn khớp** | `1.5` rad | Chặn những bước nhảy đột ngột do đảo nghiệm IK gần điểm kỳ dị. |
| **Giới hạn khớp khuỷu (elbow_joint)** | `[-2.80, 2.80]` rad | Cho phép gập sâu tự nhiên (~160 độ) khi hạ thấp gắp cà chua, CuRobo tự quản lý va chạm. |
| **Giới hạn khớp cổ tay (wrist_1, wrist_2)** | `[-2π, 2π]` rad | Duy trì dải quay đầy đủ của URDF, loại bỏ triệt để hiện tượng đảo nhánh vặn xoắn. |
| **Giới hạn khớp nâng vai (shoulder_lift)** | `[-π, 0.05]` rad | Chống chúi cắm bắp tay xuống mặt bàn làm việc. |
| **Thuật toán quy hoạch Home** | URScript `movej` | Chạy trực tiếp ở tần số 500Hz trên URControl (`a=0.8` rad/s², `v=0.5` rad/s), triệt tiêu rung giật. |
| **Bán kính bo cung waypoints Home** | `0.025` m | Bo cong góc mượt mà qua các điểm trung gian của CuRobo MotionPlanner. |
| **Khôi phục Teleop sau khi về Home** | Tự động (Zero-Click) | Tự động kết nối lại RTDE và reset mỏ neo tay cầm VR ngay khi robot về đích. |
| **Số hạt giống CuRobo IK** | 64 seeds | Nâng cao xác suất tìm nghiệm liên tục trên GPU. |
| **Hệ số phạt lệch nhánh IK** | `50.0` (khi lệch $> 0.8$ rad) | Giữ robot luôn ở nhánh động học đồng nhất với vị trí hiện tại. |
| **Deadband tĩnh tay cầm VR** | 5 mm (`0.005` m) | Loại bỏ rung tay sinh lý với cơ chế trừ mềm (Soft Deadband). |
| **Độ lệch tay cầm tối đa (max_delta_m)** | 0.25 m | Phạm vi kéo tay mỗi nhịp bóp cò, tạo cảm giác thoải mái. |
| **Bộ lọc làm mượt EMA (Alpha)** | 0.35 | Cân bằng giữa độ nhạy phản hồi và chống nhiễu jitter. |
| **Ngưỡng thời gian Auto-Snap** | 1.2 s | Tự động kéo XYZ về đầu tay máy và reset seed khi bị kẹt tư thế hoặc vướng vật cản. |
