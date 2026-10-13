# UR3e Teleoperation qua Meta Quest 3S & CuRobo GPU IK

Hệ thống điều khiển viễn thao thời gian thực (Real-time Teleoperation) cho cánh tay robot công nghiệp Universal Robots UR3e sử dụng kính thực tế ảo Meta Quest 3S (WebXR) và giao diện Web 3D Viser. Hệ thống giải bài toán động học nghịch (Inverse Kinematics) và kiểm tra va chạm hoàn toàn trên GPU bằng NVIDIA CuRobo, đồng thời hỗ trợ thu thập dữ liệu trình diễn (Demonstrations) chuẩn HDF5 phục vụ huấn luyện mô hình Robot AI (Imitation Learning, Diffusion Policy, ACT).

---

## 1. Tính Năng Nổi Bật

1. **Điều khiển Viễn Thao Thời Gian Thực (20Hz RTDE servoJ):**
   - Đồng bộ chuyển động 6DoF từ tay cầm Meta Quest 3S sang robot UR3e với độ trễ cực thấp.
   - Bộ thực thi `ServoJExecutor` sử dụng tham số tối ưu cho chu kỳ 20Hz: `dt = 0.05`s, `lookahead_time = 0.04`s, `gain = 800`.
   - Chế độ điều khiển phân tách chức năng:
     - **Chỉ bóp Cò (Trigger):** Tịnh tiến không gian (World-Centric mapping, khóa góc xoay).
     - **Chỉ bóp Grip (Nút hông):** Xoay đầu hút/gắp theo hệ tọa độ đầu công cụ (Tool-Centric mapping, khóa tịnh tiến).
     - **Bóp cả Trigger + Grip:** Chuyển động tự do 6DoF đồng thời.
     - **Cơ chế Smart Clutch:** Tự động chốt mỏ neo mới (Reset Anchor) ngay khi thay đổi nút bấm, triệt tiêu hoàn toàn lỗi nhảy vọt tọa độ (teleport).

2. **Động Học Nghịch Tránh Va Chạm Trên GPU (NVIDIA CuRobo 0.8):**
   - Giải bài toán IK thời gian thực trên GPU CUDA (3–6ms), tự động kiểm tra va chạm với bàn làm việc, vách ngăn và cụm camera/ngàm kẹp.
   - Vùng an toàn tinh gọn: `collision_sphere_buffer: 0.005` (5mm) và các quả cầu ngàm kẹp/camera bán kính 2.2cm.
   - Cơ chế chống đảo nhánh IK (Branch Consistency Penalty): Bổ sung hệ số phạt lệch nhánh (`branch_penalty = 50.0` khi lệch $> 0.8$ rad) kết hợp 64 hạt giống (seeds) và 16 nghiệm trả về, loại bỏ triệt để lỗi lật khớp cổ tay (Lỗi vặn xoắn).

3. **Luồng Stream Camera Cổ Tay Trực Tiếp Trong VR Dashboard:**
   - Khung "CAMERA STREAM" trên giao diện WebXR Cockpit hiển thị góc nhìn trực tiếp từ camera ảo gắn ở cổ tay robot (`SyntheticWristCamera`, độ phân giải 640x480, FOV 65 độ).
   - Truyền phát khung hình qua kênh WebSocket (`wrist_cam`) và endpoint HTTP `/wrist_camera.jpg`.

4. **Mô Phỏng Gắp/Thả 3D (Interactive Grasp Simulator):**
   - Mô phỏng cơ chế hút dính vật thể quả cà chua 3D khi ngàm tiếp cận đúng vị trí (khoảng cách 3D $< 9.5$cm hoặc lệch XY $< 5.5$cm kèm độ cao Z $< 13$cm).
   - Tự động nhận diện khi đưa vật thể vào khay chứa và ghi nhận sự kiện hoàn thành nhiệm vụ.
   - Nút bấm Gripper độc lập (nút A/X trên tay cầm hoặc nút bấm trên Web UI) không bị xung đột với nút Grip xoay cổ tay.

5. **Cơ Chế Tự Động Thu Hồi Tọa Độ XYZ (Auto-Snap on Stuck):**
   - Khi robot gặp tình trạng kẹt tư thế (vặn xoắn khớp) hoặc vướng vật cản kéo dài quá 1.2 giây (`snap_threshold = 1.2`), hệ thống tự động rung phản hồi Haptic tới tay cầm và giật tọa độ Widget 3D về đúng đầu tay máy robot thật (`fk_res`), giúp người vận hành tiếp tục điều khiển trở lại ngay lập tức.

6. **Thu Thập & Quản Lý Dữ Liệu AI Chuẩn HDF5:**
   - Ghi lại chuỗi thao tác đầy đủ: góc khớp (`qpos`), vận tốc (`qvel`), lệnh điều khiển (`action`), trạng thái ngàm kẹp (`gripper`), ảnh camera cổ tay (`images/wrist_camera`), timestamp ở tần số 20Hz.
   - Tự động đánh dấu kết quả thành công/thất bại (`success=True/False`).

7. **Bộ Công Cụ Kiểm Định & Tái Lập Quỹ Đạo (Data Sanity & Replay):**
   - `verify_dataset.py`: Quét toàn bộ file HDF5 kiểm tra giá trị NaN/Inf, độ lệch chu kỳ lấy mẫu (dt jitter), kích thước tensor hình ảnh.
   - `replay_episode.py`: Tái hiện lại quỹ đạo trình diễn ở chế độ Offline Dry-Run hoặc Live Replay gửi lệnh trực tiếp sang robot.

8. **Các Lớp Bảo Vệ An Toàn (Safety Gate & Watchdog):**
   - **Chống lật khớp cổ tay (Anti-Twist Guard):** Chặn các bước nhảy góc khớp đột ngột (`max_jump > 1.5` rad).
   - **Giới hạn tốc độ (Rate Limiter):** Kẹp vận tốc góc khớp tối đa $\le 2.0$ rad/s để bảo vệ động cơ robot vật lý.
   - **Giám sát mất kết nối (Timeout Watchdog):** Tự động ngắt ServoJ và thu hồi Gizmo về robot nếu mất tín hiệu VR quá 2 giây.

---

## 2. Danh Sách Thư Viện & Môi Trường Cài Đặt

### Yêu Cầu Hệ Thống
- **Hệ điều hành:** Ubuntu 20.04 hoặc 22.04 LTS (x86_64).
- **Phần cứng:** NVIDIA GPU hỗ trợ CUDA 12.0+ (khuyến nghị RTX 3060 trở lên).
- **Trình giả lập URSim:** UR3e chạy trên Docker hoặc ROS 2 driver (Port 30004 cho RTDE Data, Port 30002 cho script/dashboard).

### Bảng Thư Viện Python

| Thư viện | Phiên bản | Vai trò trong hệ thống |
| :--- | :--- | :--- |
| `torch`, `torchvision` | 2.1.0+ (CUDA) | Tính toán ma trận Tensor GPU cho CuRobo và Jacobian singularity metric. |
| `curobo` | 0.8.0+ | Bộ giải IK và quy hoạch tránh va chạm trên GPU NVIDIA. |
| `viser` | 0.2.0+ | Máy chủ WebGL 3D Visualization, Gizmo tương tác và Web UI. |
| `ur_rtde` | 1.5.0+ | Giao tiếp thời gian thực RTDE với bộ điều khiển UR qua cổng 30004. |
| `opencv-python` | 4.8.0+ | Render và mã hóa JPEG luồng camera ảo cổ tay (640x480). |
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

## 3. Cấu Trúc Thư Mục Dự Án

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
│   ├── curobo_ursim_control.py      # Bộ giải CuRoboIK (GPU batched seeds, branch penalty)
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
│   └── teleoperation.py             # TeleoperationController, ServoJExecutor, SafetyGate
├── teleop_quest/
│   ├── main_quest.py                # Vòng lặp chính 20Hz tích hợp VR, Viser, Camera và GraspSim
│   ├── viser_ui.py                  # Giao diện Viser 3D Cockpit và khung xem trước camera
│   ├── grasp_simulator.py           # Mô phỏng gắp/thả quả cà chua 3D vào giỏ
│   ├── synthetic_camera.py          # Render camera ảo cổ tay 640x480 (Eye-in-Hand)
│   ├── quest_server.py              # Server HTTPS (8443) và WSS (8444) cho WebXR Quest
│   ├── network_utils.py             # Dò tìm IP LAN và tự động sinh chứng chỉ SSL
│   ├── pose_sources.py              # WidgetPoseSource (đọc tọa độ Widget kèm bộ lọc EMA)
│   ├── delta_pose_source.py         # Chuyển đổi vi phân Quest sang UR3e base_link
│   ├── timeout_watchdog.py          # E-Stop và tự động thu hồi tọa độ khi mất tín hiệu
│   ├── home_planner.py              # Đưa robot về vị trí Home an toàn
│   └── static/
│       ├── index.html               # Giao diện WebXR Cockpit trên kính Quest
│       ├── main.js                  # Xử lý gamepad WebXR, luồng camera ảo và rung Haptic
│       └── style.css                # Style hiển thị phong cách HUD hiện đại
└── tests/
    ├── test_camera.py               # Kiểm thử render camera ảo ở các góc nhìn & giao diện GUI
    ├── test_simulated_pick_and_place.py # Kiểm thử tự động pick-place cà chua và xuất HDF5
    └── test_singularity_guard.py    # Kiểm thử tính toán Jacobian singularity metric (sigma_min)
```

---

## 4. Hướng Dẫn Vận Hành

### Bước 1: Khởi động URSim hoặc Robot thật
Nếu sử dụng trình giả lập URSim:
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```
*Ghi chú:* Mặc định hệ thống kết nối tới địa chỉ `127.0.0.1`. Nếu kết nối robot UR3e thật qua mạng LAN, cập nhật biến `ROBOT_IP` trong mã nguồn tương ứng.

### Bước 2: Chạy chương trình chính
```bash
cd /home/nguyen/ur_ws/curobo_ursim
source ~/curobo/.venv/bin/activate
python3 main.py
```
Hệ thống sẽ hiển thị các cổng truy cập:
- Viser Dashboard: `http://localhost:8080`
- WebXR Quest: `https://<IP_LAN>:8443` (Data WebSocket tại cổng `8444`)

### Bước 3: Điều khiển bằng Viser Web UI (Máy tính)
- Mở trình duyệt trên máy tính và truy cập `http://localhost:8080`.
- Tích chọn **Bật Điều Khiển (Teleop)** để cho phép robot bám theo Gizmo 3D.
- Tích chọn **Gripper (Bóp Kẹp)** để bật lực hút gắp vật thể.
- Tích chọn **Record AI Episode** để bắt đầu ghi dữ liệu sang file HDF5.
- Bấm **Go to Home** khi cần đưa robot về vị trí mặc định an toàn.

### Bước 4: Điều khiển bằng Kính Meta Quest 3S (WebXR)
1. Đảm bảo kính Meta Quest 3S và máy tính kết nối cùng mạng Wi-Fi LAN.
2. Trên trình duyệt Meta Quest Browser:
   - Truy cập `https://<IP_LAN>:8444` $\rightarrow$ Chọn **Advanced** $\rightarrow$ **Proceed to unsafe** (chấp nhận chứng chỉ SSL tự sinh cho kênh WebSocket).
   - Truy cập `https://<IP_LAN>:8443` $\rightarrow$ Chọn **Advanced** $\rightarrow$ **Proceed to unsafe** (chấp nhận chứng chỉ SSL cho kênh WebXR).
3. Bấm **START IMMERSIVE VR** trên màn hình chào mừng.
4. **Các thao tác tay cầm:**
   - **Bóp Cò (Trigger):** Tịnh tiến đầu robot trong không gian (trái/phải, tiến/lùi, lên/xuống).
   - **Bóp Grip (Nút hông):** Xoay cổ tay robot theo hướng tay cầm.
   - **Nút A / X trên tay cầm:** Bật/tắt ngàm kẹp Gripper để gắp hoặc thả quả cà chua.
   - **Khung Camera Stream:** Hiển thị góc nhìn trực tiếp từ đầu tay máy robot theo thời gian thực.

---

## 5. Kiểm Thử Hệ Thống & Quản Lý Dữ Liệu

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

## 6. Thông Số Kỹ Thuật Điều Khiển Chính

| Tham số | Giá trị | Giải thích kỹ thuật |
| :--- | :--- | :--- |
| **Tần số vòng lặp chính** | 20 Hz (`dt = 0.05`s) | Đồng bộ giữa WebXR, Viser, CuRobo và RTDE servoJ. |
| **ServoJ Lookahead Time** | `0.04` s | Nội suy chuyển động mượt mà, loại bỏ độ trễ và độ rơ giật. |
| **ServoJ Gain** | `800` | Độ cứng bám quỹ đạo cao, giúp đầu robot bám sát vị trí tay người. |
| **Vận tốc khớp tối đa (Rate Limiter)** | `2.0` rad/s | Kẹp gia tốc và vận tốc khớp để chống lỗi Protective Stop trên UR. |
| **Ngưỡng phát hiện vặn xoắn khớp** | `1.5` rad | Chặn những bước nhảy đột ngột do đảo nghiệm IK gần điểm kỳ dị. |
| **Số hạt giống CuRobo IK** | 64 seeds | Nâng cao xác suất tìm nghiệm liên tục trên GPU. |
| **Hệ số phạt lệch nhánh IK** | `50.0` (khi lệch $> 0.8$ rad) | Giữ robot luôn ở nhánh động học đồng nhất với vị trí hiện tại. |
| **Deadband tĩnh tay cầm VR** | 5 mm (`0.005` m) | Loại bỏ rung tay sinh lý với cơ chế trừ mềm (Soft Deadband). |
| **Độ lệch tay cầm tối đa (max_delta_m)** | 0.25 m | Phạm vi kéo tay mỗi nhịp bóp cò, tạo cảm giác thoải mái. |
| **Bộ lọc làm mượt EMA (Alpha)** | 0.35 | Cân bằng giữa độ nhạy phản hồi và chống nhiễu jitter. |
| **Ngưỡng thời gian Auto-Snap** | 1.2 s | Tự động kéo XYZ về đầu tay máy khi bị kẹt tư thế hoặc vướng vật cản. |
