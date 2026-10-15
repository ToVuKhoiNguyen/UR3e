# UR3e Teleoperation qua Meta Quest 3S, Viser 3D & CuRobo GPU IK

Hệ thống điều khiển viễn thao thời gian thực (Real-time Teleoperation) cho cánh tay robot công nghiệp Universal Robots UR3e sử dụng kính thực tế ảo Meta Quest 3S (WebXR) và chuột trên giao diện Web 3D Viser. Hệ thống giải bài toán động học nghịch (Inverse Kinematics) và kiểm tra va chạm hoàn toàn trên GPU bằng NVIDIA CuRobo, đồng thời hỗ trợ thu thập dữ liệu trình diễn (Demonstrations) chuẩn HDF5 phục vụ huấn luyện mô hình Robot AI (Imitation Learning, Diffusion Policy, ACT).

---

## 1. Tính Năng Nổi Bật

1. **Điều khiển Viễn Thao Thời Gian Thực (20Hz RTDE servoJ & Tách Luồng Camera):**
   - Đồng bộ chuyển động 6DoF từ tay cầm Meta Quest 3S hoặc chuột máy tính (Viser 3D Transform Controls Target Widget) sang robot UR3e với độ trễ cực thấp.
   - Bộ thực thi `ServoJExecutor` sử dụng tham số tối ưu chuẩn công nghiệp: `dt = 0.05`s, `lookahead_time = 0.12`s, `gain = 250`.
   - Cơ chế **Scaled Vector Rate Limiter** co giãn đồng bộ cả 6 khớp khi vượt ngưỡng vận tốc, bảo toàn 100% hướng vector trong không gian Descartes và loại bỏ hoàn toàn hiện tượng méo quỹ đạo/giật cục.
   - Kiến trúc đa luồng (`CameraStreamPipeline`): Tách riêng luồng render và truyền stream camera cổ tay, triệt tiêu hoàn toàn jitter timing cho vòng lặp điều khiển RTDE 20Hz.
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

4. **Mô Phỏng Gắp/Thả 3D & Cơ Chế Toggle Gripper:**
   - Mô phỏng cơ chế hút dính vật thể quả cà chua 3D khi ngàm tiếp cận đúng vị trí (khoảng cách 3D $< 9.5$cm hoặc lệch XY $< 5.5$cm kèm độ cao Z $< 13$cm).
   - Tự động nhận diện khi đưa vật thể vào khay chứa và ghi nhận sự kiện hoàn thành nhiệm vụ (`PLACE_SUCCESS`).
   - **Cơ chế Toggle Gripper (Nhấn Nhả Chuyển Trạng Thái):** Người dùng chỉ cần nhấn 1 lần nút A (tay phải) hoặc X (tay trái) để kẹp/hút vật thể, và nhấn lại lần nữa để thả vật thể. Không cần phải giữ chặt ngón tay liên tục, giảm mỏi tay tối đa và có rung haptic xác nhận mỗi lần chuyển trạng thái.

5. **Khóa Trục Xoay Cổ Tay Thông Minh (Soft Rotation Snapping):**
   - Giải quyết triệt để hiện tượng quay đa trục ký sinh khi gập hoặc chúc cổ tay (lỗi vô thức lắc ngang Yaw hoặc xoắn Roll khi đang muốn gập Pitch).
   - Áp dụng bộ lọc đường cong mềm tỷ lệ ba vùng theo độ lớn vector xoay:
     - **Tỷ lệ xoay phụ $< 25\%$:** Triệt tiêu hoàn toàn góc xoay phụ về 0, khóa chuyển động quay chuẩn xác trên một trục duy nhất.
     - **Tỷ lệ xoay phụ $25\% - 40\%$:** Làm mềm chuyển tiếp tuyến tính, không gây giật góc.
     - **Tỷ lệ xoay phụ $> 40\%$:** Cho phép quay tự do 3D khi người dùng chủ động vặn xoay tổ hợp nhiều hướng.

6. **Tự Động Kích Hoạt Docker URSim & Tiến Trình Nền ADB Watcher:**
   - Khi khởi động chế độ mô phỏng (`127.0.0.1`), hệ thống tự động kiểm tra cổng 29999. Nếu URSim chưa bật, chương trình tự động chạy `docker start ursim` (hoặc khởi chạy kịch bản ROS 2 URSim) mà người dùng không cần mở thêm terminal phụ.
   - Tích hợp tiến trình ngầm (`start_adb_watcher`) tự động kiểm tra cáp USB nối với kính Meta Quest mỗi 2.5 giây. Khi phát hiện kính đã cắm, hệ thống tự động chạy lệnh reverse toàn bộ cổng (`8443`, `8444`, `8445`, `8080`) vào `localhost`.

7. **Cơ Chế Tự Động Thu Hồi Tọa Độ XYZ (Auto-Snap on Stuck) & Reset Seed Thoát Kẹt:**
   - Khi robot gặp tình trạng kẹt tư thế (vặn xoắn khớp) hoặc vướng vật cản kéo dài quá 1.2 giây (`snap_threshold = 1.2`), hệ thống tự động rung phản hồi Haptic tới tay cầm và giật tọa độ Widget 3D về đúng đầu tay máy robot thật (`fk_res`).
   - Tự động gán lại hạt giống giải IK `teleop._last_valid_q = list(q_real)`, giúp thuật toán lập tức thoát kẹt và sẵn sàng nhận chuyển động mới từ người vận hành.

8. **Tự Động Dọn Dẹp Kịch Bản Ngầm & Giải Phóng Thanh Ghi RTDE:**
   - Bộ thực thi `ServoJExecutor` và controller `CuRoboURSim` tích hợp cơ chế tự động gửi lệnh `stop\n` tới Dashboard Server (Port 29999) trước khi kết nối RTDE.
   - Loại bỏ hoàn toàn lỗi xung đột thanh ghi (`One of the RTDE input registers are already in use!`) hoặc treo tiến trình khi khởi động lại sau các lần thoát đột ngột.
   - Tự động fallback sang dải thanh ghi trên (`FLAG_UPPER_RANGE_REGISTERS`) nếu kết nối trên robot thật có cấu hình fieldbus (Profinet/EtherNet-IP).

9. **Quy Hoạch Trở Về Home An Toàn Không Giật & Chống Rung Lắc Hình Ảnh:**
   - Chuyển đổi phương thức thực thi quy hoạch Home từ vòng lặp `servoJ` rời rạc sang bộ sinh quỹ đạo phần cứng `movej` chạy ở tần số 500Hz trực tiếp trong bộ điều khiển Universal Robots qua cổng Secondary Port 30002.
   - Tự động hòa trộn góc bo cung (`blend radius = 0.025`m) khi bám theo các waypoints né vật cản từ CuRobo `MotionPlanner`, triệt tiêu 100% hiện tượng giật cục cơ học do thiếu hụt buffer servoJ.
   - Loại bỏ xung đột đa luồng cập nhật Viser 3D, xóa bỏ triệt để hiện tượng glitch hình ảnh chập chờn.

10. **Tự Động Tái Kích Hoạt Teleop Sau Khi Về Home (Zero-Click Auto Reconnect):**
    - Tích hợp cờ giám sát `_was_navigating_home`, tự động tái kết nối RTDE `executor.connect()`, đặt lại mỏ neo tay cầm VR `pose_source.reset_anchor()` và đưa hệ thống vào trạng thái sẵn sàng điều khiển ngay khi robot về tới Home mà người dùng không cần phải click tắt/bật lại checkbox thủ công.

11. **Giao Diện Viser 3D Cockpit Tối Giản (Bố Cục Lưới 2x2):**
    - Gom cụm 4 thao tác điều khiển cốt lõi thành ma trận 2x2 tiện dụng ngay trên đầu bảng điều khiển:
      - Hàng 1: `Bật Teleop` (Checkbox) & `Gripper (Bóp Kẹp)` (Checkbox).
      - Hàng 2: `Record AI Episode` (Checkbox) & `Hủy Episode (Discard)` (Checkbox dạng tự nhả sau 200ms).
    - Tối ưu không gian hiển thị, giúp tập trung tối đa vào không gian làm việc 3D và khung camera stream.

12. **Thu Thập & Quản Lý Dữ Liệu AI Chuẩn HDF5:**
    - Ghi lại chuỗi thao tác đầy đủ: góc khớp (`qpos`), vận tốc (`qvel`), lệnh điều khiển (`action`), trạng thái ngàm kẹp (`gripper`), ảnh camera cổ tay (`images/wrist_camera`), timestamp ở tần số 20Hz.
    - Tự động đánh dấu kết quả thành công/thất bại (`success=True/False`).

13. **Bộ Công Cụ Kiểm Định & Tái Lập Quỹ Đạo (Data Sanity & Replay):**
    - `verify_dataset.py`: Quét toàn bộ file HDF5 kiểm tra giá trị NaN/Inf, độ lệch chu kỳ lấy mẫu (dt jitter), kích thước tensor hình ảnh.
    - `replay_episode.py`: Tái hiện lại quỹ đạo trình diễn ở chế độ Offline Dry-Run hoặc Live Replay gửi lệnh trực tiếp sang robot.

14. **Các Lớp Bảo Vệ An Toàn (Safety Gate & Singularity Guard):**
    - **Giám sát điểm kỳ dị (Singularity Guard):** Đánh giá ma trận Jacobian thời gian thực, tự động chặn lệnh nếu giá trị suy biến nhỏ nhất $\sigma_{min} < 0.035$.
    - **Chống chúi cắm bắp tay:** Giới hạn góc nâng vai `shoulder_lift_joint` $\le +0.05$ rad để ngăn ngừa rủi ro đâm bắp tay xuống mặt bàn.
    - **Giới hạn tốc độ và gia tốc (Scaled Vector Rate Limiter):** Kẹp vận tốc góc khớp tối đa $\le 0.6$ rad/s ($\approx 34.4^\circ$/s) để tái hiện chuyển động êm ái, an toàn.
    - **Phát hiện lật nhánh IK:** Chặn các bước nhảy góc khớp đột ngột (`max_jump > 1.5` rad).
    - **Giám sát mất kết nối (Timeout Watchdog):** Tự động dừng khẩn cấp robot qua `servoStop` nếu mất tín hiệu VR quá thời gian timeout.

15. **Kiến Trúc Mô-đun Hóa Tinh Gọn (Single Responsibility Architecture):**
    - Phân tách độc lập các khối chức năng: `bootstrap.py` (khởi tạo hệ sinh thái phần mềm và kết nối phần cứng), `camera_pipeline.py` (luồng đa nhiệm camera & OSD), `main_quest.py` (bộ điều phối trung tâm) và `teleop_quest/__init__.py` xuất khẩu các interface tiêu chuẩn.

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
  (World-Centric, Soft Snapping, Smart Clutch)     │
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
           (Jacobian sigma_min >= 0.035, Rate Limiter)
                          │
                          ▼
              [ServoJExecutor (RTDE Cổng 30004)]
           (servoJ với lookahead_time = 0.12s, gain = 250)
                          │
                          ▼
              [Robot UR3e (URSim / Robot thật)]
                          │
          ┌───────────────┴───────────────┐
          ▼                               ▼
[DataLogger HDF5]             [CameraStreamPipeline]
(q, qd, tcp, action, nhãn)    (20 FPS độc lập, OSD Watermark, Viser & Quest)
```

### 2.2. Cơ Chế Máy Trạng Thái (Edit Mode vs. Drive Mode)

Hệ thống thiết lập 2 chế độ vận hành độc lập, hỗ trợ đồng bộ hai chiều tức thì giữa kính thực tế ảo và màn hình máy tính:

1. **Chế độ Cầm lái (Drive Mode - Mặc định khi vào VR):**
   - **Mục đích:** Vừa quan sát toàn bộ giao diện Cockpit HUD (mô hình 3D, camera cổ tay, tọa độ thời gian thực) nổi trước mắt, vừa tự do điều khiển cánh tay robot bằng tay cầm mà không bị vướng chuột ảo.
   - **Tương tác:** Giao diện HUD tự động kích hoạt chế độ xuyên thấu triệt để (`pointer-events: none !important`). Hệ điều hành Meta Horizon OS sẽ tắt hoàn toàn tia laser 2D. Lúc này mọi thao tác bóp cò Trigger, bóp Grip và bấm nút A/X được chuyển 100% sang điều khiển robot 6DoF, không còn hiện tượng "di chuột trên màn hình".
   - **Phản hồi thời gian thực:** Thẻ `ROBOT POSE` cập nhật liên tục tọa độ Position (X, Y, Z tính bằng mét) và Orientation (Qx, Qy, Qz, Qw) từ vòng lặp RTDE 20Hz của cánh tay robot UR3e thực tế. Thẻ `VR CONTROLLER` hiển thị trạng thái các nút bấm.
   - **Cách kích hoạt:** Mặc định tự động bật khi người dùng bấm *START IMMERSIVE VR*, hoặc bấm phím cứng **B** (tay phải) / **Y** (tay trái) trên tay cầm, hoặc click nút *Lock UI* trên menu HUD.

2. **Chế độ Chỉnh sửa Giao diện (Edit Mode):**
   - **Mục đích:** Thao tác cài đặt thông số, dùng tia laser chuột để xoay camera xem mô hình 3D trong khung Viser hoặc click các nút bấm trên trang web.
   - **Cách chuyển sang Edit Mode:** Khi đang ở trong VR, chỉ cần **bấm nhả phím B (hoặc Y)** trên tay cầm. Nút menu góc trên bên trái chuyển sang màu cam (`Lock UI (Drive Mode)`), thẻ Mode chuyển sang màu cam (`Edit Mode`), và tia laser chuột 2D xuất hiện.
   - **An toàn:** Cánh tay robot được khóa trạng thái (`teleop.enabled = False`). Mọi thao tác bóp cò hoặc vung tay trong chế độ này đều không phát lệnh di chuyển robot, tránh va chạm ngoài ý muốn.
   - **Quay lại lái robot:** Sau khi thao tác giao diện xong, chỉ cần **bấm nhả lại phím B (hoặc Y)**. Tia laser lập tức biến mất và hệ thống quay lại Drive Mode để tiếp tục lái robot mượt mà mà không cần thoát khỏi VR.

### 2.3. Quy Trình Vòng Lặp Điều Khiển Thời Gian Thực (Chu Kỳ 20Hz - 50ms)

Trong mỗi chu kỳ 50ms của [main_quest.py](file:///home/nguyen/ur_ws/curobo_ursim/teleop_quest/main_quest.py), hệ thống thực thi tuần tự 6 công đoạn:

1. **Bước 1: Thu thập & Lọc tín hiệu điều khiển (`DeltaPoseSource` / `WidgetPoseSource`):**
   - Khi dùng kính Quest: Cò trỏ (Trigger) phụ trách tịnh tiến XYZ (World-Centric), cò báng (Grip) phụ trách xoay hướng công cụ (Tool-Centric), nút A/X chuyển đổi trạng thái ngàm kẹp theo cơ chế Toggle.
   - Áp dụng bộ lọc Soft Rotation Snapping triệt tiêu dao động góc quay phụ khi gập cổ tay.
   - Khi dùng chuột: Tọa độ đích được cập nhật trực tiếp từ khối Target Widget trên không gian 3D của Viser.
   - Giới hạn Safe Box kẹp chặt tọa độ mục tiêu trong tầm với an toàn của UR3e ($X, Y \in [-0.45, 0.45]$m, $Z \in [-0.02, 0.55]$m).

2. **Bước 2: Giải ngược động học và tránh va chạm trên GPU (`CuRoboIKSolver`):**
   - Nhận tọa độ mục tiêu 6DoF từ Widget 3D, giải bài toán IK phi tuyến kết hợp kiểm tra va chạm trên GPU NVIDIA trong khoảng 3–5ms.
   - Sử dụng 64 hạt giống song song (seeds) và hệ số phạt lệch nhánh (`branch_penalty = 50.0`), đảm bảo nghiệm góc khớp trả về liên tục và không bị lật khớp cổ tay.

3. **Bước 3: Kiểm định an toàn và điểm kỳ dị (`SafetyGate` & `SingularityGuard`):**
   - Đánh giá ma trận Jacobian của cấu hình góc khớp mục tiêu, đo lường giá trị suy biến nhỏ nhất $\sigma_{min}$.
   - Nếu $\sigma_{min} < 0.035$ (tư thế tiến sát điểm kỳ dị), cổng an toàn lập tức chặn lệnh, phát xung rung cảnh báo về tay cầm VR và giữ robot đứng yên.
   - Khối Scaled Vector Rate Limiter kiểm tra và kẹp vận tốc góc khớp tối đa $\le 0.6$ rad/s để tạo chuyển động êm dịu, không giật cục.

4. **Bước 4: Thực thi lệnh chuyển động thời gian thực (`ServoJExecutor`):**
   - Đảm bảo giải phóng thanh ghi RTDE thông qua lệnh Dashboard `stop\n`.
   - Truyền gói dữ liệu 6 góc khớp mục tiêu $q_{target}$ qua giao thức RTDE (Cổng 30004) bằng lệnh `servoJ` với `lookahead_time = 0.12`s và `gain = 250`.
   - Robot phản hồi bám sát tức thì với độ trễ chuyển động thực tế dưới 10ms.

5. **Bước 5: Mô phỏng gắp nhặt & Ghi tập dữ liệu AI (`GraspSimulator` & `DataLogger`):**
   - Kiểm tra tương tác hình học giữa đầu ngàm kẹp và quả cà chua mục tiêu. Khi tiếp cận đúng phạm vi và ngàm kẹp đóng, vật thể được gắn dính vào đầu kẹp và gửi phản hồi Haptic thành công.
   - Khi cờ ghi hình kích hoạt (`ui.cb_record`), module `DataLogger` lưu trữ đồng bộ trạng thái góc khớp thực tế, vận tốc, tọa độ TCP, lệnh điều khiển, trạng thái kẹp và ảnh camera thô vào tệp HDF5 theo chu kỳ cố định 20Hz.

6. **Bước 6: Thu nhận hình ảnh camera cổ tay độc lập (`CameraStreamPipeline`):**
   - Luồng nền camera thu nhận ảnh ở tần số 20 FPS từ camera ảo 3D hoặc camera vật lý thật gắn trên cổ tay (RGB 640x480).
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
├── main.py                          # Điểm khởi chạy CLI duy nhất của hệ thống
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
│   └── teleoperation.py             # TeleoperationController, ServoJExecutor, SafetyGate
├── teleop_quest/
│   ├── __init__.py                  # Xuất khẩu các lớp và hàm chính của package
│   ├── bootstrap.py                 # Khởi tạo Docker URSim, ADB watcher, config & SSL
│   ├── camera_pipeline.py           # Quản lý luồng nền camera, preview OSD & streaming
│   ├── camera_driver.py             # Driver camera cổ tay hợp nhất: Ảo 3D & Camera thật
│   ├── synthetic_camera.py          # Render camera ảo cổ tay 640x480 (Eye-in-Hand)
│   ├── viser_ui.py                  # Giao diện Viser 3D Cockpit và bảng điều khiển 2x2
│   ├── grasp_simulator.py           # Mô phỏng gắp/thả quả cà chua 3D vào giỏ
│   ├── quest_server.py              # Server HTTPS (8443), WSS (8444) và MJPEG stream
│   ├── network_utils.py             # Dò tìm IP LAN và tự động sinh chứng chỉ SSL
│   ├── pose_sources.py              # WidgetPoseSource (đọc tọa độ Widget kèm bộ lọc EMA)
│   ├── delta_pose_source.py         # Chuyển đổi vi phân Quest, khóa hướng đế q0 & Soft Snapping
│   ├── timeout_watchdog.py          # E-Stop và dừng khẩn cấp robot khi mất tín hiệu
│   ├── home_planner.py              # Đưa robot về vị trí Home an toàn (movej 500Hz)
│   ├── setup_quest_adb.sh           # Script chuyển tiếp cổng ADB sang localhost cho Quest
│   └── static/
│       ├── index.html               # Giao diện WebXR Cockpit trên kính Quest
│       ├── main.js                  # Xử lý gamepad WebXR, Toggle Gripper và rung Haptic
│       └── style.css                # Style hiển thị phong cách HUD hiện đại
└── tests/
    ├── test_camera.py               # Kiểm thử render camera ảo ở các góc nhìn & giao diện GUI
    ├── test_simulated_pick_and_place.py # Kiểm thử tự động pick-place cà chua và xuất HDF5
    └── test_singularity_guard.py    # Kiểm thử tính toán Jacobian singularity metric (sigma_min)
```

---

## 5. Hướng Dẫn Vận Hành

### Bước 1: Khởi động hệ thống
Bạn chỉ cần thực thi một lệnh duy nhất. Hệ thống sẽ tự động kiểm tra và khởi động container Docker URSim nếu đang chạy mô phỏng cục bộ:

```bash
cd /home/nguyen/ur_ws/curobo_ursim
source ~/curobo/.venv/bin/activate

# 1. Chạy với URSim mô phỏng (Camera ảo 3D - Mặc định, tự động kích hoạt Docker URSim):
python3 main.py

# 2. Chạy với Robot UR3e thật:
python3 main.py --robot-ip 10.42.0.242

# 3. Chạy với Robot UR3e thật kèm Camera vật lý (USB Camera / RealSense):
python3 main.py --robot-ip 10.42.0.242 --camera real --camera-device 0
```

Các tham số dòng lệnh được hỗ trợ:
- `--robot-ip`: Địa chỉ IP của Robot thật hoặc URSim (ví dụ: `10.42.0.242`, mặc định: `127.0.0.1`).
- `--camera`: Nguồn camera cổ tay: `synthetic` (mô phỏng) hoặc `real` (camera thật) (mặc định: `synthetic`).
- `--camera-device`: Chỉ số Device ID của camera thật (mặc định: `0`).

Hệ thống sẽ hiển thị các cổng truy cập:
- Viser Dashboard: `http://localhost:8080`
- WebXR Quest (Wi-Fi): `https://<IP_LAN>:8443`
- WebXR Quest (Cáp USB ADB): `https://localhost:8443`

### Bước 2: Điều khiển bằng Viser Web UI (Máy tính)
1. Mở trình duyệt web trên máy tính và truy cập `http://localhost:8080`.
2. Kiểm tra mô hình robot 3D hiển thị khớp đúng với trạng thái thực tế của robot.
3. **Thao tác trên Bảng điều khiển Lưới 2x2:**
   - **Bật Điều Khiển (Teleop):** Tích chọn để kích hoạt cánh tay robot bám theo mục tiêu (Nhãn `Status` chuyển sang `Tracking`).
   - **Gripper (Bóp Kẹp):** Tích chọn để đóng ngàm kẹp gắp quả cà chua (hoặc bỏ chọn để nhả vật thể).
   - **Record AI Episode:** Tích chọn để bắt đầu ghi chuỗi thao tác phục vụ huấn luyện mô hình Robot AI.
   - **Hủy Episode (Discard):** Tích chọn khi thực hiện thao tác hỏng; hệ thống sẽ hủy bỏ dữ liệu tập hiện tại và tự động nhả checkbox sau 200ms.
4. **Điều khiển chuyển động:** Dùng chuột nhấp và kéo các trục mũi tên hoặc vòng xoay của khối Target Widget 3D: Cánh tay robot sẽ chuyển động bám theo tọa độ đích của chuột trong thời gian thực.
5. **Trở về vị trí an toàn:** Bấm nút **Go to Home** khi cần đưa robot về vị trí mặc định an toàn. Quỹ đạo chuyển động được điều phối mượt mà không rung giật (MoveJ 500Hz), không glitch hình ảnh và tự động kích hoạt lại chế độ Teleop sau khi robot về đích.

### Bước 3: Hướng Dẫn Chi Tiết Điều Khiển Bằng Kính Meta Quest 3S (WebXR)

#### 3.1. Bảng Chức Năng Các Phím Trên Tay Cầm Meta Quest 3S

| Phím bấm | Ngón tay thao tác | Cơ chế điều khiển | Mô tả chi tiết hành vi |
| :--- | :--- | :--- | :--- |
| **Nhấn cần gạt Thumbstick (R3/L3)** | Ngón cái | **Về Vị Trí An Toàn (Go Home)** | **Nhấn 1 lần vào cần gạt Analog** (hoặc nhấn giữ nút B/Y trong 3 giây) để tự động đưa cánh tay robot về vị trí Home an toàn. Tay cầm rung xác nhận 2 nhịp, tự động mở khóa bảo vệ và tái kích hoạt Teleop khi về đích. |
| **Nút B (Tay phải) / Y (Tay trái)** | Ngón cái | **Chuyển Chế Độ / Về Home** | - **Bấm nhanh (< 3s):** Chuyển đổi giữa **Edit Mode** (chỉnh UI Cockpit) và **Drive Mode** (lái robot).<br>- **Nhấn giữ (> 3s):** Kích hoạt lệnh **Go Home** đưa robot về vị trí an toàn. |
| **Cò Trỏ (Index Trigger)** | Ngón trỏ | **Tịnh Tiến Không Gian (Arm-Plane / Cartesian)** | Bấm giữ cò và di chuyển tay trong không gian để tịnh tiến đầu robot. Hướng tiến luôn bám theo góc khớp đế $q_0$. Góc xoay cổ tay được khóa cố định. |
| **Nút Hông (Hand Grip)** | Ngón giữa / ngón áp út | **Xoay 3 Trục Độc Lập (Decoupled 3-Axis Rotation)** | Bấm giữ nút hông để xoay ngàm gắp trên cả 3 trục: Gập/chúc (Pitch), Lắc ngang (Yaw), và Vặn ngàm (Roll). Hệ thống tích hợp bộ lọc trực giao World Frame và Khóa trục thông minh (Soft Rotation Snapping), triệt tiêu hoàn toàn hiện tượng rò rỉ chéo trục khi gập mở cổ tay. |
| **Nút A (Tay phải) / X (Tay trái)** | Ngón cái | **Bật / Tắt Ngàm Kẹp (Toggle Gripper)** | **Nhấn 1 lần** để kẹp/hút quả cà chua, **nhấn lần nữa** để nhả vật thể vào giỏ. Tay cầm rung phản hồi haptic mỗi lần bấm. |
| **Nút Meta (Oculus)** | Ngón cái (Tay phải) | **Căn Chuẩn Trục Nhìn (Recenter)** | Nhấn giữ 2 giây để đồng bộ lại hệ trục phòng của kính Meta Quest. |

#### 3.2. Cơ Chế Khóa Hướng Tiến Theo Đế Robot ($q_0$)
Khi người vận hành đứng ở các góc khác nhau quanh robot hoặc khi đế robot đã xoay một góc $q_0$:
- Thao tác đẩy tay ra xa hoặc kéo về gần người sẽ luôn di chuyển dọc theo mặt phẳng cánh tay vươn của robot.
- Triệt tiêu hoàn toàn hiện tượng robot bị bẻ ngoặt hoặc quay vòng khi người dùng kéo tay thẳng về sau.

#### 3.3. Hệ Thống Lọc Nhiễu Đa Tầng & Xoay 3 Trục Độc Lập (Anti-Noise & Arm-Plane Decoupled 3-Axis)
Nhằm mang lại chuyển động siêu êm, loại bỏ triệt để rung lắc sinh trắc học và tracking jitter:
- **Tách biệt hoàn toàn 3 trục (World Frame Decoupling & Intuitive Mapping):** Tính toán biến thiên góc xoay trực tiếp trong Không gian Thực ($\Delta R_{world} = R_{hand} \cdot R_{anchor}^{-1}$), triệt tiêu 100% hiện tượng góc nghiêng tay cầm Quest làm biến dạng trục xoay:
  - **Pitch (Gập/ngửa cổ tay - Uống nước):** Khi dốc mũi tay về phía mình, đầu ngàm kẹp ngửa ra xa thân máy theo đúng chiều trực quan; khi chúi tay xuống, đầu kẹp gập vào thân máy.
  - **Roll (Lắc cổ tay trái/phải):** Khi lắc nhẹ bàn tay sang hai bên, hai ngón kẹp của robot xoay tròn tại chỗ quanh trục kẹp (Tool Z) để căn chỉnh góc đón quả cà chua trên bàn một cách tự nhiên và đỡ mỏi cơ cẳng tay.
  - **Tilt (Vặn chìa khóa quanh cẳng tay):** Nghiêng đầu ngàm kẹp sang trái hoặc sang phải (Lateral Tilt quanh trục vươn của cánh tay $\vec{u}_{reach}$), phản hồi trực quan theo hướng vặn chìa khóa.
- **Đồng bộ hóa theo Mặt Phẳng Cánh Tay (Arm-Plane Rotation Mapping):** Trục gập (Pitch quanh $\vec{u}_{lateral}$) và trục nghiêng hai bên (Tilt quanh $\vec{u}_{reach}$) tự động bám theo góc vươn của cánh tay robot ($q_0$). Khi bấm giữ cả 2 nút (vừa kéo tay thu về vừa ngửa cổ tay), chuyển động tịnh tiến và gập ngửa hòa làm một chuyển động liền mạch dọc theo đúng hướng cánh tay thu về, không bị lệch pha hay ngoẹo góc.
- **Khóa rãnh đơn trục triệt để (Soft Snapping < 55%):** Nâng ngưỡng khóa trục phụ lên $55\%$ để triệt tiêu hoàn toàn góc kéo xéo sinh học tự nhiên khi thao tác. Khi gập ngửa hoặc lắc cổ tay, robot chỉ chuyển động thẳng tắp trên trục mong muốn, triệt tiêu $100\%$ hiện tượng lắc chao đảo phụ. Chuyển tiếp mượt sang xoay 3D đa trục khi chủ ý bẻ góc lớn ($> 75\%$).
- **Vùng chết mềm chống vi rung (Soft Deadband $2.6^\circ$ & $6\text{mm}$):** Khi người vận hành bấm giữ phím Grip hoặc Trigger mà vẫn giữ yên tay, mọi dao động run cơ học tự nhiên được hấp thụ hoàn toàn về $0.00$. Chuyển động bắt đầu từ $0$ liên tục, không bị nhảy cóc khi vượt ngưỡng.
- **Chốt giữ trục chính (Hysteresis Latch):** Ngăn chặn hiện tượng lật trục chập chờn (Chatter) khi người dùng thao tác ở ranh giới giữa 2 trục, ưu tiên giữ ổn định trục ban đầu trừ khi trục mới vượt trội > 15%.
- **Bộ lọc thích ứng động (Dynamic Adaptive EMA & Slew Rate Limiter):**
  - **Trên không gian trắc địa $SO(3)$ và $\mathbb{R}^3$:** Khi tay đứng yên hoặc vi rung chậm, hệ số $\alpha$ tự động giảm thấp ($\alpha \approx 0.18$) để lọc phẳng hoàn toàn nhiễu tracking IMU. Khi di chuyển dứt khoát, $\alpha$ tự động tăng cao ($\alpha \approx 0.40 - 0.48$) để phản hồi tức thì với độ trễ cực thấp.
  - **Giới hạn biến thiên tối đa (Slew Rate Limiter):** Khống chế bước nhảy tối đa $3.5\text{cm}/50\text{ms}$ và $11.5^\circ/50\text{ms}$, triệt tiêu nguy cơ robot bị giật nảy đột biến khi WebXR rớt frame hoặc tracking bị vấp.
- **Tùy biến linh hoạt qua biến môi trường:**
  - `ROTATION_MODE=decoupled_3axis` (Mặc định): Điều khiển 3 trục độc lập theo Arm-Plane với hệ thống lọc nhiễu đa tầng.
  - `ROTATION_MODE=table_roll`: Khóa chúc ngàm cố định vuông góc bàn, chỉ Roll quanh trục kẹp.
  - `ROTATION_MODE=free_3d`: Xoay 3D tự do.
  - `ROT_DEADBAND` (Mặc định `0.045` rad ~ $2.6^\circ$), `POS_DEADBAND` (Mặc định `0.006` m ~ $6\text{mm}$).
  - `ROT_SNAP_LOW` (Mặc định `0.55`), `ROT_SNAP_HIGH` (Mặc định `0.75`).

#### 3.4. Bảng Quy Chuẩn Điều Khiển & Kịch Bản Kiểm Thử (Đã Xác Thực Đúng Logic & Sẵn Sàng Vận Hành)
Hệ thống điều khiển 6-DOF đã được chuẩn hóa và kiểm nghiệm thực tế, hoàn toàn sẵn sàng cho vận hành trên robot thật:

| Thao tác tay người cầm Quest | Trục toán học | Phản hồi chuyển động trên Robot UR3e | Trạng thái kiểm chứng |
| :--- | :--- | :--- | :--- |
| **Gập/ngửa cổ tay (Uống nước)** | Pitch ($\vec{u}_{lateral}$) | Khi dốc tay về phía mình: ngàm ngửa ra xa thân máy. Khi chúi tay: ngàm gập vào thân máy. Khóa cứng lắc phụ. | Đã xác thực đúng logic |
| **Lắc cổ tay trái / phải** | Roll ($\text{Tool Z}$) | Hai ngón kẹp xoay tròn tại chỗ quanh trục kẹp (CW / CCW) để căn góc đón quả cà chua trên bàn. | Đã xác thực đúng logic |
| **Vặn chìa khóa quanh cẳng tay** | Tilt ($\vec{u}_{reach}$) | Nghiêng ngàm kẹp sang bên phải (khi vặn phải) hoặc nghiêng sang bên trái (khi vặn trái). | Đã xác thực đúng logic |
| **Bấm giữ đồng thời Cò (Trigger) + Grip** | Tịnh tiến + Xoay | Tịnh tiến dọc theo cánh tay vươn kết hợp gập ngửa và chỉnh góc ngàm đồng bộ, không giật cục. | Đã xác thực đúng logic |
| **Bấm nút A hoặc X** | Toggle Gripper | Kích hoạt đóng / mở ngàm kẹp gắp cà chua với phản hồi rung Haptic trên tay cầm. | Đã xác thực đúng logic |

#### 3.5. Kỹ Thuật Ly Hợp Điều Khiển Tầm Xa (Smart Clutching)
Khi cần điều khiển robot di chuyển quãng đường dài vượt quá tầm với tự nhiên của sải tay:
1. Bấm giữ Cò trỏ và đưa tay về phía trước đến hết tầm với thoải mái.
2. **Nhả Cò trỏ:** Cánh tay robot sẽ dừng lại và giữ nguyên vị trí hiện tại trên bàn.
3. Rút tay cầm về gần ngực hoặc vị trí thoải mái nhất của cơ thể.
4. **Bấm giữ lại Cò trỏ:** Hệ thống tự động neo mốc mới (Reset Anchor). Robot tiếp tục di chuyển tiến tới từ vị trí đang đứng một cách mượt mà, hoàn toàn không bị giật hay nhảy cóc tọa độ (Zero Teleport).

#### 3.6. Kết Nối Kính Meta Quest Qua Cáp USB (ADB Reverse Cực Nhanh)
Khi cắm cáp USB giữa PC và kính Meta Quest:
1. Tiến trình ngầm `start_adb_watcher` trong [bootstrap.py](file:///home/nguyen/ur_ws/curobo_ursim/teleop_quest/bootstrap.py) sẽ tự động nhận diện thiết bị và chạy script chuyển tiếp cổng.
2. Nếu kính báo `unauthorized`, hãy đeo kính lên và nhấn **"Allow USB Debugging"** (và tích chọn "Always allow from this computer").
3. Khi thông báo kết nối thành công xuất hiện trên terminal, chỉ cần mở Meta Quest Browser trên kính và truy cập:
   ```text
   https://localhost:8443
   ```
4. Bấm **Advanced** -> **Proceed to localhost (Tiếp tục truy cập an toàn)** -> Bấm **Enter VR** màu xanh lục để bắt đầu điều khiển.

---

## 6. Hướng Dẫn Triển Khai Trên Cánh Tay Robot UR3e Thực Tế

### 6.1. Kết Nối Phần Cứng & Cấu Hình Mạng LAN
1. **Cắm cáp mạng LAN:** Kết nối trực tiếp từ cổng mạng của máy tính (PC chạy code) vào cổng mạng trên hộp điều khiển UR Control Box (hoặc qua Switch mạng nội bộ).
2. **Cài đặt IP tĩnh trên Robot (qua màn hình Teach Pendant - PolyScope):**
   - Vào menu: **Settings** -> **System** -> **Network**.
   - Chọn **Static Address**:
     - IP Address: ví dụ `192.168.1.100` (hoặc `10.42.0.242`)
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
3. **Bật nguồn và Nhả phanh:**
   - Kích hoạt nguồn (Power On) và bấm **Release Brakes** để robot vào trạng thái sẵn sàng hoạt động (màn hình hiển thị màu xanh lá).
4. **Hạ thanh trượt Tốc độ (Speed Slider):**
   - Trong lần đầu vận hành, kéo thanh trượt tốc độ ở góc dưới màn hình PolyScope xuống mức **10% – 20%** để đảm bảo an toàn quan sát.
5. **Cầm sẵn nút dừng khẩn cấp (Emergency Stop):**
   - Luôn cầm Teach Pendant trên tay và sẵn sàng ấn nút E-Stop màu đỏ nếu robot có hiện tượng di chuyển sai hướng.

### 6.3. Khởi Chạy Với Robot Thật
Khởi chạy hệ thống bằng lệnh:
```bash
python3 main.py --robot-ip 192.168.1.100
```
Nếu có camera vật lý USB hoặc RealSense gắn tại cổ tay:
```bash
python3 main.py --robot-ip 192.168.1.100 --camera real --camera-device 0
```

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
| **Tần số Camera Stream** | 20 FPS | Luồng nền `CameraStreamPipeline` độc lập, triệt tiêu giật lag cho vòng lặp RTDE. |
| **ServoJ Lookahead Time** | `0.12` s | Nội suy dự đoán đa chu kỳ chuẩn Universal Robots, triệt tiêu vi rung giật. |
| **ServoJ Gain** | `250` | Độ cứng vị trí tối ưu trên robot vật lý, giúp chuyển động lướt êm ái không bị gắt cơ học. |
| **Vận tốc khớp tối đa (Rate Limiter)** | `0.6` rad/s ($\approx 34.4^\circ$/s) | Scaled Vector Limiter co giãn đồng bộ cả 6 khớp, phản hồi nhanh nhạy và mượt mà. |
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
| **Tỷ lệ truyền chuyển động tay (scale)** | `0.75` | Tỷ lệ 75% giúp robot bám sát chuyển động tay nhanh nhạy, giảm mỏi khi vung tay. |
| **Độ lệch tay cầm tối đa (max_delta_m)** | `0.22` m | Phạm vi vươn dài hơn mỗi nhịp bóp cò, thoải mái bao quát không gian làm việc. |
| **Bộ lọc làm mượt EMA (Alpha)** | `0.28` | Giảm độ trễ lọc tín hiệu từ 200ms xuống ~170ms, giúp bám tay tức thì mà vẫn êm ái. |
| **Ngưỡng thời gian Auto-Snap** | 1.2 s | Tự động kéo XYZ về đầu tay máy và reset seed khi bị kẹt tư thế hoặc vướng vật cản. |
| **Khóa trục xoay cổ tay (Soft Snapping)** | $<25\%$ (cắt về 0), $25-40\%$ (làm mềm) | Triệt tiêu góc xoay phụ khi gập/chúc cổ tay, bảo vệ trục quay chính xác. |
| **Cơ chế Gripper** | Toggle Mode | Nhấn 1 lần nút A/X để kẹp/hút, nhấn lần nữa để thả; kèm rung Haptic. |
| **Khởi động URSim** | Tự động qua Docker | Tự kiểm tra cổng 29999 và chạy `docker start ursim` khi chạy `127.0.0.1`. |
| **Chuyển tiếp cổng ADB Quest** | Tự động chạy nền | Kiểm tra kết nối cáp USB mỗi 2.5s và tự động ánh xạ cổng sang `localhost`. |
