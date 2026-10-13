# BÁO CÁO ĐÁNH GIÁ CODEBASE TELEOP & THU DATA DEMONSTRATION (UR3e + IL)

> **Người thực hiện:** Senior Robotics / Robot Learning Reviewer  
> **Phạm vi kiểm tra:** Teleoperation, Controller RTDE, Simulation (Viser/CuRobo), Data Collection Pipeline (HDF5).  
> **Chế độ:** Read-only Evaluation (Không can thiệp sửa trực tiếp codebase).

---

## 1. TÓM TẮT KIẾN TRÚC & LUỒNG DỮ LIỆU (Tối đa 10 dòng)
1. **Kiến trúc:** Phân tầng tách biệt gồm WebXR/WSS (`QuestServer`) + Viser GUI -> Bộ xử lý biến thiên tọa độ (`DeltaPoseSource`/`WidgetPoseSource`) -> Bộ điều phối (`TeleoperationController`) tích hợp GPU IK (`CuRoboIK`), màng an toàn (`SingularityGuard`/`SafetyGate`) -> Bộ thực thi `ServoJExecutor` (`RTDEControlInterface`) giao tiếp URSim qua cổng 30004/30002.
2. **Luồng dữ liệu:** Vòng lặp chính 20Hz (`main_quest.py`) nhận pose delta từ VR -> giải IK CuRobo -> kiểm tra kỳ dị & kẹp vận tốc khớp -> gửi lệnh `servoJ` sang URSim -> cập nhật logic gắp hút dính vật thể (`GraspSimulator`) -> tính CuRobo FK để render ảnh cổ tay (`SyntheticWristCamera`) -> nạp `q_actual, qd_actual, tcp_actual, q_target, grip, images` vào RAM buffer (`DataLogger`).
3. **Định dạng lưu:** File HDF5 tự tạo (`episode_*.hdf5`) nén gzip chứa group `observations` (`qpos, qvel, tcp_pose, gripper, timestamps_ms, images/wrist_camera`) và dataset `action` kích thước `[T, 7]`.
4. **Điểm vào chính:** `curobo_ursim/main.py` (chuyển tiếp tới `teleop_quest/main_quest.py:main()`), các file kiểm thử độc lập: `test_simulated_pick_and_place.py`, `test_camera.py`.

---

## 2. BẢNG CHECKLIST ĐÁNH GIÁ TOÀN DIỆN

| STT | Tiêu chí | Đánh giá | Bằng chứng (File:Dòng) | Mức độ nghiêm trọng | Tóm tắt vấn đề |
| :--- | :--- | :---: | :--- | :---: | :--- |
| **1** | **Đồng bộ tần số & Timestamp** | **Có vấn đề** | `teleop_quest/main_quest.py:37, 343`<br>`core/data_logger.py:84, 128` | **Cao** | Vòng lặp dùng `time.sleep(POLL_SLEEP)` cố định sau GPU IK & OpenCV render, làm chu kỳ thực tế biến thiên (12–16Hz thay vì 20Hz). `hz=20` bị gán cứng trong HDF5 dù khoảng cách timestamp $\Delta t$ bị trôi. |
| **2** | **Observation** | **Có vấn đề** | `teleop_quest/synthetic_camera.py:132-138`<br>`core/data_logger.py:88, 138`<br>`teleop_quest/main_quest.py:329` | **Cao** | Text OSD ("GRIPPER: CLOSED", "EYE-IN-HAND CAMERA") bị vẽ đè trực tiếp lên ảnh lưu vào dataset (gây shortcut learning). Thiếu static/overhead camera. `tcp_actual` từ RTDE là 6D axis-angle nhưng buffer khởi tạo `[0.0]*7` (lẫn lộn hệ quaternion vs rotvec). Pose cà chua ground truth **KHÔNG** bị lẫn vào obs (Đạt). |
| **3** | **Action** | **Có vấn đề** | `core/data_logger.py:124`<br>`teleop_quest/main_quest.py:325-334`<br>`teleop/teleoperation.py:307, 329` | **Trung bình** | Chỉ ghi `q_target` (joint space), hoàn toàn thiếu target EE pose / delta EE pose. Khi IK fail hoặc vướng vật cản, `_last_valid_q` bị ghi lại làm action mà không có cờ đánh dấu lỗi. Gripper actual và target bị gán bằng nhau lập tức (bỏ qua độ trễ cơ học). |
| **4** | **Teleoperation** | **Có vấn đề** | `teleop_quest/delta_pose_source.py:47`<br>`teleop_quest/main_quest.py:298-301`<br>`teleop/teleoperation.py:303, 310` | **Cao** | **Xung đột phím VR nghiêm trọng:** Nút `grip` trên tay cầm vừa dùng để kích hoạt xoay cổ tay TCP, vừa dùng để đóng gripper kẹp cà chua. Khi người dùng xoay tay thì gripper tự đóng và ngược lại. Điểm tốt: Đã có clutch reset anchor, soft deadband, rate limiter khớp, watchdog timeout. |
| **5** | **Episode Management** | **Có vấn đề** | `teleop_quest/main_quest.py:82-83, 128`<br>`core/data_logger.py:128-132` | **Cao** | Vị trí ban đầu cố định cứng (không seed, không tự ngẫu nhiên hóa). Nhãn `success=True` bị gán cứng khi kết thúc episode, bỏ qua sự kiện rơi (`DROPPED`) của `GraspSimulator`. Thiếu lưu vị trí ban đầu của vật vào metadata. |
| **6** | **Định dạng & Schema** | **Có vấn đề** | `core/data_logger.py:117-150` | **Trung bình** | Thiếu file schema quy định hệ tọa độ, đơn vị, format. Lưu ảnh RGB thô dạng numpy stack nén `gzip` rất tốn RAM, làm lag khi ghi đĩa (không dùng MP4/JPEG chunking chuẩn của LeRobot). |
| **7** | **Khả năng Tái lập** | **Thiếu** | Toàn bộ codebase (không tìm thấy script `replay` hay `validate`) | **Cao** | Hoàn toàn thiếu script replay trajectory trên URSim/robot để kiểm chứng. Thiếu script tự động quét NaN, kiểm tra độ dài đồng bộ các mảng, kiểm tra phân bố tần số timestamp. |
| **8** | **Đa dạng Dữ liệu** | **Thiếu** | `teleop_quest/grasp_simulator.py:13, 22-42`<br>`teleop_quest/synthetic_camera.py:65-90` | **Cao** | Không có Domain Randomization: Mặt bàn xám, cà chua đỏ phẳng, rổ xanh cố định, ánh sáng phẳng không có đổ bóng vật lý. Policy huấn luyện từ dữ liệu này sẽ overfit hoàn toàn vào đặc trưng đồ họa đơn giản. |
| **9** | **Khả năng chuyển đổi (VR & Robot thật)** | **Có vấn đề** | `teleop_quest/synthetic_camera.py:1-140`<br>`teleop_quest/grasp_simulator.py:61-64` | **Cao** | Dùng camera ảo vẽ bằng lệnh 2D OpenCV (`cv2.circle`) và cơ chế gắp hút dính hình học (<7cm) thay vì tương tác vật lý. Không có giá trị Sim-to-Real sang robot thật hoặc simulator vật lý (Isaac Sim/MuJoCo). |

---

## 3. DANH SÁCH 3 VẤN ĐỀ NGHIÊM TRỌNG NHẤT CẦN SỬA TRƯỚC KHI THU DATA

### Vấn đề 1: Xung đột ánh xạ nút bấm điều khiển VR (Grip button conflict)
- **Mức độ:** Khẩn cấp (P0).
- **Nguyên nhân:** Trong `delta_pose_source.py:47`, cờ `is_rotating` đọc từ `frame.get("grip")`. Nhưng tại `main_quest.py:298-301`, biến `vr_grip` cũng đọc từ `frame.get("grip")` để kích hoạt `is_grip_pressed`. Hệ quả: Người điều khiển không thể xoay cổ tay mà không kẹp gripper, và không thể kẹp giữ cà chua mà không vô tình kích hoạt xoay cổ tay.
- **Cách sửa tối thiểu (Đề xuất):** 
  - Tách bạch kênh điều khiển trên tay cầm Quest: Dùng nút `Grip` (cò ngón giữa) thuần túy cho Clutch Xoay định hướng; Dùng nút `A` (hoặc `X` trên tay trái, hoặc Trigger phụ) chuyên dụng cho đóng/mở Gripper.
  - Sửa ánh xạ trong `quest_server.py` và `static/index.html` (WebXR gamepad mapping): truyền riêng trường `gripper_button: boolean`.
- **Cách kiểm chứng:** Đeo kính VR, giữ nút xoay góc cổ tay 45 độ nhưng quan sát Gripper trên Viser vẫn mở; sau đó ấn nút kẹp riêng biệt và thấy quả cà chua được gắp mà góc xoay TCP giữ nguyên.

### Vấn đề 2: Dữ liệu ảnh bị nhiễm ký tự Text Watermark & Render phi vật lý
- **Mức độ:** Nghiêm trọng (P0).
- **Nguyên nhân:** `synthetic_camera.py:132-137` gọi hàm `cv2.putText()` in trực tiếp các dòng chữ: `"EYE-IN-HAND CAMERA (SIMULATION)"` và `"GRIPPER: CLOSED [1.0]"` lên chính frame RGB được đưa vào `DataLogger`. 
- **Hậu quả học máy:** Mô hình mạng neuron (ResNet/ViT trong ACT hoặc Diffusion Policy) sẽ bị **Causal Confusion** và **Shortcut Learning**: Thay vì học trích xuất đặc trưng không gian của quả cà chua và đầu kẹp, mạng sẽ "đọc" dòng chữ text trạng thái ở góc màn hình để quyết định hành động. Ngoài ra, việc dùng OpenCV 2D canvas vẽ vòng tròn không có ánh sáng vật lý khiến dữ liệu vô dụng cho Sim-to-Real.
- **Cách sửa tối thiểu (Đề xuất):**
  - Tách hàm render ảnh sạch (`raw_rgb`) để lưu vào `DataLogger`, chỉ thêm OSD overlay lên bản sao hiển thị (`ui.update_cam_preview`).
  - Tích hợp `CameraWorker` thực tế hoặc dùng camera render từ hệ thống 3D (Pyrender / Isaac Sim / MuJoCo) thay cho canvas 2D.
- **Cách kiểm chứng:** Chạy `python test_simulated_pick_and_place.py`, mở file HDF5 sinh ra bằng script kiểm tra, trích xuất frame đầu tiên và xác nhận trên mảng pixel không chứa bất kỳ pixel text OSD nào.

### Vấn đề 3: Trôi chu kỳ lấy mẫu (Time Jitter & Uncompensated Loop Rate)
- **Mức độ:** Nghiêm trọng (P1).
- **Nguyên nhân:** `main_quest.py:343` kết thúc vòng lặp bằng `time.sleep(POLL_SLEEP)` (`POLL_SLEEP = 0.05`). Tổng thời gian một bước lặp bằng:  
  $$T_{step} = T_{IK} (\sim 10\text{ms}) + T_{render} (\sim 15\text{ms}) + T_{RTDE} (\sim 5\text{ms}) + 0.05\text{s} \approx 0.075\text{s} - 0.085\text{s} \implies f \approx 11.7 - 13.3\text{ Hz}$$
  Trong khi đó, HDF5 ghi nhận `hz = 20`. Khoảng cách $\Delta t$ giữa các frame dao động mạnh, phá vỡ giả định thời gian lấy mẫu đều của ACT và Diffusion Policy (ảnh hưởng trực tiếp tới Action Chunking).
- **Cách sửa tối thiểu (Đề xuất):**
  - Triển khai bộ điều phối tần số chính xác (Rate-compensated timer) dựa trên `time.perf_counter()`: `sleep_time = max(0.0, target_dt - (time.perf_counter() - loop_start))`.
  - Đưa việc render camera và ghi dữ liệu sang luồng nền (Asynchronous logging thread / queue).
- **Cách kiểm chứng:** Thu thập 1 demo 200 frame, dùng script tính vector $\Delta t = timestamps[i+1] - timestamps[i]$. Độ lệch chuẩn $\sigma(\Delta t) \le 2.0\text{ms}$ xung quanh giá trị kỳ vọng $50\text{ms}$.

---

## 4. CHI TIẾT NGUYÊN NHÂN, DIFF ĐỀ XUẤT & CÁCH KIỂM CHỨNG

### Chi tiết 1: Tách luồng OSD và ảnh sạch trong Camera
- **Tệp:** `teleop_quest/synthetic_camera.py`
- **Mô tả:** Hàm `render()` chỉ trả về ảnh RGB nguyên bản của khung cảnh. Tạo thêm hàm `get_annotated_preview()` để phục vụ hiển thị Viser.
- **Diff đề xuất:**
```diff
--- a/teleop_quest/synthetic_camera.py
+++ b/teleop_quest/synthetic_camera.py
@@ -129,12 +129,17 @@ class SyntheticWristCamera:
         cv2.rectangle(img, (int(self.cx + offset_x), self.height - finger_h),
                       (int(self.cx + offset_x + finger_w), self.height), (70, 75, 80), -1)
 
-        # 5. OSD thông tin trạng thái
-        cv2.putText(img, "EYE-IN-HAND CAMERA (SIMULATION)", (15, 25),
+        return img
+
+    def add_osd_preview(self, img_rgb: np.ndarray, gripper_closed: bool) -> np.ndarray:
+        """Tạo bản sao có OSD chỉ dùng hiển thị trên Viser GUI, KHÔNG lưu vào HDF5"""
+        annotated = img_rgb.copy()
+        cv2.putText(annotated, "EYE-IN-HAND CAMERA (SIMULATION)", (15, 25),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 50), 1)
         grip_str = "GRIPPER: CLOSED [1.0]" if gripper_closed else "GRIPPER: OPEN [0.0]"
-        cv2.putText(img, grip_str, (15, 45),
+        cv2.putText(annotated, grip_str, (15, 45),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 140, 0) if gripper_closed else (120, 120, 120), 1)
-
-        return img
+        return annotated
```
- **Kiểm chứng:** Gọi `SyntheticWristCamera.render()` -> ảnh đầu ra không chứa chữ, hiển thị trên Viser qua `add_osd_preview()`.

---

### Chi tiết 2: Khắc phục bù chu kỳ lặp thời gian thực 20Hz
- **Tệp:** `teleop_quest/main_quest.py`
- **Mô tả:** Thay `time.sleep(POLL_SLEEP)` bằng vòng lặp đo chính xác thời gian bắt đầu chu kỳ và chỉ ngủ phần thời gian còn lại.
- **Diff đề xuất:**
```diff
--- a/teleop_quest/main_quest.py
+++ b/teleop_quest/main_quest.py
@@ -219,6 +219,7 @@ def main():
     # 7. Vòng lặp điều khiển chính (20Hz)
     while True:
+        loop_start_time = time.perf_counter()
         # A. Kiểm tra VR E-Stop
         if quest_server.check_and_clear_estop():
@@ -320,7 +321,7 @@ def main():
                 # Cập nhật ảnh trực tiếp lên sidebar Viser để người dùng quan sát bằng mắt
-                ui.update_cam_preview(cam_img)
+                ui.update_cam_preview(wrist_cam.add_osd_preview(cam_img, is_grip_pressed))
@@ -341,5 +342,8 @@ def main():
         else:
             ui.set_status("Tắt")
 
-        time.sleep(POLL_SLEEP)
+        elapsed = time.perf_counter() - loop_start_time
+        sleep_duration = max(0.001, POLL_SLEEP - elapsed)
+        time.sleep(sleep_duration)
```
- **Kiểm chứng:** In `1.0 / (time.perf_counter() - loop_start_time)` mỗi giây, giá trị đạt ổn định $20.0 \pm 0.5\text{ Hz}$.

---

### Chi tiết 3: Tách phím Xoay và Phím Gripper trong VR WebXR
- **Tệp:** `teleop_quest/delta_pose_source.py` & `teleop_quest/main_quest.py`
- **Mô tả:** Đổi `is_rotating` sang nút grip thuần túy, còn đóng mở kẹp đọc từ trường `button_a` / `trigger_button`.
- **Diff đề xuất:**
```diff
--- a/teleop_quest/main_quest.py
+++ b/teleop_quest/main_quest.py
@@ -296,7 +296,8 @@ def main():
                 # Cập nhật logic gắp thả vật thể 3D: Tách biệt hoàn toàn nút kẹp
                 vr_frame = quest_server.get_frame()
-                vr_grip = 1.0 if (vr_frame and vr_frame.get("grip")) else 0.0
+                # Dùng nút trigger_click hoặc button_a cho kẹp, nhường Grip cho xoay cổ tay
+                vr_gripper_btn = 1.0 if (vr_frame and vr_frame.get("gripper_btn", False)) else 0.0
                 mouse_grip = 1.0 if ui.cb_gripper.value else 0.0
-                grip_val = max(vr_grip, mouse_grip)
+                grip_val = max(vr_gripper_btn, mouse_grip)
                 is_grip_pressed = bool(grip_val > 0.5)
```
- **Kiểm chứng:** Thử nghiệm bóp Grip trên Quest -> robot xoay cổ tay nhưng quả cà chua không bị hút dính; bấm nút kẹp riêng biệt -> quả cà chua dính vào gripper.

---

### Chi tiết 4: Tự động ghi nhận nhãn Success từ GraspSimulator & Bổ sung Metadata
- **Tệp:** `teleop_quest/main_quest.py` & `core/data_logger.py`
- **Mô tả:** Không gán cứng `success=True`. Khi bấm dừng record hoặc kết thúc, kiểm tra xem `grasp_sim.placed_count` có tăng trong episode hay không. Bổ sung `initial_tomato_pose` và `seed` vào metadata HDF5.
- **Diff đề xuất:**
```diff
--- a/teleop_quest/main_quest.py
+++ b/teleop_quest/main_quest.py
@@ -124,7 +124,14 @@ def main():
         if ui.cb_record.value:
-            logger.start_episode(task_description="Pick and Place Teleop Demo")
+            init_meta = {
+                "tomato_init_pos": grasp_sim.tomato_pos.tolist(),
+                "basket_pos": grasp_sim.place_basket_pos.tolist(),
+            }
+            logger.start_episode(task_description="Pick and Place Tomato", metadata=init_meta)
             ui.set_status(f"Recording Ep {logger.current_episode_idx}")
         else:
-            saved_file = logger.end_episode(success=True)
+            # Thành công thực sự nếu cà chua đã được đặt vào rổ
+            ep_success = bool(grasp_sim.placed_count > 0 and not grasp_sim.is_grasped)
+            saved_file = logger.end_episode(success=ep_success)
```
- **Kiểm chứng:** Cố tình làm rơi cà chua ra ngoài bàn rồi bấm dừng ghi -> kiểm tra thuộc tính `f.attrs["success"]` trong file HDF5 thấy giá trị `False`. Gắp thả thành công vào rổ -> giá trị `True`.

---

## 5. KỊCH BẢN KIỂM TRA TỰ ĐỘNG CHẤT LƯỢNG DỮ LIỆU (Sanity Check Script)
Tạo script `core/verify_dataset.py` để quét toàn bộ dataset trước khi huấn luyện:
```python
import os, glob, h5py, numpy as np

def verify_dataset(dataset_dir="logs/ai_training_data"):
    files = sorted(glob.glob(os.path.join(dataset_dir, "**/*.hdf5"), recursive=True))
    print(f"--- KIỂM TRA CHẤT LƯỢNG DATASET ({len(files)} files) ---")
    for fpath in files:
        with h5py.File(fpath, 'r') as f:
            T = f.attrs.get("num_steps", 0)
            success = f.attrs.get("success", False)
            qpos = f["observations/qpos"][:]
            action = f["action"][:]
            ts = f["observations/timestamps_ms"][:]
            imgs = f["observations/images/wrist_camera"][:]

            # 1. Kiểm tra NaN / Inf
            assert not np.isnan(qpos).any(), f"Lỗi NaN tại qpos: {fpath}"
            assert not np.isnan(action).any(), f"Lỗi NaN tại action: {fpath}"

            # 2. Kiểm tra độ dài đồng bộ
            assert len(qpos) == len(action) == len(ts) == len(imgs) == T, f"Lệch độ dài các mảng: {fpath}"

            # 3. Kiểm tra biến thiên tần số
            d_ts = np.diff(ts)
            dt_mean, dt_std = np.mean(d_ts), np.std(d_ts)
            print(f"[{os.path.basename(fpath)}] T={T}, Success={success}, dt={dt_mean:.1f}±{dt_std:.1f}ms")
            if dt_std > 10.0:
                print(f"  ⚠️ CẢNH BÁO: Tần số giật cục tại {fpath} (std={dt_std:.1f}ms)")
```

---

## 6. KHUYẾN NGHỊ CHO HUẤN LUYỆN IMITATION LEARNING (ACT / Diffusion Policy)
1. **Thiết lập Camera phụ (Overhead View):** Bắt buộc phải có góc nhìn toàn cảnh thứ ba (Third-person static camera) đặt cố định nhìn xuống bàn làm việc. Camera cổ tay khi gắp sát vật thể sẽ mất tầm nhìn của chiếc rổ.
2. **Action Space:** Khuyến nghị lưu đồng thời cả:
   - `action_joint`: $[q_{target\_0}, ..., q_{target\_5}, gripper]$
   - `action_cartesian`: $[\Delta x, \Delta y, \Delta z, \Delta r_x, \Delta r_y, \Delta r_z, gripper]$
   Điều này cho phép thử nghiệm song song cả Joint-space ACT và Cartesian Diffusion Policy.
3. **Độ mở Gripper:** Chuyển từ nhãn nhị phân $[0.0, 1.0]$ sang giá trị vật lý liên tục (stroke mm từ $0$ đến $85\text{mm}$).
4. **Nén Video:** Tích hợp bộ giải mã H.264 qua `av` hoặc `imageio` để nén video MP4 thay cho mảng uint8 thô trong HDF5, giảm dung lượng dataset từ 20GB xuống dưới 1GB cho 100 demonstrations.
