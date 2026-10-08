# Báo cáo 07/10/2026

## 1. Giới thiệu chung
Hệ thống Teleoperation triệt tiêu độ trễ truyền động (latency), xử lý điểm kỳ dị (singularity) và tránh vật cản thời gian thực. Hệ thống nay đã chuyển sang giao thức truyền phát liên tục (streaming) khép kín ở tần số 20Hz.

### Kết quả Đạt Được & Video Demo
Đã điều khiển thành công tay máy công nghiệp trên trình giả lập URSim. Tay máy phản hồi tức thì với thiết bị ngoại vi, đáp ứng hoàn toàn chuẩn thời gian thực không độ trễ.

**Nhận xét:** Thử nghiệm thực địa cho thấy lõi nội suy bám sát mục tiêu ổn định ở tần số 20Hz. Quỹ đạo khớp mượt mà, không giật cục (jitter) hay phát sinh lỗi khóa phần cứng (Protective Stop).

**Tài liệu đính kèm:** [Video Demo Thực nghiệm](https://fptsoftware362-my.sharepoint.com/:v:/g/personal/nguyentvk_fpt_com/IQDAH7Yz6dGnT7jjFnt5ze4vAcuA6Eir54GqtrfO8T-h8ug?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=TcKvLM)

## 2. Nâng cấp Luồng Điều khiển (RTDE)
- **Truyền tải Liên tục:** Thay thế phương pháp nạp lệnh `movej` truyền thống bằng giao thức `servoJ` qua luồng dữ liệu RTDE (Cổng 30004). Cơ chế này duy trì vòng lặp điều khiển chạy nền ở cấp độ driver, cho phép cập nhật tọa độ liên tục vào bộ đệm (buffer) mà không gây gián đoạn hay giật cục (jitter) khi tái định tuyến.
- **Tinh chỉnh Động học:** Giảm tham số `lookahead_time` xuống `0.04s` và tối đa hóa `gain` của vòng lặp PID (`gain=1000`). Thiết lập này ép bộ điều khiển công nghiệp bám sát quỹ đạo vi phân ngay lập tức thay vì áp dụng bộ lọc làm mượt quá mức, đáp ứng chính xác chu kỳ điều khiển 50ms của hệ thống.

## 3. Kiến trúc Giải động học
Nhằm khắc phục nhược điểm "mù lòa" trước vật cản của các bộ giải IK cục bộ, hệ thống áp dụng cơ chế đánh giá kép:
- **CuRobo (Bảo vệ Không gian):** Đóng vai trò lõi kiểm tra va chạm toàn cục dựa trên mô hình môi trường 3D tĩnh. Mọi lệnh viễn thao sẽ bị phong tỏa và loại bỏ cấu hình nếu quỹ đạo nội suy có nguy cơ giao cắt với không gian vật cản.
- **PyRoki (Định tuyến Cục bộ):** Hoạt động song song như một *Reachability Oracle*. Vì PyRoki không kiểm tra vật cản, sự đối chiếu kép cho phép hệ thống phân tách rạch ròi trạng thái lỗi: ngắt truyền động do tay máy chạm vách vật cản (CuRobo trả None, PyRoki có nghiệm) hay do tọa độ nằm ngoài ranh giới không gian làm việc (workspace boundary).

## 4. Cơ chế Bảo vệ Truyền động
- **Hạn mức Vặn xoắn (Singularity Jump Limit):** Thuật toán liên tục giám sát sai phân lân cận giữa nghiệm IK hiện tại (`q_target`) và dấu chân an toàn trước đó (`last_valid_q`). Nếu gia số góc khớp vượt ngưỡng 0.4 radian (~23 độ), lệnh sẽ bị từ chối nhằm ngăn ngừa hiện tượng "nhảy nhánh" thuật toán tại vùng kỳ dị hẹp, tránh gây ứng suất cắt đột ngột lên hộp số.
- **Bộ Kẹp Vận tốc (Rate Limiter):** Vận tốc nội suy của các khớp được khóa cứng ở mức tối đa 1.5 rad/s. Mọi biến thiên tọa độ cực đoan từ thiết bị ngoại vi (độ trễ nhiễu sóng, vung tay đột ngột) đều bị cắt xén (clamping) xuống mức an toàn (< 0.075 rad/chu kỳ). Cơ chế này triệt tiêu hoàn toàn gia tốc xung kích và loại trừ triệt để lỗi *Protective Stop* từ phần cứng tủ điều khiển.

## 5. Sơ đồ Kiến trúc Module & Dòng chảy Dữ liệu (Data Flow)
Hệ thống Teleoperation được Module hóa thành 6 khối độc lập, sở hữu giao thức I/O định dạng chuẩn nhằm hỗ trợ mở rộng phần cứng (VR Controllers, Haptic Gloves) mà không phá vỡ lõi điều khiển.

```text
[Khối 1: Nguồn Tọa độ] (Web, VR Controller)
         │
         ▼ (pose: pos, quat)
[Khối 2: Bộ giải IK Kép] ──(Gọi song song)──► [CuRobo IK] (Kiểm tra va chạm)
         │                                   [PyRoki IK] (Oracle tầm với)
         ▼ (q_curobo, q_pyroki)
[Khối 3: Bộ điều phối & Nội suy] (Rate Limiter, Chống nhảy nhánh IK)
         │
         ▼ (q_target)
[Khối 4: Cổng An toàn - SafetyGate] (Bảo vệ điểm kỳ dị theo chỉ số SVD)
         │
         ▼ (q_target đã xác thực)
[Khối 5: Executor - Trình thực thi] (Truyền động RTDE servoJ 20Hz)
         │
         ▼ (Giao thức mạng TCP/RTDE)
[Khối 6: Tủ điều khiển Robot Thực] (UR3e / URSim)
```

## 6. Cấu trúc Vòng lặp Trung tâm
Luồng sự kiện được phân nhánh vi phân nghiêm ngặt theo chu kỳ 0.05s, đảm bảo tính tất định (determinism) của phần mềm thời gian thực:
1. **Nhánh Teleop (Bám đuổi Real-time):** Trích xuất tọa độ mục tiêu $\rightarrow$ Tính toán Hybrid IK $\rightarrow$ Vượt qua Safety Gate $\rightarrow$ Đẩy lệnh `servoJ` trực tiếp xuống robot vật lý.
2. **Nhánh Preview (Dò đường ảo):** Kích hoạt khi chế độ Teleop tắt (nhấp giữ chuột). Tạm hoãn robot vật lý, chỉ phản hồi cấu hình IK hợp lệ lên giao diện Web (Ghost Robot).
3. **Nhánh Motion Planning (Lập quỹ đạo):** Kích hoạt khi vận tốc mục tiêu bằng 0. Hệ thống tự động quy hoạch quỹ đạo nội suy (B-Spline) lách qua vật cản thông qua mạng lưới đồ thị.
4. **Nhánh Execution (Giám sát Thực thi):** Phân phối quỹ đạo trọn gói xuống bộ điều khiển, đồng thời liên tục giám sát sai số thực tế (`actual_q`) và chỉ số kỳ dị (`sigma_min`) từ dòng chảy dữ liệu trả về.

## 7. Hướng dẫn Khởi chạy Cơ bản
Yêu cầu khởi tạo môi trường song song thông qua Terminal.

### Terminal 1: Khởi động hệ thống điều khiển URSim ảo
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e -f "-p 30001-30004:30001-30004 -p 29999:29999 -p 6080:6080"
```
*(Nếu gặp lỗi mạng Docker, sử dụng lệnh: `docker restart ursim`)*

### Terminal 2: Kích hoạt Lõi Điều khiển
```bash
# Chuyển hướng thư mục và kích hoạt môi trường CuRobo
cd ~/ur_ws/curobo_ursim
source ~/curobo/.venv/bin/activate
python3 main.py
```
