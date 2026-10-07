# UR3e Teleoperation & Motion Planning System

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![Robotics](https://img.shields.io/badge/Robot-UR3e-orange)
![Framework](https://img.shields.io/badge/Framework-CuRobo%20%7C%20PyRoki-green)
![Status](https://img.shields.io/badge/Status-Active-brightgreen)

Một hệ thống điều khiển tay máy công nghiệp UR3e toàn diện, kết hợp giữa **Điều khiển viễn thao thời gian thực (Real-time Teleoperation)** và **Tự hành né chướng ngại vật (Motion Planning)**. Dự án được tối ưu hóa bằng GPU và thiết kế theo kiến trúc Module chuẩn công nghiệp.

## Tính năng Nổi bật (Key Features)

- **Zero-Latency Teleoperation:** Sử dụng giao thức mạng RTDE (`servoJ` ở 20Hz) cho phép người dùng điều khiển robot bám theo trục tọa độ (Target Widget) trên giao diện Web mà gần như không có độ trễ.
- **Hybrid Dual-IK Solver:** Kiến trúc lai kết hợp sức mạnh của **CuRobo** (né vật cản cứng) và **PyRoki** (JAX-based Oracle kiểm tra tầm với) giúp chẩn đoán chính xác robot đang bị *vướng tường* hay *với không tới*.
- **Auto Motion Planning:** Khi thao tác teleop bị vướng vật cản, chỉ cần nhả chuột, khối Trajectory Planner sẽ tự động thức dậy, vẽ quỹ đạo uốn lượn vượt chướng ngại vật và nội suy lệnh `movej` đẩy robot về đích an toàn.
- **Lớp bảo vệ Động cơ Tối cao (Safety Gate):**
  - **Chống vặn xoắn (Anti-Twist):** Ngăn chặn hiện tượng nhảy nhánh IK (Jump Branch) > 0.4 rad, bảo vệ hộp số động cơ khỏi các cú giật bất ngờ do giới hạn hình học.
  - **Kẹp Vận Tốc (Rate Limiter):** Giới hạn phần mềm delta góc khớp luôn dưới 1.5 rad/s, triệt tiêu lỗi *Protective Stop* từ tủ điều khiển vật lý.
  - **Mở khóa Không gian (Full Range Unlock):** Khai thác tối đa thông số nhà sản xuất (quay $\pm 360^\circ$ cho 5 trục đầu và $\pm 720^\circ$ cho trục cổ tay 3).
- **Vệ sĩ Chạy ngầm (Runtime Monitor & Safe Return):** Luồng song song 20Hz liên tục đo lường ma trận Jacobian. Nếu kỹ sư vô tình bẻ tay máy vào vùng lõi kỳ dị ($\sigma_{min}$ chạm ngưỡng đỏ), hệ thống sẽ lập tức giật quyền điều khiển, hãm phanh khẩn cấp và gọi module `SafeReturn` để rụt tay máy về vị trí Home an toàn.
- **3D Web Interface:** Tích hợp bộ thư viện **Viser**, cho phép người dùng tương tác, kéo thả và giám sát robot thông qua giao diện Web trực quan (không cài cắm rườm rà).

## Kiến trúc Hệ thống (Architecture)

Hệ thống được thiết kế theo mô hình Lego (Modular Design) với 6 khối tách biệt hoàn toàn:

```text
ur_ws/curobo_ursim/
├── main.py                          <-- File chạy chính (Giao diện Web Viser & Main Loop)
├── README.md                        <-- Hướng dẫn 
├── config/                          <-- Cấu hình thông số an toàn (yaml)
├── assets/                          <-- Chứa tài nguyên mô hình 3D, vật cản
│   ├── ur3e_custom.urdf
│   ├── ur3e_custom.yml
│   └── obstacle_scene.yml
├── core/                            <-- Chứa các thư viện và công cụ điều khiển cơ bản
│   ├── curobo_ursim_control.py      <-- Bộ giao tiếp gốc với URSim
│   ├── ur_executor.py               <-- Chuyển đổi lệnh quỹ đạo sang URScript
│   └── data_logger.py               <-- Ghi log Teleop
├── planners/                        <-- Chứa các Module quy hoạch đường đi (Lego blocks)
│   ├── trajectory_planner.py        <-- Module quản lý việc tránh vật cản
│   └── pyroki_mp.py                 <-- Bộ giải nội suy của thư viện PyRoki
├── safety/                          <-- Chứa các Module bảo vệ an toàn
│   ├── runtime_monitor.py           <-- Luồng nền 20Hz giám sát robot
│   ├── safe_return.py               <-- Hàm tự động thu hồi cánh tay về Home
│   └── singularity_guard/           <-- Toán học tính toán ma trận Jacobian
└── teleop/                          <-- Chứa Module tương tác bằng chuột/VR
    └── teleoperation.py             <-- Bộ điều phối Teleop 
```

## Luồng Hoạt động Thời gian thực (Main Loop)

Hệ thống chính (`main.py`) hoạt động như một hệ điều hành với vòng tuần hoàn ở tần số **20Hz (0.05s/vòng)**. Luồng sự kiện được phân nhánh thông minh dựa trên hành vi tương tác của kỹ sư:

1. **Nhánh A (Teleop Bám đuổi):** Khi đang bật tính năng Teleop. Hệ thống trích xuất vị trí Target Widget $\rightarrow$ Gọi IK giải nghiệm $\rightarrow$ Chuyển qua Safety Gate kiểm tra vặn xoắn $\rightarrow$ Phát lệnh `servoj` chớp nhoáng ép robot bám theo trục tọa độ không độ trễ.
2. **Nhánh B (Preview Dò đường):** Khi đang nhấn giữ chuột (Teleop tắt). CuRobo IK tính nháp nghiệm và bắn kết quả lên Web hiển thị "Robot bóng ma", cho phép kỹ sư biết trước đích đến có khả thi hay không trước khi chốt hạ.
3. **Nhánh C (AI Tự hành - Motion Planning):** Khi kỹ sư vừa buông chuột. Khối `TrajectoryPlanner` thức dậy, đánh thức thuật toán nội suy vẽ "sợi dây thun" uốn lượn né qua vật cản. Toàn bộ quỹ đạo được bọc thành chuỗi lệnh `movej` và bơm một lèo xuống tủ điều khiển.
4. **Nhánh D (Thực thi Quỹ đạo):** Trong lúc tay máy tự động bò lách vật cản, hệ thống liên tục đọc vận tốc thực tế (`qd_real`) và đánh giá chỉ số rủi ro kỳ dị (`sigma_min`) để hiển thị cảnh báo.
5. **Vệ sĩ Độc lập (Background Monitor):** Bất kể Main Loop đang làm gì, luồng `RuntimeMonitor` vẫn luôn chạy nền. Nếu phát hiện rủi ro kẹt cơ học (Lõi điểm kỳ dị), luồng này sẽ cướp quyền Main Loop, ngắt Teleop và ép tay máy chạy thoát hiểm (`SafeReturn`).

## Hướng dẫn Cài đặt & Chạy thử (Getting Started)

### 1. Yêu cầu hệ thống (Prerequisites)
- Hệ điều hành: Ubuntu 22.04+
- Phần cứng: NVIDIA GPU (RTX 3060 trở lên khuyến nghị)
- Môi trường: Môi trường ảo Python chứa cài đặt CuRobo, JAX, và Viser.
- Tủ điều khiển thật UR3e hoặc URSim (Mô phỏng máy ảo).

### 2. Thiết lập Chế độ chạy (Mô phỏng vs Robot thật)
Hệ thống mặc định kết nối với máy ảo mô phỏng **URSim** thông qua IP Local. 

**Tùy chọn A: Chạy Mô phỏng (URSim)**
- Đảm bảo URSim (Docker hoặc Máy ảo) đang chạy trên máy tính.
- Để nguyên IP mặc định `127.0.0.1` trong code.

**Tùy chọn B: Chạy Robot thật (Real UR3e)**
- Cắm cáp mạng LAN từ máy tính vào tủ điều khiển (Teach Pendant).
- Mở 2 file sau và đổi thành IP thật của tủ điều khiển (Ví dụ: `192.168.1.xxx`):
  - `core/curobo_ursim_control.py` (Dòng 42)
  - `teleop/teleoperation.py` (Dòng 153)
```python
ROBOT_IP = "192.168.1.xxx" # Nhập IP thật của Teach Pendant
```

### 3. Khởi động hệ thống
Mở Terminal, di chuyển vào thư mục dự án và kích hoạt môi trường ảo (Virtual Environment) trước khi chạy:

```bash
# 1. Chuyển vào thư mục chứa code
cd /home/nguyen/ur_ws/curobo_ursim

# 2. Kích hoạt môi trường ảo Python có chứa CuRobo
source ~/curobo/.venv/bin/activate

# 3. Chạy file gốc của hệ thống
python3 main.py
```
*Dấu hiệu thành công:* Terminal sẽ hiện dòng chữ `[MP] Motion Planner sẵn sàng` và `[Viser] Đã render vật cản`. Máy chủ Web sẽ bắt đầu lắng nghe ở cổng `8080`.

### 4. Hướng dẫn Tương tác chi tiết trên Web UI

Mở trình duyệt Web (Chrome/Firefox) và truy cập địa chỉ: **[http://localhost:8080](http://localhost:8080)**

**Thao tác 1: Khám phá Motion Planning (Robot tự lách vật cản)**
1. Chắc chắn rằng ô `[Bật Teleop]` đang **TẮT**.
2. Dùng chuột click và giữ Trục tọa độ 3D (Target Widget) nằm trước mũi robot.
3. Kéo trục tọa độ đâm thẳng xuyên qua cái hộp đen hoặc đặt ra phía sau hộp đen. Lúc này hệ thống sẽ hiển thị một con "robot bóng ma" (IK Preview) báo hiệu quỹ đạo an toàn.
4. **Nhả chuột ra!** Khối AI sẽ tự động kích hoạt, vạch ra đường đi vòng qua hộp đen và ra lệnh cho robot thật chạy lượn qua tới đích một cách trơn tru.

**Thao tác 2: Khám phá Teleop thời gian thực (Zero-latency)**
1. Nhìn sang bảng Menu bên phải, tích chọn ô **`[Bật Teleop]`**.
2. Lúc này robot được chuyển sang trạng thái "Bám đuổi". Dùng chuột cầm Trục tọa độ 3D và từ từ kéo qua lại.
3. Chú ý quan sát tay máy thật (hoặc trên màn hình), nó sẽ giật và bám theo con chuột của bạn lập tức 20 lần/giây mà không hề có độ trễ.
4. Thử cố tình kéo trục tọa độ một góc vặn vẹo thật khó hoặc sát vào vật cản, Cổng an toàn sẽ lập tức ngắt động cơ và hiện cảnh báo "Lỗi vặn xoắn khớp" để bảo vệ phần cứng.

## Lưu ý An toàn (Safety Warnings)

Do hệ thống sử dụng tập lệnh truyền động liên tục `servoJ`, mô-tơ tay máy sẽ phản ứng cực kỳ bạo lực và không khoan nhượng. 
1. **LUÔN LUÔN** đặt tay lên nút Dừng Khẩn Cấp (E-Stop) của tủ điều khiển.
2. Tuyệt đối không đứng trong bán kính hoạt động 1.5 mét của robot khi đang test Teleop.
3. Nếu mục tiêu (Target Widget) bị xoay một góc quá hẹp, hệ thống bảo vệ Anti-Twist sẽ khóa động cơ và báo lỗi. Đây là tính năng, không phải lỗi.

