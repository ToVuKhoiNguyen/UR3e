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
- **Auto-Snap & VR Haptics:** Phản hồi xúc giác (Haptic) rung tay cầm lập tức khi robot chạm vật cản, mất nội suy, hoặc kẹt kỳ dị. Đặc biệt, nếu robot bị kẹt hoặc nằm ngoài tầm với quá 2.0 giây, ngàm 3D ảo sẽ tự động giật lùi (Auto-Snap) về lại đúng vị trí vật lý hiện tại của tay máy thực.
- **3D Web Interface (Modern Industrial):** Giao diện điều khiển Viser tinh giản theo hướng công nghiệp chuyên nghiệp, hỗ trợ nút quy hoạch về Home "một chạm" khép kín (tự động ngắt, chạy về, và tự động bật lại hệ thống Teleop).

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

### 4. Hướng dẫn Chạy VR Teleop Toàn tập (Meta Quest 3S)

Hệ thống giờ đây được nâng cấp mặc định hỗ trợ điều khiển cánh tay Robot bằng kính Meta Quest 3S thông qua công nghệ WebXR, AR Xuyên thấu (Passthrough) và Buồng lái ảo (Virtual Cockpit).

**Bước 1: Khởi động Máy ảo URSim (Nếu chạy mô phỏng)**
Mở Terminal 1 và chạy lệnh để tự động tải & bật giả lập UR3e:
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```

**Bước 2: Khởi động Server VR & Giao diện Điều khiển**
Mở Terminal 2, di chuyển vào thư mục gốc và khởi động hệ thống chính:
```bash
cd /home/nguyen/ur_ws/curobo_ursim
source ~/curobo/.venv/bin/activate
python3 main.py
```
*(Hệ thống sẽ **tự động dò tìm địa chỉ IP hiện tại của máy tính**, tự động nặn chứng chỉ SSL tương ứng và in đường link truy cập ra màn hình Terminal. Khi bạn đổi mạng Wi-Fi/Hotspot, hệ thống cũng sẽ tự động làm lại từ đầu).*

**Bước 3: Đeo Kính & Vào Buồng Lái Ảo (Virtual Cockpit)**
Do sử dụng chứng chỉ bảo mật tự tạo (Self-signed) cho IP LAN, bạn cần "Thông chốt" 1 lần duy nhất cho mỗi IP mới:
1. Đeo kính Quest (đảm bảo chung mạng Wi-Fi Hotspot với Laptop).
2. Mở trình duyệt Web của Quest, truy cập **Cổng Dữ Liệu Ngầm**: `https://[IP_HIỆN_TẠI]:8444` $\rightarrow$ Trình duyệt cảnh báo đỏ $\rightarrow$ Bấm **Advanced $\rightarrow$ Proceed** $\rightarrow$ Đóng Tab.
3. Truy cập **Cổng Giao Diện**: `https://[IP_HIỆN_TẠI]:8443` $\rightarrow$ Trình duyệt cảnh báo đỏ $\rightarrow$ Bấm **Advanced $\rightarrow$ Proceed**.
4. Trải nghiệm màn hình **Welcome Screen phong cách Light/Modern công nghiệp** hiện ra (nền xám khói sang trọng). Bấm nút **START IMMERSIVE VR**.
5. Đeo kính và cấp quyền WebXR. Xung quanh bạn sẽ chuyển sang chế độ AR nhìn xuyên thấu, đồng thời Bảng điều khiển ảo (Virtual Cockpit) sẽ hiển thị các thẻ thông số Robot Pose.

**Bước 4: Cơ chế Điều khiển Phân tách (Decoupled Controls & World-Centric)**
- **Kích hoạt Teleop:** Trên giao diện điều khiển (PC hoặc VR), tích chọn ô **`Bat VR Teleop`**.
- **Cơ chế Phân tách Điều khiển:** Hệ thống chia tách rõ rệt chuyển động để triệt tiêu rung nhiễu:
  - **Chỉ bóp Cò (Trigger):** Kích hoạt **Chỉ Tịnh Tiến**. Khóa chết trục xoay. Áp dụng hệ quy chiếu **World-Centric** (không gian thực). Đẩy tay cầm thẳng tới trước $\rightarrow$ robot đâm thẳng tới trước dọc theo trục X toàn cục, bất kể tay cầm của bạn đang bị nghiêng. Đưa tay lên trời $\rightarrow$ Robot đi thẳng lên trời.
  - **Chỉ bóp Nút Hông (Grip):** Kích hoạt **Chỉ Xoay**. Khóa chết tịnh tiến. Áp dụng hệ quy chiếu **Tool-Centric**. Robot neo chặt tọa độ, xoay mượt mà cổ tay dọc theo các trục của chính nó (chuẩn công nghiệp).
  - **Bóp cả 2 Nút:** Chuyển động tự do 6D (vừa đẩy vừa xoay).
- **Côn Thông Minh (Smart Clutch):** Khi chuyển qua lại giữa các nút bấm, hệ thống tự động thả lại mỏ neo hệ quy chiếu, loại bỏ hoàn toàn các điểm giật cục (Teleportation).
- **Snapping/Deadband:** Áp dụng Deadband tĩnh (1.5cm) và tính năng bám trục thẳng (Snapping) giúp bạn kéo những đường cắt laser mượt mà, thẳng tắp trong không trung.

## Lưu ý An toàn (Safety Warnings)

Hệ thống đã được tinh chỉnh thông số (Scale 0.7, EMA Filter 0.1 mượt mà, và Max Velocity Clamp 1.2 rad/s) để đảm bảo độ êm ái nhưng vẫn cực kỳ "bốc" khi chạy trên robot thật. Tuy nhiên:
1. **LUÔN LUÔN** đặt tay lên nút Dừng Khẩn Cấp (E-Stop) của tủ điều khiển.
2. Tuyệt đối không đứng trong bán kính hoạt động 1.5 mét của robot khi đang test Teleop.
3. Nếu tay cầm rung mạnh (Haptics) báo hiệu lỗi vật cản hoặc kỳ dị, hãy lập tức nhả cò để ngàm 3D tự động snap về vị trí an toàn.

