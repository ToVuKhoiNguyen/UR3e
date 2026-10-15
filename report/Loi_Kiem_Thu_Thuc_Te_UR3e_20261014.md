# Báo Cáo Ghi Nhận Lỗi Kiểm Thử Trên Robot Thật & Hướng Dẫn Khắc Phục (14/10/2026)

## 1. Giới thiệu chung
Tài liệu này ghi nhận chi tiết toàn bộ các lỗi phát sinh trong quá trình triển khai chạy thử nghiệm hệ thống điều khiển viễn thao trên cánh tay robot Universal Robots UR3e thực tế (IP: `10.42.0.242`), phân tích cơ chế nguyên nhân kỹ thuật và đưa ra hướng dẫn khắc phục từng bước.

---

## 2. Bảng Tổng Hợp Lỗi Phát Sinh

| STT | Mã / Thông báo lỗi | Vị trí phát sinh | Mức độ | Trạng thái |
| :--- | :--- | :--- | :--- | :--- |
| 1 | `ur_rtde: Please enable remote control on the robot!` | `ServoJExecutor.connect()` | Nghiêm trọng (Chặn lệnh) | Đã có hướng dẫn xử lý |
| 2 | `Dừng - singularity(σ=0.0291)` -> `Auto-Snap` liên tục | `SafetyGate.check()` | Cảnh báo an toàn (Chặn lệnh) | Đã có hướng dẫn xử lý |
| 3 | `RTDEReceiveInterface boost system Exception: (system:125) Operation canceled` | `ur_rtde` C++ Binding | Thông tin (Hủy tiến trình) | Bình thường khi Ctrl+C |

---

## 3. Phân Tích Chi Tiết Từng Lỗi & Hướng Dẫn Khắc Phục

### Lỗi 1: Bộ Điều Khiển Robot Từ Chối Lệnh Điều Khiển RTDE

#### 1. Nhật ký ghi nhận (Log):
```text
[ServoJ] Lỗi kết nối RTDE Control: ur_rtde: Please enable remote control on the robot!
[Teleop] KÍCH HOẠT ĐIỀU KHIỂN.
[ServoJ] Lỗi kết nối RTDE Control: ur_rtde: Please enable remote control on the robot!
```

#### 2. Nguyên nhân kỹ thuật:
- Universal Robots tách biệt 2 giao thức qua cổng mạng RTDE:
  - **RTDEReceive (Port 30004):** Đọc trạng thái cảm biến (vị trí góc khớp, TCP, dòng điện) - Luôn mở tự do. Vì vậy hệ thống vẫn đọc được: `[Setup] Connected. Joints: [-2.2443, -0.5997, -1.8481, -2.5082, 1.6212, 3.5086]`.
  - **RTDEControl (Lệnh servoJ / speedJ):** Kênh ghi lệnh điều khiển động cơ - **Bắt buộc robot phải ở trạng thái Remote Control**.
- Hiện tại trên màn hình điều khiển Teach Pendant (PolyScope), robot vẫn đang ở chế độ cục bộ (**Local Mode**), do đó bộ điều khiển URControl tự động từ chối thiết lập socket ghi lệnh, biến `self._ctrl` trong `ServoJExecutor` nhận giá trị `None` và không thể phát xung điều khiển.

#### 3. Các bước khắc phục trên Teach Pendant (PolyScope):
1. **Bước 1 - Cấp quyền Remote Control trong cài đặt hệ thống:**
   - Vào menu: **Settings (Cài đặt)** -> **System (Hệ thống)** -> **Remote Control (Điều khiển từ xa)**.
   - Chọn mục **Enable (Bật)** -> Bấm lưu/áp dụng.
2. **Bước 2 - Chuyển chế độ vận hành trên màn hình chính:**
   - Quan sát góc trên cùng bên phải màn hình PolyScope (cạnh đồng hồ hệ thống).
   - Nhấp vào biểu tượng tay dạy đang hiển thị chữ **Local**.
   - Chọn chuyển sang **Remote Control** (Biểu tượng chuyển sang hình máy tính/mạng).
3. **Bước 3 - Bật nguồn và nhả phanh:**
   - Nhấp vào nút trạng thái robot ở góc dưới cùng bên trái màn hình.
   - Bấm **Power On** -> Bấm **Release Brakes** (Đèn trạng thái chuyển sang màu xanh lá cây `RUNNING`).

---

### Lỗi 2: Robot Đang Đỗ Ở Tư Thế Kỳ Dị (Singularity Trap) & Kích Hoạt Auto-Snap Liên Tục

#### 1. Nhật ký ghi nhận (Log):
```text
[Auto-Snap] Lỗi vặn xoắn / vướng vật cản (Dừng - singularity(σ=0.0291)) -> Tự động thu hồi XYZ về đầu tay máy!
[Auto-Snap] Lỗi vặn xoắn / vướng vật cản (Dừng - singularity(σ=0.0292)) -> Tự động thu hồi XYZ về đầu tay máy!
```

#### 2. Nguyên nhân kỹ thuật:
- Trạng thái 6 góc khớp của robot lúc bật máy:
  - Khớp 1 (Base): `-2.2443` rad ($-128.6^\circ$)
  - Khớp 2 (Shoulder): `-0.5997` rad ($-34.4^\circ$)
  - Khớp 3 (Elbow): `-1.8481` rad ($-105.9^\circ$)
  - Khớp 4 (Wrist 1): `-2.5082` rad ($-143.7^\circ$)
  - Khớp 5 (Wrist 2): `+1.6212` rad ($+92.9^\circ$)
  - Khớp 6 (Wrist 3): `+3.5086` rad ($+201.0^\circ$)
- Module `SingularityGuard` tính toán giá trị suy biến nhỏ nhất của ma trận Jacobian:
  - Kết quả đo được: $\sigma_{min} = \mathbf{0.0291}$.
  - Ngưỡng an toàn quy định tại `config/singularity.yaml`: $\sigma_{danger} = \mathbf{0.0350}$.
- Vì $\sigma_{min} < 0.0350$, hệ thống xác định robot đang ở tư thế cận kề điểm kỳ dị (vận tốc góc khớp cục bộ bị khuếch đại vô hạn nếu di chuyển theo một số hướng).
- Lớp bảo vệ `SafetyGate` chủ động chặn toàn bộ lệnh di chuyển để bảo vệ hộp số robot. Khi thời gian kẹt vượt quá $1.2$ giây, cơ chế `Auto-Snap` liên tục kéo Target Widget về ngàm bích robot thật để chống trôi tọa độ.

#### 3. Các bước khắc phục:
1. **Đưa robot về vị trí làm việc tự nhiên (Move to Ready/Home Pose):**
   - Trên Teach Pendant, chuyển tạm về **Local Mode** -> Vào tab **Move (Di chuyển)**.
   - Bấm giữ các phím điều khiển khớp để đưa robot về tư thế gập tự nhiên, tránh duỗi thẳng hoặc gập kẹp các khớp cổ tay:
     - Khớp 1 (Base): $0^\circ$ (`0.0` rad)
     - Khớp 2 (Shoulder): $-90^\circ$ (`-1.57` rad) hoặc $-126^\circ$ (`-2.2` rad)
     - Khớp 3 (Elbow): $+90^\circ$ (`+1.57` rad) hoặc $+109^\circ$ (`+1.9` rad)
     - Khớp 4 (Wrist 1): $-90^\circ$ (`-1.57` rad)
     - Khớp 5 (Wrist 2): $-90^\circ$ (`-1.57` rad)
     - Khớp 6 (Wrist 3): $0^\circ$ (`0.0` rad)
   - Tại tư thế này, giá trị $\sigma_{min} \approx 0.12 - 0.18$ (hoàn toàn vượt xa ngưỡng an toàn $0.035$).
2. **Chuyển lại sang Remote Control:**
   - Sau khi robot đã ở tư thế thoáng đãng, chuyển lại góc trên bên phải PolyScope sang **Remote Control**.

---

### Lỗi 3: Ngoại Lệ Ngắt Tiến Trình (Operation Canceled)

#### 1. Nhật ký ghi nhận (Log):
```text
RTDEReceiveInterface boost system Exception: (system:125) Operation canceled
```

#### 2. Nguyên nhân kỹ thuật:
- Đây là ngoại lệ tiêu chuẩn từ thư viện `boost::asio` bên dưới của wrapper C++ `ur_rtde` khi tiến trình nhận tín hiệu dừng đột ngột (người dùng nhấn `Ctrl+C` trong terminal để tắt code).
- Thư viện hủy các kết nối TCP/IP đang chờ nhận dữ liệu và thông báo mã lỗi hệ thống 125 (`ECANCELED`).

#### 3. Hướng dẫn xử lý:
- Đây là hành vi bình thường khi tắt chương trình, không ảnh hưởng đến phần cứng hay mã nguồn.

---

## 4. Quy Trình Khởi Động Chuẩn Sau Khi Khắc Phục

1. **Khởi động mã nguồn trên PC:**
   ```bash
   cd /home/nguyen/ur_ws/curobo_ursim
   source ~/curobo/.venv/bin/activate
   python3 main.py --robot-ip 10.42.0.242
   ```

2. **Dấu hiệu nhận biết khởi động thành công trên terminal:**
   ```text
   [Setup] Connected. Joints: [...]
   [ServoJ] RTDEControl connected: True
   [Teleop] KÍCH HOẠT ĐIỀU KHIỂN.
   ```
   *(Không còn xuất hiện dòng báo lỗi Remote Control hay Auto-Snap Singularity).*

3. **Thao tác điều khiển trên trình duyệt:**
   - Mở `http://localhost:8080`.
   - Tích chọn **Bật Teleop**.
   - Kéo nhẹ mũi tên trục Z màu xanh dương trên Target Widget để kiểm chứng cánh tay robot thật nhấc lên theo chuột.
