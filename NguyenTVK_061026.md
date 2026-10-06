# Báo cáo Cập nhật Kiến trúc Teleoperation Thời gian thực (06/10/2026)

## 1. Tổng quan Bản cập nhật
Trong pha phát triển này, hệ thống đã được tái cấu trúc chuyên sâu ở phân hệ Teleoperation (Điều khiển viễn thao). Mục tiêu cốt lõi là giải quyết triệt để các vấn đề liên quan đến độ trễ truyền động (latency), nội suy bất thường tại điểm kỳ dị (singularity), và bổ sung khả năng tự động né tránh vật cản tĩnh/động trong chế độ thời gian thực (real-time).

Hệ thống nay đã chuyển dịch từ việc nạp kịch bản tĩnh (URScript qua Cổng 30002) sang phương pháp truyền phát liên tục (streaming) qua giao thức RTDE Control, đạt tần số đáp ứng 20Hz khép kín.

## 2. Nâng cấp Luồng Điều khiển (RTDE Control)
Trước đây, hệ thống sinh ra một khối mã URScript (lệnh `movej`) và đẩy xuống cổng 30002. Hạn chế của phương pháp này là độ trễ nội tại của bộ đệm (buffer) và sự ngắt quãng khi nạp chuỗi lệnh mới, gây ra hiện tượng giật cục (jitter) khi người dùng di chuyển liên tục.

**Giải pháp:**
- **Dịch chuyển giao thức:** Ứng dụng hàm `servoJ` thông qua `RTDEControlInterface` (Cổng 30004). Hàm này duy trì một vòng lặp điều khiển chạy nền (background loop) ở cấp độ driver, cho phép hệ thống chỉ việc "bơm" tọa độ khớp mới vào bộ đệm thời gian thực mà không làm gián đoạn quỹ đạo hiện tại.
- **Tinh chỉnh Động học:**
  - `lookahead_time` được ép xuống mức `0.04s` (tối ưu cho chu kỳ 50ms của 20Hz) giúp robot bám sát quỹ đạo ngay lập tức thay vì làm mượt quá độ.
  - `gain` được tăng lên mức `1000`, buộc vòng lặp PID của bộ điều khiển công nghiệp siết chặt độ bám vị trí.

## 3. Kiến trúc Giải động học Lai (Hybrid IK) & Tránh vật cản
Một điểm yếu chí mạng của các bộ giải IK dựa trên Jacobian (như PyRoki) là sự "mù lòa" trước vật cản. Để tích hợp khả năng né vật cản vào Teleoperation mà không làm suy giảm tốc độ, hệ thống sử dụng kiến trúc lai:

- **CuRobo làm Cốt lõi an toàn:** CuRobo IK được khởi tạo kèm mô hình môi trường (`obstacle_scene.yml`). Mọi tọa độ Cartesian đầu vào đều được quét qua CuRobo. Nếu mục tiêu đâm xuyên vật cản, thuật toán lập tức loại bỏ cấu hình và trả về "Vô nghiệm" (None), giúp khóa đứng tay máy trước khi xảy ra va chạm.
- **PyRoki làm Oracle dự báo (Reachability Oracle):** Để phân biệt giữa lỗi "Đâm vật cản" và "Ngoài tầm với" (đều làm CuRobo trả về None), hệ thống gọi song song PyRoki. Vì PyRoki không kiểm tra vật cản, nếu PyRoki tính ra nghiệm nhưng CuRobo báo Vô nghiệm $\rightarrow$ Hệ thống suy luận logic chắc chắn tay máy đang bị chặn bởi tường.

## 4. Cơ chế Bảo vệ Động cơ & Kìm hãm Điểm kỳ dị
Trong chế độ viễn thao, các chuyển động ngẫu nhiên của con người rất dễ đưa robot vào khu vực kỳ dị (Singularity) hoặc tạo ra các bước nhảy góc quá lớn, gây quá tải hộp số.

### 4.1. Hạn mức Vặn xoắn (Singularity Jump Limit)
- Khi tiến sát điểm kỳ dị, bộ giải IK thường có xu hướng "nhảy nhánh" (ví dụ: bẻ ngược khuỷu tay) để duy trì tọa độ TCP. Hệ thống hiện giám sát sai phân giữa nghiệm IK mới (`q_target`) và dấu chân an toàn trước đó (`last_valid_q`). 
- Nếu góc nhảy vượt ngưỡng **0.4 radian (~23 độ)**, hệ thống lập tức phong tỏa lệnh (Reject) và phát tín hiệu `Teleop: Tu the ket (Loi van xoan khop)`, ngăn chặn triệt để hiện tượng vung vẩy mất kiểm soát do lỗi toán học gây ra. Việc dùng `last_valid_q` làm gốc tham chiếu cũng giải quyết được rắc rối do quán tính vật lý của động cơ làm `actual_q` bị trễ (lag) so với lệnh nội suy.

### 4.2. Bộ kẹp Vận tốc (Rate Limiter)
- **Rào chắn Phần mềm:** Tích hợp thuật toán cắt xén (clamping) trực tiếp lên delta góc khớp. 
- Vận tốc tối đa của mỗi khớp được khóa cứng ở mức **1.5 rad/s (~85 độ/giây)**. Nếu người dùng vẩy chuột với tốc độ cực cao, lệnh gửi xuống robot sẽ tự động bị phân mảnh, chỉ cho phép tiến thêm tối đa `0.075 rad` mỗi chu kỳ 50ms. Điều này đảm bảo tay máy lướt đi mượt mà, triệt tiêu gia tốc xung kích và ngăn chặn triệt để lỗi *Protective Stop* từ tủ điều khiển vật lý.

## 5. Cải tiến Giao diện & Trải nghiệm
- **Chuẩn hóa Thông báo:** Loại bỏ hoàn toàn các biểu tượng cảm xúc (emoji) để định hình văn phong cảnh báo chuẩn công nghiệp.
- **Tách bạch Báo lỗi:** Hệ thống nay có khả năng phân tách rạch ròi 4 trạng thái UI độc lập để kỹ sư vận hành dễ dàng nhận diện vấn đề mà không cần truy xuất Log hệ thống:
  1. Dòng lệnh thông suốt (`Teleop [CuRobo] | sigma=...`)
  2. Bắt gặp điểm kỳ dị hẹp (`Teleop: Tu the ket (Loi van xoan khop) - Dung im`)
  3. Va chạm vật thể cứng (`Teleop: Dung vat can - Dung im`)
  4. Vượt biên giới hạn động học (`Teleop: Ngoai tam voi - Dung im`)

## 6. Sơ đồ Kiến trúc Module & Dòng chảy Dữ liệu (Data Flow)
Hệ thống Teleoperation được cấu trúc hóa thành 6 khối (Module) độc lập, với I/O định nghĩa rõ ràng nhằm hỗ trợ mở rộng (ví dụ: cắm thêm VR Controller) mà không làm phá vỡ lõi điều khiển.

```text
[Khối 1: Nguồn Tọa độ] (Chuột, VR Controller)
         │
         ▼ (pose: pos, quat)
[Khối 2: Bộ giải IK Kép] ──(Gọi song song)──► [2A: CuRobo IK] (Có né vật cản)
         │                                   [2B: PyRoki IK] (Oracle kiểm tra tầm với)
         ▼ (q_curobo, q_pyroki)
[Khối 3: Bộ điều phối & Nội suy] (Rate Limiter, Chống nhảy nhánh IK)
         │
         ▼ (q_target)
[Khối 4: Cổng An toàn - SafetyGate] (Bảo vệ điểm kỳ dị SVD)
         │
         ▼ (q_target đã xác thực)
[Khối 5: Executor - Trình thực thi] (Truyền động RTDE servoJ)
         │
         ▼ (Giao thức mạng TCP/RTDE)
[Khối 6: Tủ điều khiển Robot Thực]
```

### Chi tiết I/O từng khối:
- **Khối 1 (Pose Source):** 
  - *Input:* Tín hiệu vật lý từ thiết bị ngoại vi.
  - *Output:* `{"pos": [X,Y,Z], "quat": [Qw,Qx,Qy,Qz]}`.
- **Khối 2 (Dual IK):** 
  - *Input:* `pose` và góc khớp thực `q_real`.
  - *Output:* Mảng góc khớp `[q1...q6]` hoặc `None`.
- **Khối 3 (Controller):** 
  - *Input:* Nghiệm từ CuRobo, PyRoki và góc tham chiếu `last_valid_q`.
  - *Output:* Góc khớp mục tiêu `q_target` đã được kẹp vận tốc (< 1.5 rad/s) và chống xoắn.
- **Khối 4 (Safety Gate):** 
  - *Input:* `q_target`.
  - *Output:* Quyết định Pass/Reject dựa trên chỉ số rủi ro kỳ dị (`sigma_min`).
- **Khối 5 (RTDE Executor):** 
  - *Input:* Góc khớp mục tiêu an toàn.
  - *Output:* Tín hiệu dòng điện điều khiển khớp ở tần số 20Hz.
