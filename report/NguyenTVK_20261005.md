# Chạy CuRobo và PyRoki kết nối URSim

## 1. Giới thiệu Chung
Dự án tập trung phát triển kiến trúc điều khiển robot thời gian thực thông qua việc tích hợp hai mô hình trí tuệ nhân tạo: **CuRobo** (toán học song song trên GPU CUDA) và **PyRoki** (biên dịch JIT siêu tốc trên CPU). Hệ thống được thiết kế để nhận tín hiệu không gian liên tục từ thiết bị ngoại vi (giao diện Web/Thực tế ảo) nhằm điều hướng giả lập URSim. Quá trình di chuyển được đảm bảo an toàn tuyệt đối thông qua hệ thống bảo vệ 5 lớp giúp triệt tiêu các lỗi động học.

### Kết quả Đạt Được & Video Demo
Hệ thống đã thiết lập thành công luồng điều khiển khép kín với URSim, đáp ứng chuẩn thời gian thực, tự động né vật cản chính xác và không phát sinh lỗi khóa an toàn phần cứng. 

**Nhận xét:** Trong các thử nghiệm thực tế, khi mục tiêu bị che khuất, thuật toán tối ưu cục bộ của PyRoki thường bị mắc kẹt. Nhờ bản chất tìm kiếm toàn cục, CuRobo luôn tính toán ra quỹ đạo né tránh an toàn. Việc tích hợp kép giải quyết triệt để nhược điểm của cả hai hệ thống.

**Tài liệu đính kèm:** [Video Demo](https://fptsoftware362-my.sharepoint.com/:f:/g/personal/nguyentvk_fpt_com/IgAyON8YWBLYR5Mr5BghTh5fAdGI6qK5GViX_Pd3XNnUOBo?e=TobWh4)

## 2. Phân tích hai phương án tiếp cận
Sự kết hợp song song giúp hệ thống bù trừ trực tiếp giới hạn về phần cứng và bản chất thuật toán cho cả hai bài toán cốt lõi:

### Động học nghịch (Inverse Kinematics - IK)
- **PyRoki (Google):** Giải IK trên CPU thông qua biên dịch JIT, loại bỏ độ trễ truyền tải RAM-VRAM. Thời gian hội tụ nghiệm siêu tốc (**0.1 - 0.3 ms**), đảm nhận xuất sắc vai trò bám sát quỹ đạo liên tục (VR/Haptic). Tuy nhiên, dễ mắc kẹt ở điểm cực tiểu địa phương do dựa trên tối ưu đạo hàm (gradient).
- **CuRobo (NVIDIA):** Giải IK bằng toán học ma trận Tensor trên GPU. Dù tồn tại độ trễ truyền tải (`4 - 8 ms`), phương pháp tìm kiếm toàn cục (global search) đảm bảo luôn tìm ra cấu hình khớp hợp lệ cho các mục tiêu có biến thiên vị trí lớn, hỗ trợ thoát kẹt cho PyRoki.

### Lập kế hoạch quỹ đạo (Motion Planning - MP)
- **CuRobo (NVIDIA):** Đóng vai trò hoa tiêu chiến lược. Thuật toán quét đồ thị phân tích không gian cấu hình để né mọi vật cản, làm mượt bằng đường cong B-Spline. Đảm bảo tính toàn vẹn tuyệt đối nhưng tốn thời gian tính toán (`40 - 100 ms`), có thể gây gián đoạn vi phân nếu chạy vòng lặp kín.
- **PyRoki (Google):** Tối ưu hóa quỹ đạo cục bộ cực nhanh (`~15 ms`). Dù bản thân dễ đâm vào chướng ngại vật lớn, nhưng khi được "mớm" dữ liệu hạt giống (seeding) từ CuRobo, PyRoki tận dụng tốc độ để bám sát lộ trình an toàn một cách hoàn hảo.

### Bảng So sánh Hiệu năng

| Tiêu chí | CuRobo (NVIDIA) | PyRoki (Google) |
| :--- | :--- | :--- |
| **Phần cứng xử lý** | GPU (Toán học Tensor) | CPU (Biên dịch JIT) |
| **Độ trễ tính toán** | 4.0 - 8.0 ms | **0.1 - 0.3 ms** |
| **Lập quỹ đạo (MP)** | 40 - 100 ms | **10 - 15 ms** |
| **Bản chất Thuật toán** | Tìm kiếm toàn cục | Tối ưu hóa cục bộ |
| **Ưu điểm Cốt lõi** | Đảm bảo tính hội tụ, né vật cản lớn. | Độ trễ siêu thấp, bám sát tốt. |
| **Hạn chế Đo lường** | Gây gián đoạn vi phân nếu chạy liên tục. | Dễ mắc kẹt tại điểm cực tiểu. |
| **Vai trò Hệ thống** | Lập quy hoạch chiến lược. | Bám sát quỹ đạo thực thời. |

## 3. Luồng hoạt động & Luồng Dữ liệu
Luồng xử lý từ lúc người dùng thao tác đến khi robot thực thi được thiết kế chặt chẽ qua 3 bước:

- **Bước 1 - Quan sát và Theo vết (Preview/Tracking):** 
  - *Dữ liệu truyền vào:* Giao diện Web gửi mảng tọa độ 6 bậc tự do `[X, Y, Z, Qw, Qx, Qy, Qz]` (sử dụng quaternion cho góc xoay) thông qua WebSocket ở tần số 20Hz.
  - *Xử lý cốt lõi:* PyRoki tiếp nhận mảng tọa độ này, giải bài toán động học nghịch (IK) trong vòng 0.1 ms để tìm ra mảng 6 góc khớp mục tiêu `[q1, q2, q3, q4, q5, q6]` (đơn vị Radian).
  - *Kết quả đầu ra:* Góc khớp này ngay lập tức được gửi ngược lại Web để hiển thị "Mô hình ảo" (Ghost Robot). Lúc này, robot vật lý vẫn đứng im tuyệt đối. Song song, tham số rủi ro kỳ dị được tính toán để cảnh báo nếu tọa độ ảo rơi vào khu vực nguy hiểm.

- **Bước 2 - Tư duy và Lập Kế hoạch (Motion Planning):** 
  - *Điều kiện kích hoạt:* Thuật toán liên tục tính toán đạo hàm vận tốc của tọa độ đầu vào. Khi người dùng buông chuột (vận tốc mục tiêu = 0 trong hơn 0.4 giây), bộ não CuRobo được đánh thức.
  - *Xử lý cốt lõi:* CuRobo thu thập biến `actual_q` (6 góc khớp thực tại của robot) và biến `target_q` (6 góc khớp của mô hình ảo). Sau đó, nó quét toàn bộ không gian đa chiều, rải hàng ngàn điểm để tìm ra lộ trình (Trajectory) hoàn hảo nhất nối từ `actual_q` đến `target_q` mà không đâm vào bất cứ vật cản nào.
  - *Kết quả đầu ra:* Một ma trận chứa hàng chục điểm `[q1...q6]` liền kề nhau tạo thành một quỹ đạo nội suy mượt mà.

- **Bước 3 - Hành động và Phản hồi (Execution):** 
  - *Truyền lệnh thực thi:* Quỹ đạo an toàn được lõi Python đóng gói thành chuỗi lệnh URScript bằng văn bản thuần (ASCII). Ví dụ: `def curobo_prog(): movej([q1, q2, q3, q4, q5, q6], a=1.2, v=0.25) end\n` (với `a` là gia tốc, `v` là vận tốc). Chuỗi lệnh này được đẩy trực tiếp vào Cổng mạng để bộ điều khiển tự động chạy.
  - *Giám sát phản hồi:* Trong quá trình robot chạy, hệ thống chuyển sang chế độ "lắng nghe". Nó liên tục đọc gói tin nhị phân từ Cổng RTDE ở tần số 125Hz để bóc tách mảng `actual_q` thực tế. Giao diện Web được cập nhật thanh tiến trình liên tục, và chỉ mở khóa cho phép thao tác tiếp theo khi sai số giữa `actual_q` và `target_q` tiệm cận mức 0.

## 4. Cấu Trúc Khối & Cơ Chế Bảo Vệ
Lõi xử lý trung tâm điều phối mạng nội bộ và tích hợp hệ thống bảo vệ 5 lớp để chống lỗi phần cứng:

- **Lớp 1 - Tiền cảnh báo:** Đánh giá mức độ kỳ dị thực thời để đưa ra cảnh báo trực quan.
- **Lớp 2 - Nội suy mật độ cao:** Ép bộ điều khiển bám sát quỹ đạo được rải điểm dày đặc nhằm loại bỏ sai số thuật toán nội suy mặc định của bộ điều khiển.
- **Lớp 3 - Cứu hộ chủ động:** Luồng giám sát độc lập liên tục phân tích dữ liệu mạng. Nếu rủi ro kỳ dị vượt ngưỡng, hệ thống sẽ thu hồi quyền điều khiển và tự động đưa robot về tọa độ mặc định (`HOME_Q`).
- **Lớp 4 - Ranh giới ảo:** Thiết lập các mặt phẳng và khối cầu trong không gian để chống va chạm khu vực cơ sở.
- **Lớp 5 - Chống tự kẹp:** Chặn các lệnh gập khớp vượt quá giới hạn thiết kế (ví dụ: góc quay > 166 độ).

### 4.1. Phân tích giao thức mạng
Hệ thống được thiết kế bám sát 100% các tiêu chuẩn truyền thông công nghiệp của Universal Robots. Điều này đảm bảo việc chuyển đổi từ mô phỏng (URSim) sang robot vật lý (e-Series) chỉ cần thay đổi địa chỉ IP mà không phải sửa đổi kiến trúc lõi:

- **Cổng 8080 (Giao diện Web/VR):** 
  - *Giao thức:* WebSocket (Truyền tải gói tin nhị phân nén MessagePack).
  - *Payload Nhận:* Mảng tọa độ không gian 6 bậc tự do `[X, Y, Z, Qw, Qx, Qy, Qz]` từ thiết bị ngoại vi với tần số tối thiểu 20Hz.
  - *Payload Trả về:* Mảng góc khớp mục tiêu (`target_q`) để vẽ mô hình ảo, thanh tiến trình di chuyển, và thông số mức độ rủi ro kỳ dị để hiển thị trực quan.
- **Cổng 30002 (URScript / Primary/Secondary Client):** 
  - *Giao thức:* TCP Socket giao tiếp bằng văn bản thuần (ASCII).
  - *Payload & Ứng dụng:* Đóng vai trò bộ nạp lệnh. Lõi Python đóng gói toàn bộ quỹ đạo an toàn (hàng chục điểm nối tiếp nhau) thành một kịch bản mã nguyên khối (ví dụ: `def my_prog(): movej([q1...q6], a=1.2, v=0.25) end\n`) và đẩy duy nhất một lần vào bộ nhớ RAM của bộ điều khiển. Việc nạp trọn gói giúp tránh lỗi tràn bộ đệm (buffer overflow) - một rủi ro cực kỳ nguy hiểm nếu gửi tọa độ rác liên tục xuống robot thực.
- **Cổng 30004 (RTDE - Real-Time Data Exchange):** 
  - *Giao thức:* Truyền tải nhị phân chuẩn mạng Big-Endian do UR phát triển, không suy hao gói tin.
  - *Payload & Ứng dụng:* Robot thực dòng e-Series phát sóng dữ liệu ở tần số **500Hz** (bản mô phỏng là 125Hz). Hệ thống đăng ký (subscribe) đọc trực tiếp các thanh ghi lõi: `actual_q` (6 góc khớp thực - Radian) và `actual_qd` (6 vận tốc khớp - Radian/s). Mảng dữ liệu này được moi ra để nuôi "Luồng cứu hộ chủ động", cho phép hệ thống luôn có dữ liệu theo thời gian thực để ra quyết định phanh gấp.
- **Cổng 29999 (Dashboard Server):** 
  - *Giao thức:* TCP Socket (ASCII).
  - *Payload & Ứng dụng:* Hoạt động như một "Bảng điều khiển ảo" (Virtual Teach Pendant). Cực kỳ quan trọng khi vận hành từ xa, cho phép truyền các lệnh hệ thống như `brake release\n` (nhả phanh tự động), `unlock protective stop\n` (mở khóa an toàn) hoặc `stop\n` (dừng khẩn cấp) thay vì phải bấm nút vật lý.

### 4.2. Trình tự Xử lý Bất đồng bộ
Vòng lặp sự kiện được tối ưu hóa để vận hành song song nhiều tiến trình:
- **Chu trình Lấy mẫu:** Lấy mẫu tọa độ (50ms/lần) $\rightarrow$ JAX giải động học (< 0.5ms) $\rightarrow$ Cập nhật giao diện Web.
- **Chu trình Phê duyệt:** Nhận diện trạng thái tĩnh (0.4s) $\rightarrow$ Khóa Web $\rightarrow$ GPU quét không gian $\rightarrow$ Lọc an toàn động học.
- **Chu trình Bám sát:** Gửi lệnh qua Cổng 30002 $\rightarrow$ Truy vấn Cổng 30004 (50ms/lần) $\rightarrow$ Mở khóa khi sai số tiệm cận 0.
- **Chu trình Can thiệp:** Phát hiện dị thường trong quá trình di chuyển $\rightarrow$ Ghi đè lệnh dừng khẩn cấp $\rightarrow$ Bẻ lái quỹ đạo về Home.

## 5. Hướng Dẫn Khởi Chạy
Yêu cầu khởi tạo môi trường song song (2 Terminal):

### Terminal 1: Khởi động hệ thống điều khiển URSim
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e -f "-p 30001-30004:30001-30004 -p 29999:29999 -p 6080:6080"
```
*(Lưu ý: Để xử lý lỗi cấp phát mạng, chạy lệnh `docker restart ursim`)*

### Terminal 2: Kích hoạt Lõi Điều Khiển
```bash
cd ~/curobo && source .venv/bin/activate
python3 ~/ur_ws/curobo_ursim/interactive_obstacle_test.py
```
*(Quá trình biên dịch JIT cho lần khởi chạy đầu tiên tốn khoảng 15 giây).*

### Giám sát & Điều khiển
Truy cập qua trình duyệt Web:
- **Môi trường Tương tác 3D:** [http://localhost:8080](http://localhost:8080)
- **Bảng điều khiển Polyscope:** [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html)
