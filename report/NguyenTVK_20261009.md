# Báo cáo 09/10/2026

## 1. Giới thiệu chung
Báo cáo tập trung vào việc hoàn thiện, gỡ lỗi (debug) các góc khuất kỹ thuật của tính năng Viễn thao thực tế ảo (VR Teleoperation) và tinh chỉnh luồng trải nghiệm người dùng (UX) trên giao diện Web Dashboard, hướng tới độ ổn định công nghiệp.

## 2. Nâng cấp Cơ chế Auto-Snap (Tự động thu hồi)
- **Xử lý triệt để Lỗi Ngầm (Silent Bug) Toán học:** Phát hiện và vá lỗ hổng khi thư viện Viser trả về tọa độ ở định dạng tĩnh (`Tuple`). Định dạng này khi đưa qua bộ lọc hàm mũ (EMA Filter) đã gây ra ngoại lệ `TypeError: can't multiply sequence by float`, làm sập luồng tính toán ngầm khiến bộ đếm thời gian bị kẹt. Đã bọc ép kiểu `np.array()` ở cấp độ đầu vào để đảm bảo luồng tính toán thông suốt.
- **Tái cấu trúc Logic Đo lường:** Xóa bỏ phương pháp đo khoảng cách tuyến tính (khoảng cách > 15cm) kém hiệu quả do hiện tượng "bám đuổi ranh giới" của robot. 
- **Tham chiếu Ground-Truth từ CuRobo:** Cơ chế Auto-Snap nay được đấu nối trực tiếp vào Bộ giải mã động học (IK Solver). Bất cứ khi nào lõi CuRobo trả về trạng thái từ chối lệnh (`ngoai tam voi`, `dung vat can`, `tu the ket`...) liên tục trong 2.0 giây, hệ thống sẽ tự động giật trục tọa độ 3D về lại vị trí thực tế của tay máy.

## 3. Cải tiến Luồng Vận hành (Workflow)
- **Tối ưu Nút Về Home "Một Chạm":** Xóa bỏ các thông báo cản trở yêu cầu người dùng phải tắt Teleop thủ công. Nút bấm hiện tại tự động đảm nhận toàn bộ quy trình: Tạm ngắt luồng điều khiển thời gian thực (RTDE) $\rightarrow$ Quy hoạch và chạy quỹ đạo về Home $\rightarrow$ Đồng bộ lại Widget 3D $\rightarrow$ Tự động Bật lại Teleop. Người dùng có thể tiếp tục công việc ngay lập tức mà không phát sinh thao tác thừa.
- **Loại bỏ Homing Cưỡng bức lúc Khởi động:** Nhằm đảm bảo an toàn cho môi trường xưởng thực tế, hệ thống không còn ép robot tự động chạy về Home mỗi khi chạy script. Trục tọa độ 3D sẽ tự động sinh ra (spawn) khớp hoàn toàn với tư thế hiện tại của robot (kể cả khi đang cầm lửng lơ vật phẩm), đảm bảo tính liền mạch của dây chuyền.

## 4. Tinh chỉnh Giao diện và Trải nghiệm (UI/UX)
- **Giao diện Web Dashboard (Modern Industrial):** Thiết kế lại giao diện bảng điều khiển Viser theo ngôn ngữ công nghiệp hiện đại. Sử dụng font chữ `Inter`, bảng màu xám khói sang trọng kết hợp điểm nhấn xanh lá (Lime Green) cho các trạng thái hoạt động. Các nút bấm được bo góc mềm mại (pill-shaped), đồng thời loại bỏ toàn bộ các biểu tượng (icon/emoji) dư thừa để tạo cảm giác chuyên nghiệp, tập trung tuyệt đối vào luồng công việc.
- **Phản hồi Xúc giác Thực tế ảo (VR Haptics):** Tích hợp luồng phản hồi lực (Haptic Feedback) trực tiếp lên tay cầm Meta Quest. Bất cứ khi nào tay máy va chạm vật cản, rơi vào điểm kỳ dị (Singularity), hoặc mất khả năng nội suy (IK Thất bại), hệ thống sẽ lập tức truyền tín hiệu rung lên tay cầm VR. Tính năng này đóng vai trò như một "giác quan thứ 6", giúp người vận hành cảm nhận được giới hạn vật lý của robot ngay trong môi trường không gian ảo.
- **Tối ưu Bảng Trạng thái:** Gọt dũa lại toàn bộ các chuỗi văn bản báo cáo trạng thái (Status Strings) đẩy lên Web UI. Loại bỏ các tiền tố `"Teleop:"` lặp lại dư thừa, giúp bảng điều khiển hiển thị trạng thái ngắn gọn, trực diện và chuyên nghiệp hơn (Vd: *"Ngoài tầm với - Đứng im"* thay vì *"Teleop: Ngoài tầm với - Đứng im"*).
## 5. Nâng cấp Cơ chế Viễn thao (Teleoperation Movement)
- **Cơ chế Phân tách Chuyển động (Decoupled Controls):** Nhằm giải quyết triệt để vấn đề "trục tọa độ tự xoay khi người dùng chỉ muốn tiến/lùi", hệ thống đã phân tách quyền điều khiển thông qua hệ thống nút bấm của Meta Quest:
  - **Chỉ bóp Cò (Trigger):** Kích hoạt chế độ *Chỉ Tịnh tiến* (Translation Only). Hệ thống khóa chết trục xoay, giúp tay máy đi thẳng tắp mà không bị rung lắc góc độ.
  - **Chỉ bóp Ngón giữa (Grip):** Kích hoạt chế độ *Chỉ Xoay* (Rotation Only). Hệ thống neo chặt vị trí, người dùng xoay cổ tay để tinh chỉnh góc độ (Tool-Centric Rotation).
  - **Bóp cả 2 nút (Grip + Trigger):** Mở khóa chuyển động tự do 6D (Free Movement) như truyền thống.
- **Côn Thông Minh (Smart Clutch):** Phát triển thêm bộ nhớ đệm `_last_button_state` để theo dõi sự thay đổi trạng thái nút bấm. Bất cứ khi nào người dùng chuyển đổi chế độ (Vd: Đang Tịnh tiến $\rightarrow$ Chuyển sang Xoay), hệ thống sẽ *tự động thả neo lại* (Reset Anchor). Cơ chế này loại bỏ hoàn toàn các lỗi "nhảy cóc" tọa độ (teleportation) do sự thay đổi đột ngột cấu trúc toán học của ma trận Delta, giúp robot di chuyển cực kỳ êm ái khi đổi mode điều khiển.
