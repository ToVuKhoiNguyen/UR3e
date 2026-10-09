# Báo cáo 09/10/2026

## 1. Giới thiệu & Kết quả Đạt Được
Báo cáo tập trung vào việc tái cấu trúc cốt lõi thuật toán điều khiển (Teleoperation) và tối ưu hóa trải nghiệm người dùng (UX) nhằm đưa hệ thống chạm ngưỡng thực chiến công nghiệp.

**Nhận xét:** Việc chuyển đổi từ hệ quy chiếu cục bộ (Tool-Centric) sang hệ quy chiếu không gian thực (World-Centric) đã khắc phục triệt để lỗi nhận diện nhầm trục. Tay máy robot giờ đây bám sát hoàn hảo và chính xác tuyệt đối các hướng tiến, lùi, trái, phải, lên, xuống theo phương vật lý của tay cầm VR Meta Quest.

**Video Demo:** [demo_20261009.mp4](https://fptsoftware362-my.sharepoint.com/:v:/g/personal/nguyentvk_fpt_com/IQCJtDKHNXk3TaTxv6oy9qs6AQzb-bg3bBGWj0f2sRgJT7I?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=WWTqpV)

## 2. Tái cấu trúc Cơ chế Dịch chuyển (Teleoperation Movement)
- **Tịnh tiến World-Centric:** Đập bỏ cơ chế tính toán tịnh tiến dựa trên trục xoay của tay cầm. Việc áp dụng ánh xạ 1:1 theo hệ quy chiếu toàn cục (World Space) giúp người điều khiển không còn bị bối rối khi tay cầm nghiêng, chuyển động của tay ngoài đời thực được chuyển hóa chính xác 100% lên trục tọa độ của robot.
- **Phân tách Điều khiển (Decoupled Controls):** Khắc phục lỗi "tự xoay khi đang đẩy tới" bằng cách chia tách phím vật lý: Bóp cò (Trigger) khóa xoay để kẻ đường thẳng tắp; Bóp hông (Grip) khóa tịnh tiến để vặn cổ tay lấy góc. Chuyển động 6D tự do chỉ kích hoạt khi bóp cả hai nút.
- **Côn Thông Minh (Smart Clutch) & Bộ lọc Tín hiệu:** Áp dụng bộ nhớ đệm để theo dõi phím bấm. Bất cứ khi nào chuyển chế độ (tịnh tiến sang xoay), thuật toán tự động tái lập mỏ neo tọa độ (Reset Anchor), giúp triệt tiêu hoàn toàn sự cố giật cục (teleportation). Đồng thời, bộ lọc EMA cũng được tinh chỉnh tăng độ nhạy, cắt giảm tối đa độ trễ bám đuổi (latency).

## 3. Trải nghiệm Thực tế ảo & Giao diện (VR Haptics & UI/UX)
- **Phản hồi Xúc giác (VR Haptics) & Auto-Snap:** Tích hợp bộ rung tay cầm làm "giác quan thứ 6" để cảnh báo va chạm vật lý hoặc kẹt kỳ dị. Đấu nối trực tiếp cơ chế Auto-Snap vào lõi CuRobo IK: nếu robot từ chối lệnh liên tục quá 2 giây, ngàm 3D ảo sẽ tự động thu hồi về đúng tọa độ thực tế của tay máy (đã vá lỗi định dạng Tuple khiến luồng đếm giờ bị kẹt ngầm).
- **Quy trình Vận hành Khép kín:** Hủy bỏ lệnh ép chạy về Home lúc khởi động để bảo toàn trạng thái hiện tại của tay máy trên dây chuyền. Nút "Reset to Home" được thiết kế lại thành luồng một chạm: tự ngắt Teleop $\rightarrow$ chạy về đích $\rightarrow$ tự bật lại Teleop, đảm bảo tính liền mạch trong công việc.
- **Giao diện Modern Industrial:** Tối giản hóa Viser UI theo chuẩn công nghiệp (font Inter, màu xám khói, nút bo góc). Gọt dũa lại toàn bộ thông báo trạng thái, loại bỏ tiền tố `"Teleop:"` dư thừa để tập trung tối đa vào thông tin cốt lõi.
