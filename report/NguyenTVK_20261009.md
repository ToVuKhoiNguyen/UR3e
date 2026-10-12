# Báo cáo 09/10/2026
## 1. Giới thiệu chung
Báo cáo tập trung vào việc tái cấu trúc cốt lõi thuật toán điều khiển (Teleoperation) và tối ưu hóa trải nghiệm giao diện người dùng.

**Nhận xét:** Tay máy robot giờ đây bám sát chính xác các hướng tiến, lùi, trái, phải, lên, xuống theo phương vật lý của tay cầm VR Meta Quest. Giao diện điều khiển (Dashboard) trên môi trường AR đã trở nên tinh gọn và trực quan.

**Video Demo:** [Video Thực nghiệm](https://fptsoftware362-my.sharepoint.com/:v:/g/personal/nguyentvk_fpt_com/IQCJtDKHNXk3TaTxv6oy9qs6AQzb-bg3bBGWj0f2sRgJT7I?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=WWTqpV)

## 2. Tái cấu trúc Cơ chế Dịch chuyển (Teleoperation Movement)
- **Tịnh tiến World-Centric 1:1:** Ánh xạ 1:1 theo hệ quy chiếu toàn cục (World Space) giúp người điều khiển không còn bị bối rối khi tay cầm nghiêng, chuyển động của tay ngoài đời thực được chuyển hóa chính xác 100% lên trục tọa độ của robot.
- **Phân tách Điều khiển:** Khắc phục lỗi "tự xoay khi đang đẩy tới" bằng cách chia tách phím vật lý: Bóp cò (Trigger) khóa xoay để kẻ đường thẳng tắp; Bóp hông (Grip) khóa tịnh tiến để vặn cổ tay lấy góc. Chuyển động 6D tự do chỉ kích hoạt khi bóp cả hai nút.
- **Côn Thông Minh & Bộ lọc Tín hiệu:** Bất cứ khi nào chuyển chế độ (tịnh tiến sang xoay), thuật toán tự động tái lập mỏ neo tọa độ (Reset Anchor), triệt tiêu hoàn toàn sự cố giật cục. Bộ lọc EMA được tinh chỉnh để cắt giảm độ trễ bám đuổi (latency).

## 3. Nâng cấp Không gian Thực tế Ảo
- **Chế độ Lái Xuyên Thấu:** Sau khi bấm "Lock UI", tia laser từ tay cầm có thể bắn xuyên qua bảng điều khiển. Bảng vẫn hiển thị thông số Realtime (X, Y, Z, lỗi IK) nhưng bỏ qua các click nhầm, cho phép kỹ sư thoải mái vung tay bóp cò lái robot.
- **Phản hồi Xúc giác & Auto-Snap:** Tích hợp bộ rung tay cầm để cảnh báo va chạm vật lý hoặc kẹt kỳ dị. Đấu nối trực tiếp vào lõi CuRobo IK: nếu robot từ chối lệnh liên tục quá 2 giây, ngàm 3D ảo sẽ tự động giật lùi thu hồi về đúng tọa độ thực tế của tay máy.

## 4. Giao diện và Trải nghiệm Người dùng
- **Anh hóa và Tối giản hóa:** Dịch toàn bộ giao diện sang Tiếng Anh chuyên nghiệp và làm lại giao diện cho thân thiện, hiện đại, khoa học hơn.
- **Quy trình Vận hành Khép kín:** Nút "Go to Home" được thiết kế lại thành luồng một chạm: tự ngắt Teleop $\rightarrow$ chạy về đích an toàn $\rightarrow$ tự bật lại Teleop, đảm bảo tính liền mạch cho kỹ sư.

## 5. Kiến trúc Hệ thống & Thuật toán Cốt lõi
- **Tích hợp Camera vào Collision Checking:** Cập nhật file vật lý cấu hình robot của NVIDIA (`ur3e_custom.yml`). Bổ sung một ma trận lưới gồm các khối cầu (spheres) vào ngàm bích `tool0` để giả lập chính xác thể tích vật lý của một Camera gắn ngoài (tương đương RealSense D435). GPU của CuRobo giờ đây sẽ tự động tạo Force Field vô hình bao bọc lấy Camera, cự tuyệt mọi lệnh lái VR nếu camera có nguy cơ đập vào bàn hoặc vướng chướng ngại vật.
- **Tái cấu trúc Mã nguồn:** Giải phóng file `main_quest.py` bằng cách tách lớp (Decoupling):
  - `network_utils.py`: Chuyên trách tự động sinh SSL (HTTPS) và truy vết Local IP.
  - `pose_sources.py`: Chứa các thuật toán xử lý Tọa độ (Slerp Quaternion, lọc nhiễu EMA, neo Anchor tọa độ).
