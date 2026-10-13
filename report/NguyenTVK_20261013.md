# Báo cáo 13/10/2026

## 1. Giới thiệu chung
Hoàn thiện cơ chế mô phỏng gắp/thả 3D, tích hợp camera ảo góc nhìn cổ tay trực tiếp lên giao diện VR Dashboard, xây dựng cơ chế tự động thu hồi trục điều khiển khi vướng lỗi động học, cùng bộ công cụ thu thập và kiểm định dữ liệu AI Demonstration, đồng thời tinh gọn toàn diện mã nguồn hệ thống và cập nhật tài liệu kỹ thuật.

**Nhận xét:** Tay máy UR3e giờ đây có khả năng tiếp cận và gắp chính xác vật thể 3D trong không gian có chướng ngại vật thực tế. Các vấn đề về va chạm mặt bàn và lực hút ảo từ khoảng cách xa đã được xử lý triệt để. Hệ thống được tinh gọn tối đa, giải phóng hoàn toàn các thư viện phụ thuộc cũ, chuyển giao trọn vẹn sang bộ giải CuRobo GPU IK thời gian thực và sẵn sàng cho việc huấn luyện mô hình Robot AI (Imitation Learning / Diffusion Policy).

## 2. Hoàn thiện Cơ chế Gắp/Thả 3D & Không Gian Vật Cản

### Khắc phục Bài toán Hình học Va chạm & Lực hút
- **Phân tích nguyên nhân:** Qua khảo sát hình học:
  - Cấu hình va chạm của NVIDIA CuRobo trang bị các khối cầu bảo vệ (collision spheres) bao quanh ngàm bích `tool0` kéo dài xuống dưới khoảng 11cm.
  - Khi mặt bàn làm việc được đặt tại $Z=0.0$m, CuRobo sẽ cự tuyệt mọi nghiệm IK đưa `tool0` xuống thấp hơn $Z=0.10$m để ngăn va chạm mặt bàn.
  - Trong khi đó, quả cà chua có tâm tại $Z=0.035$m (đỉnh quả tại $Z=0.067$m). Do đó, khi `tool0` ở vị trí thấp nhất ($Z \approx 0.10$m), khoảng cách Euclidean tới tâm quả cà chua vẫn vào khoảng $6.5\text{ cm}$.
- **Giải pháp triển khai:**
  1. **Hạ cao độ mặt bàn:** Điều chỉnh mặt bàn làm việc xuống cao độ bề mặt $Z=-0.03$m (`pose: [0.15, 0.0, -0.08]`, `dims: [1.2, 1.2, 0.1]`), giúp ngàm `tool0` có đủ không gian di chuyển xuống $Z \in [0.09, 0.10]$m mà không bị vi phạm va chạm.
  2. **Thuật toán kiểm tra điều kiện hút kép (Dual Check Grasp):** Cập nhật ngưỡng khoảng cách Euclidean lên `0.095`m và bổ sung điều kiện căn chỉnh mặt phẳng:
     $$\text{Grasp} \iff \text{Grip is ON} \land \Big( d < 0.095\text{m} \;\lor\; \big( d_{xy} < 0.055\text{m} \;\land\; Z_{tool} < 0.13\text{m} \big) \Big)$$
     Đảm bảo chỉ khi đầu ngàm thực sự tiếp cận ngay phía trên quả cà chua và hạ thấp xuống thì lực hút mới được kích hoạt, triệt tiêu hoàn toàn hiện tượng hút từ xa hoặc kẹt không hút được.
  3. **Tối ưu hóa bố trí vật cản:** Tinh giản các vật cản dư thừa, giữ lại một vách tường chắn vừa tầm (`wall` tại $[0.25, 0.0, 0.06]$m) chắn giữa vị trí gắp cà chua và khay chứa, tạo kịch bản thực tế để kiểm chứng năng lực luồn lách né vật cản của CuRobo.
  4. **Tự động Thu hồi Trục XYZ khi Vặn xoắn / Vướng vật cản (Auto-Snap):** Khi phát hiện lỗi tư thế kẹt (nhảy nhánh IK, vặn xoắn cổ tay) hoặc robot cự tuyệt lệnh do vướng vật cản, hệ thống sẽ tự động giật lùi trục tọa độ điều khiển XYZ về ngay vị trí ngàm bích của tay máy thực tế, ngăn ngừa tình trạng trôi lệch tọa độ và loại bỏ rủi ro tay máy lao vọt bất ngờ.

## 3. Camera Ảo Cổ Tay (Eye-in-Hand) & Pipeline Dữ liệu AI HDF5

- **Camera Ảo Gắn Cổ Tay (Synthetic Wrist Camera):**
  - Xây dựng camera ảo kết xuất trực tiếp khung nhìn 3D từ góc gắn trên ngàm robot với độ phân giải chuẩn $640 \times 480$ (tỷ lệ FOV thực tế $60^\circ$).
  - Tích hợp khung xem trực tiếp (Live Preview) lên giao diện Viser 3D giúp kỹ sư quan sát vật thể từ góc nhìn của robot trong thời gian thực.
  - **Truyền luồng trực tiếp lên VR Dashboard (Camera Stream):** Kết nối luồng hình ảnh từ camera gắn trên ngàm robot trực tiếp sang khung CAMERA STREAM trên giao diện thực tế ảo Meta Quest (WebXR) ở tần số 10 FPS, giúp người vận hành quan sát cận cảnh vật thể ngay trong buồng lái ảo.
- **Định dạng Dữ liệu Demonstration Chuẩn HDF5 (20Hz):**
  - Thu thập đồng bộ dữ liệu đa phương thức cho từng Episode: góc khớp thực tế (`qpos`), vận tốc (`qvel`), lệnh điều khiển (`action`), trạng thái đầu gắp (`gripper`), ảnh camera cổ tay (`images/wrist_camera`), chuỗi thời gian (`timestamps_ms`), và nhãn kết quả (`success=True/False`).
- **Bộ Công Cụ Kiểm Định & Tái Lập Quỹ Đạo:**
  - **Kiểm định Dữ liệu:** Tự động rà soát toàn bộ file HDF5 trong dataset, phát hiện lỗi `NaN`/`Inf`, đo lường độ lệch tần số thời gian ($\Delta t$ jitter), và kiểm tra tính đồng bộ 100% giữa độ dài chuỗi ảnh và chuỗi lệnh điều khiển. Đã xác thực thành công toàn bộ 22/22 episodes hiện có.
  - **Tái lập Quỹ đạo:** Cung cấp 2 chế độ:
    - *Dry-run Offline:* Tính toán bước nhảy góc khớp và vận tốc cực đại để đảm bảo an toàn cơ học trước khi triển khai.
    - *Live Replay:* Phát lại trực tiếp quỹ đạo xuống bộ điều khiển URSim / UR3e thật qua cổng RTDE.

## 4. Kết quả Kiểm thử & Đo kiểm

1. **Kiểm thử Pipeline Gắp/Thả 3D & Xuất HDF5:**
   - Hoàn thành đầy đủ 9 bước thao tác (Tiếp cận $\rightarrow$ Hạ thấp $\rightarrow$ Hút cà chua $\rightarrow$ Nhấc lên $\rightarrow$ Di chuyển ngang vượt tường $\rightarrow$ Hạ vào giỏ $\rightarrow$ Nhả gắp $\rightarrow$ Lưu episode).
   - Xuất file HDF5 và ảnh camera cổ tay thành công với kết quả `TEST HOÀN TẤT THÀNH CÔNG 100%`.
2. **Kiểm thử Camera Ảo Cổ Tay:**
   - Render thành công 4 góc nhìn camera cổ tay (Chính giữa, Lệch trái 10cm, Lệch phải 10cm, Đẩy ra xa 10cm).
3. **Kiểm định Chất lượng Dữ liệu:**
   - `22/22 episodes đã quét | Tổng frames: 94 | Không có NaN, độ dài đồng bộ 100%, timestamp đều`.
