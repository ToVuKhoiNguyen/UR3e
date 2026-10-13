# UR3e Teleoperation via Meta Quest 3S & CuRobo GPU IK

Hệ thống điều khiển viễn thao thời gian thực (Real-time Teleoperation) cho cánh tay robot công nghiệp Universal Robots UR3e sử dụng kính thực tế ảo Meta Quest 3S (WebXR) và giao diện Web 3D Viser. Hệ thống giải động học nghịch (Inverse Kinematics) và kiểm tra va chạm hoàn toàn trên GPU bằng NVIDIA CuRobo, đồng thời hỗ trợ thu thập dữ liệu trình diễn (Demonstrations) chuẩn HDF5 phục vụ huấn luyện mô hình Robot AI (Imitation Learning, Diffusion Policy, ACT).

---

## 1. Tinh Nang Noi Bat

1. **Dieu khien Vien Thao Thoi Gian Thuc (20Hz RTDE servoJ):**
   - Dong bo chuyen dong 6DoF tu tay cam Meta Quest 3S sang robot UR3e voi do tre cuc thap.
   - Bo thuc thi `ServoJExecutor` su dung tham so toi uu cho chu ky 20Hz: `dt = 0.05`s, `lookahead_time = 0.04`s, `gain = 800`.
   - Che do dieu khien phan tach chuc nang:
     - **Chi bop Co (Trigger):** Tinh tien khong gian (World-Centric mapping, khoa goc xoay).
     - **Chi bop Grip:** Xoay dau hut/gap theo he toa do dau cong cu (Tool-Centric mapping, khoa tinh tien).
     - **Bop ca Trigger + Grip:** Chuyen dong tu do 6DoF dong thoi.
     - **Co che Smart Clutch:** Tu dong chot mo neo moi (Reset Anchor) ngay khi doi nut bam, triet tieu hoan toan loi nhay toa do (teleport).

2. **GPU Collision-Free Inverse Kinematics (NVIDIA CuRobo 0.8):**
   - Giai bai toan IK thoi gian thuc tren GPU CUDA (3–6ms), tu dong kiem tra va cham voi ban lam viec, tuong chan va bao ve cum camera/ngam kep.
   - Toi uu hoa vung an toan: `collision_sphere_buffer: 0.005` (5mm) va cac qua cau cum camera/tool0 ban kinh 2.2cm.
   - Co che chong dao nhanh IK (Branch Consistency Penalty): Bo sung he so phat lech nhanh (`branch_penalty = 50.0` khi lech > 0.8 rad) voi 64 hat giong (seeds) va 16 nghiem tra ve, loai bo triet de loi lat khop co tay (Loi van xoan).

3. **Luồng Stream Camera Cổ Tay Trực Tiếp Trong VR Dashboard:**
   - Khung "CAMERA STREAM" tren giao dien WebXR Cockpit hien thi goc nhin truc tiep tu camera ao gan o co tay robot (`SyntheticWristCamera`, do phan giai 640x480, FOV 65 do).
   - Truy truyen khung hinh qua WebSocket (`wrist_cam`) va endpoint HTTP `/wrist_camera.jpg`.

4. **Mo Phong Gap/Tha 3D (Interactive Grasp Simulator):**
   - Mo phong co che gap/hut vat the qua ca chua 3D khi ngam tiep can dung vi tri (khoang cach 3D < 9.5cm hoac lech XY < 5.5cm kem do cao Z < 13cm).
   - Tu dong phat hien khi dua vat the vao khay chua va ghi nhan su kien hoan thanh nhiem vu.
   - Nut bam Gripper doc lap (nut A/X tren tay cam hoac checkbox tren Web UI) khong bi xung dot voi nut Grip xoay co tay.

5. **Co Che Tu Dong Thu Hoi XYZ (Auto-Snap on Stuck):**
   - Khi robot gap tinh trang ket tu the (van xoan khop) hoac vuong vat can keo dai qua 1.2 giay (`snap_threshold = 1.2`), he thong tu dong phat tin hieu rung Haptic toi tay cam va giat toa do Widget 3D ve dung dau tay may robot that (`fk_res`), giup nguoi van hanh tiep tuc dieu khien tro lai ngay lap tuc.

6. **Thu Thap & Quan Ly Du Lieu AI Chuan HDF5:**
   - Ghi lai chuoi thao tac day du: goc khop (`qpos`), van toc (`qvel`), lenh dieu khien (`action`), trang thai ngam gap (`gripper`), anh camera co tay (`images/wrist_camera`), timestamp o tan so 20Hz.
   - Tu dong danh dau ket qua thanh cong/that bai (`success=True/False`).

7. **Bo Cong Cu Kiem Dinh & Tai Lap Quy Dao (Data Sanity & Replay):**
   - `verify_dataset.py`: Quet toan bo file HDF5 kiem tra gia tri NaN/Inf, bien thien chu ky lay mau (dt jitter), kich thuoc tensor hinh anh.
   - `replay_episode.py`: Tai hien lai quy dao trinh dien o che do Offline Dry-Run hoac Live Replay ban lenh truc tiep sang robot.

8. **Cac Lop Bao Ve An Toan (Safety Gate & Watchdog):**
   - **Chong lat khop co tay (Anti-Twist Guard):** Chan buoc nhay goc khop dot ngot (`max_jump > 1.5` rad).
   - **Gioi han toc do (Rate Limiter):** Kep van toc goc khop toi da $\le 2.0$ rad/s de bao ve hop so robot vat ly.
   - **Giam sat mat ket noi (Timeout Watchdog):** Tu dong ngat ServoJ va giat lui Gizmo ve robot neu mat tin hieu VR qua 2 giay.

---

## 2. Danh Sach Thu Vien & Moi Truong Cai Dat

### Yeu cau He thong
- **He dieu hanh:** Ubuntu 20.04 hoac 22.04 LTS (x86_64).
- **Phan cung:** NVIDIA GPU ho tro CUDA 12.0+ (RTX 3060 tro len).
- **Gia lap URSim:** UR3e chay tren Docker hoac ROS 2 driver (Port 30004 cho RTDE Data, Port 30002 cho script/dashboard).

### Bang Thu Vien Python

| Thu vien | Phien ban | Vai tro trong he thong |
| :--- | :--- | :--- |
| `torch`, `torchvision` | 2.1.0+ (CUDA) | Tinh toan ma tran Tensor GPU cho CuRobo va Jacobian singularity metric. |
| `curobo` | 0.8.0+ | Bo giai IK va quy hoach tranh va cham tren GPU NVIDIA. |
| `viser` | 0.2.0+ | May chu WebGL 3D Visualization, Gizmo tuong tac va Web UI. |
| `ur_rtde` | 1.5.0+ | Giao tiep thoi gian thuc RTDE voi bo dieu khien UR qua port 30004. |
| `opencv-python` | 4.8.0+ | Render va ma hoa JPEG luong camera ao co tay (640x480). |
| `transforms3d` | 0.4.0+ | Chuyen doi giua Axis-Angle, Quaternion va Rotation Matrix. |
| `trimesh` | 4.0.0+ | Load va phan tich mo hinh luoi 3D robot va canh vat can. |
| `h5py` | 3.9.0+ | Ghi va doc tap du lieu AI Demonstration chuan HDF5. |
| `pyyaml` | 6.0+ | Doc file cau hinh robot (`ur3e_custom.yml`) va vat can (`obstacle_scene.yml`). |
| `websockets` | 12.0+ | May chu truyen nhan du lieu 6DoF tu tay cam WebXR voi do tre thap. |
| `cryptography` | 41.0+ | Tu dong sinh chung chi SSL/TLS cho may chu HTTPS WebXR. |
| `numpy`, `scipy` | 1.24+ | Tinh toan dai so tuyen tinh, phep quay SE(3) va bo loc EMA. |

### Lenh Cai Dat

```bash
# Tao va kich hoat moi truong ao
python3 -m venv ~/curobo/.venv
source ~/curobo/.venv/bin/activate

# Cai dat PyTorch voi CUDA
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# Cai dat cac thu vien phu tro
pip install viser ur_rtde opencv-python transforms3d trimesh h5py pyyaml websockets cryptography numpy scipy

# Cai dat NVIDIA CuRobo
pip install curobo --extra-index-url https://pypi.nvidia.com
```

---

## 3. Cau Truc Thu Muc Du An

```text
ur_ws/curobo_ursim/
├── main.py                          # Diem khoi chay chinh cua he thong
├── README.md                        # Tai lieu huong dan he thong
├── assets/
│   ├── ur3e_custom.urdf             # URDF UR3e mo ta link, khop va tool0
│   ├── ur3e_custom.yml              # Cau hinh CuRobo kinematics, collision spheres va bounds
│   └── obstacle_scene.yml           # Cau hinh mat ban va vach ngan vat can
├── config/
│   └── singularity.yaml             # Nguong an toan Jacobian singularity (sigma_min)
├── core/
│   ├── curobo_ursim_control.py      # Bo giai CuRoboIK (GPU batched seeds, branch penalty)
│   ├── data_logger.py               # Module ghi Demonstration HDF5 o tan so 20Hz
│   ├── verify_dataset.py            # Cong cu kiem tra chat luong tap du lieu HDF5
│   └── replay_episode.py            # Cong cu tai hien quy dao thao tac
├── safety/
│   ├── __init__.py                  # Export SingularityGuard
│   └── singularity_guard/
│       ├── guard_core.py            # Giam sat diem ky di thoi gian thuc
│       ├── jacobian_metrics.py      # Tinh toan sigma_min tu ma tran Jacobian
│       └── calibration.py           # Can chinh nguong ky di theo khong gian lam viec
├── teleop/
│   └── teleoperation.py             # TeleoperationController, ServoJExecutor, SafetyGate
├── teleop_quest/
│   ├── main_quest.py                # Vong lap chinh 20Hz tich hop VR, Viser, Camera va GraspSim
│   ├── viser_ui.py                  # Giao dien Viser 3D Cockpit va camera preview
│   ├── grasp_simulator.py           # Mo phong gap/tha qua ca chua 3D vao gio
│   ├── synthetic_camera.py          # Render camera ao co tay 640x480 (Eye-in-Hand)
│   ├── quest_server.py              # Server HTTPS (8443) va WSS (8444) cho WebXR Quest
│   ├── network_utils.py             # Tim IP LAN va tu dong sinh chung chi SSL
│   ├── pose_sources.py              # WidgetPoseSource (doc toa do Viser Widget kem bo loc EMA)
│   ├── delta_pose_source.py         # Chuyen doi vi phan Quest sang UR3e base_link
│   ├── timeout_watchdog.py          # E-Stop va tu dong thu hoi khi mat tin hieu
│   ├── home_planner.py              # Dua robot ve vi tri Home an toan
│   └── static/
│       ├── index.html               # Giao dien WebXR Cockpit tren kinh Quest
│       ├── main.js                  # Xu ly gamepad WebXR, stream camera ao va haptic
│       └── style.css                # Style hien thi phong cach HUD
└── tests/
    ├── test_camera.py               # Kiem thu render camera ao o cac goc nhin & giao dien GUI
    ├── test_simulated_pick_and_place.py # Kiem thu tu dong pick-place ca chua va xuat HDF5
    └── test_singularity_guard.py    # Kiem thu tinh toan Jacobian singularity metric (sigma_min)
```

---

## 4. Huong Dan Van Hanh

### Buoc 1: Khoi dong URSim hoac Robot that
Neu su dung gia lap URSim:
```bash
ros2 run ur_client_library start_ursim.sh -m ur3e
```
*Ghi chu:* Mac dinh he thong ket noi toi `127.0.0.1`. Neu ket noi robot UR3e that qua mang LAN, cap nhat bien `ROBOT_IP` trong code tuong ung.

### Buoc 2: Chay chuong trinh chinh
```bash
cd /home/nguyen/ur_ws/curobo_ursim
source ~/curobo/.venv/bin/activate
python3 main.py
```
He thong se hien thi thong tin ket noi:
- Viser Dashboard: `http://localhost:8080`
- WebXR Quest: `https://<IP_LAN>:8443` (Data WebSocket tai port `8444`)

### Buoc 3: Dieu khien bang Viser Web UI (May tinh)
- Mo trinh duyet tren may tinh tai `http://localhost:8080`.
- Tich chon **Bat Dieu Khien (Teleop)** de robot bam theo Gizmo 3D.
- Tich chon **Gripper (Bop Kep)** de bat luc hut gap vat the.
- Tich chon **Record AI Episode** de bat dau ghi du lieu sang HDF5.
- Bam **Go to Home** khi can dua robot ve vi tri goc an toan.

### Buoc 4: Dieu khien bang Kinh Meta Quest 3S (WebXR)
1. Dam bao kinh Meta Quest 3S va may tinh ket noi cung mang Wi-Fi LAN.
2. Tren trinh duyet Meta Quest Browser:
   - Truy cap `https://<IP_LAN>:8444` -> Chon **Advanced** -> **Proceed to unsafe** (chap nhan chung chi SSL tu sinh cho kenh WebSocket).
   - Truy cap `https://<IP_LAN>:8443` -> Chon **Advanced** -> **Proceed to unsafe** (chap nhan chung chi SSL cho kenh WebXR).
3. Bam **START IMMERSIVE VR** tren man hinh chao mung.
4. **Cac thao tac tay cam:**
   - **Bop Co (Trigger):** Tinh tien dau robot trong khong gian (trai/phai, tien/lui, len/xuong).
   - **Bop Grip (Nut hong):** Xoay co tay robot theo huong tay cam.
   - **Nut A / X tren tay cam:** Bat/tat ngam kep Gripper de gap hoac tha qua ca chua.
   - **Khung Camera Stream:** Hien thi goc nhin truc tiep tu dau tay may robot theo thoi gian thuc.

---

## 5. Kiem Thu He Thong & Quan Ly Du Lieu

### 1. Kiem thu tu dong Gap Ca Chua & Xuat HDF5
Chay truc tiep file test trong thu muc `tests/` hoac qua shortcut goc:
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_simulated_pick_and_place.py
```
Ket qua kiem tra:
- Tap tin Demonstration HDF5 duoc tao tai: `logs/ai_training_data/test_grasp_sim/episode_X.hdf5`.
- Anh chup camera ao co tay tai thoi diem hoan thanh: `logs/simulated_wrist_camera_view.jpg`.

### 2. Kiem thu Camera ao Co tay (Synthetic Wrist Camera)
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_camera.py
```
Kiem tra render 4 goc nhin (Chinh giua, Lech trai, Lech phai, Day ra xa). Cac file anh ket xuat se duoc luu vao thu muc `logs/`.

Neu muon kiem tra truc quan bang thanh truot tren Viser:
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_camera.py --gui
```
Sau do mo `http://localhost:8089` de keo toa do qua ca chua truc tiep.

### 3. Kiem thu Lop Bao Ve Diem Ky Di (Singularity Guard)
```bash
/home/nguyen/curobo/.venv/bin/python tests/test_singularity_guard.py
```
Kiem tra tinh toan ma tran Jacobian metric ($\sigma_{min}$) tren GPU o ca tu the an toan lan tu the duoi thang co tay ($q_5 \approx 0$).

### 4. Kiem dinh chat luong Tap Du Lieu (Sanity Check)
```bash
/home/nguyen/curobo/.venv/bin/python core/verify_dataset.py
```
Kiem tra toan ven cac episode: xac minh khong co gia tri `NaN`/`Inf`, kiem tra do lech tan so lay mau `dt`, kiem tra so frame va do phan giai anh camera.

### 5. Tai lap Quy dao Demonstration (Replay)
- **Offline Dry-Run (kiem tra quy dao tren Viser khong ban lenh sang robot):**
  ```bash
  /home/nguyen/curobo/.venv/bin/python core/replay_episode.py --file logs/ai_training_data/test_grasp_sim/episode_0.hdf5
  ```
- **Live Replay (ban lenh dieu khien truc tiep toi URSim / Robot that):**
  ```bash
  /home/nguyen/curobo/.venv/bin/python core/replay_episode.py --file logs/ai_training_data/test_grasp_sim/episode_0.hdf5 --execute --ip 127.0.0.1
  ```

---

## 6. Thong So Ky Thuat Dieu Khien Chinh

| Tham so | Gia tri | Giai thich ky thuat |
| :--- | :--- | :--- |
| **Tan so vong lap chinh** | 20 Hz (`dt = 0.05`s) | Dong bo giua WebXR, Viser, CuRobo va RTDE servoJ. |
| **ServoJ Lookahead Time** | `0.04` s | Noi suy chuyen dong muot ma, loai bo do tre va do ro giat. |
| **ServoJ Gain** | `800` | Do cung bam quy dao cao, giup dau robot bam sat vi tri tay nguoi. |
| **Van toc khop toi da (Rate Limiter)** | `2.0` rad/s | Kep gia toc va van toc khop de chong loi Protective Stop tren UR. |
| **Nguong phat hien van xoan khop** | `1.5` rad | Chan nhung buoc nhay dot ngot do dao nghiem IK gan diem ky di. |
| **So hat giong CuRobo IK** | 64 seeds | Nang cao xac suat tim nghiem lien tuc tren GPU. |
| **He so phat lech nhanh IK** | `50.0` (khi lech > 0.8 rad) | Giu robot luon o nhanh dong hoc dong nhat voi vi tri hien tai. |
| **Deadband tinh tay cam VR** | 5 mm (`0.005` m) | Loai bo rung tay sinh ly voi co che tru mem (Soft Deadband). |
| **Do lech tay cam toi da (max_delta_m)** | 0.25 m | Pham vi keo tay moi nhip bop co, tao cam giac thoai mai. |
| **Bo loc lam muot EMA (Alpha)** | 0.35 | Can bang giua do nhay phan hoi va chong nhieu jitter. |
| **Nguong thoi gian Auto-Snap** | 1.2 s | Tu dong keo XYZ ve dau tay may khi bi ket tu the hoac vuong vat can. |
