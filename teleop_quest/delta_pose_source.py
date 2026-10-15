import os
import time
import numpy as np
import transforms3d
from scipy.spatial.transform import Rotation as R

class DeltaPoseSource:
    """
    Implements PoseSource contract.
    Chuyển delta tọa độ từ kính Quest sang base_link của UR3e.
    """
    def __init__(self, server, ctrl, scale: float = 0.75, max_delta_m: float = 0.22, anchor_reference_func=None, alpha: float = 0.28):
        self.server = server
        self.ctrl = ctrl
        self.scale = float(os.environ.get("TELEOP_SCALE", str(scale)))
        self.max_delta_m = float(os.environ.get("MAX_DELTA_M", str(max_delta_m)))
        self.anchor_reference_func = anchor_reference_func
        self.alpha = float(os.environ.get("TELEOP_ALPHA", str(alpha)))
        
        self._last_diag_msg = ""
        self._last_diag_time = 0

        self._anchor_pos_quest = None
        self._anchor_quat_quest_xyzw = None
        self._anchor_tcp = None
        self._anchor_quat_robot_xyzw = None
        self._anchor_theta_arm = None
        
        self._ema_pos = None
        self._ema_quat_xyzw = None
        
        self.enable_axis_snapping = os.environ.get("AXIS_SNAPPING", "1") == "1"
        self.nav_mode = os.environ.get("NAV_MODE", "arm_plane").lower()
        self.rotation_mode = os.environ.get("ROTATION_MODE", "decoupled_3axis").lower()
        self.rot_scale = float(os.environ.get("ROT_SCALE", "1.1"))

        # Cấu hình chống nhiễu chuyên sâu (Anti-Noise & Jitter Suppression)
        self.rot_deadband = float(os.environ.get("ROT_DEADBAND", "0.045"))  # ~2.6 độ (triệt tiêu rung tay khi bóp giữ phím)
        self.pos_deadband = float(os.environ.get("POS_DEADBAND", "0.006"))  # 6mm (triệt tiêu vi rung khi giữ yên tay)
        self.rot_snap_low = float(os.environ.get("ROT_SNAP_LOW", "0.55"))   # < 55%: khóa cứng rãnh đơn trục (Phương án A)
        self.rot_snap_high = float(os.environ.get("ROT_SNAP_HIGH", "0.75")) # > 75%: mở xoay 3D đa trục khi chủ ý bẻ chéo lớn
        self._last_dominant_rot_axis = None

        self._last_button_state = 0
        self.out_of_reach_time = 0.0

    def _apply_soft_axis_snapping(self, delta_robot):
        """
        Khóa trục thông minh (Soft Axis Snapping) đa tầng chống nhiễu:
        - Triệt tiêu rung lắc / lệch tay sinh trắc học khi người dùng chủ đích di chuyển theo một trục chính.
        - Khóa trục phụ < 28% về 0.0, chuyển tiếp mượt 28% - 50% bằng hàm Hermite smoothstep.
        - Nén bình phương rung lắc trục Z khi lướt phẳng XY, nén mặt phẳng XY khi nâng hạ thẳng đứng.
        """
        dx, dy, dz = delta_robot[0], delta_robot[1], delta_robot[2]
        
        # 1. Khóa trục mặt phẳng ngang (X - Tiến/Lùi vs Y - Trái/Phải):
        abs_x = abs(dx)
        abs_y = abs(dy)
        
        if abs_x > 0.006 or abs_y > 0.006:
            if abs_x >= abs_y:
                ratio = abs_y / (abs_x + 1e-6)
                if ratio < 0.28:
                    scale_y = 0.0
                elif ratio < 0.50:
                    t = (ratio - 0.28) / (0.50 - 0.28)
                    scale_y = t * t * (3.0 - 2.0 * t)
                else:
                    scale_y = 1.0
                dy = dy * scale_y
            else:
                ratio = abs_x / (abs_y + 1e-6)
                if ratio < 0.28:
                    scale_x = 0.0
                elif ratio < 0.50:
                    t = (ratio - 0.28) / (0.50 - 0.28)
                    scale_x = t * t * (3.0 - 2.0 * t)
                else:
                    scale_x = 1.0
                dx = dx * scale_x

        # 2. Tách biệt mặt phẳng ngang (X, Y) và trục đứng (Z):
        d_xy = np.sqrt(dx * dx + dy * dy)
        abs_z = abs(dz)
        
        if d_xy > 0.020 and abs_z < 0.012:
            # Người dùng đang trượt phẳng trên mặt bàn: Nén rung lắc trục Z triệt để (bình phương tỷ lệ)
            ratio_z = abs_z / 0.012
            dz = dz * (ratio_z * ratio_z)
        elif abs_z > 0.020 and d_xy < 0.012:
            # Người dùng đang nhấc/hạ thẳng đứng: Nén rung lắc mặt phẳng XY triệt để
            ratio_xy = d_xy / 0.012
            factor = ratio_xy * ratio_xy
            dx = dx * factor
            dy = dy * factor

        return np.array([dx, dy, dz])

    def _apply_soft_rotation_snapping(self, rotvec_tcp, rot_deadband=None):
        """
        Khóa trục xoay thông minh (Soft Rotation Snapping) & Bộ lọc triệt tiêu rò rỉ chéo đa tầng:
        - Soft deadband (mặc định 0.045 rad ~ 2.6 độ): Triệt tiêu hoàn toàn vi rung sinh học khi bóp giữ phím Grip.
        - Khóa trục độc lập (< 30%): Khi người dùng chủ ý xoay 1 trục (Pitch, Yaw hoặc Roll),
          các trục phụ nhỏ hơn 30% trục chính bị triệt tiêu hoàn toàn về 0.0.
        - Chống đảo trục chập chờn (Hysteresis Latch): Ưu tiên giữ trục chính đã chọn nếu trục mới chưa vượt trội > 15%.
        - Hòa trộn 3D tự nhiên (30% - 50%): Chuyển tiếp mượt mà bằng đường cong Hermite;
          khi > 50%, mở khóa hoàn toàn cho phép xoay 3D đa trục tự do.
        """
        if rot_deadband is None:
            rot_deadband = self.rot_deadband

        mag = np.linalg.norm(rotvec_tcp)
        if mag < rot_deadband:
            self._last_dominant_rot_axis = None
            return np.zeros(3)

        # Trừ hao deadband để chuyển động bắt đầu từ 0 liên tục, không giật nảy
        scaled_rotvec = rotvec_tcp * ((mag - rot_deadband) / mag)
        abs_v = np.abs(scaled_rotvec)
        curr_max_idx = int(np.argmax(abs_v))
        curr_max_val = abs_v[curr_max_idx]

        if curr_max_val < 1e-4:
            self._last_dominant_rot_axis = None
            return np.zeros(3)

        # Hysteresis Latch: Giữ ổn định trục chính, tránh hiện tượng rung giật lật trục khi 2 trục gần bằng nhau
        dominant_idx = curr_max_idx
        if self._last_dominant_rot_axis is not None and self._last_dominant_rot_axis != curr_max_idx:
            prev_val = abs_v[self._last_dominant_rot_axis]
            if prev_val > 0.85 * curr_max_val:
                dominant_idx = self._last_dominant_rot_axis
                curr_max_val = prev_val
        self._last_dominant_rot_axis = dominant_idx

        snapped = np.copy(scaled_rotvec)
        for i in range(3):
            if i == dominant_idx:
                continue
            ratio = abs_v[i] / curr_max_val
            if ratio < self.rot_snap_low:
                snapped[i] = 0.0
            elif ratio < self.rot_snap_high:
                t = (ratio - self.rot_snap_low) / (self.rot_snap_high - self.rot_snap_low)
                scale = t * t * (3.0 - 2.0 * t) # Hermite smoothstep
                snapped[i] = scaled_rotvec[i] * scale
            else:
                snapped[i] = scaled_rotvec[i]

        return snapped

    def get_pose(self):
        # Không sinh delta pose khi đang ở Edit Mode để tránh di chuyển ngoài ý muốn
        if hasattr(self.server, "get_drive_mode") and not self.server.get_drive_mode():
            self._last_button_state = 0
            return None

        frame = self.server.get_frame()
        if not frame:
            return None

        # Tách biệt điều khiển Tịnh tiến (Trigger) và Xoay (Grip)
        is_translating = frame.get("trigger", False)
        is_rotating = frame.get("grip", False)
        
        current_button_state = 0
        if is_translating: current_button_state += 1
        if is_rotating: current_button_state += 2
        
        if current_button_state == 0:
            self._last_button_state = 0
            return None

        # TÍNH NĂNG CLUTCH THÔNG MINH:
        # Bất cứ khi nào sếp thay đổi nút bấm (Vd: Đang Tịnh tiến -> Chuyển sang Xoay),
        # Hệ thống sẽ TỰ ĐỘNG CHỐT MỎ NEO MỚI (Reset Anchor). 
        # Điều này loại bỏ hoàn toàn lỗi "nhảy cóc (teleport) tọa độ" do bị ép biến delta về 0.
        if current_button_state != self._last_button_state:
            self._anchor_pos_quest = np.array(frame["pos"])
            
            q_q = frame["quat"] # [w, x, y, z]
            self._anchor_quat_quest_xyzw = np.array([q_q[1], q_q[2], q_q[3], q_q[0]])

            
            # Neo vào vị trí thực tế của robot (hoặc Widget 3D trên màn hình)
            try:
                if self.anchor_reference_func is not None:
                    # Lấy tọa độ từ Widget 3D (x, y, z, qw, qx, qy, qz)
                    ref_pose = self.anchor_reference_func()
                    self._anchor_tcp = np.array(ref_pose)
                else:
                    self._anchor_tcp = np.array(self.ctrl.tcp())
            except Exception:
                self._last_trigger = False
                return None
            
            # Tính toán quaternion neo của robot
            axang = self._anchor_tcp[3:]
            angle = np.linalg.norm(axang)
            if angle > 1e-6:
                axis = axang / angle
                qwxyz = transforms3d.quaternions.axangle2quat(axis, angle)
                self._anchor_quat_robot_xyzw = np.array([qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]])
            else:
                self._anchor_quat_robot_xyzw = np.array([0., 0., 0., 1.])
            
            # Lấy góc quay của mặt phẳng cánh tay cơ học (khớp đế q0)
            try:
                joints = self.ctrl.joints()
                if joints is not None and len(joints) > 0:
                    self._anchor_theta_arm = float(joints[0])
                else:
                    self._anchor_theta_arm = None
            except Exception:
                self._anchor_theta_arm = None
            
            self._ema_pos = np.copy(self._anchor_tcp[:3])
            self._ema_quat_xyzw = np.copy(self._anchor_quat_robot_xyzw)
            self._last_dominant_rot_axis = None
            self._last_button_state = current_button_state
            return None # Bỏ qua frame đầu tiên để tránh giật

        # ===============================
        # TÍNH TOÁN DELTA TỊNH TIẾN (WORLD-CENTRIC)
        # ===============================
        diag_msg = "STANDING STILL"
        delta_mag = 0.0
        
        R_q_anchor = R.from_quat(self._anchor_quat_quest_xyzw)
        R_r_anchor = R.from_quat(self._anchor_quat_robot_xyzw)
        
        if is_translating:
            current_pos_quest = np.array(frame["pos"])
            # delta_q_world là sự thay đổi vị trí tay người dùng trong KHÔNG GIAN THỰC
            # (Trái/Phải, Lên/Xuống, Tiến/Lùi) so với lúc vừa bấm cò
            delta_q_world = current_pos_quest - self._anchor_pos_quest
            
            # --- BỘ LỌC CHỐNG NHIỄU & CHẨN ĐOÁN ---
            delta_mag = np.linalg.norm(delta_q_world)
            
            if delta_mag < self.pos_deadband:  # Deadband tĩnh 6mm (loại bỏ vi rung sinh học khi bóp giữ cò)
                delta_q_world = np.zeros(3)
            else:
                # SOFT DEADBAND: Khử độ nhảy vọt khi vượt ngưỡng
                delta_q_world = delta_q_world * ((delta_mag - self.pos_deadband) / delta_mag)
                
                abs_delta = np.abs(delta_q_world)
                max_axis = np.argmax(abs_delta)
                
                # Chẩn đoán (Không gian WebXR: +X Phải, +Y Lên, +Z Lùi về phía người dùng)
                if max_axis == 1:
                    direction_word = "LÊN TRÊN" if delta_q_world[max_axis] > 0 else "XUỐNG DƯỚI"
                elif max_axis == 2:
                    direction_word = "LÙI (Về phía sếp)" if delta_q_world[max_axis] > 0 else "TIẾN (Đâm thẳng)"
                else:
                    direction_word = "SANG PHẢI" if delta_q_world[max_axis] > 0 else "SANG TRÁI"
                    
                diag_msg = f"{direction_word}"
                        
            # Lấy các thành phần di chuyển từ tay người dùng (WebXR):
            # - delta_q_world[2]: Lùi (+Z) / Tiến (-Z)
            # - delta_q_world[0]: Sang phải (+X) / Sang trái (-X)
            # - delta_q_world[1]: Lên (+Y) / Xuống (-Y)
            d_forward = -delta_q_world[2]   # Tiến / Lùi
            d_lateral =  delta_q_world[0]   # Phải / Trái
            d_vertical = delta_q_world[1]   # Lên / Xuống

            # Áp dụng Khóa trục thông minh (Soft Axis Snapping) trên cử động tay người
            if self.enable_axis_snapping:
                d_snapped = self._apply_soft_axis_snapping(np.array([d_forward, d_lateral, d_vertical]))
                d_forward, d_lateral, d_vertical = d_snapped[0], d_snapped[1], d_snapped[2]

            if self.nav_mode == "arm_plane" and self._anchor_tcp is not None:
                # --- CHẾ ĐỘ MẶT PHẲNG CÁNH TAY CƠ HỌC CHUẨN (TRUE MECHANICAL ARM PLANE) ---
                # Hướng "Tiến" là phương gập duỗi tự nhiên của bắp tay (trùng khớp với góc quay khớp đế q0).
                # Triệt tiêu hoàn toàn góc lệch cơ khí vai (13.1cm), giúp khớp đế q0 đứng im khi thu/duỗi tay thẳng.
                theta_arm = self._anchor_theta_arm
                if theta_arm is None:
                    x_anc = self._anchor_tcp[0]
                    y_anc = self._anchor_tcp[1]
                    r_anc = np.hypot(x_anc, y_anc)
                    theta_arm = np.arctan2(y_anc, x_anc) if r_anc > 0.03 else 0.0

                # Vector đơn vị hướng duỗi cánh tay (dọc theo mặt phẳng gập khớp vai & khuỷu): [cos(theta), sin(theta)]
                # Vector đơn vị tiếp tuyến sang phải: [sin(theta), -cos(theta)]
                dx_robot = d_forward * np.cos(theta_arm) + d_lateral * np.sin(theta_arm)
                dy_robot = d_forward * np.sin(theta_arm) - d_lateral * np.cos(theta_arm)
                dz_robot = d_vertical

                delta_robot_world = np.array([dx_robot, dy_robot, dz_robot])
            else:
                # Chế độ Cartesian Base Link truyền thống (Trục X/Y cố định của bàn máy)
                delta_robot_world = np.array([
                    d_forward,
                    -d_lateral,
                    d_vertical
                ])

            delta_robot_world = delta_robot_world * self.scale
            delta_robot_world = np.clip(delta_robot_world, -self.max_delta_m, self.max_delta_m)
        else:
            delta_robot_world = np.zeros(3)
            diag_msg = "CHỈ XOAY (KHÓA TỊNH TIẾN)"
            
        target_pos = self._anchor_tcp[:3] + delta_robot_world
        
        # Workspace safety clamp (Hardcoded UR3e safe zone)
        target_pos[0] = np.clip(target_pos[0], -0.50, 0.50) # X
        target_pos[1] = np.clip(target_pos[1], -0.50, 0.50) # Y
        target_pos[2] = np.clip(target_pos[2], -0.02, 0.60) # Z (an toàn sát mặt bàn)

        # Slew Rate Limiter: Chống đột biến giật nảy do rớt frame hoặc tracking bị nhảy cóc
        if self._ema_pos is not None:
            pos_step = target_pos - self._ema_pos
            step_mag = np.linalg.norm(pos_step)
            max_step = 0.035  # tối đa 3.5cm / 50ms = 0.7 m/s
            if step_mag > max_step:
                target_pos = self._ema_pos + pos_step * (max_step / step_mag)

        # Dynamic Adaptive EMA Filter cho Position:
        # - Khi đứng yên hoặc vi rung nhẹ (< 1cm): alpha = 0.18 (lọc phẳng 100% rung lắc)
        # - Khi di chuyển dứt khoát (> 3cm): alpha = 0.38 (nhạy bén bám sát cử động)
        if self._ema_pos is None:
            self._ema_pos = target_pos
        else:
            pos_diff = np.linalg.norm(target_pos - self._ema_pos)
            t_pos = np.clip(pos_diff / 0.03, 0.0, 1.0)
            smooth_pos = t_pos * t_pos * (3.0 - 2.0 * t_pos)
            alpha_pos = 0.18 + (0.38 - 0.18) * smooth_pos
            self._ema_pos = alpha_pos * target_pos + (1.0 - alpha_pos) * self._ema_pos

        current_time = time.time()
        if diag_msg != self._last_diag_msg or (current_time - self._last_diag_time > 1.5):
            if diag_msg != "STANDING STILL":
                tp = self._ema_pos
                print(f"[Chẩn đoán VR] Hướng: {diag_msg} | Lực: {delta_mag*100:.1f}cm | Tọa độ đích (X,Y,Z): [{tp[0]:.3f}, {tp[1]:.3f}, {tp[2]:.3f}]")
            self._last_diag_msg = diag_msg
            self._last_diag_time = current_time

        # ===============================
        # TÍNH TOÁN DELTA XOAY (DECOUPLED 3-AXIS / TABLE ROLL / FREE 3D)
        # ===============================
        if is_rotating:
            curr_q = frame["quat"]
            curr_quat_quest_xyzw = np.array([curr_q[1], curr_q[2], curr_q[3], curr_q[0]])
            R_hand_current = R.from_quat(curr_quat_quest_xyzw)

            # 1. Delta xoay trong Không gian Thực (WebXR World Frame):
            # R_delta_world = R_hand_current * R_q_anchor.inv()
            # Trực giao 100% giữa các trục không gian phòng, triệt tiêu hoàn toàn góc nghiêng tay cầm:
            # - omega_world[0]: Gập/ngửa cổ tay lên xuống (Pitch quanh trục X phòng)
            # - omega_world[1]: Quét/lắc cổ tay trái phải quanh trục thẳng đứng (Yaw quanh trục Y phòng)
            # - omega_world[2]: Nghiêng/vặn cổ tay quanh hướng nhìn tới (Roll quanh trục Z phòng)
            R_delta_world = R_hand_current * R_q_anchor.inv()
            omega_world = R_delta_world.as_rotvec()

            # Chiếu vào 3 trục điều khiển tự nhiên của người dùng:
            # - Pitch: Đảo ngược chiều theo yêu cầu: dốc tay về phía mình thì ngửa ra xa thân máy!
            # - Hoán đổi & phân định rõ 3 trục cơ học:
            #   + Lắc cổ tay trái/phải (-omega_world[1]) -> Xoay tròn hai ngón kẹp quanh trục kẹp (Tool Z Roll)
            #   + Vặn chìa khóa (omega_world[2]) -> Nghiêng trái / nghiêng phải (Lateral Tilt quanh u_reach)
            d_pitch =  omega_world[0]
            d_yaw   =  omega_world[2]   # Cử chỉ Vặn chìa khóa quanh cẳng tay: nghiêng trái / phải
            d_roll  = -omega_world[1]   # Cử chỉ Lắc cổ tay trái/phải: xoay ngàm đón vật

            raw_rotvec = np.array([d_pitch, d_yaw, d_roll]) * self.rot_scale

            if self.rotation_mode == "table_roll":
                # Chế độ Proposal 4 (Khóa chúc ngàm, chỉ Roll quanh trục kẹp):
                mag_r = abs(d_roll)
                rot_deadband = self.rot_deadband
                if mag_r < rot_deadband:
                    delta_roll = 0.0
                else:
                    delta_roll = np.sign(d_roll) * (mag_r - rot_deadband)
                R_delta_arm = R.identity()
                R_delta_tool = R.from_rotvec([0.0, 0.0, delta_roll])

                if not is_translating:
                    roll_deg = np.degrees(delta_roll)
                    if abs(roll_deg) > 2.0:
                        direction_str = "PHẢI (CW)" if delta_roll > 0 else "TRÁI (CCW)"
                        diag_msg = f"XOAY NGÀM {direction_str} ({abs(roll_deg):.1f}° | Lắc Cổ Tay)"
                    else:
                        diag_msg = "GIỮ HƯỚNG CŨ"
            elif self.rotation_mode == "free_3d":
                # Chế độ 3D tự do (không khóa trục)
                mag = np.linalg.norm(raw_rotvec)
                rot_deadband = self.rot_deadband
                if mag < rot_deadband:
                    rotvec_tcp = np.zeros(3)
                else:
                    rotvec_tcp = raw_rotvec * ((mag - rot_deadband) / mag)
                R_delta_arm = R.identity()
                R_delta_tool = R.from_rotvec(rotvec_tcp)

                if not is_translating:
                    abs_v = np.abs(rotvec_tcp)
                    max_idx = int(np.argmax(abs_v))
                    max_deg = np.degrees(abs_v[max_idx])
                    axis_names = ["GẬP/CHÚC (Pitch)", "NGHIÊNG TRÁI/PHẢI (Tilt)", "LẮC XOAY NGÀM (Roll)"]
                    if max_deg > 2.0:
                        diag_msg = f"XOAY 3D TỰ DO {axis_names[max_idx]} ({max_deg:.1f}°)"
                    else:
                        diag_msg = "GIỮ HƯỚNG CŨ"
            else:
                # Chế độ chuẩn DECOUPLED 3-AXIS (Mặc định: Kết hợp Phương án A + B):
                # 1. Phương án A: Khóa rãnh đơn trục (Soft Rotation Snapping với ngưỡng 55%) triệt tiêu 100% rung lắc
                if self.enable_axis_snapping:
                    rotvec_filtered = self._apply_soft_rotation_snapping(raw_rotvec)
                else:
                    mag = np.linalg.norm(raw_rotvec)
                    rotvec_filtered = np.zeros(3) if mag < self.rot_deadband else raw_rotvec * ((mag - self.rot_deadband) / mag)

                pitch_val = rotvec_filtered[0]
                yaw_val   = rotvec_filtered[1]
                roll_val  = rotvec_filtered[2]

                # 2. Phương án B: Đồng bộ xoay theo Mặt phẳng cánh tay robot (Arm-Plane Rotation)
                if self.nav_mode == "arm_plane":
                    theta_arm = self._anchor_theta_arm
                    if theta_arm is None:
                        x_anc = self._anchor_tcp[0]
                        y_anc = self._anchor_tcp[1]
                        r_anc = np.hypot(x_anc, y_anc)
                        theta_arm = np.arctan2(y_anc, x_anc) if r_anc > 0.03 else 0.0

                    # Vector đơn vị vươn dọc theo cánh tay trong Base:
                    u_reach = np.array([np.cos(theta_arm), np.sin(theta_arm), 0.0])
                    # Trục ngang tiếp tuyến vuông góc với phương duỗi cánh tay trong Base:
                    u_lateral = np.array([np.sin(theta_arm), -np.cos(theta_arm), 0.0])

                    # Pitch gập dọc mặt phẳng cánh tay vươn (xoay quanh u_lateral)
                    # Tilt nghiêng trái/phải dọc theo trục vươn của cánh tay (xoay quanh u_reach)
                    rotvec_arm = u_lateral * pitch_val + u_reach * yaw_val
                    R_delta_arm = R.from_rotvec(rotvec_arm)
                    # Roll vặn quanh trục kẹp Tool Z (Lắc cổ tay trái/phải)
                    R_delta_tool = R.from_rotvec([0.0, 0.0, roll_val])
                else:
                    R_delta_arm = R.identity()
                    R_delta_tool = R.from_rotvec(rotvec_filtered)

                if not is_translating:
                    abs_v = np.abs(rotvec_filtered)
                    max_idx = int(np.argmax(abs_v))
                    max_deg = np.degrees(abs_v[max_idx])
                    axis_names = ["GẬP/CHÚC (Pitch)", "NGHIÊNG TRÁI/PHẢI (Tilt)", "LẮC XOAY NGÀM (Roll)"]
                    if max_deg > 2.0:
                        ratios = [abs_v[i] / (abs_v[max_idx] + 1e-6) for i in range(3)]
                        secondary = [i for i in range(3) if i != max_idx and ratios[i] > 0.50]
                        if secondary:
                            diag_msg = f"XOAY 3D ĐA TRỤC ({max_deg:.1f}° | {axis_names[max_idx]})"
                        else:
                            diag_msg = f"XOAY {axis_names[max_idx]} ({max_deg:.1f}° | Lọc độc lập)"
                    else:
                        diag_msg = "GIỮ HƯỚNG CŨ"

            # Áp dụng góc xoay vào Robot:
            # - R_delta_arm tác động trong hệ tọa độ Base / Arm-Plane (Gập và Quét theo mặt phẳng tay)
            # - R_delta_tool tác động trong hệ tọa độ Tool TCP (Vặn chìa khóa Roll)
            R_target_robot = R_delta_arm * R_r_anchor * R_delta_tool
        else:
            self._last_dominant_rot_axis = None
            R_target_robot = R_r_anchor
        target_quat_xyzw = R_target_robot.as_quat()
        
        # Dynamic Adaptive EMA Filter cho Quaternion (mượt mà, triệt tiêu vi rung)
        if self._ema_quat_xyzw is None:
            self._ema_quat_xyzw = target_quat_xyzw
        else:
            if np.dot(self._ema_quat_xyzw, target_quat_xyzw) < 0:
                target_quat_xyzw = -target_quat_xyzw
            
            # Tính khoảng cách góc trắc địa (Geodesic Angle Distance) trên SO(3)
            dot_val = np.clip(abs(np.dot(self._ema_quat_xyzw, target_quat_xyzw)), -1.0, 1.0)
            rot_diff = 2.0 * np.arccos(dot_val)
            
            # Slew Rate Limiter cho góc xoay (chống giật đột biến nếu tracking bị vấp)
            max_rot_step = 0.20  # ~11.5 độ / 50ms
            if rot_diff > max_rot_step:
                s_factor = max_rot_step / rot_diff
                target_quat_xyzw = (1.0 - s_factor) * self._ema_quat_xyzw + s_factor * target_quat_xyzw
                target_quat_xyzw /= np.linalg.norm(target_quat_xyzw)
                rot_diff = max_rot_step

            # Dynamic Adaptive Alpha cho góc xoay:
            # - Khi đứng yên hoặc vi rung chậm (< 4 độ): alpha = 0.18 (lọc phẳng 100% rung lắc)
            # - Khi xoay dứt khoát (> 15 độ): alpha = 0.48 (phản hồi tức thì, không trễ)
            t_rot = np.clip(rot_diff / 0.25, 0.0, 1.0)
            smooth_rot = t_rot * t_rot * (3.0 - 2.0 * t_rot)
            alpha_rot = 0.18 + (0.48 - 0.18) * smooth_rot
            
            self._ema_quat_xyzw = alpha_rot * target_quat_xyzw + (1.0 - alpha_rot) * self._ema_quat_xyzw
            self._ema_quat_xyzw /= np.linalg.norm(self._ema_quat_xyzw)
            
        target_quat_wxyz = [self._ema_quat_xyzw[3], self._ema_quat_xyzw[0], self._ema_quat_xyzw[1], self._ema_quat_xyzw[2]]

        return {
            "pos": self._ema_pos.tolist(),
            "quat": target_quat_wxyz
        }

    def get_current_pose(self):
        """Dùng cho lúc init hệ thống"""
        try:
            tcp = self.ctrl.tcp()
            angle = np.linalg.norm(tcp[3:])
            if angle > 1e-6:
                axis = np.array(tcp[3:]) / angle
                quat_wxyz = transforms3d.quaternions.axangle2quat(axis, angle)
            else:
                quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
            return {
                "pos": tcp[:3],
                "quat": quat_wxyz.tolist()
            }
        except:
            return None

    def reset_anchor(self):
        self._last_button_state = 0
        self._anchor_pos_quest = None
        self._anchor_quat_quest_xyzw = None
        self._anchor_tcp = None
        self._anchor_quat_robot_xyzw = None
        self._anchor_theta_arm = None
        self._ema_pos = None
        self._ema_quat_xyzw = None
        self._last_dominant_rot_axis = None
        self.out_of_reach_time = 0.0
