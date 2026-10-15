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

        self._last_button_state = 0
        self.out_of_reach_time = 0.0

    def _apply_soft_axis_snapping(self, delta_robot):
        """
        Khóa trục thông minh (Soft Axis Snapping):
        Triệt tiêu rung lắc / lệch tay sinh trắc học khi người dùng chủ đích di chuyển theo một trục chính,
        nhưng vẫn chuyển tiếp mềm mại (không giật) khi người dùng chủ đích đi chéo 2D/3D.
        """
        dx, dy, dz = delta_robot[0], delta_robot[1], delta_robot[2]
        
        # 1. Khóa trục mặt phẳng ngang (X - Tiến/Lùi vs Y - Trái/Phải):
        abs_x = abs(dx)
        abs_y = abs(dy)
        
        if abs_x > 0.005 or abs_y > 0.005:
            if abs_x >= abs_y:
                ratio = abs_y / (abs_x + 1e-6)
                # Nếu tỷ lệ trôi ngang < 20%: Khóa hoàn toàn về 0
                # Nếu 20% <= tỷ lệ <= 45%: Hòa trộn mượt (smooth blend)
                # Nếu > 45%: Giữ nguyên cho phép đi chéo tự do
                if ratio < 0.20:
                    scale_y = 0.0
                elif ratio < 0.45:
                    scale_y = (ratio - 0.20) / (0.45 - 0.20)
                else:
                    scale_y = 1.0
                dy = dy * scale_y
            else:
                ratio = abs_x / (abs_y + 1e-6)
                if ratio < 0.20:
                    scale_x = 0.0
                elif ratio < 0.45:
                    scale_x = (ratio - 0.20) / (0.45 - 0.20)
                else:
                    scale_x = 1.0
                dx = dx * scale_x

        # 2. Tách biệt mặt phẳng ngang (X, Y) và trục đứng (Z):
        # Tránh trôi độ cao khi quét mặt phẳng, hoặc tránh trôi mặt phẳng khi nhấc thẳng lên/hạ thẳng xuống
        d_xy = np.sqrt(dx * dx + dy * dy)
        abs_z = abs(dz)
        
        if d_xy > 0.025 and abs_z < 0.012:
            # Người dùng đang trượt phẳng trên mặt bàn: Nén rung lắc trục Z
            dz = dz * (abs_z / 0.012)
        elif abs_z > 0.025 and d_xy < 0.012:
            # Người dùng đang nhấc/hạ thẳng đứng: Nén rung lắc mặt phẳng XY
            factor = d_xy / 0.012
            dx = dx * factor
            dy = dy * factor

        return np.array([dx, dy, dz])

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
            
            self._ema_pos = None
            self._ema_quat_xyzw = None
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
            
            if delta_mag < 0.005:  # Deadband tĩnh 0.5cm (5mm)
                delta_q_world = np.zeros(3)
            else:
                # SOFT DEADBAND: Khử độ nhảy vọt khi vượt ngưỡng
                delta_q_world = delta_q_world * ((delta_mag - 0.005) / delta_mag)
                
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

        if self._ema_pos is None:
            self._ema_pos = target_pos
        else:
            self._ema_pos = self.alpha * target_pos + (1.0 - self.alpha) * self._ema_pos

        current_time = time.time()
        if diag_msg != self._last_diag_msg or (current_time - self._last_diag_time > 1.5):
            if diag_msg != "STANDING STILL":
                tp = self._ema_pos
                print(f"[Chẩn đoán VR] Hướng: {diag_msg} | Lực: {delta_mag*100:.1f}cm | Tọa độ đích (X,Y,Z): [{tp[0]:.3f}, {tp[1]:.3f}, {tp[2]:.3f}]")
            self._last_diag_msg = diag_msg
            self._last_diag_time = current_time

        # ===============================
        # TÍNH TOÁN DELTA XOAY (TOOL-CENTRIC)
        # ===============================
        if is_rotating:
            curr_q = frame["quat"]
            curr_quat_quest_xyzw = np.array([curr_q[1], curr_q[2], curr_q[3], curr_q[0]])
            R_hand_current = R.from_quat(curr_quat_quest_xyzw)
            
            # R_delta trong không gian tay cầm (Local)
            R_delta_hand_local = R_q_anchor.inv() * R_hand_current
            
            # --- BỘ LỌC DEADBAND XOAY CỔ TAY (SOFT ROTATION DEADBAND) ---
            angle_rad = R_delta_hand_local.magnitude()
            rot_deadband = 0.06  # ~3.4 độ (êm ái, nhạy bén)
            if angle_rad < rot_deadband:
                R_delta_hand_local = R.identity()
            else:
                # Trừ hao phần deadband để góc xoay bắt đầu từ 0 một cách mượt mà
                scale_factor = (angle_rad - rot_deadband) / angle_rad
                rotvec = R_delta_hand_local.as_rotvec()
                R_delta_hand_local = R.from_rotvec(rotvec * scale_factor)
                
            # Ánh xạ vector góc xoay từ Tay cầm sang TCP (Tool-Centric)
            # Meta Quest GripSpace: +X(Phải), -Y(Tiến), -Z(Lên)
            # UR TCP Local: +Z(Tiến), -X(Lên), +Y(Phải)
            M_local_q2tcp = np.array([
                [ 0,  0,  1], # TCP X (Lên/Xuống) = Quest Z
                [ 1,  0,  0], # TCP Y (Trái/Phải) = Quest X
                [ 0, -1,  0]  # TCP Z (Tiến/Lùi)  = - Quest Y
            ])
            rotvec_hand = R_delta_hand_local.as_rotvec()
            rotvec_tcp = M_local_q2tcp.dot(rotvec_hand)
            R_delta_tcp_local = R.from_rotvec(rotvec_tcp)
        else:
            R_delta_tcp_local = R.identity()
            
        # Áp dụng góc xoay vào TCP hiện tại
        R_target_robot = R_r_anchor * R_delta_tcp_local
        target_quat_xyzw = R_target_robot.as_quat()
        
        # EMA Filter cho Quaternion
        if self._ema_quat_xyzw is None:
            self._ema_quat_xyzw = target_quat_xyzw
        else:
            if np.dot(self._ema_quat_xyzw, target_quat_xyzw) < 0:
                target_quat_xyzw = -target_quat_xyzw
            
            self._ema_quat_xyzw = self.alpha * target_quat_xyzw + (1.0 - self.alpha) * self._ema_quat_xyzw
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
        self.out_of_reach_time = 0.0
