import numpy as np

class DeltaPoseSource:
    """
    Implements PoseSource contract.
    Chuyển delta tọa độ từ kính Quest sang base_link của UR3e.
    """
    def __init__(self, server, ctrl, scale: float = 1.0, max_delta_m: float = 0.05, anchor_reference_func=None, alpha: float = 0.15):
        self.server = server
        self.ctrl = ctrl
        self.scale = scale
        self.max_delta_m = max_delta_m
        self.anchor_reference_func = anchor_reference_func
        self.alpha = alpha
        
        self._last_diag_msg = ""
        self._last_diag_time = 0

        # Ma trận xoay từ Quest local space sang UR3e base_link
        # Giả định sếp đứng đối diện mặt trước của robot:
        # Quest X (sang phải) -> UR3e -Y (trái của robot)
        # Quest Y (lên trên) -> UR3e +Z (lên trên)
        # Quest Z (lùi lại) -> UR3e -X (về phía sau robot)
        self.R_q2r = np.array([
            [ 0,  0, -1],  
            [-1,  0,  0],  
            [ 0,  1,  0],  
        ])

        self._anchor_pos_quest = None
        self._anchor_quat_quest_xyzw = None
        self._anchor_tcp = None
        self._anchor_quat_robot_xyzw = None
        
        self._ema_pos = None
        self._ema_quat_xyzw = None
        
        self._last_button_state = 0

    def get_pose(self):
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
            import transforms3d
            axang = self._anchor_tcp[3:]
            angle = np.linalg.norm(axang)
            if angle > 1e-6:
                axis = axang / angle
                qwxyz = transforms3d.quaternions.axangle2quat(axis, angle)
                self._anchor_quat_robot_xyzw = np.array([qwxyz[1], qwxyz[2], qwxyz[3], qwxyz[0]])
            else:
                self._anchor_quat_robot_xyzw = np.array([0., 0., 0., 1.])
            
            self._ema_pos = None
            self._ema_quat_xyzw = None
            self._last_button_state = current_button_state
            return None # Bỏ qua frame đầu tiên để tránh giật

        # ===============================
        # TÍNH TOÁN DELTA TỊNH TIẾN (TOOL-CENTRIC)
        # ===============================
        diag_msg = "STANDING STILL"
        delta_mag = 0.0
        
        from scipy.spatial.transform import Rotation as R
        R_q_anchor = R.from_quat(self._anchor_quat_quest_xyzw)
        R_r_anchor = R.from_quat(self._anchor_quat_robot_xyzw)
        
        # Meta Quest GripSpace: +X(Phải), -Y(Tiến), -Z(Lên)
        # UR TCP Local: +Z(Tiến), -X(Lên), +Y(Phải)
        M_local_q2tcp = np.array([
            [ 0,  0,  1], # TCP X (Lên/Xuống) = Quest Z
            [ 1,  0,  0], # TCP Y (Trái/Phải) = Quest X
            [ 0, -1,  0]  # TCP Z (Tiến/Lùi)  = - Quest Y
        ])
        
        if is_translating:
            current_pos_quest = np.array(frame["pos"])
            delta_q_world = current_pos_quest - self._anchor_pos_quest
            
            # 1. Chuyển delta tịnh tiến về Local của tay cầm lúc bấm cò
            delta_q_local = R_q_anchor.inv().apply(delta_q_world)
            
            # --- BỘ LỌC CHỐNG NHIỄU & CHẨN ĐOÁN (DIAGNOSTICS) ---
            delta_mag = np.linalg.norm(delta_q_local)
            
            if delta_mag < 0.015:  # Deadband tĩnh: Tăng lên 1.5cm để triệt tiêu hoàn toàn nhiễu rung tay
                delta_q_local = np.zeros(3)
            else:
                # SOFT DEADBAND: Khử độ nhảy vọt (Jerk) khi vượt qua ngưỡng 1.5cm.
                delta_q_local = delta_q_local * ((delta_mag - 0.015) / delta_mag)
                
                abs_delta = np.abs(delta_q_local)
                max_axis = np.argmax(abs_delta)
                
                # Tỷ lệ snapping 60%: Khóa trục gắt hơn để chặn các hướng dịch chuyển sai ngoài ý muốn
                for i in range(3):
                    if i != max_axis and abs_delta[i] < 0.6 * abs_delta[max_axis]:
                        delta_q_local[i] = 0.0
                        
                # Chẩn đoán (Quest Local theo GripSpace thực tế: X=Trái/Phải, Y=Lùi/Tiến, Z=Xuống/Lên)
                if max_axis == 2:
                    direction_word = "LÊN TRÊN" if delta_q_local[max_axis] < 0 else "XUỐNG DƯỚI"
                elif max_axis == 1:
                    direction_word = "TIẾN (Đâm thẳng)" if delta_q_local[max_axis] < 0 else "LÙI (Về phía sếp)"
                else:
                    direction_word = "SANG PHẢI" if delta_q_local[max_axis] > 0 else "SANG TRÁI"
                    
                diag_msg = f"{direction_word}"
                        
            # 2. Ánh xạ Local Quest sang Local TCP
            delta_tcp_local = M_local_q2tcp.dot(delta_q_local)
        else:
            delta_tcp_local = np.zeros(3)
            diag_msg = "CHỈ XOAY (KHÓA TỊNH TIẾN)"
            
        # 3. Chuyển Delta Local TCP về Robot World Space
        delta_robot_world = R_r_anchor.apply(delta_tcp_local) * self.scale
        delta_robot_world = np.clip(delta_robot_world, -self.max_delta_m, self.max_delta_m)
        
        target_pos = self._anchor_tcp[:3] + delta_robot_world
        
        # Workspace safety clamp (Hardcoded UR3e safe zone)
        target_pos[0] = np.clip(target_pos[0], -0.50, 0.50) # X
        target_pos[1] = np.clip(target_pos[1], -0.50, 0.50) # Y
        target_pos[2] = np.clip(target_pos[2],  0.05, 0.60) # Z (min 5cm above table)

        if self._ema_pos is None:
            self._ema_pos = target_pos
        else:
            self._ema_pos = self.alpha * target_pos + (1.0 - self.alpha) * self._ema_pos

        import time
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
            rot_deadband = 0.20  # ~11.5 độ
            if angle_rad < rot_deadband:
                R_delta_hand_local = R.identity()
            else:
                # Trừ hao phần deadband để góc xoay bắt đầu từ 0 một cách mượt mà
                scale_factor = (angle_rad - rot_deadband) / angle_rad
                rotvec = R_delta_hand_local.as_rotvec()
                R_delta_hand_local = R.from_rotvec(rotvec * scale_factor)
                
            # Ánh xạ vector góc xoay từ Tay cầm sang TCP
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
            import transforms3d
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
        self._last_trigger = False
