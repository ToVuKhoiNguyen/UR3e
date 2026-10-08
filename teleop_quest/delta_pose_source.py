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
        
        self._last_trigger = False

    def get_pose(self):
        frame = self.server.get_frame()
        if not frame:
            return None

        # Dead-man switch: phải bóp cả Trigger (cò) + Grip (hông)
        current_trigger = frame.get("trigger", False) and frame.get("grip", False)
        
        if not current_trigger:
            self._last_trigger = False
            return None

        # Cạnh lên của Trigger -> Ghi nhớ điểm neo (anchor)
        if current_trigger and not self._last_trigger:
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
            self._last_trigger = True
            return None # Bỏ qua frame đầu tiên để tránh giật

        # Tính toán Delta
        current_pos_quest = np.array(frame["pos"])
        delta_quest = current_pos_quest - self._anchor_pos_quest
        
        # Ánh xạ hệ tọa độ và scale (World-centric)
        delta_robot = self.R_q2r @ delta_quest * self.scale
        
        # Safety clamp delta
        delta_robot = np.clip(delta_robot, -self.max_delta_m, self.max_delta_m)
        
        # Target position
        target_pos = self._anchor_tcp[:3] + delta_robot
        
        # Workspace safety clamp (Hardcoded UR3e safe zone)
        target_pos[0] = np.clip(target_pos[0], -0.50, 0.50) # X
        target_pos[1] = np.clip(target_pos[1], -0.50, 0.50) # Y
        target_pos[2] = np.clip(target_pos[2],  0.05, 0.60) # Z (min 5cm above table)

        # Áp dụng bộ lọc EMA (Exponential Moving Average) để chống giật
        if self._ema_pos is None:
            self._ema_pos = target_pos
        else:
            self._ema_pos = self.alpha * target_pos + (1.0 - self.alpha) * self._ema_pos

        # ===============================
        # TÍNH TOÁN DELTA XOAY (ROTATION)
        # ===============================
        from scipy.spatial.transform import Rotation as R
        
        curr_q = frame["quat"]
        curr_quat_quest_xyzw = np.array([curr_q[1], curr_q[2], curr_q[3], curr_q[0]])
        
        R_hand_current = R.from_quat(curr_quat_quest_xyzw)
        R_hand_anchor = R.from_quat(self._anchor_quat_quest_xyzw)
        
        # R_delta trong không gian tay cầm
        R_delta_hand = R_hand_current * R_hand_anchor.inv()
        
        # Đưa R_delta sang không gian Base của Robot
        mat_delta_hand = R_delta_hand.as_matrix()
        mat_delta_robot = self.R_q2r @ mat_delta_hand @ self.R_q2r.T
        R_delta_robot = R.from_matrix(mat_delta_robot)
        
        # Áp dụng vào pose neo của Robot
        R_robot_anchor = R.from_quat(self._anchor_quat_robot_xyzw)
        R_target_robot = R_delta_robot * R_robot_anchor
        target_quat_xyzw = R_target_robot.as_quat()
        
        # EMA Filter cho Quaternion (Chống giật góc)
        if self._ema_quat_xyzw is None:
            self._ema_quat_xyzw = target_quat_xyzw
        else:
            if np.dot(self._ema_quat_xyzw, target_quat_xyzw) < 0:
                target_quat_xyzw = -target_quat_xyzw
            
            self._ema_quat_xyzw = self.alpha * target_quat_xyzw + (1.0 - self.alpha) * self._ema_quat_xyzw
            self._ema_quat_xyzw /= np.linalg.norm(self._ema_quat_xyzw)
            self._ema_quat_xyzw /= np.linalg.norm(self._ema_quat_xyzw)
            
        # Trả về định dạng wxyz cho cuRobo
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
