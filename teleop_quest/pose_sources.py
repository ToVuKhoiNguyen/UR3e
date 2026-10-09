import numpy as np

class WidgetPoseSource:
    """Nguồn cấp tọa độ cho Controller (Đọc từ Widget)"""
    def __init__(self, target_widget, alpha=0.12):
        self.target_widget = target_widget
        self.alpha = alpha
        self._ema_pos = None
        self._ema_quat_xyzw = None
        
    def get_pose(self):
        target_pos = np.array(self.target_widget.position)
        target_quat_wxyz = self.target_widget.wxyz
        target_quat_xyzw = np.array([target_quat_wxyz[1], target_quat_wxyz[2], target_quat_wxyz[3], target_quat_wxyz[0]])
        
        if self._ema_pos is None:
            self._ema_pos = target_pos
            self._ema_quat_xyzw = target_quat_xyzw
        else:
            self._ema_pos = self.alpha * target_pos + (1.0 - self.alpha) * self._ema_pos
            
            # Slerp/EMA cho Quaternion
            if np.dot(self._ema_quat_xyzw, target_quat_xyzw) < 0:
                target_quat_xyzw = -target_quat_xyzw
            self._ema_quat_xyzw = self.alpha * target_quat_xyzw + (1.0 - self.alpha) * self._ema_quat_xyzw
            self._ema_quat_xyzw /= np.linalg.norm(self._ema_quat_xyzw)
            
        ema_quat_wxyz = [self._ema_quat_xyzw[3], self._ema_quat_xyzw[0], self._ema_quat_xyzw[1], self._ema_quat_xyzw[2]]
        return {"pos": self._ema_pos.tolist(), "quat": ema_quat_wxyz}

def get_robot_fk_as_ur(teleop, ctrl):
    """
    Dùng vị trí khớp thật (q_real) để tính FK bằng bộ não CuRobo.
    Giúp loại bỏ sai lệch Calibration giữa URSim thật và CuRobo URDF.
    """
    import transforms3d
    q_real = ctrl.joints()
    fk_dict = teleop._curobo.fk(q_real)
    pos = fk_dict["pos"]
    quat_wxyz = fk_dict["quat"]
    axis, angle = transforms3d.quaternions.quat2axangle(quat_wxyz)
    return [pos[0], pos[1], pos[2], axis[0]*angle, axis[1]*angle, axis[2]*angle]
