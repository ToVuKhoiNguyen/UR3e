import math
import numpy as np
import torch
from safety.singularity_guard.jacobian_metrics import JacobianMetrics

class SingularityGuard:
    def __init__(self, config):
        self.warn_threshold = config["thresholds"]["sigma_min_warn"]
        self.danger_threshold = config["thresholds"]["sigma_min_danger"]
        self.max_qd_ratio = config["thresholds"].get("max_qd_ratio", 1.0)
        self.metrics_calc = JacobianMetrics(robot_config_file=config["robot"]["model_file"])
        
        # UR3e joint velocity limits (rad/s)
        self.vel_limits = torch.tensor([3.14, 3.14, 3.14, 3.14, 3.14, 3.14], device=self.metrics_calc.device)

    def check_trajectory(self, q_traj: list, dt: float = 0.05):
        """
        Kiểm tra một quỹ đạo gồm nhiều waypoint.
        Trả về: is_safe (bool), reason (str)
        """
        if len(q_traj) < 2:
            return True, "Too short"
            
        q_tensor = torch.tensor(q_traj, device=self.metrics_calc.device, dtype=torch.float32)
        
        # 1. Tính metric cho toàn bộ waypoint
        metrics = self.metrics_calc.get_metrics(q_tensor)
        sigma_min = metrics["sigma_min"]
        
        min_sigma = torch.min(sigma_min).item()
        if min_sigma < self.danger_threshold:
            return False, f"Singularity Danger: sigma_min={min_sigma:.5f} < {self.danger_threshold}"
            
        # 2. Tính vận tốc qd
        # qd = (q_{t} - q_{t-1}) / dt
        qd = (q_tensor[1:] - q_tensor[:-1]) / dt
        abs_qd = torch.abs(qd)
        
        # Tính tỷ lệ so với giới hạn
        ratio = abs_qd / self.vel_limits.unsqueeze(0)
        max_ratio = torch.max(ratio).item()
        
        if max_ratio > self.max_qd_ratio:
            # Tìm khớp bị vượt limit
            max_idx = torch.argmax(ratio).item()
            joint_idx = max_idx % 6
            return False, f"Velocity Spike: Joint {joint_idx} ratio={max_ratio:.2f} > {self.max_qd_ratio}"
            
        if min_sigma < self.warn_threshold:
            return True, f"Warning: sigma_min={min_sigma:.5f} (safe but close)"
            
        return True, "Safe"

    def clamp_step(self, q_current, q_target, dt=0.1):
        """
        Giới hạn bước nhảy (velocity clamping) để không vượt quá giới hạn khớp.
        Giúp robot lách qua singularity thay vì báo lỗi.
        """
        q_c = np.array(q_current)
        q_t = np.array(q_target)
        delta_q = q_t - q_c
        
        # Vận tốc yêu cầu
        qd = np.abs(delta_q) / dt
        max_qd_req = np.max(qd)
        
        # Giới hạn vận tốc an toàn (ví dụ: 1.0 rad/s)
        safe_qd_limit = self.max_qd_ratio * 3.14 # max_qd_ratio * pi rad/s
        
        if max_qd_req > safe_qd_limit:
            # Thu nhỏ delta_q theo tỷ lệ
            scale = safe_qd_limit / max_qd_req
            delta_q = delta_q * scale
            q_safe = q_c + delta_q
            return q_safe.tolist(), True # bị kẹp (clamped)
            
        return q_target, False # bình thường


    def filter_ik_solutions(self, q_candidates: torch.Tensor):
        """
        Nhận vào một batch các giải pháp IK, trả về các giải pháp an toàn
        """
        metrics = self.metrics_calc.get_metrics(q_candidates)
        safe_mask = metrics["sigma_min"] >= self.danger_threshold
        
        return q_candidates[safe_mask], safe_mask
