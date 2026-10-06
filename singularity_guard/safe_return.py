import torch
import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from curobo.motion_planner import MotionPlannerCfg, MotionPlanner
from curobo.types import JointState

class SafeReturn:
    def __init__(self, config, ctrl_instance, guard_instance):
        self.safe_pose = config["safe_return"]["safe_pose"]
        self.speed = config["safe_return"]["recovery_speed"]
        self.ctrl = ctrl_instance
        self.guard = guard_instance
        
        # Verify safe pose is actually safe
        q_safe = torch.tensor([self.safe_pose], device=guard_instance.metrics_calc.device, dtype=torch.float32)
        sigma_safe = guard_instance.metrics_calc.get_metrics(q_safe)["sigma_min"].item()
        
        if sigma_safe < guard_instance.warn_threshold:
            print(f"[SafeReturn] CẢNH BÁO: Safe pose trong config không đủ an toàn! (sigma_min = {sigma_safe:.5f})")
            
        scene_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), "obstacle_scene.yml")
        mp_cfg = MotionPlannerCfg.create(robot="ur3e.yml", scene_model=scene_file)
        self.mp = MotionPlanner(mp_cfg)
        self.mp.warmup(enable_graph=False, num_warmup_iterations=1)

    def execute_recovery(self, current_q):
        print(f"[SafeReturn] Đang tính toán đường thoát an toàn về {self.safe_pose}...")
        
        q_start = JointState.from_position(
            torch.tensor([current_q], dtype=torch.float32, device="cuda"),
            joint_names=self.mp.joint_names
        )
        q_goal = JointState.from_position(
            torch.tensor([self.safe_pose], dtype=torch.float32, device="cuda"),
            joint_names=self.mp.joint_names
        )
        
        result = self.mp.plan_cspace(q_goal, current_state=q_start)
        if result is not None and result.success.any():
            traj = result.solution.squeeze().cpu().tolist()
            
            # Bỏ qua check_trajectory khắt khe vì đây là quỹ đạo không gian khớp (Joint Space - movej),
            # việc đi ngang qua singularity không gây ra vận tốc vô cực như không gian Cartesian.
            print(f"[SafeReturn] Đã tìm thấy đường thoát ({len(traj)} wps). Đang thực thi...")
            for wp in traj:
                self.ctrl.move_joints(wp, speed=self.speed, wait=True)
            print("[SafeReturn] Đã về safe pose thành công!")
        else:
            print("[SafeReturn] Không thể tính toán đường thoát!")
            self._fallback_recovery(current_q)
            
    def _fallback_recovery(self, current_q):
        print("[SafeReturn] Thử chuyển động ngắn để tăng sigma_min...")
        # Lấy một cấu hình có sigma lớn hơn từ tập lân cận
        q_tensor = torch.tensor([current_q], device="cuda", dtype=torch.float32)
        best_q = current_q
        best_sigma = self.guard.metrics_calc.get_metrics(q_tensor)["sigma_min"].item()
        
        for i in range(6):
            for delta in [-0.1, 0.1]:
                q_test = list(current_q)
                q_test[i] += delta
                q_test_t = torch.tensor([q_test], device="cuda", dtype=torch.float32)
                sigma = self.guard.metrics_calc.get_metrics(q_test_t)["sigma_min"].item()
                
                if sigma > best_sigma:
                    best_sigma = sigma
                    best_q = q_test
                    
        if best_sigma > self.guard.metrics_calc.get_metrics(q_tensor)["sigma_min"].item():
            print(f"[SafeReturn] Nhích nhẹ khớp để thoát kẹt (sigma_min lên {best_sigma:.5f})")
            self.ctrl.move_joints(best_q, speed=0.1, wait=True)
            # Gọi lại recovery
            self.execute_recovery(best_q)
        else:
            print("[SafeReturn] CẠN KIỆT GIẢI PHÁP: Không thể thoát khỏi singularity tự động!")
