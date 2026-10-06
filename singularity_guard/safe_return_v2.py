"""
safe_return_v2.py - Phiên bản sửa lỗi của SafeReturn.

Thay đổi so với v1:
- Gửi toàn bộ traj bằng 1 URScript duy nhất (thay vì gọi move_joints nhiều lần)
  → Tránh wait_for_move timeout/reached sai khi robot đang di chuyển waypoint kế tiếp.
- Sử dụng wait_for_move đúng timeout theo tổng quãng đường thực sự.
"""

import torch
import math
import time
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from curobo.motion_planner import MotionPlannerCfg, MotionPlanner
from curobo.types import JointState
from curobo_ursim_control import send_urscript, wait_for_move


class SafeReturnV2:
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

    def _build_traj_script(self, waypoints: list, speed: float, accel: float = 1.2) -> str:
        """Xây dựng 1 URScript duy nhất chứa tất cả movej.
        
        Gửi 1 script thay vì nhiều lệnh giúp URSim:
        - Blend mượt mà giữa các waypoint (không dừng ở từng điểm)
        - Không bị timeout giả khi đang di chuyển tới waypoint tiếp theo
        """
        script = "def safe_recovery():\n"
        for i, q in enumerate(waypoints):
            is_last = (i == len(waypoints) - 1)
            r = 0.0 if is_last else 0.05  # blend radius nhỏ cho tất cả trừ điểm cuối
            j = ", ".join(f"{x:.6f}" for x in q)
            script += f"  movej([{j}], a={accel:.2f}, v={speed:.2f}, r={r:.3f})\n"
        script += "end\n"
        return script

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

            # Lọc trùng lặp
            filtered = [traj[0]]
            for pt in traj[1:]:
                if math.sqrt(sum((a - b) ** 2 for a, b in zip(pt, filtered[-1]))) > 0.005:
                    filtered.append(pt)
            if filtered[-1] != traj[-1]:
                filtered.append(traj[-1])

            print(f"[SafeReturn] Đã tìm thấy đường thoát ({len(filtered)} wps). Đang thực thi...")

            # ── GỬI 1 SCRIPT DUY NHẤT thay vì nhiều move_joints riêng lẻ ──
            script = self._build_traj_script(filtered, speed=self.speed)
            send_urscript(script)

            # ── CHỜ ROBOT ĐẾN WAYPOINT CUỐI CÙNG ──
            # Tính timeout theo tổng quãng đường thực (sum of all step distances)
            total_dist = sum(
                max(abs(filtered[i][j] - filtered[i-1][j]) for j in range(6))
                for i in range(1, len(filtered))
            )
            timeout = max(15.0, total_dist / self.speed * 3.0)

            ok = wait_for_move(self.ctrl.rr, self.safe_pose, timeout=timeout, tol=0.05)
            if ok:
                print("[SafeReturn] Đã về safe pose thành công!")
            else:
                print(f"[SafeReturn] TIMEOUT ({timeout:.0f}s) - robot có thể chưa đến nơi!")

        else:
            print("[SafeReturn] Không thể tính toán đường thoát! Dùng fallback...")
            self._fallback_recovery(current_q)

    def _fallback_recovery(self, current_q):
        """Nhích nhẹ từng khớp để thoát vùng kỳ dị, rồi tính lại đường."""
        print("[SafeReturn] Thử chuyển động ngắn để tăng sigma_min...")
        q_tensor = torch.tensor([current_q], device="cuda", dtype=torch.float32)
        best_q = current_q
        best_sigma = self.guard.metrics_calc.get_metrics(q_tensor)["sigma_min"].item()

        for i in range(6):
            for delta in [-0.15, 0.15]:
                q_test = list(current_q)
                q_test[i] += delta
                q_test_t = torch.tensor([q_test], device="cuda", dtype=torch.float32)
                sigma = self.guard.metrics_calc.get_metrics(q_test_t)["sigma_min"].item()
                if sigma > best_sigma:
                    best_sigma = sigma
                    best_q = q_test

        orig_sigma = self.guard.metrics_calc.get_metrics(q_tensor)["sigma_min"].item()
        if best_sigma > orig_sigma:
            print(f"[SafeReturn] Nhích nhẹ khớp để thoát kẹt (sigma_min lên {best_sigma:.5f})")
            # Gửi thẳng 1 movej nhỏ để thoát - không dùng wait_for_move phức tạp
            script = f"movej([{', '.join(f'{x:.6f}' for x in best_q)}], a=0.5, v=0.1, r=0)\n"
            send_urscript(script)
            time.sleep(3.0)  # Đợi robot di chuyển xong bước nhỏ này
            self.execute_recovery(best_q)
        else:
            print("[SafeReturn] CẠN KIỆT GIẢI PHÁP: Không thể thoát khỏi singularity tự động!")
