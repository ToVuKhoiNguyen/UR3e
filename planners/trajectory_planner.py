import time
import torch
import numpy as onp
from curobo.types import JointState
from core.curobo_ursim_control import GoalToolPose
import planners.pyroki_mp as pyroki_mp

class TrajectoryPlanner:
    def __init__(self, curobo_mp, pyroki_mp_module, guard, logger):
        """
        curobo_mp: Đối tượng MotionPlanner của CuRobo
        pyroki_mp_module: Thư viện/module pyroki_mp
        guard: SingularityGuard
        logger: DataLogger
        """
        self.mp = curobo_mp
        self.pyroki_mp = pyroki_mp_module
        self.guard = guard
        self.logger = logger

    def plan_trajectory(self, solver_type, start_q, goal_pos, goal_quat, 
                        pk_robot=None, world_coll=None, robot_coll=None, target_link_name="tool0"):
        """
        Quy hoạch quỹ đạo từ góc khớp start_q đến tọa độ mục tiêu (goal_pos, goal_quat)
        Trả về: (thành_công, danh_sách_waypoints, thông_báo)
        """
        # CuRobo
        if solver_type == "CuRobo":
            t_start = time.perf_counter()
            q_start = JointState.from_position(
                torch.tensor([start_q], dtype=torch.float32, device="cuda"),
                joint_names=self.mp.joint_names
            )
            g_pos = torch.tensor([[[[[goal_pos[0], goal_pos[1], goal_pos[2]]]]]], dtype=torch.float32, device="cuda")
            g_quat = torch.tensor([[[[[goal_quat[0], goal_quat[1], goal_quat[2], goal_quat[3]]]]]], dtype=torch.float32, device="cuda")
            goal = GoalToolPose(tool_frames=["tool0"], position=g_pos, quaternion=g_quat)

            result = self.mp.plan_pose(goal, q_start)
            t_end = time.perf_counter()
            plan_time = (t_end - t_start) * 1000

            if result is not None and result.success.any():
                if hasattr(result, 'interpolated_plan') and result.interpolated_plan is not None:
                    traj = result.interpolated_plan.squeeze().cpu().tolist()
                else:
                    traj = result.solution.squeeze().cpu().tolist()
                return True, traj, f"CuRobo tìm đường mất {plan_time:.2f} ms"
            else:
                return False, [], "CuRobo: Lỗi, Không thể tìm đường hoặc vướng vật cản!"

        # PyRoki
        elif solver_type == "PyRoki":
            if not all([pk_robot, world_coll, robot_coll]):
                return False, [], "PyRoki thiếu thông tin môi trường!"
            
            t_start = time.perf_counter()
            # Dùng số điểm từ 10 đến 50 tùy khoảng cách
            dist = sum(abs(g - s) for g, s in zip(goal_pos, [0,0,0])) # Tạm thời bỏ qua init_pos, có thể tinh chỉnh sau
            timesteps = 30 
            
            sol_traj, _, _ = self.pyroki_mp.solve_online_planning(
                robot=pk_robot,
                robot_coll=robot_coll,
                world_coll=world_coll,
                target_link_name=target_link_name,
                target_position=onp.array(goal_pos),
                target_wxyz=onp.array(goal_quat),
                timesteps=timesteps,
                dt=0.05,
                start_cfg=onp.array(start_q),
                prev_sols=onp.array([start_q]),
            )
            t_end = time.perf_counter()
            plan_time = (t_end - t_start) * 1000
            
            if sol_traj is not None and len(sol_traj) > 0:
                traj = sol_traj.tolist()
                return True, traj, f"PyRoki tìm đường mất {plan_time:.2f} ms"
            else:
                return False, [], "PyRoki: Lỗi, Không thể tìm đường!"

        return False, [], "Unknown solver type"

    def filter_and_log_trajectory(self, traj, q_start):
        """
        Kiểm tra vặn xoắn và tính an toàn của quỹ đạo, sau đó ghi log
        """
        clamped_traj = [q_start]
        for tq in traj:
            last_q = clamped_traj[-1]
            if max(abs(tq[i] - last_q[i]) for i in range(6)) > 0.4:
                print(f"[Guard] Cắt bỏ waypoint do nhảy khớp > 0.4 rad!")
                continue
            clamped_traj.append(tq)
            
        if len(clamped_traj) <= 1:
            return False, [], "Từ chối: Quỹ đạo bị vặn xoắn hoàn toàn!"

        # Kiểm tra gập cổ tay sâu
        for tq in clamped_traj:
            if tq[3] < -2.9 or tq[3] > -0.2:
                return False, [], "Từ chối: Quá gập, dễ kẹp nách!"

        # Đánh giá độ an toàn
        min_sigma_traj = 1.0
        for tq in clamped_traj:
            m = self.guard.metrics_calc.get_metrics(torch.tensor([tq], device=self.guard.metrics_calc.device))
            min_sigma_traj = min(min_sigma_traj, m["sigma_min"].item())

        warning = ""
        if min_sigma_traj < self.guard.danger_threshold:
            warning = f"CẢNH BÁO: Xuyên qua kỳ dị (min_sigma = {min_sigma_traj:.4f})"
        elif min_sigma_traj < self.guard.warn_threshold:
            warning = f"Đi sát kỳ dị (min_sigma = {min_sigma_traj:.4f})"
        else:
            warning = "An toàn"

        # Ghi log
        for q in clamped_traj:
            m = self.guard.metrics_calc.get_metrics(torch.tensor([q], device=self.guard.metrics_calc.device))
            self.logger.log_step(q, None, None, None,
                            m["sigma_min"].item(), m["manipulability"].item(),
                            m["condition_number"].item(), "none", None, "OK", "mp_execute")
        self.logger.flush()

        return True, clamped_traj, warning
