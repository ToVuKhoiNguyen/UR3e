import os
import time
import torch
from curobo.types import JointState
from curobo.motion_planner import MotionPlannerCfg, MotionPlanner

class HomePlanner:
    """
    Quản lý quy hoạch và thực thi quỹ đạo trở về vị trí Home an toàn
    (sử dụng CuRobo MotionPlanner trong không gian khớp C-space có né vật cản).
    """
    def __init__(self, robot_config_path: str, scene_file: str, viz):
        self.scene_file = scene_file
        self.viz = viz
        self.mp = None
        
        if os.path.exists(scene_file):
            mp_cfg = MotionPlannerCfg.create(robot=robot_config_path, scene_model=scene_file)
            self.viz.add_scene(mp_cfg.scene_collision_cfg.scene_model)
            self.mp = MotionPlanner(mp_cfg)
            print("[HomePlanner] MotionPlanner đã khởi tạo với bản đồ vật cản.")
        else:
            print(f"[HomePlanner] Cảnh báo: Không tìm thấy {scene_file}")

    def go_home(self, teleop, executor, ctrl, target_widget, widget_pose_source, ui_status, cb_teleop, poll_sleep=0.05) -> bool:
        """Thực hiện quy hoạch và chạy về vị trí Home an toàn"""
        if not self.mp:
            ui_status.value = "Lỗi: Chưa khởi tạo Motion Planner!"
            return False

        if cb_teleop.value:
            # Tự động TẮT Teleop an toàn trước khi chạy lệnh Home
            cb_teleop.value = False
            teleop.enabled = False
            executor.disconnect()
            time.sleep(0.5)

        print("[HomePlanner] Đang tính toán quỹ đạo về Home...")
        ui_status.value = "Planning to Home..."

        q_start = JointState.from_position(
            torch.tensor([ctrl.joints()], dtype=torch.float32, device="cuda"),
            joint_names=self.mp.joint_names
        )
        # q_home: Xoay base 90 độ (1.5708 rad) để tránh kẹt vào vách bàn
        q_home = JointState.from_position(
            torch.tensor([[1.5708, -1.5708, 1.5708, -1.5708, -1.5708, -1.5708]], dtype=torch.float32, device="cuda"),
            joint_names=self.mp.joint_names
        )

        result = self.mp.plan_cspace(q_home, current_state=q_start)
        if result is not None and result.success.any():
            if hasattr(result, 'get_interpolated_plan'):
                traj = result.get_interpolated_plan().position.squeeze().cpu().tolist()
            else:
                traj = result.solution.position.squeeze().cpu().tolist() if hasattr(result.solution, 'position') else result.solution.squeeze().cpu().tolist()

            ui_status.value = "Đang chạy về Home an toàn..."
            executor.connect()
            time.sleep(1.0) # Đợi script RTDE khởi động trên robot

            for q in traj:
                executor._ctrl.servoJ(q, 0.0, 0.0, poll_sleep, 0.1, 300)
                js = JointState.from_position(torch.tensor([q], device="cuda", dtype=torch.float32), joint_names=self.mp.joint_names)
                self.viz.set_joint_state(js)
                time.sleep(poll_sleep)

            executor.disconnect()
            ui_status.value = "Safely returned to Home!"
            print("[HomePlanner] Đã về Home an toàn!")

            # Đồng bộ Widget với vị trí mới bằng Forward Kinematics
            fk_res = teleop._curobo.fk(traj[-1])
            target_widget.position = fk_res["pos"]
            target_widget.wxyz = fk_res["quat"]
            widget_pose_source._ema_pos = None
            widget_pose_source._ema_quat_xyzw = None

            # Tự động bật lại Teleop
            cb_teleop.value = True
            return True
        else:
            ui_status.value = "Lỗi: Không tìm được đường về Home!"
            print("[HomePlanner] Không tìm được đường về Home an toàn.")
            return False
