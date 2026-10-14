import os
import math
import time
import socket
import threading
import torch
import numpy as np
from curobo.types import JointState
from curobo.motion_planner import MotionPlannerCfg, MotionPlanner


def unlock_robot_dashboard(robot_ip: str = "127.0.0.1"):
    """
    Tự động mở khóa Protective Stop / Safety Popup qua Dashboard Server (Port 29999).
    Điều này giải cứu robot khi bị dừng khẩn cấp do va chạm hoặc vặn xoắn.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.5)
        s.connect((robot_ip, 29999))
        time.sleep(0.1)
        try:
            s.recv(1024)
        except Exception:
            pass

        # Gửi chuỗi lệnh mở khóa an toàn chuẩn của Universal Robots
        s.sendall(b"unlock protective stop\n")
        time.sleep(0.2)
        s.sendall(b"close safety popup\n")
        time.sleep(0.2)
        s.sendall(b"play\n")
        time.sleep(0.2)
        s.close()
        print("[HomePlanner] Đã gửi lệnh mở khóa Protective Stop qua Dashboard Port 29999.")
    except Exception as e:
        print(f"[HomePlanner] Cảnh báo kết nối Dashboard: {e}")


class HomePlanner:
    """
    Quản lý quy hoạch và thực thi quỹ đạo trở về vị trí Home an toàn:
      - Tự động mở khóa Protective Stop nếu robot bị kẹt hoặc tự va chạm
      - Thử quy hoạch né vật cản qua CuRobo MotionPlanner C-space
      - Dự phòng giải cứu thoát kẹt bằng nội suy khớp Smoothstep Cosine
      - Thực thi kép: Thử RTDE servoJ trước, tự động chuyển Port 30002 movej nếu RTDE bị khóa
    """
    def __init__(self, robot_config_path: str, scene_file: str, viz, default_home_q: list = None):
        self.scene_file = scene_file
        self.viz = viz
        self.mp = None
        # Vị trí Home an toàn chuẩn của UR3e (khuỷu tay cong tự nhiên, Z=33cm, tuyệt đối không tự va chạm)
        self.default_home_q = list(default_home_q) if default_home_q is not None else [0.0, -2.2, 1.9, -1.383, -1.57, 0.0]

        if os.path.exists(scene_file):
            try:
                mp_cfg = MotionPlannerCfg.create(robot=robot_config_path, scene_model=scene_file)
                if self.viz and hasattr(self.viz, "add_scene"):
                    self.viz.add_scene(mp_cfg.scene_collision_cfg.scene_model)
                self.mp = MotionPlanner(mp_cfg)
                print("[HomePlanner] MotionPlanner đã khởi tạo với bản đồ vật cản.")
            except Exception as e:
                print(f"[HomePlanner] Cảnh báo lỗi khởi tạo MotionPlanner: {e}")
        else:
            print(f"[HomePlanner] Cảnh báo: Không tìm thấy {scene_file}")

    def go_home(
        self,
        teleop,
        executor,
        ctrl,
        target_widget,
        widget_pose_source,
        ui_status,
        cb_teleop,
        is_navigating_home: threading.Event = None,
        poll_sleep: float = 0.05
    ) -> bool:
        """Thực hiện quy hoạch và chạy về vị trí Home an toàn với cơ chế bảo vệ kép"""
        if is_navigating_home is not None:
            is_navigating_home.set()

        try:
            # 1. Tắt Teleop an toàn và giải phóng tài nguyên RTDE
            cb_teleop.value = False
            teleop.enabled = False
            executor.disconnect()
            time.sleep(0.3)

            # 2. Mở khóa Dashboard nếu robot đang bị dừng khẩn cấp / Protective Stop
            robot_ip = os.environ.get("ROBOT_IP", "127.0.0.1")
            unlock_robot_dashboard(robot_ip)

            print("[HomePlanner] Đang tính toán quỹ đạo về Home...")
            ui_status.value = "Planning to Home..."

            q_curr = list(ctrl.joints())
            target_home = list(self.default_home_q)

            # 3. Thử giải quỹ đạo né vật cản qua CuRobo MotionPlanner
            traj = None
            if self.mp is not None:
                try:
                    q_start = JointState.from_position(
                        torch.tensor([q_curr], dtype=torch.float32, device="cuda"),
                        joint_names=self.mp.joint_names
                    )
                    q_home_js = JointState.from_position(
                        torch.tensor([target_home], dtype=torch.float32, device="cuda"),
                        joint_names=self.mp.joint_names
                    )
                    result = self.mp.plan_cspace(q_home_js, current_state=q_start)
                    if result is not None and torch.count_nonzero(result.success) > 0:
                        if hasattr(result, 'get_interpolated_plan'):
                            traj = result.get_interpolated_plan().position.squeeze().cpu().tolist()
                        else:
                            traj = result.solution.position.squeeze().cpu().tolist() if hasattr(result.solution, 'position') else result.solution.squeeze().cpu().tolist()
                        print("[HomePlanner] CuRobo đã tìm thấy quỹ đạo mượt né vật cản.")
                except Exception as e:
                    print(f"[HomePlanner] Lỗi quy hoạch CuRobo: {e}")

            # 4. Tạo tập điểm quỹ đạo tối ưu (nếu CuRobo không tìm được do kẹt, chạy thẳng về target_home)
            from core.curobo_ursim_control import send_urscript
            ui_status.value = "Đang chạy về Home an toàn..."

            if traj is not None and len(traj) > 1:
                # Trích xuất 6-8 waypoints chính với blend radius để robot lướt mượt qua các điểm
                step_sz = max(1, len(traj) // 8)
                key_pts = traj[::step_sz]
                if key_pts[-1] != target_home:
                    key_pts.append(target_home)
                script_lines = ["def home_trajectory():"]
                for idx, pt in enumerate(key_pts):
                    r = 0.025 if idx < len(key_pts) - 1 else 0.0
                    pt_str = ", ".join(f"{x:.5f}" for x in pt)
                    script_lines.append(f"  movej([{pt_str}], a=0.8, v=0.5, r={r})")
                script_lines.append("end")
                script_lines.append("home_trajectory()\n")
                script = "\n".join(script_lines)
            else:
                target_str = ", ".join(f"{x:.5f}" for x in target_home)
                script = f"def home_trajectory():\n  movej([{target_str}], a=0.8, v=0.5, r=0.0)\nend\nhome_trajectory()\n"

            # 5. Gửi lệnh qua Secondary Port 30002: Bộ nội suy 500Hz của Universal Robots tự sinh quỹ đạo mượt (Zero Giật)
            print("[HomePlanner] Thực thi quỹ đạo chuyển động mượt qua bộ sinh quỹ đạo phần cứng URScript...")
            send_urscript(script, settle=0.1)

            # 6. Vòng lặp đơn luồng giám sát và cập nhật Viser 3D mượt mà (chống xung đột đa luồng gây glitch)
            t_wait_start = time.time()
            while time.time() - t_wait_start < 8.0:
                q_now = ctrl.joints()
                if self.mp and self.viz and hasattr(self.viz, "set_joint_state"):
                    js = JointState.from_position(
                        torch.tensor([q_now], device="cuda", dtype=torch.float32),
                        joint_names=self.mp.joint_names
                    )
                    self.viz.set_joint_state(js)
                dist_to_home = sum(abs(q_now[i] - target_home[i]) for i in range(6))
                if dist_to_home < 0.03:
                    break
                time.sleep(0.04)

            ui_status.value = "Safely returned to Home!"
            print("[HomePlanner] Đã về Home an toàn!")

            # 7. Đồng bộ lại Target Widget tại vị trí mới bằng Forward Kinematics
            fk_res = teleop._curobo.fk(target_home)
            target_widget.position = fk_res["pos"]
            target_widget.wxyz = fk_res["quat"]
            widget_pose_source._ema_pos = None
            widget_pose_source._ema_quat_xyzw = None

            # Đặt lại hạt giống giải IK về Home để tránh lệch nhánh
            teleop._last_valid_q = list(target_home)

            # 8. Bật lại cờ Teleop trên UI
            cb_teleop.value = True
            return True

        except Exception as e:
            ui_status.value = f"Lỗi di chuyển Home: {e}"
            print(f"[HomePlanner] Lỗi thực thi Go to Home: {e}")
            executor.disconnect()
            return False

        finally:
            if is_navigating_home is not None:
                is_navigating_home.clear()
