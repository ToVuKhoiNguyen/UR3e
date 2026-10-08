#!/usr/bin/env python3
import os
import sys
import time
import argparse
import signal
import yaml
import traceback
import numpy as np

from core.curobo_ursim_control import CuRoboURSim
from core.data_logger import DataLogger
from teleop.teleoperation import (
    PyRokiIKSolver, CuRoboIKSolver,
    SafetyGate, ServoJExecutor, TeleoperationController
)
from core.data_logger import DataLogger

from teleop_quest.quest_server import QuestServer
from teleop_quest.delta_pose_source import DeltaPoseSource
from teleop_quest.timeout_watchdog import TimeoutWatchdog

import viser
import torch
import subprocess
from curobo.viewer import ViserVisualizer
from curobo.types import ContentPath, JointState

POLL_SLEEP = 0.05

def load_config():
    cfg_path = os.path.join(os.path.dirname(__file__), "..", "config", "singularity.yaml")
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    print("="*60)
    print(" KHỞI ĐỘNG HỆ THỐNG VR TELEOP (META QUEST) ")
    print("="*60)

    # 1. Khởi động WebXR Server & Cập nhật IP/SSL
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = '127.0.0.1'
    finally:
        s.close()

    base_dir = os.path.dirname(os.path.dirname(__file__))
    cert_path = os.path.join(base_dir, "certs", "cert.pem")
    key_path = os.path.join(base_dir, "certs", "key.pem")
    
    print(f"[Hệ thống] IP hiện tại: {local_ip}")
    os.makedirs(os.path.join(base_dir, "certs"), exist_ok=True)
    
    ip_cache_path = os.path.join(base_dir, "certs", "ip_cache.txt")
    cached_ip = ""
    if os.path.exists(ip_cache_path):
        with open(ip_cache_path, "r") as f:
            cached_ip = f.read().strip()

    if not os.path.exists(cert_path) or cached_ip != local_ip:
        print(f"[Hệ thống] IP thay đổi (hoặc chạy lần đầu). Đang tự động tạo SSL cho IP: {local_ip}...")
        subprocess.run(
            f"openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout {key_path} -out {cert_path} -subj '/CN={local_ip}'", 
            shell=True, stderr=subprocess.DEVNULL
        )
        with open(ip_cache_path, "w") as f:
            f.write(local_ip)

    quest_server = QuestServer(cert_path, key_path)
    quest_server.start()

    # 2. Khởi động Controller, Logger & URSim
    print("[Hệ thống] Đang kết nối CuRobo và URSim...")
    scene_file = "/home/nguyen/ur_ws/curobo_ursim/assets/obstacle_scene.yml"
    ctrl = CuRoboURSim(scene_file=scene_file)
    import numpy as np
    import time
    for _ in range(10):
        if np.linalg.norm(ctrl.joints()) > 0.01:
            break
        time.sleep(0.1)
    START_JOINTS = ctrl.joints()
    config = load_config()
    logger = DataLogger(config)

    # 3. Viser UI (Hiển thị robot ảo)
    subprocess.run("fuser -k 8080/tcp 2>/dev/null", shell=True)
    time.sleep(0.5)

    viz = ViserVisualizer(
        content_path=ContentPath(robot_config_file="/home/nguyen/ur_ws/curobo_ursim/assets/ur3e_custom.yml"),
        connect_ip="0.0.0.0",
        connect_port=8080,
        add_control_frames=False,
    )
    server = viz._server
    
    # Đồng bộ Viser với trạng thái thực tế của URSim
    from curobo.types import JointState
    import torch
    js_init = JointState.from_position(torch.tensor([START_JOINTS], device="cuda", dtype=torch.float32), joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
    viz.set_joint_state(js_init)
    
    # 3.5 Setup Teleoperation (Sử dụng lại chuẩn logic từ teleoperation.py)
    from teleop.teleoperation import TeleoperationController, PyRokiIKSolver, CuRoboIKSolver, SafetyGate, ServoJExecutor
    import transforms3d
    
    # Tạo Widget 3D trên màn hình Viser
    # Khởi tạo widget tại vị trí hiện tại của robot để tránh bị rớt xuống (0,0,0)
    init_tcp = ctrl.tcp()
    init_pos = init_tcp[:3]
    import transforms3d
    axis = np.array(init_tcp[3:])
    angle = np.linalg.norm(axis)
    if angle > 1e-6:
        axis = axis / angle
        init_quat = transforms3d.quaternions.axangle2quat(axis, angle)
    else:
        init_quat = [1.0, 0.0, 0.0, 0.0]
        
    target_widget = server.scene.add_transform_controls(
        "target_pose", 
        depth_test=False, 
        position=init_pos, 
        wxyz=init_quat,
        scale=0.18,
        disable_sliders=False
    )
    
    # Hàm chuyển đổi tọa độ Widget sang dạng UR (x, y, z, rx, ry, rz)
    def get_widget_pose_as_ur():
        pos = target_widget.position
        quat_wxyz = target_widget.wxyz
        axis, angle = transforms3d.quaternions.quat2axangle(quat_wxyz)
        return [pos[0], pos[1], pos[2], axis[0]*angle, axis[1]*angle, axis[2]*angle]
        
    # Nguồn cấp tọa độ từ Kính VR, lấy Widget làm điểm neo (Anchor)
    pose_source = DeltaPoseSource(quest_server, ctrl, scale=1.0, anchor_reference_func=get_widget_pose_as_ur)
    
    # Nguồn cấp tọa độ cho Controller (Đọc từ Widget)
    class WidgetPoseSource:
        def __init__(self, alpha=0.15):
            self.alpha = alpha
            self._ema_pos = None
            self._ema_quat_xyzw = None
            
        def get_pose(self):
            import numpy as np
            target_pos = target_widget.position
            target_quat_wxyz = target_widget.wxyz
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
            
    widget_pose_source = WidgetPoseSource()
    from safety.singularity_guard.guard_core import SingularityGuard
    guard = SingularityGuard(config)
    
    # Khởi tạo Controller gốc chuẩn (có SafetyGate chống giật)
    executor = ServoJExecutor(dt=POLL_SLEEP)
    teleop = TeleoperationController(
        pose_source=widget_pose_source,
        pyroki_solver=None,
        curobo_solver=CuRoboIKSolver(ctrl.ik_solver),
        safety_gate=SafetyGate(guard=guard),
        executor=executor
    )
    
    # Ép Widget bám sát tuyệt đối vào đầu robot (bỏ qua sai lệch Calibration của URSim thật)
    fk_init = teleop._curobo.fk(START_JOINTS)
    target_widget.position = fk_init["pos"]
    target_widget.wxyz = fk_init["quat"]
    
    # Reset EMA filter
    widget_pose_source._ema_pos = None
    widget_pose_source._ema_quat_xyzw = None

    # 4. Watchdog
    watchdog = TimeoutWatchdog(quest_server, executor)
    watchdog.start()



    # Thêm bàn (vật cản) vào môi trường và khởi tạo Motion Planner
    from curobo.motion_planner import MotionPlannerCfg, MotionPlanner
    scene_file = "/home/nguyen/ur_ws/curobo_ursim/assets/obstacle_scene.yml"
    mp = None
    if os.path.exists(scene_file):
        mp_cfg = MotionPlannerCfg.create(robot="/home/nguyen/ur_ws/curobo_ursim/assets/ur3e_custom.yml", scene_model=scene_file)
        viz.add_scene(mp_cfg.scene_collision_cfg.scene_model)
        mp = MotionPlanner(mp_cfg)
    
    with server.gui.add_folder("VR Teleoperation"):
        cb_teleop = server.gui.add_checkbox("Bat VR Teleop", initial_value=False)
        cb_record = server.gui.add_checkbox("Record AI Data", initial_value=False)
        sld_speed = server.gui.add_slider("Toc do (Scale)", min=0.1, max=2.0, step=0.1, initial_value=1.0)
        btn_home = server.gui.add_button("Reset to Home")
        ui_status = server.gui.add_text("VR Status", initial_value="Chờ...", disabled=True)
        ui_conn = server.gui.add_text("Quest Connected", initial_value="No", disabled=True)
        
    @sld_speed.on_update
    def _on_speed_update(_):
        pose_source.scale = sld_speed.value
        print(f"[Cài đặt] Đã chỉnh tốc độ Robot xuống còn {sld_speed.value}x")
        
    @cb_record.on_update
    def _on_record_update(_):
        logger.is_recording = cb_record.value
        if cb_record.value:
            print("[AI Logger] BẮT ĐẦU thu thập dữ liệu!")
        else:
            logger.flush()
            print("[AI Logger] ĐÃ DỪNG thu thập dữ liệu.")
        
    def _on_btn_home_click(_):
        if not mp:
            ui_status.value = "Lỗi: Không tìm thấy Motion Planner!"
            return
        if cb_teleop.value:
            ui_status.value = "Vui lòng TẮT VR Teleop trước khi về Home!"
            return
            
        print("[Hệ thống] Đang tính toán đường về Home...")
        ui_status.value = "Đang quy hoạch về Home..."
        q_start = JointState.from_position(torch.tensor([ctrl.joints()], dtype=torch.float32, device="cuda"), joint_names=mp.joint_names)
        # Sửa q_home xoay base 90 độ (1.5708) để tránh kẹt vào bức tường ở X=0.25
        q_home = JointState.from_position(torch.tensor([[1.5708, -1.5708, 1.5708, -1.5708, -1.5708, -1.5708]], dtype=torch.float32, device="cuda"), joint_names=mp.joint_names)
        
        result = mp.plan_cspace(q_home, current_state=q_start)
        if result is not None and result.success.any():
            if hasattr(result, 'get_interpolated_plan'):
                traj = result.get_interpolated_plan().position.squeeze().cpu().tolist()
            else:
                traj = result.solution.position.squeeze().cpu().tolist() if hasattr(result.solution, 'position') else result.solution.squeeze().cpu().tolist()
            ui_status.value = "Đang chạy về Home an toàn..."
            executor.connect()
            time.sleep(1.0) # Đợi script RTDE khởi động trên robot
            for q in traj:
                executor._ctrl.servoJ(q, 0.0, 0.0, POLL_SLEEP, 0.1, 300)
                
                # Cập nhật Viser
                js = JointState.from_position(torch.tensor([q], device="cuda", dtype=torch.float32), joint_names=mp.joint_names)
                viz.set_joint_state(js)
                
                time.sleep(POLL_SLEEP)
            executor.disconnect()
            ui_status.value = "Đã về Home an toàn!"
            print("[Hệ thống] Đã về Home an toàn.")
            
            # Đồng bộ Widget với vị trí mới (bằng FK)
            fk_res = teleop._curobo.fk(traj[-1])
            target_widget.position = fk_res["pos"]
            target_widget.wxyz = fk_res["quat"]
            widget_pose_source._ema_pos = None
            widget_pose_source._ema_quat_xyzw = None
        else:
            ui_status.value = "Lỗi: Không tìm được đường về Home!"
            print("[Lỗi] Không tìm được đường về Home an toàn.")
            
    btn_home.on_click(_on_btn_home_click)

    def handle_sigint(sig, frame):
        print("\n[Hệ thống] Đang tắt an toàn...")
        teleop.enabled = False
        time.sleep(0.5)
        ctrl.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sigint)

    # ==== TỰ ĐỘNG CHẠY VỀ HOME LÚC KHỞI ĐỘNG ====
    if mp is not None:
        print("[Hệ thống] Tự động quy hoạch về Home lúc khởi động...")
        q_start = JointState.from_position(torch.tensor([START_JOINTS], dtype=torch.float32, device="cuda"), joint_names=mp.joint_names)
        # Sửa q_home xoay base 90 độ để tránh kẹt tường, và xoay cổ tay (wrist 3) -90 độ để trục X (Đỏ) chỉa thẳng tới trước
        q_home = JointState.from_position(torch.tensor([[1.5708, -1.5708, 1.5708, -1.5708, -1.5708, -1.5708]], dtype=torch.float32, device="cuda"), joint_names=mp.joint_names)
        res = mp.plan_cspace(q_home, current_state=q_start)
        if res is not None and res.success.any():
            if hasattr(res, 'get_interpolated_plan'):
                traj = res.get_interpolated_plan().position.squeeze().cpu().tolist()
            else:
                traj = res.solution.position.squeeze().cpu().tolist() if hasattr(res.solution, 'position') else res.solution.squeeze().cpu().tolist()
            executor.connect()
            time.sleep(1.0) # Đợi script RTDE khởi động trên robot
            for q in traj:
                executor._ctrl.servoJ(q, 0.0, 0.0, POLL_SLEEP, 0.1, 300)
                js = JointState.from_position(torch.tensor([q], device="cuda", dtype=torch.float32), joint_names=mp.joint_names)
                viz.set_joint_state(js)
                time.sleep(POLL_SLEEP)
            executor.disconnect()
            print("[Hệ thống] Đã khởi động an toàn tại Home.")
            
            # Cập nhật lại Widget cho khớp với Home bằng Forward Kinematics (bỏ qua độ trễ RTDE)
            fk_res = teleop._curobo.fk(traj[-1])
            target_widget.position = fk_res["pos"]
            target_widget.wxyz = fk_res["quat"]
            
            # Reset luôn EMA filter
            widget_pose_source._ema_pos = None
            widget_pose_source._ema_quat_xyzw = None

    # Vòng lặp chính (20Hz)
    print("\n" + "="*30)
    print(f"ĐỊA CHỈ IP HIỆN TẠI CỦA MÁY TÍNH LÀ: {local_ip}")
    print("="*30)
    
    print("\n[LIÊN KẾT NHANH DÀNH CHO MÁY TÍNH]")
    print(f"-> Viser Dashboard (Giao diện điều khiển): http://localhost:8080")
    print(f"-> URSim Web (Màn hình Teach Pendant): http://192.168.56.101:6080/vnc.html")
    
    print("\n[LIÊN KẾT DÀNH CHO KÍNH VR QUEST 3S]")
    print(f"1. Thông chốt SSL (Duyệt mạng): https://{local_ip}:{quest_server.ws_port}")
    print(f"2. Vào Buồng lái Ảo (VR Cockpit): https://{local_ip}:{quest_server.http_port}")
    print("="*30 + "\n")
    
    _last_teleop_state = False
    
    while True:
        # 0. Kiểm tra VR E-Stop
        if quest_server.check_and_clear_estop():
            teleop.enabled = False
            cb_teleop.value = False
            try:
                executor._ctrl.stopJ(2.0)
            except: pass
            ui_status.value = "EMERGENCY STOP (Từ VR)!"
            print("\n[CẢNH BÁO] Đã nhận lệnh E-STOP từ kính VR! Đã ngắt động cơ.")
            quest_server.send_haptic(1.0, 1000) # Rung tay cầm 1 giây
            
        # Cập nhật kết nối UI
        ui_conn.value = "Yes" if quest_server.is_connected() else "No"
        
        # Xử lý bật tắt Teleop
        if cb_teleop.value != _last_teleop_state:
            teleop.enabled = cb_teleop.value
            _last_teleop_state = cb_teleop.value
            
            if teleop.enabled:
                print("[VR] KÍCH HOẠT ĐIỀU KHIỂN.")
                executor.connect()
                watchdog.arm()
            else:
                print("[VR] HỦY KÍCH HOẠT ĐIỀU KHIỂN.")
                executor.disconnect()
                watchdog.disarm()

        if teleop.enabled:
            # 1. Đọc tín hiệu từ kính VR
            vr_pose = pose_source.get_pose()
            if vr_pose is not None:
                # Nếu sếp đang bóp cò, cập nhật tọa độ Widget 3D theo tay sếp
                target_widget.position = vr_pose["pos"]
                target_widget.wxyz = vr_pose["quat"]
            
            # Nếu sếp không bóp cò, Widget 3D sẽ đứng im chờ đợi, 
            # Controller lấy tọa độ từ Widget để giải IK (tracking the widget).

            try:
                q_real = ctrl.joints()
                
                # Update robot thật tren Viser
                js = JointState.from_position(
                    torch.tensor([q_real], device="cuda", dtype=torch.float32),
                    joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                )
                viz.set_joint_state(js)
                
                # Chạy luồng Teleoperation gốc chuẩn (Widget -> IK -> Robot)
                teleop.step(q_real)
                
                # Lấy trạng thái từ SafetyGate và CuRobo
                current_status = teleop.get_status()
                ui_status.value = current_status
                
                # Nếu IK fail hoặc vướng Singularity -> Rung tay cầm
                if "thất bại" in current_status.lower() or "cảnh báo" in current_status.lower() or "singularity" in current_status.lower():
                    quest_server.send_haptic(intensity=1.0, duration=150)
                
                # Log AI Data
                if logger.is_recording:
                    logger.log_step(q_real, ctrl.joint_velocities(), ctrl.pose(), image_path="cam_frame_placeholder.jpg")
                    
            except Exception as e:
                ui_status.value = f"Lỗi: {e}"
                print(f"[Lỗi Điều khiển] {e}")
                traceback.print_exc()
        else:
            ui_status.value = "Tắt"

        time.sleep(POLL_SLEEP)

if __name__ == "__main__":
    main()
