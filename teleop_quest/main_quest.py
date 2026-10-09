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

    from teleop_quest.network_utils import get_local_ip, setup_ssl
    local_ip = get_local_ip()
    base_dir = os.path.dirname(os.path.dirname(__file__))
    print(f"IP hiện tại: {local_ip}")
    cert_path, key_path = setup_ssl(base_dir, local_ip)

    quest_server = QuestServer(cert_path, key_path)
    quest_server.start()

    # 2. Khởi động Controller, Logger & URSim
    print("Đang kết nối CuRobo và URSim...")
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
        
    from teleop_quest.pose_sources import WidgetPoseSource, get_robot_fk_as_ur
    # Nguồn cấp tọa độ từ Kính VR, mỏ neo vào TỌA ĐỘ THỰC TẾ của robot (thông qua CuRobo FK)
    # Scale=1.0 (Tỷ lệ 1:1 giữa tay người và tay máy), alpha=0.1 (nhạy hơn)
    pose_source = DeltaPoseSource(quest_server, ctrl, scale=1.0, anchor_reference_func=lambda: get_robot_fk_as_ur(teleop, ctrl), alpha=0.1)
    
    widget_pose_source = WidgetPoseSource(target_widget)
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
        cb_teleop = server.gui.add_checkbox("VR Teleop", initial_value=False)
        cb_record = server.gui.add_checkbox("Record AI Data", initial_value=False)
        btn_home = server.gui.add_button("Go to Home")
        ui_status = server.gui.add_text("Status", initial_value="Waiting...", disabled=True)
        ui_conn = server.gui.add_text("VR Connected", initial_value="No", disabled=True)
        
    @cb_record.on_update
    def _on_record_update(_):
        logger.is_recording = cb_record.value
        if cb_record.value:
            print("BẮT ĐẦU thu thập dữ liệu!")
        else:
            logger.flush()
            print("ĐÃ DỪNG thu thập dữ liệu.")
        
    def _on_btn_home_click(_):
        nonlocal _last_teleop_state
        if not mp:
            ui_status.value = "Error: Motion Planner not found!"
            return
            
        if cb_teleop.value:
            # Tự động TẮT Teleop an toàn trước khi chạy lệnh Home
            cb_teleop.value = False
            teleop.enabled = False
            _last_teleop_state = False
            executor.disconnect()
            time.sleep(0.5) # Đợi nhả điều khiển RTDE
            
        print("[System] Calculating Home trajectory...")
        ui_status.value = "Planning to Home..."
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
            ui_status.value = "Safely returned to Home!"
            print("Safely returned to Home!")
            
            # Đồng bộ Widget với vị trí mới (bằng FK)
            fk_res = teleop._curobo.fk(traj[-1])
            target_widget.position = fk_res["pos"]
            target_widget.wxyz = fk_res["quat"]
            widget_pose_source._ema_pos = None
            widget_pose_source._ema_quat_xyzw = None
            
            # Tự động BẬT LẠI Teleop để sếp dùng luôn
            cb_teleop.value = True
        else:
            ui_status.value = "Lỗi: Không tìm được đường về Home!"
            print("[Lỗi] Không tìm được đường về Home an toàn.")
            
    btn_home.on_click(_on_btn_home_click)

    def handle_sigint(sig, frame):
        print("[Hệ thống] Đang tắt an toàn...")
        teleop.enabled = False
        time.sleep(0.5)
        ctrl.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sigint)


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
            print("Đã nhận lệnh E-STOP từ kính VR! Đã ngắt động cơ.")
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
            try:
                q_real = ctrl.joints()
                
                # 1. Tính toán khoảng cách (chỉ để in Log nếu cần)
                import numpy as np
                fk_res = teleop._curobo.fk(q_real)
                
                if not hasattr(pose_source, "out_of_reach_time"):
                    pose_source.out_of_reach_time = 0.0
                
                # Chạy luồng Teleoperation gốc chuẩn (Widget -> IK -> Robot)
                # Đưa teleop.step LÊN TRƯỚC để lấy status (Thành công hay IK Thất bại)
                teleop.step(q_real)
                current_status = teleop.get_status()
                status_low = current_status.lower()
                
                # Làm gọn và đồng bộ VR Status
                if "thất bại" in status_low or "fail" in status_low:
                    ui_status.value = "IK Error"
                elif "ngoai tam voi" in status_low or "ngoài tầm với" in status_low or "vượt giới hạn" in status_low or "out of reach" in status_low:
                    ui_status.value = "Out of Reach"
                elif "singularity" in status_low or "kì dị" in status_low:
                    ui_status.value = "Singularity"
                elif "sigma=" in status_low or "curobo" in status_low:
                    ui_status.value = "Tracking"
                else:
                    ui_status.value = current_status.split("|")[0].strip()
                
                # 2. Đọc tín hiệu từ kính VR
                vr_pose = pose_source.get_pose()
                if vr_pose is not None:
                    # Nếu đang bóp cò VR, cập nhật tọa độ Widget 3D
                    target_widget.position = vr_pose["pos"]
                    target_widget.wxyz = vr_pose["quat"]
                    pose_source.out_of_reach_time = 0.0 # Đang bóp cò thì reset bộ đếm
                else:
                    # NẾU SẾP ĐANG THẢ CÒ (Dùng chuột hoặc đứng im)
                    # Dùng trực tiếp Trạng thái để biết có ngoài tầm với hay không
                    status_low = current_status.lower()
                    is_out_of_reach = "ngoai tam voi" in status_low or "dung vat can" in status_low or "tu the ket" in status_low or "singularity" in status_low
                    
                    if is_out_of_reach:
                        pose_source.out_of_reach_time += POLL_SLEEP
                        
                    else:
                        pose_source.out_of_reach_time = 0.0
                        
                    if pose_source.out_of_reach_time > 2.0:
                        dist = np.linalg.norm(np.array(fk_res["pos"]) - np.array(target_widget.position))
                        print(f"[Auto-Snap] Trục tọa độ bị kẹt ngoài tầm với ({dist*100:.1f}cm). Tự động thu hồi!")
                        target_widget.position = fk_res["pos"]
                        target_widget.wxyz = fk_res["quat"]
                        
                        # Reset bộ lọc EMA
                        widget_pose_source._ema_pos = None
                        widget_pose_source._ema_quat_xyzw = None
                        pose_source.out_of_reach_time = 0.0
                
                # Update robot thật tren Viser
                js = JointState.from_position(
                    torch.tensor([q_real], device="cuda", dtype=torch.float32),
                    joint_names=['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                )
                viz.set_joint_state(js)
                
                # Nếu IK fail hoặc vướng Singularity -> Rung tay cầm (VR)
                if "thất bại" in current_status.lower() or "cảnh báo" in current_status.lower() or "singularity" in current_status.lower():
                    quest_server.send_haptic(intensity=1.0, duration=150)
                    
                # Log AI Data
                if logger.is_recording:
                    logger.log_step(q_real, ctrl.rr.getActualQd(), ctrl.tcp(), image_path="cam_frame_placeholder.jpg")
                    
            except Exception as e:
                ui_status.value = f"Lỗi: {e}"
                print(f"[Lỗi Điều khiển] {e}")
                traceback.print_exc()
        else:
            ui_status.value = "Tắt"

        time.sleep(POLL_SLEEP)

if __name__ == "__main__":
    main()
