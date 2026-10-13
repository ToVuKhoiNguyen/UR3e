#!/usr/bin/env python3
"""
Chương trình chính điều khiển Robot UR3e qua kính Meta Quest 3S (WebXR)
Đã mô-đun hóa:
  - quest_server: WebSocket & HTTP Server cho Quest
  - viser_ui: Giao diện trực quan 3D & nút bấm Viser
  - home_planner: Quy hoạch chuyển động về vị trí an toàn
  - delta_pose_source: Bộ xử lý biến thiên vị trí & xoay từ VR
  - data_logger: Bộ thu thập dữ liệu chuyên gia chuẩn HDF5 (ACT / Diffusion Policy)
"""

import os
import sys
import time
import signal
import yaml
import traceback
import numpy as np

from core.curobo_ursim_control import CuRoboURSim
from core.data_logger import DataLogger
from teleop.teleoperation import (
    CuRoboIKSolver, SafetyGate, ServoJExecutor, TeleoperationController
)
from safety.singularity_guard.guard_core import SingularityGuard
from teleop_quest.quest_server import QuestServer
from teleop_quest.network_utils import get_local_ip, setup_ssl
from teleop_quest.delta_pose_source import DeltaPoseSource
from teleop_quest.pose_sources import WidgetPoseSource, get_robot_fk_as_ur
from teleop_quest.timeout_watchdog import TimeoutWatchdog
from teleop_quest.viser_ui import TeleopViserUI
from teleop_quest.home_planner import HomePlanner
from teleop_quest.camera_worker import CameraWorker
from teleop_quest.grasp_simulator import GraspSimulator
from teleop_quest.synthetic_camera import SyntheticWristCamera

POLL_SLEEP = 0.05
ROBOT_CONFIG = "/home/nguyen/ur_ws/curobo_ursim/assets/ur3e_custom.yml"
SCENE_CONFIG = "/home/nguyen/ur_ws/curobo_ursim/assets/obstacle_scene.yml"

def load_config():
    cfg_path = os.path.join(os.path.dirname(__file__), "..", "config", "singularity.yaml")
    with open(cfg_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    print("=" * 60)
    print(" KHỞI ĐỘNG HỆ THỐNG VR TELEOP (META QUEST 3S) ")
    print("=" * 60)

    # 1. Khởi động mạng và máy chủ WebXR
    local_ip = get_local_ip()
    base_dir = os.path.dirname(os.path.dirname(__file__))
    cert_path, key_path = setup_ssl(base_dir, local_ip)
    quest_server = QuestServer(cert_path, key_path)
    quest_server.start()

    # Khởi động Camera Worker (30 FPS ngầm)
    cam = CameraWorker(camera_id=0, name="wrist_camera")
    cam.start()

    # 2. Khởi động Controller Robot, An toàn & Data Logger
    print("[Hệ thống] Đang kết nối CuRobo và URSim...")
    ctrl = CuRoboURSim(scene_file=SCENE_CONFIG)
    for _ in range(10):
        if np.linalg.norm(ctrl.joints()) > 0.01:
            break
        time.sleep(0.1)
    start_joints = ctrl.joints()
    
    config = load_config()
    logger = DataLogger(config)
    guard = SingularityGuard(config)

    # 3. Tính toán vị trí ban đầu của robot qua CuRobo FK (chuẩn Viser world)
    curobo_solver = CuRoboIKSolver(ctrl.ik_solver)
    fk_init = curobo_solver.fk(start_joints)
    robot_init_pos = fk_init["pos"]
    robot_init_quat = fk_init["quat"]

    # Đặt quả cà chua ban đầu ngay phía dưới/trước đầu gắp robot trên mặt bàn
    pick_pos = (robot_init_pos[0], robot_init_pos[1], 0.035)
    place_pos = (robot_init_pos[0] + 0.20, robot_init_pos[1] - 0.20, 0.02)

    # Khởi tạo Giao diện Viser 3D
    ui = TeleopViserUI(
        ROBOT_CONFIG,
        init_pos=robot_init_pos,
        init_quat=robot_init_quat,
        init_joints=start_joints,
        init_tomato_pos=(pick_pos[0], pick_pos[1])
    )
    ui.set_episodes(logger.current_episode_idx)

    # Khởi tạo mô phỏng gắp nhặt Cà chua 3D & Camera ảo Wrist
    grasp_sim = GraspSimulator(ui.server, pick_pos=pick_pos, place_pos=place_pos)
    wrist_cam = SyntheticWristCamera(width=640, height=480)

    # 4. Thiết lập nguồn tọa độ VR & Pipeline Teleop
    widget_pose_source = WidgetPoseSource(ui.target_widget)
    executor = ServoJExecutor(dt=POLL_SLEEP)
    teleop = TeleoperationController(
        pose_source=widget_pose_source,
        pyroki_solver=None,
        curobo_solver=curobo_solver,
        safety_gate=SafetyGate(guard=guard),
        executor=executor
    )

    pose_source = DeltaPoseSource(
        quest_server, ctrl, scale=1.0,
        anchor_reference_func=lambda: get_robot_fk_as_ur(teleop, ctrl),
        alpha=0.1
    )

    # 5. Khởi tạo bộ giám sát mất kết nối & Home Planner
    watchdog = TimeoutWatchdog(quest_server, executor)
    watchdog.start()
    home_planner = HomePlanner(ROBOT_CONFIG, SCENE_CONFIG, ui.viz)

    # Đăng ký các sự kiện nút bấm trên Viser GUI
    @ui.cb_record.on_update
    def _on_record_update(_):
        if ui.cb_record.value:
            logger.start_episode(task_description="Pick and Place Teleop Demo")
            ui.set_status(f"Recording Ep {logger.current_episode_idx}")
        else:
            saved_file = logger.end_episode(success=True)
            if saved_file:
                ui.set_episodes(logger.current_episode_idx)
                ui.set_status(f"Saved Ep {logger.current_episode_idx - 1}")
            else:
                ui.set_status("Idle")

    @ui.btn_discard.on_click
    def _on_btn_discard_click(_):
        if logger.is_recording:
            logger.discard_episode()
            ui.cb_record.value = False
            ui.set_status("Discarded Episode")

    @ui.btn_reset_object.on_click
    def _on_btn_reset_object(_):
        grasp_sim.reset()
        ui.cb_gripper.value = False
        ui.set_grasp_state(False)
        ui.slider_tomato_x.value = round(float(grasp_sim.tomato_pos[0]), 3)
        ui.slider_tomato_y.value = round(float(grasp_sim.tomato_pos[1]), 3)

    # Callback điều khiển vị trí Quả Cà Chua qua Slider & Nút Preset
    @ui.slider_tomato_x.on_update
    def _on_tomato_x_update(_):
        if not grasp_sim.is_grasped:
            grasp_sim.set_position(ui.slider_tomato_x.value, ui.slider_tomato_y.value)

    @ui.slider_tomato_y.on_update
    def _on_tomato_y_update(_):
        if not grasp_sim.is_grasped:
            grasp_sim.set_position(ui.slider_tomato_x.value, ui.slider_tomato_y.value)

    @ui.btn_preset_under_tool.on_click
    def _on_preset_under_tool(_):
        if not grasp_sim.is_grasped:
            fk = teleop._curobo.fk(ctrl.joints())
            new_x = round(float(fk["pos"][0]), 3)
            new_y = round(float(fk["pos"][1]), 3)
            ui.slider_tomato_x.value = new_x
            ui.slider_tomato_y.value = new_y
            grasp_sim.set_position(new_x, new_y)

    @ui.btn_preset_front.on_click
    def _on_preset_front(_):
        if not grasp_sim.is_grasped:
            ui.slider_tomato_x.value = 0.15
            ui.slider_tomato_y.value = 0.35
            grasp_sim.set_position(0.15, 0.35)

    @ui.btn_preset_left.on_click
    def _on_preset_left(_):
        if not grasp_sim.is_grasped:
            new_x = max(-0.40, round(float(ui.slider_tomato_x.value - 0.10), 3))
            ui.slider_tomato_x.value = new_x
            grasp_sim.set_position(new_x, ui.slider_tomato_y.value)

    @ui.btn_preset_right.on_click
    def _on_preset_right(_):
        if not grasp_sim.is_grasped:
            new_x = min(0.40, round(float(ui.slider_tomato_x.value + 0.10), 3))
            ui.slider_tomato_x.value = new_x
            grasp_sim.set_position(new_x, ui.slider_tomato_y.value)

    @ui.btn_home.on_click
    def _on_btn_home_click(_):
        home_planner.go_home(
            teleop, executor, ctrl, ui.target_widget,
            widget_pose_source, ui.ui_status, ui.cb_teleop,
            poll_sleep=POLL_SLEEP
        )

    # Xử lý tín hiệu tắt an toàn
    def handle_sigint(sig, frame):
        print("\n[Hệ thống] Đang tắt an toàn...")
        teleop.enabled = False
        cam.stop()
        time.sleep(0.5)
        ctrl.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sigint)

    # 6. In hướng dẫn truy cập
    print("\n" + "=" * 40)
    print(f"IP MÁY CHỦ: {local_ip}")
    print(f"-> Viser Dashboard: http://localhost:8080")
    print(f"-> VR WebXR: https://{local_ip}:{quest_server.http_port}")
    print("=" * 40 + "\n")

    _last_teleop_state = False

    # 7. Vòng lặp điều khiển chính (20Hz)
    while True:
        # A. Kiểm tra VR E-Stop
        if quest_server.check_and_clear_estop():
            teleop.enabled = False
            ui.cb_teleop.value = False
            try:
                executor._ctrl.stopJ(2.0)
            except Exception:
                pass
            ui.set_status("EMERGENCY STOP (Từ VR)!")
            quest_server.send_haptic(1.0, 1000)

        # B. Cập nhật trạng thái kết nối
        ui.set_connected(quest_server.is_connected())

        # C. Xử lý bật/tắt Teleop
        if ui.cb_teleop.value != _last_teleop_state:
            teleop.enabled = ui.cb_teleop.value
            _last_teleop_state = ui.cb_teleop.value
            if teleop.enabled:
                print("[VR] KÍCH HOẠT ĐIỀU KHIỂN.")
                executor.connect()
                watchdog.arm()
            else:
                print("[VR] HỦY KÍCH HOẠT ĐIỀU KHIỂN.")
                executor.disconnect()
                watchdog.disarm()

        # D. Điều khiển thời gian thực khi Teleop bật
        if teleop.enabled:
            try:
                q_real = ctrl.joints()
                fk_res = teleop._curobo.fk(q_real)

                # Thực thi bước điều khiển Teleop
                teleop.step(q_real)
                current_status = teleop.get_status()
                status_low = current_status.lower()

                # Đồng bộ trạng thái hiển thị
                if "thất bại" in status_low or "fail" in status_low:
                    ui.set_status("IK Error")
                elif "ngoai tam voi" in status_low or "out of reach" in status_low:
                    ui.set_status("Out of Reach")
                elif "singularity" in status_low or "kì dị" in status_low:
                    ui.set_status("Singularity")
                elif "sigma=" in status_low or "curobo" in status_low:
                    ui.set_status("Tracking")
                else:
                    ui.set_status(current_status.split("|")[0].strip())

                # Cập nhật tọa độ từ VR vào Widget 3D (nếu có kính VR)
                vr_pose = pose_source.get_pose()
                if vr_pose is not None:
                    ui.target_widget.position = vr_pose["pos"]
                    ui.target_widget.wxyz = vr_pose["quat"]
                    pose_source.out_of_reach_time = 0.0
                elif quest_server.is_connected():
                    # Chỉ tự động thu hồi nếu kính VR đang kết nối mà bị thả cò kẹt xa robot
                    is_stuck = "ngoai tam voi" in status_low or "dung vat can" in status_low or "singularity" in status_low
                    pose_source.out_of_reach_time = (pose_source.out_of_reach_time + POLL_SLEEP) if is_stuck else 0.0
                    if pose_source.out_of_reach_time > 2.0:
                        ui.target_widget.position = fk_res["pos"]
                        ui.target_widget.wxyz = fk_res["quat"]
                        widget_pose_source._ema_pos = None
                        widget_pose_source._ema_quat_xyzw = None
                        pose_source.out_of_reach_time = 0.0
                # Nếu không kết nối VR: Người dùng dùng Chuột kéo tự do Widget trên màn hình Viser!

                # Đồng bộ dáng robot ảo trên Viser
                ui.update_robot_state(q_real)

                # Rung phản hồi lực khi có lỗi hoặc vướng singularity
                if "thất bại" in status_low or "cảnh báo" in status_low or "singularity" in status_low:
                    quest_server.send_haptic(intensity=1.0, duration=150)

                # Cập nhật logic gắp thả vật thể 3D (Cà chua 🍅): Hỗ trợ cả VR Grip và Checkbox Chuột
                vr_frame = quest_server.get_frame()
                vr_grip = 1.0 if (vr_frame and vr_frame.get("grip")) else 0.0
                mouse_grip = 1.0 if ui.cb_gripper.value else 0.0
                grip_val = max(vr_grip, mouse_grip)
                is_grip_pressed = bool(grip_val > 0.5)

                is_holding, grasp_event = grasp_sim.update(fk_res["pos"], is_grip_pressed)
                ui.set_grasp_state(is_holding)

                if grasp_event == "GRASP_SUCCESS":
                    quest_server.send_haptic(0.9, 150) # Rung tay cầm báo đã gắp trúng
                elif grasp_event == "PLACE_SUCCESS":
                    quest_server.send_haptic(0.5, 300) # Rung nhẹ báo đã thả trúng giỏ

                # Render góc nhìn từ Camera ảo trên cổ tay robot (Wrist Eye-in-Hand theo CuRobo FK)
                cam_img = wrist_cam.render(
                    tool_pos=fk_res["pos"],
                    tool_quat=fk_res["quat"],
                    tomato_pos=grasp_sim.tomato_pos,
                    basket_pos=grasp_sim.place_basket_pos,
                    gripper_closed=is_grip_pressed
                )

                # Cập nhật ảnh trực tiếp lên sidebar Viser để người dùng quan sát bằng mắt
                ui.update_cam_preview(cam_img)

                # Ghi dữ liệu HDF5 phục vụ huấn luyện AI (kèm ảnh Wrist Camera mô phỏng 3D)
                if logger.is_recording:
                    q_target_cmd = teleop._last_valid_q if teleop._last_valid_q is not None else q_real
                    logger.log_step(
                        q_actual=q_real,
                        qd_actual=ctrl.rr.getActualQd(),
                        tcp_actual=ctrl.tcp(),
                        q_target=q_target_cmd,
                        gripper_actual=grip_val,
                        gripper_target=grip_val,
                        images={"wrist_camera": cam_img},
                    )

            except Exception as e:
                ui.set_status(f"Lỗi: {e}")
                print(f"[Lỗi Điều khiển] {e}")
                traceback.print_exc()
        else:
            ui.set_status("Tắt")

        time.sleep(POLL_SLEEP)

if __name__ == "__main__":
    main()
