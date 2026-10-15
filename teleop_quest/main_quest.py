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
import threading
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
from teleop_quest.grasp_simulator import GraspSimulator
from teleop_quest.camera_driver import create_wrist_camera

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

    import argparse
    parser = argparse.ArgumentParser(description="UR3e WebXR Teleoperation")
    parser.add_argument("--robot-ip", type=str, default=os.environ.get("ROBOT_IP", "127.0.0.1"), help="Địa chỉ IP của Robot UR3e thật hoặc URSim (mặc định: 127.0.0.1)")
    parser.add_argument("--camera", type=str, choices=["synthetic", "real"], default="synthetic", help="Nguồn camera: synthetic (mô phỏng) hoặc real (camera thật)")
    parser.add_argument("--camera-device", type=int, default=0, help="Device ID của camera thật (mặc định: 0)")
    args, _ = parser.parse_known_args()
    os.environ["ROBOT_IP"] = args.robot_ip

    # 1. Khởi động mạng và máy chủ WebXR
    local_ip = get_local_ip()
    base_dir = os.path.dirname(os.path.dirname(__file__))
    cert_path, key_path = setup_ssl(base_dir, local_ip)
    quest_server = QuestServer(cert_path, key_path)
    quest_server.start()


    # 2. Khởi động Controller Robot, An toàn & Data Logger
    print(f"[Hệ thống] Đang kết nối CuRobo và Robot UR3e tại IP: {args.robot_ip}...")
    ctrl = CuRoboURSim(scene_file=SCENE_CONFIG)
    for _ in range(10):
        if np.linalg.norm(ctrl.joints()) > 0.01:
            break
        time.sleep(0.1)
    start_joints = ctrl.joints()
    
    config = load_config()
    logger = DataLogger(config)
    guard = SingularityGuard(config)

    # Kiểm tra trạng thái kỳ dị ban đầu của robot
    try:
        init_m = guard.metrics_calc.get_metrics(torch.tensor([start_joints], device=guard.metrics_calc.device))
        init_sigma = init_m["sigma_min"].item()
        if init_sigma < guard.danger_threshold:
            print(f"[Cảnh báo] Robot đang ở thế kỳ dị (sigma={init_sigma:.3f} < {guard.danger_threshold:.3f}). Hãy đưa robot về Home hoặc chỉnh khớp qua PolyScope.")
    except Exception:
        pass

    # 3. Tính toán vị trí ban đầu của robot qua CuRobo FK (chuẩn Viser world)
    curobo_solver = CuRoboIKSolver(ctrl.ik_solver)
    fk_init = curobo_solver.fk(start_joints)
    robot_init_pos = fk_init["pos"]
    robot_init_quat = fk_init["quat"]

    # Đặt quả cà chua và khay chứa trong tầm với tối ưu của UR3e (Bán kính 25cm - 32cm)
    pick_pos = (0.22, 0.22, 0.035)
    place_pos = (0.20, -0.20, 0.02)

    # Khởi tạo Giao diện Viser 3D
    ui = TeleopViserUI(
        ROBOT_CONFIG,
        init_pos=robot_init_pos,
        init_quat=robot_init_quat,
        init_joints=start_joints,
        init_tomato_pos=(pick_pos[0], pick_pos[1])
    )
    ui.set_episodes(logger.current_episode_idx)

    # Khởi tạo mô phỏng gắp nhặt Cà chua 3D & Camera Wrist (Mô phỏng hoặc Camera thật)
    grasp_sim = GraspSimulator(ui.server, pick_pos=pick_pos, place_pos=place_pos)
    wrist_cam = create_wrist_camera(camera_type=args.camera, device_index=args.camera_device, width=640, height=480)

    # Khởi tạo luồng nền độc lập cho Camera Stream & Preview (Loại bỏ jitter 15-30ms cho vòng lặp RTDE 20Hz)
    camera_state = {
        "tool_pos": robot_init_pos,
        "tool_quat": robot_init_quat,
        "tomato_pos": grasp_sim.tomato_pos,
        "basket_pos": grasp_sim.place_basket_pos,
        "gripper_closed": False,
        "latest_raw_frame": None,
        "latest_preview_frame": None,
        "running": True,
    }
    camera_lock = threading.Lock()

    def camera_worker():
        while camera_state["running"]:
            t_cam_start = time.perf_counter()
            with camera_lock:
                pos = camera_state["tool_pos"]
                quat = camera_state["tool_quat"]
                tom_pos = camera_state["tomato_pos"]
                b_pos = camera_state["basket_pos"]
                grp = camera_state["gripper_closed"]

            try:
                raw_frame = wrist_cam.get_frame(
                    tool_pos=pos,
                    tool_quat=quat,
                    tomato_pos=tom_pos,
                    basket_pos=b_pos,
                    gripper_closed=grp
                )
                preview_frame = wrist_cam.add_osd_preview(raw_frame, grp)
                with camera_lock:
                    camera_state["latest_raw_frame"] = raw_frame
                    camera_state["latest_preview_frame"] = preview_frame

                ui.update_cam_preview(preview_frame)
                quest_server.send_camera_frame(preview_frame)
            except Exception:
                pass

            dur = time.perf_counter() - t_cam_start
            time.sleep(max(0.005, 0.05 - dur))

    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start()

    # 4. Thiết lập nguồn tọa độ VR & Pipeline Teleop
    widget_pose_source = WidgetPoseSource(ui.target_widget)
    executor = ServoJExecutor(dt=POLL_SLEEP, lookahead_time=0.12, gain=250)
    teleop = TeleoperationController(
        pose_source=widget_pose_source,
        curobo_solver=curobo_solver,
        safety_gate=SafetyGate(guard=guard),
        executor=executor
    )

    pose_source = DeltaPoseSource(
        quest_server, ctrl, scale=0.6,
        anchor_reference_func=lambda: get_robot_fk_as_ur(teleop, ctrl),
        alpha=0.25,
        max_delta_m=0.18
    )

    # 5. Khởi tạo bộ giám sát mất kết nối & Home Planner
    watchdog = TimeoutWatchdog(quest_server, executor)
    watchdog.start()
    SAFE_HOME_JOINTS = [0.0, -2.2, 1.9, -1.383, -1.57, 0.0]
    home_planner = HomePlanner(ROBOT_CONFIG, SCENE_CONFIG, ui.viz, default_home_q=SAFE_HOME_JOINTS)

    _placed_at_start = 0

    # Đăng ký các sự kiện nút bấm trên Viser GUI
    @ui.cb_record.on_update
    def _on_record_update(_):
        nonlocal _placed_at_start
        if ui.cb_record.value:
            _placed_at_start = grasp_sim.placed_count
            init_meta = {
                "tomato_init_pos": list(grasp_sim.tomato_pos),
                "basket_pos": list(grasp_sim.place_basket_pos),
            }
            logger.start_episode(task_description="Pick and Place Tomato Teleop", metadata=init_meta)
            ui.set_status(f"Recording Ep {logger.current_episode_idx}")
        else:
            # Thành công nếu số lượng quả cà chua đặt vào giỏ tăng lên và không còn bị kẹp
            ep_success = bool(grasp_sim.placed_count > _placed_at_start and not grasp_sim.is_grasped)
            saved_file = logger.end_episode(success=ep_success)
            if saved_file:
                ui.set_episodes(logger.current_episode_idx)
                status_str = "Thành công" if ep_success else "Chưa hoàn thành"
                ui.set_status(f"Saved Ep {logger.current_episode_idx - 1} ({status_str})")
            else:
                ui.set_status("Idle")

    @ui.cb_discard.on_update
    def _on_cb_discard_update(_):
        if ui.cb_discard.value:
            if logger.is_recording:
                logger.discard_episode()
                ui.cb_record.value = False
                ui.set_status("Discarded Episode")
                print("[AI Logger] Đã hủy Episode hiện tại theo yêu cầu!")
            time.sleep(0.2)
            ui.cb_discard.value = False

    is_navigating_home = threading.Event()

    @ui.btn_home.on_click
    def _on_btn_home_click(_):
        if is_navigating_home.is_set():
            print("[HomePlanner] Đang di chuyển về Home, vui lòng đợi...")
            return
        threading.Thread(
            target=home_planner.go_home,
            args=(
                teleop, executor, ctrl, ui.target_widget,
                widget_pose_source, ui.ui_status, ui.cb_teleop
            ),
            kwargs={
                "is_navigating_home": is_navigating_home,
                "poll_sleep": POLL_SLEEP
            },
            daemon=True
        ).start()


    # Xử lý tín hiệu tắt an toàn
    def handle_sigint(sig, frame):
        print("\n[Hệ thống] Đang tắt an toàn...")
        camera_state["running"] = False
        teleop.enabled = False
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
    _last_quest_drive_mode = quest_server.get_drive_mode()
    _was_navigating_home = False
    _frame_count = 0

    # 7. Vòng lặp điều khiển chính (20Hz)
    while True:
        loop_start_time = time.perf_counter()

        # Nếu đang thực hiện quy hoạch / di chuyển về Home, tạm hoãn luồng Teleop
        if is_navigating_home.is_set():
            _was_navigating_home = True
            time.sleep(POLL_SLEEP)
            continue

        # Sau khi vừa hoàn thành trở về Home: Tự động kích hoạt lại Teleop (không cần người dùng bấm lại)
        if _was_navigating_home:
            _was_navigating_home = False
            _last_teleop_state = False  # Đặt lại trạng thái cũ để khối đồng bộ bên dưới kích hoạt tự động
            ui.cb_teleop.value = True

        # A. Kiểm tra VR E-Stop
        if quest_server.check_and_clear_estop():
            teleop.enabled = False
            ui.cb_teleop.value = False
            quest_server.set_drive_mode(False)
            _last_teleop_state = False
            _last_quest_drive_mode = False
            try:
                executor._ctrl.stopJ(2.0)
            except Exception:
                pass
            ui.set_status("EMERGENCY STOP (Từ VR)!")
            quest_server.send_haptic(1.0, 1000)

        # A1. Kiểm tra lệnh Go Home từ tay cầm VR (Nhấn Thumbstick hoặc Giữ phím B)
        if quest_server.check_and_clear_home():
            if not is_navigating_home.is_set():
                print("[Quest] Nhận lệnh Go Home từ tay cầm VR! Bắt đầu đưa robot về Home an toàn...")
                ui.set_status("Returning Home (Từ VR)...")
                quest_server.send_haptic(1.0, 300)
                threading.Thread(
                    target=home_planner.go_home,
                    args=(
                        teleop, executor, ctrl, ui.target_widget,
                        widget_pose_source, ui.ui_status, ui.cb_teleop
                    ),
                    kwargs={
                        "is_navigating_home": is_navigating_home,
                        "poll_sleep": POLL_SLEEP
                    },
                    daemon=True
                ).start()

        # B. Cập nhật trạng thái kết nối
        ui.set_connected(quest_server.is_connected())

        # C. Xử lý bật/tắt Teleop (Đồng bộ giữa VR Quest và Viser Dashboard)
        if quest_server.is_connected():
            curr_quest_mode = quest_server.get_drive_mode()
            if curr_quest_mode != _last_quest_drive_mode:
                _last_quest_drive_mode = curr_quest_mode
                ui.cb_teleop.value = curr_quest_mode

        if ui.cb_teleop.value != _last_teleop_state:
            _last_teleop_state = ui.cb_teleop.value
            teleop.enabled = ui.cb_teleop.value
            quest_server.set_drive_mode(ui.cb_teleop.value)
            _last_quest_drive_mode = ui.cb_teleop.value

            if teleop.enabled:
                print("[Teleop] KÍCH HOẠT ĐIỀU KHIỂN.")
                pose_source.reset_anchor()
                executor.connect()
                if quest_server.is_connected():
                    watchdog.arm()
                ui.set_status("Tracking")
            else:
                print("[Teleop] HỦY KÍCH HOẠT ĐIỀU KHIỂN.")
                executor.disconnect()
                watchdog.disarm()
                ui.set_status("Tắt")

        # D. Lấy trạng thái góc khớp và FK của robot thực tế
        try:
            q_real = ctrl.joints()
            fk_res = teleop._curobo.fk(q_real)
        except Exception:
            q_real = None
            fk_res = None

        is_grip_pressed = False

        if teleop.enabled and fk_res is not None:
            try:
                # Cập nhật tọa độ từ VR vào Widget 3D ngay trước khi giải IK (giảm tối đa độ trễ)
                vr_pose = pose_source.get_pose()
                if vr_pose is not None:
                    ui.target_widget.position = vr_pose["pos"]
                    ui.target_widget.wxyz = vr_pose["quat"]

                # Thực thi bước điều khiển Teleop (Widget -> CuRobo IK -> RTDE ServoJ)
                teleop.step(q_real)
                current_status = teleop.get_status()
                status_low = current_status.lower()

                # Đồng bộ trạng thái hiển thị
                if "thất bại" in status_low or "fail" in status_low:
                    ui.set_status("IK Error")
                elif "ngoai tam voi" in status_low or "ngoài tầm với" in status_low or "out of reach" in status_low:
                    ui.set_status("Ngoài tầm với (Out of Reach)")
                elif "mặt bàn" in status_low or "mat ban" in status_low or "table limit" in status_low:
                    ui.set_status("Sát mặt bàn bảo vệ")
                elif "van xoan" in status_low or "vặn xoắn" in status_low or "tu the ket" in status_low or "tư thế kẹt" in status_low:
                    ui.set_status("Tư thế kẹt (Vặn xoắn)")
                elif "vat can" in status_low or "vật cản" in status_low:
                    ui.set_status("Vướng vật cản")
                elif "singularity" in status_low or "kì dị" in status_low:
                    ui.set_status("Singularity")
                elif "sigma=" in status_low or "curobo" in status_low:
                    ui.set_status("Tracking")
                else:
                    ui.set_status(current_status.split("|")[0].strip())

                # Kiểm tra trạng thái lỗi vặn xoắn và vướng vật cản
                is_twisting = any(err in status_low for err in ["van xoan", "vặn xoắn", "tu the ket", "tư thế kẹt", "twist"])
                is_obstacle = any(err in status_low for err in ["vat can", "vật cản", "dung vat can", "đụng vật cản", "obstacle"])
                is_stuck = is_twisting or is_obstacle or any(err in status_low for err in [
                    "ngoai tam voi", "ngoài tầm với", "out of reach",
                    "singularity", "wrist_clamp", "that bai", "thất bại", "fail", "ik error"
                ])

                # Khi bị lỗi tư thế vặn xoắn hoặc vướng vật cản kéo dài: tự động đưa XYZ về đầu tay máy
                snap_threshold = 1.2 if (is_twisting or is_obstacle) else 1.8
                if is_stuck:
                    pose_source.out_of_reach_time += POLL_SLEEP
                    quest_server.send_haptic(intensity=0.8, duration=80)
                else:
                    pose_source.out_of_reach_time = 0.0

                if pose_source.out_of_reach_time >= snap_threshold:
                    ui.target_widget.position = fk_res["pos"]
                    ui.target_widget.wxyz = fk_res["quat"]
                    widget_pose_source._ema_pos = None
                    widget_pose_source._ema_quat_xyzw = None
                    if hasattr(pose_source, "_anchor_pos_quest"):
                        pose_source._anchor_pos_quest = None
                        pose_source._anchor_tcp = None
                        pose_source._ema_pos = None
                        pose_source._ema_quat_xyzw = None
                        pose_source._last_button_state = 0
                    pose_source.out_of_reach_time = 0.0
                    teleop._last_valid_q = list(q_real)
                    ui.set_status("Tự động thu hồi XYZ")
                    quest_server.send_haptic(intensity=1.0, duration=200)
                    
                    if not hasattr(main, "_last_snap_log_time") or time.time() - main._last_snap_log_time > 3.0:
                        main._last_snap_log_time = time.time()
                        print(f"[Auto-Snap] Dừng: {current_status}")

                # Cập nhật logic gắp thả vật thể 3D (Cà chua): Độc lập nút Gripper (nút A/X) với Clutch xoay (Grip)
                vr_frame = quest_server.get_frame()
                vr_gripper = 1.0 if (vr_frame and (vr_frame.get("gripper_btn", False) or vr_frame.get("button_a", False))) else 0.0
                mouse_grip = 1.0 if ui.cb_gripper.value else 0.0
                grip_val = max(vr_gripper, mouse_grip)
                is_grip_pressed = bool(grip_val > 0.5)

                is_holding, grasp_event = grasp_sim.update(fk_res["pos"], is_grip_pressed)
                ui.set_grasp_state(is_holding)

                if grasp_event == "GRASP_SUCCESS":
                    quest_server.send_haptic(0.9, 150)
                elif grasp_event == "PLACE_SUCCESS":
                    quest_server.send_haptic(0.5, 300)

                # Ghi dữ liệu HDF5 phục vụ huấn luyện AI
                if logger.is_recording:
                    q_target_cmd = teleop._last_valid_q if teleop._last_valid_q is not None else q_real
                    with camera_lock:
                        logged_img = camera_state["latest_raw_frame"]
                    logger.log_step(
                        q_actual=q_real,
                        qd_actual=ctrl.rr.getActualQd(),
                        tcp_actual=ctrl.tcp(),
                        q_target=q_target_cmd,
                        gripper_actual=grip_val,
                        gripper_target=grip_val,
                        images={"wrist_camera": logged_img},
                    )

            except Exception as e:
                ui.set_status(f"Lỗi: {e}")
                print(f"[Lỗi Điều khiển] {e}")
                traceback.print_exc()
        elif not teleop.enabled:
            curr_st = str(ui.ui_status.value)
            if not curr_st.startswith("Lỗi") and not curr_st.startswith("EMERGENCY") and not "Error" in curr_st:
                ui.set_status("Tắt")

        # E. Cập nhật giao diện 3D và chia sẻ trạng thái cho luồng Camera nền
        if fk_res is not None:
            # Đồng bộ dáng robot thực tế lên Viser 3D (nhẹ, chỉ gửi 1 gói websocket cập nhật góc khớp)
            ui.update_robot_state(q_real)

            # Cập nhật trạng thái mới nhất cho luồng camera_worker chạy ngầm độc lập
            current_grip = is_grip_pressed if teleop.enabled else bool(ui.cb_gripper.value)
            with camera_lock:
                camera_state["tool_pos"] = fk_res["pos"]
                camera_state["tool_quat"] = fk_res["quat"]
                camera_state["tomato_pos"] = grasp_sim.tomato_pos
                camera_state["basket_pos"] = grasp_sim.place_basket_pos
                camera_state["gripper_closed"] = current_grip

        # Bù trừ thời gian trễ thực tế để duy trì chính xác tần số 20Hz (50ms/bước)
        elapsed = time.perf_counter() - loop_start_time
        sleep_duration = max(0.001, POLL_SLEEP - elapsed)
        time.sleep(sleep_duration)

if __name__ == "__main__":
    main()
