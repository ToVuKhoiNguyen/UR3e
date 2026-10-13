import time
import subprocess
import numpy as np
import torch
import transforms3d
from curobo.viewer import ViserVisualizer
from curobo.types import ContentPath, JointState

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

class TeleopViserUI:
    """
    Quản lý giao diện Viser 3D:
      - Mô hình robot ảo & đồng bộ trạng thái khớp
      - Trục tọa độ 3D Transform Controls (Target Widget)
      - Bảng điều khiển nút bấm GUI (Teleop, Record, Discard, Home, Status)
    """
    def __init__(
        self,
        robot_config_path: str,
        init_pos: list,
        init_quat: list,
        init_joints: list,
        init_tomato_pos: tuple = (0.15, 0.35),
        port: int = 8080
    ):
        # Dọn dẹp port cũ nếu có
        subprocess.run(f"fuser -k {port}/tcp 2>/dev/null", shell=True)
        time.sleep(0.5)

        self.viz = ViserVisualizer(
            content_path=ContentPath(robot_config_file=robot_config_path),
            connect_ip="0.0.0.0",
            connect_port=port,
            add_control_frames=False,
        )
        self.server = self.viz._server

        # Đồng bộ khớp ban đầu
        self.update_robot_state(init_joints)

        # Tạo Widget 3D tại tọa độ TCP ban đầu của robot (chuẩn CuRobo world)
        pos = init_pos[:3]
        if len(init_quat) == 4:
            quat = init_quat
        else:
            axis = np.array(init_quat)
            angle = np.linalg.norm(axis)
            quat = transforms3d.quaternions.axangle2quat(axis / angle, angle) if angle > 1e-6 else [1.0, 0.0, 0.0, 0.0]

        self.target_widget = self.server.scene.add_transform_controls(
            "target_pose",
            depth_test=False,
            position=pos,
            wxyz=quat,
            scale=0.18,
            disable_sliders=False
        )

        # Bảng điều khiển GUI
        with self.server.gui.add_folder("VR / Chuột Teleoperation"):
            self.cb_teleop = self.server.gui.add_checkbox("Bật Điều Khiển (Teleop)", initial_value=False)
            self.cb_gripper = self.server.gui.add_checkbox("Gripper (Bóp Kẹp)", initial_value=False)
            self.btn_reset_object = self.server.gui.add_button("Reset Cà chua 🍅")
            self.cb_record = self.server.gui.add_checkbox("Record AI Episode", initial_value=False)
            self.btn_discard = self.server.gui.add_button("Discard Episode (Lỗi/Rơi)")
            self.btn_home = self.server.gui.add_button("Go to Home")
            self.btn_snap = self.server.gui.add_button("📍 Thu hồi trục XYZ về tay máy")
            self.ui_grasp_state = self.server.gui.add_text("Grasp", initial_value="Empty", disabled=True)
            self.ui_episodes = self.server.gui.add_text("Saved Episodes", initial_value="0", disabled=True)
            self.ui_status = self.server.gui.add_text("Status", initial_value="Waiting...", disabled=True)
            self.ui_conn = self.server.gui.add_text("VR Connected", initial_value="No", disabled=True)

        with self.server.gui.add_folder("Vị trí Cà chua 🍅 (Test Camera)"):
            self.slider_tomato_x = self.server.gui.add_slider(
                "Tọa độ X (Trái / Phải)",
                min=-0.40,
                max=0.40,
                step=0.01,
                initial_value=float(init_tomato_pos[0])
            )
            self.slider_tomato_y = self.server.gui.add_slider(
                "Tọa độ Y (Gần / Xa)",
                min=0.10,
                max=0.60,
                step=0.01,
                initial_value=float(init_tomato_pos[1])
            )
            self.btn_preset_under_tool = self.server.gui.add_button("📍 Đặt ngay dưới đầu gắp")
            self.btn_preset_front = self.server.gui.add_button("🍅 Phía trước (+15cm, +35cm)")
            self.btn_preset_left = self.server.gui.add_button("⬅️ Dịch trái (-10cm)")
            self.btn_preset_right = self.server.gui.add_button("➡️ Dịch phải (+10cm)")

        with self.server.gui.add_folder("Wrist Camera Live Preview"):
            init_img = np.full((180, 240, 3), 200, dtype=np.uint8)
            self.ui_cam_preview = self.server.gui.add_image(init_img, label="Góc nhìn Cổ tay Robot")

    def update_robot_state(self, q: list):
        """Cập nhật dáng robot thực tế lên Viser 3D"""
        js = JointState.from_position(
            torch.tensor([q], device="cuda", dtype=torch.float32),
            joint_names=JOINT_NAMES
        )
        self.viz.set_joint_state(js)

    def update_cam_preview(self, frame_rgb: np.ndarray):
        """Cập nhật ảnh xem trực tiếp camera cổ tay lên sidebar Viser"""
        import cv2
        thumb = cv2.resize(frame_rgb, (240, 180))
        self.ui_cam_preview.image = thumb

    def set_status(self, text: str):
        self.ui_status.value = text

    def set_connected(self, is_connected: bool):
        self.ui_conn.value = "Yes" if is_connected else "No"

    def set_episodes(self, count: int):
        self.ui_episodes.value = str(count)

    def set_grasp_state(self, is_grasped: bool):
        self.ui_grasp_state.value = "Holding 🍅" if is_grasped else "Empty"
