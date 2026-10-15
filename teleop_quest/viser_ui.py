import time
import subprocess
import cv2
import numpy as np
import torch
import transforms3d
from curobo.viewer import ViserVisualizer
from curobo.types import ContentPath, JointState

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]

class StatusProxy:
    """Proxy hỗ trợ truy cập .value hai chiều tương thích với Viser GUI text input cũ"""
    def __init__(self, refresh_cb, initial_value=""):
        self._val = str(initial_value)
        self._refresh_cb = refresh_cb

    @property
    def value(self):
        return self._val

    @value.setter
    def value(self, new_val):
        self._val = str(new_val)
        if self._refresh_cb:
            self._refresh_cb()

    def __str__(self):
        return self._val


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

        # Trạng thái hệ thống hiển thị dạng Dashboard Markdown compact
        self._val_status = "Waiting..."
        self._val_conn = "No"
        self._val_episodes = "0"
        self._val_grasp = "Empty"

        self.ui_status = StatusProxy(self._on_status_change, "Waiting...")
        self.ui_conn = StatusProxy(self._on_conn_change, "No")
        self.ui_episodes = StatusProxy(self._on_episodes_change, "0")
        self.ui_grasp_state = StatusProxy(self._on_grasp_change, "Empty")

        # Bảng điều khiển GUI tinh gọn
        with self.server.gui.add_folder("Teleoperation"):
            # 1. Bảng trạng thái gọn gàng (Thay thế hoàn toàn 4 ô input xám)
            self._md_status = self.server.gui.add_markdown(self._build_status_md())
            self.server.gui.add_divider()

            # 2. Cụm công tắc 2x2 (Hàng 1: Teleop & Record AI | Hàng 2: Gripper & Discard)
            self.server.gui.add_html("""
            <div id="teleop-2x2-anchor" style="display:none;"></div>
            <style>
            div:has(> div > #teleop-2x2-anchor) {
                display: flex !important;
                flex-wrap: wrap !important;
                row-gap: 4px !important;
            }
            div:has(> div > #teleop-2x2-anchor) > div:has(#teleop-2x2-anchor) {
                display: none !important;
            }
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(1),
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(2),
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(8),
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(9) {
                width: 100% !important;
            }
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(4),
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(5),
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(6),
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(7) {
                width: 50% !important;
                box-sizing: border-box !important;
                display: flex !important;
                align-items: center !important;
            }
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(4) > div,
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(5) > div,
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(6) > div,
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(7) > div {
                width: 100% !important;
                padding-bottom: 0 !important;
            }
            div:has(> div > #teleop-2x2-anchor) .mantine-Flex-root {
                justify-content: space-between !important;
                width: 100% !important;
            }
            div:has(> div > #teleop-2x2-anchor) .mantine-Flex-root > div:first-child {
                width: auto !important;
                padding-right: 4px !important;
            }
            div:has(> div > #teleop-2x2-anchor) > div:nth-child(7) button {
                height: 28px !important;
                padding: 0 4px !important;
                font-size: 0.85em !important;
            }
            </style>
            """)

            # Hàng 1: Teleop & Record AI
            self.cb_teleop = self.server.gui.add_checkbox("Bật Teleop", initial_value=False)
            self.cb_record = self.server.gui.add_checkbox("Record AI", initial_value=False)

            # Hàng 2: Gripper & Discard
            self.cb_gripper = self.server.gui.add_checkbox("Gripper", initial_value=False)
            self.cb_discard = self.server.gui.add_checkbox("Discard", initial_value=False)

            self.server.gui.add_divider()

            # 3. Nút khôi phục an toàn (Full width bên dưới cụm 2x2)
            self.btn_home = self.server.gui.add_button("Go to Home")

        # Thư mục Camera cổ tay thu gọn (collapsible, có thể gập mở tùy ý)
        with self.server.gui.add_folder("Wrist Camera Live Preview", expand_by_default=True):
            init_img = np.full((150, 200, 3), 200, dtype=np.uint8)
            self.ui_cam_preview = self.server.gui.add_image(init_img)

    def _on_status_change(self):
        self._val_status = self.ui_status.value
        self._refresh_status_md()

    def _on_conn_change(self):
        self._val_conn = self.ui_conn.value
        self._refresh_status_md()

    def _on_episodes_change(self):
        self._val_episodes = self.ui_episodes.value
        self._refresh_status_md()

    def _on_grasp_change(self):
        self._val_grasp = self.ui_grasp_state.value
        self._refresh_status_md()

    def _build_status_md(self) -> str:
        return (
            f"**Status:** `{self._val_status}` &nbsp;|&nbsp; **VR:** `{self._val_conn}`  \n"
            f"**Grasp:** `{self._val_grasp}` &nbsp;|&nbsp; **Episodes:** `{self._val_episodes}`"
        )

    def _refresh_status_md(self):
        self._md_status.content = self._build_status_md()

    def update_robot_state(self, q: list):
        """Cập nhật dáng robot thực tế lên Viser 3D"""
        js = JointState.from_position(
            torch.tensor([q], device="cuda", dtype=torch.float32),
            joint_names=JOINT_NAMES
        )
        self.viz.set_joint_state(js)

    def update_cam_preview(self, frame_rgb: np.ndarray):
        """Cập nhật ảnh xem trực tiếp camera cổ tay lên sidebar Viser"""
        thumb = cv2.resize(frame_rgb, (200, 150))
        self.ui_cam_preview.image = thumb

    def set_status(self, text: str):
        self.ui_status.value = text

    def set_connected(self, is_connected: bool):
        self.ui_conn.value = "Yes" if is_connected else "No"

    def set_episodes(self, count: int):
        self.ui_episodes.value = str(count)

    def set_grasp_state(self, is_grasped: bool):
        self.ui_grasp_state.value = "Holding" if is_grasped else "Empty"
