import threading
import time
from typing import Optional, Union
import cv2
import numpy as np

from teleop_quest.synthetic_camera import SyntheticWristCamera

class BaseWristCamera:
    """Lớp cơ sở cho Camera gắn cổ tay robot (Eye-in-Hand)."""
    def get_frame(self, **kwargs) -> np.ndarray:
        raise NotImplementedError

    def add_osd_preview(self, frame_rgb: np.ndarray, gripper_closed: bool = False) -> np.ndarray:
        raise NotImplementedError

    def is_real(self) -> bool:
        return False

    def release(self):
        pass


class SyntheticWristCameraWrapper(BaseWristCamera):
    """Bọc Camera ảo 3D mô phỏng góc nhìn cổ tay robot UR3e."""
    def __init__(self, width: int = 640, height: int = 480, fov_deg: float = 65.0):
        self.cam = SyntheticWristCamera(width=width, height=height, fov_deg=fov_deg)
        self.width = width
        self.height = height

    def get_frame(
        self,
        tool_pos: Optional[Union[list, np.ndarray]] = None,
        tool_quat: Optional[Union[list, np.ndarray]] = None,
        tomato_pos: Optional[np.ndarray] = None,
        basket_pos: Optional[np.ndarray] = None,
        gripper_closed: bool = False,
        **kwargs
    ) -> np.ndarray:
        return self.cam.render(
            tool_pos=tool_pos,
            tool_quat=tool_quat,
            tomato_pos=tomato_pos,
            basket_pos=basket_pos,
            gripper_closed=gripper_closed
        )

    def add_osd_preview(self, frame_rgb: np.ndarray, gripper_closed: bool = False) -> np.ndarray:
        return self.cam.add_osd_preview(frame_rgb, gripper_closed)

    def is_real(self) -> bool:
        return False


class RealWristCamera(BaseWristCamera):
    """
    Driver đọc luồng trực tiếp thời gian thực từ Camera vật lý thật:
    - Hỗ trợ Intel RealSense D435 / D405 / D455 hoặc bất kỳ USB / UVC Camera qua OpenCV.
    - Chạy luồng nền chuyên dụng (Capture Thread) tách biệt khỏi vòng lặp điều khiển 20Hz của robot.
    - Tự động bắt lại kết nối nếu dây cắm camera bị lỏng hoặc chập chờn.
    - Cung cấp frame RGB chuẩn (480, 640, 3) thời gian thực cho Viser GUI, WebXR HUD và DataLogger HDF5.
    """
    def __init__(
        self,
        device_index: Union[int, str] = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30
    ):
        self.device_index = device_index
        self.width = width
        self.height = height
        self.fps = fps

        self._lock = threading.Lock()
        self._running = True
        self._latest_frame: Optional[np.ndarray] = None
        self._fps_actual = 0.0
        self._is_connected = False

        self._thread = threading.Thread(target=self._capture_worker, daemon=True)
        self._thread.start()

    def _capture_worker(self):
        """Worker chạy nền liên tục đọc frame từ phần cứng với độ trễ thấp nhất."""
        cap = None
        frame_count = 0
        fps_start = time.perf_counter()

        while self._running:
            if cap is None or not cap.isOpened():
                try:
                    cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
                    if not cap.isOpened():
                        cap = cv2.VideoCapture(self.device_index)
                    if cap.isOpened():
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                        cap.set(cv2.CAP_PROP_FPS, self.fps)
                        with self._lock:
                            self._is_connected = True
                        print(f"[RealCamera] Đã mở thành công camera thiết bị: {self.device_index}")
                    else:
                        with self._lock:
                            self._is_connected = False
                        time.sleep(1.0)
                        continue
                except Exception as e:
                    with self._lock:
                        self._is_connected = False
                    time.sleep(1.0)
                    continue

            ret, bgr_frame = cap.read()
            if not ret or bgr_frame is None:
                with self._lock:
                    self._is_connected = False
                if cap is not None:
                    cap.release()
                cap = None
                time.sleep(0.5)
                continue

            # Chuyển đổi sang kích thước chuẩn và hệ màu RGB
            if bgr_frame.shape[1] != self.width or bgr_frame.shape[0] != self.height:
                bgr_frame = cv2.resize(bgr_frame, (self.width, self.height))

            rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)

            frame_count += 1
            now = time.perf_counter()
            if now - fps_start >= 1.0:
                self._fps_actual = frame_count / (now - fps_start)
                frame_count = 0
                fps_start = now

            with self._lock:
                self._latest_frame = rgb_frame
                self._is_connected = True

        if cap is not None:
            cap.release()

    def get_frame(self, **kwargs) -> np.ndarray:
        """Lấy frame RGB mới nhất từ camera vật lý trong thời gian thực (<0.1ms)."""
        with self._lock:
            if self._latest_frame is not None and self._is_connected:
                return self._latest_frame.copy()

        # Tạo frame cảnh báo nếu camera thật chưa sẵn sàng hoặc bị mất kết nối
        placeholder = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        # Nền xám tối
        placeholder[:] = (20, 25, 30)
        cv2.putText(
            placeholder,
            f"REAL CAMERA [DEV: {self.device_index}] WAITING...",
            (40, self.height // 2 - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 200, 50),
            2
        )
        cv2.putText(
            placeholder,
            "Connecting hardware sensor / RealSense...",
            (40, self.height // 2 + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 180, 180),
            1
        )
        return placeholder

    def add_osd_preview(self, frame_rgb: np.ndarray, gripper_closed: bool = False) -> np.ndarray:
        """Gắn thông số OSD thời gian thực lên ảnh camera thật."""
        out = frame_rgb.copy()
        h, w = out.shape[:2]

        # Thanh tiêu đề trên
        cv2.rectangle(out, (0, 0), (w, 32), (15, 23, 42), -1)
        fps_text = f"REAL WRIST CAM (HARDWARE) | {self._fps_actual:.1f} FPS"
        cv2.putText(out, fps_text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        # Trạng thái kẹp
        status_text = "GRIPPER: CLOSED [100%]" if gripper_closed else "GRIPPER: OPEN [0%]"
        status_color = (255, 80, 80) if gripper_closed else (80, 255, 120)
        cv2.putText(out, status_text, (w - 240, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1, cv2.LINE_AA)

        # Crosshair ngắm trung tâm đầu gắp
        cx, cy = w // 2, h // 2
        cv2.drawMarker(out, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 24, 1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), 18, (0, 255, 255), 1, cv2.LINE_AA)

        return out

    def is_real(self) -> bool:
        return True

    def release(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)


def create_wrist_camera(
    camera_type: str = "synthetic",
    device_index: Union[int, str] = 0,
    width: int = 640,
    height: int = 480,
    fov_deg: float = 65.0,
    fps: int = 30
) -> BaseWristCamera:
    """
    Factory khởi tạo camera cổ tay UR3e:
    - camera_type = 'synthetic': Sử dụng Camera ảo mô phỏng 3D Eye-in-Hand.
    - camera_type = 'real': Sử dụng Camera vật lý thật gắn trên cổ tay (RealSense D435 / USB Camera).
    """
    if camera_type.lower() == "real":
        print(f"[Hệ thống] Khởi tạo Camera vật lý thật (Real Camera Driver, thiết bị: {device_index})")
        return RealWristCamera(device_index=device_index, width=width, height=height, fps=fps)
    else:
        print("[Hệ thống] Khởi tạo Camera ảo mô phỏng 3D (Synthetic Wrist Camera)")
        return SyntheticWristCameraWrapper(width=width, height=height, fov_deg=fov_deg)
