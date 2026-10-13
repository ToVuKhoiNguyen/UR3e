import time
import threading
import numpy as np
from typing import Optional, Dict

class CameraWorker:
    """
    Luồng thu thập hình ảnh camera chạy ngầm (Asynchronous Camera Worker).
    - Đảm bảo việc đọc ảnh không làm tụt tần số 20Hz của vòng lặp điều khiển robot.
    - Hỗ trợ cả Camera thật (Webcam / USB Camera qua OpenCV) và RealSense / Mock Camera.
    """
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480, name: str = "wrist_camera"):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.name = name
        
        self._latest_frame: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._cap = None

    def start(self) -> bool:
        """Khởi động luồng đọc camera ngầm"""
        import cv2
        self._cap = cv2.VideoCapture(self.camera_id)
        if not self._cap.isOpened():
            print(f"[CameraWorker] Cảnh báo: Không thể mở camera {self.camera_id}. Kích hoạt chế độ Synthetic Mock Camera.")
            self._cap = None
        else:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            print(f"[CameraWorker] Đã kết nối Camera thật (ID: {self.camera_id}, {self.width}x{self.height}) -> {self.name}")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def _capture_loop(self):
        import cv2
        while self._running:
            if self._cap is not None:
                ret, frame = self._cap.read()
                if ret:
                    # Chuyển BGR (OpenCV) sang RGB (Chuẩn Deep Learning / PyTorch)
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    with self._lock:
                        self._latest_frame = frame_rgb
                else:
                    time.sleep(0.01)
            else:
                # Mock Synthetic Frame nếu không có camera vật lý
                mock_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                # Vẽ gradient và timestamp
                mock_img[:, :, 0] = int((time.time() * 20) % 255)
                mock_img[:, :, 1] = 120
                mock_img[:, :, 2] = 200
                with self._lock:
                    self._latest_frame = mock_img
                time.sleep(0.033) # ~30 FPS

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Lấy frame mới nhất trong RAM (0ms latency, không block teleop)"""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def stop(self):
        """Dừng camera"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._cap is not None:
            self._cap.release()
        print(f"[CameraWorker] Đã tắt Camera {self.name}")
