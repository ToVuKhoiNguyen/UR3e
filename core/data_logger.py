import os
import time
import numpy as np
import h5py
from typing import Optional, Dict, Any, List

class DataLogger:
    """
    Bộ thu thập dữ liệu chuyên gia chuẩn HDF5 cho Imitation Learning (ACT / Diffusion Policy).
    Hỗ trợ quản lý theo từng Episode:
      - start_episode()
      - log_step(...)
      - end_episode(success=True)
      - discard_episode()
    """
    def __init__(self, config: Dict[str, Any]):
        self.output_dir = config.get("output_dir", "logs/ai_training_data")
        self.hz = config.get("hz", 20)
        self.is_recording = False
        
        self.current_episode_idx = 0
        self._init_episode_index()
        
        # Buffer trong RAM cho 1 episode
        self._reset_buffer()

    def _init_episode_index(self):
        """Xác định số thứ tự episode tiếp theo dựa trên các file đã có"""
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir, exist_ok=True)
            self.current_episode_idx = 0
            return
            
        existing_files = [f for f in os.listdir(self.output_dir) if f.startswith("episode_") and f.endswith(".hdf5")]
        if not existing_files:
            self.current_episode_idx = 0
        else:
            indices = []
            for f in existing_files:
                try:
                    idx = int(f.replace("episode_", "").replace(".hdf5", ""))
                    indices.append(idx)
                except ValueError:
                    pass
            self.current_episode_idx = max(indices) + 1 if indices else 0

    def _reset_buffer(self):
        self.buffer_q_actual: List[List[float]] = []
        self.buffer_qd_actual: List[List[float]] = []
        self.buffer_tcp_actual: List[List[float]] = []
        self.buffer_gripper_actual: List[float] = []
        
        self.buffer_q_target: List[List[float]] = []
        self.buffer_gripper_target: List[float] = []
        
        self.buffer_timestamps_ms: List[int] = []
        self.buffer_images: Dict[str, List[np.ndarray]] = {}
        self.episode_start_time = 0.0

    def start_episode(self, task_description: str = "UR3e Teleoperation Pick and Place"):
        """Bắt đầu một episode mới"""
        self._reset_buffer()
        self.task_description = task_description
        self.episode_start_time = time.time()
        self.is_recording = True
        print(f"[AI DataLogger] >>> BẮT ĐẦU Episode {self.current_episode_idx} ({self.task_description})")

    def log_step(
        self,
        q_actual: list,
        qd_actual: list,
        tcp_actual: list,
        q_target: Optional[list] = None,
        gripper_actual: float = 0.0,
        gripper_target: float = 0.0,
        images: Optional[Dict[str, np.ndarray]] = None,
        image_path: Optional[str] = None, # fallback tương thích ngược
        event_tag: str = "teleop"
    ):
        """Ghi nhận 1 frame đồng bộ (chu kỳ 20Hz)"""
        if not self.is_recording:
            return

        ts = int(time.time() * 1000)
        self.buffer_timestamps_ms.append(ts)
        self.buffer_q_actual.append(list(q_actual) if q_actual is not None else [0.0]*6)
        self.buffer_qd_actual.append(list(qd_actual) if qd_actual is not None else [0.0]*6)
        self.buffer_tcp_actual.append(list(tcp_actual) if tcp_actual is not None else [0.0]*7)
        self.buffer_gripper_actual.append(float(gripper_actual))

        # Target actions
        if q_target is not None:
            self.buffer_q_target.append(list(q_target))
        else:
            self.buffer_q_target.append(list(q_actual) if q_actual is not None else [0.0]*6)
        self.buffer_gripper_target.append(float(gripper_target))

        # Images (RGB numpy array uint8 [H, W, 3])
        if images:
            for cam_name, img_array in images.items():
                if cam_name not in self.buffer_images:
                    self.buffer_images[cam_name] = []
                self.buffer_images[cam_name].append(img_array)

    def end_episode(self, success: bool = True) -> Optional[str]:
        """Kết thúc và xuất file HDF5 chuẩn ACT / Diffusion Policy"""
        if not self.is_recording or len(self.buffer_timestamps_ms) == 0:
            self.is_recording = False
            return None

        self.is_recording = False
        os.makedirs(self.output_dir, exist_ok=True)
        filename = f"episode_{self.current_episode_idx}.hdf5"
        filepath = os.path.join(self.output_dir, filename)

        T = len(self.buffer_timestamps_ms)
        qpos_arr = np.array(self.buffer_q_actual, dtype=np.float32)
        qvel_arr = np.array(self.buffer_qd_actual, dtype=np.float32)
        tcp_arr = np.array(self.buffer_tcp_actual, dtype=np.float32)
        grip_arr = np.array(self.buffer_gripper_actual, dtype=np.float32).reshape(-1, 1)

        q_target_arr = np.array(self.buffer_q_target, dtype=np.float32)
        grip_target_arr = np.array(self.buffer_gripper_target, dtype=np.float32).reshape(-1, 1)
        action_arr = np.hstack([q_target_arr, grip_target_arr])  # Shape [T, 7]

        with h5py.File(filepath, "w") as f:
            # Metadata
            f.attrs["hz"] = self.hz
            f.attrs["num_steps"] = T
            f.attrs["success"] = success
            f.attrs["task_description"] = getattr(self, "task_description", "Pick and Place")
            f.attrs["duration_sec"] = time.time() - self.episode_start_time

            # Group: observations
            obs_grp = f.create_group("observations")
            obs_grp.create_dataset("qpos", data=qpos_arr, compression="gzip")
            obs_grp.create_dataset("qvel", data=qvel_arr, compression="gzip")
            obs_grp.create_dataset("tcp_pose", data=tcp_arr, compression="gzip")
            obs_grp.create_dataset("gripper", data=grip_arr, compression="gzip")
            obs_grp.create_dataset("timestamps_ms", data=np.array(self.buffer_timestamps_ms, dtype=np.int64))

            # Images
            img_grp = obs_grp.create_group("images")
            for cam_name, img_list in self.buffer_images.items():
                if img_list:
                    img_stack = np.stack(img_list, axis=0) # [T, H, W, 3]
                    img_grp.create_dataset(cam_name, data=img_stack, compression="gzip")

            # Group: action
            f.create_dataset("action", data=action_arr, compression="gzip")

        print(f"[AI DataLogger] ĐÃ LƯU Episode {self.current_episode_idx} ({T} frames, success={success}) -> {filepath}")
        self.current_episode_idx += 1
        self._reset_buffer()
        return filepath

    def discard_episode(self):
        """Hủy bỏ episode nếu thao tác lỗi/rơi đồ"""
        print(f"[AI DataLogger] ĐÃ HỦY Episode {self.current_episode_idx} ({len(self.buffer_timestamps_ms)} frames)")
        self.is_recording = False
        self._reset_buffer()

    def flush(self):
        """Hàm tương thích ngược với code cũ: tự động kết thúc episode"""
        if self.is_recording and len(self.buffer_timestamps_ms) > 0:
            self.end_episode(success=True)
