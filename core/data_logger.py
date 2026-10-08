import os
import time
import pandas as pd

class DataLogger:
    def __init__(self, config):
        self.output_dir = config.get("output_dir", "logs/ai_training_data")
        os.makedirs(self.output_dir, exist_ok=True)
        # Tạo folder con cho từng session
        session_id = f"session_{int(time.time())}"
        self.session_dir = os.path.join(self.output_dir, session_id)
        os.makedirs(self.session_dir, exist_ok=True)
        self.images_dir = os.path.join(self.session_dir, "images")
        os.makedirs(self.images_dir, exist_ok=True)
        
        self.current_run_file = os.path.join(self.session_dir, "robot_states.csv")
        self.data_buffer = []
        
        # Bật/tắt cờ ghi
        self.is_recording = False

    def log_step(self, q, qd, tcp_pose, image_path=None, event_tag="teleop"):
        if not self.is_recording:
            return
            
        row = {
            "timestamp_ms": int(time.time() * 1000),
            "event_tag": event_tag,
            "image_path": image_path if image_path else "None",
        }
        
        for i in range(6):
            row[f"q_{i}"] = float(q[i]) if q is not None else 0.0
            row[f"qd_{i}"] = float(qd[i]) if qd is not None else 0.0
                
        if tcp_pose is not None:
            # Lưu [x, y, z, q_w, q_x, q_y, q_z]
            for i, axis in enumerate(['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']):
                if i < len(tcp_pose):
                    row[f"tcp_{axis}"] = float(tcp_pose[i])
                
        self.data_buffer.append(row)
        
        # Auto save every 100 steps (5 seconds at 20Hz)
        if len(self.data_buffer) >= 100:
            self.flush()

    def flush(self):
        if not self.data_buffer:
            return
            
        df = pd.DataFrame(self.data_buffer)
        
        # Append to CSV if exists
        if os.path.exists(self.current_run_file):
            df.to_csv(self.current_run_file, mode='a', header=False, index=False)
        else:
            df.to_csv(self.current_run_file, mode='w', header=True, index=False)
            
        self.data_buffer = []
        print(f"[AI Logger] Đã ghi {len(df)} frames vào {self.current_run_file}")
