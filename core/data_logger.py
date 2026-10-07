import os
import time
import pandas as pd

class DataLogger:
    def __init__(self, config):
        self.output_dir = config["logging"].get("output_dir", "logs/singularity_data")
        os.makedirs(self.output_dir, exist_ok=True)
        self.current_run_file = os.path.join(self.output_dir, f"run_{int(time.time())}.parquet")
        self.data_buffer = []

    def log_step(self, q, qd, qdd, tcp_pose, sigma_min, w, cond, singularity_flag, cmd_vs_actual_q, safety_status, event_tag):
        # Flatten everything into a single dict
        row = {
            "timestamp": time.time(),
            "event_tag": event_tag,
            "singularity_flag": singularity_flag,
            "safety_status": safety_status,
            "sigma_min": float(sigma_min),
            "manipulability": float(w),
            "cond": float(cond),
        }
        
        for i in range(6):
            row[f"q_{i}"] = float(q[i])
            if qd is not None:
                row[f"qd_{i}"] = float(qd[i])
            if qdd is not None:
                row[f"qdd_{i}"] = float(qdd[i])
            if cmd_vs_actual_q is not None:
                row[f"cmd_q_{i}"] = float(cmd_vs_actual_q[i])
                
        if tcp_pose is not None:
            for i, axis in enumerate(['x', 'y', 'z', 'rx', 'ry', 'rz']):
                if i < len(tcp_pose):
                    row[f"tcp_{axis}"] = float(tcp_pose[i])
                
        self.data_buffer.append(row)
        
        # Auto save every 1000 steps
        if len(self.data_buffer) >= 1000:
            self.flush()

    def flush(self):
        if not self.data_buffer:
            return
            
        df = pd.DataFrame(self.data_buffer)
        
        # Append to parquet if exists
        if os.path.exists(self.current_run_file):
            existing_df = pd.read_parquet(self.current_run_file)
            df = pd.concat([existing_df, df], ignore_index=True)
            
        df.to_parquet(self.current_run_file, index=False)
        self.data_buffer = []
        print(f"[Logger] Đã ghi log ra {self.current_run_file}")
