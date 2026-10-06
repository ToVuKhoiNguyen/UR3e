import os
import sys
import yaml
import time
import torch
from singularity_guard.guard_core import SingularityGuard

class RuntimeMonitor:
    def __init__(self, config, ctrl_instance):
        self.config = config
        self.ctrl = ctrl_instance
        self.guard = SingularityGuard(config)
        self.danger_thresh = config["thresholds"]["sigma_min_danger"]
        self.warn_thresh = config["thresholds"]["sigma_min_warn"]
        
        self.danger_detected = False
        self.danger_q = None

    def start(self):
        print("[Monitor] Đã sẵn sàng (Đồng bộ trên Main Thread).")

    def stop(self):
        pass

    def check_current_state(self):
        q_real = self.ctrl.joints()
        q_tensor = torch.tensor([q_real], device=self.guard.metrics_calc.device, dtype=torch.float32)
        metrics = self.guard.metrics_calc.get_metrics(q_tensor)
        sigma = metrics["sigma_min"].item()
        
        if sigma < self.danger_thresh:
            print(f"[Monitor] DANGER! sigma_min = {sigma:.5f} < {self.danger_thresh}. Kích hoạt Safe Return!")
            self.danger_detected = True
            self.danger_q = q_real
            return True
        return False
        
    def trigger_safe_return(self, current_q):
        from singularity_guard.safe_return import SafeReturn
        sr = SafeReturn(self.config, self.ctrl, self.guard)
        sr.execute_recovery(current_q)
