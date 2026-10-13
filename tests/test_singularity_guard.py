#!/usr/bin/env python3
"""
Unit test kiem tra SingularityGuard va tinh toan Jacobian metrics (sigma_min).
"""

import os
import sys
import yaml
import torch
import numpy as np

# Them thu muc goc vao sys.path
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from safety.singularity_guard.guard_core import SingularityGuard

def test_singularity_guard():
    print("=" * 60)
    print("KIEM TRA SINGULARITY GUARD VA JACOBIAN METRICS")
    print("=" * 60)

    config_path = os.path.join(BASE_DIR, "config", "singularity.yaml")
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    guard = SingularityGuard(config)
    print("-> Da khoi tao SingularityGuard thanh cong.")

    # 1. Tu the Home an toan (canh tay gap tu nhien, xa diem ky di)
    safe_q = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    safe_tensor = torch.tensor([safe_q], device=guard.metrics_calc.device, dtype=torch.float32)
    safe_metrics = guard.metrics_calc.get_metrics(safe_tensor)
    safe_sigma = safe_metrics["sigma_min"].item()
    print(f"-> Tu the Home an toan: sigma_min = {safe_sigma:.4f} (Nguong nguy hiem: {guard.danger_threshold})")
    assert safe_sigma > guard.danger_threshold, "Tu the Home phai nam trong vung an toan!"

    # 2. Tu the duoi thang (Wrist singularity gan q5 ~ 0)
    singular_q = [0.0, -1.5708, 0.0, -1.5708, 0.005, 0.0]
    sing_tensor = torch.tensor([singular_q], device=guard.metrics_calc.device, dtype=torch.float32)
    sing_metrics = guard.metrics_calc.get_metrics(sing_tensor)
    sing_sigma = sing_metrics["sigma_min"].item()
    print(f"-> Tu the gan ky di co tay: sigma_min = {sing_sigma:.4f}")
    assert sing_sigma < guard.warn_threshold, "Tu the thang phai co sigma_min thap!"

    # 3. Kiem tra ham check_trajectory
    traj = [
        [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        [0.05, -1.55, 1.55, -1.57, -1.57, 0.0]
    ]
    is_safe, reason = guard.check_trajectory(traj, dt=0.05)
    print(f"-> Kiem tra quy dao an toan: is_safe={is_safe}, reason='{reason}'")
    assert is_safe, "Quy dao ngan muot ma phai duoc chap thuan!"

    print("=" * 60)
    print("TEST HOAN TAT THANH CONG!")
    print("=" * 60)

if __name__ == "__main__":
    test_singularity_guard()
