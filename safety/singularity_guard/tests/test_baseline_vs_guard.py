import os
import sys
import torch
import math
import yaml
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__)))) # curobo_ursim
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__))) # singularity_guard

from curobo_ursim_control import CuRoboURSim
from guard_core import SingularityGuard
from data_logger import DataLogger

def load_config():
    path = os.path.join(os.path.dirname(__file__), "..", "..", "config", "singularity.yaml")
    with open(path, "r") as f:
        return yaml.safe_load(f)

def run_test():
    print("=== TEST: Baseline vs Guard (Cartesian Straight Line near Singularity) ===")
    config = load_config()
    guard = SingularityGuard(config)
    logger = DataLogger(config)
    
    ctrl = CuRoboURSim()
    ik = ctrl.ik_solver
    
    # Đưa robot về vị trí gần điểm kỳ dị cổ tay (q5 ~ 0)
    # Tư thế chuẩn để q5=0 (tay thẳng hoặc cổ tay thẳng)
    # Ví dụ: TCP hướng ra trước mặt, các khớp tạo thành đường thẳng.
    # q = [0, -pi/2, 0, -pi/2, 0, 0]
    
    start_q = [0.0, -1.57, 0.0, -1.57, 0.01, 0.0]
    print("Đưa robot về vị trí bắt đầu...")
    ctrl.move_joints(start_q, speed=0.5)
    time.sleep(2.0)
    
    tcp_start = ctrl.tcp()
    
    # Tạo quỹ đạo thẳng (dọc trục y) qua singularity
    # Di chuyển Y từ y_start đến y_start + 0.3
    num_steps = 20
    dt = 0.1
    waypoints = []
    
    pos_seq = []
    
    for i in range(num_steps):
        t = i / (num_steps - 1)
        y_new = tcp_start[1] + (t * 0.15) # Quét qua Y
        
        pos = [tcp_start[0], y_new, tcp_start[2]]
        quat = [1.0, 0.0, 0.0, 0.0] # Giữ nguyên orientation
        
        q_sol = ik.solve(pos, quat, current_q=start_q if len(waypoints) == 0 else waypoints[-1])
        if q_sol is not None:
            waypoints.append(q_sol)
            pos_seq.append(pos)
        else:
            print(f"IK failed at step {i}")
            break
            
    print(f"\n--- 1. Kiểm tra quỹ đạo bằng Guard (Trước khi chạy) ---")
    is_safe, reason = guard.check_trajectory(waypoints, dt=dt)
    
    print(f"Bản thân quỹ đạo Cartesian stream tạo ra:")
    print(f"Safe: {is_safe}")
    print(f"Reason: {reason}")
    
    if not is_safe:
        print("\n✅ Guard đã hoạt động đúng: Từ chối quỹ đạo nguy hiểm!")
    else:
        print("\n❌ Guard KHÔNG từ chối quỹ đạo. Vui lòng xem lại config hoặc start_q.")
        
    print("\n--- 2. Thực thi Baseline (Cố tình chạy để xem có bị Protective Stop không) ---")
    print("Vui lòng xem trên URSim/Polyscope xem có báo lỗi không.")
    
    # Stream manually
    for i, q in enumerate(waypoints):
        # Tính qd để log
        if i > 0:
            qd = [(q[j] - waypoints[i-1][j])/dt for j in range(6)]
        else:
            qd = [0]*6
            
        metrics = guard.metrics_calc.get_metrics(torch.tensor([q], device=guard.metrics_calc.device))
        sigma = metrics["sigma_min"].item()
        
        flag = "danger" if sigma < guard.danger_threshold else ("warn" if sigma < guard.warn_threshold else "none")
        
        logger.log_step(
            q=q, qd=qd, qdd=[0]*6, tcp_pose=pos_seq[i] + [0,0,0], 
            sigma_min=sigma, w=metrics["manipulability"].item(), cond=metrics["condition_number"].item(),
            singularity_flag=flag, cmd_vs_actual_q=ctrl.joints(), 
            safety_status="OK", event_tag="baseline_test"
        )
        
        ctrl.move_joints(q, speed=1.0, wait=True)
        # Giả lập stream với dt=0.1
        # Thực tế lệnh trên sẽ mất nhiều thời gian hơn 0.1 vì accel/decel
        
    logger.flush()
    ctrl.close()

if __name__ == "__main__":
    run_test()
