#!/usr/bin/env python3
"""
Interactive cuRobo + URSim control via Viser UI.
Uses a SEPARATE native viser TransformControls (/target) for drag input,
while ViserVisualizer's tool0 frame updates the robot mesh visualization.

KEY DESIGN NOTE:
  - Must use /curobo_target (not tool0!) because viz.set_joint_state() resets
    tool0 position to actual robot FK every loop, overwriting user's drag.
  - Polling handle.position works; on_update callback does NOT fire reliably.
"""

import sys, os, math, time, subprocess
sys.path.insert(0, os.path.dirname(__file__))

import torch
import viser
from curobo_ursim_control import CuRoboURSim
from curobo.types import ContentPath, JointState
from curobo.viewer import ViserVisualizer

from singularity_guard.guard_core import SingularityGuard
from singularity_guard.runtime_monitor import RuntimeMonitor
from singularity_guard.data_logger import DataLogger
import yaml

def load_config():
    path = os.path.join(os.path.dirname(__file__), "config", "singularity.yaml")
    with open(path, "r") as f:
        return yaml.safe_load(f)

# ─── Config ───────────────────────────────────────────────────────────────────
VISER_PORT  = 8080
MOVE_SPEED  = 0.7
MOVE_ACCEL  = 1.2
POLL_SLEEP  = 0.1   # 10 Hz
POS_THRESH  = 0.005 # 5mm

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
]

def main():
    print("\n=== cuRobo + URSim Interactive Control ===")

    # ── 1. Robot ──────────────────────────────────────────────────────────────
    ctrl = CuRoboURSim()
    ik   = ctrl.ik_solver  # pre-init on main thread

    # ── 1.5 Singularity Guard ─────────────────────────────────────────────────
    config = load_config()
    guard = SingularityGuard(config)
    logger = DataLogger(config)
    monitor = RuntimeMonitor(config, ctrl)

    # ── 2. Free port & start ViserVisualizer ──────────────────────────────────
    subprocess.run("fuser -k 8080/tcp 2>/dev/null", shell=True)
    time.sleep(0.5)

    viz = ViserVisualizer(
        content_path=ContentPath(robot_config_file="ur3e.yml"),
        connect_ip="0.0.0.0",
        connect_port=VISER_PORT,
    )

    # ── 3. Add a SEPARATE target frame using native viser ─────────────────────
    # (NOT tool0 — that gets reset by set_joint_state every loop!)
    server: viser.ViserServer = viz._server

    # Get initial TCP position to place target frame there
    tcp = ctrl.tcp()  # [x, y, z, rx, ry, rz]
    init_pos = (tcp[0], tcp[1], tcp[2])

    target_tf = server.scene.add_transform_controls(
        "/curobo_target",
        position=init_pos,
        wxyz=(1.0, 0.0, 0.0, 0.0),
        scale=0.18,
        disable_sliders=True,
    )

    print(f"\n{'='*55}")
    print(f" >>> READY! Open http://localhost:{VISER_PORT} <<<")
    print(f" Drag the GREEN axis frame to move the robot!")
    print(f"{'='*55}\n")

    last_pos  = list(init_pos)
    last_quat = [1.0, 0.0, 0.0, 0.0]

    # Khởi động Monitor sau khi tất cả CUDA graph đã warmup xong
    monitor.start()

    try:
        while True:
            if monitor.check_current_state():
                monitor.trigger_safe_return(monitor.danger_q)
                monitor.danger_detected = False
                continue
                
            # A. Update robot mesh from real URSim joint state
            q = ctrl.joints()
            js = JointState.from_position(
                torch.tensor([q], dtype=torch.float32, device="cuda"),
                joint_names=JOINT_NAMES
            )
            viz.set_joint_state(js)

            # B. Poll the INDEPENDENT target frame (not affected by set_joint_state)
            pos  = target_tf.position.tolist()
            quat = target_tf.wxyz.tolist()

            dist  = math.sqrt(sum((pos[i]  - last_pos[i])**2  for i in range(3)))
            qdist = math.sqrt(sum((quat[i] - last_quat[i])**2 for i in range(4)))

            if dist > POS_THRESH or qdist > 0.01:
                print(f"[UI] Drag → pos={[round(v,3) for v in pos]}, dist={dist*100:.1f}cm")

                q_sol = ik.solve(pos, quat, current_q=q)
                if q_sol is not None:
                    # Tự động kẹp vận tốc để "lách" qua singularity
                    q_safe, is_clamped = guard.clamp_step(q, q_sol, dt=POLL_SLEEP)
                    
                    # Compute exact metrics for logging
                    m = guard.metrics_calc.get_metrics(torch.tensor([q_safe], device=guard.metrics_calc.device))
                    sigma = m["sigma_min"].item()
                    w = m["manipulability"].item()
                    cond = m["condition_number"].item()
                    
                    ctrl.move_joints(q_safe, speed=MOVE_SPEED,
                                     accel=MOVE_ACCEL, wait=False)
                                     
                    if is_clamped:
                        print(f"[IK] ⚠️ Cảnh báo tốc độ cao! Đã tự bẻ khớp lách qua (sigma={sigma:.4f})")
                        logger.log_step(q_safe, None, None, pos, sigma, w, cond, "danger", q, "CLAMPED", "ik_stream")
                    else:
                        print(f"[IK] movej sent ✓")
                        logger.log_step(q_safe, None, None, pos, sigma, w, cond, "warn" if sigma < guard.warn_threshold else "none", q, "OK", "ik_stream")
                else:
                    print(f"[IK] Pose unreachable")

                last_pos, last_quat = pos, quat

            time.sleep(POLL_SLEEP)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        monitor.stop()
        ctrl.close()

if __name__ == "__main__":
    main()
