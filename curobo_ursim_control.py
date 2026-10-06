#!/usr/bin/env python3
"""
cuRobo v0.8 -> URSim UR3e Controller
======================================
Pipeline:
  cuRobo GPU (IK) -> best joint solution -> URScript (port 30002) -> URSim
  RTDEReceive (port 30004) -> real-time state monitoring

Usage:
  cd ~/curobo && source .venv/bin/activate
  python3 ~/ur_ws/curobo_ursim/curobo_ursim_control.py --demo joints
  python3 ~/ur_ws/curobo_ursim/curobo_ursim_control.py --demo ik
  python3 ~/ur_ws/curobo_ursim/curobo_ursim_control.py --demo batch_ik

Key design choices:
  - Uses port 30002 (URScript secondary interface) instead of RTDEControlInterface
    so Polyscope "Remote Control" mode is NOT required.
  - IK returns multiple seeds (return_seeds=5) then picks the solution closest
    to the current joint state, minimising travel distance and avoiding timeouts.
  - IK joint angles are also normalised (mod 2pi) to the equivalent nearest
    to the current position.
"""

import math
import socket
import time
from typing import Optional

import torch

# -- cuRobo v0.8 API ---------------------------------------------------------
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, Pose, JointState

# -- ur-rtde -----------------------------------------------------------------
import rtde_receive

# ===========================================================================
# Configuration
# ===========================================================================
ROBOT_IP        = "127.0.0.1"
ROBOT_CONFIG    = "/home/nguyen/ur_ws/curobo_ursim/ur3e_custom.yml"
TCP_LINK        = "tool0"          # End-effector link defined in ur3e.yml

SECONDARY_PORT  = 30002            # URScript secondary interface
DASHBOARD_PORT  = 29999            # Dashboard server

JOINT_ACCEL     = 0.8              # rad/s^2
JOINT_SPEED     = 0.5              # rad/s
BLEND_RADIUS    = 0.0              # 0 = stop at each waypoint

IK_RETURN_SEEDS = 5                # Number of IK solutions to evaluate

# UR3e joint limits — CONSERVATIVE bounds (±π) to avoid protective stops.
# URSim's soft limits trigger protective stop for large joint angles even when
# within URDF range (±2π). Clamping to ±180° keeps motion safe and predictable.
UR3E_LIMITS = [
    (-math.pi, math.pi),  # shoulder_pan_joint   — ±180°
    (-math.pi, math.pi),  # shoulder_lift_joint  — ±180°
    (-math.pi, math.pi),  # elbow_joint          — hard limit in UR3e
    (-math.pi, math.pi),  # wrist_1_joint        — ±180°
    (-math.pi, math.pi),  # wrist_2_joint        — ±180°
    (-math.pi, math.pi),  # wrist_3_joint        — ±180°
]


# ===========================================================================
# Joint normalisation utilities
# ===========================================================================

def _nearest_equivalent(angle: float, current: float, lo: float, hi: float) -> float:
    """Return the angle equivalent (mod 2*pi) that is within [lo, hi] and
    closest to *current*. Falls back to the raw angle if no equivalent fits."""
    best, best_dist = angle, float("inf")
    for k in range(-3, 4):
        candidate = angle + k * 2 * math.pi
        if lo <= candidate <= hi:
            d = abs(candidate - current)
            if d < best_dist:
                best_dist, best = d, candidate
    return best


def normalize_to_current(q_ik: list, q_current: list) -> list:
    """
    For each joint, replace the raw IK angle with the equivalent angle that
    is within the UR3e safe limits and closest to the current joint position.
    This minimises total joint travel and prevents unnecessary wrap-arounds.
    """
    return [
        _nearest_equivalent(q_ik[i], q_current[i], *UR3E_LIMITS[i])
        for i in range(6)
    ]


def validate_joints(q: list) -> bool:
    """Return True if all joints are within UR3E_LIMITS. Rejects unsafe solutions."""
    for i, (lo, hi) in enumerate(UR3E_LIMITS):
        if not (lo <= q[i] <= hi):
            print(f"[Validate] Joint {i} = {q[i]:.4f} out of safe range [{lo:.2f}, {hi:.2f}]")
            return False
    return True


# ===========================================================================
# URSim helper functions
# ===========================================================================

def setup_ursim() -> bool:
    """Power-on robot if POWER_OFF (brake release via dashboard). Returns True when RUNNING."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ROBOT_IP, DASHBOARD_PORT))
    time.sleep(0.2)
    s.recv(4096)

    def _cmd(c, w=0.8):
        s.sendall((c + "\n").encode())
        time.sleep(w)
        try:
            s.setblocking(False)
            r = s.recv(4096).decode().strip()
            s.setblocking(True)
        except Exception:
            s.setblocking(True)
            r = ""
        return r

    mode = _cmd("robotmode", 0.3)
    print(f"[URSim] Robot mode: {mode}")

    if "POWER_OFF" in mode or "IDLE" in mode:
        print("[URSim] Releasing brakes...")
        _cmd("brake release", w=5.0)
        mode = _cmd("robotmode", 0.5)
        print(f"[URSim] Mode after: {mode}")

    s.close()
    return "RUNNING" in mode


def send_urscript(script: str, settle: float = 0.3) -> None:
    """Upload URScript via secondary interface (port 30002). No Remote Control mode needed."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ROBOT_IP, SECONDARY_PORT))
    time.sleep(0.2)
    s.recv(4096)
    s.sendall(script.encode())
    time.sleep(settle)
    s.close()


def movej_script(q: list, a=JOINT_ACCEL, v=JOINT_SPEED, r=BLEND_RADIUS) -> str:
    joints = ", ".join(f"{x:.6f}" for x in q)
    return f"movej([{joints}], a={a}, v={v}, r={r})\n"


def wait_for_move(
    rr: rtde_receive.RTDEReceiveInterface,
    target_q: list,
    timeout: float = 20.0,
    tol: float = 0.01,
) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if max(abs(rr.getActualQ()[i] - target_q[i]) for i in range(6)) < tol:
            return True
        time.sleep(0.05)
    return False


# ===========================================================================
# cuRobo v0.8 IK Solver
# ===========================================================================

class CuRoboIK:
    """
    GPU-accelerated IK using cuRobo v0.8.

    Solves return_seeds solutions in parallel and returns the one requiring
    the least joint travel from the current robot state.
    """

    def __init__(self, num_seeds: int = 20, return_seeds: int = IK_RETURN_SEEDS, scene_file: Optional[str] = None):
        print(f"[cuRobo] Initializing IK solver for UR3e (scene: {scene_file})...")
        if scene_file:
            self.cfg = InverseKinematicsCfg.create(
                robot=ROBOT_CONFIG,
                scene_model=scene_file,
                num_seeds=num_seeds,
            )
        else:
            self.cfg = InverseKinematicsCfg.create(
                robot=ROBOT_CONFIG,
                num_seeds=num_seeds,
            )
        self.solver     = InverseKinematics(self.cfg)
        self.n_seeds    = return_seeds
        print("[cuRobo] IK solver ready")

    def solve(
        self,
        position: list,
        quaternion: list,
        current_q: Optional[list] = None,
    ) -> Optional[list]:
        """
        Solve IK. Returns joint angles [rad]*6 closest to current_q, or None.

        Args:
            position:   [x, y, z] metres in robot base frame
            quaternion: [w, x, y, z]
            current_q:  Current joint angles — used to minimise travel
        """
        pos_t  = torch.tensor([position],   dtype=torch.float32, device="cuda")
        quat_t = torch.tensor([quaternion], dtype=torch.float32, device="cuda")
        goal   = GoalToolPose.from_poses(
            {TCP_LINK: Pose(position=pos_t, quaternion=quat_t)},
            num_goalset=1,
        )
        
        # Pass current_state to CuRobo IK so it penalizes branch jumping
        ref_q = current_q if current_q is not None else [0.0] * 6
        js = JointState.from_position(
            torch.tensor([ref_q], dtype=torch.float32, device="cuda")
        )
        
        result = self.solver.solve_pose(goal, current_state=js, return_seeds=self.n_seeds)

        # result.solution: [batch=1, n_seeds, dof=6]
        # result.success:  [batch=1, n_seeds]
        sols   = result.solution[0]    # [n_seeds, 6]
        mask   = result.success[0]     # [n_seeds]

        if not mask.any():
            return None

        # Pick valid solution with minimum joint travel from current_q (if provided)
        best, best_dist = None, float("inf")
        ref_q = current_q if current_q is not None else [0.0] * 6
        skipped = 0

        for i in range(sols.shape[0]):
            if not mask[i]:
                continue
            raw_q  = sols[i].cpu().tolist()
            norm_q = normalize_to_current(raw_q, ref_q)

            # Reject solutions outside safe joint limits (prevents protective stops)
            if not validate_joints(norm_q):
                skipped += 1
                continue

            dist = sum(abs(norm_q[j] - ref_q[j]) for j in range(6))
            if dist < best_dist:
                best_dist, best = dist, norm_q

        if skipped > 0 and best is None:
            print(f"[IK] All {skipped} solutions out of safe range — pose unreachable safely")

        return best


# ===========================================================================
# cuRobo v0.8 Motion Planner
# ===========================================================================

class CuRoboMP:
    """GPU-accelerated Motion Planner using cuRobo v0.8."""

    def __init__(self, scene_file: Optional[str] = None):
        print(f"[cuRobo] Initializing Motion Planner for UR3e (scene: {scene_file})...")
        if scene_file:
            self.cfg = MotionPlannerCfg.create(robot=ROBOT_CONFIG, scene_model=scene_file)
        else:
            self.cfg = MotionPlannerCfg.create(robot=ROBOT_CONFIG)
            
        self.planner = MotionPlanner(self.cfg)
        self.planner.warmup(enable_graph=True, num_warmup_iterations=2)
        print("[cuRobo] Motion Planner ready")

    def solve(self, position: list, quaternion: list, current_q: list) -> Optional[list]:
        """
        Solve Motion Planning. Returns list of waypoints (each is [rad]*6), or None.
        """
        pos_t = torch.tensor([[[[[position[0], position[1], position[2]]]]]], dtype=torch.float32, device="cuda")
        quat_t = torch.tensor([[[[[quaternion[0], quaternion[1], quaternion[2], quaternion[3]]]]]], dtype=torch.float32, device="cuda")

        goal = GoalToolPose(
            tool_frames=[TCP_LINK],
            position=pos_t,
            quaternion=quat_t
        )

        q_start = JointState.from_position(
            torch.tensor([current_q], dtype=torch.float32, device="cuda"),
            joint_names=self.planner.joint_names
        )

        result = self.planner.plan_pose(goal, q_start)

        if result is not None and result.success.any():
            interp = result.get_interpolated_plan()
            traj = interp.position.squeeze().cpu().tolist()
            if len(traj) == 6 and not isinstance(traj[0], list):
                traj = [traj]
            return traj
        return None


# ===========================================================================
# High-Level Controller
# ===========================================================================

class CuRoboURSim:
    """
    cuRobo GPU IK -> URSim via URScript.

    Architecture:
        cuRobo IK  (GPU, returns N seeds)
             |
             | pick closest solution to current joints + normalise angles
             v
        URScript -> port 30002 -> URSim UR3e
             |
        RTDEReceive <- joint state feedback (port 30004)
    """

    def __init__(self, scene_file: Optional[str] = None):
        print("\n" + "=" * 56)
        print("  cuRobo v0.8 -> URSim Controller")
        print("=" * 56)

        if not setup_ursim():
            raise RuntimeError("Robot not RUNNING. Check URSim container.")

        print("[Setup] Connecting RTDEReceive...")
        self.rr = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        q = self.rr.getActualQ()
        print(f"[Setup] Connected. Joints: {[round(j, 4) for j in q]}")

        self._ik = None
        self._mp = None
        self.scene_file = scene_file
        print("=" * 56 + "\n")

    @property
    def ik_solver(self) -> CuRoboIK:
        if self._ik is None:
            self._ik = CuRoboIK(scene_file=self.scene_file)
        return self._ik

    @property
    def mp_solver(self) -> CuRoboMP:
        if self._mp is None:
            self._mp = CuRoboMP(scene_file=self.scene_file)
        return self._mp

    # -- State ---------------------------------------------------------------

    def joints(self) -> list:
        """Current joint positions [rad] x 6."""
        return self.rr.getActualQ()

    def tcp(self) -> list:
        """Current TCP pose [x, y, z, rx, ry, rz]."""
        return self.rr.getActualTCPPose()

    # -- Motion --------------------------------------------------------------

    def move_joints(
        self,
        q: list,
        speed: float = JOINT_SPEED,
        accel: float = JOINT_ACCEL,
        wait: bool = True,
        timeout: Optional[float] = None,
    ) -> bool:
        """
        Move to joint config via URScript movej().
        Timeout is auto-computed from travel distance if not specified.
        """
        current_q = self.rr.getActualQ()
        travel = max(abs(q[i] - current_q[i]) for i in range(6))  # worst-case joint
        if timeout is None:
            # Safety factor 2.5x over theoretical minimum time
            timeout = max(8.0, travel / speed * 2.5)
        print(f"[Move] -> {[round(v, 4) for v in q]}  (travel={travel:.2f}rad, timeout={timeout:.0f}s)")
        send_urscript(movej_script(q, a=accel, v=speed))
        if wait:
            ok = wait_for_move(self.rr, q, timeout=timeout)
            print(f"[Move] {'reached' if ok else 'TIMEOUT'}")
            return ok
        return True

    def move_pose(
        self,
        position: list,
        quaternion: list,
        speed: float = JOINT_SPEED,
        accel: float = JOINT_ACCEL,
    ) -> bool:
        """
        Move to Cartesian pose via cuRobo IK (GPU, picks closest solution).

        Args:
            position:   [x, y, z] metres, robot base frame
            quaternion: [w, x, y, z]
        """
        p = [round(v, 3) for v in position]
        print(f"\n[IK] pos={p} quat={[round(v, 3) for v in quaternion]}")

        current_q = self.joints()
        q_sol = self.ik_solver.solve(position, quaternion, current_q=current_q)

        if q_sol is None:
            print("[IK] Failed — pose may be out of reach")
            return False

        travel = sum(abs(q_sol[i] - current_q[i]) for i in range(6))
        print(f"[IK] Solution (total travel {travel:.2f} rad): {[round(v, 4) for v in q_sol]}")

        # Scale timeout to actual travel distance
        move_timeout = max(10.0, travel / speed * 2.5)
        return self.move_joints(q_sol, speed=speed, accel=accel, timeout=move_timeout)

    def plan_and_move_trajectory(self, position: list, quaternion: list) -> bool:
        """Plan a collision-free trajectory to target pose and execute it."""
        p = [round(v, 3) for v in position]
        print(f"\n[MP] pos={p} quat={[round(v, 3) for v in quaternion]}")
        
        current_q = self.joints()
        waypoints = self.mp_solver.solve(position, quaternion, current_q)
        
        if waypoints is None:
            print("[MP] Failed to find collision-free path")
            return False
            
        # Subsample waypoints to avoid overwhelming URSim with dense movej commands.
        # cuRobo default interpolation is dense (e.g. dt=0.02s). 
        # We take every 8th waypoint, ensuring the final target is always included.
        sub_waypoints = waypoints[::8]
        if sub_waypoints[-1] != waypoints[-1]:
            sub_waypoints.append(waypoints[-1])
            
        print(f"[MP] Subsampled to {len(sub_waypoints)} waypoints. Sending to robot...")
        
        # Build URScript with multiple movej commands. Use a small blend radius for smooth motion.
        script = "def traj():\n"
        for i, q in enumerate(sub_waypoints):
            r = 0.01 if i < len(sub_waypoints) - 1 else 0.0
            j_str = ", ".join(f"{x:.6f}" for x in q)
            script += f"  movej([{j_str}], a=1.5, v=1.0, r={r})\n"
        script += "end\n"
        
        send_urscript(script)
        
        # Calculate a generous timeout based on number of waypoints
        timeout = max(20.0, len(sub_waypoints) * 1.5)
        
        final_q = sub_waypoints[-1]
        ok = wait_for_move(self.rr, final_q, timeout=timeout)
        print(f"[MP] {'reached' if ok else 'TIMEOUT'}")
        return ok

    def home(self, speed: float = 0.4) -> bool:
        """Move to home [0, -pi/2, pi/2, -pi/2, -pi/2, 0]."""
        PI = math.pi
        return self.move_joints([0.0, -PI/2, PI/2, -PI/2, -PI/2, 0.0], speed=speed)

    def close(self):
        self.rr.disconnect()
        print("[Setup] Disconnected")

# Demo Scenarios

def demo_joints():
    """Demo A: Joint waypoints — no IK, safest first test."""
    print("\n=== Demo A: Joint Waypoint Sequence ===")
    ctrl = CuRoboURSim()
    PI = math.pi
    wps = [
        [0.0,    -PI/2,   PI/2,  -PI/2,  -PI/2,  0.0],  # Home
        [PI/4,   -PI/2,   PI/2,  -PI/2,  -PI/2,  0.0],  # Base +45 deg
        [PI/4,   -PI/3,   PI/3,  -PI/3,  -PI/2,  0.0],  # Elbow up
        [0.0,    -PI/4,   PI/4,  -PI/4,  -PI/2,  0.0],  # Stretch
        [-PI/4,  -PI/2,   PI/2,  -PI/2,  -PI/2,  0.0],  # Base -45 deg
        [0.0,    -PI/2,   PI/2,  -PI/2,  -PI/2,  0.0],  # Home
    ]
    try:
        for i, wp in enumerate(wps):
            print(f"\n-- WP {i+1}/{len(wps)} --")
            ctrl.move_joints(wp, speed=0.5)
            time.sleep(0.2)
        print("\nJoint demo complete!")
    finally:
        ctrl.close()


def demo_ik():
    """Demo B: cuRobo GPU IK -> Cartesian moves (picks best of N solutions)."""
    print("\n=== Demo B: cuRobo IK -> Cartesian Moves ===")
    ctrl = CuRoboURSim()

    # Reachable poses for UR3e (max reach ~500mm from base)
    targets = [
        ([0.30,  0.00,  0.35], [1.0,   0.0,  0.0,  0.0 ]),  # Front-centre
        ([0.25,  0.15,  0.30], [0.707, 0.0,  0.0,  0.707]), # Left
        ([0.25, -0.15,  0.30], [0.707, 0.0,  0.0, -0.707]), # Right
        ([0.30,  0.00,  0.20], [1.0,   0.0,  0.0,  0.0 ]),  # Lower-front
        ([0.30,  0.00,  0.35], [1.0,   0.0,  0.0,  0.0 ]),  # Front again
    ]
    try:
        ctrl.home()
        time.sleep(0.5)
        for i, (pos, quat) in enumerate(targets):
            print(f"\n-- Pose {i+1}/{len(targets)} --")
            if not ctrl.move_pose(pos, quat, speed=0.4):
                print("   Skipped")
            time.sleep(0.3)
        ctrl.home()
        print("\nIK demo complete!")
    finally:
        ctrl.close()


def demo_motion_planner():
    """Demo C: cuRobo Motion Planner -> collision-free trajectories."""
    print("\n=== Demo C: cuRobo Motion Planner ===")
    ctrl = CuRoboURSim()

    # Reachable poses for UR3e
    targets = [
        ([0.30,  0.00,  0.35], [1.0,   0.0,  0.0,  0.0 ]),
        ([0.25,  0.15,  0.30], [0.707, 0.0,  0.0,  0.707]),
        ([0.25, -0.15,  0.30], [0.707, 0.0,  0.0, -0.707]),
    ]
    try:
        ctrl.home()
        time.sleep(0.5)
        for i, (pos, quat) in enumerate(targets):
            print(f"\n-- Trajectory {i+1}/{len(targets)} --")
            if not ctrl.plan_and_move_trajectory(pos, quat):
                print("   Skipped")
            time.sleep(0.3)
        ctrl.home()
        print("\nMotion Planner demo complete!")
    finally:
        ctrl.close()


def demo_obstacle():
    """Demo E: Pick & Place with Obstacle Avoidance."""
    import os
    print("\n=== Demo E: Obstacle Avoidance ===")
    
    scene_path = os.path.join(os.path.dirname(__file__), "obstacle_scene.yml")
    if not os.path.exists(scene_path):
        print(f"Error: {scene_path} not found. Please create it first.")
        return
        
    ctrl = CuRoboURSim(scene_file=scene_path)

    # Pose A (Left side of the wall)
    pos_A = [0.25, 0.15, 0.30]
    quat_A = [0.707, 0.0, 0.0, 0.707]
    
    # Pose B (Right side of the wall)
    pos_B = [0.25, -0.15, 0.30]
    quat_B = [0.707, 0.0, 0.0, -0.707]
    
    try:
        ctrl.home()
        time.sleep(0.5)
        
        print("\n--- Moving to Pick Pose A ---")
        if not ctrl.plan_and_move_trajectory(pos_A, quat_A):
            print("Failed to reach Pose A")
        time.sleep(1.0)
        
        print("\n--- Moving to Place Pose B (Avoiding Wall) ---")
        if not ctrl.plan_and_move_trajectory(pos_B, quat_B):
            print("Failed to reach Pose B")
        time.sleep(1.0)
        
        print("\n--- Returning Home ---")
        ctrl.home()
        print("\nObstacle Avoidance demo complete!")
    finally:
        ctrl.close()


def demo_batch_ik():
    """Demo D: GPU batch IK benchmark. Does NOT move the robot."""
    import itertools
    print("\n=== Demo D: cuRobo Batch IK Benchmark ===")
    ik = CuRoboIK(num_seeds=20, return_seeds=3)
    home_q = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
    quat   = [1.0, 0.0, 0.0, 0.0]

    poses  = list(itertools.product(
        [0.15, 0.20, 0.25, 0.30, 0.35],
        [-0.15, -0.10, 0.0, 0.10, 0.15],
        [0.10, 0.20, 0.30, 0.40, 0.45],
    ))

    solved, t0 = 0, time.time()
    for x, y, z in poses:
        if ik.solve([x, y, z], quat, current_q=home_q) is not None:
            solved += 1

    elapsed = time.time() - t0
    print(f"\nResults:")
    print(f"  Poses tested : {len(poses)}")
    print(f"  IK solved    : {solved} ({100*solved/len(poses):.1f}%)")
    print(f"  Total time   : {elapsed:.2f}s")
    print(f"  Per pose     : {elapsed/len(poses)*1000:.1f}ms  (GPU, sequential)")


# ===========================================================================
# Entry Point
# ===========================================================================

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="cuRobo v0.8 -> URSim UR3e",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--demo",
        choices=["joints", "ik", "motion_planner", "batch_ik", "obstacle"],
        default="joints",
        help=(
            "joints         = Direct joint waypoints (recommended first test)\n"
            "ik             = cuRobo GPU IK -> Cartesian moves\n"
            "motion_planner = cuRobo GPU Motion Planner -> collision-free trajectories\n"
            "batch_ik       = GPU IK benchmark (no robot motion)\n"
            "obstacle       = Pick & Place with Obstacle Avoidance"
        ),
    )
    args = parser.parse_args()
    {
        "joints": demo_joints, 
        "ik": demo_ik, 
        "motion_planner": demo_motion_planner,
        "batch_ik": demo_batch_ik,
        "obstacle": demo_obstacle
    }[args.demo]()
