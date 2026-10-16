"""
cuRobo v0.8 -> URSim UR3e Controller
======================================
Pipeline:
  cuRobo GPU (IK) -> best joint solution -> RTDE servoJ / URScript -> URSim
  RTDEReceive (port 30004) -> real-time state monitoring
"""

import os
import math
import socket
import time
from typing import Optional

import torch

# -- cuRobo v0.8 API ---------------------------------------------------------
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, Pose, JointState

# -- ur-rtde -----------------------------------------------------------------
import rtde_receive

# ===========================================================================
# Configuration
def get_robot_ip() -> str:
    return os.environ.get("ROBOT_IP", "127.0.0.1")

_BASE_DIR       = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_CONFIG    = os.path.join(_BASE_DIR, "assets", "ur3e_custom.yml")
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
    (-2 * math.pi, 2 * math.pi),  # shoulder_pan_joint   — ±360°
    (-math.pi, 0.05),             # shoulder_lift_joint  — [-180°, ~3°] vươn tự nhiên, chống chúi lật bàn
    (-2.80, 2.80),                # elbow_joint          — Cho phép gập sâu tự nhiên (-160° đến 160°), CuRobo tự quản lý va chạm
    (-2 * math.pi, 2 * math.pi),  # wrist_1_joint        — Cho phép quay tự nhiên tránh đảo nhánh IK gây vặn xoắn
    (-2 * math.pi, 2 * math.pi),  # wrist_2_joint        — Cho phép quay tự nhiên tránh đảo nhánh IK gây vặn xoắn
    (-4 * math.pi, 4 * math.pi),  # wrist_3_joint        — ±720°
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
            return False
    return True


# ===========================================================================
# URSim helper functions
# ===========================================================================

def setup_ursim(timeout: float = 35.0) -> bool:
    """Wait for URControl to be ready, then power-on robot and release brakes via Dashboard. Returns True when RUNNING."""
    start_time = time.time()
    print("[URSim] Đang kiểm tra trạng thái robot qua Dashboard (Port 29999)...")
    print("[URSim] Giao diện Teach Pendant PolyScope (noVNC): http://192.168.56.101:6080/vnc.html")

    while time.time() - start_time < timeout:
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((get_robot_ip(), DASHBOARD_PORT))
            time.sleep(0.3)
            try:
                s.recv(4096)
            except Exception:
                pass

            def _cmd(c, w=0.5):
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
            safety = _cmd("safetystatus", 0.3)
            print(f"[URSim] Trạng thái: Mode={mode}, Safety={safety}")

            if "RUNNING" in mode:
                # Dừng script cũ đang chạy trên robot để giải phóng thanh ghi RTDE
                _cmd("stop", 0.2)
                s.close()
                return True

            if "NO_CONTROLLER" in mode or not mode:
                print("[URSim] URControl đang khởi động, đợi 3 giây...")
                s.close()
                time.sleep(3.0)
                continue

            if any(err in safety for err in ["PROTECTIVE_STOP", "VIOLATION", "FAULT"]):
                print("[URSim] Đang mở khoá Protective Stop / Safety popup...")
                _cmd("unlock protective stop", 1.0)
                _cmd("close safety popup", 1.0)
                time.sleep(1.0)
                mode = _cmd("robotmode", 0.3)

            if "POWER_OFF" in mode or "IDLE" in mode or "DISCONNECTED" in mode:
                print("[URSim] Đang bật nguồn (power on) và nhả phanh (brake release)...")
                _cmd("power on", w=2.5)
                _cmd("brake release", w=4.0)
                mode = _cmd("robotmode", 0.5)
                print(f"[URSim] Trạng thái sau kích hoạt: {mode}")

            s.close()
            if "RUNNING" in mode:
                return True
        except Exception as e:
            if s:
                try:
                    s.close()
                except Exception:
                    pass
            print(f"[URSim] Đang kết nối Dashboard ({e})... Thử lại sau 2 giây.")
            time.sleep(2.0)

    print("[URSim] Lỗi: Hết thời gian chờ URSim chuyển sang trạng thái RUNNING.")
    return False


def send_urscript(script: str, settle: float = 0.3) -> None:
    """Upload URScript via secondary interface (port 30002). No Remote Control mode needed."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((get_robot_ip(), SECONDARY_PORT))
    time.sleep(0.2)
    s.recv(4096)
    s.sendall(script.encode())
    time.sleep(settle)
    s.close()


def movej_script(q: list, a=JOINT_ACCEL, v=JOINT_SPEED, r=BLEND_RADIUS) -> str:
    joints = ", ".join(f"{x:.6f}" for x in q)
    return f"movej([{joints}], a={a}, v={v}, r={r})\n"


# ===========================================================================
# cuRobo v0.8 IK Solver
# ===========================================================================

class CuRoboIK:
    """
    GPU-accelerated IK using cuRobo v0.8.

    Solves return_seeds solutions in parallel and returns the one requiring
    the least joint travel from the current robot state.
    """

    def __init__(self, num_seeds: int = 64, return_seeds: int = 16, scene_file: Optional[str] = None):
        print(f"[cuRobo] Initializing IK solver for UR3e (scene: {scene_file})...")
        if scene_file:
            scene_path = os.path.abspath(scene_file)
            self.cfg = InverseKinematicsCfg.create(
                robot=ROBOT_CONFIG,
                scene_model=scene_path,
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

            max_diff = max(abs(norm_q[j] - ref_q[j]) for j in range(6))
            # Ưu tiên các nghiệm cùng nhánh động học liên tục (tránh đảo nhánh vặn xoắn cổ tay)
            branch_penalty = 50.0 if max_diff > 0.8 else 0.0
            dist = sum(abs(norm_q[j] - ref_q[j]) for j in range(6)) + branch_penalty
            if dist < best_dist:
                best_dist, best = dist, norm_q

        if skipped > 0 and best is None:
            print(f"[IK] All {skipped} solutions out of safe range — pose unreachable safely")

        return best


# ===========================================================================
# High-Level Controller
# ===========================================================================

class CuRoboURSim:
    """
    cuRobo GPU IK -> URSim via URScript / RTDE.

    Architecture:
        cuRobo IK  (GPU, returns N seeds)
             |
             | pick closest solution to current joints + normalise angles
             v
        RTDEControl / URScript -> URSim UR3e
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
        self.rr = rtde_receive.RTDEReceiveInterface(get_robot_ip())
        q = self.rr.getActualQ()
        print(f"[Setup] Connected. Joints: {[round(j, 4) for j in q]}")

        self._ik = None
        self.scene_file = scene_file
        print("=" * 56 + "\n")

    @property
    def ik_solver(self) -> CuRoboIK:
        if self._ik is None:
            self._ik = CuRoboIK(scene_file=self.scene_file)
        return self._ik

    # -- State ---------------------------------------------------------------

    def joints(self) -> list:
        """Current joint positions [rad] x 6."""
        return self.rr.getActualQ()

    def tcp(self) -> list:
        """Current TCP pose [x, y, z, rx, ry, rz]."""
        return self.rr.getActualTCPPose()

    def close(self):
        self.rr.disconnect()
        print("[Setup] Disconnected")

