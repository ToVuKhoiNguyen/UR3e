import os
import time
import threading
import numpy as np
import torch

# ─────────────────────────────────────────────────────────────────────────────
# MODULE 1: CuRobo IK Solver
# ─────────────────────────────────────────────────────────────────────────────
# Input:  pose = { "pos": [X,Y,Z], "quat": [Qw,Qx,Qy,Qz] }
#         q_real = [6 góc khớp hiện tại - Radian]
# Output: { "q": [6 Radian] hoặc None, "latency_ms": float, "solver": "curobo" }
#
class CuRoboIKSolver:
    def __init__(self, ik_solver):
        self._ik = ik_solver

    def solve(self, pose, q_real):
        t0 = time.perf_counter()
        try:
            q = self._ik.solve(pose["pos"], pose["quat"], current_q=q_real)
            return {"q": q, "latency_ms": (time.perf_counter() - t0) * 1000, "solver": "curobo"}
        except Exception:
            return {"q": None, "latency_ms": (time.perf_counter() - t0) * 1000, "solver": "curobo"}

    def fk(self, q: list) -> dict:
        import torch
        from curobo.types import JointState
        js = torch.tensor([q], device="cuda", dtype=torch.float32)
        js_obj = JointState.from_position(js, joint_names=self._ik.solver.kinematics.joint_names)
        kin_state = self._ik.solver.kinematics.compute_kinematics(js_obj)
        pose = kin_state.tool_poses
        return {
            "pos": pose.position.squeeze().cpu().tolist(),
            "quat": pose.quaternion.squeeze().cpu().tolist()  # [w, x, y, z]
        }


# ─────────────────────────────────────────────────────────────────────────────
# MODULE 2: SafetyGate
# ─────────────────────────────────────────────────────────────────────────────
# Input:  q = [6 Radian] từ IK solver
# Output: { "pass": bool, "reason": str, "sigma_min": float }
#
class SafetyGate:
    WRIST_LIMIT = 2.9  # ~166 độ - giới hạn chống tự kẹp (Lớp 5)

    def __init__(self, guard):
        self._guard = guard

    def check(self, q):
        if q is None:
            return {"pass": False, "reason": "IK failed", "sigma_min": 0.0}
        # Lớp 5: Chống tự kẹp cổ tay
        if abs(q[3]) > self.WRIST_LIMIT or abs(q[4]) > self.WRIST_LIMIT:
            return {"pass": False, "reason": "wrist_clamp", "sigma_min": 0.0}
        # Lớp 1: Kiểm tra kỳ dị
        m = self._guard.metrics_calc.get_metrics(
            torch.tensor([q], device=self._guard.metrics_calc.device)
        )
        sigma = m["sigma_min"].item()
        if sigma < self._guard.danger_threshold:
            return {"pass": False, "reason": f"singularity(σ={sigma:.4f})", "sigma_min": sigma}
        return {"pass": True, "reason": "ok", "sigma_min": sigma}


# ─────────────────────────────────────────────────────────────────────────────
# MODULE 3: ServoJ Executor  (dùng RTDEControlInterface - ĐÚNG cách)
# ─────────────────────────────────────────────────────────────────────────────
# Input:  q = [6 Radian] đã qua SafetyGate
# Output: None (gọi RTDEControlInterface.servoJ trực tiếp)
#
# Lý do KHÔNG dùng send_urscript() + "def s(): servoj() end":
#   Gửi chương trình URScript mới mỗi 50ms qua port 30002 sẽ ngắt chương trình
#   cũ trước khi robot kịp thực hiện → robot đứng im.
#   RTDEControlInterface.servoJ() duy trì vòng lặp realtime trong bộ điều khiển,
#   không bị gián đoạn giữa các lần gọi.
#
class ServoJExecutor:
    ROBOT_IP = os.environ.get("ROBOT_IP", "127.0.0.1")

    def __init__(self, dt=0.05, lookahead_time=0.04, gain=800):
        self._dt = dt
        self._lookahead = lookahead_time
        self._gain = gain
        self._ctrl = None

    def connect(self):
        """Khởi tạo RTDEControlInterface. Gọi 1 lần khi bật Teleop."""
        import rtde_control
        if self._ctrl is not None:
            return
        self._ctrl = rtde_control.RTDEControlInterface(self.ROBOT_IP)
        print(f"[ServoJ] RTDEControl connected: {self._ctrl.isConnected()}")

    def disconnect(self):
        """Dừng servoJ và ngắt kết nối. Gọi 1 lần khi tắt Teleop."""
        if self._ctrl:
            try:
                self._ctrl.servoStop()
                self._ctrl.stopScript()
                self._ctrl.disconnect()
            except Exception:
                pass
            self._ctrl = None
            print("[ServoJ] RTDEControl disconnected.")

    def send(self, q):
        """
        Input:  q list[6 Radian]
        Output: None

        RTDEControlInterface.servoJ() gọi trực tiếp hàm servoj bên trong
        bộ điều khiển UR, không cần gửi script URScript qua port 30002.
        Tham số:
          speed=0, acceleration=0 → robot tự tính dựa theo lookahead_time
          dt           = chu kỳ lấy mẫu (s), khớp với vòng lặp 20Hz
          lookahead_time = nội suy mượt (0.03–0.2s)
          gain         = độ cứng bám vị trí (100–2000)
        """
        if self._ctrl is None:
            return
        try:
            # RTDEControlInterface C++ bindings only support positional arguments
            self._ctrl.servoJ(
                q,
                0.0,               # speed
                0.0,               # acceleration
                self._dt,          # dt
                self._lookahead,   # lookahead_time
                self._gain         # gain
            )
        except Exception as e:
            print(f"[ServoJ] Lỗi: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# MODULE 4: TeleoperationController (Điều phối tổng)
# ─────────────────────────────────────────────────────────────────────────────
# Giao diện với main loop:
#   - teleop.enabled = True/False  (toggle từ Web button)
#   - teleop.step(q_real)          (gọi mỗi vòng lặp 20Hz)
#   - teleop.get_status()          → string hiển thị lên UI
#   - teleop.get_latency()         → dict latency CuRobo
#
class TeleoperationController:
    def __init__(self, pose_source, curobo_solver, safety_gate, executor, dt=0.05):
        self._pose   = pose_source
        self._curobo = curobo_solver
        self._gate   = safety_gate
        self._exec   = executor
        self._dt     = dt

        self._enabled = False
        self._last_valid_q = None
        self._status  = "Đã tắt"
        self._latency = {"curobo_ms": 0.0}
        self._lock = threading.Lock()

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        """Bật/tắt Teleop: tự động mở/đóng RTDEControl."""
        if value and not self._enabled:
            self._exec.connect()
            self._status = "Chờ tín hiệu tay cầm..."
            self._last_valid_q = None
        elif not value and self._enabled:
            self._exec.disconnect()
            self._status = "Đã tắt"
        self._enabled = value

    def get_status(self):
        with self._lock:
            return self._status

    def get_latency(self):
        with self._lock:
            return dict(self._latency)

    def step(self, q_real):
        """
        Một vòng lặp 20Hz.

        Input:  q_real list[6 Radian] - góc khớp thực tế từ RTDE Port 30004
        Output: None (side-effect: gọi servoJ hoặc đứng im)

        Luồng xử lý:
          1. Lấy pose từ PoseSource (None nếu không đổi → bỏ qua)
          2. Giải IK bằng CuRobo GPU (warm-start từ last_valid_q hoặc q_real)
          3. Kiểm tra bước nhảy nhánh IK & Rate Limiter
          4. Kiểm tra SafetyGate (singularity, wrist limits)
          5. Gọi RTDEControl.servoJ nếu an toàn
        """
        if not self._enabled:
            return

        # ── Lấy pose mới từ PoseSource ──────────────────────────────────────
        pose = self._pose.get_pose()
        if pose is None:
            return  # pose không đổi → không gửi lệnh

        # ── Giải IK bằng CuRobo GPU ─────────────────────────────────────────
        q_seed = self._last_valid_q if self._last_valid_q is not None else q_real
        result_cu = self._curobo.solve(pose, q_seed)

        with self._lock:
            self._latency["curobo_ms"] = result_cu["latency_ms"]

        # ── Kiểm tra nghiệm IK CuRobo ────────────────────────────────────────
        if result_cu["q"] is not None:
            q_target = result_cu["q"]
            # Kiểm tra bước nhảy nhánh IK bất thường (phát hiện lật khớp do gần điểm kỳ dị Singularity)
            if self._last_valid_q is not None:
                max_jump = max(abs(q_target[i] - self._last_valid_q[i]) for i in range(6))
                if max_jump > 1.5:  # ~86 độ (chỉ chặn khi xảy ra đảo nhánh IK đột ngột)
                    with self._lock:
                        self._status = "Tư thế kẹt (Lỗi vặn xoắn khớp) - Dừng im"
                    return
                
                # Rate Limiter: Kẹp (clamp) vận tốc khớp tối đa để bảo vệ động cơ thực
                MAX_VEL = 2.0  # rad/s (~115 độ/s)
                DT = self._dt
                max_dq = MAX_VEL * DT
                for i in range(6):
                    if q_target[i] - self._last_valid_q[i] > max_dq:
                        q_target[i] = self._last_valid_q[i] + max_dq
                    elif q_target[i] - self._last_valid_q[i] < -max_dq:
                        q_target[i] = self._last_valid_q[i] - max_dq
        else:
            with self._lock:
                self._status = "Vật cản hoặc ngoài tầm với - Dừng im"
            return

        # ── Kiểm tra SafetyGate ──────────────────────────────────────────────
        gate = self._gate.check(q_target)
        if not gate["pass"]:
            with self._lock:
                self._status = f"Dừng - {gate['reason']}"
            return

        # ── Gọi RTDEControl.servoJ ───────────────────────────────────────────
        self._exec.send(q_target)
        self._last_valid_q = q_target

        sigma = gate["sigma_min"]
        with self._lock:
            self._status = (
                f"[CuRobo] | "
                f"sigma={sigma:.3f} | "
                f"CuRobo:{result_cu['latency_ms']:.1f}ms"
            )
