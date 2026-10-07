import time
import threading
import numpy as np
import torch

# ─────────────────────────────────────────────────────────────────────────────
# MODULE 1: PoseSource
# ─────────────────────────────────────────────────────────────────────────────
# Input:  Viser TransformControls (hiện tại) / WebSocket (tương lai VR)
# Output: dict { "pos": [X,Y,Z], "quat": [Qw,Qx,Qy,Qz] } hoặc None nếu không đổi
#
class GizmoPoseSource:
    """
    Wrapper quanh Viser TransformControls.
    Để sau này swap sang VR/WebSocket: tạo class WebSocketPoseSource với get_pose() tương tự.
    """
    def __init__(self, transform_controls):
        self._tf = transform_controls
        self._last_ts = None

    def get_pose(self):
        """
        Output: dict { "pos": [X,Y,Z], "quat": [Qw,Qx,Qy,Qz] }
                hoặc None nếu pose không thay đổi từ lần gọi trước.
        """
        current_ts = self._tf.update_timestamp
        if current_ts == self._last_ts:
            return None
        self._last_ts = current_ts
        return {
            "pos":  self._tf.position.tolist(),
            "quat": self._tf.wxyz.tolist(),
        }

    def get_current_pose(self):
        """Luôn trả về pose hiện tại dù không thay đổi. Dùng để init."""
        return {
            "pos":  self._tf.position.tolist(),
            "quat": self._tf.wxyz.tolist(),
        }

# ─────────────────────────────────────────────────────────────────────────────
# MODULE 2A: PyRoki IK Solver
# ─────────────────────────────────────────────────────────────────────────────
# Input:  pose = { "pos": [X,Y,Z], "quat": [Qw,Qx,Qy,Qz] }
#         q_real = [6 góc khớp hiện tại - Radian] (dùng làm warm-start)
# Output: { "q": [6 Radian] hoặc None, "latency_ms": float, "solver": "pyroki" }
#
class PyRokiIKSolver:
    def __init__(self, pk_robot, solve_fn, check_error_fn, target_link_index, err_thresh=0.005):
        self._robot = pk_robot
        self._solve = solve_fn
        self._check = check_error_fn
        self._link_idx = target_link_index
        self._err_thresh = err_thresh

    def solve(self, pose, q_real):
        import jax.numpy as jnp
        t0 = time.perf_counter()
        try:
            q_sol = self._solve(
                jnp.array(pose["pos"]),
                jnp.array(pose["quat"]),
                jnp.array(q_real)
            )
            latency_ms = (time.perf_counter() - t0) * 1000
            if np.isnan(np.array(q_sol)).any():
                return {"q": None, "latency_ms": latency_ms, "solver": "pyroki"}
            err = float(self._check(q_sol, jnp.array(pose["pos"])))
            if err > self._err_thresh:
                return {"q": None, "latency_ms": latency_ms, "solver": "pyroki"}
            return {"q": np.array(q_sol).tolist(), "latency_ms": latency_ms, "solver": "pyroki"}
        except Exception:
            return {"q": None, "latency_ms": (time.perf_counter() - t0) * 1000, "solver": "pyroki"}


# ─────────────────────────────────────────────────────────────────────────────
# MODULE 2B: CuRobo IK Solver
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


# ─────────────────────────────────────────────────────────────────────────────
# MODULE 3: SafetyGate
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
# MODULE 4: ServoJ Executor  (dùng RTDEControlInterface - ĐÚNG cách)
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
    ROBOT_IP = "127.0.0.1"

    def __init__(self, dt=0.05, lookahead_time=0.04, gain=1000):
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
# MODULE 5: TeleoperationController (Điều phối tổng)
# ─────────────────────────────────────────────────────────────────────────────
# Giao diện với main loop:
#   - teleop.enabled = True/False  (toggle từ Web button)
#   - teleop.step(q_real)          (gọi mỗi vòng lặp 20Hz)
#   - teleop.get_status()          → string hiển thị lên UI
#   - teleop.get_latency()         → dict latency PyRoki + CuRobo
#
class TeleoperationController:
    def __init__(self, pose_source, pyroki_solver, curobo_solver,
                 safety_gate, executor, dt=0.05):
        self._pose   = pose_source
        self._pyroki = pyroki_solver
        self._curobo = curobo_solver
        self._gate   = safety_gate
        self._exec   = executor
        self._dt     = dt

        self._enabled = False
        self._last_valid_q = None
        self._status  = "⏸ Teleop tắt"
        self._latency = {"pyroki_ms": 0.0, "curobo_ms": 0.0}
        self._lock = threading.Lock()

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        """Bật/tắt Teleop: tự động mở/đóng RTDEControl."""
        if value and not self._enabled:
            self._exec.connect()
            self._status = "⏳ Chờ tín hiệu tay cầm..."
        elif not value and self._enabled:
            self._exec.disconnect()
            self._status = "⏸ Teleop tắt"
        self._enabled = value

    def get_status(self):
        with self._lock:
            return self._status

    def get_latency(self):
        with self._lock:
            return dict(self._latency)

    def step(self, q_real):
        """
        Một vòng lặp 20Hz. Gọi từ main loop của interactive_obstacle_test.py.

        Input:  q_real list[6 Radian] - góc khớp thực tế từ RTDE Port 30004
        Output: None (side-effect: gọi servoJ hoặc đứng im)

        Luồng xử lý:
          1. Lấy pose từ PoseSource (None nếu không đổi → bỏ qua)
          2. Giải IK bằng PyRoki (warm-start từ q_real) và CuRobo song song
          3. Ưu tiên PyRoki, fallback CuRobo nếu PyRoki NaN
          4. Kiểm tra SafetyGate
          5. Gọi RTDEControl.servoJ nếu an toàn
        """
        if not self._enabled:
            return

        # ── Lấy pose mới từ PoseSource ──────────────────────────────────────
        # Output: dict { "pos":[X,Y,Z], "quat":[Qw,Qx,Qy,Qz] } hoặc None
        pose = self._pose.get_pose()
        if pose is None:
            return  # pose không đổi → không gửi lệnh

        # ── Giải IK ────────────────────────────────────────────────
        # Dùng lệnh cuối cùng (last_valid_q) làm mốc (warm-start) thay vì q_real.
        # Lý do: q_real (robot vật lý) thường bị trễ so với q_target. Nếu so với q_real, 
        # khoảng cách sẽ bị giãn ra liên tục dẫn đến lỗi "nhảy vọt góc khớp" giả.
        q_seed = self._last_valid_q if self._last_valid_q is not None else q_real
        
        result_pk = self._pyroki.solve(pose, q_seed)
        result_cu = self._curobo.solve(pose, q_seed)

        with self._lock:
            self._latency["pyroki_ms"] = result_pk["latency_ms"]
            self._latency["curobo_ms"] = result_cu["latency_ms"]

        # ── Chọn q_target: Chỉ dùng CuRobo (vì có tránh vật cản) ─────────────
        # Không dùng PyRoki làm fallback nữa vì PyRoki sẽ phớt lờ vật cản.
        if result_cu["q"] is not None:
            q_target = result_cu["q"]
            
            # Kiểm tra bước nhảy góc khớp (phát hiện nhảy nhánh IK do gần điểm kỳ dị Singularity)
            if self._last_valid_q is not None:
                max_jump = max(abs(q_target[i] - self._last_valid_q[i]) for i in range(6))
                if max_jump > 0.4:  # ~23 độ
                    with self._lock:
                        self._status = "Teleop: Tu the ket (Loi van xoan khop) - Dung im"
                    return
                
                # Rate Limiter: Kẹp (clamp) vận tốc khớp tối đa để bảo vệ động cơ thực
                MAX_VEL = 1.5  # rad/s (~85 độ/s - Tốc độ an toàn cho Teleop)
                DT = 0.05      # 20Hz loop
                max_dq = MAX_VEL * DT
                for i in range(6):
                    # Nếu lệnh yêu cầu quay nhanh hơn MAX_VEL, ta cắt ngọn bớt để tay máy đi mượt hơn
                    if q_target[i] - self._last_valid_q[i] > max_dq:
                        q_target[i] = self._last_valid_q[i] + max_dq
                    elif q_target[i] - self._last_valid_q[i] < -max_dq:
                        q_target[i] = self._last_valid_q[i] - max_dq
            
            chosen = "CuRobo"
        else:
            with self._lock:
                if result_pk["q"] is not None:
                    # PyRoki (không có check va chạm) vươn tới được, nhưng CuRobo từ chối -> Chắc chắn đụng vật cản!
                    self._status = "Teleop: Dung vat can - Dung im"
                else:
                    # Cả hai đều không giải được -> Nằm ngoài tầm với.
                    self._status = "Teleop: Ngoai tam voi - Dung im"
            return

        # ── Kiểm tra SafetyGate ──────────────────────────────────────────────
        gate = self._gate.check(q_target)
        if not gate["pass"]:
            with self._lock:
                self._status = f"Teleop: Dung - {gate['reason']}"
            return

        # ── Gọi RTDEControl.servoJ ───────────────────────────────────────────
        # Input:  q_target list[6 Radian] đã được xác nhận an toàn
        # Output: Robot di chuyển realtime theo lệnh servoJ
        self._exec.send(q_target)
        self._last_valid_q = q_target

        sigma = gate["sigma_min"]
        with self._lock:
            self._status = (
                f"Teleop [{chosen}] | "
                f"sigma={sigma:.3f} | "
                f"PyRoki:{result_pk['latency_ms']:.1f}ms | "
                f"CuRobo:{result_cu['latency_ms']:.1f}ms"
            )
