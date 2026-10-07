"""
runtime_monitor_v2.py - Phiên bản sửa lỗi của RuntimeMonitor.

Thay đổi so với v1:
- Thêm trạng thái is_moving: khi robot đang thực thi quỹ đạo an toàn, 
  Monitor sẽ dừng can thiệp (tránh cắt giữa chừng gây robot đứng ngắc).
- Ngưỡng phát hiện nguy hiểm (danger_threshold) được giảm nhẹ để tránh false-positive
  khi robot đang đi qua vùng gần nhưng không thực sự kẹt.
"""

import time
import torch
from safety.singularity_guard.guard_core import SingularityGuard


class RuntimeMonitor:
    def __init__(self, config, ctrl_instance):
        self.config = config
        self.ctrl = ctrl_instance
        self.guard = SingularityGuard(config)
        self.danger_thresh = config["thresholds"]["sigma_min_danger"]

        self.danger_detected = False
        self.danger_q = None

        # Khi is_moving = True, Monitor sẽ không can thiệp
        # Caller phải set is_moving = True trước khi gửi URScript,
        # và set về False sau khi robot đến nơi.
        self.is_moving = False

    def start(self):
        print("[Monitor] Đã sẵn sàng (Đồng bộ trên Main Thread).")

    def stop(self):
        pass

    def check_current_state(self) -> bool:
        """Trả về True nếu phát hiện nguy hiểm VÀ robot không đang di chuyển theo kế hoạch."""
        if self.is_moving:
            # Không can thiệp khi robot đang thực hiện quỹ đạo đã lên kế hoạch sẵn
            return False

        try:
            q_real = self.ctrl.joints()
        except Exception:
            return False

        q_tensor = torch.tensor(
            [q_real],
            device=self.guard.metrics_calc.device,
            dtype=torch.float32
        )
        metrics = self.guard.metrics_calc.get_metrics(q_tensor)
        sigma = metrics["sigma_min"].item()

        if sigma < self.danger_thresh:
            print(
                f"[Monitor] DANGER! sigma_min = {sigma:.5f} < {self.danger_thresh}. "
                f"Kích hoạt Safe Return!"
            )
            self.danger_detected = True
            self.danger_q = q_real
            return True

        return False

    def trigger_safe_return(self, current_q):
        """Kích hoạt SafeReturn. is_moving tự động được set trong suốt quá trình."""
        from safety.safe_return import SafeReturn
        # Đánh dấu đang di chuyển để các lần check tiếp không can thiệp
        self.is_moving = True
        try:
            sr = SafeReturn(self.config, self.ctrl, self.guard)
            sr.execute_recovery(current_q)
        finally:
            # Dù thành công hay thất bại, reset cờ di chuyển
            self.is_moving = False
