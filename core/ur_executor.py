import math
import time

class URExecutor:
    @staticmethod
    def build_urscript_traj(waypoints, speed=0.8, accel=1.5):
        """Đổi quỹ đạo waypoints thành chuỗi lệnh movej của URScript"""
        script = "def traj():\n"
        for i, q in enumerate(waypoints):
            if i < len(waypoints) - 1:
                q_next = waypoints[i+1]
                dist = math.sqrt(sum((a - b)**2 for a, b in zip(q, q_next)))
                r = min(0.02, dist * 0.2) # ép r tối đa 2cm
            else:
                r = 0.0
            j = ", ".join(f"{x:.6f}" for x in q)
            script += f"  movej([{j}], a={accel}, v={speed}, r={r})\n"
        script += "end\ntraj()\n"
        return script
