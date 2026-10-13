#!/usr/bin/env python3
"""
Script tái lập (Replay) quỹ đạo Demonstration từ file HDF5:
- Hỗ trợ chế độ Dry-Run (Kiểm tra quỹ đạo offline không cần bật robot)
- Hỗ trợ chế độ Live Replay (Gửi lệnh servoJ / moveJ sang URSim hoặc UR3e thực tế)
- Đo đạc sai lệch theo dõi (Tracking error) giữa lệnh điều khiển và trạng thái thực tế.
"""

import os
import sys
import time
import argparse
import h5py
import numpy as np

def replay_episode(hdf5_path: str, execute_robot: bool = False, robot_ip: str = "127.0.0.1"):
    if not os.path.exists(hdf5_path):
        print(f"[Replay] Lỗi: Không tìm thấy file {hdf5_path}")
        return False

    print("=" * 65)
    print(f" TÁI LẬP QUỸ ĐẠO DEMONSTRATION: {os.path.basename(hdf5_path)}")
    print("=" * 65)

    with h5py.File(hdf5_path, "r") as f:
        hz = f.attrs.get("hz", 20)
        T = f.attrs.get("num_steps", len(f["observations/timestamps_ms"]))
        success = f.attrs.get("success", "Unknown")
        task_desc = f.attrs.get("task_description", "N/A")
        
        qpos = f["observations/qpos"][:]       # [T, 6]
        action = f["action"][:]               # [T, 7]
        grip = f["observations/gripper"][:]   # [T, 1]

        print(f"-> Thông tin Episode: T={T} frames | Tần số={hz}Hz | Thành công={success}")
        print(f"-> Nhiệm vụ: {task_desc}")
        print(f"-> Dải góc khớp ban đầu: {[round(float(x), 3) for x in qpos[0]]}")
        print(f"-> Dải góc khớp kết thúc: {[round(float(x), 3) for x in qpos[-1]]}")

    dt = 1.0 / hz if hz > 0 else 0.05

    # 1. Kiểm tra tính liên tục của quỹ đạo (Trajectory Continuity & Max Velocity)
    if T > 1:
        dq = np.diff(qpos, axis=0) # [T-1, 6]
        max_joint_jump = np.max(np.abs(dq))
        max_joint_vel = max_joint_jump / dt # rad/s
        print(f"\n[Kiểm định Quỹ đạo Offline]")
        print(f"-> Bước nhảy góc khớp lớn nhất: {max_joint_jump:.4f} rad (~{np.degrees(max_joint_jump):.1f}°)")
        print(f"-> Vận tốc khớp ước lượng tối đa: {max_joint_vel:.2f} rad/s")
        if max_joint_vel > 2.0:
            print(f"[CẢNH BÁO] Vận tốc khớp vượt 2.0 rad/s (Có thể gây giật robot thật)!")
        else:
            print("[OK] Quỹ đạo mượt mà, nằm trong giới hạn an toàn.")

    # 2. Chạy trên robot / URSim nếu yêu cầu
    if execute_robot:
        print(f"\n[Kết nối Robot / URSim tại {robot_ip}]")
        try:
            import rtde_control, rtde_receive
            rc = rtde_control.RTDEControlInterface(robot_ip)
            rr = rtde_receive.RTDEReceiveInterface(robot_ip)
            print("-> Đã kết nối RTDE thành công.")

            # Di chuyển mượt về vị trí bắt đầu
            start_q = qpos[0].tolist()
            print("-> Đang di chuyển robot về vị trí bắt đầu...")
            rc.moveJ(start_q, 0.5, 0.5)
            time.sleep(1.0)

            print(f"-> Bắt đầu Replay {T} frames theo servoJ ({hz}Hz)...")
            tracking_errors = []

            for i in range(T):
                t_step_start = time.perf_counter()
                target_q = action[i, :6].tolist()
                
                # servoJ(q, speed=0, accel=0, time=dt, lookahead_time=0.1, gain=500)
                rc.servoJ(target_q, 0.0, 0.0, dt, 0.1, 500)
                
                # Đo sai lệch thực tế
                actual_q = rr.getActualQ()
                err = np.linalg.norm(np.array(target_q) - np.array(actual_q))
                tracking_errors.append(err)

                elapsed = time.perf_counter() - t_step_start
                sleep_t = max(0.001, dt - elapsed)
                time.sleep(sleep_t)

            rc.servoStop()
            rc.disconnect()
            rr.disconnect()

            avg_err = np.mean(tracking_errors)
            max_err = np.max(tracking_errors)
            print(f"\n[OK] REPLAY HOÀN TẤT TRÊN ROBOT!")
            print(f"-> Sai lệch bám vị trí trung bình: {avg_err*1000:.2f} mm/mrad")
            print(f"-> Sai lệch lớn nhất:              {max_err*1000:.2f} mm/mrad")

        except Exception as e:
            print(f"[LỖI] Không thể kết nối hoặc thực thi trên robot ({e}).")
            print("Gợi ý: Đảm bảo URSim container đang chạy và Dashboard ở chế độ RUNNING.")
            return False
    else:
        print("\n[Chế độ Dry-Run Offline: Không phát lệnh ra robot thật]")
        print("-> Để chạy thực tế trên URSim, thêm cờ: --execute")

    print("=" * 65)
    return True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Demonstration Replay Tool")
    parser.add_argument("--file", default="logs/ai_training_data/test_grasp_sim/episode_1.hdf5", help="Đường dẫn file HDF5")
    parser.add_argument("--execute", action="store_true", help="Gửi lệnh thực thi ra URSim / Robot thật")
    parser.add_argument("--ip", default="127.0.0.1", help="Địa chỉ IP của robot / URSim")
    args = parser.parse_args()

    replay_episode(hdf5_path=args.file, execute_robot=args.execute, robot_ip=args.ip)
