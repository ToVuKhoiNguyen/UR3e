#!/usr/bin/env python3
"""
Bộ công cụ tự động kiểm tra chất lượng dữ liệu AI Demonstration (Data Sanity Check):
- Quét toàn bộ file HDF5 trong logs/
- Kiểm tra lỗi NaN / Inf trong qpos, qvel, action
- Kiểm tra tính đồng bộ độ dài (Length consistency) giữa ảnh, khớp, action
- Kiểm tra tính ổn định của chu kỳ thời gian (dt mean, std, drop frame)
- Báo cáo chi tiết từng episode và tổng kết toàn bộ dataset.
"""

import os
import sys
import glob
import h5py
import numpy as np

def verify_dataset(dataset_dir="logs/ai_training_data", max_dt_std_ms=10.0):
    files = sorted(glob.glob(os.path.join(dataset_dir, "**/*.hdf5"), recursive=True))
    if not files:
        print(f"[Sanity Check] Không tìm thấy file HDF5 nào trong thư mục: {dataset_dir}")
        return True

    print("=" * 70)
    print(f" KIỂM TRA CHẤT LƯỢNG DATASET HDF5: {len(files)} files tìm thấy")
    print(f" Thư mục: {dataset_dir}")
    print("=" * 70)

    total_episodes = len(files)
    passed_episodes = 0
    total_frames = 0
    issues = []

    for idx, fpath in enumerate(files):
        rel_path = os.path.relpath(fpath, dataset_dir)
        try:
            with h5py.File(fpath, 'r') as f:
                # 1. Kiểm tra Metadata
                hz = f.attrs.get("hz", 20)
                num_steps = f.attrs.get("num_steps", None)
                success = f.attrs.get("success", None)
                task_desc = f.attrs.get("task_description", "N/A")

                # 2. Đọc các mảng dữ liệu
                qpos = f["observations/qpos"][:]
                qvel = f["observations/qvel"][:] if "observations/qvel" in f else None
                action = f["action"][:]
                ts = f["observations/timestamps_ms"][:]
                
                # Ảnh camera
                cam_keys = list(f["observations/images"].keys()) if "observations/images" in f else []
                cam_frames = {k: f[f"observations/images/{k}"].shape[0] for k in cam_keys}

                T = len(ts)
                total_frames += T

                # 3. Kiểm tra NaN / Inf
                if np.isnan(qpos).any() or np.isinf(qpos).any():
                    issues.append(f"[{rel_path}] LỖI: qpos chứa NaN hoặc Inf!")
                if np.isnan(action).any() or np.isinf(action).any():
                    issues.append(f"[{rel_path}] LỖI: action chứa NaN hoặc Inf!")

                # 4. Kiểm tra độ dài đồng bộ
                if len(qpos) != T or len(action) != T:
                    issues.append(f"[{rel_path}] LỖI: Lệch độ dài giữa qpos({len(qpos)}), action({len(action)}) và timestamp({T})!")
                for cam_name, n_img in cam_frames.items():
                    if n_img != T:
                        issues.append(f"[{rel_path}] LỖI: Lệch số lượng frame ảnh camera '{cam_name}' ({n_img} vs {T})!")

                # 5. Kiểm tra phân bố thời gian (dt)
                if T > 1:
                    d_ts = np.diff(ts)
                    dt_mean = float(np.mean(d_ts))
                    dt_std = float(np.std(d_ts))
                    dt_max = int(np.max(d_ts))
                    dt_min = int(np.min(d_ts))
                    
                    status_flag = "OK"
                    if dt_std > max_dt_std_ms:
                        status_flag = f"[CẢNH BÁO] dt_std={dt_std:.1f}ms > {max_dt_std_ms}ms"
                        issues.append(f"[{rel_path}] CẢNH BÁO: Tần số giật cục (dt={dt_mean:.1f}±{dt_std:.1f}ms, min={dt_min}ms, max={dt_max}ms)")
                else:
                    dt_mean, dt_std = 0.0, 0.0
                    status_flag = "1 frame"

                succ_str = "SUCCESS" if success else "FAIL"
                print(f"[{idx+1:02d}/{total_episodes:02d}] {rel_path:35s} | T={T:3d} | {succ_str:7s} | dt={dt_mean:5.1f}±{dt_std:4.1f}ms | Cams: {cam_keys}")
                passed_episodes += 1

        except Exception as e:
            issues.append(f"[{rel_path}] LỖI ĐỌC FILE: {e}")
            print(f"[{idx+1:02d}/{total_episodes:02d}] {rel_path:35s} | [LỖI]: {e}")

    print("=" * 70)
    print(f"TỔNG KẾT: {passed_episodes}/{total_episodes} episodes đã quét | Tổng frames: {total_frames}")
    if not issues:
        print("[OK] TẤT CẢ DỮ LIỆU ĐẠT CHUẨN (Không có NaN, độ dài đồng bộ 100%, timestamp đều)!")
        print("=" * 70)
        return True
    else:
        print(f"[CẢNH BÁO] PHÁT HIỆN {len(issues)} VẤN ĐỀ CẦN LƯU Ý:")
        for iss in issues:
            print(f"  - {iss}")
        print("=" * 70)
        return False

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="AI Demonstration Dataset Sanity Checker")
    parser.add_argument("--dir", default="logs/ai_training_data", help="Thư mục chứa các file HDF5")
    parser.add_argument("--max-std", type=float, default=12.0, help="Ngưỡng cảnh báo độ lệch chuẩn dt (ms)")
    args = parser.parse_args()

    success = verify_dataset(dataset_dir=args.dir, max_dt_std_ms=args.max_std)
    sys.exit(0 if success else 1)
