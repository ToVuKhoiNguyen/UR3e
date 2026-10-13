#!/usr/bin/env python3
"""
Script kiểm chứng Vấn đề 3: Bộ bù trừ chu kỳ lấy mẫu 20Hz (Rate-compensated timer)
Đo đạc phân bố thời gian chu kỳ (dt) và tần số (Hz) dưới tải tính toán mô phỏng.
"""

import time
import numpy as np

def run_loop_benchmark(use_compensation: bool, num_steps: int = 50, target_dt: float = 0.05):
    """
    Chạy vòng lặp mô phỏng chu kỳ điều khiển:
    - Mỗi bước có tải tính toán (GPU IK + Render ~ 12ms)
    """
    intervals = []
    
    for _ in range(num_steps):
        t0 = time.perf_counter()
        
        # Giả lập thời gian tính toán thực tế của CuRobo IK + OpenCV Render (10ms - 15ms)
        time.sleep(0.012)
        
        # Cơ chế ngủ
        if use_compensation:
            elapsed = time.perf_counter() - t0
            sleep_duration = max(0.001, target_dt - elapsed)
            time.sleep(sleep_duration)
        else:
            # Cơ chế cũ: Ngủ cố định không bù trừ
            time.sleep(target_dt)
            
        t_total = time.perf_counter() - t0
        intervals.append(t_total)
        
    intervals_ms = np.array(intervals) * 1000.0
    mean_dt = np.mean(intervals_ms)
    std_dt = np.std(intervals_ms)
    mean_hz = 1000.0 / mean_dt
    
    return mean_dt, std_dt, mean_hz

def main():
    print("=" * 60)
    print("KIỂM CHỨNG VẤN ĐỀ 3: ỔN ĐỊNH CHU KỲ LẤY MẪU 20Hz")
    print("=" * 60)
    
    # 1. Chạy với cơ chế cũ (Không bù trừ)
    print("\n1. Đo đạc cơ chế CŨ (time.sleep(0.05) cố định):")
    dt_old, std_old, hz_old = run_loop_benchmark(use_compensation=False, num_steps=60)
    print(f" -> Chu kỳ dt trung bình: {dt_old:.2f} ms")
    print(f" -> Tần số thực tế:       {hz_old:.2f} Hz  (Kỳ vọng là 20.0 Hz)")
    print(f" -> Nhận xét: Bị trôi và tụt tần số nghiêm trọng xuống ~{hz_old:.1f}Hz do cộng dồn thời gian tính toán!")

    # 2. Chạy với cơ chế MỚI (Có bù trừ time.perf_counter())
    print("\n2. Đo đạc cơ chế MỚI (Bù trừ thời gian trễ thực tế):")
    dt_new, std_new, hz_new = run_loop_benchmark(use_compensation=True, num_steps=60)
    print(f" -> Chu kỳ dt trung bình: {dt_new:.2f} ms")
    print(f" -> Độ lệch chuẩn std:    {std_new:.2f} ms")
    print(f" -> Tần số thực tế:       {hz_new:.2f} Hz  (Kỳ vọng: 20.0 ± 0.5 Hz)")

    # 3. Đánh giá tiêu chuẩn
    assert abs(hz_new - 20.0) <= 0.6, f"Lỗi: Tần số mới {hz_new:.2f}Hz lệch quá mức cho phép!"
    assert std_new <= 2.5, f"Lỗi: Độ lệch chuẩn {std_new:.2f}ms quá lớn!"
    print("\n" + "=" * 60)
    print(f"✅ KIỂM CHỨNG THÀNH CÔNG! Tần số đạt {hz_new:.2f}Hz, độ lệch chuẩn chỉ {std_new:.2f}ms.")
    print("=" * 60)

if __name__ == "__main__":
    main()
