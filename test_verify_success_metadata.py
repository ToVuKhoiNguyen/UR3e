#!/usr/bin/env python3
"""
Script kiểm chứng Vấn đề 4: Tự động gắn nhãn Success và bổ sung Metadata vào HDF5
"""

import os
import h5py
import numpy as np
from core.data_logger import DataLogger
from teleop_quest.grasp_simulator import GraspSimulator

class DummyServer:
    class DummyScene:
        def add_icosphere(self, *args, **kwargs): return self
        def add_box(self, *args, **kwargs): return self
    scene = DummyScene()

def test_success_and_metadata():
    print("=" * 60)
    print("KIỂM CHỨNG VẤN ĐỀ 4: NHÃN SUCCESS TỰ ĐỘNG & METADATA HDF5")
    print("=" * 60)

    test_dir = "logs/ai_training_data/test_meta_verify"
    logger = DataLogger({"output_dir": test_dir})
    server = DummyServer()
    grasp_sim = GraspSimulator(server, pick_pos=(0.20, 0.20, 0.035), place_pos=(0.15, -0.20, 0.02))

    # Kịch bản 1: Thao tác thất bại / rơi ra ngoài bàn
    init_meta = {
        "tomato_init_pos": list(grasp_sim.tomato_pos),
        "basket_pos": list(grasp_sim.place_basket_pos),
        "seed": 42
    }
    logger.start_episode(task_description="Test Failed Drop", metadata=init_meta)
    
    # Bước tiếp cận và bóp kẹp
    grasp_sim.update([0.20, 0.20, 0.035], grip_pressed=True)
    logger.log_step(q_actual=[0.0]*6, qd_actual=[0.0]*6, tcp_actual=[0.2, 0.2, 0.035, 0, 0, 0])
    
    # Thả rơi ở vị trí xa giỏ (x=0.20, y=0.20) -> DROPPED
    grasp_sim.update([0.20, 0.20, 0.035], grip_pressed=False)
    logger.log_step(q_actual=[0.0]*6, qd_actual=[0.0]*6, tcp_actual=[0.2, 0.2, 0.035, 0, 0, 0])

    ep_fail = bool(grasp_sim.placed_count > 0 and not grasp_sim.is_grasped)
    fail_file = logger.end_episode(success=ep_fail)

    print(f"\n[Kịch bản 1: Rơi ra ngoài giỏ]")
    print(f" -> placed_count: {grasp_sim.placed_count}")
    print(f" -> ep_success:   {ep_fail} (Kỳ vọng: False)")
    with h5py.File(fail_file, 'r') as f:
        saved_succ = f.attrs["success"]
        saved_meta_tomato = list(f.attrs["tomato_init_pos"])
        saved_meta_basket = list(f.attrs["basket_pos"])
        saved_seed = f.attrs["seed"]
        print(f" -> HDF5 success attr: {saved_succ} (Kỳ vọng: False)")
        print(f" -> HDF5 tomato_init:  {saved_meta_tomato}")
        print(f" -> HDF5 basket_pos:   {saved_meta_basket}")
        print(f" -> HDF5 seed:         {saved_seed}")
        assert saved_succ == False, "Lỗi: Episode rơi ngoài giỏ nhưng vẫn bị gán success=True!"
        assert np.allclose(saved_meta_tomato, [0.20, 0.20, 0.035]), "Lỗi: Sai metadata tomato_init_pos!"

    # Kịch bản 2: Gắp và thả trúng vào giỏ (x=0.15, y=-0.20)
    grasp_sim.reset()
    start_count = grasp_sim.placed_count
    logger.start_episode(task_description="Test Success Place", metadata=init_meta)

    # Gắp
    grasp_sim.update([0.20, 0.20, 0.035], grip_pressed=True)
    logger.log_step(q_actual=[0.0]*6, qd_actual=[0.0]*6, tcp_actual=[0.2, 0.2, 0.035, 0, 0, 0])
    
    # Mang đến giỏ và thả (x=0.15, y=-0.20)
    grasp_sim.update([0.15, -0.20, 0.05], grip_pressed=True)
    logger.log_step(q_actual=[0.0]*6, qd_actual=[0.0]*6, tcp_actual=[0.15, -0.2, 0.05, 0, 0, 0])
    grasp_sim.update([0.15, -0.20, 0.05], grip_pressed=False)
    logger.log_step(q_actual=[0.0]*6, qd_actual=[0.0]*6, tcp_actual=[0.15, -0.2, 0.05, 0, 0, 0])

    ep_succ = bool(grasp_sim.placed_count > start_count and not grasp_sim.is_grasped)
    succ_file = logger.end_episode(success=ep_succ)

    print(f"\n[Kịch bản 2: Đặt trúng vào giỏ]")
    print(f" -> placed_count: {grasp_sim.placed_count}")
    print(f" -> ep_success:   {ep_succ} (Kỳ vọng: True)")
    with h5py.File(succ_file, 'r') as f:
        saved_succ2 = f.attrs["success"]
        print(f" -> HDF5 success attr: {saved_succ2} (Kỳ vọng: True)")
        assert saved_succ2 == True, "Lỗi: Episode đặt trúng giỏ nhưng success lại là False!"

    print("\n" + "=" * 60)
    print("✅ KIỂM CHỨNG THÀNH CÔNG! Nhãn Success tự động và Metadata lưu chính xác 100%.")
    print("=" * 60)

if __name__ == "__main__":
    test_success_and_metadata()
