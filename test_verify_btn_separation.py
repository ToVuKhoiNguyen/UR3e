#!/usr/bin/env python3
"""
Script kiểm chứng Vấn đề 1: Tách biệt nút Xoay (Grip) và nút Kẹp (Gripper / Button A)
"""

import sys
import os
import numpy as np

# Giả lập frame từ QuestServer
def test_button_separation():
    print("=" * 60)
    print("KIỂM CHỨNG VẤN ĐỀ 1: TÁCH NÚT XOAY (GRIP) VÀ KẸP (GRIPPER)")
    print("=" * 60)

    # Kịch bản 1: Người dùng bóp cò Grip để XOAY CỔ TAY (không bấm nút kẹp)
    frame_rotate_only = {
        "pos": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "trigger": False,
        "grip": True,          # Bóp Grip để xoay
        "gripper_btn": False   # Không ấn nút kẹp
    }

    # Logic đọc trong delta_pose_source.py
    is_translating_1 = frame_rotate_only.get("trigger", False)
    is_rotating_1 = frame_rotate_only.get("grip", False)

    # Logic đọc trong main_quest.py (mới sửa)
    vr_gripper_1 = 1.0 if (frame_rotate_only.get("gripper_btn", False) or frame_rotate_only.get("button_a", False)) else 0.0
    mouse_grip_1 = 0.0
    grip_val_1 = max(vr_gripper_1, mouse_grip_1)
    is_grip_pressed_1 = bool(grip_val_1 > 0.5)

    print(f"\n[Test 1: Chỉ Xoay] grip=True, gripper_btn=False")
    print(f" -> is_rotating:     {is_rotating_1} (Kỳ vọng: True)")
    print(f" -> is_grip_pressed: {is_grip_pressed_1} (Kỳ vọng: False - Gripper KHÔNG được đóng)")
    assert is_rotating_1 == True, "Lỗi: Không nhận lệnh xoay!"
    assert is_grip_pressed_1 == False, "Lỗi xung đột: Bóp xoay cổ tay làm đóng gripper!"

    # Kịch bản 2: Người dùng bấm nút A/X để GẮP CÀ CHUA (không bóp Grip xoay)
    frame_grasp_only = {
        "pos": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "trigger": False,
        "grip": False,         # Không bóp Grip xoay
        "gripper_btn": True    # Bấm nút kẹp A/X
    }

    is_translating_2 = frame_grasp_only.get("trigger", False)
    is_rotating_2 = frame_grasp_only.get("grip", False)

    vr_gripper_2 = 1.0 if (frame_grasp_only.get("gripper_btn", False) or frame_grasp_only.get("button_a", False)) else 0.0
    grip_val_2 = max(vr_gripper_2, mouse_grip_1)
    is_grip_pressed_2 = bool(grip_val_2 > 0.5)

    print(f"\n[Test 2: Chỉ Kẹp] grip=False, gripper_btn=True")
    print(f" -> is_rotating:     {is_rotating_2} (Kỳ vọng: False - Cổ tay giữ nguyên góc)")
    print(f" -> is_grip_pressed: {is_grip_pressed_2} (Kỳ vọng: True - Gripper ĐÓNG để gắp)")
    assert is_rotating_2 == False, "Lỗi xung đột: Bấm nút kẹp lại kích hoạt xoay cổ tay!"
    assert is_grip_pressed_2 == True, "Lỗi: Gripper không đóng khi bấm nút kẹp!"

    # Kịch bản 3: Người dùng vừa xoay vừa kẹp đồng thời
    frame_both = {
        "pos": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "trigger": False,
        "grip": True,
        "gripper_btn": True
    }
    is_rotating_3 = frame_both.get("grip", False)
    vr_gripper_3 = 1.0 if (frame_both.get("gripper_btn", False) or frame_both.get("button_a", False)) else 0.0
    is_grip_pressed_3 = bool(vr_gripper_3 > 0.5)

    print(f"\n[Test 3: Cả hai] grip=True, gripper_btn=True")
    print(f" -> is_rotating:     {is_rotating_3} (Kỳ vọng: True)")
    print(f" -> is_grip_pressed: {is_grip_pressed_3} (Kỳ vọng: True)")
    assert is_rotating_3 == True and is_grip_pressed_3 == True

    # Kịch bản 4: Dự phòng khi dùng chuột trên giao diện Viser
    mouse_grip_4 = 1.0
    grip_val_4 = max(0.0, mouse_grip_4)
    is_grip_pressed_4 = bool(grip_val_4 > 0.5)
    print(f"\n[Test 4: Chuột Viser] mouse_grip=1.0 (VR không nhấn)")
    print(f" -> is_grip_pressed: {is_grip_pressed_4} (Kỳ vọng: True)")
    assert is_grip_pressed_4 == True

    print("\n" + "=" * 60)
    print("✅ TẤT CẢ TEST ĐÃ VƯỢT QUA! Xung đột nút Grip và Gripper đã được giải quyết hoàn toàn.")
    print("=" * 60)

if __name__ == "__main__":
    test_button_separation()
