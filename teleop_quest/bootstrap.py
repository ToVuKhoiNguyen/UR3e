#!/usr/bin/env python3
"""
Mô-đun khởi tạo môi trường và hệ thống (Bootstrap):
- Tải cấu hình an toàn từ config/singularity.yaml
- Tự động kiểm tra và khởi chạy Docker URSim nếu chạy mô phỏng
- Giám sát tiến trình ADB nền để tự động reverse cổng khi cắm cáp USB Quest
- Hiển thị thông tin kết nối và banner khởi động
"""

import os
import time
import socket
import subprocess
import threading
import yaml

def load_singularity_config(base_dir=None):
    """Tải cấu hình giám sát kỳ dị từ file YAML"""
    if base_dir is None:
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg_path = os.path.join(base_dir, "config", "singularity.yaml")
    if not os.path.exists(cfg_path):
        raise FileNotFoundError(f"Không tìm thấy file cấu hình: {cfg_path}")
    with open(cfg_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def ensure_ursim_container(robot_ip: str):
    """Kiểm tra và tự động khởi động URSim nếu chạy mô phỏng cục bộ (127.0.0.1)"""
    if robot_ip not in ("127.0.0.1", "localhost"):
        return

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect((robot_ip, 29999))
        s.close()
    except Exception:
        print("[Hệ thống] Phát hiện URSim chưa mở cổng. Đang tự động kích hoạt Docker URSim...")
        res = subprocess.run(["docker", "start", "ursim"], capture_output=True, text=True)
        if res.returncode != 0:
            subprocess.Popen(
                ["ros2", "run", "ur_client_library", "start_ursim.sh", "-m", "ur3e"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        time.sleep(3.5)

def start_adb_watcher():
    """Tự động kiểm tra và chuyển tiếp cổng ADB nền liên tục nếu kính Quest cắm cáp USB"""
    script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "setup_quest_adb.sh")
    if not os.path.exists(script_path):
        return

    def _adb_watcher():
        last_status = None
        while True:
            try:
                res = subprocess.run(["adb", "devices"], capture_output=True, text=True, timeout=2)
                out = res.stdout
                is_device = "\tdevice" in out
                is_unauth = "\tunauthorized" in out

                if is_device:
                    if last_status != "authorized":
                        res_setup = subprocess.run(["bash", script_path], capture_output=True, text=True, timeout=5)
                        if res_setup.returncode == 0:
                            print("\n" + "=" * 55)
                            print("[ADB USB] ĐÃ KẾT NỐI VÀ CHUYỂN TIẾP CỔNG THÀNH CÔNG!")
                            print("-> Hãy mở Meta Quest Browser trên kính và truy cập:")
                            print("   https://localhost:8443")
                            print("=" * 55 + "\n")
                            last_status = "authorized"
                elif is_unauth:
                    if last_status != "unauthorized":
                        print("\n[ADB CẢNH BÁO] Kính Quest đã cắm USB nhưng CHƯA CẤP QUYỀN (unauthorized)!")
                        print("-> Hãy đeo kính Meta Quest lên mắt.")
                        print("-> Bấm 'Allow USB Debugging' (Cho phép gỡ lỗi USB) trên màn hình kính!")
                        last_status = "unauthorized"
                else:
                    if last_status not in (None, "disconnected"):
                        last_status = "disconnected"
            except Exception:
                pass
            time.sleep(2.5)

    watcher_thread = threading.Thread(target=_adb_watcher, daemon=True)
    watcher_thread.start()

def print_startup_banner(local_ip: str, quest_port: int = 8443, viser_port: int = 8080):
    """In thông tin địa chỉ truy cập hệ thống"""
    print("\n" + "=" * 50)
    print(f"IP MÁY CHỦ: {local_ip}")
    print(f"-> Viser Dashboard: http://localhost:{viser_port}")
    print(f"-> VR WebXR (Wi-Fi): https://{local_ip}:{quest_port}")
    print(f"-> VR WebXR (Cáp USB): https://localhost:{quest_port}")
    print("=" * 50 + "\n")
