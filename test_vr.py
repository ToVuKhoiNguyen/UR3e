import os
import time
from vr_bridge.vr_server import VRServer

def main():
    print("=" * 50)
    print("  BÀI KIỂM TRA ĐỘC LẬP: KẾT NỐI KÍNH META QUEST")
    print("=" * 50)
    
    cert_path = os.path.join(os.path.dirname(__file__), "vr_bridge", "cert.pem")
    key_path = os.path.join(os.path.dirname(__file__), "vr_bridge", "key.pem")
    
    if not os.path.exists(cert_path) or not os.path.exists(key_path):
        print("[Lỗi] Không tìm thấy chứng chỉ SSL. Vui lòng tạo cert.pem và key.pem trong thư mục vr_bridge.")
        return
        
    server = VRServer(cert_path, key_path)
    server.start()
    
    print("\n[VR] Máy chủ đã khởi động thành công.")
    print("[VR] Hãy đeo kính Quest, mở web và truy cập địa chỉ HTTPS hiển thị bên trên.")
    print("[VR] Bấm Enter VR, sau đó thử BÓP CÒ (Trigger) trên tay cầm Phải.")
    print("Đang chờ dữ liệu...\n")
    
    last_trigger = False
    
    try:
        while True:
            pose = server.get_pose()
            if pose:
                trigger = pose.get("trigger", False)
                if trigger:
                    pos = pose["pos"]
                    quat = pose["quat"]
                    print(f"\r[BÓP CÒ] Tọa độ: X={pos[0]:.3f}, Y={pos[1]:.3f}, Z={pos[2]:.3f} | Xoay: W={quat[0]:.3f}, X={quat[1]:.3f}, Y={quat[2]:.3f}, Z={quat[3]:.3f}", end="")
                    last_trigger = True
                else:
                    if last_trigger:
                        print("\r[NHẢ CÒ] Đã dừng truyền tọa độ." + " " * 80)
                        last_trigger = False
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nĐã tắt chương trình thử nghiệm.")

if __name__ == "__main__":
    main()
