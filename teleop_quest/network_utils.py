import os
import socket
import subprocess

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = '127.0.0.1'
    finally:
        s.close()
    return local_ip

def setup_ssl(base_dir, local_ip):
    cert_path = os.path.join(base_dir, "certs", "cert.pem")
    key_path = os.path.join(base_dir, "certs", "key.pem")
    os.makedirs(os.path.join(base_dir, "certs"), exist_ok=True)
    
    ip_cache_path = os.path.join(base_dir, "certs", "ip_cache.txt")
    cached_ip = ""
    if os.path.exists(ip_cache_path):
        with open(ip_cache_path, "r") as f:
            cached_ip = f.read().strip()

    if not os.path.exists(cert_path) or cached_ip != local_ip:
        print(f"IP thay đổi (hoặc chạy lần đầu). Đang tự động tạo SSL cho IP: {local_ip}...")
        subprocess.run(
            f"openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout {key_path} -out {cert_path} -subj '/CN={local_ip}'", 
            shell=True, stderr=subprocess.DEVNULL
        )
        with open(ip_cache_path, "w") as f:
            f.write(local_ip)
            
    return cert_path, key_path
