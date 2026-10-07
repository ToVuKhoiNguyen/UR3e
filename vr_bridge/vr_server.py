import ssl
import asyncio
import websockets
import json
import threading
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler

class VRServer:
    def __init__(self, cert_file, key_file, host="0.0.0.0", http_port=8443, ws_port=8444):
        self.cert_file = cert_file
        self.key_file = key_file
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port
        self.latest_pose = None
        self._lock = threading.Lock()
        
    def start(self):
        # Start HTTP server thread
        self.http_thread = threading.Thread(target=self._run_http, daemon=True)
        self.http_thread.start()
        
        # Start WSS server thread
        self.ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        self.ws_thread.start()
        
    def get_pose(self):
        with self._lock:
            return self.latest_pose
            
    def _run_http(self):
        os.chdir(os.path.dirname(__file__))
        httpd = HTTPServer((self.host, self.http_port), SimpleHTTPRequestHandler)
        
        # Wrap with SSL
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=self.cert_file, keyfile=self.key_file)
        httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
        
        print(f"[VR] HTTPS Server running at https://{self.host}:{self.http_port}")
        httpd.serve_forever()

    async def _ws_handler(self, websocket):
        print("[VR] Kính Quest đã kết nối WebSocket!")
        try:
            async for message in websocket:
                data = json.loads(message)
                if data.get("hand") == "right":
                    with self._lock:
                        self.latest_pose = {
                            "pos": data["pos"],
                            "quat": data["quat"],
                            "trigger": data.get("trigger", False)
                        }
        except Exception as e:
            print(f"[VR] WS Error: {e}")
        finally:
            print("[VR] Kính Quest ngắt kết nối.")

    def _run_ws(self):
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(certfile=self.cert_file, keyfile=self.key_file)
        
        async def run_server():
            async with websockets.serve(self._ws_handler, self.host, self.ws_port, ssl=ssl_context):
                print(f"[VR] WSS Server running at wss://{self.host}:{self.ws_port}")
                await asyncio.Future()  # run forever
                
        asyncio.run(run_server())

if __name__ == "__main__":
    import time
    if not os.path.exists("cert.pem") or not os.path.exists("key.pem"):
        print("Lỗi: Không tìm thấy cert.pem và key.pem. Chạy lệnh openssl để tạo trước.")
        exit(1)
        
    server = VRServer("cert.pem", "key.pem")
    server.start()
    print("Đang chờ dữ liệu VR...")
    try:
        while True:
            p = server.get_pose()
            if p:
                print(f"X: {p['pos'][0]:.3f}, Y: {p['pos'][1]:.3f}, Z: {p['pos'][2]:.3f}", end="\r")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
