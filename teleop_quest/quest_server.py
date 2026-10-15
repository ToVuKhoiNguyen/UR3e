import asyncio
import http.server
import json
import os
import ssl
import threading
import time
import websockets

class QuestServer:
    def __init__(self, cert: str, key: str, host: str = "0.0.0.0", http_port: int = 8443, ws_port: int = 8444, ws_port_plain: int = 8445):
        self.cert = cert
        self.key = key
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port
        self.ws_port_plain = ws_port_plain

        self._lock = threading.Lock()
        self._frame_cond = threading.Condition(self._lock)
        self._running = True
        self._latest_frame = None
        self._last_seen = 0.0
        self._connected = False
        self._clients = set()
        self._loop = None
        self._estop_triggered = False
        self._home_requested = False
        self._latest_jpeg = None
        self._drive_mode = False

        self.static_dir = os.path.join(os.path.dirname(__file__), "static")
        self._cam_file = os.path.join(self.static_dir, "wrist_camera.jpg")

    def start(self):
        # Khởi động HTTPS server (chạy daemon thread)
        http_thread = threading.Thread(target=self._run_http, daemon=True)
        http_thread.start()

        # Khởi động WSS / WS server (chạy daemon thread)
        ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        ws_thread.start()

    def get_frame(self):
        with self._lock:
            return self._latest_frame

    @property
    def last_frame(self):
        return self.get_frame()

    def last_seen(self) -> float:
        with self._lock:
            return self._last_seen

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected
            
    def get_drive_mode(self) -> bool:
        with self._lock:
            return self._drive_mode

    def set_drive_mode(self, enabled: bool):
        """Cập nhật trạng thái Drive Mode từ hệ thống và đồng bộ sang kính Quest."""
        with self._lock:
            self._drive_mode = bool(enabled)
            clients = list(self._clients)
            loop = self._loop
        if clients and loop:
            msg = json.dumps({"type": "teleop_state", "enabled": bool(enabled)})
            for ws in clients:
                try:
                    asyncio.run_coroutine_threadsafe(ws.send(msg), loop)
                except Exception:
                    pass

    def check_and_clear_estop(self) -> bool:
        with self._lock:
            if self._estop_triggered:
                self._estop_triggered = False
                return True
            return False

    def check_and_clear_home(self) -> bool:
        with self._lock:
            if self._home_requested:
                self._home_requested = False
                return True
            return False

    def send_haptic(self, intensity: float = 1.0, duration: float = 100):
        with self._lock:
            clients = list(self._clients)
            loop = self._loop
        if clients and loop:
            msg = json.dumps({"type": "haptic", "intensity": intensity, "duration": duration})
            for ws in clients:
                try:
                    asyncio.run_coroutine_threadsafe(ws.send(msg), loop)
                except Exception:
                    pass

    def broadcast_robot_telemetry(self, pos, quat, status: str = "OK", gripper: bool = False, joints: list = None):
        """Phát tọa độ TCP thực tế, quaternion và trạng thái robot tới WebXR Cockpit HUD."""
        with self._lock:
            clients = list(self._clients)
            loop = self._loop
        if clients and loop:
            msg_dict = {
                "type": "robot_telemetry",
                "pos": [round(float(p), 4) for p in pos] if pos is not None else [0.0, 0.0, 0.0],
                "quat": [round(float(q), 4) for q in quat] if quat is not None else [1.0, 0.0, 0.0, 0.0],
                "status": str(status),
                "gripper": bool(gripper)
            }
            if joints is not None:
                msg_dict["joints"] = [round(float(j), 3) for j in joints]
            msg = json.dumps(msg_dict)
            for ws in clients:
                try:
                    asyncio.run_coroutine_threadsafe(ws.send(msg), loop)
                except Exception:
                    pass

    def send_camera_frame(self, frame_rgb, quality: int = 75):
        """Mã hóa frame ảnh RGB thành JPEG, lưu cache đĩa và gửi tới client qua WebSocket & MJPEG Stream."""
        try:
            import cv2
            import base64
            # Resize ảnh cho luồng stream VR 320x240 để siêu nhẹ (chỉ ~8KB/frame)
            small = cv2.resize(frame_rgb, (320, 240))
            bgr = cv2.cvtColor(small, cv2.COLOR_RGB2BGR)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            success, encimg = cv2.imencode('.jpg', bgr, encode_param)
            if success:
                jpeg_bytes = encimg.tobytes()
                with self._lock:
                    self._latest_jpeg = jpeg_bytes
                    clients = list(self._clients)
                    loop = self._loop
                    self._frame_cond.notify_all()

                # Lưu ảnh trực tiếp ra static/wrist_camera.jpg phục vụ chế độ xem file:/// cục bộ
                try:
                    tmp_file = self._cam_file + ".tmp"
                    with open(tmp_file, "wb") as f:
                        f.write(jpeg_bytes)
                    os.replace(tmp_file, self._cam_file)
                except Exception:
                    pass

                # Phát luồng trực tiếp qua WebSocket tới các client đang kết nối
                if clients and loop:
                    b64_str = base64.b64encode(jpeg_bytes).decode('utf-8')
                    msg = json.dumps({"type": "camera", "image": b64_str})
                    for ws in clients:
                        try:
                            asyncio.run_coroutine_threadsafe(ws.send(msg), loop)
                        except Exception:
                            pass
        except Exception:
            pass

    def _run_http(self):
        # Cấu hình HTTPServer đa luồng (ThreadingHTTPServer)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.cert, self.key)

        static_dir = self.static_dir
        server_self = self

        class RequestHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=static_dir, **kwargs)

            def do_GET(self):
                req_path = self.path.split("?")[0]
                client_ip = self.client_address[0] if self.client_address else "unknown"
                if req_path in ("/", "/index.html"):
                    print(f"[WebXR HTTPS] Thiết bị ({client_ip}) ĐÃ KẾT NỐI thành công vào trang WebXR Cockpit!")
                # 1. Luồng video MJPEG liên tục thời gian thực (Native Realtime Stream)
                if req_path in ("/video_feed", "/stream.mjpg", "/wrist_camera_stream"):
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Expires", "0")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    last_sent = None
                    try:
                        while server_self._running:
                            with server_self._frame_cond:
                                server_self._frame_cond.wait(timeout=0.1)
                                jpeg_bytes = server_self._latest_jpeg
                            if jpeg_bytes and jpeg_bytes != last_sent:
                                last_sent = jpeg_bytes
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(f"Content-Length: {len(jpeg_bytes)}\r\n\r\n".encode())
                                self.wfile.write(jpeg_bytes)
                                self.wfile.write(b"\r\n")
                                self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, Exception):
                        pass
                    return

                # 2. Ảnh đơn chụp nhanh (Snapshot)
                if req_path == "/wrist_camera.jpg" or req_path.endswith("wrist_camera.jpg"):
                    with server_self._lock:
                        jpeg_bytes = server_self._latest_jpeg
                    if jpeg_bytes:
                        self.send_response(200)
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpeg_bytes)))
                        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                        self.send_header("Pragma", "no-cache")
                        self.send_header("Expires", "0")
                        self.send_header("Access-Control-Allow-Origin", "*")
                        self.end_headers()
                        self.wfile.write(jpeg_bytes)
                        return
                    else:
                        self.send_response(404)
                        self.end_headers()
                        return
                super().do_GET()

            def log_message(self, format, *args):
                pass  # Tắt log mặc định để tránh spam terminal

        try:
            httpd = http.server.ThreadingHTTPServer((self.host, self.http_port), RequestHandler)
            httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
            print(f"[Quest] HTTPS Server running at https://{self.host}:{self.http_port}")
            httpd.serve_forever()
        except OSError as e:
            if e.errno == 98:
                print(f"[Quest] Lỗi: Cổng {self.http_port} đã bị chiếm dụng. Vui lòng tắt các tiến trình cũ.")
            else:
                print(f"[Quest] HTTPS Error: {e}")

    def _run_ws(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._ws_main())

    async def _ws_main(self):
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.cert, self.key)

        async def handler(websocket):
            remote_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
            print(f"[WebXR WebSocket] KÍNH QUEST ĐÃ KẾT NỐI ĐIỀU KHIỂN THÀNH CÔNG từ {remote_ip}!")
            with self._lock:
                self._connected = True
                self._clients.add(websocket)
                current_mode = self._drive_mode
            try:
                # Gửi trạng thái Teleop hiện tại ngay khi client vừa kết nối
                await websocket.send(json.dumps({"type": "teleop_state", "enabled": current_mode}))
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        with self._lock:
                            if data.get("type") == "estop":
                                self._estop_triggered = True
                            elif data.get("type") == "ui_state":
                                self._drive_mode = bool(data.get("drive_mode", False))
                            elif data.get("type") == "go_home":
                                self._home_requested = True
                            else:
                                self._latest_frame = data
                                self._last_seen = time.time()
                                if data.get("home_btn"):
                                    self._home_requested = True
                    except json.JSONDecodeError:
                        pass
            except (websockets.exceptions.ConnectionClosed, Exception):
                pass
            finally:
                print(f"[WebXR WebSocket] Kính Quest đã ngắt kết nối điều khiển từ {remote_ip}")
                with self._lock:
                    self._clients.discard(websocket)
                    if not self._clients:
                        self._connected = False
                        self._latest_frame = None
                        self._drive_mode = False

        try:
            # Chạy đồng thời WSS (cổng 8444 cho Quest WebXR) và WS (cổng 8445 cho PC/local preview)
            async with websockets.serve(handler, self.host, self.ws_port, ssl=context):
                async with websockets.serve(handler, self.host, self.ws_port_plain):
                    print(f"[Quest] WSS Server running at wss://{self.host}:{self.ws_port}")
                    print(f"[Quest] WS Server (Plain) running at ws://{self.host}:{self.ws_port_plain}")
                    await asyncio.Future()  # run forever
        except OSError as e:
            if e.errno == 98:
                print(f"[Quest] Lỗi: Cổng WebSocket ({self.ws_port} hoặc {self.ws_port_plain}) đã bị chiếm dụng.")
            else:
                print(f"[Quest] WSS Error: {e}")

if __name__ == "__main__":
    # Test server standalone
    import sys
    base_dir = os.path.dirname(os.path.dirname(__file__))
    cert_path = os.path.join(base_dir, "certs", "cert.pem")
    key_path = os.path.join(base_dir, "certs", "key.pem")
    
    if not os.path.exists(cert_path):
        print(f"[Error] Missing cert file at {cert_path}")
        sys.exit(1)
        
    server = QuestServer(cert_path, key_path)
    server.start()
    print("Press Ctrl+C to stop.")
    try:
        while True:
            frame = server.get_frame()
            if frame:
                # Xóa màn hình terminal và in đè lên
                print(f"\r[Tọa độ VR] X:{frame['pos'][0]:.3f} | Y:{frame['pos'][1]:.3f} | Z:{frame['pos'][2]:.3f} | Trigger: {frame.get('trigger')} | Grip: {frame.get('grip')}     ", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[Đã thoát]")
