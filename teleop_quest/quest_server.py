import asyncio
import http.server
import json
import os
import ssl
import threading
import time
import websockets

class QuestServer:
    def __init__(self, cert: str, key: str, host: str = "0.0.0.0", http_port: int = 8443, ws_port: int = 8444):
        self.cert = cert
        self.key = key
        self.host = host
        self.http_port = http_port
        self.ws_port = ws_port

        self._lock = threading.Lock()
        self._latest_frame = None
        self._last_seen = 0.0
        self._connected = False

    def start(self):
        # Khởi động HTTPS server (chạy daemon thread)
        http_thread = threading.Thread(target=self._run_http, daemon=True)
        http_thread.start()

        # Khởi động WSS server (chạy daemon thread)
        ws_thread = threading.Thread(target=self._run_ws, daemon=True)
        ws_thread.start()

    def get_frame(self):
        with self._lock:
            return self._latest_frame

    def last_seen(self) -> float:
        with self._lock:
            return self._last_seen

    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def _run_http(self):
        # Cấu hình HTTPServer
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.cert, self.key)

        static_dir = os.path.join(os.path.dirname(__file__), "static")

        class RequestHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=static_dir, **kwargs)

            def log_message(self, format, *args):
                pass  # Tắt log mặc định để tránh spam terminal

        try:
            httpd = http.server.HTTPServer((self.host, self.http_port), RequestHandler)
            httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
            print(f"[Quest] HTTPS Server running at https://{self.host}:{self.http_port}")
            httpd.serve_forever()
        except OSError as e:
            if e.errno == 98:
                print(f"[Quest] Lỗi: Cổng {self.http_port} đã bị chiếm dụng. Vui lòng tắt các tiến trình cũ.")
            else:
                print(f"[Quest] HTTPS Error: {e}")

    def _run_ws(self):
        asyncio.run(self._ws_main())

    async def _ws_main(self):
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(self.cert, self.key)

        async def handler(websocket):
            remote_ip = websocket.remote_address[0]
            print(f"[Quest] WebSocket connected from {remote_ip}")
            with self._lock:
                self._connected = True
            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        with self._lock:
                            self._latest_frame = data
                            self._last_seen = time.time()
                    except json.JSONDecodeError:
                        pass
            except websockets.exceptions.ConnectionClosed:
                print(f"[Quest] WebSocket disconnected from {remote_ip}")
            finally:
                with self._lock:
                    self._connected = False
                    self._latest_frame = None

        try:
            async with websockets.serve(handler, self.host, self.ws_port, ssl=context):
                print(f"[Quest] WSS Server running at wss://{self.host}:{self.ws_port}")
                await asyncio.Future()  # run forever
        except OSError as e:
            if e.errno == 98:
                print(f"[Quest] Lỗi: Cổng {self.ws_port} đã bị chiếm dụng. Vui lòng tắt các tiến trình cũ.")
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
            time.sleep(1)
    except KeyboardInterrupt:
        pass
