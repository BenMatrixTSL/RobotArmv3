#!/usr/bin/env python3
"""
Serve the robot arm web UI and proxy the camera MJPEG stream on the same port.

Static files:  http://<pi>/index.html
Camera proxy:  http://<pi>/camera/snapshot  ->  http://127.0.0.1:8082/snapshot
              http://<pi>/camera/stream     ->  http://127.0.0.1:8082/stream

The camera service (robot-arm-camera.service) must still run on port 8082.
"""

import os
import sys
import urllib.error
import urllib.request
from http.server import HTTPServer, SimpleHTTPRequestHandler

PORT = int(os.environ.get("ROBOT_ARM_WEB_PORT", "80"))
BIND = os.environ.get("ROBOT_ARM_WEB_BIND", "0.0.0.0")
CAMERA_BACKEND = os.environ.get(
    "ROBOT_ARM_CAMERA_BACKEND", "http://127.0.0.1:8082/stream"
)


class AppHandler(SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _camera_path(self):
        return self.path.split("?", 1)[0]

    def _is_camera_path(self):
        return self._camera_path() in ("/camera/stream", "/camera", "/camera/snapshot")

    def _camera_backend_url(self):
        path = self._camera_path()
        if path == "/camera/snapshot":
            return CAMERA_BACKEND.replace("/stream", "/snapshot")
        return CAMERA_BACKEND

    def _proxy_camera(self, method):
        try:
            backend = self._camera_backend_url()
            req = urllib.request.Request(backend, method=method)
            with urllib.request.urlopen(req, timeout=15) as upstream:
                self.send_response(upstream.status)
                for key in ("Content-Type", "Cache-Control", "Pragma"):
                    value = upstream.headers.get(key)
                    if value:
                        self.send_header(key, value)
                self.end_headers()
                if method == "GET":
                    while True:
                        chunk = upstream.read(8192)
                        if not chunk:
                            break
                        try:
                            self.wfile.write(chunk)
                        except (BrokenPipeError, ConnectionResetError):
                            break
        except urllib.error.HTTPError as exc:
            self.send_error(exc.code, exc.reason)
        except Exception as exc:
            print(f"Camera proxy error: {exc}", file=sys.stderr)
            self.send_error(502, f"Camera service not available ({exc})")

    def do_HEAD(self):
        if self._is_camera_path():
            self._proxy_camera("HEAD")
            return
        super().do_HEAD()

    def do_GET(self):
        if self._is_camera_path():
            self._proxy_camera("GET")
            return
        super().do_GET()


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    snapshot_backend = CAMERA_BACKEND.replace("/stream", "/snapshot")
    print(f"Web UI: http://{BIND}:{PORT}/")
    print(f"Camera proxy: http://{BIND}:{PORT}/camera/snapshot -> {snapshot_backend}")
    HTTPServer((BIND, PORT), AppHandler).serve_forever()


if __name__ == "__main__":
    main()
