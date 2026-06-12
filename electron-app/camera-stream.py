#!/usr/bin/env python3
"""
Serve a USB camera (V4L2) as an MJPEG stream for the robot arm web UI.

Open in a browser:  http://<pi-ip>:8082/stream

Needs ffmpeg on the Pi:  sudo apt install -y ffmpeg

Environment:
  ROBOT_ARM_CAMERA_PORT   default 8082
  ROBOT_ARM_CAMERA_DEVICE default /dev/video0
"""

import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT = int(os.environ.get("ROBOT_ARM_CAMERA_PORT", "8082"))
DEVICE = os.environ.get("ROBOT_ARM_CAMERA_DEVICE", "/dev/video0")
BOUNDARY = b"--jpgboundary"


def build_ffmpeg_commands():
    """Try MJPEG from camera first, then encode from raw video."""
    return [
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-input_format", "mjpeg",
            "-video_size", "640x480", "-framerate", "10",
            "-i", DEVICE,
            "-c", "copy", "-f", "mjpeg", "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-video_size", "640x480", "-framerate", "10",
            "-i", DEVICE,
            "-vf", "fps=10,scale=640:-1",
            "-f", "mpjpeg", "-q:v", "5", "pipe:1",
        ],
    ]


def start_ffmpeg():
    last_error = None
    for cmd in build_ffmpeg_commands():
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            return proc
        except FileNotFoundError:
            print("Error: ffmpeg not found. Install: sudo apt install -y ffmpeg", file=sys.stderr)
            sys.exit(1)
        except Exception as exc:
            last_error = exc
    if last_error:
        raise last_error
    return None


def iter_jpeg_frames(proc):
    """Read JPEG images from ffmpeg stdout (start FF D8, end FF D9)."""
    buffer = b""
    while True:
        chunk = proc.stdout.read(4096)
        if not chunk:
            break
        buffer += chunk
        while True:
            start = buffer.find(b"\xff\xd8")
            end = buffer.find(b"\xff\xd9", start + 2 if start >= 0 else 0)
            if start == -1 or end == -1:
                break
            frame = buffer[start : end + 2]
            buffer = buffer[end + 2 :]
            yield frame


class CameraHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _path_ok(self):
        path = self.path.split("?", 1)[0]
        return path in ("/", "/stream")

    def do_HEAD(self):
        if not self._path_ok():
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=jpgboundary")
        self.end_headers()

    def do_GET(self):
        if not self._path_ok():
            self.send_error(404)
            return

        proc = start_ffmpeg()
        if proc is None:
            self.send_error(500, "Could not start ffmpeg")
            return

        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=jpgboundary")
        self.end_headers()

        sent_frame = False
        try:
            for frame in iter_jpeg_frames(proc):
                sent_frame = True
                try:
                    self.wfile.write(BOUNDARY + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break
        finally:
            if not sent_frame and proc.stderr:
                err = proc.stderr.read().decode("utf-8", errors="replace").strip()
                if err:
                    print(f"ffmpeg error on {DEVICE}: {err}", file=sys.stderr)
            proc.kill()
            try:
                proc.wait(timeout=2)
            except Exception:
                pass


def main():
    if not os.path.exists(DEVICE):
        print(f"Error: camera device not found: {DEVICE}", file=sys.stderr)
        print("List devices: ls -l /dev/video*", file=sys.stderr)
        sys.exit(1)

    print(f"Camera MJPEG stream: http://0.0.0.0:{PORT}/stream")
    print(f"Device: {DEVICE}")
    HTTPServer(("0.0.0.0", PORT), CameraHandler).serve_forever()


if __name__ == "__main__":
    main()
