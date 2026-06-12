#!/usr/bin/env python3
"""
Serve a USB camera (V4L2) as an MJPEG stream for the robot arm web UI.

Open in a browser:  http://<pi-ip>:8082/stream
Single frames:      http://<pi-ip>:8082/snapshot

Needs ffmpeg on the Pi:  sudo apt install -y ffmpeg

Environment:
  ROBOT_ARM_CAMERA_PORT   default 8082
  ROBOT_ARM_CAMERA_DEVICE default /dev/video0
"""

import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

PORT = int(os.environ.get("ROBOT_ARM_CAMERA_PORT", "8082"))
DEVICE = os.environ.get("ROBOT_ARM_CAMERA_DEVICE", "/dev/video0")
BOUNDARY = b"--jpgboundary"
SNAPSHOT_WAIT_SECONDS = 8


class LatestFrame:
    """Keeps the most recent JPEG from the background capture thread."""

    def __init__(self):
        self.lock = threading.Lock()
        self.data = None

    def set(self, data):
        with self.lock:
            self.data = data

    def get(self):
        with self.lock:
            return self.data

    def wait_for_frame(self, timeout_seconds):
        deadline = time.time() + timeout_seconds
        while time.time() < deadline:
            frame = self.get()
            if frame:
                return frame
            time.sleep(0.2)
        return None


latest_frame = LatestFrame()
last_good_command = None
last_capture_error = ""


def build_ffmpeg_commands():
    """Try several capture modes. Many USB cameras need YUYV, not MJPEG."""
    return [
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-framerate", "10", "-video_size", "640x480",
            "-i", DEVICE,
            "-vf", "fps=10,scale=640:-1",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "8",
            "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-input_format", "mjpeg",
            "-video_size", "640x480", "-framerate", "10",
            "-i", DEVICE,
            "-c", "copy", "-f", "image2pipe", "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-framerate", "5", "-video_size", "320x240",
            "-i", DEVICE,
            "-vf", "fps=5",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "10",
            "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-i", DEVICE,
            "-vf", "fps=5,scale=640:-1",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "10",
            "pipe:1",
        ],
    ]


def start_ffmpeg(cmd):
    return subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def read_stderr(proc, bucket):
    try:
        if proc.stderr:
            bucket.append(proc.stderr.read())
    except Exception:
        pass


def ffmpeg_error_text(proc, stderr_bucket):
    if stderr_bucket:
        return stderr_bucket[0].decode("utf-8", errors="replace").strip()
    if proc.stderr:
        return proc.stderr.read().decode("utf-8", errors="replace").strip()
    return ""


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


def capture_worker():
    """Keep one ffmpeg process running and store the latest frame."""
    global last_good_command, last_capture_error

    while True:
        commands = []
        if last_good_command is not None:
            commands.append(last_good_command)
        for cmd in build_ffmpeg_commands():
            if cmd not in commands:
                commands.append(cmd)

        got_frames = False

        for cmd in commands:
            proc = None
            stderr_bucket = []
            stderr_thread = None
            frame_count = 0

            try:
                proc = start_ffmpeg(cmd)
                stderr_thread = threading.Thread(
                    target=read_stderr, args=(proc, stderr_bucket), daemon=True
                )
                stderr_thread.start()

                for frame in iter_jpeg_frames(proc):
                    latest_frame.set(frame)
                    frame_count = frame_count + 1
                    got_frames = True
                    last_good_command = cmd
                    last_capture_error = ""

                if stderr_thread:
                    stderr_thread.join(timeout=2)

                if frame_count == 0:
                    err = ffmpeg_error_text(proc, stderr_bucket)
                    last_capture_error = err or f"No frames from ffmpeg on {DEVICE}"
                    print(f"Camera try failed on {DEVICE}: {last_capture_error}", file=sys.stderr)
                else:
                    print(f"Camera capture stopped on {DEVICE} after {frame_count} frames", file=sys.stderr)

            except Exception as exc:
                last_capture_error = str(exc)
                print(f"Camera capture error on {DEVICE}: {exc}", file=sys.stderr)
            finally:
                if proc is not None:
                    proc.kill()
                    try:
                        proc.wait(timeout=2)
                    except Exception:
                        pass

            if got_frames:
                break

        if not got_frames:
            last_good_command = None
            print(
                f"No camera frames from {DEVICE}. "
                "Check: v4l2-ctl --list-devices "
                "and try ROBOT_ARM_CAMERA_DEVICE=/dev/video1",
                file=sys.stderr,
            )
            time.sleep(3)
        else:
            time.sleep(1)


class CameraHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _path(self):
        return self.path.split("?", 1)[0]

    def _send_snapshot(self):
        frame = latest_frame.wait_for_frame(SNAPSHOT_WAIT_SECONDS)
        if not frame:
            message = "No camera frame yet"
            if last_capture_error:
                message = message + ". " + last_capture_error
            self.send_error(503, message)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def do_HEAD(self):
        path = self._path()
        if path == "/snapshot":
            frame = latest_frame.get()
            if not frame:
                self.send_error(503, "No camera frame yet")
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            return
        if path not in ("/", "/stream"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=jpgboundary")
        self.end_headers()

    def do_GET(self):
        path = self._path()
        if path == "/snapshot":
            self._send_snapshot()
            return
        if path not in ("/", "/stream"):
            self.send_error(404)
            return

        sent_frame = False
        for cmd in build_ffmpeg_commands():
            proc = None
            stderr_bucket = []
            stderr_thread = None
            headers_sent = False
            try:
                proc = start_ffmpeg(cmd)
                stderr_thread = threading.Thread(
                    target=read_stderr, args=(proc, stderr_bucket), daemon=True
                )
                stderr_thread.start()

                for frame in iter_jpeg_frames(proc):
                    if not headers_sent:
                        self.send_response(200)
                        self.send_header("Cache-Control", "no-cache, private")
                        self.send_header("Pragma", "no-cache")
                        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=jpgboundary")
                        self.end_headers()
                        headers_sent = True

                    sent_frame = True
                    try:
                        self.wfile.write(BOUNDARY + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n")
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        break

                if sent_frame:
                    break

                if stderr_thread:
                    stderr_thread.join(timeout=2)
                err = ffmpeg_error_text(proc, stderr_bucket)
                if err:
                    print(f"ffmpeg stream try failed on {DEVICE}: {err}", file=sys.stderr)
            finally:
                if proc is not None:
                    proc.kill()
                    try:
                        proc.wait(timeout=2)
                    except Exception:
                        pass

        if not sent_frame:
            self.send_error(500, f"Could not capture from {DEVICE}")


def main():
    if not os.path.exists(DEVICE):
        print(f"Error: camera device not found: {DEVICE}", file=sys.stderr)
        print("List devices: v4l2-ctl --list-devices", file=sys.stderr)
        sys.exit(1)

    worker = threading.Thread(target=capture_worker, daemon=True)
    worker.start()

    print(f"Camera MJPEG stream: http://0.0.0.0:{PORT}/stream")
    print(f"Camera snapshots:    http://0.0.0.0:{PORT}/snapshot")
    print(f"Device: {DEVICE}")
    HTTPServer(("0.0.0.0", PORT), CameraHandler).serve_forever()


if __name__ == "__main__":
    main()
