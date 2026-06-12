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
CONFIGURED_DEVICE = os.environ.get("ROBOT_ARM_CAMERA_DEVICE", "/dev/video0")
active_device = CONFIGURED_DEVICE
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


def device_has_capture_formats(device_path):
    """True when v4l2-ctl lists real pixel formats (not a metadata-only node)."""
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device_path, "--list-formats-ext"],
            capture_output=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    if result.returncode != 0:
        return False
    text = result.stdout.decode("utf-8", errors="replace")
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("[") and "]:" in stripped:
            return True
    return False


def find_capture_devices():
    devices = []
    try:
        names = sorted(os.listdir("/dev"))
    except OSError:
        return devices
    for name in names:
        if not name.startswith("video"):
            continue
        path = os.path.join("/dev", name)
        if os.path.exists(path) and device_has_capture_formats(path):
            devices.append(path)
    return devices


def pick_active_device():
    """Use configured device, or auto-pick the first real capture node."""
    global active_device

    if os.path.exists(CONFIGURED_DEVICE) and device_has_capture_formats(CONFIGURED_DEVICE):
        active_device = CONFIGURED_DEVICE
        return active_device

    found = find_capture_devices()
    if found:
        active_device = found[0]
        if CONFIGURED_DEVICE != active_device:
            print(
                f"Camera: {CONFIGURED_DEVICE} is not a capture device. "
                f"Using {active_device} instead.",
                file=sys.stderr,
            )
        return active_device

    active_device = CONFIGURED_DEVICE
    return active_device


def build_ffmpeg_commands(device=None):
    """Try several capture modes. Many USB cameras need YUYV, not MJPEG."""
    if device is None:
        device = active_device
    return [
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-framerate", "10", "-video_size", "640x480",
            "-i", device,
            "-vf", "fps=10,scale=640:-1",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "8",
            "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-input_format", "mjpeg",
            "-video_size", "640x480", "-framerate", "10",
            "-i", device,
            "-c", "copy", "-f", "image2pipe", "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-framerate", "5", "-video_size", "320x240",
            "-i", device,
            "-vf", "fps=5",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "10",
            "pipe:1",
        ],
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-f", "v4l2", "-i", device,
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


def try_next_capture_device():
    """Switch to another V4L2 node when the current one cannot capture."""
    global active_device, last_good_command

    options = find_capture_devices()
    if not options:
        return False

    try:
        index = options.index(active_device)
        next_index = index + 1
    except ValueError:
        next_index = 0

    if next_index >= len(options):
        return False

    active_device = options[next_index]
    last_good_command = None
    print(f"Camera: trying next device {active_device}", file=sys.stderr)
    return True


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
                    last_capture_error = err or f"No frames from ffmpeg on {active_device}"
                    print(f"Camera try failed on {active_device}: {last_capture_error}", file=sys.stderr)
                else:
                    print(
                        f"Camera capture stopped on {active_device} after {frame_count} frames",
                        file=sys.stderr,
                    )

            except Exception as exc:
                last_capture_error = str(exc)
                print(f"Camera capture error on {active_device}: {exc}", file=sys.stderr)
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
            if not try_next_capture_device():
                print(
                    f"No camera frames from {active_device}. "
                    "Run: v4l2-ctl --list-devices "
                    "and: v4l2-ctl -d /dev/video0 --list-formats-ext",
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
                    print(f"ffmpeg stream try failed on {active_device}: {err}", file=sys.stderr)
            finally:
                if proc is not None:
                    proc.kill()
                    try:
                        proc.wait(timeout=2)
                    except Exception:
                        pass

        if not sent_frame:
            self.send_error(500, f"Could not capture from {active_device}")


def main():
    pick_active_device()

    if not os.path.exists(active_device):
        print(f"Error: camera device not found: {active_device}", file=sys.stderr)
        print("List devices: v4l2-ctl --list-devices", file=sys.stderr)
        sys.exit(1)

    if not device_has_capture_formats(active_device):
        print(
            f"Error: {active_device} is not a video capture device "
            '(try /dev/video0 — "Inappropriate ioctl" means wrong node).',
            file=sys.stderr,
        )
        print("Check formats: v4l2-ctl -d /dev/video0 --list-formats-ext", file=sys.stderr)
        found = find_capture_devices()
        if not found:
            sys.exit(1)
        print(f"Available capture devices: {', '.join(found)}", file=sys.stderr)
        sys.exit(1)

    worker = threading.Thread(target=capture_worker, daemon=True)
    worker.start()

    print(f"Camera MJPEG stream: http://0.0.0.0:{PORT}/stream")
    print(f"Camera snapshots:    http://0.0.0.0:{PORT}/snapshot")
    print(f"Device: {active_device}")
    HTTPServer(("0.0.0.0", PORT), CameraHandler).serve_forever()


if __name__ == "__main__":
    main()
