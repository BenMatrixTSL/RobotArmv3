#!/usr/bin/env python3
"""
USB camera vision service for the robot arm.

Captures at high resolution (1280x720), runs OpenCV detection, then serves a
lower-resolution annotated image (640x480) to the web UI.

Endpoints:
  /snapshot  - latest JPEG with overlays (for the Camera tab)
  /vision    - JSON with marker and coloured block positions
  /stream    - MJPEG stream (optional, same annotated frames)

Install on the Pi:
  sudo apt install -y python3-opencv v4l-utils

Environment:
  ROBOT_ARM_CAMERA_PORT          default 8082
  ROBOT_ARM_CAMERA_DEVICE        default /dev/video0
  ROBOT_ARM_CAPTURE_WIDTH        default 1280
  ROBOT_ARM_CAPTURE_HEIGHT       default 720
  ROBOT_ARM_STREAM_WIDTH         default 640
  ROBOT_ARM_STREAM_HEIGHT        default 480
"""

import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np

PORT = int(os.environ.get("ROBOT_ARM_CAMERA_PORT", "8082"))
CONFIGURED_DEVICE = os.environ.get("ROBOT_ARM_CAMERA_DEVICE", "/dev/video0")
CAPTURE_WIDTH = int(os.environ.get("ROBOT_ARM_CAPTURE_WIDTH", "1280"))
CAPTURE_HEIGHT = int(os.environ.get("ROBOT_ARM_CAPTURE_HEIGHT", "720"))
STREAM_WIDTH = int(os.environ.get("ROBOT_ARM_STREAM_WIDTH", "640"))
STREAM_HEIGHT = int(os.environ.get("ROBOT_ARM_STREAM_HEIGHT", "480"))
JPEG_QUALITY = 80
MIN_BLOCK_AREA = 2500
BOUNDARY = b"--jpgboundary"

# Try these dictionaries in order (most common first).
# Markers must match one of these — see chev.me/arucogen or generate-aruco-markers.py
ARUCO_DICTIONARIES = [
    ("DICT_4X4_50", "DICT_4X4_50"),
    ("DICT_4X4_100", "DICT_4X4_100"),
    ("DICT_4X4_250", "DICT_4X4_250"),
    ("DICT_5X5_50", "DICT_5X5_50"),
    ("DICT_5X5_100", "DICT_5X5_100"),
    ("DICT_6X6_50", "DICT_6X6_50"),
    ("DICT_6X6_250", "DICT_6X6_250"),
]

# HSV colour ranges for simple block detection (tune under your lighting).
COLOR_RANGES = {
    "red": [
        ((0, 120, 70), (10, 255, 255)),
        ((170, 120, 70), (180, 255, 255)),
    ],
    "green": [
        ((40, 60, 60), (85, 255, 255)),
    ],
    "blue": [
        ((95, 80, 60), (130, 255, 255)),
    ],
    "yellow": [
        ((20, 100, 100), (35, 255, 255)),
    ],
}

active_device = CONFIGURED_DEVICE


class SharedVisionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.vision_data = {
            "markers": [],
            "blocks": [],
            "capture_width": CAPTURE_WIDTH,
            "capture_height": CAPTURE_HEIGHT,
            "stream_width": STREAM_WIDTH,
            "stream_height": STREAM_HEIGHT,
        }

    def set_frame(self, jpeg_bytes, vision_data):
        with self.lock:
            self.jpeg = jpeg_bytes
            self.vision_data = vision_data

    def get_jpeg(self):
        with self.lock:
            return self.jpeg

    def get_vision_json(self):
        with self.lock:
            data = dict(self.vision_data)
            data["timestamp"] = time.time()
            return json.dumps(data)

    def wait_for_jpeg(self, timeout_seconds):
        deadline = time.time() + timeout_seconds
        while time.time() < deadline:
            jpeg = self.get_jpeg()
            if jpeg:
                return jpeg
            time.sleep(0.2)
        return None


state = SharedVisionState()


def device_has_capture_formats(device_path):
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


def create_aruco_parameters():
    """Detector settings tuned for USB webcams (more sensitive than OpenCV defaults)."""
    try:
        parameters = cv2.aruco.DetectorParameters()
    except AttributeError:
        parameters = cv2.aruco.DetectorParameters_create()

    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshConstant = 7
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.03
    parameters.minDistanceToBorder = 1
    parameters.minMarkerDistanceRate = 0.03

    try:
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    except AttributeError:
        pass

    return parameters


def get_aruco_dictionary(dict_name):
    dict_id = getattr(cv2.aruco, dict_name)
    try:
        return cv2.aruco.getPredefinedDictionary(dict_id)
    except AttributeError:
        return cv2.aruco.Dictionary_get(dict_id)


def get_aruco_dictionary_list():
    """Use one dictionary by default (faster on the Pi). Set ROBOT_ARM_ARUCO_TRY_ALL=1 for more."""
    try_all = os.environ.get("ROBOT_ARM_ARUCO_TRY_ALL", "").strip().lower()
    if try_all in ("1", "true", "yes"):
        return ARUCO_DICTIONARIES
    return [("DICT_4X4_50", "DICT_4X4_50")]


def create_aruco_detectors():
    """Build one detector per dictionary (old and new OpenCV APIs)."""
    parameters = create_aruco_parameters()
    detectors = []

    for label, dict_name in get_aruco_dictionary_list():
        try:
            dictionary = get_aruco_dictionary(dict_name)
        except AttributeError:
            continue

        try:
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            detectors.append((label, ("new", detector)))
        except AttributeError:
            detectors.append((label, ("old", (dictionary, parameters))))

    return detectors


def detect_aruco_markers(gray, detector_info):
    mode = detector_info[0]
    if mode == "new":
        corners, ids, _rejected = detector_info[1].detectMarkers(gray)
    else:
        dictionary, parameters = detector_info[1]
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )
    return corners, ids


def find_aruco_markers(gray, detectors):
    """Try each ArUco dictionary until markers are found."""
    for label, detector_info in detectors:
        corners, ids = detect_aruco_markers(gray, detector_info)
        if ids is not None and len(ids) > 0:
            return corners, ids, label
    return None, None, None


def draw_aruco_overlays(frame, corners, ids):
    """Draw thick green boxes so markers stay visible after downscaling."""
    if ids is None or corners is None:
        return

    try:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))
    except Exception:
        pass

    for index, marker_id in enumerate(ids.flatten()):
        points = corners[index][0].astype(int)
        cv2.polylines(frame, [points], True, (0, 255, 0), 4)

        center_x = int(np.mean(points[:, 0]))
        center_y = int(np.mean(points[:, 1]))
        label = "id " + str(int(marker_id))
        cv2.putText(
            frame,
            label,
            (center_x + 8, center_y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            3,
        )


def marker_centers(corners, ids, frame_width, frame_height):
    markers = []
    if ids is None:
        return markers

    for index, marker_id in enumerate(ids.flatten()):
        points = corners[index][0]
        center_x = float(np.mean(points[:, 0]))
        center_y = float(np.mean(points[:, 1]))
        markers.append(
            {
                "id": int(marker_id),
                "center_x": round(center_x / frame_width, 4),
                "center_y": round(center_y / frame_height, 4),
                "pixel_x": round(center_x, 1),
                "pixel_y": round(center_y, 1),
            }
        )
    return markers


def find_color_blocks(frame):
    """Find coloured blocks using simple HSV thresholds."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_height, frame_width = frame.shape[:2]
    blocks = []

    for color_name, ranges in COLOR_RANGES.items():
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv, np.array(low), np.array(high))
            if mask is None:
                mask = part
            else:
                mask = cv2.bitwise_or(mask, part)

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_BLOCK_AREA:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + (w / 2.0)
            center_y = y + (h / 2.0)

            blocks.append(
                {
                    "color": color_name,
                    "center_x": round(center_x / frame_width, 4),
                    "center_y": round(center_y / frame_height, 4),
                    "width": round(w / frame_width, 4),
                    "height": round(h / frame_height, 4),
                    "pixel_x": round(center_x, 1),
                    "pixel_y": round(center_y, 1),
                }
            )

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            label = color_name + " block"
            cv2.putText(
                frame,
                label,
                (x, max(y - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

    return blocks


def open_camera(device_path):
    """Open the camera, trying common resolutions until frames work."""
    if device_path.startswith("/dev/video"):
        sources = [device_path, int(device_path.replace("/dev/video", ""))]
    else:
        sources = [device_path]

    capture_modes = [
        (CAPTURE_WIDTH, CAPTURE_HEIGHT),
        (640, 480),
        (800, 600),
        (1280, 720),
    ]

    last_error = "unknown error"

    for width, height in capture_modes:
        for source in sources:
            camera = None
            try:
                camera = cv2.VideoCapture(source, cv2.CAP_V4L2)
                if not camera.isOpened():
                    last_error = f"could not open {source}"
                    continue

                camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                camera.set(cv2.CAP_PROP_FPS, 15)

                ok, test_frame = camera.read()
                if not ok or test_frame is None or test_frame.size == 0:
                    last_error = f"no frames at {width}x{height} on {source}"
                    camera.release()
                    continue

                actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(
                    f"Camera opened: {source} at {actual_width}x{actual_height}",
                    file=sys.stderr,
                )
                return camera
            except Exception as exc:
                last_error = str(exc)
                if camera is not None:
                    camera.release()

    raise RuntimeError(f"Could not open camera {device_path}: {last_error}")


def prepare_gray_for_aruco(frame):
    """Improve contrast so printed markers are easier to detect."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    try:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
    except Exception:
        pass
    return gray


def process_frame(frame, detectors):
    annotated = frame.copy()
    frame_height, frame_width = annotated.shape[:2]
    dictionary_name = None
    corners, ids = None, None
    markers = []
    blocks = []

    try:
        gray = prepare_gray_for_aruco(annotated)
        if detectors:
            corners, ids, dictionary_name = find_aruco_markers(gray, detectors)
            if ids is not None and len(ids) > 0:
                draw_aruco_overlays(annotated, corners, ids)
        markers = marker_centers(corners, ids, frame_width, frame_height)
        blocks = find_color_blocks(annotated)
    except Exception as exc:
        print(f"Vision processing error: {exc}", file=sys.stderr)
        annotated = frame.copy()

    stream_frame = cv2.resize(
        annotated, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_AREA
    )
    ok, jpeg = cv2.imencode(
        ".jpg", stream_frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    )
    if not ok:
        ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return None, None

    vision_data = {
        "markers": markers,
        "blocks": blocks,
        "dictionary": dictionary_name,
        "capture_width": frame_width,
        "capture_height": frame_height,
        "stream_width": STREAM_WIDTH,
        "stream_height": STREAM_HEIGHT,
    }
    return jpeg.tobytes(), vision_data


def capture_loop():
    try:
        detectors = create_aruco_detectors()
        print(
            f"OpenCV {cv2.__version__} — ArUco dictionaries: {len(detectors)}",
            file=sys.stderr,
        )
    except Exception as exc:
        detectors = []
        print(f"ArUco disabled ({exc}) — streaming video only", file=sys.stderr)

    while True:
        camera = None
        try:
            camera = open_camera(active_device)
            while True:
                ok, frame = camera.read()
                if not ok or frame is None:
                    print("Camera read failed, reopening...", file=sys.stderr)
                    break

                result = process_frame(frame, detectors)
                if result[0] is not None:
                    state.set_frame(result[0], result[1])

                time.sleep(0.03)
        except Exception as exc:
            print(f"Vision capture error: {exc}", file=sys.stderr)
        finally:
            if camera is not None:
                camera.release()
        time.sleep(2)


class VisionHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _path(self):
        return self.path.split("?", 1)[0]

    def do_HEAD(self):
        path = self._path()
        if path == "/snapshot":
            jpeg = state.get_jpeg()
            if not jpeg:
                self.send_error(503, "No camera frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            return
        if path == "/vision":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            return
        if path in ("/", "/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=jpgboundary")
            self.end_headers()
            return
        self.send_error(404)

    def do_GET(self):
        path = self._path()

        if path == "/snapshot":
            jpeg = state.wait_for_jpeg(8)
            if not jpeg:
                self.send_error(503, "No camera frame yet")
                return
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)
            return

        if path == "/vision":
            body = state.get_vision_json().encode("utf-8")
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path in ("/", "/stream"):
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=jpgboundary")
            self.end_headers()

            try:
                while True:
                    jpeg = state.wait_for_jpeg(2)
                    if not jpeg:
                        continue
                    self.wfile.write(BOUNDARY + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n")
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                return
            return

        self.send_error(404)


def main():
    pick_active_device()

    if not os.path.exists(active_device):
        print(f"Error: camera device not found: {active_device}", file=sys.stderr)
        sys.exit(1)

    if not device_has_capture_formats(active_device):
        print(f"Error: {active_device} is not a video capture device.", file=sys.stderr)
        sys.exit(1)

    worker = threading.Thread(target=capture_loop, daemon=True)
    worker.start()

    print(f"Vision snapshots: http://0.0.0.0:{PORT}/snapshot")
    print(f"Vision JSON:      http://0.0.0.0:{PORT}/vision")
    print(f"Capture:          {CAPTURE_WIDTH}x{CAPTURE_HEIGHT}")
    print(f"Stream:           {STREAM_WIDTH}x{STREAM_HEIGHT}")
    print(f"Device:             {active_device}")
    HTTPServer(("0.0.0.0", PORT), VisionHandler).serve_forever()


if __name__ == "__main__":
    main()
