#!/usr/bin/env python3
"""
USB camera vision service for the robot arm.

Captures at high resolution for accurate ArUco detection, then serves a
lower-resolution annotated image to the web UI.

Detection is intentionally slow (5 fps default) to keep CPU load low on the Pi.
Colour block detection is off by default - enable it only if needed.

Endpoints:
  /snapshot  - latest JPEG with overlays (for the Camera tab)
  /vision    - JSON with marker and coloured block positions
  /stream    - MJPEG stream (optional, same annotated frames)

Install on the Pi:
  sudo apt install -y python3-opencv v4l-utils

Environment:
  ROBOT_ARM_CAMERA_PORT          default 8082
  ROBOT_ARM_CAMERA_DEVICE        default /dev/video0
  ROBOT_ARM_CAPTURE_WIDTH        default 1280  (ArUco detection resolution)
  ROBOT_ARM_CAPTURE_HEIGHT       default 720
  ROBOT_ARM_STREAM_WIDTH         default 640   (web display resolution)
  ROBOT_ARM_STREAM_HEIGHT        default 480
  ROBOT_ARM_DETECTION_FPS        default 5     (camera + processing rate)
  ROBOT_ARM_COLOR_DETECTION      default 1     (set 0 to disable colour block detection)
  ROBOT_ARM_TRACK_STABILITY_FRAMES  default 5  (frames a colour block must be
                                                 consistently present/absent
                                                 before it is shown/hidden)
  ROBOT_ARM_MARKER_CONFIRM_FRAMES  default 2   (consecutive sightings before an
                                                 ArUco marker is shown)
  ROBOT_ARM_MARKER_HOLD_FRAMES   default 25    (consecutive misses before an
                                                 ArUco marker is dropped - ~5 s)
  ROBOT_ARM_ARUCO_EXPECTED_IDS   default 0,1,2,3 (the mat's marker IDs; other
                                                 IDs are ignored, a missing one
                                                 triggers the far-marker pass)
  ROBOT_ARM_ARUCO_FAR_GAMMA      default 2.2   (far-marker fallback: gamma to
                                                 darken washed-out blacks)
  ROBOT_ARM_ARUCO_FAR_STRETCH    default 2.0   (far-marker fallback: vertical
                                                 stretch to undo foreshortening;
                                                 0 disables the fallback)
  ROBOT_ARM_ARUCO_FAR_BAND       default 0.6   (fraction of the frame, from the
                                                 top, the fallback searches)
  ROBOT_ARM_BLOCK_SIZE_MM        default 30    (cube side length; a candidate's
                                                 ground footprint must be within
                                                 ROBOT_ARM_BLOCK_SIZE_TOLERANCE
                                                 (default 0.45) of it)
  ROBOT_ARM_CAMERA_CALIBRATION   default /etc/robot-arm-st3215/camera-calibration.json
                                                 (lens model from calibrate-camera.py;
                                                 frames are undistorted when it exists)
  ROBOT_ARM_LENS_K1 / _K2        default 0     (manual radial distortion fallback
                                                 when there is no calibration file;
                                                 negative k1 = barrel/fish-eye)

Block positions (world_x_mm/world_y_mm) are the centre of the cube's base on
the mat, and rotation_deg is its turn about the vertical in the mat's frame
(-45..45°), both measured from the silhouette's ground-contact edges so they
stay right when the camera looks across the mat rather than down on it.
  ROBOT_ARM_CAMERA_EXPOSURE      unset by default (auto-exposure). Set to a
                                  manual exposure value (V4L2 driver units,
                                  try 100-300) to stop glare off the mat from
                                  clipping the sensor to solid white near
                                  markers/blocks - see camera-vision.py's
                                  CAMERA_MANUAL_EXPOSURE comment.
  ROBOT_ARM_CAMERA_BRIGHTNESS    unset by default. Hardware brightness
                                  (V4L2 units) to compensate for a low
                                  manual exposure - preferred over the
                                  software fallback below (no colour cast).
  ROBOT_ARM_CAMERA_GAMMA         unset by default. Hardware gamma (V4L2
                                  units), same purpose as brightness above.
  ROBOT_ARM_DISPLAY_TARGET_BRIGHTNESS  default 0 (disabled). Software
                                  fallback: adaptive gamma on the served
                                  feed only (not detection) up to this mean
                                  brightness (0-255) - use only if hardware
                                  brightness/gamma aren't enough.
"""

import json
import math
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
# ArUco detection runs on the full CAPTURE resolution for accuracy.
# The served snapshot is downscaled to STREAM resolution for the web UI.
# 5 fps is sufficient for coordinate-mapping use; raise with ROBOT_ARM_DETECTION_FPS.
DETECTION_FPS = max(1, int(os.environ.get("ROBOT_ARM_DETECTION_FPS", "5")))
# Colour block detection runs on a 640x480 downscale of the original frame,
# keeping CPU manageable regardless of capture resolution.
# Set ROBOT_ARM_COLOR_DETECTION=0 to disable entirely if not needed.
COLOR_DETECTION_ENABLED = os.environ.get("ROBOT_ARM_COLOR_DETECTION", "1").strip() not in ("0", "false", "no")
# X and Y spacing between the corner ArUco markers, in mm.
# Used to compute world (mm) positions of detected colour blocks.
MARKER_SPACING_X_MM = float(os.environ.get("ROBOT_ARM_MARKER_SPACING_X_MM", "240"))
MARKER_SPACING_Y_MM = float(os.environ.get("ROBOT_ARM_MARKER_SPACING_Y_MM", "230"))
# Colour blocks are only reported when their world position falls within the
# marker-defined mat (0..MARKER_SPACING_X_MM, 0..MARKER_SPACING_Y_MM), plus
# this margin. Keeps the arm itself (or anything else outside the mat) from
# being picked up as a false-positive block. Requires at least 3 markers
# visible (a homography) - with fewer, no world position can be computed and
# every colour blob is discarded rather than reported unfiltered.
BLOCK_WORKSPACE_MARGIN_MM = float(os.environ.get("ROBOT_ARM_BLOCK_WORKSPACE_MARGIN_MM", "15"))
JPEG_QUALITY = 80
MIN_BLOCK_AREA = 300
# Upper area bound (px² on the 640x480 detection frame). Only a coarse
# sanity limit now: with the camera looking across the mat a near cube is
# several times bigger in the image than a far one, so the real size test is
# done in world mm from the cube's ground-contact edges (see
# measure_block_footprint()). Keep this well above any real cube.
MAX_BLOCK_AREA = float(os.environ.get("ROBOT_ARM_MAX_BLOCK_AREA", "40000"))
# Side length of the cubes on the mat, mm. The purple cube's ground-contact
# edge measured 29-30 mm with the camera in its current position (its
# rounded corners make the visible edge a little shorter than the cube).
# A candidate whose footprint is outside BLOCK_SIZE_TOLERANCE of this (as a
# fraction) is rejected - arm parts, cables and background patches rarely
# have a cube-sized footprint on the ground plane.
BLOCK_SIZE_MM = float(os.environ.get("ROBOT_ARM_BLOCK_SIZE_MM", "30"))
BLOCK_SIZE_TOLERANCE = float(os.environ.get("ROBOT_ARM_BLOCK_SIZE_TOLERANCE", "0.45"))
# Silhouette aspect limit (long/short side of the image-space rotated rect).
# Looser than it was for the overhead view: seen from the side a cube shows
# its top and front faces stacked, so the silhouette is taller than wide.
BLOCK_MAX_ASPECT_RATIO = float(os.environ.get("ROBOT_ARM_BLOCK_MAX_ASPECT_RATIO", "2.2"))
# contourArea / rotated-bounding-rect-area. A solid cube fills most of its
# own bounding rect; irregular silhouettes (the arm's joints, screws, gaps
# between parts) leave a lot of the rect empty and score low here.
BLOCK_MIN_SOLIDITY = float(os.environ.get("ROBOT_ARM_BLOCK_MIN_SOLIDITY", "0.6"))
# Lens distortion correction. The frame is undistorted before any detection
# when a calibration file exists (made with calibrate-camera.py from a few
# chessboard photos - the proper fix for a wide-angle / fish-eye lens), or,
# as a rough manual fallback, when ROBOT_ARM_LENS_K1 is set (negative =
# barrel distortion; try -0.1 .. -0.4 while watching the mat's straight
# edges in the stream). The calibration file wins if both are present.
CAMERA_CALIBRATION_FILE = os.environ.get(
    "ROBOT_ARM_CAMERA_CALIBRATION", "/etc/robot-arm-st3215/camera-calibration.json"
).strip()
LENS_K1 = float(os.environ.get("ROBOT_ARM_LENS_K1", "0") or 0)
LENS_K2 = float(os.environ.get("ROBOT_ARM_LENS_K2", "0") or 0)
# A marker/block must be detected (or absent) this many consecutive frames
# before it is added to (or removed from) the output - smooths out one-off
# missed detections instead of letting overlays flicker on and off.
TRACK_STABILITY_FRAMES = max(1, int(os.environ.get("ROBOT_ARM_TRACK_STABILITY_FRAMES", "5")))
# ArUco corner markers are static, so they get their own, more forgiving
# debounce than the colour blocks: confirmed after MARKER_CONFIRM_FRAMES
# consecutive sightings, and only dropped after MARKER_HOLD_FRAMES
# consecutive misses (~5 s at the default 5 fps). A marker at the far edge
# of a low-mounted camera's view decodes only every few frames, and this
# keeps it (and the homography built from it) from flickering in and out.
MARKER_CONFIRM_FRAMES = max(1, int(os.environ.get("ROBOT_ARM_MARKER_CONFIRM_FRAMES", "2")))
MARKER_HOLD_FRAMES = max(1, int(os.environ.get("ROBOT_ARM_MARKER_HOLD_FRAMES", "25")))
# The marker IDs that make up the mat. Anything else that decodes is dropped
# (the far-marker fallback below is deliberately permissive and occasionally
# produces a stray ID), and a missing one of these is what triggers the
# fallback pass. Set to "" to accept every ID and never run the fallback.
ARUCO_EXPECTED_IDS = {
    int(v) for v in os.environ.get("ROBOT_ARM_ARUCO_EXPECTED_IDS", "0,1,2,3").split(",") if v.strip() != ""
}
# Far-marker fallback pass. When the camera sits low and looks across the
# mat, the markers at the far edge are foreshortened to ~25 px tall (each
# cell ~4 px) and their blacks wash out to grey, and the normal CLAHE pass
# can't decode them. If any expected ID is missing after the normal pass,
# the upper ARUCO_FAR_BAND fraction of the frame is re-run through a gamma
# curve (darkens the washed-out blacks back to black) and a vertical
# stretch (undoes the foreshortening so the cells are square again).
# Measured on real frames from the current camera position: the normal pass
# found marker 1 in 0/7 frames; gamma 2.2 + 2x stretch found it in 6/7 and
# marker 0 in 7/7. Stretch 0 disables the fallback.
ARUCO_FAR_GAMMA = float(os.environ.get("ROBOT_ARM_ARUCO_FAR_GAMMA", "2.2"))
ARUCO_FAR_STRETCH = float(os.environ.get("ROBOT_ARM_ARUCO_FAR_STRETCH", "2.0"))
ARUCO_FAR_BAND = min(1.0, max(0.1, float(os.environ.get("ROBOT_ARM_ARUCO_FAR_BAND", "0.6"))))
# Max centroid distance (normalised 0-1 image coords) for matching a colour
# block detection to an existing track between frames.
BLOCK_MATCH_MAX_DIST = float(os.environ.get("ROBOT_ARM_BLOCK_MATCH_MAX_DIST", "0.08"))
# Manual exposure, in the V4L2 driver's own units (often 100us steps - try
# small integers like 100-300 first). Unset by default (camera stays on
# auto-exposure). Auto-exposure can let glare off a glossy mat surface
# clip the sensor to solid white, permanently losing detail CLAHE/contrast
# tricks can't recover; a lower fixed exposure avoids the clipping in the
# first place, at the cost of a darker image elsewhere (which the CLAHE
# brightness normalisation above is there to compensate for).
CAMERA_MANUAL_EXPOSURE = os.environ.get("ROBOT_ARM_CAMERA_EXPOSURE", "").strip()
# Hardware brightness/gamma (V4L2 driver units) to compensate for a low
# manual exposure - applied at the camera/ISP level, before any software
# processing, so it doesn't introduce the colour-cast risk a software
# brightness boost has (see DISPLAY_TARGET_BRIGHTNESS below). This is the
# preferred way to make a low-exposure capture look normal again; tune with:
#   v4l2-ctl -d /dev/video0 --set-ctrl=brightness=<value>,gamma=<value>
CAMERA_MANUAL_BRIGHTNESS = os.environ.get("ROBOT_ARM_CAMERA_BRIGHTNESS", "").strip()
CAMERA_MANUAL_GAMMA = os.environ.get("ROBOT_ARM_CAMERA_GAMMA", "").strip()
# Software fallback display brightening - applied to the served snapshot/
# stream only, never to the frame detection runs on. Off by default (0):
# the hardware brightness/gamma controls above do this job with better
# colour fidelity when the camera supports them. Only enable this on a
# camera where they don't help enough - it works in the HSV V channel
# (not raw BGR) to avoid a colour-cast, but is still a coarser tool.
DISPLAY_TARGET_BRIGHTNESS = float(os.environ.get("ROBOT_ARM_DISPLAY_TARGET_BRIGHTNESS", "0"))
DISPLAY_GAMMA_MAX = 2.2
BOUNDARY = b"--jpgboundary"

# Try these dictionaries in order (most common first).
# Markers must match one of these - see chev.me/arucogen or generate-aruco-markers.py
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
# OpenCV hue is 0-179 (half of the 0-360° standard scale).
# Saturation/value lower bounds are deliberately a bit loose - the frame's V
# channel is brightness-normalised (see normalize_brightness()) before
# thresholding, so shadowed blocks no longer need as strict a cutoff.
COLOR_RANGES = {
    "red": [
        ((0, 100, 80), (10, 255, 255)),
        ((170, 100, 80), (180, 255, 255)),
    ],
    # Saturation/value floor higher than the other colours: the dark mat
    # background has a natural olive/green tint (hue ~48, S ~65 in testing)
    # that the general shadow-tolerant floor let through, merging with any
    # real green block into one background-sized blob that failed the max-
    # area check - i.e. the block silently vanished. Real block pixels ran
    # S ~140-200, V ~140+, well clear of the background's ~S65/V65.
    "green": [
        ((40, 110, 70), (85, 255, 255)),
    ],
    "blue": [
        ((95, 65, 50), (125, 255, 255)),
    ],
    "yellow": [
        ((20, 100, 110), (35, 255, 255)),
    ],
    "purple": [
        ((125, 40, 35), (165, 255, 255)),
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
    # Lower than the OpenCV default (7) - makes the adaptive threshold more
    # sensitive so low-contrast markers in shadowed areas still separate
    # from the background. Low false-positive risk: a candidate still has
    # to decode a valid marker ID (with error correction) to be reported.
    parameters.adaptiveThreshConstant = 5
    parameters.minMarkerPerimeterRate = 0.02
    parameters.maxMarkerPerimeterRate = 4.0
    # Slightly higher than the OpenCV default (0.03) - a marker near the
    # edge of a wide-angle lens's field of view gets warped from a square
    # into a trapezoid by lens distortion, and the default tolerance can
    # reject that shape before it ever reaches ID decoding. Only a small
    # increase: this is the epsilon for approxPolyDP, so too large a value
    # over-simplifies the contour and loses the required 4-corner count
    # entirely (tested 0.10 - broke detection for every marker, not just
    # the distorted one). A candidate still has to decode a valid ID (with
    # error correction) to be reported either way.
    parameters.polygonalApproxAccuracyRate = 0.04
    parameters.minCornerDistanceRate = 0.03
    parameters.minDistanceToBorder = 1
    parameters.minMarkerDistanceRate = 0.03
    # Default is 0.35 - a bit more tolerance for bit-sampling errors along
    # the border once perspective correction has already had to stretch a
    # heavily distorted (edge-of-frame) marker back into a square.
    parameters.maxErroneousBitsInBorderRate = 0.5

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


def filter_expected_markers(corners, ids):
    """Drop any decoded ID that isn't one of ARUCO_EXPECTED_IDS (no-op if unset)."""
    if ids is None or not ARUCO_EXPECTED_IDS:
        return corners, ids
    keep = [i for i, marker_id in enumerate(ids.flatten()) if int(marker_id) in ARUCO_EXPECTED_IDS]
    if len(keep) == len(ids):
        return corners, ids
    if not keep:
        return None, None
    return [corners[i] for i in keep], np.array([int(ids.flatten()[i]) for i in keep]).reshape(-1, 1)


_far_gamma_lut = None


def far_marker_gray(frame_gray):
    """
    The far-marker fallback's input: gamma-darkened and vertically stretched
    copy of the top ARUCO_FAR_BAND of the plain grey frame (see the
    ARUCO_FAR_* settings). Returns (image, stretch_y) - corners found in it
    map back to the original frame by dividing y by stretch_y.
    """
    global _far_gamma_lut
    if _far_gamma_lut is None:
        _far_gamma_lut = np.array(
            [((i / 255.0) ** ARUCO_FAR_GAMMA) * 255 for i in range(256)]
        ).astype("uint8")
    band_rows = max(1, int(frame_gray.shape[0] * ARUCO_FAR_BAND))
    band = cv2.LUT(frame_gray[0:band_rows, :], _far_gamma_lut)
    stretched = cv2.resize(band, None, fx=1.0, fy=ARUCO_FAR_STRETCH, interpolation=cv2.INTER_LINEAR)
    return stretched, ARUCO_FAR_STRETCH


def find_far_markers(frame, detectors, found_ids):
    """
    Second detection pass for expected markers the normal pass missed (see
    ARUCO_FAR_GAMMA/STRETCH/BAND). Returns (corners, ids) for only the
    missing IDs, with corners mapped back to the original frame, or
    (None, None) if nothing new was found or the fallback is disabled.
    """
    if ARUCO_FAR_STRETCH <= 0 or not ARUCO_EXPECTED_IDS or not detectors:
        return None, None
    missing = ARUCO_EXPECTED_IDS - set(found_ids)
    if not missing:
        return None, None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    far_gray, stretch_y = far_marker_gray(gray)
    corners, ids, _label = find_aruco_markers(far_gray, detectors)
    if ids is None:
        return None, None
    new_corners, new_ids = [], []
    for index, marker_id in enumerate(ids.flatten()):
        marker_id = int(marker_id)
        if marker_id not in missing or marker_id in new_ids:
            continue
        mapped = corners[index].copy()
        mapped[..., 1] /= stretch_y
        new_corners.append(mapped)
        new_ids.append(marker_id)
    if not new_ids:
        return None, None
    return new_corners, np.array(new_ids).reshape(-1, 1)


def merge_marker_detections(corners_a, ids_a, corners_b, ids_b):
    """Concatenate two (corners, ids) detection results (either may be empty)."""
    if ids_b is None:
        return corners_a, ids_a
    if ids_a is None:
        return corners_b, ids_b
    return list(corners_a) + list(corners_b), np.vstack([ids_a.reshape(-1, 1), ids_b.reshape(-1, 1)])


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
        put_text_bg(frame, label, (center_x + 8, center_y - 8),
                    scale=0.8, color=(0, 255, 0), thickness=2)


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


def marker_exclusion_boxes(corners, ids, frame_width, frame_height, margin_ratio=0.4):
    """
    Normalised (0-1) bounding box around each detected marker, expanded by
    margin_ratio. Used to keep colour-block detection from ever scanning a
    marker's own pixels: MJPEG's chroma-subsampled compression fringes
    colour at sharp black/white edges (exactly what an ArUco marker's
    border is), which the colour thresholds can pick up as a small "block"
    sitting right on top of the marker.
    """
    boxes = []
    if ids is None or corners is None:
        return boxes
    for c in corners:
        pts = c[0]
        xs = pts[:, 0] / frame_width
        ys = pts[:, 1] / frame_height
        x0, x1 = float(xs.min()), float(xs.max())
        y0, y1 = float(ys.min()), float(ys.max())
        mx, my = (x1 - x0) * margin_ratio, (y1 - y0) * margin_ratio
        boxes.append((x0 - mx, y0 - my, x1 + mx, y1 + my))
    return boxes


def point_in_any_box(x, y, boxes):
    for x0, y0, x1, y1 in boxes:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return True
    return False


def get_homography_from_markers(markers):
    """
    Compute a transform matrix from normalised image coordinates (0–1) to
    world coordinates (mm) using 3 or 4 ArUco corner markers.

    With 4 markers: full perspective transform (cv2.getPerspectiveTransform).
    With 3 markers: affine transform (cv2.getAffineTransform) promoted to 3×3
      so transform_to_world works unchanged via cv2.perspectiveTransform.

    Markers are classified as TL/TR/BL/BR relative to their centroid, then
    mapped to the corners of a MARKER_SPACING_X_MM × MARKER_SPACING_Y_MM rectangle:
        TL → (0, 0)    TR → (wx, 0)
        BL → (0, wy)   BR → (wx, wy)

    Returns a 3×3 float32 matrix, or None if markers are insufficient/degenerate.
    """
    n = len(markers)
    if n < 3:
        return None

    pts = np.array([[m["center_x"], m["center_y"]] for m in markers], dtype=np.float32)
    centroid = pts.mean(axis=0)

    corner_map = {}
    for pt in pts:
        key = (
            "top" if pt[1] < centroid[1] else "bottom",
            "left" if pt[0] < centroid[0] else "right",
        )
        if key in corner_map:
            return None  # two markers in the same quadrant - degenerate
        corner_map[key] = pt

    wx = MARKER_SPACING_X_MM
    wy = MARKER_SPACING_Y_MM
    world_pos = {
        ("top", "left"):     [0,  0 ],
        ("top", "right"):    [wx, 0 ],
        ("bottom", "right"): [wx, wy],
        ("bottom", "left"):  [0,  wy],
    }

    if n == 4:
        if len(corner_map) != 4:
            return None
        order = [("top", "left"), ("top", "right"), ("bottom", "right"), ("bottom", "left")]
        src = np.array([corner_map[k] for k in order], dtype=np.float32)
        dst = np.array([world_pos[k] for k in order], dtype=np.float32)
        try:
            return cv2.getPerspectiveTransform(src, dst)
        except cv2.error:
            return None

    else:  # n == 3
        if len(corner_map) != 3:
            return None
        keys = list(corner_map.keys())
        src = np.array([corner_map[k] for k in keys], dtype=np.float32)
        dst = np.array([world_pos[k] for k in keys], dtype=np.float32)
        try:
            M = cv2.getAffineTransform(src, dst)
            # Promote 2×3 affine to 3×3 so perspectiveTransform works unchanged.
            return np.vstack([M, [0.0, 0.0, 1.0]]).astype(np.float32)
        except cv2.error:
            return None


def transform_to_world(H, nx, ny):
    """Apply homography H to a normalised image point, returning (x_mm, y_mm)."""
    pt = np.array([[[nx, ny]]], dtype=np.float32)
    world = cv2.perspectiveTransform(pt, H)
    return float(world[0][0][0]), float(world[0][0][1])


_gamma_lut_cache = {}


def apply_display_gamma(frame, gamma):
    """
    Brighten a frame for display via gamma correction (output = (input/255)^(1/gamma) * 255).
    Deliberately not a flat brightness/gain boost: gamma lifts shadows and
    midtones while leaving already-bright pixels close to where they were,
    so it doesn't re-clip the glare highlights a low manual exposure was
    set specifically to avoid (see ROBOT_ARM_CAMERA_EXPOSURE).
    """
    if gamma <= 1.0:
        return frame
    lut = _gamma_lut_cache.get(gamma)
    if lut is None:
        inv_gamma = 1.0 / gamma
        lut = np.array([((i / 255.0) ** inv_gamma) * 255 for i in range(256)], dtype="uint8")
        _gamma_lut_cache[gamma] = lut
    return cv2.LUT(frame, lut)


def auto_brighten_for_display(frame):
    """
    Brighten the frame actually shown to the user - entirely separate from
    (and applied after) the detection pipeline above, which always runs on
    its own copy of the raw capture. Computes the gamma needed to lift this
    specific frame's current mean brightness up to DISPLAY_TARGET_BRIGHTNESS,
    instead of using one fixed gamma constant, so the feed reads as roughly
    the same brightness no matter how low ROBOT_ARM_CAMERA_EXPOSURE is set.

    Applied to the HSV V channel only, not the raw BGR channels - gamma on
    BGR directly amplifies any per-channel imbalance into a colour cast
    (got a strong green/magenta tint doing it that way at higher gamma).
    Working in V preserves hue/saturation, so brightening doesn't distort
    colour.
    """
    if DISPLAY_TARGET_BRIGHTNESS <= 0:
        return frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    mean_brightness = float(np.mean(v))
    if mean_brightness <= 1.0 or mean_brightness >= DISPLAY_TARGET_BRIGHTNESS:
        return frame
    gamma = math.log(mean_brightness / 255.0) / math.log(DISPLAY_TARGET_BRIGHTNESS / 255.0)
    gamma = max(1.0, min(gamma, DISPLAY_GAMMA_MAX))
    v = apply_display_gamma(v, gamma)
    return cv2.cvtColor(cv2.merge([h, s, v]), cv2.COLOR_HSV2BGR)


def put_text_bg(img, text, org, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.6,
                color=(255, 255, 255), thickness=1, bg=(0, 0, 0), alpha=0.6, pad=4):
    """Draw text with a semi-transparent dark background for readability."""
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = int(org[0]), int(org[1])
    h, w = img.shape[:2]
    x1 = max(0, x - pad)
    y1 = max(0, y - th - pad)
    x2 = min(w - 1, x + tw + pad)
    y2 = min(h - 1, y + baseline + pad)
    if x2 > x1 and y2 > y1:
        roi = img[y1:y2, x1:x2]
        bg_rect = np.full_like(roi, bg)
        img[y1:y2, x1:x2] = cv2.addWeighted(bg_rect, alpha, roi, 1.0 - alpha, 0)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def draw_coordinate_frame(frame, markers, frame_width, frame_height):
    """
    Draw the mapped workspace boundary and X/Y axes on the high-res annotated frame.
    Works with 3 or 4 visible markers; draws an open polyline when one is missing.
    The origin (0,0) is at the top-left marker; X points right, Y points down.
    """
    pts_norm = np.array(
        [[m["center_x"], m["center_y"]] for m in markers], dtype=np.float32
    )
    centroid = pts_norm.mean(axis=0)

    corner_map = {}
    for pt in pts_norm:
        key = (
            "top" if pt[1] < centroid[1] else "bottom",
            "left" if pt[0] < centroid[0] else "right",
        )
        corner_map[key] = pt

    n_found = len(corner_map)
    if n_found < 3:
        return

    def to_px(pt):
        return (int(pt[0] * frame_width), int(pt[1] * frame_height))

    def lerp_pt(a, b, t):
        return (int(a[0] + (b[0] - a[0]) * t), int(a[1] + (b[1] - a[1]) * t))

    order = [("top", "left"), ("top", "right"), ("bottom", "right"), ("bottom", "left")]
    visible_px = [to_px(corner_map[k]) for k in order if k in corner_map]
    pixel_corners = np.array(visible_px, dtype=np.int32)

    # Closed polygon if all 4 present, open polyline if only 3
    cv2.polylines(frame, [pixel_corners], n_found == 4, (255, 255, 0), 2)

    # X axis (red) from TL→TR, Y axis (green) from TL→BL - only when TL is visible
    tl = corner_map.get(("top", "left"))
    tr = corner_map.get(("top", "right"))
    bl = corner_map.get(("bottom", "left"))
    if tl is not None:
        origin_px = to_px(tl)
        if tr is not None:
            tr_px = to_px(tr)
            cv2.arrowedLine(frame, origin_px, lerp_pt(origin_px, tr_px, 0.25), (0, 0, 255), 3, tipLength=0.3)
            put_text_bg(frame, "X", lerp_pt(origin_px, tr_px, 0.28),
                        scale=0.7, color=(0, 0, 255), thickness=2)
        if bl is not None:
            bl_px = to_px(bl)
            cv2.arrowedLine(frame, origin_px, lerp_pt(origin_px, bl_px, 0.25), (0, 200, 0), 3, tipLength=0.3)
            put_text_bg(frame, "Y", lerp_pt(origin_px, bl_px, 0.28),
                        scale=0.7, color=(0, 200, 0), thickness=2)

    if n_found == 3:
        put_text_bg(frame, "3/4 markers (affine)", (10, frame_height - 10),
                    scale=0.55, color=(0, 200, 255), thickness=2)


marker_tracks = {}


def update_marker_tracks(corners, ids):
    """
    Debounce ArUco detections across frames: a marker only enters the
    confirmed set after being seen MARKER_CONFIRM_FRAMES times in a row,
    and only leaves it after being missing MARKER_HOLD_FRAMES frames in a
    row (using its last known corners in between - the markers are static,
    so a stale position is still the right one). Returns (corners, ids)
    built from the confirmed set only, in the shapes detectMarkers() returns.
    """
    seen_ids = set()
    if ids is not None:
        for index, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            seen_ids.add(marker_id)
            track = marker_tracks.setdefault(
                marker_id, {"present": 0, "absent": 0, "confirmed": False, "corners": None}
            )
            track["present"] = min(track["present"] + 1, MARKER_CONFIRM_FRAMES)
            track["absent"] = 0
            track["corners"] = corners[index]
            if track["present"] >= MARKER_CONFIRM_FRAMES:
                track["confirmed"] = True

    for marker_id, track in list(marker_tracks.items()):
        if marker_id in seen_ids:
            continue
        track["present"] = 0
        track["absent"] += 1
        if track["absent"] >= MARKER_HOLD_FRAMES:
            del marker_tracks[marker_id]

    confirmed_ids = [mid for mid, track in marker_tracks.items() if track["confirmed"]]
    if not confirmed_ids:
        return None, None

    confirmed_corners = [marker_tracks[mid]["corners"] for mid in confirmed_ids]
    confirmed_ids_arr = np.array(confirmed_ids).reshape(-1, 1)
    return confirmed_corners, confirmed_ids_arr


block_tracks = []


def update_block_tracks(detections):
    """
    Debounce colour-block detections across frames the same way as
    update_marker_tracks: a block only appears after TRACK_STABILITY_FRAMES
    consecutive detections, and only disappears after that many consecutive
    misses. Each confirmed block is then given a index, globally unique
    across all colours (not per-colour - X/Y positions are looked up by
    this index elsewhere, so two blocks can never share one), ordered
    top-to-bottom then left-to-right (row-major over centre position).
    Since this is recomputed from current position every frame rather than
    being a fixed per-track slot, a block's index can shift if another
    block appears/disappears above or to the left of it.
    """
    unmatched = list(detections)

    for track in block_tracks:
        track["_matched"] = False
        best_i, best_dist = None, None
        for i, det in enumerate(unmatched):
            if det["color"] != track["color"]:
                continue
            dist = ((det["center_x"] - track["data"]["center_x"]) ** 2
                     + (det["center_y"] - track["data"]["center_y"]) ** 2) ** 0.5
            if dist <= BLOCK_MATCH_MAX_DIST and (best_dist is None or dist < best_dist):
                best_i, best_dist = i, dist

        if best_i is not None:
            track["data"] = unmatched.pop(best_i)
            track["present"] = min(track["present"] + 1, TRACK_STABILITY_FRAMES)
            track["absent"] = 0
            track["_matched"] = True
            if track["present"] >= TRACK_STABILITY_FRAMES:
                track["confirmed"] = True
        else:
            track["present"] = 0
            track["absent"] += 1

    for track in list(block_tracks):
        if track["absent"] >= TRACK_STABILITY_FRAMES:
            block_tracks.remove(track)

    for det in unmatched:
        block_tracks.append(
            {
                "color": det["color"],
                "data": det,
                "present": 1,
                "absent": 0,
                "confirmed": False,
            }
        )

    confirmed = [track for track in block_tracks if track["confirmed"]]
    confirmed.sort(key=lambda track: (track["data"]["center_y"], track["data"]["center_x"]))

    output = []
    for index, track in enumerate(confirmed):
        block = dict(track["data"])
        block["index"] = index
        output.append(block)
    return output


def normalize_brightness(hsv):
    """
    Equalise the V (brightness) channel with CLAHE before HSV thresholding -
    the same contrast trick used for ArUco detection. Shadowed and evenly-lit
    parts of the mat score closer together afterwards, so a block sitting in
    a shadow is less likely to fall below the colour thresholds entirely.
    """
    h, s, v = cv2.split(hsv)
    try:
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        v = clahe.apply(v)
    except Exception:
        pass
    return cv2.merge([h, s, v])


def detect_color_blocks(frame):
    """
    Find coloured-block candidates using HSV thresholds. Returns a list of
    block dicts with the silhouette's bounding box (normalised) and its
    contour (pixels, in this frame's coordinates) under "_contour". The
    world position, size check and rotation are added afterwards by
    measure_block_footprint(), which needs the marker homography.
    Does NOT draw on the frame - call draw_color_blocks() separately.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = normalize_brightness(hsv)
    frame_height, frame_width = frame.shape[:2]
    blocks = []

    for color_name, ranges in COLOR_RANGES.items():
        mask = None
        for low, high in ranges:
            part = cv2.inRange(hsv, np.array(low), np.array(high))
            mask = part if mask is None else cv2.bitwise_or(mask, part)

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            if contour_area < MIN_BLOCK_AREA or contour_area > MAX_BLOCK_AREA:
                continue

            # Coarse silhouette checks on the rotated bounding rect: a cube
            # (top + front face, from the side) is still a compact, mostly
            # filled shape, unlike arm links, cables or reflections.
            (_, _), (rot_w, rot_h), _angle = cv2.minAreaRect(contour)
            if rot_w <= 0 or rot_h <= 0:
                continue
            long_side, short_side = max(rot_w, rot_h), min(rot_w, rot_h)
            if long_side / short_side > BLOCK_MAX_ASPECT_RATIO:
                continue
            solidity = contour_area / (rot_w * rot_h)
            if solidity < BLOCK_MIN_SOLIDITY:
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
                    "_contour": contour,
                }
            )

    return blocks


def contour_bottom_envelope(contour, bin_px=2):
    """
    The lowest contour point in every `bin_px`-wide column: for a cube seen
    from the side these are the edges where it touches the ground, which is
    the only part of the silhouette the ground-plane homography maps
    correctly (the top face sits a cube-height above the plane and would
    land too far away). Returns an (N, 2) float32 array in pixel coords,
    ordered left to right.
    """
    # Rasterise the filled silhouette rather than using the contour's own
    # vertices: findContours' simplified contours have whole columns with
    # only a top-edge vertex, which would read as the "lowest point" there
    # and drag the footprint up onto the cube's top face.
    x, y, w, h = cv2.boundingRect(contour)
    if w <= 0 or h <= 0:
        return np.zeros((0, 2), dtype=np.float32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [contour - np.array([[x, y]])], -1, 255, thickness=cv2.FILLED)
    points = []
    for column in range(0, w, bin_px):
        strip = mask[:, column:column + bin_px]
        rows = np.nonzero(strip.any(axis=1))[0]
        if len(rows) == 0:
            continue
        points.append([x + column + min(bin_px, w - column) / 2.0, y + int(rows.max())])
    return np.array(points, dtype=np.float32)


def measure_block_footprint(block, homography, frame_width, frame_height):
    """
    Work out where a block candidate sits on the mat and how it is turned,
    from the ground-contact edges of its silhouette (see
    contour_bottom_envelope()). Seen obliquely, those edges form the two
    front sides of the cube's square base (an "L"/"V" when the cube is
    turned, a single line when a face squarely faces the camera). Mapped
    through the marker homography into mat millimetres and fitted with a
    minimum-area rectangle:

      - the rectangle's long side is the cube size - the size check that
        replaces the old pixel-area limits, which can't work when a near
        cube is several times bigger in the image than a far one;
      - its orientation is the cube's rotation about the vertical, in the
        mat's frame (folded to -45..45° - a square repeats every 90°);
      - its centre is the base centre, except that the back edges are hidden
        behind the cube, so a squarely-facing cube only shows its front
        edge: the fitted rectangle is then thin and its centre lies on that
        edge. The centre is pushed away from the camera by the missing depth
        ((size - visible depth) / 2) along the viewing direction, which is
        the ground-plane direction from the point at the bottom of the
        image in the same image column (every image column of an unrolled
        camera is a vertical plane through the camera, so that direction
        is exactly "away from the camera" for this block).

    Fills in world_x_mm/world_y_mm (base centre), rotation_deg, size_mm and
    base_corners (normalised image coords of the estimated base square, for
    the overlay). Returns False if the footprint isn't cube-sized.
    """
    contour = block.get("_contour")
    if contour is None or homography is None:
        return False
    bottom = contour_bottom_envelope(contour)
    if len(bottom) < 3:
        return False
    norm = bottom / np.array([frame_width, frame_height], dtype=np.float32)
    world = cv2.perspectiveTransform(norm.reshape(-1, 1, 2), homography).reshape(-1, 2)
    (cx, cy), (rw, rh), angle = cv2.minAreaRect(world.astype(np.float32).reshape(-1, 1, 2))
    long_side, short_side = max(rw, rh), min(rw, rh)
    if long_side <= 0:
        return False
    if abs(long_side - BLOCK_SIZE_MM) > BLOCK_SIZE_TOLERANCE * BLOCK_SIZE_MM:
        return False

    # Viewing direction on the ground plane for this block's image column.
    mid_index = len(bottom) // 2
    front_mid = world[mid_index]
    column_nx = float(norm[mid_index][0])
    toward_camera_pt = np.array(transform_to_world(homography, column_nx, 1.0), dtype=np.float64)
    view_dir = front_mid.astype(np.float64) - toward_camera_pt
    view_len = float(np.linalg.norm(view_dir))
    if view_len > 1e-6:
        view_dir /= view_len
    else:
        view_dir = np.array([0.0, -1.0])

    hidden_depth = max(0.0, BLOCK_SIZE_MM - short_side)
    base_center = np.array([cx, cy], dtype=np.float64) + view_dir * (hidden_depth / 2.0)

    block["world_x_mm"] = round(float(base_center[0]), 1)
    block["world_y_mm"] = round(float(base_center[1]), 1)
    # A square repeats every 90°; fold to -45..45 so a cube that is nearly
    # aligned with the mat reads as a small +/- angle rather than jittering
    # between ~2° and ~88° from frame to frame.
    block["rotation_deg"] = round(((float(angle) + 45.0) % 90.0) - 45.0, 1)
    block["size_mm"] = round(float(long_side), 1)

    # Estimated base square, back in image space for the overlay.
    try:
        half = BLOCK_SIZE_MM / 2.0
        theta = math.radians(float(angle))
        ux = np.array([math.cos(theta), math.sin(theta)])
        uy = np.array([-math.sin(theta), math.cos(theta)])
        square = np.array([
            base_center + ux * half + uy * half,
            base_center - ux * half + uy * half,
            base_center - ux * half - uy * half,
            base_center + ux * half - uy * half,
        ], dtype=np.float32)
        inv = np.linalg.inv(homography)
        img_pts = cv2.perspectiveTransform(square.reshape(-1, 1, 2), inv).reshape(-1, 2)
        block["base_corners"] = [[round(float(px), 4), round(float(py), 4)] for px, py in img_pts]
    except Exception:
        block["base_corners"] = None
    return True


def draw_color_blocks(frame, blocks):
    """
    Draw a border box per block with its index centered inside. Colour name
    and world position are left out of the overlay (they cluttered the feed
    when several blocks sat close together) - the UI shows them in a table
    built from the /vision JSON instead.
    """
    fh, fw = frame.shape[:2]
    for block in blocks:
        x = int((block["center_x"] - block["width"] / 2) * fw)
        y = int((block["center_y"] - block["height"] / 2) * fh)
        w = int(block["width"] * fw)
        h = int(block["height"] * fh)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        # Estimated base square on the mat (shows the position/rotation
        # actually being reported, not just the silhouette).
        corners = block.get("base_corners")
        if corners:
            pts = np.array([[int(px * fw), int(py * fh)] for px, py in corners], dtype=np.int32)
            cv2.polylines(frame, [pts.reshape(-1, 1, 2)], True, (255, 0, 255), 2)

        label = str(block["index"])
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        center_x = x + w // 2
        center_y = y + h // 2
        put_text_bg(frame, label, (center_x - tw // 2, center_y + th // 2),
                    scale=0.7, color=(0, 255, 255), thickness=2)


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
                camera.set(cv2.CAP_PROP_FPS, DETECTION_FPS)
                camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if CAMERA_MANUAL_EXPOSURE:
                    try:
                        # V4L2 exposure_auto: 1 = manual, 3 = aperture-priority (auto).
                        camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
                        camera.set(cv2.CAP_PROP_EXPOSURE, float(CAMERA_MANUAL_EXPOSURE))
                    except Exception as exc:
                        print(f"Camera: could not set manual exposure: {exc}", file=sys.stderr)
                if CAMERA_MANUAL_BRIGHTNESS:
                    try:
                        camera.set(cv2.CAP_PROP_BRIGHTNESS, float(CAMERA_MANUAL_BRIGHTNESS))
                    except Exception as exc:
                        print(f"Camera: could not set manual brightness: {exc}", file=sys.stderr)
                if CAMERA_MANUAL_GAMMA:
                    try:
                        camera.set(cv2.CAP_PROP_GAMMA, float(CAMERA_MANUAL_GAMMA))
                    except Exception as exc:
                        print(f"Camera: could not set manual gamma: {exc}", file=sys.stderr)

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
    """
    Improve contrast so printed markers are easier to detect, including in
    shadowed areas of the mat. A smaller tile grid than the colour-block
    normalisation (see normalize_brightness()) adapts more locally, and a
    higher clip limit pushes shadowed regions harder - markers are small
    and high-contrast (black/white) to begin with, so this tolerates more
    aggressive equalisation than colour thresholding does.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    try:
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
        gray = clahe.apply(gray)
    except Exception:
        pass
    return gray


_undistort_maps = {}
_calibration = None
_calibration_loaded = False


def load_camera_calibration():
    """
    Lens model used to undistort every frame, or None for no correction.
    Prefers CAMERA_CALIBRATION_FILE (written by calibrate-camera.py: keys
    "camera_matrix" 3x3, "dist_coeffs" list, "image_width"/"image_height",
    optional "model": "fisheye"), then the manual ROBOT_ARM_LENS_K1/K2
    fallback, which assumes the optical centre is the image centre and a
    focal length of half the frame width.
    """
    global _calibration, _calibration_loaded
    if _calibration_loaded:
        return _calibration
    _calibration_loaded = True
    if CAMERA_CALIBRATION_FILE and os.path.isfile(CAMERA_CALIBRATION_FILE):
        try:
            with open(CAMERA_CALIBRATION_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
            _calibration = {
                "model": data.get("model", "pinhole"),
                "camera_matrix": np.array(data["camera_matrix"], dtype=np.float64).reshape(3, 3),
                "dist_coeffs": np.array(data["dist_coeffs"], dtype=np.float64).reshape(-1, 1),
                "image_width": int(data.get("image_width", 0)),
                "image_height": int(data.get("image_height", 0)),
                "source": CAMERA_CALIBRATION_FILE,
            }
            print(f"Lens calibration loaded from {CAMERA_CALIBRATION_FILE} ({_calibration['model']})", file=sys.stderr)
            return _calibration
        except Exception as exc:
            print(f"Lens calibration file {CAMERA_CALIBRATION_FILE} ignored: {exc}", file=sys.stderr)
    if LENS_K1 != 0 or LENS_K2 != 0:
        _calibration = {"model": "manual", "k1": LENS_K1, "k2": LENS_K2, "source": "ROBOT_ARM_LENS_K1/K2"}
        print(f"Lens correction: manual k1={LENS_K1} k2={LENS_K2}", file=sys.stderr)
    return _calibration


def undistort_frame(frame):
    """Apply the lens correction (if any) with a cached remap for this frame size."""
    calibration = load_camera_calibration()
    if calibration is None:
        return frame
    h, w = frame.shape[:2]
    maps = _undistort_maps.get((w, h))
    if maps is None:
        try:
            if calibration["model"] == "manual":
                f = w / 2.0
                K = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], dtype=np.float64)
                D = np.array([calibration["k1"], calibration["k2"], 0, 0, 0], dtype=np.float64)
                maps = cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_16SC2)
            else:
                K = calibration["camera_matrix"].copy()
                cw, ch = calibration["image_width"], calibration["image_height"]
                if cw and ch and (cw != w or ch != h):
                    # Calibrated at a different resolution - scale the intrinsics.
                    K[0, :] *= w / float(cw)
                    K[1, :] *= h / float(ch)
                D = calibration["dist_coeffs"]
                if calibration["model"] == "fisheye":
                    maps = cv2.fisheye.initUndistortRectifyMap(K, D[:4], np.eye(3), K, (w, h), cv2.CV_16SC2)
                else:
                    maps = cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_16SC2)
        except Exception as exc:
            print(f"Lens correction disabled (could not build remap): {exc}", file=sys.stderr)
            maps = False
        _undistort_maps[(w, h)] = maps
    if not maps:
        return frame
    return cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)


def process_frame(frame, detectors):
    # Lens correction first, so markers, blocks and the overlay all live in
    # the same (straightened) image and the homography is a true plane map.
    frame = undistort_frame(frame)
    annotated = frame.copy()
    frame_height, frame_width = annotated.shape[:2]
    dictionary_name = None
    corners, ids = None, None
    markers = []
    blocks = []
    homography = None

    # ArUco detection at full capture resolution for accurate corner positions.
    try:
        gray = prepare_gray_for_aruco(annotated)
        if detectors:
            corners, ids, dictionary_name = find_aruco_markers(gray, detectors)
            corners, ids = filter_expected_markers(corners, ids)
            # Far-marker fallback for any expected ID the normal pass missed
            # (runs on the clean frame, not the annotated copy).
            found_ids = [int(v) for v in ids.flatten()] if ids is not None else []
            far_corners, far_ids = find_far_markers(frame, detectors, found_ids)
            corners, ids = merge_marker_detections(corners, ids, far_corners, far_ids)
        corners, ids = update_marker_tracks(corners, ids)
        if ids is not None and len(ids) > 0:
            draw_aruco_overlays(annotated, corners, ids)
        markers = marker_centers(corners, ids, frame_width, frame_height)
    except Exception as exc:
        print(f"ArUco processing error: {exc}", file=sys.stderr)
        annotated = frame.copy()

    # With 3 or 4 markers, compute image→world transform and draw the coordinate
    # frame. 4 markers = full perspective; 3 markers = affine fallback.
    if len(markers) >= 3:
        homography = get_homography_from_markers(markers)
        if homography is not None:
            draw_coordinate_frame(annotated, markers, frame_width, frame_height)

    # Downscale annotated frame to stream resolution for output.
    stream_frame = cv2.resize(
        annotated, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_AREA
    )

    # Colour detection on a clean downscale of the ORIGINAL frame so ArUco
    # overlay lines are never visible to the colour detector.
    if COLOR_DETECTION_ENABLED:
        try:
            clean_small = cv2.resize(
                frame, (STREAM_WIDTH, STREAM_HEIGHT), interpolation=cv2.INTER_AREA
            )
            raw_blocks = detect_color_blocks(clean_small)
            # Drop anything overlapping a detected marker's own footprint -
            # see marker_exclusion_boxes() for why (JPEG colour fringing at
            # the marker's edges getting mistaken for a small block).
            exclusion_boxes = marker_exclusion_boxes(corners, ids, frame_width, frame_height)
            if exclusion_boxes:
                raw_blocks = [
                    b for b in raw_blocks
                    if not point_in_any_box(b["center_x"], b["center_y"], exclusion_boxes)
                ]
            # Annotate each block with its world position, then drop anything
            # outside the marker-defined mat - without this, HSV colour
            # matches on the arm itself (or the background) get reported as
            # blocks just like a real one on the mat. With fewer than 3
            # markers visible there's no homography to check against, so
            # every detection is discarded rather than reported unfiltered.
            if homography is not None:
                in_bounds = []
                margin = BLOCK_WORKSPACE_MARGIN_MM
                for block in raw_blocks:
                    # Position/size/rotation from the ground-contact edges -
                    # also the cube-size check that rejects non-blocks.
                    if not measure_block_footprint(block, homography, STREAM_WIDTH, STREAM_HEIGHT):
                        continue
                    wx, wy = block["world_x_mm"], block["world_y_mm"]
                    if -margin <= wx <= MARKER_SPACING_X_MM + margin and -margin <= wy <= MARKER_SPACING_Y_MM + margin:
                        in_bounds.append(block)
                filtered_blocks = in_bounds
            else:
                filtered_blocks = []
            for block in filtered_blocks:
                block.pop("_contour", None)
            blocks = update_block_tracks(filtered_blocks)
            draw_color_blocks(stream_frame, blocks)
        except Exception as exc:
            print(f"Colour detection error: {exc}", file=sys.stderr)

    # Brighten for display only - detection above already ran on the darker
    # capture (see ROBOT_ARM_CAMERA_EXPOSURE), so this has no effect on
    # accuracy, only on how the feed looks to the operator.
    stream_frame = auto_brighten_for_display(stream_frame)

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
            f"OpenCV {cv2.__version__} - ArUco dictionaries: {len(detectors)}",
            file=sys.stderr,
        )
    except Exception as exc:
        detectors = []
        print(f"ArUco disabled ({exc}) - streaming video only", file=sys.stderr)

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

                time.sleep(1.0 / DETECTION_FPS)
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
