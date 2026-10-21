#!/usr/bin/env python3
"""
Camera lens calibration for camera-vision.py.

Works out the camera's intrinsics and lens distortion from photos of a
printed chessboard, and writes them to a JSON file camera-vision.py loads at
start-up (ROBOT_ARM_CAMERA_CALIBRATION, default
/etc/robot-arm-st3215/camera-calibration.json) to undistort every frame
before marker/block detection. Do this once per camera; redo it if the lens
or capture resolution changes.

Print a chessboard (e.g. https://github.com/opencv/opencv/blob/4.x/doc/pattern.png
- a 9x6 inner-corner board; any size paper is fine, it must just be flat).

Capture mode (the camera service must be stopped first - it holds the camera):

    sudo systemctl stop robot-arm-camera
    python3 calibrate-camera.py capture --count 15 --out /tmp/calib
    # hold the board in front of the camera; a photo is taken every 2 s once
    # the board is found - tilt it, move it to every part of the frame,
    # especially the corners and edges where the distortion is strongest.
    sudo systemctl start robot-arm-camera

Solve mode (from the captured photos, or any folder of chessboard photos):

    python3 calibrate-camera.py solve --images /tmp/calib --cols 9 --rows 6 \
        --output /tmp/camera-calibration.json
    sudo mkdir -p /etc/robot-arm-st3215
    sudo cp /tmp/camera-calibration.json /etc/robot-arm-st3215/camera-calibration.json
    sudo systemctl restart robot-arm-camera

--cols/--rows are the board's INNER corner counts (a 10x7-square board has
9x6 inner corners). Add --fisheye for a very wide lens if the standard model
reports a large RMS error (> ~1 px); the standard model with k1..k3 handles
most USB wide-angle cameras. --preview writes an undistorted copy of the
first image next to the output so the result can be eyeballed.
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np


def capture(args):
    device = args.device
    source = int(device.replace("/dev/video", "")) if device.startswith("/dev/video") else device
    cam = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if not cam.isOpened():
        sys.exit(f"Could not open {device} - is robot-arm-camera stopped?")
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    os.makedirs(args.out, exist_ok=True)
    pattern = (args.cols, args.rows)
    taken = 0
    last = 0.0
    print(f"Capturing {args.count} chessboard views to {args.out} - show the {args.cols}x{args.rows} board...")
    while taken < args.count:
        ok, frame = cam.read()
        if not ok:
            time.sleep(0.1)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, _ = cv2.findChessboardCorners(
            gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
        )
        now = time.time()
        if found and now - last >= args.interval:
            path = os.path.join(args.out, f"calib_{taken:02d}.png")
            cv2.imwrite(path, frame)
            taken += 1
            last = now
            print(f"  {taken}/{args.count} saved {path}")
        elif not found and now - last >= 2.0:
            print("  board not found - move it fully into view", file=sys.stderr)
            last = now
    cam.release()
    print("Done. Now run: python3 calibrate-camera.py solve --images", args.out)


def solve(args):
    files = sorted(glob.glob(os.path.join(args.images, "*.png")) + glob.glob(os.path.join(args.images, "*.jpg")))
    if not files:
        sys.exit(f"No .png/.jpg images in {args.images}")
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm
    obj_points, img_points = [], []
    size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH)
        if not found:
            print(f"  skipped (no board): {f}")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        obj_points.append(objp)
        img_points.append(corners)
        print(f"  ok: {f}")
    if len(obj_points) < 5:
        sys.exit(f"Only {len(obj_points)} usable views - need at least 5 (10-20 is better)")

    if args.fisheye:
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            [o.reshape(-1, 1, 3) for o in obj_points], img_points, size, K, D, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
        model = "fisheye"
    else:
        rms, K, D, _, _ = cv2.calibrateCamera(obj_points, img_points, size, None, None)
        model = "pinhole"

    result = {
        "model": model,
        "camera_matrix": K.tolist(),
        "dist_coeffs": D.flatten().tolist(),
        "image_width": size[0],
        "image_height": size[1],
        "rms_reprojection_error_px": float(rms),
        "views": len(obj_points),
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    print(f"\nWrote {args.output}  ({model}, {len(obj_points)} views, RMS reprojection error {rms:.3f} px)")
    if rms > 1.0:
        print("RMS above 1 px - try more/better views, or --fisheye for a very wide lens.")

    if args.preview:
        img = cv2.imread(files[0])
        h, w = img.shape[:2]
        if model == "fisheye":
            m1, m2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)
        else:
            m1, m2 = cv2.initUndistortRectifyMap(K, D, None, K, (w, h), cv2.CV_16SC2)
        out = cv2.remap(img, m1, m2, cv2.INTER_LINEAR)
        preview = os.path.splitext(args.output)[0] + "-preview.png"
        cv2.imwrite(preview, np.hstack([img, out]))
        print(f"Preview (original | undistorted): {preview}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)
    c = sub.add_parser("capture", help="take chessboard photos from the camera")
    c.add_argument("--device", default=os.environ.get("ROBOT_ARM_CAMERA_DEVICE", "/dev/video0"))
    c.add_argument("--width", type=int, default=int(os.environ.get("ROBOT_ARM_CAPTURE_WIDTH", "1280")))
    c.add_argument("--height", type=int, default=int(os.environ.get("ROBOT_ARM_CAPTURE_HEIGHT", "720")))
    c.add_argument("--count", type=int, default=15)
    c.add_argument("--interval", type=float, default=2.0, help="seconds between photos")
    c.add_argument("--cols", type=int, default=9, help="inner corners across")
    c.add_argument("--rows", type=int, default=6, help="inner corners down")
    c.add_argument("--out", default="/tmp/calib")
    s = sub.add_parser("solve", help="compute the calibration from chessboard photos")
    s.add_argument("--images", default="/tmp/calib")
    s.add_argument("--cols", type=int, default=9)
    s.add_argument("--rows", type=int, default=6)
    s.add_argument("--square-mm", type=float, default=25.0, help="square size (only scales extrinsics)")
    s.add_argument("--fisheye", action="store_true")
    s.add_argument("--preview", action="store_true")
    s.add_argument("--output", default="/tmp/camera-calibration.json")
    args = parser.parse_args()
    if args.mode == "capture":
        capture(args)
    else:
        solve(args)


if __name__ == "__main__":
    main()
