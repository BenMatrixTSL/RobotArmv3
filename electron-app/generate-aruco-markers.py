#!/usr/bin/env python3
"""
Print test ArUco markers for the robot arm camera.

Uses DICT_4X4_50 (the default dictionary in camera-vision.py).

Usage on the Pi or your PC:
  python3 generate-aruco-markers.py
  python3 generate-aruco-markers.py --ids 0 1 2 3 --size 200

PNG files are saved in the current folder. Print at 100% scale (no "fit to page").
"""

import argparse
import os
import sys

try:
    import cv2
except ImportError:
    print("Error: OpenCV not installed. On the Pi: sudo apt install -y python3-opencv")
    sys.exit(1)


def get_dictionary():
    try:
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    except AttributeError:
        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


def make_marker_image(dictionary, marker_id, pixel_size):
    try:
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixel_size)
    except AttributeError:
        return cv2.aruco.drawMarker(dictionary, marker_id, pixel_size)


def main():
    parser = argparse.ArgumentParser(description="Generate ArUco marker PNG files")
    parser.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3])
    parser.add_argument("--size", type=int, default=200, help="Marker image size in pixels")
    parser.add_argument("--output", default=".", help="Output folder")
    args = parser.parse_args()

    dictionary = get_dictionary()
    os.makedirs(args.output, exist_ok=True)

    for marker_id in args.ids:
        image = make_marker_image(dictionary, marker_id, args.size)
        filename = os.path.join(args.output, f"aruco_4x4_50_id{marker_id}.png")
        cv2.imwrite(filename, image)
        print("Saved:", filename)

    print("")
    print("Dictionary: DICT_4X4_50")
    print("Print at 100% scale. A 50 mm square marker works well in the work area.")


if __name__ == "__main__":
    main()
