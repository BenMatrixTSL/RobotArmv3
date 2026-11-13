#!/usr/bin/env python3
"""
Renders the Matrix boot-splash picture for the kiosk desktop background.

It draws the same picture as the Plymouth boot theme (matrixtsl.script): the
dark vertical gradient, with the logo scaled to at most 45% of the screen width,
centred, 40 px above the middle. Used by kiosk-boot-splash.sh, so the screen
looks unchanged from the boot splash until the kiosk window covers it.

Usage: kiosk-boot-splash.py <logo.png> <width> <height> <out.png>
Needs Pillow (python3-pil), which the boot-splash installer already installs.
"""
import sys
from PIL import Image

# Plymouth: Window.SetBackgroundTopColor(0.051, 0.075, 0.125) / Bottom(0.024, 0.035, 0.067)
TOP = (13, 19, 32)
BOTTOM = (6, 9, 17)


def render(logo_path, width, height, out_path):
    # Vertical gradient, top colour to bottom colour
    mask = Image.linear_gradient('L').resize((width, height))
    img = Image.composite(Image.new('RGB', (width, height), BOTTOM),
                          Image.new('RGB', (width, height), TOP), mask)

    logo = Image.open(logo_path).convert('RGBA')
    logo_w, logo_h = logo.size
    max_logo_w = width * 0.45
    if logo_w > max_logo_w:
        scale = max_logo_w / logo_w
        logo = logo.resize((int(logo_w * scale), int(logo_h * scale)), Image.LANCZOS)
        logo_w, logo_h = logo.size

    x = int((width - logo_w) / 2)
    y = int(height / 2 - logo_h / 2 - 40)
    img.paste(logo, (x, y), logo)
    img.save(out_path)


if __name__ == '__main__':
    if len(sys.argv) != 5:
        sys.exit(__doc__)
    render(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4])
