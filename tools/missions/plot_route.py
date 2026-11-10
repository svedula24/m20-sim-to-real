#!/usr/bin/env python3
"""Draw a route's waypoints on its map so the points can be checked by eye before the robot sees them.
  python3 tools/missions/plot_route.py maps/upstairs2026.yaml tools/missions/dock_loop.yaml out.png
Numbered dots (green) joined in order, the first in blue; a dot on a black cell or outside the map is reported and
the exit code is 1. Needs Pillow (laptop tool)."""
import os
import sys

import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import route  # noqa: E402


def main(map_yaml, route_yaml, out_png):
    m = yaml.safe_load(open(map_yaml))
    res, (ox, oy, _) = float(m["resolution"]), m["origin"]
    img = Image.open(os.path.join(os.path.dirname(map_yaml), m["image"])).convert("L")
    w, h = img.size
    px = img.load()
    draw = ImageDraw.Draw(img := img.convert("RGB"))
    wps = route.fill_auto_yaw(route.load_route(route_yaml))
    bad, pts = [], []
    for i, wp in enumerate(wps):
        c, r = (wp.x - ox) / res, h - 1 - (wp.y - oy) / res          # map rows count down from the top
        pts.append((c, r))
        inside = 0 <= c < w and 0 <= r < h
        if not inside or px[int(c), int(r)] < 128:                    # black = occupied (negate 0)
            bad.append(f"{wp.label} ({wp.x}, {wp.y}) -> {'outside' if not inside else 'on an occupied cell'}")
        colour = (40, 90, 220) if i == 0 else (30, 160, 60)
        draw.ellipse([c - 4, r - 4, c + 4, r + 4], fill=colour)
        draw.text((c + 6, r - 6), f"{i} {wp.label}", fill=colour)
    draw.line(pts, fill=(30, 160, 60), width=1)
    img.save(out_png)
    print(f"{out_png}: {len(wps)} waypoints on {w} x {h} cells at {res} m")
    for b in bad:
        print("BAD:", b)
    return 1 if bad else 0


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(__doc__); sys.exit(2)
    sys.exit(main(*sys.argv[1:]))
