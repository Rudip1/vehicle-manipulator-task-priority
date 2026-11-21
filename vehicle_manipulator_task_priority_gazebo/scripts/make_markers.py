#!/usr/bin/env python3
"""Generate the ArUco textures (DICT_ARUCO_ORIGINAL, 0.05 m markers with a white border):

- marker 1 on the pick box: 0.07 m faces, so the marker is 5/7 of the texture;
- marker 2 on the place stand: a 0.065 m square texture centred on each side face.

Run once; the PNGs are committed.

    python3 scripts/make_markers.py
"""
from pathlib import Path

import cv2
import numpy as np

MARKERS = [(1, 0.07, "aruco_box"), (2, 0.065, "aruco_stand")]  # id, texture side [m], model
SIDE_PX, MARKER_M = 700, 0.05

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
for marker_id, face_m, model in MARKERS:
    out = Path(__file__).resolve().parents[1] / "models" / model / "materials" / "textures" / f"marker_{marker_id}.png"
    marker_px = int(round(SIDE_PX * MARKER_M / face_m))
    marker = cv2.aruco.drawMarker(dictionary, marker_id, marker_px)
    image = np.full((SIDE_PX, SIDE_PX), 255, np.uint8)
    offset = (SIDE_PX - marker_px) // 2
    image[offset:offset + marker_px, offset:offset + marker_px] = marker
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image)
    print(f"wrote {out} ({marker_px} px marker on {SIDE_PX} px texture)")
