#!/usr/bin/env python3
"""Generate the ArUco texture of the pick box: marker id 1 of DICT_ARUCO_ORIGINAL with a white border.

The box faces are 0.07 m wide and the marker itself 0.05 m, so the texture is the marker (5/7 of the side)
centred on white. Run once; the PNG is committed.

    python3 scripts/make_markers.py
"""
from pathlib import Path

import cv2
import numpy as np

MARKER_ID, SIDE_PX, FACE_M, MARKER_M = 1, 700, 0.07, 0.05

out = Path(__file__).resolve().parents[1] / "models" / "aruco_box" / "materials" / "textures" / "marker_1.png"
marker_px = int(round(SIDE_PX * MARKER_M / FACE_M))
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
marker = cv2.aruco.drawMarker(dictionary, MARKER_ID, marker_px)
image = np.full((SIDE_PX, SIDE_PX), 255, np.uint8)
offset = (SIDE_PX - marker_px) // 2
image[offset:offset + marker_px, offset:offset + marker_px] = marker
out.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(out), image)
print(f"wrote {out} ({marker_px} px marker on {SIDE_PX} px face)")
