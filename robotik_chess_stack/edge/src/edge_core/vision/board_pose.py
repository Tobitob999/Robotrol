from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class BoardPose:
    homography_img_to_board: np.ndarray
    homography_board_to_img: np.ndarray
    corners_px: List[Tuple[float, float]]
    confidence: float


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


def _get_aruco_dict(name: str):
    if name not in ARUCO_DICTS:
        raise ValueError(f"Unsupported ArUco dict: {name}")
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[name])


def detect_board_pose(
    frame,
    dict_name: str,
    marker_ids: List[int],
    square_size_mm: float,
) -> Optional[BoardPose]:
    aruco_dict = _get_aruco_dict(dict_name)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _rejected = detector.detectMarkers(frame)
    if ids is None or len(ids) < 4:
        return None

    id_to_corner = {}
    for corner, marker_id in zip(corners, ids.flatten().tolist()):
        id_to_corner[int(marker_id)] = corner[0].mean(axis=0)

    found = [mid for mid in marker_ids if mid in id_to_corner]
    if len(found) < 4:
        return None

    img_pts = np.array([id_to_corner[mid] for mid in marker_ids], dtype=np.float32)

    board_size = 8 * float(square_size_mm)
    board_pts = np.array(
        [
            [0.0, 0.0],
            [board_size, 0.0],
            [board_size, board_size],
            [0.0, board_size],
        ],
        dtype=np.float32,
    )

    H, _ = cv2.findHomography(img_pts, board_pts)
    if H is None:
        return None

    H_inv = np.linalg.inv(H)
    confidence = len(found) / 4.0
    return BoardPose(
        homography_img_to_board=H,
        homography_board_to_img=H_inv,
        corners_px=[tuple(pt) for pt in img_pts.tolist()],
        confidence=confidence,
    )
