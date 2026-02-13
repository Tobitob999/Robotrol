from __future__ import annotations

from typing import Dict, Tuple

import cv2
import numpy as np

from .board_pose import BoardPose


class OccupancyDetector:
    def __init__(self, square_size_mm: float, threshold: float = 18.0):
        self.square_size_mm = float(square_size_mm)
        self.threshold = float(threshold)
        self.pose: BoardPose | None = None
        self.background_gray: np.ndarray | None = None

    def set_pose(self, pose: BoardPose) -> None:
        self.pose = pose

    def update_background(self, frame) -> None:
        self.background_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def classify(self, frame) -> Tuple[Dict[str, bool], float]:
        if self.pose is None or self.background_gray is None:
            return {}, 0.0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(gray, self.background_gray)
        occupancy: Dict[str, bool] = {}
        for rank in range(8):
            for file_idx in range(8):
                square = f"{chr(ord('a') + file_idx)}{rank + 1}"
                mask = self._square_mask(file_idx, rank, gray.shape)
                if mask is None:
                    occupancy[square] = False
                    continue
                score = float(diff[mask].mean()) if mask.any() else 0.0
                occupancy[square] = score > self.threshold
        return occupancy, 1.0

    def _square_mask(self, file_idx: int, rank: int, shape) -> np.ndarray | None:
        if self.pose is None:
            return None
        x0 = file_idx * self.square_size_mm
        y0 = rank * self.square_size_mm
        x1 = (file_idx + 1) * self.square_size_mm
        y1 = (rank + 1) * self.square_size_mm
        pts_board = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], dtype=np.float32)
        pts_board = pts_board.reshape(-1, 1, 2)
        pts_img = cv2.perspectiveTransform(pts_board, self.pose.homography_board_to_img)
        pts_img = np.round(pts_img).astype(np.int32)
        mask = np.zeros(shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, pts_img, 255)
        return mask.astype(bool)
