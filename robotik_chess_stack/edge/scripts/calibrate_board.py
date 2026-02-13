#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from edge_core.config import load_config, save_config
from edge_core.vision.board_pose import detect_board_pose


def parse_args():
    parser = argparse.ArgumentParser(description="Calibrate board pose using ArUco markers")
    parser.add_argument("--config", default="edge/config/edge.yaml", help="Path to edge.yaml")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)
    cam_cfg = cfg["cameras"]["devices"][0]
    cap = cv2.VideoCapture(cam_cfg["path"])
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {cam_cfg['path']}")

    print("Press 's' to save pose, 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        pose = detect_board_pose(
            frame,
            cfg["vision"]["aruco"]["dict"],
            cfg["vision"]["aruco"]["ids"],
            cfg["vision"]["board"]["square_size_mm"],
        )
        if pose:
            for x, y in pose.corners_px:
                cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
        cv2.imshow("board_calibration", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s") and pose:
            cfg["vision"]["board_pose"]["homography"] = pose.homography_img_to_board.tolist()
            cfg["vision"]["board_pose"]["corners_px"] = pose.corners_px
            save_config(args.config, cfg)
            print("Saved board pose to config")
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
