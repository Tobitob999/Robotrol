#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from edge_core.config import load_config
from edge_core.hw.fluidnc import FluidNC
from edge_core.vision.board_pose import detect_board_pose


def parse_args():
    parser = argparse.ArgumentParser(description="Edge diagnostics")
    parser.add_argument("--config", default="edge/config/edge.yaml", help="Path to edge.yaml")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)

    # Serial test
    serial_cfg = cfg["serial"]
    fluid = FluidNC(serial_cfg["port"], serial_cfg["baudrate"], serial_cfg["timeout_s"])
    try:
        fluid.connect()
        fluid.send_line("?", wait_ok=False)
        print("Serial OK")
    finally:
        fluid.disconnect()

    # Camera test
    cam_cfg = cfg["cameras"]["devices"][0]
    cap = cv2.VideoCapture(cam_cfg["path"])
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera {cam_cfg['path']}")
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Failed to read camera frame")
    pose = detect_board_pose(
        frame,
        cfg["vision"]["aruco"]["dict"],
        cfg["vision"]["aruco"]["ids"],
        cfg["vision"]["board"]["square_size_mm"],
    )
    if pose:
        for x, y in pose.corners_px:
            cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
    cv2.imwrite("diagnostics_frame.jpg", frame)
    cap.release()
    print("Camera OK, saved diagnostics_frame.jpg")


if __name__ == "__main__":
    main()
