#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from edge_core.config import load_config, save_config


def parse_args():
    parser = argparse.ArgumentParser(description="Calibrate robot board mapping (a1, a8, h1, h8)")
    parser.add_argument("--config", default="edge/config/edge.yaml", help="Path to edge.yaml")
    return parser.parse_args()


def _ask_point(label: str):
    raw = input(f"Enter {label} as 'x y' in mm: ").strip()
    parts = raw.split()
    if len(parts) != 2:
        raise ValueError("Expected two numbers")
    return [float(parts[0]), float(parts[1])]


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)

    print("Jog the robot to each corner and enter the XY coordinates.")
    a1 = _ask_point("a1")
    a8 = _ask_point("a8")
    h1 = _ask_point("h1")
    h8 = _ask_point("h8")

    cfg.setdefault("calibration", {})
    cfg["calibration"]["robot_board"] = {"a1": a1, "a8": a8, "h1": h1, "h8": h8}
    save_config(args.config, cfg)
    print("Saved robot board mapping to config")


if __name__ == "__main__":
    main()
