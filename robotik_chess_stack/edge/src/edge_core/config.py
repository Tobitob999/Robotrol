from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict

import yaml


DEFAULT_CONFIG: Dict[str, Any] = {
    "edge": {
        "log_dir": "/var/log/edge",
        "trial_log": "/var/log/edge/trials.jsonl",
        "image_ring_dir": "/var/log/edge/images",
        "image_ring_max": 50,
        "default_square": "a2",
    },
    "serial": {
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "timeout_s": 2.0,
        "status_interval_s": 0.5,
    },
    "cameras": {
        "devices": [
            {
                "name": "board",
                "path": "/dev/v4l/by-id/REPLACE_ME",
                "width": 1280,
                "height": 720,
                "fps": 30,
            }
        ]
    },
    "vision": {
        "aruco": {
            "dict": "DICT_4X4_50",
            "ids": [0, 1, 2, 3],
            "marker_length_mm": 30,
        },
        "board": {
            "square_size_mm": 40,
            "orientation": "a1_bottom_left",
        },
        "board_pose": {
            "homography": None,
            "corners_px": None,
        },
        "occupancy": {
            "mode": "board_pose",
            "threshold": 18.0,
            "min_confidence": 0.6,
        },
    },
    "skills": {
        "safe_z_mm": 80.0,
        "pick_z_mm": 5.0,
        "place_z_mm": 6.0,
        "feedrate_mm_min": 1500.0,
        "accel_mm_s2": 200.0,
        "clamps": {
            "dx_mm": [-2.0, 2.0],
            "dy_mm": [-2.0, 2.0],
            "dz_pick_mm": [-2.0, 2.0],
            "dz_place_mm": [-2.0, 2.0],
            "yaw_deg": [-5.0, 5.0],
            "v_approach": [300.0, 2000.0],
            "v_lift": [300.0, 2000.0],
            "v_place": [300.0, 2000.0],
            "dwell_close_ms": [100, 800],
            "dwell_release_ms": [100, 800],
        },
    },
    "safety": {
        "workspace": {
            "x_mm": [0.0, 300.0],
            "y_mm": [0.0, 300.0],
            "z_mm": [0.0, 200.0],
        },
        "max_feedrate_mm_min": 3000.0,
        "phase_timeout_s": 20.0,
    },
    "recovery": {
        "retries": 2,
        "unlock_on_alarm": True,
        "home_on_unlock": False,
    },
    "agent": {
        "enabled": True,
        "base_url": "http://192.168.50.11:8000",
        "psk": "REPLACE_ME",
        "psk_header": "X-PSK",
    },
    "gripper": {
        "enabled": False,
    },
}


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            base[key] = deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: str | Path) -> Dict[str, Any]:
    path = Path(path)
    data: Dict[str, Any] = {}
    if path.exists():
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"Config root must be a mapping, got: {type(raw)}")
        data = raw
    cfg = deep_merge(copy.deepcopy(DEFAULT_CONFIG), data)
    return cfg


def save_config(path: str | Path, cfg: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
