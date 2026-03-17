"""Vision tab -- Camera preview + Board Pose calibration.

Extracted from monolith lines 966-1017.
"""

from __future__ import annotations

import json
import logging
import os
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict

logger = logging.getLogger(__name__)

_CAMERA_CONFIG_FILE = "camera_config.json"


def _load_camera_config() -> Dict[str, Any]:
    """Load camera configuration from JSON, returning defaults on failure."""
    defaults: Dict[str, Any] = {
        "device_index": 0,
        "width": 640,
        "height": 480,
        "fps": 30,
    }
    try:
        if os.path.isfile(_CAMERA_CONFIG_FILE):
            with open(_CAMERA_CONFIG_FILE, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            if isinstance(data, dict):
                defaults.update(data)
    except (json.JSONDecodeError, OSError) as exc:
        logger.warning("Could not load camera config: %s", exc)
    return defaults


class VisionTab(ttk.Frame):
    """Camera + Board Pose setup tab."""

    def __init__(self, parent: ttk.Widget, app: Any):
        super().__init__(parent)
        self.app = app
        self.cam = None
        self.board_detector = None
        self._build_ui()

    def _build_ui(self) -> None:
        try:
            from robotrol.vision.camera_capture import CameraCapture
            from robotrol.vision.board_pose import BoardPose
        except ImportError as exc:
            logger.warning("Vision modules unavailable: %s", exc)
            ttk.Label(
                self,
                text="Vision modules not available\n(OpenCV / PIL required)",
                font=("Segoe UI", 12),
            ).pack(pady=30)
            return

        # Shared wrapper for clean alignment
        vision_wrap = ttk.Frame(self)
        vision_wrap.pack(padx=6, pady=6)

        cam_cfg = _load_camera_config()
        self.cam = CameraCapture(
            vision_wrap,
            camera_index=int(cam_cfg.get("device_index", 0)),
            width=int(cam_cfg.get("width", 640)),
            height=int(cam_cfg.get("height", 480)),
            fps=int(cam_cfg.get("fps", 30)),
            preview_width=300,
            preview_height=225,
        )
        self.board_detector = BoardPose(
            vision_wrap,
            self.cam,
            pattern_size=(7, 7),
            preview_width=300,
            preview_height=225,
        )

        # Side by side with top-edge alignment
        self.cam.get_frame().pack(side="left", padx=8, pady=6, anchor="n")
        self.board_detector.get_frame().pack(side="left", padx=8, pady=6, anchor="n")

        # --- Start button that disappears after launch ---
        btn_row = ttk.Frame(self)
        btn_row.pack(pady=4)
        btn_start = ttk.Button(btn_row, text="Start Camera")
        btn_start.pack(side=tk.LEFT, padx=4)

        # Optional vision-right toggle (only if app exposes it)
        vision_right_var = getattr(self.app, "vision_right_enabled", None)
        toggle_cb = getattr(self.app, "_toggle_right_vision", None)
        if vision_right_var is not None and callable(toggle_cb):
            chk_right = ttk.Checkbutton(
                btn_row,
                text="Vision rechts",
                variable=vision_right_var,
                command=toggle_cb,
            )
            chk_right.pack(side=tk.LEFT, padx=4)

        def _start_cam() -> None:
            try:
                self.cam.start()
                btn_start.pack_forget()
            except RuntimeError as exc:
                logger.error("Vision start error: %s", exc)

        btn_start.config(command=_start_cam)
