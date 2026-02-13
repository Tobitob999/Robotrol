from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import cv2


@dataclass
class CameraSpec:
    name: str
    path: str
    width: int
    height: int
    fps: int


class CameraDevice:
    def __init__(self, spec: CameraSpec):
        self.spec = spec
        self.cap: Optional[cv2.VideoCapture] = None

    def open(self) -> None:
        self.cap = cv2.VideoCapture(self.spec.path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.spec.name} at {self.spec.path}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.spec.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.spec.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.spec.fps)

    def read(self):
        if not self.cap:
            raise RuntimeError(f"Camera {self.spec.name} not opened")
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError(f"Failed to read camera {self.spec.name}")
        return frame

    def close(self) -> None:
        if self.cap:
            self.cap.release()
        self.cap = None


class CameraManager:
    def __init__(self, specs: List[CameraSpec]):
        self.devices = [CameraDevice(spec) for spec in specs]

    def open_all(self) -> None:
        for device in self.devices:
            device.open()

    def close_all(self) -> None:
        for device in self.devices:
            device.close()

    def read_all(self) -> Dict[str, "cv2.Mat"]:
        frames = {}
        for device in self.devices:
            frames[device.spec.name] = device.read()
        return frames
