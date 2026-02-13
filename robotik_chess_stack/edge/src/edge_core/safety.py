from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class LimitViolation(Exception):
    message: str

    def __str__(self) -> str:
        return self.message


def clamp_value(value: float, bounds: Tuple[float, float]) -> float:
    low, high = float(bounds[0]), float(bounds[1])
    return max(low, min(high, float(value)))


def clamp_theta(theta: Dict[str, float], clamps: Dict[str, Tuple[float, float]]) -> Dict[str, float]:
    clamped = {}
    for key, value in theta.items():
        if key in clamps:
            clamped[key] = clamp_value(float(value), clamps[key])
        else:
            clamped[key] = float(value)
    return clamped


def check_workspace(x: float, y: float, z: float, bounds: Dict[str, Tuple[float, float]]) -> None:
    for axis, value in (("x_mm", x), ("y_mm", y), ("z_mm", z)):
        low, high = bounds[axis]
        if not (low <= value <= high):
            raise LimitViolation(f"Workspace limit violated for {axis}: {value} not in [{low}, {high}]")


def enforce_feedrate(feedrate: float, max_feedrate: float) -> float:
    return min(float(feedrate), float(max_feedrate))
