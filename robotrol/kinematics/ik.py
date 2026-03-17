"""Inverse Kinematics for 6-DOF robots (Moveo-optimized).

Pure math, no GUI dependencies.
Ported from tcp_world_kinematics_frame.py (IK6 class, lines 33-80).
"""

from __future__ import annotations

import math
from typing import Dict


class IK6:
    """Minimal IK solver for Moveo-like 6-DOF arms."""

    def __init__(self, geom: Dict[str, float]):
        self.L1 = geom["L1"]
        self.L2 = geom["L2"]
        self.L3 = geom["L3"]
        self.L4 = geom["L4"]

    @staticmethod
    def _wrap180(a: float) -> float:
        return ((a + 180.0) % 360.0) - 180.0

    def solve_xyz(
        self,
        X: float,
        Y: float,
        Z: float,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
    ) -> Dict[str, float]:
        """Pure XYZ IK similar to FK convention.

        Args:
            X, Y, Z: Target TCP position in mm.
            pitch_deg: Tool tilt (Moveo convention).
            roll_deg: Wrist roll.

        Returns:
            Dict with joint angles {'A', 'X', 'Y', 'Z', 'B', 'C'} in degrees.
        """
        if abs(X) + abs(Y) < 1e-6:
            A = 0.0
        else:
            A = math.degrees(math.atan2(Y, X))

        r = math.sqrt(X * X + Y * Y)
        phi = math.radians(180.0 - pitch_deg)
        xw = r - self.L4 * math.sin(phi)
        zw = Z - self.L1 - self.L4 * math.cos(phi)

        r2 = xw * xw + zw * zw
        L2, L3 = self.L2, self.L3
        c2 = max(-1.0, min(1.0, (r2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)))
        s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
        q2 = math.atan2(s2, c2)
        q1 = math.atan2(xw, zw) - math.atan2(L3 * s2, L2 + L3 * c2)

        # Wrist
        q3 = (math.pi - math.radians(pitch_deg)) - (q1 + q2)

        return {
            "A": -A,
            "X": math.degrees(q1),
            "Y": math.degrees(q2),
            "Z": math.degrees(q3),
            "B": IK6._wrap180(roll_deg + 180.0),
            "C": 0.0,
        }
