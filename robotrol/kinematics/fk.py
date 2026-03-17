"""Forward Kinematics for 6-DOF DH robots.

Pure math functions, no state, no GUI dependencies.
Ported from tcp_pose_module_v3.py (lines 444-518).
"""

from __future__ import annotations

import math
from typing import Dict, List, Tuple

from robotrol.kinematics.dh_model import DHModel, fallback_dh_rows_from_geom


def _dh_transform(theta_rad: float, d_mm: float, a_mm: float, alpha_rad: float) -> List[List[float]]:
    """Standard DH transformation matrix (4x4)."""
    ct = math.cos(theta_rad)
    st = math.sin(theta_rad)
    ca = math.cos(alpha_rad)
    sa = math.sin(alpha_rad)
    return [
        [ct, -st * ca, st * sa, a_mm * ct],
        [st, ct * ca, -ct * sa, a_mm * st],
        [0.0, sa, ca, d_mm],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul4(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    """4x4 matrix multiplication."""
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i][k] * B[k][j]
            out[i][j] = s
    return out


def fk6_forward_mm(
    geom: Dict[str, float],
    joints_list: List[float],
    *,
    dh_model: DHModel | None = None,
) -> Tuple[float, float, float, float, float, float, float]:
    """6-DOF forward kinematics using DH convention.

    Args:
        geom: Geometry dict with L1..L4 (used for fallback rows if no dh_model).
        joints_list: List of post-transformed joint angles in degrees,
                     one per DH row, in DH-row order.
        dh_model: Optional DHModel instance. If provided, uses its DH rows
                  and post_transform for mirror_x. Otherwise uses fallback rows.

    Returns:
        (X_mm, Y_mm, Z_mm, Roll_deg, Pitch_deg, Yaw_deg, Tilt_deg)
        Tilt_deg is a legacy alias (= -Pitch).
    """
    if dh_model is not None and dh_model.dh_rows:
        dh_rows = dh_model.dh_rows
        post = dh_model.post_transform
    else:
        dh_rows = fallback_dh_rows_from_geom(geom)
        post = {}

    # Build chain
    T = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]

    for i, row in enumerate(dh_rows):
        model_deg = joints_list[i] if i < len(joints_list) else 0.0
        theta_deg = model_deg + row["theta_offset_deg"]
        th = math.radians(theta_deg)
        al = math.radians(row["alpha_deg"])
        d = float(row["d_mm"])
        a = float(row["a_mm"])
        Ti = _dh_transform(th, d, a, al)
        T = _matmul4(T, Ti)

    # Position
    x = T[0][3]
    y = T[1][3]
    z = T[2][3]

    # Orientation (ZYX Euler)
    r11, r21, r31 = T[0][0], T[1][0], T[2][0]
    r32, r33 = T[2][1], T[2][2]

    pitch = math.degrees(math.atan2(-r31, math.sqrt(r11 * r11 + r21 * r21)))
    roll = math.degrees(math.atan2(r32, r33))
    yaw = math.degrees(math.atan2(r21, r11))

    # Mirror X if configured
    if post.get("mirror_x"):
        x = -x

    # Tilt is legacy alias for -Pitch
    tilt = -pitch

    return x, y, z, roll, pitch, yaw, tilt
