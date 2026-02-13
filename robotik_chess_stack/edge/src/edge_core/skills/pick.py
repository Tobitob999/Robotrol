from __future__ import annotations

import time
from typing import Dict, Tuple

from ..safety import check_workspace, enforce_feedrate


def _move(fluid, x: float | None, y: float | None, z: float | None, feed: float) -> None:
    parts = ["G0"]
    if x is not None:
        parts.append(f"X{x:.3f}")
    if y is not None:
        parts.append(f"Y{y:.3f}")
    if z is not None:
        parts.append(f"Z{z:.3f}")
    parts.append(f"F{feed:.1f}")
    fluid.send_line(" ".join(parts))


def run(ctx, context: Dict, theta: Dict) -> Tuple[bool, str]:
    square = context.get("square")
    if not square:
        return False, "UNKNOWN"
    x, y = ctx.square_to_xy(square)
    x += float(theta.get("dx_mm", 0.0))
    y += float(theta.get("dy_mm", 0.0))

    skills_cfg = ctx.cfg["skills"]
    safety_cfg = ctx.cfg["safety"]
    safe_z = float(skills_cfg["safe_z_mm"])
    pick_z = float(skills_cfg["pick_z_mm"]) + float(theta.get("dz_pick_mm", 0.0))
    feed_approach = enforce_feedrate(float(theta.get("v_approach", skills_cfg["feedrate_mm_min"])), safety_cfg["max_feedrate_mm_min"])
    feed_lift = enforce_feedrate(float(theta.get("v_lift", skills_cfg["feedrate_mm_min"])), safety_cfg["max_feedrate_mm_min"])

    check_workspace(x, y, safe_z, safety_cfg["workspace"])
    check_workspace(x, y, pick_z, safety_cfg["workspace"])

    _move(ctx.fluid, None, None, safe_z, feed_approach)
    _move(ctx.fluid, x, y, safe_z, feed_approach)
    _move(ctx.fluid, None, None, pick_z, feed_approach)
    ctx.gripper.close()
    time.sleep(float(theta.get("dwell_close_ms", 200)) / 1000.0)
    _move(ctx.fluid, None, None, safe_z, feed_lift)

    ok, failure = ctx.verifier.verify_pick(square)
    if not ok:
        return False, failure or "PICK_NO_CHANGE"
    return True, ""
