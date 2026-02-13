from __future__ import annotations

from typing import Dict, Tuple

from . import pick as pick_skill
from . import place as place_skill


def run(ctx, context: Dict, theta: Dict) -> Tuple[bool, str]:
    square = context.get("square")
    if not square:
        return False, "UNKNOWN"
    graveyard = ctx.cfg.get("skills", {}).get("graveyard_square", "h8")

    ok, failure = pick_skill.run(ctx, {"square": square}, theta)
    if not ok:
        return False, failure
    ok, failure = place_skill.run(ctx, {"square": graveyard}, theta)
    if not ok:
        return False, failure
    return True, ""
