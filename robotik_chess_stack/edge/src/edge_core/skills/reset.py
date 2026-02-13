from __future__ import annotations

from typing import Dict, Tuple


def run(ctx, context: Dict, theta: Dict) -> Tuple[bool, str]:
    ctx.fluid.send_line("$H")
    return True, ""
