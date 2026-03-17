"""
Robotrol v2.0 — Serial protocol helpers.

Extracted from Robotrol_FluidNC_v7_3.py:
  - is_homing_command()       line 193-197
  - parse_status_line()       lines 322-373, 5638-5740
  - parse_settings_line()     $N=value from $$ responses
  - parse_softmax()           lines 89-96, 5619-5629
No tkinter dependency.
"""

from __future__ import annotations

import re
from typing import Any, Dict, Optional, Tuple

from robotrol.config.constants import AXES, RE_HOMING_CMD, SOFTMAX_RE


# ── Re-export from constants for convenience ──────────────────────────────────
def is_homing_command(cmd: str) -> bool:
    """Return True if *cmd* is a hardware homing command ($H, $HX, $HY, ...)."""
    return bool(RE_HOMING_CMD.match((cmd or "").strip()))


# ── Status line parser ────────────────────────────────────────────────────────
def parse_status_line(line: str) -> Dict[str, Any]:
    """Parse a GRBL/FluidNC real-time status line.

    Input format examples::

        <Idle|MPos:0.000,10.000,20.000,0.000,0.000,0.000|FS:0,0>
        <Run|MPos:1.0,2.0,3.0|WCO:0.5,0.5,0.0|Pn:X>
        <Idle|WPos:0.000,0.000,0.000>

    Returns a dict with keys:
        state  — str or None  (e.g. "Idle", "Run", "Alarm")
        mpos   — dict[str, float]  machine positions  (may be empty)
        wpos   — dict[str, float]  work positions      (may be empty)
        wco    — dict[str, float]  work-coordinate offset (may be empty)
        pn     — set[str]          active endstop pins  (may be empty)
        fs     — tuple[int, int] | None   feed & spindle override

    If the line is not a valid status line, all values are empty/None.
    """
    result: Dict[str, Any] = {
        "state": None,
        "mpos": {},
        "wpos": {},
        "wco": {},
        "pn": set(),
        "fs": None,
    }

    if not (line.startswith("<") and line.endswith(">")):
        return result

    payload = line[1:-1]
    parts = payload.split("|")
    if not parts:
        return result

    result["state"] = parts[0]

    for part in parts[1:]:
        if part.startswith("MPos:"):
            nums = part[5:].split(",")
            for idx, ax in enumerate(AXES):
                if idx < len(nums):
                    try:
                        result["mpos"][ax] = float(nums[idx])
                    except ValueError:
                        pass

        elif part.startswith("WPos:"):
            nums = part[5:].split(",")
            for idx, ax in enumerate(AXES):
                if idx < len(nums):
                    try:
                        result["wpos"][ax] = float(nums[idx])
                    except ValueError:
                        pass

        elif part.startswith("WCO:"):
            nums = part[4:].split(",")
            for idx, ax in enumerate(AXES):
                if idx < len(nums):
                    try:
                        result["wco"][ax] = float(nums[idx])
                    except ValueError:
                        pass

        elif part.startswith("Pn:"):
            result["pn"] = set(part[3:])

        elif part.startswith("FS:"):
            try:
                vals = part[3:].split(",")
                result["fs"] = (int(vals[0]), int(vals[1]))
            except (ValueError, IndexError):
                pass

    return result


# ── Settings line parser ($N=value) ───────────────────────────────────────────
_RE_SETTING = re.compile(r"^\$(\d+)=(.+)$")


def parse_settings_line(line: str) -> Optional[Tuple[int, str]]:
    """Parse a GRBL settings response line.

    Input format::

        $130=200.000
        $6=0

    Returns ``(setting_number, value_string)`` or ``None`` if not a settings line.
    The value is returned as-is (string) so the caller can decide on int/float.
    """
    m = _RE_SETTING.match(line.strip())
    if m:
        return int(m.group(1)), m.group(2).strip()
    return None


# ── Soft-limit / MaxTravel parser ─────────────────────────────────────────────
def parse_softmax(line: str) -> Optional[Tuple[str, float]]:
    """Parse a $130..$135 MaxTravel line.

    Uses the SOFTMAX_RE patterns from constants (mapping $130→X .. $135→C).

    Returns ``(axis_letter, max_travel_float)`` or ``None``.
    """
    for ax, rx in SOFTMAX_RE.items():
        m = rx.match(line)
        if m:
            try:
                return ax, float(m.group(1))
            except (ValueError, TypeError):
                pass
    return None
