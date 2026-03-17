"""
Robotrol v2.0 — Global constants.

Extracted from Robotrol_FluidNC_v7_3.py (lines 68-97, 177, 193).
All controller-level defaults live here.
"""

import re

# ── Axis definitions ──────────────────────────────────────────────────────────
AXES = ["X", "Y", "Z", "A", "B", "C"]
AXIS_INDICES = {ax: i for i, ax in enumerate(AXES)}

# ── Feed / motion defaults ────────────────────────────────────────────────────
MAX_FEED: float = 15000.0          # mm/min (interpreted as deg/min for rotary)
MAX_HOME_FEED: int = 5000          # freely adjustable
DEFAULT_TIMEOUT: float = 30.0     # seconds per command
MOTION_EPS: float = 0.01          # min. position change for live move

# ── UDP mirror (3-D visualiser) ───────────────────────────────────────────────
UDP_MIRROR_DEFAULT: bool = True
UDP_ADDR_DEFAULT: tuple = ("127.0.0.1", 9999)

# ── Axis limits (fallback when profile has no endstop data) ───────────────────
AXIS_LIMITS_DEFAULT = {ax: (-180.0, 180.0) for ax in AXES}

DEFAULT_ENDSTOP_LIMITS = {
    "X": (-100.0, 100.0),
    "Y": (-100.0, 100.0),
    "Z": (-90.0, 90.0),
    "A": (-90.0, 90.0),
    "B": (-180.0, 180.0),
    "C": (-180.0, 180.0),
}

# ── Profiles known to have no endstops ────────────────────────────────────────
NO_ENDSTOP_PROFILES = {"eb15_red", "eb300"}

# ── Compiled regex patterns ───────────────────────────────────────────────────
GC_RE = re.compile(r"\[GC:(.+)\]")

SOFTMAX_RE = {
    "X": re.compile(r"^\$130=([0-9\.]+)"),
    "Y": re.compile(r"^\$131=([0-9\.]+)"),
    "Z": re.compile(r"^\$132=([0-9\.]+)"),
    "A": re.compile(r"^\$133=([0-9\.]+)"),
    "B": re.compile(r"^\$134=([0-9\.]+)"),
    "C": re.compile(r"^\$135=([0-9\.]+)"),
}

RE_HOMING_CMD = re.compile(r"^\$H([XYZABC])?$", re.IGNORECASE)


# ── Helper ────────────────────────────────────────────────────────────────────
def is_homing_command(cmd: str) -> bool:
    """Return True if *cmd* is a hardware homing command ($H, $HX, $HY, ...)."""
    return bool(RE_HOMING_CMD.match((cmd or "").strip()))


def map_speed(val_0_1000: int) -> int:
    """Linear mapping: 0..1000 -> 0..MAX_FEED."""
    return int(MAX_FEED * max(0, min(val_0_1000, 1000)) / 1000)
