"""Robotrol v2.0 configuration package."""

from .constants import (
    AXES,
    AXIS_INDICES,
    MAX_FEED,
    MAX_HOME_FEED,
    DEFAULT_TIMEOUT,
    MOTION_EPS,
    UDP_MIRROR_DEFAULT,
    UDP_ADDR_DEFAULT,
    AXIS_LIMITS_DEFAULT,
    DEFAULT_ENDSTOP_LIMITS,
    NO_ENDSTOP_PROFILES,
    GC_RE,
    SOFTMAX_RE,
    RE_HOMING_CMD,
    is_homing_command,
    map_speed,
)
from .app_config import AppConfig
from .profiles import ProfileManager, DEFAULT_PROFILE, PROFILE_FILES

__all__ = [
    "AXES",
    "AXIS_INDICES",
    "MAX_FEED",
    "MAX_HOME_FEED",
    "DEFAULT_TIMEOUT",
    "MOTION_EPS",
    "UDP_MIRROR_DEFAULT",
    "UDP_ADDR_DEFAULT",
    "AXIS_LIMITS_DEFAULT",
    "DEFAULT_ENDSTOP_LIMITS",
    "NO_ENDSTOP_PROFILES",
    "GC_RE",
    "SOFTMAX_RE",
    "RE_HOMING_CMD",
    "is_homing_command",
    "map_speed",
    "AppConfig",
    "ProfileManager",
    "DEFAULT_PROFILE",
    "PROFILE_FILES",
]
