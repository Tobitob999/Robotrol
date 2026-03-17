"""
Robotrol v2.0 — Profile manager.

Handles robot profile loading, saving, change-callbacks, endstop detection
and kinematics settings.  Extracted from config_profiles.py.
"""

from __future__ import annotations

import json
import os
import sys
from typing import Any, Callable, Dict, List, Optional

from .constants import NO_ENDSTOP_PROFILES

# ── Well-known profile files ──────────────────────────────────────────────────
DEFAULT_PROFILE = "Moveo"

PROFILE_FILES: Dict[str, str] = {
    "Moveo": "Moveo.json",
    "EB15_red": "EB15_red.json",
    "EB300": "EB300.json",
}

LEGACY_FILES: Dict[str, str] = {
    "endstops": "endstops.json",
    "cam_to_base": "cam_to_base.json",
    "gamepad": "gamepad_config.json",
    "dh_model": os.path.join("model", "dh.json"),
}


# ── Internal JSON helpers ─────────────────────────────────────────────────────

def _read_json(path: str) -> Any:
    if not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _write_json(path: str, data: Any) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=True)


# ── Base-dir resolution (frozen / dev) ────────────────────────────────────────

def _candidate_base_dirs(base_dir: Optional[str] = None) -> List[str]:
    candidates: List[str] = []
    if base_dir:
        candidates.append(base_dir)
    if getattr(sys, "frozen", False):
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            candidates.append(meipass)
        exe_dir = os.path.dirname(sys.executable)
        if exe_dir:
            candidates.append(exe_dir)
    candidates.append(os.getcwd())
    seen: set[str] = set()
    result: List[str] = []
    for base in candidates:
        if not base:
            continue
        norm = os.path.abspath(base)
        if norm not in seen:
            seen.add(norm)
            result.append(norm)
    return result


def _resolve_write_base_dir(base_dir: Optional[str] = None) -> str:
    if base_dir:
        if getattr(sys, "frozen", False):
            meipass = getattr(sys, "_MEIPASS", None)
            if meipass and os.path.abspath(base_dir) == os.path.abspath(meipass):
                exe_dir = os.path.dirname(sys.executable)
                if exe_dir:
                    return exe_dir
        return base_dir
    if getattr(sys, "frozen", False):
        exe_dir = os.path.dirname(sys.executable)
        if exe_dir:
            return exe_dir
    return os.getcwd()


# ══════════════════════════════════════════════════════════════════════════════
# ProfileManager
# ══════════════════════════════════════════════════════════════════════════════

class ProfileManager:
    """Load, save, inspect and switch robot profiles."""

    def __init__(self, base_dir: str) -> None:
        self._base_dir = base_dir
        self._active_name: str = DEFAULT_PROFILE
        self._active_data: Dict[str, Any] = {}
        self._on_change_callbacks: List[Callable[[str, Dict[str, Any]], None]] = []

    # ── Active profile access ─────────────────────────────────────────────
    @property
    def active_name(self) -> str:
        return self._active_name

    @property
    def active_data(self) -> Dict[str, Any]:
        return self._active_data

    # ── Change callbacks ──────────────────────────────────────────────────
    def on_change(self, callback: Callable[[str, Dict[str, Any]], None]) -> None:
        """Register a callback invoked as ``callback(name, data)`` on profile switch."""
        self._on_change_callbacks.append(callback)

    def _notify(self) -> None:
        for cb in self._on_change_callbacks:
            try:
                cb(self._active_name, self._active_data)
            except Exception:
                pass

    # ── Load / Save ───────────────────────────────────────────────────────
    def load(self, name: Optional[str] = None, create_from_legacy: bool = True) -> Dict[str, Any]:
        """Load a profile by *name* (default: active) and make it active.

        Returns the profile dict.
        """
        name = name or self._active_name
        path = self._find_existing_path(name)
        data = _read_json(path) if path else None

        if data is None and create_from_legacy and name == DEFAULT_PROFILE:
            data = self._build_from_legacy(name)
            if data:
                self.save(name, data)

        if data is None:
            data = {"name": name, "version": 1}

        self._active_name = name
        self._active_data = data
        self._notify()
        return data

    def save(self, name: Optional[str] = None, data: Optional[Dict[str, Any]] = None) -> str:
        """Persist the profile to its JSON file.  Returns the written path."""
        name = name or self._active_name
        data = data if data is not None else self._active_data
        path = self._profile_path(name)
        _write_json(path, data)
        return path

    # ── Query helpers ─────────────────────────────────────────────────────
    def has_endstops(self, name: Optional[str] = None, data: Optional[Dict[str, Any]] = None) -> bool:
        """Check whether the (active or given) profile has hardware endstops."""
        name = name or self._active_name
        data = data if data is not None else self._active_data

        if isinstance(data, dict):
            features = data.get("features")
            if isinstance(features, dict) and "has_endstops" in features:
                try:
                    return bool(features["has_endstops"])
                except Exception:
                    pass

        norm = (name or "").strip().lower()
        if norm in NO_ENDSTOP_PROFILES:
            return False
        return True

    def available_profiles(self) -> List[str]:
        """Return names of all known + discovered profiles."""
        names = set(PROFILE_FILES.keys())
        # Also scan the base_dir for any *.json that look like profiles
        for base in _candidate_base_dirs(self._base_dir):
            for fname in PROFILE_FILES.values():
                if os.path.isfile(os.path.join(base, fname)):
                    # already covered
                    pass
            # Discover extra profile files in profiles/ subdir
            profiles_dir = os.path.join(base, "profiles")
            if os.path.isdir(profiles_dir):
                for entry in os.listdir(profiles_dir):
                    if entry.endswith(".json"):
                        names.add(entry[:-5])
        return sorted(names)

    # ── Section access (sub-key read/write) ───────────────────────────────
    def get_section(self, key: str, default: Any = None) -> Any:
        """Return ``active_data[key]`` or *default*."""
        return self._active_data.get(key, default)

    def set_section(self, key: str, value: Any) -> None:
        """Set ``active_data[key]`` (does NOT auto-save)."""
        self._active_data[key] = value

    # ── Kinematics settings ───────────────────────────────────────────────
    @property
    def kinematics_settings(self) -> Dict[str, Any]:
        """Return the ``kinematics_settings`` sub-dict of the active profile."""
        return self._active_data.get("kinematics_settings", {})

    @kinematics_settings.setter
    def kinematics_settings(self, value: Dict[str, Any]) -> None:
        self._active_data["kinematics_settings"] = value

    # ── Private helpers ───────────────────────────────────────────────────
    def _find_existing_path(self, name: str) -> Optional[str]:
        filename = PROFILE_FILES.get(name, f"{name}.json")
        for base in _candidate_base_dirs(self._base_dir):
            path = os.path.join(base, filename)
            if os.path.isfile(path):
                return path
        return None

    def _profile_path(self, name: str) -> str:
        base = _resolve_write_base_dir(self._base_dir)
        filename = PROFILE_FILES.get(name, f"{name}.json")
        return os.path.join(base, filename)

    def _build_from_legacy(self, name: str) -> Dict[str, Any]:
        base = self._find_base_dir_with_legacy()
        data: Dict[str, Any] = {"name": name, "version": 1}
        for key, rel in LEGACY_FILES.items():
            payload = _read_json(os.path.join(base, rel))
            if payload is not None:
                data[key] = payload
        return data

    def _find_base_dir_with_legacy(self) -> str:
        legacy_paths = list(LEGACY_FILES.values())
        for base in _candidate_base_dirs(self._base_dir):
            for rel in legacy_paths:
                if os.path.isfile(os.path.join(base, rel)):
                    return base
        return _resolve_write_base_dir(self._base_dir)
