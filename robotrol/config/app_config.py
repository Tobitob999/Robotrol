"""
Robotrol v2.0 — Application configuration.

Manages settings.json and project_flags.json.
Extracted from Robotrol_FluidNC_v7_3.py (lines 91-167).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class AppConfig:
    """Centralised holder for runtime paths and persistent settings."""

    base_dir: str = ""
    profiles_dir: str = ""
    configs_dir: str = ""
    model_dir: str = ""
    data_dir: str = ""

    # Loaded data (populated by load())
    settings: Dict[str, Any] = field(default_factory=dict)
    project_flags: Dict[str, Any] = field(default_factory=dict)
    camera_config: Dict[str, Any] = field(default_factory=dict)

    # ── Construction helpers ──────────────────────────────────────────────
    @classmethod
    def from_base_dir(cls, base_dir: str) -> "AppConfig":
        """Create an AppConfig rooted at *base_dir* and immediately load()."""
        cfg = cls(
            base_dir=base_dir,
            profiles_dir=os.path.join(base_dir, "profiles"),
            configs_dir=os.path.join(base_dir, "configs"),
            model_dir=os.path.join(base_dir, "model"),
            data_dir=os.path.join(base_dir, "data"),
        )
        cfg.load()
        return cfg

    # ── Derived paths ─────────────────────────────────────────────────────
    @property
    def settings_path(self) -> str:
        return os.path.join(self.configs_dir, "settings.json")

    @property
    def project_flags_path(self) -> str:
        return os.path.join(self.configs_dir, "project_flags.json")

    @property
    def camera_config_path(self) -> str:
        return os.path.join(self.configs_dir, "camera.json")

    # ── Load / Save ───────────────────────────────────────────────────────
    def load(self) -> None:
        """Read settings.json, project_flags.json and camera.json from disk."""
        self.settings = self._load_settings()
        self.project_flags = self._load_project_flags()
        self.camera_config = self._load_camera_config()

    def save(self) -> None:
        """Persist current settings back to settings.json."""
        self._save_settings(self.settings)

    # ── Private helpers ───────────────────────────────────────────────────
    def _load_settings(self) -> Dict[str, Any]:
        return self._read_json(self.settings_path) or {}

    def _save_settings(self, data: Dict[str, Any]) -> None:
        try:
            existing = self._read_json(self.settings_path) or {}
            existing.update(data)
            os.makedirs(os.path.dirname(self.settings_path), exist_ok=True)
            with open(self.settings_path, "w", encoding="utf-8") as f:
                json.dump(existing, f, indent=2, ensure_ascii=False)
        except Exception as exc:
            print(f"[WARN] Could not save app settings: {exc}")

    def _load_project_flags(self) -> Dict[str, Any]:
        default: Dict[str, Any] = {
            "language": "en",
            "enforce_english_text": True,
            "language_migration_policy": "low_priority_incremental",
        }
        data = self._read_json(self.project_flags_path)
        if isinstance(data, dict):
            out = dict(default)
            out.update(data)
            return out
        return default

    def _load_camera_config(self) -> Dict[str, Any]:
        default: Dict[str, Any] = {
            "device_index": 0,
            "width": 640,
            "height": 480,
            "fps": 30,
        }
        data = self._read_json(self.camera_config_path)
        if isinstance(data, dict):
            out = dict(default)
            out.update(data)
            return out
        return default

    @staticmethod
    def _read_json(path: str) -> Any:
        if not os.path.isfile(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None
