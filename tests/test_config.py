"""Test config modules: constants, AppConfig, ProfileManager."""

import pytest

from robotrol.config.constants import AXES
from robotrol.config.app_config import AppConfig
from robotrol.config.profiles import ProfileManager


class TestConstants:
    """Tests for robotrol.config.constants."""

    def test_axes_has_six_entries(self):
        assert len(AXES) == 6

    def test_axes_contains_expected(self):
        for ax in ["X", "Y", "Z", "A", "B", "C"]:
            assert ax in AXES


class TestAppConfig:
    """Tests for AppConfig."""

    def test_instantiate_default(self):
        cfg = AppConfig()
        assert cfg.base_dir == ""
        assert isinstance(cfg.settings, dict)

    def test_from_base_dir(self, base_dir):
        cfg = AppConfig.from_base_dir(str(base_dir))
        assert cfg.base_dir == str(base_dir)
        assert cfg.profiles_dir.endswith("profiles")
        assert cfg.configs_dir.endswith("configs")

    def test_load_populates_project_flags(self, base_dir):
        cfg = AppConfig.from_base_dir(str(base_dir))
        assert "language" in cfg.project_flags


class TestProfileManager:
    """Tests for ProfileManager."""

    def test_available_profiles(self, base_dir):
        pm = ProfileManager(str(base_dir))
        profiles = pm.available_profiles()
        assert "EB300" in profiles
        assert "Moveo" in profiles

    def test_available_profiles_has_at_least_three(self, base_dir):
        pm = ProfileManager(str(base_dir))
        profiles = pm.available_profiles()
        assert len(profiles) >= 3

    def test_has_endstops_eb300(self, base_dir, eb300_profile):
        pm = ProfileManager(str(base_dir))
        assert pm.has_endstops(name="EB300", data=eb300_profile) is False

    def test_has_endstops_moveo(self, base_dir, moveo_profile):
        pm = ProfileManager(str(base_dir))
        assert pm.has_endstops(name="Moveo", data=moveo_profile) is True

    def test_load_profile(self, base_dir):
        pm = ProfileManager(str(base_dir))
        data = pm.load("EB300")
        assert data["name"] == "EB300"
        assert pm.active_name == "EB300"
