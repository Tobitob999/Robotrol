"""Test forward kinematics for EB300 and Moveo profiles."""

import pytest

from robotrol.kinematics.dh_model import DHModel
from robotrol.kinematics.fk import fk6_forward_mm


ZERO_JOINTS = {"A": 0.0, "X": 0.0, "Y": 0.0, "Z": 0.0, "B": 0.0, "C": 0.0}


class TestEB300FK:
    """Forward kinematics for EB300 at MPos=0."""

    def test_tcp_at_zero(self, eb300_profile):
        dh = DHModel.from_profile(eb300_profile)
        joints = dh.apply_post_transform(ZERO_JOINTS)
        x, y, z, *_ = fk6_forward_mm(dh.geom, joints, dh_model=dh)

        assert abs(x - 179.0) < 1.0, f"EB300 X={x}, expected ~179"
        assert abs(y - 0.0) < 1.0, f"EB300 Y={y}, expected ~0"
        assert abs(z - 860.0) < 1.0, f"EB300 Z={z}, expected ~860"


class TestMoveoFK:
    """Forward kinematics for Moveo at MPos=0."""

    def test_tcp_at_zero(self, moveo_profile):
        dh = DHModel.from_profile(moveo_profile)
        joints = dh.apply_post_transform(ZERO_JOINTS)
        x, y, z, *_ = fk6_forward_mm(dh.geom, joints, dh_model=dh)

        assert abs(x - 0.0) < 1.0, f"Moveo X={x}, expected ~0"
        assert abs(y - 0.0) < 1.0, f"Moveo Y={y}, expected ~0"
        assert abs(z - 920.0) < 1.0, f"Moveo Z={z}, expected ~920"
