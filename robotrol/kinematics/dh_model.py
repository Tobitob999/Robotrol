"""DH model loading and geometry computation for robotrol v2.

Extracts DH rows, post-transform data, and link geometry (L1..L4)
from profile JSON or standalone dh.json files.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, List, Optional


_DEFAULT_DH_ORDER = ["A", "X", "Y", "B", "Z", "C"]

_DEFAULT_GEOM_DH = {
    "L1": 240.0,
    "L2": 230.0,
    "L3": 250.0,
    "L4": 180.0,
}


class DHModel:
    """Encapsulates a DH robot model: rows, post-transform, geometry."""

    def __init__(self):
        self._model: Optional[Dict[str, Any]] = None
        self._dh_rows: List[Dict[str, Any]] = []
        self._post_transform: Dict[str, Any] = {}
        self._geom: Dict[str, float] = _DEFAULT_GEOM_DH.copy()

    # ---- Class methods ----

    @classmethod
    def from_profile(cls, profile_data: Dict[str, Any]) -> "DHModel":
        """Create a DHModel from a full profile dict (expects 'dh_model' key)."""
        obj = cls()
        model = profile_data.get("dh_model")
        if isinstance(model, dict):
            obj.set_from_dict(model)
        return obj

    @classmethod
    def from_json(cls, path: str) -> "DHModel":
        """Create a DHModel from a standalone dh.json file."""
        obj = cls()
        if not os.path.isfile(path):
            return obj
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                obj.set_from_dict(data)
        except Exception as e:
            print(f"[DHModel] load failed {path}: {e}")
        return obj

    # ---- Public API ----

    def set_from_dict(self, data: Dict[str, Any]) -> None:
        """Load/override the DH model from an in-memory dict."""
        self._model = dict(data) if isinstance(data, dict) else None
        self._dh_rows = _build_dh_rows_mm_deg(self._model) if self._model else []
        self._post_transform = self._model.get("post_transform", {}) if self._model else {}
        self._geom = _geom_from_model(self._model) if self._model else _DEFAULT_GEOM_DH.copy()

    def apply_post_transform(self, joints_dict: Dict[str, float]) -> List[float]:
        """Apply sim_theta_offset_deg + sim_theta_scale to machine angles.

        Takes a dict {'A': deg, 'X': deg, ...} and returns a list of
        transformed angles in DH-row order (one per DH row).
        """
        offsets, scales = _get_joint_angle_post_maps(self._post_transform)
        result = []
        for row in self._dh_rows:
            axis = row["axis"]
            machine_deg = float(joints_dict.get(axis, 0.0))
            scale = float(scales.get(axis, 1.0))
            offset = float(offsets.get(axis, 0.0))
            result.append(machine_deg * scale + offset)
        return result

    @property
    def geom(self) -> Dict[str, float]:
        """The computed DH geometry data (L1, L2, L3, L4 in mm)."""
        return dict(self._geom)

    @property
    def dh_rows(self) -> List[Dict[str, Any]]:
        """The DH rows (alpha_deg, a_mm, d_mm, theta_offset_deg per axis)."""
        return list(self._dh_rows)

    @property
    def post_transform(self) -> Dict[str, Any]:
        """The post-transform dict (mirror_x, sim_theta_offset_deg, sim_theta_scale)."""
        return dict(self._post_transform)

    @property
    def model(self) -> Optional[Dict[str, Any]]:
        """The raw model dict."""
        return self._model


# ---- Internal helpers (ported from tcp_pose_module_v3.py) ----


def _clean_joint_map(raw_map: Any) -> Dict[str, float]:
    out = {}
    if not isinstance(raw_map, dict):
        return out
    for raw_axis, raw_val in raw_map.items():
        axis = str(raw_axis).strip().upper()
        if not axis:
            continue
        try:
            out[axis] = float(raw_val)
        except Exception:
            continue
    return out


def _get_joint_angle_post_maps(post: Dict[str, Any]):
    offsets = _clean_joint_map(post.get("sim_theta_offset_deg", {}))
    scales = _clean_joint_map(post.get("sim_theta_scale", {}))
    return offsets, scales


def _ordered_axes_from_model(model: Any, available_axes=None) -> List[str]:
    if not isinstance(model, dict):
        return list(_DEFAULT_DH_ORDER)

    available = set(available_axes or [])
    ordered = []
    seen = set()

    joint_order = model.get("joint_order", [])
    if isinstance(joint_order, list):
        for axis in joint_order:
            ax = str(axis).strip().upper()
            if not ax or ax in seen:
                continue
            if available and ax not in available:
                continue
            ordered.append(ax)
            seen.add(ax)

    for axis in _DEFAULT_DH_ORDER:
        if axis in seen:
            continue
        if available and axis not in available:
            continue
        ordered.append(axis)
        seen.add(axis)

    if available:
        for axis in sorted(available):
            if axis in seen:
                continue
            ordered.append(axis)
            seen.add(axis)
    return ordered


def _build_dh_rows_mm_deg(model: Any) -> List[Dict[str, Any]]:
    if not isinstance(model, dict):
        return []

    joints_by_axis = {}
    for j in model.get("joints", []):
        axis = (j.get("axis") or "").strip().upper()
        if not axis:
            continue
        joints_by_axis[axis] = j

    order = _ordered_axes_from_model(model, available_axes=joints_by_axis.keys())
    rows = []
    for axis in order:
        j = joints_by_axis.get(axis)
        if not j:
            continue
        rows.append(
            {
                "axis": axis,
                "alpha_deg": math.degrees(float(j.get("alpha", 0.0))),
                "a_mm": float(j.get("a", 0.0)) * 1000.0,
                "d_mm": float(j.get("d", 0.0)) * 1000.0,
                "theta_offset_deg": math.degrees(float(j.get("theta_offset", 0.0))),
            }
        )
    return rows


def _geom_from_model(model: Any) -> Dict[str, float]:
    rows = _build_dh_rows_mm_deg(model)
    by_axis = {r["axis"]: r for r in rows}
    order = _ordered_axes_from_model(model, available_axes=by_axis.keys())

    # L3 = forearm length. Try Y.a_mm first (EB300-style planar link),
    # then fall back to first d_mm>0 after Y (Moveo-style d-offset).
    y_a = abs(float(by_axis.get("Y", {}).get("a_mm", 0.0)))
    if y_a > 1e-9:
        L3 = y_a
    else:
        link2_axis = None
        if "Y" in order:
            y_idx = order.index("Y")
            tail = [ax for ax in order[y_idx + 1:] if ax != "C"]
        else:
            tail = [ax for ax in order if ax not in ("A", "X", "Y", "C")]
        for ax in tail:
            d_val = abs(float(by_axis.get(ax, {}).get("d_mm", 0.0)))
            if d_val > 1e-9:
                link2_axis = ax
                break
        if link2_axis is None:
            link2_axis = "B" if "B" in by_axis else "Z"
        L3 = abs(float(by_axis.get(link2_axis, {}).get("d_mm", _DEFAULT_GEOM_DH["L3"])))

    return {
        "L1": abs(float(by_axis.get("A", {}).get("d_mm", _DEFAULT_GEOM_DH["L1"]))),
        "L2": abs(float(by_axis.get("X", {}).get("a_mm", _DEFAULT_GEOM_DH["L2"]))),
        "L3": L3,
        "L4": abs(float(by_axis.get("C", {}).get("d_mm", _DEFAULT_GEOM_DH["L4"]))),
    }


def fallback_dh_rows_from_geom(geom: Dict[str, float]) -> List[Dict[str, Any]]:
    """Generate standard 6-DOF DH rows from geometry constants (Moveo-style fallback)."""
    L1 = float(geom.get("L1", _DEFAULT_GEOM_DH["L1"]))
    L2 = float(geom.get("L2", _DEFAULT_GEOM_DH["L2"]))
    L3 = float(geom.get("L3", _DEFAULT_GEOM_DH["L3"]))
    L4 = float(geom.get("L4", _DEFAULT_GEOM_DH["L4"]))
    return [
        {"axis": "A", "alpha_deg": 90.0, "a_mm": 0.0, "d_mm": L1, "theta_offset_deg": 0.0},
        {"axis": "X", "alpha_deg": 0.0, "a_mm": L2, "d_mm": 0.0, "theta_offset_deg": 90.0},
        {"axis": "Y", "alpha_deg": -90.0, "a_mm": 0.0, "d_mm": 0.0, "theta_offset_deg": -90.0},
        {"axis": "B", "alpha_deg": 90.0, "a_mm": 0.0, "d_mm": L3, "theta_offset_deg": 0.0},
        {"axis": "Z", "alpha_deg": -90.0, "a_mm": 0.0, "d_mm": 0.0, "theta_offset_deg": 0.0},
        {"axis": "C", "alpha_deg": 0.0, "a_mm": 0.0, "d_mm": L4, "theta_offset_deg": 0.0},
    ]
