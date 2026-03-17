"""
FixedTcpTab — Fixed TCP orientation control.

Lock the current TCP orientation (Roll/Pitch/Yaw) and move in XYZ
while keeping it.  Supports world-frame, tool-frame, and point-target
modes, plus gamepad-driven incremental moves.

Extracted from Robotrol_FluidNC_v7_3.py lines 1079-2140, 4466-4519,
5071-5124, 5802-5951.
"""

from __future__ import annotations

import json
import math
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict, Optional, Tuple

from robotrol.config.constants import AXES


# ── RPY / rotation helpers ──────────────────────────────────────────────────

def _rpy_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> list:
    """ZYX Euler angles -> 3x3 rotation matrix (list-of-lists)."""
    yr = math.radians(yaw_deg)
    pr = math.radians(pitch_deg)
    rr = math.radians(roll_deg)
    cy, sy = math.cos(yr), math.sin(yr)
    cp, sp = math.cos(pr), math.sin(pr)
    cr, sr = math.cos(rr), math.sin(rr)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rpy_from_R(R: list) -> Tuple[float, float, float]:
    """3x3 rotation matrix -> (roll, pitch, yaw) in degrees."""
    r11, r21, r31 = R[0][0], R[1][0], R[2][0]
    r32, r33 = R[2][1], R[2][2]
    pitch = math.degrees(math.atan2(-r31, math.hypot(r11, r21)))
    roll = math.degrees(math.atan2(r32, r33))
    yaw = math.degrees(math.atan2(r21, r11))
    return roll, pitch, yaw


def _dot(a: tuple, b: tuple) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: tuple, b: tuple) -> tuple:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: tuple) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _apply_rot(R: list, v: tuple) -> tuple:
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def _apply_rot_T(R: list, v: tuple) -> tuple:
    return (
        R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
        R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
        R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2],
    )


def _wrap_closest(angle: float, ref: float) -> float:
    """Wrap *angle* to be closest to *ref* (avoids +/-180 jumps)."""
    d = (angle - ref + 180.0) % 360.0 - 180.0
    return ref + d


# ── FixedTcpTab ─────────────────────────────────────────────────────────────

class FixedTcpTab(ttk.Frame):
    """Fixed TCP orientation control tab.

    Parameters
    ----------
    parent : tk widget
        The parent container (typically a Notebook tab frame).
    app : ExecuteApp
        Main application reference — provides ``dh``, ``mpos``,
        ``serial``, ``udp``, ``profile_mgr``, ``get_current_tcp_mm()``,
        ``kinematics_tabs``, ``axis_positions``, ``log()``, etc.
    """

    # Gamepad mode labels / colors (shared with app-level cycle)
    GP_MODE_LABELS = ["Normal", "Fixed/Tool", "Fixed/Point"]
    GP_MODE_COLORS = ["#455a64", "#1565c0", "#6a1b9a"]

    def __init__(self, parent: tk.Widget, app: Any) -> None:
        super().__init__(parent)
        self.app = app
        self._fixed_mode_widgets: list = []

        # ── Tk variables ────────────────────────────────────────────
        self.fixed_tcp_enabled = tk.BooleanVar(value=False)
        self.fixed_tcp_roll = tk.DoubleVar(value=0.0)
        self.fixed_tcp_pitch = tk.DoubleVar(value=0.0)
        self.fixed_tcp_yaw = tk.DoubleVar(value=0.0)
        self.fixed_tcp_step = tk.DoubleVar(value=5.0)
        self.fixed_tcp_feed = tk.DoubleVar(value=3000.0)
        self.fixed_tcp_dx = tk.DoubleVar(value=0.0)
        self.fixed_tcp_dy = tk.DoubleVar(value=0.0)
        self.fixed_tcp_dz = tk.DoubleVar(value=0.0)
        self.fixed_tcp_exec_on_release = tk.BooleanVar(value=True)
        self.tcp_gcode_mode = tk.BooleanVar(value=False)
        self.tcp_gcode_abs = True
        self.fixed_tcp_mode = tk.StringVar(value="world")
        self.fixed_tcp_point_dx = tk.DoubleVar(value=0.0)
        self.fixed_tcp_point_dy = tk.DoubleVar(value=0.0)
        self.fixed_tcp_point_dz = tk.DoubleVar(value=100.0)
        self.fixed_tcp_gamepad_mode = tk.BooleanVar(value=False)
        self.fixed_tcp_gp_xy_step = tk.DoubleVar(value=2.0)
        self.fixed_tcp_gp_z_step = tk.DoubleVar(value=1.0)
        self.fixed_tcp_max_dist = tk.DoubleVar(value=300.0)
        self.fixed_tcp_gp_invert_x = tk.BooleanVar(value=False)
        self.fixed_tcp_gp_invert_y = tk.BooleanVar(value=False)
        self.fixed_tcp_gp_invert_z = tk.BooleanVar(value=False)

        # ── Internal state ──────────────────────────────────────────
        self.fixed_tcp_origin_x: float = 0.0
        self.fixed_tcp_origin_y: float = 0.0
        self.fixed_tcp_origin_z: float = 0.0
        self.fixed_tcp_origin_axes: Dict[str, float] = {ax: 0.0 for ax in AXES}
        self.fixed_tcp_sync_pending: bool = False
        self.fixed_tcp_target_delta: Optional[Tuple[float, float, float]] = None
        self.fixed_tcp_last_delta: Optional[Tuple[float, float, float]] = None
        self.fixed_tcp_stable_count: int = 0
        self.fixed_tcp_user_dragging: bool = False
        self.fixed_tcp_point: Optional[Tuple[float, float, float]] = None

        self._vis_path_fixed: list = []
        self._vis_fixed_frame: Any = None
        self._gp_tcp_busy: bool = False
        self._gp_tcp_prev_rpy: Optional[Tuple[float, float, float]] = None
        self._fixed_tcp_guard: bool = False

        self._build_ui()

    # ── Logging helper ──────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        if hasattr(self.app, "log"):
            self.app.log(msg)

    # ── Profile persistence ─────────────────────────────────────────

    def get_fixed_tcp_settings(self) -> Dict[str, Any]:
        """Collect all saveable Fixed TCP settings into a dict."""
        return {
            "enabled": self.fixed_tcp_enabled.get(),
            "roll": self.fixed_tcp_roll.get(),
            "pitch": self.fixed_tcp_pitch.get(),
            "yaw": self.fixed_tcp_yaw.get(),
            "step": self.fixed_tcp_step.get(),
            "feed": self.fixed_tcp_feed.get(),
            "exec_on_release": self.fixed_tcp_exec_on_release.get(),
            "tcp_gcode_mode": self.tcp_gcode_mode.get(),
            "mode": self.fixed_tcp_mode.get(),
            "gamepad_mode": self.fixed_tcp_gamepad_mode.get(),
            "gp_xy_step": self.fixed_tcp_gp_xy_step.get(),
            "gp_z_step": self.fixed_tcp_gp_z_step.get(),
            "max_dist": self.fixed_tcp_max_dist.get(),
            "point_dx": self.fixed_tcp_point_dx.get(),
            "point_dy": self.fixed_tcp_point_dy.get(),
            "point_dz": self.fixed_tcp_point_dz.get(),
            "gp_invert_x": self.fixed_tcp_gp_invert_x.get(),
            "gp_invert_y": self.fixed_tcp_gp_invert_y.get(),
            "gp_invert_z": self.fixed_tcp_gp_invert_z.get(),
        }

    def apply_fixed_tcp_settings(self, data: Any) -> None:
        """Restore Fixed TCP settings from a dict."""
        if not isinstance(data, dict):
            return

        def _s(var: tk.Variable, key: str, conv=float) -> None:
            if key in data:
                try:
                    var.set(conv(data[key]))
                except (ValueError, TypeError):
                    pass

        _s(self.fixed_tcp_enabled, "enabled", bool)
        _s(self.fixed_tcp_roll, "roll")
        _s(self.fixed_tcp_pitch, "pitch")
        _s(self.fixed_tcp_yaw, "yaw")
        _s(self.fixed_tcp_step, "step")
        _s(self.fixed_tcp_feed, "feed")
        _s(self.fixed_tcp_exec_on_release, "exec_on_release", bool)
        _s(self.tcp_gcode_mode, "tcp_gcode_mode", bool)
        if "mode" in data:
            self.fixed_tcp_mode.set(str(data["mode"]))
        _s(self.fixed_tcp_gamepad_mode, "gamepad_mode", bool)
        _s(self.fixed_tcp_gp_xy_step, "gp_xy_step")
        _s(self.fixed_tcp_gp_z_step, "gp_z_step")
        _s(self.fixed_tcp_max_dist, "max_dist")
        _s(self.fixed_tcp_point_dx, "point_dx")
        _s(self.fixed_tcp_point_dy, "point_dy")
        _s(self.fixed_tcp_point_dz, "point_dz")
        _s(self.fixed_tcp_gp_invert_x, "gp_invert_x", bool)
        _s(self.fixed_tcp_gp_invert_y, "gp_invert_y", bool)
        _s(self.fixed_tcp_gp_invert_z, "gp_invert_z", bool)

    # ── Offset helpers ──────────────────────────────────────────────

    def _clamp_offset(self, v: Any) -> float:
        try:
            val = float(v)
        except (ValueError, TypeError):
            val = 0.0
        try:
            lim = max(10.0, float(self.fixed_tcp_max_dist.get()))
        except (ValueError, TypeError):
            lim = 300.0
        return max(-lim, min(lim, val))

    def _set_fixed_offsets(self, dx: float, dy: float, dz: float) -> None:
        self.fixed_tcp_dx.set(self._clamp_offset(dx))
        self.fixed_tcp_dy.set(self._clamp_offset(dy))
        self.fixed_tcp_dz.set(self._clamp_offset(dz))
        try:
            self._update_plane_markers()
        except (AttributeError, tk.TclError):
            pass

    def _get_fixed_origin(self) -> Tuple[float, float, float]:
        return (
            float(self.fixed_tcp_origin_x),
            float(self.fixed_tcp_origin_y),
            float(self.fixed_tcp_origin_z),
        )

    def _request_fixed_delta_sync(
        self, target_delta: Optional[Tuple[float, float, float]] = None
    ) -> None:
        self.fixed_tcp_sync_pending = True
        self.fixed_tcp_target_delta = target_delta
        self.fixed_tcp_last_delta = None
        self.fixed_tcp_stable_count = 0

    # ── RPY / frame helpers ─────────────────────────────────────────

    def _get_fixed_rpy(self, tcp: dict) -> Tuple[float, float, float]:
        if self.fixed_tcp_enabled.get():
            return (
                float(self.fixed_tcp_roll.get()),
                float(self.fixed_tcp_pitch.get()),
                float(self.fixed_tcp_yaw.get()),
            )
        return (
            float(tcp.get("Roll_deg", 0.0)),
            float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0))),
            float(tcp.get("Yaw_deg", 0.0)),
        )

    def _offset_to_world(
        self, dx: float, dy: float, dz: float, tcp: dict
    ) -> Tuple[float, float, float]:
        mode = (self.fixed_tcp_mode.get() or "world").lower()
        if mode == "tool":
            roll, pitch, yaw = self._get_fixed_rpy(tcp)
            R = _rpy_to_R(roll, pitch, yaw)
            dx, dy, dz = _apply_rot(R, (dx, dy, dz))
        dz = -dz
        return dx, dy, dz

    def world_to_offset(
        self, dxw: float, dyw: float, dzw: float, tcp: dict
    ) -> Tuple[float, float, float]:
        """Convert world-frame delta to offset-frame (public for external use)."""
        mode = (self.fixed_tcp_mode.get() or "world").lower()
        if mode == "tool":
            roll, pitch, yaw = self._get_fixed_rpy(tcp)
            R = _rpy_to_R(roll, pitch, yaw)
            dxw, dyw, dzw = _apply_rot_T(R, (dxw, dyw, dzw))
        dzw = -dzw
        return dxw, dyw, dzw

    # ── Target computation ──────────────────────────────────────────

    def calc_fixed_target(self) -> Tuple[float, float, float, float, float, float, float, float, float]:
        """Compute (x, y, z, roll, pitch, yaw, dx, dy, dz) from current offsets."""
        tcp = self.app.get_current_tcp_mm()
        dx = float(self.fixed_tcp_dx.get())
        dy = float(self.fixed_tcp_dy.get())
        dz = float(self.fixed_tcp_dz.get())
        dxw, dyw, dzw = self._offset_to_world(dx, dy, dz, tcp)
        ox, oy, oz = self._get_fixed_origin()
        x = ox + dxw
        y = oy + dyw
        z = oz + dzw
        roll, pitch, yaw = self._get_fixed_rpy(tcp)

        mode = (self.fixed_tcp_mode.get() or "world").lower()
        if mode == "point":
            if self.fixed_tcp_point is None:
                self._set_tcp_point_from_current()
            point = self.fixed_tcp_point
            if point is not None:
                vx = float(point[0]) - x
                vy = float(point[1]) - y
                vz = float(point[2]) - z
                dist = math.sqrt(vx * vx + vy * vy + vz * vz)
                if dist > 1e-6:
                    ux, uy, uz = vx / dist, vy / dist, vz / dist
                    z_axis = (ux, uy, uz)
                    ref_roll, ref_pitch, ref_yaw = self._get_fixed_rpy(tcp)
                    R_ref = _rpy_to_R(ref_roll, ref_pitch, ref_yaw)
                    x_ref = (R_ref[0][0], R_ref[1][0], R_ref[2][0])
                    dot_xz = _dot(x_ref, z_axis)
                    x_proj = (
                        x_ref[0] - dot_xz * z_axis[0],
                        x_ref[1] - dot_xz * z_axis[1],
                        x_ref[2] - dot_xz * z_axis[2],
                    )
                    xn = _norm(x_proj)
                    if xn < 1e-6:
                        up = (0.0, 0.0, 1.0)
                        if abs(z_axis[2]) > 0.95:
                            up = (0.0, 1.0, 0.0)
                        x_proj = _cross(up, z_axis)
                        xn = _norm(x_proj)
                    if xn > 1e-6:
                        x_ax = (x_proj[0] / xn, x_proj[1] / xn, x_proj[2] / xn)
                        y_ax = _cross(z_axis, x_ax)
                        yn = _norm(y_ax)
                        if yn > 1e-6:
                            y_ax = (y_ax[0] / yn, y_ax[1] / yn, y_ax[2] / yn)
                            x_ax = _cross(y_ax, z_axis)
                            R_point = [
                                [x_ax[0], y_ax[0], z_axis[0]],
                                [x_ax[1], y_ax[1], z_axis[1]],
                                [x_ax[2], y_ax[2], z_axis[2]],
                            ]
                            roll, pitch, yaw = _rpy_from_R(R_point)

        return x, y, z, roll, pitch, yaw, dx, dy, dz

    # ── Apply / execute target ──────────────────────────────────────

    def apply_fixed_target(self, execute: bool = False) -> bool:
        """Compute target and optionally send the IK move."""
        if not self.fixed_tcp_enabled.get():
            return False
        x, y, z, roll, pitch, yaw, dx, dy, dz = self.calc_fixed_target()
        if execute and hasattr(self.app, "kinematics_tabs"):
            self._request_fixed_delta_sync(target_delta=(dx, dy, dz))
            prev_skip = getattr(self.app, "_vis_skip_kin", False)
            self.app._vis_skip_kin = True
            try:
                ok = self.app.kinematics_tabs.move_tcp_pose(
                    x, y, z, roll, pitch, yaw,
                    feed=float(self.fixed_tcp_feed.get()),
                    allow_out_of_limits=False,
                )
            except (RuntimeError, ValueError, ZeroDivisionError) as e:
                self.app._vis_skip_kin = prev_skip
                self._log(f"Fixed TCP move error: {e}")
                return False
            self.app._vis_skip_kin = prev_skip
            if ok:
                self._append_vis_path_fixed((x, y, z))
                return True
            # Fallback: step toward target with small increments
            try:
                step = max(0.5, float(self.fixed_tcp_step.get()))
            except (ValueError, TypeError):
                step = 5.0
            cur = self.app.get_current_tcp_mm()
            cx = float(cur.get("X_mm", 0.0))
            cy_val = float(cur.get("Y_mm", 0.0))
            cz = float(cur.get("Z_mm", 0.0))
            ddx = x - cx
            ddy = y - cy_val
            ddz = z - cz
            dist = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
            if dist < 1e-6:
                return False
            ux, uy, uz = ddx / dist, ddy / dist, ddz / dist
            steps = int(dist / step) + 1
            for _ in range(min(steps, 50)):
                cx += ux * step
                cy_val += uy * step
                cz += uz * step
                prev_skip2 = getattr(self.app, "_vis_skip_kin", False)
                self.app._vis_skip_kin = True
                ok = self.app.kinematics_tabs.move_tcp_pose(
                    cx, cy_val, cz, roll, pitch, yaw,
                    feed=float(self.fixed_tcp_feed.get()),
                    allow_out_of_limits=False,
                )
                self.app._vis_skip_kin = prev_skip2
                if not ok:
                    return False
                self._append_vis_path_fixed((cx, cy_val, cz))
            return True
        return False

    # ── Visualiser path ─────────────────────────────────────────────

    def _append_vis_path_fixed(self, pt: tuple) -> None:
        try:
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
        except (ValueError, TypeError, IndexError):
            return
        try:
            a = float(self.app.axis_positions.get("A", 0.0))
        except (ValueError, TypeError, AttributeError):
            a = 0.0
        self._vis_path_fixed.append((x, y, z, a))
        if len(self._vis_path_fixed) > 400:
            self._vis_path_fixed = self._vis_path_fixed[-400:]
        self._send_vis_path("path_fixed", self._vis_path_fixed)

    def _send_vis_path(self, path_type: str, pts: list) -> None:
        udp_sock = getattr(getattr(self.app, "client", None), "udp_sock", None)
        if not udp_sock:
            return
        udp_addr = getattr(self.app, "_udp_addr", ("127.0.0.1", 9999))
        try:
            msg = json.dumps({
                "type": path_type,
                "pts": [[float(a), float(b), float(c), float(d)]
                        for a, b, c, d in pts],
            })
            udp_sock.sendto(msg.encode("utf-8"), udp_addr)
        except OSError:
            pass

    def _send_vis_fixed_frame(
        self,
        origin: tuple,
        x_axis: tuple,
        y_axis: tuple,
        z_axis: tuple,
    ) -> None:
        udp_sock = getattr(getattr(self.app, "client", None), "udp_sock", None)
        if not udp_sock:
            return
        udp_addr = getattr(self.app, "_udp_addr", ("127.0.0.1", 9999))
        try:
            msg = json.dumps({
                "type": "fixed_frame",
                "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
                "x": [float(x_axis[0]), float(x_axis[1]), float(x_axis[2])],
                "y": [float(y_axis[0]), float(y_axis[1]), float(y_axis[2])],
                "z": [float(z_axis[0]), float(z_axis[1]), float(z_axis[2])],
            })
            udp_sock.sendto(msg.encode("utf-8"), udp_addr)
        except OSError:
            pass

    # ── TCP point target ────────────────────────────────────────────

    def _set_tcp_point_from_current(self) -> None:
        tcp = self.app.get_current_tcp_mm()
        try:
            roll = float(tcp.get("Roll_deg", 0.0))
            pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
            yaw = float(tcp.get("Yaw_deg", 0.0))
            dxp = float(self.fixed_tcp_point_dx.get())
            dyp = float(self.fixed_tcp_point_dy.get())
            dzp = float(self.fixed_tcp_point_dz.get())
        except (ValueError, TypeError):
            return

        R = _rpy_to_R(roll, pitch, yaw)
        x_axis = (R[0][0], R[1][0], R[2][0])
        y_axis = (R[0][1], R[1][1], R[2][1])
        z_axis = (R[0][2], R[1][2], R[2][2])

        dxw = x_axis[0] * dxp + y_axis[0] * dyp + z_axis[0] * dzp
        dyw = x_axis[1] * dxp + y_axis[1] * dyp + z_axis[1] * dzp
        dzw = x_axis[2] * dxp + y_axis[2] * dyp + z_axis[2] * dzp

        try:
            px = float(tcp.get("X_mm", 0.0)) + dxw
            py = float(tcp.get("Y_mm", 0.0)) + dyw
            pz = float(tcp.get("Z_mm", 0.0)) + dzw
        except (ValueError, TypeError):
            return
        self.fixed_tcp_point = (px, py, pz)
        self._log(f"TCP Point set to ({px:.2f}, {py:.2f}, {pz:.2f}).")

    # ── Capture current TCP as fixed origin ─────────────────────────

    def update_fixed_from_tcp(self) -> None:
        """Snapshot current TCP pose as the fixed origin (same as GUI button)."""
        tcp = self.app.get_current_tcp_mm()
        self.fixed_tcp_origin_x = float(tcp.get("X_mm", 0.0))
        self.fixed_tcp_origin_y = float(tcp.get("Y_mm", 0.0))
        self.fixed_tcp_origin_z = float(tcp.get("Z_mm", 0.0))
        try:
            self.fixed_tcp_origin_axes = {
                ax: float(self.app.axis_positions.get(ax, 0.0)) for ax in AXES
            }
        except (ValueError, TypeError, AttributeError):
            pass
        self.fixed_tcp_roll.set(float(tcp.get("Roll_deg", 0.0)))
        self.fixed_tcp_pitch.set(
            float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
        )
        self.fixed_tcp_yaw.set(float(tcp.get("Yaw_deg", 0.0)))
        self._set_fixed_offsets(0.0, 0.0, 0.0)
        self.fixed_tcp_sync_pending = False
        self.fixed_tcp_target_delta = None
        self.fixed_tcp_last_delta = None
        self.fixed_tcp_stable_count = 0
        self.fixed_tcp_point = None

        # Send fixed frame to visualiser
        try:
            roll = float(self.fixed_tcp_roll.get())
            pitch = float(self.fixed_tcp_pitch.get())
            yaw = float(self.fixed_tcp_yaw.get())
            R = _rpy_to_R(roll, pitch, yaw)
            z_axis = (R[0][2], R[1][2], R[2][2])
            zn = _norm(z_axis)
            if zn > 1e-9:
                z_axis = (z_axis[0] / zn, z_axis[1] / zn, z_axis[2] / zn)
                up = (0.0, 0.0, 1.0)
                if abs(z_axis[2]) > 0.95:
                    up = (0.0, 1.0, 0.0)
                x_axis = _cross(up, z_axis)
                xn = _norm(x_axis)
                if xn > 1e-9:
                    x_axis = (x_axis[0] / xn, x_axis[1] / xn, x_axis[2] / xn)
                    y_axis = (
                        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
                        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
                        z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
                    )
                    self._send_vis_fixed_frame(
                        (self.fixed_tcp_origin_x, self.fixed_tcp_origin_y,
                         self.fixed_tcp_origin_z),
                        x_axis, y_axis, z_axis,
                    )
        except (ValueError, TypeError):
            pass

        self._log("Fixed TCP orientation captured from current pose.")
        self._update_fixed_gcode_preview()

    # ── Gamepad methods ─────────────────────────────────────────────

    def move_fixed_tcp_gamepad(
        self, dx: float, dy: float, dz: float, feed: float = 3000
    ) -> None:
        """Accumulate incremental XYZ offset and execute a fixed-TCP IK move.

        Called on the main thread via ``widget.after(0, ...)`` from the
        gamepad thread.  Drops stale commands when a previous IK solve is
        still running to prevent command queue buildup / jerkiness.
        """
        if not self.fixed_tcp_enabled.get():
            return
        if not self.fixed_tcp_gamepad_mode.get():
            return
        if self._gp_tcp_busy:
            return
        self._gp_tcp_busy = True
        try:
            # Per-axis inversion
            if self.fixed_tcp_gp_invert_x.get():
                dx = -dx
            if self.fixed_tcp_gp_invert_y.get():
                dy = -dy
            if self.fixed_tcp_gp_invert_z.get():
                dz = -dz

            try:
                lim = max(10.0, float(self.fixed_tcp_max_dist.get()))
            except (ValueError, TypeError):
                lim = 300.0

            def _c(v: float) -> float:
                return max(-lim, min(lim, float(v)))

            self.fixed_tcp_dx.set(_c(self.fixed_tcp_dx.get() + dx))
            self.fixed_tcp_dy.set(_c(self.fixed_tcp_dy.get() + dy))
            self.fixed_tcp_dz.set(_c(self.fixed_tcp_dz.get() + dz))

            if hasattr(self.app, "kinematics_tabs"):
                x, y, z, roll, pitch, yaw, _, _, _ = self.calc_fixed_target()
                # Normalize RPY to be closest to previous values
                prev_rpy = self._gp_tcp_prev_rpy
                if prev_rpy is not None:
                    roll = _wrap_closest(roll, prev_rpy[0])
                    pitch = _wrap_closest(pitch, prev_rpy[1])
                    yaw = _wrap_closest(yaw, prev_rpy[2])
                self._gp_tcp_prev_rpy = (roll, pitch, yaw)

                prev = getattr(self.app, "_vis_skip_kin", False)
                self.app._vis_skip_kin = True
                try:
                    self.app.kinematics_tabs.move_tcp_pose(
                        x, y, z, roll, pitch, yaw,
                        feed=int(feed),
                        allow_out_of_limits=False,
                        max_iters=15,
                    )
                finally:
                    self.app._vis_skip_kin = prev
        except (RuntimeError, ValueError, ZeroDivisionError, AttributeError) as e:
            try:
                self._log(f"[Gamepad TCP] {e}")
            except (AttributeError, tk.TclError):
                pass
        finally:
            self._gp_tcp_busy = False

    def rotate_fixed_tcp_roll(self, delta_deg: float) -> None:
        """Increment fixed TCP roll angle and re-apply (tool rotation)."""
        if not self.fixed_tcp_enabled.get():
            return
        try:
            cur_roll = float(self.fixed_tcp_roll.get())
            self.fixed_tcp_roll.set(cur_roll + delta_deg)
            self.apply_fixed_target(execute=True)
        except (ValueError, TypeError, RuntimeError) as e:
            try:
                self._log(f"[Gamepad Roll] {e}")
            except (AttributeError, tk.TclError):
                pass

    def set_gamepad_mode_cycle(self, mode: int) -> None:
        """Cycle to the given gamepad mode (0=Normal, 1=Fixed/Tool, 2=Fixed/Point)."""
        mode = int(mode) % 3
        label = self.GP_MODE_LABELS[mode]

        self._gp_tcp_prev_rpy = None
        if mode == 0:
            self.fixed_tcp_gamepad_mode.set(False)
            self.fixed_tcp_enabled.set(False)
        elif mode == 1:
            self.fixed_tcp_enabled.set(True)
            self.fixed_tcp_mode.set("tool")
            self.fixed_tcp_gamepad_mode.set(True)
        elif mode == 2:
            self.fixed_tcp_enabled.set(True)
            self.fixed_tcp_mode.set("point")
            self.fixed_tcp_gamepad_mode.set(True)

        self._log(f"[Gamepad] Mode \u2192 {label}")

    def fix_tcp_from_gamepad(self) -> None:
        """LT: capture current TCP as fixed origin (same as GUI button)
        and optionally set Point target in point mode."""
        self.update_fixed_from_tcp()
        self._gp_tcp_prev_rpy = None
        # In Point mode: additionally set the TCP point target
        mode_str = self.fixed_tcp_mode.get()
        if mode_str == "point":
            self._set_tcp_point_from_current()

        self._log("[Gamepad] Fix from current TCP")

    # ── G-code preview helpers ──────────────────────────────────────

    def _set_fixed_gcode_status(
        self, ok: Optional[bool] = None, text: Optional[str] = None
    ) -> None:
        if not hasattr(self, "_gcode_status_lbl"):
            return
        if ok is None:
            color, msg = "#777777", text or "Endstops: --"
        elif ok:
            color, msg = "#2e7d32", text or "Endstops: OK"
        else:
            color, msg = "#b71c1c", text or "Endstops: LIMIT"
        self._gcode_status_var.set(msg)
        self._gcode_status_lbl.configure(bg=color, fg="white")

    def _emit_fixed_gcode(
        self,
        gcode: str,
        lims: Optional[dict] = None,
        joints: Optional[dict] = None,
    ) -> None:
        if not hasattr(self, "_gcode_txt"):
            return
        txt = self._gcode_txt
        txt.configure(state="normal")
        txt.delete("1.0", tk.END)
        if not gcode:
            txt.insert(tk.END, "--", "plain")
        else:
            axes_order = ("A", "X", "Y", "B", "Z", "C")
            parts = gcode.split(" ")
            for part in parts:
                tag = "plain"
                for ax in axes_order:
                    if part.startswith(ax):
                        try:
                            val = float(part[1:])
                        except (ValueError, TypeError):
                            val = joints.get(ax) if joints else None
                        if val is not None and lims:
                            lo, hi = lims.get(ax, (-999, 999))
                            tag = "ok" if lo <= float(val) <= hi else "bad_val"
                        break
                txt.insert(tk.END, part + " ", tag)
        txt.configure(state="disabled")

    def _update_fixed_gcode_preview(self) -> None:
        if not hasattr(self, "_gcode_txt"):
            return
        if not self.fixed_tcp_enabled.get():
            self._emit_fixed_gcode("Fixed mode off")
            self._set_fixed_gcode_status(None, "Endstops: --")
            return
        if not hasattr(self.app, "kinematics_tabs"):
            self._emit_fixed_gcode("No kinematics")
            self._set_fixed_gcode_status(None, "Endstops: --")
            return
        try:
            x, y, z, roll, pitch, yaw, _dx, _dy, _dz = self.calc_fixed_target()
            feed = float(self.fixed_tcp_feed.get())
        except (ValueError, TypeError, AttributeError):
            self._emit_fixed_gcode("Preview error")
            self._set_fixed_gcode_status(False, "Endstops: n/a")
            return

        res = None
        try:
            res = self.app.kinematics_tabs.preview_tcp_gcode(
                x, y, z, roll, pitch, yaw, feed=feed
            )
        except (RuntimeError, ValueError, AttributeError):
            res = None

        if not res:
            self._emit_fixed_gcode("Preview error")
            self._set_fixed_gcode_status(False, "Endstops: n/a")
            return

        gcode = res.get("gcode", "")
        lims = res.get("limits", {}) or {}
        joints_data = res.get("joints", {}) or {}
        ok = bool(res.get("ok", False))
        self._emit_fixed_gcode(gcode, lims, joints_data)
        self._set_fixed_gcode_status(ok)

    # ── UI builders ─────────────────────────────────────────────────

    def _build_ui(self) -> None:  # noqa: C901 — large but flat UI-build method
        widgets = self._fixed_mode_widgets

        fixed_wrap = ttk.LabelFrame(self, text="Fixed TCP Orientation")
        fixed_wrap.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        info = ttk.Label(
            fixed_wrap,
            text="Fix current TCP orientation and move in XYZ while keeping it.",
        )
        info.pack(anchor="w", padx=4, pady=(2, 4))

        # ── Top button row ──────────────────────────────────────────
        row_fix = ttk.Frame(fixed_wrap)
        row_fix.pack(fill=tk.X, padx=4, pady=2)

        ttk.Button(
            row_fix, text="Fix from current TCP",
            command=self.update_fixed_from_tcp,
        ).pack(side=tk.LEFT)

        ttk.Button(
            row_fix, text="Set XYZ to zero",
            command=lambda: self._set_fixed_offsets(0.0, 0.0, 0.0),
        ).pack(side=tk.LEFT, padx=(6, 0))

        ttk.Button(
            row_fix, text="Home",
            command=self._home_fixed_xyz,
        ).pack(side=tk.LEFT, padx=(6, 0))

        ttk.Button(
            row_fix, text="Help",
            command=self._show_help,
        ).pack(side=tk.LEFT, padx=(6, 0))

        # ── Options row ─────────────────────────────────────────────
        row_opts = ttk.Frame(fixed_wrap)
        row_opts.pack(fill=tk.X, padx=4, pady=(2, 2))
        row_opts.columnconfigure(0, weight=1)
        row_opts.columnconfigure(1, weight=1)
        row_opts.columnconfigure(2, weight=1)

        chk_fixed = ttk.Checkbutton(
            row_opts, text="Fixed mode", variable=self.fixed_tcp_enabled,
        )
        chk_fixed.grid(row=0, column=0, sticky="w")

        chk_exec = ttk.Checkbutton(
            row_opts, text="Execute on release",
            variable=self.fixed_tcp_exec_on_release,
        )
        chk_exec.grid(row=0, column=1, sticky="w")
        widgets.append(chk_exec)

        chk_gcode = ttk.Checkbutton(
            row_opts, text="G-code TCP mode (XY(Z) uses fixed orientation)",
            variable=self.tcp_gcode_mode,
        )
        chk_gcode.grid(row=0, column=2, sticky="w")

        chk_gamepad = ttk.Checkbutton(
            row_opts, text="Gamepad XYZ",
            variable=self.fixed_tcp_gamepad_mode,
        )
        chk_gamepad.grid(row=1, column=0, sticky="w")
        widgets.append(chk_gamepad)

        # Gamepad sensitivity row
        gp_sens = ttk.Frame(row_opts)
        gp_sens.grid(row=1, column=1, columnspan=2, sticky="w")
        ttk.Label(gp_sens, text="L\u2192XY:", foreground="gray").pack(
            side=tk.LEFT, padx=(0, 2))
        ent_gp_xy = ttk.Entry(
            gp_sens, textvariable=self.fixed_tcp_gp_xy_step,
            width=5, justify="right",
        )
        ent_gp_xy.pack(side=tk.LEFT)
        ttk.Label(gp_sens, text="mm  R\u2192Z:", foreground="gray").pack(
            side=tk.LEFT, padx=(4, 2))
        ent_gp_z = ttk.Entry(
            gp_sens, textvariable=self.fixed_tcp_gp_z_step,
            width=5, justify="right",
        )
        ent_gp_z.pack(side=tk.LEFT)
        ttk.Label(gp_sens, text="mm", foreground="gray").pack(
            side=tk.LEFT, padx=(2, 6))
        ttk.Label(gp_sens, text="Max\u00b1:", foreground="gray").pack(
            side=tk.LEFT, padx=(4, 2))
        ent_max_dist = ttk.Entry(
            gp_sens, textvariable=self.fixed_tcp_max_dist,
            width=5, justify="right",
        )
        ent_max_dist.pack(side=tk.LEFT)
        ttk.Label(gp_sens, text="mm", foreground="gray").pack(
            side=tk.LEFT, padx=(2, 6))
        widgets.extend([ent_gp_xy, ent_gp_z, ent_max_dist])

        btn_gp_test = ttk.Button(
            gp_sens, text="Test \u25b6",
            command=self._gp_test_go,
        )
        btn_gp_test.pack(side=tk.LEFT)
        widgets.append(btn_gp_test)

        # Invert gamepad axes
        row_gp_inv = ttk.Frame(fixed_wrap)
        row_gp_inv.pack(fill=tk.X, padx=4, pady=(0, 2))
        ttk.Label(row_gp_inv, text="Invert Gamepad:", foreground="gray").pack(
            side=tk.LEFT, padx=(0, 4))
        chk_inv_x = ttk.Checkbutton(
            row_gp_inv, text="X", variable=self.fixed_tcp_gp_invert_x)
        chk_inv_x.pack(side=tk.LEFT, padx=(0, 6))
        chk_inv_y = ttk.Checkbutton(
            row_gp_inv, text="Y", variable=self.fixed_tcp_gp_invert_y)
        chk_inv_y.pack(side=tk.LEFT, padx=(0, 6))
        chk_inv_z = ttk.Checkbutton(
            row_gp_inv, text="Z", variable=self.fixed_tcp_gp_invert_z)
        chk_inv_z.pack(side=tk.LEFT, padx=(0, 6))
        widgets.extend([chk_inv_x, chk_inv_y, chk_inv_z])

        # ── Mode selection ──────────────────────────────────────────
        row_mode = ttk.Frame(fixed_wrap)
        row_mode.pack(fill=tk.X, padx=4, pady=(2, 2))
        ttk.Label(row_mode, text="Mode:").pack(side=tk.LEFT, padx=(0, 6))
        rb_world = ttk.Radiobutton(
            row_mode, text="World", variable=self.fixed_tcp_mode, value="world",
        )
        rb_world.pack(side=tk.LEFT, padx=(0, 10))
        widgets.append(rb_world)
        rb_tool = ttk.Radiobutton(
            row_mode, text="Tool frame", variable=self.fixed_tcp_mode, value="tool",
        )
        rb_tool.pack(side=tk.LEFT, padx=(0, 10))
        widgets.append(rb_tool)
        rb_point = ttk.Radiobutton(
            row_mode, text="TCP Point", variable=self.fixed_tcp_mode, value="point",
        )
        rb_point.pack(side=tk.LEFT)
        widgets.append(rb_point)

        self._gp_point_lbl = ttk.Label(row_mode, text="", foreground="#1565c0")
        self._gp_point_lbl.pack(side=tk.LEFT, padx=(12, 0))

        # Point offset row
        row_point = ttk.Frame(fixed_wrap)
        row_point.pack(fill=tk.X, padx=4, pady=(2, 2))
        ttk.Label(row_point, text="Point dX").pack(side=tk.LEFT, padx=(0, 2))
        ent_pt_dx = ttk.Entry(
            row_point, textvariable=self.fixed_tcp_point_dx, width=7, justify="right",
        )
        ent_pt_dx.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_point, text="dY").pack(side=tk.LEFT, padx=(0, 2))
        ent_pt_dy = ttk.Entry(
            row_point, textvariable=self.fixed_tcp_point_dy, width=7, justify="right",
        )
        ent_pt_dy.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_point, text="dZ").pack(side=tk.LEFT, padx=(0, 2))
        ent_pt_dz = ttk.Entry(
            row_point, textvariable=self.fixed_tcp_point_dz, width=7, justify="right",
        )
        ent_pt_dz.pack(side=tk.LEFT, padx=(0, 6))
        widgets.extend([ent_pt_dx, ent_pt_dy, ent_pt_dz])

        btn_set_point = ttk.Button(
            row_point, text="Set TCP Point",
            command=lambda: (
                self._set_tcp_point_from_current(),
                self._update_fixed_gcode_preview(),
            ),
        )
        btn_set_point.pack(side=tk.LEFT, padx=(4, 0))
        widgets.append(btn_set_point)

        # ── Roll / Pitch / Yaw ──────────────────────────────────────
        row_vals = ttk.Frame(fixed_wrap)
        row_vals.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(row_vals, text="Roll").pack(side=tk.LEFT, padx=(0, 2))
        ent_roll = ttk.Entry(
            row_vals, textvariable=self.fixed_tcp_roll, width=6, justify="right",
        )
        ent_roll.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_vals, text="Pitch").pack(side=tk.LEFT, padx=(0, 2))
        ent_pitch = ttk.Entry(
            row_vals, textvariable=self.fixed_tcp_pitch, width=6, justify="right",
        )
        ent_pitch.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_vals, text="Yaw").pack(side=tk.LEFT, padx=(0, 2))
        ent_yaw = ttk.Entry(
            row_vals, textvariable=self.fixed_tcp_yaw, width=6, justify="right",
        )
        ent_yaw.pack(side=tk.LEFT, padx=(0, 6))
        widgets.extend([ent_roll, ent_pitch, ent_yaw])

        # ── Step / Feed ─────────────────────────────────────────────
        row_ctrl = ttk.Frame(fixed_wrap)
        row_ctrl.pack(fill=tk.X, padx=4, pady=(2, 2))
        ttk.Label(row_ctrl, text="Step [mm]").pack(side=tk.LEFT, padx=(0, 4))
        ent_step = ttk.Entry(
            row_ctrl, textvariable=self.fixed_tcp_step, width=6, justify="right",
        )
        ent_step.pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(row_ctrl, text="Feed [mm/min]").pack(side=tk.LEFT, padx=(0, 4))
        ent_feed = ttk.Entry(
            row_ctrl, textvariable=self.fixed_tcp_feed, width=8, justify="right",
        )
        ent_feed.pack(side=tk.LEFT)
        widgets.extend([ent_step, ent_feed])

        # ── Jog frame ──────────────────────────────────────────────
        jog = ttk.LabelFrame(fixed_wrap, text="Move XYZ (relative)")
        jog.pack(fill=tk.BOTH, padx=4, pady=(4, 4))

        # Jog buttons
        btn_row = ttk.Frame(jog)
        btn_row.pack(pady=(2, 2))

        def _step_val() -> float:
            try:
                return float(self.fixed_tcp_step.get())
            except (ValueError, TypeError):
                return 0.0

        ttk.Button(
            btn_row, text="X -", width=6,
            command=lambda: self._move_rel(-_step_val(), 0.0, 0.0),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            btn_row, text="X +", width=6,
            command=lambda: self._move_rel(_step_val(), 0.0, 0.0),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            btn_row, text="Y -", width=6,
            command=lambda: self._move_rel(0.0, -_step_val(), 0.0),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            btn_row, text="Y +", width=6,
            command=lambda: self._move_rel(0.0, _step_val(), 0.0),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            btn_row, text="Z -", width=6,
            command=lambda: self._move_rel(0.0, 0.0, -_step_val()),
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            btn_row, text="Z +", width=6,
            command=lambda: self._move_rel(0.0, 0.0, _step_val()),
        ).pack(side=tk.LEFT, padx=4)

        # Sliders + side buttons
        sliders = ttk.Frame(jog)
        sliders.pack(fill=tk.BOTH, expand=True, pady=(2, 0))
        sliders_left = ttk.Frame(sliders)
        sliders_left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sliders_right = ttk.Frame(sliders)
        sliders_right.pack(side=tk.LEFT, padx=(6, 0), fill=tk.Y)

        scale_x, ent_x = self._make_slider_row(sliders_left, "X [mm]", self.fixed_tcp_dx)
        scale_y, ent_y = self._make_slider_row(sliders_left, "Y [mm]", self.fixed_tcp_dy)
        scale_z, ent_z = self._make_slider_row(sliders_left, "Z [mm]", self.fixed_tcp_dz)
        widgets.extend([scale_x, ent_x, scale_y, ent_y, scale_z, ent_z])

        btn_set = ttk.Button(
            sliders_right, text="Set", width=10,
            command=lambda: self.apply_fixed_target(execute=True),
        )
        btn_set.pack(fill=tk.X, pady=(0, 4))

        btn_set_all = ttk.Button(
            sliders_right, text="Set All", width=10,
            command=self._set_all_offsets,
        )
        btn_set_all.pack(fill=tk.X, pady=(0, 4))

        btn_queue = ttk.Button(
            sliders_right, text="Send to Queue", width=10,
            command=self._enqueue_fixed_target,
        )
        btn_queue.pack(fill=tk.X)
        widgets.extend([btn_set, btn_set_all, btn_queue])

        # ── XY / XZ / YZ plane canvases ─────────────────────────────
        self._canvas_size = 158
        self._radius_mm = 200.0
        self._radius_px = 75.0
        self._center = self._canvas_size // 2

        planes_wrap = ttk.Frame(jog)
        planes_wrap.pack(pady=(2, 0), fill=tk.X)

        planes = ttk.Frame(planes_wrap)
        planes.pack(side=tk.LEFT)

        self._xy_canvas, self._xy_dot = self._make_plane(planes, "XY")
        self._xz_canvas, self._xz_dot = self._make_plane(planes, "XZ")
        self._yz_canvas, self._yz_dot = self._make_plane(planes, "YZ")
        widgets.extend([self._xy_canvas, self._xz_canvas, self._yz_canvas])

        self._bind_plane(self._xy_canvas, "X", "Y")
        self._bind_plane(self._xz_canvas, "X", "Z")
        self._bind_plane(self._yz_canvas, "Y", "Z")

        # G-code preview
        gcode_wrap = ttk.LabelFrame(planes_wrap, text="Target G-code")
        gcode_wrap.pack(side=tk.LEFT, padx=(6, 0), fill=tk.Y)
        self._gcode_status_var = tk.StringVar(value="Endstops: --")
        self._gcode_status_lbl = tk.Label(
            gcode_wrap,
            textvariable=self._gcode_status_var,
            bg="#777777", fg="white", anchor="center",
        )
        self._gcode_status_lbl.pack(fill=tk.X, padx=4, pady=(4, 2))

        self._gcode_txt = tk.Text(
            gcode_wrap, height=5, width=45, wrap="none",
            font=("Consolas", 9),
        )
        self._gcode_txt.pack(padx=4, pady=(0, 4))
        self._gcode_txt.tag_configure("ok", background="#e8ffe8")
        self._gcode_txt.tag_configure("bad_val", background="#ffd0d0")
        self._gcode_txt.tag_configure("plain", background="white")
        self._gcode_txt.configure(state="disabled")

        # ── Variable traces ─────────────────────────────────────────
        self.fixed_tcp_enabled.trace_add("write", self._sync_modes)
        self.tcp_gcode_mode.trace_add("write", self._sync_modes)
        self._sync_modes()

        self.fixed_tcp_mode.trace_add("write", self._update_gp_point_notice)
        self.fixed_tcp_gamepad_mode.trace_add("write", self._update_gp_point_notice)

        self.fixed_tcp_enabled.trace_add("write", self._auto_enable_gamepad_xyz)

        for var in (
            self.fixed_tcp_roll, self.fixed_tcp_pitch, self.fixed_tcp_yaw,
            self.fixed_tcp_feed, self.fixed_tcp_mode, self.fixed_tcp_enabled,
            self.fixed_tcp_dx, self.fixed_tcp_dy, self.fixed_tcp_dz,
        ):
            var.trace_add("write", lambda *_: self._update_fixed_gcode_preview())

        self._update_plane_markers()

    # ── Slider builder ──────────────────────────────────────────────

    def _make_slider_row(
        self, parent: tk.Widget, label: str, var: tk.DoubleVar
    ) -> Tuple[ttk.Scale, ttk.Entry]:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label, width=6).pack(side=tk.LEFT)
        scale = ttk.Scale(row, from_=-300.0, to=300.0, variable=var, length=170)
        scale.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)
        ent = ttk.Entry(row, width=7, justify="right")
        ent.pack(side=tk.LEFT, padx=(4, 0))
        self._sync_entry(ent, var)
        self._bind_entry(ent, var)

        def _on_scale(_val: Any = None) -> None:
            self._sync_entry(ent, var)
            self._update_plane_markers()

        def _on_press(_event: Any = None) -> None:
            self.fixed_tcp_user_dragging = True

        def _on_release(_event: Any = None) -> None:
            self.fixed_tcp_user_dragging = False
            if not self.fixed_tcp_enabled.get():
                return
            if self.fixed_tcp_exec_on_release.get():
                self.apply_fixed_target(execute=True)

        def _on_var_change(*_args: Any) -> None:
            try:
                self._sync_entry(ent, var)
                self._update_plane_markers()
            except (tk.TclError, ValueError):
                pass

        scale.configure(command=_on_scale)
        scale.bind("<ButtonPress-1>", _on_press)
        scale.bind("<ButtonRelease-1>", _on_release)
        var.trace_add("write", _on_var_change)
        return scale, ent

    @staticmethod
    def _sync_entry(entry: ttk.Entry, var: tk.DoubleVar) -> None:
        entry.delete(0, tk.END)
        entry.insert(0, f"{float(var.get()):.1f}")

    def _bind_entry(self, entry: ttk.Entry, var: tk.DoubleVar) -> None:
        def _on_enter(event: Any = None) -> str:
            val = self._clamp_offset(entry.get())
            var.set(val)
            self._sync_entry(entry, var)
            if self.fixed_tcp_exec_on_release.get():
                self.apply_fixed_target(execute=True)
            return "break"

        entry.bind("<Return>", _on_enter)

    # ── Plane canvas builder ────────────────────────────────────────

    def _make_plane(self, parent: tk.Widget, label_text: str) -> Tuple[tk.Canvas, int]:
        wrap = ttk.Frame(parent)
        wrap.pack(side=tk.LEFT, padx=4)
        ttk.Label(wrap, text=label_text).pack()
        c = tk.Canvas(
            wrap,
            width=self._canvas_size, height=self._canvas_size,
            highlightthickness=1, highlightbackground="#666",
        )
        c.pack()
        center = self._center
        rp = self._radius_px
        c.create_oval(center - rp, center - rp, center + rp, center + rp, outline="#666")
        c.create_line(center, 0, center, self._canvas_size, fill="#444")
        c.create_line(0, center, self._canvas_size, center, fill="#444")
        dot = c.create_oval(center - 4, center - 4, center + 4, center + 4,
                            fill="#d32f2f", outline="")
        return c, dot

    def _bind_plane(self, canvas: tk.Canvas, a_axis: str, b_axis: str) -> None:
        def _on_drag(event: tk.Event) -> None:
            self.fixed_tcp_user_dragging = True
            self._set_from_event(event, a_axis, b_axis)

        def _on_release(event: tk.Event) -> None:
            self._set_from_event(event, a_axis, b_axis)
            self.fixed_tcp_user_dragging = False
            if not self.fixed_tcp_enabled.get():
                return
            if self.fixed_tcp_exec_on_release.get():
                self.apply_fixed_target(execute=True)

        canvas.bind("<B1-Motion>", _on_drag)
        canvas.bind("<ButtonRelease-1>", _on_release)

    def _set_from_event(self, event: tk.Event, a_axis: str, b_axis: str) -> None:
        if not self.fixed_tcp_enabled.get():
            return
        center = self._center
        rp = self._radius_px
        rm = self._radius_mm
        dx_px = event.x - center
        dy_px = center - event.y
        r = math.hypot(dx_px, dy_px)
        if r > rp and r > 1e-6:
            scale = rp / r
            dx_px *= scale
            dy_px *= scale
        a = (dx_px / rp) * rm
        b = (dy_px / rp) * rm
        axis_map = {"X": self.fixed_tcp_dx, "Y": self.fixed_tcp_dy, "Z": self.fixed_tcp_dz}
        if a_axis in axis_map:
            axis_map[a_axis].set(a)
        if b_axis in axis_map:
            axis_map[b_axis].set(b)
        self._update_plane_markers()

    def _update_plane_markers(self) -> None:
        dx = self._clamp_offset(self.fixed_tcp_dx.get())
        dy = self._clamp_offset(self.fixed_tcp_dy.get())
        dz = self._clamp_offset(self.fixed_tcp_dz.get())
        center = self._center
        rp = self._radius_px
        rm = self._radius_mm

        def _pos(a: float, b: float) -> Tuple[float, float]:
            px = center + (a / rm) * rp
            py = center - (b / rm) * rp
            return px, py

        px, py = _pos(dx, dy)
        self._xy_canvas.coords(self._xy_dot, px - 4, py - 4, px + 4, py + 4)

        px, py = _pos(dx, dz)
        self._xz_canvas.coords(self._xz_dot, px - 4, py - 4, px + 4, py + 4)

        px, py = _pos(dy, dz)
        self._yz_canvas.coords(self._yz_dot, px - 4, py - 4, px + 4, py + 4)

    # ── Button callbacks ────────────────────────────────────────────

    def _move_rel(self, dx: float, dy: float, dz: float) -> None:
        if not self.fixed_tcp_enabled.get():
            return
        cur_dx = self._clamp_offset(self.fixed_tcp_dx.get())
        cur_dy = self._clamp_offset(self.fixed_tcp_dy.get())
        cur_dz = self._clamp_offset(self.fixed_tcp_dz.get())
        self._set_fixed_offsets(cur_dx + dx, cur_dy + dy, cur_dz + dz)
        try:
            self.apply_fixed_target(execute=True)
        except (RuntimeError, ValueError, ZeroDivisionError) as e:
            self._log(f"Fixed TCP button move error: {e}")

    def _gp_test_go(self) -> None:
        if not self.fixed_tcp_enabled.get():
            return
        try:
            step = max(0.1, float(self.fixed_tcp_gp_xy_step.get()))
        except (ValueError, TypeError):
            step = 1.0
        self._move_rel(step, 0.0, 0.0)

    def _set_all_offsets(self) -> None:
        self._set_fixed_offsets(0.0, 0.0, 0.0)
        if self.fixed_tcp_exec_on_release.get():
            self.apply_fixed_target(execute=True)

    def _enqueue_fixed_target(self) -> None:
        if not self.fixed_tcp_enabled.get():
            return
        mode = (self.fixed_tcp_mode.get() or "world").lower()
        if mode == "point":
            self._log("Fixed TCP queue: point mode uses live orientation; skipping.")
            return
        x, y, z, _r, _p, _y, _dx, _dy, _dz = self.calc_fixed_target()
        feed = float(self.fixed_tcp_feed.get())
        if not self.tcp_gcode_mode.get():
            self.tcp_gcode_mode.set(True)
            self._log("Fixed TCP queue: TCP G-code mode enabled.")
        gline = f"G90 G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{feed:.0f}"
        if hasattr(self.app, "enqueue"):
            self.app.enqueue(gline)

    def _home_fixed_xyz(self) -> None:
        self._set_fixed_offsets(0.0, 0.0, 0.0)
        origin_axes = self.fixed_tcp_origin_axes
        feed = float(self.fixed_tcp_feed.get())
        if origin_axes and hasattr(self.app, "client"):
            self.app.client.send_line("G90")
            parts = [f"{ax}{origin_axes.get(ax, 0.0):.3f}" for ax in AXES]
            self.app.client.send_line("G1 " + " ".join(parts) + f" F{feed:.0f}")
            self._request_fixed_delta_sync(target_delta=(0.0, 0.0, 0.0))
            self._log("Fixed TCP home -> stored joint origin.")
            return
        roll = float(self.fixed_tcp_roll.get())
        pitch = float(self.fixed_tcp_pitch.get())
        yaw = float(self.fixed_tcp_yaw.get())
        if hasattr(self.app, "kinematics_tabs"):
            self.app.kinematics_tabs.move_tcp_pose(
                self.fixed_tcp_origin_x,
                self.fixed_tcp_origin_y,
                self.fixed_tcp_origin_z,
                roll, pitch, yaw,
                feed=feed,
                allow_out_of_limits=False,
            )

    # ── Variable trace callbacks ────────────────────────────────────

    def _sync_modes(self, *_args: Any) -> None:
        if self._fixed_tcp_guard:
            return
        self._fixed_tcp_guard = True
        try:
            enabled = bool(self.fixed_tcp_enabled.get())
            if not enabled:
                if self.fixed_tcp_exec_on_release.get():
                    self.fixed_tcp_exec_on_release.set(False)
                if self.tcp_gcode_mode.get():
                    self.tcp_gcode_mode.set(False)
                if self.fixed_tcp_gamepad_mode.get():
                    self.fixed_tcp_gamepad_mode.set(False)
            else:
                if not self.fixed_tcp_exec_on_release.get():
                    self.fixed_tcp_exec_on_release.set(True)
            if self.tcp_gcode_mode.get() and not enabled:
                self.fixed_tcp_enabled.set(True)
                enabled = True
            state = "normal" if enabled else "disabled"
            for w in self._fixed_mode_widgets:
                try:
                    w.configure(state=state)
                except (tk.TclError, AttributeError):
                    pass
        finally:
            self._fixed_tcp_guard = False

    def _update_gp_point_notice(self, *_: Any) -> None:
        if (self.fixed_tcp_mode.get() == "point"
                and self.fixed_tcp_gamepad_mode.get()):
            self._gp_point_lbl.configure(text="\u2192 Spitze zeigt auf Fixpunkt")
        else:
            self._gp_point_lbl.configure(text="")
        # Reset RPY continuity cache when gamepad mode is toggled
        if not self.fixed_tcp_gamepad_mode.get():
            self._gp_tcp_prev_rpy = None

    def _auto_enable_gamepad_xyz(self, *_: Any) -> None:
        """Auto-enable Gamepad XYZ when Fixed TCP is manually turned on."""
        if self.fixed_tcp_enabled.get() and not self.fixed_tcp_gamepad_mode.get():
            self.fixed_tcp_gamepad_mode.set(True)

    # ── Help dialog ─────────────────────────────────────────────────

    def _show_help(self) -> None:
        win = tk.Toplevel(self)
        win.title("Fixed TCP Help")
        win.geometry("640x460")
        win.resizable(False, False)
        wrap = ttk.Frame(win, padding=8)
        wrap.pack(fill=tk.BOTH, expand=True)

        ttk.Label(
            wrap, text="Fixed TCP: quick guide",
            font=("Segoe UI", 11, "bold"),
        ).pack(anchor="w", pady=(0, 6))

        c = tk.Canvas(
            wrap, width=600, height=220, bg="white",
            highlightthickness=1, highlightbackground="#999",
        )
        c.pack(pady=(0, 6))

        # Simple graphic: origin, offsets, tool frame, and point target
        c.create_text(10, 10, anchor="nw", text="Fixed mode flow", fill="#333")
        c.create_oval(60, 70, 90, 100, outline="#333")
        c.create_line(75, 100, 75, 150, arrow="last", fill="#333")
        c.create_text(75, 160, text="TCP", fill="#333")
        c.create_text(50, 55, text="Fix", fill="#333")

        # Offset arrow
        c.create_line(75, 85, 210, 100, arrow="last", fill="#2e7d32", width=2)
        c.create_text(135, 70, text="dx/dy/dz", fill="#2e7d32")

        # Tool frame axes
        c.create_line(260, 120, 320, 120, arrow="last", fill="#1565c0")
        c.create_line(260, 120, 260, 60, arrow="last", fill="#1565c0")
        c.create_text(330, 120, text="tool +X", fill="#1565c0", anchor="w")
        c.create_text(260, 50, text="tool +Z", fill="#1565c0", anchor="s")
        c.create_text(240, 40, text="Tool frame", fill="#1565c0")

        # Point target
        c.create_oval(430, 70, 460, 100, outline="#333")
        c.create_line(260, 120, 445, 85, arrow="last", fill="#ad1457", width=2)
        c.create_text(470, 85, text="TCP Point target", fill="#ad1457", anchor="w")

        ttk.Label(
            wrap, text="Controls",
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w", pady=(4, 2))

        steps = [
            "Fixed mode: lock Roll/Pitch/Yaw to the fixed values while moving XYZ.",
            "Execute on release: moves are sent only when you release a slider/drag.",
            "G-code TCP mode: incoming G0/G1 X/Y/Z are converted to TCP moves.",
            "Mode=World: offsets are in world XYZ (no rotation applied).",
            "Mode=Tool frame: offsets are applied in the tool's local axes.",
            "Mode=TCP Point: TCP keeps pointing at a fixed 3D point.",
            "Point dX/dY/dZ: defines the TCP Point relative to current TCP.",
        ]
        for s in steps:
            ttk.Label(wrap, text="- " + s).pack(anchor="w")
