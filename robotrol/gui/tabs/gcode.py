"""
Robotrol v2.0 — G-Code tab.

Plane definition (3-point UV plane from TCP), arc generator (G2/G3),
G-code file import with interpreter, and 2-D toolpath preview.

Extracted from Robotrol_FluidNC_v7_3.py lines 2140-3320.
"""
from __future__ import annotations

import math
import os
import threading
import tkinter as tk
import tkinter.filedialog as fd
import tkinter.scrolledtext as scrolledtext
from tkinter import ttk
from typing import Any, Dict, List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------
# Vector helpers (pure functions, no dependencies)
# ---------------------------------------------------------------------------

Vec3 = Tuple[float, float, float]


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vec3) -> float:
    return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5


def _normalize(v: Vec3, default: Vec3) -> Vec3:
    n = _norm(v)
    if n < 1e-9:
        return default
    return (v[0] / n, v[1] / n, v[2] / n)


def _rpy_to_rotation_matrix(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> List[List[float]]:
    """ZYX intrinsic rotation matrix from roll/pitch/yaw in degrees."""
    yaw_r = math.radians(yaw_deg)
    pitch_r = math.radians(pitch_deg)
    roll_r = math.radians(roll_deg)
    cy, sy = math.cos(yaw_r), math.sin(yaw_r)
    cp, sp = math.cos(pitch_r), math.sin(pitch_r)
    cr, sr = math.cos(roll_r), math.sin(roll_r)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _detect_native_plane(
    n_axis: Vec3,
) -> Optional[Dict[str, Any]]:
    """If *n_axis* is close to a canonical axis, return G17/G18/G19 mapping."""
    ax, ay, az = abs(n_axis[0]), abs(n_axis[1]), abs(n_axis[2])
    tol = 0.95
    if az >= ay and az >= ax and az >= tol:
        return {
            "gplane": "G17",
            "a": ("X", 0), "b": ("Y", 1), "c": ("Z", 2),
            "offsets": ("I", "J"),
        }
    if ay >= ax and ay >= az and ay >= tol:
        return {
            "gplane": "G18",
            "a": ("X", 0), "b": ("Z", 2), "c": ("Y", 1),
            "offsets": ("I", "K"),
        }
    if ax >= ay and ax >= az and ax >= tol:
        return {
            "gplane": "G19",
            "a": ("Y", 1), "b": ("Z", 2), "c": ("X", 0),
            "offsets": ("J", "K"),
        }
    return None


def _world_axis_value(
    origin: Vec3, u_axis: Vec3, v_axis: Vec3, n_axis: Vec3,
    uu: float, vv: float, ww: float, idx: int,
) -> float:
    return (
        origin[idx]
        + uu * u_axis[idx]
        + vv * v_axis[idx]
        + ww * n_axis[idx]
    )


def _parse_gcode_line(raw: str) -> Optional[Dict[str, Any]]:
    """Parse a single G-code line into {cmd, params} or *None*."""
    line = raw
    if ";" in line:
        line = line[: line.index(";")]
    line = line.strip().upper()
    if not line:
        return None
    tokens = line.split()
    if not tokens:
        return None
    cmd = tokens[0]
    params: Dict[str, float] = {}
    for tok in tokens[1:]:
        if len(tok) >= 2 and tok[0].isalpha():
            try:
                params[tok[0]] = float(tok[1:])
            except ValueError:
                pass
    return {"cmd": cmd, "params": params}


# ---------------------------------------------------------------------------
# GCodeTab
# ---------------------------------------------------------------------------


class GCodeTab(ttk.Frame):
    """G-code plane definition, arc generator, file interpreter, and preview."""

    def __init__(self, parent: tk.Widget, app: Any) -> None:
        super().__init__(parent)
        self.app = app

        # ── Plane model ──────────────────────────────────────────────────
        self.plane_model: Dict[str, Any] = {
            "origin": (0.0, 0.0, 0.0),
            "u": (1.0, 0.0, 0.0),
            "v": (0.0, 1.0, 0.0),
            "n": (0.0, 0.0, 1.0),
            "rpy": (0.0, 0.0, 0.0),
        }
        self.plane_defined: bool = False

        # ── Tk variables — plane vectors ─────────────────────────────────
        self.plane_origin_x = tk.DoubleVar(value=0.0)
        self.plane_origin_y = tk.DoubleVar(value=0.0)
        self.plane_origin_z = tk.DoubleVar(value=0.0)
        self.plane_u_x = tk.DoubleVar(value=1.0)
        self.plane_u_y = tk.DoubleVar(value=0.0)
        self.plane_u_z = tk.DoubleVar(value=0.0)
        self.plane_v_x = tk.DoubleVar(value=0.0)
        self.plane_v_y = tk.DoubleVar(value=1.0)
        self.plane_v_z = tk.DoubleVar(value=0.0)
        self.plane_n_x = tk.DoubleVar(value=0.0)
        self.plane_n_y = tk.DoubleVar(value=0.0)
        self.plane_n_z = tk.DoubleVar(value=1.0)
        self.plane_roll = tk.DoubleVar(value=0.0)
        self.plane_pitch = tk.DoubleVar(value=0.0)
        self.plane_yaw = tk.DoubleVar(value=0.0)

        # ── Tk variables — arc parameters ────────────────────────────────
        self.plane_abs_mode = tk.BooleanVar(value=True)
        self.plane_dir = tk.StringVar(value="G2")
        self.plane_end_u = tk.DoubleVar(value=20.0)
        self.plane_end_v = tk.DoubleVar(value=0.0)
        self.plane_center_i = tk.DoubleVar(value=10.0)
        self.plane_center_j = tk.DoubleVar(value=0.0)
        self.plane_w = tk.DoubleVar(value=0.0)
        self.plane_feed = tk.DoubleVar(value=3000.0)
        self.plane_seg_len = tk.DoubleVar(value=2.0)

        # ── Tk variables — G-code file interpreter ───────────────────────
        self.plane_gcode_lines: List[str] = []
        self.plane_gcode_path = tk.StringVar(value="")
        self.plane_gcode_status = tk.StringVar(value="No file loaded.")
        self.plane_gcode_progress = tk.StringVar(value="")
        self.plane_scale_x = tk.DoubleVar(value=1.0)
        self.plane_scale_y = tk.DoubleVar(value=1.0)
        self.plane_mirror_x = tk.BooleanVar(value=False)
        self.plane_mirror_y = tk.BooleanVar(value=False)
        self.plane_rotate = tk.StringVar(value="0\u00b0")
        self.plane_offset_u = tk.DoubleVar(value=0.0)
        self.plane_offset_v = tk.DoubleVar(value=0.0)
        self.plane_dry_run = tk.BooleanVar(value=True)

        # ── Worker state ─────────────────────────────────────────────────
        self.plane_gcode_worker: Optional[threading.Thread] = None
        self.plane_gcode_stop_event = threading.Event()
        self.plane_preview_canvas: Optional[tk.Canvas] = None

        self._build_ui()

    # ══════════════════════════════════════════════════════════════════════
    # Logging helper
    # ══════════════════════════════════════════════════════════════════════

    def _log(self, msg: str) -> None:
        if hasattr(self.app, "log"):
            self.app.log(msg)
        else:
            print(msg)

    # ══════════════════════════════════════════════════════════════════════
    # Plane helpers
    # ══════════════════════════════════════════════════════════════════════

    def _get_origin(self) -> Vec3:
        ox = float(self.plane_origin_x.get())
        oy = float(self.plane_origin_y.get())
        oz = float(self.plane_origin_z.get())
        self.plane_model["origin"] = (ox, oy, oz)
        return (ox, oy, oz)

    def _get_rpy(self) -> Vec3:
        roll = float(self.plane_roll.get())
        pitch = float(self.plane_pitch.get())
        yaw = float(self.plane_yaw.get())
        self.plane_model["rpy"] = (roll, pitch, yaw)
        return (roll, pitch, yaw)

    def _get_axes(self) -> Tuple[Vec3, Vec3, Vec3]:
        u: Vec3 = (
            float(self.plane_u_x.get()),
            float(self.plane_u_y.get()),
            float(self.plane_u_z.get()),
        )
        v: Vec3 = (
            float(self.plane_v_x.get()),
            float(self.plane_v_y.get()),
            float(self.plane_v_z.get()),
        )
        u = _normalize(u, (1.0, 0.0, 0.0))
        dot_vu = _dot(v, u)
        v = (v[0] - dot_vu * u[0], v[1] - dot_vu * u[1], v[2] - dot_vu * u[2])
        v = _normalize(v, (0.0, 1.0, 0.0))
        n = _cross(u, v)
        n = _normalize(n, (0.0, 0.0, 1.0))
        return u, v, n

    def _sync_axes(self) -> Tuple[Vec3, Vec3, Vec3]:
        u, v, n = self._get_axes()
        self.plane_u_x.set(u[0])
        self.plane_u_y.set(u[1])
        self.plane_u_z.set(u[2])
        self.plane_v_x.set(v[0])
        self.plane_v_y.set(v[1])
        self.plane_v_z.set(v[2])
        self.plane_n_x.set(n[0])
        self.plane_n_y.set(n[1])
        self.plane_n_z.set(n[2])
        self.plane_model["u"] = u
        self.plane_model["v"] = v
        self.plane_model["n"] = n
        return u, v, n

    def _set_model(
        self, origin: Vec3, u: Vec3, v: Vec3, n: Vec3, rpy: Vec3
    ) -> None:
        ox, oy, oz = origin
        u = _normalize(u, (1.0, 0.0, 0.0))
        v = _normalize(v, (0.0, 1.0, 0.0))
        n = _normalize(n, (0.0, 0.0, 1.0))
        self.plane_origin_x.set(ox)
        self.plane_origin_y.set(oy)
        self.plane_origin_z.set(oz)
        self.plane_u_x.set(u[0])
        self.plane_u_y.set(u[1])
        self.plane_u_z.set(u[2])
        self.plane_v_x.set(v[0])
        self.plane_v_y.set(v[1])
        self.plane_v_z.set(v[2])
        self.plane_n_x.set(n[0])
        self.plane_n_y.set(n[1])
        self.plane_n_z.set(n[2])
        roll, pitch, yaw = rpy
        self.plane_roll.set(roll)
        self.plane_pitch.set(pitch)
        self.plane_yaw.set(yaw)
        self.plane_model["origin"] = (ox, oy, oz)
        self.plane_model["u"] = u
        self.plane_model["v"] = v
        self.plane_model["n"] = n
        self.plane_model["rpy"] = (roll, pitch, yaw)
        self.plane_defined = True

    # ══════════════════════════════════════════════════════════════════════
    # Plane actions
    # ══════════════════════════════════════════════════════════════════════

    def _send_frame(self) -> None:
        try:
            origin = self._get_origin()
            u, v, n = self._sync_axes()
            if hasattr(self.app, "_send_vis_fixed_frame"):
                self.app._send_vis_fixed_frame(origin, u, v, n)
        except (ValueError, TypeError, AttributeError):
            pass

    def _set_from_current(self) -> None:
        try:
            tcp = self.app.get_current_tcp_mm()
        except (AttributeError, RuntimeError):
            tcp = None
        if not tcp:
            self._log("Plane: no TCP pose available.")
            return
        ox = float(tcp.get("X_mm", 0.0))
        oy = float(tcp.get("Y_mm", 0.0))
        oz = float(tcp.get("Z_mm", 0.0))
        roll = float(tcp.get("Roll_deg", 0.0))
        pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
        yaw = float(tcp.get("Yaw_deg", 0.0))
        R = _rpy_to_rotation_matrix(roll, pitch, yaw)
        u: Vec3 = (R[0][0], R[1][0], R[2][0])
        v: Vec3 = (R[0][1], R[1][1], R[2][1])
        n: Vec3 = (R[0][2], R[1][2], R[2][2])
        self._set_model((ox, oy, oz), u, v, n, (roll, pitch, yaw))
        self.plane_w.set(0.0)
        self._log("Plane defined from current TCP.")
        self._send_frame()

    def _set_world_xy(self) -> None:
        try:
            tcp = self.app.get_current_tcp_mm()
        except (AttributeError, RuntimeError):
            tcp = None
        if not tcp:
            self._log("Plane: no TCP pose available.")
            return
        ox = float(tcp.get("X_mm", 0.0))
        oy = float(tcp.get("Y_mm", 0.0))
        oz = float(tcp.get("Z_mm", 0.0))
        roll = float(tcp.get("Roll_deg", 0.0))
        pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
        yaw = float(tcp.get("Yaw_deg", 0.0))
        self._set_model(
            (ox, oy, oz),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (roll, pitch, yaw),
        )
        self.plane_w.set(0.0)
        self._log("Plane set to world XY at current TCP.")
        self._send_frame()

    # ══════════════════════════════════════════════════════════════════════
    # Arc computation
    # ══════════════════════════════════════════════════════════════════════

    def _arc_data(self) -> Optional[Dict[str, Any]]:
        if not self.plane_defined:
            self._log("Plane not defined. Use 'Set plane from current TCP'.")
            return None
        origin = self._get_origin()
        u_axis, v_axis, n_axis = self._sync_axes()
        roll, pitch, yaw = self._get_rpy()
        try:
            tcp = self.app.get_current_tcp_mm()
        except (AttributeError, RuntimeError):
            tcp = None
        if not tcp:
            self._log("Plane: no TCP pose available.")
            return None
        px = float(tcp.get("X_mm", 0.0))
        py = float(tcp.get("Y_mm", 0.0))
        pz = float(tcp.get("Z_mm", 0.0))
        rel: Vec3 = (px - origin[0], py - origin[1], pz - origin[2])
        u0 = _dot(rel, u_axis)
        v0 = _dot(rel, v_axis)
        w0 = _dot(rel, n_axis)
        w = float(self.plane_w.get())
        if abs(w0 - w) > 0.5:
            self._log(f"Plane: current W offset is {w0:.2f}mm, target {w:.2f}mm.")

        if self.plane_abs_mode.get():
            u1 = float(self.plane_end_u.get())
            v1 = float(self.plane_end_v.get())
        else:
            u1 = u0 + float(self.plane_end_u.get())
            v1 = v0 + float(self.plane_end_v.get())

        cx = u0 + float(self.plane_center_i.get())
        cy = v0 + float(self.plane_center_j.get())
        r0 = math.hypot(u0 - cx, v0 - cy)
        r1 = math.hypot(u1 - cx, v1 - cy)
        if r0 < 1e-6:
            self._log("Plane arc: radius too small.")
            return None
        if abs(r0 - r1) > 0.5:
            self._log(f"Plane arc: radius mismatch (r0={r0:.3f}, r1={r1:.3f}).")
            return None

        start = math.atan2(v0 - cy, u0 - cx)
        end = math.atan2(v1 - cy, u1 - cx)
        if abs(u1 - u0) < 1e-6 and abs(v1 - v0) < 1e-6:
            delta = -2.0 * math.pi if self.plane_dir.get() == "G2" else 2.0 * math.pi
        else:
            delta = end - start
            if self.plane_dir.get() == "G2":
                if delta >= 0.0:
                    delta -= 2.0 * math.pi
            else:
                if delta <= 0.0:
                    delta += 2.0 * math.pi

        arc_len = abs(delta) * r0
        try:
            feed = float(self.plane_feed.get())
        except (ValueError, tk.TclError):
            feed = 3000.0
        return {
            "origin": origin,
            "u_axis": u_axis,
            "v_axis": v_axis,
            "n_axis": n_axis,
            "rpy": (roll, pitch, yaw),
            "u0": u0, "v0": v0, "w": w,
            "u1": u1, "v1": v1,
            "cx": cx, "cy": cy,
            "r0": r0, "arc_len": arc_len,
            "start": start, "delta": delta,
            "feed": feed,
        }

    def _arc_points(
        self,
    ) -> Optional[Tuple[List[Vec3], Vec3, float]]:
        arc = self._arc_data()
        if not arc:
            return None
        origin = arc["origin"]
        u_axis = arc["u_axis"]
        v_axis = arc["v_axis"]
        n_axis = arc["n_axis"]
        w = arc["w"]
        cx = arc["cx"]
        cy = arc["cy"]
        r0 = arc["r0"]
        start = arc["start"]
        delta = arc["delta"]
        arc_len = arc["arc_len"]
        feed = arc["feed"]
        roll, pitch, yaw = arc["rpy"]

        try:
            seg_len = float(self.plane_seg_len.get())
        except (ValueError, tk.TclError):
            seg_len = 2.0
        seg_len = max(0.5, seg_len)
        steps = max(3, int(math.ceil(arc_len / seg_len)))
        pts: List[Vec3] = []
        for i in range(1, steps + 1):
            ang = start + delta * (i / steps)
            uu = cx + r0 * math.cos(ang)
            vv = cy + r0 * math.sin(ang)
            x = origin[0] + uu * u_axis[0] + vv * v_axis[0] + w * n_axis[0]
            y = origin[1] + uu * u_axis[1] + vv * v_axis[1] + w * n_axis[1]
            z = origin[2] + uu * u_axis[2] + vv * v_axis[2] + w * n_axis[2]
            pts.append((x, y, z))
        self._log(
            f"Plane arc: {len(pts)} segments, len={arc_len:.1f}mm, "
            f"dir={self.plane_dir.get()}"
        )
        return pts, (roll, pitch, yaw), feed

    # ══════════════════════════════════════════════════════════════════════
    # Arc execution / queuing
    # ══════════════════════════════════════════════════════════════════════

    def _arc_send(self) -> None:
        data = self._arc_points()
        if not data:
            return
        if not hasattr(self.app, "kinematics_tabs"):
            self._log("Plane arc: no kinematics available.")
            return
        pts, rpy, feed = data
        roll, pitch, yaw = rpy
        ok_count = 0
        for x, y, z in pts:
            ok = self.app.kinematics_tabs.move_tcp_pose(
                x, y, z, roll, pitch, yaw,
                feed=feed, allow_out_of_limits=False,
            )
            if not ok:
                self._log("Plane arc aborted (IK/limits).")
                break
            ok_count += 1
        self._log(f"Plane arc sent: {ok_count}/{len(pts)} segments.")

    def _arc_queue(self) -> None:
        data = self._arc_points()
        if not data:
            return
        if (
            not hasattr(self.app, "kinematics_tabs")
            or not hasattr(self.app.kinematics_tabs, "preview_tcp_gcode")
        ):
            self._log("Plane arc: no preview/queue available.")
            return
        pts, rpy, feed = data
        roll, pitch, yaw = rpy
        count = 0
        for x, y, z in pts:
            res = self.app.kinematics_tabs.preview_tcp_gcode(
                x, y, z, roll, pitch, yaw, feed=feed,
            )
            if not res or not res.get("gcode"):
                self._log("Plane arc queue aborted (preview failed).")
                break
            if res.get("ok") is False:
                self._log("Plane arc queue aborted (limits).")
                break
            self.app.enqueue(res["gcode"])
            count += 1
        self._log(f"Plane arc queued: {count}/{len(pts)} segments.")

    def _arc_queue_native(self) -> None:
        arc = self._arc_data()
        if not arc:
            return
        plane = _detect_native_plane(arc["n_axis"])
        if not plane:
            self._log(
                "Native G2/G3 requires an almost-canonical plane normal (XY/XZ/YZ)."
            )
            return

        origin = arc["origin"]
        u_axis = arc["u_axis"]
        v_axis = arc["v_axis"]
        n_axis = arc["n_axis"]
        w = arc["w"]
        u0, v0 = arc["u0"], arc["v0"]
        u1, v1 = arc["u1"], arc["v1"]
        cx, cy = arc["cx"], arc["cy"]
        feed = arc["feed"]

        an, ai = plane["a"]
        bn, bi = plane["b"]
        cn, ci = plane["c"]
        off_a, off_b = plane["offsets"]

        start_a = _world_axis_value(origin, u_axis, v_axis, n_axis, u0, v0, w, ai)
        start_b = _world_axis_value(origin, u_axis, v_axis, n_axis, u0, v0, w, bi)
        end_a = _world_axis_value(origin, u_axis, v_axis, n_axis, u1, v1, w, ai)
        end_b = _world_axis_value(origin, u_axis, v_axis, n_axis, u1, v1, w, bi)
        end_c = _world_axis_value(origin, u_axis, v_axis, n_axis, u1, v1, w, ci)
        center_a = _world_axis_value(origin, u_axis, v_axis, n_axis, cx, cy, w, ai)
        center_b = _world_axis_value(origin, u_axis, v_axis, n_axis, cx, cy, w, bi)
        off_val_a = center_a - start_a
        off_val_b = center_b - start_b

        gcode = (
            f"{self.plane_dir.get()} "
            f"{an}{end_a:.3f} {bn}{end_b:.3f} {cn}{end_c:.3f} "
            f"{off_a}{off_val_a:.3f} {off_b}{off_val_b:.3f} "
            f"F{feed:.0f}"
        )
        self.app.enqueue("G90")
        self.app.enqueue(plane["gplane"])
        self.app.enqueue(gcode)
        self._log(
            f"Native arc queued: {plane['gplane']} + {self.plane_dir.get()} "
            f"({an}{bn}{cn})"
        )

    # ══════════════════════════════════════════════════════════════════════
    # Example G-code helpers
    # ══════════════════════════════════════════════════════════════════════

    def _queue_gcode_example(
        self, filename: str, fallback_lines: Sequence[str], label: str,
    ) -> None:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        example_path = os.path.join(base_dir, "..", "..", "data", "examples", filename)
        lines: List[str] = []
        try:
            with open(example_path, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f.readlines() if ln.strip()]
        except FileNotFoundError:
            lines = list(fallback_lines)
        for ln in lines:
            self.app.enqueue(ln)
        self._log(f"Queued {label} ({len(lines)} lines) from {example_path}.")

    def _queue_3dp_example_g17(self) -> None:
        self._queue_gcode_example(
            "plane_g2g3_3dp_example.gcode",
            [
                "; 3D printer G2/G3 quick example (G17 XY)",
                "G21", "G90", "G17",
                "G0 X20.000 Y20.000 Z0.300 F3000",
                "G2 X60.000 Y20.000 I20.000 J0.000 F1200",
                "G3 X20.000 Y20.000 I-20.000 J0.000 F1200",
            ],
            "3DP example G17",
        )

    def _queue_3dp_example_g18(self) -> None:
        self._queue_gcode_example(
            "plane_g2g3_3dp_example_g18.gcode",
            [
                "; 3D printer G2/G3 quick example (G18 XZ)",
                "G21", "G90", "G18",
                "G0 X20.000 Y0.300 Z20.000 F3000",
                "G2 X60.000 Z20.000 I20.000 K0.000 F1200",
                "G3 X20.000 Z20.000 I-20.000 K0.000 F1200",
            ],
            "3DP example G18",
        )

    def _queue_3dp_example_g19(self) -> None:
        self._queue_gcode_example(
            "plane_g2g3_3dp_example_g19.gcode",
            [
                "; 3D printer G2/G3 quick example (G19 YZ)",
                "G21", "G90", "G19",
                "G0 X0.300 Y20.000 Z20.000 F3000",
                "G2 Y60.000 Z20.000 J20.000 K0.000 F1200",
                "G3 Y20.000 Z20.000 J-20.000 K0.000 F1200",
            ],
            "3DP example G19",
        )

    def _queue_3dp_examples_all(self) -> None:
        self.app.enqueue("; ---- Dry-run bundle start: G17/G18/G19 ----")
        self.app.enqueue("; ---- Example G17 ----")
        self._queue_3dp_example_g17()
        self.app.enqueue("; ---- Example G18 ----")
        self._queue_3dp_example_g18()
        self.app.enqueue("; ---- Example G19 ----")
        self._queue_3dp_example_g19()
        self.app.enqueue("; ---- Dry-run bundle end ----")
        self._log("Queued dry-run bundle: G17 + G18 + G19.")

    # ══════════════════════════════════════════════════════════════════════
    # G-code file interpreter
    # ══════════════════════════════════════════════════════════════════════

    def _exec_gcode(self, dry_run: bool) -> None:  # noqa: C901 — faithful port
        """Interpret loaded G-code lines; runs in a worker thread."""
        lines = self.plane_gcode_lines
        if not lines:
            self._log("G-code: no file loaded.")
            return
        if not self.plane_defined:
            self._log("G-code: plane not defined. Set plane from TCP first.")
            return
        if not dry_run and not hasattr(self.app, "kinematics_tabs"):
            self._log("G-code: kinematics not available.")
            return

        origin = self._get_origin()
        u_axis, v_axis, n_axis = self._sync_axes()
        roll, pitch, yaw = self._get_rpy()
        scale_x = max(1e-6, float(self.plane_scale_x.get()))
        scale_y = max(1e-6, float(self.plane_scale_y.get()))
        mx = -1.0 if self.plane_mirror_x.get() else 1.0
        my = -1.0 if self.plane_mirror_y.get() else 1.0
        rot = self.plane_rotate.get()
        off_u = float(self.plane_offset_u.get())
        off_v = float(self.plane_offset_v.get())
        seg_len = max(0.5, float(self.plane_seg_len.get()))
        stop = self.plane_gcode_stop_event

        # interpreter state
        abs_mode = True
        metric = True
        cur_u = 0.0
        cur_v = 0.0
        cur_w = 0.0
        cur_feed = float(self.plane_feed.get())
        move_count = 0
        total = len(lines)

        def transform_uv(
            rx: Optional[float], ry: Optional[float],
        ) -> Tuple[Optional[float], Optional[float]]:
            ff = 25.4 if not metric else 1.0
            u_r = rx * ff * scale_x * mx if rx is not None else None
            v_r = ry * ff * scale_y * my if ry is not None else None
            if u_r is not None and v_r is not None and rot != "0\u00b0":
                if rot == "90\u00b0":
                    u_r, v_r = -v_r, u_r
                elif rot == "180\u00b0":
                    u_r, v_r = -u_r, -v_r
                elif rot == "270\u00b0":
                    u_r, v_r = v_r, -u_r
            return u_r, v_r

        def to_uv(
            rx: Optional[float], ry: Optional[float], rz: Optional[float],
        ) -> Tuple[Optional[float], Optional[float], Optional[float]]:
            u_t, v_t = transform_uv(rx, ry)
            ff = 25.4 if not metric else 1.0
            u = u_t + off_u if u_t is not None else None
            v = v_t + off_v if v_t is not None else None
            w = rz * ff * (scale_x + scale_y) * 0.5 if rz is not None else None
            return u, v, w

        def world_xyz(uu: float, vv: float, ww: float) -> Vec3:
            x = origin[0] + uu * u_axis[0] + vv * v_axis[0] + ww * n_axis[0]
            y = origin[1] + uu * u_axis[1] + vv * v_axis[1] + ww * n_axis[1]
            z = origin[2] + uu * u_axis[2] + vv * v_axis[2] + ww * n_axis[2]
            return (x, y, z)

        def do_move(
            u1: Optional[float], v1: Optional[float],
            w1: Optional[float], feed: float,
        ) -> bool:
            nonlocal cur_u, cur_v, cur_w, move_count
            if u1 is None:
                u1 = cur_u
            if v1 is None:
                v1 = cur_v
            if w1 is None:
                w1 = cur_w
            if not dry_run:
                wx, wy, wz = world_xyz(u1, v1, w1)
                ok = self.app.kinematics_tabs.move_tcp_pose(
                    wx, wy, wz, roll, pitch, yaw, feed=feed,
                )
                if not ok:
                    return False
            move_count += 1
            cur_u, cur_v, cur_w = u1, v1, w1
            return True

        def do_arc(
            u1: Optional[float], v1: Optional[float],
            w1: Optional[float],
            ci: float, cj: float, direction: str, feed: float,
        ) -> bool:
            nonlocal cur_u, cur_v, cur_w, move_count
            if u1 is None:
                u1 = cur_u
            if v1 is None:
                v1 = cur_v
            if w1 is None:
                w1 = cur_w
            cxl = cur_u + ci
            cyl = cur_v + cj
            r0 = math.hypot(cur_u - cxl, cur_v - cyl)
            r1 = math.hypot(u1 - cxl, v1 - cyl)
            if r0 < 1e-6:
                self._log("G-code arc: radius too small, skipped.")
                cur_u, cur_v, cur_w = u1, v1, w1
                return True
            if abs(r0 - r1) > 0.5:
                self._log(
                    f"G-code arc: radius mismatch r0={r0:.3f} r1={r1:.3f}, skipped."
                )
                cur_u, cur_v, cur_w = u1, v1, w1
                return True
            start_a = math.atan2(cur_v - cyl, cur_u - cxl)
            end_a = math.atan2(v1 - cyl, u1 - cxl)
            if abs(u1 - cur_u) < 1e-6 and abs(v1 - cur_v) < 1e-6:
                delta = -2 * math.pi if direction == "G2" else 2 * math.pi
            else:
                delta = end_a - start_a
                if direction == "G2":
                    if delta >= 0:
                        delta -= 2 * math.pi
                else:
                    if delta <= 0:
                        delta += 2 * math.pi
            arc_len_local = abs(delta) * r0
            steps = max(3, int(math.ceil(arc_len_local / seg_len)))
            for i in range(1, steps + 1):
                if stop.is_set():
                    return False
                ang = start_a + delta * (i / steps)
                uu = cxl + r0 * math.cos(ang)
                vv = cyl + r0 * math.sin(ang)
                if not dry_run:
                    wx, wy, wz = world_xyz(uu, vv, w1)
                    ok = self.app.kinematics_tabs.move_tcp_pose(
                        wx, wy, wz, roll, pitch, yaw, feed=feed,
                    )
                    if not ok:
                        return False
                move_count += 1
            cur_u, cur_v, cur_w = u1, v1, w1
            return True

        for lineno, raw in enumerate(lines, 1):
            if stop.is_set():
                self._log(f"G-code stopped at line {lineno}.")
                break
            parsed = _parse_gcode_line(raw)
            if not parsed:
                continue
            cmd = parsed["cmd"]
            p = parsed["params"]
            if lineno % 100 == 0 or lineno == total:
                self.plane_gcode_progress.set(
                    f"Line {lineno}/{total}  moves={move_count}"
                )
            if cmd == "G20":
                metric = False
            elif cmd == "G21":
                metric = True
            elif cmd == "G90":
                abs_mode = True
            elif cmd == "G91":
                abs_mode = False
            elif cmd in ("G17", "G18", "G19", "G28", "G92"):
                pass  # plane select / home / set position: ignored
            elif cmd in ("G0", "G1"):
                if "F" in p:
                    cur_feed = p["F"] * (25.4 if not metric else 1.0)
                u1, v1, w1 = to_uv(p.get("X"), p.get("Y"), p.get("Z"))
                if not abs_mode:
                    if u1 is not None:
                        u1 += cur_u
                    if v1 is not None:
                        v1 += cur_v
                    if w1 is not None:
                        w1 += cur_w
                feed = cur_feed if cmd == "G1" else min(cur_feed, 6000.0)
                if not do_move(u1, v1, w1, feed):
                    self._log(f"G-code aborted at line {lineno} (move failed).")
                    break
            elif cmd in ("G2", "G3"):
                if "F" in p:
                    cur_feed = p["F"] * (25.4 if not metric else 1.0)
                ci_t, cj_t = transform_uv(p.get("I", 0.0), p.get("J", 0.0))
                ci_val = ci_t if ci_t is not None else 0.0
                cj_val = cj_t if cj_t is not None else 0.0
                u1, v1, w1 = to_uv(p.get("X"), p.get("Y"), p.get("Z"))
                if not abs_mode:
                    if u1 is not None:
                        u1 += cur_u
                    if v1 is not None:
                        v1 += cur_v
                if not do_arc(u1, v1, w1, ci_val, cj_val, cmd, cur_feed):
                    self._log(f"G-code aborted at line {lineno} (arc failed).")
                    break
            # M, T, S, N, E: silently ignored

        self.plane_gcode_progress.set(
            f"Done \u2014 {move_count} moves / {total} lines"
        )
        self._log(
            f"G-code {'dry-run' if dry_run else 'run'} complete: {move_count} moves."
        )

    # ══════════════════════════════════════════════════════════════════════
    # File operations
    # ══════════════════════════════════════════════════════════════════════

    def _load_gcode_file(self) -> None:
        path = fd.askopenfilename(
            title="Load G-code file",
            filetypes=[
                ("G-code", "*.gcode *.nc *.gc *.txt"),
                ("All files", "*.*"),
            ],
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as f:
                self.plane_gcode_lines = [ln.rstrip("\n") for ln in f]
            self.plane_gcode_path.set(path)
            self.plane_gcode_status.set(
                f"{len(self.plane_gcode_lines)} lines loaded."
            )
            self.plane_gcode_progress.set("")
            self._log(
                f"G-code loaded: {path} ({len(self.plane_gcode_lines)} lines)"
            )
        except OSError as e:
            self._log(f"G-code load failed: {e}")

    def _gcode_run(self) -> None:
        if self.plane_gcode_worker and self.plane_gcode_worker.is_alive():
            self._log("G-code: already running.")
            return
        self.plane_gcode_stop_event.clear()
        self.plane_gcode_progress.set("Starting\u2026")
        self.plane_gcode_worker = threading.Thread(
            target=self._exec_gcode,
            args=(bool(self.plane_dry_run.get()),),
            daemon=True,
        )
        self.plane_gcode_worker.start()

    def _gcode_stop(self) -> None:
        self.plane_gcode_stop_event.set()
        self._log("G-code stop requested.")

    # ══════════════════════════════════════════════════════════════════════
    # Preview canvas
    # ══════════════════════════════════════════════════════════════════════

    def _draw_preview(self) -> None:  # noqa: C901
        """Parse loaded G-code and draw the UV path on the preview canvas."""
        canvas = self.plane_preview_canvas
        if canvas is None:
            return
        lines = self.plane_gcode_lines
        if not lines:
            canvas.delete("all")
            canvas.create_text(150, 100, text="No file loaded.", fill="gray")
            return

        scale_x = max(1e-6, float(self.plane_scale_x.get()))
        scale_y = max(1e-6, float(self.plane_scale_y.get()))
        mx = -1.0 if self.plane_mirror_x.get() else 1.0
        my = -1.0 if self.plane_mirror_y.get() else 1.0
        rot = self.plane_rotate.get()
        off_u = float(self.plane_offset_u.get())
        off_v = float(self.plane_offset_v.get())
        seg_len = max(0.5, float(self.plane_seg_len.get()))
        metric = True
        abs_mode = True
        cur_u = 0.0
        cur_v = 0.0

        def txuv(
            rx: Optional[float], ry: Optional[float],
        ) -> Tuple[Optional[float], Optional[float]]:
            ff = 25.4 if not metric else 1.0
            u_r = rx * ff * scale_x * mx if rx is not None else None
            v_r = ry * ff * scale_y * my if ry is not None else None
            if u_r is not None and v_r is not None and rot != "0\u00b0":
                if rot == "90\u00b0":
                    u_r, v_r = -v_r, u_r
                elif rot == "180\u00b0":
                    u_r, v_r = -u_r, -v_r
                elif rot == "270\u00b0":
                    u_r, v_r = v_r, -u_r
            return u_r, v_r

        def tuv(
            rx: Optional[float], ry: Optional[float],
        ) -> Tuple[Optional[float], Optional[float]]:
            u_t, v_t = txuv(rx, ry)
            u = u_t + off_u if u_t is not None else None
            v = v_t + off_v if v_t is not None else None
            return u, v

        draw_segs: List[Tuple[float, float, float, float, bool]] = []

        for raw in lines:
            parsed = _parse_gcode_line(raw)
            if not parsed:
                continue
            cmd = parsed["cmd"]
            p = parsed["params"]
            if cmd == "G20":
                metric = False
            elif cmd == "G21":
                metric = True
            elif cmd == "G90":
                abs_mode = True
            elif cmd == "G91":
                abs_mode = False
            elif cmd in ("G0", "G1"):
                u1, v1 = tuv(p.get("X"), p.get("Y"))
                if not abs_mode:
                    if u1 is not None:
                        u1 += cur_u
                    if v1 is not None:
                        v1 += cur_v
                if u1 is None:
                    u1 = cur_u
                if v1 is None:
                    v1 = cur_v
                draw_segs.append((cur_u, cur_v, u1, v1, cmd == "G0"))
                cur_u, cur_v = u1, v1
            elif cmd in ("G2", "G3"):
                ci_t, cj_t = txuv(p.get("I", 0.0), p.get("J", 0.0))
                ci = ci_t if ci_t is not None else 0.0
                cj = cj_t if cj_t is not None else 0.0
                u1, v1 = tuv(p.get("X"), p.get("Y"))
                if not abs_mode:
                    if u1 is not None:
                        u1 += cur_u
                    if v1 is not None:
                        v1 += cur_v
                if u1 is None:
                    u1 = cur_u
                if v1 is None:
                    v1 = cur_v
                cxl = cur_u + ci
                cyl = cur_v + cj
                r0 = math.hypot(cur_u - cxl, cur_v - cyl)
                if r0 < 1e-6:
                    draw_segs.append((cur_u, cur_v, u1, v1, False))
                    cur_u, cur_v = u1, v1
                    continue
                start_a = math.atan2(cur_v - cyl, cur_u - cxl)
                end_a = math.atan2(v1 - cyl, u1 - cxl)
                if abs(u1 - cur_u) < 1e-6 and abs(v1 - cur_v) < 1e-6:
                    delta = -2 * math.pi if cmd == "G2" else 2 * math.pi
                else:
                    delta = end_a - start_a
                    if cmd == "G2":
                        if delta >= 0:
                            delta -= 2 * math.pi
                    else:
                        if delta <= 0:
                            delta += 2 * math.pi
                steps = max(3, int(math.ceil(abs(delta) * r0 / seg_len)))
                prev_u, prev_v = cur_u, cur_v
                for i in range(1, steps + 1):
                    ang = start_a + delta * (i / steps)
                    uu = cxl + r0 * math.cos(ang)
                    vv = cyl + r0 * math.sin(ang)
                    draw_segs.append((prev_u, prev_v, uu, vv, False))
                    prev_u, prev_v = uu, vv
                cur_u, cur_v = u1, v1

        if not draw_segs:
            canvas.delete("all")
            canvas.create_text(
                150, 100, text="No drawable moves found.", fill="gray",
            )
            return

        all_u = [s[0] for s in draw_segs] + [s[2] for s in draw_segs]
        all_v = [s[1] for s in draw_segs] + [s[3] for s in draw_segs]
        min_u, max_u = min(all_u), max(all_u)
        min_v, max_v = min(all_v), max(all_v)
        cw = canvas.winfo_width() or 300
        ch = canvas.winfo_height() or 260
        pad = 14
        w = cw - 2 * pad
        h = ch - 2 * pad
        du = max_u - min_u or 1.0
        dv = max_v - min_v or 1.0
        sc = min(w / du, h / dv)

        def cx_fn(u: float) -> float:
            return pad + (u - min_u) * sc

        def cy_fn(v: float) -> float:
            return pad + h - (v - min_v) * sc  # flip Y

        canvas.delete("all")
        canvas.create_rectangle(
            pad - 1, pad - 1, pad + w + 1, pad + h + 1, outline="#ddd",
        )
        for u0, v0, u1, v1, rapid in draw_segs:
            x0, y0 = cx_fn(u0), cy_fn(v0)
            x1, y1 = cx_fn(u1), cy_fn(v1)
            if rapid:
                canvas.create_line(x0, y0, x1, y1, fill="#c0c0c0", dash=(4, 3))
            else:
                canvas.create_line(x0, y0, x1, y1, fill="#1a5fb4", width=1)
        # mark G-code origin
        ox, oy = cx_fn(off_u), cy_fn(off_v)
        canvas.create_oval(ox - 3, oy - 3, ox + 3, oy + 3, fill="#e01b24", outline="")
        canvas.create_text(
            cw // 2, ch - 4,
            text=f"UV extent  {du:.1f} \u00d7 {dv:.1f} mm",
            fill="#555", font=("TkDefaultFont", 8),
        )

    # ══════════════════════════════════════════════════════════════════════
    # Help dialog
    # ══════════════════════════════════════════════════════════════════════

    def _show_help(self) -> None:
        top = tk.Toplevel()
        top.title("GCODE Tab \u2013 Help")
        top.geometry("620x520")
        txt = scrolledtext.ScrolledText(
            top, wrap=tk.WORD, font=("Consolas", 10),
        )
        txt.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        help_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "..", "data", "GCODE_help.txt",
        )
        try:
            with open(help_path, "r", encoding="utf-8") as f:
                content = f.read()
        except FileNotFoundError as e:
            content = f"Help file not found.\nExpected: {help_path}\n\n{e}"
        txt.insert("1.0", content)
        txt.config(state=tk.DISABLED)

    # ══════════════════════════════════════════════════════════════════════
    # UI construction
    # ══════════════════════════════════════════════════════════════════════

    def _build_ui(self) -> None:
        # ── Plane definition frame ───────────────────────────────────────
        plane_wrap = ttk.LabelFrame(self, text="Plane definition")
        plane_wrap.pack(fill=tk.X, expand=False, padx=4, pady=4)

        ttk.Label(
            plane_wrap,
            text="Define a UV plane from current TCP orientation. "
                 "G2/G3 run in that plane.",
        ).pack(anchor="w", padx=4, pady=(2, 4))

        plane_btns = ttk.Frame(plane_wrap)
        plane_btns.pack(anchor="w", padx=4, pady=(0, 4))
        ttk.Button(
            plane_btns, text="Set plane from current TCP",
            command=self._set_from_current,
        ).pack(side=tk.LEFT)
        ttk.Button(
            plane_btns, text="Plane = world XY",
            command=self._set_world_xy,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            plane_btns, text="Normalize axes",
            command=self._sync_axes,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            plane_btns, text="Send plane frame",
            command=self._send_frame,
        ).pack(side=tk.LEFT, padx=(6, 0))

        # Vector grid
        plane_grid = ttk.Frame(plane_wrap)
        plane_grid.pack(anchor="w", padx=4, pady=(0, 4))

        ttk.Label(plane_grid, text="X").grid(row=0, column=1)
        ttk.Label(plane_grid, text="Y").grid(row=0, column=2)
        ttk.Label(plane_grid, text="Z").grid(row=0, column=3)
        self._vec_row(plane_grid, 1, "Origin",
                      self.plane_origin_x, self.plane_origin_y, self.plane_origin_z)
        self._vec_row(plane_grid, 2, "U axis",
                      self.plane_u_x, self.plane_u_y, self.plane_u_z)
        self._vec_row(plane_grid, 3, "V axis",
                      self.plane_v_x, self.plane_v_y, self.plane_v_z)
        self._vec_row(plane_grid, 4, "N axis",
                      self.plane_n_x, self.plane_n_y, self.plane_n_z)

        # RPY row
        rpy_row = ttk.Frame(plane_wrap)
        rpy_row.pack(anchor="w", padx=4, pady=(0, 4))
        ttk.Label(rpy_row, text="Tool RPY (deg)").pack(side=tk.LEFT)
        ttk.Entry(
            rpy_row, textvariable=self.plane_roll, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(6, 2))
        ttk.Entry(
            rpy_row, textvariable=self.plane_pitch, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=2)
        ttk.Entry(
            rpy_row, textvariable=self.plane_yaw, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=2)

        # ── Sub-notebook: G-code file | Single arc ───────────────────────
        plane_nb = ttk.Notebook(self)
        plane_nb.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self._build_gcode_file_tab(plane_nb)
        self._build_single_arc_tab(plane_nb)

    # ── Reusable grid row ────────────────────────────────────────────────

    @staticmethod
    def _vec_row(
        grid: ttk.Frame, row: int, label: str,
        vx: tk.DoubleVar, vy: tk.DoubleVar, vz: tk.DoubleVar,
    ) -> None:
        ttk.Label(grid, text=label, width=8).grid(row=row, column=0, sticky="w")
        ttk.Entry(grid, textvariable=vx, width=8, justify="right").grid(
            row=row, column=1, padx=2, pady=1,
        )
        ttk.Entry(grid, textvariable=vy, width=8, justify="right").grid(
            row=row, column=2, padx=2, pady=1,
        )
        ttk.Entry(grid, textvariable=vz, width=8, justify="right").grid(
            row=row, column=3, padx=2, pady=1,
        )

    # ── G-code file tab ──────────────────────────────────────────────────

    def _build_gcode_file_tab(self, notebook: ttk.Notebook) -> None:
        tab_gfile = ttk.Frame(notebook)
        notebook.add(tab_gfile, text="G-code file")

        # File row
        file_row = ttk.Frame(tab_gfile)
        file_row.pack(fill=tk.X, padx=4, pady=(6, 2))
        ttk.Label(file_row, text="File:").pack(side=tk.LEFT)
        ttk.Entry(
            file_row, textvariable=self.plane_gcode_path, width=30,
        ).pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)
        ttk.Button(
            file_row, text="Browse\u2026", command=self._load_gcode_file,
        ).pack(side=tk.LEFT)

        ttk.Label(
            tab_gfile, textvariable=self.plane_gcode_status, foreground="gray",
        ).pack(anchor="w", padx=6, pady=(0, 2))

        # Horizontal split: controls (left) | preview canvas (right)
        main_frame = ttk.Frame(tab_gfile)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        left_frame = ttk.Frame(main_frame, width=268)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4))
        left_frame.pack_propagate(False)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Transform controls
        tf = ttk.LabelFrame(left_frame, text="Transform")
        tf.pack(fill=tk.X, padx=0, pady=(0, 4))

        tf_r1 = ttk.Frame(tf)
        tf_r1.pack(anchor="w", padx=4, pady=(4, 1))
        ttk.Label(tf_r1, text="Scale X:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(
            tf_r1, textvariable=self.plane_scale_x, width=6, justify="right",
        ).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(tf_r1, text="Scale Y:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(
            tf_r1, textvariable=self.plane_scale_y, width=6, justify="right",
        ).pack(side=tk.LEFT, padx=(2, 0))

        tf_r2 = ttk.Frame(tf)
        tf_r2.pack(anchor="w", padx=4, pady=1)
        ttk.Label(tf_r2, text="Origin U:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(
            tf_r2, textvariable=self.plane_offset_u, width=6, justify="right",
        ).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(tf_r2, text="Origin V:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(
            tf_r2, textvariable=self.plane_offset_v, width=6, justify="right",
        ).pack(side=tk.LEFT, padx=(2, 0))

        tf_r3 = ttk.Frame(tf)
        tf_r3.pack(anchor="w", padx=4, pady=1)
        ttk.Checkbutton(
            tf_r3, text="Mirror X", variable=self.plane_mirror_x,
        ).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Checkbutton(
            tf_r3, text="Mirror Y", variable=self.plane_mirror_y,
        ).pack(side=tk.LEFT)

        tf_r4 = ttk.Frame(tf)
        tf_r4.pack(anchor="w", padx=4, pady=1)
        ttk.Label(tf_r4, text="Rotate:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Combobox(
            tf_r4, textvariable=self.plane_rotate,
            values=["0\u00b0", "90\u00b0", "180\u00b0", "270\u00b0"],
            width=6, state="readonly",
        ).pack(side=tk.LEFT, padx=(2, 0))

        tf_r5 = ttk.Frame(tf)
        tf_r5.pack(anchor="w", padx=4, pady=(1, 4))
        ttk.Label(tf_r5, text="SegLen:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(
            tf_r5, textvariable=self.plane_seg_len, width=6, justify="right",
        ).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Label(tf_r5, text="Feed:", width=6, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(
            tf_r5, textvariable=self.plane_feed, width=6, justify="right",
        ).pack(side=tk.LEFT, padx=(2, 0))

        # Run controls
        run_row = ttk.Frame(left_frame)
        run_row.pack(anchor="w", padx=0, pady=(2, 2))
        ttk.Checkbutton(
            run_row, text="Dry run", variable=self.plane_dry_run,
        ).pack(side=tk.LEFT)
        ttk.Button(
            run_row, text="Run", command=self._gcode_run,
        ).pack(side=tk.LEFT, padx=(8, 4))
        ttk.Button(
            run_row, text="Stop", command=self._gcode_stop,
        ).pack(side=tk.LEFT)

        ttk.Label(
            left_frame, textvariable=self.plane_gcode_progress, foreground="gray",
        ).pack(anchor="w", padx=0, pady=(0, 4))

        btn_row = ttk.Frame(left_frame)
        btn_row.pack(anchor="w", padx=0, pady=2)
        ttk.Button(
            btn_row, text="Preview", command=self._draw_preview,
        ).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(
            btn_row, text="Help", command=self._show_help,
        ).pack(side=tk.LEFT)

        # Preview canvas
        pv_canvas = tk.Canvas(
            right_frame, bg="white", cursor="crosshair", bd=1, relief=tk.SUNKEN,
        )
        pv_canvas.pack(fill=tk.BOTH, expand=True)
        self.plane_preview_canvas = pv_canvas

    # ── Single arc tab ───────────────────────────────────────────────────

    def _build_single_arc_tab(self, notebook: ttk.Notebook) -> None:
        tab_arc = ttk.Frame(notebook)
        notebook.add(tab_arc, text="Single arc")

        arc_row0 = ttk.Frame(tab_arc)
        arc_row0.pack(anchor="w", padx=4, pady=(6, 0))
        ttk.Checkbutton(
            arc_row0, text="Absolute UV (G90)", variable=self.plane_abs_mode,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            arc_row0, text="G2 CW", variable=self.plane_dir, value="G2",
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Radiobutton(
            arc_row0, text="G3 CCW", variable=self.plane_dir, value="G3",
        ).pack(side=tk.LEFT, padx=(4, 0))

        arc_row1 = ttk.Frame(tab_arc)
        arc_row1.pack(anchor="w", padx=4, pady=2)
        ttk.Label(arc_row1, text="End U").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row1, textvariable=self.plane_end_u, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(arc_row1, text="End V").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row1, textvariable=self.plane_end_v, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 0))

        arc_row2 = ttk.Frame(tab_arc)
        arc_row2.pack(anchor="w", padx=4, pady=2)
        ttk.Label(arc_row2, text="Center I").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row2, textvariable=self.plane_center_i, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(arc_row2, text="Center J").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row2, textvariable=self.plane_center_j, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 0))

        arc_row3 = ttk.Frame(tab_arc)
        arc_row3.pack(anchor="w", padx=4, pady=2)
        ttk.Label(arc_row3, text="W offset").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row3, textvariable=self.plane_w, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(arc_row3, text="Feed").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row3, textvariable=self.plane_feed, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 10))
        ttk.Label(arc_row3, text="SegLen").pack(side=tk.LEFT)
        ttk.Entry(
            arc_row3, textvariable=self.plane_seg_len, width=8, justify="right",
        ).pack(side=tk.LEFT, padx=(4, 0))

        arc_row4 = ttk.Frame(tab_arc)
        arc_row4.pack(anchor="w", padx=4, pady=(4, 2))
        ttk.Button(
            arc_row4, text="Execute arc", command=self._arc_send,
        ).pack(side=tk.LEFT)
        ttk.Button(
            arc_row4, text="Queue arc", command=self._arc_queue,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            arc_row4, text="Queue native G2/G3", command=self._arc_queue_native,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            arc_row4, text="Queue 3DP G17", command=self._queue_3dp_example_g17,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            arc_row4, text="Queue 3DP G18", command=self._queue_3dp_example_g18,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            arc_row4, text="Queue 3DP G19", command=self._queue_3dp_example_g19,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            arc_row4, text="Queue ALL 3DP", command=self._queue_3dp_examples_all,
        ).pack(side=tk.LEFT, padx=(6, 0))
