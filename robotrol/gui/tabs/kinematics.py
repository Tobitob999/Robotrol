"""Kinematics tab -- TCP/World coordinate IK controls + DH table.

Extracted from monolith lines 1057-1076 + tcp_world_kinematics_frame.py.
The backend IK solver lives in ``robotrol.kinematics.ik``, DH model loading in
``robotrol.kinematics.dh_model``, and FK in ``robotrol.kinematics.fk``.
This module contains **only** the GUI layer.
"""

from __future__ import annotations

import json
import logging
import math
import os
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Joint axis naming convention (Moveo / 6-DOF)
AXES: List[str] = ["A", "X", "Y", "B", "Z", "C"]


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _fmt(val: object, digits: int = 4) -> str:
    try:
        return f"{float(val):.{digits}f}"
    except (TypeError, ValueError):
        return "0." + "0" * digits


def _wrap180(a: float) -> float:
    return ((a + 180.0) % 360.0) - 180.0


def _rpy_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[List[float]]:
    """ZYX rotation matrix from Euler angles (degrees)."""
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


# ---------------------------------------------------------------------------
#  TcpKinematicsPanel -- the main body of the Kinematics tab
# ---------------------------------------------------------------------------

class TcpKinematicsPanel(ttk.Frame):
    """TCP pose input mask, IK solve, DH parameter table, and G-code preview.

    This is the refactored equivalent of the monolith's ``TcpKinematicsFrame``.
    Heavy math is delegated to ``robotrol.kinematics.*`` modules.
    """

    def __init__(self, master: ttk.Widget, app: Any, client: Any):
        super().__init__(master)
        self.app = app
        self.client = client

        # Geometry from app (must contain L1..L4)
        self.geom: Dict[str, float] = getattr(app, "GEOM_DH", {
            "L1": 240.0, "L2": 230.0, "L3": 250.0, "L4": 180.0,
        })
        self._mirror_x = False
        self._last_tcp_joints: Optional[List[float]] = None

        # Scale orientation error (rad) to mm for DLS balancing
        self.rot_weight_mm = 100.0

        # --- IK solver ---
        try:
            from robotrol.kinematics.ik import IK6
            self.ik = IK6(self.geom)
        except ImportError:
            logger.warning("robotrol.kinematics.ik not available -- IK disabled")
            self.ik = None

        # --- DH model helpers ---
        try:
            from robotrol.kinematics.dh_model import DHModel
            self._dh_model_cls = DHModel
        except ImportError:
            self._dh_model_cls = None

        # --- tkinter variables ---
        self.D_ret = tk.DoubleVar(value=20.0)
        self.D_work = tk.DoubleVar(value=30.0)
        self.feed = tk.DoubleVar(value=3000.0)
        self.pitch_override = tk.DoubleVar(value=0.0)
        self.roll_override = tk.DoubleVar(value=0.0)
        self.anchor_mode = tk.StringVar(value="retreat")
        self.invert_u = tk.BooleanVar(value=False)

        # Gripper
        self.use_gripper = tk.BooleanVar(value=False)
        self.grip_s = tk.IntVar(value=1000)
        self.grip_pause_before_ms = tk.IntVar(value=0)
        self.grip_pause_after_ms = tk.IntVar(value=0)

        # TCP pose inputs
        self.tcp_x = tk.StringVar(value="0.00")
        self.tcp_y = tk.StringVar(value="0.00")
        self.tcp_z = tk.StringVar(value="0.00")
        self.tcp_roll = tk.StringVar(value="0.00")
        self.tcp_pitch = tk.StringVar(value="0.00")
        self.tcp_yaw = tk.StringVar(value="0.00")

        self.dh_vars: List[Tuple[tk.StringVar, tk.StringVar, tk.StringVar, tk.StringVar]] = []

        self._build_ui()

    # ==================================================================
    #  DH model interaction
    # ==================================================================

    def _get_dh_defaults(self) -> List[Tuple[float, float, float, float]]:
        """Return DH rows (alpha, a, d, theta_off) in deg/mm."""
        try:
            from robotrol.kinematics.dh_model import DHModel
            # Try loading a default model to get rows
            model = DHModel()
            rows = model.dh_rows_mm_deg
            if len(rows) == 6:
                return [
                    (r["alpha_deg"], r["a_mm"], r["d_mm"], r["theta_offset_deg"])
                    for r in rows
                ]
        except (ImportError, AttributeError, KeyError):
            pass

        # Fallback from geometry
        L1 = float(self.geom.get("L1", 240.0))
        L2 = float(self.geom.get("L2", 230.0))
        L3 = float(self.geom.get("L3", 250.0))
        L4 = float(self.geom.get("L4", 180.0))
        return [
            (90.0,  0.0, L1, 0.0),
            (0.0,   L2,  0.0, 90.0),
            (-90.0, 0.0, 0.0, -90.0),
            (90.0,  0.0, L3, 0.0),
            (-90.0, 0.0, 0.0, 0.0),
            (0.0,   0.0, L4, 0.0),
        ]

    def _read_dh_table(self) -> List[Dict[str, Any]]:
        """Read DH table from UI entries (values in deg/mm)."""
        table: List[Dict[str, Any]] = []
        for i, (v_alpha, v_a, v_d, v_t) in enumerate(self.dh_vars):
            def _f(var: tk.StringVar) -> float:
                try:
                    return float(var.get().replace(",", "."))
                except (ValueError, tk.TclError):
                    return 0.0

            axis = AXES[i] if i < len(AXES) else f"j{i+1}"
            table.append({
                "axis": axis,
                "alpha": _f(v_alpha),
                "a": _f(v_a),
                "d": _f(v_d),
                "theta_offset": _f(v_t),
            })
        return table

    def refresh_dh_table(self) -> None:
        """Refresh DH table entries from the current model / geometry."""
        if not self.dh_vars:
            return
        try:
            if hasattr(self.app, "GEOM_DH"):
                self.geom = self.app.GEOM_DH
        except AttributeError:
            pass
        dh_defaults = self._get_dh_defaults()
        for i, (alpha, a, d, theta_off) in enumerate(dh_defaults):
            if i >= len(self.dh_vars):
                break
            v_alpha, v_a, v_d, v_t = self.dh_vars[i]
            v_alpha.set(f"{alpha:.3f}")
            v_a.set(f"{a:.3f}")
            v_d.set(f"{d:.3f}")
            v_t.set(f"{theta_off:.3f}")

    # ==================================================================
    #  Joint helpers
    # ==================================================================

    def _get_current_joint_list(self) -> List[float]:
        """Current joint values in AXES order from the app."""
        src = getattr(self.app, "_mpos", None) or getattr(self.app, "axis_positions", {}) or {}
        joints: List[float] = []
        for ax in AXES:
            try:
                joints.append(float(src.get(ax, 0.0)))
            except (TypeError, ValueError):
                joints.append(0.0)
        return joints

    def _limits(self) -> Dict[str, Tuple[float, float]]:
        """Gather joint limits from app / DH model."""
        lims: Dict[str, Tuple[float, float]] = {}
        exec_limits = getattr(self.app, "axis_limits", {}) or {}
        use_exec = bool(
            getattr(self.app, "endstop_limits_enabled", False) and exec_limits
        )
        if use_exec:
            exec_eff = getattr(self.app, "_effective_axis_limits", None)
            if callable(exec_eff):
                for ax in AXES:
                    try:
                        lims[ax] = exec_eff(ax)
                    except (TypeError, ValueError, KeyError):
                        pass
            else:
                for ax in AXES:
                    if ax in exec_limits:
                        try:
                            lo, hi = exec_limits[ax]
                            lims[ax] = (float(lo), float(hi))
                        except (TypeError, ValueError):
                            pass

        for ax in AXES:
            if ax not in lims:
                lims[ax] = exec_limits.get(ax, (-180.0, 180.0))
        return lims

    def _check_limits(
        self, joints: Dict[str, float], lims: Dict[str, Tuple[float, float]]
    ) -> bool:
        for ax in AXES:
            lo, hi = lims.get(ax, (-999.0, 999.0))
            if not (lo <= joints.get(ax, 0.0) <= hi):
                return False
        return True

    # ==================================================================
    #  Pose helpers
    # ==================================================================

    def pose_to_mask_only(self) -> bool:
        """Copy current TCP pose from app into the input mask."""
        tcp_getter = getattr(self.app, "get_current_tcp_mm", None)
        if not callable(tcp_getter):
            return False
        try:
            tcp = tcp_getter()
        except (AttributeError, TypeError):
            tcp = None
        if not tcp:
            return False

        self.tcp_x.set(_fmt(tcp.get("X_mm", 0.0)))
        self.tcp_y.set(_fmt(tcp.get("Y_mm", 0.0)))
        self.tcp_z.set(_fmt(tcp.get("Z_mm", 0.0)))
        self.tcp_roll.set(_fmt(tcp.get("Roll_deg", 0.0)))
        self.tcp_pitch.set(_fmt(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0))))
        self.tcp_yaw.set(_fmt(tcp.get("Yaw_deg", 0.0)))
        return True

    def set_mask_from_current_pose(self) -> None:
        """Alias used by gamepad proxies."""
        self.pose_to_mask_only()

    # ==================================================================
    #  IK solve helpers
    # ==================================================================

    @staticmethod
    def _dh_transform(
        theta_deg: float, alpha_deg: float, a: float, d: float
    ) -> List[List[float]]:
        th = math.radians(theta_deg)
        al = math.radians(alpha_deg)
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        return [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
        out = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
        return out

    def _task_from_joints(self, joints: List[float]) -> Tuple[List[float], List[List[float]]]:
        """FK from joint angles using current DH table UI values."""
        dh = self._read_dh_table()
        T: List[List[float]] = [
            [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1],
        ]
        for i, row in enumerate(dh):
            if i >= len(joints):
                break
            th = joints[i] + row["theta_offset"]
            Ti = self._dh_transform(th, row["alpha"], row["a"], row["d"])
            T = self._mat_mul(T, Ti)
        x, y, z = T[0][3], T[1][3], T[2][3]
        if self._mirror_x:
            x = -x
        R = [T[r][:3] for r in range(3)]
        return [x, y, z], R

    # ==================================================================
    #  Generate (IK solve for retreat -> target -> retreat sequence)
    # ==================================================================

    def _run(self) -> None:
        """Generate G-code preview for the RET -> TAR -> RET sequence."""
        if self.ik is None:
            messagebox.showwarning("IK", "IK solver not available.")
            return

        try:
            x = float(self.tcp_x.get())
            y = float(self.tcp_y.get())
            z = float(self.tcp_z.get())
            pitch = float(self.tcp_pitch.get()) + self.pitch_override.get()
            roll = float(self.tcp_roll.get()) + self.roll_override.get()
        except (ValueError, tk.TclError) as exc:
            messagebox.showwarning("Input", f"Invalid pose value: {exc}")
            return

        d_ret = self.D_ret.get()
        d_work = self.D_work.get()
        feed = self.feed.get()

        # Compute tool axis direction
        yaw = float(self.tcp_yaw.get()) if self.tcp_yaw.get() else 0.0
        R = _rpy_to_R(roll, pitch, yaw)
        u = [R[0][2], R[1][2], R[2][2]]
        norm = math.sqrt(sum(c * c for c in u))
        if norm > 1e-9:
            u = [c / norm for c in u]
        else:
            u = [0.0, 0.0, 1.0]

        if self.invert_u.get():
            u = [-c for c in u]

        # Target, retreat, zero positions
        target = [x, y, z]
        retreat = [x - u[0] * d_ret, y - u[1] * d_ret, z - u[2] * d_ret]

        # IK solve for each waypoint
        lims = self._limits()
        lines: List[str] = []
        bad_mask: List[bool] = []

        for label, pos in [("RET", retreat), ("TAR", target), ("RET", retreat)]:
            try:
                result = self.ik.solve_xyz(
                    pos[0], pos[1], pos[2],
                    pitch_deg=pitch, roll_deg=roll,
                )
                ok = self._check_limits(result, lims)
                gcode = "G90 G1 " + " ".join(
                    f"{ax}{result[ax]:.3f}" for ax in AXES
                ) + f" F{feed:.0f}"
                lines.append(f"; {label}")
                lines.append(gcode)
                bad_mask.append(False)
                bad_mask.append(not ok)
            except (ValueError, ZeroDivisionError) as exc:
                lines.append(f"; {label} -- IK FAILED: {exc}")
                bad_mask.append(True)

        # Gripper commands
        if self.use_gripper.get():
            pause_before = self.grip_pause_before_ms.get()
            pause_after = self.grip_pause_after_ms.get()
            grip_val = self.grip_s.get()
            grip_lines = []
            if pause_before > 0:
                grip_lines.append(f"G4 P{pause_before / 1000.0:.3f}")
            grip_lines.append(f"M3 S{grip_val}")
            if pause_after > 0:
                grip_lines.append(f"G4 P{pause_after / 1000.0:.3f}")
            # Insert after target (index 3 = after TAR gcode)
            for gl in reversed(grip_lines):
                lines.insert(3, gl)
                bad_mask.insert(3, False)

        self._emit(lines, bad_mask)
        self._last_preview_lines = lines

    def _emit(self, lines: List[str], bad_mask: List[bool]) -> None:
        """Write G-code preview into the text widget with limit highlighting."""
        self.txt.configure(state="normal")
        self.txt.delete("1.0", tk.END)

        lims = self._limits()
        for i, ln in enumerate(lines):
            if not ln.strip():
                self.txt.insert(tk.END, "\n", "plain")
                continue

            is_bad = bad_mask[i] if i < len(bad_mask) else False
            parts = ln.split(" ")
            for part in parts:
                tag = "plain"
                for ax in AXES:
                    if part.startswith(ax) and len(part) > 1:
                        try:
                            val = float(part[len(ax):])
                            lo, hi = lims.get(ax, (-999, 999))
                            if lo <= val <= hi:
                                tag = "ok"
                            else:
                                tag = "bad_val"
                        except ValueError:
                            pass
                        break
                if is_bad and tag == "plain":
                    tag = "bad_val"
                self.txt.insert(tk.END, part + " ", tag)
            self.txt.insert(tk.END, "\n")

        self.txt.configure(state="disabled")

    def _execute_direct(self) -> None:
        """Send the last generated preview directly to the controller."""
        lines = getattr(self, "_last_preview_lines", None)
        if not lines:
            messagebox.showinfo("Execute", "No preview generated yet. Click 'Generate' first.")
            return
        for ln in lines:
            stripped = ln.strip()
            if stripped and not stripped.startswith(";"):
                try:
                    self.client.send_line(stripped)
                except (AttributeError, OSError) as exc:
                    logger.error("Send failed: %s", exc)
                    break

    def _send_preview_to_cli(self) -> None:
        """Log preview to the app's CLI/log area."""
        lines = getattr(self, "_last_preview_lines", None)
        if not lines:
            return
        app_log = getattr(self.app, "log", None)
        if callable(app_log):
            for ln in lines:
                app_log(ln)

    def _send_preview_to_queue(self) -> None:
        """Push preview G-code into the app's command queue (if available)."""
        lines = getattr(self, "_last_preview_lines", None)
        if not lines:
            return
        queue_add = getattr(self.app, "add_to_queue", None)
        if callable(queue_add):
            for ln in lines:
                stripped = ln.strip()
                if stripped and not stripped.startswith(";"):
                    queue_add(stripped)

    def _save_dh_to_json(self) -> None:
        """Save DH parameters from UI to JSON model file."""
        path = os.path.join(os.path.dirname(__file__), "..", "..", "model", "dh.json")
        try:
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as fh:
                    model = json.load(fh)
            else:
                model = {}
        except (json.JSONDecodeError, OSError):
            model = {}

        model["convention"] = "DH"
        model["units"] = {"length": "m", "angle": "rad"}
        model["joint_order"] = list(AXES)

        dh = self._read_dh_table()
        joints_out: List[Dict[str, Any]] = []
        for i, row in enumerate(dh):
            joints_out.append({
                "name": f"joint{i+1}_{row['axis']}",
                "axis": row["axis"],
                "a": float(row["a"]) / 1000.0,
                "alpha": math.radians(float(row["alpha"])),
                "d": float(row["d"]) / 1000.0,
                "theta_offset": math.radians(float(row["theta_offset"])),
                "q_min": -math.pi,
                "q_max": math.pi,
                "source_note": "Updated via kinematics tab UI.",
            })
        model["joints"] = joints_out

        # Try saving through app profile API first
        save_fn = getattr(self.app, "save_dh_model_to_profile", None)
        if callable(save_fn):
            ok, msg = save_fn(model)
            logger.info(msg)
        else:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, "w", encoding="utf-8") as fh:
                    json.dump(model, fh, indent=2, ensure_ascii=True)
                logger.info("DH parameters saved to %s", path)
            except OSError as exc:
                logger.error("Failed to save DH: %s", exc)

    def _save_settings_to_profile(self) -> None:
        """Save kinematics tab settings to app profile."""
        save_all = getattr(self.app, "save_all_kinematics_settings", None)
        if callable(save_all):
            save_all()
            return

        save_section = getattr(self.app, "set_profile_section", None)
        if callable(save_section):
            payload = {"kinematics": self.get_settings()}
            save_section("kinematics_settings", payload, save=True)

    def _load_settings_from_profile(self) -> None:
        """Load kinematics tab settings from app profile."""
        load_all = getattr(self.app, "load_all_kinematics_settings", None)
        if callable(load_all):
            load_all()
            return

        get_section = getattr(self.app, "get_profile_section", None)
        if callable(get_section):
            data = get_section("kinematics_settings", default=None)
            if isinstance(data, dict):
                kin = data.get("kinematics", data)
                self.apply_settings(kin)

    def get_settings(self) -> Dict[str, Any]:
        """Return a dict of all saveable settings."""
        return {
            "D_ret": self.D_ret.get(),
            "D_work": self.D_work.get(),
            "feed": self.feed.get(),
            "pitch_override": self.pitch_override.get(),
            "roll_override": self.roll_override.get(),
            "invert_u": self.invert_u.get(),
            "anchor_mode": self.anchor_mode.get(),
            "use_gripper": self.use_gripper.get(),
            "grip_s": self.grip_s.get(),
            "grip_pause_before_ms": self.grip_pause_before_ms.get(),
            "grip_pause_after_ms": self.grip_pause_after_ms.get(),
        }

    def apply_settings(self, data: Dict[str, Any]) -> None:
        """Restore settings from a dict."""
        if not isinstance(data, dict):
            return

        def _set(var: Any, key: str, conv: type = float) -> None:
            if key in data:
                try:
                    var.set(conv(data[key]))
                except (TypeError, ValueError, tk.TclError):
                    pass

        _set(self.D_ret, "D_ret")
        _set(self.D_work, "D_work")
        _set(self.feed, "feed")
        _set(self.pitch_override, "pitch_override")
        _set(self.roll_override, "roll_override")
        _set(self.invert_u, "invert_u", bool)
        if "anchor_mode" in data:
            self.anchor_mode.set(str(data["anchor_mode"]))
        _set(self.use_gripper, "use_gripper", bool)
        _set(self.grip_s, "grip_s", int)
        _set(self.grip_pause_before_ms, "grip_pause_before_ms", int)
        _set(self.grip_pause_after_ms, "grip_pause_after_ms", int)

    # ==================================================================
    #  UI BUILD
    # ==================================================================

    def _build_ui(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        # --- Top: 4-column layout ---
        top = ttk.Frame(self)
        top.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        for c in range(4):
            top.columnconfigure(c, weight=1)

        # Column 1: TCP Pose Input
        self._build_pose_column(top)
        # Column 2: Parameters
        self._build_params_column(top)
        # Column 3: Gripper / Sequence
        self._build_gripper_column(top)
        # Column 4: DH Parameters
        self._build_dh_column(top)

        # --- Buttons ---
        btn_frame = ttk.Frame(top)
        btn_frame.grid(row=1, column=0, columnspan=4, pady=6)
        ttk.Button(btn_frame, text="Generate", command=self._run).pack(
            side=tk.LEFT, padx=5,
        )
        ttk.Button(btn_frame, text="Save DH", command=self._save_dh_to_json).pack(
            side=tk.LEFT, padx=5,
        )
        ttk.Button(btn_frame, text="Send to CLI", command=self._send_preview_to_cli).pack(
            side=tk.LEFT, padx=5,
        )
        ttk.Button(btn_frame, text="Execute", command=self._execute_direct).pack(
            side=tk.LEFT, padx=5,
        )
        ttk.Button(btn_frame, text="Send to Queue", command=self._send_preview_to_queue).pack(
            side=tk.LEFT, padx=5,
        )
        ttk.Separator(btn_frame, orient="vertical").pack(
            side=tk.LEFT, fill="y", padx=8, pady=2,
        )
        ttk.Button(btn_frame, text="Save Settings", command=self._save_settings_to_profile).pack(
            side=tk.LEFT, padx=5,
        )
        ttk.Button(btn_frame, text="Load Settings", command=self._load_settings_from_profile).pack(
            side=tk.LEFT, padx=5,
        )

        # --- Preview ---
        preview = ttk.LabelFrame(self, text="G-code preview")
        preview.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        preview.columnconfigure(0, weight=1)
        preview.rowconfigure(0, weight=1)

        self.txt = tk.Text(preview, height=4, wrap="none")
        self.txt.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(preview, orient="vertical", command=self.txt.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.txt.configure(yscrollcommand=sb.set)
        self.txt.tag_configure("ok", background="#e8ffe8")
        self.txt.tag_configure("bad_val", background="#ffd0d0")
        self.txt.tag_configure("plain", background="white")

    def _build_pose_column(self, parent: ttk.Widget) -> None:
        col = ttk.LabelFrame(parent, text="TCP Pose Input (mm / deg)")
        col.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        fields = [
            ("X [mm]", self.tcp_x),
            ("Y [mm]", self.tcp_y),
            ("Z [mm]", self.tcp_z),
            ("Roll [deg]", self.tcp_roll),
            ("Pitch [deg]", self.tcp_pitch),
            ("Yaw [deg]", self.tcp_yaw),
        ]
        for i, (label, var) in enumerate(fields):
            ttk.Label(col, text=label).grid(row=i, column=0, sticky="e", padx=4, pady=1)
            ttk.Entry(col, textvariable=var, width=10, justify="right").grid(
                row=i, column=1, sticky="w", padx=4, pady=1,
            )

        ttk.Button(
            col, text="Pose -> Maske", command=self.pose_to_mask_only,
        ).grid(row=len(fields), column=0, columnspan=2, pady=6)

    def _build_params_column(self, parent: ttk.Widget) -> None:
        col = ttk.LabelFrame(parent, text="Parameters")
        col.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

        params = [
            ("Retreat D_ret [mm]", self.D_ret),
            ("Work D_work [mm]", self.D_work),
            ("Feed [mm/min]", self.feed),
            ("Pitch override [deg]", self.pitch_override),
            ("Roll override [deg]", self.roll_override),
        ]
        for i, (label, var) in enumerate(params):
            ttk.Label(col, text=label).grid(row=i, column=0, sticky="e", padx=4, pady=2)
            ttk.Entry(col, textvariable=var, width=12, justify="right").grid(
                row=i, column=1, sticky="w", padx=4, pady=2,
            )

        ttk.Checkbutton(
            col, text="Invert tool axis", variable=self.invert_u,
        ).grid(row=len(params), column=0, columnspan=2, sticky="w", padx=4, pady=4)

        # Anchor selection
        anchor = ttk.LabelFrame(col, text="Anchor (mask is ...)")
        anchor.grid(
            row=len(params) + 1, column=0, columnspan=2,
            sticky="ew", padx=2, pady=(4, 2),
        )
        for text, value in [("Target", "target"), ("Zero", "zero"), ("Retreat", "retreat")]:
            ttk.Radiobutton(
                anchor, text=text, value=value, variable=self.anchor_mode,
            ).pack(side=tk.LEFT, padx=3)

    def _build_gripper_column(self, parent: ttk.Widget) -> None:
        col = ttk.LabelFrame(parent, text="Gripper / Sequence")
        col.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)

        ttk.Checkbutton(
            col, text="Use gripper", variable=self.use_gripper,
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=4, pady=2)

        grip_fields = [
            ("S-value", self.grip_s),
            ("Pause before [ms]", self.grip_pause_before_ms),
            ("Pause after [ms]", self.grip_pause_after_ms),
        ]
        for i, (label, var) in enumerate(grip_fields, start=1):
            ttk.Label(col, text=label).grid(row=i, column=0, sticky="e", padx=4, pady=2)
            ttk.Entry(col, textvariable=var, width=10, justify="right").grid(
                row=i, column=1, sticky="w", padx=4, pady=2,
            )

    def _build_dh_column(self, parent: ttk.Widget) -> None:
        col = ttk.LabelFrame(parent, text="DH Parameters (deg / mm)")
        col.grid(row=0, column=3, sticky="nsew", padx=4, pady=4)

        headers = ["Joint", "Alpha", "A", "D", "ThetaOff"]
        for j, lbl in enumerate(headers):
            ttk.Label(col, text=lbl).grid(row=0, column=j, padx=2, pady=1)

        dh_defaults = self._get_dh_defaults()
        self.dh_vars = []
        for i, (alpha, a, d, theta_off) in enumerate(dh_defaults, start=1):
            ttk.Label(col, text=f"j{i}").grid(row=i, column=0, padx=2, pady=1)
            v_alpha = tk.StringVar(value=f"{alpha:.3f}")
            v_a = tk.StringVar(value=f"{a:.3f}")
            v_d = tk.StringVar(value=f"{d:.3f}")
            v_t = tk.StringVar(value=f"{theta_off:.3f}")
            ttk.Entry(col, textvariable=v_alpha, width=7, justify="right").grid(
                row=i, column=1, padx=1, pady=1,
            )
            ttk.Entry(col, textvariable=v_a, width=7, justify="right").grid(
                row=i, column=2, padx=1, pady=1,
            )
            ttk.Entry(col, textvariable=v_d, width=7, justify="right").grid(
                row=i, column=3, padx=1, pady=1,
            )
            ttk.Entry(col, textvariable=v_t, width=7, justify="right").grid(
                row=i, column=4, padx=1, pady=1,
            )
            self.dh_vars.append((v_alpha, v_a, v_d, v_t))


# ---------------------------------------------------------------------------
#  KinematicsTab -- the outer tab container (matches TcpWorldKinematicsTabs)
# ---------------------------------------------------------------------------

class KinematicsTab(ttk.Frame):
    """Container tab that embeds TcpKinematicsPanel and exposes proxy methods
    for gamepad/app integration.

    Matches the monolith's ``TcpWorldKinematicsTabs`` API.
    """

    def __init__(self, parent: ttk.Widget, app: Any):
        super().__init__(parent)
        self.app = app
        self.client = getattr(app, "client", None)
        self._build_ui()

    def _build_ui(self) -> None:
        self.world = TcpKinematicsPanel(self, self.app, self.client)
        self.world.pack(fill=tk.BOTH, expand=True)

    # --- Gamepad-compatible proxy methods ---

    def preview(self) -> None:
        if hasattr(self.world, "_run"):
            self.world._run()

    def preview_sequence(self) -> None:
        self.preview()

    def execute_sequence(self) -> None:
        if hasattr(self.world, "_execute_direct"):
            self.world._execute_direct()

    def queue_sequence(self) -> None:
        if hasattr(self.world, "_send_preview_to_queue"):
            self.world._send_preview_to_queue()

    def set_tcp_as_reference(self) -> Optional[bool]:
        if hasattr(self.world, "pose_to_mask_only"):
            return self.world.pose_to_mask_only()
        return None

    def use_current_tcp_as_ref(self) -> Optional[bool]:
        return self.set_tcp_as_reference()

    def solve_and_execute_sequence(self) -> None:
        self.set_tcp_as_reference()
        if hasattr(self.world, "_run"):
            self.world._run()
        if hasattr(self.world, "_execute_direct"):
            self.world._execute_direct()

    def pose_to_mask_and_generate(self) -> None:
        self.set_tcp_as_reference()
        if hasattr(self.world, "_run"):
            self.world._run()

    def solve_and_run_tcp_sequence(self) -> None:
        self.solve_and_execute_sequence()

    def refresh_dh_table(self) -> None:
        if hasattr(self.world, "refresh_dh_table"):
            self.world.refresh_dh_table()

    def get_settings(self) -> Dict[str, Any]:
        return self.world.get_settings()

    def apply_settings(self, data: Dict[str, Any]) -> None:
        self.world.apply_settings(data)
