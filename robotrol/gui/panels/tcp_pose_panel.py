"""TCP Pose Panel — live TCP position display via forward kinematics.

Extracted from tcp_pose_module_v3.py (class TcpPosePanel, lines 242-431).
Displays X/Y/Z position in mm and Roll/Pitch/Yaw orientation in degrees,
updated periodically from current machine positions via FK computation.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, Optional

from robotrol.config.constants import AXES, MOTION_EPS
from robotrol.kinematics.dh_model import DHModel
from robotrol.kinematics.fk import fk6_forward_mm


def _pose_diff(a: Dict[str, float], b: Dict[str, float]) -> float:
    """Small delta score used to suppress callback noise."""
    keys = ("X_mm", "Y_mm", "Z_mm", "Roll_deg", "Pitch_deg", "Yaw_deg")
    total = 0.0
    for k in keys:
        total += (float(a.get(k, 0.0)) - float(b.get(k, 0.0))) ** 2
    return total


class TcpPosePanel(ttk.Frame):
    """Live TCP position panel computing forward kinematics.

    Shows X/Y/Z in mm and Roll/Pitch/Yaw in degrees, refreshed
    every *poll_interval_ms* milliseconds.

    Orientation convention:
        Roll  = rotation around tool-X
        Pitch = rotation around tool-Y
        Yaw   = rotation around tool-Z (ZYX Euler from world view)

    Parameters
    ----------
    parent : tk.Widget
        Parent widget.
    app : object
        Application root.  Expected attributes:
        - ``app.dh``   — :class:`~robotrol.kinematics.dh_model.DHModel`
        - ``app.mpos`` — dict[str, float] with current machine positions
                         keyed by axis letter (A, X, Y, B, Z, C).
    poll_interval_ms : int
        Refresh interval in milliseconds (default 200).
    title : str
        LabelFrame title text.
    """

    def __init__(
        self,
        parent: tk.Widget,
        app: object,
        *,
        poll_interval_ms: int = 200,
        title: str = "TCP Pose",
    ) -> None:
        super().__init__(parent)
        self.app = app
        self._poll_interval = poll_interval_ms
        self._running = False

        # Internal position buffer
        self._axis_positions: Dict[str, float] = {ax: 0.0 for ax in AXES}

        # Current TCP pose cache (mm / deg)
        self.current_tcp_pose: Dict[str, float] = {
            "X_mm": 0.0,
            "Y_mm": 0.0,
            "Z_mm": 0.0,
            "Roll_deg": 0.0,
            "Pitch_deg": 0.0,
            "Yaw_deg": 0.0,
            "Tilt_deg": 0.0,
            "B_deg": 0.0,
        }

        # Optional callback on pose change
        self._pose_changed_cb: Optional[Callable[[Dict[str, float]], None]] = None

        # ── UI ────────────────────────────────────────────────────────
        lf = ttk.LabelFrame(self, text=title)
        lf.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        self._tcp_var = tk.StringVar(
            value=(
                "TCP \n"
                "X=0.00 mm\n"
                "Y=0.00 mm\n"
                "Z=0.00 mm\n"
                "Roll=0.00\n"
                "Pitch=0.00\n"
                "Yaw=0.00"
            )
        )
        ttk.Label(
            lf,
            textvariable=self._tcp_var,
            font=("Consolas", 9, "bold"),
            justify="left",
        ).pack(anchor="w", padx=8, pady=6)

        # Tilt display (legacy convenience)
        self._tilt_var = tk.StringVar(value="Tilt=0.00")
        ttk.Label(
            lf,
            textvariable=self._tilt_var,
            font=("Consolas", 9),
            justify="left",
        ).pack(anchor="w", padx=8, pady=(0, 6))

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the periodic TCP update loop."""
        if not self._running:
            self._running = True
            self.after(self._poll_interval, self._tick_tcp_update)

    def stop(self) -> None:
        """Stop the update loop."""
        self._running = False

    def on_pose_changed(self, cb: Optional[Callable[[Dict[str, float]], None]]) -> None:
        """Register an optional callback fired when the pose changes.

        The callback receives a dict with keys:
        X_mm, Y_mm, Z_mm, Roll_deg, Pitch_deg, Yaw_deg, Tilt_deg, B_deg.
        """
        self._pose_changed_cb = cb

    def get_current_tcp_mm(self) -> Dict[str, float]:
        """Return the currently displayed TCP pose (mm / deg)."""
        return dict(self.current_tcp_pose)

    # ------------------------------------------------------------------
    #  Periodic update
    # ------------------------------------------------------------------

    def _tick_tcp_update(self) -> None:
        if not self._running:
            return
        try:
            self._sync_positions()
            self._compute_and_display()
        except Exception as exc:
            # Avoid crashing the after-loop on transient errors.
            print(f"[TcpPosePanel] TCP update error: {exc}")

        self.after(self._poll_interval, self._tick_tcp_update)

    def _sync_positions(self) -> None:
        """Pull latest machine positions from app.mpos into the local buffer."""
        mpos: Dict[str, float] = getattr(self.app, "mpos", {})
        for ax in AXES:
            new_val = float(mpos.get(ax, 0.0))
            old_val = self._axis_positions.get(ax, 0.0)
            if abs(new_val - old_val) > MOTION_EPS:
                self._axis_positions[ax] = new_val

    def _compute_and_display(self) -> None:
        """Run FK and update the label + internal pose cache."""
        dh: Optional[DHModel] = getattr(self.app, "dh", None)
        if dh is None:
            return

        # Build joint-angle dict in machine degrees
        joints_dict: Dict[str, float] = {
            ax: float(self._axis_positions.get(ax, 0.0))
            for ax in ("A", "X", "Y", "B", "Z", "C")
        }

        # Apply post-transform (offset + scale) and get list in DH-row order
        joints_list = dh.apply_post_transform(joints_dict)

        # Forward kinematics
        X, Y, Z, Roll, Pitch, Yaw, Tilt = fk6_forward_mm(
            dh.geom, joints_list, dh_model=dh
        )

        # Update label text (RPY)
        lines = [
            "TCP ",
            f"X={X:.2f} mm",
            f"Y={Y:.2f} mm",
            f"Z={Z:.2f} mm",
            f"Roll={Roll:.2f}",
            f"Pitch={Pitch:.2f}",
            f"Yaw={Yaw:.2f}",
        ]
        self._tcp_var.set("\n".join(lines))
        self._tilt_var.set(f"Tilt={Tilt:.2f}")

        # Build consistent pose dict
        pose: Dict[str, float] = {
            "X_mm": X,
            "Y_mm": Y,
            "Z_mm": Z,
            "Roll_deg": Roll,
            "Pitch_deg": Pitch,
            "Yaw_deg": Yaw,
            "Tilt_deg": Tilt,
            "B_deg": Tilt,
        }

        # Fire callback only on real change
        if _pose_diff(self.current_tcp_pose, pose) > 1e-6:
            self.current_tcp_pose = pose
            if self._pose_changed_cb is not None:
                try:
                    self._pose_changed_cb(dict(pose))
                except Exception as exc:
                    print(f"[TcpPosePanel] pose callback error: {exc}")

    # ------------------------------------------------------------------
    #  Cleanup
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        self._running = False
        super().destroy()
