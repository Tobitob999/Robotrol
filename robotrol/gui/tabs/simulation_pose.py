"""Simulation / Pose tab -- RoboSim visualizer integration.

Extracted from monolith lines 3318-3419.
"""

from __future__ import annotations

import logging
import os
import subprocess
import sys
import time
import tkinter as tk
from tkinter import ttk
from typing import Any, Optional

logger = logging.getLogger(__name__)

try:
    import psutil  # type: ignore[import-untyped]
except ImportError:
    psutil = None  # type: ignore[assignment]


def _get_base_dir() -> str:
    """Resolve base directory for both Python and packaged EXE."""
    if getattr(sys, "frozen", False):  # running as .exe (PyInstaller)
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


_BASE_DIR = _get_base_dir()
_VISUALIZER_SCRIPT = os.path.join(_BASE_DIR, "robosim_visualizer_v90.py")
_VISUALIZER_EXE = os.path.join(_BASE_DIR, "robosim_visualizer_v90.exe")
_VISUALIZER_TAG = "RoboSim Control Center"


class SimulationPoseTab(ttk.Frame):
    """Tab for launching the external RoboSim 3D visualizer and sending poses via UDP."""

    def __init__(self, parent: ttk.Widget, app: Any):
        super().__init__(parent)
        self.app = app
        self._build_ui()

    # ------------------------------------------------------------------
    #  Internal helpers
    # ------------------------------------------------------------------

    def _log(self, msg: str) -> None:
        app_log = getattr(self.app, "log", None)
        if callable(app_log):
            try:
                app_log(msg)
                return
            except (AttributeError, TypeError):
                pass
        logger.info(msg)

    @staticmethod
    def _find_visualizer_process() -> Optional[Any]:
        """Search for a running RoboSim process to prevent double-launch."""
        if psutil is None:
            return None
        try:
            for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
                cmdline = " ".join(p.info.get("cmdline") or [])
                if (
                    "robosim_visualizer_v90" in cmdline
                    or _VISUALIZER_TAG in cmdline
                    or "RoboSim" in (p.info.get("name") or "")
                ):
                    return p
        except (psutil.NoSuchProcess, psutil.AccessDenied, OSError):
            pass
        return None

    def _start_visualizer(self) -> None:
        """Start the RoboSim visualizer as a separate process (single instance)."""
        existing = self._find_visualizer_process()
        if existing:
            self._log(
                f"Visualizer is already running (PID={existing.pid}) -- no new start."
            )
            return

        try:
            if os.path.exists(_VISUALIZER_EXE):
                self._log(f"Starting visualizer EXE: {_VISUALIZER_EXE}")
                subprocess.Popen([_VISUALIZER_EXE], cwd=_BASE_DIR)

            elif os.path.exists(_VISUALIZER_SCRIPT):
                if getattr(sys, "frozen", False):
                    self._log(
                        "Visualizer EXE missing; frozen build cannot start .py script."
                    )
                    return
                self._log(f"Starting visualizer PY: {_VISUALIZER_SCRIPT}")
                log_path = os.path.join(_BASE_DIR, "robosim_visualizer_start.log")
                try:
                    log_fp: Any = open(log_path, "a", encoding="utf-8")
                except OSError:
                    log_fp = None
                subprocess.Popen(
                    [sys.executable, _VISUALIZER_SCRIPT],
                    cwd=_BASE_DIR,
                    stdout=log_fp or subprocess.DEVNULL,
                    stderr=log_fp or subprocess.STDOUT,
                )
                if log_fp is not None:
                    log_fp.close()
                self._log(f"Visualizer startup log: {log_path}")
            else:
                self._log(f"No visualizer found in {_BASE_DIR}")
                return

            time.sleep(0.5)
            self._log("Visualizer started successfully.")

            # Send robot profile to visualizer after short delay
            send_profile = getattr(self.app, "_send_vis_robot_profile", None)
            if callable(send_profile):
                try:
                    self.after(700, send_profile)
                except (AttributeError, tk.TclError):
                    pass

        except OSError as exc:
            self._log(f"Visualizer start failed: {exc}")

    # ------------------------------------------------------------------
    #  UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        ttk.Label(
            self,
            text="RoboSim Visualizer",
            font=("Segoe UI", 12, "bold"),
        ).pack(pady=10)

        ttk.Button(
            self,
            text="Open Simulation (external process)",
            command=self._start_visualizer,
        ).pack(pady=20)

        # --- UDP mirror status (if available) ---
        udp = getattr(self.app, "udp", None)
        if udp is not None:
            status_frame = ttk.LabelFrame(self, text="UDP Pose Mirror")
            status_frame.pack(padx=20, pady=10, fill=tk.X)

            self._udp_status_var = tk.StringVar(value="Unknown")
            ttk.Label(status_frame, textvariable=self._udp_status_var).pack(
                padx=10, pady=6,
            )

            def _refresh_udp_status() -> None:
                connected = getattr(udp, "connected", None)
                if connected is not None:
                    self._udp_status_var.set(
                        "Connected" if connected else "Disconnected"
                    )
                else:
                    self._udp_status_var.set("Status unavailable")

            ttk.Button(
                status_frame,
                text="Refresh Status",
                command=_refresh_udp_status,
            ).pack(padx=10, pady=(0, 6))
