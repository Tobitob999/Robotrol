"""
Robotrol v2.0 — Main application coordinator.

Replaces the monolith's ExecuteApp class (6102 lines).
Connects backend modules (serial, kinematics, config, queue)
to GUI panels/tabs via a thin coordination layer.
"""

from __future__ import annotations

import logging
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict, Optional

from robotrol.config.constants import AXES, MOTION_EPS, SOFTMAX_RE
from robotrol.config.app_config import AppConfig
from robotrol.config.profiles import ProfileManager
from robotrol.serial.client import SerialClient
from robotrol.serial.protocol import parse_status_line, parse_softmax, is_homing_command
from robotrol.queue.gcode_queue import GCodeQueue
from robotrol.visualizer.udp_mirror import UDPMirror
from robotrol.kinematics.dh_model import DHModel
from robotrol.kinematics.fk import fk6_forward_mm

# ── Defensive imports for GUI components (may not exist yet) ─────────────────

try:
    from robotrol.gui.theme import ThemeManager
except ImportError:
    ThemeManager = None  # type: ignore[assignment,misc]

try:
    from robotrol.gui.panels.toolbar import ToolbarFrame
except ImportError:
    ToolbarFrame = None  # type: ignore[assignment,misc]

try:
    from robotrol.gui.panels.queue_panel import QueuePanel
except ImportError:
    QueuePanel = None  # type: ignore[assignment,misc]

try:
    from robotrol.gui.panels.status_block import StatusBlock
except ImportError:
    StatusBlock = None  # type: ignore[assignment,misc]

try:
    from robotrol.gui.panels.tcp_pose_panel import TcpPosePanel
except ImportError:
    TcpPosePanel = None  # type: ignore[assignment,misc]

# ── Tab imports (all defensive) ───────────────────────────────────────────────
_TAB_REGISTRY = []

def _try_tab(module_path: str, class_name: str, label: str):
    """Register a tab for lazy import."""
    _TAB_REGISTRY.append((module_path, class_name, label))

_try_tab("robotrol.gui.tabs.manual_control", "ManualControlTab", "Manual")
_try_tab("robotrol.gui.tabs.endstops", "EndstopsTab", "Endstops")
_try_tab("robotrol.gui.tabs.fixed_tcp", "FixedTcpTab", "Fixed TCP")
_try_tab("robotrol.gui.tabs.gcode", "GCodeTab", "G-Code")
_try_tab("robotrol.gui.tabs.kinematics", "KinematicsTab", "Kinematics")
_try_tab("robotrol.gui.tabs.simulation_pose", "SimulationPoseTab", "Simulation")
_try_tab("robotrol.gui.tabs.vision", "VisionTab", "Vision")
_try_tab("robotrol.gui.tabs.chess_vision", "ChessVisionTab", "Chess")
_try_tab("robotrol.gui.tabs.gamepad", "GamepadTab", "Gamepad")
_try_tab("robotrol.gui.tabs.pickplace", "PickPlaceTab", "Pick&Place")
_try_tab("robotrol.gui.tabs.ota_updater", "OtaUpdaterTab", "OTA/Config")
_try_tab("robotrol.gui.tabs.commands", "CommandsTab", "Commands")


logger = logging.getLogger(__name__)


class RobotrolApp:
    """Main window coordinator.  Connects backend to GUI."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Robotrol V2.0")

        # ── Backend (no tkinter dependencies) ────────────────────────────
        self.config = AppConfig.from_base_dir(self._detect_base_dir())
        self.profile_mgr = ProfileManager(self.config.base_dir)
        self.serial = SerialClient()
        self.dh: Optional[DHModel] = None
        self.queue = GCodeQueue(
            send_fn=self.serial.send_line,
            on_log=self.log,
            send_ctrl_x_fn=self.serial.send_ctrl_x,
        )
        self.udp = UDPMirror()

        # ── State ────────────────────────────────────────────────────────
        self.axis_positions: Dict[str, float] = {ax: 0.0 for ax in AXES}
        self.mpos: Dict[str, float] = {ax: 0.0 for ax in AXES}
        self.wco: Dict[str, float] = {ax: 0.0 for ax in AXES}
        self.machine_state: str = "Unknown"
        self.hw_limits: Dict[str, tuple] = {}
        self.can_global_home: bool = True
        self.can_axis_home: bool = True
        self._use_wpos: bool = False

        # ── GUI components (guarded against missing modules) ─────────────
        self.theme: Any = None
        if ThemeManager is not None:
            self.theme = ThemeManager(root)

        self.toolbar: Any = None
        if ToolbarFrame is not None:
            self.toolbar = ToolbarFrame(root, self)

        self.notebook = ttk.Notebook(root)

        self.queue_panel: Any = None
        if QueuePanel is not None:
            self.queue_panel = QueuePanel(root, self)

        self.status_block: Any = None
        if StatusBlock is not None:
            self.status_block = StatusBlock(root, self)

        self.tcp_panel: Any = None
        if TcpPosePanel is not None:
            self.tcp_panel = TcpPosePanel(root, self)

        # ── Layout ───────────────────────────────────────────────────────
        if self.toolbar is not None:
            self.toolbar.pack(fill="x")
        if self.status_block is not None:
            self.status_block.pack(fill="x")

        # Main area: notebook left, queue right
        main = ttk.PanedWindow(root, orient="horizontal")
        main.add(self.notebook, weight=3)
        if self.queue_panel is not None:
            main.add(self.queue_panel, weight=1)
        main.pack(fill="both", expand=True)

        if self.tcp_panel is not None:
            self.tcp_panel.pack(fill="x", side="bottom")

        self._tabs: Dict[str, Any] = {}
        self._build_tabs()

        # ── Serial listener ──────────────────────────────────────────────
        self.serial.listeners.append(self._on_serial_line)

        # ── Profile loading ──────────────────────────────────────────────
        self.profile_mgr.on_change(self._on_profile_changed)
        self._load_initial_profile()

    # ──────────────────────────────────────────────────────────────────────
    #  Tab registration
    # ──────────────────────────────────────────────────────────────────────

    def _build_tabs(self) -> None:
        """Instantiate and register all GUI tabs from _TAB_REGISTRY."""
        import importlib
        for module_path, class_name, label in _TAB_REGISTRY:
            try:
                mod = importlib.import_module(module_path)
                cls = getattr(mod, class_name)
                tab = cls(self.notebook, self)
                self.notebook.add(tab, text=label)
                self._tabs[label] = tab
                logger.debug("Tab loaded: %s", label)
            except Exception as exc:
                logger.warning("Tab %s failed to load: %s", label, exc)

    # ──────────────────────────────────────────────────────────────────────
    #  Base-dir detection
    # ──────────────────────────────────────────────────────────────────────

    @staticmethod
    def _detect_base_dir() -> str:
        """Determine the application base directory.

        Uses the ``ROBOTROL_BASE`` environment variable if set,
        otherwise falls back to the current working directory.
        """
        import os
        import sys

        env = os.environ.get("ROBOTROL_BASE")
        if env and os.path.isdir(env):
            return env
        if getattr(sys, "frozen", False):
            return os.path.dirname(sys.executable)
        return os.getcwd()

    # ──────────────────────────────────────────────────────────────────────
    #  Profile management
    # ──────────────────────────────────────────────────────────────────────

    def _load_initial_profile(self) -> None:
        """Load the last-used profile from saved settings, or the default."""
        last = self.config.settings.get("last_profile", "")
        name = last if last else None
        try:
            self.profile_mgr.load(name)
        except Exception as exc:
            logger.warning("Failed to load profile %r: %s", name, exc)
            try:
                self.profile_mgr.load()
            except Exception as exc2:
                logger.error("Failed to load default profile: %s", exc2)

    def _on_profile_changed(self, name: str, data: Dict[str, Any]) -> None:
        """Callback when the active profile changes.

        Reloads the DH model, updates hardware limit info, and refreshes
        homing capability flags.
        """
        # Rebuild DH model from profile data
        self.dh = DHModel.from_profile(data)

        # Update homing capability flags
        has_endstops = self.profile_mgr.has_endstops(name, data)
        self.can_global_home = has_endstops and self.serial.supports_global_homing
        self.can_axis_home = has_endstops and self.serial.supports_axis_homing

        # Persist last-used profile
        self.config.settings["last_profile"] = name
        try:
            self.config.save()
        except Exception as exc:
            logger.warning("Could not persist last_profile: %s", exc)

        # Notify GUI components that may need to refresh
        if self.tcp_panel is not None and hasattr(self.tcp_panel, "set_geom_dh"):
            try:
                self.tcp_panel.set_geom_dh(self.dh.geom)
            except Exception as exc:
                logger.debug("tcp_panel.set_geom_dh failed: %s", exc)

        self.log(f"[Profile] Loaded: {name}")

    # ──────────────────────────────────────────────────────────────────────
    #  Serial RX handling
    # ──────────────────────────────────────────────────────────────────────

    def _on_serial_line(self, line: str) -> None:
        """Process a single line received from the serial port.

        Runs on the RX thread — schedules GUI updates via ``root.after``.
        """
        if not line:
            return

        # ACK / error — notify queue worker
        if line == "ok":
            self.queue.notify_ack()
            return

        if line.startswith("error:"):
            self.queue.notify_error(line)
            return

        # Controller reboot / alarm resets G92 state
        if line.startswith("Grbl ") or line.startswith("ALARM"):
            self._use_wpos = False
            self.wco = {ax: 0.0 for ax in AXES}

        # Soft-limit / MaxTravel ($130..$135)
        parsed_sm = parse_softmax(line)
        if parsed_sm is not None:
            ax, max_travel = parsed_sm
            self.hw_limits[ax] = (0.0, max_travel)

        # Real-time status line  <State|MPos:...|...>
        if line.startswith("<") and line.endswith(">"):
            status = parse_status_line(line)
            self._apply_status(status)
            return

        # Everything else — log non-noise lines
        if not self._is_noise(line):
            self.root.after(0, self.log, "RX: " + line)

    def _apply_status(self, status: Dict[str, Any]) -> None:
        """Apply parsed status data to internal state and schedule UI update."""
        state = status.get("state")
        if state:
            self.machine_state = state

        mpos = status.get("mpos", {})
        wpos = status.get("wpos", {})
        wco_update = status.get("wco", {})
        endstop_pins = status.get("pn", set())

        # Update WCO cache
        if wco_update:
            self.wco.update(wco_update)

        # Keep mpos updated
        if mpos:
            for ax, val in mpos.items():
                if ax in AXES:
                    if self._use_wpos:
                        self.mpos[ax] = val - self.wco.get(ax, 0.0)
                    else:
                        self.mpos[ax] = val

        # Determine display positions
        if self._use_wpos and wpos:
            pose_src = wpos
        elif self._use_wpos and mpos:
            # FluidNC $10=1 doesn't send WPos — compute from WCO
            pose_src = {
                ax: mpos[ax] - self.wco.get(ax, 0.0)
                for ax in mpos
            }
        else:
            pose_src = mpos if mpos else wpos

        # Update axis positions
        updated = False
        for ax, val in pose_src.items():
            if ax not in AXES:
                continue
            if abs(val - self.axis_positions.get(ax, 0.0)) > MOTION_EPS:
                self.axis_positions[ax] = val
                updated = True

        # Store last status on client for external consumers
        effective_wpos = wpos
        if not effective_wpos and self._use_wpos and mpos:
            effective_wpos = {
                ax: mpos[ax] - self.wco.get(ax, 0.0)
                for ax in mpos
            }
        self.serial.last_status = {
            "state": state,
            "MPos": mpos,
            "WPos": effective_wpos,
        }

        # UDP broadcast
        full_pose: Dict[str, float] = {}
        full_pose.update(pose_src)
        for ax in AXES:
            if ax not in full_pose and ax in self.axis_positions:
                full_pose[ax] = float(self.axis_positions[ax])
        if full_pose:
            self.udp.send_joints(full_pose)

        # Schedule GUI refresh on main thread
        if updated or state:
            self.root.after(0, self._refresh_gui_positions, state, endstop_pins)

    def _refresh_gui_positions(
        self,
        state: Optional[str],
        endstop_pins: set,
    ) -> None:
        """Update GUI elements with current positions and state.

        Called on the main thread via ``root.after``.
        """
        if self.status_block is not None and state and hasattr(self.status_block, "update_state"):
            try:
                self.status_block.update_state(state)
            except Exception as exc:
                logger.debug("status_block.update_state failed: %s", exc)

        if self.tcp_panel is not None and hasattr(self.tcp_panel, "update_positions"):
            try:
                self.tcp_panel.update_positions(self.axis_positions)
            except Exception as exc:
                logger.debug("tcp_panel.update_positions failed: %s", exc)

    @staticmethod
    def _is_noise(line: str) -> bool:
        """Return True for lines that should not be logged."""
        return (
            line.startswith("<")
            or line == "ok"
            or line == "?"
            or line.startswith("[MSG:")
            or line.startswith("[GC:")
            or "MPos:" in line
            or "WPos:" in line
            or "FS:" in line
            or "Ov:" in line
        )

    # ──────────────────────────────────────────────────────────────────────
    #  FK / TCP computation
    # ──────────────────────────────────────────────────────────────────────

    def get_current_tcp_mm(self) -> Dict[str, float]:
        """Compute current TCP pose from machine positions via FK.

        Returns a dict with keys X_mm, Y_mm, Z_mm, Roll_deg, Pitch_deg,
        Yaw_deg.  Falls back to zeros on error.
        """
        zero_result: Dict[str, float] = {
            "X_mm": 0.0,
            "Y_mm": 0.0,
            "Z_mm": 0.0,
            "Roll_deg": 0.0,
            "Pitch_deg": 0.0,
            "Yaw_deg": 0.0,
        }

        if self.dh is None:
            return zero_result

        try:
            # Build joint angles in DH-row order with post-transform
            joints_transformed = self.dh.apply_post_transform(self.mpos)
            x, y, z, roll, pitch, yaw, _tilt = fk6_forward_mm(
                self.dh.geom,
                joints_transformed,
                dh_model=self.dh,
            )
            return {
                "X_mm": x,
                "Y_mm": y,
                "Z_mm": z,
                "Roll_deg": roll,
                "Pitch_deg": pitch,
                "Yaw_deg": yaw,
            }
        except Exception as exc:
            logger.debug("FK computation failed: %s", exc)
            return zero_result

    # ──────────────────────────────────────────────────────────────────────
    #  Public command interface
    # ──────────────────────────────────────────────────────────────────────

    def log(self, msg: str) -> None:
        """Log a message.  Delegates to queue_panel if available."""
        if self.queue_panel is not None and hasattr(self.queue_panel, "log"):
            try:
                self.queue_panel.log(msg)
                return
            except Exception:
                pass
        print(msg)

    def send_now(self, cmd: str) -> None:
        """Send a G-Code command immediately (bypassing the queue).

        Blocks homing commands when the profile has no endstops.
        """
        stripped = cmd.strip()
        if not stripped:
            return

        # Homing safety lock
        if is_homing_command(stripped) and not self.can_global_home:
            profile = self.profile_mgr.active_name
            self.log(
                f"BLOCKED: Homing '{stripped}' disabled for "
                f"'{profile}' (no endstops)."
            )
            return

        # Homing resets work coordinates
        if stripped.startswith("$H"):
            self._use_wpos = False
            self.wco = {ax: 0.0 for ax in AXES}

        try:
            self.serial.send_line(stripped)
            self.log("TX: " + stripped)
        except Exception as exc:
            self.log(f"Send error: {exc}")

    def enqueue(self, cmd: str) -> None:
        """Add a G-Code line to the execution queue."""
        self.queue.enqueue(cmd)

    # ──────────────────────────────────────────────────────────────────────
    #  Shutdown / cleanup
    # ──────────────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Clean up all resources and destroy the main window."""
        # Stop TCP panel updates
        if self.tcp_panel is not None and hasattr(self.tcp_panel, "stop"):
            try:
                self.tcp_panel.stop()
            except Exception as exc:
                logger.debug("tcp_panel.stop failed: %s", exc)

        # Disconnect serial
        try:
            self.serial.disconnect()
        except Exception as exc:
            logger.debug("serial.disconnect failed: %s", exc)

        # Close UDP socket
        try:
            self.udp.close()
        except Exception as exc:
            logger.debug("udp.close failed: %s", exc)

        # Save config
        try:
            self.config.save()
        except Exception as exc:
            logger.debug("config.save failed: %s", exc)

        # Destroy the window
        try:
            self.root.destroy()
        except Exception:
            pass
