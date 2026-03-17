"""
Robotrol v2.0 — Manual Control Tab.

The primary daily-use tab for direct robot joint control.
Provides per-axis jog buttons, percentage moves, sliders, entry fields,
speed control, homing, zeroing, and pose-to-queue functionality.

Extracted from Robotrol_FluidNC_v7_3.py lines 3500-4200.

Safety note: This is safety-critical code that controls physical robot motion.
All limit checks and guards from the original monolith are preserved.
"""

from __future__ import annotations

import logging
import time
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict, Optional, Tuple

from robotrol.config.constants import (
    AXES,
    AXIS_LIMITS_DEFAULT,
    DEFAULT_ENDSTOP_LIMITS,
    MAX_FEED,
    MAX_HOME_FEED,
    MOTION_EPS,
)

logger = logging.getLogger(__name__)


# ── Speed mapping ────────────────────────────────────────────────────────────

def _map_speed(slider_val: float, max_feed: float) -> float:
    """Map a 0..1000 slider value to 0..max_feed (mm/min or deg/min)."""
    clamped = max(0.0, min(1000.0, slider_val))
    return (clamped / 1000.0) * max_feed


# ── Axis limits helpers ──────────────────────────────────────────────────────

def _manual_axis_limits(
    manual_limits: Dict[str, Tuple[float, float]],
    ax: str,
) -> Tuple[float, float]:
    """Return (lo, hi) for an axis from manual overrides or defaults."""
    lo, hi = manual_limits.get(
        ax, DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax])
    )
    try:
        lo = float(lo)
        hi = float(hi)
    except (ValueError, TypeError):
        lo, hi = DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax])
    if lo > hi:
        lo, hi = hi, lo
    return lo, hi


def _effective_axis_limits(
    axis_limits: Dict[str, Tuple[float, float]],
    hw_limits: Dict[str, Tuple[float, float]],
    endstop_limits_enabled: bool,
    ax: str,
) -> Tuple[float, float]:
    """Return (lo, hi) incorporating hardware limits when available."""
    lo, hi = axis_limits.get(
        ax, DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax])
    )
    if ax in hw_limits:
        hw_lo, hw_hi = hw_limits[ax]
        use_hw = True
        if endstop_limits_enabled and lo < 0 and hw_lo >= 0:
            use_hw = False
        if use_hw:
            lo = max(lo, hw_lo)
            hi = min(hi, hw_hi)
    return lo, hi


def _clamp_with_limits(
    axis_limits: Dict[str, Tuple[float, float]],
    hw_limits: Dict[str, Tuple[float, float]],
    endstop_limits_enabled: bool,
    ax: str,
    value: Any,
) -> float:
    """Parse *value* as float and clamp to effective axis limits."""
    try:
        val = float(value)
    except (ValueError, TypeError):
        val = 0.0
    lo, hi = _effective_axis_limits(axis_limits, hw_limits, endstop_limits_enabled, ax)
    return max(lo, min(hi, val))


# ═══════════════════════════════════════════════════════════════════════════════
#  ManualControlTab
# ═══════════════════════════════════════════════════════════════════════════════

class ManualControlTab(ttk.Frame):
    """Six-axis manual control panel with jog, slider, and homing controls.

    Parameters
    ----------
    parent : tk.Widget
        The parent notebook (or any container).
    app : RobotrolApp
        The main application coordinator providing ``send_now``, ``enqueue``,
        ``axis_positions``, ``hw_limits``, ``log``, etc.
    """

    def __init__(self, parent: tk.Widget, app: Any) -> None:
        super().__init__(parent)
        self.app = app

        # ── Per-tab state ─────────────────────────────────────────────────
        self.live_move = tk.BooleanVar(value=True)
        self.mode_absolute = tk.BooleanVar(value=True)
        self.poll_positions = tk.BooleanVar(value=False)
        self.manual_speed_factor = tk.DoubleVar(value=1 / 20)

        # Speed control state
        self.speed_val = tk.DoubleVar(value=500)
        self.max_feed = tk.DoubleVar(value=MAX_FEED)
        self.speed_mode = tk.StringVar(value="Mid")

        # Widget registries (populated during build)
        self.axis_labels: Dict[str, ttk.Label] = {}
        self.axis_vars: Dict[str, tk.DoubleVar] = {}
        self.axis_entries: Dict[str, ttk.Entry] = {}
        self.axis_endstop_icons: Dict[str, tuple] = {}
        self.axis_scales: Dict[str, ttk.Scale] = {}
        self._axis_scale_redraw: Dict[str, Any] = {}
        self._axis_entry_order: list = []
        self._user_editing: Dict[str, bool] = {}
        self._start_val: float = 0.0

        # Factor-button registry for highlight toggling
        self._factor_buttons: Dict[int, ttk.Button] = {}

        # Homing button references for enable/disable
        self._homing_global_buttons: list = []
        self._homing_axis_buttons: list = []

        # Status block
        self.status_block_var = tk.StringVar(value="Status: -")
        self.endstop_indicators: Dict[str, tuple] = {}

        # ── Build the UI ──────────────────────────────────────────────────
        self._build_ui()

    # ──────────────────────────────────────────────────────────────────────
    #  Convenience accessors into app state
    # ──────────────────────────────────────────────────────────────────────

    @property
    def _axis_limits(self) -> Dict[str, Tuple[float, float]]:
        return getattr(self.app, "axis_limits", DEFAULT_ENDSTOP_LIMITS)

    @property
    def _hw_limits(self) -> Dict[str, Tuple[float, float]]:
        return getattr(self.app, "hw_limits", {})

    @property
    def _endstop_limits_enabled(self) -> bool:
        return getattr(self.app, "endstop_limits_enabled", True)

    @property
    def _manual_limits(self) -> Dict[str, Tuple[float, float]]:
        return getattr(self.app, "manual_limits", {})

    @property
    def _profile_has_endstops(self) -> bool:
        return getattr(self.app, "can_global_home", True)

    # ──────────────────────────────────────────────────────────────────────
    #  Limit helpers (delegate to module-level functions)
    # ──────────────────────────────────────────────────────────────────────

    def clamp_with_limits(self, ax: str, value: Any) -> float:
        """Clamp *value* to the effective limits of axis *ax*."""
        return _clamp_with_limits(
            self._axis_limits, self._hw_limits,
            self._endstop_limits_enabled, ax, value,
        )

    def get_manual_axis_limits(self, ax: str) -> Tuple[float, float]:
        """Return (lo, hi) for manual slider range."""
        return _manual_axis_limits(self._manual_limits, ax)

    def get_effective_axis_limits(self, ax: str) -> Tuple[float, float]:
        """Return (lo, hi) incorporating hardware limits."""
        return _effective_axis_limits(
            self._axis_limits, self._hw_limits,
            self._endstop_limits_enabled, ax,
        )

    # ──────────────────────────────────────────────────────────────────────
    #  Speed helpers
    # ──────────────────────────────────────────────────────────────────────

    def _current_feed(self) -> float:
        """Compute the feed rate from speed slider * manual speed factor."""
        return _map_speed(self.speed_val.get(), self.max_feed.get()) * self.manual_speed_factor.get()

    # ──────────────────────────────────────────────────────────────────────
    #  Position updates
    # ──────────────────────────────────────────────────────────────────────

    def update_all_positions(self) -> None:
        """Refresh all axis displays from ``app.axis_positions``."""
        for ax in AXES:
            pos = self.app.axis_positions.get(ax, 0.0)
            if ax in self.axis_vars:
                self.axis_vars[ax].set(pos)
            if ax in self.axis_entries:
                ent = self.axis_entries[ax]
                ent.delete(0, tk.END)
                ent.insert(0, f"{pos:.3f}")
            if ax in self.axis_labels:
                self.axis_labels[ax].config(text=f"{pos:.3f}")

    def update_position_display(self) -> None:
        """Update only the position labels from ``app.axis_positions``."""
        for ax in AXES:
            pos = self.app.axis_positions.get(ax, 0.0)
            if ax in self.axis_labels:
                self.axis_labels[ax].configure(text=f"{pos:.3f}")

    # ──────────────────────────────────────────────────────────────────────
    #  Movement commands
    # ──────────────────────────────────────────────────────────────────────

    def _send_percent_move(self, ax: str, pct: int) -> None:
        """Move *ax* to *pct* (-100..+100) of its manual range."""
        lo, hi = self.get_manual_axis_limits(ax)
        target = lo + (hi - lo) * (pct + 100) / 200.0
        target = self.clamp_with_limits(ax, target)
        feed = self._current_feed()
        self.app.send_now("G90")
        self.app.send_now(f"G1 {ax}{target:.3f} F{feed:.0f}")
        self.app.log(f"{ax}  {target:.3f} ({pct:+d}%) @F={feed:.0f}")
        self.update_all_positions()

    def _send_jog(self, ax: str, dist: float) -> None:
        """Send a relative jog move for *ax*."""
        feed = self._current_feed()
        self.app.send_now("G91")
        self.app.send_now(f"G1 {ax}{dist:.3f} F{feed:.0f}")
        self.app.send_now("G90")
        self.app.log(f"{ax} jog {dist:+g} @F={feed:.0f}")
        self.update_all_positions()

    def _send_axis_direct(self, ax: str, val: float) -> None:
        """Send an absolute move for *ax* to *val*."""
        feed = self._current_feed()
        self.app.send_now("G90")
        self.app.send_now(f"G1 {ax}{val:.3f} F{feed:.0f}")
        self.app.log(f"{ax}  {val:.3f} @F={feed:.0f}")
        self.update_position_display()

    def _enqueue_axis(self, ax: str, val: float) -> None:
        """Enqueue an absolute move for *ax*."""
        feed = self._current_feed()
        self.app.enqueue(f"G1 {ax}{val:.3f} F{feed:.0f}")
        self.update_position_display()

    def _to_cli(self, ax: str, val: float) -> None:
        """Put a G1 command into the CLI entry (if available)."""
        feed = self._current_feed()
        entry_cmd = getattr(self.app, "entry_cmd", None)
        if entry_cmd is not None:
            entry_cmd.delete(0, tk.END)
            entry_cmd.insert(0, f"G1 {ax}{val:.3f} F{feed:.0f}")

    # ──────────────────────────────────────────────────────────────────────
    #  Pose (all axes) to queue
    # ──────────────────────────────────────────────────────────────────────

    def add_current_pose_to_queue(self) -> None:
        """Read all axis entries, clamp, build a G1 line, and enqueue."""
        parts: list = []
        for ax in AXES:
            ent = self.axis_entries.get(ax)
            if ent is None:
                self.app.log(f"[add_current_pose_to_queue] Missing entry for {ax}")
                return
            raw = ent.get().strip().replace(",", ".")
            if not raw:
                self.app.log(f"[add_current_pose_to_queue] Empty entry for {ax}")
                return
            try:
                v = float(raw)
            except (ValueError, TypeError):
                self.app.log(f"[add_current_pose_to_queue] Invalid value for {ax}: {raw}")
                return
            v = self.clamp_with_limits(ax, v)
            parts.append(f"{ax}{v:.3f}")

        feed = self._current_feed()
        gline = "G1 " + " ".join(parts) + f" F{feed:.0f}"
        self.app.enqueue(gline)
        self.app.log(f"Pose -> Queue: {gline}")

    # ──────────────────────────────────────────────────────────────────────
    #  Zeroing (G92)
    # ──────────────────────────────────────────────────────────────────────

    def manual_zero_current_pose(self) -> None:
        """Set all current positions as zero via G92."""
        try:
            self.app.send_now("G92 X0 Y0 Z0 A0 B0 C0")
            for ax in AXES:
                self.app.axis_positions[ax] = 0.0
                if ax in self.axis_vars:
                    self.axis_vars[ax].set(0.0)
                ent = self.axis_entries.get(ax)
                if ent is not None:
                    ent.delete(0, tk.END)
                    ent.insert(0, "0.000")
            self.update_position_display()
            self.app.log("Zero All (G92)")
        except Exception as exc:
            self.app.log(f"Zero error: {exc}")

    # ──────────────────────────────────────────────────────────────────────
    #  Homing
    # ──────────────────────────────────────────────────────────────────────

    def goto_home(self) -> None:
        """Move all axes to zero at safe speed."""
        try:
            f = _map_speed(self.speed_val.get(), self.max_feed.get())
            f_home = min(f, MAX_HOME_FEED)
            self.app.send_now("$X")
            self.app.send_now("G90")
            gline = f"G1 X0 Y0 Z0 A0 B0 C0 F{f_home:.0f}"
            self.app.send_now(gline)
            self.app.log(f"Goto Home: {gline} (F_home={f_home:.0f})")
        except Exception as exc:
            self.app.log(f"Goto-home error: {exc}")

    def enqueue_home(self) -> None:
        """Enqueue a move to home (all zeros)."""
        f = _map_speed(self.speed_val.get(), self.max_feed.get())
        gline = f"G90 G1 X0 Y0 Z0 A0 B0 C0 F{f:.0f}"
        self.app.enqueue(gline)
        self.app.log("To Home -> added to queue")

    # ──────────────────────────────────────────────────────────────────────
    #  Entry read + clamp
    # ──────────────────────────────────────────────────────────────────────

    def _get_entry_value(self, ax: str) -> float:
        """Read the entry for *ax*, clamp to limits, update entry text."""
        ent = self.axis_entries[ax]
        var = self.axis_vars[ax]
        try:
            v = float(ent.get().replace(",", "."))
        except (ValueError, TypeError):
            v = var.get()
        v = self.clamp_with_limits(ax, v)
        ent.delete(0, tk.END)
        ent.insert(0, f"{v:.3f}")
        return v

    # ──────────────────────────────────────────────────────────────────────
    #  Speed control
    # ──────────────────────────────────────────────────────────────────────

    def _set_speed_percent(self, pct: int) -> None:
        """Set speed slider to a percentage (0..100)."""
        val = max(0, min(1000, int(10 * pct)))
        self.speed_val.set(val)
        mm = _map_speed(val, self.max_feed.get())
        self._lbl_speed_val.config(text=f"{self.speed_val.get():.1f}")
        self.app.log(
            f"Speed set to {pct}% ({val}/1000 -> {mm:.0f} mm/min "
            f"@ {self.speed_mode.get()})"
        )

    def _set_speed_mode(self, mode: str) -> None:
        """Switch max-feed mode (Low / Mid / High)."""
        modes = {"Low": 6000.0, "Mid": 15000.0, "High": 50000.0}
        self.speed_mode.set(mode)
        self.max_feed.set(modes[mode])
        self.app.log(f"Max-Speed-Mode: {mode} ({modes[mode]} mm/min)")

    def _set_manual_factor(self, val: float) -> None:
        """Set the manual speed factor and update button highlights."""
        self.manual_speed_factor.set(val)
        for denom, btn in self._factor_buttons.items():
            if abs(val - 1 / denom) < 1e-9:
                btn.configure(style="SpeedButtonSelected.TButton")
            else:
                btn.configure(style="SpeedButton.TButton")
        self.app.log(f"Manual Speed Factor: 1/{int(1 / val)}")

    # ══════════════════════════════════════════════════════════════════════
    #  UI Construction
    # ══════════════════════════════════════════════════════════════════════

    def _build_ui(self) -> None:
        """Construct the complete manual-control user interface."""
        self._configure_styles()

        # Main position frame
        posf = ttk.LabelFrame(self, text="Position / Manual Axis Control")
        posf.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self._build_header(posf)
        self._build_axis_button_rows(posf)
        self._build_axis_slider_rows(posf)

        # Right-side panel (speed + homing + status)
        wrap = ttk.Frame(self)
        wrap.pack(side=tk.RIGHT, fill=tk.Y, padx=2, pady=2)
        self._build_speed_panel(wrap)

    def _configure_styles(self) -> None:
        """Set up ttk styles used by manual-control widgets."""
        style = ttk.Style()

        # Speed factor buttons
        style.configure(
            "SpeedButton.TButton",
            padding=(6, 3),
            font=("Segoe UI", 9),
        )
        style.configure(
            "SpeedButtonSelected.TButton",
            padding=(6, 3),
            font=("Segoe UI", 9, "bold"),
            background="#333333",
            foreground="white",
        )
        # Ensure both share the same layout
        base_layout = [
            ("Button.border", {"sticky": "nswe", "children": [
                ("Button.padding", {"sticky": "nswe", "children": [
                    ("Button.label", {"sticky": "nswe"}),
                ]}),
            ]}),
        ]
        style.layout("SpeedButton.TButton", base_layout)
        style.layout("SpeedButtonSelected.TButton", base_layout)

        # Flat mini buttons for jog / pct / action
        style.configure("FlatMini.TButton", padding=(2, 1), font=("Segoe UI", 8))
        style.configure(
            "FlatMiniSelected.TButton",
            padding=(2, 1),
            font=("Segoe UI", 8, "bold"),
        )

        # Speed mode / pct / homing buttons
        style.configure("SpeedMode.TButton", font=("Segoe UI", 8))
        style.configure("SpeedPct.TButton", font=("Segoe UI", 8))
        style.configure("HomingCtrl.TButton", padding=(6, 1), font=("Segoe UI", 9))

    # ── Header row ────────────────────────────────────────────────────────

    def _build_header(self, parent: tk.Widget) -> None:
        """Build the header row: LiveMove, Absolute, Poll, Zero, speed factor."""
        header = ttk.Frame(parent)
        header.pack(fill=tk.X, pady=(2, 4))

        # Checkbuttons
        ttk.Checkbutton(
            header, text="Live Move (on release)",
            variable=self.live_move,
        ).pack(side=tk.LEFT, padx=(4, 10))

        ttk.Checkbutton(
            header, text="Absolute (G90)",
            variable=self.mode_absolute,
        ).pack(side=tk.LEFT, padx=(0, 10))

        ttk.Checkbutton(
            header, text="Poll Positions",
            variable=self.poll_positions,
        ).pack(side=tk.LEFT, padx=(0, 25))

        ttk.Button(
            header, text="Zero (G92)",
            command=self.manual_zero_current_pose,
        ).pack(side=tk.RIGHT, padx=(0, 6))

        # Manual speed factor label + buttons
        ttk.Label(header, text="Manual Speed:", width=12).pack(
            side=tk.LEFT, padx=(4, 6),
        )

        for denom in (1, 5, 10, 20, 50, 100):
            b = ttk.Button(
                header,
                text=f"1/{denom}",
                style="SpeedButton.TButton",
                command=lambda d=denom: self._set_manual_factor(1 / d),
                width=6,
            )
            b.pack(side=tk.LEFT, padx=4, pady=2)
            self._factor_buttons[denom] = b

        # Default: 1/20
        self._set_manual_factor(1 / 20)

    # ── Axis button rows (percent + jog) ──────────────────────────────────

    def _build_axis_button_rows(self, parent: tk.Widget) -> None:
        """Build one row per axis with position label, percent, and jog buttons."""
        for ax in AXES:
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=1)

            # Axis name
            ttk.Label(row, text=ax, width=3).pack(side=tk.LEFT)

            # MPos display label
            lbl = ttk.Label(row, text="0.000", width=10, anchor="e")
            lbl.pack(side=tk.LEFT, padx=5)
            self.axis_labels[ax] = lbl

            # Endstop indicator circle
            c = tk.Canvas(row, width=14, height=14, highlightthickness=0)
            oval = c.create_oval(2, 2, 12, 12, fill="#999999")
            c.pack(side=tk.LEFT, padx=5)
            self.axis_endstop_icons[ax] = (c, oval, "unknown")

            # Percent buttons (-100%, -50%, 0, +50%, +100%)
            pct_frame = ttk.Frame(row, relief="solid", borderwidth=1)
            pct_frame.pack(side=tk.LEFT, padx=4)
            for pct in (-100, -50, 0, 50, 100):
                btn_text = "0" if pct == 0 else f"{pct:+d}%"
                style_name = (
                    "FlatMiniSelected.TButton" if pct == 0
                    else "FlatMini.TButton"
                )
                ttk.Button(
                    pct_frame,
                    text=btn_text,
                    width=6,
                    style=style_name,
                    command=lambda a=ax, p=pct: self._send_percent_move(a, p),
                ).pack(side=tk.LEFT, padx=1)

            # Jog buttons (relative)
            jog_frame = ttk.Frame(row, relief="solid", borderwidth=1)
            jog_frame.pack(side=tk.LEFT, padx=4)

            neg_steps = [10, 5, 1, 0.5, 0.1]
            pos_steps = [0.1, 0.5, 1, 5, 10]

            for step in neg_steps:
                ttk.Button(
                    jog_frame,
                    text=f"{-step:g}",
                    width=4,
                    style="FlatMini.TButton",
                    command=lambda a=ax, s=step: self._send_jog(a, -s),
                ).pack(side=tk.LEFT, padx=1)

            for step in pos_steps:
                ttk.Button(
                    jog_frame,
                    text=f"+{step:g}",
                    width=4,
                    style="FlatMini.TButton",
                    command=lambda a=ax, s=step: self._send_jog(a, +s),
                ).pack(side=tk.LEFT, padx=1)

    # ── Axis slider rows ─────────────────────────────────────────────────

    def _build_axis_slider_rows(self, parent: tk.Widget) -> None:
        """Build slider + entry + action buttons for each axis."""
        for ax in AXES:
            slider_row = ttk.Frame(parent)
            slider_row.pack(fill=tk.X, pady=(3, 3))

            ttk.Label(slider_row, text=ax, width=9).pack(side=tk.LEFT)

            lo, hi = self.get_manual_axis_limits(ax)
            var = tk.DoubleVar(value=0.0)
            self.axis_vars[ax] = var
            self._user_editing[ax] = False

            # Canvas with zero-line and tick marks
            slider_wrap = tk.Frame(slider_row)
            slider_wrap.pack(side=tk.LEFT, padx=4)

            scale_width = 600
            scale_height = 22
            canvas = tk.Canvas(
                slider_wrap, width=scale_width, height=scale_height,
                highlightthickness=0,
            )
            canvas.pack()

            # Draw scale decorations
            def _redraw(
                lo_val: float, hi_val: float,
                _canvas: tk.Canvas = canvas,
                _w: int = scale_width, _h: int = scale_height,
            ) -> None:
                _canvas.delete("all")
                span = hi_val - lo_val
                if abs(span) < 1e-9:
                    span = 1.0

                def val_to_x(v: float) -> float:
                    return (v - lo_val) / span * _w

                # Zero line
                zero_x = val_to_x(0.0)
                _canvas.create_line(zero_x, 0, zero_x, _h, fill="black", width=2)

                # Major ticks every 10 units
                start = int(lo_val - (lo_val % 10))
                end = int(hi_val) + 1
                for v in range(start, end, 10):
                    x = val_to_x(v)
                    _canvas.create_line(x, _h - 14, x, _h, fill="black", width=2)

            _redraw(lo, hi)
            self._axis_scale_redraw[ax] = _redraw

            # Slider on canvas
            s = ttk.Scale(
                canvas, from_=lo, to=hi, variable=var,
                orient=tk.HORIZONTAL, length=scale_width,
            )
            s.place(x=0, y=0)
            self.axis_scales[ax] = s

            # Entry field
            ent = ttk.Entry(slider_row, width=7)
            ent.insert(0, "0.0")
            ent.pack(side=tk.LEFT, padx=4)
            self.axis_entries[ax] = ent
            self._axis_entry_order.append(ent)

            # Slider -> Entry sync
            def _on_var_write(
                *_ignored: Any, _ax: str = ax, _ent: ttk.Entry = ent,
                _var: tk.DoubleVar = var,
            ) -> None:
                if self._user_editing.get(_ax, False):
                    return
                _ent.delete(0, tk.END)
                _ent.insert(0, f"{_var.get():.3f}")

            var.trace_add("write", _on_var_write)

            # ENTER sends immediately
            def _on_enter(
                event: Any = None, _ax: str = ax,
            ) -> str:
                val = self._get_entry_value(_ax)
                self._send_axis_direct(_ax, val)
                return "break"

            ent.bind("<Return>", _on_enter)

            # Tab navigation between entries
            self._bind_tab_navigation(ent)

            # FocusIn: select all
            ent.bind(
                "<FocusIn>",
                lambda e, _ent=ent: _ent.after(
                    1, lambda: _ent.selection_range(0, tk.END),
                ),
            )

            # Action buttons: ToQ / ToCLI / Send
            btn_block = ttk.Frame(slider_row)
            btn_block.pack(side=tk.LEFT, padx=2)

            def _btn_toQ(_ax: str = ax) -> None:
                v = self._get_entry_value(_ax)
                self._enqueue_axis(_ax, v)

            def _btn_toCLI(_ax: str = ax) -> None:
                v = self._get_entry_value(_ax)
                self._to_cli(_ax, v)

            def _btn_send(_ax: str = ax) -> None:
                v = self._get_entry_value(_ax)
                self._send_axis_direct(_ax, v)

            ttk.Button(
                btn_block, text="ToQ", width=4,
                style="FlatMini.TButton", command=_btn_toQ,
            ).pack(side=tk.LEFT, padx=1)
            ttk.Button(
                btn_block, text="ToCLI", width=5,
                style="FlatMini.TButton", command=_btn_toCLI,
            ).pack(side=tk.LEFT, padx=1)
            ttk.Button(
                btn_block, text="Send", width=5,
                style="FlatMini.TButton", command=_btn_send,
            ).pack(side=tk.LEFT, padx=1)

            # Slider live-move bindings
            def _press(e: Any, _ax: str = ax) -> None:
                self._user_editing[_ax] = True
                self._start_val = self.axis_vars[_ax].get()

            def _release(e: Any, _ax: str = ax) -> None:
                self._user_editing[_ax] = False
                val = self.axis_vars[_ax].get()
                if not self.live_move.get():
                    return
                if abs(val - self._start_val) < MOTION_EPS:
                    return
                feed = self._current_feed()
                self.app.send_now("G90")
                self.app.send_now(f"G1 {_ax}{val:.3f} F{feed:.0f}")
                self.app.log(f"{_ax} slide -> {val:.3f} @F={feed:.0f}")
                self.update_position_display()

            s.bind("<ButtonPress-1>", _press)
            s.bind("<ButtonRelease-1>", _release)

    def _bind_tab_navigation(self, ent: ttk.Entry) -> None:
        """Bind Tab / Shift-Tab to cycle through axis entries."""

        def _focus_and_select(w: tk.Widget) -> None:
            w.focus_set()
            w.after(1, lambda: w.selection_range(0, tk.END))

        def _next_entry(e: Any, _ent: ttk.Entry = ent) -> str:
            idx = self._axis_entry_order.index(_ent)
            nxt = self._axis_entry_order[(idx + 1) % len(self._axis_entry_order)]
            _focus_and_select(nxt)
            return "break"

        def _prev_entry(e: Any, _ent: ttk.Entry = ent) -> str:
            idx = self._axis_entry_order.index(_ent)
            prv = self._axis_entry_order[(idx - 1) % len(self._axis_entry_order)]
            _focus_and_select(prv)
            return "break"

        ent.bind("<Tab>", _next_entry)
        ent.bind("<Shift-Tab>", _prev_entry)
        ent.bind("<ISO_Left_Tab>", _prev_entry)

    # ── Speed / Homing / Status panel ─────────────────────────────────────

    def _build_speed_panel(self, parent: tk.Widget) -> None:
        """Build the speed control, homing buttons, and status block."""
        spdf = ttk.LabelFrame(parent, text="Speed", width=240, height=220)
        spdf.pack(side=tk.LEFT, fill=tk.Y, padx=2, pady=2)
        spdf.pack_propagate(False)

        # ── Speed header ──────────────────────────────────────────────────
        hdr = ttk.Frame(spdf)
        hdr.pack(fill="x", pady=(4, 2))
        ttk.Label(hdr, text="Speed:", font=("Segoe UI", 10, "bold")).pack(
            side=tk.LEFT, padx=(8, 2),
        )
        self._lbl_speed_val = ttk.Label(
            hdr, text=f"{self.speed_val.get():.1f}", width=6, anchor="e",
        )
        self._lbl_speed_val.pack(side=tk.LEFT, padx=(2, 8))

        # ── Max-speed mode buttons ────────────────────────────────────────
        mode_frame = ttk.Frame(spdf)
        mode_frame.pack(pady=(0, 4))
        for m in ("Low", "Mid", "High"):
            ttk.Button(
                mode_frame, text=m, width=6,
                style="SpeedMode.TButton",
                command=lambda mm=m: self._set_speed_mode(mm),
            ).pack(side=tk.LEFT, padx=2)

        # ── Speed slider (0..1000) ────────────────────────────────────────
        ttk.Scale(
            spdf, from_=0, to=1000, variable=self.speed_val,
            orient=tk.HORIZONTAL, length=220,
        ).pack(padx=8, pady=(0, 6))

        # ── Speed percent buttons ─────────────────────────────────────────
        pct_frame = ttk.Frame(spdf)
        pct_frame.pack(pady=(0, 0))
        for pct in (0, 20, 40, 50, 60, 80, 100):
            ttk.Button(
                pct_frame,
                text=f"{pct}%",
                width=3,
                style="SpeedPct.TButton",
                command=lambda p=pct: self._set_speed_percent(p),
            ).pack(side=tk.LEFT, padx=0)

        # ── Speed label auto-update ───────────────────────────────────────
        def _update_speed_label(*_: Any) -> None:
            self._lbl_speed_val.config(text=f"{self.speed_val.get():.1f}")

        self.speed_val.trace_add("write", _update_speed_label)

        # ── Homing / Control grid ─────────────────────────────────────────
        ctrl = ttk.LabelFrame(spdf, text="Homing / Control")
        ctrl.pack(fill=tk.X, padx=6, pady=(8, 4))

        btns = [
            ("Home $H",       lambda: self.app.send_now("$H"),                "homing_global"),
            ("Home $HX",      lambda: self.app.send_now("$HX"),               "homing_axis"),
            ("Home $HY",      lambda: self.app.send_now("$HY"),               "homing_axis"),
            ("Home $HZ",      lambda: self.app.send_now("$HZ"),               "homing_axis"),
            ("Home $HA",      lambda: self.app.send_now("$HA"),               "homing_axis"),
            ("Home $HB",      lambda: self.app.send_now("$HB"),               "homing_axis"),
            ("Home $HC",      lambda: self.app.send_now("$HC"),               "homing_axis"),
            ("Unlock $X",     lambda: self.app.send_now("$X"),                "other"),
            ("Zero",          self.manual_zero_current_pose,                   "other"),
            ("Go Home",       self.goto_home,                                 "other"),
            ("Home->Q",       self.enqueue_home,                              "other"),
            ("Gripper Open",  lambda: self.app.send_now("M3 S0"),             "other"),
            ("Gripper Close", lambda: self.app.send_now("M3 S1000"),          "other"),
            ("Pose 1",        lambda: self.app.send_now(
                "G1 X45 Y90 Z45 A0 B0 C0 F6000"),                            "other"),
            ("Pose 2",        lambda: self.app.send_now(
                "G1 X45 Y90 Z-45 A0 B0 C0 F6000"),                           "other"),
        ]

        self._homing_global_buttons.clear()
        self._homing_axis_buttons.clear()

        for i, (txt, cmd, kind) in enumerate(btns):
            r, c = divmod(i, 3)
            btn = ttk.Button(
                ctrl, text=txt, width=10,
                command=cmd, style="HomingCtrl.TButton",
            )
            btn.grid(row=r, column=c, padx=2, pady=1, sticky="ew")
            if kind == "homing_global":
                self._homing_global_buttons.append(btn)
            elif kind == "homing_axis":
                self._homing_axis_buttons.append(btn)

        for c_idx in range(3):
            ctrl.columnconfigure(c_idx, weight=1)

        # ── Status block ──────────────────────────────────────────────────
        self.lbl_status_block = tk.Label(
            spdf,
            textvariable=self.status_block_var,
            font=("Segoe UI", 10, "bold"),
            width=22,
            height=2,
            relief="groove",
            bg="#777777",
            fg="white",
            anchor="w",
            padx=8,
        )
        self.lbl_status_block.pack(fill="x", padx=8, pady=(4, 8))

        # ── Endstop indicators ────────────────────────────────────────────
        endstop_frame = ttk.Frame(spdf)
        endstop_frame.pack(fill="x", padx=8, pady=(0, 4))

        for ax in AXES:
            col = ttk.Frame(endstop_frame)
            col.pack(side=tk.LEFT, expand=True, padx=3)

            c = tk.Canvas(col, width=20, height=20, highlightthickness=0)
            oval = c.create_oval(3, 3, 17, 17, fill="#999999", outline="")
            c.pack()
            ttk.Label(col, text=ax, font=("Segoe UI", 8)).pack(pady=(0, 0))

            self.endstop_indicators[ax] = (c, oval, "unknown")

    # ──────────────────────────────────────────────────────────────────────
    #  Public interface for external updates
    # ──────────────────────────────────────────────────────────────────────

    def update_endstop_indicator(self, ax: str, state: str) -> None:
        """Update the endstop indicator for *ax*.

        *state* is one of ``"triggered"``, ``"clear"``, or ``"unknown"``.
        """
        colors = {
            "triggered": "#ff3333",
            "clear": "#33cc33",
            "unknown": "#999999",
        }
        color = colors.get(state, "#999999")

        for registry in (self.endstop_indicators, self.axis_endstop_icons):
            if ax in registry:
                canvas, oval, _old = registry[ax]
                canvas.itemconfigure(oval, fill=color)
                registry[ax] = (canvas, oval, state)

    def update_status_block(self, state: str) -> None:
        """Update the coloured status label."""
        state_colors = {
            "Idle": "#33cc33",
            "Run": "#3399ff",
            "Hold": "#ffcc00",
            "Alarm": "#ff3333",
            "Check": "#ff9900",
        }
        bg = state_colors.get(state, "#777777")
        self.status_block_var.set(f"Status: {state}")
        self.lbl_status_block.configure(bg=bg)

    def refresh_slider_limits(self) -> None:
        """Re-read manual limits and update all slider ranges + scale drawings."""
        for ax in AXES:
            lo, hi = self.get_manual_axis_limits(ax)
            if ax in self.axis_scales:
                self.axis_scales[ax].configure(from_=lo, to=hi)
            if ax in self._axis_scale_redraw:
                self._axis_scale_redraw[ax](lo, hi)
