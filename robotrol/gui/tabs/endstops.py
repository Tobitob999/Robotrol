from __future__ import annotations
import tkinter as tk
from tkinter import ttk

AXES = ["X", "Y", "Z", "A", "B", "C"]


class EndstopsTab(ttk.Frame):
    """Endstop limit editor — per-axis min/max with load/save to profile."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.endstop_vars: dict[str, tuple[tk.StringVar, tk.StringVar]] = {}
        self._build_ui()

    def _build_ui(self):
        wrap = ttk.LabelFrame(self, text="Endstop Deltas (relative to zero)")
        wrap.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        ttk.Label(
            wrap,
            text="Edit min/max deltas relative to zero (deg).",
        ).pack(anchor="w", padx=6, pady=(4, 8))

        grid = ttk.Frame(wrap)
        grid.pack(anchor="nw", padx=6, pady=(0, 8))

        ttk.Label(grid, text="Axis").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Label(grid, text="Min").grid(row=0, column=1, padx=4, pady=2, sticky="w")
        ttk.Label(grid, text="Max").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        ttk.Label(grid, text="From current").grid(
            row=0, column=3, columnspan=2, padx=4, pady=2, sticky="w"
        )

        for row_idx, ax in enumerate(AXES, start=1):
            lo, hi = self._effective_limits(ax)
            vmin = tk.StringVar(value=f"{lo:.3f}")
            vmax = tk.StringVar(value=f"{hi:.3f}")
            self.endstop_vars[ax] = (vmin, vmax)

            ttk.Label(grid, text=ax, width=3).grid(
                row=row_idx, column=0, padx=4, pady=2, sticky="w"
            )
            ent_min = ttk.Entry(grid, textvariable=vmin, width=10, justify="right")
            ent_min.grid(row=row_idx, column=1, padx=4, pady=2)
            ent_min.bind("<Return>", lambda _e: self._apply())

            ent_max = ttk.Entry(grid, textvariable=vmax, width=10, justify="right")
            ent_max.grid(row=row_idx, column=2, padx=4, pady=2)
            ent_max.bind("<Return>", lambda _e: self._apply())

            ttk.Button(
                grid, text="Min<-Now", width=8,
                command=lambda a=ax: self._set_from_current(a, "min"),
            ).grid(row=row_idx, column=3, padx=4, pady=2)

            ttk.Button(
                grid, text="Max<-Now", width=8,
                command=lambda a=ax: self._set_from_current(a, "max"),
            ).grid(row=row_idx, column=4, padx=4, pady=2)

        btn_row = ttk.Frame(wrap)
        btn_row.pack(anchor="w", padx=6, pady=(0, 6))

        ttk.Button(btn_row, text="Apply", command=self._apply).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Save", command=self._save).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Load", command=self._load).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Reset Defaults", command=self._reset).pack(side=tk.LEFT, padx=4)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _effective_limits(self, axis: str) -> tuple[float, float]:
        """Return (min, max) for *axis* from the current profile, or defaults."""
        section = self.app.get_profile_section("endstops")
        if section and axis in section:
            entry = section[axis]
            return float(entry.get("min", -180.0)), float(entry.get("max", 180.0))
        return -180.0, 180.0

    def _gather_data(self) -> dict:
        """Collect current var values into a dict suitable for profile storage."""
        data: dict = {}
        for ax, (vmin, vmax) in self.endstop_vars.items():
            try:
                lo = float(vmin.get())
            except ValueError:
                lo = -180.0
            try:
                hi = float(vmax.get())
            except ValueError:
                hi = 180.0
            data[ax] = {"min": lo, "max": hi}
        return data

    def _set_from_current(self, axis: str, which: str):
        """Set min or max to the current axis position reported by the app."""
        try:
            val = float(self.app.axis_positions.get(axis, 0.0))
        except (ValueError, TypeError, AttributeError):
            val = 0.0
        vmin, vmax = self.endstop_vars.get(axis, (None, None))
        if which == "min" and vmin is not None:
            vmin.set(f"{val:.3f}")
        elif which == "max" and vmax is not None:
            vmax.set(f"{val:.3f}")
        self._apply()

    def _apply(self):
        """Push current endstop values into the running app (no persistent save)."""
        data = self._gather_data()
        self.app.set_profile_section("endstops", data)

    def _save(self):
        """Persist endstop limits to the profile."""
        data = self._gather_data()
        self.app.set_profile_section("endstops", data)

    def _load(self):
        """Reload endstop limits from the profile into the UI vars."""
        section = self.app.get_profile_section("endstops")
        if not section:
            return
        for ax, (vmin, vmax) in self.endstop_vars.items():
            entry = section.get(ax, {})
            vmin.set(f"{float(entry.get('min', -180.0)):.3f}")
            vmax.set(f"{float(entry.get('max', 180.0)):.3f}")

    def _reset(self):
        """Reset all axes to default limits (-180 / +180)."""
        for _ax, (vmin, vmax) in self.endstop_vars.items():
            vmin.set(f"{-180.000:.3f}")
            vmax.set(f"{180.000:.3f}")
        self._apply()
