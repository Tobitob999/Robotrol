"""
Robotrol v2.0 — StatusBlock widget.

Extracted from Robotrol_FluidNC_v7_3.py (lines 4046-4063, 5578-5597).
Compact colour-coded machine-state indicator.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

# ── State-to-colour mapping ──────────────────────────────────────────────────

_STATE_COLOURS: dict[str, str] = {
    "idle":       "#2e7d32",  # green
    "run":        "#1565c0",  # blue
    "hold":       "#f9a825",  # yellow
    "alarm":      "#b71c1c",  # red
    "error":      "#b71c1c",  # red (same as alarm)
    "home":       "#00897b",  # turquoise
    "disconnect": "#616161",  # grey
}

_NEUTRAL_COLOUR = "#777777"


class StatusBlock(ttk.Frame):
    """Compact colour-coded status display for the machine state.

    The label background changes to reflect the current controller state:
    Idle = green, Run = blue, Hold = yellow, Alarm/Error = red,
    Home = turquoise, Disconnect = grey.

    Parameters
    ----------
    parent : tk.Widget
        Parent container.
    app : object
        Main application coordinator (kept for consistency; not used
        directly by StatusBlock).
    """

    def __init__(self, parent: tk.Widget, app: object) -> None:
        super().__init__(parent)
        self.app = app

        self._state_var = tk.StringVar(value="Status: -")

        # Use a plain tk.Label (not ttk) so bg/fg work reliably on all
        # platforms.
        self._label = tk.Label(
            self,
            textvariable=self._state_var,
            font=("Segoe UI", 10, "bold"),
            width=22,
            height=2,
            relief="groove",
            bg=_NEUTRAL_COLOUR,
            fg="white",
            anchor="w",
            padx=8,
        )
        self._label.pack(fill="x", padx=8, pady=(4, 8))

    # ── Public API ────────────────────────────────────────────────────────

    def update_state(self, state: str) -> None:
        """Update the displayed state text and background colour.

        Parameters
        ----------
        state : str
            Raw state string from the controller (e.g. ``"Idle"``,
            ``"Run:0"``, ``"Hold:1"``, ``"Alarm"``).
        """
        colour = self._resolve_colour(state)
        self._state_var.set(f"Status: {state}")
        # Yellow gets black text for readability; everything else white.
        fg = "black" if colour == _STATE_COLOURS["hold"] else "white"
        self._label.configure(bg=colour, fg=fg)

    @property
    def current_state_text(self) -> str:
        """Return the current state variable value (e.g. ``'Status: Idle'``)."""
        return self._state_var.get() or ""

    # ── Internal ──────────────────────────────────────────────────────────

    @staticmethod
    def _resolve_colour(state: str) -> str:
        """Map a state string to its display colour."""
        st = (state or "-").lower()
        for keyword, colour in _STATE_COLOURS.items():
            if keyword in st:
                return colour
        return _NEUTRAL_COLOUR
