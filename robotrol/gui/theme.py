"""
Robotrol v2.0 — ThemeManager.

Extracted from Robotrol_FluidNC_v7_3.py (lines 571-647).
Handles dark/light theme toggling using tkinter ttk styles.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robotrol.gui.toolbar import ToolbarFrame  # noqa: F401


# ── Theme colour palettes ────────────────────────────────────────────────────

_LIGHT = {
    "bg": "#f0f0f0",
    "fg": "black",
    "btn_bg": "#e6e6e6",
    "btn_fg": "black",
    "btn_active": "#d0d0d0",
    "btn_pressed": "#c0c0c0",
    "tab_bg": "#e6e6e6",
    "tab_selected": "#ffffff",
    "entry_bg": "white",
    "entry_fg": "black",
}

_DARK = {
    "bg": "#2b2b2b",
    "fg": "#e0e0e0",
    "btn_bg": "#3c3f41",
    "btn_fg": "#ffffff",
    "btn_active": "#505354",
    "btn_pressed": "#606364",
    "tab_bg": "#3a3a3a",
    "tab_selected": "#4a4a4a",
    "entry_bg": "#3c3f41",
    "entry_fg": "#ffffff",
}


class ThemeManager:
    """Manages dark/light theme toggling for the application.

    Parameters
    ----------
    parent : tk.Tk
        The root window whose background is updated on toggle.
    app : object
        The main application coordinator (unused currently, kept for
        consistency with other GUI components).
    """

    def __init__(self, parent: tk.Tk, app: object) -> None:
        self._root = parent
        self.app = app
        self.is_dark: bool = False

        # Grab or create the ttk Style object
        self._style = ttk.Style(self._root)
        try:
            self._style.theme_use("clam")
        except tk.TclError as exc:
            print(f"[Warn] Could not set 'clam' theme: {exc}")

        # Apply initial light theme
        self._apply_palette(_LIGHT)

    # ── Public API ────────────────────────────────────────────────────────

    def toggle(self) -> None:
        """Switch between dark and light theme."""
        if self.is_dark:
            self._apply_palette(_LIGHT)
            self.is_dark = False
        else:
            self._apply_palette(_DARK)
            self.is_dark = True

    @property
    def style(self) -> ttk.Style:
        """Return the underlying ttk.Style instance."""
        return self._style

    # ── Internal ──────────────────────────────────────────────────────────

    def _apply_palette(self, p: dict[str, str]) -> None:
        """Apply a colour palette to all standard ttk widget styles."""
        s = self._style

        s.configure(".", background=p["bg"], foreground=p["fg"])
        s.configure("TLabel", background=p["bg"], foreground=p["fg"])
        s.configure("TFrame", background=p["bg"])

        s.configure("TButton", background=p["btn_bg"], foreground=p["btn_fg"])
        s.map(
            "TButton",
            background=[("active", p["btn_active"]), ("pressed", p["btn_pressed"])],
            foreground=[("active", p["btn_fg"]), ("pressed", p["btn_fg"])],
        )

        s.configure("TNotebook", background=p["bg"], foreground=p["fg"])
        s.configure("TNotebook.Tab", background=p["tab_bg"], foreground=p["fg"])
        s.map(
            "TNotebook.Tab",
            background=[("selected", p["tab_selected"])],
            foreground=[("selected", p["btn_fg"])],
        )

        s.configure("TEntry", fieldbackground=p["entry_bg"], foreground=p["entry_fg"])
        s.configure("Horizontal.TScale", background=p["bg"])

        self._root.configure(bg=p["bg"])
