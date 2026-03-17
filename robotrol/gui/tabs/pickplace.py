from __future__ import annotations
import tkinter as tk
from tkinter import ttk


class PickPlaceTab(ttk.Frame):
    """Thin wrapper that embeds the full PickPlaceTab from the inner module."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self._build_ui()

    def _build_ui(self):
        try:
            from robotrol.gui.tabs._pickplace_inner import PickPlaceTab as _Inner

            camera = getattr(self.app, "cam", None)
            logger = getattr(self.app, "log", None)
            execute_app = self.app

            inner = _Inner(
                self,
                logger=logger,
                camera_capture=camera,
                execute_app=execute_app,
            )
            inner.pack(fill="both", expand=True)
        except ImportError as exc:
            ttk.Label(
                self, text=f"Pick & Place not available: {exc}"
            ).pack(anchor="w", padx=6, pady=6)
        except RuntimeError as exc:
            ttk.Label(
                self, text=f"Pick & Place init failed: {exc}"
            ).pack(anchor="w", padx=6, pady=6)
