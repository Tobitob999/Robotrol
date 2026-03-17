from __future__ import annotations
import tkinter as tk
from tkinter import ttk


class ChessVisionTab(ttk.Frame):
    """Thin wrapper that embeds ChessVisionUI from the chess module."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self._build_ui()

    def _build_ui(self):
        try:
            from robotrol.chess.vision_pipeline import detect_chess_state, load_config  # noqa: F401
            from robotrol.gui.tabs._chess_vision_inner import ChessVisionInner
            camera = getattr(self.app, "cam", None)
            logger = getattr(self.app, "log", None)
            inner = ChessVisionInner(self, camera_capture=camera, logger=logger)
            inner.pack(fill="both", expand=True)
        except ImportError as exc:
            ttk.Label(
                self, text=f"Chess vision not available: {exc}"
            ).pack(anchor="w", padx=6, pady=6)
        except RuntimeError as exc:
            ttk.Label(
                self, text=f"Chess vision init failed: {exc}"
            ).pack(anchor="w", padx=6, pady=6)
