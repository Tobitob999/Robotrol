"""Gamepad configuration and control tab.

Wraps the gamepad_block_v3 module (GamepadConfigUI + attach_gamepad_tab)
into a v2-compatible tab class.
"""

from __future__ import annotations

import logging
import tkinter as tk
from tkinter import ttk
from typing import Any

logger = logging.getLogger(__name__)


class GamepadTab(ttk.Frame):
    """Tab that embeds the full gamepad configuration and jog control UI."""

    def __init__(self, parent: ttk.Widget, app: Any):
        super().__init__(parent)
        self.app = app
        self._stop_gamepad = None
        self._build_ui()

    def _build_ui(self) -> None:
        try:
            from gamepad_block_v3 import attach_gamepad_tab
            self._stop_gamepad = attach_gamepad_tab(
                self,
                self.app.client,
                self.app,
            )
        except ImportError:
            logger.warning("gamepad_block_v3 not found -- showing placeholder")
            self._build_placeholder()
        except AttributeError as exc:
            logger.warning("Gamepad init failed (missing app attribute): %s", exc)
            self._build_placeholder()

    def _build_placeholder(self) -> None:
        """Fallback UI when the gamepad module is unavailable."""
        ttk.Label(
            self,
            text="Gamepad Configuration",
            font=("Segoe UI", 14, "bold"),
        ).pack(pady=(30, 10))

        ttk.Label(
            self,
            text="gamepad_block_v3 module not found.\n"
                 "Connect a gamepad and ensure pygame is installed.",
        ).pack(pady=10)

        # Minimal placeholder controls
        ctl = ttk.LabelFrame(self, text="Status")
        ctl.pack(padx=20, pady=10, fill=tk.X)

        status_var = tk.StringVar(value="No gamepad detected")
        ttk.Label(ctl, textvariable=status_var).pack(padx=10, pady=6)

        def _try_detect() -> None:
            try:
                import pygame
                pygame.init()
                pygame.joystick.init()
                count = pygame.joystick.get_count()
                if count > 0:
                    js = pygame.joystick.Joystick(0)
                    js.init()
                    status_var.set(f"Detected: {js.get_name()}")
                else:
                    status_var.set("No gamepad detected")
                pygame.quit()
            except ImportError:
                status_var.set("pygame not installed")
            except RuntimeError as exc:
                status_var.set(f"Detection error: {exc}")

        ttk.Button(ctl, text="Detect Gamepad", command=_try_detect).pack(
            padx=10, pady=(0, 6),
        )

    def stop(self) -> None:
        """Stop the gamepad polling thread (call on app shutdown)."""
        if callable(self._stop_gamepad):
            try:
                self._stop_gamepad()
            except RuntimeError as exc:
                logger.warning("Error stopping gamepad: %s", exc)
