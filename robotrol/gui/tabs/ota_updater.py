from __future__ import annotations
import tkinter as tk
from tkinter import ttk


class OtaUpdaterTab(ttk.Frame):
    """Wrapper that embeds FluidNCUpdaterFrame for OTA config/update."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self._build_ui()

    def _build_ui(self):
        try:
            from robotrol.fluidnc_updater_v2 import FluidNCUpdaterFrame
        except ImportError as exc:
            ttk.Label(
                self, text=f"OTA updater not available: {exc}"
            ).pack(anchor="w", padx=6, pady=6)
            return

        default_ip = getattr(self.app, "default_ip", "192.168.25.149")
        self.updater_frame = FluidNCUpdaterFrame(self, default_ip=default_ip)

        # Disable controls when backend does not support OTA
        client = getattr(self.app, "client", None)
        if client and not getattr(client, "supports_ota", True):
            for child in self.winfo_children():
                try:
                    child.configure(state="disabled")
                except tk.TclError:
                    pass

        # Limit height to avoid excessive stretching
        self.update_idletasks()
        self.configure(height=400)
        self.pack_propagate(False)
