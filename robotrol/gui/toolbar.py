"""
Robotrol v2.0 — ToolbarFrame.

Extracted from Robotrol_FluidNC_v7_3.py (lines 649-785).
Top toolbar with port/baud selection, connect/disconnect, profile
dropdown, backend selector, and theme toggle.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass  # app protocol would go here

# ── Defaults ──────────────────────────────────────────────────────────────────

BAUD_RATES = [115200, 230400, 460800, 921600]
DEFAULT_BAUD = 115200

BACKENDS = ["FluidNC", "GRBL", "Custom"]
_BACKEND_MAP = {"FluidNC": "fluidnc", "GRBL": "grbl", "Custom": "custom"}


class ToolbarFrame(ttk.Frame):
    """Horizontal toolbar for serial connection and application settings.

    Parameters
    ----------
    parent : tk.Widget
        Parent container (usually the root window).
    app : object
        Main application coordinator.  Expected attributes/methods:
        - ``app.serial_client``   — SerialClient with connect/disconnect/list_ports
        - ``app.theme``           — ThemeManager with toggle()
        - ``app.profile_mgr``    — profile manager (load_profile, PROFILE_FILES)
        - ``app.apply_profile(name)`` — apply a named profile
        - ``app.on_connected()`` / ``app.on_disconnected()`` — optional callbacks
    """

    def __init__(self, parent: tk.Widget, app: object) -> None:
        super().__init__(parent)
        self.app = app

        self._build_widgets()
        self.refresh_ports()

    # ── Widget construction ───────────────────────────────────────────────

    def _build_widgets(self) -> None:
        """Create all toolbar widgets."""
        # --- Port selection ---
        ttk.Label(self, text="Port:").pack(side=tk.LEFT, padx=(4, 2))
        self.combo_ports = ttk.Combobox(self, width=18, state="readonly")
        self.combo_ports.pack(side=tk.LEFT, padx=(0, 10))

        # --- Baud rate ---
        ttk.Label(self, text="Baud:").pack(side=tk.LEFT, padx=(0, 2))
        self.combo_baud = ttk.Combobox(
            self, width=10, state="readonly",
            values=BAUD_RATES,
        )
        self.combo_baud.set(DEFAULT_BAUD)
        self.combo_baud.pack(side=tk.LEFT, padx=(0, 10))

        # --- Backend selection ---
        ttk.Label(self, text="Backend:").pack(side=tk.LEFT, padx=(0, 2))
        self.combo_backend = ttk.Combobox(
            self, width=10, state="readonly",
            values=BACKENDS,
        )
        self.combo_backend.set("FluidNC")
        self.combo_backend.pack(side=tk.LEFT, padx=(0, 10))

        # --- Action buttons ---
        ttk.Button(self, text="Refresh", width=8,
                   command=self.refresh_ports).pack(side=tk.LEFT, padx=2)
        ttk.Button(self, text="Connect",
                   command=self._on_connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(self, text="Disconnect",
                   command=self._on_disconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(self, text="Theme", width=12,
                   command=self._on_toggle_theme).pack(side=tk.LEFT, padx=4)

        # --- Profile dropdown ---
        ttk.Label(self, text="Config:").pack(side=tk.LEFT, padx=(4, 2))
        self._profile_var = tk.StringVar()
        self.combo_profile = ttk.Combobox(
            self, width=10, state="readonly",
            textvariable=self._profile_var,
        )
        self.combo_profile.pack(side=tk.LEFT, padx=(0, 6))
        self.combo_profile.bind("<<ComboboxSelected>>", self._on_profile_select)

        # --- Connection status label (right-aligned) ---
        self.conn_label = ttk.Label(
            self, text="Disconnected",
            foreground="#b71c1c", width=14, anchor="e",
        )
        self.conn_label.pack(side=tk.RIGHT, padx=8)

    # ── Public helpers ────────────────────────────────────────────────────

    def refresh_ports(self) -> None:
        """Scan serial ports and populate the port combobox."""
        try:
            ports = self.app.serial_client.list_ports()
        except AttributeError:
            ports = []
        self.combo_ports["values"] = ports
        if ports:
            self.combo_ports.set(ports[0])
        print("[INFO] Ports refreshed:", ports)

    def set_profile_list(self, names: list[str], current: str = "") -> None:
        """Populate the profile dropdown with *names* and optionally select *current*."""
        self.combo_profile["values"] = names
        if current:
            self._profile_var.set(current)

    def set_connected(self, connected: bool) -> None:
        """Update the connection status label."""
        if connected:
            self.conn_label.configure(text="Connected", foreground="#2e7d32")
        else:
            self.conn_label.configure(text="Disconnected", foreground="#b71c1c")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_connect(self) -> None:
        """Connect to the selected serial port."""
        port = self.combo_ports.get().strip()
        if not port:
            print("[WARN] No port selected")
            return

        baud = int(self.combo_baud.get())
        backend_key = _BACKEND_MAP.get(self.combo_backend.get(), "fluidnc")

        try:
            self.app.serial_client.set_backend(backend_key)
            self.app.serial_client.connect(port, baud)
            self.set_connected(True)
            print(f"[INFO] Connected to {port} @ {baud} Backend={backend_key}")

            # Notify app if callback exists
            if hasattr(self.app, "on_connected"):
                self.app.on_connected()
        except (OSError, ValueError) as exc:
            self.set_connected(False)
            print(f"[ERROR] Connect: {exc}")

    def _on_disconnect(self) -> None:
        """Disconnect from the serial port."""
        try:
            self.app.serial_client.disconnect()
            self.set_connected(False)
            print("[INFO] Disconnected")

            if hasattr(self.app, "on_disconnected"):
                self.app.on_disconnected()
        except OSError as exc:
            print(f"[ERROR] Disconnect: {exc}")

    def _on_toggle_theme(self) -> None:
        """Toggle between dark and light theme."""
        try:
            self.app.theme.toggle()
        except AttributeError:
            print("[WARN] No theme manager available on app")

    def _on_profile_select(self, _event: tk.Event | None = None) -> None:
        """Handle profile combobox selection."""
        name = self._profile_var.get().strip()
        if not name:
            return
        try:
            self.app.apply_profile(name)
        except (AttributeError, KeyError, ValueError) as exc:
            print(f"[WARN] Profile switch failed: {exc}")
