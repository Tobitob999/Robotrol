from __future__ import annotations
import tkinter as tk
from tkinter import ttk


COMMAND_REFERENCE = """\
 FluidNC / G-code command reference
====================================

 Machine control
-----------------
$X                  - Unlock after alarm
$H                  - Home all axes
$HX / $HY / $HZ     - Home specific axis
$$                  - Show current parameters (settings)
$G                  - Show active G-code modes
$I                  - Firmware/system information
!                   - Feed hold (pause)
~                   - Resume
Ctrl+X ()          - Software reset / emergency stop

 Motion commands
-----------------
G90                 - Absolute coordinates
G91                 - Relative coordinates
G92 X0 Y0 Z0 A0 B0  - Set current position as zero
G0 X10              - Rapid move to X10
G1 X10 F500         - Move to X10 with feedrate 500
G4 P2               - Dwell for 2 seconds

 Axis and limit configuration
------------------------------
$130..$134          - Max travel for X..B (soft limits)
$120..$124          - Accelerations (mm/s2)
$110..$114          - Max feedrate per axis (mm/min)
$20 / $21           - Enable soft/hard limits
$N / $N+            - Configure startup commands

  Tool / gripper / spindle
---------------------------
M3 S0               - Open gripper / spindle on (S=0)
M3 S1000            - Close gripper / full speed
M4                  - Spindle counter-clockwise
M5                  - Stop spindle

 Diagnostics and status
------------------------
?                   - Current status (MPos, endstops)
$#                  - Show coordinate systems
$Help               - Help / available commands
$Startup/Show       - Show startup file
$Erase/All          - Erase all saved settings
$CD / $Dir          - Browse file system (older versions)
$PrintConfig        - Show currently loaded YAML

 Misc
-------
Ctrl-X              - Restart / reset
$Report/State       - Print current machine status
$Report/Startup     - Show startup file loaded at boot
"""


class CommandsTab(ttk.Frame):
    """Read-only G-code / FluidNC command reference."""

    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self._build_ui()

    def _build_ui(self):
        frame_text = ttk.Frame(self)
        frame_text.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        text_widget = tk.Text(frame_text, wrap="word", height=6)
        text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(frame_text, orient="vertical", command=text_widget.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_widget.config(yscrollcommand=scrollbar.set)

        text_widget.insert("1.0", COMMAND_REFERENCE)
        text_widget.configure(state="disabled")
