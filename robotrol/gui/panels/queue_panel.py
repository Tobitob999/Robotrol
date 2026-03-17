"""Queue Panel — G-Code queue display, CLI input, and serial log.

Extracted from Robotrol_FluidNC_v7_3.py (lines 4216-4268 + CLI area).
Provides queue management buttons, a command-line entry with history,
and a scrollable log widget for serial output.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk, filedialog
from typing import TYPE_CHECKING

from robotrol.queue.cli_history import CLIHistory

if TYPE_CHECKING:
    from robotrol.queue.gcode_queue import GCodeQueue


class QueuePanel(ttk.Frame):
    """G-Code queue panel with CLI input, log output, and run controls.

    Parameters
    ----------
    parent : tk.Widget
        Parent widget.
    app : object
        Application root.  Expected attributes:
        - ``app.queue`` — :class:`~robotrol.queue.gcode_queue.GCodeQueue`
        - ``app.serial`` — serial client with ``send(line)`` method
    """

    def __init__(self, parent: tk.Widget, app: object) -> None:
        super().__init__(parent)
        self.app = app
        self._cli_history = CLIHistory(max_size=200)

        # ── Repeat state ─────────────────────────────────────────────
        self._repeat_var = tk.BooleanVar(value=False)
        self._repeat_times_var = tk.StringVar(value="1")
        self._repeat_count_var = tk.StringVar(value="0")

        self._build_ui()
        self._bind_cli_keys()

    # ------------------------------------------------------------------
    #  UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        # ── Command Line ─────────────────────────────────────────────
        cli_frame = ttk.Frame(self)
        cli_frame.pack(fill=tk.X, padx=2, pady=(2, 2))

        ttk.Label(cli_frame, text="Command Line (G-Code):").pack(side=tk.LEFT)
        self.entry_cmd = ttk.Entry(cli_frame)
        self.entry_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(cli_frame, text="To Queue", command=self._cli_add_to_queue).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(cli_frame, text="Send Now", command=self._cli_send_now).pack(
            side=tk.LEFT, padx=3
        )

        # ── Log window ───────────────────────────────────────────────
        self.txt_log = tk.Text(self, height=4, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=False, padx=6, pady=4)

        # ── Queue listbox ────────────────────────────────────────────
        self.listbox_queue = tk.Listbox(self, height=4)
        self.listbox_queue.pack(fill=tk.X, padx=6, pady=(0, 4))

        # ── Buttons ──────────────────────────────────────────────────
        btn_frame = ttk.Frame(self)
        btn_frame.pack(fill=tk.X, padx=6, pady=(2, 8))

        ttk.Button(btn_frame, text="Load (To Queue)", command=self._load_program).pack(
            side=tk.LEFT
        )
        ttk.Button(btn_frame, text="Save Queue", command=self._save_program).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="Clear Queue", command=self._clear_queue).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="Run", command=self._start_run).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="Pause (!)", command=self._pause_run).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="Resume (~)", command=self._resume_run).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="Stop / Abort", command=self._stop_abort).pack(
            side=tk.LEFT, padx=6
        )

        # Repeat controls
        ttk.Checkbutton(
            btn_frame, text="Repeat", variable=self._repeat_var
        ).pack(side=tk.LEFT, padx=6)
        ttk.Label(btn_frame, text="x").pack(side=tk.LEFT, padx=(2, 0))
        ttk.Entry(
            btn_frame, textvariable=self._repeat_times_var, width=4, justify="center"
        ).pack(side=tk.LEFT, padx=(2, 8))
        ttk.Label(btn_frame, text="Runs:").pack(side=tk.LEFT, padx=2)
        self.lbl_repeat_status = ttk.Label(
            btn_frame, textvariable=self._repeat_count_var
        )
        self.lbl_repeat_status.pack(side=tk.LEFT)

    # ------------------------------------------------------------------
    #  CLI history key bindings
    # ------------------------------------------------------------------

    def _bind_cli_keys(self) -> None:
        self.entry_cmd.bind("<Return>", self._on_enter)
        self.entry_cmd.bind("<Up>", self._on_history_up)
        self.entry_cmd.bind("<Down>", self._on_history_down)

    def _on_enter(self, _event: tk.Event) -> str:
        """Submit current CLI input to queue."""
        self._cli_add_to_queue()
        return "break"

    def _on_history_up(self, _event: tk.Event) -> str:
        cmd = self._cli_history.up()
        if cmd is not None:
            self.entry_cmd.delete(0, tk.END)
            self.entry_cmd.insert(0, cmd)
        return "break"

    def _on_history_down(self, _event: tk.Event) -> str:
        cmd = self._cli_history.down()
        self.entry_cmd.delete(0, tk.END)
        if cmd is not None:
            self.entry_cmd.insert(0, cmd)
        return "break"

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def log(self, msg: str) -> None:
        """Append a message to the log text widget."""
        self.txt_log.configure(state=tk.NORMAL)
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.see(tk.END)
        self.txt_log.configure(state=tk.DISABLED)

    def update_queue(self, items: list[str]) -> None:
        """Replace the listbox contents with *items*."""
        self.listbox_queue.delete(0, tk.END)
        for item in items:
            self.listbox_queue.insert(tk.END, item)

    def clear_log(self) -> None:
        """Erase all log text."""
        self.txt_log.configure(state=tk.NORMAL)
        self.txt_log.delete("1.0", tk.END)
        self.txt_log.configure(state=tk.DISABLED)

    # ------------------------------------------------------------------
    #  Queue property helper
    # ------------------------------------------------------------------

    @property
    def _queue(self) -> GCodeQueue:
        return self.app.queue  # type: ignore[attr-defined]

    # ------------------------------------------------------------------
    #  CLI actions
    # ------------------------------------------------------------------

    def _cli_add_to_queue(self) -> None:
        cmd = self.entry_cmd.get().strip()
        if not cmd:
            return
        self._cli_history.add(cmd)
        self._queue.enqueue(cmd)
        self.entry_cmd.delete(0, tk.END)
        self._refresh_listbox()

    def _cli_send_now(self) -> None:
        cmd = self.entry_cmd.get().strip()
        if not cmd:
            return
        self._cli_history.add(cmd)
        try:
            self.app.serial.send(cmd)  # type: ignore[attr-defined]
            self.log(f"> {cmd}")
        except AttributeError:
            self.log(f"[ERR] serial not available")
        self.entry_cmd.delete(0, tk.END)

    # ------------------------------------------------------------------
    #  Queue button actions
    # ------------------------------------------------------------------

    def _start_run(self) -> None:
        self._sync_repeat_settings()
        self._queue.start_run()

    def _pause_run(self) -> None:
        self._queue.pause_run()

    def _resume_run(self) -> None:
        self._queue.resume_run()

    def _stop_abort(self) -> None:
        self._queue.stop_abort()

    def _clear_queue(self) -> None:
        self._queue.clear()
        self._refresh_listbox()

    def _load_program(self) -> None:
        path = filedialog.askopenfilename(
            title="Load G-Code program",
            filetypes=[("G-Code", "*.gcode *.nc *.ngc *.txt"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f if ln.strip()]
            self._queue.enqueue_many(lines)
            self.log(f"Loaded {len(lines)} lines from {path}")
            self._refresh_listbox()
        except OSError as e:
            self.log(f"[ERR] Load failed: {e}")

    def _save_program(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save Queue",
            defaultextension=".gcode",
            filetypes=[("G-Code", "*.gcode *.nc"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                for line in self._queue.lines:
                    f.write(line + "\n")
            self.log(f"Saved {self._queue.count()} lines to {path}")
        except OSError as e:
            self.log(f"[ERR] Save failed: {e}")

    # ------------------------------------------------------------------
    #  Internal helpers
    # ------------------------------------------------------------------

    def _refresh_listbox(self) -> None:
        self.update_queue(self._queue.lines)

    def _sync_repeat_settings(self) -> None:
        self._queue.repeat_enabled = self._repeat_var.get()
        try:
            self._queue.repeat_times = int(self._repeat_times_var.get())
        except ValueError:
            self._queue.repeat_times = 1
