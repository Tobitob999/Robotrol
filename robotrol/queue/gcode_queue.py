"""
G-Code Queue — extracted from Robotrol_FluidNC_v7_3.py (ExecuteApp).

Manages a list of G-Code lines with a background worker thread
that sends them sequentially, waiting for ACK between commands.
Supports pause (Feed Hold), resume, abort, and repeat.
"""

import threading
import time
from typing import Callable, Optional


DEFAULT_TIMEOUT = 30.0  # seconds per command


class GCodeQueue:
    """Headless G-Code queue with worker thread.

    Parameters
    ----------
    send_fn : callable(line: str) -> None
        Sends a single G-Code line to the controller.
    on_log : callable(msg: str) -> None, optional
        Logging callback.  Defaults to no-op.
    send_ctrl_x_fn : callable() -> None, optional
        Sends Ctrl+X (soft reset) on abort.  If *None*, abort only
        clears internal state without resetting the controller.
    timeout : float
        Per-command ACK timeout in seconds.
    """

    def __init__(
        self,
        send_fn: Callable[[str], None],
        on_log: Optional[Callable[[str], None]] = None,
        send_ctrl_x_fn: Optional[Callable[[], None]] = None,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        self._queue: list[str] = []
        self.running = False
        self.paused = False

        self._send = send_fn
        self._send_ctrl_x = send_ctrl_x_fn
        self._log = on_log or (lambda m: None)
        self._timeout = timeout

        # Threading primitives (mirror original worker logic)
        self._run_event = threading.Event()
        self._abort_event = threading.Event()
        self._awaiting_ack = False

        # Repeat support
        self.repeat_enabled = False
        self.repeat_times = 1
        self._repeat_count = 0

        # Worker thread — daemon so it dies with the process
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    # ------------------------------------------------------------------
    #  Public queue manipulation
    # ------------------------------------------------------------------

    def enqueue(self, line: str) -> None:
        """Append a single G-Code line to the queue."""
        g = line.strip()
        if not g:
            return
        self._queue.append(g)
        self._log(f"Queued: {g}")

    def enqueue_many(self, lines: list[str]) -> None:
        """Append multiple G-Code lines at once."""
        for ln in lines:
            self.enqueue(ln)

    def clear(self) -> None:
        """Remove all pending lines from the queue."""
        self._queue.clear()
        self._log("Queue cleared.")

    def is_empty(self) -> bool:
        return len(self._queue) == 0

    def count(self) -> int:
        return len(self._queue)

    @property
    def lines(self) -> list[str]:
        """Return a shallow copy of the current queue contents."""
        return list(self._queue)

    # ------------------------------------------------------------------
    #  Run control
    # ------------------------------------------------------------------

    def start_run(self) -> None:
        """Start (or restart) queue execution from the beginning."""
        if not self._queue:
            self._log("Queue is empty.")
            return
        self._abort_event.clear()
        self.paused = False
        self._repeat_count = 0
        self.running = True
        self._log("RUN started")
        self._run_event.set()

    def pause_run(self) -> None:
        """Pause execution (sends Feed Hold '!' to controller)."""
        self._send("!")
        self.paused = True
        self._log("Paused (Feed Hold sent)")

    def resume_run(self) -> None:
        """Resume after pause (sends Cycle Start '~')."""
        if not self.paused:
            self._log("No active pause.")
            return
        self._send("~")
        self.paused = False
        self._log("Resume (~) sent")

    def stop_abort(self) -> None:
        """Full abort: stop queue, reset events, optionally Ctrl+X."""
        self._log("STOP/ABORT")
        self._abort_event.set()
        self._run_event.clear()
        self.paused = False
        self.running = False
        self._awaiting_ack = False
        if self._send_ctrl_x:
            try:
                self._send_ctrl_x()
                self._log("Ctrl-X (reset) sent")
            except Exception as e:
                self._log(f"Abort error: {e}")
        self._log("System ready for new commands")

    # ------------------------------------------------------------------
    #  ACK handling — call from serial RX callback
    # ------------------------------------------------------------------

    def notify_ack(self) -> None:
        """Notify that the controller sent 'ok' (ACK)."""
        self._awaiting_ack = False

    def notify_error(self, msg: str = "") -> None:
        """Notify that the controller sent an error response."""
        self._log(f"RX error: {msg}")
        self._awaiting_ack = False

    # ------------------------------------------------------------------
    #  Worker thread (mirrors original ExecuteApp.worker)
    # ------------------------------------------------------------------

    def _worker(self) -> None:
        """Background thread for sequential queue execution."""
        while True:
            self._run_event.wait()
            if self._abort_event.is_set():
                time.sleep(0.05)
                continue
            if not self._queue:
                time.sleep(0.05)
                continue

            lines = list(self._queue)
            self._log(f"Starting program execution ({len(lines)} line(s))")
            self._awaiting_ack = False

            for i, g in enumerate(lines, start=1):
                # Pause loop
                while self.paused and not self._abort_event.is_set():
                    time.sleep(0.05)
                if self._abort_event.is_set() or not self._run_event.is_set():
                    break

                self._awaiting_ack = True
                self._log(f"[{i}/{len(lines)}] TX: {g}")

                try:
                    self._send(g)
                except Exception as e:
                    self._log(f"TX error: {e}")
                    self._awaiting_ack = False
                    continue

                # Wait for ACK with timeout
                deadline = time.time() + self._timeout
                while time.time() < deadline:
                    if self._abort_event.is_set() or not self._run_event.is_set():
                        self._awaiting_ack = False
                        break
                    if not self._awaiting_ack:
                        break
                    time.sleep(0.02)
                else:
                    self._log(f"Timeout on: {g}")
                    self._awaiting_ack = False

            # End / abort / repeat
            if self._abort_event.is_set():
                self._log("Program execution aborted")
                self._abort_event.clear()
                self._run_event.clear()
                self.paused = False
                self.running = False
                self._awaiting_ack = False
                continue

            if self.repeat_enabled and not self._abort_event.is_set():
                self._repeat_count += 1
                total = max(1, self.repeat_times)
                if self._repeat_count < total:
                    self._log(f"Repeating queue ({self._repeat_count}/{total})")
                    continue
                else:
                    self._log(f"Repeat finished ({self._repeat_count}/{total})")

            self._run_event.clear()
            self.paused = False
            self.running = False
            self._awaiting_ack = False
            self._log("Queue finished — ready for next start")
