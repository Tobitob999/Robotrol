"""
CLI History — extracted from Robotrol_FluidNC_v7_3.py (ExecuteApp._init_cli_history).

Simple ring-buffer for command-line history with up/down navigation.
"""


class CLIHistory:
    """Ring-buffer for CLI command history.

    Parameters
    ----------
    max_size : int
        Maximum number of entries to keep.  Oldest entries are
        discarded when the buffer is full.
    """

    def __init__(self, max_size: int = 100):
        self.history: list[str] = []
        self.index: int = -1
        self.max_size: int = max_size

    def add(self, cmd: str) -> None:
        """Add a command to history (deduplicates consecutive repeats)."""
        if not cmd:
            return
        # Original logic: skip if identical to last entry
        if self.history and self.history[-1] == cmd:
            self.reset_index()
            return
        self.history.append(cmd)
        # Enforce max size
        if len(self.history) > self.max_size:
            self.history = self.history[-self.max_size:]
        self.reset_index()

    def up(self) -> str | None:
        """Navigate one entry back (older).  Returns the command or None."""
        if not self.history:
            return None
        if self.index > 0:
            self.index -= 1
        elif self.index == -1:
            # First press of up: jump to last entry
            self.index = len(self.history) - 1
        return self.history[self.index]

    def down(self) -> str | None:
        """Navigate one entry forward (newer).  Returns the command or None if at end."""
        if not self.history:
            return None
        if self.index < len(self.history) - 1:
            self.index += 1
            return self.history[self.index]
        else:
            # Past the end — clear input (original behaviour)
            self.index = len(self.history)
            return None

    def reset_index(self) -> None:
        """Reset the navigation index to 'past the end' (next up() gives last entry)."""
        self.index = len(self.history)
