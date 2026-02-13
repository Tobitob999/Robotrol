from __future__ import annotations


class Gripper:
    def __init__(self, enabled: bool = False):
        self.enabled = bool(enabled)

    def open(self) -> None:
        if not self.enabled:
            return

    def close(self) -> None:
        if not self.enabled:
            return
