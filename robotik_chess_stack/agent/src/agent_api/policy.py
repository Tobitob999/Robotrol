from __future__ import annotations

from typing import Dict, Optional


class PolicyModel:
    def __init__(self, enabled: bool = False):
        self.enabled = bool(enabled)

    def suggest(self, context: Dict) -> Optional[Dict]:
        if not self.enabled:
            return None
        return None
