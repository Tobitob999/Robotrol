from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, Optional


class TrialLogger:
    def __init__(self, log_path: str, image_ring_dir: Optional[str] = None, image_ring_max: int = 0):
        self.log_path = Path(log_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.image_ring_dir = Path(image_ring_dir) if image_ring_dir else None
        self.image_ring_max = int(image_ring_max) if image_ring_max else 0
        if self.image_ring_dir:
            self.image_ring_dir.mkdir(parents=True, exist_ok=True)

    def log_trial(self, record: Dict[str, Any]) -> None:
        record = dict(record)
        record.setdefault("timestamp", time.time())
        line = json.dumps(record, ensure_ascii=True)
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")

    def log_image(self, name: str, data: bytes) -> Optional[Path]:
        if not self.image_ring_dir or self.image_ring_max <= 0:
            return None
        filename = f"{int(time.time() * 1000)}_{name}.jpg"
        path = self.image_ring_dir / filename
        path.write_bytes(data)
        self._trim_ring()
        return path

    def _trim_ring(self) -> None:
        if not self.image_ring_dir or self.image_ring_max <= 0:
            return
        files = sorted(self.image_ring_dir.glob("*.jpg"), key=lambda p: p.stat().st_mtime)
        if len(files) <= self.image_ring_max:
            return
        for victim in files[: len(files) - self.image_ring_max]:
            try:
                victim.unlink()
            except OSError:
                pass
