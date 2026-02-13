from __future__ import annotations

import time
from typing import Callable, Dict, Tuple


class Verifier:
    def __init__(
        self,
        occupancy_fn: Callable[[], Tuple[Dict[str, bool], float]],
        retries: int = 2,
        delay_s: float = 0.5,
        min_confidence: float = 0.0,
    ):
        self.occupancy_fn = occupancy_fn
        self.retries = int(retries)
        self.delay_s = float(delay_s)
        self.min_confidence = float(min_confidence)

    def verify_square(self, square: str, expected_occupied: bool) -> Tuple[bool, str]:
        for _ in range(self.retries + 1):
            occ, confidence = self.occupancy_fn()
            if confidence < self.min_confidence:
                time.sleep(self.delay_s)
                continue
            if square in occ and occ[square] == expected_occupied:
                return True, ""
            time.sleep(self.delay_s)
        return False, "VISION_LOW_CONF"

    def verify_pick(self, square: str) -> Tuple[bool, str]:
        return self.verify_square(square, expected_occupied=False)

    def verify_place(self, square: str) -> Tuple[bool, str]:
        return self.verify_square(square, expected_occupied=True)
