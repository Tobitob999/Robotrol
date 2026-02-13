from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np

from .store import Store


THETA_FIELDS = [
    "dx_mm",
    "dy_mm",
    "dz_pick_mm",
    "dz_place_mm",
    "yaw_deg",
    "v_approach",
    "v_lift",
    "v_place",
    "dwell_close_ms",
    "dwell_release_ms",
]


@dataclass
class OptimizerConfig:
    population: int = 8
    sigma: float = 0.6
    mu: int = 4
    min_sigma: float = 0.1
    max_sigma: float = 1.5
    seed: int = 123
    exploit_after: int = 10


class CMAESOptimizer:
    """
    Simplified CMA-ES style optimizer.
    We sample from a Gaussian around a moving mean and adapt sigma based on outcomes.
    """

    def __init__(self, clamps: Dict[str, Tuple[float, float]], config: OptimizerConfig):
        self.clamps = clamps
        self.config = config
        self.rng = np.random.default_rng(config.seed)
        self.state: Dict[str, Dict[str, np.ndarray | float | int]] = {}

    def _context_key(self, context: Dict) -> str:
        return json.dumps(context, sort_keys=True)

    def _init_state(self, key: str) -> None:
        mean = np.zeros(len(THETA_FIELDS), dtype=float)
        sigma = float(self.config.sigma)
        self.state[key] = {"mean": mean, "sigma": sigma, "trials": 0}

    def _clamp(self, theta: Dict[str, float]) -> Dict[str, float]:
        clamped: Dict[str, float] = {}
        for field in THETA_FIELDS:
            value = float(theta.get(field, 0.0))
            if field in self.clamps:
                low, high = self.clamps[field]
                value = max(low, min(high, value))
            clamped[field] = value
        return clamped

    def next_theta(self, context: Dict, store: Store) -> Dict[str, Dict]:
        key = self._context_key(context)
        if key not in self.state:
            self._init_state(key)

        total_trials, _ = store.get_trial_stats(context)
        if total_trials >= self.config.exploit_after:
            best = store.get_best_theta(context)
            if best:
                return {"theta_id": best["theta_id"], "theta": best["theta"], "source": "best"}

        mean = self.state[key]["mean"]
        sigma = float(self.state[key]["sigma"])
        sample = self.rng.normal(mean, sigma)
        theta = {field: float(sample[idx]) for idx, field in enumerate(THETA_FIELDS)}
        theta = self._clamp(theta)
        theta_id = str(uuid.uuid4())
        store.create_theta_set(theta_id, context, theta, notes="sampled")
        return {"theta_id": theta_id, "theta": theta, "source": "sampled"}

    def report(self, context: Dict, theta: Dict[str, float], outcome: bool) -> None:
        key = self._context_key(context)
        if key not in self.state:
            self._init_state(key)
        mean = self.state[key]["mean"]
        sigma = float(self.state[key]["sigma"])

        vec = np.array([float(theta.get(field, 0.0)) for field in THETA_FIELDS], dtype=float)
        lr = 0.2
        if outcome:
            mean = mean + lr * (vec - mean)
            sigma = max(self.config.min_sigma, sigma * 0.9)
        else:
            mean = mean - lr * 0.1 * (vec - mean)
            sigma = min(self.config.max_sigma, sigma * 1.05)
        self.state[key]["mean"] = mean
        self.state[key]["sigma"] = sigma
        self.state[key]["trials"] = int(self.state[key]["trials"]) + 1
