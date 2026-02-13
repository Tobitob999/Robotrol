from __future__ import annotations

import json
from typing import Dict

import requests


class AgentClient:
    def __init__(self, base_url: str, psk: str, psk_header: str = "X-PSK", timeout_s: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.psk = psk
        self.psk_header = psk_header
        self.timeout_s = float(timeout_s)

    def _headers(self) -> Dict[str, str]:
        return {self.psk_header: self.psk}

    def get_next(self, context: Dict) -> Dict:
        resp = requests.get(
            f"{self.base_url}/v1/learn/next",
            params={"context": json.dumps(context, sort_keys=True)},
            headers=self._headers(),
            timeout=self.timeout_s,
        )
        resp.raise_for_status()
        return resp.json()

    def report_trial(self, theta_id: str, context: Dict, theta: Dict, result: Dict) -> None:
        payload = {
            "theta_id": theta_id,
            "context": context,
            "theta": theta,
            "outcome": result.get("success", False),
            "metrics": result,
            "failure_code": result.get("failure_code"),
        }
        resp = requests.post(
            f"{self.base_url}/v1/learn/report",
            json=payload,
            headers=self._headers(),
            timeout=self.timeout_s,
        )
        resp.raise_for_status()

    def chess_move(self, fen: str) -> Dict:
        resp = requests.post(
            f"{self.base_url}/v1/chess/move",
            json={"fen": fen},
            headers=self._headers(),
            timeout=self.timeout_s,
        )
        resp.raise_for_status()
        return resp.json()
