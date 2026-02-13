from __future__ import annotations

import json
import subprocess
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import requests


@dataclass
class ChessConfig:
    mode: str
    online_url: str
    online_token: str
    stockfish_path: str


class ChessEngine:
    def __init__(self, cfg: ChessConfig, timeout_s: float = 5.0):
        self.cfg = cfg
        self.timeout_s = timeout_s

    def move(self, fen: str) -> Tuple[str, str]:
        if self.cfg.mode == "online":
            try:
                return self._move_online(fen), "online"
            except Exception:
                return self._move_stockfish(fen), "stockfish"
        return self._move_stockfish(fen), "stockfish"

    def _move_online(self, fen: str) -> str:
        headers = {}
        if self.cfg.online_token:
            headers["Authorization"] = f"Bearer {self.cfg.online_token}"
        resp = requests.post(
            self.cfg.online_url,
            json={"fen": fen},
            headers=headers,
            timeout=self.timeout_s,
        )
        resp.raise_for_status()
        data = resp.json()
        uci = data.get("uci") or data.get("move") or data.get("bestmove")
        if not uci:
            raise RuntimeError("Online chess API did not return a move")
        return uci

    def _move_stockfish(self, fen: str) -> str:
        proc = subprocess.Popen(
            [self.cfg.stockfish_path],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if proc.stdin is None or proc.stdout is None:
            raise RuntimeError("Failed to start stockfish")

        def send(cmd: str):
            proc.stdin.write(cmd + "\n")
            proc.stdin.flush()

        def read_until(token: str, timeout: float = 2.0) -> None:
            deadline = time.time() + timeout
            while time.time() < deadline:
                line = proc.stdout.readline().strip()
                if token in line:
                    return
            raise TimeoutError(f"Timeout waiting for {token}")

        send("uci")
        read_until("uciok")
        send("isready")
        read_until("readyok")
        send(f"position fen {fen}")
        send("go movetime 1000")
        bestmove = ""
        deadline = time.time() + 3.0
        while time.time() < deadline:
            line = proc.stdout.readline().strip()
            if line.startswith("bestmove"):
                bestmove = line.split()[1]
                break
        send("quit")
        proc.kill()
        if not bestmove:
            raise RuntimeError("Stockfish did not return bestmove")
        return bestmove
