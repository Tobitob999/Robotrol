from __future__ import annotations

import json
import sqlite3
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


SCHEMA = """
CREATE TABLE IF NOT EXISTS trials (
  trial_id TEXT PRIMARY KEY,
  ts REAL NOT NULL,
  context TEXT NOT NULL,
  theta_id TEXT NOT NULL,
  theta_json TEXT NOT NULL,
  outcome INTEGER NOT NULL,
  metrics_json TEXT,
  failure_code TEXT
);

CREATE TABLE IF NOT EXISTS theta_sets (
  theta_id TEXT PRIMARY KEY,
  context TEXT NOT NULL,
  theta_json TEXT NOT NULL,
  created_ts REAL NOT NULL,
  notes TEXT,
  performance_json TEXT
);

CREATE TABLE IF NOT EXISTS models (
  model_id TEXT PRIMARY KEY,
  type TEXT NOT NULL,
  path TEXT NOT NULL,
  created_ts REAL NOT NULL,
  metrics_json TEXT
);
"""


class Store:
    def __init__(self, path: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(self.path.as_posix(), check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.executescript(SCHEMA)
        self._conn.commit()

    def close(self) -> None:
        self._conn.close()

    def create_theta_set(self, theta_id: str, context: Dict[str, Any], theta: Dict[str, Any], notes: str = "") -> None:
        context_json = json.dumps(context, sort_keys=True)
        theta_json = json.dumps(theta, sort_keys=True)
        self._conn.execute(
            "INSERT OR REPLACE INTO theta_sets(theta_id, context, theta_json, created_ts, notes, performance_json) "
            "VALUES (?, ?, ?, ?, ?, ?)",
            (theta_id, context_json, theta_json, time.time(), notes, json.dumps({})),
        )
        self._conn.commit()

    def record_trial(
        self,
        trial_id: str,
        context: Dict[str, Any],
        theta_id: str,
        theta: Dict[str, Any],
        outcome: bool,
        metrics: Dict[str, Any],
        failure_code: Optional[str],
    ) -> None:
        context_json = json.dumps(context, sort_keys=True)
        theta_json = json.dumps(theta, sort_keys=True)
        metrics_json = json.dumps(metrics, sort_keys=True)
        self._conn.execute(
            "INSERT INTO trials(trial_id, ts, context, theta_id, theta_json, outcome, metrics_json, failure_code) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (trial_id, time.time(), context_json, theta_id, theta_json, int(outcome), metrics_json, failure_code),
        )
        self._conn.commit()
        self._update_theta_performance(theta_id, context_json)

    def _update_theta_performance(self, theta_id: str, context_json: str) -> None:
        cur = self._conn.execute(
            "SELECT SUM(outcome), COUNT(*) FROM trials WHERE theta_id=? AND context=?",
            (theta_id, context_json),
        )
        row = cur.fetchone()
        if not row or row[1] == 0:
            return
        successes, total = row[0] or 0, row[1]
        performance = {"successes": successes, "trials": total, "success_rate": successes / total}
        self._conn.execute(
            "UPDATE theta_sets SET performance_json=? WHERE theta_id=?",
            (json.dumps(performance, sort_keys=True), theta_id),
        )
        self._conn.commit()

    def get_trial_stats(self, context: Dict[str, Any]) -> Tuple[int, float]:
        context_json = json.dumps(context, sort_keys=True)
        cur = self._conn.execute(
            "SELECT SUM(outcome), COUNT(*) FROM trials WHERE context=?",
            (context_json,),
        )
        row = cur.fetchone()
        if not row or row[1] == 0:
            return 0, 0.0
        successes, total = row[0] or 0, row[1]
        return total, successes / total

    def get_best_theta(self, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        context_json = json.dumps(context, sort_keys=True)
        cur = self._conn.execute(
            """
            SELECT theta_id, theta_json, SUM(outcome) as successes, COUNT(*) as total
            FROM trials
            WHERE context=?
            GROUP BY theta_id
            ORDER BY (CAST(successes AS REAL) / total) DESC, total DESC
            LIMIT 1
            """,
            (context_json,),
        )
        row = cur.fetchone()
        if not row:
            return None
        theta_id, theta_json, successes, total = row
        return {"theta_id": theta_id, "theta": json.loads(theta_json), "success_rate": (successes or 0) / total}

    def get_latest_theta(self, context: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        context_json = json.dumps(context, sort_keys=True)
        cur = self._conn.execute(
            "SELECT theta_id, theta_json FROM theta_sets WHERE context=? ORDER BY created_ts DESC LIMIT 1",
            (context_json,),
        )
        row = cur.fetchone()
        if not row:
            return None
        theta_id, theta_json = row
        return {"theta_id": theta_id, "theta": json.loads(theta_json)}
