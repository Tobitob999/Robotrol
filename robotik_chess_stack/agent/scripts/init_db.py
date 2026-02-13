#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from agent_api.store import Store


def parse_args():
    parser = argparse.ArgumentParser(description="Initialize agent database")
    parser.add_argument("--config", default="agent/config/agent.yaml", help="Path to agent.yaml")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text(encoding="utf-8"))
    db_path = cfg["database"]["path"]
    Store(db_path).close()
    print(f"Initialized DB at {db_path}")


if __name__ == "__main__":
    main()
