from __future__ import annotations

import argparse
import time

from .config import load_config
from .state_machine import EdgeCore


def parse_args():
    parser = argparse.ArgumentParser(description="Edge core runner")
    parser.add_argument("--config", default="edge/config/edge.yaml", help="Path to edge.yaml")
    parser.add_argument("--once", action="store_true", help="Run one agent cycle and exit")
    parser.add_argument("--skill", default="", help="Run a single skill (pick/place/capture/reset)")
    parser.add_argument("--square", default="", help="Square for skill, e.g. a2")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)
    core = EdgeCore(cfg)
    core.start()
    try:
        if args.skill:
            context = {"square": args.square} if args.square else {}
            result = core.execute_skill(args.skill, context, {})
            print(result)
            return
        if args.once:
            core.run_once_from_agent()
            return
        while True:
            core.run_once_from_agent()
            time.sleep(0.2)
    finally:
        core.stop()


if __name__ == "__main__":
    main()
