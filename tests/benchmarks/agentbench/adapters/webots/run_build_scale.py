#!/usr/bin/env python3
"""Run one upstream-Webots BuildScale oracle/replay pair."""

import argparse
import json
from pathlib import Path

from agentbench.adapters.webots.build_scale import run_level


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--level", required=True, type=int)
    args = parser.parse_args()
    verdict, facts = run_level(args.world, args.run_dir, args.level)
    args.run_dir.mkdir(parents=True, exist_ok=True)
    (args.run_dir / "verdict.json").write_text(
        json.dumps(verdict.as_dict(), indent=2) + "\n", encoding="utf-8")
    (args.run_dir / "facts.json").write_text(
        json.dumps(facts, indent=2, default=str) + "\n", encoding="utf-8")
    print(verdict.summary())
    return 0 if verdict.outcome == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
