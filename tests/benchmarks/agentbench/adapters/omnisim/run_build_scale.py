#!/usr/bin/env python3
"""Run one real OmniSim BuildScale oracle/replay pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from agentbench.adapters.omnisim.build_scale import run_level


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--level", required=True, type=int)
    parser.add_argument("--timeout", type=float, default=900.0)
    args = parser.parse_args()
    args.run_dir.mkdir(parents=True, exist_ok=True)
    verdict, facts = run_level(args.world, args.run_dir, args.level,
                               timeout_s=args.timeout)
    (args.run_dir / "verdict.json").write_text(
        json.dumps(verdict.as_dict(), indent=2) + "\n", encoding="utf-8")
    (args.run_dir / "facts.json").write_text(
        json.dumps(facts, indent=2, default=str) + "\n", encoding="utf-8")
    print(verdict.summary())
    return 0 if verdict.outcome == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
