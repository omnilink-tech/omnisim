#!/usr/bin/env python3
# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run every catalogue brief against its robot, and check every step.

    python scripts/dev/run_agent_catalogue.py
    python scripts/dev/run_agent_catalogue.py --only husky
    python scripts/dev/run_agent_catalogue.py --surface quadruped

Briefs are grouped BY WORLD, so each engine starts once and serves all the
briefs for that robot -- 20 launches rather than 100. A launch costs about
25 s, so grouping is most of the runtime.

A brief passes only if every step passes, and every brief ends with a
safety probe that must not actuate. So a green sweep is 100 end-to-end
assertions that the command surface does the right thing on every robot in
the tree, not 100 scripts that ran without crashing.

Engines are launched and tree-killed BY PID, never by image name: a
parallel lane may have an omnisim-bin of its own, and killing it reads as a
crash in that session's log.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import time
from collections import defaultdict

ROOT = pathlib.Path(__file__).resolve().parents[2]
CAT = ROOT / "agents/production/catalogue"
sys.path.insert(0, str(CAT))
from _driver import run_brief, wait_up  # noqa: E402

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:  # pragma: no cover
    pass


def load_briefs(only: str = "", surface: str = ""):
    by_world = defaultdict(list)
    for path in sorted(CAT.glob("*/brief.json")):
        brief = json.loads(path.read_text(encoding="utf-8"))
        if only and only not in path.parent.name:
            continue
        if surface and brief.get("surface") != surface:
            continue
        by_world[(brief["world"], brief["bridge_port"])].append(path)
    return by_world


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="", help="substring filter on the name")
    ap.add_argument("--surface", default="",
                    choices=["", "mobile", "arm", "quadruped"])
    ap.add_argument("--duration", type=int, default=900)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    by_world = load_briefs(args.only, args.surface)
    if not by_world:
        print("no briefs matched")
        return 1
    total = sum(len(v) for v in by_world.values())
    print(f"{total} briefs across {len(by_world)} worlds\n")

    logdir = pathlib.Path(tempfile.gettempdir()) / "agent_catalogue"
    logdir.mkdir(parents=True, exist_ok=True)
    results = []

    for (world, port), paths in sorted(by_world.items()):
        stem = pathlib.Path(world).stem
        print(f"=== {stem} ({len(paths)} briefs) ===", flush=True)
        proc = subprocess.Popen(
            [sys.executable, "-m", "omnisim", "run-headless", world,
             "--duration", str(args.duration)],
            cwd=str(ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env={**os.environ, "OMNISIM_URDF_USE_SENSORS": "1",
                 "OMNISIM_LOG_PATH": str(logdir / f"{stem}.log")})
        try:
            if not wait_up(port):
                for p in paths:
                    results.append({"brief": p.parent.name, "ok": False,
                                    "steps": 0,
                                    "failures": ["bridge never came up"]})
                print("  !! bridge never came up", flush=True)
                continue
            for p in paths:
                r = run_brief(p, verbose=False)
                r["brief"] = p.parent.name
                results.append(r)
                mark = "OK " if r["ok"] else "!! "
                print(f"  {mark}{r['brief']:<30} {r['steps']} steps"
                      + ("" if r["ok"] else "  " + "; ".join(r["failures"])[:70]),
                      flush=True)
        finally:
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)]
                           if os.name == "nt" else ["kill", "-9", str(proc.pid)],
                           capture_output=True)
            time.sleep(2)

    bad = [r for r in results if not r["ok"]]
    steps = sum(r["steps"] for r in results)
    print(f"\n=== {len(results) - len(bad)}/{len(results)} briefs clean "
          f"({steps} checked steps) ===")
    for r in bad:
        print(f"  FAIL {r['brief']}: {'; '.join(r['failures'])[:100]}")
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(results, indent=2),
                                          encoding="utf-8")
        print(f"wrote {args.out}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
