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

"""check_deploy_solver -- lint RL/legged Newton deploy worlds for the MuJoCo-solver pin.

THE BUG THIS GUARDS: OmniSim's Newton backend has two solvers (g1_deploy_runtime.py:1273+):
the default GPU **XPBD**, and **SolverMuJoCo** (the MuJoCo contact model the RL trainers run).
A legged policy is trained under mujoco_warp; if its deploy world resolves to XPBD -- a DIFFERENT
engine -- the policy collapses. The MuJoCo solver is selected only when the world sets
`WorldInfo.newtonSolver "mujoco"`/"mujoco_warp" OR the launcher sets OMNISIM_NEWTON_FORCE_MUJOCO.
Many deploy worlds carry only `physicsBackend "newton"` and rely on the env -- so loading them
WITHOUT the run-script (GUI / harness / headless) silently falls to XPBD. This is the documented
"long-running G1 deploy gap" (OmNewtonBackend.cpp). The robust fix is self-describing worlds: pin
`newtonSolver "mujoco"` in the WorldInfo so the solver is correct regardless of how the world loads.

USAGE:
  python scripts/dev/check_deploy_solver.py                 # lint projects/policies/worlds (exit 1 on violations)
  python scripts/dev/check_deploy_solver.py --fix           # bake the missing pin into flagged worlds
  python scripts/dev/check_deploy_solver.py --json          # machine-readable
  python scripts/dev/check_deploy_solver.py path/to/*.wbt   # lint explicit worlds

Runtime companion: set OMNISIM_REQUIRE_MUJOCO_SOLVER=1 to make a deploy fail LOUD (not silently
fall to XPBD) at world load -- see g1_deploy_runtime.py.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# Legged / contact-critical robots whose policy is mujoco_warp-trained: their deploy worlds MUST
# run SolverMuJoCo, never XPBD. (Wheeled robots e.g. Husky drive fine on XPBD -- not flagged.)
ROBOT_TOKENS = ("g1", "h1", "valkyrie",
                "omniquad", "go2", "b2", "anymal")

NEWTON_RE = re.compile(r'physicsBackend\s+"newton"')
PIN_RE = re.compile(r'newtonSolver\s+"(mujoco|mujoco_warp)"')
URL_RE = re.compile(r'url\s+"([^"]+)"')
CTRL_RE = re.compile(r'controller\s+"([^"]+)"')
NAME_RE = re.compile(r'name\s+"([^"]+)"')
# WorldInfo block opener; captures its indent and the newline right after the brace.
WORLDINFO_RE = re.compile(r'(?:^|\n)([ \t]*)WorldInfo\s*\{[ \t]*(\r?\n)')


def _identifiers(text, path):
    """Tokens that name the robot: filename + URDF basenames + controller + name fields."""
    ids = [os.path.basename(path).lower()]
    for rx in (URL_RE, CTRL_RE, NAME_RE):
        ids += [os.path.basename(v).lower() for v in rx.findall(text)]
    return " ".join(ids)


def classify(path):
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        text = f.read()
    is_newton = bool(NEWTON_RE.search(text))
    has_pin = bool(PIN_RE.search(text))
    ids = _identifiers(text, path)
    is_legged = any(tok in ids for tok in ROBOT_TOKENS)
    if not is_newton:
        return "skip-not-newton", text
    if not is_legged:
        return "skip-not-legged", text
    if has_pin:
        return "ok-pinned", text
    return "missing", text


def apply_fix(path, text):
    """Insert `newtonSolver "mujoco"` as the first WorldInfo field. Idempotent; returns True on change."""
    if PIN_RE.search(text):
        return False
    m = WORLDINFO_RE.search(text)
    if not m:
        return False
    indent, nl = m.group(1) + "  ", m.group(2)
    at = m.end()
    new_text = text[:at] + f'{indent}newtonSolver "mujoco"{nl}' + text[at:]
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(new_text)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*",
                    help="world files/globs (default: projects/policies/worlds/*.wbt)")
    ap.add_argument("--fix", action="store_true",
                    help="bake `newtonSolver \"mujoco\"` into flagged worlds")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args()

    if args.paths:
        files = []
        for p in args.paths:
            files += glob.glob(p) if any(c in p for c in "*?[") else [p]
    else:
        # projects/rl/ was renamed projects/policies/ long ago; this default
        # pointed at the dead path, so the linter globbed nothing, reported
        # {"results": [], "missing": [], "fixed": []} and exited 0 on every
        # invocation -- a green that could not go red.
        default_dir = os.path.join(REPO, "projects", "policies", "worlds")
        files = sorted(glob.glob(os.path.join(default_dir, "*.omniworld"))
                       + glob.glob(os.path.join(default_dir, "*.wbt")))
        if not files:
            print("check_deploy_solver: no worlds found under %s" % default_dir,
                  file=sys.stderr)
            return 1

    results, missing, fixed = [], [], []
    for path in sorted(set(files)):
        if not os.path.isfile(path):
            continue
        status, text = classify(path)
        rel = os.path.relpath(path, REPO)
        if status == "missing":
            if args.fix and apply_fix(path, text):
                status, fixed_flag = "fixed", True
                fixed.append(rel)
            else:
                missing.append(rel)
        results.append({"world": rel, "status": status})

    if args.json:
        print(json.dumps({"results": results, "missing": missing, "fixed": fixed}, indent=2))
    else:
        order = {"missing": 0, "fixed": 1, "ok-pinned": 2, "skip-not-legged": 3, "skip-not-newton": 4}
        label = {"missing": "XPBD-RISK (no mujoco pin)", "fixed": "FIXED -> mujoco pin added",
                 "ok-pinned": "ok (mujoco pinned)", "skip-not-legged": "skip (not legged)",
                 "skip-not-newton": "skip (not newton)"}
        for r in sorted(results, key=lambda r: (order.get(r["status"], 9), r["world"])):
            if r["status"].startswith("skip"):
                continue
            mark = {"missing": "FAIL", "fixed": "FIXT", "ok-pinned": "OK  "}[r["status"]]
            print(f"  [{mark}] {r['world']:<52} {label[r['status']]}")
        n_legged = sum(1 for r in results if not r["status"].startswith("skip"))
        print(f"\n  {n_legged} legged Newton worlds | "
              f"{sum(1 for r in results if r['status']=='ok-pinned')} pinned | "
              f"{len(fixed)} fixed | {len(missing)} still at XPBD-risk")

    if missing:
        print(f"\nERROR: {len(missing)} legged Newton world(s) would resolve to XPBD. "
              f"Run with --fix to pin the MuJoCo solver.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
