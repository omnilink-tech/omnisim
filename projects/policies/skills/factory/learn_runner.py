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

"""Skill Factory LEARN RUNNER -- births a new arm skill live, as a subprocess.

The OmniArm 6 chat bridge invokes this when the user asks the arm to LEARN something
("learn to toss the cube into the bin"). The runner executes the real pipeline --
design a reference motion, validate it against the robot's actual limits, TRAIN
(a real optimization with a success metric, minutes on this machine), certify
against a bar -- and streams progress as JSON-lines on stdout. The output is a
joint trajectory (+ gripper events) the bridge replays.

Event contract (stdout, one JSON object per line):
  {"ev":"stage","stage":"design|validate|train|certify","state":"running|pass|fail",
   "detail":"<short human line with the real numbers>","pct":<0-100>}
  {"ev":"log","line":"..."}                                  (sparingly)
  {"ev":"done","ok":true,"verb":"toss","exec":"<abs>/learned_skill.json","summary":"..."}
On failure: the failing stage emits state:"fail" with an honest detail, then
  {"ev":"done","ok":false,"verb":...,"summary":...}   (no "exec").

Usage:
  python projects/policies/skills/factory/learn_runner.py --skill omniarm6_toss \
      --events jsonl [--param bin_x=1.3] [--param bin_y=0.0] [--param seed=0]
  python projects/policies/skills/factory/learn_runner.py --skill list

Recipes live in sibling modules (one class per recipe; see recipe_toss.py --
the fully-implemented reference). Adding a recipe = a small class + a REGISTRY
entry below.
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
# factory -> skills -> policies -> projects -> repo root
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
DEFAULT_OUT_ROOT = os.path.join(HERE, "out")

# When executed as a script this module is "__main__"; recipes do
# `from learn_runner import StageFailure`, which would otherwise re-import a
# SECOND module object whose StageFailure is a different class (and the
# except-clause below would silently miss it). Alias the running instance.
sys.modules.setdefault("learn_runner", sys.modules[__name__])

# Nominal end-of-stage progress, used when a stage fails without its own pct.
STAGE_PCT = {"design": 10, "validate": 18, "train": 90, "certify": 98}

# ── Recipe registry ──────────────────────────────────────────────────
# status "ready": module/cls importable and fully implemented.
# status "stub":  declared so the catalogue (and the chat UX) can show what is
#                 COMING; invoking it fails honestly at startup.
REGISTRY = {
    "omniarm6_toss": {
        "status": "ready",
        "module": "recipe_toss",
        "cls": "TossRecipe",
        "verb": "toss",
        "robot": "omniarm6",
        "summary": "Throw the held cube into a bin (beyond kinematic reach) -- "
                   "designed swing + feasibility gates + REINFORCE over swing "
                   "parameters under domain randomization.",
        "params": {
            "bin_x": {"default": 1.3, "unit": "m", "doc": "bin centre x from the arm base"},
            "bin_y": {"default": 0.0, "unit": "m", "doc": "bin centre y (aims by base yaw)"},
            "seed": {"default": 0, "unit": "", "doc": "RNG seed (training + certify draws)"},
            "dt_ms": {"default": 16.0, "unit": "ms", "doc": "output trajectory sample period "
                      "(16 ms = the chat world's basicTimeStep)"},
            "iters": {"default": 200, "unit": "", "doc": "dev override: training iterations "
                      "(default is the demo-certified config, ~3 min)"},
        },
    },
    "omniarm6_reach_pose": {
        "status": "stub",
        "module": "recipe_reach",       # not implemented yet -- honest stub
        "cls": "ReachRecipe",
        "verb": "reach",
        "robot": "omniarm6",
        "summary": "Learn a validated reach-to-pose trajectory (stub -- not implemented).",
        "params": {},
    },
}


class StageFailure(Exception):
    """A pipeline gate genuinely failed. detail is the honest human line."""

    def __init__(self, stage: str, detail: str):
        super().__init__(detail)
        self.stage = stage
        self.detail = detail


class Emitter:
    """Streams the fixed event contract on stdout (jsonl) or as readable text."""

    def __init__(self, mode: str = "jsonl"):
        self.mode = mode

    def _write(self, obj: dict):
        if self.mode == "jsonl":
            sys.stdout.write(json.dumps(obj) + "\n")
        else:
            if obj["ev"] == "stage":
                sys.stdout.write("[%5.1f%%] %-8s %-7s %s\n" % (
                    obj.get("pct", 0.0), obj["stage"], obj["state"].upper(), obj["detail"]))
            elif obj["ev"] == "log":
                sys.stdout.write("        | %s\n" % obj["line"])
            else:
                sys.stdout.write("DONE ok=%s %s\n" % (obj.get("ok"), obj.get("summary", "")))
        sys.stdout.flush()

    def stage(self, stage: str, state: str, detail: str, pct: float):
        self._write({"ev": "stage", "stage": stage, "state": state,
                     "detail": detail, "pct": round(float(pct), 1)})

    def log(self, line: str):
        self._write({"ev": "log", "line": line})

    def done(self, ok: bool, verb: str, exec_path: str | None, summary: str):
        obj = {"ev": "done", "ok": bool(ok), "verb": verb}
        if exec_path:
            obj["exec"] = os.path.abspath(exec_path)
        obj["summary"] = summary
        self._write(obj)


def _parse_params(pairs):
    """--param k=v (repeatable) -> dict with int/float coercion."""
    out = {}
    for p in pairs or []:
        if "=" not in p:
            raise SystemExit(f"--param must be key=value, got {p!r}")
        k, v = p.split("=", 1)
        try:
            out[k] = int(v)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def _list_skills():
    cat = []
    for name, meta in REGISTRY.items():
        cat.append({
            "skill": name,
            "verb": meta["verb"],
            "robot": meta["robot"],
            "status": meta["status"],
            "summary": meta["summary"],
            "params": {k: v["default"] for k, v in meta.get("params", {}).items()},
        })
    print(json.dumps({"skills": cat}, indent=2))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Skill Factory learn runner")
    ap.add_argument("--skill", required=True,
                    help="recipe name from the registry, or 'list' for the catalogue")
    ap.add_argument("--events", choices=["jsonl", "text"], default="jsonl",
                    help="stdout event format (the bridge uses jsonl)")
    ap.add_argument("--param", action="append", default=[],
                    help="recipe parameter, key=value (repeatable)")
    ap.add_argument("--out", default=None,
                    help="output directory (default: <factory>/out/<skill>/)")
    args = ap.parse_args(argv)

    if args.skill == "list":
        return _list_skills()

    emit = Emitter(args.events)
    meta = REGISTRY.get(args.skill)
    if meta is None:
        emit.done(False, "?", None,
                  f"unknown skill {args.skill!r}; known: {sorted(REGISTRY)}")
        return 2
    verb = meta["verb"]
    if meta["status"] != "ready":
        emit.done(False, verb, None,
                  f"skill {args.skill!r} is a stub (status={meta['status']}) -- not implemented yet")
        return 2

    params = _parse_params(args.param)
    out_dir = os.path.abspath(args.out or os.path.join(DEFAULT_OUT_ROOT, args.skill))
    os.makedirs(out_dir, exist_ok=True)

    sys.path.insert(0, HERE)
    try:
        mod = importlib.import_module(meta["module"])
        recipe = getattr(mod, meta["cls"])(params=params, emit=emit, out_dir=out_dir)
    except Exception as exc:  # import/param errors are pre-pipeline
        traceback.print_exc(file=sys.stderr)
        emit.done(False, verb, None, f"recipe failed to initialize: {exc}")
        return 2

    t0 = time.perf_counter()
    try:
        # The recipe drives the four stages itself (train needs mid-stage
        # progress events), raising StageFailure when a gate genuinely fails.
        skill_path = recipe.run()
    except StageFailure as sf:
        emit.stage(sf.stage, "fail", sf.detail, STAGE_PCT.get(sf.stage, 0))
        emit.done(False, verb, None, f"{sf.stage} failed: {sf.detail}")
        return 1
    except Exception as exc:
        traceback.print_exc(file=sys.stderr)
        emit.done(False, verb, None, f"runner error: {type(exc).__name__}: {exc}")
        return 3

    emit.done(True, verb, skill_path,
              recipe.summary_line(wall_s=time.perf_counter() - t0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
