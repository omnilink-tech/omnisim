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

"""ablation.py -- the ABLATION HARNESS for structured RL training.

Runs a BASELINE config plus one-variable variations, each launched headless with the
deploy-prediction metric on, forecast + early-killed via deploy_curve (don't burn hours
on a config that won't reach target), and auto-logged to exp_ledger. Then prints the
ranked comparison so you can read off WHICH change helped -- the scientific method,
automated. Runs are sequential on 1 GPU; on H100 you'd parallelize the variations.

Config JSON:
  {
    "iters_budget": 3000, "wall_budget_s": 2400, "target": 0.85, "eval_every": 100,
    "baseline": { "AMP_MOTION": "squat", "PPO_LR": "1.5e-4", ... },   # env knobs
    "variations": [
      {"tag": "baseline", "override": {}},
      {"tag": "lstm",     "override": {"POLICY_ARCH": "lstm"}},
      {"tag": "disc_slow","override": {"DISC_LR": "3e-5"}}
    ]
  }

  python ablation.py run <config.json>
  python ablation.py rank            # (delegates to exp_ledger)
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import deploy_curve
import exp_ledger

ROOT = os.environ.get("OMNISIM_HOME", os.path.abspath(os.path.join(HERE, "..", "..", "..", "..")))
RUNNER = "projects/policies/research/mpc/foot_redesign/run_walk_rl.sh"
LOGDIR = os.path.join(ROOT, "_scratch", "foot_redesign")
PYMOD = "projects.policies.training.g1_amp:g1_amp_step"


def _kill():
    subprocess.run(["powershell.exe", "-NoProfile", "-Command",
                    "Get-CimInstance Win32_Process -Filter \"Name='omnisim-bin.exe'\" | "
                    "Where-Object { $_.CommandLine -like '*g1_walk_bigfoot*' } | "
                    "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"],
                   capture_output=True)
    time.sleep(2)


def _launch(tag, cfg):
    """Start a headless training run in the background; return the Popen."""
    kv = " ".join("%s=%s" % (k, v) for k, v in cfg.items())
    dur = cfg.get("_wall_budget_s", 3600)
    cmd = ("bash %s %d %s train headless %s OMNISIM_INENGINE_PYMOD=%s"
           % (RUNNER, dur, tag, kv, PYMOD))
    return subprocess.Popen(cmd, shell=True, cwd=ROOT,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def run(config_path):
    cfg = json.load(open(config_path))
    target = cfg.get("target", 0.85); iters_budget = cfg.get("iters_budget", 3000)
    wall_budget = cfg.get("wall_budget_s", 2400); eval_every = cfg.get("eval_every", 100)
    base = dict(cfg["baseline"]); base["EVAL_EVERY"] = str(eval_every)
    base.setdefault("PPO_ITERS", str(iters_budget + 500))
    for var in cfg["variations"]:
        tag = "abl_" + var["tag"]
        run_cfg = dict(base); run_cfg.update(var.get("override", {}))
        run_cfg["_wall_budget_s"] = wall_budget
        # unique policy file per run so they don't clobber (absolute: relative RES_POLICY = silent fresh start)
        run_cfg["RES_POLICY"] = os.path.join(
            os.path.abspath(os.environ.get("OMNISIM_HOME", ".")),
            "projects", "policies", "research", "rl_inengine", "runs", "%s.pt" % tag)
        log = os.path.join(LOGDIR, "%s_rl.txt" % tag)
        try:
            os.remove(log)
        except OSError:
            pass
        print("\n>>> [%s] launching  (%s)" % (tag, " ".join("%s=%s" % kv for kv in var.get("override", {}).items()) or "baseline"))
        _kill()
        proc = _launch(tag, run_cfg)
        t0 = time.time(); killed_reason = "budget"
        while True:
            time.sleep(30)
            fc = deploy_curve.forecast(log, target=target)
            it = fc.get("it_now", 0)
            if fc["status"] not in ("NO-DATA", "TOO-EARLY"):
                print("    it=%d surv=%.2f asympt=%s [%s]"
                      % (it, fc.get("cur", 0), fc.get("asymptote", "-"), fc["status"]))
            if fc["status"] == "KILL" and fc.get("n_evals", 0) >= 12:   # >=12 evals -> reliable asymptote
                killed_reason = "early-stop (forecast %s < target)" % fc.get("asymptote"); break
            if it >= iters_budget:
                killed_reason = "reached iters_budget"; break
            if time.time() - t0 > wall_budget:
                killed_reason = "reached wall_budget"; break
            if proc.poll() is not None:
                killed_reason = "process exited"; break
        _kill()
        exp_ledger.add(log, tag=tag, outcome=killed_reason)
    print("\n=== ablation complete ===")
    exp_ledger.rank()


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run"); r.add_argument("config")
    sub.add_parser("rank")
    a = ap.parse_args()
    if a.cmd == "run":
        run(a.config)
    else:
        exp_ledger.rank()


if __name__ == "__main__":
    main()
