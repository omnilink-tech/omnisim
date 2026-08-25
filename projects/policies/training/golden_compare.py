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

"""Compare two golden-trajectory dumps -- the BUG-vs-CHAOS verdict for deploy.

A golden dump is the (qpos, qvel) trace of the first N deploy ticks, recorded
by the recipe when OMNISIM_GOLDEN_DUMP=<out.npz> is set (see g1_walk_recipe's
deploy tick). Two runs of the SAME machine + stack + seed must agree to float
tolerance over that horizon; two runs on DIFFERENT machines agree early and
then drift apart at a rate set by the system's chaos, not by any bug.

Verdicts (magnitude + shape of the divergence is the diagnosis -- same
philosophy as docs/developer/closed-loop-chaos-diagnostic.md):

  IDENTICAL    bitwise equal. CPU-deterministic path, same machine.
  MATCH        max |dq| stays under --eps for the whole horizon.
  SOLVER-BAND  |dq| exceeds eps but stays under --band for the whole horizon.
               This is the GPU solver's own run-to-run nondeterminism (warp
               atomics reduce in nondeterministic order), measured live at
               max 8.5e-6 over 400 ticks for two same-machine same-seed
               decent-walker runs (2026-07-17). Not a defect.
  CHAOS        exceeds --band only after --early ticks: agreed at the start,
               then drifted. Expected across GPUs/OSes; fix with robustness.
  BUG          exceeds --band within the first --early ticks: the two runs did
               not start in the same state -- env/seed/stack mismatch, NOT
               floating-point noise. Diff the env fingerprints first.

Usage:
    python golden_compare.py a.npz b.npz [--eps 1e-6] [--band 1e-3] [--early 20]
Exit: 0 IDENTICAL/MATCH/SOLVER-BAND, 2 CHAOS, 3 BUG, 4 unusable inputs.
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--eps", type=float, default=1e-6,
                    help="per-coordinate tolerance for MATCH (default 1e-6)")
    ap.add_argument("--band", type=float, default=1e-3,
                    help="GPU-solver run-to-run nondeterminism ceiling; only "
                         "divergence beyond this is BUG/CHAOS (default 1e-3)")
    ap.add_argument("--early", type=int, default=20,
                    help="band exceedance before this tick classifies as BUG "
                         "(default 20)")
    args = ap.parse_args(argv)

    da, db = np.load(args.a, allow_pickle=False), np.load(args.b, allow_pickle=False)
    for k in ("qpos", "qvel"):
        if k not in da or k not in db:
            print(f"unusable: '{k}' missing"); return 4
    ma = json.loads(str(da["meta"])) if "meta" in da else {}
    mb = json.loads(str(db["meta"])) if "meta" in db else {}
    for k in sorted(set(ma) | set(mb)):
        if ma.get(k) != mb.get(k):
            print(f"META MISMATCH {k}: {ma.get(k)!r} vs {mb.get(k)!r} "
                  f"-- these dumps are not comparable apples-to-apples")

    qa, qb = da["qpos"], db["qpos"]
    if qa.shape[1] != qb.shape[1]:
        print(f"unusable: qpos width differs ({qa.shape[1]} vs {qb.shape[1]}) "
              f"-- different robot/world"); return 4
    n = min(len(qa), len(qb))
    if len(qa) != len(qb):
        print(f"note: comparing first {n} ticks (lengths {len(qa)} vs {len(qb)})")
    d = np.abs(qa[:n] - qb[:n]).max(axis=1)          # per-tick max |dq|

    if not d.any():
        print(f"IDENTICAL over {n} ticks (bitwise)"); return 0
    first = int(np.argmax(d > args.eps)) if (d > args.eps).any() else None
    if first is None:
        print(f"MATCH over {n} ticks (max |dq| = {d.max():.3e} < eps {args.eps:g})")
        return 0

    # divergence exists: report its shape, then classify by MAGNITUDE first
    q1, q2, q3 = (int(round(n * f)) - 1 for f in (0.25, 0.5, 1.0))
    print(f"first tick over eps({args.eps:g}): {first}   "
          f"|dq| at 25/50/100% of horizon: {d[q1]:.3e} / {d[q2]:.3e} / {d[q3]:.3e}")
    if d.max() <= args.band:
        print(f"VERDICT: SOLVER-BAND -- max |dq| = {d.max():.3e} stays under the "
              f"band ({args.band:g}) for the whole horizon. This is the GPU "
              f"solver's own run-to-run nondeterminism, not a defect.")
        return 0
    burst = int(np.argmax(d > args.band))
    if burst < args.early:
        print(f"VERDICT: BUG -- |dq| exceeds the band ({args.band:g}) at tick "
              f"{burst} (< {args.early}): the two runs did not start in the same "
              f"state. Diff the env fingerprints (stack/[sys] packages, binary "
              f"sha, seeds) before touching physics.")
        return 3
    print(f"VERDICT: CHAOS -- runs stay in-band for {burst} ticks then drift "
          f"beyond it. This is floating-point chaos amplified by an unstable "
          f"plant, expected across GPUs/OSes. If the drift bothers a demo, the "
          f"fix is policy robustness (DR), not parity-hunting.")
    return 2


if __name__ == "__main__":
    sys.exit(main())
