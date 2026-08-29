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

"""Docking probe: one 4-cell alternating organism, one free cell placed on
its approach axis, high charge so the ecology recruits at once. Exercises the
supervisor's approach -> run-in -> lock -> verify -> recruit path in isolation
(the reef runs put the free cell 2-3 m away and behind, and 180 s was not
enough to see the end of the manoeuvre).

  python projects/metazoa/probe_dock.py [--ahead 0.9] [--lateral 0.0] [--epoch-s 120]
"""
import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metazoa as MZ                                 # noqa: E402
from mz import worldgen as W, scene as S             # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ahead", type=float, default=0.9, help="free cell distance ahead of the head")
    ap.add_argument("--lateral", type=float, default=0.0, help="free cell lateral offset")
    ap.add_argument("--behind", action="store_true", help="put the free cell BEHIND its own tail face (go-around case)")
    ap.add_argument("--epoch-s", type=float, default=120.0)
    ap.add_argument("--seed", type=int, default=3)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    mods = MZ.load_modules(True, note=print)
    lineages = [{"id": "L0", "genome": MZ.random_genome(mods.get("organism") if mods else None, rng),
                 "bodyplan": {"target_length": 8, "dock_rotation_pattern": [0, 1], "branch_rule": "none"}}]
    reef = MZ.build_reef(lineages, 5, 10.0, 0, rng, mods, n_free=1, seed_len=4, note=print)
    MZ.check_conserved(reef)

    # Re-place by hand: organism spine along +x with the head at x = 0
    # (tail -> head = index 0..3), free cell ahead of the head.
    org = reef["organisms"][0]
    members = org["members"]
    n = len(members)
    pitch = MZ.CELL_LEN + MZ.DOCK_GAP
    for k, cid in enumerate(members):
        c = reef["cells"][cid]
        c["pos"][0], c["pos"][1] = -(n - 1 - k) * pitch, 0.0
        c["yaw"] = 0.0
        c["roll"] = math.pi / 2.0 if k >= n - 2 else 0.0        # head rudder PAIR (measured 2026-08-29)
        c["dock_rotation"] = 1 if k >= n - 2 else 0
    free = [c for c in reef["cells"] if c["organism"] is None and not c["parked"]][0]
    # TAIL DOCKING (P2 redesign): the organism backs into the free cell, whose
    # NOSE face (+x of its own frame at yaw 0) must face the organism's tail.
    # The organism's tail is at x = -(n-1)*pitch; the free cell sits `ahead`
    # metres behind it, nose toward +x. --behind puts it in FRONT of the head
    # instead (the organism must turn around first).
    tail_x = -(n - 1) * pitch
    free["pos"][0] = tail_x - args.ahead - 0.06 if not args.behind else args.ahead + 0.06
    free["pos"][1] = args.lateral
    free["yaw"] = 0.0 if not args.behind else math.pi
    free["roll"] = 0.0
    free["dock_rotation"] = 0
    for c in reef["cells"]:
        c["charge_wh"] = 8.4                              # 70 %: recruit at once

    cfg = {"arena": 10.0, "n_patches": 5, "epoch_s": args.epoch_s, "watch": False, "epoch": 0,
           "cells": 5, "organisms": 1, "free_cells": 1, "seed": args.seed, "dim": 1.0,
           "time_scale": 20.0, "controller": MZ.CONTROLLER}
    MZ.write_inputs(reef, cfg)
    W.write_world(reef["cells"], MZ.WORLD, scene_lines=S.scene_lines(10.0, 5, MZ.CONTROLLER),
                  controller=MZ.CONTROLLER, rollers="v3", substeps=4, arena=10.0)
    print("probe: organism %s cells %s at x<=0, free cell %d at (%.2f, %.2f)"
          % (org["id"], members, free["id"], free["pos"][0], free["pos"][1]))
    try:
        os.remove(os.path.join(MZ.RUN, "world.log"))     # the supervisor appends; keep this run's events alone
    except OSError:
        pass
    MZ.run_epoch(0, int(args.epoch_s * 1.9) + 45, log=print)
    tele = json.load(open(os.path.join(MZ.RUN, "telemetry.json"), encoding="utf-8"))
    print("\nresult:", "recruits", tele["recruits"], "dock", tele["dock_stats"], "welds", tele["welds_held"])
    om = tele["organisms_measured"]
    for m in (om.values() if isinstance(om, dict) else om):
        d = m.get("dock")
        if d:
            print("  dock:", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()
                              if k in ("state", "phase", "dist_goal", "along", "lateral", "sep", "axis_err", "attempts")})
    print("\nevents:")
    try:
        for ln in open(os.path.join(MZ.RUN, "world.log"), encoding="utf-8"):
            if "t=" not in ln:
                print("  " + ln.rstrip()[:150])
    except OSError:
        pass


if __name__ == "__main__":
    main()
