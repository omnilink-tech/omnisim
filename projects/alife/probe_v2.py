#!/usr/bin/env python3
"""Gate A for alife v2: build the 8-creature probe world (DESIGN_v2.md).

Eight random 2-pair x 2-segment genomes (seed 3) on a 14 m arena, driven by
the `terrarium_probe_v2` director, which reports what the gate needs: torso
rest height vs the geometric expectation, engine ms/step, per-creature
displacement, and a park -> free-fall -> revive round trip for creature 0.

    python projects/alife/probe_v2.py
    OMNISIM_LOG_PATH=$PWD/projects/alife/_run/probe_v2.log \\
        python -m omnisim run-headless projects/alife/worlds/probe_v2.omniworld --duration 60

Then: 0 errors in the log, `motorized` count == hinges printed below, the
.newton.json sidecar `finalised: true`, and _run/probe_v2_result.json.
"""
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import genome2 as G          # noqa: E402
from alife import worldgen2 as W        # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
RUN = os.path.join(ROOT, "_run")
WORLD = os.path.join(ROOT, "worlds", "probe_v2.omniworld")
POP = os.path.join(RUN, "probe_v2_population.json")

SEED = 3
N = 8
ARENA = 14.0


def main():
    rng = random.Random(SEED)
    pop = G.seed_species(rng, N, pairs=2, segments=2)
    res = W.write_world(pop, WORLD, controller="terrarium_probe_v2", arena=ARENA,
                        title="Alife v2 -- gate A probe")

    # The director gets the genome plus everything it must not re-derive:
    # slot, authored home pose, and the geometric rest height to check against.
    rows = []
    for g, p in zip(pop, res["placements"]):
        row = dict(g)
        rp = G.rest_pose(g["body"])
        row.update({"slot": p["slot"], "home": p["pos"], "yaw": p["yaw"],
                    "rest_expected": G.rest_height(g["body"]),
                    "pitch_expected_deg": math.degrees(rp["pitch"]) if rp else None,
                    "stand_margin": rp["margin"] if rp else None,
                    "spawn_z": G.spawn_z(g["body"])})
        rows.append(row)
    os.makedirs(RUN, exist_ok=True)
    with open(POP, "w", encoding="utf-8", newline="\n") as f:
        json.dump(rows, f, indent=1)

    for g, p in zip(pop, res["placements"]):
        rp = G.rest_pose(g["body"])
        print("  slot %d  %s  home=(%.2f, %.2f, %.3f) rest z=%.3f pitch=%+.1f deg margin=%.2f"
              % (p["slot"], G.describe(g), p["pos"][0], p["pos"][1], p["pos"][2],
                 G.rest_height(g["body"]), math.degrees(rp["pitch"]), rp["margin"]))
    print("wrote %s  (%d creatures, %d hinges -> expect %d 'motorized' log lines)"
          % (os.path.relpath(WORLD, os.getcwd()), res["n"], res["hinges"], res["hinges"]))
    print("wrote %s" % os.path.relpath(POP, os.getcwd()))


if __name__ == "__main__":
    main()
