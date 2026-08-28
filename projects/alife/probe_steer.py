#!/usr/bin/env python3
"""Does a turn command actually turn a creature? The foraging premise check.

Epoch 0 of the ecosystem produced 5 eats in 120 s with 10 food items always
active in a 12 m arena and sense radii of 3-5 m: food was almost always in
range, and the creatures still did not reach it. Either steering does not
work or the gait does not go where the nose points. This probe isolates the
first question.

Eight IDENTICAL copies of one evolved body, each driven with a CONSTANT turn
command in {-1, -0.5, -0.25, 0, 0, 0.25, 0.5, 1} through the exact same
left/right amplitude modulation the director uses. Measured per creature over
15 s: net yaw change, path length, and curvature (yaw / path). A working
steering channel shows curvature monotone in the command and antisymmetric
about zero; the two zero-command creatures give the baseline drift.

  python projects/alife/probe_steer.py            # build + run headless
"""
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import worldgen2 as W2       # noqa: E402
from alife import scene                  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "probe_steer")
WORLD = os.path.join(ROOT, "worlds", "probe_steer.omniworld")

TURNS = [-1.0, -0.5, -0.25, 0.0, 0.0, 0.25, 0.5, 1.0]


def main():
    src = os.path.join(ROOT, "_run", "life", "epoch_00", "population.json")
    pop0 = json.load(open(src, encoding="utf-8"))
    # the epoch-0 forager: sp3 ate the most
    base = next(g for g in pop0 if g["species"] == "sp3")

    pop = []
    for i, turn in enumerate(TURNS):
        g = json.loads(json.dumps(base))
        g["id"] = "steer_%d" % i
        g["slot"] = i
        g["alive_at_start"] = True
        g["_turn"] = turn
        g["yaw"] = 0.0                       # all face +x
        pop.append(g)

    os.makedirs(RUN, exist_ok=True)
    json.dump(pop, open(os.path.join(RUN, "population.json"), "w",
                        encoding="utf-8"), indent=1)
    arena = 30.0                             # wide lanes: 15 s of travel, no walls hit
    W2.write_world(pop, WORLD, scene_lines=scene.scene_lines(arena, 0, "terrarium_probe_steer"),
                   controller="terrarium_probe_steer", arena=arena, spacing=3.5)

    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = os.path.join(RUN, "engine.log")
    subprocess.run([sys.executable, "-m", "omnisim", "run-headless",
                    os.path.relpath(WORLD, REPO), "--duration", "90"],
                   cwd=REPO, env=env, timeout=200)

    res = json.load(open(os.path.join(RUN, "result.json"), encoding="utf-8"))
    print("\n%-6s %8s %8s %10s   %s" % ("turn", "yaw_deg", "path_m", "curv", "verdict"))
    rows = sorted(res["creatures"], key=lambda r: r["turn"])
    for r in rows:
        print("%+5.2f  %+8.1f %8.2f %+10.3f" % (r["turn"], r["yaw_deg"], r["path_m"], r["curvature"]))
    # monotonicity: curvature should increase with turn
    curv = [r["curvature"] for r in rows]
    inversions = sum(1 for a, b in zip(curv, curv[1:]) if b < a - 0.02)
    span = curv[-1] - curv[0]
    print("\ncurvature span (turn -1 -> +1): %+.3f rad/m   inversions: %d/7   engine %.2f ms/step"
          % (span, inversions, res.get("engine_ms_per_step_median", -1)))
    if abs(span) > 0.3 and inversions <= 1:
        print("VERDICT: steering channel WORKS (monotone, antisymmetric)")
    else:
        print("VERDICT: steering channel is WEAK or BROKEN -- the gait does not answer the command")


if __name__ == "__main__":
    main()
