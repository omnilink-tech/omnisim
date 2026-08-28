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

"""Generational evolution of locomoting creatures.

Each generation:
  1. write population.json + a freshly generated .omniworld
  2. run ONE headless engine process; the director actuates every creature from
     its genome and scores net displacement
  3. truncation-select the top fraction, mutate to refill

MORPHOLOGY EVOLVES because the world is REGENERATED every generation. That is
the whole trick: runtime `/scene/spawn` produces a node the solver never sees
(no physics), so a body plan cannot be created mid-run. Regenerating between
generations sidesteps that completely -- a world reload costs a few seconds
against a generation measured in tens of seconds.

Exactly ONE engine process runs at a time (thermal limit).

  python projects/alife/evolve.py --generations 12 --pop 12 --ticks 700
"""
import argparse
import json
import os
import random
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import genome as G           # noqa: E402
from alife import worldgen as W         # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run")
WORLD = os.path.join(ROOT, "worlds", "terrarium_evolve.omniworld")
HIST = os.path.join(RUN, "history.json")
CHAMPS = os.path.join(RUN, "champions.json")


def run_generation(pop, ticks, duration, spacing=20.0, quiet=True):
    """Write the world, run one engine, return the fitness dict (or None)."""
    os.makedirs(RUN, exist_ok=True)
    W.write_population(pop, os.path.join(RUN, "population.json"))
    W.write_world(pop, WORLD, controller="terrarium_evolve", spacing=spacing)

    fit_path = os.path.join(RUN, "fitness.json")
    if os.path.exists(fit_path):
        os.remove(fit_path)
    log = os.path.join(RUN, "engine.log")
    for suffix in ("", ".newton.json", ".stdout", ".stderr"):
        try:
            os.remove(log + suffix)
        except OSError:
            pass

    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = log
    env["PROBE_TICKS"] = str(ticks)
    cmd = [sys.executable, "-m", "omnisim", "run-headless",
           os.path.relpath(WORLD, REPO), "--duration", str(duration)]
    p = subprocess.run(cmd, cwd=REPO, env=env,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, timeout=duration + 180)
    if not os.path.exists(fit_path):
        print("    ENGINE PRODUCED NO FITNESS FILE (exit %d)" % p.returncode)
        print("    " + "\n    ".join(p.stdout.strip().splitlines()[-6:]))
        return None
    with open(fit_path, encoding="utf-8") as f:
        return json.load(f)


def select_and_breed(pop, fitness, rng, gen, elite_frac=0.34):
    """Truncation selection: keep the best, refill by mutating them."""
    scored = []
    for i, g in enumerate(pop):
        rec = fitness["creatures"].get(str(i))
        scored.append((rec["fitness"] if rec else 0.0, i, g))
    scored.sort(key=lambda t: -t[0])

    n_elite = max(2, int(len(pop) * elite_frac))
    elites = [g for _f, _i, g in scored[:n_elite]]

    nxt = []
    for k, g in enumerate(elites):                 # elitism: carry them intact
        e = dict(g)
        e["id"] = "g%d_e%02d" % (gen, k)
        e["parent"] = g["id"]
        nxt.append(e)
    while len(nxt) < len(pop):
        parent = elites[rng.randrange(len(elites))]
        # Late generations creep more finely than early ones.
        rate = 1.0 if gen < 4 else 0.65
        for _ in range(12):
            child = G.mutate(parent, rng, "g%d_m%02d" % (gen, len(nxt)), rate=rate)
            if not G.validate(child):
                nxt.append(child)
                break
        else:
            nxt.append(G.random_genome(rng, "g%d_r%02d" % (gen, len(nxt))))
    return nxt, scored


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--generations", type=int, default=12)
    ap.add_argument("--pop", type=int, default=12)
    ap.add_argument("--ticks", type=int, default=700)
    ap.add_argument("--duration", type=int, default=120)
    ap.add_argument("--seed", type=int, default=7)
    # Lanes must exceed 2x the best creature's travel or neighbours collide and
    # confound each other. Measured: at spacing 3.0 a 9.68 m champion re-scored
    # 0.28 m two generations later purely from pile-ups.
    ap.add_argument("--spacing", type=float, default=20.0)
    ap.add_argument("--resume", action="store_true")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    os.makedirs(RUN, exist_ok=True)

    history = []
    if args.resume and os.path.exists(HIST):
        with open(HIST, encoding="utf-8") as f:
            history = json.load(f)
    if args.resume and os.path.exists(os.path.join(RUN, "next_population.json")):
        with open(os.path.join(RUN, "next_population.json"), encoding="utf-8") as f:
            pop = json.load(f)
        print("resumed with %d genomes at generation %d" % (len(pop), len(history)))
    else:
        pop = G.seed_population(args.pop, args.seed)

    champions = []
    t_start = time.time()

    for gen in range(len(history), len(history) + args.generations):
        print("\n=== generation %d  (pop %d, %d ticks) ===" % (gen, len(pop), args.ticks))
        t0 = time.time()
        fit = run_generation(pop, args.ticks, args.duration, args.spacing)
        if fit is None:
            print("  generation failed; stopping")
            break

        nxt, scored = select_and_breed(pop, fit, rng, gen + 1)
        best_f, best_i, best_g = scored[0]
        wall = time.time() - t0
        mean_f = sum(s[0] for s in scored) / len(scored)

        row = {
            "gen": gen,
            "best": best_f,
            "mean": mean_f,
            "best_id": best_g["id"],
            "best_desc": G.describe(best_g),
            "engine_ms": fit.get("engine_ms_per_step_median"),
            "wall_s": round(wall, 1),
            "diverged": sum(1 for v in fit["creatures"].values()
                            if v["status"] == "diverged"),
        }
        history.append(row)
        champions.append({"gen": gen, "fitness": best_f, "genome": best_g})

        print("  best %.3f m   mean %.3f m   %s" % (best_f, mean_f, G.describe(best_g)))
        print("  engine %.1f ms/step   wall %.0fs   diverged %d"
              % (row["engine_ms"] or -1, wall, row["diverged"]))

        with open(HIST, "w", encoding="utf-8") as f:
            json.dump(history, f, indent=1)
        with open(CHAMPS, "w", encoding="utf-8") as f:
            json.dump(champions, f, indent=1)
        with open(os.path.join(RUN, "next_population.json"), "w", encoding="utf-8") as f:
            json.dump(nxt, f, indent=1)
        # keep the champion's world for later replay / capture
        shutil.copyfile(WORLD, os.path.join(RUN, "world_gen%02d.omniworld" % gen))
        pop = nxt

    print("\n=== summary (%.0f s total) ===" % (time.time() - t_start))
    for h in history:
        print("  gen %2d  best %6.3f m  mean %6.3f m  %s"
              % (h["gen"], h["best"], h["mean"], h["best_desc"]))
    if history:
        first, last = history[0]["best"], max(h["best"] for h in history)
        print("  improvement: %.3f m -> %.3f m (%.1fx)"
              % (first, last, (last / first) if first > 1e-6 else float("inf")))


if __name__ == "__main__":
    main()
