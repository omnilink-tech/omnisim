#!/usr/bin/env python3
"""Build the watchable demo: evolved champions from across the run, side by side.

Picks the champion of several generations spread across the evolution history and
lines them up on coloured start pads in one arena. Because they all start on a
pad and each keeps its generation's body plan and gait, the progression is
readable straight off the frame -- later champions travel visibly further.

  python projects/alife/showcase.py                  # build from _run/champions.json
  python projects/alife/showcase.py --launch         # ...and open it windowed
  python projects/alife/showcase.py --count 6
"""
import argparse
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import genome as G           # noqa: E402
from alife import worldgen as W         # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run")
OUT = os.path.join(ROOT, "worlds", "alife_champions.omniworld")


def pick(champs, count):
    """Champions spread evenly across the run, always including first and last.

    Deduplicated by genome id: elitism carries a champion forward unchanged, so
    consecutive generations often share one, and a lineup of five identical
    creatures shows nothing.
    """
    if not champs:
        return []
    best_by_gen = {}
    for c in champs:
        g = c["gen"]
        if g not in best_by_gen or c["fitness"] > best_by_gen[g]["fitness"]:
            best_by_gen[g] = c
    ordered = [best_by_gen[k] for k in sorted(best_by_gen)]

    seen, uniq = set(), []
    for c in ordered:
        gid = c["genome"]["id"]
        if gid not in seen:
            seen.add(gid)
            uniq.append(c)
    if len(uniq) <= count:
        return uniq
    idx = [round(i * (len(uniq) - 1) / (count - 1)) for i in range(count)]
    return [uniq[i] for i in sorted(set(idx))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=5)
    ap.add_argument("--spacing", type=float, default=4.5)
    ap.add_argument("--launch", action="store_true")
    ap.add_argument("--headless", type=int, default=0,
                    help="run headless for N seconds instead of opening a window")
    args = ap.parse_args()

    # Prefer rescored.json: evolution-time fitness is NOT comparable across
    # generations (each generation changes the population, which changes the
    # evaluation world -- only 1 of 21 champions reproduced its score). The
    # rescore measures every champion simultaneously in ONE world, so it is the
    # only apples-to-apples ranking. Run projects/alife/rescore.py to make it.
    rescored = os.path.join(RUN, "rescored.json")
    if os.path.exists(rescored):
        with open(rescored, encoding="utf-8") as f:
            rows = json.load(f)
        champs = [{"gen": r["gen"], "fitness": r["measured_m"],
                   "genome": r["genome"]} for r in rows]
        print("using rescored.json (honest, one-world ranking)")
    else:
        champ_path = os.path.join(RUN, "champions.json")
        if not os.path.exists(champ_path):
            sys.exit("no champions yet -- run projects/alife/evolve.py first")
        with open(champ_path, encoding="utf-8") as f:
            champs = json.load(f)
        print("WARNING: using evolution-time fitness, which does not reproduce")

    chosen = pick(champs, args.count)
    chosen.sort(key=lambda c: c["fitness"])          # slowest -> fastest, left to right
    pop = []
    for c in chosen:
        g = dict(c["genome"])
        g["_gen"] = c["gen"]
        g["_fitness"] = c["fitness"]
        pop.append(g)

    print("champion lineup (%d of %d generations):" % (len(pop), len(champs)))
    for g in pop:
        print("  gen %2d  %6.3f m  %s" % (g["_gen"], g["_fitness"], G.describe(g)))

    W.write_population(pop, os.path.join(RUN, "showcase_population.json"))
    # Tight arena + dt8 + hard contacts: measured 4.16 ms/step for 5 champions
    # against the 8 ms realtime budget (bench_realtime.py), with the exact
    # contact physics the champions evolved under. dt16 and softer contacts are
    # both cheaper AND both measurably degrade the evolved gaits -- not used.
    info = W.write_world(pop, OUT, controller="terrarium_showcase",
                         spacing=args.spacing, pads=True,
                         arena_margin=4.0, arena_min=10.0,
                         title="OmniSim alife - evolved champions")
    print("\nwrote %s  (%d creatures, %.0f m arena)"
          % (info["path"], info["n"], info["arena"]))

    rel = os.path.relpath(OUT, REPO)
    if args.headless:
        env = dict(os.environ)
        env["OMNISIM_LOG_PATH"] = os.path.join(RUN, "showcase.log")
        subprocess.run([sys.executable, "-m", "omnisim", "run-headless", rel,
                        "--duration", str(args.headless)], cwd=REPO, env=env)
    elif args.launch:
        subprocess.Popen([sys.executable, "-m", "omnisim", "run-world", rel], cwd=REPO)
        print("launching windowed...")
    else:
        print("\nwatch it:   python -m omnisim run-world %s" % rel)


if __name__ == "__main__":
    main()
