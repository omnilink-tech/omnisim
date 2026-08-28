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

"""Does a creature's fitness depend on WHO ELSE is in the world?

The champion scored 8.593 m during evolution and 3.1475 m in a world of clones.
Determinism holds (12/12 bitwise) and position sensitivity is exactly zero, and
at 20 m spacing the creatures cannot touch. So neither randomness, nor position,
nor collision explains the gap. The remaining candidate is that MuJoCo solves
every body in the world as ONE coupled constraint system, so the population a
genome is evaluated alongside is part of its measured fitness.

Three arms, identical genome and identical slot each time, one engine run each:

  ALONE     the champion by itself
  CLONES    the champion + 11 identical copies
  MIXED     the champion + the 11 other genomes it was evaluated with at gen 7

If ALONE != CLONES != MIXED, fitness is a property of (genome, population), not
of the genome -- which means selection was partly scoring world-context luck and
explains why evolution peaked at gen 7 and decayed afterwards.

  python projects/alife/attribution.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import genome as G           # noqa: E402
import evolve                            # noqa: E402

RUN = evolve.RUN


def score_slot0(pop, label):
    """Run one world and return slot 0's fitness. The champion is ALWAYS slot 0
    so the only thing varying between arms is its company."""
    fit = evolve.run_generation(pop, 700, 150, spacing=20.0)
    if fit is None:
        print("  %-8s FAILED" % label)
        return None
    v = fit["creatures"]["0"]
    print("  %-8s n=%2d   slot0 fitness = %.4f m   (status %s)"
          % (label, len(pop), v["fitness"], v["status"]))
    return v["fitness"]


def main():
    champs = json.load(open(os.path.join(RUN, "champions.json"), encoding="utf-8"))
    champ = max(champs, key=lambda c: c["fitness"])
    cg = champ["genome"]
    print("champion %s  evolution-time score %.3f m" % (cg["id"], champ["fitness"]))
    print("  %s\n" % G.describe(cg))

    def fresh(gid):
        g = json.loads(json.dumps(cg))
        g["id"] = gid
        return g

    # ARM 1 -- alone
    alone = score_slot0([fresh("champ")], "ALONE")

    # ARM 2 -- with 11 identical clones
    clones = [fresh("champ")] + [fresh("clone_%02d" % k) for k in range(11)]
    cloned = score_slot0(clones, "CLONES")

    # ARM 3 -- with the exact 11 genomes it was evaluated alongside
    world_pop = json.load(open(os.path.join(RUN, "population.json"), encoding="utf-8"))
    others = [g for g in world_pop if g["id"] != cg["id"]][:11]
    mixed = score_slot0([fresh("champ")] + others, "MIXED")

    print("\n=== VERDICT ===")
    vals = {"ALONE": alone, "CLONES": cloned, "MIXED": mixed}
    got = {k: v for k, v in vals.items() if v is not None}
    for k, v in got.items():
        print("  %-8s %.4f m" % (k, v))
    if len(set(round(v, 6) for v in got.values())) == 1:
        print("  => fitness is INDEPENDENT of the population. The evolution-time"
              " score has another explanation.")
    else:
        lo, hi = min(got.values()), max(got.values())
        print("  => fitness DEPENDS on the population: %.4f .. %.4f m"
              " (%.1fx) for one identical genome." % (lo, hi, hi / lo if lo else 0))
        print("     MuJoCo solves the world as one coupled constraint system, so")
        print("     'fitness' measured in a shared world is a property of")
        print("     (genome, population) -- not of the genome. Selection was")
        print("     partly scoring world-context luck.")
        print("     Fix: evaluate each genome in its OWN world, or hold the")
        print("     evaluation population fixed across generations.")

    json.dump(got, open(os.path.join(RUN, "attribution.json"), "w",
                        encoding="utf-8"), indent=1)


if __name__ == "__main__":
    main()
