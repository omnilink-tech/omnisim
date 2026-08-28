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

"""Re-measure every champion under identical conditions, in one run.

Why this exists: evolution-time fitness did not reproduce. The gen-7 champion
recorded 8.593 m during the run and re-scores 3.1475 m under four independent
re-measurements (alone / with clones / with its original cohort / repeat run),
all of which agree with each other to 4 decimal places. Determinism holds
(12/12 bitwise) and fitness is position-invariant (12 clones at 12 positions,
spread exactly 0.0000 m), so the re-measurement is the trustworthy number and
the evolution-time score is not.

This puts every unique champion in ONE world and scores them together, which is
sound because creatures at this spacing provably do not affect each other.
Output is the honest ranking the showcase should be built from.

  python projects/alife/rescore.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import genome as G           # noqa: E402
import evolve                            # noqa: E402

RUN = evolve.RUN
OUT = os.path.join(RUN, "rescored.json")


def main():
    champs = json.load(open(os.path.join(RUN, "champions.json"), encoding="utf-8"))

    # Elitism carries a champion forward unchanged, so consecutive generations
    # often name the same body. Score each distinct genome once.
    uniq, seen = [], set()
    for c in champs:
        key = json.dumps({k: v for k, v in c["genome"].items()
                          if k not in ("id", "parent")}, sort_keys=True)
        if key in seen:
            continue
        seen.add(key)
        g = json.loads(json.dumps(c["genome"]))
        g["_gen"] = c["gen"]
        g["_evolution_time_fitness"] = c["fitness"]
        uniq.append(g)

    print("re-scoring %d distinct champions from %d generations"
          % (len(uniq), len(champs)))
    fit = evolve.run_generation(uniq, 700, 150, spacing=20.0)
    if fit is None:
        sys.exit("no fitness produced")

    rows = []
    for i, g in enumerate(uniq):
        rec = fit["creatures"].get(str(i), {})
        rows.append({
            "gen": g["_gen"],
            "id": g["id"],
            "measured_m": rec.get("fitness", 0.0),
            "evolution_time_m": g["_evolution_time_fitness"],
            "status": rec.get("status"),
            "desc": G.describe(g),
            "genome": {k: v for k, v in g.items() if not k.startswith("_")},
        })

    rows.sort(key=lambda r: -r["measured_m"])
    print("\n=== honest ranking (all measured in one identical world) ===")
    print("  %-4s %-10s %10s %10s   %s" % ("gen", "id", "measured", "evo-time", "body"))
    for r in rows:
        flag = "" if abs(r["measured_m"] - r["evolution_time_m"]) < 0.15 else "  <-- differs"
        print("  %-4d %-10s %9.3fm %9.3fm   %s%s"
              % (r["gen"], r["id"], r["measured_m"], r["evolution_time_m"],
                 r["desc"], flag))

    agree = sum(1 for r in rows if abs(r["measured_m"] - r["evolution_time_m"]) < 0.15)
    print("\n  %d/%d champions reproduce their evolution-time score (+-0.15 m)"
          % (agree, len(rows)))
    by_gen = sorted(rows, key=lambda r: r["gen"])
    print("  first champion %.3f m -> best champion %.3f m (%.1fx)"
          % (by_gen[0]["measured_m"], max(r["measured_m"] for r in rows),
             max(r["measured_m"] for r in rows) / by_gen[0]["measured_m"]
             if by_gen[0]["measured_m"] else float("inf")))

    json.dump(rows, open(OUT, "w", encoding="utf-8"), indent=1)
    print("\nwrote %s" % OUT)


if __name__ == "__main__":
    main()
