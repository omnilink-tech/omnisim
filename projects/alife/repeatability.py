#!/usr/bin/env python3
"""Is fitness reproducible? The experiment that decides whether selection works.

Evolution peaked at 8.593 m (gen 7) and decayed to ~1.0 m by gen 23 despite
elitism carrying the champion forward intact. That is only possible if a
genome's measured fitness is not a property of the genome.

Two arms, one engine run each:

  CLONES  12 IDENTICAL copies of the champion, one per evaluation slot.
          Same genome, same gait, same flat floor, different absolute position.
          Any spread here is position sensitivity -- contact-rich locomotion is
          chaotic, and float rounding differs at x=-30 vs x=+10.

  REPEAT  the identical world run a second time.
          Any spread here would contradict the engine's documented bitwise
          determinism on CPU mj_step, which would be a much bigger finding.

  python projects/alife/repeatability.py
"""
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import genome as G           # noqa: E402
import evolve                            # noqa: E402

RUN = evolve.RUN


def main():
    champs = json.load(open(os.path.join(RUN, "champions.json"), encoding="utf-8"))
    champ = max(champs, key=lambda c: c["fitness"])
    print("champion: gen %d  %.3f m  %s"
          % (champ["gen"], champ["fitness"], G.describe(champ["genome"])))

    clones = []
    for k in range(12):
        g = json.loads(json.dumps(champ["genome"]))
        g["id"] = "clone_%02d" % k
        clones.append(g)

    print("\n--- ARM 1: 12 identical clones, 12 positions ---")
    a = evolve.run_generation(clones, 700, 150, spacing=20.0)
    if a is None:
        sys.exit("arm 1 produced no fitness")
    fa = [a["creatures"][str(i)]["fitness"] for i in range(12)]
    for i, v in enumerate(fa):
        print("  slot %2d  %.4f m" % (i, v))
    print("  mean %.4f  stdev %.4f  min %.4f  max %.4f  spread %.4f"
          % (statistics.mean(fa), statistics.pstdev(fa), min(fa), max(fa),
             max(fa) - min(fa)))

    print("\n--- ARM 2: the identical world, run again ---")
    b = evolve.run_generation(clones, 700, 150, spacing=20.0)
    if b is None:
        sys.exit("arm 2 produced no fitness")
    fb = [b["creatures"][str(i)]["fitness"] for i in range(12)]
    same = sum(1 for x, y in zip(fa, fb) if x == y)
    print("  bitwise-identical slots: %d/12" % same)
    worst = max(abs(x - y) for x, y in zip(fa, fb))
    print("  max |difference|: %.6g m" % worst)

    print("\n=== VERDICT ===")
    spread = max(fa) - min(fa)
    rel = spread / statistics.mean(fa) if statistics.mean(fa) else float("inf")
    if same == 12:
        print("  determinism: HOLDS (same world -> same numbers)")
    else:
        print("  determinism: VIOLATED on %d/12 slots -- investigate before"
              " trusting ANY measurement here" % (12 - same))
    print("  position sensitivity: spread %.4f m across identical genomes"
          " (%.0f%% of mean)" % (spread, 100 * rel))
    if rel > 0.25:
        print("  => single-trial fitness is NOT a property of the genome.")
        print("     Selection is partly selecting luck, which is why an elite")
        print("     that scored 8.59 m can re-score ~1 m and be dropped.")
        print("     Fix: average each genome over several fixed slots.")
    else:
        print("  => fitness is reasonably stable; the decay has another cause.")

    json.dump({"clone_fitness_run1": fa, "clone_fitness_run2": fb,
               "champion": champ}, open(os.path.join(RUN, "repeatability.json"),
                                        "w", encoding="utf-8"), indent=1)


if __name__ == "__main__":
    main()
