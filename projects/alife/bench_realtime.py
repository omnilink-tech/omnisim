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

"""Find the config where the champion showcase runs in REAL TIME.

Real time means physics ms/step < basicTimeStep ms -- otherwise a windowed
realtime-mode engine cannot keep up with the wall clock and the demo plays in
slow motion while burning a full core trying.

Sweeps (serially -- ONE engine at a time, thermal limit):
  dt        8 vs 16 ms   (16 halves the steps per simulated second)
  contacts  hard (ke 8000/kd 200/impratio 10, what the champions evolved under)
            vs mid (ke 4000/kd 120/impratio 5)

Each arm runs the SAME 5 champions through the evolve director, which reports
engine ms/step AND per-creature displacement -- so we see at once whether a
cheaper config breaks the evolved gaits. Sim duration is held at 5.6 s across
dt values (700 ticks @ 8 ms vs 350 @ 16 ms) so displacements are comparable.

  python projects/alife/bench_realtime.py
"""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import worldgen as W         # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run")
WORLD = os.path.join(ROOT, "worlds", "_bench.omniworld")

SIM_SECONDS = 5.6


def load_champions(count=5):
    rows = json.load(open(os.path.join(RUN, "rescored.json"), encoding="utf-8"))
    rows.sort(key=lambda r: -r["measured_m"])
    # same picker spirit as showcase.py: best distinct bodies
    pop = []
    for r in rows[:count]:
        g = json.loads(json.dumps(r["genome"]))
        g["_gen"] = r["gen"]
        pop.append(g)
    return pop


def run_arm(name, pop, dt, ke, kd, impratio):
    ticks = int(round(SIM_SECONDS * 1000.0 / dt))
    settle = int(round(0.48 * 1000.0 / dt))          # 0.48 s settle, dt-invariant
    W.write_population(pop, os.path.join(RUN, "population.json"))
    W.write_world(pop, WORLD, controller="terrarium_evolve",
                  spacing=3.5, dt=dt, ke=ke, kd=kd, impratio=impratio,
                  arena_margin=4.0, arena_min=10.0)

    fit_path = os.path.join(RUN, "fitness.json")
    if os.path.exists(fit_path):
        os.remove(fit_path)
    log = os.path.join(RUN, "bench_%s.log" % name)

    env = dict(os.environ)
    env["OMNISIM_LOG_PATH"] = log
    env["PROBE_TICKS"] = str(ticks)
    env["SETTLE_TICKS"] = str(settle)
    t0 = time.time()
    subprocess.run([sys.executable, "-m", "omnisim", "run-headless",
                    os.path.relpath(WORLD, REPO), "--duration", "150"],
                   cwd=REPO, env=env, stdout=subprocess.DEVNULL,
                   stderr=subprocess.STDOUT, timeout=300)
    wall = time.time() - t0
    if not os.path.exists(fit_path):
        print("  %-12s FAILED (no fitness file)" % name)
        return None
    fit = json.load(open(fit_path, encoding="utf-8"))
    ms = fit.get("engine_ms_per_step_median", -1)
    disp = [round(fit["creatures"][str(i)]["fitness"], 3) for i in range(len(pop))]
    rt = "REALTIME-OK" if ms < dt else "TOO SLOW (%.1fx budget)" % (ms / dt)
    print("  %-12s dt=%2d  %6.2f ms/step  budget %2d ms  %-22s disp=%s  wall=%.0fs"
          % (name, dt, ms, dt, rt, disp, wall))
    return {"name": name, "dt": dt, "ke": ke, "kd": kd, "impratio": impratio,
            "ms_per_step": ms, "realtime": ms < dt, "displacement": disp}


def main():
    pop = load_champions()
    print("bench: 5 champions, spacing 3.5, tight arena, %.1f s sim each\n"
          % SIM_SECONDS)
    arms = [
        ("dt8_hard",  8, 8000, 200, 10),
        ("dt16_hard", 16, 8000, 200, 10),
        ("dt8_mid",   8, 4000, 120, 5),
        ("dt16_mid",  16, 4000, 120, 5),
    ]
    results = []
    for a in arms:
        r = run_arm(*[a[0]] + [pop] + list(a[1:]))
        if r:
            results.append(r)

    json.dump(results, open(os.path.join(RUN, "bench_realtime.json"), "w",
                            encoding="utf-8"), indent=1)
    ok = [r for r in results if r["realtime"]]
    print("\nrealtime-capable arms: %s" % ([r["name"] for r in ok] or "NONE"))
    if os.path.exists(WORLD):
        os.remove(WORLD)


if __name__ == "__main__":
    main()
