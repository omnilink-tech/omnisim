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

"""Steering-channel sweep: amplitude asymmetry vs stride-bias asymmetry, on
one evolved body, 8 conditions, one engine run. Picks the channel to build
the steering law on. Usage: python projects/alife/probe_steer2.py [species]"""
import json, os, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from alife import worldgen2 as W2, scene
ROOT = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "probe_steer2"); WORLD = os.path.join(ROOT, "worlds", "probe_steer2.omniworld")
CONDS = [
    {"name": "base",      "ls": 1.0, "rs": 1.0, "lb": 0.0,  "rb": 0.0},
    {"name": "amp+0.3",   "ls": 0.7, "rs": 1.3, "lb": 0.0,  "rb": 0.0},
    {"name": "amp+0.6",   "ls": 0.4, "rs": 1.6, "lb": 0.0,  "rb": 0.0},
    {"name": "amp-0.6",   "ls": 1.6, "rs": 0.4, "lb": 0.0,  "rb": 0.0},
    {"name": "bias+0.15", "ls": 1.0, "rs": 1.0, "lb": 0.15, "rb": -0.15},
    {"name": "bias+0.30", "ls": 1.0, "rs": 1.0, "lb": 0.30, "rb": -0.30},
    {"name": "bias-0.30", "ls": 1.0, "rs": 1.0, "lb": -0.30, "rb": 0.30},
    {"name": "base2",     "ls": 1.0, "rs": 1.0, "lb": 0.0,  "rb": 0.0},
]
def archetype(hip_amp=0.35, knee_amp=0.35, freq=1.2, splay=0.6, seg=0.12, torso=(0.30, 0.05),
              pairs_x=(0.7, -0.7), seg_r=0.02, knee_lag=1.5708, knee_sign=-1.0, hip_bias=0.0):
    """A designed sprawling walker. Knee = -amp*(1 - cos(hip phase)): flexed
    (foot lifted) while the hip swings the foot FORWARD, extended while it
    pushes back. Trot: pairs in antiphase, left/right mirrored by pi."""
    import math
    body = {"torso": {"length": torso[0], "radius": torso[1]},
            "head": {"radius": torso[1] * 0.9},
            "pairs": [{"x": x, "z": 0.0, "splay": splay,
                       "segments": [{"length": seg, "radius": seg_r},
                                    {"length": seg, "radius": seg_r * 0.85}]} for x in pairs_x],
            "hue": 0.55}
    brain = {"freq": freq, "mirror_phase": math.pi, "steer_gain": 0.5, "heading_offset": 0.0,
             "sense_radius": 4.0, "wander": 0.3,
             "pairs": [{"hip": {"amp": hip_amp, "bias": hip_bias, "phase": k * math.pi},
                        "knee": {"amp": knee_amp, "bias": knee_sign * knee_amp,
                                 "phase": k * math.pi + knee_lag}}
                       for k in range(len(pairs_x))]}
    return {"id": "arch", "species": "arch", "parent": None, "body": body, "brain": brain}


def main():
    want = sys.argv[1] if len(sys.argv) > 1 else None
    variants = None
    if want == "arch":
        kw = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
        base = archetype(**kw)
        from alife import genome2 as G
        print("archetype validate:", G.validate(base) or "ok")
    elif want == "variants":
        # 8 gait/body variants, all UNSTEERED: which archetype walks best?
        variants = json.loads(sys.argv[2])
        base = archetype(**variants[0])
    else:
        pop0 = json.load(open(os.path.join(ROOT, "_run", "life", "epoch_00", "population.json"), encoding="utf-8"))
        base = next(g for g in pop0 if want is None or g["species"] == want)
    pop = []
    for i, c in enumerate(CONDS):
        if variants is not None:
            g = archetype(**variants[i % len(variants)])
            c = dict(CONDS[0]); c["name"] = "v%d" % i
        else:
            g = json.loads(json.dumps(base))
        g["id"] = "s%d" % i; g["slot"] = i; g["alive_at_start"] = True
        g["_cond"] = c; g["yaw"] = 0.0; pop.append(g)
    os.makedirs(RUN, exist_ok=True)
    json.dump(pop, open(os.path.join(RUN, "population.json"), "w"), indent=1)
    W2.write_world(pop, WORLD, scene_lines=scene.scene_lines(40, 0, "terrarium_probe_steer2"),
                   controller="terrarium_probe_steer2", arena=40.0, spacing=4.5)
    env = dict(os.environ); env["OMNISIM_LOG_PATH"] = os.path.join(RUN, "engine.log")
    subprocess.run([sys.executable, "-m", "omnisim", "run-headless", os.path.relpath(WORLD, REPO), "--duration", "120"],
                   cwd=REPO, env=env, timeout=240, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    res = json.load(open(os.path.join(RUN, "result.json")))
    print("body: %s  (%s)   engine %.2f ms/step" % (base["id"], base["species"], res["engine_ms"]))
    print("%-10s %8s %7s %8s %6s %5s %6s" % ("cond", "yaw_deg", "path_m", "curv", "speed", "flips", "z"))
    for i, r in enumerate(res["rows"]):
        tag = (" " + json.dumps(variants[i % len(variants)])) if variants else ""
        print("%-10s %+8.1f %7.2f %+8.3f %6.2f %5d %6.3f%s" % (r["cond"], r["yaw_deg"], r["path_m"], r["curv"], r["speed"], r["flips"], r["mean_z"], tag))
if __name__ == "__main__":
    main()
