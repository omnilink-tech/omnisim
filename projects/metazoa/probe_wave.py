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

"""Wave direction + steering sign on one welded 4-chain (see the controller)."""
import json, os, random, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mz import cell as C, worldgen as W, scene as S, organism as ORG
ROOT = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.normpath(os.path.join(ROOT, "..", ".."))
RUN = os.path.join(ROOT, "_run", "probe_wave"); WORLD = os.path.join(ROOT, "worlds", "probe_wave.omniworld")
pattern = [int(x) for x in (sys.argv[1] if len(sys.argv) > 1 else "0,1").split(",")]
rng = random.Random(3)
gen = ORG.random_genome(rng); gen.update({"A": float(os.environ.get("WAVE_A", "0.9")), "omega": float(os.environ.get("OMEGA", "4.5")), "dphi": float(os.environ.get("DPHI", "1.2")), "bias_pitch": 0.0, "bias_yaw": 0.0, "steer_gain": float(os.environ.get("STEER_GAIN", "0.5"))})
n = int(os.environ.get("N_CELLS", "4"))
pat = [pattern[k % len(pattern)] for k in range(n)]
poses = C.chain_poses((0.0, 0.0, 0.0), n, gap=C.DEFAULT_GAP, dock_rotations=pat)
cells = [{"id": k, "pos": p["pos"], "yaw": p["yaw"], "roll": p["roll"], "parked": False} for k, p in enumerate(poses)]
os.makedirs(RUN, exist_ok=True)
# chain_poses puts index 0 at the head pose and walks +x; we call index 0 the TAIL and index 3 the HEAD
rudder = os.environ.get("RUDDER", "0") == "1"
phases = [["-dphi steer0", -1.0, 0.0], ["-dphi steer+", -1.0, 1.0], ["-dphi steer-", -1.0, -1.0], ["-dphi steer0b", -1.0, 0.0]] if rudder else None
cfg = {"spine": list(range(n)), "pattern": pat, "genome": gen, "phase_s": 15.0, "rudder": rudder}
if phases: cfg["phases"] = phases
json.dump(cfg, open(os.path.join(RUN, "config.json"), "w"), indent=1)
W.write_world(cells, WORLD, scene_lines=S.scene_lines(10.0, 0, "metazoa_probe_wave"), controller="metazoa_probe_wave", rollers="v3", substeps=4, arena=10.0)
env = dict(os.environ); env["OMNISIM_LOG_PATH"] = os.path.join(RUN, "engine.log")
subprocess.run([sys.executable, "-m", "omnisim", "run-headless", os.path.relpath(WORLD, REPO), "--duration", "110"], cwd=REPO, env=env, timeout=300, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
res = json.load(open(os.path.join(RUN, "result.json")))
print("pattern", pat, "| n =", n, "cell k at x = k*0.13 (index 0 = tail, index n-1 = head)")
print("%-14s %12s %10s %8s %10s" % ("phase", "along_spine", "lateral", "dist", "yaw_chg"))
for p in res["phases"]:
    print("%-14s %+12.3f %+10.3f %8.3f %+10.2f" % (p["name"], p["along_spine_m"], p["lateral_m"], p["dist_m"], p["yaw_change_rad"]))
