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

"""PoC: GPU-batched MuJoCo-Warp throughput vs the CPU Webots pipeline.

Validates the GPU-trainer premise before building the full PPO stack:
  1. Build the representative quadruped in Newton's ModelBuilder.
  2. Export the EXACT MuJoCo model Newton simulates via SolverMuJoCo(
     save_to_mjcf=...) -- guarantees zero sim-to-sim gap with the OmniSim
     Newton deploy target.
  3. Load that MJCF into mujoco_warp, batch to N parallel worlds on the GPU
     (cuda:0), step, and measure env-steps/sec.

Compare against the current pipeline: 2 Webots envs at ~30-150 steps/s.

Run:
    python projects/policies/research/tools/mjwarp_throughput_poc.py
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(REPO / "projects" / "rl" / "tools"))


def main():
    import numpy as np
    import warp as wp
    import newton
    import mujoco
    import mujoco_warp as mjw
    from newton_friction_probe import build  # reuse the quad builder

    wp.init()
    print(f"newton {newton.__version__}  warp {wp.__version__}  mujoco {mujoco.__version__}")

    # 1) build quad in Newton, 2) export the exact MuJoCo model to MJCF
    model = build(mu=2.0)[0]
    mjcf_path = r"C:\tmp\quad_newton_export.xml"
    try:
        newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True, save_to_mjcf=mjcf_path)
        print(f"exported Newton model -> {mjcf_path}")
    except TypeError:
        # older signature: positional only
        newton.solvers.SolverMuJoCo(model, save_to_mjcf=mjcf_path)
        print(f"exported Newton model -> {mjcf_path}")

    # 3) load the exported MJCF and batch on GPU
    mjm = mujoco.MjModel.from_xml_path(mjcf_path)
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)
    print(f"model: nq={mjm.nq} nv={mjm.nv} nu={mjm.nu} nbody={mjm.nbody} ngeom={mjm.ngeom}")

    device = wp.get_device("cuda:0")
    for N in (256, 1024, 4096):
        with wp.ScopedDevice(device):
            mw_m = mjw.put_model(mjm)
            mw_d = mjw.put_data(mjm, mjd, nworld=N)
            # warmup + compile
            mjw.step(mw_m, mw_d)
            wp.synchronize()
            STEPS = 200
            t0 = time.time()
            for _ in range(STEPS):
                mjw.step(mw_m, mw_d)
            wp.synchronize()
            dt = time.time() - t0
        env_steps = N * STEPS
        print(f"  N={N:5d} worlds: {STEPS} steps in {dt:.2f}s  ->  "
              f"{env_steps/dt:,.0f} env-steps/s  ({N*1.0/ (dt/STEPS):,.0f})")


if __name__ == "__main__":
    main()
