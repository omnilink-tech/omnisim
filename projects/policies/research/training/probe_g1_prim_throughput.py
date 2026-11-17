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

"""BATCHED throughput of the PRIMITIVE-collision deploy-faithful G1, collide ON.

The sibling probe (probe_newton_batch_throughput.py) showed the deploy-faithful
build_g1_native cannot be batched with collision on: its URDF MESH colliders
overflow the SolverMuJoCo broad-phase ("Triangle pair buffer overflowed
8,771,072 > 1,000,000" at 256 worlds), so that probe must set
OMNISIM_BATCH_NOCOLLIDE=1. This probe uses build_g1_native_prim instead --
dynamics-identical but with PRIMITIVE foot colliders only -- and runs the exact
deploy substep loop WITH COLLISION ON across N worlds. This is the number that
unblocks a batched Newton (SolverMuJoCo) fine-tune trainer.

We replicate the prim BUILDER via ModelBuilder.replicate(builder, world_count=N)
(replicate needs a builder, not a finalized model), finalize the main builder,
build SolverMuJoCo(use_mujoco_cpu=False) (GPU mjwarp, separate_worlds auto), warm
up (kernel compile / autotune outside the timer) then time.

env-steps/s = N_worlds * control_steps / wall   (one control decision per env per
step, SUBSTEPS physics sub-steps each -- matches the trainer's metric and the
sibling probe). NO CUDA-graph capture -> a conservative LOWER bound. Run from
PowerShell (native warp/CUDA).
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import numpy as np
import warp as wp
import newton

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))
from projects.policies.research.training.build_g1_native import (  # noqa: E402
    NOMINAL13, DOF0, NJ)
from projects.policies.research.training.build_g1_native_prim import (  # noqa: E402
    build_g1_prim_builder)

SUBSTEPS = 4
DT = 0.016
SUB_DT = DT / SUBSTEPS


def _run(N: int, steps: int = 120) -> bool:
    # Build ONE prim G1 (no per-builder ground; add the ground once on the main
    # builder so each world gets its own ground via replicate).
    g1 = build_g1_prim_builder(add_ground=True)
    main_b = newton.ModelBuilder()
    main_b.replicate(g1, world_count=N, spacing=(3.0, 3.0, 0.0))
    model = main_b.finalize()
    wc = getattr(model, "world_count", "?")
    n_shapes = len(model.shape_type.numpy()) if hasattr(model, "shape_type") else "?"
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    # COLLISION ON (primitive feet -> tiny broad-phase, no overflow).
    contacts = model.contacts() if hasattr(model, "contacts") else None
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)
    # Hold nominal (model.joint_target_pos already carries it per world).
    try:
        if hasattr(control, "joint_target_pos") and hasattr(model, "joint_target_pos"):
            control.joint_target_q.assign(model.joint_target_q.numpy())
    except Exception as e:
        print(f"  (target_pos copy skipped: {e})")

    def _step():
        nonlocal state_a, state_b
        for _ in range(SUBSTEPS):
            state_a.clear_forces()
            if contacts is not None:
                model.collide(state_a, contacts)
                solver.step(state_a, state_b, control, contacts, SUB_DT)
            else:
                solver.step(state_a, state_b, control, None, SUB_DT)
            state_a, state_b = state_b, state_a

    # Warmup (kernel compile + autotune + first-collide outside the timer).
    for _ in range(15):
        _step()
    wp.synchronize()
    t0 = time.time()
    for _ in range(steps):
        _step()
    wp.synchronize()
    T = time.time() - t0
    env_sps = N * steps / T
    phys_sps = env_sps * SUBSTEPS
    finite = bool(np.isfinite(state_a.body_q.numpy()).all())
    print(f"N={N:5d} (world_count={wc}, shapes={n_shapes}) steps={steps} | "
          f"{T:5.2f}s | {env_sps:9.0f} env-steps/s | {phys_sps:10.0f} phys-steps/s "
          f"| collide=ON finite={finite}")
    return finite


def main() -> int:
    wp.init()
    print("[g1-prim-throughput] BATCHED SolverMuJoCo (deploy solver), PRIMITIVE "
          "collision ON, no graph capture")
    ns = os.environ.get("OMNISIM_BATCH_N", "256,1024,2048,4096")
    for N in [int(x) for x in ns.split(",") if x.strip()]:
        try:
            _run(N)
        except Exception as e:
            print(f"N={N}: FAILED ({type(e).__name__}: {e})")
            import traceback
            traceback.print_exc()
            break
    return 0


if __name__ == "__main__":
    sys.exit(main())
