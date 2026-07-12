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

"""POC: how fast is BATCHED SolverMuJoCo (the deploy solver) on the GPU?

We've trained at ~280-500k env-steps/s with raw mjw.step. The open question for
"train in the deploy solver" is the throughput of newton's SolverMuJoCo (which
wraps mjw.step) when batched to N parallel worlds. This probe replicates the
deploy-faithful native G1 (build_g1_native) to N worlds via
ModelBuilder.replicate, builds SolverMuJoCo(use_mujoco_cpu=False) (GPU mjwarp,
separate_worlds auto), runs the exact deploy substep loop, and times it.

env-steps/s here = N_worlds * control_steps / wall_time  (matches the trainer's
metric: one control decision per env per step, SUBSTEPS physics sub-steps each).
NO CUDA-graph capture -> a conservative LOWER bound (the real trainer captures
and would be faster). Run from PowerShell (native warp/CUDA).
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import warp as wp
import newton

_REPO = next(_p for _p in Path(__file__).resolve().parents if (_p / "projects" / "policies").is_dir() or (_p / "AGENTS.md").exists() or (_p / ".git").exists())
sys.path.insert(0, str(_REPO))
from projects.policies.research.training.build_g1_native import (  # noqa: E402
    URDF, NOMINAL13, KE, KD, EFFORT, SPAWN_Z, DOF0, NJ, QPOS0)

SUBSTEPS = 4
DT = 0.016
SUB_DT = DT / SUBSTEPS


def _build_one():
    """One deploy-faithful G1 + its own ground (replicated per world)."""
    mb = newton.ModelBuilder()
    mb.add_urdf(str(URDF),
                xform=wp.transform((0.0, 0.0, SPAWN_Z), (0.0, 0.0, 0.0, 1.0)),
                floating=True)
    pv = int(newton.JointTargetMode.POSITION_VELOCITY)
    for d in range(DOF0, DOF0 + NJ):
        mb.joint_target_ke[d] = KE
        mb.joint_target_kd[d] = KD
        mb.joint_target_mode[d] = pv
        mb.joint_effort_limit[d] = EFFORT
        mb.joint_target_pos[d] = float(NOMINAL13[d - DOF0])
    for i in range(NJ):
        mb.joint_q[QPOS0 + i] = float(NOMINAL13[i])
    mb.add_ground_plane()
    return mb


def _run(N: int, steps: int = 120) -> None:
    g1 = _build_one()
    main_b = newton.ModelBuilder()
    main_b.replicate(g1, world_count=N, spacing=(3.0, 3.0, 0.0))
    model = main_b.finalize()
    wc = getattr(model, "world_count", "?")
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)
    state_a, state_b = model.state(), model.state()
    control = model.control()
    # OMNISIM_BATCH_NOCOLLIDE=1 skips collision -> measures the pure SolverMuJoCo
    # articulation+actuator step rate. The G1's URDF *mesh* collision overflows
    # the broad-phase when batched (8.7M tri-pairs/256 worlds > 1M buffer); a
    # real batched trainer uses PRIMITIVE collision (like the mjwarp MJCF), so
    # this isolates the solver cost from that separate model-prep issue.
    import os as _os
    contacts = None if _os.environ.get("OMNISIM_BATCH_NOCOLLIDE") else (
        model.contacts() if hasattr(model, "contacts") else None)
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_a)
    # Hold nominal (model.joint_target_pos already carries it per world).
    try:
        if hasattr(control, "joint_target_pos") and hasattr(model, "joint_target_pos"):
            control.joint_target_pos.assign(model.joint_target_pos.numpy())
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

    # Warmup (kernel compile + autotune outside the timer).
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
    print(f"N={N:5d} (world_count={wc}) steps={steps} | {T:5.2f}s | "
          f"{env_sps:9.0f} env-steps/s | {phys_sps:10.0f} phys-steps/s | finite={finite}")


def main() -> int:
    import os
    wp.init()
    print("[batch-throughput] BATCHED SolverMuJoCo (deploy solver) on GPU, no graph capture")
    ns = os.environ.get("OMNISIM_BATCH_N", "256,1024,2048,4096")
    for N in [int(x) for x in ns.split(",") if x.strip()]:
        try:
            _run(N)
        except Exception as e:
            print(f"N={N}: FAILED ({type(e).__name__}: {e})")
            break
    return 0


if __name__ == "__main__":
    sys.exit(main())
