# G1 single MuJoCo model (MJCF) — the Stage-4 north star

> 📍 **Canonical G1 status:** [rl-current-state.md](rl-current-state.md) (the *G1 — detail*
> section) is the single source of truth. If a status claim here disagrees with that file,
> that file is right — fix this one.

**Status (2026-06-17): artifact landed + CPU-validated; both consumer switches
designed, neither flipped.** This note describes how one canonical MJCF becomes
the single physical model for *both* the G1 walk trainer and the OmniSim deploy,
why MJCF is the right representation, and exactly what gates each side's switch.

## The artifact

`scripts/dev/make_g1_mjcf.py` generates one file:

```
projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf
```

derived deterministically from the two existing sources of truth — the physics
knobs in `projects/policies/research/backends/g1_physics.json` (read through
`g1_physics_spec`) and the geometry/inertia/limits in the prim URDF
(`projects/robots/unitree/g1/urdf/g1_23dof_omnisim_prim.urdf`). It captures the
whole physical model in one native file:

| In the MJCF | Source |
|---|---|
| bodies, masses, inertials, the 23-DOF joint tree (+ free root) | prim URDF |
| collision geoms = the 2 foot boxes only | `SPEC.FOOT_BOX_SIZE` / `SPEC.FOOT_BOX_ORIGIN` |
| per-joint position limits (`range`) + effort caps (`forcerange`/`actuatorfrcrange`) | `SPEC.urdf_limits` / the URDF |
| PD actuators (kp=`SPEC.KE`, kv=`SPEC.KD`) | `SPEC.KE`=100 / `SPEC.KD`=5 |
| per-dof `armature` | `SPEC.ARMATURE` (0.0) |
| ground plane `friction[0]` | `SPEC.GROUND_MU` (1.0) |
| `<option timestep>` = `SPEC.DT/SPEC.SUBSTEPS` (0.004) | `SPEC.DT`=0.016 / `SPEC.SUBSTEPS`=4 |

CPU validation (`python scripts/dev/make_g1_mjcf.py --verify`) loads it with
plain `mujoco.MjModel.from_xml_path` and asserts every one of those against the
spec: **nq=30, nv=29, njnt=24, nu=46**, all 23 ranges, gains, armature, friction,
and timestep PASS. mjwarp/CUDA are not required to validate.

### The gain mapping (the load-bearing detail)

Both the trainer and the deploy drive every revolute joint as a Newton
`JointTargetMode.POSITION_VELOCITY` actuator with `target_ke=100`, `target_kd=5`
(the deploy via `OMNISIM_NEWTON_TARGET_KE/KD`; the trainer via
`mb.joint_target_ke[d]/joint_target_kd[d]`). Newton's `SolverMuJoCo` is what
actually compiles a `mujoco.MjModel` and steps it on both sides, and it expands
**one** POSITION_VELOCITY dof into **two** MuJoCo `general` actuators
(`newton/_src/solvers/mujoco/solver_mujoco.py`, the POSITION_VELOCITY branch):

```
position actuator:  gainprm=[ke,0,…]  biasprm=[0, -ke,  0, …]   # kp spring
velocity actuator:  gainprm=[kd,0,…]  biasprm=[0,   0, -kd, …]  # kv damper
```

`make_g1_mjcf.py` emits exactly that two-actuator-per-joint pair (`<joint>_pos`
with `gainprm="100" biasprm="0 -100"`, `<joint>_vel` with `gainprm="5"
biasprm="0 0 -5"`), so the MJCF's joint drive is **byte-equivalent** to the
articulation SolverMuJoCo builds today on both sides — not the coarser single
combined-affine actuator in `projects/policies/research/backends/_urdf_to_mjcf.py`. This is the
crux of why one file can serve both: the gains are not re-approximated, they are
the same actuators the engine already builds.

## (1) Trainer path — `add_urdf` + in-memory strip → `add_mjcf` from this file

Today `gpu_newton_g1_walk_trainer.py::_build_g1_full_prim_builder` (and the
13-DOF `build_g1_native_prim.build_g1_prim_builder`) parse the prim URDF in
memory with ElementTree, strip every `<visual>` + mesh `<collision>`, re-inject
the two foot boxes, serialize back to an XML string, and call:

```python
mb.add_urdf(urdf_xml, xform=…, floating=True)
# then per-dof: mb.joint_target_ke[d]=ke; joint_target_kd[d]=kd;
#               joint_target_mode[d]=POSITION_VELOCITY
mb.add_ground_plane()
```

The north-star replacement is to delete that strip-and-reinject dance and build
the same articulation from the committed MJCF:

```python
mb.add_mjcf(str(MJCF_PATH), xform=…, floating=True)   # newton.ModelBuilder.add_mjcf
# ground + gains + armature already in the MJCF
```

`newton.ModelBuilder.add_mjcf` exists and ingests MuJoCo XML directly (it even
reads `<option>` via `parse_mujoco_options=True`), so the MJCF — which already
carries the foot-box-only collision set, the PD actuators, armature, ranges and
the ground plane — becomes the single input. The strip logic, the per-dof gain
loop, and `add_ground_plane()` all collapse into the one file.

**Why this is staged AFTER the URDF-unification and gated on a retrain:** moving
the trainer from `add_urdf`+manual-gains to `add_mjcf` is a *behaviour change* —
even when the resulting `mujoco.MjModel` is intended to be identical, the import
path differs (collision-group masking, joint-actuator ordering, contact defaults)
and the G1 is an inverted pendulum at the stability edge where any
trainer↔deploy drift topples it. So the switch must be validated by a **local
GPU retrain** of the winning recipe (`gpu_newton_g1_walk_ft_pdoff_clamp`) and a
fresh deploy run, compared against the 5.9 m / 33.8 s baseline — not asserted
from a CPU load. Until that retrain is green, the trainer keeps its current
`add_urdf` route and this MJCF is a parallel, validated artifact.

## (2) Deploy path — body-by-body articulation → load from this MJCF

The deploy's `WbNewtonBackend` (C++ + embedded Python,
`src/omnisim/physics/WbNewtonBackend.cpp`) builds the Newton articulation
**body-by-body** from the Webots node tree: it walks the scene graph, and for
each motorized hinge calls `self.builder.add_joint_revolute(...)` with
`target_ke`/`target_kd`/`actuator_mode=POSITION_VELOCITY` (the
`OMNISIM_NEWTON_TARGET_KE/KD` overrides), then hands the finalized model to
`SolverMuJoCo`. The north-star replacement is to load the robot from the
canonical MJCF instead of reconstructing it from nodes — the MJCF already encodes
the same bodies, gains, limits, foot colliders and friction the backend hand-builds.

**Why this is design-complete but deferred:** the change lives in compiled
native code (`WbNewtonBackend.cpp`), so it requires a **native rebuild**, and the
worktree/isolated native build is blocked on this machine (C:\msys64 Qt5/Qt6
coexistence link failure; see the engine-migration / worktree-build project
notes). The mapping is fully specified (the MJCF's actuator pairs are exactly
what the backend's `add_joint_revolute(target_ke=…, target_kd=…)` produces via
SolverMuJoCo, proven above), so the deploy-from-MJCF switch is design-complete
and lands when the native build is unblocked. Until then the deploy keeps its
body-by-body builder; the MJCF and the existing path are provably equivalent at
the actuator level.

## (3) Why MJCF is the right north star

Both sides **already** run newton `SolverMuJoCo` / mjwarp — that is the whole
point of the G1 walk recipe (train in the exact deploy solver; see
`docs/developer/g1-deploy-walk.md`). SolverMuJoCo's internal representation *is*
a `mujoco.MjModel`. So MuJoCo's own MJCF is the engines' native format, and a
single MJCF can hold geometry + inertia + gains + friction + limits + armature in
one file with no lossy intermediate. The two sides differ today only in *how they
get to that MjModel* (trainer: URDF→Newton→MuJoCo; deploy: nodes→Newton→MuJoCo);
making MJCF authoritative removes the two import routes in favour of one file
both ingest. URDF cannot play this role: it has no actuator/gain concept, no
`<option timestep>`, and no friction model — those would still have to be
re-applied in code on each side, which is exactly the drift this refactor closes.

## (4) The residual one model still won't fix

A single MJCF unifies the *static* physical model. It does **not** close the
remaining dynamic trainer↔deploy residual, which is in the *loop around* the
model, not the model itself:

- **Domain randomization & control-latency:** the trainer runs with DR (push
  forces, action-latency 0–3 ticks, action-gain jitter, init tilt/vel bands) so
  the policy is robust; the deploy runs a single clean, latency-free loop. The
  trainer deliberately trains a *harder* problem than deploy presents.
- **The post-step joint clamp** (clamp qpos to URDF limits, qvel to
  ±velocity_limit, zero velocity driving into a stop) is applied in *code*
  (`WbNewtonBackend.cpp`, mirrored by the trainer's `--train-joint-clamp`). It is
  **not expressible as a static MJCF property** — MuJoCo has no per-joint qvel
  cap attribute — so even with one MJCF this clamp must stay a matched code path
  on both sides (it was the decisive parity lever; see `g1-deploy-walk.md`). The
  MJCF carries the *position* limits (`range`) and *effort* caps (`forcerange`);
  the *velocity* limit and the stop-driving rule remain code-side parity items.
- **DR vs clean-loop is the intended gap, not a bug:** per `g1-deploy-walk.md`,
  the residual policy's *deviation* from the open-loop ghost gait is what keeps
  the biped upright, and the imitation reward fights balance. One model removes
  model mismatch; it cannot remove the deliberate difference between the trainer's
  randomized world and the deploy's deterministic one.

So the canonical MJCF is the right and sufficient fix for *model* drift — and a
non-fix, by design, for the *control-loop* residual.

## Regenerate

```
python scripts/dev/make_g1_mjcf.py            # write the MJCF
python scripts/dev/make_g1_mjcf.py --verify   # write + CPU-validate vs SPEC
python scripts/dev/make_g1_mjcf.py --check-only   # validate an existing MJCF
```

Re-run after any edit to the prim URDF or `g1_physics.json`. The build is
deterministic (same inputs → identical bytes).
