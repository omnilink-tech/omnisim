# Train ↔ Deploy unification — one engine, one loop

> **Status (2026-06-24):** Phase 0 (measure) and Phase 1 (shared obs/IC layer)
> **landed and verified**; Phase 2 (shared step core) is **already functional in
> Python** and **design-complete for the C++ promotion**, which is gated on the
> blocked native rebuild. This doc is the canonical map of *why* training and
> deployment look like two systems, what is actually shared, and the concrete
> path to one system. It composes with
> [g1-single-source-of-truth.md](g1-single-source-of-truth.md) (the *model*
> single-source) and [g1-mjcf-single-model.md](g1-mjcf-single-model.md).

## 1. The mental model — it was never two physics engines

Training and deployment step the **same physics**: DeepMind's `mujoco_warp`,
reached through NVIDIA Newton's `SolverMuJoCo` (both vendored at
`msys64/mingw64/bin/newton-runtime/site-packages/`, Newton 1.3.0 / mjwarp
3.10.0 / warp 1.14.0). Newton's `Model` is **inherently multi-world**
(`Model.world_count`, flat arrays sliced by `*_world_start` offsets); the deploy
runs it at `world_count=1`, the trainer at `world_count=N` via
`ModelBuilder.replicate(world_count=N)`. **It is the same constructor and the
same `step()` for N=1 and N=16384.**

So the "two systems" are not two engines. They are:

| | **Training** | **Deployment** |
|---|---|---|
| What it is | thin Python harness driving batched `mujoco_warp` on GPU | full C++ OmniSim engine ([`OmNewtonBackend.cpp`](../../src/omnisim/physics/OmNewtonBackend.cpp)) wrapping one Newton world |
| Worlds | N = 2048–16384 | 1 |
| Around the physics | obs/reward/reset/control as torch tensor ops | scene tree, controller-IPC, WREN render, sensors, supervisor |

The difference that matters for sim-to-real is the **loop around the physics**:
the observation pipeline and the initial condition. The structural + golden
parity work already proved the *model* is identical to within 0 real-physics
gaps; the documented **H1 corollary** is that a matched model + matched solver
*still* diverge because of the obs pipeline and the launch state. That is the
gap this effort closes.

## 2. The three layers (and what's shared in each)

```
  Layer A  PHYSICS MODEL ............... g1_physics_spec.py + g1_physics.json + prim URDF   [SHARED, CI-enforced]
  Layer B  OBS + INITIAL CONDITION ..... g1_env_core.py                                     [SHARED, CI-enforced — Phase 1]
  Layer C  STEP CORE (the World.step) .. g1_deploy_runtime.py  (← OmNewtonBackend.cpp)      [SHARED in Python; C++ flip gated — Phase 2]
```

- **Layer A** was already single-source: see
  [g1-single-source-of-truth.md](g1-single-source-of-truth.md). Enforced by
  `tests/test_g1_physics_spec_conformance.py`.
- **Layer B** is new (Phase 1, below): the obs vector and the launch state.
- **Layer C** is the per-step Newton physics. It is *already* a byte-synced
  importable extract; only the C++ self-containment flip remains (Phase 2).

## 3. Phase 0 — measure the gap (done)

[`projects/policies/research/training/g1_step_obs_parity.py`](../../projects/policies/research/training/g1_step_obs_parity.py)
runs the **deploy** `World.step()` path at `world_count=1` and the **trainer**
path at `world_count=N` under the same deterministic actions, and computes both
obs definitions from the *identical* physics state (zero physics confound). It
fills the Tier-2 TODO at `g1_golden_parity.py:357-369`. Findings:

- **Batching is physics-neutral.** N=1 vs N=8 qpos divergence `< 1e-4` — the
  central thesis, measured: deploy (N=1) and training (N) are the same physics
  code path.
- **Base linear velocity:** gap exactly `0` (both world-frame) — already
  unified.
- **Base angular velocity:** the headline 1.665 rad/s was a *harness artifact*
  (it rotated Newton `body_qd`, a different internal quantity). The **real
  deploy controller already rotates `R^T·getVelocity()` into the body frame**
  (the 2026-06-09 "OBS FIX" at
  [`g1_walk_deploy.py:759-773`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py)),
  matching the trainer's body-frame `qvel[3:6]`. First-tick-upright gap:
  `2.6e-05 rad/s` ≈ 0. **Already unified.**
- **Joint qd:** the genuine open divergence. The deploy *finite-differences* qd
  from joint positions (it has only position sensors); the trainer reads exact
  `qvel`. Raw-vs-finite-diff gap: `mean 0.347 rad/s`.
- **Initial condition:** the deploy settles ~0.3 s under gravity before tick 0
  (arriving leaned + moving); the trainer teleport-resets to pitch 0 / qvel 0.

So the genuinely open obs/IC divergences are exactly **(1) qd finite-diff** and
**(2) the launch IC** — not the physics, not the angular-velocity frame.

## 4. Phase 1 — the shared obs/IC layer (done, verified)

### `projects/policies/research/backends/g1_env_core.py`
The single source for the *loop around the model* (companion to
`g1_physics_spec`, the single source for the model). Backend-agnostic
(numpy for the deploy single-env path, torch for the batched trainer; one
implementation, dispatched on input type). Exports:

- `proj_gravity_from_quat` / `proj_gravity_from_matrix` — body-frame gravity,
  the two consumers' forms, proven equal.
- `world_ang_to_body_matrix(R, omega_world)` — the 2026-06-09 deploy R^T fix,
  codified once so it can't drift.
- `JointVelEstimator(dt, tau)` — the deploy's finite-diff(+low-pass) qd. **This
  is the canonical joint velocity both sides must use.**
- `assemble_walk_obs(...)` — the 50-d core obs in canonical order
  (`lin(3) | ang(3) | proj_g(3) | q-nom(13) | qd(13) | last_action(13) |
  gait(2)`), `CORE_OBS_DIM == SPEC.OBS_DIM == 50`.
- `settle_steps(dt)` / `nominal_full()` — the IC settle recipe.

### Enforcement
[`tests/test_g1_env_core_parity.py`](../../tests/test_g1_env_core_parity.py) —
15 tests, numpy-only-importable (torch checks self-skip), wired into
[`g1-spec-conformance.yml`](../../.github/workflows/g1-spec-conformance.yml).
Locks: layout==50, proj_gravity quat==matrix, frame round-trip, finite-diff
recovers a constant velocity, and numpy==torch bit-for-bit.

### Consumers
- **Deploy controller** ([`g1_walk_deploy.py`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py)):
  refactored to call `g1_env_core` for proj_gravity, the R^T ang-vel, and the
  finite-diff qd. **Numerically identical** (`max|diff| = 0` on all three).
- **Trainer** ([`gpu_mjwarp_g1_walk_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_walk_trainer.py)):
  behind a **default-off** `G1_ENV_CORE=1` flag, routes the obs qd through
  `JointVelEstimator` (matching deploy) and seeds the reset from a precomputed
  gravity-settle IC. Default path is byte-identical (the env-core fields are
  never created when the flag is off). Both paths smoke-run clean at obs dim 50.

### What the harness now proves
The shared `JointVelEstimator` reproduces the deploy's inline finite-diff to
`0.000e+00` (a faithful drop-in), and the trainer's raw-qvel-vs-finite-diff gap
it must close is `mean 0.347 rad/s`.

### Open item (gated)
The trainer's IC-settle currently produces a `-21°` lean (still falling at
`-1.67 rad/s` after 19 steps) on the stripped-foot trainer model, vs the deploy
World's `-2.66°`. The settle plumbing is correct, but the trainer model settles
*differently* from the deploy model — reconcile (model/seed-pose differences)
before relying on it. **This is exactly why the obs/IC switch is gated behind a
local GPU retrain** that confirms the 5.9 m / 33.8 s walker survives, identical to
how the single-MJCF switch is gated.

## 5. Phase 2 — the shared step core

### It already works in Python
[`g1_deploy_runtime.py`](../../projects/policies/research/backends/g1_deploy_runtime.py) is a
**byte-verbatim, sync-tested, importable** extract of the deploy's entire
Newton physics (`OmNewtonBackend.cpp`'s `kNewtonRuntimeSource` string). The
trainer **already drives the exact deploy `World` step-for-step at N=1** through
it (`g1_golden_parity.build_deploy_model()` + the parity harness:
`add_body`/`finalize`/`set_joint_target_pos`/`step`/`body_xform`/`body_vel`/
`get_joint_angle`). The N=large equivalent is the `gpu_newton_*` trainer
(`replicate(world_count=N)` + `SolverMuJoCo`). **The functional unification —
"train through the deploy step" — is real today.**

### What remains: the C++ self-containment flip (rebuild-gated)
Today the **C++ string is canonical** and `g1_deploy_runtime.py` is the
generated mirror (`_gen_deploy_runtime.py` slices the `R"PY(...)PY"` block;
`test_g1_deploy_runtime_sync.py` asserts byte-equality). True single-source
means flipping the direction: make the shared `.py` canonical and **generate
the C++ literal from it**.

- **Recommended: build-time codegen (option A).** Keep the binary
  self-contained (no new runtime file dependency, no `sys.path` work — the
  embedded interpreter comes up via bare `Py_InitializeEx(0)` with default
  paths), flip the source of truth to the `.py`, and add a build rule that
  regenerates the `.cpp` literal from it before compiling. The existing
  byte-equality guard already protects against staleness. A reverse generator
  is included (`_gen_deploy_runtime.py --write-cpp`, dormant) ready for this.
- **Rejected: runtime load (option B) — ⚠ the rejection needs re-arguing.**
  Reading the `.py` from disk removes the rebuild requirement but breaks
  self-containment and adds a silent-ODE-fallback failure mode (missing file /
  `OMNISIM_HOME` skew). Not worth it. ⚠ **2026-08-08: the main cited cost is
  gone.** `bdc02139` deleted the ODE backend, so a missing file / `OMNISIM_HOME`
  skew can no longer degrade quietly to ODE — it is now a hard failure, which is
  exactly the loud-and-early behaviour the rejection was trying to buy. **The
  decision is NOT flipped here**; it should be re-argued on its remaining merits
  (self-containment of the binary, no new runtime file dependency, no `sys.path`
  work, and the three-copy lockstep problem in "Caveats" below), which are
  independent of the deleted backend.

### Caveats for the flip
- There are **three** verbatim copies to keep in lockstep: the `.cpp` literal,
  `g1_deploy_runtime.py`, and the out-of-process `newton_embed_smoke.cpp` copy.
  The reverse generator must update both `.cpp` files.
- The **frozen C++↔Python contract** (must be preserved by any promotion): 29
  `World` methods + the `_solver_kind`/`_solver_error` attributes. Full list in
  the Phase-2 investigation notes; the build/control/readback method names in
  `g1_env_core`/`g1_deploy_runtime` must never change signature without updating
  the C++ call sites.

### The rebuild gate (quoted)
> "the change lives in compiled native code (`OmNewtonBackend.cpp`), so it
> requires a **native rebuild**, and the worktree/isolated native build is
> blocked on this machine (C:\\msys64 Qt5/Qt6 coexistence link failure)."
> — [g1-mjcf-single-model.md](g1-mjcf-single-model.md)

So the C++ flip is **design-complete and tooled, blocked only on the rebuild.**

## 6. The gates (do not skip)

1. **obs/IC switch (Phase 1 default-on):** gated on a local GPU retrain that
   confirms the deploy walker survives with `G1_ENV_CORE=1`. The qd finite-diff
   is the high-confidence part; the IC settle needs the §4 reconciliation first.
2. **C++ step-core flip (Phase 2):** gated on the native rebuild (Qt5/Qt6).

Both are deliberate: the speed⟂fidelity split stays intact (fast raw-`mjw` bulk
training, faithful `SolverMuJoCo` fine-tune), and policies are selected by
deploy rollout, not trainer reward.

## 7. How to verify

```bash
# Layer A + B conformance (CPU, what CI runs)
python -m pytest tests/test_g1_physics_spec_conformance.py tests/test_g1_env_core_parity.py -v

# Deploy-runtime byte-sync (Layer C is still .cpp-canonical today)
python -m pytest tests/test_g1_deploy_runtime_sync.py -v

# The obs/IC gap measurement (GPU)
python projects/policies/research/training/g1_step_obs_parity.py --ticks 60 --envs 8

# Static + golden physics parity (GPU)
python projects/policies/research/training/g1_golden_parity.py
```

## 8. Open work, in priority order

1. **Reconcile the trainer IC-settle with the deploy settle** (§4) so the
   launch lean matches (`-2.66°`, not `-21°`).
2. **Local GPU retrain with `G1_ENV_CORE=1`** and measure deploy survival — flip
   the obs/IC default on success.
3. **Native rebuild → run `_gen_deploy_runtime.py --write-cpp`** to flip Layer C
   to `.py`-canonical and rebuild.
4. **Generalize** `g1_env_core` beyond G1 (H1, OmniQuad, Go2 share the same obs/IC
   pattern — see [h1-walk-rl-journey.md](h1-walk-rl-journey.md)).
