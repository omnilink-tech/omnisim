# G1 Newton walk — one source of truth for trainer ↔ deploy

> 📍 **Canonical G1 status:** [rl-current-state.md](rl-current-state.md) (the *G1 — detail*
> section) is the single source of truth. If a status claim here disagrees with that file,
> that file is right — fix this one.

> **VERIFIED (2026-06-18).** Trainer and deploy now provably run the **same physics**
> (modulo the opt-in `OMNISIM_NEWTON_USE_LINK_COM` flag + documented representational /
> residual diffs). Proven three independent ways: a structural compiled-`MjModel` field
> diff (0 real-physics gaps), a GPU golden trajectory (8.5 mm first-10-tick base drift),
> and a live training run whose persisted physics config byte-matches the deploy
> spec 11/11. See [Tier-2 golden parity](#tier-2-golden-parity--measured-on-cloud-gpu-2026-06-18)
> and [the live-run check](#third-verification--a-live-h100-training-run-matches-the-deploy-spec-1111).

> **Training venue (read before copying anything below).** Training is **in-engine and
> LOCAL** — `projects/policies/training/run_walk_rl.sh`, or the standalone-but-still-local
> mjwarp trainers in `projects/policies/research/training/`. **There is no cloud path**
> (the `cloud/` Modal wrappers were removed in `ef46a52e`). The Tier-2 and third-verification
> sections below are *dated historical records* of runs made on rented GPUs in June 2026;
> they are kept as provenance for the parity evidence, not as a venue you can reproduce.
> Re-run any of it on the local GPU.

> **⚠️ Caveat learned 2026-06-18 — "same MODEL" ≠ "same LONG-HORIZON behavior".** The byte-match
> above is of the *model* (inertia, gains, ranges…) and the *first 10 ticks* (8.5 mm drift). On an
> inverted-pendulum biped that small per-tick drift **compounds**: the *same* policy walks 33.8 s
> in the `omnisim-bin` deploy but only ~7.3 s in the trainer's Newton env. So a policy that is
> stable/high-scoring in the trainer does **not** guarantee a durable deploy walk — see the
> durability correction in [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md). Single-
> source physics removed the *per-tick* mismatch; the *long-horizon* divergence of an unstable
> system is a separate, open problem.

> **⚠️ Corollary learned 2026-06-24 (H1) — matched solver + matched model is *still* not
> enough.** The thesis below is "import one physical model; never re-declare a constant."
> The H1 walk work showed the *same rule must extend past the model* — to the **initial
> condition (launch state)** and the **observation pipeline**. See
> [the H1 corollary](#2026-06-24--the-h1-corollary-matched-solver--matched-model-is-still-not-enough)
> below, [h1-walk-rl-journey.md](h1-walk-rl-journey.md), and [rl-current-state.md](rl-current-state.md).

### 2026-06-24 — the H1 corollary: matched solver + matched model is still not enough

The H1 walk fine-tuner (`gpu_newton_h1_walk_trainer.py`, commit `cf200cdc`) runs a Phase-2
policy fine-tune through `newton.solvers.SolverMuJoCo` — the **exact deploy solver**. That
got us two lessons that *extend* this doc's central rule beyond the model.

**Lesson 1 — "same solver" ≠ "matched physics".** The first version built the Newton model
**fresh from `h1.urdf`** (`add_urdf` + `add_ground_plane()`, which gives **newton's DEFAULT
ground friction**), *not* the deploy's μ=2.0. This is the H1 analogue of this doc's whole
thesis: a fresh URDF build silently uses different friction/contact params. The fix (commit
`da8b171a`) loads the **dumped deploy model** — `h1_legs_newton.mjcf.xml` (friction 2.0,
kp800/kd70 actuators, the real foot box) — via `newton.ModelBuilder.add_mjcf`
(`H1_TRAINER_MODEL=mjcf`). *Import the dumped model; never rebuild it from URDF.* The
mjwarp trainer already did this; the Newton fine-tuner had to be brought in line.

The model-match measurably improved fidelity **on survival**: run-3 batched cold-eval went
**1.26 s (URDF model) → 1.85 s (MJCF model)**, and at the deploy launch phase the batched
survival (**2.05 s**) matches the deploy (**2.03 s**).

**Lesson 2 — even matched model + matched solver is not sufficient.** Run 3 (mjwarp-trained)
and the Newton fine-tune are **byte-identical in the batched trainer** (both **2.05 s / -0.72 m**
at the launch phase) yet **diverge in deploy** (run3 **2.03 s FORWARD** vs ft **0.66 s
BACKWARD**). The batched trainer even walks **backward** where the deploy walks **forward** —
*same model, same solver*. The residual gap is two things the model byte-match does not cover:

- **(a) Initial condition.** The deploy's 0.3 s settle imparts a forward launch lean + a
  residual velocity that the batched reset does not have.
- **(b) Observation pipeline.** The deploy obs = world-frame `getVelocity()` + finite-diff
  `qd`, vs the trainer's exact MuJoCo-frame `qvel`. This is the **same class** as the
  historical G1 obs-frame bug (qvel body-frame vs `getVelocity` world-frame).

**Takeaway — extend the single source of truth past the model.** Byte-level model parity is
**necessary but not sufficient** for sim-to-deploy parity on a marginal biped. The "one
source" must also cover the **initial condition (launch state)** and the **observation
pipeline**, or two byte-identical models still walk in opposite directions.

**Status (2026-06-17): landed.** The G1 Newton **trainer** (`gpu_newton_g1_walk_trainer.py`,
runs on the local GPU) and the OmniSim **deploy** (the embedded-Python Newton backend,
`WbNewtonBackend.cpp`) now derive their physical model from **one place** instead of
re-declaring it on each side. This document is the map.

It is the structural follow-through on [g1-deploy-walk.md](g1-deploy-walk.md), whose
method — *"train in the deploy solver, **byte-match** the physics"* — worked but kept
the two sides in agreement **by hand** across ~20 parameters in 8+ files. That manual
parity was the root of months of "G1 collapses at ~1 s" debugging (a silent solver
fallback, 4 extra shoulder colliders, ground μ 2.0 vs 1.0, a missing joint clamp). This
replaces hand-matching with derivation + an enforcement gate.

---

## The architecture: 3 sources of truth + 1 enforcement gate

| Layer | The one home | Consumed by |
|---|---|---|
| **Geometry / inertia / colliders / per-joint limits** | the prim **URDF** (`g1_23dof_omnisim_prim.urdf`), read **live** | spec loader → trainer + deploy |
| **Tunable physics knobs** (solver, gains, friction, clamp, contract scalars) | [`projects/policies/research/backends/g1_physics.json`](../../projects/policies/research/backends/g1_physics.json) | spec loader |
| **The combine point** (knobs + URDF-parsed limits + joint order) | [`projects/policies/research/backends/g1_physics_spec.py`](../../projects/policies/research/backends/g1_physics_spec.py) | trainer, deploy launcher, conformance test |
| **Enforcement** (drift fails CI) | [`tests/test_g1_physics_spec_conformance.py`](../../tests/test_g1_physics_spec_conformance.py) + [CI](../../.github/workflows/g1-spec-conformance.yml) | every PR touching `projects/policies/**` |

Nothing on either side may re-declare a model parameter as a literal — it imports
`g1_physics_spec` (`import as SPEC`) instead. Joint **order** comes from the existing
`g1_robot_spec.JOINT_NAMES` (declared "FIXED FOREVER" there), so there is one source for
ordering too. Per-joint **position / velocity / effort limits are never transcribed**:
`SPEC.leg_limits()` parses them out of the prim URDF at import.

```
g1_physics.json ─┐
                 ├─► g1_physics_spec.py ─► trainer  (SPEC.KE, SPEC.SUBSTEPS, SPEC.leg_limits(), …)
prim URDF ───────┘                      └─► deploy   (projects/policies/research/runners/g1_deploy_launch.py → SPEC.newton_env())
                                        └─► conformance test (asserts both consumers == SPEC)
```

---

## What landed, by stage

All five commits sit on `main` above the foundation; every value migration was proven
`old == new` before commit (this is a **behavior-preserving** refactor — no retuning).

| Stage | Commit | What |
|---|---|---|
| Foundation | `fc9f11ff` | `g1_physics.json` + `g1_physics_spec.py`. Verified: URDF-parsed leg limits equal the deploy's old hardcoded `LIM_LO/HI` and the trainer's `vel_lim_t`; `newton_env()` reproduces the recipe env block. |
| 0 — persist config | `62caf2a8` | Trainer writes `runs/<name>/physics_config.json` = `SPEC.resolved()` + argv + parsed args + git SHA + prim-URDF sha256 at policy-save time. Closes the "no run config is ever saved" reproducibility hole (the `_pdoff_clamp` recipe used to live only in prose). |
| 1 — one model | `62caf2a8` | Foot collider box single-sourced from `SPEC.FOOT_BOX_*` across all 3 sites (trainer in-memory strip, `build_g1_native_prim`, `make_g1_deploy_prim_urdf`) so colliders can't drift. **The trainer→prebaked-URDF file switch was *declined*** — see "deferred" below. |
| 2 — one knob set | `62caf2a8` (trainer) + `be47d149` (deploy) | Trainer reads `KE/KD/SUBSTEPS/DT/SPAWN_Z/RES_SCALE`, clamp `lo/hi/vel` from SPEC. Deploy: new `projects/policies/research/runners/g1_deploy_launch.py` emits the `OMNISIM_NEWTON_*` env from `SPEC.newton_env()` (no more hand-pasted env wall); the deploy controller reads `LIM_LO/HI/ACT_SCALE/STEP_DT/NOMINAL/ARM_NOMINAL/NJ/OBS_DIM` from SPEC. |
| 3 — enforcement | `a2a99288` | 18-test conformance suite (16 pass / 2 skips: 1 GPU/Newton golden-trajectory, 1 gated on the deploy `controller` module import) + GitHub Actions CI on `projects/policies/**`. Tier 1 (CPU) pins every scalar + the full leg limit arrays and asserts both consumers equal SPEC; Tier 2 is the GPU golden-trajectory scaffold. |
| 4 — north star | `2b0b5cb8` | Canonical MJCF [`g1_23dof_omnisim.mjcf`](../../projects/robots/unitree/g1/mjcf/g1_23dof_omnisim.mjcf) generated from SPEC+URDF via `scripts/dev/make_g1_mjcf.py`, validated on MuJoCo CPU (gains byte-equivalent to ke=100/kd=5). See [g1-mjcf-single-model.md](g1-mjcf-single-model.md). |

---

## How to use it

**Deploy (watch it walk):**
```
python projects/policies/research/runners/g1_deploy_launch.py            # spec-driven env + the winning policy
python projects/policies/research/runners/g1_deploy_launch.py --print-env # show the resolved env without launching
```
Physics env comes from `SPEC.newton_env()`; per-policy gait/balance vars are launcher
defaults you can still override from the environment.

**Train:** unchanged entry point; the trainer now imports `SPEC` for its physics
constants. The recipe in [g1-deploy-walk.md](g1-deploy-walk.md) still applies.

**Change a physics knob for BOTH sides:** edit `g1_physics.json`, run the conformance
test, then retrain + redeploy to validate. Do **not** edit a literal on one side.

**Check for drift:** `pytest tests/test_g1_physics_spec_conformance.py -v`.

---

## Behavior-preserving vs. validation-gated

Everything above is behavior-preserving (proven `old == new`) **except** two north-star
switches that change the model build and therefore need real validation:

- **Trainer builds from the canonical MJCF** (`mb.add_urdf(strip)` → `mb.add_mjcf`):
  changes the model build → needs a **local GPU retrain** to confirm the 5.9 m / 33.8 s
  walker survives before flipping.
- **Deploy builds the robot from the MJCF** (instead of the Webots node tree): lives in
  compiled native code (`WbNewtonBackend.cpp`) → needs a **native rebuild**, which is
  currently blocked on this machine (Qt5/Qt6 link failure). Design-complete in
  [g1-mjcf-single-model.md](g1-mjcf-single-model.md); deferred.

### Deferred: trainer → prebaked-URDF file switch (Stage 1 ideal)
The cleanest end state is the trainer loading the same `g1_23dof_omnisim_prim.urdf` file
the deploy loads. It was **verified-and-declined**: `add_urdf(prim)` keeps 29 `<visual>`
meshes as non-colliding shapes; harmless for one deploy robot but `replicate()`'d to 2048
worlds it reproduces exactly the broad-phase blowup the in-memory strip exists to avoid
(collide-shape sets are *identical* — 2 foot boxes + ground — so dynamics match; only the
visual-shape count differs). Resolution is a **render-stripped prim URDF variant**, then
re-check including `shape_count`. The foot box is SPEC-sourced on both sides meanwhile, so
the collider set can't drift in the interim.

---

## Residuals (explicit, not silent)

Tracked in `g1_physics.json._residuals`:

- **`joint_limit_precision`** — the trainer's clamp uses 3-decimal limits (`-2.531`) the
  winning policy was trained against; the deploy + spec read full URDF precision
  (`-2.5307`). Gap ≤ 4e-4 rad (sub-milliradian), only at a joint pinned to its limit.
  Unify by retraining with `SPEC.leg_limits()`.
- **`contact_ke_kd`** — deploy applies `contact_ke=2500/kd=100`; the trainer uses Newton
  builder defaults. Kept side-specific so this refactor stays behavior-preserving; flip +
  retrain to close.
- **`njmax_nconmax`** — deploy single-robot auto-sizes (256); trainer pins 128 for the
  2048-env batch. Side-specific by necessity.
- **The velocity clamp / "zero velocity into a stop"** is not expressible in MJCF and
  stays a code-side parity path (the deploy's load-bearing post-step clamp), mirrored in
  the trainer behind `--train-joint-clamp`.

---

## Tier-2 golden parity — measured on cloud GPU (2026-06-18)

[`projects/policies/research/training/g1_golden_parity.py`](../../projects/policies/research/training/g1_golden_parity.py)
answers "do the trainer and deploy actually step the same physics?" numerically. It builds
the G1 three ways from the spec, compiles each to a MuJoCo `MjModel`, diffs every physics
field name-aligned, and steps each on GPU mjwarp with an identical action sequence:
- **P1 trainer** — the actual trainer builder (`_build_g1_full_prim_builder`)
- **P2 deploy** — the **literal** deploy runtime, extracted verbatim into
  [`g1_deploy_runtime.py`](../../projects/policies/research/backends/g1_deploy_runtime.py) from
  `kNewtonRuntimeSource` (a CI test, `tests/test_g1_deploy_runtime_sync.py`, keeps it byte-in-sync)
- **P3 mjcf** — the Stage-4 canonical MJCF

Run locally on a CUDA GPU (pinned warp 1.13.0 / newton 1.2.0 / mujoco 3.8.1 / mujoco-warp
3.8.0.3) via [`projects/policies/research/training/g1_golden_parity.py`](../../projects/policies/research/training/g1_golden_parity.py).
The P2 driver passed its trust gate (mass 34.1339 kg = sum of URDF links, 23 actuated joints, gains 100/5).

**Structural — Trainer (P1) vs Deploy (P2): bit-identical (`0.0`) on every dynamic field** —
inertia *tensors*, PD gains (`actuator_gainprm/biasprm`), force & ctrl ranges, `dof_damping`,
`dof_armature`, `jnt_range/axis/type`, `geom_friction`, `opt.timestep/gravity/cone/solver`.
The **only real difference** is `body_ipos`: the deploy places each link's COM at the link
origin (its `add_body` has no COM arg) vs the trainer's true URDF COM (≤ 0.154 m). Other
P1↔P2 diffs (`nbody` 34 vs 25, base `body_pos`, `body_mass` rollup) are representational —
total mass is conserved.

**Trajectory — 100 ticks, identical actions (base slice = rigorous metric):**

| pair | base-pos drift |
|---|---|
| **Trainer vs Deploy (P1↔P2)** | **0.35 m** |
| Trainer vs MJCF (P1↔P3) | 11.8 m |
| Deploy vs MJCF (P2↔P3) | 11.8 m |

Trainer and deploy are **~33× closer to each other than to the MJCF**. The 0.35 m drift is
the single COM difference amplified through an open-loop (no-balance) falling biped; it is
the leading cause and a concrete fix (see `_residuals.deploy_link_com`). The MJCF is the
outlier because of a generator bug (`_residuals.mjcf_mass_force_clamp`), not the live path.

**Verdict:** trainer and deploy compile to the **same MuJoCo model** save for the deploy's
COM-at-origin simplification — which the SoT now records as a tracked, fixable residual
rather than an unknown.

### Update (2026-06-18) — both gaps fixed and re-verified on GPU

- **MJCF** (`5cdc4bea`): `make_g1_mjcf.py` now sets `compiler.fusestatic=0` (the fixed-joint
  children were fused away with `saveinertial=0`, losing ~1.28 kg on serialization) and
  `forcelimited/ctrllimited=FALSE`. The canonical MJCF now matches the trainer (P1 vs P3)
  on **all 31 structural fields** (34.1339 kg, unbounded force/ctrl).
- **Deploy COM** (`24eeab2d`): the deploy `World.add_body` now accepts a COM
  (`newton add_link(com=…)`), passed by `WbSolid::addBody` when
  `OMNISIM_NEWTON_USE_LINK_COM` is set — **default off = legacy COM-at-origin, so every
  other Newton robot is unchanged**. With the flag the structural verdict is **"NO REAL
  PHYSICS GAPS"**: trainer ≡ deploy `body_ipos` PASS; only representational diffs
  (`nbody`, mass-rollup, base-pos encoding) remain.

Post-fix golden run (A10G, `--use-link-com`), trainer ↔ deploy (P1↔P2):

| metric | result |
|---|---|
| structural compiled `MjModel` | **0 real-physics gaps** |
| trajectory, **first 10 ticks** (chaos-robust) | **8.5 mm** base drift |
| trajectory, final 100 ticks | 0.49 m (passive-biped chaos, not model error) |

Trainer and deploy now track to **~8 mm** before the open-loop passive biped becomes
chaotic — the clean dynamical confirmation of model parity. (The P1↔P3 trajectory drift is
a harness artifact: the rollout drives via newton `joint_target_pos`, which the MJCF's
MuJoCo `ctrl` actuators ignore, so P3 is undriven; the structural diff is the definitive
P1≡P3 proof.)

**Still validation-gated (not bugs):** the `WbSolid.cpp` COM caller needs a native rebuild;
before defaulting `OMNISIM_NEWTON_USE_LINK_COM` on for the live G1 deploy, run a deploy
re-validation (the walker was tuned against COM-at-origin). The MJCF→build switch (trainer
and deploy) remains its own GPU/rebuild-gated step.

---

## Third verification — a live H100 training run matches the deploy spec (11/11)

The Tier-2 work above (structural diff + GPU golden trajectory) proves the *compiled
models* agree. The third leg proves the *live trainer process* agrees with the deploy
spec on disk, with no special harness.

A real H100 training run (run name `gpu_newton_g1_walk_smoke_pdoff`) ran clean —
`NaN_seen=False`, and it logged the two parity tells the deploy mirrors:
`[parity] post-step joint clamp ON in trainer (mirrors deploy)` and
`CUDA graph captured (4 substeps)`, with `base_z 0.780`. At policy-save time the trainer
persisted its Stage-0 `runs/<name>/physics_config.json` (see Stage 0 above), and that file
**byte-matches `SPEC.newton_env()` on all 11 checks**: `solver=mujoco_warp`, `substeps=4`,
`dt=0.016`, `ke=100`, `kd=5`, `ground_mu=1.0`, joint clamp on, use-inertia on, seed-pose on,
the same prim URDF path, and a recorded prim-URDF `sha256`.

So the agreement is not only "two builders compile to the same model" but "the actual
running trainer wrote a config that equals what the deploy launcher emits." This is the
reproducibility hole Stage 0 was created to close, now exercised end-to-end on real GPU
hardware. (As with Tier-2, full deploy-side COM parity is the opt-in
`OMNISIM_NEWTON_USE_LINK_COM` path; the live-run check above is on the physics knobs +
prim-URDF identity, which are flag-independent.)

---

## What "one model" still cannot give you

Even with a perfect shared model, the trainer carries domain randomization +
action-latency modeling that the deploy (a clean closed loop) does not — by design. So
trajectories are not bit-identical and **policies are still selected by a deploy rollout**,
not by training reward alone. The single source of truth removes *structural* drift
(the class of bug that cost months); it does not remove the intended train-time stochasticity.

## The other structural drift: runtime package versions

The model spec is not the only thing that must not diverge across the trainer↔deploy
boundary — the **physics-runtime package versions** must match too, or the "same physics"
claim quietly breaks the next time the deploy bundle is re-vendored. The shared stack
(`warp-lang`, `mujoco-warp`) is pinned once in
[`scripts/packaging/newton_runtime_pins.py`](../../scripts/packaging/newton_runtime_pins.py),
the deploy bundler vendors those exact versions (no more name-only `--upgrade`), and
[`tests/test_newton_pins_parity.py`](../../tests/test_newton_pins_parity.py) fails CI if
`projects/policies/research/training/requirements-train.txt` ever disagrees. Same rule as the model: **one place declares
the version, both sides import it, CI enforces it.**
