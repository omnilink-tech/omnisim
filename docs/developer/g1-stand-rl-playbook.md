# G1 standing — the full playbook

> ⚠️ **SUPERSEDED 2026-06-10 — the deploy stand is SOLVED; the "t ≈ 1.55 s then falls"
> claims below are PRE-FIX.** The OmniSim Newton deploy now **stands indefinitely** via a
> deterministic **pure pose** (deeper-squat NOMINAL: hip −0.30 / knee 0.52, ankle PD off;
> commit `f48f00b7`) — *not* the RL policy this playbook chased. The historical RL analysis
> below (the 8-iteration journey, the `mjw.step` ≠ `SolverMuJoCo.step` gap, the more-DR /
> ground-friction dead ends) is kept as the postmortem, but wherever it presents
> **t ≈ 1.55 s** as *current* deploy behaviour, read it as the **pre-2026-06-10 state**. For
> the authoritative solved status, see [`rl-current-state.md`](rl-current-state.md) (canonical,
> re-verified 2026-06-19) and `rl-journey.md` §2.

How we got from "G1 can't stand for a second" to a robust standing policy — ≈98 % survival in the mujoco_warp trainer, and (at the time) an OmniSim Newton deploy that held to **t ≈ 1.55 s** (a characterized 2026-05-29 limitation, **since superseded** — see the banner above and the "Floor-contact regression" note below) — and the recipe to replicate it. Companion to [`spot-residual-rl.md`](spot-residual-rl.md) (the quadruped recipe this one diverges from). Always-current cross-robot status: [`rl-current-state.md`](rl-current-state.md).

## Bottom line

|                                            | Before                | After                  |
|--                                          |--                     |--                      |
| OmniSim Newton survival                    | 0.77 s, robot tips    | **44 s (2026-05-28); 1.55 s pre-fix** ⚠️ (deploy stand SOLVED 2026-06-10 — holds indefinitely via deterministic pure pose, not this RL policy) |
| Training rate (RTX 5070)                   | 14 k env-steps/s      | **132 k env-steps/s**  |
| Wall clock for 30 M-step PPO run           | ~28 min               | **3 min 43 s**         |
| Engineering iterations to get here         | 0                     | **8 substantive commits** |

The trained policy lives at [`projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx`](../../projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx). The deploy controller picks it up automatically (`G1_POLICY_ONNX` env override available).

> **⚠️ The "44 s, no fall" row is a 2026-05-28 / pre-`d56cbf5` figure (and the trainer's in-sim survival).** At the time of writing the OmniSim Newton deploy stood to **t ≈ 1.55 s** then lost balance — see ["Floor-contact regression (2026-05-29)"](#floor-contact-regression-2026-05-29--read-this-before-deploying) below. **That 1.55 s limitation was superseded 2026-06-10 (`f48f00b7`): the deploy now stands indefinitely via a deterministic pure pose (not this RL policy).** This table records the historical RL-policy figures, not today's deploy behavior (see `rl-current-state.md`). The `Training rate (RTX 5070)` / `132 k env-steps/s` / `3 min 43 s` figures are from the original training box and are **unconfirmed on this checkout's hardware** (an RTX 3060 Laptop measured ~27–62 k env-steps/s; a 29.5 M-step run took ~7.9 min). Cross-robot status: [`rl-current-state.md`](rl-current-state.md).

## What this doc is

The Spot pipeline (quadruped, gait + IK + tiny joint residual, 20 k PPO steps, transfers cleanly to OmniSim) doesn't directly port to a bipedal humanoid. We tried for a day, hit five real sim-to-deploy gaps in a row, and the recipe that finally worked is structurally different from Spot's.

This is the postmortem so the next person attacking a similar problem doesn't burn the same day.

## The dead ends, in order

Each row is a real iteration we ran, what we measured, and what we (wrongly) concluded.

| # | Hypothesis                                                       | What we tried                                                                | Result                  | What we learned |
|---|------------------------------------------------------------------|------------------------------------------------------------------------------|-------------------------|-----------------|
| 1 | "Spot's residual recipe will just work on G1."                  | Analytic baseline = NOMINAL pose + ankle PD; tiny ±0.15 rad residual.        | Robot tips in 0.5 s.    | Bipedal baseline doesn't stand on its own. There's no scaffold for residual RL to sit on. |
| 2 | "Train from scratch with PPO. CPU is fine."                      | SB3 PPO, single env, 1 M steps on CPU MuJoCo.                                | 81/500 steps survival. | CPU is too slow. 1 M steps barely starts to learn for a 23-DoF humanoid. |
| 3 | "Move to GPU mujoco_warp. Same setup, more samples."             | Wrote `gpu_mjwarp_g1_stand_trainer.py`; ran 30 M steps at 14 k env-steps/s.  | 99.6 % survival in MuJoCo, **0.77 s in OmniSim**. | Trains a beautiful MuJoCo policy that doesn't transfer to deploy. Classic sim-to-deploy gap. |
| 4 | "OmniSim Newton uses a different solver — flip to mujoco_warp."  | `OMNISIM_NEWTON_FORCE_MUJOCO=1 OMNISIM_NEWTON_MJWARP=1` on the deploy world. | Deploy: **0.94 s**.    | Solver match alone isn't enough. There are layers below the solver name. |
| 5 | "Actuator gains differ. Match them."                              | `OMNISIM_NEWTON_SAVE_MJCF=…` dumped the model Newton actually builds. It has `kp = 20, kv = 3` — my MJCF had `kp = 400, kv = 20`. Rebuilt MJCF with matched gains, retrained. | Deploy: **0.94 s**.    | 20× joint stiffness gap is real and real bad. But matching it alone didn't move the dial much. |
| 6 | "Env-step timing differs (16 ms vs 16 ms but composed differently)." | `SUBSTEPS = 8`, 2 ms physics dt → matches OmniSim's 16 ms env-step exactly.   | Deploy: **0.94 s**.    | Time-scale match matters for the baseline's roll-rate computation, but wasn't the dominant gap. |
| 7 | "Observation pipeline differs. Use `getVelocity()` not finite-diff." | Deploy controller now reads pelvis 6-DOF velocity directly. Joint qd also fixed (was silently zero!). | Deploy: **0.94 s**.    | Real bug found and fixed — but the policy is still ineffective. The action isn't doing what training thinks it does. |
| 8 | "Train on the EXACT MJCF Newton builds — joint axes, inertias, all of it." | `OMNISIM_NEWTON_SAVE_MJCF` dump → `import_newton_mjcf.py` to rename anonymous bodies/joints. Train on that. | Deploy: **0.96 s**.    | Even bit-identical MJCFs aren't enough. The trainer's `mjw.step()` vs deploy's `SolverMuJoCo.step()` wrapper differ in a way no MJCF tweak can close. |

At iteration 8 we'd matched everything we knew to match and the gap was still wide open. The Spot team's [`newton_solver_trainer.py`](../../projects/policies/research/training/newton_solver_trainer.py) docstring named this exact thing:

> *"plain `mujoco` mj_step and `mujoco_warp` both diverge from `SolverMuJoCo.step()` — verified: a policy trained on plain-mujoco hits 0.5 m/s there but stands still under SolverMuJoCo; gait-only even flips sign."*

So we knew the symptom — what we didn't know was the *creative* answer.

## The pivot: stop trying to make them match, train to be invariant

> "It is good that you updated your memory, but I want you to doc our journey for all OmniSim users to learn from."
> — user, 2026-05-28

The real fix isn't physics-matching. It's domain randomization aggressive enough that the deploy wrapper's quirks fall inside the training distribution. This is the sim-to-real path the broader RL community has converged on (OpenAI Dactyl, NVIDIA's humanoid locomotion, Boston Dynamics' Atlas locomotion). We just needed to apply it here.

### The DR cocktail that worked

| Knob                         | Old (mild)                    | New (heavy)                 |
|--                            |--                             |--                           |
| Body mass                    | ±15 % shared across bodies    | **±30 % per body**          |
| Ground friction              | ±30 %                         | **±50 %**                   |
| Joint damping                | ±30 % shared                  | **±50 % per DOF**           |
| **Actuator kp**              | **none (fixed kp = 20)**      | **±40 % per actuator**      |
| **Actuator kv**              | **none (fixed kv = 3)**       | **±40 % per actuator**      |
| Gravity                      | none                          | **±5 %**                    |
| External pushes (prob)       | 0.5 % / step                  | **2 % / step**              |
| External pushes (max v)      | 0.4 m/s                       | **1.5 m/s**                 |
| Observation noise            | 1 % gaussian                  | **3 % gaussian**            |
| **Action latency**           | **none**                      | **0 - 3 random ticks per env** |
| Initial joint q              | ± 0.05 rad                    | **± 0.15 rad**              |
| Initial base xy              | ± 0.03 m                      | **± 0.05 m**                |
| Initial base z               | none                          | **± 0.02 m**                |

The four bold rows are the ones that specifically address the deploy-wrapper drift. Especially:

- **Actuator kp/kv jitter** — Newton's `SolverMuJoCo.step()` applies controls through an extra buffer. The effective torque per commanded delta drifts from what raw `mjw.step()` would compute. Training over a ±40 % band of gains makes the deploy's actual operating point in-distribution.
- **Action latency** — Newton's wrapper has a 1-tick control-buffer indirection. Sampling a uniform 0 - 3 tick delay per env means the policy never depends on its actions taking effect *this exact step*.

### Why the wrapper drift matters for a biped but not a quadruped

For Spot, four contact points + static stability → control loop is over-determined, small wrapper drift gets soaked up by the kinematic redundancy. The Spot residual works in OmniSim despite the same `mjw.step ≠ SolverMuJoCo.step` issue.

For G1, two small foot contacts + inverted-pendulum dynamics → control loop is at the stability margin. One-tick delay + 5 % torque scaling = enough to push it from "barely standing" to "tipping in 1 second." See [`humanoid-balance-gap.md`](humanoid-balance-gap.md) for the deeper dynamics analysis.

## The 5 GPU speedups (so DR-rich training is actually fast)

Heavy DR + 30 M-step PPO would be useless if each training run took 28 minutes (which is what the pre-speedup trainer did). We stacked five optimizations to get to 3 min 43 s:

| # | Speedup                                              | What it does                                                                                                                                                                                                  | Speedup factor (this trainer) |
|---|------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------|
| 1 | **Actor on GPU**                                     | `ac.to(cuda)`, rollout buffers are cuda tensors. Eliminates the `numpy() ↔ torch.from_numpy()` round-trip on every policy call.                                                                              | ~2 ×                          |
| 2 | **GPU-native env operations**                        | `wp.to_torch()` gives zero-copy torch views of `mw_d.qpos / qvel / ctrl`. Baseline PD, obs vector, reward, termination — all run in torch on cuda. **No CPU↔GPU traffic in the hot loop.**                  | ~2 ×                          |
| 3 | **CUDA graph capture for the physics loop**           | `wp.capture_begin/end` around `for _ in range(SUBSTEPS): mjw.step(...)`, then `wp.capture_launch(graph)` per env-step. Cuts the ~200 kernel launches per substep into one graph replay.                       | ~1.5 ×                        |
| 4 | **Bigger N (envs)**                                  | `--envs 4096` instead of 2048; `--rollout 12` instead of 24 (keeps total samples per update constant). GPU saturates better at larger batch.                                                                  | ~1.5 ×                        |
| 5 | **SUBSTEPS = 4 × dt = 4 ms** (was 8 × 2 ms)          | Same 16 ms env-step but half the physics ticks. mujoco_warp stays stable for human-scale dynamics at 4 ms; the 2 ms was conservative.                                                                          | ~2 ×                          |

Combined ≈ **9 ×** (14 k → 132 k env-steps/s). The numbers multiply roughly because each removes a distinct bottleneck.

### Where each speedup lives

All implemented in [`projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py):

- `__init__` → `wp.to_torch(...)` views (speedup #2)
- `_try_capture_graph()` (speedup #3)
- `step()` → all-torch hot path (#1, #2)
- `main()` → all-cuda rollout buffers (#1)
- argparse defaults `--envs 4096 --rollout 12` (#4)
- `PHYS_DT = 0.004` + `SUBSTEPS = 4` (#5)

### Prerequisites

- **CUDA-enabled PyTorch.** The default `pip install torch` gives a CPU-only build that bricks speedup #1.

  ```bash
  pip install --upgrade --force-reinstall --no-deps torch \
      --index-url https://download.pytorch.org/whl/cu128
  ```

  Verify with `python -c "import torch; print(torch.cuda.is_available())"`.

- **NVIDIA Warp + mujoco_warp + Newton.** All pre-installed on this checkout; if they're not, `pip install warp-lang mujoco_warp newton-physics`.

## The recipe to replicate end-to-end

### 1. Dump the deploy MJCF (one-off)

```bash
OMNISIM_HOME=$PWD \
OMNISIM_URDF_USE_INERTIA=1 \
OMNISIM_NEWTON_FORCE_MUJOCO=1 \
OMNISIM_NEWTON_MJWARP=1 \
OMNISIM_NEWTON_SAVE_MJCF=$PWD/_scratch/g1_newton_exact.mjcf.xml \
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/worlds/g1_stand_deploy.wbt &
# wait until "world finalised" appears in the log, then close OmniSim.
```

### 2. Rename anonymous joints/bodies to readable names

```bash
python projects/policies/research/training/import_newton_mjcf.py
# writes projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml
```

### 3. Train

```bash
python projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py \
    --envs 4096 --iters 600 --rollout 12 \
    --save projects/policies/research/training/runs/gpu_g1_stand_robust/policy.pt
```

All heavy-DR defaults are baked in. ~3 min 43 s on RTX 5070. Outputs `policy.pt` + `policy.onnx`.

### 4. Eval in MuJoCo

```bash
python projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py \
    --eval --envs 512 --eval-steps 2500 \
    --save projects/policies/research/training/runs/gpu_g1_stand_robust/policy.pt
```

Expect ≥ 95 % survival across 512 envs × 20 s. Heavy DR will cause occasional falls — that's the policy proving it can recover, not a failure.

### 5. Deploy in OmniSim

```bash
OMNISIM_HOME=$PWD \
OMNISIM_URDF_USE_INERTIA=1 \
OMNISIM_NEWTON_FORCE_MUJOCO=1 \
OMNISIM_NEWTON_MJWARP=1 \
OMNISIM_NEWTON_STATICS=1 \
OMNISIM_NEWTON_SUBSTEPS=4 \
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/worlds/g1_stand_deploy.wbt
```

Watch the world load — should see `world finalised` and then the controller log `OK ONNX` every second. **As of 2026-05-29** the RL-policy robot stood (`bz ≈ 0.79`, `OK`) up to `t ≈ 1.55 s`, then lost balance — see the floor-contact regression note above for why. **This 1.55 s limit was superseded 2026-06-10 (`f48f00b7`): the shipped deploy stand is now a deterministic pure pose that holds indefinitely (see `rl-current-state.md`); the RL-policy recipe here is the historical path.**

> **Headless works under `--minimize` (updated 2026-05-29).** `--no-window` consistently hangs at joint-2 of 13 registrations (a wrapper-event-loop interaction), but `--minimize --batch --no-rendering` keeps a normal Qt event loop and registers all 13 hinges + steps reliably — verified 6/6 headless runs via `scripts/dev/headless_runner.py`. **Launch from a native Windows shell (PowerShell), not MSYS2 bash**, or the embedded interpreter misses `warp` (user site-packages) and silently falls back to ODE — see [`engine-migration-plan.md`](engine-migration-plan.md) §12. GUI mode also works and is the easiest way to watch it.

## The six required env vars (deploy side)

| Variable                             | Why                                                                                                       |
|--                                    |--                                                                                                         |
| `OMNISIM_URDF_USE_INERTIA=1`         | Without this OmniSim's URDF importer discards `<inertial>` tags. The robot loads with degenerate inertias and is unbalanceable. |
| `OMNISIM_NEWTON_FORCE_MUJOCO=1`      | Picks `newton.solvers.SolverMuJoCo` instead of the XPBD default — the only solver our trainer matches. |
| `OMNISIM_NEWTON_MJWARP=1`            | Inside `SolverMuJoCo`, uses GPU mujoco_warp (not CPU `mj_step`). Same engine the trainer runs.            |
| `OMNISIM_NEWTON_STATICS=1`           | **Added 2026-05-29; empirically required (3/3 vs 3/3 A/B, see floor-contact note).** Without it this deploy world *finalises but never steps* — the controller loads the ONNX and then the sim never advances. With it, G1 steps and stands. The mechanism is **not** "registers the RectangleArena floor" (it doesn't — the arena's collider is a nested `Floor`/`Plane` the statics dispatch skips): the working hypothesis is that the unconditional Newton ground plane (`WbNewtonBackend::ensureWorldOpen → addGroundPlane`) reaches the **XPBD** path but not the **forced-`SolverMuJoCo`** deploy model, so `STATICS=1` is what gives the MuJoCo model a ground the feet can rest on. The wheeled husky (XPBD, no FORCE_MUJOCO) drives fine *without* this knob — it's specific to the forced-MuJoCo legged-deploy path. |
| `OMNISIM_NEWTON_SUBSTEPS=4`          | **Added 2026-05-29.** Sub-steps the contact solve. With ground contact present, a single 16 ms contact solve NaN-explodes (`bz → 1e4`) at first foot contact; `SUBSTEPS≥2` removes the explosion. |
| (optional) `G1_POLICY_ONNX=<path>`   | Override the default policy lookup; useful for A/B testing checkpoints.                                   |

The `g1_stand_deploy.wbt` world cannot set these itself — they are read by the C++ Newton backend at world-finalize, before the Python controller starts — so they **must** be exported in the shell that launches `omnisim-bin.exe` (see recipe step 5).

### Floor-contact regression (2026-05-29) — read this before deploying

The canonical "stands 44 s, no fall" result in the Bottom line above was measured *before* the Newton chassis-freeze fix (`d56cbf5`, 2026-05-29). That fix made `WbSolid::flushPendingNewtonRegistrations` skip Solids with no `Physics` node — correctly stopping the static `RectangleArena` floor and `OmniSimSunMarker` from being registered as *dynamic* Newton bodies (the bug that froze every robot at spawn). The side effect surfaced specifically on this **forced-`SolverMuJoCo`** deploy world.

**Empirical A/B (rebuilt binary, headless `--minimize`, 2026-05-29):**

| `OMNISIM_NEWTON_STATICS` | `SUBSTEPS` | Result (3 trials each) |
|---|---|---|
| unset | 4 | world finalises (all 13 hinges, `world finalised`) but **never steps** — controller stalls right after `loaded ONNX`; 0/3 stepped |
| `1` | 4 | **steps and stands** at `bz ≈ 0.791` ("OK") to `t = 1.0 s`; 3/3 reproduced |

So `OMNISIM_NEWTON_STATICS=1` is **required for this world to run at all** — but the mechanism is *not* the obvious "re-register the arena floor." A read-only multi-agent code audit (and direct tracing) showed the `RectangleArena` collider is a nested `Floor` Solid with a `Plane` boundingObject that the statics dispatch skips (`upperPose()` gate; no `WbPlane` case in `attachNewtonShapeFromBoundingObject`), **and** that `WbNewtonBackend::ensureWorldOpen()` adds an *unconditional* z=0 ground plane on every Newton world open. The reconciling evidence: the wheeled **husky** (XPBD default, no `FORCE_MUJOCO`) drives fine *without* `STATICS`, so that unconditional ground plane reaches the **XPBD** path — but the **forced-`SolverMuJoCo`** deploy model evidently does not get it, and `STATICS=1` is what supplies a ground the MuJoCo solver sees. (Exact mechanism inside `SolverMuJoCo` model-build not yet pinned; the requirement itself is reproducible.)

**Residual, still open:** with `STATICS=1 SUBSTEPS=4` the robot stands then loses balance at a deterministic **t ≈ 1.55 s** (identical for substeps 4 and 8 — a genuine balance gap, not a solver blowup). The deploy-time contact dynamics differ enough from the trainer's that the heavy-DR policy (validated 2026-05-28) no longer holds past ~1.5 s.

**More-DR retrain does NOT fix it (empirical, 2026-05-29).** Retrained the standing policy with bumped invariance DR (friction 0.5→0.7, per-body mass 0.3→0.4, actuator kp/kv 0.4→0.5, push vmax 1.5→2.0; 600 iters / 29.5 M steps; in-sim survival **98.6 %**, mean 2466/2500). Deployed (`STATICS=1 SUBSTEPS=4`, via `G1_POLICY_ONNX`): **FALL@1.47 s plus a contact-solve coordinate explosion** (`bz → 1.6e3 … 1.1e5 m`) — *worse* than the shipped policy's 1.55 s. So heavier DR yields a more-aggressive policy that *destabilizes* the deploy contact solve; it does not bridge the gap. This confirms the gap is the **structural `mjw.step` ≠ `SolverMuJoCo.step` contact difference** (lesson #2 below), not DR coverage. (Shipped `gpu_g1_stand_robust` left untouched; the experimental policy was discarded.)

**Ground friction is NOT the lever either (empirical, 2026-05-29).** Deployed the shipped policy at `OMNISIM_NEWTON_GROUND_MU` = 1.0 / 1.5 / 2.0 — all three give the *identical* `FALL@1.55 s`. If the feet were slipping, the failure time would shift with mu; it doesn't. So the gap is a deterministic control/dynamics divergence at a fixed t≈1.55 s, independent of ground friction. (Separately: once the robot tips, the deploy contact solve *explodes* — `bz → 1e5 m` — in every run, even at `SUBSTEPS=4`; a post-fall deploy-contact-robustness issue distinct from the balance gap.)

**Two cheap levers now ruled out (more-DR, ground-friction); the gap is the structural `mjw.step` ≠ `SolverMuJoCo.step` deploy-wrapper drift.** Path forward needs deploy-side work: train *inside* the OmniSim deploy wrapper (so the policy sees the real deploy dynamics) rather than raw `mjw.step`, or fix the wrapper's contact-conversion/state-sync drift. Tuning friction/substeps/DR will not close it. Tracked as P6 + P8.2 follow-up in [`engine-migration-plan.md §13.3`](engine-migration-plan.md) (P6 row).

**Train-in-deploy-solver feasibility (assessed 2026-05-29 — multi-session, not a quick swap).** The "train on the same solver the deploy uses" infra partly exists: [`newton_solver_trainer.py`](../../projects/policies/research/training/newton_solver_trainer.py) runs PPO on `newton.solvers.SolverMuJoCo` directly. But three real obstacles make this a dedicated effort, not a tail-of-session change:
1. **Throughput.** Its own docstring: SolverMuJoCo is **~1456 steps/s single-env and does NOT GPU-batch** — it parallelises via multiprocess CPU workers (~10× slower than the `mjw.step` GPU trainer's 132 k env-steps/s). A 30 M-step heavy-DR run is ~40+ min (feasible, but each iteration is slow).
2. **It's Spot-specific (residual).** The G1 heavy-DR cocktail + G1 env/obs/reward must be ported onto it (the current `gpu_mjwarp_g1_stand_trainer.py` is built around `mjw.step` + CUDA-graph capture).
3. **Faithfulness still uncertain.** `newton_solver_trainer` uses `SolverMuJoCo(use_mujoco_cpu=True)` (CPU `mj_step`), while the **deploy** uses `SolverMuJoCo` + **MJWARP (GPU)** (`OMNISIM_NEWTON_MJWARP=1`). There may STILL be a cpu-vs-mjwarp residual inside SolverMuJoCo — the truly faithful trainer is SolverMuJoCo+MJWARP batched, which is a further build. Until proven, even this path's gap-closure is a hypothesis.

So **as analysed on 2026-05-29**, getting the *RL policy* to "stand forever" looked like a scoped multi-session RL-infra effort. At that time the deploy stood to ~1.55 s (a documented, characterized limitation — NOT a regression). **This was overtaken on 2026-06-10 (`f48f00b7`): the deploy stand was solved a different way — a deterministic deeper-squat pure pose that holds indefinitely — so the RL-infra effort below was not the path actually taken. See `rl-current-state.md` (canonical) and `rl-journey.md` §2.**

**Turn-key recipe for the faithful G1 trainer (API-verified 2026-05-29, ready to execute as a dedicated effort).** Everything below was confirmed against the installed Newton 1.2.0 + the deploy source (`WbNewtonBackend.cpp` lines 780–989):

1. **`build_g1_native(mu, ke=20, kd=3)`** — model the legs-only G1 as a NATIVE Newton articulation (mirrors [`spot_native.py`](../../projects/policies/research/training/spot_native.py); `add_mjcf` is NOT usable — its MuJoCo actuators ignore `control.joint_target_pos`). Two options: (a) hand-build 13 links from the URDF `<inertial>`/`<geom>` (verbatim, like spot_native), or (b) `mb.add_urdf("projects/robots/unitree/g1/urdf/g1_legs_omnisim.urdf", floating=True)` then set each actuated revolute's `target_ke=20`, `target_kd=3`, `actuator_mode=newton.JointTargetMode.POSITION_VELOCITY`. **DONE — option (b) is implemented + verified in [`build_g1_native.py`](../../projects/policies/research/training/build_g1_native.py)** (2026-05-29): after `add_urdf(floating=True)` the DOF arrays are `[6 free | 13 revolute]`, so DOFs 6–18 are the actuated joints (set `joint_target_ke/kd/mode` + `joint_effort_limit` there); `joint_q[7:20]` seeds `NOMINAL_POSE[:13]`; `mb.add_ground_plane()`.

**Milestone-1 RESULT (significant finding).** The native model + the EXACT deploy solver (`SolverMuJoCo(use_mujoco_cpu=False)`, mjwarp) simulate **faithfully** — gains engage (pelvis held ~0.7 m through the first ~0.4 s), gravity + foot contact respond smoothly, **no NaN / explosion** (foundation gate PASS). BUT passive NOMINAL-hold (no policy) **topples at ~1.5 s** — and that lands right on the deploy's **t≈1.55 s** gap. A static squat under soft `ke=20/kd=3` is an unstable inverted pendulum; a biped *cannot* passively hold it. So **the 1.55 s gap is largely inherent instability the policy must ACTIVELY overcome, not purely `mjw.step`≠`SolverMuJoCo.step` wrapper drift** — the deploy policy stands only marginally longer than passive-hold, i.e. its active balance is nearly ineffective at deploy time. This *reinforces* the fix: train the balance policy ON this solver (where the passive instability is real) so it learns corrections that actually hold here. Foundation for that trainer is now in place + verified.
2. **Solver = the deploy backend exactly:** `newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=False)` (GPU mjwarp — what `FORCE_MUJOCO=1 + MJWARP=1` builds).
3. **Step = the deploy path verbatim** (`WbNewtonBackend` step): `control.joint_target_pos[qd_dof] = NOMINAL + ACT_SCALE*action`; then `for _ in range(SUBSTEPS=4): state_a.clear_forces(); model.collide(state_a, contacts); solver.step(state_a, state_b, control, contacts, dt/4); swap(state_a, state_b)`. `dt=0.016`.
4. **Obs/reward** = mirror `g1_stand_env.py` (48-dim: pelvis lin/ang vel, proj-gravity, joint_q−NOMINAL, joint_qd, last_action). Heavy-DR cocktail = the §"DR cocktail" table above.
5. **Batch on GPU** (SolverMuJoCo+mjwarp supports batched models) for throughput; else multiprocess like `newton_solver_trainer.py` (~10× slower).
6. **Deploy-verify** each candidate via the headless `--minimize` recipe above (`G1_POLICY_ONNX` override); success = stands past the pre-fix 1.55 s. *(Historical: the deploy stand was ultimately solved 2026-06-10 by a deterministic pure pose, not by this RL trainer — see banner at top.)*

**Open risk that gates the whole effort:** even SolverMuJoCo-mjwarp training may leave a residual vs the deploy's *substepped* SolverMuJoCo path; if it does, the remaining lever is matching the substep/contact-conversion details exactly. Prove the milestone-1 hold-NOMINAL first before investing in the full trainer.

**P7 motor-feedback gap (assessed 2026-05-29): stays a documented no-op, no implementation needed.** The only `getTorqueFeedback()` consumers are `husky_random` (reads it but explicitly "trace-only visibility; not used" for decisions) and the `motor` device sample (prints it). Nothing *acts* on Newton motor feedback, so the documented ODE-bridge-proxy behavior on Newton-backed joints breaks no real controller. Reopen only if a caller starts depending on the value.

## What we learned that generalizes

These are the takeaways that apply to *any* humanoid-class sim-to-deploy RL pipeline in OmniSim, not just G1 standing.

1. **The Spot residual recipe doesn't port unchanged to bipeds.** Single-foot support + inverted-pendulum dynamics break the "baseline almost works at zero policy" assumption Spot's pipeline rests on. Either build a real biped baseline (whole-body ZMP control) or train pure RL with deploy-aware DR.
2. **`mjw.step` is not the same as `SolverMuJoCo.step`.** Documented in the Spot team's `newton_solver_trainer.py` docstring; missed by every iteration before #8. The trainer's `mjw.step` is the *inner* call, but the deploy uses `SolverMuJoCo.step()` which wraps it with control-application, contact-conversion, and state-sync that drift the dynamics enough to break a stability-margin policy.
3. **Train to be invariant to the gap, not match it.** The amount of engineering required to *match* every quirk of the deploy wrapper grows without bound. DR collapses the same problem into "make the gap a sample from your training distribution."
4. **DR is only feasible if training is cheap.** 30 M steps with heavy DR is only practical because we got the trainer to 132 k env-steps/s. The 5 speedups aren't optional — they're what makes the robust path tractable.
5. **The dump-the-deploy-MJCF trick (`OMNISIM_NEWTON_SAVE_MJCF`) is genuinely useful.** Even though training on it alone doesn't close the gap, it lets you (a) verify your training MJCF is in the right ballpark, (b) bootstrap a new robot without re-deriving inertias, (c) catch joint-axis differences that URDF imports might silently rotate. Run it once, save the output, train on it.
6. **`--no-window` and `--minimize` take different Newton-finalize code paths.** `--no-window` deterministically hangs G1 at joint-2 (a wrapper-event-loop interaction); `--minimize --batch --no-rendering` keeps a live Qt event loop and finalizes + steps reliably (verified 6/6 headless, 2026-05-29). Prefer `--minimize` for headless Newton runs (it's what `scripts/dev/headless_runner.py` uses); GUI works too. Separately, launch from a **native Windows shell** so the embedded interpreter finds `warp` (else silent ODE fallback).

## File map

| File                                                                                                       | Purpose                                                                          |
|--                                                                                                          |--                                                                                |
| [`projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py) | The trainer (all 5 speedups + heavy DR).                                         |
| [`projects/policies/research/training/import_newton_mjcf.py`](../../projects/policies/research/training/import_newton_mjcf.py)           | One-off: Newton's anonymous MJCF dump → readable joint/body names.               |
| [`projects/policies/research/training/build_g1_mjcf.py`](../../projects/policies/research/training/build_g1_mjcf.py)                     | Legacy — builds MJCF from URDF. Superseded by `import_newton_mjcf.py` for any robot OmniSim deploys. |
| [`projects/policies/research/envs/g1_stand_env.py`](../../projects/policies/research/envs/g1_stand_env.py)                               | Gymnasium env (CPU SB3 baseline; trainer is GPU mujoco_warp instead).             |
| [`projects/policies/research/envs/g1_stand_viewer.py`](../../projects/policies/research/envs/g1_stand_viewer.py)                         | MuJoCo viewer for the trained policy (debug-look-at-it tool).                    |
| [`projects/policies/research/controllers/g1_stand_deploy/g1_stand_deploy.py`](../../projects/policies/research/controllers/g1_stand_deploy/g1_stand_deploy.py) | OmniSim deploy controller. Loads the ONNX, applies baseline + residual.        |
| [`projects/policies/research/worlds/g1_stand_deploy.wbt`](../../projects/policies/research/worlds/g1_stand_deploy.wbt)                   | Deploy world.                                                                    |
| [`projects/robots/unitree/g1/urdf/g1_legs_omnisim.urdf`](../../projects/robots/unitree/g1/urdf/g1_legs_omnisim.urdf) | URDF (legs-only variant — full-body arm chain hangs Newton).                  |
| [`projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml`](../../projects/robots/unitree/g1/urdf/g1_legs.mjcf.xml) | The exact MJCF Newton builds (saved via `OMNISIM_NEWTON_SAVE_MJCF` + renamed).   |
| [`projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx`](../../projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx) | The shipped robust policy.                                                      |

## Commit trail

The full sim-to-deploy journey is reconstructable from these commits, in order:

| Commit       | What                                                                                          |
|--            |--                                                                                             |
| `e80163fe`   | g1: legs-only URDF + diagnostic logs for Newton joint-registration hang                       |
| `2a36e701`   | g1/rl: from-scratch PPO standing trainer + Gymnasium env (CPU baseline)                       |
| `e3c92a9b`   | g1/rl: OmniSim deploy controller + world for the PPO standing policy                          |
| `834ad0d0`   | g1/rl: close three sim2sim gaps for OmniSim Newton deploy (solver, gains, timing)            |
| `ab1c4aca`   | g1/rl: sync deploy controller with env's residual recipe                                      |
| `c545c937`   | g1/rl: heavy domain randomization for sim-to-deploy robustness                                |
| `79b6013d`   | g1/rl: GPU-native trainer — 5 stacked speedups vs the prior version                          |
| `806753dc`   | g1/rl: ROBUST G1 STANDING POLICY — sim-to-deploy gap closed via heavy DR                     |

## See also

- [`sim-to-deploy-rl-recipe.md`](sim-to-deploy-rl-recipe.md) — generalized version of this recipe for any robot.
- [`spot-residual-rl.md`](spot-residual-rl.md) — the quadruped pipeline this one diverges from.
- [`humanoid-balance-gap.md`](humanoid-balance-gap.md) — the prior analysis of why the Spot recipe wouldn't port. Now resolved.
- [`rl-accelerated-training.md`](rl-accelerated-training.md) — GPU mujoco_warp infrastructure for batched training.
