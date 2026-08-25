# projects/policies/ — OmniSim control & policy pipeline

> ## ▶ DIRECTION (2026-07-04, owner checkpoint): SHADOWING is the flagship for legged motion
>
> OmniSim makes legged-robot **motion** policies by **Shadowing** — training **in-engine**
> (train == deploy bit-exact) to shadow an **achievable, recorded ghost** via corridors +
> WBMATCH + GHOST-MORPH. It is the canonical method and lives in [`training/`](training/README.md).
> Validated end-to-end on the G1 **on the documented lambda=0.9 weight-bearing balance harness,
> not free-standing**: a live-verified durable in-engine walk — durability champion
> `training/runs/wr_showpiece.pt` (45.6 m / 101 s / 0 falls) and style champion
> `training/runs/wr_calm_champion.pt` (WBMATCH 0.908 vs the owner-approved reference). Always-current
> canonical status: [rl-current-state.md](../../docs/developer/rl-current-state.md) — read it before
> quoting any robot result.
>
> **Deterministic control remains the shipped path for STATIC stand / balance / push-recovery**
> (not walking): [`controllers/humanoid_stand_deploy/`](controllers/humanoid_stand_deploy/) is a
> stiff-hold + capture-point + arm/hip-balance controller (**no learned policy**, specs
> `g1_armsdown.json` / `g1_oneleg.json`) that absorbs cubes thrown from every side and, in one-leg
> mode, shifts weight onto a single foot. It is also Shadowing's launch/settle layer. A measured A/B
> once showed that adding learned feedback to the *static stand* made it worse — so static balance
> stays deterministic; **motion** is Shadowing.
>
> **`research/`** holds the earlier, now **de-prioritised** RL track (heavy-DR pure PPO,
> residual-on-gait, per-robot trainers). From that era, OmniSim-trained **quadruped** walks
> (OmniQuad/Go2/B2) and **re-hosted Unitree** humanoid policies are real and deploy; the old
> *from-scratch humanoid* attempts hit a ~1.4 s tip wall and are **superseded** by the Shadowing
> walk above. Nothing in `research/` is imported by the shipped controllers. (This folder was
> `projects/rl/` — renamed to reflect the control/policy scope.)
>
> **⭐ PUBLIC POLICY BRAIN** (`omnisim.policy` + [`skills/`](skills/) · [docs/developer/skill-library.md](../../docs/developer/skill-library.md)):
> the standard packaging of Shadowing + BATON. Each skill (walk / turn / carry / stand / climb, + H1 /
> Go2 / OmniQuad) is ONE versioned manifest binding its ghost + `ghost_validator` verdict + deploy env +
> champion checkpoint + provenance; `python skills/skill_lib.py sequence <name>` composes them into a
> BATON demo, and `verify-demos` proves the manifests reproduce the demo scripts. **To make a new
> skill or a new demo, start in `skills/`** — the trainer/deploy stack underneath is unchanged.
> The stable front door is `python -m omnisim policy ...`; the historical
> `python skills/skill_lib.py ...` path remains compatible. Pure algorithms and contracts
> (BATON, MotionIR, typed skill graphs, and artifact promotion) live in the installable
> `omnisim.policy` package. Robot-specific assets remain under `projects/policies`.
> `python -m omnisim policy audit` is the single release gate. Living end-to-end acceptance
> thresholds and log scoring live in [`benchmarks/`](benchmarks/); benchmark records name the
> verified benchmark records name the machine, engine, and support configuration so performance
> claims remain resolvable; candidate cases remain visibly uncertified.

---

## Layout

```
projects/policies/
├── ghosts/        ⭐ THE GHOST STORE — every achievable reference, partitioned BY ROBOT
│   ├── g1/          ghost_*_lut.json (+ dance/)      ├── h1/     ├── go2/
│                    A lut DECLARES ITS OWN IDENTITY (schema 2): `robot` + `joints` (its real
│                    joint names) are REQUIRED, and `validator` carries the stamped verdict.
├── training/       ⭐ SHADOWING — flagship in-engine motion trainer (train == deploy)
│                     g1_walk_recipe.py (humanoid) + quad_walk_recipe.py (quadruped) +
│                     compatibility BATON import + ghost_validator.py +
│                     run_walk_rl.sh + runs/<champion>.pt
├── controllers/    OmniSim controllers spawned per robot
│   ├── humanoid_stand_deploy/   THE deterministic static-balance controller (+ specs/)
│   ├── ghost_hologram/          the robot-agnostic ghost PLAYER (reads the store above)
│   └── cube_thrower/            throws cubes at the robot under test
├── control/        deterministic, model-based control LIBRARY (no learned policy)
│   ├── *.py          per-robot kinematics / balance / raibert / recovery / mpc
│   ├── gait/         reference-gait generators + gait/tools/ + gait/datasets/
│   └── planner/      trajectory-optimisation (g1_sitstand_trajopt)
├── worlds/         shipped stand + walk + ghost worlds (g1/h1/valkyrie)
├── skills/       ⭐ SKILL LIBRARY — manifest per skill (ghost+ckpt+deploy-env+provenance);
│                     skill_lib.py (audit/benchmark/train/run/sequence) + humanoid/ quadruped/
├── benchmarks/   release thresholds + fail-closed scoring of machine-readable deploy evidence
├── motions/      robot-independent MotionIR catalogue; manifests provide robot bindings
├── common/         the shared law: robot_registry (what a robot IS) + shadow_env (the ONE env
│                     contract) + env_fingerprint
└── research/       earlier RL work — de-prioritised track, BUT see the honesty note below
    ├── controllers/  RL train/deploy controllers      ├── training/  old trainers + runs/
    ├── worlds/       RL train/deploy worlds            ├── inference/ ONNX export + policies/
    ├── envs/  backends/  shadowing/  runners/  tools/  policies/
```

The split is the strategic line: **`training/` (Shadowing) is the flagship for motion; `control/` +
`controllers/humanoid_stand_deploy/` + `worlds/` is the deterministic track for static
stand/balance; everything under `research/` is the earlier, de-prioritised RL track.**

> ⚠️ **Honesty note on `research/` (2026-07-13).** This README used to end that paragraph with
> *"Nothing in `research/` is imported by the shipped controllers."* **That is not true, and
> pretending it is hides the real shape of the tree:** every non-G1 skill in the library —
> `h1_walk`, `go2_walk`, `go2_shadow_walk`, `omniquad_walk` — ships its checkpoint, world and/or
> deploy controller out of `research/`. The top-level tree is, in practice, the **G1's** tree
> (34 of 38 worlds and 11 of 11 demo scripts are G1). `research/` is therefore a *maturity*
> boundary that the *robots* cut straight across. Generalizing Shadowing means moving those
> artifacts up, not restating the claim.

## The rules that keep this generalizable

Three conventions, each ENFORCED by a tool rather than by hoping:

1. **A ghost declares what it is FOR.** `robot` + `joints` are required keys
   (`skills/ghost_lut.py`). Nothing may fall back to "it's a G1, positionally" — that guess is
   how an H1 ghost got limit-checked against a G1 URDF, and how a 12-wide *quadruped* lut looked
   exactly like a 12-wide *humanoid* one.
2. **A ghost that cannot be model-checked FAILS.** `ghost_validator.py` fails closed on an
   unregistered robot (it used to skip every model-aware gate and still print `VERDICT: PASS`).
   Verdicts are STAMPED INTO the lut (`--stamp`) and the skill manifest is cross-checked against
   the stamp, so a manifest can no longer overclaim its own ghost.
3. **Names carry the robot.** A single-robot skill is `<robot>_<skill>` (`g1_walk`, `go2_walk`);
   a genuinely multi-robot skill is `<skill>` (`balance_two_legs`). The **method is a field**
   (`method: shadowing | residual-rl | unitree-rehost`), never something you branch on by
   parsing a name. Enforced in `skills/manifest.py`.

## Deterministic control (shipped for static stand / balance)

The one validated, deployable result is a **deterministic humanoid balance controller**:
[`controllers/humanoid_stand_deploy/`](controllers/humanoid_stand_deploy/). It is a stiff
position-PD hold (per-robot `ke`/`kd`) + capture-point/LIPM ankle regulation + arm/hip balance
+ a return-to-home integral — **no learned policy**. It absorbs cubes thrown from every side,
returns to upright after each hit, and (one-leg mode) shifts its weight onto a single foot.

Per-robot behaviour is data, not code: each robot/mode is a JSON spec under
[`controllers/humanoid_stand_deploy/specs/`](controllers/humanoid_stand_deploy/specs/) —
`g1.json` / `h1.json` / `valkyrie.json` (plain stand), `g1_armsdown.json`
(cube-defense, arms hanging + return-to-home), `g1_oneleg.json` (one-leg weight-shift). The
model-based primitives the controllers compose (kinematics, gait, balance) live in `control/`.

Run it via the launcher (resolves the world in `worlds/` and the controller natively):

```powershell
# G1 cube-defense, arms down, returns to upright after each hit
scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -Throw -ArmsDown -Gui -Duration 30
# G1 one-leg weight-shift
scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -OneLeg -Gui -Duration 30
# Plain deterministic stand (h1 / valkyrie also supported)
scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1 -Duration 10
```

`-Robot {g1,h1,valkyrie}`, `-Throw` / `-Rain` (cube assault), `-ArmsDown`, `-OneLeg`,
`-Gui` (windowed; omit for headless), `-Duration <s>`.

---

## RL research track (de-prioritised — see DIRECTION above)

Two RL recipes exist; pick by robot class:

| Robot class                       | Recipe                                              | Doc                                                                            |
|--                                 |--                                                   |--                                                                              |
| **Bipeds, manipulators, anything stability-margin**  | Heavy-DR pure PPO on GPU mujoco_warp  | [sim-to-deploy-rl-recipe.md](../../docs/developer/sim-to-deploy-rl-recipe.md) (general) + [g1-stand-rl-playbook.md](../../docs/developer/g1-stand-rl-playbook.md) (case study) |
| Quadrupeds (OmniQuad, ANYmal-class)   | Residual RL on a model-based controller (gait+IK)   | [omniquad-residual-rl.md](../../docs/developer/omniquad-residual-rl.md)                |

⚠️ **These do NOT deliver a deployable from-scratch humanoid.** The heavy-DR recipe trains
G1 standing to ≈98 % in-sim but the OmniSim Newton deploy tips ~1.4 s; the residual recipe
only works for statically-stable quadrupeds. The honest, always-current status of every
robot is in **[docs/developer/rl-current-state.md](../../docs/developer/rl-current-state.md)** —
read it before trusting any "stands/walks forever" claim.

**The standard policies shipped:**

| Robot | Policy                       | Result                                       | Trainer                                                                                                          |
|--     |--                            |--                                            |--                                                                                                                |
| OmniQuad  | `omniquad_residual_main`         | 5.55 m / 30 s straight walk                  | [`train_residual.py`](research/training/train_residual.py)                                                                |
| G1    | `gpu_g1_stand_robust`        | Robust stand: **≈98 % in trainer; OmniSim Newton deploy holds to t ≈ 1.55 s today** ([rl-current-state.md](../../docs/developer/rl-current-state.md)) | [`gpu_mjwarp_g1_stand_trainer.py`](research/training/gpu_mjwarp_g1_stand_trainer.py) (default GPU + heavy DR + 5 speedups) |

Full documentation: [docs/developer/archive/rl-pipeline.md](../../docs/developer/archive/rl-pipeline.md)

## TL;DR — biped / new robot (default recipe)

```bash
# Verify CUDA torch is installed (one-off; see playbook if it isn't).
python -c "import torch; assert torch.cuda.is_available()"

# Dump the deploy MJCF (one-off per robot — capture the exact model
# Newton builds at runtime; closes most sim-to-deploy gaps at once).
OMNISIM_HOME=$PWD \
OMNISIM_URDF_USE_INERTIA=1 \
OMNISIM_NEWTON_FORCE_MUJOCO=1 \
OMNISIM_NEWTON_MJWARP=1 \
OMNISIM_NEWTON_SAVE_MJCF=$PWD/_scratch/g1_newton_exact.mjcf.xml \
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/research/worlds/g1_stand_deploy.omniworld
# (close OmniSim once "world finalised" appears in the log.)

# Rename Newton's anonymous joints/bodies to readable names.
python projects/policies/research/training/import_newton_mjcf.py

# Train. ~3 min 43 s on RTX 5070 for 30 M-step PPO with heavy DR.
python projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py \
    --envs 4096 --iters 600 --rollout 12 \
    --save projects/policies/research/training/runs/gpu_g1_stand_robust/policy.pt

# Eval in MuJoCo (deterministic — heavy DR still on; expect ~98 %).
python projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py --eval \
    --envs 512 --eval-steps 2500 \
    --save projects/policies/research/training/runs/gpu_g1_stand_robust/policy.pt

# Deploy. GUI mode required (Newton finalize hangs in --no-window).
OMNISIM_HOME=$PWD \
OMNISIM_URDF_USE_INERTIA=1 \
OMNISIM_NEWTON_FORCE_MUJOCO=1 \
OMNISIM_NEWTON_MJWARP=1 \
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/research/worlds/g1_stand_deploy.omniworld
```

## TL;DR — quadruped (OmniQuad, legacy)

```bash
# Validate the env wiring works
python projects/policies/research/envs/validate_env.py

# Train (4 envs, 500k steps, ~25 min wall on a modest box)
python projects/policies/research/training/train_omniquad.py --envs 4 --steps 500000 \
       --run-name omniquad_ppo_main

# Convert .zip -> ONNX
python projects/policies/research/inference/export_onnx.py \
       --model projects/policies/research/training/runs/omniquad_ppo_main/omniquad_final.zip \
       --out   projects/policies/research/inference/policies/omniquad_ppo_main/policy.onnx

# Smoke-test the policy headless
python projects/policies/research/tools/smoke_deploy.py \
       --policy projects/policies/research/inference/policies/omniquad_ppo_main/policy.onnx

# Run with GUI
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/research/worlds/omniquad_rl_deploy.omniworld
```

## research/ subdirs

| Dir | Purpose |
|---|---|
| `research/envs/` | `OmniQuadEnv`/`g1_stand_env` Gymnasium wrappers (one OmniSim subprocess per env, TCP IPC) + `validate_env.py` (random-action smoke) |
| `research/controllers/` | RL controllers — `*_agent` (training, speaks the env protocol) and `*_deploy` (loads ONNX/`.pt` policy) |
| `research/worlds/` | RL training templates (per-env copies are `.*_envN.wbt`) and `*_deploy.wbt` |
| `research/training/` | trainers (`train_omniquad.py`, `gpu_mjwarp_*`, …) + per-run `runs/<name>/` outputs |
| `research/inference/` | `export_onnx.py` + `policies/<run>/policy.onnx` |
| `research/backends/` | physics specs (`g1_physics.json`/`_spec.py`) shared by trainer + deploy |
| `research/shadowing/` | ghost-tracking (reference-gen + verifier + RL tracker) experiments |
| `research/tools/` | `smoke_deploy.py` + eval/probe scripts |
