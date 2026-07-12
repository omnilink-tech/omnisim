# Sim-to-deploy RL recipe for OmniSim

> 🏗️ **This is Stage C/D of [Shadowing](shadowing.md)**
> (robust RL tracking + deploy). The pipeline's Stage B — a *planner* that produces a
> *dynamically-feasible* reference ("ghost") via trajectory optimization — is what makes
> the imitation target achievable in the first place; read that doc for the full motion
> architecture. This page is the gap-crossing (deploy) half.

The general recipe to take ANY robot from "GPU-trained policy works in MuJoCo" to "deployed policy stands/walks/works in OmniSim Newton at runtime." Distilled from the [G1 standing playbook](g1-stand-rl-playbook.md) into a checklist + template you can copy.

> **The single most important idea on this page:** if your policy works in mujoco_warp but doesn't transfer to OmniSim Newton, **stop trying to match physics bit-by-bit, and instead train the policy to be invariant to the gap.** Heavy domain randomization is the fix, not yet another solver-config tweak.

> **⭐ THE GHOST MUST BE ACHIEVABLE (foundational rule for any motion-imitation policy).** When the policy imitates a reference motion — a "ghost"/shadow — that reference **must be physically reproducible by the actual robot**. Never train the robot to mimic a ghost it cannot achieve. A reference is infeasible when it ignores the robot's **morphology** (missing DOFs — e.g. the 23-DOF G1 has only a waist-*yaw* joint, no torso pitch, so it physically *cannot* sit bolt-upright), its **actuation** (not enough torque/authority to e.g. cancel a limb-motion reaction), or its **balance/contact** limits. Symptoms of an unachievable ghost: the imitation reward saturates well below max, the deploy looks "wrong"/diverges, and **more training/scale does not help** (it's a physics ceiling, not a training gap — verified on the seated G1: a bolt-upright ghost stayed ~62–68% even after a 164 M-step A100 run with arm feedforward). The fix is **not** a better policy — it's a **better ghost**: derive the reference from what the robot can actually do (record-and-replay its achieved motion, or retarget/distill the reference onto the robot's kinematics + balance limits — the "achievable/improved shadow" approach, see [`g1-improved-shadow.md`](g1-improved-shadow.md)). Then measure similarity against *that* achievable ghost. Seated-G1 proof: achievable ghost → 100% match with genuine active balance. **If a well-trained policy won't match the ghost, suspect the ghost is unachievable before blaming the policy.**

> **⚠️ Scope caveat (2026-05-29):** heavy DR works for quadrupeds and for *in-sim* training, but it does **not** fully close the deploy gap for stability-margin **bipeds**. G1 standing trains to ≈98 % in-sim yet the OmniSim Newton deploy holds only to t ≈ 1.55 s — and a heavier-DR retrain deployed *worse* (FALL@1.47 s + a contact-solve coordinate explosion). The residual is the structural `mjw.step ≠ SolverMuJoCo.step` divergence (plus inherent biped instability), which DR cannot bridge; the answer there is to **train inside the deploy solver** (see [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md) §"Train-in-deploy-solver" and the canonical status in [`rl-current-state.md`](rl-current-state.md)). Treat this recipe as the *starting* point, not a guarantee of deploy success.

### Case study (2026-06-24): H1 walk — "train inside the deploy solver" *regressed* the deploy

The scope caveat above says "train inside the deploy solver" when heavy DR can't close a biped gap. The H1 walk run is the cautionary footnote: doing exactly that — training a policy through the **literal** deploy solver (`newton.solvers.SolverMuJoCo`, via [`gpu_newton_h1_walk_trainer.py`](../../projects/policies/research/training/gpu_newton_h1_walk_trainer.py), commit `cf200cdc`) — did **not** close the gap. It **regressed the deploy every single time**:

| Policy | Deploy survival | Deploy displacement |
|--|--|--|
| **run 3** (mjwarp-trained, *no* Newton fine-tune) — **the champion** | **2.03 s** | **+1.45 m FORWARD** |
| Newton fine-tune on a fresh-URDF-built model | 1.58 s | −0.96 m BACKWARD |
| Newton fine-tune on the matched dumped-MJCF model | 0.66 s | −0.34 m BACKWARD |

The lesson is *not* "deploy-solver fine-tuning is useless" — it's that **same solver ≠ matched physics, and matched model + matched solver still ≠ deploy.** Two distinct gotchas fell out of this run (both folded into the gotchas section below):

- **`same solver != matched physics`.** A fresh `newton add_urdf` + `add_ground_plane()` build uses newton's **default ground friction**, not the deploy's `mu=2.0`. You can be in the right solver and still be training on the wrong floor. **Always train on the DUMPED deploy model** (`OMNISIM_NEWTON_SAVE_MJCF` → load via `newton.ModelBuilder.add_mjcf`), never a fresh URDF rebuild. Fixed for H1 in commit `da8b171a` (`H1_TRAINER_MODEL=mjcf`).
- **Even matched model + matched solver is not enough.** run 3 and the Newton fine-tune are **byte-identical in the batched trainer** (both 2.05 s / −0.72 m at the launch phase) yet diverge hugely in deploy (2.03 s FWD vs 0.66 s BACK) — and the batched trainer walks **backward** where the deploy walks **forward**. The binding gap is **not** the physics step. It is (1) the **launch initial condition** — the deploy's ~0.3 s settle imparts a forward launch lean + residual velocity that the batched reset (pitch 0, qvel 0) lacks — and (2) the **observation pipeline** — deploy builds obs from world-frame `getVelocity()` + finite-diff `qd`, while the trainer reads exact MuJoCo-frame `qvel`. Deploy-physics fine-tuning only *starts* to help **after** those two are aligned.

**Recipe guidance (do this before deploy-solver fine-tuning a new biped):** (1) train on the **dumped deploy MJCF**, not a fresh URDF build; (2) **replicate the deploy launch IC** in the trainer reset — the settle lean, the residual velocity, and the right gait phase; (3) **match the obs construction** (world-frame velocity / finite-diff `qd`) between trainer and deploy. Only once all three are aligned does training inside the deploy solver close the gap rather than widen it. Full journal: [`h1-walk-rl-journey.md`](h1-walk-rl-journey.md); canonical status in [`rl-current-state.md`](rl-current-state.md).

## When to use this recipe

You have a policy that hits ≥ 95 % reward in your training env (`mujoco_warp` directly, or your own trainer), but in OmniSim Newton deploy it falls/fails/drifts within seconds. The MuJoCo eval looks great, the deploy looks broken.

If you're building a quadruped, the [Spot residual recipe](spot-residual-rl.md) is probably what you want — the gap closes naturally because four-foot static stability is robust to wrapper drift. This doc is for **bipeds, manipulators, and other stability-margin systems** where the gap actually bites.

## The recipe in eight steps

### 1. Confirm the baseline (analytic-only policy) behaves the same way in train and deploy

Before any RL, run with a hand-coded baseline (e.g. NOMINAL pose + ankle PD; or gait + IK; whatever your "free" baseline is) and measure how long it survives in each env. If MuJoCo says 2 s and OmniSim says 0.2 s, **stop and find out why before training**. They should agree within ±20 % once you've enabled the env vars in step 3.

If you skip this step you'll burn days training over a physics gap the baseline already exposes.

### 2. Dump the deploy MJCF

OmniSim's Newton backend has an env-var that prints the actual MJCF Newton built for the robot. Use it.

```bash
OMNISIM_NEWTON_SAVE_MJCF=$PWD/_scratch/<robot>_deploy.mjcf.xml \
OMNISIM_NEWTON_FORCE_MUJOCO=1 \
OMNISIM_NEWTON_MJWARP=1 \
OMNISIM_URDF_USE_INERTIA=1 \
./msys64/mingw64/bin/omnisim-bin.exe <your-deploy-world>.wbt &
# wait for "world finalised" in the log, then close OmniSim.
```

Cat the resulting file. Look at:

- **Actuator gains** (`<general .../> gainprm="..." biasprm="..."`). Compare to what you put in your training MJCF. **This is the #1 silent breaker.** Newton's default kp/kv are NOT the kp/kv you wrote in the URDF/MJCF.
- **Joint axes** (`<joint axis="..."/>`). Newton may rotate them into a "principal" frame; your training MJCF might have them in URDF frame. Differing axes = same action becomes different motion.
- **Integrator** (`<option integrator="..."/>`). Newton typically uses `implicitfast`; if your training MJCF uses `Euler` you have a real integrator gap.
- **Inertia frames** (`<inertial pos="..." quat="..."/>`). Newton diagonalizes inertia tensors; your MJCF probably leaves them as full 3×3.

### 3. Train on the deploy MJCF

Rename Newton's anonymous `body_N` / `joint_N` to your robot's actual link/joint names by walking the kinematic chain (the G1 import script [`projects/policies/research/training/import_newton_mjcf.py`](../../projects/policies/research/training/import_newton_mjcf.py) does this — copy the pattern). Now you have a training MJCF that is *bit-identical* to the model Newton uses at deploy. This collapses three of the most common sim-to-deploy gaps (axes, inertias, gains) at once.

### 4. Set the matching env vars at deploy time

The OmniSim Newton backend selects its solver at runtime based on env vars. **Set these every time you run your deploy world:**

| Env var                            | Why                                                                                                |
|--                                  |--                                                                                                  |
| `OMNISIM_URDF_USE_INERTIA=1`       | Without this OmniSim's URDF importer discards `<inertial>` tags entirely. Robot won't balance.    |
| `OMNISIM_NEWTON_FORCE_MUJOCO=1`    | Pick `newton.solvers.SolverMuJoCo` instead of the XPBD default — the solver mujoco_warp matches. |
| `OMNISIM_NEWTON_MJWARP=1`          | Inside SolverMuJoCo, use GPU mujoco_warp (not CPU `mj_step`). Same engine your trainer runs.       |

Without these, you're deploying against a different solver than you trained on, and no amount of policy quality will close that.

### 5. Add heavy domain randomization to the trainer

This is the load-bearing step. Whatever your MJCF and env vars match, the deploy wrapper still adds its own bookkeeping: a control-buffer indirection (1-tick delay), a state-sync (qvel readback with extra clamping), per-joint limit projection, etc. Closing each of these in code is endless. **Train the policy to not care.**

Knobs that empirically matter (from the G1 playbook):

| Knob                       | Recommended starting band |
|--                          |--                         |
| Body mass (per-body)       | ± 30 %                    |
| Friction                   | ± 50 %                    |
| Joint damping (per-DOF)    | ± 50 %                    |
| **Actuator kp (per-act)**  | ± 40 % ← critical         |
| **Actuator kv (per-act)**  | ± 40 % ← critical         |
| Gravity                    | ± 5 %                     |
| External pushes (prob)     | 2 % per step              |
| External pushes (impulse)  | up to 1.5 m/s             |
| Observation noise          | 3 % gaussian              |
| **Action latency**         | 0 - 3 random ticks per env ← critical |
| Initial joint q            | ± 0.15 rad                |
| Initial base xy / z        | ± 0.05 m / ± 0.02 m       |

The three marked "critical" rows are the ones that close the deploy-wrapper gap specifically. The others give general robustness.

Implementation pattern (from [`gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py)):

- Per-run model jitter (mass / friction / damping / gains / gravity) applied to `self.mjm` *before* `mjw.put_model()`.
- Per-step action latency: a `(N, max_latency + 1, NJ)` torch buffer; roll forward each step, sample the effective action from a per-env-fixed delay slot.
- Per-step pushes: sample `hit ~ Bernoulli(push_prob)` per env, on hit apply a random horizontal velocity impulse to `qvel[0:3]`.
- Per-reset randomized initial conditions: spawn perturbations on joint q + base xy/z.
- Per-step observation noise: `obs + randn * obs_noise`, then clamp.

### 6. Make training fast (so heavy DR is tractable)

Heavy DR + small batch = useless. Either you wait hours per run or you DON'T heavy-randomize. The G1 playbook stacked 5 speedups to make 30 M-step PPO runs take 3 min 43 s:

1. **Actor on GPU** — `ac.to(cuda)`, rollout buffers cuda-resident. Requires CUDA-enabled PyTorch (`pip install torch --index-url https://download.pytorch.org/whl/cu128`).
2. **GPU-native env operations** — `wp.to_torch(mw_d.qpos)` gives a zero-copy torch view. Baseline + obs + reward run in torch on cuda. No CPU↔GPU traffic in the hot loop.
3. **CUDA graph capture** — wrap the SUBSTEPS physics loop in `wp.capture_begin/end`; replay each env-step via `wp.capture_launch`. Cuts ~200 kernel launches per substep to one graph replay.
4. **Bigger N** — 4096 envs, 12 rollout (vs the typical 2048/24). GPU saturates at larger batch; total samples per update is the same.
5. **SUBSTEPS = 4 × 4 ms** — same env-step semantics as 8 × 2 ms but half the physics work. mujoco_warp stays stable for human-scale dynamics at 4 ms.

Combined ≈ 9 × throughput on RTX 5070-class hardware. See [`gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py) for a working implementation you can copy.

### 7. Eval before deploy

Run a long-horizon deterministic eval inside the trainer (`--eval --eval-steps 2500`). With heavy DR active, some falls are *expected* — the policy is being randomly pushed mid-episode. **Target ≥ 90 % survival.** If you're below that, the policy isn't robust enough yet; widen the DR or train longer.

If you're at ≥ 95 % in MuJoCo eval *with* DR, you're ready to deploy.

### 8. Deploy in GUI mode

Run the OmniSim deploy world with the GUI, not `--no-window` headless. There's a known event-loop interaction in headless mode that hangs Newton finalize for tall articulations (`docs/developer/g1-stand-rl-playbook.md` covers this in the G1 case). GUI mode dodges it deterministically. This costs you nothing for a deploy demo — just `omnisim-bin.exe <world.wbt>` with no flags.

## Common gotchas

- **CPU PyTorch.** `pip install torch` ships CPU-only by default. Speedup #1 silently breaks. Always verify `torch.cuda.is_available() == True` before training.
- **`mujoco_warp` ≠ `Newton.SolverMuJoCo`.** Even if both use the same `mjw.step()` internally, `Newton.SolverMuJoCo` wraps it with extra logic. Always train against the wrapped version OR use heavy DR to absorb the difference. See [`newton_solver_trainer.py`](../../projects/policies/research/training/newton_solver_trainer.py) docstring for the source quote.
- **Joint limits inside DR.** If your DR pushes joint targets near their `<limit>` ranges, mujoco_warp may apply soft-limit forces that differ from Newton's hard clamps. Either widen limits in the training MJCF, or clamp your residual action range below the joint limits explicitly.
- **Observation noise on rates.** Velocity terms (lin_vel, ang_vel, joint qd) are MUCH noisier in deploy than in training, because deploy's `getVelocity()` reads through the Newton state buffer with a 1-tick offset. Increase `obs_noise` for the velocity channels specifically if you have selective control.
- **Action latency must be sampled per-env, fixed for the episode.** If you re-sample latency every step, the policy can't learn a coherent strategy. Sample once at reset; keep it constant through the episode.
- **`OMNISIM_NEWTON_SAVE_MJCF` overwrites every run.** Make sure your output path is one you don't mind being clobbered.
- **Same solver ≠ matched physics.** Training inside the deploy solver (`newton.solvers.SolverMuJoCo`) is necessary but not sufficient. A fresh `newton add_urdf` + `add_ground_plane()` build silently uses newton's **default ground friction**, not the deploy's `mu=2.0` — same engine, wrong floor. **Always train on the DUMPED deploy model** (`OMNISIM_NEWTON_SAVE_MJCF` → load via `newton.ModelBuilder.add_mjcf`), never a fresh URDF rebuild. Verified on H1 (2026-06-24, fix `da8b171a` `H1_TRAINER_MODEL=mjcf`); see the case study above and [`h1-walk-rl-journey.md`](h1-walk-rl-journey.md).
- **Matched model + matched solver is *still* not enough — the launch IC and the obs pipeline bind.** Even with a byte-identical model and the literal deploy solver, the batched trainer and the deploy can diverge completely (H1: same 2.05 s / −0.72 m launch phase in the trainer, but 2.03 s FWD vs 0.66 s BACK in deploy — the trainer even walks *backward* where the deploy walks *forward*). Two residual gaps cause this: (1) the **launch initial condition** — the deploy's ~0.3 s settle imparts a forward lean + residual velocity that a `pitch 0, qvel 0` reset lacks, so replicate the settle lean / residual velocity / gait phase in the trainer reset; and (2) the **observation pipeline** — match deploy's world-frame `getVelocity()` + finite-diff `qd` against the trainer's exact MuJoCo-frame `qvel`. Align both **before** expecting deploy-solver fine-tuning to help.

## Reusing this for a new robot

The shortest path from "I have a URDF" to "the robot works in OmniSim deploy":

1. **Bring up the deploy world first.** Get OmniSim to load your URDF under Newton (`physicsBackend "newton"`, `OMNISIM_URDF_USE_INERTIA=1`). Verify it doesn't immediately tip in GUI mode — even if it falls in 1 s, that's enough; we just need the model to register.
2. **Dump the deploy MJCF** via `OMNISIM_NEWTON_SAVE_MJCF`. Save this in `projects/robots/<robot>/urdf/<robot>.mjcf.xml`.
3. **Write a name-mapping script** like [`import_newton_mjcf.py`](../../projects/policies/research/training/import_newton_mjcf.py) — find each `joint_N` in the dump, identify which physical joint it maps to (by parent body + axis + limit range), and rename. Same for bodies.
4. **Copy [`gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py)** as `gpu_mjwarp_<robot>_<task>_trainer.py`. Change: `LEGS_JOINTS` / `NJ` / `NOMINAL` / `OBS_DIM`, the baseline PD logic for your robot, the reward shaping for your task. Keep the DR config + the 5 speedups unchanged.
5. **Copy [`g1_stand_deploy.py`](../../projects/policies/research/controllers/g1_stand_deploy/g1_stand_deploy.py)** as the deploy controller. Same baseline math as the trainer; loads the ONNX. Make sure pelvis velocity uses `Supervisor.getVelocity()` (not finite-difference position deltas); joint qd uses low-pass-smoothed finite-diff of the position sensors — see the template for the exact obs construction.
6. **Train and iterate.** First training run, no DR — confirm the baseline + policy works on the training MJCF. Then enable heavy DR. Then deploy.

## Files to copy as templates

| Role                  | G1 file                                                                                                                                        | What to change                                                       |
|--                     |--                                                                                                                                              |--                                                                    |
| MJCF rename script    | [`projects/policies/research/training/import_newton_mjcf.py`](../../projects/policies/research/training/import_newton_mjcf.py)                                               | `JOINT_RENAME` / `BODY_RENAME` dicts                                  |
| Trainer               | [`projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_g1_stand_trainer.py)                              | `LEGS_JOINTS` / `NOMINAL` / baseline PD / reward / DR knobs           |
| Deploy controller     | [`projects/policies/research/controllers/g1_stand_deploy/g1_stand_deploy.py`](../../projects/policies/research/controllers/g1_stand_deploy/g1_stand_deploy.py)                | Same baseline math, same obs layout                                   |
| Deploy world          | [`projects/policies/research/worlds/g1_stand_deploy.wbt`](../../projects/policies/research/worlds/g1_stand_deploy.wbt)                                                        | `URDFRobot url=...`, controller name, spawn translation               |

## When to deviate from this recipe

- **Quadrupeds**: skip heavy DR. The Spot residual recipe ([`spot-residual-rl.md`](spot-residual-rl.md)) is faster and works because four-foot static stability soaks up the deploy wrapper drift naturally.
- **Manipulators with fixed base**: most of the DR knobs don't apply (no falls). Focus on actuator kp/kv jitter + action latency + observation noise. Skip pushes and initial-pose jitter on the base.
- **Tasks with rich extrinsic state** (e.g. picking objects): add randomization on the *task* objects (size, mass, friction, initial pose) in addition to the robot's own DR.
- **Heavier / higher-DOF humanoids (Atlas-class)**: the DR profile that worked on G1 is too aggressive — see [`atlas-stand-rl-journey.md`](atlas-stand-rl-journey.md). Per-body mass jitter has a lower ceiling (0.20, not 0.30); residual scale halves (0.15 rad); the heavy-DR profile that produced robust learning on G1 floors Atlas's reward and never moves. Start with no DR, train to standing, then curriculum DR in.

## References

- [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md) — the case study this recipe was distilled from.
- [`atlas-stand-rl-journey.md`](atlas-stand-rl-journey.md) — porting this recipe to a heavier, higher-DOF humanoid. Six concrete things that needed to change.
- [`spot-residual-rl.md`](spot-residual-rl.md) — the quadruped path.
- [`humanoid-balance-gap.md`](humanoid-balance-gap.md) — why bipeds don't get the easy ride quadrupeds do.
- [`rl-accelerated-training.md`](rl-accelerated-training.md) — GPU mujoco_warp infrastructure overview.
- [OpenAI Dactyl paper](https://arxiv.org/abs/1808.00177) — the canonical "heavy DR makes sim2real work" result. Same idea, different robot.
