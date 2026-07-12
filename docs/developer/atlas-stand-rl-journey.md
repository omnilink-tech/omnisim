# Atlas standing — porting the G1 recipe to a 30-DOF biped

Companion to [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md) and [`sim-to-deploy-rl-recipe.md`](sim-to-deploy-rl-recipe.md). What changes when you move the same recipe from a 13-DOF G1-legs trainer to the 30-DOF Boston Dynamics Atlas.

## Status

**The infrastructure and analytic baseline transfer; PPO cannot extract a residual policy that meaningfully beats the baseline at this scale.** The deliverable is a working stand at baseline behavior, exported through the standard trainer/ONNX pipeline. Per-episode survival:

| Condition | Median steps-to-first-fall | Mean | Max |
|--|--|--|--|
| No DR (= OmniSim Newton deploy condition) | 41 | 44.4 | 110 |
| Heavy DR, no mass jitter (training distribution) | 31 | 33.6 | 75 |
| Full G1-class DR (mass=0.20, seed 0) | 34 | 35.9 | 52 |
| Full G1-class DR (mass=0.20, seed 3) | 32 | 34.5 | 52 |

For comparison, the analytic baseline alone gives medians 33–40 across the same conditions. The trained policy doesn't materially improve per-episode survival; it does cycle clean baseline-length episodes consistently (cumulative survival ≈ 97 % under heavy DR over a 2500-step rollout).

| Artifact | Path |
|--|--|
| GPU mujoco_warp trainer (30 DOF, curriculum support via `--init-from`) | [`projects/policies/research/training/gpu_mjwarp_atlas_stand_trainer.py`](../../projects/policies/research/training/gpu_mjwarp_atlas_stand_trainer.py) |
| Newton-MJCF → readable-MJCF importer (arm-collision strip + njmax bump) | [`projects/policies/research/training/import_newton_mjcf_atlas.py`](../../projects/policies/research/training/import_newton_mjcf_atlas.py) |
| Deploy controller (mirrors trainer baseline + loads policy.onnx) | [`projects/policies/research/controllers/atlas_stand_deploy/atlas_stand_deploy.py`](../../projects/policies/research/controllers/atlas_stand_deploy/atlas_stand_deploy.py) |
| Deploy world (z=0.95 spawn for new NOMINAL) | [`projects/policies/research/worlds/atlas_stand_deploy.wbt`](../../projects/policies/research/worlds/atlas_stand_deploy.wbt) |
| Final policy (best by EMA, then `last.pt` re-promoted) | `projects/policies/research/training/runs/gpu_atlas_stand_robust/policy.{pt,onnx}` |
| Intermediate stages (not retained — only the final `policy.{pt,onnx}` is committed) | `stage1.pt` (no DR), `stage1m.pt` (mass-only), `stage1nm.pt` (all DR except mass) |

## What the env required to be trainable at all

Three env-side fixes are mandatory; without them the trainer NaNs out or stays at baseline-only zero. Apply in this order:

### 1. Strip arm-chain collisions in the Newton MJCF

The Newton-dumped MJCF roundtrips Atlas's arm convex hulls as collidable geoms, but the source URDF defines hand/forearm physics as NULL (OmniSim load log: *"As 'physics' is set to NULL, collisions will have no effect"*). The hulls vastly overstate arm volumes — at any standing pose, `l_scap` intersects `utorso` by 26 cm, `l_uarm` by 23 cm, etc. 53 deep self-contacts at the seed pose, max penetration −0.26 m. The first physics step generates ~1400 N·m of constraint force propagating through the arm chain into the 84 kg utorso and slamming the pelvis with vang ≈ 14 rad/s — the body explodes on step 1 of every reset.

Fix in [`import_newton_mjcf_atlas.py`](../../projects/policies/research/training/import_newton_mjcf_atlas.py): walk the body tree and set `contype="0" conaffinity="0"` on all geoms whose body is in `{l_clav, l_scap, l_uarm, l_larm, l_ufarm, l_lfarm, l_hand, head, …r-side mirror…}`. After that, the only contacts left are the two feet on the ground.

### 2. Pass njmax / nconmax explicitly to `mjw.put_data(...)`

mujoco_warp does **not** read the `<size njmax="...">` tag from the MJCF. Its default njmax is computed from `nv` and is 53 for Atlas; transient contact pileups under DR regularly need 60-80 constraint rows and the solver prints `nefc overflow - please increase njmax to NN` and silently truncates. The truncated constraint vector corrupts physics → throughput halves and PPO gradients chase noise.

Fix in the trainer: pass `njmax=256, nconmax=128` to `mjw.put_data(...)`. The `<size>` tag in the MJCF is left as documentation.

### 3. Clamp realized init-jitter qpos to the joint-limit interior

Atlas's NOMINAL places `l_arm_elx` at +0.10 rad and the joint's lower limit is exactly 0.0; symmetrically for `r_arm_elx`. With `init_q_band = 0.15` (G1's value), each elbow has P = 0.167 of starting out of range; per-env P(any joint out of range) = 0.306. The reset code writes absolute jittered qpos directly into `mw_d.qpos` without clamping; mujoco_warp then fires a large Baumgarte limit-restoring impulse on step 1 that propagates through the arm into the 84 kg utorso and corrupts every gradient PPO sees for ~30 % of envs.

G1 doesn't see this — its NOMINAL has > 0.26 rad clearance on every joint, so the same DR band never lands a joint out of bounds.

Fix in `_reset_envs`: realize the qpos as `clamp(NOMINAL + jitter, jl_lo + 0.02, jl_hi - 0.02)` before writing. The DR knob value (`--dr-init-q-band 0.15`) is unchanged at the spec level; only the realized sample is truncated. Same approach every modern legged-RL trainer uses.

## The new analytic baseline

G1's analytic baseline is **NOMINAL + ankle-only PD with KP=-1.5, clamp ±0.2 rad**. On Atlas (175 kg vs 35 kg, 30 DOFs vs 13), the same gains generate only ~4 N·m of ankle torque per radian of pitch — about 20× too weak. The body tips in 3-5 steps.

The new Atlas baseline distributes balance torque across the leg chain via coupled ankle + hip + back PD. Clamps are wide because the MJCF actuator has kp=20 N·m/rad — the baseline's job is to *demand* enough error that the joint's `actuatorfrcrange` (±360 N·m on ankles) does the real torque saturation.

| Joint pair | KP | KD | Clamp |
|--|--|--|--|
| Ankle pitch/roll | −20 | −3.0 | ±1.0 rad |
| Hip pitch | −12 | −2.0 | ±1.0 rad |
| Hip roll | −8 | −1.2 | ±1.0 rad |
| Back pitch | −8 | −1.2 | ±0.75 rad |
| Back roll | −5 | −0.8 | ±0.75 rad |

Plus a cleaner upright NOMINAL: arms hanging (shx=±0.30, ely=+0.30) instead of G1's tucked squat, mild leg bend (hpy=−0.10, kny=+0.20, aky=−0.10) instead of G1's deep squat. The original NOMINAL caused the self-collision pileup of §1 and put CoM well forward of the foot center.

Baseline survival under heavy DR after these changes: median 33 (matched DR no mass), 40 (no DR). PASS the gate (median ≥ 30) under the heavy DR profile G1's baseline meets under G1's DR.

## The DR curriculum, three attempts

### Stage 1: NO DR — feasibility gate

Trained from scratch with all DR knobs at zero. Reward + V both climbed monotonically (V peaked at 17 by iter 150) — first time across multiple attempts that PPO produced sustained improvement. Per-episode survival evaluated against the same no-DR conditions: median 41 steps. Cumulative survival in 2500-step rollouts: 2444. Looked promising.

### Stage 2 attempted: warm-start from Stage 1 + 50 % DR — **failed**

Loaded stage 1's weights, ramped DR to half of G1's profile, restarted training. Result: **every env fell at step 1** (per-step reward = 0.498 = alive + upright − term). PPO got no learning signal because there was no successful trajectory anywhere in the batch. Same outcome at 25 % DR.

Bisecting the DR knobs against the stage-1 policy revealed the cause: **`dr_mass_scale=0.05` alone killed every env at step 1**. Other knobs (friction, damping, kp/kv, init_q jitter, push, latency) were tolerated. Switching the seed away from 0 reproduced the symptom: out of 8 random mass-DR seeds, only 2 left the stage-1 policy able to stand.

### Why mass DR is a single-draw lottery in this trainer

`BatchedAtlasStandEnv.__init__` applies mass perturbation **once per training run** to a single `mjm` model that all 4096 envs then share — there is no per-env mass DR in mujoco_warp (the `mw_m.body_mass` array has shape `(1, 34)`, not `(N, 34)`). The training run sees one perturbed mass distribution; the policy generalizes to that one specific mass-config and nothing else.

Worse, **the baseline itself isn't stable across mass seeds.** Tested across 8 seeds with `dr_mass_scale=0.20`:

| Seed | Baseline median survival |
|--|--|
| 0 | 41 |
| 1 | 1 |
| 2 | 1 |
| 3 | 41 |
| 4–7 | 1 |

Only 2 of 8 mass seeds leave the analytic baseline able to stand. The other 6 produce a mass distribution where Atlas's NOMINAL pose puts CoM outside the foot's stability margin and the body tips at step 1 with zero action. We were training under `seed=0` (default) which happened to be one of the "safe" seeds.

This recasts the heavy-DR result: G1's training with `dr_mass_scale=0.20` likely succeeded because G1's lighter body and lower CoM gives a larger stability margin under per-body mass perturbation (a 4× higher safe-seed rate would suffice). On Atlas, mass DR in single-draw form is a coin flip between "barely stable" and "instant fall."

### Stage 1nm: heavy DR except mass — **the actual deliverable**

Trained from scratch under the full G1 DR profile with `dr_mass_scale=0.0`. Reward and V both climbed cleanly to V=23 by iter 165 (peaked then oscillated around 17). Same shape Stage 1 (no-DR) showed, but now with per-step DR (push, latency, obs noise, friction/damping jitter) baked in.

This is what `policy.pt` / `policy.onnx` hold.

## What the trained policy doesn't fix

Per-episode survival is still baseline-level (median 31-41 depending on conditions, max 75-110). The policy mu stays near zero through all 200 iters — the value function learned to track baseline returns, but the policy itself doesn't move materially off NOMINAL+balance-PD. A residual scale bump from 0.05 → 0.10 (tested through iter 110) produced an identical V trajectory: the policy can't profitably move mu in either case.

The mechanism, cross-verified by an earlier hypothesis workflow: the baseline already harvests ~88 % of the per-step reward ceiling at iter 1, so the reward surface is locally flat around mu=0, the act² penalty + heavy DR action sampling both push mu back to zero, and PPO learns to leave it there. G1 escapes this regime because (a) 13 DOFs vs 30 means about 2× less per-update gradient noise, (b) G1's baseline is further from saturation so the residual has genuine upside, (c) G1's v_loss weight is 2× larger so V tracks the moving distribution faster.

## What carries over unchanged

Independently sound; kept across every attempt:

- **NaN sanitization** on qpos/qvel before reward — Atlas at 175 kg under heavy DR can still produce NaN qpos in deep penetration; treat as fall and reset same tick.
- **`reward = clamp(reward, −3, 3)`** and **`v_target = clamp(ret, −50, 50)`** — bounds the value-loss gradient on iter 1.
- **`v_loss` weight 0.25**.
- **`dynamo=False`** on `torch.onnx.export` — newer torch defaults to `dynamo=True` which hangs on the 30-DOF actor.
- **Zero-init the π and V final linear layers** — guarantees iter-1 residual = 0 and iter-1 V = 0, so the AC starts at exactly the analytic baseline and only moves off when advantage signal is real.
- **log_std init −2, entropy coef 0** — keeps action noise small so the strong baseline isn't perpetually destabilized by exploration.

## Where this leaves the recipe doc

[`sim-to-deploy-rl-recipe.md`](sim-to-deploy-rl-recipe.md) sells "heavy DR closes the deploy gap." For Atlas-class humanoids with a near-saturating analytic baseline, that claim has two caveats this port exposed:

1. **Mass DR in mujoco_warp is per-run, not per-env.** Any "robustness" the policy gets from mass DR is generalization to *that specific perturbed model*. For sim-to-sim deploy where the target uses URDF inertia directly (OmniSim Newton with `OMNISIM_URDF_USE_INERTIA=1`), mass DR is at best neutral and at worst trains on systematically wrong physics. For real-hardware sim-to-real where actual mass varies, per-env mass DR would be needed (would require a mujoco_warp patch).
2. **Heavy DR with a near-saturating baseline starves PPO of signal.** The act² anchor + noisy 30-DOF gradients keep μ at zero; the saved policy converges to the analytic baseline. The recipe works on G1 because G1's baseline has real headroom. On Atlas the headroom is below the noise floor.

Practical implications, all encoded in this trainer:

1. **Strip URDF-NULL collisions in the MJCF** before any RL.
2. **Pass njmax explicitly to `mjw.put_data`** — the MJCF `<size>` tag is silent.
3. **Clamp realized init-jitter qpos to the joint-limit interior** at reset.
4. **Size the baseline gains to the actuator kp**, not to a small-delta intuition.
5. **Train without mass DR for sim-to-sim deploy.** Mass DR's lottery property makes it harmful when target physics is identity. Use friction / damping / kp/kv / push / latency / init-q jitter for real per-step variance.
6. **Accept that strong baseline + heavy DR + 30 DOFs has a PPO ceiling at the baseline.** The deliverable is the analytic stack with PPO infrastructure around it; deploy ONNX matches that.

## References

- [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md) — the case study this recipe was distilled from.
- [`spot-residual-rl.md`](spot-residual-rl.md) — the quadruped path.
- [`humanoid-balance-gap.md`](humanoid-balance-gap.md) — why bipeds don't get the easy ride quadrupeds do.
- [`rl-accelerated-training.md`](rl-accelerated-training.md) — GPU mujoco_warp infrastructure overview.
