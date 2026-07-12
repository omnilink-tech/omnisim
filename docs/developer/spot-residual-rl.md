# Spot residual RL: the model walker + tiny policy

Why we replaced 200,000 PPO steps of from-scratch RL with 20,000 PPO steps of residual RL on a model-based controller — and how the new stack is put together.

> **Building a NON-quadruped robot?** This recipe (residual on a model-based gait+IK
> baseline) **doesn't directly port to bipeds, manipulators, or other stability-margin
> systems** — the analytical baseline can't be made to "almost work" the way it does
> for a quadruped. Use [`sim-to-deploy-rl-recipe.md`](sim-to-deploy-rl-recipe.md)
> (heavy-DR pure PPO on GPU mujoco_warp) instead. The G1 standing case study in
> [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md) shows the journey from "tried
> Spot's recipe, fell over" to a robust G1 stand (≈98 % in the trainer; the OmniSim
> Newton **deploy stand is SOLVED** — it holds indefinitely via a deterministic pure pose,
> not RL, fixed 2026-06-10. The old "holds to t ≈ 1.55 s then falls" was the pre-fix RL
> state. See [`rl-current-state.md`](rl-current-state.md), canonical).

## Bottom line

|                            | spot_walk_v12_200k (old) | spot_residual_main (new) |
| --                         | --                       | --                       |
| Training steps             | 200,000                  | **20,000** (10× fewer)   |
| Wall-clock to first policy | tens of minutes          | **52 seconds**           |
| Forward distance in 30s    | 3.19 m                   | **5.55 m**               |
| Lateral drift              | -1.02 m                  | **-0.30 m**              |
| Yaw drift                  | -45.8°                   | **-9.4°**                |
| Verdict                    | DRIFTED                  | **STRAIGHT** ✓           |

The new policy is both faster to train and better at the task. Both numbers measured by `projects/policies/research/tools/verify_straight_walk.py` over a 30-second straight-walk run at vx = 0.5 m/s.

## The problem we kept hitting

From-scratch PPO had to learn everything at once: which joints to actuate, in what phase, with what amplitude, to produce forward motion without falling. The reward landscape for "walk straight without falling" has a sticky local optimum at **stand perfectly still** — zero forward distance, but also zero drift and zero risk of falling. Almost every reward-shape combination we tried converged there:

- v2 (lateral & heading penalties at -0.5 / -0.3): stand still, perfect drift, zero forward.
- v3 (sharper velocity tracking + bigger vx bonus): stand-with-slight-spin for 200k steps, then catastrophic divergence at 204k (NaN explosion as the policy chased an unbounded vx bonus past its training distribution).
- v12_200k (the eventual "best" — 12 reward-tuning iterations to get there): walks, but slowly (0.1 m/s) and drifts. Continue-training it for straightness collapses the walking entirely within 10k steps.

Heading-lock at deploy time (closing a P loop on yaw and feeding it as a wz command into the policy) didn't help either: v12 was trained on randomized wz ∈ [-0.3, +0.3] and learned the shortcut "non-zero wz = stop and pivot in place". Even a tiny corrective wz from the lock made it stop walking.

The deeper issue: a 49-dim policy with no heading observation literally cannot know it has drifted. It can only react to instantaneous angular velocity, not integrated heading error. So even a perfect reward function can't teach it the right behavior — the information isn't in the input.

## The new method: model-based controller + residual RL

Production quadrupeds (Boston Dynamics, ANYbotics, Unitree) don't train end-to-end from raw state. They have a hand-coded controller that does the structural work — gait pattern, foot trajectories, inverse kinematics, body stabilization — and learning sits on top to refine the parts that are hard to model.

We did exactly that. Four layers, only the last one is learned:

```
OmniSim tick → read body state →
  [1] gait engine          → 4 foot targets in body frame
  [2] balance PD           → per-leg z offset that levels the body
  [3] inverse kinematics   → 12 joint angles
  [4] residual policy      → per-leg foot offsets, ±3 cm, applied between [2] and [3]
  → 12 motor commands
```

### Layer 1: gait engine — `projects/policies/control/spot_gait.py`

A trot foot-trajectory generator. Diagonal pairs (FL+RR, then FR+RL) alternate stance and swing on a half-cycle offset. Per-leg, per-tick, given `(time, vx, vy, wz)`:

- **Stance phase**: foot stays planted in the world; in the body frame it slides backward at `-vx` so the body moves forward at `+vx` over its planted foot. Step length is `vx × stance_duration`, which guarantees no slip.
- **Swing phase**: foot lifts in a parabolic arc, peak `step_height` above ground, while moving forward to the next stance position.

The gait engine knows nothing about physics or contacts — it just emits the foot positions a walking robot would have. It's correct by construction for straight forward walking.

### Layer 2: inverse kinematics — `projects/policies/control/spot_kinematics.py`

Closed-form analytic IK for Spot's 3-DoF legs (hip_x, hip_y, knee). Given a target foot position in the body frame, solves for the three joint angles in three closed-form steps:

1. **hip_x** (lateral roll) from the constraint that the foot's y component in the hip frame must equal the hip_y joint offset.
2. **knee** (γ) from the cosine law on the upper-leg / shank / hip-to-foot triangle, accounting for the small forward offset of the knee joint relative to the upper leg.
3. **hip_y** (sagittal swing) from the in-plane rotation that places the shank endpoint at the target.

Dimensions extracted from `projects/robots/boston_dynamics/spot/urdf/spot.urdf`: L1 = 0.3205 (thigh), L2 = 0.32 (shank), knee x-offset = 0.025, hip_y offset = 0.110945, hip_x mount = ±0.29785, ±0.055. Round-trip `fk(ik(target)) == target` to 1e-16 across 200 random feasible targets per leg — the IK is exact, not numerical.

### Layer 3: balance PD — `projects/policies/control/spot_balance.py`

Reads the body's roll and pitch each tick and outputs a per-leg z offset added to the gait's foot target. Pitched nose-up → contract front legs and extend rear → body rotates nose-down. Rolled right → extend right legs and contract left → body rotates left. PD gains tuned conservatively; clipped to ±5 cm so the IK target stays in the leg's reachable workspace.

The balance PD alone improves the model walker measurably (+0.59 m forward, -0.05 m lateral, -1.9° yaw vs no balance) but its bigger value is during recovery from perturbations.

### Layer 4: residual policy — the only learned part

A tiny MLP (18 → 64 → 64 → 12, Tanh activations). The 18-dim observation is body angular velocity (3), projected gravity (3), gait phase (1), body linear velocity (3), velocity command (3), yaw drift from spawn (1), lateral drift (1), and the first 3 dims of the previous action. The 12-dim action is a foot-position offset per leg (Δx, Δy, Δz), interpreted as ±1 each and scaled to ±3 cm.

The policy is initialized with `log_std_init = -1.5` (std ≈ 0.22, meaning effective initial residual std ≈ 7 mm per axis). It starts as essentially the identity — the model walker's gait passes through almost unchanged on tick 0. PPO then nudges it: which feet to plant a centimeter forward of nominal, which to lift a few millimeters higher, etc. The residual is too small to overpower the gait or destabilize the body, but exactly the right scale to compensate for the asymmetric drift the model walker has on its own.

## Why this is much more sample-efficient

The from-scratch PPO had to discover walking, then refine balance, then correct drift — three layers of learning each gated by the previous. The reward landscape has many sticky local optima (stand still, stand and slightly spin, walk-but-fall-soon), and PPO routinely converges to them. Every reward-shape change shifts which optimum the policy falls into.

Residual RL on the model removes the discovery problem entirely:

- The gait + IK + balance combo **already walks**, by construction. PPO starts from a competent baseline, not from random noise.
- The action space is **bounded by physics**: a ±3 cm foot offset can't tip the robot, can't violate joint limits, can't make a leg unreachable. Random exploration is safe.
- The "stand still" local optimum is **unreachable**: the gait engine is always cycling foot positions, regardless of what the policy outputs. The policy can only modulate the cycle, not stop it.
- Reward shaping is **forgiving**: the model is doing the right thing on average, so a poorly-tuned reward weight gives a slightly-worse walker, not a complete behavioral collapse.

Training collapses from 200,000 steps + 12 reward-tuning iterations to **20,000 steps in one shot, in 52 seconds wall clock on CPU**.

## Stack-wise breakdown of credit

It's worth saying clearly: the model walker is what does the heavy lifting. With **zero neural network**, just gait + IK + balance, Spot walks 4.77 m forward / -0.62 m lateral / -15.6° yaw over 30 s. That alone beats v12_200k on every axis. The 20k-step residual policy adds another +0.78 m forward, halves the lateral drift, and pulls the yaw down inside the STRAIGHT threshold. The policy is a meaningful refinement, but the model layer is doing most of the work — and that's the point.

**Under Newton this becomes even sharper: the policy is essentially a passenger.** The Newton model walker alone walks +5.03m / +0.03m lat / +2.1° yaw over 30 s — already STRAIGHT. The 50k-step residual policy on top deploys to +4.87m / +0.13m lat / +3.2° yaw — marginally *worse*. The visual demo you can watch in `spot_residual_deploy_newton.wbt` is, functionally, the hand-coded gait engine + Newton physics. The trained ONNX is being called every tick, but its output is small noise around an already-near-perfect baseline.

This is not a failure of the recipe — it's a *feature*. It tells us when the analytical model has converged on the right answer for the task. Under Newton's stiffer joints, planted feet (μ=2), and URDF-derived inertia tensors, the gait engineer's assumptions hold cleanly, and the gait equation is "the answer." Under ODE+kp=20 the assumptions hold less well — softer contacts, more sag, less foot-floor friction — so the policy has compensation work to do.

**The corollary for choosing tasks:** the residual layer earns its keep where the analytical model is *almost right* but missing something. If the model is already at the limit (as Newton straight-walking shows), pushing harder on the policy is wasted compute. The productive question becomes "what is the analytical model NOT modeling?" — and that's the task to train the policy on. See [Limitations and next steps](#limitations-and-next-steps) for the perturbation / push-recovery direction.

## How to use it

### Deploy the shipped policy

```bash
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/research/worlds/spot_residual_deploy.wbt
```

The world points at `spot_residual_deploy`, which loads `projects/policies/research/policies/spot_residual_main/policy.onnx` by default. Override with `SPOT_POLICY_ONNX=<path>` to deploy a different residual policy.

### Verify a residual policy

```bash
python projects/policies/research/tools/verify_straight_walk.py \
  --policy projects/policies/research/policies/spot_residual_main/policy.onnx \
  --world projects/policies/research/worlds/spot_residual_deploy.wbt \
  --duration 30 --vx 0.5
```

Verdict is `STRAIGHT` iff `|lat| < 0.5m & |yaw drift| < 0.2rad & fwd > 3m & no fall`.

### Train a new residual policy

```bash
SPOT_FIXED_VX=0.5 python projects/policies/research/training/train_residual.py \
  --run-name spot_residual_v2 \
  --steps 50000 \
  --ckpt-every 5000 \
  --lr 3e-4 --target-kl 0.02 --max-grad-norm 0.5 \
  --log-std-init -1.5
```

~52 seconds for 50k steps on a single CPU env (no GPU needed). Checkpoints are saved every 5k steps; export the best one with `projects/policies/research/inference/export_onnx.py --obs-dim 18`.

### Reward-shaping knobs

All passed via env vars; defaults in `spot_residual_agent.py`:

- `SPOT_RES_R_VX_WT` (2.0) — weight on the exp(-err) forward-velocity tracking term.
- `SPOT_RES_R_VX_SCALE` (0.10) — width of the tracking Gaussian (smaller = sharper).
- `SPOT_RES_R_HEADING_WT` (-1.0) — penalty on (yaw - spawn_yaw)².
- `SPOT_RES_R_LATERAL_WT` (-0.5) — penalty on (y - spawn_y)².
- `SPOT_RES_R_ACTION_WT` (-0.05) — penalty on ||action||² (smoothness).
- `SPOT_RES_R_ALIVE` (0.05), `SPOT_RES_R_TERM` (-10.0) — alive / termination shaping.

## Limitations and next steps

**The reward eventually overfits to forward velocity past ~25k steps.** The 20k checkpoint is the sweet spot for the current weights; later checkpoints (25k, 50k) drift more because the policy keeps growing forward residuals chasing extra vx-tracking reward and the heading penalty doesn't compensate in time. Lighter `R_VX_WT` and heavier `R_HEADING_WT` would push the equilibrium toward straighter rather than faster.

**Single-env training only.** The gym env has a Windows-specific TCP race fix (2.5 s sleep between `Popen` and `connect`) but hasn't been exercised in `DummyVecEnv` with envs > 1. Should be straightforward but untested.

**No KL-to-zero anchor.** The trainer doesn't currently regularize the policy toward zero residual, so after enough updates the policy can drift far from the model walker's behavior. A small `β × KL(π ‖ zero)` term would keep the policy honest — only nudging where it actually helps, returning to the model walker's safe defaults elsewhere.

## Applying the recipe to self-righting — scripted ships, RL is parked

The same model+residual idea was tried for fall recovery: a scripted FSM that classifies fallen orientation (left side, right side, on back, face plant, rear plant) and commands the geometrically-correct leg motion, plus a tiny residual policy on top to refine timing and compensate for friction. The scripted half worked. The residual half didn't, and it's parked.

### What ships: scripted FSM, no policy

The production self-righting is purely scripted, in `projects/policies/control/spot_recovery.py`:

- `classify_orientation(z_axis_world_xyz)` — picks one of UPRIGHT / ON_BACK / LEFT_SIDE / RIGHT_SIDE / FACE_PLANT / REAR_PLANT / UNKNOWN from the body's local Z axis expressed in the world frame.
- `righting_joint_targets(orientation, joint_order)` — returns 12 joint angles per fallen class. LEFT_SIDE extends the left legs (push the ground, lift the chassis) and tucks the right; RIGHT_SIDE mirrors; ON_BACK folds all legs OVER the body (requires the widened URDF `hip_y` limit ≥ 2.5 rad, see `projects/robots/boston_dynamics/spot/urdf/spot.urdf`); FACE_PLANT extends the rear legs to push the body back over its CoM; REAR_PLANT mirrors.
- `RecoveryFSM` — commits to a strategy on fall and holds it for `commit_window_s` (default 1.5 s) before reclassifying, so the body builds rotational momentum from each push instead of flailing as it tumbles across class boundaries. Exits as soon as the body crosses the UPRIGHT threshold (loose: `z_world_z > 0.30`), then waits for `is_upright` (strict: bz, roll, pitch, vz all settled) before resuming gait.

The deploy controller (`projects/policies/research/controllers/spot_residual_deploy/spot_residual_deploy.py`, FSM constructed at lines 222-225, stepped every tick at lines 520-558) runs the FSM every tick, and when it returns `RIGHTING` it commands the scripted pose and skips gait + IK + policy for that tick. No supervisor teleport. Disable with `SPOT_RECOVERY=0` if you want to see the robot just lie there.

### What was tried for the residual layer, and why it stalled

`projects/policies/research/controllers/spot_recovery_agent/`, `projects/policies/research/envs/spot_recovery_env.py`, and `projects/policies/research/training/train_recovery.py` are the parked RL stack. Three iterations of reward + curriculum + observation-space tweaks (commits 2f638404, a93a2865, 06982614, 4bd560d3, 7a9a8589) all converged to "no learning signal" — return curves flat at random-policy level across 100k+ steps. Three root causes diagnosed but not fixed:

1. **Observation space can't represent fall dynamics.** The 16-dim obs is [angular vel, projected gravity, linear vel, body z, roll, pitch, one-hot orientation]. Projected gravity *plus* the one-hot orientation is redundant (gravity alone determines the class), and neither carries the *rate of change* of the fall. The policy can see "I'm on my back," not "I'm rotating off my back fast enough that an extra push would tumble me past upright." A learned righting policy needs the derivative — either by stacking the last 2–3 gravity frames into the obs, or by giving the policy memory (LSTM / transformer). Static-pose obs + feedforward MLP can't get there.

2. **Reward shape has an end-of-episode cliff.** Each step pays a tiny dense `R_GRAV_WT = 0.02 ×` (gravity alignment) and a large sparse `UPRIGHT_BONUS = +500` if `is_upright` is reached before the 400-step budget. The dense term is small enough that cumulative return is negative for almost all trajectories that don't reach upright, so exploration toward "kick the leg hard, see what happens" can't accumulate signal — the policy is incentivized to do nothing risky and chase the small dense gravity term. Fix would be to drop the bonus and use only dense shaping, but that interacts with (3).

3. **Curriculum starts past the geometric tipping point.** The default samples all four fallen orientations from step 1 with roll ∈ [±π/2 ± 0.2]. Near 90° roll is past the point where small leg motion can right the body — the scripted FSM needs the full `OVER` pose (hip_y = 2.50 rad) and ~3–8 s of physics to roll back. The policy is being asked to learn this from random initialization. A real curriculum would start at face_plant only (simplest geometry, no over-the-equator commitment), then add roll cases starting at 30–40°.

There's a fourth, more architectural issue: the residual RL recipe assumes the model layer is *almost right* and the policy nudges it. For walking that's true — gait + IK + balance does walk, and the policy adds the last 10%. For righting, the scripted FSM is on/off (it either reaches upright in 6 s or doesn't), not "almost right." A residual that's bounded to ±3 cm foot offset can't change the outcome much, so even a perfect policy has a small ceiling. The right shape for a learned righting layer is probably "select between scripted poses + small joint-angle residuals," not "small foot-offset residuals over a continuous model layer."

### When to revive it

Don't, unless one of:

- A reproducible task where the scripted FSM genuinely fails (e.g. snagged on an obstacle, on a steep slope, partially under a shelf) and the failure mode is amenable to a learned correction.
- Switching to a recurrent or stacked-frame policy so the obs can carry fall dynamics.
- A real curriculum (face_plant → small-roll → full-roll → on-back) that doesn't start past the geometric tipping point.

The parked code is correct as a *training scaffold* — the env, controller, and reward plumbing all work, and the `SPOT_ENV_STARTUP_S` override on `spot_recovery_env.py` is now in sync with the residual walker. If you revisit this, fix the obs first; tuning the reward or curriculum without doing that won't help.

## Build-time gotchas worth knowing

- **Don't match the gait's foot z to the agent's nominal-pose FK.** Spot's body equilibrium under ODE + kp = 20 sits at ~0.67 m even with the agent's NOMINAL_POSE, higher than the IK-computed foot z of -0.559. If you command the foot to body-z = -0.559, it floats 11 cm above the ground. Use `ground_z = -0.56` with `lateral_y = 0.30` instead — empirically the only combo that keeps feet engaging the ground without stiffening the PD to the point of physics explosions.
- **MuJoCo `ground_z` ≠ Webots `ground_z`.** Same kp = 20 in MuJoCo (URDF-derived MJCF) settles Spot's chassis at ~0.11 m, not Webots's ~0.67 m. The GPU mjwarp residual trainer treadmills with the Webots-tuned `-0.56` because the foot targets push deep into the ground and the IK saturates. Stiffer kp explodes in MuJoCo at dt = 0.016. Run `python projects/policies/research/tools/calibrate_ground_z.py [--mjcf <path>]` to measure the actual settle height for your MJCF and pass the recommended value via `--ground-z` to the trainer (or replace the MJCF with the deploy-faithful one exported by WbNewtonBackend, which has different mass/inertia than the URDF derivation).
- **Don't stiffen the motor PD past kp ≈ 100.** Stiff PD causes physics explosions at swing-to-stance contact transitions (impulsive contact forces with a rigid leg). kp = 20 (the agent default) is right.
- **The gym env must sleep ~2.5 s after `Popen` before calling `_connect`.** Without it, Windows TCP buffers a SYN on a not-yet-listening port; the first `recv()` then sees a RST as the kernel closes the half-open connection. The error looks like a controller crash but isn't.

## GPU mjwarp residual trainer — same recipe, different physics, doesn't yet deploy

The CPU SB3+Webots trainer takes 52 s wall and is plenty for a single-policy run, but multi-policy sweeps (reward grids, scale sweeps, curriculum search) want a faster loop. The intended use of `projects/policies/research/training/gpu_mjwarp_residual_trainer.py` is to take the SAME residual recipe — gait engine + IK + tiny policy — and run it at ~80 k env-steps/s in batched NVIDIA Warp on mujoco_warp physics. 1024 parallel envs × 500 PPO iters in ~157 s on an RTX 5070.

It now produces a policy that walks forward at ~0.34 m/s in its own MuJoCo physics (`projects/policies/research/training/runs/gpu_spot_residual_v3/policy_main.pt`, exported to `projects/policies/research/inference/policies/gpu_spot_residual_main/policy.onnx`). It does NOT yet deploy cleanly into OmniSim — the policy was trained against MuJoCo contacts and kp = 250 position-velocity actuators; OmniSim deploy is ODE contacts and kp = 20. Both contact model and motor authority differ. Use the CPU pipeline + `spot_residual_main` for anything you actually need to deploy.

### Bugs found in the mjwarp env layer (and fixed)

This list is long because every bug masked the next: the open-loop gait+IK drifted backwards at -0.13 to -0.43 m/s and it took peeling each layer to find why. All fixed in 4c19927e.

1. **Body chassis self-collision.** The MJCF auto-generated from the URDF (`C:\tmp\spot_newton_fixed.xml`) has `<exclude>` rules only for parent-child body pairs. The body-chassis box geom geometrically overlaps all four hip_y "thigh" geoms at the spawn pose (16 simultaneous contacts on `mj_forward`). Phantom contact forces pushed the body backward regardless of gait direction. Fixed by zeroing `geom_contype` / `geom_conaffinity` on the body, all 4 hip_x boxes, and all 4 hip_y boxes — only the 4 shins remain in the collision graph. **For any URDF→MJCF auto-conversion, audit `mjd.ncon` at spawn before training.**

2. **NOMINAL fallback hip_x sign was indexed by front/rear instead of left/right.** When IK returned NaN at the workspace edge (every gait cycle's swing-stance boundary), the fallback wrote FR=+0.30 and RL=-0.30 instead of FR=-0.30 and RL=+0.30. Swapped values caused a mid-cycle joint jolt twice per period.

3. **GaitParams defaults were ODE-tuned, not mjwarp-tuned.** `ground_z = -0.56`, `lateral_y = 0.30` is the empirical sweet spot for Webots+ODE settle (body z ≈ 0.67) — in mjwarp the body settles at ≈ 0.60 with feet floating 11 cm above the floor at neutral. New trainer defaults `ground_z = -0.62`, `lateral_y = 0.344`, `front_x = 0.322`, `rear_x = -0.274` match the FK of the canonical stand pose and seat the feet on the floor.

4. **JOINT_LIMITS_HI for hip_y was +3.20** (the widened-URDF range for over-the-back recovery), but the MJCF on disk caps hip_y at +0.60. The 5× mismatch caused the actuators to push the joints against the physical stop every step. Clamped to the MJCF range; opened a follow-up to regenerate the MJCF from the widened URDF when over-the-back motions matter.

5. **Residual interpretation was ±0.03 m foot offsets.** The CPU recipe uses that scale because its model layer ALREADY walks — the residual just polishes. In mjwarp the gait+IK baseline produces near-zero net forward thrust (the leg sweeps cancel by symmetry under soft contacts), so the policy has to drive the entire forward push. ±0.03 m foot offsets give too little joint authority. Switched to ±0.15 rad joint-space delta (matches the from-scratch CPG trainer). Once you do this, you're not strictly doing "residual on the model" anymore — you're using the model as an initial pose plus a learned full-authority controller. It works but it's a different stack from the CPU one.

6. **BZ_FAIL = 0.30** let the policy collect alive bonus while belly-crawling. Raised to 0.45. With it set lower the policy converged on "lower body height for more forward speed" — a degenerate optimum the reward couldn't push it off of.

7. **No body-height penalty.** Even at BZ_FAIL = 0.45 the policy traded height for forward momentum and slowly sank until it hit the threshold (~10 s episodes). Added a quadratic penalty `-bz_pen × max(0, bz_target - body_z)²` (default `bz_pen = -5.0`, `bz_target = 0.55`, both CLI flags). The strongest-walking checkpoint (policy_c, 0.34 m/s) was trained WITHOUT this penalty; later checkpoints with `bz_pen = -100` walk slower (0.21 m/s) but hold height better. The Pareto front isn't well-explored yet.

### Why sim-to-sim ODE deploy doesn't transfer cleanly

The trained policy was committed and a parallel deploy controller written (`projects/policies/research/controllers/spot_gpu_residual_deploy/`, `projects/policies/research/worlds/spot_gpu_residual_deploy.wbt`) that mirrors the trainer's 49-dim observation, joint-space ±0.15 rad residual, and MJCF-tuned gait. The mismatch between training and deploy physics is:

| Quantity                        | mjwarp training        | Webots-ODE deploy      |
| --                              | --                     | --                     |
| Contact model                   | soft (solref/solimp)   | LCP (rigid)            |
| Motor authority                 | kp = 250 PD position+velocity | kp = 20 PID + 80 Nm cap |
| Body settle height              | ≈ 0.60 m               | ≈ 0.67 m               |
| Foot-floor friction             | μ = 2 (MJCF)           | μ = 2 (world ContactProperties) |
| Per-tick                        | 16 ms macro            | 16 ms with internal ODE substeps |

With kp = 20 in deploy, a ±0.15 rad joint target produces ~3 Nm at most, vs. ~37 Nm at kp = 250 in training. The deploy policy ends up with ~12× less actuation authority than it was trained against. Compounded by the body settling 7 cm higher (so the gait's foot z lands above the floor instead of on it), the joint-space policy doesn't have the authority profile it needs.

The cleaner fix is one of:
- **Retrain the joint-space residual against an MJCF that matches deploy physics** — generate a deploy-faithful MJCF (the `OMNISIM_NEWTON_SAVE_MJCF=<path>` recipe does exactly this for the Newton path), then train the GPU residual on top of that. Won't fix the LCP-vs-soft-contact mismatch but will at least match motor authority.
- **Retrain a foot-offset residual on the fixed mjwarp env.** Use `RES_SCALE = 0.03` and rely on the existing `spot_residual_deploy.py` controller. This is closer to the CPU recipe and has a small chance of transferring.
- **Skip the cross-engine deploy entirely** — use the GPU trainer as a reward / curriculum search tool, then port any successful reward shape back to the CPU SB3+Webots trainer for the deployable policy.

### What's committed for GPU-side that you can build on

- `projects/policies/research/training/gpu_mjwarp_residual_trainer.py` — the trainer (CLI: `--envs 1024 --iters 500 --fixed-vx 0.5 --vx-bonus 10 --alive 0.3 --lin 0 --bz-pen -100 --bz-target 0.56`).
- `projects/policies/research/training/view_gpu_spot_residual.py` — live MuJoCo viewer (Python-only, no Webots needed); shows what the policy actually does in its own physics.
- `projects/policies/research/inference/policies/gpu_spot_residual_main/policy.onnx` — exported policy (ONNX round-trip vs torch = 1.8e-7).
- `projects/policies/research/controllers/spot_gpu_residual_deploy/spot_gpu_residual_deploy.py` + `projects/policies/research/worlds/spot_gpu_residual_deploy.wbt` — parallel deploy controller and world for the joint-space residual policy. Loads, runs, doesn't walk well (see "Why sim-to-sim ODE deploy doesn't transfer cleanly" above).

## Resolved: the "drop into a sit" regression was URDF vs binary, not P1.5

On 2026-05-25 launching `projects/policies/research/worlds/spot_residual_deploy.wbt` made Spot drop into a sit instead of walking. The first hypothesis (recorded in this section's earlier version) blamed the P1.5 engine widening — commits `da10b10f` / `b2a94e10` routing `getBodyPosition` / `setBodyPosition` through `WbPhysicsBackend`. Following the prescribed diagnostic (test the model walker alone), the model walker also failed under both ODE and Newton with the same `min_bz=0.27m` signature. But the local binary (`msys64/mingw64/bin/omnisim-bin.exe`, May 18) **predates** every P1.5 commit (all landed May 25 12:13+), so the engine couldn't be the cause.

The actual cause is commit `c9e35da0` (May 24 20:00, "urdf: <rest> tag extension + widen Spot hip_y for self-righting"). That commit did three coupled things:

- Added `<rest>` XML tags to every joint in `spot.urdf` to encode the standing pose so the URDF importer can seed it.
- Widened `hip_y` from `[0.001, 0.60]` to `[-0.50, 3.20]` so the leg can fold over the body for on-back recovery.
- Added C++ in `src/omnisim/vrml/WbUrdfImporter.cpp` to actually parse `<rest>` and apply the seed.

The May 18 binary doesn't have the parser change. It silently ignores `<rest>` tags, so joints spawn at angle 0 (straight legs) instead of the standing pose (0.30, 0.30, -0.60). The body then collapses in ~0.7s regardless of stiffness, which is exactly the failure mode the `WbNewtonBackend` SEED_POSE comment documents.

The fix without rebuilding is to point the worlds at the pre-widen URDF, which is preserved as `projects/robots/boston_dynamics/spot/urdf/spot.classic.urdf`. The `spot_residual_train_newton.wbt` and `spot_residual_deploy_newton.wbt` worlds reference it directly. After the next OmniSim rebuild (which will include the `<rest>` parser), `spot.classic.urdf` can be retired and the widened `spot.urdf` will work everywhere.

The `spot_residual_main` policy is fine — running it through `spot_residual_deploy.wbt` after pointing that world at `spot.classic.urdf` reproduces +5.33m / +0.22m lateral / +1.6° yaw STRAIGHT, matching the doc's original +5.55m / -0.30m / -9.4° verification within noise.

## Newton port — same recipe, different physics, walks STRAIGHT in 84 seconds

> **STATUS 2026-06-11 (later) — ROOT CAUSE FOUND AND FIXED; recipe below needs RETRAINING.**
> The collapse was intra-robot SELF-COLLISION: W1 native mesh registration (Jun 7) made
> Spot's chassis hull overlap its four upper-leg hulls in MuJoCo's own collision, and the
> permanent internal wrench tipped every walker at ~1.2 s. Fixed bridge-side with Webots
> `selfCollision FALSE` semantics (intra-robot shape pairs filtered at finalize; opt-out
> `OMNISIM_NEWTON_SELF_COLLISION=1`) — see newton-ode-replacement-plan.md §W6.
> AFTER the fix, the numbers below still do not reproduce as-written because they were
> implicitly tuned against the pre-W1 AABB-box leg colliders (phantom geometry, now gone):
> - Spot STANDS indefinitely at `OMNISIM_NEWTON_TARGET_KE=500 KD=60..200` (250 is too soft
>   on honest geometry) and gait-steps 30 s / 100% upright at `KE=500 KD=200 SUBSTEPS=8`.
> - The default 0.05 m swing arc GRAZES the honest-geometry ground and drags the body
>   BACKWARD (this was also May's "CPG propels backward in deploy" mystery);
>   `SPOT_GAIT_STEP_HEIGHT=0.09` restores FORWARD walking (+2.7 m / 30 s, zero falls).
> - **HEADING SOLVED (2026-06-11 evening).** Two more root causes found and fixed:
>   (1) `spot_gait.foot_targets` "steered" by rotating each foot's neutral by a CONSTANT
>   half-angle — that statically reorients the stance and produces NO sustained yaw
>   (measured: commanded wz=+0.4 → −0.065 rad/s, inverted slip residue), so every
>   heading-hold PD amplified drift. Fixed: planted feet now sweep TANGENTIALLY about the
>   body (+½ → −½ of the per-stance yaw, the step_x analogue). (2) `SpotResidualEnv`'s
>   agent port (`env_port+200`) was EXACTLY its own simulator's `--port`
>   (`webots_extern_port+100`) — the trainer's TCP connect raced two listeners and ~50%
>   of training launches deadlocked at startup; ports now live in disjoint sub-bands.
>   With the fixed gait + heading lock (`SPOT_HEADING_LOCK=1 KP=1.0 CLIP=0.20
>   LAT2YAW=0.10`): **yaw drift +5.6° over 30 s and −1.4° over 60 s, lateral +0.19 m
>   at 30 s, 100 % upright, zero falls, fully deterministic** (forward +2.79 m / 30 s —
>   the verify bar's `fwd>3 m` line was calibrated on the pre-W1 box-collider physics).
>   Remaining: a slow lateral crab (~0.09 m/s) emerges past ~30 s at locked heading;
>   constant vy trims and vy feedback both flip the gait to other attractors, so the
>   crab (and the last bit of forward speed) is the residual policy's job — the agent
>   now applies the SAME hold as deploy (train/deploy parity) and training launches are
>   reliable post port-fix; 60 k-step runs don't converge yet (59 episodes, huge reward
>   variance) — needs a longer/lower-entropy campaign.
>   One-command repro: `projects/policies/research/runners/run_spot_walk_newton.ps1`.
> - Retraining pipeline is ready: `OMNISIM_NEWTON_SAVE_MJCF` dumps the post-fix model (runs
>   must last ≥6 s — finalize is ~5 s in), the GPU residual trainer's joint classifier now
>   handles the widened ±1.5 hip_x, and a bridge-trained SB3 run
>   (`train_residual_newton.py`, KE/KD honored from env) completes 50k steps in ~6 min.
> The RL worlds (all `physicsBackend "newton"` since the 2026-06-11 migration) load and run.

After the URDF fix, the same residual recipe (gait + IK + balance + tiny MLP) was ported to Newton's MuJoCo CPU backend in a single session. The headline:

|                    | ODE residual (CPU)           | **Newton residual (CPU)**         |
| --                 | --                           | --                                |
| Training time      | 52 s (50k steps)             | **84 s** (50k steps)              |
| Steps/s            | ~950                         | **598**                           |
| Forward in 30s     | 5.55m                        | **4.87m**                         |
| Lateral            | -0.30m                       | **+0.13m**                        |
| Yaw drift          | -9.4°                        | **+3.2°**                         |
| Verdict            | STRAIGHT                     | **STRAIGHT** ✓                    |

This is the **first working Spot walker under Newton on this codebase.** The prior two Newton training attempts (`spot_newton_v3`, `spot_newton_v4`, documented in [archive/spot-newton-state.md](archive/spot-newton-state.md)) both used from-scratch PPO with no model layer — `v3` converged on B-mode (joint-limit riding) and NaN'd on deploy, `v4` never even learned to stand. The model+residual recipe sidesteps both failure modes because the gait engine already produces a competent walker before training begins, so PPO is refining rather than discovering.

### The key insight: under Newton, the model walker alone is already a walker

Open-loop measurement of the gait + IK + balance combo, no policy, under Newton with `OMNISIM_NEWTON_TARGET_KE=250`, `KD=60`, `GROUND_MU=2.0`, `URDF_USE_INERTIA=1`, `SEED_POSE=1`, `FORCE_MUJOCO=1`:

```
30s eval: +5.03m fwd / +0.03m lateral / +2.1° yaw / 100% upright / no fall
```

That's already STRAIGHT, and **slightly better than the ODE model walker baseline (+4.77m / -0.62m / -15.6°).** Newton's stiffer joints, planted feet (μ=2 instead of μ=1), and URDF-derived inertia tensors are a better match for the analytical gait than ODE+kp=20's soft compliance. The policy on top adds ~5% noise around an already-near-perfect baseline; it's not the load-bearing piece here.

### Training pipeline

The recipe is `train_residual.py` driven by `train_residual_newton.py`, which sets the Newton-side opt-ins:

```bash
python projects/policies/research/training/train_residual_newton.py \
    --run-name spot_residual_newton_v1 --steps 50000
```

Internally the launcher exports:

- `SPOT_TRAIN_WORLD=projects/policies/research/worlds/spot_residual_train_newton.wbt`
- `OMNISIM_URDF_USE_INERTIA=1`
- `OMNISIM_NEWTON_FORCE_MUJOCO=1`
- `OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE=1`
- `OMNISIM_NEWTON_SEED_POSE=1`
- `OMNISIM_NEWTON_TARGET_KE=250`, `OMNISIM_NEWTON_TARGET_KD=60`
- `OMNISIM_NEWTON_GROUND_MU=2.0`

`SpotResidualEnv` honors `SPOT_TRAIN_WORLD` (added alongside the existing same-pattern hook in `SpotEnv`), so no Python changes are needed in the residual env itself for the Newton variant. The same `spot_residual_agent` controller runs against either physics backend.

### Checkpoint sweep

All five checkpoints from the 50k run deploy without falling. The 50k checkpoint is the canonical pick.

| ckpt  | fwd   | lateral | yaw     | upright | verdict   |
|-------|-------|---------|---------|---------|-----------|
| 10k   | 4.73m | +0.64m  | +7.1°   | 100%    | DRIFTED   |
| 20k   | 5.10m | +1.07m  | +17.0°  | 100%    | DRIFTED   |
| 30k   | 5.14m | +0.45m  | +5.7°   | 100%    | STRAIGHT  |
| 40k   | 5.01m | -0.19m  | -6.3°   | 100%    | STRAIGHT  |
| **50k** | **4.87m** | **+0.13m**  | **+3.2°**   | **100%**    | **STRAIGHT** |

The training metrics looked alarming mid-run (`ep_len_mean` dropped from 270 in the smoke phase to 60 at 50k), but those are *training* episodes that hit early-termination conditions; deploy with the trained policy never falls.

### Deploy

```bash
SPOT_POLICY_ONNX=projects/policies/research/policies/spot_residual_newton_main/policy.onnx \
OMNISIM_URDF_USE_INERTIA=1 OMNISIM_NEWTON_FORCE_MUJOCO=1 \
OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE=1 OMNISIM_NEWTON_SEED_POSE=1 \
OMNISIM_NEWTON_TARGET_KE=250 OMNISIM_NEWTON_TARGET_KD=60 OMNISIM_NEWTON_GROUND_MU=2.0 \
SPOT_VX=0.5 \
./msys64/mingw64/bin/omnisim-bin.exe projects/policies/research/worlds/spot_residual_deploy_newton.wbt
```

### Limitations and what's next

- **The residual policy is on-par-or-worse than the model walker alone under Newton.** 4.87m / +0.13m lateral with policy vs 5.03m / +0.03m lateral without. The 12-dim ±3cm foot-offset residual mostly adds noise around a near-perfect baseline. The right next step is to find a task where the residual earns its keep: perturbed walking, uneven terrain, faster `vx` commands, recovery from pushes — anything where the model walker's static reward landscape leaves room for the policy to add value.
- **The `<rest>`-tag URDF support is needed** to retire `spot.classic.urdf`. Until then the Newton training/deploy worlds run on the pre-widen URDF, so on-back self-righting is unavailable in those worlds.
- **No mid-training deploy eval callback yet.** Training `ep_len` numbers were a poor predictor of deploy performance in this run (60-tick training episodes deployed without falling for 30s). An SB3 callback that runs `verify_straight_walk.py --duration 30` every N checkpoints would catch genuine collapse early and pick the best checkpoint automatically. The doc's `spot_newton_v4` postmortem makes the same recommendation for the from-scratch trainer.

### Joint-velocity-limit violations under external impacts — diagnosed and worked around

Discovered during the perturbation/push-recovery experiment (cubes thrown at Spot, see the next section): under heavy external impacts, joint velocities momentarily spike to **7.6× the URDF velocity limit** (152 rad/s vs 20 rad/s) and joint positions exceed the URDF range. Normal walking is clean (~5 rad/s, within range), so it's specifically impact transients that break physics.

**Root cause:** `armature=0` in [WbNewtonBackend.cpp:1027](../../src/omnisim/physics/WbNewtonBackend.cpp#L1027) (the `ARMATURE` default, applied to each motorized joint at line 1056). Armature is the joint's *rotor inertia* — the inertia of the motor's spinning rotor + gearbox, ADDED to the limb's inertia. Real Spot motors have substantial rotor inertia (gear ratio² × motor rotor); the URDF doesn't model this and Newton defaults it to 0. With armature=0, the joint inertia is just the limb's small value, and external forces can whip the joint at arbitrary velocity — the URDF `velocity_limit=20 rad/s` only caps actuator-driven motion, not externally-driven motion.

**Workaround (no rebuild):** bump `OMNISIM_NEWTON_TARGET_KD` from 60 to 500. With KD=500, the motor's velocity damping is high enough that even at the effort_limit saturation (80 N·m), one tick of opposing torque brakes the joint significantly. Verified via [`spot_residual_deploy.py`](../../projects/policies/research/controllers/spot_residual_deploy/spot_residual_deploy.py)'s `SPOT_DEBUG_JOINTS=1` diagnostic:

| KD | Walking joint vel | Peak vel at cube impact | URDF limit (20 rad/s) | q_range during impact |
|---|---|---|---|---|
| 60 (legacy) | ~5 rad/s | **152 rad/s** | 7.6× over | [-1.43, +1.01] (past) |
| 250 | ~5 rad/s | 21.78 rad/s | 1.1× over | [-1.20, +0.62] (~at limit) |
| **500** | ~5 rad/s | **~6 rad/s** | **clean** | [-1.20, +0.62] (~at limit) |

KD=500 is now the recommended default for Newton residual deploy. Normal gait is unaffected (KD=500 can easily handle the 1-2 rad/s joint motions the gait engine commands), but impact-driven joint whipping is suppressed.

**Engine env vars added (2026-05-25 rebuild):** `OMNISIM_NEWTON_JOINT_ARMATURE`, `OMNISIM_NEWTON_LIMIT_KE`, `OMNISIM_NEWTON_LIMIT_KD` are now read by [WbNewtonBackend.cpp:1027-1032](../../src/omnisim/physics/WbNewtonBackend.cpp#L1027-L1032). All default to the original hardcoded values (armature=0, limit_ke=10000, limit_kd=100) so existing behavior is unchanged unless opted into.

### Resolution: hard post-step joint-limit clamp + build trap

The above was solved by commit `00280fe0` ("newton: hard post-step joint-limit clamp + stress test"). The fix doesn't tune the solver — it accepts that the solver will sometimes produce out-of-spec state under impact and **clamps the state buffers back to URDF range after each step.** Specifics in [WbNewtonBackend.cpp:2034-2119](../../src/omnisim/physics/WbNewtonBackend.cpp#L2034-L2119):

- `joint_qd` clamped to `±velocity_limit` (URDF).
- `joint_q` clamped to `[lower, upper]` (URDF), with end-stop semantics — only the velocity component driving INTO the stop is zeroed. Velocity heading back toward the valid range passes through, so a joint sitting at q=lo with the actuator pulling it up isn't frozen on the stop.
- Defaults to ON. Opt-out via `OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1`.
- Independent stress test in [tests/engine/joint_limits/](../../tests/engine/joint_limits/): spawns Spot, pelts it with 1 kg cubes at 5 m/s, 5 kg cubes at 10 m/s, then a 20 kg anvil dropped from 2.5 m. Asserts no joint ever exceeds URDF limits. With clamp off: 185 violations in the same run. With clamp on: 0.

**The build trap that blocked us for an afternoon.** After `00280fe0` landed I rebuilt several times without setting `OMNISIM_WITH_NEWTON=ON`. The Makefile defaults to `OFF` — `#ifdef OMNISIM_WITH_NEWTON` blocks then compile to stubs, the entire Newton backend (including the new clamp) is silently absent, and worlds with `physicsBackend "newton"` fall back to ODE without any runtime warning. My env-var tuning (armature / limit_ke / KD=500) was acting on an ODE binary the whole time. Diagnostic prints inside the embedded Python source never executed because the source itself was never compiled in.

The fix is in [scripts/dev/build_with_cd.sh](../../scripts/dev/build_with_cd.sh):
```bash
OMNISIM_WITH_CUDA=OFF OMNISIM_WITH_NEWTON=ON \
  PYTHON_HOME="$PYTHON_HOME" PYTHON_LIB="$PYTHON_LIB" \
  make -C src/omnisim release
```
plus auto-detection of the installed Python version (Makefile defaults to Python314, this machine has Python312).

**Empirical result with the clamp in a correctly-built binary (no env-var tuning needed):** headless run of [spot_residual_deploy_perturb_newton.wbt](../../projects/policies/research/worlds/spot_residual_deploy_perturb_newton.wbt) — Spot walking with cubes thrown every 3 s for ~100 sim-seconds — analyzed per-tick:

```
out-of-URDF-range positions:        0 ticks  / 6,260
over-velocity-limit (>20 rad/s):    0 ticks  / 6,260
big position jumps (>9.4 rad/s):    0 events
sustained at-limit dwells:          0 streaks
acceleration spikes (>300 rad/s²):  0 events
rapid direction reversals:          0 events
```

Every "glitch" the user reported pre-fix was either (a) the env-var-tuned ODE binary masquerading as Newton — i.e., none of the supposed Newton fixes were actually applied — or (b) the recovery FSM commanding the OVER pose (`hip_y=2.50`) into the narrow `spot.classic.urdf` (`hip_y` capped at 0.60) and joints saturating against the limit. The clamp now makes that saturation behave like a real end-stop. The deploy world now uses the widened `spot.urdf` so the recovery FSM has the range it needs.

**Tooling note** for future debugging:
- [`projects/policies/research/tools/analyze_joints.py`](../../projects/policies/research/tools/analyze_joints.py) consumes `SPOT_DEBUG_JOINTS_CSV` output and flags any unrealistic motion category (out-of-range positions, over-velocity, big jumps, acceleration spikes, direction reversals). Run after a deploy headless capture; 0-everything = simulator behaving.
- Embedded-Python stderr does NOT reach the host process reliably (see the embedded-interpreter stdio handling at [WbNewtonBackend.cpp:2482](../../src/omnisim/physics/WbNewtonBackend.cpp#L2482)). For diagnostics inside the Newton helper module, write to a file. The Newton backend's own solver-init log under `.build_tmp/newton_solver.log` confirms the helper is parsed — if that log is missing after a run, `OMNISIM_WITH_NEWTON=ON` wasn't set during the build.

## Why model+residual is our default RL approach going forward

After the Newton port shipped (2026-05-25) we adopted model+residual as the default RL stack for OmniSim robot training. The rationale isn't "it gave us better numbers" — it's that the journey to those numbers exposed a principle worth committing to.

### The principle: RL value scales inversely with analytical-model completeness

Every from-scratch RL attempt in this codebase converged on the same sticky local optima: stand still, B-mode joint-limit riding, slow-walk-then-fall. PPO is exploring noise around a randomly-initialized policy, and "walk forward without falling" has a deep stand-still basin. Twelve reward-shaping iterations on `spot_walk_v*` ate the entire month of April just to dislodge it for one task. Every later task (Newton straight walking, on-back recovery) found a new stand-still or its analog.

Model+residual sidesteps this by giving PPO a *competent baseline to refine*, not a noise prior to discover walking from. The structural work — gait pattern, foot trajectories, IK, body stabilization — is exactly the work that gait engineers solved decades ago. There is no reason for PPO to rediscover it from scratch, and trying makes the optimization problem harder than it needs to be.

Production quadrupeds (Boston Dynamics, ANYbotics, Unitree) all build this way: hand-coded controller doing the structural work, learning sitting on top to refine what's hard to model. We're following the same pattern.

### When to use model+residual (the default)

Use it when:

- **The task has analytical structure.** Locomotion, manipulation with a known kinematic chain, balance — anything where a textbook controller can produce a competent baseline.
- **You can hand-code a controller that is "almost right."** It doesn't have to be perfect; the policy is for the gap. But it has to be in the right basin.
- **You want fast iteration.** Model+residual trains in 60-180 seconds on CPU. From-scratch trains overnight on GPU.

The trade-off is **scope: the policy is bounded by what the residual can do.** Our default scope is ±3 cm foot offsets (±0.15 rad joint delta on the GPU variant). Big enough to refine slip, drift, asymmetry; small enough that even a fully-saturated random output can't violate joint limits or tip the robot.

### When NOT to use model+residual

Some tasks aren't a good fit:

- **Tasks where the analytical model is on/off, not "almost right."** Self-righting from a face-plant is the canonical example: the scripted FSM either reaches upright in 6 seconds or doesn't, and a ±3cm foot offset on top can't change the outcome much. The recovery-RL stack is parked for this reason (see [the self-righting section above](#applying-the-recipe-to-self-righting--scripted-ships-rl-is-parked)). For tasks like this, either expand the residual's action space (e.g., "select between scripted poses + small joint-angle residuals") or use a different architecture entirely.

- **Tasks where the analytical model doesn't exist.** Force-domain manipulation in cluttered scenes, object grasping with arbitrary geometry, contact-rich locomotion on unknown terrain. From-scratch RL (or learned models) makes more sense here — but accept that training will be expensive and reward-shaping iterative.

- **Tasks where the analytical model is already at the limit.** Newton straight walking, as we just showed. The policy is a passenger; training it is wasted compute. The productive move is to choose a harder task that *does* leave room.

### What changes when this is the default

Three practical defaults shift:

1. **First question on a new RL task: "what's the analytical baseline?"** Before training anything, write the hand-coded controller and measure it. Only then decide whether and where the policy should add value.
2. **Training launchers go through `train_residual_*.py`, not `train_robot.py` / `train_spot.py`.** Those legacy launchers stay for backward compatibility and for the rare task that genuinely needs from-scratch, but they aren't the default.
3. **Reward-shaping iteration is for the residual, not for discovery.** Sharp velocity-tracking, lateral / yaw penalties, action-magnitude penalties — all of these now operate on top of a controller that already walks. They can fail forward (slight drift) instead of failing to a sticky optimum (stand still). Iteration is faster because each run takes ~60s instead of 30 min.

## Files

- `projects/policies/control/spot_kinematics.py` — analytic FK + IK.
- `projects/policies/control/spot_gait.py` — trot foot-trajectory generator.
- `projects/policies/control/spot_balance.py` — body-pose PD.
- `projects/policies/control/spot_recovery.py` — scripted self-righting FSM (shipped).
- `projects/policies/research/controllers/spot_model_walk/` — model-only walker (no policy).
- `projects/policies/research/controllers/spot_residual_agent/` — training-side controller (TCP env).
- `projects/policies/research/controllers/spot_residual_deploy/` — CPU/ODE deploy controller (loads ONNX; runs scripted recovery on fall). Works in either OmniSim worlds if the URDF matches the running binary — see "Resolved" above.
- `projects/policies/research/controllers/spot_gpu_residual_deploy/` — joint-space-residual deploy controller for the mjwarp-trained policy. Sim-to-sim transfer to ODE is unreliable; runs but doesn't walk well — see "Why sim-to-sim ODE deploy doesn't transfer cleanly".
- `projects/policies/research/controllers/spot_recovery_agent/` — **parked** residual-RL recovery agent (see "Applying the recipe to self-righting" above).
- `projects/policies/control/spot_gait_np.py`, `projects/policies/control/spot_kinematics_np.py` — vectorized numpy versions of gait + IK used by the GPU trainer (round-trip the scalar references to 1e-6).
- `projects/policies/research/envs/spot_residual_env.py` — gym env.
- `projects/policies/research/envs/spot_recovery_env.py` — **parked** gym env for recovery RL.
- `projects/policies/research/training/train_residual.py` — tiny PPO trainer (CPU, SB3, Webots-ODE physics — the shipped recipe).
- `projects/policies/research/training/gpu_mjwarp_residual_trainer.py` — GPU mujoco_warp version (same recipe, batched envs, MuJoCo physics — does NOT yet deploy cleanly).
- `projects/policies/research/training/view_gpu_spot_residual.py` — live MuJoCo viewer for the GPU-trained policy.
- `projects/policies/research/training/train_recovery.py` — **parked** PPO trainer for recovery RL.
- `projects/policies/research/tools/calibrate_ground_z.py` — measure MuJoCo settle height, recommend gait `ground_z`.
- `projects/policies/research/worlds/spot_residual_train.wbt` — training world (ODE).
- `projects/policies/research/worlds/spot_residual_train_newton.wbt` — training world (Newton MuJoCo CPU).
- `projects/policies/research/worlds/spot_residual_deploy.wbt` — CPU/ODE deploy world.
- `projects/policies/research/worlds/spot_residual_deploy_newton.wbt` — Newton deploy world.
- `projects/policies/research/worlds/spot_gpu_residual_deploy.wbt` — sim-to-sim deploy world for the mjwarp policy.
- `projects/policies/research/training/train_residual.py` — tiny PPO trainer (CPU, SB3, Webots-ODE physics).
- `projects/policies/research/training/train_residual_newton.py` — Newton variant launcher (sets Newton env opt-ins, delegates to train_residual.py).
- `projects/policies/research/policies/spot_residual_main/policy.onnx` — shipped CPU/ODE policy.
- `projects/policies/research/policies/spot_residual_newton_main/policy.onnx` — shipped Newton policy.
- `projects/policies/research/inference/policies/gpu_spot_residual_main/policy.onnx` — GPU/MuJoCo policy (not deployable yet).
- `projects/robots/boston_dynamics/spot/urdf/spot.classic.urdf` — pre-widen URDF (no `<rest>` tags, narrow hip_y limits). Used by the Newton training/deploy worlds until the binary is rebuilt with `<rest>` parser support.
