# G1 walk — the deploy-proven recipe, in-engine (the first durable forward humanoid walk here)

> **Status (2026-07-01): SOLVED to a durable forward walk.** A G1 walks forward, upright,
> by real stepping, trained **in OmniSim's own mujoco_warp engine** (train == deploy, zero
> sim gap). Live GUI deploy: **stayed upright for ~1300 control-steps covering ~15 m of
> ground path** before falling — vs a topple at ~340 steps two iterations earlier.
> Long-horizon batched eval (`EVAL_H=3000`): **surv=0.92, fwd=+8.6 m, gmatch=0.87** — a
> durable **~8 m forward walk** by real stepping. A follow-up **deadband lateral-drift
> penalty** (`W_YPOS`, free ±0.3 m stepping sway) then extended this to a **~15 m forward
> walk that stays upright for the whole run (no fall)** — a gentle residual arc remains but
> no longer topples it. This is the first from-scratch humanoid walk that is durable *and*
> forward here. Trainer:
> [`projects/policies/research/rl_inengine/g1_walk_recipe.py`](../../projects/policies/research/rl_inengine/g1_walk_recipe.py).
> Remaining: a residual left **heading drift** (walks a gentle arc) that eventually topples
> it — tunable, not structural.

## Why this exists (the wrong turn we corrected)

Earlier humanoid-walk attempts here failed two ways, and a 10-source literature sweep of
how the field *actually* trains humanoids (Unitree `unitree_rl_gym`, MuJoCo Playground,
Booster Gym, Humanoid-Gym, Berkeley Humanoid, Berkeley/Radosavovic Digit, DWL, ExBody2,
Figure, 1X) explained why.

- **Sampling-MPC / deterministic gait** — cannot durably walk a biped (structural; all
  deterministic hardware walkers are quadrupeds). The deterministic G1 gait hits a ~0.5 m wall.
- **Residual RL on the deterministic gait** — trains, but never deployed durably (falls
  ~3 s / 1.15 m; the train↔deploy contact gap was never closed). *The "residual walk works"
  claim was wrong and has been retracted.*
- **AMP (adversarial style) on a hand-built gait** — the wrong mechanism for locomotion.
  Nobody in the field makes a humanoid walk this way. It produced a durable *stand* but
  never forward motion.

**The field makes humanoids walk with a specific from-scratch recipe** — velocity-command
tracking + a gait-shaping reward + position-target PD actions + an asymmetric privileged
critic + domain randomization. This file is that recipe, built in-engine, plus two additions
that were decisive here.

## The recipe (what `g1_walk_recipe.py` implements)

- **Action** = joint **position targets** over the model PD, offset from the default standing
  pose (`legs = nom_leg + a·act_scale`, `act_scale 0.6`). No deterministic-gait base — the gait
  *emerges* from the reward. Waist/torso held at nominal (legs-only policy, like Unitree G1).
- **Actor obs (49-dim, proprioception only, NO base linear velocity)**: base ang-vel(3),
  projected gravity(3), velocity **command**(3), leg joint pos(12), leg joint vel(12), previous
  action(12), gait-phase sin/cos(2), **heading sin/cos(2)**.
- **Asymmetric critic**: the value net additionally sees privileged sim-only state — base
  **linear** velocity(3), foot heights(2), foot contact(2) — that the deployed actor never gets.
- **Reward** (all in-engine, batched):
  - `W_TRACK_LIN` × a **tent (V-shaped)** forward-velocity reward — peaks at the commanded
    speed with a gradient on *both* sides (pulls forward from standstill, penalizes lunging).
  - `W_TRACK_ANG` × **heading** term `exp(−(v_y² + yaw²)/σ²)` — face +x, no lateral drift.
  - `W_GHOST` × **soft leg-gait imitation** `exp(−‖q_leg − ghost(φ)‖²/σ²)`, `GHOST_SIG 0.4`
    (generous → a *nudge*, not hard tracking). The **ghost is the deterministic gait's per-phase
    leg angles** (a kinematic walking pattern). This is the anti-reward-hack anchor: a
    dive-and-faceplant has legs nowhere near the ghost → ~0 ghost reward, so the policy must
    actually *step* to earn it. (This is exactly Humanoid-Gym's "reference-gait tracking" reward
    and what Figure/1X do with human walking references.)
  - `W_FEET` × a foot-height-vs-phase reward, orientation, base-height, action-rate, alive,
    and a large **termination** penalty on falling.
- **Domain randomization** (durability): external **pushes** (velocity kicks), per-world
  **motor-strength** scaling, observation noise, randomized **control latency** (the
  top-credited sim-to-real lever — we found it independently as the train→deploy root cause),
  and IC randomization.
- **Velocity-command curriculum**: sample `vx` per episode; widen `[VX_START, VX_MAX]` as
  training improves; deadband small commands so it also learns to stand.
- **Measured by `deploy_eval`** (deterministic mean policy, LONG horizon, fixed seed, at the
  calibrated deploy latency): `surv` / `fall` / `vtrack` / `fwd` / `gmatch` / `dist`. `gmatch`
  distinguishes real stepping from diving; `dist` is the average furthest-forward distance
  walked. **Use a long `EVAL_H` (≥3000, ≥5000 for a 20 m bar)** — a short horizon passes a
  policy that walks 3 m then falls.

## What made it work (the diagnosed sequence)

1. **Recipe backbone** → forward motion appeared but **reward-hacked** (dove forward + face-planted).
2. **Ghost leg-imitation reward** → **durability solved** (surv 0.99, fall ~0.05, gmatch 0.87 =
   real stepping, no diving) — but it stepped *in place* (forward under-served).
3. **Tent (V-shaped) forward reward** → pulled it forward without lunging (exp had no gradient
   far from target → in-place; capped-linear rewarded lunging → unstable).
4. **Heading in the observation + face-+x reward** → it walked *straight* instead of curving
   into a fall (projected gravity is yaw-blind → unbounded heading drift without this).

## How to run

Train (headless, in-engine, ~20 min on this box):

```bash
bash projects/policies/research/mpc/foot_redesign/run_walk_rl.sh 1500 wr_head train headless \
  PPO_NENV=2048 PPO_NSTEPS=32 PPO_HID=256 RES_ACT_SCALE=0.6 PPO_LR=3e-4 \
  PPO_ITERS=3000 EVAL_EVERY=150 EVAL_H=3000 CKPT_EVERY=300 \
  AMP_CTRL_LAT=6 AMP_LAT_MAX=6 OBS_NOISE=0.02 PUSH_INTERVAL=120 PUSH_VEL=1.0 MOTOR_RAND=0.08 \
  VX_START=0.3 VX_MAX=0.6 VX_CURR_ITERS=1200 \
  W_TRACK_LIN=2.0 W_TRACK_ANG=1.0 W_GHOST=2.5 GHOST_SIG=0.4 W_FEET=1.0 W_HEIGHT=20 FALL_PEN=50 \
  RES_POLICY=$OMNISIM_HOME/projects/policies/research/rl_inengine/runs/wr_head.pt \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_step
```

(`wr_head.pt` is produced by a local training run — it is not a tracked checkpoint; `projects/policies/research/rl_inengine/runs/` is gitignored.)

Deploy in the GUI (watch it walk; keep the highest-`surv` checkpoint, deploy is non-monotonic):

```bash
bash projects/policies/research/mpc/foot_redesign/run_walk_rl.sh 260 wr_show deploy gui \
  RES_POLICY=$OMNISIM_HOME/projects/policies/research/rl_inengine/runs/wr_head.pt \
  PPO_HID=256 RES_ACT_SCALE=0.6 VX_MAX=0.6 WALK_WARM_TICKS=120 OMNISIM_NEWTON_SEED_REBUILD=0 \
  OMNISIM_INENGINE_PYMOD=projects.policies.training.g1_walk_recipe:g1_walk_recipe_deploy
```

## Honest open items

- **Residual heading drift** — walks a gentle left arc; the accumulating curve eventually
  topples it (~15 m path). Tune `W_TRACK_ANG` up, or investigate a gait L/R asymmetry, for a
  dead-straight walk.
- **Long-horizon durability** — judge with `EVAL_H≥3000`; the bar is a durable **10–20 m**
  walk, not a 3 m one. Raising the fall-free distance is the next objective.
- **Physics DR is partial** — friction/mass model-parameter randomization is not per-world in
  the shared batched engine (CUDA-graph constraint); we randomize pushes + motor-strength +
  latency + obs-noise + IC, which are the top-credited levers, but friction/mass DR is a v2 item.

## Transfer to the REAL G1 foot (whole-body active balance)

The walk above is on the **"bigfoot"** G1 — an *enlarged* foot (collision box 0.26 × 0.09 m).
The **real** Unitree G1 foot is **0.17 × 0.06 × 0.012 m** (confirmed from the Unitree-derived prim
URDF `g1_23dof_omnisim_prim.urdf`; the "original" `g1_23dof_omnisim.urdf` IS the real robot). A
17 × 6 cm foot under a top-heavy 1.32 m / 35 kg robot is **not statically stable** — the
deterministic quasi-static stand gets `surv=0` on it (the documented "morphology wall"). A real G1
holds it by **active, whole-body balance** (ankle + hip + arm angular momentum), so the right tool
is a *learned* policy, not a deterministic controller. Two additions to `g1_walk_recipe.py` make
this work (real-foot world: `projects/policies/worlds/g1_walk_orig.wbt`):

- **`STAND_SEED=1` (+ `STAND_Z`)** — force a known-good crouch seed (hip −0.30, knee 0.52,
  ankle −0.23) as the reset pose, **independent of the deterministic controller**. Without it the
  trainer captured its spawn from the *collapsed* det-stand and RL was ejected backward at
  −1.46 m/s every step (a static instability — identical failure across all policies *and* rewards,
  so no learning signal). With it: mfwd −1.46 → −0.06, worlds survive, RL bootstraps.
- **`WHOLE_BODY=1`** — the actor controls **all 23 position actuators** (legs + waist + **arms**),
  not just the 12 legs (`_build_all_pos_act` enumerates them; obs 49 → 82). The **arms** provide the
  angular-momentum balance a narrow foot needs: **legs-only stand capped at surv 0.65 and degraded;
  whole-body reached surv=1.0** (a clean, still stand, dist ≈ 0.06 m over a 2000-step horizon).

**Result (2026-07-01):** a **durable learned whole-body STAND on the real G1 foot (surv=1.0)** where
the deterministic stand gets 0, and — climbing the ladder from the stand (warm-start each rung:
stand → +ghost gait +gentle forward → +faster forward +heading) — a **10.6 m forward walk on the
real foot** (clean `EVAL_ONLY` @ H=5000: dist 10.63 m, surv 0.73, straight ydrift 0.00, vtrack 0.67;
reaches ~10.6 m then eventually topples — not yet infinite). Earlier rungs: stand surv=1.0 → ~3 m
(surv 0.95) → 10.6 m. Runs are **very non-monotonic** (a PPO collapse mid-run) → **keep the best
checkpoint**, not the last. `gmatch` is low (~0.4) and that's fine: the policy
balances with its arms and doesn't track the leg-only ghost tightly — the ghost is only a nudge to
step. **Build it as a LADDER** (maintainer's key insight): stand → step + forward → then layer heading →
drift → wobble → DR/latency (same rungs the bigfoot climbed). Throwing the full final reward at the
real foot from scratch stays stuck at `surv=0`.

## The dt/discretization root cause (2026-07-02) — and the first LIVE real-foot walk

The 10.6 m result above **evaluated well but collapsed in the live deploy in ~2 s** — the same
"walks in eval, falls live" gap earlier framed as *~6 ticks of control latency*. The real cause,
found by process of elimination, is a **physics-clock/discretization mismatch**:

- The **live engine** advances `basicTimeStep` (0.016 s) of physics per hook tick, as
  `n_substeps`(4) calls to `SolverMuJoCo.step(dt=0.004)` — and the newton solver does
  `mjw_model.opt.timestep.fill_(dt)` then **one** mjw step per call (`solver_mujoco.py:3485`).
  So **live = 4 mjw steps × 0.004 s**.
- The **batched trainer/eval** stepped 4 × `model_ts`(0.002) = 0.008 s per action — **half** the
  live physics per control step. A policy deployed at half its trained control rate looks exactly
  like a big control latency on a soft, sagging plant. (8 × 0.002 — right total time, wrong
  discretization — *still* fell live: stiff PD + knife-edge contact tell 0.002 from 0.004 apart.)

**Diagnosis toolkit** (all in `g1_walk_recipe.py`): `g1_latency_probe` — an open-loop parity probe
that drives the live world and a batched K=1 buffer with identical held/step targets, tick-aligned
(seed the batched side from live at **t=1**, never at setup — the live mjw only reflects a
newton-side teleport after the first dirty copy-in); a **free-fall parabola** as the physics clock
(`LP_FREEFALL`); gain/option/contact-param diffs (`LP_GAIN_SCALE`, `LP_MATCH_LIVE`, `LP_NO_DIRTY`).
With `LP_SUB=4 LP_TS=0.004` the batched probe matches the live world **bit-identically at every
tick** — hold, step response, even the collapse. Machine-precision train == deploy, verified.

**The fix** is the `DT-PARITY` block in `_walk_recipe_train`: `rm.opt.timestep := tick/n_substeps`
(0.004) and `sub := n_substeps` (4). The deploy hook also gained whole-body support (all-23-joint
newton-DOF map via `_build_all_pos_act` + `mjc_jnt_to_newton_dof`), a recurrent-actor option
(`POLICY_ARCH=lstm|gru`, truncated-BPTT PPO, hidden state carried in rollout/eval/deploy), a crouch
teleport, and an exact-mjw **handoff reset** (`WALK_MJW_RESET`).

**Result (2026-07-02): the first LIVE walk on the real G1 foot.** Retrained on the live-exact
physics (`wr_exact_it1200`: eval surv 0.82 / dist 4.75 m @ `EVAL_IC=0.05`, ghost reward active at
`W_GHOST=1.5`), the live GUI deploy walked **~6 s / ~10 m of ground path** (x +3.9 m, y +9.1 m — a
strong left drift) before falling — matching the eval's prediction, because the eval physics now
*is* the deploy physics. Open items: the left drift (heading/anti-drift weights need retuning on
the true physics) and durability (surv 0.82 → 1.0; resume from the best checkpoint, gentle LR).

## The mimicry arc (2026-07-02): recorded ghost → posture anchor → ghost-residual (gmatch 0.81)

The maintainer's objective evolved from *walk* to *walk like the reference* (target gmatch 0.8–0.9).
The sequence, with each wall and what broke it:

1. **Recorded ghost** (`GHOST_LUT_JSON`, commit 932b1321): the reward ghost became a recording of
   the official Unitree policy walking in this engine. Result: +76% speed, drift killed — but
   free-tracking mimicry plateaued at gmatch ≈ 0.31.
2. **Posture anchor** (`STAND_POSE=unitree`, `_seed_legs()`): the policy's spawn/nominal/obs
   anchor was the deep bootstrap crouch (hip −0.30/knee 0.52) while the recording cycles around
   the official nominal (hip −0.10/knee 0.30) — a ~0.2 rad error baked into every tick. Aligning
   them bought +0.10 gmatch instantly (0.31 → 0.43) where a 16:1 reward-weight dominance had
   moved it +0.007. **Anchor alignment beats reward weight.**
3. **Free-tracking ceiling**: a precision round (pushes off, noise slashed, tight reward sigma
   with the metric pinned via `GHOST_SIG_EVAL`) measured the honest ceiling: **gmatch ≈ 0.47**.
   A raw-joint autopsy of that walker (via `WALK_GAIT_LOG`) explained why: it walks a *different
   gait family* — knees locked at 0.87 rad, left hip yaw permanently twisted −0.5 rad — stable
   and smooth on screen, structurally unlike the reference. (This also killed the
   record-our-own-gait "achieved ghost" plan: it would enshrine the weirdness.)
4. **Ghost-residual** (`GHOST_RESIDUAL=0.15`): leg actions become `ghost(phase) ± 0.15 rad`
   (policy residual bounded), arms/waist keep full authority for balance. Mimicry becomes
   STRUCTURAL — gmatch ≥ exp(−(0.15/0.35)²) ≈ 0.83 by construction — and RL solves only balance.
   **Result: gmatch 0.811 at surv 0.992 (fall 0.010) in 300 iterations**, live-verified
   (5.7 m / 41 s, straight, no fall). This is the shadowing-residual idea that failed for months
   pre-parity, redeemed on bit-exact physics with free arms.

**Reading gmatch honestly**: `gmatch = exp(−mean_12_leg_joints((q−ghost(φ))²)/σ²)`, σ pinned at
0.35 rad; 0.81 ⇒ RMS ≈ 0.16 rad ≈ 9° per leg joint *at the robot's own phase clock*. It does NOT
measure: forward speed / stride length (contact dynamics), arm motion (free by design), base
bob/attitude, or the *display* ghost's phase (the side-by-side world's lock-mode ghost paces by
distance-at-reference-vx, so a slower robot sees a slow-motion hologram — a pure visualization
artifact). A visual "they differ more than 0.81" impression usually comes from exactly these
unmeasured channels.

## WBMATCH 0.9 reached — metric v2 + the ghost split (2026-07-03)

The maintainer-approved reference is **ghost v3c** ([`ghost_v3c_lut.json`](../../projects/policies/controllers/g1_ghost/ghost_v3c_lut.json)):
the champion's own full-body gait, phase-folded at 256 bins, **harmonic-smoothed** (first 5 Fourier
harmonics for legs, 4 for arms — 0.004 rad of micro-jitter removed, shape intact) with the base sway
**becalmed** to a pure once-per-stride sinusoid at ±3° roll (the official walker's class; the raw
champion rocks ±6°, which read as "violent" in preview).

**Metric v2 (eye-aligned, vs the approved ghost)**: `WBMATCH = mean[legs, arms, attitude, speed]`
where attitude is scored against the ghost's *own sway* `att_lut(φ)` (phase-locked rocking earns
credit; jitter doesn't), arms against the *recorded* shoulder LUT, and speed against the ghost's
`vx`. Loaded via **`GHOST_METRIC_JSON`** — the *control* corridor stays on `GHOST_LUT_JSON`.
That split is load-bearing: open-loop probes showed NO lut walks without feedback (official surv
0.035, v3c 0.053 at corridor ±0.001), so swapping the control feedforward is pure risk while
swapping the *ruler* is free.

**Result (2026-07-03)**: the unmodified champion `wr_wb_champion.pt`, evaluated on its true physics,
scores **WBMATCH 0.903** `[legs .992  arms .996  attitude .887  speed .731]` at **surv 1.000**
(9.4 m / 24 s, zero falls) — the 0.9 target met with no additional training. Speed ~0.73 is near
this metric's physical ceiling (intra-stride velocity oscillation; the reference itself would score
similarly). Live side-by-side: `run_walk_rl.sh ... deploy gui` with
`G1_GHOST_LUT=ghost_v3c_lut.json G1_GHOST_LOCK=clock`.

**The three environment traps that cost 2026-07-03 a day** (all frozen in
[`run_walk_rl.sh`](../../projects/policies/research/mpc/foot_redesign/run_walk_rl.sh) — never
hand-assemble this env):
1. **Model-physics env is part of the recipe**: `OMNISIM_NEWTON_GROUND_MU=2.0`,
   `OMNISIM_NEWTON_TARGET_KE=200/KD=30`, `OMNISIM_URDF_USE_INERTIA=1` (+ STATICS, SEED_POSE,
   SEED_REBUILD, BASE_GUARD, SUBSTEPS=4, MJWARP=1). Omitting them compiles DIFFERENT physics — the
   champion degrades to surv 0.53 within 100 iters of its own config replayed.
2. **`HUMANOID_STAND_SPEC` is required**: without it the stand controller exits(1) at load and the
   trainer reads its spawn/arm-nominal from a slumped robot.
3. **Fresh-process `EVAL_ONLY` had `cmd_t=0`** — obs said "commanded velocity 0", out-of-distribution
   for a velocity-conditioned policy: the champion scored surv 0.038 *on its own config*. Fixed in
   `g1_walk_recipe.py` (`reset_fields` before `deploy_eval`); any eval run predating the fix is void.

See also: [rl-current-state.md](rl-current-state.md), [deploy-prediction-metric.md](deploy-prediction-metric.md).
