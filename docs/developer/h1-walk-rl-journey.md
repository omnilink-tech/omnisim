# OmniSim RL — H1 walk + deploy-physics fine-tuning (journey)

> **STATUS (2026-07-03):** this journal documents the PRE-PARITY H1 campaign (standalone
> trainer, unsolved deploy gaps, hand-designed ghosts). Its verdicts are superseded: Shadowing
> (in-engine, recorded ghosts) is now the validated FLAGSHIP method — see
> [rl-current-state.md](rl-current-state.md) (top banner) and
> [projects/policies/training/](../../projects/policies/training/README.md). The H1 is next in
> line for the flagship pipeline (record the official re-host -> validate -> preview -> train).


> 📍 **Canonical RL status:** [rl-current-state.md](rl-current-state.md) is the single
> source of truth. If a status claim here disagrees with that file, that file is right —
> fix this one.

**One-line verdict (2026-06-25, current):** the H1 walk is a **few forward steps, not a durable
walk — in the trainer AND in deploy.** The latest closed-loop pure-RL policy (cl3) walks forward
at **~0.31 m/s** and covers **~0.5 m (max ~2.1 m)** before falling, **falling roughly every 1.7 s
even in the trainer**. Durability past a couple of metres is the **unsolved** problem, and it is
*bigger than* the sim-to-deploy gap we spent the session chasing. The **closed-loop architecture**
(obs frame-stacking + a speed-regulating reward) is the durable deliverable — it produces a calm,
feedback-driven, speed-regulating policy — but it did **not** crack durability. **Full pure-RL +
closed-loop campaign: §7 below.** (Earlier Shadowing run-3 history: §1–§6, retained.)

**Last updated:** 2026-06-25.

---

## Status — the three policies, side by side

The champion is run 3 (no Newton fine-tuning). Both deploy-physics fine-tunes — the one on the
URDF-built model and the one on the matched-MJCF model — make the deploy *worse*, and reverse the
walk direction.

| policy | deploy first-fall | deploy distance | direction |
|---|---|---|---|
| **run 3** (`runs/gpu_h1_walk_v3`, no Newton ft) — **champion** | **2.03 s** | **+1.45 m** @ fall, +2.11 m peak | **forward** |
| Newton ft on URDF-built model | 1.58 s | −0.96 m | backward |
| Newton ft on matched-MJCF model | 0.66 s | −0.34 m | backward |

run-3 numbers are **reproduced** (GUI + headless, identical). The fine-tune comparison and all
batched-eval numbers below are from local `_scratch` runs on the laptop GPU.

---

## 1. The Shadowing setup (how H1 walks at all)

H1 is the Unitree humanoid (5-DOF legs, NJ=11). It walks in deploy via the **Shadowing recipe**: a
verified-feasible **kinematic shadow** ("ghost") plus a **bounded RL residual** that tracks it
robustly. The shadow was designed and certified feasible earlier in the H1+Valkyrie walking-shadows
work — this journey is about the RL tracker and the deploy.

Deploy harness:

```powershell
run_humanoid_walk_deploy.ps1 -Robot h1 -Ke 800 -Kd 70 -Settle 0.3
# env: HUMANOID_WALK_ONNX=<policy.onnx>  HUMANOID_WALK_RES_SCALE=0.3
```

- Controller: [`projects/policies/research/controllers/humanoid_walk_deploy/humanoid_walk_deploy.py`](../../projects/policies/research/controllers/humanoid_walk_deploy/humanoid_walk_deploy.py)
- World: [`projects/policies/research/worlds/h1_walk_deploy.wbt`](../../projects/policies/research/worlds/h1_walk_deploy.wbt)

The residual scale is 0.3 — the RL layer nudges the feasible shadow, it does not author the gait.
This is the same two-layer architecture documented in
[rl-two-layer-architecture.md](rl-two-layer-architecture.md).

---

## 2. The deploy champion (run 3)

The shipped deploy policy is **run 3** (`runs/gpu_h1_walk_v3`), trained by
`gpu_mjwarp_h1_walk_trainer.py`. The crucial detail of run 3 is *what model it trains on*: the
mjwarp trainer loads the **dumped deploy model** —

```python
MjModel.from_xml_path("projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml")
```

— so its friction (2.0 on every geom, ground μ 2.0), its kp800/kd70 position+velocity actuators,
and its real foot box all match what the deploy actually runs.

Deploy result (reproduced GUI + headless): **first fall 2.03 s, +1.45 m at the fall, +2.11 m
peak, walking forward.** The open problem is **durability** — it walks ~2 s, not a sustained 10–20 m
bout.

---

## 3. The Phase-2 experiment — fine-tune *in* the Newton deploy physics

This had never been tried for H1: instead of training in batched mjwarp and *hoping* it transfers,
**fine-tune the policy through the actual Newton deploy solver** so the trainer and the deploy are
the same physics by construction.

Built `gpu_newton_h1_walk_trainer.py` (commit `cf200cdc`): it **warm-starts run 3** and fine-tunes
through `newton.solvers.SolverMuJoCo` — the *exact* solver the OmniSim deploy uses. It runs locally
at **~98k env-steps/s @ 1024 envs** on the laptop GPU; a 400-iteration warm-started fine-tune is
**~50–60 s** (no H100 needed).

The hypothesis was the obvious one: same solver → smaller sim-to-deploy gap → a better deploy. It
was wrong, in an instructive way.

---

## 4. The four verified findings

### 4.1 Deploy-physics fine-tuning REGRESSES the deploy — every variant

| policy | deploy first-fall | deploy distance | direction |
|---|---|---|---|
| **run 3** (no Newton ft) | **2.03 s** | **+1.45 m** | **forward** ← champion |
| Newton ft on URDF-built model | 1.58 s | −0.96 m | backward |
| Newton ft on matched-MJCF model | 0.66 s | −0.34 m | backward |

Fine-tuning through the deploy solver did not just fail to help — it made the deploy strictly worse
and flipped the walk to backward in both variants.

### 4.2 SAME SOLVER ≠ MATCHED PHYSICS — the model *source* must match

The first Newton trainer built the H1 model **fresh from `h1.urdf`** (`mb.add_urdf` +
`mb.add_ground_plane()` → newton's **default** ground friction), **not** the deploy's μ=2.0. run 3's
mjwarp trainer, by contrast, loads the **dumped deploy MJCF** (friction 2.0 on every geom, ground
μ 2.0, kp800/kd70 position+velocity actuators, the real foot box). So "same solver" was a red
herring — the *model* the solver was stepping was different.

**Fix shipped** (commit `da8b171a`): `H1_TRAINER_MODEL=mjcf` (now the default) builds via

```python
newton.ModelBuilder.add_mjcf("h1_legs_newton.mjcf.xml")  # the EXACT deploy model
```

Two newton MJCF-parser quirks had to be worked around in-memory, both **physics-preserving**:

1. The dump's MuJoCo plane size is **3 values** (`"5 5 5"` = x, y, grid); newton's parser wants a
   vec2 → trim to the first two.
2. `solref="0.02"` is **1 value**; this hits newton's buggy `parse_vec` `length==1` fallback (it
   passes 3 args into a 2-element vector) → pad to the MuJoCo default `"0.02 1.0"`.

Neither change alters the physics; they only get the dump past the parser.

### 4.3 The MJCF model-match IS a real fidelity gain — *on survival*

Matching the model is not cosmetic. run-3 **batched cold-eval survival**:

- **1.26 s** on the URDF-built model → **1.85 s** on the MJCF model.
- At the `DS_PHASE` launch, batched MJCF survival is **2.05 s**, which **matches the deploy's
  2.03 s**.

So friction/contacts *were* a real model gap — the MJCF match closes it on the survival metric.
(The foot box was already matched: the `h1.urdf` box origin `0.05, 0, −0.05` / full size
`0.28 × 0.03 × 0.024` equals the dumped MJCF half-extents `0.14, 0.015, 0.012`.)

### 4.4 Even matched MODEL + matched SOLVER is NOT enough — the launch IC + obs pipeline dominate

The smoking gun: at the `DS_PHASE` launch, run 3 and the MJCF-fine-tune are **byte-identical in the
batched trainer** — both **2.05 s / −0.72 m** — yet in deploy they diverge enormously (run 3:
**2.03 s forward**; the fine-tune: **0.66 s backward**). And the batched trainer itself walks
**backward** (−0.72 m) where the deploy walks **forward** (+1.45 m) — *same model, same solver, same
launch phase.*

Two causes, both outside the physics:

- **(a) Launch initial condition.** The deploy's 0.3 s **settle** imparts a slight forward launch
  lean (pitch **−0.026**) plus a residual velocity that the batched reset (pitch 0, qvel 0) lacks.
- **(b) Observation pipeline.** The deploy obs uses world-frame `getVelocity()` + **finite-diff**
  qd; the trainer uses **exact MuJoCo-frame** qvel (`qvel[0:3]` world-linear, `qvel[3:6]`
  body-angular). This is the **same class of gap as the historical G1 obs-frame bug**.

So byte-level physics parity in the batched metric still did not predict the single-robot deploy —
the launch state and the obs frame did.

---

## 5. The verdict

**run 3 remains the H1 deploy champion.** Deploy-physics fine-tuning is **not the lever** — it
consistently regresses the deploy regardless of which model it trains on. The MJCF model-match is a
genuine fidelity improvement *on the batched survival metric*, but that metric is a poor proxy for
the deploy launch, so it does not translate into a better deploy.

The **real lever** to improve the deploy *and* extend durability toward a 10–20 m walk is to
**align the batched trainer objective to the deploy**:

1. Replicate the settle-induced **launch lean** (pitch ≈ −0.026) + residual velocity in the trainer
   reset.
2. Match the **obs pipeline** — world-frame velocity / finite-diff qd — instead of exact
   MuJoCo-frame qvel.
3. **Then** fine-tune on the matched MJCF model.

Only after the IC and obs match does fine-tuning on the matched physics have a chance of paying off.

> **⚠️ STRATEGIC COURSE-CORRECTION (2026-06-25) — supersedes §6's lever for the H1 *walk*.**
> The IC/obs alignment plan below is a *within-Shadowing* fix, and the broader 2026-06-25
> diagnosis is that **Shadowing itself is the wrong architecture for the H1 walk** (dynamic-balance
> locomotion). A hand-designed *kinematic* ghost is not a balance solution — balance lives in
> reactive foot placement + push-off timing, which is absent from a joint-angle curve — and the
> bounded residual (res_scale 0.3) has too little authority *and* fights the ghost, so the policy
> gets the natural gait but falls ~2 s with no tuning that fixes it (this session's 7 durability
> runs all overfit a residual gap → deploy regresses). The capture-point (DCM) law was the right
> idea but sagittal-only, while H1's fall went **lateral** (5-DOF legs, NO ankle-roll). **Decision:
> switch the H1 walk to full-authority RL + reward shaping (stability primary + fall termination,
> naturalness as a SOFT style term); keep Shadowing for feasible-reference, non-continuous-balance
> motions (get-up, reach, sit-to-stand, toss-place). NEXT: train H1 pure-RL + reward shaping on
> Modal H100.** Full reasoning + the field's natural-AND-stable recipe:
> [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md); canonical status:
> [rl-current-state.md](rl-current-state.md).

---

## 6. The next lever — launch-IC + obs alignment toward 10–20 m

The durability gap (~2 s, not a sustained walk) and the deploy/trainer direction mismatch are the
same problem: the trainer is not solving for the world the deploy launches into. The ordered plan:

1. **Match the launch IC** in the trainer reset — settle lean + residual velocity, so the policy
   trains from the pose it actually starts deploy in.
2. **Match the obs pipeline** — world-frame `getVelocity()` + finite-diff qd in the trainer, so the
   policy reads the signals it will read in deploy.
3. **Fine-tune on the matched MJCF model** (`H1_TRAINER_MODEL=mjcf`, default) once 1 and 2 hold.
4. Add a durability objective (alive bonus / longer episodes) to push past the ~2 s bout toward a
   sustained 10–20 m walk.

---

## 7. The pure-RL + closed-loop campaign (2026-06-25)

This is the execution of the §5 course-correction: abandon Shadowing for the H1 walk and train a
**full-authority pure-RL** policy with reward shaping, then make it **closed-loop**. It produced the
best policy of the whole effort — and, by finally running the *honest* eval, exposed that the real
wall is **durability**, not the sim-to-deploy gap §1–§6 chased.

### 7.1 Full-authority pure RL — the mode + the reward-hacking fix

`H1_PURE_RL=1` switches the trainer (`gpu_mjwarp_h1_walk_trainer.py`) and the deploy
(`humanoid_walk_deploy.py drive_purerl`) from *shadow + bounded residual* to **full authority**:
`target = nominal + act_scale·action` (no gait baseline; `H1_ACT_SCALE=1.0`). The gait phase clock
stays in the obs as a rhythm cue.

First H100 run reward-**hacked**: with `alive=1 + upright=1` guaranteed each step and a broad
velocity bell (`--vel-sigma 0.25`), the policy learned to **march in place** at vx≈0.12 (target
0.45) — lifting feet on the contact schedule for reward without translating, because real walking
risked the −50 fall. **Fix (legged_gym recipe):** make velocity-tracking *primary* and survival
*cheap* — `--vel 4.0 --vel-sigma 0.08` (sharp) + `--vel-l1 -1.5` (non-saturating linear pull) +
`--alive 0.5 --upright 0.5`. vx then broke the plateau and climbed to ~0.30. Validated locally
before any H100 spend.

### 7.2 The open-loop trap (why the first pure-RL policies didn't transfer)

The retuned pure-RL policies trained a clean-looking trainer walk (vx ~0.27, reward → +0.6, value
crossing positive) but **toppled in deploy in <1.5 s**, every variant, identically. An offline
diagnostic (`_scratch/diag_policy.py`) found why: the policy was **near observation-INDEPENDENT** —
perturbing the obs by a full 1.0 rad/s joint velocity, a body tilt, or a forward velocity changed
the action **< 5 %**. It had learned a mostly **open-loop, phase-clocked** motion and ignored
proprioception. Root cause: the **phase clock in the obs + the contact-schedule reward**
(`--rw-sched`) let the policy score by *following the clock*, and the trainer's forgiving contacts
let that open-loop gait survive — so it never needed to learn feedback balance. `H1_ENV_CORE=1`
(finite-diff qd matching the deploy) did **not** help, because the policy ignored qd anyway.

### 7.3 The closed-loop fix — frame-stacking + a speed-regulating reward (committed `f7a6ac0d`)

The chosen fix, to *force* the policy to use feedback:

- **Observation frame-stacking** (`--obs-history K`): stack the last K obs frames so a memoryless
  MLP can read velocity/accel *trends*. `_build_obs_t` wraps a single-frame `_build_frame_t` with a
  per-env rolling history `[N,K,OBS]`; reset envs get their stack refilled with the current frame
  (no stale pre-reset frames across an episode boundary). Network/buffers/ONNX use
  `eff_obs = OBS_DIM·K`. The deploy mirrors it (`HUMANOID_WALK_OBS_HISTORY`, must equal
  `--obs-history`; launcher `-ObsHistory`).
- **Speed-regulating reward** (`--overspeed`): quadratic penalty on `max(0, vx − vx_target)` — the
  anti-runaway term, so the policy learns to *throttle* when it senses it is too fast (the deploy
  runaway was vx 0.27 trainer → 1.2 m/s deploy).

This **works at the architecture level** (verified on the trained policy): the temporal test
(perturb only the newest frame) now moves the action by **d|a| ≈ 0.15–0.18** (vs ~0 for the
open-loop runs), and feeding forward velocity *lowers* the action (0.45 → 0.33) while backward
velocity raises it — genuine closed-loop speed regulation.

**Regime matters as much as architecture.** A first closed-loop run with *heavy* push-recovery
(`--dr-push-prob 0.10 --dr-push-vmax 1.5`) + a strong `--overspeed -5` made the task so punishing
the policy **saturated** (6/11 joints at the action limit even at rest, trainer value stuck −46) and
fell in 0.18 s. The fix was the **calm** regime (`--dr-push-prob 0.04 --dr-push-vmax 1.0`, soft
`--overspeed -2`, anti-saturation `--act -0.008 --act-rate -0.015`) → **cl3**, the best policy:
calm (|a|mean 0.45, sat 1/11), closed-loop, speed-regulating.

### 7.4 ⚠️ The honest result — durability is NOT solved, in the trainer OR deploy

cl3's **training curves looked excellent** — reward → +0.87, value → **+23** (best ever), vx 0.32.
**But those are per-step averages, and the environment auto-resets a fallen robot and lets it walk
again** — so a frequently-falling policy still shows high per-step reward and high "survival steps."
The honest evaluator (survival, time-to-first-fall, distance-before-fall) tells the real story:

| cl3 trainer eval (2048 robots) | value |
|---|---|
| forward walk speed | **0.31 m/s** (target 0.40) |
| distance before first fall | **mean 0.54 m, max 2.10 m** |
| time to first fall | **~1.7 s** (step 103 / 600) |
| falls per ~10 s | **5.5** (every robot falls; `never_fell_frac = 0.00`) |

So **the trainer policy itself walks ~0.5 m and falls every ~1.7 s.** Deploy (~0.3–0.7 m before
falling, <1 s) is roughly *consistent* with that — the cold start + dynamics gap shave off the
little margin it had. This **reframes §1–§6 entirely:** it was never "great in the trainer,
mysteriously broken in deploy." The walk is **marginal in both**, and the ~1–2 m wall is the *same*
wall in sim and deploy. **Lesson (this is the eval-metric trap from
[rl-newton-migration]/`project_rl_newton_migration`):** never report a walk from reward/value
curves — run the survival + distance-before-first-fall eval; auto-reset makes a faller look durable.

### 7.5 Hypotheses ruled out for the H1 walk (don't relearn)

| hypothesis | verdict | how |
|---|---|---|
| Policy is open-loop / low quality | **ruled out** | closed-loop cl3 (feedback, speed-reg) falls identically |
| Reward shaping wrong | **ruled out** | open-loop, calm, saturated, anti-runaway all tried |
| qd observation mismatch | **ruled out** | `H1_ENV_CORE` matched finite-diff qd → no change (policy ignored qd) |
| CoM-forward pose (the G1 cause) | **ruled out** | `_scratch/h1_com.py`: H1 statically stable, CoM **0.20 m behind** foot front (long 0.28 m foot); G1's fatal 35 mm foot-shift is harmless to H1's 0.20 m margin |
| Warmup-reload phantom velocity | **found + fixed** | `warmup_reload` injected a −0.157 m/s base velocity the speed-reg policy over-reacted to; `HW_NO_WARMUP=1` → vx_obs −0.157→+0.003, survival 0.43→0.72 s — **real but not the root** |
| **Durability** (sim AND deploy) | **THE WALL** | cl3 falls every ~1.7 s in the trainer; this is the open problem, *bigger than* sim-to-deploy |

### 7.6 H100 throughput economics (measured)

mujoco_warp physics is **compute-bound (fp32 SM throughput), not bandwidth-bound** — proven by a
GPU sweep: 16384 envs (586k env-steps/s) → 32768 envs (596k) is **flat** (saturated), and **H200 ≈
H100** (same compute die, +bandwidth doesn't help). **B200 ≈ 1.35× H100** (more compute) but at
~2× cost → **~15 % worse env-steps/dollar**. The tensor cores (≈95 % of a GPU's headline FLOPs)
sit idle — physics has no big matmuls. **Verdict: stay on H100;** the throughput lever is fewer
solver iterations / a cheaper contact model (keeping fidelity), not a pricier GPU.

### 7.7 Status & the open lever

- **Delivered & committed:** the closed-loop architecture (frame-stacking + speed-reg reward), in
  trainer + deploy + launcher, with the offline obs-sensitivity diagnostic. Reusable for any robot.
- **Not solved:** **durability.** cl3 is a real walk (feedback-driven, speed-regulating) but a
  *brief* one — ~0.5 m bouts, in sim and deploy alike.
- **Open levers for durability** (untried this session): (a) longer-horizon / curriculum training
  with a durability-weighted objective and an *honest* survival metric in the loop (not auto-reset
  per-step reward); (b) the trainer↔deploy **runtime** dynamics-gap close-out (contacts/solver/IC,
  the `train_deploy_one_engine` step-unification) — but note this is now a *second-order* concern,
  since the trainer walk is itself only ~0.5 m; (c) H1's structural **lateral** fragility (5-DOF
  legs, NO ankle-roll) — deploy falls were mixed forward-pitch *and* lateral-roll; a step-width /
  lateral-balance term may be needed.

### 7.8 Campaign artifacts

| item | path / id |
|---|---|
| closed-loop trainer (frame-stacking + overspeed + pure-RL mode) | `gpu_mjwarp_h1_walk_trainer.py` (commits `109614c5`, `f7a6ac0d`) |
| pure-RL + obs-history deploy path | `humanoid_walk_deploy.py` `drive_purerl` |
| deploy launcher flags | `run_humanoid_walk_deploy.ps1` `-PureRl -Onnx -ActScale -ObsHistory` |
| local GPU trainer | `projects/policies/research/training/gpu_mjwarp_h1_walk_trainer.py` |
| obs-sensitivity / closed-loop diagnostic | `_scratch/diag_policy.py` |
| CoM-vs-foot-front diagnostic | `_scratch/h1_com.py` |
| best policy (closed-loop, calm) | `runs/gpu_h1_purerl_cl3/` (frame-stack K=5, `--overspeed -2`, calm DR) |
| open-loop pure-RL runs (reference) | `runs/gpu_h1_purerl_h100`, `_ec` (qd-matched), `_cl` (heavy-push) |

---

## General lesson — physics parity is necessary but not sufficient

This extends the rule from [g1-single-source-of-truth.md](g1-single-source-of-truth.md) — *"import
the spec, never re-declare a physics constant"* — with a sharper one:

> **Matched MODEL + matched SOLVER is still insufficient for sim-to-deploy parity. The INITIAL
> CONDITION (the launch state) and the OBSERVATION PIPELINE must also match.**

A batched-trainer metric can be a poor proxy for a single-robot deploy launch **even at byte-level
physics parity** — here run 3 and the MJCF fine-tune are byte-identical in the trainer yet walk
2.03 s forward vs 0.66 s backward in deploy. When chasing a sim-to-deploy gap, diff the launch IC
and the obs frame, not just the physics.

---

## Commits & artifacts

| item | commit / path |
|---|---|
| Phase-2 Newton deploy-physics trainer | `gpu_newton_h1_walk_trainer.py` (commit `cf200cdc`) |
| `H1_TRAINER_MODEL=mjcf` `add_mjcf` model-match fix | commit `da8b171a` |
| Deploy champion (run 3, mjwarp-trained) | `runs/gpu_h1_walk_v3` |
| run-3 mjwarp trainer (loads dumped deploy MJCF) | `gpu_mjwarp_h1_walk_trainer.py` |
| Dumped deploy model | `projects/robots/unitree/h1/urdf/h1_legs_newton.mjcf.xml` |
| Deploy controller / world / harness | `humanoid_walk_deploy.py` / `h1_walk_deploy.wbt` / `run_humanoid_walk_deploy.ps1` |

## Further reading

- [rl-current-state.md](rl-current-state.md) — **canonical RL status** (start here).
- [g1-single-source-of-truth.md](g1-single-source-of-truth.md) — trainer↔deploy physics parity, the "import the spec" rule this lesson extends.
- [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md) — the sibling per-robot journey doc (G1), same honesty level.
- [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) — the general sim-to-deploy recipe.
- [rl-two-layer-architecture.md](rl-two-layer-architecture.md) — feasible shadow + bounded RL residual (the architecture H1 deploys with).
