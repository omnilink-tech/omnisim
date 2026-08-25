# The train→deploy gap: the two gaps, the parity contract, and the proven recipe

> 📍 **This is a synthesis + recipe doc, not a status doc.** For the honest
> "is it done?" answer, [rl-current-state.md](rl-current-state.md) is the single
> source of truth — if any status claim here disagrees with it, that file wins.
> This doc ties together the *mechanics* (already owned by
> [train-deploy-unification.md](train-deploy-unification.md) and
> [g1-single-source-of-truth.md](g1-single-source-of-truth.md)) with the
> *external gold-standard recipe* for a policy that deploys durably, and lays out
> the plan. Written 2026-06-26 from a 5-agent audit (G1 code, H1 code, the C++
> Newton backend, the mujoco_warp/newton library internals, and the Unitree
> deploy recipe), with the load-bearing claims **re-verified against the current
> tree** (file:line below).

> 🟢 **MAINTAINER REFRAME (2026-06-30): the residual train↔deploy MICRO-differences are a FEATURE, not a bug.**
> Once the *physics* is identical (in-engine training — same Newton/mujoco_warp model, verified
> by gain + solver-opt probes + a faithful control-path parity probe), the small remaining
> closed-loop differences (fp/order from separate buffers, warm vs reset solver state, the engine
> tick wrapper vs a hand-rolled step loop, the stand→gait handoff) are **free domain randomization**.
> A policy that survives them is *robust by construction* — and that robustness is exactly what
> transfers to a new world or to real hardware. **Do NOT chase bit-identical train/deploy parity as
> the fix** (that breeds a policy that memorizes one loop and is brittle to everything else). Parity
> is a *diagnostic* (confirm physics/control match), not the goal. The goal is a policy ROBUST to the
> variation: train it with obs noise (`OBS_NOISE`), action-rate smoothing (`W_ARATE` — smooth policies
> survive micro-diffs; this gave the first durable in-engine humanoid walk, 70 s 0 falls), varied
> initial conditions (`IC_RAND_*`), and mild dynamics randomization — tuning the *amount* (too much DR
> goes defensive). ⚠️ Deploy performance is **non-monotonic** with training (a more-trained policy can
> deploy *worse* = it overfit the exact loop); regularizers flatten that, and you keep the most *robust*
> checkpoint, not the highest training return. See the in-engine PPO trainer
> [`projects/policies/research/rl_inengine/g1_walk_ppo.py`](../../projects/policies/research/rl_inengine/g1_walk_ppo.py)
> (`_ppo_train_gpu`) and the CHAOS verdict in
> [closed-loop-chaos-diagnostic.md](closed-loop-chaos-diagnostic.md) (same lesson: fix with robustness, not parity).

## TL;DR

There is not one train→deploy gap — there are **two, and conflating them is why
this has stayed open**:

1. **Pipeline-parity gap** — the observation pipeline, the launch initial
   condition, and a short list of physics residuals (COM, contact stiffness)
   differ between trainer and deploy. **Real, fully enumerated, and mostly
   *already built* as code paths — the parity machinery just defaults OFF and the
   shipped launcher doesn't enable it.**
2. **Durability gap** — the policy is **marginal even inside the trainer** (G1
   champion ~7.3 s, H1 ~1.7 s/fall). Closing the pipeline gap does **nothing** for
   this; proven when H1's deploy-solver fine-tuning *regressed* the deploy.

> **Update 2026-06-27 — for the G1 STAND, the gap that bit was #1 (pipeline), not
> #2 (durability).** A closed-loop obs diff
> ([`closed_loop_parity_compare.py`](../../projects/policies/research/training/closed_loop_parity_compare.py))
> showed the deploy fed the policy a **different base angular-velocity** than the
> trainer (engine `R^T·getVelocity` vs mujoco `qvel[3:6]` — different frame/scale,
> ~2 rad/s divergence at tick 1 while the pose matched to 3e-3). That is squarely a
> **pipeline-parity (observation) bug**, not durability — the policy was durable in
> the trainer and *would* have been in deploy if it had been fed the signal it
> trained on. Fix: replace engine ang-vel with the reproducible **finite-diff of
> `proj_gravity`** in trainer+eval+deploy → the from-scratch G1 **stand stands 32 s
> / 0 falls in the binary** (commit `1416d52c`,
> [rl-current-state.md](rl-current-state.md) top, [binary-parity-probe.md](binary-parity-probe.md)
> finding 4). **Takeaway:** before attributing a deploy fall to durability, run the
> closed-loop obs diff — "marginal even in the trainer" is the durability signature;
> "durable in the trainer, falls in deploy" is the pipeline signature. The **walk**
> has not yet been put through this test (it feeds the same suspect ang-vel term).

**The engine is not the cause.** The deploy's `newton.solvers.SolverMuJoCo` runs
the **same `mujoco_warp` kernels** the GPU trainer calls — verified in code and
against NVIDIA's Newton docs. Model parity is single-sourced and CI-enforced.

**The fix for the durability gap is to adopt Unitree's proven deploy contract**
(the obs/action/reward recipe that demonstrably runs the *same weights* across
IsaacGym → MuJoCo → real G1/H1 hardware), not to keep refining bespoke
ghost-tracking for the walk. §4 is that recipe as a checklist.

---

## 1. Why the engine is not the gap (verified)

Training steps batched `mujoco_warp` on GPU; deploy steps one Newton world via
`newton.solvers.SolverMuJoCo`. These are the **same kernels**, not two engines:

- Solver construction: [`OmNewtonBackend.cpp:1463`](../../src/omnisim/physics/OmNewtonBackend.cpp#L1463)
  `self.solver = newton.solvers.SolverMuJoCo(self.model, **_kw)`; the
  `mujoco_warp` backend (vs MuJoCo-C CPU) is selected by
  `OMNISIM_NEWTON_MJWARP` ([`:1430`](../../src/omnisim/physics/OmNewtonBackend.cpp#L1430) →
  `use_mujoco_cpu=False`). NVIDIA's `SolverMuJoCo` is an adapter over standalone
  `mujoco_warp` — identical kernels (vendored Newton 1.3.0 / mjwarp 3.10.0 /
  warp 1.14.0, per [train-deploy-unification.md](train-deploy-unification.md) §1).
- Model parity is **single-sourced + CI-enforced**: trainer and deploy derive
  every physics constant from `g1_physics.json` + `g1_physics_spec.py` + the prim
  URDF; proven 0 real-physics field gaps + 8.5 mm/10-tick golden trajectory. Owned
  by [g1-single-source-of-truth.md](g1-single-source-of-truth.md).
- **BINARY-level confirmation (2026-06-26):** until now every parity proof above
  compared the trainer against the `g1_deploy_runtime.py` Python *extract*, stepped
  by the same in-process solver — never the real `omnisim-bin`. A deterministic
  open-loop probe ([binary-parity-probe.md](binary-parity-probe.md)) now runs the
  SAME scripted stand in the trainer Python and in the **actual binary** and diffs
  the trajectories: they track to **~0.15° median / ~0.3° plateau** per joint, and
  the binary's compiled robot mass (28.03 kg) equals the trainer's exactly
  (fixed-link fusion is dynamically exact). The residual is float32 + marginal-stand
  chaos, not a physics gap → the engine/model is confirmed equivalent **in the
  binary**, so a from-scratch RL walk that still falls in deploy is a **durability**
  problem (§3–4), not a physics one.
- Substeps match: `SPEC.newton_env()` emits `OMNISIM_NEWTON_SUBSTEPS=4` for the
  G1 pipeline → 4×4 ms per 16 ms env-step, same on both sides. (Note: the engine
  *default* is 1 substep; the 4 is set by the spec, not the engine.)

**Caveats that matter even with identical kernels** (from the library research):
GPU `mujoco_warp` is **not run-to-run deterministic** (float32 + non-associative
atomics), and Newton pins `mujoco-warp ~3.8.x` while mujoco_warp `main` is 3.10.x
— **pin versions** or contact/solver behaviour can silently shift. A single
"7.3 s vs 33.8 s" comparison on a chaotic biped is therefore weak evidence;
validate on **distributions over many seeds**, not one rollout.

---

## 2. The pipeline-parity gap — enumerated, re-verified against the current tree

Every divergence below was re-checked against today's code. The recurring shape:
**the trainer default and the deploy default are opposite, a parity path exists,
and it is default-OFF / not wired into the shipped launcher**
([`g1_deploy_launch.py`](../../projects/policies/research/runners/g1_deploy_launch.py)).

| # | Divergence | Trainer (default) | Deploy (default) | Status / fix |
|---|---|---|---|---|
| 1 | **Link COM** | true URDF COM (`add_urdf`) | **COM-at-origin** — `OMNISIM_NEWTON_USE_LINK_COM` read at [`OmSolid.cpp:3221`](../../src/omnisim/nodes/OmSolid.cpp#L3221), default OFF; launcher never sets it | ACTIVE. `g1_physics.json._residuals.deploy_link_com` calls it *"the dominant cause of trajectory drift"* (body_ipos ≤0.154 m). Flip ON both sides **+ retrain** (old walker was tuned to the wrong COM → falls ~11.7 s with the fix). |
| 2 | **Joint velocity qd** | **exact MuJoCo `qvel`** (`gpu_mjwarp_g1_walk_trainer.py:1006-1008`, `G1_ENV_CORE` default OFF) | **forced finite-diff** of position sensors ([`g1_walk_deploy.py:770`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py#L770)) — the Newton engine exposes joint *angle* but not *rate* to controllers (`getJointHingeAngleRate` returns `-1` in [`OmPhysicsBackend.cpp:147`](../../src/omnisim/physics/OmPhysicsBackend.cpp#L147); only ODE implements it) | ACTIVE & architectural. Parity path = `G1_ENV_CORE=1` (trainer routes qd through the shared `JointVelEstimator`). See the **cleaner engine fix in §5**. |
| 3 | **Launch IC** | teleport reset to pitch≈0, qvel≈0 | ~0.3–0.5 s gravity settle → forward lean + residual velocity + sagged legs at tick 0 | ACTIVE. `g1_env_core.settle_steps` reproduces it in the trainer, default-OFF; the trainer model currently settles to −21° vs deploy −2.66° → reconcile before flipping (train-deploy-unification.md §4). |
| 4 | **Contact ke/kd** | Newton builder defaults | 2500 / 100 (`SPEC.newton_env()`) | ACTIVE. `g1_physics.json._residuals.contact_ke_kd`; `spec.apply_contact_to_trainer` default OFF. Different foot-ground compliance. |
| 5 | **Analytic ankle balance PD** | code default **ON** (kp −1.5 / kd −0.2, `gpu_mjwarp_g1_walk_trainer.py:154-158`) | default **OFF**; launcher sets `G1_BAL_*=0` | LATENT — the docs say the champion trained PD-off, but the code default is ON. **Verify the actual training argv** (`G1_TRAIN_BAL_*`); if it wasn't 0, the ankle baseline is off by up to ±0.2 rad. |
| 6 | **Obs sanitation** | `nan_to_num` + clamp(±10) (`:1025-1026`) | **none** — deploy concatenates obs raw ([`g1_walk_deploy.py:859-862`](../../projects/policies/research/controllers/g1_walk_deploy/g1_walk_deploy.py#L859)) | EDGE. Only bites on a transient out-of-range spike (e.g. finite-diff qd at a footstrike). Mirror it in deploy. |

### 2b. MEASURED: the live yaw-veer — a deterministic contact-side reproducer (2026-07-10)

The in-engine Shadowing stack (one live OmniSim world vs the batched mjwarp trainer inside the
same process) has its own divergence #7: **contact generation** (Webots collision detection
feeds the live solver; the batched envs use mjwarp-native collision). It is now *measured*,
with a two-command reproducer:

- **Pure ghost-PD puppet** (corridors ≈ 0 → zero policy authority, crane λ0.9) on the 3 cm
  stair demo world: the live robot veers **−0.76 rad by x ≈ 1.0 m**, *bit-identical between
  runs*. The identical control law in the batched plant walks near-straight (full-episode
  drift 0.17). Deterministic + policy-free + sided = a systematic plant asymmetry, [BUG]-class
  under the closed-loop-chaos taxonomy, not chaos.
- Every policy checkpoint rides the same asymmetry with the opposite sign (deterministic
  **+0.6 rad** early veer — the policy overcorrects), which is the maintainer-visible "the robot
  rotates while climbing the stairs".
- Reproduce: `run_walk_rl.sh 30 <tag> deploy headless ... GHOST_RESIDUAL=0.0001
  GHOST_RESIDUAL_LAT=0.0001 ARM_RESIDUAL=0.0001 ELBOW_RESIDUAL=0.0001 SHRY_RESIDUAL=0.0001`
  on `g1_climb_stairs_demo3.omniworld` (or the 4 m-runway variant `g1_climb_stairs_runway.omniworld`),
  then read `yaw=` in the deploy log. Grade any stair deploy with
  `python projects/policies/training/certify_stair_human.py <tag>_mpc.txt`.

**Policy-side mitigation attempts (2026-07-10, all live-certified FAIL — recorded so nobody
re-burns the GPU):** (a) yaw-error reward alone (`W_TRACK_ANG`) changes nothing — batched
resets have no heading error, so the tent is born satisfied; (b) heading conditioning
(`YAW_TGT_RAND=0.26`) *does* revive live steering (first bounded-yaw live runs; 90°→5°
recovery events) but the equilibrium veer settles at ~25° and stair progress degrades;
(c) constant yaw-torque DR (`YAW_DR_TORQUE=8`, trainer hook in `g1_walk_recipe.py`) does not
null it — the live disturbance is gait-phase-locked, not a constant torque.

**MECHANISM ISOLATED (second pass, same night):** the veer driver is the crane's
`HARNESS_ATT_GHOST` sway spring. Pure-PD with the spring targeting LEVEL instead of the
ghost sway: veer −0.76 → ±0.15 (gone) — but so is the weight-shift propulsion (x/1400 ticks:
1.0 → 0.31). The sway table, indexing, torque frames, damping source, and application path
were verified IDENTICAL live-vs-batched (LATPROBE cfg + code audit) — the difference is that
the live gait lags the ghost clock, so the clock-indexed sway leads the steps and precesses
the robot (a walking-toy turn). `HARNESS_ATT_PHASE` (new deploy knob, LUT bins) re-aligns it:
−6 bins nulls the open-loop veer. The crane yaw-trim (`HARNESS_KYAW` + `HEADING_TARGET`, now
mirrored into the batched harness for parity) holds live yaw to ±4° sustained.

**CONTACT INSTRUMENTATION (same night, CONTACT_LOG=1):** live and batched foot-contact
streams in one grep-able format (`CONTACTLOG side=LIVE|BATCH t= g= d= p= n= bx= ph=`; live
side in the deploy hook, batch side in `deploy_eval`, both env-gated). First results, the
riser-refusal named in numbers: **the batched swing lands 0.5–13 cm ONTO tread 1 (median
6 cm); the live swing only ever touches the first 1 cm — the corner** — and 12% of live
stair contacts are box-EDGE contacts with backward-tilted normals (−0.79, 0, 0.62) that
push the robot off the step (a contact class the batched policy sees 0% of). At first
tread contact the batched base is 2 cm past the riser line with feet underneath; the live
base is **12 cm past with both feet trailing** — the live gait walks posturally
overextended (the sway-lag propulsion deficit expressed as posture). `GHOST_OMEGA_SCALE`
(new knob, trainer+deploy loaders) slows the shared cadence; at 0.85 both sides the live
walker is ruler-straight (max |y| 0.07 m, yaw ≤22°) but the stride still under-delivers
(sometimes barely advances). FINAL PROBE CORRECTION: batched pure-PD on the same world +
sway crane covers 0.99 m/1400 ticks ≡ live 1.0 m — open-loop propulsion is AT PARITY (the
earlier “3× deficit” was the level-crane config, which kills the propulsive sway on both
sides). The closed-loop landing shortfall is therefore the live policy SPENDING STRIDE
FIGHTING THE VEER the batched policy never faces; the single engine asymmetry to kill at
the source is the live-only sway-precession yaw moment. ⚠️ Caveat: the live
`mjw_data.contact` snapshot is PARTIAL
(no floor pairs, one foot only) — stair-contact rows cross-check against FOOTLOG and are
trusted; do not use the live snapshot for contact CENSUS claims without fixing the readback.

**THE REMAINING WALL — the live closed-loop riser refusal:** a policy trained under the full
parity crane (stairhuman10: surv 1.000, zero falls, climbs 4–5 risers batched) walks cleanly
to the staircase live, chest forward, and will NOT step up the first riser — it marches in
place or side-steps along the riser to the staircase edge (bit-repeatable). Speed (VX_MAX
0.35 vs 0.7), checkpoint choice, phase alignment, `maxContactJoints` 10→40, and trim gain
sweeps all leave it unchanged. Open-loop actuation parity is clean (LATPROBE knee tracks to
~2 mrad). The prior champion climbs live ONLY via its yaw-twist strategy — i.e. the twist was
a live-plant workaround, not style. Next experiment: instrument the live vs batched CONTACT
SET (positions/normals/forces) during a commanded step-up at the first riser — the one
measurement that can name the closed-loop difference.

### 2c. RESOLUTION (2026-07-11): there is NO live-vs-batched asymmetry — the missing
measurement was batched YAW

`EVALYAW` (world-0 base yaw logged inside `deploy_eval`, `CONTACT_LOG=1` to enable) closed
the case: the **batched pure-PD puppet veers identically to the live one** (−0.90 rad by
x≈0.93 vs live −0.76 by x≈1.0 — same sign, magnitude, shape). The sway-precession is
intrinsic to the ghost+crane+plant system on BOTH sides. Every earlier "live-only" claim in
§2b traced to a metric artifact: eval `ydrift` is lateral POSITION (meters) — batched yaw
was never logged anywhere. Likewise the chest-forward policy (yaw-trim parity) **stalls at
riser 1 in the batched engine too** (EVALYAW: yaw held ±7°, x oscillating 0.7–0.9 on
tread 1) — exactly the live picture. The plants agree everywhere once the same quantity is
measured on both sides.

**The real problem is therefore pure RL**: no policy has ever LEARNED a chest-forward
step-up — the yaw-twist was every policy's easiest-to-discover riser strategy, and holding
the chest removes it without providing a substitute. (This is the ghost-synth campaign's
"discrete step-up discovery" wall, now confirmed on level ground truth.) Tools added for the
curriculum attack: `COLLECT_ONSTAIR=1` (harvest alive on-stair states during an eval into
`COLLECT_STATES=<npz>`) + `SPAWN_STATES` mid-climb spawns, so policies practice
tread-to-tread stepping directly instead of only ever approaching riser 1 from the floor.

**Meta-lesson (twice in one campaign):** before declaring a train↔deploy divergence, log
the SAME scalar on BOTH sides. `ydrift`-vs-yaw and dist-vs-"climbs straight" both
manufactured phantom engine bugs out of unit mismatches.

**Already unified (do not re-chase — verified):** base linear velocity (both
world-frame, gap 0), base **angular** velocity frame (deploy already rotates
`Rᵀ·ω` to body to match the trainer's body-frame `qvel[3:6]` — the 2026-06-09
"obs-frame fix"), projected gravity (shared closed form), and `njmax/nconmax`
(the **mjwarp** walk trainer and the deploy both default to 256; only the
*Newton* fine-tune trainer pins 128 — not a gap for the shipped walk policy).

**What's already built (the "un-wired machinery"):**
[`g1_env_core.py`](../../projects/policies/research/backends/g1_env_core.py) is the shared
obs/IC layer (CI-enforced by `tests/test_g1_env_core_parity.py`), and
[`g1_step_obs_parity.py`](../../projects/policies/research/training/g1_step_obs_parity.py)
measures the gap. The deploy already calls `g1_env_core`; the **trainer's**
env-core path is behind the default-OFF `G1_ENV_CORE` flag, so **the shipped
policy trains under the un-matched defaults** (exact qvel, teleport IC). The
parity contract exists and is tested — it is simply not the default the policy is
trained against. That is the whole "built but not wired" story. Full mechanics:
[train-deploy-unification.md](train-deploy-unification.md).

---

## 3. The durability gap — the real wall

Even with a perfectly matched pipeline, the policy is fragile **in the trainer
itself** (status owned by [rl-current-state.md](rl-current-state.md);
[g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md);
[h1-walk-rl-journey.md](h1-walk-rl-journey.md)):

- **G1**: champion lives ~7.3 s in the trainer Newton env (the 33.8 s deploy bout
  leaned on the *wrong* origin-COM; correcting it drops to ~11.7 s).
- **H1**: ~0.5 m / ~1.7 s before falling **in the trainer**, on the honest
  survival/distance eval (the auto-reset reward curves hid this).
- **The decisive evidence that this is separate from pipeline parity**: H1's
  Phase-2 trainer fine-tuned *through the exact deploy `SolverMuJoCo` on the
  matched dumped MJCF* — i.e. it closed pipeline parity further — and **every
  variant regressed the deploy** (2.03 s → 1.58 s / 0.66 s). You cannot fine-tune
  your way out of a marginal policy.

The "policy survives longer in deploy than trainer" inversion is **expected**:
the trainer adds DR + pushes + obs-noise + hard randomized resets, so a clean
deploy is simply an easier environment. It is not evidence of a deploy bug.

---

## 4. The proven recipe (external gold standard — the durability answer)

> **Reference, not current OmniSim state.** These are the exact values from
> Unitree's shipping pipeline (`unitree_rl_gym`, `unitree_rl_lab`) and DeepMind's
> `mujoco_playground`, whose policies deploy the *same weights* across IsaacGym →
> MuJoCo → real hardware. They deploy because the **obs vector and control law are
> byte-identical** across train and deploy. This is the target to converge our
> trainer onto.

**Observation** (G1 = 47-dim, H1 = 41-dim), in order, with scales:
`ang_vel·0.25 (body) | proj_gravity (body) | cmd·[2,2,0.25] | (q−default)·1.0 |
qd·0.05 | prev_action | sin/cos phase (period 0.8 s)`.
- **Base linear velocity is NOT observed** (unmeasurable on hardware; forces a
  more robust policy). *We currently observe it world-frame on both sides — fine
  for OmniSim sim-to-deploy, but a robustness smell worth reconsidering.*

**Action / control:** `target = 0.25·action + default_angles` (residual on a
**static squat**: per leg hip_pitch −0.1, knee +0.3, ankle_pitch −0.2);
`τ = kp·(target − q) − kd·q̇` with velocity setpoint **0**. **50 Hz policy**,
decimation 4. G1 gains: hip 100 / knee 150 / ankle 40; kd 2 / 4 / 2.
H1 gains: hip 150 / knee 200 / ankle 40.

**Reward (this is what makes it a *gait*, not just "don't fall"):** tracking_lin
1.0, tracking_ang 0.5, alive 0.15, action_rate −0.01 (anti-jitter, deploy-
critical), **feet_swing_height −20** (foot clearance), **contact +0.18** (pays the
correct foot in stance per the phase clock), orientation −1, base_height −10,
hip_pos −1, contact_no_vel −0.2. Terminate on pelvis/torso contact, height < 0.2 m,
or tilt > 0.8 rad.

**Domain randomization (train only):** friction [0.1, 1.25], base mass
[−1, +3] kg, push 1.5 m/s every 5 s, per-term obs noise (dof_pos .01, dof_vel 1.5,
ang_vel .2, gravity .05). **No kp/kd randomization** — all three reference recipes
agree, matching our own finding that gain-DR hurts.

**Architecture:** **stateless MLP + history-length 5** (frame-stacking), **not an
LSTM.** Unitree's shipped `motion.pt` is an LSTM(64) → carries a hidden state the
deploy must thread (the "held-torque" hazard we already hit); the newer recipes
use MLP + stacking specifically to avoid it. We already have the obs-history
machinery (`--obs-history`).

This recipe sits **between** our two camps — neither full ghost-tracking
(Shadowing) nor pure-RL on nominal. It is **full-ish authority (0.25 rad) on a
static default**, with the gait emerging from a phase-clock + contact reward. Our
own evidence (Shadowing falls ~2 s; pure-RL hit the same ~2 m wall) points
exactly here. See [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md)
for why Shadowing can't stabilise a continuous-balance walk.

---

## 5. The one engine change worth making

The qd divergence (#2) is *architecturally forced*: the Newton backend exposes
joint **angle** but not **rate** to controllers, so the deploy can only
finite-diff. But the exact rate is already in `state_a.joint_qd`. **Add a
`getJointVelocity` that returns it** → the deploy uses exact qd, matching the
trainer's default *and* Unitree's sim2sim (which reads exact `qvel[6:]`),
eliminating a whole divergence class instead of degrading the trainer to
finite-diff to match the deploy. (Touches `OmNewtonBackend` + `OmPhysicsBackend` +
the position-sensor/joint readout path; rebuild-gated like the other native
changes.)

---

## 6. The plan

**Phase A — lock the pipeline contract** (cheap; mostly already built):
1. Make the deploy and trainer share **one** obs builder (`g1_env_core.assemble_walk_obs`)
   verbatim; delete the deploy's bespoke concatenation; add the missing obs
   sanitation (#6).
2. Promote [`g1_step_obs_parity.py`](../../projects/policies/research/training/g1_step_obs_parity.py)'s
   *reported* measurements (qd, IC) to **hard CI asserts**, and add COM + contact
   coverage, so a policy that passes the gate is **guaranteed** obs-identical at
   deploy. (qd is already hard-asserted; IC/COM/contact are currently report-only.)
3. Resolve qd: either `G1_ENV_CORE=1` (finite-diff both sides) **or** the engine
   `getJointVelocity` fix (§5, preferred).
4. Turn on `OMNISIM_NEWTON_USE_LINK_COM=1` + `apply_contact_to_trainer` on both
   sides; verify the balance-PD state (#5) matches.

**Phase B — retrain durable with the §4 recipe** (local GPU, in-engine —
`projects/policies/training/run_walk_rl.sh`; there is no cloud path): stateless MLP +
history 5, action 0.25 on a static squat, phase-clock + contact/feet-swing reward,
standard DR set, trainer **harder** than deploy.

**Phase C — prove it** (the "be sure it works" protocol):
- Disambiguate: turn trainer DR/push/noise OFF → if trainer survival now matches
  deploy, the pipeline gap is closed and the residual delta is pure durability.
- Evaluate on **distributions over many seeds** (honest survival/distance), never
  single-rollout time-to-fall or auto-reset reward curves.
- Pin library versions (Newton ↔ mujoco-warp 3.8.x line).

---

## 7. Where the deep material lives (owners — do not duplicate)

| Topic | Owner doc |
|---|---|
| Loop unification, Layer A/B/C, Phase 0/1/2, qd + IC mechanics | [train-deploy-unification.md](train-deploy-unification.md) |
| The *model* single-source contract + golden parity + CI | [g1-single-source-of-truth.md](g1-single-source-of-truth.md) |
| The heavy-DR recipe + dump-deploy-MJCF trick | [sim-to-deploy-rl-recipe.md](sim-to-deploy-rl-recipe.md) |
| Why Shadowing ≠ a balance solution; the architecture choice | [locomotion-shadowing-vs-pure-rl.md](locomotion-shadowing-vs-pure-rl.md) |
| Per-robot ledgers (the honest numbers) | [h1-walk-rl-journey.md](h1-walk-rl-journey.md), [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md) |
| **Canonical status — is it done?** | [rl-current-state.md](rl-current-state.md) |
