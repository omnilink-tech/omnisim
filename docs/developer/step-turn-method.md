# Step-Turn Method — solving, then shadowing, a feasible turn (and the foot-spin wall)

**Status: canonical (2026-07-07).** How the G1 was made to turn ~90° in place by its own
footwork — the whole method, and the two findings that unblocked it. Read this before any
"make a legged robot turn / pivot / change direction" work. It corrects two assumptions that
were wrong across the simulator (each fixed in the files noted).

The turn is a genuine **footwork** turn: the harness applies **no yaw torque** during it
(`HARNESS_KYAW=0`), verified per-tick by the `wtz` (harness yaw torque) telemetry reading
**0.00** the whole run. Nothing twists the robot — its feet do.

> This turn is packaged as the **`turn_in_place`** skill ([skill-library.md](skill-library.md) ·
> [`projects/policies/skills/humanoid/turn_in_place/`](../../projects/policies/skills/humanoid/turn_in_place/)).
> Because it is a different-cadence sequence (nb≈225, 153-dim `REF_OBS_WB`), it composes into a BATON
> demo via a **solo context-swap** (`baton.blend: solo_swap`), not an element-wise blend — see the
> `walk_turn_walk` / `turn_solo` sequences and `skill_lib.py blendable walk turn_in_place`.

---

## TL;DR — three steps, two corrected assumptions

1. **SOLVE the ghost by construction** (don't record a human, don't eyeball it):
   `build_step_turn_ghost.py` — footstep plan + inverse kinematics against the G1 model, with the
   **centre of mass kept over the stance foot** every single-support frame. Achievable *by
   construction* because it satisfies the physics.
   → **Corrected assumption #1:** "hand-designed ghosts never train." Wrong as stated. *Eyeballed*
   ghosts don't; a trajectory **solved** for feasibility does. (Fixed: `ghost-design-rules.md`
   Rule 1 now defines two provenance classes — RECORDED and SOLVED/constructed-feasible.)

2. **Enable foot spin friction.** MuJoCo foot contacts default to `condim=3` — slide friction
   only, **zero torsional (spin) resistance**. A planted flat foot freewheels about its vertical
   axis, so hip-yaw rotation never becomes base rotation. This silently caps a step-turn at ~1/3
   of its intended yaw.
   → **Corrected assumption #2:** the under-turn was blamed on morphology / "the policy can't
   turn." Wrong. It was a **contact-model default**. `OMNISIM_FOOT_TORSION=<coef>` raises the foot
   geoms to `condim=4` with a real torsional coefficient (default off → exact prior physics).

3. **Shadow it** with the sequence tracker (`GHOST_SEQ`), which is now the proven path for a
   finite (non-cyclic) motion. See "Shadowing a sequence" below.

---

## 1. Solve the ghost — `build_step_turn_ghost.py`

A turn-in-place, CCW, in `--inc` increments of `--deg / inc` each (default 3 × 30° = 90°). Per
increment: shift the COM over the stance foot → lift + rotate + plant the swing foot at the next
yaw → alternate. The base yaw follows the mean of the planted feet.

- **IK** (`G1IK.ik_legs`): damped least-squares, position-weighted 2× over foot-flatness, bent-knee
  warm start, warm-started frame-to-frame. Converges to **0.3 mm** foot error.
- **Achievability check** (`com_in_foot`): the whole-robot COM (`mj_comPos → subtree_com`)
  ground-projection must stay inside the **stance foot footprint** during single support. At
  `--shift 0.95` this passes **54/54** single-support frames (worst lateral margin 0.025 m inside a
  0.03 m half-width). Plus the validator's position + velocity limits pass. That triple — feet
  reachable, COM supported, joints/vels legal — *is* "achievable by construction."
- **Output**: the standard ghost lut (`leg_lut`, `wb_lut`, `att_lut`, `root_lut`, `arm_lut`,
  `seq=True`, `hold_end=True`). Preview it: `projects/policies/worlds/g1_step_turn_preview.wbt`
  (loads the lut via `controllerArgs` so it doesn't depend on an env var).

Run: `python projects/policies/training/build_step_turn_ghost.py --deg 90 --inc 3 --shift 0.95
--pelvis-z 0.68 --lat 0.119 --cycle-s 10`. Validate: `ghost_validator.py <lut>` → WARN is fine
(the leg-asymmetry + harmonic warnings are inherent to a turn; provenance now reads
"constructed-feasible").

## 2. The foot-spin wall — `OMNISIM_FOOT_TORSION`

**Diagnosis that nailed it:** log the stance hip-yaw actual-vs-ghost in deploy. The legs **tracked**
the ghost's hip-yaw closely, but the base rotated only **30° of 90°**. So it was never a tracking
failure — the leg motion was right, the *rotation it should have produced* leaked away. Cause:
`condim=3` foot contacts have no spin resistance → the pivot foot spins in place. Enabling spin
friction turned **30° → 200°+ instantly** (over-grip) — proof.

`_wr_patch_foot_torsion` (in `g1_walk_recipe.py`) sets the foot geoms to `condim=4` + torsional
friction on **both** the CPU model and the **stepped warp model** (the warp geom arrays are shaped
differently between the deploy solver and the batched trainer, so it applies the whole-array op the
engine's `OMNISIM_NEWTON_ROLL_MU` block uses; torsion only bites contacts that actually spin — the
feet). It runs in the trainer (`_walk_recipe_train`) and the deploy setup, so **train and deploy
must use the same `OMNISIM_FOOT_TORSION`**.

**Stable-config torsion sweep** (tight ghost feedforward: `GHOST_RESIDUAL=0.08`, `W_GHOST=26`,
`W_TRACK_ANG≈5`, `RES_ACT_PEN=0.03`), each retrained + deployed at the matched value:

| torsion | live turn | stability |
|---|---|---|
| 0.0 (condim 3) | ~30° | stable but stuck |
| 0.15 | ~49° | stable, upright |
| 0.25 | ~**90°** | stable, upright (full turn) |
| 0.3 (loose config) | 200°+ / oscillates | **unstable** — over-drive |

More grip transfers more of each kinematic step into real base rotation; too much grip + a *loose*
policy over-torques into a spin-out. The **tight feedforward** (hold the policy close to the ghost's
exact leg motion) is what keeps a high-grip turn stable — the ghost is exactly 90°, so with grip the
feedforward alone delivers ~90° and the policy must not over-push.

## 3. Shadowing a sequence (`GHOST_SEQ`) — the fixes that made it train

A walk-turn-walk / step-turn is a **finite** motion, not a cycle. Three fixes (all in
`g1_walk_recipe.py`, env-gated):

- **Retime to a feasible pace.** Raw human clips (LAFAN1) run at |vx| 1.4 / |wz| 1.45 — infeasible
  for the puppet → NaN at it~55. The *solved* ghost is already slow (`--cycle-s 10`); if you retime,
  keep |vx|,|wz| ≲ 0.4.
- **`ytgt` follows the reference yaw.** In seq_mode the harness lateral-catch + velocity frame must
  rotate *with* the turn (`ytgt = seq_yaw[bin]`), else it fights the rotation → NaN. Applied in
  training **and** deploy (the deploy heading-follow lifted the live turn 24° → 30° before the
  torsion fix).
- **`hold_end` in training.** A non-looping sequence wrapped its phase and snapped the reference
  −90°→0 at the seam → NaN. Pin the phase at the final bin (the deploy already did this).

With a **feasible** ghost, `GHOST_SEQ` trains with **zero NaN** (surv 1.0, gmatch ~0.9). The five
NaN runs earlier were all on an **infeasible human reference** — not a trainer bug.
**Reusable rule: when a shadow tracker NaNs, suspect reference infeasibility first
(is the reference COM-supported + inside joint/velocity limits?), not the trainer.**

---

## ⭐ The way that WORKS: continuous turn + decelerate-to-stop (maintainer's insight, 2026-07-07)

**Don't turn a fixed 90° and try to settle** — that overshoots because arresting a moving turn in
one shot is the unstable part. Instead: **loop a continuous turn** (a steady limit cycle, like
walking in a circle — each step re-establishes balance, so it never has to settle mid-motion), and
**decelerate to a stop at the target heading.** This reaches ANY angle stably, with no bigger foot.

Three findings that made it work:
1. **A continuous turn is stable; the fall was over-spinning with no target.** The 90°-trained policy
   on a longer (180°) ghost turned continuously to +94° upright — it *can* keep turning; it fell
   only because nothing told it to stop.
2. **Freezing the reference does NOT stop it** — the angular momentum carries on (froze at 85° →
   spun to 306° and fell). You must **decelerate**: ramp the reference speed down over the last
   `TURN_DECEL_DEG` so the turn slows to a stop and the policy has time to arrest the spin.
3. **Trigger the stop on the ACTUAL (unwrapped) heading, not the integrated yaw-rate** — the
   integral drifts and fired the stop at inconsistent real angles.

**Deploy knobs** (`_wr_...` in `g1_walk_recipe.py`, after the phase advance): `TURN_TO_DEG` (stop at
this many degrees) + `TURN_DECEL_DEG` (decelerate over the last N degrees). Off by default → exact
prior behavior. **Train the turner on a LONG turn** (`ghost_turn_180_lut.json`, 6×30° increments,
`SEQ_RSI=1` so it learns steady-state turning from any phase) so it can *sustain* the turn past 90°
before the decel-stop lands it.

**Result (`wr_stepturn.pt`, turnstep10 on the 180° ghost):** the puppet turns continuously and
**decelerates to a STABLE hold at ~86-90° CCW** — upright (z~0.68), `wtz=0` (pure footwork, no
rope), held steady for the rest of the run. Stoppable at any achievable angle via `TURN_TO_DEG`
(90, 100, ...). This supersedes the finite-turn ceiling below and needs **no bigger foot**.

Deploy command:
```
OMNISIM_FOOT_TORSION=0.25 ... GHOST_SEQ=1 GHOST_LUT_JSON=.../ghost_turn_180_lut.json
  TURN_TO_DEG=90 TURN_DECEL_DEG=40 HARNESS_KYAW=0 RES_POLICY=.../wr_stepturn.pt
```

## (Superseded) the finite one-shot turn — honest ceiling

**Delivered:** the puppet executes a **stable, upright, footwork** (wtz=0, no rope) step-turn in
the **ghost's CCW direction**, to **~71°** of the ghost's 90° (`wr_stepturn.pt`, turnstep6 it350).
Root cause found + fixed + documented; that is the substance of the result.

**The open ~20° (magnitude ↔ direction ↔ stability, a three-way tension, mapped over 8 training
cycles):**
- Grip sets magnitude: torsion 0→30°, 0.15→49°, 0.25→~70-90°.
- The turn is **bistable** (near-symmetric step → the *sign* is weakly determined): a weak
  yaw-direction reward lets full-magnitude checkpoints flip **CW** (`wr_stepturn_full.pt` = 96° CW).
- A **strong** yaw-direction reward (`W_TRACK_ANG=11`) reliably pins **CCW** but *costs* magnitude
  (~63°) — the policy prioritises heading over completing the rotation.
- Pushing grip to **0.3** to recover magnitude **destabilises** the near-symmetric turn (it reaches
  ~30° then oscillates back) — the honest ceiling of grip-based magnitude.

So ~71° CCW is the stable optimum for a **pure-footwork (wtz=0)** turn.

**The torso-lead experiment (tried, `--waist-lead 0.35`, `ghost_step_turn_torso_lut.json`):** the
waist-yaw leads 0→0.35 rad into the turn (peaking mid-turn), breaking the CW/CCW symmetry at the
reference. It **worked for direction + magnitude** — a checkpoint reached **+96° CCW upright**
(full magnitude, ghost's direction) — but it **overshot to ~141° and fell**. The turn momentum
can't *settle* at 90° by footwork alone: reaching the full magnitude and stopping there is where
pure footwork runs out. The clean fix would be **yaw damping at the top of the turn** — but a
damping torque makes `wtz ≠ 0` (the harness contributing to the yaw), which violates the
"footwork, not the rope" requirement. So a *stable, held, full 90° CCW* is genuinely out of reach
for pure footwork with this foot.

**The honest full fix is morphological:** a **larger foot** (the bigfoot precedent that fixed
shove-recovery on the walk). A ~6 cm foot has a hard physical limit on how much yaw it can grip and
how quickly it can arrest a turn; a bigger foot both transfers the full rotation *and* stops it —
closing magnitude and the overshoot at once. That is the recommended next step for a full 90°.

## Files

- `projects/policies/training/build_step_turn_ghost.py` — the solver.
- `projects/policies/controllers/g1_ghost/ghost_step_turn_lut.json` — the ghost.
- `projects/policies/worlds/g1_step_turn_preview.wbt` — hologram preview.
- `g1_walk_recipe.py`: `_wr_patch_foot_torsion` (the fix), seq `ytgt`-follow + `hold_end`, the
  `wtz`/hip-yaw deploy telemetry.
- `runs/wr_stepturn.pt` (CCW, matches the ghost) / `wr_stepturn_full.pt` (full magnitude).
- Corrected: `ghost-design-rules.md` (Rule 1 two provenance classes), `ghost_validator.py`
  (constructed-feasible provenance + legs-only static arms), `policy-switching.md`.
