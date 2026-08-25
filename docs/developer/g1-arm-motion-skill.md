# G1 arm-motion skill — balance on two legs while moving the arms in 3-D

**Status:** ✅ verified (2026-06-28) · deterministic · G1 · skill:
[`projects/policies/skills/humanoid/g1_arm_motion/`](../../projects/policies/skills/humanoid/g1_arm_motion/)

The G1 holds the deterministic two-leg stand and rides a looping routine of bilateral
arm **exercises** — the arms move freely in 3-D while the legs keep balancing, and it
keeps standing through external cube hits. This is the first skill in the new
[policy skills library](../../projects/policies/skills/README.md), and the "aware of
its hands, moving them around doing exercises, while balancing" capability.

## Run

```powershell
python projects/policies/skills/run_skill.py arm_motion --duration 60
python projects/policies/skills/run_skill.py arm_motion --throw --gui
# or: powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1 -ArmMotion [-Throw] [-Gui]
```

## What it is

An `arm_motion` overlay on the `humanoid_stand_deploy` controller (spec
[`specs/g1_arm_motion.json`](../../projects/policies/controllers/humanoid_stand_deploy/specs/g1_arm_motion.json)).
A looping list of exercises, each a set of per-joint sinusoid terms
`dev = ramp · (bias + amp·sin(2π·freq·t + phase))`, eased in/out to the rest pose at
each boundary, with a **rest dwell** between exercises. Six exercises:

| exercise | motion | balance-safe because |
|---|---|---|
| `lateral_raise` | arms out to ~50° and back | symmetric lateral → ~zero fore/aft CoM |
| `arm_circles` | hands circle in a mostly-lateral plane | roll-dominant, tiny pitch |
| `elbow_curls` | forearms curl in/out | distal → low CoM impact |
| `arm_swing` | arms swing alternately fore/aft (a march) | 180° out of phase → fore/aft CoM cancels |
| `wave` | right arm up-and-out, hand waves | one arm up-and-**out** (lateral); left arm balances |
| `flex` | arms out + forearms curl (double-biceps) | lateral + distal, two safe primitives |

The hands' positions are FK-computed each tick (the robot is "aware of its hands") and
logged; a forward-kinematics **CoM-back feedforward** (hip + ankle, proportional to how
far the hands extend ahead of their rest reach) pre-compensates the arms' forward shift.

## The core problem and the architecture

G1's arms-down stand is **marginal** (forward-CoM) and uses the **arms as its fast
balancer** (`arm_balance`, kp=6). That collides head-on with "use the arms for
exercises." Two architectures were tried:

1. **Hand the arms off to the skill** (disable `arm_balance`, balance on the legs) — the
   approach `manip` uses for one arm. It **fails for two arms**: the bare arms-down stand
   without the arm balancer is itself unstable — it toppled *backward at 2.9 s, before the
   exercise even started*. A leg-only fast balancer good enough to replace the arms is the
   open deterministic-balance problem, not a quick tune. (A hot hip PID destabilised the
   stiff stand.)
2. **Superpose the exercise on the live balancer** ✅. Keep `arm_balance` running, ride the
   exercise sinusoid on top, keep amplitudes moderate so the balancer retains authority,
   add a mild hip D assist + the FK feedforward + a rest dwell. This is what ships.

## The hard rule (learned by toppling repeatedly)

> **Bilateral *forward/up* arm raises shift the CoM forward faster than this marginal
> stand can recover — and they fail no matter how gentle.**

The iteration journal, all measured headless:

- Full-amplitude bilateral `lateral_raise` (arms to a T): **toppled forward at 7.9 s** — the
  abducted arms lost their fore/aft balancing authority. Moderating the amplitude (out to
  ~50°) made lateral **rock-solid** (pitch ±0.05 indefinitely).
- Bilateral `front_raise` at amp 0.40 → fell at 18 s; at 0.32 with *strong* feedforward →
  fell *earlier* at 17 s (a strong sudden ankle lean-back **fights** `arm_balance`); at a
  gentle 0.22 → survived one rep then fell at 42 s. Bilateral forward is simply unviable.
- **Fix:** do forward motion as an **alternating march** (`arm_swing`, left/right 180° out
  of phase so the net fore/aft CoM cancels) → survived cleanly (pitch ±0.07). And angle
  reaches **out** not forward.
- Alternating *raises* (one arm up at a time) reintroduced a **start-of-exercise transient**
  (the out-of-phase arm starts at full extension and the ramp-in becomes a fast raise) and
  toppled at the transition. `arm_swing` avoids it by using a **centered** oscillation
  (bias 0 → both arms start at rest).
- A sustained one-arm-up hold (`wave`) **slowly creeps** the body forward over ~8 s; the
  next exercise then tipped the already-leaned body. **Fix:** a **rest dwell** between
  exercises (re-centre) + a shorter, gentler, more-lateral wave.
- The big "overhead/Y reach" kept toppling on even a 0.22 bilateral pitch-up. **Fix:**
  replaced it with `flex` — arms out (lateral) + forearm curl (distal), two separately
  verified-safe primitives, no bilateral-forward component.

The general design rule for any new exercise: **lateral-dominant, fore/aft-cancelling
(alternating, centered), or distal.** Never bilateral-forward.

## Verified (measured, headless, Newton/mujoco_warp)

- **arm_motion alone:** 0 falls over **206 s sim** (2+ full loops of all 6 exercises),
  ends upright, pitch within ±0.06 rad throughout. (Robust both cold and warm.)
- **arm_motion + `-Throw`:** 0 falls; **5/8** cube throws scored solid torso HITs (≤0.18 m
  closest approach) and were absorbed while exercising — the "exercise while being hit by
  external forces" demo.

## Files

- overlay: the `arm_motion` config + runtime + loop blocks in
  [`humanoid_stand_deploy.py`](../../projects/policies/controllers/humanoid_stand_deploy/humanoid_stand_deploy.py)
  (also added an optional `kd` term to `auto_trim` → a mild hip D assist, default 0)
- spec: [`specs/g1_arm_motion.json`](../../projects/policies/controllers/humanoid_stand_deploy/specs/g1_arm_motion.json)
- world: [`research/worlds/g1_arm_motion.omniworld`](../../projects/policies/research/worlds/g1_arm_motion.omniworld)
- launch flag: `-ArmMotion` in [`run_humanoid_stand_deploy.ps1`](../../scripts/dev/run_humanoid_stand_deploy.ps1)
- skill: [`skills/humanoid/g1_arm_motion/`](../../projects/policies/skills/humanoid/g1_arm_motion/)

## What would unlock bigger arm motion

The amplitude ceiling is set by the marginal stand needing the arms as its fast balancer.
To free the arms fully (full overhead reaches, big bilateral raises) you need a **leg-based
fast balancer** good enough to hold G1 without the arms — i.e. the open deterministic
two-leg balance problem (a hot hip PID alone destabilised the stiff position-held stand;
ankle pitch feedback is forbidden on it). Until then: more alternating/lateral motions, and
a per-exercise `balance_arm` hand-off (the `manip` pattern) for genuinely large single-arm
reaches. See [rl-current-state.md](rl-current-state.md) for the strategic picture.
