# Deterministic humanoid stand (no RL) — H1, Valkyrie

This is the generalisation of the proven **G1 deterministic "pure-pose" stand**
(see [`g1_stand_arms_deploy.py`](../../projects/policies/research/controllers/g1_stand_arms_deploy/g1_stand_arms_deploy.py)
and `run_g1_standwave_pose_deploy.ps1`) to the other library humanoids added
alongside G1: **Unitree H1** and **NASA Valkyrie**.

> **Deterministic = no learning.** Every robot here holds a fixed,
> statically-stable squat pose with a stiff Newton position hold. No ONNX
> policy, no PPO, no per-step optimisation. The only feedback is, where needed,
> a fixed fore/aft trim baked into the pose. This is the same recipe class as
> the G1 deploy default.

## The recipe (one paragraph)

Find a squat pose whose **CoM projects over the feet**, hold every joint there
with a **stiff position PD** (Newton `TARGET_KE`/`TARGET_KD`), and let the flat
feet provide the support base. A thigh-forward squat (`hip_pitch<0`,
`knee>0`, `ankle_pitch=-(hip_pitch+knee)` to keep the foot flat) puts the CoM
slightly forward, so a small **backward ankle-pitch bias** (`ank_bias`) re-centres
it over the foot. Heavier robots need a **stiffer ankle** so the leg+foot behave
as a rigid body on the flat foot instead of pivoting as an inverted pendulum.

## Architecture (robot-agnostic)

| Piece | Path |
|---|---|
| Generic controller (one file, all robots) | [`projects/policies/controllers/humanoid_stand_deploy/humanoid_stand_deploy.py`](../../projects/policies/controllers/humanoid_stand_deploy/humanoid_stand_deploy.py) |
| Per-robot specs (joints + nominal pose + deploy gains) | `projects/policies/controllers/humanoid_stand_deploy/specs/{g1,h1,valkyrie}.json` |
| Per-robot worlds | `projects/policies/worlds/{g1,h1,valkyrie}_stand_deploy.wbt` (stand) + `…_hstand_cubethrow.wbt` (push-recovery demo) |
| Launch script (sets the proven Newton env) | [`scripts/dev/run_humanoid_stand_deploy.ps1`](../../scripts/dev/run_humanoid_stand_deploy.ps1) |

The controller is fully spec-driven: it reads `HUMANOID_STAND_SPEC` (a JSON with
`joint_order`, `nominal`, `ankle_pitch_joints`, `ank_bias`, `upright`, …),
holds every joint at nominal, logs base `z`/roll/pitch and an OK/FALL verdict
each second. Adding a new humanoid = write one spec JSON + one world.

## Run it

```powershell
# headless survival check (wall-clock duration; sim runs many× faster)
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1       -Duration 30
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot valkyrie -Duration 40
# GUI:
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1 -Gui
```

Per-robot deploy stiffness (`deploy_ke`/`deploy_kd`) and fore/aft trim
(`ank_bias`) live in the spec and are applied automatically; override with
`-Ke/-Kd/-AnkBias` for tuning.

## Status (verified headless)

| Robot | Result | Config | Evidence |
|---|---|---|---|
| **Unitree H1** | ✅ **Stands** | pure pose, KE 800 / KD 70, squat hip −0.30 / knee 0.60 / ankle −0.30, `ank_bias −0.06` | `bz≈0.98`, pitch −0.04, **0 falls over 100 s+ sim** |
| **NASA Valkyrie** | ✅ **Stands** | pure pose, KE 2000 / KD 120, squat hip −0.35 / knee 0.70 / ankle −0.35, `ank_bias −0.02` | `bz≈1.10`, pitch −0.02, **0 falls**. Heavy (130 kg) ⇒ needs the stiff ankle; stable basin `bias ∈ [−0.03,−0.01]` |

### Why H1 and Valkyrie need a backward bias + (Valkyrie) a stiff ankle

The thigh-forward flat-foot squat places the hip — and the heavy torso CoM —
slightly **forward of the ankle**, so at `bias 0` both robots tip forward. A
small backward ankle bias re-centres the CoM over the foot. For the light H1
the basin is wide (`~[−0.05,−0.08]`); the heavy Valkyrie additionally needs
`KE≈2000` so the ankle doesn't deflect under the body and pivot as an inverted
pendulum — below that stiffness there is no open-loop stable point at any bias.

### A structural caveat for closed-chain legs

The method above assumes an **open-chain leg** whose joints can each be held at
a nominal angle. Robots whose leg is a **closed-chain 4-bar with a passive
heel-spring / achilles rod** (the Cassie-class layout) do not satisfy that
assumption: the rod mechanically couples `knee ↔ shin_to_tarsus ↔ toe` to keep
the sole flat through the leg's range. If such a robot is imported from an
**open-chain simplified URDF** — rod/spring links absent, so those joints are
free and uncoupled — then a flat foot and an extended (standing) leg become
mutually unreachable, and no held-joint pose yields a flat-footed stand at any
stiffness or bias. That is a property of the *model*, not a tuning gap. Making
such a robot stand needs either a model with the closed loops expressed as
constraints (loop joints, plus importer support for them) or an active RL
balance policy — out of scope for a *deterministic* stand.

## Push-recovery demo: holding cubes thrown from the sides

The same generic controller drives a **disturbance demo**: 8 cubes are ringed
around the robot and launched one-by-one on a projectile arc at the torso (no
teleport — `setVelocity` on a resting cube with an exact `T = Δh/speed`
ballistic solve, re-applied a few ticks so the warp state flushes). The robot
must **hold its stand while it gets hit**. Worlds:
`projects/policies/worlds/{g1,h1,valkyrie}_hstand_cubethrow.wbt`.

```powershell
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot g1       -Throw   # lean auto-on (spec)
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot h1       -Throw   # holds passively
powershell -File scripts/dev/run_humanoid_stand_deploy.ps1 -Robot valkyrie -Throw   # holds passively
```

| Robot | Cubes | Lean | Result |
|---|---|---|---|
| **G1** | 8 × 0.5 kg | **ON** (spec) | Holds all 8; without the lean it face-plants ~2.7 s on a back-hit |
| **H1** | 8 × 0.7 kg | off | Holds all passively (peak tilt 0.04) |
| **Valkyrie** | 8 × 1.2 kg | off | Holds all passively (peak tilt 0.03) |

### The reactive lean is per-robot, not universal

A **reactive fore/aft ankle lean** (a disturbance-driven feedback term: forward
base-velocity + pitch-rate + pitch-relative-to-low-pass drive a small,
deadbanded, clamped ankle/hip offset that leans the robot *into* the hit) is the
piece that lets the **marginal G1 stand** survive a cube from behind — without
it G1 tips onto its face. But the lean is **enabled per-robot via the spec**
(`"lean": {"enabled": true}` in `g1.json`), because:

- **G1's stand is marginal** (forward-CoM, KE 400) — the lean earns its keep.
- **H1 and Valkyrie are stiff/heavy enough to hold passively** (KE 800 / 2000).
  Their stands are so rigid that the cubes barely move them, and *any* active
  ankle intervention breaks that passive stability — measured: the lean makes
  H1 fall at ~6 s and Valkyrie at ~18 s where both otherwise hold indefinitely.

So **the value of an active recovery layer scales inversely with how robust the
deterministic baseline already is** — the same lesson as the two-layer RL
residual (a stronger baseline leaves less for the correction to add, and a
correction tuned for a weak baseline *destabilises* a strong one). The launcher
only forces `HSTAND_LEAN` when `-Lean` is passed explicitly; otherwise each
robot's spec decides.

> This is one worked instance of a general result: on a **marginal static-balance
> task**, a correction layer on a complete baseline is **net-negative, not
> net-zero** — the "saboteur" end of the residual law. The same demo showed a
> *learned* RL residual self-topple a stationary G1 in ~1.3 s. Full mechanism
> (authority *is* the instability; the sim-to-deploy gap mis-times the feedback
> loop; no kinematic redundancy to absorb it) and the passenger-vs-saboteur
> framing:
> [rl-two-layer-architecture.md §3.8](rl-two-layer-architecture.md#38-stand-and-hold-cubes--passenger-vs-saboteur-and-why-a-residual-on-a-static-stand-goes-net-negative-2026-06-23)
> and [rl-journey.md §8 lesson 10](rl-journey.md#8-cross-cutting-lessons-the-rules-paid-for-in-falls).

### Vibration / steady-state jitter

The lean's deadband (`HSTAND_LEAN_DEADBAND`, default **0.008 rad**) zeroes the
correction below a small threshold so sensor noise doesn't make the ankles
twitch in the steady hold. Tuned sweet spot: it drops steady-state per-second
jitter from ~0.057 to **0.000** (visually no vibration) while still catching the
cubes. Larger deadbands (≥0.014) start to let the stand drift; input low-pass
smoothing (`HSTAND_LEAN_SMOOTH` < 1) adds loop lag that makes the lean *ring*,
so it is left off (= 1.0) by default.
