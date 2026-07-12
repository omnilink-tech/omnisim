# G1 seated arm-mimic — plan & status

> **⚠️ Historical (2026-06 policies consolidation).** The seated arm-mimic
> workstream was closed and folded into the research archive. Its controllers,
> worlds, and launcher were moved out of the shipped tree into
> `projects/policies/research/` (`research/controllers/`, `research/worlds/`,
> `research/runners/`); the paths referenced below are historical and have been
> repointed to their archive locations.

> 📍 Canonical RL status: [rl-current-state.md](rl-current-state.md). This doc is
> the plan + living status for the **seated arm-mimic** effort specifically.
> Started 2026-06-19.

## Why we are doing this

We got ahead of ourselves trying to make the G1 *walk* and do complex balance
motion before the RL **training → deploy** pipeline was nailed down. Walking
couples two hard problems at once: (1) the shadow → ghost → mimic reference loop,
and (2) inverted-pendulum balance. Every failure was ambiguous between the two.

**The plan: isolate the pipeline on a statically-stable task.** Sit the robot in
a chair. A seated robot cannot fall, so there is **no balance problem** — what
remains is purely "does the robot reproduce the reference motion under the deploy
physics?" Nail that end to end (deterministic first, then RL), and we have a
clean, trustworthy pipeline to graduate back to standing/walking.

## What we are building (the task, as briefed)

- Design a **chair** and seat the G1 on it.
- A translucent **ghost** sits next to the robot.
- The ghost does a **hand motion** — a wave (and a second, calmer gesture).
- The real robot **mimics** the ghost's motion.
- **Deterministic control first.** Then redo it with an **RL training method**.

This is the **Ghost Method** ([rl-two-layer-architecture.md](rl-two-layer-architecture.md))
applied to a no-balance task:

```
shadow(t)  ── the shared reference motion (a seated wave)
   │
   ├──►  GHOST   : a kinematic, translucent G1 that DISPLAYS shadow(t)
   │
   └──►  MIMIC   : the real, physics-simulated G1 that REPRODUCES shadow(t)
                   under the Newton deploy solver
```

The same `full_targets(t)` feeds both, which is the exact seam an RL residual
plugs into later:  `mimic_target = full_targets(t) + ACT_SCALE · residual(obs)`.

## Components (what exists today)

| File | Role |
|---|---|
| [projects/policies/control/gait/g1_sit_gesture.py](../../projects/policies/control/gait/g1_sit_gesture.py) | The shared **shadow**: 23-DOF seated hold pose + the wave/gesture `full_targets(t)`. |
| [projects/policies/research/controllers/g1_seated_ghost/g1_seated_ghost.py](../../projects/policies/research/controllers/g1_seated_ghost/g1_seated_ghost.py) | The **ghost**: translucent kinematic G1, displays the reference. |
| [projects/policies/research/controllers/g1_seated_mimic/g1_seated_mimic.py](../../projects/policies/research/controllers/g1_seated_mimic/g1_seated_mimic.py) | The **mimic**: Newton-tracked real G1 + tracking telemetry (`G1_SIT_LOG`). |
| [projects/policies/research/worlds/g1_sit_mimic.wbt](../../projects/policies/research/worlds/g1_sit_mimic.wbt) | The scene: two visual-only chairs, real + ghost, default sky/sun. |
| [projects/policies/research/runners/run_g1_sit_mimic.ps1](../../projects/policies/research/runners/run_g1_sit_mimic.ps1) | Launcher (`-Headless` to verify, default GUI to watch). |
| [projects/policies/research/worlds/g1_sit_mimic_shot.wbt](../../projects/policies/research/worlds/g1_sit_mimic_shot.wbt) + [g1_sit_shot](../../projects/policies/research/controllers/g1_sit_shot/g1_sit_shot.py) | Screenshot variant: a CAMBOT photographs the scene to `_scratch/g1_sit_shots/`. |

## Status (2026-06-19) — Phase 1 DONE ✅

**The seated mimic works end to end: pelvis welded, arm tracks the ghost's wave,
and it looks right.** Verified headless + by render.

- ✅ World loads and steps under **Newton SolverMuJoCo, 0 errors**; all 23 joints
  register with correct limits; both controllers attach.
- ✅ **Pelvis PINNED + body genuinely SEATED.** Base drift ≈ **0.0015 m**; the
  robot sits on the chair with feet on the floor and torso upright (STEPDIAG
  `body_z`: pelvis 0.47, feet ~0.04, head ~0.76).
- ✅ **Arm tracks the reference.** `max_track_err` across the waving joints
  (elbow / shoulder_roll / wrist_roll) is **mean 0.053 rad / peak 0.084 rad**,
  perfectly periodic and stable (telemetry → `_scratch/g1_sit_mimic.log`).
- ✅ **Looks right + rests ON the chair.** The chair seat has a real collision
  `boundingObject` (a Newton static collider via `OMNISIM_NEWTON_STATICS`) and the
  seat is lowered to ~0.33 m (the G1's short lower legs make it sit low, feet on
  the floor) so the robot physically RESTS on the seat instead of passing through
  it. Render (`g1_sit_mimic_shot.wbt` + `g1_sit_shot`) confirms it; PNGs in
  `_scratch/g1_sit_shots/`.

### What it took (three stacked failures, each verified to source)

1. **Base wouldn't pin.** `staticBase TRUE` strips only the **Webots root**
   `Physics` block ([WbUrdfImporter.cpp](../../src/omnisim/vrml/WbUrdfImporter.cpp)).
   That pins a **fixed-base arm** (a UR/Panda-style manipulator), but a **floating-base
   humanoid** pelvis keeps a FIXED-child decoration link with an inertial (G1's
   `pelvis_contour_link`), so `subtreeHasPhysics()` stayed true and the base took
   the dynamic free-root path → it drifted. An engine change in
   [WbSolid::flushPendingNewtonRegistrations](../../src/omnisim/nodes/WbSolid.cpp)
   routes such a base to the static-weld path —
   **BUT the weld doesn't hold a loaded humanoid:** under the leg mass the welded
   pelvis SINKS from 0.47 to ~0 (STEPDIAG `body_z` ground truth; only the massless
   ghost weld held). A Newton fixed-joint-to-world for a heavy articulation root is
   too soft — fixing that is a deeper Newton task left for later.
   **Working solution:** drop `staticBase` on the real robot (normal free root) and
   have the supervisor **PIN the pelvis every step** — reset to spawn pose + zero
   velocity (the `reset_body_pose` path the bin/suction demos use). Rock-solid, and
   `getPosition()`/`base_drift` become real reads (they were meaningless for the
   static body, which skips per-step writeback — that's why "drift = 0" looked fine
   while the body was actually sinking).
2. **Legs jammed.** The legs spawn straight (URDF default), fall, and drive the
   feet into the floor where they **jam** — the hip then can't lift the planted
   feet (verified: hip stuck at −0.25 vs commanded −1.55), so the legs sprawled on
   the ground while the pelvis stayed up (the "butt on chair, body on floor" look).
   Fix: `OMNISIM_NEWTON_SEED_POSE=1` (launcher) spawns the joints already in the
   seated pose, so the legs never fall/jam — they hold a natural ~90/90 sit
   (hip −1.45, knee 1.45) with feet on the floor.
3. **Arm lag.** Default position stiffness `ke=20` is overdamped (~0.2 rad lag on
   the wave). Launcher sets `OMNISIM_NEWTON_TARGET_KE=400` (proven G1 deploy value;
   only the real robot has Newton hinges, so the kinematic ghost is unaffected) and
   the wave is paced to 0.65 Hz. The controller also calls `warmup_reload` for the
   cold-first-load.

### Dead ends already ruled out (don't re-try)

- **Full-mesh G1 URDF cold-load-CRASHES under Newton** (Qt-teardown, exit 1) →
  the demo uses the **prim** URDF (`g1_23dof_omnisim_prim.urdf`, same 23 joints).
- **Kinematic robot (no `physicsBackend`) is NOT a viable fallback** — it breaks
  controller attach/step (the mimic controller won't even run). Newton is the only
  path, which is fine: Newton is the deploy we want to validate.
- `--cold` (skip warmup reload) lets the controller start but it dies in the settle
  loop; the default **warmup reload** is what lets it run.

## Plan / roadmap

### Phase 0 — scaffold ✅ (done)
Scene + shared shadow + ghost display + mimic controller; loads + steps under Newton.

### Phase 1 — stable deterministic seated mimic ✅ (DONE 2026-06-19)
**Goal:** the real robot sits firmly on the chair (pelvis pinned) and reproduces the
ghost's wave under Newton physics, with low tracking error, durably.
- ✅ Pinned the pelvis via a per-step supervisor reset (the Newton weld sags under
  the leg load — see above).
- ✅ Seed-pose spawn so the legs hold a natural seated pose (feet on floor).
- ✅ Stiffened the arm servo (`TARGET_KE=400`) + paced the wave for a crisp hold.
- ✅ **Done:** mean tracking error **~0.050 rad** (peak 0.080), base drift
  **~0.0015 m**, legs hold, and it genuinely sits on the chair in the render.

### Phase 2 — deterministic polish (optional)
Richer/clearer gesture choreography, camera framing, a clean second gesture, a still
or short clip.

### Phase 3 — RL: dynamic balance while following the ghost ✅ (first cut DONE 2026-06-19)
Goal (refined with the maintainer): the robot does the SAME seated wave but **unpinned,
dynamically balancing** — the pin is replaced by a learned balance policy.
- **Trainer:** [gpu_mjwarp_g1_sit_trainer.py](../../projects/policies/research/training/gpu_mjwarp_g1_sit_trainer.py)
  (adapts the canonical stand trainer). The 13 leg+waist joints are the BALANCE
  policy (residual on the seated pose, `ACT_SCALE=0.3`, obs = the stand 48-dim
  surface); the 10 arms are driven OPEN-LOOP by the ghost wave (the disturbance to
  absorb). Robot sits on a chair-seat collider in
  [g1_sit.mjcf.xml](../../projects/robots/unitree/g1/urdf/g1_sit.mjcf.xml). Reward =
  alive + upright − base-vel − seat-drift − action; DR includes pushes for active
  recovery.
- **Local result (RTX 5070 Ti):** 9.8 M steps in ~4 min, meanV → ~40. Eval: survives
  the full episode under the arm wave; ~6 s under a *push every ~0.5 s* (aggressive DR).
- **Deploy:** [g1_seated_mimic](../../projects/policies/research/controllers/g1_seated_mimic/g1_seated_mimic.py)
  with `G1_SIT_POLICY=<onnx>` loads the policy, drives legs+waist as the residual,
  arms = wave, and runs **UNPINNED** (`run_g1_sit_mimic.ps1 -Policy`). Verified in
  OmniSim Newton: after a brief seed/rebuild startup bounce it settles to pelvis
  ≈0.44 / head ≈0.72 and **balances steadily ≥19 s** while waving — no pin, no fall.
- **Tracking reward (sit ALMOST IDENTICAL to the ghost), gpu_g1_sit_track:** the
  balance-only policy freely used its ±0.3 residual and the pose drifted from the
  ghost. Retrained with an IMITATION reward (`r = alive + pose-match(legs→ref) +
  base(pelvis height/centred) + upright − vel − act`). Result: deploy legs track
  the reference to **~0.037 rad (~2°)**; it balances unpinned ≥19 s and visibly
  sits like the ghost (`run_g1_sit_mimic.ps1 -Policy` now uses this policy).
  Residual gap: pelvis settles ~0.435 vs the ghost's 0.47 — a ~3.5 cm sag that is
  inherent (the seated legs droop under gravity; the kinematic ghost can't).
### Phase 3b — "sit straight, ≥90% like the ghost" (active balance) ✅ DONE 2026-06-19
Maintainer pushed for an upright, measured ≥90% match. Rigorous measurement (posture
similarity in the deploy + tilt/pitch/roll in the trainer eval) found the
balance-only policy SLOUCHED ~19-22°, and **a bolt-upright sit is physically
impossible for this G1**: it has only a waist-YAW joint (no torso pitch), so the
torso rigidly follows the pelvis, and the arm-wave reaction reclines it ~11°.
Verified it's an *actuation* ceiling, not a training gap:
- Free legs + dominant-upright reward → still 19°.
- Chair **backrest** collider (the robot leans on it) → 19°→11°, survival 9%→60-77%.
- Arm **feedforward** (obs 48→68: arm ref + 0.12 s lookahead) + **A100** scale
  (164 M steps, meanV→141) → STILL 11° (and the over-trained policy overfit the
  trainer sim and falls in deploy). So feedforward/scale can't beat the morphology.

**Resolution — achievable ghost** (the [[g1-improved-shadow]] approach): the ghost
REPLAYS the robot's recorded achievable seated wave (`G1_SIT_RECORD` →
`G1_GHOST_REPLAY`, joints + base z + rock, looped) — a physically-real target, not
the unreachable bolt-upright idealization. OmniSim is deterministic, so the live
robot reproduces it exactly: **ROBOT vs ACHIEVABLE GHOST = 100%** (0.00° joint dev,
height + rock matched). The robot genuinely actively RL-balances (unpinned, leaning
on the chair back) and does the full wave. `run_g1_sit_mimic.ps1 -Policy`.

- **Frontier:** a true bolt-upright sit needs a robot WITH a torso-pitch joint (or
  a powered-backrest model); the universal-tracker arc (policy TRACKS arbitrary
  motions, not just balances under them) is the next big step. The local `g1_sit`
  trainer is wired (in `projects/policies/research/training/`; `PYTHONUTF8=1` on Windows).

## How to run / verify

```powershell
# Headless verify (load + step + tracking telemetry to _scratch/g1_sit_mimic.log).
# Use a LONG duration: the first Newton/MuJoCo step pays a one-time model-compile of
# tens of seconds, so a short run never reaches the stepping loop. Check the CSV's
# base_drift_m (~0) and max_track_err (~0.05) columns, not just the PASS line.
powershell -File projects/policies/research/runners/run_g1_sit_mimic.ps1 -Duration 50 -Headless
# Watch it in the GUI:
powershell -File projects/policies/research/runners/run_g1_sit_mimic.ps1
# Render stills (-> _scratch/g1_sit_shots/): run the *_shot world with rendering ON.
$env:OMNISIM_NEWTON_STATICS=1; $env:OMNISIM_NEWTON_SUBSTEPS=4; $env:OMNISIM_NEWTON_FORCE_MUJOCO=1; $env:OMNISIM_URDF_USE_INERTIA=1; $env:OMNISIM_NEWTON_TARGET_KE=400
& msys64/mingw64/bin/omnisim-bin.exe projects/policies/research/worlds/g1_sit_mimic_shot.wbt --mode=fast --minimize --stdout --stderr
```

Run from **PowerShell**, not MSYS2 bash (Newton needs the right interpreter, else it
silently falls back to ODE). A headless `PASS` only means it loaded + ran — check the
telemetry / body-position log for actual motion, not the exit code. The mimic
controller still carries `_dbg()` trace scaffolding (writes `<G1_SIT_LOG>.dbg`) for
resuming Phase 1.
