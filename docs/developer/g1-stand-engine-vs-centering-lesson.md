# Lesson: the G1 stand tip was a CoM *centering* gap, not an engine bug — prefer the simplest sufficient cause

**One line:** Two sessions fixed the same "G1 stationary stand tips forward at ~1.3–1.8 s under
Newton/MuJoCo deploy" symptom. One reached for an **engine fix** (correcting the fixed-joint
inertia rollup in `OmSolid.cpp`); the other found the **simpler, more general, lower-risk** cause
— the squat pose's CoM sits slightly forward of the foot, fixed by a **backward ankle trim** that
works identically across G1, H1, and Valkyrie. The centering approach is the one to reach for
first. This doc records both, why centering won, and the reusable engineering rules.

Companion: [humanoid-deterministic-stand.md](humanoid-deterministic-stand.md) (the generic
`humanoid_stand_deploy` harness + the centering result). Status of record on the G1 stand:
[rl-current-state.md](rl-current-state.md).

---

## The symptom

A deterministic stiff-position-hold of the G1's statically-stable squat (no RL) **settles upright
then drifts forward and tips at ~1.3–1.8 s** under the deploy solver (Newton `SolverMuJoCo` +
`mujoco_warp`). Plain MuJoCo at the same pose/model stands. This produced a long multi-session
chase (see the SUPERSEDED chain in the `project_g1_standwave` memory) through XPBD-vs-mjwarp,
contact-point culling, `mjw.step` vs `SolverMuJoCo.step`, etc. — all dead ends.

## Two fixes for the same symptom

### Approach A — engine fix (this session): composite fixed-joint inertia
Diagnosis: OmniSim's deploy merges fixed-jointed child links into their parent body but **summed
the child MASS while keeping only the LEADER link's inertia tensor + CoM** (lossy). For the G1
this put the whole-body CoM **+1.85 cm off in Y**. Fix: `gatherFixedSolids()` +
`rolledUpComInertia()` in [`src/omnisim/nodes/OmSolid.cpp`](../../src/omnisim/nodes/OmSolid.cpp)
compose the true mass + mass-weighted CoM + **parallel-axis** inertia, gated behind
`OMNISIM_NEWTON_COMPOSITE_INERTIA=1` (default OFF). With it on (KE 800, pure pose) the G1 stood
1364 s, 0 falls.

### Approach B — pose centering (other session): backward ankle trim
Diagnosis: the squat pose's CoM is slightly **forward** of the foot center; the deploy needs an
ankle trim to center it. Fix: a per-robot `ank_bias` in a JSON spec driving the generic
`humanoid_stand_deploy` controller. At `ank_bias -0.06` (KE 400) the G1 stands 0-falls 400 s+ and
stand+waves cleanly. **No engine change, no rebuild.** A controlled bias sweep showed H1 and
Valkyrie have the *identical* forward-CoM failure at bias 0, fixed by the *same* −0.06 centering
(basin ≈ [−0.05, −0.07]).

## Why approach B is the better engineering — and the right default

1. **It is the simplest sufficient cause.** The forward tip is a forward-CoM problem; an ankle
   trim addresses it directly. The bias sweep is a clean controlled experiment that *proves* the
   cause (bias 0 tips, −0.06 stands, −0.09 tips back).
2. **It generalizes.** One spec-driven controller stands G1 **and** H1 **and** Valkyrie with the
   same method and the same ~−0.06 center. That cross-robot regularity is itself evidence the
   cause is general (pose centering), not a per-robot engine defect.
3. **It is lower risk.** A JSON `ank_bias` is config. The inertia rollup is a shared physics
   primitive on *every* robot's model build — bigger blast radius, needs a rebuild and a gated
   flag to stay safe.
4. **The engine fix targeted the wrong axis.** The rollup error I fixed is **lateral (Y)**; the
   failure was **forward (pitch, X)**. The Y-inertia bug is real, but it was *not* the decisive
   lever for the forward tip — the stiff KE 800 + (incidentally) a less-forward CoM did the work.
   I never ran the controlled A/B (composite ON vs OFF at the *same* KE/bias) that would have
   exposed this. The other session's bias sweep did the experiment I skipped.

## Is the engine fix worthless? No — but it is secondary

The lossy fixed-joint inertia rollup **is a genuine bug**: it builds a model whose mass
distribution is wrong (CoM +1.85 cm off in Y, wrong inertia tensor). That matters for **dynamic**
fidelity — walking, manipulation, throwing — where inertia/CoM actually bite, independent of
standing. So the gated `OMNISIM_NEWTON_COMPOSITE_INERTIA` fix is kept (default OFF) as a *real but
secondary* fidelity improvement, **not** as the G1-stand solution. Before flipping it on by
default it needs a controlled test that it improves a *dynamic* task without regressing the
walk/manipulation deploys that were tuned against the old rollup.

## Reusable rules (the actual lesson)

1. **Find the simplest sufficient cause before reaching for the heaviest fix.** A config/pose
   compensation that makes the symptom go away — and *generalizes* — beats a "correct" engine
   change that is heavier and aimed at a secondary defect.
2. **Do the controlled A/B before attributing a fix to a mechanism.** Toggle exactly one variable.
   "I changed X and it worked" is not "X was the cause." A bias sweep / an ON-vs-OFF flag at fixed
   everything-else is worth more than a plausible mechanism story.
3. **Prefer the lowest-risk lever: pose/config → controller → engine.** Engine edits touch every
   consumer; keep them gated and default-off until a controlled test earns the default.
4. **Cross-robot regularity is a diagnosis.** When three different robots fail the same way and
   are fixed by the same knob, the cause is general (here: pose centering), not per-robot.
5. **A genuinely-correct fix can still be the wrong tool.** Correct ≠ decisive ≠ worth the risk
   *for this goal*.

## Demo-specific gotchas found along the way (G1 stand+wave)

- The forward CoM margin is **razor-thin**: any arm motion with a forward component (a bent elbow
  or forward shoulder-pitch) tips it. A wave must stay in the **frontal plane**, *or* the pose
  must be centered with a back-lean to buy forward margin first (approach B's `ank_bias` does
  exactly this — which is why its bent-elbow wave works and a pure-pose frontal bob is all the
  uncentered model can do).
- The "whippy / disconnected hands" arm artifact is **wave SPEED, not stiffness.** The global
  `OMNISIM_NEWTON_TARGET_KE` (leg stiffness) on the low-torque arm hinges (25 N·m) saturates only
  when the target moves fast; a slow (~0.3 Hz) quasi-static wave tracks smoothly even at KE 800.
- **Softening the arm to fix the whip backfires:** any arm KE < 800 lets the arm lag/swing during
  the raise, and that reaction tips the thin-margin stand. Keep the arm stiff; slow the motion.
- A gated arm-hinge stiffness scope (`OMNISIM_NEWTON_ARM_KE`/`ARM_KD`, effort-thresholded so it
  hits arms but not ankles/legs) was added in
  [`OmNewtonBackend.cpp`](../../src/omnisim/physics/OmNewtonBackend.cpp) for cases where a *soft*
  arm is wanted — but per the point above it is **not** the fix for the wave whip.
- A rebuild **invalidates the warp kernel cache** → the first 1–2 runs after a build are cold GPU
  compiles (~30–60 s) that can eat a short `--duration`; and `--gui` needs `--batch` (without it
  OmniSim runs an interactive stepping path that tips the robot).
