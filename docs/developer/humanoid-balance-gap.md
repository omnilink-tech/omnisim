# Humanoid balance gap (the bipedal blocker)

> **✅ SUPERSEDED 2026-06-10: the G1 deploy stand was SOLVED.** The RL-balance
> "deploy stands to ~1.55 s then falls" analysis below is **pre-fix and historical**.
> The deploy stand now holds **indefinitely** via a deterministic *pure pose* (deeper-squat
> NOMINAL that recenters the CoM + ankle PD off), not this RL policy (commit `f48f00b7`).
> See the canonical [`rl-current-state.md`](rl-current-state.md). The text below is kept
> for the RL-balance journey and the deploy-solver-divergence lesson.
>
> **⚠️ HISTORICAL STATUS (2026-05-29): IN-SIM SOLVED, DEPLOY PARTIAL.** Heavy domain
> randomization solved G1 standing *in the mujoco_warp trainer* (≈98 % survival),
> and as of 2026-05-28 the OmniSim Newton deploy stood 44+ s. A 2026-05-29 Newton
> floor-contact change (`d56cbf5`) shifted the deploy contact dynamics: at that time the
> **deploy stood cleanly to t ≈ 1.55 s, then lost balance** (with
> `OMNISIM_NEWTON_STATICS=1 OMNISIM_NEWTON_SUBSTEPS=4`) — since superseded by the
> 2026-06-10 pure-pose fix. More-DR retrain and
> ground-friction sweeps are both empirically ruled out; the residual is the
> structural `mjw.step ≠ SolverMuJoCo.step` divergence **plus inherent biped
> instability** — a passive NOMINAL-hold topples ~1.5 s on the *exact* deploy
> solver. Path forward: train-in-the-deploy-solver (foundation built in
> [`build_g1_native.py`](../../projects/policies/research/training/build_g1_native.py); the
> trainer itself is not yet built).
>
> **Canonical, always-current status:** [`rl-current-state.md`](rl-current-state.md).
> Journey + recipe: [`g1-stand-rl-playbook.md`](g1-stand-rl-playbook.md). General
> recipe: [`sim-to-deploy-rl-recipe.md`](sim-to-deploy-rl-recipe.md). Trained ONNX
> at `projects/policies/research/training/runs/gpu_g1_stand_robust/policy.onnx`.
>
> The "build a better analytical/LIPM baseline" conclusion in the body below was
> **wrong** (heavy DR was simpler) and is kept only as the historical
> why-it-wasn't-trivial analysis. Atlas: the same pipeline ports, but PPO
> converges to the analytic baseline with no learned gain — see
> [`atlas-stand-rl-journey.md`](atlas-stand-rl-journey.md).
>
> **🗑️ ATLAS WAS REMOVED FROM THE TREE (2026-07-17).** Every Atlas mention below is
> historical: the robot, its spec, registry entry, controllers and worlds are deleted.
> **`train_robot.py --robot atlas` no longer resolves** and `atlas_stand_newton.wbt` no
> longer exists — ignore those instructions. Only the G1 half of this document describes
> a robot that still exists. See the banner in [`rl-current-state.md`](rl-current-state.md).

The analysis below stands as the *why-it-wasn't-trivial* explanation for
why the OmniQuad residual recipe doesn't port to bipeds out of the box — but
the conclusion that bipeds need a "real analytical baseline" turned out to
be wrong. Heavy DR plus pure RL was simpler and more effective.

---

State as of 2026-05-27: both registered bipedal humanoids (Atlas, Unitree G1)
have working URDF import, spec, and tooling, but **neither can stand under
the current analytical control stack.** This is the same gap that
blocks the model+residual recipe from porting from OmniQuad to humanoids.

This doc captures the exact shape of the gap so the next session can
pick it up cleanly without re-deriving it.

## The recipe assumption that breaks for humanoids

The model+residual RL recipe ([omniquad-residual-rl.md](omniquad-residual-rl.md))
assumes the analytical layer **almost walks at zero policy.** For OmniQuad,
that's true: gait engine + IK + balance PD walks straight 5 m in 30 s
without any neural network. The policy adds ~5% refinement.

For OmniQuad this is trivial because trot has 2 feet on the ground at all
times — the support polygon is wide and the CoM stays inside it without
active management. **For bipedals, no analytical pose alone produces
a stable stand**, let alone walking, because:

1. **Single-support phases** — during each step, only one foot is on the
   ground. The support polygon shrinks to the area of that one foot.
2. **Inverted pendulum dynamics** — CoM is at ~1 m above ankle joints
   on a base ~10×25 cm wide. Tiny tilts grow exponentially fast (time
   constant `sqrt(L/g) ≈ 0.32 s` for Atlas/G1-class robots).
3. **Static PD is fundamentally insufficient** — the joint controller
   can hold a *commanded* joint angle, but it has no information about
   pelvis-vs-foot horizontal position, which is what determines tipping.

Empirically, both Atlas and G1 tip forward in 0.3-0.9 s under every
nominal-pose + balance-gain combination we tried. Same failure regardless
of:
- nominal pose (forward squat / flipped hip / straight legs / deep squat)
- TARGET_KE (500 - 2000)
- ankle pitch/roll PD gains (kp 1.5 - 8.0)

## The G1-specific bug that's NOT the cause (but is worth knowing)

While diagnosing G1 stand we found a real Newton-side bug:

  **Newton's URDF importer picks up only the FIRST `<collision>` shape
  per `<link>`.** G1's URDF defines each foot as 4 corner spheres at
  `(±0.05/+0.12, ±0.025/±0.03, -0.03)` forming a 17×6 cm support
  polygon. The importer used only the first sphere — so G1 was
  balancing on a single 5 mm point contact.

Fixed by [`projects/policies/research/tools/patch_g1_urdf_for_rl.py`](../../projects/policies/research/tools/patch_g1_urdf_for_rl.py)
which rewrites the 4 spheres as one bounding box. **The standing
problem persists even with the proper foot box** — confirming that
the issue isn't contact geometry, it's the missing balance control
layer.

(OmniQuad doesn't hit this Newton bug because its URDF defines feet as
single spheres — and it doesn't need a wider polygon anyway because
it has 4 feet.)

## What the next session needs to build

A simple LIPM (Linear Inverted Pendulum Model) / capture-point balance
controller. The full algorithm is well-known and tractable:

```python
# Once per tick, in the controller:

# 1. Estimate state
pelvis_xy = self_node.getPosition()[:2]
pelvis_v_xy = self_node.getVelocity()[:2]
z = pelvis_xy[2]

# 2. Compute capture point — the point on the ground where, if the
#    robot's swing foot lands, the body's momentum will be exactly
#    canceled and it stops.
T = sqrt(z / 9.81)          # natural pendulum time constant
cp_x = pelvis_xy[0] + pelvis_v_xy[0] * T
cp_y = pelvis_xy[1] + pelvis_v_xy[1] * T

# 3. Decide policy
support_polygon = current_foot_corners()
if (cp_x, cp_y) is inside support_polygon:
    # Stable — just hold pose with mild ankle PD
    ankle_command = static_pose_with_pd(pelvis_pitch, pelvis_roll)
else:
    # Falling — step toward CP
    swing_foot_target = (cp_x, cp_y)
    transition_to_single_support()
```

Reference reading:
- Kajita et al., "Biped walking pattern generation by using preview
  control of zero-moment point" (ICRA 2003) — original LIPM paper.
- Pratt et al., "Capture Point: A Step toward Humanoid Push Recovery"
  (Humanoids 2006) — the capture-point formalism we'd implement.
- IHMC's open-source Java humanoid stack has reference implementations
  if a full port is wanted later.

## Effort estimate

- LIPM-based **standing only** (no stepping yet) — ~1 day. Closes a
  feedback loop from pelvis lin-vel to ankle pitch/roll torque using
  the capture-point formula. Enough to stand still and recover from
  small pushes. Once this works, the model+residual recipe slots in
  cleanly because the analytical baseline is "almost right" — exactly
  OmniQuad's regime.

- **Walking on top of standing** — another 1-2 days. Need a state
  machine (double-support / left-swing / right-swing), foot-step
  planning (place swing foot at the capture point), and trajectory
  generation between footsteps.

- **PPO residual on top of walking** — should be ~1 hour of plumbing
  once the analytical stack works, mirroring the OmniQuad residual stack.

## Status by humanoid

| robot | URDF | Spec | Smoke world | Stand | Walk |
|---|---|---|---|---|---|
| Atlas | ✓ | ✓ | atlas_stand_newton.wbt | ✗ (active balance needed) | ✗ |
| G1 (23-DOF) | ✓ (foot patched) | ✓ | g1_smoke.omniworld, g1_model_walk_newton_demo.omniworld, g1_pose_sweep_newton.omniworld | ✗ (active balance needed) | ✗ |

Both robots are first-class in the registry. `train_robot.py --robot atlas`
and `--robot g1` will dispatch correctly once the analytical baseline
exists.

## What's NOT the blocker (worth being explicit)

- Not the URDF — both load cleanly, all joints registered with correct
  effort/velocity/range limits.
- Not the OmniSim binary — built with `OMNISIM_WITH_NEWTON=ON`, Newton
  is loading, the joint-limit clamp from commit `00280fe0` is enforcing
  URDF limits.
- Not the Newton physics — the body responds to commands, joints
  enforce limits, contacts work (verified in the joint-CSV diagnostic
  for OmniQuad's push-recovery experiment).
- Not the foot geometry (for G1) — the patch_g1_urdf_for_rl tool
  replaces the broken 4-sphere collision with a proper box, verified
  to load.

The single missing piece is the LIPM/capture-point controller that
sits between the gait/CPG layer and the joint commands. Once it's
in place, everything downstream (residual RL training, deploy worlds,
analyze_joints tooling) already exists and should work directly.
