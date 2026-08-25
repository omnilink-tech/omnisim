# OmniArm 6 Toss-to-Place — Shadowing on a Manipulator

> **Status:** working end-to-end in OmniSim Newton (committed on `feat/omniarm6-toss-shadowing`).
> The OmniArm 6 arm picks a cube and **throws** it into a bin **beyond its kinematic reach** — a
> placement no quasi-static IK/carry controller can do. This is [Shadowing](shadowing.md)
> (generate → verify → deploy) applied to a non-legged robot + a dynamic manipulation task.

## What it does

A bolted-down OmniArm 6 (6-DOF, ~1.0 m reach) cannot *place* an object past its workspace — that's
geometry. It **can** put one there by **throwing** it. The demo: vacuum-pick a cube, execute a
feasibility-certified swing (the "ghost"), release the vacuum at the right instant, and the cube
flies on a ballistic arc into a bin at **1.3 m** (lands ~1.5 cm from centre).

```
[toss] RELEASE @ tau=0.98s  TCP=(-0.160,1.231)  |v|=2.83 m/s  angle=+10.8 deg  proj_x=1.34
[toss] cube final pos = (1.315, 0.005, 0.045)
[toss] landed 1.5 cm from bin centre (x=1.30)  -> IN BIN   RESULT: PASS
```

## How to run

```powershell
# 3D GUI (watch the throw):
.\scripts\dev\run_omniarm6_toss_demo.ps1

# headless verify (auto-quits after one throw, writes _toss_result.txt):
.\scripts\dev\run_omniarm6_toss_demo.ps1 -Headless
```

Regenerate the ghost / re-certify (optional — the demo runs from the committed `.npz`):

```powershell
python scripts/dev/make_omniarm6_mjcf.py --verify          # build the self-limiting MJCF
python projects/policies/research/shadowing/generate_omniarm6_toss.py --bin-x 1.3
python projects/policies/research/shadowing/ghost_verifier.py `
  --npz projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz `
  --mjcf projects/robots/omnisim/omniarm6/mjcf/omniarm6_throw.mjcf.xml
python projects/policies/research/shadowing/preview_omniarm6_toss.py      # 2D side-view preview PNG
```

## Architecture mapping (and where each stage degenerated)

| Shadowing stage | On OmniArm 6 toss | Heavy form (G1/OmniQuad) |
|---|---|---|
| **Generate** | designed windup→whip→follow swing played through a **self-limiting model**; release-instant selection for aiming | trajectory optimization (MPC/contact-implicit TO) |
| **Verify** | `verify_arm`: velocity ratio, torque ratio (inverse-dyn), tracking drift, **reaches-the-bin** | ZMP/balance + torque certificate |
| **Deploy/Track** | **direct position control** of the ghost `q[t]` + adaptive release — **no RL** | RL policy under domain randomization |

A fully-actuated, fixed arm tracks a feasible joint trajectory directly, so the **tracker collapsed
to classical control** and the **generator collapsed to designed-playback** (MPPI went chaotic on a
fast fling — the doc's own warning). The piece that genuinely earned its keep vs. "just script it"
(as the [bin-pick](omniarm6-suction-bin-pick-journey.md) does) is the **feasibility layer** — see below.

## Key findings

### 1. Velocity, not torque, is the binding feasibility constraint
For a light payload (~0.2 kg) the weak distal joints (J5/J6 = 34 N·m) spin a part **~38× past their
rated speed** under a pure torque clamp. So a feasible throw is **velocity-limited**, powered by the
strong proximal joints (J1/J2 = 194 N·m). Feasibility therefore has to live in the **model**:
`make_omniarm6_mjcf.py` emits a fixed-base MJCF whose actuators carry `forcerange = URDF effort` **and**
a motor-curve joint damping `b = effort / vel_limit` (net torque → 0 at the rated speed). The swing
played through it is feasible w.r.t. **both** torque and velocity *by construction* — no penalty to tune.

### 2. MPPI is the wrong generator for a fast fling
Receding-horizon predictive sampling went chaotic at the swing's stiff-contact / high-speed knife
edge (it also underperformed hand-tuning). A **deterministic designed swing + feasibility-limited
playback** is reliable and is a legitimate trajectory generator for this task. Aiming is by
**release-instant selection**: one swing sweeps a family of release states; pick the instant whose
ballistic projection hits the bin. The set of reachable landings **is** the feasibility frontier.

### 3. The verifier is the real win over scripting
`verify_arm` certifies, *before deploy*: velocity margin, torque margin (inverse dynamics on a
damping-zeroed "true" arm), open-loop tracking drift, **and** that the throw reaches the target bin.
- Feasible **1.3 m** bin → **PASS** (velocity 0.97, torque 0.54).
- Too-far **1.8 m** bin → **FAIL** (lands 41.9 cm short = task-infeasible).

A scripted throw skips this and finds out by trial-and-error in sim. (NB: re-sim must use the ghost's
`sim_dt`, saved in the `.npz`, or the open-loop replay falsely diverges from integrator mismatch.)

## Deploy: two sim-to-deploy gaps (a certified ghost does NOT trivially deploy open-loop)

Even on a fully-actuated arm, a *dynamic* motion exposed two real gaps:

### Gap 1 — motor config (env-only)
OmniArm 6's default Newton motors are tuned for **slow precise picking** (`target_ke=20` soft,
`target_kd=500` heavy damping; a mis-applied effort cap throttled 194 N·m → ~30). A fast swing barely
moved (~0.7 rad/s). Fix (in `run_omniarm6_toss_demo.ps1`, **env only** — default picking gains untouched):
`OMNISIM_NEWTON_TARGET_KE=8000`, `TARGET_KD=30`, `NO_EFFORT_LIMIT=1`. The arm then reproduces the
swing at **2.83 m/s** (vs the ghost's 2.88). Notes: Newton ignores velocity-control mode
(`setPosition(inf)+setVelocity`); a fixed time-**lead** made the arm arrive early and *decelerate*
(killing release velocity), so **lead = 0** + stiff KE matches the through-release velocity best.

### Gap 2 — free-body `setVelocity` was a no-op under Newton (engine fix)
The cube is held by the kinematic suction teleport, so it must be **given** the arm's measured
release velocity. A Supervisor `node.setVelocity` on a free body did nothing — the cube fell straight
down. Root cause: `OmNewtonBackend.set_body_vel` wrote `body_qd`, but a **free body integrates from
`joint_qd`** and `eval_fk` overwrites `body_qd` from it every step. Fix (mirrors the `reset_body_pose`
free-body fix): also write the body's **free-joint `joint_qd`**. Newton free-joint layout is
`[linear(3), angular(3)]` world-frame (`spatial.py` twist = `(v, ω)`; `example_robot_policy` reads
`root_lin_vel_w = joint_qd[:3]`). Now a Supervisor `setVelocity` works for **any** free body — a
general "throw / impart velocity" capability, not just for this demo. **Requires a `build_omni.bat`
rebuild** (the backend's Python is embedded in the `.cpp`).

### Aiming — adaptive release
The deploy arm tracks the ghost imperfectly, so the controller doesn't blindly release at the ghost's
step. It tracks the arm's **live** TCP velocity (FK finite-diff) and releases at the instant its
ballistic projection reaches the bin — robust to the residual tracking gap. No RL needed.

## So what did Shadowing actually buy here? (the honest take)

For the **bin-pick** (quasi-static), "just design the motion" works because **feasibility is free** —
any reachable IK pose is achievable. For the **throw** (dynamic), feasibility is **not** free: a
hand-designed swing can be physically impossible or simply miss. The pipeline's contribution is making
feasibility a **certified, model-enforced property** (self-limiting model + verifier) rather than a
hope — and the verifier gives the **frontier** (which bins are reachable) before you run the robot.

What it did **not** buy: because the arm is fully actuated, the *heavy* components (TO generator, RL
tracker) degenerated to designed-playback + classical control. So on this task **Shadowing ≈ designed
intent + feasibility certificate + classical tracking**. The toss is the *easy end* of the spectrum —
its worth is mainly as a **generality** result (same pipeline, non-legged robot, manipulation). The
full architecture proves indispensable when the motion is **not intuitable** (needs the optimizer to
discover it) and **can't be tracked open-loop** (underactuated / contact-rich) — e.g. G1/OmniQuad, or a
two-arm **throw-and-catch** (the catch is a genuine feedback problem).

## File map

| File | Role |
|---|---|
| `scripts/dev/make_omniarm6_mjcf.py` | self-limiting fixed-base OmniArm 6 MJCF (gitignored output, regenerable) |
| `projects/policies/research/shadowing/generate_omniarm6_toss.py` | ghost generator (designed swing + release-instant aim) |
| `projects/policies/research/shadowing/ghost_verifier.py` | `verify_arm` fixed-base feasibility certificate |
| `projects/policies/research/shadowing/ghost_generator.py` | fixed-base detect + `task`/`vel_limits`/`active_joints` hooks (G1 unchanged) |
| `projects/policies/research/shadowing/preview_omniarm6_toss.py` | 2D side-view preview (stroboscopic swing + arc + frontier) |
| `projects/policies/research/shadowing/ghosts/omniarm6_toss_ghost.npz` | the certified ghost (deploy loads this) |
| `projects/samples/demos/worlds/flagship/omniarm6_toss_demo.omniworld` | the world (OmniArm 6 suction + pedestal + cube + bin) |
| `projects/samples/demos/controllers/omniarm6_toss_deploy/omniarm6_toss_deploy.py` | deploy controller |
| `scripts/dev/run_omniarm6_toss_demo.ps1` | launcher (bakes the working motor config) |
| `src/omnisim/physics/OmNewtonBackend.cpp` | engine fix: free-body `setVelocity` writes `joint_qd` |

## Related
- [shadowing.md](shadowing.md) — the method + paper scaffold (this is an E3 generality result).
- [omniarm6-suction-bin-pick-journey.md](omniarm6-suction-bin-pick-journey.md) — the quasi-static contrast.
- [real-grasp-and-the-cold-first-load-trap.md](real-grasp-and-the-cold-first-load-trap.md) — `warmup_reload`.
