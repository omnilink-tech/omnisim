# G1 improved shadow — making "match the shadow" an achievable goal

> 📍 **Canonical G1 status:** [rl-current-state.md](rl-current-state.md) (the *G1 — detail*
> section) is the single source of truth. If a status claim here disagrees with that file,
> that file is right — fix this one.

**Status:** shipped (the gait-model upgrades + a standalone OmniSim demo).
**Date:** 2026-06-15.

> ⚠️ **Update 2026-06-18 — (B) "achieved" is the WINNING reference for *shape*, rebuilt for
> Newton, but it does not (yet) give a durable walk.** `build_achieved_gait.py` extracts in
> mjwarp; the deploy is Newton, so the Newton trainer's new `--build-achieved` re-extracts the
> feasible ghost **in the Newton/SolverMuJoCo solver** (from `ft_pdoff_clamp`) →
> `datasets/g1_achieved_gait.npz`. A fresh policy tracking it (`--gait-style achieved`) measures
> **FAIR all-13 84 % / moving 88 % over a 3 s window** vs the ghost — vs the human ghost's ~67 %
> physical wall (the feasible reference collapses the tracking error RMSE 0.17→0.05 rad). **BUT an
> 18 s eval shows it topples ~6–8 s:** the achieved ghost was extracted in the trainer env where
> `ft_pdoff_clamp` survives only ~7.3 s (it walks 33.8 s in *deploy* — the byte-matched model's
> per-tick drift compounds on the unstable biped), so the reference and policy inherit ~7 s
> durability. So this confirms the doc's own thesis — a *feasible* shape reference makes the
> shape match achievable — but **durability is a separate, unsolved problem** (the G1
> trainer↔deploy gap). Full journal: [g1-ghost-fidelity-journey.md](g1-ghost-fidelity-journey.md).
**Code:** [`projects/policies/control/gait/g1_human_gait.py`](../../projects/policies/control/gait/g1_human_gait.py)
(the modes), [`projects/policies/control/gait/build_achieved_gait.py`](../../projects/policies/control/gait/build_achieved_gait.py)
(builds B's table), [`projects/policies/controllers/g1_ghost/g1_ghost.py`](../../projects/policies/controllers/g1_ghost/g1_ghost.py)
(self-walking ghost), [`projects/policies/research/worlds/g1_shadow_demo.wbt`](../../projects/policies/research/worlds/g1_shadow_demo.wbt),
[`projects/policies/research/runners/run_g1_shadow_demo.ps1`](../../projects/policies/research/runners/run_g1_shadow_demo.ps1).
**Related:** [`g1-deterministic-brain.md`](g1-deterministic-brain.md),
[`g1-mpc-deterministic-brain-research.md`](g1-mpc-deterministic-brain-research.md).

## Why

The "shadow" is the kinematic, physics-free gait model the RL robot is trained to
imitate (`g1_human_gait.py`, played by the ghost). The measured shadow-tracking gap on
the shipped walker (`gpu_g1_walk26_shape_c8`) was **7.2° hip+knee but 13.4° all-leg** —
and the *entire* remaining gap lives in the **frontal/transverse plane**: hip-roll
21.7/36.3°, hip-yaw 20.2/22.0°. The legacy shadow has a ~flat frontal plane (a 3° `sway`,
zero hip-yaw), so it tells the robot to keep its pelvis centred while balance *requires*
the CoM to travel over the stance foot. That contradiction is why the robot **splays**
(the "drunk gait"). Matching the legacy shadow's hip-roll≈0 literally means "stop
balancing → fall" — a **fantasy target**.

The fix here: improve the *target* so it has a principled, **achievable** lateral plane
(and the previously-missing hip-yaw). Three approaches, all shipped as composable modes.

## The three shadows (all in `g1_human_gait.py`, numpy + torch, parity <1.5e-7)

Each is gated behind `GaitParams` fields; defaults are unchanged so existing
trainers/policies are unaffected. The full self-test (`python g1_human_gait.py`) covers
all three (parity, joint limits, symmetry, and the specific feature of each).

| mode | how to select | what it adds | measured |
|---|---|---|---|
| legacy | (default) | `sway` hip-roll sine, no yaw | hip-roll ~3°, yaw 0 |
| **A — LIPM** | `lateral="lipm"` | Linear-Inverted-Pendulum weight transfer (periodic CoM orbit, ZMP under the stance foot) + **ankle-roll counter-rotation** (flat feet). LIPM sets the *waveform*; `lat_hip_amp` pins the amplitude to physiological pelvic sway. | hip-roll ±5.2°, ankle-roll counter (corr −1.0) |
| **B — achieved** | `style="achieved"` | the robot's OWN recorded walk (`gpu_g1_walk26_shape_c8` — ⛔ note: the long-distance zero-fall figure once attached to that run is **retracted**; the *recording* is still a valid, physically-executed trajectory), phase-binned, L/R-symmetrised, smoothed → a **feasible-by-construction** reference (tracking floor ≈ 0). Built by `build_achieved_gait.py` → `datasets/g1_achieved_gait.npz`. | inherits the recorded **~30° hip-roll splay** (honest: B alone does not fix the drunk gait) |
| **C — human 3D** | `lateral="human"`, `yaw="human"` | measured human normative 3D gait kinematics — hip ab/adduction (frontal) + hip rotation (transverse). The honest mocap-equivalent (averaged human motion). | hip-roll ±6.1°, **hip-yaw ±5.5°** (fills the zero-yaw gap) |

Relevant `GaitParams` knobs: `lateral` ∈ {sway, lipm, human}, `yaw` ∈ {none, human},
`style` ∈ {ik, winter, achieved}, `lat_hip_amp` (A amplitude), `step_width`/`com_height`
(A LIPM shape), `lat_scale`/`yaw_scale`.

## Honest takeaways

- **A** is the cleanest physics (weight transfer falls out of the LIPM + step width).
- **B** proves a *reachable* reference alone doesn't cure the splay — it codifies it. Its
  value is the feasible **sagittal** waveform; pair it with A/C's lateral if used.
- **C** is the most complete (real human 3D, and the only one that fills the 20° hip-yaw
  gap). Recommended starting point for training.
- None removes the trainer(mujoco_warp)↔deploy(Newton) sim gap, but a feasible reference
  *reduces* the deviation the policy needs, making that gap less punishing.

## See it in OmniSim

A standalone, physics-free ghost **self-walks** across the floor playing the pure model
(no RL robot needed; it can't fall):

```
projects/policies/research/runners/run_g1_shadow_demo.ps1 -Mode C   # human 3D (default)
projects/policies/research/runners/run_g1_shadow_demo.ps1 -Mode A   # LIPM weight transfer
projects/policies/research/runners/run_g1_shadow_demo.ps1 -Mode B   # achieved (shows the splay)
projects/policies/research/runners/run_g1_shadow_demo.ps1 -Mode legacy
```

The ghost reads the modes from `G1_GAIT_LATERAL` / `G1_GAIT_YAW` / `G1_GAIT_STYLE` and
self-propels when `G1_GHOST_SELF_WALK=1`. Watch the **frontal plane** — the legs rock
(hip-roll weight transfer) and, for C, turn (hip-yaw). Verified self-walking at 0.4 m/s
with the new lateral/yaw live (hip-roll ±6°, hip-yaw ±5.5°). (Known cosmetic quirk: the
`--gui --realtime` window exits cleanly after ~8 s of sim; headless runs the full
duration. Not chased — the visual is correct.)

Offline matplotlib comparison (curves + front/side stick figures + GIF):
`python _scratch/show_shadows.py`.

## Next step (not done)

Select a shadow (lean **C**) and wire it into the trainer/deploy as selectable flags
(`--gait-lateral/--gait-yaw`, `G1_GAIT_LATERAL/YAW` — the deploy ghost already reads
them), then train a warm-start chunk against it and re-measure the fidelity gap
(`_scratch/measure_fidelity_walk26.py`).
