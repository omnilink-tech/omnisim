# Deterministic MPC push-recovery for the G1 — literature synthesis + what it tells us

Research (web, June 2026) into deterministic (NO-RL) push-recovery stepping, to push the
in-engine MPC (`g1_step_mpc.py`) past its wall. MPC is the right vehicle: it plans by
rolling out in the deploy's OWN engine → **zero sim-to-deploy obs gap** (unlike RL).

## The three balance strategies are ONE thing: move the ground-reference with more of the body
(Pratt Humanoids'06; Stephens Humanoids'07; Kim CP-MPC arXiv:2307.13243)
- LIP: `ẍ = ω²(x − p)`, `ω = √(g/z₀)`. Capture point / DCM: `ξ = x + ẋ/ω`, `ξ̇ = ω(ξ − p)`.
- **Ankle**: move CoP `p` WITHIN the foot. Works iff `δ⁻ < ξ < δ⁺` (capture point inside foot).
- **Hip** (flywheel / centroidal angular momentum): adds CMP excursion `τ/(mg)` past the foot
  edge. Enlarged region: `δ± ± (τ_max/mg)(e^{ωT_max}−1)²`. Bounded by torque + joint angle.
- **Step**: relocate the support polygon to put `ξ` back inside; step to put the post-impact
  state on the stable eigenvector `ẋ = −ωx`.
- **Switch top-down**: ankle → (if ξ leaves foot) hip → (if ξ leaves the hip-enlarged band) step.
- A single QP makes all three emergent: decision vars = CoP `z`, centroidal torque `τ`, footstep
  `ΔF` (and step timing), each penalized + box-constrained → the optimizer escalates as each
  constraint binds. Kim numbers: τ_max ±7 N·m (real), ZMP box x∈(−.09,.12)/y±.07, step Δx±.2/
  Δy(−.1,.03), wξ 5–100, wτ swept→0 (variable CAM damping), wf 1000.

## Khadiv step-TIMING adaptation (T-RO 2020, arXiv:1704.01271) — the targeted fix
Step duration enters via `e^{ωT}` (nonlinear). The trick: substitute **τ = e^{ωT}** → the DCM
step map is LINEAR → tiny convex QP picks step location + timing together:
```
min_{u,τ,b}  α₁‖u−u_nom‖² + α₂(τ−τ_nom)² + α₃‖b−b_nom‖²
s.t.  u + b = (ξ_meas − u₀)e^{−ωt}·τ + u₀       (DCM dynamics, linear in τ)
      u_min ≤ u−u₀ ≤ u_max                      (reach box)
      e^{ωT_min} ≤ τ ≤ e^{ωT_max}               (timing bounds)
      b ≤ b_max = L_max/(e^{ωT_min}−1)          (∞-step capturability / viability)
```
Mechanism: **hard push → reduce τ → step SOONER** so `u` stays within reach. Numbers:
T_nom≈0.5s, T_min .3 / T_max .7, lateral half-width .28m, step box ±.2/±.1m.

## Lateral is harder than sagittal (Missura/Behnke; IHMC Atlas arXiv:1703.00477)
- Sagittal = divergent (`e^{ωT}−1`): step to the capture point, done.
- Lateral = oscillatory (`e^{ωT}+1`, alternating sign): you must TIME the support exchange to
  the lateral oscillation; swing-speed-up (timing) is "not effective perpendicular" → lateral
  needs step-location + CMP + angular momentum together. **Side-step wider, NEVER crossover.**
  **Pre-unload the fall-side foot** (drive contact force→0) before releasing it.

## Sampling-MPC: recovery + get-up EMERGE, no footstep planner (MuJoCo MPC arXiv:2212.00541; DIAL-MPC 2409.15610)
THE key finding for our in-engine sampler. Predictive Sampling / MPPI sample **time-varying
spline controls** (joint targets/torques) and roll out in the engine; under a balance cost the
DeepMind 27-DoF humanoid, tasked ONLY with "stand," recovers large pushes AND stands up off
the floor — **no footstep variable, no capture-point cost**. Recovery is emergent from:
cost = CoM-between-feet + CoM-velocity + head/pelvis height + posture-to-nominal + effort.
Predictive Sampling = MPPI at ∞ temperature (pick argmin of N spline samples). DIAL-MPC:
Nsample 2048, Hsample 25 (~0.5s), Hnode 5 spline knots, temp 0.05, 50Hz, torque-level,
diffusion-annealed covariance. Horizon ~1.0s is what lets a STEP (a 0.3s time-varying motion)
fit in the rollout.

## What our tests proved (g1_step_mpc, in-engine on the deploying stand baseline)
- **Adaptive step-timing IMPLEMENTED + works geometrically**: hard push → T→T_min, weight-shift
  skipped, foot lands AHEAD of the CP (e.g. push 1.0: foot 0.43m fwd vs CP 0.31). Foot placement
  is no longer the problem.
- **But it still falls — body ROTATION, not foot placement, is the binding wall**: `pitch→−1.5`
  (the CoM rotates forward over the foot). Capture-point theory is point-mass; the real G1's body
  pitches over and nothing arrests it fast enough.
- **Stepping triggers too eagerly and HURTS**: at push 0.6 (recoverable in-place) the step's
  single-support destabilises what ankle/hip would have caught → falls. Confirms the research's
  ankle→hip→step gating: only step when ξ leaves the ankle+hip-enlarged region (margin ≈ 0.25–0.35,
  not 0.12). The constant-residual + short-horizon MPPI cannot do the coordinated body-arrest.

## UPDATE — segmented spline-MPPI (the MuJoCo-MPC recipe) BUILT + tested
Implemented `_plan_seg`: each sample is NSEG piecewise-constant residual segments held HSEG
ticks, rolled out over a ~0.5s horizon in-engine, graphed via a per-segment ctrl buffer
(NSEG copies + steps -> stays fast, ~0.21x). world._smpc_nom is (NSEG,n); apply segment 0,
warm-shift (receding horizon). Gated by SMPC_NSEG>1 (FSM auto-disabled -> emergent stepping).
- **Stands cleanly UNDISTURBED**: residual ~0 (|res0|~0.02, J~0.1) -- the emergent balance
  + WRES penalty correctly do nothing, the deterministic base holds. Graph + speed good.
- **FAILS under push (0.8 and 1.0 fwd)**: the residual SATURATES (maxres at cap on all dofs,
  J~430) and the robot FLAILS + pitches over (~4.2s) -- WORSE than the base's in-place
  recovery. Three root causes found:
  1. **Sampling scale**: K=48, 0.5s horizon is far below MuJoCo-MPC's K=hundreds-thousands +
     1s. The recovery-step basin is thin; the sampler can't find it -> averages/picks falling
     samples -> flail. MuJoCo-MPC-scale sampling isn't feasible in-engine on this GPU at usable
     speed.
  2. **Rollout doesn't model the base's REACTIVITY**: the rollout holds the controller's
     command CONSTANT over the horizon, but the deployed base reactively leans/arm-balances
     each tick. So the rollout shows the robot falling with residual=0 (when really the base
     would recover) -> the MPPI over-acts to "save" it -> saturates -> overrides + breaks the
     base's working in-place recovery.
  3. **Body-rotation wall + position-residual-on-servo**: a big position-target residual yanks
     the joints (not smooth torque); and the G1 torso still pitches over for hard pushes.

## Conclusion + the two research-grounded paths that remain
The G1's deterministic push-recovery is bounded by **body-rotation arrest (ankle + hip/angular-
momentum authority)**, not foot placement. Adaptive-timing stepping (now implemented) fixes the
geometry but doesn't extend the limit, because the step's single-support lets the body rotate.
The literature's two ways past this — both deploy in-engine (no obs gap), both substantial:
1. **Unified ankle+hip+step CP-MPC** (Kim 2023): one QP carrying CoP + centroidal torque +
   footstep + timing, each box-constrained, so the body-arrest (ankle CoP + hip CAM) and the
   step are optimized TOGETHER. Needs the centroidal-momentum task + the QP (port TSID-style).
2. **Spline-MPPI emergent recovery** (MuJoCo MPC recipe): replace the constant residual with a
   ~5-knot time-varying spline over a ~1s horizon + the balance cost (CoM-in-support + CoM-vel +
   height + posture + effort) + Predictive-Sampling argmin. Recovery/stepping emerge; reuses the
   in-engine rollout (no obs gap). Breaks the constant-ctrl CUDA graph (needs per-step ctrl).
