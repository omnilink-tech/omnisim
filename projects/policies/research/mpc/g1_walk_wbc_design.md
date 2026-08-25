# Deterministic torque-WBC push-recovery stepping for the G1 — design + status

Companion to `g1_walk_wbc.py`. No RL. Goal: a torque-level whole-body controller that
takes recovery STEPS to survive shoves a fixed stance can't, surpassing the
deterministic base. Built as an external in-engine module (full mujoco dynamics, no
C++ rebuild to iterate) loaded by the engine hook
`OMNISIM_INENGINE_PYMOD="projects.policies.research.mpc.g1_walk_wbc:walk_step"`
(+ `OMNISIM_NEWTON_TORQUE_MODE=1`).

## Architecture (the standard, validated one)
- **DCM / capture-point planner** (LIPM): `ξ = c_xy + ċ_xy/ω`, `ω = sqrt(g/h_com)`.
  DCM dynamics `ξ̇ = ω(ξ − u)` (u = CoP); CoM follows stably `ċ = −ω(c − ξ)`.
  - Trigger a step when `ξ` leaves `support_polygon ⊖ margin`.
  - **Step to the PREDICTED CP at touchdown**: `p_foot = u_stance + (ξ − u_stance)·e^{ω·t_remain}` (the #1 fix — don't step to where the CP *is*, step to where it *will be*).
  - Reach-clamp `p_foot`; if beyond reach, step anyway + re-trigger → **N-step** recovery converges.
  - DCM-tracking CoP law: `u_des = ξ + (1/ω)[K_ξ(ξ − ξ_d) − ξ̇_d]`, `K_ξ > ω`.
  - Optional Khadiv step-timing QP (6 vars: foothold, `τ=e^{ωT}`, offset `b`; weights α=1/5/1000).
- **Whole-body QP / TSID** (torque): decision `y = [q̈ (n+6); f (contacts)]`.
  - **HARD equality: floating-base dynamics** `M_b q̈ + h_b = J_{c,b}^T f` (6 rows). *This is the fix for base drift* — soft base dynamics let the assumed f differ from realized → drift.
  - Hard equality: stance-foot no-accel `J_c q̈ + J̇_c qd = 0`.
  - Inequality: friction cone (`f_z≥0, |f_t|≤μf_z`) + CoP (use **4 corner forces/foot** → CoP inside for free).
  - Cost (tasks, PD `a_des = Kp(x_d−x)+Kd(ẋ_d−ẋ)`): CoM **incl. an explicit height-z task** (Kp_z≈120 — the sinking fix), swing-foot Cartesian, base orientation, angular-momentum (arms/torso = the push absorber, your arm_balance as a WBC task), posture (low weight = regularizer).
  - **Robust solve**: regularized QP, NOT lstsq/SVD on the raw stack (that's the "SVD did not converge" cause). `H = Σ w J^T J + R` (`R = diag(λ_qdd, λ_f)`, λ≈1e-3..1e-2) → PD Hessian. Solve the regularized **KKT** (`[[H, Aᵀ],[A,0]]`) with `np.linalg.solve`, or a real QP (OSQP/PROXQP/quadprog). Recover `τ = M_a q̈ + h_a − J_{c,a}^T f`.
- **Contact-state switching**: DS (both feet) → SS (stance only; swing tracked) → DS.
  **Unload the swing foot** over 50–100 ms (ramp its `f_z,max`→0) before lift-off — don't delete a contact in one tick. Soft contact on touchdown impact.
- Per-tick order: read state → ω, ξ → planner (trigger/leg/foot target/CoP) → build tasks → assemble H,g,A_eq,A_ineq → QP → τ. Filter CoM velocity. Run ≥250 Hz (per-substep) for stepping.

Reuse: `capture_step.py` `capture_point()` (= DCM) + `leg_ik()/leg_fk()` (seed the swing target / landing pose).

## Status (2026-06-29) — foundation built, hit the torque-mode contact wall
- `g1_walk_wbc.py`: infra (joint maps, state I/O, torque write, gravity-on-CPU-model fix, contact-body ids) + the **DS-stand WBC-QP** (regularized KKT, hard base-dynamics equality, CoM+z+orientation+posture tasks, 6D-per-foot wrench, friction projection, τ recovery). The generic engine hook `OMNISIM_INENGINE_PYMOD` is added + built.
- **VERIFIED correct for tick 1**: finite KKT, `Fz` per foot sums to the robot weight (gravity-supported), `|τ|≈6–7` residual, roll/pitch=0. The regularized-KKT formulation works (no SVD blowup).
- **BLOCKER**: it diverges after 1–2 ticks (non-finite) and falls. Root cause = **contact consistency + seed transient**: the WBC's assumed rigid-contact `f` must equal the solver's realized soft-contact `f_real`; the mid-step `live.qpos` squat seed leaves the solver's internal contact/warmstart state stale (the engine rebuilt for straight legs, the seed overwrote qpos to the squat) → `f_real ≠ f` → the joints get the wrong support → divergence. Seed-height sweeps (0.76 penetrates→launch; 0.78 gaps→collapse; 0.776 still diverges) don't fix it; it's a state-consistency issue, not a height issue.

## UPDATE — third wall found: control RATE
Even a robust settle-PID (gravity-comp + joint PD, NO contact-force assumptions)
DIVERGES to NaN when run from the per-tick PYMOD hook (62.5 Hz). The deterministic
position SERVO with the same gains holds — because the engine applies the servo PD
PER-SUBSTEP (250 Hz). Stiff TORQUE control at the 62.5 Hz tick is unstable
(overshoot → oscillation → divergence). So torque-WBC walking needs the controller
to run PER-SUBSTEP (250 Hz), like the embedded TSID's substep path — the generic
PYMOD hook is per-tick and is too slow. (Per-substep also disables the CUDA graph →
slow sim; acceptable for headless dev.) This compounds with the contact-consistency
and seed-transient walls below: all three must be solved together.

## UPDATE 2 — torque-mode standing itself is the wall (multiple compounding bugs)
Trying to get even a torque-mode STAND (the prerequisite) surfaced a chain of
torque-deployment issues, each fixed only to reveal the next:
1. **Rate**: per-tick (62.5 Hz) stiff torque diverges; needs per-substep (250 Hz). [done: OMNISIM_INENGINE_PYMOD_SS]
2. **Gravity-comp double-count**: a settle-PID with `+h` full gravity-comp double-supports (feet already carry the weight) → launch up. Use high-Kp PD without `h` (the servo's torque equivalent). [done]
3. **joint_f sign**: WALK_TAU_SIGN (+1 extends/launch, -1 folds/collapse). [knob added]
4. **Seed state-sync**: writing mjw_data.qpos is OVERWRITTEN on the first step by the Newton joint state (engine seeds STRAIGHT legs in torque mode); must seed model.joint_q + state.joint_q + eval_fk (joint_q index = newton_dof+1). [done] BUT then mjw_data lags the Newton seed by 1 tick AND the pose snaps in one substep → velocity spike → lurch/launch.
5. **Contact consistency**: WBC's assumed rigid-contact f vs the solver's realized soft-contact f (the original divergence).
Net: deterministic torque-mode standing is NOT working in this engine after extensive
effort — the position SERVO (engine-internal, per-substep) sidesteps all of the above,
which is exactly why the deterministic stand + the WBC FF-on-servo (both committed,
working) use the servo, and why pure-torque is the documented hard problem.

**PROMISING PATH (untried): HYBRID per-leg mode.** Don't put the STANCE leg in pure
torque — keep it on the proven position servo (robust hold, no seed/contact/sync
issues). Only the SWING leg needs to be freed (servo to a swing trajectory, or torque)
during its brief flight. This sidesteps the torque-mode standing wall entirely (the
robot always stands on a servo'd stance leg) and reduces the problem to "swing one leg
on the servo to a capture-point foothold, alternate." That's far more tractable than
full torque-WBC and reuses everything that works (servo stand + capture_step IK +
the multi-step state machine already in humanoid_stand_deploy.py). Recommend this next.

## UPDATE 3 — MPC ON THE POSITION SERVO WORKS (the torque-WBC's answer)
`g1_step_mpc.py` (loaded via `OMNISIM_INENGINE_PYMOD`, per-TICK, servo mode — NO
torque, NO PYMOD_SS). It settles on the deterministic stand, captures that pose as the
static nominal (read from qpos[qadr] — unambiguous), then OWNS the servo targets =
nominal + an MPPI residual planned by rolling out K worlds in the deploy's own
mujoco_warp (`_mpc_rollout_buffers`/`_mpc_seed_qv`, the proven quad machinery).
- **RESULT: it STANDS — 0 falls, bz≈0.75, reached t=11s sim** (`smpc_i`). The
  position-servo MPC balances in-engine with NONE of the torque-mode walls (no
  contact-consistency, no seed transient, no rate divergence, no sign). This is the
  concrete answer to "is MPC also blocked?": NO — MPC works where torque-WBC didn't,
  because it drives the robust servo and the rollout IS the realized contact.
- **Cost/stability trade-off**: stable only with K=64, H=15, EVERY=2 (res stays small);
  cheaper planning (K=40,H=8,EVERY=4) FALLS at 7s (residual grows 0.17→0.61 — short
  horizon/infrequent replanning can't react). So stable = ~0.1× realtime. Speed
  optimization (better nominal so res≈0 → plan less often; or graph the rollout) is the
  blocker for a live demo and for fast Phase-B iteration.
- **Layout fix**: the G1 newton-dof→joint map is NOT contiguous `6-11 L / 12-17 R`
  (g1_walk_wbc.py assumed that — WRONG, and its seed dict was mis-indexed). It pairs
  L/R by joint type: hip_pitch=6,7; knee=17,18; ankle_pitch=21,22; shoulder_roll=15,16.
  Read the nominal from qpos[qadr] (robust to the layout); joint_target_pos IS dof-indexed
  (tp[dof]==qpos[qadr] verified).
- **Phase A (done) = constant residual = optimized ankle/hip/arm balance** → holds, but
  by construction CANNOT step, so it won't SURPASS the base. **Phase B (next) = a
  capture-point-triggered, time-varying leg residual = recovery STEPS** (the surpass).
  Same biped single-support basin risk, BUT the MPPI plans a whole step over the horizon
  in real physics (not an instantaneous one-leg balance), which is the better shot.

## UPDATE 4 — PHASE B (capture-point stepping) built + tested: hits the single-support wall
`g1_step_mpc.py` now has a capture-point STEP state machine (`_step_fsm`): reads live
CoM/CoM-vel/feet (CPU MjData), computes the DCM/capture point, and when it leaves the
support polygon by > margin, triggers a STEP — picks the swing leg, computes a foothold
at the predicted CP (clamped to leg reach), and drives the swing foot via the validated
`capture_step.leg_ik` (geometric leg map: L=[6,9,13,17,21,25] R=[7,10,14,18,22,26],
decoded from the breadth-first newton layout). The swing leg is overridden in BOTH the
MPPI rollout and the apply; the MPPI residual balances stance+arms. It CHAINS (re-triggers
while the CP keeps escaping).
- **The machinery WORKS**: correct trigger, swing-leg pick, foothold, leg_ik execution,
  chaining, no false triggers when undisturbed (0 falls standing).
- **But it does NOT surpass the base — same single-support wall.** Tested pushes
  forward 0.8/1.3 and lateral 1.2 (all of which THIS base, g1.json arm_balance=OFF, also
  fails): the step fires but the CP GROWS during the swing's single-support phase
  (0.8 fwd: CP 0.22→0.48 during the step) faster than the foot can catch it, and the
  foothold is reach-limited (~0.22 m fwd, ~0.32 m lat). So: by the time a push is big
  enough that the base fails, the CP is already past single-step reach, and multi-step
  chaining is defeated because each step's single-support lets the CP accelerate. The
  MPPI balance during the step is NOT enough to hold single-support — the G1
  one-leg dynamic-balance wall, the SAME wall the
  torque-WBC / capture-step / multi-step hit.
- **NET across the whole effort**: deterministic G1 STANDING is solved many ways (the
  base, WBC-FF, and now MPC-on-servo Phase A all hold). Deterministic G1 push-recovery
  STEPPING is NOT — every method (servo, torque-WBC, MPC) converges on the single-support
  wall, which is documented as RL-requiring and the user excluded RL. Lateral is worst
  (support-removal paradox: lifting the fall-side foot needs a prior weight shift).
- Knobs: SMPC_STEP_EN, SMPC_CP_MARGIN, SMPC_STEP_TICKS, SMPC_STEP_LIFT.
- Speed: graphed rollout + the per-tick FSM `_read` (mj_forward) ~0.15-0.3x realtime.

## Launch / iterate (direct, NOT headless_runner)
headless_runner.py CRASHES the stand controller at startup in this env (controller exit
1, before world opens) — root cause unresolved; the warm reload also crashes there. The
DIRECT binary launch works (warm reload included). Use `_scratch/stand/run_smpc_direct.sh
<dur> <tag> [ENV=val ...]`. Knobs: SMPC_K/H/EVERY/WARM_TICKS, SMPC_SIGMA_LEG/ARM,
SMPC_RESMAX_LEG/ARM, SMPC_W_UP/H/VXY/RATE/RES/FALL, SMPC_LAM. (The `-StepMpc` launcher
switch exists but inherits the headless_runner crash.)

## NEXT STEPS (the real remaining work)
0. **Run per-substep (250 Hz)** — add a per-substep module hook (or route through the
   embedded TSID substep path) so torque control is stable. This is the prerequisite
   for everything else; the per-tick rate alone diverges.
1. **Consistent initialization** so `f_real ≈ f` from tick 1: either (a) make the ENGINE seed the squat (so the solver rebuild matches the qpos — fix the slot-ordered `OMNISIM_NEWTON_SEED_Q` or have the controller send the squat in torque mode), or (b) after the live.qpos seed, reset the solver internals (qacc/warmstart) / step the solver a few ticks under a robust torque-PID hold to settle real contact BEFORE the WBC's contact-force assumptions engage.
2. **Contact-state estimation**: only include a foot in the contact set if it's actually loaded (use `get_contacts()` force, not a height proxy); handle the no/partial-contact transient.
3. Once DS stand holds: in-place DCM/CoP rejection → single forced step (unload ramp + swing task + touchdown) → reactive step → N-step. Validate in that order.
4. Speed: the per-tick torque write disables the CUDA graph (~0.02–0.05× realtime). For a usable demo, run the WBC per-tick but minimize GPU readback, or accept slow + headless.

This is a multi-week walking-controller effort; the foundation (research-grounded correct WBC-QP + in-engine infra) is laid and the next wall (torque-mode contact consistency) is precisely identified.
