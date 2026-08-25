# G1 Centroidal MPC + WBC stack (on OmniSim)

Implements the "Implementation Stack for MPC Walking on Unitree G1 in Newton Physics" plan,
**built on OmniSim's existing Newton/mujoco_warp runtime** (which already provides the plan's
layers A–D: validated G1 model, in-engine control hook, deterministic stand, harness) rather
than greenfield on raw Newton. Deterministic, **no RL**.

Why this and not the sampling-MPC / RL attempts: the in-engine MPC plans/realizes against the
**same** mujoco_warp model it deploys on → **zero sim-to-deploy obs gap** (the wall that
killed RL here), and the QP is deterministic (no MPPI thrashing). See
`../g1_mpc_pushrecovery_research.md` for the literature + why the prior attempts wall'd.

## Modules (clean separation, the plan's architecture)
- `robot_io.py` — `CentroidalState` + the sim backend boundary (controller depends only on this).
- `centroidal_mpc.py` — the centroidal **force QP** (section 8). CoM + torso + **angular-momentum
  (CAM)** terms; friction-cone / unilateral / CoP constraints; pure-numpy regularized-normal-
  equation + cone projection (no scipy/OSQP needed in the embedded interpreter). The CAM term is
  first-class — it's the body-rotation arrest the push-recovery session found to be the binding wall.
- `wbc.py` — maps per-foot GRFs → actuated joint feedforward torques (`τ = −ΣJc,aᵀf`), applied
  on the position servo (FF-on-servo, the proven OmniSim substrate). MVP; the inverse-dynamics QP
  is the later upgrade.
- `driver.py` — in-engine loop (`cmpc_step`), loaded via `OMNISIM_INENGINE_PYMOD`. Reads live
  centroidal state, runs MPC → WBC → FF. `CMPC_STAGE` selects the ladder milestone.

## Status (the plan's M-ladder)
- **M4 (standing centroidal balance): NOT durably stable -- collapses ~9.5 s. CORRECTION.**
  An earlier "M4 works" claim came from a too-short (6 s) test. Over a longer run the force-QP
  FF holds cleanly ~5 s (CoM tracked to 0, pitch <0.04, Fz≈m·g) then the FF saturates and the
  stand DIVERGES (fall ~9.5 s). The deterministic stand ALONE holds 122 s, so the centroidal-
  MPC FF is net-DESTABILIZING for pure standing (a slow closed-loop instability between the FF
  and the servo/lean). The genuinely working, durable deliverable is the DETERMINISTIC stand
  (+ WBC-FF). Run: `_scratch/stand/run_cmpc_direct.sh`.
- **KEY FINDING (the precise wall, re-derived from first principles):** the SRBM moment is
  `dL/dt = Σ(p_i−c)×f_i` — **only the contact-force (CoP) moment, which is ankle-limited**
  (~Fz·foot_half_length ≈ 38 N·m). With the deterministic lean ON, the SRBM force-QP FF is
  largely *redundant* with the lean (both are ankle strategies) and the stand holds but pitch
  slowly drifts; with lean OFF the SRBM alone **cannot hold pitch** (0.354 rad immediately,
  QP residual resM≈203 because it demands a moment the feet can't produce → falls ~4s). A
  pitch beyond ankle CoP needs **limb angular momentum (arm/hip CAM)**, which the single-
  rigid-body model *structurally omits*. This is the same body-rotation wall every prior
  approach hit (WBC, capture-step, MPPI, RL) — now explained: they were all effectively
  CoP/ankle-limited or couldn't realize the CAM.
- **Consequence — the plan's ladder must be reordered:** robust M5/M6 (single-foot lift +
  stepping) need *more* moment than double-support, but single support has *less* CoP, so the
  SRBM MVP cannot sustain single-support balance. The **full-centroidal-momentum WBC with an
  arm/CAM angular-momentum task (plan's M7) is a prerequisite for M5/M6, not a later polish.**
  Concretely: extend the model to full centroidal momentum (incl. limb contributions) and add
  a WBC angular-momentum task that accelerates the arms/torso to produce dL beyond the ankle.
- **M5 (gait + swing + single-foot lift): BUILT + tested -> hits the single-support wall.**
  `gait.py` (weight-shift -> lift -> hold -> lower -> alternate) + swing IK + the contact-
  scheduled centroidal MPC. With tuning (slow shift, strong kp_com) the CoM does shift over
  the stance foot (comY 0.035->0.098), but the lift still topples: the SINGLE foot's CoP can't
  hold the body's pitch/roll moment during single support (resM grows), even with the CAM. This
  is the documented G1 single-foot-hold instability (project_g1_oneleg_attempt: unstable ~1-2s,
  RL-requiring). The CAM (arm windmill) extends the moment but not enough for reliable single
  support.
- **CONCLUSIVE FINDING (re-derived from first principles via the principled plan):** the gate
  to stepping/walking/push-recovery-beyond-base is **deterministic single-support balance**,
  which is bounded by the single foot's CoP + the available limb angular momentum. The
  centroidal MPC + CAM formalizes the balance and nails DOUBLE support (M4, deploys) but cannot
  reliably hold SINGLE support -- the same wall every approach this session hit (WBC, capture-
  step, torque, MPPI, RL), now explained mechanistically. For this robot it appears near the
  morphological limit (small feet, weak ankle), not a controller-tuning gap.
- **Net deterministic deliverables (solid + deployed):** the stand, double-support centroidal
  balance (M4), and the cube-defense. Single-support (lift/step/walk) is the open problem;
  the only thing that walks the G1 is a learned policy (Unitree), which itself doesn't deploy
  durably from-scratch (the obs gap). M6/M7 are gated on cracking single support, which is
  RL-territory for this robot.

## Key env knobs
`CMPC_STAGE` (4/5/6), `CMPC_WARM_TICKS`, `CMPC_KP_COM/KD_COM/KP_Z/KD_Z`, `CMPC_KP_L/KD_L`
(angular momentum), `CMPC_W_FORCE/W_MOMENT`, `CMPC_MU`, `CMPC_TAUMAX`, `CMPC_TAU_SIGN`.
