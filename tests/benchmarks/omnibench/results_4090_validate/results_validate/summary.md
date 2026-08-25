# OmniBench summary — 65dd6587d5c9

Generated 2026-07-25T21:19:57Z by run_all.py (suite omnibench/v0).

| machine | value |
|---|---|
| machine id | 65dd6587d5c9 |
| host | 80715bbfc6a0 |
| GPU | NVIDIA GeForce RTX 4090 (driver 580.126.20) |
| CPU | x86_64 (32 cores) |
| OS | Linux-6.8.0-107-generic-x86_64-with-glibc2.35 |
| python | 3.11.10 (numpy 2.4.6) |
| engine binary sha256 | 36732b708231390e |

All numbers below are attributable to THIS machine only (SPEC honesty rule 4).

## Lane 1 — physics correctness (analytic ground truth)

### T1 `bounce`

| engine | dt (ms) | bounce_height_rmse_rel | wall ms/step |
|---|---|---|---|
| omnisim-newton | 1 | GAP | — |
| omnisim-newton | 2 | GAP | — |
| omnisim-newton | 4 | 0.03811 | 15.97 |
| omnisim-newton | 8 | GAP | — |
| omnisim-newton | 16 | GAP | — |
| omnisim-newton | 32 | GAP | — |
| omnisim-ode | 1 | GAP | — |
| omnisim-ode | 2 | GAP | — |
| omnisim-ode | 4 | 0.01788 | 0.0135 |
| omnisim-ode | 8 | GAP | — |
| omnisim-ode | 16 | GAP | — |
| omnisim-ode | 32 | GAP | — |

<details><summary>deviations (bounce)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ContactProperties softCFM=1e-06 overrides the engine default 0.001 (default CFM mixes into the ODE bounce constraint and kills restitution: measured first peak 0.02 m at dt=1 ms vs 0.64 analytic)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.0 (ContactProperties.coulombFriction=0 is ODE-only)
  - Newton has no restitution coefficient; e=0.8 realised via contact-compliance calibration OMNISIM_NEWTON_CONTACT_KD=7 (ke=2500 default; damped-spring zeta=0.070 -> e~0.80, calibrated at dt=1 ms per SPEC; engine-default kd=100 gives e~0). Soft contact penetrates ~0.08 m at impact — recorded, not hidden
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ContactProperties softCFM=1e-06 overrides the engine default 0.001 (default CFM mixes into the ODE bounce constraint and kills restitution: measured first peak 0.02 m at dt=1 ms vs 0.64 analytic)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement

</details>

### T2 `incline`

| engine | dt (ms) | stick_violation_max_m | slide_accel_rel_err | transition_angle_err_deg | wall ms/step |
|---|---|---|---|---|---|
| omnisim-newton | 1 | GAP | GAP | GAP | — |
| omnisim-newton | 2 | GAP | GAP | GAP | — |
| omnisim-newton | 4 | 0.00068 | 0.03811 | 0.06505 | 8.707 |
| omnisim-newton | 8 | GAP | GAP | GAP | — |
| omnisim-newton | 16 | GAP | GAP | GAP | — |
| omnisim-newton | 32 | GAP | GAP | GAP | — |
| omnisim-ode | 1 | GAP | GAP | GAP | — |
| omnisim-ode | 2 | GAP | GAP | GAP | — |
| omnisim-ode | 4 | 2.9e-05 | 8.78e-05 | 0.06505 | 0.01454 |
| omnisim-ode | 8 | GAP | GAP | GAP | — |
| omnisim-ode | 16 | GAP | GAP | GAP | — |
| omnisim-ode | 32 | GAP | GAP | GAP | — |

<details><summary>deviations (incline)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.5 (ContactProperties.coulombFriction=0.5 is ODE-only)
  - newtonCone "elliptic" + newtonImpratio 10 pinned in the world (MuJoCo-stock pyramidal cone creeps near the friction-cone boundary: 181 mm pseudo-slip at 26 deg with mu=0.5; elliptic+impratio-10 sticks at 0.6 mm). Global Newton default stays MuJoCo stock pending champion re-verification
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s

</details>

### T3 `roll`

| engine | dt (ms) | roll_accel_rel_err | slip_ratio | wall ms/step |
|---|---|---|---|---|
| omnisim-newton | 1 | GAP | GAP | — |
| omnisim-newton | 2 | GAP | GAP | — |
| omnisim-newton | 4 | 0.000582 | 0.000362 | 26.86 |
| omnisim-newton | 8 | GAP | GAP | — |
| omnisim-newton | 16 | GAP | GAP | — |
| omnisim-newton | 32 | GAP | GAP | — |
| omnisim-ode | 1 | GAP | GAP | — |
| omnisim-ode | 2 | GAP | GAP | — |
| omnisim-ode | 4 | 0.001055 | 0.001843 | 0.01307 |
| omnisim-ode | 8 | GAP | GAP | — |
| omnisim-ode | 16 | GAP | GAP | — |
| omnisim-ode | 32 | GAP | GAP | — |

<details><summary>deviations (roll)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.8 (ContactProperties.coulombFriction=0.8 is ODE-only)
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement

</details>

### T4 `pendulum_energy`

| engine | dt (ms) | energy_drift_rel | energy_drift_slope | wall ms/step |
|---|---|---|---|---|
| omnisim-newton | 1 | GAP | GAP | — |
| omnisim-newton | 2 | GAP | GAP | — |
| omnisim-newton | 4 | 0.381 | -0.04728 | 8.3 |
| omnisim-newton | 8 | GAP | GAP | — |
| omnisim-newton | 16 | GAP | GAP | — |
| omnisim-newton | 32 | GAP | GAP | — |
| omnisim-ode | 1 | GAP | GAP | — |
| omnisim-ode | 2 | GAP | GAP | — |
| omnisim-ode | 4 | 0.13 | -0.01292 | 0.01696 |
| omnisim-ode | 8 | GAP | GAP | — |
| omnisim-ode | 16 | GAP | GAP | — |
| omnisim-ode | 32 | GAP | GAP | — |

<details><summary>deviations (pendulum_energy)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_blowup_max (metric v1) = max|E(t)| / analytic swing energy (22.07 J); >10 sets energy_blew_up and marks the peak-KE-normalized drift as meaningless (an exploding run inflates its own normalizer)
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement
  - energy_drift normalized by peak KE (21.712 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_blowup_max (metric v1) = max|E(t)| / analytic swing energy (22.07 J); >10 sets energy_blew_up and marks the peak-KE-normalized drift as meaningless (an exploding run inflates its own normalizer)

</details>

### T5 `momentum`

| engine | dt (ms) | linear_momentum_max | angular_momentum_drift_rel | wall ms/step |
|---|---|---|---|---|
| omnisim-newton | 1 | GAP | GAP | — |
| omnisim-newton | 2 | GAP | GAP | — |
| omnisim-newton | 4 | 3.91 | 0.9771 | 7.302 |
| omnisim-newton | 8 | GAP | GAP | — |
| omnisim-newton | 16 | GAP | GAP | — |
| omnisim-newton | 32 | GAP | GAP | — |
| omnisim-ode | 1 | GAP | GAP | — |
| omnisim-ode | 2 | GAP | GAP | — |
| omnisim-ode | 4 | 7.58e-15 | 0.1503 | 0.01675 |
| omnisim-ode | 8 | GAP | GAP | — |
| omnisim-ode | 16 | GAP | GAP | — |
| omnisim-ode | 32 | GAP | GAP | — |

<details><summary>deviations (momentum)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - T5 torque phase is a supervisor addTorque couple on the two links flanking the middle joint (exact for this planar chain), not Motor.setTorque: the Newton backend gives motorized hinges a hardcoded position servo that pins the joint and pumps linear momentum (~17 kg*m/s measured); the couple keeps both joints passive under both backends
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.3112 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - T5 torque phase is a supervisor addTorque couple on the two links flanking the middle joint (exact for this planar chain), not Motor.setTorque: the Newton backend gives motorized hinges a hardcoded position servo that pins the joint and pumps linear momentum (~17 kg*m/s measured); the couple keeps both joints passive under both backends
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.04577 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0

</details>

### T6 `stack`

| engine | dt (ms) | stack_survivors | settle_creep_m_s | max_penetration_m | wall ms/step |
|---|---|---|---|---|---|
| omnisim-newton | 1 | GAP | GAP | GAP | — |
| omnisim-newton | 2 | GAP | GAP | GAP | — |
| omnisim-newton | 4 | 10 | 3.94e-06 | 0.000902 | 7.157 |
| omnisim-newton | 8 | GAP | GAP | GAP | — |
| omnisim-newton | 16 | GAP | GAP | GAP | — |
| omnisim-newton | 32 | GAP | GAP | GAP | — |
| omnisim-ode | 1 | GAP | GAP | GAP | — |
| omnisim-ode | 2 | GAP | GAP | GAP | — |
| omnisim-ode | 4 | 10 | 5.21e-08 | 0.000245 | 0.1809 |
| omnisim-ode | 8 | GAP | GAP | GAP | — |
| omnisim-ode | 16 | GAP | GAP | GAP | — |
| omnisim-ode | 32 | GAP | GAP | GAP | — |

<details><summary>deviations (stack)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.8 (ContactProperties.coulombFriction=0.8 is ODE-only)
  - newtonCone "elliptic" + newtonImpratio 10 pinned in the world (MuJoCo-stock pyramidal cone creeps near the friction-cone boundary: 181 mm pseudo-slip at 26 deg with mu=0.5; elliptic+impratio-10 sticks at 0.6 mm). Global Newton default stays MuJoCo stock pending champion re-verification
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)

</details>

### T7 `spin`

| engine | dt (ms) | angmom_drift_rel | rot_ke_drift_rel | wall ms/step |
|---|---|---|---|---|
| omnisim-newton | 1 | GAP | GAP | — |
| omnisim-newton | 2 | GAP | GAP | — |
| omnisim-newton | 4 | 2.55e-05 | 7.95e-08 | 0.3605 |
| omnisim-newton | 8 | GAP | GAP | — |
| omnisim-newton | 16 | GAP | GAP | — |
| omnisim-newton | 32 | GAP | GAP | — |
| omnisim-ode | 1 | GAP | GAP | — |
| omnisim-ode | 2 | GAP | GAP | — |
| omnisim-ode | 4 | 1.93e-05 | 3.47e-08 | 0.01261 |
| omnisim-ode | 8 | GAP | GAP | — |
| omnisim-ode | 16 | GAP | GAP | — |
| omnisim-ode | 32 | GAP | GAP | — |

<details><summary>deviations (spin)</summary>

- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - initial spin: recorder tries supervisor setVelocity first (a t=0 setVelocity used to be dropped on Newton — pre-registration immediate message; now queued + drained after finalize) and falls back to a closed-loop torque-impulse spin-up if the achieved omega is off; the method actually used is in the .meta.json 'spin' block
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - initial spin: recorder tries supervisor setVelocity first (a t=0 setVelocity used to be dropped on Newton — pre-registration immediate message; now queued + drained after finalize) and falls back to a closed-loop torque-impulse spin-up if the achieved omega is off; the method actually used is in the .meta.json 'spin' block
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"). Historical: OMNISIM_FORCE_ODE alone used to be bypassed by the staticBase-Robot Newton registration (frozen chain-link pose readback, OmSolid::postPhysicsStep P3.10f); the gating gap is fixed (odeForced() gates the raw-accessor paths), the pin is kept as the declarative belt-and-braces statement

</details>

## FINDINGS (auto: sanity-threshold breaches)

- [lane1] pendulum_energy omnisim-ode dt=4 ms: energy_drift_rel=0.13 (> 0.1)
- [lane1] momentum omnisim-ode dt=4 ms: angular_momentum_drift_rel=0.1503 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=4 ms: energy_drift_rel=0.381 (> 0.1)
- [lane1] momentum omnisim-newton dt=4 ms: linear_momentum_max=3.91 (> 0.1)
- [lane1] momentum omnisim-newton dt=4 ms: angular_momentum_drift_rel=0.9771 (> 0.1)

## Gaps (recorded in MANIFEST.json)

- none
