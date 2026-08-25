# OmniBench summary — 6fa66da0cde0

Generated 2026-07-24T20:12:39Z by run_all.py (suite omnibench/v0).

| machine | value |
|---|---|
| machine id | 6fa66da0cde0 |
| host | machine-ed46230de3c4 |
| GPU | NVIDIA GeForce RTX 4090 (driver 570.195.03) |
| CPU | x86_64 (32 cores) |
| OS | Linux-6.8.0-90-generic-x86_64-with-glibc2.35 |
| python | 3.11.10 (numpy 2.4.6) |
| engine binary sha256 | 47c57f8bcec1c9ec |

All numbers below are attributable to THIS machine only (SPEC honesty rule 4).

## Lane 1 — physics correctness (analytic ground truth)

### T1 `bounce`

| engine | dt (ms) | bounce_height_rmse_rel | wall ms/step |
|---|---|---|---|
| mujoco | 1 | 0.1725 | 0.001414 |
| mujoco | 2 | 0.2715 | 0.001415 |
| mujoco | 4 | 0.1478 | 0.001331 |
| mujoco | 8 | 35.97 | 0.001339 |
| mujoco | 16 | 33.52 | 0.00126 |
| mujoco | 32 | 396.5 | 0.001309 |
| omnisim-newton | 1 | 0.01028 | 3.649 |
| omnisim-newton | 2 | 0.01769 | 1.225 |
| omnisim-newton | 4 | 0.03811 | 2.21 |
| omnisim-newton | 8 | 0.08393 | 4.129 |
| omnisim-newton | 16 | 0.2167 | 11.29 |
| omnisim-newton | 32 | 0.4986 | 15.16 |
| omnisim-ode | 1 | 0.01872 | 0.7877 |
| omnisim-ode | 2 | 0.01785 | 1.144 |
| omnisim-ode | 4 | 0.01788 | 2.032 |
| omnisim-ode | 8 | 0.01295 | 4.048 |
| omnisim-ode | 16 | 0.08463 | 12.99 |
| omnisim-ode | 32 | 0.07143 | 21.75 |
| pybullet | 1 | 0.01706 | 0.001163 |
| pybullet | 2 | 0.03924 | 0.00115 |
| pybullet | 4 | 0.07337 | 0.001163 |
| pybullet | 8 | 0.1606 | 0.001167 |
| pybullet | 16 | 0.3497 | 0.001299 |
| pybullet | 32 | 0.7752 | 0.001439 |

<details><summary>deviations (bounce)</summary>

- **mujoco**:
  - mujoco has no restitution coeff; direct solref=(-100000,-54.4678) calibrated at dt=1ms -> first peak 0.6400 m (target 0.6400)
  - solimp=(0.95,0.95,0.001) constant impedance => linear contact (speed-independent e; converges to analytic peaks at dt=0.25ms)
  - fixed contact stiffness k=100000 has sqrt(k/m)*dt>2 for dt>=8ms: contact integration unstable there (energy GAIN), which is the honest sweep result of freezing the dt=1ms mapping
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
  - only 3/5 bounce peaks detected; missing peaks scored as h=0
  - only 1/5 bounce peaks detected; missing peaks scored as h=0
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ContactProperties softCFM=1e-06 overrides the engine default 0.001 (default CFM mixes into the ODE bounce constraint and kills restitution: measured first peak 0.02 m at dt=1 ms vs 0.64 analytic)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.0 (ContactProperties.coulombFriction=0 is ODE-only)
  - Newton has no restitution coefficient; e=0.8 realised via contact-compliance calibration OMNISIM_NEWTON_CONTACT_KD=7 (ke=2500 default; damped-spring zeta=0.070 -> e~0.80, calibrated at dt=1 ms per SPEC; engine-default kd=100 gives e~0). Soft contact penetrates ~0.08 m at impact — recorded, not hidden
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ContactProperties softCFM=1e-06 overrides the engine default 0.001 (default CFM mixes into the ODE bounce constraint and kills restitution: measured first peak 0.02 m at dt=1 ms vs 0.64 analytic)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
- **pybullet**:
  - bullet restitution combines multiplicatively: plane e=1.0, ball e=0.8 (set directly); restitutionVelocityThreshold default
  - override: contactProcessingThreshold=0 (default persistence kills bounces after the 1st) and erp=contactERP=0 (ERP push-out otherwise adds ~+0.35 m/s per impact, e_eff~0.88)
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

### T2 `incline`

| engine | dt (ms) | stick_violation_max_m | slide_accel_rel_err | transition_angle_err_deg | wall ms/step |
|---|---|---|---|---|---|
| mujoco | 1 | 0.000442 | 0.008854 | 0.06505 | 0.01895 |
| mujoco | 2 | 0.000442 | 0.01089 | 0.06505 | 0.0193 |
| mujoco | 4 | 0.000442 | 0.006183 | 0.06505 | 0.01907 |
| mujoco | 8 | 0.000442 | 0.01286 | 0.06505 | 0.0195 |
| mujoco | 16 | 0.000639 | 0.04927 | 0.06505 | 0.02122 |
| mujoco | 32 | 0.000769 | 0.3298 | 0.06505 | 0.02092 |
| omnisim-newton | 1 | 0.196 | 0.2072 | 4.065 | 1.16 |
| omnisim-newton | 2 | 0.2112 | 0.2252 | 4.065 | 1.891 |
| omnisim-newton | 4 | 0.1806 | 0.1908 | 4.065 | 3.234 |
| omnisim-newton | 8 | 0.2326 | 0.2538 | 4.065 | 5.547 |
| omnisim-newton | 16 | 0.2734 | 0.3152 | 4.065 | 12.3 |
| omnisim-newton | 32 | 0.6741 | 0.006539 | 9.065 | 26.53 |
| omnisim-ode | 1 | 3.04e-05 | 9.53e-05 | 0.06505 | 1.136 |
| omnisim-ode | 2 | 2.94e-05 | 0.000114 | 0.06505 | 1.949 |
| omnisim-ode | 4 | 2.9e-05 | 8.78e-05 | 0.06505 | 4.053 |
| omnisim-ode | 8 | 2.91e-05 | 0.000142 | 0.06505 | 7.318 |
| omnisim-ode | 16 | 3.02e-05 | 0.007449 | 0.06505 | 16.71 |
| omnisim-ode | 32 | 5.22e-05 | 0.0633 | 0.06505 | 31.35 |
| pybullet | 1 | 0.001116 | 8.42e-07 | 0.06505 | 0.02698 |
| pybullet | 2 | 0.001434 | 2.92e-05 | 0.06505 | 0.02841 |
| pybullet | 4 | 0.001952 | 6.56e-05 | 0.06505 | 0.03032 |
| pybullet | 8 | 0.001349 | 0.001151 | 0.06505 | 0.03505 |
| pybullet | 16 | 0.001377 | 0.001672 | 0.06505 | 0.0376 |
| pybullet | 32 | 0.001457 | 0.007891 | 0.06505 | 0.04095 |

<details><summary>deviations (incline)</summary>

- **mujoco**:
  - override: cone=elliptic impratio=10 (default pyramidal cone creeps ~5mm/3s at 15-26 deg; with override sub-0.5mm stick below theta_c)
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.5 (ContactProperties.coulombFriction=0.5 is ODE-only)
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s
- **pybullet**:
  - bullet friction combines multiplicatively: incline mu=1.0, box mu=0.5
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s

</details>

### T3 `roll`

| engine | dt (ms) | roll_accel_rel_err | slip_ratio | wall ms/step |
|---|---|---|---|---|
| mujoco | 1 | 0.001004 | 0.00172 | 0.002407 |
| mujoco | 2 | 0.001004 | 0.00172 | 0.002777 |
| mujoco | 4 | 0.001003 | 0.00172 | 0.002348 |
| mujoco | 8 | 0.001003 | 0.001719 | 0.002488 |
| mujoco | 16 | 0.00177 | 0.003086 | 0.00242 |
| mujoco | 32 | 0.005122 | 0.009473 | 0.002618 |
| omnisim-newton | 1 | 0.4763 | 0.001737 | 1.231 |
| omnisim-newton | 2 | 0.4763 | 0.001737 | 2.53 |
| omnisim-newton | 4 | 0.4763 | 0.001737 | 3.103 |
| omnisim-newton | 8 | 0.4763 | 0.001736 | 9.564 |
| omnisim-newton | 16 | 0.477 | 0.001574 | 12.41 |
| omnisim-newton | 32 | 0.4786 | 0.000499 | 29.97 |
| omnisim-ode | 1 | 0.000263 | 0.000457 | 1.151 |
| omnisim-ode | 2 | 0.000527 | 0.000919 | 2.791 |
| omnisim-ode | 4 | 0.001055 | 0.001843 | 4.557 |
| omnisim-ode | 8 | 0.002115 | 0.003697 | 8.394 |
| omnisim-ode | 16 | 0.004192 | 0.007425 | 20.23 |
| omnisim-ode | 32 | 0.008016 | 0.01509 | 39.19 |
| pybullet | 1 | 1.3e-15 | 2.04e-15 | 0.002514 |
| pybullet | 2 | 1.85e-16 | 1.11e-15 | 0.002896 |
| pybullet | 4 | 2.04e-15 | 3.89e-15 | 0.00287 |
| pybullet | 8 | 1.04e-14 | 9.27e-16 | 0.002738 |
| pybullet | 16 | 1.17e-14 | 1.12e-15 | 0.002667 |
| pybullet | 32 | 1.09e-14 | 5.6e-15 | 0.003054 |

<details><summary>deviations (roll)</summary>

- **mujoco**:
  - override: cone=elliptic impratio=10 (same as T2)
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.8 (ContactProperties.coulombFriction=0.8 is ODE-only)
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
- **pybullet**:
  - bullet friction combines multiplicatively: incline mu=1.0, ball mu=0.8
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

### T4 `pendulum_energy`

| engine | dt (ms) | energy_drift_rel | energy_drift_slope | wall ms/step |
|---|---|---|---|---|
| mujoco | 1 | 0.09359 | -0.008973 | 0.001541 |
| mujoco | 2 | 0.1613 | -0.02263 | 0.001613 |
| mujoco | 4 | 0.3871 | -0.04797 | 0.001543 |
| mujoco | 8 | 0.6089 | -0.09184 | 0.001596 |
| mujoco | 16 | 0.4205 | -0.07004 | 0.001586 |
| mujoco | 32 | 1.55e-08 | 0.001028 | 0.1286 |
| omnisim-newton | 1 | 0.2742 | -0.03249 | 0.6107 |
| omnisim-newton | 2 | 0.4825 | -0.05923 | 0.7203 |
| omnisim-newton | 4 | 0.4974 | -0.06084 | 1.113 |
| omnisim-newton | 8 | 0.4621 | -0.03718 | 1.879 |
| omnisim-newton | 16 | 0.2779 | 0.01773 | 4.707 |
| omnisim-newton | 32 | 6.8e-09 | 0.000514 | 5.666 |
| omnisim-ode | 1 | 0.009001 | -0.000737 | 0.2957 |
| omnisim-ode | 2 | 0.04631 | -0.005271 | 0.5523 |
| omnisim-ode | 4 | 0.13 | -0.01292 | 1.456 |
| omnisim-ode | 8 | 0.3664 | -0.03581 | 2.74 |
| omnisim-ode | 16 | 0.5224 | -0.04737 | 4.885 |
| omnisim-ode | 32 | 0.7755 | -0.06037 | 9.974 |
| pybullet | 1 | 0.06948 | -0.008869 | 0.002087 |
| pybullet | 2 | 0.1585 | -0.02265 | 0.002103 |
| pybullet | 4 | 0.3859 | -0.04806 | 0.002077 |
| pybullet | 8 | 0.6113 | -0.09202 | 0.002209 |
| pybullet | 16 | 0.4056 | -0.07025 | 0.002201 |
| pybullet | 32 | 0.9992 | 0.09548 | 0.002226 |

<details><summary>deviations (pendulum_energy)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
  - energy_drift normalized by peak KE (22.401 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (22.815 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (25.696 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (27.697 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (588992845.392 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - energy_drift normalized by peak KE (24.468 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (22.399 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.197 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (28.397 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (29.405 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (2316564746.811 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
  - energy_drift normalized by peak KE (21.997 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (21.888 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (21.712 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (21.529 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (20.975 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (20.239 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
- **pybullet**:
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown
  - energy_drift normalized by peak KE (22.401 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (22.815 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (25.696 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (27.697 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (10900.541 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined

</details>

### T5 `momentum`

| engine | dt (ms) | linear_momentum_max | angular_momentum_drift_rel | wall ms/step |
|---|---|---|---|---|
| mujoco | 1 | 0.7994 | 0.002611 | 0.001824 |
| mujoco | 2 | 2.361 | 0.2663 | 0.001812 |
| mujoco | 4 | 5.847 | 2.089 | 0.001817 |
| mujoco | 8 | 5.667 | 0.07964 | 0.001969 |
| mujoco | 16 | 3.24e+04 | 1.23 | 0.08452 |
| mujoco | 32 | 4.4e+03 | 1.174 | 0.1061 |
| omnisim-newton | 1 | 17.18 | 0.9999 | 0.6234 |
| omnisim-newton | 2 | 17.46 | 1 | 0.8675 |
| omnisim-newton | 4 | 18.69 | 0.9999 | 1.281 |
| omnisim-newton | 8 | 21.32 | 1 | 1.825 |
| omnisim-newton | 16 | 24.26 | 1.76 | 3.729 |
| omnisim-newton | 32 | 17.34 | 0.9998 | 6.831 |
| omnisim-ode | 1 | 2.53e-14 | 6.691 | 0.3296 |
| omnisim-ode | 2 | 2.47e-14 | 7.988 | 0.5448 |
| omnisim-ode | 4 | 7.58e-15 | 0.3955 | 1.188 |
| omnisim-ode | 8 | 3.64e-15 | 1.442 | 3.138 |
| omnisim-ode | 16 | 4.12e-15 | 1.058 | 4.82 |
| omnisim-ode | 32 | 1.18e-15 | 0.2077 | 11.21 |
| pybullet | 1 | 0.8865 | 0.0002 | 0.001851 |
| pybullet | 2 | 2.365 | 0.2605 | 0.001884 |
| pybullet | 4 | 6.033 | 2.198 | 0.001913 |
| pybullet | 8 | 5.951 | 0.1703 | 0.001916 |
| pybullet | 16 | 355.4 | 34.45 | 0.001947 |
| pybullet | 32 | 252.6 | 47.33 | 0.002085 |

<details><summary>deviations (momentum)</summary>

- **mujoco**:
  - 'middle joint' = link1-link2 hinge (3 links have only 2 internal joints)
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - T5 torque phase is a supervisor addTorque couple on the two links flanking the middle joint (exact for this planar chain), not Motor.setTorque: the Newton backend gives motorized hinges a hardcoded position servo that pins the joint and pumps linear momentum (~17 kg*m/s measured); the couple keeps both joints passive under both backends
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - T5 torque phase is a supervisor addTorque couple on the two links flanking the middle joint (exact for this planar chain), not Motor.setTorque: the Newton backend gives motorized hinges a hardcoded position servo that pins the joint and pumps linear momentum (~17 kg*m/s measured); the couple keeps both joints passive under both backends
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
- **pybullet**:
  - 'middle joint' = link1-link2 hinge (3 links have only 2 internal joints)
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

### T6 `stack`

| engine | dt (ms) | stack_survivors | settle_creep_m_s | max_penetration_m | wall ms/step |
|---|---|---|---|---|---|
| mujoco | 1 | 10 | 1.22e-07 | 0.000618 | 0.03539 |
| mujoco | 2 | 10 | 1.46e-06 | 0.000618 | 0.03529 |
| mujoco | 4 | 10 | 2.35e-06 | 0.000618 | 0.03641 |
| mujoco | 8 | 10 | 6.14e-06 | 0.000618 | 0.03546 |
| mujoco | 16 | 10 | 0.002501 | 0.001188 | 0.0362 |
| mujoco | 32 | 1 | 1.18e-08 | 0.2979 | 0.03268 |
| omnisim-newton | 1 | 10 | 0.00067 | 0.000618 | 0.6523 |
| omnisim-newton | 2 | 10 | 0.000379 | 0.000579 | 1.028 |
| omnisim-newton | 4 | 10 | 0.001147 | 0.000617 | 1.697 |
| omnisim-newton | 8 | 1 | 1.65e-08 | 0.4989 | 1.791 |
| omnisim-newton | 16 | 2 | 0.1015 | 0.02593 | 4.282 |
| omnisim-newton | 32 | 1 | 0 | 0.2981 | 5.12 |
| omnisim-ode | 1 | 10 | 1.67e-09 | 6.13e-05 | 0.5276 |
| omnisim-ode | 2 | 10 | 1.3e-09 | 0.000123 | 0.7304 |
| omnisim-ode | 4 | 10 | 5.21e-08 | 0.000245 | 1.42 |
| omnisim-ode | 8 | 10 | 2.12e-07 | 0.000491 | 2.693 |
| omnisim-ode | 16 | 10 | 6.22e-08 | 0.000981 | 4.688 |
| omnisim-ode | 32 | 1 | 0.26 | 0.1996 | 10.07 |
| pybullet | 1 | 10 | 8.45e-05 | 3.2e-05 | 0.1127 |
| pybullet | 2 | 10 | 9.44e-05 | 9.69e-05 | 0.11 |
| pybullet | 4 | 10 | 0.000187 | 0.000356 | 0.111 |
| pybullet | 8 | 1 | 6.32e-05 | 0.2999 | 0.08438 |
| pybullet | 16 | 1 | 2.04e-06 | 0.2997 | 0.0674 |
| pybullet | 32 | 0 | 9.86e-07 | 0.1 | 0.05093 |

<details><summary>deviations (stack)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.8 (ContactProperties.coulombFriction=0.8 is ODE-only)
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)
- **pybullet**:
  - bullet friction combines multiplicatively: all bodies mu=sqrt(0.8)=0.8944 so box-box and box-plane pairs both = 0.8
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)

</details>

### T7 `spin`

| engine | dt (ms) | angmom_drift_rel | rot_ke_drift_rel | wall ms/step |
|---|---|---|---|---|
| mujoco | 1 | 0.001691 | 0.001418 | 0.00132 |
| mujoco | 2 | 0.003299 | 0.002637 | 0.001288 |
| mujoco | 4 | 0.00625 | 0.004537 | 0.001269 |
| mujoco | 8 | 0.01103 | 0.006613 | 0.001287 |
| mujoco | 16 | 0.01645 | 0.006764 | 0.001286 |
| mujoco | 32 | 0.01631 | 0.00312 | 0.001289 |
| omnisim-newton | 1 | 1 | 1 | 0.3619 |
| omnisim-newton | 2 | 1 | 1 | 0.364 |
| omnisim-newton | 4 | 1 | 1 | 0.3639 |
| omnisim-newton | 8 | 1 | 1 | 0.3698 |
| omnisim-newton | 16 | 1 | 1 | 0.3749 |
| omnisim-newton | 32 | 1 | 1 | 0.3824 |
| omnisim-ode | 1 | 4.86e-06 | 8.78e-09 | 0.01057 |
| omnisim-ode | 2 | 9.71e-06 | 1.75e-08 | 0.01053 |
| omnisim-ode | 4 | 1.93e-05 | 3.47e-08 | 0.01073 |
| omnisim-ode | 8 | 3.84e-05 | 6.81e-08 | 0.01085 |
| omnisim-ode | 16 | 7.55e-05 | 1.31e-07 | 0.01102 |
| omnisim-ode | 32 | 0.000146 | 2.42e-07 | 0.01203 |
| pybullet | 1 | 4.88e-06 | 8.84e-09 | 0.000803 |
| pybullet | 2 | 9.76e-06 | 1.77e-08 | 0.00082 |
| pybullet | 4 | 1.95e-05 | 3.55e-08 | 0.000818 |
| pybullet | 8 | 3.92e-05 | 7.14e-08 | 0.000817 |
| pybullet | 16 | 7.88e-05 | 1.45e-07 | 0.000858 |
| pybullet | 32 | 0.00016 | 2.97e-07 | 0.000856 |

<details><summary>deviations (spin)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - initial spin: supervisor setVelocity is not plumbed to Newton bodies (measured omega stays ~0), so the recorder falls back to a closed-loop torque-impulse spin-up; recording starts after spin-up, achieved omega is in the .meta.json 'spin' block
- **omnisim-ode**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - initial spin: supervisor setVelocity is not plumbed to Newton bodies (measured omega stays ~0), so the recorder falls back to a closed-loop torque-impulse spin-up; recording starts after spin-up, achieved omega is in the .meta.json 'spin' block
  - ODE lane runs the _odepin world variant (WorldInfo defaultPhysicsBackend "ode"): OMNISIM_FORCE_ODE alone does not stop the staticBase-Robot Newton registration, whose Newton-backed ancestor then freezes supervisor pose readback of chain links (OmSolid::postPhysicsStep P3.10f)
- **pybullet**:
  - pybullet damps the off-axis seed ~3x in the first second and the intermediate-axis (Dzhanibekov) eruption does NOT occur within the 10 s window (timing is chaotically sensitive: a 1e-10 inertia perturbation moves it by seconds; measured eruption ~8-10 s in some configs, adding ~2% rot KE). Conservation metrics here therefore describe a near-steady spin, not a tumble; MuJoCo's instability grows within the window.
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

## Lane 2 — throughput (three-tier, Go2)

| engine | tier | batch | env-steps/s | ms/control-step | status |
|---|---|---|---|---|---|
| omnisim-newton | sim_train | 256.0 | — | — | not_run |

## Lane 3 — determinism / parity / driveability

### 3a determinism

| test | engine | grade | max abs dev | first div step | steps |
|---|---|---|---|---|---|
| determinism_cold_cold | omnisim-ode | bitwise | 0 | -1.0 | 400.0 |
| determinism_cold_warm | omnisim-ode | bitwise | 0 | -1.0 | 400.0 |
| determinism_cold_cold | omnisim-newton | bitwise | 0 | -1.0 | 400.0 |
| determinism_cold_warm | omnisim-newton | bitwise | 0 | -1.0 | 400.0 |

### 3b train==deploy structural parity

| config | real physics gaps | repr diffs | pass |
|---|---|---|---|
| deploy-default (legacy COM-at-link-origin) | -1.0 | -1.0 | False |
| OMNISIM_NEWTON_USE_LINK_COM=1 (URDF link-COM parity) | -1.0 | -1.0 | False |

### 3c agent driveability

| probe | pass | latency (ms) |
|---|---|---|
| load_valid_world | False | 1.81e+03 |
| hot_reload_edited_world | False | 1.22e+03 |
| scene_tree_poses | False | 0.6 |
| scene_tree_bounds | False | 0.4 |
| sim_step_deterministic | False | 2.56e+03 |
| events_cursor_stream | True | 1.1 |
| robot_joints_state | False | 0.6 |
| scene_frame_verified | False | 0.5 |
| screenshot_png | False | 0.7 |
| broken_world_structured_diagnostic | True | 3.03e+03 |

**Score: 2.0/10.0 = 0.2** (engine omnisim-ode)

## FINDINGS (auto: sanity-threshold breaches)

- [lane1] bounce mujoco dt=1 ms: bounce_height_rmse_rel=0.1725 (> 0.1)
- [lane1] momentum mujoco dt=1 ms: linear_momentum_max=0.7994 (> 0.1)
- [lane1] bounce mujoco dt=2 ms: bounce_height_rmse_rel=0.2715 (> 0.1)
- [lane1] pendulum_energy mujoco dt=2 ms: energy_drift_rel=0.1613 (> 0.1)
- [lane1] momentum mujoco dt=2 ms: linear_momentum_max=2.361 (> 0.1)
- [lane1] momentum mujoco dt=2 ms: angular_momentum_drift_rel=0.2663 (> 0.1)
- [lane1] bounce mujoco dt=4 ms: bounce_height_rmse_rel=0.1478 (> 0.1)
- [lane1] pendulum_energy mujoco dt=4 ms: energy_drift_rel=0.3871 (> 0.1)
- [lane1] momentum mujoco dt=4 ms: linear_momentum_max=5.847 (> 0.1)
- [lane1] momentum mujoco dt=4 ms: angular_momentum_drift_rel=2.089 (> 0.1)
- [lane1] bounce mujoco dt=8 ms: bounce_height_rmse_rel=35.97 (> 0.1)
- [lane1] pendulum_energy mujoco dt=8 ms: energy_drift_rel=0.6089 (> 0.1)
- [lane1] momentum mujoco dt=8 ms: linear_momentum_max=5.667 (> 0.1)
- [lane1] bounce mujoco dt=16 ms: bounce_height_rmse_rel=33.52 (> 0.1)
- [lane1] pendulum_energy mujoco dt=16 ms: energy_drift_rel=0.4205 (> 0.1)
- [lane1] momentum mujoco dt=16 ms: linear_momentum_max=3.24e+04 (> 0.1)
- [lane1] momentum mujoco dt=16 ms: angular_momentum_drift_rel=1.23 (> 0.1)
- [lane1] stack mujoco dt=16 ms: settle_creep_m_s=0.002501 (> 0.001)
- [lane1] bounce mujoco dt=32 ms: bounce_height_rmse_rel=396.5 (> 0.1)
- [lane1] incline mujoco dt=32 ms: slide_accel_rel_err=0.3298 (> 0.1)
- [lane1] momentum mujoco dt=32 ms: linear_momentum_max=4.4e+03 (> 0.1)
- [lane1] momentum mujoco dt=32 ms: angular_momentum_drift_rel=1.174 (> 0.1)
- [lane1] stack mujoco dt=32 ms: stack_survivors=1 (< 10)
- [lane1] stack mujoco dt=32 ms: max_penetration_m=0.2979 (> 0.005)
- [lane1] incline pybullet dt=1 ms: stick_violation_max_m=0.001116 (> 0.001)
- [lane1] momentum pybullet dt=1 ms: linear_momentum_max=0.8865 (> 0.1)
- [lane1] incline pybullet dt=2 ms: stick_violation_max_m=0.001434 (> 0.001)
- [lane1] pendulum_energy pybullet dt=2 ms: energy_drift_rel=0.1585 (> 0.1)
- [lane1] momentum pybullet dt=2 ms: linear_momentum_max=2.365 (> 0.1)
- [lane1] momentum pybullet dt=2 ms: angular_momentum_drift_rel=0.2605 (> 0.1)
- [lane1] incline pybullet dt=4 ms: stick_violation_max_m=0.001952 (> 0.001)
- [lane1] pendulum_energy pybullet dt=4 ms: energy_drift_rel=0.3859 (> 0.1)
- [lane1] momentum pybullet dt=4 ms: linear_momentum_max=6.033 (> 0.1)
- [lane1] momentum pybullet dt=4 ms: angular_momentum_drift_rel=2.198 (> 0.1)
- [lane1] bounce pybullet dt=8 ms: bounce_height_rmse_rel=0.1606 (> 0.1)
- [lane1] incline pybullet dt=8 ms: stick_violation_max_m=0.001349 (> 0.001)
- [lane1] pendulum_energy pybullet dt=8 ms: energy_drift_rel=0.6113 (> 0.1)
- [lane1] momentum pybullet dt=8 ms: linear_momentum_max=5.951 (> 0.1)
- [lane1] momentum pybullet dt=8 ms: angular_momentum_drift_rel=0.1703 (> 0.1)
- [lane1] stack pybullet dt=8 ms: stack_survivors=1 (< 10)
- [lane1] stack pybullet dt=8 ms: max_penetration_m=0.2999 (> 0.005)
- [lane1] bounce pybullet dt=16 ms: bounce_height_rmse_rel=0.3497 (> 0.1)
- [lane1] incline pybullet dt=16 ms: stick_violation_max_m=0.001377 (> 0.001)
- [lane1] pendulum_energy pybullet dt=16 ms: energy_drift_rel=0.4056 (> 0.1)
- [lane1] momentum pybullet dt=16 ms: linear_momentum_max=355.4 (> 0.1)
- [lane1] momentum pybullet dt=16 ms: angular_momentum_drift_rel=34.45 (> 0.1)
- [lane1] stack pybullet dt=16 ms: stack_survivors=1 (< 10)
- [lane1] stack pybullet dt=16 ms: max_penetration_m=0.2997 (> 0.005)
- [lane1] bounce pybullet dt=32 ms: bounce_height_rmse_rel=0.7752 (> 0.1)
- [lane1] incline pybullet dt=32 ms: stick_violation_max_m=0.001457 (> 0.001)
- [lane1] pendulum_energy pybullet dt=32 ms: energy_drift_rel=0.9992 (> 0.1)
- [lane1] momentum pybullet dt=32 ms: linear_momentum_max=252.6 (> 0.1)
- [lane1] momentum pybullet dt=32 ms: angular_momentum_drift_rel=47.33 (> 0.1)
- [lane1] stack pybullet dt=32 ms: stack_survivors=0 (< 10)
- [lane1] stack pybullet dt=32 ms: max_penetration_m=0.1 (> 0.005)
- [lane1] pendulum_energy omnisim-ode dt=4 ms: energy_drift_rel=0.13 (> 0.1)
- [lane1] pendulum_energy omnisim-ode dt=8 ms: energy_drift_rel=0.3664 (> 0.1)
- [lane1] pendulum_energy omnisim-ode dt=16 ms: energy_drift_rel=0.5224 (> 0.1)
- [lane1] pendulum_energy omnisim-ode dt=32 ms: energy_drift_rel=0.7755 (> 0.1)
- [lane1] momentum omnisim-ode dt=1 ms: angular_momentum_drift_rel=6.691 (> 0.1)
- [lane1] momentum omnisim-ode dt=2 ms: angular_momentum_drift_rel=7.988 (> 0.1)
- [lane1] momentum omnisim-ode dt=4 ms: angular_momentum_drift_rel=0.3955 (> 0.1)
- [lane1] momentum omnisim-ode dt=8 ms: angular_momentum_drift_rel=1.442 (> 0.1)
- [lane1] momentum omnisim-ode dt=16 ms: angular_momentum_drift_rel=1.058 (> 0.1)
- [lane1] momentum omnisim-ode dt=32 ms: angular_momentum_drift_rel=0.2077 (> 0.1)
- [lane1] stack omnisim-ode dt=32 ms: stack_survivors=1 (< 10)
- [lane1] stack omnisim-ode dt=32 ms: settle_creep_m_s=0.26 (> 0.001)
- [lane1] stack omnisim-ode dt=32 ms: max_penetration_m=0.1996 (> 0.005)
- [lane1] bounce omnisim-newton dt=16 ms: bounce_height_rmse_rel=0.2167 (> 0.1)
- [lane1] bounce omnisim-newton dt=32 ms: bounce_height_rmse_rel=0.4986 (> 0.1)
- [lane1] incline omnisim-newton dt=1 ms: stick_violation_max_m=0.196 (> 0.001)
- [lane1] incline omnisim-newton dt=1 ms: slide_accel_rel_err=0.2072 (> 0.1)
- [lane1] incline omnisim-newton dt=1 ms: transition_angle_err_deg=4.065 (> 1)
- [lane1] incline omnisim-newton dt=2 ms: stick_violation_max_m=0.2112 (> 0.001)
- [lane1] incline omnisim-newton dt=2 ms: slide_accel_rel_err=0.2252 (> 0.1)
- [lane1] incline omnisim-newton dt=2 ms: transition_angle_err_deg=4.065 (> 1)
- [lane1] incline omnisim-newton dt=4 ms: stick_violation_max_m=0.1806 (> 0.001)
- [lane1] incline omnisim-newton dt=4 ms: slide_accel_rel_err=0.1908 (> 0.1)
- [lane1] incline omnisim-newton dt=4 ms: transition_angle_err_deg=4.065 (> 1)
- [lane1] incline omnisim-newton dt=8 ms: stick_violation_max_m=0.2326 (> 0.001)
- [lane1] incline omnisim-newton dt=8 ms: slide_accel_rel_err=0.2538 (> 0.1)
- [lane1] incline omnisim-newton dt=8 ms: transition_angle_err_deg=4.065 (> 1)
- [lane1] incline omnisim-newton dt=16 ms: stick_violation_max_m=0.2734 (> 0.001)
- [lane1] incline omnisim-newton dt=16 ms: slide_accel_rel_err=0.3152 (> 0.1)
- [lane1] incline omnisim-newton dt=16 ms: transition_angle_err_deg=4.065 (> 1)
- [lane1] incline omnisim-newton dt=32 ms: stick_violation_max_m=0.6741 (> 0.001)
- [lane1] incline omnisim-newton dt=32 ms: transition_angle_err_deg=9.065 (> 1)
- [lane1] roll omnisim-newton dt=1 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=2 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=4 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=8 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=16 ms: roll_accel_rel_err=0.477 (> 0.1)
- [lane1] roll omnisim-newton dt=32 ms: roll_accel_rel_err=0.4786 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=1 ms: energy_drift_rel=0.2742 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=2 ms: energy_drift_rel=0.4825 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=4 ms: energy_drift_rel=0.4974 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=8 ms: energy_drift_rel=0.4621 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=16 ms: energy_drift_rel=0.2779 (> 0.1)
- [lane1] momentum omnisim-newton dt=1 ms: linear_momentum_max=17.18 (> 0.1)
- [lane1] momentum omnisim-newton dt=1 ms: angular_momentum_drift_rel=0.9999 (> 0.1)
- [lane1] momentum omnisim-newton dt=2 ms: linear_momentum_max=17.46 (> 0.1)
- [lane1] momentum omnisim-newton dt=2 ms: angular_momentum_drift_rel=1 (> 0.1)
- [lane1] momentum omnisim-newton dt=4 ms: linear_momentum_max=18.69 (> 0.1)
- [lane1] momentum omnisim-newton dt=4 ms: angular_momentum_drift_rel=0.9999 (> 0.1)
- [lane1] momentum omnisim-newton dt=8 ms: linear_momentum_max=21.32 (> 0.1)
- [lane1] momentum omnisim-newton dt=8 ms: angular_momentum_drift_rel=1 (> 0.1)
- [lane1] momentum omnisim-newton dt=16 ms: linear_momentum_max=24.26 (> 0.1)
- [lane1] momentum omnisim-newton dt=16 ms: angular_momentum_drift_rel=1.76 (> 0.1)
- [lane1] momentum omnisim-newton dt=32 ms: linear_momentum_max=17.34 (> 0.1)
- [lane1] momentum omnisim-newton dt=32 ms: angular_momentum_drift_rel=0.9998 (> 0.1)
- [lane1] stack omnisim-newton dt=4 ms: settle_creep_m_s=0.001147 (> 0.001)
- [lane1] stack omnisim-newton dt=8 ms: stack_survivors=1 (< 10)
- [lane1] stack omnisim-newton dt=8 ms: max_penetration_m=0.4989 (> 0.005)
- [lane1] stack omnisim-newton dt=16 ms: stack_survivors=2 (< 10)
- [lane1] stack omnisim-newton dt=16 ms: settle_creep_m_s=0.1015 (> 0.001)
- [lane1] stack omnisim-newton dt=16 ms: max_penetration_m=0.02593 (> 0.005)
- [lane1] stack omnisim-newton dt=32 ms: stack_survivors=1 (< 10)
- [lane1] stack omnisim-newton dt=32 ms: max_penetration_m=0.2981 (> 0.005)
- [lane1] spin omnisim-newton dt=1 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=1 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=2 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=2 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=4 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=4 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=8 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=8 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=16 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=16 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=32 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=32 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane2] omnisim-newton tier=sim_train batch=256.0: status=not_run (in-engine trainer produced no env-steps/s window samples; started=False rc=0; see _scratch/foot_redesign/lane2c_0724_200805_console.txt)
- [lane3] parity_structural (config: deploy-default (legacy COM-at-link-origin)): pass=False real_physics_gaps=-1.0
- [lane3] parity_structural (config: OMNISIM_NEWTON_USE_LINK_COM=1 (URDF link-COM parity)): pass=False real_physics_gaps=-1.0
- [lane3] driveability_load_valid_world: FAIL (latency 1.81e+03 ms)
- [lane3] driveability_hot_reload_edited_world: FAIL (latency 1.22e+03 ms)
- [lane3] driveability_scene_tree_poses: FAIL (latency 0.6 ms)
- [lane3] driveability_scene_tree_bounds: FAIL (latency 0.4 ms)
- [lane3] driveability_sim_step_deterministic: FAIL (latency 2.56e+03 ms)
- [lane3] driveability_robot_joints_state: FAIL (latency 0.6 ms)
- [lane3] driveability_scene_frame_verified: FAIL (latency 0.5 ms)
- [lane3] driveability_screenshot_png: FAIL (latency 0.7 ms)
- [lane3] driveability: 2.0/10.0 probes passed

## Gaps (recorded in MANIFEST.json)

- none
