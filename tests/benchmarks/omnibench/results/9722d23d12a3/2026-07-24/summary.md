# OmniBench summary — 9722d23d12a3

Generated 2026-07-24T18:08:50Z by run_all.py (suite omnibench/v0).

| machine | value |
|---|---|
| machine id | 9722d23d12a3 |
| host | machine-9722d23d12a3 |
| GPU | NVIDIA GeForce RTX 3060 Laptop GPU (driver 596.36) |
| CPU | AMD64 Family 25 Model 80 Stepping 0, AuthenticAMD (16 cores) |
| OS | Windows-11 |
| python | 3.12.9 (numpy 2.4.4) |
| engine binary sha256 | 5087587d3e4b5940 |

All numbers below are attributable to THIS machine only (SPEC honesty rule 4).

## Lane 1 — physics correctness (analytic ground truth)

### T1 `bounce`

| engine | dt (ms) | bounce_height_rmse_rel | wall ms/step |
|---|---|---|---|
| mujoco | 1 | 0.1725 | 0.003114 |
| mujoco | 2 | 0.2715 | 0.003066 |
| mujoco | 4 | 0.1478 | 0.002765 |
| mujoco | 8 | 35.97 | 0.002755 |
| mujoco | 16 | 33.52 | 0.00515 |
| mujoco | 32 | 396.5 | 0.003267 |
| omnisim-newton | 1 | 0.01028 | 2.335 |
| omnisim-newton | 2 | 0.01769 | 2.759 |
| omnisim-newton | 4 | 0.03811 | 2.985 |
| omnisim-newton | 8 | 0.08393 | 4.266 |
| omnisim-newton | 16 | 0.2167 | 7.253 |
| omnisim-newton | 32 | 0.4986 | 12.2 |
| omnisim-ode | 1 | 0.01872 | 0.7493 |
| omnisim-ode | 2 | 0.01785 | 1.597 |
| omnisim-ode | 4 | 0.01788 | 2.854 |
| omnisim-ode | 8 | 0.01295 | 6.84 |
| omnisim-ode | 16 | 0.08463 | 9.989 |
| omnisim-ode | 32 | 0.07143 | 21.72 |
| pybullet | 1 | 0.01706 | 0.00396 |
| pybullet | 2 | 0.03924 | 0.003872 |
| pybullet | 4 | 0.07337 | 0.004151 |
| pybullet | 8 | 0.1606 | 0.003959 |
| pybullet | 16 | 0.3497 | 0.005137 |
| pybullet | 32 | 0.7752 | 0.004714 |

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
| mujoco | 1 | 0.000442 | 0.008854 | 0.06505 | 0.04092 |
| mujoco | 2 | 0.000442 | 0.01089 | 0.06505 | 0.03574 |
| mujoco | 4 | 0.000442 | 0.006183 | 0.06505 | 0.03875 |
| mujoco | 8 | 0.000442 | 0.01286 | 0.06505 | 0.03427 |
| mujoco | 16 | 0.000639 | 0.04927 | 0.06505 | 0.03811 |
| mujoco | 32 | 0.000769 | 0.3298 | 0.06505 | 0.03917 |
| omnisim-newton | 1 | 0.196 | 0.2072 | 4.065 | 2.223 |
| omnisim-newton | 2 | 0.2112 | 0.2252 | 4.065 | 2.717 |
| omnisim-newton | 4 | 0.1806 | 0.1908 | 4.065 | 4.35 |
| omnisim-newton | 8 | 0.2326 | 0.2538 | 4.065 | 7.021 |
| omnisim-newton | 16 | 0.2734 | 0.3152 | 4.065 | 10.77 |
| omnisim-newton | 32 | 0.6741 | 0.006539 | 9.065 | 20.64 |
| omnisim-ode | 1 | 3.04e-05 | 9.53e-05 | 0.06505 | 1.46 |
| omnisim-ode | 2 | 2.94e-05 | 0.000114 | 0.06505 | 2.571 |
| omnisim-ode | 4 | 2.9e-05 | 8.78e-05 | 0.06505 | 4.557 |
| omnisim-ode | 8 | 2.91e-05 | 0.000142 | 0.06505 | 9.08 |
| omnisim-ode | 16 | 3.02e-05 | 0.007449 | 0.06505 | 18.84 |
| omnisim-ode | 32 | 5.22e-05 | 0.0633 | 0.06505 | 40.47 |
| pybullet | 1 | 0.001116 | 8.42e-07 | 0.06505 | 0.06206 |
| pybullet | 2 | 0.001434 | 2.92e-05 | 0.06505 | 0.05757 |
| pybullet | 4 | 0.001952 | 6.56e-05 | 0.06505 | 0.06563 |
| pybullet | 8 | 0.001349 | 0.001481 | 0.06505 | 0.08222 |
| pybullet | 16 | 0.001377 | 0.001672 | 0.06505 | 0.07655 |
| pybullet | 32 | 0.001457 | 0.007891 | 0.06505 | 0.09746 |

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
| mujoco | 1 | 0.001004 | 0.00172 | 0.0056 |
| mujoco | 2 | 0.001004 | 0.00172 | 0.004979 |
| mujoco | 4 | 0.001003 | 0.00172 | 0.004935 |
| mujoco | 8 | 0.001003 | 0.001719 | 0.005 |
| mujoco | 16 | 0.00177 | 0.003086 | 0.005043 |
| mujoco | 32 | 0.005122 | 0.009473 | 0.005185 |
| omnisim-newton | 1 | 0.4763 | 0.001737 | 2.097 |
| omnisim-newton | 2 | 0.4763 | 0.001737 | 2.66 |
| omnisim-newton | 4 | 0.4763 | 0.001737 | 4.55 |
| omnisim-newton | 8 | 0.4763 | 0.001736 | 7.616 |
| omnisim-newton | 16 | 0.477 | 0.001574 | 13.42 |
| omnisim-newton | 32 | 0.4786 | 0.000499 | 22.58 |
| omnisim-ode | 1 | 0.000263 | 0.000457 | 1.58 |
| omnisim-ode | 2 | 0.000527 | 0.000919 | 3.346 |
| omnisim-ode | 4 | 0.001055 | 0.001843 | 7.333 |
| omnisim-ode | 8 | 0.002115 | 0.003697 | 12.86 |
| omnisim-ode | 16 | 0.004192 | 0.007425 | 25.75 |
| omnisim-ode | 32 | 0.008016 | 0.01509 | 49.4 |
| pybullet | 1 | 1.48e-15 | 2.04e-15 | 0.0064 |
| pybullet | 2 | 7.41e-16 | 1.11e-15 | 0.006605 |
| pybullet | 4 | 2.04e-15 | 3.89e-15 | 0.00677 |
| pybullet | 8 | 1.04e-14 | 9.27e-16 | 0.006562 |
| pybullet | 16 | 1.17e-14 | 1.12e-15 | 0.007216 |
| pybullet | 32 | 1.09e-14 | 5.6e-15 | 0.006677 |

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
| mujoco | 1 | 0.09359 | -0.008973 | 0.003491 |
| mujoco | 2 | 0.1613 | -0.02263 | 0.003104 |
| mujoco | 4 | 0.3871 | -0.04797 | 0.003621 |
| mujoco | 8 | 0.6089 | -0.09184 | 0.003405 |
| mujoco | 16 | 0.4205 | -0.07004 | 0.003203 |
| mujoco | 32 | 1.55e-08 | 0.001028 | 0.03176 |
| omnisim-newton | 1 | 0.2742 | -0.03249 | 2.009 |
| omnisim-newton | 2 | 0.4823 | -0.05923 | 2.477 |
| omnisim-newton | 4 | 0.4974 | -0.06084 | 2.788 |
| omnisim-newton | 8 | 0.4621 | -0.03718 | 2.918 |
| omnisim-newton | 16 | 0.2779 | 0.01773 | 5.012 |
| omnisim-newton | 32 | 7.26e-09 | 0.000514 | 8.262 |
| omnisim-ode | 1 | 0.009001 | -0.000737 | 0.631 |
| omnisim-ode | 2 | 0.04631 | -0.005271 | 1.042 |
| omnisim-ode | 4 | 0.13 | -0.01292 | 2.227 |
| omnisim-ode | 8 | 0.3664 | -0.03581 | 3.413 |
| omnisim-ode | 16 | 0.5224 | -0.04737 | 6.156 |
| omnisim-ode | 32 | 0.7755 | -0.06037 | 13.88 |
| pybullet | 1 | 0.06948 | -0.008869 | 0.006033 |
| pybullet | 2 | 0.1585 | -0.02265 | 0.006257 |
| pybullet | 4 | 0.3859 | -0.04806 | 0.007131 |
| pybullet | 8 | 0.6113 | -0.09202 | 0.006115 |
| pybullet | 16 | 0.4056 | -0.07025 | 0.005872 |
| pybullet | 32 | 0.9992 | 0.09548 | 0.006042 |

<details><summary>deviations (pendulum_energy)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.8.1
  - energy_drift normalized by peak KE (22.401 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (22.815 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (25.696 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (27.697 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (589062700.450 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - energy_drift normalized by peak KE (24.468 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (22.399 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (28.397 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (29.405 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (2171380848.559 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.197 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
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
| mujoco | 1 | 0.7994 | 0.002611 | 0.004058 |
| mujoco | 2 | 2.361 | 0.2663 | 0.003923 |
| mujoco | 4 | 5.847 | 2.09 | 0.003575 |
| mujoco | 8 | 5.667 | 0.07964 | 0.00421 |
| mujoco | 16 | 3.37e+04 | 1.23 | 0.005994 |
| mujoco | 32 | 4.4e+03 | 1.174 | 0.03333 |
| omnisim-newton | 1 | 17.18 | 0.9999 | 2.595 |
| omnisim-newton | 2 | 17.46 | 1 | 3.254 |
| omnisim-newton | 4 | 18.69 | 0.9999 | 3.67 |
| omnisim-newton | 8 | 21.32 | 1 | 3.967 |
| omnisim-newton | 16 | 24.26 | 1 | 5.826 |
| omnisim-newton | 32 | 17.34 | 0.9998 | 9.504 |
| omnisim-ode | 1 | 3.66e-14 | 4.738 | 0.7003 |
| omnisim-ode | 2 | 1.45e-14 | 6.674 | 0.9836 |
| omnisim-ode | 4 | 9.78e-15 | 0.05906 | 1.779 |
| omnisim-ode | 8 | 4.42e-15 | 1.442 | 3.243 |
| omnisim-ode | 16 | 3.85e-15 | 1.058 | 6.708 |
| omnisim-ode | 32 | 1.62e-15 | 0.2077 | 11.89 |
| pybullet | 1 | 0.8865 | 0.0002 | 0.005112 |
| pybullet | 2 | 2.365 | 0.2605 | 0.005546 |
| pybullet | 4 | 6.033 | 2.198 | 0.005911 |
| pybullet | 8 | 5.951 | 0.1703 | 0.00526 |
| pybullet | 16 | 182.5 | 19.48 | 0.006034 |
| pybullet | 32 | 252.5 | 15.46 | 0.005387 |

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
| mujoco | 1 | 10 | 1.22e-07 | 0.000618 | 0.07475 |
| mujoco | 2 | 10 | 1.46e-06 | 0.000618 | 0.08199 |
| mujoco | 4 | 10 | 2.35e-06 | 0.000618 | 0.07733 |
| mujoco | 8 | 10 | 6.14e-06 | 0.000618 | 0.07153 |
| mujoco | 16 | 10 | 0.002501 | 0.001188 | 0.07664 |
| mujoco | 32 | 1 | 1.18e-08 | 0.2979 | 0.06921 |
| omnisim-newton | 1 | 10 | 0.00067 | 0.000618 | 2.735 |
| omnisim-newton | 2 | 10 | 0.000204 | 0.000579 | 2.403 |
| omnisim-newton | 4 | 10 | 0.001147 | 0.000617 | 3.075 |
| omnisim-newton | 8 | 1 | 0 | 0.4989 | 3.7 |
| omnisim-newton | 16 | 2 | 0.1015 | 0.02593 | 5.354 |
| omnisim-newton | 32 | 1 | 0 | 0.2981 | 8.186 |
| omnisim-ode | 1 | 10 | 1.67e-09 | 6.13e-05 | 1.116 |
| omnisim-ode | 2 | 10 | 1.3e-09 | 0.000123 | 1.516 |
| omnisim-ode | 4 | 10 | 5.21e-08 | 0.000245 | 2.098 |
| omnisim-ode | 8 | 10 | 2.12e-07 | 0.000491 | 3.937 |
| omnisim-ode | 16 | 10 | 6.22e-08 | 0.000981 | 6.155 |
| omnisim-ode | 32 | 1 | 0.26 | 0.1996 | 14.47 |
| pybullet | 1 | 10 | 8.45e-05 | 3.2e-05 | 0.2628 |
| pybullet | 2 | 10 | 9.44e-05 | 9.69e-05 | 0.2675 |
| pybullet | 4 | 10 | 0.000187 | 0.000356 | 0.2611 |
| pybullet | 8 | 2 | 0.05631 | 0.006949 | 0.2785 |
| pybullet | 16 | 1 | 2.04e-06 | 0.2997 | 0.1628 |
| pybullet | 32 | 0 | 9.86e-07 | 0.1 | 0.1377 |

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
| mujoco | 1 | 0.001691 | 0.001418 | 0.002883 |
| mujoco | 2 | 0.003299 | 0.002637 | 0.003186 |
| mujoco | 4 | 0.00625 | 0.004537 | 0.002528 |
| mujoco | 8 | 0.01103 | 0.006613 | 0.002554 |
| mujoco | 16 | 0.01645 | 0.006764 | 0.002702 |
| mujoco | 32 | 0.01631 | 0.00312 | 0.002558 |
| omnisim-newton | 1 | 1 | 1 | 1.523 |
| omnisim-newton | 2 | 1 | 1 | 1.759 |
| omnisim-newton | 4 | 1 | 1 | 1.358 |
| omnisim-newton | 8 | 1 | 1 | 1.496 |
| omnisim-newton | 16 | 1 | 1 | 1.796 |
| omnisim-newton | 32 | 1 | 1 | 1.233 |
| omnisim-ode | 1 | 4.86e-06 | 8.78e-09 | 0.1544 |
| omnisim-ode | 2 | 9.71e-06 | 1.75e-08 | 0.17 |
| omnisim-ode | 4 | 1.93e-05 | 3.47e-08 | 0.1447 |
| omnisim-ode | 8 | 3.84e-05 | 6.81e-08 | 0.1482 |
| omnisim-ode | 16 | 7.55e-05 | 1.31e-07 | 0.1542 |
| omnisim-ode | 32 | 0.000146 | 2.42e-07 | 0.1218 |
| pybullet | 1 | 4.88e-06 | 8.84e-09 | 0.002483 |
| pybullet | 2 | 9.76e-06 | 1.77e-08 | 0.002705 |
| pybullet | 4 | 1.95e-05 | 3.55e-08 | 0.002659 |
| pybullet | 8 | 3.92e-05 | 7.14e-08 | 0.002387 |
| pybullet | 16 | 7.88e-05 | 1.45e-07 | 0.002502 |
| pybullet | 32 | 0.00016 | 2.97e-07 | 0.003029 |

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
| mujoco-warp-raw | sim_only | 256.0 | 4.57e+04 | 5.604 | ok |
| omnisim-newton | sim_only | 256.0 | 2.63e+03 | 97.29 | ok |
| mujoco-warp-raw | sim_infer | 256.0 | 3.97e+04 | 6.451 | ok |
| mujoco-warp-raw | sim_only | 1024.0 | 1.21e+05 | 8.492 | ok |
| omnisim-newton | sim_only | 1024.0 | 1.01e+04 | 101.2 | ok |
| mujoco-warp-raw | sim_infer | 1024.0 | 1.05e+05 | 9.767 | ok |
| mujoco-warp-raw | sim_only | 4096.0 | 1.66e+05 | 24.73 | ok |
| omnisim-newton | sim_only | 4096.0 | 3.7e+04 | 110.8 | ok |
| mujoco-warp-raw | sim_infer | 4096.0 | 1.45e+05 | 28.17 | ok |
| omnisim-newton | sim_train | 256.0 | — | — | not_run |
| omnisim-newton | sim_train | 256.0 | 1.02e+04 | — | ok |

OmniSim embedded-solver / raw mujoco-warp sim_only ratio (same batch): batch 256.0: 17.4x, batch 1024.0: 11.9x, batch 4096.0: 4.48x

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
| deploy-default (legacy COM-at-link-origin) | 1.0 | 3.0 | False |
| OMNISIM_NEWTON_USE_LINK_COM=1 (URDF link-COM parity) | 0.0 | 3.0 | True |

### 3c agent driveability

| probe | pass | latency (ms) |
|---|---|---|
| load_valid_world | True | 8.79e+03 |
| hot_reload_edited_world | True | 3.99e+03 |
| scene_tree_poses | True | 1.42e+03 |
| scene_tree_bounds | True | 2.5e+03 |
| sim_step_deterministic | True | 7.62e+04 |
| events_cursor_stream | True | 817.6 |
| robot_joints_state | True | 967.8 |
| scene_frame_verified | True | 3.85e+03 |
| screenshot_png | True | 1.02e+03 |
| broken_world_structured_diagnostic | True | 2.43e+05 |

**Score: 10.0/10.0 = 1.0** (engine omnisim-newton)

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
- [lane1] momentum mujoco dt=4 ms: angular_momentum_drift_rel=2.09 (> 0.1)
- [lane1] bounce mujoco dt=8 ms: bounce_height_rmse_rel=35.97 (> 0.1)
- [lane1] pendulum_energy mujoco dt=8 ms: energy_drift_rel=0.6089 (> 0.1)
- [lane1] momentum mujoco dt=8 ms: linear_momentum_max=5.667 (> 0.1)
- [lane1] bounce mujoco dt=16 ms: bounce_height_rmse_rel=33.52 (> 0.1)
- [lane1] pendulum_energy mujoco dt=16 ms: energy_drift_rel=0.4205 (> 0.1)
- [lane1] momentum mujoco dt=16 ms: linear_momentum_max=3.37e+04 (> 0.1)
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
- [lane1] stack pybullet dt=8 ms: stack_survivors=2 (< 10)
- [lane1] stack pybullet dt=8 ms: settle_creep_m_s=0.05631 (> 0.001)
- [lane1] stack pybullet dt=8 ms: max_penetration_m=0.006949 (> 0.005)
- [lane1] bounce pybullet dt=16 ms: bounce_height_rmse_rel=0.3497 (> 0.1)
- [lane1] incline pybullet dt=16 ms: stick_violation_max_m=0.001377 (> 0.001)
- [lane1] pendulum_energy pybullet dt=16 ms: energy_drift_rel=0.4056 (> 0.1)
- [lane1] momentum pybullet dt=16 ms: linear_momentum_max=182.5 (> 0.1)
- [lane1] momentum pybullet dt=16 ms: angular_momentum_drift_rel=19.48 (> 0.1)
- [lane1] stack pybullet dt=16 ms: stack_survivors=1 (< 10)
- [lane1] stack pybullet dt=16 ms: max_penetration_m=0.2997 (> 0.005)
- [lane1] bounce pybullet dt=32 ms: bounce_height_rmse_rel=0.7752 (> 0.1)
- [lane1] incline pybullet dt=32 ms: stick_violation_max_m=0.001457 (> 0.001)
- [lane1] pendulum_energy pybullet dt=32 ms: energy_drift_rel=0.9992 (> 0.1)
- [lane1] momentum pybullet dt=32 ms: linear_momentum_max=252.5 (> 0.1)
- [lane1] momentum pybullet dt=32 ms: angular_momentum_drift_rel=15.46 (> 0.1)
- [lane1] stack pybullet dt=32 ms: stack_survivors=0 (< 10)
- [lane1] stack pybullet dt=32 ms: max_penetration_m=0.1 (> 0.005)
- [lane1] pendulum_energy omnisim-ode dt=4 ms: energy_drift_rel=0.13 (> 0.1)
- [lane1] pendulum_energy omnisim-ode dt=8 ms: energy_drift_rel=0.3664 (> 0.1)
- [lane1] pendulum_energy omnisim-ode dt=16 ms: energy_drift_rel=0.5224 (> 0.1)
- [lane1] pendulum_energy omnisim-ode dt=32 ms: energy_drift_rel=0.7755 (> 0.1)
- [lane1] momentum omnisim-ode dt=1 ms: angular_momentum_drift_rel=4.738 (> 0.1)
- [lane1] momentum omnisim-ode dt=2 ms: angular_momentum_drift_rel=6.674 (> 0.1)
- [lane1] momentum omnisim-ode dt=16 ms: angular_momentum_drift_rel=1.058 (> 0.1)
- [lane1] momentum omnisim-ode dt=32 ms: angular_momentum_drift_rel=0.2077 (> 0.1)
- [lane1] stack omnisim-ode dt=32 ms: stack_survivors=1 (< 10)
- [lane1] stack omnisim-ode dt=32 ms: settle_creep_m_s=0.26 (> 0.001)
- [lane1] stack omnisim-ode dt=32 ms: max_penetration_m=0.1996 (> 0.005)
- [lane1] bounce omnisim-newton dt=16 ms: bounce_height_rmse_rel=0.2167 (> 0.1)
- [lane1] roll omnisim-newton dt=1 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=2 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=4 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=8 ms: roll_accel_rel_err=0.4763 (> 0.1)
- [lane1] roll omnisim-newton dt=16 ms: roll_accel_rel_err=0.477 (> 0.1)
- [lane1] roll omnisim-newton dt=32 ms: roll_accel_rel_err=0.4786 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=1 ms: energy_drift_rel=0.2742 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=2 ms: energy_drift_rel=0.4823 (> 0.1)
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
- [lane1] momentum omnisim-newton dt=16 ms: angular_momentum_drift_rel=1 (> 0.1)
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
- [lane1] spin omnisim-newton dt=4 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=4 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=8 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=8 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=16 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=16 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=32 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=32 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane1] momentum omnisim-ode dt=8 ms: angular_momentum_drift_rel=1.442 (> 0.1)
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
- [lane1] pendulum_energy omnisim-newton dt=4 ms: energy_drift_rel=0.4974 (> 0.1)
- [lane1] spin omnisim-newton dt=2 ms: angmom_drift_rel=1 (> 0.1)
- [lane1] spin omnisim-newton dt=2 ms: rot_ke_drift_rel=1 (> 0.1)
- [lane2] omnisim-newton tier=sim_train batch=256.0: status=not_run (in-engine trainer produced no env-steps/s window samples; started=False rc=0; see _scratch\foot_redesign\lane2c_0724_170728_console.txt)
- [lane3] parity_structural (config: deploy-default (legacy COM-at-link-origin)): pass=False real_physics_gaps=1.0

## Gaps (recorded in MANIFEST.json)

- none
