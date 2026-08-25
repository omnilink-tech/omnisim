# OmniBench summary — b5dadd645b1f

Generated 2026-08-17T08:25:16Z by run_all.py (suite omnibench/v0).

| machine | value |
|---|---|
| machine id | b5dadd645b1f |
| host | hfa5b7c0126 |
| GPU | NVIDIA GeForce RTX 4090 (driver 570.195.03) |
| CPU | x86_64 (32 cores) |
| OS | Linux-6.8.0-88-generic-x86_64-with-glibc2.35 |
| python | 3.11.10 (numpy 2.4.6) |
| engine binary sha256 | 6f7e2217426a2088 |

All numbers below are attributable to THIS machine only (SPEC honesty rule 4).

## Lane 1 — physics correctness (analytic ground truth)

### T1 `bounce`

| engine | dt (ms) | bounce_height_rmse_rel | wall ms/step |
|---|---|---|---|
| mujoco | 1 | 0.1725 | 0.001402 |
| mujoco | 2 | 0.2715 | 0.001301 |
| mujoco | 4 | 0.1478 | 0.001242 |
| mujoco | 8 | 35.97 | 0.001204 |
| mujoco | 16 | 33.52 | 0.001191 |
| mujoco | 32 | 396.5 | 0.001226 |
| omnisim-newton | 1 | 0.01042 | 0.225 |
| omnisim-newton | 2 | 0.01773 | 0.2389 |
| omnisim-newton | 4 | 0.03812 | 0.284 |
| omnisim-newton | 8 | 0.1177 | 0.5022 |
| omnisim-newton | 16 | 0.2264 | 0.598 |
| omnisim-newton | 32 | 2.04 | 1.677 |
| pybullet | 1 | 0.01706 | 0.000904 |
| pybullet | 2 | 0.03924 | 0.000904 |
| pybullet | 4 | 0.07337 | 0.000962 |
| pybullet | 8 | 0.1606 | 0.000984 |
| pybullet | 16 | 0.3497 | 0.00106 |
| pybullet | 32 | 0.7752 | 0.0012 |

<details><summary>deviations (bounce)</summary>

- **mujoco**:
  - mujoco has no restitution coeff; direct solref=(-100000,-54.4678) calibrated at dt=1ms -> first peak 0.6400 m (target 0.6400)
  - solimp=(0.95,0.95,0.001) constant impedance => linear contact (speed-independent e; converges to analytic peaks at dt=0.25ms)
  - fixed contact stiffness k=100000 has sqrt(k/m)*dt>2 for dt>=8ms: contact integration unstable there (energy GAIN), which is the honest sweep result of freezing the dt=1ms mapping
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
  - only 3/5 bounce peaks detected; missing peaks scored as h=0
  - only 1/5 bounce peaks detected; missing peaks scored as h=0
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - HISTORICAL: this scene once declared ContactProperties softCFM=1e-06 to stop the engine default 0.001 mixing into the ODE bounce constraint and killing restitution (measured first peak 0.02 m at dt=1 ms vs 0.64 analytic). ODE was deleted (bdc02139) and the Newton backend never read the field; the world no longer declares it, and restitution now comes from newtonContactKd
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.0 (ContactProperties.coulombFriction=0 is an ODE-path field the Newton backend ignores)
  - Newton has no restitution coefficient; e=0.8 realised via contact-compliance calibration OMNISIM_NEWTON_CONTACT_KD=7 (ke=2500 default; damped-spring zeta=0.070 -> e~0.80, calibrated at dt=1 ms per SPEC; engine-default kd=100 gives e~0). Soft contact penetrates ~0.08 m at impact — recorded, not hidden
- **pybullet**:
  - bullet restitution combines multiplicatively: plane e=1.0, ball e=0.8 (set directly); restitutionVelocityThreshold default
  - override: contactProcessingThreshold=0 (default persistence kills bounces after the 1st) and erp=contactERP=0 (ERP push-out otherwise adds ~+0.35 m/s per impact, e_eff~0.88)
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

### T2 `incline`

| engine | dt (ms) | stick_violation_max_m | slide_accel_rel_err | transition_angle_err_deg | wall ms/step |
|---|---|---|---|---|---|
| mujoco | 1 | 0.000442 | 0.008854 | 0.06505 | 0.01289 |
| mujoco | 2 | 0.000442 | 0.01089 | 0.06505 | 0.01327 |
| mujoco | 4 | 0.000442 | 0.006183 | 0.06505 | 0.01442 |
| mujoco | 8 | 0.000442 | 0.01286 | 0.06505 | 0.01473 |
| mujoco | 16 | 0.000639 | 0.04927 | 0.06505 | 0.01806 |
| mujoco | 32 | 0.000769 | 0.3298 | 0.06505 | 0.01769 |
| omnisim-newton | 1 | 0.000442 | 0.5599 | 0.06505 | 0.2492 |
| omnisim-newton | 2 | 0.000442 | 0.08024 | 0.06505 | 0.2832 |
| omnisim-newton | 4 | 0.000442 | 0.1059 | 0.06505 | 0.3899 |
| omnisim-newton | 8 | 0.000443 | 0.02593 | 0.06505 | 0.6449 |
| omnisim-newton | 16 | 0.000645 | 0.05811 | 0.06505 | 1.028 |
| omnisim-newton | 32 | 0.001014 | 0.3485 | 0.06505 | 1.827 |
| pybullet | 1 | 0.001116 | 8.42e-07 | 0.06505 | 0.02318 |
| pybullet | 2 | 0.001434 | 2.92e-05 | 0.06505 | 0.02407 |
| pybullet | 4 | 0.001952 | 6.56e-05 | 0.06505 | 0.02766 |
| pybullet | 8 | 0.001349 | 0.001151 | 0.06505 | 0.03048 |
| pybullet | 16 | 0.001377 | 0.001672 | 0.06505 | 0.03237 |
| pybullet | 32 | 0.001457 | 0.007891 | 0.06505 | 0.03716 |

<details><summary>deviations (incline)</summary>

- **mujoco**:
  - override: cone=elliptic impratio=10 (default pyramidal cone creeps ~5mm/3s at 15-26 deg; with override sub-0.5mm stick below theta_c)
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
  - stick displacement measured from t=0.3s (skips the soft-contact settle); slide accel = linear fit of tangential speed over t=[0.1,1.1]s
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.5 (ContactProperties.coulombFriction=0.5 is an ODE-path field the Newton backend ignores)
  - newtonCone "elliptic" + newtonImpratio 10 pinned in the world (MuJoCo-stock pyramidal cone creeps near the friction-cone boundary: 181 mm pseudo-slip at 26 deg with mu=0.5; elliptic+impratio-10 sticks at 0.6 mm). Global Newton default stays MuJoCo stock pending champion re-verification
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
| mujoco | 1 | 0.001004 | 0.00172 | 0.002087 |
| mujoco | 2 | 0.001004 | 0.00172 | 0.001893 |
| mujoco | 4 | 0.001003 | 0.00172 | 0.001951 |
| mujoco | 8 | 0.001003 | 0.001719 | 0.002084 |
| mujoco | 16 | 0.00177 | 0.003086 | 0.002157 |
| mujoco | 32 | 0.005122 | 0.009473 | 0.004054 |
| omnisim-newton | 1 | 0.000583 | 0.000362 | 0.2636 |
| omnisim-newton | 2 | 0.000583 | 0.000362 | 0.3084 |
| omnisim-newton | 4 | 0.000582 | 0.000362 | 0.6208 |
| omnisim-newton | 8 | 0.000585 | 0.000362 | 0.7339 |
| omnisim-newton | 16 | 0.001205 | 0.001265 | 0.9651 |
| omnisim-newton | 32 | 0.002554 | 0.003913 | 2.908 |
| pybullet | 1 | 1.3e-15 | 2.04e-15 | 0.001972 |
| pybullet | 2 | 1.85e-16 | 1.11e-15 | 0.002325 |
| pybullet | 4 | 2.04e-15 | 3.89e-15 | 0.002301 |
| pybullet | 8 | 1.04e-14 | 9.27e-16 | 0.002231 |
| pybullet | 16 | 1.17e-14 | 1.12e-15 | 0.0021 |
| pybullet | 32 | 1.09e-14 | 5.6e-15 | 0.002484 |

<details><summary>deviations (roll)</summary>

- **mujoco**:
  - override: cone=elliptic impratio=10 (same as T2)
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.8 (ContactProperties.coulombFriction=0.8 is an ODE-path field the Newton backend ignores)
- **pybullet**:
  - bullet friction combines multiplicatively: incline mu=1.0, ball mu=0.8
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

### T4 `pendulum_energy`

| engine | dt (ms) | energy_drift_rel | energy_drift_slope | wall ms/step |
|---|---|---|---|---|
| mujoco | 1 | 0.09359 | -0.008973 | 0.001425 |
| mujoco | 2 | 0.1613 | -0.02263 | 0.001488 |
| mujoco | 4 | 0.3871 | -0.04797 | 0.001362 |
| mujoco | 8 | 0.6089 | -0.09184 | 0.001388 |
| mujoco | 16 | 0.4205 | -0.07004 | 0.001438 |
| mujoco | 32 | 1.55e-08 | 0.001028 | 0.1014 |
| omnisim-newton | 1 | 0.171 | -0.01516 | 0.2131 |
| omnisim-newton | 2 | 0.1848 | -0.02499 | 0.2379 |
| omnisim-newton | 4 | 0.4079 | -0.0517 | 0.2598 |
| omnisim-newton | 8 | 0.6283 | -0.09403 | 0.3594 |
| omnisim-newton | 16 | 0.3846 | -0.06637 | 0.4525 |
| omnisim-newton | 32 | 2.32e-09 | 0.001615 | 0.6991 |
| pybullet | 1 | 0.06948 | -0.008869 | 0.001678 |
| pybullet | 2 | 0.1585 | -0.02265 | 0.001689 |
| pybullet | 4 | 0.3859 | -0.04806 | 0.001695 |
| pybullet | 8 | 0.6113 | -0.09202 | 0.001759 |
| pybullet | 16 | 0.4056 | -0.07025 | 0.001738 |
| pybullet | 32 | 0.9992 | 0.09548 | 0.001823 |

<details><summary>deviations (pendulum_energy)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
  - energy_drift normalized by peak KE (22.401 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_blowup_max (metric v1) = max|E(t)| / analytic swing energy (22.07 J); >10 sets energy_blew_up and marks the peak-KE-normalized drift as meaningless (an exploding run inflates its own normalizer)
  - energy_drift normalized by peak KE (22.815 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (25.696 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (27.697 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (588992845.392 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - ENERGY BLEW UP mid-window (blowup=2.67e+07): the energy_drift_rel value is an artifact of the inflated peak-KE normalizer, not conservation
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - energy_drift normalized by peak KE (22.401 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_blowup_max (metric v1) = max|E(t)| / analytic swing energy (22.07 J); >10 sets energy_blew_up and marks the peak-KE-normalized drift as meaningless (an exploding run inflates its own normalizer)
  - energy_drift normalized by peak KE (22.815 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (25.696 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (27.695 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (1470679260.642 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - ENERGY BLEW UP mid-window (blowup=6.66e+07): the energy_drift_rel value is an artifact of the inflated peak-KE normalizer, not conservation
- **pybullet**:
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown
  - energy_drift normalized by peak KE (22.401 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_blowup_max (metric v1) = max|E(t)| / analytic swing energy (22.07 J); >10 sets energy_blew_up and marks the peak-KE-normalized drift as meaningless (an exploding run inflates its own normalizer)
  - energy_drift normalized by peak KE (22.815 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (23.780 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (25.696 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (27.697 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - energy_drift normalized by peak KE (10900.541 J): E(0)=0 exactly for the horizontal release, so |E0| normalization is undefined
  - ENERGY BLEW UP mid-window (blowup=495): the energy_drift_rel value is an artifact of the inflated peak-KE normalizer, not conservation

</details>

### T5 `momentum`

| engine | dt (ms) | linear_momentum_max | angular_momentum_drift_rel | wall ms/step |
|---|---|---|---|---|
| mujoco | 1 | 0.7994 | 0.001756 | 0.001642 |
| mujoco | 2 | 2.361 | 0.2444 | 0.00162 |
| mujoco | 4 | 5.847 | 1.449 | 0.001617 |
| mujoco | 8 | 5.667 | 0.07956 | 0.001608 |
| mujoco | 16 | 3.24e+04 | 0.7836 | 0.05162 |
| mujoco | 32 | 4.4e+03 | 0.9402 | 0.1082 |
| omnisim-newton | 1 | 1.703 | 0.6974 | 0.2458 |
| omnisim-newton | 2 | 2.215 | 0.008151 | 0.2863 |
| omnisim-newton | 4 | 3.91 | 1.555 | 0.3386 |
| omnisim-newton | 8 | 6.581 | 0.4149 | 0.3377 |
| omnisim-newton | 16 | 1.09e+04 | 0.6336 | 0.4677 |
| omnisim-newton | 32 | 22.1 | 0.6565 | 0.817 |
| pybullet | 1 | 0.8865 | 0.000117 | 0.001494 |
| pybullet | 2 | 2.365 | 0.174 | 0.001512 |
| pybullet | 4 | 6.033 | 1.384 | 0.001547 |
| pybullet | 8 | 5.951 | 0.165 | 0.00169 |
| pybullet | 16 | 355.4 | 12.53 | 0.001672 |
| pybullet | 32 | 252.6 | 24.99 | 0.001833 |

<details><summary>deviations (momentum)</summary>

- **mujoco**:
  - 'middle joint' = link1-link2 hinge (3 links have only 2 internal joints)
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.07522 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.1869 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.5273 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.4743 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.6042 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (1.378 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - T5 torque phase is a supervisor addTorque couple on the two links flanking the middle joint (exact for this planar chain), not Motor.setTorque: the Newton backend gives motorized hinges a hardcoded position servo that pins the joint and pumps linear momentum (~17 kg*m/s measured); the couple keeps both joints passive under both backends
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.3597 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.5147 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.3112 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.7818 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.7802 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (1.231 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
- **pybullet**:
  - 'middle joint' = link1-link2 hinge (3 links have only 2 internal joints)
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.08665 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.2529 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.5757 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (0.4839 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (1.073 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0
  - angular_momentum_drift_rel (metric v1) normalized by peak |L| during the actuation window (1.861 kg*m^2/s); the v0 post-actuation-|L| normalization is kept as angular_momentum_drift_rel_v0

</details>

### T6 `stack`

| engine | dt (ms) | stack_survivors | settle_creep_m_s | max_penetration_m | wall ms/step |
|---|---|---|---|---|---|
| mujoco | 1 | 10 | 4.06e-07 | 0.000618 | 0.02899 |
| mujoco | 2 | 10 | 5.46e-07 | 0.000618 | 0.02672 |
| mujoco | 4 | 10 | 1.39e-07 | 0.000618 | 0.02621 |
| mujoco | 8 | 10 | 9.6e-06 | 0.000618 | 0.02498 |
| mujoco | 16 | 9 | 0.004002 | 0.0012 | 0.03106 |
| mujoco | 32 | 1 | 1.52e-08 | 0.2979 | 0.02466 |
| omnisim-newton | 1 | 10 | 2.33e-06 | 0.000902 | 0.2623 |
| omnisim-newton | 2 | 10 | 6.37e-07 | 0.000902 | 0.2846 |
| omnisim-newton | 4 | 10 | 1.69e-07 | 0.000902 | 0.3107 |
| omnisim-newton | 8 | 10 | 2.72e-06 | 0.000903 | 0.4433 |
| omnisim-newton | 16 | 0 | 41.97 | 225.6 | 0.465 |
| omnisim-newton | 32 | 1 | 5.83e-12 | 0.2961 | 0.7328 |
| pybullet | 1 | 10 | 8.45e-05 | 3.2e-05 | 0.08888 |
| pybullet | 2 | 10 | 9.44e-05 | 9.69e-05 | 0.08907 |
| pybullet | 4 | 10 | 0.000187 | 0.000356 | 0.08839 |
| pybullet | 8 | 0 | 1.46e-05 | 0.2 | 0.07095 |
| pybullet | 16 | 1 | 2.04e-06 | 0.2997 | 0.05493 |
| pybullet | 32 | 0 | 9.86e-07 | 0.1 | 0.04609 |

<details><summary>deviations (stack)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
  - penetration derived from recorded centre spacing assuming axis-aligned boxes (valid for a surviving stack)
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - Newton friction is the global OMNISIM_NEWTON_GROUND_MU=0.8 (ContactProperties.coulombFriction=0.8 is an ODE-path field the Newton backend ignores)
  - newtonCone "elliptic" + newtonImpratio 10 pinned in the world (MuJoCo-stock pyramidal cone creeps near the friction-cone boundary: 181 mm pseudo-slip at 26 deg with mu=0.5; elliptic+impratio-10 sticks at 0.6 mm). Global Newton default stays MuJoCo stock pending champion re-verification
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
| mujoco | 1 | 0.001691 | 0.001418 | 0.001186 |
| mujoco | 2 | 0.003299 | 0.002637 | 0.001173 |
| mujoco | 4 | 0.00625 | 0.004537 | 0.001155 |
| mujoco | 8 | 0.01103 | 0.006613 | 0.001189 |
| mujoco | 16 | 0.01645 | 0.006764 | 0.001165 |
| mujoco | 32 | 0.01631 | 0.00312 | 0.001155 |
| omnisim-newton | 1 | 0.003198 | 0.005973 | 0.1642 |
| omnisim-newton | 2 | 0.006377 | 0.01189 | 0.1795 |
| omnisim-newton | 4 | 0.01268 | 0.02357 | 0.1777 |
| omnisim-newton | 8 | 0.02506 | 0.04632 | 0.1639 |
| omnisim-newton | 16 | 0.04898 | 0.08947 | 0.1721 |
| omnisim-newton | 32 | 0.09376 | 0.1673 | 0.1712 |
| pybullet | 1 | 4.88e-06 | 8.84e-09 | 0.000631 |
| pybullet | 2 | 9.76e-06 | 1.77e-08 | 0.000637 |
| pybullet | 4 | 1.95e-05 | 3.55e-08 | 0.000635 |
| pybullet | 8 | 3.92e-05 | 7.14e-08 | 0.000654 |
| pybullet | 16 | 7.88e-05 | 1.45e-07 | 0.000755 |
| pybullet | 32 | 0.00016 | 2.97e-07 | 0.000833 |

<details><summary>deviations (spin)</summary>

- **mujoco**:
  - mujoco defaults: integrator=Euler, solver=Newton, iterations=100, mujoco 3.11.0
- **omnisim-newton**:
  - newtonSolver "mujoco" pinned in the world (XPBD contact pathologies: locked wheel pairs, cannot hold grasps)
  - initial spin: recorder tries supervisor setVelocity first (a t=0 setVelocity used to be dropped on Newton — pre-registration immediate message; now queued + drained after finalize) and falls back to a closed-loop torque-impulse spin-up if the achieved omega is off; the method actually used is in the .meta.json 'spin' block
- **pybullet**:
  - pybullet damps the off-axis seed ~3x in the first second and the intermediate-axis (Dzhanibekov) eruption does NOT occur within the 10 s window (timing is chaotically sensitive: a 1e-10 inertia perturbation moves it by seconds; measured eruption ~8-10 s in some configs, adding ~2% rot KE). Conservation metrics here therefore describe a near-steady spin, not a tumble; MuJoCo's instability grows within the window.
  - bullet default solver: numSolverIterations=50 numSubSteps=0
  - pybullet unknown

</details>

## Lane 2 — throughput (three-tier, Go2)

| engine | tier | batch | env-steps/s | ms/control-step | status |
|---|---|---|---|---|---|
| mujoco-warp-raw | sim_only | 256.0 | 8.08e+04 | 3.167 | ok |
| omnisim-newton | sim_only | 256.0 | 1.04e+04 | 24.64 | ok |
| omnisim-newton | sim_only | 256.0 | 6.55e+04 | 3.911 | ok |
| mujoco-warp-raw | sim_infer | 256.0 | 5.09e+04 | 5.03 | ok |
| mujoco-warp-raw | sim_only | 1024.0 | 2.83e+05 | 3.625 | ok |
| omnisim-newton | sim_only | 1024.0 | 4.07e+04 | 25.18 | ok |
| omnisim-newton | sim_only | 1024.0 | 2.26e+05 | 4.529 | ok |
| mujoco-warp-raw | sim_infer | 1024.0 | 1.38e+05 | 7.429 | ok |
| mujoco-warp-raw | sim_only | 4096.0 | 7.48e+05 | 5.474 | ok |
| omnisim-newton | sim_only | 4096.0 | 1.57e+05 | 26.02 | ok |
| omnisim-newton | sim_only | 4096.0 | 5.37e+05 | 7.631 | ok |
| mujoco-warp-raw | sim_infer | 4096.0 | 3.63e+05 | 11.29 | ok |
| omnisim-newton | sim_train | 256.0 | 4.67e+04 | — | ok |
| omnisim-newton | sim_train | 1024.0 | 1.68e+05 | — | ok |
| omnisim-newton | sim_train | 2048.0 | 2.98e+05 | — | ok |
| omnisim-newton | sim_train | 4096.0 | 5e+05 | — | ok |

OmniSim embedded-solver / raw mujoco-warp sim_only ratio (same batch): batch 256.0: 1.23x, batch 1024.0: 1.25x, batch 4096.0: 1.39x

## Lane 3 — determinism / parity / driveability

### 3a determinism

| test | engine | grade | max abs dev | first div step | steps |
|---|---|---|---|---|---|
| determinism_cold_cold | omnisim-newton | bitwise | 0 | -1.0 | 400.0 |
| determinism_cold_warm | omnisim-newton | bitwise | 0 | -1.0 | 400.0 |

## FINDINGS (auto: sanity-threshold breaches)

- [lane1] bounce mujoco dt=1 ms: bounce_height_rmse_rel=0.1725 (> 0.1)
- [lane1] momentum mujoco dt=1 ms: linear_momentum_max=0.7994 (> 0.1)
- [lane1] bounce mujoco dt=2 ms: bounce_height_rmse_rel=0.2715 (> 0.1)
- [lane1] pendulum_energy mujoco dt=2 ms: energy_drift_rel=0.1613 (> 0.1)
- [lane1] momentum mujoco dt=2 ms: linear_momentum_max=2.361 (> 0.1)
- [lane1] momentum mujoco dt=2 ms: angular_momentum_drift_rel=0.2444 (> 0.1)
- [lane1] bounce mujoco dt=4 ms: bounce_height_rmse_rel=0.1478 (> 0.1)
- [lane1] pendulum_energy mujoco dt=4 ms: energy_drift_rel=0.3871 (> 0.1)
- [lane1] momentum mujoco dt=4 ms: linear_momentum_max=5.847 (> 0.1)
- [lane1] momentum mujoco dt=4 ms: angular_momentum_drift_rel=1.449 (> 0.1)
- [lane1] bounce mujoco dt=8 ms: bounce_height_rmse_rel=35.97 (> 0.1)
- [lane1] pendulum_energy mujoco dt=8 ms: energy_drift_rel=0.6089 (> 0.1)
- [lane1] momentum mujoco dt=8 ms: linear_momentum_max=5.667 (> 0.1)
- [lane1] bounce mujoco dt=16 ms: bounce_height_rmse_rel=33.52 (> 0.1)
- [lane1] pendulum_energy mujoco dt=16 ms: energy_drift_rel=0.4205 (> 0.1)
- [lane1] momentum mujoco dt=16 ms: linear_momentum_max=3.24e+04 (> 0.1)
- [lane1] momentum mujoco dt=16 ms: angular_momentum_drift_rel=0.7836 (> 0.1)
- [lane1] stack mujoco dt=16 ms: stack_survivors=9 (< 10)
- [lane1] stack mujoco dt=16 ms: settle_creep_m_s=0.004002 (> 0.001)
- [lane1] bounce mujoco dt=32 ms: bounce_height_rmse_rel=396.5 (> 0.1)
- [lane1] incline mujoco dt=32 ms: slide_accel_rel_err=0.3298 (> 0.1)
- [lane1] momentum mujoco dt=32 ms: linear_momentum_max=4.4e+03 (> 0.1)
- [lane1] momentum mujoco dt=32 ms: angular_momentum_drift_rel=0.9402 (> 0.1)
- [lane1] stack mujoco dt=32 ms: stack_survivors=1 (< 10)
- [lane1] stack mujoco dt=32 ms: max_penetration_m=0.2979 (> 0.005)
- [lane1] incline pybullet dt=1 ms: stick_violation_max_m=0.001116 (> 0.001)
- [lane1] momentum pybullet dt=1 ms: linear_momentum_max=0.8865 (> 0.1)
- [lane1] incline pybullet dt=2 ms: stick_violation_max_m=0.001434 (> 0.001)
- [lane1] pendulum_energy pybullet dt=2 ms: energy_drift_rel=0.1585 (> 0.1)
- [lane1] momentum pybullet dt=2 ms: linear_momentum_max=2.365 (> 0.1)
- [lane1] momentum pybullet dt=2 ms: angular_momentum_drift_rel=0.174 (> 0.1)
- [lane1] incline pybullet dt=4 ms: stick_violation_max_m=0.001952 (> 0.001)
- [lane1] pendulum_energy pybullet dt=4 ms: energy_drift_rel=0.3859 (> 0.1)
- [lane1] momentum pybullet dt=4 ms: linear_momentum_max=6.033 (> 0.1)
- [lane1] momentum pybullet dt=4 ms: angular_momentum_drift_rel=1.384 (> 0.1)
- [lane1] bounce pybullet dt=8 ms: bounce_height_rmse_rel=0.1606 (> 0.1)
- [lane1] incline pybullet dt=8 ms: stick_violation_max_m=0.001349 (> 0.001)
- [lane1] pendulum_energy pybullet dt=8 ms: energy_drift_rel=0.6113 (> 0.1)
- [lane1] momentum pybullet dt=8 ms: linear_momentum_max=5.951 (> 0.1)
- [lane1] momentum pybullet dt=8 ms: angular_momentum_drift_rel=0.165 (> 0.1)
- [lane1] stack pybullet dt=8 ms: stack_survivors=0 (< 10)
- [lane1] stack pybullet dt=8 ms: max_penetration_m=0.2 (> 0.005)
- [lane1] bounce pybullet dt=16 ms: bounce_height_rmse_rel=0.3497 (> 0.1)
- [lane1] incline pybullet dt=16 ms: stick_violation_max_m=0.001377 (> 0.001)
- [lane1] pendulum_energy pybullet dt=16 ms: energy_drift_rel=0.4056 (> 0.1)
- [lane1] momentum pybullet dt=16 ms: linear_momentum_max=355.4 (> 0.1)
- [lane1] momentum pybullet dt=16 ms: angular_momentum_drift_rel=12.53 (> 0.1)
- [lane1] stack pybullet dt=16 ms: stack_survivors=1 (< 10)
- [lane1] stack pybullet dt=16 ms: max_penetration_m=0.2997 (> 0.005)
- [lane1] bounce pybullet dt=32 ms: bounce_height_rmse_rel=0.7752 (> 0.1)
- [lane1] incline pybullet dt=32 ms: stick_violation_max_m=0.001457 (> 0.001)
- [lane1] pendulum_energy pybullet dt=32 ms: energy_drift_rel=0.9992 (> 0.1)
- [lane1] momentum pybullet dt=32 ms: linear_momentum_max=252.6 (> 0.1)
- [lane1] momentum pybullet dt=32 ms: angular_momentum_drift_rel=24.99 (> 0.1)
- [lane1] stack pybullet dt=32 ms: stack_survivors=0 (< 10)
- [lane1] stack pybullet dt=32 ms: max_penetration_m=0.1 (> 0.005)
- [lane1] bounce omnisim-newton dt=8 ms: bounce_height_rmse_rel=0.1177 (> 0.1)
- [lane1] bounce omnisim-newton dt=16 ms: bounce_height_rmse_rel=0.2264 (> 0.1)
- [lane1] bounce omnisim-newton dt=32 ms: bounce_height_rmse_rel=2.04 (> 0.1)
- [lane1] incline omnisim-newton dt=1 ms: slide_accel_rel_err=0.5599 (> 0.1)
- [lane1] incline omnisim-newton dt=4 ms: slide_accel_rel_err=0.1059 (> 0.1)
- [lane1] incline omnisim-newton dt=32 ms: stick_violation_max_m=0.001014 (> 0.001)
- [lane1] incline omnisim-newton dt=32 ms: slide_accel_rel_err=0.3485 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=1 ms: energy_drift_rel=0.171 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=2 ms: energy_drift_rel=0.1848 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=4 ms: energy_drift_rel=0.4079 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=8 ms: energy_drift_rel=0.6283 (> 0.1)
- [lane1] pendulum_energy omnisim-newton dt=16 ms: energy_drift_rel=0.3846 (> 0.1)
- [lane1] momentum omnisim-newton dt=1 ms: linear_momentum_max=1.703 (> 0.1)
- [lane1] momentum omnisim-newton dt=1 ms: angular_momentum_drift_rel=0.6974 (> 0.1)
- [lane1] momentum omnisim-newton dt=2 ms: linear_momentum_max=2.215 (> 0.1)
- [lane1] momentum omnisim-newton dt=4 ms: linear_momentum_max=3.91 (> 0.1)
- [lane1] momentum omnisim-newton dt=4 ms: angular_momentum_drift_rel=1.555 (> 0.1)
- [lane1] momentum omnisim-newton dt=8 ms: linear_momentum_max=6.581 (> 0.1)
- [lane1] momentum omnisim-newton dt=8 ms: angular_momentum_drift_rel=0.4149 (> 0.1)
- [lane1] momentum omnisim-newton dt=16 ms: linear_momentum_max=1.09e+04 (> 0.1)
- [lane1] momentum omnisim-newton dt=16 ms: angular_momentum_drift_rel=0.6336 (> 0.1)
- [lane1] momentum omnisim-newton dt=32 ms: linear_momentum_max=22.1 (> 0.1)
- [lane1] momentum omnisim-newton dt=32 ms: angular_momentum_drift_rel=0.6565 (> 0.1)
- [lane1] stack omnisim-newton dt=16 ms: stack_survivors=0 (< 10)
- [lane1] stack omnisim-newton dt=16 ms: settle_creep_m_s=41.97 (> 0.001)
- [lane1] stack omnisim-newton dt=16 ms: max_penetration_m=225.6 (> 0.005)
- [lane1] stack omnisim-newton dt=32 ms: stack_survivors=1 (< 10)
- [lane1] stack omnisim-newton dt=32 ms: max_penetration_m=0.2961 (> 0.005)
- [lane1] spin omnisim-newton dt=32 ms: rot_ke_drift_rel=0.1673 (> 0.1)

## Gaps (recorded in MANIFEST.json)

- none
