# XPBD probes — notes

Goal: characterize Newton's SolverXPBD behavior on minimal cases so we can pick a working config for the husky **before** we rebuild the backend.

Each probe answers ONE question. Probes share warp's CUDA kernel cache, so we run them sequentially in the same Python process when possible.

## Final config (validated by probe 6 — mini-husky drove 4.05m / 5s = 98% of target)

```python
# Body creation:
builder.add_link(xform=..., armature=0.0, inertia=I, mass=m, label="...")
# NOT builder.add_body — that auto-adds a phantom 6-DOF free joint.

# Joint creation (one per body, plus one free for the chassis):
j_free = builder.add_joint_free(child=chassis)
j_wheel = builder.add_joint_revolute(
    parent=chassis, child=wheel, axis=...,
    target_pos=0.0, target_vel=ω,
    target_ke=0.0, target_kd=500.0,        # ← kd=500, NOT 1
    armature=0.0, limit_ke=0.0, limit_kd=0.0,
    actuator_mode=newton.JointTargetMode.POSITION_VELOCITY,
)

# Group all joints into an articulation:
builder.add_articulation([j_free, j_wheel, ...])

# Solver:
solver = newton.solvers.SolverXPBD(model, angular_damping=0.0, iterations=10)
# angular_damping=0 lets wheels spin freely. iter=10 is enough with the
# articulation tree (vs needing 100+ for ad-hoc constraints in probe 3).

# Per step:
state.clear_forces()
model.collide(state, contacts)
solver.step(state, state_next, control, contacts, dt)

# Readback (joint angles/vels): not auto-updated by XPBD.
newton.eval_ik(model, state, state.joint_q, state.joint_qd)
```

## Findings

### Probe 9 — full helper module (mirrors C++ kNewtonRuntimeSource) ✓ 97.8%
Validated the helper against the same husky setup as probe 7. Iterating found three more bugs the helper introduced:
1. `add_shape_*` defaulted to **density=1000** silently inflating wheel mass by ~19 kg (8x intended). Fix: ShapeConfig(density=0, mu=1) on every shape — mass comes from `add_link(mass=...)` alone.
2. `joint_target_vel` indexed by **DOF**, not joint. With 1 free (6 DOF) + 4 revolute, wheel DOFs sit at indices 6-9 not 0-3. Fix: translate via `joint_qd_start[joint_idx]`.
3. Free joints had to be added **eagerly** (during `add_joint_revolute` when a parent isn't yet a child of any joint), not at finalize-time. `add_articulation` requires monotonic joint indices, so finalize-time appends are too late.

After all three fixes: identical behavior to probe 7. Drives 4.04 m / 10 s = 97.8% efficiency. Final helper config encoded in WbNewtonBackend.cpp:kNewtonRuntimeSource.

### Probe 8 — Newton's `add_urdf` importer ✗ EXPLODES
- Same XPBD config that drove probe 7 cleanly.
- The husky URDF imports (15 bodies, 15 joints, 10 DOF) but **NaN by step 10** — chassis falls *below* ground in step 1 (z=0.069 < wheel rest 0.165).
- Cause: `ignore_inertial_definitions=True` recomputes inertia from collision shapes (two thin chassis boxes → unrealistic inertia tensor). With `=False` we'd hit the same "URDF spec inertia too small" bug from probe 7.
- **Conclusion:** the manual joint construction path (our existing backend) is the right path. Don't rewrite to use add_urdf.

### Probe 7 — exact husky geometry, manual construction ✓ 97.8% efficiency
- 5 bodies (chassis + 4 sphere wheels at real URDF positions), real masses (46 + 4×2.6 kg).
- **First attempt with URDF-spec inertia (ixx=0.6, iyy=1.7, izz=2.0): wheels reversed at step 226 (3.8s), efficiency 34%.** Not enough rotational inertia → chassis perturbations get amplified.
- **With geometric-block inertia (1.57, 4.45, 4.79) for the chassis: drove 4.038m in 10s = 97.8% efficiency.**
- Cylinder wheels caused wheels to lock up immediately; sphere wheels rolled cleanly. Decision: use sphere shape for wheels in the backend (cylinder→sphere fallback already exists in our WbSolid code).
- Chassis collision shapes (`has_shape_collision=False`) made no difference (collision_filter_parent=True already prevents wheel-chassis contact via the joint).

### Probe 6 — 4-wheel mini-husky ✓✓ PROOF OF CONCEPT
- 5 bodies (chassis mass-only + 4 sphere wheels), 5 joints (1 free + 4 revolute), 10 DOF.
- Chassis 46 kg, wheels 2.6 kg each, R=0.165, target ω=5 rad/s on all 4.
- **Drove 4.047 m in 5 s = 98% of theoretical max (4.125 m at 0.825 m/s).**
- All 4 wheels stayed at exactly +5.00 rad/s after step 30 ramp-up.
- chassis_y stayed at 0.000 (no drift), chassis_z 0.398 (stable, no bounce).
- Iterations=10 enough — articulation tree gives XPBD a clean kinematic structure.

### Probe 5 — 2-wheel cart: pitch wobble (bicycle problem)
- Drove 1.13m in 3s before pitch oscillation took over.
- Two wheels = no fore-aft stability. Chassis pitched, wheels lost contact.
- Not a Newton/XPBD bug — physically a 2-wheel rigid cart IS unstable. Need 4 wheels.

### Probe 4 — wheel actuation: TWO ROOT CAUSES of our husky failure ⚠⚠⚠

**Cause 1 — phantom free joints from `add_body`:**
- `builder.add_body()` AUTO-ADDS a 6-DOF JointType.FREE for that body to world.
- Calling `add_joint_revolute(parent, child)` afterwards adds a SECOND joint without removing the first.
- Net effect: every body has TWO joints (free + revolute) that fight each other; XPBD can't satisfy both.
- **Fix:** use `builder.add_link()` instead. add_link creates a body WITHOUT auto-joint, then add_joint_X adds the only joint. Topology becomes `joint_count = body_count`.
- Then `builder.add_articulation([joint_indices])` groups joints into an articulation tree.
- **This alone explains why our husky's chassis decoupled from its wheels: every WbSolid in the manual flow gets a phantom free joint.**

**Cause 2 — kd gain ~500× too low for velocity control:**
- `default_joint_cfg.target_kd = 1.0` (from example_basic_urdf.py, which is position-control only) is far too low for velocity drive.
- Newton's official velocity-control test (`newton/tests/test_joint_controllers.py` inside the bundled newton runtime) uses `target_ke=0, target_kd=500`.
- With kd=1: wheel doesn't spin at all (`omega ≈ 0` after 100 steps).
- With kd=500: wheel reaches `target = π/2` by step 10 and holds it.
- Need to also set `SolverXPBD(angular_damping=0.0)` so XPBD doesn't damp the spin.

**Recipe that works (probe 4i):**
```
builder.add_link(...)
builder.add_joint_revolute(parent=-1, child=...,
    target_pos=0, target_vel=ω,
    target_ke=0, target_kd=500,
    actuator_mode=POSITION_VELOCITY,
    armature=0, limit_ke=0, limit_kd=0)
builder.add_articulation([j])
solver = SolverXPBD(model, angular_damping=0.0, iterations=5)
# in loop: state.clear_forces() before solver.step
# after step: eval_ik(model, state, state.joint_q, state.joint_qd) to read joint state
```

### Probe 3 — iter × substeps sweep on pendulum
Setup: same pendulum, sweep iter ∈ {10,30,100}, subs ∈ {1,5}, 120 frames @ 60Hz.

| iter | subs | max_mm | settled_mm | time_s |
|------|------|--------|------------|--------|
|  10  |  1   |  77.90 |  22.48     |  0.98  |
|  10  |  5   |   4.58 |   1.40     |  2.66  |
|  30  |  1   |  31.92 |  10.18     |  1.28  |
|  30  |  5   |   1.38 |   0.43     |  5.56  |
| 100  |  1   |   7.82 |   2.46     |  3.30  |
| 100  |  5   |   0.33 |   0.10     | 15.65  |

**Key insight:** Effective constraint stiffness ∝ `iter × subs²` (substeps dominate).
- iter=10, subs=5 (effective ~250) beats iter=100, subs=1 (effective ~100) — 4.58 mm vs 7.82 mm.
- For husky chassis (~30 kg per hinge load, 30× this probe), expect drift ≈ 30× single-pendulum drift. So we need ~10 mm drift on this probe to land at ~300 mm on husky → **insufficient**. Need <1 mm here → iter=30 subs=5 or iter=100 subs=5.
- Compute cost: iter=100/subs=5 is 16× iter=10/subs=1; for the husky at 60 Hz that's still 4 ms/frame, fine for single robot.

### Probe 2b — pendulum hinge under gravity ⚠ XPBD STRETCHES
- Setup: child mass=1, anchored to world via `parent=-1`, swing radius 0.30 m
- iter=10, 120 steps (2 sec)
- **Constraint drift up to 77.9 mm** (26% of pin distance!) at peak swing-down
- Settled oscillation: 27+ mm drift
- This is the husky failure mode in microcosm: with iter=10 + 1 kg load, XPBD lets a hinge stretch a quarter of its length. With 116 kg (husky chassis), it's much worse.
- Next: how does iter count buy back rigidity?

### Probe 1 — gravity sanity ✓ PASS
- Setup: 1 body, mass 1.0, sphere r=0.05, no ground, dt=1/60
- Result: z dropped 4.99 m over 60 steps (1 sec) — expected 4.905 m from `0.5·g·t²`
- Drift: ~1.7% over (XPBD's position projection adds a small extra step; acceptable)
- Note: `body_qd[i][5]` reads as 0.0 — XPBD evidently doesn't write this index for linear z-vel. Layout TBD; rely on **position differences** (`(z_t+1 − z_t)/dt`) for velocity, not `body_qd`.
- Conclusion: gravity is on by default at -9.81 m/s². No special config needed.
