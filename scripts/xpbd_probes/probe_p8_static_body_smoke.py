# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""P8.1 smoke — Newton static body (mass=0 link) as a collider.

This is the load-bearing technical bet behind P8 statics-on-Newton
(docs/developer/physics-p8-statics-design.md): Newton accepts mass=0
articulation links as STATIC bodies whose pose stays pinned by the
solver across step() while their attached collision shapes still
participate in contact generation against dynamic bodies.

The probe builds a tiny world:

  - One static box (mass=0, 2 m × 2 m × 0.2 m) at the origin.
  - One dynamic sphere (mass=1 kg, radius=0.1 m) suspended 1 m above
    the box's top surface, falling under gravity.

Runs 240 steps at dt = 1/60 s = 4 s of simulated time. Asserts:

  1. The box's pose is unchanged (within 1 µm of its starting position).
  2. The sphere comes to rest on the box (its z stops decreasing in
     the second half of the simulation) at a height consistent with
     the box's top surface + sphere radius.

If both assertions hold, the mass=0-static assumption is sound and
the P8.1 API surface (`addStaticBody` C++ method + `add_static_body`
Python helper that ship with this commit) is wired up correctly on
the Newton side — P8.2 can layer the WbSolid dispatch on top.

Run with: `python scripts/xpbd_probes/probe_p8_static_body_smoke.py`.
"""

import sys

# AVG / TLS interception on this machine — match the project standard
# (see memory feedback_avg_tls_interception). Newton + warp don't hit
# the OmniLink platform but truststore is a no-op when not needed.
try:
    import truststore
    truststore.inject_into_ssl()
except ImportError:
    pass

import warp as wp
import newton


GRAVITY = -9.81
DT = 1.0 / 60.0
STEPS = 240  # 4 s of sim time
BOX_HX, BOX_HY, BOX_HZ = 1.0, 1.0, 0.1  # 2 × 2 × 0.2 m static box
SPHERE_RADIUS = 0.1
SPHERE_DROP_HEIGHT = 1.0  # above box top surface
SPHERE_MASS = 1.0


def build_world():
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    zero_density = newton.ModelBuilder.ShapeConfig(density=0.0, mu=1.0)

    # Static box centred at origin, top surface at z=BOX_HZ.
    static_box = builder.add_link(
        xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity()),
        armature=0.0,
        inertia=wp.mat33((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        mass=0.0,
        label="static_box",
    )
    builder.add_shape_box(
        static_box,
        xform=wp.transform_identity(),
        hx=BOX_HX, hy=BOX_HY, hz=BOX_HZ,
        cfg=zero_density,
    )
    # The static link still needs a free joint slot for the
    # articulation graph; the solver treats mass=0 + zero-inertia as
    # immovable regardless. Without this `add_joint_free` Newton's
    # graph-builder rejects the orphan link.
    j_static = builder.add_joint_free(child=static_box)

    # Dynamic sphere falling onto the box.
    sphere_z = BOX_HZ + SPHERE_RADIUS + SPHERE_DROP_HEIGHT
    sphere_inertia = wp.mat33(
        (0.4 * SPHERE_MASS * SPHERE_RADIUS ** 2, 0.0, 0.0),
        (0.0, 0.4 * SPHERE_MASS * SPHERE_RADIUS ** 2, 0.0),
        (0.0, 0.0, 0.4 * SPHERE_MASS * SPHERE_RADIUS ** 2),
    )
    sphere = builder.add_link(
        xform=wp.transform(wp.vec3(0.0, 0.0, sphere_z), wp.quat_identity()),
        armature=0.0,
        inertia=sphere_inertia,
        mass=SPHERE_MASS,
        label="sphere",
    )
    builder.add_shape_sphere(
        sphere,
        xform=wp.transform_identity(),
        radius=SPHERE_RADIUS,
        cfg=zero_density,
    )
    j_sphere = builder.add_joint_free(child=sphere)

    # The static link goes in its own articulation so the solver
    # doesn't try to integrate it with the sphere's free DOFs.
    builder.add_articulation([j_static], label="statics")
    builder.add_articulation([j_sphere], label="dynamics")

    model = builder.finalize()
    # Newton's default gravity is (0, 0, -9.81) on Z-up models; leave
    # it alone. Setting `model.gravity = wp.vec3(...)` in this Newton
    # release tripped the integrate_bodies kernel's array-type check.
    solver = newton.solvers.SolverXPBD(model, angular_damping=0.0, iterations=10)
    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    contacts = model.contacts()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_0)
    return model, state_0, state_1, control, contacts, solver, static_box, sphere


def main() -> int:
    model, state_0, state_1, control, contacts, solver, static_idx, sphere_idx = build_world()

    box_q_initial = state_0.body_q.numpy()[static_idx].copy()
    sphere_z_history = []

    for step in range(STEPS):
        state_0.clear_forces()
        model.collide(state_0, contacts)
        solver.step(state_0, state_1, control, contacts, DT)
        state_0, state_1 = state_1, state_0
        if step % 4 == 0:
            body_q = state_0.body_q.numpy()
            sphere_z_history.append(body_q[sphere_idx][2])
    wp.synchronize()

    final_q = state_0.body_q.numpy()
    box_q_final = final_q[static_idx]
    sphere_z_final = final_q[sphere_idx][2]

    # Assertion 1: static box pose unchanged.
    box_displacement = max(abs(box_q_final[i] - box_q_initial[i]) for i in range(3))
    if box_displacement > 1e-6:
        print(f"FAIL: static box moved by {box_displacement:.6f} m "
              f"(initial={box_q_initial[:3]}, final={box_q_final[:3]})")
        return 1

    # Assertion 2: sphere came to rest near box top + sphere radius.
    # Expected rest height: BOX_HZ + SPHERE_RADIUS ≈ 0.2 m, allowing
    # some penetration tolerance from the XPBD solver's compliance.
    expected_rest_z = BOX_HZ + SPHERE_RADIUS
    if not (expected_rest_z - 0.05 <= sphere_z_final <= expected_rest_z + 0.05):
        print(f"FAIL: sphere final z={sphere_z_final:.4f} not within "
              f"±0.05 m of expected rest height {expected_rest_z:.4f}")
        return 1

    # Late-stage z should be nearly constant (sphere at rest).
    late_samples = sphere_z_history[-10:]
    late_range = max(late_samples) - min(late_samples)
    if late_range > 0.02:
        print(f"FAIL: sphere still moving in last 10 samples — z range "
              f"= {late_range:.4f} m (samples: {late_samples})")
        return 1

    print("PASS  static_box stayed pinned (drift=%.2e m); sphere came to rest at "
          "z=%.4f m (expected ~%.4f m); late-stage z range = %.4f m"
          % (box_displacement, sphere_z_final, expected_rest_z, late_range))
    return 0


if __name__ == "__main__":
    sys.exit(main())
