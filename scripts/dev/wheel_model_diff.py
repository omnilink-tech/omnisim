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

"""wheel_model_diff -- build the SAME chassis+wheel two ways (probe-style direct builder calls
that DRIVE, vs omnisim_newton_runtime's API that LOCKS), with identical geometry, then drive both
with the probe step loop and DIFF the finalized model.joint_* arrays to find what g1 builds
differently. Run from PowerShell.
"""
import os
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
# The deploy runtime is imported from its source location (the same module the
# engine's embedded interpreter runs). This used to point at projects/rl/backends
# (deleted in the rl -> policies reorg) and import the g1_deploy_runtime mirror,
# which is gone too (2026-09-02).
sys.path.insert(0, str(_REPO / "src" / "omnisim" / "physics"))
sys.path.insert(0, str(_REPO / "scripts" / "xpbd_probes"))
try:
    import truststore
    truststore.inject_into_ssl()
except ImportError:
    pass

import newton
import warp as wp
import omnisim_newton_runtime as rt

DT, STEPS = 1.0 / 60.0, 400
CZ, WPOS, WR, WM, CM = 0.40, (0.256, 0.0, 0.165), 0.165, 2.6, 30.0  # PROBE geometry (drives at omega=5)


def iso(m, r):
    i = 0.4 * m * r * r
    return wp.mat33((i, 0.0, 0.0), (0.0, i, 0.0), (0.0, 0.0, i))


def block(m, hx, hy, hz):
    ixx = (1.0 / 12.0) * m * (4 * hy * hy + 4 * hz * hz)
    iyy = (1.0 / 12.0) * m * (4 * hx * hx + 4 * hz * hz)
    izz = (1.0 / 12.0) * m * (4 * hx * hx + 4 * hy * hy)
    return wp.mat33((ixx, 0.0, 0.0), (0.0, iyy, 0.0), (0.0, 0.0, izz))


def build_probe():
    b = newton.ModelBuilder(up_axis=newton.Axis.Z)
    b.default_joint_cfg.armature = 0.0
    b.default_shape_cfg.mu = 1.0
    sc = newton.ModelBuilder.ShapeConfig(density=0.0, mu=1.0)
    # (newton >=1.4 REMOVED ModelBuilder.add_link(armature=...). Dropping it is
    # byte-equivalent to the old armature=0.0: that was already the default.)
    chassis = b.add_link(xform=wp.transform((0.0, 0.0, CZ), wp.quat_identity()),
                         inertia=block(CM, 0.5, 0.275, 0.1), mass=CM, label="chassis")
    b.add_shape_box(chassis, xform=wp.transform_identity(), hx=0.5, hy=0.275, hz=0.1, cfg=sc)
    jf = b.add_joint_free(child=chassis)
    wx, wy, wz = WPOS
    # (newton >=1.4 REMOVED ModelBuilder.add_link(armature=...); armature=0.0 was
    # the default, so omitting it changes nothing.)
    wheel = b.add_link(xform=wp.transform((wx, wy, wz), wp.quat_identity()),
                       inertia=iso(WM, WR), mass=WM, label="wheel")
    b.add_shape_sphere(wheel, xform=wp.transform_identity(), radius=WR, cfg=sc)
    jw = b.add_joint_revolute(
        parent=chassis, child=wheel, axis=wp.vec3(0.0, 1.0, 0.0),
        parent_xform=wp.transform(wp.vec3(wx, wy, wz - CZ), wp.quat_identity()),
        child_xform=wp.transform_identity(),
        target_ke=0.0, target_kd=500.0, armature=0.0, limit_ke=0.0, limit_kd=0.0,
        actuator_mode=newton.JointTargetMode.POSITION_VELOCITY,
        collision_filter_parent=True, effort_limit=200.0, velocity_limit=60.0)
    b.add_articulation([jf, jw])
    b.add_ground_plane()
    return b.finalize(), jw


def build_g1():
    w = rt.World()
    chassis = w.add_body(CM, 0.0, 0.0, CZ)
    w.add_shape_box(chassis, 0.3, 0.2, 0.09)
    wx, wy, wz = WPOS
    wheel = w.add_body(WM, wx, wy, wz, ixx=0.4 * WM * WR * WR, iyy=0.4 * WM * WR * WR, izz=0.4 * WM * WR * WR)
    w.add_shape_sphere(wheel, WR)
    slot = w.add_joint_revolute(chassis, wheel, 0.0, 1.0, 0.0, wx, wy, wz - CZ, 0.0, 0.0, 0.0,
                                target_ke=0.0, target_kd=500.0, limit_lower=0.0, limit_upper=0.0,
                                effort_limit=200.0, velocity_limit=60.0)
    w.add_ground_plane()
    w.finalize()
    return w.model, w.slot_to_real_idx[slot]


def drive(model, jw, omega):
    solver = newton.solvers.SolverXPBD(model, iterations=10, angular_damping=0.0)
    s0, s1 = model.state(), model.state()
    control = model.control()
    contacts = model.contacts()
    qd_start = model.joint_qd_start.numpy()
    q_start = model.joint_q_start.numpy()
    tv = control.joint_target_qd.numpy()
    tv[int(qd_start[jw])] = omega
    control.joint_target_qd.assign(tv)
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
    x0 = float(s0.body_q.numpy()[0][0])
    for _ in range(STEPS):
        s0.clear_forces()
        model.collide(s0, contacts)
        solver.step(s0, s1, control, contacts, DT)
        s0, s1 = s1, s0
    xf = float(s0.body_q.numpy()[0][0])
    return xf - x0  # chassis +X displacement (driving wheel -> chassis moves; lock -> ~0)


def dump(model, jw, tag):
    def arr(name):
        try:
            return getattr(model, name).numpy()
        except Exception:
            return None
    out = ["--- %s (revolute joint idx=%d) ---" % (tag, jw)]
    out.append("joint_type=%s" % arr("joint_type"))
    out.append("joint_dof_dim=%s" % arr("joint_dof_dim"))
    for nm in ("joint_X_p", "joint_X_c"):
        a = arr(nm)
        out.append("%s[%d]=%s" % (nm, jw, None if a is None else [round(float(v), 4) for v in a[jw]]))
    for nm in ("joint_axis", "joint_target_ke", "joint_target_kd", "joint_target_mode",
               "joint_armature", "joint_effort_limit", "joint_velocity_limit",
               "joint_limit_lower", "joint_limit_upper", "joint_limit_ke", "joint_dof_mode"):
        a = arr(nm)
        out.append("%-22s = %s" % (nm, None if a is None else [
            (round(float(x), 3) if hasattr(x, "__float__") else x) for x in a]))
    return "\n".join(out)


def g1_step_drive(omega, substeps=None):
    """Build via g1 AND drive via g1's OWN step() (substeps/clamp/guard); chassis +X disp."""
    w = rt.World()
    if substeps is not None:
        w.set_substeps(substeps)
    chassis = w.add_body(CM, 0.0, 0.0, CZ)
    w.add_shape_box(chassis, 0.3, 0.2, 0.09)
    wx, wy, wz = WPOS
    wheel = w.add_body(WM, wx, wy, wz, ixx=0.4 * WM * WR * WR, iyy=0.4 * WM * WR * WR, izz=0.4 * WM * WR * WR)
    w.add_shape_sphere(wheel, WR)
    slot = w.add_joint_revolute(chassis, wheel, 0.0, 1.0, 0.0, wx, wy, wz - CZ, 0.0, 0.0, 0.0,
                                target_ke=0.0, target_kd=500.0, limit_lower=0.0, limit_upper=0.0,
                                effort_limit=200.0, velocity_limit=60.0)
    w.add_ground_plane()
    w.finalize()
    x0 = float(w.state_a.body_q.numpy()[0][0])
    for _ in range(STEPS):
        w.set_joint_target_vel(slot, omega)
        w.step(DT)
    xf = float(w.state_a.body_q.numpy()[0][0])
    return xf - x0


def build_4wheel_g1(omega, effort, vel, substeps=4, anchors=None, add_box=True, mu=None):
    """N-wheel battlebox rover via g1 + g1 step(); chassis +X disp."""
    if anchors is None:
        anchors = ((0.22, 0.21), (0.22, -0.21), (-0.22, 0.21), (-0.22, -0.21))
    w = rt.World()
    if mu is not None:
        w.builder.default_shape_cfg.mu = mu
    w.set_substeps(substeps)
    chassis = w.add_body(10.0, 0.0, 0.0, 0.14)
    if add_box:
        w.add_shape_box(chassis, 0.3, 0.2, 0.09)
    i = 0.4 * 0.6 * 0.09 * 0.09
    slots = []
    for ax, ay in anchors:
        wh = w.add_body(0.6, ax, ay, 0.09, ixx=i, iyy=i, izz=i)
        w.add_shape_sphere(wh, 0.09)
        s = w.add_joint_revolute(chassis, wh, 0.0, 1.0, 0.0, ax, ay, -0.05, 0.0, 0.0, 0.0,
                                 target_ke=0.0, target_kd=500.0, limit_lower=0.0, limit_upper=0.0,
                                 effort_limit=effort, velocity_limit=vel)
        slots.append(s)
    w.add_ground_plane()
    w.finalize()
    x0 = float(w.state_a.body_q.numpy()[0][0])
    for _ in range(STEPS):
        for s in slots:
            w.set_joint_target_vel(s, omega)
        w.step(DT)
    xf = float(w.state_a.body_q.numpy()[0][0])
    return xf - x0


def main():
    import probe_husky_motor_minimal as P
    bw = P.build_world(0.0, 500.0)
    mP, jwP = bw[0], bw[8]
    rP = drive(mP, jwP, 5.0)
    print("CONTROL: actual probe build_world + MY drive(omega=5) -> chassis_dx=%.3f m (%s)\n"
          % (rP, "DRIVES" if abs(rP) > 0.5 else "LOCKED"))
    print("[wheel_model_diff] narrowing the multi-wheel lock (g1 step(), chassis +X disp, omega=12)")
    cases = [
        ("1 wheel @(0.22,0)", [(0.22, 0.0)], True),
        ("2 wheels (rear axle)", [(-0.22, 0.21), (-0.22, -0.21)], True),
        ("2 wheels (diagonal)", [(0.22, 0.21), (-0.22, -0.21)], True),
        ("4 wheels", None, True),
        ("4 wheels, NO chassis box", None, False),
    ]
    for name, anchors, box in cases:
        dx = build_4wheel_g1(12.0, effort=200.0, vel=60.0, substeps=4, anchors=anchors, add_box=box)
        print("  %-26s dx=%7.3f m -> %s" % (name, dx, "DRIVES" if abs(dx) > 0.5 else "LOCKED"))
    print("  -- friction sweep on the locking rear-axle pair --")
    axle = [(-0.22, 0.21), (-0.22, -0.21)]
    for mu in (1.0, 0.2):
        dx = build_4wheel_g1(12.0, effort=200.0, vel=60.0, substeps=4, anchors=axle, mu=mu)
        print("  rear-axle mu=%-4s            dx=%7.3f m -> %s" % (mu, dx, "DRIVES" if abs(dx) > 0.5 else "LOCKED"))
    print("  -- coaxiality test: break the shared axis with a small x-offset --")
    for off in (0.0, 0.002, 0.01, 0.05):
        a = [(-0.22, 0.21), (-0.22 + off, -0.21)]
        dx = build_4wheel_g1(12.0, effort=200.0, vel=60.0, substeps=4, anchors=a)
        print("  rear-axle x-offset=%-5s      dx=%7.3f m -> %s" % (off, dx, "DRIVES" if abs(dx) > 0.5 else "LOCKED"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
