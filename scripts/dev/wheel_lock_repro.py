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

"""wheel_lock_repro -- reproduce the omnisim XPBD wheel-lock in PURE PYTHON via the
g1_deploy_runtime mirror (no omnisim-bin, no rebuild) and isolate the cause.

The wheel reaches ~3 rad then freezes. Suspect: omnisim's _add_revolute_to_builder passes
limit_ke=10000/limit_kd=100 to EVERY motorized joint, but for a CONTINUOUS wheel it does NOT
pass limit_lower/limit_upper -- so the joint inherits Newton's default_joint_cfg limit bounds
and the stiff limit spring slams the wheel to a stop at that default bound. The working
standalone probe passes limit_ke=0/limit_kd=0 (no limit enforcement) and drives.

Run from PowerShell (Newton needs warp).
"""
import os
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / "projects" / "rl" / "backends"))
try:
    import truststore
    truststore.inject_into_ssl()
except ImportError:
    pass

import g1_deploy_runtime as rt

DT, STEPS, OMEGA = 0.008, 400, 12.0


def run(label, **env):
    for k in ("OMNISIM_NEWTON_LIMIT_KE", "OMNISIM_NEWTON_LIMIT_KD"):
        os.environ.pop(k, None)
    for k, v in env.items():
        os.environ[k] = v

    w = rt.World()
    chassis = w.add_body(10.0, 0.0, 0.0, 0.14)
    w.add_shape_box(chassis, 0.3, 0.2, 0.09)
    wheel = w.add_body(0.6, 0.22, 0.21, 0.09)
    w.add_shape_sphere(wheel, 0.09)
    slot = w.add_joint_revolute(
        chassis, wheel, 0.0, 1.0, 0.0,
        0.22, 0.21, -0.05, 0.0, 0.0, 0.0,
        target_ke=0.0, target_kd=500.0,
        limit_lower=0.0, limit_upper=0.0,
        effort_limit=200.0, velocity_limit=30.0)
    w.add_ground_plane()
    w.finalize()

    lim = ""
    try:
        ll = w.model.joint_limit_lower.numpy()
        lu = w.model.joint_limit_upper.numpy()
        ke = w.model.joint_limit_ke.numpy()
        lim = "joint_limit_lower=%s upper=%s ke=%s" % (
            [round(float(x), 2) for x in ll],
            [round(float(x), 2) for x in lu],
            [round(float(x), 0) for x in ke])
    except Exception as exc:
        lim = "(limit dump failed: %s)" % exc

    for _ in range(STEPS):
        w.set_joint_target_vel(slot, OMEGA)
        w.step(DT)
    final = w.get_joint_angle(slot)
    print("  %-26s final=%7.2f rad  -> %s\n      %s"
          % (label, final, "DRIVES" if final > 20.0 else "LOCKED", lim))


def probe_step_on_g1(dt, defaults=None, label=None):
    """Build via g1's finalize(), then drive with the PROBE's manual step loop
    (no g1 step(): no substeps, no joint clamp, no base guard). Isolates whether
    the lock is in g1's model/solver BUILD or its STEP pipeline."""
    import newton
    w = rt.World()
    if defaults:
        for _k, _v in defaults.items():
            setattr(w.builder.default_joint_cfg, _k, _v)
    chassis = w.add_body(10.0, 0.0, 0.0, 0.14)
    w.add_shape_box(chassis, 0.3, 0.2, 0.09)
    wheel = w.add_body(0.6, 0.22, 0.21, 0.09)
    w.add_shape_sphere(wheel, 0.09)
    slot = w.add_joint_revolute(
        chassis, wheel, 0.0, 1.0, 0.0,
        0.22, 0.21, -0.05, 0.0, 0.0, 0.0,
        target_ke=0.0, target_kd=500.0,
        limit_lower=0.0, limit_upper=0.0,
        effort_limit=200.0, velocity_limit=30.0)
    w.add_ground_plane()
    w.finalize()
    model, solver, s0, s1 = w.model, w.solver, w.state_a, w.state_b
    control = model.control()
    contacts = model.contacts()
    real = w.slot_to_real_idx[slot]
    qd_start = model.joint_qd_start.numpy()
    q_start = model.joint_q_start.numpy()
    dof = int(qd_start[real])
    tv = control.joint_target_vel.numpy()
    tv[dof] = OMEGA
    control.joint_target_vel.assign(tv)
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
    for _ in range(STEPS):
        s0.clear_forces()
        model.collide(s0, contacts)
        solver.step(s0, s1, control, contacts, dt)
        s0, s1 = s1, s0
    newton.eval_ik(model, s0, s0.joint_q, s0.joint_qd)
    ang = float(s0.joint_q.numpy()[int(q_start[real])])
    print("  %-40s final=%7.2f rad  -> %s"
          % (label or ("PROBE-step on g1 dt=%.4f" % dt), ang,
             "DRIVES" if ang > 20.0 else "LOCKED"))


def replica_on_g1(dt, chassis_z, wheel_pos, wheel_r, wheel_m, chassis_m, label):
    """Build a chassis+1wheel via g1's API with PARAMETERIZED geometry, probe-step it."""
    import newton
    w = rt.World()
    chassis = w.add_body(chassis_m, 0.0, 0.0, chassis_z)
    w.add_shape_box(chassis, 0.3, 0.2, 0.09)
    wx, wy, wz = wheel_pos
    wheel = w.add_body(wheel_m, wx, wy, wz,
                       ixx=0.4 * wheel_m * wheel_r * wheel_r,
                       iyy=0.4 * wheel_m * wheel_r * wheel_r,
                       izz=0.4 * wheel_m * wheel_r * wheel_r)
    w.add_shape_sphere(wheel, wheel_r)
    slot = w.add_joint_revolute(
        chassis, wheel, 0.0, 1.0, 0.0,
        wx, wy, wz - chassis_z, 0.0, 0.0, 0.0,
        target_ke=0.0, target_kd=500.0,
        limit_lower=0.0, limit_upper=0.0,
        effort_limit=200.0, velocity_limit=60.0)
    w.add_ground_plane()
    w.finalize()
    model, solver, s0, s1 = w.model, w.solver, w.state_a, w.state_b
    control = model.control()
    contacts = model.contacts()
    real = w.slot_to_real_idx[slot]
    qd_start = model.joint_qd_start.numpy()
    q_start = model.joint_q_start.numpy()
    tv = control.joint_target_vel.numpy()
    tv[int(qd_start[real])] = OMEGA
    control.joint_target_vel.assign(tv)
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
    for _ in range(STEPS):
        s0.clear_forces()
        model.collide(s0, contacts)
        solver.step(s0, s1, control, contacts, dt)
        s0, s1 = s1, s0
    newton.eval_ik(model, s0, s0.joint_q, s0.joint_qd)
    ang = float(s0.joint_q.numpy()[int(q_start[real])])
    print("  %-44s final=%7.2f rad -> %s" % (label, ang, "DRIVES" if ang > 20.0 else "LOCKED"))


def main():
    print("[wheel_lock_repro] omega=%.0f dt=%.3f steps=%d (expect ~%.0f rad if rolling)"
          % (OMEGA, DT, STEPS, OMEGA * DT * STEPS))
    DTp = 1.0 / 60.0
    # probe geometry (drives standalone) vs battlebox geometry (locks), all via g1:
    replica_on_g1(DTp, 0.40, (0.256, 0.0, 0.165), 0.165, 2.6, 30.0, "probe geom (r.165 z.4 m2.6/30)")
    replica_on_g1(DTp, 0.14, (0.22, 0.21, 0.09), 0.09, 0.6, 10.0, "battlebox geom (r.09 z.14 m.6/10)")
    replica_on_g1(DTp, 0.40, (0.22, 0.0, 0.165), 0.165, 0.6, 10.0, "probe-r/z, battlebox masses")
    replica_on_g1(DTp, 0.14, (0.256, 0.0, 0.165), 0.165, 2.6, 30.0, "probe masses, low chassis z=.14")
    run("g1 step() default")
    probe_step_on_g1(1.0 / 60.0, label="probe-step, g1 defaults (kd=500/arm=0.01)")
    probe_step_on_g1(1.0 / 60.0, defaults={"target_kd": 0.0},
                     label="probe-step, default_joint_cfg.target_kd=0")
    probe_step_on_g1(1.0 / 60.0, defaults={"armature": 0.0},
                     label="probe-step, default_joint_cfg.armature=0")
    probe_step_on_g1(1.0 / 60.0, defaults={"target_kd": 0.0, "target_ke": 0.0, "armature": 0.0},
                     label="probe-step, all defaults -> probe values")
    return 0


if __name__ == "__main__":
    sys.exit(main())
