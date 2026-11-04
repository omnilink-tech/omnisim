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

"""Position servos carry the rotor inertia their damping gain implies (2026-10-17).

WHY THIS EXISTS
---------------
Every position servo is built ke = effort*10, kd = effort*0.5 with zero armature
(OmBasicJoint.cpp). MuJoCo's implicit integrator applies the velocity actuator as
(M + dt*kd), but the constraint solve (contacts, joint limits, the URDF <mimic>
equality) sizes its force against the bare M -- so on a light joint
(dt*kd/M >> 1) every constraint acts at M/(M+dt*kd) strength. Measured on the
Robonine SO-ARM 102 (7 g jaws, dt*kd/M 13.6; wrist 857): the jaws closed 21 mm
into a 40 mm cube, ran past their own limits, and the cube squirted out at
lift-off at every jaw force from 3 to 100 N. World._bound_position_servo_inertia
adds ``dt*kd - M`` of armature on fixed-base position servos (and reflects the
leader's onto a mimic follower), and refreshes MuJoCo's constraint weights.

This pins, engine-free on the SOURCE runtime, with a minimal two-jaw pinch (a
20 g palm on a vertical slider under a static mount, two 7.3 g jaws on
horizontal sliders, effort 100 -> ke 1000 / kd 50, a 40 mm 50 g cube on a
floor):

  * the armature added is exactly dt*kd - M (and the follower's is mult^2 x the
    leader's);
  * two driven jaws stop at the part (< 4 mm total interference) instead of
    closing ~21 mm into it (the defect, reproduced with the hatch off);
  * a mimic follower squeezes back instead of being pushed open;
  * the hatch OMNISIM_NEWTON_SERVO_ARMATURE_BOUND=0 leaves the model untouched;
  * a free-base robot is left alone by default and covered with SCOPE=all;
  * an explicit OMNISIM_NEWTON_TARGET_KD (an RL recipe's gain) skips the pass;
  * set_joint_effort_limit(0) takes the armature off and restoring torque puts
    it back.

The in-engine evidence (the Robonine pick through the gated /prompt, before and
after) is in social/launch/startup_outreach_2026-10-17/demos/robonine/EVIDENCE.md.

RUN:  python -m pytest tests/test_newton_servo_armature_bound.py -q
"""

import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")

_ENV = ("OMNISIM_NEWTON_SERVO_ARMATURE_BOUND", "OMNISIM_NEWTON_SERVO_ARMATURE_SCOPE",
        "OMNISIM_NEWTON_TARGET_KD", "OMNISIM_NEWTON_FINGER_KD", "OMNISIM_NEWTON_ARM_KD",
        "OMNISIM_NEWTON_TARGET_KE", "OMNISIM_NEWTON_FINGER_KE", "OMNISIM_NEWTON_SLIDER_GAIN_FLOOR")


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_servo_arm", _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                                    # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc


class _Env(object):
    def __init__(self, **kw):
        self.kw, self.old = kw, {}

    def __enter__(self):
        for k in _ENV:
            self.old[k] = os.environ.get(k)
            os.environ.pop(k, None)
        for k, v in self.kw.items():
            self.old.setdefault(k, os.environ.get(k))
            if v is not None:
                os.environ[k] = v
        return self

    def __exit__(self, *a):
        for k, v in self.old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


JAW_M = 0.0073
EFFORT = 100.0
KE, KD = EFFORT * 10.0, EFFORT * 0.5
JAW_TRAVEL = 0.034
# what the runtime builds the jaw with since 2026-10-22: a slider's gain is floored
# at effort/travel, kd scaled alike (tests/test_newton_slider_gain_floor.py)
KD_JAW = KD * max(1.0, (EFFORT / JAW_TRAVEL) / KE)
FACE_Q = 0.011      # each pad's inner face reaches the cube's face at this q


def _pinch(dt=0.002, mimic=False, fixed_base=True, lift=True):
    """Build and run the minimal pinch; return (world, slots, result dict)."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_solver_preference("mujoco")
    w.set_contact_cone("elliptic", 100.0)
    w.set_contact_condim(4)
    w.set_contact_solver_params(6.0, 8000.0, 200.0, 150, 50)
    floor = w.add_static_body(0.0, 0.0, -0.05)
    w.add_shape_box(floor, 0.5, 0.5, 0.05)
    cube = w.add_body(0.05, 0.0, 0.0, 0.0201, 0, 0, 0, 1, 1.33e-5, 1.33e-5, 1.33e-5)
    w.add_shape_box(cube, 0.02, 0.02, 0.02)
    pz = 0.072
    palm = w.add_body(0.02, 0.0, 0.0, pz, 0, 0, 0, 1, 1e-5, 1e-5, 1e-5)
    j1 = w.add_body(JAW_M, 0.0, 0.035, pz, 0, 0, 0, 1, 2e-6, 5e-6, 4e-6)
    w.add_shape_box(j1, 0.01, 0.004, 0.025, 0.0, 0.0, -0.04)
    j2 = w.add_body(JAW_M, 0.0, -0.035, pz, 0, 0, 0, 1, 2e-6, 5e-6, 4e-6)
    w.add_shape_box(j2, 0.01, 0.004, 0.025, 0.0, 0.0, -0.04)
    sp = None
    if fixed_base:
        base = w.add_static_body(0.0, 0.0, 0.3)
        sp = w.add_joint_prismatic(base, palm, 0, 0, 1, 0, 0, pz - 0.3, 0, 0, 0,
                                   KE, KD, -0.2, 0.2, EFFORT, 1.0)
    s1 = w.add_joint_prismatic(palm, j1, 0, -1, 0, 0, 0.035, 0, 0, 0, 0,
                               KE, KD, 0.0, 0.034, EFFORT, 1.0)
    if mimic:     # exactly what the engine registers for a URDF <mimic> follower
        s2 = w.add_joint_prismatic(palm, j2, 0, 1, 0, 0, -0.035, 0, 0, 0, 0,
                                   0.0, 0.0, 0.0, 0.034, 0.0, 1.0)
        w.add_joint_mimic(s2, s1, 1.0, 0.0)
    else:
        s2 = w.add_joint_prismatic(palm, j2, 0, 1, 0, 0, -0.035, 0, 0, 0, 0,
                                   KE, KD, 0.0, 0.034, EFFORT, 1.0)
    w.finalize()
    for s in (sp, s1, s2):
        if s is not None:
            w.set_joint_target_pos(s, 0.0)
            w.set_joint_target_vel(s, 0.0)
    w.step(dt)                         # the armature is added on the first step
    res = {"armature": dict(getattr(w, "_servo_armature_added", {}) or {})}
    if not lift:
        return w, (sp, s1, s2), res

    def run(sec):
        for _ in range(int(round(sec / dt))):
            w.step(dt)
    run(0.2)
    z0 = w.body_xform(cube)[2]
    w.set_joint_target_pos(s1, 0.034)
    if not mimic:
        w.set_joint_target_pos(s2, 0.034)
    run(0.6)
    q1, q2 = w.get_joint_angle(s1), w.get_joint_angle(s2)
    n = int(1.0 / dt)
    for k in range(n):
        w.set_joint_target_pos(sp, 0.10 * (k + 1) / n)
        w.step(dt)
    run(0.5)
    res.update(q1=q1, q2=q2, interference_mm=((q1 - FACE_Q) + (q2 - FACE_Q)) * 1000.0,
               asym_mm=abs(q1 - q2) * 1000.0, rise=w.body_xform(cube)[2] - z0)
    return w, (sp, s1, s2), res


def _jaw_dofs(w, slots):
    sv = w._mjc_solver()
    m = sv.mj_model
    qd = w.model.joint_qd_start.numpy()
    return m, [int(qd[w.slot_to_real_idx[s]]) for s in slots if s is not None]


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class ServoArmatureBound(unittest.TestCase):

    def test_env_value_parsing(self):
        f = _rt.World._servo_armature_bound
        for raw, want in (("", 1.0), ("0", None), ("off", None), ("False", None), ("no", None),
                          ("2.5", 2.5), ("-1", None), ("junk", None)):
            with _Env(OMNISIM_NEWTON_SERVO_ARMATURE_BOUND=raw):
                self.assertEqual(f(), want, raw)
        with _Env():
            self.assertEqual(f(), 1.0)
            self.assertEqual(_rt.World._servo_armature_scope(), "fixed")
        with _Env(OMNISIM_NEWTON_SERVO_ARMATURE_SCOPE="ALL"):
            self.assertEqual(_rt.World._servo_armature_scope(), "all")

    def test_armature_is_exactly_what_the_gain_implies(self):
        dt = 0.002
        with _Env():
            w, slots, res = _pinch(dt=dt, mimic=True, lift=False)
        m, dofs = _jaw_dofs(w, slots)
        added = res["armature"]
        _palm, leader, follower = dofs
        # leader jaw: M_ii is its own 7.3 g (a slider's diagonal is its body mass)
        self.assertAlmostEqual(added[leader], KD_JAW * dt - JAW_M, places=6)
        self.assertAlmostEqual(float(m.dof_armature[leader]), KD_JAW * dt - JAW_M, places=6)
        # mimic follower (multiplier 1): the leader's reflected inertia
        self.assertAlmostEqual(added[follower], added[leader], places=9)
        # and MuJoCo's constraint weights now describe the armed joint
        self.assertLess(float(m.dof_invweight0[leader]), 1.5 / (KD_JAW * dt))

    def test_driven_jaws_stop_at_the_part(self):
        # measured 2026-10-17 at the jaw gains of that day (ke = effort*10)
        with _Env(OMNISIM_NEWTON_SLIDER_GAIN_FLOOR="0"):
            _w, _s, fixed = _pinch(mimic=False)
        with _Env(OMNISIM_NEWTON_SLIDER_GAIN_FLOOR="0", OMNISIM_NEWTON_SERVO_ARMATURE_BOUND="0"):
            _w, _s, stale = _pinch(mimic=False)
        # the defect: the jaws close ~21 mm into a 40 mm part
        self.assertGreater(stale["interference_mm"], 15.0, stale)
        self.assertLess(fixed["interference_mm"], 4.0, fixed)
        self.assertGreater(fixed["rise"], 0.095, fixed)
        # and at today's default they still stop at the part. The floored jaw servo
        # (ke 2941 N/m, was 1000) squeezes ~61 N per jaw instead of ~21 N, so the
        # pads sit deeper in the contact: measured 4.65 mm, not the 21 mm pass-through
        with _Env():
            _w, _s, now = _pinch(mimic=False)
        self.assertLess(now["interference_mm"], 6.0, now)
        self.assertGreater(now["rise"], 0.095, now)

    def test_mimic_follower_squeezes_back(self):
        with _Env():
            _w, _s, fixed = _pinch(mimic=True)
        with _Env(OMNISIM_NEWTON_SERVO_ARMATURE_BOUND="0"):
            _w, _s, stale = _pinch(mimic=True)
        # without it the 7 g follower is pushed back to its own stop by the part
        self.assertGreater(stale["asym_mm"], 9.0, stale)
        self.assertLess(fixed["asym_mm"], 6.0, fixed)
        self.assertGreater(fixed["rise"], 0.095, fixed)

    def test_hatch_leaves_the_model_untouched(self):
        with _Env(OMNISIM_NEWTON_SERVO_ARMATURE_BOUND="0"):
            w, slots, res = _pinch(mimic=True, lift=False)
        m, dofs = _jaw_dofs(w, slots)
        self.assertEqual(res["armature"], {})
        for d in dofs:
            self.assertEqual(float(m.dof_armature[d]), 0.0)

    def test_free_base_is_left_alone_unless_scope_all(self):
        with _Env():
            _w, _s, res = _pinch(fixed_base=False, lift=False)
        self.assertEqual(res["armature"], {})
        with _Env(OMNISIM_NEWTON_SERVO_ARMATURE_SCOPE="all"):
            _w, _s, res = _pinch(fixed_base=False, lift=False)
        self.assertEqual(len(res["armature"]), 2)

    def test_explicit_gain_skips(self):
        with _Env(OMNISIM_NEWTON_TARGET_KD="50"):
            _w, _s, res = _pinch(lift=False)
        self.assertEqual(res["armature"], {})

    def test_gain_change_rejudges_the_armature(self):
        # review 2026-10-17: the pass is one-shot; a later setControlPID must not
        # leave armature sized for the old kd (nor drop the follower's)
        dt = 0.002
        with _Env():
            w, slots, _res = _pinch(dt=dt, mimic=True, lift=False)
            m, dofs = _jaw_dofs(w, slots)
            _palm, leader, follower = dofs
            self.assertEqual(w.set_joint_gains(slots[1], 0, KE, 4.0 * KD), 0)
            self.assertAlmostEqual(float(m.dof_armature[leader]), 4.0 * KD * dt - JAW_M, places=6)
            self.assertAlmostEqual(float(m.dof_armature[follower]), 4.0 * KD * dt - JAW_M, places=6)
            # a kd low enough to sit under the bound takes it off again
            self.assertEqual(w.set_joint_gains(slots[1], 0, KE, 0.5 * JAW_M / dt), 0)
            self.assertAlmostEqual(float(m.dof_armature[leader]), 0.0, places=9)
            self.assertAlmostEqual(float(m.dof_armature[follower]), 0.0, places=9)
            self.assertNotIn(leader, getattr(w, "_servo_armature_added", {}))

    def test_zero_effort_takes_the_armature_off(self):
        with _Env():
            w, slots, res = _pinch(mimic=False, lift=False)
            m, dofs = _jaw_dofs(w, slots)
            jaw = dofs[1]
            on = float(m.dof_armature[jaw])
            self.assertGreater(on, 0.09)
            self.assertEqual(w.set_joint_effort_limit(slots[1], 0, 0.0), 0)
            self.assertAlmostEqual(float(m.dof_armature[jaw]), 0.0, places=9)
            self.assertEqual(w.set_joint_effort_limit(slots[1], 0, EFFORT), 0)
            self.assertAlmostEqual(float(m.dof_armature[jaw]), on, places=9)


if __name__ == "__main__":
    unittest.main()
