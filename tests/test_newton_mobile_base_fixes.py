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

"""Mobile-base defects found running published robots (2026-10-03).

WHAT THIS PINS (runtime half, engine-free, on the SOURCE runtime)
------------------------------------------------------------------
A. A wheel declared ``type="revolute"`` with ``lower="-1e+16" upper="1e+16"``
   (the ROS/Gazebo spelling of "unlimited"; Neobotix ROX diff and MP-400) must
   drive like a ``continuous`` wheel. The engine registered it as a position
   servo (ke = effort*10, held at its start angle) and it spun at 0.15 % of its
   command. Measured below on a ROX-like rover: 0.0167 -> 0.9973.
C. The 1 mm "Robot wrapper placeholder" sphere OmSolid.cpp gives a Robot with
   no collider of its own must not carry contact when the robot's links
   collide. A robot rooted at ground level (base_footprint) or below its axles
   (Robotnik RB-ROBOUT / RB-KAIROS) otherwise rests on it: measured below,
   chassis travel 0.000 -> 0.992 of the commanded distance.
E. Motor.setAvailableTorque(0) must make a joint passive. It reached the
   solver as nothing at all, leaving a velocity servo targeting 0 -- a brake.
   The runtime half is World.set_joint_effort_limit (the engine half, which
   calls it from OmBasicJoint::pushNewtonMotorTargets, needs a C++ rebuild).

Each behavioural assertion was checked to FAIL with its hatch set (the hatch
reproduces the defect exactly), so none of them can pass vacuously.

The C++ halves (B: Mesh.scale + the URDF importer; D: the webots:// scheme
alias; the engine halves of A and E) cannot run without a rebuilt engine; they
are pinned at the source level at the bottom of this file.

RUN:  python -m pytest tests/test_newton_mobile_base_fixes.py -q
"""

import importlib.util
import math
import os
import re
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")

_HATCHES = ("OMNISIM_NEWTON_UNBOUNDED_LIMIT_AS_CONTINUOUS",
            "OMNISIM_NEWTON_WRAPPER_PLACEHOLDER_COLLIDES")


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_mobile_base",
                                                  _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                                    # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc


class _Env(object):
    """Set env vars for one block, restoring the previous values after."""

    def __init__(self, **kw):
        self.kw, self.old = kw, {}

    def __enter__(self):
        for k, v in self.kw.items():
            self.old[k] = os.environ.get(k)
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        return self

    def __exit__(self, *a):
        for k, v in self.old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v


def _rover(huge_limits, placeholder, seconds=2.0):
    """A ROX-diff-like rover: 140 kg chassis whose ORIGIN is at ground level,
    wheels r=0.075 at y=+-0.317 (sphere colliders), a free rear caster wheel.
    Joint registration mirrors what OmBasicJoint sends for a URDF revolute
    with limit +-1e16, effort 1000 (ke=effort*10, kd=effort*0.5); the engine
    pushes position target 0 (the motor default) and then velocity targets.
    Returns (chassis travel / commanded, wheel spin / commanded)."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_solver_preference("mujoco")
    floor = w.add_static_body(0.0, 0.0, -0.05)
    w.add_shape_box(floor, 20.0, 20.0, 0.05)
    z0 = 0.002
    ch = w.add_body(140.0, 0.0, 0.0, z0, 0, 0, 0, 1, 5.0, 5.0, 8.0)
    if placeholder:
        w.add_shape_sphere(ch, 0.001)          # exactly OmSolid.cpp's placeholder call
    cas = w.add_body(1.0, -0.3, 0.0, z0 + 0.05, 0, 0, 0, 1, 0.001, 0.001, 0.001)
    w.add_shape_sphere(cas, 0.05)
    slots = []
    for y in (0.317, -0.317):
        wh = w.add_body(5.0, 0.0, y, z0 + 0.075, 0, 0, 0, 1, 0.02888, 0.05625, 0.02888)
        w.add_shape_sphere(wh, 0.075)
        ke, kd, lo, hi = (10000.0, 500.0, -1e16, 1e16) if huge_limits else (0.0, 500.0, 0.0, 0.0)
        slots.append(w.add_joint_revolute(ch, wh, 0.0, 1.0, 0.0, 0.0, y, 0.075, 0.0, 0.0, 0.0,
                                          ke, kd, lo, hi, 1000.0, 100.0))
    w.add_joint_revolute(ch, cas, 0.0, 1.0, 0.0, -0.3, 0.0, 0.05, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    w.finalize()
    dt = 0.004
    for s in slots:
        w.set_joint_target_pos(s, 0.0)
        w.set_joint_target_vel(s, 0.0)
    for _ in range(75):
        w.step(dt)
    x0 = w.body_xform(ch)[0]
    q0 = [w.get_joint_angle(s) for s in slots]
    cmd = 0.3 / 0.075
    for s in slots:
        w.set_joint_target_vel(s, cmd)
    n = int(seconds / dt)
    for _ in range(n):
        w.step(dt)
    T = n * dt
    travel = (w.body_xform(ch)[0] - x0) / (0.3 * T)
    spin = min((w.get_joint_angle(s) - a) / (cmd * T) for s, a in zip(slots, q0))
    return w, travel, spin


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class UnboundedLimitWheels(unittest.TestCase):
    """A."""

    def setUp(self):
        self._env = _Env(**{k: None for k in _HATCHES})
        self._env.__enter__()

    def tearDown(self):
        self._env.__exit__()

    def test_pm_1e16_wheel_drives_like_continuous(self):
        _w, travel, spin = _rover(huge_limits=True, placeholder=False)
        self.assertGreater(spin, 0.95, "a +-1e16 'revolute' wheel must track its velocity command")
        self.assertGreater(travel, 0.9, "and the rover must actually travel")
        self.assertIn("unbounded_limits", _w._engine_notes)
        self.assertIn("registered as limit-less", _w._fin_report())

    def test_hatch_reproduces_the_pinned_wheel(self):
        with _Env(OMNISIM_NEWTON_UNBOUNDED_LIMIT_AS_CONTINUOUS="0"):
            _w, _travel, spin = _rover(huge_limits=True, placeholder=False)
        self.assertLess(spin, 0.1, "the hatch must restore the position-servo wheel (the defect)")

    def test_spec_normalisation(self):
        w = _rt.World.__new__(_rt.World)
        rev = dict(kind="revolute", target_ke=10000.0, target_kd=500.0,
                   limit_lower=-1e16, limit_upper=1e16)
        w._normalise_unbounded_limits(rev)
        self.assertEqual((rev["limit_lower"], rev["limit_upper"]), (0.0, 0.0))
        self.assertEqual((rev["target_ke"], rev["target_kd"]), (0.0, 500.0))
        # a real range is untouched, however large a multi-turn joint gets
        real = dict(kind="revolute", target_ke=200.0, target_kd=20.0,
                    limit_lower=-100.0, limit_upper=100.0)
        w._normalise_unbounded_limits(real)
        self.assertEqual((real["limit_lower"], real["target_ke"]), (-100.0, 200.0))
        # a slider keeps its gains (every slider is position-controlled on purpose)
        sl = dict(kind="prismatic", target_ke=500.0, target_kd=25.0,
                  limit_lower=-1e16, limit_upper=1e16)
        w._normalise_unbounded_limits(sl)
        self.assertEqual((sl["limit_lower"], sl["target_ke"], sl["target_kd"]), (0.0, 500.0, 25.0))
        # one-sided huge range is NOT "unlimited"
        one = dict(kind="revolute", target_ke=200.0, target_kd=20.0,
                   limit_lower=0.0, limit_upper=1e16)
        w._normalise_unbounded_limits(one)
        self.assertEqual(one["limit_upper"], 1e16)


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class WrapperPlaceholder(unittest.TestCase):
    """C."""

    def setUp(self):
        self._env = _Env(**{k: None for k in _HATCHES})
        self._env.__enter__()

    def tearDown(self):
        self._env.__exit__()

    def test_placeholder_does_not_carry_a_wheeled_robot(self):
        w, travel, spin = _rover(huge_limits=False, placeholder=True)
        self.assertGreater(spin, 0.95)
        self.assertGreater(travel, 0.9, "the robot must roll on its wheels, not sit on the 1 mm sphere")
        self.assertIn("wrapper_placeholder", w._engine_notes)

    def test_hatch_reproduces_the_robot_on_its_placeholder(self):
        with _Env(OMNISIM_NEWTON_WRAPPER_PLACEHOLDER_COLLIDES="1"):
            _w, travel, spin = _rover(huge_limits=False, placeholder=True)
        self.assertGreater(spin, 0.95, "wheels spin...")
        self.assertLess(travel, 0.1, "...in the air: the defect the hatch must reproduce")

    def test_lone_wrapper_keeps_its_placeholder(self):
        """A wrapper with no colliding descendants keeps the sphere: there it is
        the only thing between the robot and the floor."""
        w = _rt.World()
        w.set_up_axis("Z")
        w.set_gravity(0.0, 0.0, -9.81)
        w.set_solver_preference("mujoco")
        floor = w.add_static_body(0.0, 0.0, -0.05)
        w.add_shape_box(floor, 2.0, 2.0, 0.05)
        b = w.add_body(1.0, 0.0, 0.0, 0.05, 0, 0, 0, 1, 0.01, 0.01, 0.01)
        sid = w.add_shape_sphere(b, 0.001)
        w.finalize()
        for _ in range(250):
            w.step(0.004)
        self.assertGreater(w.body_xform(b)[2], -0.01, "it must still rest on the floor")
        self.assertNotIn("wrapper_placeholder", getattr(w, "_engine_notes", {}) or {})
        self.assertTrue(int(w.builder.shape_flags[sid]) & int(_rt.newton.ShapeFlags.COLLIDE_SHAPES))

    def test_only_the_exact_placeholder_call_is_recognised(self):
        w = _rt.World()
        b = w.add_body(1.0, 0.0, 0.0, 0.0)
        w.add_shape_sphere(b, 0.001, 0.0, 0.0, 0.01)   # offset: an authored collider
        w.add_shape_sphere(b, 0.002)                   # other radius
        w.add_shape_sphere(b, 0.001, mu=0.5)           # authored friction
        self.assertFalse(getattr(w, "_placeholder_shapes", {}))


def _pendulum(passive, restore=False):
    """1 kg at 0.3 m on a horizontal hinge, released 0.5 rad from the bottom;
    its motor is a velocity servo targeting 0 (kd=500) -- a brake."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_solver_preference("mujoco")
    base = w.add_static_body(0.0, 0.0, 1.0)
    a0 = 0.5
    bob = w.add_body(1.0, 0.3 * math.sin(a0), 0.0, 1.0 - 0.3 * math.cos(a0), 0, 0, 0, 1,
                     0.001, 0.001, 0.001)
    w.add_shape_sphere(bob, 0.02)
    slot = w.add_joint_revolute(base, bob, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                -0.3 * math.sin(a0), 0.0, 0.3 * math.cos(a0),
                                0.0, 500.0, 0.0, 0.0, 10.0, 10.0)
    w.finalize()
    w.set_joint_target_vel(slot, 0.0)
    w.step(0.002)                      # the wheel armature is added on the first step
    rcs = []
    if passive:
        rcs.append(w.set_joint_effort_limit(slot, 0, 0.0))
    if restore:
        rcs.append(w.set_joint_effort_limit(slot, 0, 10.0))
    qs = [w.get_joint_angle(slot)]
    for _ in range(500):
        w.step(0.002)
        qs.append(w.get_joint_angle(slot))
    return rcs, max(qs) - min(qs)


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class AvailableTorque(unittest.TestCase):
    """E (runtime half)."""

    def test_full_torque_brakes(self):
        _rcs, swing = _pendulum(passive=False)
        self.assertLess(swing, 0.05, "control: a velocity servo at 0 holds the pendulum")

    def test_zero_available_torque_is_passive(self):
        rcs, swing = _pendulum(passive=True)
        self.assertEqual(rcs, [0])
        # free: swings from +0.5 to about -0.5 rad within one second (period ~1.1 s);
        # with only the force clamp (gains/armature left on) it covered 0.52 rad.
        self.assertGreater(swing, 0.9)

    def test_restoring_torque_brakes_again(self):
        rcs, swing = _pendulum(passive=True, restore=True)
        self.assertEqual(rcs, [0, 0])
        self.assertLess(swing, 0.05)

    def test_unknown_slot_is_not_applicable(self):
        w = _rt.World()
        self.assertEqual(w.set_joint_effort_limit(99, 0, 0.0), -1)


# ---------------------------------------------------------------- C++ source pins
def _src(rel):
    with open(os.path.join(_ROOT, *rel.split("/")), encoding="utf-8") as f:
        return f.read()


class EngineHalvesSourcePins(unittest.TestCase):
    """The engine halves need a rebuilt omnisim-bin to run; pin their shape so
    a merge cannot drop them silently. Behaviour is verified by the engine runs
    named in each fix's comment once the binary is rebuilt."""

    def test_b_mesh_has_a_scale_field(self):
        self.assertRegex(_src("resources/nodes/Mesh.wrl"), r"SFVec3f\s+scale\s+1 1 1")
        mesh = _src("src/omnisim/nodes/OmMesh.cpp")
        self.assertIn('findSFVector3("scale")', mesh)
        self.assertIn("vertice[0] * meshScale.x()", mesh)
        self.assertIn("|scale=", mesh, "a scaled mesh must not share a cache key with the unit one")

    def test_b_importer_writes_collision_mesh_scale(self):
        imp = _src("src/omnisim/vrml/OmUrdfImporter.cpp")
        self.assertIn("OMNISIM_URDF_COLLISION_MESH_SCALE", imp)
        self.assertIn('Mesh { url \\"%1\\"%2 }', imp)
        self.assertIn("boundingObject Mesh { url \\\"%1\\\"%2 }", imp)

    def test_d_webots_scheme_is_an_alias(self):
        url = _src("src/omnisim/vrml/OmUrl.cpp")
        self.assertIn("QString OmUrl::normalizeLegacyScheme", url)
        self.assertIn("OMNISIM_LEGACY_WEBOTS_SCHEME", url)
        self.assertRegex(url, r"QString url = normalizeLegacyScheme\(rawUrl\);[\s\S]*combinePaths|"
                              r"combinePaths[\s\S]*QString url = normalizeLegacyScheme\(rawUrl\);")
        for f in ("OmTokenizer.cpp", "OmProtoModel.cpp"):
            self.assertIn('replace(QString("webots://")', _src("src/omnisim/vrml/" + f))
        self.assertIn("normalizeLegacyScheme(match.captured(1))", _src("src/omnisim/vrml/OmProtoManager.cpp"))

    def test_a_engine_classifies_unbounded_hinge_as_limitless(self):
        bj = _src("src/omnisim/nodes/OmBasicJoint.cpp")
        self.assertIn("isUnboundedLimitRange(limitLower, limitUpper)", bj)
        self.assertRegex(bj, r"limitLower != limitUpper && !unboundedHinge")
        # the engine and the runtime must agree on the threshold
        self.assertIn("lower <= -1.0e6 && upper >= 1.0e6", bj)
        self.assertIn("_UNBOUNDED_LIMIT = 1.0e6", _src("src/omnisim/physics/omnisim_newton_runtime.py"))

    def test_e_available_torque_reaches_the_runtime(self):
        bj = _src("src/omnisim/nodes/OmBasicJoint.cpp")
        self.assertIn("newton->setJointEffortLimit(p->mNewtonJointIndex, 0, avail)", bj)
        self.assertIn("pushedAvailableTorque().clear()", bj)
        self.assertIn('"set_joint_effort_limit"', _src("src/omnisim/physics/OmNewtonBackend.cpp"))
        self.assertIn("availableForceOrTorque()", _src("src/omnisim/nodes/OmMotor.hpp"))


if __name__ == "__main__":
    unittest.main()
