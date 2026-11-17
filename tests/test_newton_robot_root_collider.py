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

"""A Robot's ROOT collider registers by default unless it would lift the robot
off its own wheels (2026-10-17).

WHY THIS EXISTS
---------------
OmSolid.cpp used to give every Robot wrapper with colliding descendants a 1 mm
placeholder INSTEAD of its own boundingObject unless WorldInfo.newtonRobotColliders
TRUE -- so a URDF robot's root-link <collision> did not exist in the solver, and the
log told the author to declare a boundingObject the Solid already declared.
Measured on Preferred Robotics' Kachaka (chassis box underside exactly on the tyre
contact plane, no caster): by default the published file tipped onto its nose
(pitch 79 deg) where OmniSim-with-colliders and plain MuJoCo both drag the box.

The engine now registers the wrapper's own shapes and brackets them
(World.mark_robot_root_shapes); at finalize World._resolve_robot_root_shapes keeps
them colliding unless their lowest point is more than 2 mm below the lowest point
of the robot's own descendant colliders at the spawn pose -- the one envelope that
truly stops a robot driving (every wheel held off the floor).

This pins the runtime half engine-free on the SOURCE runtime, with a two-wheeled
base whose chassis box is placed above, level with, and below the tyre plane, plus
a mesh chassis. The engine half (the bracket call and the hatch
OMNISIM_NEWTON_ROBOT_ROOT_COLLIDER) is pinned by its source shape below and by the
engine dump test tests/test_newton_urdf_fixed_links.py.

RUN:  python -m pytest tests/test_newton_robot_root_collider.py -q
"""

import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_SOLID = os.path.join(_ROOT, "src", "omnisim", "nodes", "OmSolid.cpp")
_BACKEND = os.path.join(_ROOT, "src", "omnisim", "physics", "OmNewtonBackend.cpp")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")


def _load_runtime():
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_root_coll", _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                                    # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc

WHEEL_R = 0.045


def _base(box_bottom, mesh=False, bracket=True, drive=0.0, seconds=0.0, arm_low=None):
    """A 10 kg chassis box (0.39 x 0.24 x 0.095, the Kachaka's) on two r=0.045
    wheels at y=+-0.10, tyre plane z=0. ``box_bottom`` sets the box underside.
    Returns (world, chassis body, root shape ids, x travelled)."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_solver_preference("mujoco")
    floor = w.add_static_body(0.0, 0.0, -0.05)
    w.add_shape_box(floor, 2.0, 2.0, 0.05)
    hz = 0.0475
    ch = w.add_body(10.0, 0.0, 0.0, WHEEL_R, 0, 0, 0, 1, 0.06, 0.13, 0.17)
    if bracket:
        w.mark_robot_root_shapes(ch, 0)
    cz = box_bottom + hz - WHEEL_R          # box centre in the chassis frame (origin at the axle)
    if mesh:
        hx, hy = 0.1935, 0.12
        v = [(sx * hx, sy * hy, cz + sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        flat = [c for p in v for c in p]
        tris = [0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5, 0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3]
        w.add_shape_mesh(ch, flat, tris, len(v))
    else:
        w.add_shape_box(ch, 0.1935, 0.12, hz, 0.0435, 0.0, cz)
    if bracket:
        w.mark_robot_root_shapes(ch, 1)
    slots = []
    for y in (0.10, -0.10):
        wh = w.add_body(0.5, 0.0, y, WHEEL_R, 0, 0, 0, 1, 2.5e-4, 5e-4, 2.5e-4)
        w.add_shape_sphere(wh, WHEEL_R)
        slots.append(w.add_joint_revolute(ch, wh, 0.0, 1.0, 0.0, 0.0, y, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 500.0, 0.0, 0.0, 20.0, 50.0))
    if arm_low is not None:
        # a two-link "arm" hanging in front of the chassis, its tip at z=arm_low
        l1 = w.add_body(0.3, 0.15, 0.0, WHEEL_R, 0, 0, 0, 1, 1e-4, 1e-4, 1e-4)
        w.add_shape_sphere(l1, 0.01)
        w.add_joint_revolute(ch, l1, 0.0, 1.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        l2 = w.add_body(0.2, 0.15, 0.0, arm_low + 0.01, 0, 0, 0, 1, 1e-4, 1e-4, 1e-4)
        w.add_shape_sphere(l2, 0.01)
        w.add_joint_revolute(l1, l2, 0.0, 1.0, 0.0, 0.0, 0.0, arm_low + 0.01 - WHEEL_R, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    w.finalize()
    roots = list((getattr(w, "_robot_root_shapes", {}) or {}).get(ch, []))
    x0 = w.body_xform(ch)[0]
    for s in slots:
        w.set_joint_target_vel(s, drive)
    for _ in range(int(seconds / 0.002)):
        w.step(0.002)
    return w, ch, roots, w.body_xform(ch)[0] - x0


def _collides(w, sids):
    collide = int(_rt.newton.ShapeFlags.COLLIDE_SHAPES)
    return [bool(int(w.model.shape_flags.numpy()[s]) & collide) for s in sids]


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class RootColliderRule(unittest.TestCase):

    def test_chassis_above_the_tyre_plane_is_kept(self):
        w, _ch, roots, _x = _base(box_bottom=0.020)
        self.assertEqual(len(roots), 1)
        self.assertEqual(_collides(w, roots), [True])
        self.assertIn("robot_root_collider", getattr(w, "_engine_notes", {}))

    def test_chassis_level_with_the_tyre_plane_is_kept(self):
        # the Kachaka case: the file says the box rests on the floor -- so it drags
        w, _ch, roots, x = _base(box_bottom=0.0, drive=5.78, seconds=1.0)
        self.assertEqual(_collides(w, roots), [True])
        self.assertLess(x, 0.8 * 5.78 * WHEEL_R * 1.0, "a box on the floor must drag the base")

    def test_chassis_below_the_wheels_is_quieted(self):
        w, _ch, roots, _x = _base(box_bottom=-0.010)
        self.assertEqual(_collides(w, roots), [False])
        note = getattr(w, "_engine_notes", {}).get("robot_root_collider_quiet", "")
        self.assertIn("10.0 mm below", note)
        self.assertIn("newtonRobotColliders TRUE", note)

    def test_tolerance_is_two_millimetres(self):
        w, _c, roots, _x = _base(box_bottom=-0.0015)
        self.assertEqual(_collides(w, roots), [True])
        w, _c, roots, _x = _base(box_bottom=-0.0025)
        self.assertEqual(_collides(w, roots), [False])

    def test_mesh_chassis_is_judged_from_its_vertices(self):
        w, _c, roots, _x = _base(box_bottom=0.010, mesh=True)
        self.assertEqual(_collides(w, roots), [True])
        w, _c, roots, _x = _base(box_bottom=-0.010, mesh=True)
        self.assertEqual(_collides(w, roots), [False])

    def test_unbracketed_shapes_are_never_touched(self):
        # an old engine (no bracket call) keeps exactly today's behaviour
        w, _ch, roots, _x = _base(box_bottom=-0.010, bracket=False)
        self.assertEqual(roots, [])
        self.assertNotIn("robot_root_collider_quiet", getattr(w, "_engine_notes", {}) or {})

    def test_a_lone_low_branch_does_not_set_the_reference(self):
        # review 2026-10-17: an arm hanging 5 cm below the tyres at spawn must not
        # pull the reference down and let a below-the-wheels chassis box stay on
        w, _c, roots, _x = _base(box_bottom=-0.010, arm_low=-0.05)
        self.assertEqual(_collides(w, roots), [False])
        w, _c, roots, _x = _base(box_bottom=0.0, arm_low=-0.05)
        self.assertEqual(_collides(w, roots), [True])

    def test_support_level_needs_two_branches(self):
        lvl = _rt.World._support_level
        self.assertEqual(lvl([-0.05, 0.0, 0.0004]), 0.0)     # an arm below a pair of wheels
        self.assertEqual(lvl([0.0, 0.0]), 0.0)
        self.assertEqual(lvl([0.03]), 0.03)                  # a single-branch robot
        self.assertEqual(lvl([0.0, 0.2, 0.4]), 0.0)          # no pair: plain minimum

    def test_bracket_rejects_use_after_finalize(self):
        w, ch, _r, _x = _base(box_bottom=0.02)
        self.assertEqual(w.mark_robot_root_shapes(ch, 0), -1)


def _gripper(seconds=1.0, lidar=False):
    """The Xense TacCap follower as the 2026-10-22 weighing rig stood it: a
    0.415 kg base on a 60 x 60 x 4 mm foot box, two 0.367 kg mirror fingers on
    limited hinges whose lowest points sit 31.5 mm above the foot. Free body on
    a floor; returns (world, base body, root shape ids, base z after ``seconds``)."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_solver_preference("mujoco")
    floor = w.add_static_body(0.0, 0.0, -0.05)
    w.add_shape_box(floor, 0.5, 0.5, 0.05)
    bz = 0.0021
    base = w.add_body(0.415, 0.0, 0.0, bz, 0, 0, 0, 1, 4e-4, 4e-4, 4e-4)
    w.mark_robot_root_shapes(base, 0)
    w.add_shape_box(base, 0.03, 0.03, 0.002)
    w.mark_robot_root_shapes(base, 1)
    for y, lo, hi in ((0.02, -0.505, 0.0), (-0.02, 0.0, 0.505)):
        f = w.add_body(0.367, 0.0, y, bz + 0.0485, 0, 0, 0, 1, 1e-3, 1e-3, 2e-4)
        w.add_shape_box(f, 0.008, 0.008, 0.015, 0.0, 0.0, 0.0)   # lowest point 31.5 mm above the foot
        w.add_joint_revolute(base, f, 1.0, 0.0, 0.0, 0.0, y, 0.0485, 0.0, 0.0, 0.0,
                             11.0, 0.55, lo, hi, 1.1, 1.0)
    if lidar:
        # a limit-less joint spinning about the UP axis rolls nothing
        sp = w.add_body(0.05, 0.0, 0.0, bz + 0.035, 0, 0, 0, 1, 1e-5, 1e-5, 1e-5)
        w.add_shape_sphere(sp, 0.005)
        w.add_joint_revolute(base, sp, 0.0, 0.0, 1.0, 0.0, 0.0, 0.035, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    w.finalize()
    roots = list((getattr(w, "_robot_root_shapes", {}) or {}).get(base, []))
    for _ in range(int(seconds / 0.002)):
        w.step(0.002)
    return w, base, roots, w.body_xform(base)[2]


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class FingersAreNotFeet(unittest.TestCase):
    """2026-10-22: only a branch that ROLLS is a ground contact. v9.2.7 counted the
    Xense gripper's two mirror fingers as feet, quieted its base foot ("31.5 mm
    below") and the gripper fell through the weighing plate."""

    def test_a_gripper_standing_on_its_base_keeps_it(self):
        w, _b, roots, z = _gripper()
        self.assertEqual(_collides(w, roots), [True])
        self.assertGreater(z, -0.005, "the gripper fell through the floor")
        self.assertNotIn("robot_root_collider_quiet", getattr(w, "_engine_notes", {}) or {})
        self.assertIn("no wheels", w._engine_notes.get("robot_root_collider", ""))

    def test_a_vertical_spinner_is_not_a_wheel(self):
        w, _b, roots, z = _gripper(lidar=True)
        self.assertEqual(_collides(w, roots), [True])
        self.assertGreater(z, -0.005)

    def test_joint_rolls(self):
        w = _rt.World()
        up = (0.0, 0.0, 1.0)
        rev = dict(kind="revolute", parent=-1, axis=(0.0, 1.0, 0.0), limit_lower=0.0, limit_upper=0.0)
        self.assertTrue(w._joint_rolls(rev, up))                                         # wheel
        self.assertTrue(w._joint_rolls(dict(rev, limit_lower=-1e16, limit_upper=1e16), up))  # ROS "no limit"
        self.assertFalse(w._joint_rolls(dict(rev, limit_lower=-0.5, limit_upper=0.0), up))  # finger / knee
        self.assertFalse(w._joint_rolls(dict(rev, axis=(0.0, 0.0, 1.0)), up))              # lidar / swivel
        self.assertFalse(w._joint_rolls(dict(rev, kind="prismatic"), up))
        self.assertTrue(w._joint_rolls(dict(kind="hinge2", parent=-1, axis=(0.0, 0.0, 1.0),
                                            axis2=(0.0, 1.0, 0.0)), up))                 # car wheel


class EngineHalfShape(unittest.TestCase):
    """Source pins for the engine half (behaviour: tests/test_newton_urdf_fixed_links.py)."""

    def test_solid_brackets_the_wrapper_bounding_object_behind_the_hatch(self):
        src = open(_SOLID, encoding="utf-8").read()
        self.assertIn('newtonEnvFlag("OMNISIM_NEWTON_ROBOT_ROOT_COLLIDER", true)', src)
        self.assertIn("newton->markRobotRootShapes(idx, 0) == 0", src)
        self.assertIn("newton->markRobotRootShapes(idx, 1);", src)

    def test_warning_no_longer_blames_a_missing_bounding_object(self):
        src = open(_SOLID, encoding="utf-8").read()
        self.assertIn("declares a boundingObject (on a URDF robot: its root link's <collision>), but", src)
        # also when fixed-child colliders attached (shapeDesc names no placeholder then)
        self.assertIn("!useBoundingObject && wrapperDeclaresBo && !wrapperRootRegistered", src)

    def test_backend_degrades_on_an_old_bundle(self):
        src = open(_BACKEND, encoding="utf-8").read()
        self.assertIn('PyObject_HasAttrString(mRuntime->world, "mark_robot_root_shapes")', src)


if __name__ == "__main__":
    unittest.main()
