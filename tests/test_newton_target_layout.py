"""Position targets land on the right joint whatever newton's target layout.

THE BUG (measured 2026-11-06, newton 1.7.0). From newton 1.6,
`newton.use_coord_layout_targets` defaults to True: `Control.joint_target_q`
becomes COORDINATE-shaped (matching `joint_q`) instead of DOF-shaped. Every
target index in the runtime is a DOF index, so on a robot with a FREE root
(7 coordinates, 6 DOFs) every position target landed on the NEXT joint, with no
error anywhere: on the G1 deploy world `left_hip_roll` took `left_hip_yaw`'s
target, median joint error 0.28 rad against 0.03-0.09 rad of run-to-run noise.
A BALL joint is worse: 3 DOF-layout values (ZYX Euler angles) against one
4-float quaternion.

THE FIX. `World._tp_get()` / `_tp_set()` keep every write site on a DOF-shaped
host array and translate at the boundary (`_target_pos_layout()`).

WHAT THIS PINS. The same robot -- a free-floating base, a hinge, a motorised
BallJoint, then a second hinge AFTER the ball (so it sits behind both the 7/6
root offset and the 4/3 ball offset) -- is built under BOTH layouts on whichever
newton is installed (1.5 already has the flag; 1.7 defaults it on). Each run
must drive the two hinges to their own targets, and the two layouts must
produce the same motion: identical hinge angles and the same ball rotation.
Before the fix the coordinate-layout run put hinge 2's target on the wrong slot.

Engine-free: the World API is driven directly on the CPU solver.
RUN:  python -m pytest tests/test_newton_target_layout.py -q
      OMNISIM_TEST_NEWTON_SITE=<site-packages> to test another bundle (e.g. a
      trial newton) instead of msys64/mingw64/bin/newton-runtime.
"""
import importlib.util
import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.environ.get("OMNISIM_TEST_NEWTON_RUNTIME") or os.path.join(
    _ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.environ.get("OMNISIM_TEST_NEWTON_SITE") or os.path.join(
    _ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_tlayout", _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                                    # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc

KE, KD, EFFORT = 400.0, 20.0, 200.0
HINGE1, HINGE2 = 0.5, -0.4
BALL = (0.3, -0.2, 0.25)


def _run(coord_layout, steps=600, dt=0.002):
    """Build the chain under one layout, drive it, return the measured state."""
    newton = _rt.newton
    saved = getattr(newton, "use_coord_layout_targets", None)
    newton.use_coord_layout_targets = bool(coord_layout)
    try:
        w = _rt.World()
        w.set_up_axis("Z")
        w.set_gravity(0.0, 0.0, 0.0)          # free base floats; angles are relative
        w.set_solver_preference("mujoco")
        inert = (2e-3, 2e-3, 2e-3)
        base = w.add_body(5.0, 0.0, 0.0, 1.0, 0, 0, 0, 1, 0.05, 0.05, 0.05)
        w.add_shape_box(base, 0.1, 0.1, 0.1)
        l1 = w.add_body(0.5, 0.0, 0.0, 0.8, 0, 0, 0, 1, *inert)
        w.add_shape_box(l1, 0.02, 0.02, 0.08)
        l2 = w.add_body(0.5, 0.0, 0.0, 0.6, 0, 0, 0, 1, *inert)
        w.add_shape_box(l2, 0.02, 0.02, 0.08)
        l3 = w.add_body(0.5, 0.0, 0.0, 0.4, 0, 0, 0, 1, *inert)
        w.add_shape_box(l3, 0.02, 0.02, 0.08)
        h1 = w.add_joint_revolute(base, l1, 0, 1, 0, 0, 0, -0.1, 0, 0, 0.1,
                                  KE, KD, -1.5, 1.5, EFFORT, 10.0)
        ball = w.add_joint_ball_motorized(
            l1, l2, (0.0, 0.0, -0.1), (0, 0, 0, 1), (0.0, 0.0, 0.1), (0, 0, 0, 1),
            [KE, KD] * 3, [-1.5, 1.5] * 3, [EFFORT] * 3, [10.0] * 3)   # flat per axis
        h2 = w.add_joint_revolute(l2, l3, 1, 0, 0, 0, 0, -0.1, 0, 0, 0.1,
                                  KE, KD, -1.5, 1.5, EFFORT, 10.0)
        w.finalize()
        layout = w._target_pos_layout()
        for _ in range(steps):
            w.set_joint_target_pos(h1, HINGE1)
            for d, v in enumerate(BALL):
                w.set_joint_target_pos(ball, v, d)
            w.set_joint_target_pos(h2, HINGE2)
            w.step(dt)
        return dict(layout=layout, h1=w.get_joint_angle(h1), h2=w.get_joint_angle(h2),
                    ball=w.get_joint_ball_quat(ball),
                    model_coord=bool(getattr(w.model, "use_coord_layout_targets", False)))
    finally:
        if saved is not None:
            newton.use_coord_layout_targets = saved


def _quat_angle(a, b):
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return 2.0 * math.degrees(math.acos(min(1.0, dot)))


@unittest.skipIf(_rt is None, "newton runtime not importable: %r" % (_IMPORT_ERROR,))
class TargetLayout(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.dof = _run(coord_layout=False)
        cls.coord = _run(coord_layout=True)

    def test_layouts_are_the_ones_asked_for(self):
        self.assertFalse(self.dof["model_coord"])
        self.assertTrue(self.coord["model_coord"])
        self.assertIsNone(self.dof["layout"], "DOF layout needs no translation")
        self.assertIsNotNone(self.coord["layout"])
        self.assertEqual(len(self.coord["layout"][2]), 1, "one BALL joint mapped")

    def test_each_hinge_reaches_its_own_target(self):
        for r, name in ((self.dof, "dof"), (self.coord, "coord")):
            self.assertAlmostEqual(r["h1"], HINGE1, delta=0.02, msg="%s hinge1 %r" % (name, r))
            self.assertAlmostEqual(r["h2"], HINGE2, delta=0.02, msg="%s hinge2 %r" % (name, r))

    def test_both_layouts_move_the_robot_identically(self):
        self.assertAlmostEqual(self.dof["h1"], self.coord["h1"], delta=1e-3)
        self.assertAlmostEqual(self.dof["h2"], self.coord["h2"], delta=1e-3)
        self.assertLess(_quat_angle(self.dof["ball"], self.coord["ball"]), 0.5,
                        "ball rotation differs between layouts: %r vs %r"
                        % (self.dof["ball"], self.coord["ball"]))

    def test_the_ball_actually_turned(self):
        # Guards the comparison above from passing on two IDENTITY quaternions.
        self.assertGreater(_quat_angle(self.coord["ball"], (0.0, 0.0, 0.0, 1.0)), 10.0)


if __name__ == "__main__":
    unittest.main()
