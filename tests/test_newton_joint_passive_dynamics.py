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

"""A joint's declared damping / Coulomb friction reaches the MuJoCo model.

WHAT THIS PINS
--------------
Until 2026-10-03 JointParameters.dampingConstant and .staticFriction -- which
is where OmUrdfImporter puts a URDF ``<dynamics damping friction>`` -- never
reached the solver: OmBasicJoint::updateSpringAndDampingConstants() was an
empty ODE-era sink and nothing else read them. A published Raspberry Pi Mouse
URDF (damping="1.0" friction="1.0" on both wheels) compiled to

    dof 6 damping=0.00 armature=0.0000 frictionloss=0.000

(social/launch/company_outreach_2026-10-03/rtcorp/rig/results/
pimouse_square_rcF_dt2_mu1p0.mjmodel.txt).

The runtime half is ``World.set_joint_passive_dynamics(slot, damping,
friction)``, a separate verb the engine calls after add_joint_revolute /
add_joint_prismatic for a joint that declares a non-zero value (so a binary and
a runtime of different vintages can never shift a positional argument). This
file pins that half:

* the values ride the queued spec into newton's builder kwargs;
* a joint that declares nothing gets NO damping/friction kwarg (byte-identical
  to before);
* OMNISIM_NEWTON_JOINT_DYNAMICS=0 keeps them out (the revert hatch);
* a bad slot is refused, not silently applied to another joint;
* end to end through REAL newton + SolverMuJoCo (CPU): the builder kwargs land
  in ``mj_model.dof_damping`` / ``dof_frictionloss``.

The engine half (OmBasicJoint reading the JointParameters and calling
OmNewtonBackend::setJointPassiveDynamics) needs a rebuilt engine; it is pinned
by tests/test_newton_urdf_fixed_links.py::test_urdf_wheel_dynamics_reach_mujoco.

RUN:  python -m pytest tests/test_newton_joint_passive_dynamics.py -q
"""

import importlib.util
import os
import unittest

import pytest

pytest.importorskip("warp", reason="needs the GPU physics stack (warp)")
pytest.importorskip("newton", reason="needs the GPU physics stack (newton)")

_HERE = os.path.dirname(os.path.abspath(__file__))
_RUNTIME = os.path.join(_HERE, os.pardir, "src", "omnisim", "physics",
                        "omnisim_newton_runtime.py")


def _load_runtime():
    """Import the SOURCE runtime by path, never the (possibly stale) bundle copy."""
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_passive_dyn",
                                                  os.path.normpath(_RUNTIME))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _RecordingBuilder(object):
    """Stands in for newton.ModelBuilder: records every joint call's kwargs."""

    def __init__(self):
        self.calls = []

    def _record(self, kind, kw):
        self.calls.append((kind, kw))
        return len(self.calls) - 1

    def add_joint_prismatic(self, **kw):
        return self._record("prismatic", kw)

    def add_joint_revolute(self, **kw):
        return self._record("revolute", kw)


# A velocity-driven wheel exactly as OmBasicJoint registers it (ke=0, kd=500,
# no limits), the Pi Mouse case.
WHEEL = (0, 1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0, 0.0, 0.0, 70.0, 90.0)


class JointPassiveDynamicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = _load_runtime()

    def setUp(self):
        self._saved = os.environ.pop("OMNISIM_NEWTON_JOINT_DYNAMICS", None)
        # Default is OFF since the 2026-10-03 integration re-check; these cases
        # test the opted-in path, test_default_is_off pins the default.
        os.environ["OMNISIM_NEWTON_JOINT_DYNAMICS"] = "1"

    def tearDown(self):
        os.environ.pop("OMNISIM_NEWTON_JOINT_DYNAMICS", None)
        if self._saved is not None:
            os.environ["OMNISIM_NEWTON_JOINT_DYNAMICS"] = self._saved

    def _world(self):
        World = self.mod.World
        w = World.__new__(World)
        w.builder = _RecordingBuilder()
        w.pending_revolutes = []
        return w

    def _build_last(self, w):
        w._add_revolute_to_builder(w.pending_revolutes[-1])
        return w.builder.calls[-1]

    def test_declared_dynamics_reach_the_builder(self):
        w = self._world()
        slot = w.add_joint_revolute(*WHEEL)
        self.assertEqual(w.set_joint_passive_dynamics(slot, 1.0, 1.0), 0)
        kind, kw = self._build_last(w)
        self.assertEqual(kind, "revolute")
        self.assertAlmostEqual(kw.get("damping", 0.0), 1.0,
                               msg="URDF <dynamics damping> must reach newton's joint damping")
        self.assertAlmostEqual(kw.get("friction", 0.0), 1.0,
                               msg="URDF <dynamics friction> must reach newton's joint friction")

    def test_prismatic_too(self):
        w = self._world()
        slot = w.add_joint_prismatic(0, 1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                     1000.0, 50.0, 0.0, 0.04, 100.0, 0.0)
        self.assertEqual(w.set_joint_passive_dynamics(slot, 2.5, 0.0), 0)
        kind, kw = self._build_last(w)
        self.assertEqual(kind, "prismatic")
        self.assertAlmostEqual(kw.get("damping", 0.0), 2.5)
        self.assertNotIn("friction", kw, "a zero friction is not passed (newton default stays)")

    def test_undeclared_joint_is_byte_identical(self):
        w = self._world()
        w.add_joint_revolute(*WHEEL)
        _kind, kw = self._build_last(w)
        self.assertNotIn("damping", kw)
        self.assertNotIn("friction", kw)

    def test_hatch_reverts(self):
        os.environ["OMNISIM_NEWTON_JOINT_DYNAMICS"] = "0"
        w = self._world()
        slot = w.add_joint_revolute(*WHEEL)
        # 1 = queued but not applied: the engine logs "NOT applied" on it, never "applied".
        self.assertEqual(w.set_joint_passive_dynamics(slot, 1.0, 1.0), 1)
        _kind, kw = self._build_last(w)
        self.assertNotIn("damping", kw, "OMNISIM_NEWTON_JOINT_DYNAMICS=0 must drop it")
        self.assertNotIn("friction", kw, "OMNISIM_NEWTON_JOINT_DYNAMICS=0 must drop it")

    def test_default_is_off(self):
        os.environ.pop("OMNISIM_NEWTON_JOINT_DYNAMICS", None)
        w = self._world()
        slot = w.add_joint_revolute(*WHEEL)
        self.assertEqual(w.set_joint_passive_dynamics(slot, 1.0, 1.0), 1)
        _kind, kw = self._build_last(w)
        self.assertNotIn("damping", kw, "joint dynamics must be OFF by default (2026-10-03)")
        self.assertNotIn("friction", kw, "joint dynamics must be OFF by default (2026-10-03)")

    def test_bad_slot_is_refused(self):
        w = self._world()
        w.add_joint_revolute(*WHEEL)
        self.assertEqual(w.set_joint_passive_dynamics(7, 1.0, 1.0), -1)
        self.assertNotIn("damping", w.pending_revolutes[0])

    def test_fixed_spec_is_refused(self):
        w = self._world()
        w.pending_revolutes.append(dict(kind="fixed", parent=0, child=1))
        self.assertEqual(w.set_joint_passive_dynamics(0, 1.0, 1.0), -1)


class EndToEndMujocoTest(unittest.TestCase):
    """The builder kwargs really land in the compiled MuJoCo model (CPU)."""

    def test_dof_damping_and_frictionloss(self):
        import newton
        import warp as wp

        b = newton.ModelBuilder()
        inertia = wp.mat33((1e-3, 0.0, 0.0), (0.0, 1e-3, 0.0), (0.0, 0.0, 1e-3))
        base = b.add_link(xform=wp.transform((0.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
                          mass=1.0, inertia=inertia)
        wheel = b.add_link(xform=wp.transform((0.0, 0.1, 0.5), (0.0, 0.0, 0.0, 1.0)),
                           mass=0.1, inertia=inertia)
        b.add_shape_sphere(base, radius=0.05)
        b.add_shape_sphere(wheel, radius=0.03)
        jf = b.add_joint_free(base)
        jr = b.add_joint_revolute(base, wheel, axis=(0.0, 1.0, 0.0),
                                  parent_xform=wp.transform((0.0, 0.1, 0.0), (0.0, 0.0, 0.0, 1.0)),
                                  child_xform=wp.transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
                                  damping=1.0, friction=0.75)
        b.add_articulation([jf, jr])
        model = b.finalize(device="cpu")
        solver = newton.solvers.SolverMuJoCo(model, use_mujoco_cpu=True)
        m = solver.mj_model
        # free joint = dofs 0..5, the hinge = dof 6
        self.assertEqual(m.nv, 7)
        self.assertAlmostEqual(float(m.dof_damping[6]), 1.0, places=5)
        self.assertAlmostEqual(float(m.dof_frictionloss[6]), 0.75, places=5)


if __name__ == "__main__":
    unittest.main()
