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

"""The post-step joint clamp must not inject momentum into a free-floating robot.

WHAT WENT WRONG (found 2026-10-14, DLR UR10e lane)
--------------------------------------------------
The runtime holds every scalar joint to its URDF velocity limit after each step
by overwriting the joint's qvel. On a fixed base the world absorbs the change.
On a FREE-floating robot nothing did: the free joint kept its velocity, so the
momentum the cap removed from the joint simply vanished, and the base took the
opposite. A free-falling UR10e whose over-stiff shoulder-pan servo saturated
spun at 10.7 rad/s about z within 25 ms and drifted 1.7 m sideways in 0.5 s;
plain MuJoCo just falls, and plain MuJoCo plus a per-step qvel clip reproduces
the spin.

THE FIX
-------
The clamp is an internal action (a motor's back-EMF, an end-stop), so the free
joint of the same tree now takes the reaction: dv_F = -M_FF^-1 M_FC dv_C, which
holds the tree's linear and angular momentum fixed while every other joint keeps
its velocity -- the old rule exactly, with the reaction going to the floating
base instead of nowhere. Fixed-base robots are untouched. Hatch, value-parsed:
OMNISIM_NEWTON_CLAMP_MOMENTUM=0 restores the old overwrite.

THE PROBE
---------
Engine-free, in-process, through the SOURCE runtime's World API (like
tests/test_newton_joint_limit_load.py): a free base (2 kg, COM offset, tilted
0.6 rad) carrying a 2-link chain (z then y axis, caps 1.0 and 0.5 rad/s), both
joints driven by constant torques (3 and -1 N.m) that saturate the caps, gravity
on, 400 steps at 1 ms. Total angular momentum about the COM must stay at its
initial 0; the COM's horizontal position must not move; the cap must still hold.

Measured on newton 1.5.0 / mujoco 3.11.0 (CPU mj_step), 2026-10-15:
    no clamp (OMNISIM_NEWTON_DISABLE_JOINT_CLAMP=1)  |L| 0.035  joint 1 reaches 50 rad/s
    old clamp (hatch =0)                             |L| 3.33   COM 0.51 m sideways
    fixed                                            |L| 4.8e-4 COM 1.2 mm sideways
    fixed base: final state bit-identical with and without the hatch

    python -m pytest tests/test_newton_clamp_momentum.py -v
"""

import importlib.util
import math
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RUNTIME = os.path.join(_HERE, os.pardir, "src", "omnisim", "physics",
                        "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_HERE, os.pardir, "msys64", "mingw64", "bin",
                       "newton-runtime", "site-packages")

CAPS = (1.0, 0.5)
#: Env this file sets per case; cleared around every case so a developer's shell
#: cannot change the scene.
_ENV_KEYS = ("OMNISIM_NEWTON_CLAMP_MOMENTUM", "OMNISIM_NEWTON_DISABLE_JOINT_CLAMP",
             "OMNISIM_NEWTON_TORQUE_MODE", "OMNISIM_NEWTON_JOINT_ARMATURE",
             "OMNISIM_NEWTON_PASSIVE_VEL_CLAMP", "OMNISIM_NEWTON_JOINT_CLAMP_SLACK")


def _load_runtime():
    """Import the SOURCE runtime by path, never the (possibly stale) bundle copy."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)          # newton / warp / mujoco live here
    spec = importlib.util.spec_from_file_location(
        "_omnisim_newton_runtime_clamp_momentum", os.path.normpath(_RUNTIME))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    import mujoco as _mj
    import numpy as _np
    _IMPORT_ERROR = None
except Exception as exc:                      # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc


def _momentum(m, d):
    """(linear momentum, angular momentum about the COM, COM) of the robot's tree."""
    s = _mj.MjData(m)
    s.qpos[:] = d.qpos
    s.qvel[:] = d.qvel
    _mj.mj_forward(m, s)
    _mj.mj_subtreeVel(m, s)
    r = 1                                     # the one tree's root body
    return (m.body_subtreemass[r] * s.subtree_linvel[r].copy(),
            s.subtree_angmom[r].copy(), s.subtree_com[r].copy())


def _run(floating=True, env=None, steps=400, dt=0.001):
    saved = {k: os.environ.pop(k) for k in _ENV_KEYS if k in os.environ}
    os.environ.update(env or {})
    try:
        w = _rt.World()
        w.set_solver_preference("mujoco")
        w.set_up_axis("Z")
        w.set_gravity(0.0, 0.0, -9.81)
        c, s = math.cos(0.3), math.sin(0.3)
        if floating:
            base = w.add_body(2.0, 0.0, 0.0, 1.0, 0.0, s, 0.0, c, ixx=0.03, iyy=0.02, izz=0.025,
                              cx=0.05, cy=-0.02, cz=0.01)
        else:
            base = w.add_static_body(0.0, 0.0, 1.0, 0.0, s, 0.0, c)
        l1 = w.add_body(1.0, 0.3, 0.0, 1.0, ixx=0.004, iyy=0.01, izz=0.01, cx=0.05, cy=0.0, cz=0.02)
        l2 = w.add_body(0.5, 0.6, 0.0, 1.0, ixx=0.002, iyy=0.004, izz=0.004)
        # ke = kd = 0: no engine PD, the joint_f below is the only drive.
        s1 = w.add_joint_revolute(base, l1, 0, 0, 1, 0.3, 0, 0, 0, 0, 0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, CAPS[0])
        s2 = w.add_joint_revolute(l1, l2, 0, 1, 0, 0.3, 0, 0, 0, 0, 0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, CAPS[1])
        w.finalize()
        if not getattr(w.solver, "use_mujoco_cpu", False):
            raise unittest.SkipTest("solver is not on the CPU mj_step path")
        m, d = w.solver.mj_model, w.solver.mj_data
        hinge = [int(m.jnt_dofadr[j]) for j in range(m.njnt)
                 if int(m.jnt_type[j]) == int(_mj.mjtJoint.mjJNT_HINGE)]
        qds = w.model.joint_qd_start.numpy()
        nqd = [int(qds[w.slot_to_real_idx[s1]]), int(qds[w.slot_to_real_idx[s2]])]
        _p0, l0, com0 = _momentum(m, d)
        peak, held = [0.0, 0.0], [0.0, 0.0]
        for i in range(steps):
            w.set_joint_force(s1, 3.0)
            w.set_joint_force(s2, -1.0)
            w.step(dt)
            if i >= steps // 2:
                qd = w.state_a.joint_qd.numpy()
                for k in (0, 1):
                    # what the solver produced (pre-clamp) and what the clamp left
                    peak[k] = max(peak[k], abs(float(d.qvel[hinge[k]])))
                    held[k] = max(held[k], abs(float(qd[nqd[k]])))
        _p, l, com = _momentum(m, d)
        return {"L0": l0, "L": l, "com0": com0, "com": com, "peak": peak, "held": held,
                "qpos": _np.array(d.qpos), "qvel": _np.array(d.qvel),
                "fixes": getattr(w, "_clamp_momentum_fixes", 0)}
    finally:
        for k in _ENV_KEYS:
            os.environ.pop(k, None)
        os.environ.update(saved)


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class ClampMomentumTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = {
            "fixed_float": _run(),
            "hatch_float": _run(env={"OMNISIM_NEWTON_CLAMP_MOMENTUM": "0"}),
            "noclamp_float": _run(env={"OMNISIM_NEWTON_DISABLE_JOINT_CLAMP": "1"}),
            "fixed_base": _run(floating=False),
            "hatch_base": _run(floating=False, env={"OMNISIM_NEWTON_CLAMP_MOMENTUM": "0"}),
        }

    @staticmethod
    def _drift(c):
        return float(_np.linalg.norm((c["com"] - c["com0"])[:2]))

    def test_probe_saturates_the_cap(self):
        """Control: without the clamp the torques drive joint 1 far past its cap."""
        c = self.cases["noclamp_float"]
        self.assertGreater(c["peak"][0], 10.0 * CAPS[0], "peak %.3f" % c["peak"][0])

    def test_floating_base_conserves_angular_momentum(self):
        """The bug: |L| 3.3 kg.m^2/s from rest before 2026-10-15."""
        c = self.cases["fixed_float"]
        self.assertGreater(c["fixes"], 0, "the momentum correction never ran")
        self.assertLess(float(_np.linalg.norm(c["L0"])), 1e-9)
        self.assertLess(float(_np.linalg.norm(c["L"])), 0.01,
                        "angular momentum about the COM grew to %s from rest: the velocity "
                        "cap is injecting momentum into the floating base" % c["L"].round(4))

    def test_floating_base_com_does_not_drift(self):
        """Gravity is the only external force: the COM may only fall."""
        c = self.cases["fixed_float"]
        self.assertLess(self._drift(c), 0.005, "COM moved %.4f m sideways" % self._drift(c))

    def test_floating_base_cap_still_holds(self):
        """The joint speed is still capped: the state the clamp leaves sits on the cap."""
        c = self.cases["fixed_float"]
        for k in (0, 1):
            self.assertLessEqual(c["held"][k], CAPS[k] * (1.0 + 1e-6), "joint %d %.4f" % (k + 1, c["held"][k]))
            self.assertGreater(c["held"][k], 0.99 * CAPS[k], "joint %d never reached its cap" % (k + 1))

    def test_hatch_restores_the_old_clamp(self):
        """OMNISIM_NEWTON_CLAMP_MOMENTUM=0 must reproduce the defect -- which also proves this
        probe can see it."""
        c = self.cases["hatch_float"]
        self.assertEqual(c["fixes"], 0)
        self.assertGreater(float(_np.linalg.norm(c["L"])), 1.0,
                           "with the hatch at 0 |L| is %.4f; the old clamp measured 3.3"
                           % float(_np.linalg.norm(c["L"])))
        self.assertGreater(self._drift(c), 0.1)

    def test_fixed_base_is_unchanged(self):
        """A robot welded to the world: bit-identical with and without the hatch, cap enforced."""
        a, b = self.cases["fixed_base"], self.cases["hatch_base"]
        self.assertEqual(a["fixes"], 0)
        self.assertTrue(_np.array_equal(a["qpos"], b["qpos"]))
        self.assertTrue(_np.array_equal(a["qvel"], b["qvel"]))
        for k in (0, 1):
            self.assertLessEqual(a["held"][k], CAPS[k] * (1.0 + 1e-6), "joint %d %.4f" % (k + 1, a["held"][k]))
            self.assertGreater(a["held"][k], 0.99 * CAPS[k], "joint %d never reached its cap" % (k + 1))


if __name__ == "__main__":
    unittest.main()
