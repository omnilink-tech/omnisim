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

"""A supervisor force at an offset acts where it was applied, whatever the body's
centre of mass (2026-11-01).

WHY THIS EXISTS
---------------
OmSolid::applyExternalForceNewton sends (F, (P - origin) x F): a torque about the
body ORIGIN. newton's State.body_f is a wrench about the body's CENTRE OF MASS. They
agree only when the two coincide, so on any Solid whose COM is off its origin (any
Solid that declares an inertiaMatrix and a centerOfMass) every addForceWithOffset
and every propeller thrust acted as if moved by the origin-to-COM offset. Found by a
clothes rack (COM 1.02 m above its origin): pushed at its bar it tipped at 23.5 N
against the statics answer 37.8 N; re-referenced in the runtime, 39.2 N.

This pins the runtime engine-free on the SOURCE runtime.

RUN:  python -m pytest tests/test_newton_ext_wrench_at_com.py -q
"""

import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")
_HATCH = "OMNISIM_NEWTON_EXT_WRENCH_AT_COM"


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_ext_wrench", _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                                    # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc

M, COM_Z, F = 2.0, 1.0, 4.0


def _push(at_z, hatch=None, seconds=0.2, dt=0.002):
    """A free 2 kg body, no gravity, its COM 1 m above its origin, pushed along +x at
    height `at_z` above the origin the way the engine sends it. Returns body_vel."""
    old = os.environ.get(_HATCH)
    if hatch is None:
        os.environ.pop(_HATCH, None)
    else:
        os.environ[_HATCH] = hatch
    try:
        w = _rt.World()
        w.set_up_axis("Z")
        w.set_gravity(0.0, 0.0, 0.0)
        w.set_solver_preference("mujoco")
        b = w.add_body(M, 0.0, 0.0, 0.0, 0, 0, 0, 1, 0.5, 0.5, 0.5, cx=0.0, cy=0.0, cz=COM_Z)
        w.finalize()
        for _ in range(int(seconds / dt)):
            # OmSolid::applyExternalForceNewton: torque = (P - origin) x F, P = (0, 0, at_z)
            w.add_body_force(b, F, 0.0, 0.0, 0.0, at_z * F, 0.0)
            w.step(dt)
        return w.body_vel(b)
    finally:
        if old is None:
            os.environ.pop(_HATCH, None)
        else:
            os.environ[_HATCH] = old


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class ExternalWrenchAtCom(unittest.TestCase):

    def test_a_push_through_the_com_does_not_spin_it(self):
        v = _push(COM_Z)
        self.assertLess(abs(v[4]), 1e-3, "pushed through its centre of mass, the body must not turn")

    def test_a_push_at_the_origin_turns_it_by_the_right_moment(self):
        # 1 m below the COM: moment -F * 1 m about y, I_yy 0.5 -> wy = -8 rad/s^2 * 0.2 s = -1.6
        v = _push(0.0)
        self.assertAlmostEqual(v[4], -F * COM_Z / 0.5 * 0.2, delta=0.05)

    def test_hatch_restores_the_origin_torque(self):
        # Pass-through: the origin torque is read about the COM, so a push THROUGH the COM spins it.
        v = _push(COM_Z, hatch="0")
        self.assertAlmostEqual(v[4], F * COM_Z / 0.5 * 0.2, delta=0.05)


if __name__ == "__main__":
    unittest.main()
