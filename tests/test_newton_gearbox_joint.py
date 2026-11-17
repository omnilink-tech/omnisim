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

"""The Newton runtime's GEARBOX JOINT (2026-11-14): a compliant reduced-order gearbox coupled
at the physics sub-step, declared per joint by a controller.

WHY THIS EXISTS
---------------
Newton convexifies mesh colliders, so gear teeth cannot mesh in the solver; the gear-contact
analysis (an offline gear model) reduces a gearbox to a ratio, a backlash deadband, a
torque-twist curve, efficiencies and a rotor inertia. A controller can apply that spring only
at its own step; World.gearbox_attach couples it every sub-step. It is OPT-IN: a joint
carries one only when a controller declares it through the channel file and sends the token
as its first torque.

This pins, engine-free on the SOURCE runtime:
- no declaration -> the first torque on a joint is an ordinary joint torque (unchanged path);
- a declared, input-locked gearbox under gravity sits at backlash/2 + T/k (analytic);
- perturbed about that point it rings at sqrt(k / J) (analytic);
- OMNISIM_NEWTON_GEARBOX=0 ignores the declaration;
- a token that matches no declaration binds nothing.

RUN:  python -m pytest tests/test_newton_gearbox_joint.py -q
"""

import importlib.util
import json
import math
import os
import sys
import tempfile
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_gearbox", _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                                    # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc

M, D, I_C = 5.0, 0.15, 0.5           # link mass, its centre's distance from the axis, inertia about it
J = I_C + M * D * D                   # about the joint
K = 1.0e4                             # N m / rad (a linear curve: exact analytics)
HALF = 5.0e-4                         # half the backlash (rad)
G = 9.81
SPEC = {"name": "pin", "ratio": 25.0, "backlash_rad": 2 * HALF, "twist_T_Nm": [0.0, 20.0],
        "twist_rad": [0.0, 20.0 / K], "efficiency_forward": 1.0, "efficiency_back": 1.0,
        "J_rotor_kgm2": 2.0e-5, "damping_ratio": 0.0, "input_locked": True}


class _Channel(object):
    """A private channel file for one test (OMNISIM_NEWTON_GEARBOX_CMD)."""

    def __init__(self, lines=(), hatch=None):
        self.lines, self.hatch = lines, hatch

    def __enter__(self):
        fd, self.path = tempfile.mkstemp(suffix=".jsonl", prefix="gearbox_pin_")
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            for ln in self.lines:
                fh.write(json.dumps(ln) + "\n")
        self.old = {k: os.environ.get(k) for k in ("OMNISIM_NEWTON_GEARBOX_CMD", "OMNISIM_NEWTON_GEARBOX")}
        os.environ["OMNISIM_NEWTON_GEARBOX_CMD"] = self.path
        os.environ.pop("OMNISIM_NEWTON_GEARBOX", None)
        if self.hatch is not None:
            os.environ["OMNISIM_NEWTON_GEARBOX"] = self.hatch
        return self

    def acks(self):
        try:
            with open(self.path + ".ack", encoding="utf-8") as fh:
                return [json.loads(ln) for ln in fh if ln.strip()]
        except OSError:
            return []

    def __exit__(self, *a):
        for k, v in self.old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
        for p in (self.path, self.path + ".ack"):
            try:
                os.remove(p)
            except OSError:
                pass


def _arm():
    """A horizontal-axis link: gravity puts M g D on the joint at q = 0."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -G)
    w.set_solver_preference("mujoco")
    base = w.add_static_body(0.0, 0.0, 0.5)
    link = w.add_body(M, D, 0.0, 0.5, 0, 0, 0, 1, I_C, I_C, I_C)
    s = w.add_joint_revolute(base, link, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -D, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 1000.0, 1000.0)
    w.finalize()
    return w, s


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class GearboxJoint(unittest.TestCase):

    def test_no_declaration_is_an_ordinary_torque(self):
        with _Channel():
            w, s = _arm()
            w.set_joint_force(s, 1.5)
            self.assertFalse(getattr(w, "_gearboxes", None))
            self.assertEqual(list(w.joint_forces.values()), [1.5])

    def test_unmatched_token_binds_nothing(self):
        with _Channel([{"op": "declare", "token": 1.0e-7, "spec": SPEC}]) as ch:
            w, s = _arm()
            w.set_joint_force(s, 2.0e-7)
            self.assertFalse(getattr(w, "_gearboxes", None))
            self.assertEqual(ch.acks(), [])

    def test_hatch_ignores_the_declaration(self):
        tok = 3.1e-7
        with _Channel([{"op": "declare", "token": tok, "spec": SPEC}], hatch="0"):
            w, s = _arm()
            w.set_joint_force(s, tok)
            self.assertFalse(getattr(w, "_gearboxes", None))

    def _bound(self, ch_lines, tok):
        w, s = _arm()
        w.set_joint_force(s, tok)
        self.assertIn(int(s), w._gearboxes)
        return w, s

    def test_static_deflection_is_backlash_half_plus_load_over_k(self):
        tok = 3.2e-7
        with _Channel([{"op": "declare", "token": tok, "spec": SPEC}]) as ch:
            w, s = self._bound(None, tok)
            self.assertTrue(ch.acks() and ch.acks()[0].get("attached"))
            dt = 0.001
            for _ in range(3000):
                w.step(dt)
            q = w.get_joint_angle(s)
        # gravity turns the link about +y by -M g D at q=0 (the centre is on +x): the joint
        # angle settles at -(HALF + M g D / K), the small-angle load
        expect = HALF + M * G * D / K
        self.assertAlmostEqual(abs(q), expect, delta=0.02 * expect)

    def test_unresolved_first_segment_is_not_infinitely_stiff(self):
        """A curve whose first segment has no twist (the analysis resolved none below 3 % of
        rated, as some cycloid reducer models do) once read as k ~ 1e15 N m/rad and made the
        rotor's inner sub-stepping take ~1e6 steps per physics step."""
        import time
        spec = dict(SPEC, twist_T_Nm=[0.0, 0.669, 5.575, 11.15, 22.3],
                    twist_rad=[0.0, 0.0, 1.19e-4, 2.41e-4, 4.14e-4])
        tok = 3.4e-7
        with _Channel([{"op": "declare", "token": tok, "spec": spec}]) as ch:
            w, s = _arm()
            w.set_joint_force(s, tok)
            ack = ch.acks()[0]
            self.assertTrue(ack.get("attached"), ack)
            self.assertLess(ack["k_max_Nm_per_rad"], 1.0e6)
            t0 = time.time()
            for _ in range(200):
                w.step(0.001)
            self.assertLess(time.time() - t0, 30.0)

    def test_rings_at_sqrt_k_over_j(self):
        tok = 3.3e-7
        with _Channel([{"op": "declare", "token": tok, "spec": SPEC}]):
            w, s = self._bound(None, tok)
            g = w._gearboxes[int(s)]
            # start engaged at the loaded equilibrium plus a small extra twist
            x_eq = M * G * D / K
            sign = -1.0
            g["th_m"] = 0.0          # input held at zero: the link sits at sign*(HALF + x)
            dt = 0.0005
            # perturb: let it settle, then measure about the mean
            qs = []
            for i in range(4000):
                w.step(dt)
                qs.append(w.get_joint_angle(s))
        import numpy as np
        y = np.asarray(qs[1000:]) - np.mean(qs[1000:])
        zc = np.nonzero(np.sign(y[:-1]) != np.sign(y[1:]))[0]
        f = (len(zc) - 1) / (2 * (zc[-1] - zc[0]) * dt)
        self.assertAlmostEqual(f, math.sqrt(K / J) / (2 * math.pi), delta=0.03 * math.sqrt(K / J) / (2 * math.pi))
        self.assertGreater(abs(np.mean(qs[1000:])), HALF)       # engaged, not in the deadband
        del x_eq, sign


if __name__ == "__main__":
    unittest.main()
