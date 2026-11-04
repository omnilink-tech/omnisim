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

"""A slider servo can deliver its declared force inside its own travel
(2026-10-22).

WHY THIS EXISTS
---------------
OmBasicJoint builds every position servo ``ke = effort*10`` -- on a hinge that
saturates at a 0.1 rad error, on a slider at a 100 mm error. Preferred Robotics'
Kachaka docking pin (0.2 kg, travel 0..12 mm) commanded to its upper stop sat at
exactly 0.0 m in OmniSim v9.2.7 even with its effort raised to 10 N, because the
servo could push at most 100 N/m * 12 mm = 1.2 N against a 1.96 N weight. Plain
MuJoCo 3.11 lifts the same file 11.0 mm (kp 2000). The runtime now floors a
slider's gain at ``effort / travel`` (World._slider_gain_floor), kd scaled alike.

This pins the runtime engine-free on the SOURCE runtime with the pin itself.

RUN:  python -m pytest tests/test_newton_slider_gain_floor.py -q
"""

import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.path.join(_ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")

_ENV = ("OMNISIM_NEWTON_SLIDER_GAIN_FLOOR", "OMNISIM_NEWTON_TARGET_KE", "OMNISIM_NEWTON_TARGET_KD",
        "OMNISIM_NEWTON_FINGER_KE", "OMNISIM_NEWTON_FINGER_KD", "OMNISIM_NEWTON_TORQUE_MODE")


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_slider_floor", _RUNTIME)
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


PIN_M, TRAVEL = 0.2, 0.012


def _pin(effort, travel=TRAVEL, seconds=2.0, dt=0.001):
    """The Kachaka docking pin on a fixed base, built with the engine's own gains
    (ke = 10*effort, kd = 0.5*effort) and commanded to its upper stop. Returns
    (world, slot, q after ``seconds``)."""
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0.0, 0.0, -9.81)
    w.set_solver_preference("mujoco")
    base = w.add_static_body(0.0, 0.0, 0.05)
    pin = w.add_body(PIN_M, 0.0, 0.0, 0.05, 0, 0, 0, 1, 3.5e-5, 3.5e-5, 6.25e-5)
    s = w.add_joint_prismatic(base, pin, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                              effort * 10.0, effort * 0.5, 0.0, travel, effort, 1.0)
    w.finalize()
    w.set_joint_target_pos(s, travel)
    for _ in range(int(seconds / dt)):
        w.step(dt)
    return w, s, w.get_joint_angle(s)


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class SliderGainFloor(unittest.TestCase):

    def test_a_ten_newton_pin_lifts(self):
        with _Env():
            w, _s, q = _pin(10.0)
        # ke = 10/0.012 = 833 N/m: droop = weight/ke = 2.35 mm -> 9.65 mm
        self.assertGreater(q, 0.009)
        self.assertLessEqual(q, TRAVEL + 1e-4)
        self.assertIn("slider_gain_floor", getattr(w, "_engine_notes", {}))

    def test_the_published_one_newton_pin_stays_down(self):
        # 1 N < the pin's 1.96 N weight: the file's own defect, as plain MuJoCo shows it
        with _Env():
            _w, _s, q = _pin(1.0)
        self.assertLess(abs(q), 1e-4)

    def test_hatch_restores_effort_times_ten(self):
        with _Env(OMNISIM_NEWTON_SLIDER_GAIN_FLOOR="0"):
            w, _s, q = _pin(10.0)
        self.assertLess(abs(q), 1e-4, "ke = 100 N/m can push 1.2 N, under the 1.96 N weight")
        self.assertNotIn("slider_gain_floor", getattr(w, "_engine_notes", {}) or {})

    def test_explicit_finger_gain_wins(self):
        with _Env(OMNISIM_NEWTON_FINGER_KE="2000", OMNISIM_NEWTON_FINGER_KD="50"):
            w, _s, q = _pin(10.0)
        self.assertAlmostEqual(q, TRAVEL - PIN_M * 9.81 / 2000.0, delta=2e-4)    # plain MuJoCo: 0.01102
        self.assertNotIn("slider_gain_floor", getattr(w, "_engine_notes", {}) or {})


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class FloorArithmetic(unittest.TestCase):

    def _floor(self, lo, hi, effort=50.0, ke=None, kd=None, **env):
        w = _rt.World()
        j = dict(kind="prismatic", limit_lower=lo, limit_upper=hi, effort_limit=effort)
        with _Env(**env):
            return w._slider_gain_floor(j, effort * 10.0 if ke is None else ke,
                                        effort * 0.5 if kd is None else kd)

    def test_short_travel_reaches_effort_over_travel_with_kd_scaled(self):
        ke, kd = self._floor(0.0, 0.0425)                   # Robotiq 2F-85 finger
        self.assertAlmostEqual(ke, 50.0 / 0.0425)
        self.assertAlmostEqual(ke / kd, 20.0)                # the servo time constant is kept

    def test_tenth_of_a_metre_or_more_is_untouched(self):
        self.assertEqual(self._floor(-0.030, 0.070, effort=125.0), (1250.0, 62.5))   # 2F-140
        self.assertEqual(self._floor(-0.30, 0.05, effort=200.0), (2000.0, 100.0))

    def test_no_limit_unbounded_or_no_effort_is_untouched(self):
        self.assertEqual(self._floor(0.0, 0.0), (500.0, 25.0))
        self.assertEqual(self._floor(-1e16, 1e16), (500.0, 25.0))
        self.assertEqual(self._floor(0.0, 0.01, effort=0.0, ke=20.0, kd=3.0), (20.0, 3.0))

    def test_a_passive_joint_is_untouched(self):
        self.assertEqual(self._floor(0.0, 0.01, ke=0.0, kd=0.0), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
