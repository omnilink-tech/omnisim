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

"""The batched tick's conversions run through recorded launches (2026-11-15).

WHY THIS EXISTS
---------------
A batched CPU tick brackets its mj_step calls with newton's _apply_mjc_control
and _update_newton_state. On a small machine scene they cost 0.69 + 0.51 ms
around 0.11 ms of mj_step, nearly all of it per-call array allocation and
wp.launch argument packing. World._fastconv_ctrl / _fastconv_out run the SAME
kernels from persistent buffers through launches recorded once
(OMNISIM_NEWTON_FAST_CONVERT, default on; =0 is newton's own functions).

This pins, engine-free on the SOURCE runtime:

  * a gantry (three servo slides and a velocity spindle) and the same gantry
    with a free box run BIT-IDENTICAL trajectories with the path on and off,
    and the path is actually taken;
  * the hatch =0 never takes it;
  * a recorded launch that goes wrong is caught by the verify calls, the path
    turns itself off, and the trajectory is still newton's.

RUN:  python -m pytest tests/test_newton_fast_convert.py -q
"""

import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, os.pardir))
_RUNTIME = os.environ.get("OMNISIM_TEST_RUNTIME_PATH") or os.path.join(
    _ROOT, "src", "omnisim", "physics", "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_ROOT, "msys64", "mingw64", "bin", "newton-runtime", "site-packages")
_ENV = ("OMNISIM_NEWTON_FAST_CONVERT", "OMNISIM_NEWTON_FAST_CONVERT_VERIFY")


def _load_runtime():
    """Import the runtime by PATH (never a possibly stale bundle copy)."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)
    spec = importlib.util.spec_from_file_location("_omnisim_newton_runtime_fastconv", _RUNTIME)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    import numpy as np
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


def _gantry(free_box=False, sabotage=False, ticks=240):
    w = _rt.World()
    w.set_up_axis("Z")
    w.set_gravity(0, 0, -9.81)
    w.set_solver_preference("mujoco")
    w.set_substeps(8)
    base = w.add_static_body(0, 0, 0.0)
    w.add_shape_box(base, 0.5, 0.5, 0.02)
    x = w.add_body(8.0, 0, 0, 0.3, 0, 0, 0, 1, 0.1, 0.1, 0.1)
    y = w.add_body(4.0, 0, 0, 0.3, 0, 0, 0, 1, 0.05, 0.05, 0.05)
    z = w.add_body(2.0, 0, 0, 0.3, 0, 0, 0, 1, 0.02, 0.02, 0.02)
    sp = w.add_body(0.5, 0, 0, 0.2, 0, 0, 0, 1, 1e-3, 1e-3, 1e-3)
    sx = w.add_joint_prismatic(base, x, 1, 0, 0, 0, 0, 0.3, 0, 0, 0, 2e5, 2e3, -0.4, 0.4, 2e4, 1.0)
    sy = w.add_joint_prismatic(x, y, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2e5, 2e3, -0.4, 0.4, 2e4, 1.0)
    sz = w.add_joint_prismatic(y, z, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2e5, 2e3, -0.1, 0.1, 2e4, 1.0)
    ss = w.add_joint_revolute(z, sp, 0, 0, 1, 0, 0, -0.1, 0, 0, 0, 0.0, 5.0, 0, 0, 10.0, 2000.0)
    if free_box:
        b = w.add_body(0.3, 0.2, 0.2, 0.1, 0, 0, 0, 1, 1e-3, 1e-3, 1e-3)
        w.add_shape_box(b, 0.03, 0.03, 0.03)
    w.finalize()
    if sabotage:
        build = w._fastconv_ctrl_entry

        def broken(*a, **kw):
            e = build(*a, **kw)
            if e is not None:
                e["launches"] = []             # control never reaches MuJoCo
            return e
        w._fastconv_ctrl_entry = broken
    traj = []
    for k in range(ticks):
        w.set_joint_target_pos(sx, 0.2 * np.sin(k * 0.01))
        w.set_joint_target_pos(sy, 0.2 * np.cos(k * 0.013))
        w.set_joint_target_pos(sz, -0.02 * (k % 300) / 300)
        w.set_joint_target_vel(ss, 100.0 if k > 50 else 0.0)
        w.step(0.016)
        traj.append((w.state_a.body_q.numpy().copy(), w.state_a.joint_q.numpy().copy(),
                     w.state_a.joint_qd.numpy().copy()))
    return w, traj


def _same(a, b):
    return all(np.array_equal(x, y) for p, q in zip(a, b) for x, y in zip(p, q)) and len(a) == len(b)


@unittest.skipIf(_rt is None, "newton runtime not importable: %r" % (_IMPORT_ERROR,))
class FastConvert(unittest.TestCase):

    def _ab(self, **kw):
        with _Env(OMNISIM_NEWTON_FAST_CONVERT="0"):
            w0, stock = _gantry(**kw)
        with _Env():
            w1, fast = _gantry(**kw)
        return w0, stock, w1, fast

    def test_gantry_bit_identical_and_taken(self):
        w0, stock, w1, fast = self._ab()
        self.assertGreater(getattr(w1, "_n_batched", 0), 200)       # the batched tick ran
        self.assertGreater(w1._fastconv_hits, 400)                  # both halves, every tick
        self.assertTrue(w1._fastconv_flag)
        self.assertEqual(getattr(w0, "_fastconv_hits", 0), 0)       # hatch: never taken
        self.assertTrue(_same(stock, fast))
        self.assertGreater(abs(float(fast[-1][2][3])), 10.0)        # the spindle really spun up

    def test_free_body_bit_identical(self):
        _, stock, w1, fast = self._ab(free_box=True)
        self.assertGreater(w1._fastconv_hits, 400)
        self.assertTrue(_same(stock, fast))

    def test_bad_launch_is_caught_and_falls_back(self):
        with _Env(OMNISIM_NEWTON_FAST_CONVERT="0"):
            _, stock = _gantry()
        with _Env(OMNISIM_NEWTON_FAST_CONVERT_VERIFY="1000"):
            w, traj = _gantry(sabotage=True)
        self.assertFalse(w._fastconv_flag)                          # turned itself off
        self.assertTrue(_same(stock, traj))                         # and newton's result stood


if __name__ == "__main__":
    unittest.main()
