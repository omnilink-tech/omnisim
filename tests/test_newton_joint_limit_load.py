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

"""A joint resting on its stop must still pass its distal load to the joints above it.

WHAT WENT WRONG (found 2026-10-03)
----------------------------------
OpenArm v2 right arm held straight out, elbow pressed against its lower stop:
a stiff servo held the shoulder with 3.84 N.m against a true static requirement
of 11.68 N.m. With the elbow 0.15 rad off the stop the same rig read 11.655 vs
11.646. The MuJoCo model newton built was correct -- stepped in plain MuJoCo it
gave the right torque -- so the fault was in what the runtime did BETWEEN steps.

It was the post-step hard joint-limit clamp in `World.step()`. MuJoCo enforces a
finite range itself, as a soft constraint, so a joint loaded into its stop
settles a hair past it. The clamp snapped q back to EXACTLY the limit and zeroed
the inward velocity. MuJoCo only instantiates a limit row when dist < margin
(= 0), so at dist == 0 the row is inactive: on every step the distal links fell
freely about the stop and the clamp discarded the motion afterwards. Their
weight never reached the shoulder. It is a DYNAMICS bug, not a readback bug:
the servo here computes its torque from its own PD, and the arm genuinely
needed less of it.

THE FIX
-------
Within a slack (default 0.05 rad / 0.005 m) the state is left to MuJoCo's own
limit row and only the readback is clamped into range, so position sensors
still report the stop exactly as before. Past the slack the state is clamped
onto the slack boundary (the limit row stays active). Hatches, value-parsed:
OMNISIM_NEWTON_JOINT_CLAMP_SLACK (rad) and OMNISIM_NEWTON_JOINT_CLAMP_SLACK_LINEAR
(m); `=0` restores the old exact clamp.

THE PROBE
---------
Engine-free: a 2-link pendulum (m1 = 2 kg COM 0.25 m, m2 = 1 kg COM 0.75 m from
the shoulder) on a static base, built through the SOURCE runtime's own World API
and stepped at 1 ms. The shoulder is held horizontal by a stiff PD sent through
set_joint_force (no engine PD -- torque mode by construction), the elbow rests on
its stop under gravity with zero commanded torque. True static shoulder torque:
9.81 * (2*0.25 + 1*0.75) = 12.2625 N.m.

Measured on newton 1.5.0 / mujoco 3.11.0 (CPU mj_step), this file's settings
(1500 steps, mean of the last 300):
    before the fix (= hatch 0)  elbow on stop  6.496 N.m   0.15 off the stop 12.239 N.m
    after the fix               elbow on stop 12.262 N.m   (upper stop 12.262)
    LIMIT_KE 10 / KD 1          state held at -0.0500 rad (the slack), shoulder 11.634

    python -m pytest tests/test_newton_joint_limit_load.py -v

It runs IN-PROCESS (like tests/test_newton_ik_slots.py) so it stays in the
default `make tests-unit` lane, and skips cleanly where newton/warp/mujoco are
absent (the hosted engine-free lane).
"""

import importlib.util
import os
import sys
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
_RUNTIME = os.path.join(_HERE, os.pardir, "src", "omnisim", "physics",
                        "omnisim_newton_runtime.py")
_BUNDLE = os.path.join(_HERE, os.pardir, "msys64", "mingw64", "bin",
                       "newton-runtime", "site-packages")

TRUTH = 9.81 * (2.0 * 0.25 + 1.0 * 0.75)   # 12.2625 N.m
#: Static-hold accuracy. Off the stop the probe reads 12.239 (-0.19%); the bug
#: read 6.496 (-47%).
REL_TOL = 0.02
#: Env this file sets per case; cleared around every case so a developer's shell
#: cannot change the scene.
_ENV_KEYS = ("OMNISIM_NEWTON_JOINT_CLAMP_SLACK", "OMNISIM_NEWTON_JOINT_CLAMP_SLACK_LINEAR",
             "OMNISIM_NEWTON_LIMIT_KE", "OMNISIM_NEWTON_LIMIT_KD",
             "OMNISIM_NEWTON_DISABLE_JOINT_CLAMP", "OMNISIM_NEWTON_TORQUE_MODE",
             "OMNISIM_NEWTON_JOINT_ARMATURE")


def _load_runtime():
    """Import the SOURCE runtime by path, never the (possibly stale) bundle copy."""
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)          # newton / warp / mujoco live here
    spec = importlib.util.spec_from_file_location(
        "_omnisim_newton_runtime_limit_load", os.path.normpath(_RUNTIME))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                      # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc


def _hold(upper=False, elbow_target=0.0, env=None, steps=1500, tail=300):
    """Hold the shoulder horizontal; return the measured static numbers.

    Axis -y: +q lifts the distal link, so gravity drives it to -q -- onto the
    LOWER stop at 0. Axis +y with range [-2.4, 0] is the mirror: the UPPER stop.
    """
    saved = {k: os.environ.pop(k) for k in _ENV_KEYS if k in os.environ}
    os.environ.update(env or {})
    try:
        ay = 1.0 if upper else -1.0
        lo, hi = (-2.4, 0.0) if upper else (0.0, 2.4)
        tgt = -elbow_target if upper else elbow_target
        w = _rt.World()
        w.set_solver_preference("mujoco")
        w.set_up_axis("Z")
        w.set_gravity(0.0, 0.0, -9.81)
        base = w.add_static_body(0.0, 0.0, 2.0)
        l1 = w.add_body(2.0, 0.25, 0.0, 2.0, ixx=0.01, iyy=0.05, izz=0.05)
        l2 = w.add_body(1.0, 0.75, 0.0, 2.0, ixx=0.01, iyy=0.03, izz=0.03)
        # ke = kd = 0: no engine PD, the joint_f below is the only drive.
        s1 = w.add_joint_revolute(base, l1, 0.0, ay, 0.0, 0.0, 0.0, 0.0, -0.25, 0.0, 0.0,
                                  0.0, 0.0, -3.0, 3.0, 0.0, 10.0)
        s2 = w.add_joint_revolute(l1, l2, 0.0, ay, 0.0, 0.25, 0.0, 0.0, -0.25, 0.0, 0.0,
                                  0.0, 0.0, lo, hi, 0.0, 10.0)
        w.finalize()
        if not getattr(w.solver, "use_mujoco_cpu", False):
            raise unittest.SkipTest("solver is not on the CPU mj_step path")
        d = w.solver.mj_data
        dt, kp, kd = 0.001, 1000.0, 20.0
        prev = [w.get_joint_angle(s1), w.get_joint_angle(s2)]
        tau, rb, st = [], [], []
        for i in range(steps):
            q = [w.get_joint_angle(s1), w.get_joint_angle(s2)]
            dq = [(a - b) / dt for a, b in zip(q, prev)]
            prev = q
            t1 = kp * (0.0 - q[0]) - kd * dq[0]
            w.set_joint_force(s1, t1)
            w.set_joint_force(s2, kp * (tgt - q[1]) - kd * dq[1])
            w.step(dt)
            if i >= steps - tail:
                tau.append(t1)
                rb.append(w.get_joint_angle(s2))
                st.append(float(d.qpos[1]))       # MuJoCo's own coordinate
        return {"tau": abs(sum(tau) / len(tau)), "rb_min": min(rb), "rb_max": max(rb),
                "state": sum(st) / len(st), "stop": hi if upper else lo, "upper": upper}
    finally:
        for k in _ENV_KEYS:
            os.environ.pop(k, None)
        os.environ.update(saved)


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class JointLimitLoadTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = {
            "off_stop": _hold(elbow_target=0.15),
            "lower": _hold(),
            "upper": _hold(upper=True),
            "hatch_zero": _hold(env={"OMNISIM_NEWTON_JOINT_CLAMP_SLACK": "0"}),
            # A limit spring so soft the static overshoot (~0.2 rad) far exceeds
            # the slack: the safety net must still bound the STATE.
            "gross": _hold(env={"OMNISIM_NEWTON_LIMIT_KE": "10", "OMNISIM_NEWTON_LIMIT_KD": "1"}),
        }

    def _rel(self, c):
        return abs(c["tau"] - TRUTH) / TRUTH

    def test_control_off_stop_holds_the_true_load(self):
        """Control: with nothing on a stop the probe measures the static truth."""
        c = self.cases["off_stop"]
        self.assertLess(self._rel(c), REL_TOL, "off-stop shoulder %.3f N.m vs truth %.3f"
                        % (c["tau"], TRUTH))

    def test_stop_resting_joint_passes_its_load_up(self):
        """The bug: 6.50 N.m (-47%) here before 2026-10-03 -- the forearm weighed nothing."""
        for name in ("lower", "upper"):
            c = self.cases[name]
            self.assertLess(self._rel(c), REL_TOL, (
                "%s stop: shoulder holds %.3f N.m, truth %.3f -- the joint resting on its "
                "stop is hiding the distal load (the post-step clamp is snapping the state "
                "onto the limit, which deactivates MuJoCo's limit row)"
                % (name, c["tau"], TRUTH)))

    def test_readback_still_reports_the_stop(self):
        """Sensors keep the old contract: inside the slack a joint reads EXACTLY its stop,
        while the STATE sits a hair past it under MuJoCo's soft limit."""
        for name in ("lower", "upper"):
            c = self.cases[name]
            self.assertAlmostEqual(c["rb_min"], c["stop"], places=9, msg=name)
            self.assertAlmostEqual(c["rb_max"], c["stop"], places=9, msg=name)
            past = (c["state"] - c["stop"]) if c["upper"] else (c["stop"] - c["state"])
            self.assertTrue(0.0 < past < 0.05,
                            "%s: state overshoot %.3e should be small and positive" % (name, past))

    def test_hatch_zero_restores_the_old_exact_clamp(self):
        """OMNISIM_NEWTON_JOINT_CLAMP_SLACK=0 must reproduce the old (load-hiding)
        dynamics -- which also proves this probe can see the defect at all."""
        c = self.cases["hatch_zero"]
        self.assertLess(c["tau"], 0.7 * TRUTH, (
            "with the hatch at 0 the shoulder held %.3f N.m; the old clamp measured 6.50 -- "
            "either the hatch no longer reverts, or the probe stopped seeing the defect"
            % c["tau"]))

    def test_gross_overshoot_is_still_bounded(self):
        """Past the slack the STATE is pulled back to the slack boundary, and the
        readback still reports the stop."""
        c = self.cases["gross"]
        self.assertGreater(c["state"], c["stop"] - 0.05 - 0.02,
                           "state %.4f ran far past the 0.05 rad slack" % c["state"])
        self.assertAlmostEqual(c["rb_min"], c["stop"], places=9)


if __name__ == "__main__":
    unittest.main()

