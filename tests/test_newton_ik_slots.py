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

"""Batched IK must answer in OmniSim joint slots, and its residual must be TRUE.

WHAT THIS PINS, AND WHY IT IS NOT OBVIOUS
-----------------------------------------
`World.solve_ik` runs newton's IKSolver against the live model. Two things about
that are easy to get wrong in ways that LOOK like success:

1. THE MASK. newton's IKSolver optimises every joint coordinate in the model
   unless it is given a `joint_dof_mask`. A world is not one arm -- it carries
   free roots, other robots, loose props. Unmasked, the optimiser will happily
   reach a target by moving coordinates the caller cannot command, and then
   report a superb residual for a pose that is unreachable in practice.
   MEASURED on a floating-base OMNIARM6: 8 targets "solved" to 4.3e-07 m by
   TRANSLATING THE BASE 0.923 m. Writing back only the slots a controller can
   drive gave 0.536 m of real error. Masked, the same problems honestly report
   0.321 m, because those targets were out of reach.

   ⚠ The obvious assertion -- "the reported residual equals what the caller
   actually achieves" -- does NOT catch this, and a first version of this test
   made exactly that mistake. solve_ik measures its residual by forward
   kinematics on the write-back vector itself, so it is honest BY CONSTRUCTION
   whether or not the optimiser cheated. What a missing mask costs is not
   honesty, it is ANSWER QUALITY: the effort spent sliding the base is effort
   not spent on the joints the caller keeps. So the assertion that goes red is
   "a target the arm can reach is solved to sub-millimetre". MEASURED on the
   fixture below, same four targets: masked 0.000001 / 0.100000 / 0.198419 /
   0.039415 m, unmasked 0.120342 / 0.259311 / 0.247535 / 0.239436 m.

2. THE SLOT MAP. A slot id is not a joint_q offset. slot_to_real_idx maps an
   OmniSim slot to a builder joint, and joint_q_start maps that to a coordinate
   offset; a free root (7 coords / 6 dofs) or a ball (4/3) shifts every later
   joint and makes joint_coord_count != joint_dof_count. Indexing a DOF-shaped
   array (Model.joint_limit_lower/upper IS dof-shaped) with a coordinate index
   therefore reads the WRONG joint's limit -- or raises IndexError on the last
   slot, which is how this was found.

Both failure modes are silent and both produce plausible numbers, so the tests
below are written to go red on the plausible-looking version, not just on a
crash -- and each was CHECKED against a deliberately broken runtime rather than
assumed to be capable of failing.
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

L = 0.30  # link length


def _load_runtime():
    """Import the runtime by PATH, never by name.

    The engine loads this module out of the BUNDLE, so a plain
    `import omnisim_newton_runtime` can silently test a stale staged copy
    instead of the source -- the trap that once let a 23-minute-old bundle pass
    a full suite while the fix under test was never loaded.
    """
    if os.path.isdir(_BUNDLE) and _BUNDLE not in sys.path:
        sys.path.insert(0, _BUNDLE)          # newton / warp / mujoco live here
    spec = importlib.util.spec_from_file_location(
        "_omnisim_newton_runtime_ik_under_test", os.path.normpath(_RUNTIME))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


try:
    _rt = _load_runtime()
    _IMPORT_ERROR = None
except Exception as exc:                      # noqa: BLE001
    _rt, _IMPORT_ERROR = None, exc


@unittest.skipIf(_rt is None, "newton runtime unavailable: %s" % (_IMPORT_ERROR,))
class NewtonIkSlots(unittest.TestCase):
    def _arm(self, n_links=4, floating_base=True):
        """A serial revolute arm on a FLOATING base.

        ⚠ THE FLOATING BASE IS WHAT MAKES THIS FIXTURE ABLE TO GO RED, and the
        first version of this test got it wrong. An arm bolted to a STATIC base
        with an unrelated free body parked nearby does NOT detect a missing
        mask: those spare coordinates are outside the end effector's kinematic
        chain, so their Jacobian columns are zero and the optimiser gains
        nothing by moving them -- the test passed with the mask deleted, which
        made it worthless. The extra DOFs have to be IN THE CHAIN. A free root
        puts 7 coordinates and 6 dofs directly beneath the arm, owned by no
        slot, and reaching a target by sliding the whole robot is then both
        available and cheap -- which is exactly the cheat that was measured on a
        floating-base OMNIARM6 (4.3e-07 m "solved" by translating the base 0.923 m).
        It also makes joint_coord_count != joint_dof_count, which is the second
        thing under test.
        """
        w = _rt.World()
        w.set_up_axis("Z")
        w.set_gravity(0.0, 0.0, -9.81)
        if floating_base:
            base = w.add_body(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                              0.02, 0.02, 0.02)
            w.add_shape_box(base, 0.08, 0.08, 0.04)
        else:
            base = w.add_static_body(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        prev, slots = base, []
        for i in range(n_links):
            link = w.add_body(1.0, 0.0, 0.0, L * (i + 1), 0.0, 0.0, 0.0, 1.0,
                              0.01, 0.01, 0.01)
            w.add_shape_capsule(link, 0.03, L * 0.5)
            slots.append(w.add_joint_revolute(
                prev, link, 0.0, 1.0, 0.0,
                0.0, 0.0, (L if i else 0.0), 0.0, 0.0, 0.0,
                200.0, 20.0, -2.0, 2.0, 500.0, 10.0))
            prev = link
        w.finalize()
        return w, prev, slots

    # ---------------------------------------------------------------- slot map

    def test_slot_ids_are_not_coordinate_offsets(self):
        """The map must come from joint_q_start, and survive coord != dof."""
        w, _ee, slots = self._arm()
        m = w._ik_slot_map()
        self.assertTrue(set(slots).issubset(m.keys()),
                        "every revolute slot must appear in the IK slot map")
        # The free body contributes 7 coords / 6 dofs, so the two counts differ.
        # A coordinate index into a DOF-shaped array is what this guards.
        self.assertNotEqual(int(w.model.joint_coord_count),
                            int(w.model.joint_dof_count),
                            "the fixture must produce coord_count != dof_count, "
                            "or it is not exercising the bug this pins")
        qs = w.model.joint_q_start.numpy()
        for s in slots:
            q0, qw, d0, dw, lo, hi = m[s]
            real = int(w.slot_to_real_idx[s])
            self.assertEqual(q0, int(qs[real]),
                             "slot %d's q offset must come from joint_q_start" % s)
            self.assertEqual(qw, 1, "a revolute owns exactly one coordinate")
            self.assertLess(lo, hi, "authored limits must be ordered")

    # ---------------------------------------------------------------- the mask

    def test_reachable_target_is_actually_solved(self):
        """THE mask test. Red-checked against a runtime with the mask deleted.

        A 4-DOF arm asked for a pose well inside its workspace must get there.
        With joint_dof_mask the solve lands at 1e-06 m; without it the optimiser
        spends its budget sliding the floating base -- which the caller cannot
        keep -- and the same target comes back 0.120342 m out. Both numbers are
        measured on this fixture.

        This also write-backs the answer and confirms the reported residual is
        what a controller actually achieves. That part is honest by construction
        (solve_ik FKs the write-back vector), so it pins the residual
        MEASUREMENT, not the mask -- it is asserted here because it is cheap and
        because a future refactor could easily break it.
        """
        import newton
        import numpy as np
        import warp as wp

        w, ee, slots = self._arm()
        for _ in range(2):
            w.step(1.0 / 240.0)
        p0 = w.body_xform(ee)[:3]
        target = [p0[0] + 0.12, p0[1], p0[2] - 0.10]

        out = w.solve_ik(ee, target, slots=slots, iterations=120)
        self.assertEqual(len(out), len(slots) + 1)
        angles, reported = out[:len(slots)], out[len(slots)]

        # Write back ONLY the mapped slots -- exactly what a controller can do.
        m = w._ik_slot_map()
        q = w.model.joint_q.numpy().copy()
        for s, a in zip(slots, angles):
            q[m[s][0]] = a
        st = w.model.state()
        qd = wp.zeros(int(w.model.joint_dof_count), dtype=wp.float32,
                      device=w.model.device)
        newton.eval_fk(w.model,
                       wp.array(q, dtype=wp.float32, device=w.model.device),
                       qd, st)
        achieved = float(np.linalg.norm(
            st.body_q.numpy()[int(ee)][:3] - np.asarray(target, dtype=np.float64)))

        self.assertAlmostEqual(
            reported, achieved, delta=1e-3,
            msg=("solve_ik reported %.6f m but writing its answer back achieves "
                 "%.6f m -- the residual must be measured on the vector the "
                 "caller writes." % (reported, achieved)))
        # The mask assertion. Unmasked this target comes back ~0.12 m out.
        self.assertLess(
            achieved, 1e-3,
            "a target 0.16 m from the end effector, well inside a 4-link arm's "
            "reach, was solved only to %.6f m. The signature of a missing "
            "joint_dof_mask: the optimiser reached the target by moving "
            "coordinates the caller cannot command (the floating base), so the "
            "angles it kept are poor." % achieved)

    def test_solve_does_not_move_coordinates_it_was_not_given(self):
        """An unrelated free body must be untouched by a query."""
        w, ee, slots = self._arm()
        for _ in range(2):
            w.step(1.0 / 240.0)
        before = list(w.model.joint_q.numpy())
        w.solve_ik(ee, [0.1, 0.0, 0.8], slots=slots, iterations=80)
        after = list(w.model.joint_q.numpy())
        self.assertEqual(before, after,
                         "solve_ik is a QUERY: it must not write joint_q at all")

    # ---------------------------------------------------------------- honesty

    def test_unreachable_target_reports_its_error_rather_than_hiding_it(self):
        """A caller must be able to REJECT a problem instead of driving to it."""
        w, ee, slots = self._arm()
        for _ in range(2):
            w.step(1.0 / 240.0)
        out = w.solve_ik(ee, [8.0, 8.0, 8.0], slots=slots, iterations=120)
        residual = out[len(slots)]
        self.assertGreater(residual, 1.0,
                           "a target metres outside the workspace must report a "
                           "large residual, not a small one")
        self.assertTrue(math.isfinite(residual), "residual must stay finite")

    def test_answer_respects_authored_joint_limits(self):
        """IKObjectiveJointLimit is a soft residual, not a constraint.

        Measured 4.86 rad of violation with the objective active, so the output
        is clamped. This pins the clamp, not the objective.
        """
        w, ee, slots = self._arm()
        for _ in range(2):
            w.step(1.0 / 240.0)
        m = w._ik_slot_map()
        out = w.solve_ik(ee, [8.0, 8.0, 8.0], slots=slots, iterations=120)
        for s, a in zip(slots, out[:len(slots)]):
            lo, hi = m[s][4], m[s][5]
            self.assertGreaterEqual(a, lo - 1e-9,
                                    "slot %d returned %.4f below its limit %.4f"
                                    % (s, a, lo))
            self.assertLessEqual(a, hi + 1e-9,
                                 "slot %d returned %.4f above its limit %.4f"
                                 % (s, a, hi))

    def test_batching_returns_one_answer_per_target(self):
        """The batched shape is the whole reason this API exists."""
        w, ee, slots = self._arm()
        for _ in range(2):
            w.step(1.0 / 240.0)
        p0 = w.body_xform(ee)[:3]
        n = 4
        targets = []
        for k in range(n):
            targets += [p0[0] + 0.05 * (k + 1), p0[1], p0[2] - 0.04 * (k + 1)]
        out = w.solve_ik(ee, targets, slots=slots, iterations=80)
        self.assertEqual(len(out), n * len(slots) + n,
                         "expected n*len(slots) angles then n residuals")
        for r in out[n * len(slots):]:
            self.assertTrue(math.isfinite(r))


if __name__ == "__main__":
    unittest.main()
