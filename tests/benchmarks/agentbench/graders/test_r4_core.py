#!/usr/bin/env python3
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

"""R4's core, exercised with **no simulator, no WSL and no network**.

The sibling of ``adapters/webots/test_r4_discriminates_webots.py``: that file
proves the task can be passed and failed on a real arm and costs a minute of
Webots per program; this one pins the pure geometry the verdict rests on and
costs milliseconds, so a threshold cannot be quietly loosened without a red
test somewhere cheap.

Two of these tests exist because the bug they describe was SHIPPED and
measured, not imagined:

* ``test_a_perfect_fixture_match_is_accepted`` -- ``find_fixture`` was written
  as ``(aabb_error(...) or 1e9) <= tol``, and ``aabb_error`` returns ``0.0``
  for a body authored at the spec to the digit. Zero is falsy, so an EXACT
  match scored ``1e9`` and was rejected. On the oracle's first run the table
  and the pad both reported ``fixture_candidates: 0``, and the damage
  cascaded: the table fell into R4.5's collision set (the run was failed for
  "striking" the thing it was reaching into) and "airborne" lost its table
  reference, so the payload counted as carried from t = 0 while it sat
  untouched on the table.
* ``test_a_payload_at_rest_on_the_table_is_not_airborne`` -- the first cut of
  the carry test asked whether the payload was a fixed height above the FLOOR.
  The payload's authored rest height is 0.625 m on a 0.60 m table, which
  clears any floor-relative threshold the task could sensibly set, so every
  run in the suite -- including the one where the robot never moved -- began
  its "carry" at t = 0.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders import r4_core as C  # noqa: E402


class _Body:
    """A :class:`~agentbench.graders.evidence.Body`-alike, minus the adapter."""

    def __init__(self, lo, hi, name="", robot_class=False, member_of=None):
        self.aabb_min, self.aabb_max = tuple(lo), tuple(hi)
        self.name = name
        self.body_id = name
        self.robot_class = robot_class
        self.member_of = member_of
        self.position = tuple((a + b) / 2.0 for a, b in zip(lo, hi))
        self.dynamic = True

    has_aabb = True

    @property
    def aabb(self):
        return (self.aabb_min, self.aabb_max)

    @property
    def top_z(self):
        return self.aabb_max[2]


def _table():
    return _Body(C.TABLE_AABB[0], C.TABLE_AABB[1], "whatever the agent called it")


def _pad():
    return _Body(C.PAD_AABB[0], C.PAD_AABB[1], "delivery zone")


class TestFixtureMatching(unittest.TestCase):
    def test_a_perfect_fixture_match_is_accepted(self):
        """A body authored at the spec to the digit must MATCH. See module
        docstring: the falsy-zero spelling rejected exactly these."""
        bodies = [_table(), _pad()]
        table, n = C.find_fixture(bodies, C.TABLE_AABB)
        self.assertIsNotNone(table)
        self.assertEqual(n, 1)
        self.assertEqual(C.aabb_error(table, *C.TABLE_AABB), 0.0)

    def test_the_table_and_the_pad_do_not_match_each_other(self):
        bodies = [_table(), _pad()]
        table, _ = C.find_fixture(bodies, C.TABLE_AABB)
        pad, n = C.find_fixture(bodies, C.PAD_AABB, exclude=[table])
        self.assertIs(pad, bodies[1])
        self.assertEqual(n, 1)

    def test_a_raised_table_is_rejected(self):
        """R4.4-R4.8 are all stated against the table top, so a table the agent
        moved must fail R4.2 rather than silently redefining them."""
        lo, hi = C.TABLE_AABB
        moved = _Body(lo, (hi[0], hi[1], hi[2] + 0.10))
        table, n = C.find_fixture([moved], C.TABLE_AABB)
        self.assertIsNone(table)
        self.assertEqual(n, 0)

    def test_obstacles_are_matched_by_geometry_not_by_name(self):
        """An agent may call its boxes anything -- R1's first real agent called
        them 'crate A'..'crate E' and a correct world scored zero obstacles."""
        spec = C.obstacle_spec()
        bodies = []
        for i, o in enumerate(spec):
            cx, cy = o["position"][0], o["position"][1]
            sx, sy, sz = o["size"]
            bodies.append(_Body((cx - sx / 2, cy - sy / 2, 0.0),
                                (cx + sx / 2, cy + sy / 2, sz),
                                name="crate %s" % chr(65 + i)))
        found, missing = C.match_spec_obstacles(bodies, spec)
        self.assertEqual(len(found), C.N_OBSTACLES)
        self.assertEqual(missing, [])


class TestAirborne(unittest.TestCase):
    def test_a_payload_at_rest_on_the_table_is_not_airborne(self):
        """The measurement that says "carried" means "resting on nothing"."""
        supports = [_table(), _pad()]
        at_rest = [(C.TABLE_XY[0], C.TABLE_XY[1], C.PAYLOAD_REST_Z_TABLE_M)]
        self.assertEqual(C.airborne_mask(at_rest, supports), [False])

    def test_a_payload_at_rest_on_the_pad_is_not_airborne(self):
        supports = [_table(), _pad()]
        at_rest = [(C.PAD_XY[0], C.PAD_XY[1], C.PAYLOAD_REST_Z_PAD_M)]
        self.assertEqual(C.airborne_mask(at_rest, supports), [False])

    def test_a_payload_held_over_the_table_is_airborne(self):
        supports = [_table(), _pad()]
        held = [(C.TABLE_XY[0], C.TABLE_XY[1],
                 C.TABLE_TOP_Z_M + C.PAYLOAD_SIZE_M / 2.0
                 + C.CARRY_CLEARANCE_M + 0.01)]
        self.assertEqual(C.airborne_mask(held, supports), [True])

    def test_a_payload_perched_on_an_obstacle_is_not_airborne(self):
        """0.5 m up is well clear of the FLOOR; it is not clear of the box it
        is sitting on, and that is the distinction the mask exists for."""
        o = C.obstacle_spec()[0]
        cx, cy, _cz = o["position"]
        sx, sy, sz = o["size"]
        box = _Body((cx - sx / 2, cy - sy / 2, 0.0),
                    (cx + sx / 2, cy + sy / 2, sz))
        perched = [(cx, cy, sz + C.PAYLOAD_SIZE_M / 2.0)]
        self.assertEqual(C.airborne_mask(perched, [box]), [False])

    def test_support_height_falls_back_to_the_floor(self):
        self.assertEqual(C.support_height((0.0, 0.0, 1.0), []), 0.0)


class TestCarryWindow(unittest.TestCase):
    """The grasp proof, in the two ways it can be faked."""

    @staticmethod
    def _drive(n, dt=0.016, offset=(0.25, 0.0, 0.70), rigid=True):
        """A base driving +y, with a payload rigidly attached (or not)."""
        t = [i * dt for i in range(n)]
        base, pay = [], []
        for i in range(n):
            y = i * 0.5 * dt
            base.append((0.0, y, 0.0))
            if rigid:
                pay.append((offset[0], y + offset[1], offset[2]))
            else:                       # left behind on a shelf
                pay.append((offset[0], offset[1], offset[2]))
        return t, base, pay

    def test_a_rigidly_carried_payload_yields_one_long_rigid_window(self):
        t, base, pay = self._drive(1200)
        w = C.rigid_window(t, base, pay, 0, len(t) - 1,
                           C.CARRY_RADIUS_TOL_M, C.CARRY_DZ_TOL_M)
        self.assertGreaterEqual(w["base_travel"], C.TRANSPORT_MIN_M)
        self.assertGreaterEqual(w["payload_travel"], C.TRANSPORT_MIN_M)
        self.assertLessEqual(w["radius_spread"], C.CARRY_RADIUS_TOL_M)
        self.assertLessEqual(w["dz_spread"], C.CARRY_DZ_TOL_M)

    def test_a_payload_left_behind_yields_no_transport(self):
        t, base, pay = self._drive(1200, rigid=False)
        w = C.rigid_window(t, base, pay, 0, len(t) - 1,
                           C.CARRY_RADIUS_TOL_M, C.CARRY_DZ_TOL_M)
        self.assertLess(w["base_travel"], C.TRANSPORT_MIN_M)

    def test_the_lift_and_the_set_down_do_not_poison_the_window(self):
        """The whole airborne episode necessarily begins with an extending arm
        and ends with a folding one. MEASURED on a complete, successful oracle
        run: whole-episode spreads of 0.295 m (radius) and 0.570 m (height),
        entirely from those two ends -- a constancy test over the episode fails
        a run that did the task perfectly."""
        t, base, pay = self._drive(1200)
        # bolt a 100-sample lift on the front and a set-down on the back
        pre_t = [-1.6 + i * 0.016 for i in range(100)]
        pre_b = [(0.0, 0.0, 0.0)] * 100
        pre_p = [(0.75, 0.0, 0.63 + 0.001 * i) for i in range(100)]
        post_t = [t[-1] + 0.016 * (i + 1) for i in range(100)]
        post_b = [base[-1]] * 100
        post_p = [(0.75, base[-1][1], 0.70 - 0.005 * i) for i in range(100)]
        T, B, P = pre_t + t + post_t, pre_b + base + post_b, pre_p + pay + post_p
        whole = C.rigid_window(T, B, P, 0, len(T) - 1, 1e9, 1e9)
        self.assertGreater(whole["radius_spread"], C.CARRY_RADIUS_TOL_M)
        w = C.rigid_window(T, B, P, 0, len(T) - 1,
                           C.CARRY_RADIUS_TOL_M, C.CARRY_DZ_TOL_M)
        self.assertGreaterEqual(w["base_travel"], C.TRANSPORT_MIN_M)
        self.assertLessEqual(w["radius_spread"], C.CARRY_RADIUS_TOL_M)

    def test_two_episodes_are_counted_as_two(self):
        """'a dropped or re-acquired payload is a failure' -- R4.7 counts."""
        t = [i * 0.016 for i in range(600)]
        mask = [True] * 200 + [False] * 100 + [True] * 300
        runs = C.maximal_runs(mask, t, C.AIRBORNE_MIN_S)
        self.assertEqual(len(runs), 2)


class TestPlacement(unittest.TestCase):
    def test_the_published_layout_is_itself_legal(self):
        legal = C.layout_legality(C.obstacle_spec())
        self.assertTrue(legal["legal"], legal)
        self.assertGreaterEqual(legal["blocked_length_m"],
                                C.PLACEMENT_MIN_BLOCK_M)
        self.assertTrue(legal["route_to_table"] and legal["route_to_pad"])

    def test_sampling_is_pure_and_deterministic(self):
        a = C.sample_layout(7)
        b = C.sample_layout(7)
        self.assertEqual(a, b)
        self.assertNotEqual(a, C.sample_layout(8))

    def test_every_drawn_layout_keeps_the_published_sizes(self):
        """The COUNT and the SIZES are the published contract; only the
        POSITIONS are redrawn."""
        for seed in (1, 2, 3, 42):
            self.assertEqual(C.obstacle_sizes(C.sample_layout(seed)),
                             C.obstacle_sizes())

    def test_a_drawn_layout_is_legal_by_construction(self):
        for seed in (1, 2, 3, 42, 20260811):
            legal = C.layout_legality(C.sample_layout(seed))
            self.assertTrue(legal["legal"], (seed, legal))

    def test_the_sidecar_round_trips(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            want = C.sample_layout(11)
            C.write_graded_layout(d, 11, want)
            got, source = C.resolve_graded_layout(directories=(d,))
            self.assertEqual(got, want)
            self.assertIn(C.LAYOUT_SIDECAR, source)

    def test_no_placement_resolves_to_nothing_rather_than_the_published(self):
        """``(None, None)`` is what makes grade() say out loud that the row is
        not evidence of perception, instead of quietly scoring the layout the
        agent has already seen -- the memoriser's best case."""
        self.assertEqual(C.resolve_graded_layout(), (None, None))


class TestHitSet(unittest.TestCase):
    def test_the_payload_table_and_pad_are_not_collisions(self):
        """Reaching into them is the job."""
        table, pad = _table(), _pad()
        payload = _Body((2.975, 2.975, 0.60), (3.025, 3.025, 0.65), "cube")
        names, unbounded = C.hittable_bodies([table, pad, payload], None, [],
                                             spare=[table, pad, payload])
        self.assertEqual(names, set())
        self.assertEqual(unbounded, [])

    def test_a_wall_is_a_collision_by_geometry_as_well_as_by_name(self):
        anon = _Body((5.0, -5.1, 0.0), (5.1, 5.1, 0.5), "boundary")
        names, _ = C.hittable_bodies([anon], None, [])
        self.assertIn("boundary", names)

    def test_a_body_with_no_bounds_is_reported_not_counted(self):
        class _NoBounds(_Body):
            has_aabb = False
        b = _NoBounds((0, 0, 0), (1, 1, 1), "mesh hull")
        names, unbounded = C.hittable_bodies([b], None, [])
        self.assertEqual(names, set())
        self.assertEqual(unbounded, ["mesh hull"])


class TestThresholdsComeFromTheContract(unittest.TestCase):
    def test_every_constant_is_read_from_meta_json(self):
        import json
        meta = json.loads((C.TASK_DIR / "meta.json").read_text("utf-8"))
        c = meta["constants"]
        for name, key in (("CARRY_CLEARANCE_M", "CARRY_CLEARANCE_M"),
                          ("TRANSPORT_MIN_M", "TRANSPORT_MIN_M"),
                          ("GRIP_PROXIMITY_M", "GRIP_PROXIMITY_M"),
                          ("DELIVERY_TOL_M", "DELIVERY_TOL_M"),
                          ("HOLD_S", "HOLD_S"),
                          ("MIN_ROBOT_JOINTS", "MIN_ROBOT_JOINTS")):
            self.assertEqual(getattr(C, name), c[key], name)

    def test_scene_geometry_is_read_from_the_shipped_asset(self):
        s = C.scene_spec()
        self.assertEqual(C.TABLE_TOP_Z_M, s["table"]["top_z"])
        self.assertEqual(C.PAD_TOP_Z_M, s["pad"]["top_z"])
        self.assertEqual(C.PAYLOAD_SIZE_M, s["payload"]["size"])
        self.assertAlmostEqual(C.PAYLOAD_REST_Z_TABLE_M,
                               s["payload"]["rest_z_on_table"], places=9)
        self.assertAlmostEqual(C.PAYLOAD_REST_Z_PAD_M,
                               s["payload"]["rest_z_on_pad"], places=9)

    def test_the_two_arms_ship_byte_identical_assets(self):
        d = C.TASK_DIR
        for name in ("obstacles.json", "scene.json"):
            a = (d / "initial" / "benchmark_assets" / name).read_bytes()
            b = (d / "initial_webots" / "benchmark_assets" / name).read_bytes()
            self.assertEqual(a, b, name)

    def test_the_task_is_geometrically_solvable_at_all(self):
        """A sanity floor on the fixture itself: the straight start->table line
        is blocked, and the table and the pad are far enough apart that the
        transport clause is not satisfiable by standing still."""
        self.assertGreater(C.STRAIGHT_TABLE_PAD_M, C.TRANSPORT_MIN_M)
        blocked = C.segment_blocked_length(C.spec_bodies(C.obstacle_spec()))
        self.assertGreaterEqual(blocked, C.PLACEMENT_MIN_BLOCK_M)
        self.assertAlmostEqual(
            C.STRAIGHT_START_TABLE_M,
            math.hypot(C.TABLE_XY[0] - C.START_XY[0],
                       C.TABLE_XY[1] - C.START_XY[1]), places=9)


if __name__ == "__main__":
    unittest.main()
