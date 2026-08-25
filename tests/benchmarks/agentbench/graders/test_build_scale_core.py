"""Discrimination tests for the frozen BuildScale v1 core."""

from __future__ import annotations

import copy
import unittest

import numpy as np

from agentbench.preregister.v1.build_scale_core import (
    BuildScaleEvidence, ControlObservation, PortableObservation, grade)
from agentbench.graders.evidence import (Body, BodyInventory, ContactObservation,
                                         ContactPair, EngineAttribution,
                                         ProcessFacts, Trajectory)


def good_evidence(n: int = 10) -> BuildScaleEvidence:
    ids = [f"robot-{i:03d}" for i in range(n)]
    bodies, t0bodies = [], []
    starts = []
    for i, rid in enumerate(ids):
        x, y = float((i % 10) * 2), float((i // 10) * 2)
        starts.append((x, y, 0.25))
        common = dict(body_id=rid, name=f"robot_{i:03d}", kind="mobile_robot",
                      position=(x, y, 0.25), aabb_min=(x - 0.2, y - 0.2, 0.0),
                      aabb_max=(x + 0.2, y + 0.2, 0.5), robot_class=True,
                      behaviour=f"fleet/{rid}", behaviour_declared="fleet")
        bodies.append(Body(**common))
        t0bodies.append(Body(**common))

    t = np.linspace(0.0, 20.0, 201)
    xyz = np.zeros((n, len(t), 3), dtype=float)
    for i, (x, y, z) in enumerate(starts):
        angle = (i % 8) * np.pi / 4.0
        distance = 2.5 * (t / 20.0)
        xyz[i, :, 0] = x + np.cos(angle) * distance
        xyz[i, :, 1] = y + np.sin(angle) * distance
        xyz[i, :, 2] = z

    return BuildScaleEvidence(
        requested_robots=n,
        sim="testsim",
        adapter="tests.good",
        artifact="portable/build_scale.world",
        live_load_ok=True,
        roster=BodyInventory(bodies=bodies, t_s=0.0, frozen=True,
                             source="frozen live roster"),
        t0=BodyInventory(bodies=t0bodies, t_s=0.0, frozen=True,
                         source="frozen world-space bounds"),
        trajectory=Trajectory(body_ids=ids, t=t, xyz=xyz, dt_s=0.1,
                              recorded_s=20.0, complete=True,
                              source="every-step pose recorder"),
        contacts_initial=ContactObservation(
            pairs=[], steps=10, window_s=1.0, supported=True,
            total_observed=n, distinct_named=n,
            source="native contact recorder"),
        contacts_window=ContactObservation(
            pairs=[], steps=200, window_s=20.0, supported=True,
            total_observed=n * 200, distinct_named=n * 200,
            source="native contact recorder"),
        controls=ControlObservation(
            channel_by_robot={rid: f"channel/{rid}" for rid in ids},
            started_by_robot={rid: True for rid in ids},
            forbidden_world_mutation_calls=[],
            source="per-robot runtime controller ledger + source scan"),
        portability=PortableObservation(
            artifact_files=["build_scale.world", "controllers/fleet.py"],
            clean_replay_attempts=1, clean_replay_passes=1,
            clean_replay_verdicts=["PASS"],
            source="clean temporary-directory replay"),
        process=ProcessFacts(
            exit_code=0, timed_out=False, error_lines=[], log_available=True,
            log_source="simulator.log", driver_completed=True),
        attribution=EngineAttribution(
            backend="test-physics", solver="test-solver",
            source="compiled model report"),
    )


class TestBuildScaleCore(unittest.TestCase):
    def test_positive_control_passes_all_nine_assertions(self):
        verdict = grade(good_evidence())
        self.assertEqual(verdict.outcome, "PASS", verdict.summary())
        self.assertEqual(len(verdict.assertions), 9)
        self.assertFalse(verdict.failed)

    def test_null_control_fails_movement(self):
        evidence = good_evidence()
        evidence.trajectory.xyz[:] = evidence.trajectory.xyz[:, :1, :]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.5"], verdict.summary())

    def test_partial_control_is_not_independent_control(self):
        evidence = good_evidence()
        missing = evidence.trajectory.body_ids[-1]
        evidence.controls.channel_by_robot.pop(missing)
        evidence.controls.started_by_robot.pop(missing)
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.3"], verdict.summary())

    def test_gradual_world_pose_mutation_is_not_a_controller(self):
        evidence = good_evidence()
        evidence.controls.forbidden_world_mutation_calls = [
            "controllers/fleet.py:42 node.getField('translation').setSFVec3f"]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.3"], verdict.summary())

    def test_initial_overlap_is_caught(self):
        evidence = good_evidence()
        evidence.t0.bodies[1].aabb_min = evidence.t0.bodies[0].aabb_min
        evidence.t0.bodies[1].aabb_max = evidence.t0.bodies[0].aabb_max
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.4"], verdict.summary())

    def test_more_than_ten_percent_stationary_fails(self):
        evidence = good_evidence()
        evidence.trajectory.xyz[-2:] = evidence.trajectory.xyz[-2:, :1, :]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.5"], verdict.summary())

    def test_gradual_runaway_is_not_mislabelled_teleport(self):
        evidence = good_evidence()
        t = evidence.trajectory.t
        start = evidence.trajectory.xyz[0, 0].copy()
        evidence.trajectory.xyz[0, :, 0] = start[0] + 30.0 * (t / 20.0)
        evidence.trajectory.xyz[0, :, 1] = start[1]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.6"], verdict.summary())

    def test_one_collision_pair_breaks_level_ten_bound(self):
        evidence = good_evidence()
        evidence.contacts_window.pairs = [ContactPair(
            a="robot-000", b="robot-001", a_robot=True, b_robot=True,
            step=80)]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.7"], verdict.summary())

    def test_teleport_is_caught_after_other_physical_requirements_pass(self):
        evidence = good_evidence()
        evidence.trajectory.xyz[0, 100:, 0] += 1.0
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.8"], verdict.summary())

    def test_coarse_sampling_cannot_hide_teleportation(self):
        evidence = good_evidence()
        evidence.trajectory.t = np.linspace(0.0, 20.0, 101)
        evidence.trajectory.xyz = evidence.trajectory.xyz[:, ::2, :]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.8"], verdict.summary())

    def test_nonportable_artifact_fails_clean_replay(self):
        evidence = good_evidence()
        evidence.portability.absolute_host_dependencies = ["C:/author/assets/bot.obj"]
        evidence.portability.clean_replay_passes = 0
        evidence.portability.clean_replay_verdicts = ["FAIL"]
        verdict = grade(evidence)
        self.assertEqual(verdict.failed, ["BS.9"], verdict.summary())

    def test_missing_attribution_is_invalid_not_a_zero(self):
        evidence = good_evidence()
        evidence.attribution = None
        verdict = grade(evidence)
        self.assertEqual(verdict.outcome, "INVALID")
        self.assertIn("BS.1", verdict.failed)

    def test_every_frozen_level_uses_the_same_core(self):
        for level in (10, 25, 50, 100, 250):
            with self.subTest(level=level):
                verdict = grade(good_evidence(level))
                self.assertEqual(verdict.outcome, "PASS", verdict.summary())

    def test_collision_formula_scales_without_becoming_unbounded(self):
        evidence = good_evidence(50)
        evidence.contacts_window.pairs = [ContactPair(
            a="robot-000", b="robot-001", a_robot=True, b_robot=True)]
        self.assertEqual(grade(evidence).outcome, "PASS")  # floor(.02*50) = 1
        evidence.contacts_window.pairs.append(ContactPair(
            a="robot-002", b="robot-003", a_robot=True, b_robot=True))
        self.assertEqual(grade(evidence).failed, ["BS.7"])

    def test_bundle_copy_does_not_share_mutable_evidence(self):
        original = good_evidence()
        clone = copy.deepcopy(original)
        clone.controls.channel_by_robot.clear()
        self.assertEqual(grade(original).outcome, "PASS")
        self.assertEqual(grade(clone).failed, ["BS.3"])


if __name__ == "__main__":
    unittest.main()
