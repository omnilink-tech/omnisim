"""Regression checks for the versioned R2.5 ground-datum correction."""

from __future__ import annotations

import unittest

import numpy as np

from agentbench.graders import r2_core as legacy
from agentbench.graders.evidence import (
    Body,
    BodyInventory,
    EngineAttribution,
    EvidenceBundle,
    ProcessFacts,
    Trajectory,
)
from agentbench.preregister.v1 import r2_core_v1


def underground_bundle():
    t = np.arange(0.0, 1.0, 0.016)
    base = np.zeros((len(t), 3))
    tip = np.tile(np.array([0.8, 0.0, -0.2]), (len(t), 1))
    arm = Body(
        body_id="ARM",
        name="arm",
        kind="Robot",
        n_joints=6,
        robot_class=True,
        aabb_min=(-0.1, -0.1, -0.235),
        aabb_max=(0.835, 0.1, 0.1),
    )
    bundle = EvidenceBundle(
        task="R2_arm_reach",
        sim="synthetic",
        adapter="v1-regression",
        artifact="arm_reach.wbt",
    )
    bundle.roster = BodyInventory(
        bodies=[arm], frozen=True, t_s=0.0, source="test")
    bundle.t0 = BodyInventory(
        bodies=[arm], frozen=True, t_s=0.0, source="test")
    bundle.trajectory = Trajectory(
        body_ids=["ARM", "ARM/link0"],
        t=t,
        xyz=np.stack([base, tip]),
        dt_s=0.016,
        recorded_s=float(t[-1]),
        complete=True,
        source="test",
    )
    bundle.process = ProcessFacts(
        exit_code=0, error_lines=[], driver_completed=True)
    bundle.attribution = EngineAttribution(
        backend="newton", solver="mujoco", source="test")
    return bundle


class R2V1GroundDatumTests(unittest.TestCase):
    def test_legacy_freeze_is_preserved_but_v1_catches_the_dive(self):
        old = legacy.grade(underground_bundle())
        new = r2_core_v1.grade(underground_bundle())
        old_r25 = next(a for a in old.assertions if a.id == "R2.5")
        new_r25 = next(a for a in new.assertions if a.id == "R2.5")
        self.assertTrue(old_r25.ok)
        self.assertEqual(old_r25.measured["ground datum (m)"], -0.235)
        self.assertFalse(new_r25.ok)
        self.assertEqual(new_r25.measured["ground datum (m)"], 0.0)
        self.assertEqual(
            new_r25.measured["lowest tip clearance over datum (m)"], -0.2)
        self.assertIn("fixed arm base origin", new_r25.detail)


if __name__ == "__main__":
    unittest.main()
