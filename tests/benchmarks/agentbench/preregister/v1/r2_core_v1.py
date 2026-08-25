"""AgenticSimBench v1 corrections layered over the frozen v0.3 R2 grader.

The legacy grader is hash-frozen and must remain reproducible.  V1 changes one
thing: R2.5 uses the fixed base origin as its simulator-neutral mounting datum.
An articulated Robot AABB includes moving descendants on OmniSim, so using its
bottom lets an underground tip move its own supposed floor underground.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from agentbench.graders import r2_core as legacy


GRADER_ID = "agenticsimbench/v1/r2_core"
LEGACY_SOURCE = Path(legacy.__file__).resolve()
RECORD_DURATION_S = legacy.RECORD_DURATION_S
GROUND_CLEARANCE_TOL_M = legacy.GROUND_CLEARANCE_TOL_M
MIN_ARM_JOINTS = legacy.MIN_ARM_JOINTS


def _assertion(verdict, aid):
    return next((a for a in verdict.assertions if a.id == aid), None)


def grade(bundle, *, self_verified=False):
    verdict = legacy.grade(bundle, self_verified=self_verified)
    assertion = _assertion(verdict, "R2.5")
    if assertion is None or assertion.measured is None:
        return verdict

    robots = [body for body in bundle.roster.bodies if body.robot_class]
    arms = [
        body for body in robots
        if (body.n_joints or 0) >= legacy.MIN_ARM_JOINTS
    ]
    trajectory = bundle.trajectory
    if len(arms) != 1 or trajectory is None or trajectory.xyz is None:
        return verdict
    base_row = legacy._traj_row(bundle, trajectory, arms[0])
    if base_row is None:
        return verdict
    ee_row, _report = legacy.select_end_effector(trajectory.xyz, base_row)
    if ee_row is None:
        return verdict

    base_xyz = np.asarray(trajectory.xyz[base_row], dtype=float)
    ee_xyz = np.asarray(trajectory.xyz[ee_row], dtype=float)
    datum_z = float(base_xyz[0][2])
    min_ee_z = float(ee_xyz[:, 2].min())
    clearance = min_ee_z - datum_z
    labels = bundle.lbl
    assertion.ok = clearance >= -legacy.GROUND_CLEARANCE_TOL_M
    assertion.measured = {
        labels("min_ee_z", "lowest tip height (m)"): round(min_ee_z, 6),
        labels("ground_datum", "ground datum (m)"): round(datum_z, 6),
        labels("clearance", "lowest tip clearance over datum (m)"):
            round(clearance, 6),
    }
    assertion.threshold = {
        labels("clearance", "lowest tip clearance over datum (m)"):
            ">= %.3f" % (-legacy.GROUND_CLEARANCE_TOL_M),
    }
    assertion.detail = (
        "datum: the fixed arm base origin at t=0. GEOMETRIC, not a contact "
        "read. AgenticSimBench v1 deliberately does not use the articulated "
        "Robot AABB: on OmniSim it includes moving descendants, so an "
        "underground tip moves that supposed floor underground with itself."
    )
    verdict.note(
        "R2.5 graded by agenticsimbench/v1/r2_core: fixed-base mounting datum, "
        "not the frozen v0.3 articulated-subtree AABB."
    )
    return verdict.finish()


def __getattr__(name):
    """Keep the v1 runner on the frozen core's other constants and helpers."""
    return getattr(legacy, name)
