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

"""L2's phase B on the MuJoCo column: re-run a deliverable, record the base.

Why this file exists rather than reusing a sibling. ``runner_t2`` re-runs a
deliverable of exactly the right SHAPE -- a directory holding ``scene.xml`` and
``drive.py`` -- but records a manipulation scene's channels: object, end
effector, container geometry, grips. ``run_t1`` records the right CHANNEL but
drives with the column's own controller rather than re-running an agent's. L2
needs the intersection: somebody else's driver, and one body's pose.

The deliverable contract is the column's existing one, unchanged, so an agent
working on this column is not asked to learn a shape invented for this rung.

Output is written as ``phaseB.csv`` with the same wide-format columns the
ladder's own recorder writes (``t, r0_x, r0_y, r0_z``), so the neutral core
receives byte-identical input shapes from both columns and a difference in a
verdict cannot be a difference in parsing.
"""

from __future__ import annotations

import csv
import runpy
import sys
import traceback
from pathlib import Path

DRIVER_NAME = "drive.py"
SCENE_NAME = "scene.xml"
RECORD_EVERY = 1


def resolve_deliverable(path):
    """``(scene, driver)`` from a deliverable directory or a scene file."""
    p = Path(path)
    if p.is_dir():
        scene = p / SCENE_NAME
    else:
        scene = p
    if not scene.is_file():
        return None, None
    # Whatever the agent called its controller -- see runner_t2.find_driver for
    # why requiring the exact name `drive.py` made this column measure a naming
    # convention nothing tells the agent about.
    from ladder.adapters.mujoco.runner_t2 import find_driver
    return scene, find_driver(scene.parent)


def _base_body_id(mujoco, model, declared):
    """The graded body: the declared name, else the free-jointed root."""
    if declared:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, declared)
        if bid >= 0:
            return bid, "the body carrying the declared name %r" % declared
    for j in range(model.njnt):
        if int(model.jnt_type[j]) == int(mujoco.mjtJoint.mjJNT_FREE):
            bid = int(model.jnt_bodyid[j])
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
            return bid, ("no body is named %r; fell back to the free-jointed "
                         "root %r" % (declared, name))
    return -1, "no body carries the declared name and the model has no free joint"


class Result:
    def __init__(self, rc, note, samples=0, attempts_used=1):
        self.rc = rc
        self.note = note
        self.samples = samples
        self.attempts_used = attempts_used

    def as_dict(self):
        return {"rc": self.rc, "note": self.note, "samples": self.samples,
                "attempts_used": self.attempts_used}


def run_standalone(deliverable, run_dir, *, duration=30.0, settle=0.5,
                   robot_name="husky", **kw):
    """Load the deliverable's scene, drive it with ITS driver, record the base.

    Never raises for a physical failure: a scene that will not compile and a
    driver that throws are both measurements, and both come back as a
    ``phaseB.csv`` that the core reads as an unusable run.
    """
    out = Path(run_dir)
    out.mkdir(parents=True, exist_ok=True)
    scene, driver = resolve_deliverable(deliverable)
    if scene is None:
        return Result(7, "no %s in the deliverable" % SCENE_NAME)

    try:
        import mujoco
    except ImportError as exc:
        return Result(8, "mujoco is not importable: %r" % (exc,))

    try:
        model = mujoco.MjModel.from_xml_path(str(scene))
    except (ValueError, RuntimeError) as exc:
        (out / "stderr.log").write_text("scene refused: %r" % (exc,),
                                        encoding="utf-8")
        return Result(7, "the scene did not compile: %s" % exc)
    data = mujoco.MjData(model)

    bid, why = _base_body_id(mujoco, model, robot_name)
    if bid < 0:
        return Result(9, why)

    control = None
    if driver is not None:
        try:
            ns = runpy.run_path(str(driver))
            control = ns.get("control") or ns.get("step") or ns.get("policy")
        except Exception as exc:                       # noqa: BLE001
            (out / "stderr.log").write_text(
                "driver import failed: %r\n%s" % (exc, traceback.format_exc()),
                encoding="utf-8")
            return Result(6, "the driver did not import: %s" % exc)

    dt = float(model.opt.timestep)
    n_settle = int(max(0.0, settle) / dt)
    n_run = int(max(0.0, duration) / dt)

    rows = []
    err = None
    for step in range(n_settle + n_run):
        t = step * dt
        if control is not None and step >= n_settle:
            try:
                control(model, data, t - n_settle * dt)
            except Exception as exc:                   # noqa: BLE001
                err = "the driver raised at t=%.3f s: %r" % (t, exc)
                break
        mujoco.mj_step(model, data)
        if step % RECORD_EVERY == 0:
            p = data.xpos[bid]
            rows.append((round(t, 6), float(p[0]), float(p[1]), float(p[2])))

    csv_path = out / "phaseB.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "r0_x", "r0_y", "r0_z"])
        w.writerows(rows)
    (out / "phaseB.csv.meta.json").write_text(
        '{"column": "mujoco", "base_selection": %r, "timestep_s": %r, '
        '"samples": %d, "driver": %r}'
        % (why, dt, len(rows), str(driver) if driver else None),
        encoding="utf-8")

    if err:
        (out / "stderr.log").write_text(err, encoding="utf-8")
        return Result(5, err, samples=len(rows))
    return Result(0, "ok: %s" % why, samples=len(rows))


#: The name L2's phase-B lookup asks for on this column.
l2_run_standalone = run_standalone
