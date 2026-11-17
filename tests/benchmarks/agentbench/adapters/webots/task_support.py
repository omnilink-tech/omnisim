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

"""Task-world support for the Lane B task worlds on the upstream-Webots arm.

Closes BRINGUP.md sec. 4 gap 1 **for the Lane B task worlds** without touching
the grader's recorder or the launcher: the Lane B graders consume world-space
AABBs (B1 pairwise overlap, B3 taller-by-top-z, C2 rests-on-floor, B2 subject
bounds) and a camera pose (B2), and upstream's Supervisor API serves neither
directly. This module runs a **separate, grader-owned measurement launch** of
a world -- byte-identical text, same launcher, same offline asset cache --
with the ``agentbench_aabb_prober`` controller injected, and merges what the
prober measured into the graded run's roster (the nested ``bounds`` layout
:mod:`agentbench.adapters.webots.evidence` already accepts).

Why a separate launch instead of injecting the prober into the graded run:
the grader's recorder enumerates every top-level body, so a second Robot node
in the *graded* world would pollute the graded roster (an extra un-measurable
robot body flips B1 to INVALID). The two launches load the same world text
frozen at t=0 under a synchronization-TRUE Supervisor, so the t=0 geometry is
the same by construction; the merge provenance is recorded on the returned
document so a reviewer sees which numbers came from which launch.

Nothing here is imported by the graders or the launcher -- callers (the
oracle driver, tests) opt in explicitly.
"""

from __future__ import annotations

import json
import math
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from agentbench.adapters.webots import launcher, recording  # noqa: E402
from agentbench.graders.evidence import (  # noqa: E402
    CameraPose, ViewEvidence)

PROBER_NAME = "agentbench_aabb_prober"
PROBER_DIR = launcher.WEBOTS_LANE / "controllers" / PROBER_NAME

PROBER_STANZA = """
# --- appended by agentbench.adapters.webots.task_support (measurement) ------
Robot {
  name "%s"
  controller "%s"
  supervisor TRUE
  controllerArgs [
    "--out-dir=%s"
  ]
}
"""


def prober_stanza(out_dir_wsl, *, name=PROBER_NAME, controller=PROBER_NAME):
    return PROBER_STANZA % (name, controller, out_dir_wsl)


def probe_world(world, run_dir, *, duration=1.0, timeout_s=240.0,
                port=launcher.DEFAULT_PORT):
    """Measure ``world``'s t=0 AABBs + Viewpoint. Returns ``(doc, facts)``.

    ``doc`` is the prober's ``aabbs.json`` content (or ``None`` when the
    measurement run left none -- a world that fails to load, for instance),
    ``facts`` the launcher's ``process.json`` for the measurement run. Never
    raises on a failed run: a broken measurement must be reportable.
    """
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    world = Path(world)

    # A private project: the world copy + the prober controller, so the
    # launcher's own project-copy step ships the prober into WSL.
    proj = run_dir / "probe_proj"
    worlds_dir = proj / "worlds"
    ctrl_dir = proj / "controllers" / PROBER_NAME
    worlds_dir.mkdir(parents=True, exist_ok=True)
    if ctrl_dir.exists():
        shutil.rmtree(ctrl_dir, ignore_errors=True)
    ctrl_dir.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(PROBER_DIR, ctrl_dir)

    out_dir_wsl = launcher.win_to_wsl_path(run_dir.resolve())
    text = world.read_text(encoding="utf-8", errors="replace")
    if not text.endswith("\n"):
        text += "\n"
    probe_world_path = worlds_dir / world.name
    probe_world_path.write_text(text + prober_stanza(out_dir_wsl),
                                encoding="utf-8")

    _dir, facts = launcher.launch(probe_world_path, run_dir / "launch",
                                  duration=duration, contact_steps=0,
                                  timeout_s=timeout_s, port=port)
    doc = None
    aabbs = run_dir / "aabbs.json"
    if aabbs.is_file():
        try:
            doc = json.loads(aabbs.read_text(encoding="utf-8"))
        except ValueError as exc:
            facts = dict(facts, aabbs_parse_error=repr(exc))
    return doc, facts


def augment_run(run_dir, aabbs_doc):
    """``recording.read_run(run_dir)`` with the prober's AABBs merged in.

    Bounds are merged **by body name** into the roster records, in the nested
    ``bounds: {bbox_min, bbox_max}`` layout the Webots evidence adapter
    already reads. Bodies the prober could not bound stay unbounded -- an
    absent AABB must stay absent (adapter rule: never invent a witness).
    """
    run = recording.read_run(run_dir)
    merged = []
    if run.roster and isinstance(run.roster.get("bodies"), list):
        by_name = (aabbs_doc or {}).get("bodies") or {}
        for rec in run.roster["bodies"]:
            if not isinstance(rec, dict):
                continue
            got = by_name.get(rec.get("name"))
            if (isinstance(got, dict) and got.get("bbox_min")
                    and got.get("bbox_max") and "bounds" not in rec):
                rec["bounds"] = {"bbox_min": got["bbox_min"],
                                 "bbox_max": got["bbox_max"]}
                merged.append(rec.get("name"))
        run.roster["aabb_merge"] = {
            "merged_bodies": merged,
            "source": (aabbs_doc or {}).get("source"),
            "provenance": ("world-space AABBs measured by a separate "
                           "synchronization-TRUE t=0 prober launch of the "
                           "byte-identical world text (task_support.py); "
                           "merged by body name"),
        }
    return run, merged


# --- the B2 camera channel ---------------------------------------------------

# Upstream R2022b+ viewpoint convention (inherited by this fork): the camera
# frame is FLU -- +x forward, +y left, +z up. Viewpoint.fieldOfView is the
# horizontal angle.
_FWD = (1.0, 0.0, 0.0)
_UP = (0.0, 0.0, 1.0)


def _rot(axis_angle, v):
    x, y, z, a = axis_angle
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return tuple(v)
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(a), math.sin(a)
    C = 1.0 - c
    m = ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
         (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
         (z * x * C - y * s, z * y * C + x * s, c + z * z * C))
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def camera_pose(aabbs_doc, *, source):
    """The prober's recorded Viewpoint as a neutral :class:`CameraPose`."""
    vp = (aabbs_doc or {}).get("viewpoint") or {}
    pos = vp.get("position")
    rot = vp.get("orientation")
    if not pos or not rot:
        return None
    fov = vp.get("field_of_view")
    return CameraPose(
        position=tuple(float(v) for v in pos),
        forward=_rot(rot, _FWD),
        up=_rot(rot, _UP),
        fov_h_rad=float(fov) if fov is not None else 0.785398,
        source=source + " (Viewpoint read live by the t=0 prober; FLU "
                        "camera frame, fieldOfView horizontal)")


def view_evidence(initial_doc, final_doc, *, artifact=None):
    """B2's :class:`ViewEvidence` from two prober measurements.

    ``artifact_parsed`` is True when the (edited) artifact world produced a
    prober measurement, False when the artifact launch left none -- the
    agent-broke-the-scene case B2.1 fails on.
    """
    src = "agentbench.adapters.webots.task_support"
    final = camera_pose(final_doc, source=src + " [artifact world]")
    return ViewEvidence(
        initial=camera_pose(initial_doc, source=src + " [shipped world]"),
        final=final,
        artifact_parsed=(True if final is not None else False),
        artifact=str(artifact) if artifact else None,
        source=src)


__all__ = ["PROBER_DIR", "PROBER_NAME", "augment_run", "camera_pose",
           "probe_world", "prober_stanza", "view_evidence"]
