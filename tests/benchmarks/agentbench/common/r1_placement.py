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

"""R1's grade-time obstacle placement -- the INJECTION step, for every arm.

``graders/r1_core.sample_layout`` draws a legal obstacle layout from the
benchmark seed, and ``graders/r1.py`` scores a run against it. Between those
two there has to be a step that MOVES the obstacles in the deliverable the
agent authored and DECLARES the layout it moved them to. This is that step.

Without it the mechanism is a table rather than a defence: measured, a
memorising controller that reads ``benchmark_assets/obstacles.json`` and casts
zero sensor beams passes 6/6 (``adapters/mujoco/mujoco_lane/r1_hardcode.py``),
because the layout it memorised is the layout it is graded on. With it, the
same memoriser drives into a box that moved.

--------------------------------------------------------------------------
THE THREE RULES THIS MODULE IS BUILT AROUND
--------------------------------------------------------------------------

**1. There is no fallback to the published layout.** ``sample_layout`` raises
rather than returning a layout the agent has seen, and this module keeps that
property end to end: every failure -- no legal draw, an obstacle it could not
find, a rewrite that did not land where it was asked -- is a
:class:`PlacementError`, and the caller turns that into a BLOCKED cell. A cell
that cannot be placed produces no row, because the alternative is a row that
scores the layout the agent memorised while looking exactly like a row that
does not.

**2. The obstacles are found by GEOMETRY, never by name.** Agents have called
them ``crate A``..``crate E`` and ``obstacle_1``..``_6``, and the grader
already matches by measured AABB. So does this: ``r1_core.match_spec_obstacles``
-- the grader's own matcher -- runs over bodies read out of the artifact's
text, with a footprint-only second pass for an agent that put its boxes
somewhere of its own choosing. A name is never consulted, on either arm.

**3. The agent never sees the drawn layout.** Placement runs after the agent's
session has ended and writes only into the results tree; nothing it produces
exists while the agent is working, in any file the agent can reach. See
``cc_lane/run_cc_cell.run_cell`` for where in the cell sequence that is, and
why.

--------------------------------------------------------------------------
WHAT "PLACED" MEANS, AND HOW IT IS CHECKED
--------------------------------------------------------------------------

Placement is a **delta on each body's own pose field** -- the ``.wbt``
``translation``, the MJCF ``pos`` -- in x and y only. Working in deltas rather
than absolutes is what makes it correct for a body whose geometry is offset
from its origin, or whose parent is rotated: the AABB centre is what moves to
the drawn position, whatever the body's own origin happens to be. z is left
alone: nothing in R1 is graded on it, and rewriting it would move an agent's
obstacle vertically for no reason.

Afterwards the rewritten text is **re-scanned and re-matched against the drawn
layout** with the grader's matcher and the grader's tolerance, and the straight
start->goal line is re-derived from the placed geometry. A placement that
cannot prove those two facts about its own output fails; it never reports
success on the strength of having edited some bytes.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from agentbench.common import mjcftext, worldtext
from agentbench.graders import r1_core

#: The tasks this step applies to. R2/R3 have no drawn layout.
TASKS = frozenset({r1_core.TASK})

#: How close a placed obstacle's re-measured AABB centre must be to the drawn
#: centre. Far tighter than the grader's 0.05 m match tolerance on purpose:
#: this is OUR arithmetic checking itself, not a tolerance on the agent.
VERIFY_TOL_M = 1e-4

#: What the row records this mechanism as.
MECHANISM = r1_core.PLACEMENT_MECHANISM

#: Re-exported so a driver imports the placer alone and cannot end up writing
#: the handshake under a name the grader does not read.
LAYOUT_SIDECAR = r1_core.LAYOUT_SIDECAR
LAYOUT_DIR_ENV = r1_core.LAYOUT_DIR_ENV


class PlacementError(RuntimeError):
    """Placement could not be completed, so the cell must not be graded.

    Carries ``report`` (whatever was established before the failure) so a
    blocked cell can still say what it saw.
    """

    def __init__(self, message, report=None):
        super().__init__(message)
        self.report = report or {}


def default_seed(task_id, repeat, campaign_id=None):
    """The seed a cell uses when nobody passed one.

    Deterministic and legible -- ``<campaign>/<task>/r<n>`` -- so the same
    repeat of the same task draws the SAME layout on every arm, which is what
    makes a sim-vs-sim comparison a comparison of simulators rather than of
    layouts. It is recorded on the row either way, so any layout is
    re-derivable from the row alone.
    """
    return "%s/%s/r%d" % (campaign_id or "adhoc", task_id, int(repeat))


def draw(seed):
    """``(layout, report)`` -- the drawn layout for one seed, or raise."""
    try:
        layout, rep = r1_core.sample_layout_with_report(seed)
    except r1_core.LayoutError as exc:
        raise PlacementError(
            "no legal obstacle layout could be drawn for seed %r: %s"
            % (seed, exc), {"seed": seed}) from exc
    return layout, rep


# --- finding the obstacles, by geometry -------------------------------------

def _footprint_matches(body, want, tol):
    lo, hi = body.aabb
    return (abs((hi[0] - lo[0]) - float(want["size"][0])) <= tol
            and abs((hi[1] - lo[1]) - float(want["size"][1])) <= tol)


def match_obstacles(bodies, published, tol_m=r1_core.OBSTACLE_POSE_TOL_M):
    """``(pairs, missing, notes)`` -- which body is which published obstacle.

    Two geometric channels, in order:

    * ``r1_core.match_spec_obstacles`` -- position AND footprint, the grader's
      own matcher. An agent that built the published layout (which the prompt
      asks for) is matched here, and matched by exactly the rule that will
      grade it.
    * **footprint only**, nearest first, over whatever is left. An agent that
      put the right five boxes somewhere else has still built the specified
      obstacles, and the placer is about to overwrite their positions anyway --
      so refusing to recognise them would block a cell over a difference that
      placement erases. The published sizes are all distinct, which is what
      makes this channel unambiguous in practice; when it is not, the nearest
      candidate wins and the ambiguity is recorded.
    """
    found, missing = r1_core.match_spec_obstacles(bodies, published, tol_m)
    pairs, notes = {}, []
    stream = iter(found)
    for want in published:
        if want["name"] in missing:
            continue
        pairs[want["name"]] = (next(stream), "position+footprint")

    used = {id(b) for b, _ in pairs.values()}
    for want in published:
        if want["name"] in pairs:
            continue
        cands = [b for b in bodies
                 if id(b) not in used and _footprint_matches(b, want, tol_m)]
        if not cands:
            continue
        cx, cy = float(want["position"][0]), float(want["position"][1])
        cands.sort(key=lambda b: math.hypot(b.centre[0] - cx,
                                            b.centre[1] - cy))
        pairs[want["name"]] = (cands[0], "footprint")
        used.add(id(cands[0]))
        notes.append("%s matched by footprint alone (the agent placed it at "
                     "(%.3f, %.3f), not the published (%.3f, %.3f))"
                     % (want["name"], cands[0].centre[0], cands[0].centre[1],
                        cx, cy))
        if len(cands) > 1:
            notes.append("%s had %d footprint-identical candidates; the "
                         "nearest was taken" % (want["name"], len(cands)))
    still_missing = [w["name"] for w in published if w["name"] not in pairs]
    return pairs, still_missing, notes


def _reject_nested(pairs):
    """Two matched bodies where one CONTAINS the other is not two obstacles.

    It is one obstacle counted twice (a Solid and the Pose inside it can have
    identical bounds), and moving both would move it twice.
    """
    spans = []
    for name, (body, _ch) in pairs.items():
        node = getattr(body, "node", None) or getattr(body, "element", None)
        spans.append((name, getattr(node, "head", getattr(node, "start", 0)),
                      getattr(node, "inner_end", getattr(node, "end", 0))))
    for i, (an, a0, a1) in enumerate(spans):
        for bn, b0, b1 in spans[i + 1:]:
            if (a0 <= b0 and b1 <= a1) or (b0 <= a0 and a1 <= b1):
                return "%s and %s are the same body (one encloses the other)" \
                    % (an, bn)
    return None


# --- the arms ----------------------------------------------------------------

#: Deliverable suffix -> the name this module calls that format. The suffix
#: set is ``agents/external.artifact_suffixes``'s, one arm at a time.
FORMAT_BY_SUFFIX = {".omniworld": "wbt", ".wbt": "wbt", ".xml": "mjcf"}

#: ...and the text module that reads and rewrites it. Two spellings of one
#: mechanism: both offer ``scan_bodies(text)`` returning world-space bodies
#: that ``r1_core.match_spec_obstacles`` can match, and ``move_bodies(text,
#: moves)`` applying world-frame deltas to the one field that moves a body.
_ARMS_BY_FMT = {"wbt": worldtext, "mjcf": mjcftext}


def place_text(text, layout, fmt, *, published=None):
    """``(new_text, report)`` -- one artifact's obstacles moved to ``layout``.

    Pure: no file is read or written, so the whole mechanism is testable
    without an artifact, a simulator or a campaign.
    """
    module = _ARMS_BY_FMT[fmt]
    published = published if published is not None else r1_core.obstacle_spec()
    report = {"format": fmt, "mechanism": MECHANISM}

    bodies = module.scan_bodies(text)
    report["bodies_scanned"] = len(bodies)
    pairs, missing, notes = match_obstacles(bodies, published)
    report["matched"] = {n: {"body_id": b.body_id, "name": b.name or None,
                             "channel": ch, "was": [round(b.centre[0], 4),
                                                    round(b.centre[1], 4)]}
                         for n, (b, ch) in pairs.items()}
    report["missing"] = missing
    report["notes"] = notes
    if missing:
        raise PlacementError(
            "the deliverable does not contain %d of the %d specified "
            "obstacles (missing: %s; %d non-robot bodies were measured in "
            "it). The drawn layout cannot be placed into a world that has "
            "nowhere to put it, and grading it as delivered would score the "
            "PUBLISHED layout -- which is the memorising agent's best case."
            % (len(missing), len(published), ", ".join(missing), len(bodies)),
            report)
    clash = _reject_nested(pairs)
    if clash:
        raise PlacementError(
            "two of the matched obstacles are the same body: %s" % clash,
            report)

    drawn = {o["name"]: o for o in layout}
    moves, placed = [], []
    for want in published:
        body, channel = pairs[want["name"]]
        target = drawn[want["name"]]["position"]
        delta = (float(target[0]) - body.centre[0],
                 float(target[1]) - body.centre[1], 0.0)
        moves.append((body, delta))
        placed.append({"name": want["name"], "channel": channel,
                       "body_id": body.body_id,
                       "from": [round(body.centre[0], 4),
                                round(body.centre[1], 4)],
                       "to": [round(float(target[0]), 4),
                              round(float(target[1]), 4)],
                       "moved_m": round(math.hypot(delta[0], delta[1]), 4)})
    new_text, applied = module.move_bodies(text, moves)
    report["obstacles"] = placed
    report["edits"] = applied

    report["verification"] = _verify(new_text, layout, fmt)
    return new_text, report


def _verify(text, layout, fmt):
    """Re-measure the PLACED artifact and prove it is the drawn layout.

    Everything below is read back out of the rewritten text with the grader's
    own matcher, at the grader's own tolerance. A placement that edited the
    wrong node, or moved a body its parent then moved back, fails here rather
    than at grade time -- where it would read as an agent that built the wrong
    world.
    """
    module = _ARMS_BY_FMT[fmt]
    bodies = module.scan_bodies(text)
    found, missing = r1_core.match_spec_obstacles(bodies, layout)
    worst, worst_name = 0.0, None
    drawn = {o["name"]: o for o in layout}
    stream = iter(found)
    for want in layout:
        if want["name"] in missing:
            continue
        b = next(stream)
        d = math.hypot(b.centre[0] - float(drawn[want["name"]]["position"][0]),
                       b.centre[1] - float(drawn[want["name"]]["position"][1]))
        if d > worst:
            worst, worst_name = d, want["name"]
    blocked = r1_core.segment_blocked_by(found)
    blocked_m = r1_core.segment_blocked_length(found)
    out = {"matched": len(found), "missing": missing,
           "max_centre_error_m": round(worst, 6),
           "worst_obstacle": worst_name,
           "straight_line_blocked_by": blocked,
           "straight_line_blocked_m": round(blocked_m, 4)}
    if missing or len(found) != r1_core.N_OBSTACLES:
        raise PlacementError(
            "the placed artifact does not measure as the drawn layout: %d of "
            "%d obstacles matched (missing: %s). The rewrite did not land."
            % (len(found), r1_core.N_OBSTACLES, ", ".join(missing) or "-"),
            {"verification": out})
    if worst > VERIFY_TOL_M:
        raise PlacementError(
            "obstacle %s re-measures %.6f m from where it was placed (bound "
            "%.6f m): the rewrite moved something other than what was asked"
            % (worst_name, worst, VERIFY_TOL_M), {"verification": out})
    if blocked_m < r1_core.PLACEMENT_MIN_BLOCK_M:
        raise PlacementError(
            "the straight start->goal line is blocked over only %.3f m of the "
            "placed world (the layout was accepted at >= %.3f m): the whole "
            "sensing argument rests on that line being blocked"
            % (blocked_m, r1_core.PLACEMENT_MIN_BLOCK_M),
            {"verification": out})
    return out


def place_artifact(artifact, layout, *, dest=None):
    """Place ``layout`` into a deliverable ON DISK. Returns the report.

    ``dest`` defaults to the artifact itself -- the collected copy under the
    run directory, never the agent's own workspace file.
    """
    artifact = Path(artifact)
    fmt = FORMAT_BY_SUFFIX.get(artifact.suffix.lower())
    if fmt is None:
        raise PlacementError(
            "no grade-time placement is implemented for a %r deliverable; "
            "R1 is graded on a .wbt world or an MJCF .xml model"
            % (artifact.suffix,), {"artifact": str(artifact)})
    try:
        text = artifact.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise PlacementError("the deliverable could not be read: %s" % exc,
                             {"artifact": str(artifact)}) from exc
    new_text, report = place_text(text, layout, fmt)
    report["artifact"] = str(dest or artifact)
    report["source"] = str(artifact)
    Path(dest or artifact).write_text(new_text, encoding="utf-8")
    return report


def declare(directories, seed, layout):
    """Write the ``r1_graded_layout.json`` handshake into each directory.

    One file, one name (``r1_core.LAYOUT_SIDECAR``), written by the step that
    did the placing and read by ``r1_core.resolve_graded_layout``. Several
    directories because each arm's grader runs with a different idea of where
    "the run" is.
    """
    out = []
    for d in directories:
        if not d:
            continue
        out.append(str(r1_core.write_graded_layout(d, seed, layout)))
    return out


def place_and_declare(artifact, *, seed, declare_dirs=(), layout=None,
                      dest=None):
    """Draw, place, verify, declare -- the whole step, for one cell.

    Raises :class:`PlacementError` if any part of it fails. There is no
    partial success: an artifact that was edited but could not be verified,
    or a layout that was placed but not declared, would be graded against the
    wrong scene.
    """
    if layout is None:
        layout, draw_report = draw(seed)
    else:
        layout, draw_report = list(layout), {"seed": seed,
                                             "mechanism": MECHANISM,
                                             "attempts": None}
    report = place_artifact(artifact, layout, dest=dest)
    report["seed"] = seed
    report["draw"] = {k: draw_report.get(k) for k in ("attempts", "mechanism")}
    report["legality"] = (draw_report.get("legality") or {}).get("legal")
    report["layout"] = [{"name": o["name"],
                         "position": [round(float(c), 4)
                                      for c in o["position"]],
                         "size": [float(c) for c in o["size"]]}
                        for o in layout]
    report["sidecars"] = declare(declare_dirs, seed, layout)
    if declare_dirs and not report["sidecars"]:
        raise PlacementError(
            "the graded layout was placed but never declared: the grader "
            "would fall back to the published layout", report)
    return report


def read_sidecar(directory):
    """The declared layout in a directory, or ``None`` -- for tests/forensics."""
    p = Path(directory) / r1_core.LAYOUT_SIDECAR
    if not p.is_file():
        return None
    return json.loads(p.read_text(encoding="utf-8"))
