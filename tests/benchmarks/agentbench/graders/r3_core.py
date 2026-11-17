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

"""R3 ``pick_and_place`` -- the **sim-neutral core** of the robotics tier.

**Why this is the task in the tier that matters most.** A grasp that has to
HOLD is the one place a contact model is genuinely put under load: friction,
normal force and solver stability sustained over seconds, not sampled at an
instant. Driving to a goal exercises wheels and a sensor loop; carrying a part
exercises the contact solver itself. It is also standard robotics work that
every simulator claims to do -- and it is the task where a weak contact model
shows up as a **dropped cube** rather than as a number in a table. On this
engine the gap between the defaults and what the job needs is wide enough to
have its own guide (``docs/guide/friction-grasp.md``), and R3 is the
measurement that gap either survives or does not.

    R3.1  the run is clean            exit 0, no error lines, driver completed
    R3.2  the benchmark scene is intact
                                      the table's top surface and the bin's
                                      whole box, MEASURED at t=0, match the
                                      shipped spec within SCENE_TOL_M
    R3.3  there is a cube and an arm   exactly one tracked body with the cube's
                                      t=0 geometry, still dynamic, plus >= 1
                                      other robot-class body with >= 3 joints
    R3.4  the cube started at rest on the table
                                      first sample within START_TOL_M of the
                                      specified xy and within TABLE_REST_TOL_M
                                      of table_top + half the cube
    R3.5  it was lifted clear and carried
                                      >= AIRBORNE_MIN_S contiguous above the
                                      bin rim AND the table by a clearance,
                                      moving >= CARRY_MIN_XY_M horizontally,
                                      ending nearer the bin than it started
    R3.6  it was released and settled  >= RELEASE_DESCENT_M of descent after
                                      the carry, then z steady within
                                      REST_BAND_M over the last HOLD_S
    R3.7  it is in the bin, and stayed  every sample of the last HOLD_S inside
                                      the measured bin volume
    R3.8  it was never teleported      max speed <= MAX_SPEED_MPS and no single
                                      sample step over MAX_STEP_DISPLACEMENT_M

**Where the grasp proof comes from, and why it is not a device count.**
R3.5 + R3.6 + R3.7 together ARE the evidence that something held the cube. It
leaves the table, rises clear of the bin rim -- so it is touching neither the
table nor the bin -- travels horizontally to the bin, then descends under
gravity and stays there. Nothing but a held part does that. A gripper-device
check, a "fingers closed" flag or a contact count would each have been spelled
differently in every simulator, would each have been satisfiable by a gripper
that closes on nothing, and would have made this core non-neutral. The cube is
a body, so its whole story is measurable without any joint, gripper or device
API on either arm.

**What this core deliberately does NOT claim.** It cannot see the end
effector: both arms' recorders sample a Robot NODE's world position, which for
a fixed-base arm is its stationary base, so "retract the end effector" is not
in the evidence and is not asserted. It cannot separate "released" from
"lowered and still gripped" -- a cube at rest on the bin floor reads the same
either way. And R3.8 bounds a rate, so it catches a discrete teleport but not
a continuous supervisor-driven animation that respects the bound. All three
are recorded in ``meta.json`` (``not_measured``, ``known_hole``) rather than
quietly implied to be covered.

Every threshold is READ from ``tasks/R3_pick_and_place/meta.json``'s
``constants`` block and every scene dimension from that task's
``benchmark_assets/scene.json``: no number in this task is written down twice,
so the contract and the code cannot drift apart. Everything here is metres and
seconds. Nothing in this module knows what a simulator is.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier, Verdict)

TASK = "R3_pick_and_place"

TASK_DIR = Path(__file__).resolve().parents[1] / "tasks" / TASK


def _meta():
    """The task contract. Read, never duplicated."""
    return json.loads((TASK_DIR / "meta.json").read_text(encoding="utf-8"))


def scene_spec():
    """The frozen scene geometry, read from the task's own shipped asset.

    Read rather than duplicated, for the reason R1 gives about its obstacle
    list: the agent and the grader must be looking at the same numbers, and a
    second copy in code is a second source of truth waiting to drift. The
    upstream-Webots arm ships a byte-identical copy; ``test_r3_core`` pins
    both against the two worlds' text.
    """
    p = TASK_DIR / "initial" / "benchmark_assets" / "scene.json"
    return json.loads(p.read_text(encoding="utf-8"))


# --- thresholds: read from meta.json ----------------------------------------

_C = _meta()["constants"]

SCENE_TOL_M = float(_C["SCENE_TOL_M"])
CUBE_SIZE_TOL_M = float(_C["CUBE_SIZE_TOL_M"])
START_TOL_M = float(_C["START_TOL_M"])
TABLE_REST_TOL_M = float(_C["TABLE_REST_TOL_M"])
AIRBORNE_CLEARANCE_M = float(_C["AIRBORNE_CLEARANCE_M"])
AIRBORNE_MIN_S = float(_C["AIRBORNE_MIN_S"])
CARRY_MIN_XY_M = float(_C["CARRY_MIN_XY_M"])
RELEASE_DESCENT_M = float(_C["RELEASE_DESCENT_M"])
HOLD_S = float(_C["HOLD_S"])
HOLD_MIN_SAMPLES = int(_C["HOLD_MIN_SAMPLES"])
REST_BAND_M = float(_C["REST_BAND_M"])
MAX_SPEED_MPS = float(_C["MAX_SPEED_MPS"])
MAX_STEP_DISPLACEMENT_M = float(_C["MAX_STEP_DISPLACEMENT_M"])
MIN_ARM_JOINTS = int(_C["MIN_ARM_JOINTS"])

# --- scene geometry: read from benchmark_assets/scene.json ------------------

_S = scene_spec()

TABLE_TOP_Z_M = float(_S["table"]["top_z"])
TABLE_AABB = (tuple(float(v) for v in _S["table"]["aabb_min"]),
              tuple(float(v) for v in _S["table"]["aabb_max"]))

CUBE_SIZE_M = float(_S["cube"]["size"])
CUBE_START_XYZ = tuple(float(v) for v in _S["cube"]["position"])
#: Where a cube of this size sits when it is resting on that table.
CUBE_REST_Z_M = TABLE_TOP_Z_M + CUBE_SIZE_M / 2.0

BIN_CENTER_XYZ = tuple(float(v) for v in _S["bin"]["position"])
BIN_AABB = (tuple(float(v) for v in _S["bin"]["aabb_min"]),
            tuple(float(v) for v in _S["bin"]["aabb_max"]))
BIN_INNER_HALF_XY_M = float(_S["bin"]["inner_half_extent_xy"])
BIN_INNER_FLOOR_Z_M = float(_S["bin"]["inner_floor_z"])
BIN_RIM_Z_M = float(_S["bin"]["rim_z"])

#: The centre of a cube whose xy is this far from the bin's centre is still
#: WHOLLY between the bin walls. Containment, not "roughly over the bin".
BIN_INSET_M = BIN_INNER_HALF_XY_M - CUBE_SIZE_M / 2.0

#: Above this height the cube's lowest face clears the bin rim -- and so, by a
#: much larger margin, the table -- by AIRBORNE_CLEARANCE_M. It is touching
#: nothing in the scene, so something is holding it.
Z_AIRBORNE_M = BIN_RIM_Z_M + CUBE_SIZE_M / 2.0 + AIRBORNE_CLEARANCE_M

TABLE_NAME = str(_S["table"]["name"])
BIN_NAME = str(_S["bin"]["name"])

_ALL = [("R3.1", "the run is clean"),
        ("R3.2", "the benchmark scene is intact"),
        ("R3.3", "there is a graspable cube and a distinct arm"),
        ("R3.4", "the cube started at rest on the table"),
        ("R3.5", "the cube was lifted clear and carried toward the bin"),
        ("R3.6", "the cube was released and settled"),
        ("R3.7", "the cube is inside the bin and stayed there"),
        ("R3.8", "the cube was never teleported")]

_BASIS = {"R3.1": MIXED, "R3.2": CORE_PHYSICAL, "R3.3": MIXED,
          "R3.4": CORE_PHYSICAL, "R3.5": CORE_PHYSICAL,
          "R3.6": CORE_PHYSICAL, "R3.7": CORE_PHYSICAL,
          "R3.8": CORE_PHYSICAL}


# --- pure helpers (unit-testable without a simulator) -----------------------


def _f(v):
    return float(v)


def in_bin(x, y, z):
    """Is a cube centred here WHOLLY inside the bin's open volume?

    xy within the inner footprint inset by half the cube (so no part of it
    overhangs a wall), and z between the inner floor and the rim (so a cube
    perched ON the rim, centre 0.025 m above it, is excluded).
    """
    return (abs(_f(x) - BIN_CENTER_XYZ[0]) <= BIN_INSET_M
            and abs(_f(y) - BIN_CENTER_XYZ[1]) <= BIN_INSET_M
            and BIN_INNER_FLOOR_Z_M <= _f(z) <= BIN_RIM_Z_M)


def speed_profile(t, xyz):
    """``(max_speed_mps, max_step_m)`` over a pose series.

    The speed is the teleport witness and it is deliberately dt-INDEPENDENT:
    R1's per-sample distance bound assumes the recorder's timestep, and an
    agent that authors ``basicTimeStep 32`` (or 8, which the grasp recipe
    recommends) would be graded against a different implied speed. The step
    distance is kept as a second clause so a coarse recording cannot smuggle a
    jump through under the speed bound.
    """
    max_speed, max_step = 0.0, 0.0
    for i in range(1, len(xyz)):
        d = math.sqrt(sum((_f(xyz[i][k]) - _f(xyz[i - 1][k])) ** 2
                          for k in range(3)))
        max_step = max(max_step, d)
        dt = _f(t[i]) - _f(t[i - 1])
        if dt > 0:
            max_speed = max(max_speed, d / dt)
    return max_speed, max_step


def longest_true_run(mask, t):
    """``(i0, i1, seconds)`` of the longest contiguous True run in ``mask``.

    Contiguous rather than cumulative on purpose: a cube that flickers above
    the threshold on bounces accumulates time without ever having been
    carried. ``(None, None, 0.0)`` when the mask is all False.
    """
    best = (None, None, 0.0)
    i = 0
    n = len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        span = _f(t[j]) - _f(t[i])
        if best[0] is None or span > best[2]:
            best = (i, j, span)
        i = j + 1
    return best


def aabb_error(body, lo, hi):
    """Largest per-corner coordinate error between a body's AABB and a spec.

    ``None`` when the body has no world-space AABB -- which is an absent
    measurement, never a pass.
    """
    if body is None or not body.has_aabb:
        return None
    blo, bhi = body.aabb
    err = 0.0
    for k in range(3):
        err = max(err, abs(_f(blo[k]) - _f(lo[k])),
                  abs(_f(bhi[k]) - _f(hi[k])))
    return err


def find_named(bodies, name):
    """The non-robot body the fixture ships under this name, or None.

    Name-keyed, unlike R1's obstacle match, and the reason is the instrument
    rather than a preference: both arms' non-robot bounds scans are keyed by
    name (OmniSim's recorder is asked for ``--solids=table,bin``; the upstream
    prober writes ``bodies[name]``), so there is no geometric channel to match
    on. ``meta.json`` says so out loud, and R3.2's failure text says which name
    was missing rather than blaming the physics.
    """
    for b in bodies:
        if (b.name or "") == name and not b.robot_class:
            return b
    for b in bodies:                       # a fixture body the adapter could
        if (b.name or "") == name:         # not classify still counts
            return b
    return None


def find_cube(bodies):
    """``(index, body, how, candidates)`` -- the cube, found by GEOMETRY.

    Never by name and never by DEF: the same rule R1 learned the hard way when
    an agent built the specified scene and called the boxes something else. A
    body is the cube when its frozen t=0 AABB is a CUBE_SIZE_M cube (within
    CUBE_SIZE_TOL_M on every axis) centred within START_TOL_M of the specified
    start point.

    ``how`` records which channel answered, because the two are not equally
    strong: ``"aabb"`` verified the size, ``"position"`` is the fallback for an
    adapter that reports no world-space AABB and verified only the start point.
    The caller marks the size clause vacuous in that case rather than pretending
    it was checked.
    """
    robots = [b for b in bodies if b.robot_class]
    pool = robots or list(bodies)
    hits = []
    for i, b in enumerate(pool):
        if not b.has_aabb:
            continue
        lo, hi = b.aabb
        size = [_f(hi[k]) - _f(lo[k]) for k in range(3)]
        centre = [(_f(hi[k]) + _f(lo[k])) / 2.0 for k in range(3)]
        if any(abs(s - CUBE_SIZE_M) > CUBE_SIZE_TOL_M for s in size):
            continue
        if any(abs(centre[k] - CUBE_START_XYZ[k]) > START_TOL_M
               for k in range(3)):
            continue
        hits.append((i, b))
    if len(hits) == 1:
        return hits[0][0], hits[0][1], "aabb", len(hits)
    if hits:
        return None, None, "ambiguous", len(hits)

    # No AABBs anywhere: fall back to the t=0 point, and say so.
    if not any(b.has_aabb for b in pool):
        near = [(i, b) for i, b in enumerate(pool)
                if b.position is not None
                and all(abs(_f(b.position[k]) - CUBE_START_XYZ[k])
                        <= START_TOL_M for k in range(3))]
        if len(near) == 1:
            return near[0][0], near[0][1], "position", 1
        if near:
            return None, None, "ambiguous", len(near)
    return None, None, "none", 0


def same_body(a, b):
    """Do two :class:`Body` records name the same body?

    The cube and the arm are read out of two different inventories (the frozen
    t=0 scan and the roster), so "the arm is not the cube" has to be asked
    across them. Id first, then name -- the two channels the arms respectively
    key on.
    """
    if a is None or b is None:
        return False
    if a.body_id and b.body_id and a.body_id == b.body_id:
        return True
    return bool(a.name and b.name and a.name == b.name)


def trajectory_row(traj, body):
    """``(row, how)`` -- which row of ``traj.xyz`` is this body.

    Two channels, tried in order, because the arms disagree about what
    ``Trajectory.body_ids`` holds and neither is wrong. On OmniSim the ids are
    the recorder's own body ids (a DEF, else ``#<node id>``) and align with the
    roster. On upstream Webots the roster carries STATIC scenery that never
    appears in the pose series, so the adapter's name-alignment declines and
    the ids fall back to the recorder's body NAMES -- with a published note
    that a per-body index is not valid across the two inventories. Looking up
    by id and then by name resolves the body on both arms without ever
    guessing a position.
    """
    for key, how in ((body.body_id, "body_id"), (body.name, "name")):
        if key:
            i = traj.index_of(key)
            if i is not None:
                return i, how
    return None, "unresolved"


# --- the grade ---------------------------------------------------------------


def grade(bundle, *, self_verified=False):
    v = Verdict(TASK, self_verified=self_verified)
    for note in bundle.notes:
        v.note(note)
    L = bundle.lbl

    if bundle.artifact is None:
        v.progress = NO_ARTIFACT
        for aid, what in _ALL:
            v.add(aid, False, what, basis=_BASIS[aid],
                  detail=L("artifact_missing_detail", "no artifact"))
        return v.finish()
    v.artifacts["world"] = str(bundle.artifact)

    traj = bundle.trajectory
    if traj is None or traj.xyz is None:
        v.progress = ARTIFACT_INVALID
        why = (bundle.motion_error or (traj.error if traj else None)
               or "no samples")
        for aid, what in _ALL:
            v.add(aid, False, what, detail=why, basis=_BASIS[aid])
        v.measurements.update(bundle.adapter_measurements.get("motion") or {})
        return v.finish()

    v.progress = ARTIFACT_RUNS
    v.measurements.update(bundle.adapter_measurements.get("motion") or {})

    attrib = bundle.attribution
    v.attribution = attrib.as_dict() if attrib else {}
    if attrib is None:
        v.outcome = INVALID
        v.note(L("attribution_missing_note",
                 "no physics-engine attribution (SPEC A1.10 rule, applied "
                 "suite-wide): the adapter could not name the engine that "
                 "drove this run from any channel it has."))
        return v

    # --- R3.1 the run is clean --------------------------------------------
    proc = bundle.process
    errs = list(proc.error_lines) if proc else []
    complete = bool(proc and proc.driver_completed)
    v.add("R3.1", bool(proc and proc.clean and complete), "the run is clean",
          measured={L("exit_code", "exit code"): proc.exit_code if proc
                    else None,
                    L("error_lines", "error lines"): len(errs),
                    L("driver_completed", "driver reached a clean quit"):
                        complete,
                    L("timed_out", "timed out"):
                        bool(proc.timed_out) if proc else None},
          threshold={L("exit_code", "exit code"): 0,
                     L("error_lines", "error lines"): 0},
          detail="; ".join(errs[:3]), basis=MIXED,
          attested=([("error stream + exit code: %s"
                      % (proc.log_source or "adapter-supplied"))]
                    if proc else []),
          falsifiers=[Falsifier(
              "clean run",
              "a non-zero exit code, an error-class line, a timeout, or a "
              "driver that never reached its target simulated time",
              "the adapter captured the exit code AND the simulator's own "
              "error stream",
              bool(proc is not None and proc.exit_code is not None
                   and proc.log_available is not False))])

    # --- R3.2 the benchmark scene is intact -------------------------------
    #
    # MEASURED, not trusted. Every one of R3.4-R3.7 is stated against the
    # table surface and the bin volume, so an agent that quietly raised the
    # table, sank the bin or widened its mouth would be graded against a scene
    # of its own choosing -- and "the cube is in the bin" would stop meaning
    # anything. The same discipline R1.3 applies to its obstacles.
    t0 = list(bundle.t0.bodies)
    table = find_named(t0, TABLE_NAME)
    bin_body = find_named(t0, BIN_NAME)
    table_err = aabb_error(table, *TABLE_AABB)
    bin_err = aabb_error(bin_body, *BIN_AABB)
    table_top = table.top_z if (table is not None and table.has_aabb) else None
    scene_ok = (table_err is not None and table_err <= SCENE_TOL_M
                and bin_err is not None and bin_err <= SCENE_TOL_M)
    missing = [n for n, b in ((TABLE_NAME, table), (BIN_NAME, bin_body))
               if b is None]
    v.add("R3.2", scene_ok, "the benchmark scene is intact",
          measured={L("table_top_z", "measured table top z (m)"):
                    None if table_top is None else round(_f(table_top), 4),
                    L("table_aabb_error",
                      "table AABB error vs the shipped spec (m)"):
                        None if table_err is None else round(table_err, 4),
                    L("bin_aabb_error",
                      "bin AABB error vs the shipped spec (m)"):
                        None if bin_err is None else round(bin_err, 4),
                    L("scene_missing", "fixture bodies not found"): missing},
          threshold={L("table_aabb_error", "table AABB error (m)"):
                     "<= %.3f" % SCENE_TOL_M,
                     L("bin_aabb_error", "bin AABB error (m)"):
                         "<= %.3f" % SCENE_TOL_M},
          detail=("the table surface and the bin volume every later assertion "
                  "is stated against are measured from the frozen t=0 scan, "
                  "never assumed from the file"
                  + ("; both are located by the name the fixture ships, "
                     "because both arms' non-robot bounds scans are name-keyed"
                     if missing else "")),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "scene measured",
              "a table raised or lowered, or a bin moved, widened or "
              "deepened, so that 'inside the bin' means something else",
              "world-space AABBs for BOTH fixture bodies at a frozen t=0",
              table_err is not None and bin_err is not None,
              detail=bundle.t0.source or "no t=0 geometry")])

    # --- R3.3 a graspable cube, and an arm --------------------------------
    idx, cube, how, n_cand = find_cube(t0)
    arms = [b for b in bundle.roster.bodies
            if b.robot_class and (b.n_joints or 0) >= MIN_ARM_JOINTS
            and not same_body(b, cube)]
    cube_dynamic = bool(cube is not None and cube.dynamic is True)
    v.add("R3.3", bool(cube is not None and cube_dynamic and arms),
          "there is a graspable cube and a distinct arm",
          measured={L("cube_found", "cube matched at t=0 by"): how,
                    L("cube_candidates", "bodies matching the cube spec"):
                        n_cand,
                    L("cube_dynamic", "the cube still has a mass model"):
                        None if cube is None else cube.dynamic,
                    L("arm_bodies",
                      "other robot-class bodies with >= %d joints"
                      % MIN_ARM_JOINTS): len(arms),
                    L("arm_names", "their names"):
                        [b.name for b in arms][:4]},
          threshold={L("cube_candidates", "bodies matching the cube spec"): 1,
                     L("cube_dynamic", "the cube still has a mass model"): True,
                     L("arm_bodies", "robot-class bodies with enough joints"):
                         ">= 1"},
          detail=("the cube is found by its t=0 geometry -- a %.3f m cube "
                  "within %.2f m of the specified start -- and never by name "
                  "or DEF; 'dynamic' is the anti-cheat clause a body nailed in "
                  "place would fail" % (CUBE_SIZE_M, START_TOL_M)),
          basis=MIXED,
          attested=([("cube identity: %s" % (cube.identity_evidence
                                             or bundle.t0.source))]
                    if cube is not None else []),
          falsifiers=[
              Falsifier("cube size verified",
                        "a body at the right place that is not the specified "
                        "cube",
                        "a world-space AABB for the candidate body at t=0",
                        how == "aabb",
                        detail=("matched on the t=0 point only: this adapter "
                                "reports no world-space AABB, so the size was "
                                "NOT checked" if how == "position" else "")),
              Falsifier("dynamics attached",
                        "the agent 'held' the cube by removing its dynamics",
                        "the adapter answers 'is this body dynamic' either "
                        "way, rather than returning None",
                        cube is not None and cube.dynamic is not None)])

    if cube is None:
        for aid in ("R3.4", "R3.5", "R3.6", "R3.7", "R3.8"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  detail=("no body matching the specified cube was found at "
                          "t=0 (%s), so nothing can be measured; the cube is "
                          "shipped in the initial world and must not be "
                          "moved, resized or removed" % how))
        return v.finish()

    row, resolved_by = trajectory_row(traj, cube)
    if row is None:
        for aid in ("R3.4", "R3.5", "R3.6", "R3.7", "R3.8"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  detail=("the cube was found in the t=0 inventory but has no "
                          "row in the pose series (ids: %s)"
                          % (traj.body_ids[:6],)))
        return v.finish()

    xyz = traj.xyz[row]
    if traj.t is None or len(traj.t) != len(xyz):
        # Every remaining threshold is in SECONDS. Substituting sample indices
        # would keep grading and quietly mean something else, which is worse
        # than declining: unmeasured is never a pass.
        for aid in ("R3.4", "R3.5", "R3.6", "R3.7", "R3.8"):
            v.add(aid, False, dict(_ALL)[aid], basis=_BASIS[aid],
                  detail=("the pose series carries no usable time base "
                          "(%s samples, %s times), and every threshold from "
                          "here on is in seconds"
                          % (len(xyz),
                             "none" if traj.t is None else len(traj.t))))
        return v.finish()
    t = traj.t
    x0, y0, z0 = _f(xyz[0][0]), _f(xyz[0][1]), _f(xyz[0][2])

    # --- R3.4 it started at rest on the table ------------------------------
    start_xy_err = math.hypot(x0 - CUBE_START_XYZ[0], y0 - CUBE_START_XYZ[1])
    start_z_err = abs(z0 - CUBE_REST_Z_M)
    v.add("R3.4", start_xy_err <= START_TOL_M
          and start_z_err <= TABLE_REST_TOL_M,
          "the cube started at rest on the table",
          measured={L("start_xy", "first sample xy (m)"):
                    [round(x0, 4), round(y0, 4)],
                    L("start_xy_error", "xy error vs the specified start (m)"):
                        round(start_xy_err, 4),
                    L("start_z", "first sample z (m)"): round(z0, 4),
                    L("start_z_error",
                      "z error vs table top + half the cube (m)"):
                        round(start_z_err, 4),
                    L("resolved_by", "pose row resolved by"): resolved_by},
          threshold={L("start_xy_error", "xy error (m)"):
                     "<= %.3f" % START_TOL_M,
                     L("start_z_error", "z error (m)"):
                         "<= %.3f" % TABLE_REST_TOL_M},
          detail=("expected rest z is %.4f m = the MEASURED table top plus "
                  "half the cube; this is the clause a cube spawned already "
                  "inside the bin fails" % CUBE_REST_Z_M),
          basis=CORE_PHYSICAL)

    # --- R3.5 it was lifted clear, and carried -----------------------------
    #
    # Z_AIRBORNE_M puts the cube's lowest face above the bin RIM (and so far
    # above the table), which is what makes this clause mean "held". The
    # duration floor is what makes it mean "carried" rather than "knocked":
    # staying above the threshold for AIRBORNE_MIN_S ballistically would need
    # an apex ~0.3 m higher still, i.e. a throw, not a shove.
    mask = [_f(p[2]) >= Z_AIRBORNE_M for p in xyz]
    i0, i1, air_s = longest_true_run(mask, t)
    if i0 is None:
        carry_xy = 0.0
        closed = None
    else:
        carry_xy = math.hypot(_f(xyz[i1][0]) - _f(xyz[i0][0]),
                              _f(xyz[i1][1]) - _f(xyz[i0][1]))
        d_start = math.hypot(_f(xyz[i0][0]) - BIN_CENTER_XYZ[0],
                             _f(xyz[i0][1]) - BIN_CENTER_XYZ[1])
        d_end = math.hypot(_f(xyz[i1][0]) - BIN_CENTER_XYZ[0],
                           _f(xyz[i1][1]) - BIN_CENTER_XYZ[1])
        closed = d_end < d_start
    v.add("R3.5", bool(air_s >= AIRBORNE_MIN_S
                       and carry_xy >= CARRY_MIN_XY_M and closed),
          "the cube was lifted clear and carried toward the bin",
          measured={L("airborne_s",
                      "longest contiguous time clear of everything (s)"):
                    round(air_s, 4),
                    L("airborne_threshold_z",
                      "the height that counts as clear (m)"):
                        round(Z_AIRBORNE_M, 4),
                    L("max_z", "highest z reached (m)"):
                        round(max(_f(p[2]) for p in xyz), 4),
                    L("carry_xy", "horizontal travel while clear (m)"):
                        round(carry_xy, 4),
                    L("closed_on_bin", "ended nearer the bin than it started"):
                        closed},
          threshold={L("airborne_s", "time clear (s)"):
                     ">= %.2f" % AIRBORNE_MIN_S,
                     L("carry_xy", "horizontal travel while clear (m)"):
                         ">= %.2f" % CARRY_MIN_XY_M,
                     L("closed_on_bin", "ended nearer the bin"): True},
          detail=("above %.4f m the cube's lowest face clears the bin rim by "
                  "%.2f m and the table by %.2f m, so it is touching nothing "
                  "in the scene; this is the clause a cube SLID along the "
                  "table into the bin fails, and it is the whole grasp proof"
                  % (Z_AIRBORNE_M, AIRBORNE_CLEARANCE_M,
                     Z_AIRBORNE_M - CUBE_SIZE_M / 2.0 - TABLE_TOP_Z_M)),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "carried through free space",
              "a cube that never left the table, or one pushed/knocked in",
              "a pose series long enough to contain a %.2f s window"
              % AIRBORNE_MIN_S,
              bool((traj.recorded_s or 0.0) > AIRBORNE_MIN_S
                   and traj.n_samples >= 2),
              detail="recorded %s s" % (traj.recorded_s,))])

    # --- R3.6 it was released, and settled ---------------------------------
    if i1 is None:
        descent = 0.0
    else:
        descent = _f(xyz[i1][2]) - min(_f(p[2]) for p in xyz[i1:])
    hold = [k for k in range(len(xyz))
            if _f(t[k]) >= _f(t[-1]) - HOLD_S]
    hold_z = [_f(xyz[k][2]) for k in hold]
    rest_band = (max(hold_z) - min(hold_z)) if hold_z else float("nan")
    v.add("R3.6", bool(descent >= RELEASE_DESCENT_M
                       and hold_z and rest_band <= REST_BAND_M),
          "the cube was released and settled",
          measured={L("descent_after_carry",
                      "descent from the end of the carry (m)"):
                    round(descent, 4),
                    L("rest_band", "z spread over the last %.1f s (m)"
                      % HOLD_S): round(rest_band, 5)
                    if hold_z else None,
                    L("final_z", "final z (m)"): round(_f(xyz[-1][2]), 4)},
          threshold={L("descent_after_carry", "descent (m)"):
                     ">= %.2f" % RELEASE_DESCENT_M,
                     L("rest_band", "z spread (m)"): "<= %.3f" % REST_BAND_M},
          detail=("gravity acting on a body nobody is holding any more is the "
                  "witness here; a cube still being carried at the end of the "
                  "run has not descended, and one still bouncing has not "
                  "settled"),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "descended then rested",
              "a cube parked in mid-air, or one still moving at the end",
              "a recorded window longer than the %.1f s rest test" % HOLD_S,
              bool((traj.recorded_s or 0.0) > HOLD_S),
              detail="recorded %s s" % (traj.recorded_s,))])

    # --- R3.7 it is in the bin, and stayed ---------------------------------
    inside = [in_bin(*(xyz[k][:3])) for k in hold]
    hold_span = (_f(t[hold[-1]]) - _f(t[hold[0]])) if hold else 0.0
    n_out = sum(1 for ok in inside if not ok)
    v.add("R3.7", bool(inside and n_out == 0
                       and len(hold) >= HOLD_MIN_SAMPLES
                       and hold_span >= HOLD_S - 1e-9),
          "the cube is inside the bin and stayed there",
          measured={L("hold_samples",
                      "samples in the last %.1f s" % HOLD_S): len(hold),
                    L("hold_span", "the window actually covered (s)"):
                        round(hold_span, 4),
                    L("outside_samples", "of those, outside the bin"): n_out,
                    L("final_xyz", "final xyz (m)"):
                        [round(_f(xyz[-1][k]), 4) for k in range(3)],
                    L("bin_inset",
                      "how far the centre may sit from the bin centre (m)"):
                        round(BIN_INSET_M, 4)},
          threshold={L("outside_samples", "samples outside the bin"): 0,
                     L("hold_span", "window covered (s)"): ">= %.1f" % HOLD_S,
                     L("hold_samples", "samples"):
                         ">= %d" % HOLD_MIN_SAMPLES},
          detail=("'inside' means the whole cube is between the bin walls and "
                  "its centre is below the rim. The bin box it is computed "
                  "from is the shipped one, and R3.2 is what makes that "
                  "legitimate: it pins the MEASURED bin to within %.3f m of "
                  "it, so a widened or moved bin fails there rather than "
                  "quietly redefining 'inside' here. The sample count stops a "
                  "coarse recording from satisfying a 2-second claim with two "
                  "points" % SCENE_TOL_M),
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "stayed put",
              "a cube that bounced out, was dragged out, or was never in",
              ">= %d samples inside the last %.1f s of the recording"
              % (HOLD_MIN_SAMPLES, HOLD_S),
              len(hold) >= HOLD_MIN_SAMPLES)])

    # --- R3.8 it was never teleported --------------------------------------
    max_speed, max_step = speed_profile(t, xyz)
    v.add("R3.8", max_speed <= MAX_SPEED_MPS
          and max_step <= MAX_STEP_DISPLACEMENT_M,
          "the cube was never teleported",
          measured={L("max_speed", "fastest the cube ever moved (m/s)"):
                    round(max_speed, 4),
                    L("max_step", "largest single-sample step (m)"):
                        round(max_step, 4)},
          threshold={L("max_speed", "speed (m/s)"): "<= %.1f" % MAX_SPEED_MPS,
                     L("max_step", "single-sample step (m)"):
                         "<= %.2f" % MAX_STEP_DISPLACEMENT_M},
          detail=("the start-to-bin jump is %.3f m, which at any plausible "
                  "timestep is far over both bounds; a rate bound cannot see "
                  "a continuous supervisor-driven animation, and meta.json "
                  "says so rather than implying otherwise"
                  % math.hypot(CUBE_START_XYZ[0] - BIN_CENTER_XYZ[0],
                               CUBE_START_XYZ[1] - BIN_CENTER_XYZ[1])),
          basis=CORE_PHYSICAL)

    return v.finish(GRADED_PASS)
