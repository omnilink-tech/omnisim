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

"""A1 ``husky_swarm_10`` -- the **sim-neutral core** of the ten assertions.

This module reads an :class:`~agentbench.graders.evidence.EvidenceBundle` and
nothing else. It does not know what a ``URDFRobot`` is, has never heard of
``husky.urdf``, and cannot open a ``.newton.json`` sidecar. Everything it
asserts is a count, a distinctness, a metre or a second.

Constants are the SPEC's, quoted here so a drift is visible in a diff::

    N_ROBOTS = 10   SETTLE_S = 1.0 s   N = 30.0 s   X = 2.0 m
    PATH_MIN = 4.0 m   |dz| <= 0.30 m   z > -0.10 m   R < 0.70

**How the three OmniSim-specific assertions were re-expressed** (intent and
thresholds preserved exactly; SPEC 2.3's numbers are untouched):

``A1.1 "exactly ten Huskies"``
    was: ten nodes of type ``URDFRobot``/``Robot`` with >= 4 joints and a
    ``husky.urdf`` url in the world text. Now: ten distinct **robot-class
    bodies** matching the task's robot-identity predicate, which each adapter
    answers in its own terms and publishes (``IdentityRule``), corroborated by
    the adapter's count of matching *declarations* in the artifact. The count
    (== 10) and the corroboration floor (>= 10) are the core's; "is this a
    Husky" is the adapter's.

``A1.4 "all ten are controlled"``
    was: a non-empty ``controller`` field plus ten ``Starting controller:``
    lines in the engine log. Now: every robot body has a behaviour attached
    **and** the adapter observed at least as many behaviour-process starts as
    the scene requires. Both halves kept -- a scene that declares controllers
    that never start is still a failure.

``A1.10 backend attribution``
    was: read the ``<log>.newton.json`` sidecar or an ODE pin. Now: the run's
    physics engine/solver is **attributed by evidence the adapter supplies,
    with a citation**. Still not a pass/fail gate -- either engine passes -- and
    a MISSING attribution still makes the run ``INVALID`` rather than ``FAIL``
    (SPEC A1.10), because an unattributed physics result is not a result.

Everything else -- displacement, path length, |dz|, floor clearance, the
circular resultant R, AABB penetration -- was already physical and is
unchanged, threshold for threshold.
"""

from __future__ import annotations

from agentbench.graders import physical as ph
from agentbench.graders.verdict import (ADAPTER_ATTESTED, ARTIFACT_INVALID,
                                        ARTIFACT_LOADS, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, CORE_STRUCTURAL,
                                        INVALID, MIXED, NO_ARTIFACT,
                                        Falsifier, Verdict)

TASK = "A1_husky_swarm_10"

N_ROBOTS = 10
SETTLE_S = 1.0
WINDOW_S = 30.0
MIN_NET_DISP_M = 2.0
MIN_PATH_M = 4.0
MAX_ABS_DZ_M = 0.30
MIN_Z_M = -0.10
MAX_R = 0.70
MIN_AABB_CLEARANCE_M = 0.05
CONTACT_STEPS = 10
# Not a SPEC constant: the net-displacement floor below which a bearing --
# and therefore A1.8's circular resultant -- is not defined at all.
BEARING_EPS_M = 0.05

_PHASE_B = [
    ("A1.3", "none interpenetrating at t=0"),
    ("A1.5", "all ten moved >= 2.0 m"),
    ("A1.6", "they drove, not slid (path length)"),
    ("A1.7", "nobody fell through or tipped over"),
    ("A1.8", "the motion is random, not a parade"),
    ("A1.9", "the run exited cleanly"),
    ("A1.10", "backend attribution recorded"),
]

_ALL = [("A1.1", "exactly ten Huskies"),
        ("A1.2", "ten distinct robots"),
        ("A1.3", "none interpenetrating at t=0"),
        ("A1.4", "all ten are controlled"),
        ("A1.5", "all ten moved >= 2.0 m"),
        ("A1.6", "they drove, not slid (path length)"),
        ("A1.7", "nobody fell through or tipped over"),
        ("A1.8", "the motion is random, not a parade"),
        ("A1.9", "the run exited cleanly"),
        ("A1.10", "backend attribution recorded")]


def grade(bundle, *, self_verified=False):
    """Grade one A1 run from a neutral evidence bundle. Never raises."""
    v = Verdict(TASK, self_verified=self_verified)
    for note in bundle.notes:
        v.note(note)
    L = bundle.lbl

    # ---- the artifact ----------------------------------------------------
    if bundle.artifact is None:
        v.progress = NO_ARTIFACT
        v.add("A1.1", False, "exactly ten Huskies", measured=0,
              threshold=N_ROBOTS, basis=MIXED,
              detail=L("artifact_missing_detail", "no artifact was produced"))
        for aid, what in _ALL[1:]:
            v.add(aid, False, what, detail="no artifact",
                  basis=_BASIS[aid])
        return v.finish()
    v.artifacts["world"] = str(bundle.artifact)
    v.measurements.update(bundle.adapter_measurements.get("identity") or {})
    v.measurements.update(bundle.adapter_measurements.get("roster") or {})

    ident = bundle.identity
    declared = None if ident is None else ident.declared_count

    # ---- A1.1 / A1.2 / A1.4 : structure, from the neutral roster ---------
    roster = bundle.roster
    robots = roster.robots
    v.measurements["robots_seen"] = len(roster.bodies)
    v.measurements["robots_qualifying"] = len(robots)

    v.add("A1.1",
          len(robots) == N_ROBOTS and declared is not None
          and declared >= N_ROBOTS,
          "exactly ten Huskies",
          measured={L("identity_scene_count", "robot-class bodies"):
                    len(robots),
                    L("identity_declared_count",
                      "declared robot instances in the artifact"): declared},
          threshold={L("identity_scene_threshold", "robot bodies"): N_ROBOTS,
                     L("identity_declared_threshold", "declarations"):
                         N_ROBOTS},
          basis=MIXED,
          attested=_identity_citations(ident, roster),
          falsifiers=[
              Falsifier("count",
                        "eight or eleven robot-class bodies in the scene",
                        "a readable body inventory",
                        roster.error is None,
                        detail=roster.error or roster.source),
              Falsifier("identity",
                        "the ten bodies are not the robot the task named "
                        "(a swarm of some other robot scores zero)",
                        "the adapter answers the identity predicate on live "
                        "bodies AND counts matching declarations in the "
                        "artifact",
                        bool(ident is not None and declared is not None
                             and any(b.robot_class is not None
                                     for b in roster.bodies)),
                        detail=("no identity rule supplied" if ident is None
                                else ident.scene_rule)),
          ])

    names = [b.name or "" for b in robots]
    ids = [b.body_id or "" for b in robots]
    distinct_names = len({n for n in names if n})
    distinct_ids = len(set(ids))
    v.add("A1.2", distinct_names == N_ROBOTS and distinct_ids == N_ROBOTS,
          "ten distinct robots",
          measured={"distinct non-empty names": distinct_names,
                    L("distinct_ids", "distinct body ids"): distinct_ids},
          threshold=N_ROBOTS, basis=CORE_STRUCTURAL,
          falsifiers=[
              Falsifier("distinct names",
                        "ten references to one body, or ten bodies sharing a "
                        "name",
                        ">= 2 bodies in the inventory", len(robots) >= 2),
              Falsifier("distinct ids",
                        "ten scene entries resolving to the same body id",
                        ">= 2 bodies carrying an adapter-supplied id",
                        len([b for b in robots if b.body_id]) >= 2),
          ])

    # Two halves, kept separate on purpose. "Declares a behaviour" is what the
    # scene says (and is what a process start has to be looked up by); "is
    # controlled" is whether that declaration names something that will run --
    # a scene of ten robots each declaring the sim's "no controller" sentinel
    # declares ten behaviour fields and is not controlled at all.
    declared_behaviours = [(b.behaviour_declared or "") for b in robots]
    all_controlled = bool(robots) and all(b.controlled for b in robots)
    proc = bundle.process
    starts = dict((proc.behaviour_starts or {}) if proc else {})
    need = {}
    for c in declared_behaviours:
        need[c] = need.get(c, 0) + 1
    started_enough = all(starts.get(c, 0) >= n for c, n in need.items())
    v.add("A1.4",
          all_controlled and started_enough and len(robots) == N_ROBOTS,
          "all ten are controlled",
          measured={L("behaviour_fields", "bodies with a behaviour field set"):
                    sum(1 for b in robots if b.behaviour_declared),
                    L("behaviour_starts", "behaviour processes started"):
                        starts},
          threshold={L("behaviour_fields_threshold", "behaviours"): N_ROBOTS,
                     L("behaviour_starts_threshold", "starts"): need},
          basis=MIXED,
          attested=(["behaviour-process starts: %s" % proc.start_source]
                    if proc and proc.start_source else []),
          falsifiers=[
              Falsifier("behaviour attached",
                        "a robot body with no behaviour/controller attached",
                        ">= 1 robot body whose behaviour field is readable",
                        bool(robots)),
              Falsifier("processes started",
                        "fewer behaviour processes started than the scene "
                        "declares",
                        "the adapter can count behaviour-process starts and "
                        "cites where the count came from",
                        bool(proc is not None and proc.start_source),
                        detail=("no process facts supplied" if proc is None
                                else proc.start_source)),
          ])

    # ---- motion: the standalone, cold run --------------------------------
    traj = bundle.trajectory
    if traj is None or traj.xyz is None:
        v.progress = (ARTIFACT_LOADS if bundle.live_load_ok
                      else ARTIFACT_INVALID)
        why = (bundle.motion_error or (traj.error if traj else None)
               or "no motion samples")
        for aid, what in _PHASE_B:
            v.add(aid, False, what, detail=why, basis=_BASIS[aid])
        v.measurements.update(bundle.adapter_measurements.get("motion") or {})
        return v.finish()

    v.progress = ARTIFACT_RUNS
    v.measurements.update(bundle.adapter_measurements.get("motion") or {})

    # A1.3 -- t=0 AABB separation + no robot-robot contact in 10 steps
    t0_robots = bundle.t0.robots
    boxes = [b.aabb for b in t0_robots if b.has_aabb]
    missing = sum(1 for b in t0_robots if not b.has_aabb)
    if len(boxes) >= 2:
        min_clear, pair = ph.pairwise_min_clearance(boxes)
    else:
        min_clear, pair = float("nan"), None
    contacts = bundle.contacts
    rr = contacts.robot_robot_pairs if contacts else []
    steps = contacts.steps if contacts else CONTACT_STEPS
    aabb_ok = (missing == 0 and len(boxes) == len(t0_robots)
               and min_clear >= MIN_AABB_CLEARANCE_M)
    v.add("A1.3", bool(aabb_ok and not rr),
          "none interpenetrating at t=0",
          measured={"min pairwise AABB clearance (m)":
                    None if min_clear != min_clear else round(min_clear, 4),
                    "worst pair": pair,
                    "robot-robot contacts in first %d steps" % steps: len(rr),
                    "robots without bounds": missing},
          threshold={"clearance (m)": MIN_AABB_CLEARANCE_M, "contacts": 0},
          basis=MIXED,
          attested=(["contact pairing: %s" % contacts.source]
                    if contacts and contacts.source else []),
          falsifiers=[
              Falsifier("AABB separation",
                        "two robots whose world AABBs overlap on all three "
                        "axes at t=0",
                        ">= 2 robot bodies with world-space AABBs, read at a "
                        "frozen instant",
                        bool(len(boxes) >= 2 and bundle.t0.frozen is not False),
                        detail=bundle.t0.source),
              Falsifier("no robot-robot contact",
                        "a contact naming two distinct robot bodies inside "
                        "the first %d steps" % steps,
                        "the contact query produced at least one contact "
                        "naming two DISTINCT bodies (an empty list from a "
                        "query that can only ever return self-pairs proves "
                        "nothing)",
                        bool(contacts is not None
                             and contacts.can_name_a_robot_pair),
                        detail=(contacts.witness_detail if contacts
                                else "no contact observation supplied")),
          ])

    # A1.5 / A1.6 / A1.7 / A1.8 -- motion, in metres
    n = traj.n_bodies
    disp = [ph.net_displacement(traj.xy(i)) for i in range(n)]
    plen = [ph.path_length(traj.xy(i)) for i in range(n)]
    bands = [ph.z_band(traj.z(i)) for i in range(n)]
    thetas = [ph.bearing(traj.xy(i)) for i in range(n)]
    R = ph.circular_resultant(thetas)
    v.measurements["per_robot"] = {
        "net_displacement_m": [round(d, 4) for d in disp],
        "path_length_m": [round(d, 4) for d in plen],
        "z_first_last_min_max_absdz": [[round(x, 4) for x in b]
                                       for b in bands],
        "bearing_rad": [round(t, 4) for t in thetas],
    }

    recorded_s = traj.recorded_s if traj.recorded_s is not None else 0.0
    window_ok = recorded_s >= WINDOW_S - 1e-6
    sampled = traj.n_samples >= 2

    v.add("A1.5", n == N_ROBOTS and window_ok and bool(disp)
          and min(disp) >= MIN_NET_DISP_M,
          "all ten moved >= 2.0 m",
          measured={"min net displacement (m)": round(min(disp), 4) if disp
                    else None, "n robots": n,
                    "recorded window (s)": recorded_s},
          threshold={"net displacement (m)": MIN_NET_DISP_M,
                     "n robots": N_ROBOTS, "window (s)": WINDOW_S},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "displacement floor",
              "a robot whose net displacement over the window is < %.1f m"
              % MIN_NET_DISP_M,
              ">= 2 pose samples per body, so a displacement is defined",
              sampled, detail="%d samples" % traj.n_samples)])

    v.add("A1.6", n == N_ROBOTS and bool(plen) and min(plen) >= MIN_PATH_M,
          "they drove, not slid (path length)",
          measured={"min path length (m)": round(min(plen), 4) if plen
                    else None},
          threshold={"path length (m)": MIN_PATH_M}, basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "path floor",
              "a robot whose integrated path is < %.1f m" % MIN_PATH_M,
              ">= 2 pose samples per body, integrated every step",
              sampled, detail="%d samples" % traj.n_samples)])

    worst_dz = max((b[4] for b in bands), default=float("nan"))
    worst_z = min((b[2] for b in bands), default=float("nan"))
    v.add("A1.7", n == N_ROBOTS and bands
          and worst_dz <= MAX_ABS_DZ_M and worst_z > MIN_Z_M,
          "nobody fell through or tipped over",
          measured={"max |dz| (m)": round(worst_dz, 4),
                    "min z over all samples (m)": round(worst_z, 4)},
          threshold={"|dz| (m)": MAX_ABS_DZ_M, "z (m)": MIN_Z_M},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "vertical band",
              "a robot that sinks below %.2f m or moves %.2f m vertically"
              % (MIN_Z_M, MAX_ABS_DZ_M),
              ">= 1 z sample per body", bool(bands) and traj.n_samples >= 1)])

    # A bearing is only defined for a robot that actually went somewhere:
    # atan2(0, 0) is 0.0, so ten stationary robots silently score R = 1.0 for
    # the wrong reason. Make the degeneracy explicit instead of letting the
    # randomness gate pick up a motion failure by accident.
    n_undefined = sum(1 for d in disp if d < BEARING_EPS_M)
    v.add("A1.8", n == N_ROBOTS and n_undefined == 0 and R < MAX_R,
          "the motion is random, not a parade",
          measured={"circular resultant R": (None if n_undefined
                                             else round(R, 4)),
                    "robots with an undefined bearing "
                    "(net displacement < %.2f m)" % BEARING_EPS_M:
                        n_undefined},
          threshold={"R": "< %.2f" % MAX_R, "undefined bearings": 0},
          detail=("bearing undefined for %d robot(s): they did not move, so "
                  "'random' is not measurable" % n_undefined)
          if n_undefined else "",
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "bearing spread",
              "ten robots on one heading (R = 1.0 for a parade)",
              "every robot has a defined bearing (net displacement >= %.2f m)"
              % BEARING_EPS_M,
              n_undefined == 0 and n >= 2,
              detail="%d of %d bearings undefined" % (n_undefined, n))])

    # A1.9 -- clean exit. Exit code alone is explicitly NOT accepted.
    proc = bundle.process
    errs = list(proc.error_lines) if proc else []
    driver_done = bool(proc and proc.driver_completed)
    finalized = bool(proc and proc.reached_finalize)
    v.add("A1.9", bool(proc and proc.clean and driver_done and finalized),
          "the run exited cleanly",
          measured={L("exit_code", "exit code"): proc.exit_code if proc
                    else None,
                    L("error_lines", "error lines"): len(errs),
                    L("driver_completed", "driver reached a clean quit"):
                        driver_done,
                    L("finalize_evidence", "finalize evidence"):
                        (proc.finalize_evidence if proc else "none"),
                    L("timed_out", "timed out"):
                        bool(proc.timed_out) if proc else None},
          threshold={L("exit_code", "exit code"): 0,
                     L("error_lines", "error lines"): 0},
          detail=("; ".join(errs[:3]) if errs else ""),
          basis=MIXED,
          attested=([("error stream + exit code: %s"
                      % (proc.log_source or "adapter-supplied"))]
                    if proc else []),
          falsifiers=[
              Falsifier("clean exit",
                        "a non-zero exit code, an error-class line, or a "
                        "timeout",
                        "the adapter captured the exit code AND the "
                        "simulator's own error stream",
                        bool(proc is not None and proc.exit_code is not None
                             and proc.log_available is not False)),
              Falsifier("reached finalize",
                        "a world that never finalized (a failed load still "
                        "exits 0 in 2-4 s, so the exit code cannot see this)",
                        "the adapter answers 'did the world finalize' either "
                        "way, from a marker or from the driver's own "
                        "completion record",
                        bool(proc is not None
                             and proc.reached_finalize is not None),
                        detail=(proc.finalize_evidence if proc else "")),
          ])

    # A1.10 -- engine attribution. Not a gate; absence is INVALID.
    attrib = bundle.attribution
    v.attribution = attrib.as_dict() if attrib else {}
    v.add("A1.10", attrib is not None, "backend attribution recorded",
          measured=(attrib.as_dict() if attrib else None),
          threshold=L("attribution_threshold",
                      "an engine/solver attribution with a citation"),
          basis=ADAPTER_ATTESTED,
          attested=(["engine attribution: %s" % attrib.source]
                    if attrib else []),
          falsifiers=[Falsifier(
              "attribution present",
              "a run with no engine verdict record, no explicit engine pin "
              "and no documented engine invariant -- which is INVALID, not "
              "FAIL",
              "the adapter names the channel it read the attribution from",
              bool(attrib is None or attrib.source),
              detail=(attrib.source if attrib else "unattributed"))])
    if attrib is None:
        v.outcome = INVALID
        v.note(L("attribution_missing_note",
                 "no physics-engine attribution: the adapter could not name "
                 "the engine that drove this run from any channel it has. An "
                 "unattributed physics result is not a result (SPEC A1.10)."))
        return v

    return v.finish()


def _identity_citations(ident, roster):
    if ident is None:
        return []
    out = []
    if ident.scene_rule:
        out.append("robot identity in the live scene: %s" % ident.scene_rule)
    if ident.declaration_rule:
        out.append("robot declarations in the artifact: %s"
                   % ident.declaration_rule)
    if roster.source:
        out.append("inventory source: %s" % roster.source)
    return out


# Which basis each assertion carries when it is emitted without evidence (the
# no-artifact / no-motion branches), so the row's basis map is complete either
# way.
_BASIS = {"A1.1": MIXED, "A1.2": CORE_STRUCTURAL, "A1.3": MIXED,
          "A1.4": MIXED, "A1.5": CORE_PHYSICAL, "A1.6": CORE_PHYSICAL,
          "A1.7": CORE_PHYSICAL, "A1.8": CORE_PHYSICAL, "A1.9": MIXED,
          "A1.10": ADAPTER_ATTESTED}
