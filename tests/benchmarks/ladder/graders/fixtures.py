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

"""T1's negative fixtures -- the red-evidence rule, made executable.

``capability-ladder-plan.md`` §5c.2, binding and standing: *"no assertion
enters a ladder cell until it has been observed FAILING on a deliberately
wrong artifact, with that negative fixture named in the assertion's record. A
green assertion is not evidence that the assertion works ... A ``null`` agent
turning every assertion red does not satisfy this rule."*

That last sentence is what shapes this module. Every fixture below is the
:func:`oracle` run with **one** thing wrong, and :data:`FIXTURE_ASSERTION_MAP`
records which single assertion it must turn red -- so a fixture that reddens
two assertions, or the wrong one, is a test failure. A fixture set where every
artifact fails everything would satisfy a naive "we saw it fail" check and
prove nothing about whether the assertions discriminate.

Run the coverage table::

    python -m ladder.graders.fixtures

Everything here is synthetic numpy: no simulator, no build, no GPU, no
network. A third party can re-derive every red in this file on a laptop.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from agentbench.graders.evidence import (  # noqa: E402
    Body, BodyInventory, ContactObservation, EngineAttribution, EvidenceBundle,
    IdentityRule, ProcessFacts, Trajectory)
from agentbench.graders.verdict import FAIL, INVALID, PASS  # noqa: E402
from ladder.graders import t1_core  # noqa: E402
from ladder.graders.ladder_evidence import (  # noqa: E402
    SupportContact, SupportContactObservation, T1Evidence, Waypoint)

DT = 0.016                       # a plausible basic timestep, seconds
ROBOT_NAME = "rover"
SURFACE_NAME = "ground"
GOAL_XY = (0.0, 5.0)             # five metres north of the start
START_XY = (0.0, 0.0)
RIDE_HEIGHT_M = 0.15
TRAVEL_S = 12.0
HOLD_S = 4.0

WAYPOINT_SOURCE = "the task's own waypoint file, read by the grader"
SUPPORT_SOURCE = ("a synthetic support-contact scan over the travel window "
                  "(a fixture, not a run)")


# --- track construction ------------------------------------------------------


def _smoothstep(f):
    return f * f * (3.0 - 2.0 * f)


def _track(*, goal=GOAL_XY, start=START_XY, dt=DT, travel_s=TRAVEL_S,
           hold_s=HOLD_S, z0=RIDE_HEIGHT_M, arrive_frac=1.0, wander_m=0.0,
           wander_cycles=6, teleport_m=0.0, fall_to=None, leave_m=0.0,
           wait_s=0.0):
    """One synthetic pose series, with at most one thing wrong with it.

    ``wait_s`` holds the robot at the start before it moves, so a late
    arrival can be built without also shortening the path (which would
    redden a second assertion and make the fixture stop isolating one).
    """
    n_wait = int(round(wait_s / dt))
    n_travel = int(round(travel_s / dt))
    n_hold = int(round(hold_s / dt))
    n = n_wait + n_travel + n_hold + 1
    t = np.arange(n) * dt

    prog = np.clip((np.arange(n) - n_wait) / float(n_travel), 0.0, 1.0)
    s = _smoothstep(prog)
    gx = start[0] + arrive_frac * (goal[0] - start[0])
    gy = start[1] + arrive_frac * (goal[1] - start[1])
    x = start[0] + s * (gx - start[0])
    y = start[1] + s * (gy - start[1])
    z = np.full(n, float(z0))

    if wander_m:
        x = x + wander_m * np.sin(2.0 * np.pi * wander_cycles * s)

    xyz = np.stack([x, y, z], axis=1)
    end_travel = n_wait + n_travel

    if teleport_m:
        k = n_wait + int(0.45 * n_travel)
        p = xyz[k, :2] + np.array([0.0, float(teleport_m)])
        rest = end_travel - k
        g = _smoothstep(np.linspace(0.0, 1.0, rest + 1))
        xyz[k:end_travel + 1, 0] = p[0] + g * (gx - p[0])
        xyz[k:end_travel + 1, 1] = p[1] + g * (gy - p[1])
        xyz[end_travel + 1:, 0] = gx
        xyz[end_travel + 1:, 1] = gy

    if leave_m:                       # arrive, hold briefly, then drive off
        k = end_travel + int(round(1.0 / dt))
        m = n - k
        g = _smoothstep(np.linspace(0.0, 1.0, m))
        xyz[k:, 1] = gy + g * float(leave_m)

    if fall_to is not None:           # a gradual sink, no per-sample jump
        k = end_travel + int(round(1.0 / dt))
        m = n - k
        g = _smoothstep(np.linspace(0.0, 1.0, m))
        xyz[k:, 2] = z0 + g * (float(fall_to) - z0)

    return t, xyz


def _support(t, xyz, *, every=40, first=None, last=None, timed=True,
             supported=True, total_observed=None, distinct_named=None,
             pairs=True):
    """Support contacts sampled along a track. Defaults describe a rolling
    robot that is in contact with the surface the whole way."""
    n = len(t)
    lo = 0 if first is None else int(first)
    hi = n if last is None else int(last)
    out = []
    if pairs:
        for i in range(lo, hi, every):
            out.append(SupportContact(
                robot_body=ROBOT_NAME, surface_body=SURFACE_NAME,
                surface_is_robot=False,
                point=(float(xyz[i][0]), float(xyz[i][1]), 0.0),
                t_s=(float(t[i]) if timed else None), step=i))
    return SupportContactObservation(
        pairs=out, supported=supported,
        total_observed=(len(out) * 4 if total_observed is None
                        else total_observed),
        distinct_named=(len(out) * 4 if distinct_named is None
                        else distinct_named),
        steps_sampled=max(0, (hi - lo) // max(1, every)),
        window_s=float(t[-1] - t[0]) if n else None,
        source=SUPPORT_SOURCE)


# --- the bundle --------------------------------------------------------------


def _bundle(t, xyz, *, attribution=True, error_lines=(), finalize=True,
            exit_code=0, timed_out=False, contacts=None,
            deliverable="/synthetic/T1_arrive/run.txt", dt=DT,
            robot_name=ROBOT_NAME, extra_bodies=()):
    body = Body(body_id="#11", name=robot_name, kind="Robot",
                position=(float(xyz[0][0]), float(xyz[0][1]),
                          float(xyz[0][2])),
                aabb_min=(-0.4, -0.3, 0.0), aabb_max=(0.4, 0.3, 0.35),
                n_joints=4, behaviour="drive", behaviour_declared="drive",
                dynamic=True, robot_class=True,
                identity_evidence="synthetic fixture")
    surface = Body(body_id="#3", name=SURFACE_NAME, kind="Solid",
                   aabb_min=(-25.0, -25.0, -0.1), aabb_max=(25.0, 25.0, 0.0),
                   dynamic=False, robot_class=False,
                   identity_evidence="synthetic fixture")
    bodies = [body] + list(extra_bodies)
    return EvidenceBundle(
        task=t1_core.TASK, sim="synthetic", adapter="ladder.fixtures",
        artifact=deliverable,
        identity=IdentityRule(
            label="wheeled robot",
            requirement="the robot the description in the container declares",
            scene_rule="synthetic: robot_class is set on the fixture",
            declaration_rule="synthetic: not counted",
            declared_count=1),
        roster=BodyInventory(bodies=bodies, frozen=True, t_s=0.0,
                             source="synthetic frozen scan"),
        t0=BodyInventory(bodies=bodies + [surface], frozen=True, t_s=0.0,
                         source="synthetic frozen scan"),
        contacts=(contacts if contacts is not None else ContactObservation(
            pairs=[], steps=10, supported=True, total_observed=40,
            distinct_named=40, source="synthetic general contact scan")),
        trajectory=Trajectory(body_ids=[b.body_id for b in bodies], t=t,
                              xyz=np.stack([xyz] + [xyz] * (len(bodies) - 1),
                                           axis=0),
                              dt_s=dt, recorded_s=float(t[-1] - t[0]),
                              complete=True, source="synthetic pose series"),
        process=ProcessFacts(
            exit_code=exit_code, timed_out=timed_out,
            error_lines=list(error_lines), log_available=True,
            log_source="synthetic error stream", behaviour_starts={"drive": 1},
            start_source="synthetic process table",
            driver_completed=bool(finalize), reached_finalize=bool(finalize),
            finalize_evidence=("the driver reached its target simulated time"
                               if finalize else "none"),
            wall_s=42.0),
        attribution=(EngineAttribution(
            backend="synthetic", solver="synthetic-pgs",
            source="a synthetic fixture, not a run") if attribution else None))


def _evidence(t, xyz, support=None, **kw):
    return T1Evidence(
        bundle=_bundle(t, xyz, **kw),
        waypoint=Waypoint(xy=GOAL_XY, label="the commanded point",
                          source=WAYPOINT_SOURCE),
        support=(support if support is not None else _support(t, xyz)),
        robot_name=kw.get("robot_name", ROBOT_NAME))


# --- the fixtures ------------------------------------------------------------


def oracle():
    """A robot that drives to the point, holds station, and is measured."""
    t, xyz = _track()
    return _evidence(t, xyz)


def teleported():
    """T1.3: one 2.6 m inter-sample jump. Everything else is unchanged."""
    t, xyz = _track(teleport_m=2.6)
    return _evidence(t, xyz)


def never_arrived():
    """T1.1: it drove a full 5 m of path and stopped 2.5 m short of the point.

    The detour is not decoration. A robot that simply stops halfway also has
    a path length half the straight line, which reddens T1.3 as well and
    makes the fixture stop isolating T1.1 -- so this one travels the honest
    distance and still does not get there.
    """
    t, xyz = _track(arrive_frac=0.5, wander_m=0.5, wander_cycles=2)
    return _evidence(t, xyz)


def arrived_then_left():
    """T1.1: it reached the point, held for 1 s, then drove 2 m past it."""
    t, xyz = _track(leave_m=2.0)
    return _evidence(t, xyz)


def no_dwell():
    """T1.1: it waits, dashes to the point, and the run ends as it gets there.

    The distinction the tier's dwell window exists to draw: a body that is
    momentarily at the commanded point when the clock runs out has not
    demonstrably stopped there.
    """
    t, xyz = _track(wait_s=14.0, travel_s=1.6, hold_s=0.2)
    return _evidence(t, xyz)


def fell():
    """T1.2: it arrives on the mark and then sinks 0.9 m below its start."""
    t, xyz = _track(fall_to=-0.75)
    return _evidence(t, xyz)


def wandering():
    """T1.3: it arrives, having gone about seven times the straight line."""
    t, xyz = _track(wander_m=1.5, wander_cycles=6)
    return _evidence(t, xyz)


def flying_no_contact():
    """T1.4: a perfect arrival with no contact -- and the query works.

    The vacuity witness is deliberately PRESENT (other bodies in the scene
    were in contact, so the query demonstrably names two distinct bodies).
    That is what makes this a clean red rather than an unfalsifiable one.
    """
    t, xyz = _track()
    return _evidence(t, xyz, support=SupportContactObservation(
        pairs=[], supported=True, total_observed=12, distinct_named=12,
        steps_sampled=int(len(t) / 40), window_s=float(t[-1] - t[0]),
        source=SUPPORT_SOURCE))


def blind_contact_query():
    """T1.4 red AND vacuous: no pairs, and no evidence any could be reported.

    The shape of the historical failure the vacuity machinery exists for: a
    query that returns nothing and cannot say whether it *could* have returned
    something. The assertion is red (the tier requires a contact) and the row
    must also say the clause proved nothing.
    """
    t, xyz = _track()
    return _evidence(t, xyz, support=SupportContactObservation(
        pairs=[], supported=True, total_observed=None, distinct_named=None,
        steps_sampled=int(len(t) / 40), window_s=float(t[-1] - t[0]),
        source="a contact query with no witness counters"))


def contact_only_at_rest():
    """T1.4: contact exists, but only after the robot stopped moving.

    The physical story it rejects: a body dragged to the point by something
    other than its own actuation, which then settles and touches the surface.
    """
    t, xyz = _track()
    n_travel = int(round(TRAVEL_S / DT))
    return _evidence(t, xyz, support=_support(t, xyz, every=20,
                                              first=n_travel + 100))


def never_finalized():
    """T1.5: the world never finished building. Exit code still 0."""
    t, xyz = _track()
    return _evidence(t, xyz, finalize=False)


def error_lines():
    """T1.5: the engine wrote an error-class line during the run."""
    t, xyz = _track()
    return _evidence(t, xyz, error_lines=["ERROR: a fixture's error line"])


def missing_attribution():
    """T1.5 red, verdict INVALID: nothing names the engine that drove it."""
    t, xyz = _track()
    return _evidence(t, xyz, attribution=False)


def untimed_contacts():
    """A partial-evidence case, NOT a negative: contacts with no timestamps.

    T1.4 still passes on the half that is measurable (support was observed)
    and reports the during-motion clause vacuous. It is here because a column
    that can see contacts but not time them is a realistic adapter, and the
    row must show which half of the assertion was doing work.
    """
    t, xyz = _track()
    return _evidence(t, xyz, support=_support(t, xyz, timed=False))


# --- the map ----------------------------------------------------------------
#
# fixture -> what it must do to the verdict. Read by test_t1_core.py, printed
# by coverage_table(). Both columns are EXACT:
#
#   ``red``      the assertions this artifact must fail, and NO others. A
#                fixture that reddens something not listed here has stopped
#                isolating one assertion, which is what §5c's "a null agent
#                turning every assertion red does not satisfy this rule"
#                forbids.
#   ``vacuous``  {assertion: (clause, ...)} -- the clauses that could not have
#                failed on this artifact. Named per CLAUSE, not per assertion,
#                because "no contact was reported" and "no contact could ever
#                have been reported" are different findings and the row has to
#                keep them apart.

_C_TOUCHED = "touched the surface"
_C_WHILE_MOVING = "touched it while moving"

FIXTURE_ASSERTION_MAP = {
    "oracle": {
        "fn": oracle, "red": (), "outcome": PASS, "vacuous": {},
        "wrong_with_it": "nothing -- this is the positive control"},
    "never_arrived": {
        "fn": never_arrived, "red": ("T1.1",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "stops 2.5 m short of the commanded point"},
    "arrived_then_left": {
        "fn": arrived_then_left, "red": ("T1.1",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "reaches the point, then drives 2 m past it"},
    "no_dwell": {
        "fn": no_dwell, "red": ("T1.1",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "arrives as the clock runs out; no dwell window"},
    "fell": {
        "fn": fell, "red": ("T1.2",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "sinks 0.9 m below its settled start height"},
    "teleported": {
        "fn": teleported, "red": ("T1.3",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "one 2.6 m inter-sample displacement"},
    "wandering": {
        "fn": wandering, "red": ("T1.3",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "travels about 7x the straight-line distance"},
    "flying_no_contact": {
        "fn": flying_no_contact, "red": ("T1.4",), "outcome": FAIL,
        "vacuous": {"T1.4": (_C_WHILE_MOVING,)},
        "wrong_with_it": "arrives without ever touching a surface"},
    "blind_contact_query": {
        "fn": blind_contact_query, "red": ("T1.4",), "outcome": FAIL,
        "vacuous": {"T1.4": (_C_TOUCHED, _C_WHILE_MOVING)},
        "wrong_with_it": "no contact, and no witness that one could have "
                         "been reported"},
    "contact_only_at_rest": {
        "fn": contact_only_at_rest, "red": ("T1.4",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "touches the surface only after it stopped moving"},
    "never_finalized": {
        "fn": never_finalized, "red": ("T1.5",), "outcome": FAIL,
        "vacuous": {},
        "wrong_with_it": "the world never finished building; exit code 0"},
    "error_lines": {
        "fn": error_lines, "red": ("T1.5",), "outcome": FAIL, "vacuous": {},
        "wrong_with_it": "an error-class line during the run"},
    "missing_attribution": {
        "fn": missing_attribution, "red": ("T1.5",), "outcome": INVALID,
        "vacuous": {},
        "wrong_with_it": "nothing names the engine that drove the run"},
    "untimed_contacts": {
        "fn": untimed_contacts, "red": (), "outcome": PASS,
        "vacuous": {"T1.4": (_C_WHILE_MOVING,)},
        "wrong_with_it": "nothing physical -- the adapter cannot timestamp "
                         "contacts, so half of T1.4 proves nothing"},
}

# Every assertion the tier states must appear as some fixture's target.
ASSERTIONS = tuple(aid for aid, _what in t1_core._ALL)


def uncovered_assertions():
    """Assertions no negative fixture drives red. Must be empty at freeze."""
    covered = set()
    for spec in FIXTURE_ASSERTION_MAP.values():
        covered.update(spec["red"])
    return tuple(a for a in ASSERTIONS if a not in covered)


def coverage_table():
    """``[{assertion, fixture, wrong_with_it, outcome}]``, one row per red."""
    rows = []
    for aid in ASSERTIONS:
        for name, spec in FIXTURE_ASSERTION_MAP.items():
            if aid in spec["red"]:
                rows.append({"assertion": aid, "fixture": name,
                             "wrong_with_it": spec["wrong_with_it"],
                             "outcome": spec["outcome"]})
    return rows


def render_coverage_table():
    lines = ["T1 red-evidence coverage -- every assertion, the fixture that "
             "turns it red",
             "",
             "%-7s  %-22s  %-9s  %s" % ("assert", "fixture", "outcome",
                                        "what is wrong with the artifact"),
             "%-7s  %-22s  %-9s  %s" % ("-" * 7, "-" * 22, "-" * 9, "-" * 46)]
    for r in coverage_table():
        lines.append("%-7s  %-22s  %-9s  %s"
                     % (r["assertion"], r["fixture"], r["outcome"],
                        r["wrong_with_it"]))
    missing = uncovered_assertions()
    lines += ["", ("every assertion has a negative fixture"
                   if not missing else
                   "UNCOVERED (must be empty before freeze): %s"
                   % ", ".join(missing))]
    return "\n".join(lines)


def observed_vacuous(verdict):
    """``{assertion: (clause, ...)}``, normalised against the map."""
    return {aid: tuple(sorted(clauses))
            for aid, clauses in verdict.vacuous.items()}


def expected_vacuous(spec):
    return {aid: tuple(sorted(clauses))
            for aid, clauses in (spec.get("vacuous") or {}).items()}


def check_fixture(name):
    """``(verdict, problems)`` for one fixture, against its declared row."""
    spec = FIXTURE_ASSERTION_MAP[name]
    v = t1_core.grade(spec["fn"]())
    problems = []
    red, want = tuple(sorted(v.failed)), tuple(sorted(spec["red"]))
    if red != want:
        problems.append("red is %s, declared %s" % (red or ("-",), want
                                                    or ("-",)))
    if v.outcome != spec["outcome"]:
        problems.append("outcome is %s, declared %s"
                        % (v.outcome, spec["outcome"]))
    if observed_vacuous(v) != expected_vacuous(spec):
        problems.append("vacuous clauses are %s, declared %s"
                        % (observed_vacuous(v), expected_vacuous(spec)))
    return v, problems


def main():
    print(render_coverage_table())
    print("")
    print("live check -- grading every fixture now:")
    bad = 0
    for name in FIXTURE_ASSERTION_MAP:
        v, problems = check_fixture(name)
        bad += 1 if problems else 0
        vac = ";".join("%s:%s" % (a, "/".join(c))
                       for a, c in sorted(observed_vacuous(v).items()))
        print("  %-22s %-8s red=%-14s vacuous=%-42s %s"
              % (name, v.outcome, ",".join(sorted(v.failed)) or "-",
                 vac or "-", "ok" if not problems else "MISMATCH: "
                 + "; ".join(problems)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
