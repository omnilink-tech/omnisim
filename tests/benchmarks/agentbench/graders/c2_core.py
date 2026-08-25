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

"""C2 ``fall_through_floor`` -- the **sim-neutral core** of the debug tier.

"Fix it and prove it stays up" is trivially satisfiable by cheating: delete the
robot's dynamics, or nail it to a static pose, and it "stays up" forever. So the
assertions describe the physics of a body that is genuinely resting on a
surface, not the absence of falling:

    C2.1  the run is clean            exit 0, no error lines, driver complete
    C2.2  the robot stays up          z > -0.10 m at EVERY sample
    C2.3  it is still a dynamic body  a mass/dynamics model is still attached
    C2.4  gravity still acts on it    it descended >= 0.10 m and then came to
                                      rest (z varies <= 0.02 m over the last 1 s)
    C2.5  it rests ON the floor       |z_end - (floor_top + half height)|
                                      <= 0.15 m, both read from the t=0 AABBs

C2.3 + C2.4 are the anti-cheat pair. C2.5 catches "fixed" by teleporting the
robot somewhere it never touches the floor.

Everything here is metres and seconds. The only non-physical input is
``Body.dynamic`` for C2.3 -- "does this body still have a mass model" has a
different spelling in every simulator (OmniSim/Webots: a ``Physics`` node;
Gazebo: a non-``static`` link with inertia; MuJoCo: a body with a non-zero
mass), so the adapter answers it and the core asserts on the answer.
"""

from __future__ import annotations

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_RUNS,
                                        CORE_PHYSICAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier, Verdict)

TASK = "C2_fall_through_floor"

# Task data, not simulator data: the world the task ships names these two
# bodies, and the prompt is about them.
ROBOT_NAME = "crate_bot"
FLOOR_NAME = "floor"

MIN_Z_M = -0.10
MIN_DESCENT_M = 0.10
REST_BAND_M = 0.02
REST_WINDOW_S = 1.0
REST_TOL_M = 0.15

_ALL = [("C2.1", "the run is clean"),
        ("C2.2", "the robot stays up"),
        ("C2.3", "it is still a dynamic body"),
        ("C2.4", "gravity still acts on it, and it came to rest"),
        ("C2.5", "it rests on the floor surface")]

_BASIS = {"C2.1": MIXED, "C2.2": CORE_PHYSICAL, "C2.3": MIXED,
          "C2.4": CORE_PHYSICAL, "C2.5": CORE_PHYSICAL}


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

    # locate the body the task is about, in the neutral roster
    idx = bundle.roster.index_of_name(ROBOT_NAME)
    if idx is None:
        idx = 0 if bundle.roster.bodies else None
    if idx is None:
        v.outcome = INVALID
        v.note(L("no_body_note",
                 "the run reported no robot body at all"))
        return v
    body = bundle.roster.bodies[idx]

    z = traj.z(idx)
    proc = bundle.process
    errs = list(proc.error_lines) if proc else []
    complete = bool(proc and proc.driver_completed)
    v.add("C2.1", bool(proc and proc.clean and complete),
          "the run is clean",
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

    z_min = float(z.min())
    v.add("C2.2", z_min > MIN_Z_M, "the robot stays up",
          measured={"min z over all samples (m)": round(z_min, 4),
                    "z_end (m)": round(float(z[-1]), 4)},
          threshold={"z (m)": "> %.2f" % MIN_Z_M}, basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "vertical floor",
              "one z sample at or below %.2f m" % MIN_Z_M,
              ">= 1 z sample", traj.n_samples >= 1)])

    v.add("C2.3", body.dynamic is True, "it is still a dynamic body",
          measured={L("dynamic", "a dynamics/mass model is attached"):
                    body.dynamic},
          threshold=True, basis=MIXED,
          attested=(["dynamics model: %s" % (body.identity_evidence
                                             or bundle.roster.source)]
                    if bundle.roster.source else []),
          falsifiers=[Falsifier(
              "dynamics attached",
              "the agent 'fixed' the fall by removing the body's dynamics",
              "the adapter answers 'is this body dynamic' either way, rather "
              "than returning None",
              body.dynamic is not None)])

    descent = float(z.max() - z.min())
    tail = traj.tail(idx, REST_WINDOW_S)
    rest_band = float(tail.max() - tail.min()) if len(tail) else float("nan")
    v.add("C2.4", descent >= MIN_DESCENT_M and rest_band <= REST_BAND_M,
          "gravity still acts on it, and it came to rest",
          measured={"total descent (m)": round(descent, 4),
                    "z spread over the last %.1f s (m)" % REST_WINDOW_S:
                        round(rest_band, 5)},
          threshold={"descent (m)": ">= %.2f" % MIN_DESCENT_M,
                     "rest band (m)": "<= %.3f" % REST_BAND_M},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "descended then rested",
              "a body pinned in place (no descent) or still moving at the end",
              ">= 2 samples, and a recorded window longer than the %.1f s "
              "rest test" % REST_WINDOW_S,
              bool(traj.n_samples >= 2
                   and (traj.recorded_s or 0.0) > REST_WINDOW_S),
              detail="recorded %s s" % (traj.recorded_s,))])

    expected, floor_top, half_h = _expected_rest_z(bundle, idx)
    if expected is None:
        v.add("C2.5", False, "it rests on the floor surface",
              measured={"expected rest z": None},
              threshold=None, basis=CORE_PHYSICAL,
              detail="could not measure the floor top / robot height at t=0",
              falsifiers=[Falsifier(
                  "rests on the surface",
                  "a body resting anywhere other than on top of the floor it "
                  "built",
                  "world-space AABBs for BOTH the body and the floor at a "
                  "frozen t=0", False,
                  detail=bundle.t0.source or "no t=0 geometry")])
    else:
        err = abs(float(z[-1]) - expected)
        v.add("C2.5", err <= REST_TOL_M, "it rests on the floor surface",
              measured={"z_end (m)": round(float(z[-1]), 4),
                        "expected rest z (m)": round(expected, 4),
                        "floor top z (m)": round(floor_top, 4),
                        "robot half-height (m)": round(half_h, 4),
                        "error (m)": round(err, 4)},
              threshold={"error (m)": "<= %.2f" % REST_TOL_M},
              basis=CORE_PHYSICAL,
              falsifiers=[Falsifier(
                  "rests on the surface",
                  "a body resting more than %.2f m from the top of the floor "
                  "it built" % REST_TOL_M,
                  "world-space AABBs for BOTH the body and the floor at a "
                  "frozen t=0", True, detail=bundle.t0.source)])

    return v.finish(GRADED_PASS)


def _expected_rest_z(bundle, robot_index):
    """floor_top + half the body's height, BOTH measured from t=0 geometry.

    Neither number is hard-coded from the task's authored dimensions -- an
    agent that rebuilds the floor at a different thickness or height is still
    graded against the floor it actually built. The adapter is asked to include
    the named floor body in the t=0 inventory for exactly this reason.
    """
    robots = bundle.t0.robots
    if robot_index >= len(robots):
        return None, None, None
    body = robots[robot_index]
    if not body.has_aabb:
        return None, None, None
    half_h = body.height / 2.0

    floor = None
    for b in bundle.t0.bodies:
        if b.name == FLOOR_NAME and b.has_aabb:
            floor = b
            break
    if floor is None:
        return None, None, half_h
    floor_top = floor.top_z
    return floor_top + half_h, floor_top, half_h
