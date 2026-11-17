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

"""C1 ``parse_error_fix`` -- the **sim-neutral core** of the debug tier.

"This world will not load. Fix it." is trivially satisfiable by cheating:
delete whatever the parser chokes on, or replace the file wholesale with any
trivial world that parses. Both "fixes" load cleanly. So the assertions
describe a *repair* -- the same world, whole again -- not merely the absence of
a load error:

    C1.1  the world loads and the run is clean
                                exit 0, zero error-class lines, the grader's
                                own driver reached its target simulated time
    C1.2  the intended scene survived the fix
                                every body the task shipped is present, BY
                                NAME, in the frozen t=0 inventory of the
                                loaded scene -- deleting or renaming the
                                broken subtree scores FAIL here, not PASS
    C1.3  it steps, and every pose stays finite
                                the recorded window covers >= MIN_RECORDED_S
                                of simulated time and no sample is NaN/inf or
                                further than MAX_ABS_COORD_M from the origin

C1.2 is the anti-amputation / anti-replacement assertion and it is the reason
the task ships a reference roster. C1.1 and C1.2 are graded independently on
purpose: the engine under test *skips* an undefined node type while loading
the rest of the world (measured 2026-07-31: ``Skipped unknown 'Soild' node or
PROTO`` -- two error-class lines, yet the run steps to completion), so a
half-fixed world reds C1.1 AND C1.2 while C1.3 stays green, and an amputated
world reds C1.2 alone. A core that failed everything as soon as the log was
dirty could not tell those apart.

Everything here is metres, seconds, exit codes and names. Names are task
data, not simulator data: the task ships the world, so the intended inventory
is part of the task's own contract (``EXPECTED_BODIES``; the task dir carries
the same list as ``reference_roster.json`` for the adapter/orchestrator side,
and the unit tests assert the two never drift).
"""

from __future__ import annotations

import numpy as np

from agentbench.graders.verdict import (ARTIFACT_INVALID, ARTIFACT_LOADS,
                                        ARTIFACT_RUNS, CORE_PHYSICAL,
                                        CORE_STRUCTURAL, GRADED_PASS, INVALID,
                                        MIXED, NO_ARTIFACT, Falsifier, Verdict)

TASK = "C1_parse_error_fix"

# Task data, not simulator data: the world the task ships names these bodies,
# and a fix that loses any of them is an amputation, not a repair.
ROBOT_NAME = "probe_bot"
EXPECTED_BODIES = ("floor", "pallet_a", "pallet_b", "probe_bot")

# The graded standalone window is 5.0 s (meta.json); demanding at least 4.0 s
# recorded keeps a run that crashed mid-window from passing on its first
# samples, while tolerating the sub-second start-up jitter of a real launch.
MIN_RECORDED_S = 4.0

# A body further than this from the origin did not "load and step" in any
# sense the task means -- it escaped. Matches the suite's runaway convention
# (the |z| limit the runaway watchdog defaults to).
MAX_ABS_COORD_M = 1000.0

_ALL = [("C1.1", "the world loads and the run is clean"),
        ("C1.2", "the intended scene survived the fix"),
        ("C1.3", "it steps, and every pose stays finite")]

_BASIS = {"C1.1": MIXED, "C1.2": CORE_STRUCTURAL, "C1.3": CORE_PHYSICAL}


def grade(bundle, *, self_verified=False, expected_bodies=None):
    expected = (tuple(expected_bodies) if expected_bodies is not None
                else EXPECTED_BODIES)
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
    v.measurements.update(bundle.adapter_measurements.get("motion") or {})

    traj = bundle.trajectory
    proc = bundle.process

    attrib = bundle.attribution
    v.attribution = attrib.as_dict() if attrib else {}
    if traj is not None and traj.xyz is not None and attrib is None:
        # The A1.10 rule, applied suite-wide: a run that produced a physics
        # result nobody can attribute to an engine is not a result. A world
        # that never loaded produced no physics result, so it stays a FAIL
        # below rather than becoming INVALID here.
        v.outcome = INVALID
        v.note(L("attribution_missing_note",
                 "no physics-engine attribution (SPEC A1.10 rule, applied "
                 "suite-wide): the adapter could not name the engine that "
                 "drove this run from any channel it has."))
        return v

    # --- C1.1: loads, and the run is clean --------------------------------
    errs = list(proc.error_lines) if proc else []
    complete = bool(proc and proc.driver_completed)
    c11 = v.add(
        "C1.1", bool(proc and proc.clean and complete),
        "the world loads and the run is clean",
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
            "clean load",
            "a parse failure, an error-class line (a skipped node logs one "
            "even when the rest of the world loads), a non-zero exit, a "
            "timeout, or a driver that never reached its target simulated "
            "time",
            "the adapter captured the exit code AND the simulator's own "
            "error stream",
            bool(proc is not None and proc.exit_code is not None
                 and proc.log_available is not False))])

    # --- C1.2: the intended scene survived the fix ------------------------
    t0 = bundle.t0
    present = sorted(n for n in t0.names if n)
    missing = [n for n in expected if n not in set(present)]
    extra = sorted(set(present) - set(expected))
    inventory_witness = bool(t0.bodies) and t0.frozen is True and not t0.error
    v.add("C1.2", not missing,
          "the intended scene survived the fix",
          measured={"bodies present at t=0": present,
                    "intended bodies missing": missing,
                    "bodies beyond the intended roster": extra,
                    "intended roster": list(expected)},
          threshold={"intended bodies missing": 0},
          detail=(t0.error or ""), basis=CORE_STRUCTURAL,
          falsifiers=[Falsifier(
              "intended inventory",
              "a fix that deletes or renames part of the scene instead of "
              "repairing it -- the loaded inventory then misses an intended "
              "body (an amputated subtree, or a wholesale replacement world)",
              "a frozen t=0 inventory naming the bodies actually present in "
              "the loaded scene",
              inventory_witness,
              detail=t0.error or t0.source or "no inventory")])

    # --- C1.3: steps, with finite poses -----------------------------------
    if traj is None or traj.xyz is None:
        why = (bundle.motion_error or (traj.error if traj else None)
               or "no samples")
        v.add("C1.3", False, "it steps, and every pose stays finite",
              detail=why, basis=CORE_PHYSICAL,
              falsifiers=[Falsifier(
                  "finite stepping",
                  "a NaN/inf pose, a body escaping past %.0f m, or a run "
                  "that ended before the graded window" % MAX_ABS_COORD_M,
                  ">= 2 pose samples over a recorded window",
                  False, detail=why)])
        v.progress = ARTIFACT_LOADS if c11 else ARTIFACT_INVALID
        return v.finish()

    xyz = traj.xyz
    finite = bool(np.isfinite(xyz).all()) if xyz.size else False
    finite_vals = xyz[np.isfinite(xyz)]
    max_abs = (float(np.max(np.abs(finite_vals))) if finite_vals.size
               else float("nan"))
    recorded = float(traj.recorded_s or 0.0)
    v.add("C1.3",
          bool(finite and max_abs <= MAX_ABS_COORD_M
               and recorded >= MIN_RECORDED_S and traj.n_samples >= 2),
          "it steps, and every pose stays finite",
          measured={"recorded window (s)": round(recorded, 3),
                    "samples": traj.n_samples,
                    "tracked bodies": traj.n_bodies,
                    "all poses finite": finite,
                    "max |coordinate| (m)": (round(max_abs, 3)
                                             if max_abs == max_abs else None)},
          threshold={"recorded window (s)": ">= %.1f" % MIN_RECORDED_S,
                     "|coordinate| (m)": "<= %.0f" % MAX_ABS_COORD_M,
                     "all poses finite": True},
          basis=CORE_PHYSICAL,
          falsifiers=[Falsifier(
              "finite stepping",
              "a NaN/inf pose, a body escaping past %.0f m, or a run that "
              "ended before the graded window" % MAX_ABS_COORD_M,
              ">= 2 pose samples and a recorded window length",
              bool(traj.n_samples >= 2 and traj.recorded_s is not None),
              detail="recorded %s s" % (traj.recorded_s,))])
    v.progress = ARTIFACT_RUNS
    return v.finish(GRADED_PASS)
