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

"""L2's neutral core: did it converge, and did it CLOSE A LOOP to get there?

**No simulator vocabulary.** Plain numbers in, a verdict out, so the same code
grades every column. The per-column adapter's job is to turn a run into the
three inputs below and nothing else.

The five clauses
----------------

``L2.1``  it finished within the arrival tolerance of the commanded point
``L2.2``  it was stopped there by the deadline, measured from motion start
``L2.3``  the run is real -- samples exist, the clock advances, no teleports
``L2.4``  the agent closed a loop: at least ``MIN_LOOP_CYCLES`` observe ->
          change -> observe cycles, each change following an observation
``L2.5``  the loop was CAUSAL: the reported numbers changed across attempts

Why L2.4 and L2.5 are separate
------------------------------

L2.4 counts cycles and can be satisfied by an agent that re-runs an unchanged
artifact twice. L2.5 asks whether the measurements actually moved, which is the
difference between iterating and idling. Both are needed: an agent that edits
blindly without measuring fails L2.4, and one that measures repeatedly without
its numbers ever changing fails L2.5.

**A first-try pass is not a pass.** If the target is met with fewer than the
required cycles the outcome is ``NOT_DISCRIMINATING``, not ``PASS`` -- it says
the task was too easy and the rung must be retuned or cut. A rung that cannot
distinguish luck from iteration is not measuring what its name claims.
"""

from __future__ import annotations

import math

CORE_PHYSICAL = "physical"
MIXED = "mixed"
PROCESS = "process"

_ALL = [("L2.1", "it finished within tolerance of the commanded point"),
        ("L2.2", "it was stopped there by the deadline"),
        ("L2.3", "the run is real"),
        ("L2.4", "the agent closed a loop rather than guessing once"),
        ("L2.5", "the loop was causal: its measurements moved")]

_BASIS = {"L2.1": CORE_PHYSICAL, "L2.2": CORE_PHYSICAL, "L2.3": CORE_PHYSICAL,
          "L2.4": PROCESS, "L2.5": PROCESS}


class Clause:
    """One assertion's outcome, with the number that decided it."""

    def __init__(self, key, ok, detail, value=None, vacuous=False):
        self.key = key
        self.ok = bool(ok)
        self.detail = detail
        self.value = value
        self.vacuous = bool(vacuous)

    def as_dict(self):
        return {"ok": self.ok, "detail": self.detail, "value": self.value,
                "vacuous": self.vacuous, "basis": _BASIS.get(self.key)}


def _speeds(t, xy):
    out = []
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if dt <= 0:
            out.append(0.0)
            continue
        out.append(math.hypot(xy[i][0] - xy[i - 1][0],
                              xy[i][1] - xy[i - 1][1]) / dt)
    return out


def motion_start(t, xy, moving_eps):
    """The time the body first exceeds ``moving_eps``, or None if it never did.

    Returned rather than assumed to be zero: the recording begins during a
    settle window, and charging that window to the agent's deadline would
    penalise a column for the length of its own startup.
    """
    sp = _speeds(t, xy)
    for i, s in enumerate(sp):
        if s > moving_eps:
            return t[i]
    return None


def settled_time(t, xy, moving_eps):
    """Seconds from motion start until the body stops for good, or None.

    "For good" and not "the first time it dips": a base that stops, is nudged
    by a contact and rolls again has not settled, and the last crossing is the
    only honest reading of when it did.
    """
    start = motion_start(t, xy, moving_eps)
    if start is None:
        return None
    sp = _speeds(t, xy)
    last_moving = None
    for i, s in enumerate(sp):
        if s > moving_eps:
            last_moving = t[i + 1]
    if last_moving is None:
        return None
    return last_moving - start


def biggest_jump(xy):
    return max((math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
                for i in range(1, len(xy))), default=0.0)


def grade(*, t, xy, goal_xy, cycles, measurements_seen, constants,
          run_error=None):
    """``(outcome, clauses, numbers)``.

    ``t``/``xy``        the graded body's motion, seconds and metres
    ``goal_xy``         the commanded point, resolved by the caller
    ``cycles``          observe->change->observe cycles counted from the trace
    ``measurements_seen`` the ordered list of numbers the agent measured, used
                        only to ask whether they moved
    """
    C = dict(constants)
    tol = float(C["ARRIVE_TOL_M"])
    deadline = float(C["SETTLE_DEADLINE_S"])
    eps = float(C["SETTLED_SPEED_MPS"])
    need = int(C["MIN_LOOP_CYCLES"])

    clauses = {}

    if run_error or not t or len(t) < 5:
        why = run_error or ("the run produced %d samples; nothing can be read "
                            "from it" % len(t or []))
        for key, _ in _ALL:
            clauses[key] = Clause(key, False, why, vacuous=True)
        return "ERROR", clauses, {"run_error": why}

    final = math.hypot(xy[-1][0] - goal_xy[0], xy[-1][1] - goal_xy[1])
    commanded = math.hypot(goal_xy[0] - xy[0][0], goal_xy[1] - xy[0][1])
    settle = settled_time(t, xy, eps)
    jump = biggest_jump(xy)
    dt_max = max((t[i] - t[i - 1] for i in range(1, len(t))), default=0.0)

    clauses["L2.1"] = Clause(
        "L2.1", final <= tol,
        "finished %.4f m from the point (tolerance %.4f m)" % (final, tol),
        value=final)

    if settle is None:
        clauses["L2.2"] = Clause(
            "L2.2", False,
            "the body never moved, so there is no deadline to meet",
            value=None, vacuous=True)
    else:
        clauses["L2.2"] = Clause(
            "L2.2", settle <= deadline,
            "stopped %.2f s after starting to move (deadline %.2f s)"
            % (settle, deadline), value=settle)

    real, why_real = True, []
    if commanded < float(C["MIN_GOAL_DISTANCE_M"]):
        real = False
        why_real.append("the commanded point is %.3f m away, which is not a "
                        "journey" % commanded)
    if jump > float(C["MAX_STEP_JUMP_M"]):
        real = False
        why_real.append("a single step moved %.3f m, which is a teleport"
                        % jump)
    if dt_max > float(C["MAX_SAMPLE_DT_S"]):
        real = False
        why_real.append("a sample gap of %.4f s exceeds the recording "
                        "contract" % dt_max)
    clauses["L2.3"] = Clause("L2.3", real,
                             "; ".join(why_real) or
                             "%d samples, largest step %.4f m, largest gap "
                             "%.4f s" % (len(t), jump, dt_max))

    n_cycles = int(cycles or 0)
    clauses["L2.4"] = Clause(
        "L2.4", n_cycles >= need,
        "%d observe->change->observe cycle(s); %d required" % (n_cycles, need),
        value=n_cycles)

    seen = [m for m in (measurements_seen or []) if m is not None]
    moved = len({round(float(m), 4) for m in seen}) > 1
    if len(seen) < 2:
        clauses["L2.5"] = Clause(
            "L2.5", False,
            "fewer than two measurements were recovered from the trace, so "
            "whether the loop was causal cannot be read", vacuous=True)
    else:
        clauses["L2.5"] = Clause(
            "L2.5", moved,
            "%d measurements recovered, %d distinct"
            % (len(seen), len({round(float(m), 4) for m in seen})),
            value=len(seen))

    numbers = {"final_error_m": final, "settle_s": settle,
               "commanded_distance_m": commanded, "largest_step_m": jump,
               "largest_gap_s": dt_max, "loop_cycles": n_cycles,
               "measurements_recovered": len(seen)}

    hit = clauses["L2.1"].ok and clauses["L2.2"].ok and clauses["L2.3"].ok
    looped = clauses["L2.4"].ok

    if hit and not looped:
        # The rung's own falsifier. Passing here would let luck read as
        # iteration and would quietly make the benchmark uninformative.
        return "NOT_DISCRIMINATING", clauses, numbers
    if all(c.ok for c in clauses.values()):
        return "PASS", clauses, numbers
    return "FAIL", clauses, numbers


def first_failure(clauses):
    for key, _ in _ALL:
        c = clauses.get(key)
        if c is not None and not c.ok:
            return key
    return None
