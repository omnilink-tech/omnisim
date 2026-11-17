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

"""P1's neutral core: did something drive a square perimeter, over and over?

**No simulator vocabulary.** A time series of horizontal positions goes in, a
verdict comes out, so the same code grades every column.

The five clauses
----------------

``P1.1``  it completed at least ``MIN_LAPS`` laps
``P1.2``  the route is a square of about the commanded side
``P1.3``  it was STILL patrolling at the end -- a lap closed in the last third
``P1.4``  the run is real: no teleports, no impossible sample gaps
``P1.5``  it stayed on the ground rather than flying or sinking

Why "still patrolling at the end" is its own clause
---------------------------------------------------

The prompt says *keep going like that in a loop*. A robot that drives the
square twice and then stops, drifts, or falls over has done something
materially different from one that is still going when the recording ends, and
a lap count alone cannot tell them apart. It is the cheapest available proxy
for "it kept working", and it is the clause an open-loop script that runs out
of waypoints fails.

What a lap is
-------------

Leaving a circle of radius ``LAP_EXIT_M`` around the start and later coming
back inside ``LAP_RETURN_M``. Counted on returns rather than on corners because
corner-detection would encode a particular driving style, and the task is the
route, not the technique.
"""

from __future__ import annotations

import math

CORE_PHYSICAL = "physical"

_ALL = [("P1.1", "it completed the required laps"),
        ("P1.2", "the route is a square of about the commanded side"),
        ("P1.3", "it was still patrolling when the recording ended"),
        ("P1.4", "the run is real"),
        ("P1.5", "it stayed on the ground")]


class Clause:
    def __init__(self, key, ok, detail, value=None, vacuous=False):
        self.key = key
        self.ok = bool(ok)
        self.detail = detail
        self.value = value
        self.vacuous = bool(vacuous)

    def as_dict(self):
        return {"ok": self.ok, "detail": self.detail, "value": self.value,
                "vacuous": self.vacuous, "basis": CORE_PHYSICAL}


def lap_times(t, xy, *, exit_m, return_m):
    """Times at which the body came back to its start, having genuinely left.

    Requires the excursion FIRST, so a body jittering inside the return radius
    scores nothing however long it sits there.
    """
    if not t:
        return []
    ox, oy = xy[0]
    laps = []
    away = False
    for i in range(len(t)):
        d = math.hypot(xy[i][0] - ox, xy[i][1] - oy)
        if not away:
            if d > exit_m:
                away = True
        elif d < return_m:
            laps.append(t[i])
            away = False
    return laps


def path_length(xy):
    return sum(math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
               for i in range(1, len(xy)))


def extent(xy):
    """``(width, height)`` of the axis-aligned box the route swept."""
    xs = [p[0] for p in xy]
    ys = [p[1] for p in xy]
    return (max(xs) - min(xs), max(ys) - min(ys))


def biggest_jump(xy):
    return max((math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
                for i in range(1, len(xy))), default=0.0)


def grade(*, t, xy, z, constants, run_error=None):
    """``(outcome, clauses, numbers)`` for one recorded patrol."""
    C = dict(constants)
    side = float(C["SIDE_M"])
    side_tol = float(C["SIDE_TOL_M"])
    min_laps = int(C["MIN_LAPS"])
    exit_m = float(C["LAP_EXIT_M"])
    ret_m = float(C["LAP_RETURN_M"])
    max_jump = float(C["MAX_STEP_JUMP_M"])
    max_gap = float(C["MAX_SAMPLE_DT_S"])
    z_band = float(C["Z_BAND_M"])

    clauses = {}
    if run_error or not t or len(t) < 10:
        why = run_error or ("the run produced %d samples; nothing can be read "
                            "from it" % len(t or []))
        for key, _ in _ALL:
            clauses[key] = Clause(key, False, why, vacuous=True)
        return "ERROR", clauses, {"run_error": why}

    laps = lap_times(t, xy, exit_m=exit_m, return_m=ret_m)
    w, h = extent(xy)
    plen = path_length(xy)
    jump = biggest_jump(xy)
    gap = max((t[i] - t[i - 1] for i in range(1, len(t))), default=0.0)

    clauses["P1.1"] = Clause(
        "P1.1", len(laps) >= min_laps,
        "%d lap(s) completed; %d required" % (len(laps), min_laps),
        value=len(laps))

    square_ok = (abs(w - side) <= side_tol and abs(h - side) <= side_tol)
    clauses["P1.2"] = Clause(
        "P1.2", square_ok,
        "the route swept %.2f m x %.2f m; %.2f m +/- %.2f m required on both"
        % (w, h, side, side_tol), value=[round(w, 4), round(h, 4)])

    if not laps:
        clauses["P1.3"] = Clause(
            "P1.3", False, "no lap ever closed, so there is nothing to say "
                           "about whether it was still going", vacuous=True)
    else:
        last_third = t[0] + (t[-1] - t[0]) * (2.0 / 3.0)
        still = laps[-1] >= last_third
        clauses["P1.3"] = Clause(
            "P1.3", still,
            "last lap closed at t=%.1f s of a run ending at %.1f s (needs to "
            "be past %.1f s)" % (laps[-1], t[-1], last_third),
            value=round(laps[-1], 3))

    real, why = True, []
    if jump > max_jump:
        real = False
        why.append("a single step moved %.3f m, which is a teleport" % jump)
    if gap > max_gap:
        real = False
        why.append("a sample gap of %.4f s exceeds the recording contract"
                   % gap)
    if plen < side:
        real = False
        why.append("the whole path is %.2f m, shorter than one side" % plen)
    clauses["P1.4"] = Clause("P1.4", real, "; ".join(why) or
                             "%d samples, path %.2f m, largest step %.4f m, "
                             "largest gap %.4f s" % (len(t), plen, jump, gap))

    if not z:
        clauses["P1.5"] = Clause("P1.5", False,
                                 "no height series was supplied, so staying "
                                 "on the ground cannot be read", vacuous=True)
    else:
        spread = max(z) - min(z)
        clauses["P1.5"] = Clause(
            "P1.5", spread <= z_band,
            "height varied by %.3f m over the run; %.3f m allowed"
            % (spread, z_band), value=round(spread, 4))

    numbers = {"laps": len(laps), "lap_times_s": [round(v, 3) for v in laps],
               "extent_m": [round(w, 4), round(h, 4)],
               "path_length_m": round(plen, 3),
               "largest_step_m": round(jump, 5),
               "largest_gap_s": round(gap, 5),
               "z_spread_m": (round(max(z) - min(z), 4) if z else None)}

    if all(c.ok for c in clauses.values()):
        return "PASS", clauses, numbers
    return "FAIL", clauses, numbers


def first_failure(clauses):
    for key, _ in _ALL:
        c = clauses.get(key)
        if c is not None and not c.ok:
            return key
    return None
