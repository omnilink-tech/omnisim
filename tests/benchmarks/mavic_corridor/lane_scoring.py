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

"""Lane-offset scoring for a kept corridor trajectory.

This measures how far off its intended lane the aircraft flies, and how long it
takes to come back after a disturbance. It exists because the third failure
signature of the corridor benchmark cannot be read off the summary metrics: a
contact that is survived locally delays lane recovery, and an obstacle corner
sitting inside the recovery distance is what collects. Completion stays 8/8 and
minimum clearance moves only at that one corner, so nothing in the summary says
which of the two happened.

Everything here runs on samples already in the mission JSON (written by
`--keep-trajectories`), so a question about a past campaign is answered by
re-scoring rather than re-flying.

Three rules decide every number, and all three are in this module rather than in
prose because each has already produced a disagreement when left implicit.

**Leg assignment** is forward-only nearest segment by clamped point-to-segment
distance. The obvious alternative, advancing when the projection parameter on the
current leg exceeds 1, does not work on a serpentine: on a leg perpendicular to
the direction of travel that parameter does not grow, so the index sticks and a
sample 9 m downstream gets scored as an excursion on a leg the aircraft left long
ago.

**The station anchor.** A station is a distance ALONG the leg from its start
waypoint, not an absolute coordinate, so the same station means the same thing on
every leg whatever direction it runs. The reading is the first sample at or past
that distance after the leg's anchor, where the anchor is the sample of closest
approach to the start waypoint. Without an anchor the reading is ambiguous rather
than merely noisy: an aircraft that rounds a corner wide crosses a near station
three times, outbound, back, and once more settling, and on one real flight those
three readings differed by 2.5 m. The anchor selects the outbound pass and
nothing else.

**In-turn stations return None.** If the anchor already lies at or past the
station, the aircraft was still turning when it passed there, and a number would
describe how wide the corner was taken rather than how fast the lane was
recovered. Those are different quantities, and mixing them is how a station that
looked informative on one machine turned out to be measuring the turn on another.
None is the honest answer; `station_table` reports it as "in-turn".

Offsets are signed, positive to the LEFT of the direction of travel, so the mean
says which side of the lane the aircraft flew and not only how far off it was.
"""

import json
import math

# Corner exclusion for the per-leg statistics: a turn is not an excursion.
CORNER_SKIP_M = 0.6

# At or below this the aircraft is on the ground: taxiing, settling or fallen.
AIRBORNE_Z = 0.5


def legs_of(route):
    """Consecutive waypoint pairs. -> [((ax, ay), (bx, by)), ...]"""
    return list(zip(route[:-1], route[1:]))


def cross_track(p, a, b):
    """Signed offset of p from the line a->b, positive to the left of travel.

    Left of a direction (dx, dy) is (-dy, dx), which in the ENU frame the worlds
    use is the port side.
    """
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    return (-(p[0] - ax) * dy + (p[1] - ay) * dx) / length


def along_track(p, a, b):
    """Distance of p along the leg from a, in metres, unclamped."""
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    return ((p[0] - ax) * dx + (p[1] - ay) * dy) / length


def _seg_dist(p, a, b):
    """Clamped point-to-segment distance."""
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 == 0.0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def airborne(samples):
    return [s for s in samples if s["z"] > AIRBORNE_Z]


def assign_legs(samples, route):
    """Which leg each airborne sample belongs to.

    -> [(leg_index, sample), ...] in sample order. Forward-only: once a sample is
    nearest leg k, no later sample is scored against a leg before k, so the
    parallel lanes of a serpentine cannot claim each other's samples.
    """
    legs = legs_of(route)
    out = []
    leg = 0
    for s in airborne(samples):
        p = (s["x"], s["y"])
        best, best_i = float("inf"), leg
        for i in range(leg, len(legs)):
            d = _seg_dist(p, *legs[i])
            if d < best:
                best, best_i = d, i
        leg = best_i
        out.append((leg, s))
    return out


def leg_offsets(samples, route, corner_skip_m=CORNER_SKIP_M):
    """Per-leg signed cross-track with the corners excluded.

    -> {leg_index: [(t, offset_m), ...]}
    """
    legs = legs_of(route)
    out = {}
    for leg, s in assign_legs(samples, route):
        a, b = legs[leg]
        p = (s["x"], s["y"])
        if math.dist(p, a) < corner_skip_m or math.dist(p, b) < corner_skip_m:
            continue
        out.setdefault(leg, []).append((s["t"], cross_track(p, a, b)))
    return out


def leg_summary(samples, route, corner_skip_m=CORNER_SKIP_M):
    """-> {leg_index: {n, mean_abs_m, max_abs_m, mean_signed_m}}

    `max_abs_m` keeps its sign: it is the furthest excursion, reported on the
    side it happened, because which side of the lane it was on is the whole
    question when an obstacle sits on one of them.
    """
    out = {}
    for leg, vals in leg_offsets(samples, route, corner_skip_m).items():
        offs = [o for _, o in vals]
        out[leg] = {
            "n": len(offs),
            "mean_abs_m": sum(abs(o) for o in offs) / len(offs),
            "max_abs_m": max(offs, key=abs),
            "mean_signed_m": sum(offs) / len(offs),
        }
    return out


def leg_anchor(samples, route, leg):
    """The sample of closest approach to this leg's start waypoint.

    -> (sample, along_track_m), or None when the flight never reached the leg.
    """
    legs = legs_of(route)
    a, b = legs[leg]
    mine = [s for i, s in assign_legs(samples, route) if i == leg]
    if not mine:
        return None
    s = min(mine, key=lambda s: math.dist((s["x"], s["y"]), a))
    return s, along_track((s["x"], s["y"]), a, b)


def _at_station(prev, cur, station_m):
    """Cross-track exactly AT the station, from the two samples bracketing it.

    `offset_m` is read at a real sample, and a real sample sits a fraction of a
    sample period PAST the station. On an aircraft recovering toward its lane
    that is always the low side, so the sample reading carries a one-directional
    bias that grows with the sample period: on the twelve v8.4.0 flights measured
    here it runs 0.002-0.024 m, and at a quarter of the sample rate one reading
    moved by 0.065 m and another changed sign. Interpolating removes the
    dependence, so a band published from one campaign can be compared with a
    campaign sampled at a different rate. Both are reported: `offset_m` describes
    the sample it names, this one describes the station.
    """
    if prev is None:
        return cur[1]                    # crossed on the first sample: no bracket
    s0, o0 = prev
    s1, o1 = cur
    if s1 <= s0:
        return o1
    return o0 + (station_m - s0) / (s1 - s0) * (o1 - o0)


def station(samples, route, leg, station_m):
    """Cross-track at one station on one leg, or None.

    `station_m` is measured along the leg from its start waypoint. None means the
    reading does not exist: the flight never reached the leg, or never got as far
    as the station, or the station is inside the turn. See the module docstring
    for why the last of those is not a number.

    `offset_m` is the reading at the first sample past the station and `t`, `x`,
    `y`, `along_m` describe that sample. `offset_interp_m` is the same quantity
    interpolated onto the station itself; see `_at_station` for which to quote.
    """
    legs = legs_of(route)
    a, b = legs[leg]
    anchored = leg_anchor(samples, route, leg)
    if anchored is None:
        return None
    anchor, anchor_s = anchored
    if anchor_s >= station_m:
        return None                      # in-turn: a number here is the turn
    prev = None
    for i, s in assign_legs(samples, route):
        if i != leg or s["t"] < anchor["t"]:
            continue
        p = (s["x"], s["y"])
        reached = along_track(p, a, b)
        offset = cross_track(p, a, b)
        if reached >= station_m:
            return {"t": s["t"], "x": s["x"], "y": s["y"],
                    "offset_m": offset, "along_m": reached,
                    "offset_interp_m": _at_station(prev, (reached, offset),
                                                   station_m)}
        prev = (reached, offset)
    return None


def station_table(samples, route, leg, stations_m):
    """-> {station_m: reading dict, or "in-turn", or "not reached"}"""
    anchored = leg_anchor(samples, route, leg)
    out = {}
    for sm in stations_m:
        if anchored is None:
            out[sm] = "not reached"
            continue
        r = station(samples, route, leg, sm)
        if r is not None:
            out[sm] = r
        else:
            out[sm] = "in-turn" if anchored[1] >= sm else "not reached"
    return out


def score_run(samples, route, leg=None, stations_m=()):
    """Everything this module measures for one flight, ready to serialise."""
    out = {"legs": leg_summary(samples, route)}
    if leg is not None and stations_m:
        out["stations"] = {
            "leg": leg,
            "readings": station_table(samples, route, leg, stations_m),
        }
    return out


def _cli():
    import argparse
    ap = argparse.ArgumentParser(description="Re-score kept corridor trajectories.")
    ap.add_argument("report", nargs="+",
                    help="mission JSON written with --keep-trajectories")
    ap.add_argument("--leg", type=int, default=None,
                    help="0-based leg index for the station table")
    ap.add_argument("--stations", type=float, nargs="*", default=(),
                    help="metres along that leg from its start waypoint")
    args = ap.parse_args()

    for path in args.report:
        with open(path) as f:
            rep = json.load(f)
        route = [tuple(p) for p in rep["route"]]
        trajectories = rep.get("trajectories") or []
        if not trajectories:
            print("%s: no trajectories, re-run with --keep-trajectories" % path)
            continue
        for n, traj in enumerate(trajectories, 1):
            print("%s rep %d  condition=%s" % (path, n, rep.get("condition")))
            scored = score_run(traj, route, args.leg, args.stations)
            for leg in sorted(scored["legs"]):
                st = scored["legs"][leg]
                print("  leg %d  n=%4d  mean|off| %6.3f  max %+6.3f  mean %+6.3f"
                      % (leg + 1, st["n"], st["mean_abs_m"], st["max_abs_m"],
                         st["mean_signed_m"]))
            for sm, r in scored.get("stations", {}).get("readings", {}).items():
                if isinstance(r, str):
                    print("  station %.2f m on leg %d: %s" % (sm, args.leg + 1, r))
                else:
                    print("  station %.2f m on leg %d: offset %+6.3f at "
                          "(%6.2f,%6.2f) t=%6.2f"
                          % (sm, args.leg + 1, r["offset_m"], r["x"], r["y"],
                             r["t"]))


if __name__ == "__main__":
    _cli()
