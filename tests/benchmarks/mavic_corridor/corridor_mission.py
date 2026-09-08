#!/usr/bin/env python3
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
"""corridor_mission.py - fly the drone-gcs corridor mission in OmniSim.

The OmniSim half of the comparison whose kinematic half is baseline_metrics.py.
Clearance is computed with that module's own `dist_to_box` and `SHELF_BOXES`, so
both sides of the comparison share one definition rather than two that agree by
inspection.

The drone is commanded through the eight corner points rather than the 115
densified checkpoints the baseline uses. The baseline densifies because its
proportional controller needs frequent correction; `goto_waypoint` flies a leg
autonomously, so issuing 115 blocking calls would stop-and-start the aircraft
115 times and the resulting travel time would measure this script's loop rather
than the flight. Every metric here is derived from the sampled trajectory, not
from the command list, and the command list is identical across all conditions.

Prerequisites: the world loaded with OMNISIM_URDF_USE_SENSORS=1 and the
mavic_omnilink_bridge listening on 6090.

    python corridor_mission.py --condition none --out mission_none.json
"""

import argparse
import json
import math
import threading
import time
import urllib.error
import urllib.request

from baseline_metrics import (SHELF_BOXES, BLOCKING_BOX, dist_to_box, LANE_A,
                              LANE_B, LANE_C, LANE_D, Y_LO, Y_HI, Z_FIJO)

BRIDGE = "http://127.0.0.1:6090"
SAMPLE_PERIOD_S = 0.05
GOTO_TIMEOUT_S = 60

# Same eight corners the baseline's aisle route is built from.
ROUTE = [(LANE_A, Y_LO), (LANE_A, Y_HI),
         (LANE_B, Y_HI), (LANE_B, Y_LO),
         (LANE_C, Y_LO), (LANE_C, Y_HI),
         (LANE_D, Y_HI), (LANE_D, Y_LO)]

# The Mavic's authored start pose, and what actually happens to it.
#
# On v8.1.17 the parked aircraft slid along its own +x body axis at ~0.075 m per
# second of simulated time, so a fixed pre-flight delay started every flight
# somewhere different. v8.3.0 fixes the creep with a per-world newtonGroundMu of
# 1.5 (a world-level declaration, not a new default: the field still defaults to
# unset, which runs at 1.0, and 1.0 creeps exactly as unset does).
#
# What remains is a one-off spawn displacement. Measured on v8.3.0: the aircraft
# is ejected ~0.48 m in +y within the first second and then parks, byte-identical
# from sim_time 13 to 96. So it no longer starts somewhere different each time,
# it starts somewhere OTHER than the authored pose, consistently.
#
# The property a campaign needs is repeatability, not obedience to the world
# file, so the guard tests that: the pose must be static, and the pose actually
# used is recorded per flight so any drift between flights shows up in the data
# instead of being assumed away. The loose absolute bound only catches a gross
# failure such as the aircraft having fallen through the floor.
WORLD_START = (-0.5, -0.5, 0.1)
START_TOL_M = 0.15
SETTLE_TOL_M = 0.01
SETTLE_WAIT_S = 1.0
SETTLE_TIMEOUT_S = 45.0

GOAL_SHIFT_INDEX = len(ROUTE) // 2
GOAL_SHIFT_VEC = (0.40, 0.40)


def _get(path, timeout=10):
    with urllib.request.urlopen(BRIDGE + path, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def _post(action, timeout=GOTO_TIMEOUT_S + 10, **params):
    body = json.dumps(dict(action=action, **params)).encode("utf-8")
    req = urllib.request.Request(BRIDGE + "/action", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def wait_for_bridge(seconds=90):
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            return _get("/state", timeout=3)
        except (urllib.error.URLError, OSError, TimeoutError):
            time.sleep(1.0)
    raise SystemExit("bridge did not answer on %s within %ds" % (BRIDGE, seconds))


class Sampler(threading.Thread):
    """Polls /state on a fixed cadence; the trajectory is what it collects."""

    def __init__(self):
        super().__init__(daemon=True)
        self.samples = []
        self._stop_evt = threading.Event()

    def run(self):
        while not self._stop_evt.is_set():
            try:
                s = _get("/state", timeout=3)
                self.samples.append((s["sim_time"], s["x"], s["y"], s["z"],
                                     s["mode"], s["fault"], s.get("yaw", 0.0)))
            except Exception:
                pass
            time.sleep(SAMPLE_PERIOD_S)

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=5)


# The Mavic's collision geometry in the shipped URDF is one box, 0.30 x 0.08 x
# 0.05 m at the body origin; the propeller links carry no collider on purpose.
# So the aircraft is NOT a point, and its footprint half-extent against a wall
# is anywhere between 0.04 m and 0.15 m depending on heading. Distance measured
# from the body origin -- which is what this script reported until now -- is not
# clearance and cannot detect a collision: the origin never enters the
# obstacle even while the airframe is pressed against it. Measured case: the
# aircraft sat at 0.084 m from the blocker for 1.4 s, stationary, then slid
# around the corner. That is contact, and it scored as a clean flight.
HULL_L, HULL_W = 0.30, 0.08


def hull_corners(x, y, yaw):
    """The four corners of the collider footprint, in world coordinates."""
    c, s_ = math.cos(yaw), math.sin(yaw)
    hl, hw = HULL_L / 2.0, HULL_W / 2.0
    return [(x + c * dx - s_ * dy, y + s_ * dx + c * dy)
            for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def _seg_point_dist(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _poly_overlap(p, q):
    """Separating-axis test for two convex polygons."""
    for poly in (p, q):
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            nx, ny = -(by - ay), bx - ax
            pp = [nx * vx + ny * vy for vx, vy in p]
            qq = [nx * vx + ny * vy for vx, vy in q]
            if max(pp) < min(qq) or max(qq) < min(pp):
                return False
    return True


def hull_clearance(x, y, yaw, box):
    """Distance from the aircraft footprint to an axis-aligned box.

    Zero when they overlap -- that is a collision, not a near miss.
    """
    x0, y0, x1, y1 = box
    bpoly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    hpoly = hull_corners(x, y, yaw)
    if _poly_overlap(hpoly, bpoly):
        return 0.0
    d = min(_seg_point_dist(v, bpoly[i], bpoly[(i + 1) % 4])
            for v in hpoly for i in range(4))
    d = min(d, min(_seg_point_dist(v, hpoly[i], hpoly[(i + 1) % 4])
                   for v in bpoly for i in range(4)))
    return d


def obstacle_boxes(condition):
    """The footprints clearance is measured against, for this condition.

    The blocker has to be in here for the `blocked` runs or the condition is
    unmeasurable: clearance would report the distance to the shelves, which the
    obstacle does not change, and `collided` -- derived from the same set --
    could never register a hit on the one body the condition adds. Same box the
    baseline uses, so both sides of the comparison score the obstacle alike.
    """
    if condition == "blocked":
        return SHELF_BOXES + [BLOCKING_BOX["aisles"]]
    return SHELF_BOXES


AIRBORNE_Z = 0.5


def metrics(samples, arrivals, faults, boxes):
    """Route metrics over the mission window; contact over the whole flight.

    Everything before the first waypoint arrival is takeoff and pre-flight
    settling, and everything after the last is the landing. Including them made
    path length count ground drift and made the altitude error report the
    aircraft's descent -- neither is the flight being measured.
    """
    all_samples = samples
    if arrivals:
        t_start = arrivals[0]["sim_time"]
        t_end = arrivals[-1]["sim_time"]
        samples = [s for s in samples if t_start <= s[0] <= t_end]

    xy = [(s[1], s[2]) for s in samples]
    path = sum(math.dist(xy[i], xy[i + 1]) for i in range(len(xy) - 1))

    # Point clearance keeps the like-for-like comparison with the kinematic
    # baseline, which is itself a point mass. Hull clearance is the physical
    # one, and the only one a collision can be read off.
    clearances = [min(dist_to_box(p, b) for b in boxes) for p in xy]
    min_clear = min(clearances) if clearances else float("nan")

    # Contact is counted over the AIRBORNE samples of the whole flight, not
    # over the mission window. The window runs first arrival -> last arrival, so
    # an aircraft that wedges itself against an obstacle after its last
    # successful waypoint has the entire wedged period outside it: measured, two
    # flights that spent 213 s and 40 s pressed into the blocker and failed the
    # mission both scored ZERO contact samples. The window is right for route
    # metrics (time, path) and wrong for contact.
    #
    # The airborne filter then separates an in-flight collision from a downed
    # aircraft resting against a wall -- one of those two flights was on the
    # ground at z~=0.19 for almost all of its 4255 contact samples, which is a
    # crash outcome, not a flight collision.
    air = [s for s in all_samples if s[3] > AIRBORNE_Z]
    hull = [min(hull_clearance(s[1], s[2], s[6], b) for b in boxes) for s in air]
    min_hull = min(hull) if hull else float("nan")
    collision_samples = sum(1 for c in hull if c <= 0.0)

    alt_err = [abs(s[3] - Z_FIJO) for s in samples]

    t0 = arrivals[0]["sim_time"] if arrivals else None
    t1 = arrivals[-1]["sim_time"] if arrivals else None

    return {
        "completed": len(arrivals) == len(ROUTE) and not faults,
        "waypoints_arrived": len(arrivals),
        "waypoints_total": len(ROUTE),
        "travel_time_s": (t1 - t0) if (t0 is not None and t1 is not None) else None,
        "path_length_m": path,
        "min_clearance_m": min_clear,
        "min_hull_clearance_m": min_hull,
        "collision_samples": collision_samples,
        "collided": collision_samples > 0,
        "altitude_error_max_m": max(alt_err) if alt_err else None,
        "altitude_error_mean_m": (sum(alt_err) / len(alt_err)) if alt_err else None,
        "n_samples": len(samples),
        "n_airborne_samples": len(air),
        "faults": faults,
    }


def fly_once(condition, altitude, rep):
    """One flight. Returns (metrics, trajectory)."""
    sampler = Sampler()
    sampler.start()
    faults, arrivals = [], []
    try:
        _post("takeoff", altitude=altitude)
        time.sleep(6.0)

        for i, (x, y) in enumerate(ROUTE):
            cx, cy = x, y
            if condition == "goal_shift" and i == GOAL_SHIFT_INDEX:
                cx, cy = x + GOAL_SHIFT_VEC[0], y + GOAL_SHIFT_VEC[1]

            r = _post("goto_waypoint", x=cx, y=cy, altitude=altitude,
                      wait=True, timeout_s=GOTO_TIMEOUT_S)
            st = _get("/state")
            reached = r.get("status") == "ok" and not r.get("fault")
            # The goto response carries the fault that ended the leg. /state
            # only carries whatever is set at the instant it is polled, and
            # v8.3.0's `no_progress` is advisory and self-clearing (the bridge
            # reports a stall and keeps flying, by design), so reading /state
            # alone silently discarded every stall this campaign produced.
            leg_fault = r.get("fault") or st.get("fault")
            if leg_fault:
                faults.append({"waypoint": i, "fault": leg_fault,
                               "from": "goto" if r.get("fault") else "state"})
            if reached:
                arrivals.append({"waypoint": i, "sim_time": st["sim_time"],
                                 "x": st["x"], "y": st["y"], "z": st["z"]})

        _post("land")
        time.sleep(3.0)
    finally:
        sampler.stop()

    m = metrics(sampler.samples, arrivals, faults,
                obstacle_boxes(condition))
    print("  rep %2d  arrived %d/%d  t=%6.2f  path=%6.2f  pt=%.4f  hull=%.4f  "
          "coll=%s (%d samples)"
          % (rep + 1, m["waypoints_arrived"], m["waypoints_total"],
             m["travel_time_s"] or float("nan"), m["path_length_m"],
             m["min_clearance_m"], m["min_hull_clearance_m"], m["collided"],
             m["collision_samples"]))
    return m, [{"t": s[0], "x": s[1], "y": s[2], "z": s[3], "yaw": s[6],
                "mode": s[4], "fault": s[5]} for s in sampler.samples]


def summarise(runs):
    def stat(key):
        v = [r[key] for r in runs if isinstance(r[key], float) and r[key] == r[key]]
        if not v:
            return None
        n = len(v)
        mean = sum(v) / n
        var = sum((x - mean) ** 2 for x in v) / n
        return {"mean": mean, "std": var ** 0.5, "min": min(v), "max": max(v), "n": n}
    n = len(runs)
    return {
        "n_runs": n,
        "completion_rate": sum(1 for r in runs if r["completed"]) / n,
        "collision_rate": sum(1 for r in runs if r["collided"]) / n,
        "travel_time_s": stat("travel_time_s"),
        "path_length_m": stat("path_length_m"),
        "min_clearance_m": stat("min_clearance_m"),
        "min_hull_clearance_m": stat("min_hull_clearance_m"),
        "altitude_error_max_m": stat("altitude_error_max_m"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--condition", choices=["none", "blocked", "goal_shift"],
                    default="none",
                    help="'blocked' needs the obstacle authored into the world; "
                         "this script only records which condition was flown")
    ap.add_argument("--altitude", type=float, default=Z_FIJO)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--keep-trajectories", action="store_true")
    ap.add_argument("--out", default="mission.json")
    args = ap.parse_args()

    print("waiting for the bridge on %s ..." % BRIDGE)
    s0 = wait_for_bridge()
    print("  found it: mode=%s at (%.3f, %.3f, %.3f)"
          % (s0["mode"], s0["x"], s0["y"], s0["z"]))
    # The aircraft is ejected ~0.48 m from its authored pose within the first
    # second and then parks. That transient is deterministic, so the right thing
    # is to wait it out and then confirm the aircraft is static -- not to connect
    # as early as possible, which was the correct response to the v8.1.17 creep
    # and is the wrong one now that the creep is fixed.
    prev, settled, moved = s0, None, None
    deadline = time.time() + SETTLE_TIMEOUT_S
    while time.time() < deadline:
        time.sleep(SETTLE_WAIT_S)
        cur = _get("/state")
        moved = math.dist((prev["x"], prev["y"], prev["z"]),
                          (cur["x"], cur["y"], cur["z"]))
        if moved <= SETTLE_TOL_M:
            settled = cur
            break
        prev = cur
    if settled is None:
        raise SystemExit(
            "NEVER SETTLED: still moving %.4f m/s after %.0f s, last pose "
            "(%.3f, %.3f, %.3f). The aircraft is drifting, not parking."
            % (moved, SETTLE_TIMEOUT_S, prev["x"], prev["y"], prev["z"]))

    off = math.dist((settled["x"], settled["y"], settled["z"]), WORLD_START)
    if off > START_TOL_M:
        raise SystemExit(
            "START POSE: settled at (%.3f, %.3f, %.3f), %.3f m from the authored "
            "start. On a build with the gimbal-ramp fix the aircraft parks where "
            "the world says; a large offset here means the init step is throwing "
            "it again, and this flight would not start where the others did."
            % (settled["x"], settled["y"], settled["z"], off))
    start_pose = (settled["x"], settled["y"], settled["z"])
    print("  settled at (%.3f, %.3f, %.3f) after %.1f s, %.3f m from the "
          "authored start" % (settled["x"], settled["y"], settled["z"],
                              settled["sim_time"], off))

    print("condition=%s  altitude=%.2f  repeats=%d"
          % (args.condition, args.altitude, args.repeat))
    print("")

    runs, trajectories = [], []
    for rep in range(args.repeat):
        if rep:
            # reset teleports the aircraft back to its authored start pose, so
            # each flight begins from the same state without an engine restart.
            _post("reset")
            time.sleep(5.0)
        m, traj = fly_once(args.condition, args.altitude, rep)
        runs.append(m)
        if args.keep_trajectories:
            trajectories.append(traj)

    summary = summarise(runs)
    report = {"start_pose": start_pose, "condition": args.condition, "altitude_m": args.altitude,
              "route": ROUTE, "sample_period_s": SAMPLE_PERIOD_S,
              "summary": summary, "runs": runs}
    if args.keep_trajectories:
        report["trajectories"] = trajectories
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=1)

    print("")
    print("--- %s, %d runs ---" % (args.condition, summary["n_runs"]))
    print("completion rate   %.0f%%" % (summary["completion_rate"] * 100))
    print("collision rate    %.0f%%" % (summary["collision_rate"] * 100))
    for k in ("travel_time_s", "path_length_m", "min_clearance_m",
              "altitude_error_max_m"):
        st = summary[k]
        if st:
            print("%-18s mean %8.4f  sd %7.4f  min %8.4f  max %8.4f"
                  % (k, st["mean"], st["std"], st["min"], st["max"]))
    print("")
    print("wrote %s" % args.out)


if __name__ == "__main__":
    main()
