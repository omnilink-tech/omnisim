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

"""Measure the warehouse_omnilink production line — the BASELINE harness.

Why this exists
---------------
`WAREHOUSE_OMNILINK.md` states that the arm waits 40-65 s per cycle for an
empty cart, that tug_a's round trip is ~100 s against a ~45 s fill, and that
the line runs ~0.95-1.0x realtime. Those are the right facts to know and there
was no way to *re-derive* them, so "we optimised the warehouse demo" was not a
falsifiable claim. This script makes it one: run it before a change, run it
after, diff the JSON.

What it does
------------
Polls the three bridges' READ-ONLY state endpoints over loopback HTTP:

    GET http://127.0.0.1:8765/state    OMNIARM6  — carries the `line` block
    GET http://127.0.0.1:8766/state    tug_a  — dispatch tug
    GET http://127.0.0.1:8767/state    tug_b  — return tug

and derives throughput, cycle time, arm idle-wait, per-stage durations and
per-tug motion (path length, rotation, stationary time).

Two hard rules this file keeps
------------------------------
1. **GET only, never POST.** A POST to a bridge — *any* POST outside its
   read allowlist — calls `note_external_command` and pauses that robot's
   idle loop for ~60 s. A measurement tool that pauses the thing it measures
   is worthless. Every request here is a GET, and GET never arms the pause
   (`_route_get` in both bridges does not touch `last_external_command`).
2. **It writes exactly one file: `--out`.** It never opens a `.wbt`, a
   controller, a log or anything else in the tree.

Usage
-----
    python tests/benchmarks/warehouse/measure_line.py --duration 600 \
        --out tests/benchmarks/warehouse/results/baseline.json

Exit codes
----------
    0  measured cleanly
    2  a bridge did not answer preflight (or is not the warehouse world)
    3  a bridge stopped answering mid-run (partial JSON still written)
    4  bad arguments
    5  --min-cycles not met (the run was too short to conclude anything)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence, Tuple

SCHEMA = "omnisim.warehouse.measure_line/1"

TAU = 2.0 * math.pi

# The five states the front box passes through, in order, as published by
# WarehouseLine._publish() -> /state.line.fill_state. "loaded" is set in the
# same lock block that pops the box off the queue, so it is essentially never
# observable as `fill_state`; the box reappears in `in_transit` instead.
FILL_STATES = ("belt", "at_fill", "filled", "loading", "loaded")

# Coarse phases the arm's idle pick loop publishes as idle_loop.leg.
ARM_LEGS = ("idle", "pick", "place", "respawn")

# Coarse phases the tug idle loops publish as idle_loop.leg.
TUG_LEGS = ("idle", "docking", "holding", "to_park", "to_collect", "to_dock_e",
            "back_aisle", "returning", "lane_fetch", "lane_conveyor",
            "stage_to_station", "shuttle_in", "shuttle_out")


# ══════════════════════════════════════════════════════════════════════
# Pure helpers.  No I/O, no globals — unit-tested in test_measure_line.py.
# ══════════════════════════════════════════════════════════════════════

def wrap_pi(a: float) -> float:
    """Wrap an angle into (-pi, +pi]."""
    return (a + math.pi) % TAU - math.pi


def yaw_step(prev: float, cur: float) -> float:
    """The signed rotation from `prev` to `cur`, in radians.

    THIS IS THE FUNCTION THE WHOLE ROTATION METRIC RESTS ON.

    Both bridges publish `yaw` already wrapped into (-pi, pi] (their
    `_read_pose` returns `wrap_pi(yaw - yaw_offset)`), so a tug turning
    steadily through north emits e.g. ... 3.10, 3.14, -3.14, -3.10 ... A naive
    `cur - prev` scores that single 0.08 rad step as -6.20 rad — 355 degrees of
    phantom rotation, once per crossing. Wrapping the DIFFERENCE instead of the
    angles recovers the true step.

    The assumption this makes, stated plainly: the true rotation between two
    consecutive samples must be less than pi. Above that, direction is
    unrecoverable from position alone (a +190 deg turn and a -170 deg turn
    produce the identical sample pair) and the metric silently under-reports.
    The OMNITUG500's ceiling is v_max * r / ht = 10.0 * 0.10 / 0.30 = 3.33 rad/s,
    so the poll rate must exceed 3.33/pi = 1.06 Hz for the bound to hold at
    full yaw rate; the 2 Hz default clears it by 1.9x. `nyquist_margin` in the
    output reports that ratio for the rate you actually achieved, and
    `aliasing_suspect_steps` counts the individual steps that came close.
    """
    return wrap_pi(cur - prev)


def unwrap_series(yaws: Sequence[float]) -> List[float]:
    """Wrapped angles -> a continuous series with the 2*pi jumps removed.

    unwrap_series([...])[-1] - [0] is the NET rotation (signed, unbounded).
    """
    if not yaws:
        return []
    out = [float(yaws[0])]
    for i in range(1, len(yaws)):
        out.append(out[-1] + yaw_step(yaws[i - 1], yaws[i]))
    return out


def integrate_rotation(yaws: Sequence[float],
                       deadband_rad: float = 3.5e-4) -> Dict[str, Any]:
    """Total and net rotation from a series of WRAPPED yaw samples.

    `total_rad` is the sum of |step| — every degree the tug turned, in either
    direction. `net_rad` is the signed sum — where it ended up. A tug that
    spins 90 deg left then 90 deg right scores total=180, net=0, and the gap
    between them is exactly the "unnecessary rotation" this project wants to
    drive down.

    `deadband_rad` suppresses per-sample float noise so a tug standing still
    for ten minutes does not accumulate a fake few degrees. It is applied to
    the ABSOLUTE sum only; `total_rad_raw` is the undeadbanded figure, so the
    noise floor stays visible instead of being silently chosen for you. Net
    rotation is never deadbanded — dropping steps from a signed sum would bias
    it.
    """
    steps = [yaw_step(yaws[i - 1], yaws[i]) for i in range(1, len(yaws))]
    if not steps:
        return {"total_rad": 0.0, "total_rad_raw": 0.0, "net_rad": 0.0,
                "max_step_rad": 0.0, "aliasing_suspect_steps": 0, "steps": 0}
    total = sum(abs(s) for s in steps if abs(s) >= deadband_rad)
    return {
        "total_rad": total,
        "total_rad_raw": sum(abs(s) for s in steps),
        "net_rad": sum(steps),
        "max_step_rad": max(abs(s) for s in steps),
        # A step past pi/2 is not wrong, but it is within 2x of the point where
        # wrapping stops being able to tell +x from -(2pi-x). Counted, not
        # hidden.
        "aliasing_suspect_steps": sum(1 for s in steps if abs(s) > math.pi / 2),
        "steps": len(steps),
    }


def integrate_path(xy: Sequence[Tuple[float, float]],
                   deadband_m: float = 1e-3) -> Dict[str, Any]:
    """Path length and net displacement from a series of (x, y) samples.

    `path_m` is the distance actually travelled; `net_m` is the straight line
    from first sample to last. Their ratio is the route-efficiency signal.
    Same deadband contract as `integrate_rotation`.
    """
    if len(xy) < 2:
        return {"path_m": 0.0, "path_m_raw": 0.0, "net_m": 0.0,
                "max_step_m": 0.0, "steps": 0}
    steps = [math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
             for i in range(1, len(xy))]
    return {
        "path_m": sum(s for s in steps if s >= deadband_m),
        "path_m_raw": sum(steps),
        "net_m": math.hypot(xy[-1][0] - xy[0][0], xy[-1][1] - xy[0][1]),
        "max_step_m": max(steps),
        "steps": len(steps),
    }


def percentile(values: Sequence[float], p: float) -> Optional[float]:
    """Linear-interpolated percentile (p in 0..100). None for an empty input.

    Deliberately hand-rolled rather than `statistics.quantiles` so the method
    is pinned: with n<20 a p95 is barely distinguishable from the max, and the
    summary prints n beside it so nobody reads more into it than is there.
    """
    if not values:
        return None
    s = sorted(values)
    if len(s) == 1:
        return float(s[0])
    k = (len(s) - 1) * (p / 100.0)
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return float(s[int(k)])
    return float(s[lo] + (s[hi] - s[lo]) * (k - lo))


def describe(values: Sequence[float]) -> Dict[str, Any]:
    """n / mean / median / p95 / min / max / total for a duration sample."""
    v = [float(x) for x in values]
    if not v:
        return {"n": 0, "mean": None, "median": None, "p95": None,
                "min": None, "max": None, "total": 0.0}
    return {
        "n": len(v),
        "mean": statistics.fmean(v),
        "median": statistics.median(v),
        "p95": percentile(v, 95.0),
        "min": min(v),
        "max": max(v),
        "total": sum(v),
    }


def segment_runs(series: Sequence[Tuple[float, Any]]) -> List[Dict[str, Any]]:
    """(t, key) samples -> contiguous runs of equal key.

    A transition observed between samples i-1 and i actually happened somewhere
    in that half-open interval, so the boundary is placed at the MIDPOINT. That
    is the zero-bias choice: the residual error per boundary is uniform on
    +/- half a poll period instead of always late by up to a full one.

    Runs touching either end of the window are marked `censored` — we joined
    or left them part-way through and their duration is a lower bound, not a
    measurement. Duration statistics must drop them; totals may keep them.

    Samples with key None are treated as "no observation" and break the run.
    """
    runs: List[Dict[str, Any]] = []
    if not series:
        return runs
    cur_key = series[0][1]
    cur_start = float(series[0][0])
    start_censored = True
    for i in range(1, len(series)):
        t, key = float(series[i][0]), series[i][1]
        if key == cur_key:
            continue
        boundary = 0.5 * (float(series[i - 1][0]) + t)
        if cur_key is not None:
            runs.append({"key": cur_key, "t_start": cur_start,
                         "t_end": boundary, "duration": boundary - cur_start,
                         "censored": start_censored})
        cur_key, cur_start, start_censored = key, boundary, False
    if cur_key is not None:
        t_end = float(series[-1][0])
        runs.append({"key": cur_key, "t_start": cur_start, "t_end": t_end,
                     "duration": t_end - cur_start, "censored": True})
    return runs


def run_stats(runs: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Per-key duration stats. Censored runs feed `total_observed_s` only."""
    out: Dict[str, Any] = {}
    for r in runs:
        b = out.setdefault(str(r["key"]),
                           {"complete": [], "total_observed_s": 0.0,
                            "censored_runs": 0})
        b["total_observed_s"] += r["duration"]
        if r["censored"]:
            b["censored_runs"] += 1
        else:
            b["complete"].append(r["duration"])
    for key, b in out.items():
        d = describe(b.pop("complete"))
        d["total_observed_s"] = b["total_observed_s"]
        d["censored_runs"] = b["censored_runs"]
        out[key] = d
    return out


def counter_events(series: Sequence[Tuple[float, Optional[float]]]
                   ) -> Dict[str, Any]:
    """Monotonic-counter samples -> increment times, deltas and intervals.

    Increment time is the midpoint of the bracketing samples, same reasoning as
    `segment_runs`. The interval list holds only gaps BETWEEN two observed
    increments: the lead-in (window start -> first increment) and the tail
    (last increment -> window end) are partial cycles and are excluded, because
    including them would drag every mean toward zero in proportion to how short
    the run was. N complete intervals therefore needs N+1 increments.

    A counter that goes DOWN (a controller restart) is reported in `resets`
    rather than scored as a negative interval.
    """
    times: List[float] = []
    deltas: List[float] = []
    resets = 0
    first = last = None
    prev_t = prev_v = None
    for t, v in series:
        if v is None:
            continue
        t = float(t)
        v = float(v)
        if first is None:
            first = v
        last = v
        if prev_v is not None:
            if v < prev_v:
                resets += 1
            elif v > prev_v:
                times.append(0.5 * (prev_t + t))
                deltas.append(v - prev_v)
        prev_t, prev_v = t, v
    intervals = [times[i] - times[i - 1] for i in range(1, len(times))]
    return {
        "first": first, "last": last,
        "delta": (None if first is None else last - first),
        "increments": len(times),
        "increment_total": sum(deltas),
        "resets": resets,
        "event_times": times,
        "intervals": intervals,
    }


def stationary_time(t: Sequence[float],
                    xy: Sequence[Tuple[float, float]],
                    yaws: Sequence[float],
                    speed_eps_m_s: float = 0.02,
                    omega_eps_rad_s: float = math.radians(1.0)
                    ) -> Dict[str, Any]:
    """Split observed time into stationary vs moving.

    A step counts as stationary when BOTH the linear speed and the yaw rate
    over that step are below their thresholds — a tug spinning on the spot is
    not stationary, and a tug sliding sideways with a fixed heading is not
    either. Speeds are derived from the poses, not from the bridge's reported
    `v_linear`/`v_angular`: the tug is kinematic and its reported velocity is
    the COMMAND, which is exactly the number a route change would make look
    good without moving the robot any less.
    """
    stat = 0.0
    move = 0.0
    speeds: List[float] = []
    for i in range(1, len(t)):
        dt = float(t[i]) - float(t[i - 1])
        if dt <= 0:
            continue
        d = math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
        w = abs(yaw_step(yaws[i - 1], yaws[i])) / dt
        v = d / dt
        speeds.append(v)
        if v < speed_eps_m_s and w < omega_eps_rad_s:
            stat += dt
        else:
            move += dt
    total = stat + move
    return {
        "stationary_s": stat,
        "moving_s": move,
        "observed_s": total,
        "stationary_frac": (stat / total) if total > 0 else None,
        "max_speed_m_s": max(speeds) if speeds else 0.0,
        "mean_speed_moving_m_s": (
            statistics.fmean([s for s in speeds if s >= speed_eps_m_s])
            if any(s >= speed_eps_m_s for s in speeds) else 0.0),
    }


def bucket_by(t: Sequence[float], xy: Sequence[Tuple[float, float]],
              yaws: Sequence[float], keys: Sequence[Any],
              deadband_m: float = 1e-3,
              deadband_rad: float = 3.5e-4) -> Dict[str, Dict[str, float]]:
    """Attribute each step's distance / rotation / time to a bucket key.

    A step is charged to the key held at its START sample; steps that straddle
    a transition are therefore charged to the state being left. At 2 Hz that
    misattributes at most half a second of travel per transition.
    """
    out: Dict[str, Dict[str, float]] = {}
    for i in range(1, len(t)):
        k = str(keys[i - 1])
        b = out.setdefault(k, {"path_m": 0.0, "rotation_deg": 0.0,
                               "time_s": 0.0, "steps": 0})
        d = math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
        r = abs(yaw_step(yaws[i - 1], yaws[i]))
        b["path_m"] += d if d >= deadband_m else 0.0
        b["rotation_deg"] += math.degrees(r) if r >= deadband_rad else 0.0
        b["time_s"] += float(t[i]) - float(t[i - 1])
        b["steps"] += 1
    return out


def realtime_factor(wall: Sequence[float], sim: Sequence[float],
                    stall_eps_s: float = 1e-4) -> Dict[str, Any]:
    """Sim-seconds per wall-second, plus how long the sim was not advancing.

    Reported because every wall-clock number in this report is only comparable
    across runs at the same realtime factor. A "20% faster line" that is really
    a 20% faster simulator is the failure mode this catches.
    """
    pairs = [(float(w), float(s)) for w, s in zip(wall, sim) if s is not None]
    if len(pairs) < 2:
        return {"factor": None, "sim_span_s": None, "wall_span_s": None,
                "stalled_wall_s": None}
    stalled = 0.0
    for i in range(1, len(pairs)):
        if pairs[i][1] - pairs[i - 1][1] < stall_eps_s:
            stalled += pairs[i][0] - pairs[i - 1][0]
    wall_span = pairs[-1][0] - pairs[0][0]
    sim_span = pairs[-1][1] - pairs[0][1]
    return {
        "factor": (sim_span / wall_span) if wall_span > 0 else None,
        "sim_span_s": sim_span,
        "wall_span_s": wall_span,
        "stalled_wall_s": stalled,
    }


def transit_runs(series: Sequence[Tuple[float, Optional[list]]]
                 ) -> List[Dict[str, Any]]:
    """`line.in_transit` snapshots -> one run per (box, phase) occupancy.

    Each snapshot is a list of {box, trolley, delivered}. A box present with
    delivered=False is being towed; delivered=True means it has arrived at its
    park spot and the SHIP_AFTER_S timer is running. Tracked per box so two
    loads in flight at once do not blur together.
    """
    open_runs: Dict[Tuple[str, str], Dict[str, Any]] = {}
    done: List[Dict[str, Any]] = []
    prev_t: Optional[float] = None
    for t, snap in series:
        t = float(t)
        mid = t if prev_t is None else 0.5 * (prev_t + t)
        present: Dict[Tuple[str, str], str] = {}
        if snap is not None:
            for e in snap:
                box = str(e.get("box"))
                phase = "at_dispatch" if e.get("delivered") else "towing"
                present[(box, phase)] = str(e.get("trolley"))
        for k in list(open_runs):
            if k not in present:
                r = open_runs.pop(k)
                r["t_end"] = mid
                r["duration"] = mid - r["t_start"]
                done.append(r)
        for k, trolley in present.items():
            if k not in open_runs:
                open_runs[k] = {"key": k[1], "box": k[0], "trolley": trolley,
                                "t_start": mid, "censored": prev_t is None}
        prev_t = t
    for k, r in open_runs.items():
        r["t_end"] = prev_t if prev_t is not None else r["t_start"]
        r["duration"] = r["t_end"] - r["t_start"]
        r["censored"] = True
        done.append(r)
    return done


# ══════════════════════════════════════════════════════════════════════
# HTTP — read-only.
# ══════════════════════════════════════════════════════════════════════

class BridgeError(RuntimeError):
    pass


def get_json(url: str, timeout: float, token: str = "") -> Any:
    """One GET. The ONLY network verb this module uses (see module docstring)."""
    req = urllib.request.Request(url, method="GET")
    req.add_header("Accept", "application/json")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read()
            if resp.status != 200:
                raise BridgeError(f"{url} -> HTTP {resp.status}")
            return json.loads(body.decode("utf-8"))
    except urllib.error.HTTPError as e:
        raise BridgeError(f"{url} -> HTTP {e.code} {e.reason}") from e
    except urllib.error.URLError as e:
        raise BridgeError(f"{url} -> not answering ({e.reason})") from e
    except (OSError, ValueError) as e:
        raise BridgeError(f"{url} -> {e!r}") from e


# ══════════════════════════════════════════════════════════════════════
# Collection
# ══════════════════════════════════════════════════════════════════════

class Recorder:
    """Holds the raw series for one bridge."""

    def __init__(self, name: str, url: str) -> None:
        self.name = name
        self.url = url
        self.t: List[float] = []           # wall seconds since run start
        self.sim: List[Optional[float]] = []
        self.states: List[dict] = []
        self.latencies: List[float] = []
        self.failures: List[Dict[str, Any]] = []
        self.consecutive_failures = 0
        self.meta: Dict[str, Any] = {}

    def add(self, t: float, state: dict, latency: float) -> None:
        self.t.append(t)
        self.sim.append(_num(state.get("sim_time")))
        self.states.append(state)
        self.latencies.append(latency)
        self.consecutive_failures = 0

    def fail(self, t: float, msg: str) -> None:
        self.failures.append({"t": t, "error": msg})
        self.consecutive_failures += 1


def _num(v: Any) -> Optional[float]:
    if isinstance(v, bool) or v is None:
        return None
    if isinstance(v, (int, float)) and math.isfinite(float(v)):
        return float(v)
    return None


def _dig(state: dict, *path: str) -> Any:
    cur: Any = state
    for k in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur


def preflight(recs: Dict[str, Recorder], timeout: float, token: str
              ) -> List[str]:
    """Probe every bridge once. Raises BridgeError with an actionable message.

    Returns the non-fatal warnings.
    """
    warnings: List[str] = []
    problems: List[str] = []
    for name, rec in recs.items():
        try:
            st = get_json(rec.url + "/state", timeout, token)
        except BridgeError as e:
            # NOT launch.bat: it prepends msys64\mingw64\bin\newton-runtime to
            # PATH (launch.bat:20-23) and that directory holds a python.exe
            # with no `omnilink` installed. The engine spawns controllers with
            # the bare command `python`, so the bridges silently lose their
            # relay. Harmless for THIS module (it only reads /state) but the
            # same instructions get copied into runs where it is not, so the
            # hint points at the PATH-safe launcher.
            problems.append(
                f"  {name:6s} {e}\n"
                f"         Is the world open and playing? Launch it with:\n"
                f"           python scripts/dev/headless_runner.py \\\n"
                f"             projects/samples/demos/worlds/flagship/"
                f"warehouse_omnilink.omniworld \\\n"
                f"             --gui --realtime --duration 1800\n"
                f"         (--realtime matters: without it the runner passes "
                f"--mode=fast and\n"
                f"          boxes/min stops being a wall-clock rate. See "
                f"README.md.)")
            continue
        if not isinstance(st, dict):
            problems.append(f"  {name:6s} {rec.url}/state returned "
                            f"{type(st).__name__}, expected an object")
            continue
        rec.meta = {"id": st.get("id"), "model": st.get("model")}
        try:
            caps = get_json(rec.url + "/capabilities", timeout, token)
            if isinstance(caps, list) and caps:
                rec.meta["capabilities"] = caps[0].get("capabilities")
        except BridgeError:
            warnings.append(f"{name}: /capabilities unavailable "
                            f"(yaw-rate ceiling unknown)")
        if _dig(st, "idle_loop") is None:
            warnings.append(f"{name}: no idle_loop block -- this bridge is not "
                            f"running an autonomous loop, so its cycle and "
                            f"leg metrics will be empty")
    if problems:
        raise BridgeError("bridge(s) not answering:\n" + "\n".join(problems))
    arm = recs.get("omniarm6")
    if arm is not None:
        st = get_json(arm.url + "/state", timeout, token)
        if not isinstance(_dig(st, "line"), dict):
            raise BridgeError(
                f"  {arm.url}/state has no `line` block.\n"
                f"         The OMNIARM6 bridge only publishes it in a world that\n"
                f"         carries DEF BOX_* solids and a DEF FILL_STOP marker\n"
                f"         i.e. warehouse_omnilink.omniworld. Whatever is loaded\n"
                f"         now is not the warehouse line; there is nothing for\n"
                f"         this harness to measure.")
    return warnings


def collect(recs: Dict[str, Recorder], duration_s: float, hz: float,
            timeout: float, token: str, max_consecutive_failures: int,
            quiet: bool) -> Tuple[bool, Optional[str]]:
    """Poll every bridge at `hz` for `duration_s`. Returns (ok, error)."""
    period = 1.0 / hz
    t0 = time.monotonic()
    deadline = t0 + duration_s
    next_tick = t0
    names = list(recs)
    last_report = t0

    def sample(name: str) -> Tuple[str, float, Optional[dict], float, str]:
        rec = recs[name]
        a = time.monotonic()
        try:
            st = get_json(rec.url + "/state", timeout, token)
            b = time.monotonic()
            if not isinstance(st, dict):
                return name, a - t0, None, b - a, "non-object /state body"
            return name, a - t0, st, b - a, ""
        except BridgeError as e:
            return name, a - t0, None, time.monotonic() - a, str(e)

    with ThreadPoolExecutor(max_workers=max(3, len(names))) as pool:
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if now < next_tick:
                time.sleep(min(next_tick - now, 0.05))
                continue
            next_tick += period
            if next_tick < time.monotonic():          # fell behind; resync
                next_tick = time.monotonic() + period
            for name, t, st, lat, err in pool.map(sample, names):
                rec = recs[name]
                if st is None:
                    rec.fail(t, err)
                    if rec.consecutive_failures >= max_consecutive_failures:
                        return False, (
                            f"{name} ({rec.url}) stopped answering: {err}\n"
                            f"  {rec.consecutive_failures} consecutive failed "
                            f"polls. The controller probably died -- check the "
                            f"simulator window and omnisim_log.txt.")
                else:
                    rec.add(t, st, lat)
            if not quiet and time.monotonic() - last_report >= 15.0:
                last_report = time.monotonic()
                arm = recs.get("omniarm6")
                shipped = filled = "?"
                if arm is not None and arm.states:
                    ln = _dig(arm.states[-1], "line") or {}
                    shipped = ln.get("shipped_total", "?")
                    filled = ln.get("boxes_filled_total", "?")
                print(f"  [{time.monotonic() - t0:6.1f}s / {duration_s:.0f}s] "
                      f"shipped={shipped} filled={filled} "
                      f"samples={len(recs[names[0]].t)}",
                      file=sys.stderr, flush=True)
    return True, None


# ══════════════════════════════════════════════════════════════════════
# Analysis
# ══════════════════════════════════════════════════════════════════════

def analyse_line(rec: Recorder) -> Dict[str, Any]:
    """Everything derived from the OMNIARM6 bridge's `line` + `idle_loop`."""
    t = rec.t
    line = [_dig(s, "line") or {} for s in rec.states]
    arm = [_dig(s, "idle_loop") or {} for s in rec.states]

    shipped = counter_events([(t[i], _num(line[i].get("shipped_total")))
                              for i in range(len(t))])
    filled = counter_events([(t[i], _num(line[i].get("boxes_filled_total")))
                             for i in range(len(t))])
    picks = counter_events([(t[i], _num(arm[i].get("picks")))
                            for i in range(len(t))])

    # Front-box stage track. Keyed on (box, state) so a state that repeats on
    # a DIFFERENT box is a new run rather than a continuation.
    stage_series = [(t[i], (f"{line[i].get('fill_box')}|"
                            f"{line[i].get('fill_state')}")
                     if line[i].get("fill_state") else None)
                    for i in range(len(t))]
    stage_runs = segment_runs(stage_series)
    for r in stage_runs:
        r["box"], r["key"] = str(r["key"]).split("|", 1)

    # Dispatch-side stages, per box, from `in_transit`.
    tr = transit_runs([(t[i], line[i].get("in_transit"))
                       for i in range(len(t))])

    stages = run_stats(stage_runs + tr)
    for name in FILL_STATES + ("towing", "at_dispatch"):
        stages.setdefault(name, dict(describe([]),
                                     total_observed_s=0.0, censored_runs=0))

    # Arm idle-wait. TWO different numbers, both wanted:
    #  * `arm_not_working_s` — the arm's own loop reporting idle/respawn while
    #    not paused by an operator. What the ARM is doing.
    #  * `fill_blocked_s` — the `filled` stage: box full, no empty cart yet.
    #    What the LINE is doing to it. This is the 40-65 s figure in
    #    WAREHOUSE_OMNILINK.md, and it is the one to optimise against.
    idle_series = [(t[i], (None if arm[i].get("leg") is None else
                           ("waiting" if (arm[i].get("leg") in ("idle", "respawn")
                                          and not arm[i].get("paused"))
                            else ("paused" if arm[i].get("paused") else "working"))))
                   for i in range(len(t))]
    idle_runs = segment_runs(idle_series)
    idle = run_stats(idle_runs)
    arm_leg_runs = segment_runs([(t[i], arm[i].get("leg")) for i in range(len(t))])

    observed = (t[-1] - t[0]) if len(t) > 1 else 0.0
    cycles = len(shipped["intervals"])
    filled_blocked = stages.get("filled", {})

    return {
        "observed_wall_s": observed,
        "counters": {
            "shipped_total": {k: shipped[k] for k in
                              ("first", "last", "delta", "increments", "resets")},
            "boxes_filled_total": {k: filled[k] for k in
                                   ("first", "last", "delta", "increments", "resets")},
            "arm_picks": {k: picks[k] for k in
                          ("first", "last", "delta", "increments", "resets")},
        },
        "throughput": {
            "boxes_shipped": shipped["increment_total"],
            "boxes_per_minute": (60.0 * shipped["increment_total"] / observed
                                 if observed > 0 else None),
            "boxes_filled": filled["increment_total"],
            "fills_per_minute": (60.0 * filled["increment_total"] / observed
                                 if observed > 0 else None),
            "picks": picks["increment_total"],
            "picks_per_minute": (60.0 * picks["increment_total"] / observed
                                 if observed > 0 else None),
        },
        "cycle_time_s": {
            "shipped": describe(shipped["intervals"]),
            "filled": describe(filled["intervals"]),
            "pick": describe(picks["intervals"]),
        },
        "arm_wait": {
            # Per-cycle figures are only meaningful with >=1 complete cycle.
            "complete_cycles": cycles,
            "fill_blocked_per_cycle_s": describe(
                [r["duration"] for r in stage_runs
                 if r["key"] == "filled" and not r["censored"]]),
            "fill_blocked_total_s": filled_blocked.get("total_observed_s", 0.0),
            "fill_blocked_frac_of_run": (
                filled_blocked.get("total_observed_s", 0.0) / observed
                if observed > 0 else None),
            "arm_not_working_s": idle.get("waiting", {}).get("total_observed_s", 0.0),
            "arm_not_working_per_cycle_s": (
                idle.get("waiting", {}).get("total_observed_s", 0.0) / cycles
                if cycles > 0 else None),
            "arm_paused_by_operator_s": idle.get("paused", {}).get("total_observed_s", 0.0),
            "by_state": idle,
        },
        "stages_s": stages,
        "arm_legs_s": run_stats(arm_leg_runs),
        "stage_events": [
            {"box": r["box"], "stage": r["key"], "t_start": round(r["t_start"], 3),
             "duration": round(r["duration"], 3), "censored": r["censored"]}
            for r in sorted(stage_runs + [dict(x, box=x.get("box", "?"))
                                          for x in tr],
                            key=lambda r: r["t_start"])],
        "shipped_event_times": [round(x, 3) for x in shipped["event_times"]],
    }


def analyse_tug(rec: Recorder, deadband_m: float, deadband_rad: float,
                speed_eps: float, omega_eps: float, hz: float) -> Dict[str, Any]:
    """Path length, rotation and stationary time for one tug."""
    t = rec.t
    pts: List[Tuple[float, float]] = []
    yaws: List[float] = []
    keep: List[int] = []
    for i, s in enumerate(rec.states):
        x, y, yaw = _num(s.get("x")), _num(s.get("y")), _num(s.get("yaw"))
        if x is None or y is None or yaw is None:
            continue
        keep.append(i)
        pts.append((x, y))
        yaws.append(yaw)
    tt = [t[i] for i in keep]
    if len(tt) < 2:
        return {"error": "fewer than 2 usable pose samples"}

    loops = [_dig(rec.states[i], "idle_loop") or {} for i in keep]
    legs = [l.get("leg") for l in loops]
    towing = ["towing" if rec.states[i].get("carrying") else "empty"
              for i in keep]

    rot = integrate_rotation(yaws, deadband_rad)
    path = integrate_path(pts, deadband_m)
    stat = stationary_time(tt, pts, yaws, speed_eps, omega_eps)

    caps = (rec.meta.get("capabilities") or {})
    w_max = _num(caps.get("max_angular_rad_s"))
    achieved_hz = (len(tt) - 1) / (tt[-1] - tt[0]) if tt[-1] > tt[0] else hz
    nyquist_hz = (w_max / math.pi) if w_max else None

    cycles = counter_events([(tt[i], _num(loops[i].get("cycles")))
                             for i in range(len(tt))])
    jobs = counter_events([(tt[i], _num(loops[i].get("jobs_total")))
                           for i in range(len(tt))])
    delivered = counter_events([(tt[i], _num(loops[i].get("delivered_total")))
                                for i in range(len(tt))])
    observed = tt[-1] - tt[0]

    return {
        "observed_wall_s": observed,
        "samples": len(tt),
        "achieved_poll_hz": achieved_hz,
        "motion": {
            "path_length_m": path["path_m"],
            "path_length_m_raw": path["path_m_raw"],
            "net_displacement_m": path["net_m"],
            "path_efficiency": (path["net_m"] / path["path_m"]
                                if path["path_m"] > 0 else None),
            "max_step_m": path["max_step_m"],
            "rotation_deg": math.degrees(rot["total_rad"]),
            "rotation_deg_raw": math.degrees(rot["total_rad_raw"]),
            "net_rotation_deg": math.degrees(rot["net_rad"]),
            # |net| / total. 1.0 = every degree turned went somewhere;
            # near 0 = the tug turned back and forth for nothing.
            "rotation_efficiency": (abs(rot["net_rad"]) / rot["total_rad"]
                                    if rot["total_rad"] > 1e-9 else None),
            "rotation_deg_per_m": (math.degrees(rot["total_rad"]) / path["path_m"]
                                   if path["path_m"] > 0 else None),
            "max_yaw_step_deg": math.degrees(rot["max_step_rad"]),
        },
        "unwrap_audit": {
            "method": "per-step wrap_pi(cur - prev), summed",
            "max_yaw_step_deg": math.degrees(rot["max_step_rad"]),
            "aliasing_suspect_steps": rot["aliasing_suspect_steps"],
            "bridge_max_yaw_rate_rad_s": w_max,
            "nyquist_min_hz": nyquist_hz,
            "nyquist_margin": (achieved_hz / nyquist_hz
                               if nyquist_hz else None),
        },
        "time": {
            "stationary_s": stat["stationary_s"],
            "moving_s": stat["moving_s"],
            "stationary_frac": stat["stationary_frac"],
            "max_speed_m_s": stat["max_speed_m_s"],
            "mean_speed_moving_m_s": stat["mean_speed_moving_m_s"],
        },
        "counters": {
            "cycles": {k: cycles[k] for k in ("first", "last", "delta", "resets")},
            "jobs_total": {k: jobs[k] for k in ("first", "last", "delta", "resets")},
            "delivered_total": {k: delivered[k] for k in
                                ("first", "last", "delta", "resets")},
            "cycle_interval_s": describe(cycles["intervals"]),
            "job_interval_s": describe(jobs["intervals"]),
        },
        "by_leg": bucket_by(tt, pts, yaws, legs, deadband_m, deadband_rad),
        "by_load": bucket_by(tt, pts, yaws, towing, deadband_m, deadband_rad),
        "legs_s": run_stats(segment_runs(list(zip(tt, legs)))),
        "holds": {
            "holds_total": _num((loops[-1] or {}).get("holds_total")),
            "hold_secs_max": _num((loops[-1] or {}).get("hold_secs_max")),
            "holding_frac": (sum(1 for l in loops if l.get("holding"))
                             / len(loops)) if loops else None,
        },
        "paused_by_operator_frac": (sum(1 for l in loops if l.get("paused"))
                                    / len(loops)) if loops else None,
    }


def build_report(recs: Dict[str, Recorder], args: argparse.Namespace,
                 started_utc: str, warnings: List[str],
                 aborted: Optional[str]) -> Dict[str, Any]:
    bridges: Dict[str, Any] = {}
    for name, rec in recs.items():
        rt = realtime_factor(rec.t, rec.sim)
        bridges[name] = {
            "url": rec.url,
            "id": rec.meta.get("id"),
            "model": rec.meta.get("model"),
            "polls_ok": len(rec.t),
            "polls_failed": len(rec.failures),
            "poll_latency_ms": {
                "mean": (1000.0 * statistics.fmean(rec.latencies)
                         if rec.latencies else None),
                "p95": (1000.0 * percentile(rec.latencies, 95.0)
                        if rec.latencies else None),
                "max": 1000.0 * max(rec.latencies) if rec.latencies else None,
            },
            "realtime": rt,
            "first_failures": rec.failures[:5],
        }

    arm = recs.get("omniarm6")
    line = analyse_line(arm) if (arm and len(arm.t) > 1) else {
        "error": "not enough OMNIARM6 samples"}

    tugs: Dict[str, Any] = {}
    for name in ("tug_a", "tug_b"):
        rec = recs.get(name)
        if rec is None or len(rec.t) < 2:
            tugs[name] = {"error": "not enough samples"}
            continue
        tugs[name] = analyse_tug(rec, args.pos_deadband_m,
                                 math.radians(args.yaw_deadband_deg),
                                 args.stationary_speed_m_s,
                                 math.radians(args.stationary_omega_deg),
                                 args.hz)

    warn = list(warnings)
    for name, t in tugs.items():
        aud = t.get("unwrap_audit") or {}
        margin = aud.get("nyquist_margin")
        if margin is not None and margin < 1.5:
            warn.append(
                f"{name}: poll rate {aud.get('nyquist_margin'):.2f}x the "
                f"unwrap Nyquist bound -- raise --hz; rotation may be "
                f"UNDER-reported")
        if aud.get("aliasing_suspect_steps"):
            warn.append(
                f"{name}: {aud['aliasing_suspect_steps']} yaw step(s) exceeded "
                f"90 deg (max {aud.get('max_yaw_step_deg', 0):.1f} deg) -- "
                f"close to the wrap limit; consider a higher --hz")
    cyc = ((line.get("arm_wait") or {}).get("complete_cycles")
           if isinstance(line, dict) else None)
    if cyc is not None and cyc < 3:
        warn.append(f"only {cyc} complete ship cycle(s) observed -- cycle-time "
                    f"mean/median/p95 are not yet meaningful; run longer")
    for name, b in bridges.items():
        f = (b.get("realtime") or {}).get("factor")
        if f is not None and f < 0.5:
            warn.append(f"{name}: sim ran at {f:.2f}x realtime -- wall-clock "
                        f"figures are not comparable with a ~1.0x baseline")

    report: Dict[str, Any] = {
        "schema": SCHEMA,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "run": {
            "started_at_utc": started_utc,
            "duration_requested_s": args.duration,
            "poll_hz_requested": args.hz,
            "label": args.label,
            "aborted": aborted,
            "thresholds": {
                "pos_deadband_m": args.pos_deadband_m,
                "yaw_deadband_deg": args.yaw_deadband_deg,
                "stationary_speed_m_s": args.stationary_speed_m_s,
                "stationary_omega_deg": args.stationary_omega_deg,
            },
        },
        "bridges": bridges,
        "line": line,
        "tugs": tugs,
        "warnings": warn,
    }
    if args.raw:
        report["raw"] = {
            name: {
                "t": [round(x, 4) for x in rec.t],
                "sim_time": rec.sim,
                "samples": [_compact(name, s) for s in rec.states],
            } for name, rec in recs.items()
        }
    return report


def _compact(name: str, s: dict) -> dict:
    """Trim a /state snapshot to the fields this harness actually reads."""
    if name == "omniarm6":
        return {"line": s.get("line"), "idle_loop": s.get("idle_loop"),
                "mode": s.get("mode"), "fault": s.get("fault")}
    return {"x": s.get("x"), "y": s.get("y"), "yaw": s.get("yaw"),
            "carrying": s.get("carrying"), "mode": s.get("mode"),
            "fault": s.get("fault"),
            "idle_loop": {k: (s.get("idle_loop") or {}).get(k)
                          for k in ("leg", "cycles", "jobs_total",
                                    "delivered_total", "paused", "holding",
                                    "on_lane")}}


# ══════════════════════════════════════════════════════════════════════
# Human summary
# ══════════════════════════════════════════════════════════════════════

def _f(v: Any, spec: str = ".1f", dash: str = "  -  ") -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return dash
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return str(v)


def print_summary(rep: Dict[str, Any], out: Any = sys.stdout) -> None:
    # A legacy Windows codepage must never turn a finished measurement
    # into a UnicodeEncodeError traceback.
    try:
        out.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    p = lambda *a: print(*a, file=out)
    run = rep["run"]
    p("")
    p("=" * 72)
    p(f"  WAREHOUSE LINE MEASUREMENT{'  -- ' + run['label'] if run.get('label') else ''}")
    p("=" * 72)

    line = rep.get("line") or {}
    if line.get("error"):
        p(f"  line: {line['error']}")
    else:
        obs = line["observed_wall_s"]
        th = line["throughput"]
        ct = line["cycle_time_s"]["shipped"]
        aw = line["arm_wait"]
        p(f"  window            {_f(obs)} s wall "
          f"({_f(obs / 60.0, '.2f')} min), "
          f"{rep['bridges']['omniarm6']['polls_ok']} samples")
        rt = (rep["bridges"]["omniarm6"].get("realtime") or {}).get("factor")
        p(f"  realtime factor   {_f(rt, '.3f')} x  "
          f"(sim seconds per wall second, from OMNIARM6 sim_time)")
        p("")
        p("  THROUGHPUT")
        p(f"    boxes shipped         {int(th['boxes_shipped'])}"
          f"   -> {_f(th['boxes_per_minute'], '.3f')} boxes/min")
        p(f"    boxes filled          {int(th['boxes_filled'])}"
          f"   -> {_f(th['fills_per_minute'], '.3f')} fills/min")
        p(f"    parts picked          {int(th['picks'])}"
          f"   -> {_f(th['picks_per_minute'], '.3f')} picks/min")
        p("")
        p("  CYCLE TIME (seconds between consecutive shipped_total increments)")
        p(f"    n={ct['n']:<3d} mean {_f(ct['mean'])}  median {_f(ct['median'])}"
          f"  p95 {_f(ct['p95'])}  min {_f(ct['min'])}  max {_f(ct['max'])}")
        p("")
        p("  ARM IDLE-WAIT")
        p(f"    box full, no cart ('filled' stage): "
          f"n={aw['fill_blocked_per_cycle_s']['n']} "
          f"mean {_f(aw['fill_blocked_per_cycle_s']['mean'])} s  "
          f"median {_f(aw['fill_blocked_per_cycle_s']['median'])} s  "
          f"p95 {_f(aw['fill_blocked_per_cycle_s']['p95'])} s")
        p(f"    total blocked        {_f(aw['fill_blocked_total_s'])} s "
          f"({_f(100.0 * (aw['fill_blocked_frac_of_run'] or 0), '.1f')}% of the run)")
        p(f"    arm not working      {_f(aw['arm_not_working_s'])} s"
          f"  ({_f(aw['arm_not_working_per_cycle_s'])} s per shipped box)")
        p("")
        p("  PER-STAGE DURATION (s)      n    mean   median      p95    total")
        for name in list(FILL_STATES) + ["towing", "at_dispatch"]:
            st = line["stages_s"].get(name)
            if not st:
                continue
            p(f"    {name:<22s} {st['n']:>4d} {_f(st['mean']):>7s} "
              f"{_f(st['median']):>8s} {_f(st['p95']):>8s} "
              f"{_f(st['total_observed_s']):>8s}")

    for name in ("tug_a", "tug_b"):
        tug = (rep.get("tugs") or {}).get(name) or {}
        p("")
        p(f"  TUG {name}")
        if tug.get("error"):
            p(f"    {tug['error']}")
            continue
        m, tm = tug["motion"], tug["time"]
        p(f"    path length          {_f(m['path_length_m'], '.2f')} m"
          f"   (net displacement {_f(m['net_displacement_m'], '.2f')} m,"
          f" efficiency {_f(m['path_efficiency'], '.3f')})")
        p(f"    total rotation       {_f(m['rotation_deg'], '.1f')} deg"
          f"  (net {_f(m['net_rotation_deg'], '.1f')} deg,"
          f" efficiency {_f(m['rotation_efficiency'], '.3f')})")
        p(f"    rotation per metre   {_f(m['rotation_deg_per_m'], '.2f')} deg/m")
        p(f"    stationary           {_f(tm['stationary_s'])} s"
          f"  ({_f(100.0 * (tm['stationary_frac'] or 0), '.1f')}% of window),"
          f" moving {_f(tm['moving_s'])} s")
        c = tug["counters"]
        p(f"    cycles +{_f(c['cycles']['delta'], '.0f')}"
          f"   jobs +{_f(c['jobs_total']['delta'], '.0f')}"
          f"   parked-total +{_f(c['delivered_total']['delta'], '.0f')}")
        loaded = tug["by_load"]
        for k in ("towing", "empty"):
            b = loaded.get(k)
            if b:
                p(f"    while {k:<8s}       {_f(b['path_m'], '.2f'):>8s} m"
                  f"  {_f(b['rotation_deg'], '.1f'):>8s} deg"
                  f"  {_f(b['time_s']):>8s} s")
        top = sorted(tug["by_leg"].items(),
                     key=lambda kv: -kv[1]["rotation_deg"])[:5]
        if top:
            p(f"    rotation by leg:     " + "  ".join(
                f"{k}={v['rotation_deg']:.0f}deg/{v['path_m']:.1f}m"
                for k, v in top if v["rotation_deg"] > 0.5 or v["path_m"] > 0.5))

    warn = rep.get("warnings") or []
    p("")
    if warn:
        p("  WARNINGS")
        for w in warn:
            p(f"    ! {w}")
    else:
        p("  no warnings")
    p("=" * 72)
    p("")


# ══════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="measure_line.py",
        description="Measure the warehouse_omnilink line over its three "
                    "bridges. Read-only: GET requests only, one output file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--duration", type=float, default=600.0,
                    help="wall-clock seconds to observe")
    ap.add_argument("--hz", type=float, default=2.0,
                    help="poll rate per bridge (see the unwrap Nyquist note; "
                         "below ~1.1 Hz tug rotation can alias)")
    ap.add_argument("--out", default=None,
                    help="write the machine-readable JSON here")
    ap.add_argument("--label", default="",
                    help="free-text tag stored in the JSON (e.g. 'baseline')")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--arm-port", type=int, default=8765)
    ap.add_argument("--tug-a-port", type=int, default=8766)
    ap.add_argument("--tug-b-port", type=int, default=8767)
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="per-request HTTP timeout, seconds")
    ap.add_argument("--token", default=os.environ.get("OMNISIM_BRIDGE_TOKEN", ""),
                    help="bearer token, if the bridges were started with one")
    ap.add_argument("--max-consecutive-failures", type=int, default=10,
                    help="give up on a bridge after this many failed polls")
    ap.add_argument("--min-cycles", type=int, default=0,
                    help="exit 5 unless this many complete ship cycles were "
                         "observed (use as a CI gate)")
    ap.add_argument("--pos-deadband-m", type=float, default=0.001,
                    help="per-sample translation below this is noise")
    ap.add_argument("--yaw-deadband-deg", type=float, default=0.02,
                    help="per-sample rotation below this is noise")
    ap.add_argument("--stationary-speed-m-s", type=float, default=0.02)
    ap.add_argument("--stationary-omega-deg", type=float, default=1.0,
                    help="deg/s below which a tug counts as not turning")
    ap.add_argument("--raw", dest="raw", action="store_true", default=True,
                    help="embed the full sample series in the JSON")
    ap.add_argument("--no-raw", dest="raw", action="store_false")
    ap.add_argument("--quiet", action="store_true",
                    help="no progress lines on stderr")
    ap.add_argument("--selftest", action="store_true",
                    help="run the pure-helper unit tests and exit "
                         "(no simulator needed)")
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.selftest:
        here = os.path.dirname(os.path.abspath(__file__))
        sys.path.insert(0, here)
        import test_measure_line
        return test_measure_line.run_all()

    if args.duration <= 0 or args.hz <= 0:
        print("error: --duration and --hz must be positive", file=sys.stderr)
        return 4
    if args.hz > 50:
        print("error: --hz above 50 will load the bridges more than it "
              "measures them", file=sys.stderr)
        return 4

    recs = {
        "omniarm6": Recorder("omniarm6", f"http://{args.host}:{args.arm_port}"),
        "tug_a": Recorder("tug_a", f"http://{args.host}:{args.tug_a_port}"),
        "tug_b": Recorder("tug_b", f"http://{args.host}:{args.tug_b_port}"),
    }

    print(f"measure_line: probing {len(recs)} bridges ...", file=sys.stderr)
    try:
        warnings = preflight(recs, args.timeout, args.token)
    except BridgeError as e:
        print(f"\nPREFLIGHT FAILED\n{e}\n", file=sys.stderr)
        return 2
    for name, rec in recs.items():
        print(f"  {name:6s} ok  {rec.meta.get('model')}  ({rec.url})",
              file=sys.stderr)

    started_utc = datetime.now(timezone.utc).isoformat()
    print(f"measure_line: sampling for {args.duration:.0f}s at {args.hz} Hz "
          f"(Ctrl-C stops early and still reports)", file=sys.stderr)
    aborted: Optional[str] = None
    try:
        ok, err = collect(recs, args.duration, args.hz, args.timeout,
                          args.token, args.max_consecutive_failures,
                          args.quiet)
        if not ok:
            aborted = err
    except KeyboardInterrupt:
        aborted = "interrupted by the operator (Ctrl-C)"
        print("\nmeasure_line: interrupted -- reporting what was collected",
              file=sys.stderr)

    if len(recs["omniarm6"].t) < 2:
        print("\nerror: fewer than 2 OMNIARM6 samples collected; nothing to "
              "report.", file=sys.stderr)
        return 3

    report = build_report(recs, args, started_utc, warnings, aborted)
    print_summary(report)

    if args.out:
        path = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=False, default=str)
        print(f"wrote {path}", file=sys.stderr)

    if aborted:
        print(f"\nRUN ABORTED: {aborted}", file=sys.stderr)
        return 3
    cycles = ((report.get("line") or {}).get("arm_wait") or {}
              ).get("complete_cycles", 0) or 0
    if args.min_cycles and cycles < args.min_cycles:
        print(f"\nGATE FAILED: {cycles} complete ship cycle(s) observed, "
              f"--min-cycles {args.min_cycles} required. The run was too "
              f"short to conclude anything.", file=sys.stderr)
        return 5
    return 0


if __name__ == "__main__":
    sys.exit(main())
