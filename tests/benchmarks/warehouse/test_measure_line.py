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

"""Unit tests for measure_line.py's pure helpers — synthetic data only.

No simulator, no network, no files. Run either way:

    python tests/benchmarks/warehouse/test_measure_line.py
    python -m pytest tests/benchmarks/warehouse/test_measure_line.py

The heavy weighting toward the angle tests is deliberate: a +pi -> -pi wrap
scored as 360 degrees of turning is the single easiest way to make the
headline "unnecessary rotation" metric silently wrong, and it is wrong in the
direction that flatters an optimisation (a route change that alters how often
the tug crosses the wrap point would move the number without moving the robot).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from measure_line import (  # noqa: E402
    bucket_by, counter_events, describe, integrate_path, integrate_rotation,
    percentile, realtime_factor, run_stats, segment_runs, stationary_time,
    transit_runs, unwrap_series, wrap_pi, yaw_step,
)

TAU = 2.0 * math.pi


def close(a, b, tol=1e-9):
    assert abs(a - b) <= tol, f"{a!r} != {b!r} (tol {tol})"


# ── wrap_pi / yaw_step ────────────────────────────────────────────────

def test_wrap_pi_basics():
    close(wrap_pi(0.0), 0.0)
    close(wrap_pi(0.5), 0.5)
    close(wrap_pi(-0.5), -0.5)
    close(wrap_pi(TAU), 0.0, 1e-12)
    close(wrap_pi(3.5), 3.5 - TAU, 1e-12)
    close(wrap_pi(-3.5), TAU - 3.5, 1e-12)
    # +/- pi both land on a magnitude of pi; sign is immaterial for a step.
    close(abs(wrap_pi(math.pi)), math.pi, 1e-12)
    close(abs(wrap_pi(-math.pi)), math.pi, 1e-12)


def test_yaw_step_across_the_wrap():
    """THE headline case: a small turn that happens to cross +pi.

    Naive subtraction scores this as -6.20 rad (-355 deg). It is +0.08 rad.
    """
    prev, cur = 3.10, -3.10          # both wrapped, 0.0832 rad apart
    naive = cur - prev
    close(yaw_step(prev, cur), TAU - 6.20, 1e-12)
    assert abs(naive) > 6.0, "sanity: the naive value really is ~2pi wrong"
    assert abs(yaw_step(prev, cur)) < 0.1


def test_yaw_step_across_the_wrap_other_direction():
    close(yaw_step(-3.10, 3.10), -(TAU - 6.20), 1e-12)


def test_yaw_step_is_antisymmetric():
    for a, b in [(0.1, 0.9), (3.0, -3.0), (-2.0, 2.5), (0.0, 1.0)]:
        close(yaw_step(a, b), -yaw_step(b, a), 1e-12)


def test_unwrap_series_monotone_turn_through_pi():
    """A steady left turn sampled every 0.4 rad, crossing the wrap twice."""
    true = [0.4 * i for i in range(20)]          # 0 .. 7.6 rad, > one full turn
    wrapped = [wrap_pi(a) for a in true]
    un = unwrap_series(wrapped)
    for i in range(len(true)):
        close(un[i] - un[0], true[i] - true[0], 1e-9)


def test_full_circle_is_360_not_0_and_not_720():
    """One complete revolution, sampled at 8 points."""
    wrapped = [wrap_pi(TAU * i / 8.0) for i in range(9)]
    r = integrate_rotation(wrapped)
    close(math.degrees(r["total_rad"]), 360.0, 1e-6)
    close(math.degrees(r["net_rad"]), 360.0, 1e-6)


def test_standing_still_across_the_wrap_scores_zero():
    """A tug parked facing due-north, its yaw dithering across +/-pi.

    This is the regression that matters most: without wrapped differencing,
    a STATIONARY robot accumulates 360 deg per dither.
    """
    wrapped = [math.pi - 1e-6, -math.pi + 1e-6] * 200
    r = integrate_rotation(wrapped)
    assert math.degrees(r["total_rad"]) < 0.001, math.degrees(r["total_rad"])
    assert math.degrees(r["total_rad_raw"]) < 0.05


def test_back_and_forth_total_vs_net():
    """+90 then -90: 180 deg turned, 0 deg net. The 'wasted rotation' shape."""
    seq = [0.0, math.pi / 2, 0.0]
    r = integrate_rotation(seq)
    close(math.degrees(r["total_rad"]), 180.0, 1e-9)
    close(math.degrees(r["net_rad"]), 0.0, 1e-12)


def test_deadband_kills_noise_but_not_signal():
    noise = [1e-7 * ((-1) ** i) for i in range(1000)]
    r = integrate_rotation(noise, deadband_rad=3.5e-4)
    close(r["total_rad"], 0.0)
    assert r["total_rad_raw"] > 0.0          # the floor stays visible
    real = [0.0, 0.3, 0.6, 0.9]
    r2 = integrate_rotation(real, deadband_rad=3.5e-4)
    close(r2["total_rad"], 0.9, 1e-9)


def test_net_rotation_is_never_deadbanded():
    """A slow drift below the deadband must still register in `net`."""
    seq = [1e-5 * i for i in range(100)]
    r = integrate_rotation(seq, deadband_rad=3.5e-4)
    close(r["total_rad"], 0.0)
    close(r["net_rad"], 99e-5, 1e-12)


def test_aliasing_is_counted_not_hidden():
    r = integrate_rotation([0.0, 2.0, 4.0 - TAU])
    assert r["aliasing_suspect_steps"] == 2
    assert r["max_step_rad"] > math.pi / 2


def test_aliasing_beyond_pi_is_unrecoverable_and_we_say_so():
    """Documents the known limit rather than pretending it does not exist.

    Sampled too slowly, a +200 deg turn is indistinguishable from -160 deg.
    The metric under-reports; `aliasing_suspect_steps` is how you find out.
    """
    true_deg = 200.0
    wrapped = [0.0, wrap_pi(math.radians(true_deg))]
    r = integrate_rotation(wrapped)
    close(math.degrees(r["total_rad"]), 160.0, 1e-9)     # NOT 200
    assert r["aliasing_suspect_steps"] == 1


def test_nyquist_bound_for_the_omnitug500():
    """The concrete number the default --hz is chosen against."""
    w_max = 10.0 * 0.10 / 0.30            # v_max * r / ht = 3.333 rad/s
    nyquist_hz = w_max / math.pi
    close(nyquist_hz, 1.0610329539459689, 1e-9)
    assert 2.0 / nyquist_hz > 1.8         # the 2 Hz default clears it


# ── path length ───────────────────────────────────────────────────────

def test_path_length_square_loop():
    pts = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
    p = integrate_path(pts)
    close(p["path_m"], 4.0, 1e-12)
    close(p["net_m"], 0.0, 1e-12)


def test_path_length_straight_line_is_net():
    pts = [(0, 0), (1, 0), (2, 0), (3, 0)]
    p = integrate_path(pts)
    close(p["path_m"], 3.0, 1e-12)
    close(p["net_m"], 3.0, 1e-12)


def test_path_deadband():
    pts = [(0.0, 0.0)] + [(1e-6 * i, 0.0) for i in range(1, 500)]
    p = integrate_path(pts, deadband_m=1e-3)
    close(p["path_m"], 0.0)
    assert p["path_m_raw"] > 0.0


def test_path_single_sample():
    p = integrate_path([(1.0, 2.0)])
    close(p["path_m"], 0.0)
    close(p["net_m"], 0.0)


# ── percentile / describe ─────────────────────────────────────────────

def test_percentile():
    assert percentile([], 95) is None
    close(percentile([5.0], 95), 5.0)
    v = list(range(1, 101))                # 1..100
    close(percentile(v, 0), 1.0)
    close(percentile(v, 100), 100.0)
    close(percentile(v, 50), 50.5)
    close(percentile(v, 95), 95.05, 1e-9)


def test_describe():
    d = describe([1.0, 2.0, 3.0, 4.0])
    assert d["n"] == 4
    close(d["mean"], 2.5)
    close(d["median"], 2.5)
    close(d["min"], 1.0)
    close(d["max"], 4.0)
    close(d["total"], 10.0)
    e = describe([])
    assert e["n"] == 0 and e["mean"] is None and e["total"] == 0.0


# ── run segmentation ──────────────────────────────────────────────────

def test_segment_runs_midpoint_boundaries_and_censoring():
    series = [(0.0, "a"), (1.0, "a"), (2.0, "b"), (3.0, "b"), (4.0, "c")]
    runs = segment_runs(series)
    assert [r["key"] for r in runs] == ["a", "b", "c"]
    close(runs[0]["t_end"], 1.5)          # midpoint of 1.0 and 2.0
    close(runs[1]["t_start"], 1.5)
    close(runs[1]["t_end"], 3.5)
    close(runs[1]["duration"], 2.0)
    assert runs[0]["censored"] is True     # joined mid-run
    assert runs[1]["censored"] is False    # fully observed
    assert runs[2]["censored"] is True     # still running at the window end


def test_segment_runs_none_breaks_the_run():
    runs = segment_runs([(0.0, "a"), (1.0, None), (2.0, "a")])
    assert [r["key"] for r in runs] == ["a", "a"]


def test_segment_runs_empty_and_single():
    assert segment_runs([]) == []
    runs = segment_runs([(3.0, "a")])
    assert len(runs) == 1 and runs[0]["censored"] and runs[0]["duration"] == 0.0


def test_run_stats_excludes_censored_from_the_distribution():
    series = ([(0.0, "x"), (1.0, "y"), (2.0, "y"), (3.0, "y"), (4.0, "x"),
               (5.0, "y"), (6.0, "y"), (7.0, "x"), (8.0, "x")])
    stats = run_stats(segment_runs(series))
    assert stats["y"]["n"] == 2                    # both complete y runs
    assert stats["x"]["n"] == 1                    # first + last are censored
    assert stats["x"]["censored_runs"] == 2
    assert stats["x"]["total_observed_s"] > 0.0    # totals keep them


# ── counter events ────────────────────────────────────────────────────

def test_counter_events_intervals_exclude_the_partial_ends():
    # increments observed between t=1/2, t=5/6 and t=9/10 -> midpoints
    # 1.5, 5.5, 9.5 -> two complete intervals of 4.0.
    series = [(float(i), float(v)) for i, v in enumerate(
        [0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3])]
    c = counter_events(series)
    assert c["increments"] == 3
    close(c["delta"], 3.0)
    assert len(c["intervals"]) == 2
    for iv in c["intervals"]:
        close(iv, 4.0)


def test_counter_events_needs_two_increments_for_one_interval():
    c = counter_events([(0.0, 0.0), (1.0, 1.0), (2.0, 1.0)])
    assert c["increments"] == 1 and c["intervals"] == []


def test_counter_events_flags_a_reset_instead_of_a_negative_interval():
    c = counter_events([(0.0, 5.0), (1.0, 6.0), (2.0, 0.0), (3.0, 1.0)])
    assert c["resets"] == 1
    assert all(iv > 0 for iv in c["intervals"])


def test_counter_events_skips_missing_samples():
    c = counter_events([(0.0, 0.0), (1.0, None), (2.0, 1.0)])
    assert c["increments"] == 1


# ── stationary time ───────────────────────────────────────────────────

def test_stationary_time_parked_robot():
    t = [0.5 * i for i in range(21)]
    xy = [(1.0, 2.0)] * 21
    yaw = [0.3] * 21
    s = stationary_time(t, xy, yaw)
    close(s["stationary_s"], 10.0)
    close(s["moving_s"], 0.0)
    close(s["stationary_frac"], 1.0)


def test_spinning_on_the_spot_is_not_stationary():
    t = [0.5 * i for i in range(11)]
    xy = [(0.0, 0.0)] * 11
    yaw = [wrap_pi(0.5 * i) for i in range(11)]     # 1.0 rad/s
    s = stationary_time(t, xy, yaw)
    close(s["stationary_s"], 0.0)
    close(s["moving_s"], 5.0)


def test_half_moving_half_parked():
    t = [0.5 * i for i in range(21)]
    xy = [(0.3 * i, 0.0) for i in range(11)] + [(3.0, 0.0)] * 10
    yaw = [0.0] * 21
    s = stationary_time(t, xy, yaw)
    close(s["moving_s"], 5.0)
    close(s["stationary_s"], 5.0)
    close(s["max_speed_m_s"], 0.6, 1e-9)


# ── bucketing ─────────────────────────────────────────────────────────

def test_bucket_by_charges_a_step_to_its_start_key():
    t = [0.0, 1.0, 2.0, 3.0]
    xy = [(0, 0), (1, 0), (2, 0), (3, 0)]
    yaw = [0.0, 0.0, math.pi / 2, math.pi / 2]
    keys = ["towing", "towing", "empty", "empty"]
    b = bucket_by(t, xy, yaw, keys)
    close(b["towing"]["path_m"], 2.0, 1e-12)       # steps 0->1 and 1->2
    close(b["empty"]["path_m"], 1.0, 1e-12)
    close(b["towing"]["rotation_deg"], 90.0, 1e-9)  # the turn began while towing
    close(b["empty"]["rotation_deg"], 0.0)


# ── realtime factor ───────────────────────────────────────────────────

def test_realtime_factor():
    wall = [0.0, 1.0, 2.0, 3.0, 4.0]
    sim = [10.0, 10.5, 11.0, 11.5, 12.0]
    r = realtime_factor(wall, sim)
    close(r["factor"], 0.5)
    close(r["sim_span_s"], 2.0)
    close(r["stalled_wall_s"], 0.0)


def test_realtime_factor_detects_a_paused_sim():
    wall = [0.0, 1.0, 2.0, 3.0, 4.0]
    sim = [10.0, 10.0, 10.0, 11.0, 12.0]
    r = realtime_factor(wall, sim)
    close(r["stalled_wall_s"], 2.0)
    close(r["factor"], 0.5)


def test_realtime_factor_needs_two_points():
    assert realtime_factor([0.0], [1.0])["factor"] is None


# ── in-transit stage tracking ─────────────────────────────────────────

def test_transit_runs_tracks_two_boxes_independently():
    series = [
        (0.0, []),
        (1.0, [{"box": "BOX_1", "trolley": "T_A", "delivered": False}]),
        (2.0, [{"box": "BOX_1", "trolley": "T_A", "delivered": False},
               {"box": "BOX_2", "trolley": "T_B", "delivered": False}]),
        (3.0, [{"box": "BOX_1", "trolley": "T_A", "delivered": True},
               {"box": "BOX_2", "trolley": "T_B", "delivered": False}]),
        (4.0, [{"box": "BOX_2", "trolley": "T_B", "delivered": False}]),
        (5.0, []),
    ]
    runs = transit_runs(series)
    tow1 = [r for r in runs if r["box"] == "BOX_1" and r["key"] == "towing"]
    assert len(tow1) == 1 and not tow1[0]["censored"]
    close(tow1[0]["duration"], 2.0)                 # 0.5 -> 2.5
    disp1 = [r for r in runs if r["box"] == "BOX_1" and r["key"] == "at_dispatch"]
    assert len(disp1) == 1
    close(disp1[0]["duration"], 1.0)                # 2.5 -> 3.5
    tow2 = [r for r in runs if r["box"] == "BOX_2"]
    assert len(tow2) == 1
    close(tow2[0]["duration"], 3.0)                 # 1.5 -> 4.5


def test_transit_runs_marks_an_open_run_censored():
    series = [(0.0, [{"box": "B", "trolley": "T", "delivered": False}]),
              (1.0, [{"box": "B", "trolley": "T", "delivered": False}])]
    runs = transit_runs(series)
    assert len(runs) == 1 and runs[0]["censored"] is True


# ── an end-to-end synthetic line ──────────────────────────────────────

def test_synthetic_line_reproduces_its_own_ground_truth():
    """Fabricate a line whose numbers we already know, then measure it.

    Ground truth: a 100 s box cycle in which the box is at_fill for 45 s and
    then 'filled' (arm blocked, no cart) for 40 s. Three full cycles.
    """
    hz, cycle, fill_s, blocked_s = 2.0, 100.0, 45.0, 40.0
    dt = 1.0 / hz
    t, stage, shipped = [], [], []
    n = 0
    for i in range(int(3.5 * cycle * hz)):
        now = i * dt
        phase = now % cycle
        if phase < fill_s:
            s = "at_fill"
        elif phase < fill_s + blocked_s:
            s = "filled"
        else:
            s = "loading"
        if i > 0 and (now % cycle) < ((now - dt) % cycle):
            n += 1
        t.append(now)
        stage.append(s)
        shipped.append(float(n))

    runs = segment_runs(list(zip(t, stage)))
    stats = run_stats(runs)
    assert stats["filled"]["n"] >= 2
    close(stats["filled"]["mean"], blocked_s, 0.6)     # within one poll period
    close(stats["at_fill"]["mean"], fill_s, 0.6)

    c = counter_events(list(zip(t, shipped)))
    assert len(c["intervals"]) >= 2
    for iv in c["intervals"]:
        close(iv, cycle, 0.6)
    boxes_per_min = 60.0 * c["increment_total"] / (t[-1] - t[0])
    close(boxes_per_min, 60.0 / cycle, 0.15)


def run_all() -> int:
    """Run every test_* in this module. Returns a process exit code."""
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:                      # noqa: BLE001
            failed.append((name, e))
            print(f"  FAIL  {name}: {e}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    if failed:
        print("\nFAILURES:")
        for name, e in failed:
            print(f"  {name}: {e!r}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(run_all())
