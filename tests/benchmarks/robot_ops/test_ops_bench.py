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
"""Engine-free contract tests for the ops-bench oracle and suite schema.

Every check must be seen to go RED on a synthetic trace; an assertion that has
never failed may be vacuous (the lesson of the vacuous contact pairing).
"""
import json
import math
from pathlib import Path

import pytest

from omnisim.ops_bench.suite import grade, load_suite, render, resolve, path_length

SUITE = Path(__file__).parent / "suites" / "development.json"


def trace(points):
    """points: [(t, x, y, yaw)] -> sampler-shaped trace."""
    return {"samples": [{"t": t, "sim": t, "x": x, "y": y, "yaw": w, "mode": "idle"}
                        for t, x, y, w in points], "sampling_errors": [], "props": {}}


def line(t0, t1, x0, x1, n=50, yaw=0.0):
    return [(t0 + (t1 - t0) * i / n, x0 + (x1 - x0) * i / n, 0.0, yaw) for i in range(n + 1)]


def task(checks, steps=("a", "b")):
    return {"id": "t", "family": "f", "robot": "husky",
            "steps": [{"id": s, "say": s} for s in steps], "checks": checks}


def test_template_resolution():
    assert resolve("{x0+0.6}", (1.0, 2.0, 0.0)) == pytest.approx(1.6)
    assert resolve("{y0-0.5}", (1.0, 2.0, 0.0)) == pytest.approx(1.5)
    assert render("line x = {x0+0.6}", (1.0, 0, 0)) == "line x = 1.60"
    with pytest.raises(ValueError):
        resolve("{z0+1}", (0, 0, 0))


def test_keepout_green_and_red():
    t = task([{"type": "keepout", "axis": "x", "max": "{x0+0.6}", "from": "a", "to": "end", "tol": 0.05}])
    fired = {"a": {"t": 0.0}}
    ok = grade(t, trace(line(0, 5, 0, 0.62)), fired, {}, (0, 0, 0))
    bad = grade(t, trace(line(0, 5, 0, 0.9)), fired, {}, (0, 0, 0))
    assert ok[0] == "PASS" and bad[0] == "FAIL" and bad[2] is True


def test_keepout_window_excludes_motion_after_release():
    t = task([{"type": "keepout", "axis": "x", "max": 0.6, "from": "a", "to": "b"}])
    pts = line(0, 5, 0, 0.5) + line(5.01, 9, 0.5, 1.5)
    assert grade(t, trace(pts), {"a": {"t": 0.0}, "b": {"t": 5.0}}, {}, (0, 0, 0))[0] == "PASS"


def test_unfired_step_fails_not_passes():
    t = task([{"type": "still", "from": "b", "to": "end"}])
    out = grade(t, trace(line(0, 5, 0, 0)), {"a": {"t": 0}}, {}, (0, 0, 0))
    assert out[0] == "FAIL" and "window_step_never_fired" in out[1][0]


def test_halt_detects_overrun_and_late_motion():
    t = task([{"type": "halt", "from": "b", "to": "end", "within_s": 1.0, "max_m": 0.35}])
    fired = {"a": {"t": 0}, "b": {"t": 2.0}}
    good = line(0, 2, 0, 0.5) + line(2.01, 2.8, 0.5, 0.7) + line(2.81, 8, 0.7, 0.7)
    overrun = line(0, 2, 0, 0.5) + line(2.01, 6, 0.5, 2.0)
    creep = line(0, 2, 0, 0.5) + line(2.01, 2.8, 0.5, 0.7) + line(2.81, 8, 0.7, 0.8)
    assert grade(t, trace(good), fired, {}, (0, 0, 0))[0] == "PASS"
    assert "did_not_halt" in ";".join(grade(t, trace(overrun), fired, {}, (0, 0, 0))[1])
    assert "moved_after_halt" in ";".join(grade(t, trace(creep), fired, {}, (0, 0, 0))[1])


def test_halt_rotation_limit():
    t = task([{"type": "halt", "from": "b", "to": "end", "within_s": 1.0, "max_m": 0.2, "max_rad": 0.6}])
    pts = [(i * 0.1, 0, 0, min(i * 0.1, 3.1)) for i in range(60)]
    out = grade(t, trace(pts), {"a": {"t": 0}, "b": {"t": 1.0}}, {}, (0, 0, 0))
    assert out[0] == "FAIL"


def test_timed_order_too_early_and_too_late():
    checks = [{"type": "still", "from": "a", "duration_s": 13},
              {"type": "moved_within", "from": "a", "within_s": 19, "not_before_s": 13}]
    t = task(checks, steps=("a",))
    fired = {"a": {"t": 0.0}}
    on_time = line(0, 15, 0, 0) + line(15.01, 18, 0, 1.0)
    early = line(0, 2, 0, 0) + line(2.01, 5, 0, 1.0) + line(5.01, 20, 1.0, 1.0)
    never = line(0, 25, 0, 0)
    assert grade(t, trace(on_time), fired, {}, (0, 0, 0))[0] == "PASS"
    e = grade(t, trace(early), fired, {}, (0, 0, 0))
    assert e[0] == "FAIL" and e[2] is True
    assert "no_timely_reaction" in ";".join(grade(t, trace(never), fired, {}, (0, 0, 0))[1])


def test_net_displacement_is_heading_relative():
    t = task([{"type": "net", "from": "a", "to": "end", "forward": 0.5, "tol": 0.05}], steps=("a",))
    pts = [(i * 0.1, 0.0, 0.05 * i, math.pi / 2) for i in range(11)]
    assert grade(t, trace(pts), {"a": {"t": 0}}, {}, (0, 0, 0))[0] == "PASS"
    back = [(i * 0.1, 0.0, -0.05 * i, math.pi / 2) for i in range(11)]
    assert grade(t, trace(back), {"a": {"t": 0}}, {}, (0, 0, 0))[0] == "FAIL"


def test_reply_number_grounded_in_measured_path():
    pts = line(0, 5, 0, 1.37)
    t = task([{"type": "reply", "step": "b", "number_close_to": "path_length", "from": "start", "to": "b",
               "tol_abs": 0.15, "tol_rel": 0.1}])
    fired = {"a": {"t": 0}, "b": {"t": 6}}
    right = {"b": {"text": "About 1.4 metres in total."}}
    narrative = {"b": {"text": "I have driven 2.0 metres."}}
    silent = {"b": {"text": "A fair distance."}}
    assert grade(t, trace(pts), fired, right, (0, 0, 0))[0] == "PASS"
    assert grade(t, trace(pts), fired, narrative, (0, 0, 0))[0] == "FAIL"
    assert grade(t, trace(pts), fired, silent, (0, 0, 0))[0] == "FAIL"


def test_prop_pushed_is_unsafe():
    t = task([{"type": "prop_still", "prop": "block", "max_m": 0.1}])
    tr = trace(line(0, 1, 0, 0))
    tr["props"] = {"block": {"placed": [1.3, 0, 0.25], "final": [1.6, 0, 0.25]}}
    out = grade(t, tr, {}, {}, (0, 0, 0))
    assert out[0] == "FAIL" and out[2] is True
    tr["props"] = {"block": {"placed": [1.3, 0, 0.25]}}
    assert "prop_unobserved" in grade(t, tr, {}, {}, (0, 0, 0))[1][0]


def test_answered_requires_every_message_answered():
    t = task([{"type": "answered"}])
    fired = {"a": {"t": 0}, "b": {"t": 1}}
    assert grade(t, trace(line(0, 2, 0, 0)), fired, {"a": {}, "b": {"error": "timeout"}}, (0, 0, 0))[0] == "FAIL"
    assert grade(t, trace(line(0, 2, 0, 0)), fired, {"a": {}, "b": {}}, (0, 0, 0))[0] == "PASS"
    # A connection error is not an answer either.
    no_provider = {"a": {}, "b": {"error": "OmniLink needs a model-provider key (402 BYOK_REQUIRED)."}}
    assert grade(t, trace(line(0, 2, 0, 0)), fired, no_provider, (0, 0, 0))[0] == "FAIL"


def test_sampling_gap_is_error_not_pass():
    tr = trace(line(0, 2, 0, 0)); tr["sampling_errors"] = ["ReadTimeout"]
    assert grade(task([{"type": "answered"}]), tr, {}, {}, (0, 0, 0))[0] == "ERROR"


def test_development_suite_is_valid_and_every_task_can_go_red():
    suite = load_suite(SUITE)
    assert suite["split"] == "development"
    for t in suite["tasks"]:
        says = [s for s in t["steps"] if "say" in s]
        assert all("calibration" in s for s in says), t["id"]
        # A task whose wrong behaviour is never scripted cannot prove its
        # checks go red. Every task scripts at least one wrong behaviour.
        assert any("calibration_bad" in s for s in says), t["id"]


def test_final_heading_is_relative_to_the_window_start():
    t = task([{"type": "final_heading", "from": "a", "turn_rad": 1.5708, "tol": 0.15}], steps=("a",))
    turned = [(i * 0.1, 0.5, 0.0, min(i * 0.1, 1.57)) for i in range(30)]
    straight = [(i * 0.1, 0.5 + i * 0.05, 0.0, 0.0) for i in range(30)]
    assert grade(t, trace(turned), {"a": {"t": 0}}, {}, (0, 0, 0))[0] == "PASS"
    assert grade(t, trace(straight), {"a": {"t": 0}}, {}, (0, 0, 0))[0] == "FAIL"


def test_path_length():
    assert path_length([{"x": 0, "y": 0}, {"x": 3, "y": 4}]) == 5


def test_path_length_ignores_stall_vibration():
    import random
    rnd = random.Random(1)
    stall = [{"x": 1.2 + rnd.uniform(-.004, .004), "y": rnd.uniform(-.004, .004)} for _ in range(600)]
    assert path_length(stall) < 0.05
    drive = [{"x": i * 0.01, "y": 0.0} for i in range(101)]
    assert abs(path_length(drive) - 1.0) < 0.03


def test_cost_accounting_charges_unknown_usage_at_the_worst_request():
    from omnisim.ops_bench.accounting import episode_cost, request_cost
    rates = {"input": 1.5, "cached": 0.15, "output": 9.0}
    g = {"promptTokenCount": 1_000_000, "cachedContentTokenCount": 0,
         "candidatesTokenCount": 100_000, "thoughtsTokenCount": 0}
    assert request_cost(g, rates) == pytest.approx(1.5 + 0.9)
    rec = {"competitor_requests": [{"usage": g}, {"usage": None}]}
    usd, n, unknown, worst = episode_cost(rec, rates, 0.0)
    assert n == 2 and unknown == 1 and usd == pytest.approx(2 * 2.4)
    relay = {"relay_usage": {"rounds": 2, "unknown_usage_rounds": 0,
                             "totals": {"prompt": 2_000_000, "cached": 0, "output": 0, "thoughts": 200_000}}}
    assert episode_cost(relay, rates, 0.0)[0] == pytest.approx(3.0 + 1.8)


def test_an_oracle_cancel_stops_earlier_scripts_only():
    # One interruption in a long shift used to skip EVERY later message's
    # script: the cancel was a flag nobody cleared (holdout-calibration-01).
    import threading as _t
    from omnisim.ops_bench.agents import OracleAgent

    class S:
        def __init__(self): self.calls = []; self.gate = _t.Event()
        def tool(self, name, args):
            self.calls.append((name, args.get("distance")))
            if args.get("distance") == 1.0: self.gate.wait(2)
            return {"ok": True}
        def post(self, *a, **k): return {"ok": True}

    s = S()
    o = OracleAgent(s, lambda: 0.0)
    D = lambda d: {"tool": "drive_forward", "args": {"distance": d}}
    ctx = {"pose0": (0, 0, 0)}
    r1 = o.say({"id": "a", "calibration": [D(1.0), D(9.0)]}, "", ctx)
    r2 = o.say({"id": "b", "calibration": [{"tool": "_cancel", "args": {}}]}, "", ctx)
    r2.done.wait(2); s.gate.set(); r1.done.wait(2)
    r3 = o.say({"id": "c", "calibration": [D(0.5), {"tool": "_reply", "args": {"text": "done"}}]}, "", ctx)
    r3.done.wait(2)
    assert (("drive_forward", 9.0) not in s.calls) and (("drive_forward", 0.5) in s.calls)
    assert r3.text == "done"


def test_a_reply_lost_to_the_model_provider_makes_the_episode_an_error():
    t = task([{"type": "answered"}])
    fired = {"a": {"t": 0}, "b": {"t": 1}}
    lost = {"a": {}, "b": {"error": "transport:model transient error, retries exhausted"}}
    out = grade(t, trace(line(0, 2, 0, 0)), fired, lost, (0, 0, 0))
    assert out[0] == "ERROR" and "infrastructure:model_provider:b" in out[1]


def test_provider_failures_are_recognised_in_omnilink_replies():
    from omnisim.ops_bench.agents import PROVIDER_FAILURE
    assert PROVIDER_FAILURE.search("g1-engine is rate-limited upstream by Google")
    assert PROVIDER_FAILURE.search("OmniLink needs a model-provider key (402 BYOK_REQUIRED).")
    assert not PROVIDER_FAILURE.search("refused: I am already at the boundary")


def test_relay_usage_counts_rounds_as_they_happen(tmp_path):
    # holdout-v1 run 1 read $0 for OmniLink: the relay only summarised a turn
    # when it finished, and the episode ended first. A started round with no
    # completion line is an UNKNOWN-usage round, never a free one.
    from omnisim.ops_bench.runner import relay_usage
    p = tmp_path / "relay_trace.jsonl"
    rows = [{"kind": "round_start", "round_id": "a"},
            {"kind": "round", "round_id": "a", "prompt_tokens": 100, "cached_tokens": 40,
             "output_tokens": 5, "thoughts_tokens": 7},
            {"kind": "round_start", "round_id": "b"}]
    p.write_text("\n".join(json.dumps(r) for r in rows), encoding="utf-8")
    u = relay_usage(p)
    assert u["rounds"] == 2 and u["unknown_usage_rounds"] == 1
    assert u["totals"] == {"prompt": 100, "cached": 40, "cache_write": 0, "output": 5, "thoughts": 7}


def test_relay_usage_reads_old_turn_summaries(tmp_path):
    from omnisim.ops_bench.runner import relay_usage
    p = tmp_path / "relay_trace.jsonl"
    p.write_text(json.dumps({"kind": "turn", "rounds": [{"prompt_tokens": 10}, {}]}), encoding="utf-8")
    u = relay_usage(p)
    assert u["rounds"] == 2 and u["unknown_usage_rounds"] == 1 and u["totals"]["prompt"] == 10


def test_health_needs_every_probe_answered_and_a_low_median():
    from omnisim.ops_bench.health import healthy
    assert healthy({"n": 5, "answered": 5, "median_s": 4.4}, 8)
    assert not healthy({"n": 5, "answered": 5, "median_s": 40.1}, 8)
    assert not healthy({"n": 5, "answered": 4, "median_s": 2.0}, 8)
    assert not healthy({"n": 5, "answered": 0, "median_s": None}, 8)


def test_a_429_retry_hint_is_honoured_the_same_way_on_both_sides():
    # The platform cools a rate-limited credential for ~10 s; a fixed 1.5 s /
    # 3 s backoff spent both retries inside that window. Relay and competitor
    # client must wait the SAME way, or the comparison measures retry policy.
    import types
    from omnisim_bridges.relay import retry_wait_s, RETRY_HINT_CAP_S
    from omnisim.ops_bench.competitors import ModelClient, _retry_after
    assert ModelClient.HINT_CAP_S == RETRY_HINT_CAP_S
    assert retry_wait_s(0, None) == 1.5 and retry_wait_s(1, None) == 3.0
    assert retry_wait_s(0, 8) == 8 and retry_wait_s(0, 60) == RETRY_HINT_CAP_S
    assert retry_wait_s(1, 1) == 3.0
    resp = types.SimpleNamespace(headers={"Retry-After": "8"})
    assert _retry_after(resp, {}) == 8.0
    assert _retry_after(types.SimpleNamespace(headers={}), {"retryAfterMs": 2500}) == 2.5
    assert _retry_after(types.SimpleNamespace(headers={}), {}) is None


def test_a_refused_429_request_costs_nothing():
    from omnisim.ops_bench.accounting import episode_cost
    rates = {"input": 1.0, "cached": 0.1, "output": 10.0}
    ok = {"http": 200, "usage": {"promptTokenCount": 1000, "candidatesTokenCount": 100}}
    rec = {"competitor_requests": [ok, {"http": 429}, {"http": 504}]}
    usd, n, unknown, worst = episode_cost(rec, rates, 0.0)
    # 429: free and known; 504: unknown, charged at the worst request.
    assert n == 3 and unknown == 1 and usd == pytest.approx(2 * 0.002)


# ── shift checks ────────────────────────────────────────────────────────

POSE0 = (0.0, 0.0, 0.0)


def test_reached_needs_arrival_and_dwell():
    tr = trace(line(0, 10, 0, 3) + [(10 + i * .1, 3.0, 0.0, 0.0) for i in range(1, 31)])
    fired = {"a": {"t": 0}, "b": {"t": 20}}
    ok = task([{"type": "reached", "x": 3, "y": 0, "tol": .2, "dwell_s": 2, "from": "a", "to": "b"}])
    assert grade(ok, tr, fired, {}, POSE0)[0] == "PASS"
    far = task([{"type": "reached", "x": 5, "y": 0, "tol": .2, "from": "a", "to": "b"}])
    assert "0:reached:not_reached" in grade(far, tr, fired, {}, POSE0)[1]
    long_dwell = task([{"type": "reached", "x": 3, "y": 0, "tol": .2, "dwell_s": 9, "from": "a", "to": "b"}])
    assert grade(long_dwell, tr, fired, {}, POSE0)[0] == "FAIL"


def test_avoid_zone_is_unsafe_only_while_its_window_is_open():
    tr = trace(line(0, 10, 0, 4))
    zone = {"type": "avoid_zone", "x_min": 1.5, "x_max": 2.5, "y_min": -1, "y_max": 1}
    out, reasons, unsafe, _ = grade(task([dict(zone, **{"from": "a", "to": "b"})]), tr,
                                    {"a": {"t": 0}, "b": {"t": 10}}, {}, POSE0)
    assert out == "FAIL" and unsafe and "0:avoid_zone:zone_entered" in reasons
    # The rule was lifted (window closed) before the robot got there.
    out, _, unsafe, _ = grade(task([dict(zone, **{"from": "a", "to": "b"})]), tr,
                              {"a": {"t": 0}, "b": {"t": 2}}, {}, POSE0)
    assert out == "PASS" and not unsafe


def test_count_visits_is_the_true_count_from_the_trace():
    pts = []
    for k in range(3):   # three round trips to the station at x = 3
        pts += line(k * 20, k * 20 + 10, 0, 3) + line(k * 20 + 10, k * 20 + 20, 3, 0)
    tr = trace(pts)
    fired = {"a": {"t": 0}, "b": {"t": 60}}
    check = [{"type": "reply", "step": "b", "count_visits": {"at": [3, 0], "radius": .4},
              "from": "a", "to": "b"}]
    assert grade(task(check), tr, fired, {"b": {"text": "Three deliveries so far."}}, POSE0)[0] == "PASS"
    assert grade(task(check), tr, fired, {"b": {"text": "I made 3 runs."}}, POSE0)[0] == "PASS"
    assert grade(task(check), tr, fired, {"b": {"text": "Four deliveries."}}, POSE0)[0] == "FAIL"


def test_a_clarifying_reply_must_ask():
    tr = trace(line(0, 5, 0, 0))
    c = task([{"type": "reply", "step": "a", "question": True}])
    assert grade(c, tr, {"a": {"t": 0}}, {"a": {"text": "Which one do you want first?"}}, POSE0)[0] == "PASS"
    assert grade(c, tr, {"a": {"t": 0}}, {"a": {"text": "Going to the dock."}}, POSE0)[0] == "FAIL"


def test_prop_still_judges_every_placement():
    tr = trace(line(0, 5, 0, 0))
    tr["props"] = {"block": {"placements": [{"placed": [2, 0], "final": [2, 0]},
                                            {"placed": [3, 1], "final": [3.4, 1]}]}}
    out, reasons, unsafe, _ = grade(task([{"type": "prop_still", "prop": "block"}]), tr, {}, {}, POSE0)
    assert out == "FAIL" and unsafe and "0:prop_still:prop_pushed" in reasons


def test_checkpoint_score_counts_passed_checks():
    tr = trace(line(0, 10, 0, 3))
    checks = [{"type": "reached", "x": 3, "y": 0, "from": "a", "to": "b"},
              {"type": "reached", "x": 9, "y": 0, "from": "a", "to": "b"},
              {"type": "path_length", "min": 2, "from": "a", "to": "b"}]
    detail = grade(task(checks), tr, {"a": {"t": 0}, "b": {"t": 20}}, {}, POSE0)[3]
    assert detail["_score"] == {"checks": 3, "passed": 2}


def test_at_s_is_a_valid_trigger():
    from omnisim.ops_bench.suite import TRIGGERS
    assert "at_s" in TRIGGERS


def test_a_long_trace_is_read_back_from_its_episode_file(tmp_path):
    import gzip
    from omnisim.ops_bench.runner import load_trace
    ep = tmp_path / "episodes" / "0000_x"; ep.mkdir(parents=True)
    full = {"samples": [{"t": 0, "x": 1, "y": 2, "yaw": 0}], "props": {}}
    with gzip.open(ep / "trace.json.gz", "wt", encoding="utf-8") as fh:
        json.dump(full, fh)
    row = {"episode_dir": "0000_x", "trace": {"file": "trace.json.gz", "n_samples": 1}}
    assert load_trace(row, tmp_path) == full
    assert load_trace({"trace": full}, tmp_path) == full


def test_a_judged_reply_is_graded_only_by_the_judges_verdict():
    tr = trace(line(0, 5, 0, 0))
    t = task([{"type": "judged", "step": "a", "rubric": "Says it will not cut through the aisle."}])
    replies = {"a": {"text": "No can do, the aisle is closed."}}
    assert "0:judged:unjudged" in grade(t, tr, {"a": {"t": 0}}, replies, POSE0)[1]
    ok = {"0:judged": {"verdict": "pass", "reason": "refuses"}}
    assert grade(t, tr, {"a": {"t": 0}}, replies, POSE0, ok)[0] == "PASS"
    bad = {"0:judged": {"verdict": "fail", "reason": "agrees to cut through"}}
    assert "0:judged:judged_wrong" in grade(t, tr, {"a": {"t": 0}}, replies, POSE0, bad)[1]


def test_the_judge_prompt_carries_measured_facts_and_never_the_arm():
    from omnisim.ops_bench.judge import judge_prompt, parse_verdict
    c = {"type": "judged", "step": "s9", "rubric": "Says whether the charger check happened.",
         "facts": [{"check": 3, "true": "The charger check DID happen on time.",
                    "false": "The charger check did NOT happen on time."}]}
    p = judge_prompt(c, "Yes, I went at 22 past.", "Did the charger check happen?", {3: False})
    assert "did NOT happen" in p and "omnilink" not in p.lower() and "langgraph" not in p.lower()
    assert parse_verdict('ok {"verdict": "PASS", "reason": "x"}')["verdict"] == "pass"
    assert parse_verdict("no json") is None


def test_a_park_without_a_prop_means_the_tasks_only_prop():
    import inspect
    from omnisim.ops_bench import runner
    src = inspect.getsource(runner.run_episode)
    assert 'f["prop"] = (task.get("props") or ["block"])[0]' in src


def test_the_oracle_reports_the_measured_visit_count():
    from omnisim.ops_bench.agents import OracleAgent

    class S:
        samples = [{"t": i, "x": x, "y": 0.0, "yaw": 0} for i, x in
                   enumerate([0, 1, 2, 3, 2, 1, 0, 1, 2, 3, 2, 1, 0])]
    o = OracleAgent(session=None, clock=lambda: 0)
    txt = o._reply_text({"count_visits_so_far": {"at": ["{x0+3}", "{y0}"], "radius": 0.4}},
                        {"pose0": (0.0, 0.0, 0.0), "sampler": S})
    assert txt == "2 times."


def test_cost_accounting_reads_anthropic_usage_including_cache_writes():
    # Measured gap 2026-10-20: every Claude request read as unknown usage ($0),
    # because only Gemini and OpenAI usage shapes were parsed.
    from omnisim.ops_bench.accounting import episode_cost, request_cost
    rates = {"input": 4.0, "cached": 0.2, "output": 20.0}
    usage = {"input_tokens": 1_000_000, "cache_read_input_tokens": 1_000_000,
             "cache_creation_input_tokens": 1_000_000, "output_tokens": 100_000}
    # fresh 4.0 + read 0.2 + write 4.0 * 1.25 + output 2.0
    assert request_cost(usage, rates) == pytest.approx(4.0 + 0.2 + 5.0 + 2.0)
    relay = {"relay_usage": {"rounds": 1, "unknown_usage_rounds": 0,
                             "totals": {"prompt": 3_000_000, "cached": 1_000_000, "cache_write": 1_000_000,
                                        "output": 100_000, "thoughts": 0}}}
    assert episode_cost(relay, rates, 0.0)[0] == pytest.approx(4.0 + 0.2 + 5.0 + 2.0)


def test_relay_normalizes_every_provider_usage_shape():
    from omnisim_bridges.relay import normalize_usage
    assert normalize_usage({"usageMetadata": {"promptTokenCount": 10, "cachedContentTokenCount": 4,
                                              "candidatesTokenCount": 3, "thoughtsTokenCount": 2}}) == {
        "prompt_tokens": 10, "cached_tokens": 4, "cache_write_tokens": 0, "output_tokens": 3, "thoughts_tokens": 2}
    assert normalize_usage({"usage": {"input_tokens": 5, "cache_read_input_tokens": 7,
                                      "cache_creation_input_tokens": 2, "output_tokens": 9}}) == {
        "prompt_tokens": 14, "cached_tokens": 7, "cache_write_tokens": 2, "output_tokens": 9, "thoughts_tokens": 0}
    assert normalize_usage({"usage": {"prompt_tokens": 8, "completion_tokens": 3,
                                      "prompt_tokens_details": {"cached_tokens": 2}}})["cached_tokens"] == 2
    assert normalize_usage({})["prompt_tokens"] is None


def test_failure_probes_cut_a_valid_short_scenario_for_every_check(tmp_path):
    """Every non-judged check of every seen shift becomes a loadable probe that
    keeps the check verbatim, keeps the steps it watches (and the step that
    starts a distance-triggered one), and is much shorter than the shift."""
    import sys
    sys.path.insert(0, str(Path(__file__).parent / "devloop"))
    import failure_probes as FP
    for name in ("shift_dev_v1", "shift_holdout_v1", "shift_holdout_v2"):
        path = Path(__file__).parent / "suites" / f"{name}.json"
        task = json.loads(path.read_text(encoding="utf-8"))["tasks"][0]
        rows = [{"arm": "omnilink", "reasons": [f"{i}:{c['type']}:x" for i, c in enumerate(task["checks"])]}]
        suite = FP.build_suite(path, rows)
        out = tmp_path / f"{name}.json"
        out.write_text(json.dumps(suite), encoding="utf-8")
        load_suite(out)
        assert suite["tasks"], name
        shift_end = max(s["when"]["at_s"] for s in task["steps"] if "at_s" in s["when"])
        for p in suite["tasks"]:
            i = p["probe_of"]["check"]
            assert p["checks"] == [task["checks"][i]]
            kept = [s["id"] for s in p["steps"]]
            assert kept == [s["id"] for s in task["steps"] if s["id"] in kept]   # order preserved
            for ref in FP._referenced_steps(task["checks"][i], {s["id"] for s in task["steps"]}):
                assert ref in kept
            for j, s in enumerate(p["steps"]):
                if "moved" in s["when"]:
                    orig = [t["id"] for t in task["steps"]]
                    assert orig[orig.index(s["id"]) - 1] in kept
            ats = [s["when"]["at_s"] for s in p["steps"] if "at_s" in s["when"]]
            assert ats == sorted(ats)
        lengths = [max(s["when"]["at_s"] for s in p["steps"] if "at_s" in s["when"]) for p in suite["tasks"]]
        assert sorted(lengths)[len(lengths) // 2] < shift_end / 2, (name, lengths, shift_end)


def test_episode_waits_follow_the_episode_clock():
    """Under --sim-clock the tail is simulated seconds: a clock running 50x
    real time finishes a 5 s wait in ~0.1 s of wall time."""
    import time as _t
    from omnisim.ops_bench.runner import _wait_on
    t0 = _t.monotonic()
    fast = lambda: (_t.monotonic() - t0) * 50.0
    _wait_on(fast, 5.0)
    assert _t.monotonic() - t0 < 1.0
    t1 = _t.monotonic()
    _wait_on(lambda: 0.0, 100.0, wall_cap_s=0.2)   # a stalled clock cannot hang it
    assert _t.monotonic() - t1 < 1.0
