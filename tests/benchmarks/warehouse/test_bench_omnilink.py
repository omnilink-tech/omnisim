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

"""Unit tests for bench_omnilink.py's scoring predicates -- synthetic only.

No simulator, no network, no files. Run either way:

    python tests/benchmarks/warehouse/test_bench_omnilink.py
    python tests/benchmarks/warehouse/bench_omnilink.py --selftest
    python -m pytest tests/benchmarks/warehouse/test_bench_omnilink.py

The weighting is deliberate. The tests named `test_ADVERSARIAL_*` are the
reason this harness exists: they feed a predicate a fluent, confident reply
claiming the robot did the thing, alongside a before/after pair showing it did
NOT, and assert the verdict is a hard failure. If any of them ever go green
while the predicate returns "pass", the harness has stopped measuring the
robot and started grading prose -- which is the exact failure mode documented
in docs/developer/tool-design-for-agents.md.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bench_omnilink import (  # noqa: E402
    ENGINE_CLOUD, ENGINE_CLOUD_FALLBACK, ENGINE_LOCAL_OLLAMA, ENGINE_NONE,
    ENGINE_UNVERIFIED, EXIT_ENGINE, EXIT_IDENTITY, EXIT_LIVENESS,
    PREDICATES, ROLES, Redactor, SUITE, actions_of, check_number_in_text,
    classify_engine, classify_liveness, classify_resume, compare_identity,
    engine_gate, extract_numbers, frame_delta, identity_of, is_busy,
    joints_of, liveness_remedy, mode_warnings, parse_netstat_listeners,
    scan_sim_monotonicity,
    paused_of, pose_of, pred_at_rest, pred_joints_at_rest, pred_not_parked,
    pred_net_translation, pred_resumed, pred_translate_then_rotate,
    reply_of, resolve_truth, resolve_verify_s, score_probes,
    series_speeds, slice_recorder, suite_fingerprint,
)
import measure_line as ml  # noqa: E402


# ── helpers ───────────────────────────────────────────────────────────

def close(a, b, tol=1e-6):
    assert abs(a - b) <= tol, f"{a!r} != {b!r} (tol {tol})"


def mk_pose(x=0.0, y=0.0, yaw=0.0, paused=None, extra=None):
    st = {"id": "tug_b", "x": x, "y": y, "yaw": yaw,
          "v_linear": 0.0, "v_angular": 0.0, "mode": "idle"}
    if paused is not None:
        st["idle_loop"] = {"leg": "idle", "paused": bool(paused),
                           "park_row_count": 3}
    if extra:
        st.update(extra)
    return st


def mk_arm(q, paused=None):
    st = {"id": "omniarm6", "q": list(q), "tcp": [0.4, 0.0, 0.3]}
    if paused is not None:
        st["idle_loop"] = {"mode": "pick", "picks": 4, "leg": "pick",
                           "paused": bool(paused)}
    return st


def still_series(x=0.0, y=0.0, yaw=0.0, n=21, dt=0.5, paused=None):
    return [{"t": i * dt, "state": mk_pose(x, y, yaw, paused)}
            for i in range(n)]


def moving_series(v=0.4, yaw=0.0, n=21, dt=0.5, paused=None):
    return [{"t": i * dt,
             "state": mk_pose(v * i * dt * math.cos(yaw),
                              v * i * dt * math.sin(yaw), yaw, paused)}
            for i in range(n)]


def paused_series(flip_at_t, n=200, dt=0.5):
    """A series whose idle_loop.paused goes True -> False at `flip_at_t`.
    `flip_at_t` None means it never un-pauses."""
    out = []
    for i in range(n):
        t = i * dt
        p = True if (flip_at_t is None or t < flip_at_t) else False
        out.append({"t": t, "state": mk_pose(0.0, 0.0, 0.0, paused=p)})
    return out


def probe(key="p", predicate="at_rest", params=None, **kw):
    d = {"key": key, "tier": "t", "target": "tug", "text": "x",
         "predicate": predicate, "params": params or {}}
    d.update(kw)
    return d


# ── accessors ─────────────────────────────────────────────────────────

def test_pose_of_reads_a_mobile_state():
    p = pose_of(mk_pose(1.5, -2.0, 0.75))
    assert p is not None
    close(p[0], 1.5)
    close(p[1], -2.0)
    close(p[2], 0.75)


def test_pose_of_rejects_junk():
    assert pose_of(None) is None
    assert pose_of({}) is None
    assert pose_of({"x": 1.0, "y": 2.0}) is None                 # no yaw
    assert pose_of({"x": 1.0, "y": 2.0, "yaw": None}) is None
    assert pose_of({"x": float("nan"), "y": 0.0, "yaw": 0.0}) is None
    assert pose_of({"x": True, "y": 0.0, "yaw": 0.0}) is None    # bool != number


def test_joints_of():
    assert joints_of(mk_arm([0.1, 0.2, 0.3])) == [0.1, 0.2, 0.3]
    assert joints_of({"q": []}) is None
    assert joints_of({"q": [0.1, None]}) is None
    assert joints_of(mk_pose()) is None


def test_paused_of_distinguishes_absent_from_false():
    # A robot with NO idle loop must read None, not False: "not paused" and
    # "has nothing to pause" are different claims.
    assert paused_of(mk_pose(paused=True)) is True
    assert paused_of(mk_pose(paused=False)) is False
    assert paused_of(mk_pose()) is None
    assert paused_of(None) is None


# ── geometry ──────────────────────────────────────────────────────────

def test_frame_delta_forward_along_heading():
    d = frame_delta((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    close(d["forward_m"], 1.0)
    close(d["lateral_m"], 0.0)
    close(d["dist_m"], 1.0)
    close(d["dyaw_deg"], 0.0)


def test_frame_delta_forward_is_heading_relative_not_world_x():
    # Facing +y, a move to +y is FORWARD -- the whole reason the predicate
    # resolves into the body frame instead of trusting world coordinates.
    d = frame_delta((0.0, 0.0, math.pi / 2), (0.0, 1.0, math.pi / 2))
    close(d["forward_m"], 1.0, 1e-9)
    close(abs(d["lateral_m"]), 0.0, 1e-9)


def test_frame_delta_sideways_is_not_forward():
    d = frame_delta((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    close(d["forward_m"], 0.0, 1e-9)
    close(d["lateral_m"], 1.0)
    close(d["bearing_err_deg"], 90.0, 1e-6)


def test_frame_delta_reverse_is_negative_forward():
    d = frame_delta((0.0, 0.0, 0.0), (-0.5, 0.0, 0.0))
    close(d["forward_m"], -0.5)
    close(d["dist_m"], 0.5)


def test_frame_delta_yaw_wraps():
    # +170 deg -> -170 deg is a +20 deg step, not -340.
    d = frame_delta((0.0, 0.0, math.radians(170)),
                    (0.0, 0.0, math.radians(-170)))
    close(d["dyaw_deg"], 20.0, 1e-6)


def test_series_speeds_from_poses():
    s = moving_series(v=0.4, n=5, dt=0.5)
    sp = series_speeds(s)
    assert len(sp) == 4
    for v in sp:
        close(v, 0.4, 1e-6)


def test_series_speeds_ignores_commanded_velocity():
    # v_linear says 1.0 m/s; the poses say the robot did not move. The
    # predicate must believe the poses.
    s = [{"t": i * 0.5, "state": mk_pose(0.0, 0.0, 0.0,
                                         extra={"v_linear": 1.0})}
         for i in range(5)]
    assert max(series_speeds(s)) == 0.0


# ── pred_at_rest ──────────────────────────────────────────────────────

def test_at_rest_passes_when_stationary():
    r = pred_at_rest(probe(), {"series": still_series(),
                               "pre_speed_m_s": 0.4})
    assert r["verdict"] == "pass", r
    assert r["measured"]["max_speed_m_s"] == 0.0
    assert r["measured"]["trivially_satisfied"] is False


def test_ADVERSARIAL_at_rest_fails_when_the_robot_keeps_driving():
    # The bridge answered 200 and the model said "Stopping wheels." The tug
    # kept rolling at 0.4 m/s. That is a FAIL.
    r = pred_at_rest(probe(), {"series": moving_series(v=0.4),
                               "pre_speed_m_s": 0.4})
    assert r["verdict"] == "fail", r
    assert r["measured"]["max_speed_m_s"] > 0.3


def test_at_rest_flags_a_trivially_satisfied_stop():
    # Stopping an already-parked robot proves nothing. Still a pass (it IS at
    # rest), but the flag must be set so a reader can discount it.
    r = pred_at_rest(probe(), {"series": still_series(),
                               "pre_speed_m_s": 0.0})
    assert r["verdict"] == "pass"
    assert r["measured"]["trivially_satisfied"] is True


def test_at_rest_is_inconclusive_without_enough_samples():
    r = pred_at_rest(probe(), {"series": still_series(n=2, dt=0.5)})
    assert r["verdict"] == "inconclusive", r


def test_at_rest_ignores_motion_inside_the_grace_period():
    # Braking is allowed: samples before grace_s are excluded.
    s = [{"t": 0.0, "state": mk_pose(0.0, 0.0)},
         {"t": 1.0, "state": mk_pose(0.3, 0.0)},
         {"t": 2.0, "state": mk_pose(0.5, 0.0)}]
    s += [{"t": 2.5 + i * 0.5, "state": mk_pose(0.5, 0.0)} for i in range(8)]
    r = pred_at_rest(probe(params={"grace_s": 2.5}), {"series": s})
    assert r["verdict"] == "pass", r


# ── pred_net_translation ──────────────────────────────────────────────

def test_net_translation_passes_on_an_accurate_metre():
    r = pred_net_translation(
        probe(predicate="net_translation",
              params={"target_m": 1.0, "dist_tol_m": 0.15,
                      "yaw_tol_deg": 12.0}),
        {"before": mk_pose(0.0, 0.0, 0.0), "after": mk_pose(0.98, 0.0, 0.0)})
    assert r["verdict"] == "pass", r
    close(r["measured"]["error_m"], -0.02, 1e-6)


def test_ADVERSARIAL_reply_claims_drive_but_pose_unchanged():
    # THE CANONICAL FAILURE. The model replies "Driving forward 1 m (~4.2s)."
    # and the tug is exactly where it started. Score must be 0.
    reply = "Driving forward 1.00 m now (~4.2 s). Done."
    r = pred_net_translation(
        probe(predicate="net_translation",
              params={"target_m": 1.0, "dist_tol_m": 0.15,
                      "yaw_tol_deg": 12.0}),
        {"before": mk_pose(3.0, -1.0, 0.5), "after": mk_pose(3.0, -1.0, 0.5),
         "reply": reply})
    assert r["verdict"] == "fail", r
    close(r["measured"]["forward_m"], 0.0)
    close(r["measured"]["error_m"], -1.0)
    # And the predicate must not have looked at the text at all.
    assert reply not in repr(r)


def test_net_translation_fails_when_it_drove_the_wrong_way():
    r = pred_net_translation(
        probe(predicate="net_translation",
              params={"target_m": 1.0, "dist_tol_m": 0.15,
                      "yaw_tol_deg": 12.0}),
        {"before": mk_pose(0.0, 0.0, 0.0), "after": mk_pose(-1.0, 0.0, 0.0)})
    assert r["verdict"] == "fail", r


def test_net_translation_fails_on_a_metre_sideways():
    # 1.0 m of straight-line displacement, none of it forward.
    r = pred_net_translation(
        probe(predicate="net_translation",
              params={"target_m": 1.0, "dist_tol_m": 0.15,
                      "yaw_tol_deg": 12.0}),
        {"before": mk_pose(0.0, 0.0, 0.0), "after": mk_pose(0.0, 1.0, 0.0)})
    assert r["verdict"] == "fail", r


def test_net_translation_fails_when_the_heading_swung():
    r = pred_net_translation(
        probe(predicate="net_translation",
              params={"target_m": 1.0, "dist_tol_m": 0.15,
                      "yaw_tol_deg": 12.0}),
        {"before": mk_pose(0.0, 0.0, 0.0),
         "after": mk_pose(1.0, 0.0, math.radians(40))})
    assert r["verdict"] == "fail", r
    subs = {s["name"]: s["ok"] for s in r["subgoals"] if s["ok"] is not None}
    assert subs["forward_distance"] is True
    assert subs["heading_held"] is False


def test_net_translation_inconclusive_without_snapshots():
    r = pred_net_translation(probe(predicate="net_translation"),
                             {"before": None, "after": mk_pose()})
    assert r["verdict"] == "inconclusive"


# ── pred_translate_then_rotate ────────────────────────────────────────

_COMPOSE = probe(predicate="translate_then_rotate",
                 params={"target_m": -0.5, "target_deg": 45.0,
                         "dist_tol_m": 0.18, "yaw_tol_deg": 12.0})


def test_compose_passes_when_both_subgoals_are_met():
    # Reversed 0.5 m along +x heading, then turned +45 deg.
    r = pred_translate_then_rotate(
        _COMPOSE, {"before": mk_pose(0.0, 0.0, 0.0),
                   "after": mk_pose(-0.5, 0.0, math.radians(45))})
    assert r["verdict"] == "pass", r
    assert r["measured"]["subgoals_met"] == "2/2"
    assert r["measured"]["inferred_order"] == "translate_then_rotate"


def test_compose_fails_the_offline_router_shape_rotation_only():
    # This is EXACTLY what the offline regex router does with "back up half a
    # metre, then turn left 45 degrees": its turn-regex fires and returns, so
    # the reverse never happens. Partial credit is recorded, verdict is fail.
    r = pred_translate_then_rotate(
        _COMPOSE, {"before": mk_pose(0.0, 0.0, 0.0),
                   "after": mk_pose(0.0, 0.0, math.radians(45))})
    assert r["verdict"] == "fail", r
    assert r["measured"]["subgoals_met"] == "1/2"
    subs = {s["name"]: s["ok"] for s in r["subgoals"] if s["ok"] is not None}
    assert subs["rotation"] is True
    assert subs["translation"] is False


def test_compose_fails_translation_only():
    r = pred_translate_then_rotate(
        _COMPOSE, {"before": mk_pose(0.0, 0.0, 0.0),
                   "after": mk_pose(-0.5, 0.0, 0.0)})
    assert r["verdict"] == "fail", r
    assert r["measured"]["subgoals_met"] == "1/2"


def test_ADVERSARIAL_compose_fails_when_nothing_moved():
    r = pred_translate_then_rotate(
        _COMPOSE, {"before": mk_pose(2.0, 2.0, 1.0),
                   "after": mk_pose(2.0, 2.0, 1.0)})
    assert r["verdict"] == "fail", r
    assert r["measured"]["subgoals_met"] == "0/2"


def test_compose_infers_the_order_from_the_displacement_bearing():
    # Turn first, THEN reverse: displacement lies along the NEW heading.
    yaw = math.radians(45)
    r = pred_translate_then_rotate(
        _COMPOSE, {"before": mk_pose(0.0, 0.0, 0.0),
                   "after": mk_pose(-0.5 * math.cos(yaw),
                                    -0.5 * math.sin(yaw), yaw)})
    assert r["verdict"] == "pass", r          # end state is what was asked
    assert r["measured"]["inferred_order"] == "rotate_then_translate"


def test_compose_rotation_wraps_across_the_pi_boundary():
    # -175 deg -> +175 deg is a -10 deg turn, NOT +350. A predicate that
    # subtracted the raw angles would report a 340 deg error and fail a robot
    # that did exactly what it was told.
    p = probe(predicate="translate_then_rotate",
              params={"target_m": 0.0, "target_deg": -10.0,
                      "dist_tol_m": 0.18, "yaw_tol_deg": 12.0})
    ev = {"before": mk_pose(0.0, 0.0, math.radians(-175)),
          "after": mk_pose(0.0, 0.0, math.radians(175))}
    r = pred_translate_then_rotate(p, ev)
    close(r["measured"]["dyaw_deg"], -10.0, 1e-6)
    close(r["measured"]["yaw_error_deg"], 0.0, 1e-6)
    assert r["verdict"] == "pass", r
    # ... and the same crossing against a +10 deg target is a real failure,
    # scored on the wrapped -20 deg error rather than a phantom 340.
    p2 = probe(predicate="translate_then_rotate",
               params={"target_m": 0.0, "target_deg": 10.0,
                       "dist_tol_m": 0.18, "yaw_tol_deg": 12.0})
    r2 = pred_translate_then_rotate(p2, ev)
    assert r2["verdict"] == "fail", r2
    close(r2["measured"]["yaw_error_deg"], -20.0, 1e-6)


# ── pred_not_parked ───────────────────────────────────────────────────

def test_not_parked_passes_when_a_question_left_it_working():
    r = pred_not_parked(probe(predicate="not_parked"),
                        {"before": mk_pose(0, 0, 0, paused=False),
                         "after": mk_pose(1, 0, 0, paused=False)})
    assert r["verdict"] == "pass", r
    close(r["measured"]["moved_m"], 1.0)


def test_not_parked_fails_when_the_question_parked_the_robot():
    r = pred_not_parked(probe(predicate="not_parked"),
                        {"before": mk_pose(0, 0, 0, paused=False),
                         "after": mk_pose(0, 0, 0, paused=True)})
    assert r["verdict"] == "fail", r


def test_not_parked_is_inconclusive_when_it_was_already_parked():
    # No free pass: if it was parked before, nothing was preserved.
    r = pred_not_parked(probe(predicate="not_parked"),
                        {"before": mk_pose(0, 0, 0, paused=True),
                         "after": mk_pose(0, 0, 0, paused=True)})
    assert r["verdict"] == "inconclusive", r


def test_not_parked_is_inconclusive_without_an_idle_loop():
    r = pred_not_parked(probe(predicate="not_parked"),
                        {"before": mk_pose(), "after": mk_pose()})
    assert r["verdict"] == "inconclusive", r


def test_not_parked_allows_the_robot_to_keep_driving():
    # A question must NOT be scored on stillness -- the idle loop is supposed
    # to keep working through it.
    r = pred_not_parked(probe(predicate="not_parked"),
                        {"before": mk_pose(0, 0, 0, paused=False),
                         "after": mk_pose(12.0, -4.0, 2.0, paused=False)})
    assert r["verdict"] == "pass", r


# ── pred_joints_at_rest ───────────────────────────────────────────────

def test_joints_at_rest_passes_when_held():
    s = [{"t": i * 0.5, "state": mk_arm([0.1, 0.2, 0.3])} for i in range(20)]
    r = pred_joints_at_rest(probe(predicate="joints_at_rest"), {"series": s})
    assert r["verdict"] == "pass", r


def test_ADVERSARIAL_joints_at_rest_fails_while_the_arm_keeps_picking():
    s = [{"t": i * 0.5, "state": mk_arm([0.1 + 0.05 * i, 0.2, 0.3])}
         for i in range(20)]
    r = pred_joints_at_rest(probe(predicate="joints_at_rest"), {"series": s})
    assert r["verdict"] == "fail", r
    assert r["measured"]["joint_spread_rad"] > 0.1


def test_joints_at_rest_inconclusive_without_samples():
    r = pred_joints_at_rest(probe(predicate="joints_at_rest"),
                            {"series": [{"t": 9.0, "state": mk_pose()}]})
    assert r["verdict"] == "inconclusive"


# ── classify_resume  (THE DISCRIMINATOR) ──────────────────────────────

def test_classify_resume_credits_a_fast_tool_call():
    c = classify_resume(t_resume_s=3.0, latency_s=2.0, resume_s=60.0,
                        t_arm_before_send_s=3.0)
    assert c["cause"] == "tool", c
    close(c["time_to_resume_after_reply_s"], 1.0, 1e-6)


def test_classify_resume_credits_a_slow_but_still_early_tool_call():
    # A 20 s LLM turn that un-paused at 22 s is still the agent: 22 s is
    # nowhere near the 57 s timer.
    c = classify_resume(t_resume_s=22.0, latency_s=20.0, resume_s=60.0,
                        t_arm_before_send_s=3.0)
    assert c["cause"] == "tool", c


def test_ADVERSARIAL_timer_resume_is_not_credited_to_the_agent():
    # The robot DID come back -- 57 s later, when the quiet window expired.
    # Nothing the agent did caused it. Must not read as a success.
    c = classify_resume(t_resume_s=57.0, latency_s=2.0, resume_s=60.0,
                        t_arm_before_send_s=3.0)
    assert c["cause"] == "auto_timer", c


def test_classify_resume_reports_never():
    c = classify_resume(t_resume_s=None, latency_s=2.0, resume_s=60.0,
                        t_arm_before_send_s=3.0)
    assert c["cause"] == "none", c
    assert c["time_to_resume_s"] is None


def test_classify_resume_says_unclear_in_the_middle():
    c = classify_resume(t_resume_s=30.0, latency_s=2.0, resume_s=60.0,
                        t_arm_before_send_s=3.0)
    assert c["cause"] == "unclear", c


def test_classify_resume_refuses_to_guess_when_the_model_was_too_slow():
    # A 55 s turn against a 57 s timer: a tool call and the clock are not
    # separable. Saying "the agent did it" here would be a fabrication.
    c = classify_resume(t_resume_s=56.0, latency_s=55.0, resume_s=60.0,
                        t_arm_before_send_s=3.0)
    assert c["cause"] == "unseparable", c


def test_classify_resume_boundaries_are_the_documented_ones():
    c = classify_resume(t_resume_s=None, latency_s=2.0, resume_s=60.0,
                        t_arm_before_send_s=4.0, tool_grace_s=5.0,
                        timer_tol_s=8.0)
    close(c["timer_due_s"], 56.0)          # min(60, 60-4)
    close(c["tool_bound_s"], 7.0)          # latency 2 + grace 5
    close(c["timer_bound_s"], 48.0)        # 56 - 8


# ── pred_resumed ──────────────────────────────────────────────────────

_RESUME = probe(predicate="resumed")


def test_resumed_passes_when_the_agent_called_the_tool():
    ev = {"before": mk_pose(paused=True), "series": paused_series(3.0),
          "latency_s": 2.0, "resume_s": 60.0, "t_arm_before_send_s": 3.0}
    r = pred_resumed(_RESUME, ev)
    assert r["verdict"] == "pass", r
    assert r["measured"]["cause"] == "tool"


def test_ADVERSARIAL_reply_claims_resume_but_the_robot_stays_parked():
    # "Back on it -- rejoining the line now." and idle_loop.paused never
    # goes False. THE headline failure this harness was built to catch.
    ev = {"before": mk_pose(paused=True), "series": paused_series(None),
          "latency_s": 2.0, "resume_s": 60.0, "t_arm_before_send_s": 3.0,
          "reply": "Back on it -- rejoining the line now."}
    r = pred_resumed(_RESUME, ev)
    assert r["verdict"] == "fail", r
    assert r["measured"]["cause"] == "none"
    assert r["measured"]["time_to_resume_s"] is None


def test_ADVERSARIAL_waiting_out_the_timer_is_a_fail_not_a_pass():
    ev = {"before": mk_pose(paused=True), "series": paused_series(57.0),
          "latency_s": 2.0, "resume_s": 60.0, "t_arm_before_send_s": 3.0}
    r = pred_resumed(_RESUME, ev)
    assert r["verdict"] == "fail", r
    assert r["measured"]["cause"] == "auto_timer"
    # The robot IS working again -- the harness must still refuse to credit it.
    assert r["measured"]["time_to_resume_s"] == 57.0


def test_resumed_inconclusive_when_it_was_never_paused():
    ev = {"before": mk_pose(paused=False), "series": paused_series(1.0),
          "latency_s": 2.0, "resume_s": 60.0, "t_arm_before_send_s": 3.0}
    r = pred_resumed(_RESUME, ev)
    assert r["verdict"] == "inconclusive", r


def test_resumed_inconclusive_without_an_idle_loop():
    ev = {"before": mk_pose(), "series": [], "latency_s": 1.0,
          "resume_s": 60.0, "t_arm_before_send_s": 0.0}
    assert pred_resumed(_RESUME, ev)["verdict"] == "inconclusive"


def test_resumed_inconclusive_when_the_turn_outran_the_timer():
    ev = {"before": mk_pose(paused=True), "series": paused_series(56.0),
          "latency_s": 55.0, "resume_s": 60.0, "t_arm_before_send_s": 3.0}
    r = pred_resumed(_RESUME, ev)
    assert r["verdict"] == "inconclusive", r
    assert r["measured"]["cause"] == "unseparable"


def test_resumed_takes_the_FIRST_unpause_not_the_last():
    # paused goes False at 4 s. A later re-pause must not move the number.
    s = paused_series(4.0, n=40)
    for e in s:
        if e["t"] > 12.0:
            e["state"]["idle_loop"]["paused"] = True
    ev = {"before": mk_pose(paused=True), "series": s, "latency_s": 2.0,
          "resume_s": 60.0, "t_arm_before_send_s": 3.0}
    r = pred_resumed(_RESUME, ev)
    close(r["measured"]["time_to_resume_s"], 4.0)
    assert r["verdict"] == "pass"


# ── answer accuracy (SECONDARY) ───────────────────────────────────────

def test_extract_numbers_digits_and_words():
    assert extract_numbers("there are 4 carts") == [4.0]
    assert 3.0 in extract_numbers("three carts are parked")
    assert extract_numbers("") == []
    assert -0.5 in extract_numbers("I moved -0.5 m")


def test_check_number_in_text_matches_measured_truth():
    r = check_number_in_text("Four carts are parked in the row.", [4.0])
    assert r["match"] is True


def test_check_number_in_text_rejects_a_wrong_count():
    r = check_number_in_text("There are 7 carts parked.", [4.0])
    assert r["match"] is False


def test_check_number_in_text_accepts_either_snapshot():
    # The counter ticked between the ask and the reply; both are acceptable.
    r = check_number_in_text("I've shipped 5 boxes.", [4.0, 5.0])
    assert r["match"] is True


def test_check_number_in_text_handles_the_offline_fallback_reply():
    r = check_number_in_text(
        "I don't recognise that. Try: \"forward 1 m\", \"turn left 90 "
        "degrees\", \"stop\".", [4.0])
    # It DOES contain numbers (1, 90) -- none of which is the truth.
    assert r["match"] is False


def test_check_number_in_text_is_none_without_ground_truth():
    r = check_number_in_text("Four carts.", [None])
    assert r["match"] is None


def test_check_number_in_text_tolerance():
    assert check_number_in_text("x is 3.4", [3.5], tol=0.2)["match"] is True
    assert check_number_in_text("x is 3.4", [3.5], tol=0.01)["match"] is False


def test_resolve_truth_from_state():
    snaps = {"tug_a": {"idle_loop": {"park_row_count": 4}},
             "tug_b": {"idle_loop": {"park_row_count": 4}},
             "omniarm6": {"line": {"shipped_total": 9}}}
    v, note = resolve_truth("park_row_count", snaps)
    assert v == 4.0 and "tug_a" in note
    v, _ = resolve_truth("shipped_total", snaps)
    assert v == 9.0


def test_resolve_truth_refuses_when_the_tugs_disagree():
    snaps = {"tug_a": {"idle_loop": {"park_row_count": 4}},
             "tug_b": {"idle_loop": {"park_row_count": 2}}}
    v, note = resolve_truth("park_row_count", snaps)
    assert v is None
    assert "disagree" in note


def test_resolve_truth_unknown_source():
    v, note = resolve_truth("not_a_thing", {})
    assert v is None and "unknown" in note


# ── busy / reply parsing ──────────────────────────────────────────────

def test_is_busy_detects_a_409():
    assert is_busy(409, None) is True


def test_is_busy_detects_a_busy_action_inside_a_200_prompt_reply():
    # THE ONE THAT IS EASY TO MISS: /prompt answers 200 and buries the
    # refusal in the actions list. Scoring that as a model failure would be
    # measuring the harness's impatience.
    body = {"response": "I'm still finishing the last move.",
            "actions": [{"tool": "drive_forward", "result": "busy",
                         "summary": "busy"}]}
    assert is_busy(200, body) is True


def test_is_busy_false_for_a_normal_reply():
    body = {"response": "Driving forward 1 m.",
            "actions": [{"tool": "drive_forward", "result": "ok",
                         "summary": "1.00 m"}]}
    assert is_busy(200, body) is False


def test_reply_and_actions_survive_both_shapes():
    assert reply_of({"response": "hi"}) == "hi"
    assert reply_of({"agent": "hi"}) == "hi"
    assert reply_of(None) == ""
    assert actions_of({"actions": [{"tool": "t"}]}) == [{"tool": "t"}]
    assert actions_of({"actions": "nope"}) == []


# ── redaction ─────────────────────────────────────────────────────────

def test_redactor_masks_the_env_key_everywhere():
    os.environ["OMNI_KEY"] = "olink_TESTKEY_abcdef123456"
    try:
        r = Redactor()
        out = r({"reply": "using olink_TESTKEY_abcdef123456 now",
                 "list": ["olink_TESTKEY_abcdef123456"],
                 "n": 3})
        assert "olink_TESTKEY_abcdef123456" not in json.dumps(out)
        assert out["n"] == 3
    finally:
        os.environ.pop("OMNI_KEY", None)


def test_redactor_masks_key_shapes_it_was_never_told_about():
    r = Redactor()
    s = r("bearer sk-abcd1234efgh5678 and olink_zzzzzzzzzz")
    assert "sk-abcd1234efgh5678" not in s
    assert "olink_zzzzzzzzzz" not in s


def test_redactor_leaves_ordinary_text_alone():
    r = Redactor()
    assert r("Driving forward 1.00 m.") == "Driving forward 1.00 m."


# ── suite integrity ───────────────────────────────────────────────────

def test_every_probe_names_a_real_predicate():
    for s in SUITE:
        assert s["predicate"] in PREDICATES, s["key"]


def test_every_probe_targets_a_known_role():
    for s in SUITE:
        assert s["target"] in ROLES, s["key"]


def test_probe_keys_are_unique():
    keys = [s["key"] for s in SUITE]
    assert len(keys) == len(set(keys))


def test_resume_probes_require_a_paused_precondition_and_a_pause_setup():
    for s in SUITE:
        if s["predicate"] == "resumed":
            assert s.get("requires_paused") is True, s["key"]
            assert s.get("setup") == "pause", s["key"]


def test_non_resume_probes_reset_the_robot_first():
    for s in SUITE:
        if s["predicate"] != "resumed":
            assert s.get("setup") == "resume", s["key"]


def test_the_suite_covers_the_whole_difficulty_gradient():
    tiers = {s["tier"] for s in SUITE}
    for needed in ("1-literal", "2-parametric", "3-query", "4-compositional",
                   "5-world-state", "6-resume"):
        assert needed in tiers, needed


def test_verify_window_outlives_the_auto_resume_timer():
    # A resume probe watched for less than resume_s cannot tell the agent
    # from the clock, which would silently destroy the discriminator.
    for s in SUITE:
        v = resolve_verify_s(s, 60.0)
        if s["predicate"] == "resumed":
            assert v > 60.0, s["key"]
        else:
            assert v == float(s["verify_s"])


def test_suite_fingerprint_is_stable_and_sensitive():
    a = suite_fingerprint()
    assert a == suite_fingerprint()
    SUITE.append({"key": "tmp", "target": "tug", "text": "x",
                  "predicate": "at_rest", "params": {}, "verify_s": 1.0,
                  "setup": None})
    try:
        assert suite_fingerprint() != a
    finally:
        SUITE.pop()


def test_every_probe_carries_its_rationale_and_prediction():
    for s in SUITE:
        assert s.get("why"), s["key"]
        assert s.get("offline_expectation"), s["key"]


# ── scoring roll-up ───────────────────────────────────────────────────

def test_score_probes_excludes_inconclusive_from_the_rate():
    probes = [
        {"key": "a", "tier": "1-literal", "verdict": "pass", "latency_s": 1.0},
        {"key": "b", "tier": "2-parametric", "verdict": "fail", "latency_s": 2.0},
        {"key": "c", "tier": "6-resume", "verdict": "inconclusive"},
    ]
    sc = score_probes(probes)
    assert sc["state_score"]["scored"] == 2
    assert sc["state_score"]["passed"] == 1
    close(sc["state_score"]["rate"], 0.5)


def test_score_probes_keeps_answer_accuracy_separate():
    probes = [
        {"key": "a", "tier": "5-world-state", "verdict": "pass",
         "text_check": {"match": False}},
        {"key": "b", "tier": "5-world-state", "verdict": "pass",
         "text_check": {"match": True}},
        {"key": "c", "tier": "5-world-state", "verdict": "pass",
         "text_check": {"match": None}},
    ]
    sc = score_probes(probes)
    # A perfect state score alongside a 1/2 answer score: the two must not
    # be blended, or "did not park the robot" would launder a wrong answer.
    close(sc["state_score"]["rate"], 1.0)
    assert sc["answer_accuracy_SECONDARY"]["checked"] == 2
    assert sc["answer_accuracy_SECONDARY"]["correct"] == 1


def test_score_probes_handles_an_empty_run():
    sc = score_probes([])
    assert sc["probes_run"] == 0
    assert sc["state_score"]["rate"] is None


# ── recorder slicing (the throughput lane) ────────────────────────────

def test_slice_recorder_keeps_only_the_window():
    rec = ml.Recorder("omniarm6", "http://x")
    for i in range(10):
        rec.add(float(i), {"sim_time": float(i)}, 0.001)
    sub = slice_recorder(rec, 3.0, 6.0)
    assert sub.t == [3.0, 4.0, 5.0, 6.0]
    assert len(sub.states) == 4
    assert sub.name == "omniarm6"


def test_slice_recorder_empty_window():
    rec = ml.Recorder("omniarm6", "http://x")
    rec.add(0.0, {}, 0.001)
    assert slice_recorder(rec, 50.0, 60.0).t == []


# ══════════════════════════════════════════════════════════════════════
# INTEGRITY GATE 1 -- LIVENESS
#
# The measured failure: `scripts/dev/headless_runner.py` stops the engine with
# proc.terminate(), the bridge CONTROLLERS are never told to quit, their main
# threads block forever in robot.step() on a dead pipe, and their daemon HTTP
# threads keep answering 200. `ThreadingHTTPServer.allow_reuse_address` is 1,
# and a duplicate bind on one 127.0.0.1 port was reproduced on this box (two
# LISTENING pids; the EARLIER binder answered every request). These tests pin
# the only HTTP-visible tell: last_tick_at stops moving.
# ══════════════════════════════════════════════════════════════════════

def mk_live(tick, sim=None, **extra):
    st = {"id": "tug_b", "model": "MAV", "x": 0.0, "y": 0.0, "yaw": 0.0,
          "last_tick_at": tick}
    if sim is not None:
        st["sim_time"] = sim
    st.update(extra)
    return st


def test_liveness_passes_when_the_tick_loop_advances():
    r = classify_liveness("tug_b", "http://127.0.0.1:8767",
                          mk_live(1000.0, 120.0), mk_live(1001.2, 121.2), 1.2)
    assert r["verdict"] == "alive", r
    close(r["tick_advance_s"], 1.2, 1e-6)
    close(r["sim_advance_s"], 1.2, 1e-6)


def test_ADVERSARIAL_liveness_refuses_a_frozen_zombie_bridge():
    # THE HIJACK. Both reads are a clean 200 with a complete, plausible pose.
    # The ONLY difference from a live bridge is that last_tick_at did not
    # move -- because the controller's main thread is blocked forever in
    # robot.step() on a dead IPC pipe. Anything but a refusal here lets a
    # whole run be measured against a robot that does not exist.
    corpse = mk_live(1_700_000_000.5, 842.176)
    r = classify_liveness("tug_a", "http://127.0.0.1:8766", corpse,
                          dict(corpse), 1.2)
    assert r["verdict"] == "frozen", r
    assert r["tick_advance_s"] == 0.0
    # The refusal message must name the port and BOTH timestamps, or the
    # operator cannot tell this from a network blip.
    msg = liveness_remedy([r])
    assert "8766" in msg
    assert "1700000000.5" in msg
    assert "taskkill" in msg and "netstat" in msg
    assert "launch.bat" in msg          # points at the documented remedy


def test_liveness_refuses_a_tick_that_went_backwards():
    r = classify_liveness("tug_b", "u", mk_live(500.0), mk_live(499.0), 1.2)
    assert r["verdict"] == "frozen", r


def test_liveness_is_unverifiable_without_last_tick_at():
    # Absent field != frozen. A bridge that publishes no tick stamp cannot be
    # gated on one, and saying "frozen" there would be a fabrication.
    r = classify_liveness("x", "u", {"id": "x"}, {"id": "x"}, 1.2)
    assert r["verdict"] == "unverifiable", r


def test_liveness_reports_no_answer_when_a_read_failed():
    r = classify_liveness("x", "u", mk_live(1.0), None, 1.2)
    assert r["verdict"] == "no_answer", r


def test_liveness_ignores_sim_speed():
    # last_tick_at is WALL clock stamped per tick, so a world running at 0.05x
    # realtime is still demonstrably alive. Gating on sim_time instead would
    # fail every heavy world.
    r = classify_liveness("x", "u", mk_live(10.0, 100.0),
                          mk_live(11.2, 100.06), 1.2)
    assert r["verdict"] == "alive", r


# ══════════════════════════════════════════════════════════════════════
# INTEGRITY GATE 2 -- IDENTITY
# ══════════════════════════════════════════════════════════════════════

def mk_ident_state(sim, cycles, jobs, robot_id="tug_b", model="MAV"):
    return {"id": robot_id, "model": model, "sim_time": sim,
            "idle_loop": {"cycles": cycles, "jobs_total": jobs,
                          "paused": False}}


def test_identity_of_reads_the_available_fingerprint():
    f = identity_of(mk_ident_state(120.0, 3, 7))
    assert f["id"] == "tug_b" and f["model"] == "MAV"
    close(f["sim_time"], 120.0)
    assert f["counters"]["idle_loop.cycles"] == 3.0
    assert f["counters"]["idle_loop.jobs_total"] == 7.0


def test_identity_of_reads_the_arm_line_counter():
    f = identity_of({"id": "omniarm6", "line": {"shipped_total": 12}})
    assert f["counters"]["line.shipped_total"] == 12.0


def test_identity_stable_across_a_normal_run():
    a = identity_of(mk_ident_state(100.0, 3, 7))
    b = identity_of(mk_ident_state(1000.0, 9, 21))
    r = compare_identity("tug_b", a, b, 900.0, [4242], [4242])
    assert r["verdict"] == "stable", r
    assert r["findings"] == []
    assert "4242" in r["pid_note"]
    # ...and it must not oversell itself.
    assert "weaker than proof" in r["STRENGTH"]


def test_ADVERSARIAL_identity_flags_a_mid_run_process_swap():
    # A fresh bridge took the port over: its session counters restarted at 0
    # and its engine clock rewound. Both are impossible within one process.
    a = identity_of(mk_ident_state(400.0, 6, 14))
    b = identity_of(mk_ident_state(12.0, 0, 0))
    r = compare_identity("tug_b", a, b, 900.0, None, None)
    assert r["verdict"] == "swapped", r
    joined = " | ".join(r["findings"])
    assert "sim_time went BACKWARDS" in joined
    assert "idle_loop.cycles DECREASED" in joined


def test_identity_flags_a_changed_listener_pid():
    a = identity_of(mk_ident_state(100.0, 3, 7))
    b = identity_of(mk_ident_state(1000.0, 9, 21))
    r = compare_identity("tug_b", a, b, 900.0, [111], [222])
    assert r["verdict"] == "swapped", r
    assert any("pid changed" in f or "LISTENING pid changed" in f
               for f in r["findings"])


def test_ADVERSARIAL_identity_flags_a_duplicate_listener():
    # The exact hijack precondition: two processes bound to one loopback port
    # with SO_REUSEADDR. Which one answers is not ours to decide, so the run
    # cannot be trusted even if every other signal looks clean.
    a = identity_of(mk_ident_state(100.0, 3, 7))
    b = identity_of(mk_ident_state(1000.0, 9, 21))
    r = compare_identity("tug_a", a, b, 900.0, [111, 222], [111, 222])
    assert r["verdict"] == "swapped", r
    assert "MORE THAN ONE process is LISTENING" in " | ".join(r["findings"])


def test_identity_flags_a_different_robot_behind_the_port():
    a = identity_of(mk_ident_state(100.0, 3, 7, robot_id="tug_a"))
    b = identity_of(mk_ident_state(200.0, 4, 8, robot_id="tug_b"))
    r = compare_identity("tug_a", a, b, 100.0, None, None)
    assert r["verdict"] == "swapped", r
    assert "/state.id changed" in " | ".join(r["findings"])


def test_identity_fast_mode_clock_is_suspect_not_fatal():
    # sim_time far outrunning wall time is NORMAL under --mode=fast and is
    # also what a swap to an older process looks like. Flag, never fail.
    a = identity_of(mk_ident_state(0.0, 0, 0))
    b = identity_of(mk_ident_state(9000.0, 5, 5))
    r = compare_identity("tug_b", a, b, 100.0, None, None,
                         max_realtime_factor=20.0)
    assert r["verdict"] == "suspect", r
    close(r["implied_realtime_factor"], 90.0, 1e-6)


def test_identity_empty_pid_parse_is_inconclusive_not_zero_listeners():
    # A non-English netstat, or one we failed to parse, must NOT read as
    # "nothing is bound" -- that would turn a parse bug into a green light.
    a = identity_of(mk_ident_state(100.0, 3, 7))
    b = identity_of(mk_ident_state(200.0, 4, 8))
    r = compare_identity("tug_b", a, b, 100.0, [], [])
    assert r["verdict"] == "stable", r
    assert "inconclusive" in r["pid_note"]


def test_sim_scan_is_monotone_on_a_clean_run():
    ts = [i * 0.5 for i in range(40)]
    sims = [i * 0.5 for i in range(40)]
    r = scan_sim_monotonicity(ts, sims)
    assert r["monotone"] is True
    assert r["backward_jumps"] == 0
    assert r["worst"] is None
    assert r["samples"] == 40


def test_sim_scan_tolerates_gaps_in_the_series():
    # Failed polls record None. They must be skipped, not read as a drop.
    ts = [0.0, 0.5, 1.0, 1.5]
    sims = [10.0, None, 11.0, None]
    assert scan_sim_monotonicity(ts, sims)["monotone"] is True


def test_ADVERSARIAL_sim_scan_catches_a_MID_RUN_swap_the_endpoints_miss():
    # THE MEASURED HOLE. A process that takes the port over halfway through
    # has the rest of the run to climb its clock back above the preflight
    # reading, so an endpoint-only comparison scores the swap as `stable`
    # (reproduced end to end with --no-port-identity). The full-run series
    # still shows the discontinuity.
    ts = [i * 0.5 for i in range(40)]
    sims = [i * 0.5 for i in range(20)] + [0.2 + i * 0.5 for i in range(20)]
    r = scan_sim_monotonicity(ts, sims)
    assert r["monotone"] is False, r
    assert r["backward_jumps"] == 1
    close(r["worst"]["from_sim_s"], 9.5)
    close(r["worst"]["to_sim_s"], 0.2)

    # ...and the endpoint comparison alone genuinely does NOT see it, which is
    # why the scan is wired into the identity gate rather than trusted to the
    # start/end pair. This assertion is the record of that limitation.
    a = identity_of(mk_ident_state(0.0, 0, 0))
    b = identity_of(mk_ident_state(10.0, 1, 2))
    endpoints_only = compare_identity("tug_b", a, b, 20.0, None, None)
    assert endpoints_only["verdict"] == "stable", endpoints_only
    with_scan = compare_identity("tug_b", a, b, 20.0, None, None,
                                 sim_scan=r)
    assert with_scan["verdict"] == "swapped", with_scan
    assert "went BACKWARDS" in " | ".join(with_scan["findings"])


def test_parse_netstat_listeners_finds_both_pids_of_a_duplicate_bind():
    # Verbatim shape of `netstat -ano -p TCP` output, including the duplicate
    # bind reproduced on this machine.
    out = "\n".join([
        "Active Connections",
        "",
        "  Proto  Local Address          Foreign Address        State"
        "           PID",
        "  TCP    127.0.0.1:8766         0.0.0.0:0              LISTENING"
        "       33336",
        "  TCP    127.0.0.1:8766         0.0.0.0:0              LISTENING"
        "       31664",
        "  TCP    127.0.0.1:8766         127.0.0.1:55110        ESTABLISHED"
        "     31664",
        "  TCP    127.0.0.1:8767         0.0.0.0:0              LISTENING"
        "       31664",
        "  TCP    0.0.0.0:135            0.0.0.0:0              LISTENING"
        "       1200",
        "  UDP    127.0.0.1:8766         *:*                    4321",
    ])
    assert parse_netstat_listeners(out, 8766) == [31664, 33336]
    assert parse_netstat_listeners(out, 8767) == [31664]
    assert parse_netstat_listeners(out, 9999) == []
    assert parse_netstat_listeners("", 8766) == []


def test_parse_netstat_listeners_does_not_match_a_port_suffix():
    out = ("  TCP    127.0.0.1:18766        0.0.0.0:0              LISTENING"
           "       7\n")
    assert parse_netstat_listeners(out, 8766) == []


# ══════════════════════════════════════════════════════════════════════
# INTEGRITY GATE 3 -- ENGINE
#
# `enabled: true` says a relay exists. It does NOT say WHICH relay, and an
# OllamaRelay reports it identically. The discriminator is the shape each one
# writes into /usage.latest (relay.py:1981-1989 vs relay.py:1082-1085).
# ══════════════════════════════════════════════════════════════════════

_OLLAMA_USAGE = {"enabled": True,
                 "latest": {"engine": "ollama:qwen3:8b",
                            "input_tokens": 2100, "output_tokens": 130,
                            "text": "local qwen3:8b: 2100 in / 130 out tok, "
                                    "3.4s (38 tok/s)"}}
_CLOUD_USAGE = {"enabled": True,
                "latest": {"elapsed_s": 14.0, "input_units": 3100,
                           "output_units": 310, "credits": 0.12,
                           "text": "window=14s tokens=3,410 (in 3.1k + out "
                                   "0.3k) -> 877,000 tokens/hour, 0.03 "
                                   "credits/hour"}}
_FALLBACK_USAGE = {"enabled": True,
                   "latest": {"engine": "g1-engine",
                              "text": "cloud fallback via g1-engine"}}


def test_engine_offline_router_is_identified_from_enabled_false():
    r = classify_engine({"enabled": False})
    assert r["engine_class"] == ENGINE_NONE
    assert r["cloud"] is False


def test_ADVERSARIAL_enabled_true_with_an_ollama_latest_is_NOT_cloud():
    # THE ONE THIS GATE EXISTS FOR. `enabled: true` looks exactly like a
    # platform run, and the old check stopped there. A run labelled
    # `--mode omnilink` that was actually served by a local model must be
    # refused, not quietly written into the results file.
    r = classify_engine(_OLLAMA_USAGE)
    assert r["engine_class"] == ENGINE_LOCAL_OLLAMA, r
    assert r["cloud"] is False
    gate = engine_gate("omnilink", {"tug_b": r}, stage="after_first_prompt")
    assert gate["verdict"] == "contradicted", gate
    assert gate["fatal"], gate
    assert "LOCAL OLLAMA" in " | ".join(gate["fatal"])


def test_engine_cloud_rollup_is_identified_as_the_platform():
    r = classify_engine(_CLOUD_USAGE)
    assert r["engine_class"] == ENGINE_CLOUD, r
    assert r["cloud"] is True
    gate = engine_gate("omnilink", {"a": r, "b": r, "c": r},
                       stage="after_first_prompt")
    assert gate["verdict"] == "verified", gate
    assert gate["fatal"] == []


def test_engine_short_window_rollup_still_reads_as_cloud():
    # UsageDelta.report() degrades to "usage: window too short (0.4s)" on a
    # fast turn. That is still the platform meter and must not fall through
    # to `unverified`.
    r = classify_engine({"enabled": True,
                         "latest": {"elapsed_s": 0.4,
                                    "text": "usage: window too short (0.4s)"}})
    assert r["engine_class"] == ENGINE_CLOUD, r


def test_engine_ollama_cloud_fallback_is_flagged_not_claimed_clean():
    # The platform WAS reached -- but by a hybrid OllamaRelay, whose ordinary
    # turns are local. Neither a refusal nor a clean pass; a loud warning.
    r = classify_engine(_FALLBACK_USAGE)
    assert r["engine_class"] == ENGINE_CLOUD_FALLBACK, r
    assert r["cloud"] is True
    gate = engine_gate("omnilink", {"tug_b": r}, stage="after_first_prompt")
    assert gate["fatal"] == [], gate
    assert any("CLOUD FALLBACK" in w for w in gate["warnings"]), gate
    assert gate["verdict"] == ENGINE_UNVERIFIED


def test_engine_null_latest_is_unverified_not_a_verdict_either_way():
    # `latest` is null until a chat turn COMPLETES, and OMNILINK_USAGE=0
    # suppresses the cloud relay's meter forever. So null proves nothing --
    # not "no relay", and not "cloud".
    r = classify_engine({"enabled": True, "latest": None})
    assert r["engine_class"] == ENGINE_UNVERIFIED, r
    assert r["cloud"] is None
    assert "OMNILINK_USAGE=0" in r["evidence"]
    gate = engine_gate("omnilink", {"tug_b": r}, stage="after_first_prompt")
    assert gate["fatal"] == [], gate            # refuses to guess
    assert gate["verdict"] == ENGINE_UNVERIFIED


def test_engine_explicit_omnilink_identity_verifies_before_first_turn():
    r = classify_engine({
        "enabled": True,
        "relay": {"kind": "omnilink", "engine": "g1-engine"},
        "latest": None,
    })
    assert r["engine_class"] == ENGINE_CLOUD, r
    assert r["cloud"] is True
    assert r["engine"] == "g1-engine"


def test_engine_explicit_ollama_identity_verifies_before_first_turn():
    r = classify_engine({
        "enabled": True,
        "relay": {"kind": "ollama", "model": "qwen3:8b", "mode": "local"},
        "latest": None,
    })
    assert r["engine_class"] == ENGINE_LOCAL_OLLAMA, r
    assert r["cloud"] is False
    assert r["engine"] == "ollama:qwen3:8b"


def test_engine_explicit_identity_does_not_hide_cloud_fallback_turn():
    usage = dict(_FALLBACK_USAGE)
    usage["relay"] = {
        "kind": "ollama", "model": "qwen3:8b", "mode": "hybrid",
    }
    r = classify_engine(usage)
    assert r["engine_class"] == ENGINE_CLOUD_FALLBACK, r
    assert r["cloud"] is True


def test_engine_gate_refuses_omnilink_when_a_bridge_has_no_relay():
    # The live launch.bat trap: an interpreter without `omnilink` on PATH ->
    # relay setup fails -> the offline regex router answers, enabled=false.
    off = classify_engine({"enabled": False})
    cloud = classify_engine(_CLOUD_USAGE)
    gate = engine_gate("omnilink", {"omniarm6": off, "tug_a": cloud,
                                    "tug_b": cloud}, stage="preflight")
    assert gate["verdict"] == "contradicted", gate
    assert "omniarm6" in " | ".join(gate["fatal"])
    assert "launch.bat" in " | ".join(gate["fatal"])


def test_engine_gate_refuses_offline_when_a_relay_answered():
    gate = engine_gate("offline",
                       {"tug_b": classify_engine(_OLLAMA_USAGE)},
                       stage="preflight")
    assert gate["verdict"] == "contradicted", gate
    assert gate["fatal"]


def test_engine_gate_verifies_a_clean_local_ollama_run():
    local = classify_engine(_OLLAMA_USAGE)
    gate = engine_gate("local", {"a": local, "b": local, "c": local},
                       stage="after_first_prompt")
    assert gate["verdict"] == "verified", gate
    assert gate["fatal"] == []


def test_engine_gate_refuses_local_when_the_regex_router_answered():
    local = classify_engine(_OLLAMA_USAGE)
    off = classify_engine({"enabled": False})
    gate = engine_gate("local", {"omniarm6": local, "tug_a": off,
                                 "tug_b": local},
                       stage="after_first_prompt")
    assert gate["verdict"] == "contradicted", gate
    assert "tug_a" in " | ".join(gate["fatal"])
    assert "OFFLINE REGEX ROUTER" in " | ".join(gate["fatal"])


def test_engine_gate_refuses_local_when_the_cloud_answered():
    local = classify_engine(_OLLAMA_USAGE)
    cloud = classify_engine(_CLOUD_USAGE)
    gate = engine_gate("local", {"omniarm6": local, "tug_a": cloud,
                                 "tug_b": local},
                       stage="after_first_prompt")
    assert gate["verdict"] == "contradicted", gate
    assert "tug_a" in " | ".join(gate["fatal"])
    assert "CLOUD" in " | ".join(gate["fatal"])


def test_engine_gate_verifies_a_clean_offline_run():
    off = classify_engine({"enabled": False})
    gate = engine_gate("offline", {"a": off, "b": off, "c": off},
                       stage="preflight")
    assert gate["verdict"] == "verified", gate
    assert gate["fatal"] == []


def test_engine_gate_mode_none_never_refuses():
    # mode none issues NO prompts, so the chat layer is never exercised and a
    # live relay cannot corrupt the measurement. Note it, do not fail it.
    gate = engine_gate("none", {"tug_b": classify_engine(_CLOUD_USAGE)},
                       stage="preflight")
    assert gate["fatal"] == [], gate
    assert gate["verdict"] == "not_applicable"
    assert gate["warnings"]


def test_engine_gate_with_no_bridges_is_unverified():
    gate = engine_gate("omnilink", {}, stage="preflight")
    assert gate["verdict"] == ENGINE_UNVERIFIED
    assert gate["fatal"] == []


def test_mode_warnings_still_reports_both_halves():
    probed = {"tug_b": classify_engine(_OLLAMA_USAGE), "_all_relay": True}
    msgs = mode_warnings("omnilink", probed)
    assert any("LOCAL OLLAMA" in m for m in msgs), msgs


def test_the_new_exit_codes_are_distinct_and_documented():
    codes = {EXIT_LIVENESS: 7, EXIT_IDENTITY: 8, EXIT_ENGINE: 9}
    assert list(codes.values()) == [7, 8, 9]
    import bench_omnilink as _b
    doc = _b.__doc__ or ""
    for c in ("7", "8", "9"):
        assert f"    {c}  " in doc, f"exit code {c} is not in the module docstring"


# ── run_all ───────────────────────────────────────────────────────────

def run_all() -> int:
    """Run every test_* in this module. Returns a process exit code."""
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:                                # noqa: BLE001
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
