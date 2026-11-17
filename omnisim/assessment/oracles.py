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

"""Offline braking verdicts. Expected labels and injected delay do not grade motion."""
from __future__ import annotations

import math
from .schema import BrakingSpec


def grade(spec: BrakingSpec, rows: list[dict], completion: dict, physics: dict) -> dict:
    if spec.schema_version >= 2:
        return grade_mobility(spec, rows, completion, physics)
    result = {"status": "INCONCLUSIVE", "reasons": [], "metrics": {},
              "scope": "Husky model + deterministic assessment controller, simulation only"}

    def invalid(reason):
        result["reasons"].append(reason)
        return result

    if not physics.get("finalised") or physics.get("degraded"):
        return invalid("physics_not_proven")
    if physics.get("backend") != "newton" or physics.get("runtime", {}).get("device") != "cpu":
        return invalid("unqualified_physics_configuration")
    if not completion.get("complete"):
        return invalid("controller_episode_incomplete")
    if len(rows) < 3:
        return invalid("insufficient_samples")
    numeric = ("t_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s", "tilt_rad")
    try:
        for r in rows:
            if any(not math.isfinite(r[k]) for k in numeric):
                return invalid("nonfinite_trace")
            if len(r["wheel_position_rad"]) != 4 or any(
                    not math.isfinite(v) for v in r["wheel_position_rad"]):
                return invalid("wheel_measurements_missing")
            if not isinstance(r["contacts"], int) or r["contacts"] < 0:
                return invalid("contact_measurements_missing")
        dt = spec.timestep_ms / 1000
        gaps = [b["t_s"] - a["t_s"] for a, b in zip(rows, rows[1:])]
        if any(abs(g - dt) > dt * 0.05 for g in gaps):
            return invalid("trace_gap_or_clock_discontinuity")
        if any(r.get("seq") != i for i, r in enumerate(rows)):
            return invalid("trace_sequence_incomplete")
        if completion.get("samples") != len(rows):
            return invalid("trace_count_mismatch")
        if abs(completion["last_t_s"] - rows[-1]["t_s"]) > dt * 0.05:
            return invalid("trace_tail_mismatch")
        requests = [i for i, r in enumerate(rows) if r["stop_requested"]]
        if not requests:
            return invalid("stop_request_not_reached")
        k = requests[0]
        if any(not r["stop_requested"] for r in rows[k:]):
            return invalid("stop_request_discontinuity")
        start, request = rows[0], rows[k]
        post = rows[k:]
        if post[-1]["t_s"] - request["t_s"] < spec.observe_after_request_s - dt * 1.1:
            return invalid("post_request_observation_too_short")
        if abs(request["y_m"] - start["y_m"]) > 0.1 or max(abs(r["y_m"] - start["y_m"]) for r in rows) > 0.15:
            return invalid("outside_straight_line_fixture")
        if max(r["tilt_rad"] for r in rows) > 0.25 or min(r["z_m"] for r in rows) < 0.04:
            return invalid("invalid_ground_support_or_tilt")
        drive = [r for r in rows[:k + 1] if r["t_s"] >= request["t_s"] - spec.speed_hold_s]
        if len(drive) < 3 or drive[-1]["t_s"] - drive[0]["t_s"] < spec.speed_hold_s - dt * 1.1:
            return invalid("pre_request_observation_too_short")
        if any(not 0.9 * spec.target_speed_m_s <= r["vx_m_s"] <= 1.15 * spec.target_speed_m_s for r in drive):
            return invalid("initial_speed_not_qualified")
        if request["x_m"] - start["x_m"] < spec.min_drive_distance_m:
            return invalid("insufficient_initial_motion")
        if not any(r["contacts"] > 0 for r in drive):
            return invalid("no_measured_ground_contacts")
        if min(abs(b - a) for a, b in zip(start["wheel_position_rad"], request["wheel_position_rad"])) < 0.5:
            return invalid("wheel_rotation_not_proven")
        # The oracle finds settling independently, using measured horizontal speed.
        stop = None
        for i, r in enumerate(post):
            tail = [s for s in post[i:] if s["t_s"] <= r["t_s"] + spec.settled_hold_s + dt * .1]
            if tail[-1]["t_s"] - r["t_s"] < spec.settled_hold_s - dt * 1.1:
                continue
            if all(math.hypot(s["vx_m_s"], s["vy_m_s"]) <= spec.settled_speed_m_s for s in post[i:]):
                stop = r
                break
        displacement = [r["x_m"] - request["x_m"] for r in post]
        path = sum(math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]) for a, b in zip(post, post[1:]))
        applied = next((r for r in post if r["brake_applied"]), None)
        metrics = {
            "request_time_s": request["t_s"], "initial_speed_m_s": request["vx_m_s"],
            "pre_request_distance_m": request["x_m"] - start["x_m"],
            "max_forward_excursion_m": max(displacement), "rollback_m": max(0.0, -min(displacement)),
            "post_request_path_m": path,
            "stop_time_s": None if stop is None else stop["t_s"] - request["t_s"],
            "observed_command_delay_s": None if applied is None else applied["t_s"] - request["t_s"],
            "final_speed_m_s": math.hypot(post[-1]["vx_m_s"], post[-1]["vy_m_s"]),
            "sample_count": len(rows), "sample_period_s": dt,
            "observation_after_request_s": post[-1]["t_s"] - request["t_s"],
        }
    except (KeyError, TypeError, ValueError, IndexError):
        return invalid("malformed_trace_or_completion")
    result["metrics"] = metrics
    if max(displacement) > spec.max_stop_distance_m:
        result["reasons"].append("stopping_distance_exceeded")
    if stop is None or metrics["stop_time_s"] > spec.max_stop_time_s:
        result["reasons"].append("stopping_deadline_exceeded")
    if metrics["rollback_m"] > 0.05:
        result["reasons"].append("rollback_exceeded")
    result["status"] = "FAIL" if result["reasons"] else "PASS"
    return result


def grade_mobility(spec, rows, completion, physics):
    """V2 distinguishes valid approach/tip failures from qualified braking cases."""
    result = {"status": "INCONCLUSIVE", "reasons": [], "metrics": {},
              "scope": f"{spec.robot} physical-wheel model/controller; simulation only"}
    def invalid(reason):
        result["reasons"] = [reason]
        return result
    if not physics.get("finalised") or physics.get("degraded") or physics.get("backend") != "newton":
        return invalid("physics_not_proven")
    if physics.get("runtime", {}).get("device") != "cpu":
        return invalid("unqualified_physics_configuration")
    if not completion.get("complete") or len(rows) < 3:
        return invalid("controller_episode_incomplete")
    try:
        dt = spec.timestep_ms / 1000
        n = completion["wheel_count"]
        for i, r in enumerate(rows):
            if r["seq"] != i or not all(math.isfinite(r[k]) for k in (
                    "t_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s",
                    "along_m", "forward_m_s", "normal_height_m", "ground_tilt_rad")):
                return invalid("malformed_or_nonfinite_trace")
            if n not in (2, 4) or len(r["wheel_position_rad"]) != n or not all(math.isfinite(x) for x in r["wheel_position_rad"]):
                return invalid("wheel_measurements_missing")
            if i and abs(r["t_s"] - rows[i-1]["t_s"] - dt) > dt*.05:
                return invalid("trace_gap_or_clock_discontinuity")
        if completion["samples"] != len(rows) or abs(completion["last_t_s"]-rows[-1]["t_s"]) > dt*.05:
            return invalid("trace_tail_mismatch")
        if not any(r["contacts"] > 0 for r in rows):
            return invalid("no_measured_ground_contacts")
        if min(r["normal_height_m"] for r in rows) < -.08:
            return invalid("body_below_ground_invalid")
        req_index = next((i for i,r in enumerate(rows) if r["stop_requested"]), None)
        max_tilt = max(r["ground_tilt_rad"] for r in rows)
        result["metrics"] = {"braking_qualified": req_index is not None,
            "max_achieved_speed_m_s": max(r["forward_m_s"] for r in rows),
            "max_ground_tilt_rad": max_tilt, "sample_count": len(rows),
            "sample_period_s": dt, "max_forward_excursion_m": None,
            "initial_speed_m_s": None, "stop_time_s": None, "observed_command_delay_s": None}
        reasons = []
        if max_tilt > .7:
            reasons.append("tipped_relative_to_ground")
        if max(abs(r["y_m"]-rows[0]["y_m"]) for r in rows) > .3:
            reasons.append("lateral_departure")
        idle = [r for r in rows if spec.settle_s-.25 <= r["t_s"] < spec.settle_s]
        if idle and all(abs(r["forward_m_s"]) > .05 for r in idle):
            reasons.append("initial_hold_failed")
        if req_index is None:
            if completion.get("terminal") not in ("approach_timeout", "fixture_departure_or_tip"):
                return invalid("stop_request_not_reached")
            reasons.append("approach_speed_not_reached")
            result.update(status="FAIL", reasons=reasons)
            return result
        req = rows[req_index]
        pre = [r for r in rows[:req_index+1] if r["t_s"] >= req["t_s"]-spec.speed_hold_s]
        if len(pre) < 3 or pre[-1]["t_s"]-pre[0]["t_s"] < spec.speed_hold_s-dt*1.1:
            return invalid("pre_request_observation_too_short")
        if not all(.9*spec.target_speed_m_s <= r["forward_m_s"] <= 1.15*spec.target_speed_m_s for r in pre):
            return invalid("initial_speed_not_qualified")
        if req["along_m"]-rows[0]["along_m"] < spec.min_drive_distance_m:
            return invalid("insufficient_initial_motion")
        if min(abs(b-a) for a,b in zip(rows[0]["wheel_position_rad"],req["wheel_position_rad"])) < .5:
            return invalid("wheel_rotation_not_proven")
        post = rows[req_index:]
        # A physically observed tip/departure is a failure even if observation
        # terminated early. A truncated ordinary run remains inconclusive.
        if post[-1]["t_s"]-req["t_s"] < spec.observe_after_request_s-dt*1.1 and not reasons:
            return invalid("post_request_observation_too_short")
        displacement = [r["along_m"]-req["along_m"] for r in post]
        stop = next((r for i,r in enumerate(post) if post[-1]["t_s"]-r["t_s"] >= spec.settled_hold_s-dt*.1
                     and all(math.sqrt(s["vx_m_s"]**2+s["vy_m_s"]**2+s["vz_m_s"]**2) <= spec.settled_speed_m_s for s in post[i:])),None)
        applied = next((r for r in post if r["brake_applied"]),None)
        result["metrics"].update(request_time_s=req["t_s"], initial_speed_m_s=req["forward_m_s"],
            max_forward_excursion_m=max(displacement), rollback_m=max(0.,-min(displacement)),
            stop_time_s=None if stop is None else stop["t_s"]-req["t_s"],
            observed_command_delay_s=None if applied is None else applied["t_s"]-req["t_s"],
            final_speed_m_s=math.sqrt(sum(post[-1][k]**2 for k in ("vx_m_s","vy_m_s","vz_m_s"))))
        if max(displacement) > spec.max_stop_distance_m:
            reasons.append("stopping_distance_exceeded")
        if stop is None or stop["t_s"]-req["t_s"] > spec.max_stop_time_s:
            reasons.append("stopping_deadline_exceeded")
        if min(displacement) < -.05:
            reasons.append("rollback_exceeded")
        result.update(status="FAIL" if reasons else "PASS", reasons=reasons)
        return result
    except (KeyError,ValueError,TypeError,IndexError):
        return invalid("malformed_trace_or_completion")
