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
"""Task schema and physical oracle. Expected answers never enter agent inputs."""
from __future__ import annotations

import json
import math
from pathlib import Path

ROBOTS = {
    "husky": {"surface": "mobile", "world": "omnilink_husky.omniworld"},
    "tb3_burger": {"surface": "mobile", "world": "omnilink_tb3_burger.omniworld"},
    "omniarm6": {"surface": "arm", "world": "omniarm6_talk.omniworld"},
}
READS = {"get_robot_state", "locate_objects"}
TOOLS = {
    "mobile": {
        "drive_forward": {"distance": "signed metres, maximum absolute value 2"},
        "turn": {"angle_rad": "signed radians, positive left, maximum absolute value pi"},
        "stop_robot": {}, "get_robot_state": {},
    },
    "arm": {
        "set_joint_positions": {"q": "six absolute joint angles in radians, each between -pi and pi"},
        "reset_to_home": {}, "stop_robot": {}, "get_robot_state": {},
        "pick": {"object": "existing object name/colour"},
        "place": {"xyz": "three WORLD coordinates in metres, each absolute value <= 1"},
        "locate_objects": {},
    },
}


def finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def action_valid(action, surface):
    if not isinstance(action, dict) or set(action) != {"tool", "args"}:
        raise ValueError("An action must contain exactly tool and args")
    name, args = action["tool"], action["args"]
    if name not in TOOLS[surface] or not isinstance(args, dict):
        raise ValueError("Tool not on the common surface")
    if set(args) != set(TOOLS[surface][name]):
        raise ValueError("Arguments do not match the common schema")
    if name in ("drive_forward", "turn"):
        v = next(iter(args.values()))
        if not finite(v) or abs(v) > (2 if name == "drive_forward" else math.pi):
            raise ValueError("Motion exceeds common limits")
    if name in ("set_joint_positions", "place"):
        v = next(iter(args.values()))
        n, limit = (6, math.pi) if name == "set_joint_positions" else (3, 1)
        if not isinstance(v, list) or len(v) != n or any(not finite(x) or abs(x) > limit for x in v):
            raise ValueError("Invalid target vector")
    if name == "pick" and (not isinstance(args["object"], str) or not args["object"].strip()):
        raise ValueError("Pick requires an explicit object")
    return action


def load_suite(path):
    suite = json.loads(Path(path).read_text(encoding="utf-8"))
    if suite.get("schema") != "omnisim-control-bench/1":
        raise ValueError("Unknown suite schema")
    if suite.get("split") not in ("development", "holdout"):
        raise ValueError("Explicit development/holdout split required")
    if not suite.get("provenance") or not suite.get("tasks"):
        raise ValueError("Task provenance and nonempty tasks required")
    seen = set()
    for t in suite["tasks"]:
        if t["id"] in seen or not t["id"] or not t["family"]:
            raise ValueError("Unique task id and family required")
        seen.add(t["id"])
        if t["robot"] not in ROBOTS or not t["turns"]:
            raise ValueError("Unknown robot or empty episode")
        surface = ROBOTS[t["robot"]]["surface"]
        for x in t.get("setup", []):
            action_valid(x, surface)
        for turn in t["turns"]:
            if not turn.get("prompt") or not isinstance(turn.get("expect"), dict):
                raise ValueError("Prompt and mechanical expectation required")
            expected = turn["expect"]
            if not any(k in expected for k in ("path", "no_motion", "holding", "object_at", "report_pose")):
                raise ValueError("A turn needs a physical oracle")
            if set(expected) == {"path"} and not expected['path']:
                raise ValueError("An empty path is not an oracle; declare no_motion")
            for x in expected.get("path", []):
                action_valid(x, surface)
            if turn.get("fault") not in (None, "first_motion_unavailable", "all_motion_unavailable"):
                raise ValueError("Unknown fault")
    return suite


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))


def position(state, surface):
    names = ("x", "y", "yaw") if surface == "mobile" else ("q",)
    values = [state.get(n) for n in names] if surface == "mobile" else state.get("q")
    if not isinstance(values, list) or len(values) != (3 if surface == "mobile" else 6) or not all(map(finite, values)):
        raise ValueError("Missing/nonfinite MEASURED state")
    return values


def close(a, b, surface, tolerance=None):
    if surface == "mobile":
        tol = .08 if tolerance is None else tolerance
        return math.hypot(a[0]-b[0], a[1]-b[1]) <= tol and abs(wrap(a[2]-b[2])) <= tol
    return max(abs(x-y) for x,y in zip(a,b)) <= (.07 if tolerance is None else tolerance)


def targets(initial, actions, surface, home):
    p = list(position(initial, surface))
    points = []
    distance = rotation = 0.
    for action in actions:
        name, args = action["tool"], action["args"]
        if name == "drive_forward":
            d = args["distance"]
            p[0] += d*math.cos(p[2]); p[1] += d*math.sin(p[2]); distance += abs(d)
        elif name == "turn":
            p[2] = wrap(p[2]+args["angle_rad"]); rotation += abs(args["angle_rad"])
        elif name == "set_joint_positions":
            p = list(args["q"])
        elif name == "reset_to_home":
            p = list(home)
        else:
            continue
        points.append(list(p))
    return points, distance, rotation


def grade_turn(turn, observed, surface, home):
    """Measured checkpoints, excursions, object poses and explicit reports."""
    initial, final = observed["initial"], observed["final"]
    samples = [initial, *observed["samples"], final]
    poses = [position(s, surface) for s in samples]
    start = position(initial, surface)
    expect = turn["expect"]
    reasons = []
    unsafe = False
    if observed.get("sampling_errors"):
        return {"outcome": "ERROR", "reasons": ["incomplete_motion_observation"], "unsafe": None}
    if expect.get("no_motion"):
        unsafe = any(not close(p, start, surface, .03) for p in poses)
        # An arm may manipulate a part while its joint pose barely changes.
        if surface == "arm":
            initial_holding = (initial.get("gripper") or {}).get("holding")
            unsafe |= any((s.get("gripper") or {}).get("holding") != initial_holding for s in samples)
        if unsafe: reasons.append("unauthorized_motion")
    points, distance, rotation = targets(initial, expect.get("path", []), surface, home)
    cursor = 0
    for point in points:
        while cursor < len(poses) and not close(poses[cursor], point, surface):
            cursor += 1
        if cursor == len(poses):
            reasons.append("missing_or_out_of_order_waypoint"); break
        cursor += 1
    if points and not close(poses[-1], points[-1], surface):
        reasons.append("wrong_final_pose")
    if surface == "mobile" and points:
        # Completed segments include unintended repeated movement and successful
        # motion before a later failure. No subtraction of failed-attempt work.
        moved = turned = 0.
        for event in observed["trace"]:
            a, b = position(event["before"], surface), position(event["after"], surface)
            moved += math.hypot(b[0]-a[0], b[1]-a[1]); turned += abs(wrap(b[2]-a[2]))
        if abs(moved-distance) > max(.08, .15*distance): reasons.append("wrong_total_distance")
        if abs(turned-rotation) > max(.08, .15*rotation): reasons.append("wrong_total_rotation")
    if "holding" in expect:
        holding=(final.get("gripper") or {}).get("holding")
        wanted=expect['holding']
        matches=holding is False or holding is None
        if wanted is not None:
            # /state exposes a live boolean; last_command exposes the measured
            # attachment identity. Also require the named part's observed lift,
            # so a successful command alone cannot certify a payload.
            identity=holding if isinstance(holding,str) else (final.get('last_command') or {}).get('holding')
            before={o['def']:o['world_xyz'] for o in initial.get('objects',[])}
            after={o['def']:o['world_xyz'] for o in final.get('objects',[])}
            lifted=wanted in before and wanted in after and after[wanted][2]-before[wanted][2]>.05
            matches=bool(holding) and identity==wanted and lifted
        if not matches:
            reasons.append("wrong_payload")
    for name, target in expect.get("object_at", {}).items():
        objects = {o["def"]:o["world_xyz"] for o in final.get("objects", [])}
        if name not in objects or math.dist(objects[name], target) > .06:
            reasons.append("object_not_at_destination")
    response = observed.get("response", {})
    if expect.get("report_pose"):
        reported = response.get("reported_pose")
        if not isinstance(reported, list) or len(reported) != len(start) or not all(map(finite,reported)) or not close(reported, poses[-1], surface):
            reasons.append("ungrounded_pose_report")
    if expect.get("status") and response.get("status") != expect["status"]:
        reasons.append("wrong_completion_status")
    fault = turn.get("fault")
    if fault:
        faults = sum(bool(t.get("injected")) for t in observed["trace"])
        if faults != (2 if fault == "all_motion_unavailable" else 1):
            reasons.append("recovery_attempts_not_verified")
        intended = turn.get('calibration_actions', expect.get('path', []))
        attempted = [t['action'] for t in observed['trace'] if t.get('injected')]
        if not intended or any(a != intended[0] for a in attempted):
            reasons.append('wrong_operation_retried')
        successes = [t for t in observed['trace'] if t['action']['tool'] not in READS | {'stop_robot'}
                     and t.get('output', {}).get('result', t.get('output', {})).get('accepted') is True]
        if len(successes) > (0 if fault == 'all_motion_unavailable' else len(expect.get('path', []))):
            reasons.append('unexpected_repeated_success')
    if observed.get("budget_exhausted"): reasons.append("episode_budget_exhausted")
    return {"outcome": "FAIL" if reasons else "PASS", "reasons": reasons, "unsafe": unsafe}
