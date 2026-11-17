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

"""Manual Husky AMCL stationary-endpoint trial; no navigation or SLAM."""

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
import time
import traceback
import urllib.request
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
WORLD = ROOT / "projects/samples/demos/worlds/showcase/husky_lidar_walls.omniworld"
MAP = HERE / "maps/husky_lidar_walls.yaml"
PARAMS = ROOT / "packages/omnisim-ros2/src/omnisim_ros2_nav2/params/omnisim_nav2_params.yaml"
PORTS = (6789, 6790, 8783)
# Conservative functional regression gates; not statistical accuracy bounds.
DEFAULT_POSITION_LIMIT_M = 0.10
DEFAULT_YAW_LIMIT_RAD = 0.05
OBSERVATION_INTERVAL_S = 0.4
OBSERVATION_MAX_GAP_S = 1.0
OBSERVATION_MIN_SAMPLES = 6
OBSERVATION_MIN_DURATION_S = 2.0
DEFAULT_CONVERGENCE_HOLD_S = OBSERVATION_MIN_DURATION_S
DEFAULT_CONVERGENCE_TIMEOUT_S = 15.0
DEFAULT_POSITION_JITTER_M = 0.03
DEFAULT_YAW_JITTER_RAD = 0.02
# Integrity bounds, not localization acceptance thresholds. Endpoints span at
# least one second, with a scan update strictly inside that stationary interval.
INTEGRITY = dict(position_span_m=0.002, yaw_span_rad=0.002,
                 linear_speed_m_s=0.005, angular_speed_rad_s=0.005,
                 message_age_wall_s=1.0, time_domain_difference_s=0.25)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def finite(values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def world_to_map(node):
    """The fixture's URDF root is base_link; map/world XY are identical."""
    xyz, rotation = node.get("position", []), node.get("orientation", [])
    if node.get("def") != "HUSKY" or len(xyz) != 3 or len(rotation) != 9:
        raise ValueError("missing HUSKY world pose")
    if not finite([*xyz, *rotation]):
        raise ValueError("nonfinite world pose")
    # Reject a tipped chassis: the planar scanner/map model would be invalid.
    c, s = rotation[0], rotation[3]
    expected = [c, -s, 0, s, c, 0, 0, 0, 1]
    if abs(math.hypot(c, s) - 1) > 0.001 or any(abs(a-b) > 0.001 for a, b in zip(rotation, expected)):
        raise ValueError("world orientation is not a level planar rotation")
    return dict(frame="map", x=xyz[0], y=xyz[1], yaw=math.atan2(rotation[3], rotation[0]))


def pose_errors(estimate, reference):
    if estimate.get("frame") != "map" or reference.get("frame") != "map":
        raise ValueError("comparison requires map-frame poses")
    if not finite([p.get(k) for p in (estimate, reference) for k in ("x", "y", "yaw")]):
        raise ValueError("missing/nonfinite planar pose")
    return dict(position_error_m=math.hypot(estimate["x"] - reference["x"],
                                            estimate["y"] - reference["y"]),
                yaw_error_rad=wrap(estimate["yaw"] - reference["yaw"]))


def endpoint_reasons(endpoint, minimum_samples=3, minimum_duration=1.0):
    """Validate actual acquisition evidence before allowing endpoint scoring."""
    try:
        estimate, samples = endpoint["amcl"], endpoint["ground_truth"]
        if len(samples) < minimum_samples:
            return ["insufficient stationary ground-truth acquisitions"]
        references = [world_to_map(s["node"]) for s in samples]
        pose_errors(estimate, references[-1])
        start, end = samples[0]["clock_before"], samples[-1]["clock_after"]
        if not finite([start, end, estimate["stamp"], endpoint["refresh_request_clock"]]):
            return ["missing/nonfinite simulator timestamps"]
        reasons = []
        if end - start < minimum_duration:
            reasons.append("stationary acquisition interval is too short")
        if not start <= endpoint["refresh_request_clock"] <= estimate["stamp"] <= end:
            reasons.append("AMCL estimate is stale or outside the stationary interval")
        if (not finite([endpoint["matching_scan_stamp"]]) or abs(endpoint["matching_scan_stamp"]-estimate["stamp"]) > 1e-6
                or set(endpoint["tf_at_scan"]) != {"map->odom", "odom->base_link", "base_link->base_laser"}):
            reasons.append("missing matching scan or scan-time TF evidence")
        for transform in endpoint["tf_at_scan"].values():
            translation = transform["translation"]
            if (len(translation) != 3 or not finite([*translation, transform["yaw"], transform["stamp"]])
                    or abs(transform["stamp"]-estimate["stamp"]) > 1e-6):
                reasons.append("invalid or mistimed scan-time TF evidence")
        cov = estimate["covariance_diagonal"]
        if len(cov) != 3 or not finite(cov) or min(cov) < -1e-9:
            reasons.append("invalid localization covariance")
        for index, sample in enumerate(samples):
            if not finite([sample["requested_monotonic_ns"], sample["received_monotonic_ns"]]):
                reasons.append("nonfinite ground-truth request/response timing")
            elif sample["received_monotonic_ns"] < sample["requested_monotonic_ns"]:
                reasons.append("reversed ground-truth request/response timing")
            if index and (sample["requested_monotonic_ns"] < samples[index-1]["received_monotonic_ns"]
                          or sample["clock_before"] < samples[index-1]["clock_after"]):
                reasons.append("ground-truth acquisitions overlap or clock rewound")
            if not finite([sample["clock_before"], sample["clock_after"],
                           sample["v_linear"], sample["v_angular"]]):
                reasons.append("nonfinite stationary-state evidence")
            elif sample["clock_before"] > sample["clock_after"]:
                reasons.append("simulator clock rewound during acquisition")
            if (abs(sample["v_linear"]) > INTEGRITY["linear_speed_m_s"]
                    or abs(sample["v_angular"]) > INTEGRITY["angular_speed_rad_s"]):
                reasons.append("robot velocity is not stationary")
        if any(math.hypot(a["x"]-b["x"], a["y"]-b["y"]) > INTEGRITY["position_span_m"]
               or abs(wrap(a["yaw"]-b["yaw"])) > INTEGRITY["yaw_span_rad"] for a in references for b in references):
            reasons.append("independent ground truth moved during the stationary interval")
        return sorted(set(reasons))
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        return [f"missing/invalid endpoint evidence: {exc}"]


def score(endpoints, invalid_reasons, position_limit=DEFAULT_POSITION_LIMIT_M, yaw_limit=DEFAULT_YAW_LIMIT_RAD):
    reasons = list(invalid_reasons)
    for label in ("initial", "post_motion"):
        reasons.extend(f"{label}: {r}" for r in endpoint_reasons(endpoints.get(label, {})))
    if reasons:
        return dict(measurement="INVALID", localization="NOT_SCORED", overall="INVALID", reasons=reasons)
    if position_limit is None or yaw_limit is None:
        return dict(measurement="PASS", localization="NOT_SCORED", overall="NOT_SCORED", reasons=[])
    errors = [pose_errors(endpoints[label]["amcl"], world_to_map(endpoints[label]["ground_truth"][-1]["node"]))
              for label in ("initial", "post_motion")]
    passed = all(e["position_error_m"] <= position_limit and abs(e["yaw_error_rad"]) <= yaw_limit for e in errors)
    return dict(measurement="PASS", localization="PASS" if passed else "FAIL",
                overall="PASS" if passed else "FAIL", reasons=[])


def series_sample_reasons(sample, previous=None, anchor=None):
    reasons = endpoint_reasons(sample, minimum_samples=2, minimum_duration=0)
    if sample.get("valid") is False:
        reasons.extend(sample.get("invalid_reasons") or ["collector marked sample invalid"])
    try:
        t = sample["monotonic_s"]
        before, after = sample["ground_truth"][0], sample["ground_truth"][-1]
        if (not finite([t]) or not before["requested_monotonic_ns"] / 1e9 <= t <= after["received_monotonic_ns"] / 1e9
                or (after["received_monotonic_ns"]-before["requested_monotonic_ns"]) / 1e9 > OBSERVATION_MAX_GAP_S):
            reasons.append("ground truth does not closely bracket AMCL reception")
        timing, ages = sample["timing"], sample["receipt_ages_wall_s"]
        if (set(timing) != {"clock", "odom", "scan"} or not finite(list(timing.values()))
                or max(timing.values())-min(timing.values()) > INTEGRITY["time_domain_difference_s"]):
            reasons.append("inconsistent simulator-time domains")
        if (set(ages) != {"clock", "odom", "scan"} or not finite(list(ages.values()))
                or min(ages.values()) < 0 or max(ages.values()) > INTEGRITY["message_age_wall_s"]):
            reasons.append("stale clock/odom/scan acquisition")
        if previous is not None:
            clocks = [t, previous["monotonic_s"], sample["amcl"]["stamp"], previous["amcl"]["stamp"]]
            if not finite(clocks) or clocks[0] <= clocks[1] or clocks[2] <= clocks[3]:
                reasons.append("unordered/repeated AMCL or monotonic timestamp")
            elif clocks[0]-clocks[1] > OBSERVATION_MAX_GAP_S:
                reasons.append("excessive sampling gap")
        if anchor is not None:
            for gt in sample["ground_truth"]:
                drift = pose_errors(world_to_map(gt["node"]), anchor)
                if (drift["position_error_m"] > INTEGRITY["position_span_m"]
                        or abs(drift["yaw_error_rad"]) > INTEGRITY["yaw_span_rad"]):
                    reasons.append("robot moved within stationary observation")
    except (KeyError, TypeError, ValueError, IndexError, AttributeError) as exc:
        reasons.append(f"missing/invalid stationary sample evidence: {exc}")
    return sorted(set(reasons))


def analyze_observation(samples, start, end, hold=DEFAULT_CONVERGENCE_HOLD_S,
                        timeout=DEFAULT_CONVERGENCE_TIMEOUT_S,
                        position_limit=DEFAULT_POSITION_LIMIT_M, yaw_limit=DEFAULT_YAW_LIMIT_RAD,
                        position_jitter_limit=DEFAULT_POSITION_JITTER_M,
                        yaw_jitter_limit=DEFAULT_YAW_JITTER_RAD, scoring=True):
    """Host-monotonic convergence and population jitter of fresh stationary pairs."""
    reasons, valid, gaps = [], [], []
    confirmation = entry = streak_start = anchor = previous = None
    streak_count = 0
    interval_valid = finite([start, end]) and end >= start
    if not interval_valid:
        reasons.append("invalid observation clock interval")
    for index, sample in enumerate(samples):
        bad = series_sample_reasons(sample, previous, anchor)
        if previous is not None and finite([sample.get("monotonic_s"), previous.get("monotonic_s")]):
            gaps.append(sample["monotonic_s"]-previous["monotonic_s"])
        previous = sample
        if bad:
            reasons.extend(f"sample {index}: {r}" for r in bad)
            streak_start, streak_count = None, 0
            continue
        if anchor is None:
            anchor = world_to_map(sample["ground_truth"][0]["node"])
        valid.append(sample)
        error = pose_errors(sample["amcl"], world_to_map(sample["ground_truth"][-1]["node"]))
        t = sample["monotonic_s"]
        if error["position_error_m"] <= position_limit and abs(error["yaw_error_rad"]) <= yaw_limit:
            if streak_start is None:
                streak_start = t
            streak_count += 1
            if (interval_valid and confirmation is None and streak_count >= OBSERVATION_MIN_SAMPLES
                    and t-streak_start >= hold and t-start <= timeout):
                confirmation, entry = t-start, streak_start-start
        else:
            streak_start, streak_count = None, 0
    duration = valid[-1]["monotonic_s"]-valid[0]["monotonic_s"] if valid else 0
    if len(valid) < OBSERVATION_MIN_SAMPLES or duration < OBSERVATION_MIN_DURATION_S:
        reasons.append("insufficient stationary observation coverage")
    if valid and interval_valid:
        if (valid[0]["monotonic_s"] < start or valid[-1]["monotonic_s"] > end
                or valid[0]["monotonic_s"]-start > OBSERVATION_MAX_GAP_S
                or end-valid[-1]["monotonic_s"] > OBSERVATION_MAX_GAP_S):
            reasons.append("uncovered observation boundary")
        if confirmation is None and end-start < timeout:
            reasons.append("observation ended before convergence or timeout")
    jitter = dict(valid_samples=len(valid), duration_s=duration)
    if valid:
        xs, ys, angles = ([s["amcl"][key] for s in valid] for key in ("x", "y", "yaw"))
        mx, my = statistics.mean(xs), statistics.mean(ys)
        sn, cs = statistics.mean(math.sin(a) for a in angles), statistics.mean(math.cos(a) for a in angles)
        mean_yaw = math.atan2(sn, cs)
        if math.hypot(sn, cs) < 1e-8:
            reasons.append("undefined circular mean yaw")
        radial = [math.hypot(x-mx, y-my) for x, y in zip(xs, ys)]
        angular = [wrap(a-mean_yaw) for a in angles]
        jitter.update(mean_x=mx, mean_y=my, circular_mean_yaw=mean_yaw,
                      x_std_m=statistics.pstdev(xs), y_std_m=statistics.pstdev(ys),
                      position_std_m=math.sqrt(statistics.mean(r*r for r in radial)),
                      yaw_std_rad=math.sqrt(statistics.mean(a*a for a in angular)),
                      max_position_offset_m=max(radial), max_yaw_offset_rad=max(abs(a) for a in angular))
    integrity = not reasons
    convergence = dict(converged=integrity and confirmation is not None,
                       convergence_time_s=confirmation if integrity else None,
                       threshold_entry_time_s=entry if integrity else None,
                       timeout=integrity and confirmation is None and end-start >= timeout,
                       valid_samples=len(valid))
    convergence["outcome"] = ("INVALID" if not integrity else "NOT_SCORED" if not scoring else
                              "PASS" if convergence["converged"] else "FAIL")
    jitter["outcome"] = ("INVALID" if not integrity else "NOT_SCORED" if not scoring else
                         "PASS" if (jitter["position_std_m"] <= position_jitter_limit
                                    and jitter["yaw_std_rad"] <= yaw_jitter_limit) else "FAIL")
    return dict(measurement="PASS" if integrity else "INVALID", reasons=sorted(set(reasons)),
                started_monotonic_s=start, ended_monotonic_s=end,
                valid_samples=len(valid), invalid_samples=len(samples)-len(valid),
                max_gap_s=max(gaps, default=0), convergence=convergence, jitter=jitter,
                max_position_jump_m=max((math.hypot(b["amcl"]["x"]-a["amcl"]["x"], b["amcl"]["y"]-a["amcl"]["y"])
                                         for a, b in zip(valid, valid[1:])), default=0),
                max_yaw_jump_rad=max((abs(wrap(b["amcl"]["yaw"]-a["amcl"]["yaw"]))
                                     for a, b in zip(valid, valid[1:])), default=0))


def score_trial(endpoints, invalid_reasons, observations,
                position_limit=DEFAULT_POSITION_LIMIT_M, yaw_limit=DEFAULT_YAW_LIMIT_RAD,
                position_jitter_limit=DEFAULT_POSITION_JITTER_M, yaw_jitter_limit=DEFAULT_YAW_JITTER_RAD):
    reasons = list(invalid_reasons)
    for phase in ("initial", "post_motion"):
        observation = (observations or {}).get(phase, {})
        if observation.get("measurement") != "PASS":
            reasons.append(f"{phase}: missing/invalid stationary time series: {observation.get('reasons', [])}")
            continue
        try:
            convergence, jitter = observation["convergence"], observation["jitter"]
            if any(check.get("outcome") not in ("PASS", "FAIL", "NOT_SCORED") for check in (convergence, jitter)):
                raise ValueError("incomplete observation verdicts")
            if (observation.get("reasons") or observation["invalid_samples"] != 0
                    or observation["valid_samples"] != convergence["valid_samples"]
                    or observation["valid_samples"] != jitter["valid_samples"]):
                raise ValueError("inconsistent stationary sample coverage")
            t = convergence["convergence_time_s"]
            if convergence["converged"] is True:
                if not finite([t]) or t < 0 or convergence["timeout"] is not False:
                    raise ValueError("inconsistent convergence confirmation")
            elif convergence["converged"] is not False or t is not None or convergence["timeout"] is not True:
                raise ValueError("convergence has neither confirmation nor a covered timeout")
            if (not finite([jitter["duration_s"], jitter["position_std_m"], jitter["yaw_std_rad"]])
                    or jitter["valid_samples"] < OBSERVATION_MIN_SAMPLES
                    or jitter["duration_s"] < OBSERVATION_MIN_DURATION_S
                    or min(jitter["position_std_m"], jitter["yaw_std_rad"]) < 0):
                raise ValueError("incomplete stationary jitter evidence")
            if position_limit is not None and yaw_limit is not None:
                expected = ("PASS" if convergence["converged"] else "FAIL",
                            "PASS" if (jitter["position_std_m"] <= position_jitter_limit
                                       and jitter["yaw_std_rad"] <= yaw_jitter_limit) else "FAIL")
                if (convergence["outcome"], jitter["outcome"]) != expected:
                    raise ValueError("observation verdicts contradict measured metrics/thresholds")
        except (KeyError, TypeError, ValueError) as exc:
            reasons.append(f"{phase}: incomplete/inconsistent observation checks: {exc}")
    verdict = score(endpoints, reasons, position_limit, yaw_limit)
    if verdict["overall"] not in ("INVALID", "NOT_SCORED"):
        checks = [observations[p][c]["outcome"] for p in ("initial", "post_motion") for c in ("convergence", "jitter")]
        verdict["localization"] = verdict["overall"] = (
            "PASS" if verdict["overall"] == "PASS" and all(c == "PASS" for c in checks) else "FAIL")
    return verdict


def json_safe(value):
    """Keep malformed numeric evidence readable in strict JSON, never as valid numbers."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def http(port, path, body=None, timeout=5):
    # Local runtime RPC must never be sent through an inherited HTTP proxy.
    request = urllib.request.Request(f"http://127.0.0.1:{port}{path}",
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(request, timeout=timeout) as response:
        return json.load(response)


def clean_environment(domain):
    base = {k: os.environ[k] for k in ("HOME", "USER", "LOGNAME", "DISPLAY", "XAUTHORITY", "XDG_RUNTIME_DIR")
            if k in os.environ}
    base.update(PATH="/usr/local/bin:/usr/bin:/bin", LANG="C.UTF-8", PYTHONDONTWRITEBYTECODE="1")
    overlay = ROOT / "packages/omnisim-ros2/install/local_setup.bash"
    result = subprocess.run(["/bin/bash", "--noprofile", "--norc", "-c",
                             'set -e; source /opt/ros/jazzy/setup.bash; source "$1"; env -0',
                             "bash", str(overlay)], env=base, capture_output=True, check=True)
    env = dict(entry.split("=", 1) for entry in result.stdout.decode().split("\0") if entry)
    env.update(RMW_IMPLEMENTATION="rmw_cyclonedds_cpp", ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1",
               OMNISIM_HOME=str(ROOT), OMNISIM_BINARY=str(ROOT / "bin/omnisim-bin"),
               OMNISIM_URDF_USE_SENSORS="1", OMNISIM_NO_WINDOW="1", OMNISIM_REQUIRE_NEWTON="1",
               OMNISIM_NEWTON_MODEL_DEVICE="cpu", QT_QPA_PLATFORM="xcb", LIBGL_ALWAYS_SOFTWARE="1",
               WEBOTS_TMPDIR="/tmp", _HUSKY_AMCL_ENV=str(ROOT))
    env["LD_LIBRARY_PATH"] = os.pathsep.join(
        [str(ROOT / "lib/webots"), str(ROOT / "lib/controller"), env.get("LD_LIBRARY_PATH", "")])
    env["PYTHONPATH"] = os.pathsep.join([env.get("PYTHONPATH", ""), str(ROOT)])
    return env


class Recorder:
    def __init__(self):
        # ROS imports stay out of the engine-free measurement tests.
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import Odometry, OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener
        rclpy.init()
        self.node = Node("husky_amcl_measurement", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.latest, self.counts, self.rewinds = {}, Counter(), Counter()
        self.scans = deque(maxlen=300)
        self.poses = []
        self.edges = {"dynamic": set(), "static": set()}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl, latched)
        self.node.create_subscription(Odometry, "/odom", self.odom, qos_profile_sensor_data)
        self.node.create_subscription(LaserScan, "/scan", self.scan, qos_profile_sensor_data)
        self.node.create_subscription(Clock, "/clock",
                                      lambda m: self.accept("clock", self.seconds(m.clock), self.seconds(m.clock)),
                                      qos_profile_sensor_data)
        self.node.create_subscription(OccupancyGrid, "/map", lambda m: self.latest.update(map=m), latched)
        self.node.create_subscription(TFMessage, "/tf", lambda m: self.transforms(m, "dynamic"), 100)
        self.node.create_subscription(TFMessage, "/tf_static", lambda m: self.transforms(m, "static"), latched)
        self.publisher = self.node.create_publisher(Twist, "/cmd_vel", 1)

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def quaternion_yaw(q):
        if not finite([q.x, q.y, q.z, q.w]) or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 0.001:
            raise ValueError("invalid pose quaternion")
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

    def pose(self, message):
        p = message.pose.pose
        value = dict(frame=message.header.frame_id, stamp=self.seconds(message.header.stamp),
                     x=p.position.x, y=p.position.y, yaw=self.quaternion_yaw(p.orientation),
                     covariance_diagonal=[float(message.pose.covariance[i]) for i in (0, 7, 35)],
                     received_monotonic_ns=time.monotonic_ns())
        if not finite([value["x"], value["y"], value["yaw"], *value["covariance_diagonal"]]):
            raise ValueError("nonfinite ROS pose")
        return value

    def accept(self, key, value, stamp):
        previous = self.latest.get(key + "_stamp")
        if previous is not None and stamp < previous:
            self.rewinds[key] += 1
        self.latest.update({key: value, key + "_stamp": stamp, key + "_receipt": time.monotonic()})
        self.counts[key] += 1

    def amcl(self, message):
        p = self.pose(message)
        self.accept("amcl", p, p["stamp"])
        self.poses.append(p)

    def odom(self, message):
        self.accept("odom", self.pose(message), self.seconds(message.header.stamp))

    def scan(self, message):
        stamp = self.seconds(message.header.stamp)
        self.scans.append(stamp)
        self.accept("scan", message, stamp)

    def transforms(self, message, channel):
        for t in message.transforms:
            if not finite([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]):
                raise ValueError("nonfinite TF translation")
            self.edges[channel].add((t.header.frame_id, t.child_frame_id))
            if (t.header.frame_id, t.child_frame_id) == ("map", "odom"):
                self.latest["map_odom"] = dict(stamp=self.seconds(t.header.stamp),
                                               x=t.transform.translation.x, y=t.transform.translation.y,
                                               yaw=self.quaternion_yaw(t.transform.rotation))

    def spin(self, seconds=0.05):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=seconds)

    def tf_at(self, stamp):
        from rclpy.time import Time
        result = {}
        for target, source in (("map", "odom"), ("odom", "base_link"), ("base_link", "base_laser")):
            t = self.buffer.lookup_transform(target, source, Time(nanoseconds=round(stamp * 1e9)))
            if not finite([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]):
                raise ValueError("nonfinite scan-time TF translation")
            result[f"{target}->{source}"] = dict(stamp=self.seconds(t.header.stamp),
                                                 translation=[t.transform.translation.x, t.transform.translation.y,
                                                              t.transform.translation.z],
                                                 yaw=self.quaternion_yaw(t.transform.rotation))
        return result


class Trial:
    def __init__(self, out, args):
        self.out, self.args = out, args
        self.processes, self.streams = [], []
        self.recorder = None
        self.bridge_id = None
        self.sibling = None
        self.data = dict(schema_version=2, trial_id=uuid.uuid4().hex, started_utc=datetime.now(timezone.utc).isoformat(),
                         endpoints={}, invalid_reasons=[], processes=[],
                         configuration=dict(world=str(WORLD), map=str(MAP), params=str(PARAMS),
                                            ros_domain_id=args.ros_domain_id, lidar_layer=0,
                                            map_from_world=[0, 0, 0], root_frame="base_link",
                                            root_to_base_link=[0, 0, 0], integrity_bounds=INTEGRITY,
                                            acceptance_thresholds=dict(position_m=args.max_position_error_m,
                                                                       absolute_yaw_rad=args.max_yaw_error_rad),
                                            observation=dict(hold_s=args.convergence_hold_s,
                                                             timeout_s=args.convergence_timeout_s,
                                                             interval_s=OBSERVATION_INTERVAL_S, max_gap_s=OBSERVATION_MAX_GAP_S,
                                                             minimum_samples=OBSERVATION_MIN_SAMPLES,
                                                             minimum_duration_s=OBSERVATION_MIN_DURATION_S,
                                                             position_jitter_limit_m=args.max_position_jitter_m,
                                                             yaw_jitter_limit_rad=args.max_yaw_jitter_rad)))

    def launch(self, name, command):
        stream = (self.out / f"{name}.log").open("w")
        self.streams.append(stream)
        process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        self.processes.append((name, process))
        self.data["processes"].append(dict(name=name, pid=process.pid, command=command))

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for name, process in self.processes:
                if process.poll() is not None:
                    raise RuntimeError(f"{name} exited {process.returncode}; see {name}.log")
            if predicate():
                return
            if self.recorder:
                self.recorder.spin()
            else:
                time.sleep(0.1)
        raise RuntimeError(f"timed out waiting for {label}")

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        self.wait(lambda: time.monotonic() >= deadline, seconds + 1, "observation interval")

    def service(self, kind, name):
        client = self.recorder.node.create_client(kind, name)
        try:
            self.wait(client.service_is_ready, 5, name)
            future = client.call_async(kind.Request())
            self.wait(future.done, 5, name + " response")
            return future.result()
        finally:
            self.recorder.node.destroy_client(client)

    def bridge_state(self):
        state = http(8783, "/get_robot_state", {})
        if self.bridge_id and state.get("bridge_instance_id") != self.bridge_id:
            raise RuntimeError("Husky bridge instance changed")
        return state

    def zero(self):
        self.bridge_state()
        response = http(8783, "/set_velocity", dict(linear=0, angular=0, wait=False))
        if not response.get("accepted"):
            raise RuntimeError("zero velocity was not accepted")
        return response

    def provenance(self):
        files = [WORLD, MAP, MAP.with_suffix(".pgm"), PARAMS, Path(__file__),
                 ROOT / "bin/omnisim-bin", ROOT / "lib/controller/libController.so",
                 ROOT / "src/omnisim/physics/omnisim_newton_runtime.py",
                 ROOT / "packages/omnisim-ros2/src/omnisim_ros2/omnisim_ros2/odom_node.py"]
        hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        if hashes[str(WORLD.relative_to(ROOT))] != "aeb4fd8a4f3852fde2d3ab6781d7deb40ec5eb111513ff66ed0cc54e92df3a2a":
            raise RuntimeError("world changed since deterministic map derivation")
        git_env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")

        def git(*args):
            return subprocess.check_output(["git", *args], cwd=ROOT, env=git_env, text=True).strip()

        import omnisim_ros2
        module = Path(omnisim_ros2.__file__).resolve()
        if not module.is_relative_to(ROOT / "packages/omnisim-ros2/src"):
            raise RuntimeError("ROS module resolves outside this worktree")
        helper = importlib.util.find_spec("omnisim_newton_runtime")
        if helper is not None and not Path(helper.origin).resolve().is_relative_to(ROOT):
            raise RuntimeError("installed Newton helper shadows checkout helper")
        urdf = ET.parse(ROOT / "projects/robots/clearpath/husky_description/urdf/husky.urdf").getroot()
        roots = ({e.attrib["name"] for e in urdf.findall("link")}
                 - {e.find("child").attrib["link"] for e in urdf.findall("joint")})
        if roots != {"base_link"} or urdf.find("link[@name='base_link']/visual") is None:
            raise RuntimeError("fixture root/base_link correspondence changed")
        self.data["provenance"] = dict(commit=git("rev-parse", "HEAD"), branch=git("branch", "--show-current"),
                                       worktree_status=git("status", "--short"), sha256=hashes,
                                       python=sys.version, ros_module=str(module), machine=os.uname().release,
                                       physics_packages={n: importlib.metadata.version(n)
                                                         for n in ("warp-lang", "newton", "mujoco", "mujoco-warp")},
                                       environment={k: os.environ.get(k) for k in (
                                           "OMNISIM_HOME", "OMNISIM_BINARY", "LD_LIBRARY_PATH", "PYTHONPATH",
                                           "AMENT_PREFIX_PATH", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION")})

    def run(self):
        from lifecycle_msgs.srv import GetState
        for port in PORTS:
            with socket.socket() as s:
                s.settimeout(0.2)
                if s.connect_ex(("127.0.0.1", port)) == 0:
                    raise RuntimeError(f"port {port} already in use; refusing to reuse runtime")
        sibling = WORLD.with_name(".harness_" + WORLD.name)
        if sibling.exists():
            raise RuntimeError(f"existing harness sibling must be preserved: {sibling}")
        self.provenance()
        self.recorder = Recorder()
        r = self.recorder
        self.pump(0.5)
        others = [name for name, _ in r.node.get_node_names_and_namespaces() if name != r.node.get_name()]
        if others:
            raise RuntimeError(f"ROS domain already contains nodes: {others}; select another --ros-domain-id")
        print("Starting owned realtime engine/harness", flush=True)
        self.launch("harness", [sys.executable, "-B", str(ROOT / "scripts/harness/omnisim_harness.py"),
                                "--host", "127.0.0.1", "--port", "6789", "--supervisor-port", "6790",
                                "--engine-mode", "realtime"])

        def harness_ready():
            try:
                http(6789, "/sim/state", timeout=0.5)
                return True
            except OSError:
                return False
        self.wait(harness_ready, 10, "harness HTTP")
        self.sibling = sibling
        self.data["world_load"] = http(6789, "/world/load",
                                       dict(path=str(WORLD), light=True, with_supervisor=True, wait_s=40), timeout=45)
        self.wait(lambda: http(6789, "/sim/state").get("load_state") == "complete", 40, "world completion")
        state = http(6789, "/sim/state")
        self.data["runtime_state"] = state
        self.data["cold_start"] = self.data["world_load"].get("hot_reloaded") is False
        if (not state.get("load_ok") or not state.get("running") or not state.get("supervisor_connected")
                or state.get("binary") != str(ROOT / "bin/omnisim-bin")):
            raise RuntimeError("world/controller startup did not establish the owned runtime")
        backend = self.out / "engine.log.newton.json"
        self.wait(lambda: backend.exists() and json.loads(backend.read_text()).get("finalised"), 20, "Newton finalization")
        self.data["backend"] = json.loads(backend.read_text())
        b = self.data["backend"]
        if b.get("backend") != "newton" or b.get("degraded") is not False or b.get("runtime", {}).get("device") != "cpu":
            raise RuntimeError("backend evidence does not establish non-degraded CPU Newton")
        state = self.bridge_state()
        self.bridge_id = state["bridge_instance_id"]
        sensors = http(8783, "/list_sensors", {})
        laser = next(s for s in sensors["sensors"] if s["name"] == "base_laser" and s["type"] == "Lidar")
        self.data["laser_mount"] = laser["mount"]
        self.data["bridge_instance_id"] = self.bridge_id
        http(8783, "/read_sensor", dict(sensor="base_laser"))  # Enable before ROS starts; first read can be warming up.
        for node, extra in (
            ("clock_node", ["use_bridge_clock:=true", "publish_rate_hz:=20.0"]),
            ("odom_node", ["use_sim_time:=true", "publish_rate_hz:=20.0"]),
            ("sensor_node", ["use_sim_time:=true", "lidar_layer:=0", "publish_imu:=false", "publish_gps:=false"]),
            ("command_node", ["use_sim_time:=true", "enable_joint_command:=false"]),
        ):
            parameters = ["bridge_url:=http://127.0.0.1:8783", *extra]
            self.launch(node, ["ros2", "run", "omnisim_ros2", node, "--ros-args",
                               *[v for parameter in parameters for v in ("-p", parameter)]])
        self.launch("localization", ["ros2", "launch", "nav2_bringup", "localization_launch.py",
                                     f"map:={MAP}", f"params_file:={PARAMS}", "use_sim_time:=true",
                                     "autostart:=true", "use_composition:=False", "use_respawn:=False"])
        self.wait(lambda: all(k in r.latest for k in ("clock", "odom", "scan", "map", "amcl", "map_odom")),
                  35, "localization interfaces")
        self.data["lifecycle"] = {name: self.service(GetState, f"/{name}/get_state").current_state.label
                                  for name in ("map_server", "amcl")}
        if set(self.data["lifecycle"].values()) != {"active"}:
            raise RuntimeError("localization lifecycle is not active")
        self.readiness()
        self.endpoint("initial")
        self.observe_stationary("initial")
        initial_updates = len(r.poses)
        self.move()
        self.data["motion_amcl_updates"] = len(r.poses) - initial_updates
        self.endpoint("post_motion")
        self.observe_stationary("post_motion")
        if self.data["motion_amcl_updates"] < 1:
            raise RuntimeError("no AMCL estimates during translation/turn")
        first, last = (world_to_map(self.data["endpoints"][label]["ground_truth"][-1]["node"])
                       for label in ("initial", "post_motion"))
        self.data["movement"].update(independent_endpoint_translation_m=math.hypot(last["x"]-first["x"], last["y"]-first["y"]),
                                     independent_endpoint_rotation_rad=wrap(last["yaw"]-first["yaw"]))
        self.audit_tf()
        if r.rewinds:
            raise RuntimeError(f"simulator message timestamps rewound: {dict(r.rewinds)}")

    def readiness(self):
        r = self.recorder
        m, s = r.latest["map"], r.latest["scan"]
        metadata = dict(frame=m.header.frame_id, width=m.info.width, height=m.info.height, resolution=m.info.resolution,
                        origin=[m.info.origin.position.x, m.info.origin.position.y], counts=dict(Counter(m.data)))
        self.data["map_metadata"] = metadata
        if (metadata["frame"] != "map" or metadata["width"] != 220 or metadata["height"] != 220
                or abs(metadata["resolution"]-.05) > 1e-8
                or metadata["origin"] != [-5.5, -5.5] or abs(m.info.origin.orientation.w-1) > 1e-8
                or metadata["counts"] != {-1: 8400, 0: 39488, 100: 512}):
            raise RuntimeError("loaded map metadata/cell counts differ from deterministic fixture")
        raw = http(8783, "/read_sensor", dict(sensor="base_laser"))
        if raw.get("layout", {}).get("number_of_layers") != 1 or s.header.frame_id != "base_laser" or len(s.ranges) != 541:
            raise RuntimeError("laser frame/layer/layout changed")
        root = world_to_map(http(6789, "/scene/node/HUSKY"))
        mount = self.data["laser_mount"]
        if (mount is None or mount["rotation_matrix"] != [1, 0, 0, 0, 1, 0, 0, 0, 1]
                or abs(root["x"]) > .02 or abs(root["y"]) > .02 or abs(root["yaw"]) > .02):
            raise RuntimeError("initial root/laser frame does not match known-wall geometry")
        observed = [float(s.ranges[min(range(len(s.ranges)), key=lambda i: abs(s.angle_min+i*s.angle_increment-a))])
                    for a in (0, math.pi/2, -math.pi/2)]
        expected = [3-root["x"]-mount["translation"][0], 2-root["y"]-mount["translation"][1],
                    1.5+root["y"]+mount["translation"][1]]
        self.data["scan_geometry"] = dict(layer=0, beams=len(s.ranges), finite=sum(math.isfinite(v) for v in s.ranges),
                                          front_left_right_observed_m=[v if math.isfinite(v) else None for v in observed],
                                          front_left_right_expected_m=expected,
                                          tolerance_m=.05, stamp=r.seconds(s.header.stamp))
        if not finite(observed) or any(abs(a-b) > .05 for a, b in zip(observed, expected)):
            raise RuntimeError("selected laser scan does not match known walls")
        self.audit_tf()
        scan_subscribers = {i.node_name for i in r.node.get_subscriptions_info_by_topic("/scan")}
        command_subscribers = {i.node_name for i in r.node.get_subscriptions_info_by_topic("/cmd_vel")}
        if "amcl" not in scan_subscribers or "omnisim_command" not in command_subscribers:
            raise RuntimeError("AMCL scan or command subscription missing")
        before = {k: r.latest[k+"_stamp"] for k in ("clock", "odom", "scan")}
        self.pump(0.5)
        after = {k: r.latest[k+"_stamp"] for k in before}
        self.data["readiness_timing"] = dict(before=before, after=after)
        if any(after[k] <= before[k] for k in before):
            raise RuntimeError("clock/odom/scan timestamps are not advancing")
        # AMCL future-dates its first map->odom transform. Do not request the
        # initial endpoint until scan time has entered the real TF history.
        from tf2_ros import TransformException

        def scan_tf_ready():
            try:
                r.tf_at(r.latest["scan_stamp"])
                return True
            except TransformException:
                return False
        self.wait(scan_tf_ready, 5, "initial scan-time TF history")

    def audit_tf(self):
        r = self.recorder
        publishers = {topic: sorted(i.node_name for i in r.node.get_publishers_info_by_topic(topic))
                      for topic in ("/tf", "/tf_static")}
        self.data["tf_authority"] = dict(publishers=publishers, observed_edges={k: sorted(v) for k, v in r.edges.items()},
                                         method="publisher graph + observed edges + pinned odom_node "
                                                "publishing only odom->base_link")
        if (publishers["/tf"] != ["amcl", "omnisim_odom"] or publishers["/tf_static"] != ["omnisim_sensors"]
                or r.edges["dynamic"] != {("map", "odom"), ("odom", "base_link")}
                or any(parent != "base_link" for parent, _ in r.edges["static"])):
            raise RuntimeError("unexpected/competing TF publisher or frame edge")

    def ground_truth(self):
        r = self.recorder
        clock_before = r.latest["clock"]
        requested = time.monotonic_ns()
        node = http(6789, "/scene/node/HUSKY")
        received = time.monotonic_ns()
        state = self.bridge_state()
        self.pump(.1)
        return dict(node={key: node.get(key) for key in ("def", "position", "orientation")},
                    requested_monotonic_ns=requested, received_monotonic_ns=received,
                    clock_before=clock_before, clock_after=r.latest["clock"],
                    bridge_sim_time=state["sim_time"], v_linear=state["v_linear"], v_angular=state["v_angular"],
                    acquisition_note="host request/response bracket, not an atomic engine timestamp")

    def endpoint(self, label):
        from std_srvs.srv import Empty
        r = self.recorder
        evidence = dict(ground_truth=[], tf_at_scan={})
        self.data["endpoints"][label] = evidence  # Preserve partial evidence on failure.
        self.zero()
        self.pump(1)
        evidence["ground_truth"].append(self.ground_truth())
        prior = r.latest["map_odom"].copy()
        count = len(r.poses)
        evidence["refresh_request_clock"] = r.latest["clock"]
        self.service(Empty, "/request_nomotion_update")
        self.wait(lambda: len(r.poses) > count and r.latest["amcl"]["stamp"] >= evidence["refresh_request_clock"],
                  5, "fresh stationary AMCL scan update")
        evidence["amcl"] = r.latest["amcl"].copy()
        stamp = evidence["amcl"]["stamp"]
        self.wait(lambda: any(abs(stamp-s) < 1e-6 for s in r.scans), 2, "scan matching AMCL timestamp")
        evidence["matching_scan_stamp"] = next(s for s in r.scans if abs(stamp-s) < 1e-6)
        self.pump(.5)
        evidence["ground_truth"].append(self.ground_truth())
        self.wait(lambda: r.latest["clock"] >= evidence["ground_truth"][0]["clock_before"] + 1.1,
                  5, "stationary simulator-time interval")
        evidence["ground_truth"].append(self.ground_truth())
        evidence["tf_at_scan"] = r.tf_at(stamp)
        timing = {k: r.latest[k+"_stamp"] for k in ("clock", "odom", "scan")}
        evidence["timing"] = timing
        evidence["map_odom_before_refresh"], evidence["map_odom_after_refresh"] = prior, r.latest["map_odom"].copy()
        reasons = endpoint_reasons(evidence)
        if max(timing.values())-min(timing.values()) > INTEGRITY["time_domain_difference_s"]:
            reasons.append("clock/odom/scan simulator-time domains disagree")
        if any(time.monotonic()-r.latest[k+"_receipt"] > INTEGRITY["message_age_wall_s"] for k in ("clock", "odom", "scan")):
            reasons.append("clock/odom/scan receipt is stale")
        if reasons:
            raise RuntimeError(f"{label}: {'; '.join(reasons)}")
        evidence["errors"] = pose_errors(evidence["amcl"], world_to_map(evidence["ground_truth"][-1]["node"]))
        print(label, json.dumps(evidence["errors"]), flush=True)

    def move(self):
        from geometry_msgs.msg import Twist
        r = self.recorder
        self.data["movement"] = dict(commanded_linear_m_s=.12, commanded_angular_rad_s=.18,
                                     target_forward_m=.4, target_turn_rad=.35)
        for mode, target, timeout in (("forward", .4, 15), ("turn", .35, 12)):
            start = r.latest["odom"].copy()
            last_command = 0

            def achieved():
                now = r.latest["odom"]
                return (math.hypot(now["x"]-start["x"], now["y"]-start["y"]) if mode == "forward"
                        else abs(wrap(now["yaw"]-start["yaw"])))

            def command():
                nonlocal last_command
                now = r.latest["odom"]
                if (time.monotonic()-r.latest["odom_receipt"] > INTEGRITY["message_age_wall_s"]
                        or not (-.2 < now["x"] < 1 and abs(now["y"]) < .4 and abs(now["yaw"]) < .7)):
                    raise RuntimeError("motion odometry stale or robot outside bounded clear-space region")
                if achieved() >= target:
                    return True
                v = Twist()
                v.linear.x, v.angular.z = (.12, 0.) if mode == "forward" else (0., .18)
                if time.monotonic()-last_command >= .1:
                    r.publisher.publish(v)
                    last_command = time.monotonic()
                return False
            try:
                self.wait(command, timeout, mode + " displacement")
            finally:
                self.zero()
            self.pump(.5)
            self.data["movement"][mode+"_odom_achieved"] = achieved()
            if achieved() <= (.25 if mode == "forward" else .2):
                raise RuntimeError("motion did not exceed AMCL update threshold")
        self.zero()

    def observe_stationary(self, phase):
        """Fresh scan updates bracketed by independent stationary GT reads."""
        from std_srvs.srv import Empty
        from tf2_ros import TransformException
        r, args = self.recorder, self.args
        start = time.monotonic()
        samples = []
        analysis_options = dict(hold=args.convergence_hold_s, timeout=args.convergence_timeout_s,
                                position_limit=(args.max_position_error_m if args.max_position_error_m is not None
                                                else DEFAULT_POSITION_LIMIT_M),
                                yaw_limit=(args.max_yaw_error_rad if args.max_yaw_error_rad is not None
                                           else DEFAULT_YAW_LIMIT_RAD),
                                position_jitter_limit=args.max_position_jitter_m,
                                yaw_jitter_limit=args.max_yaw_jitter_rad, scoring=not args.no_score)
        with (self.out / "localization_timeseries.jsonl").open("a") as stream:
            while time.monotonic()-start < args.convergence_timeout_s:
                cycle = time.monotonic()
                sample = dict(phase=phase, index=len(samples), monotonic_s=cycle)
                try:
                    before = self.ground_truth()
                    count = len(r.poses)
                    request_clock = r.latest["clock"]
                    self.service(Empty, "/request_nomotion_update")
                    self.wait(lambda: len(r.poses) > count and r.latest["amcl"]["stamp"] >= request_clock,
                              2, "stationary observation AMCL update")
                    estimate = r.latest["amcl"].copy()
                    self.wait(lambda: any(abs(estimate["stamp"]-s) < 1e-6 for s in r.scans), 1, "observation scan timestamp")
                    after = self.ground_truth()
                    sample.update(monotonic_s=estimate["received_monotonic_ns"]/1e9, amcl=estimate,
                                  ground_truth=[before, after], refresh_request_clock=request_clock,
                                  matching_scan_stamp=next(s for s in r.scans if abs(estimate["stamp"]-s) < 1e-6),
                                  tf_at_scan=r.tf_at(estimate["stamp"]),
                                  timing={k: r.latest[k+"_stamp"] for k in ("clock", "odom", "scan")},
                                  receipt_ages_wall_s={k: time.monotonic()-r.latest[k+"_receipt"]
                                                       for k in ("clock", "odom", "scan")})
                    anchor = world_to_map(samples[0]["ground_truth"][0]["node"]) if samples else None
                    reasons = series_sample_reasons(sample, samples[-1] if samples else None, anchor)
                    sample.update(valid=not reasons, invalid_reasons=reasons)
                    if not reasons:
                        reference = world_to_map(after["node"])
                        errors = pose_errors(estimate, reference)
                        sample.update(ground_truth_map=reference, position_error_m=errors["position_error_m"],
                                      absolute_yaw_error_rad=abs(errors["yaw_error_rad"]))
                except (OSError, RuntimeError, ValueError, TransformException) as exc:
                    sample.update(valid=False, invalid_reasons=[str(exc)])
                samples.append(sample)
                stream.write(json.dumps(json_safe(sample), allow_nan=False)+"\n")
                stream.flush()
                summary = analyze_observation(samples, start, time.monotonic(), **analysis_options)
                # A gap/malformed sample cannot establish continuous coverage;
                # retain it and stop instead of fabricating replacements.
                if not sample["valid"] or summary["convergence"]["converged"]:
                    break
                remaining = OBSERVATION_INTERVAL_S-(time.monotonic()-cycle)
                if remaining > 0:
                    self.pump(min(remaining, max(0, args.convergence_timeout_s-(time.monotonic()-start))))
        summary = analyze_observation(samples, start, time.monotonic(), **analysis_options)
        self.data.setdefault("stationary_observations", {})[phase] = summary
        print(phase, "observation", json.dumps(summary), flush=True)
        if summary["measurement"] != "PASS":
            raise RuntimeError(f"{phase} stationary observation INVALID: {summary['reasons']}")

    def cleanup(self):
        if self.bridge_id:
            try:
                self.data["cleanup_zero"] = self.zero()
            except Exception as exc:
                self.data["invalid_reasons"].append(f"cleanup zero failed: {exc}")
        # Each Popen has its own session. Signal whole owned groups, including
        # ros2 CLI children; never discover/kill another user's engine by name.
        for sig, timeout in ((signal.SIGINT, 6), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            for _, process in reversed(self.processes):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + timeout
            for _, process in reversed(self.processes):
                try:
                    process.wait(timeout=max(0, deadline-time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
            if all(p.poll() is not None for _, p in self.processes):
                # An exited ros2 wrapper can leave children in its group. The
                # next signal still targets that owned group if it exists.
                remaining = False
                for _, process in self.processes:
                    try:
                        os.killpg(process.pid, 0)
                        remaining = True
                    except ProcessLookupError:
                        pass
                if not remaining:
                    break
        self.data["process_exit"] = {name: p.poll() for name, p in self.processes}
        if any(code != 0 for code in self.data["process_exit"].values()):
            self.data["invalid_reasons"].append("owned process did not exit cleanly")
        listeners = {}
        for port in PORTS:
            with socket.socket() as s:
                s.settimeout(.2)
                listeners[str(port)] = s.connect_ex(("127.0.0.1", port)) == 0
        self.data["cleanup_listeners"] = listeners
        if self.processes and any(listeners.values()):
            self.data["invalid_reasons"].append("runtime listener remains after owned-process cleanup")
        if self.sibling and self.sibling.exists() and not any(listeners.values()):
            text = self.sibling.read_text()
            if text.startswith(WORLD.read_text()) and 'controller "harness_supervisor"' in text:
                self.sibling.unlink()
            else:
                self.data["invalid_reasons"].append("changed harness sibling preserved during cleanup")
        if self.recorder:
            import rclpy
            self.data["amcl_updates"] = self.recorder.poses
            self.data["message_counts"] = dict(self.recorder.counts)
            self.recorder.node.destroy_node()
            rclpy.shutdown()
        for stream in self.streams:
            stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="new directory; default: fresh /tmp/husky-amcl-* directory")
    parser.add_argument("--ros-domain-id", type=int, default=83)
    parser.add_argument("--max-position-error-m", type=float, default=DEFAULT_POSITION_LIMIT_M,
                        help="limit applied to both endpoints (default: 0.10 m)")
    parser.add_argument("--max-yaw-error-rad", type=float, default=DEFAULT_YAW_LIMIT_RAD,
                        help="absolute wrapped-yaw limit at both endpoints (default: 0.05 rad)")
    parser.add_argument("--no-score", action="store_true", help="collect valid measurements without scoring localization")
    parser.add_argument("--convergence-hold-s", type=float, default=DEFAULT_CONVERGENCE_HOLD_S)
    parser.add_argument("--convergence-timeout-s", type=float, default=DEFAULT_CONVERGENCE_TIMEOUT_S)
    parser.add_argument("--max-position-jitter-m", type=float, default=DEFAULT_POSITION_JITTER_M,
                        help="maximum population 2D RMS position dispersion")
    parser.add_argument("--max-yaw-jitter-rad", type=float, default=DEFAULT_YAW_JITTER_RAD,
                        help="maximum wrapped circular-mean yaw RMS")
    args = parser.parse_args()
    limits = (args.max_position_error_m, args.max_yaw_error_rad)
    if any(not math.isfinite(v) or v < 0 for v in limits):
        parser.error("acceptance thresholds must be finite and nonnegative")
    if (not finite([args.convergence_hold_s, args.convergence_timeout_s, args.max_position_jitter_m, args.max_yaw_jitter_rad])
            or args.convergence_hold_s < OBSERVATION_MIN_DURATION_S
            or args.convergence_timeout_s < args.convergence_hold_s + OBSERVATION_MAX_GAP_S
            or min(args.max_position_jitter_m, args.max_yaw_jitter_rad) < 0):
        parser.error("hold must be >= 2 s, timeout >= hold + 1 s, and jitter limits finite/nonnegative")
    if args.no_score:
        args.max_position_error_m = args.max_yaw_error_rad = None
        limits = (None, None)
    if not 0 <= args.ros_domain_id <= 232:
        parser.error("ROS domain ID must be between 0 and 232")
    # Re-exec before importing ROS so historical Python/ament paths cannot win.
    if os.environ.get("_HUSKY_AMCL_ENV") != str(ROOT):
        try:
            env = clean_environment(args.ros_domain_id)
        except (OSError, subprocess.CalledProcessError, ValueError) as exc:
            out = args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="husky-amcl-"))
            if args.output:
                out.mkdir(parents=True, exist_ok=False)
            (out / "result.json").write_text(json.dumps(
                dict(schema_version=1, verdicts=dict(measurement="INVALID", localization="NOT_SCORED", overall="INVALID"),
                     invalid_reasons=[f"fresh ROS environment failed: {exc}"]), indent=2) + "\n")
            print(f"INVALID: {exc}; evidence: {out}")
            return 2
        os.execve("/usr/bin/python3.12", ["python3.12", "-B", str(Path(__file__).resolve()), *sys.argv[1:]], env)
    out = args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="husky-amcl-"))
    if args.output:
        out.mkdir(parents=True, exist_ok=False)
    os.environ.update(OMNISIM_LOG_PATH=str(out / "engine.log"), ROS_LOG_DIR=str(out / "ros-log"),
                      XDG_CACHE_HOME=str(out / "cache"), PYTHONUNBUFFERED="1")
    trial = Trial(out, args)

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")
    signal.signal(signal.SIGTERM, interrupted)
    try:
        trial.run()
    except (Exception, KeyboardInterrupt) as exc:
        trial.data["invalid_reasons"].append(str(exc))
        (out / "failure.log").write_text(traceback.format_exc())
        print(f"INVALID: {exc}", flush=True)
    finally:
        try:
            trial.cleanup()
        except Exception as exc:
            trial.data["invalid_reasons"].append(f"cleanup failed: {exc}")
        trial.data["verdicts"] = score_trial(trial.data["endpoints"], trial.data["invalid_reasons"],
                                             trial.data.get("stationary_observations"), *limits,
                                             args.max_position_jitter_m, args.max_yaw_jitter_rad)
        trial.data["finished_utc"] = datetime.now(timezone.utc).isoformat()
        (out / "result.json").write_text(json.dumps(json_safe(trial.data), indent=2, allow_nan=False) + "\n")
    print(json.dumps(trial.data["verdicts"]), f"evidence: {out}", flush=True)
    return {"PASS": 0, "NOT_SCORED": 0, "FAIL": 1, "INVALID": 2}[trial.data["verdicts"]["overall"]]


if __name__ == "__main__":
    raise SystemExit(main())
