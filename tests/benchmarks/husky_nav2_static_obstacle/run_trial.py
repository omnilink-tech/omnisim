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

"""One cold, all-in-WSL Husky static-obstacle trial; results are sampled evidence.

Run from a sourced Jazzy + OmniSim ROS workspace. No engine/ROS nodes may already
own the normal ports. Artifacts go outside the repository by default. This does
not certify continuous collision freedom or change the original Nav2 baseline.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request


ROOT = Path(__file__).resolve().parents[3]
WORLD = "projects/samples/demos/worlds/chat/husky_nav2_static_obstacle.omniworld"
PARAMS = ROOT / "packages/omnisim-ros2/src/omnisim_ros2_nav2/params/husky_static_obstacle_params.yaml"
BOX = [(1.7, -0.5), (2.3, -0.5), (2.3, 0.5), (1.7, 0.5)]


def hull(x, y, yaw):
    """URDF collision envelope, including wheels; excludes visual-only bumpers."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in [(-0.4937, -0.33465), (0.4937, -0.33465),
                         (0.4937, 0.33465), (-0.4937, 0.33465)]]


def clearance(poly):
    """Convex SAT overlap/touch test, then Euclidean polygon separation (metres)."""
    separated = False
    for shape in (poly, BOX):
        for a, b in zip(shape, shape[1:] + shape[:1]):
            axis = (a[1] - b[1], b[0] - a[0])
            p = [v[0] * axis[0] + v[1] * axis[1] for v in poly]
            q = [v[0] * axis[0] + v[1] * axis[1] for v in BOX]
            separated |= max(p) < min(q) or max(q) < min(p)
    if not separated:
        return 0.0, True

    def point_segment(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = max(0, min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)))
        return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)

    return min(point_segment(p, a, b) for points, edges in ((poly, BOX), (BOX, poly))
               for p in points for a, b in zip(edges, edges[1:] + edges[:1])), False


def check_geometry():
    for x, y, yaw, expected in [(0, 0, 0, 1.2063), (2, 1, 0, 0.16535),
                                (2, 1, math.pi / 2, 0.0063)]:
        distance, overlap = clearance(hull(x, y, yaw))
        assert not overlap and abs(distance - expected) < 1e-9
    assert clearance(hull(2, 0, 0))[1]
    assert clearance(hull(1.7 - 0.4937, 0, 0))[0] < 1e-9
    # Diagonal separation exercises vertex-to-edge rather than AABB distance.
    distance, overlap = clearance(hull(0, 2, math.pi / 4))
    assert not overlap and distance > 1
    print("geometry checks PASS")


def http(port, path, body=None, timeout=20):
    request = urllib.request.Request(f"http://127.0.0.1:{port}{path}",
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


class StartupTiming:
    """Compact independent observer; never contributes to the trial verdict."""

    FRAMES = {('map', 'odom'), ('odom', 'base_link'), ('base_link', 'base_laser')}

    def __init__(self, out):
        """Open the standalone diagnostic stream without changing scoring."""
        self.lock = threading.Lock()
        self.errors = []
        self.stream = None
        try:
            self.stream = (out / 'startup_timing.jsonl').open('x')
        except OSError as exc:
            self.error(exc)

    def error(self, exc):
        """Expose diagnostic failures separately from measurement validity."""
        self.errors.append(str(exc))
        print(f"startup timing diagnostic error: {exc}", file=sys.stderr, flush=True)

    def record(self, channel, **fields):
        """Append one receipt or boundary with both host clocks."""
        row = {'channel': channel, 'received_monotonic_ns': time.monotonic_ns(),
               'received_wall_ns': time.time_ns(), **fields}
        try:
            with self.lock:
                if self.stream is not None:
                    self.stream.write(json.dumps(row, separators=(',', ':')) + '\n')
        except (OSError, ValueError, TypeError) as exc:
            self.error(exc)

    def message(self, channel, message, info=None):
        """Serialize stamps and relevant frames, excluding sensor payloads."""
        try:
            field = info.get if isinstance(info, dict) else lambda key: getattr(info, key, None)
            metadata = {'dds_source_ns': field('source_timestamp'),
                        'dds_received_ns': field('received_timestamp')}
            if channel in ('tf', 'tf_static'):
                transforms = [{'parent': t.header.frame_id, 'child': t.child_frame_id,
                               'stamp_ns': t.header.stamp.sec * 1000000000 + t.header.stamp.nanosec}
                              for t in message.transforms
                              if (t.header.frame_id, t.child_frame_id) in self.FRAMES]
                if transforms:
                    self.record(channel, transforms=transforms, **metadata)
            else:
                stamp = message.clock if channel == 'clock' else message.header.stamp
                fields = {'stamp_ns': stamp.sec * 1000000000 + stamp.nanosec}
                if channel == 'scan':
                    fields['frame_id'] = message.header.frame_id
                self.record(channel, **fields, **metadata)
        except Exception as exc:
            self.error(exc)

    def boundary(self, event, **fields):
        """Record a runner-side lifecycle or action boundary."""
        self.record('boundary', event=event, **fields)

    def nav_log(self, text):
        """Import original Nav2 log times after process cleanup."""
        # These are imported log events, not live observer receipt timestamps.
        for line in text.splitlines():
            if any(token in line for token in ('costmap]: Activating', 'Managed nodes are active',
                                               'Begin navigating', 'Message Filter dropping message')):
                try:
                    wall = line.split('[INFO] [', 1)[1].split(']', 1)[0]
                    sec, nano = wall.split('.')
                    self.record('nav_log', event_wall_ns=int(sec) * 1000000000 + int(nano.ljust(9, '0')),
                                line=line)
                except (IndexError, ValueError) as exc:
                    self.error(f"cannot timestamp Nav2 log: {exc}: {line}")

    def close(self):
        """Persist diagnostic completeness and close the stream."""
        self.record('diagnostic_end', errors=self.errors)
        if self.stream is not None:
            try:
                self.stream.close()
            except OSError as exc:
                self.error(exc)


class PoseRecorder:
    """Independent ROS receiver and cached bridge sampler; never interpolate poses.

    DDS writer/reader and callback times distinguish publication from delivery
    and scheduling. A second read-only channel covers gaps in the ROS publisher
    without changing that publisher's rate or the navigation stack.
    """

    def __init__(self, trial):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        self.trial = trial
        self.closed = False
        self.node = rclpy.create_node("husky_pose_recorder")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.done = threading.Event()
        self.rows = []
        self.clocks = []
        self.errors = []
        self.lock = threading.Lock()
        self.stream = (trial.out / "timing.jsonl").open("w")
        self.node.create_subscription(Odometry, "/odom", self.odom, 1000)
        self.node.create_subscription(Clock, "/clock", self.clock, 1000)
        self.node.create_subscription(LaserScan, "/scan",
                                      lambda m, i: trial.startup.message("scan", m, i), qos_profile_sensor_data)
        self.node.create_subscription(TFMessage, "/tf", lambda m, i: trial.startup.message("tf", m, i), 100)
        self.node.create_subscription(TFMessage, "/tf_static",
                                      lambda m, i: trial.startup.message("tf_static", m, i),
                                      QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        trial.startup.boundary("observer_ready")
        self.ros_thread = threading.Thread(target=self.spin, daemon=True)
        self.http_thread = threading.Thread(target=self.bridge, daemon=True)
        self.ros_thread.start()
        self.http_thread.start()

    def spin(self):
        try:
            self.executor.spin()
        except Exception as exc:
            self.errors.append(f"ROS receiver stopped: {exc}")
            self.trial.startup.error(f"ROS receiver stopped: {exc}")

    def record(self, row):
        with self.lock:
            self.stream.write(json.dumps(row) + "\n")
            if row["channel"] == "clock":
                self.clocks.append(row)
            else:
                self.rows.append(row)

    @staticmethod
    def receipt(message, info, channel):
        stamp = message.header.stamp
        field = info.get if isinstance(info, dict) else lambda key: getattr(info, key, None)
        return {"channel": channel, "stamp": stamp.sec + stamp.nanosec / 1e9,
                "callback_monotonic_ns": time.monotonic_ns(), "callback_wall_ns": time.time_ns(),
                "dds_source_ns": field("source_timestamp"), "dds_received_ns": field("received_timestamp"),
                "publication_sequence": field("publication_sequence_number")}

    def odom(self, message, info):
        row = self.receipt(message, info, "odom")
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        row["pose"] = [p.x, p.y, yaw]
        self.record(row)

    def clock(self, message, info=None):
        self.trial.startup.message("clock", message, info)
        value = message.clock.sec + message.clock.nanosec / 1e9
        self.trial.clock = value
        self.record({"channel": "clock", "stamp": value, "callback_monotonic_ns": time.monotonic_ns()})

    def bridge(self):
        while not self.done.is_set():
            begin = time.monotonic_ns()
            try:
                body = http(8765, "/get_robot_state", {}, timeout=2)
                end = time.monotonic_ns()
                self.record({"channel": "bridge", "stamp": body["sim_time"],
                             "pose": [body["x"], body["y"], body["yaw"]],
                             "request_monotonic_ns": begin, "callback_monotonic_ns": end,
                             "request_latency_s": (end - begin) / 1e9})
            except Exception as exc:
                self.errors.append(str(exc))
            self.done.wait(max(0, 0.05 - (time.monotonic_ns() - begin) / 1e9))

    def window(self, begin, end):
        with self.lock:
            return [r for r in self.rows if begin <= r["stamp"] <= end]

    def poses(self, begin, end):
        rows = self.window(begin, end)
        # Same-stamp repeated cached readings are retained raw but count once in
        # geometric metrics. Both sources read the same simulator ground truth.
        unique = {r["stamp"]: (r["stamp"], *r["pose"]) for r in rows}
        return [unique[t] for t in sorted(unique)]

    def summary(self, begin, end):
        rows = self.window(begin, end)
        out = {"http_errors": self.errors}
        for channel in ("odom", "bridge"):
            selected = [r for r in rows if r["channel"] == channel]
            times = sorted(set(r["stamp"] for r in selected))
            out[channel] = {"samples": len(selected), "unique_stamps": len(times),
                            "max_sim_gap_s": max((b - a for a, b in zip(times, times[1:])), default=None)}
            if channel == "odom":
                gaps = []
                for a, b in zip(selected, selected[1:]):
                    gaps.append({"from": a["stamp"], "to": b["stamp"], "sim_gap_s": b["stamp"] - a["stamp"],
                                 "dds_publish_gap_s": (b["dds_source_ns"] - a["dds_source_ns"]) / 1e9,
                                 "callback_gap_s": (b["callback_monotonic_ns"] - a["callback_monotonic_ns"]) / 1e9})
                out[channel]["largest_gaps"] = sorted(gaps, key=lambda r: r["sim_gap_s"], reverse=True)[:5]
                out[channel]["max_callback_after_publish_s"] = max(
                    ((r["callback_wall_ns"] - r["dds_source_ns"]) / 1e9 for r in selected
                     if r["dds_source_ns"] > 0), default=None)
                out[channel]["max_dds_transport_s"] = max(
                    ((r["dds_received_ns"] - r["dds_source_ns"]) / 1e9 for r in selected
                     if r["dds_received_ns"] > 0 and r["dds_source_ns"] > 0), default=None)
                out[channel]["max_callback_after_receive_s"] = max(
                    ((r["callback_wall_ns"] - r["dds_received_ns"]) / 1e9 for r in selected
                     if r["dds_received_ns"] > 0), default=None)
            else:
                out[channel]["max_request_latency_s"] = max((r["request_latency_s"] for r in selected), default=None)
        clocks = [r for r in self.clocks if begin <= r["stamp"] <= end]
        out["max_clock_sim_gap_s"] = max((b["stamp"] - a["stamp"] for a, b in zip(clocks, clocks[1:])), default=None)
        return out

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.done.set()
        self.http_thread.join(timeout=3)
        self.executor.shutdown(timeout_sec=3)
        self.ros_thread.join(timeout=3)
        self.node.destroy_node()
        self.stream.close()


class Trial:
    def __init__(self, out, control=False):
        self.out = out
        self.control = control
        self.processes = []
        self.files = []
        self.data = {"outcome": "INVALID", "invalid_reasons": [], "failure_reasons": [],
                     "observed_contact_episodes": 0, "contact_events": [], "event_errors": [],
                     "event_drops": 0, "ring_overwrites": 0, "event_polls": 0,
                     "robot_contact_readbacks": 0, "sequence_errors": [], "plans": 0, "recoveries": 0,
                     "local_marking_during_approach": None}
        self.data["commit"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
        self.data["machine"] = {"platform": os.uname().release, "cpu_count": os.cpu_count(),
                                "ros_distro": os.environ.get("ROS_DISTRO"),
                                "rmw": os.environ.get("RMW_IMPLEMENTATION")}
        self.stop = threading.Event()

    def launch(self, name, args):
        stream = (self.out / f"{name}.log").open("w")
        self.files.append(stream)
        process = subprocess.Popen(args, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        self.processes.append((name, process))

    def wait_log(self, name, text, timeout=80):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if text in (self.out / f"{name}.log").read_text(errors="replace"):
                return
            process = next(p for n, p in self.processes if n == name)
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited; see log")
            time.sleep(0.2)
        raise RuntimeError(f"{name} readiness timeout: {text}")

    def events(self):
        since = 0
        log_since = 0
        active = set()
        with (self.out / "events.jsonl").open("w") as stream:
            while not self.stop.is_set():
                try:
                    # Drain unfiltered supervisor events so sequence gaps are meaningful.
                    requested_at = time.monotonic_ns()
                    body = http(6789, f"/sim/events?since={since}&log_since={log_since}&limit=1000")
                    stream.write(json.dumps({"requested_since": since, "requested_monotonic_ns": requested_at,
                                             "received_monotonic_ns": time.monotonic_ns(), "response": body}) + "\n")
                    # EventBus.dropped counts ALL evictions, including already-read
                    # entries. Only eviction beyond our cursor means unread evidence loss.
                    self.data["event_drops"] += max(0, body["dropped_sup"] - since)
                    self.data["ring_overwrites"] = body["dropped_sup"]
                    sequences = [e["seq"] for e in body["events"] if e.get("source") == "sup"]
                    if sequences and sequences != list(range(since + 1, sequences[-1] + 1)):
                        self.data["sequence_errors"].append({"since": since, "sequences": sequences})
                    if body["next_since"] != (sequences[-1] if sequences else since):
                        self.data["sequence_errors"].append({"cursor_mismatch": body["next_since"]})
                    seen_since = since
                    since, log_since = body["next_since"], body["next_log_since"]
                    self.data["event_polls"] += 1
                    for event in body["events"]:
                        if event.get("source") != "sup" or event["type"] not in ("contact.began", "contact.ended"):
                            continue
                        a, b = event.get("a_def"), event.get("b_def")
                        if a in self.robot_ids or b in self.robot_ids:
                            self.data["robot_contact_readbacks"] += 1
                        if "NAV_OBSTACLE" not in (a, b):
                            continue
                        other = b if a == "NAV_OBSTACLE" else a
                        if other not in self.robot_ids:
                            if other not in self.scene_ids:
                                self.data["event_errors"].append({"unattributed_obstacle_contact": event})
                            continue
                        key = tuple(sorted((a, b)))
                        self.data["contact_events"].append(event)
                        if event["type"] == "contact.began":
                            if not active:
                                self.data["observed_contact_episodes"] += 1
                            active.add(key)
                        else:
                            # Only new, trial-attributed ends require a matching begin.
                            # Replayed sequences already invalidate continuity above.
                            if key not in active and event["seq"] > seen_since:
                                self.data["event_errors"].append({"unpaired_contact_ended": event})
                            active.discard(key)
                        seen_since = max(seen_since, event["seq"])
                except Exception as exc:
                    self.data["event_errors"].append(str(exc))
                self.stop.wait(0.25)

    def cleanup(self):
        self.stop.set()
        if hasattr(self, "event_thread"):
            self.event_thread.join(timeout=21)
        # Stop while the CommandNode and bridge still exist; terminate only our groups.
        if self.processes:
            try:
                self.data["final_zero"] = http(8765, "/set_velocity", {"linear": 0, "angular": 0, "wait": False})
                if not self.data["final_zero"].get("accepted"):
                    self.data["invalid_reasons"].append("final zero command not accepted")
            except Exception as exc:
                self.data["invalid_reasons"].append(f"zero command failed: {exc}")
        for name, process in reversed(self.processes):
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=15)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            self.data.setdefault("process_exit", {})[name] = process.returncode
        if hasattr(self, "recorder") and not self.recorder.closed:
            self.recorder.close()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        if hasattr(self, "startup"):
            nav_log = self.out / "nav.log"
            if nav_log.exists():
                self.startup.nav_log(nav_log.read_text(errors="replace"))
            self.startup.close()
        for stream in self.files:
            stream.close()

    def validate_collectors(self, node, publisher, spin_until):
        """Intentional sustained bumper/body contact, then separation; no Nav2."""
        from geometry_msgs.msg import Twist

        begin = http(8765, "/get_robot_state", {})["sim_time"]
        self.data["control_before_contacts"] = http(6789, "/sim/contacts")
        self.data["control_capabilities"] = http(6789, "/capabilities")
        pause_begin = time.monotonic_ns()
        # Exercise the old collector's scheduling failure mode while stationary.
        # The independent receiver and cached-pose sampler continue running.
        time.sleep(0.6)
        self.data["intentional_observer_pause"] = [pause_begin, time.monotonic_ns()]

        def event_seen(kind):
            return any(e["type"] == kind for e in self.data["contact_events"])

        def drive(speed, predicate, timeout, stop=True):
            deadline, next_command = time.monotonic() + timeout, 0
            while not predicate() and time.monotonic() < deadline:
                if time.monotonic() >= next_command:
                    command = Twist()
                    command.linear.x = speed
                    publisher.publish(command)
                    next_command = time.monotonic() + 0.1
                # Retain scan/TF processing while the independent pose receiver runs.
                import rclpy
                rclpy.spin_once(node, timeout_sec=0.02)
            if stop:
                publisher.publish(Twist())
            return predicate()

        if not drive(0.2, lambda: event_seen("contact.began"), 45, stop=False):
            raise RuntimeError("known obstacle contact did not produce attributed contact.began")
        self.data["control_contact_state"] = http(8765, "/get_robot_state", {})
        self.data["control_touching_contacts"] = http(6789, "/sim/contacts")
        if not drive(-0.2, lambda: event_seen("contact.ended"), 15):
            raise RuntimeError("separation did not produce attributed contact.ended")
        # Move clear after the first ended edge, which may be transient contact chatter.
        touching_x = self.data["control_contact_state"]["x"]
        if not drive(-0.2, lambda: self.recorder.rows and self.recorder.rows[-1]["pose"][0] < touching_x - 0.25, 10):
            raise RuntimeError("positive control could not establish physical separation")
        spin_until(lambda: False, 0.5)
        end_state = http(8765, "/get_robot_state", {})
        self.data["control_separated_state"] = end_state
        self.data["control_after_contacts"] = http(6789, "/sim/contacts")
        self.data["timing"] = self.recorder.summary(begin, end_state["sim_time"])
        poses = self.recorder.poses(begin, end_state["sim_time"])
        self.data["max_pose_gap_sim_s"] = max(b[0] - a[0] for a, b in zip(poses, poses[1:]))
        pairs_began = {tuple(sorted((e["a_def"], e["b_def"]))) for e in self.data["contact_events"]
                       if e["type"] == "contact.began"}
        pairs_ended = {tuple(sorted((e["a_def"], e["b_def"]))) for e in self.data["contact_events"]
                       if e["type"] == "contact.ended"}
        physical = [c for c in self.data["control_touching_contacts"].get("contacts", [])
                    if "NAV_OBSTACLE" in (c.get("a_def"), c.get("b_def"))
                    and (c.get("a_def") in self.robot_ids or c.get("b_def") in self.robot_ids)]
        self.data["collector_control"] = {"matched_began_ended_pairs": sorted(pairs_began & pairs_ended),
                                           "physical_contact_readback": physical,
                                           "native_contact_event_points": [e["point"] for e in self.data["contact_events"]
                                                                           if e["type"] == "contact.began"],
                                           "separation_m": touching_x - end_state["x"]}
        print(json.dumps({"collector_control": self.data["collector_control"], "timing": self.data["timing"]}), flush=True)

    def retry_scans(self, pending_scans, project_scan, raw, final=False):
        """Retry the existing batch; finalization cannot leave evidence pending."""
        def stamp(value):
            return value.sec + value.nanosec / 1e9

        waiting = []
        for message, begin, error, measuring in pending_scans:
            delay = time.monotonic() - begin
            try:
                project_scan(message)
                self.tf_resolved += 1
                raw.write(json.dumps({"scan_tf_resolved": {"stamp": stamp(message.header.stamp),
                                     "wait_wall_s": delay, "initial_error": error}}) + "\n")
            except Exception as exc:
                if delay < 2 and not final:
                    waiting.append((message, begin, error, measuring))
                else:
                    expired = self.tf_expired if measuring else self.tf_startup_expired
                    evidence = {"stamp": stamp(message.header.stamp), "error": str(exc),
                                "wait_wall_s": delay, "callback_monotonic_ns": time.monotonic_ns()}
                    expired.append(evidence)
                    raw.write(json.dumps({"scan_tf_expired" if measuring else "scan_tf_startup_expired": evidence}) + "\n")
        pending_scans[:] = waiting

    def drain_scan_retries(self, pending_scans, project_scan, raw, spin_until):
        """Settle only the frozen measurement batch within its original deadlines."""
        finishing = [item for item in pending_scans if item[3]]
        pending_scans[:] = [item for item in pending_scans if not item[3]]
        if not finishing:
            return
        deadline = max(begin + 2 for _, begin, _, _ in finishing)

        def settled():
            self.retry_scans(finishing, project_scan, raw)
            return not finishing

        spin_until(settled, max(0, deadline - time.monotonic()))
        # Even an interrupted drain must not silently discard unresolved evidence.
        self.retry_scans(finishing, project_scan, raw, final=True)

    def run(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
        from rclpy.action import ActionClient
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformListener

        self.launch("harness", ["xvfb-run", "-a", sys.executable, "-m", "omnisim", "harness", "--engine-mode", "realtime"])
        self.wait_log("harness", "listening on")
        loaded = http(6789, "/world/load", {"path": WORLD, "light": False,
                                          "tracking": {"contacts": True, "joint_limits": False, "grips": False}}, 100)
        self.data["load"] = loaded
        if not loaded.get("ok") or loaded.get("hot_reloaded"):
            raise RuntimeError("fresh world load failed")
        # /world/load can acknowledge a still-loading engine after 30 seconds.
        # Scene RPCs require the supervisor, not merely an accepted load request.
        readiness = self.data["load_readiness"] = []
        deadline = time.monotonic() + 100
        state = loaded
        while not state.get("supervisor_connected") or state.get("load_state") != "complete":
            if time.monotonic() >= deadline:
                raise RuntimeError("fresh world supervisor readiness timeout")
            time.sleep(0.25)
            state = http(6789, "/sim/state", timeout=5)
            readiness.append({"received_monotonic_ns": time.monotonic_ns(), "state": state})
            if state.get("load_state") == "failed" or not state.get("running", True):
                raise RuntimeError("fresh world engine failed during loading")
        scene = http(6789, "/scene/tree")
        self.data["scene"] = scene
        nodes = scene["nodes"]
        self.scene_ids = {entry.get("def") or f"#{entry['id']}" for entry in nodes}
        # Imported children may have no DEF: observe.py uses '#<id>' then.
        self.robot_ids = {"HUSKY"}
        for entry in nodes:
            if entry.get("def") == "HUSKY" or entry.get("parent_def") in self.robot_ids:
                self.robot_ids.update((entry.get("def") or f"#{entry['id']}", f"#{entry['id']}"))
        self.data["robot_contact_ids"] = sorted(self.robot_ids)
        self.data["obstacle_node"] = http(6789, "/scene/node/NAV_OBSTACLE")
        obstacle = self.data["obstacle_node"]
        fields = obstacle.get("fields", {})
        if (obstacle.get("position") != [2.0, 0.0, 0.6]
                or fields.get("boundingObject", {}).get("summary", {}).get("size") != [0.6, 1.0, 1.2]
                or fields.get("physics", {}).get("present") is not False):
            raise RuntimeError("authored static obstacle geometry/pose not confirmed")
        self.data["robot_start_node"] = http(6789, "/scene/node/HUSKY")
        self.event_thread = threading.Thread(target=self.events, daemon=True)
        self.event_thread.start()
        self.launch("sidecar", ["ros2", "launch", "omnisim_ros2", "omnisim_bringup.launch.py",
                               "use_sim_time:=true", "tf_root_only:=true", "lidar_layer:=0",
                               "clock_rate_hz:=20.0", "state_rate_hz:=5.0", "odom_rate_hz:=10.0", "sensor_rate_hz:=10.0"])
        self.wait_log("sidecar", "publishing odom")
        self.launch("static", ["ros2", "run", "tf2_ros", "static_transform_publisher", "--frame-id", "map",
                              "--child-frame-id", "odom", "--ros-args", "-p", "use_sim_time:=true"])
        self.wait_log("static", "from 'map' to 'odom'")
        rclpy.init()
        self.clock = None
        self.startup = StartupTiming(self.out)
        self.recorder = PoseRecorder(self)
        if not self.control:
            self.startup.boundary("nav_launch")
            self.launch("nav", ["ros2", "launch", "omnisim_ros2_nav2", "omnisim_nav2_lean.launch.py",
                               "use_sim_time:=true", f"params_file:={PARAMS}"])
            self.wait_log("nav", "Managed nodes are active")
            self.startup.boundary("nav_active_observed")
        node = rclpy.create_node("husky_obstacle_measurement")
        buffer = Buffer()
        self.listener = TransformListener(buffer, node)
        self.poses = []
        self.navigating = False
        self.scan_hits = None
        self.global_marking = None
        self.scan_ages = []
        self.tf_misses = 0
        self.tf_resolved = 0
        self.tf_expired = []
        self.tf_startup_expired = []
        self.measurement_started = False
        self.measurement_finished = False
        pending_scans = []
        raw = (self.out / "samples.jsonl").open("w")
        self.files.append(raw)

        def stamp(value):
            return value.sec + value.nanosec / 1e9

        def odom(message):
            if not self.navigating:
                return
            p, q = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            row = (stamp(message.header.stamp), p.x, p.y, yaw)
            if not self.poses or row[0] > self.poses[-1][0]:
                self.poses.append(row)
                raw.write(json.dumps({"odom": row}) + "\n")

        def project_scan(message):
            transform = buffer.lookup_transform("map", message.header.frame_id, Time.from_msg(message.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            hits = []
            for index, distance in enumerate(message.ranges):
                if not math.isfinite(distance) or not message.range_min <= distance <= message.range_max:
                    continue
                angle = yaw + message.angle_min + index * message.angle_increment
                x, y = t.x + distance * math.cos(angle), t.y + distance * math.sin(angle)
                if abs(x - 1.7) <= 0.08 and abs(y) <= 0.55:
                    hits.append((x, y, distance))
            age = None if self.clock is None else self.clock - stamp(message.header.stamp)
            if age is not None:
                self.scan_ages.append(age)
            if hits and not self.navigating:
                self.scan_hits = {"frame": message.header.frame_id, "beams": len(message.ranges),
                                  "front_face_hits": len(hits), "center_hit": min(hits, key=lambda p: abs(p[1])),
                                  "stamp": stamp(message.header.stamp), "clock_age_s": age,
                                  "laser_xyz": [t.x, t.y, t.z]}

        def scan(message):
            if self.measurement_finished:
                return
            try:
                project_scan(message)
            except Exception as exc:
                self.tf_misses += 1
                pending_scans.append((message, time.monotonic(), str(exc), self.measurement_started))
                raw.write(json.dumps({"scan_tf_immediate_failure": {"stamp": stamp(message.header.stamp),
                                     "callback_monotonic_ns": time.monotonic_ns(), "error": str(exc)}}) + "\n")

        def retry_scans():
            self.retry_scans(pending_scans, project_scan, raw)

        # Match the monotonic retry deadline; an inactive ROS clock follows
        # system realtime and stalls its timer on an OS clock rollback.
        self.scan_retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.scan_retry_timer = node.create_timer(0.02, retry_scans, clock=self.scan_retry_clock)
        def costmap(message, local):
            if message.header.frame_id != ("odom" if local else "map"):
                self.data["invalid_reasons"].append("unexpected costmap frame")
                return
            info = message.info
            q = info.origin.orientation
            if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
                self.data["invalid_reasons"].append("rotated costmap origin unsupported")
                return
            ox, oy, resolution = info.origin.position.x, info.origin.position.y, info.resolution
            marked = []
            for row in range(max(0, int((-0.55 - oy) / resolution)), min(info.height, math.ceil((0.55 - oy) / resolution))):
                for col in range(max(0, int((1.625 - ox) / resolution)), min(info.width, math.ceil((2.375 - ox) / resolution))):
                    if message.data[row * info.width + col] == 100:
                        marked.append((ox + (col + 0.5) * resolution, oy + (row + 0.5) * resolution))
            evidence = {"frame": message.header.frame_id, "stamp": stamp(message.header.stamp),
                        "bounds": [ox, oy, ox + info.width * resolution, oy + info.height * resolution],
                        "lethal_cells_at_obstacle": len(marked), "example_cells": marked[:8]}
            raw.write(json.dumps({"local_costmap" if local else "global_costmap": evidence}) + "\n")
            if marked:
                if local and self.navigating and self.poses and self.poses[-1][1] <= 2.0:
                    previous = self.data["local_marking_during_approach"]
                    if previous is None or len(marked) > previous["lethal_cells_at_obstacle"]:
                        self.data["local_marking_during_approach"] = evidence
                elif not local and not self.navigating:
                    self.global_marking = evidence

        def plan(message):
            if self.navigating:
                self.data["plans"] += 1
                points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
                raw.write(json.dumps({"plan": points}) + "\n")

        def feedback(message):
            self.data["recoveries"] = max(self.data["recoveries"], message.feedback.number_of_recoveries)

        self.subscriptions = [
            node.create_subscription(Odometry, "/odom", odom, 100),
            node.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data),
            node.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: costmap(m, False), 10),
            node.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: costmap(m, True), 10),
            node.create_subscription(NavPath, "/plan", plan, 10),
        ]
        client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        zero = node.create_publisher(Twist, "/cmd_vel", 10)

        def spin_until(predicate, timeout):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            return predicate()

        try:
            if not spin_until(lambda: self.scan_hits and (self.control or self.global_marking)
                              and self.clock is not None
                              and any(r["channel"] == "odom" for r in self.recorder.rows), 30):
                raise RuntimeError("preflight scan/global obstacle marking missing")
            self.data["preflight"] = {"scan": self.scan_hits, "global_costmap": self.global_marking}
            # World/load can return while the initial z=0.2 robot is still falling.
            # Check the readback positive control only after settled sensor preflight.
            self.data["contact_scope"] = http(6789, "/sim/contacts")
            self.data["settled_start_node"] = http(6789, "/scene/node/HUSKY")
            self.data["navigation_tf"] = {}
            for parent, child in (("map", "odom"), ("odom", "base_link"), ("base_link", "base_laser")):
                transform = buffer.lookup_transform(parent, child, Time())
                t, q = transform.transform.translation, transform.transform.rotation
                self.data["navigation_tf"][f"{parent}->{child}"] = {
                    "xyz": [t.x, t.y, t.z], "quaternion": [q.x, q.y, q.z, q.w]}
            identity = self.data["navigation_tf"]["map->odom"]
            if sum(abs(v) for v in identity["xyz"]) + sum(abs(v) for v in identity["quaternion"][:3]) > 1e-6:
                raise RuntimeError("map->odom is not identity")
            nodes = node.get_node_names()
            monitor = "collision_monitor" in nodes
            self.data["collision_monitor"] = {"active": monitor, "configured_radius_m": 0.35,
                                               "interventions": None, "reason": "not launched by lean stack"}
            if monitor:
                raise RuntimeError("unexpected collision monitor; trial would change baseline routing")
            self.measurement_started = True
            if self.control:
                self.validate_collectors(node, zero, spin_until)
                self.measurement_started = False
                self.measurement_finished = True
                self.drain_scan_retries(pending_scans, project_scan, raw, spin_until)
                self.data.update(scan_tf_misses=self.tf_misses, scan_tf_resolved=self.tf_resolved,
                                 scan_tf_expired=self.tf_expired)
                return
            if not client.wait_for_server(timeout_sec=10):
                raise RuntimeError("NavigateToPose unavailable")
            initial = http(8765, "/get_robot_state", {})
            self.data["initial_pose"] = initial
            if math.hypot(initial["x"], initial["y"]) > 0.02 or abs(initial["yaw"]) > 0.02:
                raise RuntimeError("robot not at fresh origin")
            self.poses.append((initial["sim_time"], initial["x"], initial["y"], initial["yaw"]))
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.pose.position.x = 4.0
            goal.pose.pose.orientation.w = 1.0
            self.data["goal"] = {"x": 4, "y": 0, "yaw": 0, "frame": "map"}
            self.navigating = True
            start, sim_start = time.monotonic(), self.clock
            self.startup.boundary("goal_send", clock_sim_s=self.clock)
            sent = client.send_goal_async(goal, feedback_callback=feedback)
            if not spin_until(sent.done, 10) or not sent.result().accepted:
                raise RuntimeError("goal not accepted")
            self.startup.boundary("goal_accepted", clock_sim_s=self.clock)
            handle = sent.result()
            result = handle.get_result_async()
            completed = spin_until(lambda: result.done() or self.clock - sim_start > 60, 120)
            self.data["wall_navigation_s"] = time.monotonic() - start
            self.data["sim_navigation_s"] = self.clock - sim_start
            if not completed or not result.done():
                cancellation = handle.cancel_goal_async()
                spin_until(cancellation.done, 5)
                self.data["action_status"] = "TIMEOUT"
            else:
                response = result.result()
                self.data.update(action_status=response.status, action_error=response.result.error_code,
                                 action_message=response.result.error_msg)
            self.measurement_started = False
            self.measurement_finished = True
            self.startup.boundary("action_result", status=self.data["action_status"], clock_sim_s=self.clock)
            zero.publish(Twist())
            self.data["final_pose"] = http(8765, "/get_robot_state", {})
            final = self.data["final_pose"]
            self.data["timing"] = self.recorder.summary(initial["sim_time"], final["sim_time"])
            self.poses = self.recorder.poses(initial["sim_time"], final["sim_time"])
            self.poses.extend([(initial["sim_time"], initial["x"], initial["y"], initial["yaw"]),
                               (final["sim_time"], final["x"], final["y"], final["yaw"])])
            self.poses = sorted({p[0]: p for p in self.poses}.values())
            raw.write(json.dumps({"metric_pose_samples": self.poses}) + "\n")
            self.data["robot_end_node"] = http(6789, "/scene/node/HUSKY")
            self.data["position_error_m"] = math.hypot(final["x"] - 4, final["y"])
            self.data["yaw_error_rad"] = abs(math.atan2(math.sin(final["yaw"]), math.cos(final["yaw"])))
            self.data["traveled_distance_m"] = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                                                    for a, b in zip(self.poses, self.poses[1:]))
            self.data["max_pose_gap_sim_s"] = max(b[0] - a[0] for a, b in zip(self.poses, self.poses[1:]))
            distances = [clearance(hull(x, y, yaw)) for _, x, y, yaw in self.poses]
            self.data["min_sampled_hull_clearance_m"] = min(d[0] for d in distances)
            self.data["sampled_overlap_count"] = sum(d[1] for d in distances)
            crossing = [p for p in self.poses if 1.7 <= p[1] <= 2.3]
            self.data["detour"] = {"crossing_samples": len(crossing),
                                   "minimum_abs_y_at_crossing": min((abs(p[2]) for p in crossing), default=None),
                                   "max_abs_y": max(abs(p[2]) for p in self.poses)}
            self.data["pose_samples"] = len(self.poses)
            self.data["scan_tf_misses"] = self.tf_misses
            # Flush only the observer's pending TF work; navigation time is already frozen.
            self.drain_scan_retries(pending_scans, project_scan, raw, spin_until)
            self.data["scan_tf_resolved"] = self.tf_resolved
            self.data["scan_tf_expired"] = self.tf_expired
            self.data["max_scan_age_sim_s"] = max(self.scan_ages, default=None)
            print(json.dumps({k: v for k, v in self.data.items() if k in (
                "preflight", "action_status", "position_error_m", "min_sampled_hull_clearance_m", "detour")}), flush=True)
        finally:
            self.navigating = False
            zero.publish(Twist())
            self.recorder.close()
            self.data["scan_tf_startup_expired"] = self.tf_startup_expired
            if self.recorder.errors:
                self.data["invalid_reasons"].append("pose receiver/bridge recorder errors; see timing diagnostics")
            node.destroy_node()
            rclpy.shutdown()

    def verdict(self):
        data = self.data
        for name in ("sidecar", "nav"):
            path = self.out / f"{name}.log"
            text = path.read_text(errors="replace") if path.exists() else ""
            if name == "sidecar":
                data["watchdog_stops"] = text.count("no cmd_vel for 0.5 s; stopping the robot")
            else:
                data["stale_scans"] = sum("Message Filter dropping message" in line and
                                          "earlier than all the data" in line for line in text.splitlines())
        invalid = data["invalid_reasons"]
        if data["event_errors"] or data["event_drops"] or data["sequence_errors"] or not data["event_polls"]:
            invalid.append("contact event evidence unreliable")
        if not data.get("contact_scope", {}).get("contacts") and not data.get("robot_contact_readbacks"):
            invalid.append("contact readback has no positive control (expected resting wheel/floor contacts)")
        if data.get("scan_tf_expired"):
            invalid.append("scan transform unavailable after bounded observer retry")
        if self.control:
            control = data.get("collector_control", {})
            # ContactTracker emits these points only from paired native readback.
            # The separate /sim/contacts snapshot may miss a short contact after
            # the event was captured; retain that completeness diagnostic too.
            if not control.get("matched_began_ended_pairs") or not control.get("native_contact_event_points"):
                invalid.append("physical-contact positive control incomplete")
            if data.get("max_pose_gap_sim_s", math.inf) > 0.2:
                invalid.append("positive-control pose completeness exceeds 0.20 s gate")
            data["navigation_outcome"] = "NOT_RUN"
            data["measurement_outcome"] = data["outcome"] = "INVALID" if invalid else "PASS"
            (self.out / "result.json").write_text(json.dumps(data, indent=2) + "\n")
            print(json.dumps({"outcome": data["outcome"], "invalid": invalid, "artifacts": str(self.out)}), flush=True)
            return
        for key in ("robot_start_node", "robot_end_node"):
            matrix = data.get(key, {}).get("orientation", [])
            if len(matrix) != 9 or matrix[8] is None or math.acos(max(-1, min(1, matrix[8]))) > 0.03:
                invalid.append(f"{key}: flat-ground hull assumption not verified")
        if data.get("max_pose_gap_sim_s", math.inf) > 0.2:
            invalid.append("missing pose data or sim-time sample gap exceeds 0.20 s")
        if not data["local_marking_during_approach"]:
            invalid.append("local obstacle marking during approach missing")
        if not data.get("detour", {}).get("crossing_samples"):
            invalid.append("no sampled crossing of obstacle x interval")
        failed = data["failure_reasons"]
        for condition, reason in [
            (data.get("action_status") != 4 or data.get("action_error", 0) != 0, "goal did not succeed"),
            (data.get("position_error_m", math.inf) > 0.25, "position error > 0.25 m"),
            (data.get("yaw_error_rad", math.inf) > 0.25, "yaw error > 0.25 rad"),
            (data["observed_contact_episodes"] > 0, "observed obstacle contact"),
            (data.get("sampled_overlap_count", 0) > 0, "sampled collision-envelope overlap"),
            (data.get("min_sampled_hull_clearance_m", 0) < 0.1, "sampled hull clearance < 0.10 m"),
            (data.get("traveled_distance_m", math.inf) > 8, "traveled distance > 8 m"),
            (data.get("sim_navigation_s", math.inf) > 60 or data.get("wall_navigation_s", math.inf) > 120,
             "navigation timeout"),
            (data["watchdog_stops"] or data["stale_scans"], "watchdog stop or stale scan"),
        ]:
            if condition:
                failed.append(reason)
        data["navigation_outcome"] = "FAIL" if failed else "PASS"
        data["measurement_outcome"] = "INVALID" if invalid else "PASS"
        data["outcome"] = "INVALID" if invalid else "FAIL" if failed else "PASS"
        (self.out / "result.json").write_text(json.dumps(data, indent=2) + "\n")
        print(json.dumps({"outcome": data["outcome"], "invalid": invalid, "failed": failed,
                          "artifacts": str(self.out)}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-geometry", action="store_true")
    parser.add_argument("--validate-collectors", action="store_true", help="intentional contact/separation; no Nav2")
    parser.add_argument("--contact-control", type=Path, help="PASS result.json from the separate collector control")
    parser.add_argument("--output", type=Path, default=Path("/tmp/husky-nav2-obstacle"))
    args = parser.parse_args()
    if args.check_geometry:
        check_geometry()
        return 0
    if os.environ.get("ROS_DISTRO") != "jazzy" or os.environ.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
        parser.error("source Jazzy/workspace and set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp")
    proof = None
    if not args.validate_collectors:
        if not args.contact_control:
            parser.error("run --validate-collectors first, then pass its result.json with --contact-control")
        proof = json.loads(args.contact_control.read_text())
        if proof.get("outcome") != "PASS" or not proof.get("collector_control", {}).get("matched_began_ended_pairs"):
            parser.error("contact collector positive control did not PASS")
    for port in (6789, 6790, 8765):
        with socket.socket() as connection:
            connection.settimeout(0.2)
            if connection.connect_ex(("127.0.0.1", port)) == 0:
                parser.error(f"port {port} already owned; refusing to reuse another session")
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(NO_PROXY="127.0.0.1,localhost", no_proxy="127.0.0.1,localhost",
                      OMNISIM_URDF_USE_SENSORS="1", OMNISIM_NEWTON_MODEL_DEVICE="cpu", OMNISIM_NO_WINDOW="1",
                      OMNISIM_LOG_PATH=str(args.output / "engine.log"), PYTHONUNBUFFERED="1")
    trial = Trial(args.output, control=args.validate_collectors)
    if proof:
        trial.data["contact_control_validation"] = {
            "artifact": str(args.contact_control.resolve()), "outcome": proof["outcome"],
            "matched_pairs": proof["collector_control"]["matched_began_ended_pairs"],
            "unread_event_loss": proof["event_drops"], "sequence_errors": proof["sequence_errors"]}
    try:
        trial.run()
    except Exception as exc:
        trial.data["invalid_reasons"].append(str(exc))
        print(f"INVALID: {exc}", flush=True)
    finally:
        trial.cleanup()
        trial.verdict()
    return 0 if trial.data["outcome"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
