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

"""Evidence controller for the Agent Build extreme-terrain film.

The experiment has two honest modes selected with ``OMNISIM_TERRAIN_MODE``:

``baseline``
    Equal wheel velocity and no steering feedback.  It establishes that a
    fixed-throttle command cannot solve the authored chicane.

``adaptive``
    Follows the frozen course centreline, reduces throttle as pitch/roll grow,
    and performs a short wheel-driven recovery if commanded motion stops.

The controller never writes the robot pose.  It records the measured Newton
run to ``OMNISIM_TERRAIN_RESULT`` and can export deterministic wgpu main-view
frames for editorial coverage.  Camera modes are locked shots or direct cuts;
there is no high-speed chase camera.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile
import time

from omnisim import Supervisor


WHEEL_NAMES = (
    "front_left_wheel_motor",
    "rear_left_wheel_motor",
    "front_right_wheel_motor",
    "rear_right_wheel_motor",
)

# The authored centreline.  This is a terrain-control challenge, not an unseen
# mapping claim: the controller knows these waypoints and must physically drive
# between them under Newton contact.
WAYPOINTS = (
    (-12.2, 0.0),
    (-9.6, 0.0),
    (-7.5, 0.95),
    (-5.5, 0.95),
    (-2.0, -0.95),
    (1.0, -0.95),
    (4.0, 0.95),
    (6.8, 0.95),
    (8.2, 0.0),
    (10.2, 0.0),
    (12.9, 0.0),
)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def find_link_by_name(node, wanted: str):
    """Find a physical URDF link beneath the imported wrapper."""
    if node is None:
        return None
    try:
        name_field = node.getField("name")
        if name_field is not None and name_field.getSFString() == wanted:
            return node
    except Exception:
        pass
    try:
        children = node.getField("children")
        count = children.getCount() if children is not None else 0
    except Exception:
        count = 0
    for index in range(count):
        child = children.getMFNode(index)
        match = find_link_by_name(child, wanted)
        if match is not None:
            return match
    try:
        endpoint = node.getField("endPoint")
        endpoint_node = endpoint.getSFNode() if endpoint is not None else None
    except Exception:
        endpoint_node = None
    if endpoint_node is not None:
        return find_link_by_name(endpoint_node, wanted)
    return None


def axis_angle_to_target(position: tuple[float, float, float],
                         target: tuple[float, float, float]) -> list[float]:
    """Return a horizon-stable Viewpoint orientation aimed at ``target``."""
    px, py, pz = position
    tx, ty, tz = target
    fx, fy, fz = tx - px, ty - py, tz - pz
    length = math.sqrt(fx * fx + fy * fy + fz * fz)
    if length < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    fx, fy, fz = fx / length, fy / length, fz / length
    ux, uy, uz = 0.0, 0.0, 1.0
    dot = fx * ux + fy * uy + fz * uz
    ux, uy, uz = ux - dot * fx, uy - dot * fy, uz - dot * fz
    up_length = math.sqrt(ux * ux + uy * uy + uz * uz)
    if up_length < 1e-9:
        ux, uy, uz, up_length = 0.0, 1.0, 0.0, 1.0
    ux, uy, uz = ux / up_length, uy / up_length, uz / up_length
    yx, yy, yz = uy * fz - uz * fy, uz * fx - ux * fz, ux * fy - uy * fx
    matrix = ((fx, yx, ux), (fy, yy, uy), (fz, yz, uz))
    angle = math.acos(clamp((matrix[0][0] + matrix[1][1] + matrix[2][2] - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return [0.0, 0.0, 1.0, 0.0]
    denominator = 2.0 * math.sin(angle)
    return [
        (matrix[2][1] - matrix[1][2]) / denominator,
        (matrix[0][2] - matrix[2][0]) / denominator,
        (matrix[1][0] - matrix[0][1]) / denominator,
        angle,
    ]


class CameraDirector:
    """Locked, motivated coverage of one continuous physical run."""

    MODES = {"wide", "top", "story", "start", "gate1", "gate2", "gate3", "causeway"}

    def __init__(self, supervisor: Supervisor, enabled: bool, mode: str):
        self.enabled = enabled
        self.mode = mode
        self.viewpoint = None
        self.story_zone = ""
        if not enabled:
            return
        root = supervisor.getRoot()
        children = root.getField("children") if root is not None else None
        for index in range(children.getCount() if children is not None else 0):
            node = children.getMFNode(index)
            if node is not None and node.getTypeName() == "Viewpoint":
                self.viewpoint = node
                break

    def _apply(self, position: list[float], target: list[float]) -> None:
        if self.viewpoint is None:
            return
        self.viewpoint.getField("position").setSFVec3f(position)
        self.viewpoint.getField("orientation").setSFRotation(
            axis_angle_to_target(tuple(position), tuple(target))
        )

    def update(self, sim_time: float, x: float, phase: str) -> None:
        if not self.enabled or self.viewpoint is None or sim_time < 3.0:
            return
        mode = self.mode
        if mode == "story":
            if phase.startswith("finished"):
                mode = "causeway"
            elif x < -8.2:
                mode = "start"
            elif x < -4.0:
                mode = "gate1"
            elif x < 2.3:
                mode = "gate2"
            elif x < 7.4:
                mode = "gate3"
            else:
                mode = "causeway"
            if mode != self.story_zone:
                self.story_zone = mode
                print(f"[husky_extreme_terrain] CAMERA_CUT {mode}", flush=True)

        if mode == "top":
            self._apply([0.0, 0.0, 35.0], [0.0, 0.0, 0.0])
        elif mode == "wide":
            self._apply([20.0, -23.0, 20.0], [0.0, 0.0, 0.15])
        elif mode == "start":
            self._apply([-10.4, -6.6, 3.5], [-10.4, 0.0, 0.32])
        elif mode == "gate1":
            self._apply([-7.8, -7.4, 4.2], [-6.2, 0.0, 0.30])
        elif mode == "gate2":
            self._apply([-2.0, -8.6, 5.0], [-1.0, 0.0, 0.30])
        elif mode == "gate3":
            self._apply([3.8, -8.2, 4.6], [4.8, 0.0, 0.30])
        else:
            self._apply([10.6, -6.8, 3.8], [10.6, 0.0, 0.32])


def write_result(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"[husky_extreme_terrain] RESULT {json.dumps(payload, separators=(',', ':'))}", flush=True)


def main() -> int:
    mode = os.environ.get("OMNISIM_TERRAIN_MODE", "adaptive").strip().lower()
    if mode not in {"baseline", "adaptive"}:
        print(f"[husky_extreme_terrain] invalid mode={mode!r}", flush=True)
        return 2
    default_result = Path(tempfile.gettempdir()) / f"husky_extreme_terrain_{mode}_{os.getpid()}.json"
    result_path = Path(os.environ.get("OMNISIM_TERRAIN_RESULT", str(default_result))).resolve()
    auto_quit = os.environ.get("OMNISIM_TERRAIN_AUTO_QUIT", "1") != "0"
    direct_camera = os.environ.get("OMNISIM_TERRAIN_DIRECT_CAMERA", "0") == "1"
    camera_mode = os.environ.get("OMNISIM_TERRAIN_CAMERA_MODE", "story").strip().lower()
    if camera_mode not in CameraDirector.MODES:
        print(f"[husky_extreme_terrain] invalid camera mode={camera_mode!r}", flush=True)
        return 2
    frame_value = os.environ.get("OMNISIM_TERRAIN_FRAME_DIR", "").strip()
    frame_dir = Path(frame_value).resolve() if frame_value else None
    frame_acceleration = max(1, int(os.environ.get("OMNISIM_TERRAIN_FRAME_ACCELERATION", "4")))
    frame_start_s = max(0.0, float(os.environ.get("OMNISIM_TERRAIN_FRAME_START_S", "4")))
    start_delay_s = max(1.0, float(os.environ.get("OMNISIM_TERRAIN_START_DELAY_S", "4")))
    finish_hold_s = max(2.0, float(os.environ.get("OMNISIM_TERRAIN_FINISH_HOLD_S", "4")))
    timeout_s = max(30.0, float(os.environ.get("OMNISIM_TERRAIN_TIMEOUT_S", "180")))

    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0
    self_node = robot.getSelf()
    if self_node is None:
        print("[husky_extreme_terrain] getSelf() failed", flush=True)
        return 1

    motors = []
    for name in WHEEL_NAMES:
        motor = robot.getDevice(name)
        if motor is None:
            print(f"[husky_extreme_terrain] missing motor={name}", flush=True)
            return 1
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
        motors.append(motor)
    left_motors, right_motors = motors[:2], motors[2:]

    # URDFRobot is a non-physical import wrapper.  Its translation follows the
    # robot, but its orientation can remain the authored zero rotation.  Drive
    # and measure from the physical base link so yaw feedback is real.
    robot.step(dt_ms)
    robot.step(dt_ms)
    pose_node = (
        find_link_by_name(self_node, "base_link")
        or find_link_by_name(self_node, "base_footprint")
        or self_node
    )

    camera = CameraDirector(robot, direct_camera, camera_mode)
    if frame_dir is not None:
        frame_dir.mkdir(parents=True, exist_ok=True)

    sim_started = float(robot.getTime())
    wall_started = time.time()
    previous_position = pose_node.getPosition()
    previous_time = sim_started
    waypoint_index = 0
    distance_m = 0.0
    max_pitch_deg = 0.0
    max_roll_deg = 0.0
    max_z_m = float(previous_position[2])
    recoveries = 0
    recovery_until = -1.0
    stall_started: float | None = None
    finish_payload: dict | None = None
    finish_time = 0.0
    next_sample_time = sim_started
    next_frame_time = frame_start_s
    frame_index = 0
    current_left = 0.0
    current_right = 0.0
    telemetry: list[dict] = []
    best_x = float(previous_position[0])
    last_forward_progress_time = sim_started

    print(
        f"[husky_extreme_terrain] ready mode={mode} waypoints={len(WAYPOINTS)} "
        f"camera={camera_mode} frame_acceleration={frame_acceleration}",
        flush=True,
    )

    while robot.step(dt_ms) != -1:
        sim_time = float(robot.getTime())
        position = pose_node.getPosition()
        x, y, z = (float(position[0]), float(position[1]), float(position[2]))
        rotation = pose_node.getOrientation()
        yaw = math.atan2(rotation[3], rotation[0])
        pitch = math.asin(clamp(-rotation[6], -1.0, 1.0))
        roll = math.atan2(rotation[7], rotation[8])
        pitch_deg = math.degrees(pitch)
        roll_deg = math.degrees(roll)
        sample_dt = max(sim_time - previous_time, 1e-6)
        travelled = math.hypot(x - float(previous_position[0]), y - float(previous_position[1]))
        speed = travelled / sample_dt
        distance_m += travelled
        previous_position = list(position)
        previous_time = sim_time
        max_pitch_deg = max(max_pitch_deg, abs(pitch_deg))
        max_roll_deg = max(max_roll_deg, abs(roll_deg))
        max_z_m = max(max_z_m, z)
        if x > best_x + 0.05:
            best_x = x
            last_forward_progress_time = sim_time

        phase = "settle" if sim_time < start_delay_s else "traverse"
        if finish_payload is not None:
            phase = "finished_" + finish_payload["outcome"]
        camera.update(sim_time, x, phase)

        if frame_dir is not None and sim_time + 1e-9 >= next_frame_time:
            robot.exportImage(str(frame_dir / f"frame_{frame_index:06d}.png"), 100)
            frame_index += 1
            next_frame_time += frame_acceleration / 30.0

        if sim_time + 1e-9 >= next_sample_time:
            telemetry.append({
                "t": round(sim_time, 3),
                "x": round(x, 4),
                "y": round(y, 4),
                "z": round(z, 4),
                "yaw_deg": round(math.degrees(yaw), 2),
                "pitch_deg": round(pitch_deg, 2),
                "roll_deg": round(roll_deg, 2),
                "speed_m_s": round(speed, 4),
                "waypoint": waypoint_index,
                "recovering": sim_time < recovery_until,
            })
            next_sample_time += 0.25

        if finish_payload is not None:
            for motor in motors:
                motor.setVelocity(0.0)
            if sim_time - finish_time >= finish_hold_s and auto_quit:
                robot.simulationQuit(0)
                return 0
            continue

        if sim_time < start_delay_s:
            continue

        goal_distance = math.hypot(x - WAYPOINTS[-1][0], y - WAYPOINTS[-1][1])
        outcome = ""
        reason = ""
        if goal_distance < 0.72 and waypoint_index >= len(WAYPOINTS) - 2:
            outcome, reason = "success", "goal_reached"
        elif sim_time >= timeout_s:
            outcome, reason = "failed", "timeout"
        elif abs(y) > 4.7:
            outcome, reason = "failed", "left_course"

        if outcome:
            finish_payload = {
                "version": 1,
                "experiment": "husky_extreme_terrain",
                "mode": mode,
                "outcome": outcome,
                "reason": reason,
                "goal_reached": outcome == "success",
                "sim_time_s": round(sim_time, 3),
                "wall_time_s": round(time.time() - wall_started, 3),
                "distance_m": round(distance_m, 3),
                "progress_x_m": round(x + 13.5, 3),
                "final_pose": {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "yaw": round(yaw, 5)},
                "goal_error_m": round(goal_distance, 4),
                "waypoints_reached": waypoint_index,
                "waypoints_total": len(WAYPOINTS),
                "max_abs_pitch_deg": round(max_pitch_deg, 2),
                "max_abs_roll_deg": round(max_roll_deg, 2),
                "max_z_m": round(max_z_m, 3),
                "recoveries": recoveries,
                "controller_inputs": ["robot_pose", "robot_attitude", "frozen_course_waypoints"],
                "controller_denied": ["pose_writes", "teleportation", "kinematic_goal_snap"],
                "capture": {
                    "camera_mode": camera_mode,
                    "frame_acceleration": frame_acceleration,
                    "frames": frame_index,
                },
                "telemetry": telemetry,
            }
            write_result(result_path, finish_payload)
            finish_time = sim_time
            continue

        if mode == "baseline":
            left_target = right_target = 3.2
            if sim_time - last_forward_progress_time > 6.0:
                finish_payload = {
                    "version": 1,
                    "experiment": "husky_extreme_terrain",
                    "mode": mode,
                    "outcome": "failed",
                    "reason": "blocked_at_first_gate",
                    "goal_reached": False,
                    "sim_time_s": round(sim_time, 3),
                    "wall_time_s": round(time.time() - wall_started, 3),
                    "distance_m": round(distance_m, 3),
                    "progress_x_m": round(x + 13.5, 3),
                    "max_forward_x_m": round(best_x, 4),
                    "final_pose": {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "yaw": round(yaw, 5)},
                    "goal_error_m": round(goal_distance, 4),
                    "waypoints_reached": 0,
                    "waypoints_total": len(WAYPOINTS),
                    "max_abs_pitch_deg": round(max_pitch_deg, 2),
                    "max_abs_roll_deg": round(max_roll_deg, 2),
                    "max_z_m": round(max_z_m, 3),
                    "recoveries": 0,
                    "controller_inputs": [],
                    "controller_denied": ["steering_feedback", "pose_writes", "teleportation"],
                    "capture": {"camera_mode": camera_mode, "frame_acceleration": frame_acceleration, "frames": frame_index},
                    "telemetry": telemetry,
                }
                write_result(result_path, finish_payload)
                finish_time = sim_time
                continue
            if speed < 0.018:
                if stall_started is None:
                    stall_started = sim_time
                elif sim_time - stall_started > 5.0:
                    finish_payload = {
                        "version": 1,
                        "experiment": "husky_extreme_terrain",
                        "mode": mode,
                        "outcome": "failed",
                        "reason": "stalled",
                        "goal_reached": False,
                        "sim_time_s": round(sim_time, 3),
                        "wall_time_s": round(time.time() - wall_started, 3),
                        "distance_m": round(distance_m, 3),
                        "progress_x_m": round(x + 13.5, 3),
                        "final_pose": {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "yaw": round(yaw, 5)},
                        "goal_error_m": round(goal_distance, 4),
                        "waypoints_reached": 0,
                        "waypoints_total": len(WAYPOINTS),
                        "max_abs_pitch_deg": round(max_pitch_deg, 2),
                        "max_abs_roll_deg": round(max_roll_deg, 2),
                        "max_z_m": round(max_z_m, 3),
                        "recoveries": 0,
                        "controller_inputs": [],
                        "controller_denied": ["steering_feedback", "pose_writes", "teleportation"],
                        "capture": {"camera_mode": camera_mode, "frame_acceleration": frame_acceleration, "frames": frame_index},
                        "telemetry": telemetry,
                    }
                    write_result(result_path, finish_payload)
                    finish_time = sim_time
                    continue
            else:
                stall_started = None
        else:
            while waypoint_index < len(WAYPOINTS) - 1:
                wx, wy = WAYPOINTS[waypoint_index]
                if math.hypot(wx - x, wy - y) >= 0.72:
                    break
                waypoint_index += 1
                print(f"[husky_extreme_terrain] waypoint={waypoint_index}/{len(WAYPOINTS) - 1}", flush=True)

            target_x, target_y = WAYPOINTS[waypoint_index]
            desired_yaw = math.atan2(target_y - y, target_x - x)
            heading_error = wrap_pi(desired_yaw - yaw)
            attitude_load = max(abs(pitch_deg) / 18.0, abs(roll_deg) / 15.0)
            terrain_scale = clamp(1.0 - 0.42 * attitude_load, 0.42, 1.0)
            if abs(heading_error) > 0.85:
                # This is the measured turn primitive used by the maze
                # benchmark.  A near-zero crawl and strong differential make
                # the high-friction skid-steer rotate before it advances into
                # a gate; the wheels still supply every bit of motion.
                linear = 0.025
                angular = clamp(heading_error * 3.2, -2.0, 2.0)
                left_target = clamp((linear - angular * 0.2854) / 0.1651, -6.0, 6.0)
                right_target = clamp((linear + angular * 0.2854) / 0.1651, -6.0, 6.0)
            else:
                linear = 0.72 * terrain_scale * max(0.55, math.cos(heading_error))
                angular = clamp(heading_error * 2.2, -0.82, 0.82)
                left_target = clamp((linear - angular * 0.2854) / 0.1651, -6.0, 6.0)
                right_target = clamp((linear + angular * 0.2854) / 0.1651, -6.0, 6.0)

            commanded = abs(left_target) + abs(right_target) > 1.0
            if commanded and speed < 0.022:
                # Low translational speed is expected while correcting a
                # large heading error; only arm recovery on a drive segment.
                if abs(heading_error) >= 0.85:
                    stall_started = None
                elif stall_started is None:
                    stall_started = sim_time
                elif sim_time - stall_started > 2.2 and sim_time >= recovery_until:
                    recoveries += 1
                    recovery_until = sim_time + 1.2
                    stall_started = None
                    print(f"[husky_extreme_terrain] recovery={recoveries} x={x:.2f} y={y:.2f}", flush=True)
            else:
                stall_started = None

            if sim_time < recovery_until:
                # A short reverse arc unloads a wedged front corner.  Direction
                # alternates so repeated recoveries cannot bias one side.
                bias = -0.9 if recoveries % 2 else 0.9
                left_target = -2.4 - bias
                right_target = -2.4 + bias

            if recoveries > 8:
                finish_payload = {
                    "version": 1,
                    "experiment": "husky_extreme_terrain",
                    "mode": mode,
                    "outcome": "failed",
                    "reason": "recovery_limit",
                    "goal_reached": False,
                    "sim_time_s": round(sim_time, 3),
                    "wall_time_s": round(time.time() - wall_started, 3),
                    "distance_m": round(distance_m, 3),
                    "progress_x_m": round(x + 13.5, 3),
                    "final_pose": {"x": round(x, 4), "y": round(y, 4), "z": round(z, 4), "yaw": round(yaw, 5)},
                    "goal_error_m": round(goal_distance, 4),
                    "waypoints_reached": waypoint_index,
                    "waypoints_total": len(WAYPOINTS),
                    "max_abs_pitch_deg": round(max_pitch_deg, 2),
                    "max_abs_roll_deg": round(max_roll_deg, 2),
                    "max_z_m": round(max_z_m, 3),
                    "recoveries": recoveries,
                    "controller_inputs": ["robot_pose", "robot_attitude", "frozen_course_waypoints"],
                    "controller_denied": ["pose_writes", "teleportation", "kinematic_goal_snap"],
                    "capture": {"camera_mode": camera_mode, "frame_acceleration": frame_acceleration, "frames": frame_index},
                    "telemetry": telemetry,
                }
                write_result(result_path, finish_payload)
                finish_time = sim_time
                continue

        current_left += clamp(left_target - current_left, -0.24, 0.24)
        current_right += clamp(right_target - current_right, -0.24, 0.24)
        for motor in left_motors:
            motor.setVelocity(current_left)
        for motor in right_motors:
            motor.setVelocity(current_right)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
