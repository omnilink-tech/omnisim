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

"""Robot-control tools — thin HTTP proxies to the OmniSim bridge.

Every tool in this module forwards a JSON payload to the OmniSim bridge
and returns the bridge's response as-is (with a defensive outer wrapper
for transport errors). Axis never touches OmniSim directly: the bridge
owns every `Robot()`, `getDevice`, `setPosition`, and `robot.step`.

The contract is documented in `knowledge/omnisim-bridge.md`. Tools here
should stay thin — validation belongs in the system prompt ("don't
command without first reading telemetry", "cap joint steps at
IK_MAX_DQ") and in the bridge's clamp, not in a tool wrapper.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from ._base import ALWAYS, GUARDED, SAFE, ToolSpec, bridge_call


# ── Telemetry (safe) ─────────────────────────────────────────────────

def _impl_list_robots(**_: Any) -> Dict[str, Any]:
    return bridge_call("list_robots")


def _impl_get_robot_state(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("get_robot_state", {"id": id})


def _impl_read_joints(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("read_joints", {"id": id})


def _impl_read_tcp_pose(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("read_tcp_pose", {"id": id})


def _impl_read_sensor(id: str = "", sensor: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not sensor:
        return {"error": "sensor is required"}
    return bridge_call("read_sensor", {"id": id, "sensor": sensor})


def _impl_get_simulation_time(**_: Any) -> Dict[str, Any]:
    return bridge_call("get_simulation_time")


def _impl_solve_ik(
    id: str = "",
    xyz: Optional[List[float]] = None,
    rpy: Optional[List[float]] = None,
    **_: Any,
) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not xyz or len(xyz) != 3:
        return {"error": "xyz must be a 3-element list"}
    payload: Dict[str, Any] = {"id": id, "xyz": list(xyz)}
    if rpy:
        payload["rpy"] = list(rpy)
    return bridge_call("solve_ik", payload)


# ── Motion (guarded) ─────────────────────────────────────────────────

def _impl_set_joint_positions(
    id: str = "",
    q: Optional[List[float]] = None,
    **_: Any,
) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not q or not isinstance(q, list):
        return {"error": "q must be a list of joint angles"}
    return bridge_call("set_joint_positions", {"id": id, "q": list(q)})


def _impl_set_tcp_target(
    id: str = "",
    xyz: Optional[List[float]] = None,
    rpy: Optional[List[float]] = None,
    **_: Any,
) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not xyz or len(xyz) != 3:
        return {"error": "xyz must be a 3-element list"}
    payload: Dict[str, Any] = {"id": id, "xyz": list(xyz)}
    if rpy:
        payload["rpy"] = list(rpy)
    return bridge_call("set_tcp_target", payload)


def _impl_reset_to_home(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("reset_to_home", {"id": id})


def _impl_plan_trajectory(
    id: str = "",
    waypoints: Optional[List[Any]] = None,
    constraints: Optional[Dict[str, Any]] = None,
    **_: Any,
) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not waypoints:
        return {"error": "waypoints is required"}
    payload: Dict[str, Any] = {"id": id, "waypoints": list(waypoints)}
    if constraints:
        payload["constraints"] = dict(constraints)
    return bridge_call("plan_trajectory", payload)


def _impl_execute_trajectory(id: str = "", traj_id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    if not traj_id:
        return {"error": "traj_id is required"}
    return bridge_call("execute_trajectory", {"id": id, "traj_id": traj_id})


def _impl_open_gripper(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("open_gripper", {"id": id})


def _impl_close_gripper(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("close_gripper", {"id": id})


# stop_robot is SAFE by design — never gate, never rate-limit.
def _impl_stop_robot(id: str = "", **_: Any) -> Dict[str, Any]:
    if not id:
        return {"error": "id is required"}
    return bridge_call("stop_robot", {"id": id})


# ── Simulation-control (guarded; sim.admin) ─────────────────────────

def _impl_pause_simulation(**_: Any) -> Dict[str, Any]:
    return bridge_call("pause_simulation")


def _impl_resume_simulation(**_: Any) -> Dict[str, Any]:
    return bridge_call("resume_simulation")


# ── SPECS ────────────────────────────────────────────────────────────

_Q_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "description": "Joint angles in radians, ordered to match the robot's joint_names.",
}
_XYZ_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "minItems": 3,
    "maxItems": 3,
    "description": "TCP position in meters: [x, y, z] in the world frame.",
}
_RPY_SCHEMA = {
    "type": "array",
    "items": {"type": "number"},
    "minItems": 3,
    "maxItems": 3,
    "description": "Optional TCP orientation in radians: [roll, pitch, yaw].",
}

SPECS = [
    ToolSpec(
        name="list_robots",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Enumerate every robot the OmniSim bridge knows about. Returns "
            "a list of {id, model, capabilities}. Call this at the start of "
            "a session, when you don't know which robot the operator means, "
            "or to confirm a newly-registered robot is visible."
        ),
        parameters={"type": "object", "properties": {}},
        impl=_impl_list_robots,
        tags=["robot", "discovery"],
    ),
    ToolSpec(
        name="get_robot_state",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Full state snapshot for one robot: q, qdot, TCP pose, fault "
            "flag, last_tick_at. Use before any motion command to confirm "
            "telemetry is fresh and no fault is active."
        ),
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string", "description": "Robot id from list_robots."}},
            "required": ["id"],
        },
        impl=_impl_get_robot_state,
        tags=["robot", "telemetry"],
    ),
    ToolSpec(
        name="read_joints",
        tier=SAFE,
        surface=ALWAYS,
        description="Freshest joint-angle read for one robot. Returns {q: float[]}. Ordering matches the robot's joint_names capability field.",
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_read_joints,
        tags=["robot", "telemetry"],
    ),
    ToolSpec(
        name="read_tcp_pose",
        tier=SAFE,
        surface=ALWAYS,
        description="Freshest TCP pose for one robot. Returns {xyz: float[3], rpy: float[3]} in the world frame.",
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_read_tcp_pose,
        tags=["robot", "telemetry"],
    ),
    ToolSpec(
        name="read_sensor",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Read a named sensor on a robot (force/torque, camera, etc.). "
            "Returns whatever shape the sensor reports, stamped with a "
            "timestamp. Sensor names are documented in the robot's "
            "capability record."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "sensor": {"type": "string", "description": "Sensor name from the robot's capability record."},
            },
            "required": ["id", "sensor"],
        },
        impl=_impl_read_sensor,
        tags=["robot", "telemetry", "sensor"],
    ),
    ToolSpec(
        name="get_simulation_time",
        tier=SAFE,
        surface=ALWAYS,
        description="Read OmniSim's current simulation time (seconds). Use to stamp setpoints and measure elapsed motion time.",
        parameters={"type": "object", "properties": {}},
        impl=_impl_get_simulation_time,
        tags=["simulation"],
    ),
    ToolSpec(
        name="solve_ik",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Solve inverse kinematics for a TCP target without issuing motion. "
            "Returns {q, err_norm}. ALWAYS call this before set_tcp_target on "
            "an unfamiliar target; if err_norm exceeds IK_TOL or the solved q "
            "is outside joint_limits, refuse the motion and report likely "
            "singularity or unreachable target."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "xyz": _XYZ_SCHEMA,
                "rpy": _RPY_SCHEMA,
            },
            "required": ["id", "xyz"],
        },
        impl=_impl_solve_ik,
        tags=["robot", "ik"],
    ),
    ToolSpec(
        name="stop_robot",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Emergency halt for one robot. ALWAYS AVAILABLE — never gated, "
            "never rate-limited. Idempotent. Call immediately on telemetry "
            "stale, fault flag, or any situation where the next motion step "
            "is unsafe."
        ),
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_stop_robot,
        tags=["robot", "safety"],
    ),
    ToolSpec(
        name="set_joint_positions",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Command joint-space setpoint for one robot. Bridge clamps the "
            "payload to joint_limits before executing. Call read_joints FIRST "
            "so the commanded delta stays within IK_MAX_DQ (default 0.08 rad) "
            "per control tick — for larger moves, break into a trajectory "
            "via plan_trajectory + execute_trajectory. Returns {accepted, "
            "clamped_q}."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "q": _Q_SCHEMA,
            },
            "required": ["id", "q"],
        },
        impl=_impl_set_joint_positions,
        tags=["robot", "motion"],
    ),
    ToolSpec(
        name="set_tcp_target",
        tier=GUARDED,
        surface=ALWAYS,
        description=(
            "Command task-space target for one robot. Bridge solves IK and "
            "steps the joints. Returns {accepted, solved_q, err_norm}. Call "
            "solve_ik first to preview convergence; refuse on "
            "err_norm > IK_TOL or solved_q outside joint_limits. rpy is "
            "optional — position-only targets are accepted for robots with "
            "ik_position_only capability."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "xyz": _XYZ_SCHEMA,
                "rpy": _RPY_SCHEMA,
            },
            "required": ["id", "xyz"],
        },
        impl=_impl_set_tcp_target,
        tags=["robot", "motion", "ik"],
    ),
    ToolSpec(
        name="reset_to_home",
        tier=GUARDED,
        surface=ALWAYS,
        description="Move one robot to its home_pose via a bridge-planned trajectory. Safe baseline for 'park the arm'.",
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_reset_to_home,
        tags=["robot", "motion"],
    ),
    ToolSpec(
        name="plan_trajectory",
        tier=SAFE,
        surface=ALWAYS,
        description=(
            "Plan a time-parameterized trajectory through a sequence of "
            "waypoints. Does NOT execute — returns {traj_id, duration, "
            "preview} that you then pass to execute_trajectory. Use for "
            "moves too large for a single IK_MAX_DQ step."
        ),
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "waypoints": {
                    "type": "array",
                    "items": {"type": "object"},
                    "description": "Sequence of {xyz, rpy?} or {q} waypoints.",
                },
                "constraints": {
                    "type": "object",
                    "description": "Optional: max_velocity, max_acceleration, blend_radius.",
                },
            },
            "required": ["id", "waypoints"],
        },
        impl=_impl_plan_trajectory,
        tags=["robot", "trajectory"],
    ),
    ToolSpec(
        name="execute_trajectory",
        tier=GUARDED,
        surface=ALWAYS,
        description="Execute a previously-planned trajectory. Streams setpoints through the bridge clamp on every tick.",
        parameters={
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "traj_id": {"type": "string", "description": "Trajectory id returned by plan_trajectory."},
            },
            "required": ["id", "traj_id"],
        },
        impl=_impl_execute_trajectory,
        tags=["robot", "motion", "trajectory"],
    ),
    ToolSpec(
        name="open_gripper",
        tier=GUARDED,
        surface=ALWAYS,
        description="Open the gripper on a robot that has one. Returns {state}. Refuses with effector_unavailable when no gripper is registered.",
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_open_gripper,
        tags=["robot", "effector"],
    ),
    ToolSpec(
        name="close_gripper",
        tier=GUARDED,
        surface=ALWAYS,
        description="Close the gripper on a robot that has one. Returns {state}.",
        parameters={
            "type": "object",
            "properties": {"id": {"type": "string"}},
            "required": ["id"],
        },
        impl=_impl_close_gripper,
        tags=["robot", "effector"],
    ),
    ToolSpec(
        name="pause_simulation",
        tier=GUARDED,
        surface=ALWAYS,
        description="Pause OmniSim's world clock. Requires sim.admin scope. Use sparingly — paused sim means stale telemetry, so plan to resume promptly.",
        parameters={"type": "object", "properties": {}},
        impl=_impl_pause_simulation,
        tags=["simulation", "admin"],
    ),
    ToolSpec(
        name="resume_simulation",
        tier=GUARDED,
        surface=ALWAYS,
        description="Resume OmniSim's world clock after a pause. Requires sim.admin scope.",
        parameters={"type": "object", "properties": {}},
        impl=_impl_resume_simulation,
        tags=["simulation", "admin"],
    ),
]
