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

"""``simulation_interfaces`` server for OmniSim (Tier 1).

Implements the ros-simulation/simulation_interfaces standard (v2.1.0) on top of
the OmniSim World Harness HTTP surface, making OmniSim the fourth implementation
alongside Gazebo, Isaac Sim and O3DE.

Every verb here maps onto a harness endpoint that already existed; nothing in the
OmniSim engine was changed to support ROS.

============================  ==========================================
ROS 2 service                 OmniSim harness endpoint
============================  ==========================================
GetSimulatorFeatures          (static, cross-checked against /capabilities)
GetSimulationState            GET  /sim/state
SetSimulationState            POST /sim/reset            (see caveats)
StepSimulation                POST /sim/step
SimulateSteps (action)        POST /sim/step, chunked for feedback
ResetSimulation               POST /sim/reset + POST /scene/delete
GetEntities                   GET  /scene/tree
GetEntityState                GET  /scene/tree
GetEntitiesStates             GET  /scene/tree
SetEntityState                POST /scene/set_pose
GetEntityInfo                 GET  /scene/node/<def>
GetEntityBounds               GET  /scene/tree?bounds=1
SpawnEntity                   POST /scene/spawn
DeleteEntity                  POST /scene/delete
LoadWorld                     POST /world/load
GetCurrentWorld               GET  /sim/state
============================  ==========================================

DISCLOSED DIVERGENCES FROM THE STANDARD
---------------------------------------
These are declared through the feature flags and repeated in ``custom_info`` so a
caller learns them from the API, not from surprise:

1. **There is no pause.** OmniSim's engine free-runs between HTTP calls; the
   harness exposes no pause verb. ``SIMULATION_STATE_PAUSE`` is therefore *not*
   advertised, ``GetSimulationState`` answers ``STATE_PLAYING`` whenever a world
   is loaded, and ``StepSimulation`` means "advance at least N basic steps from
   here", not "advance exactly N from a frozen state".
2. **Twist and acceleration are not measured.** The harness reports body poses
   but no body velocities, so ``EntityState.twist`` and ``.acceleration`` are
   returned as zeros. They are *unmeasured*, not observed-to-be-zero. Joint
   velocities are available separately on ``/joint_states`` (Tier 2).
3. **``ResetSimulation`` re-pins every motor.** This is the harness's documented
   behaviour: after a reset each velocity-mode wheel becomes a position hold.
   The node surfaces it in ``error_message`` on success so it cannot be missed.
"""

from __future__ import annotations

import threading
from typing import Any

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from simulation_interfaces.action import SimulateSteps
from simulation_interfaces.msg import (
    Bounds,
    EntityCategory,
    EntityInfo,
    EntityState,
    Result,
    SimulationState,
    SimulatorFeatures,
)
from simulation_interfaces.srv import (
    DeleteEntity,
    GetCurrentWorld,
    GetEntities,
    GetEntitiesStates,
    GetEntityBounds,
    GetEntityInfo,
    GetEntityState,
    GetSimulationState,
    GetSimulatorFeatures,
    LoadWorld,
    ResetSimulation,
    SetEntityState,
    SetSimulationState,
    SpawnEntity,
    StepSimulation,
)

from omnisim_ros2 import entities as ent
from omnisim_ros2.conversions import (
    is_valid_quaternion,
    orientation_to_quaternion,
    position_to_xyz,
    quaternion_to_axis_angle,
    sim_time_ms_to_ros,
)
from omnisim_ros2.harness_client import HarnessClient, HarnessUnreachable

# The feature set OmniSim actually implements. Kept as a literal list (rather
# than derived) so that adding a service without deciding what to advertise is a
# visible omission rather than a silent overclaim.
SUPPORTED_FEATURES = [
    SimulatorFeatures.SPAWNING,
    SimulatorFeatures.DELETING,
    SimulatorFeatures.ENTITY_TAGS,
    SimulatorFeatures.ENTITY_BOUNDS,
    SimulatorFeatures.ENTITY_BOUNDS_BOX,
    SimulatorFeatures.ENTITY_CATEGORIES,
    SimulatorFeatures.SPAWNING_RESOURCE_STRING,
    SimulatorFeatures.ENTITY_STATE_GETTING,
    SimulatorFeatures.ENTITY_STATE_SETTING,
    SimulatorFeatures.ENTITY_INFO_GETTING,
    SimulatorFeatures.SIMULATION_RESET,
    SimulatorFeatures.SIMULATION_RESET_TIME,
    SimulatorFeatures.SIMULATION_RESET_STATE,
    SimulatorFeatures.SIMULATION_RESET_SPAWNED,
    SimulatorFeatures.SIMULATION_STATE_GETTING,
    SimulatorFeatures.SIMULATION_STATE_SETTING,
    SimulatorFeatures.STEP_SIMULATION_SINGLE,
    SimulatorFeatures.STEP_SIMULATION_MULTIPLE,
    SimulatorFeatures.STEP_SIMULATION_ACTION,
    SimulatorFeatures.WORLD_LOADING,
    SimulatorFeatures.WORLD_INFO_GETTING,
]

CUSTOM_INFO = (
    "OmniSim ROS 2 sidecar (packages/omnisim-ros2). Backed by the OmniSim World "
    "Harness HTTP surface; the engine carries no ROS dependency. "
    "DIVERGENCES: (1) no pause -- the engine free-runs, so STATE_PAUSED is not "
    "supported and StepSimulation advances at least N basic steps rather than "
    "exactly N from a frozen state; (2) EntityState.twist and .acceleration are "
    "NOT measured and are returned as zeros -- the harness reports poses only "
    "(joint velocities are on /joint_states); (3) ResetSimulation re-pins every "
    "motor to a position hold, per the harness's documented reset behaviour. "
    "Docs: docs/developer/ros2-integration.md"
)

# The harness step RPC has a 120 s ceiling. Chunking the action keeps each call
# well inside it and is what lets the action publish feedback at all.
ACTION_CHUNK_STEPS = 10


def _ok(msg: str = "") -> Result:
    r = Result()
    r.result = Result.RESULT_OK
    r.error_message = msg
    return r


def _err(code: int, msg: str) -> Result:
    r = Result()
    r.result = code
    r.error_message = msg
    return r


def _guarded(handler):
    """Wrap a service handler so an internal error cannot hang the caller.

    rclpy lets an exception escape a service callback without sending a reply,
    which leaves ``ros2 service call`` blocked for ever with no indication of
    why. Every handler therefore returns a populated ``Result`` even when it
    fails unexpectedly -- an error a caller can read beats a silent hang.
    """
    import functools

    @functools.wraps(handler)
    def wrapper(self, req, resp):
        try:
            return handler(self, req, resp)
        except Exception as exc:  # noqa: BLE001 -- deliberate catch-all, see docstring
            self.get_logger().error(f"{handler.__name__} failed: {exc!r}")
            try:
                resp.result = _err(
                    Result.RESULT_OPERATION_FAILED,
                    f"internal error in {handler.__name__}: {exc}",
                )
            except AttributeError:
                # A response type with no `result` field cannot carry the error;
                # returning it unpopulated at least unblocks the caller.
                pass
            return resp

    return wrapper


class SimulationInterfacesNode(Node):
    """Serves the simulation_interfaces standard against one OmniSim harness."""

    def __init__(self) -> None:
        super().__init__("omnisim_simulation_interfaces")

        self.declare_parameter("harness_url", "http://127.0.0.1:6789")
        self.declare_parameter("request_timeout_s", 30.0)
        self.declare_parameter("world_frame", "world")

        url = self.get_parameter("harness_url").get_parameter_value().string_value
        timeout = self.get_parameter("request_timeout_s").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.client = HarnessClient(url, timeout_s=timeout)

        # Entities this node spawned, so ResetSimulation SCOPE_SPAWNED can honour
        # its contract. The harness's own reset restores authored poses but does
        # not remove nodes added at runtime, so we must track them ourselves.
        self._spawned: set[str] = set()
        self._spawned_lock = threading.Lock()

        # (world_path, frozenset_of_injected_defs) -- see _injected_defs().
        self._injected_cache: tuple[Any, frozenset[str]] = (None, frozenset())

        # Every handler talks HTTP, so they must not serialise behind one another
        # or a slow world load would block an unrelated state query.
        cb = ReentrantCallbackGroup()

        self.create_service(GetSimulatorFeatures, "~/get_simulator_features",
                            self.on_get_features, callback_group=cb)
        self.create_service(GetSimulationState, "~/get_simulation_state",
                            self.on_get_sim_state, callback_group=cb)
        self.create_service(SetSimulationState, "~/set_simulation_state",
                            self.on_set_sim_state, callback_group=cb)
        self.create_service(StepSimulation, "~/step_simulation",
                            self.on_step, callback_group=cb)
        self.create_service(ResetSimulation, "~/reset_simulation",
                            self.on_reset, callback_group=cb)
        self.create_service(GetEntities, "~/get_entities",
                            self.on_get_entities, callback_group=cb)
        self.create_service(GetEntityState, "~/get_entity_state",
                            self.on_get_entity_state, callback_group=cb)
        self.create_service(GetEntitiesStates, "~/get_entities_states",
                            self.on_get_entities_states, callback_group=cb)
        self.create_service(SetEntityState, "~/set_entity_state",
                            self.on_set_entity_state, callback_group=cb)
        self.create_service(GetEntityInfo, "~/get_entity_info",
                            self.on_get_entity_info, callback_group=cb)
        self.create_service(GetEntityBounds, "~/get_entity_bounds",
                            self.on_get_entity_bounds, callback_group=cb)
        self.create_service(SpawnEntity, "~/spawn_entity",
                            self.on_spawn, callback_group=cb)
        self.create_service(DeleteEntity, "~/delete_entity",
                            self.on_delete, callback_group=cb)
        self.create_service(LoadWorld, "~/load_world",
                            self.on_load_world, callback_group=cb)
        self.create_service(GetCurrentWorld, "~/get_current_world",
                            self.on_get_current_world, callback_group=cb)

        self._action = ActionServer(
            self, SimulateSteps, "~/simulate_steps",
            execute_callback=self.on_simulate_steps,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        self.get_logger().info(f"simulation_interfaces server up, harness={url}")
        self._log_harness_banner()

    # -- helpers ----------------------------------------------------------

    def _log_harness_banner(self) -> None:
        """Report the harness we are attached to, or why we are not."""
        try:
            resp = self.client.sim_state()
        except HarnessUnreachable as exc:
            self.get_logger().warn(
                f"{exc} -- services are up and will answer with "
                f"RESULT_OPERATION_FAILED until the harness is reachable."
            )
            return
        world = resp.body.get("world")
        self.get_logger().info(
            f"harness reachable; world={world or '<none loaded>'} "
            f"supervisor_connected={resp.body.get('supervisor_connected')}"
        )

    def _scene_tree(self, bounds: bool = False) -> tuple[dict[str, Any] | None, Result]:
        try:
            resp = self.client.scene_tree(bounds=bounds)
        except HarnessUnreachable as exc:
            return None, _err(Result.RESULT_OPERATION_FAILED, str(exc))
        if not resp.ok:
            return None, _err(Result.RESULT_OPERATION_FAILED, resp.error)
        return resp.body, _ok()

    def _injected_defs(self) -> set[str]:
        """DEFs of harness scaffolding robots, which are not user entities.

        Sourced from ``/robots?include_harness=1`` because ``/scene/tree``'s own
        ``harness_injected`` list is unreliable (see entities.py). Cached per
        world, since the supervisor cannot change without a reload.
        """
        try:
            state = self.client.sim_state()
            world = state.body.get("world")
        except HarnessUnreachable:
            return set(self._injected_cache[1])
        if self._injected_cache[0] == world:
            return set(self._injected_cache[1])
        found: set[str] = set()
        try:
            r = self.client.robots(include_harness=True)
            if r.ok:
                for robot in r.body.get("robots") or []:
                    if robot.get("harness_injected") and robot.get("def"):
                        found.add(str(robot["def"]))
        except HarnessUnreachable:
            return set(self._injected_cache[1])
        self._injected_cache = (world, frozenset(found))
        return found

    def _entities(self, tree: dict[str, Any]) -> list[ent.Entity]:
        return ent.entities_from_scene_tree(tree, exclude_defs=self._injected_defs())

    def _entity_state_msg(self, entity: ent.Entity, stamp=None) -> EntityState:
        st = EntityState()
        st.header.frame_id = self.world_frame
        # The stamp is passed in by callers that build several states at once:
        # every _sim_stamp() is an HTTP round trip, and doing one per entity made
        # GetEntitiesStates cost N+1 requests for a single snapshot.
        st.header.stamp = stamp if stamp is not None else self._sim_stamp()
        x, y, z = position_to_xyz(entity.position)
        st.pose.position.x, st.pose.position.y, st.pose.position.z = x, y, z
        qx, qy, qz, qw = orientation_to_quaternion(entity.orientation)
        st.pose.orientation.x = qx
        st.pose.orientation.y = qy
        st.pose.orientation.z = qz
        st.pose.orientation.w = qw
        # twist / acceleration deliberately left at zero -- see module docstring.
        return st

    def _sim_stamp(self):
        """Stamp from the simulator clock, falling back to the node clock.

        A wrong-but-monotonic stamp is worse than an honest fallback, so when the
        harness cannot tell us the sim time we use the node clock rather than
        inventing one.
        """
        from builtin_interfaces.msg import Time

        try:
            resp = self.client.sim_state()
            ms = resp.body.get("sim_time_ms")
        except HarnessUnreachable:
            ms = None
        if ms is None:
            return self.get_clock().now().to_msg()
        sec, nanosec = sim_time_ms_to_ros(ms)
        t = Time()
        t.sec = sec
        t.nanosec = nanosec
        return t

    def _select_entities(self, filters) -> tuple[list[ent.Entity] | None, Result]:
        """Apply EntityFilters to the scene, returning the matching entities."""
        tree, result = self._scene_tree()
        if tree is None:
            return None, result
        found = self._entities(tree)

        pattern = getattr(filters, "filter", "") or ""
        if pattern:
            found = [e for e in found if ent.matches_filter(e.name, pattern)]

        # Bounds filtering is not implemented; the standard requires us to say so
        # rather than silently ignore the field.
        bounds = getattr(filters, "bounds", None)
        if bounds is not None and getattr(bounds, "type", 0):
            return None, _err(
                Result.RESULT_FEATURE_UNSUPPORTED,
                "bounds-based entity filtering is not implemented; "
                "filter by name or category instead",
            )

        tags_filter = getattr(filters, "tags", None)
        wanted_tags = list(getattr(tags_filter, "tags", []) or []) if tags_filter else []
        if wanted_tags:
            mode = getattr(tags_filter, "filter_mode", 0)
            def keep(e: ent.Entity) -> bool:
                have = set(e.tags)
                return (
                    all(t in have for t in wanted_tags)
                    if mode == 1
                    else any(t in have for t in wanted_tags)
                )
            found = [e for e in found if keep(e)]

        categories = [c.category for c in (getattr(filters, "categories", []) or [])]
        if categories:
            kept: list[ent.Entity] = []
            for e in found:
                detail = None
                if e.addressable:
                    try:
                        r = self.client.scene_node(e.name)
                        detail = r.body if r.ok else None
                    except HarnessUnreachable:
                        detail = None
                if ent.category_for(e.node_type, detail) in categories:
                    kept.append(e)
            found = kept
        return found, _ok()

    # -- simulator features / state --------------------------------------

    @_guarded
    def on_get_features(self, _req, resp):
        f = SimulatorFeatures()
        f.features = list(SUPPORTED_FEATURES)
        f.spawn_formats = ["vrml", "urdf"]
        f.custom_info = CUSTOM_INFO
        resp.features = f
        return resp

    @_guarded
    def on_get_sim_state(self, _req, resp):
        try:
            r = self.client.sim_state()
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        if not r.ok:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
            return resp
        body = r.body
        state = SimulationState()
        load_state = body.get("load_state")
        if load_state == "in_progress":
            state.state = SimulationState.STATE_LOADING_WORLD
        elif not body.get("world") or not body.get("running"):
            state.state = SimulationState.STATE_NO_WORLD
        else:
            # OmniSim's engine free-runs; a loaded, running world is PLAYING.
            state.state = SimulationState.STATE_PLAYING
        resp.state = state
        resp.result = _ok()
        return resp

    @_guarded
    def on_set_sim_state(self, req, resp):
        target = req.state.state
        if target == SimulationState.STATE_PAUSED:
            resp.result = _err(
                Result.RESULT_FEATURE_UNSUPPORTED,
                "OmniSim's engine free-runs and the harness exposes no pause verb; "
                "SIMULATION_STATE_PAUSE is not advertised in GetSimulatorFeatures",
            )
            return resp
        if target == SimulationState.STATE_QUITTING:
            resp.result = _err(
                Result.RESULT_FEATURE_UNSUPPORTED,
                "refusing to quit the simulator: the harness owns the engine "
                "process and other clients may be attached to it",
            )
            return resp
        try:
            current = self.client.sim_state()
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        running = bool(current.body.get("world")) and bool(current.body.get("running"))
        if target == SimulationState.STATE_PLAYING:
            if running:
                resp.result = _err(
                    SetSimulationState.Response.ALREADY_IN_TARGET_STATE,
                    "already playing; OmniSim's engine free-runs whenever a world is loaded",
                )
            else:
                resp.result = _err(
                    Result.RESULT_INCORRECT_STATE,
                    "no world is loaded; call LoadWorld first",
                )
            return resp
        if target == SimulationState.STATE_STOPPED:
            if not running:
                resp.result = _err(
                    SetSimulationState.Response.ALREADY_IN_TARGET_STATE,
                    "no world is loaded",
                )
                return resp
            try:
                r = self.client.sim_reset()
            except HarnessUnreachable as exc:
                resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
                return resp
            if not r.ok:
                resp.result = _err(
                    SetSimulationState.Response.STATE_TRANSITION_ERROR, r.error
                )
                return resp
            resp.result = _ok(
                "reset to authored state and t=0; note OmniSim cannot hold a "
                "stopped state -- the engine resumes free-running immediately"
            )
            return resp
        resp.result = _err(
            Result.RESULT_INCORRECT_STATE, f"unrecognised target state {target}"
        )
        return resp

    @_guarded
    def on_get_current_world(self, _req, resp):
        try:
            r = self.client.sim_state()
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        world = r.body.get("world")
        if not world:
            resp.result = _err(
                GetCurrentWorld.Response.NO_WORLD_LOADED, "no world is loaded"
            )
            return resp
        self._fill_world_resource(resp.world, str(world))
        resp.result = _ok()
        return resp

    @staticmethod
    def _fill_world_resource(world_msg, path: str) -> None:
        """Populate a ``WorldResource`` from an OmniSim world path.

        ``WorldResource`` nests the URI inside a ``Resource`` sub-message; the
        top level carries only the human-readable name, description and tags.
        """
        world_msg.name = path.replace("\\", "/").rsplit("/", 1)[-1]
        world_msg.world_resource.uri = path
        world_msg.description = f"OmniSim world loaded by the harness from {path}"

    @_guarded
    def on_load_world(self, req, resp):
        uri = (req.world_resource.uri or "").strip()
        if not uri:
            if req.world_resource.resource_string:
                resp.result = _err(
                    LoadWorld.Response.UNSUPPORTED_FORMAT,
                    "world_resource.resource_string is not supported "
                    "(WORLD_RESOURCE_STRING is not advertised); pass a uri "
                    "naming a .omniworld or .wbt file the harness can open",
                )
            else:
                resp.result = _err(
                    LoadWorld.Response.NO_RESOURCE,
                    "world_resource.uri is required",
                )
            return resp
        path = uri[len("file://") :] if uri.startswith("file://") else uri
        try:
            r = self.client.world_load(path, wait_s=120.0)
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        if not r.ok:
            codes = ", ".join(
                str(d.get("code"))
                for d in (r.body.get("diagnostics") or [])
                if d.get("severity") == "error"
            )
            detail = f"{r.error}{'; diagnostics: ' + codes if codes else ''}"
            # The harness classifies load failures precisely; map the parse-ish
            # ones onto the standard's specific codes rather than flattening
            # everything to OPERATION_FAILED.
            code = Result.RESULT_OPERATION_FAILED
            if "PARSE" in codes or "PROTO" in codes:
                code = LoadWorld.Response.RESOURCE_PARSE_ERROR
            elif "TEXTURE" in codes or "MESH" in codes or "DOWNLOAD" in codes:
                code = LoadWorld.Response.MISSING_ASSETS
            resp.result = _err(code, detail)
            return resp
        with self._spawned_lock:
            self._spawned.clear()
        self._fill_world_resource(resp.world, str(r.body.get("world") or path))
        resp.result = _ok(f"loaded in {r.body.get('load_ms')} ms")
        return resp

    # -- stepping ---------------------------------------------------------

    @_guarded
    def on_step(self, req, resp):
        steps = int(req.steps)
        if steps < 1:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, "steps must be >= 1")
            return resp
        try:
            r = self.client.sim_step(steps, timeout_s=max(self.client.timeout_s, 120.0))
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        if not r.ok:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
            return resp
        resp.result = _ok(
            f"advanced to {r.body.get('advanced_to_ms')} ms sim time "
            f"in {r.body.get('wall_ms')} ms wall"
        )
        return resp

    def on_simulate_steps(self, goal_handle):
        total = int(goal_handle.request.steps)
        feedback = SimulateSteps.Feedback()
        result = SimulateSteps.Result()
        if total < 1:
            goal_handle.abort()
            result.result = _err(Result.RESULT_OPERATION_FAILED, "steps must be >= 1")
            return result
        done = 0
        while done < total:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.result = _err(
                    Result.RESULT_OPERATION_FAILED,
                    f"cancelled after {done} of {total} steps",
                )
                return result
            chunk = min(ACTION_CHUNK_STEPS, total - done)
            try:
                r = self.client.sim_step(chunk, timeout_s=max(self.client.timeout_s, 120.0))
            except HarnessUnreachable as exc:
                goal_handle.abort()
                result.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
                return result
            if not r.ok:
                goal_handle.abort()
                result.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
                return result
            done += int(r.body.get("steps_executed") or chunk)
            feedback.completed_steps = min(done, total)
            feedback.remaining_steps = max(total - done, 0)
            goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result.result = _ok(f"completed {done} steps")
        return result

    @_guarded
    def on_reset(self, req, resp):
        scope = int(req.scope) or ResetSimulation.Request.SCOPE_ALL
        wants_spawned = bool(scope & ResetSimulation.Request.SCOPE_SPAWNED) or (
            scope == ResetSimulation.Request.SCOPE_ALL
        )
        wants_state = bool(scope & ResetSimulation.Request.SCOPE_STATE) or (
            scope == ResetSimulation.Request.SCOPE_ALL
        )
        wants_time = bool(scope & ResetSimulation.Request.SCOPE_TIME) or (
            scope == ResetSimulation.Request.SCOPE_ALL
        )
        notes: list[str] = []

        if wants_spawned:
            with self._spawned_lock:
                pending = sorted(self._spawned)
            if pending:
                try:
                    r = self.client.scene_delete(pending)
                except HarnessUnreachable as exc:
                    resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
                    return resp
                if r.ok:
                    removed = [x.get("def") for x in (r.body.get("removed") or [])]
                    with self._spawned_lock:
                        self._spawned.difference_update(x for x in removed if x)
                    notes.append(f"de-spawned {len(removed)} entities")
                else:
                    notes.append(f"de-spawn failed: {r.error}")

        if wants_state or wants_time:
            try:
                # /sim/reset restores the engine's authored "__init__" state AND
                # rewinds the clock; the harness offers no way to do one without
                # the other, so both scopes take the same call.
                r = self.client.sim_reset()
            except HarnessUnreachable as exc:
                resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
                return resp
            if not r.ok:
                resp.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
                return resp
            notes.append("restored authored poses and rewound the clock")
            warning = r.body.get("warning")
            if warning:
                notes.append(f"harness warning: {warning}")
            if wants_time and not wants_state:
                notes.append(
                    "note: SCOPE_TIME also restored entity state -- OmniSim's "
                    "reset cannot separate the two"
                )
        resp.result = _ok("; ".join(notes) if notes else "nothing to do for this scope")
        return resp

    # -- entities ---------------------------------------------------------

    @_guarded
    def on_get_entities(self, req, resp):
        found, result = self._select_entities(req.filters)
        if found is None:
            resp.result = result
            return resp
        resp.entities = [e.name for e in found]
        resp.result = _ok()
        return resp

    @_guarded
    def on_get_entities_states(self, req, resp):
        found, result = self._select_entities(req.filters)
        if found is None:
            resp.result = result
            return resp
        resp.entities = [e.name for e in found]
        stamp = self._sim_stamp()
        resp.states = [self._entity_state_msg(e, stamp) for e in found]
        resp.result = _ok()
        return resp

    @_guarded
    def on_get_entity_state(self, req, resp):
        tree, result = self._scene_tree()
        if tree is None:
            resp.result = result
            return resp
        for e in self._entities(tree):
            if e.name == req.entity:
                resp.state = self._entity_state_msg(e)
                resp.result = _ok()
                return resp
        resp.result = _err(Result.RESULT_NOT_FOUND, f"no entity named {req.entity!r}")
        return resp

    @_guarded
    def on_set_entity_state(self, req, resp):
        if req.set_twist or req.set_acceleration:
            resp.result = _err(
                Result.RESULT_FEATURE_UNSUPPORTED,
                "OmniSim's harness can set pose but not twist or acceleration; "
                "call again with set_pose only",
            )
            return resp
        if not req.set_pose:
            resp.result = _ok("nothing requested (set_pose is false)")
            return resp
        p = req.state.pose.position
        q = req.state.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        body: dict[str, Any] = {
            "def": req.entity,
            "translation": [float(p.x), float(p.y), float(p.z)],
        }
        if is_valid_quaternion(quat):
            body["rotation"] = quaternion_to_axis_angle(quat)
        try:
            r = self.client.scene_set_pose(body)
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        if not r.ok:
            if r.code == "DEF_NOT_FOUND" or r.status == 404:
                resp.result = _err(Result.RESULT_NOT_FOUND, r.error)
            elif r.code == "FIELD_NOT_ON_NODE":
                resp.result = _err(
                    SetEntityState.Response.INVALID_POSE,
                    f"{r.error} (this node type has no translation/rotation field)",
                )
            else:
                resp.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
            return resp
        # The harness reports the achieved world position; pass it on so the
        # caller can see what actually happened rather than trusting the request.
        resp.result = _ok(f"placed at world position {r.body.get('position')}")
        return resp

    @_guarded
    def on_get_entity_info(self, req, resp):
        tree, result = self._scene_tree()
        if tree is None:
            resp.result = result
            return resp
        match = next(
            (e for e in self._entities(tree) if e.name == req.entity), None
        )
        if match is None:
            resp.result = _err(Result.RESULT_NOT_FOUND, f"no entity named {req.entity!r}")
            return resp
        detail = None
        if match.addressable:
            try:
                r = self.client.scene_node(match.name)
                detail = r.body if r.ok else None
            except HarnessUnreachable as exc:
                resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
                return resp
        info = EntityInfo()
        cat = EntityCategory()
        cat.category = ent.category_for(match.node_type, detail)
        info.category = cat
        info.tags = match.tags
        controller = ((detail or {}).get("fields") or {}).get("controller")
        info.description = (
            f"OmniSim {match.node_type} node, DEF {match.name}"
            + (f", controller {controller!r}" if controller else "")
        )
        resp.info = info
        resp.result = _ok()
        return resp

    @_guarded
    def on_get_entity_bounds(self, req, resp):
        tree, result = self._scene_tree(bounds=True)
        if tree is None:
            resp.result = result
            return resp
        for node in tree.get("nodes") or []:
            if node.get("def") != req.entity:
                continue
            bounds = node.get("bounds")
            if not bounds:
                resp.result = _err(
                    Result.RESULT_OPERATION_FAILED,
                    f"{req.entity!r} has no computable geometric bounds",
                )
                return resp
            lo = bounds.get("bbox_min") or [0.0, 0.0, 0.0]
            hi = bounds.get("bbox_max") or [0.0, 0.0, 0.0]
            # The standard wants bounds relative to the entity's own transform,
            # but the harness computes a WORLD-axis-aligned box. Subtracting the
            # entity origin gives the required relative box exactly when the
            # entity is axis-aligned, and a world-axis-aligned approximation
            # otherwise -- said plainly in the result message rather than hidden.
            ox, oy, oz = position_to_xyz(node.get("position"))
            # Bounds.points is geometry_msgs/Vector3[], not Point[].
            from geometry_msgs.msg import Vector3

            resp.bounds.type = Bounds.TYPE_BOX
            resp.bounds.points = [
                Vector3(x=float(lo[0]) - ox, y=float(lo[1]) - oy, z=float(lo[2]) - oz),
                Vector3(x=float(hi[0]) - ox, y=float(hi[1]) - oy, z=float(hi[2]) - oz),
            ]
            resp.result = _ok(
                ("exact" if bounds.get("exact") else "approximate")
                + " world-axis-aligned box, expressed relative to the entity origin"
            )
            return resp
        resp.result = _err(Result.RESULT_NOT_FOUND, f"no entity named {req.entity!r}")
        return resp

    @_guarded
    def on_spawn(self, req, resp):
        uri = req.entity_resource.uri or ""
        resource_string = req.entity_resource.resource_string or ""
        if not uri and not resource_string:
            resp.result = _err(
                SpawnEntity.Response.NO_RESOURCE,
                "provide entity_resource.uri or entity_resource.resource_string",
            )
            return resp
        fmt = ent.spawn_format_for(uri, resource_string)
        if not fmt:
            resp.result = _err(
                SpawnEntity.Response.UNSUPPORTED_FORMAT,
                f"unsupported resource {uri!r}; OmniSim spawns 'vrml' "
                f"(as resource_string) or 'urdf' (as a .urdf uri)",
            )
            return resp
        name = (req.name or "").strip()
        if not name and not req.allow_renaming:
            resp.result = _err(
                SpawnEntity.Response.NAME_INVALID,
                "name is required unless allow_renaming is true",
            )
            return resp

        p = req.initial_pose.pose.position
        q = req.initial_pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        rotation = quaternion_to_axis_angle(quat) if is_valid_quaternion(quat) else [0.0, 0.0, 1.0, 0.0]
        # OmniSim DEFs are uppercase by convention and must be unique; a ROS
        # caller's name is used verbatim so the two namespaces stay identical.
        body = ent.compose_spawn_body(name, uri, resource_string, (p.x, p.y, p.z), rotation)
        try:
            r = self.client.scene_spawn(body)
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        if not r.ok:
            if r.code == "DEF_TAKEN":
                resp.result = _err(SpawnEntity.Response.NAME_NOT_UNIQUE, r.error)
            elif r.code in ("SPAWN_REJECTED", "SPAWN_SPEC_INVALID"):
                diag = "; ".join(
                    str(d.get("message")) for d in (r.body.get("engine_diagnostics") or [])
                )
                resp.result = _err(
                    SpawnEntity.Response.RESOURCE_PARSE_ERROR,
                    f"{r.error}{'; ' + diag if diag else ''}",
                )
            else:
                resp.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
            return resp
        spawned = str(r.body.get("def") or name)
        with self._spawned_lock:
            self._spawned.add(spawned)
        resp.entity_name = spawned
        resp.result = _ok(f"spawned at {r.body.get('position')}")
        return resp

    @_guarded
    def on_delete(self, req, resp):
        try:
            r = self.client.scene_delete([req.entity])
        except HarnessUnreachable as exc:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, str(exc))
            return resp
        if not r.ok:
            resp.result = _err(Result.RESULT_OPERATION_FAILED, r.error)
            return resp
        if req.entity in (r.body.get("missing") or []):
            resp.result = _err(Result.RESULT_NOT_FOUND, f"no entity named {req.entity!r}")
            return resp
        with self._spawned_lock:
            self._spawned.discard(req.entity)
        # Disclosed engine limitation: removing a node from the scene graph does
        # not remove its geometry from the physics model.
        resp.result = _ok(
            "removed from the scene graph; note OmniSim does not remove deleted "
            "geometry from the physics model, so it can still block rays and "
            "contacts until the world is reloaded"
        )
        return resp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationInterfacesNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
