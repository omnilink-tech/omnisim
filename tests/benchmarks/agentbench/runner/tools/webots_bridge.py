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

"""Webots ``shell+tools``: shell + the upstream R2025a Supervisor/Robot API.

Upstream Webots publishes no packaged tool surface, so the plan
(agent-edge-validation-plan.md 2.2) defines its ``shell+tools`` condition as a
bridge **we** author -- and 5.9 names the incentive problem that creates: a
subtly lame bridge manufactures the comparative win, a subtly chatty one
manufactures the cost win. The mechanical countermeasures, implemented here:

* **Completeness.** One tool per function of the published Supervisor and
  Robot (device-generic) function reference of upstream R2025a, mechanically
  enumerated from the local controller package (this repo is a Webots fork;
  the surface is upstream's). The canonical table lives in
  ``adapters/webots/bridge_controller.py`` (:data:`FUNCTIONS`) so the
  runner-side tools and the controller-side server cannot diverge, and
  ``test_webots_bridge.py`` asserts every public method of the controller
  package's Supervisor/Robot/Node/Field/Proto/Device classes is either
  wrapped or justified in ``adapters/webots/EXCLUSIONS.md``. Exclusions are
  countersigned at pre-registration (plan 2.2).
* **Fidelity.** Tool names are upstream's own C function-index names
  (``wb_supervisor_node_get_contact_points``, ...); descriptions follow the
  upstream reference manual's wording for each function. A final cross-check
  of every name and description against cyberbotics.com's published R2025a
  reference is a review-window item (plan V5 / SPEC 6.2.3).
* **No invented composites.** The bridge adds no batch verbs upstream does
  not publish (no scene-tree aggregate, no multi-get). The granularity guard
  in plan 2.2 is enforced by the scripted oracles, not by us padding or
  slimming the surface.

Handlers speak newline-delimited JSON over TCP to the bridge controller (an
extern Supervisor controller running inside the Webots process -- see
``adapters/webots/bridge_controller.py`` for the protocol and error codes).
The API is handle-based, exactly like upstream's C reference: functions
return integer handles for nodes/fields/protos/devices and take them back as
arguments. Simulated time advances only when the agent calls
``wb_robot_step`` (or ``step_begin``/``step_end``).

Like the OmniSim condition, the bridge controller is **not** pre-started for
authoring tasks where starting the simulator is part of the job; the tools
report plainly when nothing is listening.
"""

from __future__ import annotations

import json
import os
import socket

from agentbench.adapters.webots.bridge_controller import (FUNCTIONS,
                                                          PROTOCOL,
                                                          _VEC_LEN)
from agentbench.runner.tools import ToolResult
from agentbench.runner.tools.manifest import ToolSet, ToolSpec

MODEL_TEXT_LIMIT = 20_000

# What each table kind looks like in JSON Schema. Handle kinds are integers
# (upstream's refs/tags), vectors are fixed-length number arrays.
_KIND_SCHEMA = {
    "int": {"type": "integer"},
    "float": {"type": "number"},
    "str": {"type": "string"},
    "bool": {"type": "boolean"},
}
for _k, _n in _VEC_LEN.items():
    _KIND_SCHEMA[_k] = {"type": "array", "items": {"type": "number"},
                        "minItems": _n, "maxItems": _n}
for _k in ("node", "field", "proto", "device"):
    _KIND_SCHEMA[_k] = {"type": "integer"}

# Per-argument description snippets, keyed (kind, name) with a kind fallback.
_ARG_DESC = {
    ("node", "node"): "node handle (from a wb_supervisor_node_get_* tool)",
    ("node", "from_node"): "node handle of the reference frame node",
    ("field", "field"): "field handle (from a wb_supervisor_node_get_field* "
                        "or wb_supervisor_proto_get_field* tool)",
    ("proto", "proto"): "proto handle (from wb_supervisor_node_get_proto or "
                        "wb_supervisor_proto_get_parent)",
    ("device", "device"): "device handle (from wb_robot_get_device or "
                          "wb_robot_get_device_by_index)",
    ("int", "index"): "0-based index",
    ("int", "sampling_period"): "sampling period in milliseconds",
    ("int", "time_step"): "duration in milliseconds; defaults to the "
                          "world's basicTimeStep",
}
_KIND_DESC = {
    "vec2": "array of 2 numbers [x, y]",
    "vec3": "array of 3 numbers [x, y, z]",
    "vec4": "rotation as 4 numbers [x, y, z, angle] (axis-angle, radians)",
    "vec6": "array of 6 numbers",
    "color": "RGB color as 3 numbers, each in [0, 1]",
}


# ---------------------------------------------------------------------------
# Descriptions -- one per wrapped function, following the upstream R2025a
# reference manual's wording (Robot / Supervisor / Device chapters).
# The generated SF/MF families reuse upstream's own repetitive phrasing.
# ---------------------------------------------------------------------------

_SF_VALUE_TEXT = {
    "bool": "boolean",
    "int32": "32-bit integer",
    "float": "floating point",
    "vec2f": "2D vector [x, y]",
    "vec3f": "3D vector [x, y, z]",
    "rotation": "rotation [x, y, z, angle] (axis-angle, angle in radians)",
    "color": "RGB color [r, g, b], each channel in [0, 1]",
    "string": "string",
}

DESCRIPTIONS = {
    # -- Robot --------------------------------------------------------------
    "wb_robot_step":
        "Advance the controller by time_step milliseconds: actuator commands "
        "are sent, the simulation steps forward, and sensor data is read "
        "back. Returns -1 if the simulation is about to terminate, else 0 or "
        "a positive value. Simulated time does not advance unless this (or "
        "wb_robot_step_begin/wb_robot_step_end) is called.",
    "wb_robot_step_begin":
        "Send the actuation commands and start a simulation step of "
        "time_step milliseconds without waiting for it to complete; pair "
        "with wb_robot_step_end. Returns -1 if the simulation is about to "
        "terminate.",
    "wb_robot_step_end":
        "Wait for the simulation step started with wb_robot_step_begin to "
        "complete and read the sensor data back. Returns -1 if the "
        "simulation is about to terminate.",
    "wb_robot_get_device":
        "Get a unique handle to the robot's device with the given name, "
        "usable with the wb_device_* tools. Returns null if the robot has "
        "no device with this name.",
    "wb_robot_get_device_by_index":
        "Get a handle to the device at the given index in the robot's "
        "device list (0 <= index < wb_robot_get_number_of_devices).",
    "wb_robot_get_number_of_devices":
        "Return the number of devices of the robot.",
    "wb_robot_get_basic_time_step":
        "Return the value of the basicTimeStep field of the WorldInfo node, "
        "in milliseconds.",
    "wb_robot_get_name":
        "Return the name defined in the name field of the Robot node. Robot "
        "names are unique within a world.",
    "wb_robot_get_model":
        "Return the model string defined in the model field of the Robot "
        "node.",
    "wb_robot_get_custom_data":
        "Return the string contained in the customData field of the Robot "
        "node.",
    "wb_robot_set_custom_data":
        "Write the given string into the customData field of the Robot "
        "node.",
    "wb_robot_get_project_path":
        "Return the full path of the current project directory.",
    "wb_robot_get_world_path":
        "Return the full path of the currently opened world file (.wbt).",
    "wb_robot_get_supervisor":
        "Return true if the supervisor field of this controller's Robot "
        "node is TRUE, i.e. the wb_supervisor_* functions are available.",
    "wb_robot_get_synchronization":
        "Return the value of the synchronization field of the Robot node.",
    "wb_robot_get_time":
        "Return the current simulation time in seconds.",
    "wb_robot_get_urdf":
        "Return the URDF representation of the robot; link and joint names "
        "are prefixed with the optional prefix string.",
    "wb_robot_get_mode":
        "Return the current operating mode of the controller: 0 = "
        "simulation, 1 = cross-compilation, 2 = remote-control.",
    "wb_robot_set_mode":
        "Set the operating mode of the controller (e.g. switch between "
        "simulation and remote-control); arg is passed to the remote "
        "control plugin.",
    "wb_robot_battery_sensor_enable":
        "Enable battery level measurement, sampled every sampling_period "
        "milliseconds.",
    "wb_robot_battery_sensor_disable":
        "Disable battery level measurement.",
    "wb_robot_battery_sensor_get_sampling_period":
        "Return the sampling period of the battery sensor, in milliseconds.",
    "wb_robot_battery_sensor_get_value":
        "Return the current battery level in joules; requires "
        "wb_robot_battery_sensor_enable to have been called.",
    # -- Device (generic) ---------------------------------------------------
    "wb_device_get_name":
        "Return the name of the device, as defined in its name field.",
    "wb_device_get_model":
        "Return the model string of the device, as defined in its model "
        "field.",
    "wb_device_get_node_type":
        "Return the node type of the device as a WB_NODE_* constant "
        "integer.",
    # -- Supervisor ---------------------------------------------------------
    "wb_supervisor_node_get_root":
        "Get a handle to the root node of the scene tree; its 'children' "
        "field lists every top-level node of the world, and importing into "
        "that field spawns top-level nodes.",
    "wb_supervisor_node_get_self":
        "Get a handle to the Robot node of this supervisor controller "
        "itself.",
    "wb_supervisor_node_get_from_def":
        "Get a handle to the node identified by the given DEF name in the "
        "scene tree; dot-separated DEF paths ('PARENT.CHILD') reach nested "
        "DEF nodes. Returns null if no matching node exists.",
    "wb_supervisor_node_get_from_id":
        "Get a handle to the node with the given unique id (as returned by "
        "wb_supervisor_node_get_id). Returns null if no node has this id.",
    "wb_supervisor_node_get_from_device":
        "Get a handle to the scene-tree node corresponding to the given "
        "device handle.",
    "wb_supervisor_node_get_selected":
        "Get a handle to the node currently selected in the scene tree "
        "view. Returns null if no node is selected.",
    "wb_supervisor_set_label":
        "Display a text label overlaid on the 3D view. id identifies the "
        "label (reusing an id updates that label), x and y are screen "
        "coordinates as fractions in [0, 1], size is the text height as a "
        "fraction of the screen height, color is a 0xRRGGBB integer, "
        "transparency is in [0, 1] (0 = opaque), font is a font family "
        "name.",
    "wb_supervisor_simulation_quit":
        "Terminate the simulator with the given exit status. The bridge "
        "acknowledges this call and then shuts down; no further tool calls "
        "will succeed.",
    "wb_supervisor_simulation_set_mode":
        "Set the simulation mode: 0 = pause, 1 = real-time, 2 = fast (no "
        "rendering).",
    "wb_supervisor_simulation_get_mode":
        "Return the current simulation mode: 0 = pause, 1 = real-time, 2 = "
        "fast.",
    "wb_supervisor_simulation_reset":
        "Reset the simulation to the initial state it had when the world "
        "was loaded: node fields and the simulation time are restored. "
        "Controllers are NOT restarted; use "
        "wb_supervisor_node_restart_controller for that.",
    "wb_supervisor_simulation_reset_physics":
        "Stop the inertia of every solid in the world: all linear and "
        "angular velocities are zeroed.",
    "wb_supervisor_world_load":
        "Load the world given by filename (.wbt), replacing the current "
        "one. Every controller process, this bridge included, is terminated "
        "and restarted by the simulator.",
    "wb_supervisor_world_save":
        "Save the current world; with no filename the currently loaded "
        ".wbt file is overwritten. Returns true on success.",
    "wb_supervisor_world_reload":
        "Reload the current world file, restarting the simulation and "
        "every controller process (this bridge included).",
    "wb_supervisor_export_image":
        "Save the current main-window 3D view to an image file; the format "
        "is chosen by the filename extension (PNG or JPEG), quality applies "
        "to JPEG (1-100).",
    # -- Node ---------------------------------------------------------------
    "wb_supervisor_node_get_def":
        "Return the DEF name of the node, or an empty string if it has "
        "none.",
    "wb_supervisor_node_get_id":
        "Return the unique id of the node (resolvable back with "
        "wb_supervisor_node_get_from_id).",
    "wb_supervisor_node_get_parent_node":
        "Get a handle to the parent node. Returns null for the root node.",
    "wb_supervisor_node_is_proto":
        "Return true if the node is a PROTO instance.",
    "wb_supervisor_node_get_proto":
        "Get a handle to the PROTO of the node, usable with the "
        "wb_supervisor_proto_* tools. Returns null if the node is not a "
        "PROTO instance.",
    "wb_supervisor_node_get_from_proto_def":
        "Get a handle to the node with the given DEF name inside the "
        "internal (hidden) subtree of this PROTO instance. Returns null if "
        "no matching node exists.",
    "wb_supervisor_node_get_type":
        "Return the type of the node as a WB_NODE_* constant integer.",
    "wb_supervisor_node_get_type_name":
        "Return the node's type name as displayed in the scene tree (the "
        "PROTO name for PROTO instances, e.g. 'Pioneer3at').",
    "wb_supervisor_node_get_base_type_name":
        "Return the base node type name of the node (e.g. 'Robot' for a "
        "PROTO whose base type is Robot).",
    "wb_supervisor_node_remove":
        "Remove the node from the scene tree.",
    "wb_supervisor_node_export_string":
        "Export the node and its subtree as a .wbt-syntax node string, "
        "suitable for re-import with the import-from-string tools.",
    "wb_supervisor_node_get_field":
        "Get a handle to the node's field with the given name. Returns "
        "null if the node has no such field.",
    "wb_supervisor_node_get_field_by_index":
        "Get a handle to the node's field at the given index (0 <= index < "
        "wb_supervisor_node_get_number_of_fields).",
    "wb_supervisor_node_get_number_of_fields":
        "Return the number of fields of the node.",
    "wb_supervisor_node_get_base_node_field":
        "Get a handle to a field of the underlying base node of a PROTO "
        "instance by name (bypassing the PROTO interface). Returns null if "
        "there is no such field.",
    "wb_supervisor_node_get_base_node_field_by_index":
        "Get a handle to a field of the underlying base node of a PROTO "
        "instance by index.",
    "wb_supervisor_node_get_number_of_base_node_fields":
        "Return the number of fields of the underlying base node of a "
        "PROTO instance.",
    "wb_supervisor_node_get_position":
        "Return the position [x, y, z] of the node's origin, expressed in "
        "the global (world) coordinate system, in meters. The node must "
        "derive from Pose (e.g. Solid, Robot).",
    "wb_supervisor_node_get_orientation":
        "Return the 3x3 rotation matrix of the node (9 values, row-major) "
        "expressing its orientation in the global coordinate system.",
    "wb_supervisor_node_get_pose":
        "Return the 4x4 transformation matrix of the node (16 values, "
        "row-major) expressed relative to from_node, or in the global "
        "coordinate system if from_node is omitted.",
    "wb_supervisor_node_enable_pose_tracking":
        "Force the simulator to stream this node's pose to the controller "
        "every sampling_period milliseconds, making subsequent pose reads "
        "cheap.",
    "wb_supervisor_node_disable_pose_tracking":
        "Stop the pose tracking started with "
        "wb_supervisor_node_enable_pose_tracking.",
    "wb_supervisor_node_get_center_of_mass":
        "Return the position [x, y, z] of the center of mass of a Solid "
        "node, in the global coordinate system.",
    "wb_supervisor_node_get_contact_points":
        "Return the list of current contact points of a Solid node; with "
        "include_descendants true, contacts of descendant solids are "
        "included. Each contact is {point: [x, y, z] in the global "
        "coordinate system, node_id: id of the other contacted solid, "
        "depth: penetration depth in meters}.",
    "wb_supervisor_node_enable_contact_points_tracking":
        "Force the simulator to stream this node's contact points to the "
        "controller every sampling_period milliseconds.",
    "wb_supervisor_node_disable_contact_points_tracking":
        "Stop the contact points tracking started with "
        "wb_supervisor_node_enable_contact_points_tracking.",
    "wb_supervisor_node_get_static_balance":
        "Return the boolean static balance measure of a Solid node: true "
        "if the projection of its center of mass lies inside the support "
        "polygon of its ground contact points.",
    "wb_supervisor_node_get_velocity":
        "Return the 6 velocity components [vx, vy, vz, wx, wy, wz] of a "
        "Solid node: linear velocity in m/s and angular velocity in rad/s, "
        "both in the global coordinate system.",
    "wb_supervisor_node_set_velocity":
        "Set the 6 velocity components [vx, vy, vz, wx, wy, wz] of a Solid "
        "node: linear velocity in m/s and angular velocity in rad/s, both "
        "in the global coordinate system.",
    "wb_supervisor_node_save_state":
        "Save the current state of the node and its descendants under the "
        "given state name, restorable later with "
        "wb_supervisor_node_load_state.",
    "wb_supervisor_node_load_state":
        "Restore the state of the node previously saved under the given "
        "state name ('__init__' is the state at world load).",
    "wb_supervisor_node_reset_physics":
        "Stop the inertia of the node and its descendants: linear and "
        "angular velocities are zeroed.",
    "wb_supervisor_node_set_joint_position":
        "Set the position of a joint node (HingeJoint, Hinge2Joint, "
        "SliderJoint, BallJoint) instantaneously; index selects the axis "
        "for multi-axis joints (1 by default, 2 or 3 for Hinge2Joint and "
        "BallJoint).",
    "wb_supervisor_node_restart_controller":
        "Restart the controller of the given Robot node.",
    "wb_supervisor_node_move_viewpoint":
        "Move the viewpoint so that the given node is visible, as the "
        "'move viewpoint to object' menu action does.",
    "wb_supervisor_node_set_visibility":
        "Set whether the node is rendered from the point of view of "
        "from_node (a Viewpoint, Camera, Lidar or RangeFinder node); "
        "physics is unaffected.",
    "wb_supervisor_node_add_force":
        "Add a force [fx, fy, fz] in newtons to a Solid node at its center "
        "of mass for the current time step; if relative is true the force "
        "is expressed in the node's frame, otherwise in the global frame.",
    "wb_supervisor_node_add_force_with_offset":
        "Add a force [fx, fy, fz] in newtons to a Solid node, applied at "
        "the point given by offset [x, y, z] expressed in the node's "
        "frame; if relative is true the force is expressed in the node's "
        "frame, otherwise in the global frame.",
    "wb_supervisor_node_add_torque":
        "Add a torque [tx, ty, tz] in newton-meters to a Solid node for "
        "the current time step; if relative is true the torque is "
        "expressed in the node's frame, otherwise in the global frame.",
    # -- Field --------------------------------------------------------------
    "wb_supervisor_field_get_name":
        "Return the name of the field.",
    "wb_supervisor_field_get_type":
        "Return the type of the field as a WB_SF_* / WB_MF_* constant "
        "integer.",
    "wb_supervisor_field_get_type_name":
        "Return the type name of the field as a string (e.g. 'SFVec3f', "
        "'MFNode').",
    "wb_supervisor_field_get_count":
        "Return the number of items of a multiple (MF) field, or -1 if the "
        "field is a single (SF) field.",
    "wb_supervisor_field_get_actual_field":
        "For a field of a PROTO instance, get a handle to the internal "
        "base-node field it is bound to through IS statements. Returns "
        "null if the field is not bound.",
    "wb_supervisor_field_enable_sf_tracking":
        "Force the simulator to stream this field's value to the "
        "controller every sampling_period milliseconds, making subsequent "
        "reads cheap.",
    "wb_supervisor_field_disable_sf_tracking":
        "Stop the field tracking started with "
        "wb_supervisor_field_enable_sf_tracking.",
    "wb_supervisor_field_get_sf_node":
        "Return the node handle stored in an SF_NODE field, or null if the "
        "field is empty.",
    "wb_supervisor_field_get_mf_node":
        "Return the node handle at the given index of an MF_NODE field.",
    "wb_supervisor_field_remove_mf":
        "Remove the item at the given index from a multiple (MF) field.",
    "wb_supervisor_field_remove_sf":
        "Remove the node stored in an SF_NODE field.",
    "wb_supervisor_field_import_mf_node_from_string":
        "Import a node into an MF_NODE field (typically a 'children' "
        "field, e.g. the root node's, which spawns it into the world), "
        "constructed from a .wbt-syntax node string, inserted at the given "
        "position; negative positions count from the end (-1 appends). "
        "This is the supervisor's spawn-from-string verb.",
    "wb_supervisor_field_import_sf_node_from_string":
        "Import a node into an empty SF_NODE field, constructed from a "
        ".wbt-syntax node string.",
    # -- Proto --------------------------------------------------------------
    "wb_supervisor_proto_get_type_name":
        "Return the name of the PROTO.",
    "wb_supervisor_proto_get_parent":
        "Get a handle to the parent PROTO if this PROTO is derived from "
        "another PROTO. Returns null otherwise.",
    "wb_supervisor_proto_get_field":
        "Get a handle to the PROTO interface field with the given name. "
        "Returns null if the PROTO has no such field.",
    "wb_supervisor_proto_get_field_by_index":
        "Get a handle to the PROTO interface field at the given index.",
    "wb_supervisor_proto_get_number_of_fields":
        "Return the number of fields of the PROTO interface.",
    "wb_supervisor_proto_is_derived":
        "Return true if the PROTO is derived from another PROTO.",
}

# Generated families, phrased the way the upstream reference phrases them.
for _wire, _text in _SF_VALUE_TEXT.items():
    DESCRIPTIONS["wb_supervisor_field_get_sf_%s" % _wire] = (
        "Return the %s value of an SF_%s field." % (_text, _wire.upper()))
    DESCRIPTIONS["wb_supervisor_field_get_mf_%s" % _wire] = (
        "Return the %s value at the given index of an MF_%s field."
        % (_text, _wire.upper()))
    DESCRIPTIONS["wb_supervisor_field_set_sf_%s" % _wire] = (
        "Set the %s value of an SF_%s field." % (_text, _wire.upper()))
    DESCRIPTIONS["wb_supervisor_field_set_mf_%s" % _wire] = (
        "Set the %s value at the given index of an MF_%s field."
        % (_text, _wire.upper()))
    DESCRIPTIONS["wb_supervisor_field_insert_mf_%s" % _wire] = (
        "Insert a new %s value at the given index into an MF_%s field; "
        "negative indices count from the end (-1 appends)."
        % (_text, _wire.upper()))

TOOL_SOURCE = ("webots-r2025a-reference/v1 (names from upstream's function "
               "index via the local fork's controller package; descriptions "
               "follow the upstream reference manual; cyberbotics.com "
               "cross-check pending the V5 review window)")


def _arg_description(kind, name):
    got = _ARG_DESC.get((kind, name))
    if got:
        return got
    got = _KIND_DESC.get(kind)
    if got:
        return got
    return name.replace("_", " ")


def input_schema_for(fn) -> dict:
    """JSON schema for one Fn from the canonical table."""
    properties = {}
    required = []
    for name, kind, req in fn.args:
        schema = dict(_KIND_SCHEMA[kind])
        schema["description"] = _arg_description(kind, name)
        properties[name] = schema
        if req:
            required.append(name)
    out = {"type": "object", "properties": properties}
    if required:
        out["required"] = required
    return out


def tool_definitions() -> list[dict]:
    """One model-facing definition per wrapped upstream function."""
    defs = []
    for name, fn in FUNCTIONS.items():
        desc = DESCRIPTIONS.get(name)
        if not desc:
            raise RuntimeError("no description for bridged function %r; the "
                               "fidelity rule (plan 2.2) forbids shipping an "
                               "undescribed tool" % name)
        defs.append({"name": name, "description": desc,
                     "input_schema": input_schema_for(fn)})
    return defs


# WRAPPED_API: tool name -> "Class.method" in the local controller package.
# This is what the completeness test checks against the enumerated public API.
WRAPPED_API = {name: "%s.%s" % (fn.cls, fn.method)
               for name, fn in FUNCTIONS.items()}


# ---------------------------------------------------------------------------
# Handlers: one TCP round trip per call to the bridge controller.
# ---------------------------------------------------------------------------

def bridge_endpoint(sandbox):
    """(host, port) of this run's bridge controller.

    Priority: an explicit ``webots_bridge_port`` attribute on the sandbox
    (the Phase W launcher's job), the ``AGENTBENCH_WEBOTS_BRIDGE_PORT`` env
    var, then the run's assigned harness port -- in the Webots cell nothing
    else uses that reservation, so it doubles as the bridge port.
    """
    port = getattr(sandbox, "webots_bridge_port", 0) or 0
    if not port:
        try:
            port = int(os.environ.get("AGENTBENCH_WEBOTS_BRIDGE_PORT", "0"))
        except ValueError:
            port = 0
    if not port:
        port = getattr(sandbox, "harness_port", 0) or 0
    return ("127.0.0.1", int(port))


def call_bridge(host, port, fn_name, args, timeout_s) -> dict:
    """One request/response round trip. Raises OSError on transport failure."""
    payload = json.dumps({"fn": fn_name, "args": args or {}},
                         ensure_ascii=False) + "\n"
    with socket.create_connection((host, port),
                                  timeout=max(1.0, float(timeout_s))) as conn:
        conn.sendall(payload.encode("utf-8"))
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = conn.recv(65536)
            if not chunk:
                break
            buf += chunk
    if not buf.strip():
        raise OSError("the bridge closed the connection without replying")
    return json.loads(buf.decode("utf-8"))


def _clip(text, limit=MODEL_TEXT_LIMIT):
    if len(text) <= limit:
        return text
    return ("%s\n...[%d characters omitted from this view; the full result "
            "is in the run trace]..." % (text[: limit - 200],
                                         len(text) - limit + 200))


def _make_handler(fn_name, sandbox):
    def handler(args, timeout_s):
        host, port = bridge_endpoint(sandbox)
        if not port:
            return ToolResult(
                text="error: no bridge port is configured for this run "
                     "(sandbox.webots_bridge_port / "
                     "AGENTBENCH_WEBOTS_BRIDGE_PORT / harness_port are all "
                     "unset)", is_error=True)
        try:
            response = call_bridge(host, port, fn_name, dict(args or {}),
                                   timeout_s)
        except (OSError, ValueError) as exc:
            return ToolResult(
                text="error: cannot reach the Webots bridge controller on "
                     "%s:%d (%s). It runs as an extern Supervisor controller "
                     "inside the Webots process; if the simulator or the "
                     "bridge is not running yet, start it first."
                     % (host, port, exc),
                data={"transport_error": str(exc)}, is_error=True)
        if response.get("ok"):
            text = json.dumps(response.get("result"), ensure_ascii=False,
                              default=str)
            return ToolResult(text=_clip(text), data=response)
        error = response.get("error") or {}
        return ToolResult(
            text="error [%s]: %s" % (error.get("code", "UNKNOWN"),
                                     error.get("message", "")),
            data=response, is_error=True)
    return handler


def build(sandbox, base: ToolSet | None = None) -> ToolSet:
    """``base`` (the shell set) + one tool per wrapped upstream function."""
    if base is None:
        from agentbench.runner.tools import shell as shell_mod
        base = shell_mod.build(sandbox)
    specs = list(base.specs)
    for definition in tool_definitions():
        specs.append(ToolSpec(
            name=definition["name"],
            description=definition["description"],
            input_schema=definition["input_schema"],
            handler=_make_handler(definition["name"], sandbox),
            source=TOOL_SOURCE))
    return ToolSet(
        id="shell_plus_tools",
        description=("The shell baseline plus upstream Webots R2025a's "
                     "published Supervisor and Robot (device-generic) "
                     "function reference, wrapped one-tool-per-function by "
                     "the AgentBench Webots bridge (plan 2.2; %s)"
                     % PROTOCOL),
        specs=specs,
        env_policy=sandbox.env_policy(),
        notes=list(base.notes) + [
            "Tool names and argument shapes are upstream R2025a's own "
            "function reference, enumerated mechanically from the "
            "controller package this fork inherited; every unwrapped "
            "public function is justified in "
            "adapters/webots/EXCLUSIONS.md (completeness rule, plan 2.2). "
            "A cross-check against cyberbotics.com's published R2025a "
            "reference is a V5 review-window item.",
            "No batch or composite verbs were added: the bridge is "
            "faithful function-for-function; upstream publishes no "
            "aggregate scene read and none was invented for it.",
            "The API is handle-based like upstream's C reference: tools "
            "return integer node/field/proto/device handles and take them "
            "as arguments. Handles are only valid for the current bridge "
            "process.",
            "Simulated time advances ONLY via wb_robot_step / "
            "wb_robot_step_begin + wb_robot_step_end; between calls the "
            "simulation waits on the bridge (synchronous controller).",
            # Deliberately no port number here: the manifest hash must be
            # stable across runs; the per-run endpoint is in the trace's
            # sandbox block.
            "The bridge controller is NOT necessarily pre-started: where "
            "starting the simulator is part of the task, these tools "
            "report that nothing is listening until the agent starts it.",
        ])
