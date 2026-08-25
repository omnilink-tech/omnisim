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

"""The Webots ``shell+tools`` bridge controller (Phase W, plan 2.2).

An **extern Supervisor controller** for upstream Webots R2025a that serves the
entire wrapped Supervisor + Robot function surface as a tiny JSON-over-TCP
request/response loop, so the AgentBench runner can expose each function as one
tool definition (``runner/tools/webots_bridge.py``).

Why this file is written the way it is:

* **It must run under upstream Webots' own Python**, inside WSL2 or a
  container, with nothing but the standard library and upstream's
  ``controller`` package available. Nothing here may import ``agentbench``;
  the ``controller`` import happens lazily inside :func:`main` so that the
  runner-side module and the unit tests can import *this* module (for the
  canonical :data:`FUNCTIONS` table) with no Webots installed.
* **The function table lives here, once.** ``webots_bridge.py`` imports
  :data:`FUNCTIONS` and adds descriptions/schemas; a unit test asserts the two
  sides agree, and the completeness test asserts the table covers the whole
  public Supervisor/Robot API of the controller package (or that the gap is
  justified in ``EXCLUSIONS.md``). Curation-by-omission is the abuse the
  plan's completeness rule exists to catch, so the table is enumerated from
  the API, not from what a task happens to need.
* **The API is handle-based** (upstream's C reference passes ``WbNodeRef`` /
  ``WbFieldRef`` / ``WbProtoRef`` / ``WbDeviceTag``), so the bridge mints
  integer handles for Node / Field / Proto / Device values and resolves them
  on the way back in. Unknown handles and wrong-kind handles are structured
  errors, never crashes.
* **Simulated time advances only through ``wb_robot_step`` /
  ``wb_robot_step_begin``/``wb_robot_step_end``** -- the controller is
  synchronous, so between requests the simulation waits. That is the correct
  benchmark semantics: the agent owns the clock.

Wire protocol (newline-delimited JSON over TCP, one server, sequential
clients; a client may send many requests per connection)::

    -> {"fn": "wb_supervisor_node_get_from_def", "args": {"def": "FLOOR"}}
    <- {"ok": true, "result": {"node": 1}}
    -> {"fn": "wb_robot_step", "args": {"time_step": 32}}
    <- {"ok": true, "result": 0}
    -> {"fn": "nope", "args": {}}
    <- {"ok": false, "error": {"code": "UNKNOWN_FUNCTION", "message": "..."}}

Error codes: ``BAD_REQUEST`` (not JSON / not an object), ``UNKNOWN_FUNCTION``,
``MISSING_ARGUMENT``, ``BAD_ARGUMENT``, ``UNKNOWN_HANDLE``,
``BAD_HANDLE_TYPE``, ``EXCEPTION`` (the wrapped call raised; message + class
name returned, the controller survives).

``wb_supervisor_simulation_quit`` is answered *before* the bridge shuts its
server down, so the agent sees the acknowledgement.

Standalone launch (the Phase W launcher owns the real invocation)::

    WEBOTS_CONTROLLER_URL=tcp://127.0.0.1:1234/<robot-name> \
    python3 bridge_controller.py --port 6799
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import socket
import sys
import traceback

PROTOCOL = "agentbench/webots_bridge/v1"

# --------------------------------------------------------------------------
# The canonical function table.
#
# Fn.name   -- the upstream R2025a C-reference function name (the tool name).
# Fn.cls    -- the owning class in lib/controller/python/controller (the
#              completeness test maps enumerated public methods to this).
# Fn.method -- the Python method of that class the bridge calls.
# Fn.target -- what the method is called on: "robot" (the Supervisor
#              instance; Supervisor extends Robot) or a handle kind.
# Fn.args   -- ordered (name, kind, required). Optional args are trailing,
#              matching the Python signatures' defaulted parameters.
#
# Kinds: node/field/proto/device (integer handles), int, float, str, bool,
#        vec2, vec3, vec4 (rotation), vec6, color.
# --------------------------------------------------------------------------

Fn = collections.namedtuple("Fn", "name cls target method args")

_LIST: list[Fn] = []


def _f(name, cls, target, method, *args):
    _LIST.append(Fn(name, cls, target, method, tuple(args)))


# -- Robot (called on the Supervisor instance; Supervisor extends Robot) ----
_f("wb_robot_step", "Robot", "robot", "step", ("time_step", "int", False))
_f("wb_robot_step_begin", "Robot", "robot", "stepBegin",
   ("time_step", "int", False))
_f("wb_robot_step_end", "Robot", "robot", "stepEnd")
_f("wb_robot_get_device", "Robot", "robot", "getDevice", ("name", "str", True))
_f("wb_robot_get_device_by_index", "Robot", "robot", "getDeviceByIndex",
   ("index", "int", True))
_f("wb_robot_get_number_of_devices", "Robot", "robot", "getNumberOfDevices")
_f("wb_robot_get_basic_time_step", "Robot", "robot", "getBasicTimeStep")
_f("wb_robot_get_name", "Robot", "robot", "getName")
_f("wb_robot_get_model", "Robot", "robot", "getModel")
_f("wb_robot_get_custom_data", "Robot", "robot", "getCustomData")
_f("wb_robot_set_custom_data", "Robot", "robot", "setCustomData",
   ("data", "str", True))
_f("wb_robot_get_project_path", "Robot", "robot", "getProjectPath")
_f("wb_robot_get_world_path", "Robot", "robot", "getWorldPath")
_f("wb_robot_get_supervisor", "Robot", "robot", "getSupervisor")
_f("wb_robot_get_synchronization", "Robot", "robot", "getSynchronization")
_f("wb_robot_get_time", "Robot", "robot", "getTime")
_f("wb_robot_get_urdf", "Robot", "robot", "getUrdf",
   ("prefix", "str", False))
_f("wb_robot_get_mode", "Robot", "robot", "getMode")
_f("wb_robot_set_mode", "Robot", "robot", "setMode",
   ("mode", "int", True), ("arg", "str", True))
_f("wb_robot_battery_sensor_enable", "Robot", "robot", "batterySensorEnable",
   ("sampling_period", "int", True))
_f("wb_robot_battery_sensor_disable", "Robot", "robot",
   "batterySensorDisable")
_f("wb_robot_battery_sensor_get_sampling_period", "Robot", "robot",
   "batterySensorGetSamplingPeriod")
_f("wb_robot_battery_sensor_get_value", "Robot", "robot",
   "batterySensorGetValue")

# -- Device (generic device functions; handle from wb_robot_get_device) -----
_f("wb_device_get_name", "Device", "device", "getName",
   ("device", "device", True))
_f("wb_device_get_model", "Device", "device", "getModel",
   ("device", "device", True))
_f("wb_device_get_node_type", "Device", "device", "getNodeType",
   ("device", "device", True))

# -- Supervisor -------------------------------------------------------------
_f("wb_supervisor_node_get_root", "Supervisor", "robot", "getRoot")
_f("wb_supervisor_node_get_self", "Supervisor", "robot", "getSelf")
_f("wb_supervisor_node_get_from_def", "Supervisor", "robot", "getFromDef",
   ("def", "str", True))
_f("wb_supervisor_node_get_from_id", "Supervisor", "robot", "getFromId",
   ("id", "int", True))
_f("wb_supervisor_node_get_from_device", "Supervisor", "robot",
   "getFromDevice", ("device", "device", True))
_f("wb_supervisor_node_get_selected", "Supervisor", "robot", "getSelected")
_f("wb_supervisor_set_label", "Supervisor", "robot", "setLabel",
   ("id", "int", True), ("label", "str", True), ("x", "float", True),
   ("y", "float", True), ("size", "float", True), ("color", "int", True),
   ("transparency", "float", False), ("font", "str", False))
_f("wb_supervisor_simulation_quit", "Supervisor", "robot", "simulationQuit",
   ("status", "int", True))
_f("wb_supervisor_simulation_set_mode", "Supervisor", "robot",
   "simulationSetMode", ("mode", "int", True))
_f("wb_supervisor_simulation_get_mode", "Supervisor", "robot",
   "simulationGetMode")
_f("wb_supervisor_simulation_reset", "Supervisor", "robot",
   "simulationReset")
_f("wb_supervisor_simulation_reset_physics", "Supervisor", "robot",
   "simulationResetPhysics")
_f("wb_supervisor_world_load", "Supervisor", "robot", "worldLoad",
   ("filename", "str", True))
_f("wb_supervisor_world_save", "Supervisor", "robot", "worldSave",
   ("filename", "str", False))
_f("wb_supervisor_world_reload", "Supervisor", "robot", "worldReload")
_f("wb_supervisor_export_image", "Supervisor", "robot", "exportImage",
   ("filename", "str", True), ("quality", "int", True))

# -- Node -------------------------------------------------------------------
_f("wb_supervisor_node_get_def", "Node", "node", "getDef",
   ("node", "node", True))
_f("wb_supervisor_node_get_id", "Node", "node", "getId",
   ("node", "node", True))
_f("wb_supervisor_node_get_parent_node", "Node", "node", "getParentNode",
   ("node", "node", True))
_f("wb_supervisor_node_is_proto", "Node", "node", "isProto",
   ("node", "node", True))
_f("wb_supervisor_node_get_proto", "Node", "node", "getProto",
   ("node", "node", True))
_f("wb_supervisor_node_get_from_proto_def", "Node", "node", "getFromProtoDef",
   ("node", "node", True), ("def", "str", True))
_f("wb_supervisor_node_get_type", "Node", "node", "getType",
   ("node", "node", True))
_f("wb_supervisor_node_get_type_name", "Node", "node", "getTypeName",
   ("node", "node", True))
_f("wb_supervisor_node_get_base_type_name", "Node", "node", "getBaseTypeName",
   ("node", "node", True))
_f("wb_supervisor_node_remove", "Node", "node", "remove",
   ("node", "node", True))
_f("wb_supervisor_node_export_string", "Node", "node", "exportString",
   ("node", "node", True))
_f("wb_supervisor_node_get_field", "Node", "node", "getField",
   ("node", "node", True), ("name", "str", True))
_f("wb_supervisor_node_get_field_by_index", "Node", "node", "getFieldByIndex",
   ("node", "node", True), ("index", "int", True))
_f("wb_supervisor_node_get_number_of_fields", "Node", "node",
   "getNumberOfFields", ("node", "node", True))
_f("wb_supervisor_node_get_base_node_field", "Node", "node",
   "getBaseNodeField", ("node", "node", True), ("name", "str", True))
_f("wb_supervisor_node_get_base_node_field_by_index", "Node", "node",
   "getBaseNodeFieldByIndex", ("node", "node", True), ("index", "int", True))
_f("wb_supervisor_node_get_number_of_base_node_fields", "Node", "node",
   "getNumberOfBaseNodeFields", ("node", "node", True))
_f("wb_supervisor_node_get_position", "Node", "node", "getPosition",
   ("node", "node", True))
_f("wb_supervisor_node_get_orientation", "Node", "node", "getOrientation",
   ("node", "node", True))
_f("wb_supervisor_node_get_pose", "Node", "node", "getPose",
   ("node", "node", True), ("from_node", "node", False))
_f("wb_supervisor_node_enable_pose_tracking", "Node", "node",
   "enablePoseTracking", ("node", "node", True),
   ("sampling_period", "int", True), ("from_node", "node", False))
_f("wb_supervisor_node_disable_pose_tracking", "Node", "node",
   "disablePoseTracking", ("node", "node", True), ("from_node", "node", False))
_f("wb_supervisor_node_get_center_of_mass", "Node", "node", "getCenterOfMass",
   ("node", "node", True))
_f("wb_supervisor_node_get_contact_points", "Node", "node",
   "getContactPoints", ("node", "node", True),
   ("include_descendants", "bool", False))
_f("wb_supervisor_node_enable_contact_points_tracking", "Node", "node",
   "enableContactPointsTracking", ("node", "node", True),
   ("sampling_period", "int", True), ("include_descendants", "bool", False))
_f("wb_supervisor_node_disable_contact_points_tracking", "Node", "node",
   "disableContactPointsTracking", ("node", "node", True))
_f("wb_supervisor_node_get_static_balance", "Node", "node",
   "getStaticBalance", ("node", "node", True))
_f("wb_supervisor_node_get_velocity", "Node", "node", "getVelocity",
   ("node", "node", True))
_f("wb_supervisor_node_set_velocity", "Node", "node", "setVelocity",
   ("node", "node", True), ("velocity", "vec6", True))
_f("wb_supervisor_node_save_state", "Node", "node", "saveState",
   ("node", "node", True), ("state_name", "str", True))
_f("wb_supervisor_node_load_state", "Node", "node", "loadState",
   ("node", "node", True), ("state_name", "str", True))
_f("wb_supervisor_node_reset_physics", "Node", "node", "resetPhysics",
   ("node", "node", True))
_f("wb_supervisor_node_set_joint_position", "Node", "node",
   "setJointPosition", ("node", "node", True), ("position", "float", True),
   ("index", "int", False))
_f("wb_supervisor_node_restart_controller", "Node", "node",
   "restartController", ("node", "node", True))
_f("wb_supervisor_node_move_viewpoint", "Node", "node", "moveViewpoint",
   ("node", "node", True))
_f("wb_supervisor_node_set_visibility", "Node", "node", "setVisibility",
   ("node", "node", True), ("from_node", "node", True),
   ("visible", "bool", True))
_f("wb_supervisor_node_add_force", "Node", "node", "addForce",
   ("node", "node", True), ("force", "vec3", True), ("relative", "bool", True))
_f("wb_supervisor_node_add_force_with_offset", "Node", "node",
   "addForceWithOffset", ("node", "node", True), ("force", "vec3", True),
   ("offset", "vec3", True), ("relative", "bool", True))
_f("wb_supervisor_node_add_torque", "Node", "node", "addTorque",
   ("node", "node", True), ("torque", "vec3", True),
   ("relative", "bool", True))

# -- Field ------------------------------------------------------------------
_f("wb_supervisor_field_get_name", "Field", "field", "getName",
   ("field", "field", True))
_f("wb_supervisor_field_get_type", "Field", "field", "getType",
   ("field", "field", True))
_f("wb_supervisor_field_get_type_name", "Field", "field", "getTypeName",
   ("field", "field", True))
_f("wb_supervisor_field_get_count", "Field", "field", "getCount",
   ("field", "field", True))
_f("wb_supervisor_field_get_actual_field", "Field", "field",
   "getActualField", ("field", "field", True))
_f("wb_supervisor_field_enable_sf_tracking", "Field", "field",
   "enableSFTracking", ("field", "field", True),
   ("sampling_period", "int", True))
_f("wb_supervisor_field_disable_sf_tracking", "Field", "field",
   "disableSFTracking", ("field", "field", True))

# (wire-type, value-kind, Python method suffix) for the SF/MF families. The
# order is the upstream function index's order and is load-bearing for the
# manifest hash, so do not re-sort it.
_SF_TYPES = (("bool", "bool", "Bool"), ("int32", "int", "Int32"),
             ("float", "float", "Float"), ("vec2f", "vec2", "Vec2f"),
             ("vec3f", "vec3", "Vec3f"), ("rotation", "vec4", "Rotation"),
             ("color", "color", "Color"), ("string", "str", "String"))

for _t, _k, _m in _SF_TYPES:
    _f("wb_supervisor_field_get_sf_%s" % _t, "Field", "field",
       "getSF%s" % _m, ("field", "field", True))
_f("wb_supervisor_field_get_sf_node", "Field", "field", "getSFNode",
   ("field", "field", True))
for _t, _k, _m in _SF_TYPES:
    _f("wb_supervisor_field_get_mf_%s" % _t, "Field", "field",
       "getMF%s" % _m, ("field", "field", True), ("index", "int", True))
_f("wb_supervisor_field_get_mf_node", "Field", "field", "getMFNode",
   ("field", "field", True), ("index", "int", True))
for _t, _k, _m in _SF_TYPES:
    _f("wb_supervisor_field_set_sf_%s" % _t, "Field", "field",
       "setSF%s" % _m, ("field", "field", True), ("value", _k, True))
for _t, _k, _m in _SF_TYPES:
    _f("wb_supervisor_field_set_mf_%s" % _t, "Field", "field",
       "setMF%s" % _m, ("field", "field", True), ("index", "int", True),
       ("value", _k, True))
for _t, _k, _m in _SF_TYPES:
    _f("wb_supervisor_field_insert_mf_%s" % _t, "Field", "field",
       "insertMF%s" % _m, ("field", "field", True), ("index", "int", True),
       ("value", _k, True))

_f("wb_supervisor_field_remove_mf", "Field", "field", "removeMF",
   ("field", "field", True), ("index", "int", True))
_f("wb_supervisor_field_remove_sf", "Field", "field", "removeSF",
   ("field", "field", True))
_f("wb_supervisor_field_import_mf_node_from_string", "Field", "field",
   "importMFNodeFromString", ("field", "field", True),
   ("position", "int", True), ("node_string", "str", True))
_f("wb_supervisor_field_import_sf_node_from_string", "Field", "field",
   "importSFNodeFromString", ("field", "field", True),
   ("node_string", "str", True))

# -- Proto ------------------------------------------------------------------
_f("wb_supervisor_proto_get_type_name", "Proto", "proto", "getTypeName",
   ("proto", "proto", True))
_f("wb_supervisor_proto_get_parent", "Proto", "proto", "getParent",
   ("proto", "proto", True))
_f("wb_supervisor_proto_get_field", "Proto", "proto", "getField",
   ("proto", "proto", True), ("name", "str", True))
_f("wb_supervisor_proto_get_field_by_index", "Proto", "proto",
   "getFieldByIndex", ("proto", "proto", True), ("index", "int", True))
_f("wb_supervisor_proto_get_number_of_fields", "Proto", "proto",
   "getNumberOfFields", ("proto", "proto", True))
_f("wb_supervisor_proto_is_derived", "Proto", "proto", "isDerived",
   ("proto", "proto", True))

FUNCTIONS = {fn.name: fn for fn in _LIST}
assert len(FUNCTIONS) == len(_LIST), "duplicate function name in the table"

HANDLE_KINDS = ("node", "field", "proto", "device")


# --------------------------------------------------------------------------
# Errors, handles, coercion
# --------------------------------------------------------------------------

class BridgeError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class HandleTable:
    """Integer ids for Node/Field/Proto/Device values (the API is
    handle-based; upstream's C reference passes refs/tags, we pass ints).

    Interning is keyed on the underlying engine ref (``_ref`` for
    Node/Field/Proto, ``_tag`` for Device) when present, so asking for the
    same node twice yields the same handle, matching upstream's stable-ref
    behaviour.
    """

    def __init__(self):
        self._by_id = {}
        self._interned = {}
        self._next = 1

    @staticmethod
    def _identity(obj):
        ref = getattr(obj, "_ref", None)
        val = getattr(ref, "value", None)
        if val:
            return ("ref", val)
        tag = getattr(obj, "_tag", None)
        if tag is not None:
            return ("tag", tag)
        return ("pyid", id(obj))

    def intern(self, kind, obj):
        key = (kind, self._identity(obj))
        got = self._interned.get(key)
        if got is not None:
            return got
        handle = self._next
        self._next += 1
        self._by_id[handle] = (kind, obj)
        self._interned[key] = handle
        return handle

    def resolve(self, kind, handle):
        entry = self._by_id.get(handle)
        if entry is None:
            raise BridgeError("UNKNOWN_HANDLE",
                              "no %s handle %r; handles come from the return "
                              "values of earlier calls" % (kind, handle))
        got_kind, obj = entry
        if got_kind != kind:
            raise BridgeError("BAD_HANDLE_TYPE",
                              "handle %d is a %s handle, but a %s handle is "
                              "required here" % (handle, got_kind, kind))
        return obj


_VEC_LEN = {"vec2": 2, "vec3": 3, "vec4": 4, "vec6": 6, "color": 3}


def coerce(kind, name, value, handles):
    """Validate/convert one JSON argument; raises BridgeError, never crashes."""
    if kind in HANDLE_KINDS:
        if isinstance(value, bool) or not isinstance(value, int):
            raise BridgeError("BAD_ARGUMENT",
                              "'%s' must be an integer %s handle" % (name, kind))
        return handles.resolve(kind, value)
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise BridgeError("BAD_ARGUMENT", "'%s' must be an integer" % name)
        return value
    if kind == "float":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise BridgeError("BAD_ARGUMENT", "'%s' must be a number" % name)
        return float(value)
    if kind == "str":
        if not isinstance(value, str):
            raise BridgeError("BAD_ARGUMENT", "'%s' must be a string" % name)
        return value
    if kind == "bool":
        if not isinstance(value, bool):
            raise BridgeError("BAD_ARGUMENT", "'%s' must be a boolean" % name)
        return value
    if kind in _VEC_LEN:
        n = _VEC_LEN[kind]
        if (not isinstance(value, (list, tuple)) or len(value) != n
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       for v in value)):
            raise BridgeError("BAD_ARGUMENT",
                              "'%s' must be an array of %d numbers" % (name, n))
        return [float(v) for v in value]
    raise BridgeError("EXCEPTION", "unhandled kind %r in the table" % kind)


def serialize(value, handles):
    """Python API return value -> JSON-able. Node/Field/Proto/Device become
    ``{"<kind>": <handle>}``; ContactPoint lists become plain dicts (their
    accessors are data carriers, not API functions -- see EXCLUSIONS.md)."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    tname = type(value).__name__
    if tname == "Node":
        return {"node": handles.intern("node", value)}
    if tname == "Field":
        return {"field": handles.intern("field", value)}
    if tname == "Proto":
        return {"proto": handles.intern("proto", value)}
    if tname == "ContactPoint":
        return {"point": [float(v) for v in value.point],
                "node_id": int(value.node_id),
                "depth": float(getattr(value, "depth", 0.0))}
    if isinstance(value, (list, tuple)):
        return [serialize(v, handles) for v in value]
    # Device subclasses (Motor, Camera, ...) all carry _tag.
    if hasattr(value, "_tag"):
        return {"device": handles.intern("device", value)}
    return repr(value)


# --------------------------------------------------------------------------
# Dispatch
# --------------------------------------------------------------------------

class Bridge:
    """Maps ``{fn, args}`` requests onto a Supervisor instance.

    ``robot`` is duck-typed: the real ``controller.Supervisor`` in production,
    a fake in the unit tests (which is what lets the dispatch/error surface be
    tested with no Webots installed).
    """

    def __init__(self, robot):
        self.robot = robot
        self.handles = HandleTable()
        self.quit_requested = False

    # getFromDevice takes a device *tag* in the Python API while every other
    # device argument resolves to the Device object; keep the special case in
    # one place.
    def _special_get_from_device(self, resolved):
        device = resolved[0]
        tag = getattr(device, "_tag", None)
        if tag is None:
            raise BridgeError("BAD_ARGUMENT",
                              "device handle does not carry a device tag")
        return self.robot.getFromDevice(tag)

    def dispatch(self, request) -> dict:
        try:
            if not isinstance(request, dict):
                raise BridgeError("BAD_REQUEST",
                                  "a request must be a JSON object with 'fn' "
                                  "and optional 'args'")
            fn_name = request.get("fn")
            fn = FUNCTIONS.get(fn_name)
            if fn is None:
                raise BridgeError(
                    "UNKNOWN_FUNCTION",
                    "%r is not a bridged function; the bridge serves exactly "
                    "the %d functions in its tool list" % (fn_name,
                                                           len(FUNCTIONS)))
            args = request.get("args") or {}
            if not isinstance(args, dict):
                raise BridgeError("BAD_REQUEST", "'args' must be an object")

            resolved = []
            stopped = None  # first absent optional arg, if any
            for name, kind, required in fn.args:
                if name not in args:
                    if required:
                        raise BridgeError("MISSING_ARGUMENT",
                                          "missing required argument '%s'"
                                          % name)
                    if stopped is None:
                        stopped = name
                    continue
                if stopped is not None:
                    raise BridgeError(
                        "MISSING_ARGUMENT",
                        "optional argument '%s' was given but earlier "
                        "optional '%s' was not; optionals fill left to right"
                        % (name, stopped))
                resolved.append(coerce(kind, name, args[name], self.handles))

            try:
                if fn.name == "wb_supervisor_node_get_from_device":
                    result = self._special_get_from_device(resolved)
                elif fn.target == "robot":
                    result = getattr(self.robot, fn.method)(*resolved)
                else:
                    receiver = resolved[0]
                    result = getattr(receiver, fn.method)(*resolved[1:])
            except BridgeError:
                raise
            except Exception as exc:                      # noqa: BLE001
                return {"ok": False, "error": {
                    "code": "EXCEPTION",
                    "message": "%s raised %s: %s" % (fn.name,
                                                     type(exc).__name__, exc),
                    "detail": traceback.format_exc(limit=4)}}

            if fn.name == "wb_supervisor_simulation_quit":
                self.quit_requested = True
            return {"ok": True, "result": serialize(result, self.handles)}
        except BridgeError as exc:
            return {"ok": False,
                    "error": {"code": exc.code, "message": str(exc)}}


# --------------------------------------------------------------------------
# Server
# --------------------------------------------------------------------------

def serve(bridge, host, port, log=lambda msg: None):
    """Sequential JSON-lines server. Returns when the agent calls
    wb_supervisor_simulation_quit (after the reply is sent) or on EOF from
    the operator side (Ctrl-C)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    log("[webots_bridge] %s: serving %d functions on %s:%d"
        % (PROTOCOL, len(FUNCTIONS), host, port))
    try:
        while not bridge.quit_requested:
            try:
                conn, _peer = srv.accept()
            except OSError:
                break
            try:
                reader = conn.makefile("rb")
                for line in reader:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        request = json.loads(line.decode("utf-8"))
                    except ValueError:
                        response = {"ok": False, "error": {
                            "code": "BAD_REQUEST",
                            "message": "request is not valid JSON"}}
                    else:
                        response = bridge.dispatch(request)
                    conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
                    if bridge.quit_requested:
                        break
            except OSError as exc:
                log("[webots_bridge] client connection error: %s" % exc)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
    finally:
        srv.close()
    log("[webots_bridge] server stopped")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=int(os.environ.get(
        "AGENTBENCH_WEBOTS_BRIDGE_PORT", "6799")))
    opts = parser.parse_args(argv)

    # Lazy on purpose: the module must be importable (for its FUNCTIONS
    # table) with no Webots installed.
    from controller import Supervisor  # noqa: PLC0415
    supervisor = Supervisor()
    if not supervisor.getSupervisor():
        print("[webots_bridge] FATAL: this controller's Robot node has "
              "supervisor FALSE; the wb_supervisor_* functions would all "
              "fail. Set `supervisor TRUE` on the bridge robot.",
              file=sys.stderr, flush=True)
        return 1

    bridge = Bridge(supervisor)

    def log(msg):
        print(msg, flush=True)

    try:
        serve(bridge, opts.host, opts.port, log=log)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
