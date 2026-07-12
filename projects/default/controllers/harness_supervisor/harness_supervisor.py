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

"""harness_supervisor — generic supervisor used by the OmniSim agent-facing
validation harness.

Injected into any user world by the harness via a sibling file (see
scripts/harness/README.md). Listens on a local TCP socket for
length-prefixed JSON commands from the harness process and proxies them
onto the Supervisor API.

Wire protocol
-------------
Each frame is a 4-byte big-endian length followed by a UTF-8 JSON payload.

Request:   {"id": <int>, "cmd": "<name>", "args": {...}}
Response:  {"id": <int>, "ok": true, "result": {...}}
       or  {"id": <int>, "ok": false, "error": "<message>"}

Commands
--------
ping              -> {}
sim_state         -> {sim_time_ms, basic_time_step_ms, ...}
step              -> {sim_time_ms}                  args: {steps?: int}
reset             -> {sim_time_ms}                  full simulation reset
world_load        -> {path}                          args: {path: str}     hot reload
screenshot        -> {path}                          args: {path: str, quality?: int}
scene_tree        -> {nodes: [{def, type, position, ...}, ...]}
scene_node        -> {def, type, fields: {...}, position, orientation}   args: {def: str}
set_viewpoint     -> {position, orientation}         args: {position: [x,y,z], orientation: [ax,ay,az,angle]}
damage_state      -> {robot, attached, parts, damage:{part:{state,hp,hp_max,...}}, game_over, ...}
damage_events     -> {events: [...], last_step_id, events_total}         args: {since?: int, limit?: int}
                     events have type:"impact" or type:"state_transition";
                     impact = {step_id, sim_time_ms, part, impulse_J, point, other};
                     state_transition = {step_id, sim_time_ms, part, from_state, to_state, hp, trigger_impulse_J}
damage_reset      -> {ok: true}                       heals all parts to pristine without resetting the sim
damage_inject     -> {state, hp, hp_max, ...}                          args: {part: str, hp_delta?: float, state?: str}
                     test/debug hook to set a part's state directly without the contact pipeline
robots_list       -> {robots: [{def, name, model, controller, type, position, orientation, num_joints}]}
robot_joints      -> {robot, joints: [{name, type, position, velocity, lower, upper, hit_limit}]}   args: {def: str}
robot_devices     -> {robot, devices: [{name, type}]}                  args: {def: str}
sim_contacts      -> {contacts: [{a_def, b_def, point}]}
sim_grips         -> {grips: [{gripper_def, held_def, since_t_ms}]}
events_drain      -> {events: [...], next_seq, total, dropped, buffered}
                     args: {since?: int, limit?: int, types?: [str]}

The bind host/port can be overridden via OMNISIM_HARNESS_SUPERVISOR_HOST
and OMNISIM_HARNESS_SUPERVISOR_PORT environment variables (defaults
127.0.0.1:6790). The robot name the damage tracker looks for can be set
via OMNISIM_HARNESS_DAMAGE_ROBOT (default 'husky'); if no robot of that
name is in the world, the tracker idles and damage_* commands return
the empty/idle state.
"""

from __future__ import annotations

import json
import os
import select
import socket
import struct
import sys
import time
import traceback

from omnisim import Supervisor

from damage_tracker import DamageTracker
from event_bus import ContactTracker, EventBus, GripTracker, JointLimitTracker
import observe

# Optional: mirror stderr to a file (OMNISIM_SUPERVISOR_STDERR_LOG=path).
# Useful when the OmniSim console isn't accessible -- e.g. debugging FPS
# issues remotely or post-mortem grep.
_STDERR_LOG_PATH = os.environ.get("OMNISIM_SUPERVISOR_STDERR_LOG", "")
if _STDERR_LOG_PATH:
    class _StderrTee:
        def __init__(self, stream, path):
            self._stream = stream
            try:
                self._file = open(path, "w", buffering=1, encoding="utf-8")
            except OSError:
                self._file = None
        def write(self, s):
            self._stream.write(s)
            if self._file is not None:
                self._file.write(s)
        def flush(self):
            self._stream.flush()
            if self._file is not None:
                self._file.flush()
    sys.stderr = _StderrTee(sys.stderr, _STDERR_LOG_PATH)

HOST = os.environ.get("OMNISIM_HARNESS_SUPERVISOR_HOST", "127.0.0.1")
PORT = int(os.environ.get("OMNISIM_HARNESS_SUPERVISOR_PORT", "6790"))
DAMAGE_ROBOT_NAME = os.environ.get("OMNISIM_HARNESS_DAMAGE_ROBOT", "husky")
# Optional secondary robots tracked alongside the primary. Each name in
# the comma-separated list gets its own DamageTracker with its own
# state, contact polling, scene mutations (Phase 7 appearance, Phase 9
# detach, etc.). Wire-protocol damage_* commands address the PRIMARY
# tracker only; secondaries run for visual symmetry (e.g. so a head-on
# crash damages BOTH huskies on screen, not just the tracked one).
DAMAGE_EXTRA_ROBOTS = [
    n.strip() for n in os.environ.get(
        "OMNISIM_HARNESS_DAMAGE_EXTRA_ROBOTS", "husky_b").split(",")
    if n.strip()
]


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = n
    while remaining > 0:
        try:
            chunk = sock.recv(remaining)
        except OSError:
            # Aborted/reset connections (e.g. the harness timing out an RPC
            # and closing its socket) are a disconnect, not a controller
            # crash — the caller drops this client and keeps serving.
            return None
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame(sock: socket.socket) -> dict | None:
    header = recv_exact(sock, 4)
    if header is None:
        return None
    (length,) = struct.unpack(">I", header)
    if length == 0 or length > 16 * 1024 * 1024:
        return None
    payload = recv_exact(sock, length)
    if payload is None:
        return None
    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError:
        return None


def write_frame(sock: socket.socket, obj: dict) -> bool:
    body = json.dumps(obj).encode("utf-8")
    try:
        sock.sendall(struct.pack(">I", len(body)) + body)
        return True
    except OSError:
        return False


def node_summary(node) -> dict:
    if node is None:
        return {}
    out: dict = {"type": node.getTypeName()}
    def_name = node.getDef()
    if def_name:
        out["def"] = def_name
    try:
        out["position"] = list(node.getPosition())
    except Exception:
        pass
    try:
        # 3x3 rotation matrix flattened
        out["orientation"] = list(node.getOrientation())
    except Exception:
        pass
    return out


def walk_scene_tree(root) -> list[dict]:
    nodes: list[dict] = []

    def visit(node, parent_def):
        if node is None:
            return
        summary = node_summary(node)
        summary["parent_def"] = parent_def
        nodes.append(summary)
        own_def = summary.get("def")
        for field_name in ("children", "endPoint"):
            field = node.getField(field_name)
            if field is None:
                continue
            try:
                if field_name == "endPoint":
                    visit(field.getSFNode(), own_def or parent_def)
                else:
                    count = field.getCount()
                    for i in range(count):
                        visit(field.getMFNode(i), own_def or parent_def)
            except Exception:
                continue

    visit(root, None)
    return nodes


def find_node_by_def(supervisor: Supervisor, def_name: str):
    return supervisor.getFromDef(def_name)


def find_viewpoint(supervisor: Supervisor):
    """Walk the root group looking for the Viewpoint node. There is no
    direct Supervisor.getViewpoint() in the Python binding, so we scan the
    top-level children of the root.
    """
    root = supervisor.getRoot()
    if root is None:
        return None
    children = root.getField("children")
    if children is None:
        return None
    try:
        count = children.getCount()
    except Exception:
        return None
    for i in range(count):
        node = children.getMFNode(i)
        if node is not None and node.getTypeName() == "Viewpoint":
            return node
    return None


def field_value(field) -> object:
    if field is None:
        return None
    try:
        ftype = field.getType()
    except Exception:
        return None
    # Map common single-field types to Python primitives. The constants come
    # from the Webots controller library; we use the int values as fallback
    # if the symbolic names aren't exposed.
    try:
        from omnisim import Field

        type_map = {
            Field.SF_BOOL: lambda f: f.getSFBool(),
            Field.SF_INT32: lambda f: f.getSFInt32(),
            Field.SF_FLOAT: lambda f: f.getSFFloat(),
            Field.SF_VEC2F: lambda f: list(f.getSFVec2f()),
            Field.SF_VEC3F: lambda f: list(f.getSFVec3f()),
            Field.SF_ROTATION: lambda f: list(f.getSFRotation()),
            Field.SF_COLOR: lambda f: list(f.getSFColor()),
            Field.SF_STRING: lambda f: f.getSFString(),
        }
        if ftype in type_map:
            return type_map[ftype](field)
    except Exception:
        pass
    return None


def node_detail(node) -> dict:
    summary = node_summary(node)
    fields: dict = {}
    # Best-effort field dump: walk known field names. The full field list isn't
    # cheaply enumerable from Python, so we expose what callers can see by
    # name. Future revisions can add an explicit field-introspection RPC.
    for fname in ("name", "controller", "translation", "rotation", "scale", "model"):
        f = node.getField(fname)
        v = field_value(f)
        if v is not None:
            fields[fname] = v
    summary["fields"] = fields
    contacts: list[list[float]] = []
    try:
        for cp in node.getContactPoints(False) or []:
            contacts.append(list(cp.point))
    except Exception:
        pass
    if contacts:
        summary["contact_points"] = contacts
    return summary


class CommandError(Exception):
    pass


def dispatch(supervisor: Supervisor, basic_step_ms: int, sim_time_ms: float,
             cmd: str, args: dict, damage: DamageTracker | None = None,
             bus: EventBus | None = None,
             contact_tracker: ContactTracker | None = None,
             grip_tracker: GripTracker | None = None,
             joint_velocity_cache: dict | None = None):
    if cmd == "ping":
        return {}
    if cmd == "sim_state":
        return {
            "sim_time_ms": sim_time_ms,
            "basic_time_step_ms": basic_step_ms,
        }
    if cmd == "damage_state":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        return damage.state_snapshot()
    if cmd == "damage_events":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        since = int(args.get("since", 0))
        limit = int(args.get("limit", 256))
        if limit < 1:
            limit = 1
        if limit > 1024:
            limit = 1024
        evts = damage.events_since(since, limit)
        last_id = evts[-1]["step_id"] if evts else since
        return {"events": evts, "last_step_id": last_id,
                "events_total": damage.event_counter}
    if cmd == "damage_reset":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        damage.reset()
        return {"ok": True}
    if cmd == "damage_geometry_stats":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        return damage.geometry_stats()
    if cmd == "damage_set_heal_rate":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        rate_hp = args.get("rate_hp")
        rate_mesh = args.get("rate_mesh")
        parts = args.get("parts")
        if rate_hp is not None:
            try:
                rate_hp = float(rate_hp)
            except (TypeError, ValueError):
                raise CommandError("'rate_hp' must be a number")
        if rate_mesh is not None:
            try:
                rate_mesh = float(rate_mesh)
            except (TypeError, ValueError):
                raise CommandError("'rate_mesh' must be a number")
        if parts is not None and not isinstance(parts, list):
            raise CommandError("'parts' must be a list of strings")
        return damage.set_heal_rate(rate_hp=rate_hp, rate_mesh=rate_mesh,
                                     parts=parts)
    if cmd == "damage_heal_to_pristine":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        return damage.heal_to_pristine(sim_time_ms=int(sim_time_ms))
    if cmd == "damage_inject":
        if damage is None:
            raise CommandError("damage tracker not initialised")
        part = args.get("part")
        if not isinstance(part, str) or not part:
            raise CommandError("damage_inject requires 'part'")
        result = damage.inject(
            part,
            hp_delta=args.get("hp_delta"),
            state=args.get("state"),
            sim_time_ms=int(sim_time_ms),
        )
        if "error" in result:
            raise CommandError(result["error"])
        return result
    if cmd == "step":
        steps = int(args.get("steps", 1))
        if steps < 1:
            steps = 1
        # Tick damage detection once per inner step so headless harness
        # callers (--mode=fast scripted runs) see contact-driven dent
        # accumulation. Without this, damage.poll() only fires on the
        # outer main-loop iterations between commands — which is roughly
        # one tick per RPC, not one per sim step.
        local_sim_ms = float(sim_time_ms)
        for _ in range(steps):
            if supervisor.step(basic_step_ms) == -1:
                raise CommandError("simulator step returned -1 (terminating)")
            local_sim_ms += basic_step_ms
            if damage is not None:
                damage.poll(int(local_sim_ms))
            if contact_tracker is not None:
                try:
                    contact_tracker.poll(local_sim_ms)
                except Exception:
                    pass
            if grip_tracker is not None and contact_tracker is not None:
                try:
                    grip_tracker.poll(
                        contact_tracker.current_pairs(),
                        observe.build_robot_subtree_index(supervisor),
                        local_sim_ms,
                    )
                except Exception:
                    pass
        return {"sim_time_ms": local_sim_ms, "advanced_to_ms": local_sim_ms}
    if cmd == "reset":
        supervisor.simulationReset()
        return {"sim_time_ms": 0}
    if cmd == "world_load":
        path = args.get("path")
        if not isinstance(path, str) or not path:
            raise CommandError("world_load requires a 'path' string")
        # Webots's worldLoad() resolves relative paths against the
        # controller's CWD — which for our supervisor is its own
        # controller folder, NOT the repo root. That gave us a
        # `controllers/harness_supervisor/projects/samples/.../foo.wbt`
        # nonsense path that failed to open, leaving the simulator in
        # a half-reverted broken state. Accept relative paths from
        # callers but resolve them against OMNISIM_HOME or the current
        # working directory before forwarding.
        if not os.path.isabs(path):
            base = os.environ.get("OMNISIM_HOME") or os.getcwd()
            path = os.path.normpath(os.path.join(base, path))
        if not os.path.exists(path):
            raise CommandError(f"world file not found: {path!r}")
        # The reply must be sent BEFORE the simulator actually swaps worlds —
        # worldLoad terminates this controller process, dropping the socket.
        # We return early; the caller must handle the disconnect and reconnect
        # to the new supervisor instance after the world swap completes.
        supervisor.worldLoad(path)
        return {"path": path}
    if cmd == "screenshot":
        path = args.get("path")
        if not isinstance(path, str) or not path:
            raise CommandError("screenshot requires a 'path' string")
        quality = int(args.get("quality", 90))
        if not (1 <= quality <= 100):
            quality = 90
        ok = supervisor.exportImage(path, quality)
        if ok is False:  # Webots API returns None on success, False on failure
            raise CommandError(f"exportImage failed for path '{path}'")
        return {"path": path}
    if cmd == "scene_tree":
        return {"nodes": walk_scene_tree(supervisor.getRoot())}
    if cmd == "scene_node":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("scene_node requires a 'def' string")
        node = find_node_by_def(supervisor, def_name)
        if node is None:
            raise CommandError(f"no node with DEF '{def_name}'")
        return node_detail(node)
    if cmd == "set_viewpoint":
        position = args.get("position")
        orientation = args.get("orientation")
        if position is not None and (not isinstance(position, list) or len(position) != 3):
            raise CommandError("position must be a list of 3 numbers")
        if orientation is not None and (not isinstance(orientation, list) or len(orientation) != 4):
            raise CommandError("orientation must be a list of 4 numbers")
        viewpoint = find_viewpoint(supervisor)
        if viewpoint is None:
            raise CommandError("no Viewpoint node found in the scene")
        if position is not None:
            field = viewpoint.getField("position")
            if field is None:
                raise CommandError("Viewpoint has no 'position' field")
            field.setSFVec3f([float(x) for x in position])
        if orientation is not None:
            field = viewpoint.getField("orientation")
            if field is None:
                raise CommandError("Viewpoint has no 'orientation' field")
            field.setSFRotation([float(x) for x in orientation])
        return {
            "position": position if position is not None else None,
            "orientation": orientation if orientation is not None else None,
        }
    if cmd == "robots_list":
        return {"robots": observe.list_robots(supervisor)}
    if cmd == "robot_joints":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("robot_joints requires a 'def' string")
        cache = joint_velocity_cache if joint_velocity_cache is not None else {}
        try:
            return observe.list_joints(supervisor, def_name, cache, sim_time_ms)
        except KeyError as exc:
            raise CommandError(str(exc))
    if cmd == "robot_devices":
        def_name = args.get("def")
        if not isinstance(def_name, str) or not def_name:
            raise CommandError("robot_devices requires a 'def' string")
        try:
            return observe.list_devices(supervisor, def_name)
        except KeyError as exc:
            raise CommandError(str(exc))
    if cmd == "sim_contacts":
        return {"contacts": observe.list_contacts(supervisor)}
    if cmd == "sim_grips":
        if grip_tracker is None:
            return {"grips": []}
        return {"grips": grip_tracker.active_grips()}
    if cmd == "events_drain":
        if bus is None:
            return {"events": [], "next_seq": 0, "total": 0,
                    "dropped": 0, "buffered": 0}
        since = int(args.get("since", 0))
        limit = int(args.get("limit", 256))
        if limit < 1:
            limit = 1
        if limit > 1024:
            limit = 1024
        types = args.get("types")
        if types is not None and not isinstance(types, list):
            raise CommandError("'types' must be a list of strings")
        events = bus.since(since, limit, types=types)
        next_seq = events[-1]["seq"] if events else since
        return {
            "events": events,
            "next_seq": next_seq,
            "total": bus.total,
            "dropped": bus.dropped,
            "buffered": bus.buffered,
        }
    raise CommandError(f"unknown cmd: {cmd}")


def _load_inject_schedule(supervisor) -> list[dict]:
    """Parse a `damage_inject_schedule` array from the supervisor robot's
    own customData. Returns a sorted list of {t_ms, part, state, hp_delta}
    dicts. Bad entries are silently skipped; malformed JSON yields []
    with a stderr note. World authors use this to script broken-state
    transitions without an external wire-protocol client.
    """
    self_node = supervisor.getSelf()
    if self_node is None:
        return []
    f = self_node.getField("customData")
    if f is None:
        return []
    raw = f.getSFString() or ""
    raw = raw.strip()
    if not raw:
        return []
    try:
        cfg = json.loads(raw)
    except (TypeError, ValueError) as exc:
        sys.stderr.write(
            f"[harness_supervisor] customData JSON invalid ({exc}); "
            "ignoring inject schedule\n"
        )
        return []
    sched = cfg.get("damage_inject_schedule") if isinstance(cfg, dict) else None
    if not isinstance(sched, list):
        return []
    out: list[dict] = []
    for raw_entry in sched:
        if not isinstance(raw_entry, dict):
            continue
        try:
            t_ms = int(raw_entry["t_ms"])
            part = str(raw_entry["part"])
        except (KeyError, TypeError, ValueError):
            continue
        out.append({
            "t_ms": t_ms,
            "part": part,
            "state": raw_entry.get("state"),
            "hp_delta": raw_entry.get("hp_delta"),
        })
    out.sort(key=lambda e: e["t_ms"])
    return out


def main() -> int:
    supervisor = Supervisor()
    basic_step_ms = int(supervisor.getBasicTimeStep())

    # Bind the control socket before the first sim step so the harness can
    # connect immediately after world load completes.
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((HOST, PORT))
    except OSError as exc:
        sys.stderr.write(f"[harness_supervisor] bind {HOST}:{PORT} failed: {exc}\n")
        return 1
    server.listen(4)
    server.setblocking(False)
    sys.stderr.write(f"[harness_supervisor] listening on {HOST}:{PORT}\n")
    sys.stderr.flush()

    # Damage tracker: discovers a robot by name on startup and polls per
    # step. Idles silently if the named robot isn't in the world (most
    # harnessed worlds have no Husky and no damage to report).
    damage = DamageTracker(supervisor, robot_name=DAMAGE_ROBOT_NAME)
    # Secondary trackers for additional robots (visual symmetry only;
    # the wire protocol still operates on `damage`). Each idles silently
    # if its robot isn't in the world. With OMNISIM_HARNESS_DAMAGE_EXTRA_ROBOTS
    # set to "husky_b" by default, a husky_head_on world tracks both
    # huskies — head-on collision damages both bumpers on screen.
    extra_damages = []
    for extra_name in DAMAGE_EXTRA_ROBOTS:
        if extra_name == DAMAGE_ROBOT_NAME:
            continue  # don't double-track the same robot
        try:
            extra_damages.append(
                DamageTracker(supervisor, robot_name=extra_name))
        except Exception as exc:  # noqa: BLE001
            sys.stderr.write(
                f"[harness_supervisor] failed to create tracker for "
                f"{extra_name!r}: {exc}\n")
    if extra_damages:
        sys.stderr.write(
            f"[harness_supervisor] tracking {1 + len(extra_damages)} robot(s): "
            f"{DAMAGE_ROBOT_NAME!r} + extras "
            f"{[t.robot_name for t in extra_damages]}\n")
        sys.stderr.flush()

    # Unified event bus + producers (Phase 2). Trackers run each step
    # alongside damage.poll(); their events feed the same /sim/events
    # endpoint that fans out damage events too. Walking the scene tree
    # every step is fine for the worlds the harness sees today
    # (typically <50 solids); bigger worlds will want a longer cadence.
    #
    # P6: --light flag (passed via controllerArgs in the world file)
    # skips contact_tracker, joint_limit_tracker, grip_tracker. They
    # each walk the scene tree per step, which on multi-husky worlds
    # (newton_husky_head_on_damage = 8 huskies, ~150 solids) drops
    # the rendered framerate to slow motion. damage.poll() does not
    # depend on those trackers, so disabling them keeps the visible
    # damage demo and gets the framerate back.
    LIGHT_MODE = "--light" in sys.argv
    bus = EventBus()
    if LIGHT_MODE:
        sys.stderr.write("[harness_supervisor] --light: skipping contact/joint-limit/grip trackers\n")
        sys.stderr.flush()
        contact_tracker = None
        joint_limit_tracker = None
        grip_tracker = None
        # --light also implies lite damage FX: no spawned-in-scene
        # markers, debris, smoke, sparks, decals, or mesh re-emit. The
        # robots still take damage internally (HP, state transitions);
        # only the visual "rash" on top of the chassis goes away.
        # Explicit OMNISIM_LITE_DAMAGE=0 in the environment opts back in.
        if os.environ.get("OMNISIM_LITE_DAMAGE") is None:
            os.environ["OMNISIM_LITE_DAMAGE"] = "1"
    else:
        contact_tracker = ContactTracker(supervisor, bus)
        joint_limit_tracker = JointLimitTracker(supervisor, bus)
        grip_tracker = GripTracker(supervisor, bus)
    # Per-joint velocity cache: id -> (last_position, last_t_s). Shared
    # across all dispatch("robot_joints") calls so velocities are
    # meaningful across snapshots.
    joint_velocity_cache: dict = {}
    # Hook the damage tracker so its events are also visible in the
    # unified bus. We monkey-patch the existing _emit / _emit_transition
    # methods to fan-out — keeps the existing /robot/damage/events
    # endpoint working byte-for-byte.
    _orig_emit = damage._emit
    _orig_emit_transition = damage._emit_transition

    def _emit_with_fanout(sim_time_ms_inner, part, impulse_J, point, other_name):
        _orig_emit(sim_time_ms_inner, part, impulse_J, point, other_name)
        bus.emit("damage.impact", {
            "part": part,
            "impulse_J": float(impulse_J),
            "point": [float(point[0]), float(point[1]), float(point[2])],
            "other": other_name,
        }, t_sim_ms=sim_time_ms_inner)

    def _emit_transition_with_fanout(sim_time_ms_inner, part, from_state,
                                     to_state, hp, trigger_J, point=None):
        _orig_emit_transition(sim_time_ms_inner, part, from_state, to_state,
                              hp, trigger_J, point)
        bus.emit("damage.state_transition", {
            "part": part,
            "from_state": from_state,
            "to_state": to_state,
            "hp": float(hp),
            "trigger_impulse_J": float(trigger_J),
        }, t_sim_ms=sim_time_ms_inner)

    damage._emit = _emit_with_fanout
    damage._emit_transition = _emit_transition_with_fanout

    # Parse an optional inject schedule from the supervisor's own
    # customData. Lets a world script "break this part at t=20s" without
    # any external client — the supervisor runs the schedule itself.
    # Schema: {"damage_inject_schedule": [{"t_ms": int, "part": str,
    # "state": str|null, "hp_delta": float|null}, ...]}. Sorted by t_ms;
    # entries fire once when sim_time_ms first crosses their threshold.
    inject_schedule = _load_inject_schedule(supervisor)
    inject_idx = 0
    if inject_schedule:
        sys.stderr.write(
            f"[harness_supervisor] loaded {len(inject_schedule)} scheduled "
            f"inject(s)\n"
        )

    clients: list[socket.socket] = []
    sim_time_ms = 0.0

    # Damage poll sub-sampling. damage.poll() does ~15 supervisor IPC
    # calls per invocation (one per tracked solid for getPosition + the
    # robot_node.getContactPoints call). At 62 Hz that's ~1000 IPC
    # calls/sec from damage alone, which dominates the rendered FPS on
    # multi-husky worlds. In --light we sub-sample to ~8 Hz; damage
    # state is visual feedback, not a control loop. Sweep on the head-
    # on damage world (8 huskies) measured cumulative sim-speed of
    # 1.75x at every-step polling vs 3.54x at every-8th-step, and the
    # speed during the post-collision damage-effect burst stays above
    # real-time only at >=8. 16 and 32 didn't improve further.
    # OMNISIM_DAMAGE_POLL_EVERY overrides the default (1 = every step).
    _default_poll_every = 8 if LIGHT_MODE else 1
    damage_poll_every = int(os.environ.get(
        "OMNISIM_DAMAGE_POLL_EVERY", _default_poll_every))
    damage_poll_tick = 0

    # Optional FPS measurement: ratio of sim-time advance to wall-time
    # elapsed (1.0 = real-time, <1.0 = slow motion). Enabled by setting
    # OMNISIM_FPS_LOG to a writable path; one line per FPS_REPORT_S
    # carries the cumulative + windowed speed. The poll cadence is in
    # the header so logs across runs are self-identifying.
    import time as _stdtime
    _FPS_LOG_PATH = os.environ.get("OMNISIM_FPS_LOG", "")
    FPS_REPORT_S = 2.0
    fps_t0_wall = _stdtime.time()
    fps_t0_sim_ms = 0
    fps_last_report_wall = fps_t0_wall
    if _FPS_LOG_PATH:
        try:
            with open(_FPS_LOG_PATH, "w", buffering=1) as _f:
                _f.write(
                    f"# sim_fps.log poll_every={damage_poll_every} "
                    f"light={LIGHT_MODE} basic_step_ms={basic_step_ms}\n"
                )
        except OSError:
            _FPS_LOG_PATH = ""

    while supervisor.step(basic_step_ms) != -1:
        # Webots's simulationReset rewinds sim time to 0 without
        # restarting controller processes. Detect that by watching
        # supervisor.getTime() jump backwards relative to our local
        # counter; when it does, rewind sim_time_ms and rearm the
        # inject schedule so a reloaded demo replays the same
        # detachments.
        engine_time_ms = supervisor.getTime() * 1000.0
        if engine_time_ms + 1.0 < sim_time_ms:
            sys.stderr.write(
                f"[harness_supervisor] sim reset detected "
                f"(was {sim_time_ms:.0f}ms, now {engine_time_ms:.0f}ms); "
                "rearming inject schedule and clearing damage state\n"
            )
            sim_time_ms = engine_time_ms
            inject_idx = 0
            # Reset accumulated damage state too — otherwise the user
            # sees one robot starting "already destroyed" because HP
            # values and spawned debris carry over from the prior run.
            for tracker in (damage, *extra_damages):
                try:
                    tracker.reset()
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[harness_supervisor] {tracker.robot_name!r} "
                        f"reset on sim-reset failed: {exc}\n"
                    )
        else:
            sim_time_ms += basic_step_ms
        damage_poll_tick += 1
        if damage_poll_tick >= damage_poll_every:
            damage_poll_tick = 0
            # Defensive: a crash inside damage.poll() would kill the
            # supervisor, leaving the harness HTTP service unable to
            # service any command until the world is reloaded. Worth
            # far more than the minor risk of masking a damage-tracker
            # bug — the bug log goes to stderr where it can be
            # inspected.
            try:
                damage.poll(int(sim_time_ms))
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] damage.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )
            # Secondary trackers run in parallel for visual symmetry.
            # One secondary crashing doesn't take down the others,
            # doesn't take down the primary, and doesn't kill the
            # supervisor.
            for extra in extra_damages:
                try:
                    extra.poll(int(sim_time_ms))
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(
                        f"[harness_supervisor] extra damage.poll on "
                        f"{extra.robot_name!r} crashed: {exc}\n"
                        f"{traceback.format_exc()}"
                    )
        # Phase 2 producers: contact deltas, joint-limit transitions,
        # grip detection. Each is wrapped so a producer crash on one
        # step doesn't kill the whole supervisor — stderr captures the
        # bug for inspection. Skipped in --light mode (P6).
        if contact_tracker is not None:
            try:
                contact_tracker.poll(sim_time_ms)
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] contact_tracker.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )
        if joint_limit_tracker is not None:
            try:
                joint_limit_tracker.poll(sim_time_ms)
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] joint_limit_tracker.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )
        if grip_tracker is not None and contact_tracker is not None:
            try:
                grip_tracker.poll(
                    contact_tracker.current_pairs(),
                    observe.build_robot_subtree_index(supervisor),
                    sim_time_ms,
                )
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] grip_tracker.poll crashed: {exc}\n"
                    f"{traceback.format_exc()}"
                )

        # Periodic FPS measurement (sim time / wall time over the last
        # FPS_REPORT_S window). Only writes if OMNISIM_FPS_LOG is set.
        if _FPS_LOG_PATH:
            _wall_now = _stdtime.time()
            if _wall_now - fps_last_report_wall >= FPS_REPORT_S:
                window_wall = _wall_now - fps_last_report_wall
                window_sim_ms = sim_time_ms - fps_t0_sim_ms
                total_wall = _wall_now - fps_t0_wall
                window_speed = (window_sim_ms / 1000.0) / max(window_wall, 1e-6)
                cumulative_speed = (sim_time_ms / 1000.0) / max(total_wall, 1e-6)
                try:
                    with open(_FPS_LOG_PATH, "a", buffering=1) as _f:
                        _f.write(
                            f"sim={sim_time_ms:7.0f}ms wall={total_wall:6.2f}s "
                            f"speed_window={window_speed:5.2f}x "
                            f"speed_cum={cumulative_speed:5.2f}x "
                            f"effective_hz={window_speed*1000.0/basic_step_ms:5.1f}\n"
                        )
                except OSError:
                    pass
                fps_last_report_wall = _wall_now
                fps_t0_sim_ms = sim_time_ms

        # Fire any scheduled injects whose t_ms is now due. Schedule is
        # sorted, so a single forward index suffices.
        while inject_idx < len(inject_schedule) and \
                sim_time_ms >= inject_schedule[inject_idx]["t_ms"]:
            entry = inject_schedule[inject_idx]
            inject_idx += 1
            try:
                damage.inject(
                    entry["part"],
                    hp_delta=entry.get("hp_delta"),
                    state=entry.get("state"),
                    sim_time_ms=int(sim_time_ms),
                )
                sys.stderr.write(
                    f"[harness_supervisor] scheduled inject t={entry['t_ms']}ms "
                    f"part={entry['part']} state={entry.get('state')}\n"
                )
            except Exception as exc:  # noqa: BLE001
                sys.stderr.write(
                    f"[harness_supervisor] scheduled inject failed: {exc}\n"
                )

        # Accept any pending connections (non-blocking).
        try:
            while True:
                client, _ = server.accept()
                client.setblocking(True)
                clients.append(client)
        except BlockingIOError:
            pass

        # Drain ready clients. Each iteration handles at most one frame per
        # client per step so a chatty client can't starve the sim loop.
        if clients:
            ready, _, _ = select.select(clients, [], [], 0)
            for client in ready:
                request = read_frame(client)
                if request is None:
                    try:
                        client.close()
                    except OSError:
                        pass
                    clients.remove(client)
                    continue
                req_id = request.get("id", 0)
                cmd = request.get("cmd", "")
                args = request.get("args") or {}
                try:
                    result = dispatch(supervisor, basic_step_ms, sim_time_ms, cmd, args,
                                      damage=damage, bus=bus,
                                      contact_tracker=contact_tracker,
                                      grip_tracker=grip_tracker,
                                      joint_velocity_cache=joint_velocity_cache)
                    # Step command may have advanced sim time inside its
                    # own loop (so it could tick damage.poll per inner
                    # step); pull the new counter back so the outer loop
                    # stays in sync.
                    if cmd == "step" and isinstance(result, dict) and "advanced_to_ms" in result:
                        sim_time_ms = float(result["advanced_to_ms"])
                    write_frame(client, {"id": req_id, "ok": True, "result": result})
                except CommandError as exc:
                    write_frame(client, {"id": req_id, "ok": False, "error": str(exc)})
                except Exception as exc:  # noqa: BLE001
                    sys.stderr.write(f"[harness_supervisor] {cmd} crashed: {exc}\n{traceback.format_exc()}")
                    write_frame(client, {"id": req_id, "ok": False, "error": f"internal: {exc}"})

    for client in clients:
        try:
            client.close()
        except OSError:
            pass
    server.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
