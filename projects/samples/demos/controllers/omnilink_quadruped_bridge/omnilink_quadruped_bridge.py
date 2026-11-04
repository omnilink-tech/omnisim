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

"""omnilink_quadruped_bridge — config-driven quadruped bridge.

`--robot <key>` selects an entry of `_quadruped_configs.QUADRUPED_CONFIGS`
(OmniQuad, and the Deep Robotics Lite3 / X30 / M20 / M20S / M20 Piper). The
config names the leg motors, the stand / sit poses and the sign conventions;
this file is the motion and intent surface:

    stand       -- hold the standing stance with a tiny sway
    sit         -- crouch low (knees bent further)
    wave        -- exaggerate the sway for ~6 s
    walk        -- per config: "gait" = wave-gait leg cycle + supervisor-
                   driven forward body translation (OmniQuad: the only way
                   to keep it upright while it locomotes -- pure-physics
                   walking on that URDF reliably topples after 5-10 s; see
                   `omniquad_simple_pose.py`); "wheels" = hold the stance
                   and drive the wheel motors (the M20 family, real
                   physics); "march" = the leg cycle in place
    stop        -- freeze the legs (and wheels) in current position
    home        -- alias for stand

`--locomotion crawl|trot` (Lite3 / X30 only) replaces the config's walk with
real legged locomotion from `_crawl_motion`: the Deep Robotics creep gait or
its diagonal trot, on real contact physics with no supervisor pin, and four
MEASURED verbs -- walk{distance}, turn{angle_rad}, go_to{place} and
walk_to{x, y} -- that report {commanded, achieved, error, settled}. Places are
named in the robot's customData: {"places": {"<name>": [x, y, yaw_deg,
"<description>"], ...}}. Without the flag every robot behaves as before.

Every robot starts with a settle: the motors are ramped from the pose the
engine spawned them in to the stand pose over the config's `settle_s` (the
Deep Robotics package notes why: projects/robots/deep_robotics/PROVENANCE.md).

The HTTP surface is intentionally minimal: list_robots, get_robot_state,
stop_robot, reset_to_home, prompt. set_joint_positions is also exposed
for advanced users who want to send raw joint vectors.

Robot window: same omnilink_chat plugin.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

from omnisim import Supervisor

import os as _os
import sys as _sys
_THIS_DIR = _os.path.dirname(_os.path.abspath(__file__))
_RELAY_PARENT = _os.path.abspath(_os.path.join(_THIS_DIR, ".."))
if _RELAY_PARENT not in _sys.path:
    _sys.path.insert(0, _RELAY_PARENT)

from _omnilink_relay.http_security import (  # noqa: E402
    RequestError,
    RequestIdGuard,
    allowed_origins,
    check_authorization,
    check_protocol_version,
    checked_origin,
    configured_token,
    error_envelope,
    finite_number,
    nonempty_string,
    prompt_budget,
    read_json,
    require_field,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)

try:
    from _omnilink_relay import OmniLinkRelay, Tool, is_enabled as omnilink_enabled, get_omni_key
except Exception:
    OmniLinkRelay = None  # type: ignore[assignment]
    Tool = None  # type: ignore[assignment]
    def omnilink_enabled() -> bool: return False
    def get_omni_key() -> str: return ""


from _quadruped_configs import LEGS, QUADRUPED_CONFIGS  # noqa: E402

# Real legged locomotion for the Deep Robotics Lite3 / X30 (`--locomotion`).
# Optional: without the Deep Robotics package the flag degrades to the
# config's walk and says so at start-up.
try:
    import _crawl_motion  # noqa: E402
except Exception as _exc:  # pragma: no cover - missing package
    _crawl_motion = None
    _CRAWL_IMPORT_ERROR = repr(_exc)
else:
    _CRAWL_IMPORT_ERROR = ""

# Bridge leg names -> the crawl model's.
CRAWL_LEG = {"front_left": "FL", "front_right": "FR",
             "rear_left": "RL", "rear_right": "RR"}

# Walk parameters (wave gait + supervisor-driven body translation; see
# omniquad_simple_pose for the longer rationale and physics caveats). The
# deltas are MAGNITUDES; the config's hip_sweep_dir / knee_flex_dir give them
# a sign per leg.
WALK_PERIOD_S = 4.0
SWING_FRACTION = 0.20
SWING_KNEE_DELTA = 0.30        # extra knee flexion at mid-swing
HIP_SWEEP_AMP = 0.18
WALK_VELOCITY_RAMP_S = 2.0     # accelerate from 0 to the config's walk_velocity_ms
COM_SHIFT_AMP_Y = 0.08
SETTLE_HOLD_S = 1.5            # OmniQuad only: hold the `extend` pose before the ramp

# A sit / stand is DONE when it is measured, not when its setpoint is switched.
# Until 2026-10-20 `is_busy()` called the robot idle the moment the target
# changed, so "Lie down, then stand up." ran both legs in the same tick and
# replied "Sit. Stand." while the XGO Mini2S body never went below 0.1332 m
# (it reaches 0.088 m when it really lies down). Settled = every leg joint
# within POSE_TOL_RAD of its setpoint AND the body height steady to
# POSE_Z_STEADY_M, both held for POSE_HOLD_S of sim time; POSE_SETTLE_MAX_S
# of sim time without that is a timeout, reported as one.
POSE_TOL_RAD = math.radians(3.0)
POSE_Z_STEADY_M = 0.002
POSE_HOLD_S = 0.2
POSE_SETTLE_MAX_S = 5.0

LEG_PHASES = {
    "front_right": 0.00,
    "rear_left":   0.25,
    "front_left":  0.50,
    "rear_right":  0.75,
}


def _smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--robot", default="omniquad", choices=sorted(QUADRUPED_CONFIGS))
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--locomotion", default="config", choices=("config", "crawl", "trot"))
    args, _ = p.parse_known_args()
    return args


class QuadrupedBridge:
    def __init__(self, robot: Supervisor, robot_id: str = "omniquad",
                 locomotion: str = "config") -> None:
        self.robot = robot
        self.robot_id = robot_id
        self.cfg = QUADRUPED_CONFIGS[robot_id]
        self.model = self.cfg["model"]
        self.timestep = int(robot.getBasicTimeStep())

        # 12 leg motors, named by the config (URDF joint name + "_motor",
        # falling back to the bare joint name), plus their position sensors
        # so the settle can start from the pose the engine actually spawned.
        self.motors = {}
        self.sensors = {}
        for leg in LEGS:
            for joint in ("hip_x", "hip_y", "knee"):
                name = self.cfg["legs"][leg][joint]
                m = robot.getDevice(name)
                if m is None and name.endswith("_motor"):
                    m = robot.getDevice(name[:-len("_motor")])
                if m is None:
                    print(f"[omnilink_quadruped_bridge] WARNING: missing {name}")
                self.motors[(leg, joint)] = m
                s = m.getPositionSensor() if m is not None else None
                if s is not None:
                    s.enable(self.timestep)
                self.sensors[(leg, joint)] = s
        # Wheel motors (M20 family): velocity-controlled, parked at spawn.
        self.wheels = {}
        for leg, name in self.cfg.get("wheels", {}).items():
            w = robot.getDevice(name)
            if w is None and name.endswith("_motor"):
                w = robot.getDevice(name[:-len("_motor")])
            if w is None:
                print(f"[omnilink_quadruped_bridge] WARNING: missing wheel {name}")
                continue
            w.setPosition(float("inf"))
            w.setVelocity(0.0)
            self.wheels[leg] = w

        # Motion state.
        self.lock = threading.RLock()
        # ("settle"|"crouch"|"stand"|"sit"|"wave"|"walk"|"stop", params)
        self.motion = ("settle", {"t0": robot.getTime()})
        # Per-(leg, joint) pose the settle ramps FROM: the config's `extend`
        # pose when it has one (OmniQuad), else the measured spawn pose.
        self.settle_from: Dict[Tuple[str, str], float] = {}
        self.last_tick_at = time.time()
        # The measured sit / stand (POSE_* above): the setpoint each joint was
        # last given, the pose being tracked, and the last verdict.
        self._applied: Dict[Tuple[str, str], float] = {}
        self.pose_seq = 0
        self.pose_goal: Optional[Dict[str, Any]] = None
        self.pose_result: Optional[Dict[str, Any]] = None
        self._sim_thread: Optional[int] = None

        # Supervisor handles for walk-phase body translation + orientation lock.
        self.self_node = None
        self.translation_field = None
        self.rotation_field = None
        try:
            self.self_node = robot.getSelf()
        except Exception:
            self.self_node = None
        if self.self_node is not None:
            try:
                self.translation_field = self.self_node.getField("translation")
            except Exception:
                self.translation_field = None
            try:
                self.rotation_field = self.self_node.getField("rotation")
            except Exception:
                self.rotation_field = None

        # Called when an OPERATOR halts the robot (relay.cancel_inflight, so a
        # model turn stops issuing the rest of its plan too).
        self.on_operator_halt: List[Any] = []
        # Operator halts, counted BEFORE the halt acts, so route.execute can
        # see one arrive between the legs of a parsed plan and skip the rest.
        self.halt_seq = 0

        # -- Real legged locomotion (--locomotion crawl|trot) --------------
        self.driver = None
        self.locomotion = "config"
        self.places: Dict[str, dict] = {}
        self.walkway_nodes: Dict[str, Tuple[float, float]] = {}
        self.walkway_edges: Dict[str, List[str]] = {}
        self.site_bounds: Optional[Tuple[float, float, float, float]] = None
        self.home_place: Optional[str] = None
        self.last_q: Optional[Dict[str, Tuple[float, float, float]]] = None
        self._last_pose: Optional[Tuple[float, float, float]] = None
        if locomotion != "config":
            self._setup_driver(locomotion)

        # Persistent floating-base anchor (x, y, z), captured lazily on the
        # first body-lock. The body is supervisor-pinned to this every tick so
        # the stiff-legged stance holds upright under any physics backend
        # (without it the unbalanced stance topples under Newton/XPBD; ODE's
        # softer solver merely tolerated it).
        self.body_anchor: Optional[Tuple[float, float, float]] = None

        # wwi outbox.
        self.window_outbox: List[str] = []
        self.window_configured = False

        # -- D1 / D4 / D6: the clock, the ring, the detectors, the hold --
        # ONE installer, shared with the other four bridges. The joint
        # detector is ON: a quadruped has twelve motors with declared limits,
        # and a leg pinned against one is the failure that looks exactly like
        # a stance holding still.
        self.clock = None
        self.events = None
        self.hold = None
        self.fault_detector = None
        self.contact_detector = None
        self.joint_detector = None
        self.sim_time = 0.0
        self.sim_step = 0
        self.fault: Optional[str] = None
        self.surface = "quadruped"
        self.world = str(self.model or "")
        if attach_telemetry is not None:
            attach_telemetry(self, dt_s=self.timestep / 1000.0,
                             robot_id=robot_id, surface="quadruped",
                             contacts=False)
        # PROTOCOL.md 5.2.1. `ungated_paths` is the load-bearing half and is
        # exact for THIS bridge's route table: every verb in it actuates
        # without passing the gate, and always has.
        self.GATED_PATHS = ["/prompt", "/tool"]
        self.UNGATED_PATHS = ["/stop_robot", "/stand", "/sit", "/walk",
                              "/wave", "/reset_to_home"]
        self.capabilities = {
            "model": self.model,
            "safety_gate": (safety_gate_block(
                "quadruped", gated_paths=self.GATED_PATHS,
                ungated_paths=self.UNGATED_PATHS)
                if safety_gate_block is not None else None),
            "events": {
                "endpoint": "GET /events?since=<cursor>&limit=&types=",
                "state_field": "events",
                "types": list(BRIDGE_EVENT_TYPES) if BRIDGE_EVENT_TYPES else [],
            },
            "legs": list(LEGS),
            "joints_per_leg": ["hip_x", "hip_y", "knee"],
            "stand_pose": {leg: {"hip_y": hy, "knee": kn} for leg, (hy, kn) in self.cfg["stand"].items()},
            "sit_pose": {leg: {"hip_y": hy, "knee": kn} for leg, (hy, kn) in self.cfg["sit"].items()},
            "walk": self.cfg["walk"],
            "walk_velocity_ms": self.cfg["walk_velocity_ms"],
            "body_lock": bool(self.cfg["body_lock"]),
            "locomotion": self.locomotion,
            "places": sorted(self.places),
        }

    # ── Real legged locomotion ────────────────────────────────────

    def _setup_driver(self, gait: str) -> None:
        if _crawl_motion is None:
            print("[omnilink_quadruped_bridge] --locomotion %s unavailable (%s); "
                  "using the config's walk" % (gait, _CRAWL_IMPORT_ERROR), flush=True)
            return
        if self.robot_id not in _crawl_motion._dr.ROBOTS:
            print("[omnilink_quadruped_bridge] --locomotion %s: no gait geometry "
                  "for %r (only %s); using the config's walk"
                  % (gait, self.robot_id, ", ".join(sorted(_crawl_motion._dr.ROBOTS))),
                  flush=True)
            return
        terrain = None
        try:
            terrain = _crawl_motion._dr.TerrainMap.load(self.robot, "TERRAIN_GEOM", "TERRAIN")
        except Exception:
            terrain = None
        self.driver = _crawl_motion.CrawlDriver(
            self.robot_id, self.timestep / 1000.0, terrain=terrain, gait=gait)
        self.locomotion = gait
        self.places = self._read_places()
        # The measured verbs exist only in this mode. Bound on the INSTANCE,
        # so route._call reads their real signatures -- and a config-mode
        # robot keeps its old act_walk(), whose missing `distance` the
        # router reports as a dropped slot instead of pretending to honour it.
        self.act_walk = self._act_walk_measured          # type: ignore[assignment]
        self.act_turn = self._act_turn_measured          # type: ignore[attr-defined]
        self.act_go_to = self._act_go_to                 # type: ignore[attr-defined]
        self.act_walk_to = self._act_walk_to             # type: ignore[attr-defined]
        # Halts, new orders and questions all get past the action lock in
        # this mode (do_POST), so a parsed order may wait for its motion and
        # be answered from what was MEASURED (route.execute; ops-bench P7).
        self.replies_after_motion = True
        print("[omnilink_quadruped_bridge] locomotion: %s on real contact physics "
              "(stride %.2f m/s at %.1f Hz; terrain map: %s; %d named place(s))"
              % (gait, self.driver.vx_max, self.driver.gait.freq,
                 "yes" if terrain is not None else "none", len(self.places)), flush=True)

    def _read_places(self) -> Dict[str, dict]:
        """The site, from the robot's customData (JSON):

            {"places":  {"<name>": [x, y, yaw_deg, "<description>", "<node>"], ...},
             "walkway": {"nodes": {"<node>": [x, y], ...},
                         "edges": [["<node>", "<node>"], ...]},
             "bounds":  [x_min, x_max, y_min, y_max]}

        A place's optional fifth entry names the walkway node it is reached
        from; go_to then follows the walkway there instead of cutting
        straight across the site. Nothing here senses obstacles: the walkway
        is the map of where walking is clear."""
        try:
            raw = self.robot.getCustomData() or ""
            data = json.loads(raw) if raw.strip() else {}
        except Exception as exc:
            print("[omnilink_quadruped_bridge] customData is not JSON (%r); no places" % (exc,),
                  flush=True)
            return {}
        walk = data.get("walkway") or {}
        for n, xy in (walk.get("nodes") or {}).items():
            try:
                self.walkway_nodes[str(n)] = (float(xy[0]), float(xy[1]))
            except Exception:
                continue
        for e in walk.get("edges") or []:
            try:
                a, b = str(e[0]), str(e[1])
            except Exception:
                continue
            if a in self.walkway_nodes and b in self.walkway_nodes:
                self.walkway_edges.setdefault(a, []).append(b)
                self.walkway_edges.setdefault(b, []).append(a)
        home = data.get("home")
        if isinstance(home, str) and home:
            self.home_place = home
        try:
            b = data.get("bounds")
            if b:
                self.site_bounds = (float(b[0]), float(b[1]), float(b[2]), float(b[3]))
        except Exception:
            self.site_bounds = None
        out = {}
        for name, spec in (data.get("places") or {}).items():
            try:
                x, y = float(spec[0]), float(spec[1])
                yaw = None if len(spec) < 3 or spec[2] is None else math.radians(float(spec[2]))
                desc = str(spec[3]) if len(spec) > 3 else ""
                node = str(spec[4]) if len(spec) > 4 and spec[4] else None
            except Exception:
                continue
            if node is not None and node not in self.walkway_nodes:
                node = None
            out[str(name)] = {"x": x, "y": y, "yaw": yaw, "description": desc, "node": node}
        return out

    def _walkway_route(self, x0: float, y0: float, goal: Optional[str]) -> List[Tuple[float, float]]:
        """Walkway points from (x0, y0) to node `goal` (shortest path on the
        graph, entered at the node nearest the robot). [] without a graph."""
        if goal is None or not self.walkway_nodes:
            return []
        nodes = self.walkway_nodes

        def d(a, b):
            return math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])

        start = min(nodes, key=lambda n: math.hypot(nodes[n][0] - x0, nodes[n][1] - y0))
        dist = {start: 0.0}
        prev: Dict[str, str] = {}
        todo = set(nodes)
        while todo:
            u = min(todo, key=lambda n: dist.get(n, float("inf")))
            todo.discard(u)
            if u == goal or dist.get(u, float("inf")) == float("inf"):
                break
            for v in self.walkway_edges.get(u, []):
                alt = dist[u] + d(u, v)
                if alt < dist.get(v, float("inf")):
                    dist[v], prev[v] = alt, u
        if goal not in dist:
            return []
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        path.reverse()
        # Already past the entry node on the first edge? Do not walk back to it.
        if len(path) >= 2:
            n0, n1 = nodes[path[0]], nodes[path[1]]
            if math.hypot(n1[0] - x0, n1[1] - y0) < math.hypot(n1[0] - n0[0], n1[1] - n0[1]):
                path = path[1:]
        return [nodes[n] for n in path]

    def _outside_site(self, x: float, y: float) -> bool:
        b = self.site_bounds
        return b is not None and not (b[0] <= x <= b[1] and b[2] <= y <= b[3])

    def _find_place(self, place: str) -> Optional[str]:
        """Exact name, else a case/space/dash-insensitive match, else a
        unique substring. None when it does not name exactly one place."""
        if place in self.places:
            return place

        def norm(t: str) -> str:
            return re.sub(r"[^a-z0-9]", "", t.lower())

        want = norm(place)
        exact = [n for n in self.places if norm(n) == want]
        if len(exact) == 1:
            return exact[0]
        sub = [n for n in self.places if want and (want in norm(n) or norm(n) in want)]
        return sub[0] if len(sub) == 1 else None

    def _stand_target(self, leg: str) -> Tuple[float, float, float]:
        if self.driver is not None:
            return tuple(self.driver.stand_q[CRAWL_LEG[leg]])
        return self._pose(leg, "stand")

    def _read_pose2d(self):
        """SIM THREAD ONLY. (x, y, yaw, (up_x_body, up_y_body)) or None."""
        if self.self_node is None:
            return None
        try:
            pos = self.self_node.getPosition()
            o = self.self_node.getOrientation()
        except Exception:
            return None
        return float(pos[0]), float(pos[1]), math.atan2(o[3], o[0]), (o[6], o[7])

    def _tick_driver(self, sim_t: float) -> None:
        pose = self._read_pose2d()
        if pose is None:
            return
        x, y, yaw, up = pose
        self._last_pose = (x, y, yaw)
        with self.lock:
            q = self.driver.tick(sim_t, x, y, yaw, up_body=up)
        self.last_q = {leg: tuple(q[CRAWL_LEG[leg]]) for leg in LEGS}
        for leg in LEGS:
            self._apply(leg, *self.last_q[leg])

    def _tick_pose_blend(self, sim_t: float, p: Dict[str, Any]) -> None:
        a = _smoothstep((sim_t - p["t0"]) / max(1e-6, p["dur"]))
        q = {leg: tuple(f + (t - f) * a for f, t in zip(p["from"][leg], p["to"][leg]))
             for leg in LEGS}
        self.last_q = q
        for leg in LEGS:
            self._apply(leg, *q[leg])
        if a >= 1.0 and p.get("then"):
            with self.lock:
                if self.motion[0] == "pose_blend":
                    self.motion = (p["then"], {"t0": sim_t})

    def _blend_to(self, target: Dict[str, Tuple[float, float, float]],
                  then: Optional[str], dur: float = 1.2) -> None:
        start = self.last_q or {leg: self._stand_target(leg) for leg in LEGS}
        with self.lock:
            self.motion = ("pose_blend", {"t0": self.sim_time, "dur": dur,
                                          "from": dict(start), "to": dict(target),
                                          "then": then})

    def is_busy(self) -> bool:
        with self.lock:
            kind = self.motion[0]
            # A sit / stand is busy until it is MEASURED settled (POSE_*):
            # switching the setpoint is not arriving.
            if self.pose_goal is not None:
                return True
            if kind == "driver" and self.driver is not None:
                return self.driver.busy() or self.driver.moving()
            return kind not in ("stop", "stand", "sit", "driver")

    # ── Measured sit / stand ─────────────────────────────────────

    def _begin_pose(self, pose: str) -> int:
        """Start tracking `pose` ("sit" / "stand"); -> its seq. Any thread.
        A pose still being tracked is reported superseded by this one."""
        terminal = ("sit",) if pose == "sit" else ("stand", "driver")
        with self.lock:
            prev = self.pose_goal
            if prev is not None:
                self.pose_result = self._pose_verdict(prev, superseded=True)
            self.pose_seq += 1
            self.pose_goal = {"seq": self.pose_seq, "pose": pose,
                              "terminal": terminal, "t0": self.sim_time,
                              "z_start": None, "z_ref": None,
                              "within_since": None, "err": None, "z": None}
            return self.pose_seq

    def _pose_error(self) -> Optional[float]:
        """SIM THREAD. Largest |measured - setpoint| over the leg joints, or
        None when no joint can be read."""
        worst = None
        for key, target in self._applied.items():
            s = self.sensors.get(key)
            if s is None:
                continue
            try:
                v = float(s.getValue())
            except Exception:
                continue
            if not math.isfinite(v):
                continue
            e = abs(v - target)
            worst = e if worst is None else max(worst, e)
        return worst

    def _body_z(self) -> Optional[float]:
        """SIM THREAD. The floating base's height, or None."""
        if self.self_node is None:
            return None
        try:
            return float(self.self_node.getPosition()[2])
        except Exception:
            return None

    def _pose_verdict(self, g: Dict[str, Any], settled: bool = False,
                      timed_out: bool = False, superseded: bool = False) -> dict:
        """The measured result of pose goal `g`: {commanded, achieved, error,
        settled} (docs/developer/tool-design-for-agents.md). Caller holds lock."""
        err = g.get("err")
        z0, z1 = g.get("z_start"), g.get("z")
        out = {"accepted": True, "pose": g["pose"], "seq": g["seq"],
               "commanded": {"pose": g["pose"]},
               "achieved": {
                   "joint_error_max_deg": None if err is None else round(math.degrees(err), 2),
                   "body_z_start_m": None if z0 is None else round(z0, 4),
                   "body_z_m": None if z1 is None else round(z1, 4),
                   "body_dz_m": None if z0 is None or z1 is None else round(z1 - z0, 4)},
               "error": err, "settled": bool(settled), "timed_out": bool(timed_out),
               "settle_s": round(max(0.0, self.sim_time - g["t0"]), 3),
               "sim_time_start": round(g["t0"], 6),
               "sim_time_end": round(self.sim_time, 6)}
        if superseded:
            out["superseded"] = True
            out["note"] = ("a later command took over before this pose settled; "
                           "read get_robot_state for the live pose")
        return out

    def _track_pose(self, sim_t: float) -> None:
        """SIM THREAD, after this tick's setpoints were applied."""
        with self.lock:
            g = self.pose_goal
            kind = self.motion[0]
        if g is None:
            return
        z = self._body_z()
        err = self._pose_error()
        with self.lock:
            if self.pose_goal is not g:
                return
            if g["z_start"] is None:
                g["z_start"] = z
            g["z"], g["err"] = z, err
            if kind not in g["terminal"] and kind != "pose_blend":
                # stop / wave / walk took over: not this pose any more.
                self.pose_result = self._pose_verdict(g, superseded=True)
                self.pose_goal = None
                return
            arrived = kind in g["terminal"] and err is not None and err <= POSE_TOL_RAD
            if arrived and z is not None and g["z_ref"] is not None \
                    and abs(z - g["z_ref"]) > POSE_Z_STEADY_M:
                arrived = False          # joints there, body still moving
            if not arrived:
                g["within_since"], g["z_ref"] = None, None
            elif g["within_since"] is None:
                g["within_since"], g["z_ref"] = sim_t, z
            if g["within_since"] is not None and sim_t - g["within_since"] >= POSE_HOLD_S:
                self.pose_result = self._pose_verdict(g, settled=True)
                self.pose_goal = None
            elif sim_t - g["t0"] > POSE_SETTLE_MAX_S:
                self.pose_result = self._pose_verdict(g, timed_out=True)
                self.pose_goal = None

    def _await_pose(self, seq: int) -> dict:
        """Block (NOT on the sim thread) until pose `seq` is measured."""
        clock = getattr(self, "clock", None)
        budget = clock.budget(POSE_SETTLE_MAX_S + 1.0) if clock is not None else None
        deadline = time.time() + (POSE_SETTLE_MAX_S + 1.0) * 4.0
        while True:
            with self.lock:
                res = self.pose_result
                if res is not None and res.get("seq") == seq:
                    return dict(res)
                if self.pose_seq > seq:
                    return {"accepted": True, "seq": seq, "settled": False,
                            "superseded": True, "achieved": None, "error": None,
                            "note": "a later command took over before this pose settled"}
                g = self.pose_goal
            if budget is not None and budget.stalled(self.last_tick_at):
                return {"accepted": True, "seq": seq, "pose": g and g["pose"],
                        "settled": False, "stalled": True, "achieved": None,
                        "error": None, **budget.result(),
                        "note": "the simulation stopped stepping; nothing could be measured"}
            if (budget is not None and budget.expired()) or \
                    (budget is None and time.time() > deadline):
                with self.lock:
                    if self.pose_goal is not None and self.pose_goal["seq"] == seq:
                        out = self._pose_verdict(self.pose_goal, timed_out=True)
                        return out
                return {"accepted": True, "seq": seq, "settled": False,
                        "timed_out": True, "achieved": None, "error": None}
            time.sleep(0.02)

    def _pose_reply(self, pose: str, wait: bool) -> dict:
        seq = self._begin_pose(pose)
        if wait and threading.get_ident() != self._sim_thread:
            out = self._await_pose(seq)
            out["pose"] = out.get("pose") or pose
            return out
        # Not waited (the sim thread cannot wait on itself): say so, never
        # "done". The verdict lands in get_robot_state's `last_pose`.
        return {"accepted": True, "pose": pose, "seq": seq, "settled": None,
                "commanded": {"pose": pose}, "achieved": None, "error": None,
                "note": "not waited for; read get_robot_state.last_pose"}

    # Longest a motion verb will wait, in sim seconds, whatever it asked.
    WAIT_MAX_S = 600.0

    def _await_driver(self, seq: int, budget_s: float) -> dict:
        """Block until driver motion `seq` reports, measured in SIM steps
        (the mobile bridge's _await_completion, same verdicts)."""
        span = min(max(budget_s, 1.0), self.WAIT_MAX_S)
        clock = getattr(self, "clock", None)
        budget = clock.budget(span) if clock is not None else None
        deadline = time.time() + span * 4.0
        while True:
            with self.lock:
                done = self.driver.completed
                latest = self.driver.seq
            if done is not None and done.get("seq") == seq:
                out = dict(done)
                out.setdefault("timed_out", False)
                return out
            if latest > seq:
                return {"seq": seq, "achieved": None, "error": None, "settled": False,
                        "timed_out": False, "superseded": True,
                        "note": ("a later command superseded this motion before it "
                                 "reported; how far it got was not measured -- read "
                                 "get_robot_state for the live pose")}
            if budget is not None:
                if budget.stalled(self.last_tick_at):
                    return {"seq": seq, "achieved": None, "error": None, "settled": False,
                            "timed_out": False, "stalled": True, **budget.result(),
                            "note": "the simulation stopped stepping while this motion "
                                    "was in flight; nothing could be measured"}
                if budget.expired():
                    break
            elif time.time() > deadline:
                break
            time.sleep(0.05)
        return {"seq": seq, "achieved": None, "error": None, "settled": False,
                "timed_out": True, **(budget.result() if budget is not None else {}),
                "note": "wait budget expired before the motion reported; the robot "
                        "may still be moving -- poll get_robot_state"}

    def _start_driver_verb(self, start) -> int:
        """Start a driver verb from ANY posture. A sitting, waving or frozen
        robot is blended back to the gait's standing pose first; the driver
        holds the verb until the blend hands control back to it."""
        with self.lock:
            kind = self.motion[0]
            self.driver.sim_time = self.sim_time
            seq = start()
        if kind != "driver":
            self._blend_to({leg: self._stand_target(leg) for leg in LEGS},
                           then="driver", dur=1.0)
        return seq

    def _accepted(self, seq: int, commanded, unit: str, eta_s: float) -> dict:
        return {"accepted": True, "seq": seq, "commanded": commanded, "unit": unit,
                "eta_s": round(eta_s, 1),
                "note": "NOT complete -- this returns on acceptance. Pass wait=true, "
                        "or poll get_robot_state until last_motion.seq matches, for "
                        "the achieved value."}

    def _act_walk_measured(self, distance: Optional[float] = None,
                           wait: bool = False) -> dict:
        if distance is None:
            # No default distance (the owner's rule for every drive, 2026-09-23).
            return {"accepted": False, "refused": "distance_required",
                    "error": "no distance was given -- say how far, e.g. "
                             "'walk forward 2 metres'"}
        d = float(distance)
        if not math.isfinite(d) or abs(d) < 0.02:
            return {"accepted": False, "refused": "distance_too_small",
                    "error": f"{d} m is not a walk this gait can measure (min 0.02 m)"}
        seq = self._start_driver_verb(lambda: self.driver.walk(d))
        eta = abs(d) / 0.3 + 3.0
        if wait:
            return {"accepted": True, "commanded": d, "unit": "m",
                    **self._await_driver(seq, eta * 3.0 + 20.0)}
        return self._accepted(seq, d, "m", eta)

    def _act_turn_measured(self, angle_rad: float, wait: bool = False) -> dict:
        a = float(angle_rad)
        if not math.isfinite(a):
            return {"accepted": False, "refused": "bad_angle",
                    "error": "angle is not a number"}
        seq = self._start_driver_verb(lambda: self.driver.turn(a))
        eta = abs(a) / 0.25 + 3.0
        if wait:
            return {"accepted": True, "commanded": a, "unit": "rad",
                    **self._await_driver(seq, eta * 3.0 + 20.0)}
        return self._accepted(seq, a, "rad", eta)

    def _act_walk_to(self, x: float, y: float, yaw_deg: Optional[float] = None,
                     wait: bool = True, label: Optional[str] = None,
                     via: Optional[List[Tuple[float, float]]] = None) -> dict:
        """Walk to (x, y) on the site, optionally ending facing yaw_deg.
        ALWAYS BLOCKING: its value is the pose it reports reaching."""
        if not wait:
            return {"accepted": False, "refused": "wait_false_unsupported",
                    "message": "walk_to is always blocking -- it reports the pose "
                               "it actually reached"}
        if self._outside_site(float(x), float(y)):
            b = self.site_bounds
            return {"accepted": False, "refused": "outside_site_bounds",
                    "error": (f"({float(x):+.2f}, {float(y):+.2f}) is outside the site: "
                              f"x in [{b[0]}, {b[1]}], y in [{b[2]}, {b[3]}] metres")}
        pose = self._last_pose
        yaw = None if yaw_deg is None else math.radians(float(yaw_deg))
        via = list(via or [])
        seq = self._start_driver_verb(
            lambda: self.driver.walk_to(float(x), float(y), yaw, label=label, via=via))
        pts = ([pose[:2]] if pose else []) + via + [(float(x), float(y))]
        dist = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])) or 10.0
        res = self._await_driver(seq, dist / 0.2 * 3.0 + 60.0)
        return {"accepted": True, **res}

    def _act_go_to(self, place: str, wait: bool = True) -> dict:
        name = self._find_place(str(place or ""))
        if name is None:
            return {"accepted": False, "refused": "unknown_place",
                    "error": f"no place called {place!r} on this site",
                    "known_places": sorted(self.places)}
        spec = self.places[name]
        yaw_deg = None if spec["yaw"] is None else math.degrees(spec["yaw"])
        pose = self._last_pose or (0.0, 0.0, 0.0)
        via = self._walkway_route(pose[0], pose[1], spec.get("node"))
        res = self._act_walk_to(spec["x"], spec["y"], yaw_deg, wait=True, label=name, via=via)
        res["place"] = name
        if spec.get("description"):
            res["place_description"] = spec["description"]
        return res

    # ── D4: the joint-limit feed ──────────────────────────────────

    # Every N ticks. Twelve sensor reads is not free, and a leg on its stop
    # is not a millisecond-scale event.
    JOINT_POLL_TICKS = 4

    def _joint_limit_table(self):
        """(names, limits) read ONCE from the motors themselves.

        ⚠️ ONLY THE JOINTS THAT ACTUALLY DECLARE A LIMIT. A URDF import can
        leave `getMinPosition()` reading 0.0 for both ends (recorded in this
        tree: URDF joint limits clamped at +/-pi with getMinPosition()
        returning 0), and a detector comparing a position against (0, 0)
        would fire on every joint on every tick -- a detector that always
        fires is exactly as useless as one that never does, and far noisier.
        So a degenerate pair is DROPPED, and if none survive the detector is
        switched off and says so once rather than pretending to watch.
        """
        table = getattr(self, "_jl_table", None)
        if table is not None:
            return table
        names, limits, sensors = [], [], []
        for leg in LEGS:
            for joint in ("hip_x", "hip_y", "knee"):
                motor = self.motors.get((leg, joint))
                sensor = self.sensors.get((leg, joint))
                if motor is None or sensor is None:
                    continue
                try:
                    lo = float(motor.getMinPosition())
                    hi = float(motor.getMaxPosition())
                except Exception:
                    continue
                if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
                    continue
                names.append(f"{leg}_{joint}")
                limits.append((lo, hi))
                sensors.append(sensor)
        if not names:
            self.joint_detector = None
            print("[omnilink_quadruped_bridge] joint.limit_hit events OFF: "
                  "no motor on this robot declares a usable position limit",
                  flush=True)
        self._jl_table = (names, limits, sensors)
        return self._jl_table

    def _poll_joint_limits(self) -> None:
        """SIM THREAD ONLY. Read the leg joints against their declared stops."""
        if self.joint_detector is None:
            return
        if (self.sim_step % self.JOINT_POLL_TICKS) != 0:
            return
        names, limits, sensors = self._joint_limit_table()
        if not names or self.joint_detector is None:
            return
        try:
            q = [float(s.getValue()) for s in sensors]
        except Exception:
            return
        self.joint_detector.update(names, q, limits, sim_time=self.sim_time,
                                   step=self.sim_step, robot=self.robot_id)

    def queue_window(self, line: str) -> None:
        with self.lock:
            self.window_outbox.append(line)

    def _pose(self, leg: str, base: str, hip_y_delta: float = 0.0, knee_delta: float = 0.0,
              hip_x_extra: float = 0.0) -> Tuple[float, float, float]:
        """Absolute (hip_x, hip_y, knee) for `leg`: the config's `base` pose
        ("stand" / "sit") plus signed deltas. `hip_y_delta` > 0 moves the
        foot backward, `knee_delta` > 0 flexes the knee, whatever the
        robot's joint convention."""
        hy, kn = self.cfg[base][leg]
        return (self.cfg["hip_x"][leg] + hip_x_extra,
                hy + self.cfg["hip_sweep_dir"][leg] * hip_y_delta,
                kn + self.cfg["knee_flex_dir"][leg] * knee_delta)

    def _apply(self, leg: str, hip_x: float, hip_y: float, knee: float) -> None:
        for joint, value in (("hip_x", hip_x), ("hip_y", hip_y), ("knee", knee)):
            m = self.motors[(leg, joint)]
            if m is not None:
                m.setPosition(value)
                self._applied[(leg, joint)] = value

    def _set_all(self, base: str = "stand", hip_y_delta: float = 0.0, knee_delta: float = 0.0,
                 hip_x_extra: float = 0.0) -> None:
        for leg in LEGS:
            self._apply(leg, *self._pose(leg, base, hip_y_delta, knee_delta, hip_x_extra))

    def _capture_settle_from(self) -> None:
        """Where the settle ramp starts: the config's `extend` pose, else the
        joints as the engine spawned them (they start at q=0 clamped into
        range whatever the URDF's rest pose says -- PROVENANCE.md)."""
        extend = self.cfg.get("extend")
        for leg in LEGS:
            for joint in ("hip_x", "hip_y", "knee"):
                if extend is not None:
                    hy, kn = extend[leg]
                    v = {"hip_x": self.cfg["hip_x"][leg], "hip_y": hy, "knee": kn}[joint]
                else:
                    s = self.sensors[(leg, joint)]
                    v = s.getValue() if s is not None else float("nan")
                    if not math.isfinite(v):
                        v = {"hip_x": self.cfg["hip_x"][leg], "hip_y": self.cfg["stand"][leg][0],
                             "knee": self.cfg["stand"][leg][1]}[joint]
                self.settle_from[(leg, joint)] = v

    def _set_wheels(self, velocity_radps: float) -> None:
        for w in self.wheels.values():
            w.setVelocity(velocity_radps)

    def _ensure_anchor(self) -> None:
        """Capture the floating-base anchor once, from the live body pose."""
        if self.body_anchor is not None:
            return
        pos0 = None
        if self.self_node is not None:
            try:
                pos0 = self.self_node.getPosition()
            except Exception:
                pos0 = None
        if pos0 is not None and len(pos0) >= 3:
            self.body_anchor = (float(pos0[0]), float(pos0[1]), float(pos0[2]))
        else:
            self.body_anchor = (0.0, 0.0, 0.85)

    def _write_body_pose(self, x: float, y: float, z: float) -> None:
        """Rewrite the floating-base translation + level orientation.

        Only the body link's pose is rewritten; the leg joints are NOT
        zeroed (that would need Node.resetPhysics), so position-controlled
        gaits/poses keep animating underneath.
        """
        if self.translation_field is None:
            return
        try:
            self.translation_field.setSFVec3f([x, y, z])
            if self.rotation_field is not None:
                self.rotation_field.setSFRotation([0.0, 0.0, 1.0, 0.0])
        except Exception:
            pass

    def _lock_body_upright(self, z_extra: float = 0.0) -> None:
        """Hold the body at its anchor, level — the same supervisor pose-lock
        _tick_walk uses, applied to the standing/idle poses too. Without it the
        stiff-legged stance has no balance feedback and topples under
        Newton/XPBD (it stood only because ODE's softer solver tolerated it)."""
        if self.self_node is None or self.translation_field is None:
            return
        self._ensure_anchor()
        ax, ay, az = self.body_anchor
        self._write_body_pose(ax, ay, az + z_extra)

    def tick(self, sim_t: float) -> None:
        # THE SIM CLOCK IS CACHED HERE AND NOWHERE ELSE: nothing off the sim
        # thread may ask the engine what time it is (MainThreadCalls in the
        # mobile bridge records what threaded supervisor reads cost). It also
        # clears the staleness detector and files a rising-edge fault.
        telemetry_tick(self, sim_t)
        self._sim_thread = threading.get_ident()
        with self.lock:
            kind, p = self.motion
            self.last_tick_at = time.time()
        # D4: a leg pinned against its declared stop. Hysteresis is inside
        # the detector -- a joint parked on a stop jitters at solver noise
        # and would otherwise file an event per tick.
        self._poll_joint_limits()

        if kind == "settle":
            # OmniQuad: hold its `extend` pose for a soft drop, then ramp.
            # Every other robot: ramp from the measured spawn pose at once.
            if not self.settle_from:
                self._capture_settle_from()
            hold = SETTLE_HOLD_S if self.cfg.get("extend") is not None else 0.0
            if sim_t - p["t0"] < hold:
                for leg in LEGS:
                    self._apply(leg, *(self.settle_from[(leg, j)] for j in ("hip_x", "hip_y", "knee")))
            else:
                with self.lock:
                    self.motion = ("crouch", {"t0": sim_t})
        elif kind == "crouch":
            # Smoothstep from the settle-from pose into the stand over settle_s
            # (the gait's own standing pose when --locomotion drives the legs).
            a = _smoothstep((sim_t - p["t0"]) / max(1e-6, float(self.cfg["settle_s"])))
            q = {}
            for leg in LEGS:
                target = self._stand_target(leg)
                start = tuple(self.settle_from[(leg, j)] for j in ("hip_x", "hip_y", "knee"))
                q[leg] = tuple(s + (t - s) * a for s, t in zip(start, target))
                self._apply(leg, *q[leg])
            self.last_q = q
            if a >= 1.0:
                with self.lock:
                    self.motion = ("driver" if self.driver is not None else "stand",
                                   {"t0": sim_t})
        elif kind == "driver":
            self._tick_driver(sim_t)
        elif kind == "pose_blend":
            self._tick_pose_blend(sim_t, p)
        elif kind == "stand":
            sway = 0.04 * math.sin(2 * math.pi * (sim_t - p["t0"]) / 3.0)
            self._set_all("stand", hip_y_delta=sway)
        elif kind == "sit":
            self._set_all("sit")
        elif kind == "wave":
            t = sim_t - p["t0"]
            sway = 0.10 * math.sin(2 * math.pi * 0.8 * t)
            if self.driver is not None:
                # Around the gait's standing pose, so the wave neither starts
                # nor ends with a jump to the config's slightly different one.
                q = {}
                for leg in LEGS:
                    hx, hy, kn = self._stand_target(leg)
                    q[leg] = (hx + sway * 0.5, hy + self.cfg["hip_sweep_dir"][leg] * sway, kn)
                    self._apply(leg, *q[leg])
                self.last_q = q
            else:
                self._set_all("stand", hip_y_delta=sway, hip_x_extra=sway * 0.5)
            if t > p["duration_s"]:
                with self.lock:
                    self.motion = ("driver" if self.driver is not None else "stand",
                                   {"t0": sim_t})
        elif kind == "walk":
            if self.cfg["walk"] == "wheels":
                self._tick_drive(sim_t, p)
            else:
                self._tick_walk(sim_t, p, translate=(self.cfg["walk"] == "gait"))
        elif kind == "stop":
            # Do nothing -- last setpoints stay applied.
            pass
        self._track_pose(sim_t)

        # Pin the floating base upright for every non-walk pose (walk runs its
        # own translation+rotation lock in _tick_walk). This is what keeps OmniQuad
        # standing under Newton/XPBD; see _lock_body_upright. Robots whose config
        # says body_lock False stand on their own physics.
        if kind != "walk" and self.cfg["body_lock"]:
            self._lock_body_upright()

    def _tick_drive(self, sim_t: float, p: Dict[str, Any]) -> None:
        """Wheeled-legged walk: hold the stance and roll the wheels. Real
        physics -- no supervisor writes."""
        self._set_all("stand")
        v_cmd = self.cfg["walk_velocity_ms"] * max(0.0, min(1.0, (sim_t - p["t0"]) / WALK_VELOCITY_RAMP_S))
        self._set_wheels(self.cfg.get("wheel_forward_sign", -1.0) * v_cmd / self.cfg["wheel_radius_m"])

    def _tick_walk(self, sim_t: float, p: Dict[str, Any], translate: bool = True) -> None:
        """Joint-space wave gait (+ supervisor-driven forward translation when
        `translate`).

        Each leg cycles realistically through stance/swing (animated by
        position-controlled motors). With `translate`, the body's X position
        is locked to a straight forward line from the walk-start anchor,
        advancing at the config's walk_velocity_ms (ramped from 0). Pure
        physics walking is not stable on the OmniQuad URDF; supervisor
        translation lets it visibly walk while keeping the gait visually
        realistic. Without it ("march") the legs cycle in place.
        """
        walk_t = sim_t - p["t0"]
        cycle_phase = (walk_t / WALK_PERIOD_S) % 1.0
        amp_scale = max(0.0, min(1.0, walk_t / (2.0 * WALK_PERIOD_S)))

        # Lateral CoM shift via hip_x delta during single-leg swing.
        com_shift_y = -COM_SHIFT_AMP_Y * math.cos(2.0 * math.pi * cycle_phase) * amp_scale

        for leg in LEGS:
            offset = (cycle_phase - LEG_PHASES[leg]) % 1.0
            if offset < SWING_FRACTION:
                # Swing: the foot travels from back (+A) to front (-A); knee flexes.
                pp = offset / SWING_FRACTION
                smooth = pp * pp * (3.0 - 2.0 * pp)
                hip_y_delta = HIP_SWEEP_AMP * (1.0 - 2.0 * smooth) * amp_scale
                knee_delta = SWING_KNEE_DELTA * math.sin(math.pi * pp) * amp_scale
            else:
                # Stance: the foot travels from front (-A) to back (+A); planted
                # foot -> body moves forward in world.
                pp = (offset - SWING_FRACTION) / (1.0 - SWING_FRACTION)
                hip_y_delta = HIP_SWEEP_AMP * (-1.0 + 2.0 * pp) * amp_scale
                knee_delta = 0.0
            self._apply(leg, *self._pose(leg, "stand", hip_y_delta, knee_delta, hip_x_extra=com_shift_y))

        if not translate:
            return
        # Advance the shared body anchor along +x by this tick's ramped
        # distance, then write it (level). Using the persistent anchor (rather
        # than a per-walk start offset) means a later 'stand' holds the
        # advanced position instead of snapping back to the spawn point.
        if self.self_node is not None and self.translation_field is not None:
            self._ensure_anchor()
            v_cmd = self.cfg["walk_velocity_ms"] * max(0.0, min(1.0, walk_t / WALK_VELOCITY_RAMP_S))
            dt = max(0.0, self.timestep / 1000.0)
            ax, ay, az = self.body_anchor
            ax += v_cmd * dt
            self.body_anchor = (ax, ay, az)
            body_bob = 0.015 * math.cos(4.0 * math.pi * cycle_phase) * amp_scale
            self._write_body_pose(ax, ay, az + body_bob)

    # ── Actions ──────────────────────────────────────────────────

    def _halt_driver(self, budget_s: float = 5.0) -> Optional[dict]:
        """Fade the gait out and wait (sim steps) until the robot stands.
        None when the legs were not walking."""
        with self.lock:
            walking = (self.motion[0] == "driver" and self.driver is not None
                       and (self.driver.busy() or self.driver.moving()))
            if not walking:
                return None
            self.driver.sim_time = self.sim_time
            seq = self.driver.halt()
        return self._await_driver(seq, budget_s)

    def working(self, relay: Any = None) -> bool:
        """Is the robot moving, or a model turn still running for it?"""
        if self.is_busy():
            return True
        return bool(relay is not None and getattr(relay, "turn_active", None)
                    and relay.turn_active())

    def act_stop(self, wait: bool = True, operator: bool = True) -> dict:
        """`operator` False when a MODEL calls stop_robot as a tool: that
        must not cancel the model's own turn."""
        if self.driver is not None:
            # A walking robot cannot freeze mid-stride: two feet in the air
            # is not a pose it can hold. Stop = fade the gait into the
            # standing pose (always statically stable) and MEASURE that it
            # came to rest.
            if getattr(self, "hold", None) is not None:
                self.hold.request_release("stop_robot")
            if operator:
                self.halt_seq += 1
            for cb in (list(self.on_operator_halt) if operator else []):
                try:
                    cb()
                except Exception:
                    pass
            if not wait:
                # The robot window's Stop runs on the SIM THREAD, which is
                # what has to advance the fade: waiting here would deadlock.
                with self.lock:
                    if self.driver.busy() or self.driver.moving():
                        self.driver.sim_time = self.sim_time
                        self.driver.halt()
                    elif self.motion[0] in ("wave", "pose_blend"):
                        self.motion = ("stop", {})
                return {"halted_at": time.time(), "stationary": None,
                        "measured": {"reason": "stop issued; the gait fades into "
                                               "the standing pose over 1.5 s"}}
            res = self._halt_driver()
            if res is None:
                with self.lock:
                    if self.motion[0] in ("wave", "pose_blend"):
                        self.motion = ("stop", {})
                    # Not walking -- but still say what the body is doing, as
                    # measured: the driver differences the pose every tick.
                    v = abs(self.driver.v_meas)
                return {"halted_at": time.time(), "stationary": v < 0.02,
                        "measured": {"speed_mps": v, "over_s": self.driver.RATE_WINDOW_S,
                                     "note": "the legs were not walking"}}
            with self.lock:
                v = abs(self.driver.v_meas)
            still = bool(res.get("settled")) and v < 0.02
            return {"halted_at": time.time(), "stationary": still,
                    "measured": {"speed_mps": v, "over_s": self.driver.RATE_WINDOW_S},
                    "pose": res.get("pose")}
        return self._act_stop_config()

    def _act_stop_config(self) -> dict:
        # D6: STOP ALWAYS RUNS. Under lockstep the world is frozen between
        # commands; this asks the LOOP to lift the hold. A flag, not a call:
        # releasing touches simulationSetMode, and this runs on an HTTP
        # thread where that is the unsafe call MainThreadCalls forbids.
        if getattr(self, "hold", None) is not None:
            self.hold.request_release("stop_robot")
        with self.lock:
            self.motion = ("stop", {})
        self._set_wheels(0.0)
        return {"halted_at": time.time()}

    def act_stand(self, wait: bool = True) -> dict:
        """Stand, and report the MEASURED pose ({commanded, achieved, error,
        settled}) once it settles or times out. `wait=False` returns at once
        with `settled: None` -- never a claim that it stood."""
        if self.driver is not None:
            self._halt_driver()
            with self.lock:
                kind = self.motion[0]
            if kind != "driver":
                self._blend_to({leg: self._stand_target(leg) for leg in LEGS},
                               then="driver", dur=1.2)
            return self._pose_reply("stand", wait)
        with self.lock:
            # ⚠️ THE CACHED CLOCK, NOT `robot.getTime()`. This runs on an
            # HTTP worker, and the controller API is not thread-safe -- a
            # supervisor read from here is the call that dragged the
            # warehouse demo to ~0.2x realtime (MainThreadCalls).
            self.motion = ("stand", {"t0": self.sim_time})
        self._set_wheels(0.0)
        return self._pose_reply("stand", wait)

    def act_sit(self, wait: bool = True) -> dict:
        """Sit / lie down; measured like act_stand."""
        if self.driver is not None:
            self._halt_driver()
            self._blend_to({leg: self._pose(leg, "sit") for leg in LEGS},
                           then="sit", dur=1.5)
            return self._pose_reply("sit", wait)
        with self.lock:
            self.motion = ("sit", {})
        self._set_wheels(0.0)
        return self._pose_reply("sit", wait)

    def act_wave(self, duration_s: float = 6.0) -> dict:
        if self.driver is not None:
            self._halt_driver()
        with self.lock:
            self.motion = ("wave", {"t0": self.sim_time,
                                    "duration_s": duration_s})
        return {"accepted": True, "duration_s": duration_s}

    def act_walk(self) -> dict:
        with self.lock:
            self.motion = ("walk", {"t0": self.sim_time})
        return {"accepted": True, "pose": "walk", "walk": self.cfg["walk"],
                "velocity_ms": self.cfg["walk_velocity_ms"]}

    def act_reset_to_home(self, wait: bool = True) -> dict:
        # With real legs, "go home" is a walk to the site's declared home
        # place (customData "home"), measured like any go_to -- never a
        # teleport, and never a stand that the reply calls "home".
        if self.driver is not None and self.home_place in self.places:
            return self._act_go_to(self.home_place)
        return self.act_stand()

    def get_state(self) -> dict:
        with self.lock:
            kind = self.motion[0]
        pos = None
        yaw = None
        if self.self_node is not None:
            try:
                pos = [float(v) for v in self.self_node.getPosition()]
            except Exception:
                pos = None
            try:
                # Webots orientation is a 9-element row-major rotation matrix;
                # Z-up convention puts yaw at atan2(o[3], o[0]), same as the
                # mobile bridge's yaw_from_orientation().
                o = self.self_node.getOrientation()
                yaw = math.atan2(float(o[3]), float(o[0]))
            except Exception:
                yaw = None
        return {
            "id": self.robot_id,
            "model": self.model,
            "mode": kind,
            # SIM SECONDS (PROTOCOL.md 5.3), with the wall clock in its own
            # field. `last_tick_at` used to be time.time() on every bridge,
            # so a client differencing it against `sim_time` got the age of
            # the Unix epoch.
            "sim_time": self.sim_time,
            "last_tick_at": self.sim_time,
            "wall_time": self.last_tick_at,
            "step": self.sim_step,
            "fault": self.fault,
            "held": bool(getattr(self.hold, "held", False)),
            "events": events_summary(self),
            "position": pos,
            # ⚠️ x / y / yaw are the COMMON POSE CONTRACT every bridge owes a
            # caller that wants to know whether the robot moved. `position`
            # alone was not enough: a harness reading s.get("x", 0) against a
            # bridge that does not publish x cannot tell "did not move" from
            # "cannot see this robot", and the second one scores as a clean
            # safety result. See docs/developer/v9-release-plan.md (the v9
            # generalization plan this used to cite was folded into it).
            "x": None if pos is None else pos[0],
            "y": None if pos is None else pos[1],
            "z": None if pos is None else pos[2],
            "yaw": yaw,
            # The last MEASURED sit / stand verdict, and whether one is still
            # being tracked (`pose_settling`).
            "last_pose": self.pose_result,
            "pose_settling": self.pose_goal is not None,
            **self._locomotion_state(),
        }

    def _locomotion_state(self) -> dict:
        if self.driver is None:
            return {"locomotion": self.locomotion}
        with self.lock:
            last = dict(self.driver.completed) if self.driver.completed else None
            return {"locomotion": self.locomotion,
                    "busy": self.driver.busy() or self.driver.moving(),
                    "speed_mps": round(self.driver.v_meas, 3),
                    "yaw_rate_rps": round(self.driver.w_meas, 3),
                    "last_motion": last,
                    "nearest_place": self._nearest_place(),
                    "places": {n: {"x": p["x"], "y": p["y"], "description": p["description"]}
                               for n, p in self.places.items()}}

    def _nearest_place(self) -> Optional[dict]:
        """The named place closest to the robot's last pose, with the distance,
        so "where are you?" can be answered in the site's own words."""
        pose = self._last_pose
        if pose is None or not self.places:
            return None
        name = min(self.places, key=lambda n: math.hypot(self.places[n]["x"] - pose[0],
                                                        self.places[n]["y"] - pose[1]))
        p = self.places[name]
        return {"name": name, "distance_m": round(math.hypot(p["x"] - pose[0], p["y"] - pose[1]), 2)}


# ── Intent ───────────────────────────────────────────────────────────

# The deterministic interpreter. Optional: a bare clone without the bridges
# package installed loses language control entirely -- there is no keyword
# ladder under it any more (deleted 2026-09-22) and none may return.
#
# WIRED into both live `/prompt` entry paths. Order: access check ->
# parser -> model -> gate. The OmniKey check runs in front of the parser,
# and the gate is inside `route.execute`, so a parser-produced frame is
# vetted on the same "quadruped" rail a model-produced one is.
try:
    from omnisim_bridges.route import reply_payload as _reply_payload
    from omnisim_bridges.route import short_circuit as _shared_short_circuit
    from omnisim_bridges.route import parser_first_window as _shared_parser_window
    from omnisim_bridges.route import stamp_via as _shared_stamp_via
except ImportError:
    _reply_payload = None
    _shared_short_circuit = None
    _shared_parser_window = None

# Halt preemption and interrupt-first (the mobile bridge's ops-bench P1 / P8),
# used in --locomotion mode: a walk takes tens of seconds, and a "stop" or a
# change of plan must not queue behind it.
try:
    from omnisim_bridges.route import is_halt_order as _shared_is_halt_order
    from omnisim_bridges.route import interrupts_motion as _shared_interrupts_motion
    from omnisim_bridges.route import with_halt_note as shared_with_halt
except ImportError:
    _shared_is_halt_order = None
    _shared_interrupts_motion = None

    def shared_with_halt(halted, out):  # type: ignore[misc]
        return out

    def _shared_stamp_via(payload, default="relay"):  # type: ignore[misc]
        return payload

# D1/D4/D6: the two clocks, the event ring and its detectors, the hold.
# Optional exactly like everything else the package provides -- a bare clone
# keeps every motion verb and simply cannot report events, which is the
# honest degradation: the surface is ABSENT rather than present and silent.
try:
    from omnisim_bridges.bridge_base import (
        attach_relay,
        close_relay,
        attach_telemetry,
        events_summary,
        profile_extras,
        safety_gate_block,
        serve_events,
        telemetry_tick,
    )
    from omnisim_bridges.events import BRIDGE_EVENT_TYPES
except ImportError:
    attach_relay = None
    close_relay = None  # type: ignore[assignment]
    attach_telemetry = None
    profile_extras = None
    BRIDGE_EVENT_TYPES = ()
    safety_gate_block = None
    serve_events = None

    def telemetry_tick(bridge, sim_time=None):  # type: ignore[misc]
        return None

    def events_summary(bridge):  # type: ignore[misc]
        return {"total": 0, "last": None, "next_since": 0, "dropped": 0}


# ── HTTP ─────────────────────────────────────────────────────────────

def _json_finite(obj: Any) -> Any:
    """Recursively replace non-finite floats (NaN/Inf) with None.

    Sensor / clock reads can be NaN before the first robot.step()
    completes (the HTTP server is up BEFORE the main loop starts), and
    json.dumps happily emits bare `NaN` -- which is NOT valid JSON, so
    clients (jq, JSON.parse, json.loads) reject the whole body and the
    endpoint LOOKS empty/broken."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _json_finite(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_finite(v) for v in obj]
    return obj


from omnisim_bridges.access import connection_error, chat_config, reject_window_prompt

# THE ONE /tool implementation. Do not copy it back in here: five
# near-identical handlers, each with its own fail-closed wrapper, is
# how a gated bridge_base came to cover none of the bridges.
from omnisim_bridges.bridge_base import serve_tool


def make_handler(bridge: QuadrupedBridge, relay: Any = None):
    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    trusted_origins = allowed_origins()
    bridge_token = configured_token()

    class _H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _json(self, code, obj):
            # allow_nan=False guarantees strictly valid JSON on the wire;
            # the sanitizer maps any NaN/Inf field to null instead.
            try:
                data = json.dumps(obj, default=str, allow_nan=False).encode("utf-8")
            except ValueError:
                data = json.dumps(_json_finite(obj), default=str,
                                  allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            origin = getattr(self, "_response_origin", None)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            return read_json(self, allow_empty=True)

        def _guard(self) -> None:
            check_protocol_version(self.headers)
            self._response_origin = checked_origin(self.headers, trusted_origins)
            check_authorization(self.headers, bridge_token)

        def do_OPTIONS(self):
            try:
                self._response_origin = checked_origin(self.headers, trusted_origins)
                self.send_response(204)
                if self._response_origin:
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header(
                    "Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token"
                )
                self.end_headers()
            except RequestError as exc:
                self._json(exc.status, error_envelope(exc.code, exc.message, exc.details))

        # Top-level guards: an uncaught exception in a route used to
        # propagate into socketserver, which logs it server-side and closes
        # the connection with ZERO bytes sent -- the client sees an empty
        # reply (curl error 52) with no clue why. Every route now answers
        # with a JSON body: data, or a clear {"error": ...}.
        def do_GET(self):
            try:
                self._guard()
                self._route_get()
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:
                self._error_response(e)

        def do_POST(self):
            try:
                self._guard()
                path = self.path.split("?", 1)[0].rstrip("/") or "/"
                body = self._read_json()
                request_id = validate_request_id(body.pop("id", None))
                if path not in ("/state", "/get_robot_state", "/list_robots", "/capabilities"):
                    request_ids.claim(path, request_id)
                # --locomotion only (the config-mode demos keep their exact
                # behaviour): a HALT however phrased skips the lock the running
                # walk holds, and a NEW instruction while the robot works stops
                # the work first; a question is answered beside it.
                halt = aside = False
                self._halted_first = None
                if bridge.driver is not None and path == "/prompt" and relay is not None:
                    text = body.get("text")
                    halt = bool(_shared_is_halt_order is not None
                                and _shared_is_halt_order(text, "quadruped"))
                    if not halt and _shared_interrupts_motion is not None and bridge.working(relay):
                        if _shared_interrupts_motion(text, "quadruped"):
                            res = bridge.act_stop()
                            self._halted_first = {"pose": res.get("pose"),
                                                  "stationary": res.get("stationary")}
                        else:
                            aside = True
                bypass = (path == "/stop_robot" or halt or aside
                          or (bridge.driver is not None and path == "/tool"
                              and body.get("tool") == "stop_robot"))
                if bypass:
                    self._route_post(body)
                else:
                    with action_lock:
                        self._route_post(body)
            except RequestError as e:
                self._json(e.status, error_envelope(e.code, e.message, e.details))
            except Exception as e:
                self._error_response(e)

        def _error_response(self, e: Exception) -> None:
            import traceback
            print(f"[omnilink_quadruped_bridge] HTTP {self.command} {self.path} "
                  f"failed: {e!r}\n{traceback.format_exc()}")
            try:
                self._json(500, error_envelope("internal_error", "The bridge could not complete the request."))
            except Exception:
                pass  # headers already sent / socket gone -- nothing to add

        def _route_get(self):
            if self.path == "/protocol":
                return self._json(200, {
                    "ok": True, "omnisim_wire": WIRE_VERSION,
                    "service": WIRE_SERVICE,
                    "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                    "instance": {"name": "omnilink_quadruped_bridge", "robot_id": bridge.robot_id},
                    "extensions": [],
                })
            if self.path in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if self.path.split("?", 1)[0].rstrip("/") == "/events":
                # D4. Same envelope as the harness's /sim/events, so one
                # client loop drains both: {events, next_since, dropped}.
                if serve_events is None:
                    return self._json(501, error_envelope(
                        "not_supported", "this bridge serves no event ring"))
                return self._json(200, serve_events(bridge, self.path))
            if self.path in ("/capabilities", "/list_robots"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.model,
                    "capabilities": bridge.capabilities,
                }])
            if self.path == "/usage":
                if relay is None:
                    return self._json(200, {"enabled": False})
                return self._json(200, {
                    "enabled": True,
                    "latest": relay.latest_usage(),
                })
            return self._json(404, error_envelope("not_found", "Endpoint not found."))

        def _route_post(self, body):
            p = self.path.rstrip("/")
            if p == "/prompt" and relay is None:
                return self._json(401 if connection_error()["error"] == "omnikey_required" else 503, connection_error())
            if p in ("/state", "/get_robot_state"):
                return self._json(200, bridge.get_state())
            if p in ("/list_robots", "/capabilities"):
                return self._json(200, [{
                    "id": bridge.robot_id, "model": bridge.model,
                    "capabilities": bridge.capabilities,
                }])
            if p == "/stop_robot":
                return self._json(200, bridge.act_stop())
            if p == "/reset_to_home":
                return self._json(200, bridge.act_reset_to_home())
            if p == "/prompt":
                text = nonempty_string(require_field(body, "text"), "text")
                # A walked inspection round outlasts the ordinary chat wait.
                # Same bound as the mobile bridge: positive, at most 600 s.
                # Counted on the SLOWER of the sim and wall clocks, with
                # `wall_timeout_s` as the hard wall cap (dispatch_sync).
                prompt_timeout_s, prompt_wall_s = prompt_budget(body)
                if relay is not None:
                    # ── PARSER FIRST ──────────────────────────────────
                    # Reached only WITH a relay: the access check at the
                    # top of _route_post already refused a keyless prompt,
                    # so this can never become a keyless path. A non-None
                    # result is a confident, exactly-parsed order the gate
                    # (inside route.execute) has already vetted on the
                    # "quadruped" rail; no model is called for it.
                    # A CREWMATE's message (omnisim_bridges.crew) always reaches the model:
                    # the parser would answer the sender, who never hears that answer,
                    # and only the model can message back. The gate still vets it.
                    if body.get("crew"):  # a crewmate's DETAILS go to the model's liveContext, not the turn
                        try:
                            from omnisim_bridges.crew import note_incoming
                            note_incoming(bridge, body)
                        except Exception:
                            pass
                    _early = (_shared_short_circuit(bridge, text, "quadruped")
                              if _shared_short_circuit is not None
                              and not body.get("crew") else None)
                    if _early is not None:
                        return self._json(200, _shared_stamp_via(shared_with_halt(
                            getattr(self, "_halted_first", None),
                            _reply_payload(_early.get("agent", ""),
                                           _early.get("tools") or [],
                                           via="parser"))))
                    # §5.7.2 / D3: `via` is REQUIRED on a 200 from /prompt.
                    # The parser stamps itself; anything reaching here was
                    # answered by the model relay.
                    return self._json(
                        200, _shared_stamp_via(shared_with_halt(
                            getattr(self, "_halted_first", None),
                            relay.dispatch_sync(
                                text, timeout_s=prompt_timeout_s,
                                sim_clock=lambda: bridge.sim_time,
                                wall_timeout_s=prompt_wall_s))))
                return self._json(503, connection_error())
            if p == "/tool":
                # Platform-side tool callback. omnilink-agents.com web UI
                # POSTs {"tool": "<name>", ...args} after producing tool
                # calls on its side; dispatch via the relay-registered Tool.
                tool_name = nonempty_string(require_field(body, "tool"), "tool")
                # ⚠️ ONE IMPLEMENTATION, in bridge_base.serve_tool: strip the
                # transport fields, refuse an unregistered tool, vet with THIS
                # bridge's surface, fail closed, dispatch. Each bridge used to
                # own a copy of that sequence, so gating the reference
                # implementation did NOT cover them -- measured 2026-09-21,
                # when a gated bridge_base still let
                # POST /tool {"tool":"drive_forward","distance":300} through a
                # bridge's own handler and the call hung for 90 s.
                # `bridge=` is D4: a gate refusal here is filed as a
                # `gate.refused` event, which is otherwise the one thing on
                # this path that nobody ever sees -- the caller gets a 400
                # and the robot's own agent learns nothing at all.
                code, payload = serve_tool(
                    tool_name, body,
                    lambda args: relay.tools[tool_name].dispatch(args),
                    surface="quadruped", bridge=bridge, origin="tool",
                    registered=(relay is not None
                                and tool_name in getattr(relay, "tools", {})))
                return self._json(code, payload)
            return self._json(404, error_envelope("not_found", "Endpoint not found.", {"path": p}))
    return _H


def start_http(bridge: QuadrupedBridge, port: int, relay: Any = None):
    server = ThreadingHTTPServer(("127.0.0.1", port), make_handler(bridge, relay))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[omnilink_quadruped_bridge] HTTP on http://127.0.0.1:{port}")


def build_locomotion_tools(bridge: QuadrupedBridge) -> List[Any]:
    """The measured tool set of `--locomotion crawl|trot`. Every motion tool
    blocks until the robot has stopped and reports what it measured."""
    places = sorted(bridge.places)
    place_lines = "; ".join(
        f"{n}" + (f" ({bridge.places[n]['description']})" if bridge.places[n]["description"] else "")
        for n in places)
    tools = [
        Tool(name="walk", physical=True, surface="quadruped",
             description=("Walk straight along the current heading by a signed distance in "
                          "metres (negative = walk backwards), holding the line. Blocks until "
                          "the robot has stopped; returns the MEASURED distance (achieved), "
                          "the error and settled. Nothing senses obstacles on the way."),
             parameters={"type": "object",
                         "properties": {"distance": {"type": "number",
                                                     "description": "metres, signed"}},
                         "required": ["distance"]},
             dispatch=lambda args: bridge.act_walk(distance=args.get("distance"), wait=True)),
        Tool(name="turn", physical=True, surface="quadruped",
             description=("Turn in place by a signed angle in RADIANS: positive = left "
                          "(counter-clockwise), negative = right. Blocks until settled; "
                          "returns the measured angle."),
             parameters={"type": "object",
                         "properties": {"angle_rad": {"type": "number"}},
                         "required": ["angle_rad"]},
             dispatch=lambda args: bridge.act_turn(angle_rad=args.get("angle_rad"), wait=True)),
    ]
    if places:
        tools.append(Tool(
            name="go_to", physical=True, surface="quadruped",
            description=("Walk to a named place on this site, following the marked walkway, "
                         "and stop facing what is there. Blocks until arrived; returns the "
                         "measured final position and error. Places: " + place_lines),
            parameters={"type": "object",
                        "properties": {"place": {"type": "string", "enum": places}},
                        "required": ["place"]},
            dispatch=lambda args: bridge.act_go_to(place=args.get("place"))))
    tools.append(Tool(
        name="walk_to", physical=True, surface="quadruped",
        description=("Walk in a straight line to site coordinates (x, y) in metres, optionally "
                     "ending facing yaw_deg (0 = +x, 90 = +y). Prefer go_to for named places: "
                     "a straight line is not checked for obstacles."),
        parameters={"type": "object",
                    "properties": {"x": {"type": "number"}, "y": {"type": "number"},
                                   "yaw_deg": {"type": "number"}},
                    "required": ["x", "y"]},
        dispatch=lambda args: bridge.act_walk_to(args.get("x"), args.get("y"),
                                                 args.get("yaw_deg"))))
    return tools


def build_quadruped_tools(bridge: QuadrupedBridge) -> List[Any]:
    if Tool is None:
        return []
    if bridge.driver is not None:
        return build_locomotion_tools(bridge) + [
            Tool(name="stand", description="Stop walking and stand still.",
                 parameters={"type": "object", "properties": {}},
                 dispatch=lambda args: bridge.act_stand()),
            Tool(name="sit", description="Stop and crouch down low.",
                 parameters={"type": "object", "properties": {}},
                 dispatch=lambda args: bridge.act_sit()),
            Tool(name="wave", description="A ~6 s body sway -- a hello gesture.",
                 parameters={"type": "object", "properties": {}},
                 dispatch=lambda args: bridge.act_wave()),
            Tool(name="stop_robot", description=(
                    "Stop now: the gait fades into the standing pose and the reply says "
                    "whether the robot was measured at rest."),
                 parameters={"type": "object", "properties": {}},
                 dispatch=lambda args: bridge.act_stop(operator=False)),
            Tool(name="get_robot_state", description=(
                    "Position (x, y), heading (yaw, rad), speed, whether it is moving, the "
                    "last motion's measured result, and the named places."),
                 parameters={"type": "object", "properties": {}},
                 dispatch=lambda args: bridge.get_state()),
        ]
    return [
        Tool(name="stand", description="Hold a standing stance with gentle sway.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_stand()),
        Tool(name="sit", description="Crouch / sit pose (knees bent further).",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_sit()),
        Tool(name="wave", description="Exaggerated sway for ~6 s -- 'hello' gesture.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_wave()),
        Tool(name="walk", description=(
                {"gait": "Walk forward at %.2f m/s with a wave-gait leg cycle.",
                 "wheels": "Drive forward at %.2f m/s on the wheels, legs held in the stance.",
                 "march": "Cycle the legs through a walking gait in place (%.2f m/s of body motion)."}
                [bridge.cfg["walk"]] % bridge.cfg["walk_velocity_ms"]
                + " Continues until 'stand' or 'stop' is called."),
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_walk()),
        Tool(name="stop_robot", description="Freeze the legs at their current pose.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.act_stop()),
        Tool(name="get_robot_state", description="Read current pose mode.",
             parameters={"type": "object", "properties": {}},
             dispatch=lambda args: bridge.get_state()),
    ]


def setup_omnilink_relay(bridge: QuadrupedBridge, http_port: int = 8765) -> Optional[Any]:
    if OmniLinkRelay is None or not omnilink_enabled():
        return None
    try:
        agent_name = f"OmniSim-{bridge.robot_id}"
        tools = build_quadruped_tools(bridge)
        # A crew channel when the world declares one (OMNILINK_CREW).
        try:
            from omnisim_bridges.crew import crew_tools
            tools = list(tools) + crew_tools(bridge, http_port)
        except Exception as e:
            print(f"[crew] not registered: {e}", flush=True)
        if bridge.driver is not None:
            site = "; ".join(
                f"{n}: {p['description'] or 'a place'}" for n, p in sorted(bridge.places.items()))
            main_task = (
                f"You operate a {bridge.model} four-legged robot in OmniSim through the "
                "OmniLink bridge. It walks on its own legs with a "
                f"{bridge.locomotion} gait at about 0.3 m/s and turns in place. "
                + (f"Named places on this site: {site}. " if site else "")
                + "Use go_to for a named place (it follows the walkway), walk for a "
                "distance along the current heading, turn for an angle in radians "
                "(positive = left). For an order with several steps, call the tools one "
                "after another in the order given; each call blocks until the robot has "
                "stopped and returns what was MEASURED. Report measured results, never the "
                "numbers you asked for. If the operator's order is ambiguous or names a "
                "place that is not on the list, ask instead of guessing. The robot has no "
                "obstacle sensing: never walk it off the walkway on a guess. A question "
                "(where are you, what can you do, what is at a place) is answered from "
                "get_robot_state and the place list and never moves the robot. Keep "
                "replies to one or two short sentences."
            )
        else:
            main_task = (
                f"You operate the {bridge.model} in OmniSim through the OmniLink bridge. "
                f"Available actions: stand, sit, wave, walk ({bridge.cfg['walk']}, "
                f"{bridge.cfg['walk_velocity_ms']:.2f} m/s), stop_robot. Translate operator "
                "requests into one tool call. Keep responses short."
            )
        relay = OmniLinkRelay(
            omni_key=get_omni_key(),
            agent_name=agent_name,
            main_task=main_task,
            tools=tools,
            # The robot CLASS this bridge serves. The relay hands it to
            # gate.register_tools(), so every tool below is registered
            # against this surface and judged on its own magnitude rail
            # instead of the strictest one.
            surface="quadruped",
        )
        # Push a OmniQuad profile so operators can chat to it from the
        # omnilink-agents.com web UI; tool calls round-trip via /tool.
        from _omnilink_relay import profile_sync
        if profile_sync.is_enabled():
            profile_sync.ensure_profile(
                client=relay._client,
                agent_name=agent_name,
                main_task=main_task,
                tool_defs=[t.to_definition() for t in tools],
                engine=relay.engine,
                tool_callback_url=f"http://127.0.0.1:{http_port}/tool",
                **(profile_extras(bridge, http_port=http_port,
                                  surface="quadruped")
                   if profile_extras is not None else {}),
            )
            relay.set_presence_endpoint(
                f"http://127.0.0.1:{http_port}/tool",
                robot=str(bridge.robot_id),
            )
        # -- Plan D4 step 5 / D5: the relay seam ----------------------
        # WITHOUT attach_bridge THE WHOLE EVENT LOOP IS DEAD CODE: the relay
        # owns the wake dispatcher and the presence heartbeat and reads the
        # bridge through this one handle, so no handle means no ring, no
        # wakes, and a beat that reports a held or faulted robot as healthy.
        # Centralised in bridge_base.attach_relay so a bridge added later
        # cannot quietly miss it.
        #
        # The event sink is the SAME callback an operator's window turn
        # uses, because a wake IS an ordinary turn: same cascade, same gate,
        # same memory write, and only the author of the sentence differs.
        #
        # WARNING, the window raiser: it is invoked from the PRESENCE
        # thread. It must not touch the Robot API -- `queue_window` appends
        # under the bridge's own lock and the SIM THREAD drains the outbox,
        # so the marshalling is the outbox itself.
        if attach_relay is not None:
            attach_relay(
                relay, bridge,
                event_sink=lambda k, p: _on_relay_event(bridge, k, p),
                window_raiser=lambda: bridge.queue_window(
                    "system:the platform asked for your attention"))
        if hasattr(relay, "cancel_inflight"):
            bridge.on_operator_halt.append(relay.cancel_inflight)
        print(f"[omnilink_quadruped_bridge] OmniLink relay ON (agent='{agent_name}')")
        try:  # a crew's details reach the model as liveContext (omnisim_bridges.crew)
            from omnisim_bridges.crew import attach_relay as crew_attach
            crew_attach(relay, bridge)
        except Exception:
            pass
        return relay
    except Exception as e:
        print(f"[omnilink_quadruped_bridge] OmniLink relay setup failed: {e}")
        return None


def push_configure(bridge: QuadrupedBridge, relay: Any) -> None:
    agent_label = (
        f"OmniLink relay ({_os.environ.get('OMNILINK_ENGINE', 'g4-engine')})"
        if relay is not None else "OmniLink connection required"
    )
    cfg = {
        "robot": bridge.model,
        "robot_class": "quadruped",
        "agent": agent_label,
        **chat_config(relay),
        "suggestions": (
            ["walk forward 2 metres", "turn left 90 degrees"]
            + ([f"go to {sorted(bridge.places)[0]}"] if bridge.places else [])
            + ["sit", "stop"]
            if bridge.driver is not None else
            ["stand", "sit", "wave hello",
             "drive forward" if bridge.cfg["walk"] == "wheels" else "walk", "stop"]),
    }
    bridge.queue_window("configure:" + json.dumps(cfg))
    bridge.queue_window("status:connected")
    bridge.window_configured = True


def _on_relay_event(bridge: QuadrupedBridge, kind: str, payload: Dict[str, Any]) -> None:
    if kind == "status":
        bridge.queue_window(f"status:{payload.get('state', 'idle')}")
    elif kind == "tool":
        bridge.queue_window(
            f"tool:{payload.get('name', '?')}:{payload.get('status', 'ok')}:{payload.get('summary', '')}")
    elif kind == "agent":
        bridge.queue_window("agent:" + str(payload.get("text", "")))
    elif kind == "usage":
        bridge.queue_window("usage:" + json.dumps(payload, default=str))
    elif kind == "audio_out":
        bridge.queue_window("audio_out:" + json.dumps(payload, default=str))
    elif kind == "error":
        bridge.queue_window("error:" + str(payload.get("text", "")))


def _parser_first_window(bridge: QuadrupedBridge, relay: Any, text: str,
                         to_model: Any, spawn: Any = None) -> bool:
    """Parser-first on the robot-window path. True = the turn is taken.

    ⚠️ handle_wwi runs on the SIM THREAD, so the shared helper decides
    here (pure regex) and executes on a worker: a compound order asks its
    first motion to BLOCK, and blocking here deadlocks the sim that has to
    advance it.

    ⚠️ The caller checks `relay is None` FIRST and refuses. This is never
    reached without an OmniKey, and must never be made reachable without
    one -- see the 2026-09-22 access policy.
    """
    if _shared_parser_window is None or relay is None:
        return False
    try:
        return bool(_shared_parser_window(
            bridge, text, "quadruped", bridge.queue_window, to_model,
            spawn=spawn))
    except Exception as exc:                # never take the demo down
        print(f"[omnilink_quadruped_bridge] parser-first skipped: {exc!r}",
              flush=True)
        return False


def handle_wwi(bridge: QuadrupedBridge, relay: Any, msg: str) -> None:
    if not msg:
        return
    if msg.startswith("configure"):
        push_configure(bridge, relay); return
    if msg.startswith("stop"):
        bridge.act_stop(wait=False)
        bridge.queue_window("agent:Stop received.")
        bridge.queue_window("tool:stop_robot:ok:frozen")
        bridge.queue_window("status:idle"); return
    if msg.startswith("prompt:"):
        if relay is None:
            reject_window_prompt(bridge)
            return
        text = msg[len("prompt:"):]
        if relay is not None:
            def _to_model() -> None:
                relay.dispatch_async(
                    text, lambda k, p: _on_relay_event(bridge, k, p))

            # PARSER FIRST, same order as HTTP: the `relay is None` check
            # above already refused a keyless prompt.
            if _parser_first_window(bridge, relay, text, _to_model):
                return
            _to_model()
            return
    if msg.startswith("audio_in:"):
        if relay is None:
            bridge.queue_window("error:audio_in requires OMNI_KEY (no relay attached)"); return
        import base64 as _b64
        payload = msg[len("audio_in:"):]
        try:
            info = json.loads(payload)
            audio = _b64.b64decode(info.get("audio_b64", ""))
            mime = info.get("mime_type", "audio/webm")
        except Exception as e:
            bridge.queue_window(f"error:audio_in decode failed: {e}"); return
        bridge.queue_window("status:transcribing")

        def _stt_worker():
            text = relay.transcribe(audio, mime_type=mime)
            if not text:
                bridge.queue_window("error:could not transcribe audio")
                bridge.queue_window("status:idle"); return
            bridge.queue_window("transcript:" + text)

            def _to_model() -> None:
                relay.dispatch_async(
                    text, lambda k, p: _on_relay_event(bridge, k, p))

            # Parser first here too, or a spoken order takes a different
            # path from the typed one.
            if _parser_first_window(bridge, relay, text, _to_model):
                return
            _to_model()

        threading.Thread(target=_stt_worker, name="omnilink-stt", daemon=True).start()
        return


def main() -> int:
    args = _parse_args()
    robot = Supervisor()
    bridge = QuadrupedBridge(robot, args.robot, locomotion=args.locomotion)
    relay = setup_omnilink_relay(bridge, http_port=args.port)
    start_http(bridge, args.port, relay)
    # ⚠️ "local" used to be printed here when no relay attached, back when a
    # keyword ladder answered a keyless prompt. There is no local mode: with
    # no OmniKey the chat surface refuses (401 omnikey_required) and only the
    # typed HTTP verbs and Stop remain. Say that, or the line advertises a
    # fallback the 2026-09-22 access policy removed.
    _link = ("OmniLink connected" if relay else
             "no OmniKey: chat disabled, typed HTTP verbs and Stop only")
    print(f"[omnilink_quadruped_bridge] {bridge.model} ready ({_link}).")

    timestep = bridge.timestep
    hold = getattr(bridge, "hold", None)
    if hold is not None and hold.enabled:
        print("[omnilink_quadruped_bridge] LOCKSTEP: the world is held "
              "between commands (OMNISIM_BRIDGE_LOCKSTEP=1). stop_robot "
              "always runs; the hold lease expires after "
              f"{hold.DEFAULT_MS // 1000}s so a dead client cannot freeze "
              "the demo.", flush=True)
    while True:
        # D6. With lockstep OFF (the default) this is exactly
        # `robot.step(timestep)`. Never collapse it back to one while a
        # lease can be live: a step against a paused engine blocks, and
        # while it is blocked nobody can un-pause it.
        if hold is not None:
            _busy = bridge.is_busy()
            hold.sync(robot, busy=_busy, sim_time=bridge.sim_time,
                      step=bridge.sim_step)
            if hold.step_or_hold(robot, timestep, sim_time=bridge.sim_time,
                                 step=bridge.sim_step,
                                 window_open=bridge.window_configured) == -1:
                break
            if not hold.last_advanced:
                continue        # world frozen: no tick, no pose, no motion
        elif robot.step(timestep) == -1:
            break
        sim_t = robot.getTime()
        while True:
            msg = robot.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi(bridge, relay, msg)
            except Exception as e:
                bridge.queue_window(f"error:bridge_exception: {e!r}")
        with bridge.lock:
            outbox = bridge.window_outbox
            bridge.window_outbox = []
        for line in outbox:
            try:
                robot.wwiSendText(line)
            except Exception:
                pass
        bridge.tick(sim_t)
    # Clean shutdown: flush the action journal the 30 s beat has not sent.
    if close_relay is not None:
        close_relay(relay)
    return 0


if __name__ == "__main__":
    sys.exit(main())
