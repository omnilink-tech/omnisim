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
    nonempty_string,
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
    args, _ = p.parse_known_args()
    return args


class QuadrupedBridge:
    def __init__(self, robot: Supervisor, robot_id: str = "omniquad") -> None:
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
        }

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
            # Smoothstep from the settle-from pose into the stand over settle_s.
            a = _smoothstep((sim_t - p["t0"]) / max(1e-6, float(self.cfg["settle_s"])))
            for leg in LEGS:
                target = self._pose(leg, "stand")
                start = tuple(self.settle_from[(leg, j)] for j in ("hip_x", "hip_y", "knee"))
                self._apply(leg, *(s + (t - s) * a for s, t in zip(start, target)))
            if a >= 1.0:
                with self.lock:
                    self.motion = ("stand", {"t0": sim_t})
        elif kind == "stand":
            sway = 0.04 * math.sin(2 * math.pi * (sim_t - p["t0"]) / 3.0)
            self._set_all("stand", hip_y_delta=sway)
        elif kind == "sit":
            self._set_all("sit")
        elif kind == "wave":
            t = sim_t - p["t0"]
            sway = 0.10 * math.sin(2 * math.pi * 0.8 * t)
            self._set_all("stand", hip_y_delta=sway, hip_x_extra=sway * 0.5)
            if t > p["duration_s"]:
                with self.lock:
                    self.motion = ("stand", {"t0": sim_t})
        elif kind == "walk":
            if self.cfg["walk"] == "wheels":
                self._tick_drive(sim_t, p)
            else:
                self._tick_walk(sim_t, p, translate=(self.cfg["walk"] == "gait"))
        elif kind == "stop":
            # Do nothing -- last setpoints stay applied.
            pass

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

    def act_stop(self) -> dict:
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

    def act_stand(self) -> dict:
        with self.lock:
            # ⚠️ THE CACHED CLOCK, NOT `robot.getTime()`. This runs on an
            # HTTP worker, and the controller API is not thread-safe -- a
            # supervisor read from here is the call that dragged the
            # warehouse demo to ~0.2x realtime (MainThreadCalls).
            self.motion = ("stand", {"t0": self.sim_time})
        self._set_wheels(0.0)
        return {"accepted": True, "pose": "stand"}

    def act_sit(self) -> dict:
        with self.lock:
            self.motion = ("sit", {})
        self._set_wheels(0.0)
        return {"accepted": True, "pose": "sit"}

    def act_wave(self, duration_s: float = 6.0) -> dict:
        with self.lock:
            self.motion = ("wave", {"t0": self.sim_time,
                                    "duration_s": duration_s})
        return {"accepted": True, "duration_s": duration_s}

    def act_walk(self) -> dict:
        with self.lock:
            self.motion = ("walk", {"t0": self.sim_time})
        return {"accepted": True, "pose": "walk", "walk": self.cfg["walk"],
                "velocity_ms": self.cfg["walk_velocity_ms"]}

    def act_reset_to_home(self) -> dict:
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
        }


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
                if path == "/stop_robot":
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
                if relay is not None:
                    # ── PARSER FIRST ──────────────────────────────────
                    # Reached only WITH a relay: the access check at the
                    # top of _route_post already refused a keyless prompt,
                    # so this can never become a keyless path. A non-None
                    # result is a confident, exactly-parsed order the gate
                    # (inside route.execute) has already vetted on the
                    # "quadruped" rail; no model is called for it.
                    _early = (_shared_short_circuit(bridge, text, "quadruped")
                              if _shared_short_circuit is not None else None)
                    if _early is not None:
                        return self._json(200, _shared_stamp_via(
                            _reply_payload(_early.get("agent", ""),
                                           _early.get("tools") or [],
                                           via="parser")))
                    # §5.7.2 / D3: `via` is REQUIRED on a 200 from /prompt.
                    # The parser stamps itself; anything reaching here was
                    # answered by the model relay.
                    return self._json(
                        200, _shared_stamp_via(relay.dispatch_sync(text)))
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


def build_quadruped_tools(bridge: QuadrupedBridge) -> List[Any]:
    if Tool is None:
        return []
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
        print(f"[omnilink_quadruped_bridge] OmniLink relay ON (agent='{agent_name}')")
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
        "suggestions": ["stand", "sit", "wave hello",
                        "drive forward" if bridge.cfg["walk"] == "wheels" else "walk", "stop"],
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
        bridge.act_stop()
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
    bridge = QuadrupedBridge(robot, args.robot)
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
            with bridge.lock:
                _busy = bridge.motion[0] not in ("stop", "stand", "sit")
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
