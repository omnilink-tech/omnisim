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

"""Mavic OmniLink bridge — supervisor controller for the drone_survey world.

Mirrors the design of `husky_omnilink_bridge`: a Webots controller that
owns the Mavic 2 Pro's motors + camera + pose, and exposes a small HTTP
surface so an external OmniLink agent can drive the drone. The agent
never imports the Webots `controller` module — it talks JSON over HTTP,
this bridge translates into propeller velocities and gimbal positions.

Wired into the world: chat/omnilink_mavic.omniworld's Mavic 2 Pro is a URDFRobot
with supervisor=TRUE and controller="mavic_omnilink_bridge". The
controller runs inside the Mavic's process, so `Supervisor.getSelf()`
returns the Mavic node and we can read its world pose every tick.
The bridge speaks two contracts on the same BridgeState: HTTP /action
and /prompt + /tool for the OmniLink agent, and wwi-chat text (handled
in mavic_chat_router.py) for the human operator typing into the right-
click Robot Window side panel. Both need an OmniKey: there is no
keyword fallback underneath either of them.

Single-process design (vs. husky's bridge + eye-sidecar split): the
Mavic URDF declares Camera/IMU/GPS via <gazebo><sensor> blocks under
the camera_roll_link / base_link (importer materialises them when
OMNISIM_URDF_USE_SENSORS=1), and the gimbal pitch motor is on the same
Robot, so the bridge can call `getDevice("camera")` directly. The
husky URDF can't host a Camera in the same way for historical importer
reasons, which is why husky_eye exists as a separate Robot. Mavic
doesn't have that constraint — perception lives in the same controller.

URDF caveat — rotor physics: URDF has no `Propeller` analogue, so the
continuous-rotation prop joints spin visually but generate no lift on
their own. The companion module `mavic_dynamics.py` (RotorDynamics)
applies F = k_thrust * ω² per propeller as a body-frame force via
`supervisor.getSelf().addForceWithOffset(...)`, plus a yaw torque
proportional to the diagonal-pair asymmetry, mirroring the
thrustConstants / torqueConstants the legacy PROTO carried.

HTTP surface (default port 6090, loopback only):

    GET  /state
        {
          x, y, z, roll, pitch, yaw,    # world frame, m / rad
          v_xy, v_z,                    # measured ground / vertical speed
          target_altitude_m,            # commanded hover altitude
          gimbal_pitch_rad,             # current camera pitch (0=forward, pi/2=down)
          mode,                         # idle | takeoff | hover | goto | land | landed
          fault,                        # null | "no_progress" (advisory: no closer
                                        #   by 0.25 m for 12 s of sim time; cleared on
                                        #   progress or arrival) | "crashed" (hard: tipped
                                        #   past 1.2 rad for 1 s in a flight mode; ends a
                                        #   wait) | "goto_waypoint_timeout" (wait responses)
          sim_time, last_tick_at,       # seconds, wall-clock epoch
          target,                       # nullable: {x, y, altitude} for goto
          mission_complete              # agent-set via complete_mission
        }

    GET  /capabilities
        {
          robot_id, model,
          mass_kg, max_horizontal_speed_m_s, max_vertical_speed_m_s,
          camera: {width, height, fov_h_rad, fov_v_rad, pitch_range_rad},
          world_title, mission_brief, mission_complete,
          ground_truth_def_names: [...]   # marker DEFs the agent can /solid lookup
        }

    GET  /scan
        Pure-Python analysis of the latest camera frame, projected to
        world coordinates using the drone pose + gimbal pitch + FoV:
        {
          markers: [
            {color, world_x, world_y, fraction, centroid_x_norm, centroid_y_norm,
             distance_m},
            ...
          ],
          frame_summary: {width, height, mean_brightness, frame_age_s},
          pose: {x, y, z, yaw, gimbal_pitch_rad},
        }

    GET  /image
        Latest camera frame as base64 PNG. Fallback / debugging only —
        agents should prefer /scan (perception-as-tool, ~150x cheaper).

    GET  /solid?def=NAME
        Ground-truth pose of any DEF'd Solid in the scene. Used by the
        agent (or by tests) to verify a /scan detection is actually where
        the bridge says it is.

    GET  /mission
        Mission brief (free-form text from WorldInfo.info). The agent
        interprets it; the bridge does not score it.

    POST /action  {"action": "...", ...params}
        takeoff             - {altitude} m; spool motors and climb to altitude
        land                - cut to a controlled descent, then idle motors
        hover               - hold current xy + altitude
        goto_waypoint       - {x, y, altitude?, wait?, timeout_s?, yaw_to?}
                              fly to (x, y) at altitude (default = current).
                              wait=true blocks until arrival (or timeout).
                              yaw_to=[lx, ly] aims the nose at (lx, ly) on arrival.
        set_gimbal_pitch    - {pitch_rad} 0=forward, pi/2=straight down
        set_yaw             - {yaw_rad} rotate body to absolute world yaw
        stop                - cut all attitude inputs, free-fall the props
                              (use only as last resort)
        reset               - teleport back to start pose, mission_complete=false
        complete_mission    - {rationale, payload?} log mission completion;
                              mission_complete -> true. Bridge does not score.

The bridge does primitive flight only. Mission planning, waypoint pattern
selection, and red-marker counting all live in the agent prompt.
"""

from __future__ import annotations

import argparse as _argparse
import base64
import json
import math
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

from omnisim import Supervisor

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from _omnilink_relay.http_security import (  # noqa: E402
    RequestError,
    RequestIdGuard,
    bearer_token,
    check_protocol_version,
    checked_origin,
    error_envelope,
    finite_number,
    nonempty_string,
    read_json,
    require_field,
    require_authorized,
    trusted_origins_from_env,
    validate_request_id,
    WIRE_SERVICE,
    WIRE_VERSION,
)

from mavic_dynamics import RotorDynamics
from mavic_chat_router import (handle_wwi_message, push_configure,
                               queue_window as _queue_window,
                               relay_event_to_window as _relay_event_to_window,
                               _StateBridge)

try:
    from omnisim_bridges.route import short_circuit as _shared_short_circuit
except ImportError:                                # pragma: no cover
    _shared_short_circuit = None

try:
    from omnisim_bridges.route import stamp_via as _shared_stamp_via
except ImportError:                                # pragma: no cover
    def _shared_stamp_via(payload, default="relay"):
        return payload

# D1/D4/D6: the two clocks, the event ring and its detectors, the hold.
# Optional exactly like the interpreter above -- a bare clone keeps every
# flight verb and simply cannot report events, which is the honest
# degradation: the surface is ABSENT rather than present and silent.
try:                                               # pragma: no cover
    from omnisim_bridges.bridge_base import (
        attach_relay,
        close_relay,
        attach_telemetry,
        events_summary,
        profile_extras,
        safety_gate_block,
        serve_events,
    )
    from omnisim_bridges.events import BRIDGE_EVENT_TYPES
except ImportError:                                # pragma: no cover
    attach_relay = None
    close_relay = None  # type: ignore[assignment]
    attach_telemetry = None
    profile_extras = None
    BRIDGE_EVENT_TYPES = ()
    safety_gate_block = None
    serve_events = None

    def events_summary(bridge):
        return {"total": 0, "last": None, "next_since": 0, "dropped": 0}

try:
    from omnisim_bridges.route import reply_payload as _reply_payload
except ImportError:                                # pragma: no cover
    def _reply_payload(reply, tools, **extra):
        """Fallback for a bare clone with no bridges package installed.

        Same contract as route.reply_payload: a FAILED action must reach
        the caller as a top-level `error`, or a refused order is
        indistinguishable from a completed one over the wire.
        """
        actions = [{"tool": t[0], "result": t[1], "summary": t[2]}
                   for t in (tools or ()) if len(t) >= 3]
        bad = next((a for a in actions if a["result"] != "ok"), None)
        out = {"ok": bad is None, "response": reply, "actions": actions}
        if bad is not None:
            out["error"] = bad["summary"]
        out.update(extra)
        return out


# ── The safety gate, on the drone's paths ────────────────────────────
#
# ⚠️ UNTIL 2026-09-21 THIS AIRCRAFT WAS THE ONE ROBOT NOTHING VETTED.
#
# The gate already knew how to fly: `takeoff`, `land`, `hover` and
# `move_body` are declared in gate.SPECS, and MAX_ALTITUDE_M (120 m, the
# usual small-UAS ceiling) has sat in that file unreached, because no
# drone frame could get to it. The Mavic served `POST /action` and
# nothing else -- no `/prompt`, no `/tool` -- so it was excluded from the
# chat-demo sweep by declaration (smoke_chat_demos.EXPECTED_UNSCRIPTED)
# and its only actuation surface was ungated (GATE_COVERAGE.md).
#
# That is the worst combination of the three: the vocabulary existed, the
# rails existed, and the wiring that would have connected them did not.
# "get up to 200 metres and have a look" had no rail to hit.
def _drone_tools(state):
    """The drone's tool surface, as {name: (dispatch, physical)}.

    Used by the authenticated OmniLink relay and typed tool dispatch.

    `goto_waypoint` and `set_yaw` are the two verbs `route._ADAPTERS` has
    no entry for, so they were reachable ONLY by the ungated path.
    """
    sb = _StateBridge(state)
    return {
        "takeoff":       (lambda a: sb.act_takeoff(a.get("altitude")), True),
        "land":          (lambda a: sb.act_land(), True),
        "hover":         (lambda a: sb.act_hover(), True),
        "stop_robot":    (lambda a: sb.act_stop(), True),
        "reset_to_home": (lambda a: sb.act_reset_to_home(), True),
        "turn":          (lambda a: sb.act_turn(a.get("angle_rad")), True),
        "move_body":     (lambda a: sb.act_move_body(
                              forward=a.get("forward"),
                              vertical=a.get("vertical")), True),
        "get_robot_state": (lambda a: sb.get_state_for_query(), False),
    }


# ── `POST /action`: its verbs, and which of them the gate ever sees ───────
#
# ⚠️ ONE SOURCE, BECAUSE THE PUBLICATION IS A SAFETY CLAIM. `/capabilities
# .safety_gate` (PROTOCOL.md §5.2.1) tells a client which of this bridge's
# routes are vetted, and `ungated_paths` is the half it reads to decide
# whether IT is responsible for bounding a command. Hand-maintained beside
# the router, that claim drifts, and it drifted in BOTH directions:
#
#   - `UNGATED_PATHS = ["/reset", "/complete_mission"]` named action VERBS,
#     not routes. The only POST routes here are /prompt, /tool and /action;
#     `POST /reset` has always been a 404, so neither string told a client
#     anything about anything.
#   - `/reset` was published as UNVETTED while the map below sends it
#     through `vet_toolcall` as `reset_to_home`. Wrong in the conservative
#     direction is still wrong: it teaches a client the block cannot be
#     trusted in EITHER direction.
#   - the genuinely unvetted verbs were absent. `goto_waypoint` flies the
#     aircraft to a coordinate and has no mapping onto the gate's
#     vocabulary, so it is the most consequential unchecked verb on this
#     bridge -- and the published block said it was checked.
#
# So the verb inventory and the mapping live here, the two halves are
# DERIVED from them, and the note is generated rather than written. Adding
# a verb to the router without adding it here is caught by
# `packages/omnisim-bridges/tests/test_mavic_gate_publication.py`.
ACTION_VERBS = ("takeoff", "land", "hover", "goto_waypoint",
                "set_gimbal_pitch", "set_yaw", "stop", "reset",
                "complete_mission")

# The verb names differ from the gate's frame vocabulary, so they are mapped
# rather than passed through. A verb with NO entry here is passed through
# unchanged -- refusing what this bridge has always accepted is a behaviour
# change, not a safety fix -- which is exactly the carve-out the note below
# has to publish. Every target must be a tool `_drone_tools` serves: the
# gate deliberately drops `unknown_tool`, so a typo'd target would not be a
# gate at all.
_ACTION_AS_TOOL = {
    "takeoff": "takeoff", "land": "land", "hover": "hover",
    "set_yaw": "turn", "stop": "stop_robot", "reset": "reset_to_home",
}

VETTED_ACTION_VERBS = tuple(v for v in ACTION_VERBS if v in _ACTION_AS_TOOL)
UNVETTED_ACTION_VERBS = tuple(v for v in ACTION_VERBS if v not in _ACTION_AS_TOOL)

# PROTOCOL.md §5.2.1. /action carries vetted AND unvetted verbs, so it is
# published in BOTH halves: a client reading `gated_paths` alone would be
# told to relax on `goto_waypoint`, and a client reading an EMPTY
# `ungated_paths` -- which is what "routes only" would leave here, since
# every other POST route on this bridge is fully gated -- would read
# "everything here is vetted", the one reading §5.2.1 exists to prevent.
# In both halves is the fail-safe shape: the load-bearing half names the
# route, and the note names the verbs on each side of the line.
GATED_PATHS = ["/prompt", "/tool", "/action"]
UNGATED_PATHS = ["/action"]

SAFETY_GATE_NOTE = (
    "/action is PARTIALLY vetted, so it is published in both gated_paths "
    "and ungated_paths. Vetted verbs (mapped onto the gate's vocabulary, "
    "refused with 400 refused_by_gate): "
    + ", ".join(VETTED_ACTION_VERBS) + ". "
    "UNVETTED verbs (no mapping, passed through unchanged -- the caller is "
    "responsible for bounding them): "
    + ", ".join(UNVETTED_ACTION_VERBS) + ". "
    "goto_waypoint flies the aircraft to a coordinate with no rail on it. "
    "The verbs are values of the request body's `action` field, not routes: "
    "this bridge's only POST routes are /prompt, /tool and /action."
)


# Mirror stdout/stderr to a tempfile so we can debug from outside Webots.
class _Tee:
    def __init__(self, *streams):
        self._streams = streams
    def write(self, s):
        for st in self._streams:
            try: st.write(s); st.flush()
            except Exception: pass
    def flush(self):
        for st in self._streams:
            try: st.flush()
            except Exception: pass

import os as _os_for_log
import tempfile as _tempfile_for_log
import sys as _sys_for_log
_log_path = _os_for_log.path.join(_tempfile_for_log.gettempdir(),
                                  f"mavic_omnilink_bridge_{_os_for_log.getpid()}.log")
try:
    _log_fp = open(_log_path, "w", encoding="utf-8", buffering=1)
    _sys_for_log.stdout = _Tee(_sys_for_log.__stdout__, _log_fp)
    _sys_for_log.stderr = _Tee(_sys_for_log.__stderr__, _log_fp)
    print(f"[mavic_omnilink_bridge] log file: {_log_path}")
except Exception:
    pass


# --- Mavic flight constants (calibrated empirically from mavic2pro.py) -----
# Same values the stock C / Python controllers use — tuned for the Mavic2Pro
# proto's mass + propeller geometry. Don't rebalance these without verifying
# stable hover first; a 2% change in K_VERTICAL_THRUST tips the system.

K_VERTICAL_THRUST = 68.5   # propeller thrust at hover
# Attitude rescale paired with the k_thrust recalibration (public issue #10):
# k went 0.00026 -> 0.00054 so the 1.0333 kg URDF can hover at omega = 68.5,
# and attitude torque per unit motor-delta scales linearly with k. Multiply the
# attitude inputs by k_old/k_new so the verbatim mavic2pro.py tuning keeps the
# torque scale it was calibrated for. Measured without this: lift-off to 0.87 m,
# then |roll| -> pi and a 22 m skid.
ATT_SCALE = 0.00026 / 0.00054
# Rate-damping gain on the IMU-differenced roll/pitch rates (rad/s). The
# classic law used the Gyro's rate with an implicit gain of 1.0; the Gyro is
# dead under Newton (constant zeros), and 1.0 was underdamped anyway once the
# rates come from clean IMU differencing. Tuned by measurement in issue #10.
K_ATT_D = 10.0
# The URDF prop joints declare velocity=576 rad/s; the real motor clamps there,
# but RotorDynamics computes thrust from the COMMANDED input, so an unclamped
# mixer output (D-term spikes reached thousands of rad/s) produced unbounded
# force -- measured: ~1900 N and a 0.1 -> 37 m "rocket" in 1.3 s. Clamp what we
# both command and model to the motor's physical range; floor at 0 because a
# fixed-pitch quad never reverses its props (and reverse thrust on the ground
# is how the airframe flipped).
MOTOR_MAX_RAD_S = 576.0
# IMU-differenced rates spike across Euler-angle flips (|roll| near pi, yaw
# wraps); clamp them so one bad tick cannot dominate the mixer.
MAX_ATT_RATE = 15.0
# Yaw rate damping (the classic law has NO yaw stabilisation beyond the
# steering disturbance, and the URDF craft spins freely about z). Sized
# against MAX_YAW_DISTURBANCE = 0.4: at 30 the terminal commanded yaw rate was
# 0.013 rad/s and the craft could never turn back to its hover point (measured
# 223 m of drift in 25 s); at 2 the terminal rate is 0.2 rad/s.
K_YAW_D = 2.0
# Vertical climb-rate damping: the classic vertical law is a cubed-P on
# altitude with NO rate term, i.e. a double integrator under bang thrust --
# it overshot a 1.0 m target to 3+ m. Rate from sim-tick altitude differencing.
K_VERT_D = 8.0
# Slow altitude integrator. The classic cubed-P law can only null steady-state
# error if hover thrust matches the weight to ~1e-7 in k_thrust (the cube root
# magnifies any bias: a 0.3% thrust deficit measured as a 0.34 m sag). The
# integrator trims that out; clamped hard so it can never fly the craft away.
K_VERT_I = 0.4
VERT_I_CLAMP = 3.0
# Horizontal position hold (public issue #10). The classic law had none: once
# "arrived" it stopped commanding xy entirely, so any trim tilt accelerated the
# craft for ever (measured: 220 m of drift in 25 s of hover); and its far-field
# dash braked with a fixed +0.1 nose-up that pushes WITH a backward drift.
# Near the target (< POS_HOLD_RADIUS_M) command tilt from position error +
# velocity damping instead; far from it, keep the classic yaw-and-dash but add
# the same velocity damping so speed stays bounded. Gains are in the mixer's
# input units (the same scale the disturbances always used).
POS_HOLD_RADIUS_M = 3.0
K_POS = 3.0        # input units per metre of position error
K_XY_V = 8.0       # input units per m/s of velocity (always on)
MAX_HOLD_TILT = 8.0  # clamp on the hold branch's roll/pitch command
# Integral position hold (public issue #14, second round). The P+D hold above
# parks the aircraft 0.2-0.6 m short of every target -- a constant body-frame
# trim the P term can only balance with a standing error. The tester's course
# has 0.5 m of standoff, so each leg started off-lane and cut the next corner
# along a shelf face; a 0.6 m arrival tolerance hid it. Measured on that course,
# three flights per arm, one engine per flight: without the integrator the
# closest approach per waypoint was 0.17-0.60 m and the hull grazed the shelves
# (min clearance 0.000-0.005 m); with it every waypoint is met on the lane and
# the hull clears by 0.15-0.18 m. Body-frame, clamped, integrated only inside
# POS_HOLD_RADIUS_M so a long transit cannot wind it up; reset with the pose.
K_XY_I = 0.4        # input units per (m*s) of integrated body-frame position error
XY_I_CLAMP = 3.0    # integrator clamp (input units); the trim it absorbs is ~1.5
K_VERTICAL_OFFSET = 0.0    # was 0.6: a standing bias for the era when hover
                           # thrust was short of the weight; k_thrust is now
                           # calibrated so omega=68.5 hovers exactly, and the
                           # 0.6 just parked the craft 0.25 m ABOVE every target
                           # (measured in issue #10).
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

MAX_YAW_DISTURBANCE = 0.4
MAX_PITCH_DISTURBANCE = -1.0   # negative = pitch nose down to fly forward

# Approach-target precision (meters).
WAYPOINT_REACH_TOL_M = 0.6

# Stall detection (public issue #14). A flight campaign measured two flights that
# wedged against an obstacle and sat there for 212 s and 40 s while the bridge
# still reported `mode=goto` and `fault=None`: "the aircraft does not detect the
# obstacle, does not report a fault, and does not give up." There is no planner
# here and none is claimed -- goto_waypoint is a position setpoint -- but a
# harness cannot distinguish "flying there" from "pinned against a shelf" without
# a signal, and that is cheap to provide. We REPORT and do not act: giving up is
# the operator's call, so the mode is left alone and only `fault` is set.
STALL_PROGRESS_TOL_M = 0.25   # closer than this to the best-so-far is "no progress"
STALL_TIMEOUT_S = 12.0        # sim seconds without progress before reporting


# Crash detection (public issue #14, second round). The stall probe that
# reproduced the tester's case sat on its nose against a shelf -- z 0.19 m,
# pitch -pi/2 -- for 85 s in mode=goto with fault=None, because the stall check
# below lived inside the `altitude > 0.25` gate and a fallen aircraft never
# reached it. An aircraft in a flight mode that is tipped past CRASH_TILT_RAD
# for CRASH_HOLD_S of sim time is not flying and never will be on its own; that
# is a HARD fault (it ends a `wait`), unlike the advisory no_progress.
CRASH_TILT_RAD = 1.2      # ~69 degrees of roll or pitch
CRASH_HOLD_S = 1.0        # sustained, so a hard bump is not a crash


def crash_verdict(roll, pitch, tipped_since_s):
    """(is_crashed, still_tipped) for one tick of a flight mode."""
    tipped = abs(roll) > CRASH_TILT_RAD or abs(pitch) > CRASH_TILT_RAD
    return (tipped and tipped_since_s >= CRASH_HOLD_S), tipped


def stall_verdict(dist_xy, best_dist, since_progress_s):
    """(is_stalled, new_best) for one tick of a goto.

    Pure so it can be tested without an engine. `best_dist` is the closest the
    aircraft has been to this target so far; progress resets the clock.
    """
    if best_dist is None or dist_xy < best_dist - STALL_PROGRESS_TOL_M:
        return False, dist_xy
    return since_progress_s >= STALL_TIMEOUT_S, best_dist


def stall_step(best, since, fault, dist_xy, sim_time):
    """One tick of the stall tracker, pure: (best, since, fault) -> (best, since, fault).

    The v8.3.0 wiring decided "we just set a new best" with a float equality
    (`state.stall_best == dist_xy`), which a distance that is bit-identical from
    tick to tick satisfies on EVERY tick -- so an aircraft wedged at exact rest
    reset its own clock forever and never reported. A tester's 197 s pin at
    cruise moved by millimetres a few times a minute; each 60 s leg window saw
    a constant distance and `no_progress` never fired. The clock now resets
    only on actual progress.
    """
    if best is None or dist_xy < best - STALL_PROGRESS_TOL_M:
        return dist_xy, sim_time, (None if fault == "no_progress" else fault)
    if sim_time - since >= STALL_TIMEOUT_S and fault is None:
        fault = "no_progress"
    return best, since, fault
ALTITUDE_REACH_TOL_M = 0.4

# Gimbal pitch motor range (camera pitch). 0 = forward, pi/2 = down.
GIMBAL_PITCH_MIN = -0.5
GIMBAL_PITCH_MAX = 1.7
GIMBAL_DOWN_RAD = math.pi / 2
# Gimbal slew rate (public issue #14, the "spawn ejection"). Commanding the
# pitch servo straight from 0 to pi/2 at init is a step of 1.57 rad against a
# 5 N*m effort limit, and the reaction into the airframe kicks the parked
# aircraft 1.15 m along its heading in under a second -- measured with the
# engine's body-pose trace: HEAD and the v8.1.17 bridge eject byte-identically,
# a bridge that never commands the gimbal does not move, and a ramped command
# parks at the authored pose to the millimetre. Lowering the URDF effort to
# 0.2 N*m only shrank the kick to 0.3 m, so the fix is to never present the
# servo with a large error: the commanded position walks toward the target at
# this rate, from wherever the joint is, at init and on every later change.
GIMBAL_RATE_RAD_S = 1.0

# Default takeoff / cruise altitude.
DEFAULT_TAKEOFF_ALTITUDE = 12.0


# --- Bridge config ---------------------------------------------------------

BRIDGE_HOST = "127.0.0.1"


def _parse_bridge_args() -> int:
    parser = _argparse.ArgumentParser(add_help=False)
    parser.add_argument("--port", type=int, default=6090)
    parser.add_argument("--name", default="mavic2pro")
    args, _unknown = parser.parse_known_args()
    return args.port


BRIDGE_PORT = _parse_bridge_args()


# --- Math / image helpers --------------------------------------------------

def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_axis_angle(rotation) -> float:
    """Heading (rotation about +Z) of a VRML axis-angle rotation [ax, ay, az, angle].

    Pure so it can be tested without an engine. Goes through the quaternion so
    an axis that is not exactly +Z still yields the right heading.
    """
    ax, ay, az, angle = (float(v) for v in rotation)
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-9:
        return 0.0
    ax, ay, az = ax / norm, ay / norm, az / norm
    half = 0.5 * angle
    sh = math.sin(half)
    qw, qx, qy, qz = math.cos(half), ax * sh, ay * sh, az * sh
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def encode_png(width: int, height: int, bgra_bytes: bytes) -> bytes:
    """BGRA -> RGB PNG. No PIL dependency. Same encoder as husky_eye."""
    rgb = bytearray(width * height * 3)
    for i in range(width * height):
        b = bgra_bytes[i * 4 + 0]
        g = bgra_bytes[i * 4 + 1]
        r = bgra_bytes[i * 4 + 2]
        rgb[i * 3 + 0] = r
        rgb[i * 3 + 1] = g
        rgb[i * 3 + 2] = b
    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    raw = bytearray()
    stride = width * 3
    for y in range(height):
        raw.append(0)
        raw.extend(rgb[y * stride:(y + 1) * stride])
    idat = _png_chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    iend = _png_chunk(b"IEND", b"")
    return sig + ihdr + idat + iend


def _png_chunk(tag: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)


# --- Perception: pure-Python BGRA -> structured marker digest --------------

def _classify_blobs(bgra_bytes: bytes, w: int, h: int) -> dict:
    """Walk the BGRA frame once and return per-colour {pixels, sum_x, sum_y,
    fraction, centroid_norm}. Threshold and gates match husky_eye._analyze_bgra
    so the demo's perception story is consistent across robots.

    Six recognised colours: red, green, blue, yellow, magenta, cyan. The
    drone_survey world also has orange + white markers as distractors —
    they get classified as the nearest colour (red / yellow / cyan blends
    or "other"). Only colour blobs above a noise floor are returned.
    """
    total = w * h
    if len(bgra_bytes) < total * 4:
        return {"error": "frame buffer too small"}

    counts = {"red": 0, "green": 0, "blue": 0,
              "yellow": 0, "magenta": 0, "cyan": 0}
    sum_x = {k: 0 for k in counts}
    sum_y = {k: 0 for k in counts}
    bright_sum = 0

    raw = bgra_bytes
    for y in range(h):
        row_off = y * w * 4
        for x in range(w):
            i = row_off + x * 4
            B = raw[i]
            G = raw[i + 1]
            R = raw[i + 2]
            bright_sum += R + G + B
            tag = None
            # The PBR camera's exposure maps emissive primary-colour pads to
            # lighter, partially desaturated pixels. Classify by channel
            # dominance instead of requiring the two secondary channels to
            # remain below an absolute value. Neutral asphalt and highlights
            # therefore stay out while the rendered red/pink pad remains red.
            if R > 140 and R > 1.35 * G and R > 1.35 * B:
                tag = "red"
            elif G > 140 and G > 1.35 * R and G > 1.35 * B:
                tag = "green"
            elif B > 140 and B > 1.35 * R and B > 1.35 * G:
                tag = "blue"
            elif R > 140 and G > 130 and min(R, G) > 1.35 * B:
                tag = "yellow"
            elif R > 140 and B > 130 and min(R, B) > 1.35 * G:
                tag = "magenta"
            elif G > 140 and B > 140 and min(G, B) > 1.35 * R:
                tag = "cyan"
            if tag is not None:
                counts[tag] += 1
                sum_x[tag] += x
                sum_y[tag] += y

    # Threshold: 0.05 % of frame area. Drone_survey markers are 0.8 m
    # squares viewed from 12 m — they subtend ~60 px in a 400×240 frame
    # (~0.6 % of total pixels), comfortably above noise. The threshold
    # is conservative enough that distant markers in oblique views still
    # register but stray red pixels from the warehouse roof or shadows
    # don't generate false detections.
    threshold = total * 0.0005
    blobs = []
    for c, n in counts.items():
        if n <= threshold:
            continue
        cx = sum_x[c] / n
        cy = sum_y[c] / n
        blobs.append({
            "color": c,
            "pixels": n,
            "fraction": round(n / total, 4),
            "centroid_x_norm": round(cx / max(w - 1, 1), 4),
            "centroid_y_norm": round(cy / max(h - 1, 1), 4),
        })
    return {
        "blobs": blobs,
        "mean_brightness": round(bright_sum / (total * 3), 1),
    }


def _project_to_world(centroid_x_norm: float, centroid_y_norm: float,
                       drone_x: float, drone_y: float, drone_alt: float,
                       drone_yaw: float, gimbal_pitch_rad: float,
                       fov_h_rad: float, fov_v_rad: float) -> dict:
    """Map an image-space centroid to world (x, y) for a downward-pointing
    camera. Assumes the gimbal is close to straight down (pitch ≈ pi/2)
    and the ground is z = 0.

    Geometry: centroid_x_norm/y_norm is (0,0)=top-left, (1,1)=bottom-right.
    Image x grows to the right, y grows down. With the camera pointed
    straight down and the drone facing +X (yaw 0), pixel +x maps to body
    +Y (left in body frame becomes +Y in world after yaw rotation), pixel
    -y (up in image) maps to body +X (forward).

    For pitch != pi/2, the projection skews along the camera's optical
    axis. We use a small-angle approximation: deviations of ±0.1 rad from
    pi/2 introduce <1 m error at 12 m altitude, which is well under the
    1.5 m dedup tolerance the agent uses. For larger deviations the
    projection becomes meaningfully wrong and the caller should re-take
    the scan with the gimbal closer to straight down.
    """
    # Image-frame angular offset from camera optical axis.
    # x_norm=0.5 is the optical axis (no offset). +x_norm => +angle right
    # (in camera body frame). +y_norm => +angle down (in camera body frame).
    half_fov_h = fov_h_rad / 2.0
    half_fov_v = fov_v_rad / 2.0
    # tan(angle) = (centroid - 0.5) / 0.5 * tan(half_fov)
    tan_h = (centroid_x_norm - 0.5) * 2.0 * math.tan(half_fov_h)
    tan_v = (centroid_y_norm - 0.5) * 2.0 * math.tan(half_fov_v)
    # Distance from drone to ground point along camera optical axis.
    # When gimbal is straight down, this is just drone altitude.
    # For oblique angles (pitch < pi/2), the slant range increases.
    sin_pitch = math.sin(gimbal_pitch_rad)
    if sin_pitch < 0.05:
        # Camera is nearly horizontal — projection is undefined / very
        # noisy. Punt with NaN and let the caller decide.
        return {"world_x": float("nan"), "world_y": float("nan"),
                "distance_m": float("nan"), "valid": False}
    # Body-frame ground offsets (+x = forward, +y = left).
    # With camera pointed down: image-up = body-forward, image-right = body-right.
    body_dx = -tan_v * drone_alt / sin_pitch     # forward (image-up = forward)
    body_dy = -tan_h * drone_alt / sin_pitch     # left  (image-right = body-right = -y)
    # Rotate body-frame into world-frame using yaw.
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)
    world_dx = body_dx * cos_y - body_dy * sin_y
    world_dy = body_dx * sin_y + body_dy * cos_y
    world_x = drone_x + world_dx
    world_y = drone_y + world_dy
    distance = math.hypot(world_dx, world_dy) + drone_alt  # rough slant range
    return {
        "world_x": round(world_x, 2),
        "world_y": round(world_y, 2),
        "distance_m": round(distance, 2),
        "valid": True,
    }


# --- Shared state ----------------------------------------------------------

class BridgeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.prev_rp = None  # (roll, pitch, yaw, altitude) one sim tick ago
        self.vert_i = 0.0    # altitude-error integrator (K_VERT_I)
        self.pitch = 0.0
        self.yaw = 0.0
        # Velocities — derived from successive poses each tick.
        self.last_pose_for_v = None  # (x, y, z, t)
        self.v_xy = 0.0
        self.v_z = 0.0
        # Commanded setpoints (consumed by the flight loop).
        self.target_altitude = 0.0
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None
        self.target_yaw: Optional[float] = None
        self.gimbal_pitch_target = GIMBAL_DOWN_RAD
        self.gimbal_pitch_actual = 0.0
        self.gimbal_cmd = 0.0   # the angle currently commanded to the pitch servo (ramps)
        self.xy_i_fwd = 0.0     # body-frame position-hold integrators (K_XY_I)
        self.xy_i_right = 0.0
        self.mode = "idle"        # idle | takeoff | hover | goto | land | landed
        self.fault: Optional[str] = None
        # Stall detection (issue #14): closest approach to the live target and
        # the sim time at which it was achieved.
        self.stall_best: Optional[float] = None
        self.stall_since: float = 0.0
        self.tipped_since: Optional[float] = None   # sim time the airframe tipped past CRASH_TILT_RAD
        self.sim_time = 0.0
        self.last_tick_at = time.time()
        self.tick_period_s = 0.008
        # -- D1 / D4: the clock mirror, the ring and the detectors --
        # This bridge integrates its own `sim_time` in the flight loop
        # (above) rather than reading robot.getTime(), so `attach_telemetry`
        # installs the ring and the detectors and the loop keeps ownership
        # of the clock; `sim_step` is incremented beside it.
        # The hold is NOT wired here: a held drone is a drone falling out of
        # the sky the moment the rotors stop being commanded, and lockstep
        # is declared unsupported on this surface rather than shipped as
        # something that looks like it works. See the handoff.
        self.sim_step = 0
        self.events = None
        self.hold = None
        self.fault_detector = None
        self.contact_detector = None
        self.joint_detector = None
        self.robot_id = "mavic2pro"
        self.surface = "drone"
        # PROTOCOL.md 5.2.1, read by BOTH `/capabilities` and the profile
        # push (bridge_base.profile_extras), so the two cannot disagree
        # about which of this bridge's routes are vetted. Both halves are
        # DERIVED from ACTION_VERBS + _ACTION_AS_TOOL at the top of this
        # file -- see the comment there for what a hand-maintained copy got
        # wrong, in both directions, and why /action is in both halves.
        self.GATED_PATHS = list(GATED_PATHS)
        self.UNGATED_PATHS = list(UNGATED_PATHS)
        self.capabilities = {"actions": ["takeoff", "land", "hover", "turn",
                                         "move_body", "stop_robot",
                                         "reset_to_home", "get_robot_state"]}
        self.world = ""
        if attach_telemetry is not None:
            attach_telemetry(self, dt_s=self.tick_period_s,
                             robot_id="mavic2pro", surface="drone",
                             joints=False, contacts=False)
            self.sim_time = 0.0
            self.sim_step = 0
            # Installed and then DROPPED, deliberately: the installer wires a
            # hold for every bridge and this surface does not honour one.
            # Leaving a live HoldLease here would publish `lockstep` support
            # nothing implements.
            self.hold = None
        # Camera frame buffer (latest BGRA + timestamp). Refreshed when the
        # /scan or /image handler asks for it (on-demand capture keeps the
        # main loop unblocked when nobody is watching).
        self.camera_w = 0
        self.camera_h = 0
        self.camera_fov_h = 0.0
        self.camera_fov_v = 0.0
        self.frame_request = 0
        self.frame_served = 0
        self.frame_bgra: Optional[bytes] = None
        self.frame_pose = (0.0, 0.0, 0.0, 0.0, 0.0)  # x, y, z, yaw, gimbal_pitch
        self.frame_sim_time = 0.0
        # Mission state.
        self.world_title = ""
        self.mission_brief = ""
        self.mission_complete = False
        self.mission_log = []        # list of {ts, sim_time, rationale, payload}
        # Reset request — main loop teleports the drone back to start.
        self.reset_request: Optional[dict] = None
        # One-use anchor consumed by an immediate follow-up takeoff.  This is
        # separate from reset_request because the simulation thread clears that
        # queue while the HTTP thread may still be preparing the next action.
        self.reset_anchor: Optional[dict] = None
        # The pose the world file authored for this robot, captured at init.
        # `reset` returns here unless told otherwise (public issue #14: the
        # old default was the omnilink_mavic spawn, hard-coded, which on any
        # other world teleported the aircraft to a point that may not even be
        # over the floor).
        self.authored_pose = {"x": 0.0, "y": 0.0, "z": 0.1, "yaw": 0.0}
        # Ground-truth-able DEFs in the world (advertised in /capabilities).
        self.gt_def_names: list = []
        # wwi-chat outbox: lines the main loop should push to the robot
        # window (configure / status / agent / tool). Populated by the
        # chat router; drained in main() each tick.
        self.window_outbox: list = []
        self.window_configured = False

    def snapshot(self) -> dict:
        with self.lock:
            target = None
            if self.target_x is not None and self.target_y is not None:
                target = {
                    "x": self.target_x, "y": self.target_y,
                    "altitude": self.target_altitude,
                }
            return {
                "x": self.x, "y": self.y, "z": self.z,
                "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw,
                "v_xy": self.v_xy, "v_z": self.v_z,
                "target_altitude_m": self.target_altitude,
                "gimbal_pitch_rad": self.gimbal_pitch_actual,
                "gimbal_pitch_target_rad": self.gimbal_pitch_target,
                "mode": self.mode,
                "fault": self.fault,
                "sim_time": self.sim_time,
                # SIM SECONDS (PROTOCOL.md 5.3), with the wall clock in its
                # own field. `last_tick_at` used to be time.time() on every
                # bridge, so a client differencing it against `sim_time` got
                # the age of the Unix epoch.
                "last_tick_at": self.sim_time,
                "wall_time": self.last_tick_at,
                "step": self.sim_step,
                "held": False,        # lockstep unsupported on this surface
                "events": events_summary(self),
                "target": target,
                "mission_complete": self.mission_complete,
            }


# --- Synchronous-mode helper (mirrors husky bridge's _wait_for_task_idle) --

def _wait_until_arrived(state: BridgeState, target_x: float, target_y: float,
                         target_alt: float, timeout_s: float,
                         poll_s: float = 0.05) -> dict:
    """Block until the drone is within tolerance of (target_x, target_y, target_alt),
    or until the wall-clock deadline expires. Returns
    {done, fault, x, y, z, yaw, sim_time, distance_remaining_m}.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        with state.lock:
            f = state.fault
            x, y, z, yaw, st = state.x, state.y, state.z, state.yaw, state.sim_time
        # `no_progress` is ADVISORY (issue #14): it is reported in /state and in
        # this response, but it must not end the wait. The v8.3.0 wait returned
        # on it after 12 s of sim time, which turned every slow, scraping leg
        # that used to arrive inside the caller's timeout into a failed leg --
        # the completion regression the same reporter then measured (14/17 on
        # v8.1.17 -> 8/18). Only a hard fault ends the wait early.
        if f and f != "no_progress":
            return {"done": False, "fault": f, "x": x, "y": y, "z": z, "yaw": yaw,
                    "sim_time": st, "distance_remaining_m": math.hypot(x - target_x, y - target_y)}
        d_xy = math.hypot(x - target_x, y - target_y)
        d_z = abs(z - target_alt)
        if d_xy < WAYPOINT_REACH_TOL_M and d_z < ALTITUDE_REACH_TOL_M:
            with state.lock:
                if state.fault == "no_progress":
                    state.fault = None   # it got there; the advisory is stale
            return {"done": True, "fault": None, "x": x, "y": y, "z": z, "yaw": yaw,
                    "sim_time": st, "distance_remaining_m": d_xy}
        time.sleep(poll_s)
    with state.lock:
        f = state.fault
        x, y, z, yaw, st = state.x, state.y, state.z, state.yaw, state.sim_time
    return {"done": False, "fault": f or "goto_waypoint_timeout",
            "x": x, "y": y, "z": z, "yaw": yaw, "sim_time": st,
            "distance_remaining_m": math.hypot(x - target_x, y - target_y)}


# --- HTTP handlers ---------------------------------------------------------

def _json_finite(obj):
    """Recursively replace non-finite floats (NaN/Inf) with None.

    /scan legitimately carries NaN world coordinates when the gimbal is
    near horizontal (_project_to_world punts with NaN + valid=False), and
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


from omnisim_bridges.access import connection_error

# THE ONE /tool implementation. Do not copy it back in here: five
# near-identical handlers, each with its own fail-closed wrapper, is
# how a gated bridge_base came to cover none of the bridges.
#
# `_emit_refusal` / `refusal_rule` are the SHARED producers of a
# `gate.refused` event and of the `rule` that rides in a 400. /action
# refuses before it reaches `serve_tool`, so it has to call them itself --
# and it calls THESE rather than spelling the fallback again, because the
# two producers must name `gate_unavailable` identically or a consumer
# branching on `rule` cannot tell which of them wrote the event.
from omnisim_bridges.bridge_base import (
    _emit_refusal as emit_refusal, refusal_rule, serve_tool, tool_args,
    vet_toolcall,
)


def setup_omnilink_relay(state):
    from _omnilink_relay import OmniLinkRelay, Tool, get_omni_key, profile_sync
    if not get_omni_key():
        return None
    registry = _drone_tools(state)
    arguments = {
        "takeoff": {"altitude": {"type": "number"}},
        "turn": {"angle_rad": {"type": "number"}},
        "move_body": {"forward": {"type": "number"}, "vertical": {"type": "number"}},
    }
    tools = [Tool(name=name, description=name.replace("_", " "),
                  parameters={"type": "object", "properties": arguments.get(name, {}),
                              **({"required": ["angle_rad"]} if name == "turn" else {})},
                  dispatch=dispatch) for name, (dispatch, _) in registry.items()]
    try:
        agent_name = profile_sync.agent_name_for("mavic")
        task = ("Operate the simulated Mavic in OmniSim using the supplied tools. "
                "Read robot state before moving. Commands may continue after acceptance; "
                "check state before claiming completion. Keep replies brief.")
        # surface="drone": the relay hands it to gate.register_tools(), so
        # move_body{vertical} is judged on the drone's climb rail and not on
        # a quadruped's 1.0 m body shift.
        relay = OmniLinkRelay(omni_key=get_omni_key(), agent_name=agent_name,
                              main_task=task, tools=tools, surface="drone")
        if profile_sync.is_enabled():
            profile_sync.ensure_profile(client=relay._client, agent_name=agent_name,
                                        main_task=task, tool_defs=relay.tool_defs,
                                        engine=relay.engine,
                                        tool_callback_url=f"http://127.0.0.1:{BRIDGE_PORT}/tool",
                                        **(profile_extras(
                                            state, http_port=BRIDGE_PORT,
                                            surface="drone")
                                           if profile_extras is not None else {}))
            relay.set_presence_endpoint(f"http://127.0.0.1:{BRIDGE_PORT}/tool", robot="mavic")
        # -- Plan D4 step 5 / D5: the relay seam ----------------------
        # Without attach_bridge the relay has no handle to the event ring,
        # so no wake can ever fire and presence reports a faulted aircraft
        # as a healthy one. The sink is the window outbox, which the SIM
        # THREAD drains -- the raiser is called from the presence thread and
        # must never touch the Robot API itself.
        if attach_relay is not None:
            attach_relay(
                relay, state,
                event_sink=lambda k, p: _relay_event_to_window(state, k, p),
                window_raiser=lambda: _queue_window(
                    state, "system:the platform asked for your attention"))
        return relay
    except Exception:
        print("[mavic_omnilink_bridge] OmniLink unavailable. Check your OmniKey and model connection.")
        return None


def make_handler(state: BridgeState):
    trusted_origins = trusted_origins_from_env()
    token = bearer_token()
    action_lock = threading.RLock()
    request_ids = RequestIdGuard()
    # The HTTP surface's tool registry. It shares `state` with the
    # robot-window panel, which is the thing that is actually authoritative.
    tools = _drone_tools(state)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _json(self, code, obj):
            # allow_nan=False guarantees strictly valid JSON on the wire;
            # the sanitizer maps any NaN/Inf field to null instead.
            try:
                body = json.dumps(obj, allow_nan=False).encode("utf-8")
            except ValueError:
                body = json.dumps(_json_finite(obj),
                                  allow_nan=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("X-OmniSim-Wire", WIRE_VERSION)
            self.send_header("X-OmniSim-Service", WIRE_SERVICE)
            self.send_header("Content-Length", str(len(body)))
            origin = getattr(self, "_response_origin", None)
            if origin:
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
            self.end_headers()
            self.wfile.write(body)

        def _ok(self, extra=None):
            payload = {"status": "ok"}
            if extra:
                payload.update(extra)
            self._json(200, payload)

        def _read_json(self):
            return read_json(self, allow_empty=True)

        def _guard(self):
            check_protocol_version(self.headers)
            self._response_origin = checked_origin(self.headers, trusted_origins)
            require_authorized(self.headers, token)

        def do_OPTIONS(self):
            try:
                self._guard()
                self.send_response(204)
                if self._response_origin:
                    self.send_header("Access-Control-Allow-Origin", self._response_origin)
                    self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-OmniSim-Token")
                self.end_headers()
            except RequestError as exc:
                self._json(exc.status, error_envelope(exc.code, exc.message, exc.details))

        def _request_frame(self) -> Optional[dict]:
            """Bump frame_request, wait briefly for the main loop to refresh
            the buffer, return the captured frame + pose. Returns None on
            timeout."""
            with state.lock:
                state.frame_request += 1
                req_id = state.frame_request
            deadline = time.time() + 3.0
            while time.time() < deadline:
                with state.lock:
                    if state.frame_served >= req_id and state.frame_bgra:
                        return {
                            "bgra": state.frame_bgra,
                            "pose": state.frame_pose,
                            "sim_time": state.frame_sim_time,
                            "w": state.camera_w,
                            "h": state.camera_h,
                            "fov_h": state.camera_fov_h,
                            "fov_v": state.camera_fov_v,
                        }
                time.sleep(0.02)
            return None

        # Top-level guards: an uncaught exception in a route used to
        # propagate into socketserver, which logs it server-side and closes
        # the connection with ZERO bytes sent -- the client sees an empty
        # reply (curl error 52) with no clue why. Every route now answers
        # with a JSON body: data, or a clear {"error": ...}.
        def do_GET(self):
            try:
                self._guard()
                self._route_get()
            except RequestError as exc:
                self._json(exc.status, error_envelope(exc.code, exc.message, exc.details))
            except Exception as e:
                self._error_response(e)

        def do_POST(self):
            try:
                self._guard()
                with action_lock:
                    self._route_post()
            except RequestError as exc:
                self._json(exc.status, error_envelope(exc.code, exc.message, exc.details))
            except Exception as e:
                self._error_response(e)

        def _error_response(self, e: Exception) -> None:
            import traceback
            print(f"[mavic_omnilink_bridge] HTTP {self.command} {self.path} "
                  f"failed: {e!r}\n{traceback.format_exc()}")
            try:
                self._json(500, error_envelope("internal_error", "The bridge could not complete the request."))
            except Exception:
                pass  # headers already sent / socket gone -- nothing to add

        # ----- GET endpoints ----------------------------------------------

        def _route_get(self):
            if self.path == "/protocol":
                self._json(200, {
                    "ok": True, "omnisim_wire": WIRE_VERSION,
                    "service": WIRE_SERVICE,
                    "service_versions": {WIRE_SERVICE: WIRE_VERSION},
                    "instance": {"name": "mavic_omnilink_bridge", "robot_id": "mavic2pro"},
                    "extensions": [],
                })
                return
            if self.path == "/state":
                self._json(200, state.snapshot())
                return
            if self.path.split("?", 1)[0].rstrip("/") == "/events":
                # D4. Same envelope as the harness's /sim/events, so one
                # client loop drains both: {events, next_since, dropped}.
                if serve_events is None:
                    self._json(501, {"error": "this bridge serves no event ring"})
                    return
                self._json(200, serve_events(state, self.path))
                return
            if self.path == "/capabilities":
                self._json(200, {
                    "robot_id": "mavic2pro",
                    "model": "DJI Mavic 2 Pro",
                    "mass_kg": 0.9,                  # ~real Mavic 2 Pro mass
                    "max_horizontal_speed_m_s": 18,  # spec sport-mode top speed
                    "max_vertical_speed_m_s": 5,
                    "default_takeoff_altitude_m": DEFAULT_TAKEOFF_ALTITUDE,
                    "camera": {
                        "width": state.camera_w,
                        "height": state.camera_h,
                        "fov_h_rad": state.camera_fov_h,
                        "fov_v_rad": state.camera_fov_v,
                        "pitch_range_rad": [GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX],
                        "down_pitch_rad": GIMBAL_DOWN_RAD,
                    },
                    "world_title": state.world_title,
                    "mission_brief": state.mission_brief,
                    "mission_complete": state.mission_complete,
                    "ground_truth_def_names": list(state.gt_def_names),
                    # PROTOCOL.md 5.2.1. `ungated_paths` is the load-bearing
                    # half: it is what a client reads to decide whether IT
                    # is responsible for bounding a command.
                    #
                    # /prompt and /tool are fully gated. /action is
                    # PARTIALLY gated -- six of its nine verbs map onto the
                    # gate's vocabulary and three do not -- so it is
                    # published in BOTH halves. That is not tidy and it is
                    # the only honest shape available here: the protocol has
                    # no per-verb field, publishing /action as gated only
                    # would tell a client it need not bound `goto_waypoint`,
                    # and leaving ungated_paths EMPTY (which "routes only"
                    # would do, since every other POST route here is fully
                    # gated) reads as "everything here is vetted" -- the one
                    # reading §5.2.1 exists to prevent. `safety_gate_note`
                    # carries the per-verb split, naming both sides.
                    #
                    # ⚠️ The old lists named ACTION VERBS, not routes:
                    # `ungated_paths` was ["/reset", "/complete_mission"],
                    # and `POST /reset` has always been a 404. /reset was
                    # published as unvetted while it IS vetted (it maps to
                    # reset_to_home), and the three verbs that genuinely are
                    # not vetted were absent altogether.
                    #
                    # ONE source for all of it: ACTION_VERBS +
                    # _ACTION_AS_TOOL at the top of this file feed the two
                    # lists AND the note, and the same two lists are what
                    # the profile push reads (bridge_base.profile_extras),
                    # so `/capabilities` and the platform's profile cannot
                    # disagree about which routes are vetted.
                    "safety_gate": (safety_gate_block(
                        "drone", gated_paths=list(state.GATED_PATHS),
                        ungated_paths=list(state.UNGATED_PATHS))
                        if safety_gate_block is not None else None),
                    "safety_gate_note": SAFETY_GATE_NOTE,
                    "events": {
                        "endpoint": "GET /events?since=<cursor>&limit=&types=",
                        "state_field": "events",
                        "types": (list(BRIDGE_EVENT_TYPES)
                                  if BRIDGE_EVENT_TYPES else []),
                    },
                    "lockstep": {
                        "supported": False,
                        "why": ("a held drone is a drone whose rotors stop "
                                "being commanded; OMNISIM_BRIDGE_LOCKSTEP "
                                "is not honoured on this surface"),
                    },
                    "perception_hint": (
                        "Prefer /scan over /image — /scan returns structured "
                        "marker positions in world coordinates (~150x cheaper "
                        "than sending pixels to the LLM). /image is fallback "
                        "for genuinely ambiguous frames."
                    ),
                })
                return
            if self.path == "/mission":
                self._json(200, {
                    "world_title": state.world_title,
                    "brief": state.mission_brief,
                    "complete": state.mission_complete,
                    "log": list(state.mission_log),
                    "hint": (
                        "Read 'brief' in natural language. Plan + execute. "
                        "When the mission is satisfied POST /action "
                        "{action:'complete_mission', rationale:'<one sentence>', "
                        "payload:{...}}. The bridge logs your claim, it does "
                        "not score it. Operators audit the log."
                    ),
                })
                return
            if self.path == "/scan":
                frame = self._request_frame()
                if frame is None:
                    self._json(504, {"error": "timed out waiting for camera frame"})
                    return
                analysis = _classify_blobs(frame["bgra"], frame["w"], frame["h"])
                if "error" in analysis:
                    self._json(500, analysis)
                    return
                fx, fy, fz, fyaw, fpitch = frame["pose"]
                markers = []
                for blob in analysis["blobs"]:
                    proj = _project_to_world(
                        blob["centroid_x_norm"], blob["centroid_y_norm"],
                        fx, fy, fz, fyaw, fpitch,
                        frame["fov_h"], frame["fov_v"],
                    )
                    markers.append({
                        "color": blob["color"],
                        "pixels": blob["pixels"],
                        "fraction": blob["fraction"],
                        "centroid_x_norm": blob["centroid_x_norm"],
                        "centroid_y_norm": blob["centroid_y_norm"],
                        "world_x": proj["world_x"],
                        "world_y": proj["world_y"],
                        "distance_m": proj["distance_m"],
                        "projection_valid": proj["valid"],
                    })
                # Sort by world distance so the closest detection (most
                # reliable projection) shows up first.
                markers.sort(key=lambda m: (
                    not m["projection_valid"],
                    m.get("distance_m") or 1e9,
                ))
                self._json(200, {
                    "markers": markers,
                    "frame_summary": {
                        "width": frame["w"],
                        "height": frame["h"],
                        "mean_brightness": analysis["mean_brightness"],
                        "frame_age_s": round(state.sim_time - frame["sim_time"], 3),
                    },
                    "pose": {
                        "x": fx, "y": fy, "z": fz, "yaw": fyaw,
                        "gimbal_pitch_rad": fpitch,
                    },
                    "hint": (
                        "world_x/world_y are the projected ground positions of "
                        "each detected colour blob. Aggregate detections from "
                        "multiple waypoints by deduplicating within ~1.5 m "
                        "(detections of the same marker from different vantages "
                        "should cluster within that radius)."
                    ),
                })
                return
            if self.path == "/image":
                frame = self._request_frame()
                if frame is None:
                    self._json(504, {"error": "timed out waiting for camera frame"})
                    return
                png = encode_png(frame["w"], frame["h"], frame["bgra"])
                fx, fy, fz, fyaw, fpitch = frame["pose"]
                self._json(200, {
                    "width": frame["w"],
                    "height": frame["h"],
                    "encoding": "image/png; base64",
                    "image_base64": base64.b64encode(png).decode("ascii"),
                    "pose": {
                        "x": fx, "y": fy, "z": fz, "yaw": fyaw,
                        "gimbal_pitch_rad": fpitch,
                    },
                    "sim_time": frame["sim_time"],
                })
                return
            if self.path.startswith("/solid"):
                from urllib.parse import urlparse, parse_qs as _parse_qs
                qs = _parse_qs(urlparse(self.path).query)
                def_name = (qs.get("def") or [""])[0].strip()
                if not def_name:
                    self._json(400, {
                        "error": "missing 'def' query param",
                        "example": "/solid?def=MARKER_RED_1",
                    })
                    return
                sup = getattr(state, "supervisor", None)
                if sup is None:
                    self._json(503, {"error": "supervisor not initialised"})
                    return
                node = sup.getFromDef(def_name)
                if node is None:
                    self._json(404, {
                        "error": f"no DEF named {def_name!r}",
                        "hint": (
                            "names are case-sensitive; this world advertises: "
                            + ", ".join(state.gt_def_names)
                        ),
                    })
                    return
                try:
                    pos = list(node.getPosition())
                    orient = list(node.getOrientation())
                except Exception as exc:
                    self._json(500, {
                        "error": f"node {def_name!r} has no pose: {exc.__class__.__name__}: {exc}",
                    })
                    return
                self._json(200, {
                    "def": def_name,
                    "world_position": [float(p) for p in pos],
                    "world_orientation_3x3_row_major": [float(v) for v in orient],
                })
                return
            self._json(404, {"error": "not found"})

        # ----- POST /action ----------------------------------------------

        def _route_post(self):
            p = self.path.rstrip("/")

            # ── /prompt: the operator's own sentence ──────────────────
            # The parser already speaks drone -- interpret.py carries nine
            # DRONE rules and mavic_chat_router routes through it with
            # surface="drone". This endpoint is the only reason the
            # aircraft was absent from the chat-demo sweep.
            if p == "/prompt":
                body = self._read_json()
                text = nonempty_string(require_field(body, "text"), "text")
                relay = getattr(state, "omnilink_relay", None)
                if relay is None:
                    return self._json(401 if connection_error()["error"] == "omnikey_required" else 503, connection_error())
                # ── PARSER FIRST ──────────────────────────────────────
                # Reached only WITH a relay: the access check immediately
                # above refuses a keyless prompt, so this can never become
                # a keyless path. A non-None result is a confident,
                # exactly-parsed order the gate (inside route.execute) has
                # already vetted on the "drone" rail -- the rail that
                # carries MAX_ALTITUDE_M, which is why `move_body
                # {vertical}` must be judged as a CLIMB here and as a body
                # shift on a quadruped.
                _early = (_shared_short_circuit(_StateBridge(state), text,
                                                "drone")
                          if _shared_short_circuit is not None else None)
                if _early is not None:
                    return self._json(200, _shared_stamp_via(_reply_payload(
                        _early.get("agent", ""),
                        _early.get("tools") or [], via="parser")))
                # PROTOCOL.md 5.7.2 / D3: `via` is REQUIRED on a 200 from
                # /prompt. The parser stamps itself; anything reaching here
                # was answered by the model relay.
                return self._json(
                    200, _shared_stamp_via(relay.dispatch_sync(text)))

            # ── /tool: the platform's callback, gated ─────────────────
            if p == "/tool":
                if getattr(state, "omnilink_relay", None) is None:
                    return self._json(401 if connection_error()["error"] == "omnikey_required" else 503, connection_error())
                body = self._read_json()
                tool_name = nonempty_string(require_field(body, "tool"), "tool")
                # ⚠️ THE TRANSPORT FIELDS ARE STRIPPED INSIDE serve_tool, AND
                # THAT IS WHY IT IS SHARED. `id` is §3.3 request-idempotency
                # metadata, not an argument. Left in the body it reached the
                # gate as one and a perfectly good `takeoff{altitude: 3}` came
                # back `unknown_arg: id is not a parameter of takeoff` -- a
                # transport detail wearing a safety verdict's clothes. This
                # bridge was the one that got it wrong, because its POST
                # preamble is /action's; three of the other four never
                # stripped it either. One implementation, one answer.
                request_ids.claim(f"/tool/{tool_name}",
                                  validate_request_id(body.get("id")))
                entry = tools.get(tool_name)
                # `bridge=` is D4: a gate refusal here is filed as a
                # `gate.refused` event, which is otherwise the one thing on
                # this path that nobody ever sees -- the caller gets a 400
                # and the robot's own agent learns nothing at all.
                code, payload = serve_tool(
                    tool_name, body,
                    (lambda args: entry[0](args)) if entry else (lambda args: None),
                    surface="drone", bridge=state, origin="tool",
                    registered=entry is not None)
                return self._json(code, payload)

            if p != "/action":
                self._json(404, {"error": "not found"})
                return
            body = self._read_json()
            action = nonempty_string(require_field(body, "action"), "action")
            request_ids.claim(f"/action/{action}", validate_request_id(body.pop("id", None)))

            # ⚠️ GATE /action TOO. It is this bridge's oldest surface and
            # the one the mission runners use, and it was listed in
            # GATE_COVERAGE.md as ungated. Gating only the new endpoints
            # would have repeated b977772b0 exactly -- adding a check to
            # the path nobody was using and missing the path everybody was.
            #
            # The verb names differ from the gate's frame vocabulary, so
            # they are mapped rather than passed through; an action with no
            # mapping is left alone rather than refused, because refusing
            # what this bridge has always accepted is a behaviour change,
            # not a safety fix.
            #
            # ⚠️ THE MAP LIVES AT MODULE SCOPE NOW, and it is the single
            # source `/capabilities.safety_gate` is derived from. A copy
            # here would let the two disagree about which verbs are vetted,
            # which is the defect this whole arrangement exists to close.
            _as_tool = _ACTION_AS_TOOL.get(action)
            if _as_tool is not None:
                _args = {k: v for k, v in tool_args(body).items()
                         if k != "action"}
                if _as_tool == "turn" and "yaw" in _args:
                    _args = {"angle_rad": _args.get("yaw")}
                if _as_tool == "takeoff" and "altitude" not in _args:
                    _args = {k: v for k, v in _args.items() if k == "altitude"}
                _utt = body.get("utterance", "")
                _grej = vet_toolcall(_as_tool, _args, _utt, surface="drone")
                if _grej is not None:
                    # ⚠️ THIS WAS THE ONE GATED PATH CARRYING NEITHER. It
                    # refuses here, before `serve_tool`, so until now its
                    # 400 had no machine-readable `rule` (PROTOCOL.md
                    # §11.1) and no `gate.refused` ever reached the ring
                    # (§5.9) -- the caller had to re-parse prose that may
                    # change between releases, and the robot's own agent
                    # never learned the refusal happened at all. Both
                    # producers are the SHARED ones, so the rule name here
                    # and the rule name on /tool cannot drift apart: a
                    # consumer branching on it cannot tell which wrote it.
                    _rule = refusal_rule(_grej)
                    emit_refusal(state, _as_tool, _grej, origin="action",
                                 utterance=_utt, rule=_rule)
                    raise RequestError(400, "refused_by_gate", _grej,
                                       {"action": action, "tool": _as_tool,
                                        "rule": _rule})

            if action == "stop":
                with state.lock:
                    state.target_x = None
                    state.target_y = None
                    state.target_yaw = None
                    state.target_altitude = 0.0
                    # ⛔ DO NOT PUBLISH A POSE HERE. Until 2026-09-22 this
                    # branch assigned `state.x/y/z/yaw = reset_x/...`, copied
                    # from the `reset` branch BELOW it -- where those four
                    # names are assigned. They do not exist yet at this point,
                    # so `POST /action {"action": "stop"}` raised
                    # UnboundLocalError and the aircraft was never halted.
                    # Stop is the one verb PROTOCOL.md 5.5 says a caller may
                    # rely on without preconditions, so it crashed in exactly
                    # the situation it exists for.
                    #
                    # The teleport was wrong on its own terms too. `state.x/y/z`
                    # is the PUBLISHED pose, written each tick from the real
                    # body (see the simulation loop); `reset` may publish a
                    # requested pose ahead of its own teleport because it has
                    # one. A stop requests no pose: it drops the targets and
                    # lets the aircraft come to rest where it is. Equating a
                    # halt with homing is the error the sim-to-real guide
                    # names explicitly -- a reset may teleport a model, a stop
                    # must never be silently turned into one.
                    state.mode = "idle"
                    state.fault = None
                self._ok({"halted_at": time.time()})
                return

            if action == "reset":
                with state.lock:
                    home = dict(state.authored_pose)
                reset_x = finite_number(body.get("x", home["x"]), "x")
                reset_y = finite_number(body.get("y", home["y"]), "y")
                reset_z = finite_number(body.get("z", home["z"]), "z")
                reset_yaw = finite_number(body.get("yaw", home["yaw"]), "yaw")
                with state.lock:
                    state.target_x = None
                    state.target_y = None
                    state.target_yaw = None
                    state.target_altitude = 0.0
                    state.mode = "idle"
                    state.fault = None
                    state.mission_complete = False
                    state.mission_log = []
                    state.reset_request = {
                        "x": reset_x, "y": reset_y, "z": reset_z, "yaw": reset_yaw,
                    }
                    state.reset_anchor = dict(state.reset_request)
                    state.xy_i_fwd = 0.0
                    state.xy_i_right = 0.0
                self._ok({"reset": True, "pose": dict(state.reset_request)})
                return

            if action == "complete_mission":
                rationale = (body.get("rationale") or "").strip()
                payload = body.get("payload") or {}
                if not rationale:
                    self._json(400, {"error": "rationale is required (one-sentence why)"})
                    return
                ts = time.time()
                with state.lock:
                    state.mission_complete = True
                    state.mission_log.append({
                        "timestamp": ts,
                        "sim_time": state.sim_time,
                        "rationale": rationale,
                        "payload": payload,
                    })
                self._ok({
                    "mission_complete": True,
                    "logged_at": ts,
                    "log_entry": state.mission_log[-1],
                })
                return

            if action == "takeoff":
                altitude = finite_number(body.get("altitude", DEFAULT_TAKEOFF_ALTITUDE), "altitude")
                with state.lock:
                    state.stall_best = None
                    state.stall_since = state.sim_time
                    state.tipped_since = None
                    state.target_altitude = max(0.5, altitude)
                    anchor = state.reset_anchor
                    state.reset_anchor = None
                    state.target_x = anchor["x"] if anchor else state.x
                    state.target_y = anchor["y"] if anchor else state.y
                    state.mode = "takeoff"
                    state.fault = None
                if body.get("wait"):
                    timeout_s = finite_number(body.get("timeout_s", 30.0), "timeout_s")
                    res = _wait_until_arrived(state, state.x, state.y, altitude, timeout_s)
                    self._ok({"target_altitude": altitude, "waited": True, **res})
                    return
                self._ok({"target_altitude": altitude})
                return

            if action == "land":
                with state.lock:
                    state.target_x = state.x
                    state.target_y = state.y
                    state.target_altitude = 0.0
                    state.mode = "land"
                    state.fault = None
                if body.get("wait"):
                    timeout_s = finite_number(body.get("timeout_s", 30.0), "timeout_s")
                    deadline = time.time() + timeout_s
                    while time.time() < deadline:
                        with state.lock:
                            z = state.z
                            f = state.fault
                            mode = state.mode
                        if f:
                            self._ok({"waited": True, "landed": False, "fault": f, "z": z})
                            return
                        if z < 0.30 and mode == "landed":
                            self._ok({"waited": True, "landed": True, "z": z})
                            return
                        time.sleep(0.1)
                    self._ok({"waited": True, "landed": False, "fault": "land_timeout"})
                    return
                self._ok({"target_altitude": 0.0})
                return

            if action == "hover":
                with state.lock:
                    state.target_x = state.x
                    state.target_y = state.y
                    if state.target_altitude < 0.5:
                        state.target_altitude = max(state.z, DEFAULT_TAKEOFF_ALTITUDE)
                    state.mode = "hover"
                    state.fault = None
                self._ok({"hovering_at": {"x": state.x, "y": state.y, "z": state.z}})
                return

            if action == "goto_waypoint":
                tx = finite_number(require_field(body, "x"), "x")
                ty = finite_number(require_field(body, "y"), "y")
                with state.lock:
                    target_alt = finite_number(
                        body.get("altitude", state.target_altitude or DEFAULT_TAKEOFF_ALTITUDE), "altitude"
                    )
                    state.target_x = tx
                    state.target_y = ty
                    state.target_altitude = target_alt
                    state.mode = "goto"
                    state.fault = None
                    # A new target restarts progress tracking (issue #14): the
                    # previous target's best-approach must not leak into it.
                    state.stall_best = None
                    state.stall_since = state.sim_time
                    state.tipped_since = None
                if body.get("wait"):
                    timeout_s = finite_number(body.get("timeout_s", 60.0), "timeout_s")
                    res = _wait_until_arrived(state, tx, ty, target_alt, timeout_s)
                    # Optional yaw_to: aim nose at (lx, ly) on arrival.
                    yaw_to = body.get("yaw_to")
                    if (res["done"] and not res["fault"]
                            and isinstance(yaw_to, (list, tuple)) and len(yaw_to) == 2):
                        try:
                            lx = float(yaw_to[0]); ly = float(yaw_to[1])
                            desired_yaw = wrap_pi(math.atan2(ly - res["y"], lx - res["x"]))
                            with state.lock:
                                state.target_yaw = desired_yaw
                            time.sleep(0.5)  # let yaw settle
                            with state.lock:
                                res["yaw"] = state.yaw
                        except Exception:
                            pass
                    self._ok({"x": tx, "y": ty, "altitude": target_alt, "waited": True, **res})
                    return
                self._ok({"x": tx, "y": ty, "altitude": target_alt})
                return

            if action == "set_gimbal_pitch":
                pitch = finite_number(require_field(body, "pitch_rad"), "pitch_rad")
                pitch = clamp(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX)
                with state.lock:
                    state.gimbal_pitch_target = pitch
                self._ok({"gimbal_pitch_target_rad": pitch})
                return

            if action == "set_yaw":
                yaw = finite_number(require_field(body, "yaw_rad"), "yaw_rad")
                with state.lock:
                    state.target_yaw = wrap_pi(yaw)
                self._ok({"target_yaw_rad": wrap_pi(yaw)})
                return

            # Same inventory the safety_gate publication is derived from, so
            # a verb cannot be advertised here and missing from there.
            self._json(400, error_envelope(
                "invalid_action", f"Unknown action {action!r}.",
                {"available_actions": list(ACTION_VERBS)},
            ))

    return Handler


# --- Webots-side scaffolding -----------------------------------------------

def _collect_marker_defs(supervisor) -> list:
    """Walk the world for Solid nodes whose name starts with 'marker_'.
    Returns the DEF names so /capabilities can advertise what's
    /solid-lookup-able (mirrors the husky bridge's gt_def_names contract).
    """
    root = supervisor.getRoot()
    children_field = root.getField("children") if root is not None else None
    defs = []
    if children_field is None:
        return defs
    try:
        count = children_field.getCount()
    except Exception:
        return defs
    for i in range(count):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            name_f = node.getField("name")
            if name_f is None:
                continue
            name = name_f.getSFString()
        except Exception:
            continue
        if name and (name.startswith("marker_") or name == "warehouse"):
            try:
                def_name = node.getDef()
                if def_name:
                    defs.append(def_name)
            except Exception:
                pass
    return defs


def _read_world_brief(supervisor) -> tuple:
    """Pull WorldInfo.title + WorldInfo.info as the operator-visible brief."""
    root = supervisor.getRoot()
    children_field = root.getField("children") if root is not None else None
    if children_field is None:
        return ("", "")
    try:
        count = children_field.getCount()
    except Exception:
        return ("", "")
    title = ""
    info_lines = []
    for i in range(count):
        node = children_field.getMFNode(i)
        if node is None:
            continue
        try:
            type_name = node.getTypeName()
        except Exception:
            continue
        if type_name == "WorldInfo":
            try:
                title = node.getField("title").getSFString()
            except Exception:
                pass
            try:
                info_field = node.getField("info")
                n_lines = info_field.getCount()
                for j in range(n_lines):
                    info_lines.append(info_field.getMFString(j))
            except Exception:
                pass
            break
    return (title, "\n".join(info_lines))


def main():
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(encoding="utf-8", errors="backslashreplace")
        except Exception:
            pass

    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    self_node = supervisor.getSelf()
    if self_node is None:
        print("[mavic_omnilink_bridge] ERROR: getSelf() returned None — supervisor TRUE missing?")
        return

    # Enable sensors.
    camera = supervisor.getDevice("camera")
    if camera is None:
        print("[mavic_omnilink_bridge] ERROR: no 'camera' device. The Mavic is a URDFRobot, and "
              "the importer emits its <gazebo> sensors only when the world was loaded with "
              "OMNISIM_URDF_USE_SENSORS=1 in the ENGINE's environment. Set it and relaunch.")
        return
    camera.enable(time_step)
    cam_w = camera.getWidth()
    cam_h = camera.getHeight()
    cam_fov_h = camera.getFov()
    # Approximate vertical FoV from horizontal FoV + aspect ratio. Webots
    # cameras report the horizontal FoV; vertical = 2 * atan(tan(fov_h/2) * h/w).
    cam_fov_v = 2.0 * math.atan(math.tan(cam_fov_h / 2.0) * (cam_h / cam_w))
    print(f"[mavic_omnilink_bridge] camera: {cam_w}x{cam_h} fov_h={cam_fov_h:.2f} fov_v={cam_fov_v:.2f}")

    imu = supervisor.getDevice("inertial unit")
    gps = supervisor.getDevice("gps")
    # The URDF importer emits the IMU triplet as "inertial unit" +
    # "inertial unit_gyro" + "inertial unit_accel" (OmUrdfImporter, Imu kind);
    # a bare "gyro" only exists on older hand-authored models. Accept either
    # so the bridge survives the importer naming.
    gyro = supervisor.getDevice("inertial unit_gyro") or supervisor.getDevice("gyro")
    for d in (imu, gps, gyro):
        if d is None:
            print("[mavic_omnilink_bridge] ERROR: missing IMU/GPS/Gyro on Mavic")
            return
        d.enable(time_step)

    # Propeller motors. The URDF importer emits motor devices as
    # "<joint>_motor"; older hand-authored models used the bare joint name.
    # Accept either (same tolerant-lookup pattern as omnilink_arm_bridge).
    def _motor(name):
        return (supervisor.getDevice(name + "_motor")
                or supervisor.getDevice(name))

    front_left = _motor("front left propeller")
    front_right = _motor("front right propeller")
    rear_left = _motor("rear left propeller")
    rear_right = _motor("rear right propeller")
    motors = [front_left, front_right, rear_left, rear_right]
    if any(m is None for m in motors):
        print("[mavic_omnilink_bridge] ERROR: missing propeller motors")
        return
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(1.0)

    # URDF prop joints don't generate lift on their own. Drive the body
    # forces from supervisor add_force_with_offset each tick, using the
    # same motor target velocities the PID below computes.
    dynamics = RotorDynamics(self_node)

    # Gimbal pitch motor — drives the camera angle. Default to straight
    # down so /scan's world projection works out of the box.
    gimbal_pitch_motor = _motor("camera pitch")
    if gimbal_pitch_motor is None:
        print("[mavic_omnilink_bridge] WARN: no 'camera pitch' device — gimbal control disabled")
    else:
        # Start where the joint is; the per-tick ramp below walks it down
        # (GIMBAL_RATE_RAD_S) instead of slamming the airframe.
        gimbal_pitch_motor.setPosition(0.0)

    # Pose-tracking field (translation/rotation) for reset_request.
    translation_field = self_node.getField("translation")
    rotation_field = self_node.getField("rotation")
    initial_translation = list(translation_field.getSFVec3f()) if translation_field else [0.0, 0.0, 0.1]
    initial_rotation = list(rotation_field.getSFRotation()) if rotation_field else [0.0, 0.0, 1.0, 0.0]

    state = BridgeState()
    state.supervisor = supervisor   # type: ignore[attr-defined]
    state.tick_period_s = time_step / 1000.0
    state.camera_w = cam_w
    state.camera_h = cam_h
    state.camera_fov_h = cam_fov_h
    state.camera_fov_v = cam_fov_v
    state.gimbal_pitch_target = GIMBAL_DOWN_RAD
    state.gimbal_pitch_actual = 0.0   # ramps to GIMBAL_DOWN_RAD over ~1.6 s

    # Seed pose so /state is sane before the first tick.
    state.x = initial_translation[0]
    state.y = initial_translation[1]
    state.z = initial_translation[2]
    state.yaw = yaw_from_axis_angle(initial_rotation)
    state.authored_pose = {"x": state.x, "y": state.y, "z": state.z, "yaw": state.yaw}

    # Mission brief / world title.
    title, brief = _read_world_brief(supervisor)
    state.world_title = title
    state.mission_brief = brief

    # Discoverable ground-truth DEFs.
    state.gt_def_names = _collect_marker_defs(supervisor)
    print(f"[mavic_omnilink_bridge] world_title={title!r}")
    print(f"[mavic_omnilink_bridge] gt_defs: {state.gt_def_names}")

    state.omnilink_relay = setup_omnilink_relay(state)

    # Start HTTP server.
    server = ThreadingHTTPServer((BRIDGE_HOST, BRIDGE_PORT), make_handler(state))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[mavic_omnilink_bridge] HTTP listening on http://{BRIDGE_HOST}:{BRIDGE_PORT}")


    # Stabiliser PID — adapted from the stock mavic2pro.py controller.
    # The flight-loop math runs unchanged from the reference controller; we
    # add the bridge's per-mode setpoint logic on top of it.

    def _flight_step():
        roll, pitch, yaw = imu.getRollPitchYaw()
        x_pos, y_pos, altitude = gps.getValues()
        # Rates are DIFFERENCED from the InertialUnit angles rather than read
        # from the Gyro. History, because the original reason is no longer true:
        # the Gyro device used to read a constant (0,0,0) under Newton, the
        # classic law's only damping was that rate, so the attitude loop flew
        # UNDAMPED and flipped on lift-off (public issue #10).
        #
        # ⚠ CORRECTED 2026-09-11. That defect was FIXED on 2026-09-01 by
        # bde550489 (carrierBodyHandle() resolution for Gyro/Accelerometer/GPS);
        # the lane-4 probe `device.gyro` now PASSES, reading 2.0 rad/s against a
        # supervisor-measured turntable. This comment also claimed the defect was
        # "documented in AGENTS.md" -- AGENTS.md has never contained the word
        # gyro. That false citation was quoted outward in two cold emails on
        # 2026-09-11 before anyone checked it, so it is corrected here at the
        # source rather than only in the outbound copy.
        #
        # The differencing below is KEPT: it works, it is exercised, and dt is
        # the basic time step rather than wall clock so --mode=fast does not
        # distort it. Switching this flight path back to the Gyro would be a
        # behaviour change on a demo that public issue #10 was filed against,
        # and it needs its own measurement rather than a comment edit.
        dt_sim = max(time_step / 1000.0, 1e-4)
        prev_rp = state.prev_rp
        if prev_rp is None:
            roll_rate = pitch_rate = 0.0
        else:
            roll_rate = clamp(wrap_pi(roll - prev_rp[0]) / dt_sim, -MAX_ATT_RATE, MAX_ATT_RATE)
            pitch_rate = clamp(wrap_pi(pitch - prev_rp[1]) / dt_sim, -MAX_ATT_RATE, MAX_ATT_RATE)
        if prev_rp is None:
            yaw_rate = 0.0
            climb_rate = 0.0
            vx_w = vy_w = 0.0
        else:
            yaw_rate = clamp(wrap_pi(yaw - prev_rp[2]) / dt_sim, -MAX_ATT_RATE, MAX_ATT_RATE)
            climb_rate = clamp((altitude - prev_rp[3]) / dt_sim, -10.0, 10.0)
            vx_w = clamp((x_pos - prev_rp[4]) / dt_sim, -20.0, 20.0)
            vy_w = clamp((y_pos - prev_rp[5]) / dt_sim, -20.0, 20.0)
        state.prev_rp = (roll, pitch, yaw, altitude, x_pos, y_pos)
        # World -> body-horizontal: forward is the yaw heading, right is 90 deg
        # clockwise from it (z-up).
        cy, sy = math.cos(yaw), math.sin(yaw)
        v_fwd = vx_w * cy + vy_w * sy
        v_right = vx_w * sy - vy_w * cy
        roll_acc = K_ATT_D * roll_rate
        pitch_acc = K_ATT_D * pitch_rate

        # Update measured velocities from successive poses.
        now = time.time()
        prev = state.last_pose_for_v
        if prev is not None:
            dt = now - prev[3]
            if dt > 1e-3:
                state.v_xy = math.hypot(x_pos - prev[0], y_pos - prev[1]) / dt
                state.v_z = (altitude - prev[2]) / dt
        state.last_pose_for_v = (x_pos, y_pos, altitude, now)

        with state.lock:
            state.x = x_pos
            state.y = y_pos
            state.z = altitude
            state.roll = roll
            state.pitch = pitch
            state.yaw = yaw
            state.last_tick_at = now
            state.sim_time += time_step / 1000.0
            # D1: the step counter, beside the clock this loop already
            # integrates. Both are read by the HTTP threads and neither is
            # ever a Robot API call from one.
            state.sim_step += 1
            if state.fault_detector is not None:
                state.fault_detector.clear_stale()
                state.fault_detector.on_tick(state.fault,
                                             sim_time=state.sim_time,
                                             step=state.sim_step,
                                             robot="mavic2pro")
            target_x = state.target_x
            target_y = state.target_y
            target_altitude = state.target_altitude
            target_yaw = state.target_yaw
            mode = state.mode

        # Mode-specific setpoint computation.
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        # `land` sets target_altitude to 0, which used to drop the whole xy
        # block and let trim tilt carry the craft ~6 m sideways on the way down
        # (measured in issue #10) -- the land action stores the touchdown point
        # in target_x/y, so keep holding it.
        # Crash + stall checks run BEFORE the altitude gate below (issue #14): a
        # wedged aircraft that has fallen to z 0.19 m is exactly the one that must
        # be reported, and the gate used to hide it from both checks.
        if mode in ("takeoff", "hover", "goto"):
            with state.lock:
                crashed, tipped = crash_verdict(
                    roll, pitch, 0.0 if state.tipped_since is None else state.sim_time - state.tipped_since)
                if tipped and state.tipped_since is None:
                    state.tipped_since = state.sim_time
                elif not tipped:
                    state.tipped_since = None
                if crashed and state.fault in (None, "no_progress"):
                    state.fault = "crashed"
            if mode == "goto" and target_x is not None and target_y is not None:
                dist_goto = math.hypot(target_x - x_pos, target_y - y_pos)
                # Stall check -- report only, never act. Skipped once the aircraft is
                # within the reach tolerance: sitting still AT the target is not
                # progress either, and holding a hover must never be a fault.
                if dist_goto > WAYPOINT_REACH_TOL_M:
                    with state.lock:
                        state.stall_best, state.stall_since, state.fault = stall_step(
                            state.stall_best, state.stall_since, state.fault, dist_goto, state.sim_time)

        if mode in ("takeoff", "hover", "goto", "land") and (target_altitude > 0.05 or mode == "land"):
            # Hold the authored launch point throughout the climb.  Deferring
            # xy control until the last metre of ascent let small trim errors
            # integrate into a large lateral excursion before correction.
            if (mode == "land" or altitude > 0.25) and target_x is not None and target_y is not None:
                # Heading-to-target in world frame.
                dx = target_x - x_pos
                dy = target_y - y_pos
                dist_xy = math.hypot(dx, dy)
                # One law at every distance (public issue #10). The classic
                # far-field "turn toward the target, then dash nose-down, brake
                # with a fixed +0.1" cannot work with this airframe's steering
                # authority -- measured: a 12 m hop crawled 44 m the WRONG way.
                # A quad is holonomic: command tilt from the body-frame position
                # error + velocity damping, clamped, and the clamp IS the cruise
                # limit (MAX_HOLD_TILT / K_XY_V = ~1.0 m/s). Yaw steering stays
                # only so the camera faces the direction of travel.
                e_fwd = dx * cy + dy * sy
                e_right = dx * sy - dy * cy
                # +pitch_input lifts the FRONT pair -> nose up -> backward.
                if dist_xy < POS_HOLD_RADIUS_M:
                    state.xy_i_fwd = clamp(state.xy_i_fwd + K_XY_I * e_fwd * state.tick_period_s,
                                           -XY_I_CLAMP, XY_I_CLAMP)
                    state.xy_i_right = clamp(state.xy_i_right + K_XY_I * e_right * state.tick_period_s,
                                             -XY_I_CLAMP, XY_I_CLAMP)
                pitch_disturbance = clamp(-K_POS * e_fwd - state.xy_i_fwd + K_XY_V * v_fwd,
                                          -MAX_HOLD_TILT, MAX_HOLD_TILT)
                # +roll_input lifts the RIGHT pair (FR/RR faster) -> rolls left.
                roll_disturbance = clamp(-K_POS * e_right - state.xy_i_right + K_XY_V * v_right,
                                         -MAX_HOLD_TILT, MAX_HOLD_TILT)
                if dist_xy > POS_HOLD_RADIUS_M:
                    desired_heading = math.atan2(dy, dx)
                    angle_left = wrap_pi(desired_heading - yaw)
                    yaw_disturbance = MAX_YAW_DISTURBANCE * angle_left / (2.0 * math.pi)
                    if mode == "takeoff":
                        with state.lock:
                            state.mode = "goto"
                else:
                    if target_yaw is not None:
                        ye = wrap_pi(target_yaw - yaw)
                        yaw_disturbance = MAX_YAW_DISTURBANCE * ye / (2.0 * math.pi)
                    if mode in ("goto", "takeoff") and dist_xy <= WAYPOINT_REACH_TOL_M:
                        with state.lock:
                            state.mode = "hover"
            elif target_yaw is not None:
                # Climbing to altitude but operator set a yaw — still apply it.
                ye = wrap_pi(target_yaw - yaw)
                yaw_disturbance = MAX_YAW_DISTURBANCE * ye / (2.0 * math.pi)

        if mode == "land":
            # Bring vertical setpoint smoothly to ground; below 0.4 m -> idle motors.
            if altitude < 0.4 and abs(state.v_z) < 0.3:
                with state.lock:
                    state.mode = "landed"
                    state.target_altitude = 0.0
                for m in motors:
                    m.setVelocity(0.0)
                return

        # Stabiliser PID (verbatim from mavic2pro.py — don't rebalance).
        roll_input = ATT_SCALE * (K_ROLL_P * clamp(roll, -1, 1) + roll_acc + roll_disturbance)
        pitch_input = ATT_SCALE * (K_PITCH_P * clamp(pitch, -1, 1) + pitch_acc + pitch_disturbance)
        yaw_input = yaw_disturbance - K_YAW_D * yaw_rate
        # Read target_altitude AFTER the land branch may have set it to 0.
        with state.lock:
            t_alt = state.target_altitude
        clamped_diff_alt = clamp(t_alt - altitude + K_VERTICAL_OFFSET, -1, 1)
        # Integrate only when airborne with a live setpoint; freeze the trim on
        # the ground / while landed so it cannot wind up pre-takeoff.
        if t_alt > 0.05:
            state.vert_i = clamp(state.vert_i + K_VERT_I * clamped_diff_alt * dt_sim,
                                 -VERT_I_CLAMP, VERT_I_CLAMP)
        else:
            state.vert_i = 0.0
        # Linear P (the classic cube has ~zero gain near the target, which left
        # the integrator alone in charge and produced a slow limit cycle).
        vertical_input = (2.0 * K_VERTICAL_P * clamped_diff_alt
                          + state.vert_i - K_VERT_D * climb_rate)

        front_left_input = clamp(K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input, 0.0, MOTOR_MAX_RAD_S)
        front_right_input = clamp(K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input, 0.0, MOTOR_MAX_RAD_S)
        rear_left_input = clamp(K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input, 0.0, MOTOR_MAX_RAD_S)
        rear_right_input = clamp(K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input, 0.0, MOTOR_MAX_RAD_S)

        # Idle motors when the bridge is purely passive (no takeoff requested).
        if mode == "idle" or mode == "landed":
            for m in motors:
                m.setVelocity(0.0)
            return

        front_left.setVelocity(front_left_input)
        front_right.setVelocity(-front_right_input)
        rear_left.setVelocity(-rear_left_input)
        rear_right.setVelocity(rear_right_input)

        # Replace the Propeller-node lift/torque the legacy PROTO did
        # automatically. Pass raw signed inputs — RotorDynamics squares
        # for thrust and uses pair asymmetry for yaw torque.
        dynamics.step(front_left_input, front_right_input, rear_left_input, rear_right_input)

        # Gimbal: walk the commanded position toward the target at
        # GIMBAL_RATE_RAD_S (issue #14 -- a step command ejects the parked
        # airframe). `gimbal_pitch_actual` reports the commanded position.
        if gimbal_pitch_motor is not None:
            with state.lock:
                tgt = state.gimbal_pitch_target
            step_max = GIMBAL_RATE_RAD_S * state.tick_period_s
            state.gimbal_cmd += clamp(tgt - state.gimbal_cmd, -step_max, step_max)
            gimbal_pitch_motor.setPosition(state.gimbal_cmd)
            with state.lock:
                state.gimbal_pitch_actual = state.gimbal_cmd

    while supervisor.step(time_step) != -1:
        # Honour a queued reset (only the main thread is allowed to teleport).
        with state.lock:
            req = state.reset_request
            state.reset_request = None
        if req and translation_field and rotation_field:
            translation_field.setSFVec3f([req["x"], req["y"], req.get("z", 0.1)])
            rotation_field.setSFRotation([0.0, 0.0, 1.0, req.get("yaw", state.authored_pose["yaw"])])
            # A pose reset must also clear the rigid body's accumulated linear
            # and angular velocity.  Capture sessions can leave the aircraft
            # landed or drifting before a new take starts; carrying that
            # momentum through the teleport makes the repeated take diverge
            # even though its authored start pose is identical.
            try:
                self_node.resetPhysics()
            except Exception:
                try:
                    self_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                except Exception as exc:
                    print(f"[mavic_omnilink_bridge] resetPhysics failed: {exc}")
            with state.lock:
                state.last_pose_for_v = None
                state.v_xy = 0.0
                state.v_z = 0.0

        _flight_step()

        # Drain wwi inbox (robot-window messages from the chat side panel).
        while True:
            msg = supervisor.wwiReceiveText()
            if msg is None or msg == "":
                break
            try:
                handle_wwi_message(state, msg)
            except Exception as exc:
                with state.lock:
                    state.window_outbox.append(f"error:chat_router_exception: {exc!r}")

        # Flush window outbox (configure / status / agent / tool lines).
        with state.lock:
            outbox = state.window_outbox
            state.window_outbox = []
        for line in outbox:
            try:
                supervisor.wwiSendText(line)
            except Exception as exc:
                print(f"[mavic_omnilink_bridge] wwiSendText failed: {exc}")

        # Honour camera capture requests (cheap — Webots' getImage reuses
        # the buffer the renderer already produced this tick).
        with state.lock:
            req = state.frame_request
            served = state.frame_served
        if req > served:
            try:
                raw = camera.getImage()
                if raw:
                    with state.lock:
                        state.frame_bgra = raw
                        state.frame_pose = (state.x, state.y, state.z,
                                             state.yaw, state.gimbal_pitch_actual)
                        state.frame_sim_time = state.sim_time
                        state.frame_served = req
                else:
                    with state.lock:
                        state.frame_served = req
            except Exception as exc:
                print(f"[mavic_omnilink_bridge] camera capture failed: {exc}")
                with state.lock:
                    state.frame_served = req
    # Clean shutdown: flush the action journal the 30 s beat has not sent.
    if close_relay is not None:
        close_relay(getattr(state, "omnilink_relay", None))


if __name__ == "__main__":
    main()
