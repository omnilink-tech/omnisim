#!/usr/bin/env python3
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

"""Fixed-wing flight software for the OmniSim hardware-in-the-loop lane.

This is the AUTOPILOT half of the rig, and it is a separate process on purpose.
It binds a UDP port, decodes the MAVLink HIL sensor stream an aircraft sends it,
runs a cascaded fixed-wing control stack, and answers with
``HIL_ACTUATOR_CONTROLS``. Nothing here imports OmniSim, knows what a Solid is,
or can see the simulator's state except through the socket -- which is the only
arrangement under which "the flight software flew the mission" means anything.
Swapping in PX4 SITL, ArduPlane SITL, or a board on a bench is a substitution at
this boundary, not a rewrite.

The cascade
-----------
Standard fixed-wing loop ordering, slowest outer to fastest inner::

    waypoint  -> bearing  -> heading loop -> bank command
    altitude  -> climb rate -> pitch command
    airspeed  -> throttle
                              bank/pitch command -> attitude loop -> surfaces

Each stage is a pure function of its inputs plus, where it needs one, an
explicit integrator. They are module-level functions rather than methods so
``tests/test_autopilot.py`` can exercise the control laws with no socket, no
simulator and no mission.

What this is NOT
----------------
There is no state estimator. Attitude, position and velocity are taken from
``HIL_STATE_QUATERNION``, which is ground truth from the simulator; body rates
and airspeed are taken from ``HIL_SENSOR``, which is what a real autopilot reads
off its gyro and pitot. So this rig exercises control laws, mission logic and
the protocol, and it does NOT exercise an EKF -- a real autopilot's estimator is
a large share of its failure surface and none of that share is under test here.
It is also not L1 or TECS: navigation is a bearing to the active waypoint with a
bank-limited turn, and the energy loops are separate rather than coupled. Both
are named for what they are so nobody quotes this as more than it is.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from omnisim_hil.frames import global_to_local  # noqa: E402
from omnisim_hil.mavlink import MavlinkCodec  # noqa: E402

# The simulated world's geodetic origin. It has to match the controller's
# constants because the mission is written in local ENU metres about the WORLD
# origin, not about wherever the aircraft happened to be launched.
DEFAULT_ORIGIN_LAT = 47.397742
DEFAULT_ORIGIN_LON = 8.545594

SEA_LEVEL_DENSITY = 1.225

# -- limits ----------------------------------------------------------------

BANK_LIMIT_RAD = math.radians(35.0)
PITCH_LIMIT_RAD = math.radians(20.0)
CLIMB_LIMIT_M_S = 4.0

# Airspeed floor. The airframe stalls near 0.30 rad of alpha, which for the
# 2.5 kg delivery aircraft is about 9 m/s clean; 13 m/s is that with the usual
# 1.3 margin. THIS IS THE LIMIT THAT STOPS THE OUTER LOOP FROM KILLING THE
# AIRCRAFT: the altitude loop is an altitude loop and knows nothing about energy,
# so when the aircraft is low and slow it will keep demanding nose-up to make its
# climb rate, and nose-up at low speed is exactly how an autopilot flies an
# aeroplane into a departure. apply_stall_floor() outranks it and can only ever
# push the nose DOWN, never raise a pitch command.
STALL_FLOOR_M_S = 13.0
STALL_FLOOR_GAIN = 0.06        # rad of nose-down per m/s below the floor

# -- surface channel signs -------------------------------------------------
#
# Measured from omnisim_hil.aero by summing r x F over every surface of
# delivery_aircraft() at its 16.8 m/s trim, in the simulator's FLU body frame:
#
#   aileron  +0.30 -> +5.03 N.m about +X (forward) -> left wing UP  -> +roll
#   elevator +0.30 -> +2.46 N.m about +Y (left)    -> nose DOWN     -> -pitch
#   rudder   +0.30 -> +1.40 N.m about +Z (up)      -> nose LEFT     -> -yaw
#
# Every control law below is written in the aerospace sense -- positive roll is
# right wing down, positive pitch is nose up, positive yaw is nose right -- and
# this is the single place that sense is mapped onto the wire. Two of the three
# channels are inverted with respect to it. A sign discovered inside a control
# law instead of here is a sign that will be discovered again.
CHANNEL_SIGN_AILERON = +1.0
CHANNEL_SIGN_ELEVATOR = -1.0
CHANNEL_SIGN_RUDDER = -1.0


def clamp(value: float, low: float, high: float) -> float:
    return low if value < low else (high if value > high else value)


def wrap_pi(angle: float) -> float:
    """Fold an angle into (-pi, pi].

    Used on every heading difference. Without it a turn from 179 deg to
    -179 deg -- two degrees apart -- reads as 358 degrees of error and the
    aircraft rolls to its bank limit and goes the long way round.
    """
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def bearing_to(east: float, north: float, target_east: float, target_north: float) -> float:
    """Compass bearing in radians from a local ENU position to a target.

    Zero is north and positive is toward east, matching the yaw that
    :func:`omnisim_hil.frames.euler_frd_ned` produces, so the two can be
    subtracted directly. The argument order is (east, north) because that is the
    order the rest of this file carries a position in; the atan2 arguments are
    the other way round, which is the whole trap.
    """
    return math.atan2(target_east - east, target_north - north)


def distance_to(east: float, north: float, target_east: float, target_north: float) -> float:
    return math.hypot(target_east - east, target_north - north)


# --------------------------------------------------------------------------
# Control laws. Pure functions: no state, no clock, no socket.
# --------------------------------------------------------------------------


def heading_to_bank(heading_error: float, gain: float = 1.1) -> float:
    """Heading error (already wrapped) to a bank command, hard-limited.

    Proportional and limited, not a coordinated-turn-rate inversion. The limit
    is what keeps the turn inside the airframe's envelope: at 35 degrees the
    load factor is 1/cos(35) = 1.22 g, which raises the stall speed by 10 per
    cent -- accounted for by the airspeed floor, which is why the two limits
    belong to the same design and not to two different people.
    """
    return clamp(gain * heading_error, -BANK_LIMIT_RAD, BANK_LIMIT_RAD)


def altitude_to_climb_rate(altitude_error: float, gain: float = 0.35) -> float:
    """Altitude error (commanded minus actual) to a climb-rate command."""
    return clamp(gain * altitude_error, -CLIMB_LIMIT_M_S, CLIMB_LIMIT_M_S)


def apply_stall_floor(
    pitch_cmd: float,
    airspeed: float,
    floor: float = STALL_FLOOR_M_S,
    gain: float = STALL_FLOOR_GAIN,
) -> float:
    """Clip a pitch command so the altitude loop cannot command a stall.

    One-sided by construction: above the floor the command passes through
    untouched, and below it the return value is the MINIMUM of the command and a
    nose-down ceiling proportional to the deficit. It can never raise a pitch
    command, so it cannot itself drive the aircraft anywhere -- it can only
    decline to climb.
    """
    if airspeed >= floor:
        return pitch_cmd
    ceiling = -gain * (floor - airspeed)
    return min(pitch_cmd, ceiling)


def roll_axis(roll_error: float, roll_rate: float, kp: float = 1.6, kd: float = 0.30) -> float:
    """Roll angle error and body roll rate to an aileron command, aerospace sense."""
    return clamp(kp * roll_error - kd * roll_rate, -1.0, 1.0)


def pitch_axis(pitch_error: float, pitch_rate: float, kp: float = 2.4, kd: float = 0.45) -> float:
    """Pitch angle error and body pitch rate to an elevator command, aerospace sense."""
    return clamp(kp * pitch_error - kd * pitch_rate, -1.0, 1.0)


def yaw_axis(yaw_rate: float, bank: float, kd: float = 0.35, k_turn: float = 0.25) -> float:
    """Yaw damper plus a turn-coordination feed-forward, aerospace sense.

    The feed-forward is proportional to bank rather than to a computed sideslip:
    there is no sideslip vane in the HIL sensor set, and estimating beta from the
    accelerometer is an estimator, which this autopilot deliberately does not
    have. So this reduces adverse yaw in a turn without claiming to null it.
    """
    return clamp(-kd * yaw_rate + k_turn * math.sin(bank), -1.0, 1.0)


class Integrator:
    """A clamped integrator. Separate class so anti-windup is impossible to skip.

    The clamp is on the accumulated term, not on the output of the loop that
    uses it, because a loop that saturates for ten seconds and then has to
    unwind an unbounded integral is a loop that overshoots by the same ten
    seconds' worth of error.
    """

    def __init__(self, gain: float, limit: float) -> None:
        self.gain = gain
        self.limit = limit
        self.value = 0.0

    def update(self, error: float, dt: float) -> float:
        self.value = clamp(self.value + error * dt, -self.limit / max(self.gain, 1e-9),
                           self.limit / max(self.gain, 1e-9))
        return self.gain * self.value

    def reset(self) -> None:
        self.value = 0.0


# --------------------------------------------------------------------------
# Navigation
# --------------------------------------------------------------------------


class Waypoint:
    __slots__ = ("east", "north", "altitude", "label", "acceptance", "is_rtb")

    def __init__(self, east: float, north: float, altitude: float, label: str,
                 acceptance: float, is_rtb: bool = False) -> None:
        self.east = east
        self.north = north
        self.altitude = altitude
        self.label = label
        self.acceptance = acceptance
        self.is_rtb = is_rtb

    def __repr__(self) -> str:  # pragma: no cover - diagnostic only
        return "Waypoint(%s, %.1f, %.1f, %.1f)" % (
            self.label, self.east, self.north, self.altitude)


class Navigator:
    """Sequential waypoint follower over a fixed list.

    Two switching rules, and no others:

    * inside the acceptance radius, and
    * past the waypoint -- the waypoint lies behind the aircraft's ground track
      -- while still inside a capture radius.

    The second rule exists because a turn radius can exceed an acceptance
    radius: at 17 m/s and 35 degrees of bank the aircraft turns in 42 m, so a
    30 m acceptance circle placed after a sharp turn can be missed entirely, and
    a follower with only the first rule then orbits it for the rest of the
    flight. It is a standard abeam test, not a fudge, and it is bounded by the
    capture radius so it can never fire from far away.

    The index is monotonic. It is never decremented and never jumps: a follower
    that can go backwards can be trapped between two waypoints forever.
    """

    def __init__(self, waypoints: Sequence[Waypoint], capture_multiplier: float = 2.5) -> None:
        if not waypoints:
            raise ValueError("a mission needs at least one waypoint")
        self.waypoints: Tuple[Waypoint, ...] = tuple(waypoints)
        self.index = 0
        self.capture_multiplier = capture_multiplier
        self.reached: List[Tuple[str, float]] = []

    @property
    def active(self) -> Waypoint:
        return self.waypoints[min(self.index, len(self.waypoints) - 1)]

    @property
    def complete(self) -> bool:
        return self.index >= len(self.waypoints)

    def update(self, east: float, north: float, track_east: float, track_north: float,
               t: float = 0.0) -> bool:
        """Advance at most one waypoint. Returns True if it switched."""
        if self.complete:
            return False
        wp = self.active
        dist = distance_to(east, north, wp.east, wp.north)
        if dist <= wp.acceptance:
            self._advance(wp, t)
            return True

        if dist <= wp.acceptance * self.capture_multiplier:
            # Abeam test: the vector to the waypoint no longer has a forward
            # component along the ground track, so the aircraft is moving away
            # from it and closing is no longer possible on this pass.
            to_wp = (wp.east - east, wp.north - north)
            speed = math.hypot(track_east, track_north)
            if speed > 1e-3:
                forward = (to_wp[0] * track_east + to_wp[1] * track_north) / speed
                if forward < 0.0:
                    self._advance(wp, t)
                    return True
        return False

    def _advance(self, wp: Waypoint, t: float) -> None:
        self.reached.append((wp.label, t))
        self.index += 1


def load_mission(path: str, default_acceptance: float = 30.0,
                 default_altitude: float = 45.0) -> Tuple[str, List[Waypoint]]:
    with open(path, "r", encoding="utf-8") as handle:
        blob = json.load(handle)
    raw = blob.get("waypoints") or []
    if not raw:
        raise ValueError("%s declares no waypoints" % path)
    name = blob.get("name") or os.path.basename(path)
    acceptance = float(blob.get("acceptance_radius", default_acceptance))
    cruise = float(blob.get("cruise_altitude", default_altitude))
    out: List[Waypoint] = []
    for i, item in enumerate(raw):
        out.append(Waypoint(
            east=float(item["x"]),
            north=float(item["y"]),
            altitude=float(item.get("alt", cruise)),
            label=str(item.get("label", "WP%d" % (i + 1))),
            acceptance=float(item.get("acceptance", acceptance)),
            # A mission may mark its return leg explicitly; otherwise the last
            # waypoint is the one you go home to, which is what "return to base"
            # means in every mission file that does not say.
            is_rtb=bool(item.get("rtb", i == len(raw) - 1)),
        ))
    return name, out


# --------------------------------------------------------------------------
# Vehicle state, decoded off the wire
# --------------------------------------------------------------------------


class VehicleState:
    """Everything the control stack knows, and where each field came from."""

    def __init__(self) -> None:
        self.have_attitude = False
        self.have_sensor = False
        self.t = 0.0                # sim seconds, from the sensor time stamp
        self.east = 0.0
        self.north = 0.0
        self.altitude = 0.0
        self.roll = 0.0             # aerospace: right wing down positive
        self.pitch = 0.0            # aerospace: nose up positive
        self.yaw = 0.0             # compass radians, 0 = north
        self.p = 0.0                # body roll rate, from the gyro
        self.q = 0.0                # body pitch rate
        self.r = 0.0                # body yaw rate
        self.airspeed = 0.0         # indicated, from the pitot differential
        self.true_airspeed = 0.0
        self.vn = 0.0
        self.ve = 0.0
        self.vd = 0.0
        self.gps_east = 0.0
        self.gps_north = 0.0
        self.have_gps = False

    @property
    def climb_rate(self) -> float:
        return -self.vd

    @property
    def ground_speed(self) -> float:
        return math.hypot(self.vn, self.ve)


def euler_from_quaternion(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) from a MAVLink (w,x,y,z) attitude quaternion.

    The quaternion is already FRD-in-NED, so this is the plain aerospace
    Z-Y-X extraction and no frame conversion belongs here.
    """
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


# --------------------------------------------------------------------------
# The autopilot
# --------------------------------------------------------------------------

WAIT_FOR_SENSORS = "WAIT_FOR_SENSORS"
ARMED = "ARMED"
CRUISE = "CRUISE"
NAV = "NAV"
RTB = "RTB"
LOITER = "LOITER"


class Autopilot:
    """The control stack and its mission state machine.

    Holds no socket. :meth:`tick` takes a decoded :class:`VehicleState` and
    returns the four normalised channel values, so the whole thing can be flown
    against a desktop model with no MAVLink at all.
    """

    def __init__(self, navigator: Navigator, cruise_airspeed: float = 16.0,
                 arm_settle_s: float = 1.0, loiter_bank_deg: float = 15.0) -> None:
        self.nav = navigator
        self.cruise_airspeed = cruise_airspeed
        self.arm_settle_s = arm_settle_s
        self.loiter_bank = math.radians(loiter_bank_deg)

        self.state = WAIT_FOR_SENSORS
        self.state_since = 0.0
        self.armed_heading = 0.0
        self.mission_complete_t: Optional[float] = None

        # Only the two loops that need to hold a non-zero output at zero error
        # get an integrator: pitch, because level flight needs a trim attitude,
        # and throttle, because level flight needs a trim power setting. Adding
        # one to the roll loop would integrate against a symmetric airframe.
        self.pitch_i = Integrator(gain=0.020, limit=0.12)
        self.throttle_i = Integrator(gain=0.020, limit=0.35)

        self.last_command: Dict[str, float] = {
            "aileron": 0.0, "elevator": 0.0, "rudder": 0.0, "throttle": 0.0}
        self.debug: Dict[str, float] = {}

    # -- state machine -----------------------------------------------------

    def _enter(self, state: str, t: float) -> None:
        if state != self.state:
            self.state = state
            self.state_since = t

    def _advance_state(self, s: VehicleState) -> None:
        if self.state == WAIT_FOR_SENSORS:
            if s.have_attitude and s.have_sensor:
                self.armed_heading = s.yaw
                self._enter(ARMED, s.t)
            return

        if self.state == ARMED:
            if s.t - self.state_since >= self.arm_settle_s:
                self._enter(CRUISE, s.t)
            return

        if self.state == CRUISE:
            # Leave the climb-out only once the altitude is roughly made, so
            # the first navigation leg is not flown while pitching hard.
            if abs(self.nav.active.altitude - s.altitude) < 6.0:
                self._enter(RTB if self.nav.active.is_rtb else NAV, s.t)
            return

        if self.state in (NAV, RTB):
            if self.nav.complete:
                if self.mission_complete_t is None:
                    self.mission_complete_t = s.t
                self._enter(LOITER, s.t)
            elif self.nav.active.is_rtb:
                self._enter(RTB, s.t)
            else:
                self._enter(NAV, s.t)

    # -- one control update ------------------------------------------------

    def tick(self, s: VehicleState, dt: float) -> Dict[str, float]:
        """Run the cascade once. ``dt`` is SIMULATED seconds, never wall clock."""
        self._advance_state(s)

        if self.state == WAIT_FOR_SENSORS:
            # Nothing is commanded before the first real sensor message. An
            # autopilot that outputs during startup is an autopilot that has
            # already moved a surface it had no state for.
            return dict(self.last_command)

        dt = clamp(dt, 0.0, 0.2)

        # -- navigation ----------------------------------------------------
        wp = self.nav.active
        if self.state in (NAV, RTB):
            self.nav.update(s.east, s.north, s.ve, s.vn, s.t)
            self._advance_state(s)
            wp = self.nav.active

        if self.state in (ARMED, CRUISE):
            heading_cmd = self.armed_heading
        elif self.state == LOITER:
            heading_cmd = None
        else:
            heading_cmd = bearing_to(s.east, s.north, wp.east, wp.north)

        altitude_cmd = wp.altitude

        # -- outer loops ---------------------------------------------------
        if heading_cmd is None:
            bank_cmd = self.loiter_bank
            heading_error = 0.0
        else:
            heading_error = wrap_pi(heading_cmd - s.yaw)
            bank_cmd = heading_to_bank(heading_error)

        climb_cmd = altitude_to_climb_rate(altitude_cmd - s.altitude)
        climb_error = climb_cmd - s.climb_rate
        pitch_cmd = 0.10 * climb_error + self.pitch_i.update(climb_error, dt)

        # A banked wing carries less of the aircraft's weight vertically, so a
        # turn sinks unless the pitch command is raised by the load factor. This
        # is a feed-forward, not a correction: without it the altitude loop
        # discovers every turn only after it has already lost height.
        pitch_cmd += 0.35 * (1.0 / max(math.cos(clamp(s.roll, -BANK_LIMIT_RAD,
                                                      BANK_LIMIT_RAD)), 0.5) - 1.0)
        pitch_cmd = clamp(pitch_cmd, -PITCH_LIMIT_RAD, PITCH_LIMIT_RAD)
        pitch_cmd = apply_stall_floor(pitch_cmd, s.airspeed)

        # 0.65 is the measured trim setting for this airframe near 16 m/s, not a
        # guess: leaving the integrator to discover it from a low base is what
        # pins the throttle against its clamp and leaves the speed loop with
        # authority in one direction only.
        speed_error = self.cruise_airspeed - s.airspeed
        throttle = clamp(0.65 + 0.055 * speed_error + self.throttle_i.update(speed_error, dt)
                         + 0.030 * climb_cmd, 0.0, 1.0)
        if s.airspeed < STALL_FLOOR_M_S:
            # Energy first: the same condition that refuses the climb also asks
            # for every watt the propeller has.
            throttle = 1.0

        # -- inner loop ----------------------------------------------------
        aileron = roll_axis(bank_cmd - s.roll, s.p)
        elevator = pitch_axis(pitch_cmd - s.pitch, s.q)
        rudder = yaw_axis(s.r, s.roll)

        command = {
            "aileron": clamp(CHANNEL_SIGN_AILERON * aileron, -1.0, 1.0),
            "elevator": clamp(CHANNEL_SIGN_ELEVATOR * elevator, -1.0, 1.0),
            "rudder": clamp(CHANNEL_SIGN_RUDDER * rudder, -1.0, 1.0),
            "throttle": throttle,
        }
        self.last_command = command
        self.debug = {
            "heading_cmd": 0.0 if heading_cmd is None else heading_cmd,
            "heading_error": heading_error,
            "bank_cmd": bank_cmd,
            "climb_cmd": climb_cmd,
            "pitch_cmd": pitch_cmd,
            "altitude_cmd": altitude_cmd,
            "wp_index": float(self.nav.index),
            "wp_distance": distance_to(s.east, s.north, wp.east, wp.north),
        }
        return command


# --------------------------------------------------------------------------
# MAVLink link and the run loop
# --------------------------------------------------------------------------


class Link:
    """UDP endpoint. Binds, decodes, and answers whoever last spoke to it.

    Replying to the sender's address rather than a configured one is what makes
    the rig work with an ephemeral simulator port, which is what the OmniSim
    controller uses.
    """

    def __init__(self, host: str, port: int, sysid: int = 1, compid: int = 1) -> None:
        self.codec = MavlinkCodec(sysid=sysid, compid=compid)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            # A HIL sensor stream from a sim running many times faster than real
            # time arrives in bursts. A default receive buffer drops them, and a
            # dropped sensor message reads exactly like a slow control loop.
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)
        self.peer: Optional[Tuple[str, int]] = None
        self.rx_counts: Dict[str, int] = {}
        self.tx_packets = 0
        self.rx_bytes = 0
        self.tx_bytes = 0

    def poll(self, budget: int = 256) -> List[Any]:
        """Return every message currently available, oldest first."""
        out: List[Any] = []
        try:
            data, addr = self.sock.recvfrom(4096)
        except socket.timeout:
            return out
        except OSError:
            return out
        self.peer = addr
        self.rx_bytes += len(data)
        out.extend(self.codec.feed(data))

        self.sock.setblocking(False)
        try:
            for _ in range(budget):
                try:
                    data, addr = self.sock.recvfrom(4096)
                except (BlockingIOError, socket.timeout, OSError):
                    break
                self.peer = addr
                self.rx_bytes += len(data)
                out.extend(self.codec.feed(data))
        finally:
            self.sock.settimeout(0.05)

        for msg in out:
            self.rx_counts[msg.name] = self.rx_counts.get(msg.name, 0) + 1
        return out

    def send_actuators(self, t_usec: int, command: Dict[str, float]) -> None:
        if self.peer is None:
            return
        controls = [0.0] * 16
        controls[0] = command["aileron"]
        controls[1] = command["elevator"]
        controls[2] = command["rudder"]
        controls[3] = command["throttle"]
        frame = self.codec.encode(
            "HIL_ACTUATOR_CONTROLS",
            time_usec=t_usec,
            controls=controls,
            mode=1,
            flags=0,
        )
        try:
            self.sock.sendto(frame, self.peer)
        except OSError:
            return
        self.tx_packets += 1
        self.tx_bytes += len(frame)

    def send_heartbeat(self) -> None:
        if self.peer is None:
            return
        frame = self.codec.encode(
            "HEARTBEAT", type=1, autopilot=12, base_mode=209,
            system_status=4, mavlink_version=3,
        )
        try:
            self.sock.sendto(frame, self.peer)
        except OSError:
            return
        self.tx_packets += 1
        self.tx_bytes += len(frame)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class FlightRecorder:
    """Running statistics, so the final report is measured rather than claimed."""

    def __init__(self) -> None:
        self.ticks = 0
        self.distance_m = 0.0
        self._last_xy: Optional[Tuple[float, float]] = None
        self.alt_error_sum = 0.0
        self.alt_error_max = 0.0
        self.alt_error_n = 0
        # A second, settled window. The commanded altitude STEPS at every
        # waypoint, so the raw worst-case error is dominated by the size of the
        # step the autopilot was asked to fly, not by how well it holds -- and
        # quoting that as tracking error is quoting the mission back as a defect.
        self.settled_sum = 0.0
        self.settled_max = 0.0
        self.settled_n = 0
        self._alt_cmd = None
        self._alt_cmd_changed_t = 0.0
        self.settle_delay_s = 5.0
        self.airspeed_min = float("inf")
        self.airspeed_max = 0.0
        self.airspeed_sum = 0.0
        self.altitude_min = float("inf")
        self.altitude_max = 0.0
        self.bank_max = 0.0
        self.t_first: Optional[float] = None
        self.t_last = 0.0
        self.control_dt_sum = 0.0
        self.gps_disagreement_max = 0.0

    def sample(self, s: VehicleState, altitude_cmd: float, tracking: bool, dt: float) -> None:
        self.ticks += 1
        if self.t_first is None:
            self.t_first = s.t
        self.t_last = s.t
        self.control_dt_sum += dt
        if self._last_xy is not None:
            self.distance_m += math.hypot(s.east - self._last_xy[0], s.north - self._last_xy[1])
        self._last_xy = (s.east, s.north)

        self.airspeed_min = min(self.airspeed_min, s.airspeed)
        self.airspeed_max = max(self.airspeed_max, s.airspeed)
        self.airspeed_sum += s.airspeed
        self.altitude_min = min(self.altitude_min, s.altitude)
        self.altitude_max = max(self.altitude_max, s.altitude)
        self.bank_max = max(self.bank_max, abs(s.roll))
        if s.have_gps:
            self.gps_disagreement_max = max(
                self.gps_disagreement_max,
                math.hypot(s.gps_east - s.east, s.gps_north - s.north))

        if self._alt_cmd is None or abs(altitude_cmd - self._alt_cmd) > 1e-6:
            self._alt_cmd = altitude_cmd
            self._alt_cmd_changed_t = s.t

        if tracking:
            # Only score altitude tracking where an altitude was actually being
            # tracked. Scoring the climb-out too would report a tracking error
            # that is really a commanded manoeuvre.
            err = abs(altitude_cmd - s.altitude)
            self.alt_error_sum += err
            self.alt_error_max = max(self.alt_error_max, err)
            self.alt_error_n += 1
            if s.t - self._alt_cmd_changed_t >= self.settle_delay_s:
                self.settled_sum += err
                self.settled_max = max(self.settled_max, err)
                self.settled_n += 1

    def summary(self) -> Dict[str, Any]:
        n = max(self.ticks, 1)
        return {
            "control_ticks": self.ticks,
            "sim_seconds": round(self.t_last - (self.t_first or 0.0), 3),
            "distance_m": round(self.distance_m, 2),
            "altitude_error_mean_m": (round(self.alt_error_sum / self.alt_error_n, 3)
                                      if self.alt_error_n else None),
            "altitude_error_max_m": (round(self.alt_error_max, 3)
                                     if self.alt_error_n else None),
            "altitude_tracked_ticks": self.alt_error_n,
            "altitude_settled_error_mean_m": (round(self.settled_sum / self.settled_n, 3)
                                              if self.settled_n else None),
            "altitude_settled_error_max_m": (round(self.settled_max, 3)
                                             if self.settled_n else None),
            "altitude_settled_ticks": self.settled_n,
            "settle_delay_s": self.settle_delay_s,
            "altitude_min_m": round(self.altitude_min, 2) if self.ticks else None,
            "altitude_max_m": round(self.altitude_max, 2) if self.ticks else None,
            "airspeed_min_m_s": round(self.airspeed_min, 2) if self.ticks else None,
            "airspeed_max_m_s": round(self.airspeed_max, 2) if self.ticks else None,
            "airspeed_mean_m_s": round(self.airspeed_sum / n, 2),
            "bank_max_deg": round(math.degrees(self.bank_max), 2),
            "mean_control_dt_ms": round(1000.0 * self.control_dt_sum / n, 3),
            "gps_vs_truth_max_m": round(self.gps_disagreement_max, 3),
        }


def decode_into(state: VehicleState, msg: Any, origin_lat: float, origin_lon: float) -> bool:
    """Fold one decoded message into the vehicle state. True if it was a sensor tick."""
    if msg.name == "HIL_SENSOR":
        state.have_sensor = True
        state.t = msg["time_usec"] / 1e6
        # Body rates straight off the gyro, in FRD. No estimator involved: this
        # is the one quantity an autopilot really does read directly.
        state.p = float(msg["xgyro"])
        state.q = float(msg["ygyro"])
        state.r = float(msg["zgyro"])
        # Indicated airspeed from the pitot differential, exactly as a real
        # air-data unit computes it. diff_pressure is hPa on the wire.
        q_pa = max(float(msg["diff_pressure"]) * 100.0, 0.0)
        state.airspeed = math.sqrt(2.0 * q_pa / SEA_LEVEL_DENSITY)
        return True

    if msg.name == "HIL_STATE_QUATERNION":
        state.have_attitude = True
        quat = msg["attitude_quaternion"]
        state.roll, state.pitch, state.yaw = euler_from_quaternion(
            quat[0], quat[1], quat[2], quat[3])
        east, north, _ = global_to_local(
            msg["lat"] / 1e7, msg["lon"] / 1e7, 0.0, origin_lat, origin_lon)
        state.east = east
        state.north = north
        state.altitude = msg["alt"] / 1000.0
        state.vn = msg["vx"] / 100.0
        state.ve = msg["vy"] / 100.0
        state.vd = msg["vz"] / 100.0
        state.true_airspeed = msg["true_airspeed"] / 100.0
        return False

    if msg.name == "HIL_GPS":
        state.have_gps = True
        east, north, _ = global_to_local(
            msg["lat"] / 1e7, msg["lon"] / 1e7, 0.0, origin_lat, origin_lon)
        state.gps_east = east
        state.gps_north = north
        return False

    return False


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Fixed-wing autopilot for the OmniSim MAVLink HIL lane.")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=14560,
                   help="UDP port to BIND; the aircraft sends here (default 14560)")
    p.add_argument("--mission", default=None,
                   help="mission JSON; defaults to missions/delivery_route.json")
    p.add_argument("--cruise-airspeed", type=float, default=16.0,
                   help="commanded indicated airspeed; the airframe trims near "
                        "16.8 m/s, so 16.0 leaves the throttle loop authority "
                        "in both directions (default 16.0)")
    p.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT)
    p.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON)
    p.add_argument("--log", default="", help="write per-tick decisions as JSONL")
    p.add_argument("--log-decimate", type=int, default=4,
                   help="write one log line per N control ticks (default 4)")
    p.add_argument("--summary", default="", help="write the final report as JSON")
    p.add_argument("--status-interval", type=float, default=2.0,
                   help="seconds of SIM time between status lines")
    p.add_argument("--link-timeout", type=float, default=5.0,
                   help="wall seconds of silence after which the flight is over")
    p.add_argument("--startup-timeout", type=float, default=120.0,
                   help="wall seconds to wait for the first sensor message")
    p.add_argument("--max-sim-seconds", type=float, default=0.0,
                   help="stop after this much SIM time (0 = no limit)")
    p.add_argument("--loiter-seconds", type=float, default=8.0,
                   help="sim seconds to hold after the mission completes")
    return p.parse_args(argv)


def default_mission_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "missions", "delivery_route.json")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    mission_path = args.mission or default_mission_path()
    mission_name, waypoints = load_mission(mission_path)
    nav = Navigator(waypoints)
    autopilot = Autopilot(nav, cruise_airspeed=args.cruise_airspeed)

    link = Link(args.host, args.port)
    print("[autopilot] %s: %d waypoints, bound udp %s:%d"
          % (mission_name, len(waypoints), args.host, args.port), flush=True)
    for wp in waypoints:
        print("[autopilot]   %-10s e=%+8.1f n=%+8.1f alt=%5.1f r=%.0f%s"
              % (wp.label, wp.east, wp.north, wp.altitude, wp.acceptance,
                 "  (RTB)" if wp.is_rtb else ""), flush=True)
    print("[autopilot] state %s -- no surface is commanded until the aircraft "
          "speaks first" % WAIT_FOR_SENSORS, flush=True)

    state = VehicleState()
    recorder = FlightRecorder()
    log = None
    if args.log:
        try:
            log = open(args.log, "w", encoding="utf-8")
        except OSError as exc:
            print("[autopilot] log disabled: %r" % (exc,), file=sys.stderr, flush=True)

    started = time.time()
    last_rx_wall = None
    last_status_t = -1e9
    last_heartbeat_t = -1e9
    prev_t = None
    reason = "unknown"
    ticks = 0

    try:
        while True:
            messages = link.poll()
            now_wall = time.time()

            if not messages:
                if last_rx_wall is None:
                    if now_wall - started > args.startup_timeout:
                        reason = "no sensor message within %.0f s" % args.startup_timeout
                        break
                elif now_wall - last_rx_wall > args.link_timeout:
                    reason = "link silent for %.1f s (the flight ended)" % args.link_timeout
                    break
                continue

            last_rx_wall = now_wall
            sensor_ticks = 0
            for msg in messages:
                if decode_into(state, msg, args.origin_lat, args.origin_lon):
                    sensor_ticks += 1

            if not sensor_ticks or not state.have_attitude:
                continue

            dt = 0.0 if prev_t is None else (state.t - prev_t)
            prev_t = state.t
            if dt < 0.0:
                # A sim restart rewinds the clock. Drop the integrators rather
                # than integrate a negative interval.
                autopilot.pitch_i.reset()
                autopilot.throttle_i.reset()
                dt = 0.0

            was_state = autopilot.state
            command = autopilot.tick(state, dt)
            link.send_actuators(int(state.t * 1e6), command)
            ticks += 1

            if autopilot.state != WAIT_FOR_SENSORS:
                recorder.sample(
                    state,
                    autopilot.debug.get("altitude_cmd", 0.0),
                    autopilot.state in (NAV, RTB, LOITER),
                    dt,
                )

            if autopilot.state != was_state:
                print("[autopilot] t=%7.2f  %s -> %s" % (state.t, was_state, autopilot.state),
                      flush=True)
            if nav.reached and nav.reached[-1][1] == state.t:
                print("[autopilot] t=%7.2f  reached %s  (alt %.1f m, %.1f m/s)"
                      % (state.t, nav.reached[-1][0], state.altitude, state.airspeed),
                      flush=True)

            if state.t - last_heartbeat_t >= 1.0:
                last_heartbeat_t = state.t
                link.send_heartbeat()

            if state.t - last_status_t >= args.status_interval:
                last_status_t = state.t
                wp = nav.active
                print("[autopilot] t=%7.2f  %-5s alt %6.1f/%5.1f m  ias %5.1f m/s  "
                      "hdg %6.1f deg  roll %+6.1f  ->%-10s %6.1f m  thr %.2f"
                      % (state.t, autopilot.state, state.altitude,
                         autopilot.debug.get("altitude_cmd", 0.0), state.airspeed,
                         math.degrees(state.yaw) % 360.0, math.degrees(state.roll),
                         wp.label, autopilot.debug.get("wp_distance", 0.0),
                         command["throttle"]), flush=True)

            if log is not None and ticks % max(args.log_decimate, 1) == 0:
                log.write(json.dumps({
                    "t": round(state.t, 4),
                    "state": autopilot.state,
                    "east": round(state.east, 3), "north": round(state.north, 3),
                    "alt": round(state.altitude, 3),
                    "alt_cmd": round(autopilot.debug.get("altitude_cmd", 0.0), 3),
                    "roll": round(state.roll, 5), "pitch": round(state.pitch, 5),
                    "yaw": round(state.yaw, 5),
                    "p": round(state.p, 5), "q": round(state.q, 5), "r": round(state.r, 5),
                    "ias": round(state.airspeed, 3), "tas": round(state.true_airspeed, 3),
                    "climb": round(state.climb_rate, 3),
                    "bank_cmd": round(autopilot.debug.get("bank_cmd", 0.0), 5),
                    "pitch_cmd": round(autopilot.debug.get("pitch_cmd", 0.0), 5),
                    "hdg_err": round(autopilot.debug.get("heading_error", 0.0), 5),
                    "wp": int(autopilot.debug.get("wp_index", 0)),
                    "wp_dist": round(autopilot.debug.get("wp_distance", 0.0), 2),
                    "aileron": round(command["aileron"], 4),
                    "elevator": round(command["elevator"], 4),
                    "rudder": round(command["rudder"], 4),
                    "throttle": round(command["throttle"], 4),
                }) + "\n")

            if state.altitude < 0.5 and state.t > 3.0:
                reason = "the aircraft reached the ground at t=%.2f s" % state.t
                break
            if (autopilot.mission_complete_t is not None
                    and state.t - autopilot.mission_complete_t >= args.loiter_seconds):
                reason = "mission complete, loiter finished"
                break
            if args.max_sim_seconds and state.t >= args.max_sim_seconds:
                reason = "reached --max-sim-seconds"
                break

    except KeyboardInterrupt:
        reason = "interrupted"
    finally:
        if log is not None:
            log.close()
        link.close()

    airborne = state.altitude > 2.0
    report: Dict[str, Any] = {
        "mission": mission_name,
        "mission_file": mission_path,
        "stopped_because": reason,
        "final_state": autopilot.state,
        "waypoints_total": len(waypoints),
        "waypoints_reached": len(nav.reached),
        "waypoints_reached_labels": [label for label, _ in nav.reached],
        "waypoint_reach_times_s": [round(t, 2) for _, t in nav.reached],
        "mission_complete": nav.complete,
        "airborne_at_end": airborne,
        "final_altitude_m": round(state.altitude, 2),
        "final_airspeed_m_s": round(state.airspeed, 2),
        "mavlink_rx_from_sim": dict(link.rx_counts),
        "mavlink_rx_total": sum(link.rx_counts.values()),
        "mavlink_tx_to_sim": link.tx_packets,
        "mavlink_rx_bytes": link.rx_bytes,
        "mavlink_tx_bytes": link.tx_bytes,
        "codec_stats": dict(link.codec.stats),
    }
    report.update(recorder.summary())

    print("", flush=True)
    print("=" * 72, flush=True)
    print("MISSION REPORT -- %s" % mission_name, flush=True)
    print("=" * 72, flush=True)
    print("  stopped            %s" % reason, flush=True)
    print("  final state        %s" % autopilot.state, flush=True)
    print("  waypoints          %d of %d reached: %s"
          % (len(nav.reached), len(waypoints),
             ", ".join(label for label, _ in nav.reached) or "none"), flush=True)
    print("  flight time        %.1f s of simulated time" % report["sim_seconds"], flush=True)
    print("  distance flown     %.1f m" % report["distance_m"], flush=True)
    if report["altitude_error_mean_m"] is not None:
        print("  altitude tracking  mean %.2f m, worst %.2f m (over %d ticks, "
              "commanded steps included)"
              % (report["altitude_error_mean_m"], report["altitude_error_max_m"],
                 report["altitude_tracked_ticks"]), flush=True)
    else:
        print("  altitude tracking  never entered a tracking state", flush=True)
    if report["altitude_settled_error_mean_m"] is not None:
        print("  ... settled        mean %.2f m, worst %.2f m (%d ticks at least "
              "%.0f s after a commanded change)"
              % (report["altitude_settled_error_mean_m"],
                 report["altitude_settled_error_max_m"],
                 report["altitude_settled_ticks"], report["settle_delay_s"]), flush=True)
    print("  airspeed           %.1f to %.1f m/s (mean %.1f), floor is %.1f"
          % (report["airspeed_min_m_s"] or 0.0, report["airspeed_max_m_s"] or 0.0,
             report["airspeed_mean_m_s"], STALL_FLOOR_M_S), flush=True)
    print("  peak bank flown    %.1f deg (the LIMIT is on the command, %.0f deg; "
          "the airframe overshoots it)"
          % (report["bank_max_deg"], math.degrees(BANK_LIMIT_RAD)), flush=True)
    print("  control rate       %d ticks, mean %.2f ms of sim time apart"
          % (report["control_ticks"], report["mean_control_dt_ms"]), flush=True)
    print("  mavlink            rx %d (%s), tx %d"
          % (report["mavlink_rx_total"],
             ", ".join("%s %d" % kv for kv in sorted(link.rx_counts.items())),
             link.tx_packets), flush=True)
    print("  codec              %r" % (link.codec.stats,), flush=True)
    print("  airborne at end    %s (%.1f m)" % (airborne, state.altitude), flush=True)
    print("=" * 72, flush=True)

    if args.summary:
        try:
            with open(args.summary, "w", encoding="utf-8") as handle:
                json.dump(report, handle, indent=2, sort_keys=True)
        except OSError as exc:
            print("[autopilot] could not write summary: %r" % (exc,), file=sys.stderr)

    # Exit code reports the MISSION, not the process: a flight that ended on the
    # ground short of its waypoints is a failed run and a caller should be able
    # to see that without parsing the report.
    return 0 if (nav.complete and airborne) else 1


if __name__ == "__main__":
    sys.exit(main())
