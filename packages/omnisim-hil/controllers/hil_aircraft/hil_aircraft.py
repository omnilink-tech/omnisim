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

"""Flight dynamics and the MAVLink hardware-in-the-loop link for one aircraft.

This controller is what makes a fixed-wing airframe fly in OmniSim, which has
no aerodynamics of its own. Each tick it:

1. reads the aircraft's true state from the supervisor,
2. computes aerodynamic and propulsive forces with :mod:`omnisim_hil.aero`,
3. applies them through ``addForceWithOffset`` -- every tick, because Newton
   consumes its external-wrench dict at the end of each step,
4. synthesises the sensor stream an autopilot expects and sends it as MAVLink
   ``HIL_SENSOR`` / ``HIL_GPS`` / ``HIL_STATE_QUATERNION`` over UDP,
5. reads back ``HIL_ACTUATOR_CONTROLS`` and applies it to the control surfaces
   and throttle.

The flight software is a SEPARATE PROCESS. That is the whole point: what is
under test is the real autopilot binary talking over the real protocol, not a
control law living inside the simulator. ``--internal-autopilot`` exists only to
exercise the flight model without a link, and it says so on every run.

Threading: none. The socket is non-blocking and pumped inline on the sim thread.
The OmniSim controller API is not thread-safe, and a device read from a worker
thread stalls the whole controller-to-engine step exchange, so the lockstep
shape used by the RL agents in this tree is the right one here.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from omnisim import Supervisor

from omnisim_hil import atmosphere as atm
from omnisim_hil.aero import delivery_aircraft
from omnisim_hil.frames import enu_to_ned, flu_to_frd, local_to_global, quaternion_frd_ned
from omnisim_hil.mavlink import MavlinkCodec
from omnisim_hil.vec3 import add, clamp, cross, mat_t_vec, norm, scale, sub

GRAVITY = 9.80665
ORIGIN_LAT = 47.397742
ORIGIN_LON = 8.545594

# MAVLink HIL_SENSOR fields_updated bitmask: accel(0-2), gyro(3-5), mag(6-8),
# abs pressure(9), diff pressure(10), pressure alt(11), temperature(12).
FIELDS_UPDATED_ALL = 0x1FFF


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument("--mavlink-host", default="127.0.0.1")
    p.add_argument("--mavlink-port", type=int, default=14560)
    p.add_argument("--launch-speed", type=float, default=17.0)
    p.add_argument("--payload-kg", type=float, default=0.0)
    p.add_argument("--wind", type=float, default=0.0, help="steady wind, m/s from the east")
    p.add_argument("--turbulence", type=float, default=0.0, help="gust sigma, m/s")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--internal-autopilot", action="store_true")
    p.add_argument("--per-surface-forces", action="store_true",
                   help="apply one wrench per lifting surface instead of one lumped "
                        "wrench; physically equivalent, ~4x the supervisor traffic")
    p.add_argument("--target-altitude", type=float, default=45.0)
    p.add_argument("--telemetry", default="", help="write a JSONL flight log here")
    p.add_argument("--max-seconds", type=float, default=0.0)
    args, _ = p.parse_known_args()

    # Environment overrides, so one committed world can serve a HIL run, a
    # flight-model run and a weather sweep without being edited each time. The
    # world file stays the record of the AIRCRAFT; the run conditions are the
    # caller's business.
    def _env(name, cast, current):
        raw = os.environ.get(name)
        if raw is None or raw == "":
            return current
        try:
            return cast(raw)
        except (TypeError, ValueError):
            print("[hil_aircraft] ignoring malformed %s=%r" % (name, raw), file=sys.stderr)
            return current

    args.internal_autopilot = _env(
        "HIL_INTERNAL_AUTOPILOT", lambda v: v not in ("0", "false", "False"),
        args.internal_autopilot)
    args.mavlink_port = _env("HIL_MAVLINK_PORT", int, args.mavlink_port)
    args.launch_speed = _env("HIL_LAUNCH_SPEED", float, args.launch_speed)
    args.payload_kg = _env("HIL_PAYLOAD_KG", float, args.payload_kg)
    args.wind = _env("HIL_WIND", float, args.wind)
    args.turbulence = _env("HIL_TURBULENCE", float, args.turbulence)
    args.target_altitude = _env("HIL_TARGET_ALTITUDE", float, args.target_altitude)
    args.telemetry = _env("HIL_TELEMETRY", str, args.telemetry)
    args.max_seconds = _env("HIL_MAX_SECONDS", float, args.max_seconds)
    args.seed = _env("HIL_SEED", int, args.seed)
    return args


class InternalAutopilot:
    """A minimal stabiliser, for exercising the flight model without a link.

    Deliberately simple and deliberately NOT the deliverable. It holds altitude
    and wings level with cascaded proportional loops. Anything it can fly, a
    real autopilot can fly; the reverse is not implied.
    """

    def __init__(self, target_altitude: float) -> None:
        self.target_altitude = target_altitude

    def update(self, altitude, roll, pitch, yaw, airspeed, climb_rate, roll_rate, pitch_rate):
        altitude_error = self.target_altitude - altitude
        # Altitude -> desired climb rate -> desired pitch. Cascaded so each
        # stage can be limited independently; an unlimited outer loop is how a
        # simple hold ends up commanding a stall.
        climb_target = clamp(0.35 * altitude_error, -4.0, 4.0)
        pitch_target = clamp(0.12 * (climb_target - climb_rate), -0.30, 0.30)
        elevator = clamp(-1.8 * (pitch_target - pitch) - 0.35 * pitch_rate, -1.0, 1.0)
        aileron = clamp(-1.6 * roll - 0.30 * roll_rate, -1.0, 1.0)
        rudder = 0.0
        throttle = clamp(0.55 + 0.06 * (16.0 - airspeed) + 0.05 * altitude_error, 0.0, 1.0)
        return {"aileron": aileron, "elevator": elevator, "rudder": rudder}, throttle


def main() -> int:
    args = parse_args()
    robot = Supervisor()
    node = robot.getSelf()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    craft = delivery_aircraft(payload_kg=args.payload_kg)

    # THE SIMULATOR OWNS THE MASS. Reading it back rather than trusting the
    # model's own number removes a physics constant that was declared twice --
    # the world's Physics node and the flight model -- and could therefore
    # disagree. It did: --payload-kg raised the model's mass while the Newton
    # body stayed at the world's value, so a 1 kg parcel produced a
    # byte-identical flight. Nothing in Airframe.forces() reads mass, so the
    # divergence was invisible until a payload sweep produced two identical rows.
    sim_mass = _read_simulated_mass(node)
    if sim_mass is not None:
        if abs(sim_mass - craft.mass_kg) > 1e-6:
            print("[hil_aircraft] mass: model %.3f kg -> using the world's %.3f kg. "
                  "A payload only changes the FLIGHT if the world declares it: "
                  "runtime mass writes do not reach the frozen solver model, so "
                  "author a world whose Physics.mass includes the parcel."
                  % (craft.mass_kg, sim_mass))
        craft.mass_kg = sim_mass
    else:
        print("[hil_aircraft] could not read Physics.mass from the world; "
              "flight-model mass %.3f kg is unverified against the simulator."
              % craft.mass_kg)
    wind = atm.Wind(
        steady=(args.wind, 0.0, 0.0),
        turbulence_sigma_m_s=args.turbulence,
        seed=args.seed,
    )
    air = atm.Atmosphere()

    prop_motor = robot.getDevice("prop_motor")
    if prop_motor is not None:
        prop_motor.setPosition(float("inf"))
        prop_motor.setVelocity(0.0)

    # The sim's own sensors are enabled and read as a CROSS-CHECK, not as the
    # source of the MAVLink stream. AGENTS.md records that Gyro and
    # Accelerometer read a constant zero on carriers that are not Newton
    # bodies, so a rig that silently depended on them would report a level,
    # motionless aircraft while it spiralled into the ground.
    sensors = {}
    for name in ("imu", "gyro", "accelerometer", "gps"):
        dev = robot.getDevice(name)
        if dev is not None:
            dev.enable(dt_ms)
        sensors[name] = dev

    codec = MavlinkCodec(sysid=1, compid=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    try:
        sock.bind((args.mavlink_host, 0))
    except OSError as exc:
        print("[hil_aircraft] could not bind UDP socket: %r" % (exc,), file=sys.stderr)
    peer = (args.mavlink_host, args.mavlink_port)

    internal = InternalAutopilot(args.target_altitude) if args.internal_autopilot else None
    if internal is not None:
        print("[hil_aircraft] INTERNAL autopilot: the flight model is under test, "
              "not any external flight software.")
    else:
        print("[hil_aircraft] waiting for an external autopilot on %s:%d "
              "(MAVLink HIL). Until one connects the aircraft glides with "
              "neutral controls -- it is not being flown." % peer)

    # Hand launch: released at cruise speed, nose level. Doing this through the
    # supervisor rather than a catapult keeps the world free of scenery whose
    # only job is to get the aircraft moving.
    node.setVelocity([args.launch_speed, 0.0, 0.0, 0.0, 0.0, 0.0])

    telemetry = None
    if args.telemetry:
        try:
            telemetry = open(args.telemetry, "w", encoding="utf-8")
        except OSError as exc:
            print("[hil_aircraft] telemetry disabled: %r" % (exc,), file=sys.stderr)

    controls = {"aileron": 0.0, "elevator": 0.0, "rudder": 0.0}
    throttle = 0.0
    prev_velocity = (args.launch_speed, 0.0, 0.0)
    last_actuator_time = None
    heartbeat_at = 0.0
    gps_at = 0.0
    sim_time = 0.0
    steps = 0
    boot_wall = time.time()
    peak_altitude = 0.0
    crashed = False

    while robot.step(dt_ms) != -1:
        steps += 1
        sim_time = robot.getTime()

        position = node.getPosition()
        orientation = tuple(node.getOrientation())
        velocity6 = node.getVelocity()
        v_world = (velocity6[0], velocity6[1], velocity6[2])
        omega_world = (velocity6[3], velocity6[4], velocity6[5])

        altitude = position[2]
        peak_altitude = max(peak_altitude, altitude)

        gust = wind.sample(dt, max(norm(v_world), 1.0))
        v_air_world = sub(v_world, gust)
        v_air_body = mat_t_vec(orientation, v_air_world)
        omega_body = mat_t_vec(orientation, omega_world)
        rho = air.density(max(altitude, 0.0))
        airspeed = norm(v_air_body)

        throttle_rps = throttle * craft.propeller.max_rev_per_s if craft.propeller else 0.0
        applied = craft.forces(v_air_body, omega_body, rho, controls, throttle_rps)

        # EVERY tick. Newton clears body_f each substep and drops the pending
        # wrench dict at the end of the step, so a force applied once is a force
        # applied for one step only.
        #
        # LUMPED: sum the surface forces into one force at the centre of mass
        # plus one couple. Exactly equivalent for a rigid body -- the moment of
        # a force at an offset IS r x F -- and it cuts the supervisor round
        # trips per aircraft per tick from eight to two, which is what makes a
        # whole fleet in one world affordable. --per-surface-forces keeps the
        # original path so the equivalence can be A/B'd rather than asserted.
        if args.per_surface_forces:
            for force, offset in applied:
                node.addForceWithOffset(list(force), list(offset), True)
        elif applied:
            fx = fy = fz = 0.0
            tx = ty = tz = 0.0
            for force, offset in applied:
                fx += force[0]
                fy += force[1]
                fz += force[2]
                mx, my, mz = cross(offset, force)
                tx += mx
                ty += my
                tz += mz
            node.addForce([fx, fy, fz], True)
            if abs(tx) + abs(ty) + abs(tz) > 1e-12:
                node.addTorque([tx, ty, tz], True)

        # Proper acceleration: what an accelerometer actually measures, which is
        # the non-gravitational part. Differencing the world velocity and
        # removing gravity is correct in flight AND on the ground, where the
        # normal force must show up as +1g; deriving it from the aero forces
        # alone would read zero while the aircraft sat on the runway.
        accel_world = scale(sub(v_world, prev_velocity), 1.0 / dt)
        proper_world = add(accel_world, (0.0, 0.0, GRAVITY))
        proper_body = mat_t_vec(orientation, proper_world)
        prev_velocity = v_world

        roll, pitch, yaw = _euler(orientation)
        climb_rate = v_world[2]

        if internal is not None:
            controls, throttle = internal.update(
                altitude, roll, pitch, yaw, airspeed, climb_rate,
                omega_body[0], omega_body[1],
            )

        now_us = int(sim_time * 1e6)
        accel_frd = flu_to_frd(proper_body)
        gyro_frd = flu_to_frd(omega_body)
        pressure = air.pressure_pa(max(altitude, 0.0))

        packet = codec.encode(
            "HIL_SENSOR",
            time_usec=now_us,
            xacc=accel_frd[0], yacc=accel_frd[1], zacc=accel_frd[2],
            xgyro=gyro_frd[0], ygyro=gyro_frd[1], zgyro=gyro_frd[2],
            xmag=0.21, ymag=0.0, zmag=0.43,
            abs_pressure=pressure / 100.0,
            diff_pressure=0.5 * rho * airspeed * airspeed / 100.0,
            pressure_alt=atm.pressure_altitude_m(pressure),
            temperature=air.temperature_k(max(altitude, 0.0)) - 273.15,
            fields_updated=FIELDS_UPDATED_ALL,
        )
        _send(sock, packet, peer)

        quat = quaternion_frd_ned(orientation)
        v_ned = enu_to_ned(v_world)
        _send(sock, codec.encode(
            "HIL_STATE_QUATERNION",
            time_usec=now_us,
            attitude_quaternion=list(quat),
            rollspeed=gyro_frd[0], pitchspeed=gyro_frd[1], yawspeed=gyro_frd[2],
            lat=int(local_to_global(position[0], position[1], altitude,
                                    ORIGIN_LAT, ORIGIN_LON)[0] * 1e7),
            lon=int(local_to_global(position[0], position[1], altitude,
                                    ORIGIN_LAT, ORIGIN_LON)[1] * 1e7),
            alt=int(altitude * 1000.0),
            vx=int(v_ned[0] * 100.0), vy=int(v_ned[1] * 100.0), vz=int(v_ned[2] * 100.0),
            ind_airspeed=int(airspeed * 100.0),
            true_airspeed=int(airspeed * 100.0),
            xacc=int(accel_frd[0] * 1000.0),
            yacc=int(accel_frd[1] * 1000.0),
            zacc=int(accel_frd[2] * 1000.0),
        ), peer)

        if sim_time - gps_at >= 0.1:
            gps_at = sim_time
            lat, lon, alt = local_to_global(position[0], position[1], altitude,
                                            ORIGIN_LAT, ORIGIN_LON)
            ground_speed = math.hypot(v_world[0], v_world[1])
            _send(sock, codec.encode(
                "HIL_GPS",
                time_usec=now_us,
                fix_type=3,
                lat=int(lat * 1e7), lon=int(lon * 1e7), alt=int(alt * 1000.0),
                eph=80, epv=120,
                vel=int(ground_speed * 100.0),
                vn=int(v_ned[0] * 100.0), ve=int(v_ned[1] * 100.0), vd=int(v_ned[2] * 100.0),
                cog=int((math.degrees(math.atan2(v_ned[1], v_ned[0])) % 360.0) * 100.0),
                satellites_visible=14,
            ), peer)

        if sim_time - heartbeat_at >= 1.0:
            heartbeat_at = sim_time
            _send(sock, codec.encode(
                "HEARTBEAT", type=1, autopilot=0, base_mode=128,
                system_status=4, mavlink_version=3,
            ), peer)

        for msg in _drain(sock, codec):
            if msg.name == "HIL_ACTUATOR_CONTROLS":
                raw = msg["controls"]
                # PX4's fixed-wing HIL channel order.
                controls = {
                    "aileron": clamp(float(raw[0]), -1.0, 1.0),
                    "elevator": clamp(float(raw[1]), -1.0, 1.0),
                    "rudder": clamp(float(raw[2]), -1.0, 1.0),
                }
                throttle = clamp(float(raw[3]), 0.0, 1.0)
                last_actuator_time = sim_time

        if prop_motor is not None:
            prop_motor.setVelocity(clamp(throttle * 1200.0, 0.0, 1400.0))

        if telemetry is not None and steps % 4 == 0:
            telemetry.write(
                '{"t":%.3f,"x":%.3f,"y":%.3f,"z":%.3f,"roll":%.4f,"pitch":%.4f,'
                '"yaw":%.4f,"airspeed":%.3f,"climb":%.3f,"throttle":%.3f,'
                '"elevator":%.3f,"aileron":%.3f,"alpha":%.4f}\n'
                % (sim_time, position[0], position[1], altitude, roll, pitch, yaw,
                   airspeed, climb_rate, throttle, controls["elevator"],
                   controls["aileron"],
                   math.atan2(-v_air_body[2], max(v_air_body[0], 1e-6)))
            )

        if altitude < 0.3 and sim_time > 2.0:
            crashed = True
            print("[hil_aircraft] ground contact at t=%.2f s, x=%.1f m, "
                  "airspeed=%.1f m/s" % (sim_time, position[0], airspeed))
            break

        if args.max_seconds and sim_time >= args.max_seconds:
            break

    link = "none"
    if last_actuator_time is not None:
        link = "external autopilot, last command at t=%.2f s" % last_actuator_time
    elif internal is not None:
        link = "internal autopilot (flight model under test)"

    print("[hil_aircraft] flew %.2f s of sim time in %.2f s wall, %d steps"
          % (sim_time, time.time() - boot_wall, steps))
    print("[hil_aircraft] control link: %s" % link)
    print("[hil_aircraft] peak altitude %.2f m, ended %s"
          % (peak_altitude, "on the ground" if crashed else "airborne"))
    print("[hil_aircraft] mavlink rx: %r" % (codec.stats,))

    if telemetry is not None:
        telemetry.close()
    return 0


def _read_simulated_mass(node):
    """The mass Newton is actually integrating, from the world's Physics node.

    Returns None rather than a guess when the field cannot be read: a fabricated
    mass would make every acceleration the flight model predicts quietly wrong,
    and wrong by a factor nobody would think to check.
    """
    try:
        physics = node.getField("physics").getSFNode()
        if physics is None:
            return None
        mass = physics.getField("mass").getSFFloat()
        return mass if mass and mass > 0.0 else None
    except Exception:
        return None


def _euler(m):
    """(roll, pitch, yaw) from an OmniSim row-major orientation matrix, FLU/ENU."""
    pitch = math.asin(clamp(-m[6], -1.0, 1.0))
    if abs(m[6]) > 0.999999:
        return (0.0, pitch, math.atan2(-m[1], m[4]))
    return (math.atan2(m[7], m[8]), pitch, math.atan2(m[3], m[0]))


def _send(sock, data, peer) -> None:
    try:
        sock.sendto(data, peer)
    except OSError:
        # A HIL link with nothing listening is a normal startup state, not an
        # error worth spamming a log with. The summary line reports whether an
        # autopilot ever spoke.
        pass


def _drain(sock, codec, budget: int = 64):
    out = []
    for _ in range(budget):
        try:
            data, _addr = sock.recvfrom(4096)
        except (BlockingIOError, OSError):
            break
        if not data:
            break
        out.extend(codec.feed(data))
    return out


if __name__ == "__main__":
    sys.exit(main())
