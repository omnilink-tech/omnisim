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

"""Standalone CUBE-THROWER supervisor.

A bodyless Supervisor whose only job is to throw DEF CUBE0..n at a target robot
(DEF ROBOT), one at a time, on an arc from where each cube is parked -- the
validated mechanic: setVelocity on the resting cube (NO teleport, which froze in
the warp state), with an exact projectile solve so the cube passes through the
body centre at torso height = a solid hit. Decoupled from the robot controller,
so it works with ANY robot (deterministic stand OR the RL residual deploy).

Env:
    THROW_SPEED   horizontal hit speed, m/s (default 4.0)
    THROW_PERIOD  seconds between throws (default 3.0)
    THROW_Z       aim height above the base (default 0.12 = lower torso)
    THROW_START   wait this long before the first throw, s (default 2.5)
"""
import math
import os
import sys

try:
    from omnisim import Supervisor
except Exception:
    from omnisim import Supervisor


def main() -> int:
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())

    target = robot.getFromDef("ROBOT")
    cubes = []
    i = 0
    while True:
        c = robot.getFromDef(f"CUBE{i}")
        if c is None:
            break
        cubes.append(c)
        i += 1
    sys.stderr.write(f"[thrower] target={'ok' if target else 'NONE'} cubes={len(cubes)}\n")
    sys.stderr.flush()

    speed = float(os.environ.get("THROW_SPEED", "4.0"))
    period_ms = float(os.environ.get("THROW_PERIOD", "3.0")) * 1000.0
    aim_z = float(os.environ.get("THROW_Z", "0.12"))
    start_ms = float(os.environ.get("THROW_START", "2.5")) * 1000.0
    # THREAT CHANNEL (optional): when G1_THREAT_CHANNEL is set, publish a one-shot JSON
    # per throw so a defending robot can ORIENT-AND-BAR toward the incoming cube. The
    # thrower is the perfect oracle (it computes the bearing + exact flight time). The
    # robot consumes RELATIVE ttf, so no cross-process clock sync is needed.
    channel = os.environ.get("G1_THREAT_CHANNEL")
    reach_max = float(os.environ.get("G1_THREAT_REACH_MAX", "1.2"))  # |bearing| reachable by waist yaw

    sim_ms = 0.0
    throw_idx = 0
    last_ms = start_ms - period_ms     # first throw at THROW_START
    launch_cube = None
    launch_vel = [0.0] * 6
    launch_left = 0

    while robot.step(step_ms) != -1:
        sim_ms += step_ms
        if target is None or not cubes:
            continue
        tp = target.getPosition() or [0.0, 0.0, 0.0]
        bx, by, bz = float(tp[0]), float(tp[1]), float(tp[2])

        if (throw_idx < len(cubes) and launch_left <= 0
                and sim_ms - last_ms >= period_ms):
            cube = cubes[throw_idx]
            try:
                cp = cube.getPosition() or [0.0, 0.0, 0.0]
                tx, ty, tz = bx, by, bz + aim_z
                ddx, ddy, ddz = tx - cp[0], ty - cp[1], tz - cp[2]
                dh = math.hypot(ddx, ddy)
                if dh > 1e-3:
                    ux, uy = ddx / dh, ddy / dh
                    T = dh / max(speed, 0.5)
                    vz = (ddz + 0.5 * 9.81 * T * T) / T
                    launch_vel = [ux * speed, uy * speed, vz, 0.0, 0.0, 0.0]
                    cube.setVelocity(launch_vel)
                    launch_cube = cube
                    launch_left = 3
                    sys.stderr.write(
                        f"[thrower] THROW #{throw_idx} t={sim_ms/1000:.1f}s "
                        f"from=({cp[0]:+.1f},{cp[1]:+.1f}) speed={speed:.1f}\n")
                    sys.stderr.flush()
                    if channel:
                        # bearing the robot must FACE to meet the cube = direction
                        # robot->cube (world == robot frame; robot is unyawed at rest).
                        theta = math.atan2(cp[1] - by, cp[0] - bx)
                        try:
                            import json
                            payload = {
                                "throw_idx": throw_idx,
                                "theta": theta,
                                "ttf_ms": T * 1000.0,            # horizontal flight time
                                "bearing_reachable": abs(theta) < reach_max,
                            }
                            tmp = channel + ".tmp"
                            with open(tmp, "w") as cf:
                                json.dump(payload, cf)
                            os.replace(tmp, channel)
                        except Exception as ce:
                            sys.stderr.write(f"[thrower] threat write failed: {ce}\n")
            except Exception as e:
                sys.stderr.write(f"[thrower] throw failed: {e}\n")
            throw_idx += 1
            last_ms = sim_ms

        if launch_left > 0 and launch_cube is not None:
            try:
                launch_cube.setVelocity(launch_vel)
            except Exception:
                pass
            launch_left -= 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
