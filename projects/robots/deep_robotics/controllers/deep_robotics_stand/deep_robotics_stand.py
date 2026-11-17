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

"""deep_robotics_stand -- hold a Deep Robotics robot in its URDF rest pose.

On its first tick it reads each motor's target (the rest pose) and its
sensor (the pose the engine spawned the joint in), then drives every motor
along a smoothstep from spawn to rest over RAMP_S seconds and holds there.

Since 2026-09-08 the engine starts every joint AT its authored position
(OmBasicJoint::flushPendingNewtonRegistrations, hatch
OMNISIM_NEWTON_SPAWN_AT_POSITION=0), so spawn == rest and the ramp is a
no-op: the controller is a hold plus the trace below. It earns its keep when
the hatch is off, where the joints start at q = 0 (clamped into range) and
only the motor TARGETS carry the rest pose -- with no controller the servos
then yank all twelve leg joints from straight to crouched in ~0.2 s while the
body is still at spawn height, the feet punch through the floor, and the
robot lands on its thighs or its back (the Lite3 trace that found this is in
projects/robots/deep_robotics/PROVENANCE.md).

Works for any robot: a joint whose rest equals its spawn value is simply
held.

Optional trace (for validation): set OMNISIM_PROBE_OUT=<file> and the
controller writes one line per step -- body position and up-vector, every
joint's measured angle and target, and the engine's contact points for the
whole robot -- for OMNISIM_PROBE_STEPS steps (default 500). The robot must be
a `supervisor TRUE` node for the body pose and contacts to be readable; the
ramp itself needs nothing.
"""
import math
import os
import sys

from omnisim import Node, Supervisor

RAMP_S = float(os.environ.get("OMNISIM_STAND_RAMP_S", "1.5"))


def _smoothstep(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def main():
    robot = Supervisor()
    ts = int(robot.getBasicTimeStep())

    motors = []
    for i in range(robot.getNumberOfDevices()):
        d = robot.getDeviceByIndex(i)
        if d.getNodeType() in (Node.ROTATIONAL_MOTOR, Node.LINEAR_MOTOR):
            motors.append(d)
    motors.sort(key=lambda m: m.getName())
    sensors = []
    for m in motors:
        s = m.getPositionSensor()
        if s is not None:
            s.enable(ts)
        sensors.append(s)

    trace_path = os.environ.get("OMNISIM_PROBE_OUT")
    trace_steps = int(os.environ.get("OMNISIM_PROBE_STEPS", "500"))
    trace = open(trace_path, "w", encoding="utf-8") if trace_path else None
    me = robot.getSelf() if trace else None
    if trace:
        trace.write("# motors: " + " ".join(m.getName() for m in motors) + "\n")

    # First tick: the engine has assigned the rest pose to the motor targets
    # and the spawn pose to the joints. Capture both before touching anything.
    if robot.step(ts) == -1:
        return
    rest = [m.getTargetPosition() for m in motors]
    start = [(s.getValue() if s is not None else r) for s, r in zip(sensors, rest)]
    for k, (st, r) in enumerate(zip(start, rest)):
        if not math.isfinite(st):
            start[k] = r
    t0 = robot.getTime()
    print("[deep_robotics_stand] %d motors; ramping spawn -> rest over %.2f s" % (len(motors), RAMP_S),
          file=sys.stderr)

    k = 0
    while True:
        t = robot.getTime() - t0
        a = _smoothstep(t / RAMP_S) if RAMP_S > 0 else 1.0
        for m, st, r in zip(motors, start, rest):
            m.setPosition(st + (r - st) * a)
        if trace and k < trace_steps:
            p = me.getPosition()
            o = me.getOrientation()
            q = [(s.getValue() if s is not None else float("nan")) for s in sensors]
            tg = [m.getTargetPosition() for m in motors]
            try:
                cps = me.getContactPoints(True)
                npts = len(cps)
                pts = " ".join("(%.3f,%.3f,%.3f)" % (c.point[0], c.point[1], c.point[2]) for c in cps[:8])
            except Exception as exc:
                npts, pts = -1, repr(exc)[:60]
            trace.write("t=%.3f pos=(%.3f,%.3f,%.3f) up=(%.2f,%.2f,%.2f) q=[%s] tg=[%s] contacts=%d %s\n" % (
                robot.getTime(), p[0], p[1], p[2], o[2], o[5], o[8],
                " ".join("%.3f" % v for v in q), " ".join("%.3f" % v for v in tg), npts, pts))
            trace.flush()
        k += 1
        if robot.step(ts) == -1:
            break
    if trace:
        trace.close()


if __name__ == "__main__":
    main()
