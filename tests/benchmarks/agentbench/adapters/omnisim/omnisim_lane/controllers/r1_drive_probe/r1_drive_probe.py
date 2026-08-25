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

"""DRIVE PROBE -- the minimal reproduction of the defect that blocks R1 here.

**The finding, stated first.** On this tree's engine build, a motor target set
AFTER the Newton world is FINALISED has no effect. Whatever target is in place
at finalize is what the joint does for the rest of the run. It is not specific
to a command type: ``setVelocity``, ``setPosition`` on a range-limited motor,
and ``setTorque`` were each measured and each ignored. It is not specific to a
hand-authored robot: a stock ``URDFRobot`` Husky (``husky.urdf``, plain
revolute wheel joints, no world of ours involved) behaves identically -- it
left its spawn at ~15 m/s uncommanded and then ignored ``setVelocity(4.0)``,
``(0.0)`` and ``(8.0)`` in turn.

⚠ **One tempting piece of evidence is NOT evidence, and is excluded on
purpose.** The engine's own ``tests/api/worlds/motor2_velocity.omniworld`` also
fails on this build, and it looks like a perfect corroboration -- it varies a
commanded velocity every tick. It is a ``Hinge2Joint`` world, and motorised
``Hinge2Joint``/``BallJoint`` not actuating is a SEPARATE, already-documented
gap (AGENTS.md; ``docs/developer/ode-retirement-campaign.md`` lists that world
by name under "hinge2 bodies"). Citing it here would have been two defects
counted as one.

The consequence for AgentBench is not subtle: **no closed-loop controller can
steer anything**, so R1's oracle (perceive, plan, follow) cannot be built on
this arm until it is fixed. That is why this file exists next to an oracle
that does not run.

**What it measures.** Four phases on the R1 rover, one run:

    0.0 - 2.0 s   no command at all
    2.0 - 4.0 s   setVelocity(6.0) on all four wheels
    4.0 - 6.0 s   setVelocity(0.0)
    6.0 - 8.0 s   setVelocity(12.0)

and it records GPS xy and IMU yaw throughout. A working engine gives four
distinct speeds, one of them zero. Measured 2026-08-09 on
``msys64/mingw64/bin/omnisim-bin.exe`` (built 13:43, repo at ``f45e0b652``),
rover starting at (-4, -4) heading +x -- mean speed per commanded value:

    command   None    6.0     0.0     12.0
    m/s       0.997   1.026   1.034   1.041      responds_to_command: False

Four different commands, one speed. The robot drives the whole width of the
arena in a straight line and the ``setVelocity(0.0)`` phase is
indistinguishable from the others.

The uncommanded 1.0 m/s is the motor's own default target: Webots leaves a
velocity-controlled motor at ``maxVelocity`` until a controller says otherwise,
and ``maxVelocity 12`` x the 0.08 m wheel radius is 0.96 m/s. So the ONE value
the run honours is the one that was in place before the first step. Set
``setVelocity(0.0)`` *before* the first ``robot.step()`` instead and the rover
never moves at all, for the whole 60 s, whatever is commanded afterwards --
which is exactly how ``r1_null`` gets a robot that stays parked.

**And the C++ side is not where it is lost.** The engine logs
``[OmNewtonBackend] motor target_vel reached joint N: 6 rad/s (controller ->
OmRotationalMotor -> backend chain verified)`` at t=2.0 in exactly the run
above -- the command crossed the controller IPC boundary and reached
``OmNewtonBackend``, and the robot still did not change speed. So the loss is
downstream of ``setJointTargetVelocity``, in the Newton/MuJoCo control path
(``control.joint_target_vel`` vs what ``SolverMuJoCo`` actually reads), not in
libController and not in the change-detection cache
(``setJointTargetVelocityIfChanged``, whose logic reads correct).

Run it: ``omnisim-bin worlds/r1_drive_probe.wbt --batch --mode=fast
--no-rendering --minimize`` with ``AGENTBENCH_R1_PROBE_OUT`` pointing where you
want the JSON. It reports to a FILE, because ``omnisim-bin.exe`` is a
GUI-subsystem binary on Windows and a controller's stdout goes nowhere.
"""

import json
import os

MOTORS = ("left front motor", "left rear motor",
          "right front motor", "right rear motor")

#: (t_from_s, commanded rad/s). ``None`` means "issue no command at all".
#:
#: The schedule stops at 8 s ON PURPOSE: uncommanded, the rover crosses the
#: arena at ~1.03 m/s and reaches the east wall at about 8.7 s, and a phase
#: that ends against a wall would report a speed change that is the wall's
#: doing rather than the command's. The verdict below truncates at the wall
#: anyway, but a probe should not need its own escape hatch.
PHASES = ((0.0, None), (2.0, 6.0), (4.0, 0.0), (6.0, 12.0))
RUN_S = 8.0

OUT = os.environ.get(
    "AGENTBENCH_R1_PROBE_OUT",
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "drive_probe.json"))


def commanded(t):
    cmd = None
    for t0, v in PHASES:
        if t >= t0:
            cmd = v
    return cmd


def main():
    # A Supervisor purely so the probe can END the run: nothing else quits a
    # world, and a probe that leaves an engine spinning until somebody's
    # timeout fires is a probe that costs five minutes per use. It reads no
    # scene state and writes none.
    from omnisim import Supervisor

    robot = Supervisor()
    dt = int(robot.getBasicTimeStep())
    gps = robot.getDevice("gps")
    gps.enable(dt)
    imu = robot.getDevice("imu")
    imu.enable(dt)
    motors = [robot.getDevice(n) for n in MOTORS]
    for m in motors:
        m.setPosition(float("inf"))

    doc = {"phases": [list(p) for p in PHASES], "dt_ms": dt, "samples": [],
           "columns": ["t_s", "commanded_rad_s", "x_m", "y_m", "yaw_rad",
                       "vx_mps"]}
    prev = None
    k = 0
    while robot.getTime() < RUN_S:
        cmd = commanded(k * dt / 1000.0)
        if cmd is not None:
            for m in motors:
                m.setVelocity(cmd)
        if robot.step(dt) == -1:
            break
        k += 1
        if k % 25:
            continue
        x, y, _z = gps.getValues()
        t = robot.getTime()
        vx = None if prev is None else round((x - prev[1]) / (t - prev[0]), 3)
        doc["samples"].append([round(t, 3), cmd, round(x, 4), round(y, 4),
                               round(imu.getRollPitchYaw()[2], 4), vx])
        prev = (t, x)

    # The verdict, computed rather than left to a reader: did the measured
    # speed EVER change when the command did?
    #
    # Samples from the first stop onwards are DROPPED. A rover that has run
    # into a wall reads 0 m/s whatever it was told, and crediting that as
    # "it obeyed setVelocity(0)" would let the probe pass on the strength of
    # an obstacle -- the same class of mistake as crediting a parked robot
    # with being collision-free.
    live = []
    for s in doc["samples"]:
        if s[5] is not None and abs(s[5]) < 0.05 and live:
            doc["truncated_at_s"] = s[0]
            break
        live.append(s)
    speeds = {}
    for s in live:
        if s[5] is not None:
            speeds.setdefault(s[1], []).append(abs(s[5]))
    doc["mean_speed_by_command"] = {
        str(k): round(sum(v) / len(v), 4) for k, v in speeds.items()}
    doc["responds_to_command"] = len(
        {round(v, 1) for v in doc["mean_speed_by_command"].values()}) > 1
    try:
        with open(OUT, "w", encoding="utf-8") as fh:
            json.dump(doc, fh, indent=2)
    except OSError:
        pass
    # Written FIRST, quit second: a probe whose report depends on a clean
    # shutdown reports nothing when the shutdown is what broke.
    robot.simulationQuit(0)
    robot.step(dt)


if __name__ == "__main__":
    main()
