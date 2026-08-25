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
"""Fly a two-jaw gripper into a hanging cloth, pinch it, and lift.

Drives `newton_vbd_cloth_grasp.omniworld`, the world that exercises
WorldInfo.newtonSolver "vbd" -- one newton SolverVBD owning the gantry, the
jaws and every cloth particle.

WHAT THIS CONTROLLER IS RESPONSIBLE FOR, AND WHAT IT IS NOT. It commands a
scripted trajectory and it records the PADS. It does NOT decide whether the
grasp worked and it deliberately cannot: the fabric is invisible from here.
Cloth particles are not scene nodes, so there is no supervisor accessor and no
HTTP endpoint for them -- the only readback is the engine's own
OMNISIM_CLOTH_TELEMETRY JSONL. The verdict is computed afterwards by joining
that file with this one. A controller that scored its own grasp would be
scoring a thing it never saw.

⚠ THE PADS ARE READ AS SOLID POSES, NOT AS JOINT POSITIONS, AND THAT IS
DELIBERATE. Under newtonSolver "vbd" there is no mj_model, and VBD -- like
every maximal-coordinate solver -- integrates body_q and does not maintain
joint_q. A PositionSensor on these sliders is therefore not trustworthy on this
path. Supervisor pose reads come from body_q, which VBD does maintain, so
getFromDef(...).getPosition() is the honest instrument here. Both are recorded
below so the discrepancy is visible rather than assumed.

RUN
    OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/vbd_grasp_cloth.jsonl \
    OMNISIM_CLOTH_TELEMETRY_FULL=1 OMNISIM_CLOTH_TELEMETRY_EVERY=1 \
    VBD_GRASP_LOG=$PWD/.build_tmp/vbd_grasp_pads.jsonl \
    python -m omnisim run-headless \
      projects/samples/demos/worlds/physics/newton_vbd_cloth_grasp.omniworld --duration 25

    VBD_GRASP_NEGATIVE=1   the REQUIRED negative control: identical flight
                           path, jaws never close. If this also "holds", the
                           measurement is measuring the jaw geometry acting as
                           a ledge, not a pinch.
"""

from __future__ import annotations

import json
import os
import sys

from omnisim import Supervisor

# --------------------------------------------------------------------------
# Geometry and schedule. Every number here is carried over from newton's own
# gripper example via a standalone reproduction of it; the world header records
# which of them are load-bearing and which are inherited unablated.
# --------------------------------------------------------------------------

PARTICLE_RADIUS = 0.005
PAD_HALF_THICK = 0.005          # the jaw box is 10 mm thick along the closing axis
GRIP_COMPRESSION = 0.002        # squeeze this far INSIDE the particle-radius shell
OPEN_EXTRA = 0.045              # open jaws sit this much further out than closed

# Jaw joint coordinate: 0 == open (the authored pad pose), + == closing.
JAW_OPEN = 0.0
JAW_CLOSED = OPEN_EXTRA
# ⚠ Opening to the NORMAL open gap after a hold does not release the sheet: by
# then the fabric has gathered into a bundle ~130-150 mm wide along the jaw
# axis, so the pads re-open INSIDE the bundle and hold it up as ledges with no
# pinch at all. Measured in the standalone reproduction, where it produced a
# false FAIL. Clearing the bundle needs ~180 mm of half-gap, i.e. a jaw
# coordinate of -(0.18 - closed_half_gap).
CLOSED_HALF_GAP = PAD_HALF_THICK + PARTICLE_RADIUS - GRIP_COMPRESSION   # 0.008
JAW_RELEASE = -(0.18 - CLOSED_HALF_GAP - OPEN_EXTRA)                    # -0.127

# Gantry coordinates are DELTAS from the authored start pose (0.30 m to the +X
# side of the grasp point, at grasp height), so every joint starts at 0 and no
# PD has a step to chase at t = 0.
APPROACH_DX = -0.30
LIFT_DZ = 0.25

# Phase durations in seconds, in order.
PHASES = [
    ("settle", 0.60),    # the sheet hangs and stops ringing
    ("approach", 0.80),  # fly in, jaws open
    ("close", 0.50),     # jaws open -> closed
    ("pinch", 0.30),     # hold closed and still
    ("preload", 0.70),   # let the pinch settle before anything moves
    ("lift", 1.20),
    ("hold", 1.50),      # long and static: this is where the verdict is read
    ("release", 0.35),   # jaws open WIDE, clear of the gathered bundle
    ("fall", 1.00),      # ...and the sheet must drop. REPORTED, NOT SCORED --
                         # see the note above; the release outcome measures the
                         # release motion, not the grasp.
]


def smoothstep(a, b, u):
    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
    return a + (b - a) * (u * u * (3.0 - 2.0 * u))


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    negative = os.environ.get("VBD_GRASP_NEGATIVE", "").strip().lower() not in (
        "", "0", "false", "off", "no")
    log_path = os.environ.get("VBD_GRASP_LOG") or os.path.join(
        os.getcwd(), ".build_tmp", "vbd_grasp_pads.jsonl")
    try:
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
    except OSError:
        pass

    motors = {}
    for name in ("gantry_x", "gantry_y", "gantry_z", "jaw_left", "jaw_right"):
        m = robot.getDevice(name)
        if m is None:
            print("vbd_cloth_grasp: FATAL device '%s' missing" % name, flush=True)
            return 1
        motors[name] = m

    pad_l = robot.getFromDef("LEFT_PAD")
    pad_r = robot.getFromDef("RIGHT_PAD")
    if pad_l is None or pad_r is None:
        print("vbd_cloth_grasp: FATAL LEFT_PAD / RIGHT_PAD not resolvable -- the pad "
              "poses are the whole measurement, so refusing to run blind.", flush=True)
        return 1

    # Absolute step index of each phase boundary, so the analysis can address a
    # window without re-deriving the schedule from wall-clock time.
    bounds, acc = [], 0
    for name, secs in PHASES:
        n = int(round(secs / dt))
        bounds.append((name, acc, acc + n))
        acc += n
    total = acc

    print("vbd_cloth_grasp: dt=%d ms, %d steps (%.2f s), negative_control=%s"
          % (dt_ms, total, total * dt, negative), flush=True)
    print("vbd_cloth_grasp: PHASES " + json.dumps(
        [{"phase": n, "start": s, "end": e} for n, s, e in bounds]), flush=True)

    def schedule(step):
        for name, s, e in bounds:
            if step < e:
                u = (step - s) / max(e - s, 1)
                if name == "settle":
                    return name, 0.0, 0.0, JAW_OPEN
                if name == "approach":
                    return name, smoothstep(0.0, APPROACH_DX, u), 0.0, JAW_OPEN
                if name == "close":
                    return name, APPROACH_DX, 0.0, smoothstep(JAW_OPEN, JAW_CLOSED, u)
                if name in ("pinch", "preload"):
                    return name, APPROACH_DX, 0.0, JAW_CLOSED
                if name == "lift":
                    return name, APPROACH_DX, smoothstep(0.0, LIFT_DZ, u), JAW_CLOSED
                if name == "hold":
                    return name, APPROACH_DX, LIFT_DZ, JAW_CLOSED
                if name == "release":
                    return name, APPROACH_DX, LIFT_DZ, smoothstep(JAW_CLOSED, JAW_RELEASE, u)
                return name, APPROACH_DX, LIFT_DZ, JAW_RELEASE
        return "done", APPROACH_DX, LIFT_DZ, JAW_RELEASE

    fh = open(log_path, "w", encoding="utf-8")
    step = 0
    prev_phase = None
    while robot.step(dt_ms) != -1:
        if step >= total:
            break
        phase, dx, dz, jaw = schedule(step)
        if negative:
            # The control differs in EXACTLY one variable: the jaws never close.
            # Same flight path, same timing, same everything else.
            jaw = JAW_OPEN if phase != "release" and phase != "fall" else JAW_RELEASE

        motors["gantry_x"].setPosition(dx)
        motors["gantry_y"].setPosition(0.0)
        motors["gantry_z"].setPosition(dz)
        motors["jaw_left"].setPosition(jaw)
        motors["jaw_right"].setPosition(jaw)

        pl = pad_l.getPosition()
        pr = pad_r.getPosition()
        rec = {
            "step": step,
            "t": round(robot.getTime(), 6),
            "phase": phase,
            "cmd": [round(dx, 6), 0.0, round(dz, 6), round(jaw, 6)],
            "pad_l": [round(v, 6) for v in pl],
            "pad_r": [round(v, 6) for v in pr],
            # MEASURED midpoint of the two jaw bodies, never the commanded pose.
            "pad_c": [round(0.5 * (pl[i] + pr[i]), 6) for i in range(3)],
            # MEASURED jaw separation along the closing axis. The commanded
            # separation is 2 * CLOSED_HALF_GAP = 16 mm; the fabric pushes the
            # PD-driven jaws apart from it, and the difference is the real shell
            # compression -- the standalone reproduction measured 18.66 mm, i.e.
            # 0.67 mm per side rather than the 2 mm commanded.
            "gap": round(pr[1] - pl[1], 6),
        }
        fh.write(json.dumps(rec) + "\n")

        if phase != prev_phase:
            print("vbd_cloth_grasp: MARK %-9s step=%d t=%.3f pad_c=(%.5f, %.5f, %.5f) gap=%.5f"
                  % (phase, step, rec["t"], rec["pad_c"][0], rec["pad_c"][1],
                     rec["pad_c"][2], rec["gap"]), flush=True)
            prev_phase = phase
        step += 1

    fh.close()
    print("vbd_cloth_grasp: wrote %d pad samples to %s" % (step, log_path), flush=True)
    print("vbd_cloth_grasp: DONE (this controller does not score the grasp -- join "
          "this file with the OMNISIM_CLOTH_TELEMETRY JSONL to get the verdict)",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
