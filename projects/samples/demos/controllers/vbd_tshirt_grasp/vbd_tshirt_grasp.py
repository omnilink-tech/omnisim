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
"""Fly a two-jaw gripper to a chosen point on a hanging T-SHIRT, pinch it, lift.

Drives `newton_vbd_tshirt_grasp.omniworld`. It is the controller of
`newton_vbd_cloth_grasp` with one thing added: the grasp point is a PARAMETER
rather than a constant, because a garment has several places worth grabbing and
comparing them must not require editing the world (an edited world confounds the
comparison it was meant to serve). The gantry's three world-axis DoF can reach
any of them, so the world stays byte-identical across every run below.

WHAT THIS CONTROLLER IS RESPONSIBLE FOR, AND WHAT IT IS NOT. It commands a
scripted trajectory and it records the PADS. It does NOT decide whether the
grasp worked and it deliberately cannot: the fabric is invisible from here.
Cloth particles are not scene nodes, so there is no supervisor accessor and no
HTTP endpoint -- the only readback is the engine's own OMNISIM_CLOTH_TELEMETRY
JSONL. The verdict is computed afterwards by joining that file with this one. A
controller that scored its own grasp would be scoring a thing it never saw.

⚠ THE PADS ARE READ AS SOLID POSES, NOT AS JOINT POSITIONS, AND THAT IS
DELIBERATE. Under newtonSolver "vbd" there is no mj_model, and VBD -- like every
maximal-coordinate solver -- integrates body_q and does not maintain joint_q. A
PositionSensor on these sliders is not trustworthy on this path; supervisor pose
reads come from body_q, which VBD does maintain.

MODES (TSHIRT_MODE)
    grasp     (default) fly in, close, lift, hold, release
    negative  THE REQUIRED CONTROL: identical flight path, jaws never close. If
              this also "holds", the measurement is measuring the jaw geometry
              acting as a ledge rather than a pinch.
    settle    park the gantry clear and just let the garment hang. Used to
              MEASURE where the settled panels actually are before choosing a
              grasp point -- the rest mesh is authored as worn and the hanging
              shape is not it.

GRASP POINT
    TSHIRT_GRASP_PRESET=front_low|hem_edge|sleeve   (default front_low)
    TSHIRT_GX / TSHIRT_GY / TSHIRT_GZ               override any component (m,
                                                    world frame)
    TSHIRT_LIFT                                     lift height (m, default 0.25)

RUN
    OMNISIM_CLOTH_SELF_CONTACT=0 \
    OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/tshirt_cloth.jsonl \
    OMNISIM_CLOTH_TELEMETRY_FULL=1 OMNISIM_CLOTH_TELEMETRY_EVERY=1 \
    TSHIRT_GRASP_LOG=$PWD/.build_tmp/tshirt_pads.jsonl \
    python -m omnisim run-headless \
      projects/samples/demos/worlds/physics/newton_vbd_tshirt_grasp.omniworld \
      --duration 45
"""

from __future__ import annotations

import json
import os
import sys

from omnisim import Supervisor

# --------------------------------------------------------------------------
# Pinch geometry. Every number is derived from the world's own dimensions; none
# of it is tuned. See the world header for why maxForce is a stiffness.
# --------------------------------------------------------------------------

PARTICLE_RADIUS = 0.010         # Cloth.particleRadius
PAD_HALF_THICK = 0.005          # the jaw box is 10 mm thick along the closing axis
PAD_REST_OFFSET = 0.070         # authored |y| of each pad -> 140 mm open gap
GRIP_COMPRESSION = 0.002        # squeeze this far INSIDE the particle shell.
                                # newton's guidance is ~0.2 x particle radius.

# Jaw joint coordinate: 0 == open (the authored pad pose), + == closing.
CLOSED_HALF_GAP = PAD_HALF_THICK + PARTICLE_RADIUS - GRIP_COMPRESSION    # 0.013
JAW_OPEN = 0.0
JAW_CLOSED = PAD_REST_OFFSET - CLOSED_HALF_GAP                           # 0.057
# ⚠ Opening to the NORMAL open gap after a hold does not release the fabric: by
# then it has gathered into a bundle, so the pads re-open INSIDE the bundle and
# hold it up as ledges with no pinch at all. Measured on the patch world, where
# it produced a false FAIL. Clearing it needs ~0.20 m of half-gap.
RELEASE_HALF_GAP = 0.20
JAW_RELEASE = PAD_REST_OFFSET - RELEASE_HALF_GAP                         # -0.13

# ⚠ Multi-layer grabs (the sleeve preset gathers both walls of the cuff) need NO
# extra gap, and the reason is worth stating because it is not obvious: this
# world runs with particle self-contact OFF, so two fabric layers do not push
# each other apart and both relax into the same pinch plane. Self-contact is
# also the single biggest slip term measured on the patch world (24x), which is
# why it is off in the first place.

# Where the gantry's Robot root is authored, so a joint coordinate is a DELTA
# from it and every joint starts at 0 with no step for a PD to chase at t = 0.
GANTRY_ROOT = (0.75, 0.0, 0.70)

# The shirt's own origin in the world, so a preset can be written in the MESH's
# frame (which is what the asset measurements are in) and converted here.
SHIRT_ORIGIN = (0.0, 0.0, 0.62)

# Grasp presets, in MESH-LOCAL coordinates (x across the shoulders, y front/back,
# z up from the hem).
#
# ⚠ EVERY y HERE IS MEASURED ON THE SETTLED GARMENT, NOT READ OFF THE REST MESH,
# and that distinction is the whole reason TSHIRT_MODE=settle exists. The asset
# is authored as worn; where its panels end up once it is hanging is a physics
# question. (It turned out to barely move -- a closed tube pinned along its
# shoulders holds its cross-section, and the centroid dropped 4.5 mm and stopped
# -- but that was worth one 30 s run to establish rather than assume, because a
# wrong y produces a clean miss that looks exactly like a failed pinch.)
#
# Measured on the settle run, last telemetry record, particles inside the
# 90 x 60 mm pad footprint at each target:
#
#   preset      particles in footprint      front-panel y      panel y-spread
#   front_low   11 (6 front / 5 back)       +0.1275            17.6 mm
#   hem_edge     9 (5 front / 4 back)       +0.0943            77.9 mm
#   sleeve      12 (both walls together)    centre -0.0111     95.0 mm span
#
PRESETS = {
    # PRIMARY. Front panel, full-face, low on the torso. A single panel with its
    # normal along the jaw axis is exactly the geometry the patch-world pinch was
    # proven on, and here it is also the flattest thing on the garment: 6
    # particles inside the pad within a 17.6 mm band, with the back panel 190 mm
    # away and nowhere near the jaws.
    "front_low": (0.00, 0.1275, 0.055),
    # HARD CASE, kept because it is informative rather than because it works.
    # The hem's free bottom edge, off-centre, where the tube curves round toward
    # the side seam: the 5 front particles under the pad are spread over 77.9 mm
    # of Y, so a 16 mm pinch cannot possibly catch them all, and the pads
    # STRADDLE the edge so only their upper half has fabric on it at all.
    "hem_edge": (0.10, 0.0943, 0.005),
    # SECOND GRASP, and the interesting one. The sleeve cuff is a small tube:
    # 12 particles spanning 95 mm of Y sit inside the 140 mm open gap, so
    # closing does not pinch a surface, it GATHERS both walls of the sleeve into
    # the jaws -- a different mechanism from the flat-panel pinch, on a
    # different part of the garment.
    "sleeve": (0.305, -0.0111, 0.384),
}

# Phase durations in seconds, in order. Cloth runs at ~0.15x realtime here, so
# every second of schedule is ~6 s of wall clock and ~1 C on the GPU; these are
# sized to the measurement and not padded. `settle` in particular is 0.50 s
# because the settle probe showed the garment stable to ~1 mm by 0.36 s.
PHASES = [
    ("settle", 0.50),    # the garment hangs and stops swinging
    ("align", 0.60),     # move to the grasp Y and Z while still clear in X
    ("approach", 0.90),  # fly in along -X, jaws open
    ("close", 0.50),     # jaws open -> closed, IN PLACE (newton's guidance)
    ("pinch", 0.30),     # hold closed and still
    ("preload", 0.70),   # let the pinch settle before anything moves
    ("lift", 1.20),
    ("hold", 1.00),      # long and static: this is where the verdict is read
    ("release", 0.40),   # jaws open WIDE, clear of the gathered bundle
    ("fall", 0.60),      # ...and the fabric must drop. REPORTED, NOT SCORED:
                         # the release outcome measures the release motion.
]


def smoothstep(a, b, u):
    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
    return a + (b - a) * (u * u * (3.0 - 2.0 * u))


def env_float(name, default):
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return float(raw)
    except ValueError:
        print("vbd_tshirt_grasp: WARNING %s=%r is not a number, using %r"
              % (name, raw, default), flush=True)
        return default


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    mode = (os.environ.get("TSHIRT_MODE") or "grasp").strip().lower()
    if mode not in ("grasp", "negative", "settle"):
        print("vbd_tshirt_grasp: FATAL TSHIRT_MODE=%r not in grasp|negative|settle"
              % mode, flush=True)
        return 1
    preset_name = (os.environ.get("TSHIRT_GRASP_PRESET") or "front_low").strip().lower()
    if preset_name not in PRESETS:
        print("vbd_tshirt_grasp: FATAL TSHIRT_GRASP_PRESET=%r not in %s"
              % (preset_name, "|".join(PRESETS)), flush=True)
        return 1
    lx, ly, lz = PRESETS[preset_name]
    if ly is None:
        # No settled Y is baked in for this preset yet. Refuse rather than guess:
        # the rest-mesh Y is measurably not the hanging Y, and a silent wrong
        # guess produces a miss that looks exactly like a failed pinch.
        #
        # ⚠ settle mode is exempt -- it exists precisely to MEASURE that Y, and
        # is the run you make before any grasp point is known. (This guard did
        # fire in settle mode once. The cost is worth recording: a controller
        # that exits non-zero does not stop the engine, which then free-ran the
        # cloth flat out for the whole --duration and put 12 C on the GPU for
        # nothing.)
        ly = os.environ.get("TSHIRT_GY")
        if ly is None or not str(ly).strip():
            if mode != "settle":
                print("vbd_tshirt_grasp: FATAL preset '%s' has no settled Y and "
                      "TSHIRT_GY is unset. Run TSHIRT_MODE=settle first and pass the "
                      "MEASURED panel Y -- guessing it from the rest mesh is how you "
                      "get a miss that looks like a failed pinch." % preset_name,
                      flush=True)
                return 1
            ly = 0.0
        else:
            ly = float(ly)
    ly = float(ly)

    # World-frame grasp point, overridable component-wise.
    gx = env_float("TSHIRT_GX", SHIRT_ORIGIN[0] + lx)
    gy = env_float("TSHIRT_GY", SHIRT_ORIGIN[1] + ly)
    gz = env_float("TSHIRT_GZ", SHIRT_ORIGIN[2] + lz)
    lift_dz = env_float("TSHIRT_LIFT", 0.25)

    # Joint coordinates are deltas from the authored gantry root.
    tgt_dx = gx - GANTRY_ROOT[0]
    tgt_dy = gy - GANTRY_ROOT[1]
    tgt_dz = gz - GANTRY_ROOT[2]

    log_path = os.environ.get("TSHIRT_GRASP_LOG") or os.path.join(
        os.getcwd(), ".build_tmp", "tshirt_pads.jsonl")
    try:
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
    except OSError:
        pass

    motors = {}
    for name in ("gantry_x", "gantry_y", "gantry_z", "jaw_left", "jaw_right"):
        m = robot.getDevice(name)
        if m is None:
            print("vbd_tshirt_grasp: FATAL device '%s' missing" % name, flush=True)
            return 1
        motors[name] = m

    pad_l = robot.getFromDef("LEFT_PAD")
    pad_r = robot.getFromDef("RIGHT_PAD")
    if pad_l is None or pad_r is None:
        print("vbd_tshirt_grasp: FATAL LEFT_PAD / RIGHT_PAD not resolvable -- the pad "
              "poses are the whole measurement, so refusing to run blind.", flush=True)
        return 1

    phases = PHASES
    if mode == "settle":
        # Park clear and hang. Long enough for the garment to stop moving, and
        # not one second longer: cloth runs at ~0.17x realtime.
        phases = [("settle", 3.00)]

    bounds, acc = [], 0
    for name, secs in phases:
        n = int(round(secs / dt))
        bounds.append((name, acc, acc + n))
        acc += n
    total = acc

    print("vbd_tshirt_grasp: mode=%s preset=%s grasp=(%.4f, %.4f, %.4f) lift=%.3f"
          % (mode, preset_name, gx, gy, gz, lift_dz), flush=True)
    print("vbd_tshirt_grasp: dt=%d ms, %d steps (%.2f s), jaw closed=%.4f release=%.4f"
          % (dt_ms, total, total * dt, JAW_CLOSED, JAW_RELEASE), flush=True)
    print("vbd_tshirt_grasp: PHASES " + json.dumps(
        [{"phase": n, "start": s, "end": e} for n, s, e in bounds]), flush=True)

    def schedule(step):
        for name, s, e in bounds:
            if step < e:
                u = (step - s) / max(e - s, 1)
                if name == "settle":
                    return name, 0.0, 0.0, 0.0, JAW_OPEN
                if name == "align":
                    # Y and Z move while X is still parked outside the garment's
                    # 0.324 m half-width, so nothing sweeps through the fabric.
                    return (name, 0.0, smoothstep(0.0, tgt_dy, u),
                            smoothstep(0.0, tgt_dz, u), JAW_OPEN)
                if name == "approach":
                    return name, smoothstep(0.0, tgt_dx, u), tgt_dy, tgt_dz, JAW_OPEN
                if name == "close":
                    return (name, tgt_dx, tgt_dy, tgt_dz,
                            smoothstep(JAW_OPEN, JAW_CLOSED, u))
                if name in ("pinch", "preload"):
                    return name, tgt_dx, tgt_dy, tgt_dz, JAW_CLOSED
                if name == "lift":
                    return (name, tgt_dx, tgt_dy, tgt_dz + smoothstep(0.0, lift_dz, u),
                            JAW_CLOSED)
                if name == "hold":
                    return name, tgt_dx, tgt_dy, tgt_dz + lift_dz, JAW_CLOSED
                if name == "release":
                    return (name, tgt_dx, tgt_dy, tgt_dz + lift_dz,
                            smoothstep(JAW_CLOSED, JAW_RELEASE, u))
                return name, tgt_dx, tgt_dy, tgt_dz + lift_dz, JAW_RELEASE
        return "done", tgt_dx, tgt_dy, tgt_dz + lift_dz, JAW_RELEASE

    fh = open(log_path, "w", encoding="utf-8")
    fh.write(json.dumps({
        "meta": True, "mode": mode, "preset": preset_name,
        "grasp": [gx, gy, gz], "lift": lift_dz, "dt": dt,
        "jaw_closed": JAW_CLOSED, "closed_half_gap": CLOSED_HALF_GAP,
        "pad_size": [0.09, 0.01, 0.06],
    }) + "\n")

    step = 0
    prev_phase = None
    while robot.step(dt_ms) != -1:
        if step >= total:
            break
        phase, dx, dy, dz, jaw = schedule(step)
        if mode == "negative":
            # The control differs in EXACTLY one variable: the jaws never close.
            # Same flight path, same timing, same everything else.
            jaw = JAW_OPEN if phase not in ("release", "fall") else JAW_RELEASE

        motors["gantry_x"].setPosition(dx)
        motors["gantry_y"].setPosition(dy)
        motors["gantry_z"].setPosition(dz)
        motors["jaw_left"].setPosition(jaw)
        motors["jaw_right"].setPosition(jaw)

        pl = pad_l.getPosition()
        pr = pad_r.getPosition()
        rec = {
            "step": step,
            "t": round(robot.getTime(), 6),
            "phase": phase,
            "cmd": [round(dx, 6), round(dy, 6), round(dz, 6), round(jaw, 6)],
            "pad_l": [round(v, 6) for v in pl],
            "pad_r": [round(v, 6) for v in pr],
            # MEASURED midpoint of the two jaw bodies, never the commanded pose.
            "pad_c": [round(0.5 * (pl[i] + pr[i]), 6) for i in range(3)],
            # MEASURED jaw separation along the closing axis. Commanded is
            # 2 * CLOSED_HALF_GAP = 26 mm; the fabric pushes the PD-driven jaws
            # apart from it and the difference is the real shell compression.
            "gap": round(pr[1] - pl[1], 6),
        }
        fh.write(json.dumps(rec) + "\n")

        if phase != prev_phase:
            print("vbd_tshirt_grasp: MARK %-9s step=%d t=%.3f pad_c=(%.5f, %.5f, %.5f) "
                  "gap=%.5f" % (phase, step, rec["t"], rec["pad_c"][0], rec["pad_c"][1],
                                rec["pad_c"][2], rec["gap"]), flush=True)
            prev_phase = phase
        step += 1

    fh.close()
    # ⚠ PAUSE THE ENGINE, DO NOT JUST RETURN. `run-headless --duration N` is a
    # wall-clock SLEEP, not a progress target, so an engine whose controller has
    # finished keeps stepping the world flat out until N expires. On a cloth
    # world that is a GPU workload running for no reason: measured here, a 40 s
    # duration whose controller died at t=0 put 12 C on the GPU (61 -> 73) and
    # produced nothing. Pausing hands the remaining wall time back.
    try:
        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
    except Exception as e:                                   # noqa: BLE001
        print("vbd_tshirt_grasp: could not pause the engine (%s) -- the run will "
              "free-run until --duration expires" % (e,), flush=True)
    print("vbd_tshirt_grasp: wrote %d pad samples to %s" % (step, log_path), flush=True)
    print("vbd_tshirt_grasp: DONE (this controller does not score the grasp -- join "
          "this file with the OMNISIM_CLOTH_TELEMETRY JSONL to get the verdict)",
          flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
