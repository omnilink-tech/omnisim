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
"""vbd_sponge_dishwash -- a gripper picks up a soft sponge and scrubs a dish.

Drives the 5-DoF prismatic gantry in
projects/samples/demos/worlds/physics/newton_vbd_sponge_dishwash.omniworld:
approach -> close on the sponge -> lift -> traverse -> press -> raster scrub ->
lift -> park. Cleanliness is measured, not asserted.

RUN IT
------
    OMNISIM_CLOTH_SELF_CONTACT=0 \\
    OMNISIM_CLOTH_TELEMETRY=$PWD/.build_tmp/sponge.jsonl \\
    OMNISIM_CLOTH_TELEMETRY_EVERY=10 \\
    SPONGE_LOG=$PWD/.build_tmp/sponge_pads.jsonl \\
    python -m omnisim run-headless \\
      projects/samples/demos/worlds/physics/newton_vbd_sponge_dishwash.omniworld \\
      --duration 420

WHAT THIS CONTROLLER CAN AND CANNOT SEE
---------------------------------------
Under `newtonSolver "vbd"` there is no mj_model, so there is NO contact
readback of any kind: getContactPoints, GET /sim/contacts and GET /sim/grips
are all empty. This controller therefore CANNOT ask the engine whether the
sponge is gripped, and does not pretend to.

Worse, a `SoftBody` has no supervisor readback at all -- its `translation` is
never written back by the deformation, so `getFromDef("SPONGE").getPosition()`
returns the AUTHORED corner for ever, whether the block is simulating
perfectly or is completely inert. Never use it as evidence of anything. The
only readback that exists is the engine's own OMNISIM_CLOTH_TELEMETRY JSONL
(per-grid centroid / bbox / soft_contacts / nonfinite), which is written
out-of-band and joined against this controller's log after the run.

`PositionSensor` is also dead on this path -- VBD integrates body_q and never
maintains joint_q, so a sensor reads 0.0 for ever, silently. Every position
below comes from Supervisor.getFromDef(...).getPosition(), which reads body_q.

So the two things this controller CAN measure honestly are:
  1. the pad Solids' world poses, and hence the LIVE JAW GAP; and
  2. how far the specks it decided to clean have actually been lerped.

THE GATE
--------
Everything hinges on (1), and on ONE quantity: how far the jaws fall SHORT of
what they were just commanded. An empty gripper reaches its command at any
opening; a loaded one cannot, because the sponge is a spring it has to
compress. So

    gap_excess = measured_pad_separation - 2 * (PAD_REST_OFFSET - jaw_command)

is positive only while something is actually between the pads, and no speck is
cleaned unless it clears `SPONGE_GAP_EXCESS`.

⚠ IT IS DELIBERATELY RELATIVE TO THE LIVE COMMAND, NOT AN ABSOLUTE GAP. An
earlier version compared the raw gap against a fixed threshold derived from the
CLOSED command, which an open jaw at 0.140 m clears trivially -- it reported
"holding" for all 492 samples of a run, including the phases before the gripper
had gone anywhere near the sponge.

MEASURED on the working run: excess runs ~1.2 mm at the pinch and the held gap
is 0.0822-0.0983 against a commanded 0.0810. The signal is millimetres, because
the sponge yields almost completely -- which is exactly why an absolute
threshold guessed from geometry does not work here (the first attempt guessed
0.093 against a real 0.082 and gated out all 825 scrub steps of a run in which
the telemetry shows the sponge was gripped, lifted and carried).

That gate is what stops this demo from scoring the gripper's INTENT instead of
its achievement: run it with SPONGE_CONTROL_MISS=1 and the gantry deliberately
grabs empty air 120 mm to the side of the sponge, executes the identical scrub
path, and MUST leave the dish dirty. A demo whose control arm also passes is
not measuring what it claims to.

⚠ SPONGE_GAP_EXCESS's default is still only calibrated against ONE control run.
Whatever excess the control arm reports is the noise floor this must clear;
re-read `gap_excess_max` from both logs before quoting a cleanliness number.

⚠ `print()` output does not reliably reach anywhere on Windows (omnisim-bin is
a GUI-subsystem binary), which is why every number goes to SPONGE_LOG as JSONL.
The verdict is that file, NOT the headless exit code -- this class of demo
reports a non-zero exit AFTER the controller completes and pauses the sim,
while a run that dies early reports PASS.
"""

import json
import math
import os

from omnisim import Supervisor

# --- geometry, all mirrored from the .omniworld ----------------------------
GANTRY_Z = 1.30          # DEF GANTRY translation z; pad world z = GANTRY_Z + gz

SPONGE_CX = -0.18        # sponge centre x (min corner -0.23 + half of 0.10)
SPONGE_CY = 0.0
SPONGE_CZ = 0.93         # spans z 0.91 .. 0.95
SPONGE_HALF_Y = 0.0375   # half of the 75 mm pinch dimension
PARTICLE_R = 0.01

DISH_CX = 0.18
DISH_CY = 0.0
DISH_TOP_Z = 0.912       # plate surface: translation 0.906 + half of 0.012
DISH_R = 0.10

PAD_REST_OFFSET = 0.07   # |pad translation y| at jaw command 0
PAD_HALF_THICK = 0.005   # half of the 0.01 pad Box y-size
PAD_HALF_HEIGHT = 0.02   # half of the 0.04 pad Box z-size

# Jaw command is "how far closed from open", identical for both jaws because
# their axes oppose. Contact when the pad inner face meets the sponge's outer
# particle surface:
#     PAD_REST_OFFSET - q - PAD_HALF_THICK == SPONGE_HALF_Y + PARTICLE_R
JAW_OPEN = 0.0
JAW_TOUCH = PAD_REST_OFFSET - PAD_HALF_THICK - (SPONGE_HALF_Y + PARTICLE_R)
JAW_CLOSED = JAW_TOUCH + float(os.environ.get("SPONGE_INTERFERENCE", "0.012"))
# Opening to merely JAW_OPEN does not reliably release a deformable -- the
# sibling t-shirt demo found the bundle re-forms inside the pads and they act
# as ledges. Retract well past rest.
JAW_RELEASE = -0.06

# Commanded gap with the jaws at JAW_CLOSED and nothing between them.
GAP_COMMANDED = 2.0 * (PAD_REST_OFFSET - JAW_CLOSED)
# Gap the sponge would hold if it were RIGID. Retained only to show how far off
# a geometric prediction is -- see below.
GAP_SPONGE_RIGID = 2.0 * (SPONGE_HALF_Y + PARTICLE_R + PAD_HALF_THICK)

# ⚠ THE FIRST VERSION OF THIS GATE WAS WRONG AND IT MATTERS HOW.
#
# It thresholded at the midpoint of GAP_COMMANDED (0.081) and GAP_SPONGE_RIGID
# (0.105), i.e. 0.093, on the reasoning that the sponge holds the jaws apart
# somewhere between "not there" and "rigid". MEASURED, the held gap is 0.08200
# -- the sponge yields almost completely, so the jaws very nearly reach their
# command and the whole signal is the ~1 mm they fall short by. The gate
# therefore read "nothing in the jaws" for all 825 scrub steps of a run in
# which the telemetry shows the sponge was gripped, lifted 88 mm, carried
# 360 mm and pressed onto the plate.
#
# That failure was in the SAFE direction -- it refused to clean rather than
# cleaning on a false positive -- which is the only reason it is a calibration
# bug and not a fabricated result. Keep the gate biased that way.
#
# So the gate keys on the EXCESS over the commanded gap, which is the jaws'
# measured shortfall against a spring they cannot fully close. Its default is
# deliberately small, and it is still a PREDICTION until the control arm has
# run: SPONGE_CONTROL_MISS=1 grabs air, and whatever excess THAT reports is the
# noise floor this threshold must clear.
SPONGE_GAP_EXCESS = float(os.environ.get("SPONGE_GAP_EXCESS", "0.0005"))
SPONGE_MIN_GAP = float(
    os.environ.get("SPONGE_MIN_GAP", "%.6f" % (GAP_COMMANDED + SPONGE_GAP_EXCESS))
)

# --- scrub model -----------------------------------------------------------
SCRUB_RADIUS = 0.045     # sponge contact-patch half-width on the plate
SCRUB_RATE = 2.2         # grime units removed per second at the pad centre
CONTACT_BAND = 0.010     # how close the sponge underside must be to the plate
N_SPECKS = 12
GRIME_RGB = (0.34, 0.26, 0.14)
CLEAN_RGB = (0.94, 0.945, 0.93)
QUANT = 1.0 / 255.0      # skip a colour write smaller than one 8-bit step

# --- the negative control --------------------------------------------------
CONTROL_MISS = os.environ.get("SPONGE_CONTROL_MISS", "") not in ("", "0")
MISS_OFFSET_Y = 0.12     # grab empty air this far to the side of the sponge

LOG_PATH = os.environ.get(
    "SPONGE_LOG",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".build_tmp",
                 "sponge_pads.jsonl"),
)


def smoothstep(t):
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return t * t * (3.0 - 2.0 * t)


def lerp(a, b, t):
    return a + (b - a) * t


def lerp3(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


robot = Supervisor()
dt_ms = int(robot.getBasicTimeStep())
dt = dt_ms / 1000.0

motors = {}
for nm in ("gantry_x", "gantry_y", "gantry_z", "jaw_left", "jaw_right"):
    m = robot.getDevice(nm)
    if m is None:
        raise SystemExit("vbd_sponge_dishwash: missing motor %r" % nm)
    motors[nm] = m

left_pad = robot.getFromDef("LEFT_PAD")
right_pad = robot.getFromDef("RIGHT_PAD")
if left_pad is None or right_pad is None:
    raise SystemExit("vbd_sponge_dishwash: LEFT_PAD / RIGHT_PAD not found")

# Each speck owns its own PBRAppearance -- a USE would share one node and a
# single write would clean the whole plate at once.
specks = []
for i in range(N_SPECKS):
    pose = robot.getFromDef("GRIME_%02d" % i)
    app = robot.getFromDef("GRIME_APP_%02d" % i)
    if pose is None or app is None:
        continue
    px, py, _ = pose.getPosition()          # world, already includes the dish
    radius = (pose.getField("children").getMFNode(0)
                  .getField("geometry").getSFNode()
                  .getField("radius").getSFFloat())
    specks.append({
        "i": i, "x": px, "y": py,
        "w": radius * radius,               # area weight
        "g": 1.0,                           # 1 = filthy, 0 = clean
        "written": 1.0,
        "color": app.getField("baseColor"),
    })
total_w = sum(s["w"] for s in specks) or 1.0

grab_y = SPONGE_CY + (MISS_OFFSET_Y if CONTROL_MISS else 0.0)

# (name, seconds, gx0, gy0, gz0, jaw0) -> (gx1, gy1, gz1, jaw1), interpolated.
# gz is the CARRIAGE offset; pad world z = GANTRY_Z + gz.
PARK = (0.0, 0.0, -0.20)
GRAB_Z = SPONGE_CZ - GANTRY_Z                       # pads centred on the sponge
LIFT_Z = GRAB_Z + 0.09
# Pressing: the sponge underside sits PAD_HALF_HEIGHT + PARTICLE_R below the pad
# centre when held. Put it PRESS_DEPTH into the plate surface.
PRESS_DEPTH = 0.006
PRESS_Z = (DISH_TOP_Z + PAD_HALF_HEIGHT + PARTICLE_R - PRESS_DEPTH) - GANTRY_Z

plan = [
    ("settle",   1.0, (PARK[0], PARK[1], PARK[2], JAW_OPEN),
                      (PARK[0], PARK[1], PARK[2], JAW_OPEN)),
    ("align",    1.5, (PARK[0], PARK[1], PARK[2], JAW_OPEN),
                      (SPONGE_CX, grab_y, PARK[2], JAW_OPEN)),
    ("descend",  1.5, (SPONGE_CX, grab_y, PARK[2], JAW_OPEN),
                      (SPONGE_CX, grab_y, GRAB_Z, JAW_OPEN)),
    ("close",    1.2, (SPONGE_CX, grab_y, GRAB_Z, JAW_OPEN),
                      (SPONGE_CX, grab_y, GRAB_Z, JAW_CLOSED)),
    ("preload",  0.6, (SPONGE_CX, grab_y, GRAB_Z, JAW_CLOSED),
                      (SPONGE_CX, grab_y, GRAB_Z, JAW_CLOSED)),
    ("lift",     1.5, (SPONGE_CX, grab_y, GRAB_Z, JAW_CLOSED),
                      (SPONGE_CX, grab_y, LIFT_Z, JAW_CLOSED)),
    ("traverse", 1.8, (SPONGE_CX, grab_y, LIFT_Z, JAW_CLOSED),
                      (DISH_CX, DISH_CY, LIFT_Z, JAW_CLOSED)),
    ("press",    1.2, (DISH_CX, DISH_CY, LIFT_Z, JAW_CLOSED),
                      (DISH_CX, DISH_CY, PRESS_Z, JAW_CLOSED)),
]

# The scrub itself is a raster, generated rather than written out. Three passes
# across the plate in x at three y offsets, staying inside the supported
# collider (Box side 0.170 -> |x|,|y| <= 0.085 from the dish centre).
SCRUB_HALF = 0.055
SCRUB_ROWS = (-SCRUB_HALF, 0.0, SCRUB_HALF)
prev = (DISH_CX, DISH_CY, PRESS_Z, JAW_CLOSED)
for row_i, ry in enumerate(SCRUB_ROWS):
    y = DISH_CY + ry
    x_a = DISH_CX - SCRUB_HALF if row_i % 2 == 0 else DISH_CX + SCRUB_HALF
    x_b = DISH_CX + SCRUB_HALF if row_i % 2 == 0 else DISH_CX - SCRUB_HALF
    plan.append(("scrub%d_in" % row_i, 0.6, prev, (x_a, y, PRESS_Z, JAW_CLOSED)))
    plan.append(("scrub%d" % row_i, 1.6, (x_a, y, PRESS_Z, JAW_CLOSED),
                 (x_b, y, PRESS_Z, JAW_CLOSED)))
    prev = (x_b, y, PRESS_Z, JAW_CLOSED)
plan.append(("retreat", 1.0, prev, (DISH_CX, DISH_CY, LIFT_Z, JAW_CLOSED)))
plan.append(("release", 0.8, (DISH_CX, DISH_CY, LIFT_Z, JAW_CLOSED),
             (DISH_CX, DISH_CY, LIFT_Z, JAW_RELEASE)))
plan.append(("park", 1.0, (DISH_CX, DISH_CY, LIFT_Z, JAW_RELEASE),
             (PARK[0], PARK[1], PARK[2], JAW_RELEASE)))

try:
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
except OSError:
    pass
log = open(LOG_PATH, "w", encoding="utf-8")
log.write(json.dumps({
    "kind": "header",
    "control_miss": CONTROL_MISS,
    "dt_ms": dt_ms,
    "jaw_touch": round(JAW_TOUCH, 6),
    "jaw_closed": round(JAW_CLOSED, 6),
    "gap_commanded": round(GAP_COMMANDED, 6),
    "gap_sponge_rigid": round(GAP_SPONGE_RIGID, 6),
    "sponge_min_gap": round(SPONGE_MIN_GAP, 6),
    "press_z_pad_world": round(GANTRY_Z + PRESS_Z, 6),
    "dish_top_z": DISH_TOP_Z,
    "n_specks": len(specks),
}) + "\n")
log.flush()

step = 0
gap_held_min = None
gap_held_max = None
excess_max = None
n_gated_out = 0
n_scrub_steps = 0

for name, dur, a, b in plan:
    n = max(1, int(round(dur / dt)))
    for k in range(1, n + 1):
        if robot.step(dt_ms) == -1:
            break
        step += 1
        s = smoothstep(k / float(n))
        gx, gy, gz, jaw = (lerp(a[j], b[j], s) for j in range(4))
        motors["gantry_x"].setPosition(gx)
        motors["gantry_y"].setPosition(gy)
        motors["gantry_z"].setPosition(gz)
        motors["jaw_left"].setPosition(jaw)
        motors["jaw_right"].setPosition(jaw)

        # MEASURED pad poses -- never the commanded values.
        pl = left_pad.getPosition()
        pr = right_pad.getPosition()
        gap = pr[1] - pl[1]
        cx = 0.5 * (pl[0] + pr[0])
        cy = 0.5 * (pl[1] + pr[1])
        cz = 0.5 * (pl[2] + pr[2])

        # Is a sponge actually between the pads? Measured, not assumed.
        #
        # ⚠ COMPARE AGAINST THE LIVE COMMAND, NOT A FIXED NUMBER. An earlier
        # version tested `gap >= SPONGE_MIN_GAP` with SPONGE_MIN_GAP baked from
        # the CLOSED command, so during `settle` and `align` -- jaws wide open
        # at 0.140 m -- it trivially passed and reported "holding" for all 492
        # samples of a run. It never produced a false clean, because only the
        # scrub phases clean and there the jaws ARE commanded closed, but a
        # gate that is true when the gripper is empty and open is not measuring
        # what it says it measures.
        #
        # The honest quantity is the jaws' SHORTFALL against whatever they were
        # just told to do: an empty gripper reaches its command at any opening,
        # a loaded one cannot.
        gap_cmd = 2.0 * (PAD_REST_OFFSET - jaw)
        gap_excess = gap - gap_cmd
        holding = gap_excess >= SPONGE_GAP_EXCESS
        # Recorded UNCONDITIONALLY over the working phases, not only when the
        # gate says we are holding -- a gate that is mis-calibrated must still
        # leave behind the numbers needed to re-calibrate it.
        if name.startswith("scrub") or name == "press":
            gap_held_min = gap if gap_held_min is None else min(gap_held_min, gap)
            gap_held_max = gap if gap_held_max is None else max(gap_held_max, gap)
            excess_max = gap_excess if excess_max is None else max(excess_max, gap_excess)

        # The sponge underside, inferred from the pad centre. This is a
        # geometric inference from a MEASURED pad pose, not a particle read --
        # the particles are only visible through OMNISIM_CLOTH_TELEMETRY.
        sponge_bottom = cz - PAD_HALF_HEIGHT - PARTICLE_R
        pressed = (sponge_bottom - DISH_TOP_Z) <= CONTACT_BAND

        if pressed and name.startswith("scrub"):
            n_scrub_steps += 1
            if not holding:
                n_gated_out += 1
            else:
                for sp in specks:
                    if sp["g"] <= 0.0:
                        continue
                    d = math.hypot(cx - sp["x"], cy - sp["y"])
                    if d >= SCRUB_RADIUS:
                        continue
                    fall = 0.5 * (1.0 + math.cos(math.pi * d / SCRUB_RADIUS))
                    sp["g"] = max(0.0, sp["g"] - SCRUB_RATE * fall * dt)

        # Push only what changed; every setSFColor is an RPC at a step boundary.
        for sp in specks:
            if abs(sp["g"] - sp["written"]) >= QUANT:
                sp["color"].setSFColor(lerp3(CLEAN_RGB, GRIME_RGB, sp["g"]))
                sp["written"] = sp["g"]

        if step % 5 == 0:
            log.write(json.dumps({
                "kind": "s", "t": round(robot.getTime(), 4), "phase": name,
                "cmd": [round(gx, 6), round(gy, 6), round(gz, 6), round(jaw, 6)],
                "pad_c": [round(cx, 6), round(cy, 6), round(cz, 6)],
                "gap": round(gap, 6),
                "gap_cmd": round(gap_cmd, 6),
                "gap_excess": round(gap_excess, 6),
                "holding": holding,
                "pressed": pressed,
                "clean": round(1.0 - sum(s2["g"] * s2["w"] for s2 in specks) / total_w, 6),
            }) + "\n")
    log.flush()

remaining = sum(s["g"] * s["w"] for s in specks)
cleanliness = 1.0 - remaining / total_w
n_clean = sum(1 for s in specks if s["g"] <= 0.0)

# THE VERDICT. The control arm must FAIL to clean; the real arm must clean.
if CONTROL_MISS:
    ok = cleanliness < 0.05
    why = "control arm: grabbed air, dish must stay dirty"
else:
    # gap_held_* is now recorded unconditionally, so it no longer witnesses a
    # grip. The gate does: a scrub step only cleans while the jaws measurably
    # fail to close, so cleanliness > 0 already implies the sponge was there.
    ok = cleanliness >= 0.80 and n_gated_out < 0.2 * max(1, n_scrub_steps)
    why = "real arm: sponge held through the scrub and dish measurably cleaned"

log.write(json.dumps({
    "kind": "result",
    "control_miss": CONTROL_MISS,
    "cleanliness": round(cleanliness, 6),
    "specks_fully_clean": n_clean,
    "n_specks": len(specks),
    "gap_held_min": None if gap_held_min is None else round(gap_held_min, 6),
    "gap_held_max": None if gap_held_max is None else round(gap_held_max, 6),
    "gap_commanded": round(GAP_COMMANDED, 6),
    "gap_excess_max": None if excess_max is None else round(excess_max, 6),
    "gap_excess_threshold": SPONGE_GAP_EXCESS,
    "scrub_steps": n_scrub_steps,
    "scrub_steps_gated_out": n_gated_out,
    "steps": step,
    "sim_time_s": round(robot.getTime(), 3),
    "ok": ok,
    "why": why,
}) + "\n")
log.close()

print("SPONGE cleanliness=%.4f specks=%d/%d gap_held=[%s,%s] cmd=%.4f ok=%s"
      % (cleanliness, n_clean, len(specks),
         "n/a" if gap_held_min is None else "%.4f" % gap_held_min,
         "n/a" if gap_held_max is None else "%.4f" % gap_held_max,
         GAP_COMMANDED, ok), flush=True)

robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
