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

"""What MuJoCo's ``rangefinder`` actually does -- MEASURED, not quoted.

    python rangefinder_conventions.py            # table
    python rangefinder_conventions.py --json     # machine-readable

Rung 5 asks a distance sensor for a range to a wall and compares it with the
geometry.  That comparison is only meaningful if both columns agree on what
the number MEANS, and the two do not agree by default.  This file pins
MuJoCo's answer with a scene whose right answer is known by construction, so
the conventions are a measurement of the version actually installed rather
than a recollection of the docs.  Every check below is a hard assertion: if a
future MuJoCo changes one, this goes red instead of quietly shifting rung 5's
ground truth underneath it.

WHAT IT MEASURES (MuJoCo 3.8.1, and re-derived on every run)

1.  ORIGIN AND AXIS.  The ray starts at the SITE ORIGIN and points along the
    site's +Z, not the body's +X.  ``zaxis="1 0 0"`` on the site is the
    idiom for a forward-facing sensor on a robot that drives along +X.
2.  IT RETURNS THE DISTANCE TO THE FIRST SURFACE, not to a centre and not a
    normalised value: a wall whose near face is at x = 3.0 read from a site
    at x = 0.3 gives 2.700000.
3.  NO HIT IS -1, NOT A MAX RANGE.  A ray pointing at nothing returns exactly
    -1.0.  Any consumer that treats the reading as a positive length must
    gate on ``>= 0`` first; ``-1`` is closer than any obstacle and a naive
    ``reading < threshold`` brake fires instantly on an EMPTY horizon.
4.  ``cutoff`` IS A CLAMP, NOT A RANGE LIMIT.  ``cutoff="1.0"`` on a sensor
    looking at a wall 2.7 m away reports 1.0 -- a saturated reading that is
    indistinguishable from a wall genuinely 1 m away.  It does NOT make the
    sensor report "nothing there" past 1 m.  Leave it at 0 (off) unless a
    saturating sensor is the thing being modelled.
5.  SAME-BODY GEOMS ARE EXCLUDED.  A site buried inside its own body's geom
    does not read that geom; it reads through it.  So a sensor mounted flush
    with (or inside) the chassis measures to the world, not to the chassis --
    which is what makes a front-face mount work at all.
6.  A RAY THAT STARTS INSIDE A GEOM READS ITS FAR FACE.  Not -1, not 0, not
    a negative depth: a plausible positive number that is not the distance to
    the surface the site is inside.  Rung 6 hits this for real -- its sensor
    is mounted 20 mm proud of the bumper, so a rover that runs into the wall
    buries it, and the reading jumps from ~0 to (wall thickness - 20 mm)
    while the true gap is -20 mm.  Any consumer that treats a range reading
    as "how much room is left" is wrong by a wall thickness at exactly the
    moment it matters most.
7.  IT IS A ``mjSTAGE_POS`` SENSOR, so it is refreshed by the forward pass and
    is exactly as fresh as ``site_xpos`` in the same ``mjData``.  That is why
    ``run.py`` samples after ``mj_step1``: reading and pose then describe the
    same instant, and rung 5's "reading == geometry" comparison has no clock
    skew in it (see ``drivers.py``).

WHERE OMNISIM IS EXPECTED TO DIFFER -- these are the hypotheses rung 5 tests
-------------------------------------------------------------------------
Every one of these is a difference in the WRAPPER, not in the solver, since
MuJoCo is the solver on both sides.

*   **No-hit encoding.**  A Webots-lineage ``DistanceSensor`` saturates at its
    own ``lookupTable`` maximum when nothing is in range.  MuJoCo returns -1.
    Two engines can be perfectly correct and disagree in sign on an empty
    horizon, so a shared assertion must be written on a HIT.
*   **Units and the lookup table.**  ``DistanceSensor`` maps raw range through
    ``lookupTable`` and may report volts, a 0..1000 count, or metres depending
    on the table the world declares.  MuJoCo reports metres, always.  A rung-5
    disagreement that is a clean linear scale factor is a table, not physics.
*   **Aperture and ray count.**  ``DistanceSensor`` can average N rays over an
    aperture; ``rangefinder`` is exactly one ray.  A multi-ray sensor reads
    SHORTER than the single-ray geometry near an edge.
*   **Staleness under ``mujoco_warp``.**  OmniSim's raycast-backed sensors have
    been measured returning the authored t=0 scene for ever on the GPU path
    while the CPU path tracks.  That is the failure this rung is shaped to
    catch, and it is why the rung must assert the reading CHANGES as the robot
    moves and not merely that one reading is right.
*   **Deleted geometry still occluding.**  A supervisor-deleted ``Solid``
    stays in OmniSim's compiled model, so a removed wall keeps stopping rays.
    Nothing in this arm can reproduce that: MJCF has no runtime delete.
"""

from __future__ import annotations

import argparse
import json
import sys

# Geometry of the probe scene.  Chosen so every right answer is an exact
# decimal, computable by hand from the numbers in this file alone.
WALL_NEAR_X = 3.0                   # the wall's near FACE
WALL_HALF = 0.1
SITE_X = 0.3                        # sensor sits on the body's front face
BODY_HALF = (0.3, 0.2, 0.075)
CUTOFF = 1.0
INSIDE_BY = 0.02                    # how far past the wall's near face the
                                    # buried-site probe sits

XML = """
<mujoco model="rangefinder_conventions">
  <option timestep="0.004" gravity="0 0 0"/>
  <worldbody>
    <geom name="wall" type="box" size="{wh} 2 1" pos="{wx} 0 0"/>
    <body name="rob" pos="0 0 0">
      <freejoint/>
      <geom name="robgeom" type="box" size="{bx} {by} {bz}" mass="5"/>
      <site name="front"  pos="{sx} 0 0" zaxis="1 0 0"/>
      <site name="buried" pos="0 0 0"    zaxis="1 0 0"/>
      <site name="back"   pos="-{sx} 0 0" zaxis="-1 0 0"/>
      <site name="up"     pos="0 0 {bz}" zaxis="0 0 1"/>
      <site name="in_wall" pos="{inx} 0 0" zaxis="1 0 0"/>
    </body>
  </worldbody>
  <sensor>
    <rangefinder name="s_front"  site="front"/>
    <rangefinder name="s_buried" site="buried"/>
    <rangefinder name="s_back"   site="back"/>
    <rangefinder name="s_up"     site="up"/>
    <rangefinder name="s_clamp"  site="front" cutoff="{cut}"/>
    <rangefinder name="s_inwall" site="in_wall"/>
  </sensor>
</mujoco>
""".format(wh=WALL_HALF, wx=WALL_NEAR_X + WALL_HALF, bx=BODY_HALF[0],
           by=BODY_HALF[1], bz=BODY_HALF[2], sx=SITE_X, cut=CUTOFF,
           inx=WALL_NEAR_X + INSIDE_BY)

NO_HIT = -1.0


def measure():
    """Run the probe.  Returns (facts, checks) -- no printing, no exit code."""
    import mujoco

    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def read(name):
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return float(data.sensordata[int(model.sensor_adr[i])])

    def stage(name):
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return str(mujoco.mjtStage(model.sensor_needstage[i])).split(".")[-1]

    free = model.jnt_qposadr[0]
    at_origin = {n: read(n) for n in ("s_front", "s_buried", "s_back", "s_up",
                                      "s_clamp", "s_inwall")}

    # Drive the body forward 1 m and re-read: the sensor must TRACK.
    data.qpos[free] = 1.0
    mujoco.mj_forward(model, data)
    moved = read("s_front")

    # A pose the forward pass has NOT yet seen: qpos is written but no
    # forward has run, so the reading must still be the OLD one.  This is the
    # freshness contract the step loop depends on.
    data.qpos[free] = 2.0
    stale = read("s_front")
    mujoco.mj_forward(model, data)
    refreshed = read("s_front")

    facts = {
        "mujoco_version": mujoco.__version__,
        "wall_near_face_x_m": WALL_NEAR_X,
        "site_x_m": SITE_X,
        "reading_at_origin": at_origin,
        "reading_after_1m": moved,
        "reading_before_forward_at_2m": stale,
        "reading_after_forward_at_2m": refreshed,
        "sensor_stage": stage("s_front"),
        "cutoff_declared": CUTOFF,
    }

    exact = WALL_NEAR_X - SITE_X                    # 2.7 m, by construction
    checks = [
        ("ray starts at the SITE and follows its +Z",
         at_origin["s_front"], exact, 1e-9),
        ("distance is to the first SURFACE, not a centre",
         at_origin["s_front"], exact, 1e-9),
        ("it tracks the body: +1 m of motion is -1 m of range",
         moved, exact - 1.0, 1e-9),
        ("no hit is exactly -1, NOT a max range (backward ray)",
         at_origin["s_back"], NO_HIT, 0.0),
        ("no hit is exactly -1, NOT a max range (upward ray)",
         at_origin["s_up"], NO_HIT, 0.0),
        ("cutoff CLAMPS the reading, it does not limit range",
         at_origin["s_clamp"], CUTOFF, 1e-9),
        ("a site inside its own body's geom reads THROUGH it",
         at_origin["s_buried"], WALL_NEAR_X, 1e-9),
        ("a ray STARTING INSIDE a geom reads its FAR face, not -1",
         at_origin["s_inwall"], 2.0 * WALL_HALF - INSIDE_BY, 1e-9),
        ("stale until the forward pass runs (freshness contract)",
         stale, exact - 1.0, 1e-9),
        ("refreshed by the forward pass",
         refreshed, exact - 2.0, 1e-9),
    ]
    rows = [{"what": w, "measured": m, "expected": e, "tol": t,
             "ok": bool(abs(m - e) <= t)} for w, m, e, t in checks]
    facts["sensor_stage_is_pos"] = (facts["sensor_stage"] == "mjSTAGE_POS")
    rows.append({"what": "it is a POSITION-stage sensor",
                 "measured": facts["sensor_stage"], "expected": "mjSTAGE_POS",
                 "tol": 0.0, "ok": facts["sensor_stage_is_pos"]})
    return facts, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    try:
        facts, rows = measure()
    except ImportError as exc:
        print("mujoco not importable: %s" % exc, file=sys.stderr)
        return 2
    ok = all(r["ok"] for r in rows)
    if args.json:
        json.dump({"facts": facts, "checks": rows, "ok": ok}, sys.stdout,
                  indent=2, sort_keys=True)
        print("")
        return 0 if ok else 1
    print("MuJoCo %s -- rangefinder conventions, measured" % facts["mujoco_version"])
    print("scene: wall near face x=%.1f, sensor site x=%.1f, so the exact "
          "answer is %.1f m" % (WALL_NEAR_X, SITE_X, WALL_NEAR_X - SITE_X))
    print("")
    for r in rows:
        m = r["measured"]
        print("  %-3s %-58s %s"
              % ("OK" if r["ok"] else "RED", r["what"],
                 m if isinstance(m, str) else "%.6f" % m))
    print("")
    print("VERDICT: %s" % ("conventions hold" if ok else "A CONVENTION MOVED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
