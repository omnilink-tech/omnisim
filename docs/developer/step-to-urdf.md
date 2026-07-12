# STEP (CAD) → URDF

How to turn a `.step` / `.stp` CAD assembly into an OmniSim URDF robot you can
drop into a world, colour-correct, and (optionally) drive.

The whole thing is wrapped in one script —
[`scripts/dev/step_to_urdf.py`](../../scripts/dev/step_to_urdf.py) — but the
**orientation** and **verification** steps below are where it's easy to go
wrong, so read them before trusting the output.

Throughout, the worked example is a fictional wheeled rover (`acme_rover.step`
→ `projects/robots/acme/rover/`). Substitute your own CAD file and target
directory.

---

## TL;DR

```bash
# one-time
python -m pip install cascadio trimesh

# convert (CAD already Z-up; a robot that sits on the ground):
python scripts/dev/step_to_urdf.py \
    3d-models/acme_rover.step \
    projects/robots/acme/rover \
    --name rover --up z --ground bottom
```

That writes `meshes/*.stl`, `urdf/rover.urdf`, and `omnisim.yaml`, and prints a
report plus a ready-to-paste `URDFRobot { … }` snippet. **Then verify the
orientation** (next section) before believing it.

---

## What the tool does

1. **Tessellate** the STEP with `cascadio` (an OpenCASCADE wheel) → a GLB. CAD
   is exact surfaces; the simulator needs triangles. `--tol` controls fineness.
2. **Group triangles by CAD colour.** A STEP assembly carries per-solid colours;
   cascadio preserves them in the GLB. The tool buckets geometry by colour and
   exports **one mesh per colour**.
3. **Orient** (`--up`) and **recenter** (`--ground`) — see below.
4. **Emit a URDF**: a single `base_link` with **one `<visual>` per colour
   group**, each with its own `<material><color>`.
5. Write `omnisim.yaml` (`publish: true`) and print a report.

### Why one `<visual>` per colour (don't merge into a single mesh)

The OmniSim URDF importer turns each `<mesh>` into a Webots `Mesh {}` wrapped in
a `Shape` with **one** appearance, and it builds that appearance from the URDF
`<material><color>` — **not** from the mesh file's own materials
([`WbUrdfImporter.cpp`](../../src/omnisim/vrml/WbUrdfImporter.cpp), the
`emitVisual` / appearance path). So:

- A single combined mesh → one flat colour for the whole model.
- One `<visual>` per colour group, each with a matching `<material><color>` →
  the importer emits one `PBRAppearance` per part and the **original CAD colours
  survive**. This is why the tool splits by colour.

### Static prop by construction

The link has **no `<collision>` and no `<inertial>`**, so the importer emits no
`Physics` and no `boundingObject` → a **static, kinematic-only prop** (AGENTS.md
"visual-only prop"). It won't fall or snag. To make it move, drive it from a
supervisor (see "Making it move").

---

## Orientation — the part everyone gets wrong

**CAD files do not agree on which axis is "up".** cascadio's GLB is often
already Z-up, but glTF's nominal convention is Y-up, and a given STEP can be
anything. If you guess wrong you stand the model **on its face**.

> Cautionary tale. Take a low, long 4-wheeled rover: its GLB comes out of
> cascadio already Z-up, but an assumed "glTF Y-up → Z-up" 90° rotation tips it
> onto its nose and turns a 1.3 m-**long** rover into a 1.3 m-**tall** tower.
> Rendered from a flattering angle, that mistake is genuinely hard to see — it
> just looks like an oddly boxy robot. Check the numbers, not the picture.

The tool will **not guess**. You pass `--up {x,-x,y,-y,z,-z}` (which CAD axis
maps to Webots +Z) and then **verify**:

1. **Cheap sanity check (printed for you).** A grounded robot is wider/longer
   than it is tall, so its **height (Z) should be the smallest extent**. The
   report flags it if Z is *not* the smallest — that usually means `--up` is
   wrong.

2. **The real check — render six axis views and find the wheels/feet/base.**
   Put the model in a throwaway world and look at it from ±X, ±Y, ±Z. The
   bottom is the face whose outward axis shows the wheels/feet/base. Use the
   harness:

   ```bash
   python scripts/harness/omnisim_harness.py --port 6789       # in one shell
   curl -s -X POST localhost:6789/world/load -d '{"path":"…/your_check.wbt"}'
   # for each of 6 camera positions:
   curl -s -X POST localhost:6789/scene/look_at \
       -d '{"position":[0,0,5.2],"target":[0,0,1.2]}'          # top (+Z)
   curl -s -X POST localhost:6789/world/screenshot -d '{}' -o top.png
   #   …repeat for [0,0,-2.8] (bottom), [4,0,1.2]/[-4,0,1.2] (±X),
   #     [0,4,1.2]/[0,-4,1.2] (±Y), all targeting the model centre.
   ```

   If wheels/feet/base aren't on the bottom, re-run `step_to_urdf.py` with a
   different `--up`.

### `--ground`

- `--ground bottom` (default): the **lowest point sits at mesh z = 0**, so a
  world `translation … 0` puts the wheels/feet on the floor.
- `--ground center`: bounding-box centre at the origin (handy for a display
  prop you'll place by its middle).

---

## Putting it in a world

Paste the snippet the tool prints into a `.wbt` that uses the
[canonical lighting recipe](../WORLD_RECIPE.md):

```vrml
DEF MYBOT URDFRobot {
  url "../urdf/mybot.urdf"
  translation 0 0 0          # wheels/feet on the floor (--ground bottom)
  name "mybot"
  controller "<none>"
}
```

Two worlds are usually worth authoring per import: a **static showroom** (the
model alone on a plain floor, for the six-axis orientation check) and a
**driven** world (a supervisor moving it along a path, for the motion check
below).

### Making it move (supervisor-driven)

The prop has no drivetrain, so a supervisor moves its pose each tick. Set
`supervisor TRUE` and a controller, then write the robot's own `translation` /
`rotation` fields:

```python
from omnisim import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
node = sup.getFromDef("MYBOT")
tf = node.getField("translation")
rf = node.getField("rotation")

while sup.step(dt) != -1:
    x, y, heading = next_pose_on_path(sup.getTime())
    tf.setSFVec3f([x, y, 0.0])
    rf.setSFRotation([0, 0, 1, heading - YAW_OFFSET])
```

**Mind the forward axis.** A CAD import's "forward" is whichever horizontal axis
runs along the model's length, and that is frequently local **+Y**, not +X. If
it is +Y, then to face world heading `h` you set yaw = `h − π/2` (that's
`YAW_OFFSET` above). Get this wrong and the robot crabs sideways along its path
while facing 90° off. Read the axis off the tool's bounding-box report (the long
horizontal axis) and confirm it with the heading check below.

This is honest *scripted kinematic* motion, not wheel-torque physics — say so in
the demo's README rather than implying a drivetrain the model doesn't have.

---

## Verify NUMERICALLY, not just by screenshot

Screenshots hide geometry errors (perspective + missing contact shadows make a
grounded object look like it's floating, and a flattering angle hides a tipped
model). Prove it with numbers via the harness `/robots` endpoint:

- **Grounding / orientation** — compute the world-space AABB of the meshes as
  placed (apply the world yaw + translation). Assert **z-min ≈ 0** (on the
  floor) and that **height is the smallest extent** (lies flat, not on its
  face). Yaw about Z doesn't change the Z extent, so this holds at every
  heading.
- **Motion** — sample the pose over time and check it follows the intended
  path (e.g. on-ellipse residual `|(x/A)²+(y/B)²−1| ≈ 0`), z stays constant,
  and the heading tracks the velocity (forward-vs-tangent error in degrees).

A passing report for a grounded rover on an elliptical patrol looks like:
z-min = 0.0000 at every sampled heading; height is the smallest of the three
extents; the full loop is traversed; heading error mean well under 1°. Anything
else — a nonzero z-min, a height that isn't the smallest extent, a heading error
near 90° — points straight back at `--up`, `--ground`, or the forward-axis
offset.

### Two harness gotchas that will silently break your check

- **`/robots` `orientation` is a 3×3 row-major rotation matrix** (9 floats),
  **not** axis-angle. The robot's local +Y in world coords is **column 1** =
  `(m[1], m[4], m[7])`. (Parsing it as axis-angle gives garbage headings.)
- **The harness runs the sim several × faster than wall-clock.** Sampling
  positions on a wall-clock timer **aliases** (you'll measure absurd speeds and
  random headings). Either sample as fast as possible and compute per-sample
  quantities (recover the path phase from position, compare to reported
  orientation), or drive the sim with `/sim/step`. The real, deterministic
  speed is whatever your controller's period implies; the GUI runs real-time.

---

## Tips & gotchas

- **96 MB+ STEP files** are common. The converted meshes (a few MB of STL) are
  the reproducible deliverable; you generally **don't** commit the raw `.step`
  to git history (it's huge). Keep it in `3d-models/` (untracked — see
  `.gitignore`) or note where it came from.
- **Too many tiny solids?** `--no-group-by-color` gives one mesh per CAD solid;
  the default `--group-by-color` merges same-colour solids into far fewer
  visuals (a typical assembly collapses to well under a dozen).
- **Heavy meshes** (>100 k vertices per part) draw a harmless importer warning;
  decimate only if the GUI feels slow.
- **Colours look off?** The tool normalises CAD colours to 0–1 `baseColor`. CAD
  display colours are sRGB-ish; if you need physically-accurate values, tweak
  the `<material><color>` in the generated URDF by hand.

---

## See also

- [`scripts/dev/step_to_urdf.py`](../../scripts/dev/step_to_urdf.py) — the tool.
- [AGENTS.md](../../AGENTS.md) — URDF import, the harness, "visual-only prop".
- [urdf-import-debugging.md](urdf-import-debugging.md) — when an import misbehaves.
- [WORLD_RECIPE.md](../WORLD_RECIPE.md) — canonical world lighting.
