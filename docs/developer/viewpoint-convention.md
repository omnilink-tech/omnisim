# Viewpoint convention — the default world camera

**TL;DR.** Every world should open *looking at its subject*. The default framing
is an **angled "hero" view** (elevated 3/4) that shows a robot's full structure.
Navigation / overview worlds may opt into a **top-down** view. The framing math
lives in one place — [`omniworld.viewpoint`](../../src/python/omniworld/viewpoint.py) —
and is shared by the world generators and the retrofit tool. New worlds get a
good camera for free; you should never have to hand-tune `position` and
`orientation` by trial and error again. The convention is **enforced** by a
checker ([`omniworld.validation.viewpoint`](../../src/python/omniworld/validation/viewpoint.py))
that runs inside `omniworld validate` and as a pre-push gate — see
[Enforcement](#enforcement).

## The problem this solves

A world's opening camera is its [`Viewpoint`](../../resources/nodes/Viewpoint.wrl)
node: an eye `position` (xyz, metres) and an `orientation` in axis-angle form
(`x y z angle`, angle in radians). OmniSim worlds are **Z-up** — the
`WorldInfo.coordinateSystem` default `"ENU"`; `"NUE"` is the Y-up system and no
world in this tree selects it. The camera's own frame is **+X forward, +Y left,
+Z up**.

When a world has no `Viewpoint`, the engine falls back to a fixed
`position -10 0 0` looking down +X. Robots are almost never sitting there, so
they open off-screen or as a tiny speck, and every hand-authored world drifted
to its own ad-hoc camera — which is why opening worlds felt like landing in a
random omniquad and having to fly around to find the robot.

**The measured damage (2026-07-22 audit, 703 worlds).** 39% of worlds
containing a robot opened with that robot completely out of frame — 19% of
`projects/`, 28% of the user-facing demos under `projects/samples/demos/`. The
single biggest cause: **160 worlds share the literal string
`orientation -0.5773 0.5773 0.5773 2.0944`**, which decodes to forward
`(0, 0, -1)` — straight down. That is correct for an overhead camera, but it
was copy-pasted into worlds whose `position` was then moved to an oblique eye
point, so the camera stared at empty floor metres from the robot. Compliance
with this document was 6%, because nothing checked it.

The fix is not a single magic angle; it is **always frame the subject**. The
subject is the robot when there is one, otherwise the scene.

## The two framings

| Framing | When | What it looks like |
|---|---|---|
| **`hero`** (default) | Single robot, manipulation cell, any "show the robot" world | Elevated 3/4 view from the front-right, ~33° above horizontal, distance sized so the subject fills the frame. Shows the robot's full vertical structure. |
| **`top_down`** (opt-in) | Navigation maps, fleets, cities, large layouts | Straight overhead. Great for spatial layout; flattens a robot's silhouette, so only use it when the *layout* is the subject. |

The canonical hero direction is `HERO_DIRECTION = (0.62, -0.66, 0.58)` (the eye
sits along this vector from the subject). Distance is derived from the field of
view so the subject fills the frame at any scale:
`distance = radius / sin(fov/2) * margin`.

> **Top-down gotcha.** A straight overhead view is degenerate with the natural
> Z-up vector (the view direction is parallel to "up"). `top_down_view` uses
> `up = +X` so the world's forward axis points up on screen — handled for you;
> don't hand-roll it.

## How new worlds get a good camera (you usually do nothing)

- **`omniworld` generated worlds** (`distribution/generated_worlds/*`, anything
  from `omniworld generate`): the emitter
  ([`omniworld/emit/wbt.py`](../../src/python/omniworld/emit/wbt.py),
  `_frame_world`) computes the camera automatically — it frames the robot
  spawn(s) when the world has any, otherwise the terrain/arena extent.
- **Procedural generators** (`scripts/dev/gen_*.py`): import the helpers and
  build the block from your scene's center + radius:

  ```python
  from omniworld.viewpoint import viewpoint_block
  wbt.append(viewpoint_block(center=(0, 0, 0.5), radius=1.4, mode="hero"))
  ```

- **Hand-authored worlds**: don't eyeball `position`/`orientation`. Bake the
  convention with the retrofit tool below.

## Retrofitting / setting a hand-authored world's camera

[`scripts/dev/set_viewpoint.py`](../../scripts/dev/set_viewpoint.py) rewrites
**only** the `orientation`/`position` lines of a world's `Viewpoint` block (and
`fieldOfView` with `--fov`). Every other field — `follow`, `followType`, `near`,
`bloomThreshold`, `ambientOcclusionRadius` — is preserved, so follow / cinematic
cameras keep their behaviour and only the *initial* framing changes.

```bash
export PYTHONPATH=src/python

# Easiest: let the tool find the subject itself. --auto uses the same scene walk
# as the checker, so it resolves PROTO-instanced and nested/parented robots and
# frames them at their true WORLD position (not their local `translation`).
python scripts/dev/set_viewpoint.py <world> --auto --dry-run

# Frame a robot by its DEF or name "..." field, using its class preset:
python scripts/dev/set_viewpoint.py projects/policies/research/worlds/omniquad_walk_deploy.omniworld \
    --subject omniquad --class quadruped

# Explicit centre + radius, top-down (overview / nav):
python scripts/dev/set_viewpoint.py .../showcase/husky_fleet_arena.omniworld \
    --mode topdown --center 0 0 0.3 --radius 12

# Preview without writing:
python scripts/dev/set_viewpoint.py <world> --subject g1 --class humanoid --dry-run
```

### Subject-class presets

These set a sensible framing radius and a look-at height above the spawn so the
camera centres on the body, not the feet. Keep this table in sync with
`SUBJECT_PRESETS` in [`omniworld/viewpoint.py`](../../src/python/omniworld/viewpoint.py).

| `--class` | radius (m) | look-height (m) | mode |
|---|---|---|---|
| `arm` | 1.0 | 0.45 | hero |
| `humanoid` | 1.3 | 0.75 | hero |
| `quadruped` | 1.4 | 0.45 | hero |
| `mobile` | 1.2 | 0.25 | hero |
| `fleet` | scene extent | 0.30 | topdown |
| `scene` | scene extent | 0.50 | hero |

Override the preset radius any time with `--radius` (e.g. a tiny TurtleBot wants
`--radius 0.7`; an arm with an offset bin wants a larger radius to include the
workspace).

### The curated retrofit set

[`scripts/dev/viewpoint_targets.json`](../../scripts/dev/viewpoint_targets.json)
is the re-runnable list of hand-authored presentation worlds (RL deploys and
previews, the chat "Talk to the Robot" demos, the showcase/husky worlds, the
robot-combat arenas, the G1/OmniQuad/B2 policy deploy + probe worlds, and the
device sample worlds) — 159 entries today.
Re-apply the whole set with:

```bash
PYTHONPATH=src/python python scripts/dev/set_viewpoint.py --batch scripts/dev/viewpoint_targets.json
```

It is idempotent — a re-run on an unchanged tree reports `0 world(s) changed`.
Add a world by appending an entry (`{"world": ..., "auto": true}` for the common
case); it is the canonical record of which worlds were framed and how.

## Keeping the subject in frame *after* t=0 — `follow`

A good opening frame only fixes t=0. A robot that drives away leaves the frame
and never comes back, so a demo that looked right on load is pointing at empty
floor by t=10. The engine already ships the fix: the `Viewpoint` fields
`follow` (a Solid's **name**), `followType`
(`"Tracking Shot"` | `"Mounted Shot"` | `"Pan and Tilt Shot"`) and
`followSmoothness`. It is driven from the physics loop
([`OmSimulationWorld::step()`](../../src/omnisim/engine/OmSimulationWorld.cpp)
→ `OmViewpoint::updateFollowUp()`), **not** from the GUI, so it works headless.

Rules of thumb:

- **`follow` takes the robot's `name` field, not its DEF.**
  `OmViewpoint::startFollowUpFromField` calls
  `OmSolid::findSolidFromUniqueName`, which matches `solid->name()` over the
  world's top solids (nested solids use a `parent:child` path). A name that
  matches nothing fails **silently** — the camera simply never moves. Verify the
  string exists before you commit it.
- **Only add it where the subject actually moves.** An arm bolted to a table or
  a stand-still humanoid gains nothing, and a tracking camera on a static
  subject can drift on contact jitter.
- **Not for whole-arena overviews.** If the opening frame already contains the
  entire drivable area (the `husky_maze*` mazes span ±11 m and the topdown at
  z = 50 covers ±11.6 m vertically), the robot cannot leave the frame, and
  following it would push the arena *out* of frame instead.
- **Multi-robot worlds**: follow the designated hero robot, or leave it off.
  Following one of a fleet hides the rest.
- `followSmoothness` is a spring constant (0 = snap instantly, 1 = very laggy).
  Demo worlds in this tree use `0.3`; the schema default is `0.5`.

`set_viewpoint.py` never touches these fields, so retrofitting a camera and
setting `follow` are independent operations.

## Enforcement

The convention used to be advisory, which is why compliance was 6%. Three things
now check it, all sharing one implementation —
[`omniworld/validation/viewpoint.py`](../../src/python/omniworld/validation/viewpoint.py):

| Where | What it does |
|---|---|
| `omniworld validate <world>` | `viewpoint_framing` runs alongside `asset_locality` / `prop_overlap` / `spawn_reachability`. Fails the world when its **robot** is out of frame. |
| `.githooks/pre-push` | Checks only the `.wbt` files in the push, under `projects/` and `distribution/`. Sub-second. Prints the exact `set_viewpoint.py` command that fixes each failure. Bypass with `OMNISIM_SKIP_VIEWPOINT_GATE=1` (this gate) or `OMNISIM_SKIP_PUSH_CHECK=1` (all gates). |
| `python -m omniworld.validation.viewpoint` | The audit CLI. Defaults to `projects tests distribution`; `--json`, `--summary`, `--quiet`, `--fail-on broken`. A full-tree scan is ~14 s. |

### How the check decides

1. **Parse the `Viewpoint`** — `position`, `orientation`, `fieldOfView`. Missing
   fields get the schema defaults from `Viewpoint.wrl` (`-10 0 0`, identity),
   because that is what the engine will actually show.
2. **Find the subject(s)** — every `Robot` / `Supervisor` / `URDFRobot`,
   *including PROTO instances whose base node resolves to one of those* (a
   `Husky { }` is still a robot), with parent transforms accumulated so a nested
   robot lands at its true world position. Excluded: `OmniSimSunMarker`,
   `ConveyorBelt` / `Mirror` / traffic-light props, passive wrappers
   (`controller "<none>"`), and **device mounts** — a `Robot` whose `children`
   draw nothing because they are only a Camera/Lidar/GPS (`CAMBOT`, `BIN_CAM`,
   `CEILING_CAM`). A world with no robot falls back to framing the scene extent,
   which is **advisory only** and can never fail the gate.
3. **Frustum test.** `fieldOfView` is VRML's *larger-dimension* angle, so on a
   16:9 viewport the binding axis is the vertical one. The verdict is `broken`
   when the subject's bounding sphere is entirely outside the frustum (or is a
   sub-1.5% speck), `borderline` when it is small / clipped / overflowing, else
   `ok`. Only `broken` fails anything.

### What is intentionally left alone

- **Follow / cinematic cameras** keep their `follow`/`followType`; only the
  opening frame is set (so the world looks right before you press play).
- **Deliberately composed scenes** are exempt by path, in `EXEMPT_FRAGMENTS` in
  the checker (each entry carries its reason). Today:
  - `distribution/generated_worlds/earth_night.wbt` and `mars_night.wbt` — sky
    showcases whose low-horizon framing is the whole point. The convention is
    "frame the subject," and for those worlds the sky *is* the subject.
  - `projects/samples/demos/worlds/rendering/**` (19 worlds) — the wgpu / lidar
    render-oracle fixtures. Their camera is the test instrument; moving it
    invalidates the comparison.
- **`tests/**` worlds** are outside the pre-push gate. Many use a deliberately
  odd camera to exercise a specific code path, and reframing them risks changing
  what a test measures. The audit still reports them (`61.9%` of test worlds with
  a robot open with it out of frame) — treat that as a backlog, not a bug list.
