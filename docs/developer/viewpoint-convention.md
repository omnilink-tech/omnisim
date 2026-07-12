# Viewpoint convention — the default world camera

**TL;DR.** Every world should open *looking at its subject*. The default framing
is an **angled "hero" view** (elevated 3/4) that shows a robot's full structure.
Navigation / overview worlds may opt into a **top-down** view. The framing math
lives in one place — [`omniworld.viewpoint`](../../src/python/omniworld/viewpoint.py) —
and is shared by the world generators and the retrofit tool. New worlds get a
good camera for free; you should never have to hand-tune `position` and
`orientation` by trial and error again.

## The problem this solves

A world's opening camera is its [`Viewpoint`](../../resources/nodes/Viewpoint.wrl)
node: an eye `position` (xyz, metres) and an `orientation` in axis-angle form
(`x y z angle`, angle in radians). OmniSim worlds are **Z-up**
(`coordinateSystem "NUE"`).

When a world has no `Viewpoint`, the engine falls back to a fixed
`position -10 0 0` looking down +X. Robots are almost never sitting there, so
they open off-screen or as a tiny speck, and every hand-authored world drifted
to its own ad-hoc camera — which is why opening worlds felt like landing in a
random spot and having to fly around to find the robot.

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

# Frame a robot by its DEF or name "..." field, using its class preset:
python scripts/dev/set_viewpoint.py projects/policies/research/worlds/spot_walk_deploy.wbt \
    --subject spot --class quadruped

# Explicit centre + radius, top-down (overview / nav):
python scripts/dev/set_viewpoint.py .../showcase/husky_fleet_arena.wbt \
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
is the re-runnable list of hand-authored presentation worlds (RL deploys, the
chat "Talk to the Robot" demos, showcase arenas). Re-apply the whole set with:

```bash
PYTHONPATH=src/python python scripts/dev/set_viewpoint.py --batch scripts/dev/viewpoint_targets.json
```

Add a world by appending an entry; it is the canonical record of which worlds
were framed and how.

## What is intentionally left alone

- **Follow / cinematic cameras** keep their `follow`/`followType`; only the
  opening frame is set (so the world looks right before you press play).
- **Deliberately composed scenes** — e.g. the `earth_night` / `mars_night`
  sky showcases, whose low-horizon framing is the whole point — are not
  auto-reframed. The convention is "frame the subject," and for those worlds the
  sky *is* the subject.
