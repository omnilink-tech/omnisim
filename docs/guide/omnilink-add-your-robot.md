# Add your own robot to the OmniLink demos

Four-step recipe to get a new URDF robot answering chat prompts.
Total work: about 50 lines of new code and 30 lines of new world file.

Prerequisites:
- the robot's URDF (and any meshes it references) somewhere under [`projects/robots/`](../../projects/robots/)
- a clear answer to one question — which **bridge class** does your robot belong to? It determines which bridge you'll piggy-back on. (Quadrupeds also have a bridge, but it's OmniQuad-specific.)

The whole recipe assumes you've read [the beginner guide](omnilink-chat-demos.md) and have one of the existing demos working.

---

## Step 1 — pick the bridge class

| Robot class | Bridge | Existing examples | Configs file |
|---|---|---|---|
| Mobile base (wheels) | `omnilink_mobile_bridge` | Husky, Jackal, TB3 family, Rosbot/XL | [`_mobile_configs.py`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py) |
| Arm / manipulator | `omnilink_arm_bridge` | UR3e, UR5e, UR10e | [`_arm_configs.py`](../../projects/samples/demos/controllers/omnilink_arm_bridge/_arm_configs.py) |
| Quadruped | `omnilink_quadruped_bridge` | OmniQuad | hard-coded |
| Aerial | `mavic_omnilink_bridge` | DJI Mavic 2 Pro | hard-coded |

Bridge classes are not interchangeable. A mobile config cannot make an arm or
aerial robot work: each class has different state, units, safety checks, and
motion verbs. If your class is not represented, implement a bridge against
[`BridgeBase`](../../packages/omnisim-bridges/) and add conformance tests before
building the chat world.

This guide uses a **wheeled mobile base** as the example — the class with the fully generic, config-driven bridge, and the one a new URDF robot most often lands in.

---

## Step 2 — add a config entry

Open `_mobile_configs.py` and add a dict for your robot. Concrete example for a hypothetical 4-wheel skid-steer rover:

```python
# projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py

MY_ROVER = {
    "model": "My Rover",
    "layout": "4wheel_full",            # or "4wheel_fl" / "2wheel"
    "wheel_radius_m": 0.105,
    "half_track_m": 0.262,
    "max_wheel_speed_radps": 6.0,
    "cruise_frac": 0.55,
    "spin_speed": 0.8,
}

MOBILE_CONFIGS = {
    # ... existing entries ...
    "my_rover": MY_ROVER,
}
```

That's it. The bridge's generic differential / skid-steer driver + intent router picks it up automatically. The configurable fields:

- **`layout`** — which wheel-motor naming convention the URDF importer produced. The existing 3 are at the bottom of `_mobile_configs.py` (`WHEEL_MOTORS`): `4wheel_full` (Husky / Jackal naming), `4wheel_fl` (Rosbot family), `2wheel` (TurtleBot3 family). Add a new one if your URDF uses an unfamiliar naming pattern.
- **`wheel_radius_m`** and **`half_track_m`** — the geometry the bridge inverts to turn a `(linear, angular)` command into per-wheel speeds. Half-track is the wheel separation divided by two.
- **`max_wheel_speed_radps`** — ceiling on rad/s for each wheel. The bridge clamps every command against it.
- **`cruise_frac`** — fraction of the ceiling used as the default forward speed for `drive_forward`.
- **`spin_speed`** — rad/s used by the "spin in place" intent preset.

The URDF importer turns a `<joint name="foo">` into a motor named `foo_motor`; the bridge tries that first and falls back to the bare name. Which names it looks for is exactly what `layout` selects.

---

## Step 3 — copy a world file template

Pick the closest existing demo and copy it, e.g. for the rover we just configured:

```bash
cp projects/samples/demos/worlds/chat/omnilink_husky.omniworld \
   projects/samples/demos/worlds/chat/omnilink_my_rover.omniworld
```

Name the copy **`.omniworld`**. OmniSim reads `.wbt` forever — there are external
forks and old worlds that must keep working — but nothing in the tree writes one any
more, and the extension is a capability signal: `URDFRobot`, `Cloth`, the `omnisim://`
URL scheme and every `newton*` field are unloadable in Webots.

Edit four lines:

```vrml
DEF MY_ROVER URDFRobot {
  url "../../../../robots/acme/my_rover_description/urdf/my_rover.urdf"  # ← your URDF
  translation 0 0 0.2
  name "my_rover"                                                        # ← your id (matches the config key)
  supervisor TRUE
  controller "omnilink_mobile_bridge"
  controllerArgs [ "--robot" "my_rover" "--port" "8765" ]                # ← your id again
  window "omnilink_chat"
}
```

Give `translation` enough height that the wheels rest on the floor rather than starting interpenetrated with it.

---

## Step 4 — launch and verify

```bat
launch.bat projects\samples\demos\worlds\chat\omnilink_my_rover.omniworld
```

Sanity-check the bridge:

```bash
curl -X POST http://127.0.0.1:8765/list_robots
# → [{"id": "my_rover", "model": "My Rover", "capabilities": {...}}]

curl -X POST http://127.0.0.1:8765/get_robot_state
# → {"id": "my_rover", "x": ..., "y": ..., "yaw": ..., "mode": "idle", ...}
```

Then right-click the robot in the 3D view → **Show Robot Window** → type `forward 1 m` / `turn left 90 degrees` / `spin` / `stop`. Tool-call lines appear in the transcript; the robot moves.

Set `OMNI_KEY=olink_...` to upgrade from the regex router to the live OmniLink agent — no other code change.

---

## Common pitfalls

- **The robot doesn't move, but the tool call fires.** The `layout` doesn't match the motor names the URDF importer actually produced. Check the OmniSim console for the bridge's motor-lookup warnings and compare against `WHEEL_MOTORS`.
- **A fixed-joint child part drifts off the robot.** A fixed-joint child link with collision geometry got a synthetic Physics block. This is fixed in the URDF importer (`emitLinkPhysics` with `allowSyntheticPhysics=false` for fixed-joint children) — make sure your `omnisim-bin` is built from a commit that includes that fix.
- **`/list_robots` returns a different robot id than what's in your config.** The world's `controllerArgs ["--robot" "x"]` value must match the dict key in `_mobile_configs.py`.
- **"Show Robot Window" missing from the context menu.** Left-click the robot first (which selects the root URDFRobot in the scene tree), *then* right-click. The 3D viewport's right-click selects the part under the cursor by default.
- **Chat panel opens but is light-themed, not the OmniLink dark panel.** The plugin isn't in the project's plugins directory. Copy `projects/samples/demos/plugins/robot_windows/omnilink_chat/` if your demo lives elsewhere; the path is per-project for custom robot windows.

---

## What you get for free

By piggy-backing on `omnilink_mobile_bridge`, your new robot inherits:

- The **OmniLink chat panel** (right-click → Show Robot Window).
- The **right-side dock Chat tab** (talks to the same bridge HTTP).
- The **HTTP surface on port 8765** that matches the Axis bridge contract — so OmniLink's Axis agent (the first-party `axis` agent in the OmniLink repo) drives your robot with zero new code on the agent side.
- Both **offline mode** (regex intent router) and **OmniLink mode** (set `OMNI_KEY`, real LLM picks tools).

When you're ready, the same tool surface points at a real robot — see [the sim-to-real walkthrough](omnilink-sim-to-real.md).
