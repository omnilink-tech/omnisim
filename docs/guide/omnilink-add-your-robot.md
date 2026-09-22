# Add your own robot to the OmniLink demos

Four-step recipe to get a new URDF robot answering chat prompts.
Total work: about 50 lines of new code and 30 lines of new world file.

Prerequisites:
- the robot's URDF (and any meshes it references) somewhere under [`projects/robots/`](../../projects/robots/)
- a clear answer to one question — which **bridge class** does your robot belong to? It determines which bridge you'll piggy-back on. (Quadrupeds have a config-driven bridge too — see the table.)

The whole recipe assumes you've read [the beginner guide](omnilink-chat-demos.md) and have one of the existing demos working.

---

## Step 1 — pick the bridge class

| Robot class | Bridge | Existing examples | Configs file |
|---|---|---|---|
| Mobile base (wheels) | `omnilink_mobile_bridge` | Husky, Jackal, TB3 family, Rosbot/XL | [`_mobile_configs.py`](../../projects/samples/demos/controllers/omnilink_mobile_bridge/_mobile_configs.py) |
| Arm / manipulator | `omnilink_arm_bridge` | UR3e, UR5e, UR10e | [`_arm_configs.py`](../../projects/samples/demos/controllers/omnilink_arm_bridge/_arm_configs.py) |
| Quadruped (legged or wheeled-legged) | `omnilink_quadruped_bridge` | OmniQuad, Deep Robotics Lite3 / X30 / M20 / M20S / M20 Piper | [`_quadruped_configs.py`](../../projects/samples/demos/controllers/omnilink_quadruped_bridge/_quadruped_configs.py) — leg motor names, stand / sit poses, per-leg sign conventions, `walk` = `gait` / `wheels` / `march`, `body_lock` |
| Aerial | `mavic_omnilink_bridge` | DJI Mavic 2 Pro | hard-coded |

Bridge classes are not interchangeable. A mobile config cannot make an arm or
aerial robot work: each class has different state, units, safety checks, and
motion verbs. If your class is not represented, implement a bridge against
[`BridgeBase`](../../packages/omnisim-bridges/) and add conformance tests before
building the chat world — and read [the gate section](#the-safety-gate-and-what-your-robot-has-to-do-about-it)
first, because a new bridge class is the one case where you have to wire the
veto yourself.

The class name is not only a routing label: it is the **surface** the parser
and the gate are told about (`mobile`, `arm`, `quadruped`, `drone`). A rule
only fires on a surface that can do it, so `sit` never reaches a wheeled tug
and `drive forward` never reaches an arm, and the magnitude rail for a tool
two classes share is picked from it. ⚠️ Pass it in the exact spelling above.
`interpret()` used to accept anything, and an unrecognised string matched no
rule, so every utterance fell through to conversation with no frames —
indistinguishable from a parser that could not understand the sentence. It
cost a published result: `interpbench` was run with `surface="MOBILE"`,
uppercase, and reported the deterministic parser at 68.3% with 19 missed
commands against the model's 91.7%, concluding the model was the better
interpreter by 23 points. With the case corrected the parser scores 90.0% with
**one** missed command and the gap is 1.7 points. It now raises
`interpret.UnknownSurface` instead.

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
    "yaw_rate_gain": 1.0,               # MEASURE THIS -- see below
}

MOBILE_CONFIGS = {
    # ... existing entries ...
    "my_rover": MY_ROVER,
}
```

That's it. The bridge's generic differential / skid-steer driver picks it up automatically, and so do the structural parser and the safety gate — you inherit the whole `mobile` surface without writing a rule or a tool. The configurable fields:

- **`layout`** — which wheel-motor naming convention the URDF importer produced. The existing 3 are at the bottom of `_mobile_configs.py` (`WHEEL_MOTORS`): `4wheel_full` (Husky / Jackal naming), `4wheel_fl` (Rosbot family), `2wheel` (TurtleBot3 family). Add a new one if your URDF uses an unfamiliar naming pattern.
- **`wheel_radius_m`** and **`half_track_m`** — the geometry the bridge inverts to turn a `(linear, angular)` command into per-wheel speeds. Half-track is the wheel separation divided by two.
- **`max_wheel_speed_radps`** — ceiling on rad/s for each wheel. The bridge clamps every command against it.
- **`cruise_frac`** — fraction of the ceiling used as the default forward speed for `drive_forward`.
- **`spin_speed`** — rad/s used by the "spin in place" intent preset. It is taken as `min(spin_speed, max_angular_rad_s)`, so it only binds if it is below the measured ceiling — and it feeds three things, not one: the `spin` verb, the `circle` verb's curvature (`set_velocity(cruise*0.6, spin_speed*0.6)`, i.e. a turn radius of `cruise_frac*max_linear / spin_speed`), and the magnitude of each `turn` pulse. Pick it against all three: the shipped values are 0.6–1.0 rad/s, which give 1.1–1.8 revolutions inside the 12 s `set_velocity` expiry. ⚠️ Check the radius `circle` implies — on a base that is slow but nimble it can fall below `half_track_m`, at which point the inner wheel reverses and "circle" is a pirouette (measured: the TurtleBot3 Waffle traces 0.097 m against a 0.144 m half-track).
- **`yaw_rate_gain`** — **the one field here that is not geometry, and the one you have to measure.** It is the fraction of the ideal kinematic yaw ceiling your base actually holds at full command, and it sets `max_angular_rad_s` — the ceiling published to callers, the clamp on `set_velocity`, and whether `turn` accepts a rotation or refuses it. **Omitting it defaults to 1.0, i.e. "assume the ideal kinematics", which for a four-wheel skid-steer is about 2x optimistic**: an agent then plans turns your base cannot finish in the time it budgeted. Measured values for the shipped bases run from 0.49 (Clearpath Jackal) to 0.96 (TurtleBot3 Waffle) — a two-wheel differential drive pivots about its own axle and scrubs almost nothing, while a four-wheel skid-steer has to drag all four tyres sideways and loses about half.

  To measure it, run your world with the servo off and the gain forced to 1.0 so the bridge commands the raw kinematic differential:

  ```bash
  OMNISIM_MOBILE_YAW_SERVO=0 OMNISIM_MOBILE_YAW_GAIN=1.0       python -m omnisim run-headless <your world> --duration 60
  ```

  Then hold a pure yaw command at your base's kinematic ceiling
  (`max_wheel_speed_radps * wheel_radius_m / half_track_m`) with
  `POST /set_velocity {"linear": 0, "angular": <ceiling>}`, and divide the
  settled yaw rate you measure by that ceiling. Set
  `OMNISIM_MOBILE_TRACE_PATH=<file>` to get a per-tick JSONL trace with
  `pose.yaw_rad` and `sim_time_s` so the rate is differenced in sim time.
  Sweep a few rates below the ceiling too: the shipped skid-steers droop
  20-35% at very low commands, and the bridge's yaw servo is what closes
  that. The full recipe and the current measured table live in the
  `_mobile_configs.py` module docstring.

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

The chat panel needs an OmniKey — on every plan, including Free. Set it
before you launch:

```bash
export OMNI_KEY=olink_...            # PowerShell: $env:OMNI_KEY = 'olink_...'
```

With no key, `POST /prompt` answers `401 omnikey_required`, nothing
actuates, and the panel shows a setup link instead of a transcript. There
is no keyless or local-command mode to fall into, and a connection that
fails later is an error rather than a downgrade. The direct REST controls
below (`/drive_forward`, `/stop_robot`) are ordinary simulator controls
and do not need a key — they are also ungated, which is the trade.

Then right-click the robot in the 3D view → **Show Robot Window** → type `forward 1 m` / `turn left 90 degrees` / `spin` / `stop`. Tool-call lines appear in the transcript; the robot moves.

Try the trap sentence too — `how many times have you had to stop on this run?`
should be **answered**, not obeyed. That is the one behaviour worth
re-checking on every new robot, because it is the failure the structural
parser exists to remove, and `scripts/dev/smoke_chat_demos.py` checks exactly
it across the gallery.

Behind that panel the deterministic parser interprets the sentence first
and a model is called only for what the parser declines. Both need the
key: the access check runs before the parser sees the sentence. Neither
path skips a safety check — the gate vets whichever one produced the
frames (see below).

---

## The safety gate, and what your robot has to do about it

Between a typed tool call and a motor sits
[`gate.check(utterance, frames, surface=...)`](../../packages/omnisim-bridges/src/omnisim_bridges/gate.py):
a deterministic veto that does not look at who produced the frame. It refuses
an interrogative that produced motion, a prohibition that produced the thing
it prohibits, a self-negating order, a magnitude outside a sanity rail, a
direction word that disagrees with its sign, args that do not match the
tool's declared schema, an unresolved referent (`it`, `there`), and a
confirm-required tool with no authorization token.

### If you piggy-backed on an existing bridge

Nothing. Your robot is gated the moment it boots, on every path the bridge
you copied is gated on: the parser, the relay's model dispatch, and
`POST /tool` in `bridge_base` and in the bridge's own handler. (Each bridge
used to keep a keyword ladder underneath the parser as a no-key fall-through.
Those are **retired and deleted** — there is no command path that answers
without an OmniKey, and none is coming back.) Go read
[`GATE_COVERAGE.md`](../../packages/omnisim-bridges/GATE_COVERAGE.md) so you
know what the gate does *not* cover, and then get on with your robot.

### If you wrote a new bridge class

Two things, both one line.

**1. Your tools are registered with the gate automatically, from their own
schema — if you construct them as `Tool`s and hand them to a relay.**
`OmniLinkRelay` calls `gate.register_tools(tools, surface=...)` at start-up
and builds `SPECS` from each `Tool.parameters` JSON schema.

```python
from omnisim_bridges.tool import Tool

Tool(
    name="set_gripper_width",
    description="Open or close the gripper to a width in metres.",
    parameters={
        "type": "object",
        "properties": {"width": {"type": "number"}},
        "required": ["width"],
    },
    dispatch=self.act_set_gripper_width,
    physical=True,          # <- declare it. See below.
    surface="arm",
)
```

⚠️ **Declare `physical=`.** Whether a tool can move a robot used to be decided
by substring-matching its *name* against a tuple that was copy-pasted into
five files, and that heuristic is wrong in both directions: `get_drive_status`
contains `drive` and is read-only, `activate_sprayer` contains nothing and is
not. The heuristic survives as the fallback for a tool nobody declared, so
adding the field changed no behaviour the day it landed — but it is the only
thing that will get *your* verb right. A tool that declares `physical=False`
stays read-only even if its name contains `stop`; the reverse is not
symmetric, since nothing registered may downgrade a tool the hand-written
table calls physical.

⚠️ **A registered physical tool is `GUARDED`, never `SAFE`.** `SAFE` is the
de-escalation carve-out — it exempts a tool from the intent rules so that
*"stop moving"* and *"do not move"* still halt the robot — and it is only ever
granted by hand, in `gate.py`, to tools that *reduce* what the robot is doing.
Inheriting it from a registry would let a bridge name a tool `stop_and_fling`
into the exemption.

**2. Call the canonical rejector from your own `/tool` handler.**

```python
from omnisim_bridges import gate

rej = gate.reject_toolcall(tool_name, args, utterance, surface="arm")
if rej is not None:
    return self._json(400, error_envelope("refused_by_gate", rej,
                                          {"tool": tool_name}))
```

Pass one of the four surface names the parser knows (`mobile`, `arm`,
`quadruped`, `drone`). A name of your own is *accepted* but has no rails
declared for it, so every vertical magnitude is judged on the ground /
body-shift rail (1.0 m) — it does **not** fall back to the tool's recorded
surface, because the caller's surface always wins once it is non-empty.

Omitting the argument is the different case, and it is not "the strictest
rail" either: with no caller surface the gate uses the surface recorded on
the tool's spec **when exactly one robot class registered that tool**, and
only discards it for the ground rail when two or more did. Declare your
surface — the fallback is a guess, the argument is a fact.

`reject_toolcall()` is the one implementation. It used to be copy-pasted into
five files — `bridge_base`, `relay`, and the mobile / arm / quadruped
controllers — which is how a commit came to add a gate to `bridge_base` and
miss all four bridges: a gated base still let `distance: 300` through a
bridge's own handler and hung the HTTP call for 90 s while the robot drove
it. **A safety check that has to be applied in five places is a safety check
that will be applied in four.**

Wrap it so it **fails closed on physical tools**: if `gate` cannot be imported
or raises, refuse motion and let read-only tools answer anyway. Refusing to
report a position because a *motion* check is unavailable is a self-inflicted
outage.

Note that `reject_toolcall()` drops `unknown_tool`. The gate does not police
tool *existence* — your registry already did, and overruling it would refuse
verbs named at runtime, like the arm's learned skills. That is exactly why
registration matters: **a tool the gate has never heard of is not merely
unknown, it is unchecked.** Until 2026-09-21 that included ten of the arm
bridge's nineteen tools, among them `grasp`, `set_tcp_target` and
`set_gripper_width`.

### Rails are per quantity, and per surface

The magnitude limits live in `gate.py`, hand-declared, and they are rails
("nobody meant this"), not bounds:

| quantity | rail | constant |
|---|---|---|
| distance | 50 m | `MAX_DISTANCE_M` |
| angle | 4 full turns | `MAX_ANGLE_RAD` |
| speed | 5 m/s | `MAX_SPEED_MPS` |
| yaw rate | 6 rad/s | `MAX_YAW_RATE_RPS` |
| altitude | 120 m | `MAX_ALTITUDE_M` |
| quadruped body shift | 1 m | `MAX_BODY_SHIFT_M` |

They are keyed on the **quantity**, not the argument name: railing only the
literal argument `speed` had let `set_velocity {v: 40}` — the actual velocity
channel — through unrailed. And where two classes share a tool the rail is
chosen by the caller's `surface=`: `move_body{vertical}` is a climb on a drone
and a body shift on a quadruped, and 40 m is routine for one and absurd for
the other. Before the split, one global rail refused **every real drone climb
above 1 m**.

⚠️ **Do not move a rail into your bridge.** OmniLink ships a TypeScript port
of `gate.py`, and the only thing binding the two implementations is the
generated fixture
[`tests/benchmarks/gate_parity/cases.json`](../../tests/benchmarks/gate_parity/cases.json).
A safety constant that moves into a bridge is a safety constant the fixture
stops freezing. Schemas come from the registry; limits do not.

### What the gate will not do for your robot

It is a sanity rail, not a world model. It does not know your arena, and
**nothing in the stack limits a robot to the floor it is standing on** —
`act_drive_forward` takes a distance and drives it. A world-aware clamp is
still your bridge's job. And the direct REST routes your bridge exposes
(`/drive_forward`, `/set_velocity`, the gripper and joint routes, any verb
promoted to an endpoint at runtime) reach an actuator with nothing in between;
[`GATE_COVERAGE.md`](../../packages/omnisim-bridges/GATE_COVERAGE.md) is the
authoritative list of what is and is not vetted.

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
- The **structural parser** on the `mobile` surface: question form, clause structure and consumption settled before any keyword rule fires, so *"how much charge is left in your battery?"* is answered rather than spun.
- The **safety gate** on every path the mobile bridge is gated on — the parser, the relay's model dispatch, and `POST /tool` in both handlers. Your robot's direct REST routes are still ungated, as they are on every bridge.
- The **connected OmniLink chat surface**, on the same terms as every other robot here: an OmniKey on every plan including Free, the parser answering what it can and a model answering the rest, and the gate vetting whichever one produced the frames. With no key the chat surface returns an error and nothing moves — it does not degrade into a local command handler.

When you're ready, the same tool surface points at a real robot — see [the sim-to-real walkthrough](omnilink-sim-to-real.md).
