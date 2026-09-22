# OmniLink chat demos — index

**21 chat demos: 20 `omnilink_<robot>.omniworld` worlds plus
`omniarm6_talk.omniworld`, the one chat world whose name does not follow the
pattern.** Broadly one world per URDF robot in the repo, with three exceptions in
the arm set: the 3-arm `omnilink_multi_arm.omniworld`, and the two gripper
variants of the OmniArm 6 (`_2f140` and `omniarm6_talk`'s 2F-85). Every one of the 21 is listed below — this is the complete index, and
the [launcher catalogue](../../controllers/omnilink_launcher/demos.json) and
[DEMOS.md §1](../../../../../DEMOS.md#1-chat-demos--single-robot-natural-language-console)
carry the same 21. `tests/test_demo_catalogue.py` fails if the three ever disagree.

`omnilink_husky_langsoak.omniworld` is the 22nd TRACKED `.omniworld` here and is
**not** one of the 21: it is the fixture the
[langsoak](../../../../../tests/benchmarks/langsoak/) language benchmark drives on
port 8775, not a demo.

Each demo opens a robot in a small arena with an **OmniLink robot console** side
menu attached: right-click the robot in the 3D view → *Show Robot Window*, then
type prompts like `home`, `wave hello`, `forward 1 meter`,
`turn left 90 degrees`, `stop`.

⚠️ **One chat demo at a time.** 20 of the 21 pin their bridge to port **8765**
(the exceptions: the Mavic on 6090, and `omnilink_multi_arm`, which numbers its
three consoles 8765/8766/8767). Launch a second one while the first is running and
its bridge cannot bind — and an orphaned bridge from a killed engine keeps the port
and answers for the next world, which reads as the new robot ignoring you. Stop the
running demo, and confirm 8765 is closed, before starting another.

Every OmniLink chat session requires an OmniKey and a connected model provider.
There is no keyless chat mode or automatic local-command fallback. Start with
the [Husky film and setup guide](../../../../../docs/guide/omnilink-chat-demos.md).
Voice availability depends on the configured connection; it is not included
merely by supplying an OmniKey.

## The demos

### Arms (7) — all on `omnilink_arm_bridge`

| World | Robot | Try saying |
|---|---|---|
| `omnilink_ur5e.omniworld`  | Universal Robots UR5e (**+ IK**) | "home", "wave hello", "move to 0.4 0.2 0.4", "joint 3 to 1.5", "stop" |
| `omnilink_ur3e.omniworld`  | Universal Robots UR3e            | "home", "wave hello", "joint 3 to 1.5", "stop" |
| `omnilink_ur10e.omniworld` | Universal Robots UR10e           | "home", "wave hello", "joint N to V", "stop" |
| `omnilink_multi_arm.omniworld` | 3 x UR5e (ports 8765/8766/8767) | one console per arm; drive them independently |
| `omnilink_omniarm6.omniworld` | OmniArm 6 6-axis cobot | "home", "wave hello", "joint 3 to 1.5", "stop" — holds pose under gravity |
| `omnilink_omniarm6_2f140.omniworld` | OmniArm 6 + 140 mm two-finger gripper | "pick up the block", "put it down", "open the gripper" — a **real friction hold**, not a weld: open mid-carry and the block falls |
| `omniarm6_talk.omniworld` | OmniArm 6 + Robotiq 2F-85, as "Ari" | right-click → *Talk to the Robot* for the in-sim chat card; "pick up the red cube", "wave". Launch helper: `run_omniarm6_talk.ps1` |

The **UR5e is the only arm with a pre-baked DLS IK chain**, so it is the one
that accepts a Cartesian `move to X Y Z` / `set_tcp_target`. The others take
joint commands and presets.

`omniarm6_talk` is the reason this index says "21 chat demos" rather than
"21 `omnilink_*` worlds": it predates the naming convention and kept its name.

### Mobile bases (7) — all on `omnilink_mobile_bridge`

| World | Robot | Try saying |
|---|---|---|
| `omnilink_tb3_burger.omniworld`    | TurtleBot3 Burger    | "forward 1 m", "turn left 90 degrees", "spin", "stop" |
| `omnilink_tb3_waffle.omniworld`    | TurtleBot3 Waffle    | same |
| `omnilink_tb3_waffle_pi.omniworld` | TurtleBot3 Waffle Pi | same |
| `omnilink_husky.omniworld`         | Clearpath Husky      | "forward 1 m", "turn around", "circle" |
| `omnilink_jackal.omniworld`        | Clearpath Jackal     | same |
| `omnilink_rosbot.omniworld`        | Husarion Rosbot      | same |
| `omnilink_rosbot_xl.omniworld`     | Husarion Rosbot XL   | same |

### Quadrupeds (6) — all on `omnilink_quadruped_bridge`

| World | Robot | Try saying |
|---|---|---|
| `omnilink_omniquad.omniworld` | OmniQuad (poses only) | "stand", "sit", "wave hello", "stop" |
| `omnilink_lite3.omniworld` | Deep Robotics Lite3 (stands on its own physics) | "stand", "sit", "wave hello", "walk" (legs cycle in place), "stop" |
| `omnilink_x30.omniworld` | Deep Robotics X30 | same as the Lite3 |
| `omnilink_m20.omniworld` | Deep Robotics M20 (wheeled-legged; the wheels are real physics) | "stand", "sit", "wave hello", "drive forward", "stop" |
| `omnilink_m20s.omniworld` | Deep Robotics M20S | same as the M20 |
| `omnilink_m20_piper.omniworld` | Deep Robotics M20 + AgileX Piper arm (base only; the arm holds its zero pose) | same as the M20 |

### Aerial (1) — `mavic_omnilink_bridge`

| World | Robot | Try saying |
|---|---|---|
| `omnilink_mavic.omniworld` | DJI Mavic 2 Pro (**port 6090**, not 8765) | "takeoff", "forward 1 m", "up 2 m", "turn right 90 degrees", "land" |

7 + 7 + 6 + 1 = **21**. Four bridge controllers serve them.

## Connection required

Use the [setup guide](../../../../../docs/guide/omnilink-chat-demos.md) to connect
your account and selected model. Model usage is billed separately. Direct
simulator controls and Stop remain available without an OmniKey.
