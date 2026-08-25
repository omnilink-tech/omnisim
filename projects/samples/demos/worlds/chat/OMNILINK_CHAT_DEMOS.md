# OmniLink chat demos — index

This folder contains 13 `omnilink_<robot>.wbt` chat-driven demos (one
per URDF robot in the repo), plus the 3-arm `omnilink_multi_arm.omniworld`.
Each opens a robot in
a small arena with an **OmniLink robot console** side menu attached:
right-click the robot in the 3D view → *Show Robot Window*, then type
prompts like `home`, `wave hello`, `forward 1 meter`,
`turn left 90 degrees`, `stop`.

With an Omni Key set the panel grows a **mic button** (click to talk,
click to send) and speaks the reply back (Chirp3-HD TTS). Gemini
(`g1-engine`) works on any Omni Key. To use Claude (`g4-engine`) you
need an Anthropic key added on the OmniLink *API & Keys* page,
otherwise the platform returns `402 BYOK_REQUIRED`. Without a key it
still runs as the terse offline command console.

Full beginner walkthrough (env vars, OmniLink integration, internals,
adding new robots): [`docs/guide/omnilink-chat-demos.md`](../../../../../docs/guide/omnilink-chat-demos.md).

## The demos

### Arms

| World | Robot | Try saying |
|---|---|---|
| `omnilink_ur5e.omniworld`  | Universal Robots UR5e (**+ IK**) | "home", "wave hello", "move to 0.4 0.2 0.4", "joint 3 to 1.5", "stop" |
| `omnilink_ur3e.omniworld`  | Universal Robots UR3e            | "home", "wave hello", "joint 3 to 1.5", "stop" |
| `omnilink_ur10e.omniworld` | Universal Robots UR10e           | "home", "wave hello", "joint N to V", "stop" |
| `omnilink_multi_arm.omniworld` | 3 x UR5e (ports 8765/8766/8767) | one console per arm; drive them independently |

The **UR5e is the only arm with a pre-baked DLS IK chain**, so it is the one
that accepts a Cartesian `move to X Y Z` / `set_tcp_target`. The others take
joint commands and presets.

### Mobile bases

| World | Robot | Try saying |
|---|---|---|
| `omnilink_tb3_burger.omniworld`    | TurtleBot3 Burger    | "forward 1 m", "turn left 90 degrees", "spin", "stop" |
| `omnilink_tb3_waffle.omniworld`    | TurtleBot3 Waffle    | same |
| `omnilink_tb3_waffle_pi.omniworld` | TurtleBot3 Waffle Pi | same |
| `omnilink_husky.omniworld`         | Clearpath Husky      | "forward 1 m", "turn around", "circle" |
| `omnilink_jackal.omniworld`        | Clearpath Jackal     | same |
| `omnilink_rosbot.omniworld`        | Husarion Rosbot      | same |
| `omnilink_rosbot_xl.omniworld`     | Husarion Rosbot XL   | same |

### Quadruped

| World | Robot | Try saying |
|---|---|---|
| `omnilink_omniquad.omniworld` | OmniQuad (poses only) | "stand", "sit", "wave hello", "stop" |

### Aerial

| World | Robot | Try saying |
|---|---|---|
| `omnilink_mavic.omniworld` | DJI Mavic 2 Pro | "takeoff", "forward 1 m", "up 2 m", "turn right 90 degrees", "land" |

## Three modes

- **Offline (default).** No setup. A small regex inside the bridge
  matches prompts to actions. Works for the canonical phrasings shown
  above.
- **Local Ollama (any phrasing, free, no account).** Run an
  [Ollama](https://ollama.com) server locally and launch with no
  `OMNI_KEY` — the bridge auto-upgrades to a local LLM with tool
  calling. Setup + measured model comparison:
  [LOCAL_OLLAMA.md](LOCAL_OLLAMA.md).
- **OmniLink (any phrasing).** Set `OMNI_KEY=olink_...` before launching
  OmniSim and the same prompt goes through OmniLink to a real LLM
  (Gemini by default; configurable via `OMNILINK_ENGINE`). Paraphrases,
  state queries, multi-step plans all work — plus voice, memory, and
  web-UI control.

See [the full guide](../../../../../docs/guide/omnilink-chat-demos.md)
for env vars, troubleshooting, the bridge HTTP surface, and how to
add a new robot (~30 lines of code).
