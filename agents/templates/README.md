# OmniSim specialist agents

Example agents that drive OmniSim robots through OmniLink. Each
is an end-to-end illustration of how to build a purpose-built OmniLink
agent on top of OmniSim's bridge HTTP surface — copy any of them, edit
the profile + tool implementations, point at your own robot bridges,
and you have a working OmniLink-driven agent.

| Agent | Role | Targets |
|---|---|---|
| [omnisim_roomba](roomba/) | Waypoint patrol + return-to-dock mobile navigator | Any wheeled base (Husky/Jackal/TB3/Rosbot) |
| [husky_swarm](husky_swarm/) | 34-tool swarm coordinator (parallel execution, persistent memory, routines) | `omnilink_husky_swarm.wbt` (4× Husky) |
| [mission_control](mission_control/) | Free-form fleet dispatcher across a 12-zone campus | `omnilink_mission_control.wbt` (6× Husky) |

## The profile-only pattern

**Profile-only specialist** (Roomba): just a `profile.json`
with a specialised system prompt that reuses an existing bridge's
`/tool` endpoint. The `register.py` script pushes / updates the
profile via OmniLink's API. No Python process to keep running.

These are single-bridge behavioural tuning — point the profile at one
bridge and chat. For cross-bridge coordination across multiple robots,
see the production orchestrators (e.g. `mission_captain`).

## Common environment

| Variable | Notes |
|---|---|
| `OMNI_KEY` | Required for every agent. Get one from https://omnilink-agents.com/account. |
| `OMNILINK_BASE_URL` | Override platform URL (default `https://www.omnilink-agents.com`). |

Each agent also reads its own `*_BRIDGE` / `*_BRIDGES` env to find the
bridge(s) it talks to. See each agent's README for details.

## Sim-to-real

None of these agents have any Webots dependency. The bridges they
talk to do — replace any bridge URL with a real robot's HTTP surface
(speaking the same `/list_robots`, `/prompt`, `/tool`,
`/get_robot_state`, `/stop_robot` schema) and the agent drives the
real robot with no other changes. That's the whole point of the
sim-to-real story: same prompts, same tools, same agent code.
