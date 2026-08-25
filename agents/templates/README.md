# OmniSim specialist agents

Example agents that drive OmniSim robots through OmniLink. Each
is an end-to-end illustration of how to build a purpose-built OmniLink
agent on top of OmniSim's bridge HTTP surface — copy any of them, edit
the profile + tool implementations, point at your own robot bridges,
and you have a working OmniLink-driven agent.

| Agent | Role | Targets |
|---|---|---|
| [omnisim_roomba](roomba/) | Waypoint patrol + return-to-dock mobile navigator | Any wheeled base (Husky/Jackal/TB3/Rosbot) |

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
| `OMNI_KEY` | Required for every agent. Get one at https://www.omnilink-agents.com/key, or run `python -m omnisim key`. |
| `OMNILINK_BASE_URL` | Override platform URL (default `https://www.omnilink-agents.com`). |

Each agent also reads its own `*_BRIDGE` / `*_BRIDGES` env to find the
bridge(s) it talks to. See each agent's README for details.

## Sim-to-real

None of these agents have any Webots dependency. The bridges they
talk to do — replace any bridge URL with a real robot's HTTP surface
(speaking the same `/list_robots`, `/prompt`, `/tool`,
`/get_robot_state`, `/stop_robot` schema) and the agent drives the
a separately validated real-robot bridge without changing the agent's prompts
or tool contract. Hardware safety, calibration, and dynamics remain separate
integration work.
