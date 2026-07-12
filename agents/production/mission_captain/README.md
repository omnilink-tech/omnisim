# Mission Captain

**Mission.** Translate operator goals into a sequence of delegations to specialist sub-agents. Doesn't drive robots itself. Decomposes, routes, aggregates, reports.

This is the **multi-agent fabric** demo for the OmniSim ↔ OmniLink integration. The captain holds the operator's natural-language goal and turns it into discrete sub-missions, each handed off to whichever specialist agent owns that capability.

## Specialists

| name | what it does | reachable at |
|---|---|---|
| `Husky Maze` | Drives the Clearpath Husky in maze worlds. BFS / lidar wall-follow / vision-based marker hunting. | `127.0.0.1:51517` |

Each registered specialist must be running its own runner (which pushes its profile to OmniLink and serves a local tool-callback HTTP server) before the captain can delegate to it. The captain discovers them via `list_agents` and probes their `/status` endpoints via `query_agent_status`. Registering more specialists is a `SPECIALIST_REGISTRY` edit — see "Why this matters" below.

## Architecture

```
operator goal  ─►  Mission Captain (this agent)
                       │
                       │ delegate_to_agent("Husky Maze", "drive to (5,3)")
                       ▼
                  Husky Maze runner  ─► husky_omnilink_bridge ─► Husky in OmniSim
                       │
                       │ delegate_to_agent(<specialist>, "<sub-mission>")
                       ▼
                  specialist runner  ─► its bridge ─► its robot in OmniSim
```

OmniLink platform-level delegation can't reach loopback URLs from the internet, so the captain runs the sub-chat-loop **locally** via its `delegate_to_agent` tool. The sub-agent's profile is pushed to OmniLink (so it's a first-class agent the operator can also chat with directly), but the captain's delegation runs the conversation locally and dispatches every sub-tool call against the sub-agent's runner.

## Files

- [`profile.json`](profile.json) — OmniLink profile + standing orders, engine `g1-engine`.
- [`prompts/system.md`](prompts/system.md) — system instruction: decompose, delegate, aggregate, report, save the pattern.
- [`tools/`](tools/) — 10 tools: `delegate_to_agent`, `query_agent_status`, `list_agents`, `complete_mission`, plus `recall` / `save_local_memory` / `search_local_memory` / `list_local_memories` / `forget_local_memory` / `search_knowledge`.
- [`mission_captain_agent.py`](mission_captain_agent.py) — runner. Pushes profile, starts tool server on `127.0.0.1:51518`, exposes `/status` + `/activity`.
- [`scripts/chat_drive.py`](scripts/chat_drive.py) — programmatic chat driver (sends operator missions, runs the captain's tool loop locally).
- [`long_term_memory/`](long_term_memory/) — saved mission patterns (specialist sequence + outcome) so future similar missions skip rediscovery.

## Run

```bash
# Specialist runners (each in its own shell):
python agents/production/husky_maze/husky_maze_agent.py

# Then the captain:
set OMNI_KEY=olink_YOUR_KEY
python agents/production/mission_captain/mission_captain_agent.py

# Then send a mission via chat_drive:
python agents/production/mission_captain/scripts/chat_drive.py \
    "Drive the husky to cell (3, 5), then come back to start (0, 10)."
```

The captain plans, delegates each leg to `Husky Maze`, waits for completion, calls its own `complete_mission` to close the operator goal.

## Why this matters

Three reasons:

1. **OmniLink as a fabric, not a single agent.** The platform's value isn't only "one agent runs forever"; it's "many agents compose, each owning a narrow specialty, with audit trails between them".
2. **Stable interfaces.** The captain never needs to know how Husky Maze drives, only that it accepts natural-language sub-missions and reports back. Husky Maze can swap its internals freely without breaking the captain.
3. **Cross-team scaling.** Different teams can own different specialists. The captain glues them together. Anyone can add a new specialist by registering its `/status` + `/tool` URLs in `tools/_base.py`'s `SPECIALIST_REGISTRY`.

## Limitations

- **Latency stacks.** Captain chat tick (~3-5 s) + sub-agent's full mission (minutes). A 3-leg mission is ~15+ minutes wall-clock.
- **Sequential by default.** The current `delegate_to_agent` is blocking. A truly parallel captain would issue concurrent delegations to independent specialists; the OmniLink billing + chat-loop machinery would need extending for that.
- **No mid-leg interruption.** Once delegated, a leg runs to completion or its own max-turns. The operator can't inject mid-leg guidance.
- **Specialists must be on localhost.** No remote specialists today; the registry is loopback-only.
