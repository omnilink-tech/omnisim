# agents/ — OmniLink agents that drive OmniSim

Every OmniLink agent in the repo lives under this tree. Three slots, organized by how much code you bring:

| Slot | Subdir | What lives here | Pattern |
|---|---|---|---|
| **Templates** | [`templates/`](templates/) | Profile-only specialist starters that reuse an existing bridge | `profile.json` + `register.py` |
| **Production** | [`production/`](production/) | Full agents with their own runner, tools, knowledge, long-term memory | `*_agent.py` + `profile.json` + `tools/` + `prompts/` + `knowledge/` |
| **Bridges** | [`bridges/`](bridges/) | Sim-to-real bridge stubs (no Webots, no OmniSim) | `*_bridge_stub.py` |

The bridge HTTP surface (`/list_robots`, `/prompt`, `/tool`, `/get_robot_state`, `/stop_robot`) is identical across the three slots, so the same agent runs against a sim bridge or a real-robot bridge with no code changes.

Shared utility code lives in [`production/_lib/`](production/_lib/) (bridge base classes, runner base, damage helpers, usage logging) and is imported by the production agents.

## Start a new agent

Use the scaffold instead of copying a production agent by hand:

```bash
python -m omnisim agent new inspection_rover --robot-class mobile
export OMNI_KEY="olink_..."
python -m omnisim run-agent inspection_rover
```

Choose `mobile`, `arm`, `quadruped`, or `flying`. The generated directory has a
profile, manifest, runnable callback server, validated tool schemas, and run
instructions. The full decision guide is
[`docs/guide/omnilink-build-agent.md`](../docs/guide/omnilink-build-agent.md).

## Quick navigation

### Templates *(profile-only starters)*

| Agent | Targets | What it shows |
|---|---|---|
| [`templates/roomba/`](templates/roomba/) | Any wheeled base (Husky/Jackal/TB3/Rosbot) | Profile-only waypoint patrol + return-to-dock |

See [`templates/README.md`](templates/README.md) for the profile-only vs. coordinator pattern comparison.

### Production *(full agents)*

| Agent | Demo | Status |
|---|---|---|
| [`production/husky_maze/`](production/husky_maze/) | Husky Maze (5 variants) | Reference vision-driven nav agent |
| [`production/husky_swarm/`](production/husky_swarm/) | HuskySwarm (`omnilink_husky_swarm.omniworld`, 4× Husky) | **Flagship multi-robot demo** — 45 tools, true parallel execution, persistent memory |
| [`production/mission_captain/`](production/mission_captain/) | Multi-agent orchestrator | Template for cross-agent delegation |

### Bridges *(sim-to-real stubs)*

| Stub | What it shows |
|---|---|
| [`bridges/arm_bridge_stub.py`](bridges/arm_bridge_stub.py) | Arm bridge HTTP surface for a real arm driver |
| [`bridges/mobile_bridge_stub.py`](bridges/mobile_bridge_stub.py) | Mobile bridge HTTP surface for a real wheeled-base driver |
| [`bridges/bridge_base.py`](bridges/bridge_base.py) | Shared base class |

Drop in a separately validated real-robot driver underneath and keep the HTTP
surface; the OmniLink agent can keep the same tool contract. Hardware safety,
calibration, and dynamics validation are still your responsibility.

## Build roadmap

See [`ROADMAP.md`](ROADMAP.md) — what agents to build next, ordered by cool-to-effort ratio, with selection criteria and reused-vs-new-infrastructure callouts.

For cross-demo patterns (defaults to use, mistakes to skip), see [`AGENT_PATTERNS.md`](AGENT_PATTERNS.md).

## Running an agent

```bash
# 1. Launch the world that the agent's bridge talks to (in another terminal)
launch.bat projects\samples\demos\worlds\flagship\husky_maze.omniworld

# 2. Drive locally without OmniLink (validates the OmniSim side end-to-end)
python agents/production/husky_maze/solve.py

# 3. Or run as a productized OmniLink agent
export OMNI_KEY="olink_..."
export HUSKY_BRIDGE_URL="http://127.0.0.1:6070"
python agents/production/husky_maze/husky_maze_agent.py
```

## Layout convention

Each production agent folder contains:

- `README.md` — mission, status, how to run
- `profile.json` — OmniLink agent-profile scaffold pushed to <https://www.omnilink-agents.com>
- `omnilink.json` — workspace pin
- `prompts/system.md` — system prompt that feeds `mainTask` *(optional — some agents keep the prompt in `profile.json`)*
- `tools/` — auto-discovered tool modules (HTTP proxies to the bridge)
- `knowledge/` — curated grounding docs *(optional)*
- `long_term_memory/` — persisted memory state
- `<agent>_agent.py` — thin runner (profile push + `/tool` callback server + memory poll)
- `solve.py` — local solver that drives the bridge directly without OmniLink *(optional — only agents with a scriptable brief ship one)*

Templates are simpler: typically just `profile.json` + `register.py` (no Python runtime).

## Dependencies

```bash
pip install -r agents/requirements.txt
```

The `omnilink-lib` Python library is required for production agents; bridge stubs and templates work without it.
