# `agents/production/_lib`

Shared SDK that hoists the boilerplate every productized OmniLink
agent under `agents/production/` previously hand-rolled. New agents
import from here; existing agents (husky_maze, axis, …) can adopt
incrementally — nothing is forced.

## Modules

| Module | Purpose |
|---|---|
| [`runner_base.py`](runner_base.py) | Agent-side runner: profile push + tool-callback HTTP server + UsageMeter + memory poll |
| [`bridge_base.py`](bridge_base.py) | OmniSim-side controller helpers: action dispatch, `/state`, `/capabilities`, CORS |
| [`omnisim_damage.py`](omnisim_damage.py) | Existing damage-system client (kept here for back-compat) |

### Two bridge styles — `OmniSimBridgeServer` vs. `OmniLinkHTTPBridge`

OmniLink ships **two** bridge primitives, and they are not redundant — they handle different surfaces. `_lib` re-exports both so you can pick by transport style:

| Style | When to use | Endpoints | Where it lives |
|---|---|---|---|
| **Action-dispatch** (`OmniSimBridgeServer`) | OmniSim robot bridges where the agent posts structured `{action, ...args}` JSON. Every existing `*_omnilink_bridge` controller in OmniSim uses this style. | `GET /state`, `GET /capabilities`, `GET /healthz`, `POST /action` | [`bridge_base.py`](bridge_base.py) |
| **Text-command** (`OmniLinkHTTPBridge`) | Natural-language broker bridges driven by `OmniLinkEngine`. Templates like `"launch [vehicle]"` match plain-English strings to handler functions. | `GET /context`, `GET /feedback`, `POST /command`, `POST /inline-code` | `omnilink-lib` (the OmniLink Python library) |

Use the action-dispatch style for new OmniSim bridges — it's what the rest of the family speaks. Use the text-command style when you genuinely want plain English in, plain English out (chat brokers, classic OmniLink demos in the lib's `examples/` folder).

Either can be imported through `_lib`:

```python
from _lib import OmniSimBridgeServer, action            # OmniSim style
from _lib import OmniLinkHTTPBridge, OmniLinkEngine     # OmniLink lib style (if reachable)
```

## Why

Before the SDK, every `*_agent.py` re-implemented:

- omnilink-lib path discovery (the sibling-directory dance)
- A `BaseHTTPRequestHandler` subclass with CORS
- `do_GET` for `/activity` and `/status`
- `do_POST` for `/tool` with try/except/JSON serialisation
- `ThreadingHTTPServer` startup with port-fallback
- `OmniLinkClient.list_profiles()` → create-or-update flow
- `UsageMeter` baseline + `/status.usage` block
- A polling loop with heartbeat prints

Combined that's ~250 lines per agent (verify in `husky_maze_agent.py`).
With the SDK, a new agent's runner is ~30 lines.

## Minimum runner

```python
from pathlib import Path
import sys

# Ensure `agents/production/_lib` is importable when the agent script is
# launched directly (i.e. not as a package).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from _lib import OmniLinkAgentRunner

REGISTRY = load_my_tools()  # your existing tool loader
QUERY_TOOLS = [s.to_query_tool() for s in REGISTRY.values()]


def dispatch(tool_name: str, args: dict) -> dict:
    spec = REGISTRY.get(tool_name)
    if spec is None:
        return {"error": f"unknown tool: {tool_name}"}
    return spec.impl(**args)


if __name__ == "__main__":
    OmniLinkAgentRunner(
        agent_name="My Agent",
        profile_path=Path(__file__).parent / "profile.json",
        port=51530,
        dispatch=dispatch,
        query_tools=QUERY_TOOLS,
        port_env="MY_AGENT_PORT",
    ).run()
```

Set `OMNI_KEY=olink_…` and run. The runner handles profile push,
tool-callback server, CORS, status snapshot, and memory polling.

## Minimum bridge

```python
from omnisim import Supervisor
from _lib import OmniSimBridgeServer, action


class MyBridge(OmniSimBridgeServer):
    def __init__(self, supervisor):
        super().__init__(host="127.0.0.1", port=6080)
        self.sup = supervisor

    def get_state(self):
        return {"x": self.x, "y": self.y, "yaw": self.yaw}

    def get_capabilities(self):
        return {"world_title": "my_world", "actions": self.list_actions()}

    @action("stop")
    def _stop(self, body):
        self.target_velocity = 0.0
        return {"halted_at": time.time()}

    @action("set_velocity")
    def _set_velocity(self, body):
        self.target_velocity = float(body.get("v", 0.0))
        return {"v": self.target_velocity}


sup = Supervisor()
bridge = MyBridge(sup)
bridge.serve_in_background()
while sup.step(int(sup.getBasicTimeStep())) != -1:
    bridge.tick()
```

The framework auto-collects `@action`-decorated methods at instantiation
and serves them via `POST /action`, with consistent error handling and
CORS so the OmniLink web UI can call directly.

## Migration notes for existing agents

The eight agents under `agents/production/` predate this SDK and continue
to work as-is. To migrate one:

1. Replace the omnilink-lib path-discovery block with
   `from _lib import OmniLinkAgentRunner, locate_omnilink_lib` and
   `locate_omnilink_lib(env_var="<AGENT>_OMNILINK_LIB")` early in the
   script.
2. Delete the local `do_GET`/`do_POST` Handler class, the
   `start_tool_server` helper, and the `ensure_profile` helper —
   they're now in `OmniLinkAgentRunner`.
3. Pass your existing `dispatch_tool` and `QUERY_TOOLS` straight into
   the runner constructor.
4. Move agent-specific status fields into a `status_snapshot=` callback;
   keep your `classify_tool_result` (the SDK accepts it via
   `classify_result=`).

Each migration removes ~200 lines without changing behaviour. None is
required; the SDK is opt-in.

## What the SDK does NOT do

- It doesn't load tools. Tool discovery (`tools/_base.py`,
  auto-loading, schema generation) stays per-agent — those vary
  meaningfully across demos.
- It doesn't decide on a chat-loop strategy. Some agents poll memory
  for user messages, some run a `chat_drive.py` script externally,
  some are server-driven. The runner just exposes the tool-callback
  surface; how the agent gets prompted is the agent's choice.
- It doesn't proxy bridge calls. Each agent's `tools/` package owns
  the HTTP shape its bridge speaks.
