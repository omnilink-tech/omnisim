# Build an OmniLink agent for OmniSim

There are three supported starting points. Pick the smallest one that matches
what you are building.

| Goal | Start here | What you own |
|---|---|---|
| Change how an existing robot behaves | A profile-only template in [`agents/templates/`](../../agents/templates/) | Prompt and profile settings |
| Build a new agent with tools and its own callback server | `python -m omnisim agent new` | Profile, tool schemas, dispatch functions |
| Connect a physical robot | [`packages/omnisim-bridges/`](../../packages/omnisim-bridges/) | The hardware driver behind `BridgeBase` |

## Create a complete agent

From the repository root:

```bash
python -m omnisim agent new inspection_rover --robot-class mobile
```

Robot classes are `mobile`, `arm`, `quadruped`, and `flying`. The command
creates `agents/production/inspection_rover/` with:

- `profile.json` — the OmniLink behavior profile;
- `omnilink.json` — the world, ports, environment variables, and launcher metadata;
- `inspection_rover_agent.py` — a runnable `OmniLinkAgentRunner` with state,
  stop, and one class-specific motion tool;
- `README.md` — exact run instructions.

The scaffold refuses to overwrite an existing directory. Review the generated
world and ports, then run:

```bash
export OMNI_KEY="olink_YOUR_KEY"
python -m omnisim run-agent inspection_rover
```

`run-agent` starts the declared OmniSim world, waits for the robot bridge, and
then starts the agent. If `OMNI_KEY` is missing it exits before launching the
world. Use `--no-agent` for an intentionally local-only simulation.

## Add a tool safely

Every callable tool needs two matching pieces in the generated agent:

1. A JSON-schema entry in `QUERY_TOOLS`. Required fields belong in the
   schema's `required` array.
2. A branch in `dispatch()` that calls a bridge endpoint and returns JSON.

Keep `stop_robot` available. Read state before motion, reject ambiguous units,
and verify state after motion before reporting success. The shared callback
server validates tool arguments, limits request size, serializes non-stop tool
execution, and logs activity.

## HTTP trust boundary

Robot bridges and agent callback servers bind to `127.0.0.1` by default.
Browser requests are accepted only from OmniLink or explicitly allowed origins.
Set `OMNISIM_ALLOWED_ORIGINS` to a comma-separated list for another trusted UI.

To bind a robot bridge to a LAN interface, set a token first:

```bash
export OMNISIM_BRIDGE_TOKEN="a-long-random-secret"
```

Clients send it as `Authorization: Bearer <token>` or `X-OmniSim-Token`.
An unauthenticated non-loopback bind is refused. This token protects the HTTP
control surface; use a TLS reverse proxy as well when traffic leaves the host.

## Test without a cloud key

The contract suites are offline:

```bash
python packages/omnisim-bridges/tests/test_smoke.py
pytest tests/omnilink_integration/test_bridge_conformance.py
pytest tests/agents/test_agent_runner_smoke.py
```

They cover bridge routes, malformed JSON, required parameters, CORS/auth,
protocol negotiation, and every production agent that uses the shared runner.
A real OmniLink model turn still requires `OMNI_KEY` and network access; do not
interpret an offline smoke pass as a live-provider result.

## Connect a physical robot

Subclass `omnisim_bridges.BridgeBase`, implement the actions supported by the
hardware, and point the agent's `*_BRIDGE_URL` at that service. This preserves
the agent/tool HTTP contract. It does not by itself validate dynamics, safety,
calibration, or policy transfer on physical hardware; those remain integration
work for the robot and its driver.
