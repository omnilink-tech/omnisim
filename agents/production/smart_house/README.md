# SmartHouse — the OmniLink smart-home agent, running a physics-backed house

OmniLink ships a dormant smart-home agent, **Haven**, whose missing piece has
always been the hub it proxies device commands to. The OmniSim smart-house
world + `smart_house_bridge` controller **is** that hub: a real,
physics-simulated house — thermal model, energy metering, a door that is an
actual body in the scene — served over the same 19-tool surface Haven
defined. This package is the OmniSim-side agent (Haven-shaped, self-contained)
plus the benchmark that answers the demo's headline question:

> **What is a persistent agent worth?** An interactive-only agent can only
> fix what you ask it to, when you ask. A persistent agent that wakes every
> hour while you're out catches the oven you left on 420 house-minutes
> sooner — measured from the simulator, not from the agent's self-report.

## Layout

| path | what |
|---|---|
| `omnilink.json` | launcher manifest (`python -m omnisim run-agent --list`) |
| `profile.json` | OmniLink profile (`{"name": "SmartHouse", "settings": {...}}`) |
| `smart_house_agent.py` | interactive runner on the `_lib` SDK (tool server on :51534) |
| `prompts/system.md` | the house-manager mandate — canonical; synced into `settings.mainTask` at startup |
| `tools/` | the 19 Haven-shaped tool specs, auto-discovered (`SPEC`/`SPECS` modules) |
| `benchmark/` | normal-vs-persistent tier comparison + `mock_hub.py` (offline house) |

## Quick start

```bash
# 1. The house. Either the real world (smart_house_bridge hub on :8766):
python -m omnisim run-world projects/samples/demos/worlds/flagship/omnilink_smart_house.omniworld
#    ...or the offline mock (no simulator needed):
python agents/production/smart_house/benchmark/mock_hub.py --port 8766

# 2. The agent (OMNI_KEY from env, else OMNI_KEY_FILE, else ~/.omnilink/omni_key.txt):
python agents/production/smart_house/smart_house_agent.py

# 3. Talk to SmartHouse in the OmniLink UI: "movie night", "away mode",
#    "is anything wrong with the house?"

# Or let the launcher boot world + agent in order:
python -m omnisim run-agent --agent smart_house
```

## Tool tiers (Haven's convention, enforced client-side)

- **SAFE** — reads, `lock_door`, `arm_security`, `notify_occupant`
  (tightening security posture is always allowed).
- **GUARDED** — routine control (`set_device`, `set_scene`,
  `adjust_thermostat`, `toggle_device`, `set_schedule`); honours
  `SMART_HOUSE_AGENT_DRY_RUN=1`.
- **CONFIRM_REQUIRED** — `unlock_door`, `disarm_security`,
  `shut_water_main`, `shut_gas_main`: the impl refuses without an
  occupant-provided `authorization` token; the request never reaches the
  hub. The hub gates them a second time server-side.

Honesty rules ride the hub contract: every state in a response is measured
from the house after the change settles (never an echo of the argument),
schedules and mains return honest structured refusals, and the benchmark
scores only `/scenario/metrics`.

## The benchmark

See [benchmark/README.md](benchmark/README.md). Offline CI mode
(`--mock --fake-llm`) runs the whole harness — scenario engine, wake
cadence, metrics, report — with no network, no key, and no simulator, in a
few seconds. Live mode points the same driver at the real bridge and the
real `/api/chat` tool loop.

Environment: `SMART_HOUSE_HUB_URL` (default `http://127.0.0.1:8766`),
`SMART_HOUSE_AGENT_PORT` (default 51534), `OMNI_KEY` / `OMNI_KEY_FILE`.
