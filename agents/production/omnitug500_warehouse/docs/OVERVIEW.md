# OmniTug 500 Warehouse Courier — overview

A productized OmniLink agent that runs a **transportation warehouse**: an operator
(or the agent itself) tells an OmniTug 500 AGV, in plain language, which package
to pick from which bay and which dock to deliver it to. The rover routes through
the aisle grid, loads the package onto its deck, drives to the dock, and sets it
down. Multi-stop runs ("collect from A and C, deliver both to dock 2") are one
call.

## Architecture

```
Operator @ omnilink-agents.com  ──or──  chat side-panel in the world
        │ chat() -> toolCalls                      │ "prompt:<text>"
        ▼                                           ▼
  omnitug500_warehouse_agent.py (this agent)     in-controller OmniLink relay (key)
        │ POST /tool (tool dispatch)                │
        └──────────────┬────────────────────────────┘
                       ▼  POST /goto_station /pick_package /deliver_package /run_route
        ┌──────────────────────────────────────────────┐
        │ omnitug500_courier controller  (HTTP :8765)       │
        │  • CourierBridge: mission queue, deck carry    │
        │  • CourierNav: seeded known-map A* + pursuit   │
        │  • drives the rover via Newton setVelocity     │
        └──────────────────────────────────────────────┘
                       ▼ OmniSim Supervisor (Newton/MuJoCo)
        ┌──────────────────────────────────────────────┐
        │ omnitug500_courier.omniworld — racks, docks, bays,      │
        │ staged packages, OmniTug 500 rover + scanners       │
        └──────────────────────────────────────────────┘
```

There are **two ways to drive it**, both onto the same bridge surface:

1. **Chat side-panel** (no agent process): open the world, right-click the rover
   → *Show Robot Window*, type. The controller's own OmniLink relay handles it,
   so `OMNI_KEY` has to be set in the shell that launches the world — the relay
   is built once, at controller start. Without it the panel answers that the
   OmniLink connection is required, `POST /prompt` answers `401 omnikey_required`
   and nothing actuates; the direct bridge verbs and Stop still work.
2. **This productized agent** (autonomous, `run-agent` launchable): a separate
   process that pushes its own OmniLink profile and drives the bridge's HTTP
   endpoints — the form you reach for to script, automate, or expose the courier
   as a first-party agent.

## What's verified

A headless multi-stop run (pick Bay A in the NW, pick Bay F in the SE, deliver
both to Dock 2) completes end-to-end with a **minimum oriented-footprint
clearance ≈ 0.33 m** — genuinely collision-safe, two packages on the deck at
once.

⚠️ The resolution of bay letters, colours ("the green package"), dock numbers,
multi-stop routes, status and stop/reset was measured against the controller's
keyword router, `courier_intent.CourierIntent` — which was **deleted on
2026-09-22** with the other four ladders (it had in fact been constructed and
threaded through the controller without ever being called). Those phrasings are
now resolved by the model's tool call behind an OmniKey, and that has **not**
been re-measured.

## Run it

```bash
# both world + agent in one step (probes bridge readiness):
set OMNI_KEY=olink_...
python -m omnisim run-agent --agent omnitug500_warehouse

# or separately:
launch.bat projects\robots\omnisim\omnitug500\worlds\omnitug500_courier.omniworld
set OMNI_KEY=olink_...
python agents/production/omnitug500_warehouse/omnitug500_warehouse_agent.py
```

Then pick **"OmniTug 500 Warehouse Courier"** at omnilink-agents.com and say
*"take the package from bay B to dock 2"*.

See [knowledge/courier-bridge.md](../knowledge/courier-bridge.md) for the HTTP
contract and [knowledge/warehouse-layout.md](../knowledge/warehouse-layout.md)
for the map.
