# OmniTug 500 Warehouse Courier (OmniLink agent)

Natural-language warehouse pick-and-deliver for the OmniTug 500 AGV. Tell the
rover which package to pick from which bay and which dock to deliver it to — in
plain language — and it routes the aisles, loads its deck, and sets the package
down. Multi-stop routes in one request.

```bash
set OMNI_KEY=olink_YOUR_KEY
python -m omnisim run-agent --agent omnitug500_warehouse
# -> opens omnitug500_courier.omniworld, waits for the bridge, starts this agent.
# Pick "OmniTug 500 Warehouse Courier" at omnilink-agents.com and chat:
#   "take the package from bay B to dock 2"
#   "collect from bay A and bay C, deliver both to dock 3"
#   "where are you?"  /  "return to the charging dock"  /  "stop"
```

No `OMNI_KEY`? The world still works standalone — open it, right-click the rover
→ *Show Robot Window*, and the offline regex router drives the same actions.

| File | What |
|---|---|
| `omnilink.json` | run-agent registry entry (world, bridge `:8765`, this script) |
| `profile.json` | the OmniLink agent profile + dispatcher system prompt |
| `omnitug500_warehouse_agent.py` | thin runner on the `_lib` SDK |
| `tools/courier.py` | the 8 tools, each one bridge endpoint |
| `knowledge/` | bridge contract + warehouse layout (read [docs/OVERVIEW.md](docs/OVERVIEW.md) first) |
| `long_term_memory/` | agent-written solved-route notes |

The world, controller, and layout are under
[`projects/robots/omnisim/omnitug500/`](../../../projects/robots/omnisim/omnitug500/);
the generator is
[`scripts/dev/gen_courier_world.py`](../../../scripts/dev/gen_courier_world.py).
