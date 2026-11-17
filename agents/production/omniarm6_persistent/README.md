# OmniArm 6 — a persistent agent

A scheduled OmniLink agent that wakes on its own, moves the simulated OmniArm 6 arm,
and reports what it saw. It exists to make persistence **observable**: leave it
running, close everything, come back, and see whether the arm moved while you
were away.

## What "persistent" actually means here

The work is split, deliberately:

| Where | What happens there |
|---|---|
| **OmniLink's servers** | A cron tick wakes the agent on schedule, calls the model, and returns tool calls. Memory and history live here too. |
| **Your machine** | Every tool actually runs — the connector receives each tool call over the WebSocket it holds open and POSTs it to the arm bridge on `127.0.0.1:8765`. |

That split is the point. It is why a robot bridge on localhost is reachable at
all, and why your tools and files never leave your network. A Cloud Run instance
cannot reach `127.0.0.1` on your PC, so the tools *have* to run here.

**So be precise about what survives what:**

| Action | Agent keeps running? |
|---|---|
| Close the OmniLink browser tab | ✅ yes — the browser was never involved |
| Log out of omnilink-agents.com | ✅ yes |
| Close OmniSim | ⚠️ the agent runs, but arm tools fail — the bridge went with it |
| Stop the edge connector | ❌ no — nothing on your side can execute a tool |
| Switch the machine off | ❌ no |

An agent whose machine is off is not lost — it keeps its memory and resumes on
the next tick after the connector reconnects. It simply cannot act in the
meantime, and each missed run is recorded with the reason rather than being
silently skipped.

## Setup

**1. Export your Omni Key**

```bash
export OMNI_KEY=olink_...          # Windows: set OMNI_KEY=olink_...
```

**2. Launch the OmniArm 6 world**

```bash
launch.bat projects\samples\demos\worlds\chat\omnilink_omniarm6.omniworld
```

Launch it from a shell where `OMNI_KEY` is set. On startup the bridge registers
an OmniLink profile called **`OmniSim-omniarm6`** carrying its tool list and
`toolCallbackUrl` (`http://127.0.0.1:8765/tool`). Without `OMNI_KEY` the bridge
skips profile sync entirely and there is nothing for a scheduled agent to drive.

**3. Install the connector as a service**

```bash
python -m omnilink.edge_service install
python -m omnilink.edge_service status
```

Run it in a terminal instead if you prefer, but then persistence lasts exactly
as long as that terminal does. Installing it is what makes the agent survive a
reboot.

**4. Arm the agent**

```bash
python create_order.py                 # every 5 minutes
python create_order.py --every 2m      # faster, for watching
```

The script preflights before arming: it confirms the profile exists, has a
`toolCallbackUrl`, and declares tools — so it fails with something you can act
on rather than creating an agent that silently cannot move.

## Watching it

```bash
python create_order.py --list          # schedule, next run, last run
python -m omnilink.edge_service status # is the connector alive, and where are its logs
```

The arm waving is the honest signal. The connector log shows exactly which tools
ran on your machine while you were away.

## Turning it off

```bash
python create_order.py --delete
python -m omnilink.edge_service uninstall
```

## If it does not move

Work down the chain — each link fails loudly and differently:

- **Refused with `PERSISTENT_AGENTS_NOT_ON_PLAN`** — the free tier runs agents
  only while you are connected. Scheduled agents are a paid tier.
- **`EDGE_NOT_CONNECTED` on the run** — the connector is not running. Check
  `edge_service status`.
- **`EDGE_TOOL_UNSUPPORTED`** — the connector predates tool frames. Upgrade the
  `omnilink` package; older builds silently drop frame types they do not know,
  which is why the server refuses them outright instead of waiting.
- **A tool refused by name** — the connector only runs tools the agent declares.
  The tool name comes from a language model, so an undeclared name is rejected
  rather than looked up on disk. Relaunching the world re-syncs the tool list.
- **Tool timed out** — the arm bridge did not answer within 55s. Usually the
  world is paused or OmniSim is closed.

## Known rough edges

- `schedule_timezone` is stored but the tick matches cron fields against its own
  host clock (`calculateNextRun`). Interval schedules like `5m` are unaffected;
  cron schedules are, so this demo uses an interval.
- The persistent-agent count shown in the UI counts standing orders and channel
  bindings but not every source, so it can under-report. A refusal is still
  correct when it fires — the number is display-only.
