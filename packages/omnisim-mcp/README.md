# omnisim-mcp

**Talk to OmniSim from any MCP client.** A [Model Context Protocol](https://modelcontextprotocol.io)
server that exposes the OmniSim **World Harness** (author → inspect → screenshot →
hot-reload) as MCP tools, so Claude Desktop, Cursor, or any MCP-capable agent can drive
world authoring and debugging directly.

## Why this exists

OmniSim already ships the thing other simulators only get through third-party glue: a
**first-party, agent-facing HTTP surface** for authoring and debugging worlds (the harness,
[`PROTOCOL.md` §world_harness](../../PROTOCOL.md)). The agent ecosystem standardized on MCP —
and the comparable community servers (`omni-mcp/isaac-sim-mcp`, `kvgork/gazebo-mcp`) wrap a
*non*-agent-native simulator. This one wraps an *already* agent-native one, so it is a thin,
honest adapter rather than a re-plumbing.

**Zero runtime dependencies.** The MCP stdio protocol and the harness HTTP client are both
implemented on the Python standard library, matching the harness's own design — so it runs on
a fresh clone, and under OmniSim's embedded interpreter, with nothing installed.

## Setup

1. **Start the harness** (this server is a stateless proxy to it):

   ```bash
   python scripts/harness/omnisim_harness.py --port 6789
   # or: python scripts/dev/omnisim_dev.py harness
   ```

2. **Register the MCP server** with your client. Either install it —

   ```bash
   pip install -e packages/omnisim-mcp     # provides the `omnisim-mcp` command
   ```

   — or run it straight from source with no install (`python -m omnisim_mcp`, with
   `packages/omnisim-mcp/src` on `PYTHONPATH`).

   Claude Desktop / Cursor `mcpServers` entry:

   ```json
   {
     "mcpServers": {
       "omnisim": {
         "command": "omnisim-mcp",
         "env": { "OMNISIM_HARNESS_URL": "http://127.0.0.1:6789" }
       }
     }
   }
   ```

3. **Call `harness_status` first.** It reports whether the harness is reachable and, if not,
   exactly how to start it.

## Tools

Each tool is one HTTP call to the harness — the surface mirrors
[`AGENTS.md` §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness) and
[`PROTOCOL.md`](../../PROTOCOL.md) so it stays honest to the real endpoints.

| Tool | Harness endpoint | Purpose |
|---|---|---|
| `harness_status` | `GET /healthz`, `/sim/state` | Is the harness up, and on what world? Start here. |
| `load_world` | `POST /world/sync` (or `/world/load`) | Default safe iteration path: batch live pose-only edits, automatically reload anything else; `force_reload=true` restarts deliberately. |
| `get_scene_tree` | `GET /scene/tree` | Every node's type, DEF, pose. |
| `get_scene_node` | `GET /scene/node/<def>` | Full field dump + contacts for one node. |
| `look_at` | `POST /scene/look_at` | Aim the Viewpoint from a position at a target. |
| `screenshot` | `POST /world/screenshot` | Render PNG — returned **inline** (so a vision agent sees it) or written to a path. |
| `render_stats` | `GET /world/render_stats` | Exposure/brightness stats — catch blown-out lighting without eyeballing. |
| `sim_step` | `POST /sim/step` | Advance N basic timesteps. |
| `sim_reset` | `POST /sim/reset` | Reset to t=0 without re-parsing. |
| `get_events` | `GET /sim/events` | Unified event stream (`controller.log`, `contact.*`, `joint.limit_hit`, `damage.*`). |
| `list_robots` | `GET /robots` | Every Robot with pose + joint count. |
| `get_robot_joints` | `GET /robot/<def>/joints` | Per-joint position/velocity/limits. |
| `get_contacts` | `GET /sim/contacts` | Global contact set. |
| `get_diagnostics` | `GET /world/diagnostics` | Re-fetch the current load's diagnostics. |

## Config

| Env var | Default | Meaning |
|---|---|---|
| `OMNISIM_HARNESS_URL` | `http://127.0.0.1:6789` | Base URL of the running harness. |
| `OMNISIM_MCP_TIMEOUT` | `60` | Per-request HTTP timeout (seconds). |

## Notes & limits

- **Stateless.** The server holds no simulator state; restart it freely. All state lives in
  the harness.
- **One harness at a time.** For parallel sessions, run harnesses on different ports (see
  [`AGENTS.md` §3e](../../AGENTS.md)) and point separate `omnisim` MCP entries at each.
- **Not the capture/cinema service.** This wraps the *authoring* harness (`:6789`), not the
  capture service (`:6791`). High-res/movie rendering stays in
  [`scripts/capture/`](../../scripts/capture/).
- **Protocol.** Implements the tools-only MCP subset (`initialize` / `tools/list` /
  `tools/call`) over newline-delimited JSON-RPC on stdio. Tested against protocol versions
  `2024-11-05` and `2025-06-18`.

Run the tests with `PYTHONPATH=src python -m pytest packages/omnisim-mcp/tests -q`.
