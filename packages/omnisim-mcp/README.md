# omnisim-mcp

<!-- mcp-name: io.github.omnilink-tech/omnisim -->

**Talk to OmniSim from any MCP client.** A [Model Context Protocol](https://modelcontextprotocol.io)
server that exposes the OmniSim **World Harness** (author → inspect → screenshot →
hot-reload) as MCP tools, so Claude Code, Claude Desktop, Cursor, or any MCP-capable agent
can drive world authoring and debugging directly.

## Why this exists

OmniSim already ships the thing other simulators only get through third-party glue: a
**first-party, agent-facing HTTP surface** for authoring and debugging worlds (the harness,
[`PROTOCOL.md` §world_harness](../../PROTOCOL.md)). The agent ecosystem standardized on MCP —
and the comparable community servers (`omni-mcp/isaac-sim-mcp`, `kvgork/gazebo-mcp`) wrap a
*non*-agent-native simulator. This one wraps an *already* agent-native one, so it is a thin,
honest adapter rather than a re-plumbing.

**The server itself has zero runtime dependencies.** The MCP stdio protocol and the harness
HTTP client are both implemented on the Python standard library, matching the harness's own
design — so the *proxy* runs from a fresh clone, and under OmniSim's embedded interpreter,
with nothing installed. That is a statement about this package only. The thing it proxies is
a simulator, and the simulator has to exist first — see Prerequisites.

## Prerequisites

This server is a **stateless proxy**. Three things sit behind it, and a fresh `git clone`
supplies none of them:

1. **A working OmniSim install.** A clone contains no engine: `msys64/` is gitignored and has
   **0 tracked files**, so there is no `omnisim-bin` until you install or build one. Either
   install the Windows package from
   [Releases](https://github.com/omnilink-tech/omnisim/releases), or build from source
   ([`docs/developer/quickstart.md`](../../docs/developer/quickstart.md)) and then vendor the
   physics runtime:

   ```bash
   make -C src/omnisim bundle-newton-runtime
   ```

   That step is not optional. Newton is the only physics backend, so an engine without its
   runtime has no dynamics at all — nothing falls, and it fails quietly. Check the whole
   chain with:

   ```bash
   python -m omnisim doctor      # prints a VERDICT line; non-zero exit if it cannot run
   ```

   **macOS is not supported** — no package, no verified build, Newton unverified.

2. **A running harness** on `:6789`. See Setup step 1.

3. **Pillow, for exactly one tool.** `render_stats` is served by `GET /world/render_stats`,
   which returns **503** when the harness's interpreter has no Pillow
   (`omnisim_harness.py:1753`). No other tool needs it — `screenshot` works without.
   `pip install Pillow` into whichever interpreter runs the harness.

## Setup

1. **Start the harness** (this server is a stateless proxy to it):

   ```bash
   python -m omnisim harness
   ```

   **Use the module form, not `python scripts/harness/omnisim_harness.py`.** The module form
   runs the harness under `omnisim_env()` (`omnisim/dev/runner.py:79`), which pins
   `OMNISIM_HOME` to this clone, uses `sys.executable`, and on Windows prepends the bundled
   `msys64/mingw64/bin` so the engine's Qt6 DLLs resolve. The raw script does none of that,
   so on Windows it starts fine, answers `harness_status` as healthy, and then fails the
   **first world load** with `LAUNCHER_DLL_NOT_FOUND` (Windows exit code `0xC0000135`).

2. **Register the MCP server** with your client.

   **Claude Code** — nothing to do. Open the OmniSim directory and the checked-in
   [`.mcp.json`](../../.mcp.json) registers the server: no install, no `claude mcp add`. To
   register it explicitly (for example from another directory):

   ```bash
   claude mcp add omnisim -e PYTHONPATH=packages/omnisim-mcp/src \
     -e OMNISIM_HARNESS_URL=http://127.0.0.1:6789 -- python -m omnisim_mcp
   ```

   **Claude Desktop / Cursor** — add this to the client's `mcpServers` config. The paths are
   relative to the OmniSim checkout, so set the client's `cwd` to it if it supports one, or
   make `PYTHONPATH` absolute:

   ```json
   {
     "mcpServers": {
       "omnisim": {
         "command": "python",
         "args": ["-m", "omnisim_mcp"],
         "env": {
           "PYTHONPATH": "packages/omnisim-mcp/src",
           "OMNISIM_HARNESS_URL": "http://127.0.0.1:6789"
         }
       }
     }
   }
   ```

   `python -m omnisim_mcp` is used deliberately instead of the `omnisim-mcp` console script.
   That script only exists after `pip install -e`, and it lands in the installing
   interpreter's `Scripts/` directory, which an MCP client's spawn environment often does not
   inherit on Windows. The package is pure stdlib, so the module form needs no install at all.

   `pip install -e packages/omnisim-mcp` still works and provides the `omnisim-mcp` command if
   you want it. **The package is not on PyPI** (verified: the JSON API returns 404) and
   [`publish-omnisim-mcp.yml`](../../.github/workflows/publish-omnisim-mcp.yml) is
   `workflow_dispatch`-only, so it has never fired. It is not in the official MCP Registry
   either. `pip install omnisim-mcp` does not resolve — install from this checkout.

3. **Call `harness_status` first.** It reports whether the harness is reachable and, if not,
   exactly how to start it.

   ⚠️ **It reports on the *harness*, not the engine.** The harness is a Python HTTP server; it
   comes up and answers healthy on an install that cannot load a single world — no engine
   binary, no Newton runtime, missing Qt DLLs. Reachable means "the proxy has something to
   talk to", not "this install works". `python -m omnisim doctor` is the check for that.

### First call

With the harness running:

```
load_world      {"path": "projects/samples/demos/worlds/showcase/warehouse_husky.omniworld",
                 "light": true}
get_scene_tree  {}
frame           {"def": "HUSKY"}
screenshot      {}
```

Relative paths resolve against the **repo root of the clone whose `omnisim_harness.py` is
running** (`omnisim_harness.py:2972`) — not your editor's working directory, and not the MCP
client's. If the harness runs out of a different checkout, pass an absolute path.

`light: true` is not an optimisation you add later; it is how you open the world. Without it
the harness is **slower than just re-running `run-headless`** (a default-mode reload of the
cloth world measured 13.4 s against 6.37 s headless). Measured on the 309-node fleet arena
(2026-08-29, CPU `mj_step`): a full-tracking `sim_step` costs 573–606 ms against 6–35 ms light
(~17×), ten steps 2855–3187 ms vs 48–67 ms (~47×), the load 12.1 s vs 4.1 s. The trade is that
`get_contacts` still answers but the `contact.*` / `grip.*` / `joint.limit_hit` **events** and
`get_grips` go quiet. Need exactly one tracker? Pass `tracking` instead —
`{"tracking": {"contacts": false, "joint_limits": true, "grips": false}}` keeps
`joint.limit_hit` while paying no contact walk (per-tracker toggles since 2026-09-01) — see
[`AGENTS.md` §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness).

## Tools

37 tools, each one HTTP call to the harness — the surface mirrors
[`AGENTS.md` §5](../../AGENTS.md#5-iterating-on-worlds-with-the-validation-harness) and
[`PROTOCOL.md`](../../PROTOCOL.md) so it stays honest to the real endpoints. The live list is
`python -m omnisim_mcp --help`.

| Tool | Harness endpoint | Purpose |
|---|---|---|
| `harness_status` | `GET /sim/state` | Is the harness up, and on what world? Start here — but see the warning above about what it does *not* prove. |
| `get_capabilities` | `GET /capabilities` | What this harness can and will not do: verified physics backend, measured step cost + recommended step budget (`probe_step=true` measures one), event types (with what light mode suppresses), every endpoint and every gap under `not_supported`. The harness docs say to call it first. |
| `load_world` | `POST /world/sync` (or `/world/load`) | Default safe iteration path: batch live pose-only edits, automatically reload anything else; `force_reload=true` restarts deliberately. **Pass `light: true`** (or a per-tracker `tracking` object, which forces a `/world/load`) — see the note above. |
| `world_sync` | `POST /world/sync` | The sync semantics by name: live pose batch (`mode=live_pose`) or automatic hot reload (`mode=full_reload`); also `no_change` / `rejected` / `busy`. |
| `get_scene_tree` | `GET /scene/tree` | Every node's type, DEF, pose. |
| `get_scene_node` | `GET /scene/node/<def>` | Full field dump + contacts for one node. |
| `get_viewpoint` | `GET /scene/viewpoint` | **Read** the live camera: position, orientation, FOV, near/far, plus derived forward/up/right and the resolved FOV for the real viewport aspect. Every other camera tool writes to a camera you otherwise cannot read. |
| `frame` | `POST /scene/frame` | **The camera verb to reach for first.** Computes aim *and* distance from the node's real geometric bounds, pushes it, and returns a numeric proof the subject is in frame. Prefer it over guessing a `look_at` position. |
| `orbit` | `POST /scene/orbit` | Nudge the camera *relative* to the current view (azimuth, elevation, dolly, pan). Every other camera tool is absolute. |
| `visible` | `GET /scene/visible` | What is on screen right now: frustum test, screen-space bbox in pixels, angular offset, plus hints like `"off-screen: 34 deg to the left"`. The closed-loop feedback signal for aiming. |
| `look_at` | `POST /scene/look_at` | Aim the Viewpoint from an explicit position at a target. Use when you already know both points. |
| `screenshot` | `POST /world/screenshot` | Render PNG — returned **inline** (so a vision agent sees it) or written to a path. Call `frame` first; do not guess a pose and iterate on screenshots. |
| `render_stats` | `GET /world/render_stats` | Exposure/brightness stats — catch blown-out lighting without eyeballing. **Needs Pillow** (see Prerequisites). |
| `scene_spawn` | `POST /scene/spawn` | Import a node from VRML, a type+fields spec, or a clone of a DEF. ⛔ By default a scene-graph verb, **not** a physics verb: the spawned node has NO physics (the response's `physics_warning` says so). Pass `physics: "rebuild"` — or call `rebuild_physics` — and it is simulated. |
| `scene_delete` | `POST /scene/delete` | Remove nodes by DEF. ⛔ By default the frozen solver model keeps the deleted colliders as phantoms — a deleted wall still blocks, a deleted floor still holds bodies up. Pass `physics: "rebuild"` — or call `rebuild_physics` — and they are gone. |
| `scene_set_pose` | `POST /scene/set_pose` | Move an existing node (velocity reset + settle by default). ⚠ Nothing checks interpenetration — check bounds first. |
| `sim_step` | `POST /sim/step` | Advance N basic timesteps. Size N from `get_capabilities` → `limits.recommended_max_steps_per_request` (a rolling median of the measured per-step cost on *this* world). |
| `rebuild_physics` | `POST /sim/rebuild_physics` | W1.7: rebuild the Newton world at the scene's **current** poses (97–267 ms measured) so runtime-spawned nodes gain physics and deleted ones lose it; 409 `REBUILD_REFUSED` on cloth/soft/granular worlds; engaged welds are dropped loudly. |
| `read_bench` | `GET /debug/read_bench` | Diagnostic: measured cost of one supervisor read on this session, free-running vs paused (`n` reads per arm). |
| `scene_node_particles` | `GET /scene/node/<def>/particles` | Particle stats for one Cloth / SoftBody / GranularBed / GranularGroup node: count, world-frame min/max/centroid over the finite particles, `non_finite`; `sample=N` adds every N-th particle. |
| `sim_reset` | `POST /sim/reset` | Reset to t=0 **and restore the authored scene** without re-parsing; forwards `restore`/`verify`/`settle_steps`. |
| `sim_snapshot` | `POST /sim/snapshot` | Save a named engine-side state snapshot — a rollback point that is not t=0. |
| `sim_restore` | `POST /sim/restore` | Restore a named snapshot without rewinding the clock; reports how far it landed. Unknown names are refused (on purpose). |
| `list_snapshots` | `GET /sim/snapshots` | The named snapshots taken in this world. |
| `get_events` | `GET /sim/events` | Unified event stream (`controller.log`, `contact.*`, `joint.limit_hit`, `damage.*`). |
| `list_robots` | `GET /robots` | Every Robot with pose + joint count. |
| `get_robot_joints` | `GET /robot/<def>/joints` | Per-joint position/velocity/limits. |
| `robot_devices` | `GET /robot/<def>/devices` | Device inventory of a robot's subtree. |
| `robot_joints_set` | `POST /robot/<def>/joints/set` | Command joint position targets, settle-and-verify: measured `{commanded, achieved, error}` per joint, never the argument echoed back. |
| `robot_ik` | `POST /robot/<def>/ik` | Batched IK **preview** against the exact model the solver steps — nothing moves; per-target `residual_m`, apply via `robot_joints_set`. |
| `get_contacts` | `GET /sim/contacts` | Global contact set: `[{a_def, b_def, point, paired}]` plus a `tracking` block (what was walked, `empty_set_reasons`, `inert_pinned_solids`). **Works in light mode** — it is walked per call and never reads the dropped tracker. |
| `get_grips` | `GET /sim/grips` | Inferred grips. ⚠ Empty in a light-mode session (the tracker is dropped). |
| `get_diagnostics` | `GET /world/diagnostics` | Re-fetch the current load's diagnostics. |
| `robot_damage` | `GET /robot/damage` | Damage state of the tracked robot (per-part HP / state) — damage-tracking worlds only. |
| `robot_damage_events` | `GET /robot/damage/events` | Filtered view of the `damage.*` events with their own `since` cursor. |
| `robot_damage_reset` | `POST /robot/damage/reset` | Heal every part **without** resetting the simulation. |
| `robot_damage_inject` | `POST /robot/damage/inject` | Set a part's damage state directly — the fault-injection verb. |

The four camera tools (`get_viewpoint`, `frame`, `orbit`, `visible`) are the ones
[`AGENTS.md`](../../AGENTS.md) tells agents to reach for *instead of* guessing a pose and
iterating on screenshots.

## Command line

The server is normally spawned by an MCP client and speaks JSON-RPC on stdio, but it answers
three flags without entering that loop:

```bash
PYTHONPATH=packages/omnisim-mcp/src python -m omnisim_mcp --help       # tool list + harness URL
PYTHONPATH=packages/omnisim-mcp/src python -m omnisim_mcp --version
PYTHONPATH=packages/omnisim-mcp/src python -m omnisim_mcp --self-test  # calls harness_status; exit 1 if unreachable
```

`--self-test` is the fastest way to check a registration end to end without an MCP client.
Before these existed, `--help` printed a startup line and then blocked forever reading stdin.

## Config

| Env var | Default | Meaning |
|---|---|---|
| `OMNISIM_HARNESS_URL` | `http://127.0.0.1:6789` | Base URL of the running harness. |
| `OMNISIM_MCP_TIMEOUT_S` | `130` | Per-request HTTP timeout (seconds). Deliberately **above** the harness's own 120 s supervisor-RPC timeout, so the wrapper never abandons a request the harness is still faithfully serving (which would silently desync the agent's world-model). `OMNISIM_MCP_TIMEOUT` is the accepted legacy spelling. |
| `OMNISIM_MCP_KEEPALIVE` | `1` | Pool one `http.client` connection per harness (`0` = a fresh connection per request, for an A/B). The harness speaks HTTP/1.1 with keep-alive since 2026-09-01 (measured on the flip: reuse 0/229 → 229/229, `GET /healthz` 5.09 → 0.31 ms), so the pool genuinely reuses the socket; against an older HTTP/1.0 harness it degrades to per-request automatically. |

## Notes & limits

- **Stateless.** The server holds no simulator state; restart it freely. All state lives in
  the harness.
- **One harness at a time.** For parallel sessions, run harnesses on different ports (see
  [`AGENTS.md` §3e](../../AGENTS.md)) and point separate `omnisim` MCP entries at each via
  `OMNISIM_HARNESS_URL`.
- **Not the capture/cinema service.** This wraps the *authoring* harness (`:6789`), not the
  capture service (`:6791`). High-res/movie rendering stays in
  [`scripts/capture/`](../../scripts/capture/).
- **Protocol.** Implements the tools-only MCP subset (`initialize` / `tools/list` /
  `tools/call`) over newline-delimited JSON-RPC on stdio. Tested against protocol versions
  `2024-11-05` and `2025-06-18`.
- **Responsive under long calls.** Tool calls run serialized on one worker thread; the
  reader thread answers `ping` / `initialize` / `tools/list` immediately, so a 13 s world
  load no longer makes the server read as dead to the client's keep-alive.

Run the tests with `PYTHONPATH=src python -m pytest packages/omnisim-mcp/tests -q`.
