---
name: omnisim-world-builder
description: Build, inspect, run, and debug OmniSim robotics worlds through the first-party MCP tools. Use for requests involving OmniSim scenes, robots, cameras, contacts, physics checks, or screenshots; do not use for unrelated simulators.
---

# OmniSim World Builder

Use the `omnisim` MCP server as the live control surface. It proxies a local
OmniSim World Harness and does not send simulation data to OmniLink.

## Start from measured state

Call `harness_status` first. If it is unreachable, tell the user to run:

```bash
python -m omnisim harness
```

The `omnisim-mcp` command must also be installed or available from the
`packages/omnisim-mcp` source package. Do not invent scene state while the
harness is unavailable.

## Author and verify a world

1. Inspect the requested world and related controller files before editing.
2. Use `.omniworld` for new OmniSim worlds. Existing `.wbt` worlds remain
   readable, but do not give new files the legacy extension.
3. Call `load_world` with `light: true`. After an edit, call it again: the tool
   safely live-applies proven root-DEF pose changes and automatically reloads
   structural edits. Use `force_reload` only when a controller restart or full
   reparse is intentional.
4. Verify placement with `get_scene_tree` or `get_scene_node`, using geometric
   bounds when framing or spatial relationships matter.
5. Use `frame` to aim at a DEF instead of guessing camera coordinates. Confirm
   framing with `visible`, then capture a `screenshot` when visual evidence is
   useful.
6. Step the simulation only as far as the requested behavior requires. Inspect
   `get_events`, `get_diagnostics`, robot joints, and contacts for numeric
   evidence. A clean screenshot alone does not prove the physics is correct.

## Report honestly

Distinguish a world that loaded from a behavior that was physically verified.
Name the check you performed and any remaining limitation. In particular, do
not describe OmniSim's weight-bearing G1 walk as free-standing, and do not make
sim-to-real claims without physical-robot evidence.
