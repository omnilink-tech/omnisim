# OmniSim Codex plugin

Packages OmniSim's world-authoring surface for **ChatGPT and Codex**, the way
[`.mcp.json`](../../.mcp.json) at the repo root packages it for Claude Code.

## What it is

Four files and no new code:

| File | What it carries |
|---|---|
| [`.codex-plugin/plugin.json`](.codex-plugin/plugin.json) | Plugin manifest: name, version, licence, and the ChatGPT interface metadata (display name, category, default prompts, brand colour). Points at `./skills/` and `./.mcp.json`. |
| [`.mcp.json`](.mcp.json) | Spawns the MCP server: `python -m omnisim_mcp` with `PYTHONPATH=packages/omnisim-mcp/src`, harness at `http://127.0.0.1:6789`. |
| [`skills/omnisim-world-builder/SKILL.md`](skills/omnisim-world-builder/SKILL.md) | The guidance the model reads: call `harness_status` first, write `.omniworld` not `.wbt`, load with `light: true`, use `frame` instead of guessing camera coordinates, and report a load and a physical verification as different claims. |
| `skills/omnisim-world-builder/agents/openai.yaml` | Display strings for the OpenAI agent surface. |

**It wraps the same MCP server** — [`packages/omnisim-mcp`](../../packages/omnisim-mcp/) —
against the same harness, with the same 18 tools. It duplicates rather than diverges: there
is no second protocol, no second tool set, and no OmniSim-side code here at all. What the
plugin adds over the bare MCP registration is the `SKILL.md` guidance and the ChatGPT
interface metadata.

Simulation data stays local. The MCP server proxies a harness on `127.0.0.1`; nothing in this
plugin sends world or robot state to OmniLink.

## Prerequisites

The same chain the MCP server needs, because it is the same server:

1. **A working OmniSim install** — the Windows package, or a source build followed by
   `make -C src/omnisim bundle-newton-runtime`. A fresh `git clone` has no engine
   (`msys64/` is gitignored, 0 tracked files). Verify with `python -m omnisim doctor`,
   which prints a VERDICT line and exits non-zero when the install cannot run.
2. **A running harness**: `python -m omnisim harness`. Use the module form — the raw
   `scripts/harness/omnisim_harness.py` skips the Qt DLL path setup and fails the first
   world load on Windows.
3. **Pillow**, only if you want `render_stats`.

Full detail, including the Claude Code / Claude Desktop / Cursor registrations and the tool
table: [`packages/omnisim-mcp/README.md`](../../packages/omnisim-mcp/README.md).

## Install it

Point your Codex plugin loader at this directory in a checkout (or in an installed OmniSim
package — `plugins/omnisim` ships in the Windows package via
[`files_core.txt`](../../scripts/packaging/files_core.txt)). The manifest is
`.codex-plugin/plugin.json`; `mcpServers` and `skills` resolve relative to it.

The MCP entry uses `"cwd": "."`, so the loader's working directory must be the OmniSim
install root — `PYTHONPATH=packages/omnisim-mcp/src` is resolved from there.

This plugin is not published to any plugin registry. Install it from the checkout or package.
