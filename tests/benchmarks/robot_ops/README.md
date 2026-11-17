# Robot operations under change

A development benchmark for what happens **while** a robot works: standing
rules that must hold across turns, orders meant for later, interruptions
mid-motion, obstacles and shoves, and reports that must match what was
measured. [SPEC.md](SPEC.md) is the contract. [FINDINGS.md](FINDINGS.md) lists
what the first runs found in the product.

This complements [../robot_control/](../robot_control/), which grades single
turns, and does not replace it.

## Run

A built OmniSim that passes `python -m omnisim doctor` and a valid OmniKey
(every bridge requires one) are needed.

```powershell
python -m omnisim ops-bench validate
# Calibration: scripted correct + scripted wrong behaviour; no model calls.
python -m omnisim ops-bench run --arms oracle oracle_bad --out tests/benchmarks/robot_ops/evidence/<new> --key-file <omnikey-file>
# The OmniLink product arm: needs a connected model provider for any step the parser declines.
python -m omnisim ops-bench run --arms omnilink --out tests/benchmarks/robot_ops/evidence/<new> --key-file <omnikey-file> [--only <task-or-family> ...]
# Recompute every grade from saved traces, no simulator and no model.
python -m omnisim ops-bench regrade tests/benchmarks/robot_ops/evidence/<run>
# Oracle contract tests (engine-free).
python -m pytest tests/benchmarks/robot_ops/test_ops_bench.py -q --confcutdir=tests/benchmarks/robot_ops
```

Output directories are append-only and refuse to overwrite. Each contains:

- `manifest.json`: suite hash, git HEAD, dirty product sources and runner
  hashes;
- `rows.jsonl`: one row per episode, with the full pose trace, fired steps,
  replies, relay usage and bridge events;
- `episodes/*/`: that episode's engine log, Newton sidecar, relay trace and
  intent/journal directory.

Episodes run in REALTIME mode, one private engine each, and take 20â€“60 s.
Run one campaign at a time on a machine: concurrent engines distort
realtime.

## Status

Development only: the tasks were written by someone who had read the product
source. See SPEC.md, "Before a comparative claim".

## Native Codex comparison

The `codex_full` arm uses a signed-in Codex CLI app-server, with Codex owning
the conversation and tool loop. It receives only live operator messages, the
same robot brief and full robot tools. Shell, filesystem, web, plugins and
external MCP access are disabled for the tested session. It requires an empty
workspace outside this repository and uses the same gated bridge as the
existing full-tools arms.

```powershell
python -m omnisim ops-bench run `
  --suite tests/benchmarks/robot_ops/suites/shift_holdout_v1.json `
  --arms codex_full --codex-model gpt-6.1-sol --codex-reasoning high `
  --codex-workspace C:/benchmark-workspaces/codex-new-run `
  --repeat-base 0 --max-requests 1000 `
  --out tests/benchmarks/robot_ops/evidence/codex-new-run `
  --key-file <omnikey-file>
```

Use a new output and workspace for each independent repeat. The shift takes
30 minutes plus startup and reply drain. The reported model rounds count
native token-usage notifications; interrupted calls may lack final usage.
Codex uses the signed-in account and its billed cost is unavailable, not zero.
The runner reports Codex dollar cost as unavailable and rejects provider-rate
assumptions for this arm. Use `codex_usage` in `rows.jsonl` for token counters.
The frozen study predates that display correction; its legacy generic cost
summaries are omitted from the public archive.

The September 30 extension is retrospective: it uses GPT-6.1 Sol with high
reasoning, while the original study used Gemini 3.5 Flash. The same shift and
grader do not remove that model/runtime difference. Conditions, the narrowly
scoped transport amendment and retained attempts are documented in
[shift/CODEX_EXTENSION.md](shift/CODEX_EXTENSION.md). See the
[measured report](evidence/CODEX_SHIFT_V1_RESULTS.md) for results and evidence.

## Native Claude Code comparison

The `claude_full` arm is the Codex arm's counterpart. A signed-in, headless
Claude Code process (stream-JSON) owns the conversation and tool loop. The
robot tools reach it through a standard-library stdio MCP server
(`omnisim/ops_bench/claude_mcp_stdio.py`) that forwards each call to the
adapter, which applies the same gate, operator words, emergency stop,
interruption classifier and cancellation as `codex_full`. Every built-in tool,
setting source, skill, slash command and other MCP server is disabled. An
episode is an infrastructure ERROR unless the session reports exactly the
bridge tools, no memory files and no discoverable CLAUDE.md or AGENTS.md.

```powershell
python -m omnisim ops-bench run `
  --suite tests/benchmarks/robot_ops/suites/shift_holdout_v1.json `
  --arms claude_full --claude-model claude-opus-5-5 --claude-effort high `
  --claude-binary <claude.exe 2.1.280 or newer> `
  --claude-workspace C:/benchmark-workspaces/claude-new-run `
  --repeat-base 0 --max-requests 1000 `
  --out tests/benchmarks/robot_ops/evidence/claude-new-run `
  --key-file <omnikey-file>
```

Opus 5.5 needs Claude Code 2.1.280 or newer; `--claude-binary` defaults to
`claude` on PATH. Billed cost is unavailable, not zero, and the runner rejects
rate assumptions for this arm. For tokens, use the final `result` event's
`modelUsage` in `claude-events.jsonl`: the adapter's `claude_usage.tokens`
reads partial stream snapshots and undercounts output. Conditions are in
[shift/CLAUDE_EXTENSION.md](shift/CLAUDE_EXTENSION.md); results and evidence
are in the [measured report](evidence/CLAUDE_SHIFT_V1_RESULTS.md).
