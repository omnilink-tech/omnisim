# Codex CLI lane

This lane runs a fresh Codex CLI session in the same answer-key-filtered product
workspace used by the existing Claude Code lane, then grades the collected
artifact with the same AgenticSimBench adapter and grader.

It uses the official non-interactive shape:

```text
codex exec --ephemeral --json --sandbox workspace-write \
  --ask-for-approval never --ignore-user-config --ignore-rules \
  --skip-git-repo-check \
  --cd <workspace> --model <pinned-model> \
  --output-last-message <answer.txt> <prompt>
```

The JSONL event stream, stderr, exact argv, final answer, workspace, collected
project and grader row are preserved. Token counts and tool calls come from the
CLI events; absent values remain null.

For OmniSim, tracked `projects/` content is copied into the disposable Codex
workspace. It is deliberately not junctioned back into the source checkout,
so normal world and controller edits cannot mutate the campaign host's working
tree. Only the large `msys64`, `lib`, and `resources` runtime trees remain
linked and are outside the Codex write root.

Official interface reference: <https://learn.chatgpt.com/docs/non-interactive-mode>

## Run one cell

The staging root must be outside the OmniSim checkout, and the output directory
must not already exist:

```powershell
python tests/benchmarks/agentbench/codex_lane/run_codex_task.py `
  --sim omnisim `
  --task A1_husky_swarm_10 `
  --model <exact-model-id> `
  --root D:\agenticsimbench-codex `
  --out D:\agenticsimbench-results\a1-omnisim
```

The model is mandatory. Allowing a local default would make two rows that say
“Codex” potentially different experiments.

## Run the complete frontier campaign

```powershell
python tests/benchmarks/agentbench/codex_lane/run_codex_campaign.py `
  --sims omnisim,webots `
  --tasks frontier `
  --model <exact-model-id> `
  --root D:\agenticsimbench-codex `
  --out D:\agenticsimbench-results\codex-frontier-001
```

The campaign launches a fresh ephemeral Codex session for every cell, resumes
only cells that already have exactly one completed row, and emits one
`frontier_<sim>.json` report per arm. Missing simulator fixtures are recorded
as omitted capability gaps; they are never converted into agent failures.

## Containment and claim status

`workspace-write` limits writes; it is not treated as proof that every read was
confined. The runner therefore inspects Codex's JSONL tool events for access to
the real, unstaged checkout. A hit—or no parseable event record—blocks grading.

Even a clean PASS is marked exploratory unless `readiness.py` says the task is
publishable on that simulator. A current session that has already inspected
the benchmark cannot be used as a scored agent cell.

## Platform note

The Codex desktop app's packaged executable may not be launchable as a child
process through the Windows app alias (`Access is denied` was observed on this
machine). That is an execution-environment limitation, not an OmniSim or task
failure. Point `--codex` at a separately installed, executable Codex CLI when
the alias is not child-launchable.
