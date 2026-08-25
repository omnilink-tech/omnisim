# AgenticSimBench campaign — the key-day runbook

> **v0.3 instrument (freeze v3, 2026-08-09) — read this before anything
> below.** Three things changed and the rest of this file predates them:
>
> - **Model: `claude-opus-5`**, pinned in code as
>   `cc_lane/run_cc_cell.DEFAULT_MODEL`. It is no longer inherited from the
>   local CLI default, which is how the superseded grid ended up on a
>   different model than intended. Pass `--model` only for a deliberate,
>   recorded experiment.
> - **Budget: 45 minutes per cell, hard.** `timeout_s = min(3 x par_s,
>   2700 s)`, so every task now gets the full 3x-par rule and nothing is
>   truncated (SPEC §2.4). The ceiling has moved twice inside v0.3
>   (900 -> 1800 -> 2700) and it is GLOBAL, so **cells scored under
>   different ceilings are different experiments and may not be pooled**
>   (SPEC §2.4.1).
> - **ONE run per (task, sim). No repeats.** Owner's decision 2026-08-10:
>   repeats were the dominant token cost, so the suite buys a longer single
>   run instead of five short ones. The consequence is stated wherever a
>   number is: **a single run measures an OUTCOME and estimates NO
>   variance** — no pass@1, no percentage, no interval (SPEC §3.5). Every
>   row carries its own `protocol` block saying so.
> - **F cannot be evaluated under this protocol.** The frozen withdrawal
>   rule is arithmetic over n = 5; at n = 1 both channels are structurally
>   unevaluable and `f_eval.py` REFUSES rather than printing a withdrawal
>   the design could not have avoided (SPEC §3.5).
> - **Suite id: `agenticsimbench/v0.3`.** Rows tagged `agentbench/v0` are the
>   superseded Fable grid and may never be pooled with new rows.
>
> **Gate:** `pytest tests/benchmarks/agentbench/preregister/test_freeze.py`
> must be green before a single cell is scored. A red freeze test means the
> suite has no valid pre-registration and everything it produces is INVALID
> (SPEC §8.4).

The entire campaign is **one command** once an `ANTHROPIC_API_KEY` exists.
This file is the exact sequence for that day. The design being executed is
[docs/developer/agent-edge-validation-plan.md](../../../docs/developer/agent-edge-validation-plan.md)
(§2 the falsification design, §3 Phase W, §4 what may be said afterwards);
the contract is [SPEC.md](SPEC.md).

Moving parts (all in this directory):

| file | role |
|---|---|
| `campaign.py` | the driver — schedule, gates, retries, resume, publish |
| `f_eval.py` | the F evaluator (plan §2.4, implemented exactly) |
| `run_agentbench.py` | the existing cell runner the driver composes (its CLI is the contract) |
| `results/campaigns/<id>/` | gitignored scratch: rows.jsonl, state.json, per-run dirs |
| `results_published/<id>/` | the tracked "no row, no result" destination |

---

## 0. Pre-flight checklist (before exporting the key)

1. **`python -m omnisim doctor`** — binary present, engine↔libController ABI
   compatible (a stale libController silently hangs every controller while
   still printing PASS), ports 6789/6790 free.
2. **`python projects/policies/common/env_fingerprint.py`** — record the
   machine id. Every row carries it; a fingerprint that moves between
   conditions voids the campaign (plan §3 Phase W).
3. **No concurrent simulators.** Cells run strictly sequentially by design
   (plan §5.3 — wall clock measured under contention is not a measurement).
   Close any open OmniSim GUI; make sure no other lane on this machine is
   launching engines while the campaign runs.
4. **Light-mode note:** harness cells load worlds with `light: true` per the
   endpoint-latency ledger (plan §3 Phase R item 7 — `--light` took
   `/sim/step` from 27 s to 0.034 s on the 298-node world). The cost that
   remains in the rows is the agent's, not a known-fixable endpoint tax.
5. **Disk/results hygiene:** `results/` is gitignored scratch — rows only
   survive via the publish step (below). Do not hand-edit anything under
   `results_published/`.
6. Optional but cheap: `python tests/benchmarks/agentbench/run_agentbench.py
   --tasks all --agent all` — the Phase-0 oracle/null/wrong expectations
   should be green before tokens are spent.

## 1. Export the key

```bash
export ANTHROPIC_API_KEY=sk-ant-...        # bash
# PowerShell: $env:ANTHROPIC_API_KEY = "sk-ant-..."
```

The driver refuses scored runs (and `--smoke`) without it. `--dry-run` is
the only mode that runs keyless.

## 2. Smoke — one real n=1 cell (Phase R item 1)

```bash
python tests/benchmarks/agentbench/campaign.py --smoke \
    --model <EXACT-MODEL-ID> --campaign-id phasew_smoke
```

- The model id is **required and pinned** — the driver refuses a default.
  Use the exact dated id you intend to run the campaign with.
- The row lands in `results/campaigns/phasew_smoke/smoke/rows.jsonl`,
  marked `exploratory` — n = 1 is a demo, barred from every aggregate and
  every claim (SPEC §3.5). It is never appended to campaign rows.

**Review the smoke row before spending anything else.** The printout shows
what to check:

- `outcome` is a real verdict (not SKIPPED — SKIPPED means the credential
  did not reach the backend);
- `tokens_in` / `tokens_out` / `tokens_cache_read` / `tokens_cache_write`
  are real numbers, and `usd` is **non-null** — the Anthropic backend's
  `untested_until_credential` list (HTTP round trip, real usage, stop-reason
  paths) is exactly what this run retires;
- `t_agent_s` is present and less than `t_total_s`;
- `stop_reason` is `model_stopped` (a budget stop on a smoke run means the
  budgets need a look before the campaign).

The smoke run is also the **only honest wall-time estimate**: cells run
sequentially and per-run time is dominated by model latency, so campaign
wall time is unknown until smoke. Rough arithmetic for planning only:
70 runs × (smoke's `t_total_s`), plus grader/standalone passes on the
heavier tasks; budget a working day for the omnisim arm and re-estimate
after the first few cells.

## 3. The campaign

```bash
python tests/benchmarks/agentbench/campaign.py \
    --model <EXACT-MODEL-ID> --sims omnisim --campaign-id phasew_01
```

- Schedule: A1 × 2 conditions × 1, plus 5 Lane B tasks × 2 conditions × 1
  = **12 scored runs per simulator**, strictly sequential. (Plan §3.1's
  n=10 / n=5 schedule — 70 runs per simulator — is superseded by the
  single-run protocol, SPEC §3.5.) `--sims omnisim,webots` schedules 140 — but note the Webots
  execution path (plan Phase R item 4: launcher + Supervisor recorder) is
  not built yet; the omnisim arm may run first and the webots arm follow
  under the same pre-registration.
- **Flake budget** (plan §3 item 8): each run gets up to 2 launch retries on
  INVALID/ERROR; every attempt is appended to `rows.jsonl` (earlier attempts
  marked `campaign.superseded`), so retries are attributed, never silent. A
  cell over 20 % INVALID after re-runs is marked
  `published-as-unreliable` (SPEC §3.3) in `state.json` and
  `campaign_meta.json`.
- **Interruption:** Ctrl-C, a crash, or a mid-campaign credential failure
  (the driver stops on a SKIPPED scored row rather than burning cells) all
  leave `state.json` intact. **Re-run the identical command to resume** —
  the campaign continues from the next incomplete run and never re-runs a
  completed one. The driver refuses to resume with a changed model/sims
  configuration.

## 4. Where the rows land

- During the run: `results/campaigns/phasew_01/rows.jsonl` (append-only)
  plus per-run dirs under `cells/` with each child's trace, verdict and
  driver log. All gitignored scratch.
- On completion the driver **publishes automatically** via
  `run_agentbench.py`'s existing `--publish` mechanism:
  `results_published/phasew_01/rows.jsonl` + `publish_meta.json`
  (row count, sha256) + `campaign_meta.json` + `f_report.md`.
  Publishing refuses to overwrite an existing id.
- **Review and commit** `results_published/phasew_01/` — the "no row, no
  result" rule (plan §0.2): a number that is not a tracked row may not be
  quoted anywhere, `AGENTS.md` included.

## 5. The F evaluation

The driver invokes `f_eval.py` on the published rows automatically and
writes `results_published/<id>/f_report.md`. To re-run it by hand:

```bash
python tests/benchmarks/agentbench/f_eval.py \
    --rows tests/benchmarks/agentbench/results_published/phasew_01/rows.jsonl \
    --out  tests/benchmarks/agentbench/results_published/phasew_01/f_report.md
```

What it implements (plan §2.4, frozen): eligibility (≥ 1 pass by either
condition), the ≥ 3-of-5 definedness floor for cost ratios, the
`Rcalls ≤ 0.85 AND Δpass ≥ 0` counting gate, channels A/B (surface) and C/D
(comparative), the mechanical best-condition rule (near-ties go to the
cheaper condition), the death condition on aggregate totals, and the
three-way verdict — survive / narrow finding (four pre-written templates,
two adverse) / withdrawn — with denominators printed and every unevaluable
channel named in the verdict's opening sentence. Every number in the report
resolves to `rows.jsonl` line references (the traceability section).

The emitter **greps its own output** for the plan §4 forbidden-sentence
patterns and refuses to write a report containing one.

## 6. What may and may not be said afterwards

Read [plan §4](../../../docs/developer/agent-edge-validation-plan.md)
**before writing a single external sentence** — it is a list of exact
licensed and forbidden sentences per phase, and the report emitter enforces
the Phase-W forbidden list mechanically. The short form:

- If F **withdraws** a conjunct: plan §2.5's scripted statement is
  published, `AGENTS.md`/`README.md` framing is removed in the same commit,
  and a dated entry lands in `simulator-comparison.md` §9.2.
- If F **survives**: survival licenses *nothing* beyond plan §4's Phase-W
  text — quoting survival as an achievement is itself on the forbidden
  list. Confirmation is SPEC §9.3's business (n ≥ 20 within a CI width).
- Narrow findings publish in the holding channel's direction **with the
  adverse channel's full table printed**.
- Never: comparatives against simulators that were not run, anything
  "most agent-driveable", anything with "first", any unqualified "agents
  get more done on OmniSim", any Lane C count or competitor Lane C cost in
  prose. (The old "Phase W cells run ODE" caveat here is DEAD: ODE was
  deleted in `bdc02139` and Newton is the only backend, so v0.3 cells run
  Newton and the backend must be read from the `.newton.json` sidecar.)
- n = 1 rows (the smoke run) are exploratory and quotable nowhere.

## 7. The Claude Code lane (`cc_lane/`) — the product-workspace instrument

A separate instrument from the API campaign above: one cell = **one fresh
headless Claude Code session in a staged product workspace** (condition
`claude_code`), graded by the same shipped graders. The prompt (the task's
`prompt.txt`, verbatim) and the model are identical on both arms; the
workspace is the variable — the OmniSim product clone vs the upstream Webots
R2025a install in WSL. Full reference: [`cc_lane/README.md`](cc_lane/README.md).

**Staging** (idempotent; rebuild with `--force`):

```bash
python tests/benchmarks/agentbench/cc_lane/stage_workspaces.py --sim both
```

Templates land under `%TEMP%\agentbench_cc\` (never inside the repo): the
OmniSim arm is the repo's tracked files minus the answer key
(`tests/benchmarks/agentbench/**`, the validation-plan / control-baseline /
harness-latency docs, `CHANGELOG.md`, private trees), with `msys64/`,
`projects/`, `lib/`, `resources/` attached as junctions; `python -m omnisim
doctor --strict` passes inside an instance. The Webots arm is a near-empty
dir exposing `/opt/upstream-webots/R2025a` via a `\\wsl$` directory symlink.
Every workspace ships with a sibling `*.manifest.json` (file-list hash,
exclusions, junctions, known disclosures, **redactions**). Teardown severs
junctions with `rmdir` semantics before any recursive delete — never delete
a workspace by hand with `rm -rf`/`Remove-Item -Recurse`; use
`stage_workspaces.teardown_workspace()`.

**Answer-key redaction (the 2026-08-01 ruling, plan §2.7 / FREEZE.md
Amendment 2):** benchmark **self-references** in staged docs are answer-key
and are removed by the staging pass — sentences/clauses naming agentbench
task ids (`A1_`/`B1_`/`B2_`/`B3_`/`C1_`/`C2_` patterns and their task-shaped
short spellings) or naming AgentBench itself; **capability documentation
stays** (the `--fail-on-runaway` docs ship untouched; only the clause naming
the task goes). Every redaction is recorded in the staging manifest as an
exact before/after diff, published with the campaign.

**Cell command** (the per-task artifact conventions — world for C2/C1/B2/A1,
answer.txt for B1/B3, world+answer for B2 — are applied automatically):

```bash
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --sim omnisim --task C2_fall_through_floor
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --sim webots  --task C2_fall_through_floor
```

**Concurrency protocol (plan §2.7, Amendment 2 — enforced by
`cc_lane/concurrency.py`, not by operator discipline):**

- Cells may run in parallel across different **(task, sim) groups**; within
  a group they are **strictly sequential**, and **same-task cells never
  overlap** (per-task file lock held for the whole cell — plan §5.3's
  shared-global evidence risk).
- **Max 2 lanes on this machine, split by simulator** (lane A = omnisim
  cells, lane B = webots cells): a global file-lock semaphore of
  `--engine-slots` (default 2) engine slots, acquired around engine-heavy
  phases — the WHOLE session for omnisim cells (the agent may launch the
  engine at any point); the grading/recorder pass always. All lanes must
  share one `--lock-root` (default `<staging-root>/locks`).
- **Pre-cell resource guard**: a cell is skipped-and-retried-later while
  free RAM < 4 GB or CPU load is saturated (psutil, else wmic/procfs — no
  new hard dependency).
- Rows produced while another lane was active carry
  `agent_artifacts.measured_under_concurrency: true`; **their time columns
  are excluded from every latency statement** (plan §2.3; `f_eval.py`
  enforces it). Verdicts, `tool_calls`, tokens stand.
- **Rate-limit resilience**: a `claude -p` usage/rate-limit refusal is a
  **deferred attempt, not a failed run** — recorded, backed off
  (`--rate-limit-backoff-s`, default 15 min), and the same cell retried with
  a fresh workspace. Only sessions where Claude actually started working
  count against the one-run-per-cell rule.

**Campaign driver (one lane):** `cc_lane/run_campaign_cc.py` walks a lane
assignment of (task, sim) groups sequentially, ONE cell per group (SPEC
§3.5; `--n` / `--n-a1` still accept more for a deliberate, recorded variance
experiment), resumes from `state.json` on the identical command,
and publishes each finished group's rows through the existing
`run_agentbench.py --publish` path:

```bash
# lane A (omnisim cells) and lane B (webots cells), in two shells:
python tests/benchmarks/agentbench/cc_lane/run_campaign_cc.py \
    --campaign-id phasew_cc --lane A --sim omnisim
python tests/benchmarks/agentbench/cc_lane/run_campaign_cc.py \
    --campaign-id phasew_cc_webots --lane B --sim webots
```

**Quota note:** a cell burns the owner's Claude **subscription** quota (no
`ANTHROPIC_API_KEY` involved — it is scrubbed from the child env). Run each
cell once; retry only a launch that errored before Claude started, or a
deferred (rate-limited) attempt — the deferral machinery does this itself.
Smoke cells are n = 1, exploratory, quotable nowhere (SPEC §3.5).

**Pinned-version discipline:** preflight records `claude --version` and the
CLI's own default model id, but the session is pinned to the BENCHMARK's
model (`DEFAULT_MODEL`, currently `claude-opus-5`) — not to the CLI default.
The row records the pinned id, the CLI default and which won
(`model_pin_source`), so a machine defaulting elsewhere is visible instead of
silently changing the experiment; all of it lands in the row (`agent_artifacts.claude_code`) together with
the permission mode that actually ran. A campaign must hold all three fixed
across every cell; a version that moves between conditions voids the
comparison, same rule as the machine fingerprint.

Rows: the OmniSim arm's row comes out of `run_agentbench.py --agent external`
(the `agents/external.py` env contract; label `claude_code` becomes the
condition); the Webots arm's out of the Phase W launcher + prober + neutral
grader core, exactly as `preregister/run_oracles.py` grades its webots
cells. The Claude Code metrics (cost, turns, tool_calls from the session
transcript, tokens; `metrics_source: "claude_code_headless_json"`) are merged
into the grader's row without touching the verdict; unmeasured is null,
never invented.

## 8. Dry run (any day, no key)

```bash
python tests/benchmarks/agentbench/campaign.py --dry-run --sims omnisim
```

Walks the full 70-run schedule through the scripted/fake-sim path — no key,
no model, no simulator — exercising the scheduler, per-run isolation, row
plumbing, state/resume, the INVALID machinery and the F-evaluator hookup end
to end. Dry-run rows are SKIPPED and marked so the evaluator excludes them;
the resulting `f_report.md` demonstrates the unevaluable-channel reporting
path.
