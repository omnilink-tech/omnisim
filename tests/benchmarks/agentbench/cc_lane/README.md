# cc_lane — the Claude Code product lane (Phase W)

One cell = **one fresh headless Claude Code session in a staged product
workspace**, given exactly the task's `prompt.txt` text, then graded by the
shipped AgentBench graders. The prompt and the model are identical on both
arms; **the workspace is the variable** — the OmniSim product clone on one
arm, the upstream Webots R2025a install on the other, and (since 2026-08-09)
the bare `mujoco` package on a third. Condition name: `claude_code`; `sim` is
`omnisim`, `webots` or `mujoco`.

**The MuJoCo arm covers three tasks, not nine, and that is enforced rather
than documented.** `sims.SIMS["mujoco"].tasks` names `A1_husky_swarm_10`,
`R1_lidar_nav`, `R2_arm_reach` — the authoring tasks, which start from an
empty or world-free `initial/`. The other six ship a `.wbt` fixture with no
MJCF equivalent; `run_cell` refuses them through
`sims.require_implemented(sim, task)` before a workspace is staged, and a
campaign records them as `not_expressible` (a third status, never `blocked`
and never a row). That is SPEC 6.4: a missing fixture is **ours**, and
scoring it would print MuJoCo's name on our own gap.

**On MuJoCo a deliverable is a PAIR.** MJCF declares no controller and starts
no process, so the model alone cannot move: the artifact is `<task>.xml` and
the program that steps it is collected beside it as `<task>.py`.
`agents/external.ARTIFACT_NAME_BY_SIM` / `artifact_name(task, sim)` name the
file, `artifact_suffixes(sim)` drives discovery, and
`adapters/mujoco/launcher.find_driver` is the one rule for finding the driver
(matching stem, else the only `.py` beside the model, else a recorded refusal
— it never guesses). Collecting only the model is the same defect as lifting
a `.wbt` out of its project without `controllers/`, which zeroed R1 on the
OmniSim arm.

## Files

| file | role |
|---|---|
| `stage_workspaces.py` | builds per-sim workspace **templates** under a scratch root (default `%TEMP%\agentbench_cc`, never inside the repo), instantiates fresh per-cell copies, stages task files, applies the **answer-key redaction pass** (recorded as before/after diffs in the manifest), and owns the link-safe teardown |
| `run_cc_cell.py` | one graded cell end to end: preflight → session → metrics → artifact (per-task convention) → grade → row; takes the concurrency locks itself |
| `concurrency.py` | the plan §2.7 protocol: N-slot engine file-lock semaphore, per-task same-task exclusion lock, pre-cell resource guard (psutil else wmic/procfs), usage/rate-limit recognition for deferred attempts |
| `run_campaign_cc.py` | one LANE of the campaign: walks (task, sim) groups sequentially, **one cell per group** (the single-run protocol, SPEC §3.5 — the old n=5 / A1=10 repeats are gone), resume-from-state, publishes each finished group via the existing `--publish` path. Does not touch `campaign.py` (the API lane's driver) |
| `evidence.py` | the cell's evidence: unconditional workspace preservation (+ a live mirror), the `stream-json` session reader, and the read-only `--status` view over both |
| `test_cc_lane.py` | unit tests: include/exclude correctness on a synthetic tree, junction teardown safety on a dummy tree, transcript tool-call counter, env scrub, row merge, artifact discovery, redaction rules, lock semantics, rate-limit recognition |
| `test_cc_evidence.py` | regressions for the four `r1_3arm_20260810` defects: the cmd.exe argv truncation, grading from stdout instead of the workspace, a starved session published as an agent FAIL, and a cell that could not be inspected while running |
| `test_cc_isolation.py` | regressions for the nine `20260812_round1` isolation defects: cross-cell process and port reaping, a teardown that ate a live cell's deliverable, grading on a `.capture_*` sibling, a killed session mis-read as a rate limit, a preserved workspace without the deliverable, and the recorded search-visibility bias |

## Staging

**OmniSim workspace** = the repo's *tracked* files (`git ls-files`; untracked
scratch can never leak) **minus the answer key**:

- `tests/benchmarks/agentbench/**` (graders, oracle solutions, thresholds,
  task worlds),
- `docs/developer/agent-edge-validation-plan.md`,
  `docs/developer/webots-control-baseline.md`,
  `docs/developer/harness-latency-2026-07-31.md` (names the task worlds),
- `CHANGELOG.md` (names the C2 defect verbatim),
- private trees (`social/`, `cloud/`, the roadmap doc).

The multi-GB runtime dirs — `msys64/`, `projects/`, `lib/`, `resources/` —
are **directory junctions** into the real tree, not copies (none contains
answer-key material; `lib/` must be a junction because libController is an
untracked build output a tracked-file copy would miss). `python -m omnisim
doctor --strict` passes inside an instance with `OMNISIM_HOME=<instance>`
(verified 2026-08-01).

**Answer-key redaction (the 2026-08-01 ruling — supersedes the old
"AGENTS.md §3b ships verbatim" disclosure):** benchmark **self-references**
in staged docs are answer-key and are removed by `redact_staged_docs()` —
sentences/clauses naming agentbench task ids (`A1_`/`B1_`/… patterns and
task-shaped short spellings like "the C2 pair") or naming AgentBench itself;
**capability documentation stays** (everything `--fail-on-runaway` does
ships untouched; only the clause naming the task goes). Every removal is
recorded in the staging manifest (`redactions`) as an exact before/after
diff and rides into every row (`agent_artifacts.staging_redactions` count +
`known_disclosures`).

**Webots workspace** = a small dir with a directory symlink
`webots-R2025a` → `\\wsl$\Ubuntu-22.04\opt\upstream-webots\R2025a` (the
verified install, `adapters/webots/BRINGUP.md`; its own `docs/` ship inside
the install tree). Nothing authored by us enters the workspace beyond the
task files — our text there would contaminate the product comparison.

**MuJoCo workspace** = an empty dir plus the task's staged files. On this arm
the product is a *package inside the cell's interpreter* (`mujoco` 3.8.1,
selected by `$AGENTBENCH_MUJOCO_PYTHON` else the parent's `sys.executable`),
not a tree on disk, so there is no install to expose and nothing of ours to
stage or redact.

**Templates hold plain files only.** Junctions/symlinks are recorded in the
sibling `*.manifest.json` and materialised per instance, so instantiation can
never recurse through a junction, and **teardown severs every link with
`rmdir` semantics before any recursive delete** — nothing is ever deleted
*through* a junction (property proven on dummy trees in `test_cc_lane.py`).

```bash
python tests/benchmarks/agentbench/cc_lane/stage_workspaces.py --sim both
```

## Running a cell

```bash
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --sim omnisim --task B1_overlap_audit
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --sim webots  --task B1_overlap_audit
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --sim mujoco  --task R1_lidar_nav
```

Two lanes may run in parallel on this machine (lane A = omnisim, lane B =
webots) — the cell takes its own locks: a per-task lock for the whole cell
(same-task cells never overlap) and an engine slot (max `--engine-slots`,
default 2) around engine-heavy phases. All lanes must share one
`--lock-root`. Rows produced while the other lane was active are flagged
`measured_under_concurrency` and their time columns are excluded from
latency statements (plan §2.3/§2.7). For whole-lane runs use
`run_campaign_cc.py` (resume + per-group publish).

> **Parallel lanes over ONE task do not parallelise.** The per-task lock is
> exactly what stops them: three lanes launched on the same task run strictly
> one after another. That is correct (same-task cells must never overlap) but
> it means an N-arm single-task campaign costs N × the cell budget in wall
> clock, not one. Budget for it — `r1_3arm_20260810` did not, and the third
> arm reached the front of the queue with three minutes left. Queue time no
> longer eats the agent's budget, so the cost now shows up as elapsed time
> rather than as a corrupted row, which is the right place for it.

Per cell:

1. **Preflight**: `claude --version` recorded; a **multi-line**
   `claude -p <probe> --output-format json` proves auth *and argv integrity*,
   and yields the default model id, which is then **pinned explicitly** with
   `--model` for the task session (or pass `--model` yourself). The probe is
   multi-line on purpose — see "The launcher" below.
2. **Fresh workspace instance** per cell (cells never share state); child env
   scrubbed of `CLAUDECODE`/`CLAUDE_CODE_*`, `AGENTBENCH_*`, `ANTHROPIC_*`
   (the API key stays absent — the session runs on the operator's
   subscription), `OMNISIM_*`, `WEBOTS_HOME`; `CLAUDE_CONFIG_DIR` is kept
   (auth locator). `OMNISIM_HOME=<workspace>` on the omnisim arm.
3. **Session**: `claude -p "<prompt.txt text>" --output-format stream-json
   --verbose --model <pinned> --dangerously-skip-permissions`, cwd = the
   workspace, 45-min default timeout, launched through the **real
   `claude.exe`** and never npm's `.cmd` shim. NDJSON events land in
   `<cell>/cc_stream.jsonl` as they happen. If the permissions flag is
   refused before Claude starts, the documented fallback
   (`--permission-mode acceptEdits` + a broad `--allowedTools` list) runs
   instead, and the mode that ran is recorded.
4. **Metrics**: cost/turns/duration/tokens from the stream's `result` event;
   `tool_calls` counted from the session transcript
   (`~/.claude/projects/**/<session_id>.jsonl`, assistant `tool_use` blocks),
   falling back to the stream when the transcript cannot be located.

### The isolation rules (2026-08-12) — a cell may not destroy another cell

The 2026-08-12 diagnostic round found that cells were reaping each other's
processes, deleting each other's deliverables, and being graded on files they
did not write. Nothing measured through this lane before these fixes is
trustworthy. Regressions: [`test_cc_isolation.py`](test_cc_isolation.py)
(43 tests; each names the cell in `results/cc_lane/20260812_round1_*` it was
written against).

* **Sweeps are scoped by OWNERSHIP, not by port or by name.** Every running
  cell publishes a claim (`concurrency.register_cell` → `<lock-root>/active/`).
  The port sweep reaps a listener that is ours (our process tree, or its argv
  names our workspace) or that **no live cell claims** — the leaked-harness
  case it was written for. A listener another live cell owns is recorded
  `skipped_other_cell` and left running. Measured cost of not doing this:
  `r4_omnisim_c1`'s post-session sweep killed a live `A1` cell's harness, both
  engines, ten controllers and three supervisors.
* **The pre-session repo sweep protects live work.** `window=None` used to take
  every eligible untracked file under the real `projects/` tree; it
  `preserved_and_deleted` a concurrent cell's live deliverable into the wrong
  cell's directory. It now runs with `protect_after_ts =
  concurrency.oldest_active_start(...)`: anything newer than the oldest live
  cell is `skipped_owned_by_active_cell`.
* **Agents may not terminate processes by name.** The containment guard denies
  `taskkill /IM`, `Stop-Process -Name`, `pkill`, `killall`, a bare
  `| Stop-Process`, and the compound "select by image name, then stop by pid"
  form that `a1_omnisim_c1` actually ran. PID-scoped kills and read-only
  enumeration stay legal. The refusal is product language and never names the
  benchmark.
* **The artifact is the agent's file, not the newest one.** Discovery skips
  every dot-prefixed sibling (`.harness_*`, `.capture_*`, `.omnisim_*`) and
  ranks a world outside a verification directory above one inside it, whatever
  the mtimes say. `a1_omnisim_c1` was graded on
  `.capture_newton_husky_swarm_drive.wbt` — the shipped 8-Husky demo plus a
  capture supervisor — and FAILED every assertion for an agent whose ten
  robots drove 5.14–30.27 m.
* **A killed session is not a rate limit.** `deferral_reason` reads the
  child's own stderr and result JSON, never our `launch_error` prose, and
  `429` is matched digit-bounded. Windows reports a killed process as
  `rc=4294967295`, and "4294967295" contains "429": `a1_omnisim_c2` deferred,
  tore down its workspace and slept 900 s holding the task lock. A cell in a
  backoff now records `deferred_until_utc` and `--status` says so.
* **The preserved workspace contains the deliverable.** `preserve_workspace`
  takes `newer_than` + `link_dirs` and copies session-window writes made
  *through* the `projects/` junction (listed as `link_window_files`; the link
  itself is still recorded as excluded). The live mirror carries the same
  window, so a cell killed at plateau keeps its evidence.
* **`--status` reports SESSION elapsed.** Cell elapsed and queue time are
  printed separately and never added into the clock a plateau is judged by.
* **The session's `~/.claude/projects/<slug>/` state is collected** into
  `<cell>/claude_home/` (`report.session_home`) — memory above all. Scoped to
  the slug derived from this cell's own workspace; the operator's own project
  slugs are never read.
* **Search blindness is recorded, not fixed.** `report.search_visibility`
  counts what a link-refusing walk finds against what is on disk. The runtime
  dirs are NTFS junctions and ripgrep/`find`/`grep -r` will not traverse a
  reparse point, so `Glob **/controllers/...` at the workspace root finds
  nothing while the same glob at the checkout finds it — 3/3 cells were
  distorted this way. See "Known design gaps" below.

### Known design gaps (not bugs — staging changes, costed)

* **Agents write their deliverable into the REAL repo through the `projects/`
  junction** (3/3 cells did), and **repo-wide search cannot see behind the
  junctions** (3/3 cells). Both have the same cause and the same fix: stage
  `projects/` as a real directory tree instead of a junction. Measured costs
  for that decision — `projects/` is 1.7 GB over 9,706 files; the 2,246
  text-shaped files are 94.9 MB (2,144 of them ≤256 KB = 25.0 MB). A hybrid
  (real dirs, real copies of text files, per-file symlinks for binary assets)
  would cost ~25–65 MB and ~7,500 symlinks per instance — but note that
  **ripgrep skips symlinked files as well as junctions** (verified), so only
  the *copied* half becomes searchable. Until then: writes are repatriated by
  the post-session sweep, preserved by the link window, and the blindness is
  on the report.

### The evidence rules (2026-08-11)

Three rules, each written after `r1_3arm_20260810` produced three cells and
zero usable evidence. Read [`evidence.py`](evidence.py) for the long form.

* **The workspace is the artifact of record.** It is mirrored into
  `<cell>/workspace/` every 20 s while the session runs and copied
  authoritatively when it exits — on **every** outcome, including blocked and
  crashed. Junctions and symlinks are recorded, never followed (the omnisim
  workspace junctions the real repo; the webots one symlinks the whole
  upstream install). Exclusions are enumerated in
  `<cell>/workspace/workspace_manifest.json`; nothing is silently dropped.
* **The session's stdout is metadata, not the verdict.** A cell whose agent
  left a gradeable deliverable is graded even if the session emitted prose,
  crashed, or returned nothing. The gap is recorded as
  `session_incomplete`.
* **Liveness gates publication.** `assess_liveness` has exactly two vetoes,
  and both are *our* failures: a session that never started, and one the
  scheduler starved (granted < 50 % of the task's declared budget). A session
  killed at its **full** budget stays an honest scored FAIL — SPEC 2.4. Every
  row carries the `liveness` block with its criteria so a reader re-checks
  rather than trusts.

Related: cell wall deadlines now **exclude** time spent queued behind another
lane's same-task lock (recorded as `budget.queue_waits`). It was included,
and that is why the webots arm of `r1_3arm_20260810` was handed 156.7 s of an
1800 s budget after waiting 42 of its 45 minutes.

### The launcher

`shutil.which("claude")` on Windows returns npm's **`claude.CMD`**, which is
not an executable: `CreateProcess` runs a `.cmd` through `cmd.exe`, and
cmd.exe ends the command line **at the first newline of any argument**. Every
task prompt here is multi-line, so the child used to receive line 1 and lose
every flag positioned after the prompt — `--output-format json`, `--model`,
`--dangerously-skip-permissions` — silently, with `rc=0`. `_claude_exe()`
resolves the shim to the real `claude.exe`, `_run_claude` **refuses** to
launch a multi-line argument through a `.cmd`/`.bat`, and the preflight probe
is multi-line so the failure mode is exercised before any tokens are spent.

### Watching a live cell

```bash
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --status <cell-dir>
python tests/benchmarks/agentbench/cc_lane/run_cc_cell.py --status <cell-dir> --json
```

Read-only: no locks, no signals, nothing written. Prints elapsed vs budget,
model actually running vs pinned, turns, the last tool calls with their
targets, the files appearing in the workspace, and the liveness verdict. It
reads the **live** workspace while it exists (located from the stream's
`init.cwd`, so it works from the session's first millisecond) and the
preserved copy afterwards — and says which.
   Unmeasured is null with the reason recorded — never invented.
   `metrics_source: "claude_code_headless_json"`.
5. **Artifact, per the task's deliverable convention** (agents/external.py
   is the authority). World tasks (C2/C1/B2/A1): the staged task world when
   the session modified it (the deliverable outranks newer verification
   scratch — the first webots smoke cell wrote a labelled-broken CONTROL
   copy last, and a newest-first rule collected it); else the newest `.wbt`
   modified after session start (links never traversed; instantiation
   preserves mtimes, so only session writes qualify); else the staged world
   unchanged. Answer tasks (B1/B3): the session's final message text is the
   artifact (`answer.txt`); the grader measures ground truth from the task's
   PRISTINE staged world, so a session cannot move the scene to match a
   wrong answer. B2 collects both (world + `answer.txt` for the committed
   proof, threaded via `AGENTBENCH_EXTERNAL_ANSWER`).
6. **Grading — real pipelines only.** OmniSim arm:
   `run_agentbench.py --agent external` with the `agents/external.py` env
   contract (`AGENTBENCH_EXTERNAL_ARTIFACT`, label `claude_code` → the row's
   condition). Webots arm: the Phase W launcher + AABB prober + the
   sim-neutral grader core, exactly as `preregister/run_oracles.py` grades
   its webots cells. MuJoCo arm: `adapters/mujoco/launcher.launch` runs the
   collected model **and its driver** under the grader-owned recorder, then
   the same sim-neutral core (no AABB probe — that arm's t=0 scan bounds
   every body and world geom in the run it already did).
7. **Row**: the grader's row with the CC metrics merged in (the verdict,
   assertions and measurements are never touched; the grading stub's own
   turn/tool counts are overwritten — with null if unmeasured), appended to
   `rows.jsonl` in the cell run dir, plus `cell_report.md`/`.json`.

## Discipline

- **Quota**: a cell consumes the owner's Claude subscription quota. Run each
  cell once; retry only when the launch itself errored before Claude
  started. A usage/rate-limit refusal is a **deferred attempt** — recorded,
  backed off (default 15 min) and retried automatically; it never counts as
  the cell's one run.
- **Concurrency is locked, not promised**: same-task cells never overlap
  (per-task lock); at most `--engine-slots` engine-heavy phases machine-wide
  (file-lock semaphore); rows measured while another lane was active are
  flagged and their time columns excluded from latency statements. Within a
  cell the grading launches still start only after the Claude Code child has
  exited (and the workspace is torn down first, so the grader can never read
  the session's scene).
- **Pinned versions**: every row records `claude --version`, the pinned
  model id, and the permission mode in `agent_artifacts.claude_code`.
- **Honesty**: rows come only from the real graders; no metric is ever
  fabricated; a missing transcript means `tool_calls: null` plus the reason.
- Smoke cells are **n = 1, exploratory** — quotable nowhere (SPEC §3.5).
