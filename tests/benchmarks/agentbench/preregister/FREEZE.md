# AgenticSimBench pre-registration — FROZEN 2026-08-09, freeze v3

**Suite `agenticsimbench/v0.3`.** Freeze v3 is a SUITE VERSION BUMP taken with 35 scored runs on record — see the freeze v3 section at the end of this file for the model repin, the budget ceiling it set (15 minutes; **now 45 minutes, per Amendment 4**), the 7-comparator expansion and the recorded post-freeze drift. **Read the amendments after it: they are the current design.** The v1 and v2 sections below are retained verbatim as the audit trail.

This file executes item 3 of the programme's order of work
([`docs/developer/agent-edge-validation-plan.md`](../../../../docs/developer/agent-edge-validation-plan.md)
§6) and the SPEC's §6.2.1 pre-registration machinery: the campaign design is
frozen **before any scored model run**, its content is hashed file by file
into [`freeze_manifest.json`](freeze_manifest.json), and
[`test_freeze.py`](test_freeze.py) recomputes every hash on every test run
and fails on any drift.

- **Freeze date:** 2026-08-09 — **freeze v3**, plus **Amendments 1–5**
  (1 and 2 the same day, 3 on 2026-08-10, 4 on 2026-08-11, 5 on 2026-08-12),
  each a legal pre-run amendment recording zero scored runs at its own
  amendment time.
  v2 (2026-08-01) and v1 (same day, superseded by v2) are retained below
  as the audit trail; when they disagree with the v3 section, v3 wins —
  and **when the v3 section disagrees with a later amendment, the amendment
  wins**, because the amendments are the only forward-dated content here.
  ⚠️ Two figures in the v3 section are superseded by Amendment 4 and are
  deliberately left standing as history: the **15-minute** ceiling (now
  **2700 s / 45 min**) and the **n = 5 / n = 10** repeat design (now **one
  run per cell**).
- **Machine:** `9722d23d12a3` (RTX 3060 Laptop GPU, Windows 11 — `M1` in
  OmniBench's machine table)
- **Engine build:** `518a335e`
- **Hash rule:** SHA-256 over LF-normalised bytes (CRLF → LF), so a
  git-autocrlf checkout verifies identically; any content change still
  changes the hash. Recorded in the manifest's `meta.hash_rule`.

## Freeze v2 — 2026-08-01, supersedes v1

- **Reason: instrument change before any scored run.** The plan's
  **revision 3** (2026-08-01) makes the product-level Claude Code lane
  (plan §2.7) the Phase W headline instrument, retains the API-runner lane
  as the mechanism-isolation follow-up, and moves F-surface (conjunct i)
  explicitly to that retained lane (plan §2.4 scope note). The plan's §2 —
  frozen content — changed, so the freeze is re-executed as a version bump,
  never a silent edit.
- **Legality:** zero scored runs existed at amendment time (the credential
  gate below was never crossed), so this is the legal pre-run path — the
  same clause that legalised revision 2's redesign before v1.
- **Supersedes freeze v1** (2026-08-01, earlier the same day). For the
  audit trail, v1's `freeze_manifest.json` hashes to
  `76c2739a5cbebc0619936a5f62c2b06f3fa02feff04088de010a44588feefebd`
  (SHA-256 over LF-normalised bytes — the freeze's own hash rule).
- **What changed in the frozen content:** the plan file only — §2 gains
  §2.7 (the product-level lane: the `claude_code` condition, the
  answer-key quarantine, instrumentation honesty, the retained API lane)
  and the §2.4 revision-3 scope note (F-surface → API lane; F-comparative
  degenerates to the single `claude_code` condition in the product lane;
  the F rule text itself is unchanged). **No task, grader, oracle script,
  tool-set manifest, or oracle verdict changed** — v2's per-file hashes for
  all of those are identical to v1's.
- **Deliberately not frozen in v2, stated loudly:** the Claude Code
  instrument itself — it cannot be byte-hashed; the pre-registration pins
  its CLI version + model id and every row records them, with replication
  scoped "at this version" (plan §5.11); and the product lane's staging +
  wrapper code (`tests/benchmarks/agentbench/cc_lane/`, under construction)
  — instrumentation, same rule as the runner scaffolding. The **staging
  manifest** for the answer-key quarantine (what each staged workspace
  includes/excludes, with hashes) is published with the campaign; a cell
  whose workspace contained answer-key material is INVALID (plan §2.7).

## What is frozen

Every path below is hashed in `freeze_manifest.json` → `files` (built and
verified by the same code, `test_freeze.py`, so generator and guard cannot
disagree):

1. **The plan's §2** — `docs/developer/agent-edge-validation-plan.md`,
   section `## 2. The pre-registered falsification design` (lanes,
   conditions, decision metric, F, §2.5's scripted retreat, §2.6, and —
   from freeze v2 — §2.7's product-level lane and the §2.4 scope note). Two
   hashes are recorded: `sha256_section_2` (the frozen design — its drift
   voids the campaign) and `sha256_whole_file` (informational context).
2. **The contract** — `tests/benchmarks/agentbench/SPEC.md`.
3. **Every task**, both simulator arms — for each of the six
   `tasks/<id>/` directories: `meta.json`, `prompt.txt`, every file under
   `initial/` and `initial_webots/`, and the reference rosters
   (C1's `reference_roster.json`).
4. **Every grader** — all non-test modules in `graders/` (the six
   task entries, the six sim-neutral cores, and the shared grading
   infrastructure `evidence.py`, `physical.py`, `verdict.py`,
   `__init__.py`).
5. **The agents fixture modules** — `agents/__init__.py` (the registry with
   its `expect_failures` declarations), `null.py`, `a1_fixtures_extra.py`,
   `b1_fixtures.py`, `b2_fixtures.py`, `c1_fixtures.py`, and the Phase-0
   oracles `oracle_a1.py`, `oracle_b1.py`, `oracle_b3.py`, `oracle_c2.py`.
6. **The tool-set manifests** — `runner/manifests/*.json`. The tool-set
   hashes these carry ARE frozen content: they are what makes "we gave
   every simulator the same interface" checkable (SPEC §4.2).
7. **The Lane B oracle scripts** — `runner/scripts/oracle_*.json`, the
   deterministic instrument behind the plan-§2.1 lane test and the
   plan-§2.2 bridge guards.
8. **The oracle verdicts** — `preregister/oracle_verdicts.json` (lane test,
   granularity guard, distinctness), measured on this machine from real
   sequential runs of the committed scripts through the real runner and
   graders (cells in `preregister/runs/cells.json`, evidence under
   `preregister/evidence/`).

**Deliberately not frozen** (recorded so the omission is loud, not silent):
the runner scaffolding (`runner/*.py`, `agents/llm.py`, `agents/external.py`,
`agents/base.py`, `campaign.py`, `run_agentbench.py`) — instrumentation may
be repaired without changing the pre-registered design; the grader test
files; and `results/` (gitignored by design — campaign rows land in a
tracked path per §0.2's "no row, no result" rule, decided in Phase R).
A repair that changes any *frozen* file is not an instrumentation repair.

## The lane assignment (frozen content, plan §2.1)

- **Lane A — control:** `A1_husky_swarm_10` (n = 10; never in the decision
  set; always reported separately).
- **Lane B — the decision set, the only lane F reads:** `B1_overlap_audit`,
  `B2_subject_in_frame`, `B3_measure_and_report`, `C1_parse_error_fix`,
  `C2_fall_through_floor` (n = 5 per cell, both conditions, per simulator).
  Membership is confirmed by the committed oracle lane test
  (`oracle_verdicts.json`: every task completes on both simulators, worst
  ratio 1.6667 ≤ 3×) and awaits the reviewer countersignature named below.
- **Lane C — the capability frontier** (per plan §2.1, reported only as the
  quarantined capability-and-cost table, n = 3, never aggregated into F or
  any throughput number): the five-edit iteration loop (the SPEC's
  OmniSim-only annex task N3, re-expressed as a cross-sim outcome),
  snapshot/rollback experimentation, structured-diagnostics triage of a
  deliberately broken world, and live spawn/delete.

Moving a task across a lane boundary after this freeze **voids the
campaign** (plan §2.4 no-escape).

## The B1 disclosure repair (pre-freeze, same change as this freeze)

Both B1 initial worlds' headers used to name the overlapping pair, so the
task priced file-reading rather than measurement. On 2026-08-01, before
this freeze: the disclosure was stripped from both arms (comments and
`WorldInfo.title` only — **no pose or geometry changed**, so all committed
grader verdicts remain valid, re-verified by the grader suites and the
red-evidence coverage check); equivalent self-disclosures were stripped
from the B2 and B3 worlds (grading criteria, the initial camera's off-axis
angle, B3's answer values) under the same rule — **a world must not answer
its own task in comments; the prompt is the only place the question
appears**; and the B1 oracles were regenerated to measure (read the scene
for poses, read the robot model's own geometry source for the footprint,
compute pairwise clearances: 2 calls per arm), then re-run for real on this
machine. All lane-test ratios and the three verdicts were re-measured and
re-committed in `oracle_verdicts.json`.

## The withdrawal rule — plan §2.4, verbatim

### 2.4 F — the withdrawal rule (revision 2)

> Evaluated on **Lane B only**: 5 tasks, both conditions, n = 5 per cell, per simulator.
> Lane membership, thresholds, and every definition below are frozen content (SPEC §6.2.1).
> The report header states plainly that the effect-size constants — `0.85`, the `+2`
> completion bar — are **conventional, not power-derived** (§7 Q2).
>
> **Eligibility and denominators, fixed in advance.** A task is **eligible** for a
> completion channel if at least one of the two compared conditions passed it at least once;
> a task where neither passed is excluded and reported as unresolvable by this design. A
> cost ratio is **defined** on a task only where **both** compared conditions passed
> **≥ 3 of 5** runs — a median over one or two passing runs is a coin flip, not a statistic.
> Every channel verdict below is stated **with its denominator printed**. A completion
> channel with fewer than 4 eligible tasks, or a cost channel with fewer than 3 defined
> tasks, is **unevaluable**; an unevaluable channel does not hold, and the F verdict's first
> sentence names every unevaluable channel.
>
> ### F-surface — conjunct (i), OmniSim cells only
>
> Per eligible task: **`Δpass` = passes(`shell+tools`) − passes(`shell`)**, each out of 5.
> Per defined task: **`Rcalls` = median tool_calls(`shell+tools`) ÷ median
> tool_calls(`shell`)** over passing runs. A task **counts** for the cost channel only if
> `Rcalls ≤ 0.85` **and** `Δpass ≥ 0` on that task — a cost win on a task the surface
> completes *less* is not a win (§1's gate).
>
> - **Completion channel (A) holds** iff `Δpass ≥ +2` on **≥ 2** eligible tasks and no
>   eligible task has `Δpass ≤ −2`.
> - **Cost channel (B) holds** iff **≥ 3** defined tasks count.
>
> **Verdict.** Both channels hold → conjunct (i) **survives**. Exactly one holds → a
> **narrow finding**, published in the holding channel's direction **with the other
> channel's full per-task table printed regardless of what it shows**. The four narrow
> templates are pre-written and two of them are adverse: *"same completions, materially
> fewer calls"*, *"more completions at no call saving"*, *"fewer completions"*,
> *"materially more calls"*. Neither holds → conjunct (i) is **WITHDRAWN**.
>
> Survival licenses **nothing** beyond §4's phase text — **"the claim survived F" is itself
> a forbidden sentence**; confirmation is SPEC §9.3's business (n ≥ 20 within a CI width of
> a competitor). The burden runs toward withdrawal on purpose: this revision's first draft
> withdrew only when both channels failed, and under a no-effect null that kept the claim
> alive roughly half the time (`Δpass ≥ +1` at n = 5 is a one-run fluctuation; two of five
> tasks showing it by chance alone is ~40–60%). Under the rule above, the rough null
> operating characteristics — stated here so they are on record; computed under a lognormal
> call-count model with 25–35% CV — are: full survival ~2–3%, a narrow finding ~25–30%,
> withdrawal otherwise. A low-power design can still honestly fail to find a real effect;
> that is what a *withdrawal* rule is for, and the n = 5 resolution limit (pass counts 0–5,
> wide CIs) is printed in the report header.
>
> ### The best-condition rule — mechanical, identical for every simulator
>
> A simulator's **best condition** = the condition with more **total Lane B passes** (out of
> 25); if the totals differ by **≤ 1**, the condition with the **lower sum of per-task
> median `tool_calls` over passing runs**. No tie-break toward `shell+tools`, ours or
> anyone's — near-ties go to the *cheaper* condition, which is the direction that disfavours
> the bridge's author (§5.9).
>
> ### F-comparative — conjunct (ii), best vs best
>
> Per eligible task: **`Δx` = passes(OmniSim best) − passes(competitor best)**, each out of
> 5. Per defined task: **`Rx` = median tool_calls(OmniSim best) ÷ median tool_calls
> (competitor best)** over passing runs; a task **counts** only if `Rx ≤ 0.85` **and**
> `Δx ≥ 0`. Same channel forms as F-surface:
>
> - **Completion channel (C) holds** iff `Δx ≥ +2` on ≥ 2 eligible tasks and no eligible
>   task has `Δx ≤ −2`.
> - **Cost channel (D) holds** iff ≥ 3 defined tasks count.
>
> Both hold → conjunct (ii) survives; exactly one → a narrow finding under the same
> four-template, adverse-table-printed rule; neither → conjunct (ii) is **WITHDRAWN**.
> (E1's conjunct (ii) is worded conjunctively in §1 to match: its former disjuncts — "same
> outcomes cheaper", "more outcomes at equal cost" — are exactly the narrow findings.)
>
> ### The death condition, defined so it can actually fire
>
> Per SPEC §6.1, unconditionally: if **total Lane B passes(competitor best) ≥ total Lane B
> passes(OmniSim best)** AND **median-of-per-task-median `tool_calls`(competitor best) ≤
> that of OmniSim best**, the entire agent-native claim is dead, and that is published as
> prominently as a win would have been. (Aggregate totals, deliberately — an
> all-five-tasks-simultaneously reading is nearly untriggerable at n = 5 and would make the
> loudest commitment in this file vacuous.)
>
> ### Composition and no-escape
>
> **E1 requires both conjuncts**; either withdrawal is published under §2.5, scoped to the
> conjunct that fell. **No escape clauses.** F is evaluated on the pre-registered task set,
> lane assignment, simulators, and metric definitions only. Adding a task after the runs
> start, **moving a task between lanes**, dropping a simulator because it went badly,
> re-reading `tool_calls` as "tool calls that mattered", relaxing the ≥ 3-passes definedness
> floor, or switching the decision metric after seeing the rows each **void the campaign**
> and force a suite version bump and a full re-run (SPEC §6.2.1).
>
> **F is first evaluable at the end of Phase W.**

*(Freeze v2 note: the plan's revision 3 adds a scope note directly beneath
this rule — F-surface is evaluated only in the retained API-runner lane;
F-comparative applies in the product-level lane with best-vs-best
degenerating to the single `claude_code` condition per simulator. The rule
text quoted above is unchanged by revision 3.)*

## The two open human gates

Frozen with the design, because neither can be closed by this repository's
own author or code:

1. **The non-OmniSim reviewer countersignature (plan §2.1, §2.2 / SPEC
   §6.2.4)** — required for every comparative cell before it is quotable.
   Awaiting countersignature *now*: the lane assignment above, and the
   recorded **NOT DISTINCT** bridge verdict in `oracle_verdicts.json`
   (distinctness guard: no Lane B task saved ≥ 15% tool calls under the
   Webots bridge vs shell; operator-measured, explicitly **not final**
   until a non-OmniSim reviewer renders it). The consequence rule if it
   stands is already frozen (plan §2.2 guard 3): the Webots within-sim
   delta publishes as descriptive only, and nothing is removed from
   F-comparative.
2. **The API credential** — no row in this repository has ever been
   produced by a real model call (plan §0.2). The credentialed smoke run
   (Phase R item 1) and every **API-runner-lane** scored cell are blocked
   on an `ANTHROPIC_API_KEY` the owner has not yet provided. (From freeze
   v2, the product-level lane runs through Claude Code's own auth and is
   not behind this gate — plan §3 Phase W.)

## The no-escape rule

Any post-freeze change to a file hashed in `freeze_manifest.json` — or to
the plan's §2 — **voids the campaign** and forces a suite version bump and
a full re-run (plan §2.4, SPEC §6.2.1). `test_freeze.py` enforces this
mechanically on every test run; re-running `test_freeze.py --write` after
the freeze commit is itself such a change and must never be done casually.
There is no legitimate reason to edit a frozen file "in place"; the honest
paths are (a) run the campaign as frozen, or (b) bump, re-freeze, re-run.

---

## Freeze v2 — Amendment 1 (2026-08-01): product-lane grading wiring

**What changed:** `agents/__init__.py` gained one registry entry —
`("C2_fall_through_floor", "external")`, the unknown-outcome external-agent
registration the Claude Code product lane (`cc_lane/run_cc_cell.py`) grades
through. No task, grader, fixture, oracle, prompt, world, or threshold
content changed; the sibling `ARTIFACT_NAME` entry lives in the deliberately
unfrozen `agents/external.py`.

**Why it is instrumentation, not evidence:** the entry mirrors A1's
pre-existing external registration and carries `expect_pass: None` — it
declares no expectation and validates nothing; it only lets the real grading
pipeline score an artifact produced outside the runner.

**Audit trail:** `agents/__init__.py` sha256 (LF-normalised)
`bd2bf2e40fe99a009ff7226c110d91cda92bc80bb76f1ef75eab5e987b29f1ec` →
`d18047be26f41c9cf864badd05c8ba1da3d097fdd4b87243cca4207745e59cc7`;
manifest regenerated. Scored runs at amendment time: **two exploratory n=1
smoke cells** (both PASS, quotable nowhere); zero campaign runs.

**Forward note:** wiring the remaining Lane B tasks (B1, B2, B3, C1) for
product-lane grading will require equivalent external registrations — that
will be **one** further recorded instrumentation amendment before the
campaign starts, not silent edits. *(Executed as Amendment 2 below.)*

---

## Freeze v2 — Amendment 2 (2026-08-01): product-lane campaign prerequisites

One recorded amendment, three items, executed together before any scored
product-lane campaign run. **Scored runs at amendment time: the same two
exploratory n=1 smoke cells as Amendment 1** (both PASS, quotable nowhere);
zero campaign runs; the API-credential gate remains uncrossed.

### Item 1 — the concurrency protocol (plan §2.7 gains it; §2.3 gains the time-exclusion note)

The plan's §2.7 now encodes, and `cc_lane/` now implements
(`concurrency.py`, `run_cc_cell.py`, the new lane driver
`run_campaign_cc.py` — `campaign.py`, the API lane's driver, is untouched):
parallel cells only across different (task, sim) groups with **strictly
sequential same-task cells** (per-task file lock, plan §5.3's shared-global
risk); **max 2 lanes on this machine split by simulator**, bounded by an
N-slot engine file-lock semaphore (default 2, `--engine-slots`) held around
engine-heavy phases (whole session for omnisim cells; grading/recorder pass
always); a pre-cell **resource guard** (RAM ≥ 4 GB, CPU not saturated;
psutil else wmic/procfs); **`measured_under_concurrency: true`** flagged
into `agent_artifacts` for rows produced while another lane was active, with
§2.3 (and `f_eval.py`, which nulls flagged rows' `t_agent_s`/`t_total_s` at
load) excluding their time columns from every latency statement; and
**usage/rate-limit deferral** — a limit-refused `claude -p` is a deferred
attempt, not a failed run: recorded, backed off (default 15 min), retried;
only sessions where Claude actually started working count against
one-run-per-cell.

### Item 2 — external-grading wiring for B1, B2, B3, C1

`agents/__init__.py` (frozen) gained **four** registry entries in one edit,
mirroring Amendment 1's C2 entry — `("B1_overlap_audit", "external")`,
`("B2_subject_in_frame", "external")`, `("B3_measure_and_report",
"external")`, `("C1_parse_error_fix", "external")` — all
`expect_pass: None` (unknown-outcome; they declare no expectation and
validate nothing). Artifact conventions live in the deliberately unfrozen
`agents/external.py`: **B1/B3** — the deliverable is the agent's ANSWER
(`answer.txt` is the artifact; its text feeds the grader's answer channel;
ground truth is measured from the task's **pristine** staged world, which
the session cannot move); **B2** — the modified world (same convention as
C2) plus the answer channel (`AGENTBENCH_EXTERNAL_ANSWER`) for the committed
proof; **C1** — the repaired world. `cc_lane/run_cc_cell.py` collects per
task accordingly and its webots arm mirrors `preregister/run_oracles.py`
per task (AABB probe for B1/B2/B3/C2, B2 shipped-world view evidence, C1
graded without an answer argument).

**Pre-quota wiring validation (no Claude sessions; synthetic artifacts built
from the frozen oracle scripts, graded through the real external path):**

| task | synthetic artifact | verdict |
|---|---|---|
| B1 | oracle answer text | **PASS 4/4** |
| B1 (negative) | wrong pair named | **FAIL, exactly B1.4** |
| B3 | oracle answer text | **PASS 4/4** |
| B2 | oracle world + oracle answer | **PASS 6/6** |
| C1 | oracle repaired world | **PASS 3/3** |

### Item 3 — the answer-key redaction ruling (ratified by the owner)

Benchmark **self-references** in staged docs are answer-key; **capability
documentation stays**. Encoded in plan §2.7 (one paragraph) and implemented
as a redaction pass in `cc_lane/stage_workspaces.py`: sentences/clauses
naming agentbench task ids (`A1_`/`B1_`/`B2_`/`B3_`/`C1_`/`C2_` patterns and
task-shaped short spellings) or naming AgentBench itself are removed from
every staged `.md`; `--fail-on-runaway`'s documentation ships untouched
minus only the clause naming C2. Every redaction is recorded in the staging
manifest as an exact before/after diff. Both templates re-staged 2026-08-01:
**7 redactions in 4 files** on the OmniSim arm (`AGENTS.md` ×4 — the
AgentBench routing-table row and three §3b/§8 sentences/clauses;
`docs/benchmarks/performance-comparison.md`, `docs/developer/README.md`,
`scripts/harness/README.md` ×1 each); zero residual task-id or AgentBench
matches in any staged `.md`. The webots arm stages nothing authored by us,
so nothing to redact. This **supersedes** the v2 "AGENTS.md §3b ships its C2
disclosure verbatim" known-disclosure; the manifests say so.

### Instrument-debris cleanup, recorded because the files were hashed

Three **gitignored, untracked** harness-injected sibling files had been left
inside task `initial/` dirs (from loading task worlds in place, against the
adapter's own staging rule) and were swept into the v2 manifest by
`test_freeze.py`'s tree walk:
`tasks/B1_overlap_audit/initial/..harness_six_huskies.wbproj` (sha256
`36dc297b8bb14ab49b4c7d7fbe3ca9c1964aaf34a4ab2b37838f190408443339`),
`tasks/B1_overlap_audit/initial/.harness_six_huskies.wbt`
(`9be74b32d295dc7bb50002e2aed2f88f2818371d1aa2fe2b2ee78fdd71fddd10`), and
`tasks/B2_subject_in_frame/initial/..harness_frame_the_cylinder.wbproj`
(`36dc297b…`, identical content). They are instrument debris, not designed
task content — **no tracked task file changed** — and they were actively
harmful: a stale `.wbproj` copied beside the world made the phase-B engine
launch flake (measured during Item 2's validation: rc=1 with it present,
clean without). Removed; `materialise_scratch` and the workspace stager now
skip harness siblings (`common/worldtext.is_harness_sibling`).

### Audit trail

- `agents/__init__.py` sha256 (LF-normalised)
  `d18047be26f41c9cf864badd05c8ba1da3d097fdd4b87243cca4207745e59cc7` →
  `855e434870931a0e7d6dba1238cb7ca86d69f404e0eb7e660a7246df4a0fa94c`.
- Plan §2 sha256 (the frozen design; §2.3 note + §2.7's two new paragraphs)
  `dbb757b16ccb70e372e724572facb52635894268d5dfae8d2bb0c29b7669a0c8` →
  `6fc13671552c555c97921a3ceb0d7a4d71cd704ea646643efcf3b75f80bf7fa7`;
  whole-file
  `4e24fe188c16a99d74478d63daf1d91d2e7b9c39701aa7a617a313f25a2a027a` →
  `83cef52413cc6f1eddd576f0a2c1bb490266adbd6bad84522f5e78759bff4ff6`.
- Manifest regenerated (`test_freeze.py --write`) with all of the above;
  the three debris paths left the manifest with the files.
- Unfrozen instrumentation touched in the same change (listed for
  completeness, not hashed): `agents/external.py`, `cc_lane/*`,
  `run_agentbench.py` (the harness-sibling skip), `common/worldtext.py`,
  `f_eval.py`, `RUNBOOK.md`.

---

## Freeze v3 — 2026-08-09, supersedes v2 (SUITE VERSION BUMP)

**This is not a legal pre-run edit.** Freeze v1 and v2 were both executed while
**zero** scored runs existed — that clause is what legalised them. It does not
apply here: **35 scored cells existed at amendment time** (the Phase W grid,
2026-08-01). Under SPEC §6.2.1 a post-hoc change to frozen content therefore
"forces a suite version bump and re-runs everything", and that is exactly what
this is.

- **Suite id:** `agentbench/v0` → **`agenticsimbench/v0.3`**
- **v2 manifest hash, for the audit trail:**
  `4661c893aae69a5f1577c7b784a781fa95edb56268f9a72bcb23ed6f907be639`
  (SHA-256 over LF-normalised bytes — the freeze's own hash rule)
- **Scored runs at amendment time:** **35**
- **Disposition of those runs:** **preserved, never deleted, never pooled.**
  They keep suite id `agentbench/v0` and may be quoted only as what they are —
  a different model, on a budget three times longer, against two simulators.

### What changed, and why each alone forces the bump

**1. The model is repinned: `claude-fable-5` → `claude-opus-5`.**
Owner decision, 2026-08-09: Claude Code Opus is the model for all benchmarks
from here. A campaign that mixes models is not a comparison (SPEC §4.4), so
this alone ends the v0 grid.

It also fixes a latent instrument defect. The model was never actually pinned:
`run_cell` did `pinned = model or preflight["default_model"]`, i.e. it
inherited whatever the local CLI happened to default to. The v0 grid was
scored on Fable because that was the machine's default, not because anyone
chose it. It is now `model or DEFAULT_MODEL` with the id in code, and rows
record the pinned id, the CLI's own default, and which won
(`model_pin_source`) — so a machine that defaults elsewhere shows up in the
row instead of silently changing the experiment.

**2. The wall-clock budget is capped at 15 minutes per cell.**
Owner decision, 2026-08-09: bound campaign cost. `timeout_s = min(3 × par_s,
900 s)`, enforced in three places (SPEC §2.4).

This too fixes a latent defect: **the per-task `timeout_s` was never
enforced.** It was written into `meta.json`, recorded in the row, and then
ignored — the actual kill used a single global `DEFAULT_TIMEOUT_S = 45 min`.
So every v0 cell had 45 minutes regardless of what its task declared, and the
`timeout_s` field in those 35 rows describes a limit that was not applied.

Effective budgets, with the truncation disclosed rather than hidden by
rewriting `par_s` downward:

| task | par | v0 declared | v0 *actual* | v0.3 | ratio |
|---|---|---|---|---|---|
| A1 | 720 s | 2160 s | 2700 s | **900 s** | 1.25× par |
| C2 | 600 s | 1800 s | 2700 s | **900 s** | 1.50× par |
| C1 | 480 s | 1440 s | 2700 s | **900 s** | 1.88× par |
| B1 | 300 s | 900 s | 2700 s | **900 s** | 3.00× par |
| B2, B3 | 240 s | 720 s | 2700 s | **720 s** | 3.00× par |

Budget-exhaustion `FAIL`s are expected to rise on A1/C1/C2. That is an
accepted, recorded cost of the cap, not a regression to explain away later.

**3. The comparator set expands: 2 → 7 primary + 3 extended.**
The frozen design now names OmniSim, upstream Webots, MuJoCo, Gazebo Jetty,
Isaac Sim 6.0, CoppeliaSim and Genesis, with PyBullet, SAPIEN and Newton as a
separately-published extended set (SPEC §6.1). Five primary arms are
**declared and unbuilt**; `sims.py` names each one's blocker, and
`require_implemented()` refuses them with that blocker rather than pretending
they were never in the design.

`tests/benchmarks/agentbench/sims.py` joins the frozen file set for this
reason: the comparator set is design content, and an arm appearing or vanishing
between runs would change what the leaderboard means without changing a single
task.

**4. The platform-accessibility programme is frozen before it is run.**
SPEC §§10–11: the P0–P6 ladder, the outcome taxonomy (`PASS-BELOW-MIN`,
`UNSUPPORTED-OS`, `OOM-VRAM`, …), the G0–G6 staged gate, the resource
ablations, the OS matrix and the four platform task prompts. **No platform cell
has been run.** It is frozen now precisely so no result exists to shape it.

### Recorded drift: five frozen files moved after v2, with no amendment

Between the 2026-08-01 freeze and 2026-08-08, five hash-frozen files changed
through commits doing unrelated and individually correct work. The guard
(`test_freeze.py`) caught every one and had been failing ever since; **the
failure was simply not acted on.** Recorded here because an undisclosed drift
is worse than the drift:

| file | commit | change |
|---|---|---|
| `B1/initial/six_huskies.wbt` | `9db0e1628` | `defaultPhysicsBackend "ode"` → `"newton"` |
| `B2/initial/frame_the_cylinder.wbt` | `9db0e1628` | same |
| `C1/initial/parse_error.wbt` | `9db0e1628` | same |
| `C2/initial/fall_through.wbt` | `9db0e1628`, `187a9baab` | same, **plus the floor moved z=0 → z=0.5** |
| `C2/initial_webots/fall_through.wbt` | `187a9baab` | floor z=0 → z=0.5, mirroring the OmniSim arm |
| plan §2 | `88c3f2290`, `3d1f88514`, `cbb1a1675` | ODE-deletion doc reframing |

**A second, downstream drift followed from the first and was also unactioned:**
four frozen oracle scripts (`oracle_b2_omnisim`, `oracle_c1_omnisim`,
`oracle_c2_omnisim`, `oracle_c2_webots`) embed the task world text, so they
went stale the moment those worlds changed. `test_oracle_scripts.py` had been
failing on it with the exact remedy in its own assertion message
(*"re-run preregister/gen_oracle_scripts.py"*). Regenerated at v3 and
re-hashed. Two guards, both correct, both red, both ignored — which is the
whole argument for SPEC §8.4 making a red guard a release gate rather than a
test result.

The ODE-pin retirements were **forced**: ODE was deleted from the engine in
`bdc02139` (2026-08-08), and a world still pinned to `"ode"` is registered with
no solver at all — it would not be simulated. Keeping the frozen pins would
have produced worlds with no physics.

**The C2 geometry change is the serious one, and it invalidates C2's v0
column.** With the floor at z = 0, the engine's implicit z = 0 ground plane
held the crate at a rest height *inside* C2.5's 0.15 m tolerance, so the
**unfixed** world passed 5/5 (measured 2026-08-08). C2's 5/5-on-both-arms in
the Phase W grid was scoring the tolerance, not the agent. The floor now sits
at z = 0.5 (top 0.55), the invariant `floor_top > REST_TOL_M` is pinned by
`test_c2_discriminates.py`, and the oracle/null gate is what should have caught
this — SPEC §7.1's rule that *no task may be passable by doing nothing* is
exactly the property C2 lacked.

SPEC §8.4 is the process rule added so this cannot recur silently: a red
freeze test is a **release gate**, frozen paths are amended rather than edited,
`tests/benchmarks/**` is opt-in for tree-wide sweeps, and drift is disclosed
even when we believe it changed nothing.

### The v0 grid, as it now stands

| task | omnisim | webots | status under v0.3 |
|---|---|---|---|
| B1 overlap | 3/5 | 4/5 | superseded; re-run |
| B2 frame | 0/5 (3 INVALID) | 4/5 | superseded; re-run |
| B3 measure | 2/5 | 1/5 | superseded; re-run |
| C1 parse fix | 5/5 | 5/5 | superseded; re-run |
| C2 physics fix | 5/5 | 5/5 | **invalid as measured** — task did not discriminate |
| A1 swarm | 0/7 | 0/10 | superseded; re-run |

Nothing in that grid may be quoted as an `agenticsimbench/v0.3` result, and the
C2 row may not be quoted at all except as the worked example of a
non-discriminating task.

### Gates carried forward unchanged from v2

The two open human gates, the countersign requirement, the no-escape rule and
the withdrawal rule (plan §2.4) are unchanged and still bind. Withdrawal
remains a live possibility and publishes either way.

### Freeze v3 — Amendment 1 (2026-08-09): the model pin, evidenced

**Scored runs at amendment time: none.** One **exploratory n = 1 smoke cell**
exists (C1 / webots / claude_code, PASS) — quotable nowhere under SPEC §3.5,
the same standing that legalised freeze v2's Amendments 1 and 2. This is
therefore a legal pre-run amendment, not a second suite bump.

**What changed:** SPEC §4.4 gains the measured evidence that the model pin is
load-bearing. Nothing else — no task, grader, oracle script, oracle verdict,
comparator entry or budget moved; their v3 hashes are unchanged.

**The evidence.** The smoke cell recorded:

```
pinned_model      claude-opus-5
cli_default_model claude-opus-4-8[1m]
model_pin_source  DEFAULT_MODEL
cli_version       2.1.179 (Claude Code)
```

The benchmark machine's CLI defaults to **Opus 4.8**. Under the pre-v0.3 code
(`model or preflight["default_model"]`) that cell would have executed on Opus
4.8 and been filed as an Opus 5 result, with nothing in the row to reveal the
substitution — the identical failure mode that put the v0 grid on Fable, still
live and still silent eight days later.

This is why the row carries all three fields rather than just the winner: the
pin *fixes* the substitution, but only the record makes it *detectable*. A
future row whose `model_pin_source` is `DEFAULT_MODEL` while `cli_default_model`
differs from the pinned id is evidence the pin is actively holding on that
machine.

**Instrument note carried forward from v2:** the Claude Code instrument still
cannot be byte-hashed, so replication stays scoped "at this CLI version and
this model id", both recorded per row (plan §5.11).

---

### Freeze v3 — Amendment 2 (2026-08-09): R1's grader, contract and the three arms' gates

**Scored runs at amendment time: none for R1.** Its cells to date are
exploratory (`agentbench/v0.3` pilots, every one recorded FAIL with
`budget_exhausted`), and the task's own `status` has said "may not be
published" throughout. This is therefore a legal pre-run amendment, not a
suite bump.

**Frozen files changed:** `graders/r1.py`, `graders/r1_core.py`,
`tasks/R1_lidar_nav/meta.json`.

**What changed and why — all of it measured:**

1. **The anti-hardcode mechanism was REPLACED, not tuned.** The seeded
   perturbation shipped in `0f889b21e` does not work: over 510 graded MuJoCo
   runs it catches a memoriser 15% at 0.6 m and 56% at 1.8 m, but against an
   OPTIMAL fixed path it saturates at ~51% near 3.0 m and then DECLINES, while
   layouts start becoming fatally illegal (25% at 3.0 m). Replaced by
   grade-time placement (`r1_core.sample_layout`): **79.2% catch (95/120)**,
   **120/120 honest-oracle pass**, 500/500 legal layouts, and no fallback to
   the published layout.
2. **Two defects that would have fired the moment any placement was wired.**
   R1.5 took its hit set from obstacles matched against the PUBLISHED layout,
   so moving them emptied it — measured, the memoriser struck an obstacle in
   20 runs and R1.5 counted **0**; now 95/95. And `MIN_PATH_LENGTH_M = 11.5`
   was a published-layout constant that failed honest agents on a shorter
   legal route (12.5% of oracle runs); the floor is now derived from the
   graded layout.
3. **`contact_steps: 0 -> -1`.** A zero window samples nothing, and R1.5
   refuses to credit an unmeasured channel — so even the honest oracle failed
   it (5/6 at 0, 6/6 at -1, measured on the webots arm).
4. **R1.3's `solids` list** was added so the obstacles are scanned at all,
   then had to be reconciled with the name-free scan: `--solids=` claims node
   ids and the scan skips claimed bodies, so a world using the PUBLISHED names
   read `found: 0`. Both paths now feed the roster.

**Gate status recorded with this amendment (SPEC 7.1, per (task, arm)):**

| arm | null/blind FAILs | oracle PASSes 6/6 |
|---|---|---|
| mujoco | yes | **yes** (mj_ray, 20/20 unseen layouts) |
| webots | yes | **yes** (SickLms291, 17/17 unseen layouts) |
| omnisim | yes | **NO — blocked by an engine defect** |

**The OmniSim blocker is not the task.** On the build of 2026-08-09 13:43
(`f45e0b652`) a motor target set AFTER Newton world-finalize has no effect —
`setVelocity`, `setPosition` on a limited motor and `setTorque` alike. Four
commanded wheel speeds (none / 6.0 / **0.0** / 12.0 rad/s) produced
0.997 / 1.026 / **1.034** / 1.041 m/s; commanding zero does not stop the
robot. It reproduces on a stock `URDFRobot` Husky, and the engine LOGS the
changed target arriving at `OmNewtonBackend` while nothing changes, so the
loss is downstream of `setJointTargetVelocity`. Repro:
`adapters/omnisim/omnisim_lane/controllers/r1_drive_probe/` (7.6 s, free).
Prime suspect: `55164f986` ("change-detect motors"), whose own comment states
the precondition this symptom would violate. **R1 therefore cannot be scored
on the OmniSim arm until the motor path is fixed**, and no R1 number from any
arm may be published while one of the three cannot express the task.

**Still unbuilt, and why R1 stays unpublishable:** the per-arm INJECTION step.
No arm yet places the drawn layout into the agent's authored world, so a
campaign run today would still score the published layout and a memoriser
would still pass 6/6. Every rate above was measured by placing layouts
directly on the MuJoCo lane.

---

### Freeze v3 — Amendment 3 (2026-08-10): the OmniSim R1 rover could not roll

**Scored runs at amendment time: none for R1**, unchanged from Amendment 2 —
its cells remain exploratory and `tasks/R1_lidar_nav/meta.json` still carries
`status: "NOT READY FOR A PUBLISHED NUMBER"`. This is therefore a legal
pre-run amendment, not a suite bump.

**Frozen file changed: `preregister/oracle_verdicts.json`, and only that one.**
The change that *caused* it is in files this manifest does not hash — the five
R1 OmniSim lane worlds
(`adapters/omnisim/omnisim_lane/worlds/r1_{null,blind,oracle,probe,drive_probe}.wbt`)
and `adapters/omnisim/test_r1_discriminates_omnisim.py`. They are fixtures and
instrument, not pre-registered design content, so no hash moved with them; they
are named here anyway, because SPEC §8.4(4) asks for drift to be disclosed even
where we believe it changed nothing, and here it demonstrably changed the
recorded measurements.

**What was wrong: the rover's wheels had never rolled.** Amendment 2 recorded
the OmniSim arm as blocked by an engine defect. That was half the story. There
were **two stacked defects**, and the second one is ours:

1. **ENGINE — fixed in `3cf70f120`.** The batched-substep path decided whether
   to copy newton's `control` into `mj_data.ctrl` by re-reading the
   `joint_targets` dicts *after* the same tick had already consumed and cleared
   them, so the copy never ran and whatever ctrl was latched at the batch
   transition drove the actuators for the rest of the run.
2. **THIS ARM'S ROVER — fixed by this amendment.** `RotationalMotor maxTorque
   0.4` cannot turn four 0.15 kg wheels under a 3 kg chassis. The four wheel
   hinges barely moved while the body **slid** across the floor, which is
   indistinguishable, from a controller's point of view, from a command being
   ignored. Repaired to `maxTorque 12` (the value proven rolling on
   `projects/robot_combat/worlds/tests/drive_test.wbt`), with `basicTimeStep
   16 → 8` and `newtonSubsteps 4`. The wheel *geometry* and the
   `Pose { rotation 1 0 0 π/2 }` wrap were correct and were not touched.

**Measured, all of it on `msys64/mingw64/bin/omnisim-bin-fixed.exe` (the build
carrying `3cf70f120`), machine `9722d23d12a3`,** so the engine is held constant
and the only variable is the world. `r1_drive_probe`, commanded wheel speed vs
measured ground speed (a rolling 0.08 m wheel at 6.0 rad/s is 0.480 m/s):

| commanded | broken rover | repaired rover |
|---|---|---|
| 6.0 rad/s | 1.026 m/s | **0.480 m/s** (exact) |
| 0.0 rad/s | 0.796 m/s **backward** | **0.000 m/s** |

The two gate cells, same harness the recorded cells used (60 s window,
`contact_steps=0`):

| cell | broken rover | repaired rover |
|---|---|---|
| `driver:null` | final (−4.0, −4.0), 0.0 m travelled, 0 contacts, **FAIL** | *identical* — **FAIL** |
| `driver:blind` | final (−1.9575, −1.9997), 8.455 m from goal, 3.079 m travelled, 1 × rover/OBSTACLE_2, **FAIL** | final (−1.8582, −1.8260), 8.2621 m from goal, **15.3061 m travelled**, 1 × rover/OBSTACLE_2, **FAIL** |

The broken-rover column reproduced the Amendment-2 record to four decimals,
which is what licenses reading the two columns as a controlled comparison.
**SPEC §7.1's failing half still holds and still fails for the stated reasons**
— `r1_null` fails R1.4/R1.5/R1.6, `r1_blind` fails R1.4 and R1.5 and is still
recorded striking the blocking obstacle by name. `r1_blind`'s R1.6 flips
False → True, which is an improvement in the fixture rather than a loss: it
used to fail R1.6 for barely moving, and now fails only for the two things the
fixture is designed to demonstrate.

**The passing half is STILL NOT established, and the reason has changed.**
`r1_oracle` was run for the first time (n = 1, same window): it now **steers** —
23.5842 m of closed-loop path — and does **not** solve, finishing 5.7968 m from
the goal having clipped OBSTACLE_4 and OBSTACLE_1. So the blocker is now an
untuned navigator, not an engine that cannot express the task. Recorded as
`outcome: "FAIL"` on the oracle cell rather than left as `null`.

**One motor defect is NOT fixed and is documented rather than repaired:**
commanding *exactly* `maxVelocity` runs away instead of saturating (Newton's
MuJoCo converter never reads `velocity_limit`; OmniSim's post-step `|qd|`
saturation is the only enforcement). It bites the probe's uncommanded phase
too, since an uncommanded velocity motor sits at `maxVelocity` by default.
Raising the cap does not remove it, it moves it — measured at `maxVelocity 20`
the uncommanded phase peaked at 10.6 m/s and flipped the rover, versus 9.6 m/s
at 12 — so the field is **left at 12** and every driver commands below it
(`r1_blind` 8.0, `r1_oracle` ≤ 10, `r1_null` 0.0).

**Disclosed and deliberately NOT changed:** `tasks/R1_lidar_nav/meta.json`
(frozen) still says the OmniSim arm "is blocked by an engine defect … no
closed-loop navigator can be run there at all". That sentence is now false on
both clauses. It is left to the owning R1 workstream, which edited that same
field on 2026-08-10 for the injection step; amending it here would collide with
live work. **Until it is corrected, the meta and this file disagree and this
file is the later measurement.**

**Audit trail.** `preregister/oracle_verdicts.json` sha256 (the freeze's own
rule — LF-normalised, volatile fields stripped)
`bde830aec28a19abaaee34645a01889f30608ab71e1029268ed946b6149a791f` →
`a45f2b8ac6e9e728005afe87e597618cc93c11b8e47b007144464eb257d56b98`; manifest
regenerated with `test_freeze.py --write` from a tree verified green
immediately beforehand, so exactly one hash moved. Pre-amendment manifest
sha256 (LF-normalised), for the trail:
`8935d0b53bc8fa2d3aa64fcb84d817fe06229943c9240a72cc1a4ff1cacd40a9`.
No task, grader, prompt, oracle script, tool-set manifest, comparator entry or
budget changed; the plan's §2 is untouched.

---

### Freeze v3 — Amendment 4 (2026-08-11): one run under a 45-minute ceiling, a task-scoped publication bar, and R1's OmniSim oracle finally PASSes

**Scored runs at amendment time: ZERO — and this one was verified rather than
asserted, because it is the whole legality argument.** Under SPEC §6.2.1 the
count is what decides between a legal pre-run amendment and a suite version
bump; freeze v3 was a bump precisely because 35 scored cells existed. The
check made here, three independent ways, all agreeing:

1. **The tree's own publication rule is "no row, no result"**
   (`results_published/README.md`): a number that lives only in a commit
   message or an operator's note may not be quoted, and rows count only in a
   tracked path. There are **87 tracked published rows** — 73 under the
   superseded suite id `agentbench/v0` (freeze v3 preserved them, never pooled)
   and **14 under `agenticsimbench/v0.3`**: 11 `v03_pilot_*` cells plus 3 R1
   cells (`r1_fair_omnisim`, `r1_omnisim_fixed`, `r1_settled_omnisim`), one row
   each.
2. **Not one row anywhere in the tree carries a `protocol` block** — not the 87
   tracked rows, and not one of the 319 `rows.jsonl` files in the gitignored
   `results/` scratch tree either (`grep -rl '"protocol"' results/` → 0). The
   `protocol` block is the machine-readable marker of the protocol this
   amendment freezes, so **no cell has ever been scored under it.**
3. **Every one of the 14 was already barred from claims when it was produced.**
   The 3 R1 rows are barred by `tasks/R1_lidar_nav/meta.json`'s own `status`
   ("NOT READY FOR A PUBLISHED NUMBER; R1 MAY BE RUN EXPLORATORILY BUT ITS
   NUMBERS MAY NOT BE PUBLISHED"), unchanged since Amendment 2. The 11 pilots
   are single cells from campaigns named `v03_pilot_*`, and they were produced
   under the then-binding rule that an `n = 1` row is `exploratory` — the same
   standing that legalised freeze v2's Amendments 1 and 2 and freeze v3's
   Amendments 1–3.

**⚠️ Disclosed, not repaired: point 3's rule no longer says what it said.**
`results_published/README.md` still reads *"Rows with `n = 1` remain
`exploratory` and barred from published claims (SPEC §3.5)"*, and the §3.5 it
cites was **rewritten by this very amendment** into a protocol under which one
run per cell is the norm, not a disqualification. So that sentence is now
stale, and it is deliberately left standing rather than edited: it is the rule
the 14 rows were produced under and therefore part of the evidence that they
are exploratory. Repairing it is the §3.5 owner's call and must not be done in
the same change that would benefit from it. Until then, **the rows' standing
rests on point 2**, which is mechanical and does not depend on it.

**Frozen files changed: twelve.** `SPEC.md`, `sims.py`, all nine
`tasks/*/meta.json`, and `preregister/oracle_verdicts.json`. Verified by
running `test_freeze.py` before writing anything — the drift list is the
measurement, not the changelog — and the manifest diff after `--write` moved
exactly those twelve of ninety-one entries, with `meta`, `plan` and
`lane_assignment` byte-identical.

**Unhashed files changed alongside them, named because SPEC §8.4(4) asks for
drift to be disclosed even where we believe it changed nothing.** Three files
this manifest does not hash moved with this amendment, all of them instrument
rather than pre-registered design content:
`adapters/omnisim/omnisim_lane/controllers/r1_oracle/r1_oracle.py` (the
navigator rewrite, commit `dc9bf156f`, which is the *cause* of Change 3), and
two test files whose prose had gone stale in the opposite direction —
`adapters/omnisim/test_r1_discriminates_omnisim.py`, which **asserted** that
R1's passing half was missing and whose assertion is inverted here into one
that runs the oracle and requires PASS 6/6, and
`adapters/omnisim/test_r1_placement_omnisim.py`, whose docstring still said no
closed-loop navigator could steer on this arm. **A green test that asserts the
opposite of the record is the §8.4 failure mode wearing its other face** — not
a red guard nobody acted on, but a green one nobody reconciled. Left alone,
this amendment would have shipped a suite whose own passing test says R1's
oracle half is missing while `oracle_verdicts.json`, `readiness.py` and this
file all say it PASSes.

---

#### Change 1 — the measurement protocol (`SPEC.md` + all 9 `tasks/*/meta.json`)

Commits `6d5c6a79f` and `7fae23004`. Owner's decision, 2026-08-10: repeats are
removed and the wall-clock ceiling is raised, as **one trade** — repeats were
the dominant token cost, so a single longer run is cheaper than five shorter
ones.

| | before | after |
|---|---|---|
| `tasks.TASK_HARD_CEILING_S` | 1800 s (30 min) | **2700 s (45 min)** |
| runs per cell | 5 (A1: 10) | **1** |
| `budget.runs_per_cell` | *absent* | **1** |
| `budget.variance_measured` | *absent* | **false** |
| cell wall bound | `budget × 2.0` | **`budget + 900 s`** (additive) |
| SPEC §3.5 | "Repeats" (n = 5 / n = 10) | the single-run protocol + 6 binding reporting rules |
| SPEC §8.1.3 | "n = 1 is a demo" | replaced, not deleted |

Per task, measured from the tree rather than from the commit message
(`timeout_s` / `repeats_default` / `budget.hard_ceiling_s`):

| task | before | after |
|---|---|---|
| A1_husky_swarm_10 | 1800 / 10 / 1800 | **2160** / **1** / **2700** |
| B1_overlap_audit | 900 / 5 / 1800 | 900 / **1** / **2700** |
| B2_subject_in_frame | 720 / 5 / 1800 | 720 / **1** / **2700** |
| B3_measure_and_report | 720 / 5 / 1800 | 720 / **1** / **2700** |
| C1_parse_error_fix | 1440 / 5 / 1800 | 1440 / **1** / **2700** |
| C2_fall_through_floor | 1800 / 5 / 1800 | 1800 / **1** / **2700** |
| R1_lidar_nav | 1800 / 5 / 1800 | 1800 / **1** / **2700** |
| R2_arm_reach | 1800 / 5 / 1800 | 1800 / **1** / **2700** |
| R3_pick_and_place | 1800 / 5 / 1800 | 1800 / **1** / **2700** |

**Only A1's budget actually moved** (1800 → 2160 s): at a 2700 s ceiling the
`min(3 × par_s, ceiling)` rule truncates nothing, so A1 gets back the 3 × par
it was always owed and had been carrying the previous ceiling's truncation
baked in. Every other task already sat below both ceilings.

**The epistemic cost is recorded in the frozen text, not in a footnote.** One
run estimates no variance: a pass fraction, a `pass@1` and a confidence
interval are **undefined** at one sample, not merely wide, and the variance is
**unmeasured** rather than small. SPEC §3.5's six rules bind that (never report
a single observation as a rate; no interval on 1/1 or 0/1; every row states its
own sample count; every published table states the protocol in its *header*;
counts across tasks remain the primary reading; a margin of one task is not a
result).

**A frozen rule was made unevaluable and is disclosed rather than quietly
dropped.** The withdrawal rule F (plan §2.4, frozen, quoted verbatim earlier in
this file) is arithmetic over `n = 5`: `Δpass ≥ +2` out of 5 and a ≥ 3-of-5
definedness floor are both unreachable at `n = 1`, so F would withdraw every
time on the arithmetic alone — a property of the formula, never evidence about
the claim. **The plan's §2 is NOT amended here** (its hash is unchanged, and
re-deriving F is its owner's call); until it is, no F verdict may be computed,
quoted or read as a withdrawal, and `f_eval` refuses over single-run rows
instead of printing one. SPEC §6.2.8's reproduce-or-drop gate **survives**,
scoped to published comparisons.

---

#### Change 2 — the publication bar became task-scoped (`sims.py`)

Commit `a0bcc70ab`. (`fc10e741f`, the same afternoon, is named here only to be
excluded: it fixed a `NameError` in `readiness.py`, which is **not** a frozen
file, so no hash moved with it.)

| | before | after |
|---|---|---|
| `Sim.pending` | one free-text string per arm | a list of `Pending` items, each declaring `blocks` |
| publication scope | one arm-wide boolean | `Sim.publishable_for(task)` |
| narrowing an item | (not expressible) | **requires** `why_scoped`; the constructor raises without it |

The MuJoCo arm's three items were **scoped, never cleared or softened**:
`A1_gate` → A1 only, `A1_husky_analogue` → A1 only, `R2_gate` → R2 only. Net
effect over the 9 × 3 grid was exactly one cell — `(R1, mujoco)` became
publishable, because R1's own oracle/null gate is green on that arm and on
record.

It also closed a **false green**, which is the more important half.
`readiness._discriminating` used to accept a cell when the arm's free-text
`pending` contained both "gate" and the task id — and the MuJoCo arm's own
sentence *"A1_husky_swarm_10 and R2_arm_reach are still UNGATED"* contains
"gate". Over the 9 × 3 grid that prose sniff produced two greens, `(A1, mujoco)`
and `(R2, mujoco)`, **with no true positives**: it reported as gated exactly the
two cells the text said were not. Only a recorded verdict satisfies that gate
now, and both cells correctly read `NO`.

---

#### Change 3 — R1's OmniSim oracle PASSes, and the gate closes (`preregister/oracle_verdicts.json`)

The cause is commit `dc9bf156f`, which touches **one file this manifest does
not hash** — `adapters/omnisim/omnisim_lane/controllers/r1_oracle/r1_oracle.py`.
The world `r1_oracle.wbt` was **not** changed, so the two readings below are a
controlled comparison with the navigator as the only variable. Named here
anyway per SPEC §8.4(4).

Amendments 2 and 3 recorded this arm's passing half as missing — first because
the engine dropped post-finalize motor targets, then because this arm's rover
could not turn its own wheels, then because the navigator was untuned. All
three are now fixed and the gate closes. Measured on the fixtures
`test_r1_discriminates_omnisim.py` uses (60 s window, `contact_steps 0`),
engine `msys64/mingw64/bin/omnisim-bin.exe` (sha256 `82d5964335feaeaf`),
machine `9722d23d12a3`:

| | Amendment 3 (2026-08-10) | Amendment 4 (2026-08-11) |
|---|---|---|
| outcome | **FAIL** (R1.4, R1.5) | **PASS 6/6** |
| distance to goal | 5.7968 m | **0.1506 m** (tol 0.30) |
| contacts | 2 (OBSTACLE_4, OBSTACLE_1) | **0** |
| path length | 23.5842 m | 14.2671 m (floor 10.769 m) |
| largest single step | — | 0.0084 m (bound 0.25) |
| max tilt | 3.142 rad — **the rover flipped** | 0.033 rad |
| occupied cells learned | 1303 (~70 % phantom) | 227 |
| arrival | never | t = 33.9 s, then latches PARKED |

Deterministic in both eras: bit-identical across 3 runs before the fix and 5
after, plus **2 further bit-identical replays made for this record**, run
through the same `_run` helper the discrimination test uses. The null and blind
halves are untouched and still FAIL: the discrimination file passed **11/11**
against the new navigator before this amendment touched it, and **12/12** after
the missing-gate assertion was inverted into a running one (18/18 together with
`test_r1_placement_omnisim.py`).

**The bound on the claim, which belongs in the record and not only in a commit
message.** R1 draws its layout **at grade time**, so an oracle that solves the
published layout has proved less than it looks. Against grade-time placed
layouts it had never seen, over **two independent five-layout probes**:

| probe | result | the failing layout |
|---|---|---|
| with the fixing change (`dc9bf156f`) | 4 PASS 6/6, 1 FAIL | reached the goal (0.213 m) but grazed a box — **R1.5 only** |
| independent, for this record (seeds `omnisim/gen/1`…`5`) | 4 PASS 6/6, 1 FAIL | **did not arrive**: 7.8065 m from goal, 20.8522 m of path, one `rover/OBSTACLE_1` contact — **R1.4 and R1.5** |

The 4-of-5 reproduces on independent seeds; **the failure mode is worse than
first reported**. It is not only "a graze near the goal" — on some drawn
layouts this navigator does not arrive at all. SPEC §7.1 asks that the task be shown **passable** on the
arm, which this establishes; it does **not** establish that every drawn layout
is solved, and ten layouts at `n = 1` is not a rate. Recorded on the cell as
"4 of 5, twice".

**Why the zero contacts are credible and not merely unmeasured.** The fixture
runs at `contact_steps 0`, and R1.5 refuses to credit an unmeasured channel —
that is exactly why the webots arm needs `contact_steps = -1` (5/6 at 0, 6/6 at
−1). On this arm a **run-long** contact watch supplies the channel at 0, and
the control that proves it is live is the **blind** cell on the identical
setting, which records a NAMED `rover/OBSTACLE_2` contact and fails for it.
A graded campaign cell runs at the meta's `-1`, a strictly wider watch. This is
recorded as the cell's `requires`.

`readiness.py` consequently moves from `discriminating NO` to `OK` for
`(R1_lidar_nav, omnisim)`, and R1 becomes the **only** task in the 9 × 3 grid
`readiness.py` will call publishable on any arm: `PUBLISHABLE for R1_lidar_nav
: omnisim, webots, mujoco`. Every other task reads `PUBLISHABLE … none`,
because SPEC §7.1's gate has not been run for it on any arm — which is the
correct reading and is left that way.

**⚠️ THIS AMENDMENT DOES NOT AUTHORISE PUBLISHING AN R1 NUMBER, AND THE TREE
STILL FORBIDS IT.** Two publication gates now disagree and the disagreement is
deliberate, not an oversight:

| gate | says |
|---|---|
| `readiness.py` (reads `sims.py` + this file's verdicts) | R1 is publishable on omnisim, webots and mujoco |
| `tasks/R1_lidar_nav/meta.json` → `status` (**frozen**) | *"NOT READY FOR A PUBLISHED NUMBER; R1 MAY BE RUN EXPLORATORILY BUT ITS NUMBERS MAY NOT BE PUBLISHED"* |

**The stricter gate wins, and it is enforced in code**, not merely written
down: `graders/test_r1_core.py` asserts `"may not be published" in status`, and
`adapters/mujoco/test_r1_discriminates_mujoco.py` asserts
`"MAY NOT BE PUBLISHED" in status.upper()` with a message that anticipates
exactly today — *"R1's numbers stay unpublishable while either arm still lacks
its oracle/null gate; if that has changed, re-read what this fixture is
claiming."* It has changed. All three arms now have the gate.

**What this amendment therefore does NOT do: lift that bar.** Two of that
`status` string's clauses are now measurably false — *"no oracle/null gate on
omnisim or webots for THIS task"* and *"on the OmniSim arm it is blocked by an
engine defect … no closed-loop navigator can be run there at all"* — and the
same false sentences appear in `anti_hardcode.what_is_still_unbuilt`. They are
**left standing**, for three reasons, and the disagreement is disclosed here
instead: the surrounding clause is a **publication verdict**, which is the
owner's call and not an editor's; two committed tests pin it, so changing it is
a behaviour change and not a docs fix; and Amendment 3 deferred the same field
to the owning R1 workstream for the same reason.

**Until the owner rules, the operative reading is: R1 may be RUN and its cells
recorded; no R1 number may be PUBLISHED.** `readiness.py`'s green means the
oracle/null gate is satisfied — it does not and cannot read the task's own
status string. When the bar is lifted, it takes an amendment, a re-freeze of
`tasks/R1_lidar_nav/meta.json`, and an update to those two tests, together.

---

#### Previously-undisclosed drift, recorded now under SPEC §8.4(4)

**The ceiling moved 900 s → 1800 s on 2026-08-09 and no amendment was ever
written.** Commit `35ceb134f` (2026-08-09 19:12) moved **all nine frozen
`meta.json` files** — the same nine this amendment re-freezes — and the change
was then silently absorbed into the next `test_freeze.py --write`
(`c9788f1f9`, 21:28 the same evening) without a numbered entry in this file.
Freeze v3's own §"Recorded drift" section, written that same day, is about the
2026-08-01→08 worlds; this one came after it and slipped through the same gap
the section was written to close.

Its measurable consequence is that **frozen content contradicted itself for two
days**: the nine `meta.json` files said `hard_ceiling_s: 1800` from
2026-08-09 19:12 onward, while the frozen `SPEC.md` §2.4 still read
`timeout_s = min(3 × par_s, 900 s)` and claimed `cells × 900 s` as the
campaign's worst case — right up until `6d5c6a79f` rewrote it today. Any reader
taking the SPEC at its word during that window would have understated the
worst-case bill by half.

One of the fourteen `agenticsimbench/v0.3` rows sits on the far side of that
line, and it matters: `r1_fair_omnisim` was scored at `timeout_s 1800`, while
the other thirteen were scored at 900 s or 720 s. **Those are two different
ceilings, and with today's 2700 s that is three experiments**, all filed under
one suite id with nothing in the rows to say so. Nothing is published from
them, which is the only reason this is a disclosure and not a retraction.

The reconciliation this amendment makes: the ceiling's full history is now
frozen text (SPEC §2.4.1 — 900 s at freeze v3, 1800 s on 2026-08-09, 2700 s on
2026-08-10, each with its reason), and it is stated there as a property of the
**experiment** rather than of a task, because `TASK_HARD_CEILING_S` is global
and rewrites every task's budget at once.

---

#### The non-comparability consequence

**Cells scored under different ceilings may never be pooled, averaged, or
placed in one column.** This is not visible from `timeout_s` alone for four of
the six core tasks — B1 (900 s), B2/B3 (720 s) and C1 (1440 s) sit below every
ceiling the suite has used, so their rows look identical across all three eras.
Only **A1** (1800 → 2160 s) and **C2** (900 → 1800 s) moved.

The fence is therefore the new machine-readable **`protocol` block** on each
row (samples, `runs_per_cell`, `variance_measured: false`, `is_rate: false`,
the ceiling in force, the cell wall bound, and the per-attempt session
budgets). **Its ABSENCE marks all 87 pre-existing published rows** — 73
`agentbench/v0` and 14 `agenticsimbench/v0.3` — as predating the single-run
protocol. Verified for this record: zero of the 87 carry one.

---

#### Audit trail

`preregister/oracle_verdicts.json` sha256 (the freeze's own rule — LF-normalised,
volatile fields stripped)
`a45f2b8ac6e9e728005afe87e597618cc93c11b8e47b007144464eb257d56b98` →
`e2f72c364e508b70bc1d1dcaabd83ec378a844f4a6f138df917f9f0545379867`;
`SPEC.md` `bb2d28196230f6d0effb465de809e34b013db568a060116ddd7629342082701d` →
`8b1d53f9a5972ba018c44fc672b95643d133d930d5677bba51858979df2840f1`;
`sims.py` `387e9630d34763cad1ec2a4a4d609816463d0c45d44dabcd7b1d52a28803d686` →
`81555adc011ba466cc06b28cb6a6a3692acc01dac6d0e1b1550b54e0d28ce480`; the nine
`tasks/*/meta.json` moved with them. Manifest regenerated with
`test_freeze.py --write`, and the pre/post manifests diffed entry by entry:
**exactly 12 of 91 file hashes moved and no others**, with `meta`, `plan`
(including `sha256_section_2`) and `lane_assignment` byte-identical — so the
plan's §2 is untouched and no grader, prompt, task world, oracle script,
tool-set manifest or comparator entry changed.

Pre-amendment manifest sha256 (LF-normalised):
`baabc1866a4f7fd256c0844ece70b912462ba66201ba83db816b71527d676a80` →
post-amendment `19a4d33f846e7e4438aa9ac5500299f6ef2460825f20bbf66b48ce5701ea6f5e`.

---

### Freeze v3 — Amendment 5 (2026-08-12): R4 joins the frozen set, and the scored-run check had to be strengthened before it could answer

**Scored runs at amendment time: ZERO for R4 — and saying so honestly required
STRENGTHENING the check, because the marker Amendment 4 leaned on no longer
separates what it separated.** Amendment 4's mechanical test was: a row scored
under the current protocol carries a machine-readable `protocol` block, and not
one row in the tree carried one. That was true on 2026-08-11. It is **false
today** — the lane now emits the block, and **three R4 rows carry it**.
Re-running that test unchanged would have returned a green that means nothing,
so it is restated here in the form that still discriminates, and its old form is
retired as a *sole* criterion.

**Verified four ways, all agreeing on zero:**

1. **No R4 row exists in a tracked path — none, anywhere.** Under the tree's own
   "no row, no result" rule (`results_published/README.md`) a row counts only in
   a tracked path. `results_published/` contains **zero** files mentioning
   `R4_mobile_manipulation`, tracked or untracked; the tracked corpus is the
   same 27 `rows.jsonl` directories Amendment 4 audited and not one of them is
   R4. There is therefore no R4 result to supersede, preserve or quarantine. (A
   28th directory, `r1_3arm_20260810_R1_lidar_nav_webots/`, sits on disk
   **untracked**; it is R1, not R4, and is named only so its presence is not
   misread as drift.)
2. **Three R4 rows DO carry a `protocol` block — and all three live in the
   gitignored `results/` scratch tree**, which is not a tracked path and never
   was: `results/cc_lane/r4_webots_20260811` (2026-08-11T17:55:40Z, webots,
   **FAIL 3/9** — the `first_cell` already recorded in R4's own `meta.json`),
   `results/cc_lane/20260812_round1_r4_omnisim_c1` (2026-08-12T08:39:05Z,
   omnisim, **FAIL 1/9**) and `..._c3` (09:27:04Z, omnisim, **FAIL 5/9**). All
   three carry `protocol.id "single-run-under-ceiling/2026-08-10"`. **This is
   exactly why Amendment 4's test is retired as a sole criterion:** the block
   marks *"produced by the current instrument"*, which is not the same property
   as *"scored"*. From here the two are told apart by where the row lives and by
   whether the freeze was green when the row was produced — both mechanical,
   neither dependent on the block.
3. **Every R4 cell ever run was barred from being a score at the moment it was
   produced, by the suite's own release gate.** SPEC §8.4 rule 1: *a red
   `test_freeze.py` blocks scoring a cell, publishing a number, or quoting a
   grid — in that state the suite has no valid pre-registration and anything it
   produces is `INVALID` by construction.* The freeze test has been RED **for R4
   specifically** from the commit that created the task (`1f2c0241a`,
   2026-08-11) until this amendment. This is not a technicality noticed after
   the fact: `1f2c0241a`'s own commit message says the freeze is red because of
   it, and R4's `meta.json` `freeze` block names in advance the eight paths that
   had to be frozen and states the consequence — *"Any R4 row produced before
   the manifest is regenerated is EXPLORATORY and its number may not be
   published."*
4. **The task says it in its own words.** `status`: *"NOT READY FOR A PUBLISHED
   NUMBER"*. `first_cell.publishable`: the literal `false`, with its three
   reasons.

---

#### ⚠️ The `20260812_round1` rows are VOID — not merely exploratory — and may not be pooled with anything

Two of the three protocol-carrying R4 rows above come from the 2026-08-12
diagnostic round, and they are **void on top of** being unpublishable: the
instrument that produced them was destroying its own neighbours and grading
against files no agent wrote. The defects were measured, then fixed the same day
in `286117d4e` (cc_lane isolation), `57f0abb16` (five harness endpoints that
lied to the agent driving them) and `2010888cc` (a capture screenshot path that
503'd on a healthy service). The three that bear directly on these rows:

- **Cells killed each other.** The port sweep terminated any listener on
  6789–6792 and the pre-session repo sweep took every untracked `.wbt` under the
  real `projects/` tree. Measured: **`r4_omnisim_c1`'s own post-session sweep**
  terminated a live A1 cell's harness, both engines, ten controllers and three
  supervisors — its row's `port_hygiene.post_session` still records the kill of
  `pid 15900 python.exe … omnisim_harness.py --port 6789` and its
  `omnisim-bin.exe` children. Separately, `a1_omnisim_c1` matched processes by
  name, killed another lane's engine twice, and then reported *"remaining
  omnisim-bin: 0"*.
- **A cell was graded on a world its agent never wrote.** Artifact discovery was
  newest-first and skipped `.harness_*` siblings but not `.capture_*`, so
  `a1_omnisim_c1`'s deliverable resolved to
  `.capture_newton_husky_swarm_drive.wbt` — the shipped 8-Husky demo plus a
  capture supervisor — as its own `cell_report.json` still records. It failed
  nearly every assertion for an agent whose ten robots had driven 5.14–30.27 m
  and passed `--fail-on-runaway`. The `.capture_*` sibling existed only because
  `/world/screenshot` was returning `200 image/png` with **zero bytes**, pushing
  the agent onto the capture service as a workaround.
- **A killed cell was misread as rate-limited.** Windows reports a killed
  process as `rc=4294967295`, which contains `429`; `a1_omnisim_c2` deferred,
  tore down the workspace it had just preserved, and slept 900 s holding the
  task lock — **no artifact, no grade, no row**. `r4_omnisim_c2` likewise has no
  row.

Both surviving R4 rows carry `measured_under_concurrency: true`, both record
`budget_exhausted: true` at the full 2700 s, and `c3` additionally waited
2839.9 s on the same-task lock. **They are quotable nowhere — not as an outcome,
not as an anecdote, and above all not alongside any post-fix row.** The
instrument differs, which is the same non-comparability fence Amendment 4 drew
across the ceiling changes. They are preserved rather than deleted, in the
gitignored tree where they already sit, as the evidence for the three fixes.

---

#### What is frozen by this amendment: the R4 task, its graders, and one drifted registry file

**Frozen entries changed: nine — eight ADDED, one MOVED.** Verified by running
`test_freeze.py` before writing anything (the drift list is the measurement, not
the changelog: `4 failed / 36 passed / 271 subtests passed`), and the manifest
diffed entry by entry after `--write`: **91 → 99 entries, 8 added, 0 removed, 1
changed**, with `meta`, `plan` (including `sha256_section_2`) and
`lane_assignment` byte-identical. No other task, grader, prompt, world, oracle
script, tool-set manifest, comparator entry, budget or threshold moved.

| path | how it drifted | commit |
|---|---|---|
| `graders/r4.py` | **added** — uncovered by the manifest | `1f2c0241a` |
| `graders/r4_core.py` | **added** — uncovered | `1f2c0241a` |
| `tasks/R4_mobile_manipulation/meta.json` | **added** — uncovered | `1f2c0241a`, then `56c1e25fa`, `e52a1af42` |
| `tasks/R4_mobile_manipulation/prompt.txt` | **added** — uncovered | `1f2c0241a` |
| `…/initial/benchmark_assets/{obstacles,scene}.json` | **added** — uncovered | `1f2c0241a` |
| `…/initial_webots/benchmark_assets/{obstacles,scene}.json` | **added** — uncovered | `1f2c0241a` |
| `agents/__init__.py` | **hash moved** — the only pre-existing frozen file to drift | `ddf533fb3` |

The first eight are `test_freeze.py`'s *no-silent-omissions* guard firing
correctly: `frozen_files()` discovers task dirs and `graders/*.py` from the
tree, so a task that exists and is not in the manifest is a red test by
construction. Nothing already frozen was edited to accommodate them.

**`agents/__init__.py` is the one that genuinely drifted, and it gained TWO
registry entries, not one.** The commit message for `ddf533fb3` names only the
first; both are recorded here because the manifest hashes the file, not the
message.

1. `("R4_mobile_manipulation", "external")`, `expect_pass: None` — the
   unknown-outcome external registration, identical in shape to freeze v2's
   Amendments 1 and 2 (C2, then B1/B2/B3/C1) and to R1/R2/R3's. It declares no
   expectation and validates nothing; it only lets the real grading pipeline
   score an artifact produced outside the runner. **Its absence was a measured
   defect, not a tidiness issue:** the first R4/omnisim cell spent its whole
   2700 s budget, delivered a world and a controller, and died at grading with
   `skip: no 'external' agent for R4_mobile_manipulation`. It bit the OmniSim
   arm and not the upstream one because only the OmniSim path grades through
   `run_agentbench` — `cc_lane`'s webots arm calls the grader in process and
   never consults this registry — so R4 read as expressible on both arms while
   being expressible on one. This is verbatim the recurrence the file's own
   comment already recorded for R1.
2. `("R4_mobile_manipulation", "null")`, `expect_pass: False` — appended to the
   existing R1/R2/R3 null loop. **This one does declare an expectation** (a null
   agent must FAIL), which is a stronger statement than the `expect_pass: None`
   entries the earlier amendments justified, so it is called out rather than
   folded in with them. It changes no threshold and adds no fixture: it is the
   registry side of SPEC §7.1's rule that *no task is passable by doing nothing,
   and a task with no null entry is a task where nobody ever checked*. **No R4
   null cell has been run on the OmniSim arm** — that arm has no R4 fixture at
   all (see the gate table below), so the entry makes the check *possible*,
   which is precisely what it was not before.

---

#### The task this freezes, and exactly how far its gate reaches

R4 is mobile manipulation end to end: drive a blocked route to a table, pick a
payload, carry it several metres, put it down on a pad and leave it there. It is
the only task in the suite that loads a contact model **while the body holding
the payload is accelerating** — a grip that holds a static pinch and lets go the
moment the base turns fails here and passes R3. The agent authors the world, the
robot, the gripper and the controller from an empty project plus two frozen
assets. Nine assertions, all judged in physical units:

| id | what it asserts |
|---|---|
| R4.1 | the run is clean |
| R4.2 | the specified scene is present and intact |
| R4.3 | there is one articulated mobile manipulator |
| R4.4 | the payload started at rest on the table |
| R4.5 | the base drove the route without striking anything |
| R4.6 | the payload was lifted clear and carried |
| R4.7 | the payload was held continuously, never re-acquired |
| R4.8 | the payload was delivered onto the pad and stayed |
| R4.9 | nothing was teleported and nothing was attached |

**Gate status at freeze time (SPEC §7.1, per (task, arm)) — one arm green, two
arms with no fixture at all:**

| arm | oracle | null | blind (extra control) | fixture |
|---|---|---|---|---|
| **webots** (upstream R2025a, WSL2) | **PASS 9/9**, reproduced twice, every number identical to 4 dp | **FAIL 4/9** (R4.3, R4.5–R4.8 fail) | **FAIL 5/9** (R4.5–R4.8 fail) | `adapters/webots/test_r4_discriminates_webots.py` |
| **omnisim** | — | — | — | **none exists** |
| **mujoco** | — | — | — | **none exists** |

So the task is **demonstrably completable** — a mobile manipulator authored from
upstream base nodes only drove 25.07 m with zero obstacle or wall contacts,
lifted the payload off the table, held it within 0.031 m of a gripper finger
link through a 6.85 m transport, and left it at rest on the pad at
(−2.9997, 3.5034, 0.125). The blind control (the same robot, planner, arm and
speeds with the LiDAR read deleted) strikes `OBSTACLE_1` and **R4.5 names what
it hit**, which is the measurement proving that clause can go red — R1 shipped a
collision clause that structurally could not, and every R1 number from that week
was uninformative.

**⚠️ THE OMNISIM ARM IS UNGATED FOR R4, AND NOTHING ABOUT AN OMNISIM R4 ROW SAYS
THE TASK IS PASSABLE THERE.** There is no oracle, no null and no fixture on this
arm — `adapters/` carries R4 controllers and a world for `webots` only. The
webots gate transfers nothing: SPEC §7.1 is per (task, arm) precisely because a
task can be expressible on one simulator and not another, which is the property
the whole comparison exists to measure. An OmniSim R4 cell may therefore be
*run* and *recorded*; its FAIL is a statement about that agent's session and
**not** evidence that OmniSim can or cannot express the task, and no
cross-simulator R4 comparison exists or may be quoted. The same holds for
MuJoCo, where the task is not even expressible (`readiness.py`: *fixture missing
for this arm*).

Two further bounds carried into the freeze rather than left implicit:

- **Grade-time placement is implemented but NOT INJECTED.**
  `r4_core.sample_layout` draws a legal obstacle layout at grade time and
  `graders/r4.py` resolves a declared one, but no arm places the drawn layout
  into the agent's authored world. A cell run today is scored against the
  **published** layout and the verdict says so (`layout_source "published (NO
  grade-time placement)"`; both round-1 rows carry `layout_seed: null` and that
  note). On such a row R4.5's collision-freedom is **not by itself** evidence of
  perception. R4's discriminating power there rests on R4.6–R4.8, which no
  amount of layout memorisation helps with.
- **The known hole is declared, not implied to be covered.** A runtime weld
  between gripper and payload is not distinguishable from a friction grasp in
  the neutral evidence. R4.9 catches the structural cheats it can see; the
  prompt forbids the rest. Closing it needs a per-step constraint inventory no
  arm's recorder produces.

---

#### Disclosed and deliberately NOT changed

**1. `meta.json`'s `status` still says "NOT READY FOR A PUBLISHED NUMBER", and
this amendment does not lift it.** One clause of that string is now stale —
*"test_freeze.py is red because of it"* ceases to be true the moment this
amendment lands. The surrounding sentence is a **publication verdict**, which is
the owner's call and not an editor's, and it is pinned in code:
`graders/test_r4_core.py` and `adapters/webots/test_r4_discriminates_webots.py`
both read that field, so changing it is a behaviour change and not a docs fix.
It is also **substantively right for reasons this amendment cannot touch**: the
gate exists on one arm only, no injection step is wired, and one observation is
an outcome and never a rate. Amendments 3 and 4 deferred R1's identical field
for the identical reason. **Until the owner rules, the operative reading is: R4
may be RUN and its cells recorded; no R4 number may be PUBLISHED.** When the bar
is lifted it takes an amendment, a re-freeze of
`tasks/R4_mobile_manipulation/meta.json`, and an update to those tests,
together.

**2. R4's gate is recorded in `meta.json`, NOT in
`preregister/oracle_verdicts.json` — and `readiness.py` consequently
under-reports the webots arm.** `readiness.py` reads the verdicts file, which
has no R4 cells, so it prints *"discriminating: NO — no oracle/null gate on
record for (R4_mobile_manipulation, webots)"* for an arm whose gate is green and
reproduced twice. This is the **opposite-direction twin** of the disagreement
Amendment 4 recorded for R1, where `readiness.py` read green against a
`meta.json` that said no. Both are the same underlying fault: two gates reading
two different files. It is left as-is here on purpose — folding R4's cells into
`oracle_verdicts.json` means editing a second frozen file's *content*, which
requires re-running the gate through `preregister/run_oracles.py` on the
upstream-Webots arm rather than transcribing `meta.json`, and this amendment's
diff is deliberately confined to the drift it was called to clear. R4's own
`meta.json` already nominates it (`freeze.also_needs_an_amendment`). Until then
the arm-level truth is: **gate green on webots per `meta.json` and the committed
discrimination test; absent everywhere else; `readiness.py` will say NO for
every arm.**

**3. SPEC §8.4's opening sentence says the pre-registration "hashes 68 files".**
It hashes 99 after this amendment (91 before it), and it has not read 68 since
before freeze v3. `SPEC.md` is itself frozen, so the number is left standing and
disclosed here rather than corrected in a change that would then have to
re-freeze `SPEC.md` too. The count that binds is the manifest's, not the prose's.

**4. Unhashed files that moved with the same commits, named per SPEC §8.4(4)
even though we believe they changed nothing frozen.** `agents/external.py` and
`cc_lane/evidence.py` (artifact conventions and evidence collection for R4),
`graders/test_r4_core.py`, the webots-arm fixture set
(`adapters/webots/test_r4_discriminates_webots.py`,
`webots_lane/worlds/r4_mobile_manipulation.wbt`, and the `r4_oracle` /
`r4_null` / `r4_blind` controllers), and — from `ddf533fb3` —
`adapters/omnisim/evidence.py` with `adapters/omnisim/test_recorder_tracks.py`.
That last pair is the one worth stating plainly, because it changed a
**measurement** on the OmniSim arm: R4.5's falsifier reads
`ContactObservation.total_observed` to ask *"could this channel have reported a
collision at all"*, and the adapter filled those counters from the phase-A
window, which R4's `contact_steps: -1` clamps to a single pre-step sample that
observes nothing. Measured on the first R4/omnisim run, the witness read **0**
while the run-long watch held **25,209** contacts and had NAMED the robot's
collision with `OBSTACLE_2` — so R4.5 could not go green whatever a robot did,
the mirror image of R1's defect where a collision clause could not go red. The
counters are now filled from the run-long document when, and only when, phase
A's window was zero-width. **No grader and no threshold was touched, and the
repair could not flatter the run that exposed it:** re-graded either side of the
change every assertion is identical (FAIL 5/9, first failing R4.5) and the only
difference is the witness reading 25209 instead of 0.

---

#### Audit trail

`agents/__init__.py` sha256 (LF-normalised)
`af6ef99926964f2db86d38143138cb9406dd16acb074f65bf3d6591b3d7381b0` →
`71872168120b08fd7a1c92b3d5fecb5c91a1320a1ed4f3e45558b4e7f23016bf`.

Eight entries added, at these hashes:

| path | sha256 (LF-normalised) |
|---|---|
| `graders/r4.py` | `8a2ea74557a9516de1e16049d26e804eb6c0f6d09b52db4361f9e0882cc16c0c` |
| `graders/r4_core.py` | `bc461923ee5a972a3fd37ea7ad282d4e745fba99a0e05030ee48cafa2ba1e2e2` |
| `tasks/R4_mobile_manipulation/meta.json` | `c049e7ce94bda1fecca1035bada9a70f68712dedd984b8522880972f24c1debf` |
| `tasks/R4_mobile_manipulation/prompt.txt` | `5a0dcc0f59bee149dec4437678056eb93490d9e41965f1683dd3a0e173e856fe` |
| `…/initial/benchmark_assets/obstacles.json` | `dd2eb7572651797f3a89dbeace4d6784521382c103772a562e7b0b114deebcf9` |
| `…/initial/benchmark_assets/scene.json` | `bbcdfb73cc1276ac4b2f982ec77e37a7cdf9a138811da714a652ce636c18cab5` |
| `…/initial_webots/benchmark_assets/obstacles.json` | `dd2eb7572651797f3a89dbeace4d6784521382c103772a562e7b0b114deebcf9` |
| `…/initial_webots/benchmark_assets/scene.json` | `bbcdfb73cc1276ac4b2f982ec77e37a7cdf9a138811da714a652ce636c18cab5` |

(The `initial/` and `initial_webots/` asset pairs are byte-identical by design —
both arms author from the same frozen scene and obstacle contract.)

Manifest regenerated with `test_freeze.py --write` from a tree whose ONLY red
was the drift above, and the pre/post manifests diffed entry by entry: **91 → 99
entries, 8 added, 0 removed, exactly 1 hash moved**, with `meta`, `plan`
(including `sha256_section_2`) and `lane_assignment` byte-identical — so the
plan's §2 is untouched and no grader, prompt, task world, oracle script,
tool-set manifest, comparator entry or budget outside R4 changed.

Pre-amendment manifest sha256 (LF-normalised):
`19a4d33f846e7e4438aa9ac5500299f6ef2460825f20bbf66b48ce5701ea6f5e` →
post-amendment `cac5a5bb14db8e3039ef52920465b31c3adf0114753b8cbf93e657028228d7fc`.
(The pre-amendment value is Amendment 4's post-amendment value unchanged, which
is itself the check that nothing drifted between the two amendments.)

---

### Freeze v3 — Amendment 6 (2026-08-13): the default world-iteration tool changed, so its manifest and its result boundary move together

**Scored runs under the changed tool surface at amendment time: ZERO.** Commit
`7a0ce8c476e25b3b8348f7cc9b4798127acfdd66` changed the shipped OmniSim MCP
`load_world` contract on 2026-08-12 at 14:25:50Z, making safe world sync the
default iteration path. A complete search of every `rows.jsonl` below
`tests/benchmarks/agentbench/` found no row whose filesystem timestamp is later
than that commit, and `git log` found no result or published-result commit after
it. The changed surface therefore has no result to preserve, supersede, or
pool. All earlier `shell_plus_tools` rows belong to the old manifest and remain
historical evidence under that exact tool contract.

**Exactly one of the 99 frozen file hashes moved.** The generated
`runner/manifests/shell_plus_tools.json` now matches the shipped tool registry:

- `load_world` documents its load-once, live-pose-or-reload sync behaviour;
- its schema adds `light`, `settle_steps`, `reset_physics`, and
  `force_reload`;
- the embedded `tools_sha256` moves from
  `79df5f321040c0c1db98706a0ff7552906ab896500399b7f151196eabd3abed4` to
  `fd086dbe4b349f126d4bffd7dd2183b37db1d636be5956dfb0808b1ab47f795c`;
- the embedded `manifest_sha256` moves from
  `b0ba49e60404d89729e9daa377c35b65d7e01c554f5671c975517ab55e1286f8` to
  `67f0d7933154c9c5fcc8544aeb9d6e68b7fe1d8d98f76619d33f6192efbc1635`.

This is a material tool improvement, not a grader, prompt, task, threshold,
budget, model, comparator, or physics change. It may improve an agent's edit
loop, which is precisely why the old and new `shell_plus_tools` cells may not
be pooled. Future reporting must carry the row's tool-set manifest hash; a
table mixing the two hashes is a protocol violation even though both rows say
`condition: shell_plus_tools`.

The stale manifest was regenerated from the shipped registry before the freeze
was rewritten. The pre/post freeze-manifest diff is **99 → 99 entries, 0 added,
0 removed, exactly 1 hash moved**. `meta`, `plan` (including
`sha256_section_2`), `lane_assignment`, every task, every grader, every oracle,
and both other tool manifests are byte-identical.

#### Audit trail

`runner/manifests/shell_plus_tools.json` sha256 (LF-normalised)
`bed200f777e8441de42d6185a19fdf2428cbffc5a77deef501075d5b78a4b67b` →
`411cdf5b341442cdd41efb31ddd06825368886b7810a4753e4312735256207bd`.

Pre-amendment freeze-manifest sha256 (LF-normalised):
`cac5a5bb14db8e3039ef52920465b31c3adf0114753b8cbf93e657028228d7fc` →
post-amendment `4269bb10d5b1d2e5d62c9f3905601f525c4b5562fe1d6445b7775776cd814bb5`.
