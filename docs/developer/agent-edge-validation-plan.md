# Do agents actually get more done here? — the validation programme

**Status:** experimental plan, **revision 3 — 2026-08-01** (revision 2: 2026-07-31;
revision 1: 2026-07-26). Nothing in this file is a result.

**Why it exists:** on 2026-07-26 we ran the first A/B of our own agent surface against a bare
POSIX shell, and **the bare shell won or tied every run.** The claim that an LLM agent gets
more done on OmniSim than elsewhere is therefore **currently unsupported by any measurement
we hold**. This document is the programme that either supports it properly or retires it in
public, and it is written before the runs so the retreat cannot be improvised afterwards.

**What changed in revision 3, and why.** One decision, made by the owner before any scored
run: **the headline Phase W campaign is a product-level comparison driven by Claude Code**
(§2.7) — the product as shipped, per simulator, one condition named `claude_code` — replacing
the API-runner as the headline instrument. The API-runner lane — everything revision 2
designed and froze about conditions, the Webots bridge and the oracle guards — is **retained,
unchanged in substance, as the mechanism-isolation follow-up** (§2.7), to run when an API
credential exists; **F-surface (conjunct i) moves explicitly to that lane**, because the
product lane has no shell/tools split to evaluate it on (§2.4 scope note). Zero scored runs
have happened, so this is a legal pre-run amendment under the same clause that legalised
revision 2 — executed as an honest version bump, never a silent edit: the pre-registration
freeze is re-executed as **freeze v2** (superseding v1, with v1's manifest hash recorded in
[`preregister/FREEZE.md`](../../tests/benchmarks/agentbench/preregister/FREEZE.md) for the
audit trail). The trade, in one line each: it **buys ecological validity** — the campaign
measures the product an agent actually meets (repo, docs, defaults, error messages, and a
real shipped coding agent), which is the sentence our positioning actually utters; it
**costs instrument freezability** — Claude Code is a moving product we cannot byte-hash; the
version is pinned, replication is scoped "at this version", and the weakening is recorded as
threat §5.11.

**What changed in revision 2, and why.** Revision 1 was written the evening of the null result
and encoded three design errors plus a status table that the following week's work overtook:

1. **E1 is restated as a cost-to-outcome claim.** Completion is now a gate, not the headline.
   §0's author tier showed why: a strong model saturates completion in both conditions, so a
   completion-scored design measures the model, not the surface (§5.4). The product claim was
   always *"the same job, cheaper, in fewer round trips"* — revision 1's F could not express it.
2. **The task set is split into three lanes** (§2.1): a control lane where a tie is the
   *expected* result, a decision lane of closed-loop tasks where F is evaluated, and a
   capability-frontier lane reported as a capability table and never aggregated into a
   throughput score. Revision 1's portable-only decision set structurally selected against the
   surface under test; the opposite error — grading a competitor FAIL on tasks defined by our
   own API — is what the lane C reporting rule exists to prevent.
3. **F is re-specified per conjunct, at the SPEC's own repeat floors (n = 5; n = 10 for A1),
   with the Webots `shell+tools` condition defined** (§2.2, §2.4). Revision 1 cited
   "n ≥ 3 (SPEC §3.5)" — the SPEC contains no such floor; it requires **n = 5** per cell and
   **n = 10 for A1**. And revision 1 evaluated F over 12 (task, simulator) pairs while leaving
   the Webots `shell+tools` condition undefined — upstream ships no packaged tool surface — so
   up to half the pairs were structurally unable to show the effect and the rule was close to
   a guaranteed withdrawal regardless of the truth. A rule that can only lose is as worthless
   as one that can only win.
4. **The ground truth is refreshed** (§1.1): between 2026-07-26 and 07-27 most of Phase R was
   *built* — the standalone runner with enforced budgets and token accounting (`084e69ce`,
   `1fb331a7`), condition-in-row (`2a234a7f`), the Webots grading adapter (`3c995c9c`), the
   committed Phase-0 verdicts (`1c93d6f1`). None of it has ever executed a real model call.
5. **Citation repairs**, so the file stops mis-quoting its own evidence base: the SPEC's
   repeat rule is n = 5 (n = 10 for A1), not n ≥ 3; "fairness floor" is SPEC §6.2.2, not
   §4.1; "we are the author and a contestant" is SPEC §8.2 (task-selection bias is §8.3); the
   four-negative packaging sentence ("no ROS, no DDS, no in-process Python, no editor
   plugin") lives in [`simulator-comparison.md`](simulator-comparison.md) §8, not
   `agent-native-api.md` §1.3 (§1.3 carries the narrower "packaging, not capability" claim
   *and* the caveat that the advantage evaporates when the agent's environment already has
   ROS); and simulator-comparison §9.2 now holds **eight** prior corrections, not six.
6. **Revision 2's own new degrees of freedom are named and guarded** — added the same day,
   after an adversarial design review of this revision's first draft. The redesign introduced
   four author-controlled levers revision 1 did not have: lane assignment, best-condition
   selection, authorship of the competitor's tool bridge, and the Lane C table. Each now
   carries a mechanical guard: an oracle-based lane test countersigned by the external
   reviewer (§2.1); a mechanical best-condition rule with no default toward our own condition
   (§2.4); completeness / granularity / distinctness checks on the bridge, judged pre-freeze
   (§2.2); and a numerics quarantine on Lane C (§2.1, §4). The withdrawal rule's burden was
   also inverted — the first draft withdrew only when *both* evidence channels failed, which
   under a no-effect null kept the claim alive roughly half the time; survival now requires
   both channels to hold, and single-channel outcomes publish as narrow findings with the
   adverse channel's table printed (§2.4).

**Relationship to the other files.** [`tests/benchmarks/agentbench/SPEC.md`](../../tests/benchmarks/agentbench/SPEC.md)
is the **contract** — tasks, prompts, graders, budgets, outcomes, the two-condition design,
the competitor-fairness plan. This file is the **validation programme that consumes it**:
which experiments, in which order, at what cost, and — the part the SPEC does not contain —
**the pre-registered condition under which we withdraw the claim.** The SPEC has falsification
*language* (§0, §6.1, §8.2.2) but no withdrawal *procedure* — no trigger, no retraction list,
no scripted statement — and it does not reference this file; this file is that procedure.
`agent-native-api.md` (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) is the capability audit the claim rests on
(6 ahead / 5 partial / 10 missing vs ROS 2 `simulation_interfaces`).
[`tool-design-for-agents.md`](tool-design-for-agents.md) is the principle this programme tests
at task level — its own §5 names `omnilink-bench` as the next task-level measurement; that
lane and this one are complementary (theirs within-OmniSim, ours cross-simulator), and
neither number exists yet. [`webots-control-baseline.md`](webots-control-baseline.md) is the
Phase W bring-up recipe. [`simulator-comparison.md`](simulator-comparison.md) §9.2 is where a
withdrawal gets recorded.

---

## Contents

- [0. The measurement that started this](#0-the-measurement-that-started-this)
- [1. The claim, stated so it can fail](#1-the-claim-stated-so-it-can-fail)
- [2. The pre-registered falsification design](#2-the-pre-registered-falsification-design)
- [3. The phases](#3-the-phases)
- [4. What each phase licenses, and the forbidden sentences](#4-what-each-phase-licenses-and-the-forbidden-sentences)
- [5. Threats to validity, ours included](#5-threats-to-validity-ours-included)
- [6. Order of work](#6-order-of-work)
- [7. Open questions, recorded rather than resolved](#7-open-questions-recorded-rather-than-resolved)

---

## 0. The measurement that started this

Measured **2026-07-26** on machine **`9722d23d12a3`** (RTX 3060 Laptop, Windows 11 — `M1` in
OmniBench's machine table), commits **`cf74d7b7`** (the A/B) and **`03e988c5`** (four tooling
defects the A/B exposed). Two tiers, two tool conditions, **same model**, artifacts graded
blind by the validated Phase-0 grader.

| tier | task | condition | outcome | tool calls | wall clock | artifact |
|---|---|---|---|---|---|---|
| author | `A1_husky_swarm_10` | **`shell`** (shell + files, no harness) | **PASS 10/10** | **~36** | ~32 min | 5,217 B, R = 0.259 |
| author | `A1_husky_swarm_10` | `shell+tools` (full HTTP surface) | PASS 10/10 | ~50 | ~32 min | 10,509 B, R = 0.194 |
| debug | `C2_fall_through_floor` | **`shell`** | correct fix | **21** | **~8 min** | — |
| debug | `C2_fall_through_floor` | `shell+tools` | correct fix | ~21 | ~13 min | — |

*(`shell` / `shell+tools` are the SPEC §4.1 condition names; the commit message calls the same
two conditions `shell_only` / `full_surface`. This file uses the SPEC's. The "no harness"
description of the `shell` condition was this ad-hoc A/B's setup — the frozen condition
boundary, decided in §2.2, differs.)*

**⚠️ Provenance differs by tier.** The A1 rows are recorded in `cf74d7b7`'s commit body. The
C2 rows survive **only as an operator's note** — no commit, no row records them — and are kept
here as history under that label; per §0.2's standing rule they are quotable nowhere else.

**The bare condition was equal or better in every run.** On the author tier it produced the
same graded PASS with ~28% fewer tool calls; on the debug tier both conditions reached the
same correct fix (the debug figures carry the provenance warning above, and wall clock from
this A/B is invalid either way — item 4 below). The `shell+tools` agent, asked afterwards
what saved it time, credited **documentation facts** — `newtonSolver "mujoco"`, the njmax
rule, the lighting recipe — **not endpoints.**

### 0.1 Why these four runs do not settle anything, in either direction

Stated first, because it is the reason this document is a plan and not a conclusion.

1. **n = 1 per cell.** The SPEC (§3.5, §8.1.3) calls n = 1 a demo, marks it `exploratory`,
   and bars it from any published claim. That rule binds the null result too.
2. **The ablation leaked, and cannot be un-leaked this way.** Both agents were Claude Code
   subagents, and the harness auto-injects `CLAUDE.md` → `AGENTS.md` into every subagent's
   context. The `shell` condition therefore had our entire agent-facing manual — the
   non-obvious answers included. A documentation-free condition was **never achieved and is
   structurally unachievable with subagents** (V2 below — since built, see §1.1).
3. **Two of the four cells are the wrong tier for the hypothesis.** The author tier is where a
   capable model can simply write the file; the surface's hypothesised value is in
   inspect / iterate, which was not run at all.
4. **The wall-clock numbers are contaminated by concurrency.** The two author runs ran
   *simultaneously* on one box, sharing one engine binary, the CPU, and — as `cf74d7b7`
   records — one hardcoded global trace directory. Both A1 runs reporting "~32 min" is what a
   shared pacing constraint looks like, not what two independent measurements look like.
   **Do not reuse wall clock from that A/B — either tier.** The tool-call comparison
   survives; the clock does not.
5. **The agent's own explanation is narration, not evidence.** SPEC §8.1.2 forbids believing
   what an agent says over what was measured, and that rule does not suspend itself when the
   agent is agreeing with us. "Docs, not endpoints" is a **hypothesis** to be recomputed from
   the trace's tool-call histogram — not a finding.
6. **The comparison is OmniSim-vs-OmniSim.** It measures our surface against our own shell. It
   says nothing whatsoever about any other simulator, and the claim in question is comparative.

**What §0 taught, in one sentence:** *an authoring task graded on completion cannot
discriminate* — both conditions reduce to "write the file", and a strong model writes it
either way. Revision 2's lane structure (§2.1) and cost-first rule (§2.4) are that lesson
applied.

### 0.2 The numbers were not in a result row — partly fixed, and the residue is listed

Revision 1 recorded three instrumentation defects; the week's commits paid most of them down,
and this section now tracks the residue, because **a campaign cannot be more falsifiable than
its bookkeeping** (§5.6).

**Fixed in code since revision 1:**

- Rows record their real condition and sim (`2a234a7f`): `condition` comes from the agent's
  own artifacts, `tool_set` carries `tools_sha256` + `manifest_sha256`, the `agent` block
  carries model / scaffold sha / system-prompt sha, and `stop_reason` rides along.
- Token accounting exists end to end (`084e69ce`, `1fb331a7`): the runner's `Ledger` records
  turns, model calls, tool calls, `tokens_in/out/cache_read/cache_write`, per-turn usage; the
  row carries `metrics.{t_agent_s, t_total_s, turns, tool_calls, tokens_in, tokens_out,
  tokens_cache_read, usd}`.
- The duplicate-`artifacts`-key bug that silently dropped `external_label` (and everything
  else the agent attached) from every A/B row is fixed (`1fb331a7`; the surviving key is
  `agent_artifacts`).
- The four Phase-0 verdicts (oracle / null / wrong / parade) are **committed** under
  `tests/benchmarks/agentbench/phase0_validation/` (`1c93d6f1`), so the grader-discrimination
  claim is checkable from a clean clone instead of a commit message.

**Still open, and blocking (Phase R remainder, §3):**

- `CONDITION = "scripted"` is still the module-level **fallback** in
  [`run_agentbench.py`](../../tests/benchmarks/agentbench/run_agentbench.py), there is no
  `--condition` CLI flag, and **no test forbids a scored row from carrying
  `condition: "scripted"`.** 56 of the 58 rows in the local results tree still say `scripted`.
- **No row in this repository has ever been produced by a real model call.** The one row with
  a real condition and tool-set hash came from the scripted replay backend; `usd` is `null`
  on all 58 rows; the Anthropic backend's own `untested_until_credential` list still names
  the HTTP round trip, real token usage, and the refusal/stop-reason paths.
- `tokens_cache_write` is accounted by the Ledger but **dropped from the row**.
- **The row-writer collapses the two clocks:** `run_agentbench.py` writes `t_agent_s` and
  `t_total_s` as the same number — the measured agent clock reaches only the trace event.
  §2.3's clock-reporting resolution is unimplementable until Phase R item 2 separates them.
- `results/` is **entirely gitignored**, so even correctly-instrumented rows die on the
  machine that produced them. The Webots grading claimed in `3c995c9c`'s commit message
  (10 Pioneer 3-AT, FAIL 5/10) is a **new instance of the original sin**: it exists nowhere
  but the commit message and is not reproducible from the tree.

> **Standing rule, adopted here: no row, no result.** A number that exists only in a commit
> message, an operator's note, or an agent's summary may not be quoted in any document,
> `AGENTS.md` included. Campaign rows must live in a tracked path (mechanism decided in
> Phase R — a `results_published/` dir or equivalent), or the campaign did not happen.

---

## 1. The claim, stated so it can fail

> **E1 (revision 2).** Given one sentence and no human help, on tasks that require a
> **closed observe–act loop against a live scene**:
>
> **(i) — the surface conjunct.** An LLM agent reaches the same or more graded outcomes at
> **materially lower cost** — fewer tool calls, with tokens and agent-time reported beside —
> when it has OmniSim's agent-facing surface packaged as tool definitions than when it has
> only a POSIX shell on the same simulator.
>
> **(ii) — the comparative conjunct.** The same agent reaches **more** graded outcomes on
> OmniSim than on another simulator given each simulator's own best first-party surface,
> **at no cost penalty**. The partial versions — same outcomes at materially lower cost, or
> more outcomes at equal cost — are **narrow findings** (§2.4), not E1.

Both conjuncts must hold for E1. Completion acts as a **gate** (a condition that completes
less cannot win on cost — §2.4 enforces this per task); the headline is cost-to-outcome.
Under the frozen condition boundary (§2.2) the `shell` agent may bootstrap any installed
surface itself, so conjunct (i) is, precisely, a claim about **packaged tool definitions**:
whether handing the agent the surface *as tools* beats making it discover and drive the same
surface by hand. E1 is deliberately narrower than SPEC §0's H1: H1 is the whole benchmark
thesis, E1 is the single load-bearing sentence in our positioning that no measurement
currently supports. §0's A/B attacked conjunct (i) on the wrong lane (authoring) and is the
reason the claim is scoped to closed-loop tasks: on tasks with no loop, we now *expect* no
effect, and §2.1's control lane exists to keep us honest about that expectation.

### 1.1 The five validity requirements — status as of 2026-07-31

A run that violates any of these produces a number we are not allowed to quote. Statuses
distinguish **built** (code exists), **tested** (unit-tested), and **run** (has produced a
real row) — a distinction revision 1 did not need because nothing existed.

| | requirement | status 2026-07-31 |
|---|---|---|
| **V1** | **A real competitor in the comparison.** OmniSim-vs-OmniSim cannot support conjunct (ii), ever. | ❌ **no competitor row exists.** The Webots *grading* adapter is built and unit-tested against synthetic fixtures (`adapters/webots/`, `3c995c9c`); the launcher, the Webots-side Supervisor recorder, and the install itself are not built — they exist as prose in [`webots-control-baseline.md`](webots-control-baseline.md). One Webots grading is claimed in that commit's message and is not reproducible (§0.2). |
| **V2** | **An agent runner that is not a Claude Code subagent.** | ◐ **built and tested, never run.** [`runner/loop.py`](../../tests/benchmarks/agentbench/runner/loop.py) is a standalone, sim-agnostic loop with all four SPEC §2 budgets enforced, full token accounting, per-run sandboxing, and a **documentation-free system prompt asserted by test** (the composed prompt is tested to contain none of `AGENTS.md`/`CLAUDE.md`/`husky`/`newtonSolver`…). Note the design supersedes revision 1's wording: instead of a "hashed read-only docs corpus", docs are reachable only through the agent's own file-reading tools, and the docs-**ablation** cell becomes a read-root deny-list (`AGENTS.md` + `docs/developer/` removed) — still unbuilt. Residual gap: **no credentialed model call has ever been made** (§0.2). |
| **V3** | **Repeats with reported variance.** **n = 5** per (task, sim, condition, model), **n = 10 for A1** — the SPEC's floors (§3.5; n = 1 rows are `exploratory` and barred). *(Revision 1 cited an "n ≥ 3 floor (SPEC §3.5)" that the SPEC does not contain.)* | ❌ not met — every non-scripted cell on record is n = 1; `--repeats` exists, variance reporting does not. |
| **V4** | **A task set we did not author alone.** At least two tasks per tier from sources we do not control — this file's floor; SPEC §8.3's own commitments are two tasks solicited from each competitor community and the external column **leading the report if it disagrees with ours**. | ❌ not met — every task is ours (3 of the SPEC's 15 are implemented, all ours). |
| **V5** | **Competitor scaffolding a competitor's maintainer would accept** (SPEC §6.2): generated from *their* published definitions, published before the numbers, 30-day correction window, one non-OmniSim reviewer runs each competitor cell, a maintainer's better scaffold becomes the published headline. | ◐ **the rule is encoded, the artifact is not built.** `tools_sha256` makes the byte-identical-`shell` check a hash instead of a promise; `adapters/__init__.py::REQUIRED_EVIDENCE` publishes the cost of a new column. No competitor toolset exists — the Webots bridge (§2.2) is the first one required. |

**Zero of five were met on 2026-07-26; two are now partially met, and none fully.** The only
publishable sentence about E1 today is unchanged: *"we have not measured it, and our first
attempt pointed the other way."*

---

## 2. The pre-registered falsification design

This is the most important section in the file. It is written **before** the runs, and the
SPEC's pre-registration machinery (§6.2.1: freeze, publish the SHA-256 in `preregister/`,
timestamp it publicly) applies to this section verbatim. Revision 2 replaces revision 1's §2
**before any freeze occurred** — this redesign is legal exactly once; after the freeze,
changes void the campaign.

### 2.1 The three lanes

**Lane A — control (authoring): `A1_husky_swarm_10`, at n = 10 (SPEC §3.5's flagship
floor).** Both conditions reduce to writing a `.wbt` file, so the **expected** result is a
tie — and the lane stays in every campaign *because* of that expectation. It calibrates the
model against the task family, it proves the set was not curated down to tasks our surface is
good at, and if the surface somehow *loses* the control lane, that is published. A1 is
additionally the owner's demo task (SPEC §8.3) and is always reported separately. **Lane A is
never in the decision set.**

**Lane B — the decision set (closed loop): `B1_overlap_audit`, `B2_subject_in_frame`,
`B3_measure_and_report`, `C1_parse_error_fix`, `C2_fall_through_floor`.** Five tasks, each
requiring observe → act → re-observe against a live scene, each expressible on both
simulators through their native idiom — ours via harness verbs and hot reload; upstream via
writing Supervisor controllers, restarting, and reading stderr. Both paths are *legal*; the
question F asks is what each one *costs*. This is the only lane F reads.

**Lane membership is decided by an operational test, not by our rationale.** A task is
Lane B only if the scripted per-task oracle (the same instrument §5.5's red-evidence rule
requires) completes it on **every** simulator in the campaign, with the competitor oracle's
`tool_calls` at most **3×** the OmniSim oracle's; anything else is Lane C. The §6.2.4
non-OmniSim reviewer countersigns the lane assignment at freeze, and lane membership is
frozen content — moving a task across the boundary afterwards voids the campaign (§2.4).
`B2` deserves a named caveat: its inclusion rationale — `POST /scene/frame` is the most
differentiated verb we own (`agent-native-api.md` (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) §1.2), so if the
surface pays anywhere it pays there — is an **admitted seeding of the decision set with the
surface's best case**. The oracle test and the countersignature govern its membership, not
the rationale. And from Phase G onward, at least one externally-sourced (V4) task must sit
in Lane B, or every F verdict opens with the sentence *"evaluated on an owner-authored task
set."*

**Lane C — the capability frontier.** Tasks whose *loop shape itself* has no packaged
counterpart upstream: the five-edit iteration loop (the SPEC's OmniSim-only annex task `N3`,
re-expressed as a cross-sim outcome), snapshot/rollback experimentation, structured-diagnostics
triage of a deliberately broken world, live spawn/delete. Attempted on **both** simulators
through whatever idiom each has; graded by physical outcome; reported as a
**capability-and-cost table** in the style of OmniBench lane 3c
([DRIVEABILITY.md](../../tests/benchmarks/omnibench/lane3/DRIVEABILITY.md)) — per (task, sim):
the outcome, its cost, or an evidence-cited `NOT_EXPRESSIBLE` under SPEC §6.4's rules (written
justification citing the simulator's own docs; a single credible counter-example flips the
label). **All Lane C numerics live in that table and nowhere else:** the table is marked
`exploratory` (Lane C cells run at n = 3), appears only in the same artifact as — and below —
the Lane B F verdict, and no prose sentence may quote a competitor's Lane C cost or budget
exhaustion, nor any count or ratio over Lane C cells ("N of M capabilities"); competitor
cells are described in prose by capability label and idiom only (§4). **Lane C is never
aggregated into F and never into any throughput number.** The rule binds in both directions:
grading a competitor FAIL on a task defined by our API would be the rigging §2.6 exists to
prevent — but a decision set restricted to what every simulator can do comfortably, which is
what revision 1 had, structurally selects against the thing under test. A quarantined table
shows the frontier without contaminating the fair fight and without becoming a curated-rout
marketing channel.

**Grader debt, stated:** `B1`, `B2`, `C1` are still SPEC text — only `a1`, `b3`, `c2` exist
(each now split into a simulator-neutral physical core plus adapters, `4b61326d`). The three
missing graders are pre-registration work: written **from the physical specification before
any scaffolding** (SPEC §8.2.5), each born with the negative fixtures §5.5 requires. If a
Lane B task's grader cannot be written in physical units that hold on both simulators, the
task leaves the set and the set is re-frozen *before* any run — not after.

### 2.2 The conditions — including the one revision 1 left undefined

Both conditions run on both simulators. Lane B cells at **n = 5**; `A1` at **n = 10**
(SPEC §3.5 — the flagship is held to its own floor, not quietly reduced).

- **`shell`** (both sims): POSIX shell + `read_file`/`write_file`/`list_dir`, byte-identical
  across simulators — the fairness floor (SPEC §6.2.2) — enforced by `tools_sha256`.
- **The `shell` condition's boundary, decided and frozen:** the shell agent may use
  **anything installed in the image** — including starting OmniSim's harness and driving it
  with `curl`, and including a competitor's own CLIs (SPEC §4.1: the simulator is installed;
  the agent figures the rest out). The floor must mean the same thing on every simulator, and
  "installed but forbidden" is not honestly enforceable. Consequence, stated in §1:
  conjunct (i) measures the value of *packaged tool definitions*, not of the surface's
  existence. (§0's A/B described its `shell` condition as "no harness"; that was the ad-hoc
  subagent setup, not this frozen definition.)
- **OmniSim `shell+tools`**: the shell set plus the **18 tools** generated from the shipped
  `omnisim-mcp` registry — 22 tools in the full manifest with the shell set (SPEC §4.2's
  integrity property: a test asserts byte-equality with what ships).
- **Webots `shell+tools` — defined here, because nothing else defines it.** The SPEC's §6.1
  table assigns upstream "extern controllers on TCP:1234, `--stream`, the Supervisor API",
  but no packaged toolset exists and upstream publishes no service definitions to generate
  one from. The condition is therefore: the shell set **plus a tool bridge wrapping the
  entire published Supervisor and Robot function reference of upstream R2025a** —
  mechanically enumerated from upstream's own function index, explicitly including the verbs
  this section's first draft forgot (contact-point queries, which `B1` needs, and
  import-from-string spawning) — running as an extern Supervisor controller, with tool names
  and descriptions taken from upstream's documented reference, not paraphrased. **Any
  exclusion is listed and justified in the pre-registration and countersigned by the §6.2.4
  reviewer** — a bridge faithful in every included verb but curated by omission is the abuse
  fidelity rules alone cannot catch. We author the packaging; therefore it is published with
  the scaffolding **before any number** (SPEC §6.2.3), carries the 30-day correction window,
  a non-OmniSim reviewer runs those cells (§6.2.4), and **a maintainer's better bridge
  replaces ours as the published condition.** This is §5.9's threat, recorded there.
- **Three oracle-based guards on the bridge — all pre-freeze, all recorded in the
  pre-registration** (they reuse the scripted per-task oracles §5.5 and §2.1 already
  require):
  1. **Granularity:** on every Lane B task, the oracle's `tool_calls` under the bridge must
     be **≤** its `tool_calls` under `shell`. A bridge chattier than the shell on a scripted
     optimal path is fixed before freeze — fine-grained verbs are exactly how a "faithful"
     bridge loses a cost comparison its opponent authored.
  2. **Distinctness — the anti-degeneracy test, made operational:** the bridge is *distinct*
     iff the oracle completes at least one Lane B task with **≥ 15% fewer** `tool_calls`
     under the bridge than under `shell`. The verdict belongs to the **§6.2.4 non-OmniSim
     reviewer**, is rendered **before any scored run**, and is frozen with the
     pre-registration.
  3. **Consequence, stated so it cannot be improvised after unblinding:** if the bridge is
     *not* distinct, Webots's within-sim `shell+tools` − `shell` delta is published as
     descriptive only — and **nothing is removed from F-comparative**: both Webots
     conditions still run, and §2.4's best-condition rule selects among them mechanically.
     Degeneracy never deletes a comparison. (This revision's first draft said "F-surface is
     evaluated on OmniSim alone", which was a no-op — F-surface is defined on OmniSim only —
     and left the real consequence to be decided after seeing data.)

### 2.3 The decision metric, frozen (closes revision 1's open question 1)

- **Completion gate:** passes out of n, reported with Wilson 95% CI and `n` printed, per
  SPEC §3.5.
- **Decision cost metric: median `tool_calls` over passing runs.** Chosen because it is
  recorded identically in both conditions and on both simulators by our own runner; it is
  largely insensitive to endpoint latency — which is ours to fix and must not masquerade as
  agent inefficiency or efficiency (§3 Phase R item 7) — and to machine, cache state, and
  model pricing. Two honesty notes ride with it: its crudeness is accepted and stated (a
  `read_file` and a long-running step count one each), and latency still couples to call
  counts through agent *strategy* — a 23 s scene read pushes an agent toward fewer, larger
  reads — which is another reason Phase R item 7 lands before any scored run, not after.
- **Selection-effect guards:** per-condition pass counts are printed beside every cost
  ratio, and the all-runs (passing + failing) median is published beside the passing-runs
  median as a sensitivity row — §2.4's cost channels condition on passing runs, which is a
  selection effect whenever pass rates differ, and budget censoring (par times are
  author-set) acts on the same metric; the report header says so.
- **Reported beside, never summed, never the decision variable:** `tokens_in`, `tokens_out`,
  `tokens_cache_read` — separately, per SPEC §3.1's never-sum rule; `usd` at list price;
  and time with **`t_agent_s` leading and `t_total_s` printed beside it** — which resolves
  SPEC §9.1 ("fix before pre-registration") for this campaign. (The row-writer currently
  writes the two clocks as the same number — §0.2 — so this resolution is contingent on
  Phase R item 2 separating them before any scored run.)
- **Concurrency exclusion (Amendment 2; product lane, §2.7):** a row flagged
  `measured_under_concurrency` — produced while another lane was active on this machine —
  has its **time columns excluded from every latency statement** (`f_eval.py` nulls
  `t_agent_s`/`t_total_s` for flagged rows at load and the report header prints the excluded
  count). Wall clock measured under contention is not a measurement (§5.3); the flagged row's
  verdict, `tool_calls` and token counts are contention-insensitive and remain fully scored.

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

**Revision-3 scope note — which sub-rule applies in which lane; the F text above is
unchanged.** In the product-level lane (§2.7) there is no `shell`/`shell+tools` split, so
**F-surface (conjunct i) is not evaluable there and is not evaluated there** — it is
evaluated in the retained API-runner lane when that lane runs, and the report says so in its
first paragraph. **F-comparative (conjunct ii) applies in the product lane**, with the
best-condition rule degenerating mechanically to the single `claude_code` condition per
simulator (a one-condition simulator's best condition is that condition — no selection
occurs). The death condition applies unchanged. A product-lane result is never presentable as
conjunct-(i) evidence, in either direction (§4).

### 2.5 What we will say if F triggers

Pre-committed, so the retreat is scripted rather than negotiated. The template, instantiated
per conjunct:

> **We tested whether OmniSim's agent-facing surface makes an LLM agent more effective than a
> POSIX shell on the same simulator [conjunct i], and whether an agent gets more done on
> OmniSim than on another simulator [conjunct ii]. On five closed-loop tasks, two conditions,
> five repeats per cell, on OmniSim and `<sim>`, at `<model>`, on machine `<id>`: it does
> not. `<the conjunct(s) that fell, with the per-task tables for both channels>`. We withdraw
> the corresponding claim. It was never measured, the first measurement pointed the other
> way, and this one — pre-registered — confirmed it. Every trace, row and artifact is
> published.**

And in the same commit as that publication:

- `AGENTS.md` (the AgentBench routing row and any "agents get more done" phrasing),
  `README.md` where applicable, and this file are edited to remove the withdrawn framing.
- A dated entry lands in [`simulator-comparison.md`](simulator-comparison.md) **§9.2 "things
  this document previously got wrong"**, alongside the eight already there. The withdrawal
  is not a footnote, not a "clarification", and not filed as documentation drift. (Note:
  §5.1 and §8 of that file carry **packaging and capability** claims, not throughput claims —
  verified 2026-07-31 — so they survive a withdrawal untouched; revision 1's instruction to
  strip "throughput framing" from them had no actual target.)
- **What survives is listed explicitly**, because it is separable and it is real: bitwise
  determinism on 10/10 rows across three machines, scoped per
  [determinism-scope.md](../benchmarks/determinism-scope.md) 📊; ~~OmniSim/ODE as OmniBench's
  correctness star 📊~~ — ⚠ **2026-08-08: this one does NOT survive, and it was mis-stated
  besides.** `bdc02139` deleted ODE, so the claim is unquotable: OmniBench lane 1's second
  in-engine arm no longer runs, and the surviving Newton-vs-analytic comparison is a
  *different, weaker* claim that has not been re-measured. It was also never a *solver*
  result — `omnisim-ode` outscored `omnisim-newton`, bare MuJoCo scored fine on the same
  scenes, and the deficit was in our integration layer ([correctness-scope.md](../benchmarks/correctness-scope.md)). Strike it from any withdrawal-survivors list; a GPU-batched path on a laptop 3060 where Isaac Sim will not start
  without an RT-core card ✅; the **packaging** claim — "plain HTTP and JSON with no ROS, no
  DDS, no in-process Python and no editor plugin" ([simulator-comparison.md](simulator-comparison.md)
  §8), which `agent-native-api.md` (archived 2026-09-02, see [docs/ARCHIVE.md](../ARCHIVE.md)) §1.3 states precisely as *packaging,
  not capability*, **including its own caveat that the advantage evaporates when the agent's
  environment already has ROS**; 33 structured diagnostic codes; and the 10/10 driveability
  score, which is a **capability** measurement and was never a throughput one. None of those
  sentences contains the word "faster". If conjunct (i) alone falls, the Lane C capability
  table also survives — under its §2.1 quarantine, capabilities are not throughput.

### 2.6 Why the pre-registration is the whole point

This is the discipline that makes our physics numbers credible: OmniBench found **five bugs in
our own Newton integration** and published them at the top of the report; it published T6
penetration getting **46% worse** next to the creep win; it retired our own 17.4× throughput
headline as a graphed-vs-ungraphed artifact. Those admissions are why the surviving numbers
are worth reading.

It is also precisely the discipline **SimBenchmark lacked** — the canonical cross-simulator
accuracy suite, run by RaiSim's own developers (◐, per their own site), who win most of its
rows, last pushed 2021-09-05 ✅ (`simulator-comparison.md` §2.1). We are in the identical
structural position: author and contestant (SPEC §8.2). The difference has to be mechanical,
not attitudinal — a frozen hash, a published rule, and a scripted retreat — because "we'll be
honest about it" is what every vendor benchmark says.

### 2.7 The product-level lane — the Phase W headline instrument (revision 3)

**The decision (owner, 2026-08-01, before any scored run):** the headline Phase W campaign is
a **product-level comparison**. The instrument is **Claude Code** — pinned CLI version and
pinned model id, recorded in every row — run headless (`claude -p`); one cell = one fresh
session in a **staged clean workspace**. There is a **single condition per simulator, named
`claude_code`**: the product as shipped. For OmniSim that is the repo/product with its
`AGENTS.md` — which Claude Code auto-injects into the session, and which is **counted
deliberately**: the docs are part of the product surface (SPEC §6.3's docs-are-part-of-it
logic, applied rather than ablated here; the docs-*ablation* cell stays the API lane's,
Phase R item 6). For upstream Webots the condition is its own install plus its own docs;
upstream ships no `AGENTS.md`, and **that asymmetry IS product surface — stated in the report
header, not hidden.**

**Answer-key quarantine — the load-bearing validity rule of this lane.** A product-level cell
hands the agent the product tree, and the product tree contains the benchmark. Cells
therefore run in staged workspaces that **exclude the benchmark's own answer key**:
`tests/benchmarks/agentbench/` in its entirety (graders, verdicts, fixtures, `preregister/`),
this plan document, and any document that reveals grader thresholds or task internals. The
staging manifest — what is included and what is excluded, with hashes — is published with the
campaign. **A cell whose workspace is shown to have contained answer-key material is
INVALID.**

**The answer-key redaction ruling (Amendment 2, ratified by the owner 2026-08-01).** Benchmark
**self-references** in staged docs are answer-key; **capability documentation stays.** A
sentence or clause in a staged document that names an agentbench task id (the
`A1_`/`B1_`/`B2_`/`B3_`/`C1_`/`C2_` patterns or their task-shaped short spellings) or that
names AgentBench itself is removed by the staging pass (`cc_lane/stage_workspaces.py`); the
documentation of what a product capability *does* — `--fail-on-runaway` being the canonical
example — ships untouched, because doctoring capability docs would void the product
comparison while shipping the benchmark's own task names prices file-reading rather than
competence. Every redaction is recorded in the published staging manifest as an exact
before/after diff, so a reviewer can re-derive each removal; the earlier "AGENTS.md §3b ships
its C2 disclosure verbatim" known-disclosure is superseded by this ruling and the manifests
say so.

**What is unchanged — deliberately.** Lanes, tasks, graders and the F arithmetic are
§2.1–§2.4's, verbatim: the same Lane A control and Lane B decision set, the same n (5 per
Lane B cell, 10 for A1), the same graders — they judge artifacts and recorder evidence in
physical units, so they are **driver-independent**: the same grader grades a Claude Code cell
and an API-runner cell — the same "no row, no result" rule (§0.2), and the same
sequential-cells rule (§5.3). **F-comparative applies** with best-vs-best degenerating to the
single `claude_code` condition per simulator; **F-surface (conjunct i) is not evaluable in
this lane** — there is no shell/tools split — and moves explicitly to the retained API lane;
the report must say so (§2.4 scope note).

**Concurrency protocol — how §5.3 is enforced in this lane (Amendment 2, 2026-08-01).**
Cells may run in parallel across different **(task, sim) groups**; within a group they are
**strictly sequential**, and **same-task cells never overlap** under any scheduling — the
shared-global evidence risk §5.3 documents (`husky_random`'s hardcoded trace dir) is
task-shaped, so the exclusion is a per-task lock held for the whole cell, not an operator
promise. At most **two lanes run on this machine, split by simulator** (lane A = omnisim
cells, lane B = webots cells), bounded mechanically by a **file-lock semaphore of N engine
slots** (N = 2 default, `--engine-slots`) acquired around **engine-heavy phases only**: the
headless session's *whole run* counts as engine-heavy for omnisim cells — the agent may
launch the engine at any point — and the grading/recorder pass always takes a slot. A
**pre-cell resource guard** (free RAM ≥ 4 GB, CPU load not saturated; psutil with a cheap
wmic/procfs fallback) skips-and-retries-later rather than starting a starved cell. Every row
produced while another lane was active is flagged **`measured_under_concurrency: true`** in
`agent_artifacts`, and the reporting rule in §2.3 excludes flagged rows' time columns from
every latency statement — verdicts, `tool_calls` and tokens are contention-insensitive and
stand. Finally, a `claude -p` refusal that is a **usage/rate limit is recorded as a deferred
attempt, not a failed run**: the worker waits (default 15 min, configurable) and retries the
same cell; the one-run-per-cell rule counts only sessions where Claude actually started
working.

**Instrumentation honesty.** Every row records: the Claude Code version, the model id, cost
and turns **as reported by the headless JSON output** — carrying a `metrics_source` note
stating these are the instrument's self-reported figures — and `tool_calls` **counted from
the session transcript**. Unmeasured fields are `null`, never a number nobody measured. The
instrument itself is **not frozen** — Claude Code updates change behavior — so the
pre-registration pins the version and the report header states that replication means *"at
this version"*. That is a stated weakening versus the API lane's byte-hashed conditions,
recorded as threat §5.11 alongside the ecological-validity gain that motivates the trade.

**The retained API-runner lane.** Everything revision 2 designed and froze about conditions,
the Webots tool bridge and the oracle guards (§2.2–§2.3) is retained as the
**mechanism-isolation lane** — explicitly out of the headline path, to run when an API
credential exists. Nothing built is discarded; conjunct (i) is decided there.

**Implementation path:** the lane's staging and wrapper code lives in
[`tests/benchmarks/agentbench/cc_lane/`](../../tests/benchmarks/agentbench/cc_lane/)
(under construction at this revision; not frozen content — it is instrumentation, like the
runner scaffolding).

---

## 3. The phases

Cheapest-decisive-first, with the same correction as revision 1: the cheap decisive phase
cannot run first, because the runner had to exist before any condition was real. Most of that
prerequisite is now built; what remains is listed, not assumed.

### Phase R — the remainder. (free; ~1 engineer-week)

What is **done** (built + unit-tested, 2026-07-26/27): the standalone loop with enforced
budgets and token accounting; condition/tool-set/prompt hashes in the row; the sandbox with
port reservation and env stripping; the scripted-replay backend and e2e fixtures; the
neutral-core grader split; the committed Phase-0 verdicts; the Webots *grading* adapter.

What is **not**, in order:

1. **One real credentialed run.** The Anthropic backend has never made an HTTP call
   (`untested_until_credential`). A smoke run (n = 1, marked `exploratory`, never quoted)
   validates the round trip, real token numbers, and real `usd` before any budget arithmetic
   is trusted.
2. **The row guard.** A test asserting a scored row cannot carry `condition: "scripted"`;
   condition made an explicit, documented run parameter; `tokens_cache_write` added to the
   row; **`t_agent_s` separated from `t_total_s`** (the row-writer currently writes the same
   number to both — §0.2).
3. **Tracked results.** Decide and build the mechanism (a `results_published/` dir or
   equivalent) that makes campaign rows survive the machine that produced them — the
   "no row, no result" rule (§0.2) is unenforceable while `results/` is a gitignore.
4. **The Webots execution path.** Launcher + Webots-side Supervisor recorder implementing the
   [`webots-control-baseline.md`](webots-control-baseline.md) recipe (WSL2, offline asset
   pre-seed, per-process `WEBOTS_HOME`), producing the artifact set the grading adapter
   already reads.
5. **The Webots tool bridge** (§2.2) — whole published function reference, exclusions
   countersigned — plus the scripted per-task oracles on both simulators and the three
   oracle-based bridge guards (granularity, distinctness, consequence), all published for the
   V5 review window.
6. **The docs-ablation cell** as a read-root deny-list (`AGENTS.md` + `docs/developer/`
   removed), OmniSim-only (SPEC §6.3). If the ablation collapses OmniSim's result, the honest
   headline is **"our documentation is the moat"** — different from, and more defensible
   than, "our API is the moat".
7. **Re-measure the harness read paths and publish the endpoint latency table beside the
   campaign.** Promoted from revision 1's open question 5: `GET /scene/tree` measured
   **23.0 s** and `/robots` ~22–23 s on the 298-node Newton world
   (`agent-native-api.md` Appendix A.2, build `806b055c`) — *before* `--light` took
   `/sim/step` from 27 s to 0.034 s (`06a0e23d`) — and **no read path has been re-measured
   since** (verified repo-wide, 2026-07-31). A `shell+tools` agent losing on wall clock
   because our endpoints are slow is a fixable defect and must not be reportable as "the
   surface is useless"; conversely the fix must land *before* the campaign, not after a bad
   number.
8. **The launch-flake budget.** The engine's ~1-in-3 startup race (`03e988c5` item 4) already
   turned one C2 row INVALID. With 2 retries per launch, expect a few INVALID runs per phase;
   they are *attributed*, never silently retried inside a scored lane, and SPEC §3.3 governs:
   a cell over 20% INVALID after re-runs is **published as unreliable**, not published as a
   score.

Phase R licenses **nothing** on its own. It is what makes the rest licensable.

### Phase W — upstream Webots R2025a. 🚦 **THE GO/NO-GO GATE.** (free; days, after Phase R)

Same file-format lineage, same base engine, same robot/sensor/world heritage — no OmniSim
harness, no capture service, no MCP server, no `AGENTS.md`. Any OmniSim − Webots delta is
*exactly* the surface we added, with physics and format held nearly constant. It runs on a
laptop and it is the cheapest informative comparison in existence for us.

**Revision 3 — the headline instrument for this phase is the product-level lane (§2.7).**
Headline run count: Lane A (A1 at n = 10) + Lane B (5 tasks at n = 5) × **one condition
(`claude_code`) × 2 simulators = 70 scored runs** (20 + 50), plus the Lane C capability table
under its §2.1 quarantine. Every row records the Claude Code CLI version and model id, cells
run sequentially in staged clean workspaces under the §2.7 answer-key quarantine, and the
phase evaluates **F-comparative only** — F-surface belongs to the retained API lane (§2.4
scope note). Phase R items 1–2 (credentialed smoke run, row guard) gate the *API* lane, not
this one; items 3–4 and 7–8 (tracked results, the Webots execution path, read-path latency,
the flake budget) gate both lanes.

**Run count (retained API-runner lane, when a credential exists):** Lane A (A1 at n = 10) +
Lane B (5 tasks at n = 5), × 2 conditions × 2 simulators = **140 scored runs** (40 + 100),
plus the Lane C capability table (costed separately; Lane C cells run at n = 3, are marked
`exploratory`, and are never quoted as scores — their numbers appear only in the quarantined
table, §2.1). **If F triggers here — in either lane, scoped to the conjunct that lane can
evaluate — §2.5 is published and Phases G and I are cancelled** rather than pursued in the
hope of a better answer. The SPEC already commits to this (§6.1).

**Practical wrinkles this phase must handle** — each one is a way to accidentally rig the
gate; verified in [`webots-control-baseline.md`](webots-control-baseline.md) except where
another source is named:

- **`URDFRobot` is an OmniSim-only node** (ours, expanded at tokenize-file time — per
  `AGENTS.md`/`agent-native-api.md`, not a baseline-doc finding) and **upstream ships no
  Husky at all** (checked: zero matches across all 15,404 paths in the R2025a tag). A single
  shared `.wbt` is impossible and must not be attempted. Every task is stated as an
  **outcome in physical units** with each simulator using its native robot representation
  (SPEC §6.2.6); the robot asset is provided pre-converted in both containers — for Webots
  via upstream's own first-party `urdf2webots`, with the conversion published alongside the
  scaffolding. The asymmetry is published, not hidden: we import URDF natively, upstream
  needs a conversion step.
- **⚠️ Never install upstream Webots on the dev box.** The Windows installer writes
  machine-scope `WEBOTS_HOME` with no uninstall cleanup, and our own build exports
  `WEBOTS_HOME` as an alias — a collision that has already degraded `env_fingerprint.py`
  once. WSL2 or a container only; per-process env; `python -m omnisim doctor` +
  `env_fingerprint.py` before and after, and a fingerprint that moved between conditions
  voids the campaign.
- **Both cells run ODE.** Upstream has no Newton; §0's agents both independently pinned ODE
  on ours anyway. The physics is genuinely held constant — and Phase W therefore says
  **nothing about our Newton default**.
  > **🔴 2026-08-08 — PHASE W IS BLOCKED, AND IT NEEDS AN OWNER DECISION, NOT AN EDIT.**
  > Phase W holds physics constant by pinning `defaultPhysicsBackend "ode"` in **four
  > hash-frozen scored fixtures**. `bdc02139` deleted ODE, so the mechanism that made the
  > comparison fair is gone — **and the fixtures do NOT fail, which is what makes this
  > dangerous.** An explicit `"ode"` still *wins* (`OmSolid::effectivePhysicsBackendName`) and
  > resolves to an inert stub: the four worlds still load, still run, still get graded, and are
  > simulating **nothing at all** — no FATAL, no ERROR, no warning, nothing moves, nothing
  > collides. **Phase W will still produce a full set of scores, from worlds with no physics,
  > and nothing in the run signals it.** And it cannot simply be
  > repaired: [`tests/benchmarks/agentbench/preregister/FREEZE.md`](../../tests/benchmarks/agentbench/preregister/FREEZE.md)
  > states that **any post-freeze change voids the campaign**, so re-pinning the fixtures to
  > Newton is itself a campaign-voiding act. Phase W can therefore be neither **re-run as
  > frozen** nor **repaired without voiding**. The two available choices, both requiring an
  > owner call:
  > 1. **re-freeze on Newton and re-run from zero** — new hashes, new pre-registration, the
  >    prior freeze explicitly superseded (and note upstream Webots has no Newton, so
  >    "physics held constant" is no longer available as a property: the cross-sim arm becomes
  >    a genuinely different, weaker design); or
  > 2. **drop Phase W** and say so, recording the loss rather than quietly re-scoping it.
  >
  > 🔴 **And a hard warning for anyone reading results:** a Phase W result gathered **after**
  > `bdc02139` is not merely **NOT comparable** to a pre-deletion one — it is **actively
  > misleading**, because the OmniSim arm ran with no physics and said nothing about it. Do not
  > put them in the same table, do not treat a post-deletion run as a continuation of the
  > pre-registered campaign, and do not publish a post-deletion Phase W number at all until the
  > fixtures are re-based and re-frozen. Until then, gate any run with
  > `OMNISIM_REQUIRE_NEWTON=1` and assert the `.newton.json` sidecar — a zero exit code proves
  > nothing here. (This is exactly the failure class the ODE-retirement rule forbids: *a wrong
  > result is worse than a lost one*, and a reachable stub must warn. **It does not. Open
  > defect.**)
- **Grade sequentially, or in full per-run isolation** (§5.3). Wall clock measured under
  contention is not a measurement.
- **One honest asymmetry to state, not fix:** upstream Webots is decaying (48 commits/12
  months, last stable tag 2025-02-04). A Webots win would be very damaging to us; a Webots
  loss is only the *minimum* bar — Gazebo and Isaac are the live projects.

### Phase M — the model ladder, on our own sim (free compute; tokens only; ~1 week)

§0's null came from a **strong** model that reads source to route around a missing tool. The
question that needs no competitor install: **does the surface matter more as the model gets
cheaper?**

Design: the same Lane A (n = 10) + Lane B (n = 5) set × 2 conditions, on OmniSim only,
across **three model tiers** (small / mid / large) — exact model ids and dates pinned in the
pre-registration (this file's requirement; SPEC §9.5 supplies the ≥ 2-tier floor for any
published comparison). Report the `shell+tools` − `shell` delta **per tier** and the trend.

**If the gap widens as the model shrinks**, that is a narrower but genuinely defensible
claim: *"our surface lets cheaper agents succeed at simulator work"* — bounded to OmniSim,
needing no competitor, and directly useful to anyone paying per token. If the gap is flat or
inverts, that is a real finding too, and it *strengthens* F rather than rescuing E1.

**Composition rule (pre-committed):** if Phase W withdrew conjunct (i), any Phase-M positive
sentence must quote that withdrawal **in the same paragraph** — the ladder must not become a
substitute marketing channel for the claim that just fell.

**The measurement risk is a floor effect**, the mirror of the strong-model ceiling: a small
model that fails everything in both conditions yields Δ = 0 for a reason that has nothing to
do with the surface. Countermeasure: an `exploratory` n = 1 pilot **first**, purely to pick
rungs where the small model passes something in at least one condition, with the pilot rows
marked and barred from the claim.

### Phase G — Gazebo Jetty 10.0.0 LTS (free compute; 2–4 engineer-weeks)

Free in dollars, expensive in engineering, and the first phase that tests conjunct (ii)
against a *live* project. A fair fight means standing up its real surface:
`ros_gz/src/gz_simulation_interfaces` — `SpawnEntity`, `DeleteEntity`, `GetEntities`,
`Get`/`SetEntityState`, `GetEntityBounds`, `StepSimulation`, `Reset`/`Get`/`SetSimulationState`,
`LoadWorld`, `GetSimulatorFeatures`, `GetSpawnables`, `GetNamedPoses` — with tool definitions
generated **from their `.srv` files**, plus the `gz` CLI and `ros2 topic`/`service`. Handing
Gazebo a shell and calling it even is not a baseline, it is a rigged cell. (Gazebo, unlike
upstream Webots, *has* a first-party service surface — §2.2's distinctness test is not
expected to be close here.)

Work items: ROS 2 + `ros_gz` container, a `simulation_interfaces` client, SDF initial states
for the six tasks, a Gazebo pose recorder, tool defs + docs published for the SPEC §6.2
review + 30-day window before any number ships. Expect at least one `NOT_EXPRESSIBLE` cell
and expect to discover a capability we did not know they had (SPEC §6.4).

### Phase I — Isaac Sim (the only phase that costs money; 2–3 engineer-weeks)

**Hardware floor, verified:** RTX 4080 / 16 GB VRAM minimum, and GPUs without RT cores are
not supported — the local 3060 is out; this runs on a **RunPod 4090**. Compute is the cheap
part: order **$20–40** at ~$0.7/hr including image pull, bring-up, cells and re-runs (the
SPEC's §7 Phase-3 planning figure is ~$30–60 — same order; re-price before committing). The
engineering is the expensive part.

Planning figures to **verify before committing budget** (quoted, not re-fetched): ~13 GB
install + ~80 GB assets. On a metered pod with a network volume, an 80 GB asset pull is a real
line item — price it from NVIDIA's current requirements page, not from this paragraph.

**Pod discipline is not ours to improvise:** arm the delete-watchdog **before anything
else**, batch and detach, write results to the network volume, **TERMINATE** rather than
stop, and confirm `GET /v1/pods` returns `[]` at the end.

### 3.1 Run count and token budget, computed rather than asserted

One phase-cell = (A1 × 2 conditions × n = 10) + (5 Lane B tasks × 2 conditions × n = 5) =
**70 scored runs per simulator per model tier** (Lane C excluded; it is costed per phase and
never scored).

| scope | scored runs |
|---|---|
| **Phase W** — OmniSim + upstream Webots, one model | **140** |
| **Phase M** — OmniSim only, two *additional* model tiers | **+140** |
| **Phase G** — + Gazebo, one model | **+70** |
| **Phase I** — + Isaac Sim, one model | **+70** |
| Whole programme | **420** |

That is 1.94× revision 1's 216, from two SPEC-compliance corrections: n = 5 replaced the
SPEC-non-compliant n = 3, and A1 runs at its own n = 10 floor. §0's runs consumed roughly
**80k–350k tokens each** — an operator observation, not an instrumented figure (§0.2). Taken
at face value:

| scope | low end (80k/run) | high end (350k/run) |
|---|---|---|
| Phase W (140) | 11.2 M | 49 M |
| Phase M (+140) | 11.2 M | 49 M |
| Whole programme (420) | **33.6 M** | **147 M** |

So **order 10⁷ tokens per phase and order 10⁸ for the programme** — a factor-of-four spread
that is an artifact of never having recorded tokens. **Phase R item 1 fixes the arithmetic
before Phase W spends against it**: the SPEC's dollar estimates (§7: ~$200–600 Phase 1,
~$600–1800 Phase 2) are re-derived from the first ten instrumented runs rather than carried
forward. Phase M is not cheap in tokens (it equals Phase W); it is cheap in engineering and
dollars because two of its three tiers are cheaper models, and billed cost depends heavily on
the cache-read fraction, which the runner now records.

---

## 4. What each phase licenses, and the forbidden sentences

These are *exact sentences we may not write*, not sentiments to avoid.

### After Phase R

**May say:** nothing about E1. "The runner exists, the conditions are separable, the rows
carry their condition and their token counts, and a real model call has produced a real row."

**May NOT say:**
- ❌ anything at all about agent effectiveness — Phase R produces no task result
- ❌ "the previous null result was invalid, so the claim stands" — the null being
  under-powered does not restore a claim that was never measured

### After Phase W (OmniSim + upstream Webots R2025a)

**May say (product-level lane, the revision-3 headline):** *"Against our own parent engine —
same `.wbt` lineage, same base ODE physics, no OmniSim harness — **Claude Code
`<CLI version>` at `<model id>`**, given one sentence and a staged clean workspace, completed
k/5 vs j/5 of each closed-loop task (n = 5, Wilson CIs, resolution limit stated), at median
`tool_calls` X vs Y per success (counted from the session transcript; cost and turns are the
instrument's self-reported figures, `metrics_source` stated), on machine `<id>`. Here is the
per-task table, the F-comparative evaluation stated in the first paragraph either way —
F-surface is not evaluable in this lane and awaits the API lane — the staging manifest,
every trace, and the Lane C capability table. Replication means at this Claude Code
version."* The instrument and its version appear in the sentence, not in a footnote.

**May say (retained API-runner lane, if/when it runs):** *"a `<model>` agent given one
sentence completed k/5 vs j/5 …"* — as above, with both conditions reported and the
per-conjunct F evaluation including F-surface.

Lane C sentences take the capability form **with no competitor numerics in prose**: *"the
five-edit loop is a packaged loop on OmniSim (its cost is in the capability table); upstream
reaches the same outcome through a write-a-controller-and-restart idiom, or did not reach it
within budget (label and evidence in the table)."* All Lane C numbers live only in the
quarantined table (§2.1: `exploratory`, n = 3, printed below the F verdict).

**May NOT say:**
- ❌ "faster than Gazebo / Isaac Sim / MuJoCo" — neither was run
- ❌ any sentence in which "than" is followed by the name of a simulator we did not run
- ❌ "the most agent-driveable simulator"
- ❌ "agents get more done on OmniSim" **unqualified** — Phase W's entire scope is *versus
  our own parent*
- ❌ "our harness pays for itself" if F-surface triggered — that is the sentence F exists to
  stop
- ❌ **"the claim survived F"** or any equivalent ("F did not withdraw E1, so…") — survival
  licenses only this section's text (§2.4)
- ❌ **"Webots FAILED the hot-reload task"** or any Lane C outcome rendered as a competitor
  *failure* — Lane C outcomes are capability observations under SPEC §6.4's label
  discipline, and a `NOT_EXPRESSIBLE` is never rendered in a failure's colour in any chart
- ❌ any count or ratio over Lane C cells ("4 of 4 frontier capabilities") — a count is an
  aggregate
- ❌ any prose quote of a competitor's Lane C cost or budget exhaustion (§2.1's quarantine)
- ❌ any aggregate that mixes Lane C into Lane A/B numbers
- ❌ anything about our Newton default (both cells ran ODE)
- ❌ **presenting a product-lane result as an API-vs-API result** — the lane's instrument is
  a shipped coding agent, not the byte-hashed API runner, and the two are not interchangeable
- ❌ **presenting a product-lane result as evidence for or against conjunct (i)** — the lane
  has no `shell`/`shell+tools` split; F-surface is the retained API lane's business (§2.4
  scope note), in either direction
- ❌ quoting a product-lane cost or turn figure without its `metrics_source` caveat (§2.7 —
  the instrument's self-reported figures)
- ❌ anything with "first"

### After Phase M (model ladder, OmniSim only)

**May say:** *"On OmniSim, on these six tasks, the `shell+tools` − `shell` delta was X at
`<large-id>`, Y at `<mid-id>`, Z at `<small-id>` (n = 5 per Lane B cell, n = 10 for A1,
pinned ids and dates)."* — subject to the Phase M composition rule (§3): a conjunct-(i)
withdrawal, if one happened, is quoted in the same paragraph.

**May NOT say:**
- ❌ any comparative claim against another simulator — none was run
- ❌ "cheaper models work on OmniSim" without naming the exact model ids and the exact tasks
- ❌ extrapolating the trend to a model tier not run ("so a small local model would…")
- ❌ presenting the pilot's `exploratory` n = 1 rows as part of the result

### After Phase G (+ Gazebo Jetty 10.0.0)

**May say:** comparative claims against **Gazebo Jetty 10.0.0 and upstream Webots R2025a**,
on the pre-registered task set, at the pinned versions, on the stated machines, with the
`shell` / `shell+tools` split reported separately and Isaac named as absent **in every
statement**.

**May NOT say:**
- ❌ any Gazebo number before the 30-day correction window closes and a non-OmniSim reviewer
  has run each competitor cell (SPEC §6.2.3–4)
- ❌ quote the `shell+tools` column without the `shell` column beside it
- ❌ call a `NOT_EXPRESSIBLE` cell a failure, or render it in a failure's colour in any chart
- ❌ "beats Gazebo" without version, condition split, and Isaac's absence in the same sentence
- ❌ merge the V4 external-task column into ours

### After Phase I (+ Isaac Sim)

**May say:** the full E1 statement, bounded to the tested task set, models, versions,
machines and conditions — **with the documentation-asymmetry caveat (SPEC §6.3) in the same
paragraph, not in a footnote.**

**May NOT say:**
- ❌ "OmniSim beats Isaac Sim" as a bare sentence, ever
- ❌ any physics-fidelity or rendering claim derived from an agent-throughput result
- ❌ imply the hardware-accessibility win (Isaac needs an RT-core GPU; we run on a 3060 and
  on CPU-only ODE) is an agent-throughput win — it is a separate, already-measured,
  *stronger* claim and blending them cheapens both
- ❌ "the only simulator an agent can drive end to end"
- ❌ anything with "first"

---

## 5. Threats to validity, ours included

### 5.1 We authored the tasks and we are a contestant

The deepest problem (SPEC §8.2), and pre-registration does not fix it — it only stops us
reshaping the set *after* seeing results. A comparison decided by task selection is worthless
however clean the scaffolding is. Mitigation is V4 (an externally sourced column that
**leads the report if it disagrees with ours**, SPEC §8.3) plus the standing rule that
`A1_husky_swarm_10` is labelled what it is: **the owner's demo task**, reported separately so
it cannot silently inflate an aggregate. **No published comparison before the external column
exists.** Revision 2 adds a subtler variant of the same threat: **we also authored the lane
boundaries.** The guards are §2.1's oracle-based lane test with the reviewer's
countersignature at freeze, §2.4's no-escape clause covering lane membership explicitly, and
the from-Phase-G rule that an owner-only Lane B is named as such in the verdict's first
sentence.

### 5.2 The contamination limit (and the part of it we cannot fix)

V2's runner removes the *mechanical* leak — subagent auto-injection of `CLAUDE.md`/`AGENTS.md`
(asserted absent from the composed prompt by test). It does not remove the **model-priors**
leak, and nothing can: the model has read vastly more SDF, MJCF, USD and ROS 2 than `.wbt`,
and the `.wbt` in its training data is *Webots'*, not ours. That bias runs **against** us on
authoring and **for** Webots specifically on format familiarity. We do not know the sign of
the net effect and must say so with equal prominence to the docs-advantage caveat (SPEC §6.3).

### 5.3 Concurrency contaminates every shared global — grade sequentially

`husky_random` writes its per-robot trace to a **hardcoded global directory**
(`C:\tmp\husky_trace` / `/tmp/husky_trace`), keyed by robot name only, so two concurrent runs
overwrite each other's traces. That already happened during §0's A/B; both agents detected it
independently. Standing rules: **run graded cells sequentially**, one simulator process at a
time, or give every run its own filesystem namespace *and* its own `OMNISIM_LOG_PATH`,
harness `--port`/`--supervisor-port`, and scratch dir (SPEC §4.3 — the runner's `Sandbox` now
implements the port reservation and env stripping). Any metric read from a process-global
path in a concurrent campaign is void. And more generally: **wall clock measured under
contention is not a measurement** — §0's "~32 min both" is the cautionary example.

### 5.4 Strong-model ceiling effects

If a strong model passes everything in both conditions, a completion-scored design cannot see
a difference — and reporting that as "the surface adds nothing" over-reads a saturated
instrument. §0's author tier is exactly this shape. Revision 2's countermeasures: the
cost-first decision metric (§2.3) is the primary one — cost can separate what completion
cannot; the SPEC's binding budgets (timeout = 3 × par, max 60 turns, token caps) stay so cost
is bounded; the decision set is all closed-loop (§2.1); and Phase M's ladder distinguishes
"the surface does not help" from "this model does not need help".

### 5.5 🔴 The red-evidence rule — generalised from a vacuous assertion of our own

**For weeks, half of `A1.3` could not fail.** `ContactPoint.node_id` returned the queried
solid's own id, so every contact pair was keyed `(id, id)` and a robot-robot contact count
**could never be non-zero**. The check read green in every run, including the runs used to
validate the grader. Fixed in `03e988c5`; the A1 oracle still PASSes 10/10 — *now with a
check that works.*

> **The rule, standing, binding on every assertion in every lane:**
> **No assertion may enter a scored campaign until it has been observed FAILING on a
> deliberately wrong artifact, with that negative fixture named in the assertion's record.**
> A green assertion is not evidence that the assertion works. Red on a known-bad input is the
> only evidence there is.

**A `null` agent turning every assertion red does not satisfy this rule** — that is exactly
how `A1.3` hid. Status as of 2026-07-31: the four Phase-0 verdicts are now **committed**
(`phase0_validation/`, `1c93d6f1`), which makes the oracle/null/wrong/parade discrimination
checkable from a clean clone — real progress. The fixture set itself is **unchanged**:

| fixture | assertions driven red |
|---|---|
| `null` (no artifact) | A1.1–A1.10 — all ten, trivially. Validates none of them. |
| `wrong` (uncontrolled robots) | A1.4, A1.5, A1.6, A1.8 |
| `parade` (ten identical headings) | A1.8 only |
| `wrong` (B3 variant) | B3.2, B3.4 |
| `wrong` (C2 freeze-cheat) | C2.3, C2.4, C2.5 |

So **six** A1 assertions still have no targeted negative fixture — `A1.1`, `A1.2`, `A1.3`,
`A1.7`, `A1.9`, `A1.10` (`A1.1`'s only red comes from the `null` fixture, which validates
nothing) — including `A1.3`, the one that was provably vacuous: nothing in the set places two
robots deliberately interpenetrating. It remains **unvalidated** under this rule even though
the underlying bug is fixed. Operationally: the coverage table (§6 item 2) has one row per
assertion id and the fixture that turned it red; an assertion whose only red evidence is the
`null` agent counts as having none; an unvalidated assertion's cell is not quotable. The
three new Lane B graders (B1, B2, C1) are **born** with their negative fixtures or they do
not enter the freeze.

Two further traps this rule catches, both real. The first is **now historical**: ODE
auto-disabled an idle body after `physicsDisableTime` (default 1 s) and a sleeping body
generated no contacts, so a contact assertion had to use `?wake=1` or a window where the body
was known moving. ⚠ **Since `bdc02139` that premise is gone** — ODE is deleted, Newton has no
sleep, `physicsDisableTime` is a field nothing reads, and `?wake=1` is a **no-op that still
costs two steps**. The rule for Newton instead: native contact readback is **default-ON**
(since 2026-08-07), so a resting body genuinely reports its contacts and a contact assertion
needs no wake flag — but an empty contact set must still never be graded as "nothing is
touching" without a geometric cross-check, and `OMNISIM_NEWTON_NATIVE_CONTACTS=0` reverts the
default and reinstates blindness. And `run-headless`
certified a world in which a crate fell 348 km through a missing floor — only the opt-in
`--fail-on-runaway` distinguishes C2's broken variant from its fixed one. A grading lane
whose "pass" cannot tell those apart is not grading.

### 5.6 Instrumentation debt is a validity threat, not a chore

Revision 1's version of this section is partly paid down (§0.2 lists what landed). The
residue that still gates any campaign: the `condition:"scripted"` fallback with no guard
test; zero real-model rows; `tokens_cache_write` dropped from the row; the two clocks written
as one number; and — the big one — **`results/` is gitignored, so every scored row on record
is untracked local state**, and a new commit-message-only result (the Webots FAIL 5/10)
appeared *after* revision 1 named this exact failure mode. **A campaign cannot be more
falsifiable than its bookkeeping.** The "no row, no result" rule (§0.2) and Phase R items 2–3
exist for this.

### 5.7 Threats to the *null* — the ways §0 could be wrong in our favour

Symmetry demands these be listed too, and none of them is a reason to keep the claim in the
meantime:

- Only 2 of 5 tiers were touched; **inspect and iterate — where `/scene/frame`,
  `/scene/visible` and `/world/render_stats` live — were never exercised at all.** (Revision
  2's Lane B exists because of this line.)
- Both agents pinned **ODE**, so the surface's Newton-specific verbs and the `.newton.json`
  attribution path were never in the loop.
- The surface improved *materially* just before the A/B — `/capabilities`,
  `/scene/spawn`/`/delete`/`/set_pose`, `/sim/snapshot`/`/restore`, a repaired `/sim/reset`,
  and `--light` taking `/sim/step` from 27 s to 0.034 s — so this is **not** a stale-surface
  excuse: the full-surface condition had all of it and still did not win. Stated here
  precisely so nobody reaches for that excuse later.
- The read-path latencies were, and remain, unmeasured post-`--light` (Phase R item 7): if
  `/scene/tree` still costs ~23 s on a big scene, the surface loses wall clock for a fixable
  reason. That cuts in *both* directions and is why `tool_calls`, not time, is the decision
  metric (§2.3).

### 5.8 Where this sits relative to `tool-design-for-agents.md`

That document's thesis — **the tool contract is a first-order term in agent task success** —
is proven at the *primitive* level (the turn primitive: 56.7% of commanded angle delivered →
mean |error| 0.44°, n = 8, no prompt/model change) and its §5 states plainly that the
task-level lift *"has not been made"* and no task-level number may be quoted until one
exists — naming `omnilink-bench` as that next measurement. This programme is the
**cross-simulator** task-level lane beside it; §0 is its first data point, and the two lanes
answer different questions (does tool quality lift agent outcomes on OmniSim; does our
surface beat other surfaces).

The two results are compatible, and the distinction matters: the mobile-bridge win came from
**correcting a tool that installed a false belief** (`turn` reporting a commanded value it
did not achieve). The A/B tested **adding endpoints to an agent that already had a shell**.
Those are different interventions, and only the first has evidence behind it here. If F
triggers, the honest reading is not "tool design does not matter" — it is *"fixing a lying
tool pays; **adding** a truthful tool to a capable agent that already has a shell may not."*
That is a sharper principle than either doc currently states, and it is worth writing down
whichever way F lands.

### 5.9 We author the competitor's tool packaging

New in revision 2, created by §2.2: upstream Webots ships no packaged tool surface, so its
`shell+tools` condition is a bridge **we** write. Every incentive problem in §5.1 applies to
it doubled — a subtly lame bridge manufactures the comparative win, and a subtly *chatty* one
manufactures the cost win. Mitigations, all pre-committed and all mechanical where possible:
the **completeness rule** (the whole published function reference, exclusions countersigned —
curation-by-omission is the abuse fidelity rules alone cannot catch); the **granularity
guard** (the oracle must not pay more calls under the bridge than under shell on any Lane B
task); the **distinctness verdict** rendered by the non-OmniSim reviewer before any scored
run, with a stated consequence that deletes nothing (§2.2); publication before numbers, the
30-day window, reviewer-run cells, and a maintainer's better bridge becoming the published
condition; and §2.4's best-condition rule breaking near-ties toward the *cheaper* condition —
the direction that disfavours the bridge's author.

### 5.10 Results that live in commit messages keep happening

Named as a §0.2 defect in revision 1 — and then it happened again the next day (`3c995c9c`'s
Webots FAIL 5/10 grading, quoted nowhere else, reproducible from nothing). This is now a
standing rule, not an observation: **no row, no result** (§0.2). It binds this file too — no
number from this programme is quotable, including internally, until it exists as a tracked
row.

### 5.11 The headline instrument is a product we do not control and cannot freeze

New in revision 3, created by §2.7. The API lane's conditions are byte-hashed
(`tools_sha256`, `manifest_sha256`, the system-prompt hash): a replication can prove it ran
the same instrument. **Claude Code cannot be held to that standard** — it is a shipped,
auto-updating product whose system prompts, tool surfaces, context injection and internal
heuristics change between versions and are not ours to pin byte-for-byte. What we do
instead, and what it honestly buys:

- **Pin and record**: the CLI version and model id are pinned in the pre-registration and
  recorded in every row; the report header states that replication means *"at this
  version"*. A version drift mid-campaign voids the affected cells.
- **Self-reported metrics**: cost and turns come from the instrument's own headless JSON
  output and are labelled so (`metrics_source`); `tool_calls` is counted from the session
  transcript, which is ours to parse. Unmeasured fields are `null`.
- **The trade is the point, and it is stated rather than netted off**: this is a **stated
  weakening** of instrument control versus the API lane, accepted because the product-level
  question — *does an agent someone actually ships get the job done on this product as
  shipped?* — is the ecologically valid form of our positioning sentence, and no byte-hashed
  lab runner can ask it. The two lanes exist so neither has to pretend to be the other:
  the product lane carries the headline and this threat; the API lane carries the
  mechanism isolation and the byte-hashes.
- **The residual confound nobody removes**: Claude Code is made by the same vendor as the
  model, its scaffolding is tuned for that model, and its behavior on our repo may be shaped
  by `AGENTS.md` conventions this vendor's harness popularised. That cuts for us on OmniSim
  cells and is part of what "product surface" means here — but it is named, because a reader
  comparing simulators through this lane is also comparing how well each product fits one
  particular shipped agent, not agents in general. Generalising beyond the pinned instrument
  is forbidden (§4).

---

## 6. Order of work

| # | item | gate |
|---|---|---|
| 1 | **Phase R remainder** (§3): credentialed smoke run; row guard test (+ clock separation); tracked results; Webots launcher + recorder; Webots tool bridge + oracle guards; docs-ablation cell; **read-path latency re-measure**; flake budget | nothing downstream is valid without it |
| 2 | **Red-evidence coverage table** (§5.5) — every assertion in the six scored tasks has a named negative fixture that turned it red; B1/B2/C1 graders written (physical spec first) with fixtures at birth; the scripted per-task oracles double as §2.1's lane test and §2.2's bridge guards | no cell is quotable without it |
| 3 | **Freeze the pre-registration**: this file's §2 + the SPEC's task set, prompts, graders, thresholds, lane assignment (reviewer-countersigned), bridge verdicts. In the same change, repair the SPEC's dangling citations (§7.1 cited twice, does not exist — the content is the unnumbered Phase 0 block; L70's "§5.2" should be §4.5.1), add the missing-surface clause (a simulator with no packaged first-party toolset — currently unhandled by the SPEC, §2.2), and reconcile the SPEC's n-language with this file (n = 5; n = 10 for A1). Publish the SHA-256 in `tests/benchmarks/agentbench/preregister/`, timestamp publicly | required before any competitor run (SPEC §6.2.1) |
| 4 | **Phase W** — upstream Webots R2025a, in WSL2/container. **Evaluate F per conjunct.** | 🚦 if F triggers: publish §2.5, cancel G and I |
| 5 | **Phase M** — model ladder on OmniSim (may run in parallel with 4; needs no competitor) | licenses only the narrow per-tier claim |
| 6 | **V4** — externally sourced tasks as a separate column; from Phase G onward at least one V4 task in Lane B | required before any *published* comparison |
| 7 | **Phase G** — Gazebo Jetty, with its real `simulation_interfaces` surface, + review + 30-day window | licenses the Gazebo/Webots comparison |
| 8 | **Phase I** — Isaac Sim on a RunPod 4090, watchdog first | licenses the full E1 statement, caveats in-paragraph |

---

## 7. Open questions, recorded rather than resolved

1. ~~Which cost metric is the headline?~~ **Closed by §2.3** (revision 2): `tool_calls` is
   the decision metric; tokens reported separately per SPEC §3.1's never-sum rule;
   `t_agent_s` leads time reporting with `t_total_s` beside it (contingent on Phase R
   item 2's clock separation).
2. **Are `0.85` and the `+2` completion bar the right effect sizes?** Both are conventional,
   chosen for what small-n medians and n = 5 pass counts can express — not power-derived,
   and the report header says so (§2.4). A pilot could set them empirically — but then the
   pilot must itself be pre-registered as a pilot, or the thresholds become fitted to our own
   data.
3. **Should the docs ablation be a third condition rather than a separate cell?** Three
   conditions triples the design. Current answer: keep it a separate OmniSim-only cell
   (SPEC §6.3), implemented as the runner's read-root deny-list, and accept that it cannot
   run against competitors (we cannot ablate *their* docs without crippling them).
4. **What counts as "another simulator" for conjunct (ii)?** Upstream Webots satisfies the
   letter of V1 and is the weakest possible competitor — same lineage, decaying, no packaged
   surface of its own. F is written so Webots is enough to *withdraw* the claim and not
   enough to *confirm* it — the asymmetry we want — and the report header states it rather
   than leaving it to be inferred. A defensible published comparison needs Gazebo.
5. ~~Does the harness's own latency swamp the effect?~~ **Promoted to Phase R item 7**: the
   read paths are re-measured and the latency table published beside the campaign, and the
   decision metric was chosen (§2.3) to be *largely* insensitive to it — with the
   strategy-coupling caveat stated there.
6. **If conjunct (i) fails but conjunct (ii) holds** — no `shell+tools` advantage on OmniSim,
   but OmniSim beating the competitor in *both* conditions — the advantage is the simulator,
   the format, the defaults and the docs rather than the API. That is a real and publishable
   claim that E1 as written does not capture; F now reports per conjunct so the outcome is at
   least representable. Decide the exact wording before Phase G, not after.
7. **Can a Lane C task ever migrate into the decision set?** Only by a re-freeze *before* a
   campaign, passing §2.1's oracle lane test on every simulator in that campaign, with a
   grader in physical units and negative fixtures at birth, countersigned like any other
   lane assignment. Recorded now because the temptation will arrive with the first
   impressive Lane C table.
8. **Who judges the Webots bridge faithful?** The §6.2.4 non-OmniSim reviewer, during the
   30-day window — and the reviewer also renders §2.2's distinctness verdict and
   countersigns the exclusion list and lane assignment. If no reviewer can be found, the
   comparative cells **wait** — an unreviewed competitor cell is not run late, it is not
   run.

---

**When this file and the code disagree, the code wins — and update this file in the same
change.** When this file and a marketing sentence disagree, this file wins.
