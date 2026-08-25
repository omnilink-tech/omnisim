# LOOPBENCH — the closed-loop benchmark

**Status:** plan, 2026-08-02. Nothing measured yet. No rung may be quoted until
its row exists.

This replaces the head-to-head framing of the capability ladder as *the
comparison*. The ladder stays, in a narrower role (§7).

---

## 1. Why the previous comparisons could not settle anything

Two campaigns produced honest numbers that told us little, for two different
structural reasons. Both reasons are worth writing down, because a third
benchmark that repeats either is not worth building.

**Against upstream Webots (Phase W), the tasks had no resolution.** The
decision set was world-file editing, and editing a world file is editing text.
A capable model writes the file; every simulator with a text format ties. The
measured result — upstream at 4/5 where we were at 0/5 on one task, us ahead on
two others — is a fact about *those tasks*, not about the simulators. Nothing
about a text-editing task can separate two products whose formats are both
text.

**Against MuJoCo, the comparison is arithmetic.** OmniSim's Newton path is
`omnisim-bin → embedded Python → newton → newton.solvers.SolverMuJoCo →
mj_step`. **We run MuJoCo.** A contact-fidelity benchmark against MuJoCo is
therefore us versus ourselves plus three layers of adapter: the ceiling is a
tie and the expected result is a loss by exactly the adapter's overhead. That
is what T2 measured — a working friction pinch that needed eight environment
variables here, of which five have no `.wbt` field at all
([ladder-findings §6f](ladder-findings-2026-08-02.md)). The solver was never
the variable.

So: **do not benchmark the solver, and do not benchmark text editing.** Both
are settled in advance by structure.

---

## 2. What is actually different, stated so it can be wrong

The claim under test is **not** "OmniSim has more features". Feature counts are
unfalsifiable and every integrated product claims them.

> **THE CLAIM.** For a task that requires several capabilities *at once*, with
> **no human in the loop**, OmniSim's integration cost is materially lower than
> the alternatives — because the agent can observe and re-run the world from
> **outside the process it does not own**, and because previously-made skills
> are reusable artifacts rather than code to rewrite.

Two structural properties carry it, and only these two:

1. **Observation from outside the process.** MuJoCo hands you complete state —
   if you *are* the process. Isaac Sim gives you Python APIs — if you own the
   loop. An agent driving a simulator it did not launch, asking *what is on
   screen, what is touching, what did the controller print, did a joint hit its
   limit, is the object still held* — that is a service surface, and it is the
   thing we have and they do not ship.
2. **Composition.** Skills made earlier, packaged as versioned artifacts, and
   sequenced into a compound behaviour without rewriting them.

Everything else on the differentiator list — rendering, sensors, world
generation, damage, the GUI — is real but is **not** what this benchmark tests,
because each is individually replaceable with glue.

### What this claim predicts against us

A claim that predicts only wins is marketing. This one predicts:

- **We lose every single-capability rung.** Pure physics: MuJoCo. Pure text
  editing: anyone. Pure throughput at batch: Isaac.
- **We currently FAIL our own flagship rung, L4.** A `.wbt` does not carry its
  own physics, so an OmniSim result cannot be handed to a stranger and
  reproduced. Measured today. Pre-registered as a loss (§5).
- **If the glue a competitor's user must write turns out to be small, the claim
  is thin** and we publish that sentence.

---

## 3. The rungs

Each rung is stated as a **physical or evidentiary outcome**, never as an API
call. No rung may name a capability only OmniSim has — see the anti-rigging
rules (§6). The advantage, if it is real, must appear as **cost**, not as
impossibility.

### L1 — Observe and correct

The agent is handed a scene that is wrong in a way **only observation
reveals**: the commanded behaviour looks correct in the source, and the run
does something else. One sentence, no hint about the defect.

*Graded:* the defect is fixed AND the agent's own trace shows it **observed the
symptom before changing anything** — a fix that arrives without a prior
observation is not a pass, because guessing correctly is not closing a loop.

### L2 — Iterate to a numeric target

One sentence carrying a measurable target ("arrive within 5 cm and settle in
under 8 seconds"). No human between attempts.

*Graded:* the target is met, AND the trace shows at least **two
measure→adjust→re-measure cycles whose adjustments follow from the
measurements**. Hitting the target first try is not a pass for this rung; it is
a `NOT_DISCRIMINATING` cell and says the task was too easy.

### L3 — Compose

Given a library of skills produced earlier, achieve a compound goal that no
single skill achieves.

*Graded:* the compound outcome physically happens, AND the artifacts are
**reused rather than rewritten** (the ladder's `reuse_class`, decided by a
human reviewer, never by a grader).

### L4 — Hand off

A **second, fresh agent**, on a **clean checkout**, given **only the artifact**
the first produced, reproduces the measured outcome within tolerance.

*Graded:* pass/fail on reproduction. This is the rung that converts "it worked
on my machine" into a product property, and it is the one we expect to fail
today.

---

## 4. Columns, and the glue ledger

`omnisim`, `mujoco`, `webots`, `isaac` (cloud-gated, §8).

Every cell records, beside its verdict, the **glue**: lines of adapter, harness
or driver code the *benchmark author* had to write because the simulator did
not provide it, counted per column and per rung, with the files named. This is
how "integration cost" becomes a number instead of an adjective.

A column that passes a rung with 400 lines of glue and a column that passes it
with 20 have not done the same thing, and the table must show that.

---

## 5. Pre-registered outcomes — written before any run

| # | if this happens | then we say |
|---|---|---|
| 1 | OmniSim fails **L4** | publish it as a failure of our flagship rung, name self-containment as the cause, and fix it before re-running. **This is the expected result today.** |
| 2 | A competitor completes a rung end-to-end | publish it plainly; no rung is ours by right |
| 3 | Competitor glue is consistently **< 100 lines** per rung | the claim is thin; say so in the abstract, not the appendix |
| 4 | **No** rung separates the columns | the benchmark is uninformative; withdraw it and publish why |
| 5 | A rung passes on every column first try | that rung is `NOT_DISCRIMINATING` and is cut, not kept as a tie |
| 6 | We win only on rungs that name our own APIs | the benchmark was rigged; rebuild it |

Thresholds are frozen with the spec. **A threshold that moves after measurement
is worthless whichever way it moves.**

---

## 6. Anti-rigging rules

1. **Outcome, never mechanism.** A task says *"prove the block was held for ten
   seconds"*, never *"use `/sim/grips`"*.
2. **Every rung must be possible in principle on every column.** If a rung
   cannot be attempted on a competitor at all, it is not a benchmark rung, it
   is an advertisement. Cut it.
3. **The graders are simulator-neutral cores plus per-column adapters**, held
   apart by the existing AST vocabulary guard.
4. **Red evidence.** No assertion certifies anything until it has been observed
   FAILING on a deliberately wrong artifact.
5. **No row, no result.** A number in a commit message is not a measurement.
6. **The glue ledger is written by whoever builds the column**, before that
   column's first cell runs, and is not editable afterwards.

---

## 7. What happens to the capability ladder

It stays, demoted to its useful role: **the achievability floor.** Its scripted
oracles prove a rung's assets are possible on a column, so that a LOOPBENCH
failure is attributable to the agent rather than to an impossible task. It is
no longer the comparison and its grid is not a scoreboard
([ladder-grid](ladder-grid-2026-08-02.md) says so in its own words).

LOOPBENCH cannot start on a column whose ladder floor is unbuilt — which today
means OmniSim needs T2, T3 and T4 oracles first.

---

## 8. The Isaac column

Isaac Sim is the **strongest** competitor on this axis, not the weakest:
Omniverse ships extensions, USD, and a real API surface, and Isaac Lab is a
mature training stack. Any expectation that it fails these rungs is unfounded
until measured.

It needs a GPU pod. **Gated on the owner's explicit approval and a stated
dollar ceiling**, and not started until the OmniSim column is finished — paying
cloud rates to measure an empty row is the worst version of this.

---

## 9. Honest limits, recorded now

- **n is small.** Three repeats per cell is enough to see a coin-flip, not
  enough for a p-value. Say "three runs", never "reliably".
- **One machine.** Everything is `9722d23d12a3` unless a row says otherwise.
- **One model family.** Results are about Claude driving these simulators, not
  about all agents.
- **The author is not neutral.** We built one of the columns. The defences are
  the pre-registered losses, the neutral cores, the glue ledger and the red
  evidence — not our good intentions.
