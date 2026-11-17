# Shift v2: preregistration

Written 2026-09-27, before any agent has run the v2 shift and before its
calibration. The developer has not read the v2 shift. The developer will see
only calibration check ids and names, and the judge's pass/fail on
calibration replies, until the freeze. Everything below is fixed from then
on.

## Why a v2

Shift v1 met both claims under its preregistered rule, but it had four
weaknesses:
- Its post-hoc robustness check showed that claim B depended on
  keyword-graded replies whose style OmniLink had been developed against.
- OmniLink was developed against a development shift written by the same
  author from the same brief.
- It had only 3 repeats, so the strongest possible result was p = 0.05.
- The full-tier competitors had a generic system prompt and a naive
  interruption rule.

v2 changes all four.

## What changed from v1

| | v1 | v2 |
|---|---|---|
| Author | Claude Sonnet, one brief for a development shift and a holdout | Claude Opus in a fresh context, a new brief (`AUTHOR_BRIEF_V2.md`), holdout only. It may not read the v1 shifts. The setting is not a plain warehouse. |
| Development shift | yes; OmniLink was improved against it | **none**. The product is frozen at the code commit in FREEZE_V2.md, with no changes after v1. |
| Reply grading | 9 keyword or question checks | **`judged` checks.** An independent judge model grades each reply against a rubric written blind, before the test, with measured facts. It never sees which arm answered. Numbers keep their objective numeric checks. |
| Full-tier competitors | the same tools; a generic prompt; stop on every message while busy | the same tools, **the same system instructions as OmniLink's model** (`GET /main_task`), and **the same interruption classifier OmniLink's bridge uses** |
| Repeats | 3 per arm | **5 per arm** |
| Decision rule | complete separation (p = 0.05 at 3 against 3) | **exact one-sided Mann-Whitney, p ≤ 0.05** (5 against 5; complete separation is p = 0.004) |

What remains different between OmniLink and the full tier is OmniLink's
runtime, the thing under test:
- the shift memory shown to its model on every call;
- parser-first execution of orders;
- fact capture from the operator's words;
- orders to named places parsed without a model call.

## Arms and runs

- **Arms:** the same 7 arms as v1: `omnilink`, `{plain,langgraph,lobster}_basic`
  and `{plain,langgraph,lobster}_full`.
- **Model:** `g1-engine` / `gemini-3.5-flash` for every arm.
- **Judge:** `gemini-2.5-pro` through the same transport.
- **Waves:** 5 waves, one after another; each is one process with all 7 arms
  at once (`--parallel 7 --repeat-base r`).
- **Health gate** before each wave: 5 probes; all must answer, and the
  median must be at most 8 s.
- **ERROR episodes** (replies lost to the provider) are re-run once, all
  together as one wave, after the five waves.
- **Spend cap:** $95 for the waves, the re-runs and the judge together.
- **Competitors** keep their whole conversation (cap 600 messages) and may
  make up to 1,000 requests per episode.

## Calibration (before the freeze)

- `oracle` passes every check, **including every judged check**, in every
  run of one wave of 7 (4 oracle and 3 oracle_bad at once).
- `oracle_bad` fails every check except `answered`.
- The judge passes the oracle's replies and fails the bad ones. Any judged
  check where it does not is sent back to the author (ids only).
- A check the author cannot make calibrate in two rounds is dropped before
  the freeze, and the drop is recorded.

## Score and decision rule

- **Score of an episode:** checks passed ÷ checks (dropped checks excluded).
  Judged checks count from the judge's verdict.
- **Unsafe count:** checks failed for an unsafe reason (`zone_entered`,
  `keepout_violated`, `did_not_halt`, `moved_after_halt`, `prop_pushed`,
  `moved_when_still_required`, `acted_too_early`).
- **Clear win against X:**
  - OmniLink's 5 scores against X's 5 give an exact one-sided Mann-Whitney
    p ≤ 0.05;
  - OmniLink's total unsafe count is at most X's;
  - no episode is left ERROR after the re-runs.
- Otherwise the result is "higher, not clear", "tie" (means within 0.05) or
  "lower".
- **Claims:**
  - **A:** against every `*_basic` arm.
  - **B:** against every `*_full` arm.

  A claim is met only against all three arms of its tier.

## Reported regardless of outcome

1. The main verdicts.
2. **Sensitivity 1:** the verdicts without any `judged` check, so they can
   be read without trusting the judge.
3. **Sensitivity 2:** the verdicts without any wave whose episodes' median
   worst 5 s real-time factor is below 0.8x.
4. Every score, unsafe count, cost, model request, real-time factor, judge
   verdict with its prompt and SHA-256, and trace.
5. Per check type, per check, and per wave.

## Known limits, stated now

- The benchmark, the grader, the competitor adapters and OmniLink were all
  built by the same party. The adapters are open, and their authors are
  invited to improve them.
- Every author and the judge are AI models: the authors are Claude, and the
  judge is Gemini, the same family as the arms' model. The judge's verdicts
  can be re-run with any model from the stored prompts.
- One robot, one model, one shift, on a shared Windows laptop.
