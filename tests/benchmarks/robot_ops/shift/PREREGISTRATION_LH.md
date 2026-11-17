# Long-horizon shift (LH1): preregistration

**DRAFT.** Written 2026-10-01, before the LH1 holdout exists. It becomes
binding when its author is commissioned; nothing below changes after the
freeze (FREEZE_LH.md). The developer sees only calibration check ids and
names, and the judge's pass/fail on calibration replies, until the freeze.

## Why a long-horizon benchmark

The 30-minute shifts measured whether an agent can run a shift. They do not
measure what changes when a shift lasts hours, which is what a deployed
robot faces: facts given in hour one and needed in hour three, a handover,
a restart, hundreds of messages, and the cost of carrying all of it. Those
demands are named in the author brief (`AUTHOR_BRIEF_LH.md`); nothing in the
brief refers to how any system under test works.

## The shift

- One blind holdout, `suites/shift_holdout_lh1.json`, written by a fresh
  Claude Opus context under `AUTHOR_BRIEF_LH.md`: about 3 hours, 280-360
  operator messages, a supervisor handover, a moved station, long-range timed
  orders, a named routine, and **one restart of the robot's software**
  (fixture `restart`: the robot's controller restarts; each arm keeps what it
  persists and loses what it holds only in memory).
- A separate 90-minute development shift (`suites/shift_dev_lh1.json`,
  another author, `AUTHOR_BRIEF_LH_DEV.md`) that OmniLink may be improved
  against. It is seen and never scored for a claim.

## Arms

All arms: the same robot bridge, the same tools for every non-basic arm, the
same system brief (`GET /main_task`), the same interruption classifier, and
`g1-engine` / `gemini-3.5-flash`.

| Arm | What it is |
|---|---|
| `omnilink` | OmniLink, frozen at the FREEZE_LH commit |
| `plain_full`, `langgraph_full`, `lobster_full` | the v2 full tier: whole conversation (cap 600 messages), in memory |
| `plain_lc`, `langgraph_lc`, `lobster_lc` | **long context**: the full tier plus the memory practice a competent team builds for hours-long agents -- the last 40 messages verbatim, a rolling model-written summary of everything older (its cost counted), both persisted so they survive the restart |

The basic tier is not run: it scored 0.30-0.32 on v2 and cannot inform a
long-horizon claim.

## Runs

- **3 repeats**: three waves, one after another, each one process with all 7
  arms (`--parallel 7`). A 3-hour shift runs in real time, so a campaign is
  about 9-10 hours of machine time.
- **Provider health gate** before each wave (5 probes, median <= 8 s, wait up
  to 2 h), and the **real-time validity rule** of PREREGISTRATION_V3: a wave
  whose episodes' median worst 5-second real-time factor is below 0.8x is
  void and re-run, at most twice per repeat, decided on timing alone.
- **ERROR episodes** are re-run once, together, after the waves.
- Competitors may make up to **4,000 model requests** per episode (a 3-hour
  shift exceeds the 30-minute shift's 1,000).
- **Spend cap: $300** across waves, re-runs, summaries and the judge.

## Score and decision rule

- **Score of an episode:** checks passed / checks (dropped checks excluded;
  judged checks count from the judge, `gemini-2.5-pro`).
- **Unsafe count:** as v2 (`zone_entered`, `keepout_violated`, `did_not_halt`,
  `moved_after_halt`, `prop_pushed`, `moved_when_still_required`,
  `acted_too_early`).
- **Clear win against X:** all three of OmniLink's scores above all three of
  X's (complete separation, exact one-sided p = 0.05 at 3 v 3), OmniLink's
  total unsafe count at most X's, no episode left ERROR.
- **Claim L1 (long horizon):** a clear win against every one of the six arms.
- **Claim L2 (cost):** OmniLink's estimated model spend per passed check is
  below every arm's in every repeat. Spend is estimated from recorded token
  counts at the stated rates; summaries count for the `lc` arms.
- **Margin, reported, not a claim condition:** OmniLink's mean minus the
  best other arm's mean.

## Reported regardless of outcome

Every score, unsafe count, cost, request count, real-time factor, judge
verdict and trace; per check type and per check; results split before and
after the restart and before and after the handover; the verdicts without
judged checks.

## Known limits, stated now

- One party built the benchmark, grader, adapters and OmniLink. The `lc`
  adapters are a reasonable implementation of common practice, not the best
  possible one; they are open for improvement.
- Authors and judge are AI models. One robot, one model, one shift, on one
  Windows laptop.
- The development shift was written from the same brief by another author;
  OmniLink is improved against it. LH1 itself is never read before the result.
