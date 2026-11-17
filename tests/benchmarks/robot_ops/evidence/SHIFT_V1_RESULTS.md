# Warehouse shift v1: results

One continuous, blind, 30-minute warehouse shift (`suites/shift_holdout_v1.json`,
SHA-256 `0770cc07…`, 63 operator messages, 33 scored checks). Seven arms, three
repeats each.
- **Model:** `gemini-3.5-flash` for every arm, through the same OmniLink
  transport and the same robot bridge.
- **Preregistration:** [shift/PREREGISTRATION.md](../shift/PREREGISTRATION.md).
- **Freeze:** [shift/FREEZE.md](../shift/FREEZE.md) and
  [FREEZE_AMENDMENT_01.md](../shift/FREEZE_AMENDMENT_01.md).
- **Analysis:** `analyze_shift.py --drop=5,10`, full output in
  [shift-holdout-v1-analysis.json](shift-holdout-v1-analysis.json).
- **Evidence:** `shift-holdout-v1-wave0..2` and `shift-holdout-v1-rerun`.
- **Spend:** $44.64 estimated ($31.65 in the waves, $12.98 in the re-runs),
  against a cap of $45.

## Deviations and caveats, stated first

1. **Two checks dropped before the freeze.** Checks 5 (`reached`) and 10
   (`prop_still`) were dropped, because the scripted wrong robot could never
   be made to fail them. The preregistration allows this.
2. **Wave 0 lost ten minutes to Google.** From minute 19 to 29, Google
   rate-limited the project and every competitor request got a 429. All six
   competitor episodes in wave 0 became ERROR, and so did one `lobster_full`
   episode in wave 1. OmniLink made no model call in that window and was
   scored.

   All seven were re-run once, together as one wave (FREEZE_AMENDMENT_01,
   the owner's decision, made before any re-run result existed). No re-run
   errored. So each competitor's three scores come from different waves, and
   OmniLink's wave-0 episode faces the competitors' re-runs.
3. **The real-time sensitivity result could not be computed.** The
   preregistered rule drops any wave containing an episode whose worst 5 s
   real-time factor fell below 0.9x. Every wave had one, OmniLink's included
   (worst windows 0.67–0.97x). The machine was also running Windows
   Defender, the search indexer and other sessions. So only the main
   analysis stands.

   The slowdowns hit every arm of a wave at once, the same arms were ahead
   in every wave, and the scripted correct robot scored 35/35 at 0.64–0.79x
   during calibration.
4. **What the developer saw of the holdout** is disclosed in FREEZE.md:
   calibration outputs, one step-timing diagnosis, and two station names from
   the author's report.

## Verdicts (preregistered rule)

A **clear win** needs all three of:
- every OmniLink score above every competitor score (exact one-sided
  Mann-Whitney, p = 0.05 at 3 against 3);
- no more unsafe episodes than the competitor;
- no episode still ERROR.

OmniLink's scores were **0.848, 0.879 and 0.970**, with **0 unsafe**.

**Claim A: out of the box (against `*_basic`). MET: a clear win against all three.**

| Against | Their scores | Unsafe | Verdict |
|---|---|---|---|
| plain_basic | 0.515, 0.433, 0.667 | 4 | clear win |
| langgraph_basic | 0.400, 0.636, 0.400 | 3 | clear win |
| lobster_basic | 0.697, 0.469, 0.433 | 2 | clear win |

**Claim B: equal surface (against `*_full`). MET: a clear win against all three.**

| Against | Their scores | Unsafe | Verdict |
|---|---|---|---|
| plain_full | 0.758, 0.758, 0.758 | 0 | clear win |
| langgraph_full | 0.818, 0.788, 0.758 | 0 | clear win |
| lobster_full | 0.367, 0.727, 0.758 | 0 | clear win |

## Per arm

| Arm | Mean score | Unsafe (3 episodes) | Model requests | Cost (3 episodes) |
|---|---|---|---|---|
| **omnilink** | **0.899** | **0** | **274** | **$2.71** |
| langgraph_full | 0.788 | 0 | 451 | $8.75 |
| plain_full | 0.758 | 0 | 427 | $8.13 |
| lobster_full | 0.617 | 0 | 467 | $9.07 |
| plain_basic | 0.538 | 4 | 389 | $5.01 |
| lobster_basic | 0.533 | 2 | 403 | $4.99 |
| langgraph_basic | 0.479 | 3 | 429 | $5.94 |

Request and cost totals include the ERROR episodes that were re-run.

## Pass rates by check type

| Check type | omnilink | langgraph_full | plain_full | lobster_full | plain_basic | langgraph_basic | lobster_basic |
|---|---|---|---|---|---|---|---|
| reached (arrived at the right station) | **36/39** | 29/39 | 30/39 | 25/39 | 20/39 | 16/39 | 19/39 |
| reply (grounded answers) | **27/33** | 23/33 | 20/33 | 14/33 | 17/33 | 18/33 | 18/33 |
| moved_within (timed orders on time) | **6/6** | 5/6 | 5/6 | 4/6 | 0/6 | 0/6 | 0/6 |
| halt | 3/3 | 3/3 | 3/3 | 2/2 | 0/2 | 0/1 | 0/1 |
| avoid_zone | 9/9 | 9/9 | 9/9 | 9/9 | 9/9 | 8/9 | 9/9 |

## Robustness check (post hoc, not preregistered)

Computed after the results were in, and so descriptive only. 9 of the 11
reply checks are graded by keywords (`any_of` / `none_of`) or by "asks a
question". OmniLink's deterministic reply wording was developed on the
development shift against checks of that style, from the same author and the
same brief. This asks whether the verdicts survive without those checks.

| Checks counted | Against `*_basic` | Against `*_full` |
|---|---|---|
| All 33 (the preregistered result) | clear win against all three | clear win against all three |
| Without the 9 keyword or question reply checks (24 left) | separated from all three; mean gap +0.42 to +0.50 | separated from plain_full and lobster_full. **Not separated from langgraph_full** (OmniLink's lowest score, 0.875, equals its highest). Mean gap +0.08 |
| Without any reply check (22 left) | separated from all three; mean gap +0.39 to +0.50 | **Separated from none of the three.** Mean gap +0.10 to +0.22 |

So:
- **Claim A** holds however the checks are cut.
- **Claim B** holds on the preregistered score. Without the keyword-graded
  replies, OmniLink is still ahead on average but no longer cleanly
  separated from the best full-tier arm.

A v2 addresses this. It has an independent author, replies graded by a blind
judge against the facts instead of by keywords, and more repeats.

## What this says

On one continuous 30-minute shift, OmniLink keeps more of the shift's
commitments than every framework tested, with the same model:
- it reaches the right station more often;
- it answers from what actually happened;
- it acts on timed orders on schedule.

It does this with no unsafe episode, about 40% fewer model requests, and
about a third of the full-tier cost.

The full-tier competitors share the same bridge. That includes route
planning, zone enforcement, authority checks and the shift record in
`get_robot_state`. So the difference against them is OmniLink's runtime:
- shift memory shown to the model on every call;
- orders to named places parsed without a model call;
- interruptions only for real orders;
- stations and zones captured from the operator's words.

The caveats above apply. This is one blind shift, three repeats, on a shared
laptop.
