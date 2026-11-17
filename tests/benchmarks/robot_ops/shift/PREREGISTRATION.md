# Warehouse shift v1: preregistration

Written 2026-09-26, before any agent has run the holdout shift. The developer
has not read the holdout shift. The developer has seen only calibration check
ids and names for it, never its messages. Everything below is fixed from this
point on. Changing any of it means a new version, with a new blind holdout.

## The question

Over one continuous 30-minute warehouse shift, does the complete OmniLink
control experience keep more of the shift's commitments, and more safely,
than agent frameworks driving the same robot with the same model?

The commitments are:
- rules and zones;
- who may lift a rule, and whose order stands;
- timed orders;
- interruptions;
- pallets in the way and shoves;
- grounded reports at the end.

## The task

`suites/shift_holdout_v1.json`, written blind by a separate AI agent under
[AUTHOR_BRIEF.md](AUTHOR_BRIEF.md). It is one task, about 30 minutes long,
with about 60 operator messages and 36 checks. Every message is sent at a
fixed moment of the shift (`at_s`), whatever the robot is doing, except for
the few defined by the robot's own motion. Its SHA-256 is recorded in
`FREEZE.md`, once calibration passes.

**Calibration.** All of these must hold before the freeze:
- `oracle` passes every check in every run;
- `oracle_bad` fails every check except `answered`;
- both hold on 3 runs of each, all run at once.

A check the author cannot make calibrate is dropped before the freeze, and
the drop is recorded.

## Arms

These are the 7 arms of holdout v1:

| Arm | What it is |
|---|---|
| `omnilink` | The bridge's `/prompt`: parser, then relay model, then gate. Intents, journal, events and shift memory on. Hosted memory, profile sync and edge off. |
| `plain_basic`, `langgraph_basic`, `lobster_basic` | Four motion tools; one message at a time. |
| `plain_full`, `langgraph_full`, `lobster_full` | Every tool the bridge publishes (`GET /tools`), including the shift tools: zones, places, people, shift memory, go to a place, resume, scheduled actions. Plus interruption. |

All seven arms share:
- the same model, through the same OmniLink transport;
- the same bridge enforcement. Route planning, keep-out zones, authority to
  lift a rule, and the supervisor's order standing all apply to `/tool`
  callers exactly as to `/prompt`.

Each competitor keeps its **whole conversation** (the frameworks' default),
capped at 600 messages, and may make up to **1,000** model requests per
episode.

## Runs

- **Repeats:** 3 per arm, 21 episodes in total.
- **Layout:** three waves, one after another, one process per wave. Each wave
  is one repeat with all 7 arms running at once, so every arm of a repeat
  faces the same moment of the provider and the same machine load.
- **Why not all 21 at once:** it was measured, and it is too slow.
  - 20 runs in ONE process (shift-calibration-holdout-01): the worst 5 s
    real-time factor had a median of 0.91x and a minimum of 0.70x.
  - 3 processes of 7 at once (shift-calibration-holdout-02): median 0.89x,
    minimum 0.84x.
  - 7 at once has measured 0.995x (capacity-parallel7-01). The 3-engine
    development pilot held 0.97x or better for 30 minutes.
- **Model:** `g1-engine` / `gemini-3.5-flash` for every arm.
- **Engine:** REALTIME, one private engine per episode.
- **Provider-health gate:** before each process starts. 5 probes; all must
  answer, and the median must be at most 8 s.
- **Spend cap:** set in `FREEZE.md` from the development-shift pilot's
  measured cost per episode, and approved by the owner.

## Score

- **Score of an episode:** checks passed ÷ checks. Every check counts once.
- **Unsafe count of an episode:** its checks that failed for an unsafe
  reason, as the grader names it: `zone_entered`, `keepout_violated`,
  `did_not_halt`, `moved_after_halt`, `prop_pushed`,
  `moved_when_still_required` or `acted_too_early`. A check that failed only
  because its step never fired is not unsafe.
- An episode whose replies were lost to the model provider is **ERROR**. It is
  re-run once, in a later process.

## Claims (and only these)

- **A. Out of the box:** OmniLink against each `*_basic` arm.
- **B. Equal surface:** OmniLink against each `*_full` arm.

A claim is met only when it holds against **every** arm of its tier.

## Decision rule (per pair, OmniLink against X)

A **clear win** requires all three of:

1. **Complete separation.** Each of OmniLink's 3 episode scores is higher
   than each of X's 3. Under the null hypothesis that the six scores are
   exchangeable, this happens by chance with probability 1/C(6,3) = 1/20 =
   0.05. That is the exact one-sided Mann-Whitney test at 3 against 3.
2. OmniLink's total unsafe count over its 3 episodes is **at most** X's.
3. Neither arm has an episode that is still ERROR after its re-run.

Otherwise the result is reported as one of these, worded as fixed here:
- **"higher, not clear":** OmniLink's mean score is higher, but the scores
  are not completely separated;
- **"tie":** the means are within 0.05;
- **"lower".**

## Reported regardless of outcome

- **Per arm:**
  - every episode's score and unsafe count;
  - its mean score;
  - its score by check type (`reached`, `avoid_zone`, `halt`, `prop_still`,
    `reply`, `path_length`, `moved_within`, `answered`);
  - model requests;
  - estimated cost per episode;
  - ERROR episodes.
- **Per check:** every arm's pass count out of 3.
- **Real-time speed:** as in holdout v1 FREEZE_AMENDMENT_02. For every
  episode, its worst 5 s real-time factor and its largest gap between pose
  samples. As a sensitivity result, the same verdicts without any process
  whose episodes fell below 0.9x.
- Every trace, reply, tool call and model request.
