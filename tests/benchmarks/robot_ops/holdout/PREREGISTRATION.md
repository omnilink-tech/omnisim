# ops-bench holdout v1: preregistration

Written 2026-09-25, **before** the holdout task set existed. No competitor arm
had run on any task, and no model provider was connected. Everything below is
fixed from this point on. Changing any of it creates a new version (v2), with
a new blind holdout and a new preregistration. v1's results stay published
either way.

## The question

On unfamiliar robot-operations tasks (standing rules, orders for later,
interruptions, disturbances, grounded reports and long shifts), does the
complete OmniLink control experience in OmniSim finish more jobs, safely,
than agent frameworks driving the same robot with the same model?

## Claims tested (and only these)

- **A. Out of the box.** OmniLink vs each `*_basic` arm (plain loop,
  LangGraph, Lobster). The four motion tools, one message at a time.
- **B. Equal surface.** OmniLink vs each `*_full` arm. The same tool surface
  OmniLink's model gets, plus interruption.

A claim is met only when it holds against **every** arm in its tier.

## Decision rule (per pair OmniLink vs X)

"**Clear win**" requires all three:

1. The 95% bootstrap interval of the task success-rate difference
   (OmniLink − X) has a lower bound **above 0**. The bootstrap resamples
   tasks, keeping each task's repeats together: 2,000 draws, seed 250925.
2. OmniLink's count of unsafe episodes is **≤** X's.
3. Neither arm has more than 10% of its episodes as ERROR (infrastructure).

Anything else is reported as what it is: "higher, not clear", "tie" or
"loss". Wording is fixed in advance:

- "clear win" only when the rule is met;
- "tie" when the interval contains 0 and the point estimate is within ±5
  points.

## Episodes

- **Tasks:** `suites/holdout_v1.json`, written blind by a separate AI agent
  under [AUTHOR_BRIEF.md](AUTHOR_BRIEF.md): 24 tasks, 4 in each of F1, F2,
  F3, F4, F6 and F9. The developer does not read its contents before the
  freeze. Its SHA-256 is recorded in `FREEZE.md`.
- **Arms (7):** `omnilink`, `plain_basic`, `langgraph_basic`,
  `lobster_basic`, `plain_full`, `langgraph_full`, `lobster_full`.
- **Repeats:** 2 per task per arm, 336 episodes in total. Order is task-major
  with the arm order rotated per task, so no arm always runs first.
- **Model:** ONE engine and model id for every arm, chosen when a provider is
  connected and recorded in `FREEZE.md` before the first scored episode.
  OmniLink's relay gets it through `OMNILINK_ENGINE` / `OMNILINK_MODEL`; the
  competitors get the same id through the same OmniLink chat transport and the
  same agent profile.
- **Engine:** REALTIME, one private engine per episode, one campaign at a time
  on the machine.

## Before the freeze (allowed)

- **Calibration of the holdout:** `oracle` must pass 24/24 and `oracle_bad`
  must fail 24/24. The developer sees only task ids and failing check names.
  A failing task goes back to the blind author with only those names. A task
  the author cannot make calibrate is dropped before the freeze, and the drop
  is recorded.
- **A capped pilot:** at most $3, on the DEVELOPMENT suite only, never the
  holdout. Its purpose is to find integration bugs in any arm's model path.
  Bugs found are fixed in every affected arm before the freeze, and listed in
  `FREEZE.md`.

## The freeze

`FREEZE.md` records:

- the git commit (product, bridges, runner, adapters, grader);
- the holdout's SHA-256;
- the engine and model id;
- the spend cap;
- the machine fingerprint.

After the freeze, nothing changes until the campaign ends.

## Spend and stopping

- **Cap:** $40 of model spend for the whole campaign, estimated from each
  request's reported usage at the provider's list rates. A request with
  unknown usage counts at the per-request maximum observed so far.
- **Transient errors:** a transient provider failure (429 or 5xx) is retried
  once with backoff. An episode that still fails is recorded as ERROR. Every
  ERROR episode is re-run once at the end. An episode that errors again is
  excluded from **both** sides of every pair it belongs to, and reported.
- **Early stop:** if the cap is reached, the campaign stops. Only complete
  task blocks (every arm and repeat of a task) are analysed. The stop is
  reported.

## Reported regardless of outcome

- Per arm:
  - success rate, with its interval;
  - unsafe episodes;
  - ERROR episodes;
  - model requests;
  - estimated cost per success;
  - wall time per success.
- Per family, with F9 shown separately. With 4 tasks it is descriptive only;
  no claim is made from a family alone.
- Per task: every arm's outcome.
- Adapter lines of code (`competitors.adapter_loc`).
- Every episode's raw trace, all stopped attempts, and the offline regrade.

## If OmniLink does not win clearly

v1 is reported exactly as it came out. The holdout is spent. Harder or longer
tasks can be pursued only as **v2**, which requires:

- a new family design, written and committed before any v2 result;
- a new blind author and a new holdout;
- a new preregistration.

Designing v2 tasks *from v1's per-task outcomes*, to find where OmniLink
wins, is not permitted: that is selecting the test to fit the result. v2 may
use v1's aggregate lessons (for example "long horizons separate the arms")
only if they are stated in the v2 preregistration.

## Known limitations, stated now

- **Harness and author:** the developer wrote the harness, the checks and
  OmniLink's recent fixes. The holdout author is another AI agent, and the
  agent harness gives every subagent the repository's `CLAUDE.md` /
  `AGENTS.md`, a general product description with no tasks, parser rules or
  findings.
- **Reply checks** are text heuristics.
- **Assisted conditions:** a single machine, simulated robots, and no
  hardware.
- **The `full` tier's interruption policy** (stop on every new message) is
  one reasonable design. Another adapter could do better. Its line count is
  reported so the reader can judge the effort.
