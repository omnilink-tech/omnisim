# Verified Build Frontier

**Instrument id:** `agenticsimbench/verified-build-frontier-v0.1`

The marketable claim OmniSim needs is not “we have many demos.” It is:

> Given the same instruction, model, budget and hardware, an agent completes a
> larger verified build envelope on OmniSim than on the comparison systems.

`frontier.py` is the claim-safe report for that sentence. It derives its result
from AgenticSimBench rows; it does not introduce another grader, subjective
points, or a hand-curated demo score.

## The four tracks

| track | ordered levels | what reaching the end establishes |
|---|---|---|
| scene reasoning | B1 overlap audit → B3 quantitative measurement → B2 camera iteration | the agent can inspect, measure, change and numerically verify a scene |
| debug loop | C1 parse repair → C2 physical fall-through repair | the agent can close both authoring and physics-debug loops |
| autonomy scale | R1 sensed navigation → A1 ten-robot swarm | the agent can build closed-loop autonomy and scale it to ten independently moving robots |
| manipulation | R2 reach → R3 pick/place → R4 mobile manipulation | the agent can progress from actuation to contact-rich manipulation to a composite mobile-manipulation job |

A track score is the longest **contiguous** prefix passed. Passing A1 while
failing R1 is still valuable evidence and is shown in the task table, but it
does not become “2/2 autonomy.” This prevents one spectacular artifact from
hiding a missing prerequisite.

Tasks may occur in a future second track when they are a real prerequisite for
both. Track membership and ordering are versioned in `frontier.py`; changing
either requires a new frontier id.

## Measured is not automatically claimable

Every report carries two values:

- `measured_frontier`: the selected physical grader rows that passed;
- `claimable_frontier`: the same prefix after all publication gates in
  `readiness.py` are applied.

The publication gates require: the task is expressible, its deliverable
convention exists, **every assertion has non-null red evidence**, its oracle
passes and null fails on that simulator, and the arm has no task-scoped
bring-up blocker. Missing evidence produces an exploratory row, never a green
claim.

This distinction is the core of the instrument. A benchmark that can only go
green is a demo with a spreadsheet attached.

## Generate a report

```powershell
python tests/benchmarks/agentbench/frontier.py `
  tests/benchmarks/agentbench/results/<campaign>/rows.jsonl `
  --sim omnisim --condition codex_cli --model <pinned-model>
```

Add `--json` for the machine-readable report. Selection refuses duplicate
task rows and rows mixing suite ids, protocol ids, models or conditions.
AgenticSimBench v0.3 is a one-run-under-ceiling protocol, so silently choosing
the best repeat would invalidate the result.

## Current state — 2026-08-13

- Ten executable task/grader packages exist across the four tracks.
- OmniSim, upstream Webots and MuJoCo have runnable adapters, with MuJoCo
  intentionally limited to the tasks for which an MJCF fixture/deliverable
  convention exists.
- The generated red-evidence table is **25/61 assertions validated**. The
  remaining 36 are named in `phase0_validation/COVERAGE.md`.
- Consequently there is **no full claimable frontier today**. Physical pilot
  passes may be shown as exploratory engineering evidence, but “best place to
  build” is not yet licensed.
- A fresh, isolated agent must produce scored artifacts. The agent that builds
  or reads this benchmark has seen its graders and is contaminated by design;
  its output can test plumbing, never support a public score.

## The first marketable milestone

Finish the red fixtures and oracle/null gates for the two headline endpoints:

1. R1 → A1 on OmniSim and upstream Webots (closed-loop autonomy plus the exact
   ten-robot scale claim).
2. R2 → R3 → R4 on both arms (the strongest “build whatever” composite).

Then run one fresh pinned Codex cell per task/arm under the identical protocol.
The first honest headline should name the observed outcome, model, date,
machine and ceiling—for example: “Model X reached autonomy-scale 2/2 on
OmniSim and 1/2 on upstream Webots in one run per task under the 45-minute
ceiling.” Do not shorten that to “best simulator” unless the declared primary
comparator set has actually been run.
