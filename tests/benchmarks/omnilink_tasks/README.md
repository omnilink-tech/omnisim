# OmniLink agent benchmark suite

A small parameterised suite for measuring how well an OmniLink agent
(or any bridge-compatible agent) executes single-prompt tasks. Each
task is:

```text
   world             one of the OmniLink chat-demo worlds
   prompt            single-prompt instruction the agent runs once
   grader            programmatic predicate on the bridge's
                     /get_robot_state response after `timeout_s` seconds
```

The runner launches the world headlessly, hits `/prompt`, polls
`/get_robot_state` while the grader watches, and reports pass / fail
with the realised metric and wall-clock time.

## Tasks (initial set)

| ID | World | Prompt | Goal |
|---|---|---|---|
| `mobile_drive_1m` | `omnilink_husky.wbt` | `forward 1 meter` | Husky displacement ≥ 0.9 m from spawn. |

Add new tasks by appending a `Task` to `tasks.py`.

## Run

```bash
# Run every task with the bridge's local intent router (no OmniLink):
python tests/benchmarks/omnilink_tasks/run.py

# Run a single task:
python tests/benchmarks/omnilink_tasks/run.py --only mobile_drive_1m

# Run against a specific OmniLink engine (requires OMNI_KEY):
export OMNI_KEY=olink_...
OMNILINK_ENGINE=g2-engine python tests/benchmarks/omnilink_tasks/run.py
```

The runner produces a TSV of `task | mode | duration_s | pass | metric`
plus a console table at the end. Use the matrix script to aggregate
across engines:

```bash
bash tests/benchmarks/omnilink_tasks/matrix.sh  # planned; currently
                                                # hand-rolled
```

## Reporting

Each run drops a JSON record at
`tests/benchmarks/omnilink_tasks/results/<task>-<timestamp>.json` with:

  - `task`: id
  - `world`: world file
  - `prompt`: prompt text
  - `mode`: `"local"` (offline regex) or `"omnilink"` (relay attached)
  - `engine`: omnilink engine when mode=omnilink
  - `duration_s`: time from /prompt POST to grader returning pass/fail
  - `passed`: bool
  - `metric`: task-specific success measure (q error, TCP error, m driven)
  - `final_state`: last /get_robot_state response

Aggregate across runs to build a leaderboard (planned).

## Status

Phase 5.1 of the OmniLink integration roadmap. **Scaffold-only**:

  - 1 task defined
  - Runner works against the local intent router
  - JSON record output works
  - Matrix script (engine × task) is hand-rolled until the leaderboard
    aggregation lands
