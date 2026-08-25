# OmniLink Long-Horizon Robot Control Benchmark

This benchmark measures the production PRO-agent stack, not a scripted
controller and not a language-model judge.

## LH-1: Hierarchical corners campaign

The operator gives Mission Captain one natural-language objective: complete
the current OmniSim mission end to end. Mission Captain must inspect its
specialist roster, form a delegation plan, and hand one self-contained mission
to Husky Maze. The specialist must read the authoritative world brief, plan
and execute a wheel-driven tour of all four maze corners, recover from any
partial motion, return to the start cell, and submit a verified mission claim.
Mission Captain may close the operator mission only by binding its own claim
to the completed delegation ledger.

World:
`projects/samples/demos/worlds/flagship/husky_maze_corners.omniworld`

Start/final cell: `(0, 10)`

Required visited cells: `(0,10)`, `(10,10)`, `(10,0)`, `(0,0)`

## What is measured

| Capability | Mechanical evidence |
|---|---|
| Planning | Captain emitted `execute_mission_plan`; its task is self-contained and identifies tour, return, verification, and recovery |
| Delegation | Ledger names a reachable specialist and contains a real sub-chat result |
| Long-horizon execution | Bridge's physics-derived visited trail contains all four corners and at least 20 settled cells |
| Recovery discipline | Plan is fail-closed; every ledger step is verified; active fault is absent |
| Goal completion | Bridge accepted a verified claim and robot is back at `(0,10)` |
| Supervisory closeout | Captain's claim references the exact completed `plan_id` |

The bridge rejects a corners claim if any corner is missing **or the robot has
not returned to `(0,10)`**. `accepted=true`, narration, target coordinates,
and the legacy single-goal `goal_reached` flag earn no completion credit.

## Scoring

The score is 0–100:

- 5: all three local endpoints were reachable
- 20: roster inspection and a well-formed supervised delegation plan
- 45: physical trail, all corners, return-to-start, and bridge verification
- 20: verified ledger, non-faulted settled robot, meaningful specialist work
- 10: Captain closeout bound to the same `plan_id`

Passing requires at least 80 points plus the hard gates: every corner visited,
return-to-start, bridge verification, verified plan, and bound Captain claim.

## Run

Start the corners world, Husky Maze runner, and Mission Captain runner, then:

```powershell
python tests/benchmarks/omnilink_long_horizon/benchmark.py --run-chat
```

To grade an already completed live session without spending another model run:

```powershell
python tests/benchmarks/omnilink_long_horizon/benchmark.py --grade-only
```

Results are written under `tests/benchmarks/omnilink_long_horizon/results/`.
