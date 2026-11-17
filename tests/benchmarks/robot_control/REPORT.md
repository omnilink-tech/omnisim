# Robotics control benchmark development report

2026-09-24. This is benchmark development evidence, not a market-leadership claim. It extends the earlier single-robot parser/workflow comparison to three robot fixtures, measured state, ordered trajectories, multi-turn operation, object transfer and failure recovery.

## What is implemented

`python -m omnisim control-bench` validates task packs, freezes implementations and configuration, runs private simulator episodes, verifies saved evidence without model calls, and generates scorecards. The development suite has 33 episodes across Husky, TurtleBot3 Burger and OmniArm 6. Six implementations share the same model, tools, gate, observations and execution limits: OmniLink parser integration, plain model loop, LangGraph, Lobster SDK, and parser-enabled versions of both frameworks.

The parser-enabled frameworks are attribution controls. If they tie OmniLink, that result must be shown. These are controlled integrations; they are not complete hosted-product configurations or a test of the entire robotics-control market.

## Validation history

| Run | Purpose | Result |
|---|---|---|
| calibration-01 | Initial physical launch | Interrupted after runtime cache access and process-permission failures; no model requests |
| calibration-02 | Four physical fixture checks | 3/4 passed; pick/place exposed the grader's incorrect interpretation of the gripper boolean |
| calibration-03 | Corrected physical checks | 4/4 passed; all grades reproduced offline |
| full-calibration-01 | All development fixtures using expected actions | 28/33 passed; five shared gate failures; no infrastructure errors; all grades reproduced offline |
| pilot-01 | First six-implementation development comparison | 36/36 episodes recorded; unchanged frozen inputs; all grades reproduced offline; $0.04368475 estimated model cost |
| pilot-02 | Same tasks with equal bounded protocol repair | 36/36 episodes recorded; all six implementations passed 4/6; no infrastructure errors; all grades reproduced offline; $0.06577220 estimated model cost |

The gripper fix requires live holding state, measured attachment identity and an observed lift of the named object. It did not relax a failed physical destination. Old evidence remains available.

All physical runs above used Newton's MuJoCo CPU solver. The recorded host has an NVIDIA GeForce RTX 3060 Laptop GPU, 16 logical CPU cores, Windows 11 and Python 3.12.14. Exact machine, runtime, model and source provenance lives in each `lock.json`; Newton sidecars record the solver that actually ran. FAST-mode task latency is simulated workflow wall time, not real-robot cycle time.

The benchmark tests pass: **29/29**, covering adversarial motion grading, missing observations, actual framework execution, bounded protocol repair, accounting and offline detection of an incorrect recorded grade. Both pilots' model IDs and per-request/per-episode costs were independently recomputed from saved provider metadata and frozen rates. No credential occurred in the saved evidence. Combined estimated model cost: **$0.10945695**.

## Completed pilot-02 results

| Implementation | Completed episodes | Unwanted-motion episodes | Model calls | Estimated model USD per success | Wall seconds per success |
|---|---:|---:|---:|---:|---:|
| OmniLink parser integration | 4/6 | 1 | 14 | 0.003502 | 7.910500 |
| Plain model loop | 4/6 | 1 | 15 | 0.003199 | 8.539000 |
| LangGraph | 4/6 | 1 | 12 | 0.002339 | 7.152250 |
| Lobster SDK | 4/6 | 1 | 13 | 0.002948 | 8.140250 |
| LangGraph + same parser | 4/6 | 1 | 9 | 0.001944 | 5.836250 |
| Lobster SDK + same parser | 4/6 | 1 | 11 | 0.002511 | 6.800750 |

All six passed the forward/reverse mission, false-condition branch, arm joint target and two-turn object transfer. All six failed the unspecified-distance and transient-recovery cases. All-attempt costs and times include failed episodes. These are six development cases with one repetition each: the numeric ordering is not evidence of a stable ranking. **This pilot does not support OmniLink superiority or a cost-leadership claim.**

The automatically generated [pilot-02 scorecard](evidence/pilot-02/scorecard.md) and [machine-readable results](evidence/pilot-02/scorecard.json) include paired intervals and claim blockers. The [first pilot](evidence/pilot-01/scorecard.md), [full calibration](evidence/full-calibration-01/scorecard.md), original freezes and all raw rows remain available.

Regrade the final pilot without simulator/model calls:

```powershell
python -m omnisim control-bench verify tests/benchmarks/robot_control/evidence/pilot-02
python -m omnisim control-bench report tests/benchmarks/robot_control/evidence/pilot-02
```

## Findings that matter before publication

1. **The shared gate rejects legitimate instructions.** All three transient-recovery fixtures are blocked by the instruction not to repeat a successful action. Both mobile multi-turn fixtures are blocked by “reverse 0.25 metres without turning.” These failures occurred even when the calibration oracle supplied the correct tool calls. They cannot be attributed to LangGraph, Lobster or their planning quality.
2. **Unspecified distance can still produce motion.** The first pilot reproduced physical motion on “back up a bit,” including in the OmniLink parser integration. The benchmark expects clarification. The measured displacement remains in the evidence even if a later response fails to parse.
3. **The shared model protocol needs bounded repair.** In pilot-01, the model sometimes followed a valid tool plan with ordinary prose. Fail-fast parsing turned that into an episode failure. Pilot-02 adds the same protocol-error feedback and six-round bound to every implementation, repeats the original instruction/JSON contract, and charges all repair time and tokens. Do not pool these two implementations of the runner.

The arm transfer uses the shipped simulator-assisted grasp attachment. It establishes agent use of that primitive, not friction-only grasp quality or hardware transfer. The common gate is part of the shared environment, so this lane does not compare competing safety systems.

## What must happen before a homepage claim

Fix and regression-test unintended actuation, overbroad gate refusals and duplicate-success recovery in the product. Use this development pack to verify fixes, preserving each frozen run. Review and optimize every baseline under equal budgets, including the parser-enabled controls. Obtain a separately authored and reviewed holdout; freeze code, configuration and the decision rule before running it. Add full product entry points and representative perception/navigation/fleet tasks before extending the claim to those capabilities.

Publish the exact tested implementations, model, tasks, denominators, uncertainty, failures, cost basis and reproduction command. The current statistical rule tests lower model cost and lower task time with success non-inferiority; it does not establish higher success or superiority to untested products. Token-derived cost excludes subscriptions, hosting, local compute and integration labor.

See [README.md](README.md) for reproduction and [SPEC.md](SPEC.md) for the scoring contract. Raw evidence, including failed attempts, is retained under `evidence/`.
