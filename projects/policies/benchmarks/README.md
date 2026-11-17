# OmniSim policy-block benchmarks

This folder is the acceptance layer for the Shadowing + Skill Library + BATON block.
It separates three claims that must not be conflated:

1. `skill_lib.py audit` proves the versioned catalog is structurally consistent, its
   generated registry is current, and every reproduced launcher has the same deploy env.
2. Shadowing quality is measured in-engine with the policy's declared kinematic ruler
   (WBMATCH for gait shape) plus the legitimacy ruler. A manifest records that evidence.
3. BATON sequence quality is measured end-to-end from machine-readable cycle verdicts,
   physical-task events, falls, minimum height, and duration. `policy_bench.py` applies
   the thresholds in `suite.json` to a run's logs.

Quick start:

```bash
python projects/policies/skills/skill_lib.py audit
python projects/policies/skills/skill_lib.py benchmark list
python projects/policies/skills/skill_lib.py benchmark command g1_box_delivery_e2e
python projects/policies/skills/skill_lib.py benchmark score g1_box_delivery_e2e \
  --rl _scratch/foot_redesign/box_place_arrest_final_rl.txt \
  --mpc _scratch/foot_redesign/box_place_arrest_final_mpc.txt
```

The G1 benchmark runs on the documented lambda=0.9 weight-bearing balance harness; it
is not a free-standing humanoid result. Benchmark cases must declare their support
configuration and reference machine. New cases are not `verified` until their evidence
record names a machine, engine, date, and measured result.

The current suite contains the verified physical box-delivery cycle and a cross-morphology
Go2 walk→turn→walk candidate. The Go2 scorer already measures policy loads, gait-gated
switches, height, roll, turn drift, and post-turn progress; it remains a candidate until a
fresh run is recorded with resolvable machine provenance. Add cases only when the deploy
emits enough machine-readable evidence to fail closed. An upright final frame, a pose-match
score, or process exit code alone is not an acceptance benchmark.

Compact, versioned reference verdicts live under `results/`; raw simulator logs remain
runtime artifacts. The suite validator requires every `verified` case to point at a readable
PASS result for the same benchmark, in addition to naming its machine and engine.
