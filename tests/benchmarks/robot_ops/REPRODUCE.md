# Reproducing the robot-operations benchmark

This is how anyone can check the warehouse-shift results for themselves, from
a free re-analysis of our evidence up to a full re-run on their own machine.
Every number we publish comes from a file in this directory. Nothing is
reported that a reader cannot recompute.

| Level | What it checks | Cost | Time |
|---|---|---|---|
| 1. Re-analyse our evidence | that the verdicts follow from the recorded data | free | 1 min |
| 2. Re-grade every trace offline | that the grader gives the recorded grades | free | 5 min |
| 3. Re-judge every reply with your own judge | that the reply verdicts do not depend on our judge model | cents | 10 min |
| 4. Re-run the whole campaign | that the result holds on your machine, with your keys | ~$90 in model use | 4 h |

## What was compared

One robot, the Clearpath Husky in OmniSim, works one scripted 30-minute shift.
The shift was written blind by a separate AI author. Operator messages arrive
on a clock, and a grader scores what the robot measurably did: 30 ms pose
samples, the obstacle prop's position, and every reply.

There are seven arms. All use the same model (`gemini-3.5-flash`) through the
same transport, on the same robot bridge:

- **`omnilink`:** OmniLink's runtime on the bridge's `/prompt`, i.e. parser
  first, then the model, then a safety gate, with its shift memory.
- **`plain`, `langgraph`, `lobster` × `basic`:** each framework with four
  motion tools, handling one message at a time.
- **`plain`, `langgraph`, `lobster` × `full`:** each framework with every tool
  OmniLink's model gets (`GET /tools`) and the same system instructions
  (`GET /main_task`, v2 on). They interrupt with the same classifier the
  bridge uses. The adapters are in `omnisim/ops_bench/competitors.py`.

The bridge's rules apply to every arm alike: route planning, keep-out zones,
who may lift a rule, stopping a push, and the shift record in
`get_robot_state`.

- **Rules fixed in advance:** `shift/PREREGISTRATION.md` (v1) and
  `shift/PREREGISTRATION_V2.md`.
- **What was frozen:** `shift/FREEZE*.md`.
- **Every deviation:** stated first in `evidence/SHIFT_V1_RESULTS.md` and the
  v2 results file.

## Level 1: re-analyse our evidence (no simulator, no keys)

From a clone of the repository at the result's commit:

```bash
python tests/benchmarks/robot_ops/analyze_shift.py \
  tests/benchmarks/robot_ops/evidence/shift-holdout-v1-wave0 \
  tests/benchmarks/robot_ops/evidence/shift-holdout-v1-wave1 \
  tests/benchmarks/robot_ops/evidence/shift-holdout-v1-wave2 \
  tests/benchmarks/robot_ops/evidence/shift-holdout-v1-rerun --drop=5,10
```

For v2, add `--rule=mw` and pass the `shift-holdout-v2-*` directories, with
the `--drop` list from `FREEZE_V2.md`. The output holds:
- every score and unsafe count;
- the exact Mann-Whitney p for each pair;
- the verdicts;
- the sensitivity results.

## Level 2: re-grade every trace offline

```bash
python -m omnisim ops-bench regrade tests/benchmarks/robot_ops/evidence/shift-holdout-v1-wave1 \
  --suite tests/benchmarks/robot_ops/suites/shift_holdout_v1.json
```

This recomputes every check from the recorded pose trace, prop positions and
replies, and prints any mismatch with the recorded grade. The grader is
`omnisim/ops_bench/suite.py`, one pure function.

## Level 3: re-judge the replies with your own judge (v2)

Every judge prompt is stored verbatim in `<run>/judgments.jsonl`, with its
SHA-256. To grade the same replies with a different model, copy a run
directory, delete its `judgments.jsonl` and `graded.jsonl`, and run:

```bash
python -m omnisim ops-bench judge <copied run dir> \
  --suite tests/benchmarks/robot_ops/suites/shift_holdout_v2.json \
  --engine <engine> --model <any model id> --key-file <your OmniKey file>
```

Then run Level 1 on the copies. The judge never sees which arm answered.
Its instructions are `JUDGE_SYSTEM` in `omnisim/ops_bench/judge.py`.

## Level 4: re-run the whole campaign

**You need:**

- **OmniSim.** Windows has the only prebuilt package; Linux builds from
  source per `AGENTS.md` §2. Run `python -m omnisim doctor`: the verdict must
  be OK, with a physics runtime present.
- **An OmniLink account and an OmniKey** (free tier works; `python -m omnisim
  key`).
- **A Google model key** connected to it (`python -m omnisim byok`) that can
  serve `gemini-3.5-flash` and `gemini-2.5-pro`. Our runs used a Vertex AI
  service account at location `global`.
- **The competitor stack** (each run's `manifest.json` records it under
  `competitor_stack`):
  - Python 3.12 with `tests/benchmarks/robot_ops/competitor-requirements.lock.txt`
    installed in `tests/benchmarks/harness_comparison/.venv` (LangGraph
    1.2.12). The lock is a byte-identical copy of
    `tests/benchmarks/harness_comparison/requirements.lock.txt`, the file our
    runs used; that directory is not in the public repository, so create the
    venv there yourself;
  - Node ≥ 22;
  - https://github.com/openclaw/lobster cloned into
    `tests/benchmarks/harness_comparison/vendor/lobster` at commit
    `71bd8145055498116de35623dbea0d7dda9b4cd5`, with `npm install` then
    `node node_modules/typescript/bin/tsc -p tsconfig.build.json`.
- **A quiet machine.** 7 real-time simulators run at once. We measured
  0.95–0.97x real time with 7, and 0.84x or worse with 21. Close other
  simulators. Excluding the repository from antivirus real-time scanning
  helps. Every episode records its real-time factor.

**Run it:**

```bash
# 1. The shift loads and its scripted correct/wrong robots calibrate (free).
python -m omnisim ops-bench run --suite tests/benchmarks/robot_ops/suites/shift_holdout_v2.json \
  --arms oracle oracle_bad oracle oracle_bad oracle oracle_bad oracle --parallel 7 \
  --out <dir> --key-file <OmniKey file>

# 2. The campaign: 5 waves of 7, then the re-runs, then the judge, then the analysis.
bash tests/benchmarks/robot_ops/shift/run_campaign_v2.sh <OmniKey file> 0 <your out prefix>
```

**What should reproduce:**
- the verdicts;
- the ordering of the arms;
- roughly the size of the gaps.

**What will not reproduce exactly:** the scores. The model is sampled, the
provider's latency varies (its 429s and slow minutes are recorded as ERROR
and re-run), and a real-time simulator on a different machine drifts
differently.

## Machine the published runs used

`python projects/policies/common/env_fingerprint.py` output, recorded in each
FREEZE file:
- machine `9722d23d12a3`: AMD64 Family 25 Model 80, 16 cores, RTX 3060
  Laptop, Windows 11;
- Python 3.12;
- newton 1.5.0 / mujoco 3.11.0.
