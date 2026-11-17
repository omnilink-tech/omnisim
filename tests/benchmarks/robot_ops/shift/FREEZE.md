# FREEZE — warehouse shift v1

Frozen 2026-09-26, before any agent ran the holdout shift, under
[PREREGISTRATION.md](PREREGISTRATION.md). Nothing below changes until the
campaign ends.

## What is frozen

| Item | Value |
|---|---|
| Code commit (product, bridges, runner, adapters, grader) | `9dcca15dd` |
| Holdout | `suites/shift_holdout_v1.json`, SHA-256 `0770cc0771a4a6d8db38823ba6282d65f5a1bf589f045a1f35fc42a1203e7094` |
| Development shift | `suites/shift_dev_v1.json`, SHA-256 `8f99ccda370bdde3cfb118bfe5b4e07a4544edc7ec36fa44d553f8a6b02d188e` |
| Calibration | `evidence/shift-calibration-holdout-04` and `-05`: `oracle` passes 35/35 in 8 of 8 runs; `oracle_bad` trips every check except `answered`, 5 and 10 |
| Dropped checks | **5 (`reached`) and 10 (`prop_still`)**. `oracle_bad` never tripped them after two author rounds (the bridge now routes round pallets and stops a push, so a scripted wrong robot could not be made to fail them). They are dropped before the freeze, as the preregistration says, and scored nowhere: 33 checks count. |
| Arms | `omnilink`, `plain_basic`, `langgraph_basic`, `lobster_basic`, `plain_full`, `langgraph_full`, `lobster_full` |
| Repeats | 3: three waves, one after another; each wave is one process with all 7 arms at once (`--parallel 7 --repeat-base r`) |
| Engine / model | OmniLink `g1-engine`, `gemini-3.5-flash`, for every arm (Vertex AI, location `global`) |
| Rates (USD per million tokens) | 1.50 / 0.15 / 9.00 (input / cached / output including thinking) |
| Spend cap | $45 for the three waves together (pilot: omnilink ~$0.9, `*_full` ~$1.9, `*_basic` ~$1.0 per episode, so about $30 expected) |
| Competitor limits | whole conversation (cap 600 messages); at most 1,000 model requests per episode |
| Provider-health gate | 5 probes before each wave; all must answer, median at most 8 s; wait up to 2 h |
| Machine | `9722d23d12a3`, AMD64 Family 25 Model 80, 16 cores, RTX 3060 Laptop, Windows 11, Python 3.12.9, newton 1.5.0 / mujoco 3.11.0, engine binary `d8473e670ff0dedd` |

## The command (one per wave, r = 0, 1, 2, run one after another; `shift/run_campaign.sh` runs them)

```bash
tests/benchmarks/harness_comparison/.venv/Scripts/python.exe -m omnisim ops-bench run \
  --suite tests/benchmarks/robot_ops/suites/shift_holdout_v1.json \
  --arms omnilink plain_basic langgraph_basic lobster_basic plain_full langgraph_full lobster_full \
  --parallel 7 --repeat-base r --repeats 1 \
  --engine g1-engine --model gemini-3.5-flash \
  --cap-usd <what the earlier waves left of $45> --rates 1.5,0.15,9.0 --max-requests 1000 \
  --health-median-s 8 --health-max-wait-h 2 \
  --out tests/benchmarks/robot_ops/evidence/shift-holdout-v1-wave<r> \
  --key-file <the OmniKey file, never recorded>
```

- An ERROR episode (lost to the model provider) is re-run once, after the
  three waves, as its own wave of that single arm, into
  `shift-holdout-v1-rerun`.
- The analysis is `tests/benchmarks/robot_ops/analyze_shift.py --drop=5,10`
  over the three wave directories and the re-run directory.
- The waves are driven by `shift/run_campaign.sh`.

## Disclosures

- **What the developer saw of the holdout.**
  - The calibration outputs: check ids, check names, pass counts, and the
    robot's measured positions and tool results in calibration runs.
  - In round 3, to diagnose a check that never fired, the developer read the
    calibration tool results around steps s35 and s36. From these the
    developer learned that the robot was already at s35's destination. The
    developer did not read the messages.
  - The blind author's reports named two holdout stations ("shipping" and
    "receiving"), and said a timed order targets receiving.

  The product has nothing specific to any station name. Named places come
  only from what the operator says during the shift.
- **Product work since holdout v1.** All of it was built and tested on the
  development shift and development tasks only:
  - route planning, push detection, shove reporting, measured drive_to
    replies;
  - site memory (places, people, rules, visits, odometer), authority, the
    supervisor's order standing, resuming an order;
  - place orders, fact capture, selective interruptions, zone look-ahead,
    timed trips to places.

  The commits are listed in `git log 7cd19f4dc..9dcca15dd`.
- **Equal surface.** Every bridge rule applies to `/tool` callers exactly as
  to `/prompt`:
  - planning;
  - refusing targets inside zones;
  - stopping a push;
  - who may lift a rule;
  - the supervisor's order standing.

  `get_robot_state` carries the shift record for every arm. OmniLink's relay
  also sees the shift memory on every call, and its parser answers orders to
  named places without a model. That is OmniLink's runtime, the thing under
  test.
- **Competitor adapters changed after holdout v1.** They now keep the whole
  conversation (it was the last 24 messages) and may make up to 1,000
  requests. Both changes make the competitors stronger.
- **The shared machine.** Windows Defender, the search indexer and other
  sessions' processes were seen loading the CPU during calibration (round 4
  dipped to 0.69x). Each wave starts only when no other simulator is
  running, and every episode's real-time speed is reported, with the
  preregistered sensitivity result.
