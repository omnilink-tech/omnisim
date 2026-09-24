# Control-language fixes and benchmark follow-up

2026-09-24. Follow-up to [the initial development report](REPORT.md).

The product changes are in the shared bridge gate and deterministic parser.
The benchmark tasks, grading tolerances, model-loop implementation and statistical
rule are unchanged from pilot-02. All implementations receive the same gate
fixes; the parser-enabled controls receive the same parser change as OmniLink.

## Changes

- Recognized vague mobile commands ask for a distance instead of requesting
  a guessed motion. The gate independently blocks guessed nonzero magnitudes
  for the covered vague phrases, including when unrelated numbers occur nearby.
- The gate scopes “without turning” to permit straight driving and continue
  refusing rotation. Other no-motion restrictions remain active.
- A success-qualified no-repeat clause no longer prohibits a requested first
  attempt. Adjacent unconditional duplicates in a batch remain blocked.

The gate still uses bounded language rules. It does not track success across
separate requests; an executor must use actual tool feedback for retry decisions.
No key requirement or ungated REST path was changed.

## Verification

The complete bridge test suite plus benchmark contract tests passed:
**855 passed, 1 skipped**. New regressions include negative controls that
must still refuse actuation, precise commands with unrelated vague asides,
unrelated numbers, real arm-tool schemas, and typed calls without utterances.

The first corrected full calibration, full-calibration-02, passed **33/33**
fixtures, compared with **28/33** before the product fixes. A subsequent
calibration was stopped before model testing to narrow the quantity rule
further; its partial evidence and explanation are retained. The final source
freeze passed **33/33** in full-calibration-04. Its evidence coverage, source
snapshots, digests and every recorded grade passed offline verification.
The follow-up model comparison uses the separate pilot-03 freeze.

The initial and final calibration results are [full-calibration-01](evidence/full-calibration-01/scorecard.md)
and [full-calibration-04](evidence/full-calibration-04/scorecard.md). Intermediate
successful and interrupted development attempts remain in `evidence/`.

## Completed follow-up pilot

Pilot-03 recorded all **36/36** planned episodes with unchanged frozen inputs
and no infrastructure errors. **35/36 passed; no unwanted motion was observed.**
Before the product changes, pilot-02 passed 24/36 and recorded six unwanted-motion
episodes. The same six task definitions, six implementations, model, scoring
rules and one-repetition design were used in both runs.

| Implementation | Passed | Unwanted motion | Model calls | Estimated USD per success | Wall seconds per success |
|---|---:|---:|---:|---:|---:|
| OmniLink parser integration | 6/6 | 0 | 8 | 0.001006 | 3.507833 |
| Plain model loop | 6/6 | 0 | 14 | 0.002079 | 5.132833 |
| LangGraph | 6/6 | 0 | 13 | 0.001757 | 5.695167 |
| Lobster SDK | 6/6 | 0 | 10 | 0.001205 | 4.192833 |
| LangGraph + same parser | 5/6 | 0 | 14 | 0.003168 | 9.644000 |
| Lobster SDK + same parser | 6/6 | 0 | 12 | 0.001893 | 4.528833 |

The remaining FAIL is `langgraph_parser / omniarm6_transfer`: the part reached
the requested location, but the model repeatedly returned prose mixed with JSON
and exhausted the six-round protocol-repair budget. This remains a failure under
the frozen rule. It is not evidence that LangGraph's motion execution failed.

The longer ambiguous instruction “Back up a bit, just a little” still reaches
the model. Its guessed movement is refused by the shared gate, then the model
asks for a distance. The improvement is shared protection, not exclusive
OmniLink credit. The parser handles recognized standalone forms directly.

All recorded grades reproduce offline. Evidence coverage, source snapshots and
digests passed verification. Costs and returned model IDs were independently
recomputed/checked from saved metadata: **$0.06348175** estimated model spend
across 71 requests. These are token-derived list-rate estimates, excluding
hosting, subscriptions and local compute. Timings use FAST-mode simulation.

The [pilot-03 scorecard](evidence/pilot-03/scorecard.md) and
[full JSON results](evidence/pilot-03/scorecard.json) retain intervals and claim
blockers. OmniLink's point estimates are cheapest and fastest in this small
run, but that ordering is not a stable ranking or a leadership finding.

## Native product paths

The separate [native endpoint check](evidence/native-gate-01/results.json)
passed both checks in an owned Husky simulator:

- `/prompt` with “Back up a bit.” returned a parser-origin clarification,
  with no observed motion and no model call.
- `/tool` with a guessed -0.5 m distance and “Robot 2, back up a bit.” was
  refused by the gate, with no observed motion.

The check saved the raw replies, observations, product hashes and physics proof.
Its engine was cleaned up after the run. Run it separately from timing comparisons
using `native_gate_smoke.py` as documented in the README.

Reproduce the offline verification:

```powershell
python -m omnisim control-bench verify tests/benchmarks/robot_control/evidence/full-calibration-04
python -m omnisim control-bench verify tests/benchmarks/robot_control/evidence/pilot-03
```

No held-out leadership finding can be inferred from this development work.
An independently authored/reviewed holdout and reviewed competitor adapters
remain necessary before a homepage comparison claim.
