# FREEZE AMENDMENT 01 — holdout v1, run 2

Recorded 2026-09-25, before run 2's first scored episode. It amends
[FREEZE.md](FREEZE.md). Everything not named here is unchanged: the
holdout and its SHA-256, the 7 arms, 2 repeats, the analysis, the claims,
the ERROR re-run rule and the interruption rule.

## Why there is an amendment

Run 1 was abandoned after 37 episodes because the provider was congested,
not the agents. The evidence, and the partial outcomes known when the
owner stopped it, are in
[evidence/holdout-v1-run1-abandoned/ABANDONED.md](../evidence/holdout-v1-run1-abandoned/ABANDONED.md).
Run 2 starts from scratch, and nothing from run 1 is pooled with it.

## What changes

| Item | FREEZE.md | Run 2 |
|---|---|---|
| Model | `gemini-3.5-flash` | `gemini-3.5-flash` |
| Rates (USD per million tokens) | 0.75 / 0.075 / 3.75 | 1.50 / 0.15 / 9.00 (input / cached / output including thinking; ai.google.dev/gemini-api/docs/pricing, read 2026-09-25) |
| Spend cap | $40 | $40, now also counting the health probes |
| Provider-health gate | none | before every task block (see below) |
| Output directory | `evidence/holdout-v1` | `evidence/holdout-v1-run2` (re-runs in `holdout-v1-run2-reruns`, continuations in `holdout-v1-run2-cont-<n>`) |
| Code commit | `4707517fb` | the commit that adds this file |

The FREEZE.md row for run 1's model read `gemini-3.8-flash`. The owner chose
`gemini-3.5-flash` for run 2 over three alternatives, each measured at 19:36
UTC:
- `gemini-3.8-flash`, which was congested;
- `gemini-2.5-flash`, which was faster but older and weaker, and so would
  favour OmniLink's parser over model-driven competitors;
- `gemini-3-flash-preview`, a preview model that could change mid-run.

`gemini-3.5-flash` is also the relay's own default model.

## The provider-health gate

Before every task block (all 7 arms of one task in one repeat), the runner
sends 5 short fixed requests through the same transport the arms use
(`omnisim/ops_bench/health.py`).

- **Healthy** means all 5 answered and their median latency is at most
  **8 s**. The block runs.
- **Unhealthy:** the runner re-probes every 10 minutes. If the provider
  stays unhealthy for 12 hours, the run stops as if interrupted.

The gate reads nothing from any arm, and it is applied before whole blocks,
so every arm of a task meets the same provider window. Every probe is kept
in `health.jsonl` with its latency, errors, returned model and cost.

## Code changes since FREEZE.md

All five change measurement or transport, not behaviour.

1. **OmniLink's model usage is recorded per round.** In run 1 the relay
   logged usage only when a whole turn ended, so a turn still waiting on the
   model when the episode ended counted as $0. The grounding re-ask was in
   no turn log at all. The relay now writes a `round_start` line and a
   `round` line with usage around every model call. A round that never
   completes is charged at the worst observed request, the preregistered
   rule for unknown usage.
2. **Retry hints are honoured the same way on both sides.** When Google
   rate-limits it, the platform cools the credential for about 10 s and
   answers each request in that window with 429 plus `Retry-After`. The
   relay's and the competitors' fixed 1.5 s and 3 s backoffs spent both
   retries inside that window. So one burst of upstream 429s became a
   failed reply, for whichever arm happened to be running. Now both wait
   the server's hint, at least the backoff and at most 15 s: the relay via
   `relay.retry_wait_s`, the competitors via `ModelClient` with
   `HINT_CAP_S`. A test pins the two rules together.
3. **The relay records which model answered each round**, for the fallback disclosure below.
4. **A refused request costs nothing.** A competitor request answered with a
   429 ran no model and is charged $0. It used to count as unknown usage,
   charged at the worst request. The relay's 429s are retries inside one
   traced round and were already free, so the old rule tilted cost against
   the competitors.
5. **Run manifests fingerprint more sources.** They now also record the
   adapters (`competitors.py`), accounting and the health gate.

## Disclosure corrected

FREEZE.md said engine fallback was impossible because only Google is
configured. That was wrong. In run 1, after each of three upstream timeouts,
the platform tried its g2 engine, and that attempt also failed.
- **Competitor calls:** every request records the model that answered
  (`competitor_requests[].model_returned`). In run 1, every answer came from
  the frozen model.
- **OmniLink's relay:** its trace had no such field, so an answer from a
  fallback engine could not be told apart there. It now records
  `model_returned` for every round (`relay_usage.models_returned`).

Run 2 reports any answer, from either side, that came from a model other
than `gemini-3.5-flash`.

## The command

```bash
tests/benchmarks/harness_comparison/.venv/Scripts/python.exe -m omnisim ops-bench run \
  --suite tests/benchmarks/robot_ops/suites/holdout_v1.json \
  --arms omnilink plain_basic langgraph_basic lobster_basic plain_full langgraph_full lobster_full \
  --repeats 2 --engine g1-engine --model gemini-3.5-flash \
  --cap-usd 40 --rates 1.5,0.15,9.0 --health-median-s 8 --health-max-wait-h 12 \
  --out tests/benchmarks/robot_ops/evidence/holdout-v1-run2 \
  --key-file <the OmniKey file, never recorded>
```

## Live check before the freeze

One development episode per side (`keepout_after_chatter_husky`, `omnilink`
and `plain_full`) was run on this code in `evidence/amend-smoke-01`, to show
that the health gate, the round trace and the costing work end to end. It
used the development suite only.
