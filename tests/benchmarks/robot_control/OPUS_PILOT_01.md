# Opus 5.5 low-effort pilot 01

Declared on 2026-09-24 before any paid Anthropic generation. The owner's total
authorization for this pilot is $1 in standard model token charges. No automatic
full campaign, no retries on infrastructure failures, no model fallback.

## Question and selection

Does changing the fallback model in the existing OmniLink parser integration
improve any of the measured robotics-control behaviors? Run one repeat of nine
existing development tasks, chosen before Opus outcomes:

- husky_square: sequential motion
- husky_condition_false: measured-state branch
- husky_transient: recovery without repeating a successful motion
- husky_persistent: report failure after two unsuccessful attempts
- husky_conversation: multi-turn state and relative commands
- tb3_burger_rotated: different mobile robot, non-origin pose
- omniarm6_sequence: joint trajectory and home
- omniarm6_unspecified: known Gemini failure under the suite's no-guessing contract
- omniarm6_transfer: multi-turn assisted manipulation

Use only `omnilink_parser`. Keep task text, product, runtime, mechanical oracle,
six-round/24-action limits and 90-second dispatch deadline unchanged. Fresh
owned engine per task, fixed seed 240926, all outcomes retained. This is a
diagnostic selection that deliberately includes a known failure, NOT a holdout
or an estimate of general robot-task accuracy. The ambiguous arm sentence
delegates a choice in ordinary language, but this suite's system contract
explicitly disallows guessing unspecified angles; score that contract as frozen.

Compare to the exact same nine task IDs in publication-04's saved
`omnilink_parser` Gemini 3.5 Flash rows. No extra Gemini spend. Disclose that
these are historical, one-repeat observations and different model transports,
not simultaneous matched provider settings or a causal latency comparison.
The hosted Gemini connector builds its own system prompt; the direct Anthropic
pilot uses the common benchmark SYSTEM and tools without that hosted wrapper.
Do not label any difference a proven platform or pure-model advantage.

## Direct transport and spending

Use `claude-opus-5-5`, `output_config.effort=low`, `max_tokens=2048` including
thinking, `service_tier=standard_only`. No temperature, paid server tools,
premium speed mode, caching instructions or explicit thinking budget.
This new pilot adapter does not deploy or change the existing website's Claude
connector. Robot bridges still require a separate OmniKey.

At most 24 generation requests. Count input tokens with Anthropic's free count
endpoint first. Before each generation, reserve input cost for the larger of
twice the estimated count and the serialized request's UTF-8 byte length, plus
4096 tokens, and reserve the full 2048 output tokens. Send only if known spend
plus that reservation fits within $1. This deliberately conservative bound
assumes the documented tokenizer/pricing and provider-enforced output cap;
it is not a billing-account limit. The existing runner also leaves a $0.10
margin before starting a new episode. Stop on unknown usage, model mismatch,
transport errors or exhausted budget. No retry after uncertain billing.
Write a durable reservation journal before each paid request; retain actual
reported usage and all invalid or truncated answers. A crash does not authorize
restarting with a fresh budget. Cost is list-price model usage, not an invoice.

Rates per million tokens: input $4, output $20, cache read $0.20. No cache writes
are requested; unexpected writes halt accounting rather than guessing the TTL.
The Models API lookup confirmed access at no generation charge.

Official sources checked 2026-09-24:
- https://platform.claude.com/docs/en/models/opus-5-5/overview
- https://platform.claude.com/docs/en/build-with-claude/effort
- https://platform.claude.com/docs/en/build-with-claude/token-counting

## Reproduction

Generate `suites/opus-pilot-01.json` as the exact ordered subset above from
`suites/development.json`. Freeze before running:

```powershell
python -m omnisim control-bench freeze --suite tests/benchmarks/robot_control/suites/opus-pilot-01.json --lock tests/benchmarks/robot_control/locks/opus-pilot-01.json --arms omnilink_parser --repeats 1 --cap-usd 1 --anthropic-pilot
python -m omnisim control-bench run --lock tests/benchmarks/robot_control/locks/opus-pilot-01.json --out tests/benchmarks/robot_control/evidence/opus-pilot-01 --key-file <private-omnikey-file> --anthropic-key-file <private-anthropic-key-file>
python -m omnisim control-bench verify tests/benchmarks/robot_control/evidence/opus-pilot-01
```

Preserve the former Gemini publication artifacts unchanged. Never add pilot
scores to homepage leadership claims. A useful outcome here informs the next
matched experiment; ties, regressions and higher costs matter equally.
