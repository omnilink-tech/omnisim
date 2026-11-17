# Warehouse shift v1 — cost addendum

Retrospective analysis, September 30, 2026. The frozen score evidence and original
archives are unchanged. The cost chart compares the mean estimated model cost of
the three primary scored shifts for each configuration, including shifts with
failed checks. Infrastructure-error attempts and the entire superseded initial
Codex wave are retained as separate overhead. Development pilots are excluded.

OmniLink: **$0.9033 per scored shift**, the lowest among the original seven
configurations. Its estimated cost was **32–59% below** those six comparisons,
using the same Gemini 3.5 Flash model. **Codex's actual cost is unavailable and it
is excluded from the cost ranking, as is Claude Code's.** Its later GPT-6.1 Sol high-reasoning run
supports a **conditional recorded-token pricing scenario of $0.9126 per shift**,
assuming Standard API rates and the native counters' recorded cache categories.
This establishes neither a cost tie nor an advantage over Codex.

| Configuration | USD / scored shift | Three scored shifts | Other attempt overhead | All attempts |
|---|---:|---:|---:|---:|
| OmniLink | 0.9033 | 2.7099 | 0.0000 | 2.7099 |
| Codex · unranked pricing scenario | 0.9126 | 2.7378 | 2.3726 | 5.1104 |
| Lobster SDK · basic tools | 1.3338 | 4.0013 | 0.9894 | 4.9907 |
| LangGraph · basic tools | 1.3443 | 4.0329 | 1.9033 | 5.9362 |
| Plain model loop · basic tools | 1.4186 | 4.2557 | 0.7558 | 5.0115 |
| Lobster SDK · full tools | 1.7568 | 5.2705 | 3.8040 | 9.0745 |
| Plain model loop · full tools | 2.1830 | 6.5491 | 1.5857 | 8.1348 |
| LangGraph · full tools | 2.1945 | 6.5835 | 2.1632 | 8.7467 |

Codex values are **conditional pricing scenarios, not its signed-in account
charges**. Its original billed-dollar field remains unavailable, not zero. Native
usage counters are best-effort; interrupted calls may lack final usage. Codex and
the original seven used different models, builds and concurrency; this is a
retrospective system comparison. These configurations do not establish a universal
cheapest-agent claim.

Claude Code's native cumulative `modelUsage` reports API list-price estimates
of $1.221749, $1.262099 and $1.240484 for its three scored shifts: $3.724332
total, averaging $1.241444. These are the CLI's own estimates, not a separate
rate-card recalculation or a subscription invoice, and are **unranked**.
Its frozen adapter's partial output snapshots undercount output; reporting uses
the final native cumulative counters instead. The three Claude attempts had no
infrastructure error or rerun. Its development pilot is excluded. See the
[Claude report](CLAUDE_SHIFT_V1_RESULTS.md) and
[published evidence](https://www.omnilink-agents.com/benchmarks/operations-shift/index.html#claude-extension).

Original rates per million tokens: Gemini 3.5 Flash input $1.50, cached input
$0.15, output including thinking $9.00. For Codex we checked the exact
[GPT-6.1 Sol API rate card](https://developers.openai.com/api/docs/models/gpt-6.1-sol)
on September 30, 2026: Standard input $2, cached input $0.10, cache writes $2.50,
output $10. The estimate assumes Standard API processing without Fast/Ultrafast
or regional premiums. Every observed request was below 272,000 input tokens;
no long-context premium was applied. Recorded cache writes were zero; API
cache-write behaviour can differ from this signed-in run. Codex credit billing
has no separate write charge, whereas API cache writes have their own rate;
zero native write tokens do not establish zero API write fees. Historical billing
tier was not preserved in the frozen run metadata; today's configuration cannot
prove past billing. Official OpenAI documentation says API prices do not estimate
included subscription usage.

## Audit of the disputed $0.91 figure

Across the three primary runs, the logs contain 18,785,672 input tokens, of which
18,437,120 (98.1%) are cached, plus 19,700 output tokens including reasoning.
At the stated rates: $0.697104 uncached input + $1.843712 cached input + $0.197
output = $2.737816 / 3 = $0.912605 per shift. The arithmetic is correct under
those assumptions, but cannot verify the real bill. With caching disabled,
pricing the same token counts at Standard rates gives $12.589448 per shift;
that is a sensitivity scenario, not what the logs say happened. Run duration
was about 31 minutes per primary shift; token billing does not charge that idle
or robot-execution time as continuous model generation.

The audit found no decreases or resets in the thread cumulative totals. Three
duplicate usage notifications were correctly excluded by using the final totals.
One primary interrupted turn had no individual usage notification; the logs cannot
determine its billed usage. The full six-attempt recorded-token Standard scenario
is $5.110395, separate from the three-run $2.737816. Development pilots and work
performed by this coding chat are excluded. No API-billed replay was performed.

For each Codex thread, use the last cumulative native totals, not a sum of
repeated notifications. Cached input and cache writes are subsets of input;
reasoning is already included in output. The formula is:

```
USD = ((input - cached - writes) * 2 + cached * 0.10
       + writes * 2.50 + output * 10) / 1_000_000
```

[Cache categories](https://developers.openai.com/api/docs/guides/prompt-caching),
[usage accounting and limitations](https://developers.openai.com/api/docs/guides/agents-api/observability),
[Codex subscriptions versus API billing](https://learn.chatgpt.com/docs/pricing).
Only model usage is included: OmniLink and Codex subscriptions, purchased credits,
hosting, local compute and tax are excluded. OmniLink requires an OmniKey and the
user's own model-provider key; its AI layer is not cost-free.

The machine and workload are those of the score reports: one simulated Husky,
30-minute warehouse shift, 33 scored checks, three primary repeats per system,
Windows 11 / Ryzen 5800H / RTX 3060 Laptop GPU; Newton/MuJoCo physics on CPU,
machine fingerprint `9722d23d12a3`. Simulation only.

## Recompute offline

Download `evidence.zip`, `codex-evidence.zip`, `claude-evidence.zip`, `results.json`,
`codex-results.json`, `claude-results.json`
and `cost-estimates.json` from the [cost section](https://www.omnilink-agents.com/benchmarks/operations-shift/index.html#cost),
put them in one folder, and run the standard-library-only calculator:

```
python -I -S tests/benchmarks/robot_ops/publish/cost_estimates.py <folder> --check
```

The calculator reads all 37 archived attempts, validates the published records,
checks Codex cumulative counters against both raw events and the score report,
checks the observed context threshold, and recomputes the cost graph and overhead
ledger. It also checks Claude's final native cumulative usage and CLI list-price
estimates against the published report, without ranking its unavailable actual cost.
No network, simulator or paid call is needed. Per-attempt usage and archive
hashes are in [SHIFT_V1_COSTS.json](SHIFT_V1_COSTS.json).
