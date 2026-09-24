# Cached Opus comparison 01 — prepared, not started

Prepared at the owner's request after stopping the uncached comparison.
No paid cached requests have been sent. A freeze is a reviewable plan, not
authorization to restart spending. This proposed new campaign has its own
$5 maximum including cache warm-up. Starting it requires the owner's next
instruction; the stopped campaign's expenditure is not reset or hidden.

## Design

Use the same 33 development tasks, all six integrations, one repeat, fixed
shuffle seed 240926, Opus 5.5 low effort, maximum 2048 output tokens including
thinking, standard service tier, no retries after unknown billing. Robots,
controllers, gate, oracle, messages and budgets for actions/rounds remain the
same. No product fixes are folded into the caching change. Developer-authored,
previously seen tasks and controlled integrations do not support market-wide
leadership claims. Do not pool this run with the interrupted uncached run.

## Caching and fairness

Each arm receives exactly the same caching policy: an explicit five-minute
breakpoint after the common system instruction and tool schema, plus automatic
five-minute caching of growing message history. Only request metadata and
system-block representation change; the system text does not.

Before any scored episode, perform one common-prefix warm-up for each of the
two tool surfaces (arm and mobile), with max_tokens=0 and no top-level history
cache breakpoint. These two calls still incur input/cache charges: record them
in provider-setup.json and the durable provider journal and count them against
the $5/450-request limits. Confirm zero output tokens and max_tokens stop reason.
Require a nonzero cache-write or cache-read counter for each shared prefix;
otherwise stop before scored requests. Offline transport/accounting tests pass;
actual provider cache hits have not yet been measured for this configuration.

Report warm-up cost separately as shared benchmark setup, never charge it only
to the first scored implementation. Per-arm scored-call costs exclude that
shared setup; the all-in campaign total includes it. Also show a per-arm
sensitivity allocation of one-sixth of the shared setup cost. This does not
claim isolated production cache behavior: all calls use one workspace, shared
static prefixes and shuffled order. Cache expiry, exact matching and shared
prefix reuse can influence actual cache hits. Report real usage counters for
every arm; enabling cache_control alone is not proof of a hit. Cold and warm
cost conditions must not be silently mixed in homepage comparisons.

## Accounting and stop rules

List rates per million tokens: ordinary input $4, output $20, five-minute
cache writes $5, reads $0.20. Sum all three separate input categories; do not
treat input_tokens as including cached tokens. Confirm the five-minute write
breakdown and reject nonzero one-hour writes or inconsistent counters.

Before each paid call, reserve input at the higher $5 write rate, using the
larger of serialized UTF-8 byte length and twice the count-API estimate plus
4096 tokens, and reserve the full output allowance. Warm-ups use zero output
allowance but the same conservative input rule. No paid request is sent unless
known spending plus its reservation fits. Stop on unknown usage, changed model,
unexpected output, infrastructure failure or exhausted budget. A journaled
request interrupted before receiving its response keeps an unresolved charge;
never count it as zero. The $5 cap is not a provider invoice guarantee.

## Reproduction

Offline freeze (no provider calls):

```powershell
python -m omnisim control-bench freeze --suite tests/benchmarks/robot_control/suites/development.json --lock tests/benchmarks/robot_control/locks/opus-cached-comparison-01.json --repeats 1 --cap-usd 5 --anthropic-cached-comparison
```

After a separate instruction to start, use private key files:

```powershell
python -m omnisim control-bench run --lock tests/benchmarks/robot_control/locks/opus-cached-comparison-01.json --out tests/benchmarks/robot_control/evidence/opus-cached-comparison-01 --key-file <omnikey-file> --anthropic-key-file <anthropic-key-file>
python -m omnisim control-bench verify tests/benchmarks/robot_control/evidence/opus-cached-comparison-01
python tests/benchmarks/robot_control/audit_anthropic_campaign.py tests/benchmarks/robot_control/evidence/opus-cached-comparison-01
```

Preserve the first two responses and all scored responses, source snapshots,
request reservations, model identity, token counters, mechanical observations,
physics sidecars, truncations, failures and errors. Incomplete coverage forbids
ranking unequal task sets. Existing family-bootstrap rules remain unchanged.

Official caching documentation checked 2026-09-24:
https://platform.claude.com/docs/en/build-with-claude/prompt-caching
