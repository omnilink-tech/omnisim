# Matched Opus robotics-control comparison 01

Declared 2026-09-24 before scored requests. The owner authorized the next
capability test after the nine-task Opus pilot. The agent chooses a conservative
additional $5 model-token ceiling; the previous $0.310024 pilot is separate.
This is not authorization for an automatic restart or increased budget.

## Frozen design

All 33 tasks from development.json, all six existing integrations, one repeat:
198 episodes, randomized with seed 240926. Arms: omnilink_parser, plain,
langgraph, lobster, langgraph_parser, lobster_parser. Model for EVERY arm:
claude-opus-5-5, low effort, 2048 maximum output tokens including thinking,
standard service tier, direct Anthropic transport, no provider retries or
fallback. The common system instruction, tool schema, JSON action protocol,
controller, gate, grader, observation surface, six planning rounds, 24 actions,
90-second dispatch deadline, fixtures and fresh-engine resets are the same
for every arm. Parser controls remain included to isolate parser contributions.

No product fixes are made between the Gemini campaign, Opus pilot and this
comparison. In particular, the known arm unspecified-magnitude gate weakness
remains, so this measures model behavior against the same control contract.
Fixing that defect is separate follow-up work and must produce a new freeze.
Expected paths and grading criteria never enter model prompts.

This suite is developer-authored and previously seen, with 19 task families
and three robots. It is NOT an independent holdout, physical-hardware result,
complete hosted-platform comparison or a market census. Assisted arm grasp
remains assisted. No result, including a sweep, permits "better than everyone".

## Budget and stopping

Same tested reservation guard as the pilot: free input-token counting, reserve
the greater of serialized UTF-8 byte length or twice estimated input tokens,
plus 4096 input tokens and all 2048 output tokens. Send only if known spend plus
this conservative reservation is <= $5. Maximum 450 generation requests.
The runner leaves $0.10 before beginning another episode. Actual usage replaces
the reservation after a known response. No cache writes, paid server tools,
premium speed mode or hidden retries. Rates/MTok: $4 input, $20 output, $0.20
cache reads; no writes requested. Unknown usage or infrastructure failure
halts new spending. No zero-token invention. All truncations and malformed
responses count, including bounded JSON-repair attempts.

The pilot suggests the $5 ceiling may prevent completion. Preserve every
attempt and explicitly report incomplete coverage if that happens; do not
increase the budget, drop costly tasks or restart because of outcomes.
Durable per-request reservations and usage records must reconcile with rows.
The ceiling is enforced against documented token pricing and output limits,
not a provider billing-account limit or invoice guarantee.

## Analysis and reproduction

Regrade raw measured traces, verify model identity and low-effort settings,
recompute usage costs including failed attempts, reconcile the full provider
journal, compare product/fixture hashes against publication-04, and scan for
both keys before any sharing. Bootstrap paired differences by task family
using the existing predeclared 2000-draw analysis. Retain ties and losses.
If incomplete, no rankings from unequal denominators; partial counts only.

Use a new lock and output directory, never overwrite prior evidence:

```powershell
python -m omnisim control-bench freeze --suite tests/benchmarks/robot_control/suites/development.json --lock tests/benchmarks/robot_control/locks/opus-comparison-01.json --repeats 1 --cap-usd 5 --anthropic-comparison
python -m omnisim control-bench run --lock tests/benchmarks/robot_control/locks/opus-comparison-01.json --out tests/benchmarks/robot_control/evidence/opus-comparison-01 --key-file <private-omnikey-file> --anthropic-key-file <private-anthropic-key-file>
python -m omnisim control-bench verify tests/benchmarks/robot_control/evidence/opus-comparison-01
```

Keep the prior website evidence and production model unchanged. This campaign
uses the test adapter, not the deployed Claude chat connector. Historical
Gemini numbers may be descriptive context only: its hosted transport and
system wrapper differ, so this is not a pure Opus-versus-Gemini causal test.
