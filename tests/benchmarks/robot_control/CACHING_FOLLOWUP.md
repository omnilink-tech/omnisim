# Prompt caching follow-up (proposal, no paid cached requests sent)

The owner asked about caching during opus-comparison-01. The frozen run has
no cache_control and returned zero cache-read/write tokens at 88 episodes,
159 paid calls, $3.001796. The question does not retroactively change that
protocol. Preserve its results and classify its costs as an uncached baseline,
not an optimized cost frontier. The agent acknowledges that enabling caching
before freezing would have better served the request to conserve money.

## Proposed next configuration

All six implementations get the same explicit five-minute cache breakpoint
after the common system instruction/tool schema, plus top-level automatic
five-minute caching for growing conversation history. Preserve prompt text,
model, low effort, output cap, tools and grading. Count actual cache hits;
configuration alone is not proof that requests hit the cache.

Official documentation checked 2026-09-24 says Opus 5.5 accepts a minimum
512-token cache prefix. Five-minute writes cost $5/MTok, reads $0.20/MTok,
ordinary input $4/MTok, output $20/MTok. Input, cache-read and cache-write counts
are separate, so all three must be included. Cache storage is workspace-scoped.
Source: https://platform.claude.com/docs/en/build-with-claude/prompt-caching

## Required implementation and evidence changes before running

- The existing adapter deliberately rejects cache writes. Add a separately
  versioned cached configuration and account for confirmed five-minute writes;
  reject unexpected one-hour writes or inconsistent usage rather than guess.
- Reserve every possible input token at the higher $5 write rate, plus the full
  output allowance. Keep the durable journal, unknown-usage halt, no retry and
  an explicitly frozen total spend cap that includes warm-up calls.
- Test billing arithmetic, unknown usage, cold write, partial hit, full hit,
  mixed input, invalid TTL, budget reservation and no credential persistence.
- Shared workspace cache warmth can depend on which implementation goes first.
  Predeclare a common system-prefix warm-up, retain its actual cost separately,
  and specify its allocation before comparing per-implementation costs.
  Report cache creation/read counts and both warm-up and scored spending.
  Do not silently pool cold and warm runs or charge first-write overhead only
  to whichever implementation happens to run first.
- Freeze a new protocol. Keep the uncached evidence intact. No inferred cache
  savings from the old run may be presented as measured savings.

The current uncached production-comparison artifact is not a cached benchmark,
and the original Gemini/Opus pilot is not directly comparable to the proposed
cached campaign's cost conditions. This proposal does not authorize a restart
or additional spending beyond the current $5 comparison ceiling.
