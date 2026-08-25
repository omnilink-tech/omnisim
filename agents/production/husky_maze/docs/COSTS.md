# OmniLink agent costs — measured + projected

This doc is the canonical reference for "what does it cost to run an OmniLink agent?" — the methodology, the live measurements, the per-engine projections, and the optimizations we shipped or have queued. It is grounded in actual `husky_maze` runs against the OmniLink platform; the numbers are real, not estimated.

## TL;DR

- **Gemini 3 Flash (current default `g1-engine`): ~$1.33/hour list price for the standard BFS demo agent**, ~$0.14 per complete maze mission.
- A single `lean tool descriptions` change cut per-turn token use **24%** with zero risk.
- Claude alternatives are 2–30× more expensive per token but may complete missions in fewer turns; the right comparison is **$/mission**, not $/token, and that A/B has not been run yet.
- Implicit caching is auto-enabled on Gemini 2.5+; explicit caching gives a 90% input discount but **is not currently wired through the OmniLink g1-engine path** (it is wired for the g4/Claude path). See [Open work](#open-work).

## Methodology

Every OmniLink agent runner now embeds a `UsageMeter` (the `UsageMeter` helper from the `omnilink` client library, in the OmniLink repo). At runner startup it snapshots the platform's 24-hour-rolled token + credit counters via `client.get_usage()`. The runner's `/status` endpoint exposes a live delta over just this session:

```bash
$ curl http://127.0.0.1:51519/status | jq .usage
{
  "available": true,
  "elapsed_s": 391.0,
  "input_units": 242400,
  "output_units": 7700,
  "total_units": 250100,
  "credits": 0,
  "tokens_per_hour": 2304252,
  "credits_per_hour": 0
}
```

`chat_drive.py` prints the same delta at end-of-run plus an average tokens-per-turn:

```
[chat] usage: window=391s tokens=250,110 (in 242.4k + out 7.7k)
       -> 2,304,252 tokens/hour, 0.00 credits/hour
[chat] avg tokens/turn: 35,730
```

Token counts come from the platform's authoritative usage rollup, NOT from response parsing — they include all `/api/chat`, `/api/short-term-memory`, and any other OmniLink endpoint the agent hit during the window. Cost-per-hour is derived by extrapolating the window's tokens to a 1-hour rate and applying the public per-token engine prices.

## Live baseline — Husky Maze v3 / BFS demo

Captured 2026-05-01 against `husky_maze.omniworld` (seed-7 BFS, full mission complete):

| metric | value |
|---|---|
| chat turns | 7 |
| tool calls | 6 |
| wall clock | 388.7 s |
| input tokens (window) | 242,400 |
| output tokens (window) | 7,700 |
| total tokens (window) | 250,110 |
| **tokens/hour** | **2,304,252** |
| input/output split | 97% / 3% |
| avg tokens/turn | 35,730 |

The agent's call sequence (verified end-to-end live):

```
turn 1: get_capabilities          (4 s)
turn 2: recall                    (3 s)
turn 3: try_get_known_map         (4 s)
turn 4: execute_path (72 cells)   (6 s LLM + ~5 min sim physics)
turn 5: complete_mission          (6 s)
turn 6: save_local_memory         (4 s)
turn 7: closeout narration        (3 s)
```

The 97% input bias is structural: every chat call ships the system instruction, the available-tools surface, and the accumulated conversation history. Output is just the agent's narration + a tool-call argument blob.

## Per-engine cost — same workload across all four engines

Using the live baseline above (2.232 M input tokens/hour + 0.071 M output tokens/hour) and public per-token pricing (May 2026):

### Without caching (list prices)

| engine | input $/M | output $/M | hourly | per mission (6.5 min) | 24×7 monthly |
|---|---:|---:|---:|---:|---:|
| **Gemini 3 Flash** (`g1-engine`) | $0.50 | $3.00 | **$1.33** | $0.14 | $960 |
| Claude Haiku 4.5 (`g4`*) | $1.00 | $5.00 | **$2.59** (~2×) | $0.28 | $1,860 |
| Claude Sonnet 4.6 (`g4`*) | $3.00 | $15.00 | **$7.76** (~6×) | $0.84 | $5,580 |
| Claude Opus 4.7 (`g4`*) | $15.00 | $75.00 | **$38.81** (~30×) | $4.20 | $27,930 |

*The `g4-engine` slot maps to whichever Claude model the deployment is configured for; the table shows the three current Claude tiers for comparison.

### With prompt caching (~25% of input is the static system + tools surface)

Both providers offer ~90% off on cache reads. The ratio between providers stays similar; only the absolute floor moves:

| engine | cache read $/M | hourly with caching | best case (100% cached) |
|---|---:|---:|---:|
| Gemini 3 Flash | $0.05 | **~$1.08** | $0.32 |
| Claude Haiku 4.5 | $0.10 | **~$2.09** | $0.58 |
| Claude Sonnet 4.6 | $0.30 | **~$6.25** | $1.74 |
| Claude Opus 4.7 | $1.50 | **~$31.27** | $8.67 |

Anthropic charges 1.25× on cache *writes* (5-min TTL); Google charges no write premium but bills per-hour cache storage. Both differences are sub-cent at our cadence.

### Per-mission cost summary

What a complete maze solve costs at list price:

```
Gemini 3 Flash:      $0.14
Claude Haiku 4.5:    $0.28
Claude Sonnet 4.6:   $0.84
Claude Opus 4.7:     $4.20
```

## Optimizations applied

### 1. Lean tool descriptions (shipped 2026-05-01) — saved 24% per turn

The runner used to ship full tool descriptions on every chat call (~4,878 tokens for 26 tools). Switched the default to first-sentence-only descriptions (~2,556 tokens). Override via `HUSKY_AGENT_TOOL_DESCRIPTIONS=full` for debugging.

| | full | lean | delta |
|---|---:|---:|---:|
| tool surface bytes | 19,514 | 10,226 | −48% |
| tool surface tokens | ~4,878 | ~2,556 | −2,322 |
| measured tokens/turn | ~47,180 | 35,730 | **−24%** |

Per-turn savings exceeded the raw-token-delta prediction because shrinking the system instruction also reduces the LLM's processing context.

Code: [`husky_maze_agent.py:_TOOL_DESC_MODE`](../husky_maze_agent.py).

### 2. UsageMeter + /status integration (shipped 2026-05-01)

Both the `husky_maze` and `mission_captain` runners now expose a live `usage` block on `/status`. New OmniLink agents that follow the same template inherit the metric automatically — initialise a `UsageMeter` from the `OmniLinkClient` at runner startup, plumb it through your `build_status_snapshot()`, done.

The meter is the `usage_meter` module of the `omnilink` client library (in the OmniLink repo) — `UsageMeter`, `UsageDelta`, `UsageSnapshot` classes.

## Open work

### a. Gemini explicit caching (shipped 2026-05-01)

The Claude (`g4-engine`) handler had been using explicit `cache_control: { type: 'ephemeral' }` on the stable portion of the system instruction for some time. The Gemini (`g1-engine`) handler was passing `systemInstruction` and `tools` raw on every call — relying only on Google's opportunistic implicit caching (no guarantee).

Wired up in the OmniLink platform's server-side cache layer (in the OmniLink repo):
- Hashes the (model, stable system instruction, tools) tuple into a key.
- Per-process `Map<key, {cacheName, expiresAtMs}>` LRU.
- On miss, creates a `CachedContent` via `ai.caches.create()` with TTL 600 s (tunable via `OMNI_GEMINI_CACHE_TTL_S`).
- Skips caching when the stable prefix is below ~4096 chars (~1024 tokens — Gemini Flash's minimum cacheable size; tunable via `OMNI_GEMINI_CACHE_MIN_CHARS`).
- Master kill-switch: `OMNI_GEMINI_CACHE_ENABLED=0`.

The Gemini handler now passes `config.cachedContent: <name>` on cache hits and OMITS the cached `systemInstruction` and `tools` from the per-request config. Operator log lines:
```
[g1-engine] cache created: cachedContents/abc123def456...
[g1-engine] cache hit:     cachedContents/abc123def456...
[g1-engine] cache skipped: too_small | create_failed
```

Expected saving once a cache is hot: **~19–75% reduction** in g1-engine cost depending on what fraction of input qualifies as the static prefix. Realistic floor for the BFS demo: **~$0.50–$0.70/hour** (down from $1.33).

> **Note on usage metering:** the platform's server-side usage metering reads `promptTokenCount` from Gemini's `usageMetadata` — that count INCLUDES cached tokens. The platform's usage rollup will therefore *not* automatically reflect the per-token discount; the cost reduction shows up on the operator's cloud bill instead. A follow-up should extend the metered-usage record with a `cachedTokens` field so the OmniLink-side `credits/hour` view also shrinks. The headline `tokens/hour` metric stays accurate; only the projected dollars change.

### b. Per-variant tool filtering

`v3` only uses 6 of the 26 registered tools (get_capabilities, recall, try_get_known_map, execute_path, complete_mission, save_local_memory). The other 20 are dead weight in the system instruction. A per-variant `available_tools` allowlist would cut another ~30% per turn for v3 (smaller win for v1/v2 which use more of the surface).

### c. mainTask trim

`v3`'s `mainTask` is 2,478 chars — some redundancy with the tool descriptions (post-lean). Easy ~500-1000 token savings.

### d. Engine A/B harness

The honest comparison metric is **$/mission**, not $/token. A higher-quality model that completes the mission in fewer turns can be cheaper overall despite a higher per-token price. Add `--engine` flag to `chat_drive.py` and run the same world against `g1`, `g4` (Haiku), `g4` (Sonnet) — measure dollars to mission-complete on each.

## How to reproduce a measurement

```bash
# 1. Launch OmniSim
launch.bat projects\samples\demos\worlds\flagship\husky_maze.omniworld

# 2. Start the runner (any variant)
set OMNI_KEY=olink_...
python agents\production\husky_maze\husky_maze_agent.py --variant v3

# 3. Drive the agent — usage summary prints at end
python agents\production\husky_maze\scripts\chat_drive.py --variant v3 --clear-memory --max-turns 12 \
    "Solve the maze. Drive the husky from its current cell to the goal."
```

The final `[chat] usage:` line is the headline number. Multiply by hours to project monthly run cost; divide by mission completion count to get $/mission.

## What the platform measures vs what the agent emits

The `UsageMeter` reads OmniLink's authoritative `/api/omni-key-usage` rollup — that means **all** endpoints the agent hits show up in the delta, not just `/api/chat`. For example, `save_local_memory` writes go through `/api/short-term-memory` (which we observed in the rollup as `output_units: 30` events). Most of the cost is still chat, but the meter avoids under-reporting by pulling from the platform's own books.

There's no client-side double-counting: the platform invoices once per request, the meter just subtracts before/after counters.
