# The OmniLink development loop

Owner decision, 2026-10-01: improving OmniLink is a development cycle. A
full blind campaign (seven arms, five repeats, four to five hours, up to
$95) is how a result is **published**, and only once OmniLink is clearly
ahead. Running one to find out what to fix next is waste: the number is
obsolete as soon as the next fix lands.

## The four tiers

| Tier | What runs | Wall time | Cost | When |
|---|---|---|---|---|
| 0 unit | Engine-free tests of the parser, bridge and relay; every failure class found in a trace gets one | seconds | $0 | every change |
| 1 probes | `suites/probes_v1.json`: one short scenario per failure class, OmniLink only (competitors once, as a reference) | minutes | about $1 | every fix |
| 2 seen shifts | the three seen 30-minute shifts (dev v1, holdout v1, holdout v2), OmniLink only, all at once, against stored competitor baselines | about 35 min | about $3-4 | a few times a day |
| 3 blind campaign | a fresh, sealed blind shift under a preregistration, all seven arms | 4-5 h | up to $95 | only to publish |

```bash
# Tier 0
python -m pytest packages/omnisim-bridges/tests -q

# Tier 1
python tests/benchmarks/robot_ops/devloop/devloop.py probes --key-file O:/omnilink-keys/omni_key.txt

# Tier 2
python tests/benchmarks/robot_ops/devloop/devloop.py run --key-file O:/omnilink-keys/omni_key.txt
python tests/benchmarks/robot_ops/devloop/devloop.py report tests/benchmarks/robot_ops/evidence/devloop/<loop>
```

### The fast path (2026-10-20)

```bash
# Model presets: gemini (default), claude (Opus 5.5), claude-fast (Sonnet 5.5, the inner loop)
python tests/benchmarks/robot_ops/devloop/devloop.py run --preset claude-fast --key-file O:/omnilink-keys/omni_key.txt

# Simulator-time clock: fast mode while OmniLink is idle, real time while it is busy (OmniLink only)
python tests/benchmarks/robot_ops/devloop/devloop.py run --preset claude --sim-clock --key-file O:/omnilink-keys/omni_key.txt

# One short probe per check OmniLink failed in a loop, all run at once (minutes, not a 30-minute shift)
python tests/benchmarks/robot_ops/devloop/devloop.py failure-probes tests/benchmarks/robot_ops/evidence/devloop/<loop> --run --preset claude --sim-clock --key-file O:/omnilink-keys/omni_key.txt

# Re-run a loop answering the model from that loop's recorded replies (no model cost; for non-prompt changes)
python tests/benchmarks/robot_ops/devloop/devloop.py run --preset claude --replay-from tests/benchmarks/robot_ops/evidence/devloop/<loop> --key-file O:/omnilink-keys/omni_key.txt
```

- **`--sim-clock`** sets `OMNILINK_CLOCK=sim` (see `omnisim_bridges/runtime_clock.py`): the
  runtime schedules on simulator time, the engine runs in FAST mode capped at
  `--sim-clock-speed` (default 6x), and drops to real time whenever OmniLink is handling a
  message or waiting on the model. Competitor arms are refused: their model calls run in the
  benchmark process, where the runtime cannot slow the world for them.
- **`failure-probes`** keeps each failed check, the steps it watches, the shift's opening and a
  short lead-in, and squeezes chat-only gaps. The robot starts at home and everything else is
  dropped, so a probe is a fast reproducer: confirm every fix on the full shift.
- **`--replay-from`**: every OmniLink episode records its model replies
  (`model_replies.jsonl`). Replay matches on the conversation with numbers rounded and the
  per-turn liveContext left out, so a reply can come from a slightly different world.
- **`confirm`** now runs the shifts of a wave side by side (`--max-engines`, default 9); the
  judge has its own `--judge-engine` (default g1-engine) so a Claude run is judged by Gemini.

Measured 2026-10-20 (Claude Opus 5.5, each job alone on the laptop): holdout_v2 on the sim clock
19.3 min vs 32.5 min in real time (score 0.821 vs 0.769 in the paired real-time run; one repeat
each). The shift is busy most of the time, and busy time runs at real time, so ~1.7x is the
ceiling for a full shift. Eight failure probes of holdout_v1, four at a time: 6.6 min (they had
run one after another, 19.3 min). A fix-and-check cycle on probes is therefore ~7 minutes.

None of this changes Tier 3: a published result runs in real time, live model, all arms.

Every run lands in `evidence/devloop/`. The Tier 2 report gives, per shift,
OmniLink's score and unsafe count, the best stored competitor, the gap, and
the check ids OmniLink fails that that competitor passes: the next thing to
fix.

## Why the shifts are not cut into segments

It was the first idea, and it was measured before being built. On all three
seen shifts the checks' windows chain across the whole shift ("reach X any
time after this order", rules in force to the end, an order timed eleven
minutes out, "is that job still on?"), so no shift splits into independent
pieces without dropping or rewriting the checks that make it a shift. The
speed comes instead from running OmniLink alone, every shift at once, and
from the short probes.

## Stored baselines (`baselines.json`)

Competitor scores per seen shift, built with `devloop.py baseline <shift>
<evidence dirs>` and tagged with their source. The competitors share code
with OmniLink (the bridge, the parser their interruption classifier calls,
the gate), so each baseline carries the hash of that code (`devloop.py
hash`). The report marks a baseline **STALE** when that code has changed
since, and **hash unknown** for the baselines imported from the v1 and v2
campaigns, which ran before the 2026-09-30 fixes. Neither counts towards
READY: refresh the full tier with a fresh run first.

## When to run Tier 3

`READY FOR A BLIND CAMPAIGN: YES` needs, on every seen shift: a gap of at
least 0.15 over the best competitor, zero unsafe checks, current baselines
and at least three OmniLink repeats. Then the sealed holdout
(`suites/shift_holdout_v3.json`, written blind on 2026-10-01 and never read)
gets calibrated, frozen and run under `shift/PREREGISTRATION_V3.md`.

## Honesty

Tiers 0-2 use material the developer has seen and is improving against.
Their numbers can reward overfitting, and they are never published. The v3
holdout is deliberately absent from `devloop.py` and must stay so.
