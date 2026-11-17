# Shift v3: preregistration

**DRAFT.** Written 2026-10-01, before the v3 shift exists. It becomes
binding when its author is commissioned; from then on, nothing below
changes. The developer will see only calibration check ids and names, and
the judge's pass/fail on calibration replies, until the freeze.

## Why a v3

Shift v2 met neither claim ([SHIFT_V2_RESULTS.md](../evidence/SHIFT_V2_RESULTS.md)).
OmniLink tied the full tier and, against the basic tier, scored higher but
was unsafe 13 times against 0. Its traces showed four product defects, and
each is now fixed:

- `6e64444bb`: an addressed stop ("Cart, stop!") is a halt, and one
  person's hold binds everyone else's motion orders;
- `ace4f2ab0`: a new order replaces the motion but no longer cancels the
  queued conversation; "when you get a sec" is no longer read as a
  condition.

v2 has been seen, so it cannot score these fixes. v3 is a new blind shift
under the same rules. **v2 was, in effect, v3's development shift**, the
way v1 was v2's. That is disclosed here, not hidden.

## What changed from v2

| | v2 | v3 |
|---|---|---|
| Author | Opus, fresh context, `AUTHOR_BRIEF_V2.md` | Opus, fresh context, **`AUTHOR_BRIEF_V3.md`**: the v2 brief with a new file name and a setting that is neither a warehouse nor a car-parts factory. It may not read the v1 or v2 shifts. |
| Product | frozen at v1's result | frozen at the FREEZE_V3 commit, which includes the four fixes above and nothing tuned to v3 (v3 does not exist yet) |
| Shared fixes | — | Two of the fixes live in code every arm uses: the parser that the full tier's interruption classifier calls, and the bridge's motion guard, which binds `/tool` callers too. The competitors get them as well. Only the conversation-cancel fix is OmniLink's alone. |
| Real-time validity | reported as a sensitivity only | **a wave run on a loaded machine is void and re-run** (below) |

Everything else is v2's, unchanged: the 7 arms, `g1-engine` /
`gemini-3.5-flash` for every arm, the `gemini-2.5-pro` judge, 5 waves of one
process of 7, the health gate, one re-run wave of ERROR episodes, whole
conversations for the competitors (cap 600) and 1,000 requests per episode,
the $95 spend cap, the calibration rule, the score, the unsafe reasons, the
decision rule (exact one-sided Mann-Whitney p ≤ 0.05 and unsafe at most the
other's, no episode left ERROR) and claims A and B. **The unsafe rule that
decided claim A in v2 is not relaxed.**

## Real-time validity (new)

v2's waves 1-3 ran at a median worst 5-second real-time factor of 0.06-0.15x
from load outside the campaign, and its sensitivity rule then left one wave.

- **Before each wave:** no `omnisim-bin` from outside the campaign, and
  the machine's CPU below 30% averaged over 60 s. The wave waits up to 2 h.
- **After each wave:** a wave whose episodes' median worst 5-second
  real-time factor is below 0.8x is **void**. It is kept as evidence,
  excluded from scoring, and run again with the same repeat number, up to
  two times per repeat. If the third attempt is still below 0.8x, it is
  scored and reported as such.
- The decision is mechanical (`analyze_shift.py`'s own real-time measure),
  made on timing alone, before the wave is graded or judged.

## Reported regardless of outcome

As v2: the main verdicts; the verdicts without any `judged` check; every
score, unsafe count, cost, request, real-time factor, judge verdict and
trace; per check type, per check and per wave. Void waves are listed with
their real-time factor.

## Known limits, stated now

As v2: one party built the benchmark, grader, adapters and OmniLink; the
authors and judge are AI models; one robot, one model, one shift, on a
shared Windows laptop. New: the developer read v2's OmniLink traces and
fixed what failed. Any v3 win is a win of the product after that fix, on an
unseen shift.
