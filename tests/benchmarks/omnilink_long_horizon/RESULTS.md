# Long-Horizon Campaign Results — 2026-07-26

> ⚠️ **The headline "improved" run is `INVALID`, not a 40/100 FAIL.** Of the five
> artifacts, two (`-195352`, `-185635`) were cut mid-episode by an `/api/chat`
> `ConnectionError` and are excluded from every pass rate — the sibling suite's
> rule applies here too ([`omnilink_tasks/README.md`](../omnilink_tasks/README.md):
> "Infrastructure failure is not a model result … all `INVALID`, all excluded from
> every pass rate"). One more (`-185140`) ran on a stale stack. The two that ended
> on the benchmark's own 12-turn budget (`-184627` 35/100, `-191558` 20/100) *are*
> genuine agent outcomes — but they sit on **different commits, n=1 each**, so the
> campaign has two isolated scores and **no valid comparison at all**: the run that
> was supposed to demonstrate the improvement never finished.

**Machine, as recorded.** Every artifact carries exactly four environment fields:
`host: machine-c385771a1404` (a sha256 of the hostname — the only machine
identifier [`benchmark.py`](benchmark.py) writes), `platform:
Windows-11-10.0.26200-SP0`, `python`, and `git_commit`. **GPU, core count,
physics backend and model are not captured by this harness at all**, so none of
them can be attributed to these runs after the fact. A previous edition of this
file named machine `9722d23d12a3`, an RTX 3060 Laptop GPU, a 16-core AMD64 host
and "Backend: OmniSim ODE"; none of those strings appear in any of the five
artifacts. The operating note at the time was that ODE was selected after Newton
cold initialization failed to bring up the controller bridge — retained here as
an **unverified operator note, not recorded provenance**. Capturing GPU, backend
and model in the artifact is the first thing to fix before re-running.

This benchmark measures agent planning and wheel-driven execution, not solver
throughput.

## Initial run — the one clean baseline

Artifact: `results/20260726-184627.json` (commit `4b61326d`)

- Score: **35/100 — FAIL**, on the benchmark's own terms: it ended on
  `chat.error: "hit max_turns=12"`, i.e. the agent exhausted its turn budget.
  It is the campaign's cleanest scored run; the only other genuine one
  (`-191558`, 20/100) visited zero cells on a superseded commit.
- Settled cells visited: 24
- Specialist: 30 turns / 30 tool calls
- Final state: cell `(8,1)`, `goto_cell_timeout`
- Completed hard gates: none

Mission Captain correctly formed a supervised delegation plan, and Husky Maze
made real physical progress. The specialist then spent its entire 30-turn
budget manually calling path primitives and never reached a verified corner
tour or Captain closeout.

## Changes prompted by the baseline

- Shared fail-closed outcome vocabulary and dependent-step ledger.
- Mission Captain `execute_mission_plan` with fresh-state preflight, duplicate
  ownership rejection, stop-on-first-failure, and stored `plan_id`.
- Captain `complete_mission` now requires that exact verified `plan_id`.
- Husky Maze `execute_cell_mission`: the model chooses ordered objective cells;
  the runtime performs live replanning, bounded slower retries, fault clearing,
  return-to-start, and the bridge claim.
- Current `executionContract` is placed before historical profile guidance in
  delegated sub-chats.
- The corners bridge now refuses completion unless the robot is physically back
  at `(0,10)`.

## Fully budgeted improved run — `INVALID`, terminated by the API

Artifact: `results/20260726-195352.json`

**This run has no valid score.** Its `chat` block has no `completed` key at all,
and records:

```
chat.error: "chat failed: ConnectionError: HTTPSConnectionPool(
  host='www.omnilink-agents.com', port=443): Max retries exceeded with
  url: /api/chat"
```

The transcript stops at turn 6 with `execute_mission_plan` still in flight. The
episode was cut by the platform, not ended by the agent, so the **40/100 the
scorer printed is `INVALID` and is excluded from every pass rate** — the same
treatment the sibling suite gives any infrastructure failure.

What the partial trace does show before the cut:

- Settled cells visited: 55 (the initial run reached 24)
- Specialist: 18 turns / 17 tool calls (initial run: 30 / 30)
- Verified remote corner `(10,10)` — a hard gate the initial run never reached
- Final state: cell `(8,2)` while another recovery leg was active
- Every hard gate still open: all four corners, return-to-start, bridge
  verification, verified Captain ledger, bound Captain closeout

⚠️ **Do not read those counts as a measured improvement.** They compare one
terminated run at commit `2a234a7f` against one run at commit `4b61326d`, n=1 on
both sides, model unrecorded on both sides. The previously published "**2.29×** /
40% fewer turns / 43% fewer calls" deltas were arithmetic on those two single
samples across different commits, and are **withdrawn**.

Two findings survive as qualitative product results rather than scores: the
bridge correctly refused the specialist's attempted premature completion claim,
and the planning/delegation layer is auditable and fail-closed. Neither is a
number. **This stack has not been shown reliable enough for unsupervised
long-horizon robot control — and no run in this campaign is a valid measurement
of whether it is.**

## The other three runs

Retained for audit, each labelled from what its own artifact records:

- `20260726-185140.json` — **`INVALID`** (scorer printed 25/100). Stale
  canonical-port runners; not comparable to anything else here. Ended on
  `chat.error: "hit max_turns=12"`.
- `20260726-185635.json` — **`INVALID`** (25/100). It ran before the
  execution-contract precedence change and reproduced the manual-primitive
  failure, but it ends on the **same** `/api/chat` `ConnectionError` as the
  improved run, with no `completed` key. It was **not** a "clean session", as a
  previous edition of this file called it; it was infrastructure-terminated and
  is excluded from any rate.
- `20260726-191558.json` — **genuine, low** (20/100), on a superseded commit.
  It ended on `chat.error: "hit max_turns=12"`: the **12-turn agent budget** ran
  out, which is the benchmark's own contract, not an infrastructure failure. A
  previous edition called it infrastructure-invalid because "the simulator's
  2,400-second cap expired mid-mission" — the string `2400` does not appear in
  the artifact, and `metrics.sim_time` is `null` for this run, so **no sim-time
  claim can be made about it at all** and the invalidation had no basis.
  `visited_cells` is 0; the trace does show the supervised mission primitive
  being selected, which is the one thing it demonstrates.

The `goto_cell_timeout` fault is real but belongs to the **initial** run: it
appears 13 times in `20260726-184627.json` (and 13 times in `-185635`), **zero
times** in `20260726-195352.json`. A previous edition attributed "repeated
skid-steer `goto_cell_timeout` failures around the east-side maze corridor" to
the improved run; the string `east` appears **zero** times in every artifact in
this directory, so that corridor attribution was never in the data and has been
removed rather than relocated.

No model judge was used. Every score came from local runner activity, the
physics-derived visited-cell trail, live fault/mode state, bridge claim log,
and Captain ledger IDs.
