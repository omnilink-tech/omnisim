# OmniLink hard agent benchmark

`omnilink-hard/v1` is the post-baseline suite. It is intentionally separate
from `omnilink-tasks/v1`, so improving the original 16-task score cannot make
this benchmark easier. (Sixteen, not fifteen: `ol_suite.py` defines 16 tasks and
[`README.md`](README.md) already says sixteen.)

Run the seven hard tasks:

```bash
python tests/benchmarks/omnilink_tasks/matrix.py \
  --suite hard \
  --engines g1-engine \
  --model gemini-3.1-flash-lite \
  --repeat 3 \
  --pace 5
```

The suite combines dependent motion, compound concurrency, state restoration,
ambiguity handling, guard-triggered recovery, two-round live delegation, and
fleet failure containment. Every verdict uses measured robot pose and the
recorded tool/delegation trace; there is no model judge.

Use `--suite hard --list` for the generated contract table. Results carry
`suite: omnilink-hard/v1` and a row per run recording the machine, model, Git
state, latency, tool trace, and measured account cost — so a run is attributable
to all of them *provided you pass `--model` and run on a clean tree*. The
2026-07-26 campaign below did neither, which is why it is not a measurement.

## 2026-07-26 exploratory campaign — NOT a reproducible measurement

Venue: machine `9722d23d12a3`, NVIDIA GeForce RTX 3060 Laptop GPU, `g1-engine`.

Read the three caveats before any number below. They are larger than the numbers.

1. **The rows cannot be reproduced from a clone.** `results/` is `.gitignore`d,
   so a fresh checkout gets this prose and zero rows. Worse, every row records a
   `+dirty` tree at a sha — `3786fb6e`, `25fbc755`, `cf74d7b7` — that **predates
   `ol_hard_suite.py`**, the file that defines all seven tasks *and* their
   graders (first committed in `ee103aff`). Nothing here can be re-run at the sha
   it claims to have run at.
2. **The model is unpinned on the later runs.** Every row at `cf74d7b7` records
   `"model": null`; only the two baseline runs carry
   `"model": "gemini-3.1-flash-lite"`. An unpinned route serves its provider's
   current default tier, which changes underneath you — see
   [`README.md`](README.md), "Always `--model` and always label by model".
3. **n=1 is an anecdote by this suite's own rule** ([`README.md`](README.md): "A
   single sample is an anecdote").

### Best available estimate: 11 PASS / 2 FAIL / 2 INVALID

From `hard-conquered-g1-n3-20260726` — the widest repeated sweep at the final
sha, 15 rows of an n=3 run over all seven tasks that stopped part-way through its
third repeat. The two genuine failures:

- `hard_square_return`, repeat 3 — FAIL;
- `hard_conditional_delegation`, repeat 2 — FAIL.

The two `INVALID`s are infrastructure and are excluded from the rate, per the
sibling rule: `hard_conditional_delegation` repeat 1 (external edge relay
returned `tools_unavailable`) and `hard_guarded_recovery` repeat 2 ("near-boundary
setup was not established").

A single-repeat sweep at the same sha, `hard-final-g1-n1-20260726`, did pass
**7/7 in 385.1 s for a measured $0.0113**. That is one sample per task, and it is
reported here as an existence proof that all seven tasks *can* pass in one
session — not as a score, and not as evidence of improvement.

⚠️ **A previous edition of this file claimed "+28.6 percentage points, or a 40%
relative increase" over a "fair baseline" of 5/7. That delta is withdrawn.** The
5/7 baseline was never a run: it spliced five tasks from
`hard-baseline-g1-20260726` (sha `3786fb6e+dirty`, 3 PASS / 4 FAIL) with two from
`hard-baseline-corrected-g1-20260726` (sha `25fbc755+dirty`, recorded 14 minutes
later after harness edits). Two different shas, two different harnesses, no run
that ever scored 5/7 — so nothing valid can be subtracted from anything.

The harness defects the corrected run addressed were real (guard-scene
orientation and an omitted edge-error classification), and both re-run tasks
passed afterwards. That is a fix worth recording; it is not a baseline.

### Per-skill repeated runs, including the runs that disagree

- **square return** — 3/3 in `hard-targeted-v3-g1-n3-20260726`, but FAIL on
  repeat 3 of `hard-conquered-g1-n3-20260726` at the *same* sha.
- **guarded rejection plus fallback** — 3/3 in `hard-targeted-v3`; one `INVALID`
  in `hard-conquered`.
- **two-round live delegation** — 2/3 valid passes in `hard-targeted-v3` (third
  run `INVALID` on the edge relay), and FAIL on repeat 2 of `hard-conquered`.
- **four-way inward parallel motion** — 3/3 in
  `hard-parallel-radial-g1-n3-20260726`, after 2/3 in
  `hard-parallel-racefix-g1-n3-20260726` at the same sha.

### All raw rows

Under the ignored local `results/` directory, complete rather than selected:
`hard-baseline-g1-20260726`, `hard-baseline-corrected-g1-20260726`,
`hard-focused-improved-g1-20260726`, `hard-improved-g1-n3-20260726`,
`hard-parallel-racefix-g1-n3-20260726`, `hard-parallel-radial-g1-n3-20260726`,
`hard-conquered-g1-n3-20260726`, `hard-targeted-v3-g1-n3-20260726`, and
`hard-final-g1-n1-20260726`.

**A real measurement of this suite has not been made.** It needs: one clean sha
at or after `ee103aff`, a pinned `--model`, `--repeat 3` across all seven tasks in
a single sweep, and the rows archived somewhere a clone can read.
