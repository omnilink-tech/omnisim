# Lane 2, corrected: OmniSim's embedded batched-GPU overhead is ~1.2–1.3×, not 4.5–17×

**Date:** 2026-08-08 · **Machine:** `9722d23d12a3` (RTX 3060 Laptop, driver 596.36,
AMD 16-core, Windows 11) · **Build:** post-`5b380175` working tree ·
**Raw rows:** [`lane2/results/lane2_graphed_ab_2026-08-08.jsonl`](../../tests/benchmarks/omnibench/lane2/results/lane2_graphed_ab_2026-08-08.jsonl)
(16 rows, n=3 per cell)

## The finding

OmniBench lane 2's published overhead ratios (17.4× / 11.9× / 4.5× on this GPU)
compared a **CUDA-graph-captured** raw `mujoco_warp` baseline against an
**ungraphed** OmniSim probe. Tier `Ag` — the graphed OmniSim arm, described in
`run_throughput.py`'s own header as *"the like-for-like partner of the graphed raw
baseline"* — existed in the code and **had never been run**.

Run like-for-like, both arms `cuda_graph: true`:

| nworld | raw `mujoco_warp` (graphed) | OmniSim embedded (graphed, `Ag`) | **overhead** |
|---:|---:|---:|---:|
| 256  | 47,949 env-steps/s | 40,092 env-steps/s | **1.20×** |
| 1024 | 117,349 env-steps/s | 91,577 env-steps/s | **1.28×** |

Medians of 3. Spread is tight: raw@1024 0.4%, `Ag`@256 1.4%, worst cell
(`Ag`@1024) 6%.

For contrast, the same data reassembled the way the published figure was built —
graphed raw against the **ungraphed** OmniSim arm measured in the same session:

| nworld | ungraphed OmniSim (`A`) | ratio vs graphed raw |
|---:|---:|---:|
| 256  | 1,571 env-steps/s | 30.5× |
| 1024 | 5,801 env-steps/s | 20.2× |

So the overhead the lane has been reporting is **dominated by CUDA-graph capture,
not by OmniSim's engine layer**. The honest statement is that OmniSim's embedded
deploy-solver path costs roughly **20–28% more than driving `mujoco_warp`
directly**, on this GPU, at these batch sizes — for which you get the full scene
graph, sensors, controller IPC and the train==deploy guarantee.

## What this does NOT say

- **It is not a speedup.** Nothing got faster; a comparison got fair. The
  ungraphed numbers are real and are what you get if you do not capture a graph.
- **`Ag` is not the default.** `--tiers` defaults to `raw,A,B` and `run_all`
  passes `raw,A,B,C`, so a stock run still produces the misleading pairing.
  Quoting `Ag` obliges you to re-quote `A` beside it, as this page does.
- **One machine, one GPU, two batch sizes.** 4096 was not attempted (6 GB card);
  the published campaign's larger-batch cells are not superseded by this page.
- **`sim_only`, not training.** This is tier-1/2 sim throughput on
  `go2_newton.xml`, not an RL train rate. Lane 2's B and C tiers are untouched.
- **Not a cross-engine correctness claim.** Throughput says nothing about
  fidelity; see [lane1-validity-2026-08-07.md](lane1-validity-2026-08-07.md).

## Why it was wrong for so long

The asymmetry is invisible from the outside: both tiers are labelled
`sim_only` and both emit `engine=omnisim-newton` / `engine=mujoco-warp-raw`, so a
results table shows two `sim_only` rows and one ratio. The distinguishing field
is `metrics.cuda_graph`, which the published summary did not group by. The fix is
not a measurement change but a **grouping** change: never compare two lane-2 rows
whose `cuda_graph` differs.

**Recommended follow-ups**, in order: make `--tiers` default to `raw,Ag` (or
refuse a `raw`-vs-`A` ratio outright); add `cuda_graph` to the aggregator's
group-by key so the pairing cannot be built by accident; re-run on the 4090 with
4096 to see whether the ~1.2× holds as the batch grows.
