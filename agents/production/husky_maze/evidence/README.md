# husky_maze A/B evidence

Raw rows behind [`docs/AB_ARMS_2026-09-19.md`](../docs/AB_ARMS_2026-09-19.md).
Produced by [`scripts/ab_arms.py`](../scripts/ab_arms.py). Each run directory holds,
per trial: the engine log, its `.newton.json` sidecar (the proof Newton drove the run),
the engine's stdout, the arm's own stdout, and `trials.jsonl`.

Machine `9722d23d12a3`, engine `d25c20b7b44fb050`, newton/MuJoCo `cpu/mj_step`.

| run | what it is | cited for |
|---|---|---|
| `ab_20260919_162707` | first pilot, **before** the solver fix | the `410 Gone` traceback proving `solve.py` died at step 1 of 72 on `snap_to_cell` |
| `ab_20260919_162857` | harness crash (a `setdefault` bug in `grade()`, since fixed) | nothing — kept only so the run numbering has no silent hole |
| `ab_20260919_163004` | first trial **after** the solver fix | nothing — superseded by the n=5 run below |
| `ab_20260919_163119` | **arm D, n=5, `husky_maze`** | 5/5 goal reached, 46.4 s ± 1.96 s, and the determinism result (9 of 148 lines differ, all ±0.01 rad yaw) |
| `ab_20260919_163837` | **arm D, n=1, `husky_maze_unknown`** | wall-follower reaches the goal in 118.8 s, 2.6× BFS-with-map |
| `ab_llm_arm` | **arm L, blocked** | `400 invalid_grant: account not found` — the LLM arm was never measured |

⚠️ No arm-L performance number exists anywhere in this directory. The comparison is
owed, not concluded.
