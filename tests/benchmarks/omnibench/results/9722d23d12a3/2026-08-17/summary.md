# `9722d23d12a3` — 2026-08-17, lane 4 + cloth arm

Laptop arm of the lane-4 / cloth multi-machine campaign. Write-up, with the
attribution of every cross-machine disagreement:
[docs/benchmarks/lane4-multimachine-2026-08-17.md](../../../../../../docs/benchmarks/lane4-multimachine-2026-08-17.md).

RTX 3060 Laptop 6 GB · AMD Ryzen, 16 threads · Windows 11 · engine binary
`13906cc6f12451eb` · newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0.

| file | what it is |
|---|---|
| `lane4/envelope.jsonl` | same-day re-sweep of the resource envelope. **200 boxes @ 1.45× real time**; rovers **overflow at N=16 (peak nefc 336 vs njmax 256)**, so `cliff_detector_validated: true`. |
| `lane4/lidar_recheck.jsonl` | `device.lidar` re-run on the current binary: `works` in 3.7 s. The published matrix had carried `no result` since 2026-08-15. Merged into `lane4/results/coverage.jsonl` by `merge_coverage.py`. |
| `lane4/controls/hinge2_ball_gate_off.jsonl` | ⚠ **CONTROL RUN, not a measurement of the shipped default.** `joint.hinge2_motor` under `OMNISIM_NEWTON_BALL_HINGE2=0`, which is what the pod's older binary runs. Reproduces the pod's arm displacement (1.4276448946637377e-14 m) to all 17 printed digits, attributing that disagreement to the engine build rather than the machine. |
| `cloth_step_cost/sweep_results.jsonl` | cloth step-cost matrix on HEAD's runtime. Supersedes `tests/benchmarks/cloth_step_cost/sweep_results.jsonl`, whose shipped-drape cell was taken 29 min before `1fb7f135f` and reads 30.323 ms/step against 2.847 today. |
| `cloth_step_cost/cpu_forced.jsonl` | the drape with the device forced to CPU: **51.87 ms/step, 19.3 frames/s, 0.154× real time**, correcting the "6.7 fps" figure in `docs/developer/cloth-simulation.md`. Still one machine. |

The lane-4a capability rows for this machine live in
`tests/benchmarks/omnibench/lane4/results/coverage.jsonl`, which is the file
`report.py` renders and is deliberately per-machine.
