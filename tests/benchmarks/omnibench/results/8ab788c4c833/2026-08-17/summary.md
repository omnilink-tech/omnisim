# `8ab788c4c833` — 2026-08-17, lane 4 + cloth

RunPod RTX A4500 20 GB · **AMD EPYC 7352 24-Core @ 2.3 GHz, 48 threads** ·
Ubuntu 22.04 (Secure, EU-RO-1) · engine binary `6f7e2217426a2088` (built from
committed sources 2026-08-16 14:47Z, off a shared network volume) ·
newton 1.5.0 / warp 1.16.0 / mujoco 3.11.0 / mujoco-warp 3.11.0 · $0.25/hr,
~35 min, $0.15 total.

Write-up, including why a disagreement with the laptop is not automatically a
machine finding:
[docs/benchmarks/lane4-multimachine-2026-08-17.md](../../../../../../docs/benchmarks/lane4-multimachine-2026-08-17.md).

| file | headline |
|---|---|
| `lane4/coverage.jsonl` | all 45 capability probes. **31 works / 4 degraded / 5 broken / 4 absent / 1 no-result = 78% of what exists** — the same figure as the laptop, reached by three offsetting per-probe changes rather than by replication. **43 of 45 verdicts agree.** |
| `lane4/envelope.jsonl` | **200 boxes @ 1.35× real time**, ceiling not reached; rovers **overflow at N=16 (peak nefc 328 vs a njmax pinned at 256)**, `cliff_detector_validated: true`. |
| `lane4/cpu_only.jsonl` | Newton finalises with no CUDA device visible; analytic drop rests at 0.6499 m against 0.65 expected; trajectory identical to the GPU-visible run (max deviation 0.0 m). |
| `cloth_step_cost/sweep_results.jsonl` | cloth on HEAD's runtime, **no engine involved**, so this is the one lane here with no binary confound. Drape as shipped 2.359 ms/step (3.39×); at 2 VBD iterations 0.604 ms/step (13.24×). |

⚠ This machine's engine binary is ~20 h older than the laptop's and they are
different compilers on different operating systems. Read each row's own
`machine` block before quoting it.
