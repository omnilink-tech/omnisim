# agents/benchmarks/

Reproducible head-to-head benchmarks of OmniLink agents against scripted (open-loop) baselines on shared OmniSim worlds. The point of each benchmark is to answer:

> **Where exactly does scripted control break down, and where does an OmniLink-style agent earn its keep?**

Each benchmark ships both solvers, a runner, and verified results. Same physical surface, same task — only the brain differs.

## Benchmarks

_No benchmarks currently shipped in this directory._

Looking for the real, shipped suite? The **OmniLink benchmark suite** (three graded tasks, headless) lives at [`tests/benchmarks/omnilink_tasks/`](../../tests/benchmarks/omnilink_tasks/) — run it with `python tests/benchmarks/omnilink_tasks/run.py`.

## Adding a new benchmark

```
agents/benchmarks/<your_benchmark>/
├── README.md          What it tests, results table, how to run
├── run_benchmark.py   Orchestrator that runs both solvers and grades
├── scripted_solver.py Open-loop baseline (no manifest reads, no fault probes)
└── agent_solver.py    State-aware solver (reads manifest, probes for faults, replans)
```

Run convention: launch the world separately, then invoke `python agents/benchmarks/<name>/run_benchmark.py`. The runner does not launch OmniSim — that decoupling lets you watch the simulation while the bench runs.
