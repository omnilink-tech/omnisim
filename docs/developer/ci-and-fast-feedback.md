# CI And Fast Feedback

This document describes how OmniSim should structure CI and local validation so contributors can get useful answers quickly instead of waiting for the full product pipeline on every change.

It complements:

- `build-and-iteration.md` for local build commands
- `validation-playbook.md` for choosing a validation lane
- `profiling-playbook.md` for performance measurement

## The Problem To Solve

Right now the repository contains both:

- expensive product-level build and packaging logic
- small, targeted changes that should be provable much earlier

If every pull request defaults to full packaging and broad regression, the cost is predictable:

- slower contributor feedback
- slower coding-agent iteration
- more pressure to batch unrelated changes together
- worse signal because failures arrive late and mix many causes

Fast CI is not about skipping quality. It is about ordering checks by cost and confidence.

## What Exists Today

Two workflow files already point in the right direction. Both currently live under `.github/workflows.disabled/` as templates rather than active checks (the only workflows under the active `.github/workflows/` are `g1-spec-conformance.yml` and `update_sponsors.yml`):

- `.github/workflows.disabled/developer_fast_path.yml`
- `.github/workflows.disabled/smoke_linux_fast.yml`

They demonstrate two important ideas:

- repository surface and command contracts can be validated cheaply
- a Linux fast-path build with `ccache` and a small smoke suite can provide much earlier signal than a full package job

Those ideas should become the default structure, not exceptions.

## Recommended CI Lanes

Keep CI organized into explicit lanes with different goals.

### 1. Surface validation

Purpose:

- confirm the documented developer contract still exists
- validate lightweight repo invariants
- fail fast on obviously broken entrypoints

Typical checks:

- docs and repo-guide presence
- top-level target existence
- script syntax and argument validation
- lightweight static checks

When to run:

- every PR
- every branch push

Target runtime:

- a few minutes at most

### 2. Fast build and smoke

Purpose:

- compile the narrow product slices most likely to break
- run one or two representative smoke worlds

Typical checks:

- `make sim-core`
- `make sim-gui` or a narrow desktop binary target
- `make tests-smoke`
- one headless world run

When to run:

- every PR by default

Target runtime:

- short enough that developers still wait for the result before context-switching away

### 3. Targeted subsystem lanes

Purpose:

- validate only the subsystem family touched by a change

Examples:

- renderer-focused build for `src/wren` and `src/omnisim/wren`
- controller-library lane for `src/controller`
- parser or world-load lane for `src/omnisim/vrml` and `src/omnisim/nodes`

When to run:

- path-based trigger
- label-based trigger
- optional PR comment trigger for deeper checks

Target runtime:

- moderate, but still cheaper than full regression

### 4. Benchmark lane

Purpose:

- detect obvious performance regressions on a stable small scenario set

Typical checks:

- startup time
- world load time
- one physics-heavy scenario
- one sensor-heavy scenario

When to run:

- nightly
- merge queue
- performance-sensitive PRs

Important rule:

- keep benchmark worlds local, deterministic, and small enough to avoid flaky infrastructure noise

### 5. Full regression and package validation

Purpose:

- protect releases
- verify packaging and broad product behavior

When to run:

- merge queue
- release branches
- nightly or scheduled validation

This should not be the default feedback path for every incremental contributor edit.

## CI Design Rules

### Make the public command set stable

CI is cheaper when it calls a small supported interface instead of reproducing ad hoc build logic in each workflow.

Prefer workflows that call stable commands such as:

- `make sim-core`
- `make sim-gui`
- `make renderer`
- `make tests-smoke`
- `make benchmarks`

If a workflow must know internal build plumbing to do its job, the developer surface is still too implicit.

### Cache expensive setup aggressively

The repo pays a large fixed cost for environment setup and dependency staging. Cache anything that is:

- large
- deterministic
- slow to reconstruct

Examples:

- compiler caches such as `ccache`
- MSYS or equivalent toolchain setup
- unpacked dependencies
- generated compile database artifacts when safe

Do not spend ten minutes rebuilding the environment to re-check a two-line C++ change.

### Keep PR defaults cheap and broad enough

A good PR default lane should:

- compile enough code to catch obvious integration mistakes
- exercise one representative runtime path
- finish early enough to guide the author before they move on

That usually means one fast build lane plus one smoke lane, not full matrix packaging.

### Use later lanes for confidence, not discovery

Broad package and regression jobs should confirm that the change is ready to merge. They should not be the first time anyone learns the branch is broken.

## Path-Based Trigger Ideas

These mappings keep CI proportional to the change.

- `src/wren/**`, `src/omnisim/wren/**`: renderer build, renderer smoke, benchmark lane if rendering-sensitive
- `src/controller/**`: controller-libs build, controller smoke
- `src/omnisim/vrml/**`, `src/omnisim/nodes/**`: sim-core build, headless smoke, world-load benchmark
- `src/omnisim/gui/**`, `src/omnisim/scene_tree/**`, `src/omnisim/editor/**`: sim-gui build, desktop smoke
- `docs/developer/**`, top-level repo guide files: surface validation only

Path filters should not be perfect. They only need to keep obvious low-risk changes off the most expensive lanes.

## How This Improves Coding-Agent Productivity

Agents are especially sensitive to feedback latency because they iterate through many small hypotheses.

Fast CI and narrow local commands help agents by:

- making the correct validation command easy to choose
- reducing the risk of touching broad build surfaces accidentally
- allowing smaller, more focused changes
- providing faster proof that a boundary or refactor actually improved the build surface

An agent cannot optimize the repo well if every experiment requires the full desktop package pipeline.

## Immediate Improvements Worth Implementing

Short-term work that gives real value:

- promote the `developer_fast_path.yml` and `smoke_linux_fast.yml` templates out of `.github/workflows.disabled/` into active checks, then keep them green and documented
- add path-aware targeted lanes instead of one broad default lane
- standardize benchmark result collection on a small world set
- cache heavy environment setup more aggressively
- make workflow logs clearly report which public target was built

## Failure Modes To Avoid

Avoid these CI mistakes:

- defaulting every PR to full packaging and cross-platform release validation
- hiding important build logic directly in workflow YAML instead of behind stable commands
- using benchmark worlds that fetch remote assets
- letting smoke tests depend on editor-only behavior when the change is runtime-only
- building more targets than the workflow claims to validate

These failure modes make CI slower and make it harder for contributors to trust the results.

## Definition Of A Good Fast Lane

A fast lane is good if it:

- maps cleanly to one documented command
- finishes quickly enough to shape local iteration
- validates the subsystem that actually changed
- fails with logs that explain which boundary broke

That is the standard to aim for as the build surface becomes more modular.
