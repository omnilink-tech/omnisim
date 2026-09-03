# Startup, Reset, And Asset Lifecycle

This document explains how world loading, asset download, reset behavior, and startup latency currently interact in OmniSim.

It is grounded in:

- `src/omnisim/app/OmApplication.cpp`
- `src/omnisim/nodes/utils/OmWorld.cpp`
- `src/omnisim/engine/OmSimulationWorld.cpp`
- `src/omnisim/core/OmDownloadManager.cpp`
- `src/omnisim/core/OmDownloader.cpp`

## Why This Area Matters

Startup and reset cost are a first-order usability problem.

They affect:

- local iteration speed
- CI runtime
- benchmark repeatability
- offline reliability
- the perceived quality of the simulator even before steady-state stepping begins

In OmniSim, startup cost is not one thing. It is an interaction between parsing, node creation, regeneration, finalization, and asset availability.

## Current Load Path

At a high level the current path is:

1. tokenize the world file
2. syntax-parse it
3. instantiate nodes through `OmNodeReader`
4. insert the nodes into the world
5. allow deferred regeneration and dictionary effects to settle
6. finalize the world
7. download referenced assets
8. enter or resume the simulation

This means startup latency is a blended metric. If we only time "loading" as one number, we hide which phase is actually responsible.

## Code-Backed Pressure Points

### 1. Asset download is still part of the critical path

`OmSimulationWorld` explicitly emits a `"Downloading assets"` status, resets the download manager, asks the root node to download assets, and then waits in a loop until download progress reaches 100.

During that loop it calls:

- `QCoreApplication::processEvents(QEventLoop::WaitForMoreEvents)`

This confirms that asset availability is still treated as blocking world startup behavior rather than as a separate lifecycle.

### 2. Download manager behavior pauses and resumes the simulation globally

`OmDownloadManager::createDownloader()` calls:

- `OmSimulationState::instance()->pauseSimulation()`

And once all downloads complete, `OmDownloadManager` calls:

- `OmSimulationState::instance()->resumeSimulation()`

That is a simple policy, but it couples:

- network behavior
- asset cache behavior
- simulation runtime state

more tightly than is ideal.

### 3. Reset is sensitive to asset state

`OmSimulationWorld::reset()` pauses simulation, resets time, blocks regeneration, resets runtime state, and only resumes simulation immediately if download progress is already 100.

That means reset cost and reset semantics are partly driven by asset lifecycle state, not just by world and physics state.

### 4. Successful downloads are cached, but cache behavior still leaks into startup predictability

`OmDownloader::finished()` saves successful primary downloads to disk through `OmNetwork::instance()->save(...)`.

That is useful, but it also means:

- cold-cache startup differs from warm-cache startup
- network dependence still matters for core world behavior
- benchmark results become noisier when worlds are not fully local

### 5. The load metric is still too coarse

The runtime already records a `loading` performance bucket, but the current path clearly contains multiple distinct phases:

- parse and syntax validation
- node instantiation
- template and dictionary work
- finalization
- asset wait time

Those should not stay merged forever.

## Why The Current Behavior Is A Problem

### For users

- startup can feel inconsistent
- reset latency is harder to explain
- remote or cache-sensitive worlds feel fragile offline

### For contributors

- it is hard to tell whether a change improved parsing, node creation, or asset wait time
- benchmark numbers are harder to trust
- changes that should only touch runtime behavior can still be distorted by download timing

### For coding agents

- the narrow validation story is weaker
- one world can behave differently depending on cache state
- the repo does not yet expose a crisp contract for "headless deterministic world load"

## Recommended Direction

### 1. Separate world construction from asset readiness

The simulator should distinguish between:

- world structure is loaded and valid
- optional or remote assets are still resolving
- simulation may or may not proceed depending on mode

That can still preserve current product behavior while making the lifecycle explicit.

### 2. Forbid remote asset dependence in benchmark and smoke worlds

Core validation worlds should not rely on runtime download at all.

That is the simplest way to make startup and reset numbers trustworthy.

### 3. Split load timing into real sub-buckets

Add distinct measurements for:

- tokenization and syntax parse
- node read and instantiation
- template and dictionary settlement
- world finalization
- asset wait time
- first-frame render time

Without this, contributors keep optimizing the wrong part of startup.

### 4. Add explicit load and reset modes

The runtime should eventually support a documented distinction between:

- strict deterministic headless mode: local assets only, no remote waits
- interactive desktop mode: remote assets allowed, with explicit progress behavior
- benchmark mode: local assets only, stable measurement contract

Right now those modes are only partially implied by world choice and CLI options.

### 5. Treat asset prefetch as a separate workflow

If remote assets must exist, their resolution should ideally happen through:

- packaging
- project import
- explicit cache warm-up

not as hidden startup work in the main simulation path.

## Low-Risk Changes To Do First

These are good immediate improvements:

- split the `loading` performance bucket into smaller load-phase buckets
- add an explicit asset-wait metric
- audit smoke and benchmark worlds for remote assets
- document cold-cache versus warm-cache expectations

These improve observability without changing the fundamental user-facing behavior yet.

## Phase-Two Refactor Sequence

1. instrument parse, instantiate, regenerate, finalize, and asset-wait time separately
2. keep all benchmark and smoke scenarios local-asset only
3. make headless mode reject remote-asset dependence explicitly
4. introduce a cleaner world-ready versus assets-ready lifecycle
5. move long-latency asset acquisition out of the synchronous simulation hot path where possible

That order keeps determinism and benchmark quality improving throughout the refactor.

## Validation Guidance

After startup or reset work, validate with:

- `python -m omnisim test-smoke`
- `python -m omnisim profile-world resources/projects/worlds/empty.omniworld`
- `python -m omnisim benchmarks`

Then compare:

- cold-cache versus warm-cache behavior
- empty-world startup versus template-heavy world load
- reset latency before and after the change

If those measurements are not easy to collect, the startup contract is still too implicit.
