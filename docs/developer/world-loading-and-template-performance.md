# OmniSim World Loading and Template Performance

This document focuses on one of the most important and least isolated parts of the simulator: the path from a world file on disk to a fully initialized scene ready to step.

World-load performance matters because it affects:

- startup time
- reset time
- benchmark repeatability
- scene editing responsiveness
- automation and coding-agent workflows

## Current Load Pipeline

The current world-load flow is spread across several subsystems.

### Stage 1: Tokenization

`src/omnisim/app/WbApplication.cpp` tokenizes the world file before any real syntax or node work begins.

Implication:

- large worlds pay a full tokenizer pass up front
- tokenization errors fail early, which is good

### Stage 2: Syntax parse

`WbApplication` then runs `WbParser::parseWorld(...)`.

Implication:

- syntax validation is a full pass over the token stream
- progress reporting is tied to tokenizer position and token count

### Stage 3: Node reader pass

After syntax parsing succeeds, `WbWorld` constructs a `WbNodeReader` and rewinds the tokenizer again to instantiate nodes.

Implication:

- world loading is effectively multi-pass
- large files pay for syntax validation and node creation as separate token-stream traversals

This is one of the clearest current opportunities for load-time reduction.

### Stage 4: Node insertion

`WbWorld` inserts the instantiated nodes into the root group and validates insertion constraints as it goes.

Implication:

- node creation cost is not the whole load cost
- insertion-time validation and compatibility checks are part of the critical path

### Stage 5: Template regeneration control

`WbTemplateManager` is blocked during initial insertion and then unblocked once the bulk insert phase completes.

Implication:

- the code is already trying to batch some expensive regeneration behavior
- but regeneration remains a broad system with complex signal wiring

### Stage 6: Dictionary and DEF/USE updates

After world creation, dictionary updates and related scene consistency work still run.

Implication:

- load cost includes more than parser and node instantiation
- template regeneration and dictionary maintenance can amplify one another

### Stage 7: Node finalization and world initialization

Finalization, mandatory-node checks, and later startup work still happen before the world is really ready.

### Stage 8: Asset download and cache-dependent startup

The simulation world still downloads assets in the load path. The download manager pauses simulation while downloads are active.

Implication:

- remote assets directly affect world-load time
- cache state affects benchmark stability
- “load time” currently blends content retrieval with parsing and instantiation

## Major Cost Centers

## 1. Multi-pass parsing and reading

Today the load path pays for:

- tokenization
- syntax parsing
- node-reader pass
- insertion/finalization work

That is a reasonable safety-first design, but it is not the cheapest design.

### Improvement path

Near-term:

- instrument load time by phase so the cost of tokenization, syntax parsing, and node creation are visible separately

Later:

- introduce an optional fused parse-and-instantiate path for trusted, benchmarked scenarios
- or keep the syntax pass but make the second pass cheaper by caching parse structure

## 2. Template regeneration machinery

`WbTemplateManager` currently:

- tracks template nodes in a central list
- connects to parameter and field change signals
- blocks regeneration during bulk operations
- replays regeneration when unblocked
- can trigger nested regeneration
- coordinates with dictionary updates

This is flexible, but it creates broad coupling.

### Why it matters

- regeneration cost is hard to predict
- edit-time performance depends on signal fan-out
- world-load performance depends on how much deferred regeneration flushes at unblock time

### Improvement path

Near-term:

- log number of regenerating nodes and regeneration passes during world load and reset
- make regeneration timing visible in performance logs

Medium-term:

- narrow which fields subscribe descendants for regeneration
- separate “must regenerate now” from “can defer until post-load”
- reduce regeneration work during dictionary maintenance

## 3. Dictionary maintenance and DEF/USE behavior

`WbDictionary` and related node operations still do broad scene-consistency work during regeneration and insertion.

This affects:

- world-load cost
- node deletion/insertion cost
- scene-tree stability
- PROTO and USE semantics

### Improvement path

Near-term:

- distinguish world-load dictionary updates from edit-time dictionary updates
- reduce layout and scene-tree side effects during bulk world-load updates

Medium-term:

- batch dictionary updates as transactions
- reduce repeated node replacement patterns during regeneration

## 4. Asset retrieval in the critical path

The current load path still allows remote and cache-dependent assets to influence world startup.

This is one of the biggest reasons world-load numbers can be noisy.

### Improvement path

Near-term:

- keep benchmark and smoke worlds free of remote assets
- make cache misses a visible world-quality problem, not just a runtime surprise

Medium-term:

- prefetch remote assets before entering world-finalization work
- separate “world model is ready” from “all optional assets are resident”

## 5. Bounding-sphere and derived-state management

Bounding-sphere updates are explicitly disabled and later re-enabled around world load, and the bounding-sphere system documents that some cached values are not dirtied for all graphical cases.

That is a hint that load and edit behavior rely on careful cache management rather than a simple always-correct incremental model.

### Improvement path

- keep cache invalidation rules explicit
- reduce surprise recomputation at first use
- avoid broad recomputation in edit-time paths unless the world truly changed structurally

## What To Improve First

### Priority A: Add phase-level world-load metrics

Split current load time into:

- tokenize
- syntax parse
- node reader instantiate
- insertion and validation
- template regeneration
- dictionary update
- finalization
- asset retrieval

Without this split, world-load optimization remains guesswork.

### Priority B: Reduce double-work in parsing and creation

The current multi-pass design is safe but expensive. The main effort should be to reduce repeated token-stream work while keeping good error behavior.

### Priority C: Make template work narrower and more transactional

Template regeneration should be easier to reason about in three situations:

- initial world load
- supervisor/world mutation at runtime
- scene-tree editing

These are related, but they should not all pay the same coordination cost.

### Priority D: Keep remote assets off the critical path

At minimum:

- no remote assets in performance worlds
- no remote assets in smoke worlds
- clear diagnostics when a world relies on cache state

## Developer Checklist For Load-Path Changes

When touching:

- `src/omnisim/app/WbApplication.*`
- `src/omnisim/vrml/WbParser.*`
- `src/omnisim/vrml/WbNodeReader.*`
- `src/omnisim/nodes/utils/WbTemplateManager.*`
- `src/omnisim/nodes/utils/WbDictionary.*`
- `src/omnisim/nodes/utils/WbWorld.*`
- `src/omnisim/engine/WbSimulationWorld.*`

Do all of the following:

- identify which load stage you are changing
- run at least one benchmark or profile-world scenario before and after
- note whether the change affects determinism, warning behavior, or cancellation behavior
- avoid introducing new asset or network work into the critical path

## World Authoring Rules That Help Load Time

- keep benchmark worlds local-asset and deterministic
- avoid large unused PROTO nesting when simpler composition will do
- keep texture and mesh counts proportional to what the scenario actually needs
- prefer worlds that do not trigger dictionary churn or regeneration-heavy editing patterns during tests

## What A Better Load Path Looks Like Later

A later, better world-load design should have:

- explicit phase timing
- one supported headless load contract
- narrower template regeneration scopes
- transactional dictionary updates
- optional fused parse-and-instantiate behavior
- asset retrieval separated from “core world is ready”

That is the path toward faster starts, faster resets, and more stable automation.
