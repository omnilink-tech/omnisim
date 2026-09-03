# Template Regeneration And Dictionary Coherence

This document explains the current relationship between template regeneration, DEF/USE dictionary maintenance, and downstream desktop-shell behavior.

It is grounded in:

- `src/omnisim/nodes/utils/OmTemplateManager.cpp`
- `src/omnisim/nodes/utils/OmDictionary.cpp`

This is one of the most cross-cutting seams in the simulator. It affects:

- world-load time
- runtime mutation cost
- scene-tree behavior
- contributor ability to reason about PROTO-driven changes

## Why This Area Matters

The template system is powerful, but the cost of that power shows up in several places:

- subscriptions to regeneration triggers
- deferred or blocked regeneration
- dictionary rebuilding and nested dictionary maintenance
- node replacement during regeneration
- secondary effects in the scene tree and selection model

This is exactly the kind of subsystem that can make the simulator feel both slow and brittle if its contracts are not explicit.

## What The Current Code Shows

### Regeneration is intentionally deferrable

`OmTemplateManager::blockRegeneration(bool block)` flips a global manager flag.

When regeneration is blocked:

- nodes are marked as needing regeneration
- regeneration is deferred
- unblocking later replays the pending work

That is a practical mechanism for avoiding inconsistent mid-update state, but it also means regeneration is already operating like a coarse transaction system without being named as one.

### Subscription is recursive and broad

`OmTemplateManager::subscribe(...)` and `recursiveFieldSubscribeToRegenerateNode(...)` walk node fields and descendants to attach regeneration behavior where needed.

This gives the feature good coverage, but it also means:

- a lot of coupling is expressed through dynamic subscriptions
- the manager owns a wide slice of the mutation graph
- contributors have to reason about both immediate field changes and deferred regeneration side effects

### Regeneration and dictionary maintenance are tightly interwoven

During regeneration, the template manager explicitly blocks regeneration around:

- finalization of the new node
- dictionary updates for the regenerated node

It also tracks the upper regenerated template node to avoid infinite loops while the USE/DEF dictionary is being updated.

This is a strong signal that:

- regeneration and dictionary maintenance are not independent concerns
- nested mutation loops are a real risk
- the current safety mechanism is mostly stateful coordination

### Dictionary updates explicitly forbid nested updates

`OmDictionary` contains asserts stating that nested dictionary updates should be avoided.

That is useful defensive code, but it also tells us the subsystem boundary is still fragile:

- correctness depends on update ordering
- re-entrancy is feared rather than encapsulated
- the contract is enforced partly through global state and assertions

### Some dictionary paths still fall back to broad recomputation

There are at least two important signs of incompleteness in `OmDictionary.cpp`:

- `TODO` comments noting `setItem(...)` should replace `insertItem(...)` once fixed
- `removeNodeFromDictionary(...)` returning early when `useCount() > 0`, because the dictionary will be completely recomputed

So the current system still contains operations that are broader or more fragile than the intended steady state.

### USE-local and nested dictionaries make scope explicit but expensive

`OmDictionary` creates local dictionaries for USE nodes and nested PROTO situations.

That is semantically necessary, but it also means dictionary work scales with:

- nesting depth
- regeneration frequency
- whether nodes switch between USE and DEF forms

This is why seemingly small PROTO edits can have wide side effects.

## The Main Problems

### 1. Regeneration behaves like a transaction system without a formal transaction model

The manager blocks regeneration, defers work, updates nodes, updates dictionaries, and then may trigger regeneration again.

That is a transaction in practice, but not yet a clearly modeled one.

### 2. Dictionary correctness still leans on global mutable state

Flags such as:

- the current regenerated node
- whether proto regeneration is active
- whether regeneration is blocked

all work, but they increase the chance that a future change introduces subtle ordering bugs.

### 3. Broad recomputation paths are still present

Whenever the system falls back to full dictionary recomputation, contributor intuition about mutation cost becomes unreliable.

That is bad for both performance and code reasoning.

### 4. Downstream UI costs are coupled to regeneration semantics

Because regeneration can turn USE nodes into DEF nodes or alter tree structure, the scene tree ends up paying for the same mutation event through:

- layout changes
- selection clearing
- tree-state restoration

This is why the template/dictionary seam is not just a parser concern.

## Recommended Direction

### 1. Name the lifecycle explicitly

Later refactors should treat regeneration as a structured pipeline:

1. collect affected nodes
2. freeze regeneration side effects
3. apply node replacement or finalization
4. update dictionary state
5. publish a summarized change set

Right now those phases exist, but they are encoded mostly in control flow.

### 2. Introduce smaller change sets for dictionary updates

The long-term goal should be a model where dictionary maintenance publishes:

- inserted DEF entries
- removed DEF entries
- USE-to-DEF transitions
- subtree-level scope changes

That is better than forcing downstream consumers to infer the change from broad model invalidation.

### 3. Reduce full recomputation paths

The remaining broad recompute cases should be identified and turned into targeted updates wherever safe.

This matters for:

- mutation latency
- scene-tree coherence
- contributor trust in the fast path

### 4. Separate runtime correctness from UI reaction

The template and dictionary system should maintain correct runtime state first.
The desktop shell should then react to a summarized mutation result.

That direction keeps editor work from shaping regeneration internals.

### 5. Add regeneration diagnostics

Useful counters would include:

- number of nodes marked for regeneration
- number of deferred regenerations replayed after unblocking
- dictionary recomputation count
- USE-to-DEF transition count
- time spent in regeneration versus dictionary maintenance

Those are the numbers contributors need in order to reason about real-world PROTO churn.

## Low-Risk Changes To Do First

These are good early improvements:

- add counters for deferred and replayed regeneration work
- log or count full dictionary recomputation paths
- document the regeneration pipeline beside the code
- make scene-tree consumers react to narrower change categories where possible

These improve understanding before deeper refactors begin.

## Review Checklist For Template-System Changes

When reviewing a PROTO, dictionary, or regeneration change, ask:

1. Does this introduce another broad recomputation path?
2. Does it widen the set of nodes that must subscribe to regeneration?
3. Does it change runtime correctness, UI behavior, or both?
4. Can the result be described as a structured change set?
5. Is there a headless validation story for the change?

If the answer to the fourth question is no, the architecture is probably still too implicit.

## Validation Guidance

After regeneration or dictionary changes, validate with:

- `python -m omnisim test-group protos`
- `python -m omnisim test-world tests/protos/worlds/template_deterministic.omniworld`
- one mutation-heavy editor workflow if desktop behavior changed

The important thing is to verify both:

- runtime correctness of PROTO and DEF/USE behavior
- whether the mutation became cheaper and more explainable than before
