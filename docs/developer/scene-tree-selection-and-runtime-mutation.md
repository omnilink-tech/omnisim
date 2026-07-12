# Scene Tree, Selection, And Runtime Mutation

This document explains why desktop-shell responsiveness is still tightly coupled to runtime mutation behavior, and what should change so the editor can stay responsive without distorting runtime ownership.

It is grounded in:

- `src/omnisim/core/WbGuiRefreshOracle.cpp`
- `src/omnisim/scene_tree/WbSceneTreeModel.cpp`
- `src/omnisim/scene_tree/WbSceneTree.cpp`

## Why This Area Matters

Many contributors think of simulator performance as:

- physics
- rendering
- sensors

But day-to-day usability is also shaped by:

- how expensive runtime-originating tree updates are
- how often selection is cleared and restored
- how broad the view invalidation is
- how much work the desktop shell performs during live simulation

That is why scene-tree behavior deserves its own architecture guidance rather than being treated as "just UI."

## What The Current Code Shows

### GUI refresh is already globally throttled

`WbGuiRefreshOracle` only allows refresh roughly every 300 ms while the simulation is running.

That confirms two things:

- the desktop shell already needs protection against overly frequent updates
- the current update path is expensive enough that coarse throttling is considered necessary

### Some value updates still trigger layout invalidation

In `WbSceneTreeModel::updateData()`:

- the code emits `dataChanged(...)`
- and then also emits `layoutChanged()`

The comment says that without the extra layout change, some values were not updated correctly when modified from the editor.

This is a strong signal that the view-model contract is broader and more fragile than it should be.

### Dictionary-sensitive mutations still trigger broad tree reactions

`WbSceneTree.cpp` checks whether a deleted node is a DEF node or has a descendant DEF node on which USE nodes depend.

When dictionary-sensitive removal happens:

- the model emits a layout change so expandable state becomes visible for nodes that changed role
- selection may be cleared again

This confirms that dictionary behavior is still directly shaping the scene-tree update strategy.

### Selection clearing is a recurring recovery mechanism

`clearSelection()` appears in several important paths:

- showing an extern PROTO panel
- row removal
- selecting a null pose
- node regeneration preparation
- dictionary-sensitive mutation handling

This means selection stability is still being protected partly by broad reset behavior.

### Runtime mutation and scene-tree recovery are still mixed

The scene tree stores and restores tree item state around regeneration, but the need to do that so often is itself a sign that the model sees changes at too coarse a grain.

## The Main Problems

### 1. The model does not cleanly distinguish structural change from value change

If a value edit needs both `dataChanged(...)` and `layoutChanged()`, the view contract is too expensive and too broad.

That has two costs:

- more repaint and relayout work
- more contributor uncertainty about the cheapest safe signal to emit

### 2. Runtime mutation storms can still become GUI storms

Template regeneration, DEF/USE updates, supervisor mutations, and field edits can all cascade into:

- model updates
- layout invalidation
- selection clearing
- state restoration

That makes the desktop shell feel more fragile than it should.

### 3. Selection identity is not stable enough

The repeated use of broad selection clearing suggests that selection is still recovered through reset-and-restore logic more often than through stable identity.

That increases both cost and risk.

### 4. The refresh throttle is global, not semantic

The current oracle answers "can the GUI refresh now?" but not:

- is this change structural or non-structural?
- can it be coalesced?
- should it be deferred differently while running versus paused?

So cheap updates and expensive updates share too much of the same policy path.

## Recommended Direction

### 1. Define explicit scene-model change categories

The desktop shell should eventually operate on typed events such as:

- value changed
- label or DEF name changed
- children inserted
- children removed
- subtree regenerated
- dictionary scope changed

That gives the model a smaller and cheaper contract than ad hoc mixtures of `dataChanged(...)` and `layoutChanged()`.

### 2. Preserve selection by stable identity where possible

Selection should ideally survive most non-structural changes through stable node identity rather than through broad clear-and-restore behavior.

This is especially important for:

- repeated field edits
- regeneration that preserves logical node identity
- supervisor-driven runtime mutations

### 3. Batch runtime-originating model work

If the runtime knows a wave of related mutations is happening, the desktop shell should consume one summarized transaction instead of many independent updates.

This will matter most for:

- template regeneration
- dictionary-sensitive changes
- subtree insert/remove operations

### 4. Separate running-mode and paused-mode policy

While paused, contributors usually want immediacy.
While running, they want bounded UI cost.

That policy should be explicit in the scene-model update layer rather than only in the global refresh oracle.

### 5. Add desktop-shell metrics

The UI side needs counters for:

- number of `layoutChanged()` emissions
- number of `dataChanged(...)` emissions
- selection clear count
- tree-state restore count
- worst-case model update duration

Without those numbers, desktop responsiveness work stays too anecdotal.

## Low-Risk Changes To Do First

These are good immediate improvements:

- count and log broad layout invalidations
- audit which value-change paths still require `layoutChanged()`
- document which scene-tree mutations are structural versus non-structural
- reduce unnecessary selection clearing in the safest obvious cases

These changes improve understanding before the deeper model refactor begins.

## Review Checklist For Scene-Tree Changes

When reviewing scene-tree or selection code, ask:

1. Is this really a structural change?
2. Can the update be expressed as `dataChanged(...)` only?
3. Is selection being cleared because identity is unstable, or because it is truly invalid?
4. Can related updates be batched?
5. Will this change fire repeatedly during a mutation-heavy scenario?

If a change adds more broad invalidation without new measurement, it should be challenged.

## Validation Guidance

After scene-tree or selection work, validate with:

- one mutation-heavy editor scenario
- one runtime mutation scenario driven by a supervisor
- paused-mode editing
- running-mode editing with live simulation

The right outcome is not only correctness. It is that the tree reacts more narrowly and more predictably than before.
