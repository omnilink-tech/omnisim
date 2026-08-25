# Editor And GUI Responsiveness

This document focuses on the desktop-shell side of performance: scene-tree updates, editor responsiveness, selection stability, and how runtime mutations should interact with the GUI.

It matters because a simulator can be fast in headless stepping and still feel slow or fragile in day-to-day use if the desktop shell does too much work on every world change.

## What The Current Code Suggests

Two files are especially important here:

- `src/omnisim/core/OmGuiRefreshOracle.cpp`
- `src/omnisim/scene_tree/OmSceneTreeModel.cpp`

`OmGuiRefreshOracle` throttles GUI refreshes while the simulation is running. It only allows refresh roughly every 300 ms when the simulator is not paused.

That is a useful guardrail, but it is also a warning sign: the GUI update path is expensive enough that the code already has to protect itself against over-refreshing.

`OmSceneTreeModel` also mixes fine-grained `dataChanged(...)` notifications with broader `layoutChanged()` emissions. Broad layout changes are much more disruptive because they can invalidate more UI state and trigger more view work than a targeted data update.

## The Main Responsiveness Risks

### 1. Runtime mutation storms become GUI storms

If one runtime action causes many node, field, or dictionary updates, the desktop shell may end up doing:

- repeated model updates
- repeated selection bookkeeping
- repeated layout recalculation
- repeated scrolling and expansion restoration work

Even when rendering is fast, that can make the editor feel sluggish.

### 2. Dictionary and regeneration work disturbs selection state

The scene-tree code already has to clear and restore selection in several regeneration and dictionary-update paths.

That means there is still too much coupling between:

- runtime graph mutation
- dictionary maintenance
- tree-model structure
- selection state

The behavior may be correct, but it is fragile and expensive.

### 3. The GUI refresh budget is global, not subsystem-aware

The current refresh oracle is a coarse gate. It does not distinguish between:

- a tiny label change
- a large subtree regeneration
- a scene-tree structural rewrite

That means inexpensive updates may wait behind expensive ones, and expensive updates may still be too broad when they do happen.

## Design Rules For Future GUI Work

### Prefer value updates over layout updates

Use `dataChanged(...)` for changes that do not alter tree structure.

Reserve `layoutChanged()` or model reset behavior for true structural changes such as:

- child insertion or removal
- node regeneration that really changes ancestry or ordering
- switching between different tree views of the same data

If a change only alters a label, icon, or field value, a broad layout invalidation is usually the wrong tool.

### Batch runtime-originating GUI updates

If a runtime operation is known to change many related items:

- collect the change set
- coalesce updates
- publish one structured UI transaction instead of many immediate signals

This matters for:

- template regeneration
- dictionary updates
- import or paste operations
- supervisor-driven world mutations

### Keep selection recovery out of the hot path

Selection preservation is important, but it should not dominate mutation cost.

Prefer:

- stable item identity
- targeted invalidation
- restore-by-identity when structural change is unavoidable

Over:

- clearing selection broadly and reconstructing it from scratch for minor updates

### Treat the scene tree as a projection, not the source of truth

Runtime ownership should stay in the world and node graph. The scene tree should adapt to runtime events, not shape runtime mutation semantics.

This keeps desktop concerns from bleeding back into simulation code.

## Practical Improvements Worth Making

### Add clearer event types for tree updates

Instead of letting many code paths choose between ad hoc data or layout notifications, define explicit event categories such as:

- value changed
- node renamed
- subtree inserted
- subtree removed
- subtree regenerated

That makes it easier to map runtime changes to the cheapest safe UI update.

### Coalesce dictionary-triggered model work

Dictionary updates can cascade into selection handling and tree rebuild behavior. A coalescing layer would allow the runtime to finish its work first and then let the desktop shell process one summarized update.

### Separate paused-mode and running-mode update policy

The right refresh behavior while paused is different from the right behavior during live stepping.

Paused mode can favor immediacy.
Running mode should favor bounded update frequency and minimal invalidation.

That policy should be explicit rather than spread across individual UI call sites.

### Instrument the desktop shell

The runtime already has performance logging. The desktop shell needs lighter but still useful observability for:

- scene-tree update counts
- number of layout changes per second
- selection restore cost
- worst-case UI transaction time

Without that, "the editor feels slow" remains hard to diagnose.

## Review Checklist For GUI Changes

When changing editor, scene-tree, or GUI glue code, ask:

1. Is this a structural change or just a value change?
2. Can this update be delayed or coalesced while the simulation is running?
3. Does this runtime change really need to know about selection or tree layout?
4. Can identity be preserved so the view does less restoration work?
5. Will this signal fire once or many times in a mutation-heavy scenario?

If the change adds more broad invalidation, it needs strong justification.

## Validation Scenarios

Use these scenarios after GUI responsiveness changes:

- import or paste a moderately large subtree and watch scene-tree responsiveness
- rename nodes or DEF names repeatedly and confirm the tree updates stay targeted
- run a world while a supervisor makes runtime mutations and confirm the desktop shell remains responsive
- switch between paused and running mode and compare update behavior

These are important because GUI performance bugs often hide from headless smoke tests.

## Long-Term Direction

The best desktop-shell architecture for OmniSim has these properties:

- runtime code emits structured world-change events
- the desktop shell consumes those events through adapters
- scene-tree updates are coalesced and minimally invalidating
- selection state is preserved by stable identity rather than by broad reset logic

That is how the simulator becomes both faster and easier to extend without fear of breaking the editor.
