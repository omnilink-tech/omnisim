# Header Hygiene And Rebuild Reduction

This document explains how to reduce rebuild scope in the current make-based codebase without waiting for a full build-system migration.

It is aimed at the most common expensive failure mode in OmniSim development:

- a small code change touches a broad header
- make invalidates a large fraction of `src/omnisim`
- the developer waits through a mostly unrelated rebuild

The first defense against slow iteration is not a new compiler flag. It is dependency discipline.

## Why Small Changes Rebuild Too Much

The simulator is still organized around a broad monolithic product build. That makes a few patterns especially expensive:

- high-fanout headers in `src/omnisim`
- inline code inside headers that pulls heavy includes everywhere
- node-level headers depending on GUI, rendering, or asset-system types
- utility headers that quietly become "include this from anywhere" dumping grounds

If one commonly included header changes, the build cost is often dominated by recompiling downstream translation units rather than by the logic you actually changed.

## High-Risk Headers

Treat these headers as rebuild amplifiers and touch them carefully:

- `src/omnisim/vrml/WbNode.hpp`
- `src/omnisim/nodes/WbBaseNode.hpp`
- `src/omnisim/nodes/utils/WbWorld.hpp`
- `src/omnisim/nodes/WbSolid.hpp`
- `src/omnisim/engine/WbSimulationWorld.hpp`
- `src/omnisim/control/WbController.hpp`

This does not mean "never edit them." It means:

- avoid adding heavy includes unless absolutely necessary
- avoid adding new inline logic
- avoid widening their public surface when a narrower helper would work

## Core Rules

### Prefer forward declarations in headers

If a header only stores a pointer or reference, forward declare the type and include the real header in the `.cpp`.

Good:

- `class WbNode;`
- `class QImage;`
- `class WbWrenTextureOverlay;`

Bad:

- including the full renderer, Qt widget, or parser header in a broadly used node header only to store a pointer

### Keep heavy framework includes in `.cpp` files

Try hard to keep these out of lower-level headers:

- `QtWidgets`
- `QtOpenGL`
- WREN-specific headers
- ODE-specific implementation details
- asset-decoding helpers

Parser, runtime, and node headers should not depend on desktop or renderer internals unless that header is already a true leaf.

### Do not hide work in inline methods

Inline code in a high-fanout header is expensive in two ways:

- it forces more recompilation when the implementation changes
- it increases compile time for every translation unit that includes it

If a method is not obviously trivial, move it into the `.cpp`.

### Separate stable interfaces from volatile implementation details

If a class is widely used but one area of behavior changes often, extract that behavior into:

- a private implementation helper in the `.cpp`
- a narrow strategy/service class
- a renderer-only or desktop-only adapter

This keeps frequent churn out of rebuild-amplifying headers.

## Patterns That Usually Help

### Pattern 1: move convenience includes downward

Common mistake:

- a header includes a second header "for convenience"
- dozens of downstream files start relying on that accidental transitive include
- any change to the second header now rebuilds a large surface

Preferred fix:

- include only what the header itself requires
- add missing includes directly to each `.cpp` or leaf header that truly needs them

This may look noisier at first, but it reduces accidental coupling.

### Pattern 2: split data ownership from expensive behavior

If a node or world class owns a small piece of state but one method needs heavy rendering or asset logic:

- keep the state in the main class
- move the heavy behavior into a helper implementation unit
- pass only the narrow data needed across the boundary

This is especially valuable in texture, camera, sensor, and background code.

### Pattern 3: add a narrow facade instead of a broad include

If a subsystem needs one query from another subsystem, do not include the world.

Prefer:

- one method on a narrow service
- one small data struct
- one callback boundary

Over:

- including a large manager or singleton header everywhere

### Pattern 4: prefer local static helpers over utility-header growth

Many "small" helper methods do not need to become globally shared headers. If the helper is tightly coupled to one translation unit, keep it there.

This avoids building another general-purpose dependency surface that later becomes impossible to narrow.

## Specific Pressure Points In OmniSim

### Node headers that pull rendering or GUI policy

`WbImageTexture` and related classes sit near a bad boundary today: node semantics, asset loading, and renderer upload behavior are tightly coupled.

Guideline:

- keep URL/state semantics in node-facing headers
- keep decoding, upload, and renderer object management in implementation files or renderer-side helpers

### World and runtime headers that absorb editor needs

When a runtime-facing header grows because editor or scene-tree code wants direct access, rebuild scope gets worse and architecture gets blurrier at the same time.

Guideline:

- prefer read-only adapters or event translation layers for desktop tools
- avoid adding scene-tree or editor-specific concepts to runtime headers

### Utility headers that become global dumping grounds

Files in `nodes/utils` and `core` can quietly become high-fanout choke points because everyone feels entitled to include them.

Guideline:

- before adding a helper, ask whether it is really cross-cutting
- if it is used by one subsystem family, keep it there
- if it needs a broad home, keep the header surface minimal and the implementation out of line

## How To Review A Change For Rebuild Impact

Use this checklist before merging:

1. Did the change touch a high-fanout header?
2. Did the header gain new includes?
3. Could one or more of those includes become forward declarations?
4. Did any new inline methods or templates appear in a broad header?
5. Did a lower-level header gain a GUI, renderer, or editor dependency?
6. Can the same behavior be implemented in a `.cpp` or adapter instead?

If the answer to any of the middle questions is yes, keep tightening the change.

## Practical Refactors That Pay Off Quickly

These are the kinds of changes that usually improve build speed without major behavior risk:

- move Qt GUI includes out of lower-level headers into implementation files
- replace transitive includes with direct local includes in leaf files
- extract renderer-only helpers from node headers
- move rarely used utility methods from headers into `.cpp`
- replace broad singleton header use with narrower query interfaces

## Validation Strategy

After a rebuild-scope refactor:

1. regenerate or verify `compile_commands.json`
2. run the narrowest relevant build target such as `make sim-core`, `make renderer`, or `make sim-gui`
3. run one smoke scenario that touches the changed code path
4. compare rebuild scope informally on the next small edit to confirm the header blast radius really dropped

The fourth step matters. A refactor only counts as successful if the next nearby edit becomes cheaper.

## Anti-Patterns To Reject In Review

Reject or rework changes that:

- add `QtWidgets` or scene-tree includes to runtime-facing headers
- add WREN implementation headers to parser or world headers
- expand a global utility header because "it is already included everywhere"
- add non-trivial inline methods to common node or world headers
- rely on accidental transitive includes

These are the same patterns that make coding-agent iteration slow, because they obscure ownership and inflate compile scope.

## Long-Term Direction

Header hygiene is not separate from architecture work. It is how the architecture becomes real.

If we keep reducing include blast radius, three future outcomes become much easier:

- a true `simulation core` target
- targeted renderer builds and validation
- repo guidance that tells agents exactly which files they can edit without triggering a full rebuild

Until then, every include is a build decision.
