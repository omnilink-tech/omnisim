# OmniSim Architecture

This document is the short, implementation-facing architecture map for contributors and coding agents. It is not a full design document. Its purpose is to answer two questions quickly:

1. Where does a change belong?
2. What else is likely to move when I touch that area?

For deeper follow-up, see:

- [runtime-hotspots.md](runtime-hotspots.md)
- [rendering-and-visual-quality.md](rendering-and-visual-quality.md)
- [build-and-iteration.md](build-and-iteration.md)

## Product Layers

### Desktop shell
Main directories:
- `src/omnisim/gui`
- `src/omnisim/scene_tree`
- `src/omnisim/editor`
- `src/omnisim/widgets`
- parts of `src/omnisim/user_commands`

Responsibilities:
- application startup and task routing
- main window and menus
- scene tree and property editing
- project/world editing UX
- dialogs, wizards, and desktop-only workflows

Entry points:
- `src/omnisim/gui/main.cpp`
- `src/omnisim/gui/OmGuiApplication.*`
- `src/omnisim/gui/OmMainWindow.*`

When to edit here:
- menu behavior
- window/layout behavior
- editor or scene tree behavior
- desktop interaction workflows

### Simulation runtime
Main directories:
- `src/omnisim/engine`
- `src/omnisim/control`
- relevant parts of `src/omnisim/nodes`
- `src/omnisim/physics`

Responsibilities:
- stepping the simulation
- physics stepping
- controller lifecycle and synchronization
- runtime reset/pause/mode control

Key classes:
- `OmWorld`
- `OmSimulationWorld`
- `OmControlledWorld`
- `OmController`

When to edit here:
- simulation step behavior
- timing/synchronization
- controller orchestration
- physics/runtime state handling

### World model and parsing
Main directories:
- `src/omnisim/vrml`
- `src/omnisim/nodes`
- `src/omnisim/nodes/utils`

Responsibilities:
- tokenization and parsing
- node instantiation and finalization
- PROTO and EXTERNPROTO handling
- template regeneration
- save/export/reset behavior

Key classes:
- `OmTokenizer`
- `OmParser`
- `OmNodeReader`
- `OmNode`
- `OmProtoManager`
- `OmTemplateManager`
- `OmWorld`

When to edit here:
- world loading
- PROTO behavior
- field handling
- serialization/export
- node semantics

### Rendering
Main directories:
- `src/wren`
- `src/omnisim/wren`
- `src/omnisim/render`
- parts of `src/omnisim/gui`

Responsibilities:
- renderer object model and GPU resources
- rendering context and passes
- sensor rendering and viewport rendering
- post-processing and picking

Key classes:
- Wren texture/material/frame-buffer/scene classes in `src/wren`
- `OmWrenRenderingContext`
- `OmWrenCamera`
- `OmView3D`

When to edit here:
- rendering performance
- visual effects
- sensor rendering
- OpenGL/Wren integration

### Controller APIs and protocol
Main directories:
- `src/controller/c`
- `src/controller/cpp`
- `src/controller/java`
- `src/controller/launcher`
- `include/controller`
- `src/omnisim/control`

Responsibilities:
- controller runtime protocol
- controller launcher behavior
- C/C++/Java controller APIs
- IPC/TCP controller connectivity

When to edit here:
- controller API additions
- controller protocol behavior
- extern controller startup/connectivity
- supervisor API/runtime packets

## Build Reality

The current codebase is still built as a large make-driven product. The logical build targets added for faster workflows are wrappers over the existing structure, not a fully modularized build graph.

Useful wrapper targets:
- `make sim-core`
- `make sim-gui`
- `make renderer`
- `make controller-libs`
- `make tests-smoke`
- `make package`

Developer CLI:
- `python scripts/dev/omnisim_dev.py build core`
- `python scripts/dev/omnisim_dev.py build renderer`
- `python scripts/dev/omnisim_dev.py test-smoke`
- `python scripts/dev/omnisim_dev.py test-world <path>`
- `python scripts/dev/omnisim_dev.py run-headless <path>`
- `python scripts/dev/omnisim_dev.py profile-world <path>`

## Safe Validation Paths

### Renderer changes
- `python scripts/dev/omnisim_dev.py build renderer`
- `python scripts/dev/omnisim_dev.py test-world tests/rendering/worlds/normals.omniworld --nomake`

### Runtime or node changes
- `python scripts/dev/omnisim_dev.py build core`
- `python scripts/dev/omnisim_dev.py test-smoke --nomake`

### GUI/editor changes
- `python scripts/dev/omnisim_dev.py build gui`
- launch one small world locally

### Headless automation or agent-driven scenarios
- `python scripts/dev/omnisim_dev.py run-headless <world>`
- Supported headless flags come from the existing simulator CLI: `--mode=fast --batch --no-rendering --stdout --stderr --minimize`

### Controller API changes
- `python scripts/dev/omnisim_dev.py build controller-libs`
- `python scripts/dev/omnisim_dev.py test-group api --nomake`

## Constraints

- `src/omnisim` is still a product-style target, so many changes have wider rebuild scope than ideal.
- The runtime is not yet cleanly separated from the desktop shell.
- The smoke and benchmark runners are intentionally small and do not replace full regression.
