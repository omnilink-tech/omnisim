# Upgrading OmniSim

OmniSim does issue numbered releases — `v8.0.0` through `v8.1.6` are tagged, and
the root [`CHANGELOG.md`](https://github.com/omnilink-tech/omnisim/blob/main/CHANGELOG.md)
is the authoritative record of what changed in each. Read it first; this page
only collects the migrations that need more than a changelog line.

### Breaking changes you are most likely to hit

- **The Python module is `omnisim`.** The `controller` alias was deleted on
  2026-08-16, so `from controller import Robot` raises `ModuleNotFoundError`.
  One line per controller: `controller` → `omnisim`.
- **The C++ include path and namespace moved**, same date: `<webots/Robot.hpp>`
  → `<omnisim/Robot.hpp>`, and `namespace webots` → `namespace omnisim`. The
  namespace is part of every mangled symbol, so **every C++ controller must be
  recompiled**, not just re-edited. The **C** API is unaffected — it is
  `wb_*`-prefixed, no symbol moved, and C controllers keep working as built.
- **Worlds are `.omniworld`.** `.wbt` is still read, forever; it is simply never
  written. Nothing you own has to be renamed.
- **ODE is gone** (2026-08-08, `bdc02139`). Newton/MuJoCo is the only backend.
  `Fluid` / `ImmersionProperties` and the physics-plugin API were removed with
  it, and `Solid.immersionProperties` is now an unknown field — an ERROR, not a
  warning, so a legacy world declaring it fails a headless run.
- **WREN is gone** (2026-08-23, `976b9449d`). wgpu-native is the only renderer;
  `renderBackend "wren"` parses but renders wgpu. See
  [System Requirements](system-requirements.md).

For the inherited pre-fork history, see the upstream Webots
[changelog](../reference/changelog.md).
