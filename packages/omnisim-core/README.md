# omnisim-core

The OmniSim in-process core: the engine's compiled MuJoCo model of a world, stepped in one Python
process together with the world's unmodified controllers.

- The **engine is the compiler**: one short headless load saves the model, the starting state and the
  joint-clamp tables (`scripts/core/compile_hook.py`). The core never rebuilds physics from a world file.
- The **core steps it** with the engine's own per-tick rules reproduced (`omnisim_core/core.py`) and
  serves the controller API as plain function calls. A call it does not model raises `CoreUnsupported`.

```bash
python -m omnisim core run    <world> --seconds 10 --lenient
```


Time control on top of it (`omnisim_core/timeline.py`): `core record`, `replay`, `inspect`, `events`, `branch`, `diff`.
A recorded run replays bit for bit, opens at any tick, and can be stepped again from any tick with a change.
Guide: [docs/developer/core-timeline.md](../../docs/developer/core-timeline.md).

It needs the engine's physics runtime bundle (`python -m omnisim doctor`), `greenlet` and `numba`.
`mujoco` and `numpy` are deliberately taken from the bundle, not from the environment: the core must
step with the same build the engine ran.

Tests: `tests/test_omnisim_core_stepping.py` (the stepping rules on the engine's MuJoCo build),
`tests/test_omnisim_core_timeline.py`.
