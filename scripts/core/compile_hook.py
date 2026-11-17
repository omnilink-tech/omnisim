# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The engine as the compiler of the in-process core (packages/omnisim-core).

Runs INSIDE the engine's physics runtime, once per tick, when the engine is
started with OMNISIM_INENGINE_PYMOD=scripts.core.compile_hook:tick. It changes
nothing about the run. At the engine's first physics step it saves the exact
MuJoCo model and the state that step starts from; one tick later it adds the
engine's joint-clamp tables and substep count. The core then steps that model
itself (omnisim_core.compile drives this; never set the variables by hand).
"""

import json
import os

_S = {}


def _out(name):
    # OMNISIM_CORE_COMPILE_OUT: the directory the compiled world is written into.
    return os.path.join(os.environ.get("OMNISIM_CORE_COMPILE_OUT", "."), name)


def tick(world):
    if _S.get("w") is world:
        if _S.get("n", 0) == 1 and not _S.get("clamp_done"):
            _save_clamp(world)
        _S["n"] = _S.get("n", 0) + 1
        return
    import mujoco
    import numpy as np
    _S.clear()
    _S["w"], _S["n"] = world, 1
    sv = getattr(world, "solver", None)
    if sv is None or getattr(sv, "mj_data", None) is None or not getattr(sv, "use_mujoco_cpu", False):
        with open(_out("engine.json"), "w", encoding="utf-8") as f:
            json.dump({"ok": False, "why": "this world does not run on the CPU MuJoCo solver"}, f)
        return
    spec = int(mujoco.mjtState.mjSTATE_FULLPHYSICS) | int(mujoco.mjtState.mjSTATE_WARMSTART)
    orig = mujoco.mj_step

    def once(m, d, *a, **k):
        if d is world.solver.mj_data and not _S.get("saved"):
            _S["saved"] = True
            mujoco.mj_saveModel(m, _out("model.mjb"), None)
            s = np.zeros(mujoco.mj_stateSize(m, spec))
            mujoco.mj_getState(m, d, s, spec)
            np.save(_out("state0.npy"), s)
            mujoco.mj_step = orig
        return orig(m, d, *a, **k)

    mujoco.mj_step = once


def _save_clamp(world):
    import numpy as np
    _S["clamp_done"] = True
    fc = getattr(world, "_fastclamp", None)
    slk = getattr(world, "_joint_clamp_slack", (0.05, 0.005))
    info = {"ok": bool(_S.get("saved")), "substeps": int(getattr(world, "_n_substeps", 1) or 1),
            "slack": [float(slk[0]), float(slk[1])], "has_clamp": fc is not None}
    if not info["ok"]:
        info["why"] = "the engine did not reach a physics step on the CPU MuJoCo solver"
    if fc is not None:
        np.savez(_out("clamp.npz"), qi=fc[0], qdi=fc[1], vlim=fc[2], lo=fc[3], hi=fc[4])
    with open(_out("engine.json"), "w", encoding="utf-8") as f:
        json.dump(info, f)
