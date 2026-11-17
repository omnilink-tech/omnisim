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

"""One-shot screenshot supervisor for headless sky-rendering benches.

Reads OMNISIM_SKY_RENDER_OUTPUT (an absolute PNG path) and
OMNISIM_SKY_RENDER_SETTLE (settle-step count, default 24).

On startup: steps the sim N times so the procedural-sky bake settles,
exports the main viewport to the PNG, then quits Webots.

This deliberately does NOT mutate the scene at runtime, because
OmBackground.applySkyBoxToWren is not safe to re-enter — the IBL bake
goes off-spec on the second call (multi-scatter contamination → green
ground, gray sky).  Bake-per-shot worlds are templated by
scripts/bench_sky.py instead.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from omnisim import Supervisor


def main():
    out_path = os.environ.get("OMNISIM_SKY_RENDER_OUTPUT")
    if not out_path:
        print("[sky_render] OMNISIM_SKY_RENDER_OUTPUT not set — nothing to do", flush=True)
        return
    settle = int(os.environ.get("OMNISIM_SKY_RENDER_SETTLE", "24"))

    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    for _ in range(settle):
        if supervisor.step(time_step) == -1:
            return

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    supervisor.exportImage(str(out), 100)
    print(f"[sky_render] -> {out}", flush=True)

    # exportImage is asynchronous: it queues a screenshotRequested signal
    # that fires on the next View3D::refresh, which triggers renderNow,
    # which emits screenshotReady, which writes the PNG.  Step many
    # times to let the Qt event loop drain that pipeline before we quit.
    for _ in range(40):
        if supervisor.step(time_step) == -1:
            return
    supervisor.simulationQuit(0)


if __name__ == "__main__":
    main()
