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

"""The OmniSim in-process core.

The engine compiles a world into a MuJoCo model. This package steps that exact
model in ONE Python process, with the engine's own per-tick rules reproduced,
and runs the world's unmodified controllers in the same process as plain
function calls. No per-step message passing, no second process.

It is a fast path for worlds it has been checked against, not a second
simulator: the engine stays the compiler (one short headless load produces the
model, the starting state and the clamp tables) and the reference. Anything
the core does not model raises `CoreUnsupported` instead of being ignored, and
callers fall back to the engine.

    python -m omnisim core run    <world>     run it on the core
    python -m omnisim core record <world>     run it and keep its timeline (docs/developer/core-timeline.md)
"""


from .errors import CoreDeclined, CoreUnsupported

__version__ = "0.1.0"
__all__ = ["CoreDeclined", "CoreUnsupported", "__version__"]
