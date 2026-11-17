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

"""OmniSim cinema — agent-driven cinematic capture pipeline.

The capture service in `scripts/capture/` is the renderer: it knows how to
move an OmniSim camera to a pose and dump frames. This package is the
*director* on top: it speaks cinematic vocabulary (orbit, push-in,
tracking shot), locks onto named subjects instead of world coordinates,
applies look presets (lens + grade), assembles cuts with branding, and
emits multi-aspect deliverables.

Entry point: `python -m omnisim cinema <storyboard.json>`.
"""

from __future__ import annotations

__all__ = ["__version__"]
__version__ = "0.1.0"
