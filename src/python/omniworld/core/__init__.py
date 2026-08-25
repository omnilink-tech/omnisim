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

"""Core primitives that the rest of the library is built on.

- ``api`` exposes the top-level ``generate`` entry point.
- ``manifest`` emits the seed manifest that pins a generator run.
- ``recipe`` defines the Recipe protocol every biome implements.
- ``registry`` maps recipe names to classes.
- ``rng`` provides deterministic sub-seed derivation.
"""

from __future__ import annotations
