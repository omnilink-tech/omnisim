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

"""Biome recipes.

Importing this package self-registers every shipped recipe in the core
registry. Adding a new biome means:

1. Create ``biomes/<name>.py`` that defines a ``Recipe``-conformant
   object and calls ``register_recipe(...)`` at module scope.
2. Import it here.

Tier 1 delivers: ``flat_ground`` (stub, always available),
``outdoor_forest`` (lands with T1.2/T1.3/T1.6), plus four more biomes.
"""

from __future__ import annotations

from . import flat_ground  # noqa: F401  (import for side effect: registration)
from . import outdoor_forest  # noqa: F401
from . import outdoor_desert  # noqa: F401
from . import warehouse  # noqa: F401
from . import urban_block  # noqa: F401
from . import indoor_apartment  # noqa: F401
from . import mars  # noqa: F401
