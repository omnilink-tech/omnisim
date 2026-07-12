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

"""Deterministic sub-seed derivation.

Every primitive in the library takes an explicit ``random.Random``. No
primitive reads the global RNG or the wall clock. To keep sub-systems
independent without letting them stomp on each other's stream, each
sub-system asks for a seed derived from a stable name, and builds its own
Random instance from it.

    parent_seed = 42
    heightmap_rng = rng_from_seed(derive_seed(parent_seed, "heightmap"))
    scatter_rng   = rng_from_seed(derive_seed(parent_seed, "scatter.rocks"))

Derivation is stateless (blake2b keyed by the parent seed). That gives:

- Same parent seed + same name => same child seed, always.
- Adding a new named consumer does NOT perturb existing consumers, because
  each child is a pure function of (parent_seed, name).
- Cross-platform determinism (unlike ``hash(str)``, which is salted by
  ``PYTHONHASHSEED``).
"""

from __future__ import annotations

import hashlib
import random


def rng_from_seed(seed: int) -> random.Random:
    """Return a fresh ``random.Random`` seeded deterministically."""
    return random.Random(seed)


def derive_seed(parent_seed: int, name: str) -> int:
    """Derive a 64-bit child seed from (parent_seed, name).

    Pure function — no hidden state, no wall clock, no global RNG.
    """
    key = (parent_seed & 0xFFFFFFFFFFFFFFFF).to_bytes(8, "little", signed=False)
    digest = hashlib.blake2b(name.encode("utf-8"), digest_size=8, key=key).digest()
    return int.from_bytes(digest, "little", signed=False)


def derive(parent_seed: int, name: str) -> random.Random:
    """Convenience: derive a child seed and wrap it in ``random.Random``."""
    return rng_from_seed(derive_seed(parent_seed, name))
