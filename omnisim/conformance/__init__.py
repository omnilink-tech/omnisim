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

"""OmniSim install-conformance self-test.

Phase 0-1 (this package today): a **non-gating reporter**,
``python -m omnisim verify-install``. It captures a config fingerprint, runs a
small acceptance manifest of leading demos, compares each measured metric to a
calibrated tolerance band, and reports PASS / PASS-WITH-DRIFT / FAIL — without
ever blocking a launch.

The full design (the gating roadmap, the deep lane, the report bundle, the
remediation advisor) lives in ``docs/developer/install-conformance.md``.

Key honesty constraint baked in from the design: bit-exact cross-machine
reproduction is **not** feasible (GPU/CUDA ordering, SIMD/FMA, Newton-vs-ODE).
So only a handful of genuinely bit-exact canaries + binary/load liveness are
HARD asserts; everything physical is a SOFT band whose miss is *drift*, never
*broken*.
"""

from .fingerprint import collect as collect_fingerprint
from .fingerprint import fingerprint_id, resolved_backend

__all__ = ["collect_fingerprint", "fingerprint_id", "resolved_backend"]
