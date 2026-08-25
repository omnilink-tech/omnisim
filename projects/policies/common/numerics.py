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

"""Deterministic numerics for DEPLOY-side policy inference.

Deploy policies run on CPU torch (loaded with map_location="cpu" -- see
g1_walk_recipe's deploy init). Bit-stability of that inference across runs and
machines is then decided by three things this module pins:

  * thread count -- multi-threaded CPU reductions sum in nondeterministic
    order; a 23-joint LSTM gains nothing from >1 thread, so pin to 1;
  * kernel selection -- torch.use_deterministic_algorithms() forces the
    deterministic implementation where one exists (warn_only: never fatal);
  * TF32 -- a no-op on CPU today, but pinned OFF so that if anyone ever moves
    inference to CUDA it does not silently pick up GPU-generation-dependent
    reduced-precision matmuls (TF32 on Ampere+ vs fp32 elsewhere is a real
    cross-machine gait difference).

What this module can NOT pin: the torch *version* itself. On Windows the
engine's _pth resolves torch from the SYSTEM site-packages (it is deliberately
not in the deploy bundle), so version parity across machines is a fingerprint
concern -- env_fingerprint tags such packages [sys]; match them across machines.

TRAINING deliberately does not call this: the trainer wants TF32/multithread
throughput, and its cross-seed variance is handled by domain randomization,
not bitwise parity.

Escape hatch: OMNISIM_LOOSE_NUMERICS=1 skips all pinning (for A/B triage).
"""
from __future__ import annotations

import os


def pin_deploy_numerics(log=None) -> bool:
    """Pin torch for reproducible deploy inference. Returns True if pinned.

    Never raises: a torch too old for a knob just skips it (warn via log).
    """
    if os.environ.get("OMNISIM_LOOSE_NUMERICS", "0") not in ("0", "", "false"):
        if log:
            log("numerics: OMNISIM_LOOSE_NUMERICS=1 -- deploy inference NOT pinned")
        return False
    try:
        import torch
    except Exception:
        return False
    applied = []
    for name, fn in (
        ("threads=1", lambda: torch.set_num_threads(1)),
        ("interop=1", lambda: torch.set_num_interop_threads(1)),
        ("deterministic", lambda: torch.use_deterministic_algorithms(True, warn_only=True)),
        ("tf32-off", lambda: (
            setattr(torch.backends.cuda.matmul, "allow_tf32", False),
            setattr(torch.backends.cudnn, "allow_tf32", False))),
    ):
        try:
            fn()
            applied.append(name)
        except Exception:
            pass  # e.g. interop must be set before parallel work begins
    if log:
        log("numerics: deploy inference pinned (%s) torch=%s"
            % (", ".join(applied) or "nothing applied",
               getattr(torch, "__version__", "?")))
    return True
