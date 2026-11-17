// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OM_CUDA_SMOKE_HPP
#define OM_CUDA_SMOKE_HPP

//
// Description: in-process M0 smoke for the CUDA compute layer. Exercises
//   OmCudaBuffer's host<->device round trip in the same ABI as omnisim-bin
//   itself — complementary to tests/cuda/cuda_smoke.cu, which validates the
//   full nvcc + kernel-launch path but lives in a separate build.
//
//   Wired in via OmApplication::setup() and gated on the
//   OMNISIM_CUDA_SMOKE=1 environment variable so it costs nothing in normal
//   runs. Result lands in omnisim_log.txt as a single INFO line plus a
//   structured-diagnostic tag for the harness to branch on.
//

namespace OmCudaSmoke {
  // Run a fixed-size buffer round trip if the CUDA context is available and
  // the OMNISIM_CUDA_SMOKE env var is set. No-op otherwise. Idempotent —
  // safe to call from any startup hook.
  void runIfRequested();

  // Returns true on success. Public so unit tests can call it directly
  // without having to set the env var.
  bool runMemcpyRoundtrip();
}

#endif
