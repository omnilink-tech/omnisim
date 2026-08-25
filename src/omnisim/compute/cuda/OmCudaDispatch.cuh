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

#ifndef OM_CUDA_DISPATCH_CUH
#define OM_CUDA_DISPATCH_CUH

//
// Description: tiny launch-config wrapper for the common case of an N-wide
//   parallel-for. Kernel authors with custom shared-memory or multi-D grids
//   call CUDA directly. See docs/developer/cuda-compute-infrastructure-plan.md.
//
// This header must only be included from .cu files (it uses the <<<...>>>
// launch syntax and CUDA built-ins like blockIdx).
//

#ifndef OMNISIM_WITH_CUDA
#error "OmCudaDispatch.cuh requires OMNISIM_WITH_CUDA to be defined."
#endif

#ifndef __CUDACC__
#error "OmCudaDispatch.cuh must be included from a CUDA (.cu) translation unit."
#endif

#include "OmCudaContext.hpp"
#include "OmCudaError.hpp"

#include <QtCore/QObject>

#include <cuda_runtime.h>

#include <cstddef>

namespace OmCudaDispatch {

  // The kernel functor is passed `(std::size_t i, Args...)` for index i in
  // [0, n). Out-of-range threads are skipped inside the wrapper so the caller
  // doesn't need to do the bounds check.
  template <typename Kernel, typename... Args>
  __global__ void parallelForKernel(std::size_t n, Kernel kernel, Args... args) {
    const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i < n)
      kernel(i, args...);
  }

  // Default block size; one warp per scheduler keeps simple kernels well-fed
  // on Turing/Ampere/Ada without over-subscribing register-heavy ones.
  static constexpr unsigned int kDefaultBlockSize = 256;

  // Launches `kernel(i, args...)` for i in [0, n) on the context's stream.
  // Throws OmCudaError if the launch itself fails — a kernel runtime error
  // surfaces on the next stream synchronize() instead.
  template <typename Kernel, typename... Args>
  inline void parallelFor(std::size_t n, Kernel kernel, Args... args) {
    if (n == 0)
      return;
    OmCudaContext *ctx = OmCudaContext::instance();
    if (ctx == nullptr)
      throw OmCudaError(OmCudaCodes::NOT_AVAILABLE,
                        QObject::tr("OmCudaDispatch::parallelFor called without a CUDA context."));
    const unsigned int block = kDefaultBlockSize;
    const unsigned int grid = static_cast<unsigned int>((n + block - 1) / block);
    parallelForKernel<<<grid, block, 0, ctx->stream()>>>(n, kernel, args...);
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
      throw OmCudaError(OmCudaCodes::KERNEL_LAUNCH_FAILED,
                        QObject::tr("kernel launch failed (n=%1, grid=%2, block=%3): %4")
                          .arg(static_cast<qulonglong>(n))
                          .arg(grid)
                          .arg(block)
                          .arg(cudaGetErrorString(err)));
  }

}  // namespace OmCudaDispatch

#endif
