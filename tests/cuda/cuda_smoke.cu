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

// M0 smoke test for the CUDA compute layer (physics-roadmap.md §2).
// Standalone — does not link against omnisim-bin, so it stays runnable in CI even
// when the simulator binary isn't built. Exercises the full round trip:
//   pick a device → cudaMalloc → kernel fills a constant → cudaMemcpy back →
//   verify every entry → free.
//
// Exit codes:
//   0   PASS (kernel ran on a real GPU and the round trip matched)
//   1   FAIL (any CUDA call returned non-zero, or the values mismatched)
//   77  SKIPPED (no CUDA-capable device on this machine — autotools convention)

#include <cuda_runtime.h>

#include <cstdio>
#include <cstdlib>
#include <vector>

#define CUDA_CHECK(expr)                                                                    \
  do {                                                                                      \
    cudaError_t _err = (expr);                                                              \
    if (_err != cudaSuccess) {                                                              \
      std::fprintf(stderr, "[cuda_smoke] FAIL %s:%d: %s -> %s\n", __FILE__, __LINE__, #expr, \
                   cudaGetErrorString(_err));                                               \
      std::exit(1);                                                                         \
    }                                                                                       \
  } while (0)

namespace {
  __global__ void constantFill(float *out, std::size_t n, float value) {
    const std::size_t i = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (i < n)
      out[i] = value;
  }
}  // namespace

int main(int argc, char **argv) {
  std::size_t count = 1024 * 1024;  // 1M floats = 4 MiB
  float value = 3.14159f;
  if (argc >= 2)
    count = static_cast<std::size_t>(std::atoll(argv[1]));
  if (argc >= 3)
    value = static_cast<float>(std::atof(argv[2]));

  int deviceCount = 0;
  cudaError_t err = cudaGetDeviceCount(&deviceCount);
  if (err != cudaSuccess || deviceCount <= 0) {
    std::fprintf(stderr, "[cuda_smoke] SKIP: no CUDA device (err=%d, count=%d)\n", err, deviceCount);
    return 77;
  }

  CUDA_CHECK(cudaSetDevice(0));

  cudaDeviceProp props{};
  CUDA_CHECK(cudaGetDeviceProperties(&props, 0));
  std::printf("[cuda_smoke] device 0: %s (sm_%d%d, %llu MiB)\n", props.name, props.major, props.minor,
              static_cast<unsigned long long>(props.totalGlobalMem >> 20));

  float *device = nullptr;
  CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&device), count * sizeof(float)));

  const unsigned int block = 256;
  const unsigned int grid = static_cast<unsigned int>((count + block - 1) / block);
  constantFill<<<grid, block>>>(device, count, value);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  std::vector<float> host(count, 0.0f);
  CUDA_CHECK(cudaMemcpy(host.data(), device, count * sizeof(float), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaFree(device));

  for (std::size_t i = 0; i < count; ++i) {
    if (host[i] != value) {
      std::fprintf(stderr, "[cuda_smoke] FAIL: index %zu holds %g, expected %g\n", i,
                   static_cast<double>(host[i]), static_cast<double>(value));
      return 1;
    }
  }

  std::printf("[cuda_smoke] PASS: %zu floats round-tripped through a kernel.\n", count);
  return 0;
}
