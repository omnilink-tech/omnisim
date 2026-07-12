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

#ifndef WB_CUDA_BUFFER_HPP
#define WB_CUDA_BUFFER_HPP

//
// Description: RAII wrapper around a typed device-side allocation. Throws
//   WbCudaError on cudaMalloc / memcpy failure; subsystem authors catch and
//   translate to WbLog::error. M0 only — registerGlBuffer is declared as a
//   stub that returns failure (M1 fills it in). See
//   docs/developer/cuda-compute-infrastructure-plan.md.
//
// This header is *only* included from translation units compiled with
// OMNISIM_WITH_CUDA defined. Subsystems that need a CPU fallback path must
// guard their includes the same way.
//

#ifndef OMNISIM_WITH_CUDA
#error "WbCudaBuffer.hpp requires OMNISIM_WITH_CUDA to be defined."
#endif

#include "WbCudaContext.hpp"
#include "WbCudaError.hpp"

#include <QtCore/QObject>
#include <QtCore/QString>

#include <cuda_runtime.h>

#include <cstddef>
#include <utility>

template <typename T>
class WbCudaBuffer {
public:
  explicit WbCudaBuffer(std::size_t count) : mDevice(nullptr), mCount(count) {
    if (count == 0)
      return;
    cudaError_t err = cudaMalloc(reinterpret_cast<void **>(&mDevice), count * sizeof(T));
    if (err != cudaSuccess) {
      mDevice = nullptr;
      throw WbCudaError(err == cudaErrorMemoryAllocation ? WbCudaCodes::OUT_OF_MEMORY :
                                                           WbCudaCodes::DEVICE_INIT_FAILED,
                        QObject::tr("cudaMalloc(%1 bytes) failed: %2")
                          .arg(static_cast<qulonglong>(count * sizeof(T)))
                          .arg(cudaGetErrorString(err)));
    }
  }

  ~WbCudaBuffer() {
    if (mDevice != nullptr)
      cudaFree(mDevice);
  }

  WbCudaBuffer(const WbCudaBuffer &) = delete;
  WbCudaBuffer &operator=(const WbCudaBuffer &) = delete;

  WbCudaBuffer(WbCudaBuffer &&other) noexcept : mDevice(other.mDevice), mCount(other.mCount) {
    other.mDevice = nullptr;
    other.mCount = 0;
  }
  WbCudaBuffer &operator=(WbCudaBuffer &&other) noexcept {
    if (this != &other) {
      if (mDevice != nullptr)
        cudaFree(mDevice);
      mDevice = other.mDevice;
      mCount = other.mCount;
      other.mDevice = nullptr;
      other.mCount = 0;
    }
    return *this;
  }

  T *device() noexcept { return mDevice; }
  const T *device() const noexcept { return mDevice; }
  std::size_t count() const noexcept { return mCount; }
  std::size_t bytes() const noexcept { return mCount * sizeof(T); }

  void copyFromHost(const T *host, std::size_t n) {
    if (n == 0)
      return;
    if (n > mCount)
      throw WbCudaError(WbCudaCodes::MEMCPY_FAILED,
                        QObject::tr("copyFromHost: requested %1 elements, buffer holds %2.")
                          .arg(static_cast<qulonglong>(n))
                          .arg(static_cast<qulonglong>(mCount)));
    cudaError_t err = cudaMemcpy(mDevice, host, n * sizeof(T), cudaMemcpyHostToDevice);
    if (err != cudaSuccess)
      throw WbCudaError(WbCudaCodes::MEMCPY_FAILED,
                        QObject::tr("cudaMemcpy(host -> device, %1 bytes) failed: %2")
                          .arg(static_cast<qulonglong>(n * sizeof(T)))
                          .arg(cudaGetErrorString(err)));
  }

  void copyToHost(T *host, std::size_t n) const {
    if (n == 0)
      return;
    if (n > mCount)
      throw WbCudaError(WbCudaCodes::MEMCPY_FAILED,
                        QObject::tr("copyToHost: requested %1 elements, buffer holds %2.")
                          .arg(static_cast<qulonglong>(n))
                          .arg(static_cast<qulonglong>(mCount)));
    cudaError_t err = cudaMemcpy(host, mDevice, n * sizeof(T), cudaMemcpyDeviceToHost);
    if (err != cudaSuccess)
      throw WbCudaError(WbCudaCodes::MEMCPY_FAILED,
                        QObject::tr("cudaMemcpy(device -> host, %1 bytes) failed: %2")
                          .arg(static_cast<qulonglong>(n * sizeof(T)))
                          .arg(cudaGetErrorString(err)));
  }

  // GL/CUDA interop — declared in M0 to lock the public API surface, fully
  // implemented in M1. The stub throws so callers see the deferred state via
  // a structured CUDA_GL_INTEROP_NOT_IMPLEMENTED diagnostic rather than a
  // compile error.
  static WbCudaBuffer<T> registerGlBuffer(unsigned int /*glBuffer*/, std::size_t /*count*/) {
    throw WbCudaError(WbCudaCodes::GL_INTEROP_NOT_IMPLEMENTED,
                      QObject::tr("WbCudaBuffer::registerGlBuffer is a M1 stub; "
                                  "GL/CUDA interop is not implemented yet."));
  }

private:
  T *mDevice;
  std::size_t mCount;
};

#endif
