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

#ifndef OM_CUDA_CONTEXT_HPP
#define OM_CUDA_CONTEXT_HPP

//
// Description: lazy singleton owning the CUDA runtime initialization for
//   OmniSim. Picks one device and exposes one stream. Other compute
//   subsystems (granular, future MPM/SPH, batched sensors) borrow the stream
//   from here rather than initializing CUDA themselves. See
//   docs/developer/cuda-compute-infrastructure-plan.md.
//

#include <QtCore/QString>

#include <cstdint>

#ifdef OMNISIM_WITH_CUDA
struct cudaDeviceProp;
typedef struct CUstream_st *cudaStream_t;
#endif

class OmCudaContext {
public:
  // Returns the singleton, or null if CUDA is unavailable on this build/box.
  // Calling this triggers lazy initialization on first use; subsequent calls
  // are cheap. Initialization failures emit OmLog::error with a CUDA_* code
  // and cause this to return null.
  static OmCudaContext *instance();

  // True when the build was compiled with CUDA support AND a usable device
  // was found at startup. Subsystems should branch on this before allocating
  // any GPU resources.
  static bool available();

  // 0-based device index that was selected. Honors $OMNISIM_CUDA_DEVICE.
  int deviceId() const { return mDeviceId; }

  // Human-readable device name and "<major>.<minor>" compute capability,
  // valid only when available() is true.
  const QString &deviceName() const { return mDeviceName; }
  int computeMajor() const { return mComputeMajor; }
  int computeMinor() const { return mComputeMinor; }

#ifdef OMNISIM_WITH_CUDA
  // Default stream owned by this context. All M0 work uses this single
  // stream; M1+ may add more.
  cudaStream_t stream() const { return mStream; }

  // Block until the stream drains, mapping any error into OmLog::error with
  // a CUDA_KERNEL_EXECUTION_ERROR diagnostic. Returns true on clean drain.
  // The optional `context` string lands in the error message to help triage.
  bool synchronize(const QString &context = QString());
#endif

private:
  OmCudaContext();
  ~OmCudaContext();
  OmCudaContext(const OmCudaContext &) = delete;
  OmCudaContext &operator=(const OmCudaContext &) = delete;

  // Returns true if device init succeeded and the context is usable.
  bool initialize();

  bool mAvailable;
  int mDeviceId;
  QString mDeviceName;
  int mComputeMajor;
  int mComputeMinor;

#ifdef OMNISIM_WITH_CUDA
  cudaStream_t mStream;
#endif

  static OmCudaContext *cInstance;
  static bool cInitAttempted;
};

#endif
