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

#ifndef OM_CUDA_ERROR_HPP
#define OM_CUDA_ERROR_HPP

//
// Description: structured-diagnostic codes and the exception type used by the
//   CUDA compute layer. Codes are mirrored verbatim in
//   scripts/harness/diagnostic_codes.py so the agent-facing harness can branch
//   on them. See docs/developer/cuda-compute-infrastructure-plan.md.
//

#include <QtCore/QString>
#include <stdexcept>

class OmCudaError : public std::runtime_error {
public:
  OmCudaError(const char *code, const QString &message) :
    std::runtime_error(message.toUtf8().constData()),
    mCode(code),
    mMessage(message) {}

  const char *code() const noexcept { return mCode; }
  const QString &qmessage() const noexcept { return mMessage; }

private:
  const char *mCode;
  QString mMessage;
};

namespace OmCudaCodes {
  // CUDA runtime / driver discovery
  static constexpr const char *NOT_AVAILABLE = "CUDA_NOT_AVAILABLE";
  static constexpr const char *DEVICE_INIT_FAILED = "CUDA_DEVICE_INIT_FAILED";
  static constexpr const char *DRIVER_TOO_OLD = "CUDA_DRIVER_TOO_OLD";
  static constexpr const char *COMPUTE_CAPABILITY_TOO_OLD = "CUDA_COMPUTE_CAPABILITY_TOO_OLD";

  // Per-operation failures
  static constexpr const char *OUT_OF_MEMORY = "CUDA_OUT_OF_MEMORY";
  static constexpr const char *KERNEL_LAUNCH_FAILED = "CUDA_KERNEL_LAUNCH_FAILED";
  static constexpr const char *KERNEL_EXECUTION_ERROR = "CUDA_KERNEL_EXECUTION_ERROR";
  static constexpr const char *MEMCPY_FAILED = "CUDA_MEMCPY_FAILED";

  // Reserved for M1 (GL/CUDA interop)
  static constexpr const char *GL_INTEROP_NOT_IMPLEMENTED = "CUDA_GL_INTEROP_NOT_IMPLEMENTED";
}  // namespace OmCudaCodes

#endif
