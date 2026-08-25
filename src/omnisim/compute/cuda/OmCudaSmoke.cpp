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

#include "OmCudaSmoke.hpp"

#include "OmCudaContext.hpp"
#include "OmCudaError.hpp"
#include "OmLog.hpp"

#include <QtCore/QByteArray>
#include <QtCore/QObject>

#ifdef OMNISIM_WITH_CUDA
#include "OmCudaBuffer.hpp"

#include <vector>
#endif

namespace {
  constexpr const char *kEnvVar = "OMNISIM_CUDA_SMOKE";
}

#ifdef OMNISIM_WITH_CUDA

bool OmCudaSmoke::runMemcpyRoundtrip() {
  OmCudaContext *ctx = OmCudaContext::instance();
  if (ctx == nullptr) {
    OmLog::warning(QObject::tr("CUDA smoke skipped: no CUDA context available."));
    return false;
  }
  try {
    constexpr std::size_t kCount = 4096;
    constexpr float kSentinel = 2.71828f;
    OmCudaBuffer<float> buffer(kCount);
    std::vector<float> host(kCount, kSentinel);
    buffer.copyFromHost(host.data(), kCount);
    std::vector<float> back(kCount, 0.0f);
    buffer.copyToHost(back.data(), kCount);
    for (std::size_t i = 0; i < kCount; ++i) {
      if (back[i] != kSentinel) {
        const QString msg = QObject::tr("CUDA smoke FAIL: index %1 = %2, expected %3.")
                              .arg(static_cast<qulonglong>(i))
                              .arg(back[i])
                              .arg(kSentinel);
        OmLog::diagnostic(OmCudaCodes::MEMCPY_FAILED, msg);
        OmLog::error(msg);
        return false;
      }
    }
    OmLog::info(QObject::tr("CUDA smoke PASS: OmCudaBuffer round-tripped %1 floats on '%2'.")
                  .arg(static_cast<qulonglong>(kCount))
                  .arg(ctx->deviceName()));
    return true;
  } catch (const OmCudaError &e) {
    OmLog::diagnostic(e.code(), e.qmessage());
    OmLog::error(e.qmessage());
    return false;
  }
}

#else  // OMNISIM_WITH_CUDA

bool OmCudaSmoke::runMemcpyRoundtrip() {
  OmLog::warning(QObject::tr("CUDA smoke skipped: build was compiled without CUDA support."));
  return false;
}

#endif  // OMNISIM_WITH_CUDA

void OmCudaSmoke::runIfRequested() {
  static const bool requested = !qgetenv(kEnvVar).isEmpty();
  if (!requested)
    return;
  static bool ranOnce = false;
  if (ranOnce)
    return;
  ranOnce = true;
  runMemcpyRoundtrip();
}
