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

#include "WbCudaContext.hpp"
#include "WbCudaError.hpp"

#include "WbLog.hpp"

#include <QtCore/QByteArray>
#include <QtCore/QCoreApplication>
#include <QtCore/QObject>

#ifdef OMNISIM_WITH_CUDA
#include <cuda_runtime.h>
#endif

// Minimum compute capability targeted (sm_75 = Turing). Anything older is
// rejected at init with CUDA_COMPUTE_CAPABILITY_TOO_OLD.
static constexpr int kMinComputeMajor = 7;
static constexpr int kMinComputeMinor = 5;

WbCudaContext *WbCudaContext::cInstance = nullptr;
bool WbCudaContext::cInitAttempted = false;

WbCudaContext::WbCudaContext() :
  mAvailable(false),
  mDeviceId(-1),
  mComputeMajor(0),
  mComputeMinor(0)
#ifdef OMNISIM_WITH_CUDA
  ,
  mStream(nullptr)
#endif
{
}

WbCudaContext::~WbCudaContext() {
#ifdef OMNISIM_WITH_CUDA
  if (mStream != nullptr) {
    cudaStreamDestroy(mStream);
    mStream = nullptr;
  }
#endif
}

WbCudaContext *WbCudaContext::instance() {
  if (cInitAttempted)
    return cInstance ? (cInstance->mAvailable ? cInstance : nullptr) : nullptr;
  cInitAttempted = true;

  cInstance = new WbCudaContext();
  qAddPostRoutine([]() {
    delete cInstance;
    cInstance = nullptr;
  });

  if (!cInstance->initialize())
    return nullptr;
  return cInstance;
}

bool WbCudaContext::available() {
#ifndef OMNISIM_WITH_CUDA
  return false;
#else
  if (!cInitAttempted)
    instance();  // trigger lazy init
  return cInstance != nullptr && cInstance->mAvailable;
#endif
}

bool WbCudaContext::initialize() {
#ifndef OMNISIM_WITH_CUDA
  WbLog::diagnostic(WbCudaCodes::NOT_AVAILABLE,
                    QObject::tr("CUDA support was disabled at build time (OMNISIM_WITH_CUDA=OFF)."));
  // Silent on the user-visible log: subsystems decide whether to surface this.
  return false;
#else
  // Pick device. Honor $OMNISIM_CUDA_DEVICE for multi-GPU boxes.
  int requested = 0;
  const QByteArray override = qgetenv("OMNISIM_CUDA_DEVICE");
  if (!override.isEmpty()) {
    bool ok = false;
    const int v = override.toInt(&ok);
    if (ok && v >= 0)
      requested = v;
  }

  int deviceCount = 0;
  cudaError_t err = cudaGetDeviceCount(&deviceCount);
  if (err != cudaSuccess || deviceCount <= 0) {
    const QString msg = QObject::tr("No CUDA-capable device found (cudaGetDeviceCount returned %1, count=%2).")
                          .arg(err)
                          .arg(deviceCount);
    WbLog::diagnostic(WbCudaCodes::NOT_AVAILABLE, msg);
    WbLog::warning(msg);
    return false;
  }

  if (requested >= deviceCount) {
    WbLog::warning(QObject::tr("OMNISIM_CUDA_DEVICE=%1 but only %2 device(s) present; falling back to device 0.")
                     .arg(requested)
                     .arg(deviceCount));
    requested = 0;
  }

  err = cudaSetDevice(requested);
  if (err != cudaSuccess) {
    const QString msg = QObject::tr("cudaSetDevice(%1) failed: %2").arg(requested).arg(cudaGetErrorString(err));
    WbLog::diagnostic(WbCudaCodes::DEVICE_INIT_FAILED, msg);
    WbLog::error(msg);
    return false;
  }

  cudaDeviceProp props{};
  err = cudaGetDeviceProperties(&props, requested);
  if (err != cudaSuccess) {
    const QString msg =
      QObject::tr("cudaGetDeviceProperties(%1) failed: %2").arg(requested).arg(cudaGetErrorString(err));
    WbLog::diagnostic(WbCudaCodes::DEVICE_INIT_FAILED, msg);
    WbLog::error(msg);
    return false;
  }

  if (props.major < kMinComputeMajor || (props.major == kMinComputeMajor && props.minor < kMinComputeMinor)) {
    const QString msg = QObject::tr("CUDA device '%1' has compute capability %2.%3, "
                                    "but OmniSim requires at least %4.%5.")
                          .arg(QString::fromUtf8(props.name))
                          .arg(props.major)
                          .arg(props.minor)
                          .arg(kMinComputeMajor)
                          .arg(kMinComputeMinor);
    WbLog::diagnostic(WbCudaCodes::COMPUTE_CAPABILITY_TOO_OLD, msg);
    WbLog::error(msg);
    return false;
  }

  err = cudaStreamCreate(&mStream);
  if (err != cudaSuccess) {
    const QString msg = QObject::tr("cudaStreamCreate failed: %1").arg(cudaGetErrorString(err));
    WbLog::diagnostic(WbCudaCodes::DEVICE_INIT_FAILED, msg);
    WbLog::error(msg);
    return false;
  }

  mDeviceId = requested;
  mDeviceName = QString::fromUtf8(props.name);
  mComputeMajor = props.major;
  mComputeMinor = props.minor;
  mAvailable = true;

  WbLog::info(QObject::tr("CUDA initialized: device %1 '%2' (compute capability %3.%4, %5 MiB total).")
                .arg(mDeviceId)
                .arg(mDeviceName)
                .arg(mComputeMajor)
                .arg(mComputeMinor)
                .arg(static_cast<qulonglong>(props.totalGlobalMem >> 20)));
  return true;
#endif
}

#ifdef OMNISIM_WITH_CUDA
bool WbCudaContext::synchronize(const QString &context) {
  if (!mAvailable)
    return false;
  cudaError_t err = cudaStreamSynchronize(mStream);
  if (err != cudaSuccess) {
    const QString where = context.isEmpty() ? QStringLiteral("(no context)") : context;
    const QString msg = QObject::tr("CUDA stream sync failed in %1: %2").arg(where).arg(cudaGetErrorString(err));
    WbLog::diagnostic(WbCudaCodes::KERNEL_EXECUTION_ERROR, msg);
    WbLog::error(msg);
    return false;
  }
  return true;
}
#endif
