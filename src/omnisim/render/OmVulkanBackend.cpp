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

#include "OmVulkanBackend.hpp"

#include "OmLog.hpp"

#include <QtCore/QFile>
#include <QtCore/QString>
#include <QtCore/QThread>

#include <cstdio>

// R3.1 of engine-migration-plan.md §14.3:
//   - At R1 this was a marker stub (isAvailable() hardcoded false).
//   - At R3.1 the build flag actually does something. With wgpu-native
//     available on the build host (detected via WB_WGPU_NATIVE_AVAILABLE
//     from the Makefile — gated on $WGPU_NATIVE_HOME pointing at a
//     directory containing include/webgpu/webgpu.h), the constructor
//     opens an instance + adapter + device. Any failure leaves
//     mAvailable=false and the registry's fall-through silently routes
//     the camera to WREN.
//
//   - Without WB_WGPU_NATIVE_AVAILABLE the constructor stays as the R1
//     marker: mAvailable=false, no wgpu types referenced, no linker
//     dep on wgpu_native. So OMNISIM_WITH_VULKAN=ON builds stay green
//     on machines without the dep — contributors don't have to install
//     wgpu-native before they can build OmniSim.
//
//   - The variable + macro names keep the historical "Vulkan" stem
//     even though the actual API is wgpu-native (which sits on top of
//     Vulkan/Metal/D3D12). Rename is a Phase ζ cleanup.
//
//   - API tracked against wgpu-native v29.0.0+. Earlier wgpu-native
//     releases (≤ v24) had a different callback signature; if rolling
//     back to an older pinned release, this file's adapterCb/deviceCb
//     need their userdata1/userdata2 args dropped back to a single
//     `void *userdata`.

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
#    include "webgpu/webgpu.h"
#    include "webgpu/wgpu.h"
#  endif
#endif

#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
namespace {
  struct AdapterCapture {
    WGPUAdapter adapter = nullptr;
    bool done = false;
  };
  struct DeviceCapture {
    WGPUDevice device = nullptr;
    bool done = false;
  };

  // v29 callback shape: (status, handle, message[StringView], userdata1, userdata2)
  void onAdapter(WGPURequestAdapterStatus status, WGPUAdapter adapter,
                 WGPUStringView /*message*/, void *userdata1, void * /*userdata2*/) {
    auto *cap = static_cast<AdapterCapture *>(userdata1);
    if (status == WGPURequestAdapterStatus_Success)
      cap->adapter = adapter;
    cap->done = true;
  }

  void onDevice(WGPURequestDeviceStatus status, WGPUDevice device,
                WGPUStringView /*message*/, void *userdata1, void * /*userdata2*/) {
    auto *cap = static_cast<DeviceCapture *>(userdata1);
    if (status == WGPURequestDeviceStatus_Success)
      cap->device = device;
    cap->done = true;
  }

  // R4 lighting-debug: surface wgpu validation/device errors (otherwise silent on a
  // windows-subsystem GUI binary). Logs via OmLog AND, if OMNISIM_WGPU_ERRLOG is set,
  // appends to that file so headless self-checks can read the exact failure.
  void onUncapturedError(WGPUDevice const * /*device*/, WGPUErrorType type, WGPUStringView message,
                         void * /*userdata1*/, void * /*userdata2*/) {
    const char *data = message.data ? message.data : "";
    int len = static_cast<int>(message.length);
    if (message.length == WGPU_STRLEN || len < 0) {
      len = 0;
      for (const char *q = data; q && *q; ++q)
        ++len;
    }
    const QString msg = QString("[OmWgpuBackend] WGPU-ERROR type=%1: %2")
                          .arg(static_cast<int>(type))
                          .arg(QString::fromUtf8(data, len));
    OmLog::info(msg);
    if (qEnvironmentVariableIsSet("OMNISIM_WGPU_ERRLOG")) {
      QFile f(qEnvironmentVariable("OMNISIM_WGPU_ERRLOG"));
      if (f.open(QIODevice::Append | QIODevice::Text)) {
        f.write(msg.toUtf8());
        f.write("\n");
        f.close();
      }
    }
  }
}  // namespace
#  endif
#endif

OmVulkanBackend::OmVulkanBackend() : mAvailable(false) {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  // Diagnostic: when OMNISIM_WGPU_INITLOG=<file> is set, append the init stage
  // reached to that file (the OmLog lines below are invisible on a windows-
  // subsystem binary). Tells us exactly which step — instance/adapter/device —
  // fails. Inert when the env var is unset.
  const QString wgpuInitLog = qEnvironmentVariable("OMNISIM_WGPU_INITLOG");
  auto note = [&wgpuInitLog](const char *m) {
    if (wgpuInitLog.isEmpty())
      return;
    QFile f(wgpuInitLog);
    if (f.open(QIODevice::Append | QIODevice::Text)) {
      f.write(m);
      f.write("\n");
      f.close();
    }
  };
  note("init begin");
  // Step 1: instance. Default descriptor opts into all platform-native
  // backends (Vulkan/Metal/D3D12). The wgpu-native impl exposes
  // Backend_Vulkan / _Metal / _D3D12 / _GL — we leave the default
  // (any-non-Force32) so the implementation picks whichever native
  // backend is available; on Linux that's Vulkan, on macOS Metal, on
  // Windows D3D12.
  WGPUInstanceDescriptor instDesc = {};
  WGPUInstance instance = wgpuCreateInstance(&instDesc);
  if (!instance) {
    note("FAIL: wgpuCreateInstance returned NULL");
    OmLog::info("[OmWgpuBackend] wgpuCreateInstance returned NULL; falling back to WREN");
    return;
  }
  note("instance OK");

  // Step 2+3: adapter + device. wgpu-native fires these callbacks
  // synchronously (wgpuInstanceWaitAny is unimplemented in v29, so we
  // busy-poll wgpuInstanceProcessEvents on the done flag).
  //
  // HARDENING (live-realtime work, 2026-06): the old single-shot, 100-
  // spin-no-sleep acquisition gave up the instant the driver was slow or
  // transiently refused an adapter/device -- and fell back to WREN
  // SILENTLY (OmLog::info is invisible on the windows-subsystem binary).
  // That's the "wgpu activated once then never again" intermittency on a
  // GPU already busy with mujoco_warp's CUDA context. Now:
  //   (a) poll up to ~1.5 s WITH a real 1 ms sleep, so a slow driver gets
  //       time to answer instead of being declared dead in microseconds;
  //   (b) RETRY the whole adapter+device acquisition a few times with a
  //       linear back-off, so a transient refusal recovers;
  //   (c) on persistent failure, signal LOUDLY (stderr -> omnisim_log.txt
  //       + the init-log note()) instead of vanishing into WREN.
  auto pollDone = [&](const bool *done) {
    for (int i = 0; i < 1500 && !*done; ++i) {
      wgpuInstanceProcessEvents(instance);
      if (!*done)
        QThread::msleep(1);
    }
  };

  const int kMaxAttempts = 4;
  for (int attempt = 0; attempt < kMaxAttempts && !mAvailable; ++attempt) {
    if (attempt > 0) {
      note("retrying adapter+device acquisition after a transient failure");
      QThread::msleep(40 * attempt);  // linear back-off: 40, 80, 120 ms
    }

    // --- adapter ---
    WGPURequestAdapterOptions adapterOpts = {};
    adapterOpts.powerPreference = WGPUPowerPreference_HighPerformance;
    AdapterCapture aCap;
    WGPURequestAdapterCallbackInfo adapterCb = {};
    adapterCb.mode = WGPUCallbackMode_AllowSpontaneous;
    adapterCb.callback = onAdapter;
    adapterCb.userdata1 = &aCap;
    (void)wgpuInstanceRequestAdapter(instance, &adapterOpts, adapterCb);
    pollDone(&aCap.done);
    if (!aCap.done || !aCap.adapter) {
      note(aCap.done ? "FAIL: requestAdapter done but adapter NULL (no GPU adapter)"
                     : "FAIL: requestAdapter callback never fired (poll timeout)");
      continue;  // retry the whole acquisition
    }
    note("adapter OK");

    // Diagnostic: which GPU backend wgpu-native chose (Vulkan/D3D12/...).
    {
      WGPUAdapterInfo info = {};
      if (wgpuAdapterGetInfo(aCap.adapter, &info) == WGPUStatus_Success) {
        const char *bk = "other";
        switch (info.backendType) {
          case WGPUBackendType_Vulkan: bk = "Vulkan (RenderDoc-capturable)"; break;
          case WGPUBackendType_D3D12: bk = "D3D12 (RenderDoc-capturable)"; break;
          case WGPUBackendType_D3D11: bk = "D3D11 (RenderDoc-capturable)"; break;
          case WGPUBackendType_OpenGL: bk = "OpenGL (RenderDoc-capturable)"; break;
          case WGPUBackendType_Metal: bk = "Metal (use Xcode GPU capture)"; break;
          default: break;
        }
        const QString dev =
          info.device.data ? QString::fromUtf8(info.device.data, static_cast<int>(info.device.length)) : QString();
        const QByteArray line = QString("adapter backend=%1  device=%2").arg(bk).arg(dev).toUtf8();
        note(line.constData());
        // Lane E4: keep the identity for adapterSummary() -- the OpenGL-info dialog shows it
        // where the WREN/GL vendor+renderer strings used to be the only identification.
        {
          const QByteArray summary = QString("%1 via %2").arg(dev.isEmpty() ? QStringLiteral("unknown adapter") : dev,
                                                              QString::fromUtf8(bk).section(QLatin1Char(' '), 0, 0)).toUtf8();
          qstrncpy(mAdapterSummary, summary.constData(), sizeof(mAdapterSummary));
        }
      } else {
        note("adapter backend: wgpuAdapterGetInfo failed");
      }
    }

    // --- device ---
    WGPUDeviceDescriptor deviceDesc = {};
    deviceDesc.uncapturedErrorCallbackInfo.callback = onUncapturedError;
    DeviceCapture dCap;
    WGPURequestDeviceCallbackInfo deviceCb = {};
    deviceCb.mode = WGPUCallbackMode_AllowSpontaneous;
    deviceCb.callback = onDevice;
    deviceCb.userdata1 = &dCap;
    (void)wgpuAdapterRequestDevice(aCap.adapter, &deviceDesc, deviceCb);
    pollDone(&dCap.done);
    if (!dCap.done || !dCap.device) {
      note(dCap.done ? "FAIL: requestDevice done but device NULL"
                     : "FAIL: requestDevice callback never fired (poll timeout)");
      wgpuAdapterRelease(aCap.adapter);  // drop the adapter before retrying
      continue;
    }
    note("device OK");

    // Success: retain instance + adapter + device + queue (released in
    // ~OmVulkanBackend). Queue fetched eagerly -- every upload/submit needs it.
    mInstance = instance;
    mAdapter = aCap.adapter;
    mDevice = dCap.device;
    mQueue = wgpuDeviceGetQueue(dCap.device);
    OmLog::info("[OmWgpuBackend] wgpu-native init OK (instance + adapter + device + queue)");
    note("ALL OK (instance+adapter+device+queue) -> mAvailable=true");
    mAvailable = true;
  }

  if (!mAvailable) {
    note("FAIL: gave up after retries -> WREN");
    // Loud, always-on signal: stderr is captured into omnisim_log.txt even on
    // the windows-subsystem binary (where OmLog::info is not).
    fprintf(stderr,
            "[OmWgpuBackend] wgpu init FAILED after %d attempts; falling back to WREN. "
            "Set OMNISIM_WGPU_INITLOG=<file> for the per-stage reason (most common: a "
            "GPU adapter/device the driver transiently refused -- a fresh process/boot "
            "usually clears it).\n",
            kMaxAttempts);
    fflush(stderr);
    wgpuInstanceRelease(instance);
    return;
  }
#  else
  // OMNISIM_WITH_VULKAN=ON but wgpu-native isn't on this build host.
  // Stay R1-style unavailable; see docs/developer/wgpu-native-setup.md.
  mAvailable = false;
#  endif
#else
  // OMNISIM_WITH_VULKAN=OFF — zero wgpu code is linked.
  mAvailable = false;
#endif
}

OmVulkanBackend::~OmVulkanBackend() {
#ifdef OMNISIM_WITH_VULKAN
#  ifdef WB_WGPU_NATIVE_AVAILABLE
  // Order matters: queue -> device -> adapter -> instance. Each release
  // is a no-op on null, so the early-fail branches above don't need
  // matching cleanup.
  if (mQueue)
    wgpuQueueRelease(static_cast<WGPUQueue>(mQueue));
  if (mDevice)
    wgpuDeviceRelease(static_cast<WGPUDevice>(mDevice));
  if (mAdapter)
    wgpuAdapterRelease(static_cast<WGPUAdapter>(mAdapter));
  if (mInstance)
    wgpuInstanceRelease(static_cast<WGPUInstance>(mInstance));
#  endif
#endif
}
