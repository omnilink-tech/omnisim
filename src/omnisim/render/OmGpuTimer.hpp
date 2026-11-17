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

#ifndef OM_GPU_TIMER_HPP
#define OM_GPU_TIMER_HPP

// Private to the native render target implementation. No synchronous GPU waits.
#include <webgpu/webgpu.h>
#include <webgpu/wgpu.h>
#include <array>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <map>
#include <sstream>
#include <string>

class OmGpuTimer {
  static constexpr uint32_t kPasses = 64;
  static constexpr uint64_t kBytes = kPasses * 2 * sizeof(uint64_t);
  struct Capture {
    WGPUBuffer buffer = nullptr;
    std::atomic<bool> done{true};
    std::string path;
    std::array<std::string, kPasses> names;
    uint64_t frame = 0, target = 0;
    uint32_t count = 0, width = 0, height = 0;
    float period = 0;
    double cpuIntervalUs = 0;
    ~Capture() { if (buffer) wgpuBufferRelease(buffer); }
  };
  struct Slot {
    WGPUQuerySet queries = nullptr;
    WGPUBuffer resolve = nullptr;
    std::shared_ptr<Capture> capture;
  };
  std::array<Slot, 3> mSlots;
  Slot *mActive = nullptr;
  WGPUPassTimestampWrites mWrites = {};
  uint64_t mFrame = 0;
  std::chrono::steady_clock::time_point mPreviousStart;
  bool mEnabled = false;

  static void mapped(WGPUMapAsyncStatus status, WGPUStringView, void *userdata, void *) {
    // The callback owns its capture, independent of the render target's lifetime.
    std::unique_ptr<std::shared_ptr<Capture>> owner(static_cast<std::shared_ptr<Capture> *>(userdata));
    const auto &c = *owner;
    if (status == WGPUMapAsyncStatus_Success) {
      const auto *ticks = static_cast<const uint64_t *>(wgpuBufferGetConstMappedRange(c->buffer, 0, kBytes));
      if (ticks) {
        uint64_t first = UINT64_MAX, last = 0;
        std::map<std::string, double> totals;
        bool valid = c->count > 0;
        for (uint32_t i = 0; i < c->count; ++i) {
          const uint64_t a = ticks[2*i], b = ticks[2*i+1];
          // Reject wrapped/invalid samples, rather than publishing a fake zero.
          if (b < a || a == 0) { valid = false; break; }
          first = std::min(first, a);
          last = std::max(last, b);
          totals[c->names[i]] += static_cast<double>(b-a) * c->period / 1000.0;
        }
        if (valid) {
          std::ostringstream line;
          line.precision(9);
          line << "target=" << c->target << " frame=" << c->frame << " width=" << c->width
               << " height=" << c->height << " gpuSpanUs=" << static_cast<double>(last-first)*c->period/1000.0;
          if (c->cpuIntervalUs > 0) line << " cpuIntervalUs=" << c->cpuIntervalUs;
          for (const auto &entry : totals) line << ' ' << entry.first << "Us=" << entry.second;
          line << " passes=" << c->count << '\n';
          if (FILE *f = std::fopen(c->path.c_str(), "ab")) {
            const std::string text = line.str();
            std::fwrite(text.data(), 1, text.size(), f);
            std::fclose(f);
          }
        }
      }
      wgpuBufferUnmap(c->buffer);
    }
    c->done.store(true, std::memory_order_release);
  }

public:
  OmGpuTimer(WGPUDevice device, WGPUQueue queue, const char *path, uint32_t width, uint32_t height) {
    static std::atomic<uint64_t> nextTarget{0};
    const uint64_t target = ++nextTarget;
    const float period = wgpuQueueGetTimestampPeriod(queue);
    if (!wgpuDeviceHasFeature(device, WGPUFeatureName_TimestampQuery) || !std::isfinite(period) || period <= 0) {
      if (FILE *f = std::fopen(path, "ab")) {
        std::fprintf(f, "target=%llu width=%u height=%u status=unavailable\n",
                     static_cast<unsigned long long>(target), width, height);
        std::fclose(f);
      }
      return;
    }
    for (auto &slot : mSlots) {
      WGPUQuerySetDescriptor q = {};
      q.type = WGPUQueryType_Timestamp;
      q.count = kPasses*2;
      slot.queries = wgpuDeviceCreateQuerySet(device, &q);
      WGPUBufferDescriptor b = {};
      b.size = kBytes;
      b.usage = WGPUBufferUsage_QueryResolve | WGPUBufferUsage_CopySrc;
      slot.resolve = wgpuDeviceCreateBuffer(device, &b);
      b.usage = WGPUBufferUsage_MapRead | WGPUBufferUsage_CopyDst;
      slot.capture = std::make_shared<Capture>();
      auto &c = *slot.capture;
      c.buffer = wgpuDeviceCreateBuffer(device, &b);
      c.path = path;
      c.period = period;
      c.target = target;
      c.width = width;
      c.height = height;
      if (!slot.queries || !slot.resolve || !c.buffer) return;
    }
    mEnabled = true;
  }
  ~OmGpuTimer() {
    for (auto &slot : mSlots) {
      // Destroy cancels a pending map; its callback never touches this object.
      if (slot.capture && slot.capture->buffer) wgpuBufferDestroy(slot.capture->buffer);
      if (slot.resolve) wgpuBufferRelease(slot.resolve);
      if (slot.queries) wgpuQuerySetRelease(slot.queries);
    }
  }
  void begin(WGPUDevice device) {
    mActive = nullptr;
    ++mFrame;
    if (!mEnabled) return;
    const auto now = std::chrono::steady_clock::now();
    const double interval = mFrame > 1 ? std::chrono::duration<double, std::micro>(now-mPreviousStart).count() : 0;
    mPreviousStart = now;
    wgpuDevicePoll(device, false, nullptr);
    for (auto &slot : mSlots) {
      if (slot.capture->done.load(std::memory_order_acquire)) {
        mActive = &slot;
        slot.capture->frame = mFrame;
        slot.capture->count = 0;
        slot.capture->cpuIntervalUs = interval;
        break;
      }
    }
    // All three maps busy? Skip this frame, never stall the renderer.
  }
  const WGPUPassTimestampWrites *stamp(const char *name) {
    if (!mActive || mActive->capture->count == kPasses) return nullptr;
    const uint32_t pass = mActive->capture->count++;
    mActive->capture->names[pass] = name;
    mWrites.querySet = mActive->queries;
    mWrites.beginningOfPassWriteIndex = pass*2;
    mWrites.endOfPassWriteIndex = pass*2+1;
    return &mWrites;
  }
  void resolve(WGPUCommandEncoder encoder) {
    if (!mActive || !mActive->capture->count) return;
    wgpuCommandEncoderResolveQuerySet(encoder, mActive->queries, 0, mActive->capture->count*2, mActive->resolve, 0);
    wgpuCommandEncoderCopyBufferToBuffer(encoder, mActive->resolve, 0, mActive->capture->buffer, 0, kBytes);
  }
  void submitted() {
    if (!mActive || !mActive->capture->count) return;
    auto capture = mActive->capture;
    capture->done.store(false, std::memory_order_release);
    WGPUBufferMapCallbackInfo cb = {};
    cb.mode = WGPUCallbackMode_AllowProcessEvents;
    cb.callback = mapped;
    cb.userdata1 = new std::shared_ptr<Capture>(capture);
    wgpuBufferMapAsync(capture->buffer, WGPUMapMode_Read, 0, kBytes, cb);
    mActive = nullptr;
  }
};
#endif
