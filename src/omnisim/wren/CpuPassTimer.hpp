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

#ifndef WB_CPU_PASS_TIMER_HPP
#define WB_CPU_PASS_TIMER_HPP

//
// Description: CPU companion to GpuPassTimer. Wraps QElapsedTimer with the
// same begin / end / poll lifecycle, reports rolling averages on the same
// schedule (every ~120 frames), and respects the same enable env var
// (OMNISIM_RENDERER_TIMINGS).
//
// CPU timers are needed alongside GPU timers because they measure
// different things:
//   - GpuPassTimer measures the GPU's own work between two
//     glQueryCounter calls. It will show ~0 ms for CPU-bound code paths
//     even when wall-clock time is significant.
//   - CpuPassTimer measures wall-clock CPU time the host spent inside
//     the section, including driver waits and vsync stalls. It is the
//     right tool for quantifying CPU-side render-loop overhead.
//
// Usage mirrors GpuPassTimer:
//
//   static CpuPassTimer t("FullRenderNow");
//   t.begin();
//   ... CPU + driver work ...
//   t.end();
//   t.poll();
//

#include <QtCore/QElapsedTimer>

class CpuPassTimer {
public:
  explicit CpuPassTimer(const char *name);

  void begin();
  void end();
  void poll();

  float lastMs() const { return mLastMs; }

private:
  CpuPassTimer(const CpuPassTimer &) = delete;
  CpuPassTimer &operator=(const CpuPassTimer &) = delete;

  // Log every ~2 s at 60 fps. Same cadence as GpuPassTimer so paired
  // CPU/GPU measurements land in the log together for easy reading.
  static const int kLogIntervalFrames = 120;

  const char *mName;
  QElapsedTimer mActiveTimer;
  bool mActive;

  float mLastMs;
  float mSumMs;
  int mSampleCount;
  int mFramesSinceLog;
};

#endif
