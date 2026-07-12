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

#ifndef WB_GPU_PASS_TIMER_HPP
#define WB_GPU_PASS_TIMER_HPP

//
// Description: GL-timestamp-query wrapper for measuring how long a render
// pass takes on the GPU. Tier 2 instrumentation foundation per the
// rendering plan — a thin RAII helper that more invasive features (budget
// gates, debug overlays) can be layered on top of.
//
// Disabled by default: every begin/end/poll call returns immediately when
// OMNISIM_RENDERER_TIMINGS is not set to "1" in the environment. When
// disabled there are no GL calls, no allocations, and no observable
// effect on rendering.
//
// Usage (the static-local pattern keeps the queries alive across frames
// without forcing a member of the calling class):
//
//   static GpuPassTimer t("SceneRender");
//   t.begin();
//   ... GPU work ...
//   t.end();
//   t.poll();   // call once per frame after end()
//
// The class keeps a 4-slot ring of GL query pairs so the GPU never stalls:
// the result for frame N is read on frame N+3 or later, by which point
// glGetQueryObjectiv(..., GL_QUERY_RESULT_AVAILABLE) returns true without
// blocking. Slots that are still in-flight when their turn comes round
// are dropped (the measurement for that frame is lost; the next frame
// starts a fresh one).
//

class GpuPassTimer {
public:
  explicit GpuPassTimer(const char *name);
  ~GpuPassTimer();

  void begin();
  void end();
  void poll();

  // Last measured pass time in milliseconds, or 0 if never harvested.
  float lastMs() const { return mLastMs; }

  // True if OMNISIM_RENDERER_TIMINGS is set to a truthy value. Cached.
  static bool isEnabled();

private:
  GpuPassTimer(const GpuPassTimer &) = delete;
  GpuPassTimer &operator=(const GpuPassTimer &) = delete;

  static const int kRingSize = 4;
  // Log the rolling average once every N frames (~2 s at 60 fps).
  static const int kLogIntervalFrames = 120;

  const char *mName;
  bool mInitialized;
  unsigned int mStartIds[kRingSize];
  unsigned int mEndIds[kRingSize];
  bool mInFlight[kRingSize];
  int mWriteIndex;
  bool mSkipThisPair;

  // Rolling-average accumulator.
  float mLastMs;
  float mSumMs;
  int mSampleCount;
  int mFramesSinceLog;
};

#endif
