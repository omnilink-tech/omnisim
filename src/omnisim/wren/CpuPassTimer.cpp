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

#include "CpuPassTimer.hpp"

#include "GpuPassTimer.hpp"
#include "WbLog.hpp"

#include <QtCore/QCoreApplication>

CpuPassTimer::CpuPassTimer(const char *name) :
  mName(name),
  mActive(false),
  mLastMs(0.0f),
  mSumMs(0.0f),
  mSampleCount(0),
  mFramesSinceLog(0) {
}

void CpuPassTimer::begin() {
  if (!GpuPassTimer::isEnabled())
    return;
  mActiveTimer.start();
  mActive = true;
}

void CpuPassTimer::end() {
  if (!GpuPassTimer::isEnabled() || !mActive)
    return;
  const qint64 elapsedNs = mActiveTimer.nsecsElapsed();
  const float ms = static_cast<float>(static_cast<double>(elapsedNs) / 1.0e6);
  mLastMs = ms;
  mSumMs += ms;
  ++mSampleCount;
  mActive = false;
}

void CpuPassTimer::poll() {
  if (!GpuPassTimer::isEnabled())
    return;
  ++mFramesSinceLog;
  if (mFramesSinceLog >= kLogIntervalFrames && mSampleCount > 0) {
    const float avgMs = mSumMs / static_cast<float>(mSampleCount);
    WbLog::info(QCoreApplication::translate("CpuPassTimer",
      "CPU %1: %2 ms (avg of %3 samples)").arg(QString::fromUtf8(mName))
                                          .arg(static_cast<double>(avgMs), 0, 'f', 3)
                                          .arg(mSampleCount));
    mFramesSinceLog = 0;
    mSumMs = 0.0f;
    mSampleCount = 0;
  }
}
