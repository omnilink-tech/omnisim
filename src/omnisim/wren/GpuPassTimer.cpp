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

#include "GpuPassTimer.hpp"

#include "WbLog.hpp"
#include "WbWrenOpenGlContext.hpp"

#include <QtCore/QByteArray>
#include <QtCore/QCoreApplication>
#include <QtOpenGL/QOpenGLFunctions_3_3_Core>
#include <QtOpenGL/QOpenGLVersionFunctionsFactory>

namespace {

  bool sCachedEnabled = false;
  bool sEnabledQueried = false;

  QOpenGLFunctions_3_3_Core *currentGl() {
    QOpenGLContext *ctx = WbWrenOpenGlContext::instance();
    if (!ctx)
      return NULL;
    return QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(ctx);
  }

}  // namespace

bool GpuPassTimer::isEnabled() {
  if (!sEnabledQueried) {
    const QByteArray raw = qgetenv("OMNISIM_RENDERER_TIMINGS").trimmed().toLower();
    sCachedEnabled = (raw == "1" || raw == "true" || raw == "on" || raw == "yes");
    sEnabledQueried = true;
    if (sCachedEnabled)
      WbLog::info(QCoreApplication::translate(
        "GpuPassTimer",
        "OMNISIM_RENDERER_TIMINGS=1: GPU pass timing enabled (rolling averages every ~2 s per pass)."));
  }
  return sCachedEnabled;
}

GpuPassTimer::GpuPassTimer(const char *name) :
  mName(name),
  mInitialized(false),
  mWriteIndex(0),
  mSkipThisPair(false),
  mLastMs(0.0f),
  mSumMs(0.0f),
  mSampleCount(0),
  mFramesSinceLog(0) {
  for (int i = 0; i < kRingSize; ++i) {
    mStartIds[i] = 0;
    mEndIds[i] = 0;
    mInFlight[i] = false;
  }
}

GpuPassTimer::~GpuPassTimer() {
  // GL resources are not explicitly deleted here. Static-local timers
  // outlive most of the GL teardown, and a few unsigned ints worth of
  // query objects are reclaimed by the OS at process exit. Avoiding the
  // delete also sidesteps a class of "GL context already gone" crashes
  // we have hit before in shutdown ordering.
}

void GpuPassTimer::begin() {
  if (!isEnabled())
    return;
  QOpenGLFunctions_3_3_Core *f = currentGl();
  if (!f)
    return;
  if (!mInitialized) {
    f->glGenQueries(kRingSize, mStartIds);
    f->glGenQueries(kRingSize, mEndIds);
    mInitialized = true;
  }
  if (mInFlight[mWriteIndex]) {
    // Ring lapped — the query at this slot has not yet completed. Drop
    // the measurement for this frame rather than stall the pipeline.
    mSkipThisPair = true;
    return;
  }
  mSkipThisPair = false;
  f->glQueryCounter(mStartIds[mWriteIndex], 0x8E28 /* GL_TIMESTAMP */);
}

void GpuPassTimer::end() {
  if (!isEnabled() || !mInitialized || mSkipThisPair)
    return;
  QOpenGLFunctions_3_3_Core *f = currentGl();
  if (!f)
    return;
  f->glQueryCounter(mEndIds[mWriteIndex], 0x8E28 /* GL_TIMESTAMP */);
  mInFlight[mWriteIndex] = true;
  mWriteIndex = (mWriteIndex + 1) % kRingSize;
}

void GpuPassTimer::poll() {
  if (!isEnabled() || !mInitialized)
    return;
  QOpenGLFunctions_3_3_Core *f = currentGl();
  if (!f)
    return;

  // Walk the ring for completed results. The just-written slot will
  // typically not be available yet; that's fine, we'll harvest it on a
  // later frame.
  for (int i = 0; i < kRingSize; ++i) {
    if (!mInFlight[i])
      continue;
    int available = 0;
    f->glGetQueryObjectiv(mEndIds[i], 0x8867 /* GL_QUERY_RESULT_AVAILABLE */, &available);
    if (!available)
      continue;
    GLuint64 startNs = 0;
    GLuint64 endNs = 0;
    f->glGetQueryObjectui64v(mStartIds[i], 0x8866 /* GL_QUERY_RESULT */, &startNs);
    f->glGetQueryObjectui64v(mEndIds[i], 0x8866 /* GL_QUERY_RESULT */, &endNs);
    mInFlight[i] = false;
    const float deltaMs = static_cast<float>(static_cast<double>(endNs - startNs) / 1.0e6);
    mLastMs = deltaMs;
    mSumMs += deltaMs;
    ++mSampleCount;
  }

  // Periodically log a rolling average. Keep separate frame counter from
  // sample counter — frames where the harvest skipped (no completed
  // queries) still count toward the print interval.
  ++mFramesSinceLog;
  if (mFramesSinceLog >= kLogIntervalFrames && mSampleCount > 0) {
    const float avgMs = mSumMs / static_cast<float>(mSampleCount);
    WbLog::info(QCoreApplication::translate("GpuPassTimer",
      "GPU %1: %2 ms (avg of %3 samples)").arg(QString::fromUtf8(mName))
                                          .arg(static_cast<double>(avgMs), 0, 'f', 3)
                                          .arg(mSampleCount));
    mFramesSinceLog = 0;
    mSumMs = 0.0f;
    mSampleCount = 0;
  }
}
