// Copyright 1996-2024 Cyberbotics Ltd.
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
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#include "OmWrenOpenGlContext.hpp"

#include <QtCore/QStack>

#include <cstdio>

OmWrenOpenGlContext *OmWrenOpenGlContext::mWrenContext;
QSurface *OmWrenOpenGlContext::mWrenSurface;
bool OmWrenOpenGlContext::mIsCurrent;
QStack<bool> OmWrenOpenGlContext::mPreviousState;

void OmWrenOpenGlContext::destroy() {
  assert(mPreviousState.empty());
  delete mWrenContext;
  // Null it: isInitialized() is the engine-wide "does a GL context exist?" predicate, and
  // without this it keeps answering true on a dangling pointer after destroy() -- any
  // post-destroy caller then dereferences freed memory through instance().
  mWrenContext = NULL;
}

// Makes WREN's OpenGL context current and marks it as active.
// WREN will immediately apply changes to OpenGL state until the context has been marked as inactive.
bool OmWrenOpenGlContext::makeWrenCurrent() {
  // Compute-only headless mode (OMNISIM_NO_GL): no context was ever created. Degrade to a
  // no-op (paired doneWren() tolerates the empty stack) so unguarded legacy call sites
  // don't dereference NULL -- WREN itself stays inactive, so no GL call follows.
  if (!mWrenContext)
    return false;
  assert(mWrenSurface);

  mPreviousState.push(mIsCurrent);
  mIsCurrent = true;

  const bool ok = mWrenContext->forceMakeCurrent(mWrenSurface);
  // Callers ignore this return value throughout the engine, so a failing makeCurrent turns
  // every GL call into a silent no-op (and GL status reads into stack garbage). Say it once,
  // loudly: this is how "phantom shader-link failures with empty logs" happen.
  if (!ok) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      fprintf(stderr, "OmWrenOpenGlContext::makeWrenCurrent: QOpenGLContext::makeCurrent FAILED -- all subsequent GL "
                      "work silently no-ops. The WREN surface is not usable (unexposed window? destroyed surface?).\n");
    }
  }
  return ok;
}

// Marks WREN's OpenGL context as inactive, preventing WREN from making OpenGL calls until it is marked as active again.
// Any call to makeWrenCurrent should be followed by a call to doneWren before any possible OpenGL context changes.
void OmWrenOpenGlContext::doneWren() {
  if (!mPreviousState.empty() && mPreviousState.pop())
    return;

  mIsCurrent = false;
}

bool OmWrenOpenGlContext::isCurrent() {
  return mPreviousState.size() > 0;
}
