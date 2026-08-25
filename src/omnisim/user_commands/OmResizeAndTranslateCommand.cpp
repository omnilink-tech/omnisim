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

#include "OmResizeAndTranslateCommand.hpp"
#include "OmIndexedFaceSet.hpp"

#include <cassert>

OmResizeAndTranslateCommand::OmResizeAndTranslateCommand(OmGeometry *geometry, const OmVector3 &scale,
                                                         const OmVector3 &translation, QUndoCommand *parent) :
  OmResizeCommand(geometry, scale, parent),
  mTranslation(translation) {
  mIsTranslationSet = (mTranslation != OmVector3());
}

void OmResizeAndTranslateCommand::undo() {
  if (!mIsTranslationSet)
    OmResizeCommand::undo();
  else
    resetValue(true);
}

void OmResizeAndTranslateCommand::redo() {
  if (!mIsTranslationSet) {
    OmResizeCommand::redo();
    return;
  }

  if (mIsFirstCall) {
    mIsFirstCall = false;
    return;
  }

  resetValue(false);
}

void OmResizeAndTranslateCommand::resetValue(bool invertedAction) {
  assert(mGeometry);

  OmIndexedFaceSet *indexedFaceSet = dynamic_cast<OmIndexedFaceSet *>(mGeometry);
  if (!indexedFaceSet)
    return;

  if (invertedAction) {
    indexedFaceSet->translate(-mTranslation);
    indexedFaceSet->rescale(mInvScale);
  } else
    indexedFaceSet->rescaleAndTranslate(mScale, mTranslation);
}
