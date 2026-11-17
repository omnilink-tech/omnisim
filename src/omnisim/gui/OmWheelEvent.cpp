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

#include "OmWheelEvent.hpp"

#include "OmEditCommand.hpp"
#include "OmSolid.hpp"
#include "OmUndoStack.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#define SIGN(x) ((x) > 0.0 ? 1.0 : -1.0)

// OmWheelEvent functions

OmWheelEvent::OmWheelEvent() {
}

// OmWheelLiftSolidEvent functions

OmWheelLiftSolidEvent::OmWheelLiftSolidEvent(OmViewpoint *viewpoint, OmSolid *selectedSolid) :
  mViewpoint(viewpoint),
  mSelectedSolid(selectedSolid),
  mInitialTranslation(selectedSolid->translation()) {
  mScaleFactor = OmWorld::instance()->worldInfo()->lineScale();
  mUpWorldVector = OmWorld::instance()->worldInfo()->upVector();
  mViewpoint->lock();
  mSelectedSolid->pausePhysics();
}

OmWheelLiftSolidEvent::~OmWheelLiftSolidEvent() {
  mViewpoint->unlock();
  if (mInitialTranslation != mSelectedSolid->translation())
    OmWorld::instance()->setModified();
  OmUndoStack::instance()->push(new OmEditCommand(mSelectedSolid->translationFieldValue(), OmVariant(mInitialTranslation),
                                                  OmVariant(mSelectedSolid->translationFieldValue()->variantValue())));
  mSelectedSolid->resumePhysics();
}

void OmWheelLiftSolidEvent::apply(int delta) {
  mSelectedSolid->setTranslation(SIGN(delta) * mScaleFactor * mUpWorldVector + mSelectedSolid->translation());
  mSelectedSolid->resetPhysics();
}
