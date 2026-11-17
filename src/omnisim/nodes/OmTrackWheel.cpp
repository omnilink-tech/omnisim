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

#include "OmTrackWheel.hpp"

#include "OmFieldChecker.hpp"
#include "OmSFBool.hpp"
#include "OmSFDouble.hpp"
#include "OmSFVector2.hpp"

void OmTrackWheel::init() {
  mPosition = findSFVector2("position");
  mRadius = findSFDouble("radius");
  mInner = findSFBool("inner");

  // define Pose fields
  mTranslation = new OmSFVector3(OmVector3());
  mRotation = findSFRotation("rotation");
  mTranslationStep = new OmSFDouble(0.1);
  mRotationStep = new OmSFDouble(0.1);
}

OmTrackWheel::OmTrackWheel(OmTokenizer *tokenizer) : OmPose("TrackWheel", tokenizer) {
  init();
}

OmTrackWheel::OmTrackWheel(const OmTrackWheel &other) : OmPose(other) {
  init();
}

OmTrackWheel::OmTrackWheel(const OmNode &other) : OmPose(other) {
  init();
}

OmTrackWheel::~OmTrackWheel() {
}

void OmTrackWheel::preFinalize() {
  OmPose::preFinalize();

  updatePosition();
  updateRadius();
}

void OmTrackWheel::postFinalize() {
  OmPose::postFinalize();

  connect(mPosition, &OmSFVector2::changed, this, &OmTrackWheel::updatePosition);
  connect(mRadius, &OmSFDouble::changed, this, &OmTrackWheel::updateRadius);
  connect(mInner, &OmSFBool::changed, this, &OmTrackWheel::changed);

  emit changed();  // trigger OmTrack belt update
}

void OmTrackWheel::updatePosition() {
  setTranslation(mPosition->x(), 0, mPosition->y());
  updateTranslation();
  if (isPostFinalizedCalled())
    emit changed();
}

void OmTrackWheel::updateRadius() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mRadius, 0.1) && isPostFinalizedCalled())
    emit changed();
}

void OmTrackWheel::rotate(double traveledDistance) {
  double angle = traveledDistance / radius();
  if (mInner->value())
    angle = -angle;

  OmRotation newRotation(OmMatrix3(0, -1, 0, angle) * rotation().toMatrix3());
  newRotation.normalize();
  setRotation(newRotation);
  updateRotation();
}

bool OmTrackWheel::shallExport() const {
  return true;
}

void OmTrackWheel::write(OmWriter &writer) const {
  if (writer.isUrdf())
    return;
  OmPose::write(writer);
}

QStringList OmTrackWheel::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "position"
         << "radius"
         << "rotation"
         << "inner";
  return fields;
}

void OmTrackWheel::exportNodeFields(OmWriter &writer) const {
  OmBaseNode::exportNodeFields(writer);

  if (writer.isW3d())
    writer << " rotation=\'" << mRotation->value() << "\'";
}
