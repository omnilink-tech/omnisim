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

#include "OmPen.hpp"

#include "OmFieldChecker.hpp"
#include "OmMatrix3.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPaintTexture.hpp"
#include "OmRay.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmShape.hpp"
#include "OmSimulationState.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>
#include <cassert>

void OmPen::init() {
  mInkColor = findSFColor("inkColor");
  mInkDensity = findSFDouble("inkDensity");
  mLeadSize = findSFDouble("leadSize");
  mWrite = findSFBool("write");
  mMaxDistance = findSFDouble("maxDistance");

  mLastPaintTexture = NULL;
  OmSimulationState::instance()->subscribeToRayTracing();
}

OmPen::OmPen(OmTokenizer *tokenizer) : OmSolidDevice("Pen", tokenizer) {
  init();
}

OmPen::OmPen(const OmPen &other) : OmSolidDevice(other) {
  init();
}

OmPen::OmPen(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmPen::~OmPen() {
  OmSimulationState::instance()->unsubscribeToRayTracing();
}

void OmPen::preFinalize() {
  OmSolidDevice::preFinalize();

  OmFieldChecker::clampDoubleToRangeWithIncludedBounds(this, mInkDensity, 0.0, 1.0);
}

void OmPen::handleMessage(QDataStream &stream) {
  unsigned char command;
  stream >> command;

  switch (command) {
    case C_PEN_WRITE:
      mWrite->setValue(true);
      return;
    case C_PEN_DONT_WRITE:
      mWrite->setValue(false);
      return;
    case C_PEN_SET_INK_COLOR: {
      unsigned char r, g, b;
      stream >> r >> g >> b;
      mInkColor->setValue(r / 255.0f, g / 255.0f, b / 255.0f);
      double density;
      stream >> density;
      mInkDensity->setValue(density);
      OmFieldChecker::clampDoubleToRangeWithIncludedBounds(this, mInkDensity, 0.0, 1.0);
      return;
    }
    default:
      assert(0);
  }
}

void OmPen::prePhysicsStep(double ms) {
  OmSolidDevice::prePhysicsStep(ms);

  double maxDistance = mMaxDistance->value();
  if (maxDistance <= 0.0)
    maxDistance = std::numeric_limits<double>::infinity();

  if (mWrite->isTrue()) {
    // find shape/texture that intersects the ray
    const OmMatrix4 &m = matrix();
    const OmVector3 globalDirection = m.sub3x3MatrixDot(OmVector3(0, 0, -1));
    const OmRay ray(m.translation(), globalDirection);
    double distance;
    const OmShape *shape = OmNodeUtilities::findIntersectingShape(ray, maxDistance, distance);

    if (shape && OmPaintTexture::isPaintable(shape)) {
      if (!mLastPaintTexture || shape != mLastPaintTexture->shape())
        mLastPaintTexture = OmPaintTexture::paintTexture(shape);

      if (mLastPaintTexture)
        mLastPaintTexture->paint(ray, mLeadSize->value(), mInkColor->value(), mInkDensity->value());
    }
  }
}

void OmPen::reset(const QString &id) {
  OmSolid::reset(id);
  OmPaintTexture::clearAllTextures();
}

// D1.4: the WREN pen-ray visualisation (the VF_PEN_RAYS line, violet when writing) is gone
// with the renderer that drew it. The painting logic above is untouched.
