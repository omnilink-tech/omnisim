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

#include "OmTextureTransform.hpp"
#include "OmMatrix4.hpp"
#include "OmSFDouble.hpp"
#include "OmWorld.hpp"

#include <cmath>

void OmTextureTransform::init() {
  mCenter = findSFVector2("center");
  mRotation = findSFDouble("rotation");
  mScale = findSFVector2("scale");
  mTranslation = findSFVector2("translation");
}

OmTextureTransform::OmTextureTransform(OmTokenizer *tokenizer) : OmBaseNode("TextureTransform", tokenizer) {
  init();
}

OmTextureTransform::OmTextureTransform(const OmTextureTransform &other) : OmBaseNode(other) {
  init();
}

OmTextureTransform::OmTextureTransform(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmTextureTransform::~OmTextureTransform() {
}

void OmTextureTransform::preFinalize() {
  OmBaseNode::preFinalize();

  updateCenter();
  updateRotation();
  updateScale();
  updateTranslation();
}

void OmTextureTransform::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mCenter, &OmSFVector2::changed, this, &OmTextureTransform::updateCenter);
  connect(mRotation, &OmSFDouble::changed, this, &OmTextureTransform::updateRotation);
  connect(mScale, &OmSFVector2::changed, this, &OmTextureTransform::updateScale);
  connect(mTranslation, &OmSFVector2::changed, this, &OmTextureTransform::updateTranslation);

  if (!OmWorld::instance()->isLoading())
    emit changed();
}

void OmTextureTransform::updateCenter() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmTextureTransform::updateRotation() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmTextureTransform::updateScale() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmTextureTransform::updateTranslation() {
  if (isPostFinalizedCalled())
    emit changed();
}

OmVector2 OmTextureTransform::transformUVCoordinate(const OmVector2 &uv) const {
  // D1.4: CPU re-implementation of WREN's TextureTransform matrix (X3D TextureTransform,
  // composed as -C * S * R * C * T with the GL-inverted v axis, exactly as the deleted
  // src/wren/TextureTransform.cpp built it). Consumed by the picking path and probed by
  // the wgpu collector to derive the affine uvA/uvB pair.
  const double cx = mCenter->x();
  const double cy = mCenter->y();
  const double sx = mScale->x();
  const double sy = mScale->y();
  const double rot = mRotation->value();

  const double ax = uv.x() + mTranslation->x() + cx;
  const double ay = uv.y() - mTranslation->y() - cy - 1.0;
  const double rx = cos(rot) * ax + sin(rot) * ay;
  const double ry = -sin(rot) * ax + cos(rot) * ay;
  return OmVector2(sx * rx - cx, sy * ry + cy + 1.0);
}

void OmTextureTransform::translate(const OmVector2 &offset) {
  const OmVector2 value = mTranslation->value() + offset;
  const OmVector2 intpart((int)value.x(), (int)value.y());
  mTranslation->setValueFromOmniSim(value - intpart);
}

QStringList OmTextureTransform::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "center"
         << "rotation"
         << "scale"
         << "translation";
  return fields;
}
