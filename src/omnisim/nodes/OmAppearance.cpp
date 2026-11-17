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

#include "OmAppearance.hpp"

#include "OmImageTexture.hpp"
#include "OmMFNode.hpp"
#include "OmMaterial.hpp"
#include "OmRgb.hpp"
#include "OmSFNode.hpp"
#include "OmTextureTransform.hpp"
#include "OmVector2.hpp"
#include "OmWorld.hpp"

void OmAppearance::init() {
  mMaterial = findSFNode("material");
  mTexture = findSFNode("texture");
}

OmAppearance::OmAppearance(OmTokenizer *tokenizer) : OmAbstractAppearance("Appearance", tokenizer) {
  init();
}

OmAppearance::OmAppearance(const OmAppearance &other) : OmAbstractAppearance(other) {
  init();
}

OmAppearance::OmAppearance(const OmNode &other) : OmAbstractAppearance(other) {
  init();
}

OmAppearance::~OmAppearance() {
}

void OmAppearance::downloadAssets() {
  OmBaseNode::downloadAssets();
  if (texture())
    texture()->downloadAssets();
}

void OmAppearance::preFinalize() {
  OmAbstractAppearance::preFinalize();

  if (material())
    material()->preFinalize();

  if (texture())
    texture()->preFinalize();

  updateMaterial();
  updateTexture();
}

void OmAppearance::postFinalize() {
  OmAbstractAppearance::postFinalize();

  if (material())
    material()->postFinalize();
  if (texture())
    texture()->postFinalize();

  connect(mMaterial, &OmSFNode::changed, this, &OmAppearance::updateMaterial);
  connect(mTexture, &OmSFNode::changed, this, &OmAppearance::updateTexture);
  if (!OmWorld::instance()->isLoading())
    emit changed();
}

void OmAppearance::reset(const QString &id) {
  OmAbstractAppearance::reset(id);

  OmNode *const m = mMaterial->value();
  if (m)
    m->reset(id);
  OmNode *const t = mTexture->value();
  if (t)
    t->reset(id);
}

void OmAppearance::updateMaterial() {
  if (material())
    connect(material(), &OmMaterial::changed, this, &OmAppearance::updateMaterial, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmAppearance::updateTexture() {
  if (texture())
    connect(texture(), &OmImageTexture::changed, this, &OmAppearance::updateTexture, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

OmMaterial *OmAppearance::material() const {
  return dynamic_cast<OmMaterial *>(mMaterial->value());
}

OmImageTexture *OmAppearance::texture() const {
  return dynamic_cast<OmImageTexture *>(mTexture->value());
}

void OmAppearance::createWrenObjects() {
  OmAbstractAppearance::createWrenObjects();

  if (material())
    material()->createWrenObjects();
  if (texture())
    texture()->createWrenObjects();
}

bool OmAppearance::isTextureLoaded() const {
  return (texture() && texture()->hasImage());
}

OmRgb OmAppearance::diffuseColor() const {
  if (material() && !isTextureLoaded())
    return material()->diffuseColor();
  else
    return OmRgb(1.0f, 1.0f, 1.0f);
}

void OmAppearance::pickColorInTexture(const OmVector2 &uv, OmRgb &pickedColor) const {
  OmImageTexture *tex = texture();
  if (tex) {
    OmVector2 uvTransformed = transformUVCoordinate(uv);
    tex->pickColor(uvTransformed, pickedColor);
  } else
    pickedColor.setValue(1.0, 1.0, 1.0);  // default value
}
