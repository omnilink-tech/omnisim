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

#include "OmAbstractAppearance.hpp"

#include "OmSFNode.hpp"
#include "OmTextureTransform.hpp"
#include "OmVector2.hpp"

#include <assimp/material.h>

void OmAbstractAppearance::init() {
  mName = findSFString("name");
  mTextureTransform = findSFNode("textureTransform");
  mNameValue = mName->value();
}

OmAbstractAppearance::OmAbstractAppearance(const QString &modelName, OmTokenizer *tokenizer) :
  OmBaseNode(modelName, tokenizer) {
  init();
}

OmAbstractAppearance::OmAbstractAppearance(const OmAbstractAppearance &other) : OmBaseNode(other) {
  init();
}

OmAbstractAppearance::OmAbstractAppearance(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmAbstractAppearance::OmAbstractAppearance(const QString &modelName, const aiMaterial *material) : OmBaseNode(modelName) {
  aiString nameString("PBRAppearance");
  material->Get(AI_MATKEY_NAME, nameString);
  mName = new OmSFString(QString(nameString.C_Str()));
  mNameValue = mName->value();

  mTextureTransform = new OmSFNode(NULL);
}

OmAbstractAppearance::~OmAbstractAppearance() {
  if (mIsShallowNode) {
    delete mName;
    delete mTextureTransform;
  }
}

void OmAbstractAppearance::preFinalize() {
  OmBaseNode::preFinalize();
  if (textureTransform())
    textureTransform()->preFinalize();

  updateTextureTransform();
}

void OmAbstractAppearance::postFinalize() {
  OmBaseNode::postFinalize();

  if (textureTransform())
    textureTransform()->postFinalize();

  connect(mName, &OmSFString::changed, this, &OmAbstractAppearance::updateName);
  connect(mTextureTransform, &OmSFNode::changed, this, &OmAbstractAppearance::updateTextureTransform);
}

void OmAbstractAppearance::reset(const QString &id) {
  OmBaseNode::reset(id);
  if (textureTransform())
    textureTransform()->reset(id);
}

void OmAbstractAppearance::createWrenObjects() {
  OmBaseNode::createWrenObjects();
  updateName();

  if (textureTransform())
    textureTransform()->createWrenObjects();
}

void OmAbstractAppearance::updateTextureTransform() {
  if (textureTransform())
    connect(textureTransform(), &OmTextureTransform::changed, this, &OmAbstractAppearance::updateTextureTransform,
            Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmAbstractAppearance::updateName() {
  if (isPostFinalizedCalled())
    emit nameChanged(mName->value(), mNameValue);
  mNameValue = mName->value();
}

OmTextureTransform *OmAbstractAppearance::textureTransform() const {
  return dynamic_cast<OmTextureTransform *>(mTextureTransform->value());
}

OmVector2 OmAbstractAppearance::transformUVCoordinate(const OmVector2 &uv) const {
  const OmTextureTransform *tt = textureTransform();
  if (tt)
    return tt->transformUVCoordinate(uv);
  return uv;
}
