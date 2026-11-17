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

#include "OmFog.hpp"

#include "OmFieldChecker.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFString.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"

#include <cassert>
#include <limits>

QList<OmFog *> OmFog::cFogList;

void OmFog::init() {
  mFogTypeMode = FOG_TYPE_LINEAR;

  mColor = findSFColor("color");
  mFogType = findSFString("fogType");
  mVisibilityRange = findSFDouble("visibilityRange");
}

OmFog::OmFog(OmTokenizer *tokenizer) : OmBaseNode("Fog", tokenizer) {
  init();
}

OmFog::OmFog(const OmFog &other) : OmBaseNode(other) {
  init();
}

OmFog::OmFog(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmFog::~OmFog() {
  cFogList.removeAll(this);

  foreach (OmFog *fog, cFogList)
    fog->updateAndConnectIfNeeded();

  if (!OmWorld::instance()->isCleaning())
    emit OmWrenRenderingContext::instance()->fogNodeDeleted();
}

void OmFog::preFinalize() {
  OmBaseNode::preFinalize();

  cFogList << this;

  if (isFirstInstance()) {
    updateColor();
    updateFogType();
    updateVisibilityRange();

    if (!OmWorld::instance()->isLoading())
      emit OmWrenRenderingContext::instance()->fogNodeAdded();
  } else
    parsingWarn(tr("Only one Fog node is allowed. Only the first Fog node will be taken into account."));
}

void OmFog::postFinalize() {
  OmBaseNode::postFinalize();

  if (isFirstInstance()) {
    connect(mColor, &OmSFColor::changed, this, &OmFog::updateColor);
    connect(mFogType, &OmSFString::changed, this, &OmFog::updateFogType);
    connect(mVisibilityRange, &OmSFDouble::changed, this, &OmFog::updateVisibilityRange);
  }
}

void OmFog::updateAndConnectIfNeeded() {
  if (isFirstInstance()) {
    updateColor();
    updateFogType();
    updateVisibilityRange();
    connect(mColor, &OmSFColor::changed, this, &OmFog::updateColor);
    connect(mFogType, &OmSFString::changed, this, &OmFog::updateFogType);
    connect(mVisibilityRange, &OmSFDouble::changed, this, &OmFog::updateVisibilityRange);
  }
}

void OmFog::updateColor() {
  if (OmFieldChecker::resetColorIfInvalid(this, mColor))
    return;
}

void OmFog::updateFogType() {
  const QString fogText(mFogType->value().toLower());
  if (fogText == QString("exponential"))
    mFogTypeMode = FOG_TYPE_EXPONENTIAL;
  else if (fogText == QString("exponential2"))
    mFogTypeMode = FOG_TYPE_EXPONENTIAL2;
  else
    mFogTypeMode = FOG_TYPE_LINEAR;

  if (mFogTypeMode == FOG_TYPE_LINEAR && fogText != QString("linear"))
    parsingWarn(tr("Unknown 'fogType': \"%1\". Set to \"LINEAR\"").arg(mFogType->value()));

  if (areWrenObjectsInitialized())
    emit modeChanged();
}

void OmFog::updateVisibilityRange() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mVisibilityRange, 0.0))
    return;
}

QStringList OmFog::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "color"
         << "fogType"
         << "visibilityRange";
  return fields;
}
