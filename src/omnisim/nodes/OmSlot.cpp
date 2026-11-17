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

#include "OmSlot.hpp"

#include "OmBoundingSphere.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSolid.hpp"
#include "OmSolidReference.hpp"

void OmSlot::init() {
  // user fields
  mEndPoint = findSFNode("endPoint");
  mSlotType = findSFString("type");
}

OmSlot::OmSlot(OmTokenizer *tokenizer) : OmBaseNode("Slot", tokenizer) {
  init();
}

OmSlot::OmSlot(const OmSlot &other) : OmBaseNode(other) {
  init();
}

OmSlot::OmSlot(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmSlot::~OmSlot() {
}

void OmSlot::validateProtoNode() {
  OmSlot *slot = slotEndPoint();
  if (slot) {
    slot->validateProtoNode();
    return;
  }

  OmSolid *solid = solidEndPoint();
  if (solid)
    solid->validateProtoNode();
}

void OmSlot::downloadAssets() {
  OmBaseNode::downloadAssets();
  if (hasEndPoint())
    static_cast<OmBaseNode *>(endPoint())->downloadAssets();
}

void OmSlot::preFinalize() {
  OmBaseNode::preFinalize();

  connect(mEndPoint, &OmSFString::changed, this, &OmSlot::endPointChanged);
  OmGroup *pg = dynamic_cast<OmGroup *>(parentNode());
  if (pg)  // parent is a group
    connect(this, &OmSlot::endPointInserted, pg, &OmGroup::insertChildFromSlotOrJoint);
  OmSlot *ps = dynamic_cast<OmSlot *>(parentNode());
  if (ps)  // parent is another slot
    connect(this, &OmSlot::endPointInserted, ps, &OmSlot::endPointInserted);

  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e && !e->isPreFinalizedCalled())
    e->preFinalize();
}

void OmSlot::postFinalize() {
  OmBaseNode::postFinalize();

  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e && !e->isPostFinalizedCalled())
    e->postFinalize();

  connect(mSlotType, &OmSFString::changed, this, &OmSlot::updateType);
}

void OmSlot::updateType() {
  QString connectedType;
  const OmSlot *const parentSlot = dynamic_cast<OmSlot *>(parentNode());
  const OmSlot *const childSlot = slotEndPoint();
  if (parentSlot)
    connectedType = parentSlot->slotType();
  else if (childSlot)
    connectedType = childSlot->slotType();
  else
    return;

  QString errorMessage;
  if (!OmNodeUtilities::isSlotTypeMatch(slotType(), connectedType, errorMessage))
    parsingWarn(tr("Invalid 'type' changed to '%1': %2").arg(slotType()).arg(errorMessage));
}

OmSolid *OmSlot::solidEndPoint() const {
  return dynamic_cast<OmSolid *>(mEndPoint->value());
}

OmSolidReference *OmSlot::solidReferenceEndPoint() const {
  return dynamic_cast<OmSolidReference *>(mEndPoint->value());
}

OmSlot *OmSlot::slotEndPoint() const {
  return dynamic_cast<OmSlot *>(mEndPoint->value());
}

OmGroup *OmSlot::groupEndPoint() const {
  return dynamic_cast<OmGroup *>(mEndPoint->value());
}

void OmSlot::setEndPoint(OmNode *node) {
  OmBaseNode *const e = static_cast<OmBaseNode *>(node);
  mEndPoint->removeValue();
  mEndPoint->setValue(e);
  endPointChanged();
}

void OmSlot::createOdeObjects() {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->createOdeObjects();
}

void OmSlot::createWrenObjects() {
  OmBaseNode::createWrenObjects();
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->createWrenObjects();
}

void OmSlot::propagateSelection(bool selected) {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->propagateSelection(selected);
}

void OmSlot::setMatrixNeedUpdate() {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->setMatrixNeedUpdate();
}

void OmSlot::setScaleNeedUpdate() {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->setScaleNeedUpdate();
}

void OmSlot::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->updateCollisionMaterial(triggerChange, onSelection);
}

void OmSlot::setSleepMaterial() {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->setSleepMaterial();
}

OmBoundingSphere *OmSlot::boundingSphere() const {
  const OmBaseNode *const baseNode = static_cast<OmBaseNode *>(mEndPoint->value());
  if (baseNode)
    return baseNode->boundingSphere();

  return NULL;
}

void OmSlot::endPointChanged() {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e) {
    e->setParentNode(this);
    emit endPointInserted(e);
  }
}

QString OmSlot::endPointName() const {
  if (!mEndPoint->value())
    return QString();

  QString name = mEndPoint->value()->computeName();
  if (name.isEmpty())
    name = mEndPoint->value()->endPointName();
  return name;
}

void OmSlot::reset(const QString &id) {
  OmBaseNode::reset(id);

  OmNode *const e = mEndPoint->value();
  if (e)
    e->reset(id);
}

void OmSlot::save(const QString &id) {
  OmBaseNode::save(id);

  OmNode *const e = mEndPoint->value();
  if (e)
    e->save(id);
}

void OmSlot::updateSegmentationColor(const OmRgb &color) {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->updateSegmentationColor(color);
}

//////////////////////////////////////////////////////////////
//  WREN related methods for resizable OmGeometry children  //
//////////////////////////////////////////////////////////////

void OmSlot::attachResizeManipulator() {
  OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->attachResizeManipulator();
}

void OmSlot::detachResizeManipulator() const {
  const OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    e->detachResizeManipulator();
}

void OmSlot::write(OmWriter &writer) const {
  if (writer.isOmniSim())
    OmBaseNode::write(writer);
  else {
    if (hasEndPoint())
      mEndPoint->value()->write(writer);
  }
}

QList<const OmBaseNode *> OmSlot::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  const OmBaseNode *const e = static_cast<OmBaseNode *>(mEndPoint->value());
  if (e)
    list << e->findClosestDescendantNodesWithDedicatedWrenNode();
  return list;
}
