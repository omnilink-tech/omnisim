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

#include "OmGroup.hpp"

#include "OmBasicJoint.hpp"
#include "OmBoundingSphere.hpp"
#include "OmGeometry.hpp"
#include "OmNodeOperations.hpp"
#include "OmSlot.hpp"
#include "OmSolid.hpp"

using namespace OmHiddenKinematicParameters;

void OmGroup::init() {
  mHasNoSolidAncestor = true;
  mBoundingSphere = NULL;

  mChildren = findMFNode("children");
}

OmGroup::OmGroup(OmTokenizer *tokenizer) : OmBaseNode("Group", tokenizer) {
  init();
}

OmGroup::OmGroup(const QString &modelName, OmTokenizer *tokenizer) : OmBaseNode(modelName, tokenizer) {
  init();
}

OmGroup::OmGroup(const OmGroup &other) : OmBaseNode(other) {
  init();
}

OmGroup::OmGroup(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmGroup::~OmGroup() {
  delete mBoundingSphere;
}

void OmGroup::downloadAssets() {
  OmBaseNode::downloadAssets();
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmBaseNode *const n = static_cast<OmBaseNode *>(it.next());
    n->downloadAssets();
  }
}

void OmGroup::preFinalize() {
  OmBaseNode::preFinalize();

  if (isWorldRoot()) {
    emit worldLoadingStatusHasChanged(tr("Pre-finalizing nodes"));
    mLoadProgress = 0;
  }

  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    mLoadProgress++;
    OmBaseNode *const n = static_cast<OmBaseNode *>(it.next());
    if (!mHasNoSolidAncestor) {
      OmGroup *group = dynamic_cast<OmGroup *>(n);
      if (group)
        group->mHasNoSolidAncestor = false;
    }
    n->preFinalize();
    emit childFinalizationHasProgressed(mLoadProgress * 100 / (4 * childCount()));
    if (mFinalizationCanceled)
      return;
  }
}

void OmGroup::postFinalize() {
  OmBaseNode::postFinalize();

  if (isWorldRoot())
    emit worldLoadingStatusHasChanged(tr("Post-finalizing nodes"));

  recomputeBoundingSphere();

  connect(mChildren, &OmMFNode::changed, this, &OmGroup::childrenChanged);
  connect(mChildren, &OmMFNode::itemInserted, this, &OmGroup::insertChildPrivate);
  connect(mChildren, &OmMFNode::itemChanged, this, &OmGroup::insertChildPrivate);
  // if parent is a slot, it needs to be notified when a new node is inserted
  OmSlot *ps = dynamic_cast<OmSlot *>(parentNode());
  if (ps)
    connect(this, &OmGroup::notifyParentSlot, ps, &OmSlot::endPointInserted);
  // if parent is a joint, it needs to be notified when a new node is inserted
  const OmBasicJoint *pj = dynamic_cast<OmBasicJoint *>(parentNode());
  if (pj)
    connect(this, &OmGroup::notifyParentJoint, pj, &OmBasicJoint::endPointChanged);

  const OmGroup *const parent = dynamic_cast<const OmGroup *const>(parentNode());
  if (parent && parent->mHasNoSolidAncestor) {
    connect(mChildren, &OmMFNode::changed, this, &OmGroup::topLevelListsUpdateRequested);
    connect(this, &OmGroup::topLevelListsUpdateRequested, parent, &OmGroup::topLevelListsUpdateRequested);
  } else if (mHasNoSolidAncestor)
    connect(mChildren, &OmMFNode::changed, this, &OmGroup::topLevelListsUpdateRequested);
}

void OmGroup::recomputeBoundingSphere() {
  mBoundingSphere = new OmBoundingSphere(this);
  mBoundingSphere->empty();

  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    mLoadProgress++;
    OmBaseNode *const n = static_cast<OmBaseNode *>(it.next());
    if (!n->isPostFinalizedCalled())
      n->postFinalize();
    mBoundingSphere->addSubBoundingSphere(n->boundingSphere());
    emit childFinalizationHasProgressed(mLoadProgress * 100 / (4 * childCount()));
    if (mFinalizationCanceled)
      return;
  }
}

void OmGroup::insertChild(int index, OmNode *child) {
  child->setParentNode(this);
  mChildren->insertItem(index, child);
}

void OmGroup::setChild(int index, OmNode *child) {
  child->setParentNode(this);
  mChildren->setItem(index, child);
}

void OmGroup::addChild(OmNode *child) {
  child->setParentNode(this);
  mChildren->addItem(child);
}

void OmGroup::clear() {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmNode *const n = it.next();
    n->setParentNode(NULL);
  }
  mChildren->clear();
}

void OmGroup::deleteAllChildren() {
  mChildren->clear();
}

void OmGroup::deleteAllSolids() {
  OmMFNode::Iterator it(mChildren);
  QList<OmSolid *> solids;
  while (it.hasNext()) {
    OmNode *const n = it.next();
    // cppcheck-suppress constVariablePointer
    OmSolid *s = dynamic_cast<OmSolid *const>(n);
    if (s)
      solids << s;
    else {
      OmGroup *g = dynamic_cast<OmGroup *>(n);
      if (g)
        g->deleteAllSolids();
    }
  }
  foreach (OmSolid *s, solids)
    OmNodeOperations::instance()->deleteNode(s);
}

OmBaseNode *OmGroup::child(int index) const {
  return static_cast<OmBaseNode *>(mChildren->item(index));
}

int OmGroup::nodeIndex(OmNode *child) const {
  return mChildren->nodeIndex(child);
}

void OmGroup::createOdeObjects() {
  OmBaseNode::createOdeObjects();

  if (isWorldRoot())
    emit worldLoadingStatusHasChanged(tr("Creating ODE objects"));

  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    mLoadProgress++;
    static_cast<OmBaseNode *>(it.next())->createOdeObjects();
    emit childFinalizationHasProgressed(mLoadProgress * 100 / (4 * childCount()));
    if (mFinalizationCanceled)
      return;
  }
}

void OmGroup::createWrenObjects() {
  OmBaseNode::createWrenObjects();

  if (isWorldRoot())
    emit worldLoadingStatusHasChanged(tr("Creating WREN objects"));

  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    mLoadProgress++;
    static_cast<OmBaseNode *>(it.next())->createWrenObjects();
    emit childFinalizationHasProgressed(mLoadProgress * 100 / (4 * childCount()));
    if (mFinalizationCanceled)
      return;
  }
}

void OmGroup::cancelFinalization() {
  mFinalizationCanceled = true;
}

void OmGroup::propagateSelection(bool selected) {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<OmBaseNode *>(it.next())->propagateSelection(selected);
}

void OmGroup::setMatrixNeedUpdate() {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<OmBaseNode *>(it.next())->setMatrixNeedUpdate();
}

void OmGroup::setScaleNeedUpdate() {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<OmBaseNode *>(it.next())->setScaleNeedUpdate();
}

void OmGroup::updateCollisionMaterial(bool triggerChange, bool onSelection) {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmBaseNode *const n = static_cast<OmBaseNode *>(it.next());
    n->updateCollisionMaterial(triggerChange, onSelection);
  }
}

void OmGroup::setSleepMaterial() {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    static_cast<OmBaseNode *>(it.next())->setSleepMaterial();
}

bool OmGroup::isSuitableForInsertionInBoundingObject(bool warning) const {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    const OmBaseNode *const n = static_cast<OmBaseNode *>(it.next());
    if (n->isSuitableForInsertionInBoundingObject(warning) == false)
      return false;
  }

  return true;
}

bool OmGroup::isAValidBoundingObject(bool checkOde, bool warning) const {
  // Checks dimensions of bounding Geometries
  if (isSuitableForInsertionInBoundingObject(warning) == false)
    return false;

  OmMFNode::Iterator it(*mChildren);
  // Checks if there is at least one Geometry (with valid dimensions)
  while (it.hasNext()) {
    const OmBaseNode *const n = static_cast<OmBaseNode *>(it.next());
    if (n->isAValidBoundingObject(checkOde, false))
      return true;
  }

  return false;
}

void OmGroup::descendantNodeInserted(OmBaseNode *decendant) {
  if (!parentNode())
    return;

  OmGroup *pg = dynamic_cast<OmGroup *>(parentNode());
  if (pg) {
    pg->descendantNodeInserted(decendant);
    return;
  }

  if (dynamic_cast<OmBasicJoint *>(parentNode()))
    emit notifyParentJoint(decendant);
  else if (dynamic_cast<OmSlot *>(parentNode()))
    emit notifyParentSlot(decendant);
}

void OmGroup::insertChildFromSlotOrJoint(OmBaseNode *decendant) {
  descendantNodeInserted(decendant);
  emit childAdded(decendant);
}

void OmGroup::insertChildPrivate(int index) {
  OmBaseNode *childNode = static_cast<OmBaseNode *>(mChildren->item(index));
  if (childNode->isPostFinalizedCalled() && mBoundingSphere)
    mBoundingSphere->addSubBoundingSphere(childNode->boundingSphere());
  emit childAdded(childNode);
  descendantNodeInserted(childNode);

  if (isPostFinalizedCalled())
    connect(childNode, &OmBaseNode::finalizationCompleted, this, &OmGroup::monitorChildFinalization);
}

void OmGroup::monitorChildFinalization(OmBaseNode *child) {
  disconnect(child, &OmBaseNode::finalizationCompleted, this, &OmGroup::monitorChildFinalization);
  if (mBoundingSphere)
    mBoundingSphere->addSubBoundingSphere(child->boundingSphere());
  emit finalizedChildAdded(child);
}

bool OmGroup::shallExport() const {
  return !mChildren->isEmpty();
}

void OmGroup::reset(const QString &id) {
  OmBaseNode::reset(id);
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    it.next()->reset(id);
}

void OmGroup::save(const QString &id) {
  OmBaseNode::save(id);
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext())
    it.next()->save(id);
}

void OmGroup::forwardJerk() {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmGroup *const childGroup = dynamic_cast<OmGroup *>(it.next());
    if (childGroup)
      childGroup->forwardJerk();
  }
}

QList<const OmBaseNode *> OmGroup::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    const OmBaseNode *const childNode = static_cast<OmBaseNode *>(it.next());
    assert(childNode);
    list << childNode->findClosestDescendantNodesWithDedicatedWrenNode();
  }
  return list;
}

void OmGroup::updateSegmentationColor(const OmRgb &color) {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmBaseNode *const childNode = dynamic_cast<OmBaseNode *>(it.next());
    if (childNode)
      childNode->updateSegmentationColor(color);
  }
}

///////////////////
// Hidden fields //
///////////////////

bool OmGroup::restoreHiddenKinematicParameters(const HiddenKinematicParametersMap &map, int &counter) {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmGroup *const g = dynamic_cast<OmGroup *>(it.next());
    if (!g)
      continue;

    if (!g->restoreHiddenKinematicParameters(map, counter))
      return false;
  }

  return true;
}

bool OmGroup::resetHiddenKinematicParameters() {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    OmGroup *const g = dynamic_cast<OmGroup *>(it.next());
    if (!g)
      continue;
    if (!g->resetHiddenKinematicParameters())
      return false;
  }

  return true;
}

void OmGroup::collectHiddenKinematicParameters(HiddenKinematicParametersMap &map, int &counter) const {
  OmMFNode::Iterator it(*mChildren);
  while (it.hasNext()) {
    const OmGroup *const g = dynamic_cast<OmGroup *>(it.next());
    if (g)
      g->collectHiddenKinematicParameters(map, counter);
  }
}

void OmGroup::writeParameters(OmWriter &writer) const {
  if (!isProtoParameterNode()) {
    HiddenKinematicParametersMap map;
    int counter = 0;
    collectHiddenKinematicParameters(map, counter);
    const HiddenKinematicParametersMap::const_iterator end = map.constEnd();
    for (HiddenKinematicParametersMap::const_iterator i = map.constBegin(); i != end; ++i) {
      const HiddenKinematicParameters *const hkp = i.value();
      assert(hkp);
      const int solidIndex = i.key();
      const OmVector3 *const t = hkp->translation();
      if (t) {
        writer.writeFieldStart(QString("hidden translation_%1").arg(solidIndex), true);
        writer << *t;
        writer.writeFieldEnd(true);
      }
      const OmRotation *const r = hkp->rotation();
      if (r) {
        writer.writeFieldStart(QString("hidden rotation_%1").arg(solidIndex), true);
        writer << *r;
        writer.writeFieldEnd(true);
      }
      const PositionMap *const m = hkp->positions();
      if (m) {
        const PositionMap::const_iterator hkpEnd = m->constEnd();
        for (PositionMap::const_iterator it = m->constBegin(); it != hkpEnd; ++it) {
          const OmVector3 *const p = it.value();
          assert(p);
          const int jointIndex = it.key();
          for (int j = 0; j < 3; ++j) {
            const double pj = (*p)[j];
            if (!std::isnan(pj)) {
              QString axisIndex;
              if (j > 0)
                axisIndex.setNum(j + 1);
              writer.writeFieldStart(QString("hidden position%1_%2_%3").arg(axisIndex).arg(solidIndex).arg(jointIndex), true);
              writer << OmPrecision::doubleToString(pj, OmPrecision::DOUBLE_MAX);
              writer.writeFieldEnd(true);
            }
          }
        }
      }

      const OmVector3 *const l = hkp->linearVelocity();
      if (l) {
        writer.writeFieldStart(QString("hidden linearVelocity_%1").arg(solidIndex), true);
        writer << *l;
        writer.writeFieldEnd(true);
      }

      const OmVector3 *const a = hkp->angularVelocity();
      if (a) {
        writer.writeFieldStart(QString("hidden angularVelocity_%1").arg(solidIndex), true);
        writer << *a;
        writer.writeFieldEnd(true);
      }
    }
    qDeleteAll(map);
    map.clear();
  }

  OmNode::writeParameters(writer);
}

void OmGroup::readHiddenKinematicParameter(OmField *field) {
  createHiddenKinematicParameter(field, mHiddenKinematicParametersMap);
}

////////////
// Export //
////////////

void OmGroup::exportBoundingObjectToW3d(OmWriter &writer) const {
  assert(writer.isW3d());

  if (isUseNode() && defNode())
    writer << "<" << w3dName() << " role='boundingObject' USE=\'n" + QString::number(defNode()->uniqueId()) + "\'/>";
  else {
    writer << "<Group role='boundingObject'"
           << " id=\'n" << QString::number(uniqueId()) << "\'>";

    OmMFNode::Iterator it(*mChildren);
    while (it.hasNext()) {
      const OmNode *const childNode = static_cast<OmNode *>(it.next());
      childNode->write(writer);
    }

    writer << "</Group>";
  }
}
