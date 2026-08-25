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

#include "OmPose.hpp"

#include "OmNodeUtilities.hpp"
#include "OmOdeContext.hpp"
#include "OmSolid.hpp"

void OmPose::init() {
  mPoseChangedSignalEnabled = false;

  // store position
  // note: this cannot be put into the preFinalize function because
  //       of the copy constructor last initialization
  if (nodeModelName() != "TrackWheel") {
    mSavedTranslations[stateId()] = translation();
    mSavedRotations[stateId()] = rotation();
  }
}

OmPose::OmPose(OmTokenizer *tokenizer) : OmGroup("Pose", tokenizer), OmAbstractPose(this) {
  init();
}

OmPose::OmPose(const OmPose &other) : OmGroup(other), OmAbstractPose(this) {
  init();
}

OmPose::OmPose(const OmNode &other) : OmGroup(other), OmAbstractPose(this) {
  init();
}

OmPose::OmPose(const QString &modelName, OmTokenizer *tokenizer) : OmGroup(modelName, tokenizer), OmAbstractPose(this) {
  init();
}

OmPose::~OmPose() {
  // D1.4: the dedicated WREN transform node died with the WREN renderer.
}

void OmPose::reset(const QString &id) {
  OmGroup::reset(id);
  // note: for solids, the set of these parameters has to occur only if mJointParents.size() == 0 and it is handled in
  // OmSolid::reset, otherwise it breaks the reset of hinge based joints
  if (nodeType() != WB_NODE_TRACK_WHEEL && !dynamic_cast<OmSolid *>(this)) {
    setTranslation(mSavedTranslations[id]);
    setRotation(mSavedRotations[id]);
  }
}

void OmPose::save(const QString &id) {
  OmGroup::save(id);
  if (nodeType() != WB_NODE_TRACK_WHEEL) {
    mSavedTranslations[id] = translation();
    mSavedRotations[id] = rotation();
  }
}

void OmPose::preFinalize() {
  OmGroup::preFinalize();
}

void OmPose::postFinalize() {
  OmGroup::postFinalize();

  connect(mTranslation, &OmSFVector3::changed, this, &OmPose::updateTranslation);
  connect(mTranslation, &OmSFVector3::changedByUser, this, &OmPose::translationOrRotationChangedByUser);
  if (!isInBoundingObject())
    connect(this, &OmPose::translationOrRotationChangedByUser, this, &OmPose::notifyJerk);
  connect(mRotation, &OmSFRotation::changed, this, &OmPose::updateRotation);
  connect(mRotation, &OmSFRotation::changedByUser, this, &OmPose::translationOrRotationChangedByUser);
}

void OmPose::updateTranslation() {
  OmAbstractPose::updateTranslation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomPosition();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void OmPose::updateRotation() {
  OmAbstractPose::updateRotation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomRotation();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void OmPose::updateTranslationAndRotation() {
  OmAbstractPose::updateTranslationAndRotation();

  if (isInBoundingObject() && isAValidBoundingObject(true))
    applyToOdeGeomRotation();

  if (mPoseChangedSignalEnabled)
    emit poseChanged();

  if (mHasNoSolidAncestor)
    forwardJerk();
}

void OmPose::notifyJerk() {
  OmSolid *s = upperSolid();
  if (s)
    s->notifyChildJerk(this);
}

void OmPose::emitTranslationOrRotationChangedByUser() {
  // supervisor = false, because this function is currently only called from the drag events.
  emit mTranslation->changedByUser(false);
  emit mRotation->changedByUser(false);
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void OmPose::createWrenObjects() {
  // D1.4: the dedicated WREN transform (and its translation/rotation pushes,
  // formerly OmAbstractPose::applyTranslationToWren/applyRotationToWren) is
  // gone; matrix state lives in OmAbstractPose and the wgpu path reads it
  // directly. Only the child recursion survives.
  OmBaseNode::createWrenObjects();

  const int size = children().size();
  for (int i = 0; i < size; ++i) {
    OmBaseNode *const n = child(i);
    n->createWrenObjects();
  }
}

void OmPose::setMatrixNeedUpdate() {
  OmAbstractPose::setMatrixNeedUpdateFlag();
  OmGroup::setMatrixNeedUpdate();
}

// WREN Material Methods

// for OmPose lying into a bounding object only
void OmPose::updateCollisionMaterial(bool isColliding, bool onSelection) {
  OmGeometry *const g = geometry();
  if (g)
    g->updateCollisionMaterial(isColliding, onSelection);
}

void OmPose::setSleepMaterial() {
  OmGeometry *const g = geometry();
  if (g)
    g->setSleepMaterial();
}

///////////////////////////////////////////////////////////////
//  ODE related methods for OmPoses in boundingObjects  //
///////////////////////////////////////////////////////////////

void OmPose::listenToChildrenField() {
  connect(childrenField(), &OmMFNode::itemChanged, this, &OmPose::createOdeGeom, Qt::UniqueConnection);
  connect(childrenField(), &OmMFNode::itemInserted, this, &OmPose::createOdeGeom, Qt::UniqueConnection);
  const OmGeometry *const g = geometry();
  if (g)
    connect(g, &OmGeometry::destroyed, this, &OmPose::createOdeGeomIfNeeded, Qt::UniqueConnection);
  const OmShape *const s = shape();
  if (s) {
    s->connectGeometryField();
    connect(s, &OmShape::geometryInShapeInserted, this, &OmPose::geometryInPoseInserted, Qt::UniqueConnection);
  }
}

void OmPose::createOdeGeomIfNeeded() {
  if (isBeingDeleted())
    return;

  if (childCount() != 0) {
    createOdeGeom();  // a physically inactive OmGeometry or OmShape pop up and the corresponding ODE dGeom has to be created
    // D1.4: the computeWrenRenderable() refresh that followed died with OmGeometry's WREN renderable.
  }
}

void OmPose::createOdeGeom(int index) {
  if (index > 0)
    return;

  if (index == 0 && childCount() > 1)
    destroyPreviousOdeGeoms();  // a child node was inserted at the first position in a non-empty list; if the second child
                                // contained a OmGeometry descendant then it has to be destroyed

  const OmShape *const s = shape();
  if (s) {
    s->connectGeometryField();
    connect(s, &OmShape::geometryInShapeInserted, this, &OmPose::geometryInPoseInserted, Qt::UniqueConnection);
  }

  emit geometryInPoseInserted();
}

void OmPose::destroyPreviousOdeGeoms() {
  OmNode *const secondChild = child(1);
  const OmShape *const s = dynamic_cast<OmShape *>(secondChild);
  OmGeometry *g = NULL;
  if (s) {
    g = s->geometry();
    s->disconnectGeometryField();
  } else
    g = dynamic_cast<OmGeometry *>(secondChild);
  if (g && g->odeGeom())
    g->destroyOdeObjects();  // D1.4: the deleteWrenRenderable() that followed is gone with OmGeometry's WREN renderable
}

bool OmPose::isSuitableForInsertionInBoundingObject(bool warning) const {
  if (childCount() == 0)
    return true;

  const OmGeometry *const g = geometry();
  return g ? g->isSuitableForInsertionInBoundingObject(warning) : true;
}

bool OmPose::isAValidBoundingObject(bool checkOde, bool warning) const {
  assert(isInBoundingObject());
  const int cc = childCount();

  if (cc == 0) {
    if (warning)
      parsingInfo(tr("A Transform placed in 'boundingObject' needs a Geometry or Shape as its first child to be valid."));
    return false;
  }

  if (cc > 1 && warning)
    parsingWarn(tr("A Transform placed inside a 'boundingObject' can only contain one child. Remaining children are ignored."));

  const OmGeometry *const g = geometry();
  if (g == NULL) {
    if (warning)
      parsingInfo(
        tr("The first child of a Transform placed in 'boundingObject' must be a Geometry or a Shape filled with a Geometry."));
    return false;
  }

  if (g->isAValidBoundingObject(checkOde, warning) == false)
    return false;

  if (checkOde && g->odeGeom() == NULL)
    return false;

  return true;
}

// Updates the ODE geometry: only for OmPose lying into a bounding object
void OmPose::applyToOdeData(bool correctMass) {
  OmGeometry *const g = geometry();
  if (g) {
    applyToOdeGeomPosition(false);   // updates the position relative to the Solid parent
    g->applyToOdeData(correctMass);  // updates the ODE dGeom itself
  }
}

void OmPose::applyToOdeGeomPosition(bool correctMass) {
  (void)correctMass;  // ODE is gone: no geom to place
}

void OmPose::applyToOdeGeomRotation() {
}

void OmPose::applyToOdeMass(OmGeometry *g, dGeomID geom) {
  (void)g;
  (void)geom;  // ODE is gone: no dMass bookkeeping
}

OmShape *OmPose::shape() const {
  if (childCount() == 0)
    return NULL;

  return dynamic_cast<OmShape *>(child(0));
}

////////////
// Export //
////////////

void OmPose::exportBoundingObjectToW3d(OmWriter &writer) const {
  assert(writer.isW3d());

  if (isUseNode() && defNode())
    writer << "<" << w3dName() << " role='boundingObject' USE=\'n" + QString::number(defNode()->uniqueId()) + "\'/>";
  else {
    writer << QString("<Pose translation='%1' rotation='%2' role='boundingObject'")
                .arg(translation().toString(OmPrecision::DOUBLE_MAX))
                .arg(rotation().toString(OmPrecision::DOUBLE_MAX))
           << " id=\'n" << QString::number(uniqueId()) << "\'>";
    ;

    OmMFNode::Iterator it(children());
    while (it.hasNext()) {
      const OmNode *const childNode = static_cast<OmNode *>(it.next());
      childNode->write(writer);
    }

    writer << "</Pose>";
  }
}

QStringList OmPose::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "translation"
         << "rotation";
  return fields;
}

OmVector3 OmPose::translationFrom(const OmNode *fromNode) const {
  const OmPose *parentNode = OmNodeUtilities::findUpperPose(this);
  const OmPose *childNode = this;
  QList<const OmPose *> poseList;

  poseList.append(childNode);
  while (parentNode != fromNode) {
    childNode = parentNode;
    parentNode = OmNodeUtilities::findUpperPose(parentNode);
    poseList.append(childNode);
    assert(parentNode);
  }

  const OmPose *previousPose = const_cast<OmPose *>(poseList.takeLast());
  OmVector3 translationResult = previousPose->translation();
  while (poseList.size() > 0) {
    const OmPose *pose = poseList.takeLast();
    translationResult += previousPose->rotation().toMatrix3() * pose->translation();
    previousPose = const_cast<OmPose *>(pose);
  }

  return translationResult;
}

OmMatrix3 OmPose::rotationMatrixFrom(const OmNode *fromNode) const {
  const OmPose *parentNode = OmNodeUtilities::findUpperPose(this);
  const OmPose *childNode = this;

  QList<const OmPose *> poseList;
  poseList.append(childNode);
  while (parentNode != fromNode) {
    childNode = parentNode;
    parentNode = OmNodeUtilities::findUpperPose(parentNode);
    poseList.append(childNode);
    assert(parentNode);
  }

  OmMatrix3 rotationResult = poseList.takeLast()->rotation().toMatrix3();
  while (poseList.size() > 0)
    rotationResult *= poseList.takeLast()->rotation().toMatrix3();

  return rotationResult;
}
