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

//
//  OmMatter.cpp
//

#include "OmMatter.hpp"

#include "OmElevationGrid.hpp"
#include "OmField.hpp"
#include "OmGeometry.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmMFNode.hpp"
#include "OmMFVector3.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmMatrix4.hpp"
#include "OmNodeUtilities.hpp"
#include "OmOdeContext.hpp"
#include "OmPlane.hpp"
#include "OmResizeManipulator.hpp"
#include "OmRotation.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSolidUtilities.hpp"
#include "OmTranslateRotateManipulator.hpp"
#include "OmVector4.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#include <QtCore/QStringList>

bool OmMatter::cShowMatterCenter = false;

void OmMatter::init() {
  // Flags
  mBoundingObjectHasChanged = false;
  mSelected = false;
  mNeedToHandleJerk = false;

  // user fields
  mName = findSFString("name");
  mModel = findSFString("model");
  mDescription = findSFString("description");
  mBoundingObject = findSFNode("boundingObject");
  mLocked = findSFBool("locked");
}

OmMatter::OmMatter(const OmMatter &other) : OmPose(other) {
  init();
}

OmMatter::OmMatter(const OmNode &other) : OmPose(other) {
  init();
}

OmMatter::OmMatter(const QString &modelName, OmTokenizer *tokenizer) : OmPose(modelName, tokenizer) {
  init();
}

OmMatter::~OmMatter() {
  disconnectFromBoundingObjectUpdates(mBoundingObject->value());
}

void OmMatter::disconnectFromBoundingObjectUpdates(const OmNode *node) const {
  if (!node)
    return;

  const OmGroup *const group = dynamic_cast<const OmGroup *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (group) {
    for (int i = 0; i < group->childCount(); ++i)
      disconnectFromBoundingObjectUpdates(group->child(i));
    return;
  }

  const OmShape *const shape = dynamic_cast<const OmShape *>(node);
  if (shape) {
    disconnectFromBoundingObjectUpdates(shape->geometry());
    return;
  }

  const OmGeometry *const geometry = dynamic_cast<const OmGeometry *>(node);
  if (geometry)
    disconnect(geometry, &OmGeometry::boundingGeometryRemoved, this, &OmMatter::removeBoundingGeometry);
}

dSpaceID OmMatter::space() const {
  return OmOdeContext::instance()->space();
}

void OmMatter::postFinalize() {
  OmPose::postFinalize();

  if (mBoundingObject->value())
    boundingObject()->postFinalize();

  connectNameUpdates();
  // in case of follow solid option we need also to listen to parameter nodes (non instantiated) updates
  if (protoParameterNode()) {
    OmNode *parameter = protoParameterNode();
    while (parameter->protoParameterNode())
      parameter = parameter->protoParameterNode();
    const OmMatter *matter = dynamic_cast<OmMatter *>(parameter);
    if (matter)
      matter->connectNameUpdates();
  }

  connect(mLocked, &OmSFBool::changed, this, &OmMatter::updateLocked);
  connect(mBoundingObject, &OmSFNode::changed, this, &OmMatter::updateBoundingObject, Qt::UniqueConnection);
  connect(mModel, &OmSFString::changed, this, &OmMatter::matterModelChanged);
  updateManipulatorVisibility();
}

void OmMatter::setBoundingObject(OmNode *boundingObject) {
  mBoundingObject->removeValue();
  mBoundingObject->setValue(boundingObject);
}

void OmMatter::reset(const QString &id) {
  OmPose::reset(id);

  OmNode *const b = mBoundingObject->value();
  if (b)
    b->reset(id);
}

void OmMatter::save(const QString &id) {
  OmPose::save(id);

  OmNode *const b = mBoundingObject->value();
  if (b)
    b->save(id);
}

void OmMatter::connectNameUpdates() const {
  connect(mName, &OmSFString::changed, this, &OmMatter::updateName, Qt::UniqueConnection);
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void OmMatter::createWrenObjects() {
  OmPose::createWrenObjects();

  if (mBoundingObject->value())
    boundingObject()->createWrenObjects();
}

////////////////////////////
//   Create ODE Objects   //
////////////////////////////

bool OmMatter::isBoundingObjectFinalizationCompleted(OmBaseNode *node) {
  if (!node)
    return false;

  if (node->isPostFinalizedCalled())
    return true;

  connect(node, &OmBaseNode::finalizationCompleted, this, &OmMatter::boundingObjectFinalizationCompleted);
  return false;
}

void OmMatter::boundingObjectFinalizationCompleted(OmBaseNode *node) {
  disconnect(node, &OmBaseNode::finalizationCompleted, this, &OmMatter::boundingObjectFinalizationCompleted);
  updateBoundingObject();
}

dSpaceID OmMatter::groupSpace() const {
  const OmGroup *const gp = dynamic_cast<OmGroup *>(mBoundingObject->value());
  return gp ? gp->odeSpace() : NULL;
}

dSpaceID OmMatter::upperSpace() const {
  assert(areOdeObjectsCreated());
  dSpaceID s = groupSpace();
  if (s)
    return s;

  return NULL;  // ODE is gone: no collision space exists
}

dGeomID OmMatter::odeGeom() const {
  OmBaseNode *const bo = boundingObject();
  if (bo == NULL)
    return NULL;

  const OmGeometry *g = NULL;

  const OmPose *const p = dynamic_cast<OmPose *>(bo);
  // cppcheck-suppress knownConditionTrueFalse
  if (p)
    g = p->geometry();
  else {
    const OmShape *const s = dynamic_cast<OmShape *>(bo);
    g = s ? s->geometry() : dynamic_cast<OmGeometry *>(bo);
  }

  if (g)
    return g->odeGeom();

  const OmGroup *const gp = dynamic_cast<OmGroup *>(bo);
  if (gp)
    return (dGeomID)gp->odeSpace();

  return NULL;
}

dGeomID OmMatter::createOdeGeomFromGeometry(dSpaceID space, OmGeometry *geometry, bool setOdeData) {
  if (geometry == NULL)
    return NULL;

  dGeomID geom = geometry->createOdeGeom(space);

  if (geom && setOdeData) {
    // Stores a pointer to the ODE geometry into the OmGeometry node & sets the OmGeometry node and its OmMatter parent node as
    // reference data
    geometry->setOdeData(geom, this);
    connect(geometry, &OmGeometry::boundingGeometryRemoved, this, &OmMatter::removeBoundingGeometry, Qt::UniqueConnection);
  }

  return geom;
}

dGeomID OmMatter::createOdeGeomFromPose(dSpaceID space, OmPose *pose) {
  assert(space);

  // Listens to insertion/deletion in the children field of the OmPose
  connect(pose, &OmPose::geometryInPoseInserted, this, &OmMatter::createOdeGeomFromInsertedPoseItem, Qt::UniqueConnection);
  pose->listenToChildrenField();

  const int n = pose->childCount();
  if (n == 0) {
    parsingInfo(tr("A child to the Transform placed in 'boundingObject' is expected."));
    return NULL;
  }

  if (n != 1)
    pose->parsingWarn(tr("A Pose node inside a 'boundingObject' can only contain one child. Remaining children are ignored."));

  OmBaseNode *const poseChild = pose->child(0);
  const OmShape *const shape = dynamic_cast<OmShape *>(poseChild);
  if (shape) {
    const OmIndexedFaceSet *const ifs = dynamic_cast<OmIndexedFaceSet *>(shape->geometry());
    if (ifs)
      connect(ifs, &OmIndexedFaceSet::validIndexedFaceSetInserted, shape, &OmShape::geometryInShapeInserted,
              Qt::UniqueConnection);
    const OmElevationGrid *const eg = dynamic_cast<OmElevationGrid *>(shape->geometry());
    if (eg)
      connect(eg, &OmElevationGrid::validElevationGridInserted, shape, &OmShape::geometryInShapeInserted, Qt::UniqueConnection);
    connect(shape, &OmShape::geometryInShapeInserted, this, &OmMatter::createOdeGeomFromInsertedShapeItem,
            Qt::UniqueConnection);
    shape->connectGeometryField();
  } else if (dynamic_cast<OmGeometry *>(poseChild) == NULL) {
    pose->parsingWarn(tr("A Pose node inside a 'boundingObject' can only contain one Shape or one Geometry node. The child "
                         "node is ignored."));
  }

  OmGeometry *const geometry = pose->geometry();
  if (geometry == NULL)
    return NULL;

  const OmIndexedFaceSet *const ifs = dynamic_cast<OmIndexedFaceSet *>(geometry);
  // cppcheck-suppress knownConditionTrueFalse
  if (ifs)
    connect(ifs, &OmIndexedFaceSet::validIndexedFaceSetInserted, pose, &OmPose::geometryInPoseInserted, Qt::UniqueConnection);

  const OmElevationGrid *const eg = dynamic_cast<OmElevationGrid *>(geometry);
  if (eg)  // TODO: rename slot?
    connect(eg, &OmElevationGrid::validElevationGridInserted, pose, &OmPose::geometryInPoseInserted, Qt::UniqueConnection);

  return createOdeGeomFromGeometry(space, geometry);
}

void OmMatter::createOdeGeomFromInsertedPoseItem() {
  assert(dynamic_cast<OmPose *>(sender()));
  OmPose *const pose = static_cast<OmPose *>(sender());
  dGeomID g = createOdeGeomFromPose(upperSpace(), pose);
  if (g) {
    setGeomMatter(g, pose);
    if (isInsertedOdeGeomPositionUpdateRequired())
      updateOdeGeomPosition(g);
  }
}

void OmMatter::createOdeGeomFromInsertedShapeItem() {
  assert(dynamic_cast<OmShape *>(sender()));
  OmShape *const shape = static_cast<OmShape *>(sender());
  OmGeometry *const geometry = shape->geometry();
  OmPose *const pose = shape->upperPose();

  dGeomID insertedGeom;
  if (pose && pose->isInBoundingObject()) {
    insertedGeom = createOdeGeomFromPose(upperSpace(), pose);
    if (insertedGeom)
      setGeomMatter(insertedGeom);
  } else {  // no Pose in the boundingObject is a parent of this Shape
    OmIndexedFaceSet *const ifs = dynamic_cast<OmIndexedFaceSet *>(geometry);
    if (ifs)
      connect(ifs, &OmIndexedFaceSet::validIndexedFaceSetInserted, shape, &OmShape::geometryInShapeInserted,
              Qt::UniqueConnection);

    OmElevationGrid *const eg = dynamic_cast<OmElevationGrid *>(geometry);
    if (eg)
      connect(eg, &OmElevationGrid::validElevationGridInserted, shape, &OmShape::geometryInShapeInserted, Qt::UniqueConnection);

    insertedGeom = createOdeGeomFromGeometry(upperSpace(), geometry);
    if (insertedGeom == NULL) {
      assert(ifs || eg);
      return;
    }

    setGeomMatter(insertedGeom, geometry);
  }

  if (isInsertedOdeGeomPositionUpdateRequired())
    updateOdeGeomPosition(insertedGeom);
}

dGeomID OmMatter::createOdeGeomFromGroup(dSpaceID space, OmGroup *group) {  // group is a *OmGroup but not a *OmPose
  // ODE is gone: no collision geoms/spaces are created
  (void)space;
  (void)group;
  return NULL;
}

dGeomID OmMatter::createOdeGeomFromBoundingObject(dSpaceID space) {
  return createOdeGeomFromNode(space, boundingObject());
}

void OmMatter::insertValidGeometryInBoundingObject() {
  assert(dynamic_cast<OmIndexedFaceSet *>(sender()) || dynamic_cast<OmElevationGrid *>(sender()));
  OmGeometry *const geometry = static_cast<OmGeometry *>(sender());
  createOdeGeomFromGeometry(upperSpace(), geometry);
}

dGeomID OmMatter::createOdeGeomFromNode(dSpaceID space, OmBaseNode *node) {
  if (!node)
    return NULL;

  OmPose *const pose = dynamic_cast<OmPose *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (pose)
    return createOdeGeomFromPose(space, pose);

  OmGroup *const group = dynamic_cast<OmGroup *>(node);
  if (group)
    return createOdeGeomFromGroup(space, group);

  const OmShape *const shape = dynamic_cast<OmShape *>(node);
  if (shape) {
    const OmIndexedFaceSet *const ifs = dynamic_cast<OmIndexedFaceSet *>(shape->geometry());
    if (ifs)
      connect(ifs, &OmIndexedFaceSet::validIndexedFaceSetInserted, shape, &OmShape::geometryInShapeInserted,
              Qt::UniqueConnection);
    connect(shape, &OmShape::geometryInShapeInserted, this, &OmMatter::createOdeGeomFromInsertedShapeItem,
            Qt::UniqueConnection);
    shape->connectGeometryField();
  }

  OmGeometry *const geometry = OmSolidUtilities::geometry(node);
  if (!geometry)
    return NULL;  // the boundingObject is neither a OmGroup, OmShape nor a OmGeometry => ignored (maybe empty Shape)

  const OmIndexedFaceSet *const ifs = dynamic_cast<OmIndexedFaceSet *>(geometry);
  // cppcheck-suppress knownConditionTrueFalse
  if (ifs)
    connect(ifs, &OmIndexedFaceSet::validIndexedFaceSetInserted, this, &OmMatter::insertValidGeometryInBoundingObject,
            Qt::UniqueConnection);

  const OmElevationGrid *const eg = dynamic_cast<OmElevationGrid *>(geometry);
  if (eg)
    connect(eg, &OmElevationGrid::validElevationGridInserted, this, &OmMatter::insertValidGeometryInBoundingObject,
            Qt::UniqueConnection);

  return createOdeGeomFromGeometry(space, geometry);
}

bool OmMatter::handleJerkIfNeeded() {
  if (mNeedToHandleJerk) {
    mNeedToHandleJerk = false;
    handleJerk();
    return true;
  }
  return false;
}

/////////////////////
// Update Methods  //
/////////////////////

void OmMatter::updateTranslation() {
  OmPose::updateTranslation();

  // the translation of the OmMatter was changed through the GUI, a Supervisor or
  // automatically by Webots (if kinematic mode)
  mNeedToHandleJerk = true;
}

void OmMatter::updateRotation() {
  OmPose::updateRotation();

  // the rotation of the OmMatter was changed through the GUI, a Supervisor or
  // automatically by Webots (if kinematic mode)
  mNeedToHandleJerk = true;
}

void OmMatter::updateLineScale() {
  applyChangesToWren();
}

void OmMatter::updateName() {
  const QString &nameValue = mName->value();
  if (nameValue.isEmpty()) {
    const QString &defaultName = dynamic_cast<const OmSFString *>(findField("name")->defaultValue())->value();
    parsingWarn(tr("'name' cannot be empty. Default node name '%1' is automatically set.").arg(defaultName));
    mName->blockSignals(true);
    mName->setValue(defaultName);
    mName->blockSignals(false);
  }
  emit matterNameChanged();
}

// Places ODE dGeoms through their absolute coordinates
void OmMatter::updateOdePlaceableGeomPosition(dGeomID g) {
  (void)g;  // ODE is gone: nothing to place
}

void OmMatter::updateOdePlanePosition(dGeomID plane) {
  (void)plane;  // ODE is gone: nothing to place
}

void OmMatter::updateOdeGeomPosition(dGeomID g) {
  if (g == NULL)
    return;

  return;  // ODE is gone: no geoms to reposition
}

void OmMatter::updateLocked() {
  if (mLocked)
    detachResizeManipulator();

  updateManipulatorVisibility();
}

////////////
// Others //
////////////

OmBaseNode *OmMatter::boundingObject() const {
  return static_cast<OmBaseNode *>(mBoundingObject->value());
}

// Selection management
void OmMatter::select(bool selected) {
  if (mSelected == selected)
    return;

  mSelected = selected;
  propagateSelection(selected);
  if (!mSelected || cShowMatterCenter) {
    applyVisibilityFlagsToWren(selected);
    applyChangesToWren();
  }
}

// Method that checks the validity of a boundingObject
bool OmMatter::checkBoundingObject() const {
  if (boundingObject() == NULL)
    return false;

  const OmGroup *const group = dynamic_cast<OmGroup *>(boundingObject());
  if (group) {
    const OmMFNode &children = group->children();
    const int size = children.size();
    if (size == 0)
      return false;
  }

  return true;
}

// Collision and sleep flags management

void OmMatter::updateSleepFlag() {
  assert(boundingObject());
  mBoundingObjectHasChanged = false;
}

/////////////////////////////////////////////
//  Translation and Rotation manipulator   //
/////////////////////////////////////////////

void OmMatter::updateManipulatorVisibility() {
  if (mSelected) {
    if (isLocked() || OmNodeUtilities::isNodeOrAncestorLocked(this))
      detachTranslateRotateManipulator();
    else {
      updateTranslateRotateHandlesSize();
      attachTranslateRotateManipulator();
    }
  }
}
