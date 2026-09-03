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

#include "OmTrack.hpp"

#include "OmAppearance.hpp"
#include "OmBrake.hpp"
#include "OmDictionary.hpp"
#include "OmField.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmLinearMotor.hpp"
#include "OmMFNode.hpp"
#include "OmMathsUtilities.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmPositionSensor.hpp"
#include "OmPrecision.hpp"
#include "OmRobot.hpp"
#include "OmSFInt.hpp"
#include "OmSFVector2.hpp"
#include "OmShape.hpp"
#include "OmSlot.hpp"
#include "OmTextureTransform.hpp"
#include "OmTrackWheel.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmDeformableFrameListener.hpp"

#include <cassert>

void OmTrack::init() {
  mDeviceField = findMFNode("device");
  mTextureAnimationField = findSFVector2("textureAnimation");
  mGeometryField = findSFNode("animatedGeometry");
  mGeometriesCountField = findSFInt("geometriesCount");

  mSurfaceVelocity = 0.0;
  mMotorPosition = 0.0;
  mLinearMotor = NULL;
  mBrake = NULL;

  // texture animation
  mShape = NULL;
  mTextureTransform = NULL;
  mSavedTextureTransformTranslations[stateId()] = OmVector2();

  // geometries animation
  mPathLength = 0.0;
  mPathStepSize = 0.0;
  mAnimationStepSize = 0;
}

// THE TRACK REGISTRY (P2). Same construction and the same reason as OmMuscle's and
// OmGranularGroup's: file-static, ctor/dtor maintained, so "does this world have a Track?"
// costs one empty-test on an existing vector and constructs nothing.
//
// ⚠ It is NOT OmDeformableFrameListener's track list. That list is a PENDING-REBUILD
// queue -- frameStarted() clears it after running it, because prePhysicsStep re-subscribes
// only on the steps where the belt actually moved -- so it is empty most of the time and
// answers a different question. anyDeformablesSubscribed() covers Cloth and SoftBody only.
static std::vector<OmTrack *> gLiveTracks;

bool OmTrack::anyTracks() {
  return !gLiveTracks.empty();
}

const std::vector<OmTrack *> &OmTrack::liveTracks() {
  return gLiveTracks;
}

OmTrack::OmTrack(OmTokenizer *tokenizer) : OmSolid("Track", tokenizer) {
  init();
  gLiveTracks.push_back(this);
}

OmTrack::OmTrack(const OmTrack &other) : OmSolid(other) {
  init();
  gLiveTracks.push_back(this);
}

OmTrack::OmTrack(const OmNode &other) : OmSolid(other) {
  init();
  gLiveTracks.push_back(this);
}

OmTrack::~OmTrack() {
  for (std::size_t i = 0; i < gLiveTracks.size(); ++i)
    if (gLiveTracks[i] == this) {
      gLiveTracks.erase(gLiveTracks.begin() + i);
      break;
    }
  OmDeformableFrameListener::instance()->unsubscribeTrack(this);
  clearWheelsList();
  clearAnimatedGeometries();
}

void OmTrack::wgpuBeltDraws(std::vector<WgpuBeltDraw> &out) const {
  // The legacy WREN node graph for the belt, reproduced as flat world matrices. Per element
  // i and animated shape j the chain was
  //
  //     track transform       the Track's own transform            == matrix()
  //       transform           T(bp.position.x, 0, bp.position.y) * R(+Y, bp.rotation)
  //         meshTransform     T+R extracted from geom->matrix() * matrix().pseudoInversed()
  //           geomTransform   the geometry's SCALE
  //             renderable    the geometry's mesh
  //
  // composed parent-to-child, so world = matrix() * beltTRS * geomMatrix * scale.
  // The scale is left to the caller because the wgpu Shape path already derives it from the
  // geometry's own fields (localScale).
  //
  // ⚠ `geom->matrix() * matrix().pseudoInversed()` IS REPRODUCED VERBATIM, INCLUDING ITS
  // ODDITY. For a rigid track pose M and a geometry world matrix M*P it evaluates to a pure
  // translation by R_track * p rather than the p a "geometry expressed in track-local
  // coordinates" reading would give -- i.e. the sub-pose offset is rotated once too often.
  // That is upstream's expression and it is what WREN draws; parity means matching it, not
  // correcting it, and correcting it here would move the wgpu image away from the WREN one
  // with no test able to say which is right.
  if (mAnimatedObjectList.isEmpty() || mBeltPositions.isEmpty())
    return;
  const OmMatrix4 trackWorld = matrix();
  const OmMatrix4 trackInv = trackWorld.pseudoInversed();
  const int n = qMin(mBeltPositions.size(), mGeometriesCountField->value());
  for (int i = 0; i < n; ++i) {
    const BeltPosition &bp = mBeltPositions[i];
    if (bp.segmentIndex < 0)
      continue;  // the abort marker computeNextGeometryPosition sets
    const OmMatrix4 beltTRS(OmVector3(bp.position.x(), 0.0, bp.position.y()),
                            OmRotation(0.0, 1.0, 0.0, bp.rotation));
    const OmMatrix4 base = trackWorld * beltTRS;
    for (int j = 0; j < mAnimatedObjectList.size(); ++j) {
      const AnimatedObject *ao = mAnimatedObjectList[j];
      if (!ao || !ao->geometry)
        continue;
      const OmMatrix4 geomMatrix = ao->geometry->matrix() * trackInv;
      // WREN kept only the position and orientation of that matrix, so any scale
      // in it was DROPPED. Rebuild it the same way rather than passing the raw
      // product through.
      const OmMatrix4 meshTransform(geomMatrix.translation(), OmRotation(geomMatrix.extracted3x3Matrix()));
      WgpuBeltDraw d;
      d.shape = ao->shape;
      d.geometry = ao->geometry;
      d.castShadows = ao->castShadows;
      d.world = base * meshTransform;
      out.push_back(d);
    }
  }
}

bool OmTrack::wgpuFirstBeltProbe(double &x, double &y, double &rotation) {
  for (std::size_t i = 0; i < gLiveTracks.size(); ++i) {
    const OmTrack *t = gLiveTracks[i];
    if (!t || t->mBeltPositions.isEmpty() || t->mAnimatedObjectList.isEmpty())
      continue;
    const BeltPosition &bp = t->mBeltPositions[0];
    x = bp.position.x();
    y = bp.position.y();
    rotation = bp.rotation;
    return true;
  }
  return false;
}

bool OmTrack::anyTextureAnimation() {
  for (std::size_t i = 0; i < gLiveTracks.size(); ++i) {
    const OmTrack *t = gLiveTracks[i];
    if (t && t->mTextureTransform && !t->mTextureAnimationField->value().isNull())
      return true;
  }
  return false;
}

void OmTrack::preFinalize() {
  OmSolid::preFinalize();

  updateDevices();

  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    OmLogicalDevice *device = dynamic_cast<OmLogicalDevice *>(it.next());
    device->preFinalize();
  }

  OmBaseNode *node = dynamic_cast<OmBaseNode *>(mGeometryField->value());
  if (node)
    node->preFinalize();
}

void OmTrack::createWrenObjects() {
  OmSolid::createWrenObjects();

  QList<OmShape *> shapeNodes;
  findAndConnectAnimatedGeometries(false, &shapeNodes);

  OmNode *node = mGeometryField->value();
  if (node != NULL)
    static_cast<OmBaseNode *>(node)->createWrenObjects();
}

void OmTrack::postFinalize() {
  OmSolid::postFinalize();

  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    OmLogicalDevice *device = dynamic_cast<OmLogicalDevice *>(it.next());
    device->postFinalize();
  }
  connect(mDeviceField, &OmMFNode::changed, this, &OmTrack::updateDevices);
  connect(mDeviceField, &OmMFNode::itemChanged, this, &OmTrack::addDevice);
  connect(mDeviceField, &OmMFNode::itemInserted, this, &OmTrack::addDevice);

  if (childCount() > 0) {
    OmGroup *group = dynamic_cast<OmGroup *>(child(0));
    if (group)
      connect(group, &OmGroup::childrenChanged, this, &OmTrack::updateChildren);
    if (mShape)
      connect(mShape, &OmShape::wrenMaterialChanged, this, &OmTrack::updateTextureTransform, Qt::UniqueConnection);
  }
  connect(this, &OmGroup::childrenChanged, this, &OmTrack::updateChildren);

  // animated geometries
  OmBaseNode *geometry = dynamic_cast<OmBaseNode *>(mGeometryField->value());
  if (geometry)
    geometry->postFinalize();
  updateAnimatedGeometriesPath();

  connect(mGeometryField, &OmSFNode::changed, this, &OmTrack::updateAnimatedGeometries);
  connect(mGeometriesCountField, &OmSFInt::changed, this, &OmTrack::updateAnimatedGeometries);

  for (int i = 0; i < mWheelsList.size(); ++i)
    connect(mWheelsList[i], &OmTrackWheel::changed, this, &OmTrack::updateAnimatedGeometriesPath, Qt::UniqueConnection);

  connect(mTextureAnimationField, &OmSFVector2::changed, this, &OmTrack::updateTextureAnimation);
}

void OmTrack::setMatrixNeedUpdate() {
  OmSolid::setMatrixNeedUpdate();

  OmNode *node = mGeometryField->value();
  if (node != NULL)
    static_cast<OmBaseNode *>(node)->setMatrixNeedUpdate();
}

void OmTrack::addDevice(int index) {
  OmRobot *const r = robot();
  assert(r);
  OmBaseNode *decendant = dynamic_cast<OmBaseNode *>(mDeviceField->item(index));
  r->descendantNodeInserted(decendant);
}

void OmTrack::reset(const QString &id) {
  OmSolid::reset(id);

  OmNode *const g = mGeometryField->value();
  if (g)
    g->reset(id);
  for (int i = 0; i < mDeviceField->size(); ++i)
    mDeviceField->item(i)->reset(id);

  mMotorPosition = 0.0;
  mSurfaceVelocity = 0.0;
  if (mTextureTransform)
    mTextureTransform->setTranslation(mSavedTextureTransformTranslations[id]);
}

void OmTrack::save(const QString &id) {
  OmSolid::save(id);

  OmNode *const g = mGeometryField->value();
  if (g)
    g->save(id);
  for (int i = 0; i < mDeviceField->size(); ++i)
    mDeviceField->item(i)->save(id);

  mSavedTextureTransformTranslations[id] = OmVector2();
  if (mShape && mShape->abstractAppearance()) {
    mTextureTransform = mShape->abstractAppearance()->textureTransform();
    if (mTextureTransform)
      mSavedTextureTransformTranslations[id] = mTextureTransform->translation();
  }
}

void OmTrack::updateDevices() {
  mLinearMotor = NULL;
  mBrake = NULL;
  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    OmNode *node = it.next();
    if (!mLinearMotor) {
      mLinearMotor = dynamic_cast<OmLinearMotor *>(node);
      if (mLinearMotor)
        continue;
    }
    if (!mBrake)
      mBrake = dynamic_cast<OmBrake *>(node);
  }
}

bool OmTrack::findAndConnectAnimatedGeometries(bool connectSignals, QList<OmShape *> *shapeList) {
  // cppcheck-suppress constVariablePointer
  OmBaseNode *geometry = dynamic_cast<OmBaseNode *>(mGeometryField->value());
  if (!geometry)
    return false;

  QList<OmBaseNode *> geometryNodes;
  geometryNodes.append(geometry);
  OmBaseNode *node = NULL;
  for (int i = 0; i < geometryNodes.size(); ++i) {
    node = geometryNodes[i];
    if (connectSignals && !node->isPostFinalizedCalled()) {
      connect(node, &OmBaseNode::finalizationCompleted, this, &OmTrack::updateAnimatedGeometriesAfterFinalization,
              Qt::UniqueConnection);
      return false;
    }

    OmShape *s = dynamic_cast<OmShape *>(node);
    if (s) {
      if (connectSignals) {
        // material automatically updated
        connect(s->geometryField(), &OmSFNode::changed, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
        connect(s, &OmShape::castShadowsChanged, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
        if (s->geometry())
          connect(s->geometry(), &OmGeometry::changed, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }
      if (shapeList != NULL)
        shapeList->append(s);
      continue;
    }

    OmGroup *g = dynamic_cast<OmGroup *>(node);
    if (g) {
      // group, pose or transform nodes
      if (connectSignals) {
        connect(g, &OmGroup::finalizedChildAdded, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
        connect(g->childrenField(), &OmMFNode::itemRemoved, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }
      for (int j = 0; j < g->childCount(); ++j)
        geometryNodes.append(g->child(j));

      OmPose *t = dynamic_cast<OmPose *>(g);
      if (t) {
        t->enablePoseChangedSignal();
        connect(t, &OmPose::poseChanged, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }

      continue;
    }

    const OmSlot *slot = dynamic_cast<OmSlot *>(node);
    if (slot) {
      const OmSlot *slot2 = slot->slotEndPoint();
      if (slot2) {
        // cppcheck-suppress constVariablePointer
        OmBaseNode *endPoint = dynamic_cast<OmBaseNode *>(slot2->endPoint());
        if (endPoint)
          geometryNodes.append(endPoint);
        connect(slot2->endPointField(), &OmSFNode::changed, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      }
      connect(slot->endPointField(), &OmSFNode::changed, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);
      continue;
    }

    // ignore invalid node
    parsingWarn(tr("Invalid %1 used in 'animatedGeometries' field.").arg(node->nodeModelName()));
  }

  return true;
}

void OmTrack::updateChildren() {
  // update texture animation
  if (childCount() == 0) {
    mShape = NULL;
    mTextureTransform = NULL;
  } else
    updateShapeNode();

  // update geometry animation
  updateWheelsList();
}

void OmTrack::updateShapeNode() {
  mShape = NULL;

  OmBaseNode *firstChild = child(0);
  mShape = dynamic_cast<OmShape *>(firstChild);
  if (!mShape) {
    const OmGroup *group = dynamic_cast<OmGroup *>(firstChild);
    if (group && group->children().size() > 0)
      mShape = dynamic_cast<OmShape *>(group->child(0));
  }
  updateTextureTransform();
}

void OmTrack::updateTextureTransform() {
  mTextureTransform = NULL;

  if (mShape && mShape->abstractAppearance()) {
    mTextureTransform = mShape->abstractAppearance()->textureTransform();
    if (mTextureTransform) {
      mSavedTextureTransformTranslations[stateId()] = mTextureTransform->translation();
      QList<OmNode *> useNodesList = OmVrmlNodeUtilities::findUseNodeAncestors(mTextureTransform);
      if (!useNodesList.isEmpty()) {
        mTextureTransform->parsingWarn(tr("Non-admissible TextureTransform USE node inside Track node."
                                          "This and ancestor USE nodes turned into DEF nodes: if texture animation enabled, "
                                          "the USE texture transform values will change independently from DEF node ones."));
        const int size = useNodesList.size();
        if (size > 0) {
          for (int i = 0; i < size; ++i)
            useNodesList[i]->makeDefNode();
          OmNodeOperations::instance()->updateDictionary(false, NULL);
        }
      }
    } else if (!mTextureAnimationField->value().isNull())
      mShape->abstractAppearance()->parsingWarn(
        tr("Texture animation is enabled only if the TextureTransform node is explicitly defined."));
    if (isPostFinalizedCalled())
      connect(mShape->abstractAppearance(), &OmAppearance::changed, this, &OmTrack::updateTextureTransform,
              Qt::UniqueConnection);
  }
}

void OmTrack::updateTextureAnimation() {
  if (!mTextureTransform && !mTextureAnimationField->value().isNull())
    parsingWarn(tr("Texture animation is enabled only if the TextureTransform node is explicitly defined."));
}

void OmTrack::updateWheelsList() {
  clearWheelsList();

  OmMFNode::Iterator it(*childrenField());
  OmTrackWheel *wheel = NULL;
  while (it.hasNext()) {
    wheel = dynamic_cast<OmTrackWheel *>(it.next());
    if (wheel) {
      mWheelsList.append(wheel);
      if (isPostFinalizedCalled())
        connect(wheel, &OmTrackWheel::changed, this, &OmTrack::updateAnimatedGeometriesPath, Qt::UniqueConnection);
    }
  }

  if (mWheelsList.isEmpty()) {
    mPathLength = 0.0;
    mPathList.clear();
    clearAnimatedGeometries();
  } else
    updateAnimatedGeometriesPath();
}

void OmTrack::clearWheelsList() {
  if (mWheelsList.isEmpty())
    return;

  for (int i = 0; i < mWheelsList.size(); ++i)
    disconnect(mWheelsList[i], &OmTrackWheel::changed, this, &OmTrack::updateAnimatedGeometriesPath);
  mWheelsList.clear();
}

QVector<OmLogicalDevice *> OmTrack::devices() const {
  QVector<OmLogicalDevice *> devices;
  // cppcheck-suppress constVariablePointer
  OmLogicalDevice *device = NULL;
  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    device = dynamic_cast<OmLogicalDevice *>(it.next());
    if (device)
      devices.append(device);
  }
  return devices;
}

OmPositionSensor *OmTrack::positionSensor() const {
  OmPositionSensor *s = NULL;
  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    s = dynamic_cast<OmPositionSensor *>(it.next());
    if (s)
      return s;
  }
  return NULL;
}

OmLinearMotor *OmTrack::motor() const {
  if (isPreFinalizedCalled())
    return mLinearMotor;

  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    OmLinearMotor *m = dynamic_cast<OmLinearMotor *>(it.next());
    if (m)
      return m;
  }
  return NULL;
}

OmBrake *OmTrack::brake() const {
  if (isPreFinalizedCalled())
    return mBrake;

  OmMFNode::Iterator it(*mDeviceField);
  while (it.hasNext()) {
    OmBrake *b = dynamic_cast<OmBrake *>(it.next());
    if (b)
      return b;
  }
  return NULL;
}

void OmTrack::updateAnimatedGeometriesPath() {
  if (!isPostFinalizedCalled())
    return;

  mPathLength = 0.0;
  mPathList.clear();

  if (mWheelsList.isEmpty()) {
    clearAnimatedGeometries();
    return;
  }

  computeBeltPath();

  if (mGeometriesCountField->value() > 0 && mAnimatedObjectList.isEmpty())
    updateAnimatedGeometries();
  else {
    initAnimatedGeometriesBeltPosition();
    OmDeformableFrameListener::instance()->subscribeTrack(this);
  }
}

void OmTrack::initAnimatedGeometriesBeltPosition() {
  const int numGeometries = mGeometriesCountField->value();
  mPathStepSize = mPathLength / numGeometries;
  BeltPosition beltPosition(mPathList[0].startPoint, mPathList[0].initialRotation, 0);
  mFirstGeometryPosition = beltPosition;
}

void OmTrack::updateAnimatedGeometriesAfterFinalization(OmBaseNode *node) {
  disconnect(node, &OmBaseNode::finalizationCompleted, this, &OmTrack::updateAnimatedGeometriesAfterFinalization);
  updateAnimatedGeometries();
}

void OmTrack::updateAnimatedGeometries() {
  clearAnimatedGeometries();

  if (mWheelsList.isEmpty())
    return;

  int numGeometries = mGeometriesCountField->value();
  const OmBaseNode *geometry = dynamic_cast<OmBaseNode *>(mGeometryField->value());
  if (numGeometries <= 0 || !geometry)
    return;

  QList<OmShape *> shapeNodes;
  bool success = findAndConnectAnimatedGeometries(isPostFinalizedCalled(), &shapeNodes);
  if (!success)
    return;

  for (int i = 0; i < shapeNodes.size(); ++i) {
    OmGeometry *geom = shapeNodes[i]->geometry();
    if (geom == NULL)
      continue;

    OmIndexedFaceSet *ifs = dynamic_cast<OmIndexedFaceSet *>(geom);
    // cppcheck-suppress knownConditionTrueFalse
    if (ifs)
      ifs->updateTriangleMesh();

    if (!geom->isPostFinalizedCalled())
      connect(geom, &OmBaseNode::finalizationCompleted, this, &OmTrack::updateAnimatedGeometriesAfterFinalization,
              Qt::UniqueConnection);
    else {
      AnimatedObject *object = new AnimatedObject();
      object->geometry = geom;
      object->shape = shapeNodes[i];  // P2: the wgpu appearance source
      object->castShadows = shapeNodes[i]->isCastShadowsEnabled();
      connect(shapeNodes[i], &OmShape::wrenMaterialChanged, this, &OmTrack::updateAnimatedGeometries, Qt::UniqueConnection);

      // D1.4: the source geometry needs no explicit hide any more -- the wgpu
      // children walk never reaches the `animatedGeometry` SFNode field, so the
      // belt copies are the only thing drawn (see the header note).
      mAnimatedObjectList.append(object);
    }
  }

  initAnimatedGeometriesBeltPosition();

  double stepSize = 0;
  mBeltPositions.reserve(numGeometries);
  BeltPosition beltPosition = mFirstGeometryPosition;

  for (int i = 0; i < numGeometries; ++i) {
    beltPosition = computeNextGeometryPosition(beltPosition, stepSize);
    mBeltPositions.append(beltPosition);
    if (beltPosition.segmentIndex < 0) {
      // abort
      clearAnimatedGeometries();
      return;
    }
    // D1.4: no per-element WREN transform tree is built any more; mBeltPositions
    // is the whole belt state and wgpuBeltDraws() turns it into model matrices.

    if (i == 0)
      stepSize = mPathStepSize;
  }

  OmDeformableFrameListener::instance()->subscribeTrack(this);
}

void OmTrack::clearAnimatedGeometries() {
  if (!mAnimatedObjectList.isEmpty())
    OmDeformableFrameListener::instance()->unsubscribeTrack(this);

  for (int i = 0; i < mAnimatedObjectList.size(); ++i)
    delete mAnimatedObjectList[i];

  mAnimatedObjectList.clear();
}

void OmTrack::prePhysicsStep(double ms) {
  OmSolid::prePhysicsStep(ms);

  const double sec = ms * 0.001;
  // ⚠ THE `&& mBodyID` GATE THAT USED TO BE HERE WAS PERMANENTLY FALSE, AND IT KILLED EVERY
  // TRACK ANIMATION IN THE TREE -- ON BOTH RENDERERS. mBodyID comes from OmSolid::body() ->
  // OmSolidMerger::body(), and OmSolidMerger::mBody is a `dBodyID` that is set to NULL in the
  // constructor and assigned NOWHERE ELSE since ODE was deleted (bdc02139). So the whole
  // branch was dead, mSurfaceVelocity was 0.0 on every step, and with it travelledDistance:
  // the belt never advanced, the TrackWheels never rotated, the `textureAnimation` scroll
  // never moved, and the LinearMotor's PositionSensor read 0.0 for ever. Measured 2026-08-22
  // on projects/samples/devices/worlds/track.omniworld: the leading belt element sat at its
  // authored path position (-0.15, 0.0535) for 900 rendered frames.
  //
  // Nothing here needs a body. The one thing that did -- FORCE control, which turned a force
  // into a surface velocity via ODE's dBodyGetMass -- is separately unimplemented and still
  // returns 0.0 below. Removing the gate is therefore the same class of fix as `e4fb9e822`
  // (call sites gated on a dead ODE field), and it has NO physics consumer to disturb:
  // contactSurfaceVelocity() has zero readers in the tree, because the ODE contact-surface-
  // velocity plumbing went with ODE. What comes back is the KINEMATIC animation and the
  // position readback.
  //
  // ⚠ It does NOT make a tracked robot DRIVE. Track propulsion under Newton is a separate,
  // still-open gap: the belt spins, the wheels turn, and the chassis does not move.
  if (mLinearMotor) {
    if (mLinearMotor->userControl()) {
      // ⚠ Track FORCE control (setForce on its LinearMotor) IS UNIMPLEMENTED. It
      // needed the belt body's mass to turn force into a surface velocity, and
      // that read was ODE's dBodyGetMass. Dropping it also removes a live
      // divide-by-zero: against the inert stub the mass came back 0.0 and
      // `force * sec / 0.0` fed inf/NaN into mSurfaceVelocity. Position and
      // velocity control (the else branch) are unaffected.
      mSurfaceVelocity = 0.0;
    } else
      // position or velocity control
      mSurfaceVelocity = -mLinearMotor->computeCurrentDynamicVelocity(ms, mMotorPosition);
  } else
    mSurfaceVelocity = 0.0;

  const double travelledDistance = mSurfaceVelocity * sec;
  mMotorPosition += travelledDistance;

  for (int i = 0; i < mWheelsList.size(); ++i)
    mWheelsList[i]->rotate(travelledDistance);

  // texture animation. D1.4: the translate() updates the TextureTransform's
  // own state, which the wgpu collect reads into the draw's UV affine (see
  // anyTextureAnimation()); the WREN material re-bind that used to follow is
  // gone.
  if (mTextureTransform)
    mTextureTransform->translate(-0.001 * ms * mSurfaceVelocity * mTextureAnimationField->value());

  // geometries animation
  if (!mAnimatedObjectList.isEmpty()) {
    mAnimationStepSize += travelledDistance;
    while (mAnimationStepSize > mPathStepSize)
      mAnimationStepSize -= mPathStepSize;
    while (mAnimationStepSize < -mPathStepSize)
      mAnimationStepSize += mPathStepSize;
    OmDeformableFrameListener::instance()->subscribeTrack(this);
  }
}

void OmTrack::animateMesh() {
  if (mAnimatedObjectList.isEmpty())
    return;

  double stepSize = mAnimationStepSize;
  mAnimationStepSize = 0;

  BeltPosition beltPosition = mFirstGeometryPosition;
  for (int i = 0; i < mGeometriesCountField->value(); ++i) {
    beltPosition = computeNextGeometryPosition(beltPosition, stepSize);
    mBeltPositions[i] = beltPosition;
    if (beltPosition.segmentIndex < 0) {
      // abort
      clearAnimatedGeometries();
      return;
    }

    // D1.4: mBeltPositions[i] IS the belt element's pose now -- the WREN
    // per-element transform writes are gone, and wgpuBeltDraws() reads this
    // list on the same frame.

    if (i == 0) {
      mFirstGeometryPosition = beltPosition;
      stepSize = mPathStepSize;
    }
  }
}

OmTrack::BeltPosition OmTrack::computeNextGeometryPosition(OmTrack::BeltPosition current, double stepSize,
                                                           bool segmentChanged) const {
  if (stepSize == 0)
    return current;

  const bool isPositiveStep = stepSize >= 0;
  const bool singleWheelCase = mWheelsList.size() == 1;
  const PathSegment segment = mPathList[current.segmentIndex];
  const OmVector2 endPoint = stepSize < 0 ? segment.startPoint : segment.endPoint;
  const OmVector2 maxDistanceVector = endPoint - current.position;
  double newStepSize = stepSize;

  if (singleWheelCase || (!maxDistanceVector.isNull() && maxDistanceVector.length() > 1e-10)) {
    double maxStepSize = 0.0;
    if (segment.radius < 0) {
      // straight
      maxStepSize = maxDistanceVector.length();
      if (singleWheelCase || fabs(stepSize) <= maxStepSize) {
        OmVector2 nextPosition = current.position + segment.increment * stepSize;
        return BeltPosition(nextPosition, segment.initialRotation, current.segmentIndex);
      }
    } else {
      // round
      OmVector2 relativePosition = current.position - segment.center;
      if (singleWheelCase)
        maxStepSize = fabs(stepSize);
      else {
        OmVector2 relativeEndPosition = endPoint - segment.center;
        OmVector2 c1, c2;
        if (isPositiveStep * segment.increment[0] > 0) {
          c1 = relativePosition;
          c2 = relativeEndPosition;
        } else {
          c1 = relativeEndPosition;
          c2 = relativePosition;
        }
        double angle = atan2(c2.x() * c1.y() - c2.y() * c1.x(), c2.x() * c1.x() + c2.y() * c1.y());
        if (angle < 0)
          angle += 2 * M_PI;

        maxStepSize = angle * segment.radius;
        if (maxStepSize < 0)
          maxStepSize += 2 * M_PI;
      }

      if (fabs(stepSize) <= maxStepSize) {
        const double angle = -(stepSize * segment.increment[0]) / segment.radius;
        const double newPositionX = relativePosition.x() * cos(angle) - relativePosition.y() * sin(angle);
        const double newPositionY = relativePosition.y() * cos(angle) + relativePosition.x() * sin(angle);
        const OmVector2 nextPosition = segment.center + OmVector2(newPositionX, newPositionY);
        double rotation = current.rotation - angle;
        if (segmentChanged) {
          if (!isPositiveStep) {
            int previousStep = current.segmentIndex + 1;
            if (previousStep == mPathList.size())
              previousStep = 0;
            rotation = mPathList[previousStep].initialRotation;
          } else
            rotation = segment.initialRotation;
          rotation -= angle;
        }
        return BeltPosition(nextPosition, rotation, current.segmentIndex);
      }
    }

    current.position = endPoint;
    current.rotation = 0.0;
    if (newStepSize < 0)
      newStepSize += maxStepSize;
    else
      newStepSize -= maxStepSize;
  }

  if (newStepSize != newStepSize) {  // NAN
    // abort generation
    parsingWarn(tr("Error during computation of Track animated geometries. "
                   "Please check the TrackWheel nodes in 'children' field."));
    return BeltPosition(OmVector2(), 0.0, -1);
  }

  int nextSegmentIndex = current.segmentIndex;
  if (isPositiveStep) {
    ++nextSegmentIndex;
    if (nextSegmentIndex == mPathList.size())
      nextSegmentIndex = 0;
  } else {
    --nextSegmentIndex;
    if (nextSegmentIndex == -1)
      nextSegmentIndex = mPathList.size() - 1;
  }
  current.segmentIndex = nextSegmentIndex;
  return computeNextGeometryPosition(current, newStepSize, true);
}

void OmTrack::computeBeltPath() {
  mPathLength = 0.0;

  int wheelsCount = mWheelsList.size();
  if (wheelsCount <= 0)
    return;

  OmVector2 center(mWheelsList[0]->position());
  double radius = mWheelsList[0]->radius();
  if (wheelsCount == 1) {
    // round path
    OmVector2 startPoint(center.x(), center.y() + radius);
    mPathList.append(PathSegment(startPoint, startPoint, 0.0, radius, center, OmVector2(1, 1)));
    mPathLength = 2 * M_PI * radius;
    return;
  }

  OmVector2 firstPoint, previousPoint, distanceVector;
  OmVector2 pointA, pointB;
  double previousRotation = 0.0;
  int nextIndex = 1;

  bool wheelsPositionError = false;
  for (int w = 0; w < wheelsCount; ++w) {
    if (w == wheelsCount - 1)
      nextIndex = 0;

    OmVector2 nextCenter = mWheelsList[nextIndex]->position();
    double nextRadius = mWheelsList[nextIndex]->radius();
    distanceVector = nextCenter - center;
    if (!wheelsPositionError && distanceVector.length() < 0.0000001) {
      wheelsPositionError = true;
      continue;
    }
    double wheelsAngle = atan2(distanceVector.y(), distanceVector.x());
    double absAngle = 0.0;
    bool isWheelInner = mWheelsList[w]->inner();
    bool isOuterTangent = isWheelInner == mWheelsList[nextIndex]->inner();
    if (isOuterTangent) {
      // outer tangent
      double relAngle = OmMathsUtilities::clampedAcos((radius - nextRadius) / distanceVector.length());
      assert(!std::isnan(relAngle));
      if (isWheelInner == 0)
        relAngle = -relAngle;
      absAngle = relAngle + wheelsAngle;
      pointA = OmVector2(cos(absAngle), sin(absAngle)) * radius + center;
      pointB = OmVector2(cos(absAngle), sin(absAngle)) * nextRadius + nextCenter;
    } else {
      // inner tangent
      double relAngle = OmMathsUtilities::clampedAcos((radius + nextRadius) / distanceVector.length());
      assert(!std::isnan(relAngle));
      if (isWheelInner == 0)
        relAngle = -relAngle;
      absAngle = relAngle + wheelsAngle;
      pointA.setXy(radius * cos(absAngle) + center.x(), radius * sin(absAngle) + center.y());
      pointB.setXy(nextRadius * cos(absAngle + M_PI) + nextCenter.x(), nextRadius * sin(absAngle + M_PI) + nextCenter.y());
    }

    if (w == 0)
      firstPoint = pointA;
    else {
      OmVector2 c1, c2, inc;
      if (isWheelInner == 1) {
        c1 = previousPoint - center;
        c2 = pointA - center;
        inc.setXy(1, 1);
      } else {
        c2 = previousPoint - center;
        c1 = pointA - center;
        inc.setXy(-1, -1);
      }
      double angle = atan2(c2.x() * c1.y() - c2.y() * c1.x(), c2.x() * c1.x() + c2.y() * c1.y());
      if (angle < 0)
        angle += 2 * M_PI;
      mPathLength += radius * angle;
      // round path
      mPathList.append(PathSegment(previousPoint, pointA, previousRotation, radius, center, inc));
    }

    // straight path
    distanceVector = pointB - pointA;
    mPathLength += distanceVector.length();
    if (fabs(distanceVector.y()) < 1e-10 && fabs(distanceVector.x()) < 1e-10) {
      OmVector2 centerDistanceVector = nextCenter - center;
      previousRotation = -atan2(centerDistanceVector.y(), centerDistanceVector.x()) - M_PI_2;
    } else
      previousRotation = -atan2(distanceVector.y(), distanceVector.x());
    mPathList.append(PathSegment(pointA, pointB, previousRotation, distanceVector.normalized()));

    previousPoint = pointB;
    center = nextCenter;
    radius = nextRadius;
    ++nextIndex;
  }

  // add last round path
  radius = mWheelsList[0]->radius();
  OmVector2 firstWheelCenter(mWheelsList[0]->position());
  OmVector2 c1(previousPoint - firstWheelCenter);
  OmVector2 c2(firstPoint - firstWheelCenter);
  double angle = atan2(c2.x() * c1.y() - c2.y() * c1.x(), c2.x() * c1.x() + c2.y() * c1.y());
  if (angle < 0)
    angle += 2 * M_PI;
  mPathLength += radius * angle;
  mPathList.append(PathSegment(previousPoint, firstPoint, previousRotation, radius, firstWheelCenter, OmVector2(1, 1)));

  if (wheelsPositionError)
    // multiple wheels at the same location
    parsingWarn(tr("Two or more consecutive TrackWheel nodes are located at the same position. "
                   "Only the first node is used."));
}

void OmTrack::exportAnimatedGeometriesMesh(OmWriter &writer) const {
  if (mAnimatedObjectList.size() == 0 || writer.isUrdf())
    return;

  const OmNode *node = mGeometryField->value();

  QString positionString =
    QString("%1").arg(OmPrecision::doubleToString(mBeltPositions[0].position.x(), OmPrecision::DOUBLE_MAX)) + " 0 " +
    QString("%1").arg(OmPrecision::doubleToString(mBeltPositions[0].position.y(), OmPrecision::DOUBLE_MAX));
  QString rotationString =
    QString("0 1 0 %1").arg(OmPrecision::doubleToString(mBeltPositions[0].rotation, OmPrecision::DOUBLE_MAX));

  if (writer.isW3d())
    writer << "<Pose role='animatedGeometry'>";
  else {
    writer.indent();
    writer << "Transform {\n";
    writer.increaseIndent();
    writer.indent();
    writer << "translation " << positionString << "\n";
    writer.indent();
    writer << "rotation " << rotationString << "\n";
    writer.indent();
    writer << "children [\n";
    writer.increaseIndent();
  }

  writer.indent();
  node->write(writer);

  if (writer.isW3d())
    writer << "</Pose>";
  else {
    writer.indent();
    writer << "]\n";
    writer.decreaseIndent();
    writer.indent();
    writer << "}\n";
  }
}

void OmTrack::exportNodeSubNodes(OmWriter &writer) const {
  if (writer.isOmniSim()) {
    OmSolid::exportNodeSubNodes(writer);
    return;
  }

  foreach (const OmField *field, fields()) {
    if (!field->isDeprecated() && (field->isW3d() && field->singleType() == WB_SF_NODE)) {
      const OmSFNode *const node = dynamic_cast<OmSFNode *>(field->value());
      if (node == NULL || node->value() == NULL || node->value()->shallExport()) {
        if (field->name() == "children")
          // export it manually in order to include animated geometries
          continue;

        if (writer.isW3d())
          field->value()->write(writer);
        else
          field->write(writer);
      }
    }
  }

  bool isEmpty = true;
  if (!writer.isW3d() && !writer.isUrdf()) {
    writer.indent();
    writer << "children [";
    writer.increaseIndent();
  }

  // write children nodes
  const OmBaseNode *subNode = NULL;
  for (int i = 0; i < childCount(); ++i) {
    subNode = child(i);
    if (subNode->shallExport()) {
      writer.writeMFSeparator(!isEmpty, false);
      subNode->write(writer);
      isEmpty = false;
    }
  }

  // write animated geometries
  if (!writer.isW3d() && !writer.isUrdf() && !isEmpty)
    writer << "\n";
  isEmpty |= mAnimatedObjectList.isEmpty();

  exportAnimatedGeometriesMesh(writer);

  if (!writer.isW3d() && !writer.isUrdf()) {
    writer.decreaseIndent();
    if (!isEmpty)
      writer.indent();
    writer << "]\n";
  }
}
