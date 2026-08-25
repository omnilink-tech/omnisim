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

#include "OmViewpoint.hpp"

#include "OmBoundingSphere.hpp"
#include "OmFieldChecker.hpp"
#include "OmGuiRefreshOracle.hpp"
#include "OmLensFlare.hpp"
#include "OmLight.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmMatrix3.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPreferences.hpp"
#include "OmRay.hpp"
#include "OmRenderBackend.hpp"
#include "OmRgb.hpp"
#include "OmSFDouble.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSolid.hpp"
#include "OmVector4.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"
#include "OmWrenRenderingContext.hpp"


#include <QtCore/QVariantAnimation>

#define ANIMATION_DURATION 1000

const double OmViewpoint::INCREASE_FACTOR = 1.1;
const double OmViewpoint::DECREASE_FACTOR = 0.9;

static const float DEFAULT_FAR = 1000000.0f;

void OmViewpoint::init() {
  static const double TAN_M_PI_8 = tan(M_PI_4 / 2.0);
  mAspectRatio = 1.0;
  mOrthographicViewHeight = 1.0;
  mFollowedSolid = NULL;
  mFollowedSolidPreviousPosition = OmVector3();
  mFollowedSolidReferenceRotation = OmMatrix3();
  mViewPointReferenceRotation = OmRotation();
  mReferenceOffset = OmVector3();
  mIsLocked = false;
  mRotationCenterIsLocked = false;
  mFieldOfViewY = M_PI_4;
  mTanHalfFieldOfViewY = TAN_M_PI_8;
  mFollowChangedBySelection = false;
  mFollowEmptiedByDestroyedSolid = false;
  mFollowChangedBySolidName = false;
  mNeedToUpdateFollowSolidState = false;
  mFromOrthographic = false;
  mTranslateAnimation = NULL;
  mRotateAnimation = NULL;
  mOrbitAnimation = NULL;
  mOrbitRadius = 0.0;
  mSavedFieldOfView[stateId()] = 0.0;
  mSavedFar[stateId()] = 0.0;
  mSavedOrthographicHeight[stateId()] = 0.0;
  mSavedNear[stateId()] = 0.0;
  mFinalOrbitTargetPostion = NULL;
  mInitialOrientationQuaternion = OmQuaternion();
  mInitialOrbitQuaternion = OmQuaternion();
  mFinalOrientationQuaternion = OmQuaternion();
  mFinalOrbitQuaternion = OmQuaternion();
  mLookAtInitialQuaternion = OmQuaternion();
  mLookAtFinalQuaternion = OmQuaternion();
  mSpaceQuaternion = OmQuaternion();
  mFieldOfView = findSFDouble("fieldOfView");
  mOrientation = findSFRotation("orientation");
  mPosition = findSFVector3("position");
  mDescription = findSFString("description");
  mNear = findSFDouble("near");
  mFar = findSFDouble("far");
  mExposure = findSFDouble("exposure");
  mFollow = findSFString("follow");
  mFollowType = findSFString("followType");
  mFollowSmoothness = findSFDouble("followSmoothness");
  mLensFlare = findSFNode("lensFlare");
  mAmbientOcclusionRadius = findSFDouble("ambientOcclusionRadius");
  mRenderBackend = findSFString("renderBackend");
  mBloomThreshold = findSFDouble("bloomThreshold");
  mProjectionMode = OmWrenRenderingContext::PM_PERSPECTIVE;
  mRotationCenter = OmVector3(mPosition->value());

  mNodeVisibilityEnabled = false;

  // backward compatibility
  OmSFBool *followOrientation = findSFBool("followOrientation");
  if (followOrientation->value()) {
    parsingWarn("Deprecated 'followOrientation' field, please use the 'followType' field instead.");
    if (mFollowType->value() == "Tracking Shot") {
      mFollowType->setValue("Mounted Shot");
      followOrientation->setValue(false);
    }
  }
}

OmViewpoint::OmViewpoint(OmTokenizer *tokenizer) : OmBaseNode("Viewpoint", tokenizer) {
  init();
}

OmViewpoint::OmViewpoint(const OmViewpoint &other) : OmBaseNode(other) {
  init();
}

OmViewpoint::OmViewpoint(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmViewpoint::~OmViewpoint() {
  if (areWrenObjectsInitialized())
    deleteWrenObjects();

  delete mFinalOrbitTargetPostion;
}

void OmViewpoint::preFinalize() {
  OmBaseNode::preFinalize();

  updateFieldOfView();
  updateNear();
  updateFar();

  if (lensFlare())
    lensFlare()->preFinalize();
}

// R2 of engine-migration-plan.md: parser-only renderBackend accessors.
// No call site consults these yet; they exist so R3+ can route this
// Viewpoint's draw submission through OmRenderBackendRegistry::resolve.
// Mirrors OmSolid::physicsBackendName / physicsBackend exactly.
const QString &OmViewpoint::renderBackendName() const {
  // F1: the field-missing fallback answers the DEFAULT renderer, which is wgpu ("wren" is
  // retired and only survives as an authored legacy value).
  static const QString kWgpu = QStringLiteral("wgpu");
  if (!mRenderBackend)
    return kWgpu;
  return mRenderBackend->value();
}

OmRenderBackend *OmViewpoint::renderBackend() const {
  QString nameStr = renderBackendName();
  // World-level default (default-flip-plan.md §3.2): a renderBackend left empty or "auto" defers to
  // WorldInfo.defaultRenderBackend, so a world can pin its renderer without editing every node. An
  // explicit per-node value is used as-is.
  bool fromWorldInfo = false;
  if (nameStr.isEmpty() || nameStr == QStringLiteral("auto")) {
    const OmWorldInfo *const wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
    if (wi != nullptr && !wi->defaultRenderBackend().isEmpty()) {
      nameStr = wi->defaultRenderBackend();
      fromWorldInfo = true;
    }
  }
  // F1 (wren-deletion-runbook.md): "wren" keeps PARSING (an undeclared field is an ERROR ->
  // headless exit 1, the Solid.immersionProperties precedent) but no longer selects the
  // legacy renderer -- warn once, naming this node, and resolve to wgpu below.
  if (!mWarnedRetiredWrenBackend && nameStr == QStringLiteral("wren")) {
    mWarnedRetiredWrenBackend = true;
    if (fromWorldInfo)
      warn(tr("WorldInfo.defaultRenderBackend \"wren\" is RETIRED: WREN is no longer selectable, so the main view "
              "now renders through wgpu. Remove the WorldInfo value (wgpu is the default)."));
    else
      warn(tr("renderBackend \"wren\" is RETIRED: WREN is no longer selectable, so the main view now renders "
              "through wgpu. Remove the field (wgpu is the default)."));
  }
  const QByteArray name = nameStr.toUtf8();
  return OmRenderBackendRegistry::resolve(OmRenderBackendKindFromString(name.constData()));
}

void OmViewpoint::postFinalize() {
  OmBaseNode::postFinalize();

  connect(mFieldOfView, &OmSFDouble::changed, this, &OmViewpoint::updateFieldOfView);
  connect(mOrientation, &OmSFRotation::changed, this, &OmViewpoint::updateOrientation);
  connect(mPosition, &OmSFVector3::changed, this, &OmViewpoint::updatePosition);
  connect(mNear, &OmSFDouble::changed, this, &OmViewpoint::updateNear);
  connect(mFar, &OmSFDouble::changed, this, &OmViewpoint::updateFar);
  connect(mExposure, &OmSFDouble::changed, this, &OmViewpoint::updateExposure);
  connect(mFollow, &OmSFString::changed, this, &OmViewpoint::updateFollow);
  connect(mFollowType, &OmSFString::changed, this, &OmViewpoint::updateFollowSolidState);
  connect(mFollowType, &OmSFString::changed, this, &OmViewpoint::updateFollowType);
  connect(mAmbientOcclusionRadius, &OmSFDouble::changed, this, &OmViewpoint::updateAmbientOcclusionRadius);
  connect(mBloomThreshold, &OmSFDouble::changed, this, &OmViewpoint::updateBloomThreshold);

  save(stateId());

  if (lensFlare())
    lensFlare()->postFinalize();

  startFollowUpFromField();
}

void OmViewpoint::deleteWrenObjects() {
}

void OmViewpoint::reset(const QString &id) {
  OmBaseNode::reset(id);

  OmNode *const l = mLensFlare->value();
  if (l)
    l->reset(id);

  mOrientation->setValue(mSavedOrientation[id]);
  mPosition->setValue(mSavedPosition[id]);
  resetAnimations();
  // we can't call 'updateFollowSolidState' here because the followed solid will probably moved
  mNeedToUpdateFollowSolidState = true;
  mEquilibriumVector.setXyz(0.0, 0.0, 0.0);
  mVelocity.setXyz(0.0, 0.0, 0.0);

  setNodesVisibility(mInvisibleNodes, true);

  mInvisibleNodes.clear();
}

/////////////
// Cleanup //
/////////////

void OmViewpoint::terminateFollowUp() {
  mFollowedSolid = NULL;
}

void OmViewpoint::emptyFollow() {
  mFollowedSolid = NULL;
  emit followInvalidated(false);          // turn off the follow object action at the OmView3D level
  mFollowEmptiedByDestroyedSolid = true;  // do nothing in mViewpoint when emitting the changed() signal of mFollow
  mFollow->setValue(QString());
  mFollowEmptiedByDestroyedSolid = false;
}

// Getters //
/////////////

bool OmViewpoint::isFollowed(const OmSolid *solid) const {
  return solid == mFollowedSolid;
}

float OmViewpoint::viewDistanceUnscaling(const OmVector3 &position) const {
  const OmVector3 eye(mPosition->value());
  const OmVector3 forward(mOrientation->value().direction().normalized());

  float w;
  if (mProjectionMode == OmWrenRenderingContext::PM_PERSPECTIVE) {
    const OmVector4 bottomRow(-forward.x(), -forward.y(), -forward.z(), forward.dot(eye));
    w = bottomRow.dot(OmVector4(position, 1.0f));

    // Remove scaling due to FOV
    w *= tanf(0.5f * mFieldOfView->value());
  } else {
    const float halfHeight = orthographicViewHeight() / 2.0f, width = halfHeight * aspectRatio();
    if (halfHeight > width)
      w = halfHeight;
    else
      w = width;
  }
  return w < 0.0 ? -w : w;
}

// Setters //
/////////////

void OmViewpoint::startFollowUp(OmSolid *solid, bool updateField) {
  if (solid == NULL)
    return;
  if (mFollowedSolid) {
    disconnect(mFollowedSolid, &OmMatter::destroyed, this, &OmViewpoint::emptyFollow);
    disconnect(mFollowedSolid, &OmMatter::matterNameChanged, this, &OmViewpoint::synchronizeFollowWithSolidName);
  }

  // only a node instance can be followed
  OmNode *node = solid;
  if (node->isProtoParameterNode())
    node = static_cast<OmBaseNode *>(node)->getFirstFinalizedProtoInstance();
  OmSolid *solidInstance = solid;
  if (node != solid) {
    solidInstance = dynamic_cast<OmSolid *>(node);
    if (solidInstance == NULL)
      // no valid solid instance found
      return;
  }

  mFollowedSolid = solidInstance;
  mEquilibriumVector.setXyz(0.0, 0.0, 0.0);
  mVelocity.setXyz(0.0, 0.0, 0.0);
  // listens and reacts to solid's name changes and solid's life cycle
  connect(mFollowedSolid, &OmMatter::destroyed, this, &OmViewpoint::emptyFollow);
  connect(mFollowedSolid, &OmMatter::matterNameChanged, this, &OmViewpoint::synchronizeFollowWithSolidName);
  updateFollowSolidState();
  connect(mPosition, &OmSFVector3::changed, this, &OmViewpoint::updateFollowSolidState);
  connect(mOrientation, &OmSFRotation::changed, this, &OmViewpoint::updateFollowSolidState);

  if (updateField)
    recomputeFollowField();
}

void OmViewpoint::updateFollowSolidState() {
  if (mFollowedSolid) {
    mFollowedSolidPreviousPosition = mFollowedSolid->position();
    mFollowedSolidReferenceRotation = mFollowedSolid->rotationMatrix();
    mViewPointReferenceRotation = mOrientation->value();
    mReferenceOffset = mPosition->value() - mFollowedSolid->position();
  }
}

void OmViewpoint::updateFollowType() {
  emit followTypeChanged(followStringToType(mFollowType->value()));
}

void OmViewpoint::updateAmbientOcclusionRadius() {
  OmFieldChecker::resetDoubleIfNegative(this, mAmbientOcclusionRadius, 2.0);
}

void OmViewpoint::updateBloomThreshold() {
  OmFieldChecker::resetDoubleIfNegativeAndNotDisabled(this, mBloomThreshold, 21.0, -1.0);
}

OmLensFlare *OmViewpoint::lensFlare() const {
  return dynamic_cast<OmLensFlare *>(mLensFlare->value());
}

void OmViewpoint::startFollowUpFromField() {
  OmSolid *solid = OmSolid::findSolidFromUniqueName(mFollow->value());
  if (solid != NULL)
    startFollowUp(solid, false);
}

void OmViewpoint::setFollowType(int followType) {
  disconnect(mFollowType, &OmSFBool::changed, this, &OmViewpoint::updateFollowType);
  mFollowType->setValue(followTypeToString(followType));
  connect(mFollowType, &OmSFBool::changed, this, &OmViewpoint::updateFollowType);
}

void OmViewpoint::recomputeFollowField() {
  if (mFollowedSolid == NULL)
    return;
  mFollowChangedBySelection = true;  // do nothing in mViewpoint when the changed() signal of mFollow is emitted
  mFollow->setValue(mFollowedSolid->computeUniqueName());
  mFollowChangedBySelection = false;
}

void OmViewpoint::synchronizeFollowWithSolidName() {
  mFollowChangedBySolidName = true;  // do nothing in mViewpoint when the changed() signal of mFollow is emitted
  mFollow->setValue(mFollowedSolid->computeUniqueName());
  mFollowChangedBySolidName = false;
}

void OmViewpoint::setOrthographicViewHeight(double ovh) {
  mOrthographicViewHeight = ovh;
  mSavedOrthographicHeight[stateId()] = ovh;
  updateOrthographicViewHeight();
}

void OmViewpoint::incOrthographicViewHeight() {
  mOrthographicViewHeight *= INCREASE_FACTOR;
  updateOrthographicViewHeight();
}

void OmViewpoint::decOrthographicViewHeight() {
  mOrthographicViewHeight *= DECREASE_FACTOR;
  updateOrthographicViewHeight();
}

void OmViewpoint::setNodesVisibility(QList<const OmBaseNode *> nodes, bool visible) {
  QListIterator<const OmBaseNode *> it(nodes);
  while (it.hasNext()) {
    const OmBaseNode *node = it.next();

    if (visible)
      mInvisibleNodes.removeAll(node);
    else if (!mInvisibleNodes.contains(node))
      mInvisibleNodes.append(node);
    emit nodeVisibilityChanged(node, visible);
  }
}

void OmViewpoint::enableNodeVisibility(bool enabled) {
  // apply action only if needed
  // and avoid enabling/disabling visibility multiple times in the same step in case of multiple cameras
  if (mNodeVisibilityEnabled == enabled)
    return;

  const int size = mInvisibleNodes.size();
  for (int i = 0; i < size; ++i) {
  }
  mNodeVisibilityEnabled = enabled;
}

void OmViewpoint::save(const QString &id) {
  OmBaseNode::save(id);
  mSavedNear[id] = mNear->value();
  mSavedFar[id] = mFar->value();
  mSavedFieldOfView[id] = mFieldOfView->value();
  mSavedPosition[id] = mPosition->value();
  mSavedOrientation[id] = mOrientation->value();
  mSavedDescription[id] = mDescription->value();
  mSavedFollow[id] = mFollow->value();
  recomputeFollowField();
}

void OmViewpoint::setPosition(const OmVector3 &position) {
  // will update and emit necessary signals
  mPosition->setValue(position);
}

void OmViewpoint::restore() {
  mNear->setValue(mSavedNear[stateId()]);
  mFar->setValue(mSavedFar[stateId()]);
  mFieldOfView->setValue(mSavedFieldOfView[stateId()]);
  mDescription->setValue(mSavedDescription[stateId()]);
  mFollow->setValue(mSavedFollow[stateId()]);

  if (mProjectionMode == OmWrenRenderingContext::PM_ORTHOGRAPHIC) {
    mOrthographicViewHeight = mSavedOrthographicHeight[stateId()];
    updateOrthographicViewHeight();
  }
  moveTo(mSavedPosition[stateId()], mSavedOrientation[stateId()]);
}

void OmViewpoint::lookAt(const OmVector3 &target, const OmVector3 &upVector) {
  // compute the forward vector from target to eye
  OmVector3 forward = mPosition->value() - target;
  forward.normalize();

  OmVector3 normalizedUpVector = upVector;
  normalizedUpVector.normalize();

  // don't bother looking if we're already looking at the object
  if (fabs(forward.dot(mOrientation->value().direction())) > 0.9999999)
    return;

  // compute the right vector
  OmVector3 right = normalizedUpVector.cross(forward);
  right.normalize();

  // recompute the orthonormal up vector
  OmVector3 up = forward.cross(right);
  OmQuaternion newLookAtQuaternion = OmQuaternion(-forward, -right, up);
  newLookAtQuaternion.normalize();
  OmRotation newOrientation = OmRotation(newLookAtQuaternion);
  mOrientation->setValue(newOrientation);
}

// Create //
////////////

void OmViewpoint::createWrenObjects() {
  OmBaseNode::createWrenObjects();
  // D1.4: the WREN viewport/camera and the corner coordinate-system overlay died with WREN;
  // the wgpu main view builds its camera from this node's fields every frame.
  updateFieldOfViewY();

  // once the viewpoint is created, update everything in the world instance
  OmWorld::instance()->setViewpoint(this);
}
/////////////////////
// updates methods //
/////////////////////

void OmViewpoint::updateFieldOfView() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(this, mFieldOfView, 0.0, M_PI, M_PI_2))
    return;

  updateFieldOfViewY();

  emit cameraParametersChanged();
}

void OmViewpoint::updateOrientation() {
  if (areWrenObjectsInitialized())
    emit cameraParametersChanged();
}

void OmViewpoint::updatePosition() {
  if (areWrenObjectsInitialized())
    emit cameraParametersChanged();
}

void OmViewpoint::updateNear() {
  if (OmFieldChecker::resetDoubleIfNonPositive(this, mNear, 0.05))
    return;

  if (mFar->value() > 0.0 and mFar->value() < mNear->value()) {
    mNear->setValue(mFar->value());
    parsingWarn(tr("'near' is greater than 'far'. Setting 'near' to %1.").arg(mNear->value()));
  }

}

void OmViewpoint::updateFar() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mFar, 0.0))
    return;

  if (mFar->value() > 0.0 and mFar->value() < mNear->value()) {
    mFar->setValue(mNear->value() + 1.0);
    parsingWarn(tr("'far' is less than 'near'. Setting 'far' to %1.").arg(mFar->value()));
    return;
  }

}

void OmViewpoint::updateExposure() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mExposure, 1.0))
    return;
  // D1.4: the wgpu tonemap reads exposure() live.
}

void OmViewpoint::updateAspectRatio(double renderWindowAspectRatio) {
  mAspectRatio = renderWindowAspectRatio;

  updateFieldOfViewY();

  emit cameraParametersChanged();
}

void OmViewpoint::updateOrthographicViewHeight() {
  emit cameraParametersChanged();
}

QString OmViewpoint::followTypeToString(int type) {
  if (type == FOLLOW_MOUNTED)
    return "Mounted Shot";
  else if (type == FOLLOW_PAN_AND_TILT)
    return "Pan and Tilt Shot";
  else if (type == FOLLOW_TRACKING)
    return "Tracking Shot";
  return "None";
}

int OmViewpoint::followStringToType(const QString &type) {
  if (type == "Tracking Shot")
    return FOLLOW_TRACKING;
  else if (type == "Mounted Shot")
    return FOLLOW_MOUNTED;
  else if (type == "Pan and Tilt Shot")
    return FOLLOW_PAN_AND_TILT;
  return FOLLOW_NONE;
}

void OmViewpoint::updateFieldOfViewY() {
  mTanHalfFieldOfViewY = tan(0.5 * mFieldOfView->value());  // stored for reuse in viewpointRay()

  // According to VRML standards, the meaning of mFieldOfView depends on the aspect ratio:
  // the view angle is taken with respect to the largest dimension
  if (mAspectRatio < 1.0)
    mFieldOfViewY = mFieldOfView->value();
  else {
    mTanHalfFieldOfViewY /= mAspectRatio;
    mFieldOfViewY = 2.0 * atan(mTanHalfFieldOfViewY);
  }
}

void OmViewpoint::updateFollow() {
  if (mFollowChangedBySelection || mFollowChangedBySolidName || mFollowEmptiedByDestroyedSolid)
    return;

  if (!mFollow->value().isEmpty()) {
    OmSolid *s = OmSolid::findSolidFromUniqueName(mFollow->value());
    if (s) {
      startFollowUp(s, false);
      emit followInvalidated(true);  // checks the follow object action at the OmView3D level
      return;
    }
    parsingWarn(tr("'follow' field is filled with an invalid Solid name."));
  }
  mFollowedSolid = NULL;
  emit followInvalidated(false);  // unchecks the follow object action at the OmView3D level
}

void OmViewpoint::updateFollowUp() {
  if (mNeedToUpdateFollowSolidState) {
    mNeedToUpdateFollowSolidState = false;
    updateFollowSolidState();
  }

  if (!mFollowedSolid)
    return;

  disconnect(mPosition, &OmSFVector3::changed, this, &OmViewpoint::updateFollowSolidState);
  disconnect(mOrientation, &OmSFRotation::changed, this, &OmViewpoint::updateFollowSolidState);

  //  Translates viewpoint according to solid displacement
  const OmVector3 &followedSolidCurrentPosition = mFollowedSolid->position();
  const OmVector3 delta(followedSolidCurrentPosition - mFollowedSolidPreviousPosition);
  mFollowedSolidPreviousPosition = followedSolidCurrentPosition;

  if (!mIsLocked) {
    int type = followStringToType(mFollowType->value());
    if (type == FOLLOW_PAN_AND_TILT)
      lookAt(mFollowedSolid->position(), OmWorld::instance()->worldInfo()->upVector());
    else if (type == FOLLOW_MOUNTED) {
      // Update Orientation
      OmMatrix3 solidRotation = mFollowedSolid->rotationMatrix() * mFollowedSolidReferenceRotation.transposed();
      OmRotation newOrientation = OmRotation(solidRotation * mViewPointReferenceRotation.toMatrix3());
      newOrientation.normalize();
      mOrientation->setValue(newOrientation);
      // Update Position (position is computed relatively to the solid)
      mPosition->setValue(mFollowedSolid->position() + solidRotation * mReferenceOffset);
    } else if (type == FOLLOW_TRACKING) {
      mEquilibriumVector += delta;
      // clang-format off
      // clang-format 11.0.0 is not compatible with previous versions with respect to nested conditional operators
      const double mass = ((mFollowSmoothness->value() < 0.05) ? 0.0 :
                           (mFollowSmoothness->value() > 1.0)  ? 1.0 :
                                                                 mFollowSmoothness->value());
      // clang-format on
      // If mass is 0, we instantly move the viewpoint to its equilibrium position.
      if (mass == 0.0) {
        // Moves the rotation point if a drag rotating the viewpoint is active
        if (!mRotationCenterIsLocked)
          mRotationCenter += mEquilibriumVector;

        mPosition->setValue(mPosition->value() + mEquilibriumVector);
        mVelocity.setXyz(0.0, 0.0, 0.0);
        mEquilibriumVector.setXyz(0.0, 0.0, 0.0);
      } else {  // Otherwise we apply a force and let physics do the rest.
        const double timeStep = OmWorld::instance()->worldInfo()->basicTimeStep() / 1000.0;
        const OmVector3 acceleration = mEquilibriumVector / mass;
        mVelocity += acceleration * timeStep;

        const double viewPointScalarVelocity = mVelocity.length();
        double followedObjectScalarVelocity;
        OmVector3 followedObjectVelocity;
        if (delta.length() > 0.0) {
          followedObjectVelocity = (delta / timeStep);
          followedObjectScalarVelocity = followedObjectVelocity.dot(mVelocity) / viewPointScalarVelocity;
        } else {
          followedObjectVelocity.setXyz(0.0, 0.0, 0.0);
          followedObjectScalarVelocity = 0.0;
        }

        // If the viewpoint is going faster than the followed object, we slow it down to avoid oscillations
        if (viewPointScalarVelocity > followedObjectScalarVelocity) {
          const double relativeSpeed = viewPointScalarVelocity - followedObjectScalarVelocity;
          if (relativeSpeed < 0.0001)
            mVelocity *= followedObjectScalarVelocity / viewPointScalarVelocity;
          else {
            double friction = 0.05 / mass;
            if (friction > 1.0)
              friction = 1.0;
            mVelocity *= (viewPointScalarVelocity - relativeSpeed * friction) / viewPointScalarVelocity;
          }
        }

        const OmVector3 deltaPosition(mVelocity * timeStep);
        mPosition->setValue(mPosition->value() + deltaPosition);
        // Moves the rotation point if a drag rotating the viewpoint is active
        if (!mRotationCenterIsLocked)
          mRotationCenter += deltaPosition;
        mEquilibriumVector -= deltaPosition;
      }
    }
  }

  connect(mPosition, &OmSFVector3::changed, this, &OmViewpoint::updateFollowSolidState);
  connect(mOrientation, &OmSFRotation::changed, this, &OmViewpoint::updateFollowSolidState);
}

const double OmViewpoint::X_OFFSET = 0.075;    // 7.5 percents of the width from the right side of the screen
const double OmViewpoint::Z_THRESHOLD = 0.05;  // rate of the near clipping distance from the 3D-immersed camera screen
const double OmViewpoint::X_REL_ORTHOGRAPHIC = 0.5 - X_OFFSET;

void OmViewpoint::setCoordinateSystemVisibility(bool visible) {
  // D1.4: the corner coordinate-system overlay was a WREN visual (already dark on the wgpu
  // default); the menu state is tracked by OmWrenRenderingContext's VF flag alone.
  (void)visible;
}
// Ray picking //
/////////////////

void OmViewpoint::viewpointRay(int x, int y, OmRay &ray) const {
  // Relative position of the picked pixel in the selected viewport with respect to viewport's center
  double w = ((double)x) / OmWrenRenderingContext::instance()->width() - 0.5;
  double h = ((double)y) / OmWrenRenderingContext::instance()->height() - 0.5;
  OmVector3 origin(mPosition->value());
  OmVector3 direction;
  const double nearValue = mNear->value();
  const OmMatrix3 &viewpointMatrix = mOrientation->value().toMatrix3();
  if (mProjectionMode == OmWrenRenderingContext::PM_PERSPECTIVE) {
    const double scaleFactor = 2.0 * nearValue * mTanHalfFieldOfViewY;
    // World position on camera's screen (we refer here to the world dimensions of camera's screen)
    w *= scaleFactor * mAspectRatio;  // right - left in openGL terms
    h *= scaleFactor;                 // top  - bottom in openGL terms
    // Origin and direction of the mouse ray intersecting with camera's screen
    direction = viewpointMatrix * OmVector3(-nearValue, w, h);
  } else {
    w *= mOrthographicViewHeight * mAspectRatio;
    h *= mOrthographicViewHeight;
    origin -= viewpointMatrix * OmVector3(0.0, w, h);
    direction = viewpointMatrix.column(0);
  }
  ray.redefine(origin, direction);
}

// Returns the intersection point of a casted (x, y)-pixel ray with the plane of equation z = z0 in within viewpoint's
// coordinate frame
OmVector3 OmViewpoint::pick(int x, int y, double z0) const {
  // Relative position of the picked pixel in the selected viewport with respect to viewport's center
  double w = ((double)x) / OmWrenRenderingContext::instance()->width() - 0.5;
  double h = ((double)y) / OmWrenRenderingContext::instance()->height() - 0.5;
  OmVector3 rayOrigin(mPosition->value());
  OmVector3 rayDirection;
  const double nearValue = mNear->value();
  const OmMatrix3 &viewpointMatrix = mOrientation->value().toMatrix3();
  const OmVector3 &cameraDirection = viewpointMatrix.column(0);
  if (mProjectionMode == OmWrenRenderingContext::PM_PERSPECTIVE) {
    const double scaleFactor = 2.0 * nearValue * mTanHalfFieldOfViewY;
    // World position on camera's screen (we refer here to the world dimensions of camera's screen)
    w *= scaleFactor * mAspectRatio;  // right - left in openGL terms
    h *= scaleFactor;                 // top  - bottom in openGL terms
    // Origin and direction of the mouse ray intersecting with camera's screen
    rayDirection = viewpointMatrix * OmVector3(-nearValue, w, h);
    const double factor = -z0 / nearValue;
    rayDirection *= factor;
  } else {
    w *= mOrthographicViewHeight * mAspectRatio;
    h *= mOrthographicViewHeight;
    rayOrigin -= viewpointMatrix * OmVector3(0.0, w, h);
    rayDirection = z0 * cameraDirection;
  }

  return rayOrigin + rayDirection;
}

// Converts absolute world coordinates of a 3D-point into screen pixel coordinates
void OmViewpoint::toPixels(const OmVector3 &pos, OmVector2 &P) const {
  const OmMatrix3 &viewpointMatrix = mOrientation->value().toMatrix3();
  OmVector3 eyePosition((pos - mPosition->value()) * viewpointMatrix);
  eyeToPixels(eyePosition, P);
}

// Converts absolute world coordinates of a two 3D-points into screen pixel coordinates
void OmViewpoint::toPixels(const OmVector3 &pos1, OmVector2 &P1, const OmVector3 &pos2, OmVector2 &P2) const {
  const OmMatrix3 &viewpointMatrix = mOrientation->value().toMatrix3();

  const OmVector3 &eyePosition1 = (pos1 - mPosition->value()) * viewpointMatrix;
  const OmVector3 &eyePosition2 = (pos2 - mPosition->value()) * viewpointMatrix;
  eyeToPixels(eyePosition1, P1);
  eyeToPixels(eyePosition2, P2);
}

// Converts screen coordinates to world coordinates
void OmViewpoint::toWorld(const OmVector3 &pos, OmVector3 &P) const {
  double zFar = mFar->value();
  if (zFar == 0)
    zFar = DEFAULT_FAR;

  double zNear = mNear->value();
  OmMatrix4 projection;
  if (mProjectionMode == OmWrenRenderingContext::PM_PERSPECTIVE) {
    OmMatrix4 perspective(1.0 / (mAspectRatio * mTanHalfFieldOfViewY), 0, 0, 0, 0, 1.0 / mTanHalfFieldOfViewY, 0, 0, 0, 0,
                          zFar / (zNear - zFar), -(zFar * zNear) / (zFar - zNear), 0, 0, -1, 0);
    projection = perspective;
  } else {
    const double halfHeight = mOrthographicViewHeight * 0.5;
    const double right = halfHeight * mAspectRatio, left = -right;
    const double top = halfHeight, bottom = -halfHeight;
    OmMatrix4 orthographic(2.0 / (right - left), 0, 0, -(right + left) / (right - left), 0, 2.0 / (top - bottom), 0,
                           -(top + bottom) / (top - bottom), 0, 0, -1.0 / (zFar - zNear), -zNear / (zFar - zNear), 0, 0, 0, 1);
    projection = orthographic;
  }

  OmVector3 eye = mPosition->value(), center = eye - mOrientation->value().direction(), up = mOrientation->value().up();

  OmVector3 f = (center - eye).normalized(), s = f.cross(up).normalized(), u = s.cross(f);

  OmMatrix4 view(-s.x(), -s.y(), -s.z(), s.dot(eye), u.x(), u.y(), u.z(), -u.dot(eye), f.x(), f.y(), f.z(), -f.dot(eye), 0, 0,
                 0, 1);

  OmMatrix4 inverse = projection * view;
  if (!inverse.inverse())
    return;

  OmVector4 screen(pos.x(), pos.y(), pos.z(), 1.0);
  screen = inverse * screen;
  screen /= screen.w();
  P.setXyz(screen.ptr());
}

// Converts eye coordinates of a 3D-point into screen pixel coordinates
void OmViewpoint::eyeToPixels(const OmVector3 &eyePosition, OmVector2 &P) const {
  const double x = eyePosition.x();
  if (x == 0.0) {
    P.setX(0.0);
    P.setY(0.0);
    return;
  }

  double w, h;
  if (mProjectionMode == OmWrenRenderingContext::PM_PERSPECTIVE) {
    const double factor = 0.5 / (x * mTanHalfFieldOfViewY);
    h = -factor * eyePosition.z();
    w = mAspectRatio != 0.0 ? -factor * eyePosition.y() / mAspectRatio : 0.0;
  } else {  // ORTHOGRAPHIC
    w = -eyePosition.y() / (mAspectRatio * mOrthographicViewHeight);
    h = -eyePosition.z() / mOrthographicViewHeight;
  }

  P.setX((w + 0.5) * OmWrenRenderingContext::instance()->width());
  P.setY((h + 0.5) * OmWrenRenderingContext::instance()->height());
}

// Retrieves the z-eye coordinate of a 3D-point defined by its world coordinates
double OmViewpoint::zEye(const OmVector3 &pos) const {
  const OmVector3 &direction = mOrientation->value().direction();
  return direction.dot(pos - mPosition->value());
}

bool OmViewpoint::moveViewpointToObject(OmBaseNode *node) {
  if (!node)
    return false;

  OmBoundingSphere *boundingSphere = OmNodeUtilities::boundingSphereAncestor(reinterpret_cast<OmNode *>(node));

  boundingSphere->recomputeIfNeeded(false);
  if (boundingSphere->isEmpty())
    // empty world
    return false;

  OmVector3 absoluteCenter;
  double radius;
  boundingSphere->computeSphereInGlobalCoordinates(absoluteCenter, radius);
  const OmVector3 boundingSphereCenter(absoluteCenter.x(), absoluteCenter.y(), absoluteCenter.z());

  // Compute direction vector where the viewpoint is looking at.
  // For all orientation and a zero angle, the viewpoint is looking at the x-axis.
  const OmVector3 viewpointDirection = mOrientation->value().toQuaternion() * OmVector3(1, 0, 0);

  // Compute a distance coefficient between the object and future viewpoint.
  // The bounding sphere will be entirely contained in the 3D view.
  // Use a slightly larger sphere to keep some space between the object and the 3D view borders
  radius *= 1.05;
  double distance = radius / (sin(mFieldOfView->value() / 2.0) * ((mAspectRatio <= 1.0) ? mAspectRatio : (1.0 / mAspectRatio)));

  // set a minimum distance
  if (distance < mNear->value() + radius)
    distance = mNear->value() + radius;

  // Compute new position. From the center of the object, move back the viewpoint along
  // its direction axis.
  const OmVector3 newViewpointPosition = boundingSphereCenter + viewpointDirection * (-distance);

  if (newViewpointPosition != mPosition->value()) {
    // move to target using eased animation
    OmWorld::instance()->setModified();
    moveTo(OmVector3(newViewpointPosition.x(), newViewpointPosition.y(), newViewpointPosition.z()), mOrientation->value());
    return true;
  }

  return false;
}

void OmViewpoint::southView() {
  orbitTo(OmVector3(0, -1, 0), OmRotation(0, 0, 1, M_PI_2));
}

void OmViewpoint::northView() {
  orbitTo(OmVector3(0, 1, 0), OmRotation(0, 0, 1, -M_PI_2));
}

void OmViewpoint::westView() {
  orbitTo(OmVector3(-1, 0, 0), OmRotation(0, 0, 1, 0));
}

void OmViewpoint::eastView() {
  orbitTo(OmVector3(1, 0, 0), OmRotation(0, 0, 1, -M_PI));
}

void OmViewpoint::topView() {
  orbitTo(OmVector3(0, 0, 1), OmRotation(-0.5773, 0.5773, 0.5773, 2.0944));
}

void OmViewpoint::bottomView() {
  orbitTo(OmVector3(0, 0, -1), OmRotation(0.5773, 0.5773, 0.5773, -2.0944));
}

OmAbstractPose *OmViewpoint::computeSelectedObjectPose() {
  OmBaseNode *node = OmSelection::instance()->selectedNode();
  assert(node);
  OmAbstractPose *pose = dynamic_cast<OmAbstractPose *>(node);
  if (!pose)
    pose = OmNodeUtilities::findUpperPose(node);
  return pose;
}

OmRotation OmViewpoint::computeObjectViewRotation(const OmRotation &rotation, const OmAbstractPose *selectedObject) {
  OmQuaternion q = rotation.toQuaternion();
  if (selectedObject)
    q = selectedObject->rotationMatrix().toQuaternion() * q;
  q.normalize();
  return OmRotation(q);
}

void OmViewpoint::objectFrontView() {
  const OmAbstractPose *pose = computeSelectedObjectPose();
  orbitTo(OmVector3(1, 0, 0), computeObjectViewRotation(OmRotation(0, 0, 1, -M_PI), pose), pose);
}

void OmViewpoint::objectBackView() {
  const OmAbstractPose *pose = computeSelectedObjectPose();
  orbitTo(OmVector3(-1, 0, 0), computeObjectViewRotation(OmRotation(0, 0, 1, 0), pose), pose);
}

void OmViewpoint::objectLeftView() {
  const OmAbstractPose *pose = computeSelectedObjectPose();
  orbitTo(OmVector3(0, 1, 0), computeObjectViewRotation(OmRotation(0, 0, 1, -M_PI_2), pose), pose);
}

void OmViewpoint::objectRightView() {
  const OmAbstractPose *pose = computeSelectedObjectPose();
  orbitTo(OmVector3(0, -1, 0), computeObjectViewRotation(OmRotation(0, 0, 1, M_PI_2), pose), pose);
}

void OmViewpoint::objectTopView() {
  const OmAbstractPose *pose = computeSelectedObjectPose();
  orbitTo(OmVector3(0, 0, 1), computeObjectViewRotation(OmRotation(-0.5773, 0.5773, 0.5773, 2.0944), pose), pose);
}

void OmViewpoint::objectBottomView() {
  const OmAbstractPose *pose = computeSelectedObjectPose();
  orbitTo(OmVector3(0, 0, -1), computeObjectViewRotation(OmRotation(0.5773, 0.5773, 0.5773, -2.0944), pose), pose);
}

void OmViewpoint::orbitTo(const OmVector3 &targetUnitVector, const OmRotation &targetRotation,
                          const OmAbstractPose *selectedObject) {
  resetAnimations();
  lock();

  OmWorld::instance()->setModified();

  // first, we need to calculate the orientation of the world as this will be applied to all orbits
  const OmVector3 &defaultUpVector = OmVector3(0, 0, 1);
  const OmVector3 &gravityUpVector = -OmWorld::instance()->worldInfo()->gravityUnitVector();
  if (gravityUpVector.dot(defaultUpVector) > 0.9999)
    // In the case of the gravity vector being the default create the identity quaternion
    mSpaceQuaternion = OmQuaternion();
  else if (gravityUpVector.dot(defaultUpVector) < -0.9999)
    // The gravity vector is the opposite of the default, so our transform is a vertical flip
    mSpaceQuaternion = OmQuaternion(OmVector3(0, 0, 1), M_PI);
  else {  // otherwise we can safely get a rotation axis using the cross product of both vectors
    mSpaceQuaternion = OmQuaternion(defaultUpVector.cross(gravityUpVector), gravityUpVector.angle(defaultUpVector));
    mSpaceQuaternion.normalize();
  }

  const OmNode *selectedNode = reinterpret_cast<OmNode *>(OmSelection::instance()->selectedNode());
  // for UX reasons, we want the default rotation height just above the floor,
  // meaning any horizontal view can see a floor at height 0
  OmVector3 centerToViewpoint;
  OmBoundingSphere *const boundingSphere = OmNodeUtilities::boundingSphereAncestor(selectedNode);
  // if an object is selected use its bounding sphere center to orbit around
  if (boundingSphere) {
    OmVector3 absoluteCenter;
    double unused;  // passed to computeSphereInGlobalCoordinates but not needed
    boundingSphere->computeSphereInGlobalCoordinates(absoluteCenter, unused);
    mRotationCenter = absoluteCenter;
    centerToViewpoint = mPosition->value() - mRotationCenter;
  } else {
    mRotationCenter = OmVector3();
    centerToViewpoint = mPosition->value();
  }
  // preserve the original distance to object / world center
  // the orbit radius is only updated if the last animation completed successfully
  if (mOrbitRadius == 0.0)
    mOrbitRadius = centerToViewpoint.length();

  if (boundingSphere && selectedObject) {
    delete mFinalOrbitTargetPostion;
    mFinalOrbitTargetPostion =
      new OmVector3(mRotationCenter + selectedObject->rotationMatrix() * targetUnitVector * mOrbitRadius);
  }

  mCenterToViewpointUnitVector = centerToViewpoint / mOrbitRadius;
  mOrbitTargetUnitVector = mSpaceQuaternion * targetUnitVector;
  mInitialOrientationQuaternion = mSpaceQuaternion * OmQuaternion(mOrientation->value().axis(), mOrientation->value().angle());
  mFinalOrientationQuaternion = mSpaceQuaternion * OmQuaternion(targetRotation.axis(), targetRotation.angle());
  mInitialOrientationQuaternion.normalize();
  mFinalOrientationQuaternion.normalize();

  if (fabs(mInitialOrientationQuaternion.dot(mFinalOrientationQuaternion)) < 0.99994)
    animateLookAtIfNeeded();
  else
    // otherwise initial and final orientations are already the same
    resetAnimations();
}

void OmViewpoint::animateLookAtIfNeeded() {
  lock();
  lockRotationCenter();

  mLookAtInitialQuaternion = OmQuaternion(mOrientation->value().axis(), mOrientation->value().angle());
  // find out where we're going to be looking
  lookAt(mRotationCenter, mOrientation->value().up());
  // get this as a quaternion
  mLookAtFinalQuaternion = OmQuaternion(mOrientation->value().axis(), mOrientation->value().angle());
  // reset viewpoint to where it was just before
  mLookAtInitialQuaternion.normalize();
  mLookAtFinalQuaternion.normalize();
  mOrientation->setValue(OmRotation(mLookAtInitialQuaternion));

  if (fabs(mLookAtInitialQuaternion.dot(mLookAtFinalQuaternion)) < 0.99994) {
    mRotateAnimation = new QVariantAnimation(this);
    mRotateAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
    mRotateAnimation->setDuration(ANIMATION_DURATION / 2);
    mRotateAnimation->setStartValue(0.0);
    mRotateAnimation->setEndValue(1.0);
    connect(mRotateAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::lookAtAnimationStep);
    connect(mRotateAnimation, &QVariantAnimation::finished, this, &OmViewpoint::firstOrbitStep);
    // we can safely delete this animation when stopped
    mRotateAnimation->start(QAbstractAnimation::DeleteWhenStopped);
  } else {
    firstOrbitStep();
  }
}

void OmViewpoint::firstOrbitStep() {
  // no need to lock the viewpoint or its rotation center, they're already locked
  const double angleBetweenStartAndFinish = mCenterToViewpointUnitVector.angle(mOrbitTargetUnitVector);
  OmVector3 orbitAxis;
  // choose prefereable axes for axis-to-axis rotations
  if (mCenterToViewpointUnitVector.dot(mOrbitTargetUnitVector) < -0.99) {
    if ((mSpaceQuaternion.conjugated() * mOrbitTargetUnitVector).y() == 0.0)
      orbitAxis = mSpaceQuaternion * OmVector3(0, 1, 0);
    else
      orbitAxis = mSpaceQuaternion * OmVector3(0, 0, 1);
  } else
    orbitAxis = mOrbitTargetUnitVector.cross(mCenterToViewpointUnitVector).normalized();

  mInitialOrbitQuaternion = OmQuaternion(orbitAxis, 0.0);
  mFinalOrbitQuaternion = OmQuaternion(orbitAxis, angleBetweenStartAndFinish);

  mOrbitAnimation = new QVariantAnimation(this);
  mOrbitAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
  mOrbitAnimation->setDuration(ANIMATION_DURATION);
  mOrbitAnimation->setStartValue(0.0);
  mOrbitAnimation->setEndValue(1.0);
  connect(mOrbitAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::translateOrbitAnimationStep);
  connect(mOrbitAnimation, &QVariantAnimation::finished, this, &OmViewpoint::secondOrbitStep);
  mOrbitAnimation->start();

  mRotateAnimation = new QVariantAnimation(this);
  mRotateAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
  mRotateAnimation->setDuration(ANIMATION_DURATION);
  mRotateAnimation->setStartValue(0.0);
  mRotateAnimation->setEndValue(0.5);
  connect(mRotateAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::rotateOrbitAnimationStep);
  mRotateAnimation->start();
}

void OmViewpoint::secondOrbitStep() {
  resetAnimations();
  lock();

  bool skipped = true;
  mInitialOrientationQuaternion = OmQuaternion(mOrientation->value().axis(), mOrientation->value().angle());
  if (fabs(mInitialOrientationQuaternion.dot(mFinalOrientationQuaternion)) < 0.99994) {
    mRotateAnimation = new QVariantAnimation(this);
    mRotateAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
    mRotateAnimation->setDuration(ANIMATION_DURATION);
    mRotateAnimation->setStartValue(0.0);
    mRotateAnimation->setEndValue(1.0);
    connect(mRotateAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::rotateAnimationStep);
    connect(mRotateAnimation, &QVariantAnimation::finished, this, &OmViewpoint::resetAnimations);
    mRotateAnimation->start();
    skipped = false;
  } else {
    mOrientation->setValue(OmRotation(mFinalOrientationQuaternion));
    emit refreshRequired();
  }

  if (mFinalOrbitTargetPostion) {
    const OmVector3 differenceVector = *mFinalOrbitTargetPostion - mPosition->value();
    const double distance = differenceVector.length();
    // don't animate if the target position is very close to avoid numerical errors
    if (distance > 0.00001) {
      mInitialMoveToPosition = mPosition->value();
      mMoveToDirection = differenceVector / distance;
      mTranslateAnimation = new QVariantAnimation(this);
      mTranslateAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
      mTranslateAnimation->setDuration(ANIMATION_DURATION);
      mTranslateAnimation->setStartValue(0.0);
      mTranslateAnimation->setEndValue(distance);
      connect(mTranslateAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::translateAnimationStep);
      connect(mTranslateAnimation, &QVariantAnimation::finished, this, &OmViewpoint::resetAnimations);
      mTranslateAnimation->start();
      skipped = false;
    } else {
      mPosition->setValue(*mFinalOrbitTargetPostion);
      emit refreshRequired();
    }
    delete mFinalOrbitTargetPostion;
    mFinalOrbitTargetPostion = NULL;
  }

  if (skipped)
    resetAnimations();
}

void OmViewpoint::moveTo(const OmVector3 &targetPosition, const OmRotation &targetRotation) {
  resetAnimations();
  lock();
  const OmVector3 differenceVector = targetPosition - mPosition->value();
  const double distance = differenceVector.length();
  // don't animate if the target position is very close to avoid numerical errors
  if (distance > 0.00001) {
    mInitialMoveToPosition = mPosition->value();
    mMoveToDirection = differenceVector / distance;
    mTranslateAnimation = new QVariantAnimation(this);
    mTranslateAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
    mTranslateAnimation->setDuration(ANIMATION_DURATION);
    mTranslateAnimation->setStartValue(0.0);
    mTranslateAnimation->setEndValue(distance);
    connect(mTranslateAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::translateAnimationStep);
    connect(mTranslateAnimation, &QVariantAnimation::finished, this, &OmViewpoint::resetAnimations);
    mTranslateAnimation->start();
  } else {
    unlock();
    mPosition->setValue(targetPosition);
    emit refreshRequired();
  }

  if (mOrientation->value().direction().dot(targetRotation.direction()) < 0.99994) {
    // build the start and end quaternions for camera orientation
    mInitialOrientationQuaternion = OmQuaternion(mOrientation->value().axis(), mOrientation->value().angle());
    mFinalOrientationQuaternion = OmQuaternion(targetRotation.axis(), targetRotation.angle());
    mFinalOrientationQuaternion.normalize();
    mRotateAnimation = new QVariantAnimation(this);
    mRotateAnimation->setEasingCurve(QEasingCurve(QEasingCurve::InOutCubic));
    mRotateAnimation->setDuration(ANIMATION_DURATION);
    mRotateAnimation->setStartValue(0.0);
    mRotateAnimation->setEndValue(1.0);
    connect(mRotateAnimation, &QVariantAnimation::valueChanged, this, &OmViewpoint::rotateAnimationStep);
    connect(mRotateAnimation, &QVariantAnimation::finished, this, &OmViewpoint::resetAnimations);
    mRotateAnimation->start();
  } else {
    unlock();
    mOrientation->setValue(targetRotation);
    emit refreshRequired();
  }
}

void OmViewpoint::resetAnimations() {
  delete mTranslateAnimation;
  mTranslateAnimation = NULL;

  delete mRotateAnimation;
  mRotateAnimation = NULL;

  delete mOrbitAnimation;
  mOrbitAnimation = NULL;

  mOrbitRadius = 0.0;
  unlock();
  unlockRotationCenter();
}

void OmViewpoint::translateAnimationStep(const QVariant &value) {
  mPosition->setValue(mInitialMoveToPosition + value.toDouble() * mMoveToDirection);
  emit refreshRequired();
}

void OmViewpoint::rotateAnimationStep(const QVariant &value) {
  OmQuaternion slerpedQuaternion(
    OmQuaternion::slerp(mInitialOrientationQuaternion, mFinalOrientationQuaternion, value.toDouble()));
  slerpedQuaternion.normalize();
  // deal with numerical errors when slerping to identity quaternion
  if (OmRotation(slerpedQuaternion).direction().isNan())
    mOrientation->setValue(OmRotation(mFinalOrientationQuaternion));
  else
    mOrientation->setValue(OmRotation(slerpedQuaternion));
  emit refreshRequired();
}

void OmViewpoint::translateOrbitAnimationStep(const QVariant &value) {
  OmQuaternion slerpedQuaternion(OmQuaternion::slerp(mInitialOrbitQuaternion, mFinalOrbitQuaternion, value.toDouble()));
  mPosition->setValue(mRotationCenter + (mCenterToViewpointUnitVector * OmMatrix3(slerpedQuaternion)) * mOrbitRadius);
}

void OmViewpoint::rotateOrbitAnimationStep(const QVariant &value) {
  OmRotation normalisedRotation = mOrientation->value();
  normalisedRotation.normalizeAxis();
  mOrientation->setValue(normalisedRotation);
  lookAt(mRotationCenter, mOrientation->value().up());
  emit refreshRequired();
}

void OmViewpoint::lookAtAnimationStep(const QVariant &value) {
  OmQuaternion slerpedQuaternion(OmQuaternion::slerp(mLookAtInitialQuaternion, mLookAtFinalQuaternion, value.toDouble()));
  slerpedQuaternion.normalize();
  // deal with numerical errors when slerping to identity quaternion
  if (OmRotation(slerpedQuaternion).direction().isNan())
    mOrientation->setValue(OmRotation(mLookAtFinalQuaternion));
  else
    mOrientation->setValue(OmRotation(slerpedQuaternion));
  emit refreshRequired();
}
