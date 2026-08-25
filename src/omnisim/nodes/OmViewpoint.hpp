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

#ifndef OM_VIEWPOINT_HPP
#define OM_VIEWPOINT_HPP

#include "OmBackground.hpp"
#include "OmBaseNode.hpp"
#include "OmMatrix3.hpp"
#include "OmQuaternion.hpp"
#include "OmRotation.hpp"
#include "OmSFBool.hpp"
#include "OmSFString.hpp"


class OmAbstractPose;
class OmLensFlare;
class OmRay;
class OmRenderBackend;
class OmSolid;
class OmVector2;

class QVariantAnimation;

class OmViewpoint : public OmBaseNode {
  Q_OBJECT

public:
  enum { FOLLOW_NONE, FOLLOW_TRACKING, FOLLOW_MOUNTED, FOLLOW_PAN_AND_TILT };

  // constructors and destructor
  explicit OmViewpoint(OmTokenizer *tokenizer = NULL);
  OmViewpoint(const OmViewpoint &other);
  explicit OmViewpoint(const OmNode &other);
  virtual ~OmViewpoint() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_VIEWPOINT; }
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;

  static QString followTypeToString(int type);
  static int followStringToType(const QString &type);

  // getters

  OmSFVector3 *position() const { return mPosition; }
  OmSFRotation *orientation() const { return mOrientation; }
  OmSFString *follow() const { return mFollow; }
  double aspectRatio() const { return mAspectRatio; }
  double orthographicViewHeight() const { return mOrthographicViewHeight; }
  bool isLocked() const { return mIsLocked; }
  bool rotationCenterIsLocked() const { return mRotationCenterIsLocked; }
  const OmVector3 &rotationCenter() { return mRotationCenter; }
  OmSolid *followedSolid() const { return mFollowedSolid; }
  int followType() const { return followStringToType(mFollowType->value()); }
  bool isFollowed(const OmSolid *solid) const;
  OmSFDouble *followSmoothness() const { return mFollowSmoothness; }
  OmSFDouble *fieldOfView() const { return mFieldOfView; }
  int projectionMode() const { return mProjectionMode; }
  float viewDistanceUnscaling(const OmVector3 &position) const;
  OmSFDouble *exposure() const { return mExposure; }
  // Additive getters for the wgpu main view's post-passes (mirror fieldOfView()/exposure()).
  OmSFDouble *bloomThresholdField() const { return mBloomThreshold; }
  OmSFDouble *ambientOcclusionRadiusField() const { return mAmbientOcclusionRadius; }
  OmSFDouble *nearField() const { return mNear; }
  // renderBackend: resolves this Viewpoint's renderer through OmRenderBackendRegistry.
  // F1 (wren-deletion-runbook.md): always the wgpu backend when wgpu-native is available --
  // "wren" is retired, warns once per node, and resolves to wgpu; the WREN backend is
  // returned only as the wgpu-native-unavailable last resort (deleted at D1.4). The string
  // accessor returns the raw field value for diagnostic logging.
  const QString &renderBackendName() const;
  OmRenderBackend *renderBackend() const;

  // setters
  void decOrthographicViewHeight();
  void incOrthographicViewHeight();
  void setOrthographicViewHeight(double ovh);
  void lock() { mIsLocked = true; }
  void unlock() { mIsLocked = false; }
  void lockRotationCenter() { mRotationCenterIsLocked = true; }
  void unlockRotationCenter() { mRotationCenterIsLocked = false; }
  void startFollowUp(OmSolid *solid, bool updateField);
  void startFollowUpFromField();
  void setFollowType(int followType);
  void storePickedCoordinates(const OmVector3 &v) { mRotationCenter = v; }
  void setCoordinateSystemVisibility(bool visible);
  void restore();
  void save(const QString &id) override;
  void setPosition(const OmVector3 &position);
  void setProjectionMode(int projectionMode) {
    if (projectionMode != mProjectionMode) {
      mProjectionMode = projectionMode;
      emit cameraModeChanged();
    }
  }
  void lookAt(const OmVector3 &target, const OmVector3 &upVector);

  // fixed views
  void southView();
  void northView();
  void westView();
  void eastView();
  void topView();
  void bottomView();
  void objectFrontView();
  void objectBackView();
  void objectLeftView();
  void objectRightView();
  void objectTopView();
  void objectBottomView();

  // public cleanup
  void terminateFollowUp();

  // public updates
  void updateAspectRatio(double renderWindowAspectRatio);
  void updateFollowUp();
  void updateFollowSolidState();
  void updateOrthographicViewHeight();

  void setNodesVisibility(QList<const OmBaseNode *> nodes, bool visible);
  const QList<const OmBaseNode *> &getInvisibleNodes() const { return mInvisibleNodes; }
  void enableNodeVisibility(bool enabled);

  // Ray picking based on current projection mode
  void viewpointRay(int x, int y, OmRay &ray) const;
  OmVector3 pick(int x, int y, double z0) const;
  void toPixels(const OmVector3 &pos, OmVector2 &P) const;
  void toPixels(const OmVector3 &pos1, OmVector2 &P1, const OmVector3 &pos2, OmVector2 &P2) const;
  void toWorld(const OmVector3 &pos, OmVector3 &P) const;
  void eyeToPixels(const OmVector3 &eyePosition, OmVector2 &P) const;
  double zEye(const OmVector3 &pos) const;

  bool moveViewpointToObject(OmBaseNode *node);  // return true if node was valid

  OmLensFlare *lensFlare() const;
  // True when this Viewpoint can actually reach wr_scene_render through WREN, i.e. when the WREN

public slots:

private:
  // user accessible fields
  OmSFDouble *mFieldOfView;
  OmSFRotation *mOrientation;
  OmSFVector3 *mPosition;
  OmSFString *mDescription;
  OmSFDouble *mNear;
  OmSFDouble *mFar;
  OmSFDouble *mExposure;
  OmSFString *mFollow;
  OmSFString *mFollowType;
  OmSFDouble *mFollowSmoothness;
  OmSFNode *mLensFlare;
  OmSFDouble *mAmbientOcclusionRadius;
  OmSFDouble *mBloomThreshold;
  OmSFString *mRenderBackend;
  // F1 (WREN unselectable): once-per-node latch for the retired-"wren" warning emitted by
  // renderBackend(). Mutable because the accessor is const.
  mutable bool mWarnedRetiredWrenBackend = false;

  // post-processing effects

  // to restore viewpoint
  QMap<QString, double> mSavedFieldOfView;
  QMap<QString, OmVector3> mSavedPosition;
  QMap<QString, OmRotation> mSavedOrientation;
  QMap<QString, QString> mSavedDescription;
  QMap<QString, double> mSavedNear;
  QMap<QString, double> mSavedFar;
  QMap<QString, double> mSavedOrthographicHeight;
  QMap<QString, QString> mSavedFollow;

  // follow solid stuff
  OmSolid *mFollowedSolid;
  OmVector3 mFollowedSolidPreviousPosition;
  OmMatrix3 mFollowedSolidReferenceRotation;
  OmRotation mViewPointReferenceRotation;
  OmVector3 mReferenceOffset;
  OmVector3 mEquilibriumVector;
  OmVector3 mVelocity;
  bool mNeedToUpdateFollowSolidState;

  // other variables
  int mProjectionMode;

  QList<const OmBaseNode *> mInvisibleNodes;
  bool mNodeVisibilityEnabled;

  double mAspectRatio;
  double mFieldOfViewY;
  double mTanHalfFieldOfViewY;
  double mOrthographicViewHeight;
  bool mFromOrthographic;

  bool mIsLocked;
  OmVector3 mRotationCenter;
  bool mRotationCenterIsLocked;
  bool mFollowChangedBySelection;
  bool mFollowChangedBySolidName;
  bool mFollowEmptiedByDestroyedSolid;

  // viewpoint translation animation
  OmVector3 mMoveToDirection;
  OmVector3 mInitialMoveToPosition;

  // viewpoint look-at animation
  OmQuaternion mLookAtInitialQuaternion;
  OmQuaternion mLookAtFinalQuaternion;

  // viewpoint orbit animation
  OmVector3 mCenterToViewpointUnitVector;
  OmVector3 mOrbitTargetUnitVector;
  OmVector3 *mFinalOrbitTargetPostion;
  OmQuaternion mInitialOrientationQuaternion;
  OmQuaternion mFinalOrientationQuaternion;
  OmQuaternion mInitialOrbitQuaternion;
  OmQuaternion mFinalOrbitQuaternion;
  double mOrbitRadius;

  // used to ensure view animations correspond to the world coordinate system
  OmQuaternion mSpaceQuaternion;

  // Qt animations
  QVariantAnimation *mTranslateAnimation;
  QVariantAnimation *mRotateAnimation;
  QVariantAnimation *mOrbitAnimation;

  // static non-const variables
  static int cViewpointCameraNumber;
  static int cCameraCounter;
  static int cZorder;

  // static const variables
  static const double INCREASE_FACTOR;
  static const double DECREASE_FACTOR;
  static const double X_OFFSET;
  static const double X_REL_ORTHOGRAPHIC;
  static const double Z_THRESHOLD;

  OmViewpoint &operator=(const OmViewpoint &);  // non copyable
  OmNode *clone() const override { return new OmViewpoint(*this); }

  // private initialization methods
  void init();

  // private cleanup methods
  void deleteWrenObjects();

  // private apply methods
  // non-slot private updates
  void updateFieldOfViewY();


  void recomputeFollowField();
  void createCameraListenerIfNeeded();

  // can be used for any generic animated viewpoint movement
  void moveTo(const OmVector3 &targetPosition, const OmRotation &targetRotation);
  void orbitTo(const OmVector3 &targetUnitVector, const OmRotation &targetRotation,
               const OmAbstractPose *selectedObject = NULL);

  static OmAbstractPose *computeSelectedObjectPose();
  static OmRotation computeObjectViewRotation(const OmRotation &rotation, const OmAbstractPose *selectedObject);

private slots:
  void updateFieldOfView();
  void updateOrientation();
  void updatePosition();
  void updateNear();
  void updateFar();
  void updateExposure();
  void updateFollow();
  void updateFollowType();
  void updateAmbientOcclusionRadius();
  void updateBloomThreshold();
  // cleanup
  void emptyFollow();
  // synchronize
  void synchronizeFollowWithSolidName();

  // used for each step of animated viewpoint movement
  void translateAnimationStep(const QVariant &value);
  void rotateAnimationStep(const QVariant &value);
  void translateOrbitAnimationStep(const QVariant &value);
  void rotateOrbitAnimationStep(const QVariant &value);
  void lookAtAnimationStep(const QVariant &value);
  void animateLookAtIfNeeded();
  void firstOrbitStep();
  void secondOrbitStep();

  // called when an animation completes successfully, deletes animations and
  // resets default view radius
  void resetAnimations();

signals:
  void followInvalidated(bool valid);
  void followTypeChanged(int type);
  void cameraParametersChanged();
  void refreshRequired();
  void nodeVisibilityChanged(const OmNode *node, bool visibility);
  void cameraModeChanged();
};

#endif
