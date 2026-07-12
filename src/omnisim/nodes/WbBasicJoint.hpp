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

// Abstract node class representing the most general mechanical joint

#ifndef WB_BASIC_JOINT_HPP
#define WB_BASIC_JOINT_HPP

#include "WbBaseNode.hpp"
#include "WbOdeTypes.hpp"
#include "WbRotation.hpp"
#include "WbVector3.hpp"

class WbBoundingSphere;
class WbSlot;
class WbSolid;
class WbSolidReference;

struct WrTransform;
struct WrRenderable;
struct WrStaticMesh;
struct WrMaterial;

class WbBasicJoint : public WbBaseNode {
  Q_OBJECT

public:
  virtual ~WbBasicJoint() override;

  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  virtual void prePhysicsStep(double ms) {}
  virtual void postPhysicsStep() {}
  virtual bool setJoint();
  void write(WbWriter &writer) const override;
  virtual bool resetJointPositions();
  void setMatrixNeedUpdate() override;
  virtual void updateOdeWorldCoordinates() {}
  virtual void computeEndPointSolidPositionFromParameters(WbVector3 &translation, WbRotation &rotation) const = 0;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  void updateSegmentationColor(const WbRgb &color) override;

  void setSolidEndPoint(WbSolid *solid);
  void setSolidEndPoint(WbSolidReference *solid);
  void setSolidEndPoint(WbSlot *slot);

  WbSolid *solidEndPoint() const;
  WbSolidReference *solidReference() const;
  WbSolid *solidParent() const;
  virtual dJointID jointID() const { return mJoint; }
  // endPoint Solid translation and rotation if joint position is 0
  const WbVector3 &zeroEndPointTranslation() const { return mEndPointZeroTranslation; }
  const WbRotation &zeroEndPointRotation() const { return mEndPointZeroRotation; }
  bool isEnabled() const;

  // ray tracing
  WbBoundingSphere *boundingSphere() const override;

  void updateAfterParentPhysicsChanged();
  virtual void updateEndPointZeroTranslationAndRotation() = 0;

  QList<const WbBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;
  QString endPointName() const override;

  // P3.7 of cuda-newton-physics-plan.md: queue this joint for Newton-side
  // registration if both endpoints opt into the Newton backend. The actual
  // registry handshake is deferred to flushPendingNewtonRegistrations(),
  // which runs from WbSimulationWorld::step right before the Newton model
  // is finalised. Deferral is necessary because joint postFinalize fires
  // *during* the parent Solid's children traversal, before the parent's
  // own Newton body index has been allocated.
  static void flushPendingNewtonRegistrations();

  // P3.8.b: per-tick drive for any registered Newton hinge with a motor.
  // Reads each motor's targetVelocity() and pushes it through
  // backend->setJointTargetVelocity. Called from WbSimulationWorld::step
  // before newton->step. No-op if no motorized hinges are Newton-backed.
  static void pushNewtonMotorTargets();

  // P3.7 status accessor: 0/-1 from addJointRevolute, or -1 if not a
  // hinge or one of the endpoints isn't Newton-backed.
  int newtonJointIndex() const { return mNewtonJointIndex; }

public slots:
  void updateEndPoint();

signals:
  void endPointChanged(WbBaseNode *node);

protected:
  WbBasicJoint(const QString &modelName, WbTokenizer *tokenizer = NULL);
  WbBasicJoint(const WbBasicJoint &other);
  WbBasicJoint(const WbNode &other);

  virtual void setOdeJoint(dBodyID body, dBodyID parentBody);
  // anchor point on an hinge axis; also defined for a slider axis to set its graphical represention position
  virtual WbVector3 anchor() const;

  dJointID mJoint;
  // joint attached to the static environment (internally in ODE the first body cannot be NULL)
  bool mIsReverseJoint;
  // the second end of the joint, the first being its parent; this is is either a Solid or a SolidReference
  WbSFNode *mEndPoint;
  // axis, anchor, initial position, damping and spring constants, min and max stop
  WbSFNode *mParameters;

  // variables and methods about the endPoint Solid translation and rotation when joint position is 0
  WbVector3 mEndPointZeroTranslation;
  WbRotation mEndPointZeroRotation;
  void retrieveEndPointSolidTranslationAndRotation(WbVector3 &it, WbRotation &ir) const;
  dJointID mSpringAndDamperMotor;  // ODE linear motor used to simulate spring and damper effects by means of stops
  virtual void applyToOdeSpringAndDampingConstants(dBodyID body, dBodyID parentBody) = 0;

  bool mIsEndPointPositionChangedByJoint;

  // P3.7: Newton joint index in the active Model, or -1 when the joint
  // isn't (yet) registered with WbNewtonBackend (default-ODE world,
  // mismatched endpoints, non-hinge joint type, etc.).
  int mNewtonJointIndex;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrStaticMesh *mMesh;
  WrMaterial *mMaterial;

  const bool isJoint() const override { return true; }

protected slots:
  virtual void updateParameters() = 0;
  void updateSpringAndDampingConstants();
  virtual void updateOptionalRendering(int option);

private:
  WbBasicJoint &operator=(const WbBasicJoint &);  // non copyable
  void init();

private slots:
  virtual void updateJointAxisRepresentation() = 0;  // updates the WREN joint axis when the line scale changes
  void updateEndPointPosition();
  void updateBoundingSphere(WbBaseNode *subNode);
};
#endif
