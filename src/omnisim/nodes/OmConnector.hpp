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

#ifndef OM_CONNECTOR_HPP
#define OM_CONNECTOR_HPP

#include "OmSolidDevice.hpp"

class OmNewtonBackend;
class OmSensor;
class OmVector3;

class OmConnector : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmConnector(OmTokenizer *tokenizer = NULL);
  OmConnector(const OmConnector &other);
  explicit OmConnector(const OmNode &other);
  virtual ~OmConnector() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CONNECTOR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(OmDataStream &stream) override;
  void writeConfigure(OmDataStream &) override;
  void prePhysicsStep(double ms) override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  enum FaceType { UNKNOWN, SYMMETRIC, ACTIVE, PASSIVE };
  FaceType faceType() const { return mFaceType; }

  // interface with auto-assembly mechanism
  static bool isAllowingMouseMotion();
  static void solidHasMoved(OmSolid *solid);

  // Newton-native weld (_scratch/design_weld_touch.md W2, gated by
  // OMNISIM_NEWTON_WELDS, default OFF): called by the weld-slot sweep in
  // OmSolid::flushPendingNewtonRegistrations while the Newton world is open
  // for build. Reserves one inactive MuJoCo equality-weld slot anchored on
  // this connector's merge-leader body; idempotent, no-op when the gate is
  // off or the leader owns no Newton body. Allocated for PASSIVE connectors
  // too: the connection's single weld can live on either side (a bodiless
  // active side welds the passive side's body to the world), exactly like
  // ODE's one-joint-per-connection ownership.
  void ensureNewtonWeldSlot(OmNewtonBackend *newton);
  // W1.7 mid-run physics rebuild: forget the slot (it indexed the OLD Newton
  // world). Returns true when a weld was ENGAGED and is therefore dropped --
  // the caller warns loudly; re-engagement across a rebuild is future work.
  bool resetNewtonWeldSlotForRebuild() {
    const bool wasEngaged = mNewtonWeldActive;
    mNewtonWeldSlot = -1;
    mNewtonWeldActive = false;
    return wasEngaged;
  }

private:
  // fields
  OmSFString *mType;               // connection type "symmetric", "active" or "passive"
  OmSFBool *mIsLocked;             // current locked state
  OmSFBool *mAutoLock;             // locks automatically in case of presence (but only when "isLocked")
  OmSFBool *mUnilateralLock;       // for "symmetric" type only: enable unilateral locking
  OmSFBool *mUnilateralUnlock;     // for "symmetric" type only: enable unilateral unlocking
  OmSFDouble *mDistanceTolerance;  // acceptable error in the distance between both Connectors
  OmSFDouble *mAxisTolerance;      // acceptable error angle between the two normals vectors
  OmSFDouble *mRotationTolerance;  // acceptable error in rotation angle
  OmSFInt *mNumberOfRotations;     // number of possible docking rotations
  OmSFBool *mSnap;                 // should automatically snap with peer connector when locked
  OmSFDouble *mTensileStrength;    // max pull force that the connector can withstand without breaking (Newtons)
  OmSFDouble *mShearStrength;      // max shear force that the connector can withstand without breaking (Newtons)

  // other stuff
  FaceType mFaceType;    // UNKNOWN, SYMMETRIC, ACTIVE or PASSIVE
  double mMinDist2;      // squared distanceTolerance
  OmConnector *mPeer;    // peer connector or NULL
  bool mStartup;         // do once flag
  OmSensor *mSensor;     // presence sensor
  int mValue;
  bool mIsJointInversed;
  QMap<QString, bool> mIsInitiallyLocked;
  bool mNeedToReconfigure;

  // Newton-native weld state (OMNISIM_NEWTON_WELDS). mNewtonWeldSlot is the
  // backend slot id reserved by ensureNewtonWeldSlot (-1 = none);
  // mNewtonWeldActive marks the SIDE THAT OWNS the engaged weld, mirroring
  // the ODE convention that only one of the two connectors holds mFixedJoint.
  // mIsJointInversed is shared with the ODE path: true when this side is
  // bodiless and the weld's obj1 is the peer's body.
  int mNewtonWeldSlot;
  bool mNewtonWeldActive;

  OmConnector &operator=(const OmConnector &);  // non copyable
  OmNode *clone() const override { return new OmConnector(*this); }
  void addConfigure(OmDataStream &);

  bool isReadyToAttachTo(const OmConnector *other) const;
  void attachTo(OmConnector *other);
  void detachFromPeer();
  void createFixedJoint(OmConnector *other);
  void createNewtonWeld(OmConnector *other, int newtonBody1, int newtonBody2);
  void releaseNewtonWeld();
  void lock();
  void unlock();
  void computeValue();
  OmConnector *detectPresence() const;
  bool isCompatibleWith(const OmConnector *other) const;
  double getDistance2(const OmConnector *other) const;
  bool isAlignedWith(const OmConnector *other) const;
  bool isXAlignedWith(const OmConnector *other) const;
  bool isZAlignedWith(const OmConnector *other) const;
  void detachIfForceExceedStrength();
  double findClosestRotationalAlignment(double alpha) const;
  double getEffectiveTensileStrength() const;
  double getEffectiveShearStrength() const;
  void init();
  void hasMoved();
  void assembleWith(OmConnector *other);
  void assembleAxes(OmConnector *other);

private slots:
  void updateType();
  void updateIsLocked();
  void updateNumberOfRotations();
  void updateDistanceTolerance();
  void updateAxisTolerance();
  void updateRotationTolerance();
  void updateTensileStrength();
  void updateShearStrength();
};

#endif
