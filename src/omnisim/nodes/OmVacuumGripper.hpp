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

#ifndef OM_VACUUM_GRIPPER_HPP
#define OM_VACUUM_GRIPPER_HPP

#include <OmSFInt.hpp>
#include <OmSolidDevice.hpp>

class OmNewtonBackend;
class OmSensor;

class OmVacuumGripper : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmVacuumGripper(OmTokenizer *tokenizer = NULL);
  OmVacuumGripper(const OmVacuumGripper &other);
  explicit OmVacuumGripper(const OmNode &other);
  virtual ~OmVacuumGripper() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_VACUUM_GRIPPER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &stream) override;
  void writeAnswer(OmDataStream &stream) override;
  void writeConfigure(OmDataStream &) override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  bool isWaitingForConnection();
  int contactPoints() const { return mContactPoints->value(); }
  void addCollidedSolid(OmSolid *solid, const double depth);

  // Newton-native weld (_scratch/design_weld_touch.md W3, gated by
  // OMNISIM_NEWTON_WELDS, default OFF): called by the weld-slot sweep in
  // OmSolid::flushPendingNewtonRegistrations while the Newton world is open
  // for build. Reserves one inactive MuJoCo equality-weld slot anchored on
  // this gripper's merge-leader body; idempotent, no-op when the gate is off
  // or the leader owns no Newton body.
  void ensureNewtonWeldSlot(OmNewtonBackend *newton);
  // W1.7 mid-run physics rebuild: forget the slot (old-world index). Returns
  // true when the grip was ENGAGED and is therefore dropped (warned loudly).
  bool resetNewtonWeldSlotForRebuild() {
    const bool wasEngaged = mNewtonWeldActive;
    mNewtonWeldSlot = -1;
    mNewtonWeldActive = false;
    return wasEngaged;
  }

private:
  // fields
  OmSFBool *mIsOn;               // current state
  OmSFDouble *mTensileStrength;  // max pull force that the connector can withstand without breaking (Newtons)
  OmSFDouble *mShearStrength;    // max shear force that the connector can withstand without breaking (Newtons)
  OmSFInt *mContactPoints;       // minimum number of contact points required to connect with a Solid

  // other stuff
  OmSolid *mSolid;       // connected object or NULL
  OmSensor *mSensor;     // presence sensor
  bool mValue;
  bool mNeedToReconfigure;
  QMap<QString, bool> mIsInitiallyOn;
  QList<std::pair<OmSolid *, const double>> mCollidedSolidList;  // list of Solid that collided with the deepest contact depth

  // Newton-native weld state (OMNISIM_NEWTON_WELDS): the backend slot id
  // reserved by ensureNewtonWeldSlot (-1 = none) and whether it is currently
  // engaged. mIsJointInversed analogue is unnecessary: the gripper side
  // always owns the slot and its body is obj1 (a bodiless gripper gets no
  // slot at all).
  int mNewtonWeldSlot;
  bool mNewtonWeldActive;

  OmVacuumGripper &operator=(const OmVacuumGripper &);  // non copyable
  OmNode *clone() const override { return new OmVacuumGripper(*this); }
  void addConfigure(OmDataStream &);

  void attachToSolid();
  void detachFromSolid();
  void createFixedJoint(OmSolid *other);
  void destroyFixedJoint();
  OmSolid *newtonDeepestCollidedSolid() const;
  void turnOn();
  void turnOff();
  void computeValue();
  void detachIfForceExceedStrength();
  double getEffectiveTensileStrength() const;
  double getEffectiveShearStrength() const;
  void init();

private slots:
  void updateIsOn();
  void updateTensileStrength();
  void updateShearStrength();
  void updateContactPoints();
};

#endif
