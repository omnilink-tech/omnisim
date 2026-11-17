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

#ifndef OM_PROPELLER_HPP
#define OM_PROPELLER_HPP

#include "OmBaseNode.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"

class OmLogicalDevice;
class OmSFNode;
class OmSolid;
class OmRotationalMotor;

class OmPropeller : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPropeller(OmTokenizer *tokenizer = NULL);
  OmPropeller(const OmPropeller &other);
  explicit OmPropeller(const OmNode &other);
  virtual ~OmPropeller() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PROPELLER; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  void propagateSelection(bool selected) override;
  void setMatrixNeedUpdate() override;
  void write(OmWriter &writer) const override;
  void reset(const QString &id) override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  void prePhysicsStep(double ms);

  // field accessors
  const OmVector3 &axis() const { return mShaftAxis->value(); }  // thrust direction
  // point where the thrust applies (expressed in relative coordinates)
  const OmVector3 &centerOfThrust() const { return mCenterOfThrust->value(); }
  const OmVector2 &thrustConstants() const { return mThrustConstants->value(); }  // constants used to compute thrust output
  const OmVector2 &torqueConstants() const { return mTorqueConstants->value(); }  // constants used to compute torque output
  OmRotationalMotor *motor() const;
  enum HelixType { FAST_HELIX, SLOW_HELIX };
  OmSolid *helix(HelixType type) const;
  OmSolid *helix() const { return mHelix; }  // current helix
  OmLogicalDevice *device() const;
  double position() const { return mPosition; }

  double currentThrust() const { return mCurrentThrust; }
  double currentTorque() const { return mCurrentTorque; }

private:
  // Scene Tree fields
  OmSFVector3 *mShaftAxis;
  OmSFVector3 *mCenterOfThrust;
  OmSFVector2 *mThrustConstants;
  OmSFVector2 *mTorqueConstants;
  OmSFDouble *mFastHelixThreshold;
  OmSFNode *mDevice;
  OmSFNode *mFastHelix;
  OmSFNode *mSlowHelix;
  double mPosition;
  OmSolid *mHelix;
  HelixType mHelixType;

  OmVector3 mNormalizedAxis;

  double mCurrentThrust;
  double mCurrentTorque;

  OmPropeller &operator=(const OmPropeller &);  // non copyable
  OmNode *clone() const override { return new OmPropeller(*this); }
  void init();
  void updateHelix(double angularSpeed, bool ode = true);

private slots:
  // Update methods
  void updateShaftAxis();
  void updateDevice();
  void updateHelixRepresentation();
};
#endif
