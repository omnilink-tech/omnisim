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

// Abstract node class representing a logical joint device

#ifndef OM_JOINT_DEVICE_HPP
#define OM_JOINT_DEVICE_HPP

#include "OmLogicalDevice.hpp"

class OmJoint;
class OmPropeller;
class OmRobot;
class OmTrack;

class OmJointDevice : public OmLogicalDevice {
  Q_OBJECT

public:
  virtual ~OmJointDevice() override;

  // inherited from OmBaseNode
  void preFinalize() override;

  OmJoint *joint() const;          // joint attached to the device
  OmPropeller *propeller() const;  // propeller attached to the device
  OmTrack *track() const;          // track attached to the device
  OmRobot *robot() const;          // robot that owns the joint
  double position() const;
  virtual int type() const;
  int positionIndex() const { return mPositionIndex; }

  OmLogicalDevice *getSiblingDeviceByType(int nodeType) const;

protected:
  explicit OmJointDevice(const QString &modelName, OmTokenizer *tokenizer = NULL);
  explicit OmJointDevice(OmTokenizer *tokenizer = NULL);
  OmJointDevice(const OmJointDevice &other);
  explicit OmJointDevice(const OmNode &other);

private:
  OmJointDevice &operator=(const OmJointDevice &);  // non copyable
  void init();
  mutable bool mRobotFound;
  mutable OmRobot *mRobot;
  int mPositionIndex;
};

#endif
