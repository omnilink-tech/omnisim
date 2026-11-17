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

#include "OmJointDevice.hpp"

#include "OmBallJoint.hpp"
#include "OmField.hpp"
#include "OmHinge2Joint.hpp"
#include "OmJoint.hpp"
#include "OmPropeller.hpp"
#include "OmSolid.hpp"
#include "OmTrack.hpp"

#include <cassert>

OmJointDevice::OmJointDevice(const QString &modelName, OmTokenizer *tokenizer) : OmLogicalDevice(modelName, tokenizer) {
  init();
}

OmJointDevice::OmJointDevice(OmTokenizer *tokenizer) : OmLogicalDevice("PositionSensor", tokenizer) {
  init();
}

OmJointDevice::OmJointDevice(const OmJointDevice &other) : OmLogicalDevice(other) {
  init();
}

OmJointDevice::OmJointDevice(const OmNode &other) : OmLogicalDevice(other) {
  init();
}

OmJointDevice::~OmJointDevice() {
}

void OmJointDevice::init() {
  mRobotFound = false;
  mPositionIndex = 1;
  mRobot = NULL;
}

void OmJointDevice::preFinalize() {
  OmBaseNode::preFinalize();
  // Cache position index
  const OmField *const f = parentField(true);
  assert(f);
  mPositionIndex = 1;
  if (f->name().endsWith("2"))
    mPositionIndex = 2;
  else if (f->name().endsWith("3"))
    mPositionIndex = 3;
}

OmJoint *OmJointDevice::joint() const {
  return dynamic_cast<OmJoint *>(parentNode());
}

OmPropeller *OmJointDevice::propeller() const {
  return dynamic_cast<OmPropeller *>(parentNode());
}

OmTrack *OmJointDevice::track() const {
  return dynamic_cast<OmTrack *>(parentNode());
}

OmLogicalDevice *OmJointDevice::getSiblingDeviceByType(int nodeType) const {
  OmJoint *j = joint();
  if (j) {
    const OmBallJoint *ballJoint = dynamic_cast<OmBallJoint *>(j);
    if (ballJoint) {
      // special case for nodes in devices3 field
      bool isDevice3 = false;
      for (int i = 0; i < ballJoint->devices3Number(); ++i) {
        if (this == ballJoint->device3(i)) {
          isDevice3 = true;
          break;
        }
      }
      if (isDevice3) {
        for (int i = 0; i < ballJoint->devices3Number(); ++i) {
          OmLogicalDevice *device3 = ballJoint->device3(i);
          if (device3 && device3->nodeType() == nodeType)
            return device3;
        }
        return NULL;
      }
    }
    const OmHinge2Joint *hinge2 = dynamic_cast<OmHinge2Joint *>(j);
    if (hinge2) {
      // special case for nodes in devices2 field
      bool isDevice2 = false;
      for (int i = 0; i < hinge2->devices2Number(); ++i) {
        if (this == hinge2->device2(i)) {
          isDevice2 = true;
          break;
        }
      }
      if (isDevice2) {
        for (int i = 0; i < hinge2->devices2Number(); ++i) {
          OmLogicalDevice *device2 = hinge2->device2(i);
          if (device2 && device2->nodeType() == nodeType)
            return device2;
        }
        return NULL;
      }
    }

    for (int i = 0; i < j->devicesNumber(); ++i) {
      OmLogicalDevice *device = j->device(i);
      if (device && device->nodeType() == nodeType)
        return device;
    }
    return NULL;
  }

  const OmTrack *t = track();
  if (t) {
    foreach (OmLogicalDevice *device, t->devices()) {
      if (device && device->nodeType() == nodeType)
        return device;
    }
  }

  return NULL;
}

int OmJointDevice::type() const {
  const OmJoint *const j = joint();
  if (j)
    return j->nodeType() == WB_NODE_SLIDER_JOINT ? WB_LINEAR : WB_ROTATIONAL;

  if (track())
    return WB_LINEAR;

  // defaults to ROTATIONAL for OmPropeller
  return WB_ROTATIONAL;
}

OmRobot *OmJointDevice::robot() const {
  if (mRobotFound == true)
    return mRobot;

  const OmSolid *const us = upperSolid();
  if (us)
    mRobot = us->robot();

  if (mRobot)
    mRobotFound = true;

  return mRobot;
}

double OmJointDevice::position() const {
  // return exact position
  const OmJoint *const j = joint();
  if (j)
    return j->position(mPositionIndex);

  const OmTrack *const t = track();
  if (t)
    return t->position();

  assert(propeller());
  return propeller()->position();
}
