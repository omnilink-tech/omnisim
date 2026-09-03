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

#ifndef OM_TOUCH_SENSOR_HPP
#define OM_TOUCH_SENSOR_HPP

#include "OmSolidDevice.hpp"

class OmSensor;
class OmLookupTable;
struct dJointFeedback;

class OmTouchSensor : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmTouchSensor(OmTokenizer *tokenizer = NULL);
  OmTouchSensor(const OmTouchSensor &other);
  explicit OmTouchSensor(const OmNode &other);
  virtual ~OmTouchSensor() override;

  void setTouching(bool touching) { mIsTouching = touching; }
  void setGuiTouch(bool touching) { mIsGuiTouch = touching; }

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TOUCH_SENSOR; }
  enum Type { BUMPER, FORCE, FORCE3D };
  virtual Type deviceType() { return mDeviceType; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void setSolidMerger() override;
  void createOdeObjects() override;
  bool refreshSensorIfNeeded() override;

protected:
  // reimplemented protected functions

private:
  Type mDeviceType;

  // user accessible fields
  OmMFVector3 *mLookupTable;
  OmSFString *mType;
  OmSFDouble *mResolution;

  // other stuff
  bool mIsTouching;
  bool mWarnedNewtonBumperInert;
  bool mIsGuiTouch;
  OmSensor *mSensor;
  OmLookupTable *mLut;
  double mValues[3];                  // current sensor value according to lookup table
  mutable dJointFeedback *mFeedback;  // for measuring force feedback on the fixed joint
  bool mNeedToReconfigure;
  bool mWarnedNewtonInert;  // one-shot "reads zeros under Newton" warning latch

  // private functions
  OmTouchSensor &operator=(const OmTouchSensor &);  // non copyable
  OmNode *clone() const override { return new OmTouchSensor(*this); }
  void init();
  void computeValue();
  bool forceBehavior() const;
  // Newton mount-wrench read (_scratch/design_weld_touch.md T1, gated by
  // OMNISIM_NEWTON_TOUCH_FORCE): when this sensor was un-folded into its own
  // Newton body, fills `f` with the ODE-f1-compatible world-frame mount force
  // (negated cfrc_int, see OmNewtonBackend::touchForce) and returns true;
  // false = keep the ODE feedback path.
  bool newtonMountForce(OmVector3 &f) const;
  // true when the native contact set answered; `touching` is then the verdict
  bool newtonBumperTouching(bool &touching) const;
  void setODEDynamicFlag(const OmBaseNode *_node);
  void addConfigure(OmDataStream &);

private slots:
  void updateLookupTable();
  void updateType();
  void updateResolution();
};

#endif
