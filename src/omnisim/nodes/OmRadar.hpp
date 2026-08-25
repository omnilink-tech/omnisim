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

#ifndef OM_RADAR_HPP
#define OM_RADAR_HPP

#include "OmSolidDevice.hpp"

class OmAffinePlane;
class OmSensor;
class OmRadarTarget;

class OmRadar : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmRadar(OmTokenizer *tokenizer = NULL);
  OmRadar(const OmRadar &other);
  explicit OmRadar(const OmNode &other);
  virtual ~OmRadar() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_RADAR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void prePhysicsStep(double ms) override;
  void postPhysicsStep() override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

  // field accessors
  double minRange() const { return mMinRange->value(); }
  double maxRange() const { return mMaxRange->value(); }
  double horizontalFieldOfView() const { return mHorizontalFieldOfView->value(); }
  double verticalFieldOfView() const { return mVerticalFieldOfView->value(); }
  double wavelength() const { return 0.299792458 / mFrequency->value(); }

private:
  // user accessible fields
  OmSFDouble *mMinRange;
  OmSFDouble *mMaxRange;
  OmSFDouble *mHorizontalFieldOfView;
  OmSFDouble *mVerticalFieldOfView;
  OmSFDouble *mMinAbsoluteRadialSpeed;
  OmSFDouble *mMinRadialSpeed;
  OmSFDouble *mMaxRadialSpeed;
  OmSFDouble *mCellDistance;
  OmSFDouble *mCellSpeed;
  OmSFDouble *mRangeNoise;
  OmSFDouble *mSpeedNoise;
  OmSFDouble *mAngularNoise;
  OmSFDouble *mAntennaGain;
  OmSFDouble *mFrequency;
  OmSFDouble *mTransmittedPower;
  OmSFBool *mOcclusion;
  OmSFDouble *mMinDetectableSignal;

  // precomputed values needed for target computation
  // converted transmittedPower from dBm to W
  double mTransmittedPowerInW;
  // converted antennaGain from dBi to to real gain
  double mRealGain;
  // converted minDetectableSignal from dBm into W
  double mReceivedPowerThreshold;
  double mReceivedPowerFactor;
  double mSensorElapsedTime;
  OmVector3 mPreviousRadarPosition;

  // other stuff
  OmSensor *mSensor;
  QList<OmRadarTarget *> mRadarTargets;
  QList<OmRadarTarget *> mInvalidRadarTargets;
  QMap<OmSolid *, OmVector3> mRadarTargetsPreviousTranslations;
  QVector<int> mNewtonExcludeBodies;  // cached own-robot Newton bodies ([-999] = built, empty)

  // private functions
  OmRadar &operator=(const OmRadar &);  // non copyable
  OmNode *clone() const override { return new OmRadar(*this); }
  void init();
  void mergeTargets(int startingIndex = 0);
  // Newton raycast service consumer: answer every target's occlusion ray from
  // the live mjModel via OmObjectDetection::refreshCollisionDepthsFromNewton,
  // behind the value-parsed OMNISIM_NEWTON_RAYCAST gate (default ON). Returns
  // false when the occlusion question could NOT be answered (gate off, Newton
  // runtime unavailable, or the service declined) -- the caller must then leave
  // the target list alone rather than declaring every target visible.
  bool refreshTargetOcclusionFromNewton();
  void removeOccludedTargets();
  void updateRaysSetupIfNeeded() override;
  void updateReceivedPowerFactor();
  void computeTargets(bool finalSetup, bool needCollisionDetection);
  bool setTargetProperties(const OmVector3 &radarPosition, const OmMatrix3 &radarRotation, const OmVector3 &radarAxis,
                           const OmAffinePlane &radarPlane, OmRadarTarget *radarTarget);

private slots:
  void updateMinRange();
  void updateMaxRange();
  void updateHorizontalFieldOfView();
  void updateVerticalFieldOfView();
  void updateMinAbsoluteRadialSpeed();
  void updateMinAndMaxRadialSpeed();
  void updateCellDistance();
  void updateCellSpeed();
  void updateRangeNoise();
  void updateSpeedNoise();
  void updateAngularNoise();
  void updateFrequency();
  void updateAntennaGain();
  void updateTransmittedPower();
  void updateMinDetectableSignal();
};

#endif
