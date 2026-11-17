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

#ifndef OM_GPS_HPP
#define OM_GPS_HPP

#include "OmSolidDevice.hpp"

class OmSFString;
class OmSFDouble;
class OmSensor;
class OmUTMConverter;

class OmGps : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmGps(OmTokenizer *tokenizer = NULL);
  OmGps(const OmGps &other);
  explicit OmGps(const OmNode &other);
  virtual ~OmGps() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_GPS; }
  enum CoordinateSystem { LOCAL = 0, WGS84 };
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  bool refreshSensorIfNeeded() override;
  void reset(const QString &id) override;

private:
  // user accessible fields
  OmSFString *mType;
  OmSFDouble *mAccuracy;
  OmSFDouble *mNoiseCorrelation;
  OmSFDouble *mResolution;
  OmSFDouble *mSpeedNoise;
  OmSFDouble *mSpeedResolution;

  // other fields
  OmSensor *mSensor;
  OmVector3 mMeasuredPosition;
  OmVector3 mPreviousPosition;
  OmVector3 mSpeedVector;
  double mMeasuredSpeed;
  OmUTMConverter *mUTMConverter;
  bool mNeedToUpdateCoordinateSystem;

  OmGps &operator=(const OmGps &);  // non copyable
  OmNode *clone() const override { return new OmGps(*this); }
  void init();

  void addConfigureToStream(OmDataStream &stream);

private slots:
  void updateResolution();
  void updateSpeedNoise();
  void updateSpeedResolution();
  void updateCorrelation();
  void updateCoordinateSystem();
  void updateReferences();
};

#endif
