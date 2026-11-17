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

#ifndef OM_ACCELEROMETER_HPP
#define OM_ACCELEROMETER_HPP

#include "OmSolidDevice.hpp"

class OmSensor;
class OmLookupTable;

class OmAccelerometer : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmAccelerometer(OmTokenizer *tokenizer = NULL);
  OmAccelerometer(const OmAccelerometer &other);
  explicit OmAccelerometer(const OmNode &other);
  virtual ~OmAccelerometer() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ACCELEROMETER; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  bool refreshSensorIfNeeded() override;

  // field accessors
  void setAcceleration(double x, double y, double z) {
    mValues[0] = x;
    mValues[1] = y;
    mValues[2] = z;
  }
  const double *getAcceleration() const { return mValues; }

private:
  // user accessible fields
  OmMFVector3 *mLookupTable;
  OmSFBool *mXAxis, *mYAxis, *mZAxis;
  OmSFDouble *mResolution;

  // other stuff
  OmSensor *mSensor;
  OmLookupTable *mLut;
  double mValues[3];  // current sensor value according to lookup table
  double mVelocity[3];
  bool mNeedToReconfigure;
  bool mWarningWasPrinted;

  // private functions
  OmAccelerometer &operator=(const OmAccelerometer &);  // non copyable
  OmNode *clone() const override { return new OmAccelerometer(*this); }
  void addConfigure(OmDataStream &);
  void init();
  void computeValue();

private slots:
  void updateLookupTable();
  void updateResolution();
};

#endif
