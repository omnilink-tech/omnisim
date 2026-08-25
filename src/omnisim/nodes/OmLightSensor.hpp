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

#ifndef OM_LIGHT_SENSOR_HPP
#define OM_LIGHT_SENSOR_HPP

#include <QtCore/QList>
#include "OmSolidDevice.hpp"

class OmLight;
class OmLookupTable;
class OmSensor;
class LightRay;

class OmLightSensor : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmLightSensor(OmTokenizer *tokenizer = NULL);
  OmLightSensor(const OmLightSensor &other);
  explicit OmLightSensor(const OmNode &other);
  virtual ~OmLightSensor() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_LIGHT_SENSOR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void prePhysicsStep(double ms) override;
  bool refreshSensorIfNeeded() override;

  // specific functions
  void rayCollisionCallback(dGeomID geom);

  void computeLightMeasurement(const OmLight *light, const OmVector3 &sensorAxis, const OmVector3 &sensorPos,
                               OmVector3 &lightDirection, double &distance, double &direct, double &attenuation) const;

private:
  // private functions
  OmLightSensor &operator=(const OmLightSensor &);  // non copyable
  OmNode *clone() const override { return new OmLightSensor(*this); }
  void init();
  void computeValue();
  void setupRaysAndComputeDirectContributions(bool finalSetup);
  void addSingleLightContribution(const OmLight *light, double direct, double attenuation);
  // Newton raycast service consumer: re-answer every sensor->light occlusion
  // ray from the live mjModel via OmNewtonBackend::raycastBatch when
  // OMNISIM_NEWTON_RAYCAST is enabled (value-parsed; default off until the
  // parity suite covers all consumers). No-op when the gate is off or the
  // Newton runtime is not running.
  void refreshRayCollisionsFromNewton();
  void postProcessLightMeasurement();
  void updateRaysSetupIfNeeded() override;
  void addConfigure(OmDataStream &);

  // user accessible fields
  OmMFVector3 *mLookupTable;
  OmSFColor *mColorFilter;
  OmSFBool *mOcclusion;
  OmSFDouble *mResolution;

  // other stuff
  OmLookupTable *mLut;
  OmSensor *mSensor;
  QList<LightRay *> mRayList;
  QVector<int> mNewtonExcludeBodies;  // cached own-robot Newton bodies ([-999] = built, empty)
  OmRgb *mRgbIntensity;
  double mValue;  // current sensor value according to lookup table
  bool mSensorUpdateRequested;
  bool mNeedToReconfigure;

private slots:
  void updateLookupTable();
  void updateResolution();
};

#endif
