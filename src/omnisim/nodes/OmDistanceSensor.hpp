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

#ifndef OM_DISTANCE_SENSOR_HPP
#define OM_DISTANCE_SENSOR_HPP

#include "OmSolidDevice.hpp"

class OmSFDouble;
class OmSFInt;
class OmSFString;
class OmSensor;
class OmLookupTable;

class QDataStream;
class SensorRay;
typedef struct dxGeom *dGeomID;
struct dContactGeom;

class OmDistanceSensor : public OmSolidDevice {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmDistanceSensor(OmTokenizer *tokenizer = NULL);
  OmDistanceSensor(const OmDistanceSensor &other);
  explicit OmDistanceSensor(const OmNode &other);
  virtual ~OmDistanceSensor() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_DISTANCE_SENSOR; }
  void preFinalize() override;
  void postFinalize() override;
  void handleMessage(QDataStream &) override;
  void writeAnswer(OmDataStream &) override;
  void writeConfigure(OmDataStream &) override;
  void createOdeObjects() override;
  void prePhysicsStep(double ms) override;
  bool refreshSensorIfNeeded() override;

  // other functions
  void rayCollisionCallback(OmGeometry *object, dGeomID rayGeom, const dContactGeom *);

private slots:
  void updateRaySetup();

private:
  enum { GENERIC, INFRA_RED, SONAR, LASER };

  // private functions
  OmDistanceSensor &operator=(const OmDistanceSensor &);  // non copyable
  OmNode *clone() const override { return new OmDistanceSensor(*this); }
  void init();
  // Newton raycast service consumer: fill mRays from the live mjModel via
  // OmNewtonBackend::raycastBatch when OMNISIM_NEWTON_RAYCAST is enabled
  // (value-parsed; default off until the parity suite covers all consumers).
  // No-op when the gate is off or the Newton runtime is not running.
  void refreshRaysFromNewton();
  QVector<int> mNewtonExcludeBodies;  // cached own-robot Newton bodies ([-999] = built, empty)
  void polarTo3d(double alpha, double theta, int i);
  void setupRayDirs();
  void setupRayWeights();
  void setSensorRays();
  void createOdeRays();
  void computeValue();
  void addConfigure(OmDataStream &stream);
  void updateRaysSetupIfNeeded() override;

  // user accessible fields
  OmMFVector3 *mLookupTable;
  OmSFString *mType;
  OmSFDouble *mAperture;
  OmSFInt *mNumberOfRays;
  OmSFDouble *mGaussianWidth;
  OmSFDouble *mResolution;
  OmSFDouble *mRedColorSensitivity;

  // other fields
  OmLookupTable *mLut;
  OmSensor *mSensor;
  SensorRay *mRays;  // rays
  bool mIsSubscribedToRayTracing;

  double mValue;     // current sensor value according to lookup table
  double mDistance;  // current averaged measured distance
  int mRayType;      // GENERIC, INFRA_RED, SONAR or LASER
  int mNRays;
  bool mNeedToReconfigure;
};

#endif
