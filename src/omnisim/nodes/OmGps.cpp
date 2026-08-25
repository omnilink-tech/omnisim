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

#include "OmGps.hpp"

#include "OmDataStream.hpp"
#include "OmFieldChecker.hpp"
#include "OmMathsUtilities.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmRandom.hpp"
#include "OmSFDouble.hpp"
#include "OmSensor.hpp"
#include "OmWorld.hpp"

#include "../../controller/c/messages.h"

#include <QtCore/QDataStream>

#include <cassert>

/* ------- OmUTMConverter ------- */

class OmUTMConverter {
  // This class is used to make conversion between latitude-longitude
  // and North-East coordinate using a Universal Transverse Mercator projection
  // (https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system)
  // The WGS84 World Geodetic System was chosen for the parameter of the reference
  // ellipsoid because this model is widely used and it is the one used by GPS.
  // WARNING: This projection should match the default one used in the OSM importer

public:
  // constructors and destructor
  OmUTMConverter() {
    mReferenceCoordinatesHasBeenSet = false;
    // set fix parameters of the WSG84
    mE0 = 500000;
    mK0 = 0.9996;
    mA = 6378137.0;
    mF = 1.0 / 298.2572236;

    mReferenceLatitude = 0.0;
    mReferenceLongitude = 0.0;
    mZone = 1;
    mLongitude0Radian = 0.0;
    mN0 = 0.0;
    mB = 0.0;
    mE = 0.0;
    mEsq = 0.0;
    mE0sq = 0.0;
    mNorth = 0.0;
    mEast = 0.0;
    mLatitude = 0.0;
    mLongitude = 0.0;
  }
  virtual ~OmUTMConverter() {}

  void setReferenceCoordinates(double latitude, double longitude) {
    mReferenceCoordinatesHasBeenSet = true;
    mReferenceLatitude = latitude;
    mReferenceLongitude = longitude;
    mLatitude = latitude;
    mLongitude = longitude;
    // compute parameters dependant on the reference coordinates
    mN0 = 0.0;
    if (latitude < 0.0)  // south hemisphere
      mN0 = 10000000.0;  // 10'000'000 [m]
    mB = mA * (1.0 - mF);
    mE = sqrt(1.0 - pow(mB, 2.0) / pow(mA, 2.0));
    mEsq = (1.0 - (mB / mA) * (mB / mA));
    mE0sq = mE * mE / (1.0 - pow(mE, 2.0));
    mZone = 1 + floor((mReferenceLongitude + 180.0) / 6.0);
    mLongitude0Radian = (3.0 + 6.0 * (mZone - 1.0) - 180.0) * M_PI / 180.0;
    computeNorthEast(latitude, longitude);
  }

  void computeNorthEast(double latitude, double longitude) {
    assert(mReferenceCoordinatesHasBeenSet);
    double latitudeRadian = (latitude * M_PI) / 180.0;
    double longitudeRadian = (longitude * M_PI) / 180.0;

    double N = mA / sqrt(1.0 - pow(mE * sin(latitudeRadian), 2.0));
    double T = pow(tan(latitudeRadian), 2);
    double C = mE0sq * pow(cos(latitudeRadian), 2);
    double A = (longitudeRadian - mLongitude0Radian) * cos(latitudeRadian);
    double M = latitudeRadian * (1.0 - mEsq * (1.0 / 4.0 + mEsq * (3.0 / 64.0 + 5.0 * mEsq / 256.0)));
    M -= sin(2.0 * latitudeRadian) * (mEsq * (3.0 / 8.0 + mEsq * (3.0 / 32.0 + 45.0 * mEsq / 1024.0)));
    M += sin(4.0 * latitudeRadian) * (mEsq * mEsq * (15.0 / 256.0 + mEsq * 45.0 / 1024.0));
    M -= sin(6.0 * latitudeRadian) * (mEsq * mEsq * mEsq * (35.0 / 3072.0));
    M *= mA;
    mEast =
      mK0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * mE0sq) / 120.0));
    mEast += mE0;
    double tmp0 = A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * mE0sq) / 720.0;
    double tmp1 = (5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0;
    mNorth = mK0 * (M + N * tan(latitudeRadian) * (A * A * (1.0 / 2.0 + A * A * (tmp0 + tmp1))));
    mNorth += mN0;
  }

  double getNorth() const { return mNorth; }
  double getEast() const { return mEast; }

  void computeLatitudeLongitude(double north, double east) {
    assert(mReferenceCoordinatesHasBeenSet);
    double e1 = (1.0 - sqrt(1.0 - pow(mE, 2.0))) / (1.0 + sqrt(1.0 - pow(mE, 2.0)));
    double M = (north - mN0) / mK0;
    double mu = M / (mA * (1.0 - mEsq * (1.0 / 4.0 + mEsq * (3.0 / 64.0 + 5.0 * mEsq / 256.0))));
    double phi1 = mu + e1 * (3.0 / 2.0 - 27.0 * e1 * e1 / 32.0) * sin(2.0 * mu) +
                  e1 * e1 * (21.0 / 16.0 - 55.0 * e1 * e1 / 32.0) * sin(4.0 * mu);
    phi1 += e1 * e1 * e1 * (sin(6.0 * mu) * 151.0 / 96.0 + e1 * sin(8.0 * mu) * 1097.0 / 512.0);
    double C1 = mE0sq * pow(cos(phi1), 2.0);
    double T1 = pow(tan(phi1), 2.0);
    double N1 = mA / sqrt(1.0 - pow(mE * sin(phi1), 2.0));
    double R1 = N1 * (1.0 - pow(mE, 2.0)) / (1.0 - pow(mE * sin(phi1), 2.0));
    double D = (east - mE0) / (N1 * mK0);
    double phi = (D * D) * (1.0 / 2.0 - D * D * (5.0 + 3.0 * T1 + 10.0 * C1 - 4.0 * C1 * C1 - 9.0 * mE0sq) / 24.0);
    phi += pow(D, 6.0) * (61.0 + 90.0 * T1 + 298.0 * C1 + 45.0 * T1 * T1 - 252.0 * mE0sq - 3.0 * C1 * C1) / 720.0;
    phi = phi1 - (N1 * tan(phi1) / R1) * phi;

    double latitude = phi;
    double tmp = (5.0 - 2.0 * C1 + 28.0 * T1 - 3.0 * C1 * C1 + 8.0 * mE0sq + 24.0 * T1 * T1);
    double longitude = D * (1.0 + D * D * ((-1.0 - 2.0 * T1 - C1) / 6.0 + D * D * tmp / 120.0)) / cos(phi1);
    longitude += mLongitude0Radian;

    // convert to degree
    mLatitude = (latitude * 180.0) / M_PI;
    mLongitude = (longitude * 180.0) / M_PI;
  }

  double getLatitude() const { return mLatitude; }
  double getLongitude() const { return mLongitude; }

private:
  // variables of the WSG84
  double mE0;  // East reference [m]
  double mK0;
  double mA;  // major radius of the ellipse [m]
  double mF;  // flattening of the ellipse

  // variables dependant of the reference coordinates
  bool mReferenceCoordinatesHasBeenSet;
  double mReferenceLatitude;
  double mReferenceLongitude;
  int mZone;                 // utm zone
  double mLongitude0Radian;  // longitude associated to the utm zone
  // Those intermediate variables are only dependant of the reference coordinates
  // we therefore don't want to recompute them at each conversion but only when
  // the reference coordinates changes
  double mN0;  // North reference [m]
  double mB;
  double mE;
  double mEsq;
  double mE0sq;

  // current coordinates (result of last conversion)
  double mNorth;
  double mEast;
  double mLatitude;
  double mLongitude;
};

/* ------- OmUTMConverter ------- */

void OmGps::init() {
  mType = findSFString("type");
  mAccuracy = findSFDouble("accuracy");
  mNoiseCorrelation = findSFDouble("noiseCorrelation");
  mResolution = findSFDouble("resolution");
  mSpeedNoise = findSFDouble("speedNoise");
  mSpeedResolution = findSFDouble("speedResolution");

  mSensor = NULL;
  mMeasuredSpeed = 0.0;
  mUTMConverter = NULL;
  mNeedToUpdateCoordinateSystem = false;
  mPreviousPosition = OmVector3(NAN, NAN, NAN);
  mMeasuredSpeed = 0.0;
}

OmGps::OmGps(OmTokenizer *tokenizer) : OmSolidDevice("GPS", tokenizer) {
  init();
}

OmGps::OmGps(const OmGps &other) : OmSolidDevice(other) {
  init();
}

OmGps::OmGps(const OmNode &other) : OmSolidDevice(other) {
  init();
}

OmGps::~OmGps() {
  delete mSensor;
  delete mUTMConverter;
}

void OmGps::preFinalize() {
  OmSolidDevice::preFinalize();
  mSensor = new OmSensor();
  mUTMConverter = new OmUTMConverter();
  if (OmWorld::instance()->worldInfo()->gpsCoordinateSystem() == "WGS84") {
    OmVector3 reference = OmWorld::instance()->worldInfo()->gpsReference();
    mUTMConverter->setReferenceCoordinates(reference[0], reference[1]);
  }
}

void OmGps::postFinalize() {
  OmSolidDevice::postFinalize();

  connect(OmWorld::instance()->worldInfo(), &OmWorldInfo::gpsCoordinateSystemChanged, this, &OmGps::updateCoordinateSystem);
  connect(OmWorld::instance()->worldInfo(), &OmWorldInfo::gpsReferenceChanged, this, &OmGps::updateReferences);
  connect(mNoiseCorrelation, &OmSFDouble::changed, this, &OmGps::updateCorrelation);
  connect(mResolution, &OmSFDouble::changed, this, &OmGps::updateResolution);
  connect(mSpeedNoise, &OmSFDouble::changed, this, &OmGps::updateSpeedNoise);
  connect(mSpeedResolution, &OmSFDouble::changed, this, &OmGps::updateSpeedResolution);
}

void OmGps::updateResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mResolution, -1.0, -1.0);
}

void OmGps::updateSpeedNoise() {
  OmFieldChecker::resetDoubleIfNegative(this, mSpeedNoise, 0.0);
}

void OmGps::updateSpeedResolution() {
  OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(this, mSpeedResolution, -1.0, -1.0);
}

void OmGps::updateCorrelation() {
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mNoiseCorrelation, 0.0, 1.0, 0.0);
}

void OmGps::updateCoordinateSystem() {
  mNeedToUpdateCoordinateSystem = true;
  updateReferences();
}

void OmGps::updateReferences() {
  if (mUTMConverter && OmWorld::instance()->worldInfo()->gpsCoordinateSystem() == "WGS84") {
    OmVector3 reference = OmWorld::instance()->worldInfo()->gpsReference();
    mUTMConverter->setReferenceCoordinates(reference[0], reference[1]);
  }
}

bool OmGps::refreshSensorIfNeeded() {
  if (!isPowerOn() || !mSensor->needToRefresh())
    return false;

  const OmVector3 &t = matrix().translation();

  // compute current position
  double accuracy = mAccuracy->value();
  double correlation = mNoiseCorrelation->value();
  static double noise[3] = {0, 0, 0};
  double ratio = 0;
  if (accuracy != 0.0 && correlation != 0.0)
    ratio = pow(correlation, mSensor->elapsedTime() / 1000.0);

  OmVector3 reference = OmWorld::instance()->worldInfo()->gpsReference();
  if (OmWorld::instance()->worldInfo()->gpsCoordinateSystem() == "WGS84") {
    // convert reference from lat-long into UTM X-Y coordinates
    mUTMConverter->computeNorthEast(reference[0], reference[1]);
    double altitude = reference[2];
    double north = mUTMConverter->getNorth();
    double east = mUTMConverter->getEast();
    const QString &coordinateSystem = OmWorld::instance()->worldInfo()->coordinateSystem();
    reference[coordinateSystem.indexOf('E')] = east;
    reference[coordinateSystem.indexOf('N')] = north;
    reference[coordinateSystem.indexOf('U')] = altitude;
  }

  for (int i = 0; i < 3; ++i)  // get exact position
    mMeasuredPosition[i] = t[i];

  for (int i = 0; i < 3; ++i) {
    // add the reference
    mMeasuredPosition[i] += reference[i];
    // add noise if necessary
    if (accuracy != 0.0) {
      // generate correlated gaussian number from previous one
      // https://www.cmu.edu/biolphys/deserno/pdf/corr_gaussian_random.pdf
      noise[i] = ratio * noise[i] + sqrt(1 - pow(ratio, 2)) * OmRandom::nextGaussian();
      mMeasuredPosition[i] += accuracy * noise[i];
    }
    // apply resolution if necessary
    if (mResolution->value() != -1.0)
      mMeasuredPosition[i] = OmMathsUtilities::discretize(mMeasuredPosition[i], mResolution->value());
  }

  if (OmWorld::instance()->worldInfo()->gpsCoordinateSystem().compare("WGS84") == 0) {
    // convert position from X-Y UTM coordinates into lat-long
    // we need to swap coordinates according to the world coordinate system
    const QString &coordinateSystem = OmWorld::instance()->worldInfo()->coordinateSystem();
    const double north = mMeasuredPosition[coordinateSystem.indexOf('N')];
    const double east = mMeasuredPosition[coordinateSystem.indexOf('E')];
    const double altitude = mMeasuredPosition[coordinateSystem.indexOf('U')];
    mUTMConverter->computeLatitudeLongitude(north, east);
    mMeasuredPosition[0] = mUTMConverter->getLatitude();
    mMeasuredPosition[1] = mUTMConverter->getLongitude();
    mMeasuredPosition[2] = altitude;
  }

  // P1.5 + Newton integration: dispatch through whichever backend
  // owns the upper Solid's body. Newton-backed Solids now get a
  // real GPS speed reading from the Newton override (was previously
  // reading bridge-proxy ODE state, which is stale or zero).
  OmSolid *const us = upperSolid();
  const OmBodyHandle upperHandle = us ? us->bodyHandle() : nullptr;
  if (upperHandle) {
    // ⚠ CHECK THE RETURN -- it was being discarded. getBodyPointVel returns -1
    // WITHOUT WRITING v[] when the Newton world is not running, and always on a
    // NEWTON=OFF build, so this published raw stack memory as a GPS speed.
    // Falling back to the finite-difference branch below is strictly better than
    // an arbitrary number, and it is the same estimate a physics-less GPS uses.
    double newVelocity[3] = {0.0, 0.0, 0.0};
    const OmVector3 &p = position();
    const double pt[3] = {p.x(), p.y(), p.z()};
    if (us->physicsBackend()->getBodyPointVel(upperHandle, pt, newVelocity) == 0)
      mSpeedVector = OmVector3(newVelocity[0], newVelocity[1], newVelocity[2]);
    else if (!mPreviousPosition.isNan())
      mSpeedVector = (t - mPreviousPosition) * 1000.0 / mSensor->elapsedTime();
    else
      mSpeedVector = OmVector3(NAN, NAN, NAN);
  } else if (!mPreviousPosition.isNan())
    // no physics node, compute it manually
    mSpeedVector = (t - mPreviousPosition) * 1000.0 / mSensor->elapsedTime();
  else
    mSpeedVector = OmVector3(NAN, NAN, NAN);

  // compute current speed [m/s]
  mMeasuredSpeed = mSpeedVector.length();
  mPreviousPosition = t;
  if (mSpeedNoise->value() > 0.0)
    mMeasuredSpeed *= 1.0 + mSpeedNoise->value() * OmRandom::nextGaussian();
  if (mSpeedResolution->value() != -1.0)
    mMeasuredSpeed = OmMathsUtilities::discretize(mMeasuredSpeed, mSpeedResolution->value());

  mSensor->updateTimer();
  return true;
}

void OmGps::reset(const QString &id) {
  OmSolidDevice::reset(id);
  mPreviousPosition = OmVector3(NAN, NAN, NAN);
  mMeasuredSpeed = 0.0;
  mSpeedVector = OmVector3();
}

void OmGps::handleMessage(QDataStream &stream) {
  unsigned char command;
  short refreshRate;
  stream >> command;

  switch (command) {
    case C_SET_SAMPLING_PERIOD:
      stream >> refreshRate;
      mSensor->setRefreshRate(refreshRate);
      break;
    default:
      assert(0);
  }
}

void OmGps::writeAnswer(OmDataStream &stream) {
  if (mNeedToUpdateCoordinateSystem)
    addConfigureToStream(stream);

  if (refreshSensorIfNeeded() || mSensor->hasPendingValue()) {
    stream << tag();
    stream << (unsigned char)C_GPS_DATA;
    for (int i = 0; i < 3; ++i)
      stream << (double)mMeasuredPosition[i];
    stream << (double)mMeasuredSpeed;
    for (int i = 0; i < 3; ++i)
      stream << (double)mSpeedVector[i];

    mSensor->resetPendingValue();
  }
}

void OmGps::addConfigureToStream(OmDataStream &stream) {
  stream << (short unsigned int)tag();
  stream << (unsigned char)C_CONFIGURE;
  if (OmWorld::instance()->worldInfo()->gpsCoordinateSystem().compare("WGS84") == 0)
    stream << (int)WGS84;
  else
    stream << (int)LOCAL;
  mNeedToUpdateCoordinateSystem = false;
}

void OmGps::writeConfigure(OmDataStream &stream) {
  mSensor->connectToRobotSignal(robot());
  addConfigureToStream(stream);
}
