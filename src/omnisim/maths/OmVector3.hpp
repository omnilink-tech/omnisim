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

#ifndef OM_VECTOR3_HPP
#define OM_VECTOR3_HPP

//
// Description: 3D vector
//

#include "OmPrecision.hpp"

#include <QtCore/QStringList>

#include <cassert>
#include <cfloat>
#include <cmath>

class OmVector3 {
public:
  // construct as null vector
  OmVector3() : mX(0.0), mY(0.0), mZ(0.0) {}

  // construct from other vector
  OmVector3(double x, double y, double z) : mX(x), mY(y), mZ(z) {}
  OmVector3(const OmVector3 &other) : mX(other.mX), mY(other.mY), mZ(other.mZ) {}
  explicit OmVector3(const double v[3]) : mX(v[0]), mY(v[1]), mZ(v[2]) {}
  explicit OmVector3(const float v[3]) : mX(v[0]), mY(v[1]), mZ(v[2]) {}
  explicit OmVector3(const QString &string) {
    const QStringList splittedText = string.split(' ');
    assert(splittedText.count() == 3);
    mX = splittedText[0].toDouble();
    mY = splittedText[1].toDouble();
    mZ = splittedText[2].toDouble();
  }

  // pointer representation
  const double *ptr() const { return &mX; }

  // getters
  double x() const { return mX; }
  double y() const { return mY; }
  double z() const { return mZ; }
  double operator[](int index) const { return *(&mX + index); }

  // setters
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setZ(double z) { mZ = z; }
  void setXyz(double x, double y, double z) {
    mX = x;
    mY = y;
    mZ = z;
  }
  void setXyz(const double v[3]) {
    mX = v[0];
    mY = v[1];
    mZ = v[2];
  }
  void setXyz(const float v[3]) {
    mX = v[0];
    mY = v[1];
    mZ = v[2];
  }
  double &operator[](int index) { return *(&mX + index); }

  // unary plus and minus signs: +v, -v
  const OmVector3 &operator+() const { return *this; }
  OmVector3 operator-() const { return OmVector3(-mX, -mY, -mZ); }
  OmVector3 abs() const { return OmVector3(fabs(mX), fabs(mY), fabs(mZ)); }

  // vector vector operations: v+w, v-w, v*w, v+=w, v-=w, v*=w
  OmVector3 operator+(const OmVector3 &v) const { return OmVector3(mX + v.mX, mY + v.mY, mZ + v.mZ); }
  OmVector3 operator-(const OmVector3 &v) const { return OmVector3(mX - v.mX, mY - v.mY, mZ - v.mZ); }
  OmVector3 operator*(const OmVector3 &v) const { return OmVector3(mX * v.mX, mY * v.mY, mZ * v.mZ); }
  OmVector3 &operator+=(const OmVector3 &v) {
    mX += v.mX;
    mY += v.mY;
    mZ += v.mZ;
    return *this;
  }
  OmVector3 &operator-=(const OmVector3 &v) {
    mX -= v.mX;
    mY -= v.mY;
    mZ -= v.mZ;
    return *this;
  }
  OmVector3 &operator*=(const OmVector3 &v) {
    mX *= v.mX;
    mY *= v.mY;
    mZ *= v.mZ;
    return *this;
  }
  OmVector3 &operator/=(const OmVector3 &v) {
    mX /= v.mX;
    mY /= v.mY;
    mZ /= v.mZ;
    return *this;
  }

  // vector scalar operations: v*s, v/s, s*v, v*=s, v/=s
  OmVector3 operator*(double d) const { return OmVector3(mX * d, mY * d, mZ * d); }
  OmVector3 operator/(double d) const {
    double inv = 1.0 / d;
    return OmVector3(mX * inv, mY * inv, mZ * inv);
  }
  friend OmVector3 operator*(double d, const OmVector3 &v) { return OmVector3(d * v.mX, d * v.mY, d * v.mZ); }
  OmVector3 &operator*=(double d) {
    mX *= d;
    mY *= d;
    mZ *= d;
    return *this;
  }
  OmVector3 &operator/=(double d) {
    double inv = 1.0 / d;
    mX *= inv;
    mY *= inv;
    mZ *= inv;
    return *this;
  }

  // assignment: v = w
  OmVector3 &operator=(const OmVector3 &v) {
    mX = v.mX;
    mY = v.mY;
    mZ = v.mZ;
    return *this;
  }

  // length and squared length (magnitude)
  double length() const { return sqrt(mX * mX + mY * mY + mZ * mZ); }
  double length2() const { return mX * mX + mY * mY + mZ * mZ; }

  // distance and squared distance
  double distance(const OmVector3 &v) const { return (*this - v).length(); }
  double distance2(const OmVector3 &v) const { return (*this - v).length2(); }

  // normalization: |length| = 1.0
  void normalize() {
    const double l = length();
    if (l)
      *this /= l;
  }
  OmVector3 normalized() const {
    const double l = length();
    return l ? *this / l : *this;
  }

  void clamp(double min = -FLT_MAX, double max = FLT_MAX) {
    if (mX > max)
      mX = max;
    else if (mX < min)
      mX = min;
    if (mY > max)
      mY = max;
    else if (mY < min)
      mY = min;
    if (mZ > max)
      mZ = max;
    else if (mZ < min)
      mZ = min;
  }

  OmVector3 rounded(OmPrecision::Level level) const {
    return OmVector3(OmPrecision::roundValue(mX, level), OmPrecision::roundValue(mY, level),
                     OmPrecision::roundValue(mZ, level));
  }

  // dot product
  double dot(const OmVector3 &v) const { return mX * v.mX + mY * v.mY + mZ * v.mZ; }

  // cross product
  OmVector3 cross(const OmVector3 &v) const {
    return OmVector3(mY * v.mZ - mZ * v.mY, mZ * v.mX - mX * v.mZ, mX * v.mY - mY * v.mX);
  }

  // angle between two vectors (in radians)
  double angle(const OmVector3 &v) const {
    const double l = length2();
    const double lv = v.length2();
    const double s = (l && lv) ? dot(v) / sqrt(l * lv) : 0.0;
    assert(std::abs(s) < 1.0000000001);
    return (s >= 1.0) ? 0 : (s <= -1.0) ? M_PI : acos(s);
  }

  // vector comparison
  bool almostEquals(const OmVector3 &v, double tolerance = OmPrecision::DOUBLE_EQUALITY_TOLERANCE) const {
    return std::abs(mX - v.mX) < tolerance && std::abs(mY - v.mY) < tolerance && std::abs(mZ - v.mZ) < tolerance;
  }
  bool operator==(const OmVector3 &v) const { return mX == v.mX && mY == v.mY && mZ == v.mZ; }
  bool operator!=(const OmVector3 &v) const { return mX != v.mX || mY != v.mY || mZ != v.mZ; }

  // null test
  bool isNull() const { return mX == 0.0 && mY == 0.0 && mZ == 0.0; }

  // validity test for WREN
  bool isNan() const { return std::isnan(mX) || std::isnan(mY) || std::isnan(mZ); }

  void toFloatArray(float *out) const {
    out[0] = static_cast<float>(mX);
    out[1] = static_cast<float>(mY);
    out[2] = static_cast<float>(mZ);
  }

  // test if this point is on a given line segment
  bool isOnEdgeBetweenVertices(const OmVector3 &lineStart, const OmVector3 &lineEnd, const double tolerance = 0.000001) const {
    const OmVector3 lineSegment = lineEnd - lineStart;
    const OmVector3 toPoint = OmVector3(mX, mY, mZ) - lineStart;

    // the points aren't aligned
    if (!lineSegment.cross(toPoint).almostEquals(OmVector3(), tolerance))
      return false;

    // the point isn't on the segment
    if (lineSegment.dot(toPoint) < 0 || lineSegment.dot(toPoint) > lineSegment.length2())
      return false;

    return true;
  }

  // text conversion
  QString toString(OmPrecision::Level level = OmPrecision::DOUBLE_MAX) const {
    return QString("%1 %2 %3")
      .arg(OmPrecision::doubleToString(mX, level))
      .arg(OmPrecision::doubleToString(mY, level))
      .arg(OmPrecision::doubleToString(mZ, level));
  }

private:
  double mX, mY, mZ;
};

#endif
