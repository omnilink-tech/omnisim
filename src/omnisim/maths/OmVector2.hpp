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

#ifndef OM_VECTOR2_HPP
#define OM_VECTOR2_HPP

//
// Description: 2D vector
//

#include "OmPrecision.hpp"

#include <QtCore/QStringList>

#include <cassert>
#include <cfloat>
#include <cmath>

class OmVector2 {
public:
  // construct as null vector
  OmVector2() : mX(0.0), mY(0.0) {}

  // construct from other vectors
  OmVector2(double x, double y) : mX(x), mY(y) {}
  OmVector2(const OmVector2 &other) : mX(other.mX), mY(other.mY) {}
  explicit OmVector2(const double v[2]) : mX(v[0]), mY(v[1]) {}
  explicit OmVector2(const float v[2]) : mX(v[0]), mY(v[1]) {}
  explicit OmVector2(const QString &string) {
    const QStringList splittedText = string.split(' ');
    assert(splittedText.count() == 2);
    mX = splittedText[0].toDouble();
    mY = splittedText[1].toDouble();
  }

  // pointer representation
  const double *ptr() const { return &mX; }

  // getters
  double x() const { return mX; }
  double y() const { return mY; }
  double operator[](int index) const { return *(&mX + index); }

  // setters
  void setX(double x) { mX = x; }
  void setY(double y) { mY = y; }
  void setXy(double x, double y) {
    mX = x;
    mY = y;
  }
  void setXy(const double v[2]) {
    mX = v[0];
    mY = v[1];
  }
  void setXy(const float v[2]) {
    mX = v[0];
    mY = v[1];
  }
  double &operator[](int index) { return *(&mX + index); }

  // uniry plus and minus signs: +v, -v
  const OmVector2 &operator+() const { return *this; }
  OmVector2 operator-() const { return OmVector2(-mX, -mY); }

  // vector vector operations: v+w, v-w, v+=w, v-=w, v*=w
  OmVector2 operator+(const OmVector2 &v) const { return OmVector2(mX + v.mX, mY + v.mY); }
  OmVector2 operator-(const OmVector2 &v) const { return OmVector2(mX - v.mX, mY - v.mY); }
  OmVector2 &operator+=(const OmVector2 &v) {
    mX += v.mX;
    mY += v.mY;
    return *this;
  }
  OmVector2 &operator-=(const OmVector2 &v) {
    mX -= v.mX;
    mY -= v.mY;
    return *this;
  }
  OmVector2 &operator*=(const OmVector2 &v) {
    mX *= v.mX;
    mY *= v.mY;
    return *this;
  }

  // vector scalar operations: v*s, v/s, s*v, v*=s, v/=s
  OmVector2 operator*(double d) const { return OmVector2(mX * d, mY * d); }
  OmVector2 operator/(double d) const {
    double inv = 1.0 / d;
    return OmVector2(mX * inv, mY * inv);
  }
  friend OmVector2 operator*(double d, const OmVector2 &v) { return OmVector2(d * v.mX, d * v.mY); }
  OmVector2 &operator*=(double d) {
    mX *= d;
    mY *= d;
    return *this;
  }
  OmVector2 &operator/=(double d) {
    double inv = 1.0 / d;
    mX *= inv;
    mY *= inv;
    return *this;
  }

  // assignment: v = w
  OmVector2 &operator=(const OmVector2 &v) {
    mX = v.mX;
    mY = v.mY;
    return *this;
  }

  // length and squared length
  double length() const { return sqrt(mX * mX + mY * mY); }
  double length2() const { return mX * mX + mY * mY; }

  // distance and squared distance
  double distance(const OmVector2 &v) const { return (*this - v).length(); }
  double distance2(const OmVector2 &v) const { return (*this - v).length2(); }

  // returns a unit vector with the same direction: / length
  void normalize() {
    const double l = length();
    if (l)
      *this /= l;
  }
  OmVector2 normalized() const {
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
  }

  // vector comparison
  bool operator==(const OmVector2 &v) const { return mX == v.mX && mY == v.mY; }
  bool operator!=(const OmVector2 &v) const { return mX != v.mX || mY != v.mY; }

  // dot product
  double dot(const OmVector2 &v) const { return mX * v.mX + mY * v.mY; }

  // null test
  bool isNull() const { return mX == 0.0 && mY == 0.0; }

  // text conversion
  QString toString(OmPrecision::Level level = OmPrecision::DOUBLE_MAX) const {
    return QString("%1 %2").arg(OmPrecision::doubleToString(mX, level)).arg(OmPrecision::doubleToString(mY, level));
  }

private:
  double mX, mY;
};

#endif
