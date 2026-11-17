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

#ifndef OM_MATRIX3_HPP
#define OM_MATRIX3_HPP

//
// Description: 3x3 rotation matrix
//  OmVector3 can be multiplied (=rotated) by this kind of matrix
//

#include "OmVector3.hpp"

#include <cmath>
#include <cstring>

class OmQuaternion;

class OmMatrix3 {
public:
  // construct as identity matrix
  OmMatrix3() { setIdentity(); }

  // construct from other 3x3 matrix
  OmMatrix3(const OmMatrix3 &m) { memcpy(mM, m.mM, sizeof(mM)); }
  explicit OmMatrix3(const double m[9]) { memcpy(mM, m, sizeof(mM)); }
  explicit OmMatrix3(const double m[3][3]) { memcpy(mM, m, sizeof(mM)); }
  OmMatrix3(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22) {
    mM[0] = m00;
    mM[1] = m01;
    mM[2] = m02;
    mM[3] = m10;
    mM[4] = m11;
    mM[5] = m12;
    mM[6] = m20;
    mM[7] = m21;
    mM[8] = m22;
  }

  // construct from axis/angle rotation
  OmMatrix3(double rx, double ry, double rz, double angle) { fromAxisAngle(rx, ry, rz, angle); }
  OmMatrix3(const OmVector3 &r, double angle) { fromAxisAngle(r.x(), r.y(), r.z(), angle); }
  OmMatrix3(double rx, double ry, double rz) { fromEulerAngles(rx, ry, rz); };
  explicit OmMatrix3(const OmQuaternion &q) { fromQuaternion(q); }

  // construct from a set column vectors
  OmMatrix3(const OmVector3 &ex, const OmVector3 &ey, const OmVector3 &ez) {
    mM[0] = ex.x();
    mM[1] = ey.x();
    mM[2] = ez.x();
    mM[3] = ex.y();
    mM[4] = ey.y();
    mM[5] = ez.y();
    mM[6] = ex.z();
    mM[7] = ey.z();
    mM[8] = ez.z();
  }

  // assignment: M = N
  OmMatrix3 &operator=(const OmMatrix3 &m) {
    memcpy(mM, m.mM, sizeof(mM));
    return *this;
  }

  // matrix * matrix multiplication
  OmMatrix3 &operator*=(const OmMatrix3 &m) {
    *this = *this * m;
    return *this;
  }
  OmMatrix3 operator*(const OmMatrix3 &m) const;
  OmVector3 operator*(const OmVector3 &v) const;

  // scaling rows
  OmMatrix3 scaled(double x, double y, double z) const;
  OmMatrix3 scaled(const OmVector3 &s) const { return scaled(s.x(), s.y(), s.z()); }
  void scale(double x, double y, double z);
  void scale(const OmVector3 &s) { scale(s.x(), s.y(), s.z()); }

  // matrix * vector multiplication
  friend OmVector3 operator*(const OmVector3 &v, const OmMatrix3 &m);

  // matrix * scalar multiplication
  inline OmMatrix3 operator*(double s) const;
  OmMatrix3 &operator*=(double s) {
    *this = *this * s;
    return *this;
  }

  // getters
  double operator()(int row, int col) const { return *(mM + row * 3 + col); }
  OmVector3 row(int row) const { return OmVector3(mM + row * 3); }
  OmVector3 column(int col) const { return OmVector3(mM[col], mM[col + 3], mM[col + 6]); }

  // make identity
  void setIdentity();

  // transpose
  void transpose();
  OmMatrix3 transposed() const;

  // inverse
  void inverse();

  // to other rotation types
  OmQuaternion toQuaternion() const;
  OmVector3 toEulerAnglesZYX() const;

  QString toString(OmPrecision::Level level) const;

  // assign from other types of rotation
  void fromOpenGlMatrix(const double m[16]);
  void fromAxisAngle(double rx, double ry, double rz, double angle);
  void fromEulerAngles(double rx, double ry, double rz);
  void fromAxisAngle(const OmVector3 &axis, double angle) { fromAxisAngle(axis.x(), axis.y(), axis.z(), angle); }
  void fromQuaternion(const OmQuaternion &q);

private:
  double mM[9];
};

inline OmMatrix3 OmMatrix3::operator*(const OmMatrix3 &m) const {
  return OmMatrix3(mM[0] * m.mM[0] + mM[1] * m.mM[3] + mM[2] * m.mM[6], mM[0] * m.mM[1] + mM[1] * m.mM[4] + mM[2] * m.mM[7],
                   mM[0] * m.mM[2] + mM[1] * m.mM[5] + mM[2] * m.mM[8],

                   mM[3] * m.mM[0] + mM[4] * m.mM[3] + mM[5] * m.mM[6], mM[3] * m.mM[1] + mM[4] * m.mM[4] + mM[5] * m.mM[7],
                   mM[3] * m.mM[2] + mM[4] * m.mM[5] + mM[5] * m.mM[8],

                   mM[6] * m.mM[0] + mM[7] * m.mM[3] + mM[8] * m.mM[6], mM[6] * m.mM[1] + mM[7] * m.mM[4] + mM[8] * m.mM[7],
                   mM[6] * m.mM[2] + mM[7] * m.mM[5] + mM[8] * m.mM[8]);
}
// Matrix times vector & vector times matrix

inline OmVector3 OmMatrix3::operator*(const OmVector3 &v) const {
  return OmVector3(mM[0] * v.x() + mM[1] * v.y() + mM[2] * v.z(), mM[3] * v.x() + mM[4] * v.y() + mM[5] * v.z(),
                   mM[6] * v.x() + mM[7] * v.y() + mM[8] * v.z());
}
// This operation allows direct multiplication by the transposed matrix avoiding thus unnecessary calls to transpose methods
inline OmVector3 operator*(const OmVector3 &v, const OmMatrix3 &m) {
  return OmVector3(v.x() * m.mM[0] + v.y() * m.mM[3] + v.z() * m.mM[6], v.x() * m.mM[1] + v.y() * m.mM[4] + v.z() * m.mM[7],
                   v.x() * m.mM[2] + v.y() * m.mM[5] + v.z() * m.mM[8]);
}

// rotate (assuming the rotation vector is normalized)
inline void OmMatrix3::fromAxisAngle(double rx, double ry, double rz, double angle) {
  const double c = cos(angle);
  const double s = sin(angle);

  const double t1 = 1.0 - c;
  const double t2 = rx * rz * t1;
  const double t3 = rx * ry * t1;
  const double t4 = ry * rz * t1;

  mM[0] = rx * rx * t1 + c;
  mM[1] = t3 - rz * s;
  mM[2] = t2 + ry * s;
  mM[3] = t3 + rz * s;
  mM[4] = ry * ry * t1 + c;
  mM[5] = t4 - rx * s;
  mM[6] = t2 - ry * s;
  mM[7] = t4 + rx * s;
  mM[8] = rz * rz * t1 + c;
}

inline void OmMatrix3::fromEulerAngles(double rx, double ry, double rz) {
  // Reference: https://www.geometrictools.com/Documentation/EulerAngles.pdf

  const double cx = cos(rx);
  const double sx = sin(rx);
  const double cy = cos(ry);
  const double sy = sin(ry);
  const double cz = cos(rz);
  const double sz = sin(rz);

  mM[0] = cy * cz;
  mM[1] = -cy * sz;
  mM[2] = sy;
  mM[3] = cz * sx * sy + cx * sz;
  mM[4] = cx * cz - sx * sy * sz;
  mM[5] = -cy * sx;
  mM[6] = -cx * cz * sy + sx * sz;
  mM[7] = cz * sx + cx * sy * sz;
  mM[8] = cx * cy;
}

inline void OmMatrix3::transpose() {
  double t[9];
  memcpy(t, mM, sizeof(t));
  mM[1] = t[3];
  mM[2] = t[6];
  mM[3] = t[1];
  mM[5] = t[7];
  mM[6] = t[2];
  mM[7] = t[5];
}

inline OmMatrix3 OmMatrix3::transposed() const {
  return OmMatrix3(mM[0], mM[3], mM[6], mM[1], mM[4], mM[7], mM[2], mM[5], mM[8]);
}

inline void OmMatrix3::inverse() {
  double t[9];
  memcpy(t, mM, sizeof(t));
  double determinant =
    t[0] * (t[4] * t[8] - t[7] * t[5]) + t[1] * (t[3] * t[8] - t[5] * t[6]) + t[2] * (t[3] * t[7] - t[4] * t[6]);
  double invdet = 1.0 / determinant;
  mM[0] = (t[4] * t[8] - t[7] * t[5]) * invdet;
  mM[1] = -(t[1] * t[8] - t[2] * t[7]) * invdet;
  mM[2] = (t[1] * t[5] - t[2] * t[4]) * invdet;
  mM[3] = -(t[3] * t[8] - t[5] * t[6]) * invdet;
  mM[4] = (t[0] * t[8] - t[2] * t[6]) * invdet;
  mM[5] = -(t[0] * t[5] - t[3] * t[2]) * invdet;
  mM[6] = (t[3] * t[7] - t[6] * t[4]) * invdet;
  mM[7] = -(t[0] * t[7] - t[6] * t[1]) * invdet;
  mM[8] = (t[0] * t[4] - t[3] * t[1]) * invdet;
}

inline void OmMatrix3::fromOpenGlMatrix(const double m[16]) {
  mM[0] = m[0];
  mM[1] = m[4];
  mM[2] = m[8];
  mM[3] = m[1];
  mM[4] = m[5];
  mM[5] = m[9];
  mM[6] = m[2];
  mM[7] = m[6];
  mM[8] = m[10];
}

inline OmMatrix3 OmMatrix3::scaled(double x, double y, double z) const {
  return OmMatrix3(x * mM[0], y * mM[1], z * mM[2], x * mM[3], y * mM[4], z * mM[5], x * mM[6], y * mM[7], z * mM[8]);
}

inline void OmMatrix3::scale(double x, double y, double z) {
  mM[0] *= x;
  mM[3] *= x;
  mM[6] *= x;

  mM[1] *= y;
  mM[4] *= y;
  mM[7] *= y;

  mM[2] *= z;
  mM[5] *= z;
  mM[8] *= z;
}

inline OmMatrix3 OmMatrix3::operator*(double s) const {
  return OmMatrix3(mM[0] * s, mM[1] * s, mM[2] * s, mM[3] * s, mM[4] * s, mM[5] * s, mM[6] * s, mM[7] * s, mM[8] * s);
}

inline QString OmMatrix3::toString(OmPrecision::Level level = OmPrecision::Level::DOUBLE_MAX) const {
  QString result = "[\n";
  result += QString("  %1 %2 %3\n")
              .arg(OmPrecision::doubleToString(mM[0], level))
              .arg(OmPrecision::doubleToString(mM[1], level))
              .arg(OmPrecision::doubleToString(mM[2], level));
  result += QString("  %1 %2 %3\n")
              .arg(OmPrecision::doubleToString(mM[3], level))
              .arg(OmPrecision::doubleToString(mM[4], level))
              .arg(OmPrecision::doubleToString(mM[5], level));
  result += QString("  %1 %2 %3\n")
              .arg(OmPrecision::doubleToString(mM[6], level))
              .arg(OmPrecision::doubleToString(mM[7], level))
              .arg(OmPrecision::doubleToString(mM[8], level));
  result += "]\n";
  return result;
}

#endif
