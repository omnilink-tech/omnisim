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

#ifndef OM_MATRIX4_HPP
#define OM_MATRIX4_HPP

//
// Description: 4x4 transformation matrix
//  A OmMatrix4 represents a rotation, a translation and a scale
//  OmVector4 can be multiplied (=transformed) by this kind of matrix
//

#include "OmQuaternion.hpp"
#include "OmVector3.hpp"
#include "OmVector4.hpp"

#include <cmath>
#include <cstring>

class OmRotation;
class OmMatrix3;

class OmMatrix4 {
public:
  // construct as identity matrix
  OmMatrix4() { setIdentity(); }

  // construct from other 4x4 matrix
  OmMatrix4(const OmMatrix4 &m) { memcpy(mM, m.mM, sizeof(mM)); }
  explicit OmMatrix4(const double m[16]) { memcpy(mM, m, sizeof(mM)); }
  explicit OmMatrix4(const double m[4][4]) { memcpy(mM, m, sizeof(mM)); }
  OmMatrix4(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20,
            double m21, double m22, double m23, double m30, double m31, double m32, double m33);

  // construct from translation and rotation
  OmMatrix4(double tx, double ty, double tz, double rx, double ry, double rz, double angle);
  OmMatrix4(const OmVector3 &trans, const OmRotation &rot);

  // construct from VRML representation
  OmMatrix4(double tx, double ty, double tz,                // translation
            double rx, double ry, double rz, double angle,  // rotation
            double sx, double sy, double sz);               // scale

  // assignment: M = N
  OmMatrix4 &operator=(const OmMatrix4 &m) {
    memcpy(mM, m.mM, sizeof(mM));
    return *this;
  }

  // matrix * matrix multiplication
  OmMatrix4 &operator*=(const OmMatrix4 &m) {
    *this = *this * m;
    return *this;
  }
  OmMatrix4 operator*(const OmMatrix4 &m) const;
  OmMatrix4 dotTransposed(const OmMatrix4 &m) const;

  // matrix * vector multiplication
  OmVector4 operator*(const OmVector4 &v) const;
  OmVector3 operator*(const OmVector3 &v) const;

  // vector * matrix multiplication (i.e. matrix.transposed() * vector)
  friend OmVector4 operator*(const OmVector4 &v, const OmMatrix4 &m);
  friend OmVector3 operator*(const OmVector3 &v, const OmMatrix4 &m);

  // matrix * scalar multiplication
  inline OmMatrix4 operator*(double s) const;
  OmMatrix4 &operator*=(double s) {
    *this = *this * s;
    return *this;
  }

  // getters
  double operator()(int row, int col) const { return *(mM + row * 4 + col); }
  OmVector4 row(int row) const { return OmVector4(mM + row * 4); }
  OmVector4 column(int col) const { return OmVector4(mM[col], mM[col + 4], mM[col + 8], mM[col + 12]); }
  OmVector3 translation() const { return OmVector3(mM[3], mM[7], mM[11]); }
  OmVector3 scale() const {
    return OmVector3(OmVector3(mM[0], mM[4], mM[8]).length(), OmVector3(mM[1], mM[5], mM[9]).length(),
                     OmVector3(mM[2], mM[6], mM[10]).length());
  }
  OmVector3 xAxis() const { return OmVector3(mM[0], mM[4], mM[8]); }
  OmVector3 yAxis() const { return OmVector3(mM[1], mM[5], mM[9]); }
  OmVector3 zAxis() const { return OmVector3(mM[2], mM[6], mM[10]); }

  QString toString(OmPrecision::Level level) const;

  // make identity
  void setIdentity();

  // transpose
  void transpose();
  OmMatrix4 transposed() const;

  // inverse
  bool inverse();
  void pseudoInverse();
  OmVector3 pseudoInversed(const OmVector3 &v) const;
  OmMatrix4 pseudoInversed() const;
  void inverseTransform(double sx, double sy, double sz);
  OmMatrix4 inversedTransform(double sx, double sy, double sz) const;
  OmMatrix4 inversedTransform(double scale = 0.0);
  void scale(double sx, double sy, double sz);

  // extracts the rotation submatrix
  OmMatrix3 extracted3x3Matrix() const;
  OmVector3 sub3x3MatrixDot(const OmVector3 &v) const;

  // rescale, extract and convert rotation submatrix as quaternion
  OmQuaternion extractedQuaternion(double x, double y, double z) const;
  OmQuaternion extractedQuaternion(double s = 1.0) const { return extractedQuaternion(s, s, s); }

  // extracts the rotation submatrix as a 3x4 (ODE compatible) matrix
  void extract3x4Matrix(double m[12]) const;
  void extract3x4Matrix(double m[12], double scale) const;

  // transpose and copy to/from OpenGL matrix (as in Webots 5)
  void fromOpenGlMatrix(const double m[16]);
  void toOpenGlMatrix(double m[16]) const;

  // assign from VRML representation
  void fromVrml(double tx, double ty, double tz,                // translation
                double rx, double ry, double rz, double angle,  // rotation
                double sx, double sy, double sz);               // scale
  void fromVrml(const OmVector3 &t, const OmRotation &r, const OmVector3 &s);

  // set the 3x3 rotation submatrix from euler axis-angle
  void setRotation(double rx, double ry, double rz, double angle);
  void setRotation(const OmRotation &r);

  // setters
  void setTranslation(double tx, double ty, double tz) {
    mM[3] = tx;
    mM[7] = ty;
    mM[11] = tz;
  }
  void setTranslation(const OmVector3 &trans) {
    mM[3] = trans.x();
    mM[7] = trans.y();
    mM[11] = trans.z();
  }
  void setScale(double sx, double sy, double sz) {
    mM[0] = sx;
    mM[5] = sy;
    mM[10] = sz;
  }
  void setScale(const OmVector3 &scale) {
    mM[0] = scale.x();
    mM[5] = scale.y();
    mM[10] = scale.z();
  }

private:
  double mM[16];
};

// This operation allows a direct multiplication by the transposed matrix avoiding thus an unecessary call to the transpose()
// method
inline OmVector4 operator*(const OmVector4 &v, const OmMatrix4 &m) {
  return OmVector4(v.x() * m.mM[0] + v.y() * m.mM[4] + v.z() * m.mM[8] + v.w() * m.mM[12],
                   v.x() * m.mM[1] + v.y() * m.mM[5] + v.z() * m.mM[9] + v.w() * m.mM[13],
                   v.x() * m.mM[2] + v.y() * m.mM[6] + v.z() * m.mM[10] + v.w() * m.mM[14],
                   v.x() * m.mM[3] + v.y() * m.mM[7] + v.z() * m.mM[11] + v.w() * m.mM[15]);
}

// This operation allows a direct multiplication by the transpose of the 3-by-3 "rotation" submatrix
inline OmVector3 operator*(const OmVector3 &v, const OmMatrix4 &m) {
  return OmVector3(v.x() * m.mM[0] + v.y() * m.mM[4] + v.z() * m.mM[8], v.x() * m.mM[1] + v.y() * m.mM[5] + v.z() * m.mM[9],
                   v.x() * m.mM[2] + v.y() * m.mM[6] + v.z() * m.mM[10]);
}

inline OmMatrix4 OmMatrix4::operator*(double s) const {
  return OmMatrix4(mM[0] * s, mM[1] * s, mM[2] * s, mM[3] * s, mM[4] * s, mM[5] * s, mM[6] * s, mM[7] * s, mM[8] * s, mM[9] * s,
                   mM[10] * s, mM[11] * s, mM[12] * s, mM[13] * s, mM[14] * s, mM[15] * s);
}

inline void OmMatrix4::scale(double sx, double sy, double sz) {
  mM[0] *= sx;
  mM[4] *= sx;
  mM[8] *= sx;
  mM[12] *= sx;

  mM[1] *= sy;
  mM[5] *= sy;
  mM[9] *= sy;
  mM[13] *= sy;

  mM[2] *= sz;
  mM[6] *= sz;
  mM[10] *= sz;
  mM[14] *= sz;
}

inline void OmMatrix4::fromVrml(double tx, double ty, double tz, double rx, double ry, double rz, double angle, double sx,
                                double sy, double sz) {
  const double c = cos(angle);
  const double s = sin(angle);

  const double t1 = 1.0 - c;
  const double t2 = rx * rz * t1;
  const double t3 = rx * ry * t1;
  const double t4 = ry * rz * t1;

  // translation
  mM[3] = tx;
  mM[7] = ty;
  mM[11] = tz;
  mM[12] = mM[13] = mM[14] = 0.0;
  mM[15] = 1.0;

  // rotate and scale (assuming the rotation vector is normalized)
  mM[0] = (rx * rx * t1 + c) * sx;
  mM[1] = (t3 - rz * s) * sy;
  mM[2] = (t2 + ry * s) * sz;
  mM[4] = (t3 + rz * s) * sx;
  mM[5] = (ry * ry * t1 + c) * sy;
  mM[6] = (t4 - rx * s) * sz;
  mM[8] = (t2 - ry * s) * sx;
  mM[9] = (t4 + rx * s) * sy;
  mM[10] = (rz * rz * t1 + c) * sz;
}

inline QString OmMatrix4::toString(OmPrecision::Level level = OmPrecision::Level::DOUBLE_MAX) const {
  QString result = "[\n";
  result += QString("  %1 %2 %3 %4\n")
              .arg(OmPrecision::doubleToString(mM[0], level))
              .arg(OmPrecision::doubleToString(mM[1], level))
              .arg(OmPrecision::doubleToString(mM[2], level))
              .arg(OmPrecision::doubleToString(mM[3], level));
  result += QString("  %1 %2 %3 %4\n")
              .arg(OmPrecision::doubleToString(mM[4], level))
              .arg(OmPrecision::doubleToString(mM[5], level))
              .arg(OmPrecision::doubleToString(mM[6], level))
              .arg(OmPrecision::doubleToString(mM[7], level));
  result += QString("  %1 %2 %3 %4\n")
              .arg(OmPrecision::doubleToString(mM[8], level))
              .arg(OmPrecision::doubleToString(mM[9], level))
              .arg(OmPrecision::doubleToString(mM[10], level))
              .arg(OmPrecision::doubleToString(mM[11], level));
  result += QString("  %1 %2 %3 %4\n")
              .arg(OmPrecision::doubleToString(mM[12], level))
              .arg(OmPrecision::doubleToString(mM[13], level))
              .arg(OmPrecision::doubleToString(mM[14], level))
              .arg(OmPrecision::doubleToString(mM[15], level));
  result += "]\n";
  return result;
}

#endif
