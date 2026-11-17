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

#ifndef OM_AFFINE_PLANE_HPP
#define OM_AFFINE_PLANE_HPP

//
// Description: data structure for a 3D affine plane defined by the equation ax + by + cz = d.
//              Normal vector (a, b, c) is a unit vector.
//

#include <cmath>
#include "OmVector3.hpp"

class OmAffinePlane {
public:
  // construct an identity quaternion
  OmAffinePlane() : mA(1.0), mB(0.0), mC(0.0), mD(0.0) {}

  // construct from coefficients
  OmAffinePlane(const OmAffinePlane &other) : mA(other.mA), mB(other.mB), mC(other.mC), mD(other.mD) {}
  // construct from a normal vector and a scalar
  OmAffinePlane(const OmVector3 &v, double d) : mA(v.x()), mB(v.y()), mC(v.z()), mD(d) { normalize(); }
  // construct from a normal vector and a point
  OmAffinePlane(const OmVector3 &v, const OmVector3 &P) {
    mA = v.x(), mB = v.y(), mC = v.z(), mD = v.x() * P.x() + v.y() * P.y() + v.z() * P.z();
    normalize();
  }
  // construct from 3 points
  OmAffinePlane(const OmVector3 &P, const OmVector3 &Q, const OmVector3 &R) { from3Points(P, Q, R); }

  void from3Points(const OmVector3 &P, const OmVector3 &Q, const OmVector3 &R);

  // getters
  double a() const { return mA; }
  double b() const { return mB; }
  double c() const { return mC; }
  double d() const { return mD; }
  OmVector3 normal() const { return OmVector3(mA, mB, mC); }

  void redefine(const OmVector3 &v, const OmVector3 &P) {
    mA = v.x(), mB = v.y(), mC = v.z(), mD = v.x() * P.x() + v.y() * P.y() + v.z() * P.z();
    normalize();
  }

  // assignment: P1 = P2
  OmAffinePlane &operator=(const OmAffinePlane &p) {
    mA = p.mA;
    mB = p.mB;
    mC = p.mC;
    mD = p.mD;
    return *this;
  }

  // pseudo-distance to a 3D point
  // this is the signed distance from P to the plane,
  // the sign is positive if P lies in the upper plane determined by (a, b, c) and negative otherwise.
  double distance(const OmVector3 &v) const { return mA * v.x() + mB * v.y() + mC * v.z() - mD; }

  // project a vector on this plane, returns the project vector
  OmVector3 vectorProjection(const OmVector3 &v) const { return v + normal() * v.dot(normal()); }

private:
  double mA, mB, mC, mD;
  void normalize();
};

inline void OmAffinePlane::normalize() {
  double length = mA * mA + mB * mB + mC * mC;
  if (length == 0.0) {
    mA = 1.0;
    return;
  }
  length = 1.0 / sqrt(length);
  mA *= length;
  mB *= length;
  mC *= length;
  mD *= length;
}

#endif
