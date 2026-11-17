// Copyright 2026 OmniLink
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

#ifndef OM_INERTIA_HPP
#define OM_INERTIA_HPP

//
// Description: ODE-free mass-property value type (the dMass replacement).
//
// Mirrors ODE's dMass semantics exactly so it can be parity-tested against it
// while ODE still ships, and survive src/ode deletion afterwards:
//   - mass, center of mass c, and inertia tensor I are all expressed about the
//     CURRENT REFERENCE POINT (the owning Solid's origin after composition),
//     with c pointing from that reference to the center of mass;
//   - translate() is dMassTranslate's two-reference parallel-axis form,
//     rotate() is R*I*R', add() merges two bodies sharing a reference point,
//     adjust() rescales mass and tensor together (dMassAdjust);
//   - setTrimesh() ports Mirtich's exact polyhedral algorithm from
//     ode/src/mass.cpp:213-411 (including the SF-1729095 translate).
//
// The formulas are transcribed from ode/src/mass.cpp with the same operation
// ordering so the dMass-oracle parity test can hold ~1e-12 on primitives.
//

class OmMatrix3;
class OmTriangleMesh;

class OmInertia {
public:
  OmInertia() { setZero(); }

  void setZero();
  // 1 kg + identity tensor -- ODE's setDefaultMass fallback.
  void setDefault() { setParameters(1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0); }
  void setParameters(double themass, double cgx, double cgy, double cgz, double i11, double i22, double i33, double i12,
                     double i13, double i23);

  // Primitive setters (density-based, tensor about the geometry frame origin).
  // Cylinder/capsule are z-axis (dir=3), matching OmSolidUtilities::addMass.
  void setSphere(double density, double radius);
  void setBox(double density, double lx, double ly, double lz);
  void setCylinderZ(double density, double radius, double height);
  void setCapsuleZ(double density, double radius, double height);
  // Mirtich exact polyhedral mass properties over the mesh's SCALED vertices
  // (falls back to raw vertices x (sx,sy,sz) if the scaled cache is empty).
  // Returns false when the result is unusable (open/degenerate mesh: mass <= 0
  // or non-positive-definite tensor) -- caller applies the default-mass
  // contract, exactly like the dMassSetTrimesh consumer.
  bool setTrimesh(double density, const OmTriangleMesh *mesh, double sx, double sy, double sz);

  // dMass-equivalent composition ops.
  void rotate(const OmMatrix3 &R);              // I <- R*I*R', c <- R*c
  void translate(double x, double y, double z); // parallel axis, two-reference form
  void add(const OmInertia &other);             // references must coincide
  void adjust(double newMass);                  // rescale mass + tensor

  double mass() const { return mMass; }
  double cx() const { return mC[0]; }
  double cy() const { return mC[1]; }
  double cz() const { return mC[2]; }
  double ixx() const { return mI[0]; }
  double iyy() const { return mI[4]; }
  double izz() const { return mI[8]; }
  double ixy() const { return mI[1]; }
  double ixz() const { return mI[2]; }
  double iyz() const { return mI[5]; }

  bool isPositiveDefinite() const;  // Sylvester on the 3x3 tensor

private:
  double mMass;
  double mC[3];
  double mI[9];  // row-major 3x3, kept symmetric by every op

  void symmetrize();
};

#endif
