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

#include "OmInertia.hpp"

#include "OmMatrix3.hpp"
#include "OmTriangleMesh.hpp"

#include <cmath>

// Every formula below is transcribed from ode/src/mass.cpp, keeping the
// operation ordering so the dMass-oracle parity test can hold ~1e-12.

void OmInertia::setZero() {
  mMass = 0.0;
  mC[0] = mC[1] = mC[2] = 0.0;
  for (int i = 0; i < 9; ++i)
    mI[i] = 0.0;
}

void OmInertia::setParameters(double themass, double cgx, double cgy, double cgz, double i11, double i22, double i33,
                              double i12, double i13, double i23) {
  setZero();
  mMass = themass;
  mC[0] = cgx;
  mC[1] = cgy;
  mC[2] = cgz;
  mI[0] = i11;
  mI[4] = i22;
  mI[8] = i33;
  mI[1] = i12;
  mI[2] = i13;
  mI[5] = i23;
  symmetrize();
}

void OmInertia::symmetrize() {
  mI[3] = mI[1];
  mI[6] = mI[2];
  mI[7] = mI[5];
}

void OmInertia::setSphere(double density, double radius) {
  // dMassSetSphere -> dMassSetSphereTotal (mass.cpp:114-133)
  setZero();
  mMass = (4.0 / 3.0) * M_PI * radius * radius * radius * density;
  const double II = 0.4 * mMass * radius * radius;
  mI[0] = II;
  mI[4] = II;
  mI[8] = II;
}

void OmInertia::setBox(double density, double lx, double ly, double lz) {
  // dMassSetBox -> dMassSetBoxTotal (mass.cpp:192-211)
  setZero();
  mMass = lx * ly * lz * density;
  mI[0] = mMass / 12.0 * (ly * ly + lz * lz);
  mI[4] = mMass / 12.0 * (lx * lx + lz * lz);
  mI[8] = mMass / 12.0 * (lx * lx + ly * ly);
}

void OmInertia::setCylinderZ(double density, double radius, double height) {
  // dMassSetCylinder(dir=3) -> dMassSetCylinderTotal (mass.cpp:165-190)
  setZero();
  const double r2 = radius * radius;
  mMass = M_PI * r2 * height * density;
  const double I = mMass * (0.25 * r2 + (1.0 / 12.0) * height * height);
  mI[0] = I;
  mI[4] = I;
  mI[8] = mMass * 0.5 * r2;
}

void OmInertia::setCapsuleZ(double density, double radius, double height) {
  // dMassSetCapsule(dir=3) (mass.cpp:135-156)
  setZero();
  const double M1 = M_PI * radius * radius * height * density;                  // cylinder
  const double M2 = (4.0 / 3.0) * M_PI * radius * radius * radius * density;    // both caps
  mMass = M1 + M2;
  const double Ia = M1 * (0.25 * radius * radius + (1.0 / 12.0) * height * height) +
                    M2 * (0.4 * radius * radius + 0.375 * radius * height + 0.25 * height * height);
  const double Ib = (M1 * 0.5 + M2 * 0.4) * radius * radius;
  mI[0] = Ia;
  mI[4] = Ia;
  mI[8] = Ib;
}

void OmInertia::rotate(const OmMatrix3 &R) {
  // dMassRotate (mass.cpp:475-506): I <- R*I*R', c <- R*c, same multiply order
  // (t1 = I*R' via dMultiply2_333, then I = R*t1 via dMultiply0_333).
  double t1[9];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      // t1(i,j) = row i of I . row j of R   (I * R')
      t1[i * 3 + j] = mI[i * 3 + 0] * R(j, 0) + mI[i * 3 + 1] * R(j, 1) + mI[i * 3 + 2] * R(j, 2);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      // I(i,j) = row i of R . column j of t1   (R * t1)
      mI[i * 3 + j] = R(i, 0) * t1[0 * 3 + j] + R(i, 1) * t1[1 * 3 + j] + R(i, 2) * t1[2 * 3 + j];
  // ensure perfect symmetry (copy the upper triangle down, as ODE does)
  mI[3] = mI[1];
  mI[6] = mI[2];
  mI[7] = mI[5];
  const double c0 = R(0, 0) * mC[0] + R(0, 1) * mC[1] + R(0, 2) * mC[2];
  const double c1 = R(1, 0) * mC[0] + R(1, 1) * mC[1] + R(1, 2) * mC[2];
  const double c2 = R(2, 0) * mC[0] + R(2, 1) * mC[1] + R(2, 2) * mC[2];
  mC[0] = c0;
  mC[1] = c1;
  mC[2] = c2;
}

static void crossMatrixSquared(const double v[3], double out[9]) {
  // out = crossmat(v)^2, computed as the product of the skew matrix with
  // itself (dSetCrossMatrixPlus followed by dMultiply0_333 in dMassTranslate).
  const double s[9] = {0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out[i * 3 + j] = s[i * 3 + 0] * s[0 * 3 + j] + s[i * 3 + 1] * s[1 * 3 + j] + s[i * 3 + 2] * s[2 * 3 + j];
}

void OmInertia::translate(double x, double y, double z) {
  // dMassTranslate (mass.cpp:432-473): I += m*(chat^2 - ahat^2), a = c + t
  const double a[3] = {x + mC[0], y + mC[1], z + mC[2]};
  double ahat2[9], chat2[9];
  crossMatrixSquared(a, ahat2);
  crossMatrixSquared(mC, chat2);
  for (int i = 0; i < 9; ++i)
    mI[i] += mMass * (chat2[i] - ahat2[i]);
  mI[3] = mI[1];
  mI[6] = mI[2];
  mI[7] = mI[5];
  mC[0] += x;
  mC[1] += y;
  mC[2] += z;
}

void OmInertia::add(const OmInertia &other) {
  // dMassAdd (mass.cpp:508-517)
  const double denom = 1.0 / (mMass + other.mMass);
  for (int i = 0; i < 3; ++i)
    mC[i] = (mC[i] * mMass + other.mC[i] * other.mMass) * denom;
  mMass += other.mMass;
  for (int i = 0; i < 9; ++i)
    mI[i] += other.mI[i];
}

void OmInertia::adjust(double newMass) {
  // dMassAdjust (mass.cpp:421-431)
  const double scale = newMass / mMass;
  mMass = newMass;
  for (int i = 0; i < 9; ++i)
    mI[i] *= scale;
}

bool OmInertia::isPositiveDefinite() const {
  // Sylvester's criterion on the symmetric 3x3 tensor -- same verdict as
  // ODE's dIsPositiveDefinite (Cholesky) for the well/ill-conditioned cases
  // this gate distinguishes (open meshes give zero/negative eigenvalues).
  const double d1 = mI[0];
  const double d2 = mI[0] * mI[4] - mI[1] * mI[3];
  const double d3 = mI[0] * (mI[4] * mI[8] - mI[5] * mI[7]) - mI[1] * (mI[3] * mI[8] - mI[5] * mI[6]) +
                    mI[2] * (mI[3] * mI[7] - mI[4] * mI[6]);
  return d1 > 0.0 && d2 > 0.0 && d3 > 0.0;
}

#define SQR(x) ((x) * (x))
#define CUBE(x) ((x) * (x) * (x))

bool OmInertia::setTrimesh(double density, const OmTriangleMesh *mesh, double sx, double sy, double sz) {
  // dMassSetTrimesh (mass.cpp:213-411) -- Mirtich, "Fast and Accurate
  // Computation of Polyhedral Mass Properties", JGT 1(2), 1996. Identical
  // control flow and accumulation order; the only difference is the triangle
  // source (OmTriangleMesh scaled vertices instead of the ODE trimesh geom,
  // which was built from the very same scaled-coordinate array).
  setZero();

  const int triangles = mesh->numberOfTriangles();
  const bool useScaledCache = !mesh->areScaledCoordinatesEmpty();
  const double scale[3] = {sx, sy, sz};

  double nx, ny, nz;
  unsigned int A, B, C;
  // face integrals
  double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;
  // projection integrals
  double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

  double T0 = 0;
  double T1[3] = {0., 0., 0.};
  double T2[3] = {0., 0., 0.};
  double TP[3] = {0., 0., 0.};

  for (int i = 0; i < triangles; i++) {
    double v[3][3];
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        v[j][k] = useScaledCache ? mesh->scaledVertex(i, j, k) : mesh->vertex(i, j, k) * scale[k];

    // n = (v2-v0) x (v1-v0), exactly dCalcVectorCross3(n, b, a)
    double a[3], b[3], n[3];
    for (int k = 0; k < 3; ++k) {
      a[k] = v[1][k] - v[0][k];
      b[k] = v[2][k] - v[0][k];
    }
    n[0] = b[1] * a[2] - b[2] * a[1];
    n[1] = b[2] * a[0] - b[0] * a[2];
    n[2] = b[0] * a[1] - b[1] * a[0];
    nx = fabs(n[0]);
    ny = fabs(n[1]);
    nz = fabs(n[2]);

    if (nx > ny && nx > nz)
      C = 0;
    else
      C = (ny > nz) ? 1 : 2;

    // a triangle may degenerate into a segment after scaling
    if (n[C] != 0.0) {
      A = (C + 1) % 3;
      B = (A + 1) % 3;

      // face integrals
      {
        double w;
        double k1, k2, k3, k4;

        // projection integrals
        {
          double a0 = 0, a1 = 0, da;
          double b0 = 0, b1 = 0, db;
          double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
          double a1_2, a1_3, b1_2, b1_3;
          double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
          double Cab, Kab, Caab, Kaab, Cabb, Kabb;

          P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

          for (int j = 0; j < 3; j++) {
            switch (j) {
              case 0:
                a0 = v[0][A];
                b0 = v[0][B];
                a1 = v[1][A];
                b1 = v[1][B];
                break;
              case 1:
                a0 = v[1][A];
                b0 = v[1][B];
                a1 = v[2][A];
                b1 = v[2][B];
                break;
              case 2:
                a0 = v[2][A];
                b0 = v[2][B];
                a1 = v[0][A];
                b1 = v[0][B];
                break;
            }
            da = a1 - a0;
            db = b1 - b0;
            a0_2 = a0 * a0;
            a0_3 = a0_2 * a0;
            a0_4 = a0_3 * a0;
            b0_2 = b0 * b0;
            b0_3 = b0_2 * b0;
            b0_4 = b0_3 * b0;
            a1_2 = a1 * a1;
            a1_3 = a1_2 * a1;
            b1_2 = b1 * b1;
            b1_3 = b1_2 * b1;

            C1 = a1 + a0;
            Ca = a1 * C1 + a0_2;
            Caa = a1 * Ca + a0_3;
            Caaa = a1 * Caa + a0_4;
            Cb = b1 * (b1 + b0) + b0_2;
            Cbb = b1 * Cb + b0_3;
            Cbbb = b1 * Cbb + b0_4;
            Cab = 3 * a1_2 + 2 * a1 * a0 + a0_2;
            Kab = a1_2 + 2 * a1 * a0 + 3 * a0_2;
            Caab = a0 * Cab + 4 * a1_3;
            Kaab = a1 * Kab + 4 * a0_3;
            Cabb = 4 * b1_3 + 3 * b1_2 * b0 + 2 * b1 * b0_2 + b0_3;
            Kabb = b1_3 + 2 * b1_2 * b0 + 3 * b1 * b0_2 + 4 * b0_3;

            P1 += db * C1;
            Pa += db * Ca;
            Paa += db * Caa;
            Paaa += db * Caaa;
            Pb += da * Cb;
            Pbb += da * Cbb;
            Pbbb += da * Cbbb;
            Pab += db * (b1 * Cab + b0 * Kab);
            Paab += db * (b1 * Caab + b0 * Kaab);
            Pabb += da * (a1 * Cabb + a0 * Kabb);
          }

          P1 /= 2.0;
          Pa /= 6.0;
          Paa /= 12.0;
          Paaa /= 20.0;
          Pb /= -6.0;
          Pbb /= -12.0;
          Pbbb /= -20.0;
          Pab /= 24.0;
          Paab /= 60.0;
          Pabb /= -60.0;
        }

        w = -(n[0] * v[0][0] + n[1] * v[0][1] + n[2] * v[0][2]);

        k1 = 1 / n[C];
        k2 = k1 * k1;
        k3 = k2 * k1;
        k4 = k3 * k1;

        Fa = k1 * Pa;
        Fb = k1 * Pb;
        Fc = -k2 * (n[A] * Pa + n[B] * Pb + w * P1);

        Faa = k1 * Paa;
        Fbb = k1 * Pbb;
        Fcc = k3 * (SQR(n[A]) * Paa + 2 * n[A] * n[B] * Pab + SQR(n[B]) * Pbb +
                    w * (2 * (n[A] * Pa + n[B] * Pb) + w * P1));

        Faaa = k1 * Paaa;
        Fbbb = k1 * Pbbb;
        Fccc = -k4 * (CUBE(n[A]) * Paaa + 3 * SQR(n[A]) * n[B] * Paab + 3 * n[A] * SQR(n[B]) * Pabb + CUBE(n[B]) * Pbbb +
                      3 * w * (SQR(n[A]) * Paa + 2 * n[A] * n[B] * Pab + SQR(n[B]) * Pbb) +
                      w * w * (3 * (n[A] * Pa + n[B] * Pb) + w * P1));

        Faab = k1 * Paab;
        Fbbc = -k2 * (n[A] * Pabb + n[B] * Pbbb + w * Pbb);
        Fcca = k3 * (SQR(n[A]) * Paaa + 2 * n[A] * n[B] * Paab + SQR(n[B]) * Pabb + w * (2 * (n[A] * Paa + n[B] * Pab) + w * Pa));
      }

      T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

      T1[A] += n[A] * Faa;
      T1[B] += n[B] * Fbb;
      T1[C] += n[C] * Fcc;
      T2[A] += n[A] * Faaa;
      T2[B] += n[B] * Fbbb;
      T2[C] += n[C] * Fccc;
      TP[A] += n[A] * Faab;
      TP[B] += n[B] * Fbbc;
      TP[C] += n[C] * Fcca;
    }
  }

  T1[0] /= 2;
  T1[1] /= 2;
  T1[2] /= 2;
  T2[0] /= 3;
  T2[1] /= 3;
  T2[2] /= 3;
  TP[0] /= 2;
  TP[1] /= 2;
  TP[2] /= 2;

  mMass = density * T0;
  mI[0] = density * (T2[1] + T2[2]);
  mI[4] = density * (T2[2] + T2[0]);
  mI[8] = density * (T2[0] + T2[1]);
  mI[1] = -density * TP[0];
  mI[3] = -density * TP[0];
  mI[7] = -density * TP[1];
  mI[5] = -density * TP[1];
  mI[6] = -density * TP[2];
  mI[2] = -density * TP[2];

  if (T0 == 0.0 || mMass <= 0.0)
    return false;

  // SF bug 1729095: shift the reference so I is about the origin with c = COM
  translate(T1[0] / T0, T1[1] / T0, T1[2] / T0);

  return isPositiveDefinite();
}

#undef SQR
#undef CUBE
