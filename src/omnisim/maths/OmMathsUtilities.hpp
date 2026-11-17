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

#ifndef OM_MATHS_UTILITIES_HPP
#define OM_MATHS_UTILITIES_HPP

#include <math.h>

// Deliberately Qt-free (core-evolution-plan.md, Phase Q2): maths/ is future-core code.
#include <cassert>
#include <vector>

class OmPolygon;
class OmVector2;
class OmVector3;

namespace OmMathsUtilities {

  static const double EPSILON = 0.0001;

  enum { X, Y, Z };

  bool isPowerOf2(unsigned int n);
  unsigned int nextPowerOf2(unsigned int n);
  int round(double n);
  inline double discretize(double value, double resolution) {
    return ((int)(value / resolution + 0.5)) * resolution;
  }
  inline double discretize(float value, float resolution) {
    return ((int)(value / resolution + 0.5f)) * ((double)resolution);
  }

  // performs two Graham scan and returns the indices of points in the convex hull
  int twoStepsConvexHull(const std::vector<OmVector2> &points, std::vector<int> &hullIndices);
  int convexHull(const std::vector<OmVector2> &points, std::vector<int> &hullIndices);
  // builds a direct orthonormal basis (b[X], b[Y] = vY.normalized(), b[Z])
  void orthoBasis(const OmVector3 &vY, OmVector3 b[3]);
  bool isConvex(const OmPolygon &p);

  // Angles
  // find alpha such that |alpha - lastSpot| <= pi and alpha = angle mod 2 * pi
  inline double normalizeAngle(double angle, double lastSpot);
  bool isZeroAngle(double angle);
  // clamps angles in the -pi..pi range and possibly swap clamped angles to comply with min <= max.
  void clampAngles(double &min, double &max);
  // Vectors
  bool isZeroVector3(const double *v);
  void printVector3(const char *str, const double *v);
  void printMatrix3x4(const char *str, const double *m);
  // Find rational approximation of given real number
  // returns false if no approximation is found
  bool computeRationalApproximation(double value, int maxDenominator, int &numerator, int &denominator);
  // This will clamp value between -1 and 1 to avoid NaN generation
  inline double clampedAcos(double value);
  // This will clamp value between -1 and 1 to avoid indefinite value being generated
  inline double clampedAsin(double value);
};  // namespace OmMathsUtilities

// Normalize angle to be within +/-pi of lastSpot.
inline double OmMathsUtilities::normalizeAngle(double angle, double lastSpot = 0.0) {
  static const double INV_TWO_PI = 0.5 / M_PI;
  double d = angle - lastSpot;
  d -= floor(d * INV_TWO_PI) * 2.0 * M_PI;
  if (d > M_PI)
    d -= 2.0 * M_PI;
  return d + lastSpot;
}

// Make sure that the value is within the valid range before calling acos
// If not then the behavior is similar to a clamped value.
inline double OmMathsUtilities::clampedAcos(double value) {
  assert((fabs(value) < 1.0 + EPSILON) && "Value passed to clampedAcos out of range.");
  if (value >= 1.0)
    return 0.0;
  if (value <= -1.0)
    return M_PI;
  return acos(value);
}

// Make sure that the value is within the valid range before calling asin
// If not then the behavior is similar to a clamped value.
inline double OmMathsUtilities::clampedAsin(double value) {
  assert((fabs(value) < 1.0 + EPSILON) && "Value passed to clampedAsin out of range.");
  if (value >= 1.0)
    return M_PI / 2;
  if (value <= -1.0)
    return -M_PI / 2;
  return asin(value);
}

#endif  // OM_MATHS_UTILITIES_HPP
