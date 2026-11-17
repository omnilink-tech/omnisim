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

// The differential-drive fold turns diff_drive_controller's per-wheel velocity
// commands back into the body twist OmniSim's mobile bridge accepts. It has to
// be an EXACT inverse of the controller's own forward kinematics, or every
// commanded speed is quietly scaled and every turn is quietly wrong -- the
// class of bug this repo guards hardest against, because the robot still moves
// and the number still looks plausible.

#include <gtest/gtest.h>

#include <cmath>

#include "omnisim_ros2_control/omnisim_system.hpp"

using omnisim_ros2_control::fold_diff_drive;

namespace
{

// diff_drive_controller (Humble, DiffDriveController::update):
//   velocity_left  = (linear - angular * wheel_separation / 2.0) / wheel_radius
//   velocity_right = (linear + angular * wheel_separation / 2.0) / wheel_radius
void controller_forward(
  double linear, double angular, double r, double b, double & left, double & right)
{
  left = (linear - angular * b / 2.0) / r;
  right = (linear + angular * b / 2.0) / r;
}

// The Husky as OmniSim's mobile bridge configures it (_mobile_configs.py:
// wheel_radius_m 0.1651, half_track_m 0.2854).
constexpr double kHuskyRadius = 0.1651;
constexpr double kHuskySeparation = 0.5708;  // 2 * half_track_m

}  // namespace

TEST(DiffDriveFold, RoundTripsTheControllerExactly)
{
  const double cases[][2] = {
    {0.0, 0.0}, {0.5, 0.0}, {0.0, 0.8}, {-0.35, 0.4}, {1.2, -1.5}, {0.05, 0.01},
  };
  for (const auto & c : cases) {
    double left = 0.0, right = 0.0;
    controller_forward(c[0], c[1], kHuskyRadius, kHuskySeparation, left, right);
    double linear = 0.0, angular = 0.0;
    fold_diff_drive(left, right, kHuskyRadius, kHuskySeparation, linear, angular);
    EXPECT_NEAR(linear, c[0], 1e-12) << "linear round trip";
    EXPECT_NEAR(angular, c[1], 1e-12) << "angular round trip";
  }
}

TEST(DiffDriveFold, SignConventionMatchesRepe103)
{
  // Right wheels faster than left => positive yaw rate (counter-clockwise seen
  // from above), which is REP-103. Getting this backwards is the classic
  // silent failure: the robot turns, just the wrong way.
  double linear = 0.0, angular = 0.0;
  fold_diff_drive(1.0, 2.0, kHuskyRadius, kHuskySeparation, linear, angular);
  EXPECT_GT(angular, 0.0);
  EXPECT_GT(linear, 0.0);

  fold_diff_drive(2.0, 1.0, kHuskyRadius, kHuskySeparation, linear, angular);
  EXPECT_LT(angular, 0.0);
}

TEST(DiffDriveFold, PureSpinHasNoForwardComponent)
{
  double linear = 0.0, angular = 0.0;
  fold_diff_drive(-3.0, 3.0, kHuskyRadius, kHuskySeparation, linear, angular);
  EXPECT_NEAR(linear, 0.0, 1e-15);
  EXPECT_NEAR(angular, 3.0 * 2.0 * kHuskyRadius / kHuskySeparation, 1e-12);
}

TEST(DiffDriveFold, ZeroSeparationCannotProduceInfiniteYaw)
{
  // A misconfigured (or unset) wheel_separation must not send inf to the
  // bridge. on_init rejects it outright; this pins the arithmetic too.
  double linear = 0.0, angular = 0.0;
  fold_diff_drive(-1.0, 1.0, kHuskyRadius, 0.0, linear, angular);
  EXPECT_TRUE(std::isfinite(angular));
  EXPECT_DOUBLE_EQ(angular, 0.0);
}
