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

// Verb selection for joint commands is the piece that decides whether a
// ros2_control position stream lands on the streaming servo lane (never 409s)
// or on the goal lane (409-busy on every overlap). Getting it wrong is silent
// in the happy case -- both verbs accept the FIRST command -- so it is pinned
// here against the arm bridge's real /capabilities shape.

#include <gtest/gtest.h>

#include <string>

#include "omnisim_ros2_control/omnisim_system.hpp"

using omnisim_ros2_control::servo_verb_path;

TEST(ServoVerb, RealArmBridgeCapabilitiesShape)
{
  // Shape as omnilink_arm_bridge serves it: a JSON ARRAY of robot entries,
  // capabilities.servo.verb naming the streaming lane (PROTOCOL.md 6.1).
  const std::string body =
    R"([{"id": "ur5e", "model": "UR5e", "capabilities": {)"
    R"("joint_names": ["shoulder_pan_joint"], "has_gripper": true,)"
    R"("busy_rejecting_actions": ["pick", "set_joint_positions"],)"
    R"("busy_overriding_actions": ["stop_robot", "reset_to_home", "servo_joint_positions"],)"
    R"("servo": {"verb": "servo_joint_positions", "non_blocking": true,)"
    R"("last_write_wins": true, "done_tolerance_rad": 0.02}}}])";
  EXPECT_EQ(servo_verb_path(body), "/servo_joint_positions");
}

TEST(ServoVerb, BusyOverridingListAloneIsEnough)
{
  // A bridge that predates capabilities.servo but lists the verb among its
  // busy-overriding actions still advertises the lane.
  const std::string body =
    R"([{"id": "arm", "capabilities": {)"
    R"("busy_overriding_actions": ["stop_robot", "servo_joint_positions"]}}])";
  EXPECT_EQ(servo_verb_path(body), "/servo_joint_positions");
}

TEST(ServoVerb, MobileBridgeHasNoServo)
{
  // A mobile bridge's capabilities carry actions but no servo block; the
  // fallback (goal verb) must be selected, signalled by "".
  const std::string body =
    R"([{"id": "husky", "model": "Clearpath Husky", "capabilities": {)"
    R"("actions": [{"name": "set_velocity"}, {"name": "drive_forward"}]}}])";
  EXPECT_EQ(servo_verb_path(body), "");
}

TEST(ServoVerb, BareObjectAndBareCapabilitiesTolerated)
{
  EXPECT_EQ(
    servo_verb_path(R"({"capabilities": {"servo": {"verb": "servo_joint_positions"}}})"),
    "/servo_joint_positions");
  EXPECT_EQ(
    servo_verb_path(R"({"servo": {"verb": "servo_joint_positions"}})"),
    "/servo_joint_positions");
}

TEST(ServoVerb, MalformedOrEmptyMeansFallback)
{
  // Any failure to understand the reply must select the goal verb (a working
  // lane), never crash the comms thread and never invent a path.
  EXPECT_EQ(servo_verb_path(""), "");
  EXPECT_EQ(servo_verb_path("not json"), "");
  EXPECT_EQ(servo_verb_path("[]"), "");
  EXPECT_EQ(servo_verb_path(R"([{"capabilities": {"servo": {}}}])"), "");
  EXPECT_EQ(servo_verb_path(R"([{"capabilities": {"servo": {"verb": ""}}}])"), "");
  EXPECT_EQ(servo_verb_path(R"([{"capabilities": null}])"), "");
  EXPECT_EQ(servo_verb_path(R"(["nonsense", 5, null])"), "");
}

TEST(ServoVerb, LeadingSlashInAdvertisedVerbNotDoubled)
{
  EXPECT_EQ(
    servo_verb_path(R"([{"capabilities": {"servo": {"verb": "/servo_joint_positions"}}}])"),
    "/servo_joint_positions");
}
