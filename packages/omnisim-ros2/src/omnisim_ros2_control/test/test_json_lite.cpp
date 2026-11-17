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

// The JSON reader is the one place where a bug is INVISIBLE at runtime: a
// mis-parsed joint list looks exactly like a stationary robot, and a null
// velocity read as 0.0 looks exactly like a joint that has stopped.

#include <gtest/gtest.h>

#include <locale>
#include <string>

#include "omnisim_ros2_control/json_lite.hpp"

using omnisim_ros2_control::json::Type;
using omnisim_ros2_control::json::Value;
using omnisim_ros2_control::json::parse;

namespace
{
Value must_parse(const std::string & text)
{
  Value v;
  std::string err;
  EXPECT_TRUE(parse(text, v, err)) << text << " -> " << err;
  return v;
}
}  // namespace

TEST(JsonLite, ParsesARealHarnessJointsReply)
{
  // Captured verbatim from GET /robot/HUSKY/joints.
  const std::string body =
    R"({"robot": "HUSKY", "joints": [)"
    R"({"name": "front_left_wheel_motor", "type": "HingeJoint", "position": 4.84e-09, )"
    R"("velocity": -1.46e-10, "lower": 0.0, "upper": 0.0, "hit_limit": null}, )"
    R"({"name": "rear_right_wheel_motor", "type": "HingeJoint", "position": -7.19e-08, )"
    R"("velocity": null, "lower": -1.5, "upper": 1.5, "hit_limit": "lower"}]})";
  const Value doc = must_parse(body);
  ASSERT_EQ(doc["joints"].type, Type::Array);
  ASSERT_EQ(doc["joints"].array.size(), 2u);

  const Value & a = doc["joints"].array[0];
  EXPECT_EQ(a["name"].str(), "front_left_wheel_motor");
  EXPECT_DOUBLE_EQ(a["position"].num(-1.0), 4.84e-09);
  EXPECT_TRUE(a["velocity"].is_num());
  EXPECT_DOUBLE_EQ(a["velocity"].num(-1.0), -1.46e-10);
  EXPECT_TRUE(a["hit_limit"].is_null());

  const Value & b = doc["joints"].array[1];
  // THE POINT OF THIS TEST. `velocity: null` means "the harness has no sample
  // yet", never "this joint is not moving". is_num() must say so, and num()
  // must return the caller's fallback rather than 0.
  EXPECT_FALSE(b["velocity"].is_num());
  EXPECT_DOUBLE_EQ(b["velocity"].num(42.0), 42.0);
  EXPECT_DOUBLE_EQ(b["lower"].num(0.0), -1.5);
  EXPECT_EQ(b["hit_limit"].str(), "lower");
}

TEST(JsonLite, MissingKeysAreNullAndChainSafely)
{
  const Value doc = must_parse(R"({"a": {"b": 1}})");
  EXPECT_TRUE(doc["nope"].is_null());
  EXPECT_TRUE(doc["nope"]["deeper"].is_null());
  EXPECT_DOUBLE_EQ(doc["nope"]["deeper"].num(7.0), 7.0);
  EXPECT_DOUBLE_EQ(doc["a"]["b"].num(0.0), 1.0);
}

TEST(JsonLite, RejectsMalformedInput)
{
  Value v;
  std::string err;
  EXPECT_FALSE(parse("{\"a\": }", v, err));
  EXPECT_FALSE(parse("{\"a\": 1", v, err));
  EXPECT_FALSE(parse("{\"a\": 1} trailing", v, err));
  EXPECT_FALSE(parse("", v, err));
  EXPECT_FALSE(err.empty());
}

TEST(JsonLite, HandlesEscapesAndUnicode)
{
  const Value doc = must_parse(R"({"n": "a\"b\\c\ndé"})");
  EXPECT_EQ(doc["n"].str(), std::string("a\"b\\c\nd\xc3\xa9"));
}

TEST(JsonLite, NumbersAreLocaleIndependent)
{
  // ⚠ A comma-decimal locale is the failure this guards. `strtod` and
  // `ostream <<` follow the GLOBAL locale, so under de_DE a wheel radius of
  // 0.1651 parses as 0 and a 0.5 m/s command goes out as "0,5" -- which the
  // harness then rejects, or worse, reads as two arguments. from_chars /
  // to_chars are locale-independent by definition; this proves we use them.
  const char * candidates[] = {"de_DE.UTF-8", "fr_FR.UTF-8", "de_DE", "fr_FR"};
  bool installed = false;
  for (const char * name : candidates) {
    try {
      std::locale::global(std::locale(name));
      installed = true;
      break;
    } catch (const std::runtime_error &) {
      continue;
    }
  }
  const Value doc = must_parse(R"({"position": 0.1651, "velocity": -2.5e-3})");
  EXPECT_DOUBLE_EQ(doc["position"].num(0.0), 0.1651);
  EXPECT_DOUBLE_EQ(doc["velocity"].num(0.0), -2.5e-3);
  EXPECT_EQ(omnisim_ros2_control::json::number_to_string(0.5).find(','), std::string::npos);
  EXPECT_EQ(omnisim_ros2_control::json::number_to_string(0.5), "0.5");
  std::locale::global(std::locale::classic());
  if (!installed) {
    GTEST_SKIP() << "no comma-decimal locale installed; the classic-locale half still ran";
  }
}

TEST(JsonLite, NonFiniteCommandsNeverBecomeZero)
{
  // JSON has no inf/nan literal. Emitting 0 would execute a command nobody
  // sent; emitting a quoted token makes the far side reject it loudly.
  const std::string inf = omnisim_ros2_control::json::number_to_string(
    std::numeric_limits<double>::infinity());
  EXPECT_NE(inf.find('"'), std::string::npos);
  EXPECT_NE(inf, "0");
}
