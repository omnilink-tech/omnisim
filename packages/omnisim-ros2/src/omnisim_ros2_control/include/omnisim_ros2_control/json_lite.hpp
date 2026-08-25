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

// A minimal, dependency-free JSON reader for the handful of OmniSim payloads
// this plugin consumes.
//
// WHY NOT nlohmann/json: it is not a ROS 2 Humble dependency and is absent from
// a stock `ros-humble-desktop` install, so depending on it would turn "colcon
// build" into "colcon build after an apt install" for every user. The payloads
// we parse are small and regular (`{"joints":[{"name":..,"position":..}]}`,
// `{"x":..,"y":..,"yaw":..}`), so a ~200-line reader is a better trade than a
// dependency that has to be documented, pinned and installed.
//
// ⚠ LOCALE. `strtod` and `ostream <<` use the *global* locale's decimal point,
// and a ROS node inherits whatever the process set. Under a comma-decimal locale
// (de_DE, fr_FR, …) that silently turns 0.5 into "0,5" on the wire and parses
// "0.5" as 0. Every number in and out of this file therefore goes through
// std::from_chars / std::to_chars, which are locale-INDEPENDENT by definition.

#ifndef OMNISIM_ROS2_CONTROL__JSON_LITE_HPP_
#define OMNISIM_ROS2_CONTROL__JSON_LITE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace omnisim_ros2_control
{
namespace json
{

enum class Type { Null, Bool, Number, String, Array, Object };

/// One JSON value. Copyable; children are held by value, which is fine at the
/// payload sizes involved (tens of nodes, not megabytes).
class Value
{
public:
  Type type = Type::Null;
  bool boolean = false;
  double number = 0.0;
  std::string string;
  std::vector<Value> array;
  std::map<std::string, Value> object;

  bool is_null() const {return type == Type::Null;}

  /// Member lookup on an object; returns a static null Value when absent, so
  /// `v["a"]["b"]` never throws and never needs a null check per level.
  const Value & operator[](const std::string & key) const;

  /// Number, or `fallback` when this value is not a number (including JSON
  /// null). The caller decides what a missing measurement means -- this class
  /// never substitutes 0 silently.
  double num(double fallback) const {return type == Type::Number ? number : fallback;}

  /// True only when this value really is a number. `null` is what OmniSim sends
  /// for an unmeasured joint velocity, and it must not read as 0.0.
  bool is_num() const {return type == Type::Number;}

  std::string str(const std::string & fallback = "") const
  {
    return type == Type::String ? string : fallback;
  }
};

/// Parse `text`. Returns false and fills `error` on malformed input; the
/// returned Value is then unspecified.
bool parse(const std::string & text, Value & out, std::string & error);

/// Locale-independent number formatting for request bodies.
std::string number_to_string(double v);

/// JSON string escaping, for the one place we send a name rather than a number.
std::string escape(const std::string & s);

}  // namespace json
}  // namespace omnisim_ros2_control

#endif  // OMNISIM_ROS2_CONTROL__JSON_LITE_HPP_
