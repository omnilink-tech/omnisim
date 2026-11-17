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

#include "omnisim_ros2_control/omnisim_system.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "omnisim_ros2_control/http_client.hpp"
#include "omnisim_ros2_control/json_lite.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omnisim_ros2_control
{

namespace
{

double now_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// Locale-independent parse of a URDF `<param>` value. `std::stod` reads the
/// global locale's decimal point, so under a comma-decimal locale it would turn
/// a wheel radius of 0.1651 into 0.
bool to_double(const std::string & text, double & out)
{
  size_t begin = text.find_first_not_of(" \t");
  if (begin == std::string::npos) {return false;}
  size_t end = text.find_last_not_of(" \t") + 1;
  const auto res = std::from_chars(text.data() + begin, text.data() + end, out);
  return res.ec == std::errc() && res.ptr == text.data() + end;
}

std::string param(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

double param_double(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  double fallback, bool & bad)
{
  const auto it = params.find(key);
  if (it == params.end()) {return fallback;}
  double v = 0.0;
  if (!to_double(it->second, v)) {
    bad = true;
    return fallback;
  }
  return v;
}

std::string join(const std::vector<std::string> & items, const char * sep)
{
  std::string out;
  for (size_t i = 0; i < items.size(); ++i) {
    if (i != 0) {out += sep;}
    out += items[i];
  }
  return out;
}

}  // namespace

void fold_diff_drive(
  double left, double right, double wheel_radius, double wheel_separation,
  double & linear, double & angular)
{
  linear = wheel_radius * (left + right) * 0.5;
  angular = (wheel_separation > 0.0) ?
    wheel_radius * (right - left) / wheel_separation :
    0.0;
}

std::string servo_verb_path(const std::string & capabilities_body)
{
  json::Value doc;
  std::string error;
  if (!json::parse(capabilities_body, doc, error)) {return "";}
  // The arm bridge answers a JSON ARRAY of robot entries
  // ([{id, model, capabilities: {...}}]); tolerate a bare object too.
  std::vector<json::Value> entries;
  if (doc.type == json::Type::Array) {
    entries = doc.array;
  } else {
    entries.push_back(doc);
  }
  for (const auto & entry : entries) {
    // Accept either a wrapped entry or a bare capabilities dict.
    const json::Value & caps =
      entry["capabilities"].type == json::Type::Object ? entry["capabilities"] : entry;
    const std::string verb = caps["servo"]["verb"].str();
    if (!verb.empty()) {
      return verb[0] == '/' ? verb : "/" + verb;
    }
    // Fallback advertisement: the arm bridge listed the verb among its
    // busy-overriding actions before capabilities.servo existed.
    const json::Value & overriding = caps["busy_overriding_actions"];
    if (overriding.type == json::Type::Array) {
      for (const auto & action : overriding.array) {
        if (action.str() == "servo_joint_positions") {
          return "/servo_joint_positions";
        }
      }
    }
  }
  return "";
}

OmniSimSystem::~OmniSimSystem()
{
  running_ = false;
  if (comms_thread_.joinable()) {comms_thread_.join();}
}

hardware_interface::CallbackReturn OmniSimSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = rclcpp::get_logger("OmniSimSystem." + info_.name);
  // on_init can legitimately run twice (a cleanup/configure cycle), and the
  // joint table is built by appending. Without this, the second pass exports
  // every interface twice and controller_manager rejects the whole component
  // with a duplicate-interface error that names nothing useful.
  joints_.clear();

  bool bad = false;
  harness_url_ = param(info_.hardware_parameters, "harness_url", "http://127.0.0.1:6789");
  bridge_url_ = param(info_.hardware_parameters, "bridge_url", "http://127.0.0.1:8765");
  robot_def_ = param(info_.hardware_parameters, "robot_def", "");
  comms_rate_hz_ = param_double(info_.hardware_parameters, "comms_rate_hz", 25.0, bad);
  request_timeout_s_ = param_double(info_.hardware_parameters, "request_timeout_s", 2.0, bad);
  command_refresh_s_ = param_double(info_.hardware_parameters, "command_refresh_s", 0.25, bad);
  diagnostics_period_s_ =
    param_double(info_.hardware_parameters, "diagnostics_period_s", 10.0, bad);
  activate_timeout_s_ = param_double(info_.hardware_parameters, "activate_timeout_s", 10.0, bad);
  wheel_radius_ = param_double(info_.hardware_parameters, "wheel_radius", 0.0, bad);
  wheel_separation_ = param_double(info_.hardware_parameters, "wheel_separation", 0.0, bad);
  if (bad) {
    RCLCPP_ERROR(
      logger_,
      "a numeric <param> in the <hardware> block could not be parsed. Values must be plain "
      "decimals with a '.' separator (e.g. 0.1651).");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (robot_def_.empty()) {
    RCLCPP_ERROR(
      logger_,
      "<param name=\"robot_def\"> is required: it is the DEF name the harness knows this robot "
      "by (GET /robots lists them), and joint state is read from GET /robot/<def>/joints.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (comms_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(logger_, "comms_rate_hz must be > 0, got %g", comms_rate_hz_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::string mode = param(info_.hardware_parameters, "command_mode", "read_only");
  if (mode == "diff_drive") {
    mode_ = CommandMode::DiffDrive;
  } else if (mode == "joint_positions") {
    mode_ = CommandMode::JointPositions;
  } else if (mode == "read_only") {
    mode_ = CommandMode::ReadOnly;
  } else {
    RCLCPP_ERROR(
      logger_, "unknown command_mode '%s'; expected diff_drive, joint_positions or read_only",
      mode.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & j : info_.joints) {
    JointConfig cfg;
    cfg.ros_name = j.name;
    cfg.omnisim_name = param(j.parameters, "omnisim_joint", j.name);
    const std::string side = param(j.parameters, "omnisim_side", "");
    if (side == "left") {
      cfg.side = 0;
    } else if (side == "right") {
      cfg.side = 1;
    } else if (!side.empty()) {
      RCLCPP_ERROR(
        logger_, "joint '%s': omnisim_side must be 'left' or 'right', got '%s'",
        j.name.c_str(), side.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (const auto & ci : j.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        cfg.cmd_position = true;
      } else if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        cfg.cmd_velocity = true;
      } else {
        RCLCPP_ERROR(
          logger_,
          "joint '%s' asks for command interface '%s'. OmniSim's bridges accept position and "
          "velocity only -- there is no effort command path (see PROTOCOL.md 6.1/6.2).",
          j.name.c_str(), ci.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!j.state_interfaces.empty()) {
      cfg.state_position = false;
      cfg.state_velocity = false;
      for (const auto & si : j.state_interfaces) {
        if (si.name == hardware_interface::HW_IF_POSITION) {
          cfg.state_position = true;
        } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
          cfg.state_velocity = true;
        } else {
          RCLCPP_ERROR(
            logger_,
            "joint '%s' asks for state interface '%s'. OmniSim reports position and velocity "
            "only; effort is not exposed on any surface.",
            j.name.c_str(), si.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }
    joints_.push_back(std::move(cfg));
  }

  if (mode_ == CommandMode::DiffDrive) {
    if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0) {
      RCLCPP_ERROR(
        logger_,
        "command_mode=diff_drive needs <param name=\"wheel_radius\"> and "
        "<param name=\"wheel_separation\">. They MUST equal the values the controller uses, "
        "and the bridge's own wheel_radius_m / 2*half_track_m -- the twist sent to OmniSim is "
        "an exact inverse of the controller's wheel kinematics only when they agree.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool any_left = std::any_of(
      joints_.begin(), joints_.end(), [](const JointConfig & c) {return c.side == 0;});
    const bool any_right = std::any_of(
      joints_.begin(), joints_.end(), [](const JointConfig & c) {return c.side == 1;});
    if (!any_left || !any_right) {
      RCLCPP_ERROR(
        logger_,
        "command_mode=diff_drive needs at least one joint tagged "
        "<param name=\"omnisim_side\">left</param> and one tagged right.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const size_t n = joints_.size();
  state_position_.assign(n, 0.0);
  state_velocity_.assign(n, 0.0);
  command_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  command_velocity_.assign(n, 0.0);
  snap_position_.assign(n, 0.0);
  snap_velocity_.assign(n, 0.0);
  snap_lower_.assign(n, 0.0);
  snap_upper_.assign(n, 0.0);
  pending_position_.assign(n, std::numeric_limits<double>::quiet_NaN());
  pending_velocity_.assign(n, 0.0);
  last_sent_.assign(n, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(
    logger_,
    "OmniSim hardware '%s': %zu joints, robot_def=%s, state <- %s/robot/%s/joints, "
    "commands -> %s (mode=%s), comms_rate=%.1f Hz",
    info_.name.c_str(), n, robot_def_.c_str(), harness_url_.c_str(), robot_def_.c_str(),
    mode_ == CommandMode::ReadOnly ? "(none)" : bridge_url_.c_str(), mode.c_str(),
    comms_rate_hz_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniSimSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].state_position) {
      out.emplace_back(
        joints_[i].ros_name, hardware_interface::HW_IF_POSITION, &state_position_[i]);
    }
    if (joints_[i].state_velocity) {
      out.emplace_back(
        joints_[i].ros_name, hardware_interface::HW_IF_VELOCITY, &state_velocity_[i]);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> OmniSimSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (joints_[i].cmd_position) {
      out.emplace_back(
        joints_[i].ros_name, hardware_interface::HW_IF_POSITION, &command_position_[i]);
    }
    if (joints_[i].cmd_velocity) {
      out.emplace_back(
        joints_[i].ros_name, hardware_interface::HW_IF_VELOCITY, &command_velocity_[i]);
    }
  }
  return out;
}

hardware_interface::CallbackReturn OmniSimSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  running_ = true;
  comms_thread_ = std::thread(&OmniSimSystem::comms_loop, this);

  // Block until the first real measurement lands. Activating without it would
  // hand every controller a fabricated all-zero joint state, and
  // diff_drive_controller would integrate odometry from it -- a moving robot
  // reported as stationary is worse than a failed activation.
  {
    std::unique_lock<std::mutex> lock(mtx_);
    const bool got = first_snapshot_cv_.wait_for(
      lock, std::chrono::duration<double>(activate_timeout_s_),
      [this] {return snapshot_valid_ || !running_;});
    if (!got || !snapshot_valid_) {
      lock.unlock();
      running_ = false;
      if (comms_thread_.joinable()) {comms_thread_.join();}
      RCLCPP_ERROR(
        logger_,
        "no joint state from %s/robot/%s/joints within %.1f s. Is the harness up and a world "
        "loaded? (python -m omnisim harness --auto-port, then POST /world/load). Refusing to "
        "activate rather than reporting a fabricated zero state.",
        harness_url_.c_str(), robot_def_.c_str(), activate_timeout_s_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // THE UNLIMITED-MOTOR TRAP, checked against the harness's own report rather
  // than against the URDF. OmniSim builds a motor with no position limits as a
  // velocity wheel with ke = 0 and IGNORES setPosition() on it -- so a position
  // command interface on such a joint is not "slightly off", it does nothing at
  // all. Failing here is the whole point: falling back to velocity would make
  // the plugin report success for a command it silently discarded.
  std::vector<std::string> unlimited;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!joints_[i].cmd_position) {continue;}
      if (limits_known_ && snap_lower_[i] == 0.0 && snap_upper_[i] == 0.0) {
        unlimited.push_back(joints_[i].ros_name + " (" + joints_[i].omnisim_name + ")");
      }
    }
  }
  if (!unlimited.empty()) {
    running_ = false;
    if (comms_thread_.joinable()) {comms_thread_.join();}
    RCLCPP_ERROR(
      logger_,
      "these joints declare a POSITION command interface but OmniSim reports them as "
      "unconstrained (minStop == maxStop == 0): %s. OmniSim builds such a motor as a VELOCITY "
      "wheel with ke = 0 and ignores setPosition() on it by design "
      "(src/omnisim/nodes/OmBasicJoint.cpp), so those commands would be discarded with no "
      "error. Either declare minPosition/maxPosition on the motor in the world, or command "
      "these joints by velocity.",
      join(unlimited, ", ").c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start from the measured position so a position controller does not lurch on
  // its first update from a NaN or a zero.
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < joints_.size(); ++i) {
      state_position_[i] = snap_position_[i];
      state_velocity_[i] = snap_velocity_[i];
      if (joints_[i].cmd_position) {
        command_position_[i] = snap_position_[i];
        pending_position_[i] = snap_position_[i];
      }
    }
  }
  RCLCPP_INFO(logger_, "activated; joint state is live");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniSimSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the comms thread FIRST, then send the stop. The other order races: the
  // thread's next cycle would re-send the controller's last (non-zero) command
  // straight after our stop, leaving the robot driving with nothing left to
  // stop it. A simulator that keeps driving after its controller is deactivated
  // is the same hazard as one that keeps driving after the planner dies.
  running_ = false;
  if (comms_thread_.joinable()) {comms_thread_.join();}
  if (mode_ == CommandMode::DiffDrive) {
    HttpClient bridge;
    std::string error;
    if (bridge.configure(bridge_url_, request_timeout_s_, error)) {
      const HttpResponse resp = bridge.post("/set_velocity", "{\"linear\":0,\"angular\":0}");
      if (!resp.transport_ok() || !resp.ok()) {
        RCLCPP_WARN(
          logger_, "could not stop the robot on deactivate: %s",
          resp.transport_ok() ? resp.body.c_str() : resp.transport_error.c_str());
      }
    }
  }
  RCLCPP_INFO(logger_, "deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniSimSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // NO I/O HERE -- see the header. This is a mutex-guarded copy so it is safe at
  // any controller_manager update_rate.
  std::lock_guard<std::mutex> lock(mtx_);
  if (!snapshot_valid_) {return hardware_interface::return_type::OK;}
  for (size_t i = 0; i < joints_.size(); ++i) {
    state_position_[i] = snap_position_[i];
    state_velocity_[i] = snap_velocity_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmniSimSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (size_t i = 0; i < joints_.size(); ++i) {
    pending_position_[i] = command_position_[i];
    pending_velocity_[i] = command_velocity_[i];
  }
  return hardware_interface::return_type::OK;
}

void OmniSimSystem::comms_loop()
{
  HttpClient harness;
  HttpClient bridge;
  std::string error;
  // ⚠ Take the mutex before clearing running_ on the give-up paths. on_activate
  // waits on first_snapshot_cv_ with the predicate `snapshot_valid_ ||
  // !running_`; a notify that lands between its predicate check and its wait is
  // lost, and the activation then blocks for the whole activate_timeout_s over
  // a URL typo that was known instantly.
  const auto give_up = [this](const std::string & what, const std::string & why) {
      RCLCPP_ERROR(logger_, "%s: %s", what.c_str(), why.c_str());
      {
        std::lock_guard<std::mutex> lock(mtx_);
        running_ = false;
      }
      first_snapshot_cv_.notify_all();
    };
  if (!harness.configure(harness_url_, request_timeout_s_, error)) {
    give_up("harness_url", error);
    return;
  }
  if (mode_ != CommandMode::ReadOnly && !bridge.configure(bridge_url_, request_timeout_s_, error)) {
    give_up("bridge_url", error);
    return;
  }
  if (mode_ == CommandMode::JointPositions) {detect_joint_verb(bridge);}

  const double period = 1.0 / comms_rate_hz_;
  last_diag_at_ = now_s();
  while (running_) {
    const double t0 = now_s();
    poll_state(harness);
    if (mode_ != CommandMode::ReadOnly) {push_command(bridge);}
    ++cycles_;
    log_diagnostics(harness, bridge);

    const double elapsed = now_s() - t0;
    if (elapsed < period) {
      // Sleep in slices so a deactivate does not wait out a whole period.
      const double deadline = t0 + period;
      while (running_ && now_s() < deadline) {
        std::this_thread::sleep_for(
          std::chrono::duration<double>(std::min(0.02, deadline - now_s())));
      }
    }
  }
}

void OmniSimSystem::detect_joint_verb(HttpClient & bridge)
{
  // The bridge self-describes (PROTOCOL.md §6.1): an arm bridge with the
  // streaming lane advertises it in capabilities.servo. Read it once instead
  // of assuming -- older bridges and other robot classes do not have the verb,
  // and probing by POSTing it blind would put a motion command behind a 404.
  const HttpResponse resp = bridge.post("/capabilities", "{}");
  if (!resp.transport_ok() || !resp.ok()) {
    RCLCPP_WARN(
      logger_,
      "could not read the bridge's /capabilities (%s); joint commands will use the goal verb "
      "POST /set_joint_positions. ⚠ On that lane a setpoint arriving while the previous "
      "interpolation is still running answers 409 busy and is NOT applied, so a trajectory "
      "stream lands in pieces.",
      resp.transport_ok() ? ("HTTP " + std::to_string(resp.status)).c_str() :
      resp.transport_error.c_str());
    return;
  }
  const std::string path = servo_verb_path(resp.body);
  if (path.empty()) {
    RCLCPP_WARN(
      logger_,
      "this bridge does not advertise capabilities.servo; joint commands will use the goal verb "
      "POST /set_joint_positions. ⚠ That verb refuses a command while the previous one is still "
      "interpolating (409 busy, NOT applied), so a ros2_control position stream lands in pieces. "
      "Newer arm bridges have the streaming verb and are picked up automatically.");
    return;
  }
  joint_command_path_ = path;
  RCLCPP_INFO(
    logger_,
    "bridge advertises capabilities.servo; joint commands -> POST %s "
    "(non-blocking, last-write-wins, preempts goals -- a setpoint stream never answers 409)",
    joint_command_path_.c_str());
}

bool OmniSimSystem::poll_state(HttpClient & harness)
{
  const HttpResponse resp = harness.get("/robot/" + robot_def_ + "/joints");
  state_rtt_sum_ += resp.round_trip_s;
  ++state_rtt_n_;
  if (!resp.transport_ok() || !resp.ok()) {
    ++state_failures_;
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 10000,
      "joint state read failed (%s); holding the last measurement",
      resp.transport_ok() ? ("HTTP " + std::to_string(resp.status)).c_str() :
      resp.transport_error.c_str());
    return false;
  }

  json::Value doc;
  std::string parse_error;
  if (!json::parse(resp.body, doc, parse_error)) {
    ++state_failures_;
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 10000,
      "could not parse the joint state reply: %s", parse_error.c_str());
    return false;
  }

  const json::Value & list = doc["joints"];
  if (list.type != json::Type::Array) {
    ++state_failures_;
    return false;
  }
  std::map<std::string, const json::Value *> by_name;
  for (const auto & entry : list.array) {
    const std::string name = entry["name"].str();
    if (!name.empty()) {by_name[name] = &entry;}
  }

  std::vector<std::string> missing;
  bool any_null_velocity = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (size_t i = 0; i < joints_.size(); ++i) {
      const auto it = by_name.find(joints_[i].omnisim_name);
      if (it == by_name.end()) {
        missing.push_back(joints_[i].omnisim_name);
        continue;
      }
      const json::Value & j = *it->second;
      snap_position_[i] = j["position"].num(snap_position_[i]);
      // ⚠ null velocity is "not measured yet", NOT zero. The harness
      // finite-differences velocity between its own successive reads and
      // returns null on the first one. Holding the previous value is the least
      // wrong of the fixed-width options a state interface allows -- a
      // substituted 0.0 would read to every controller as "this joint has
      // stopped", which is a claim nothing measured.
      if (j["velocity"].is_num()) {
        snap_velocity_[i] = j["velocity"].number;
      } else {
        any_null_velocity = true;
      }
      snap_lower_[i] = j["lower"].num(0.0);
      snap_upper_[i] = j["upper"].num(0.0);
    }
    if (missing.empty()) {
      limits_known_ = true;
      if (!snapshot_valid_) {
        snapshot_valid_ = true;
        first_snapshot_cv_.notify_all();
      }
    }
  }

  if (!missing.empty() && !warned_missing_joint_) {
    warned_missing_joint_ = true;
    std::vector<std::string> reported;
    for (const auto & kv : by_name) {reported.push_back(kv.first);}
    RCLCPP_ERROR(
      logger_,
      "these joints are not in the harness's report for DEF '%s': %s. OmniSim names a joint "
      "after its FIRST MOTOR DEVICE, not after the URDF joint (a URDF joint 'front_left_wheel' "
      "is reported as 'front_left_wheel_motor'), so set "
      "<param name=\"omnisim_joint\">...</param> on each joint. The harness reports: %s",
      robot_def_.c_str(), join(missing, ", ").c_str(), join(reported, ", ").c_str());
  }
  if (any_null_velocity && !warned_null_velocity_) {
    warned_null_velocity_ = true;
    RCLCPP_INFO(
      logger_,
      "at least one joint velocity came back null (the harness finite-differences it and had no "
      "previous sample); holding the last measured value rather than substituting 0.0");
  }
  return missing.empty();
}

bool OmniSimSystem::push_command(HttpClient & bridge)
{
  std::string path;
  std::string body;
  std::vector<double> to_send;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mode_ == CommandMode::DiffDrive) {
      double left = 0.0, right = 0.0;
      size_t n_left = 0, n_right = 0;
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (joints_[i].side == 0) {left += pending_velocity_[i]; ++n_left;} else if (joints_[i].side == 1) {
          right += pending_velocity_[i];
          ++n_right;
        }
      }
      if (n_left == 0 || n_right == 0) {return false;}
      left /= static_cast<double>(n_left);
      right /= static_cast<double>(n_right);
      double linear = 0.0, angular = 0.0;
      fold_diff_drive(left, right, wheel_radius_, wheel_separation_, linear, angular);
      // DEBUG, but keep it: this line is how the fold was PROVEN exact against
      // a live diff_drive_controller (left=right=0.302847, r=0.1651, b=0.5708
      // -> linear=0.050000, angular=0.000000 for a commanded 0.05 m/s), and it
      // is the first thing to enable when a robot moves at the wrong speed.
      RCLCPP_DEBUG_THROTTLE(
        logger_, clock_, 2000,
        "fold: left=%.6f (n=%zu) right=%.6f (n=%zu) r=%.6f b=%.6f -> linear=%.6f angular=%.6f",
        left, n_left, right, n_right, wheel_radius_, wheel_separation_, linear, angular);
      to_send = {linear, angular};
      path = "/set_velocity";
      body = "{\"linear\":" + json::number_to_string(linear) + ",\"angular\":" +
        json::number_to_string(angular) + "}";
    } else {
      std::string q;
      for (size_t i = 0; i < joints_.size(); ++i) {
        if (!joints_[i].cmd_position) {continue;}
        const double v = pending_position_[i];
        if (std::isnan(v)) {return false;}  // controller has not written yet
        if (!q.empty()) {q += ",";}
        q += json::number_to_string(v);
        to_send.push_back(v);
      }
      if (to_send.empty()) {return false;}
      path = joint_command_path_;
      body = "{\"q\":[" + q + "]}";
    }
  }

  // Skip an unchanged command, but never for longer than command_refresh_s.
  // Both halves matter: a stationary robot must not spend a TCP connection
  // every cycle for a command the bridge already holds, and a bridge that was
  // restarted -- or preempted by another client -- must not be left holding a
  // stale one for ever.
  //
  // ⚠ THE UPPER BOUND IS NOT A STYLE CHOICE. The mobile bridge EXPIRES an
  // open-ended velocity command after VELOCITY_MAX_S = 12 s measured in
  // SIMULATED time (omnilink_mobile_bridge.py, the `velocity` branch of tick:
  // `robot.getTime() - t0_sim > VELOCITY_MAX_S`), and the harness runs worlds
  // at ~13x realtime -- so 12 simulated seconds is under ONE wall second. A
  // refresh period chosen against wall time therefore has to stay well inside
  // 12 / realtime_factor, or a constant-velocity command silently expires
  // between refreshes and the robot stutters. 0.25 s holds down to a 48x
  // realtime factor.
  const double t = now_s();
  bool identical = to_send.size() == last_sent_.size();
  for (size_t i = 0; identical && i < to_send.size(); ++i) {
    identical = std::abs(to_send[i] - last_sent_[i]) <= 1e-9;
  }
  if (identical && (t - last_command_sent_at_) < command_refresh_s_) {return true;}

  const HttpResponse resp = bridge.post(path, body);
  command_rtt_sum_ += resp.round_trip_s;
  ++command_rtt_n_;
  ++commands_sent_;
  if (!resp.transport_ok() || !resp.ok()) {
    ++command_failures_;
    if (resp.status == 409 && path == "/set_joint_positions") {
      // ⚠ NAME THE MECHANISM, because "409" on its own reads as a transient
      // hiccup and this one is structural. The GOAL verb refuses any external
      // motion verb while an interpolation is in flight and answers 409 with
      // `accepted: false` (omnilink_arm_bridge.py, BUSY_MOTION_KINDS), so a
      // trajectory controller streaming setpoints gets one accepted command
      // and a rejection for every one after it, until that motion ends --
      // which is not a rate problem and cannot be tuned away here. This
      // explainer applies to the FALLBACK lane only: on a bridge advertising
      // capabilities.servo the plugin streams to /servo_joint_positions,
      // where a 409 is a contract violation (handled below), not a property.
      RCLCPP_ERROR_THROTTLE(
        logger_, clock_, 10000,
        "the bridge answered 409 BUSY to POST %s. That is the goal-level API "
        "refusing a setpoint stream, not a transient error: the goal verb "
        "rejects any external motion verb while the previous one is still "
        "interpolating. This bridge did not advertise capabilities.servo (the "
        "non-blocking, superseding servo verb newer arm bridges have), so "
        "there is no streaming lane to fall forward to. Reply: %s",
        path.c_str(), resp.body.c_str());
      return false;
    }
    if (resp.status == 409) {
      // On the servo lane a 409 is NOT an expected condition: the contract is
      // last-write-wins, servo-on-servo retargets in place and an in-flight
      // goal is preempted. Seeing one means the bridge is not honouring its
      // own capabilities.servo advertisement.
      RCLCPP_WARN_THROTTLE(
        logger_, clock_, 10000,
        "the bridge answered 409 BUSY to POST %s -- unexpected: the servo lane "
        "is last-write-wins and never refuses a stream (PROTOCOL.md 6.1). This "
        "bridge advertised capabilities.servo and is not honouring it; check "
        "the bridge version. Reply: %s",
        path.c_str(), resp.body.c_str());
      return false;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 10000,
      "command POST %s rejected (%s); body was %s",
      path.c_str(),
      resp.transport_ok() ? ("HTTP " + std::to_string(resp.status) + ": " + resp.body).c_str() :
      resp.transport_error.c_str(),
      body.c_str());
    return false;
  }
  last_sent_ = to_send;
  last_command_sent_at_ = t;
  return true;
}

void OmniSimSystem::log_diagnostics(const HttpClient & harness, const HttpClient & bridge)
{
  const double t = now_s();
  if (diagnostics_period_s_ <= 0.0 || t - last_diag_at_ < diagnostics_period_s_) {return;}
  const double window = t - last_diag_at_;
  const uint64_t reqs = harness.requests_sent() + bridge.requests_sent();
  const uint64_t conns = harness.connections_opened() + bridge.connections_opened();

  std::ostringstream oss;
  oss.precision(3);
  oss << std::fixed;
  oss << "comms " << (static_cast<double>(cycles_) / window) << " Hz (target " << comms_rate_hz_
      << ")";
  if (state_rtt_n_ > 0) {
    oss << " | state RTT " << (1000.0 * state_rtt_sum_ / static_cast<double>(state_rtt_n_))
        << " ms";
  }
  if (command_rtt_n_ > 0) {
    oss << " | cmd RTT " << (1000.0 * command_rtt_sum_ / static_cast<double>(command_rtt_n_))
        << " ms";
  }
  // Report keep-alive PER SURFACE. The two are different servers and they do
  // not agree: a single "granted" would hide the fact that every state read is
  // still costing its own TCP connection while the command socket is reused.
  oss << " | " << reqs << " requests over " << conns << " TCP connections"
      << " (keep-alive: harness " << (harness.keep_alive_granted() ? "yes" : "NO")
      << ", bridge " << (mode_ == CommandMode::ReadOnly ? "n/a" :
    (bridge.keep_alive_granted() ? "yes" : "NO")) << ")";
  oss << " | " << commands_sent_ << " commands sent"
      << " | failures state=" << state_failures_ << " cmd=" << command_failures_;
  RCLCPP_INFO(logger_, "%s", oss.str().c_str());

  last_diag_at_ = t;
  cycles_ = 0;
  state_rtt_sum_ = 0.0;
  state_rtt_n_ = 0;
  command_rtt_sum_ = 0.0;
  command_rtt_n_ = 0;
}

}  // namespace omnisim_ros2_control

PLUGINLIB_EXPORT_CLASS(
  omnisim_ros2_control::OmniSimSystem, hardware_interface::SystemInterface)
