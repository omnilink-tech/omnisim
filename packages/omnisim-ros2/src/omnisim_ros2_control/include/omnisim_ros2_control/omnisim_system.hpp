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

// `ros2_control` hardware interface for OmniSim (Tier 3).
//
// WHAT IT IS
// ----------
// A `hardware_interface::SystemInterface` that presents an OmniSim robot to
// `controller_manager` as ordinary hardware, so stock controllers
// (`diff_drive_controller`, `joint_trajectory_controller`,
// `joint_state_broadcaster`, …) — and therefore Nav2 and MoveIt — drive it
// without knowing OmniSim exists.
//
// TWO HTTP SURFACES, FOR THE SAME STRUCTURAL REASON AS TIER 2
// -----------------------------------------------------------
//   * **State** comes from the World Harness: `GET /robot/<def>/joints`, which
//     reports every joint's live position, the harness's finite-differenced
//     velocity, and its stops.
//   * **Commands** go to the robot's own **bridge**: `POST /set_velocity`, or
//     for joint commands the streaming `POST /servo_joint_positions` when the
//     bridge advertises it in `capabilities.servo`, else the goal verb
//     `POST /set_joint_positions`. The harness's supervisor can teleport a body
//     but cannot drive a motor belonging to another robot — OmniSim restricts
//     device APIs to the owning controller, which is the same reason
//     `GET /robot/<def>/sensor/<name>` is a 501 by design.
//
// Neither endpoint is new. This plugin adds no OmniSim-side surface at all.
//
// ⚠ THE CONTROL RATE AND THE TRANSPORT RATE ARE DIFFERENT NUMBERS
// ----------------------------------------------------------------
// `controller_manager`'s `update_rate` is typically 100–1000 Hz. OmniSim's HTTP
// surfaces are Python `BaseHTTPRequestHandler` servers that speak **HTTP/1.0**,
// so **every request costs a fresh TCP connection** — keep-alive is not
// available to ask for (`HttpClient` asks anyway and records the refusal). At
// 1 kHz with two requests per cycle that is 2000 connections/second, which
// exhausts an ephemeral-port range in seconds.
//
// So `read()` and `write()` do **no I/O**. They exchange values with a snapshot
// under a mutex — sub-microsecond, safe at any `update_rate`. A single
// background thread owns the HTTP at its own `comms_rate_hz`, exactly the way a
// ros2_control driver for a slow fieldbus is written.
//
// The consequence is honest and must be stated wherever this is quoted:
// **the actuation and sensing bandwidth is `comms_rate_hz`, not `update_rate`.**
// Running the controller manager at 100 Hz over a 20 Hz link does not give you
// 100 Hz control; it gives you a 100 Hz controller reading a 20 Hz sensor. The
// plugin logs its measured comms rate, per-request round trip and TCP
// connections-per-request every `diagnostics_period_s` so the real number is
// visible rather than assumed.
//
// ⚠ POSITION COMMANDS ARE SILENTLY IGNORED ON AN UNLIMITED OMNISIM MOTOR
// -----------------------------------------------------------------------
// OmniSim builds a motor with no `minPosition`/`maxPosition` (and no joint
// `minStop`/`maxStop`) as a **velocity wheel with `ke = 0`**, and `setPosition()`
// on it is ignored by design. A Husky's wheels are exactly that. This plugin
// detects the case from the harness's own `lower`/`upper` report and refuses to
// activate a position command interface on such a joint, naming it — rather than
// quietly falling back to velocity control and reporting success.

#ifndef OMNISIM_ROS2_CONTROL__OMNISIM_SYSTEM_HPP_
#define OMNISIM_ROS2_CONTROL__OMNISIM_SYSTEM_HPP_

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace omnisim_ros2_control
{

/// How a joint's velocity command reaches OmniSim.
enum class CommandMode
{
  /// Fold per-wheel velocity commands into one body twist and send it to the
  /// mobile bridge's `POST /set_velocity`. See `fold_diff_drive`.
  DiffDrive,
  /// Send the joint position vector to an arm bridge. The verb is chosen from
  /// the bridge's own `/capabilities`: the streaming
  /// `POST /servo_joint_positions` (non-blocking, last-write-wins, never 409s
  /// for a stream) when advertised, else the goal verb
  /// `POST /set_joint_positions`, whose 409-busy-on-overlap contract makes it
  /// unfit for a setpoint stream. See `servo_verb_path`.
  JointPositions,
  /// Read state, command nothing. Useful to bring `joint_state_broadcaster` up
  /// against a robot whose bridge is not running.
  ReadOnly,
};

/// Inverse of the standard differential-drive forward kinematics.
///
/// `diff_drive_controller` computes, for a commanded body twist (v, ω):
///
///     ω_left  = (v - ω·b/2) / r
///     ω_right = (v + ω·b/2) / r
///
/// and OmniSim's mobile bridge computes exactly the same thing from the twist it
/// is given (`omnilink_mobile_bridge.py`: `v_left = (linear - angular*ht)/r`,
/// with `ht = b/2`). So folding the controller's wheel commands back into a
/// twist is an **exact algebraic inverse**, not an approximation — provided the
/// controller's `wheel_radius` / `wheel_separation` match the bridge's
/// `wheel_radius_m` / `2 × half_track_m`. When they do, the twist that reaches
/// the bridge is bit-for-bit the twist `diff_drive_controller` was given.
///
/// `left`/`right` are mean wheel speeds in rad/s for their side; the output is
/// (linear m/s, angular rad/s) in the robot's body frame.
void fold_diff_drive(
  double left, double right, double wheel_radius, double wheel_separation,
  double & linear, double & angular);

/// Parse a bridge's `POST /capabilities` reply body and return the request
/// path of the streaming setpoint verb it advertises (PROTOCOL.md §6.1
/// `capabilities.servo`, e.g. `"/servo_joint_positions"`), or `""` when it
/// advertises none — older bridges and non-arm bridges. Tolerates both reply
/// shapes (the JSON array the arm bridge serves, or a bare object) and also
/// recognises `"servo_joint_positions"` in `busy_overriding_actions`, which
/// the arm bridge published first. Free function so it is unit-testable
/// without a bridge.
std::string servo_verb_path(const std::string & capabilities_body);

class OmniSimSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmniSimSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~OmniSimSystem() override;

private:
  struct JointConfig
  {
    std::string ros_name;      ///< as it appears in the URDF and to controllers
    std::string omnisim_name;  ///< as `GET /robot/<def>/joints` reports it
    int side = -1;             ///< 0 = left, 1 = right, -1 = not a drive wheel
    bool cmd_position = false;
    bool cmd_velocity = false;
    bool state_position = true;
    bool state_velocity = true;
  };

  void comms_loop();
  /// One `POST /capabilities` against the bridge to pick the joint-command
  /// verb (comms thread, before the first push). Falls back to the goal verb
  /// on any failure — deterministic, never blocks the loop.
  void detect_joint_verb(class HttpClient & bridge);
  /// One `GET /robot/<def>/joints` -> snapshot. Returns false on any failure.
  bool poll_state(class HttpClient & harness);
  /// One command push, or none when the command is unchanged and still fresh.
  bool push_command(class HttpClient & bridge);
  void log_diagnostics(const class HttpClient & harness, const class HttpClient & bridge);

  rclcpp::Logger logger_ = rclcpp::get_logger("OmniSimSystem");
  // Steady clock: the throttled warnings below must keep working when the ROS
  // clock is simulation time, which under `--mode=fast` runs ~13x realtime.
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  std::vector<JointConfig> joints_;
  std::string harness_url_;
  std::string bridge_url_;
  std::string robot_def_;
  CommandMode mode_ = CommandMode::ReadOnly;
  double wheel_radius_ = 0.0;
  double wheel_separation_ = 0.0;
  double comms_rate_hz_ = 25.0;
  double request_timeout_s_ = 2.0;
  double command_refresh_s_ = 0.25;
  double diagnostics_period_s_ = 10.0;
  double activate_timeout_s_ = 10.0;

  // Exported to controllers. Touched only by read()/write() on the CM thread.
  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;

  // Shared with the comms thread.
  mutable std::mutex mtx_;
  std::condition_variable first_snapshot_cv_;
  std::vector<double> snap_position_;
  std::vector<double> snap_velocity_;
  std::vector<double> snap_lower_;
  std::vector<double> snap_upper_;
  std::vector<double> pending_position_;
  std::vector<double> pending_velocity_;
  bool snapshot_valid_ = false;
  bool limits_known_ = false;

  std::thread comms_thread_;
  std::atomic<bool> running_{false};

  // Diagnostics, comms thread only.
  double last_diag_at_ = 0.0;
  uint64_t cycles_ = 0;
  uint64_t state_failures_ = 0;
  uint64_t command_failures_ = 0;
  uint64_t commands_sent_ = 0;
  double state_rtt_sum_ = 0.0;
  uint64_t state_rtt_n_ = 0;
  double command_rtt_sum_ = 0.0;
  uint64_t command_rtt_n_ = 0;
  bool warned_missing_joint_ = false;
  bool warned_null_velocity_ = false;
  double last_command_sent_at_ = 0.0;
  std::vector<double> last_sent_;
  // The joint-command request path, chosen by detect_joint_verb() on the comms
  // thread before its first push and read only there afterwards. Defaults to
  // the goal verb so a failed detection is a working (if stream-hostile)
  // fallback, never a dead one.
  std::string joint_command_path_ = "/set_joint_positions";
};

}  // namespace omnisim_ros2_control

#endif  // OMNISIM_ROS2_CONTROL__OMNISIM_SYSTEM_HPP_
