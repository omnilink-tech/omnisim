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

// A blocking, dependency-free HTTP/1.1 client for OmniSim's harness and robot
// bridges.
//
// WHY A HAND-ROLLED CLIENT: the plugin runs inside `controller_manager`, so its
// only hard dependency should be ros2_control itself. libcurl would work but
// adds a system dependency for four request shapes, none of which needs TLS,
// redirects, auth, proxies or compression.
//
// ⚠ KEEP-ALIVE IS *ATTEMPTED*, NOT ASSUMED -- and against today's OmniSim it is
// not granted. Both the harness and the robot bridges are Python
// `BaseHTTPRequestHandler` servers that leave `protocol_version` at its default
// `HTTP/1.0`, which makes the server close the socket after every response no
// matter what the client asks for. This client sends `Connection: keep-alive`,
// reuses the socket when the server actually honours it, and records the answer
// in `keep_alive_granted()` so the cost model is measured rather than guessed.
// That measurement is what the update-rate ceiling in the package README is
// derived from: one TCP connection per request, and a TIME_WAIT entry per
// connection on whichever host closes first.

#ifndef OMNISIM_ROS2_CONTROL__HTTP_CLIENT_HPP_
#define OMNISIM_ROS2_CONTROL__HTTP_CLIENT_HPP_

#include <cstdint>
#include <string>

namespace omnisim_ros2_control
{

struct HttpResponse
{
  /// HTTP status, or 0 when the request never got an answer (connect failure,
  /// timeout, reset). 0 is "transport", any other value is "the server spoke".
  int status = 0;
  std::string body;
  /// Error text when `status == 0`, empty otherwise.
  std::string transport_error;
  /// Wall-clock round trip, including connect when a new socket was needed.
  double round_trip_s = 0.0;

  bool transport_ok() const {return status != 0;}
  bool ok() const {return status >= 200 && status < 300;}
};

/// One client per base URL. NOT thread-safe: it owns a single reusable socket,
/// so give each thread its own instance.
class HttpClient
{
public:
  HttpClient() = default;

  /// `base_url` is `http://host:port` (no path, no trailing slash required).
  /// Returns false when the URL cannot be parsed; the object is then unusable.
  bool configure(const std::string & base_url, double timeout_s, std::string & error);

  HttpResponse get(const std::string & path);
  HttpResponse post(const std::string & path, const std::string & json_body);

  /// True once a server has actually kept a connection open for us. Stays false
  /// against an HTTP/1.0 server, which is the case for OmniSim today.
  bool keep_alive_granted() const {return keep_alive_granted_;}

  /// How many TCP connections this client has opened. Divided by the request
  /// count, this is the exact per-request connection cost the ephemeral-port
  /// budget is spent on.
  uint64_t connections_opened() const {return connections_opened_;}
  uint64_t requests_sent() const {return requests_sent_;}

  const std::string & base_url() const {return base_url_;}

  ~HttpClient();

private:
  HttpResponse request(const char * method, const std::string & path, const std::string * body);
  bool ensure_connected(std::string & error);
  void disconnect();
  /// Read until `needle` is found or the socket ends. Returns false on error.
  bool read_until(const std::string & needle, std::string & buf, std::string & error);
  bool read_exactly(size_t n, std::string & buf, std::string & error);

  std::string base_url_;
  std::string host_;
  std::string port_ = "80";
  std::string host_header_;
  double timeout_s_ = 2.0;
  int fd_ = -1;
  std::string pending_;  // bytes read past the end of the last response
  bool keep_alive_granted_ = false;
  uint64_t connections_opened_ = 0;
  uint64_t requests_sent_ = 0;
};

}  // namespace omnisim_ros2_control

#endif  // OMNISIM_ROS2_CONTROL__HTTP_CLIENT_HPP_
