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

#include "omnisim_ros2_control/http_client.hpp"

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <string>

namespace omnisim_ros2_control
{

namespace
{

double now_s()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

/// Case-insensitive header lookup over a raw header block.
std::string header_value(const std::string & headers_lower, const char * name)
{
  const std::string key = std::string("\r\n") + name + ":";
  size_t pos = headers_lower.find(key);
  if (pos == std::string::npos) {return "";}
  pos += key.size();
  const size_t end = headers_lower.find("\r\n", pos);
  std::string v = headers_lower.substr(pos, end - pos);
  const size_t first = v.find_first_not_of(" \t");
  if (first == std::string::npos) {return "";}
  return v.substr(first, v.find_last_not_of(" \t") - first + 1);
}

}  // namespace

HttpClient::~HttpClient()
{
  disconnect();
}

bool HttpClient::configure(const std::string & base_url, double timeout_s, std::string & error)
{
  std::string url = base_url;
  const std::string scheme = "http://";
  if (url.compare(0, scheme.size(), scheme) == 0) {
    url = url.substr(scheme.size());
  } else if (url.find("://") != std::string::npos) {
    error = "only http:// URLs are supported, got: " + base_url;
    return false;
  }
  while (!url.empty() && url.back() == '/') {url.pop_back();}
  const size_t slash = url.find('/');
  if (slash != std::string::npos) {
    error = "base_url must not carry a path, got: " + base_url;
    return false;
  }
  const size_t colon = url.rfind(':');
  if (colon == std::string::npos) {
    host_ = url;
    port_ = "80";
  } else {
    host_ = url.substr(0, colon);
    port_ = url.substr(colon + 1);
  }
  if (host_.empty() || port_.empty()) {
    error = "could not parse host/port from: " + base_url;
    return false;
  }
  host_header_ = host_ + ":" + port_;
  base_url_ = "http://" + host_header_;
  timeout_s_ = timeout_s > 0.0 ? timeout_s : 2.0;
  error.clear();
  return true;
}

void HttpClient::disconnect()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  pending_.clear();
}

bool HttpClient::ensure_connected(std::string & error)
{
  if (fd_ >= 0) {return true;}

  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo * res = nullptr;
  const int rc = ::getaddrinfo(host_.c_str(), port_.c_str(), &hints, &res);
  if (rc != 0 || res == nullptr) {
    error = "getaddrinfo(" + host_header_ + "): " + std::string(::gai_strerror(rc));
    return false;
  }
  int sock = -1;
  for (addrinfo * ai = res; ai != nullptr; ai = ai->ai_next) {
    sock = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock < 0) {continue;}
    timeval tv{};
    tv.tv_sec = static_cast<time_t>(timeout_s_);
    tv.tv_usec = static_cast<suseconds_t>((timeout_s_ - tv.tv_sec) * 1e6);
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    int one = 1;
    // Nagle would add up to 40 ms to a small request/response pair, which is
    // most of the per-cycle budget at any control rate worth having.
    ::setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    if (::connect(sock, ai->ai_addr, ai->ai_addrlen) == 0) {break;}
    ::close(sock);
    sock = -1;
  }
  ::freeaddrinfo(res);
  if (sock < 0) {
    error = "connect(" + host_header_ + "): " + std::string(std::strerror(errno));
    return false;
  }
  fd_ = sock;
  ++connections_opened_;
  error.clear();
  return true;
}

bool HttpClient::read_until(const std::string & needle, std::string & buf, std::string & error)
{
  char chunk[4096];
  while (buf.find(needle) == std::string::npos) {
    const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
    if (n > 0) {
      buf.append(chunk, static_cast<size_t>(n));
      continue;
    }
    if (n == 0) {
      error = "peer closed after " + std::to_string(buf.size()) +
        " bytes, before the response headers were complete";
      return false;
    }
    error = std::string("recv: ") + std::strerror(errno);
    return false;
  }
  return true;
}

bool HttpClient::read_exactly(size_t n, std::string & buf, std::string & error)
{
  char chunk[8192];
  while (buf.size() < n) {
    const ssize_t got = ::recv(fd_, chunk, std::min(sizeof(chunk), n - buf.size()), 0);
    if (got > 0) {
      buf.append(chunk, static_cast<size_t>(got));
      continue;
    }
    if (got == 0) {
      error = "peer closed with " + std::to_string(n - buf.size()) + " body bytes outstanding";
      return false;
    }
    error = std::string("recv: ") + std::strerror(errno);
    return false;
  }
  return true;
}

HttpResponse HttpClient::get(const std::string & path)
{
  return request("GET", path, nullptr);
}

HttpResponse HttpClient::post(const std::string & path, const std::string & json_body)
{
  return request("POST", path, &json_body);
}

HttpResponse HttpClient::request(
  const char * method, const std::string & path, const std::string * body)
{
  HttpResponse out;
  const double t0 = now_s();

  // One retry, and only for a socket we were REUSING. A kept-alive connection
  // can be closed by the far side at any moment between our last read and this
  // write, and that race is indistinguishable from a real failure until we try.
  // A fresh socket that fails is a real failure and must not be retried, or a
  // dead harness costs two connect timeouts per cycle instead of one.
  for (int attempt = 0; attempt < 2; ++attempt) {
    const bool reused = fd_ >= 0;
    std::string error;
    if (!ensure_connected(error)) {
      out.transport_error = error;
      out.round_trip_s = now_s() - t0;
      return out;
    }

    std::string req;
    req.reserve(256 + (body ? body->size() : 0));
    req += method;
    req += ' ';
    req += path;
    req += " HTTP/1.1\r\nHost: ";
    req += host_header_;
    req += "\r\nAccept: application/json\r\nConnection: keep-alive\r\n";
    if (body != nullptr) {
      req += "Content-Type: application/json\r\nContent-Length: ";
      req += std::to_string(body->size());
      req += "\r\n";
    }
    req += "\r\n";
    if (body != nullptr) {req += *body;}

    size_t sent = 0;
    bool write_failed = false;
    while (sent < req.size()) {
      const ssize_t n = ::send(fd_, req.data() + sent, req.size() - sent, MSG_NOSIGNAL);
      if (n <= 0) {
        write_failed = true;
        break;
      }
      sent += static_cast<size_t>(n);
    }
    if (write_failed) {
      const std::string why = std::string("send: ") + std::strerror(errno);
      disconnect();
      if (reused && attempt == 0) {continue;}
      out.transport_error = why;
      out.round_trip_s = now_s() - t0;
      return out;
    }
    ++requests_sent_;

    std::string buf = pending_;
    pending_.clear();
    std::string err;
    if (!read_until("\r\n\r\n", buf, err)) {
      disconnect();
      if (reused && attempt == 0 && buf.empty()) {continue;}
      out.transport_error = err;
      out.round_trip_s = now_s() - t0;
      return out;
    }
    const size_t head_end = buf.find("\r\n\r\n") + 4;
    const std::string head = buf.substr(0, head_end);
    const std::string head_lower = lower(head);

    // Status line: "HTTP/1.x NNN Reason".
    const size_t sp = head.find(' ');
    if (sp == std::string::npos || head.size() < sp + 4) {
      disconnect();
      out.transport_error = "malformed status line";
      out.round_trip_s = now_s() - t0;
      return out;
    }
    int status = 0;
    std::from_chars(head.data() + sp + 1, head.data() + sp + 4, status);
    out.status = status;

    const bool server_is_http10 = head_lower.compare(0, 9, "http/1.0 ") == 0;
    const std::string conn = header_value("\r\n" + head_lower, "connection");
    const std::string te = header_value("\r\n" + head_lower, "transfer-encoding");
    const std::string cl = header_value("\r\n" + head_lower, "content-length");

    std::string rest = buf.substr(head_end);
    if (!te.empty() && te.find("chunked") != std::string::npos) {
      // Chunked framing. OmniSim does not use it today, but a reverse proxy in
      // front of the harness would, and mis-reading a chunk header as JSON
      // would be a silent corruption rather than an error.
      std::string decoded;
      while (true) {
        if (!read_until("\r\n", rest, err)) {break;}
        const size_t eol = rest.find("\r\n");
        size_t size = 0;
        const std::string hex = rest.substr(0, eol);
        std::from_chars(hex.data(), hex.data() + hex.size(), size, 16);
        rest.erase(0, eol + 2);
        if (size == 0) {break;}
        std::string chunk_buf = rest;
        rest.clear();
        if (!read_exactly(size + 2, chunk_buf, err)) {break;}
        decoded.append(chunk_buf, 0, size);
        rest = chunk_buf.substr(size + 2);
      }
      out.body = decoded;
      pending_.clear();
    } else if (!cl.empty()) {
      size_t length = 0;
      std::from_chars(cl.data(), cl.data() + cl.size(), length);
      if (rest.size() < length && !read_exactly(length, rest, err)) {
        disconnect();
        out.transport_error = err;
        out.round_trip_s = now_s() - t0;
        return out;
      }
      out.body = rest.substr(0, length);
      pending_ = rest.substr(length);
    } else {
      // No framing at all: read to EOF, which is HTTP/1.0's only option.
      char chunk[8192];
      while (true) {
        const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
        if (n <= 0) {break;}
        rest.append(chunk, static_cast<size_t>(n));
      }
      out.body = rest;
      pending_.clear();
      disconnect();
    }

    const bool keep = !server_is_http10 && conn.find("close") == std::string::npos;
    if (keep) {
      keep_alive_granted_ = true;
    } else {
      disconnect();
    }
    out.round_trip_s = now_s() - t0;
    return out;
  }

  out.transport_error = "unreachable";
  out.round_trip_s = now_s() - t0;
  return out;
}

}  // namespace omnisim_ros2_control
