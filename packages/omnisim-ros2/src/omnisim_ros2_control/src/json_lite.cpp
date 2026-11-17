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

#include "omnisim_ros2_control/json_lite.hpp"

#include <charconv>
#include <cstdio>
#include <cstring>

namespace omnisim_ros2_control
{
namespace json
{

namespace
{

struct Parser
{
  const std::string & s;
  size_t i = 0;
  std::string err;

  explicit Parser(const std::string & text)
  : s(text) {}

  void skip_ws()
  {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {++i;}
  }

  bool fail(const std::string & why)
  {
    if (err.empty()) {
      err = why + " at offset " + std::to_string(i);
    }
    return false;
  }

  bool literal(const char * word)
  {
    const size_t n = std::strlen(word);
    if (s.compare(i, n, word) != 0) {return fail(std::string("expected ") + word);}
    i += n;
    return true;
  }

  bool parse_string(std::string & out)
  {
    if (i >= s.size() || s[i] != '"') {return fail("expected string");}
    ++i;
    out.clear();
    while (i < s.size()) {
      const char c = s[i++];
      if (c == '"') {return true;}
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (i >= s.size()) {return fail("truncated escape");}
      const char e = s[i++];
      switch (e) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
            if (i + 4 > s.size()) {return fail("truncated \\u escape");}
            unsigned code = 0;
            for (int k = 0; k < 4; ++k) {
              const char h = s[i + k];
              code <<= 4;
              if (h >= '0' && h <= '9') {code |= static_cast<unsigned>(h - '0');} else if (h >= 'a' && h <= 'f') {
                code |= static_cast<unsigned>(h - 'a' + 10);
              } else if (h >= 'A' && h <= 'F') {
                code |= static_cast<unsigned>(h - 'A' + 10);
              } else {return fail("bad \\u escape");}
            }
            i += 4;
            // Minimal UTF-8 encode. Surrogate pairs are not recombined: no
            // OmniSim payload carries astral-plane text, and mangling one is
            // better than failing the whole parse of a joint list over a name.
            if (code < 0x80) {
              out.push_back(static_cast<char>(code));
            } else if (code < 0x800) {
              out.push_back(static_cast<char>(0xC0 | (code >> 6)));
              out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
            } else {
              out.push_back(static_cast<char>(0xE0 | (code >> 12)));
              out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
              out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
            }
            break;
          }
        default: return fail("unknown escape");
      }
    }
    return fail("unterminated string");
  }

  bool parse_number(double & out)
  {
    const size_t start = i;
    if (i < s.size() && (s[i] == '-' || s[i] == '+')) {++i;}
    while (i < s.size() &&
      ((s[i] >= '0' && s[i] <= '9') || s[i] == '.' || s[i] == 'e' || s[i] == 'E' ||
      ((s[i] == '-' || s[i] == '+') && (s[i - 1] == 'e' || s[i - 1] == 'E'))))
    {
      ++i;
    }
    if (i == start) {return fail("expected number");}
    // Locale-independent by definition -- see the header's LOCALE note.
    const auto res = std::from_chars(s.data() + start, s.data() + i, out);
    if (res.ec != std::errc()) {return fail("malformed number");}
    return true;
  }

  bool parse_value(Value & v, int depth)
  {
    if (depth > 64) {return fail("nesting too deep");}
    skip_ws();
    if (i >= s.size()) {return fail("unexpected end of input");}
    const char c = s[i];
    if (c == '{') {
      ++i;
      v.type = Type::Object;
      skip_ws();
      if (i < s.size() && s[i] == '}') {++i; return true;}
      while (true) {
        skip_ws();
        std::string key;
        if (!parse_string(key)) {return false;}
        skip_ws();
        if (i >= s.size() || s[i] != ':') {return fail("expected ':'");}
        ++i;
        Value child;
        if (!parse_value(child, depth + 1)) {return false;}
        v.object[key] = std::move(child);
        skip_ws();
        if (i < s.size() && s[i] == ',') {++i; continue;}
        if (i < s.size() && s[i] == '}') {++i; return true;}
        return fail("expected ',' or '}'");
      }
    }
    if (c == '[') {
      ++i;
      v.type = Type::Array;
      skip_ws();
      if (i < s.size() && s[i] == ']') {++i; return true;}
      while (true) {
        Value child;
        if (!parse_value(child, depth + 1)) {return false;}
        v.array.push_back(std::move(child));
        skip_ws();
        if (i < s.size() && s[i] == ',') {++i; continue;}
        if (i < s.size() && s[i] == ']') {++i; return true;}
        return fail("expected ',' or ']'");
      }
    }
    if (c == '"') {
      v.type = Type::String;
      return parse_string(v.string);
    }
    if (c == 't') {
      if (!literal("true")) {return false;}
      v.type = Type::Bool;
      v.boolean = true;
      return true;
    }
    if (c == 'f') {
      if (!literal("false")) {return false;}
      v.type = Type::Bool;
      v.boolean = false;
      return true;
    }
    if (c == 'n') {
      if (!literal("null")) {return false;}
      v.type = Type::Null;
      return true;
    }
    v.type = Type::Number;
    return parse_number(v.number);
  }
};

const Value & null_value()
{
  static const Value kNull;
  return kNull;
}

}  // namespace

const Value & Value::operator[](const std::string & key) const
{
  if (type != Type::Object) {return null_value();}
  const auto it = object.find(key);
  return it == object.end() ? null_value() : it->second;
}

bool parse(const std::string & text, Value & out, std::string & error)
{
  Parser p(text);
  out = Value();
  if (!p.parse_value(out, 0)) {
    error = p.err;
    return false;
  }
  p.skip_ws();
  if (p.i != text.size()) {
    error = "trailing content at offset " + std::to_string(p.i);
    return false;
  }
  error.clear();
  return true;
}

std::string number_to_string(double v)
{
  // std::to_chars(double) is locale-independent and round-trips exactly with
  // from_chars, which snprintf("%g") does neither of.
  char buf[40];
  const auto res = std::to_chars(buf, buf + sizeof(buf), v);
  if (res.ec != std::errc()) {return "0";}
  std::string out(buf, res.ptr);
  // JSON has no `inf`/`nan` literals. A non-finite command is a bug upstream;
  // sending 0 would execute a command nobody asked for, so send a token the
  // far side will reject loudly instead.
  if (out.find("inf") != std::string::npos || out.find("nan") != std::string::npos) {
    return "\"" + out + "\"";
  }
  return out;
}

std::string escape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out.push_back(c);
        }
    }
  }
  return out;
}

}  // namespace json
}  // namespace omnisim_ros2_control
