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

#ifndef WB_APPLICATION_INFO_HPP
#define WB_APPLICATION_INFO_HPP

#include <QtCore/QString>

class WbVersion;

namespace WbApplicationInfo {
  // version() is the upstream Webots format version (e.g. "R2025a"). It is the
  // file-format compatibility marker written to .wbt / .proto headers
  // ("#VRML_SIM ... utf8") and validated when loading them. Bumping it would
  // mark every existing world as "from a future version".
  const WbVersion &version();
  // omniSimVersion() is the OmniSim release label (e.g. "1.0"). Used purely
  // for user-facing branding (About box, splash, window title, --version,
  // welcome dialog). Independent of the Webots format version above.
  const QString &omniSimVersion();
  const QString &branch();
  const QString &repo();
  const QString &commit();
  unsigned int releaseDate();  // returns the UNIX time stamp of the compilation date
  const QString getInfoFromFile(const QString &name);
}  // namespace WbApplicationInfo

#endif
