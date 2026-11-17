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

#ifndef OM_CONTROLLER_PLUGIN_HPP
#define OM_CONTROLLER_PLUGIN_HPP

#include <QtCore/QStringList>

namespace OmControllerPlugin {

  enum Type { ROBOT_WINDOW, REMOTE_CONTROL };

  // absolute filenames in the resources directory, the files doesn't exist necessarily
  const QStringList &defaultList(Type type);

  // The HTML robot window `key` shared by the installation: the first existing
  // `<plugins>/robot_windows/<key>/<key>.<extension>` among the plugin folders defaultList()
  // searches that lie inside the installation directory (resources/projects/..., projects/default/...).
  // Empty if there is none. A world outside the install has no copy of such a window in its own
  // project, and only files inside the install are served to the browser from anywhere but that project.
  QString defaultHtmlWindowFile(const QString &key, const QString &extension);

  Type pluginSubDirectoryToType(const QString &pluginSubDirectory);

};  // namespace OmControllerPlugin

#endif
