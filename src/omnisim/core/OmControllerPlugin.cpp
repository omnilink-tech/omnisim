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

#include "OmControllerPlugin.hpp"
#include "OmFileUtil.hpp"
#include "OmProject.hpp"
#include "OmStandardPaths.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QString>
#include <QtCore/QStringList>

static QString gTypeNames[2] = {
  "robot_windows",
  "remote_controls",
};

static void searchPossibleControllerPlugins(QStringList &out, const QStringList &plugins, OmControllerPlugin::Type type) {
  foreach (const QString &pluginPath, plugins) {
    QDir dir(pluginPath + gTypeNames[type]);
    if (dir.exists()) {
      QStringList subDirs = dir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
      foreach (const QString &subDir, subDirs) {
        QString filename = dir.absolutePath() + '/' + subDir + '/' + OmStandardPaths::dynamicLibraryPrefix() + subDir +
                           OmStandardPaths::dynamicLibraryExtension();
        out << filename;
      }
    }
  }
}

// Every "plugins" folder of the installation's default projects and of the extra projects, found once.
static const QStringList &pluginFolders() {
  static QStringList pluginsList;
  static bool firstCall = true;
  if (firstCall) {
    firstCall = false;
    OmFileUtil::searchDirectoryNameRecursively(pluginsList, "plugins", OmStandardPaths::projectsPath() + "default/");
    OmFileUtil::searchDirectoryNameRecursively(pluginsList, "plugins", OmStandardPaths::resourcesProjectsPath());
    foreach (const OmProject *extraProject, *OmProject::extraProjects())
      OmFileUtil::searchDirectoryNameRecursively(pluginsList, "plugins", extraProject->path());
  }
  return pluginsList;
}

const QStringList &OmControllerPlugin::defaultList(Type type) {
  static QStringList lists[2];
  static bool firstCall = true;
  if (firstCall) {
    firstCall = false;

    const QStringList &pluginsList = pluginFolders();

    searchPossibleControllerPlugins(lists[ROBOT_WINDOW], pluginsList, ROBOT_WINDOW);
    lists[ROBOT_WINDOW].sort();

    searchPossibleControllerPlugins(lists[REMOTE_CONTROL], pluginsList, REMOTE_CONTROL);
    lists[REMOTE_CONTROL].sort();
  }
  return lists[type];
}

QString OmControllerPlugin::defaultHtmlWindowFile(const QString &key, const QString &extension) {
  if (key.isEmpty() || key.startsWith('<'))
    return QString();
  foreach (const QString &pluginPath, pluginFolders()) {
    // Only the install's own folders: OmRobotWindow serves a window from anywhere else as a path
    // relative to the CURRENT project, where an extra project's file is not.
    if (!OmFileUtil::isLocatedInInstallationDirectory(pluginPath, true))
      continue;
    const QString path =
      pluginPath + gTypeNames[ROBOT_WINDOW] + '/' + key + '/' + key + '.' + extension;
    const QFileInfo file(path);
    if (file.exists() && file.isFile() && file.isReadable())
      return path;
  }
  return QString();
}

OmControllerPlugin::Type OmControllerPlugin::pluginSubDirectoryToType(const QString &pluginSubDirectory) {
  if (pluginSubDirectory == "remote_controls")
    return REMOTE_CONTROL;
  else
    return ROBOT_WINDOW;
}
