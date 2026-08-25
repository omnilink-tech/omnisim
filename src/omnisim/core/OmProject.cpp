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

#include "OmProject.hpp"
#include "OmWorldFileFormat.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QRegularExpression>

#include <cassert>
#include "OmFileUtil.hpp"
#include "OmLanguage.hpp"
#include "OmPreferences.hpp"
#include "OmStandardPaths.hpp"

static const QString WORLDS_DIR("worlds");
static const QString CONTROLLERS_DIR("controllers");
static const QString PROTOS_DIR("protos");
static const QString PLUGINS_DIR("plugins");
static const QString LIBRARIES_DIR("libraries");
static const QString REMOTE_CONTROL_PLUGINS_DIR("plugins/remote_controls");
static const QString ROBOT_WINDOW_PLUGINS_DIR("plugins/robot_windows");
static const QString NEW_WORLD_FILE_NAME("empty" + OmWorldFileFormat::writeExtension());

static QString gPreviousPath = QString();

static OmProject *gCurrentProject = NULL;
static OmProject *gSystemProject = NULL;
static OmProject *gDefaultProject = NULL;
static QList<OmProject *> *gExtraProjects = NULL;

void OmProject::cleanupCurrentProject() {
  delete gCurrentProject;
}

void OmProject::cleanupDefaultProject() {
  delete gDefaultProject;
}

void OmProject::cleanupSystemProject() {
  delete gSystemProject;
}

OmProject *OmProject::current() {
  if (gCurrentProject == NULL) {
    gCurrentProject = new OmProject(QDir::currentPath());
    qAddPostRoutine(OmProject::cleanupCurrentProject);
  }

  return gCurrentProject;
}

OmProject *OmProject::defaultProject() {
  if (gDefaultProject == NULL) {
    gDefaultProject = new OmProject(OmStandardPaths::projectsPath() + "default/");
    qAddPostRoutine(OmProject::cleanupDefaultProject);
  }

  return gDefaultProject;
}

QList<OmProject *> *OmProject::extraProjects() {
  if (gExtraProjects == NULL) {
    gExtraProjects = new QList<OmProject *>();
    // collect extra project paths in a QSet to avoid duplicate entries
    QSet<QString> projectPaths;

    if (!OmPreferences::instance()->value("General/extraProjectPath").toString().isEmpty()) {
      foreach (const QString &pathString, OmPreferences::instance()
                                            ->value("General/extraProjectPath")
                                            .toString()
                                            .split(QDir::listSeparator(), Qt::SkipEmptyParts))
        projectPaths << pathString;
    }

    // OMNISIM_EXTRA_PROJECT_PATH is preferred; WEBOTS_EXTRA_PROJECT_PATH is the legacy alias.
    const QString extraProjectPath = !qEnvironmentVariable("OMNISIM_EXTRA_PROJECT_PATH").isEmpty() ?
                                       qEnvironmentVariable("OMNISIM_EXTRA_PROJECT_PATH") :
                                       qEnvironmentVariable("WEBOTS_EXTRA_PROJECT_PATH");
    if (!extraProjectPath.isEmpty()) {
      foreach (const QString &pathString, extraProjectPath.split(QDir::listSeparator(), Qt::SkipEmptyParts))
        projectPaths << pathString;
    }

    foreach (const QString &projectPath, projectPaths)
      *gExtraProjects << new OmProject(projectPath);

    qAddPostRoutine(OmProject::cleanupExtraProjects);
  }
  return gExtraProjects;
}

void OmProject::cleanupExtraProjects() {
  foreach (OmProject *extraProject, *gExtraProjects)
    delete extraProject;
  delete gExtraProjects;
}

OmProject *OmProject::system() {
  if (gSystemProject == NULL) {
    gSystemProject = new OmProject(OmStandardPaths::resourcesProjectsPath());
    qAddPostRoutine(OmProject::cleanupSystemProject);
  }

  return gSystemProject;
}

void OmProject::setCurrent(OmProject *project) {
  delete gCurrentProject;
  gCurrentProject = project;
}

// Locate the enclosing "worlds" directory by walking up from the world file.
// Upstream Webots only recognizes worlds placed directly under worlds/; OmniSim
// organizes its demo worlds into worlds/<category>/ subfolders (e.g.
// worlds/flagship/foo.wbt), so the project root is the parent of the nearest
// ancestor named "worlds", not just the world file's immediate parent.
// Returns true and sets dir to the "worlds" directory when found.
static bool findWorldsDir(QDir &dir) {
  do {
    if (dir.dirName() == WORLDS_DIR)
      return true;
  } while (dir.cdUp());
  return false;
}

QString OmProject::projectPathFromWorldFile(const QString &fileName, bool &valid) {
  QFileInfo info(fileName);
  assert(info.suffix() == "wbt");
  QDir directory = info.absoluteDir();
  QDir worldsDir = directory;
  valid = findWorldsDir(worldsDir);
  if (valid) {
    // cppcheck-suppress ignoredReturnErrorCode
    worldsDir.cdUp();  // remove "worlds"
    return worldsDir.absolutePath() + "/";
  }
  // Legacy fallback: treat the world file's immediate parent as the worlds dir.
  // cppcheck-suppress ignoredReturnErrorCode
  directory.cdUp();
  return directory.absolutePath() + "/";
}

QString OmProject::projectNameFromWorldFile(const QString &fileName) {
  QFileInfo info(fileName);
  assert(info.suffix() == "wbt");
  QDir directory = info.absoluteDir();
  if (findWorldsDir(directory)) {
    // cppcheck-suppress ignoredReturnErrorCode
    directory.cdUp();  // remove "worlds"
    return directory.dirName();
  }

  return "";
}

QString OmProject::computeBestPathForSaveAs(const QString &fileName) {
  QString suffix = QString("/") + QFileInfo(fileName).fileName();

  QFileInfo fileInfo(fileName);
  if (fileInfo.fileName() != fileName && fileInfo.absoluteDir().exists()) {
    if (!OmFileUtil::isLocatedInInstallationDirectory(fileName))
      return fileName;
  } else {
    const QString projectPath = OmProject::current() ? OmProject::current()->path() : "";
    if (!projectPath.isEmpty() && !OmFileUtil::isLocatedInInstallationDirectory(projectPath))
      return projectPath + suffix;
  }
  return QDir::homePath() + suffix;
}

OmProject::OmProject(const QString &path) {
  if (OmWorldFileFormat::isWorldFile(path)) {
    bool isValidProject = true;
    setPath(projectPathFromWorldFile(path, isValidProject));
  } else
    setPath(path);
}

OmProject::~OmProject() {
  gPreviousPath = mPath;
}

void OmProject::setPath(const QString &path) {
  if (!mPath.isEmpty())
    gPreviousPath = mPath;
  QString oldPath = mPath;
  mPath = QDir(path).absolutePath() + "/";
  emit pathChanged(oldPath, mPath);
}

QString OmProject::dirName() const {
  return QDir(mPath).dirName();
}

QString OmProject::worldsPath() const {
  return mPath + WORLDS_DIR + "/";
}

QDir OmProject::dir() const {
  return QDir(mPath);
}

QString OmProject::controllersPath() const {
  return mPath + CONTROLLERS_DIR + "/";
}

QString OmProject::librariesPath() const {
  return mPath + LIBRARIES_DIR + "/";
}

QString OmProject::protosPath() const {
  return mPath + PROTOS_DIR + "/";
}

QString OmProject::pluginsPath() const {
  return mPath + PLUGINS_DIR + "/";
}

QString OmProject::remoteControlPluginsPath() const {
  return mPath + REMOTE_CONTROL_PLUGINS_DIR + "/";
}

QString OmProject::robotWindowPluginsPath() const {
  return mPath + ROBOT_WINDOW_PLUGINS_DIR + "/";
}

QStringList OmProject::newProjectFiles() const {
  QStringList list;
  list << mPath;
  list << worldsPath();
  list << worldsPath() + NEW_WORLD_FILE_NAME;
  list << controllersPath();
  list << protosPath();
  list << pluginsPath();
  list << remoteControlPluginsPath();
  list << robotWindowPluginsPath();
  list << librariesPath();
  return list;
}

QString OmProject::newWorldPath() {
  return OmStandardPaths::emptyProjectPath() + "worlds/" + NEW_WORLD_FILE_NAME;
}

bool OmProject::createNewProjectFolders() {
  QDir directory(mPath);

  // create sub dirs
  bool success = directory.mkpath(WORLDS_DIR);
  success = success && directory.mkpath(CONTROLLERS_DIR);
  success = success && directory.mkpath(PROTOS_DIR);
  success = success && directory.mkpath(PLUGINS_DIR);
  success = success && directory.mkpath(REMOTE_CONTROL_PLUGINS_DIR);
  success = success && directory.mkpath(ROBOT_WINDOW_PLUGINS_DIR);
  success = success && directory.mkpath(LIBRARIES_DIR);
  return success;
}

bool OmProject::isReadOnly() const {
  return OmFileUtil::isLocatedInInstallationDirectory(mPath, true);
}

QString OmProject::controllerPathFromDir(const QString &dirPath) {
  if (dirPath.isEmpty())
    return QString();

  QDir controllersDir(dirPath);
  if (!controllersDir.exists())
    return QString();

  QString controllersName = controllersDir.dirName();
  QStringList fileNameFilters = OmLanguage::sourceFileExtensions();
  fileNameFilters.replaceInStrings(QRegularExpression("^"), controllersName);  // prepend controller name to each item

  // Search into the current controllers directory (perfect match)
  // case sensitive
  QStringList fileList = controllersDir.entryList(fileNameFilters, QDir::Files | QDir::CaseSensitive);
  if (!fileList.isEmpty())
    return controllersDir.absoluteFilePath(fileList.at(0));

  // case insensitive
  fileList = controllersDir.entryList(fileNameFilters, QDir::Files);
  if (!fileList.isEmpty())
    return controllersDir.absoluteFilePath(fileList.at(0));

  // any source file
  QStringList sourceFileFilters = OmLanguage::sourceFileExtensions();
  sourceFileFilters.replaceInStrings(QRegularExpression("^"), "*");     // prepend "*" to each item
  fileList = controllersDir.entryList(sourceFileFilters, QDir::Files);  // case insensitive
  if (!fileList.isEmpty())
    return controllersDir.absoluteFilePath(fileList.at(0));

  return QString();
}
