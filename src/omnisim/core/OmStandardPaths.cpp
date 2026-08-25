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

#include "OmStandardPaths.hpp"

#include "OmApplicationInfo.hpp"
#include "OmLog.hpp"
#include "OmPreferences.hpp"
#include "OmSimulationState.hpp"
#include "OmSysInfo.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QHash>
#include <QtCore/QProcess>
#include <QtCore/QStandardPaths>
#include <QtCore/QString>
#include <QtCore/QTextStream>
#include <QtCore/QTimer>

#ifdef _WIN32
#include "../../../include/controller/c/omnisim/utils/system.h"
#endif

const QString &OmStandardPaths::omniSimHomePath() {
  static QString path;
#ifdef __linux__
  // on Linux, the webots binary is located in $WEBOTS_HOME/bin/omnisim-bin
  const int depth = 1;
#elif defined(__APPLE__)
  // on macOS, the webots binary is located in $WEBOTS_HOME/Contents/MacOS/webots
  const int depth = 2;
#else
  // on Windows, the webots binary is located in $WEBOTS_HOME/msys64/mingw64/bin/webots
  const int depth = 3;
#endif
  if (path.isEmpty()) {
    QDir dir(QCoreApplication::applicationDirPath());
    for (int i = 0; i < depth; i++)
      if (!dir.cdUp())
        assert(false);
    path = dir.absolutePath() + "/";
  }
  return path;
};

#ifdef __APPLE__
static const QString cMacOsContents = "Contents/";
#else
static const QString cMacOsContents;
#endif

const QString &OmStandardPaths::omniSimLibPath() {
  static QString path = omniSimHomePath() + cMacOsContents + "lib/webots/";
  return path;
}

const QString &OmStandardPaths::controllerLibPath() {
  static QString path = omniSimHomePath() + cMacOsContents + "lib/controller/";
  return path;
}

#ifdef _WIN32
const QString &OmStandardPaths::omniSimMsys64Path() {
  static QString path = omniSimHomePath() + "msys64/";
  return path;
}
#endif

const QString &OmStandardPaths::localDocPath() {
  static QString url(omniSimHomePath() + cMacOsContents + "docs/");
  return url;
};

const QString &OmStandardPaths::projectsPath() {
  static QString path(omniSimHomePath() + cMacOsContents + "projects/");
  return path;
};

const QString &OmStandardPaths::resourcesPath() {
#ifdef __APPLE__
  static QString path(omniSimHomePath() + "Contents/Resources/");
#else
  static QString path(omniSimHomePath() + "resources/");
#endif
  return path;
};

const QString &OmStandardPaths::templatesPath() {
  static QString path(resourcesPath() + "templates/");
  return path;
};

const QString &OmStandardPaths::fontsPath() {
  static QString path(resourcesPath() + "fonts/");
  return path;
};

// Legacy font-name compatibility layer.
//
// Webots historically shipped (and Webots-era controllers therefore request by name) the Microsoft
// core fonts: Arial, Times New Roman, Verdana, ... Those binaries are proprietary and cannot be
// redistributed under the Apache-2.0 license OmniSim ships under, so they were removed. OmniSim
// ships metric-compatible (Liberation) or visually-equivalent (DejaVu) libre replacements instead,
// and maps the legacy names onto them here so that existing controllers keep rendering text.
//
// "Metric-compatible" means identical glyph advance widths, so text laid out for the original font
// occupies exactly the same space: Liberation Sans <-> Arial, Liberation Serif <-> Times New Roman,
// Liberation Mono <-> Courier New. The remaining mappings are style substitutions, not metric
// matches. Comic Sans MS and Impact have no libre substitute and are no longer available.
static const QHash<QString, QString> &fontAliases() {
  static const QHash<QString, QString> aliases({
    // legacy name            libre replacement (basename in resources/fonts/)
    {"Arial", "LiberationSans-Regular"},                 // metric-compatible
    {"Ariali", "LiberationSans-Italic"},                 // metric-compatible (Arial Italic)
    {"Arial Black", "LiberationSans-Bold"},              // approximation (no libre Arial Black)
    {"Times New Roman", "LiberationSerif-Regular"},      // metric-compatible
    {"Courier New", "LiberationMono-Regular"},           // metric-compatible
    {"Lucida Console", "DejaVuSansMono"},                // monospace substitute
    {"Lucida Sans Unicode", "DejaVuSans"},               // substitute
    {"Tahoma", "DejaVuSans"},                            // substitute
    {"Verdana", "DejaVuSans"},                           // substitute
    {"Trebuchet MS", "LiberationSans-Regular"},          // substitute
    {"Georgia", "DejaVuSerif"},                          // substitute
    {"Palatino Linotype", "DejaVuSerif"},                // substitute
  });
  return aliases;
}

QString OmStandardPaths::shippedFontFile(const QString &fontName) {
  const QString &directHit = fontsPath() + fontName + ".ttf";
  if (QFile::exists(directHit))
    return directHit;

  const QString &alias = fontAliases().value(fontName);
  if (!alias.isEmpty()) {
    const QString &aliasHit = fontsPath() + alias + ".ttf";
    if (QFile::exists(aliasHit))
      return aliasHit;
  }

  return QString();
}

const QString &OmStandardPaths::resourcesProjectsPath() {
  static QString path(resourcesPath() + "projects/");
  return path;
};

const QString &OmStandardPaths::resourcesControllersPath() {
  static QString path(resourcesProjectsPath() + "controllers/");
  return path;
};

const QString &OmStandardPaths::resourcesSoundPluginsPath() {
  static QString path(resourcesProjectsPath() + "plugins/sound/");
  return path;
};

const QString &OmStandardPaths::resourcesRobotWindowsPluginsPath() {
  static QString path(resourcesProjectsPath() + "plugins/robot_windows/");
  return path;
}

const QString &OmStandardPaths::resourcesWebPath() {
  static QString path(resourcesPath() + "web/");
  return path;
}

const QString &OmStandardPaths::omniSimDocsBaseUrl() {
  // OmniSim ships its docs/ tree in the repo. GitHub renders Markdown nicely,
  // so the "User Guide" / "Reference Manual" / per-node "Help" links point at
  // the raw repo blob URLs rather than a hosted documentation site (which
  // OmniSim does not currently run).
  static QString url("https://github.com/omnilink-tech/omnisim/blob/main/docs");
  return url;
};

const QString &OmStandardPaths::githubRepositoryUrl() {
  static const QString url("https://github.com/omnilink-tech/omnisim");
  return url;
};

const QString &OmStandardPaths::dynamicLibraryExtension() {
#ifdef __APPLE__
  static QString extension(".dylib");
#elif defined(_WIN32)
  static QString extension(".dll");
#else  // __linux__
  static QString extension(".so");
#endif
  return extension;
}

const QString &OmStandardPaths::dynamicLibraryPrefix() {
#ifdef _WIN32
  static QString suffix("");
#else
  static QString suffix("lib");
#endif
  return suffix;
}

const QString &OmStandardPaths::executableExtension() {
#ifdef _WIN32
  static QString extension(".exe");
#else
  static QString extension("");
#endif
  return extension;
}

const QString &OmStandardPaths::emptyProjectPath() {
  // OMNISIM_EMPTY_PROJECT_PATH is preferred; WEBOTS_EMPTY_PROJECT_PATH is the legacy alias.
  const QByteArray emptyProjectEnv =
    qgetenv("OMNISIM_EMPTY_PROJECT_PATH").isEmpty() ? qgetenv("WEBOTS_EMPTY_PROJECT_PATH") : qgetenv("OMNISIM_EMPTY_PROJECT_PATH");
  if (emptyProjectEnv.isEmpty())
    return resourcesProjectsPath();

  static QString path;
  if (path.isEmpty())
    path = QDir(emptyProjectEnv).absolutePath() + "/";
  return path;
}

const QString &OmStandardPaths::unnamedTextFile() {
  static QString fileName("unnamed.txt");
  return fileName;
};

static void liveWebotsTmpPath() {
  QFile file(OmStandardPaths::webotsTmpPath() + "live.txt");
  if (file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
    QTextStream out(&file);
    out << QDateTime::currentSecsSinceEpoch();
    file.close();
  }
}

static QString cWebotsTmpPath;
static int cWebotsTmpPathId = -1;

bool OmStandardPaths::webotsTmpPathCreate(const int id) {
  assert(cWebotsTmpPathId == -1 && cWebotsTmpPath.isEmpty());  // we should create it once
#ifdef _WIN32
  // We do not use QDir::tempPath() as it relies on the TEMP/TMP environment variables which are overriden by the MSYS2
  // console to C:\msys2\tmp whereas the libController uses the LOCALAPPDATA version, e.g., C:\Users\user\AppData\Local\Temp
  cWebotsTmpPath =
    QDir::fromNativeSeparators(OmSysInfo::environmentVariable("LOCALAPPDATA")) + QString("/Temp/webots-%1/").arg(id);
#else
  QString username = qgetenv("USER");
  if (username.isEmpty()) {
    username = qgetenv("USERNAME");
    if (username.isEmpty()) {
      OmLog::error(QObject::tr("USER or USERNAME environment variable not set, falling back to 'default' username."));
      username = "default";
    }
  }
#if defined(__APPLE__)
  cWebotsTmpPath = QString("/tmp/webots/%1/%2/").arg(username).arg(id);
#else  // __linux__
  // OMNISIM_TMPDIR is preferred; WEBOTS_TMPDIR is the legacy alias.
  QString WEBOTS_TMPDIR = OmSysInfo::environmentVariable("OMNISIM_TMPDIR");
  if (WEBOTS_TMPDIR.isEmpty())
    WEBOTS_TMPDIR = OmSysInfo::environmentVariable("WEBOTS_TMPDIR");
  if (!WEBOTS_TMPDIR.isEmpty() && QDir(WEBOTS_TMPDIR).exists())
    cWebotsTmpPath = QString("%1/webots/%2/%3/").arg(WEBOTS_TMPDIR).arg(username).arg(id);
  else {
    cWebotsTmpPath = QString("/tmp/webots/%1/%2/").arg(username).arg(id);
    OmLog::error(QObject::tr("OmniSim has not been started regularly. Some features may not work. "
                             "Please start OmniSim from its launcher."));
  }
#endif
#endif
  // cleanup old and unused tmp directories
  QDir directory(cWebotsTmpPath);
  if (directory.cdUp()) {
    const QStringList &webotsTmp = directory.entryList(QStringList() << "webots-*", QDir::Dirs | QDir::Writable);
    foreach (const QString &dirname, webotsTmp) {
      const QString fullName(directory.absolutePath() + "/" + dirname);
      const QFileInfo fileInfo(fullName + "/live.txt");
      const QDateTime &lastModified = fileInfo.fileTime(QFileDevice::FileModificationTime);
      const qint64 diff = lastModified.secsTo(QDateTime::currentDateTime());
      if (diff > 3600) {  // if the live.txt file was not modified for more than one hour, delete the tmp folder
        QDir d(fullName);
        d.removeRecursively();
      }
    }
  }

  // create the required tmp directories
  QDir dir(cWebotsTmpPath);
  if (!dir.exists() && !dir.mkpath("."))
    return false;

  // write a new live.txt file in the webots tmp folder every hour to prevent any other webots process to delete it
  static QTimer timer;
  liveWebotsTmpPath();
  QTimer::connect(&timer, &QTimer::timeout, liveWebotsTmpPath);
  timer.start(30 * 60 * 1000);  // call every 30 minutes
  cWebotsTmpPathId = id;
  return true;
}

int OmStandardPaths::webotsTmpPathId() {
  return cWebotsTmpPathId;
}

const QString &OmStandardPaths::webotsTmpPath() {
  return cWebotsTmpPath;
}

const QString &OmStandardPaths::cachedAssetsPath() {
  static QString path(QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/assets/");
  return path;
}

const QString &OmStandardPaths::vehicleLibraryPath() {
  static QString path(omniSimHomePath() + cMacOsContents + "projects/default/libraries/vehicle/");
  return path;
}
