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

#include "OmPreferences.hpp"

#include "OmLog.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"

#ifdef _WIN32
#include "OmWindowsRegistry.hpp"
#endif

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QRegularExpressionMatch>
#include <QtCore/QStandardPaths>

static OmPreferences *gInstance = NULL;

OmPreferences *OmPreferences::createInstance(const QString &companyName, const QString &applicationName,
                                             const OmVersion &version) {
  if (gInstance)
    delete gInstance;

#ifdef __linux__
  if (OmSysInfo::isRootUser())
    QSettings::setPath(QSettings::NativeFormat, QSettings::UserScope, "/root/.config/");
#endif
  gInstance = new OmPreferences(companyName, applicationName, version);
  return gInstance;
}

OmPreferences *OmPreferences::instance() {
  return gInstance;
}

void OmPreferences::cleanup() {
  delete gInstance;
  gInstance = NULL;
}

OmPreferences::OmPreferences(const QString &companyName, const QString &applicationName, const OmVersion &version) :
  QSettings(QSettings::NativeFormat, QSettings::UserScope, companyName,
            QString("%1-%2").arg(applicationName).arg(version.toString(false))),
  mCompanyName(companyName),
  mApplicationName(applicationName),
  mVersion(version) {
  // use only one preferences file
  setFallbacksEnabled(false);
  // set defaults for preferences that are accessed from several locations
  setDefault("General/startupMode", "Real-time");
  setDefault("General/rendering", true);
  setDefault("General/language", "");
  setDefault("General/numberOfThreads", OmSysInfo::coreCount());
  setDefault("General/disableSaveWarning", false);
  setDefault("General/thumbnail", true);
  setDefault("Sound/mute", true);
  setDefault("Sound/volume", 80);
  setDefault("OpenGL/disableShadows", false);
  setDefault("OpenGL/disableAntiAliasing", false);
  setDefault("OpenGL/GTAO", 2);
  setDefault("OpenGL/textureQuality", 4);
  setDefault("OpenGL/textureFiltering", 4);
  setDefault("View3d/hideAllCameraOverlays", false);
  setDefault("View3d/hideAllRangeFinderOverlays", false);
  setDefault("View3d/hideAllDisplayOverlays", false);
  setDefault("Network/cacheSize", 1024);
  // OmniSim does not operate a simulation upload service, and must not default to
  // upstream's (webots.cloud) -- that would silently point our users at Cyberbotics'
  // infrastructure. Empty means "no upload service configured"; users who run their
  // own can set it in Preferences > Web Services.
  setDefault("Network/uploadUrl", "");
  setDefault("RobotWindow/newBrowserWindow", false);
  setDefault("RobotWindow/browser", "");

  // Migrate legacy theme keys: the QSS resources were renamed from
  // webots_{classic,dusk,night}.qss to omnisim_*.qss. Without this rewrite,
  // a preferences file from before the rename points at a resource that no
  // longer exists and Qt falls back to the unstyled native look.
  const QString existingTheme = value("General/theme").toString();
  if (existingTheme.startsWith("webots_") && existingTheme.endsWith(".qss"))
    setValue("General/theme", "omnisim_" + existingTheme.mid(7));

  // Dark (Night) is the default face of OmniSim on every platform, matching the
  // OmniSim/OmniLink brand. The light "Classic" theme stays available for users
  // who want it (bright rooms, projector demos, white-background screenshots).
#ifdef _WIN32
  // "Monospace" isn't supported under Windows: the non-monospaced Arial font is loaded instead
  setDefault("Editor/font", "Consolas,10");
  setDefault("General/theme", "omnisim_night.qss");
#elif defined(__APPLE__)
  setDefault("Editor/font", "Courier New,14");  // "Monospace" isn't supported under MacOS
  setDefault("General/theme", "omnisim_night.qss");
#else
  setDefault("Editor/font", "Monospace, 9");
  setDefault("General/theme", "omnisim_night.qss");
#endif  // "Consolas" seems to be a standard Windows monospaced font, so we use it instead
  setDefault("Internal/firstLaunch", true);
  setDefault("Movie/resolution", 6);  // 480p: 854 x 480
  setDefault("Movie/quality", 90);
  setDefault("Movie/acceleration", 1.0);
  setDefault("Movie/caption", false);

  setDefault("Directories/projects", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");
  setDefault("Directories/movies", QStandardPaths::writableLocation(QStandardPaths::MoviesLocation) + "/");
  setDefault("Directories/screenshots", QStandardPaths::writableLocation(QStandardPaths::PicturesLocation) + "/");
  setDefault("Directories/vrml", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");
  setDefault("Directories/objects", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");
  setDefault("Directories/www", QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) + "/");

  setDefaultPythonCommand();
}

OmPreferences::~OmPreferences() {
  setValue("Internal/firstLaunch", false);
}

void OmPreferences::setDefaultPythonCommand() {
#ifdef _WIN32
  const QString command = "python";
#else
  const QString command = "python3";
#endif
  QProcess process;
  process.start(command + OmStandardPaths::executableExtension(), QStringList() << "-c"
                                                                                << "print('PYTHON_COMMAND_FOUND');");
  process.waitForFinished();
  if (process.readAll().startsWith("PYTHON_COMMAND_FOUND")) {
    setDefault("General/pythonCommand", command);
    return;
  }
  setDefault("General/pythonCommand", "");
}

void OmPreferences::setDefault(const QString &key, const QVariant &value) {
  if (!contains(key))
    setValue(key, value);
}

void OmPreferences::setMoviePreferences(int resolutionIndex, int quality, double acceleration, bool caption) {
  setValue("Movie/resolution", resolutionIndex);
  setValue("Movie/quality", quality);
  setValue("Movie/acceleration", acceleration);
  setValue("Movie/caption", caption);
}

void OmPreferences::moviePreferences(int &resolutionIndex, int &quality, double &acceleration, bool &caption) const {
  resolutionIndex = value("Movie/resolution").toInt();
  quality = value("Movie/quality").toInt();
  acceleration = value("Movie/acceleration").toDouble();
  caption = value("Movie/caption").toBool();
}

QString OmPreferences::accessErrorString() const {
  if (status() == QSettings::AccessError) {
    if (QFile::exists(fileName()))
      return "<font color=\"red\">" + tr("Errors when accessing the preferences file.") + "<br>" +
             tr("If the error persists, please check your access rights on file:") + "<br>" + fileName() + "</font>";
  }

  return QString();
}

QString OmPreferences::findPreviousSettingsLocation() const {
  QStringList potentialLocations;

#ifdef _WIN32
  const QString registryRootLocation = QString("\\HKEY_CURRENT_USER\\SOFTWARE\\%1\\").arg(mCompanyName);
  potentialLocations = OmWindowsRegistry(registryRootLocation).subKeys();
  potentialLocations.replaceInStrings(QRegularExpression("^"), registryRootLocation);
#else

#ifdef __APPLE__
  QDir preferencesDirectory(QStandardPaths::writableLocation(QStandardPaths::ConfigLocation));
#else  // __linux__
  QDir preferencesDirectory(QStandardPaths::writableLocation(QStandardPaths::ConfigLocation) + "/" + mCompanyName);
#endif

  preferencesDirectory.setFilter(QDir::Files);
  QStringList filters;
#ifdef __APPLE__
  filters << QString("com.%1.%2*.plist").arg(mCompanyName.toLower()).arg(mApplicationName);
#else  // __linux__
  filters << QString("%1*.conf").arg(mApplicationName);
#endif
  preferencesDirectory.setNameFilters(filters);

  QFileInfoList preferencesFileInfos = preferencesDirectory.entryInfoList();
  for (int i = 0; i < preferencesFileInfos.size(); ++i) {
    QFileInfo preferencesFileInfo = preferencesFileInfos.at(i);
    potentialLocations << preferencesFileInfo.absoluteFilePath();
  }
#endif

  QString lastLocation;
  OmVersion lastLocationVersion;

  foreach (const QString &location, potentialLocations) {
    QFileInfo preferencesFileInfo(location);
    QRegularExpressionMatch match;
    OmVersion versionOfMatchedFile;
    OmVersion versionToTest(mVersion);
    if (preferencesFileInfo.fileName().contains(QRegularExpression("R\\d+\\w+"), &match))
      versionToTest.setRevision(0);  // the maintenance version should not be present when testing
    else if (preferencesFileInfo.fileName().contains(QRegularExpression("\\d+\\.\\d+\\.\\d+"), &match)) {
    } else if (preferencesFileInfo.fileName().contains(QRegularExpression("\\d+\\.\\d+"), &match))
      versionToTest.setRevision(0);  // the maintenance version should not be present when testing
    else
      // file name doesn't match any expected config file pattern
      continue;

    versionOfMatchedFile.fromString(match.captured());
    if (versionOfMatchedFile > lastLocationVersion && versionOfMatchedFile < versionToTest) {
      lastLocationVersion = versionOfMatchedFile;
      lastLocation = location;
    }
  }

  return lastLocation;
}

#ifdef __linux__
void OmPreferences::checkIsWritable() {
  if (!isWritable())
    OmLog::warning(tr("\nPreferences file cannot be overwritten.\n"
                      "Any change to the current settings won't be restored at next OmniSim start.\n\n"
                      "Please check the write permissions on file:\n\"%1\"")
                     .arg(fileName()),
                   true);
}
#endif

bool OmPreferences::booleanEnvironmentVariable(const QByteArray &variable) {
  const QByteArray content = qgetenv(variable).toLower();
  return !content.isEmpty() && content != "0" && content != "false";
}
