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
//
// Modifications copyright 2026 OmniLink, licensed under the Apache License, Version 2.0.

#ifdef _WIN32
#include <dwmapi.h>
#include <windows.h>
#include <QtGui/QWindow>
#endif

#include "OmGuiApplication.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmApplication.hpp"
#include "OmApplicationInfo.hpp"
#include "OmConsole.hpp"
#include "OmMainWindow.hpp"
#include "OmMessageBox.hpp"
#include "OmMultimediaStreamingServer.hpp"
#include "OmNetwork.hpp"
#include "OmNewVersionDialog.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSingleTaskApplication.hpp"
#include "OmSplashScreen.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"
#include "OmVersion.hpp"
#include "OmW3dStreamingServer.hpp"
#include "OmWrenLabelOverlay.hpp"
#include "OmWrenRenderingContext.hpp"
#include "OmWorld.hpp"
#include "OmWrenOpenGlContext.hpp"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QProcess>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtCore/QUrl>
#include <QtGui/QFontDatabase>
#include <QtGui/QScreen>

#ifdef __APPLE__
#include <QtGui/QFileOpenEvent>
#endif

#include <iostream>

using namespace std;

// Informational-task chrome skip (--help / --sysinfo / --version): set by
// main.cpp's pre-scan through skipStartupChromeForInformationalTask() before
// the OmGuiApplication below is constructed. File-scope static rather than a
// constructor argument so the QApplication(argc, argv) call signature (which
// must receive the ORIGINAL argc by reference, see the comment below) stays
// untouched.
static bool gSkipStartupChrome = false;

void OmGuiApplication::skipStartupChromeForInformationalTask() {
  gSkipStartupChrome = true;
}

// QApplication needs the reference to the original argc directly to run properly.
// Otherwise, bugs related with threads (splashscreen, tooltips, ...) can appear.
// This was observed on Linux 64
// cf:
// - http://lists-archives.org/kde-devel/20232-qt-4-5-related-crash-on-kdm-startup.html
// - http://www.qtcentre.org/archive/index.php/t-28785.html
OmGuiApplication::OmGuiApplication(int &argc, char **argv) :
  QApplication(argc, argv),
  mMainWindow(NULL),
  mNoWindowMode(false),
  mHeartbeat(0),
  mTask(NORMAL),
  mTcpServer(NULL) {
  setApplicationName("OmniSim");
  setApplicationVersion(OmApplicationInfo::omniSimVersion());
  setOrganizationName("OmniLink");
  setOrganizationDomain("omnilink-agents.com");
#ifdef _WIN32
  // Skipped for --help/--sysinfo/--version (gSkipStartupChrome): these print and
  // exit, so widget style, application font and stylesheet are dead weight on
  // their critical path -- expected 20-60 ms of the 0.36 s --version baseline
  // (main.cpp's Newton-preload pre-scan comment); the parent A/B measures the
  // real figure. GUI launches never set the flag, so they are byte-identical.
  if (!gSkipStartupChrome)
    QApplication::setStyle("windowsvista");
#endif

  mApplication = new OmApplication();  // creates OmApplication singleton
  connect(mApplication, &OmApplication::createWorldLoadingProgressDialog, this, &OmGuiApplication::closeSplashScreenIfNeeded);

  QDir::addSearchPath("icons", OmStandardPaths::resourcesPath() + "nodes/icons");
  QDir::addSearchPath("images", OmStandardPaths::resourcesPath() + "images");

  if (!gSkipStartupChrome) {
    QFontDatabase::addApplicationFont(OmStandardPaths::fontsPath() + "Raleway-Light.ttf");

    // setup the stylesheet for the application
    updateStyleSheet();
  }

  // Qt has its own arguments, see Qt doc
  mShouldMinimize = false;
  mShouldRunBackground = false;
  mShouldStartFullscreen = false;
  mStartupMode = OmSimulationState::NONE;
  mShouldDoRendering = true;

  parseArguments();
}

OmGuiApplication::~OmGuiApplication() {
  delete mMainWindow;
}

void OmGuiApplication::restart() {
  if (mMainWindow)
    mMainWindow->close();
  else
    qApp->quit();
  QStringList nonProgramArgs = qApp->arguments();
  nonProgramArgs.removeFirst();
#ifdef __linux__
  QProcess::startDetached("./webots", nonProgramArgs);
#elif defined(_WIN32)
  exit(3030);  // this special code tells the launcher to restart Webots, see launcher.c
#else  // macOS
  QProcess::startDetached(qApp->arguments()[0], nonProgramArgs);
#endif
}

void OmGuiApplication::commandLineError(const QString &message, bool fatal) {
  cerr << "omnisim: " << message.toUtf8().constData() << endl;
  cerr << tr("Try 'omnisim --help' for more information.").toUtf8().constData() << endl;
  // ...AND to the log file, because on Windows `omnisim-bin.exe` is a
  // GUI-SUBSYSTEM binary: it has no console, so everything written to cerr
  // above goes nowhere at all. A fatal startup error therefore killed the
  // process with exit code 1 and left NO trace anywhere a caller could read.
  //
  // Measured 2026-08-02: repeated headless runs began failing with rc=1, an
  // empty log and no message. The cause was the TCP port scan exhausting its
  // 11-slot range [1234-1244] because earlier engines had been KILLED rather
  // than allowed to exit and were still holding their ports -- a diagnosis
  // that took hours precisely because the engine's own explanation was
  // discarded. A batch caller reads "the run produced nothing" and blames
  // whatever it was measuring.
  OmLog::error(QString("omnisim: %1").arg(message), false, OmLog::ODE);
  if (fatal)
    mTask = FAILURE;
}

void OmGuiApplication::parseArguments() {
  // faster when copied according to Qt's doc
  QStringList args = arguments();
  bool logPerformanceMode = false, batch = false;
  int port = 1234;  // default value
  mStream = '\0';

  const int size = args.size();
  for (int i = 1; i < size; ++i) {
    const QString &arg = args[i];
    if (arg == "--minimize")
      mShouldMinimize = true;
    else if (arg == "--no-window") {
      // True background mode: window is never shown, no taskbar entry.
      // Implies --minimize (no splash), --no-rendering, --batch.
      mShouldRunBackground = true;
      mShouldMinimize = true;
      mShouldDoRendering = false;
      // Record the CLI intent immediately (setup() may later flip
      // mShouldDoRendering from preferences, so mPerformRendering alone
      // cannot distinguish "launched headless" from "user toggled rendering
      // off"). Consumed by the headless texture-decode gate in OmImageTexture.
      OmSimulationState::instance()->setStartedWithoutRendering(true);
      batch = true;
      OmMessageBox::disable();
    } else if (arg == "--fullscreen")
      mShouldStartFullscreen = true;
    else if (arg == "--mode=stop") {
      commandLineError(tr("'--mode=stop' is deprecated."), false);
      mStartupMode = OmSimulationState::PAUSE;
    } else if (arg == "--mode=pause")
      mStartupMode = OmSimulationState::PAUSE;
    else if (arg == "--mode=realtime")
      mStartupMode = OmSimulationState::REALTIME;
    else if (arg == "--mode=fast")
      mStartupMode = OmSimulationState::FAST;
    else if (arg == "--mode=run") {
      commandLineError(tr("`--mode=run` is deprecated, falling back to `fast` mode."), false);
      mStartupMode = OmSimulationState::FAST;
    } else if (arg == "--no-rendering") {
      mShouldDoRendering = false;
      // CLI intent, captured at parse time -- see the --no-window branch above.
      OmSimulationState::instance()->setStartedWithoutRendering(true);
    } else if (arg == "convert") {
      mTask = CONVERT;
      mTaskArguments = args.mid(i);
      break;
    } else if (arg == "--help")
      mTask = HELP;
    else if (arg == "--sysinfo")
      mTask = SYSINFO;
    else if (arg == "--version")
      mTask = VERSION;
    else if (arg == "--batch") {
      batch = true;
      OmMessageBox::disable();
    } else if (arg.startsWith("--update-world"))
      mTask = UPDATE_WORLD;
    else if (arg.startsWith("--port")) {
      int index = arg.indexOf('=');
      if (index == -1)
        commandLineError(tr("omnisim: missing '=' sign right after --port option").arg(arg));
      else {
        port = arg.mid(index + 1).toInt();
        if (port < 1 || port > 65535) {
          commandLineError(tr("omnisim: port value %1 out of range [1;65535], reverting to 1234 default value").arg(port));
          port = 1234;
        }
      }
    } else if (arg == "--stream") {
      mStream = 'w';  // w3d is the default mode
    } else if (arg.startsWith("--stream=")) {
      const QString mode = arg.mid(arg.indexOf('=') + 1);
      if (mode != "w3d" && mode != "mjpeg")
        commandLineError(tr("invalid value \"%1\" to '--stream' option.").arg(mode));
      else
        mStream = mode[0].toLatin1();
    } else if (arg == "--extern-urls")
      OmWorld::setPrintExternUrls();
    else if (arg == "--heartbeat")
      mHeartbeat = startTimer(1000);
    else if (arg.startsWith("--heartbeat=")) {
      bool ok;
      const int value = arg.mid(arg.indexOf('=') + 1).toInt(&ok);
      if (ok)
        mHeartbeat = startTimer(value);
      else
        commandLineError(tr("invalid value \"%1\" to '--heartbeat' option.").arg(arg.mid(arg.indexOf('=') + 1)));
    } else if (arg == "--stdout")
      OmLog::enableStdOutRedirectToTerminal();
    else if (arg == "--stderr")
      OmLog::enableStdErrRedirectToTerminal();
    else if (arg == "--log-performance")
      commandLineError(tr("invalid '--log-performance' option: log file path is missing."), false);
    else if (arg.startsWith("--log-performance=")) {
      QString logArgument = arg.mid(arg.indexOf('=') + 1);
      // remove starting/trailing double quotes
      if (logArgument.startsWith('"'))
        logArgument = logArgument.right(logArgument.size() - 1);
      if (logArgument.endsWith('"'))
        logArgument = logArgument.left(logArgument.size() - 1);
      if (logArgument.contains(",")) {
        QStringList argumentsList = logArgument.split(",");
        OmPerformanceLog::createInstance(argumentsList[0], argumentsList[1].trimmed().toInt());
      } else
        OmPerformanceLog::createInstance(logArgument);
      logPerformanceMode = true;
    } else if (arg == "--clear-cache")
      OmNetwork::instance()->clearCache();
    else if (arg.startsWith("-")) {
      commandLineError(tr("invalid option: '%1'").arg(arg));
    } else {
      if (mStartWorldName.isEmpty())
        mStartWorldName = QDir::fromNativeSeparators(arg);
      else
        commandLineError(tr("too many arguments."));
    }
  }
  if (mStream == '\0')  // we need a simple streaming server for robot windows and remote controllers
    mTcpServer = new OmTcpServer(mStream);
  else {
    if (!batch)
      commandLineError(tr("you should also use --batch (in addition to --stream) for production."), false);
    if (mStream == 'm')
      mTcpServer = new OmMultimediaStreamingServer();
    else {  // w3d
      mTcpServer = new OmW3dStreamingServer();
      OmWorld::enableW3dStreaming();
    }
  }
  mTcpServer->start(port);
  if (mTcpServer->port() == -1)
    commandLineError(tr("failed to open TCP server in the port range starting at %1. Every port in the scan range is "
                        "held by a process that has not released it -- most often simulator instances that were killed "
                        "rather than closed. See the preceding log line for the range and the socket error.\n")
                       .arg(port));

  // create the OmniSim temporary path based on the TCP port early in the process
  // in order to be sure that the Qt internal files will be stored at the right place
  else if (!OmStandardPaths::webotsTmpPathCreate(mTcpServer->port()))
    commandLineError(tr("failed to create the OmniSim temporary path \"%1\".\n").arg(OmStandardPaths::webotsTmpPath()));

  if (logPerformanceMode) {
    OmPerformanceLog::enableSystemInfoLog(mTask == SYSINFO);
    mTask = NORMAL;
  }

  // OMNISIM_SAFE_MODE is preferred; WEBOTS_SAFE_MODE is the legacy alias.
  if (qEnvironmentVariableIsSet("OMNISIM_SAFE_MODE") ? OmPreferences::booleanEnvironmentVariable("OMNISIM_SAFE_MODE") :
                                                       OmPreferences::booleanEnvironmentVariable("WEBOTS_SAFE_MODE")) {
    OmPreferences::instance()->setValue("OpenGL/disableShadows", true);
    OmPreferences::instance()->setValue("OpenGL/disableAntiAliasing", true);
    OmPreferences::instance()->setValue("OpenGL/GTAO", 0);
    OmPreferences::instance()->setValue("OpenGL/textureQuality", 0);
    OmPreferences::instance()->setValue("OpenGL/textureFiltering", 0);
    mStartupMode = OmSimulationState::PAUSE;
    mStartWorldName =
      OmWorldFileFormat::resolveExisting(OmStandardPaths::resourcesPath() + "projects/worlds/empty" +
                                         OmWorldFileFormat::writeExtension());
  }
}

int OmGuiApplication::exec() {
  if (mTask == NORMAL || mTask == UPDATE_WORLD) {
    if (mTask == UPDATE_WORLD)
      connect(mApplication, &OmApplication::worldLoadCompleted, this, &OmGuiApplication::taskExecutor);

    if (setup()) {
      QApplication::processEvents();
      loadInitialWorld();
    }
  }

  // with the addition of EXTERNPROTO, the load of a world is not linear anymore and takes two passes hence the task must be
  // invoked only when the loading effectively takes place
  const OmSingleTaskApplication *task = NULL;
  if (mTask != NORMAL && mTask != UPDATE_WORLD)
    task = taskExecutor();

  const int status = QApplication::exec();
  delete task;
  return status;
}

const OmSingleTaskApplication *OmGuiApplication::taskExecutor() {
  assert(mTask != NORMAL);

  if (mTask == UPDATE_WORLD)
    disconnect(mApplication, &OmApplication::worldLoadCompleted, this, &OmGuiApplication::taskExecutor);

  const OmSingleTaskApplication *task = new OmSingleTaskApplication(mTask, mTaskArguments, this, mApplication->startupPath());
  if (mMainWindow)
    connect(task, &OmSingleTaskApplication::finished, mMainWindow, &OmMainWindow::close);
  else
    connect(task, &OmSingleTaskApplication::finished, this, &QApplication::exit);
  // run the task from the application event loop
  QTimer::singleShot(0, task, SLOT(run()));

  return task;
}

bool OmGuiApplication::setup() {
  OmPreferences *const prefs = OmPreferences::instance();
  if (mStartupMode == OmSimulationState::NONE)
    mStartupMode = startupModeFromPreferences();
  if (mShouldDoRendering)
    mShouldDoRendering = renderingFromPreferences();

  OmSimulationState::instance()->setMode(mStartupMode);
  OmSimulationState::instance()->setRendering(mShouldDoRendering);

  // check specified world file if any
  if (!mStartWorldName.isEmpty()) {
    // if relative, make absolute
    if (QDir::isRelativePath(mStartWorldName))
      mStartWorldName = mApplication->startupPath() + '/' + mStartWorldName;

    QFileInfo info(mStartWorldName);
    if (!info.isReadable())
      commandLineError(tr("could not open file: '%1'.").arg(mStartWorldName));
  }

  // No-window headless mode (Phase Q1 first slice) / compute-only mode (Tier C spike):
  // skip the entire widget tree. Placed after the mode/rendering flags and the world-file
  // check so those behave identically, and before any dialog/splash/window construction.
  if (qEnvironmentVariableIsSet("OMNISIM_NO_WINDOW") || qEnvironmentVariableIsSet("OMNISIM_NO_GL")) {
    mNoWindowMode = true;
    // Same intent as the --no-window CLI flag: this process will never show a
    // main view, so the headless texture-decode gate may engage (camera-family
    // devices still render offscreen -- the world scan in OmSimulationWorld
    // keeps textures for any world that contains one).
    OmSimulationState::instance()->setStartedWithoutRendering(true);
    return setupNoWindow();
  }

  if (OmMessageBox::enabled() && !OmPreferences::instance()->contains("General/theme")) {
    if (OmNewVersionDialog::run() != QDialog::Accepted) {
      mTask = QUIT;
      return false;
    } else if (OmPreferences::instance()->value("General/theme").toString() != mThemeLoaded)
      updateStyleSheet();
  }

  // Show guided tour if first ever launch and no command line world argument is given
  bool showGuidedTour =
    prefs->value("Internal/firstLaunch", true).toBool() && mStartWorldName.isEmpty() && OmMessageBox::enabled();

#ifndef _WIN32
  // create main window on Linux and macOS before the splash screen otherwise, the
  // image in the splash screen is empty...
  // Doing the same on Windows slows down the popup of the SplashScreen, therefore
  // the main window is created later on Windows.
  mMainWindow = new OmMainWindow(mShouldMinimize, mTcpServer, NULL, mShouldRunBackground);
#endif

  if (!mShouldMinimize) {
    // splash screen
    // Warning: using heap allocated splash screen and/or pixmap cause crash while
    // showing tooltips in the main window under Linux.
    mSplash = new OmSplashScreen();
    if (OmPreferences::instance()->value("MainWindow/maximized", false).toBool()) {
      // we need to center the splash screen on the same window as the mainWindow,
      // which is positioned wherever the mouse is on launch
      const QScreen *mainWindowScreen = QGuiApplication::screenAt(QCursor::pos());
      const QRect mainWindowScreenRect = mainWindowScreen->geometry();
      QPoint targetPosition = mainWindowScreenRect.center();
      targetPosition.setX(targetPosition.x() - mSplash->width() / 2);
      targetPosition.setY(targetPosition.y() - mSplash->height() / 2);
      mSplash->move(targetPosition);
    }

    // now we can safely show the splash screen, knowing it will be in the right place
    mSplash->show();
#ifdef __APPLE__
    // On macOS, when the OmSplashScreen is shown, Qt calls a resize event on the QMainWindow (not shown yet) with the size of
    // the splash screen. This overrides the OmMainWindow size preferences. This sounds like a Qt bug.
    mMainWindow->restorePreferredGeometry(mShouldMinimize);
#endif
    connect(OmLog::instance(), &OmLog::popupOpen, mSplash, &QSplashScreen::hide);
    connect(OmLog::instance(), &OmLog::popupClosed, mSplash, &QSplashScreen::show);
    processEvents();
    setSplashMessage(tr("Starting up..."));
  } else
    mSplash = NULL;
  // otherwise get it from the list of recent files
  if (mStartWorldName.isEmpty() || showGuidedTour)
    mStartWorldName =
      OmWorldFileFormat::resolveExisting(prefs->value("RecentFiles/file0", OmProject::newWorldPath()).toString());

  setSplashMessage(tr("Loading world..."));

#ifdef __APPLE__
  /**
   * Fixed the floating docks which are not rendered (gray panel)
   * This is a know issue between Qt 5.0.0 and 5.0.2
   * Hopefully this can be removed later.
   * cf: https://bugreports.qt-project.org/browse/QTBUG-30655
   **/
  setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
#endif

#ifdef _WIN32
  // create main window
  mMainWindow = new OmMainWindow(mShouldMinimize, mTcpServer, NULL, mShouldRunBackground);
#endif

  if (mShouldRunBackground) {
    // True background mode: never realize the main window as a top-level OS
    // window — no taskbar entry on Windows, no dock icon on macOS. The widget
    // tree still exists so child WREN/Qt subsystems initialize normally.
  } else if (mShouldMinimize)
    mMainWindow->showMinimized();
  else {
    if (prefs->value("MainWindow/maximized", false).toBool())
      mMainWindow->showMaximized();
    else
      mMainWindow->showNormal();
  }

  connect(mMainWindow, &OmMainWindow::restartRequested, this, &OmGuiApplication::restart);
  connect(mMainWindow, &OmMainWindow::splashScreenCloseRequested, this, &OmGuiApplication::closeSplashScreenIfNeeded);
  mApplication->setup();

#ifdef __linux__
  // popup a warning message if the preferences file is not writable
  prefs->checkIsWritable();
  if (OmSysInfo::isRootUser() && OmSysInfo::environmentVariable("CI").isEmpty())
    OmLog::warning("It is not recommended to run OmniSim as root.");

  if (prefs->value("Internal/firstLaunch", true).toBool()) {
    // Delete previous desktop application info files so they are regenerated from the
    // current installation data. "webots.desktop" is the pre-rebrand name -- purge it
    // too, or an upgrading user keeps a stale duplicate entry in their app menu.
    const QString applicationsDir = QDir::homePath() + "/.local/share/applications/";
    for (const QString &name : {QStringLiteral("omnisim.desktop"), QStringLiteral("webots.desktop")}) {
      const QString desktopFilePath = applicationsDir + name;
      if (QFile::exists(desktopFilePath))
        QFile::remove(desktopFilePath);
    }
  }
#endif

  // GL-less degrade (shipped since D1.5): initializeOpenGlInfo() calls glad function pointers, which are
  // NULL when no GL context was ever made current (a null CALL, not garbage). Inert when GL is
  // present (the shipped default).
  if (OmWrenOpenGlContext::isInitialized()) {
    OmWrenOpenGlContext::makeWrenCurrent();
    OmSysInfo::initializeOpenGlInfo();
    OmWrenOpenGlContext::doneWren();
  }

  if (showGuidedTour)
    mMainWindow->showUpdatedDialog();  // the guided tour will be shown after the updated dialog

  return true;
}

// The widget-free counterpart of setup() (OMNISIM_NO_WINDOW=1). No OmMainWindow, no
// OmSimulationView, no OmView3D, no splash, no dialogs, no actions/toolbars/icons.
// D1.4 (WREN deletion): the headless WREN window + QOffscreenSurface GL bring-up is GONE --
// this mode is now the OMNISIM_NO_GL shape for rendering too, except that camera-family
// devices set up WRENLESS and render their images through the wgpu offscreen path (no GL
// context needed), which is what keeps OMNISIM_NO_WINDOW camera worlds producing images.
// Supervisor labels are state-only and keep working; the main view never renders here.
bool OmGuiApplication::setupNoWindow() {
  mSplash = NULL;

  const bool noGl = qEnvironmentVariableIsSet("OMNISIM_NO_GL");

  // The kept "gl:" assets (gizmo meshes, HUD icons, muscle.png) resolve through this Qt
  // search path, normally registered by the OmView3D constructor -- which this mode skips.
  QDir::addSearchPath("gl", OmStandardPaths::resourcesPath() + "wren");

  // The rendering context singleton is normally seeded by OmView3D::initialize(); nodes
  // (e.g. OmSolid) connect to it at build time, so it must exist before any world loads.
  OmWrenRenderingContext::setWrenRenderingContext(160, 120);

  // On reload, stale supervisor labels belong to the torn-down world: drop them.
  connect(mApplication, &OmApplication::preWorldLoaded, this,
          [](bool /*reloading*/) { OmWrenLabelOverlay::removeAllLabels(); });
  connect(mApplication, &OmApplication::postWorldLoaded, this, [noGl](bool /*reloading*/, bool /*firstLoad*/) {
    OmLog::info(noGl ? tr("No-GL mode: world loaded (compute-only).") : tr("No-window mode: world loaded."));
  });
  connect(mApplication, &OmApplication::simulationQuitRequested, this,
          [](int exitStatus) { QApplication::exit(exitStatus); });

  mApplication->setup();

  if (noGl)
    OmLog::info(tr("Compute-only headless mode (OMNISIM_NO_GL): no window, no GL context. "
                   "Rendering devices (cameras, lidars, displays) will produce NO data in this mode."));
  else
    OmLog::info(tr("No-window headless mode (OMNISIM_NO_WINDOW): GUI layer skipped; rendering devices "
                   "render through the wgpu offscreen path."));
  return true;
}

OmSimulationState::Mode OmGuiApplication::startupModeFromPreferences() const {
  OmPreferences *const prefs = OmPreferences::instance();
  const QString startupMode(prefs->value("General/startupMode").toString());

  if (startupMode == "Real-time")
    return OmSimulationState::REALTIME;
  if (startupMode == "Fast")
    return OmSimulationState::FAST;
  return OmSimulationState::PAUSE;
}

bool OmGuiApplication::renderingFromPreferences() const {
  OmPreferences *const prefs = OmPreferences::instance();
  return prefs->value("General/rendering", true).toBool();
}

#ifdef __APPLE__
// Fixed the OmniSim opening by double-clicking on .wbt file in the Finder
bool OmGuiApplication::event(QEvent *event) {
  switch (event->type()) {
    case QEvent::FileOpen:
      mStartWorldName = static_cast<QFileOpenEvent *>(event)->file();
      return true;
    default:
      return QApplication::event(event);
  }
}
#endif

void OmGuiApplication::setSplashMessage(const QString &message) {
  if (!mSplash)
    return;

  // Brand line on the splash: OmniSim is the product. The upstream Webots
  // attribution is kept (factual, and mirrors the NOTICE file) but is secondary;
  // the About box carries the full attribution.
  QString copyright = tr("\u00A9 OmniLink \u00B7 built on Webots \u00A9 Cyberbotics Ltd. \u00B7 Apache-2.0");
  mSplash->setLiveMessage(copyright + "\n" + message);
  mSplash->repaint();
}

void OmGuiApplication::closeSplashScreenIfNeeded() {
  if (mSplash) {
    mSplash->finish(mMainWindow);
    delete mSplash;
    mSplash = NULL;
  }
}

void OmGuiApplication::loadInitialWorld() {
  if (mNoWindowMode) {
    // no-window mode: load through the engine directly -- no save prompt (nothing to
    // save), no GUI pre/post hooks (our setupNoWindow() lambdas cover the essentials)
    if (!mApplication->isValidWorldFileName(mStartWorldName)) {
      OmLog::error(tr("Invalid world file: '%1'.").arg(mStartWorldName));
      QApplication::exit(1);
      return;
    }
    mApplication->loadWorld(mStartWorldName, false);
    return;
  }

  if (!mShouldMinimize && mShouldStartFullscreen)
    mMainWindow->setFullScreen(true, false, false, true);

  mMainWindow->loadWorld(mStartWorldName);
}

#ifdef _WIN32
static bool windowsDarkMode = false;

enum PreferredAppMode { Default, AllowDark, ForceDark, ForceLight, Max };

enum WINDOWCOMPOSITIONATTRIB {
  WCA_UNDEFINED = 0,
  WCA_NCRENDERING_ENABLED = 1,
  WCA_NCRENDERING_POLICY = 2,
  WCA_TRANSITIONS_FORCEDISABLED = 3,
  WCA_ALLOW_NCPAINT = 4,
  WCA_CAPTION_BUTTON_BOUNDS = 5,
  WCA_NONCLIENT_RTL_LAYOUT = 6,
  WCA_FORCE_ICONIC_REPRESENTATION = 7,
  WCA_EXTENDED_FRAME_BOUNDS = 8,
  WCA_HAS_ICONIC_BITMAP = 9,
  WCA_THEME_ATTRIBUTES = 10,
  WCA_NCRENDERING_EXILED = 11,
  WCA_NCADORNMENTINFO = 12,
  WCA_EXCLUDED_FROM_LIVEPREVIEW = 13,
  WCA_VIDEO_OVERLAY_ACTIVE = 14,
  WCA_FORCE_ACTIVEWINDOW_APPEARANCE = 15,
  WCA_DISALLOW_PEEK = 16,
  WCA_CLOAK = 17,
  WCA_CLOAKED = 18,
  WCA_ACCENT_POLICY = 19,
  WCA_FREEZE_REPRESENTATION = 20,
  WCA_EVER_UNCLOAKED = 21,
  WCA_VISUAL_OWNER = 22,
  WCA_HOLOGRAPHIC = 23,
  WCA_EXCLUDED_FROM_DDA = 24,
  WCA_PASSIVEUPDATEMODE = 25,
  WCA_USEDARKMODECOLORS = 26,
  WCA_LAST = 27
};

struct WINDOWCOMPOSITIONATTRIBDATA {
  // cppcheck-suppress unusedStructMember
  WINDOWCOMPOSITIONATTRIB Attrib;
  PVOID pvData;
  SIZE_T cbData;
};

using fnAllowDarkModeForWindow = BOOL(WINAPI *)(HWND hWnd, BOOL allow);
using fnSetPreferredAppMode = PreferredAppMode(WINAPI *)(PreferredAppMode appMode);
using fnSetWindowCompositionAttribute = BOOL(WINAPI *)(HWND hwnd, WINDOWCOMPOSITIONATTRIBDATA *);

static void setDarkTitlebar(HWND hwnd) {
  static fnAllowDarkModeForWindow AllowDarkModeForWindow = NULL;
  static fnSetWindowCompositionAttribute SetWindowCompositionAttribute = NULL;
  if (!AllowDarkModeForWindow) {  // first call
    HMODULE hUxtheme = LoadLibraryExW(L"uxtheme.dll", NULL, LOAD_LIBRARY_SEARCH_SYSTEM32);
    HMODULE hUser32 = GetModuleHandleW(L"user32.dll");
    AllowDarkModeForWindow = reinterpret_cast<fnAllowDarkModeForWindow>(GetProcAddress(hUxtheme, MAKEINTRESOURCEA(133)));
    SetWindowCompositionAttribute =
      reinterpret_cast<fnSetWindowCompositionAttribute>(GetProcAddress(hUser32, "SetWindowCompositionAttribute"));
    fnSetPreferredAppMode SetPreferredAppMode =
      reinterpret_cast<fnSetPreferredAppMode>(GetProcAddress(hUxtheme, MAKEINTRESOURCEA(135)));
    SetPreferredAppMode(AllowDark);
  }
  BOOL dark = TRUE;
  AllowDarkModeForWindow(hwnd, dark);
  WINDOWCOMPOSITIONATTRIBDATA data = {WCA_USEDARKMODECOLORS, &dark, sizeof(dark)};
  SetWindowCompositionAttribute(hwnd, &data);
}
#endif  // _WIN32

void OmGuiApplication::updateStyleSheet() {
  mThemeLoaded = OmPreferences::instance()->value("General/theme").toString();
  QFile qssFile(OmStandardPaths::resourcesPath() + mThemeLoaded);
  QString styleSheet;
  if (qssFile.open(QFile::ReadOnly))
    styleSheet = QString::fromUtf8(qssFile.readAll());
  else
    OmLog::warning(tr("Could not open theme file: '%1'.").arg(qssFile.fileName()));

#ifdef __APPLE__
  const QString platformStylesheet = "stylesheet.macos.qss";
#elif defined(__linux__)
  const QString platformStylesheet = "stylesheet.linux.qss";
#elif _WIN32
  const QString platformStylesheet = "stylesheet.windows.qss";
#endif

  QFile platformQssFile(OmStandardPaths::resourcesPath() + platformStylesheet);
  if (platformQssFile.open(QFile::ReadOnly))
    styleSheet += QString::fromUtf8(platformQssFile.readAll());
  else
    OmLog::warning(tr("Could not open stylesheet file: '%1'.").arg(platformQssFile.fileName()));

  qApp->setStyleSheet(styleSheet);
#ifdef _WIN32
  if (mThemeLoaded != "omnisim_classic.qss")
    windowsDarkMode = true;
#endif
}

void OmGuiApplication::setWindowsDarkMode(QWidget *window) {
#ifdef _WIN32
  if (windowsDarkMode)
    setDarkTitlebar(reinterpret_cast<HWND>(window->winId()));
#endif
}

void OmGuiApplication::timerEvent(QTimerEvent *event) {
  if (event->timerId() == mHeartbeat)
    cout << "." << endl;
}
