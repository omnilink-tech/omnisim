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

#include "OmMainWindow.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmAboutBox.hpp"
#include "OmActionManager.hpp"
#include "OmAnimationRecorder.hpp"
#include "OmApplication.hpp"
#include "OmApplicationInfo.hpp"
#include "OmAgentHud.hpp"
#include "OmClipboard.hpp"
#include "OmConsole.hpp"
#include "OmContextMenuGenerator.hpp"
#include "OmControlledWorld.hpp"
#include "OmDesktopServices.hpp"
#include "OmDockWidget.hpp"
#include "OmFileUtil.hpp"
#include "OmGuiApplication.hpp"
#include "OmGuidedTour.hpp"
#include "OmJoystickInterface.hpp"
#include "OmMessageBox.hpp"
#include "OmMultimediaStreamingServer.hpp"
#include "OmNewControllerWizard.hpp"
#include "OmNewProjectWizard.hpp"
#include "OmNewProtoWizard.hpp"
#include "OmNewWorldWizard.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmOpenSampleWorldDialog.hpp"
#include "OmPerformanceLog.hpp"
#include "OmPerspective.hpp"
#include "OmPreferences.hpp"
#include "OmPreferencesDialog.hpp"
#include "OmProject.hpp"
#include "OmProjectRelocationDialog.hpp"
#include "OmProtoManager.hpp"
#include "OmRecentFilesList.hpp"
#include "OmRenderingDevice.hpp"
#include "OmRenderingDeviceWindowFactory.hpp"
#include "OmRobot.hpp"
#include "OmRobotWindow.hpp"
#include "OmSaveWarningDialog.hpp"
#include "OmSceneTree.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSimulationView.hpp"
#include "OmSimulationWorld.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"
#include "OmTcpServer.hpp"
#include "OmTemplateManager.hpp"
#include "OmUpdatedDialog.hpp"
#include "OmVideoRecorder.hpp"
#include "OmView3D.hpp"
#include "OmVisualBoundingSphere.hpp"
#include "OmVulkanBackend.hpp"

#include <QtCore/QDir>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QUrl>

#include <QtNetwork/QHostInfo>

#include <QtGui/QActionGroup>
#include <QtGui/QCloseEvent>
#include <QtGui/QScreen>
#include <QtGui/QWindow>
#include <QtNetwork/QHttpMultiPart>
#include <QtNetwork/QNetworkReply>
#include <QtOpenGL/QOpenGLFunctions_3_3_Core>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressDialog>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QStyle>

OmMainWindow::OmMainWindow(bool minimizedOnStart, OmTcpServer *tcpServer, QWidget *parent, bool runBackground) :
  QMainWindow(parent),
  mExitStatus(0),
  mRunBackground(runBackground),
  mAgentHud(NULL),
  mSimulationView(NULL),
  mRecentFiles(NULL),
  mOverlayMenu(NULL),
  mWorldLoadingProgressDialog(NULL),
  mIsFullScreenLocked(false),
  mMaximizedWidget(NULL),
  mTcpServer(tcpServer) {
#ifdef __APPLE__
  // This flag is required to hide a second and useless title bar.
  setUnifiedTitleAndToolBarOnMac(true);
#endif
  setObjectName("MainWindow");
  QStatusBar *statusBar = new QStatusBar(this);
  statusBar->showMessage(tr("Welcome to OmniSim — open-source robot simulator, built on Webots, for the OmniLink agentic platform."));
  setStatusBar(statusBar);

  OmGuiApplication::setWindowsDarkMode(this);
  style()->polish(this);
  QDir::addSearchPath("enabledIcons", OmStandardPaths::resourcesPath() + enabledIconPath());
  QDir::addSearchPath("disabledIcons", OmStandardPaths::resourcesPath() + disabledIconPath());
  QDir::addSearchPath("coreIcons", OmStandardPaths::resourcesPath() + coreIconPath());
  style()->polish(this);

  QApplication::setWindowIcon(QIcon("coreIcons:omnisim.png"));

  // listen to the application
  connect(OmApplication::instance(), &OmApplication::preWorldLoaded, this, &OmMainWindow::updateBeforeWorldLoading);
  connect(OmApplication::instance(), &OmApplication::postWorldLoaded, this, &OmMainWindow::updateAfterWorldLoading);
  connect(OmApplication::instance(), &OmApplication::createWorldLoadingProgressDialog, this,
          &OmMainWindow::createWorldLoadingProgressDialog);
  connect(OmApplication::instance(), &OmApplication::deleteWorldLoadingProgressDialog, this,
          &OmMainWindow::deleteWorldLoadingProgressDialog);
  connect(OmApplication::instance(), &OmApplication::worldLoadingHasProgressed, this, &OmMainWindow::setWorldLoadingProgress);
  connect(OmApplication::instance(), &OmApplication::worldLoadingStatusHasChanged, this, &OmMainWindow::setWorldLoadingStatus);

  connect(OmSimulationState::instance(), &OmSimulationState::enabledChanged, this, &OmMainWindow::simulationEnabledChanged);

  // listen to log
  connect(OmLog::instance(), &OmLog::logEmitted, this, &OmMainWindow::showStatusBarMessage);

  // world reload or simulation quit should not be executed directly (Qt::QueuedConnection)
  // because it is call in a OmniSim state where events have to be solved
  // (typically packets comming from libController)
  // applying the reload or quit directly may imply a OmniSim crash
  connect(OmApplication::instance(), &OmApplication::worldReloadRequested, this, &OmMainWindow::reloadWorld,
          Qt::QueuedConnection);
  connect(OmApplication::instance(), &OmApplication::simulationResetRequested, this, &OmMainWindow::resetGui,
          Qt::QueuedConnection);
  connect(OmApplication::instance(), &OmApplication::simulationQuitRequested, this, &OmMainWindow::simulationQuit,
          Qt::QueuedConnection);
  connect(OmApplication::instance(), &OmApplication::worldLoadRequested, this, &OmMainWindow::loadDifferentWorld,
          Qt::QueuedConnection);

  createMainTools();
  createMenus();

  OmActionManager *actionManager = OmActionManager::instance();
  QAction *action = actionManager->action(OmAction::SHOW_ROBOT_WINDOW);
  connect(action, &QAction::triggered, this, &OmMainWindow::showRobotWindow);
  addAction(action);

  restorePreferredGeometry(minimizedOnStart);

  mFactoryLayout = new QByteArray(saveState());

  updateWindowTitle();

  // toggling the animation icon
  mAnimationRecordingTimer = new QTimer(this);
  connect(mAnimationRecordingTimer, &QTimer::timeout, this, &OmMainWindow::toggleAnimationIcon);
  toggleAnimationAction(false);

  OmAnimationRecorder *recorder = OmAnimationRecorder::instance();
  connect(recorder, &OmAnimationRecorder::initalizedFromStreamingServer, this, &OmMainWindow::disableAnimationAction);
  connect(recorder, &OmAnimationRecorder::cleanedUpFromStreamingServer, this, &OmMainWindow::enableAnimationAction);
  connect(recorder, &OmAnimationRecorder::requestOpenUrl, this,
          [this](const QString &fileName, const QString &content, const QString &title) {
            openUrl(fileName, content, title);
          });

  OmJoystickInterface::setWindowHandle(winId());

  connect(OmTemplateManager::instance(), &OmTemplateManager::preNodeRegeneration, this, &OmMainWindow::prepareNodeRegeneration);
  connect(OmTemplateManager::instance(), &OmTemplateManager::abortNodeRegeneration, this,
          &OmMainWindow::discardNodeRegeneration);
  connect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this,
          &OmMainWindow::finalizeNodeRegeneration);
}

OmMainWindow::~OmMainWindow() {
  delete mFactoryLayout;
}

void OmMainWindow::lockFullScreen(bool isLocked) {
  mIsFullScreenLocked = isLocked;
}

void OmMainWindow::exitFullScreen() {
  if (mIsFullScreenLocked) {
    // stop video recording
    mSimulationView->movieAction()->trigger();
    mIsFullScreenLocked = false;
  }

  mToggleFullScreenAction->setChecked(false);
}

void OmMainWindow::toggleFullScreen(bool enabled) {
  setFullScreen(enabled);
}

bool OmMainWindow::setFullScreen(bool isEnabled, bool isRecording, bool showDialog, bool startup) {
  static const QString msgEnterFullScreenMode = tr("You are entering fullscreen mode.") + "<br/><br/>";

  static const QString fullscreenEsc = tr("Press ESC to quit fullscreen mode.") + "<br/>";
  static const QString movieEsc =
    "<strong>" + tr("Press ESC to stop recording the movie and quit fullscreen mode.") + "</strong><br/>";

  static const QString ctrlZero = tr("Press <i>Ctrl+0</i> to pause the simulation.") + "<br/>";
  static const QString ctrlOne = tr("Press <i>Ctrl+1</i> to execute one basic time step.") + "<br/>";
  static const QString ctrlTwo = tr("Press <i>Ctrl+2</i> to run the simulation in real time.") + "<br/>";
  static const QString ctrlThree = tr("Press <i>Ctrl+3</i> to run the simulation as fast as possible.") + "<br/>";
  static const QString ctrlFour = tr("Press <i>Ctrl+4</i> to toggle the 3D scene rendering.") + "<br/>" + "<br/>";

  static const QString cmdZero = tr("Press <i>Cmd+0</i> to pause the simulation.") + "<br/>";
  static const QString cmdOne = tr("Press <i>Cmd+1</i> to execute one basic time step.") + "<br/>";
  static const QString cmdTwo = tr("Press <i>Cmd+2</i> to run the simulation in real time.") + "<br/>";
  static const QString cmdThree = tr("Press <i>Cmd+3</i> to run the simulation as fast as possible.") + "<br/>";
  static const QString cmdFour = tr("Press <i>Cmd+4</i> to toggle the 3D scene rendering.") + "<br/>" + "<br/>";

  static const QString ctrlScreenshot =
    tr("Press <i>Ctrl+Shift+P</i> to take a screenshot of the 3D screen.") + "<br/>" + "<br/>";
  static const QString cmdScreenshot =
    tr("Press <i>Cmd+Shift+P</i> to take a screenshot of the 3D screen.") + "<br/>" + "<br/>";
  static const QString screenshotNotes = tr("<b>Note:</b> When taking a screenshot in fullscreen mode, the resulting "
                                            "screenshot's save path will be chosen automatically.") +
                                         "<br/>";

  static QString screenshotPath;
  screenshotPath =
    tr("Screenshots will be saved in <i>%1</i>.").arg(OmPreferences::instance()->value("Directories/screenshots").toString());

  static bool macos = OmSysInfo::platform() == OmSysInfo::MACOS_PLATFORM;
  static const QString &zero = macos ? cmdZero : ctrlZero;
  static const QString &one = macos ? cmdOne : ctrlOne;
  static const QString &two = macos ? cmdTwo : ctrlTwo;
  static const QString &three = macos ? cmdThree : ctrlThree;
  static const QString &four = macos ? cmdFour : ctrlFour;
  static const QString &five = macos ? cmdScreenshot : ctrlScreenshot;

  static QByteArray currentPerspective = *mFactoryLayout;

  if (mIsFullScreenLocked)
    return false;

  if (isEnabled) {
    // store actual window geometry and perspective
    writePreferences();
    currentPerspective = saveState();

    if (showDialog) {
      QString message = msgEnterFullScreenMode;
      if (isRecording)
        message += movieEsc;
      else
        message += fullscreenEsc;
      message += zero;
      message += one;
      message += two;
      message += three;
      message += four;
      message += five;
      message += screenshotNotes;
      message += screenshotPath;
      if (OmMessageBox::question(message, this, tr("Fullscreen mode"), QMessageBox::Ok) == QMessageBox::Cancel) {
        mToggleFullScreenAction->blockSignals(true);
        mToggleFullScreenAction->setChecked(false);
        mToggleFullScreenAction->blockSignals(false);
        return false;
      }
    }

    if (startup) {
      if (!mToggleFullScreenAction->isChecked()) {
        disconnect(mToggleFullScreenAction, &QAction::toggled, this, &OmMainWindow::toggleFullScreen);
        mToggleFullScreenAction->setChecked(true);
        connect(mToggleFullScreenAction, &QAction::toggled, this, &OmMainWindow::toggleFullScreen);
      }
    }

    // hide docks
    for (int i = 0; i < mConsoles.size(); ++i)
      mConsoles.at(i)->hide();
    if (mAgentHud)
      mAgentHud->hide();

    // hide menu bar and status bar
    mMenuBar->hide();
    statusBar()->hide();

    // remove tool bar in OmSimulationView
    mSimulationView->show();
    mSimulationView->setDecorationVisible(false);

    // show main window in fullscreen mode
    showFullScreen();

    // connect exit shortcut
    connect(mExitFullScreenAction, &QAction::triggered, this, &OmMainWindow::exitFullScreen);

  } else {
    // show main window in normal mode
    showNormal();

    // show docks
    for (int i = 0; i < mConsoles.size(); ++i)
      mConsoles.at(i)->show();
    if (mAgentHud)
      mAgentHud->show();

    // show menu bar and status bar
    mMenuBar->show();
    statusBar()->show();

    // show tool bar in OmSimulationView
    mSimulationView->setDecorationVisible(true);

    restorePreferredGeometry();
    restoreState(currentPerspective);

    // disconnect exit shortcut
    disconnect(mExitFullScreenAction, &QAction::triggered, this, &OmMainWindow::exitFullScreen);
  }

  return true;
}

void OmMainWindow::addDock(QWidget *dock) {
  mDockWidgets.append(dock);
  connect(dock, SIGNAL(needsMaximize()), this, SLOT(maximizeDock()));
  connect(dock, SIGNAL(needsMinimize()), this, SLOT(minimizeDock()));
}

void OmMainWindow::createMainTools() {
  // extend Scene Tree to bottom left corner
  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

  mSimulationView = new OmSimulationView(this, toolBarAlign());
  setCentralWidget(mSimulationView);
  addDock(mSimulationView);
  connect(mSimulationView, &OmSimulationView::requestOpenUrl, this, &OmMainWindow::openUrl);
  if (mTcpServer) {
    mTcpServer->setMainWindow(this);
    if (mTcpServer->streamStatus()) {
      OmMultimediaStreamingServer *multimediaStreamingServer = dynamic_cast<OmMultimediaStreamingServer *>(mTcpServer);
      if (multimediaStreamingServer)
        multimediaStreamingServer->setView3D(mSimulationView->view3D());
    }
  }

  // OmniLink agent HUD — live status panel polling a runner's /status
  // endpoint, docked on the right.
  if (OmAgentHud::isEnabled()) {
    mAgentHud = new OmAgentHud(this);
    addDockWidget(Qt::RightDockWidgetArea, mAgentHud, Qt::Vertical);
    addDock(mAgentHud);
  }

  connect(mSimulationView->sceneTree(), &OmSceneTree::documentationRequest, this, &OmMainWindow::showOnlineDocumentation);
  // this instruction does nothing but prevents issues resizing QDockWidgets
  // https://stackoverflow.com/questions/48766663/resize-qdockwidget-without-undocking-and-docking

  connect(OmVideoRecorder::instance(), &OmVideoRecorder::requestOpenUrl, this, &OmMainWindow::openUrl);
}

QMenu *OmMainWindow::createFileMenu() {
  QMenu *menu = new QMenu(this);
  menu->setTitle(tr("&File"));

  QAction *action;
  OmActionManager *manager = OmActionManager::instance();

  QMenu *newMenu = menu->addMenu(tr("New"));

  action = new QAction(this);
  action->setText(tr("New Project &Directory..."));
  action->setStatusTip(tr("Create a new project directory."));
  action->setToolTip(action->statusTip());
  connect(action, &QAction::triggered, this, &OmMainWindow::newProjectDirectory);
  newMenu->addAction(action);

  action = new QAction(this);
  action->setText(tr("&New World File..."));
  action->setStatusTip(tr("Create a new simulation world."));
  action->setToolTip(action->statusTip());
  action->setShortcut(Qt::SHIFT | Qt::CTRL | Qt::Key_N);
  connect(action, &QAction::triggered, this, &OmMainWindow::newWorld);
  newMenu->addAction(action);

  action = new QAction(this);
  action->setText(tr("New Robot &Controller..."));
  action->setStatusTip(tr("Create a new controller program."));
  action->setToolTip(action->statusTip());
  connect(action, &QAction::triggered, this, &OmMainWindow::newRobotController);
  newMenu->addAction(action);

  action = new QAction(this);
  action->setText(tr("New P&ROTO..."));
  action->setStatusTip(tr("Create a new PROTO."));
  action->setToolTip(action->statusTip());
  connect(action, &QAction::triggered, this, &OmMainWindow::newProto);
  newMenu->addAction(action);

  action = manager->action(OmAction::OPEN_WORLD);
  connect(action, &QAction::triggered, this, &OmMainWindow::openWorld);
  menu->addAction(action);

  mRecentFilesSubMenu = menu->addMenu(tr("&Open Recent World"));
  mRecentFiles = new OmRecentFilesList(10, mRecentFilesSubMenu);
  connect(mRecentFiles, &OmRecentFilesList::fileChosen, this, &OmMainWindow::loadDifferentWorld);

  action = manager->action(OmAction::OPEN_SAMPLE_WORLD);
  connect(action, &QAction::triggered, this, &OmMainWindow::openSampleWorld);
  menu->addAction(action);

  action = manager->action(OmAction::SAVE_WORLD);
  connect(action, &QAction::triggered, this, &OmMainWindow::saveWorld);
  menu->addAction(action);

  action = manager->action(OmAction::SAVE_WORLD_AS);
  connect(action, &QAction::triggered, this, &OmMainWindow::saveWorldAs);
  menu->addAction(action);

  action = manager->action(OmAction::RELOAD_WORLD);
  connect(action, &QAction::triggered, this, &OmMainWindow::reloadWorld);
  menu->addAction(action);

  action = manager->action(OmAction::RESET_SIMULATION);
  connect(action, &QAction::triggered, this, &OmMainWindow::resetWorldFromGui);
  menu->addAction(action);

  menu->addSeparator();

  menu->addAction(manager->action(OmAction::TAKE_SCREENSHOT));
  menu->addAction(mSimulationView->movieAction());
  menu->addAction(manager->action(OmAction::ANIMATION));
  connect(manager->action(OmAction::ANIMATION), &QAction::triggered, this, &OmMainWindow::startAnimationRecording);

  menu->addSeparator();

#ifdef _WIN32  // On Windows, applications generally use the "Exit" terminology to terminate.
  const QString terminateWord(tr("Exit"));
#else  // On Linux and macOS, they use "Quit" instead of "Exit".
  const QString terminateWord(tr("Quit"));
#endif

  action = new QAction(terminateWord, this);
  action->setMenuRole(QAction::QuitRole);  // Mac: put the menu respecting the MacOS specifications
  action->setShortcut(Qt::CTRL | Qt::Key_Q);
  action->setStatusTip(tr("Terminate the OmniSim application."));
  action->setToolTip(action->statusTip());
  connect(action, &QAction::triggered, this, &OmMainWindow::close);
  menu->addAction(action);

  return menu;
}

QMenu *OmMainWindow::createEditMenu() {
  QMenu *menu = new QMenu(this);
  menu->setTitle(tr("&Edit"));

  OmActionManager *manager = OmActionManager::instance();
  menu->addAction(manager->action(OmAction::UNDO));
  menu->addAction(manager->action(OmAction::REDO));
  menu->addSeparator();
  menu->addAction(manager->action(OmAction::CUT));
  menu->addAction(manager->action(OmAction::COPY));
  menu->addAction(manager->action(OmAction::PASTE));
  menu->addAction(manager->action(OmAction::SELECT_ALL));
  menu->addSeparator();
  menu->addAction(manager->action(OmAction::FIND));
  menu->addAction(manager->action(OmAction::FIND_NEXT));
  menu->addAction(manager->action(OmAction::FIND_PREVIOUS));
  menu->addAction(manager->action(OmAction::REPLACE));
  menu->addSeparator();
  menu->addAction(manager->action(OmAction::GO_TO_LINE));
  menu->addSeparator();
  menu->addAction(manager->action(OmAction::TOGGLE_LINE_COMMENT));

  return menu;
}

QMenu *OmMainWindow::createViewMenu() {
  QMenu *menu = new QMenu(this);
  QMenu *subMenu;
  menu->setTitle(tr("&View"));

  OmActionManager *actionManager = OmActionManager::instance();
  menu->addAction(actionManager->action(OmAction::RENDERING));
  menu->addSeparator();

  subMenu = menu->addMenu(tr("&Follow Object"));
  subMenu->addAction(actionManager->action(OmAction::FOLLOW_NONE));
  subMenu->addAction(actionManager->action(OmAction::FOLLOW_TRACKING));
  subMenu->addAction(actionManager->action(OmAction::FOLLOW_MOUNTED));
  subMenu->addAction(actionManager->action(OmAction::FOLLOW_PAN_AND_TILT));
  menu->addAction(actionManager->action(OmAction::RESTORE_VIEWPOINT));
  menu->addAction(actionManager->action(OmAction::MOVE_VIEWPOINT_TO_OBJECT));

  subMenu = menu->addMenu(tr("Align View to Object"));
  subMenu->addAction(actionManager->action(OmAction::OBJECT_FRONT_VIEW));
  subMenu->addAction(actionManager->action(OmAction::OBJECT_BACK_VIEW));
  subMenu->addAction(actionManager->action(OmAction::OBJECT_LEFT_VIEW));
  subMenu->addAction(actionManager->action(OmAction::OBJECT_RIGHT_VIEW));
  subMenu->addAction(actionManager->action(OmAction::OBJECT_TOP_VIEW));
  subMenu->addAction(actionManager->action(OmAction::OBJECT_BOTTOM_VIEW));
  menu->addSeparator();

  QIcon icon = QIcon();
  icon.addFile("enabledIcons:front_view.png", QSize(), QIcon::Normal);
  icon.addFile("disabledIcons:front_view.png", QSize(), QIcon::Disabled);
  subMenu = menu->addMenu(icon, tr("Change View"));
  subMenu->addAction(actionManager->action(OmAction::EAST_VIEW));
  subMenu->addAction(actionManager->action(OmAction::WEST_VIEW));
  subMenu->addAction(actionManager->action(OmAction::NORTH_VIEW));
  subMenu->addAction(actionManager->action(OmAction::SOUTH_VIEW));
  subMenu->addAction(actionManager->action(OmAction::TOP_VIEW));
  subMenu->addAction(actionManager->action(OmAction::BOTTOM_VIEW));
  menu->addSeparator();

  mToggleFullScreenAction = new QAction(this);
  mToggleFullScreenAction->setText(tr("&Fullscreen"));
  mToggleFullScreenAction->setStatusTip(tr("Show the simulation view in fullscreen mode."));
  mToggleFullScreenAction->setToolTip(mToggleFullScreenAction->statusTip());
  mToggleFullScreenAction->setShortcut(Qt::CTRL | Qt::SHIFT | Qt::Key_F);
  mToggleFullScreenAction->setCheckable(true);
  connect(mToggleFullScreenAction, &QAction::toggled, this, &OmMainWindow::toggleFullScreen);
  menu->addAction(mToggleFullScreenAction);

  mExitFullScreenAction = new QAction(this);
  mExitFullScreenAction->setCheckable(false);
  mExitFullScreenAction->setShortcut(Qt::Key_Escape);

  // add fullscreen actions to the current widget too
  // otherwise it will be disabled when hiding the menu
  addAction(mToggleFullScreenAction);
  addAction(mExitFullScreenAction);

  menu->addSeparator();
  subMenu = menu->addMenu(tr("&Theme"));
  QActionGroup *themeGroup = new QActionGroup(this);
  themeGroup->setExclusive(true);
  struct ThemeEntry {
    const char *label;
    const char *file;
  };
  const ThemeEntry themes[] = {{QT_TR_NOOP("&Light (Classic)"), "omnisim_classic.qss"},
                               {QT_TR_NOOP("&Dark (Night)"), "omnisim_night.qss"},
                               {QT_TR_NOOP("Dark (D&usk)"), "omnisim_dusk.qss"}};
  const QString currentTheme = OmPreferences::instance()->value("General/theme").toString();
  for (const ThemeEntry &t : themes) {
    QAction *themeAction = new QAction(tr(t.label), this);
    themeAction->setCheckable(true);
    themeAction->setChecked(currentTheme == QLatin1String(t.file));
    const QString file = QString::fromLatin1(t.file);
    connect(themeAction, &QAction::triggered, this, [this, file]() { setTheme(file); });
    themeGroup->addAction(themeAction);
    subMenu->addAction(themeAction);
  }

  menu->addSeparator();
  menu->addAction(actionManager->action(OmAction::PERSPECTIVE_PROJECTION));
  menu->addAction(actionManager->action(OmAction::ORTHOGRAPHIC_PROJECTION));
  menu->addSeparator();
  menu->addAction(actionManager->action(OmAction::PLAIN_RENDERING));
  menu->addAction(actionManager->action(OmAction::WIREFRAME_RENDERING));
  menu->addSeparator();

  subMenu = menu->addMenu(tr("&Optional Rendering"));
  subMenu->addAction(actionManager->action(OmAction::COORDINATE_SYSTEM));
  subMenu->addAction(actionManager->action(OmAction::BOUNDING_OBJECT));
  subMenu->addAction(actionManager->action(OmAction::CONTACT_POINTS));
  subMenu->addAction(actionManager->action(OmAction::CONNECTOR_AXES));
  subMenu->addAction(actionManager->action(OmAction::JOINT_AXES));
  subMenu->addAction(actionManager->action(OmAction::RANGE_FINDER_FRUSTUMS));
  subMenu->addAction(actionManager->action(OmAction::LIDAR_RAYS_PATH));
  subMenu->addAction(actionManager->action(OmAction::LIDAR_POINT_CLOUD));
  subMenu->addAction(actionManager->action(OmAction::CAMERA_FRUSTUM));
  subMenu->addAction(actionManager->action(OmAction::DISTANCE_SENSOR_RAYS));
  subMenu->addAction(actionManager->action(OmAction::LIGHT_SENSOR_RAYS));
  subMenu->addAction(actionManager->action(OmAction::LIGHT_POSITIONS));
  subMenu->addAction(actionManager->action(OmAction::PEN_PAINTING_RAYS));
  subMenu->addAction(actionManager->action(OmAction::NORMALS));
  subMenu->addAction(actionManager->action(OmAction::RADAR_FRUSTUMS));
  subMenu->addAction(actionManager->action(OmAction::SKIN_SKELETON));

  // OMNISIM_DEBUG is preferred; WEBOTS_DEBUG is the legacy alias.
  if (!OmSysInfo::environmentVariable("OMNISIM_DEBUG").isEmpty() || !OmSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty()) {
    subMenu->addSeparator();
    subMenu->addAction(actionManager->action(OmAction::BOUNDING_SPHERE));
    subMenu->addAction(actionManager->action(OmAction::PHYSICS_CLUSTERS));
  }

  // these optional renderings are selection dependent
  subMenu->addSeparator();
  subMenu->addAction(actionManager->action(OmAction::CENTER_OF_MASS));
  subMenu->addAction(actionManager->action(OmAction::CENTER_OF_BUOYANCY));
  subMenu->addAction(actionManager->action(OmAction::SUPPORT_POLYGON));

  menu->addSeparator();
  subMenu = menu->addMenu(tr("&Scene Interactions"));
  subMenu->addAction(actionManager->action(OmAction::LOCK_VIEWPOINT));
  subMenu->addAction(actionManager->action(OmAction::DISABLE_SELECTION));
  subMenu->addAction(actionManager->action(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU));
  subMenu->addAction(actionManager->action(OmAction::DISABLE_OBJECT_MOVE));
  subMenu->addAction(actionManager->action(OmAction::DISABLE_FORCE_AND_TORQUE));
  subMenu->addAction(actionManager->action(OmAction::DISABLE_RENDERING));

  return menu;
}

QMenu *OmMainWindow::createSimulationMenu() {
  OmActionManager *manager = OmActionManager::instance();

  QMenu *menu = new QMenu(this);
  menu->setTitle(tr("&Simulation"));
  menu->addAction(manager->action(OmAction::PAUSE));
  menu->addAction(manager->action(OmAction::STEP));
  menu->addAction(manager->action(OmAction::REAL_TIME));
  menu->addAction(manager->action(OmAction::FAST));
  return menu;
}

QMenu *OmMainWindow::createOverlayMenu() {
  mOverlayMenu = new QMenu(this);
  mOverlayMenu->setTitle(tr("&Overlays"));

  mOverlayMenu->addAction(OmActionManager::instance()->action(OmAction::HIDE_ALL_CAMERA_OVERLAYS));
  mOverlayMenu->addAction(OmActionManager::instance()->action(OmAction::HIDE_ALL_RANGE_FINDER_OVERLAYS));
  mOverlayMenu->addAction(OmActionManager::instance()->action(OmAction::HIDE_ALL_DISPLAY_OVERLAYS));
  mOverlayMenu->addSeparator();

  OmContextMenuGenerator::setOverlaysMenu(mOverlayMenu);

  return mOverlayMenu;
}

void OmMainWindow::enableToolsWidgetItems(bool enabled) {
  OmActionManager::setActionEnabledSilently(mSimulationView->toggleView3DAction(), enabled);
  OmActionManager::setActionEnabledSilently(mSimulationView->toggleSceneTreeAction(), enabled);
  for (int i = 0; i < mConsoles.size(); ++i)
    OmActionManager::setActionEnabledSilently(mConsoles.at(i)->toggleViewAction(), enabled);
}

// we need this function because OmDockWidget and OmSimulationView don't have a common base class
void OmMainWindow::setWidgetMaximized(QWidget *widget, bool maximized) {
  OmDockWidget *dock = dynamic_cast<OmDockWidget *>(widget);
  OmSimulationView *view = dynamic_cast<OmSimulationView *>(widget);
  if (dock)
    dock->setMaximized(maximized);
  else
    view->setMaximized(maximized);
}

// maximize the sender widget
void OmMainWindow::maximizeDock() {
  mMinimizedDockState = saveState();
  mMaximizedWidget = static_cast<QWidget *>(sender());

  // close every other dock widget
  foreach (QWidget *dock, mDockWidgets) {
    OmDockWidget *dockWidget = dynamic_cast<OmDockWidget *>(dock);
    if (dock != mMaximizedWidget && (dockWidget == NULL || !dockWidget->isFloating()))
      dock->close();
  }
  enableToolsWidgetItems(false);
  setWidgetMaximized(mMaximizedWidget, true);
}

// minimize the maximized widget
void OmMainWindow::minimizeDock() {
  setWidgetMaximized(mMaximizedWidget, false);
  mMaximizedWidget = NULL;
  mSimulationView->show();
  restoreState(mMinimizedDockState);
  enableToolsWidgetItems(true);
}

QMenu *OmMainWindow::createToolsMenu() {
  QMenu *menu = new QMenu(this);
  menu->setTitle(tr("&Tools"));

  menu->addAction(mSimulationView->toggleView3DAction());
  menu->addAction(mSimulationView->toggleSceneTreeAction());

  QAction *action = new QAction(this);
  action->setText(tr("Restore &Layout"));
  action->setShortcut(Qt::CTRL | Qt::Key_J);
  action->setStatusTip(tr("Restore windows factory layout."));
  action->setToolTip(action->statusTip());
  connect(action, &QAction::triggered, this, &OmMainWindow::restoreLayout);
  menu->addAction(action);

  menu->addSeparator();

  menu->addAction(OmActionManager::instance()->action(OmAction::CLEAR_CONSOLE));
  menu->addAction(OmActionManager::instance()->action(OmAction::NEW_CONSOLE));
  connect(OmActionManager::instance()->action(OmAction::NEW_CONSOLE), SIGNAL(triggered()), this, SLOT(openNewConsole()));

  menu->addSeparator();

  action = new QAction(this);
  action->setMenuRole(QAction::PreferencesRole);  // Mac: put the menu respecting the MacOS specifications
  action->setText(tr("&Preferences..."));
  action->setStatusTip(tr("Open the Preferences window."));
  connect(action, &QAction::triggered, this, &OmMainWindow::openPreferencesDialog);
  menu->addAction(action);

  return menu;
}

QMenu *OmMainWindow::createHelpMenu() {
  QMenu *menu = new QMenu(this);
  menu->setTitle(tr("&Help"));

  QAction *action = new QAction(this);
  action->setMenuRole(QAction::AboutRole);  // Mac: put the menu respecting the MacOS specifications
  action->setText(tr("&About..."));
  action->setStatusTip(tr("Display information about OmniSim."));
  connect(action, &QAction::triggered, this, &OmMainWindow::showAboutBox);
  menu->addAction(action);

  action = new QAction(this);
  action->setText(tr("OmniSim &Guided Tour..."));
  action->setStatusTip(tr("Start a guided tour demonstrating OmniSim capabilities."));
  connect(action, &QAction::triggered, this, &OmMainWindow::showGuidedTour);
  menu->addAction(action);

  menu->addSeparator();

  action = new QAction(this);
  action->setText(tr("How do I &navigate in 3D?"));
  action->setStatusTip(tr("Show information about navigation in the 3D window."));
  connect(action, &QAction::triggered, this, &OmMainWindow::show3DViewingInfo);
  menu->addAction(action);

  action = new QAction(this);
  action->setText(tr("How do I &move an object?"));
  action->setStatusTip(tr("Show information about moving an object in the 3D window."));
  connect(action, &QAction::triggered, this, &OmMainWindow::show3DMovingInfo);
  menu->addAction(action);

  action = new QAction(this);
  action->setText(tr("How do I &apply a force or a torque to an object?"));
  action->setStatusTip(tr("Show information about applying a force or a torque to an object in the 3D window."));
  connect(action, &QAction::triggered, this, &OmMainWindow::show3DForceInfo);
  menu->addAction(action);

  menu->addSeparator();

  action = new QAction(this);
  action->setText(tr("&OpenGL Information..."));
  action->setStatusTip(tr("Show information about the current OpenGL hardware and driver."));
  connect(action, &QAction::triggered, this, &OmMainWindow::showOpenGlInfo);
  menu->addAction(action);

  menu->addSeparator();

  action = new QAction(this);
  action->setText(tr("&User Guide"));
  action->setStatusTip(tr("Open the OmniSim user guide online."));
  action->setShortcut(Qt::Key_F1);
  connect(action, &QAction::triggered, this, &OmMainWindow::showUserGuide);
  menu->addAction(action);

  action = new QAction(this);
  action->setText(tr("&Reference manual"));
  action->setStatusTip(tr("Open the OmniSim reference manual online."));
  action->setShortcut(Qt::Key_F2);
  connect(action, &QAction::triggered, this, &OmMainWindow::showReferenceManual);
  menu->addAction(action);

  action = new QAction(this);
  action->setText(tr("&OmniSim for automobiles"));
  action->setStatusTip(tr("Open the OmniSim for automobiles book online."));
  connect(action, &QAction::triggered, this, &OmMainWindow::showAutomobileDocumentation);
  menu->addAction(action);

  menu->addSeparator();

  action = new QAction(this);
  action->setText(tr("&GitHub repository..."));
  action->setStatusTip(tr("Open the OmniSim git repository on GitHub."));
  connect(action, &QAction::triggered, this, &OmMainWindow::openGithubRepository);
  menu->addAction(action);

  action = new QAction(this);
  action->setText(tr("&Bug Report..."));
  action->setStatusTip(tr("Report a bug to the GitHub repository."));
  connect(action, &QAction::triggered, this, &OmMainWindow::openBugReport);
  menu->addAction(action);

  return menu;
}

void OmMainWindow::createMenus() {
  mMenuBar = new QMenuBar(this);

  QMenu *menu = createFileMenu();
  mMenuBar->addAction(menu->menuAction());

  menu = createEditMenu();
  mMenuBar->addAction(menu->menuAction());

  menu = createViewMenu();
  mMenuBar->addAction(menu->menuAction());

  mSimulationMenu = createSimulationMenu();
  mMenuBar->addAction(mSimulationMenu->menuAction());

  menu = createOverlayMenu();
  mMenuBar->addAction(menu->menuAction());

  menu = createToolsMenu();
  mMenuBar->addAction(menu->menuAction());

  menu = createHelpMenu();
  mMenuBar->addAction(menu->menuAction());

  setMenuBar(mMenuBar);
}

void OmMainWindow::restorePreferredGeometry(bool minimizedOnStart) {
  OmPreferences *prefs = OmPreferences::instance();
  if (mRunBackground)
    // Background mode: do not realize the window. Skip restoring geometry —
    // any show*() call here would create a taskbar entry.
    return;
#ifdef __linux__
  if (minimizedOnStart && prefs->value("MainWindow/maximized", false).toBool())
    return;
#endif

  if (prefs->value("MainWindow/maximized", false).toBool()) {
    showMaximized();
    return;
  }

  const QRect &desktopRect = QGuiApplication::primaryScreen()->geometry();
  QRect preferedRect(prefs->value("MainWindow/position", QPoint(0, 0)).toPoint(),
                     prefs->value("MainWindow/size", QSize(0, 0)).toSize());

  if (preferedRect == QRect(0, 0, 0, 0) || !desktopRect.contains(preferedRect)) {
    preferedRect.setTopLeft(desktopRect.topLeft());
    preferedRect.setSize(desktopRect.size() - frameGeometry().size() + geometry().size());
  }

  resize(preferedRect.size());
  move(preferedRect.topLeft());
}

void OmMainWindow::writePreferences() const {
  OmPreferences *prefs = OmPreferences::instance();
  prefs->setValue("MainWindow/maximized", isMaximized());
  prefs->setValue("MainWindow/size", size());
  prefs->setValue("MainWindow/position", pos());
  prefs->sync();
}

void OmMainWindow::simulationQuit(int exitStatus) {
  mExitStatus = exitStatus;
  emit close();
}

bool OmMainWindow::event(QEvent *event) {
  if (mSimulationView && event->type() == QEvent::ScreenChangeInternal)
    mSimulationView->internalScreenChangedCallback();
  return QMainWindow::event(event);
}

void OmMainWindow::closeEvent(QCloseEvent *event) {
  if (!proposeToSaveWorld()) {
    event->ignore();
    return;
  }

  logActiveControllersTermination();

  if (OmWorld::instance()) {
    disconnect(OmWorld::instance(), &OmWorld::robotAdded, this, &OmMainWindow::addRobotInOverlaysMenu);
    disconnect(OmWorld::instance(), &OmWorld::robotRemoved, this, &OmMainWindow::removeRobotInOverlaysMenu);
  }

  // perspective need to be saved before deleting the
  // simulationView (and therefore the sceneTree), otherwise
  // the perspective of the node editor is not correctly saved
  if (OmApplication::instance())
    savePerspective(false, true);

  // if there is a pending recording, stop it correctly
  if (OmAnimationRecorder::instance()) {
    // setting the gui flag to false to prevent the dialog box "exporting success" to pop-up
    OmAnimationRecorder::instance()->setStartFromGuiFlag(false);
    OmAnimationRecorder::instance()->stop();
  }

  // the scene tree qt model should be cleaned first
  // otherwise some signals can be fired after the
  // QCoreApplication::exit() call
  // A better fix would be to move this code in a higher
  // level class deleting the mainwindow before deleting
  // OmApplication (and so OmWorld)
  mSimulationView->view3D()->logWrenStatistics();
  mSimulationView->view3D()->cleanupOptionalRendering();
  mSimulationView->view3D()->cleanupFullScreenOverlay();
  mSimulationView->cleanup();
  OmClipboard::deleteInstance();
  OmVisualBoundingSphere::deleteInstance();

  // really close
  if (OmApplication::instance()) {
    writePreferences();
    delete OmApplication::instance();
  }

  OmRenderingDeviceWindowFactory::deleteInstance();
  OmPerformanceLog::deleteInstance();

  event->accept();
  QCoreApplication::exit(mExitStatus);
}

void OmMainWindow::restoreLayout() {
  mSimulationView->show();
  restoreState(*mFactoryLayout);
  mMaximizedWidget = NULL;
  foreach (QWidget *dock, mDockWidgets)
    setWidgetMaximized(dock, false);
  if (mConsoles.size() >= 1) {
    for (int i = 1; i < mConsoles.size(); ++i)
      tabifyDockWidget(mConsoles.at(0), mConsoles.at(i));
  } else
    openNewConsole();
  mSimulationView->restoreFactoryLayout();
  enableToolsWidgetItems(true);
}

void OmMainWindow::savePerspective(bool reloading, bool saveToFile, bool isSaveEvent) {
  const OmWorld *world = OmWorld::instance();
  if (!world || OmFileUtil::isLocatedInInstallationDirectory(world->fileName()))
    return;

  OmPerspective *perspective = world->perspective();
  if (reloading) {
    // load previous settings
    // for example the perspectives of devices that have been deleted since the
    // last world save have to be loaded from the existing perspective file
    perspective->load(true);
    perspective->clearEnabledOptionalRenderings();
    perspective->clearRenderingDevicesPerspectiveList();
  }

  perspective->setMainWindowState(saveState());
  perspective->setSimulationViewState(mSimulationView->saveState());
  perspective->setMinimizedState(mMinimizedDockState);

  const int id = mDockWidgets.indexOf(mMaximizedWidget);
  perspective->setMaximizedDockId(id);
  perspective->setCentralWidgetVisible(mSimulationView->isVisible());

  perspective->setOrthographicViewHeight(world->orthographicViewHeight());

  QStringList robotWindowNodeNames;
  foreach (OmRobotWindow *robotWindow, mRobotWindows)
    // save only if a client is connected or in connection (empty client) to robotWindow.
    if (robotWindow->getClientID() != "-1")
      robotWindowNodeNames << robotWindow->robot()->computeUniqueName();
  perspective->setRobotWindowNodeNames(robotWindowNodeNames);

  QStringList centerOfMassEnabledNodeNames, centerOfBuoyancyEnabledNodeNames, supportPolygonEnabledNodeNames;
  world->retrieveNodeNamesWithOptionalRendering(centerOfMassEnabledNodeNames, centerOfBuoyancyEnabledNodeNames,
                                                supportPolygonEnabledNodeNames);
  perspective->setEnabledOptionalRendering(centerOfMassEnabledNodeNames, centerOfBuoyancyEnabledNodeNames,
                                           supportPolygonEnabledNodeNames);

  // save consoles perspective
  QVector<ConsoleSettings> settingsList;
  foreach (const OmConsole *console, mConsoles) {
    ConsoleSettings settings;
    settings.enabledFilters = console->getEnabledFilters();
    settings.enabledLevels = console->getEnabledLevels();
    settings.name = console->name();
    settingsList.append(settings);
  }
  perspective->setConsolesSettings(settingsList);

  // save rendering devices perspective
  const QList<OmRenderingDevice *> renderingDevices = OmRenderingDevice::renderingDevices();
  foreach (const OmRenderingDevice *device, renderingDevices) {
    if (device->overlay() != NULL)
      perspective->setRenderingDevicePerspective(device->computeShortUniqueName(), device->perspective());
  }

  // save rendering devices perspective of external window
  OmRenderingDeviceWindowFactory::instance()->saveWindowsPerspective(*perspective);

  // when saving using the save button, the disabler is bypassed
  // OMNISIM_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE is preferred; the WEBOTS_* name is the legacy alias.
  if ((qEnvironmentVariableIsSet("OMNISIM_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE") ?
         OmPreferences::booleanEnvironmentVariable("OMNISIM_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE") :
         OmPreferences::booleanEnvironmentVariable("WEBOTS_DISABLE_SAVE_SCREEN_PERSPECTIVE_ON_CLOSE")) &&
      !isSaveEvent)
    return;

  // save our new perspective in the file
  if (saveToFile)
    perspective->save();
}

void OmMainWindow::restorePerspective(bool reloading, bool firstLoad, bool loadingFromMemory) {
  OmWorld *world = OmWorld::instance();
  const OmPerspective *perspective = world->perspective();
  bool meansOfLoading = false;
  if (loadingFromMemory)
    meansOfLoading = true;
  else {
    meansOfLoading = world->reloadPerspective();
    perspective = world->perspective();
  }

  if (!loadingFromMemory) {
    // restore consoles
    const QVector<ConsoleSettings> consoleList = perspective->consoleList();
    for (int i = 0; i < consoleList.size(); ++i) {
      openNewConsole(consoleList.at(i).name);
      mConsoles.last()->setEnabledFilters(consoleList.at(i).enabledFilters);
      mConsoles.last()->setEnabledLevels(consoleList.at(i).enabledLevels);
    }
  }

  if (meansOfLoading) {
    if (!perspective->enabledRobotWindowNodeNames().isEmpty()) {
      const QList<OmRobot *> &robots = world->robots();
      mRobotWindowClosed = false;
      foreach (OmRobot *robot, robots) {
        if (perspective->enabledRobotWindowNodeNames().contains(robot->computeUniqueName()))
          // show robot window if it is in the perspective file
          showHtmlRobotWindow(robot, false);
      }
    }
    restoreState(perspective->mainWindowState());
    mMinimizedDockState = perspective->minimizedState();
    const int id = perspective->maximizedDockId();
    mMaximizedWidget = (id >= 0 && id < mDockWidgets.size()) ? mDockWidgets.at(id) : NULL;
    enableToolsWidgetItems(mMaximizedWidget == NULL);
    if (!reloading) {
      mSimulationView->setVisible(perspective->centralWidgetVisible());
      mSimulationView->restoreState(perspective->simulationViewState(), firstLoad);
    }
    // update icons
    foreach (QWidget *dock, mDockWidgets)
      setWidgetMaximized(dock, dock == mMaximizedWidget);
  } else if (firstLoad)
    // set default simulation view perspective
    mSimulationView->restoreFactoryLayout();

  const double ovh = perspective->orthographicViewHeight();
  world->setOrthographicViewHeight(ovh);

  mSimulationView->view3D()->restoreOptionalRendering(perspective->enabledCenterOfMassNodeNames(),
                                                      perspective->enabledCenterOfBuoyancyNodeNames(),
                                                      perspective->enabledSupportPolygonNodeNames());

  if (firstLoad)  // for the first load we can't restore the rendering devices perspective now because the size of the wren
                  // window has not be set yet
    connect(mSimulationView->view3D(), &OmView3D::resized, this, &OmMainWindow::restoreRenderingDevicesPerspective);
  else
    restoreRenderingDevicesPerspective();

  // Refreshing
  mSimulationView->repaintView3D();

  OmLog::setConsoleLogsPostponed(false);
  OmLog::instance()->showPendingConsoleMessages();
}

void OmMainWindow::restoreRenderingDevicesPerspective() {
  const OmPerspective *perspective = OmWorld::instance()->perspective();
  const QList<OmRenderingDevice *> devices = OmRenderingDevice::renderingDevices();
  for (int i = 0; i < devices.size(); ++i) {
    OmRenderingDevice *device = devices[i];
    QStringList devicePerspective = perspective->renderingDevicePerspective(device->computeShortUniqueName());
    if (!devicePerspective.isEmpty())
      device->restorePerspective(devicePerspective);
  }
  disconnect(mSimulationView->view3D(), &OmView3D::resized, this, &OmMainWindow::restoreRenderingDevicesPerspective);
  updateOverlayMenu();
}

void OmMainWindow::loadDifferentWorld(const QString &fileName) {
  loadWorld(fileName, false);
}

bool OmMainWindow::proposeToSaveWorld(bool reloading) {
  const OmWorld *world = OmWorld::instance();
  if (world != NULL && world->needSaving() && !OmProject::current()->isReadOnly() && OmMessageBox::enabled()) {
    OmSaveWarningDialog *dialog = new OmSaveWarningDialog(world->fileName(), world->isModifiedFromSceneTree(), reloading, this);
    int result = dialog->exec();
    if (result == QMessageBox::Cancel)
      return false;
    if (result == QMessageBox::Save)
      saveWorld();
  }
  return true;
}

QString OmMainWindow::findHtmlFileName(const char *title) {
  OmSimulationState::instance()->pauseSimulation();
  const QString worldName = QFileInfo(OmWorld::instance()->fileName()).baseName();

  QString fileName;
  for (int i = 0; i < 1000; ++i) {
    const QString suffix = i == 0 ? "" : QString("_%1").arg(i);
    fileName = OmPreferences::instance()->value("Directories/www").toString() + worldName + suffix + ".html";
    if (!QFileInfo::exists(fileName))
      break;
  }

  fileName = QFileDialog::getSaveFileName(this, tr(title), OmProject::computeBestPathForSaveAs(fileName),
                                          tr("HTML Files (*.html *.HTML)"));

  if (fileName.isEmpty()) {
    return QString();
  }

  if (!fileName.endsWith(".html", Qt::CaseInsensitive))
    fileName.append(".html");

  return fileName;
}

void OmMainWindow::loadWorld(const QString &fileName, bool reloading) {
  if (!proposeToSaveWorld(reloading))
    return;
  if (!OmApplication::instance()->isValidWorldFileName(fileName)) {
    OmApplication::instance()->cancelWorldLoading(true);
    return;
  }
  mSimulationView->cancelSupervisorMovieRecording();
  if (OmWorld::instance()) {
    disconnect(OmWorld::instance(), &OmWorld::robotAdded, this, &OmMainWindow::addRobotInOverlaysMenu);
    disconnect(OmWorld::instance(), &OmWorld::robotRemoved, this, &OmMainWindow::removeRobotInOverlaysMenu);
  }
  logActiveControllersTermination();
  OmLog::setConsoleLogsPostponed(true);
  // Suspend the wgpu main-view render BEFORE the load begins: world loading pumps the Qt event loop,
  // so a paint can fire while the old world is being torn down — driving the wgpu path against a
  // half-freed world/scene crashed on reload. The view falls back to WREN (which reloads safely) until
  // updateAfterWorldLoading() re-enables it on the fully-loaded new world.
  mSimulationView->view3D()->setWgpuMainViewSuspended(true);
  OmApplication::instance()->loadWorld(fileName, reloading);
}

void OmMainWindow::updateBeforeWorldLoading(bool reloading) {
  OmLog::setPopUpPostponed(true);
  savePerspective(reloading, true);

  deleteRobotWindow(NULL);  // delete all the robot windows

  mSimulationView->view3D()->logWrenStatistics();
  if (!reloading && OmClipboard::instance()->type() == WB_SF_NODE)
    OmClipboard::instance()->replaceAllExternalDefNodesInString();
  mSimulationView->prepareWorldLoading();
  OmVisualBoundingSphere::deleteInstance();

  foreach (OmConsole *console, mConsoles) {
    mDockWidgets.removeAll(console);
    delete console;
  }
  mConsoles.clear();
}

void OmMainWindow::updateAfterWorldLoading(bool reloading, bool firstLoad) {
  const OmWorld *world = OmWorld::instance();
  if (world->fileName() != OmProject::newWorldPath())
    mRecentFiles->makeRecent(world->fileName());

  mSimulationView->setWorld(OmSimulationWorld::instance());
  mSimulationView->view3D()->setWgpuMainViewSuspended(false);  // new world is ready → wgpu main view may resume

  // update 'view' menu
  const OmPerspective *perspective = world->perspective();
  OmActionManager::instance()
    ->action(OmAction::LOCK_VIEWPOINT)
    ->setChecked(perspective->isUserInteractionDisabled(OmAction::LOCK_VIEWPOINT));
  OmActionManager::instance()
    ->action(OmAction::DISABLE_SELECTION)
    ->setChecked(perspective->isUserInteractionDisabled(OmAction::DISABLE_SELECTION));
  OmActionManager::instance()
    ->action(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU)
    ->setChecked(perspective->isUserInteractionDisabled(OmAction::DISABLE_3D_VIEW_CONTEXT_MENU));
  OmActionManager::instance()
    ->action(OmAction::DISABLE_OBJECT_MOVE)
    ->setChecked(perspective->isUserInteractionDisabled(OmAction::DISABLE_OBJECT_MOVE));
  OmActionManager::instance()
    ->action(OmAction::DISABLE_FORCE_AND_TORQUE)
    ->setChecked(perspective->isUserInteractionDisabled(OmAction::DISABLE_FORCE_AND_TORQUE));
  OmActionManager::instance()
    ->action(OmAction::DISABLE_RENDERING)
    ->setChecked(perspective->isUserInteractionDisabled(OmAction::DISABLE_RENDERING));
  mSimulationView->disableRendering(perspective->isUserInteractionDisabled(OmAction::DISABLE_RENDERING));
  // OMNISIM_DEBUG is preferred; WEBOTS_DEBUG is the legacy alias.
  if (!OmSysInfo::environmentVariable("OMNISIM_DEBUG").isEmpty() || !OmSysInfo::environmentVariable("WEBOTS_DEBUG").isEmpty()) {
    OmVisualBoundingSphere::enable(perspective->isGlobalOptionalRenderingEnabled("BoundingSphere"), NULL);
    connect(mSimulationView->sceneTree(), &OmSceneTree::nodeSelected, OmVisualBoundingSphere::instance(),
            &OmVisualBoundingSphere::show);
  }

  OmRenderingDeviceWindowFactory::reset();
  restorePerspective(reloading, firstLoad, false);

  emit splashScreenCloseRequested();

  connect(world, &OmWorld::modificationChanged, this, &OmMainWindow::updateWindowTitle);
  connect(world, &OmWorld::resetRequested, this, &OmMainWindow::resetGui, Qt::QueuedConnection);
  // update 'overlays' menu
  connect(OmWorld::instance(), &OmWorld::robotAdded, this, &OmMainWindow::addRobotInOverlaysMenu);
  connect(OmWorld::instance(), &OmWorld::robotRemoved, this, &OmMainWindow::removeRobotInOverlaysMenu);

  updateWindowTitle();

  if (!reloading)
    OmActionManager::instance()->resetApplicationActionsState();
  // reset focus widget used to identify the actions target widget
  OmActionManager::instance()->setFocusObject(mSimulationView->view3D());

  OmLog::setPopUpPostponed(false);
  OmLog::showPostponedPopUpMessages();
  connect(OmProject::current(), &OmProject::pathChanged, this, &OmMainWindow::updateProjectPath);
}

void OmMainWindow::openWorld() {
  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  QString fileName =
    QFileDialog::getOpenFileName(this, tr("Open World File"), OmProject::current()->worldsPath(),
                                 tr("World Files (*.omniworld *.OMNIWORLD *.wbt *.WBT)"));
  if (!fileName.isEmpty())
    loadWorld(fileName);

  simulationState->resumeSimulation();
}

void OmMainWindow::openSampleWorld() {
  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  OmOpenSampleWorldDialog dialog(this);
  if (dialog.exec())
    loadWorld(dialog.selectedWorld());

  simulationState->resumeSimulation();
}

bool OmMainWindow::runSimulationHasRunWarningMessage() {
  const QString message(tr("The simulation has run!") + "\n" +
                        tr("Saving the world file will store the current world state: the objects position and rotation and "
                           "other fields may differ from the original file!") +
                        "\n" + tr("Do you want to save this modified world?"));
  return OmMessageBox::question(message, this, tr("Question"), QMessageBox::Cancel, QMessageBox::Cancel | QMessageBox::Save) ==
         QMessageBox::Save;
}

void OmMainWindow::saveWorld() {
  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  OmSimulationWorld *world = OmSimulationWorld::instance();
  if (OmSimulationState::instance()->hasStarted() && !runSimulationHasRunWarningMessage()) {
    simulationState->resumeSimulation();
    return;
  }

  QString worldFileName = world->fileName();

  if (!OmProjectRelocationDialog::validateLocation(this, worldFileName)) {
    simulationState->resumeSimulation();
    return;
  }

  mSimulationView->applyChanges();
  if (world->save()) {
    QString thumbnailFileName = worldFileName;
    const QString thumbnailName =
      "." + OmWorldFileFormat::replaceExtension(thumbnailFileName.split("/").takeLast(), ".jpg");
    thumbnailFileName.replace(thumbnailFileName.split("/").takeLast(), thumbnailName, Qt::CaseInsensitive);

    savePerspective(false, true, true);
    updateWindowTitle();
    mSimulationView->takeThumbnail(thumbnailFileName);
  } else
    OmMessageBox::warning(tr("Unable to save '%1'.").arg(world->fileName()));

  simulationState->resumeSimulation();
}

void OmMainWindow::saveWorldAs(bool skipSimulationHasRunWarning) {
  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  mSimulationView->applyChanges();

  if (!skipSimulationHasRunWarning && OmSimulationState::instance()->hasStarted() && !runSimulationHasRunWarningMessage()) {
    simulationState->resumeSimulation();
    return;
  }

  OmWorld *world = OmWorld::instance();

  QString fileName = QFileDialog::getSaveFileName(
    this, tr("Save World File"), OmProject::computeBestPathForSaveAs(world->fileName()),
    tr("World Files (*.omniworld *.OMNIWORLD *.wbt *.WBT)"));

  if (fileName.isEmpty()) {
    simulationState->resumeSimulation();
    return;
  }

  if (QFileInfo(fileName).dir().dirName() != "worlds") {
    const QString warning = tr("The selected directory for saving the world file is not named \"worlds\".\n"
                               "Thus it is not located in a valid OmniSim project.\n"
                               "As a consequence, some project-related functionalities may not work.");
    if (OmMessageBox::enabled()) {
      QMessageBox msgBox(QMessageBox::Warning, tr("Save World File"), warning, QMessageBox::Cancel, this);
      msgBox.addButton(
        new QPushButton(QApplication::style()->standardIcon(QStyle::SP_DialogOkButton), tr("Save Anyway"), &msgBox),
        QMessageBox::AcceptRole);
      msgBox.setDefaultButton(QMessageBox::Cancel);
      if (msgBox.exec() == QMessageBox::Cancel) {
        simulationState->resumeSimulation();
        return;
      }
    } else
      OmLog::warning(warning);
  }

  if (!OmWorldFileFormat::isWorldFile(fileName))
    fileName.append(OmWorldFileFormat::writeExtension());

  if (OmProjectRelocationDialog::validateLocation(this, fileName)) {
    mRecentFiles->makeRecent(fileName);
    if (world->saveAs(fileName)) {
      QString thumbnailFileName = fileName;
      const QString thumbnailName =
        "." + OmWorldFileFormat::replaceExtension(thumbnailFileName.split("/").takeLast(), ".jpg");
      thumbnailFileName.replace(thumbnailFileName.split("/").takeLast(), thumbnailName, Qt::CaseInsensitive);

      savePerspective(false, true, true);
      updateWindowTitle();
      mSimulationView->takeThumbnail(thumbnailFileName);
    } else
      OmMessageBox::warning(tr("Unable to save '%1'.").arg(fileName));
  }

  simulationState->resumeSimulation();
}

void OmMainWindow::reloadWorld() {
  toggleAnimationAction(false);
  if (!OmWorld::instance())
    loadWorld(OmProject::newWorldPath());
  else
    loadWorld(OmWorld::instance()->fileName(), true);
}

void OmMainWindow::resetWorldFromGui() {
  if (!OmWorld::instance())
    loadWorld(OmProject::newWorldPath());
  else
    OmWorld::instance()->reset(true);

  if (mAnimationRecordingTimer->isActive())
    OmLog::info(tr("HTML recording canceled, locally stored files will still be available."));

  resetGui(true);
}

void OmMainWindow::resetGui(bool restartControllers) {
  toggleAnimationAction(false);
  if (OmWorld::instance() && restartControllers)
    mSimulationView->cancelSupervisorMovieRecording();
  mSimulationView->view3D()->renderLater();
  mSimulationView->disableStepButton(false);
}

QString OmMainWindow::exportHtmlFiles() {
  QString fileName = findHtmlFileName("Export HTML File");
  OmSimulationState::instance()->resumeSimulation();
  return fileName;
}

void OmMainWindow::showAboutBox() {
  OmAboutBox *box = new OmAboutBox(this);
  box->exec();
}

void OmMainWindow::showUpdatedDialog() {
  OmUpdatedDialog *updatedDialog = new OmUpdatedDialog(this);
  updatedDialog->show();
  updatedDialog->raise();
  connect(updatedDialog, &OmUpdatedDialog::rejected, this, &OmMainWindow::showGuidedTour);
}

void OmMainWindow::showGuidedTour() {
  OmGuidedTour *tour = OmGuidedTour::instance(this);
  tour->show();
  tour->raise();
  connect(tour, &OmGuidedTour::loadWorldRequest, this, &OmMainWindow::loadDifferentWorld);
}

void OmMainWindow::setView3DSize(const QSize &size) {
  mSimulationView->enableView3DFixedSize(size);
}

void OmMainWindow::show3DViewingInfo() {
  const QString info =
    tr("<strong>Rotate:</strong><br/>"
       "To rotate the camera around the x and y axis, you have to set the mouse pointer in the 3D scene, press the left mouse "
       "button and drag the mouse:<br/>"
       "- if you clicked on an object, the rotation will be centered around the picked point on this object.<br/>"
       "- if you clicked outside of any object, the rotation will be centered around the position of the camera.<br/>"
       "Dragging the mouse horizontally will rotate the camera around the world up axis. "
       "Dragging the mouse vertically will rotate the camera around its horizontal axis.<br/><br/>"
       "<strong>Translate:</strong><br/>"
       "To translate the camera in the x and y directions, you have to set the mouse pointer in the 3D scene, press the right "
       "mouse button and drag the mouse.<br/><br/>"
       "<strong>Zoom / Tilt:</strong><br/>"
       "Set the mouse pointer in the 3D scene, then:<br/>"
       "- if you press both left and right mouse buttons (or the middle button) and drag the mouse vertically, the camera will "
       "zoom in or out.<br/>"
       "- if you press both left and right mouse buttons (or the middle button) and drag the mouse horizontally, the camera "
       "will rotate around the viewing axis (tilt movement).<br/>"
       "- if you use the wheel of the mouse, the camera will zoom in or out.");
  OmMessageBox::info(info, this, tr("How do I navigate in 3D?"));
}

void OmMainWindow::show3DMovingInfo() {
  const QString info =
    tr("In order to move an object: first <strong>select the object</strong> with a left mouse button click.<br/><br/>"
       "Then <strong>click and drag the arrow-shaped handles</strong> to translate or "
       "rotate the object along the corresponding axis.<br/><br/>"
       "Alternatively, you can hold the shift key and use the mouse:<br/>"
       "<em>Horizontal translation:</em><br/>"
       "Use the left mouse button while the shift key is down to drag an object parallel to the ground.<br/>"
       "<em>Vertical rotation:</em><br/>"
       "Use the right mouse button while the shift key is down to rotate an object around the world's vertical axis.<br/>"
       "<em>Lift:</em><br/>"
       "Press both left and right mouse buttons, press the middle mouse button, or roll the mouse wheel "
       "while the shift key is down to raise or lower the selected object.");
  OmMessageBox::info(info, this, tr("How do I move an object?"));
}

void OmMainWindow::show3DForceInfo() {
  static const QString infoLinux(
    tr("<strong>Force:</strong><br/> Place the mouse pointer where the force will apply and hold down the Alt key"
       " and the left mouse button together while dragging the mouse. In some window managers it might be necessary"
       " to also hold the Control (ctrl) key together with the Alt key.<br/><br/> <strong>Torque:</strong><br/>"
       "Place the mouse pointer on the object and hold down the Alt key"
       " and the right mouse button together while dragging the mouse. In some window managers it might be necessary"
       " to also hold the Control (ctrl) key together with the Alt key."));

  static const QString infoWindows(
    tr("<strong>Force:</strong><br/> Place the mouse pointer where the force will apply and hold down the Alt key"
       " and the left mouse button together while dragging the mouse.<br/><br/> <strong>Torque:</strong><br/>"
       "Place the mouse pointer on the object and hold down the Alt key"
       " and the right mouse button together while dragging the mouse."));

  static const QString infoMac(
    tr("<strong>Force:</strong><br/> Place the mouse pointer where the force will apply and hold down the Alt key"
       " and the left mouse button together while dragging the mouse.<br/><br/> <strong>Torque:</strong><br/>"
       "Place the mouse pointer on the object and hold down the Alt key"
       " and the right mouse button together while dragging the mouse."
       "<br/><br/>If you have a one-button mouse, hold down also the Control key (Ctrl) to emulate the right mouse button."));

  QString info;

  switch (OmSysInfo::platform()) {
    case OmSysInfo::LINUX_PLATFORM:
      info = infoLinux;
      break;
    case OmSysInfo::MACOS_PLATFORM:
      info = infoMac;
      break;
    case OmSysInfo::WIN32_PLATFORM:
      info = infoWindows;
      break;
    default:
      assert(false);
  }
  OmMessageBox::info(info, this, tr("How do I apply a force or a torque to an object?"));
}

void OmMainWindow::showOpenGlInfo() {
  QOpenGLFunctions_3_3_Core gl;
  gl.initializeOpenGLFunctions();
  QString info;
  info += tr("Host name: ") + QHostInfo::localHostName() + "\n";
  info += tr("System: ") + OmSysInfo::sysInfo() + "\n";
  info += tr("OpenGL vendor: ") + reinterpret_cast<const char *>(gl.glGetString(GL_VENDOR)) + "\n";
  info += tr("OpenGL renderer: ") + reinterpret_cast<const char *>(gl.glGetString(GL_RENDERER)) + "\n";
  info += tr("OpenGL version: ") + reinterpret_cast<const char *>(gl.glGetString(GL_VERSION)) + "\n";
  // Lane E4 (WREN-deletion runbook): identify the wgpu adapter alongside the GL strings --
  // post-D1.4 the GL lines above go with WREN and this becomes the primary identification.
  OmVulkanBackend *wgpu = static_cast<OmVulkanBackend *>(OmRenderBackendRegistry::vulkanBackend());
  const bool wgpuUp = wgpu && wgpu->isAvailable();
  if (wgpuUp)
    info += tr("wgpu adapter: ") + QString::fromUtf8(wgpu->adapterSummary()) + "\n";
  info += tr("Available GPU memory: ");
  // D1.4: the WREN GL query is gone. The wgpu source is gpuMemoryBytes(), and its honest
  // answer is "the wgpu C API has no memory query" -- so when there is no real figure the
  // dialog SAYS unavailable, never 0 MB.
  const long long wgpuMemory = wgpuUp ? wgpu->gpuMemoryBytes() : -1;
  if (wgpuMemory > 0)
    info += tr("%1 bytes").arg(wgpuMemory);
  else if (wgpuUp)
    info += tr("unavailable (the wgpu C API exposes no GPU-memory query)");
  else
    info += tr("N/A");
  info += "\n";
  OmMessageBox::info(info, this, tr("OpenGL information"));
}

void OmMainWindow::openNewConsole(const QString &name) {
  OmConsole *console = new OmConsole(this, name);
  connect(console, &OmConsole::closed, this, &OmMainWindow::handleConsoleClosure);
  addDockWidget(Qt::BottomDockWidgetArea, console);
  if (!mConsoles.isEmpty()) {
    tabifyDockWidget(mConsoles.at(0), console);
    console->show();
    console->raise();
  }
  addDock(console);
  console->setStyleSheet(styleSheet());
  console->setVisible(true);
  mConsoles.append(console);
}

void OmMainWindow::handleConsoleClosure() {
  OmConsole *console = dynamic_cast<OmConsole *>(sender());
  if (console) {
    mConsoles.removeAll(console);
    mDockWidgets.removeAll(console);
    delete console;
  }
}

void OmMainWindow::showDocument(const QString &url) {
  bool ret;
  if (url.startsWith("http") || url.startsWith("www"))
    ret = OmDesktopServices::openUrl(url);
  else {
#ifdef __linux__  // on linux, the '/lib' directory need to be removed from the LD_LIBRARY_PATH,
                  // otherwise their is some libraries conflicts when trying to open pdf with Evince
    QString WEBOTS_HOME(QDir::toNativeSeparators(OmStandardPaths::omniSimHomePath()));
    QByteArray ldLibraryPathBackup = qgetenv("LD_LIBRARY_PATH");
    QByteArray newLdLibraryPath = ldLibraryPathBackup;
    newLdLibraryPath.replace((WEBOTS_HOME + "lib/webots/").toUtf8(), "");
    newLdLibraryPath.replace((WEBOTS_HOME + "lib/webots").toUtf8(), "");
    qputenv("LD_LIBRARY_PATH", newLdLibraryPath);
#endif
    QString u("file:///" + url);
    ret = OmDesktopServices::openUrl(u);
#ifdef __linux__
    qputenv("LD_LIBRARY_PATH", ldLibraryPathBackup);
#endif
  }
  if (!ret)
    OmMessageBox::warning(tr("Cannot open the document: '%1'.").arg(url), this, tr("Internal error"));
}

void OmMainWindow::showOnlineDocumentation(const QString &book, const QString &page) {
  // Open the matching Markdown page in the OmniSim repo on GitHub. Index pages
  // (e.g. book="guide" with no explicit page) resolve to .../docs/guide/index.md
  // because page defaults to "index" at the call sites.
  const QString url = OmStandardPaths::omniSimDocsBaseUrl() + "/" + book + "/" + page + ".md";
  showDocument(url);
}

void OmMainWindow::showUserGuide() {
  showOnlineDocumentation("guide");
}

void OmMainWindow::showReferenceManual() {
  showOnlineDocumentation("reference");
}

void OmMainWindow::showAutomobileDocumentation() {
  showOnlineDocumentation("automobile");
}

void OmMainWindow::openGithubRepository() {
  showDocument(OmStandardPaths::githubRepositoryUrl());
}

void OmMainWindow::openBugReport() {
  showDocument(QString("%1/issues/new/choose").arg(OmStandardPaths::githubRepositoryUrl()));
}

void OmMainWindow::newProjectDirectory() {
  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();
  OmNewProjectWizard wizard(this);
  wizard.exec();
  simulationState->resumeSimulation();
  if (!wizard.fileName().isEmpty())
    loadWorld(OmProject::current()->worldsPath() + wizard.fileName());
}

void OmMainWindow::newWorld() {
  QString worldsPath = OmProject::current()->worldsPath();
  if (!OmProjectRelocationDialog::validateLocation(this, worldsPath))
    return;

  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();
  OmNewWorldWizard wizard(this);
  wizard.exec();
  simulationState->resumeSimulation();

  const QString worldPath = OmProject::current()->worldsPath() + wizard.fileName();
  if (!wizard.fileName().isEmpty() && QFile::exists(worldPath))
    loadWorld(worldPath);
}

void OmMainWindow::newRobotController() {
  QString controllersPath = OmProject::current()->controllersPath();
  if (!OmProjectRelocationDialog::validateLocation(this, controllersPath))
    return;

  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  OmNewControllerWizard wizard(this);
  wizard.exec();

  simulationState->resumeSimulation();
}

void OmMainWindow::newProto() {
  // should not generate the wizard until all the dependencies are available, double re-entry is necessary
  if (qobject_cast<QAction *>(sender())) {  // if the function is reached by the GUI menu
    connect(OmProtoManager::instance(), &OmProtoManager::dependenciesAvailable, this, &OmMainWindow::newProto);
    OmProtoManager::instance()->retrieveLocalProtoDependencies();
    return;
  }

  // reachable only if the function was called by OmProtoManager
  disconnect(OmProtoManager::instance(), &OmProtoManager::dependenciesAvailable, this, &OmMainWindow::newProto);

  QString protosPath = OmProject::current()->protosPath();
  if (!OmProjectRelocationDialog::validateLocation(this, protosPath))
    return;

  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  OmNewProtoWizard wizard(this);
  wizard.exec();

  simulationState->resumeSimulation();
}

void OmMainWindow::openPreferencesDialog() {
  OmPreferencesDialog dialog(this);
  connect(&dialog, &OmPreferencesDialog::restartRequested, this, &OmMainWindow::restartRequested);
  dialog.exec();
}

void OmMainWindow::setTheme(const QString &qssFile) {
  OmPreferences *prefs = OmPreferences::instance();
  if (prefs->value("General/theme").toString() == qssFile)
    return;
  prefs->setValue("General/theme", qssFile);
  prefs->sync();
  const bool restart = OmMessageBox::question(
                         tr("The theme change will be fully applied after OmniSim is restarted. Restart OmniSim now?"),
                         this, tr("Restart Now?"), QMessageBox::Yes, QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes;
  if (restart)
    emit restartRequested();
}

void OmMainWindow::updateWindowTitle() {
  const QString nameAndVersion("OmniSim " + OmApplicationInfo::omniSimVersion());
  QString title;

  if (OmWorld::instance()) {
    title = QDir::toNativeSeparators(OmWorld::instance()->fileName());
    QString projectName = OmProject::projectNameFromWorldFile(OmWorld::instance()->fileName());
    if (!projectName.isEmpty())
      title += " (" + projectName + ") - ";
    else
      title += " (No Project) - ";

    title += nameAndVersion;
  } else
    title = nameAndVersion;

  setWindowTitle(title);
}

void OmMainWindow::simulationEnabledChanged(bool e) {
  mSimulationMenu->setEnabled(e);
}

void OmMainWindow::showStatusBarMessage(OmLog::Level level, const QString &message) {
  if (level == OmLog::STATUS)
    statusBar()->showMessage(message);
}

void OmMainWindow::showRobotWindow() {
  OmRobot *robot = mSimulationView->selectedRobot();
  mRobotWindowClosed = false;
  if (robot) {
    if (robot->window() == "<none>")  // no robot window
      OmMessageBox::info(tr("Cannot show <none> robot window."));
    else if (robot->windowFile().isEmpty())
      robot->showWindow();  // not a HTML robot window
    else
      showHtmlRobotWindow(robot, true);  // show HTML robot window
  }
}

void OmMainWindow::showHtmlRobotWindow(OmRobot *robot, bool manualTrigger) {
  OmRobotWindow *currentRobotWindow = NULL;
  foreach (OmRobotWindow *robotWindow, mRobotWindows) {
    if (robotWindow->robot() == robot) {  // close only the client of the robot window associated with the robot.
      if (robotWindow->getClientID() == "-1" || robotWindow->getClientID().isEmpty()) {
        if (manualTrigger)
          robotWindow->setupPage(mTcpServer->port());
        else
          mRobotWindowClosed = true;
        return;
      } else
        mTcpServer->closeClient(robotWindow->getClientID());
      currentRobotWindow = robotWindow;
    }
  }

  if (mOnSocketOpen) {
    mOnSocketOpen = false;

    if (currentRobotWindow == NULL) {  // if no robot window associated with the robot, create one.
      currentRobotWindow = new OmRobotWindow(robot);
      mRobotWindows << currentRobotWindow;
      connect(mTcpServer, &OmTcpServer::sendRobotWindowClientID, currentRobotWindow, &OmRobotWindow::setClientID);
      connect(robot, &OmNode::isBeingDestroyed, this, [this, robot]() { deleteRobotWindow(robot); });
      connect(robot, &OmMatter::matterNameChanged, this, [this, robot]() { showHtmlRobotWindow(robot, false); });
      connect(robot, &OmRobot::controllerChanged, this, [this, robot]() { showHtmlRobotWindow(robot, false); });
      connect(robot, &OmRobot::externControllerChanged, this, [this, robot]() { showHtmlRobotWindow(robot, false); });
      connect(robot, &OmRobot::windowChanged, this, [this, robot]() { deleteRobotWindow(robot); });
      connect(currentRobotWindow, &OmRobotWindow::socketOpened, this, &OmMainWindow::onSocketOpened);
    }

    if (currentRobotWindow && currentRobotWindow->robot() == robot && !mRobotWindowClosed)
      currentRobotWindow->setupPage(mTcpServer->port());
  } else if (!mRobotWindowClosed) {
    const int maxPendingRobotWindows = 3;
    if (mRobotsWaitingForWindowToOpen.size() < maxPendingRobotWindows)
      mRobotsWaitingForWindowToOpen << robot;
    else
      OmLog::warning(tr("Maximum number of pending robot windows reached."));
  }
}

void OmMainWindow::onSocketOpened() {
  mOnSocketOpen = true;
  if (!mRobotsWaitingForWindowToOpen.isEmpty())
    showHtmlRobotWindow(mRobotsWaitingForWindowToOpen.takeFirst(), false);
}

void OmMainWindow::closeClientRobotWindow(OmRobot *robot) {
  foreach (OmRobotWindow *robotWindow, mRobotWindows)
    if ((robotWindow->robot() == robot))
      mTcpServer->closeClient(robotWindow->getClientID());
}

void OmMainWindow::deleteRobotWindow(OmRobot *robot) {
  // delete the robot window and client of robot, delete all if NULL.
  foreach (OmRobotWindow *robotWindow, mRobotWindows)
    if ((robotWindow->robot() == robot) || robot == NULL) {
      closeClientRobotWindow(robotWindow->robot());
      disconnect(mTcpServer, &OmTcpServer::sendRobotWindowClientID, robotWindow, &OmRobotWindow::setClientID);
      disconnect(robotWindow, &OmRobotWindow::socketOpened, this, &OmMainWindow::onSocketOpened);
      robotWindow->robot()->disconnect(this);
      mRobotWindows.removeAll(robotWindow);
      delete robotWindow;
    }

  mOnSocketOpen = true;
}

void OmMainWindow::clearOverlaysMenu() {
  QList<QAction *> actions = mOverlayMenu->actions();
  while (actions.size() > 4) {
    QAction *action = actions.last();
    if (action->menu())
      mOverlayMenu->removeAction(action);
    actions.removeLast();
  }
}

void OmMainWindow::updateRobotNameInOverlaysMenu() {
  const OmRobot *robot = static_cast<OmRobot *>(sender());
  QListIterator<QAction *> it(mOverlayMenu->actions());
  while (it.hasNext()) {
    const QAction *action = it.next();
    QMenu *menu = action->menu();
    if (menu && menu->property("robot").value<void *>() == robot) {
      QString robotName = robot->name();
#ifdef __linux__
      // fix Unity bug with underscores in menu item text
      if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity")
        robotName.replace("_", "__");
#endif
      menu->setTitle(tr("'%1' Overlays").arg(robotName));
    }
  }
}

void OmMainWindow::removeRobotInOverlaysMenu(const OmRobot *robot) {
  QListIterator<QAction *> it(mOverlayMenu->actions());
  while (it.hasNext()) {
    QAction *action = it.next();
    const QMenu *menu = action->menu();
    if (menu && menu->property("robot").value<void *>() == robot) {
      mOverlayMenu->removeAction(action);
      return;
    }
  }
}

void OmMainWindow::addRobotInOverlaysMenu(OmRobot *robot) {
  QAction *action = NULL;
  QList<QAction *> cameraActions;
  QList<QAction *> rangeFinderActions;
  QList<QAction *> displayActions;

  QString robotName = robot->name();
#ifdef __linux__
  // fix Unity bug with underscores in menu item text
  if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity")
    robotName.replace("_", "__");
#endif
  QMenu *robotMenu = mOverlayMenu->addMenu(tr("'%1' Overlays").arg(robotName));
  robotMenu->setProperty("robot", QVariant::fromValue(static_cast<void *>(const_cast<OmRobot *>(robot))));
  connect(robot, &OmMatter::matterNameChanged, this, &OmMainWindow::updateRobotNameInOverlaysMenu);

  QListIterator<OmRenderingDevice *> devicesIt(robot->renderingDevices());
  while (devicesIt.hasNext()) {
    const OmRenderingDevice *device = devicesIt.next();
    QString deviceName = device->name();
#ifdef __linux__
    // fix Unity bug with underscores in menu item text
    if (qgetenv("XDG_CURRENT_DESKTOP") == "Unity")
      deviceName.replace("_", "__");
#endif
    action = new QAction(this);
    action->setText(tr("Show '%1' Overlay").arg(deviceName));
    if (device->nodeType() == WB_NODE_CAMERA) {
      action->setStatusTip(tr("Show overlay of camera device '%1' for robot '%2'.").arg(deviceName).arg(robotName));
      cameraActions << action;
    } else if (device->nodeType() == WB_NODE_RANGE_FINDER) {
      action->setStatusTip(tr("Show overlay of range-finder device '%1' for robot '%2'.").arg(deviceName).arg(robotName));
      rangeFinderActions << action;
    } else if (device->nodeType() == WB_NODE_DISPLAY) {
      action->setStatusTip(tr("Show overlay of display device '%1' for robot '%2'.").arg(deviceName).arg(robotName));
      displayActions << action;
    } else {
      delete action;
      continue;
    }
    action->setToolTip(mToggleFullScreenAction->statusTip());
    action->setCheckable(true);
    action->setChecked(device->isOverlayEnabled());
    action->setEnabled(!device->isWindowActive());
    action->setProperty("renderingDevice", QVariant::fromValue(static_cast<void *>(const_cast<OmRenderingDevice *>(device))));
    connect(action, &QAction::toggled, mSimulationView->view3D(), &OmView3D::setShowRenderingDevice);
    connect(device, &OmRenderingDevice::overlayVisibilityChanged, action, &QAction::setChecked);
    connect(device, &OmRenderingDevice::overlayStatusChanged, action, &QAction::setEnabled);
  }

  if (cameraActions.isEmpty() && rangeFinderActions.isEmpty() && displayActions.isEmpty()) {
    robotMenu->setEnabled(false);
    return;
  }

  QMenu *cameraMenu = robotMenu->addMenu(tr("Camera Devices"));
  if (cameraActions.isEmpty())
    cameraMenu->setEnabled(false);
  else {
    QListIterator<QAction *> actionIt(cameraActions);
    while (actionIt.hasNext())
      cameraMenu->addAction(actionIt.next());
  }
  QMenu *rangeFinderMenu = robotMenu->addMenu(tr("RangeFinder Devices"));
  if (rangeFinderActions.isEmpty())
    rangeFinderMenu->setEnabled(false);
  else {
    QListIterator<QAction *> actionIt(rangeFinderActions);
    while (actionIt.hasNext())
      rangeFinderMenu->addAction(actionIt.next());
  }
  QMenu *displayMenu = robotMenu->addMenu(tr("Display Devices"));
  if (displayActions.isEmpty())
    displayMenu->setEnabled(false);
  else {
    QListIterator<QAction *> actionIt(displayActions);
    while (actionIt.hasNext())
      displayMenu->addAction(actionIt.next());
  }
}

void OmMainWindow::updateOverlayMenu() {
  clearOverlaysMenu();

  if (!OmWorld::instance())
    return;

  QListIterator<OmRobot *> robotIt(OmWorld::instance()->robots());
  while (robotIt.hasNext())
    addRobotInOverlaysMenu(robotIt.next());
}

void OmMainWindow::updateProjectPath(const QString &oldPath, const QString &newPath) {
  updateWindowTitle();
  Q_UNUSED(oldPath);
  Q_UNUSED(newPath);
  mRecentFiles->makeRecent(OmWorld::instance()->fileName());
}

void OmMainWindow::createWorldLoadingProgressDialog() {
  if (mWorldLoadingProgressDialog)
    return;

  if (isMinimized() || mRunBackground)
    return;

  QPushButton *cancelButton = new QPushButton();
  cancelButton->setText(tr("Cancel"));
  cancelButton->setAutoDefault(false);
  cancelButton->setDefault(false);
  cancelButton->setChecked(false);

  mWorldLoadingProgressDialog = new QProgressDialog(this);
  mWorldLoadingProgressDialog->setModal(true);
  mWorldLoadingProgressDialog->setAutoClose(false);
  mWorldLoadingProgressDialog->show();
  OmGuiApplication::setWindowsDarkMode(mWorldLoadingProgressDialog);
  mWorldLoadingProgressDialog->setValue(0);
  mWorldLoadingProgressDialog->setWindowTitle(tr("Loading world"));
  mWorldLoadingProgressDialog->setLabelText(tr("Opening world file"));
  mWorldLoadingProgressDialog->setCancelButton(cancelButton);
  connect(mWorldLoadingProgressDialog, &QProgressDialog::canceled, OmApplication::instance(),
          &OmApplication::setWorldLoadingCanceled);
  QApplication::processEvents();
}

void OmMainWindow::deleteWorldLoadingProgressDialog() {
  if (mWorldLoadingProgressDialog) {
    disconnect(mWorldLoadingProgressDialog, &QProgressDialog::canceled, OmApplication::instance(),
               &OmApplication::setWorldLoadingCanceled);
    delete mWorldLoadingProgressDialog;
    mWorldLoadingProgressDialog = NULL;
  }
}

void OmMainWindow::setWorldLoadingProgress(const int progress) {
  if (mWorldLoadingProgressDialog) {
    mWorldLoadingProgressDialog->setValue(progress);
    QApplication::processEvents();
  }
}

void OmMainWindow::setWorldLoadingStatus(const QString &status) {
  if (mWorldLoadingProgressDialog) {
    mWorldLoadingProgressDialog->setLabelText(status);
    QApplication::processEvents();
  }
}

void OmMainWindow::startAnimationRecording() {
  const QString fileName = exportHtmlFiles();
  if (fileName.isEmpty())
    return;

  QString thumbnailFileName = fileName;
  thumbnailFileName.replace(QRegularExpression(".html$", QRegularExpression::CaseInsensitiveOption), ".jpg");
  mSimulationView->takeThumbnail(thumbnailFileName);

  OmAnimationRecorder::instance()->setStartFromGuiFlag(true);

  OmAnimationRecorder::instance()->start(fileName);
  toggleAnimationAction(true);

  OmSimulationState::instance()->resumeSimulation();
}

void OmMainWindow::stopAnimationRecording() {
  OmAnimationRecorder::instance()->stop();
  OmAnimationRecorder::instance()->setStartFromGuiFlag(false);
  toggleAnimationAction(false);
}

void OmMainWindow::toggleAnimationIcon() {
  static bool isRecOn = false;

  QAction *action = OmActionManager::instance()->action(OmAction::ANIMATION);
  if (!isRecOn) {
    action->setIcon(QIcon("enabledIcons:share_red_button.png"));
    isRecOn = true;
  } else {
    action->setIcon(QIcon("enabledIcons:share_button.png"));
    isRecOn = false;
  }
}

void OmMainWindow::toggleAnimationAction(bool isRecording) {
  QAction *action = OmActionManager::instance()->action(OmAction::ANIMATION);
  if (isRecording) {
    action->setText(tr("Stop HTML5 &Animation..."));
    action->setStatusTip(tr("Stop HTML5 animation recording."));
    action->setIcon(QIcon("enabledIcons:share_red_button.png"));
    disconnect(action, &QAction::triggered, this, &OmMainWindow::startAnimationRecording);
    connect(action, &QAction::triggered, this, &OmMainWindow::stopAnimationRecording, Qt::UniqueConnection);
    mAnimationRecordingTimer->start(800);
  } else {
    action->setText(tr("Record HTML5 &Animation..."));
    action->setStatusTip(tr("Record an HTML5 animation of the simulation to a local file."));
    QIcon icon = QIcon();
    icon.addFile("enabledIcons:share_button.png", QSize(), QIcon::Normal);
    icon.addFile("disabledIcons:share_button.png", QSize(), QIcon::Disabled);
    action->setIcon(icon);
    disconnect(action, &QAction::triggered, this, &OmMainWindow::stopAnimationRecording);
    connect(action, &QAction::triggered, this, &OmMainWindow::startAnimationRecording, Qt::UniqueConnection);
    mAnimationRecordingTimer->stop();
  }

  action->setToolTip(action->statusTip());
}

void OmMainWindow::enableAnimationAction() {
  OmActionManager::instance()->action(OmAction::ANIMATION)->setEnabled(true);
}

void OmMainWindow::disableAnimationAction() {
  OmActionManager::instance()->action(OmAction::ANIMATION)->setEnabled(false);
}

void OmMainWindow::logActiveControllersTermination() {
  const OmControlledWorld *controlledWorld = OmControlledWorld::instance();
  if (controlledWorld) {
    QStringList activeControllers = controlledWorld->activeControllersNames();
    foreach (QString controllerName, activeControllers)
      OmLog::info(tr("%1: Terminating.").arg(controllerName));
    QCoreApplication::processEvents();
  }
}

void OmMainWindow::openUrl(const QString &fileName, const QString &message, const QString &title) {
  if (OmMessageBox::question(message, this, title) == QMessageBox::Ok)
    OmDesktopServices::openUrl(QUrl::fromLocalFile(fileName).toString());
}

void OmMainWindow::prepareNodeRegeneration(OmNode *node) {
  // save devices perspective if node contains a rendering device
  // the device identification method could fail if the PROTO contains many
  // robots using the same device names, but usual node unique id cannot be used
  // because won't match before and after regeneration
  OmRenderingDeviceWindowFactory *factory = OmRenderingDeviceWindowFactory::instance();
  OmRenderingDevice *device;
  const QList<OmNode *> nodes = QList<OmNode *>() << const_cast<OmNode *>(node) << node->subNodes(true);
  foreach (OmNode *n, nodes) {
    device = dynamic_cast<OmRenderingDevice *>(n);
    if (device) {
      QStringList perspective = factory->windowPerspective(device);
      if (perspective.isEmpty())
        perspective = device->perspective();
      const OmRobot *robot = OmNodeUtilities::findRobotAncestor(device);
      assert(robot);
      mTemporaryProtoPerspectives.insert(robot->name() + "\n" + device->name(), perspective);
    }
  }
}

void OmMainWindow::finalizeNodeRegeneration(OmNode *node) {
  if (OmTemplateManager::isRegenerating())
    return;

  if (node != NULL && !mTemporaryProtoPerspectives.isEmpty()) {
    // apply temporary saved perspectives
    const QList<OmNode *> nodes = QList<OmNode *>() << node << node->subNodes(true);
    const OmRenderingDeviceWindowFactory *factory = OmRenderingDeviceWindowFactory::instance();
    foreach (OmNode *n, nodes) {
      OmRenderingDevice *device = dynamic_cast<OmRenderingDevice *>(n);
      if (device != NULL) {
        factory->listenToRenderingDevice(device);
        const OmRobot *robot = OmNodeUtilities::findRobotAncestor(device);
        assert(robot);
        const QString key = robot->name() + "\n" + device->name();
        QStringList perspective = mTemporaryProtoPerspectives.value(key);
        if (!perspective.isEmpty())
          device->restorePerspective(perspective);
        mTemporaryProtoPerspectives.remove(key);
      }
    }
    mTemporaryProtoPerspectives.clear();
  }
}
