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

#ifndef OM_MAIN_WINDOW_HPP
#define OM_MAIN_WINDOW_HPP

//
// Description: OmniSim main window
//

#include <QtCore/QMap>
#include <QtWidgets/QMainWindow>

#include "OmLog.hpp"

class OmAgentHud;
class OmConsole;
class OmNode;
class OmRecentFilesList;
class OmRobot;
class OmRobotWindow;
class OmSimulationView;
class OmTcpServer;

class QMenu;
class QMenuBar;
class QProgressDialog;
class QTimer;

// cppcheck-suppress noConstructor
class OmMainWindow : public QMainWindow {
  Q_OBJECT
  Q_PROPERTY(QString enabledIconPath MEMBER mEnabledIconPath READ enabledIconPath WRITE setEnabledIconPath)
  Q_PROPERTY(QString disabledIconPath MEMBER mDisabledIconPath READ disabledIconPath WRITE setDisabledIconPath)
  Q_PROPERTY(QString coreIconPath MEMBER mCoreIconPath READ coreIconPath WRITE setCoreIconPath)
  Q_PROPERTY(QString toolBarAlign MEMBER mToolBarAlign READ toolBarAlign WRITE setToolBarAlign)

public:
  explicit OmMainWindow(bool minimizedOnStart, OmTcpServer *tcpServer, QWidget *parent = NULL, bool runBackground = false);
  virtual ~OmMainWindow();

  void lockFullScreen(bool isLocked);
  void savePerspective(bool reloading, bool saveToFile, bool isSaveEvent = false);
  void restorePerspective(bool reloading, bool firstLoad, bool loadingFromMemory);

  const QString &enabledIconPath() const { return mEnabledIconPath; }
  const QString &disabledIconPath() const { return mDisabledIconPath; }
  const QString &coreIconPath() const { return mCoreIconPath; }
  const QString &toolBarAlign() const { return mToolBarAlign; }

  void setEnabledIconPath(const QString &path) { mEnabledIconPath = path; }
  void setDisabledIconPath(const QString &path) { mDisabledIconPath = path; }
  void setCoreIconPath(const QString &path) { mCoreIconPath = path; }
  void setToolBarAlign(const QString &align) { mToolBarAlign = align; }

  void restorePreferredGeometry(bool minimizedOnStart = false);

  void deleteRobotWindow(OmRobot *robot);

signals:
  void restartRequested();
  void splashScreenCloseRequested();

public slots:
  void loadDifferentWorld(const QString &fileName);
  void loadWorld(const QString &fileName, bool reloading = false);
  bool setFullScreen(bool isEnabled, bool isRecording = false, bool showDialog = true, bool startup = false);
  void showGuidedTour();
  void showUpdatedDialog();
  void setView3DSize(const QSize &size);
  void restoreRenderingDevicesPerspective();
  void resetWorldFromGui();

  QString exportHtmlFiles();
  void startAnimationRecording();

protected:
  bool event(QEvent *event) override;
  void closeEvent(QCloseEvent *event) override;

private slots:
  void updateBeforeWorldLoading(bool reloading);
  void updateAfterWorldLoading(bool reloading, bool firstLoad);
  void newWorld();
  void openWorld();
  void openSampleWorld();
  void saveWorld();
  void saveWorldAs(bool skipSimulationHasRunWarning = false);
  void reloadWorld();
  void resetGui(bool restartControllers);
  void showAboutBox();
  void show3DViewingInfo();
  void show3DMovingInfo();
  void show3DForceInfo();
  void showOpenGlInfo();
  void showUserGuide();
  void showReferenceManual();
  void showAutomobileDocumentation();

  void openGithubRepository();
  void openBugReport();

  void newProjectDirectory();
  void newRobotController();
  void newProto();
  void openPreferencesDialog();
  void setTheme(const QString &qssFile);
  void restoreLayout();
  void simulationEnabledChanged(bool);
  void showStatusBarMessage(OmLog::Level level, const QString &message);
  void showRobotWindow();
  void clearOverlaysMenu();
  void updateOverlayMenu();
  void updateRobotNameInOverlaysMenu();
  void addRobotInOverlaysMenu(OmRobot *robot);
  void removeRobotInOverlaysMenu(const OmRobot *robot);
  void createWorldLoadingProgressDialog();
  void deleteWorldLoadingProgressDialog();
  void setWorldLoadingProgress(const int progress);
  void setWorldLoadingStatus(const QString &status);
  void stopAnimationRecording();
  void toggleAnimationIcon();
  void toggleAnimationAction(bool isRecording);
  void enableAnimationAction();
  void disableAnimationAction();

private:
  void showHtmlRobotWindow(OmRobot *robot, bool manualTrigger);
  void closeClientRobotWindow(OmRobot *robot);
  void onSocketOpened();
  QList<OmRobotWindow *> mRobotWindows;
  QList<OmRobot *> mRobotsWaitingForWindowToOpen;
  bool mOnSocketOpen;
  bool mRobotWindowClosed;

  int mExitStatus;
  bool mRunBackground;  // when true, never realize the window as a top-level OS window
  QList<OmConsole *> mConsoles;
  OmAgentHud *mAgentHud;
  OmSimulationView *mSimulationView;
  OmRecentFilesList *mRecentFiles;
  QMenu *mRecentFilesSubMenu;
  QByteArray *mFactoryLayout;
  QMenu *mSimulationMenu;
  QMenuBar *mMenuBar;
  QMenu *mOverlayMenu;
  QMenu *mRobotCameraMenu;
  QMenu *mRobotRangeFinderMenu;
  QMenu *mRobotDisplayMenu;
  QAction *mToggleFullScreenAction;
  QAction *mExitFullScreenAction;
  QProgressDialog *mWorldLoadingProgressDialog;
  QTimer *mAnimationRecordingTimer;
  bool mIsFullScreenLocked;
  bool mWorldIsBeingDeleted;

  void createMainTools();
  void createMenus();

  QMenu *createFileMenu();
  QMenu *createEditMenu();
  QMenu *createViewMenu();
  QMenu *createSimulationMenu();
  QMenu *createOverlayMenu();
  QMenu *createToolsMenu();
  QMenu *createHelpMenu();
  bool proposeToSaveWorld(bool reloading = false);
  QString findHtmlFileName(const char *title);
  void enableToolsWidgetItems(bool enabled);
  void updateWindowTitle();
  void updateSimulationMenu();
  void writePreferences() const;
  void showDocument(const QString &url);
  bool runSimulationHasRunWarningMessage();
  void logActiveControllersTermination();
  void addDock(QWidget *);

  // maximized/minimize dock widgets
  QList<QWidget *> mDockWidgets;
  QWidget *mMaximizedWidget;
  QByteArray mMinimizedDockState;

  // temporarily save devices perspective during PROTO template regeneration
  QHash<QString, QStringList> mTemporaryProtoPerspectives;

  // QSS properties
  QString mEnabledIconPath, mDisabledIconPath, mCoreIconPath, mToolBarAlign;

  OmTcpServer *mTcpServer;

private slots:
  void showOnlineDocumentation(const QString &book, const QString &page = "index");
  void updateProjectPath(const QString &oldPath, const QString &newPath);
  void simulationQuit(int exitStatus);

  void maximizeDock();
  void minimizeDock();
  void setWidgetMaximized(QWidget *widget, bool maximized);

  void toggleFullScreen(bool enabled);
  void exitFullScreen();

  void openNewConsole(const QString &name = QString("Console"));
  void handleConsoleClosure();

  void openUrl(const QString &fileName, const QString &message, const QString &title);

  void prepareNodeRegeneration(OmNode *node);
  void discardNodeRegeneration() { finalizeNodeRegeneration(NULL); }
  void finalizeNodeRegeneration(OmNode *node);
};

#endif
