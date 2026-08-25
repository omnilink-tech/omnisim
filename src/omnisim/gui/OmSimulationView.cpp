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

#include "OmSimulationView.hpp"

#include "OmActionManager.hpp"
#include "OmApplication.hpp"
#include "OmContextMenuGenerator.hpp"
#include "OmControlledWorld.hpp"
#include "OmDockTitleBar.hpp"
#include "OmLog.hpp"
#include "OmMainWindow.hpp"
#include "OmMessageBox.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPreferences.hpp"
#include "OmProject.hpp"
#include "OmRobot.hpp"
#include "OmSceneTree.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSimulationStateIndicator.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmSoundEngine.hpp"
#include "OmStandardPaths.hpp"
#include "OmSysInfo.hpp"
#include "OmVideoRecorder.hpp"
#include "OmView3D.hpp"
#include "OmViewpoint.hpp"
#include "OmWgpuView.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"

#include <QtCore/QFileInfo>
#include <QtCore/QTimer>
#include <QtGui/QAction>
#include <QtGui/QImage>
#include <QtGui/QResizeEvent>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMenu>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>

enum { HIDE, SHOW };
enum { VIEW3D, MESSAGE };

OmSimulationView::OmSimulationView(QWidget *parent, const QString &toolBarAlign) :
  QWidget(parent),
  mIsScreenshotRequestedFromGui(false),
  mIsDecorationVisible(true),
  mToolBarExtensionMenu(NULL),
  mSelection(new OmSelection()),
  mSplitter(new QSplitter()),
  mSceneTree(new OmSceneTree(mSplitter)),
  mSplitterStatus(0),
  mSupervisorMovieRecordingEnabled(false) {
  // add container widget to 'hide' the black window by forcing it to
  // have the same position of the rendering window
  QWidget *const view3DVideoResizeWidget = new QWidget(mSplitter);
  mView3D = new OmView3D;
  mView3DContainer = QWidget::createWindowContainer(mView3D, view3DVideoResizeWidget);
  mView3DContainer->setMinimumSize(mView3D->minimumSize());
  mView3D->setParentWidget(mView3DContainer);
  mView3DContainer->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(mView3D, &OmView3D::contextMenuRequested, this, &OmSimulationView::showMenu);

  // it is preferable to resize the mView3DContainer instead of the mView3D
  // when creating a video so that the black window always has exactly the
  // same position than the rendering window
  // but we have to add some special stretching factor during the video
  // recording to fill the remaining available space otherwise the
  // toolbar and scene tree will be automatically resized as well
  mView3DResizeLayout = new QGridLayout(view3DVideoResizeWidget);
  mView3DResizeLayout->setSpacing(0);
  mView3DResizeLayout->setContentsMargins(0, 0, 0, 0);
  mView3DResizeLayout->addWidget(mView3DContainer, 0, 0);
  setView3DResizeStretch(false);

  // central widget
  mSplitter->setObjectName("horizontalSplitter");
  mSplitter->addWidget(mSceneTree);
  mSplitter->addWidget(view3DVideoResizeWidget);
  mSplitter->setStretchFactor(0, 0);
  mSplitter->setStretchFactor(1, 1);
  QList<int> initialSplitterSizes;
  initialSplitterSizes << 0 << (mSceneTree->sizeHint().width() + view3DVideoResizeWidget->sizeHint().width());
  mSplitter->setSizes(initialSplitterSizes);

  // R4 step-3b (opt-in via OMNISIM_VIEW3D_WGPU): embed a SECOND wgpu-rendered viewport as a
  // side-by-side pane next to the main view, so two renders of the same world + camera can be
  // compared in-app. Still a live debug lever post-F1 (the pane is an ADDITION, not a WREN
  // selector), so it stays functional rather than becoming a warned no-op. Default-off =>
  // layout byte-unchanged. F1: VALUE-PARSED per campaign rule 4 -- it was presence-gated, so
  // `OMNISIM_VIEW3D_WGPU=0` ARMED the pane instead of disarming it (the OMNISIM_REQUIRE_NEWTON
  // trap); now "0"/"false"/"off"/"no" (or set-but-empty) mean OFF.
  const QByteArray view3dWgpu = qgetenv("OMNISIM_VIEW3D_WGPU").trimmed().toLower();
  const bool view3dWgpuOn = qEnvironmentVariableIsSet("OMNISIM_VIEW3D_WGPU") && !view3dWgpu.isEmpty() &&
                            view3dWgpu != "0" && view3dWgpu != "false" && view3dWgpu != "off" && view3dWgpu != "no";
  if (view3dWgpuOn) {
    mWgpuView = new OmWgpuView();
    QWidget *const wgpuContainer = QWidget::createWindowContainer(mWgpuView, mSplitter);
    wgpuContainer->setMinimumWidth(160);
    mSplitter->addWidget(wgpuContainer);
    mSplitter->setStretchFactor(mSplitter->indexOf(wgpuContainer), 1);
  }

  mShowSceneTreeButton = new QToolButton(this);
  mShowSceneTreeButton->setObjectName("menuButton");
  mShowSceneTreeButton->setFocusPolicy(Qt::ClickFocus);
  connect(mShowSceneTreeButton, &QToolButton::pressed, this, &OmSimulationView::toggleSceneTreeVisibility);

  // main objects
  createActions();
  mTitleBar = new OmDockTitleBar(false, this);
  mToolBar = createToolBar();
  mNeedToRestoreBlackRenderingOverlay = false;
  mNeedToRestoreRendering = false;

  // top level layout
  QVBoxLayout *vlayout = new QVBoxLayout(this);
  vlayout->setSpacing(0);
  vlayout->setContentsMargins(0, 0, 0, 0);
  vlayout->addWidget(mTitleBar, 0);
  if (toolBarAlign == "center") {
    QHBoxLayout *hlayout = new QHBoxLayout();
    hlayout->addWidget(mToolBar);
    vlayout->addLayout(hlayout, 0);
  } else  // assuming left alignment
    vlayout->addWidget(mToolBar);
  vlayout->addWidget(mSplitter, 1);

  const OmSimulationState *state = OmSimulationState::instance();

  //  show a black screen if rendering is turned off
  if (!state->isRendering())
    renderABlackScreen();

  connect(mTitleBar, &OmDockTitleBar::closeClicked, this, &OmSimulationView::hide);
  connect(mTitleBar, &OmDockTitleBar::maximizeClicked, this, &OmSimulationView::needsMaximize);
  connect(mTitleBar, &OmDockTitleBar::minimizeClicked, this, &OmSimulationView::needsMinimize);
  connect(mSplitter, &QSplitter::splitterMoved, this, &OmSimulationView::needsActionsUpdate);
  connect(OmActionManager::instance()->action(OmAction::STEP), &QAction::triggered, mView3D, &OmView3D::unleashAndClean);
  connect(OmActionManager::instance()->action(OmAction::DISABLE_RENDERING), &QAction::triggered, this,
          &OmSimulationView::disableRendering);
  connect(mView3D, &OmView3D::applicationActionsUpdateRequested, mSceneTree, &OmSceneTree::updateApplicationActions);

  // video recording
  mRecordingTimer = new QTimer(this);
  connect(mRecordingTimer, &QTimer::timeout, this, &OmSimulationView::toggleRecordingIcon);
  connect(OmApplication::instance(), &OmApplication::requestScreenshot, this, &OmSimulationView::takeScreenshotAndSaveAs);
  connect(OmApplication::instance(), SIGNAL(videoCaptureStarted(const QString &, int, int, int, int, int, bool)), this,
          SLOT(startVideoCapture(const QString &, int, int, int, int, int, bool)));
  connect(OmApplication::instance(), &OmApplication::videoCaptureStopped, this, &OmSimulationView::stopVideoCapture);
  connect(OmVideoRecorder::instance(), &OmVideoRecorder::videoCreationStatusChanged, OmApplication::instance(),
          &OmApplication::videoCreationStatusChanged);

  OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parent);
  assert(mainWindow);
  OmVideoRecorder::setMainWindow(mainWindow);
  mWasMinimized = false;
}

OmSimulationView::~OmSimulationView() {
  // explicitly delete scene tree and 3D view before the selection
  delete mSceneTree;
  mSceneTree = NULL;

  delete mView3D;
  mView3D = NULL;

  delete mWgpuView;  // R4 step-3b embedded pane (NULL if OMNISIM_VIEW3D_WGPU unset)
  mWgpuView = NULL;

  delete mSelection;
}

QToolBar *OmSimulationView::createToolBar() {
  mToolBar = new QToolBar(this);

  OmActionManager *manager = OmActionManager::instance();

  mToolBar->addWidget(mShowSceneTreeButton);

  QAction *action = manager->action(OmAction::ADD_NEW);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  mToolBar->addSeparator();

  action = manager->action(OmAction::RESTORE_VIEWPOINT);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(OmAction::VIEW_MENU);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");
  QToolButton *viewMenuButton = dynamic_cast<QToolButton *>(mToolBar->widgetForAction(action));
  viewMenuButton->setPopupMode(QToolButton::InstantPopup);
  QMenu *viewMenu = new QMenu(viewMenuButton);
  viewMenu->addAction(manager->action(OmAction::EAST_VIEW));
  viewMenu->addAction(manager->action(OmAction::WEST_VIEW));
  viewMenu->addAction(manager->action(OmAction::NORTH_VIEW));
  viewMenu->addAction(manager->action(OmAction::SOUTH_VIEW));
  viewMenu->addAction(manager->action(OmAction::TOP_VIEW));
  viewMenu->addAction(manager->action(OmAction::BOTTOM_VIEW));
  viewMenuButton->setMenu(viewMenu);

  mToolBar->addSeparator();

  mToolBarExtensionMenu = mToolBar->findChild<QMenu *>();
  connect(mToolBarExtensionMenu, &QMenu::aboutToShow, this, &OmSimulationView::hideInappropriateToolBarItems);

  action = manager->action(OmAction::OPEN_WORLD);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(OmAction::SAVE_WORLD);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(OmAction::RELOAD_WORLD);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  mToolBar->addSeparator();

  mToolBar->addWidget(new OmSimulationStateIndicator(mToolBar));

  action = manager->action(OmAction::RESET_SIMULATION);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = manager->action(OmAction::STEP);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  action = mPlayAnchor = new QAction(this);  // anchor to help replacing actions
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("invisibleButton");
  mToolBar->widgetForAction(action)->setVisible("false");

  updatePlayButtons();
  connect(OmApplication::instance(), &OmApplication::postWorldLoaded, this, &OmSimulationView::updatePlayButtons);
  connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this, &OmSimulationView::updatePlayButtons);

  action = manager->action(OmAction::RENDERING);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  OmActionManager::instance()->updateRenderingButton();
  connect(OmSimulationState::instance(), &OmSimulationState::renderingStateChanged, this, &OmSimulationView::updateRendering);

  mToolBar->addSeparator();

  mToolBar->addAction(mTakeScreenshotAction);
  mToolBar->widgetForAction(mTakeScreenshotAction)->setObjectName("menuButton");

  mToolBar->addAction(mMovieAction);
  mToolBar->widgetForAction(mMovieAction)->setObjectName("menuButton");

  action = manager->action(OmAction::ANIMATION);
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("menuButton");

  mToolBar->addSeparator();

  action = mSoundAnchor = new QAction(this);  // anchor to help replacing actions
  mToolBar->addAction(action);
  mToolBar->widgetForAction(action)->setObjectName("invisibleButton");
  mToolBar->widgetForAction(action)->setVisible("false");

  mSoundVolumeSlider = new QSlider(Qt::Horizontal, this);

  if (OmPreferences::instance()->value("Sound/mute", false).toBool())
    mSoundVolumeSlider->setSliderPosition(0);
  else
    mSoundVolumeSlider->setSliderPosition(OmPreferences::instance()->value("Sound/volume", 80).toInt());

  mSoundVolumeSlider->setFocusPolicy(Qt::ClickFocus);
  mSoundVolumeSlider->setFixedWidth(102);
  mToolBar->addWidget(mSoundVolumeSlider);
  connect(mSoundVolumeSlider, &QSlider::valueChanged, this, &OmSimulationView::updateSoundVolume);

  updateSoundButtons();

  return mToolBar;
}

void OmSimulationView::createActions() {
  QAction *action;

  action = mToggleView3DAction = new QAction(this);
  action->setCheckable(true);
  action->setChecked(true);
  action->setText(tr("3D View"));
  action->setStatusTip("Toggle the 3D View.");
  action->setShortcut(Qt::CTRL | Qt::Key_B);
  connect(action, &QAction::toggled, this, &OmSimulationView::updateVisibility);

  action = mToggleSceneTreeAction = new QAction(this);
  action->setCheckable(true);
  action->setChecked(true);
  action->setText(tr("Scene Tree"));
  action->setStatusTip("Toggle the scene tree.");
  action->setShortcut(Qt::CTRL | Qt::Key_T);
  connect(action, &QAction::toggled, this, &OmSimulationView::updateVisibility);

  updateSceneTreeActions(true);

  // TODO: for sure there is a clever location to do the following connections
  //       (this has nothing to do with a window)
  OmActionManager *manager = OmActionManager::instance();
  connect(manager->action(OmAction::PAUSE), &QAction::triggered, this, &OmSimulationView::pause);
  connect(manager->action(OmAction::STEP), &QAction::triggered, this, &OmSimulationView::step);
  connect(manager->action(OmAction::REAL_TIME), &QAction::triggered, this, &OmSimulationView::realTime);
  connect(manager->action(OmAction::FAST), &QAction::triggered, this, &OmSimulationView::fast);
  connect(manager->action(OmAction::RENDERING), &QAction::triggered, this, &OmSimulationView::toggleRendering);

  // add actions available in full-screen mode to the current widget
  // otherwise they will be automatically disabled when the toolbar is hidden
  addAction(manager->action(OmAction::PAUSE));
  addAction(manager->action(OmAction::STEP));
  addAction(manager->action(OmAction::REAL_TIME));
  addAction(manager->action(OmAction::FAST));
  addAction(manager->action(OmAction::RENDERING));
  addAction(manager->action(OmAction::DEL));
  addAction(manager->action(OmAction::MOVE_VIEWPOINT_TO_OBJECT));

  mMovieAction = new QAction(this);
  toggleMovieAction(false);

  mTakeScreenshotAction = manager->action(OmAction::TAKE_SCREENSHOT);
  connect(mTakeScreenshotAction, &QAction::triggered, this, &OmSimulationView::takeScreenshot);
  // so taking screenshots can be done in full-screen mode, when the toolbar is hidden
  addAction(mTakeScreenshotAction);

  connect(manager->action(OmAction::SOUND_UNMUTE), &QAction::triggered, this, &OmSimulationView::unmuteSound);
  connect(manager->action(OmAction::SOUND_MUTE), &QAction::triggered, this, &OmSimulationView::muteSound);
}

void OmSimulationView::setMaximized(bool maximized) {
  mTitleBar->setMaximized(maximized);
}

void OmSimulationView::setDecorationVisible(bool visible) {
  static QList<QByteArray> previousState;

  if (visible) {
    restoreState(previousState);
    previousState.clear();
  } else
    previousState = saveState();

  mSceneTree->setVisible(visible);
  mToolBar->setVisible(visible);
  mTitleBar->setVisible(visible);
  mIsDecorationVisible = visible;
}

bool OmSimulationView::isSceneTreeButtonStatusVisible() const {
  return mShowSceneTreeButton->property("state") == HIDE;
}

void OmSimulationView::updateSceneTreeActions(bool enabled) {
  if (enabled) {
    // side bar visible
    mShowSceneTreeButton->setStatusTip(tr("Hide the Scene Tree side bar."));
    mShowSceneTreeButton->setIcon(QIcon("enabledIcons:hide_side_bar.png"));
    mShowSceneTreeButton->setToolTip(mShowSceneTreeButton->statusTip());
    mShowSceneTreeButton->setProperty("state", HIDE);
  } else {
    // side bar hidden
    mShowSceneTreeButton->setStatusTip(tr("Show the Scene Tree side bar."));
    mShowSceneTreeButton->setIcon(QIcon("enabledIcons:show_side_bar.png"));
    mShowSceneTreeButton->setToolTip(mShowSceneTreeButton->statusTip());
    mShowSceneTreeButton->setProperty("state", SHOW);
  }

  mToggleSceneTreeAction->blockSignals(true);
  mToggleSceneTreeAction->setChecked(enabled);
  mToggleSceneTreeAction->blockSignals(false);
}

void OmSimulationView::updateToggleView3DAction(bool enabled) {
  mToggleView3DAction->blockSignals(true);
  mToggleView3DAction->setChecked(enabled);
  mToggleView3DAction->blockSignals(false);
}

void OmSimulationView::needsActionsUpdate(int position, int index) {
  static bool hidden = false;

  if (position == 0 && !hidden) {
    updateSceneTreeActions(false);
    hidden = true;
  } else if (position > 0 && hidden) {
    updateSceneTreeActions(true);
    hidden = false;
  }

  updateToggleView3DAction(mView3D->width() > 1);
}

void OmSimulationView::toggleSceneTreeVisibility() {
  static int lastSplitterPosition = -1;

  setUpdatesEnabled(false);

  bool show = (mShowSceneTreeButton->property("state") == SHOW);
  if (show) {
    // show scene tree
    if (lastSplitterPosition <= 0)
      lastSplitterPosition = mSceneTree->sizeHint().width();

    const int view3DWidth = mView3D->width();
    if (lastSplitterPosition >= view3DWidth)
      lastSplitterPosition = view3DWidth / 2;

    QList<int> sizes = QList<int>() << lastSplitterPosition << (view3DWidth - lastSplitterPosition);
    mSplitter->setSizes(sizes);

  } else {
    // hide scene tree
    lastSplitterPosition = mSceneTree->width();
    QList<int> sizes = QList<int>() << 0 << (lastSplitterPosition + mView3D->width());
    mSplitter->setSizes(sizes);
    updateToggleView3DAction(true);
  }

  updateSceneTreeActions(show);

  setUpdatesEnabled(true);
}

void OmSimulationView::setView3DVisibility(bool visible) {
  static int lastSplitterPosition = -1;
  const int view3DWidth = mView3D->width();

  if (!visible && (view3DWidth > 0)) {
    // hide view 3D
    lastSplitterPosition = view3DWidth;

    QList<int> sizes = QList<int>() << (lastSplitterPosition + mSceneTree->width()) << 0;
    mSplitter->setSizes(sizes);
    updateToggleView3DAction(false);

  } else if (visible && view3DWidth <= 1) {
    // show view 3D
    if (lastSplitterPosition <= 0)
      lastSplitterPosition = mView3D->sizeHint().width();

    const int sceneTreeWidth = mSceneTree->width();
    if (lastSplitterPosition >= sceneTreeWidth)
      lastSplitterPosition = sceneTreeWidth / 2;

    QList<int> sizes = QList<int>() << (sceneTreeWidth - lastSplitterPosition) << lastSplitterPosition;
    mSplitter->setSizes(sizes);
    updateToggleView3DAction(true);
  }
}

void OmSimulationView::updateVisibility() {
  const bool isView3DVisible = mToggleView3DAction->isChecked();
  const bool isSceneTreeVisible = mToggleSceneTreeAction->isChecked();
  const bool isSimulationViewVisible = isView3DVisible || isSceneTreeVisible;

  if (isSimulationViewVisible) {
    setView3DVisibility(isView3DVisible);
    if (isSceneTreeVisible != isSceneTreeButtonStatusVisible())
      toggleSceneTreeVisibility();
  }

  const bool show = isSimulationViewVisible && !isVisible();
  QSize tempMinimumSize;
  if (show) {
    // hack to restore simulation view size
    tempMinimumSize = minimumSize();
    setMinimumSize(mLastSize);
  }

  setVisible(isSimulationViewVisible);

  if (show)
    // restore minimum size
    setMinimumSize(tempMinimumSize);
}

void OmSimulationView::unmuteSound() {
  if (!OmSoundEngine::openAL()) {
    OmLog::warning("no audio device found.");
    return;
  }
  OmPreferences::instance()->setValue("Sound/mute", false);
  const OmSimulationState::Mode mode = OmSimulationState::instance()->mode();
  if (mode != OmSimulationState::FAST && OmSimulationState::instance()->isRendering())
    OmSoundEngine::setMute(false);
  mSoundVolumeSlider->setSliderPosition(OmPreferences::instance()->value("Sound/volume", 80).toInt());
  connect(mSoundVolumeSlider, &QSlider::valueChanged, this, &OmSimulationView::updateSoundVolume);
  updateSoundButtons();
}

void OmSimulationView::muteSound() {
  OmPreferences::instance()->setValue("Sound/mute", true);
  disconnect(mSoundVolumeSlider, &QSlider::valueChanged, this, &OmSimulationView::updateSoundVolume);
  mSoundVolumeSlider->setSliderPosition(0);
  OmSoundEngine::setMute(true);
  updateSoundButtons();
}

void OmSimulationView::updateSoundVolume(int volume) {
  OmSoundEngine::setVolume(volume);
  OmPreferences::instance()->setValue("Sound/volume", volume);
}

void OmSimulationView::hideInappropriateToolBarItems() {
  foreach (QAction *const action, mToolBarExtensionMenu->actions()) {
    // widgets that aren't de facto menu actions (speedometer and volume slider)
    // have blank action text and aren't parented by the toolbar. We need to check
    // the parent as menu separators have blank text but are always parented by the
    // QToolBar instance
    if (action->text().isEmpty() && qobject_cast<QWidget *>(action->parent()) != mToolBar)
      action->setVisible(false);
  }
}

void OmSimulationView::toggleMovieAction(bool isRecording) {
  if (isRecording) {
    mMovieAction->setText(tr("Stop &Movie..."));
    mMovieAction->setStatusTip(tr("Stop video recording."));
    mMovieAction->setIcon(QIcon("enabledIcons:movie_red_button.png"));
    disconnect(mMovieAction, &QAction::triggered, this, &OmSimulationView::makeMovie);
    connect(mMovieAction, &QAction::triggered, this, &OmSimulationView::stopMovie);
  } else {
    mMovieAction->setText(tr("Make &Movie..."));
    mMovieAction->setStatusTip(tr("Start video recording of the current simulation."));
    mMovieAction->setIcon(QIcon("enabledIcons:movie_black_button.png"));
    disconnect(mMovieAction, &QAction::triggered, this, &OmSimulationView::stopMovie);
    connect(mMovieAction, &QAction::triggered, this, &OmSimulationView::makeMovie);
  }

  mMovieAction->setToolTip(mMovieAction->statusTip());
}

void OmSimulationView::toggleRecordingIcon() {
  static bool isRecOn = false;

  if (!isRecOn) {
    mMovieAction->setIcon(QIcon("enabledIcons:movie_red_button.png"));
    isRecOn = true;
  } else {
    mMovieAction->setIcon(QIcon("enabledIcons:movie_black_button.png"));
    isRecOn = false;
  }
}

void OmSimulationView::startVideoCapture(const QString &fileName, int codec, int width, int height, int quality,
                                         int acceleration, bool showCaption) {
  OmVideoRecorder *videoRecorder = OmVideoRecorder::instance();
  const bool success = videoRecorder->initRecording(this, OmWorld::instance()->worldInfo()->basicTimeStep(),
                                                    QSize(width, height), quality, codec, acceleration, showCaption, fileName);
  if (success) {
    mSupervisorMovieRecordingEnabled = true;
    mRecordingTimer->start(800);
    toggleMovieAction(true);
    mTakeScreenshotAction->setEnabled(false);
    showRenderingIfNecessary();
    OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parentWidget());
    if (mainWindow->isMinimized()) {
      mWasMinimized = true;
      mainWindow->showMaximized();
    }
  }
}

void OmSimulationView::stopVideoCapture(bool canceled) {
  OmVideoRecorder::instance()->stopRecording(canceled);
  restoreNoRenderingIfNecessary();
  if (mWasMinimized) {
    OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parentWidget());
    mainWindow->showMinimized();
    mWasMinimized = false;
  }
  // re-enable take screenshot action
  mTakeScreenshotAction->setEnabled(true);
  mRecordingTimer->stop();
  toggleMovieAction(false);
  mSupervisorMovieRecordingEnabled = false;
}

void OmSimulationView::cancelSupervisorMovieRecording() {
  if (mSupervisorMovieRecordingEnabled) {
    mView3D->resetScreenshotRequest();
    stopVideoCapture(true);
  }
}

void OmSimulationView::stopMovie() {
  stopVideoCapture();
  OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parentWidget());
  mainWindow->restorePerspective(false, false, true);
}

void OmSimulationView::makeMovie() {
  if (!OmSimulationState::instance()->isRendering()) {
    OmLog::warning(tr("Impossible to record a movie while rendering is turned off."), true);
    return;
  }

  // pause simulation before recording video
  OmSimulationState::Mode currentMode = OmSimulationState::instance()->mode();
  if (!OmSimulationState::instance()->isPaused())
    pause();

  // store our perspective for when we stop
  OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parentWidget());
  mainWindow->savePerspective(false, false);

  OmVideoRecorder *videoRecorder = OmVideoRecorder::instance();
  bool success = videoRecorder->initRecording(this, OmWorld::instance()->worldInfo()->basicTimeStep());
  if (success) {
    mRecordingTimer->start(800);
    toggleMovieAction(true);
    // disable take screenshot action
    mTakeScreenshotAction->setEnabled(false);
  }

  // reset current simulation mode
  OmSimulationState::instance()->setMode(currentMode);
  updateBlackRenderingOverlay();
}

void OmSimulationView::showRenderingIfNecessary() {
  // remove "No Rendering" overlay if necessary
  if (!OmSimulationState::instance()->isRendering()) {
    OmSimulationState::instance()->setRendering(true);
    mView3D->hideBlackRenderingOverlay();
    mNeedToRestoreBlackRenderingOverlay = true;
  }
}

void OmSimulationView::restoreNoRenderingIfNecessary() {
  if (mNeedToRestoreBlackRenderingOverlay) {
    mView3D->showBlackRenderingOverlay();
    OmSimulationState::instance()->setRendering(false);
    mNeedToRestoreBlackRenderingOverlay = false;
  }
}

void OmSimulationView::writeScreenshot() {
  const QImage &image = mView3D->grabWindowBufferNow();
  disconnect(mView3D, &OmView3D::screenshotReady, this, &OmSimulationView::writeScreenshot);

  while (!mScreenshotFileNameList.isEmpty() && !mScreenshotQualityList.isEmpty()) {
    const QString filename = mScreenshotFileNameList.takeFirst();
    if (!image.save(filename, 0, mScreenshotQualityList.takeFirst()))
      OmLog::error(QString("Error while writing file: %1").arg(filename));
    else if (mIsScreenshotRequestedFromGui && mIsDecorationVisible)
      emit requestOpenUrl(filename, tr("The screenshot has been created:\n%1\n\nDo you want to open it now?").arg(filename),
                          tr("Take Screenshot"));
  }

  if (mIsScreenshotRequestedFromGui) {
    OmSimulationState::instance()->resumeSimulation();
    mIsScreenshotRequestedFromGui = false;
  }
  restoreNoRenderingIfNecessary();

  if (mWasMinimized) {
    OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parentWidget());
    mainWindow->showMinimized();
    mWasMinimized = false;
  }

  emit screenshotWritten();
}

void OmSimulationView::takeScreenshotAndSaveAs(const QString &fileName, int quality) {
  mScreenshotQualityList.append(quality);
  mScreenshotFileNameList.append(fileName);
  OmMainWindow *mainWindow = dynamic_cast<OmMainWindow *>(parentWidget());
  if (mainWindow->isMinimized()) {
    mWasMinimized = true;
    mainWindow->showMaximized();
  }
  // In fullscreen mode we don't have a handy dialog to delay things for us so
  // we must ensure the OpenGL context is correct, delaying the screenshot like
  // we do for movies. We can only ask for a screenshot if the view3D is definitely
  // ready.
  if (OmSimulationState::instance()->isPaused() && mIsDecorationVisible) {
    writeScreenshot();
    return;
  }
  connect(mView3D, &OmView3D::screenshotReady, this, &OmSimulationView::writeScreenshot);
  showRenderingIfNecessary();
  mView3D->requestScreenshot();
  mView3D->refresh();

  if (mIsScreenshotRequestedFromGui) {
    OmSimulationState::instance()->resumeSimulation();
    repaintView3D();
  }
}

void OmSimulationView::takeScreenshot() {
  OmSimulationState *simulationState = OmSimulationState::instance();
  simulationState->pauseSimulation();

  static QString otherFilters = tr("Images (*.png *.jpg *.jpeg)");
  static QString dialogCaption = tr("Save as...");

  QFileInfo fi(OmWorld::instance()->fileName());
  QString worldBaseName = fi.baseName();

  QString fileName;
  for (int i = 0; i < 1000; ++i) {
    QString suffix = i == 0 ? "" : QString("_%1").arg(i);
    fileName = OmPreferences::instance()->value("Directories/screenshots").toString() + worldBaseName + suffix + ".png";
    if (!QFileInfo::exists(fileName))
      break;
  }

  // show "save as" dialog when not in full-screen mode
  if (mIsDecorationVisible)
    fileName = QFileDialog::getSaveFileName(this, dialogCaption, OmProject::computeBestPathForSaveAs(fileName), otherFilters);

  if (fileName.isEmpty()) {
    simulationState->resumeSimulation();
    return;
  }

  OmPreferences::instance()->setValue("Directories/screenshots", QFileInfo(fileName).absolutePath() + "/");

  QFileInfo file(fileName);
  QString suffix = file.suffix();
  if (suffix.toLower() == "png" || suffix.toLower() == "jpg" || suffix.toLower() == "jpeg") {
    mIsScreenshotRequestedFromGui = true;
    takeScreenshotAndSaveAs(fileName);
    return;
  }

  if (suffix.isEmpty())
    OmMessageBox::warning(tr("Unable to save screenshot because the file format is missing. "
                             "Please use the '.png', '.jpg' or '.jpeg' file extension to set the file format.")
                            .arg(suffix),
                          this, tr("Missing file format"));
  else
    OmMessageBox::warning(tr("Unable to save screenshot because the '.%1' format is unsupported. "
                             "Please use only the '.png', '.jpg' or '.jpeg' file extension.")
                            .arg(suffix),
                          this, tr("Unsupported file format"));

  simulationState->resumeSimulation();
}

void OmSimulationView::takeThumbnail(const QString &fileName) {
  if (!OmPreferences::instance()->value("General/thumbnail").toBool()) {
    emit thumbnailTaken();
    return;
  }

  mThumbnailFileName = fileName;
  mSizeBeforeThumbnail.setWidth(mView3DContainer->width());
  mSizeBeforeThumbnail.setHeight(mView3DContainer->height());

  mView3D->disableOptionalRenderingAndOverLays();

  const QSize thumnailSize(768, 432);
  enableView3DFixedSize(thumnailSize);
  connect(mView3D, &OmView3D::resized, this, &OmSimulationView::takeScreesnhotForThumbnail);
}

void OmSimulationView::takeScreesnhotForThumbnail() {
  disconnect(mView3D, &OmView3D::resized, this, &OmSimulationView::takeScreesnhotForThumbnail);
  connect(mView3D, &OmView3D::screenshotReady, this, &OmSimulationView::writeScreenshotForThumbnail);
  mView3D->requestScreenshot();
}

void OmSimulationView::writeScreenshotForThumbnail() {
  disconnect(mView3D, &OmView3D::screenshotReady, this, &OmSimulationView::writeScreenshotForThumbnail);
  connect(this, &OmSimulationView::screenshotWritten, this, &OmSimulationView::restoreViewAfterThumbnail);
  takeScreenshotAndSaveAs(mThumbnailFileName);
}

void OmSimulationView::restoreViewAfterThumbnail() {
  disconnect(this, &OmSimulationView::screenshotWritten, this, &OmSimulationView::restoreViewAfterThumbnail);
  mView3D->restoreOptionalRenderingAndOverLays();
  enableView3DFixedSize(mSizeBeforeThumbnail);
  disableView3DFixedSize();
  emit thumbnailTaken();
}

void OmSimulationView::pause() {
  repaintView3D();  // update 3D view if not refreshed after last step
  OmSimulationState::instance()->setMode(OmSimulationState::PAUSE);
}

void OmSimulationView::step() {
  OmSimulationState::instance()->setMode(OmSimulationState::STEP);
  OmSimulationWorld::instance()->step();
  OmSimulationState::instance()->setMode(OmSimulationState::PAUSE);
}

void OmSimulationView::realTime() {
  OmSimulationState::instance()->setMode(OmSimulationState::REALTIME);
}

void OmSimulationView::fast() {
  OmSimulationState::instance()->setMode(OmSimulationState::FAST);
}

void OmSimulationView::disableRendering(bool disabled) {
  if (disabled) {
    mNeedToRestoreRendering = OmSimulationState::instance()->isRendering();
    OmSimulationState::instance()->setRendering(false);
  } else if (mNeedToRestoreRendering) {
    mNeedToRestoreRendering = false;
    OmSimulationState::instance()->setRendering(true);
  }

  OmActionManager::instance()->action(OmAction::RENDERING)->setEnabled(!disabled);
  mView3D->setUserInteractionDisabled(OmAction::DISABLE_RENDERING, disabled);
}

void OmSimulationView::toggleRendering() {
  OmSimulationState::instance()->setRendering(!OmSimulationState::instance()->isRendering());
}

void OmSimulationView::updateBlackRenderingOverlay() {
  if (OmSimulationState::instance()->isRendering())
    retrieveSimulationView();
  else
    renderABlackScreen();
}

void OmSimulationView::prepareWorldLoading() {
  mSceneTree->prepareWorldLoading();
  mView3D->prepareWorldLoading();

  disconnect(mSceneTree, &OmSceneTree::valueChangedFromGui, mView3D, &OmView3D::renderLater);

  // solid selection
  disconnect(mSelection, &OmSelection::visibleHandlesChanged, mView3D, &OmView3D::renderLater);
  disconnect(mSelection, &OmSelection::selectionChangedFromSceneTree, mView3D, &OmView3D::renderLater);
  disconnect(mSelection, &OmSelection::selectionChangedFromView3D, mSceneTree, &OmSceneTree::selectPose);
  disconnect(mSelection, &OmSelection::selectionConfirmedFromView3D, mSceneTree, &OmSceneTree::selectPose);
  disconnect(mSceneTree, &OmSceneTree::nodeSelected, mSelection, &OmSelection::selectNodeFromSceneTree);
}

void OmSimulationView::setWorld(OmSimulationWorld *w) {
  // first set world in mView3D and then in mSceneTree
  // otherwise mView3D receives a selection changed signal from mSceneTree when no
  // world is load and the selection in the two views will mismatch
  mView3D->setWorld(w);
  mSceneTree->setWorld(w);
  updateTitleBarTitle();
  connect(w->worldInfo(), &OmWorldInfo::titleChanged, this, &OmSimulationView::updateTitleBarTitle);
  connect(w, &OmSimulationWorld::physicsStepEnded, OmSelection::instance(),
          &OmSelection::propagateBoundingObjectMaterialUpdate);
  OmControlledWorld *cw = dynamic_cast<OmControlledWorld *>(w);
  assert(cw);
  connect(cw, &OmControlledWorld::stepBlocked, this, &OmSimulationView::disableStepButton);

  // update save action based on simulation world state
  OmActionManager::instance()->setEnabled(OmAction::SAVE_WORLD, false);
  connect(w, &OmWorld::modificationChanged, OmActionManager::instance()->action(OmAction::SAVE_WORLD), &QAction::setEnabled);
  connect(w, &OmSimulationWorld::simulationStartedAfterSave, OmActionManager::instance()->action(OmAction::SAVE_WORLD),
          &QAction::setEnabled);

  connect(mSceneTree, &OmSceneTree::valueChangedFromGui, mView3D, &OmView3D::renderLater);

  // solid selection
  connect(mSelection, &OmSelection::visibleHandlesChanged, mView3D, &OmView3D::renderLater);
  connect(mSelection, &OmSelection::selectionChangedFromSceneTree, mView3D, &OmView3D::renderLater);
  connect(mSelection, &OmSelection::selectionChangedFromView3D, mSceneTree, &OmSceneTree::selectPose);
  connect(mSelection, &OmSelection::selectionConfirmedFromView3D, mSceneTree, &OmSceneTree::selectPose);
  connect(mSceneTree, &OmSceneTree::nodeSelected, mSelection, &OmSelection::selectNodeFromSceneTree);

  OmActionManager *const actionManager = OmActionManager::instance();
  OmViewpoint *const viewpoint = OmSimulationWorld::instance()->viewpoint();
  connect(actionManager->action(OmAction::SOUTH_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::southView);
  connect(actionManager->action(OmAction::NORTH_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::northView);
  connect(actionManager->action(OmAction::EAST_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::eastView);
  connect(actionManager->action(OmAction::WEST_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::westView);
  connect(actionManager->action(OmAction::TOP_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::topView);
  connect(actionManager->action(OmAction::BOTTOM_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::bottomView);
  connect(actionManager->action(OmAction::OBJECT_FRONT_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::objectFrontView);
  connect(actionManager->action(OmAction::OBJECT_BACK_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::objectBackView);
  connect(actionManager->action(OmAction::OBJECT_LEFT_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::objectLeftView);
  connect(actionManager->action(OmAction::OBJECT_RIGHT_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::objectRightView);
  connect(actionManager->action(OmAction::OBJECT_TOP_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::objectTopView);
  connect(actionManager->action(OmAction::OBJECT_BOTTOM_VIEW), &QAction::triggered, viewpoint, &OmViewpoint::objectBottomView);

  connect(OmVideoRecorder::instance(), &OmVideoRecorder::videoCreationStatusChanged, w, &OmWorld::updateVideoRecordingStatus);
  w->updateVideoRecordingStatus(OmVideoRecorder::instance()->isRecording() ? WB_SUPERVISOR_MOVIE_RECORDING :
                                                                             WB_SUPERVISOR_MOVIE_READY);

  mSceneTree->updateSelection();
  disableStepButton(false);
}

// update title bar's title
void OmSimulationView::updateTitleBarTitle() {
  OmWorld *world = OmSimulationWorld::instance();
  const QString &title = world->worldInfo()->title();
  if (title.isEmpty())
    mTitleBar->setTitle(tr("Simulation View"));
  else
    mTitleBar->setTitle(title);
}

void OmSimulationView::repaintView3D() {
  mView3D->renderLater();
}

void OmSimulationView::renderABlackScreen() {
  if (mView3D)
    mView3D->showBlackRenderingOverlay();
}

void OmSimulationView::retrieveSimulationView() {
  if (mView3D)
    mView3D->hideBlackRenderingOverlay();
}

void OmSimulationView::modeKeyPressed(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_0:
      // Ctrl + 0
      pause();
      return;
    case Qt::Key_1:
      // Ctrl + 1
      step();
      return;
    case Qt::Key_2:
      // Ctrl + 2
      realTime();
      return;
    case Qt::Key_3:
      // Ctrl + 3
      fast();
      return;
    case Qt::Key_4:
      // Ctrl + 4
      toggleRendering();
      return;
    default:
      break;
  }
}

void OmSimulationView::disableStepButton(bool disabled) {
  OmActionManager::instance()->action(OmAction::STEP)->setEnabled(!disabled);
}

void OmSimulationView::updatePlayButtons() {
  mToolBar->setUpdatesEnabled(false);

  OmActionManager *manager = OmActionManager::instance();

  QAction *pauseMode = manager->action(OmAction::PAUSE);
  QAction *realtimeMode = manager->action(OmAction::REAL_TIME);
  QAction *fastMode = manager->action(OmAction::FAST);

  mToolBar->removeAction(pauseMode);
  mToolBar->removeAction(realtimeMode);
  mToolBar->removeAction(fastMode);

  QList<QAction *> actions;

  switch (OmSimulationState::instance()->mode()) {
    case OmSimulationState::REALTIME:
      actions << pauseMode << fastMode;
      break;

    case OmSimulationState::FAST:
      actions << realtimeMode << pauseMode;
      break;

    default:  // PAUSE
      actions << realtimeMode << fastMode;
      break;
  }

  mToolBar->insertActions(mPlayAnchor, actions);

  // setObjectName (used by the stylesheet)
  QWidget *pauseWidget = mToolBar->widgetForAction(pauseMode);
  QWidget *realTimeWidget = mToolBar->widgetForAction(realtimeMode);
  QWidget *fastWidget = mToolBar->widgetForAction(fastMode);
  if (fastWidget)
    fastWidget->setObjectName("menuButton");
  if (realTimeWidget)
    realTimeWidget->setObjectName("menuButton");
  if (pauseWidget)
    pauseWidget->setObjectName("menuButton");

  mToolBar->update();

  mToolBar->setUpdatesEnabled(true);
}

void OmSimulationView::updateRendering() {
  OmActionManager::instance()->updateRenderingButton();
  updateBlackRenderingOverlay();
}

void OmSimulationView::updateSoundButtons() {
  mToolBar->setUpdatesEnabled(false);

  OmActionManager *manager = OmActionManager::instance();

  QAction *soundUnmuteAction = manager->action(OmAction::SOUND_UNMUTE);
  QAction *soundMuteAction = manager->action(OmAction::SOUND_MUTE);

  mToolBar->removeAction(soundUnmuteAction);
  mToolBar->removeAction(soundMuteAction);

  bool mute = OmPreferences::instance()->value("Sound/mute", true).toBool();
  mSoundVolumeSlider->setEnabled(!mute);
  mToolBar->insertAction(mSoundAnchor, mute ? soundUnmuteAction : soundMuteAction);

  // setObjectName (used by the stylesheet)
  QWidget *soundEnableWidget = mToolBar->widgetForAction(soundUnmuteAction);
  QWidget *soundDisableWidget = mToolBar->widgetForAction(soundMuteAction);
  if (soundEnableWidget)
    soundEnableWidget->setObjectName("menuButton");
  if (soundDisableWidget)
    soundDisableWidget->setObjectName("menuButton");

  mToolBar->setUpdatesEnabled(true);

  mToolBar->update();
}

QList<QByteArray> OmSimulationView::saveState() const {
  QList<QByteArray> state;
  state << mSplitter->saveState() << mSceneTree->saveState();
  return state;
}

void OmSimulationView::restoreState(QList<QByteArray> state, bool firstLoad) {
  assert(state.size() == 2);

  if (!state[0].isEmpty()) {
    mSplitter->restoreState(state[0]);
    mSplitter->setHandleWidth(mHandleWidth);
  } else if (firstLoad)
    restoreFactoryLayout();

  if (!state[1].isEmpty())
    mSceneTree->restoreState(state[1]);

  updateSceneTreeActions(isVisible() && mSplitter->sizes()[0] > 0);
  updateToggleView3DAction(isVisible() && mView3D->width() > 0);
}

void OmSimulationView::restoreFactoryLayout() {
  const int halfSplitterWidth = mSplitter->width() * 0.5;
  int preferredSceneTreeWidth = mSceneTree->sizeHint().width();
  if (preferredSceneTreeWidth > halfSplitterWidth)
    // default scene tree width should never be bigger than 3D view width
    preferredSceneTreeWidth = halfSplitterWidth;

  QList<int> sizes = QList<int>() << preferredSceneTreeWidth << (mSplitter->width() - preferredSceneTreeWidth);
  mSplitter->setSizes(sizes);
  mSplitter->setHandleWidth(mHandleWidth);
  mSceneTree->restoreFactoryLayout();
  updateSceneTreeActions(isVisible() && mSceneTree->width() > 0);
  updateToggleView3DAction(isVisible() && mView3D->width() > 0);
}

void OmSimulationView::setView3DResizeStretch(bool isSizeFixed) {
  const int view3DStretch = isSizeFixed ? 0 : 1;
  const int emptyCellStretch = isSizeFixed ? 1 : 0;
  mView3DResizeLayout->setColumnStretch(0, view3DStretch);
  mView3DResizeLayout->setColumnStretch(1, emptyCellStretch);
  mView3DResizeLayout->setRowStretch(0, view3DStretch);
  mView3DResizeLayout->setRowStretch(1, emptyCellStretch);
}

void OmSimulationView::enableView3DFixedSize(const QSize &size) {
  setView3DResizeStretch(true);
  mView3DContainer->setMinimumSize(size);
  mView3DContainer->setMaximumSize(size);
  // manually update the WREN render window size
  // on Jenkins machine the resize event is processed too late after the first
  // scene screenshot has already been taken
  mView3D->resizeWren(size.width(), size.height());
}

void OmSimulationView::disableView3DFixedSize() {
  setView3DResizeStretch(false);
  mView3DContainer->setMinimumSize(0, 0);
  mView3DContainer->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
}

void OmSimulationView::applyChanges() {
  mSceneTree->applyChanges();
}

void OmSimulationView::cleanup() {
  mView3D->cleanupEvents();
  mSceneTree->cleanup();
}

void OmSimulationView::internalScreenChangedCallback() {
  mView3D->updateScreenPixelRatio();
}

OmRobot *OmSimulationView::selectedRobot() const {
  OmBaseNode *selectedNode = mSelection->selectedNode();
  if (selectedNode) {
    OmRobot *robot = dynamic_cast<OmRobot *>(selectedNode);
    // cppcheck-suppress knownConditionTrueFalse
    if (!robot)
      robot = OmNodeUtilities::findRobotAncestor(selectedNode);
    // cppcheck-suppress knownConditionTrueFalse
    if (robot)
      return robot;
  }

  return NULL;
}

void OmSimulationView::keyPressEvent(QKeyEvent *event) {
  mView3D->handleModifierKey(event, true);
}

void OmSimulationView::keyReleaseEvent(QKeyEvent *event) {
  mView3D->handleModifierKey(event, false);
}

void OmSimulationView::hideEvent(QHideEvent *event) {
  mLastSize = size();
  mSplitterStatus = mToggleSceneTreeAction->isChecked() ? SCENE_TREE_VISIBLE : 0;
  mSplitterStatus = mSplitterStatus | (mToggleView3DAction->isChecked() ? VIEW_3D_VISIBLE : 0);
  mToggleSceneTreeAction->blockSignals(true);
  mToggleSceneTreeAction->setChecked(false);
  mToggleSceneTreeAction->blockSignals(false);
  updateToggleView3DAction(false);
}

void OmSimulationView::showEvent(QShowEvent *event) {
  if (mToggleSceneTreeAction->isChecked() || mToggleView3DAction->isChecked())
    return;

  // show after minimize dock event
  mToggleSceneTreeAction->blockSignals(true);
  mToggleSceneTreeAction->setChecked(mSplitterStatus & SCENE_TREE_VISIBLE);
  mToggleSceneTreeAction->blockSignals(false);
  updateToggleView3DAction(mSplitterStatus & VIEW_3D_VISIBLE);
}

void OmSimulationView::showMenu(const QPoint &position, QWidget *parentWidget) {
  const OmBaseNode *selectedNode = OmSelection::instance() ? OmSelection::instance()->selectedNode() : NULL;
  OmContextMenuGenerator::generateContextMenu(position, selectedNode, parentWidget);
}
