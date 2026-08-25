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

#ifndef OM_APPLICATION_HPP
#define OM_APPLICATION_HPP

//
// Description: OmniSim main application
// instanciated/deleted by OmGuiApplication (the only owner today; there is no
// console-application variant)
//

#include <QtCore/QObject>
#include <QtCore/QString>

class OmControlledWorld;
class QElapsedTimer;

class OmApplication : public QObject {
  Q_OBJECT

public:
  static OmApplication *instance() { return cInstance; }

  OmApplication();
  virtual ~OmApplication();
  void setup();

  const QString &worldName() const { return mWorldName; }

  // directory from which OmniSim executable was called (argv[0])
  const QString &startupPath() const { return mStartupPath; }

  // check if the loading of the world was canceled by the user
  bool wasWorldLoadingCanceled() const;

  // delete the progress dialog and eventually load empty world
  void cancelWorldLoading(bool loadEmpty, bool deleteWorld = false);
  bool isValidWorldFileName(const QString &worldName);

  // take a sceenshot of the 3d view
  // quality must be between 0 and 100 included
  void takeScreenshot(const QString &fileName, int quality);

  // quite the simulation and return 'exitStatus' to the calling shell
  void simulationQuit(int exitStatus);

  // reload the currently loaded world
  void worldReload();

  // reset the simulation
  void simulationReset(bool restartControllers);

  // start/stop video capture
  void startVideoCapture(const QString &fileName, int type, int width, int height, int quality, int acceleration,
                         bool showCaption);
  void stopVideoCapture();

  // start/stop animation capture
  void startAnimationCapture(const QString &fileName);
  void stopAnimationCapture();

  // reset physics on all solids in the world
  void resetPhysics();

signals:
  void preWorldLoaded(bool reloading);
  void postWorldLoaded(bool reloading, bool firstLoad);

  void worldLoadRequested(const QString &filename);

  void requestScreenshot(const QString &fileName, int quality);
  void simulationQuitRequested(int exitStatus);
  void worldReloadRequested();
  void simulationResetRequested(bool restartControllers);
  void videoCaptureStarted(const QString &fileName, int type, int width, int height, int quality, int acceleration,
                           bool showCaption);
  void videoCaptureStopped(bool canceled);
  void videoCreationStatusChanged(int status);

  void animationCaptureStarted(const QString &fileName);
  void animationCaptureStopped();
  void animationStartStatusChanged(int status);
  void animationStopStatusChanged(int status);

  void createWorldLoadingProgressDialog();
  void deleteWorldLoadingProgressDialog();
  void worldLoadingHasProgressed(const int progress);
  void worldLoadingStatusHasChanged(const QString &status);
  void worldLoadingWasCanceled();
  void worldLoadCompleted();

public slots:
  // load a world .wbt file
  // worldName must be absolute or specified with respect to WEBOTS_HOME
  // return true on success, false otherwise
  void loadWorld(QString worldName, bool reloading, bool isLoadingAfterDownload = false);

  void setWorldLoadingCanceled();
  void setWorldLoadingProgress(const int progress);
  void setWorldLoadingStatus(const QString &status);
  void setWorldLoadingProgressDialogCreatedtoFalse();

private:
  static OmApplication *cInstance;
  static QString cWebotsTmpDirName;

  OmControlledWorld *mWorld;
  QString mWorldName;
  QString mStartupPath;

  bool mWorldLoadingCanceled;
  bool mWorldLoadingProgressDialogCreated;
};

#endif
