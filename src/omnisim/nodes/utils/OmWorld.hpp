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

#ifndef OM_WORLD_HPP
#define OM_WORLD_HPP

//
// Description: Webots world
//

#include "OmWorldInfo.hpp"

#include <QtCore/QMutex>
#include <QtCore/QObject>
#include <QtCore/QString>

class OmGroup;
class OmNode;
class OmPerspective;
class OmRobot;
class OmSolid;
class OmTokenizer;
class OmViewpoint;


class OmWorld : public QObject {
  Q_OBJECT;

public:
  // unique world instance (can be NULL)
  static OmWorld *instance();

  // constructor
  // the world is read using 'tokenizer': the file syntax must have been checked with OmParser
  // if 'tokenizer' is not specified, the world is created with default WorldInfo and Viewpoint nodes
  explicit OmWorld(OmTokenizer *tokenizer = NULL);

  // destructor
  virtual ~OmWorld();

  void finalize();

  // current file name
  const QString &fileName() const { return mFileName; }
  bool needSaving() const;
  bool isModifiedFromSceneTree() const { return mIsModifiedFromSceneTree; }
  bool isModified() const { return mIsModified; }
  void setModified(bool isModified = true);
  void setModifiedFromSceneTree();

  // world loading functions
  bool isLoading() const { return mIsLoading; }
  void setIsLoading(bool loading) { mIsLoading = loading; }
  bool isCleaning() const { return mIsCleaning; }
  void setIsCleaning(bool cleaning) { mIsCleaning = cleaning; }
  bool wasWorldLoadingCanceled() { return mWorldLoadingCanceled; }

  // Headless texture-decode gate (see OmImageTexture::updateUrl): does this world
  // contain any node whose SIMULATION output depends on decoded texture pixels
  // (Camera, infra-red DistanceSensor, Pen)? Defaults to true (decode) and is
  // computed by a pre-finalize scan in OmSimulationWorld; conservative by design.
  bool needsTextures() const { return mNeedsTexturesForSensors; }
  void setNeedsTextures(bool needs) { mNeedsTexturesForSensors = needs; }

  bool isVideoRecording() const { return mIsVideoRecording; }

  static bool isW3dStreaming() { return cW3dStreaming; }
  static void enableW3dStreaming() { cW3dStreaming = true; }
  static bool printExternUrls() { return cPrintExternUrls; }
  static void setPrintExternUrls() { cPrintExternUrls = true; }

  // save
  bool save();
  virtual bool saveAs(const QString &fileName);

  // save and replace Webots specific nodes by VRML/W3D nodes
  bool exportAsHtml(const QString &fileName, bool animation) const;
  bool exportAsW3d(const QString &fileName) const;
  void write(OmWriter &writer) const;

  // nodes that do always exist
  OmGroup *root() const { return mRoot; }
  OmWorldInfo *worldInfo() const { return mWorldInfo; }
  OmViewpoint *viewpoint() const { return mViewpoint; }
  void setWorldInfo(OmWorldInfo *worldInfo) { mWorldInfo = worldInfo; }
  void setViewpoint(OmViewpoint *viewpoint);
  double orthographicViewHeight() const;
  void setOrthographicViewHeight(double ovh) const;

  // current perspective
  OmPerspective *perspective() const { return mPerspective; }
  bool reloadPerspective();

  // find a solid by its "name" field
  OmSolid *findSolid(const QString &name) const;

  // create a list of all solids (on the fly), look recursively
  // if 'visibleNodes' is true: return list of Solid nodes visible in the scene tree
  // if 'visibleNodes' is false: return instantiated Solid nodes (i.e. excluding proto parameter nodes)
  QList<OmSolid *> findSolids(bool visibleNodes = false) const;

  // return the list of all robots
  const QList<OmRobot *> &robots() const { return mRobots; }

  // return the list of all top solids (not looking recursively)
  const QList<OmSolid *> &topSolids() const { return mTopSolids; }

  // return the list of all solids that have a positive radar cross-section (radar target)
  const QList<OmSolid *> &radarTargetSolids() const { return mRadarTargets; }
  void addRadarTarget(OmSolid *target) { mRadarTargets.append(target); }
  void removeRadarTarget(OmSolid *target) { mRadarTargets.removeAll(target); }

  // return the list of all solids that have a non-empty 'recognitionColors' field
  const QList<OmSolid *> &cameraRecognitionObjects() const { return mCameraRecognitionObjects; }
  void addCameraRecognitionObject(OmSolid *object) { mCameraRecognitionObjects.append(object); }
  void removeCameraRecognitionObject(OmSolid *object) { mCameraRecognitionObjects.removeAll(object); }

  // functions to maintain global list of robots
  void removeRobotIfPresent(OmRobot *robot);
  void addRobotIfNotAlreadyPresent(OmRobot *robot);

  // return the list of texture files used in this world (no duplicates)
  QList<std::pair<QString, OmMFString *>> listTextureFiles() const;

  // shortcut
  double basicTimeStep() const { return mWorldInfo->basicTimeStep(); }
  int optimalThreadCount() const { return mWorldInfo->optimalThreadCount(); }


  void retrieveNodeNamesWithOptionalRendering(QStringList &centerOfMassNodeNames, QStringList &centerOfBuoyancyNodeNames,
                                              QStringList &supportPolygonNodeNames) const;

  void setResetRequested(bool restartControllers) {
    mResetRequested = true;
    if (!mRestartControllers)
      mRestartControllers = restartControllers;
  }
  virtual void reset(bool restartControllers) {
    mResetRequested = false;
    mRestartControllers = false;
  }

signals:
  void modificationChanged(bool modified);
  void worldLoadingStatusHasChanged(QString status);
  void worldLoadingHasProgressed(int percent);
  void viewpointChanged();
  void robotAdded(OmRobot *robot);
  void robotRemoved(OmRobot *robot);
  void resetRequested(bool restartControllers);

public slots:
  void awake();
  void updateVideoRecordingStatus(int status) {
    mIsVideoRecording = (status == WB_SUPERVISOR_MOVIE_RECORDING || status == WB_SUPERVISOR_MOVIE_SAVING);
  }

protected:
  bool mWorldLoadingCanceled;
  bool mResetRequested;
  bool mRestartControllers;

  QString logWorldMetrics() const;

  // called when a node is added to the children of a group which checks if a
  // controller needs starting, should the added node be a Robot
  virtual void setUpControllerForNewRobot(OmRobot *robot) {}

protected slots:
  virtual void storeAddedNodeIfNeeded(OmNode *node) {}

private:
  QString mFileName;
  bool mIsModified;
  bool mIsModifiedFromSceneTree;
  OmGroup *mRoot;
  OmWorldInfo *mWorldInfo;
  OmViewpoint *mViewpoint;
  // true only when the world file contained NO Viewpoint node and the engine inserted a default
  // one in checkPresenceOfMandatoryNodes(). Gates the load-time auto-framing: an authored
  // Viewpoint (cinematic worlds, render-oracle fixtures) must never be overridden.
  bool mViewpointAutoInserted;
  OmPerspective *mPerspective;
  QList<OmRobot *> mRobots;
  QList<OmSolid *> mTopSolids;
  QList<OmSolid *> mRadarTargets;
  QList<OmSolid *> mCameraRecognitionObjects;
  double mLastAwakeningTime;
  bool mIsLoading;
  bool mIsCleaning;
  bool mIsVideoRecording;
  bool mNeedsTexturesForSensors = true;  // headless texture-decode gate; see needsTextures()

  void checkPresenceOfMandatoryNodes();
  // Frames the auto-inserted Viewpoint on the scene. No-op unless mViewpointAutoInserted.
  void frameViewpointOnScene();
  OmNode *findTopLevelNode(const QString &modelName, int preferredPosition) const;

  virtual void storeLastSaveTime(){};

  static bool cW3dStreaming;
  static bool cPrintExternUrls;

private slots:
  void updateProjectPath(const QString &oldPath, const QString &newPath);
  void updateTopLevelLists();
};

#endif
