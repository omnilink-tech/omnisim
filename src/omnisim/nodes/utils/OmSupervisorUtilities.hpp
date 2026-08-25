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

#ifndef OM_SUPERVISOR_UTILITIES_HPP
#define OM_SUPERVISOR_UTILITIES_HPP

#include "OmSimulationState.hpp"

#include <QtCore/QObject>
#include <QtCore/QVector>

class QDataStream;

struct OmUpdatedFieldInfo;
struct OmFieldGetRequest;
struct OmTrackedFieldInfo;
struct OmTrackedPoseInfo;
struct OmTrackedContactPointInfo;
class OmFieldSetRequest;

class OmBaseNode;
class OmDataStream;
class OmNode;
class OmRobot;
class OmPose;
class OmSolid;
class OmWrenLabelOverlay;
class OmField;

class OmSupervisorUtilities : public QObject {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSupervisorUtilities(OmRobot *robot);
  virtual ~OmSupervisorUtilities();

  void handleMessage(QDataStream &stream);
  void writeAnswer(OmDataStream &stream);
  void writeConfigure(OmDataStream &stream);
  void processImmediateMessages(bool blockRegeneration = false);
  void postPhysicsStep();
  void reset();  // should be called when controllers are restarted

  bool shouldBeRemoved() const { return mShouldRemoveNode; }
  QStringList labelsState() const;

signals:
  void worldModified();
  void changeSimulationModeRequested(int newMode);
  void labelChanged(const QString &labelDescription);  // i.e. json format

private slots:
  void animationStartStatusChanged(int status);
  void animationStopStatusChanged(int status);
  void movieStatusChanged(int status);
  void changeSimulationMode(int newMode);
  void updateDeletedNodeList(OmNode *node);
  void notifyNodeUpdate(OmNode *node);
  void notifyFieldUpdate();
  void updateProtoRegeneratedFlag(OmNode *node);
  void removeTrackedContactPoints(QObject *obj);
  void removeTrackedPoseNode(QObject *obj);
  void removeTrackedField(QObject *obj);
  void simulationReset(bool restartControllers);

private:
  OmRobot *mRobot;
  int mFoundNodeUniqueId;
  int mFoundNodeType;
  int mFoundNodeTag;
  QString mFoundNodeModelName;
  QString mCurrentDefName;
  int mFoundNodeParentUniqueId;
  bool mFoundNodeIsProto;
  bool mFoundNodeIsProtoInternal;
  int mFoundProtoId;
  QString mFoundProtoTypeName;
  bool mFoundProtoIsDerived;
  int mFoundProtoParameterCount;
  int mFoundFieldIndex;
  int mFoundFieldType;
  int mFoundFieldCount;
  QString mFoundFieldName;
  bool mFoundFieldIsInternal;
  int mFoundFieldActualFieldNodeId;
  int mFoundFieldActualFieldIndex;
  int mNodeFieldCount;
  int mGetNodeRequest;
  QList<int> mUpdatedNodeIds;
  OmPose *mNodeGetPosition;
  OmPose *mNodeGetOrientation;
  std::pair<OmPose *, OmPose *> mNodeGetPose;
  OmSolid *mNodeGetCenterOfMass;
  OmSolid *mNodeGetContactPoints;
  int mNodeIdGetContactPoints;
  bool mGetContactPointsIncludeDescendants;
  OmSolid *mNodeGetStaticBalance;
  OmSolid *mNodeGetVelocity;
  // wb_supervisor_node_solve_ik pending request (internal parity plan, item W2.1).
  // Decoded in handleMessage, answered (always -- failures included, so the
  // libController side never blocks on a missing answer) in writeAnswer.
  bool mSolveIkRequested;
  OmSolid *mSolveIkSolid;
  QVector<double> mSolveIkTargets;    // 3 * n
  QVector<double> mSolveIkRotations;  // empty or 4 * n
  QVector<double> mSolveIkToolOffset; // empty or 3
  int mSolveIkIterations;
  QString mNodeExportString;
  bool mNodeExportStringRequest;
  bool mIsProtoRegenerated;
  bool mShouldRemoveNode;
  bool mSimulationReset;

  // pointer to a single integer: if not NULL, the new status has to be sent to the libController
  int *mAnimationStartStatus;
  int *mAnimationStopStatus;
  int *mMovieStatus;
  bool *mSaveStatus;

  int mImportedNodeId;
  bool mLoadWorldRequested;
  QString mWorldToLoad;

  bool mVirtualRealityHeadsetIsUsedRequested;

  QVector<int> mNodesDeletedSinceLastStep;
  QVector<OmUpdatedFieldInfo> mWatchedFields;  // fields used by the libController that need to be updated on change
  QVector<OmUpdatedFieldInfo> mUpdatedFields;  // changed fields that have to be notified to the libController
  QVector<OmFieldSetRequest *> mFieldSetRequests;
  struct OmFieldGetRequest *mFieldGetRequest;

  void pushSingleFieldContentToStream(OmDataStream &stream, OmField *field);
  void pushRelativePoseToStream(OmDataStream &stream, OmPose *fromNode, OmPose *toNode);
  void pushContactPointsToStream(OmDataStream &stream, OmSolid *solid, int solidId, bool includeDescendants);
  void pushSolveIkToStream(OmDataStream &stream);
  void initControllerRequests();
  void deleteControllerRequests();
  void writeNode(OmDataStream &stream, const OmBaseNode *baseNode, int messageType);
  const OmNode *getNodeFromDEF(const QString &defName, bool allowSearchInProto, const OmNode *fromNode = NULL);
  const OmNode *getNodeFromProtoDEF(const OmNode *fromNode, const QString &defName) const;
  OmNode *getProtoParameterNodeInstance(int nodeId, const QString &functionName) const;
  void applyFieldSetRequest(struct field_set_request *request);
  QString readString(QDataStream &);
  void makeFilenameAbsolute(QString &filename);
  OmSimulationState::Mode convertSimulationMode(int supervisorMode);
  QString createLabelUpdateString(const OmWrenLabelOverlay *labelOverlay) const;

  QList<int> mLabelIds;
  QVector<OmTrackedFieldInfo> mTrackedFields;
  QVector<OmTrackedPoseInfo> mTrackedPoses;
  QVector<OmTrackedContactPointInfo> mTrackedContactPoints;
};

#endif
