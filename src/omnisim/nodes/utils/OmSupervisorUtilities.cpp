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

#include "OmSupervisorUtilities.hpp"

#include "OmAbstractCamera.hpp"
#include "OmApplication.hpp"
#include "OmDataStream.hpp"
#include "OmDevice.hpp"
#include "OmDictionary.hpp"
#include "OmField.hpp"
#include "OmFieldModel.hpp"
#include "OmJoint.hpp"
#include "OmJointParameters.hpp"
#include "OmMFBool.hpp"
#include "OmMFColor.hpp"
#include "OmMFDouble.hpp"
#include "OmMFInt.hpp"
#include "OmMFNode.hpp"
#include "OmMFRotation.hpp"
#include "OmMFString.hpp"
#include "OmMFVector2.hpp"
#include "OmMFVector3.hpp"
#include "OmMotor.hpp"
#include "OmBasicJoint.hpp"
#include "OmNewtonBackend.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeProtoInfo.hpp"
#include "OmNodeUtilities.hpp"
#include "OmProject.hpp"
#include "OmRgb.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmRobot.hpp"
#include "OmSFColor.hpp"
#include "OmSFNode.hpp"
#include "OmSFVector2.hpp"
#include "OmSelection.hpp"
#include "OmStandardPaths.hpp"
#include "OmSolidMerger.hpp"
#include "OmTemplateManager.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"
#include "OmWrenLabelOverlay.hpp"


#include "../../../include/controller/c/omnisim/supervisor.h"
#include "../../controller/c/messages.h"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <cassert>

static const int MAX_LABELS = 100;

struct OmTrackedFieldInfo {
  OmField *field;
  int fieldId;
  int nodeId;
  bool internal;
  int samplingPeriod;
  double lastUpdate;
};

struct OmTrackedPoseInfo {
  OmPose *fromNode;
  OmPose *toNode;
  int samplingPeriod;
  double lastUpdate;
};

struct OmTrackedContactPointInfo {
  OmSolid *solid;
  int solidId;
  bool includeDescendants;
  int samplingPeriod;
  double lastUpdate;
};

struct OmFieldGetRequest {
  OmField *field;
  int fieldId;
  int nodeId;
  int protoId;
  int index;  // for MF fields only
};

struct OmUpdatedFieldInfo {
  int nodeId;
  QString fieldName;
  int fieldCount;
  OmUpdatedFieldInfo(int nodeId, const QString &fieldName, int fieldCount) :
    nodeId(nodeId),
    fieldName(fieldName),
    fieldCount(fieldCount) {}
};

class OmFieldSetRequest {
public:
  virtual void apply() const = 0;
  virtual ~OmFieldSetRequest() {}

protected:
  OmFieldSetRequest(OmField *field, int index) : mField(field), mIndex(index) {}
  OmField *mField;
  int mIndex;
};

class OmBoolFieldSetRequest : public OmFieldSetRequest {
public:
  OmBoolFieldSetRequest(OmField *f, int index, bool value) : OmFieldSetRequest(f, index), mValue(value) {
    assert((f->type() == WB_SF_BOOL && dynamic_cast<OmSFBool *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_BOOL && dynamic_cast<OmMFBool *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFBool *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<OmMFBool *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  bool mValue;
};

class OmIntFieldSetRequest : public OmFieldSetRequest {
public:
  OmIntFieldSetRequest(OmField *f, int index, int value) : OmFieldSetRequest(f, index), mValue(value) {
    assert((f->type() == WB_SF_INT32 && dynamic_cast<OmSFInt *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_INT32 && dynamic_cast<OmMFInt *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFInt *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<OmMFInt *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  int mValue;
};

class OmDoubleFieldSetRequest : public OmFieldSetRequest {
public:
  OmDoubleFieldSetRequest(OmField *f, int index, double value) : OmFieldSetRequest(f, index), mValue(value) {
    assert((f->type() == WB_SF_FLOAT && dynamic_cast<OmSFDouble *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_FLOAT && dynamic_cast<OmMFDouble *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFDouble *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<OmMFDouble *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  double mValue;
};

class OmVector2FieldSetRequest : public OmFieldSetRequest {
public:
  OmVector2FieldSetRequest(OmField *f, int index, double x, double y) : OmFieldSetRequest(f, index), mValue(x, y) {
    mValue.clamp();
    assert((f->type() == WB_SF_VEC2F && dynamic_cast<OmSFVector2 *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_VEC2F && dynamic_cast<OmMFVector2 *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFVector2 *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<OmMFVector2 *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  OmVector2 mValue;
};

class OmVector3FieldSetRequest : public OmFieldSetRequest {
public:
  OmVector3FieldSetRequest(OmField *f, int index, double x, double y, double z) : OmFieldSetRequest(f, index), mValue(x, y, z) {
    mValue.clamp();
    assert((f->type() == WB_SF_VEC3F && dynamic_cast<OmSFVector3 *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_VEC3F && dynamic_cast<OmMFVector3 *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFVector3 *>(mField->value()))->setValueByUser(mValue, true);
    else
      (dynamic_cast<OmMFVector3 *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  OmVector3 mValue;
};

class OmColorFieldSetRequest : public OmFieldSetRequest {
public:
  OmColorFieldSetRequest(OmField *f, int index, double red, double green, double blue) :
    OmFieldSetRequest(f, index),
    mValue(red, green, blue) {
    assert((f->type() == WB_SF_COLOR && dynamic_cast<OmSFColor *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_COLOR && dynamic_cast<OmMFColor *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFColor *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<OmMFColor *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  OmRgb mValue;
};

class OmRotationFieldSetRequest : public OmFieldSetRequest {
public:
  OmRotationFieldSetRequest(OmField *f, int index, double x, double y, double z, double a) :
    OmFieldSetRequest(f, index),
    mValue(x, y, z, a) {
    assert((f->type() == WB_SF_ROTATION && dynamic_cast<OmSFRotation *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_ROTATION && dynamic_cast<OmMFRotation *>(f->value()) && index >= 0));
    mValue.normalize();
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFRotation *>(mField->value()))->setValueByUser(mValue, true);
    else
      (dynamic_cast<OmMFRotation *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  OmRotation mValue;
};

class OmStringFieldSetRequest : public OmFieldSetRequest {
public:
  OmStringFieldSetRequest(OmField *f, int index, const QString &string) : OmFieldSetRequest(f, index), mValue(string) {
    assert((f->type() == WB_SF_STRING && dynamic_cast<OmSFString *>(f->value()) && index == -1) ||
           (f->type() == WB_MF_STRING && dynamic_cast<OmMFString *>(f->value()) && index >= 0));
  }
  void apply() const override {
    if (mIndex == -1)
      (dynamic_cast<OmSFString *>(mField->value()))->setValue(mValue);
    else
      (dynamic_cast<OmMFString *>(mField->value()))->setItem(mIndex, mValue);
  }

private:
  QString mValue;
};

OmSupervisorUtilities::OmSupervisorUtilities(OmRobot *robot) : mRobot(robot) {
  initControllerRequests();

  connect(OmApplication::instance(), &OmApplication::animationStartStatusChanged, this,
          &OmSupervisorUtilities::animationStartStatusChanged);
  connect(OmApplication::instance(), &OmApplication::animationStopStatusChanged, this,
          &OmSupervisorUtilities::animationStopStatusChanged);
  connect(OmApplication::instance(), &OmApplication::videoCreationStatusChanged, this,
          &OmSupervisorUtilities::movieStatusChanged);
  connect(OmNodeOperations::instance(), &OmNodeOperations::nodeDeleted, this, &OmSupervisorUtilities::updateDeletedNodeList);
  connect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this,
          &OmSupervisorUtilities::updateProtoRegeneratedFlag);

  //  Do not apply the change simulation mode during dealing with a controller message
  //  otherwise, conflicts can occur in case of multiple controllers
  connect(this, &OmSupervisorUtilities::changeSimulationModeRequested, this, &OmSupervisorUtilities::changeSimulationMode,
          Qt::QueuedConnection);
  connect(OmWorld::instance(), &OmWorld::resetRequested, this, &OmSupervisorUtilities::simulationReset, Qt::QueuedConnection);
}

OmSupervisorUtilities::~OmSupervisorUtilities() {
  foreach (int labelId, mLabelIds)
    OmWrenLabelOverlay::removeLabel(labelId);

  deleteControllerRequests();
}

void OmSupervisorUtilities::deleteControllerRequests() {
  foreach (OmFieldSetRequest *request, mFieldSetRequests)
    delete request;
  mFieldSetRequests.clear();
  delete mFieldGetRequest;
  delete mAnimationStartStatus;
  delete mAnimationStopStatus;
  delete mMovieStatus;
  delete mSaveStatus;
}

void OmSupervisorUtilities::initControllerRequests() {
  mFoundNodeUniqueId = -1;
  mFoundNodeType = 0;
  mFoundNodeParentUniqueId = -1;
  mFoundNodeIsProto = false;
  mFoundNodeIsProtoInternal = false;
  mNodeFieldCount = -1;
  mFoundProtoId = -2;
  mFoundProtoTypeName.clear();
  mFoundProtoIsDerived = false;
  mFoundProtoParameterCount = -1;
  mFoundFieldIndex = -2;
  mFoundFieldType = 0;
  mFoundFieldCount = -1;
  mFoundFieldIsInternal = false;
  mFoundFieldName.clear();
  mFoundFieldActualFieldNodeId = -1;
  mFoundFieldActualFieldIndex = -1;
  mGetNodeRequest = 0;
  mNodeGetPosition = NULL;
  mNodeGetOrientation = NULL;
  mNodeGetCenterOfMass = NULL;
  mNodeGetContactPoints = NULL;
  mGetContactPointsIncludeDescendants = false;
  mNodeGetStaticBalance = NULL;
  mNodeGetVelocity = NULL;
  mSolveIkRequested = false;
  mSolveIkSolid = NULL;
  mSolveIkTargets.clear();
  mSolveIkRotations.clear();
  mSolveIkToolOffset.clear();
  mSolveIkIterations = 64;
  mNodeExportStringRequest = false;
  mIsProtoRegenerated = false;
  mShouldRemoveNode = false;
  mImportedNodeId = -1;
  mLoadWorldRequested = false;
  mVirtualRealityHeadsetIsUsedRequested = false;
  mFieldGetRequest = NULL;
  mAnimationStartStatus = NULL;
  mAnimationStopStatus = NULL;
  mMovieStatus = NULL;
  mSaveStatus = NULL;
  mWorldToLoad.clear();
  mNodesDeletedSinceLastStep.clear();
  mUpdatedFields.clear();
  mWatchedFields.clear();
  mSimulationReset = false;
}

QString OmSupervisorUtilities::readString(QDataStream &stream) {
  QByteArray txt;
  unsigned char uc;
  do {
    stream >> uc;
    txt.append(uc);
  } while (uc != 0 && !stream.atEnd());
  return QString(txt.constData());
}

// if filename is relative: make it absolute with respect to Supervisor's controller dir
void OmSupervisorUtilities::makeFilenameAbsolute(QString &filename) {
  if (QDir::isRelativePath(filename)) {
    QDir dir(mRobot->controllerDir());
    filename = dir.absoluteFilePath(filename);
  }
}

OmSimulationState::Mode OmSupervisorUtilities::convertSimulationMode(int supervisorMode) {
  switch (supervisorMode) {
    case WB_SUPERVISOR_SIMULATION_MODE_REAL_TIME:
      return OmSimulationState::REALTIME;
    case WB_SUPERVISOR_SIMULATION_MODE_FAST:
      return OmSimulationState::FAST;
    default:
      return OmSimulationState::PAUSE;
  }
}

void OmSupervisorUtilities::processImmediateMessages(bool blockRegeneration) {
  const int n = mFieldSetRequests.size();
  if (n == 0)
    return;
  OmTemplateManager::instance()->blockRegeneration(true);
  for (int i = 0; i < n; ++i) {
    const OmFieldSetRequest *r = mFieldSetRequests.at(i);
    r->apply();
    delete r;
  }
  mFieldSetRequests.clear();
  if (blockRegeneration)
    return;

  OmTemplateManager::instance()->blockRegeneration(false);
  // only emit if robot still exists (FieldSetRequest could have regenerated it)
  if (OmWorld::instance()->robots().contains(mRobot))
    emit worldModified();
}

void OmSupervisorUtilities::postPhysicsStep() {
  if (mLoadWorldRequested) {
    emit OmApplication::instance()->worldLoadRequested(mWorldToLoad);
    mLoadWorldRequested = false;
  }
  if (mShouldRemoveNode) {
    emit worldModified();
    OmNodeOperations::instance()->deleteNode(mRobot, true);
  }
}

void OmSupervisorUtilities::reset() {
  foreach (int labelId, mLabelIds)
    OmWrenLabelOverlay::removeLabel(labelId);
  mLabelIds.clear();
  mTrackedFields.clear();
  mTrackedPoses.clear();
  mTrackedContactPoints.clear();

  // delete pending requests and reinitialize them
  deleteControllerRequests();
  initControllerRequests();
}

const OmNode *OmSupervisorUtilities::getNodeFromProtoDEF(const OmNode *fromNode, const QString &defName) const {
  // recursively search in PROTO body for the DEF node
  QList<OmNode *> descendants = fromNode->subNodes(false, true, false);  // get nodes from PROTO fields
  for (int i = 0; i < descendants.size(); ++i) {
    const OmNode *child = descendants.at(i);
    if (child->defName() == defName)
      return child;
    // recursively search in field or parameters (if PROTO) of descendant nodes
    descendants.append(child->subNodes(true, false, false));
  }
  return NULL;
}

const OmNode *OmSupervisorUtilities::getNodeFromDEF(const QString &defName, bool allowSearchInProto, const OmNode *fromNode) {
  assert(!defName.isEmpty());
  if (defName.isEmpty())
    return NULL;

  const QStringList list(defName.split("."));

  const QString &currentDefName = list.at(0);
  const int remainingChars = defName.size() - (currentDefName.size() + 1);
  const QString &nextDefName = (remainingChars <= 0) ? QString() : defName.right(remainingChars);

  const OmNode *baseNode = fromNode;
  if (baseNode == NULL || allowSearchInProto) {
    if (allowSearchInProto)
      baseNode = getNodeFromProtoDEF(baseNode ? baseNode : OmWorld::instance()->root(), defName);
    else
      baseNode = OmDictionary::instance()->getNodeFromDEF(currentDefName);

    if (!baseNode || nextDefName.isEmpty())
      return baseNode;
    return getNodeFromDEF(nextDefName, false, baseNode);
  }

  const QList<OmNode *> &descendants = baseNode->subNodes(false, allowSearchInProto, false);
  for (int i = 0; i < descendants.size(); ++i) {
    const OmNode *child = descendants.at(i);
    if (child->defName() == currentDefName) {
      if (nextDefName.isEmpty())
        return child;
      return getNodeFromDEF(nextDefName, false, child);
    }
  }

  return NULL;
}

void OmSupervisorUtilities::notifyNodeUpdate(OmNode *node) {
  if (!mRobot->isConfigureDone())
    return;
  // send updated node info to the libController
  // this is mainly used to update the cached DEF names
  mUpdatedNodeIds.append(node->uniqueId());
}

void OmSupervisorUtilities::notifyFieldUpdate() {
  if (!mRobot->isConfigureDone())
    return;
  // send updated field info to the libController
  const OmField *field = static_cast<OmField *>(sender());
  if (!field->parentNode())
    return;
  const int listSize = mUpdatedFields.size();
  const OmMultipleValue *mv = dynamic_cast<OmMultipleValue *>(field->value());
  int fieldCount;
  if (mv)
    fieldCount = mv->size();
  else {
    const OmSFNode *sfNode = dynamic_cast<OmSFNode *>(field->value());
    assert(sfNode);  // field should be a OmMultipleValue or a OmSFNode
    fieldCount = sfNode->value() ? 1 : 0;
  }
  const OmUpdatedFieldInfo info(field->parentNode()->uniqueId(), field->name(), fieldCount);
  for (int i = 0; i < listSize; ++i) {
    OmUpdatedFieldInfo &existingInfo = mUpdatedFields[i];
    if (existingInfo.nodeId == info.nodeId && existingInfo.fieldName == info.fieldName) {
      existingInfo.fieldCount = info.fieldCount;
      return;
    }
  }
  mUpdatedFields.append(info);
}

OmNode *OmSupervisorUtilities::getProtoParameterNodeInstance(int nodeId, const QString &functionName) const {
  OmNode *node = OmNode::findNode(nodeId);
  if (!node) {
    mRobot->warn(tr("%1: node not found.").arg(functionName));
    return NULL;
  }
  OmBaseNode *proto = static_cast<OmBaseNode *>(node)->getFirstFinalizedProtoInstance();
  if (!proto) {
    if (node->modelName() != node->nodeModelName())
      mRobot->warn(
        tr("Cannot get the PROTO instance for node '%1' (derived from '%2').").arg(node->usefulName(), node->nodeModelName()));
    else
      mRobot->warn(tr("Cannot get the PROTO instance for node '%1'.").arg(node->usefulName()));
  }
  return proto;
}

void OmSupervisorUtilities::changeSimulationMode(int newMode) {
  OmSimulationState::Mode mode = convertSimulationMode(newMode);
  OmSimulationState::instance()->setMode(mode);
}

void OmSupervisorUtilities::updateProtoRegeneratedFlag(OmNode *node) {
  mIsProtoRegenerated = true;

  if (mWatchedFields.isEmpty())
    return;
  const int nodeId = node->uniqueId();
  foreach (const OmUpdatedFieldInfo &info, mWatchedFields) {
    if (info.nodeId == nodeId) {
      const OmField *field = node->findField(info.fieldName, false);
      assert(field->isMultiple() || field->type() == WB_SF_NODE);
      field->listenToValueSizeChanges();
      connect(field, &OmField::valueSizeChanged, this, &OmSupervisorUtilities::notifyFieldUpdate, Qt::UniqueConnection);
    }
  }
}

void OmSupervisorUtilities::updateDeletedNodeList(OmNode *node) {
  if (!node)
    return;

  // check if node already in the list
  if (mNodesDeletedSinceLastStep.contains(node->uniqueId()))
    return;

  mNodesDeletedSinceLastStep.push_back(node->uniqueId());
  QList<OmNode *> children = node->subNodes(false, true, true);
  const int childrenSize = children.size();
  for (int i = 0; i < childrenSize; ++i)
    updateDeletedNodeList(children[i]);

  // update mWatchedFields
  QMutableVectorIterator<OmUpdatedFieldInfo> it(mWatchedFields);
  while (it.hasNext()) {
    OmUpdatedFieldInfo info = it.next();
    if (info.nodeId == node->uniqueId())
      it.remove();
  }
}

void OmSupervisorUtilities::removeTrackedContactPoints(QObject *obj) {
  for (int i = 0; i < mTrackedContactPoints.size(); ++i) {
    if (mTrackedContactPoints[i].solid == obj) {
      mTrackedContactPoints.removeAt(i);
      break;
    }
  }
}

void OmSupervisorUtilities::removeTrackedPoseNode(QObject *obj) {
  for (int i = mTrackedPoses.size() - 1; i >= 0; --i) {
    if (mTrackedPoses[i].fromNode == obj || mTrackedPoses[i].toNode == obj)
      mTrackedPoses.removeAt(i);
  }
}

void OmSupervisorUtilities::removeTrackedField(QObject *obj) {
  for (int i = 0; i < mTrackedFields.size(); ++i) {
    if (mTrackedFields[i].field == obj) {
      mTrackedFields.removeAt(i);
      break;
    }
  }
}

void OmSupervisorUtilities::handleMessage(QDataStream &stream) {
  unsigned char byte;
  stream >> byte;

  switch (byte) {
    case C_SUPERVISOR_SIMULATION_QUIT: {
      int exitStatus;
      stream >> exitStatus;
      OmApplication::instance()->simulationQuit(exitStatus);
      return;
    }
    case C_SUPERVISOR_SIMULATION_RESET:
      OmWorld::instance()->setResetRequested(false);
      return;
    case C_SUPERVISOR_NODE_RESET_STATE: {
      unsigned int nodeId;
      stream >> nodeId;
      const QString &stateName = readString(stream);
      OmNode *const node = getProtoParameterNodeInstance(nodeId, "wb_supervisor_node_load_state()");
      if (node) {
        // wb_supervisor_node_load_state() has no controller-restart concept at
        // all, so a motor command overwritten here would never be re-issued.
        // Measured 2026-08-12: this call ALONE freezes a driving rover for the
        // rest of the session. See OmMotor::resetMayOverwriteMotorCommand.
        const OmMotor::ResetPolicy motorPolicy(false);
        node->reset(stateName);
      }
      return;
    }
    case C_SUPERVISOR_NODE_SAVE_STATE: {
      unsigned int nodeId;
      stream >> nodeId;
      const QString &stateName = readString(stream);
      OmNode *const node = getProtoParameterNodeInstance(nodeId, "wb_supervisor_node_save_state()");
      if (node)
        node->save(stateName);
      return;
    }
    case C_SUPERVISOR_NODE_SET_JOINT_POSITION: {
      unsigned int nodeId, index;
      double position;
      stream >> nodeId >> position >> index;
      OmNode *const node = getProtoParameterNodeInstance(nodeId, "wb_supervisor_node_set_joint_position()");
      OmJoint *joint = dynamic_cast<OmJoint *>(node);
      assert(joint);
      if (joint) {
        // check if position is valid
        const OmJointParameters *parameters;
        if (index == 1)
          parameters = joint->parameters();
        else if (index == 2)
          parameters = joint->parameters2();
        else if (index == 3)
          parameters = joint->parameters3();
        else {
          assert(false);
          parameters = NULL;
        }
        if (parameters) {
          const double userPosition = position;
          if (parameters->clampPosition(position))
            mRobot->warn(tr("wb_supervisor_node_set_joint_position() called with a 'position' argument %1 outside hard limits "
                            "of the joint. Applied position is %2.")
                           .arg(userPosition)
                           .arg(position));
        }

        joint->setPosition(position, index);
        if (!parameters)
          // force updating the joint position (this slot is automatically triggered by OmJointParameters node)
          joint->updatePosition();
      }
      return;
    }
    case C_SUPERVISOR_RELOAD_WORLD:
      OmApplication::instance()->worldReload();
      return;
    case C_SUPERVISOR_SIMULATION_RESET_PHYSICS:
      OmApplication::instance()->resetPhysics();
      return;
    case C_SUPERVISOR_SIMULATION_CHANGE_MODE: {
      int newMode;
      stream >> newMode;
      emit changeSimulationModeRequested(newMode);
      return;
    }
    case C_SUPERVISOR_SET_LABEL: {
      unsigned short id;
      double x, y, size;
      unsigned int color;

      stream >> id;
      stream >> x;
      stream >> y;
      stream >> size;
      stream >> color;
      const QString &text = readString(stream);
      const QString &font = readString(stream);

      bool fileFound = false;
      // resolves both the fonts shipped in resources/fonts/ and the legacy (proprietary) font names
      // they replaced, e.g. "Arial" -> LiberationSans-Regular.ttf
      QString filename = OmStandardPaths::shippedFontFile(font);
      if (!filename.isEmpty())
        fileFound = true;
      else {
        filename = OmProject::current()->path() + "fonts/" + font + ".ttf";
        if (QFile::exists(filename))
          fileFound = true;
      }

      if (!fileFound) {
        mRobot->warn(
          tr("wb_supervisor_set_label() called with an invalid '%1' font, 'Liberation Sans' used instead.").arg(font));
        filename = OmStandardPaths::fontsPath() + "LiberationSans-Regular.ttf";
      }

      int labelId;
      if (id < MAX_LABELS)
        labelId = (int)id + mRobot->uniqueId() * MAX_LABELS;  // kind of hack to avoid an id clash.
      else {
        mRobot->warn(tr("wb_supervisor_set_label() is out of range. The supported range is [0, %1].").arg(MAX_LABELS - 1));
        return;
      }

      const OmWrenLabelOverlay *existingLabel = OmWrenLabelOverlay::retrieveById(labelId);
      if (existingLabel && x == existingLabel->x() && y == existingLabel->y() && size == existingLabel->size() &&
          filename == existingLabel->font() && text == existingLabel->text()) {
        const float *oldColors = existingLabel->color();
        float colorArray[4];
        OmWrenLabelOverlay::colorToArray(colorArray, color);
        if (colorArray[0] == oldColors[0] && colorArray[1] == oldColors[1] && colorArray[2] == oldColors[2] &&
            colorArray[3] == oldColors[3])
          return;
      }

      mLabelIds.removeAll(labelId);
      mLabelIds << labelId;

      OmWrenLabelOverlay *label = OmWrenLabelOverlay::createOrRetrieve(labelId, filename);
      const QString error = label->getFontError();
      if (error != "") {
        mRobot->warn(tr(error.toStdString().c_str()));
        return;
      }
      label->setText(text);
      label->setPosition(x, y);
      label->setSize(size);
      label->setColor(color);
      label->applyChangesToWren();
      emit labelChanged(createLabelUpdateString(label));

      return;
    }
    case C_SUPERVISOR_EXPORT_IMAGE: {
      unsigned char quality;
      stream >> quality;
      QString filename = readString(stream);
      makeFilenameAbsolute(filename);
      OmApplication::instance()->takeScreenshot(filename, quality);
      return;
    }
    case C_SUPERVISOR_START_MOVIE: {
      int width, height;
      unsigned char codec, quality, acceleration, caption;
      stream >> width;
      stream >> height;
      stream >> codec;
      stream >> quality;
      stream >> acceleration;
      stream >> caption;
      QString filename = readString(stream);
      makeFilenameAbsolute(filename);
      // cppcheck-suppress knownConditionTrueFalse
      OmApplication::instance()->startVideoCapture(filename, codec, width, height, quality, acceleration, caption == 1);
      return;
    }
    case C_SUPERVISOR_STOP_MOVIE:
      OmApplication::instance()->stopVideoCapture();
      return;
    case C_SUPERVISOR_START_ANIMATION: {
      QString filename = readString(stream);
      makeFilenameAbsolute(filename);
      OmApplication::instance()->startAnimationCapture(filename);
      return;
    }
    case C_SUPERVISOR_STOP_ANIMATION:
      OmApplication::instance()->stopAnimationCapture();
      return;
    case C_SUPERVISOR_NODE_GET_FROM_ID: {
      int id;
      stream >> id;
      const OmBaseNode *node = dynamic_cast<const OmBaseNode *>(OmNode::findNode(id));
      if (node) {
        // since 8.6 -> each message has its own mechanism
        mGetNodeRequest = C_SUPERVISOR_NODE_GET_FROM_ID;
        mCurrentDefName = node->defName();
        mFoundNodeUniqueId = node->uniqueId();
        mFoundNodeType = node->nodeType();
        const OmDevice *device = dynamic_cast<const OmDevice *>(node);
        mFoundNodeTag = (device && mRobot->findDevice(device->tag()) == device) ? device->tag() : -1;
        mFoundNodeModelName = node->modelName();
        mFoundNodeParentUniqueId = (node->parentNode() ? node->parentNode()->uniqueId() : -1);
        mFoundNodeIsProto = node->isProtoInstance();
        mFoundNodeIsProtoInternal =
          node->parentNode() != OmWorld::instance()->root() && !OmVrmlNodeUtilities::isVisible(node->parentField());
        connect(node, &OmNode::defUseNameChanged, this, &OmSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      }

      return;
    }
    case C_SUPERVISOR_NODE_GET_FROM_DEF: {
      const QString &nodeName = readString(stream);
      int parentProtoId;
      stream >> parentProtoId;  // if > 0, then search for a PROTO internal node
      const OmNode *proto = parentProtoId > 0 ? OmNode::findNode(parentProtoId) : NULL;
      const OmBaseNode *baseNode = dynamic_cast<const OmBaseNode *>(getNodeFromDEF(nodeName, proto != NULL, proto));
      if (!proto && baseNode && !baseNode->parentField())  // make sure the parent field is visible
        baseNode = NULL;
      mFoundNodeUniqueId = baseNode ? baseNode->uniqueId() : 0;
      mFoundNodeType = baseNode ? baseNode->nodeType() : 0;
      const OmDevice *device = dynamic_cast<const OmDevice *>(baseNode);
      mFoundNodeTag = (device && mRobot->findDevice(device->tag()) == device) ? device->tag() : -1;
      mFoundNodeModelName = baseNode ? baseNode->modelName() : QString();
      mFoundNodeIsProtoInternal = false;
      if (baseNode) {
        if (baseNode->parentNode()) {
          if (baseNode->parentNode() != OmWorld::instance()->root())
            mFoundNodeParentUniqueId = baseNode->parentNode()->uniqueId();
          else
            mFoundNodeParentUniqueId = 0;
        }
        mFoundNodeIsProto = baseNode->isProtoInstance();
        connect(baseNode, &OmNode::defUseNameChanged, this, &OmSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      } else {
        mFoundNodeParentUniqueId = -1;
        mFoundNodeIsProto = false;
      }
      return;
    }
    case C_SUPERVISOR_NODE_GET_FROM_TAG: {
      int tag;
      stream >> tag;

      mFoundNodeUniqueId = -1;
      const OmDevice *device = mRobot->findDevice(tag);
      if (!device)
        return;
      const OmBaseNode *baseNode = dynamic_cast<const OmBaseNode *>(device);
      assert(baseNode);
      mFoundNodeIsProtoInternal =
        baseNode->parentNode() != OmWorld::instance()->root() && !OmVrmlNodeUtilities::isVisible(baseNode->parentField());
      mGetNodeRequest = C_SUPERVISOR_NODE_GET_FROM_TAG;
      mCurrentDefName = baseNode->defName();
      mFoundNodeUniqueId = baseNode->uniqueId();
      mFoundNodeType = baseNode->nodeType();
      mFoundNodeTag = tag;
      mFoundNodeModelName = baseNode->modelName();
      if (baseNode->parentNode()) {
        if (baseNode->parentNode() != OmWorld::instance()->root())
          mFoundNodeParentUniqueId = baseNode->parentNode()->uniqueId();
        else
          mFoundNodeParentUniqueId = 0;
      }
      mFoundNodeIsProto = baseNode->isProtoInstance();
      connect(baseNode, &OmNode::defUseNameChanged, this, &OmSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      return;
    }
    case C_SUPERVISOR_NODE_GET_SELECTED: {
      const OmBaseNode *baseNode = dynamic_cast<const OmBaseNode *>(OmSelection::instance()->selectedNode());
      if (baseNode) {
        mGetNodeRequest = C_SUPERVISOR_NODE_GET_SELECTED;
        mCurrentDefName = baseNode->defName();
        mFoundNodeUniqueId = baseNode->uniqueId();
        mFoundNodeType = baseNode->nodeType();
        const OmDevice *device = dynamic_cast<const OmDevice *>(baseNode);
        mFoundNodeTag = (device && mRobot->findDevice(device->tag()) == device) ? device->tag() : -1;
        mFoundNodeModelName = baseNode->modelName();
        mFoundNodeParentUniqueId = -1;
        mFoundNodeIsProtoInternal = false;
        if (baseNode->parentNode()) {
          if (baseNode->parentNode() != OmWorld::instance()->root())
            mFoundNodeParentUniqueId = baseNode->parentNode()->uniqueId();
          else
            mFoundNodeParentUniqueId = 0;
        }
        connect(baseNode, &OmNode::defUseNameChanged, this, &OmSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
      }
      return;
    }
    case C_SUPERVISOR_NODE_GET_POSITION: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_get_position()");
      OmPose *const pose = dynamic_cast<OmPose *>(node);
      mNodeGetPosition = pose;
      if (!pose)
        mRobot->warn(tr("wb_supervisor_node_get_position() can exclusively be used with Pose (or derived)."));
      return;
    }
    case C_SUPERVISOR_NODE_GET_ORIENTATION: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_get_orientation()");
      OmPose *const pose = dynamic_cast<OmPose *>(node);
      mNodeGetOrientation = pose;
      if (!pose)
        mRobot->warn(tr("wb_supervisor_node_get_orientation() can exclusively be used with Pose (or derived)."));
      return;
    }
    case C_SUPERVISOR_NODE_GET_POSE: {
      unsigned int idFrom;
      unsigned int idTo;

      stream >> idFrom;
      stream >> idTo;

      if (idFrom) {
        OmNode *const fromNode = getProtoParameterNodeInstance(idFrom, "wb_supervisor_node_get_pose()");
        OmPose *const poseFrom = dynamic_cast<OmPose *>(fromNode);
        mNodeGetPose.first = poseFrom;
      } else
        mNodeGetPose.first = NULL;
      OmNode *const toNode = getProtoParameterNodeInstance(idTo, "wb_supervisor_node_get_pose()");
      OmPose *const poseTo = dynamic_cast<OmPose *>(toNode);
      mNodeGetPose.second = poseTo;

      if (!poseTo)
        mRobot->warn(tr("wb_supervisor_node_get_pose() can exclusively be used with Pose (or derived)."));
      return;
    }
    case C_SUPERVISOR_NODE_GET_CENTER_OF_MASS: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_get_center_of_mass()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      mNodeGetCenterOfMass = solid;
      if (!solid)
        mRobot->warn(tr("wb_supervisor_node_get_center_of_mass() can exclusively be used with Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_GET_CONTACT_POINTS: {
      unsigned int id;
      unsigned char includeDescendantsChar;

      stream >> id;
      stream >> includeDescendantsChar;

      const bool includeDescendants = includeDescendantsChar == 1;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_get_number_of_contact_points()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      mNodeGetContactPoints = solid;
      mNodeIdGetContactPoints = id;
      mGetContactPointsIncludeDescendants = includeDescendants;
      if (!solid)
        mRobot->warn(
          tr("wb_supervisor_node_get_number_of_contact_points() and wb_supervisor_node_get_contact_point() can exclusively "
             "be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_GET_STATIC_BALANCE: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_get_static_balance()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      mNodeGetStaticBalance = solid;
      if (!solid || !solid->isTopLevel())
        mRobot->warn(tr("wb_supervisor_node_get_static_balance() can exclusively be used with a top Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_GET_VELOCITY: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_get_velocity()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      if (solid)
        mNodeGetVelocity = solid;
      else
        mRobot->warn(tr("wb_supervisor_node_get_velocity() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_SOLVE_IK: {
      // ⚠ The WHOLE request must be consumed from the stream regardless of
      // node validity -- the buffer is shared with the next message. The
      // answer is computed in writeAnswer (pushSolveIkToStream), which
      // always streams a status so the blocked libController call returns.
      unsigned int id;
      int nTargets;
      unsigned char hasRotations, hasToolOffset;
      stream >> id;
      stream >> nTargets;
      mSolveIkTargets.clear();
      mSolveIkRotations.clear();
      mSolveIkToolOffset.clear();
      // Bounded STORE, full CONSUME: nTargets is int32 off the wire, so a
      // corrupt value must not drive a multi-GB allocation -- but every
      // double the libController wrote must still be read, or the stream
      // desyncs for the next message. 4096 targets is far beyond any sane
      // batch and still cheap to stream.
      const int safeTargets = qBound(0, nTargets, 4096);
      mSolveIkTargets.reserve(3 * safeTargets);
      double v;
      for (int i = 0; i < 3 * nTargets; ++i) {
        stream >> v;
        if (i < 3 * safeTargets)
          mSolveIkTargets.append(v);
      }
      stream >> hasRotations;
      if (hasRotations == 1) {
        mSolveIkRotations.reserve(4 * safeTargets);
        for (int i = 0; i < 4 * nTargets; ++i) {
          stream >> v;
          if (i < 4 * safeTargets)
            mSolveIkRotations.append(v);
        }
      }
      stream >> hasToolOffset;
      if (hasToolOffset == 1)
        for (int i = 0; i < 3; ++i) {
          stream >> v;
          mSolveIkToolOffset.append(v);
        }
      stream >> mSolveIkIterations;
      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_solve_ik()");
      mSolveIkSolid = dynamic_cast<OmSolid *>(node);
      mSolveIkRequested = true;
      if (!mSolveIkSolid)
        mRobot->warn(tr("wb_supervisor_node_solve_ik() can exclusively be used with a Solid (the end effector)"));
      return;
    }
    case C_SUPERVISOR_NODE_SET_VELOCITY: {
      unsigned int id;
      double a0, a1, a2, l0, l1, l2;

      stream >> id;
      stream >> l0;
      stream >> l1;
      stream >> l2;
      stream >> a0;
      stream >> a1;
      stream >> a2;

      const double linearVelocity[3] = {l0, l1, l2};
      const double angularVelocity[3] = {a0, a1, a2};
      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_set_velocity()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      if (solid) {
        solid->setLinearVelocity(linearVelocity);
        solid->setAngularVelocity(angularVelocity);
      } else
        mRobot->warn(tr("wb_supervisor_node_set_velocity() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_RESET_PHYSICS: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_simulation_reset_physics()");
      OmSolid *solidNode = dynamic_cast<OmSolid *>(node);
      if (solidNode) {
        solidNode->resetPhysics(false);
        solidNode->pausePhysics(true);
      }
      QList<OmNode *> descendants = node->subNodes(true);
      for (int i = 0; i < descendants.size(); i++) {
        OmNode *child = descendants.at(i);
        OmSolid *solidChild = dynamic_cast<OmSolid *>(child);
        if (solidChild) {
          solidChild->resetPhysics(false);
          solidChild->pausePhysics(true);
        }
      }
      return;
    }
    case C_SUPERVISOR_NODE_RESTART_CONTROLLER: {
      unsigned int id;

      stream >> id;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_simulation_restart_controller()");
      OmRobot *const robot = dynamic_cast<OmRobot *>(node);
      if (robot)  // postpone the restart to the end of the physic step.
        robot->setControllerNeedRestart();
      else
        mRobot->warn(tr("wb_supervisor_node_restart_controller() can exclusively be used with a Robot"));
      return;
    }
    case C_SUPERVISOR_NODE_SET_VISIBILITY: {
      unsigned int nodeId, fromId;
      unsigned char visible;

      stream >> nodeId;
      stream >> fromId;
      stream >> visible;

      OmNode *const node = getProtoParameterNodeInstance(nodeId, "wb_supervisor_node_set_visibility()");
      OmNode *const cameraNode = getProtoParameterNodeInstance(fromId, "wb_supervisor_node_set_visibility()");
      OmAbstractCamera *const camera = dynamic_cast<OmAbstractCamera *>(cameraNode);
      OmViewpoint *const viewpoint = dynamic_cast<OmViewpoint *>(cameraNode);
      OmBaseNode *const baseNode = dynamic_cast<OmBaseNode *>(node);
      assert(baseNode);
      if (camera)
        // cppcheck-suppress knownConditionTrueFalse
        camera->setNodesVisibility(baseNode->findClosestDescendantNodesWithDedicatedWrenNode(), visible == 1);
      else if (viewpoint)
        // cppcheck-suppress knownConditionTrueFalse
        viewpoint->setNodesVisibility(baseNode->findClosestDescendantNodesWithDedicatedWrenNode(), visible == 1);
      return;
    }
    case C_SUPERVISOR_NODE_MOVE_VIEWPOINT: {
      unsigned int nodeId;
      stream >> nodeId;
      OmNode *const node = getProtoParameterNodeInstance(nodeId, "wb_supervisor_node_move_viewpoint()");
      OmBaseNode *const baseNode = dynamic_cast<OmBaseNode *>(node);
      assert(baseNode);
      if (OmNodeUtilities::boundingSphereAncestor(baseNode) != NULL)
        OmWorld::instance()->viewpoint()->moveViewpointToObject(baseNode);
      return;
    }
    case C_SUPERVISOR_NODE_ADD_FORCE: {
      unsigned int id;
      double fx, fy, fz;
      unsigned char relative;

      stream >> id;
      stream >> fx;
      stream >> fy;
      stream >> fz;
      stream >> relative;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_add_force()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      if (solid) {
        OmVector3 force(fx, fy, fz);
        if (relative == 1)
          force = solid->matrix().extracted3x3Matrix() * force;
        // THE GATE WAS THE ODE MERGER BODY, WHICH HAS BEEN PERMANENTLY NULL
        // SINCE THE ODE DELETION. OmSolidMerger::mBody is set to NULL in the
        // constructor and assigned nowhere in the tree, so bodyMerger() returned
        // NULL for EVERY Solid and every supervisor force was dropped before it
        // could reach the Newton path below -- while warning "kinematic Solid"
        // about a body that was fully dynamic, which sends the reader after the
        // world file instead of the gate. Measured: a 2.5 kg aircraft applying
        // lift every tick free-fell 45 m in 3.02 s.
        // Gate on the NEWTON body handle, exactly as C_SUPERVISOR_NODE_ADD_TORQUE
        // already does -- which is why torque worked while force did not.
        const OmBodyHandle h = solid->bodyHandle();
        if (h) {
          OmPhysicsBackend *const backend = solid->physicsBackend();
          backend->setBodyEnabled(h, true);
          if (!solid->applyExternalForceNewton(force, solid->computedGlobalCenterOfMass(), OmVector3()))
            mRobot->warn(tr("wb_supervisor_node_add_force() is not supported on the '%1' physics backend; "
                            "drive the joint via target velocity/position instead.")
                           .arg(backend->name()));
        } else
          mRobot->warn(tr("wb_supervisor_node_add_force() can't be used with a Solid that has no "
                          "Physics node; add one to make the body dynamic"));
      } else
        mRobot->warn(tr("wb_supervisor_node_add_force() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_ADD_FORCE_WITH_OFFSET: {
      unsigned int id;
      double fx, fy, fz, ox, oy, oz;
      unsigned char relative;

      stream >> id;
      stream >> fx;
      stream >> fy;
      stream >> fz;
      stream >> ox;
      stream >> oy;
      stream >> oz;
      stream >> relative;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_add_force_with_offset()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      if (solid) {
        // Same dead ODE gate as C_SUPERVISOR_NODE_ADD_FORCE above; see the
        // comment there. This is the verb an aerodynamic model needs, because a
        // force at an offset is what produces a moment.
        const OmBodyHandle h = solid->bodyHandle();
        if (h) {
          OmPhysicsBackend *const backend = solid->physicsBackend();
          backend->setBodyEnabled(h, true);
          // The offset is ALWAYS in the node's local frame; the force is local
          // only when `relative` is set. Keeping that asymmetry is deliberate --
          // it is the documented contract and callers depend on it.
          const OmMatrix4 &solidMatrixN = solid->matrix();
          const OmVector3 offsetW = solidMatrixN * OmVector3(ox, oy, oz);
          OmVector3 forceW(fx, fy, fz);
          if (relative == 1)
            forceW = solidMatrixN.extracted3x3Matrix() * forceW;
          if (!solid->applyExternalForceNewton(forceW, offsetW, OmVector3()))
            mRobot->warn(tr("wb_supervisor_node_add_force_with_offset() is not supported on the '%1' physics backend; "
                            "drive the joint via target velocity/position instead.")
                           .arg(backend->name()));
        } else
          mRobot->warn(tr("wb_supervisor_node_add_force_with_offset() can't be used with a Solid that has no "
                          "Physics node; add one to make the body dynamic"));
      } else
        mRobot->warn(tr("wb_supervisor_node_add_force_with_offset() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_ADD_TORQUE: {
      unsigned int id;
      double tx, ty, tz;
      unsigned char relative;

      stream >> id;
      stream >> tx;
      stream >> ty;
      stream >> tz;
      stream >> relative;

      OmNode *const node = getProtoParameterNodeInstance(id, "wb_supervisor_node_add_torque()");
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      if (solid) {
        OmVector3 torque(tx, ty, tz);
        if (relative == 1)
          torque = solid->matrix().extracted3x3Matrix() * torque;
        // Polymorphic dispatch via OmSolid::bodyHandle. ODE-backed
        // Solids hit the ODE override (byte-equivalent to the prior
        // path). Newton-backed Solids dispatch to Newton, which
        // currently returns -1 because Newton drives via joint
        // targets, not direct body torque -- we surface that as a
        // user warning rather than silently dropping the call.
        const OmBodyHandle h = solid->bodyHandle();
        if (h) {
          OmPhysicsBackend *const backend = solid->physicsBackend();
          backend->setBodyEnabled(h, true);
          // W3.1: a pure torque -> Newton's external-wrench path (force=0, so the reference point is moot).
          // For non-Newton Solids applyExternalForceNewton returns false and we fall through to the generic
          // dispatch, which on the inert tombstone returns -1 and therefore warns. That is the honest
          // outcome for a Solid pinned to the retired "ode" selector: the torque is not applied.
          if (!solid->applyExternalForceNewton(OmVector3(), solid->matrix().translation(), torque)) {
            const double t[3] = {torque.x(), torque.y(), torque.z()};
            if (backend->addBodyTorque(h, t) != 0)
              mRobot->warn(tr("wb_supervisor_node_add_torque() is not supported on the '%1' physics backend; "
                              "drive the joint via target velocity/position instead.")
                             .arg(backend->name()));
          }
        } else
          mRobot->warn(tr("wb_supervisor_node_add_torque() can't be used with a kinematic Solid"));
      } else
        mRobot->warn(tr("wb_supervisor_node_add_torque() can exclusively be used with a Solid"));
      return;
    }
    case C_SUPERVISOR_NODE_EXPORT_STRING: {
      unsigned int nodeId;
      stream >> nodeId;
      const OmNode *node = OmNode::findNode(nodeId);

      mNodeExportString = OmVrmlNodeUtilities::exportNodeToString(node);
      mNodeExportStringRequest = true;
      return;
    }
    case C_SUPERVISOR_LOAD_WORLD: {
      mWorldToLoad = readString(stream);
      makeFilenameAbsolute(mWorldToLoad);
      mLoadWorldRequested = true;
      return;
    }
    case C_SUPERVISOR_SAVE_WORLD: {
      unsigned char saveAs;

      stream >> saveAs;

      if (saveAs) {
        QString filename = readString(stream);
        makeFilenameAbsolute(filename);
        bool status = OmWorld::instance()->saveAs(filename);
        mSaveStatus = new bool[1];
        *mSaveStatus = status;
      } else {
        bool status = OmWorld::instance()->save();
        mSaveStatus = new bool[1];
        *mSaveStatus = status;
      }
      return;
    }
    case C_SUPERVISOR_NODE_GET_FIELD_COUNT: {
      int nodeId;
      unsigned char allowSearchInProto;
      stream >> nodeId;
      stream >> allowSearchInProto;

      const OmNode *const node = OmNode::findNode(nodeId);
      if (node)
        mNodeFieldCount = allowSearchInProto == 1 ? node->fields().size() : node->numFields();
      else
        mNodeFieldCount = -1;
      return;
    }
    case C_SUPERVISOR_NODE_GET_PROTO: {
      int nodeId, parentProtoId;
      stream >> nodeId;
      stream >> parentProtoId;

      mFoundProtoId = -1;
      mFoundProtoTypeName = "";
      mFoundProtoIsDerived = false;
      mFoundProtoParameterCount = -1;

      const OmNode *const node = OmNode::findNode(nodeId);
      if (node && node->isProtoInstance()) {
        if (parentProtoId < 0)
          mFoundProtoId = 0;
        else if (parentProtoId < node->protoParents().size() - 1)
          mFoundProtoId = parentProtoId + 1;
        else
          return;

        const OmNodeProtoInfo *protoInfo = node->protoParents().at(mFoundProtoId);
        mFoundProtoTypeName = protoInfo->modelName();
        mFoundProtoIsDerived = node->protoParents().size() > mFoundProtoId + 1;
        mFoundProtoParameterCount = protoInfo->parameters().size();
      }
      return;
    }
    case C_SUPERVISOR_FIELD_GET_FROM_NAME: {
      int nodeId, protoIndex;
      unsigned char allowSearchInProto;
      stream >> nodeId;
      stream >> protoIndex;
      const QString name = readString(stream);
      stream >> allowSearchInProto;

      mFoundFieldIndex = -1;
      mFoundFieldType = 0;
      mFoundFieldCount = -1;
      mFoundFieldIsInternal = false;
      mFoundFieldActualFieldNodeId = -1;
      mFoundFieldActualFieldIndex = -1;

      const OmNode *const node = OmNode::findNode(nodeId);
      if (node) {
        int fieldId;
        const OmField *field = NULL;
        if (protoIndex < 0) {
          fieldId = node->findFieldId(name, allowSearchInProto == 1);
          if (fieldId != -1) {
            field = node->field(fieldId, allowSearchInProto == 1);
            if (field) {
              const OmMultipleValue *mv = dynamic_cast<OmMultipleValue *>(field->value());
              const OmSFNode *sfNode = dynamic_cast<OmSFNode *>(field->value());
              if (mv)
                mFoundFieldCount = mv->size();
              else if (sfNode)
                mFoundFieldCount = sfNode->value() ? 1 : 0;

              mFoundFieldIsInternal = allowSearchInProto == 1;
              mFoundFieldName = field->name();

              if (mv || sfNode) {
                mWatchedFields.append(OmUpdatedFieldInfo(node->uniqueId(), field->name(), mFoundFieldCount));
                field->listenToValueSizeChanges();
                connect(field, &OmField::valueSizeChanged, this, &OmSupervisorUtilities::notifyFieldUpdate,
                        Qt::UniqueConnection);
              }
            }
          }
        } else if (protoIndex < node->protoParents().size()) {
          const OmNodeProtoInfo *protoInfo = node->protoParents().at(protoIndex);
          fieldId = protoInfo->findFieldIndex(name);
          if (fieldId >= 0) {
            const OmFieldReference &fieldRef = protoInfo->findFieldByIndex(fieldId);
            field = fieldRef.actualField;
            mFoundFieldIsInternal = true;
            mFoundFieldName = fieldRef.name;
          }

          // the supervisor will lookup the actual field for proto field reads, so there's no need send the count
          // or listen to value size changes
        }

        if (field) {
          mFoundFieldIndex = fieldId;
          mFoundFieldType = field->type();

          if (OmVrmlNodeUtilities::isVisible(field)) {
            // This only tells us that there is a corresponding parameter in the scene tree.
            // Not that this specific field is the actual field. We still have to find it.
            const OmField *actualField = field;
            while (actualField->parameter())
              actualField = actualField->parameter();

            mFoundFieldActualFieldNodeId = actualField->parentNode()->uniqueId();
            mFoundFieldActualFieldIndex = actualField->parentNode()->fieldsOrParameters().indexOf(actualField);
            assert(mFoundFieldActualFieldIndex >= 0);
          }
        }
      }
      return;
    }
    case C_SUPERVISOR_FIELD_GET_FROM_INDEX: {
      int nodeId, protoIndex, fieldIndex;
      unsigned char allowSearchInProto;
      stream >> nodeId;
      stream >> protoIndex;
      stream >> fieldIndex;
      stream >> allowSearchInProto;

      mFoundFieldIndex = -1;
      mFoundFieldType = 0;
      mFoundFieldCount = -1;
      mFoundFieldIsInternal = false;
      mFoundFieldActualFieldNodeId = -1;
      mFoundFieldActualFieldIndex = -1;

      const OmNode *const node = OmNode::findNode(nodeId);
      if (node) {
        const OmField *field = NULL;
        if (protoIndex < 0) {
          field = node->field(fieldIndex, allowSearchInProto == 1);
          if (field) {
            const OmMultipleValue *mv = dynamic_cast<OmMultipleValue *>(field->value());
            const OmSFNode *sfNode = dynamic_cast<OmSFNode *>(field->value());
            if (mv)
              mFoundFieldCount = mv->size();
            else if (sfNode)
              mFoundFieldCount = sfNode->value() ? 1 : 0;

            mFoundFieldIsInternal = allowSearchInProto == 1;
            mFoundFieldName = field->name();

            if (mv || sfNode) {
              mWatchedFields.append(OmUpdatedFieldInfo(node->uniqueId(), field->name(), mFoundFieldCount));
              field->listenToValueSizeChanges();
              connect(field, &OmField::valueSizeChanged, this, &OmSupervisorUtilities::notifyFieldUpdate, Qt::UniqueConnection);
            }
          }
        } else if (protoIndex < node->protoParents().size()) {
          const OmNodeProtoInfo *protoInfo = node->protoParents().at(protoIndex);
          const OmFieldReference &fieldRef = protoInfo->findFieldByIndex(fieldIndex);
          field = fieldRef.actualField;
          if (field) {
            mFoundFieldIsInternal = true;
            mFoundFieldName = fieldRef.name;
          }

          // the supervisor will lookup the actual field for proto field reads, so there's no need send the count
          // or listen to value size changes
        }

        if (field) {
          mFoundFieldIndex = fieldIndex;
          mFoundFieldType = field->type();

          if (OmVrmlNodeUtilities::isVisible(field)) {
            // This only tells us that there is a corresponding parameter in the scene tree.
            // Not that this specific field is the actual field. We still have to find it.
            const OmField *actualField = field;
            while (actualField->parameter())
              actualField = actualField->parameter();

            mFoundFieldActualFieldNodeId = actualField->parentNode()->uniqueId();
            mFoundFieldActualFieldIndex = actualField->parentNode()->fieldsOrParameters().indexOf(actualField);
            assert(mFoundFieldActualFieldIndex >= 0);
          }
        }
      }
      return;
    }
    case C_SUPERVISOR_CONTACT_POINTS_CHANGE_TRACKING_STATE: {
      unsigned int nodeId;
      unsigned char includeDescendantsChar;
      unsigned char enable;
      unsigned int samplingPeriod;

      stream >> nodeId;
      stream >> includeDescendantsChar;
      const bool includeDescendants = includeDescendantsChar == 1;
      stream >> enable;
      if (enable)
        stream >> samplingPeriod;

      OmNode *const node = OmNode::findNode(nodeId);
      if (!node) {
        mRobot->warn(
          tr("'wb_supervisor_node_%1_contact_point_tracking' called for an invalid node.").arg(enable ? "enable" : "disable"));
        return;
      }
      OmSolid *const solid = dynamic_cast<OmSolid *>(node);
      if (!solid) {
        mRobot->warn(tr("Node '%1' (%2) is not suitable for contact points tracking, aborting request.")
                       .arg(node->usefulName())
                       .arg(node->modelName()));
        return;
      }

      int trackingInfoIndex = -1;
      for (int i = 0; i < mTrackedContactPoints.size(); i++) {
        if (mTrackedContactPoints[i].solid == solid) {
          trackingInfoIndex = i;
          break;
        }
      }

      if (enable) {
        if (trackingInfoIndex == -1) {
          OmTrackedContactPointInfo trackedContactPoint;
          trackedContactPoint.solid = solid;
          trackedContactPoint.solidId = nodeId;
          trackedContactPoint.includeDescendants = includeDescendants;
          trackedContactPoint.samplingPeriod = 0;
          trackedContactPoint.lastUpdate = 0;
          mTrackedContactPoints.append(trackedContactPoint);
          connect(solid, &OmSolid::destroyed, this, &OmSupervisorUtilities::removeTrackedContactPoints);
        } else {
          mTrackedContactPoints[trackingInfoIndex].includeDescendants = includeDescendants;
          mTrackedContactPoints[trackingInfoIndex].samplingPeriod = samplingPeriod;
          mTrackedContactPoints[trackingInfoIndex].lastUpdate = -INFINITY;
        }
      } else if (trackingInfoIndex != -1)
        mTrackedContactPoints.removeAt(trackingInfoIndex);
      else
        mRobot->warn(tr("No active contact points tracking could be found for the node '%1'.").arg(solid->usefulName()));
      return;
    }
    case C_SUPERVISOR_POSE_CHANGE_TRACKING_STATE: {
      unsigned int fromNodeId;
      unsigned int toNodeId;
      unsigned char enable;
      unsigned int samplingPeriod;

      stream >> fromNodeId;
      stream >> toNodeId;
      stream >> enable;
      if (enable)
        stream >> samplingPeriod;

      OmNode *const toNode = OmNode::findNode(toNodeId);
      if (!toNode) {
        mRobot->warn(
          tr("'wb_supervisor_node_%1_pose_tracking' called for an invalid node.").arg(enable ? "enable" : "disable"));
        return;
      }
      OmPose *const toPoseNode = dynamic_cast<OmPose *>(toNode);
      if (!toPoseNode) {
        mRobot->warn(tr("Node '%1' (%2) is not suitable for pose tracking, aborting request.")
                       .arg(toNode->usefulName())
                       .arg(toNode->modelName()));
        return;
      }

      OmNode *const fromNode = OmNode::findNode(fromNodeId);
      int index = -1;
      for (int i = 0; i < mTrackedPoses.size(); i++) {
        if (mTrackedPoses[i].fromNode == fromNode && mTrackedPoses[i].toNode == toPoseNode) {
          index = i;
          break;
        }
      }

      if (enable) {
        OmPose *const fromPoseNode = fromNode ? dynamic_cast<OmPose *>(fromNode) : NULL;
        if (fromNodeId && !fromPoseNode)
          mRobot->warn(tr("Pose tracking can be exclusively used with Pose (or derived) 'from_node' argument, but '%1' (%2) is "
                          "given. The absolute pose in global coordinates will be returned.")
                         .arg(fromNode->usefulName())
                         .arg(fromNode->modelName()));

        if (index < 0) {
          OmTrackedPoseInfo trackedPose;
          trackedPose.fromNode = fromPoseNode;
          trackedPose.toNode = toPoseNode;
          trackedPose.samplingPeriod = samplingPeriod;
          trackedPose.lastUpdate = -INFINITY;
          mTrackedPoses.append(trackedPose);
          if (fromPoseNode)
            connect(fromPoseNode, &OmNode::destroyed, this, &OmSupervisorUtilities::removeTrackedPoseNode);
          connect(toPoseNode, &OmNode::destroyed, this, &OmSupervisorUtilities::removeTrackedPoseNode);
        } else {
          mTrackedPoses[index].samplingPeriod = samplingPeriod;
          mTrackedPoses[index].lastUpdate = -INFINITY;
        }
      } else if (index >= 0)
        mTrackedPoses.removeAt(index);
      else
        mRobot->warn(tr("No active pose tracking could be found matching nodes '%1' (to) and '%2' (from) arguments.")
                       .arg(toNode->usefulName())
                       .arg(fromNode->usefulName()));

      return;
    }
    case C_SUPERVISOR_FIELD_CHANGE_TRACKING_STATE: {
      unsigned int nodeId;
      unsigned int fieldId;
      unsigned char internal = false;
      unsigned char enable;
      unsigned int samplingPeriod;

      stream >> nodeId;
      stream >> fieldId;
      stream >> internal;
      stream >> enable;
      if (enable)
        stream >> samplingPeriod;

      const OmNode *const node = OmNode::findNode(nodeId);
      if (!node) {
        mRobot->warn(tr("'wb_supervisor_field_%1_sf_tracking' called for an invalid node.").arg(enable ? "enable" : "disable"));
        return;
      }

      OmField *field = node->field(fieldId, internal == 1);
      if (!field) {
        mRobot->warn(
          tr("'wb_supervisor_field_%1_sf_tracking' called for an invalid field.").arg(enable ? "enable" : "disable"));
        return;
      }

      int index = -1;
      for (int i = 0; i < mTrackedFields.size(); i++) {
        if (mTrackedFields[i].field == field) {
          index = i;
          break;
        }
      }

      if (enable) {
        if (index < 0) {
          OmTrackedFieldInfo trackedField;
          trackedField.field = field;
          trackedField.samplingPeriod = samplingPeriod;
          trackedField.lastUpdate = -INFINITY;
          trackedField.fieldId = fieldId;
          trackedField.nodeId = nodeId;
          trackedField.internal = internal;
          mTrackedFields.append(trackedField);
          connect(field, &OmField::destroyed, this, &OmSupervisorUtilities::removeTrackedField);
        } else {
          mTrackedFields[index].samplingPeriod = samplingPeriod;
          mTrackedFields[index].lastUpdate = -INFINITY;
        }
      } else if (index >= 0)
        mTrackedFields.removeAt(index);
      else
        mRobot->warn(tr("No active field tracking could be found matching the field '%1' of node '%2'.")
                       .arg(field->name())
                       .arg(node->modelName()));

      return;
    }
    case C_SUPERVISOR_FIELD_GET_VALUE: {
      unsigned int uniqueId, fieldId;
      int protoId, index = -1;
      unsigned char internal = false;

      stream >> uniqueId;
      stream >> protoId;
      stream >> fieldId;
      stream >> internal;

      const OmNode *const node = OmNode::findNode(uniqueId);
      OmField *field = NULL;

      if (node) {
        if (protoId < 0) {
          field = node->field(fieldId, internal == 1);
        } else if (protoId < node->protoParents().size()) {
          const OmNodeProtoInfo *protoInfo = node->protoParents().at(protoId);
          field = protoInfo->findFieldByIndex(fieldId).actualField;
        }

        if (field && field->isMultiple())
          stream >> index;
      }

      assert(!mFieldGetRequest);
      mFieldGetRequest = new struct OmFieldGetRequest;
      mFieldGetRequest->field = field;
      mFieldGetRequest->index = index;
      mFieldGetRequest->fieldId = fieldId;
      mFieldGetRequest->nodeId = uniqueId;
      mFieldGetRequest->protoId = protoId;
      return;
    }
    case C_SUPERVISOR_FIELD_SET_VALUE: {
      unsigned int uniqueId, fieldId, fieldType;
      int index;

      stream >> uniqueId;
      stream >> fieldId;
      stream >> fieldType;
      stream >> index;
      const OmNode *const node = OmNode::findNode(uniqueId);
      OmField *field = node ? node->field(fieldId) : NULL;

      // we read the data depending on the field type
      unsigned char b = 0;
      int i = 0;
      double d0 = 0.0, d1 = 0.0, d2 = 0.0, d3 = 0.0;

      switch (fieldType) {
        case WB_SF_BOOL:
        case WB_MF_BOOL:
          stream >> b;
          mFieldSetRequests << new OmBoolFieldSetRequest(field, index, b);
          break;
        case WB_SF_INT32:
        case WB_MF_INT32:
          stream >> i;
          mFieldSetRequests << new OmIntFieldSetRequest(field, index, i);
          break;
        case WB_SF_FLOAT:
        case WB_MF_FLOAT:
          stream >> d0;
          mFieldSetRequests << new OmDoubleFieldSetRequest(field, index, d0);
          break;
        case WB_SF_VEC2F:
        case WB_MF_VEC2F:
          stream >> d0;
          stream >> d1;
          mFieldSetRequests << new OmVector2FieldSetRequest(field, index, d0, d1);
          break;
        case WB_SF_COLOR:
        case WB_MF_COLOR:
          stream >> d0;
          stream >> d1;
          stream >> d2;
          mFieldSetRequests << new OmColorFieldSetRequest(field, index, d0, d1, d2);
          break;
        case WB_SF_VEC3F:
        case WB_MF_VEC3F:
          stream >> d0;
          stream >> d1;
          stream >> d2;
          mFieldSetRequests << new OmVector3FieldSetRequest(field, index, d0, d1, d2);
          break;
        case WB_SF_ROTATION:
        case WB_MF_ROTATION:
          stream >> d0;
          stream >> d1;
          stream >> d2;
          stream >> d3;
          mFieldSetRequests << new OmRotationFieldSetRequest(field, index, d0, d1, d2, d3);
          break;
        case WB_SF_STRING:
        case WB_MF_STRING: {
          const QString s = readString(stream);
          mFieldSetRequests << new OmStringFieldSetRequest(field, index, s);
          break;
        }
      }
      return;
    }
    case C_SUPERVISOR_FIELD_INSERT_VALUE: {
      unsigned int nodeId, fieldId, index;

      stream >> nodeId;
      stream >> fieldId;
      stream >> index;

      // apply queued set field operations
      processImmediateMessages(true);

      const OmNode *const node = OmNode::findNode(nodeId);
      const OmField *field = node->field(fieldId);

      switch (field->type()) {  // import value
        case WB_MF_BOOL: {
          // cppcheck-suppress unassignedVariable
          unsigned char value;
          stream >> value;
          // cppcheck-suppress knownConditionTrueFalse
          (dynamic_cast<OmMFBool *>(field->value()))->insertItem(index, value == 1);
          break;
        }
        case WB_MF_INT32: {
          int value;
          stream >> value;
          (dynamic_cast<OmMFInt *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_FLOAT: {
          double value;
          stream >> value;
          (dynamic_cast<OmMFDouble *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_VEC2F: {
          double d0, d1;
          stream >> d0;
          stream >> d1;
          OmVector2 value(d0, d1);
          (dynamic_cast<OmMFVector2 *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_VEC3F: {
          double d0, d1, d2;
          stream >> d0;
          stream >> d1;
          stream >> d2;
          OmVector3 value(d0, d1, d2);
          (dynamic_cast<OmMFVector3 *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_ROTATION: {
          double d0, d1, d2, d3;
          stream >> d0;
          stream >> d1;
          stream >> d2;
          stream >> d3;
          OmRotation value(d0, d1, d2, d3);
          (dynamic_cast<OmMFRotation *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_COLOR: {
          double d0, d1, d2;
          stream >> d0;
          stream >> d1;
          stream >> d2;
          OmRgb value(d0, d1, d2);
          (dynamic_cast<OmMFColor *>(field->value()))->insertItem(index, value);
          break;
        }
        case WB_MF_STRING: {
          const QString string = readString(stream);
          (dynamic_cast<OmMFString *>(field->value()))->insertItem(index, string);
          break;
        }
        case WB_MF_NODE:
        case WB_SF_NODE: {
          const QString nodeString = readString(stream);
          processImmediateMessages(true);  // apply queued set field operations
          OmNodeOperations::instance()->importNode(nodeId, fieldId, index, OmNodeOperations::FROM_SUPERVISOR, nodeString);
          const OmSFNode *sfNode = dynamic_cast<OmSFNode *>(OmNode::findNode(nodeId)->field(fieldId)->value());
          mImportedNodeId = sfNode && sfNode->value() ? sfNode->value()->uniqueId() : -1;
          break;
        }
        default:
          assert(0);
      }

      OmTemplateManager::instance()->blockRegeneration(false);
      emit worldModified();
      return;
    }
    case C_SUPERVISOR_NODE_REMOVE_NODE: {
      unsigned int nodeId;
      stream >> nodeId;
      OmNode *node = OmNode::findNode(nodeId);

      // as findNode might return the internal one, it's necessary to climb the ladder up to the protoParameterNode otherwise
      // the scene tree will not be refreshed when deleting it
      while (node && node->protoParameterNode())
        node = node->protoParameterNode();

      if (!OmVrmlNodeUtilities::isVisible(node)) {
        mRobot->warn(
          tr("Node '%1' is internal to a PROTO and therefore cannot be deleted from a Supervisor.").arg(node->usefulName()));
        return;
      }

      if (node) {
        if (node == mRobot)
          mShouldRemoveNode = true;
        else {
          OmNodeOperations::instance()->deleteNode(node, true);
          emit worldModified();
        }
      }
      return;
    }
    case C_SUPERVISOR_FIELD_REMOVE_VALUE: {
      int index;
      unsigned int nodeId, fieldId;
      stream >> nodeId;
      stream >> fieldId;
      stream >> index;

      // apply queued set field operations
      processImmediateMessages(true);

      bool modified = false;
      const OmNode *parentNode = OmNode::findNode(nodeId);
      OmField *field = parentNode->field(fieldId);
      switch (field->type()) {  // remove value
        case WB_MF_BOOL:
        case WB_MF_INT32:
        case WB_MF_FLOAT:
        case WB_MF_VEC2F:
        case WB_MF_VEC3F:
        case WB_MF_ROTATION:
        case WB_MF_COLOR:
        case WB_MF_STRING: {
          OmMultipleValue *multipleValue = dynamic_cast<OmMultipleValue *>(field->value());
          assert(multipleValue->size() > index);
          multipleValue->removeItem(index);
          modified = true;
          break;
        }
        case WB_MF_NODE: {
          const OmMFNode *mfNode = dynamic_cast<OmMFNode *>(field->value());
          assert(mfNode->size() > index);
          OmNode *node = mfNode->item(index);

          const OmViewpoint *viewpoint = dynamic_cast<OmViewpoint *>(node);
          const OmWorldInfo *worldInfo = dynamic_cast<OmWorldInfo *>(node);
          if (viewpoint || worldInfo) {
            node = NULL;
            mRobot->warn(tr(
              "wb_supervisor_field_remove_mf() called with the 'index' argument referring to a Viewpoint or WorldInfo node."));
          }

          if (node) {
            if (node == mRobot)
              mShouldRemoveNode = true;
            else {
              OmNodeOperations::instance()->deleteNode(node, true);
              modified = true;
            }
          }
          break;
        }
        case WB_SF_NODE: {
          const OmSFNode *sfNode = dynamic_cast<OmSFNode *>(field->value());
          if (sfNode->value()) {
            if (sfNode->value() == mRobot)
              mShouldRemoveNode = true;
            else {
              OmNodeOperations::instance()->deleteNode(sfNode->value(), true);
              modified = true;
            }
          }
          break;
        }
        default:
          assert(0);
      }

      OmTemplateManager::instance()->blockRegeneration(false);
      if (modified)
        emit worldModified();

      return;
    }
    // The VirtualRealityHeadset node was retired together with the WREN renderer. These three
    // protocol cases are deliberately KEPT (inert): this switch ends in `default: assert(0)`, so an
    // already-compiled controller's request byte must still be accepted here. They answer exactly
    // what the pre-existing "no headset in use" path answered -- which is also what every non-Windows
    // build has always answered -- namely "not used", and no position/orientation reply at all, which
    // libController turns into its invalid (NaN) vector.
    case C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_IS_USED: {
      mVirtualRealityHeadsetIsUsedRequested = true;
      return;
    }
    case C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_POSITION: {
      mRobot->warn(tr(
        "wb_supervisor_virtual_reality_headset_get_position() called but no virtual reality headset is currently in use."));
      return;
    }
    case C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_GET_ORIENTATION: {
      mRobot->warn(
        tr("wb_supervisor_virtual_reality_headset_get_orientation() called but no virtual reality headset is currently in "
           "use."));
      return;
    }
    default:
      assert(0);
  }
}

void OmSupervisorUtilities::writeNode(OmDataStream &stream, const OmBaseNode *baseNode, int messageType) {
  assert(baseNode);
  stream << (int)baseNode->uniqueId();
  stream << (int)baseNode->nodeType();
  const OmDevice *device = dynamic_cast<const OmDevice *>(baseNode);
  stream << (int)((device && mRobot->findDevice(device->tag()) == device) ? device->tag() : -1);
  stream << (int)(baseNode->parentNode() ? baseNode->parentNode()->uniqueId() : -1);
  stream << (unsigned char)baseNode->isProtoInstance();
  const QByteArray &modelName = baseNode->modelName().toUtf8();
  const QByteArray &defName = baseNode->defName().toUtf8();
  stream.writeRawData(modelName.constData(), modelName.size() + 1);
  stream.writeRawData(defName.constData(), defName.size() + 1);
  if (messageType == C_SUPERVISOR_FIELD_GET_VALUE)
    connect(baseNode, &OmNode::defUseNameChanged, this, &OmSupervisorUtilities::notifyNodeUpdate, Qt::UniqueConnection);
}

void OmSupervisorUtilities::pushSingleFieldContentToStream(OmDataStream &stream, OmField *field) {
  switch (field->type()) {
    case WB_SF_BOOL: {
      bool v = dynamic_cast<OmSFBool *>(field->value())->value();
      stream << (unsigned char)v;
      break;
    }
    case WB_SF_INT32: {
      int v = dynamic_cast<OmSFInt *>(field->value())->value();
      stream << (int)v;
      break;
    }
    case WB_SF_FLOAT: {
      double v = dynamic_cast<OmSFDouble *>(field->value())->value();
      stream << (double)v;
      break;
    }
    case WB_SF_VEC2F: {
      const OmVector2 &v = dynamic_cast<OmSFVector2 *>(field->value())->value();
      stream << (double)v.x();
      stream << (double)v.y();
      break;
    }
    case WB_SF_VEC3F: {
      const OmVector3 &v = dynamic_cast<OmSFVector3 *>(field->value())->value();
      stream << (double)v.x();
      stream << (double)v.y();
      stream << (double)v.z();
      break;
    }
    case WB_SF_ROTATION: {
      const OmRotation &v = dynamic_cast<OmSFRotation *>(field->value())->value();
      stream << (double)v.x();
      stream << (double)v.y();
      stream << (double)v.z();
      stream << (double)v.angle();
      break;
    }
    case WB_SF_COLOR: {
      const OmRgb &v = dynamic_cast<OmSFColor *>(field->value())->value();
      stream << (double)v.red();
      stream << (double)v.green();
      stream << (double)v.blue();
      break;
    }
    case WB_SF_STRING: {
      const QString &v = dynamic_cast<OmSFString *>(field->value())->value();
      QByteArray ba = v.toUtf8();
      stream.writeRawData(ba.constData(), ba.size() + 1);
      break;
    }
    case WB_SF_NODE: {
      OmNode *const node = dynamic_cast<OmSFNode *>(field->value())->value();
      const OmBaseNode *const baseNode = dynamic_cast<OmBaseNode *>(node);
      if (baseNode)
        writeNode(stream, baseNode, C_SUPERVISOR_FIELD_GET_VALUE);
      else
        stream << (int)0;  // NULL node case
      break;
    }
    default:
      assert(0);
      break;
  }
}

void OmSupervisorUtilities::pushRelativePoseToStream(OmDataStream &stream, OmPose *fromNode, OmPose *toNode) {
  OmMatrix4 m;

  OmMatrix4 mTo(toNode->matrix());
  const OmVector3 &sTo = mTo.scale();
  mTo.scale(1.0 / sTo.x(), 1.0 / sTo.y(), 1.0 / sTo.z());

  if (fromNode) {
    OmMatrix4 mFrom(fromNode->matrix());
    const OmVector3 &sFrom = mFrom.scale();
    mFrom.scale(1.0 / sFrom.x(), 1.0 / sFrom.y(), 1.0 / sFrom.z());

    m = mFrom.pseudoInversed() * mTo;
  } else
    m = mTo;

  stream << (short unsigned int)0;
  stream << (unsigned char)C_SUPERVISOR_NODE_GET_POSE;
  if (fromNode)
    stream << (int)fromNode->uniqueId();
  else
    stream << 0;
  stream << (int)toNode->uniqueId();
  stream << (double)m(0, 0) << (double)m(0, 1) << (double)m(0, 2) << (double)m(0, 3);
  stream << (double)m(1, 0) << (double)m(1, 1) << (double)m(1, 2) << (double)m(1, 3);
  stream << (double)m(2, 0) << (double)m(2, 1) << (double)m(2, 2) << (double)m(2, 3);
  stream << (double)m(3, 0) << (double)m(3, 1) << (double)m(3, 2) << (double)m(3, 3);
}

void OmSupervisorUtilities::pushContactPointsToStream(OmDataStream &stream, OmSolid *solid, int solidId,
                                                      bool includeDescendants) {
  const QVector<OmVector3> &contactPoints = solid->computedContactPoints(includeDescendants);
  const QVector<const OmSolid *> &solids = solid->computedSolidPerContactPoints();
  const QVector<double> &depths = solid->computedContactPointDepths(includeDescendants);
  const int size = contactPoints.size();
  stream << (short unsigned int)0;
  stream << (unsigned char)C_SUPERVISOR_NODE_GET_CONTACT_POINTS;
  stream << (int)solidId;
  stream << (unsigned char)(includeDescendants ? 1 : 0);
  stream << (int)size;
  for (int i = 0; i < size; ++i) {
    const OmVector3 &v = contactPoints.at(i);
    stream << (double)v.x();
    stream << (double)v.y();
    stream << (double)v.z();
    stream << (int)(includeDescendants ? solids.at(i)->uniqueId() : solid->uniqueId());
    // Trailing per-contact penetration depth (new field 2026-05-29); readers
    // (C supervisor.c, Python node.py) consume it positionally after node_id.
    stream << (double)(i < depths.size() ? depths.at(i) : 0.0);
  }
}

void OmSupervisorUtilities::pushSolveIkToStream(OmDataStream &stream) {
  // wb_supervisor_node_solve_ik answer (internal parity plan, item W2.1). A PURE
  // PREVIEW: World.solve_ik owns its buffers and never writes state_a, so
  // nothing in the scene moves. Status codes mirror the public header:
  //   0 ok, -1 no Newton backend / world not finalised, -2 the end effector
  //   owns no Newton body, -3 the robot has no IK-solvable joints, -4 the
  //   solver call failed (engine log carries the Python error).
  // The answer is ALWAYS streamed -- the libController side selectively
  // blocks on it, and a silent no-answer would read as -9 forever.
  int status = 0;
  const int nTargets = mSolveIkTargets.size() / 3;
  QVector<int> jointIds;
  std::vector<int> ikSlots;
  std::vector<double> angles;
  std::vector<double> residuals;

  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  OmNewtonBackend *const newton =
    (raw != nullptr && raw->isAvailable()) ? static_cast<OmNewtonBackend *>(raw) : nullptr;
  if (newton == nullptr)
    status = -1;

  int bodyIdx = -1;
  OmRobot *targetRobot = nullptr;
  if (status == 0) {
    if (mSolveIkSolid == nullptr)
      status = -2;
    else {
      // Merger-aware: a gripper-tip Solid folded into its parent body still
      // resolves to the body that carries it in the Newton model.
      bodyIdx = mSolveIkSolid->nearestNewtonBodyIndex();
      targetRobot = mSolveIkSolid->robot();
      if (bodyIdx < 0 || targetRobot == nullptr) {
        status = -2;
        mRobot->warn(tr("wb_supervisor_node_solve_ik(): the end effector owns no Newton physics body "
                        "(is the world finalised, and does the chain carry Physics nodes?)"));
      }
    }
  }

  if (status == 0) {
    // Solve for every 1-coordinate joint slot of the END EFFECTOR'S robot.
    // EXACT nodeType() match, not dynamic_cast: Hinge2Joint and BallJoint
    // both INHERIT OmHingeJoint but register multi-coordinate joints, which
    // World.solve_ik refuses (the scalar write-back would be wrong for
    // them). The slot restriction doubles as the joint_dof_mask: without
    // it the optimiser "reaches" targets by moving other robots' joints or
    // a floating base the caller cannot command (measured 0.923 m of base
    // translation -- see tests/test_newton_ik_slots.py).
    foreach (OmNode *sub, targetRobot->subNodes(true)) {
      OmBasicJoint *const j = dynamic_cast<OmBasicJoint *>(sub);
      if (j == nullptr)
        continue;
      const int nt = j->nodeType();
      if (nt != WB_NODE_HINGE_JOINT && nt != WB_NODE_SLIDER_JOINT)
        continue;
      const int slot = j->newtonJointIndex();
      if (slot < 0)
        continue;
      ikSlots.push_back(slot);
      jointIds.append(j->uniqueId());
    }
    if (ikSlots.empty()) {
      status = -3;
      mRobot->warn(tr("wb_supervisor_node_solve_ik(): robot '%1' has no IK-solvable joints (no Hinge/Slider "
                      "joint registered with the physics backend)").arg(targetRobot->name()));
    }
  }

  if (status == 0) {
    const int rc = newton->solveIk(
      bodyIdx, nTargets, mSolveIkTargets.constData(),
      mSolveIkRotations.isEmpty() ? nullptr : mSolveIkRotations.constData(), ikSlots,
      mSolveIkToolOffset.size() == 3 ? mSolveIkToolOffset.constData() : nullptr, mSolveIkIterations, angles, residuals);
    if (rc != 0) {
      status = -4;
      mRobot->warn(tr("wb_supervisor_node_solve_ik(): the IK solve failed -- see the engine log for the "
                      "solver's own error"));
    }
  }

  const int nJoints = status == 0 ? (int)ikSlots.size() : 0;
  const int nAnswerTargets = status == 0 ? nTargets : 0;
  stream << (short unsigned int)0;
  stream << (unsigned char)C_SUPERVISOR_NODE_SOLVE_IK;
  stream << (int)status;
  stream << (int)nJoints;
  stream << (int)nAnswerTargets;
  for (int i = 0; i < nJoints; ++i)
    stream << (int)jointIds.at(i);
  for (int i = 0; i < nAnswerTargets * nJoints; ++i)
    stream << (double)angles[i];
  for (int i = 0; i < nAnswerTargets; ++i)
    stream << (double)residuals[i];
}

void OmSupervisorUtilities::writeAnswer(OmDataStream &stream) {
  if (!mUpdatedNodeIds.isEmpty()) {
    foreach (int id, mUpdatedNodeIds) {
      const OmBaseNode *baseNode = dynamic_cast<const OmBaseNode *>(OmNode::findNode(id));
      if (baseNode) {
        stream << (short unsigned int)0;
        stream << (unsigned char)C_SUPERVISOR_NODE_GET_FROM_ID;
        writeNode(stream, baseNode, C_SUPERVISOR_NODE_GET_FROM_ID);
      }
    }
    mUpdatedNodeIds.clear();
  }
  if (mGetNodeRequest > 0) {
    stream << (short unsigned int)0;
    stream << (unsigned char)mGetNodeRequest;
    stream << (int)mFoundNodeUniqueId;
    stream << (int)mFoundNodeType;
    stream << (int)mFoundNodeTag;
    stream << (int)mFoundNodeParentUniqueId;
    stream << (unsigned char)mFoundNodeIsProto;
    stream << (unsigned char)mFoundNodeIsProtoInternal;
    const QByteArray &modelName = mFoundNodeModelName.toUtf8();
    const QByteArray &defName = mCurrentDefName.toUtf8();
    stream.writeRawData(modelName.constData(), modelName.size() + 1);
    stream.writeRawData(defName.constData(), defName.size() + 1);
    mFoundNodeUniqueId = -1;
    mCurrentDefName.clear();
    mGetNodeRequest = 0;
  }
  if (mFoundNodeUniqueId != -1) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_FROM_DEF;
    stream << (int)mFoundNodeUniqueId;
    stream << (int)mFoundNodeType;
    stream << (int)mFoundNodeTag;
    stream << (int)mFoundNodeParentUniqueId;
    stream << (unsigned char)mFoundNodeIsProto;
    const QByteArray s = mFoundNodeModelName.toUtf8();
    stream.writeRawData(s.constData(), s.size() + 1);
    mFoundNodeUniqueId = -1;
  }
  if (mFoundProtoId != -2) {  // enabled, -1 means not found
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_PROTO;
    stream << (int)mFoundProtoId;
    stream << (unsigned char)mFoundProtoIsDerived;
    stream << (int)mFoundProtoParameterCount;
    const QByteArray ba = mFoundProtoTypeName.toUtf8();
    stream.writeRawData(ba.constData(), ba.size() + 1);
    mFoundProtoId = -2;
  }
  if (mFoundFieldIndex != -2) {  // enabled, -1 means not found
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_FIELD_GET_FROM_NAME;
    stream << (int)mFoundFieldIndex;
    stream << (int)mFoundFieldType;
    stream << (unsigned char)mFoundFieldIsInternal;
    stream << (int)mFoundFieldCount;
    stream << (int)mFoundFieldActualFieldNodeId;
    stream << (int)mFoundFieldActualFieldIndex;
    const QByteArray ba = mFoundFieldName.toUtf8();
    stream.writeRawData(ba.constData(), ba.size() + 1);
    mFoundFieldIndex = -2;
  }
  if (mIsProtoRegenerated) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_REGENERATED;
    mIsProtoRegenerated = false;
  }
  if (!mNodesDeletedSinceLastStep.isEmpty()) {
    const int size = mNodesDeletedSinceLastStep.size();
    for (int i = 0; i < size; ++i) {
      stream << (short unsigned int)0;
      stream << (unsigned char)C_SUPERVISOR_NODE_REMOVE_NODE;
      stream << (int)mNodesDeletedSinceLastStep[i];
    }
    mNodesDeletedSinceLastStep.clear();
  }
  if (mNodeGetPosition) {
    const OmVector3 &pos = mNodeGetPosition->matrix().translation();
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_POSITION;
    stream << (double)pos.x();
    stream << (double)pos.y();
    stream << (double)pos.z();
    mNodeGetPosition = NULL;
  }
  if (mNodeGetOrientation) {
    OmMatrix4 m(mNodeGetOrientation->matrix());
    // remove scale from matrix
    const OmVector3 &s = m.scale();
    m.scale(1.0 / s.x(), 1.0 / s.y(), 1.0 / s.z());
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_ORIENTATION;
    stream << (double)m(0, 0) << (double)m(0, 1) << (double)m(0, 2);
    stream << (double)m(1, 0) << (double)m(1, 1) << (double)m(1, 2);
    stream << (double)m(2, 0) << (double)m(2, 1) << (double)m(2, 2);
    mNodeGetOrientation = NULL;
  }
  for (OmTrackedPoseInfo &pose : mTrackedPoses) {
    const double time = OmSimulationState::instance()->time();
    if (time < pose.lastUpdate || time >= pose.lastUpdate + pose.samplingPeriod) {
      pushRelativePoseToStream(stream, pose.fromNode, pose.toNode);
      pose.lastUpdate = time;
    }
  }
  if (mNodeGetPose.second) {
    pushRelativePoseToStream(stream, mNodeGetPose.first, mNodeGetPose.second);
    mNodeGetPose.first = NULL;
    mNodeGetPose.second = NULL;
  }
  if (mNodeGetCenterOfMass) {
    const OmVector3 &com = mNodeGetCenterOfMass->computedGlobalCenterOfMass();
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_CENTER_OF_MASS;
    stream << (double)com.x() << (double)com.y() << (double)com.z();
    mNodeGetCenterOfMass = NULL;
  }
  for (OmTrackedContactPointInfo &info : mTrackedContactPoints) {
    const double time = OmSimulationState::instance()->time();
    if (time < info.lastUpdate || time >= info.lastUpdate + info.samplingPeriod) {
      pushContactPointsToStream(stream, info.solid, info.solidId, info.includeDescendants);
      info.lastUpdate = time;
    }
  }
  if (mNodeGetContactPoints) {
    pushContactPointsToStream(stream, mNodeGetContactPoints, mNodeIdGetContactPoints, mGetContactPointsIncludeDescendants);
    mNodeGetContactPoints = NULL;
  }
  if (mSolveIkRequested) {
    pushSolveIkToStream(stream);
    mSolveIkRequested = false;
    mSolveIkSolid = NULL;
    mSolveIkTargets.clear();
    mSolveIkRotations.clear();
    mSolveIkToolOffset.clear();
  }
  if (mNodeGetStaticBalance) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_STATIC_BALANCE;
    stream << (unsigned char)mNodeGetStaticBalance->staticBalance();
    mNodeGetStaticBalance = NULL;
  }
  if (mNodeGetVelocity) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_VELOCITY;
    const OmVector3 linearVelocity = mNodeGetVelocity->relativeLinearVelocity();
    const OmVector3 angularVelocity = mNodeGetVelocity->relativeAngularVelocity();
    stream << (double)linearVelocity[0];
    stream << (double)linearVelocity[1];
    stream << (double)linearVelocity[2];
    stream << (double)angularVelocity[0];
    stream << (double)angularVelocity[1];
    stream << (double)angularVelocity[2];
    mNodeGetVelocity = NULL;
  }
  if (mNodeExportStringRequest) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_EXPORT_STRING;
    const QByteArray ba = mNodeExportString.toUtf8();
    stream.writeRawData(ba.constData(), ba.size() + 1);
    mNodeExportStringRequest = false;
  }
  if (mImportedNodeId >= 0) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_FIELD_INSERT_VALUE;
    stream << (int)mImportedNodeId;
    mImportedNodeId = -1;
  }
  if (mNodeFieldCount >= 0) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_NODE_GET_FIELD_COUNT;
    stream << mNodeFieldCount;
    mNodeFieldCount = -1;
    return;
  }
  for (OmTrackedFieldInfo &field : mTrackedFields) {
    const double time = OmSimulationState::instance()->time();
    if (time < field.lastUpdate || time >= field.lastUpdate + field.samplingPeriod) {
      stream << (short unsigned int)0;
      stream << (unsigned char)C_SUPERVISOR_FIELD_GET_VALUE;
      stream << (int)field.field->type();
      stream << (int)field.nodeId;
      stream << (int)-1;  // Proto fields cannot be tracked
      stream << (int)field.fieldId;
      pushSingleFieldContentToStream(stream, field.field);
      field.lastUpdate = time;
    }
  }
  if (mFieldGetRequest) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_FIELD_GET_VALUE;

    OmField *field = mFieldGetRequest->field;
    if (!field) {  // may happen if the object was deleted
      delete mFieldGetRequest;
      mFieldGetRequest = NULL;
      stream << (int)0;
      return;
    }
    stream << (int)field->type();
    stream << (int)mFieldGetRequest->nodeId;
    stream << (int)mFieldGetRequest->protoId;
    stream << (int)mFieldGetRequest->fieldId;
    switch (field->type()) {
      case WB_MF_BOOL: {
        const bool v = dynamic_cast<OmMFBool *>(field->value())->item(mFieldGetRequest->index);
        stream << (unsigned char)v;
        break;
      }
      case WB_MF_INT32: {
        const int v = dynamic_cast<OmMFInt *>(field->value())->item(mFieldGetRequest->index);
        stream << (int)v;
        break;
      }
      case WB_MF_FLOAT: {
        const double v = dynamic_cast<OmMFDouble *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v;
        break;
      }
      case WB_MF_VEC2F: {
        const OmVector2 &v = dynamic_cast<OmMFVector2 *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.x();
        stream << (double)v.y();
        break;
      }
      case WB_MF_VEC3F: {
        const OmVector3 &v = dynamic_cast<OmMFVector3 *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.x();
        stream << (double)v.y();
        stream << (double)v.z();
        break;
      }
      case WB_MF_COLOR: {
        const OmRgb &v = dynamic_cast<OmMFColor *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.red();
        stream << (double)v.green();
        stream << (double)v.blue();
        break;
      }
      case WB_MF_ROTATION: {
        const OmRotation &v = dynamic_cast<OmMFRotation *>(field->value())->item(mFieldGetRequest->index);
        stream << (double)v.x();
        stream << (double)v.y();
        stream << (double)v.z();
        stream << (double)v.angle();
        break;
      }
      case WB_MF_STRING: {
        const QString &v = dynamic_cast<OmMFString *>(field->value())->item(mFieldGetRequest->index);
        QByteArray ba = v.toUtf8();
        stream.writeRawData(ba.constData(), ba.size() + 1);
        break;
      }
      case WB_MF_NODE: {
        const OmMFNode::OmNodePtr &v = dynamic_cast<OmMFNode *>(field->value())->item(mFieldGetRequest->index);
        OmNode *const node = dynamic_cast<OmNode *>(v);
        const OmBaseNode *const baseNode = dynamic_cast<OmBaseNode *>(node);
        if (baseNode)
          writeNode(stream, baseNode, C_SUPERVISOR_FIELD_GET_VALUE);
        else
          stream << (int)0;  // NULL node case
        break;
      }
      default:
        pushSingleFieldContentToStream(stream, field);
        break;
    }
    delete mFieldGetRequest;
    mFieldGetRequest = NULL;
  }
  if (!mUpdatedFields.isEmpty()) {
    foreach (const OmUpdatedFieldInfo &info, mUpdatedFields) {
      stream << (short unsigned int)0;
      stream << (unsigned char)C_SUPERVISOR_FIELD_COUNT_CHANGED;
      stream << (int)info.nodeId;
      const QByteArray ba = info.fieldName.toUtf8();
      stream.writeRawData(ba.constData(), ba.size() + 1);
      stream << (int)info.fieldCount;
    }
    mUpdatedFields.clear();
  }
  if (mMovieStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_MOVIE_STATUS;
    stream << (unsigned char)*mMovieStatus;
    delete mMovieStatus;
    mMovieStatus = NULL;
  }
  if (mAnimationStartStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_ANIMATION_START_STATUS;
    stream << (unsigned char)*mAnimationStartStatus;
    delete mAnimationStartStatus;
    mAnimationStartStatus = NULL;
  }
  if (mAnimationStopStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_ANIMATION_STOP_STATUS;
    stream << (unsigned char)*mAnimationStopStatus;
    delete mAnimationStopStatus;
    mAnimationStopStatus = NULL;
  }
  if (mSaveStatus) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_SAVE_WORLD;
    stream << (unsigned char)*mSaveStatus;
    delete mSaveStatus;
    mSaveStatus = NULL;
  }
  if (mVirtualRealityHeadsetIsUsedRequested) {
    // The VirtualRealityHeadset node was retired together with the WREN renderer. The answer is now
    // unconditionally "not used"; the reply itself is kept so that an already-compiled controller
    // calling wb_supervisor_virtual_reality_headset_is_used() still receives a well-formed response.
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_VIRTUAL_REALITY_HEADSET_IS_USED;
    stream << (unsigned char)0;
    mVirtualRealityHeadsetIsUsedRequested = false;
  }
  if (mSimulationReset) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_SIMULATION_RESET;
    mSimulationReset = false;
  }
}

void OmSupervisorUtilities::writeConfigure(OmDataStream &stream) {
  OmNode *selfNode = mRobot;
  while (selfNode->protoParameterNode())
    selfNode = selfNode->protoParameterNode();
  stream << (short unsigned int)0;
  stream << (unsigned char)C_CONFIGURE;
  stream << (int)selfNode->uniqueId();
  stream << (int)selfNode->parentNode()->uniqueId();
  stream << (unsigned char)selfNode->isProtoInstance();
  stream << (unsigned char)(selfNode->parentNode() != OmWorld::instance()->root() &&
                            !OmVrmlNodeUtilities::isVisible(selfNode->parentField()));
  const QByteArray &s = selfNode->modelName().toUtf8();
  stream.writeRawData(s.constData(), s.size() + 1);
  const QByteArray &ba = selfNode->defName().toUtf8();
  stream.writeRawData(ba.constData(), ba.size() + 1);

  if (OmWorld::instance()->isVideoRecording()) {
    stream << (short unsigned int)0;
    stream << (unsigned char)C_SUPERVISOR_MOVIE_STATUS;
    stream << (unsigned char)WB_SUPERVISOR_MOVIE_RECORDING;
    delete mMovieStatus;
    mMovieStatus = NULL;
  }
}

void OmSupervisorUtilities::movieStatusChanged(int status) {
  mMovieStatus = new int[1];
  *mMovieStatus = status;
}

void OmSupervisorUtilities::animationStartStatusChanged(int status) {
  mAnimationStartStatus = new int[1];
  *mAnimationStartStatus = status;
}

void OmSupervisorUtilities::animationStopStatusChanged(int status) {
  mAnimationStopStatus = new int[1];
  *mAnimationStopStatus = status;
}

QStringList OmSupervisorUtilities::labelsState() const {
  QStringList labelsList;
  foreach (int labelId, mLabelIds) {
    const OmWrenLabelOverlay *labelOverlay = OmWrenLabelOverlay::retrieveById(labelId);
    if (labelOverlay)
      labelsList << createLabelUpdateString(labelOverlay);
  }
  return labelsList;
}

QString OmSupervisorUtilities::createLabelUpdateString(const OmWrenLabelOverlay *labelOverlay) const {
  assert(labelOverlay);
  double x, y;
  float alpha;
  int r, g, b;
  labelOverlay->position(x, y);
  labelOverlay->color(r, g, b, alpha);
  QString text = labelOverlay->text();
  return QString("\"id\":%1,\"font\":\"%2\",\"rgba\":\"%3,%4,%5,%6\",\"size\":%7,\"x\":%8,\"y\":%9,\"text\":\"%10\"")
    .arg(labelOverlay->id())
    .arg(labelOverlay->font())
    .arg(r)
    .arg(g)
    .arg(b)
    .arg(alpha)
    .arg(labelOverlay->size())
    .arg(x)
    .arg(y)
    .arg(text.replace("\n", "\\n"));
}

void OmSupervisorUtilities::simulationReset(bool restartControllers) {
  if (!restartControllers)  // If the controller is about to be restarted, there's no need to tell it to reset its own state
    mSimulationReset = true;
}
