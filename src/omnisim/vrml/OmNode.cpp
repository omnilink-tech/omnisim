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

#include "OmNode.hpp"

#include "OmField.hpp"
#include "OmFieldModel.hpp"
#include "OmLog.hpp"
#include "OmMFBool.hpp"
#include "OmMFColor.hpp"
#include "OmMFDouble.hpp"
#include "OmMFInt.hpp"
#include "OmMFNode.hpp"
#include "OmMFRotation.hpp"
#include "OmMFString.hpp"
#include "OmMFVector2.hpp"
#include "OmMFVector3.hpp"
#include "OmNetwork.hpp"
#include "OmNodeFactory.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeProtoInfo.hpp"
#include "OmNodeReader.hpp"
#include "OmParser.hpp"
#include "OmProject.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFNode.hpp"
#include "OmSFRotation.hpp"
#include "OmSFString.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"
#include "OmStandardPaths.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmUrl.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWriter.hpp"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>
#include <QtCore/QSet>
#include <QtCore/QUrl>

#include <cassert>

struct ProtoParameters {
  const QList<OmField *> *params;
};
static QList<ProtoParameters *> gProtoParameterList;
static QList<const OmNode *> gUrdfNodesQueue;
static QMap<int, QString> gUrdfNames;
static const OmNode *gUrdfCurrentNode;
static int gUrdfNameIndex = 0;

static bool gInstantiateMode = true;
static QList<OmNode *> gNodes = {NULL};  // id 0 is reserved for root node
static OmNode *gParent = NULL;
static QStringList *gInternalDefNamesInWrite = NULL;
static QList<std::pair<OmNode *, int>> *gExternalUseNodesInWrite = NULL;
static bool gRestoreUniqueIdOnClone;
static bool gDefCloneFlag = false;

bool OmNode::cUpdatingDictionary = false;

const QStringList OmNode::cHiddenParameterNames = QStringList() << "translation"
                                                                << "rotation"
                                                                << "linearVelocity"
                                                                << "angularVelocity"
                                                                << "position"
                                                                << "position2"
                                                                << "position3";

void OmNode::setInstantiateMode(bool mode) {
  gInstantiateMode = mode;
}

bool OmNode::instantiateMode() {
  return gInstantiateMode;
}

void OmNode::setGlobalParentNode(OmNode *parent, bool protoParameterNodeFlag) {
  gParent = parent;
}

OmNode *OmNode::globalParentNode() {
  return gParent;
}

bool OmNode::setRestoreUniqueIdOnClone(bool enable) {
  const bool previousValue = gRestoreUniqueIdOnClone;
  gRestoreUniqueIdOnClone = enable;
  return previousValue;
}

void OmNode::setUniqueId(int newId) {
  // check new id is valid
  // - correct range
  // - the newId slot is empty (the node was deleted)
  assert(newId >= 0);
  assert(newId < gNodes.size());
  assert(gRestoreUniqueIdOnClone || gNodes[newId] == NULL);

  // update the id
  if (mUniqueId != -1 && mUniqueId != newId) {
    if (mUniqueId == gNodes.size() - 1)
      gNodes.removeLast();
    else
      gNodes[mUniqueId] = NULL;
  }
  gNodes[newId] = this;
  mUniqueId = newId;
}

int OmNode::getFreeUniqueId() {
  int id = gNodes.size();
  gNodes.append(NULL);
  return id;
}

void OmNode::init() {
  // nodes at the .wbt level are created with a mUniqueId
  if (gInstantiateMode && !gRestoreUniqueIdOnClone) {
    mUniqueId = gNodes.size();
    gNodes.append(this);
  } else
    // nodes in .proto files are created with mUniqueId = -1
    mUniqueId = -1;

  mIsShallowNode = false;
  mDefNode = NULL;
  mHasUseAncestor = false;
  setParentNode(gParent);
  gParent = this;
  mIsBeingDeleted = false;
  mProtoParameterNode = NULL;
  mIsRedirectedToParameterNode = false;
  mIsNestedProtoNode = false;
  mProtoParameterNodeInstances.clear();
  mRegenerationRequired = false;
  mIsCreationCompleted = false;
  mInsertionCompleted = false;
  mProto = NULL;
  mCurrentStateId = "__init__";
  mProtoParameterParentNode = NULL;
  mIsProtoParameterNode = NULL;
}

// special constructor for shallow nodes, it's used by CadShape to instantiate PBRAppearances from an assimp material in
// order to configure the WREN materials. Shallow nodes are invisible but persistent, and due to their incompleteness should not
// be modified or interacted with in any other way other than through the creation and destruction of CadShape nodes
OmNode::OmNode(const QString &modelName) {
  mModel = OmNodeModel::findModel(modelName);
  init();
  mIsShallowNode = true;
  mUniqueId = -2;
}

OmNode::OmNode(const QString &modelName, const QString &worldPath, OmTokenizer *tokenizer) :
  mModel(OmNodeModel::findModel(modelName)) {
  init();

  // create fields from model
  foreach (OmFieldModel *const fieldModel, mModel->fieldModels())
    mFields.append(new OmField(fieldModel, this));

  if (tokenizer)
    readFields(tokenizer, worldPath);

  foreach (const OmField *const f, mFields)
    connect(f, &OmField::valueChanged, this, &OmNode::notifyFieldChanged);

  gParent = parentNode();
}

OmNode::OmNode(const OmNode &other) :
  mModel(other.mModel),
  mDefName(other.mDefName),
  mUseName(other.mUseName),
  mProtoInstanceTemplateContent(other.mProtoInstanceTemplateContent) {
  init();
  if (gRestoreUniqueIdOnClone)
    setUniqueId(other.mUniqueId);

  // copy mProto reference
  if (other.mProto) {
    mProto = other.mProto;
    mProto->ref();
    foreach (const OmNodeProtoInfo *protoInfo, other.mProtoParents)
      mProtoParents << new OmNodeProtoInfo(*protoInfo);
  }

  // do not redirect fields of DEF node descendant even if included in a PROTO parameter
  if (gDefCloneFlag || (parentNode() && parentNode()->mHasUseAncestor))
    mHasUseAncestor = true;

  // copy fields
  foreach (const OmField *f, other.fields()) {
    OmField *copy = new OmField(*f, this);
    mFields.append(copy);
    connect(copy, &OmField::valueChanged, this, &OmNode::notifyFieldChanged);
  }

  // copy parameters
  if (other.mProto) {
    foreach (const OmField *parameter, other.parameters()) {
      OmField *copy = new OmField(*parameter, this);
      mParameters.append(copy);
      connect(copy, &OmField::valueChanged, this, &OmNode::notifyParameterChanged);

      // Redirect field references in proto info
      foreach (OmNodeProtoInfo *protoInfo, mProtoParents)
        protoInfo->redirectFields(parameter, copy);
    }

    // connect fields to PROTO parameters
    foreach (OmField *parameter, mParameters)
      redirectAliasedFields(parameter, this);

    // copy internal PROTO parameters
    foreach (const OmField *parameter, other.mInternalProtoParameters) {
      OmField *copy = new OmField(*parameter, this);
      mInternalProtoParameters << copy;

      // No need to connect any of these. They can only be changed by regenerating the proto

      // Redirect field references in proto info
      foreach (OmNodeProtoInfo *protoInfo, mProtoParents)
        protoInfo->redirectFields(parameter, copy);
    }
  }

  gParent = parentNode();
}

OmNode::~OmNode() {
  mIsBeingDeleted = true;
  emit isBeingDestroyed(this);

  // Delete fields backwards to always delete USE nodes before DEF nodes
  int n = mFields.size() - 1;
  for (int i = n; i >= 0; --i)
    delete mFields[i];

  if (mProto) {
    // Delete parameters backwards to always delete USE nodes before DEF nodes
    n = mParameters.size() - 1;
    for (int i = n; i >= 0; --i)
      delete mParameters[i];
    n = mInternalProtoParameters.size() - 1;
    for (int i = n; i >= 0; --i)
      delete mInternalProtoParameters[i];
    foreach (OmNodeProtoInfo *protoInfo, mProtoParents)
      delete protoInfo;
    mProto->unref();
  }

  foreach (OmNode *instance, mProtoParameterNodeInstances) {
    assert(instance->mProtoParameterNode == this);
    if (instance->mProtoParameterNode == this)
      instance->mProtoParameterNode = NULL;
  }
  mIsRedirectedToParameterNode = false;
  mProtoParameterNodeInstances.clear();

  // nodes in PROTO definitions and in scene tree clipboard are not in gNodes[]
  // in case of PROTO regeneration, the unique ID could already be assigned to a new node
  if (mUniqueId != -1 && gNodes[mUniqueId] == this)
    gNodes[mUniqueId] = NULL;

  if (isUseNode() && mDefNode) {
    mDefNode->mUseNodes.removeOne(this);
    mDefNode->useNodesChanged();
  }

  if (isDefNode()) {
    foreach (OmNode *const useNode, mUseNodes)
      useNode->setDefNode(NULL);
  }

  if (mIsProtoParameterNode)
    delete[] mIsProtoParameterNode;
}

const QString &OmNode::modelName() const {
  return mProto ? mProto->name() : mModel->name();
}

const QString &OmNode::info() const {
  return mProto ? mProto->info() : mModel->info();
}

void OmNode::setDefName(const QString &defName, bool recurse) {
  if (defName == mDefName)
    return;

  foreach (OmNode *const useNode, mUseNodes)
    useNode->setUseName(defName);

  mDefName = defName;
  emit defUseNameChanged(this, false);

  if (!recurse)
    return;

  const OmNode *parent = parentNode();
  const OmNode *previousParentNode = this;
  QList<int> parentIndices;
  while (parent) {
    int index = subNodeIndex(previousParentNode, parent);
    if (index < 0)
      // parent-child link not correctly setup
      return;
    parentIndices.prepend(index);
    if (!parent->useNodes().isEmpty()) {
      const QList<OmNode *> &useList = parent->useNodes();
      foreach (OmNode *const useNode, useList) {
        OmNode *const node = findNodeFromSubNodeIndices(parentIndices, useNode);
        assert(node != NULL);
        node->setDefName(defName, false);
      }
    }
    previousParentNode = parent;
    parent = parent->parentNode();
  }
}

void OmNode::setUseName(const QString &useName, bool signal) {
  if (mUseName == useName)
    return;

  mUseName = useName;

  if (signal)
    emit defUseNameChanged(this, false);
}

QString OmNode::fullName() const {
  if (isUseNode())
    return "USE " + mUseName;
  if (!defName().isEmpty())
    return "DEF " + mDefName + " " + modelName();
  return modelName();
}

QString OmNode::computeName() const {
  if (isUseNode())
    return mUseName;
  if (!defName().isEmpty())
    return mDefName;
  const OmSFString *name = findSFString("name");
  return name ? "\"" + name->value() + "\"" : QString();
}

QString OmNode::usefulName() const {
  QString usefulName = fullName();

  if (isUseNode() || !defName().isEmpty())
    return usefulName;

  const OmSFString *name = findSFString("name");
  if (name)
    usefulName += " \"" + name->value() + "\"";
  else
    usefulName += " " + endPointName();
  return usefulName;
}

const QString &OmNode::nodeModelName() const {
  return mModel->name();
}

QString OmNode::fullPath(const QString &fieldName, QString &parameterName) const {
  if (mIsShallowNode)
    return "";

  const OmNode *n = this;
  const OmField *f = fieldName.isEmpty() ? NULL : findField(fieldName, true);

  if (f && f->parameter()) {
    // find visible parameter
    OmField *parameter = f->parameter();
    while (parameter->parameter())
      parameter = parameter->parameter();

    parameterName = parameter->name();
    n = parameter->parentNode();
    assert(n);
  } else
    parameterName = "";

  QString path;
  while (n && !n->isWorldRoot()) {
    if (n->protoParameterNode())
      n = n->protoParameterNode();

    if (path.isEmpty())
      path = n->usefulName();
    else
      path = n->usefulName() + " > " + path;

    n = n->parentNode();
  }

  return path;
}

QString OmNode::extractFieldName(const QString &message) {
  // extract field name
  QString fieldName;
  QRegularExpressionMatch match = QRegularExpression("'(\\w)+'").match(message);
  if (match.hasMatch()) {
    fieldName = match.captured();
    fieldName = fieldName.mid(1, fieldName.length() - 2);  // remove single quotes
  }
  return fieldName;
}

void OmNode::parsingWarn(const QString &message) const {
  warn(message, true);
}

void OmNode::parsingInfo(const QString &message) const {
  info(message, true);
}

void OmNode::warn(const QString &message, bool parsingMessage) const {
  const QString fieldName = extractFieldName(message);
  QString parameterName;
  const QString path = fullPath(fieldName, parameterName);
  QString improvedMsg = message;

  if (!fieldName.isEmpty() && !parameterName.isEmpty())
    // improve message by displaying parameter name instead of hidden field name
    improvedMsg.replace("'" + fieldName + "'", "'" + parameterName + "'");

  if (parsingMessage)
    OmLog::warning(path + ": " + improvedMsg, false, OmLog::PARSING);
  else
    OmLog::warning(path + ": " + improvedMsg);
}

void OmNode::info(const QString &message, bool parsingMessage) const {
  const QString fieldName = extractFieldName(message);
  QString parameterName;
  const QString path = fullPath(fieldName, parameterName);
  QString improvedMsg = message;

  if (!fieldName.isEmpty() && !parameterName.isEmpty())
    // improve message by displaying parameter name instead of hidden field name
    improvedMsg.replace("'" + fieldName + "'", "'" + parameterName + "'");

  if (parsingMessage)
    OmLog::info(path + ": " + improvedMsg, false, OmLog::PARSING);
  else
    OmLog::info(path + ": " + improvedMsg);
}

void OmNode::cleanup() {
  gNodes.clear();
  gNodes.append(NULL);  // id 0 is reserved for root node
}

OmNode *OmNode::findNode(int uniqueId) {
  if (uniqueId >= 0 && uniqueId < gNodes.size())
    return gNodes[uniqueId];

  return NULL;
}

OmField *OmNode::field(int index, bool internal) const {
  if (index < 0)
    return NULL;
  const QList<OmField *> &fieldsList = internal ? mFields : fieldsOrParameters();
  return index < fieldsList.size() ? fieldsList.at(index) : NULL;
}

OmField *OmNode::findField(const QString &fieldName, bool internal) const {
  const QList<OmField *> &list = internal ? mFields : fieldsOrParameters();

  foreach (OmField *const f, list)
    if (fieldName == f->name())
      return f;

  return NULL;
}

int OmNode::findFieldId(const QString &fieldName, bool internal) const {
  int counter = 0;
  const QList<OmField *> &list = internal ? mFields : fieldsOrParameters();
  foreach (const OmField *const f, list) {
    if (f->name() == fieldName)
      return counter;
    ++counter;
  }
  return -1; /* not found */
}

int OmNode::fieldIndex(const OmField *field) const {
  const QList<OmField *> &list = fieldsOrParameters();
  return list.indexOf(const_cast<OmField *>(field));
}

// For PROTOs
int OmNode::parameterIndex(const OmField *field) const {
  const QList<OmField *> &parameterList = parameters();
  return parameterList.indexOf(const_cast<OmField *>(field));
}

// Retrieves the field in which this node sits and returns the index of the node within this field
OmField *OmNode::parentFieldAndIndex(int &index, bool internal) const {
  index = -1;
  const OmNode *const parent = parentNode();
  if (!parent)
    return NULL;

  const QList<OmField *> &list = internal ? parent->fields() : parent->fieldsOrParameters();
  foreach (OmField *const f, list) {
    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(f->value());
    if (sfnode && sfnode->value() == this) {
      index = 0;
      return f;
    }
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(f->value());
    if (mfnode) {
      index = mfnode->nodeIndex(this);
      if (index != -1)
        return f;
    }
  }

  return NULL;
}

OmValue *OmNode::findValue(const QString &fieldName) const {
  foreach (const OmField *const f, mFields) {
    if (fieldName == f->name())
      return f->value();
  }
  return NULL;
}

OmSFString *OmNode::findSFString(const QString &fieldName) const {
  return dynamic_cast<OmSFString *>(findValue(fieldName));
}

OmSFInt *OmNode::findSFInt(const QString &fieldName) const {
  return dynamic_cast<OmSFInt *>(findValue(fieldName));
}

OmSFDouble *OmNode::findSFDouble(const QString &fieldName) const {
  return dynamic_cast<OmSFDouble *>(findValue(fieldName));
}

OmSFVector2 *OmNode::findSFVector2(const QString &fieldName) const {
  return dynamic_cast<OmSFVector2 *>(findValue(fieldName));
}

OmSFVector3 *OmNode::findSFVector3(const QString &fieldName) const {
  return dynamic_cast<OmSFVector3 *>(findValue(fieldName));
}

OmSFColor *OmNode::findSFColor(const QString &fieldName) const {
  return dynamic_cast<OmSFColor *>(findValue(fieldName));
}

OmSFNode *OmNode::findSFNode(const QString &fieldName) const {
  return dynamic_cast<OmSFNode *>(findValue(fieldName));
}

OmSFBool *OmNode::findSFBool(const QString &fieldName) const {
  return dynamic_cast<OmSFBool *>(findValue(fieldName));
}

OmSFRotation *OmNode::findSFRotation(const QString &fieldName) const {
  return dynamic_cast<OmSFRotation *>(findValue(fieldName));
}

OmMFString *OmNode::findMFString(const QString &fieldName) const {
  return dynamic_cast<OmMFString *>(findValue(fieldName));
}

OmMFInt *OmNode::findMFInt(const QString &fieldName) const {
  return dynamic_cast<OmMFInt *>(findValue(fieldName));
}

OmMFDouble *OmNode::findMFDouble(const QString &fieldName) const {
  return dynamic_cast<OmMFDouble *>(findValue(fieldName));
}

OmMFVector2 *OmNode::findMFVector2(const QString &fieldName) const {
  return dynamic_cast<OmMFVector2 *>(findValue(fieldName));
}

OmMFVector3 *OmNode::findMFVector3(const QString &fieldName) const {
  return dynamic_cast<OmMFVector3 *>(findValue(fieldName));
}

OmMFBool *OmNode::findMFBool(const QString &fieldName) const {
  return dynamic_cast<OmMFBool *>(findValue(fieldName));
}

OmMFRotation *OmNode::findMFRotation(const QString &fieldName) const {
  return dynamic_cast<OmMFRotation *>(findValue(fieldName));
}

OmMFColor *OmNode::findMFColor(const QString &fieldName) const {
  return dynamic_cast<OmMFColor *>(findValue(fieldName));
}

OmMFNode *OmNode::findMFNode(const QString &fieldName) const {
  return dynamic_cast<OmMFNode *>(findValue(fieldName));
}

void OmNode::makeUseNode(OmNode *defNode, bool reading) {
  assert(defNode && defNode->isDefNode());

  mDefName = "";  // A USE node cannot be a DEF node at the same time
  mUseName = defNode->defName();

  if (reading)
    return;

  if (mDefNode) {
    mDefNode->mUseNodes.removeOne(this);  // Unsubscribes from updates of the previous referred DEF node
    mDefNode->useNodesChanged();
  }

  mDefNode = defNode;

  if (!mUseNodes.empty()) {
    mUseNodes.clear();
    useNodesChanged();
  }

  mDefNode->mUseNodes.append(this);  // Subscribes to updates of the referred DEF node
  mDefNode->useNodesChanged();
}

// Turns an orphan USE node into a DEF node with the same name
void OmNode::makeDefNode() {
  if (mDefNode) {  // Unsubscribes from previous DEF node
    mDefNode->mUseNodes.removeOne(this);
    mDefNode->useNodesChanged();
  }
  mDefName = mUseName;
  mUseName = "";
  mDefNode = NULL;

  if (parentNode() && parentNode()->mHasUseAncestor)
    return;

  // reset mHasUseAncestor flag in descendant nodes
  resetUseAncestorFlag();

  emit defUseNameChanged(this, true);
}

void OmNode::resetUseAncestorFlag() {
  if (!mHasUseAncestor || mDefNode)
    return;

  mHasUseAncestor = false;
  QList<OmNode *> subNodeList = subNodes(false);
  foreach (OmNode *const n, subNodeList)
    n->resetUseAncestorFlag();
}

// called after any field of this node has changed
void OmNode::notifyFieldChanged() {
  // this is the changed field
  OmField *const f = static_cast<OmField *>(sender());

  OmField *const pf = this->parentField();
  if (pf && pf->parameter() && isProtoParameterNode())
    emit pf->parentNode()->parameterChanged(pf);

  if (mIsBeingDeleted || cUpdatingDictionary) {
    emit fieldChanged(f);
    return;
  }

  // see if this node or any of its ancestors is a DEF node
  // in which case we must propagate the change to all matching USE nodes
  // note that there can be several DEF ancestors, and each must be treated
  OmNode *n = this;
  do {
    // is n a DEF node with USE nodes ?
    if (!n->mUseNodes.isEmpty()) {
      // find where the changed field is located in the DEF node
      int index = n->findSubFieldIndex(f);
      if (index >= 0) {
        OmNode *parent = NULL;
        // apply changes to the same field in each USE node
        foreach (const OmNode *const useNode, n->mUseNodes) {
          OmField *const subField = useNode->findSubField(index, parent);
          if (!subField || subField->type() != f->type() || subField->name() != f->name())
            continue;
          assert(parent);
          setGlobalParentNode(parent);
          const bool isTemplateRegenerationRequired =
            OmVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(subField, subField->parentNode());
          subField->copyValueFrom(f);
          if (!isTemplateRegenerationRequired)
            subField->defHasChanged();
        }

        setGlobalParentNode(NULL);
      } else
        // index could be invalid for fields in sub-PROTO scope
        // but in this case the USE node will be updated by the parameter change notification
        break;
    }

    // climb up the family tree but stop if child insertion is not completed
    OmNode *p = n->parentNode();
    if (!p || !n->mInsertionCompleted)
      break;
    n = p;
  } while (!n->isWorldRoot());

  emit fieldChanged(f);
}

void OmNode::notifyParameterChanged() {
  OmField *const parameter = static_cast<OmField *>(sender());

  emit parameterChanged(parameter);
}

int OmNode::findSubFieldIndex(const OmField *const searched) const {
  int count = 0;
  QList<OmNode *> list(subNodes(true, true, false));
  list.prepend(const_cast<OmNode *>(this));
  foreach (const OmNode *const node, list) {
    foreach (const OmField *const f, node->mFields) {
      if (f == searched)
        return count;
      ++count;
    }
  }
  return -1;
}

OmField *OmNode::findSubField(int index, OmNode *&parent) const {
  int count = 0;
  QList<OmNode *> list(subNodes(true, true, false));
  list.prepend(const_cast<OmNode *>(this));
  foreach (OmNode *const node, list) {
    foreach (OmField *const f, node->mFields) {
      if (count == index) {
        parent = node;
        return f;
      }
      ++count;
    }
  }
  return NULL;
}

void OmNode::validate(const OmNode *upperNode, const OmField *upperField, bool isInBoundingObject) const {
  if (isUseNode())
    return;
  const OmNode *parent = upperNode == NULL ? this : upperNode;
  const QList<OmField *> fieldsToBeValidated =
    upperField ? (QList<OmField *>() << const_cast<OmField *>(upperField)) : fields();

  foreach (const OmField *f, fieldsToBeValidated) {
    OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(f->value());
    OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(f->value());
    if (sfnode) {
      if (f->name() == "boundingObject")
        isInBoundingObject = true;
      const OmNode *child = sfnode->value();
      if (child) {
        QString errorMessage;
        if (!OmNodeFactory::instance()->validateExistingChildNode(f, child, parent, isInBoundingObject, errorMessage)) {
          bool emptyErrorMessage = errorMessage.isEmpty();
          if (emptyErrorMessage)
            errorMessage = QString("Skipped unexpected %1 node in '%2' field of %3 node.")
                             .arg(child->nodeModelName(), f->name(), this->nodeModelName());
          else if (upperNode)
            errorMessage.append(tr(" Using 'Slot' nodes mechanism."));

          OmField *parameter = f->parameter();
          if (parameter) {
            OmSFNode *sfParameter = dynamic_cast<OmSFNode *>(parameter->value());
            assert(sfParameter);
            sfParameter->setValue(NULL);
            if (!emptyErrorMessage)
              errorMessage.append(tr(" Skipped node in PROTO parameter '%1'.").arg(parameter->name()));
          } else {
            sfnode->setValue(NULL);
            if (!emptyErrorMessage)
              errorMessage.prepend(tr(" Skipped node: "));
          }

          parsingWarn(errorMessage);
        } else
          child->validate(NULL, NULL, isInBoundingObject);
      }
    } else if (mfnode) {
      for (int i = 0; i < mfnode->size(); ++i) {
        const OmNode *child = mfnode->item(i);
        QString errorMessage;
        if (!OmNodeFactory::instance()->validateExistingChildNode(f, child, parent, isInBoundingObject, errorMessage)) {
          bool emptyErrorMessage = errorMessage.isEmpty();
          if (emptyErrorMessage)
            errorMessage = QString("Skipped unexpected %1 node in '%2' field of %3 node.")
                             .arg(child->nodeModelName(), f->name(), this->nodeModelName());
          else if (upperNode)
            errorMessage.append(tr(" Using 'Slot' nodes mechanism."));

          OmField *parameter = f->parameter();
          if (parameter) {
            OmMFNode *mfParameter = dynamic_cast<OmMFNode *>(parameter->value());
            assert(mfParameter);
            mfParameter->removeItem(i);
            if (!emptyErrorMessage)
              errorMessage.append(tr(" Skipped node in PROTO parameter '%1'.").arg(parameter->name()));
          } else {
            mfnode->removeItem(i);
            if (!emptyErrorMessage)
              errorMessage.prepend(tr("Skipped node: "));
          }

          --i;
          parsingWarn(errorMessage);
        } else
          child->validate(NULL, NULL, isInBoundingObject);
      }
    }
  }
}

void OmNode::readFieldValue(OmField *field, OmTokenizer *tokenizer, const QString &worldPath) const {
  if (field->name() == "boundingObject")
    OmNodeReader::current()->setReadingBoundingObject(true);

  field->readValue(tokenizer, worldPath);

  if (field->name() == "boundingObject")
    OmNodeReader::current()->setReadingBoundingObject(false);
}

void OmNode::readFields(OmTokenizer *tokenizer, const QString &worldPath) {
  gParent = this;
  tokenizer->skipToken("{");

  while (tokenizer->peekWord() != "}") {
    const QString &w(tokenizer->nextWord());
    OmField *const f = findField(w);
    if (!f)
      tokenizer->skipField();
    else {
      const QString &referral = tokenizer->referralFile().isEmpty() ? tokenizer->fileName() : tokenizer->referralFile();
      f->setScope(referral);

      if (tokenizer->peekWord() == "IS") {
        tokenizer->skipToken("IS");
        const QString &alias = tokenizer->nextWord();
        bool exists = false;
        foreach (const OmField *p, *(gProtoParameterList.last()->params)) {
          if (p->name() == alias) {
            exists = true;
            break;
          }
        }
        if (exists) {
          f->setAlias(alias);
          copyAliasValue(f, alias);
        } else
          parsingWarn(tr("Field IS reference '%1' has no matching PROTO parameter.").arg(alias));
      } else
        readFieldValue(f, tokenizer, worldPath);
    }
  }

  tokenizer->skipToken("}");
  gParent = parentNode();
}

void OmNode::enableDefNodeTrackInWrite(bool substituteInStream) {
  assert(gInternalDefNamesInWrite == NULL && gExternalUseNodesInWrite == NULL);
  gInternalDefNamesInWrite = new QStringList();
  if (!substituteInStream)
    gExternalUseNodesInWrite = new QList<std::pair<OmNode *, int>>();
}

void OmNode::disableDefNodeTrackInWrite() {
  delete gInternalDefNamesInWrite;
  delete gExternalUseNodesInWrite;
  gInternalDefNamesInWrite = NULL;
  gExternalUseNodesInWrite = NULL;
}

QList<std::pair<OmNode *, int>> *OmNode::externalUseNodesPositionsInWrite() {
  return gExternalUseNodesInWrite;
}

void OmNode::writeParameters(OmWriter &writer) const {
  foreach (const OmField *parameter, parameters())
    parameter->write(writer);
}

bool OmNode::isUrdfRootLink() const {
  return findSFString("name") ? true : false;
}

const OmNode *OmNode::findUrdfLinkRoot() const {
  const OmNode *parentRoot = parentNode();
  while (parentRoot && !parentRoot->isUrdfRootLink())
    parentRoot = parentRoot->parentNode();

  return parentRoot;
}

void OmNode::write(OmWriter &writer) const {
  if (writer.isUrdf()) {
    // Start naming from scratch
    if (isRobot()) {
      gUrdfNames.clear();
      gUrdfNameIndex = 0;
    }

    if (gUrdfCurrentNode != this && isJoint() && !gUrdfNodesQueue.contains(this)) {
      gUrdfNodesQueue.append(this);
      return;
    }
    if (!gUrdfCurrentNode)
      gUrdfCurrentNode = this;

    writeExport(writer);

    if (gUrdfCurrentNode == this) {
      if (gUrdfNodesQueue.size() > 0) {
        gUrdfCurrentNode = gUrdfNodesQueue.takeLast();
        gUrdfCurrentNode->write(writer);
      } else
        gUrdfCurrentNode = NULL;
    }
    return;
  }
  if (writer.isW3d() || (writer.isProto() && (!writer.rootNode() || this == writer.rootNode() ||
                                              OmVrmlNodeUtilities::findContainingProto(this) ==
                                                OmVrmlNodeUtilities::findContainingProto(writer.rootNode())))) {
    writeExport(writer);
    return;
  }

  if (gInternalDefNamesInWrite) {
    // handle DEF nodes defined outside the current export node in order to export a self-describing node
    if (isUseNode() && !gInternalDefNamesInWrite->contains(mUseName)) {
      if (gExternalUseNodesInWrite)
        // keep track of DEF node
        gExternalUseNodesInWrite->append(std::pair<OmNode *, int>(mDefNode, writer.string()->size()));
      else {
        // write definition directly on the stream
        mDefNode->write(writer);
        return;
      }
      gInternalDefNamesInWrite->append(mUseName);
    } else if (isDefNode())
      gInternalDefNamesInWrite->append(mDefName);
  }

  writer << fullName();

  if (isUseNode())
    return;

  writer << " {\n";

  writer.increaseIndent();

  if (isProtoInstance())
    writeParameters(writer);
  else
    foreach (const OmField *f, fields())
      if (!f->isDeprecated())
        f->write(writer);

  writer.decreaseIndent();
  writer.indent();
  writer << "}";
}

// This function lists only the texture files which are explicitly referred to in
// this world file and not the one implicitly referred to by included PROTO files.
// This list may contain duplicate texture files.
QList<std::pair<QString, OmMFString *>> OmNode::listTextureFiles() const {
  QList<std::pair<QString, OmMFString *>> list;
  const bool imageTexture = model()->name() == "ImageTexture";
  const QString currentTexturePath = OmProject::current()->worldsPath();
  foreach (OmField *f, fields())
    if (f->value()->type() == WB_SF_NODE) {
      const OmSFNode *node = dynamic_cast<OmSFNode *>(f->value());
      if (node->value())
        list << node->value()->listTextureFiles();
    } else if (f->value()->type() == WB_MF_NODE) {
      const OmMFNode *mfnode = dynamic_cast<OmMFNode *>(f->value());
      OmMFNode::Iterator it(*mfnode);
      while (it.hasNext()) {
        const OmNode *n = static_cast<OmNode *>(it.next());
        list << n->listTextureFiles();
      }
    } else if (imageTexture && f->value()->type() == WB_MF_STRING && f->name() == "url") {
      const OmNode *p = protoAncestor();
      QString protoPath;
      if (p)
        protoPath = p->proto()->path();
      OmMFString *mfstring = dynamic_cast<OmMFString *>(f->value());
      for (int i = 0; i < mfstring->size(); i++) {
        const QString &textureFile = mfstring->item(i);
        if (p && QFile::exists(protoPath + textureFile))  // PROTO texture
          continue;                                       // skip it
        if (QFile::exists(currentTexturePath + textureFile))
          list << std::pair<QString, OmMFString *>(textureFile, mfstring);
      }
    }
  return list;
}

const OmNode *OmNode::containingProto(bool skipThis) const {
  const OmNode *n = this;
  while (n) {
    const OmProtoModel *protoModel = n->proto();
    if (protoModel && (!skipThis || n != this))
      return n;
    else {
      const OmNode *ppn = n->protoParameterNode();
      if (ppn && ppn->proto() && (!skipThis || ppn->proto() != proto()))
        return ppn;

      n = n->parentNode();
    }
  }

  return NULL;
}

const QString OmNode::urdfName() const {
  // Use existing name if already given
  if (gUrdfNames.contains(uniqueId()))
    return gUrdfNames[uniqueId()];

  // Name the link/joint according to priority: name -> def -> model
  QString name;
  if (this->findSFString("name") && this->findSFString("name")->value() != "")
    name = this->findSFString("name")->value();
  else if (this->defName() != "")
    name = this->defName();
  else
    name = QString(mModel->name().toLower());
  QString fullUrdfName = getUrdfPrefix() + name;

  // Add suffix if needed
  if (gUrdfNames.values().contains(fullUrdfName))
    fullUrdfName += "_" + QString::number(gUrdfNameIndex++);

  // Return
  gUrdfNames[uniqueId()] = fullUrdfName;
  return fullUrdfName;
}

bool OmNode::exportNodeHeader(OmWriter &writer) const {
  if (writer.isW3d())  // actual export is done in OmBaseNode
    return false;
  else if (writer.isUrdf()) {
    if (gUrdfCurrentNode == this) {
      writer.increaseIndent();
      writer.indent();
      writer << "<link name=\"" + urdfName() + "\">\n";
      return false;
    } else if (isUrdfRootLink()) {
      gUrdfNodesQueue.append(this);
      return true;
    }
    return false;
  }
  if (isUseNode()) {
    writer << "USE " << mUseName << "\n";
    return true;
  }

  if (isDefNode())
    writer << "DEF " << defName() << " ";
  writer << nodeModelName();

  writer << " {\n";
  writer.increaseIndent();
  return false;
}

void OmNode::exportNodeFields(OmWriter &writer) const {
  if (writer.isUrdf())
    return;

  foreach (const OmField *f, fields()) {
    if (!f->isDeprecated() && ((f->isW3d() || writer.isProto()) && f->singleType() != WB_SF_NODE) &&
        !customExportedFields().contains(f->name()))
      f->write(writer);
  }
}

void OmNode::exportNodeSubNodes(OmWriter &writer) const {
  foreach (const OmField *f, fields()) {
    if (!f->isDeprecated() && ((f->isW3d() || writer.isProto() || writer.isUrdf()) && f->singleType() == WB_SF_NODE)) {
      const OmSFNode *const node = dynamic_cast<OmSFNode *>(f->value());
      if (node == NULL || node->value() == NULL || node->value()->shallExport() || writer.isProto() || writer.isUrdf()) {
        if (writer.isW3d() || writer.isUrdf())
          f->value()->write(writer);
        else
          f->write(writer);
      }
    }
  }
}

void OmNode::exportNodeFooter(OmWriter &writer) const {
  if (writer.isW3d())
    writer << "</" << w3dName() << ">";
  else if (writer.isUrdf()) {
    if (gUrdfCurrentNode == this) {
      writer.indent();
      writer << "</link>\n";
      writer.decreaseIndent();
    }
  } else {  // VRML
    writer.decreaseIndent();
    writer.indent();
    writer << "}";
  }
}

void OmNode::exportNodeContents(OmWriter &writer) const {
  if (writer.isProto() && isRobot())
    fixMissingResources();

  exportNodeFields(writer);
  if (writer.isW3d())
    writer << ">";
  exportNodeSubNodes(writer);
}

void OmNode::exportExternalSubProto(OmWriter &writer) const {
  if (!isProtoInstance())
    return;

  // find all proto that were already exposed prior to converting the root (typically, slots with world visibility)
  const QList<const OmNode *> protos = OmVrmlNodeUtilities::protoNodesInWorldFile(this);
  foreach (const OmNode *p, protos) {
    // the node itself doesn't need to be re-declared since it won't exist after conversion
    if (p != this) {
      const QString protoDeclaration = OmProtoManager::instance()->externProtoUrl(p, false);
      assert(!protoDeclaration.isEmpty());  // since the proto has world-visibility, a declaration for it must exist
      writer.trackDeclaration(p->modelName(), protoDeclaration);
    }
  }

  addExternProtoFromFile(mProto, writer);
}

void OmNode::addExternProtoFromFile(const OmProtoModel *proto, OmWriter &writer) const {
  const QString path = (OmUrl::isWeb(proto->url()) && OmNetwork::instance()->isCachedWithMapUpdate(proto->url())) ?
                         OmNetwork::instance()->get(proto->url()) :
                         proto->url();

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    parsingWarn(tr("File '%1' is not readable.").arg(path));
    return;
  }

  QString ancestorName;
  if (proto->isDerived())
    ancestorName = proto->ancestorProtoName();

  // check if the root file references external PROTO
  const QRegularExpression re("^\\s*EXTERNPROTO\\s+\"(.*\\.proto)\"", QRegularExpression::MultilineOption);
  QRegularExpressionMatchIterator it = re.globalMatch(file.readAll());

  // begin by populating the list of all sub-PROTO
  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    if (match.hasMatch()) {
      const QString subProto = match.captured(1);

      const QString subProtoUrl = OmUrl::combinePaths(subProto, path);
      if (subProtoUrl.isEmpty())
        continue;

      if (!subProtoUrl.endsWith(".proto", Qt::CaseInsensitive)) {
        parsingWarn(tr("Malformed EXTERNPROTO URL. The URL should end with '.proto'."));
        continue;
      }

      // sanity check (must either be: relative, absolute, starts with omnisim://, starts with https://)
      if (!subProtoUrl.startsWith("https://") && !subProtoUrl.startsWith("omnisim://") && !QFileInfo(subProtoUrl).isRelative() &&
          !QFileInfo(subProtoUrl).isAbsolute()) {
        parsingWarn(tr("Malformed EXTERNPROTO URL. Invalid URL provided: %1.").arg(subProtoUrl));
        continue;
      }

      // ensure there's no ambiguity between the declarations
      const QString subProtoName = QUrl(subProtoUrl).fileName().replace(".proto", "", Qt::CaseInsensitive);
      writer.trackDeclaration(subProtoName, subProtoUrl);
      if (!ancestorName.isEmpty() && ancestorName == subProtoName)
        addExternProtoFromFile(OmProtoManager::instance()->findModel(proto->ancestorProtoName(), "", proto->diskPath()),
                               writer);
    }
  }
}

void OmNode::writeExport(OmWriter &writer) const {
  if (!mIsProtoParameterNode)
    isProtoParameterNode();
  assert(!(writer.isW3d() && mIsProtoParameterNode[0]));
  if (exportNodeHeader(writer))
    return;
  if (writer.isUrdf()) {
    exportNodeSubNodes(writer);
    exportNodeFooter(writer);
    if (isUrdfRootLink() && nodeModelName() != "Robot")
      exportUrdfJoint(writer);
  } else {
    if (writer.isProto() && this == writer.rootNode())
      exportExternalSubProto(writer);
    exportNodeContents(writer);
    exportNodeFooter(writer);
  }
}

QString OmNode::exportResource(const QString &rawURL, const QString &resolvedURL, const QString &relativeResourcePath,
                               const OmWriter &writer) const {
  if (OmUrl::isLocalUrl(resolvedURL))
    return OmUrl::computeLocalAssetUrl(resolvedURL, writer.isW3d());
  else if (OmUrl::isWeb(resolvedURL))
    return resolvedURL;
  else {
    if (writer.isWritingToFile())
      return OmUrl::exportResource(this, rawURL, resolvedURL, relativeResourcePath, writer);
    else
      return OmUrl::expressRelativeToWorld(resolvedURL);
  }
}

void OmNode::exportMFResourceField(const QString &fieldName, const OmMFString *value, const QString &relativeResourcePath,
                                   OmWriter &writer) const {
  const OmField *originalField = findField(fieldName, true);
  assert(originalField && originalField->type() == WB_MF_STRING);

  // only w3c and proto exports need urls to be resolved
  if (!(writer.isW3d() || writer.isProto())) {
    originalField->write(writer);
    return;
  }

  if (value->size() == 0)
    return;

  OmField copiedField(*originalField);
  OmMFString *newValue = dynamic_cast<OmMFString *>(copiedField.value());

  for (int i = 0; i < value->size(); ++i) {
    const QString &rawURL = value->item(i);
    const QString &resolvedURL = OmUrl::computePath(this, fieldName, rawURL);
    newValue->setItem(i, exportResource(rawURL, resolvedURL, relativeResourcePath, writer));
  }

  copiedField.write(writer);
}

void OmNode::exportSFResourceField(const QString &fieldName, const OmSFString *value, const QString &relativeResourcePath,
                                   OmWriter &writer) const {
  const OmField *originalField = findField(fieldName, true);
  assert(originalField && originalField->type() == WB_SF_STRING);

  // only w3c and proto exports need urls to be resolved
  if (!(writer.isW3d() || writer.isProto())) {
    originalField->write(writer);
    return;
  }

  const QString &rawURL = value->value();
  if (rawURL.isEmpty())
    return;

  OmField copiedField(*originalField);
  OmSFString *newValue = dynamic_cast<OmSFString *>(copiedField.value());

  const QString &resolvedURL = OmUrl::computePath(this, fieldName, rawURL);
  newValue->setValue(exportResource(rawURL, resolvedURL, relativeResourcePath, writer));

  copiedField.write(writer);
}

bool OmNode::operator==(const OmNode &other) const {
  if (mModel != other.mModel || isProtoInstance() != other.isProtoInstance() ||
      (mProto && mProto->url() != other.mProto->url()) || mDefName != other.mDefName)
    return false;

  if (this == &other)
    return true;

  const QList<OmField *> fieldsList = fieldsOrParameters();
  const QList<OmField *> otherFieldsList = other.fieldsOrParameters();
  const int size = fieldsList.size();

  assert(size == otherFieldsList.size());

  for (int i = 0; i < size; ++i) {
    const OmField *const f1 = fieldsList[i];
    const OmField *const f2 = otherFieldsList[i];
    if (!(f1->isDeprecated() || f1->value()->equals(f2->value())))
      return false;
  }

  return true;
}

bool OmNode::isDefault() const {
  QList<OmField *> fieldsList = fieldsOrParameters();
  foreach (const OmField *f, fieldsList) {
    if (!(f->isDeprecated() || f->isDefault()))
      return false;
  }

  return true;
}

// recursively search for matching IS fields/parameters and redirect them to the PROTO parameter
// the search does not look up inside fields of other PROTO instances or PROTO parameter node instances
// because the scope of a PROTO parameter must be local to a PROTO instance
// but, when loading from file, it looks up in the parameters of direct PROTO instances in order to pass a parameter to a sub
// PROTO
void OmNode::redirectAliasedFields(OmField *param, OmNode *protoInstance, bool searchInParameters, bool copyValueOnly) {
  QList<OmField *> fieldsList;
  if (searchInParameters) {
    // search for matching IS fields in subPROTO declaration
    // and redirect them to the upper PROTO parameter
    if (this != protoInstance && isProtoInstance())
      fieldsList = mParameters;
  } else
    fieldsList = mFields;

  // always assign new unique id when copying due to field redirection
  const bool restoreUniqueId = gRestoreUniqueIdOnClone;
  gRestoreUniqueIdOnClone = false;

  // search self
  foreach (OmField *f, fieldsList) {
    if (f->alias() == param->name() && f->type() == param->type()) {
      f->setScope(param->scope());

      // set parent node
      OmNode *tmpParent = gParent;
      gParent = this;

      if (copyValueOnly) {
        f->copyValueFrom(param);
        // reset alias value so that the value is copied when node is cloned
        // this is needed for derived PROTO nodes linked to a default base PROTO parameter
        f->setAlias(QString());
      } else
        f->redirectTo(param);
      gParent = tmpParent;
    }
  }

  // do not search in sub nodes of a sub PROTO node
  foreach (OmNode *node, subNodes(false, !searchInParameters, searchInParameters)) {
    if (node->isProtoInstance()) {
      // search also in parameters of direct sub PROTO nodes
      node->redirectAliasedFields(param, protoInstance, true, copyValueOnly);
    } else {
      // do not search in fields of PROTO parameter node instances
      const OmNode *ppn = node->protoParameterNode();
      if (!ppn || !ppn->isProtoInstance())
        node->redirectAliasedFields(param, protoInstance, false, copyValueOnly);
    }
  }

  // restore flag
  gRestoreUniqueIdOnClone = restoreUniqueId;
}

OmNode *OmNode::cloneDefNode() {
  gDefCloneFlag = true;
  OmNode *copy = clone();
  gDefCloneFlag = false;
  return copy;
}

OmNode *OmNode::cloneAndReferenceProtoInstance() {
  OmNode *copy = clone();

  if (copy && !copy->mHasUseAncestor) {
    // associate instance with respective PROTO parameter node
    // DEF/USE nodes should not be associated
    copy->setProtoParameterNode(this);
  }

  return copy;
}

bool OmNode::setProtoParameterNode(OmNode *node) {
  if (mProtoParameterNode == node)
    return false;  // nothing to do

  if (mProtoParameterNode) {
    // remove connections to other node
    disconnect(this, &QObject::destroyed, mProtoParameterNode, &OmNode::removeProtoParameterNodeInstance);
    mProtoParameterNode->removeProtoParameterNodeInstance(this);
  }

  mProtoParameterNode = node;
  mIsRedirectedToParameterNode = mProtoParameterNode == NULL;
  if (mProtoParameterNode) {
    mProtoParameterNode->mProtoParameterNodeInstances.append(this);
    connect(this, &QObject::destroyed, mProtoParameterNode, &OmNode::removeProtoParameterNodeInstance);
  }
  return true;
}

void OmNode::finalizeProtoParametersRedirection() {
  if (isProtoInstance())
    redirectInternalFields(NULL, true);

  const QList<OmNode *> nodeInstances = protoParameterNodeInstances();
  if (!nodeInstances.isEmpty()) {
    foreach (OmNode *n, nodeInstances)
      redirectInternalFields(n, this, true);
    return;
  }

  const QList<OmNode *> nodes = subNodes(false, true, true);
  foreach (OmNode *n, nodes)
    n->finalizeProtoParametersRedirection();
}

void OmNode::redirectInternalFields(OmField *param, bool finalize) {
  if (mIsRedirectedToParameterNode)
    return;
  mIsRedirectedToParameterNode = true;
  const QList<OmField *> parametersList = param ? QList<OmField *>() << param : mParameters;
  foreach (OmField *parameter, parametersList) {
    QList<OmField *> internalFields = parameter->internalFields();
    while (!internalFields.isEmpty()) {
      OmField *f = internalFields.takeFirst();
      redirectInternalFields(f, parameter, finalize);
      internalFields << f->internalFields();
    }
  }
}

void OmNode::redirectInternalFields(OmField *field, OmField *parameter, bool finalize) {
  switch (field->type()) {
    case WB_MF_NODE: {
      OmMFNode *mfnode = static_cast<OmMFNode *>(field->value());
      assert(mfnode);
      for (int i = 0; i < mfnode->size(); i++) {
        OmNode *subnode = mfnode->item(i);
        OmNode *parameterNode = static_cast<OmMFNode *>(parameter->value())->item(i);
        if (subnode->setProtoParameterNode(parameterNode) || finalize)
          redirectInternalFields(subnode, parameterNode, finalize);
      }
      break;
    }
    case WB_SF_NODE: {
      OmSFNode *sfnode = static_cast<OmSFNode *>(field->value());
      assert(sfnode);
      OmNode *subnode = sfnode->value();
      if (subnode) {
        OmNode *parameterNode = static_cast<OmSFNode *>(parameter->value())->value();
        if (subnode->setProtoParameterNode(parameterNode) || finalize)
          redirectInternalFields(subnode, parameterNode, finalize);
      }
      break;
    }
    default:
      break;
  }
}

void OmNode::redirectInternalFields(OmNode *instance, OmNode *parameterNode, bool finalize) {
  if (instance && instance->mIsRedirectedToParameterNode)
    return;
  if (instance)
    instance->mIsRedirectedToParameterNode = true;
  assert(instance && parameterNode);

  QListIterator<OmField *> referenceIt(parameterNode->fieldsOrParameters());
  QListIterator<OmField *> instanceIt(instance->fieldsOrParameters());
  while (referenceIt.hasNext() && instanceIt.hasNext()) {
    OmField *param = referenceIt.next();
    OmField *instanceParam = instanceIt.next();
    if (instanceParam->name() != param->name() || instanceParam->type() != param->type())
      continue;
    instanceParam->redirectTo(param, true);  // redirect without copying value
    redirectInternalFields(instanceParam, param, finalize);
  }
}

void OmNode::removeProtoParameterNodeInstance(QObject *node) {
  QMutableVectorIterator<OmNode *> it(mProtoParameterNodeInstances);
  while (it.hasNext()) {
    if (it.next() == static_cast<OmNode *>(node))
      it.remove();
  }
}

OmNode *OmNode::clone() const {
  // if clone() is called while reading a .proto file we create a lightweight node
  // to be placed in the PROTO model
  if (!gInstantiateMode)
    return new OmNode(*this);

  // otherwise we need to instantiate the node for a PROTO instance
  OmNode *const copy = OmNodeFactory::instance()->createCopy(*this);
  if (!copy)
    parsingWarn(tr("Could not instantiate '%1' node: this class is not yet implemented in OmniSim.").arg(model()->name()));

  return copy;
}

void OmNode::copyAliasValue(OmField *field, const QString &alias) {
  // instantiate if possible the alias parameter from the parent PROTO.
  // this avoids double setup of the parameter and is particularly
  // helpful for nested PROTOs
  if (!gProtoParameterList.isEmpty()) {
    foreach (const OmField *p, *(gProtoParameterList.last()->params)) {
      if (p->name() == alias && p->type() == field->type())
        field->copyValueFrom(p);
    }
  }
}

OmNode *OmNode::createProtoInstance(OmProtoModel *proto, OmTokenizer *tokenizer, const QString &worldPath) {
  // 1. get PROTO's field models
  const QList<OmFieldModel *> &protoFieldModels = proto->fieldModels();

  // 2. create the parameters list from the model (default values)
  QList<OmField *> parametersList;
  QList<QMap<QString, OmNode *>> parametersDefMap;
  bool hasDefaultDefNodes = false;
  QListIterator<OmFieldModel *> fieldModelsIt(protoFieldModels);
  while (fieldModelsIt.hasNext()) {
    OmField *defaultParameter = new OmField(fieldModelsIt.next(), NULL);
    defaultParameter->setScope(proto->url());
    parametersList.append(defaultParameter);

    parametersDefMap.append(QMap<QString, OmNode *>());
    if (tokenizer && OmNodeReader::current()) {
      // extract DEF nodes defined in default PROTO parameter
      const QList<OmNode *> defNodes = subNodes(defaultParameter, true, false, false);
      QListIterator<OmNode *> defNodesIt(defNodes);
      while (defNodesIt.hasNext()) {
        OmNode *node = defNodesIt.next();
        if (!node->defName().isEmpty())
          parametersDefMap.last().insert(node->defName(), node);
      }
      hasDefaultDefNodes = hasDefaultDefNodes || !parametersDefMap.last().isEmpty();
    }
  }

  // 3. populate the parameters from the tokenizer if existing
  QSet<QString> parameterNames;
  if (tokenizer) {
    tokenizer->skipToken("{");

    int nextParameterIndex = 0;
    int currentParameterIndex = 0;
    bool fieldOrderWarning = true;
    while (tokenizer->peekWord() != "}") {
      QString parameterName = tokenizer->nextWord();
      const OmFieldModel *parameterModel = NULL;
      const bool hidden = parameterName == "hidden";
      if (hidden) {
        static const QRegularExpression rx1("(_\\d+)+$");  // looks for a substring of the form _7 or _13_1 at the end of
                                                           // the parameter name, e.g. as in rotation_7, position2_13_1
        const QString &hiddenParameterName(tokenizer->peekWord());
        const QRegularExpressionMatch match1 = rx1.match(hiddenParameterName);
        static const QRegularExpression rx2("^[A-Za-z]+\\d?");
        const QRegularExpressionMatch match2 = rx2.match(hiddenParameterName);
        tokenizer->ungetToken();
        if (match1.hasMatch() && match2.hasMatch() && cHiddenParameterNames.indexOf(match2.captured()) != -1)
          parameterModel = new OmFieldModel(tokenizer, worldPath);
      } else {
        for (currentParameterIndex = 0; currentParameterIndex < protoFieldModels.size(); ++currentParameterIndex) {
          OmFieldModel *protoFieldModel = protoFieldModels.at(currentParameterIndex);
          if (parameterName == protoFieldModel->name()) {
            parameterModel = protoFieldModel;
            break;
          }
        }
      }

      if (hasDefaultDefNodes) {
        if (currentParameterIndex < nextParameterIndex) {
          // if the parameters are not listed using the order defined in the PROTO declaration the DEF map could be wrong
          if (fieldOrderWarning) {
            tokenizer->reportFileError(
              tr("Wrong order of fields in the instance of PROTO %1: USE nodes might refer to the wrong DEF nodes")
                .arg(proto->name()));
            fieldOrderWarning = false;
          }
          // remove DEF nodes from default parameter if any
          QMapIterator<QString, OmNode *> defNodesMapIt(parametersDefMap[currentParameterIndex]);
          while (defNodesMapIt.hasNext()) {
            defNodesMapIt.next();
            OmNodeReader::current()->removeDefNode(defNodesMapIt.value());
          }
        } else {
          // add DEF nodes from default parameter defined before the current parameter
          for (int i = nextParameterIndex; i < currentParameterIndex; ++i) {
            QMapIterator<QString, OmNode *> defNodesMapIt(parametersDefMap[i]);
            while (defNodesMapIt.hasNext()) {
              defNodesMapIt.next();
              OmNodeReader::current()->addDefNode(defNodesMapIt.value());
            }
          }
          nextParameterIndex = currentParameterIndex + 1;
        }
      }

      if (parameterModel) {
        OmField *parameter = new OmField(parameterModel, NULL);
        parameter->setScope(tokenizer->referralFile());

        bool toBeDeleted = parameterNames.contains(parameter->name());
        if (toBeDeleted)
          // duplicated parameter definition to be ignored
          tokenizer->reportFileError(
            tr("Duplicated definition of field %1 in the instance of PROTO %2").arg(parameter->name()).arg(proto->name()));
        else {
          parameterNames << parameter->name();
          bool substitution = false;
          for (int i = 0; i < parametersList.size(); ++i) {
            if (parameter->name() == parametersList.at(i)->name() && parameter->type() == parametersList.at(i)->type()) {
              delete parametersList.at(i);
              parametersList.replace(i, parameter);
              substitution = true;
              break;
            }
          }

          if (hidden)
            parametersList.append(parameter);
          else if (!substitution) {
            toBeDeleted = true;
            tokenizer->reportFileError(
              tr("Parameter '%1' not supported in PROTO '%2'").arg(parameter->name()).arg(proto->name()));
          }
        }

        if (tokenizer->peekWord() == "IS") {
          tokenizer->skipToken("IS");
          const QString &alias = tokenizer->nextWord();
          if (!toBeDeleted) {
            parameter->setAlias(alias);
            copyAliasValue(parameter, alias);
          }
        } else if (!hidden)
          parameter->readValue(tokenizer, worldPath);

        if (toBeDeleted)
          delete parameter;
      } else
        tokenizer->reportFileError(tr("Parameter '%1' not supported in PROTO '%2'").arg(parameterName).arg(proto->name()));
    }

    if (hasDefaultDefNodes) {
      // add DEF nodes from the last default parameters
      for (int i = nextParameterIndex; i < protoFieldModels.size(); ++i) {
        QMapIterator<QString, OmNode *> defNodesMapIt(parametersDefMap[i]);
        while (defNodesMapIt.hasNext()) {
          defNodesMapIt.next();
          OmNodeReader::current()->addDefNode(defNodesMapIt.value());
        }
      }
    }
    tokenizer->skipToken("}");
  }

  parametersDefMap.clear();

  return createProtoInstanceFromParameters(proto, parametersList, worldPath);
}

OmNode *OmNode::createProtoInstanceFromParameters(OmProtoModel *proto, const QList<OmField *> &parameters,
                                                  const QString &worldPath, int uniqueId) {
  ProtoParameters *p = new ProtoParameters;
  p->params = &parameters;
  gProtoParameterList << p;

  const bool prevInstantiateMode = instantiateMode();
  setInstantiateMode(false);
  OmNode *newNode = proto->generateRoot(parameters, worldPath, uniqueId);
  setInstantiateMode(prevInstantiateMode);
  if (!newNode) {
    delete gProtoParameterList.takeLast();
    // cppcheck-suppress memleak
    return NULL;
  }
  proto->ref(true);

  OmNode *const instance = newNode->cloneAndReferenceProtoInstance();
  const int id = newNode->uniqueId();  // we want to keep this id because it should match the 'context.id' value used when
                                       // generating procedural PROTO nodes
  delete newNode;

  instance->mProto = proto;
  instance->mProtoParents.prepend(new OmNodeProtoInfo(proto->name(), parameters));
  if (id >= 0)
    instance->setUniqueId(id);

  QList<OmField *> notAssociatedDerivedParameters;  // populated for derived PROTO only
  if (proto->isDerived()) {
    QMutableVectorIterator<OmField *> paramIt(instance->mParameters);
    while (paramIt.hasNext()) {
      OmField *param = paramIt.next();

      // search for alias parameter and remove intermediate parameters
      QListIterator<OmField *> aliasIt(parameters);
      bool remove = false;
      bool aliasNotFound = true;
      while (aliasIt.hasNext()) {
        OmField *aliasParam = aliasIt.next();
        if (aliasParam->type() != param->type())
          continue;
        if (aliasParam->name() == param->alias()) {
          if (!aliasParam->isTemplateRegenerator()) {
            const bool paramTemplate = param->isTemplateRegenerator();
            aliasParam->setTemplateRegenerator(paramTemplate);
            if (paramTemplate)
              instance->mProto->setIsTemplate(true);
          }

          OmNode *tmpParent = gParent;
          foreach (OmField *internalField, param->internalFields()) {
            gParent = internalField->parentNode();
            internalField->redirectTo(aliasParam);
            internalField->setAlias(aliasParam->name());
          }
          gParent = tmpParent;

          foreach (OmNodeProtoInfo *protoInfo, instance->mProtoParents)
            protoInfo->redirectFields(param, aliasParam);

          aliasNotFound = false;
          remove = true;
        } else if (aliasParam->name() == param->name()) {
          // homonymous derived and base parameters
          notAssociatedDerivedParameters.append(aliasParam);
          aliasNotFound = false;
        }
      }

      if (remove) {
        paramIt.remove();
        param->clearInternalFields();
        delete param;
      } else if (aliasNotFound)
        // base PROTO parameter not overwritten by derived PROTO parameter
        // copy values from base PROTO default parameter
        instance->redirectAliasedFields(param, instance, false, true);
    }
  }

  foreach (OmField *parameter, parameters) {
    // remove first the parameters in case of direct nested PROTOs
    QMutableVectorIterator<OmField *> it(instance->mParameters);
    while (it.hasNext()) {
      OmField *f = it.next();
      if (f->name() == parameter->name() && f->type() == parameter->type()) {
        it.remove();
        delete f;
      }
    }

    if (parameter->parentNode())
      disconnect(parameter, &OmField::valueChanged, parameter->parentNode(), &OmNode::notifyParameterChanged);
    parameter->setParentNode(instance);
    instance->mParameters.append(parameter);
    connect(parameter, &OmField::valueChanged, instance, &OmNode::notifyParameterChanged);

    // set the parent of the parameter nodes
    switch (parameter->type()) {
      case WB_MF_NODE: {
        OmMFNode *mfnode = static_cast<OmMFNode *>(parameter->value());
        assert(mfnode);
        for (int i = 0; i < mfnode->size(); i++) {
          OmNode *subnode = mfnode->item(i);
          subnode->setParentNode(instance);
        }
        break;
      }
      case WB_SF_NODE: {
        OmSFNode *sfnode = static_cast<OmSFNode *>(parameter->value());
        assert(sfnode);
        OmNode *subnode = sfnode->value();
        if (subnode)
          subnode->setParentNode(instance);
        break;
      }
      default:
        break;
    }

    if (!(proto->isDerived() && notAssociatedDerivedParameters.contains(parameter)))
      instance->redirectAliasedFields(parameter, instance);
  }

  // set the parent node of internal parameters
  foreach (const OmField *f, instance->mInternalProtoParameters) {
    QList<OmField *> internalFields = f->internalFields();
    while (!internalFields.isEmpty()) {
      OmField *internalField = internalFields.takeFirst();
      internalField->setParentNode(instance);
      internalFields << internalField->internalFields();
    }
  }

  // remove the fake parameters introduced in case of direct nested PROTOs
  QMutableVectorIterator<OmField *> fieldIt(instance->mParameters);
  while (fieldIt.hasNext()) {
    // cppcheck-suppress constVariablePointer
    OmField *f = fieldIt.next();
    if (!f->isHiddenParameter() && proto->findFieldModel(f->name()) == NULL) {
      fieldIt.remove();
      // The field can still be accessed through the proto info, so don't delete it
      instance->mInternalProtoParameters << f;
    }
  }
  delete gProtoParameterList.takeLast();

  instance->updateNestedProtoFlag();
  if (!instance->mIsNestedProtoNode) {
    QMutableVectorIterator<OmField *> it(instance->mParameters);
    while (it.hasNext()) {
      OmField *parameter = it.next();
      if (parameter->isHiddenParameter()) {
        instance->readHiddenKinematicParameter(parameter);
        it.remove();
        delete parameter;
      }
    }
  }

  //  make sure internal fields are correctly connected to ancestor PROTO parameters
  instance->redirectInternalFields();

  return instance;
}

void OmNode::updateNestedProtoFlag(bool hasAProtoAncestorFlag) {
  const bool newValue = isProtoInstance() && (hasAProtoAncestorFlag || hasAProtoAncestor());
  if (newValue && mIsNestedProtoNode)
    return;  // flag already set
  mIsNestedProtoNode = newValue;
  const QList<OmField *> fieldList = fields() + parameters();
  foreach (const OmField *f, fieldList) {
    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(f->value());
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(f->value());
    if (sfnode) {
      OmNode *n = sfnode->value();
      if (n)
        n->updateNestedProtoFlag(mIsNestedProtoNode);
    } else if (mfnode) {
      for (int i = 0; i < mfnode->size(); ++i)
        mfnode->item(i)->updateNestedProtoFlag(mIsNestedProtoNode);
    }
  }
  foreach (OmNode *instance, protoParameterNodeInstances())
    instance->updateNestedProtoFlag();
}

void OmNode::setCreationCompleted() {
  if (mIsShallowNode)
    return;

  mIsProtoParameterNodeDescendant = isProtoParameterNode();
  mIsCreationCompleted = true;
}

void OmNode::reset(const QString &id) {
  mCurrentStateId = id;
  if (isTemplate() && !mProto->isDeterministic())
    // nonDeterministic procedural PROTO must be regenerated on reset
    setRegenerationRequired(true);
}

bool OmNode::isProtoParameterChild(const OmNode *node) const {
  if (!isProtoInstance())
    return false;

  if (node->mProtoParameterParentNode)
    return node->mProtoParameterParentNode == this;

  foreach (const OmField *const p, parameters()) {
    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(p->value());
    if (sfnode && sfnode->value() == node) {
      node->mProtoParameterParentNode = this;
      return true;
    }
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(p->value());
    if (mfnode && mfnode->nodeIndex(node) != -1) {
      node->mProtoParameterParentNode = this;
      return true;
    }
  }

  node->mProtoParameterParentNode = node;  // set to self if parent is not a PROTO
  return false;
}

bool OmNode::isProtoParameterNode() const {
  if (mIsCreationCompleted)
    return mIsProtoParameterNodeDescendant;

  if (mIsProtoParameterNode)
    return mIsProtoParameterNode[0];
  mIsProtoParameterNode = new bool[1];
  const OmNode *parent = parentNode();
  if (!parent || parent->isWorldRoot()) {
    mIsProtoParameterNode[0] = false;
    return false;
  }

  if (parent->isProtoParameterChild(this)) {
    mIsProtoParameterNode[0] = true;
    return true;
  }

  mIsProtoParameterNode[0] = parent->isProtoParameterNode();
  return mIsProtoParameterNode[0];
}

QList<OmNode *> OmNode::subNodes(bool recurse, bool searchInFields, bool searchInParameters) const {
  QList<OmNode *> result;
  QList<OmField *> fieldsList;
  // first add the parameters and then the fields
  if (searchInParameters)
    fieldsList += mParameters;
  if (searchInFields)
    fieldsList += mFields;
  if (!searchInFields && !searchInParameters)
    fieldsList += fieldsOrParameters();

  QList<OmField *>::iterator fieldIt;
  for (fieldIt = fieldsList.begin(); fieldIt != fieldsList.end(); ++fieldIt)
    result.append(subNodes((*fieldIt), recurse, searchInFields, searchInParameters));
  return result;
}

QList<OmNode *> OmNode::subNodes(const OmField *field, bool recurse, bool searchInFields, bool searchInParameters) {
  QList<OmNode *> result;
  const OmValue *const value = field->value();
  const OmSFNode *const sfnode = dynamic_cast<const OmSFNode *>(value);
  if (sfnode && sfnode->value()) {
    // cppcheck-suppress constVariablePointer
    OmNode *const node = sfnode->value();
    result.append(node);
    if (recurse)
      result.append(node->subNodes(recurse, searchInFields, searchInParameters));
  } else {
    const OmMFNode *const mfnode = dynamic_cast<const OmMFNode *>(value);
    if (mfnode) {
      for (int i = 0; i < mfnode->size(); ++i) {
        // cppcheck-suppress constVariablePointer
        OmNode *const node = mfnode->item(i);
        result.append(node);
        if (recurse)
          result.append(node->subNodes(recurse, searchInFields, searchInParameters));
      }
    }
  }

  return result;
}

bool OmNode::isTemplate() const {
  if (mProto)
    return mProto->isTemplate();
  return false;
}

void OmNode::setRegenerationRequired(bool required) {
  mRegenerationRequired = required;
  if (required)
    emit regenerationRequired();
}

bool OmNode::hasAProtoAncestor() const {
  const OmNode *currentNode = parentNode();
  while (currentNode) {
    if (currentNode->isProtoInstance())
      return true;

    const OmNode *ppn = currentNode->protoParameterNode();
    if (ppn && ppn->isProtoInstance())
      return true;

    currentNode = currentNode->parentNode();
  }

  return false;
}

OmNode *OmNode::protoAncestor() const {
  OmNode *currentNode = parentNode();
  while (currentNode) {
    if (currentNode->isProtoInstance())
      return currentNode;

    OmNode *ppn = currentNode->protoParameterNode();
    if (ppn && ppn->isProtoInstance())
      return ppn;

    currentNode = currentNode->parentNode();
  }
  return NULL;
}

bool OmNode::isAnAncestorOf(const OmNode *node) const {
  const OmNode *currentNode = node;
  while (currentNode) {
    currentNode = currentNode->parentNode();
    if (currentNode == this)
      return true;
  }
  return false;
}

int OmNode::level() const {
  int level = 0;
  const OmNode *node = this;
  while (node->parentNode()) {
    node = node->parentNode();
    ++level;
  };

  return level;
}

/////////////////////
// Index functions //
/////////////////////

int OmNode::subNodeIndex(const OmNode *subNode, const OmNode *root) {
  assert(subNode && root);

  if (subNode == root)
    return 0;

  bool subNodeFound = false;
  int result = 0;

  const QList<OmField *> &fieldsList = root->fields();
  foreach (const OmField *f, fieldsList) {
    const OmValue *value = f->value();
    const OmSFNode *sfNode = dynamic_cast<const OmSFNode *>(value);
    if (sfNode) {
      const OmNode *node = sfNode->value();
      if (node)
        subNodeIndex(node, subNode, result, subNodeFound);
    } else {
      const OmMFNode *mfNode = dynamic_cast<const OmMFNode *>(value);
      if (mfNode) {
        const int n = mfNode->size();
        for (int i = 0; !subNodeFound && (i < n); ++i) {
          const OmNode *node = mfNode->item(i);
          subNodeIndex(node, subNode, result, subNodeFound);
        }
      }
    }

    if (subNodeFound)
      return result;
  }

  return -1;
}

void OmNode::subNodeIndex(const OmNode *currentNode, const OmNode *targetNode, int &index, bool &subNodeFound) {
  ++index;

  if (targetNode == currentNode) {
    subNodeFound = true;
    return;
  }

  const QList<OmField *> &fieldsList = currentNode->fields();
  foreach (const OmField *f, fieldsList) {
    const OmValue *value = f->value();
    const OmSFNode *sfNode = dynamic_cast<const OmSFNode *>(value);
    if (sfNode) {
      const OmNode *node = sfNode->value();
      if (node)
        subNodeIndex(node, targetNode, index, subNodeFound);
    } else {
      const OmMFNode *mfNode = dynamic_cast<const OmMFNode *>(value);
      if (mfNode) {
        const int n = mfNode->size();
        for (int i = 0; !subNodeFound && (i < n); i++) {
          const OmNode *node = mfNode->item(i);
          subNodeIndex(node, targetNode, index, subNodeFound);
        }
      }
    }

    if (subNodeFound)
      return;
  }
}

OmNode *OmNode::findNodeFromSubNodeIndices(const QList<int> &indices, OmNode *root) {
  OmNode *n = root;
  for (int i = 0; i < indices.size() && n != NULL; ++i)
    n = findNodeFromSubNodeIndex(indices[i], n);
  return n;
}

OmNode *OmNode::findNodeFromSubNodeIndex(int index, OmNode *root) {
  if (index == 0)
    return root;

  const QList<OmField *> &fieldsList = root->fields();
  foreach (const OmField *f, fieldsList) {
    const OmValue *value = f->value();
    const OmSFNode *sfNode = dynamic_cast<const OmSFNode *>(value);
    if (sfNode) {
      OmNode *node = sfNode->value();
      if (node) {
        OmNode *returnNode = findNode(index, node);
        if (index == 0)
          return returnNode;
      }
    } else {
      const OmMFNode *mfNode = dynamic_cast<const OmMFNode *>(value);
      if (mfNode) {
        const int n = mfNode->size();
        for (int i = 0; (index > 0) && (i < n); i++) {
          OmNode *node = mfNode->item(i);
          OmNode *returnNode = findNode(index, node);
          if (index == 0)
            return returnNode;
        }
      }
    }
  }

  return NULL;
}

OmNode *OmNode::findNode(int &index, OmNode *root) {
  --index;

  if (index == 0)
    return root;

  const QList<OmField *> &fieldsList = root->fields();
  foreach (const OmField *f, fieldsList) {
    const OmValue *value = f->value();
    const OmSFNode *sfNode = dynamic_cast<const OmSFNode *>(value);
    if (sfNode) {
      OmNode *node = sfNode->value();
      if (node) {
        OmNode *returnNode = findNode(index, node);
        if (index == 0)
          return returnNode;
      }
    } else {
      const OmMFNode *mfNode = dynamic_cast<const OmMFNode *>(value);
      if (mfNode) {
        const int n = mfNode->size();
        for (int i = 0; (index > 0) && (i < n); i++) {
          OmNode *returnNode = findNode(index, mfNode->item(i));
          if (index == 0)
            return returnNode;
        }
      }
    }
  }

  return NULL;
}

void OmNode::disconnectFieldNotification(const OmValue *value) {
  foreach (const OmField *f, mFields) {
    if (f->value() == value)
      disconnect(f, &OmField::valueChanged, this, &OmNode::notifyFieldChanged);
  }
}

void OmNode::setFieldsParentNode() {
  QList<OmNode *> nodes(subNodes(true, true, true));
  nodes.prepend(this);
  foreach (OmNode *n, nodes) {
    QList<OmField *> fieldsList = n->mFields + n->mParameters;
    foreach (OmField *f, fieldsList)
      f->setParentNode(n);
  }
}

QStringList OmNode::documentationBookAndPage(bool isRobot) const {
  if (isProtoInstance()) {
    QStringList bookAndPage(mProto->documentationBookAndPage(isRobot, false));
    if (!bookAndPage.isEmpty())
      return bookAndPage;
  }
  return mModel->documentationBookAndPage();
}

QString OmNode::getUrdfPrefix() const {
  const OmNode *robotAncestor = this;
  while (robotAncestor && !robotAncestor->isRobot())
    robotAncestor = robotAncestor->parentNode();

  return robotAncestor ? robotAncestor->mUrdfPrefix : QString();
}

/*
#include <QtCore/QDebug>
void OmNode::printDebugNodeStructure(int level) {
  QString indent;
  for (int i = 0; i < level; ++i)
    indent += "  ";

  qDebug() << QString("%1Node %2 0x%3 id %4 parameterNode 0x%5")
                .arg(indent.toStdString().c_str())
                .arg(usefulName().toStdString().c_str())
                .arg((quintptr)this, 0, 16)
                .arg(uniqueId())
                .arg((quintptr)protoParameterNode(), 0, 16);
  printDebugNodeFields(level, true);
  printDebugNodeFields(level, false);
}

void OmNode::printDebugNodeFields(int level, bool printParameters) {
  QString indent;
  for (int i = 0; i < level; ++i)
    indent += "  ";

  QString line;
  const QString type = printParameters ? "Parameter" : "Field";
  const QList<OmField *> fieldList = printParameters ? parameters() : fields();
  foreach (OmField *p, fieldList) {
    qDebug() << QString("%1%2 %3 0x%4 (alias 0x%5):")
                  .arg(indent.toStdString().c_str())
                  .arg(type.toStdString().c_str())
                  .arg(p->name().toStdString().c_str())
                  .arg((quintptr)p, 0, 16)
                  .arg((quintptr)p->parameter(), 0, 16);
    if (p->type() == WB_SF_NODE) {
      OmNode *n = dynamic_cast<OmSFNode *>(p->value())->value();
      if (n)
        n->printDebugNodeStructure(level + 1);
    } else if (p->type() == WB_MF_NODE) {
      OmMFNode *mfnode = dynamic_cast<OmMFNode *>(p->value());
      for (int i = 0; i < mfnode->size(); ++i) {
        OmNode *n = mfnode->item(i);
        if (n)
          n->printDebugNodeStructure(level + 1);
      }
    } else
      qDebug() << QString("%1 %2 (alias %3)")
                    .arg(indent.toStdString().c_str())
                    .arg(p->toString(OmPrecision::GUI_LOW).toStdString().c_str())
                    .arg(p->alias());
  }
}
*/
