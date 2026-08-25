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

#include "OmNodeOperations.hpp"

#include "OmBaseNode.hpp"
#include "OmDictionary.hpp"
#include "OmField.hpp"
#include "OmFileUtil.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmNode.hpp"
#include "OmNodeReader.hpp"
#include "OmNodeUtilities.hpp"
#include "OmParser.hpp"
#include "OmProject.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmRobot.hpp"
#include "OmSFNode.hpp"
#include "OmSelection.hpp"
#include "OmSolid.hpp"
#include "OmTemplateManager.hpp"
#include "OmTokenizer.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"

#include <QtCore/QCoreApplication>
#include <QtCore/QFileInfo>

#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <cassert>

static bool isRegionOccupied(const OmVector3 &pos) {
  const OmWorld *const world = OmWorld::instance();
  const double ls = world->worldInfo()->lineScale();
  const QList<OmSolid *> &l = world->topSolids();
  foreach (const OmSolid *const solid, l) {
    const OmVector3 &dist = solid->translation() - pos;
    if (dist.length() < ls)
      return true;
  }

  return false;
}

// simple rule to avoid that inserted or pasted robots occupy exactly the same 3d region
static void tryToAvoidIntersections(OmNode *node) {
  OmRobot *const robot = dynamic_cast<OmRobot *>(node);
  if (robot) {
    OmVector3 tr = robot->translation();
    while (isRegionOccupied(tr)) {
      const double ls = OmWorld::instance()->worldInfo()->lineScale();
      tr.setXyz(tr.x() + ls, tr.y(), tr.z() + ls);
    }
    robot->setTranslation(tr.x(), tr.y(), tr.z());
  }
}

OmNodeOperations *OmNodeOperations::cInstance = NULL;

OmNodeOperations *OmNodeOperations::instance() {
  if (!cInstance)
    cInstance = new OmNodeOperations();
  return cInstance;
}

void OmNodeOperations::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

OmNodeOperations::OmNodeOperations() : mNodesAreAboutToBeInserted(false), mSkipUpdates(false), mFromSupervisor(false) {
}

void OmNodeOperations::enableSolidNameClashCheckOnNodeRegeneration(bool enabled) const {
  if (enabled)
    connect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this,
            &OmNodeOperations::resolveSolidNameClashIfNeeded, Qt::UniqueConnection);
  else
    disconnect(OmTemplateManager::instance(), &OmTemplateManager::postNodeRegeneration, this,
               &OmNodeOperations::resolveSolidNameClashIfNeeded);
}

OmNodeOperations::OperationResult OmNodeOperations::importNode(int nodeId, int fieldId, int itemIndex, ImportType origin,
                                                               const QString &nodeString) {
  OmBaseNode *parentNode = static_cast<OmBaseNode *>(OmNode::findNode(nodeId));
  assert(parentNode);

  OmField *field = parentNode->field(fieldId);
  assert(field);

  return importNode(parentNode, field, itemIndex, origin, nodeString, false);
}

OmNodeOperations::OperationResult OmNodeOperations::importNode(OmNode *parentNode, OmField *field, int itemIndex,
                                                               ImportType origin, const QString &nodeString,
                                                               bool avoidIntersections) {
  setFromSupervisor(origin == FROM_SUPERVISOR);

  OmSFNode *sfnode = dynamic_cast<OmSFNode *>(field->value());
#ifndef NDEBUG
  OmMFNode *mfnode = dynamic_cast<OmMFNode *>(field->value());
  assert(mfnode || sfnode);
  // index value is assumed to be in range [0, mfnode->size()]
  // user input checked in wb_supervisor_field_import_mf_node_from_string or OmSceneTree
  assert(!mfnode || (itemIndex >= 0 && itemIndex <= mfnode->size()));
#endif

  OmTokenizer tokenizer;
  int errors = 0;
  if (!nodeString.isEmpty()) {
    tokenizer.setReferralFile(OmWorld::instance() ? OmWorld::instance()->fileName() : "");
    errors = tokenizer.tokenizeString(nodeString);
  } else {
    setFromSupervisor(false);
    return FAILURE;
  }

  if (errors) {
    setFromSupervisor(false);
    return FAILURE;
  }

  // note: the presence of the declaration for importable PROTO must be checked prior to checking the syntax since
  // in order to evaluate the latter the PROTO themselves must be locally available and readable
  OmParser parser(&tokenizer);
  const QStringList protoList = parser.protoNodeList();
  foreach (const QString &protoName, protoList) {
    // ensure the node was declared as EXTERNPROTO prior to import it using a supervisor
    if (mFromSupervisor && !OmProtoManager::instance()->isImportableExternProtoDeclared(protoName)) {
      OmLog::error(
        tr("In order to import the PROTO '%1', first it must be declared in the IMPORTABLE EXTERNPROTO list.").arg(protoName));
      setFromSupervisor(false);
      return FAILURE;
    }
  }

  // check syntax
  if (!parser.parseObject(OmWorld::instance()->fileName())) {
    setFromSupervisor(false);
    return FAILURE;
  }

  if (sfnode && sfnode->value() != NULL)
    // clear selection and set mSelectedItem to NULL
    OmSelection::instance()->selectPoseFromView3D(NULL);

  // read node
  OmNode::setGlobalParentNode(parentNode);
  OmNodeReader nodeReader;
  // set available DEF nodes to be used while reading the new nodes
  QList<OmNode *> defNodes = OmDictionary::instance()->computeDefForInsertion(parentNode, field, itemIndex, false);
  foreach (OmNode *node, defNodes)
    nodeReader.addDefNode(node);

  QList<OmNode *> nodes = nodeReader.readNodes(&tokenizer, OmWorld::instance()->fileName());
  if (sfnode && nodes.size() > 1)
    OmLog::warning(tr("Trying to import multiple nodes in the '%1' SFNode field. "
                      "Only the first node will be inserted")
                     .arg(field->name()),
                   false, OmLog::PARSING);

  const OmNode::NodeUse nodeUse = dynamic_cast<OmBaseNode *>(parentNode)->nodeUse();
  OmBaseNode *childNode = NULL;
  bool isNodeRegenerated = false;
  int nodeIndex = itemIndex;
  foreach (OmNode *node, nodes) {
    childNode = static_cast<OmBaseNode *>(node);
    QString errorMessage;
    if (OmNodeUtilities::isAllowedToInsert(field, parentNode, errorMessage, nodeUse, OmNodeUtilities::slotType(childNode),
                                           childNode, false)) {
      if (avoidIntersections)
        tryToAvoidIntersections(childNode);
      const OperationResult result = initNewNode(childNode, parentNode, field, nodeIndex, true);
      if (result == FAILURE)
        continue;
      else if (result == REGENERATION_REQUIRED)
        isNodeRegenerated = true;
      ++nodeIndex;
      if (!field->isTemplateRegenerator() && !isNodeRegenerated)
        emit nodeAdded(childNode);
      // we need to emit this signal after finalize so that the mass properties are displayed properly
      // in the scene tree.
      // FIXME: this should be removed as the emit massPropertiesChanged() should be called from within
      // the OmSolid class when actually changing the mass properties...
      // OmSolid *const solid = dynamic_cast<OmSolid*>(childNode);
      // if (solid)
      //  solid->emit massPropertiesChanged();
    } else {
      assert(!errorMessage.isEmpty());
      OmLog::error(errorMessage, false, OmLog::PARSING);
    }

    if (sfnode)
      break;
  }

  setFromSupervisor(false);
  return isNodeRegenerated ? REGENERATION_REQUIRED : SUCCESS;
}

OmNodeOperations::OperationResult OmNodeOperations::initNewNode(OmNode *newNode, OmNode *parentNode, OmField *field,
                                                                int newNodeIndex, bool subscribe, bool finalize) {
  const bool isInBoundingObject = dynamic_cast<OmSolid *>(parentNode) && field->name() == "boundingObject";
  if (!OmNodeUtilities::validateInsertedNode(field, newNode, parentNode, isInBoundingObject)) {
    delete newNode;
    return FAILURE;
  }

  OmBaseNode *const baseNode = dynamic_cast<OmBaseNode *>(newNode);
  // set parent node
  newNode->setParentNode(parentNode);
  OmNode *upperTemplate = OmVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(field, parentNode);
  bool isInsideATemplateRegenerator = upperTemplate && (upperTemplate != baseNode);

  // insert in parent field
  mNodesAreAboutToBeInserted = true;
  OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(field->value());
  if (mfnode) {
    if (isInsideATemplateRegenerator) {
      mfnode->blockSignals(true);  // otherwise, the node regeneration is called too early
      mfnode->insertItem(newNodeIndex, newNode);
      upperTemplate->regenerateNode();
    } else
      mfnode->insertItem(newNodeIndex, newNode);

  } else {
    OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(field->value());
    if (sfnode) {
      if (isInsideATemplateRegenerator) {
        sfnode->blockSignals(true);  // otherwise, the node regeneration is called too early
        sfnode->setValue(newNode);
        upperTemplate->regenerateNode();
      } else
        sfnode->setValue(newNode);
    }
  }
  mNodesAreAboutToBeInserted = false;

  // in case of template the newNode/baseNode pointers are no more available here
  // because the template node was regenerated, the node was finalized,
  // and the scene tree was updated
  if (isInsideATemplateRegenerator)
    return REGENERATION_REQUIRED;

  if (parentNode && parentNode->isProtoInstance())
    parentNode->redirectInternalFields(field);

  // update flag for PROTO nodes and their instances if any
  baseNode->updateNestedProtoFlag();
  if (finalize) {
    baseNode->finalize();

    assert(!OmWorld::instance()->isLoading());
  }
  resolveSolidNameClashIfNeeded(newNode);

  if (subscribe && baseNode->isTemplate())
    OmTemplateManager::instance()->subscribe(newNode, OmTemplateManager::isNodeChangeTriggeringRegeneration(baseNode));

  updateDictionary(baseNode->isUseNode(), baseNode);

  return SUCCESS;
}

void OmNodeOperations::resolveSolidNameClashIfNeeded(OmNode *node) const {
  const QList<OmNode *> instances = node->protoParameterNodeInstances();
  if (!instances.isEmpty()) {
    foreach (OmNode *n, instances)
      resolveSolidNameClashIfNeeded(n);
    return;
  }
  QList<OmSolid *> solidNodes;
  // cppcheck-suppress constVariablePointer
  OmSolid *solidNode = dynamic_cast<OmSolid *>(node);
  if (solidNode)
    solidNodes << solidNode;
  else
    solidNodes << OmNodeUtilities::findSolidDescendants(node);
  while (!solidNodes.isEmpty()) {
    OmSolid *s = solidNodes.takeFirst();
    const OmBaseNode *const parentBaseNode = dynamic_cast<OmBaseNode *>(s->parentNode());
    const OmSolid *parentSolidNode = dynamic_cast<const OmSolid *>(parentBaseNode);
    const OmSolid *upperSolid = parentSolidNode ? parentSolidNode : parentBaseNode->upperSolid();
    s->resolveNameClashIfNeeded(true, true,
                                upperSolid ? upperSolid->solidChildren().toList() : OmWorld::instance()->topSolids(), NULL);
  }
}

bool OmNodeOperations::deleteNode(OmNode *node, bool fromSupervisor) {
  if (node == NULL)
    return false;

  setFromSupervisor(fromSupervisor);

  if (dynamic_cast<OmSolid *>(node))
    OmWorld::instance()->awake();

  const bool dictionaryNeedsUpdate = OmVrmlNodeUtilities::hasAreferredDefNodeDescendant(node);
  OmField *parentField = node->parentField();
  assert(parentField);
  OmSFNode *sfnode = dynamic_cast<OmSFNode *>(parentField->value());
  OmMFNode *mfnode = dynamic_cast<OmMFNode *>(parentField->value());
  assert(sfnode || mfnode);
  notifyNodeDeleted(node);
  bool success;
  if (sfnode) {
    sfnode->setValue(NULL);
    success = true;
  } else {
    assert(mfnode);
    success = mfnode->removeNode(node);
    delete node;
  }

  if (success && dictionaryNeedsUpdate)
    updateDictionary(false, NULL);

  setFromSupervisor(false);

  purgeUnusedExternProtoDeclarations();

  return success;
}

void OmNodeOperations::requestUpdateDictionary() {
  updateDictionary(false, NULL);
}

bool OmNodeOperations::updateDictionary(bool load, OmBaseNode *protoRoot) {
  mSkipUpdates = true;
  OmNode::setDictionaryUpdateFlag(true);
  OmDictionary *dictionary = OmDictionary::instance();
  const bool regenerationRequired = dictionary->update(load);  // update all DEF-USE dependencies
  if (protoRoot && !protoRoot->isUseNode())
    dictionary->updateProtosPrivateDef(protoRoot);
  OmNode::setDictionaryUpdateFlag(false);
  mSkipUpdates = false;
  return regenerationRequired;
}

void OmNodeOperations::requestUpdateSceneDictionary(OmNode *node, bool fromUseToDef) {
  OmDictionary::instance()->updateNodeDefName(node, fromUseToDef);
}

void OmNodeOperations::notifyNodeAdded(OmNode *node) {
  emit nodeAdded(node);
}

void OmNodeOperations::notifyNodeDeleted(OmNode *node) {
  emit nodeDeleted(node);
}

void OmNodeOperations::setFromSupervisor(bool value) {
  mFromSupervisor = value;
  OmProtoManager::instance()->setImportedFromSupervisor(value);
}

void OmNodeOperations::purgeUnusedExternProtoDeclarations() {
  assert(OmWorld::instance());
  // list all the PROTO model names used in the world file
  QList<const OmNode *> protoList(OmVrmlNodeUtilities::protoNodesInWorldFile(OmWorld::instance()->root()));
  QSet<QString> modelNames;
  foreach (const OmNode *proto, protoList)
    modelNames.insert(proto->modelName());

  // delete PROTO declaration if not found in list
  OmProtoManager::instance()->purgeUnusedExternProtoDeclarations(modelNames);
}

void OmNodeOperations::updateExternProtoDeclarations(OmField *modifiedField) {
  if (modifiedField->isDefault())
    return;  // OmNodeOperations::purgeUnusedExternProtoDeclarations() will be called

  OmNode *modifiedNode = static_cast<OmNode *>(sender());
  if (modifiedNode == NULL || modifiedNode->isWorldRoot())
    return;

  const OmNode *topProto = modifiedNode->isProtoInstance() ? modifiedNode : NULL;
  OmNode *n = modifiedNode->parentNode();
  while (n) {
    if (n->isProtoInstance())
      topProto = n;
    n = n->parentNode();
  }
  if (!topProto)
    return;

  QList<const OmNode *> protoList(OmVrmlNodeUtilities::protoNodesInWorldFile(topProto));
  foreach (const OmNode *proto, protoList) {
    const QString previousUrl(
      OmProtoManager::instance()->declareExternProto(proto->modelName(), proto->proto()->url(), false, false));
    if (!previousUrl.isEmpty())
      OmLog::warning(tr("Conflicting declarations for '%1' are provided: \"%2\" and \"%3\", the first one will be used after "
                        "saving and reverting the world. "
                        "To use the other instead you will need to change it manually in the world file.")
                       .arg(proto->modelName())
                       .arg(previousUrl)
                       .arg(proto->proto()->url()));
  }
}
