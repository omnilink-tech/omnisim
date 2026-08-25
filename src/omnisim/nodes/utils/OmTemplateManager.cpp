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

#include "OmTemplateManager.hpp"

#include "OmAppearance.hpp"
#include "OmBasicJoint.hpp"
#include "OmDictionary.hpp"
#include "OmField.hpp"
#include "OmFieldModel.hpp"
#include "OmGeometry.hpp"
#include "OmGroup.hpp"
#include "OmLog.hpp"
#include "OmMFNode.hpp"
#include "OmNode.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPbrAppearance.hpp"
#include "OmProtoModel.hpp"
#include "OmSFNode.hpp"
#include "OmShape.hpp"
#include "OmSlot.hpp"
#include "OmSolid.hpp"
#include "OmSolidReference.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"

#include <QtCore/QCoreApplication>

#include <cassert>

OmTemplateManager *OmTemplateManager::cInstance = NULL;
int OmTemplateManager::cRegeneratingNodeCount = 0;

OmTemplateManager *OmTemplateManager::instance() {
  if (!cInstance) {
    cInstance = new OmTemplateManager();
    qAddPostRoutine(OmTemplateManager::cleanup);
  }
  return cInstance;
}

void OmTemplateManager::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

OmTemplateManager::OmTemplateManager() :
  mBlockRegeneration(false),
  mTemplatesNeedRegeneration(false),
  mRegeneratingUpperTemplateNode(NULL) {
}

OmTemplateManager::~OmTemplateManager() {
  clear();
}

void OmTemplateManager::blockRegeneration(bool block) {
  mBlockRegeneration = block;

  if (!block && mTemplatesNeedRegeneration) {  // regenerates all the required nodes
    while (true) {
      bool regenerated = false;
      foreach (OmNode *node, mTemplates) {
        if (node->isRegenerationRequired()) {
          regenerateNode(node);  // mTemplates can be modified during this call
          regenerated = true;
          break;
        }
      }
      if (regenerated)
        continue;
      else
        break;
    }
    mTemplatesNeedRegeneration = false;
  }
}

void OmTemplateManager::clear() {
  foreach (const OmNode *node, mTemplates)
    disconnect(node, &OmNode::regenerationRequired, this, &OmTemplateManager::nodeNeedRegeneration);
  mTemplates.clear();
}

void OmTemplateManager::subscribe(OmNode *node, bool subscribedDescendant) {
  bool subscribed = false;
  if (node->isTemplate() && !mTemplates.contains(node)) {
    subscribed = true;
    mTemplates << node;
    connect(node, &OmNode::regenerateNodeRequest, this, &OmTemplateManager::regenerateNode, Qt::UniqueConnection);
    connect(node, &OmNode::regenerationRequired, this, &OmTemplateManager::nodeNeedRegeneration);
  }
  if (subscribedDescendant)
    mNodesSubscribedForRegeneration.insert(node);
  recursiveFieldSubscribeToRegenerateNode(node, subscribed, subscribedDescendant);
  connect(node, &QObject::destroyed, this, &OmTemplateManager::unsubscribe, Qt::UniqueConnection);
}

void OmTemplateManager::unsubscribe(QObject *node) {
  const OmNode *n = static_cast<OmNode *>(node);
  if (n->isTemplate() && mTemplates.removeAll(n) > 0)
    disconnect(n, &OmNode::regenerationRequired, this, &OmTemplateManager::nodeNeedRegeneration);
  mNodesSubscribedForRegeneration.remove(n);
}

bool OmTemplateManager::nodeNeedsToSubscribe(OmNode *node) {
  if (!node->isProtoInstance())
    return false;

  foreach (const OmField *field, node->fieldsOrParameters()) {
    if (!field->alias().isEmpty())
      return true;
  }
  return false;
}

void OmTemplateManager::recursiveFieldSubscribeToRegenerateNode(OmNode *node, bool subscribedNode, bool subscribedDescendant) {
  if (subscribedNode || subscribedDescendant) {
    if (node->isProtoInstance())
      connect(node, &OmNode::parameterChanged, this, &OmTemplateManager::regenerateNodeFromField, Qt::UniqueConnection);
    else
      connect(node, &OmNode::fieldChanged, this, &OmTemplateManager::regenerateNodeFromField, Qt::UniqueConnection);
  }

  // if PROTO node:
  //   - subscribe sub-nodes in parameters
  //   - subscribe sub-nodes in fields if a parameter is redirected to the sub-node
  // else normal nodes:
  //   - subscribe sub-nodes in fields
  QVector<OmField *> fields = node->fields();
  int directSubscribeMinIndex = 0;
  if (node->isProtoInstance()) {
    directSubscribeMinIndex = fields.size();
    fields.append(node->parameters());
  }
  OmField *field = NULL;
  for (int i = 0; i < fields.size(); ++i) {
    field = fields[i];
    bool directSubscriptionEnabled = i >= directSubscribeMinIndex;
    switch (field->type()) {
      case WB_MF_NODE: {
        OmMFNode *mfnode = static_cast<OmMFNode *>(field->value());
        assert(mfnode);
        for (int j = 0; j < mfnode->size(); j++) {
          OmNode *subnode = mfnode->item(j);
          if (directSubscriptionEnabled || nodeNeedsToSubscribe(subnode))
            subscribe(subnode, subscribedDescendant || (subscribedNode && field->isTemplateRegenerator()));
        }
        break;
      }
      case WB_SF_NODE: {
        OmSFNode *sfnode = static_cast<OmSFNode *>(field->value());
        assert(sfnode);
        OmNode *subnode = sfnode->value();
        if (subnode && (directSubscriptionEnabled || nodeNeedsToSubscribe(subnode)))
          subscribe(subnode, subscribedDescendant || (subscribedNode && field->isTemplateRegenerator()));
        break;
      }
      default:
        break;
    }
  }
}

// intermediate function to determine which node should be updated
// Note: The security is probably overkill there, but its also safer for the first versions of the template mechanism
void OmTemplateManager::regenerateNodeFromField(OmField *field) {
  // retrieve the right node
  OmNode *templateNode = dynamic_cast<OmNode *>(sender());
  assert(templateNode);
  if (!templateNode)
    return;

  // 1. retrieve upper template node where the modification appeared in a template regenerator field
  OmNode *upperTemplateNode = OmVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(field, templateNode);

  if (!upperTemplateNode)
    return;

  // 2. check it's not a parameter managed by ODE
  if (!field->isParameter() && dynamic_cast<const OmSolid *>(templateNode) &&
      ((field->name() == "translation" && field->type() == WB_SF_VEC3F) ||
       (field->name() == "rotation" && field->type() == WB_SF_ROTATION) ||
       (field->name() == "position" && field->type() == WB_SF_FLOAT)))
    return;

  // Store regenerator field and node to prevent infinite loop when updating the USE/DEF dictionary
  mRegeneratingUpperTemplateNode = upperTemplateNode;

  // 3. regenerate template where the modification appeared in a template regenerator field
  regenerateNode(upperTemplateNode);
}

void OmTemplateManager::regenerateNode(OmNode *node, bool restarted) {
  assert(node);

  if (mBlockRegeneration) {
    node->setRegenerationRequired(true);  // will be regenerated when deblocking this manager
    return;
  } else
    node->setRegenerationRequired(false);

  // 1. get stuff
  OmNode *parent = node->parentNode();
  OmProtoModel *proto = node->proto();
  assert(parent && proto);
  if (!parent || !proto)
    return;
  const bool isInBoundingObject = dynamic_cast<OmBaseNode *>(node)->isInBoundingObject();

  QList<OmField *> previousParentRedirections;
  OmField *parentField = node->parentField();
  QVector<OmField *> parameters;
  OmNode::setRestoreUniqueIdOnClone(true);
  foreach (const OmField *parameter, node->parameters()) {
    parameters << new OmField(*parameter, NULL);
    if (parameter->parameter() != NULL)
      previousParentRedirections.append(parameter->parameter());
  }
  OmNode::setRestoreUniqueIdOnClone(false);
  const int uniqueId = node->uniqueId();
  const QString &stateId = node->stateId();
  const OmSolid *solid = dynamic_cast<const OmSolid *>(node);
  OmVector3 translationFromFile;
  OmRotation rotationFromFile;
  if (solid) {
    translationFromFile = solid->translationFromFile(stateId);
    rotationFromFile = solid->rotationFromFile(stateId);
  }

  OmWorld *world = OmWorld::instance();
  OmGroup *root = OmWorld::instance()->root();
  bool isWorldInitialized = root && root->isPostFinalizedCalled();

  OmViewpoint *viewpoint = world->viewpoint();
  OmSolid *followedSolid = viewpoint == NULL ? NULL : viewpoint->followedSolid();
  bool isFollowedSolid = followedSolid == node;
  QString followedSolidName;
  if (followedSolid)
    followedSolidName = followedSolid->name();

  // 2. regenerate the new node
  OmNode *upperTemplateNode = OmVrmlNodeUtilities::findUpperTemplateNeedingRegeneration(node);
  bool nested = upperTemplateNode && upperTemplateNode != node;
  cRegeneratingNodeCount++;
  if (isWorldInitialized && !restarted)
    // signal is not emitted in case a node has been regenerated twice in a row (`restart` == TRUE)
    // to preserve the scene tree selection
    emit preNodeRegeneration(node, nested);

  OmNode::setGlobalParentNode(parent);

  OmNode *newNode = OmNode::createProtoInstanceFromParameters(proto, parameters, OmWorld::instance()->fileName(), uniqueId);

  if (!newNode) {
    OmLog::error(tr("Template regeneration failed. The node cannot be generated."), false, OmLog::PARSING);
    delete newNode;
    if (isWorldInitialized)
      emit abortNodeRegeneration();
    return;
  }

  if (mRegeneratingUpperTemplateNode == node)
    mRegeneratingUpperTemplateNode = newNode;  // update reference to base regenerated node

  newNode->setDefName(node->defName());
  OmNode::setGlobalParentNode(NULL);

  OmNodeUtilities::validateInsertedNode(parentField, newNode, parent, isInBoundingObject);

  subscribe(newNode, mNodesSubscribedForRegeneration.contains(node));

  const bool ancestorTemplateRegeneration = upperTemplateNode != NULL;
  if (node->isProtoParameterNode()) {
    // internal PROTO child could be regenerated due to a parameter exposed in the parent PROTO node
    // so for parent PROTO instances both fields and parameters needs to be checked
    const QList<OmField *> parentFields = (parent->isProtoInstance() ? QList(parent->parameters()) : QList<OmField *>())
                                          << parent->fields();
    foreach (OmField *const pf, parentFields) {
      if (pf->type() == WB_SF_NODE) {
        OmSFNode *sfnode = static_cast<OmSFNode *>(pf->value());
        if (sfnode->value() == node) {
          if (ancestorTemplateRegeneration)
            sfnode->blockSignals(true);

          sfnode->setValue(newNode);

          if (ancestorTemplateRegeneration) {
            sfnode->blockSignals(false);
            regenerateNode(upperTemplateNode);
            return;
          }
          break;
        }
      } else if (pf->type() == WB_MF_NODE) {
        OmMFNode *mfnode = static_cast<OmMFNode *>(pf->value());
        bool found = false;
        for (int i = 0; i < mfnode->size(); ++i) {
          const OmNode *n = mfnode->item(i);
          if (n == node) {
            if (ancestorTemplateRegeneration)
              mfnode->blockSignals(true);

            mfnode->removeItem(i);
            mfnode->insertItem(i, newNode);

            if (ancestorTemplateRegeneration) {
              mfnode->blockSignals(false);
              regenerateNode(upperTemplateNode);
              return;
            }
            found = true;
            break;
          }
        }
        if (found) {
          if (parent->isProtoInstance())
            parent->redirectInternalFields(parentField);
          break;
        }
      }
    }
  } else {
    // reassign pointer in parent
    OmSolid *const parentSolid = dynamic_cast<OmSolid *>(parent);
    OmGroup *const parentGroup = dynamic_cast<OmGroup *>(parent);
    OmBasicJoint *const parentJoint = dynamic_cast<OmBasicJoint *>(parent);
    OmShape *const parentShape = dynamic_cast<OmShape *>(parent);
    OmSlot *const parentSlot = dynamic_cast<OmSlot *>(parent);
    OmAppearance *const newAppearance = dynamic_cast<OmAppearance *>(newNode);
    OmPbrAppearance *const newPbrAppearance = dynamic_cast<OmPbrAppearance *>(newNode);
    OmGeometry *const newGeometry = dynamic_cast<OmGeometry *>(newNode);
    OmSlot *const newSlot = dynamic_cast<OmSlot *>(newNode);
    OmSolid *const newSolid = dynamic_cast<OmSolid *>(newNode);
    OmSolidReference *const newSolidReference = dynamic_cast<OmSolidReference *>(newNode);

    if (parentSolid && isInBoundingObject)
      parentSolid->setBoundingObject(newNode);
    else if (parentGroup) {
      int i = parentGroup->nodeIndex(node);
      assert(i != -1);
      parentGroup->setChild(i, newNode);
    } else if (parentShape && newGeometry)
      parentShape->setGeometry(newGeometry);
    else if (parentShape && newAppearance)
      parentShape->setAppearance(newAppearance);
    else if (parentShape && newPbrAppearance)
      parentShape->setPbrAppearance(newPbrAppearance);
    else if (parentSlot)
      parentSlot->setEndPoint(newNode);
    else if (parentJoint && newSolid)
      parentJoint->setSolidEndPoint(newSolid);
    else if (parentJoint && newSolidReference)
      parentJoint->setSolidEndPoint(newSolidReference);
    else if (parentJoint && newSlot)
      parentJoint->setSolidEndPoint(newSlot);
    else {
      OmLog::error(tr("Template regeneration failed. Unsupported node type."), false, OmLog::PARSING);
      delete newNode;
      emit abortNodeRegeneration();
      return;
    }
  }

  // let the supervisor set field functions work as if the node has not been deleted
  newNode->setUniqueId(uniqueId);

  // restore translation and rotation loaded from file
  OmSolid *newSolid = dynamic_cast<OmSolid *>(newNode);
  if (solid && newSolid) {
    newSolid->setTranslationFromFile(translationFromFile);
    newSolid->setRotationFromFile(rotationFromFile);
  }

  // update nested proto flag of current PROTO node and his instances if any
  newNode->updateNestedProtoFlag();

  // redirect parent parameters
  if (!previousParentRedirections.isEmpty()) {
    foreach (OmField *parentParameter, previousParentRedirections) {
      foreach (OmField *newParam, newNode->parameters()) {
        if (parentParameter->name() == newParam->alias()) {
          newParam->blockSignals(true);
          newParam->redirectTo(parentParameter);
          newParam->blockSignals(false);
        }
      }
    }
  }

  mBlockRegeneration = true;  // prevent regenerating `newNode` in the finalization step due to field checks

  OmBaseNode *base = dynamic_cast<OmBaseNode *>(newNode);
  if (isWorldInitialized) {
    assert(base);
    base->finalize();
  }

  mBlockRegeneration = false;
  if (newNode->isRegenerationRequired()) {  // if needed, trigger `newNode` regeneration with finalized fields values
    regenerateNode(newNode, true);
    return;
  }

  // if the viewpoint is being re-generated we need to re-get the correct pointer, not the old dangling pointer from before
  // the node was regenerated
  viewpoint = world->viewpoint();
  if (isFollowedSolid)
    viewpoint->startFollowUp(newSolid, true);
  else if (!followedSolidName.isEmpty() && viewpoint->followedSolid() == NULL)
    // restore follow solid
    viewpoint->startFollowUp(OmSolid::findSolidFromUniqueName(followedSolidName), true);

  cRegeneratingNodeCount--;
  assert(cRegeneratingNodeCount >= 0);
  if (isWorldInitialized) {
    // update dictionary
    mBlockRegeneration = true;  // prevent regenerating `newNode` while updating the dictionary
    if (mRegeneratingUpperTemplateNode == newNode)
      OmDictionary::instance()->setRegeneratedNode(mRegeneratingUpperTemplateNode);
    const bool regenerationRequired = OmNodeOperations::instance()->updateDictionary(false, static_cast<OmBaseNode *>(newNode));
    if (mRegeneratingUpperTemplateNode == newNode)
      OmDictionary::instance()->setRegeneratedNode(NULL);
    mBlockRegeneration = false;
    if (!regenerationRequired)
      emit postNodeRegeneration(newNode);
    else {
      regenerateNode(newNode, true);
      return;
    }
  }

  if (mRegeneratingUpperTemplateNode == newNode)
    mRegeneratingUpperTemplateNode = NULL;
}

void OmTemplateManager::nodeNeedRegeneration() {
  mTemplatesNeedRegeneration = true;
}
