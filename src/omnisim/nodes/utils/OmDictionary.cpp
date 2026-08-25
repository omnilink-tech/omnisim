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

#include "OmDictionary.hpp"

#include "OmBasicJoint.hpp"
#include "OmField.hpp"
#include "OmGroup.hpp"
#include "OmJointParameters.hpp"
#include "OmLogicalDevice.hpp"
#include "OmMFNode.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSFNode.hpp"
#include "OmSolid.hpp"
#include "OmWorld.hpp"

#include "../../../include/controller/c/omnisim/nodes.h"

#include <QtCore/QSet>
#include <cassert>

typedef QMultiMap<QString, OmNode *> Dictionary;

OmDictionary *OmDictionary::cInstance = NULL;

OmDictionary *OmDictionary::instance() {
  if (!cInstance)
    cInstance = new OmDictionary();
  return cInstance;
}

void OmDictionary::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

OmDictionary::OmDictionary() :
  mTargetNode(NULL),
  mTargetField(NULL),
  mTargetIndex(-1),
  mStopUpdate(false),
  mLoad(false),
  mCurrentProtoRegeneration(false),
  mCurrentProtoRegenerationNode(NULL) {
}

OmDictionary::~OmDictionary() {
}

/////////////////////////////////////////
// Updates of the DEF names dictionary //
/////////////////////////////////////////

void OmDictionary::setRegeneratedNode(const OmNode *node) {
  assert(!node || !mCurrentProtoRegenerationNode);  // nested dictionary updates should be avoided
  mCurrentProtoRegenerationNode = node;
}

bool OmDictionary::update(bool load) {
  mLoad = load;
  clearNestedDictionaries();
  mSceneDictionary.clear();
  assert(!mCurrentProtoRegeneration);  // nested dictionary updates should be avoided
  mCurrentProtoRegeneration = false;
  bool regenerationRequired = false;

  OmBaseNode *rootNode = OmWorld::instance()->root();
  updateDef(rootNode, NULL, NULL, -1, false, regenerationRequired);

  mCurrentProtoRegeneration = false;
  return regenerationRequired;
}

bool OmDictionary::updateDef(OmBaseNode *&node, OmSFNode *sfNode, OmMFNode *mfNode, int index, bool isTemplateRegenerator,
                             bool &regenerationRequired) {
  const QString &defName = node->defName();
  const QString &useName = node->useName();
  const bool useCase = !useName.isEmpty();
  const int useNestingDegree = mNestedDictionaries.size() - 1;
  mCurrentProtoRegeneration |= node == mCurrentProtoRegenerationNode;

  // Solid, Device, JointParameters and BasicJoint DEF nodes are allowed but not registered in the dictionary,
  // Solid, Device, JointParameters and BasicJoint USE nodes are prohibited
  // Charger and LED USE nodes in the first child have to link to DEF nodes in the first child

  if (!defName.isEmpty() && mNestedDictionaries.size() == 1)
    mSceneDictionary.append(std::pair<OmNode *, QString>(node, defName));

  QString warning;
  const bool isAValidUseableNode =
    (!defName.isEmpty() || useCase) && OmNodeUtilities::isAValidUseableNode(node, &warning);  // if false, generates warning

  if (!useCase && isAValidUseableNode) {
    assert(!mNestedDictionaries.isEmpty());
    mNestedDictionaries.last().insert(defName, node);
  } else if (useCase) {
    if (isAValidUseableNode) {
      const OmNode::NodeUse nodeUse = node->nodeUse();
      bool match = false;
      bool typeMatch = false;
      OmBaseNode *definitionNode = NULL;
      const int ind = useNestingDegree - 1;
      assert(ind >= 0 && ind < mNestedDictionaries.size());
      const QList<OmNode *> &defNodes = mNestedDictionaries.at(ind).values(useName);
      const int numberOfDefs = defNodes.size();
      // cppcheck-suppress knownConditionTrueFalse
      for (int defIndex = 0; !match && defIndex < numberOfDefs; ++defIndex) {
        definitionNode = static_cast<OmBaseNode *>(defNodes[defIndex]);
        QString error;
        assert(node->parentField() && node->parentNode());
        typeMatch = OmNodeUtilities::isAllowedToInsert(node->parentField(), node->parentNode(), error, nodeUse, QString(),
                                                       definitionNode);
        match = typeMatch && !definitionNode->isAnAncestorOf(node);
      }

      OmNode *matchingNode = NULL;
      // cppcheck-suppress knownConditionTrueFalse
      if (!match && !mNestedUseNodes.isEmpty()) {
        definitionNode = NULL;
        const OmNode *const upperUseNode = mNestedUseNodes.last();
        OmNode *const upperDefinition = upperUseNode->defNode();
        const int childIndex = OmNode::subNodeIndex(node, upperUseNode);
        matchingNode = OmNode::findNodeFromSubNodeIndex(childIndex, upperDefinition);
        if (matchingNode && matchingNode->isUseNode()) {
          definitionNode = static_cast<OmBaseNode *>(matchingNode->defNode());
          QString error;
          assert(node->parentField() && node->parentNode());
          typeMatch = OmNodeUtilities::isAllowedToInsert(node->parentField(), node->parentNode(), error, nodeUse, QString(),
                                                         definitionNode);
        }
      }

      if (definitionNode && typeMatch && node->defNode() != definitionNode) {
        QString deviceModelName;
        if (node->isInBoundingObject() != definitionNode->isInBoundingObject() &&
            !checkBoundingObjectConstraints(definitionNode, warning)) {
          node->parentNode()->parsingWarn(QObject::tr("Deleted invalid USE %1 node: %3").arg(useName).arg(warning));
          OmNodeOperations::instance()->deleteNode(node);
          if (node == mCurrentProtoRegenerationNode)
            mCurrentProtoRegeneration = false;
          return false;
        } else if (!checkChargerAndLedConstraints(node->parentNode(), definitionNode, deviceModelName, index == 0)) {
          node->parsingWarn(
            QObject::tr("Non-admissible USE %1 node inside first child of %2 node.\n"
                        "Invalid USE nodes that refer to DEF nodes defined outside the %2 node are turned into DEF nodes "
                        "otherwise the emissive color cannot be updated correctly.")
              .arg(useName)
              .arg(deviceModelName));
          makeDefNodeAndUpdateDictionary(node, true);
        } else {
          if (!mLoad && !mCurrentProtoRegeneration) {
            OmNode *parent = node->parentNode();
            OmNode::setGlobalParentNode(parent);
            OmBaseNode *const newUseNode = static_cast<OmBaseNode *>(definitionNode->cloneDefNode());
            OmNode::setGlobalParentNode(NULL);
            newUseNode->setUseName(useName);  // Deactivates the creation of children items triggered by insertion
            if (sfNode)
              sfNode->setValue(newUseNode);
            else if (mfNode) {
              mfNode->removeItem(index);
              mfNode->insertItem(index, newUseNode);  // TODO: replace by setItem(index, newUseNode) when it is fixed
            }
            regenerationRequired |= isTemplateRegenerator;
            while (parent) {
              parent = parent->parentNode();
            }
            newUseNode->finalize();
            node = newUseNode;
          }
          node->makeUseNode(definitionNode);  // Sets USE name, DEF reference, unregisters from previous DEF reference and
                                              // registers to the new one
          mNestedUseNodes.append(node);
        }
      }

      if (matchingNode && matchingNode->isDefNode()) {
        if (!mLoad) {
          OmNode *parent = node->parentNode();
          OmNode::setGlobalParentNode(parent);
          OmBaseNode *const newDefNode = static_cast<OmBaseNode *>(matchingNode->cloneAndReferenceProtoInstance());
          newDefNode->setUseName(useName);  // Deactivates the creation of children items triggered by insertion
          if (sfNode)
            sfNode->setValue(newDefNode);
          else if (mfNode) {
            mfNode->removeItem(index);
            mfNode->insertItem(index, newDefNode);  // TODO: replace by setItem(index, newUseNode) when it is fixed
          }
          regenerationRequired |= isTemplateRegenerator;
          newDefNode->finalize();
          node = newDefNode;
        }
      }

      if (!definitionNode || !typeMatch) {
        if (useNestingDegree == 1) {
          if (!typeMatch)
            node->parsingWarn(
              QObject::tr("Previous DEF node cannot be used to replace the current USE node; USE node turned into DEF node."));
          else
            node->parsingWarn(QObject::tr("No previous DEF nodes match; USE node turned into DEF node."));
        }
        makeDefNodeAndUpdateDictionary(node, true);
      }

    } else {
      node->parsingWarn(warning + " " + QObject::tr("Non-admissible USE node turned into DEF node."));
      makeDefNodeAndUpdateDictionary(node, true);
    }

    if (node == mCurrentProtoRegenerationNode)
      mCurrentProtoRegeneration = false;
    return true;
  }

  const QVector<OmField *> &fields = node->fieldsOrParameters();
  foreach (const OmField *const field, fields) {
    OmValue *const value = field->value();
    OmSFNode *const sf = dynamic_cast<OmSFNode *>(value);
    if (sf) {
      OmBaseNode *n = static_cast<OmBaseNode *>(sf->value());
      if (n) {
#ifndef NDEBUG
        const int nestedDictionariesSize = mNestedDictionaries.size();
#endif

        const bool createDictionary = n->isUseNode();
        if (createDictionary)  // Appends a local dictionary which is limited to the scope of this USE node
          mNestedDictionaries.append(Dictionary());

        bool success = updateDef(n, sf, NULL, -1, field->isTemplateRegenerator(), regenerationRequired);

        // dictionary already removed if USE node has been turned into DEF node
        if (createDictionary && (!success || n->isUseNode())) {
          mNestedDictionaries.removeLast();
          if (!mNestedUseNodes.isEmpty())
            mNestedUseNodes.removeLast();
        }
        assert(nestedDictionariesSize == mNestedDictionaries.size());
      }
    } else {
      OmMFNode *const mf = dynamic_cast<OmMFNode *>(value);
      if (mf) {
        const int size = mf->size();
        for (int i = 0; i < size; ++i) {
          OmBaseNode *n = static_cast<OmBaseNode *>(mf->item(i));
          if (n) {
#ifndef NDEBUG
            const int nestedDictionariesSize = mNestedDictionaries.size();
#endif

            const bool createDictionary = n->isUseNode();
            if (createDictionary)  // Appends a local dictionary which is limited to the scope of this USE node
              mNestedDictionaries.append(Dictionary());

            bool success = updateDef(n, NULL, mf, i, field->isTemplateRegenerator(), regenerationRequired);

            // dictionary already removed if USE node has been turned into DEF node
            if (createDictionary && (!success || n->isUseNode())) {
              mNestedDictionaries.removeLast();
              if (!mNestedUseNodes.isEmpty())
                mNestedUseNodes.removeLast();
            }
            assert(nestedDictionariesSize == mNestedDictionaries.size());
          }
        }
      }
    }
  }
  if (node == mCurrentProtoRegenerationNode)
    mCurrentProtoRegeneration = false;
  return true;
}

void OmDictionary::updateProtosPrivateDef(OmBaseNode *&node) {
  mNestedProtos.clear();
  clearNestedDictionaries();
  if (node->isProtoInstance()) {
    mNestedProtos.append(node);
    mNestedDictionaries.append(Dictionary());
  }
  updateProtosDef(node);
}

void OmDictionary::updateProtosDef(OmBaseNode *&node, OmSFNode *sfNode, OmMFNode *mfNode, int index) {
  const QString &defName = node->defName();

  // Solid, Device, BasicJoint and JointParameters DEF nodes are allowed but not registered in the dictionary,
  // Solid, Device, BasicJoint and JointParameters USE nodes are prohibited
  QString warning;
  const bool isAValidUseableNode = OmNodeUtilities::isAValidUseableNode(node, &warning);

  // Handles nodes in protos
  if (mNestedProtos.size() > 0) {
    const int nestingDegree = mNestedDictionaries.size() - 1;
    int lookupDegree = nestingDegree;
    if (!defName.isEmpty() && isAValidUseableNode) {
      if (node->isProtoInstance())
        lookupDegree--;
      mNestedDictionaries[lookupDegree].insert(defName, node);
    } else {
      const QString &useName = node->useName();
      const bool useCase = !useName.isEmpty();
      lookupDegree--;
      if (useCase && lookupDegree >= 0 && isAValidUseableNode) {
        bool match = false;
        OmBaseNode *definitionNode = NULL;
        const QList<OmNode *> &defNodes = mNestedDictionaries.at(lookupDegree).values(useName);
        const int numberOfDefs = defNodes.size();
        for (int defIndex = 0; (!match) && defIndex < numberOfDefs; ++defIndex) {
          definitionNode = static_cast<OmBaseNode *>(defNodes[defIndex]);
          match = (node->nodeType() == definitionNode->nodeType()) && !definitionNode->isAnAncestorOf(node);
        }

        if (!match && !mNestedUseNodes.isEmpty()) {
          const OmNode *const upperUseNode = mNestedUseNodes.last();
          OmNode *upperDefinition = upperUseNode->defNode();
          if (mNestedProtos.last()->isAnAncestorOf(upperDefinition)) {
            int childIndex = OmNode::subNodeIndex(node, upperUseNode);
            const OmNode *matchingUse = OmNode::findNodeFromSubNodeIndex(childIndex, upperDefinition);
            definitionNode = static_cast<OmBaseNode *>(matchingUse->defNode());
          }
        }

        if (definitionNode) {
          if (node->defNode() != definitionNode)
            node->makeUseNode(definitionNode);
          mNestedUseNodes.append(node);
        } else {
          node->parsingWarn(
            QObject::tr("No previous DEF nodes match; USE node turned into DEF node. Please check that the fields are "
                        "listed in the same order as in the base node definition."));
          makeDefNodeAndUpdateDictionary(node, false);
        }
      } else if (useCase && lookupDegree >= 0) {
        if (!isAValidUseableNode)
          node->parsingWarn(
            warning + " " +
            QObject::tr("Please replace this non-admissible USE node by an expanded DEF node in your proto file."));
        makeDefNodeAndUpdateDictionary(node, false);
      }
    }
  }

  if (node->isUseNode())
    // do not check validity of USE nodes in USE nodes
    // because it has already been checked for the referenced DEF node
    return;

  // Handles non-parameter fields only
  const QVector<OmField *> &fields = node->fields();
  foreach (const OmField *const field, fields) {
    if (field->parameter())
      continue;

    OmValue *const value = field->value();
    OmSFNode *const sf = dynamic_cast<OmSFNode *>(value);
    if (sf) {
      OmBaseNode *n = static_cast<OmBaseNode *>(sf->value());
      if (n) {
        const bool isUseNode = n->isUseNode();
        const bool isProtoInstance = n->isProtoInstance();
        const bool localDictionary = isUseNode || isProtoInstance;

        if (isProtoInstance)
          mNestedProtos.append(n);

        if (localDictionary)  // Appends a local dictionary which is limited to the scope of this USE node / proto instance
          mNestedDictionaries.append(Dictionary());

        updateProtosDef(n, sf);

        assert(isProtoInstance == n->isProtoInstance());

        // dictionary already removed if USE node has been turned into DEF node
        if (localDictionary && !(isUseNode && !n->isUseNode())) {
          assert(!mNestedDictionaries.isEmpty());
          mNestedDictionaries.removeLast();
        }

        if (isProtoInstance)
          mNestedProtos.removeLast();

        if (isUseNode && !mNestedUseNodes.isEmpty())
          mNestedUseNodes.removeLast();
      }
    } else {
      OmMFNode *const mf = dynamic_cast<OmMFNode *>(value);
      if (mf) {
        const int size = mf->size();
        for (int i = 0; i < size; ++i) {
          OmBaseNode *n = static_cast<OmBaseNode *>(mf->item(i));
          if (n) {
            const bool isUseNode = n->isUseNode();
            const bool isProtoInstance = n->isProtoInstance();
            const bool localDictionary = isUseNode || isProtoInstance;

            if (isProtoInstance)
              mNestedProtos.append(n);

            if (localDictionary)  // Appends a local dictionary which is limited to the scope of this USE node / proto instance
              mNestedDictionaries.append(Dictionary());

            updateProtosDef(n, NULL, mf, i);

            assert(n->isProtoInstance() == isProtoInstance);

            // dictionary already removed if USE node has been turned into DEF node
            if (localDictionary && !(isUseNode && !n->isUseNode())) {
              assert(!mNestedDictionaries.isEmpty());
              mNestedDictionaries.removeLast();
            }

            if (isProtoInstance)
              mNestedProtos.removeLast();

            if (isUseNode && !mNestedUseNodes.isEmpty())
              mNestedUseNodes.removeLast();
          }
        }
      }
    }
  }
}

void OmDictionary::makeDefNodeAndUpdateDictionary(OmBaseNode *node, bool updateSceneDictionary) {
  const QString &useName = node->useName();
  node->makeDefNode();
  assert(mNestedDictionaries.size() >= 2);
  mNestedDictionaries.removeLast();  // remove USE node local dictionary
  mNestedDictionaries.last().insert(useName, node);
  if (updateSceneDictionary && mNestedDictionaries.size() == 1)
    mSceneDictionary.append(std::pair<OmNode *, QString>(node, node->defName()));
}

bool OmDictionary::checkBoundingObjectConstraints(const OmBaseNode *defNode, QString &errorMessage) {
  OmNode::NodeUse nodeUse = OmNodeUtilities::checkNodeUse(defNode);
  if (nodeUse == OmNode::BOTH_USE)
    return true;
  // assuming that DEF node is valid check other case
  if (nodeUse & OmNode::BOUNDING_OBJECT_USE)
    nodeUse = OmNode::STRUCTURE_USE;
  else
    nodeUse = OmNode::BOUNDING_OBJECT_USE;
  QList<const OmNode *> subNodes;
  subNodes << defNode;
  while (!subNodes.isEmpty()) {
    const OmNode *const parentNode = subNodes.takeFirst();
    const QVector<OmField *> &fields = parentNode->fields();
    for (int i = 0, size = fields.size(); i < size; ++i) {
      const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(fields[i]->value());
      if (sfnode) {
        const OmNode *const n = sfnode->value();
        if (n) {
          if (!OmNodeUtilities::isAllowedToInsert(fields[i], parentNode, errorMessage, nodeUse, QString(), n, false))
            return false;
          subNodes << n;
        }
      } else {
        const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(fields[i]->value());
        if (mfnode) {
          const int size = mfnode->size();
          for (int j = 0; j < size; ++j) {
            const OmNode *const n = mfnode->item(j);
            if (n) {
              if (!OmNodeUtilities::isAllowedToInsert(fields[i], parentNode, errorMessage, nodeUse, QString(), n, false))
                return false;

              subNodes << n;
            }
          }
        }
      }
    }
  }

  return true;
}

bool OmDictionary::checkChargerAndLedConstraints(OmNode *useNodeParent, const OmBaseNode *defNode, QString &deviceModelName,
                                                 bool isFirstChild) {
  // In case of Material or Light USE node inserted in first child of Charger or LED nodes:
  // the corresponding DEF node has also to be a descendant of first child
  OmNode *upperLedOrCharger;
  const OmBaseNode *parentBaseNode = dynamic_cast<OmBaseNode *>(useNodeParent);
  if (parentBaseNode->nodeType() == WB_NODE_LED)
    upperLedOrCharger = useNodeParent;
  else
    upperLedOrCharger = OmNodeUtilities::findUpperNodeByType(useNodeParent, WB_NODE_LED);
  if (!upperLedOrCharger) {
    if (parentBaseNode->nodeType() == WB_NODE_CHARGER)
      upperLedOrCharger = useNodeParent;
    else
      upperLedOrCharger = OmNodeUtilities::findUpperNodeByType(useNodeParent, WB_NODE_CHARGER);
    deviceModelName = "Charger";
  } else
    deviceModelName = "LED";
  if (!upperLedOrCharger) {
    deviceModelName = "";
    return true;
  }

  QList<int> types;
  types << WB_NODE_MATERIAL << WB_NODE_POINT_LIGHT << WB_NODE_SPOT_LIGHT << WB_NODE_DIRECTIONAL_LIGHT;
  if (!types.contains(defNode->nodeType()) && !OmNodeUtilities::hasDescendantNodesOfType(defNode, types))
    return true;

  // cppcheck-suppress constVariablePointer
  OmNode *firstChild = dynamic_cast<OmGroup *>(upperLedOrCharger)->child(0);
  QList<OmNode *> firstChildDescendants = firstChild->subNodes(true);
  firstChildDescendants.prepend(firstChild);
  if (isFirstChild)
    firstChildDescendants.prepend(upperLedOrCharger);
  return !firstChildDescendants.contains(useNodeParent) || firstChildDescendants.contains(const_cast<OmBaseNode *>(defNode));
}

/////////////////////////////////
// Update related to insertion //
/////////////////////////////////

bool OmDictionary::isSuitable(const OmNode *defNode, const QString &type) const {
  assert(mTargetField);
  QString errorMessage;
  const OmNode::NodeUse targetNodeUse = static_cast<OmBaseNode *>(mTargetNode)->nodeUse();
  if (!OmNodeUtilities::isAllowedToInsert(mTargetField, mTargetNode, errorMessage, targetNodeUse, type, defNode, true))
    return false;

  const OmBaseNode *defBaseNode = dynamic_cast<const OmBaseNode *>(defNode);

  // recheck validity of DEF node and subnodes if the USE is used in a different context (boundingObject or not)
  if (((mTargetField->name() == "boundingObject" && defBaseNode->nodeUse() & OmNode::STRUCTURE_USE) ||
       (targetNodeUse != defBaseNode->nodeUse() && mTargetField->name() != "boundingObject")) &&
      !checkBoundingObjectConstraints(defBaseNode, errorMessage))
    return false;

  // check special Charger and LED case
  QString deviceModelName;
  return checkChargerAndLedConstraints(mTargetNode, defBaseNode, deviceModelName, mTargetIndex == 0);
}

QList<OmNode *> OmDictionary::computeDefForInsertion(OmNode *const targetNode, OmField *const targetField, int targetIndex,
                                                     bool suitableOnly) {
  clearNestedDictionaries();
  mTargetNode = targetNode;
  mTargetField = targetField;
  mTargetIndex = targetIndex;
  mStopUpdate = false;

  QList<OmNode *> defNodes;
  updateForInsertion(OmWorld::instance()->root(), suitableOnly, defNodes);
  return defNodes;
}

void OmDictionary::updateForInsertion(const OmNode *const node, bool suitableOnly, QList<OmNode *> &defNodes) {
  if (!mStopUpdate) {
    const QString &definitionName = node->defName();
    if (!definitionName.isEmpty()) {
      // check if a node with the same DEF name is already in the list and remove it
      QMutableListIterator<OmNode *> it(defNodes);
      it.toBack();
      while (it.hasPrevious()) {
        if (it.previous()->defName() == definitionName) {
          it.remove();
          break;  // only one instance with same DEF name is included in the list
        }
      }

      // Solid, Device, BasicJoint and JointParameters USE nodes are prohibited
      if (node != mTargetNode && OmNodeUtilities::isAValidUseableNode(node) &&
          (!suitableOnly || isSuitable(node, OmNodeUtilities::slotType(node))) && !node->isAnAncestorOf(mTargetNode))
        defNodes.append(const_cast<OmNode *>(node));
    }
  }

  // Check fields and parameters
  const QVector<OmField *> &fields = node->fieldsOrParameters();
  for (int i = 0, size = fields.size(); !mStopUpdate && i < size; ++i) {
    bool isInsertionField = node == mTargetNode && fields[i] == mTargetField;
    if (isInsertionField && mTargetIndex < 1) {
      mStopUpdate = true;
      return;
    }

    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(fields[i]->value());
    if (sfnode) {
      const OmNode *const n = sfnode->value();
      if (n && !n->isUseNode())
        updateForInsertion(n, suitableOnly, defNodes);
    } else {
      const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(fields[i]->value());
      if (mfnode) {
        const int size = mfnode->size();
        for (int j = 0; !mStopUpdate && j < size; ++j) {
          if (isInsertionField && j >= mTargetIndex) {
            mStopUpdate = true;
            return;
          }

          const OmNode *const n = mfnode->item(j);
          if (n && !n->isUseNode())
            updateForInsertion(n, suitableOnly, defNodes);
        }
      }  // else no node to check
    }
  }

  mStopUpdate = mStopUpdate || (node == mTargetNode);
}

OmNode *OmDictionary::getNodeFromDEF(const QString &defName) const {
  const int size = mSceneDictionary.size();
  for (int i = 0; i < size; ++i) {
    const std::pair<OmNode *, QString> entry = mSceneDictionary.at(i);
    if (entry.second == defName)
      return entry.first;
  }
  return NULL;
}

void OmDictionary::updateNodeDefName(OmNode *node, bool fromUseToDef) {
  if (fromUseToDef || !node->useName().isEmpty())
    // ignore USE nodes updates or USE nodes just turned into DEF
    return;

  const int size = mSceneDictionary.size();
  for (int i = 0; i < size; ++i) {
    std::pair<OmNode *, QString> &entry = mSceneDictionary[i];
    if (entry.first == node) {
      if (node->defName().isEmpty())
        mSceneDictionary.removeAt(i);
      else
        entry.second = node->defName();
      return;
    }
  }

  // DEF name previously empty: we need to recompute the dictionary
  update(false);
}

void OmDictionary::removeNodeFromDictionary(const OmNode *node) {
  if (node->useCount() > 0)
    // dictionary will be completely recomputed
    return;

  QMutableListIterator<std::pair<OmNode *, QString>> it(mSceneDictionary);
  while (it.hasNext()) {
    if (it.next().first == node) {
      it.remove();
      return;
    }
  }
}
