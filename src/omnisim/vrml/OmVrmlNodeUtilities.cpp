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

#include "OmVrmlNodeUtilities.hpp"
#include "OmWorldFileFormat.hpp"

#include "OmField.hpp"
#include "OmMFNode.hpp"
#include "OmNode.hpp"
#include "OmSFNode.hpp"
#include "OmTokenizer.hpp"
#include "OmWriter.hpp"

#include <QtCore/QQueue>

namespace {
  bool checkForUseOrDefNode(const OmNode *node, const QString &useName, const QString &previousUseName, bool &useOverlap,
                            bool &defOverlap, bool &abortSearch);

  bool checkForUseOrDefNode(const OmField *field, const QString &useName, const QString &previousUseName, bool &useOverlap,
                            bool &defOverlap, bool &abortSearch) {
    OmValue *const value = field->value();
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(value);
    if (mfnode) {
      const int size = mfnode->size();
      for (int i = 0; i < size; ++i) {
        const OmNode *const n = mfnode->item(i);
        if (!n)
          continue;

        if (!previousUseName.isEmpty() && n->defName() == previousUseName) {
          abortSearch = true;
          return false;
        }

        if (defOverlap) {
          if (!previousUseName.isEmpty() && n->useName() == previousUseName)
            return true;
        } else if (n->defName() == useName) {
          defOverlap = true;
        } else if (n->useName() == useName) {
          useOverlap = true;
        }

        if (checkForUseOrDefNode(n, useName, previousUseName, useOverlap, defOverlap, abortSearch))
          return true;

        if (abortSearch)
          return false;
      }
      return false;
    } else {
      const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(value);
      if (sfnode) {
        const OmNode *const n = sfnode->value();
        if (n) {
          if (!previousUseName.isEmpty() && n->defName() == previousUseName) {
            abortSearch = true;
            return false;
          }

          if (defOverlap) {
            if (!previousUseName.isEmpty() && n->useName() == previousUseName)
              return true;
          } else if (n->defName() == useName) {
            defOverlap = true;
          } else if (n->useName() == useName) {
            useOverlap = true;
          }

          if (checkForUseOrDefNode(n, useName, previousUseName, useOverlap, defOverlap, abortSearch))
            return true;
        }
      }
    }

    return false;
  }

  bool checkForUseOrDefNode(const OmNode *node, const QString &useName, const QString &previousUseName, bool &useOverlap,
                            bool &defOverlap, bool &abortSearch) {
    // Check fields and parameters
    const QVector<OmField *> &fields = node->fieldsOrParameters();
    for (int i = 0, size = fields.size(); i < size; ++i) {
      if (checkForUseOrDefNode(fields[i], useName, previousUseName, useOverlap, defOverlap, abortSearch))
        return true;
      if (abortSearch)
        return false;
    }
    return false;
  }
}  // namespace

const OmNode *OmVrmlNodeUtilities::findTopNode(const OmNode *node) {
  if (node == NULL || node->isWorldRoot())
    return NULL;

  const OmNode *n = node;
  const OmNode *parent = n->parentNode();
  while (parent) {
    if (parent->isWorldRoot())
      return n;

    n = parent;
    parent = n->parentNode();
  }
  return NULL;
}

bool OmVrmlNodeUtilities::isVisible(const OmNode *node) {
  if (node == NULL)
    return false;

  const OmNode *n = node;
  const OmNode *p = n->parentNode();
  while (n && p && !n->isTopLevel()) {
    if (p->isProtoInstance()) {
      if (p->fields().contains(n->parentField(true)))
        // internal node of a PROTO
        return false;
    }
    n = p;
    p = p->parentNode();
  }
  return true;
}

bool OmVrmlNodeUtilities::isVisible(const OmField *target) {
  if (target == NULL)
    return false;

  const OmField *parameter = target;
  const OmField *parentParameter = target->parameter();

  while (parentParameter) {
    parameter = parentParameter;
    parentParameter = parentParameter->parameter();
  }
  const OmNode *parentNode = parameter->parentNode();
  assert(parentNode);
  if (parentNode->fieldsOrParameters().contains(const_cast<OmField *>(parameter)))
    return isVisible(parentNode);
  return false;
}

bool OmVrmlNodeUtilities::isFieldDescendant(const OmNode *node, const QString &fieldName) {
  if (node == NULL)
    return false;

  const OmNode *n = node->parentNode();
  const OmField *field = node->parentField(true);
  while (n && !n->isWorldRoot() && field) {
    if (field->name() == fieldName)
      return true;

    field = n->parentField(true);
    n = n->parentNode();
  }

  return false;
}

OmField *OmVrmlNodeUtilities::findFieldParent(const OmField *target, bool internal) {
  const OmNode *const nodeParent = target->parentNode();
  assert(nodeParent);
  bool valid = false;
  if (internal)
    valid = nodeParent->fieldsOrParameters().contains(const_cast<OmField *>(target));
  else
    valid = nodeParent->fields().contains(const_cast<OmField *>(target));
  return valid ? nodeParent->parentField() : NULL;
}

OmProtoModel *OmVrmlNodeUtilities::findContainingProto(const OmNode *node) {
  const OmNode *n = node;
  do {
    OmProtoModel *proto = n->proto();
    if (proto)
      return proto;
    else {
      const OmNode *const protoParameterNode = n->protoParameterNode();
      proto = protoParameterNode ? protoParameterNode->proto() : NULL;
      if (proto)
        return proto;

      n = n->parentNode();
    }
  } while (n);
  return NULL;
}

const OmNode *OmVrmlNodeUtilities::findFieldProtoScope(const OmField *field, const OmNode *proto) {
  assert(field);

  if (!proto)
    return NULL;

  const OmNode *node;
  const OmField *candidate = field;
  const OmField *parameter = NULL;
  while (candidate) {
    node = candidate->parentNode();
    if (!node->parentField() && node->protoParameterNode())
      node = node->protoParameterNode();

    parameter = findClosestParameterInProto(candidate, proto);
    if (parameter)
      break;

    candidate = node->parentField();
  }

  if (parameter && !parameter->isDefault())
    return findFieldProtoScope(parameter, proto->containingProto(true));
  else
    return proto;
}

const OmField *OmVrmlNodeUtilities::findClosestParameterInProto(const OmField *field, const OmNode *proto) {
  if (!field || !proto || !proto->proto())
    return NULL;

  const OmNode *parameterNode = proto;
  while (parameterNode) {
    const OmField *parameter = field;
    const QVector<OmField *> parameterList = parameterNode->parameters();

    while (parameter) {
      if (parameterList.contains(const_cast<OmField *>(parameter)))
        return parameter;

      parameter = parameter->parameter();
    }

    parameterNode = parameterNode->protoParameterNode();
  }

  return NULL;
}

OmNode *OmVrmlNodeUtilities::findRootProtoNode(OmNode *const node) {
  OmNode *n = node;
  do {
    const OmProtoModel *proto = n->proto();
    if (proto)
      return n;
    n = n->parentNode();
  } while (n);
  return NULL;
}

QList<const OmNode *> OmVrmlNodeUtilities::protoNodesInWorldFile(const OmNode *root) {
  QList<const OmNode *> result;
  QQueue<const OmNode *> queue;
  queue.append(root);
  while (!queue.isEmpty()) {
    const OmNode *node = queue.dequeue();
    if (node->isProtoInstance())
      result.append(node);
    QVector<OmField *> fields = node->fieldsOrParameters();
    QVectorIterator<OmField *> it(fields);
    while (it.hasNext()) {
      const OmField *field = it.next();
      if (field->isDefault())
        continue;  // ignore default fields that will not be written to file
      const QList<OmNode *> children(node->subNodes(field, false, false, false));
      foreach (OmNode *child, children)
        queue.enqueue(child);
    }
  }

  return result;
}

bool OmVrmlNodeUtilities::existsVisibleProtoNodeNamed(const QString &modelName, const OmNode *root) {
  if (!root)
    return false;

  QQueue<OmNode *> queue;
  queue.append(root->subNodes(false, false, false));
  while (!queue.isEmpty()) {
    const OmNode *node = queue.dequeue();
    if (node->modelName() == modelName)
      return true;
    QVector<OmField *> fields = node->fieldsOrParameters();
    QVectorIterator<OmField *> it(fields);
    while (it.hasNext()) {
      const OmField *field = it.next();
      if (field->isDefault())
        continue;  // ignore default fields that will not be written to file
      queue.append(node->subNodes(field, false, false, false));
    }
  }
  return false;
}

OmNode *OmVrmlNodeUtilities::findUpperTemplateNeedingRegenerationFromField(const OmField *modifiedField, OmNode *parentNode) {
  if (parentNode == NULL || modifiedField == NULL)
    return NULL;

  if (parentNode->isTemplate() && modifiedField->isTemplateRegenerator())
    return parentNode;

  return findUpperTemplateNeedingRegeneration(parentNode);
}

OmNode *OmVrmlNodeUtilities::findUpperTemplateNeedingRegeneration(const OmNode *modifiedNode) {
  if (modifiedNode == NULL)
    return NULL;

  const OmField *field = modifiedNode->parentField();
  OmNode *node = modifiedNode->parentNode();
  while (node && field && !node->isWorldRoot()) {
    if (node->isTemplate() && field->isTemplateRegenerator())
      return node;

    field = node->parentField();
    node = node->parentNode();
  }

  return NULL;
}

bool OmVrmlNodeUtilities::hasAUseNodeAncestor(const OmNode *node) {
  const OmNode *p = node;
  while (p) {
    if (p->isUseNode())
      return true;
    p = p->parentNode();
  }

  return false;
}

bool OmVrmlNodeUtilities::hasASubsequentUseOrDefNode(const OmNode *defNode, const QString &defName,
                                                     const QString &previousDefName, bool &useOverlap, bool &defOverlap) {
  if (defName.isEmpty())
    return false;

  useOverlap = false;
  defOverlap = false;
  bool abortSearch = false;

  if (checkForUseOrDefNode(defNode, defName, previousDefName, useOverlap, defOverlap, abortSearch))
    return true;

  if (abortSearch) {
    defOverlap = false;
    return useOverlap;
  }

  const OmNode *node = defNode;
  const OmNode *parentNode = node->parentNode();

  while (parentNode) {
    const OmField *const parentField = node->parentField();
    const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(parentField->value());
    if (mfnode) {
      const int index = mfnode->nodeIndex(node) + 1;
      const int size = mfnode->size();
      for (int i = index; i < size; ++i) {
        const OmNode *const n = mfnode->item(i);
        if (!previousDefName.isEmpty() && n->defName() == previousDefName) {
          defOverlap = false;
          return useOverlap;
        }

        if (defOverlap) {
          if (!previousDefName.isEmpty() && n->useName() == previousDefName)
            return true;
        } else if (n->defName() == defName) {
          defOverlap = true;
        } else if (n->useName() == defName) {
          useOverlap = true;
        }

        if (checkForUseOrDefNode(n, defName, previousDefName, useOverlap, defOverlap, abortSearch))
          return true;

        if (abortSearch) {
          defOverlap = false;
          return useOverlap;
        }
      }
    }

    const int fieldIndex = parentNode->fieldIndex(parentField) + 1;
    const QVector<OmField *> &fields = parentNode->fieldsOrParameters();
    const int size = fields.size();
    for (int i = fieldIndex; i < size; ++i) {
      OmField *const f = fields[i];
      if (checkForUseOrDefNode(f, defName, previousDefName, useOverlap, defOverlap, abortSearch))
        return true;
      if (abortSearch) {
        defOverlap = false;
        return useOverlap;
      }
    }

    node = parentNode;
    parentNode = parentNode->parentNode();
  }

  return useOverlap;
}

bool OmVrmlNodeUtilities::hasAreferredDefNodeDescendant(const OmNode *node, const OmNode *root) {
  const OmNode *rootNode = root ? root : node;
  const int count = node->useCount();
  const QList<OmNode *> &useNodes = node->useNodes();
  for (int i = 0; i < count; ++i) {
    if (!rootNode->isAnAncestorOf(useNodes.at(i)))
      return true;
  }

  foreach (const OmField *field, node->fieldsOrParameters()) {
    OmValue *value = field->value();
    const OmSFNode *const sfnode = dynamic_cast<OmSFNode *>(value);
    if (sfnode && sfnode->value()) {
      const OmNode *childNode = sfnode->value();
      const int nodeCount = childNode->useCount();
      const QList<OmNode *> &nodeUseNodes = childNode->useNodes();
      for (int i = 0; i < nodeCount; ++i) {
        if (!rootNode->isAnAncestorOf(nodeUseNodes.at(i)))
          return true;
      }
      const bool subtreeHasDef = hasAreferredDefNodeDescendant(childNode, rootNode);
      if (subtreeHasDef)
        return subtreeHasDef;
    } else {
      const OmMFNode *const mfnode = dynamic_cast<OmMFNode *>(value);
      if (mfnode) {
        const int size = mfnode->size();
        for (int i = 0; i < size; ++i) {
          const OmNode *childNode = mfnode->item(i);
          const int nodeCount = childNode->useCount();
          const QList<OmNode *> &nodeUseNodes = childNode->useNodes();
          for (int j = 0; j < nodeCount; ++j) {
            if (!rootNode->isAnAncestorOf(nodeUseNodes.at(j)))
              return true;
          }
          const bool subtreeHasDef = hasAreferredDefNodeDescendant(childNode, rootNode);
          if (subtreeHasDef)
            return subtreeHasDef;
        }
      }
    }
  }

  return false;
}

QList<OmNode *> OmVrmlNodeUtilities::findUseNodeAncestors(OmNode *node) {
  QList<OmNode *> list;

  if (node == NULL)
    return list;

  // cppcheck-suppress constVariablePointer
  OmNode *n = node;
  while (n && !n->isWorldRoot()) {
    if (n->isUseNode())
      list.prepend(n);
    n = n->parentNode();
  }

  return list;
}

QString OmVrmlNodeUtilities::exportNodeToString(const OmNode *node) {
  QString nodeString;
  OmWriter writer(&nodeString, OmWorldFileFormat::writeExtension());
  node->write(writer);
  return nodeString;
}

// Return true if we can convert the Transform to a Pose.
bool OmVrmlNodeUtilities::transformBackwardCompatibility(OmTokenizer *tokenizer) {
  if (!tokenizer)
    return true;

  const int initalIndex = tokenizer->pos();
  bool inChildren = false;
  int bracketCount = 0;
  while (tokenizer->hasMoreTokens()) {
    const QString token = tokenizer->nextWord();
    if (inChildren) {
      if (token == "[")
        bracketCount++;
      else if (token == "]") {
        bracketCount--;
        if (bracketCount == 0)
          inChildren = false;
      }
    } else if (token == "children") {
      inChildren = true;
    } else if (token == "scale") {
      for (int i = 0; i < 3; i++) {
        if (tokenizer->nextWord().toFloat() != 1.0f) {
          tokenizer->seek(initalIndex);
          return false;
        }
      }
      // We have identified that the scale is the default one.
      break;
      // End of the Transform
    } else if (token == "}")
      break;
  }
  tokenizer->seek(initalIndex);

  return true;
}
