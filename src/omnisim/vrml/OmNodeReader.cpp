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

#include "OmNodeReader.hpp"

#include "OmNode.hpp"
#include "OmNodeFactory.hpp"
#include "OmNodeModel.hpp"
#include "OmParser.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmVrmlNodeUtilities.hpp"

#include <QtCore/QStack>
#include <cassert>

static QStack<OmNodeReader *> gCallStack;

OmNodeReader *OmNodeReader::current() {
  if (gCallStack.isEmpty())
    return NULL;

  return gCallStack.top();
}

OmNodeReader::OmNodeReader(Mode mode) : mMode(mode), mIsReadingBoundingObject(false), mReadNodesCanceled(false) {
  gCallStack.push(this);
}

OmNodeReader::~OmNodeReader() {
  assert(!gCallStack.isEmpty());
  gCallStack.pop();
}

OmNode *OmNodeReader::createNode(const QString &modelName, OmTokenizer *tokenizer, const QString &worldPath,
                                 const QString &fileName) {
  if (mMode == NORMAL)
    return OmNodeFactory::instance()->createNode(OmNodeModel::compatibleNodeName(modelName), tokenizer);

  if (modelName == "Transform")
    return OmVrmlNodeUtilities::transformBackwardCompatibility(tokenizer) ? new OmNode("Pose", worldPath, tokenizer) :
                                                                            new OmNode("Transform", worldPath, tokenizer);
  else {
    if (OmNodeModel::findModel(modelName))
      return new OmNode(modelName, worldPath, tokenizer);
  }
  OmProtoModel *const proto = OmProtoManager::instance()->findModel(modelName, worldPath, fileName);
  if (proto)
    return OmNode::createProtoInstance(proto, tokenizer, worldPath);

  tokenizer->reportError(QObject::tr("Skipped unknown '%1' node or PROTO").arg(modelName));
  return NULL;
}

OmNode *OmNodeReader::readNode(OmTokenizer *tokenizer, const QString &worldPath) {
  if (tokenizer->peekWord() == "NULL") {
    tokenizer->skipToken("NULL");
    return NULL;
  }

  if (tokenizer->peekWord() == "USE") {
    tokenizer->skipToken("USE");
    const QString &useName = tokenizer->nextWord();

    // find USE name
    OmNode *const defNode = mDefs.value(useName, NULL);
    if (!defNode) {
      tokenizer->reportError(QObject::tr("Did not find a 'DEF %1' to match 'USE %1'").arg(useName));
      return NULL;
    }

    OmNode *const useNode = defNode->cloneDefNode();
    useNode->makeUseNode(defNode, true);
    return useNode;
  }

  QString defName;
  const OmNode *previousDefNode;
  if (tokenizer->peekWord() == "DEF") {
    tokenizer->skipToken("DEF");
    defName = tokenizer->nextWord();
    previousDefNode = mDefs.value(defName, NULL);
  } else
    previousDefNode = NULL;

  const QString &modelName = tokenizer->nextWord();
  const QString &parentFilePath = tokenizer->fileName().isEmpty() ? tokenizer->referralFile() : tokenizer->fileName();
  OmNode *const node = createNode(OmNodeModel::compatibleNodeName(modelName), tokenizer, worldPath, parentFilePath);
  if (!node) {
    if (tokenizer->lastWord() != "}")
      tokenizer->skipNode();
    return NULL;
  }

  if (!defName.isEmpty()) {
    node->setDefName(defName);
    if (previousDefNode == mDefs.value(defName, NULL))
      // check if descendant nodes have the same DEF name
      // if it is the case, then we should not overwrite the value
      addDefNode(node);
  }

  return node;
}

QList<OmNode *> OmNodeReader::readNodes(OmTokenizer *tokenizer, const QString &worldPath) {
  tokenizer->rewind();

  OmParser parser(tokenizer);
  while (tokenizer->peekWord() == "EXTERNPROTO" || tokenizer->peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
    parser.skipExternProto();

  QList<OmNode *> nodes;
  while (!tokenizer->peekToken()->isEof()) {
    emit readNodesHasProgressed(100 * tokenizer->pos() / tokenizer->totalTokensNumber());
    if (mReadNodesCanceled) {
      mReadNodesCanceled = false;
      return nodes;
    }
    // cppcheck-suppress constVariablePointer
    OmNode *node = readNode(tokenizer, worldPath);
    if (node)
      nodes.append(node);
  }

  return nodes;
}

void OmNodeReader::cancelReadNodes() {
  mReadNodesCanceled = true;
}

void OmNodeReader::addDefNode(OmNode *defNode) {
  mDefs.insert(defNode->defName(), defNode);
}

void OmNodeReader::removeDefNode(OmNode *defNode) {
  if (mDefs.value(defNode->defName()) == defNode)
    mDefs.remove(defNode->defName());
}
