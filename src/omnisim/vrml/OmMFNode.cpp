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

#include "OmMFNode.hpp"
#include "OmNode.hpp"
#include "OmNodeReader.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmWriter.hpp"

#include <cassert>

OmMFNode::OmMFNode(const OmMFNode &other) {
  foreach (OmNode *const node, other.mVector) {
    OmNode *const copy = node->cloneAndReferenceProtoInstance();
    assert(copy);
    mVector.append(copy);
    copy->setInsertionCompleted();
  }
}

OmMFNode::~OmMFNode() {
  // qDeleteAll(mVector);  // Delete always USE nodes before DEF nodes
  const int n = mVector.size() - 1;
  for (int i = n; i >= 0; --i)
    delete mVector[i];
}

void OmMFNode::readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) {
  OmNode *node;
  if (OmNodeReader::current())
    // reading a regular list of nodes (file scope)
    node = OmNodeReader::current()->readNode(tokenizer, worldPath);
  else {
    // reading a default proto parameter (private scope)
    OmNodeReader reader(OmNodeReader::PROTO_MODEL);
    node = reader.readNode(tokenizer, worldPath);
  }
  if (node) {
    mVector.append(node);
    node->setInsertionCompleted();
  }
}

void OmMFNode::clear() {
  if (mVector.empty())
    return;

  QVector<OmNode *> tmp = mVector;
  mVector.clear();
  emit changed();

  // We don't want to use qDeleteAll(tmp) because we need to delete USE nodes before DEF nodes
  const int n = tmp.size() - 1;
  for (int i = n; i >= 0; --i)
    delete tmp[i];
}

void OmMFNode::insertDefaultItem(int index) {
  assert(index >= 0 && index <= size());
  mVector.insert(index, defaultNewVariant().toNode());
  emit changed();
}

void OmMFNode::removeItem(int index) {
  assert(index >= 0 && index < size());
  OmNode *const tmp = mVector[index];
  mVector.remove(index);
  emit itemRemoved(index);
  emit changed();
  delete tmp;
}

bool OmMFNode::removeNode(OmNode *node) {
  const int index = mVector.indexOf(node);
  if (index == -1)
    return false;

  mVector.remove(index);
  emit itemRemoved(index);
  emit changed();
  return true;
}

void OmMFNode::setItem(int index, OmNode *node) {
  assert(index >= 0 && index < size());
  // NULL nodes are illegal in an MFNode
  assert(node);

  if (mVector[index] != node) {
    OmNode *const tmp = mVector[index];
    mVector[index] = node;
    node->setInsertionCompleted();
    emit itemChanged(index);
    emit changed();
    delete tmp;
  }
}

void OmMFNode::addItem(OmNode *node) {
  // NULL nodes are illegal in an MFNode
  assert(node);

  mVector.append(node);
  node->setInsertionCompleted();
  emit itemInserted(mVector.size() - 1);  // warning: inserted item may not be finalized
  emit changed();
}

void OmMFNode::insertItem(int index, OmNode *node) {
  assert(index >= 0 && index <= size());
  // NULL nodes are illegal in an MFNode
  assert(node);

  mVector.insert(index, node);
  node->setInsertionCompleted();
  emit itemInserted(index);
  emit changed();
}

OmMFNode &OmMFNode::operator=(const OmMFNode &other) {
  if (mVector == other.mVector)
    return *this;

  while (mVector.size() > 0)
    removeItem(0);

  const int m = other.mVector.size();
  for (int i = 0; i < m; ++i) {
    OmNode *const copy = other.mVector[i]->cloneAndReferenceProtoInstance();
    assert(copy);  // test clone() function
    addItem(copy);
  }

  emit changed();
  return *this;
}

bool OmMFNode::operator==(const OmMFNode &other) const {
  if (this == &other)
    return true;

  if (size() != other.size())
    return false;

  const int vectorSize = mVector.size();
  for (int i = 0; i < vectorSize; ++i) {
    const OmNode *const n1 = mVector[i];
    const OmNode *const n2 = other.mVector[i];
    if (*n1 != *n2)
      return false;
  }

  return true;
}

bool OmMFNode::equals(const OmValue *other) const {
  const OmMFNode *that = dynamic_cast<const OmMFNode *>(other);
  return that && *this == *that;
}

void OmMFNode::copyFrom(const OmValue *other) {
  const OmMFNode *that = dynamic_cast<const OmMFNode *>(other);
  *this = *that;
}

void OmMFNode::writeItem(OmWriter &writer, int index) const {
  assert(index >= 0 && index < size());
  mVector[index]->write(writer);
}

void OmMFNode::defHasChanged() {
  const int vectorSize = mVector.size();
  for (int i = 0; i < vectorSize; ++i)
    mVector[i]->defHasChanged();
}

int OmMFNode::nodeIndex(const OmNode *node) const {
  if (mVector.contains(const_cast<OmNode *>(node)))
    return mVector.indexOf(const_cast<OmNode *>(node));
  return -1;
}

void OmMFNode::write(OmWriter &writer) const {
  writer.writeMFStart();
  int c = 0;
  const int vectorSize = mVector.size();
  for (int i = 0; i < vectorSize; ++i) {
    if (writer.isOmniSim() || writer.isUrdf() || mVector[i]->shallExport()) {
      if (!writer.isW3d())
        writer.writeMFSeparator(c == 0, smallSeparator(i));
      writeItem(writer, i);
      ++c;
    }
  }
  writer.writeMFEnd(c == 0);
}
