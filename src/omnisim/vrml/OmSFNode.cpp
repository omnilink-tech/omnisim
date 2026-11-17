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

#include "OmSFNode.hpp"

#include "OmNode.hpp"
#include "OmNodeReader.hpp"
#include "OmTokenizer.hpp"
#include "OmWriter.hpp"

OmSFNode::OmSFNode(OmTokenizer *tokenizer, const QString &worldPath) {
  mValue = NULL;
  readSFNode(tokenizer, worldPath);
}

OmSFNode::OmSFNode(const OmSFNode &other) {
  if (other.mValue) {
    mValue = other.mValue->cloneAndReferenceProtoInstance();
    if (mValue)
      mValue->setInsertionCompleted();
  } else
    mValue = NULL;
}

void OmSFNode::readSFNode(OmTokenizer *tokenizer, const QString &worldPath) {
  delete mValue;

  if (OmNodeReader::current())
    // reading a regular list of nodes (file scope)
    mValue = OmNodeReader::current()->readNode(tokenizer, worldPath);
  else {
    // reading a default proto parameter (private scope)
    OmNodeReader reader;
    mValue = reader.readNode(tokenizer, worldPath);
  }
  if (mValue)
    mValue->setInsertionCompleted();
}

OmSFNode::OmSFNode(OmNode *node) {
  if (mValue == node)
    return;

  mValue = node;
  if (mValue)
    mValue->setInsertionCompleted();
}

OmSFNode::~OmSFNode() {
  delete mValue;
}

void OmSFNode::write(OmWriter &writer) const {
  if (mValue)
    mValue->write(writer);
  else if (!writer.isW3d() && !writer.isUrdf())
    writer << "NULL";
}

void OmSFNode::setValue(OmNode *node) {
  if (mValue == node)
    return;

  const OmNode *tmp = mValue;
  mValue = node;
  if (mValue)
    mValue->setInsertionCompleted();
  emit changed();
  delete tmp;
}

OmSFNode &OmSFNode::operator=(const OmSFNode &other) {
  if (this == &other)
    return *this;

  OmNode *tmp = mValue;
  if (other.mValue) {
    mValue = other.mValue->cloneAndReferenceProtoInstance();
    if (mValue)
      mValue->setInsertionCompleted();
  } else
    mValue = NULL;

  emit changed();
  delete tmp;
  return *this;
}

bool OmSFNode::operator==(const OmSFNode &other) const {
  if (this == &other)
    return true;

  if (mValue == NULL && other.mValue == NULL)
    return true;

  if (mValue == NULL || other.mValue == NULL)
    return false;

  return *mValue == *other.mValue;
}

bool OmSFNode::equals(const OmValue *other) const {
  const OmSFNode *that = dynamic_cast<const OmSFNode *>(other);
  return that && *this == *that;
}

void OmSFNode::copyFrom(const OmValue *other) {
  const OmSFNode *that = dynamic_cast<const OmSFNode *>(other);
  *this = *that;
}

void OmSFNode::defHasChanged() {
  if (mValue)
    mValue->defHasChanged();
}
