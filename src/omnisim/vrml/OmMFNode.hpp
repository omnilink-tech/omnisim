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

#ifndef OM_MF_NODE_HPP
#define OM_MF_NODE_HPP

//
// Description: field value that contains a multiple OmNode
//

#include "OmMultipleValue.hpp"

#include <QtCore/QVector>

#include <cassert>

class OmNode;

class OmMFNode : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmNode *OmNodePtr;
  typedef OmMFIterator<OmMFNode, OmNode *> Iterator;

  OmMFNode(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFNode(const OmMFNode &other);
  virtual ~OmMFNode() override;
  OmValue *clone() const override { return new OmMFNode(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void insertDefaultItem(int index) override;
  // cppcheck-suppress knownPointerToBool
  OmVariant defaultNewVariant() const override { return OmVariant((OmNode *)NULL); }
  void removeItem(int index) override;  // remove and delete the node instance
  bool removeNode(OmNode *node);        // remove without deleting the node instance
  void writeItem(OmWriter &writer, int index) const override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_NODE; }
  const OmNodePtr &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, OmNode *node);  // replace node at index and delete the previous node instance
  void addItem(OmNode *node);
  void insertItem(int index, OmNode *node);
  OmMFNode &operator=(const OmMFNode &other);
  bool operator==(const OmMFNode &other) const;
  int nodeIndex(const OmNode *node) const;
  void write(OmWriter &) const override;

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override { return false; }

private:
  QVector<OmNode *> mVector;
  void defHasChanged() override;
};

#endif
