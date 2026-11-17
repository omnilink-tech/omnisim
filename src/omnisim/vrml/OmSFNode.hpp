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

#ifndef OM_SF_NODE_HPP
#define OM_SF_NODE_HPP

//
// Description: field value that contains a single OmNode
//

#include "OmSingleValue.hpp"

class OmNode;

class OmSFNode : public OmSingleValue {
  Q_OBJECT

public:
  OmSFNode(OmTokenizer *tokenizer, const QString &worldPath);
  OmSFNode(const OmSFNode &other);
  explicit OmSFNode(OmNode *node);
  virtual ~OmSFNode() override;
  void read(OmTokenizer *tokenizer, const QString &worldPath) override { readSFNode(tokenizer, worldPath); }
  void write(OmWriter &writer) const override;
  OmValue *clone() const override { return new OmSFNode(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  OmVariant variantValue() const override { return OmVariant(mValue); }
  WbFieldType type() const override { return WB_SF_NODE; }
  OmNode *value() const { return mValue; }
  void setValue(OmNode *node);
  void removeValue() { setValue(NULL); }
  OmSFNode &operator=(const OmSFNode &other);
  bool operator==(const OmSFNode &other) const;

private:
  OmNode *mValue;
  void readSFNode(OmTokenizer *tokenizer, const QString &worldPath);
  void defHasChanged() override;
};

#endif
