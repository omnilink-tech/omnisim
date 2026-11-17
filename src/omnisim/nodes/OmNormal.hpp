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

#ifndef OM_NORMAL_HPP
#define OM_NORMAL_HPP

#include "OmBaseNode.hpp"
#include "OmMFVector3.hpp"

class OmVector3;

class OmNormal : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmNormal(OmTokenizer *tokenizer = NULL);
  OmNormal(const OmNormal &other);
  explicit OmNormal(const OmNode &other);
  virtual ~OmNormal() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_NORMAL; }

  // field accessors
  const OmMFVector3 &vector() const { return *mVector; }
  const OmVector3 &vector(int index) const { return mVector->item(index); }
  int vectorSize() const { return mVector->size(); }
  void setVector(int index, const OmVector3 &vector) { mVector->setItem(index, vector); }

  QStringList fieldsToSynchronizeWithW3d() const override;

private:
  // user accessible fields
  OmMFVector3 *mVector;

  OmNormal &operator=(const OmNormal &);  // non copyable
  OmNode *clone() const override { return new OmNormal(*this); }
  void init();
};

#endif
