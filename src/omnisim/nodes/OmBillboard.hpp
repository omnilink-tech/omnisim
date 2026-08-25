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

#ifndef OM_BILLBOARD_HPP
#define OM_BILLBOARD_HPP

#include "OmGroup.hpp"

class OmBillboard : public OmGroup {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmBillboard(OmTokenizer *tokenizer = NULL);
  OmBillboard(const OmBillboard &other);
  explicit OmBillboard(const OmNode &other);
  virtual ~OmBillboard() override;

  // reimplemented functions
  int nodeType() const override { return WB_NODE_BILLBOARD; }
  void createWrenObjects() override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override {
    return QList<const OmBaseNode *>() << this;
  }

private:
  OmBillboard &operator=(const OmBillboard &);  // non copyable
  OmNode *clone() const override { return new OmBillboard(*this); }
};

#endif
