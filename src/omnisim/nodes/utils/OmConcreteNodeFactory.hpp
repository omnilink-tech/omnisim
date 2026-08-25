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

#ifndef OM_CONCRETE_NODE_FACTORY_HPP
#define OM_CONCRETE_NODE_FACTORY_HPP

//
// Description: a class used to instantiate nodes and protos
//

#include "OmNodeFactory.hpp"

class OmTokenizer;
class OmNode;
class QString;
class OmWriter;

class OmConcreteNodeFactory : public OmNodeFactory {
public:
  // reimplemented public functions
  OmNode *createNode(const QString &modelName, OmTokenizer *tokenizer = 0, OmNode *parentNode = NULL,
                     const QString *protoUrl = NULL) override;
  OmNode *createCopy(const OmNode &original) override;
  const QString slotType(OmNode *node) override;
  bool validateExistingChildNode(const OmField *field, const OmNode *childNode, const OmNode *node, bool isInBoundingObject,
                                 QString &errorMessage) const override;

private:
  OmConcreteNodeFactory() {}
  virtual ~OmConcreteNodeFactory() override {}
  static OmConcreteNodeFactory gFactory;
};

#endif
