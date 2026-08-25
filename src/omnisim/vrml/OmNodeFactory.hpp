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

#ifndef OM_NODE_FACTORY_HPP
#define OM_NODE_FACTORY_HPP

//
// Description: singleton class responsible for instantiating nodes and protos
//
// Inherited by: OmConcreteNodeFactory
//

#include <cstddef>

#include <QtCore/QString>

class OmField;
class OmNode;
class OmTokenizer;
class OmWriter;

class OmNodeFactory {
public:
  static OmNodeFactory *instance();

  // create a built-in node or proto instance matching 'modelName'
  // if 'tokenizer' is not specified, the node or proto instance is constructed with default field values
  // 'parentNode' specifies the global parent value to be set before creating PROTO instances, if it is
  // not specified, the current global parent is used
  virtual OmNode *createNode(const QString &modelName, OmTokenizer *tokenizer = 0, OmNode *parentNode = NULL,
                             const QString *protoFilePath = NULL) = 0;

  // create and return a copy of a node
  // the fields of the copy are initialized with the values found in the original
  // the copy will have the same model as the original
  virtual OmNode *createCopy(const OmNode &original) = 0;

  virtual const QString slotType(OmNode *node) = 0;
  virtual bool validateExistingChildNode(const OmField *field, const OmNode *childNode, const OmNode *node,
                                         bool isInBoundingObject, QString &errorMessage) const = 0;

protected:
  OmNodeFactory();
  virtual ~OmNodeFactory();

private:
};

#endif
