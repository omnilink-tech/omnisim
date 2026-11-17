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

#ifndef OM_TEXTURE_COORDINATE_HPP
#define OM_TEXTURE_COORDINATE_HPP

#include "OmBaseNode.hpp"
#include "OmMFVector2.hpp"

class OmVector2;

class OmTextureCoordinate : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmTextureCoordinate(OmTokenizer *tokenizer = NULL);
  OmTextureCoordinate(const OmTextureCoordinate &other);
  explicit OmTextureCoordinate(const OmNode &other);
  virtual ~OmTextureCoordinate() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TEXTURE_COORDINATE; }

  // field accessors
  const OmMFVector2 &point() const { return *mPoint; }
  const OmVector2 &point(int index) const { return mPoint->item(index); }
  int pointSize() const { return mPoint->size(); }

  QStringList fieldsToSynchronizeWithW3d() const override;

private:
  // user accessible fields
  OmMFVector2 *mPoint;

  OmTextureCoordinate &operator=(const OmTextureCoordinate &);  // non copyable
  OmNode *clone() const override { return new OmTextureCoordinate(*this); }
  void init();
};

#endif
