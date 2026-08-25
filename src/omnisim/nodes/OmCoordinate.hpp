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

#ifndef OM_COORDINATE_HPP
#define OM_COORDINATE_HPP

#include "OmBaseNode.hpp"
#include "OmMFVector3.hpp"

class OmVector3;

class OmCoordinate : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCoordinate(OmTokenizer *tokenizer = NULL);
  OmCoordinate(const OmCoordinate &other);
  explicit OmCoordinate(const OmNode &other);
  virtual ~OmCoordinate() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_COORDINATE; }

  // field accessors
  const OmMFVector3 &point() const { return *mPoint; }
  const OmVector3 &point(int index) const { return mPoint->item(index); }
  int pointSize() const { return mPoint->size(); }
  void rescale(const OmVector3 &v) { mPoint->rescale(v); }
  void rescaleAndTranslate(int coordinate, double scale, double translation) {
    mPoint->rescaleAndTranslate(coordinate, scale, translation);
  }
  void rescaleAndTranslate(const OmVector3 &s, const OmVector3 &t) { mPoint->rescaleAndTranslate(s, t); }
  void translate(const OmVector3 &v) { mPoint->translate(v); }
  QStringList fieldsToSynchronizeWithW3d() const override;

private:
  // user accessible fields
  OmMFVector3 *mPoint;

  OmCoordinate &operator=(const OmCoordinate &);  // non copyable
  OmNode *clone() const override { return new OmCoordinate(*this); }
  void init();
};

#endif
