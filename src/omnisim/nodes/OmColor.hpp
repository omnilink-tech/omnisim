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

#ifndef OM_COLOR_HPP
#define OM_COLOR_HPP

#include "OmBaseNode.hpp"

class OmColor : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmColor(OmTokenizer *tokenizer = NULL);
  OmColor(const OmColor &other);
  explicit OmColor(const OmNode &other);
  virtual ~OmColor() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_COLOR; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  const OmMFColor &color() const { return *mColor; }

  // helper, the size of receiving array should be equal to (or greater than) the number of mColor items
  void copyValuesToArray(double array[][3]) const;

  QStringList fieldsToSynchronizeWithW3d() const override;

signals:
  void changed();

protected slots:
  void updateColor();

private:
  // user accessible fields
  OmMFColor *mColor;

  OmColor &operator=(const OmColor &);  // non copyable
  OmNode *clone() const override { return new OmColor(*this); }
  void init();
};

#endif
