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

#ifndef OM_DIRECTIONAL_LIGHT_HPP
#define OM_DIRECTIONAL_LIGHT_HPP

#include "OmLight.hpp"

class OmVector3;

class OmDirectionalLight : public OmLight {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmDirectionalLight(OmTokenizer *tokenizer = NULL);
  OmDirectionalLight(const OmDirectionalLight &other);
  explicit OmDirectionalLight(const OmNode &other);
  virtual ~OmDirectionalLight() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_DIRECTIONAL_LIGHT; }
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  const OmVector3 &direction() const;

  QStringList fieldsToSynchronizeWithW3d() const override;

private slots:
  void updateDirection();

private:
  // user accessible fields
  OmSFVector3 *mDirection;

  OmDirectionalLight &operator=(const OmDirectionalLight &);  // non copyable
  OmNode *clone() const override { return new OmDirectionalLight(*this); }
  void init();
};

#endif
