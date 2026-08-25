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

#ifndef OM_APPEARANCE_HPP
#define OM_APPEARANCE_HPP

#include "OmAbstractAppearance.hpp"

class OmImageTexture;
class OmMaterial;
class OmRgb;
class OmVector2;

class OmAppearance : public OmAbstractAppearance {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmAppearance(OmTokenizer *tokenizer = NULL);
  OmAppearance(const OmAppearance &other);
  explicit OmAppearance(const OmNode &other);
  virtual ~OmAppearance() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_APPEARANCE; }
  void downloadAssets() override;
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override { return true; }

  // field accessors
  OmMaterial *material() const;
  OmImageTexture *texture() const;

  // specific functions
  bool isTextureLoaded() const;
  void pickColorInTexture(const OmVector2 &uv, OmRgb &pickedColor) const;
  OmRgb diffuseColor() const;

private:
  // user accessible fields
  OmSFNode *mMaterial;
  OmSFNode *mTexture;

  OmAppearance &operator=(const OmAppearance &);  // non copyable
  OmNode *clone() const override { return new OmAppearance(*this); }

  void init();

private slots:
  void updateMaterial();
  void updateTexture();
};

#endif
