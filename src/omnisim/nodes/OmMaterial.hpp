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

#ifndef OM_MATERIAL_HPP
#define OM_MATERIAL_HPP

#include "OmBaseNode.hpp"
#include "OmRgb.hpp"

class OmMaterial : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmMaterial(OmTokenizer *tokenizer = NULL);
  OmMaterial(const OmMaterial &other);
  explicit OmMaterial(const OmNode &other);
  virtual ~OmMaterial() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_MATERIAL; }
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  float transparency() const;
  const OmRgb &emissiveColor() const;
  void setEmissiveColor(const OmRgb &color);
  const OmRgb &diffuseColor() const;
  // VRML ambient reflection coefficient. The renderer turns it into the phong `ambient`
  // term: (ai,ai,ai) on a textured material, ai x diffuseColor otherwise.
  double ambientIntensity() const;

  const OmRgb &initialEmissiveColor() const { return mInitialEmissiveColor; }

  // export
  QStringList fieldsToSynchronizeWithW3d() const override;

signals:
  void changed();

private:
  // user accessible fields
  OmSFDouble *mAmbientIntensity;
  OmSFColor *mDiffuseColor;
  OmSFColor *mEmissiveColor;
  OmSFDouble *mShininess;
  OmSFColor *mSpecularColor;
  OmSFDouble *mTransparency;

  OmRgb mInitialEmissiveColor;

  OmMaterial &operator=(const OmMaterial &);  // non copyable
  OmNode *clone() const override { return new OmMaterial(*this); }

  void init();

private slots:
  void updateAmbientIntensity();
  void updateDiffuseColor();
  void updateEmissiveColor();
  void updateShininess();
  void updateSpecularColor();
  void updateTransparency();
};

#endif
