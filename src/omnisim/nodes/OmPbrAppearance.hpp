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

#ifndef OM_PBR_APPEARANCE_HPP
#define OM_PBR_APPEARANCE_HPP

#include "OmAbstractAppearance.hpp"
#include "OmRgb.hpp"

class OmImageTexture;

struct aiMaterial;

class OmPbrAppearance : public OmAbstractAppearance {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPbrAppearance(OmTokenizer *tokenizer = NULL);
  OmPbrAppearance(const OmPbrAppearance &other);
  explicit OmPbrAppearance(const OmNode &other);
  OmPbrAppearance(const aiMaterial *material, const QString &filePath);
  virtual ~OmPbrAppearance() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PBR_APPEARANCE; }
  void downloadAssets() override;
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override { return true; }

  void setEmissiveColor(const OmRgb &color);

  const OmRgb &initialEmissiveColor() const { return mInitialEmissiveColor; }

  // field accessors
  OmImageTexture *baseColorMap() const;
  OmImageTexture *roughnessMap() const;
  OmImageTexture *metalnessMap() const;
  OmImageTexture *normalMap() const;
  OmImageTexture *occlusionMap() const;
  OmImageTexture *emissiveColorMap() const;

  // specific functions
  bool isBaseColorTextureLoaded() const;
  bool isRoughnessTextureLoaded() const;
  bool isOcclusionTextureLoaded() const;
  void pickColorInBaseColorTexture(OmRgb &pickedColor, const OmVector2 &uv) const;
  void pickRoughnessInTexture(double *roughness, const OmVector2 &uv) const;
  void pickOcclusionInTexture(double *occlusion, const OmVector2 &uv) const;
  OmRgb baseColor() const;
  OmRgb emissiveColor() const;
  double emissiveIntensity() const;
  double transparency() const;
  double roughness() const;
  // PBRAppearance.IBLStrength: the renderer premultiplies it with Background.luminosity into
  // ONE scale factor applied to BOTH the diffuse and the specular image-based ambient.
  double iblStrength() const;

  QStringList fieldsToSynchronizeWithW3d() const override;
  void exportShallowNode(const OmWriter &writer) const;

protected:
  bool exportNodeHeader(OmWriter &writer) const override;
  void exportNodeSubNodes(OmWriter &writer) const override;

private:
  OmPbrAppearance &operator=(const OmPbrAppearance &);  // non copyable
  OmNode *clone() const override { return new OmPbrAppearance(*this); }
  double getRedValueInTexture(OmImageTexture *texture, const OmVector2 &uv) const;

  void init();
  void sanitizeFields();

  OmSFColor *mBaseColor;
  OmSFNode *mBaseColorMap;
  OmSFDouble *mTransparency;
  OmSFDouble *mRoughness;
  OmSFNode *mRoughnessMap;
  OmSFDouble *mMetalness;
  OmSFNode *mMetalnessMap;
  OmSFDouble *mIblStrength;
  OmSFNode *mNormalMap;
  OmSFDouble *mNormalMapFactor;
  OmSFNode *mOcclusionMap;
  OmSFDouble *mOcclusionMapStrength;
  OmSFColor *mEmissiveColor;
  OmSFNode *mEmissiveColorMap;
  OmSFDouble *mEmissiveIntensity;

  // Fabric / cloth extension. All default to "inert" (see PBRAppearance.wrl).
  OmSFDouble *mSheen;
  OmSFColor *mSheenColor;
  OmSFDouble *mSheenRoughness;
  OmSFColor *mTransmissionColor;
  OmSFDouble *mTransmissionPower;
  OmSFDouble *mTransmissionThickness;
  OmSFDouble *mDiffuseWrap;
  OmSFDouble *mSpecularAntiAliasing;
  OmSFBool *mDoubleSided;

  OmRgb mInitialEmissiveColor;

private slots:
  void updateCubeMap();
  void updateBackgroundColor();
  void updateBaseColor();
  void updateBaseColorMap();
  void updateTransparency();
  void updateRoughness();
  void updateRoughnessMap();
  void updateMetalness();
  void updateMetalnessMap();
  void updateIblStrength();
  void updateNormalMap();
  void updateNormalMapFactor();
  void updateOcclusionMap();
  void updateOcclusionMapStrength();
  void updateEmissiveColor();
  void updateEmissiveColorMap();
  void updateEmissiveIntensity();
  void updateFabricParameters();
};

#endif
