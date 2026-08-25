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

#include "OmPbrAppearance.hpp"

#include "OmBackground.hpp"
#include "OmField.hpp"
#include "OmFieldChecker.hpp"
#include "OmImageTexture.hpp"
#include "OmMaterial.hpp"
#include "OmRgb.hpp"
#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFNode.hpp"
#include "OmTextureTransform.hpp"
#include "OmWorld.hpp"
#include "OmWrenRenderingContext.hpp"

#include <assimp/material.h>

void OmPbrAppearance::init() {
  mBaseColor = findSFColor("baseColor");
  mBaseColorMap = findSFNode("baseColorMap");
  mTransparency = findSFDouble("transparency");
  mRoughness = findSFDouble("roughness");
  mRoughnessMap = findSFNode("roughnessMap");
  mMetalness = findSFDouble("metalness");
  mMetalnessMap = findSFNode("metalnessMap");
  mIblStrength = findSFDouble("IBLStrength");
  mNormalMap = findSFNode("normalMap");
  mNormalMapFactor = findSFDouble("normalMapFactor");
  mOcclusionMap = findSFNode("occlusionMap");
  mOcclusionMapStrength = findSFDouble("occlusionMapStrength");
  mEmissiveColor = findSFColor("emissiveColor");
  mEmissiveColorMap = findSFNode("emissiveColorMap");
  mEmissiveIntensity = findSFDouble("emissiveIntensity");
  // fabric / cloth extension
  mSheen = findSFDouble("sheen");
  mSheenColor = findSFColor("sheenColor");
  mSheenRoughness = findSFDouble("sheenRoughness");
  mTransmissionColor = findSFColor("transmissionColor");
  mTransmissionPower = findSFDouble("transmissionPower");
  mTransmissionThickness = findSFDouble("transmissionThickness");
  mDiffuseWrap = findSFDouble("diffuseWrap");
  mSpecularAntiAliasing = findSFDouble("specularAntiAliasing");
  mDoubleSided = findSFBool("doubleSided");
}

OmPbrAppearance::OmPbrAppearance(OmTokenizer *tokenizer) : OmAbstractAppearance("PBRAppearance", tokenizer) {
  init();
}

OmPbrAppearance::OmPbrAppearance(const OmPbrAppearance &other) : OmAbstractAppearance(other) {
  init();
}

OmPbrAppearance::OmPbrAppearance(const OmNode &other) : OmAbstractAppearance(other) {
  init();
}

OmPbrAppearance::OmPbrAppearance(const aiMaterial *material, const QString &filePath) :
  OmAbstractAppearance("PBRAppearance", material) {
  aiColor3D baseColorCode(1.0f, 1.0f, 1.0f);
  material->Get(AI_MATKEY_COLOR_DIFFUSE, baseColorCode);
  mBaseColor = new OmSFColor(baseColorCode[0], baseColorCode[1], baseColorCode[2]);

  aiColor3D emissiveColor(0.0f, 0.0f, 0.0f);
  material->Get(AI_MATKEY_COLOR_EMISSIVE, emissiveColor);
  mEmissiveColor = new OmSFColor(emissiveColor[0], emissiveColor[1], emissiveColor[2]);

  float opacity = 1.0f;
  material->Get(AI_MATKEY_OPACITY, opacity);
  mTransparency = new OmSFDouble(1.0 - opacity);

  float r = 1.0f;
  if (material->Get(AI_MATKEY_SHININESS, r) == AI_SUCCESS)
    r = 1.0 - r / 255.0;
  else if (material->Get(AI_MATKEY_SHININESS_STRENGTH, r) == AI_SUCCESS)
    r = 1.0 - r;
  else if (material->Get(AI_MATKEY_REFLECTIVITY, r) == AI_SUCCESS)
    r = 1.0 - r;
  mRoughness = new OmSFDouble(r);

  mMetalness = new OmSFDouble(0.0);
  mIblStrength = new OmSFDouble(1.0);
  mNormalMapFactor = new OmSFDouble(1.0);
  mOcclusionMapStrength = new OmSFDouble(1.0);
  mEmissiveIntensity = new OmSFDouble(1.0);

  // Fabric / cloth extension. This constructor builds a SHALLOW node straight
  // from an assimp material and never runs init(), so every field has to be
  // allocated here or the accessors below dereference garbage. Assimp has no
  // sheen/transmission channel we can map, so these take the PROTO defaults --
  // which are the inert ones.
  mSheen = new OmSFDouble(0.0);
  mSheenColor = new OmSFColor(0.0, 0.0, 0.0);
  mSheenRoughness = new OmSFDouble(0.3);
  mTransmissionColor = new OmSFColor(0.0, 0.0, 0.0);
  mTransmissionPower = new OmSFDouble(12.0);
  mTransmissionThickness = new OmSFDouble(0.5);
  mDiffuseWrap = new OmSFDouble(0.0);
  mSpecularAntiAliasing = new OmSFDouble(0.0);
  mDoubleSided = new OmSFBool(false);

  // initialize maps
  OmNode::setGlobalParentNode(this);
  if (material->GetTextureCount(aiTextureType_BASE_COLOR) > 0)
    mBaseColorMap = new OmSFNode(new OmImageTexture(material, aiTextureType_BASE_COLOR, filePath));
  else if (material->GetTextureCount(aiTextureType_DIFFUSE) > 0)
    mBaseColorMap = new OmSFNode(new OmImageTexture(material, aiTextureType_DIFFUSE, filePath));
  else
    mBaseColorMap = new OmSFNode(NULL);

  OmNode::setGlobalParentNode(this);
  if (material->GetTextureCount(aiTextureType_DIFFUSE_ROUGHNESS) > 0)
    mRoughnessMap = new OmSFNode(new OmImageTexture(material, aiTextureType_DIFFUSE_ROUGHNESS, filePath));
  else
    mRoughnessMap = new OmSFNode(NULL);

  OmNode::setGlobalParentNode(this);
  if (material->GetTextureCount(aiTextureType_METALNESS) > 0)
    mMetalnessMap = new OmSFNode(new OmImageTexture(material, aiTextureType_METALNESS, filePath));
  else
    mMetalnessMap = new OmSFNode(NULL);

  OmNode::setGlobalParentNode(this);
  if (material->GetTextureCount(aiTextureType_NORMALS) > 0)
    mNormalMap = new OmSFNode(new OmImageTexture(material, aiTextureType_NORMALS, filePath));
  else if (material->GetTextureCount(aiTextureType_NORMAL_CAMERA) > 0)
    mNormalMap = new OmSFNode(new OmImageTexture(material, aiTextureType_NORMAL_CAMERA, filePath));
  else
    mNormalMap = new OmSFNode(NULL);

  OmNode::setGlobalParentNode(this);
  if (material->GetTextureCount(aiTextureType_AMBIENT_OCCLUSION) > 0)
    mOcclusionMap = new OmSFNode(new OmImageTexture(material, aiTextureType_AMBIENT_OCCLUSION, filePath));
  else if (material->GetTextureCount(aiTextureType_LIGHTMAP) > 0)
    mOcclusionMap = new OmSFNode(new OmImageTexture(material, aiTextureType_LIGHTMAP, filePath));
  else
    mOcclusionMap = new OmSFNode(NULL);

  OmNode::setGlobalParentNode(this);
  if (material->GetTextureCount(aiTextureType_EMISSION_COLOR) > 0)
    mEmissiveColorMap = new OmSFNode(new OmImageTexture(material, aiTextureType_EMISSION_COLOR, filePath));
  else if (material->GetTextureCount(aiTextureType_EMISSIVE) > 0)
    mEmissiveColorMap = new OmSFNode(new OmImageTexture(material, aiTextureType_EMISSIVE, filePath));
  else
    mEmissiveColorMap = new OmSFNode(NULL);
}

OmPbrAppearance::~OmPbrAppearance() {
  if (mIsShallowNode) {
    delete mBaseColor;
    delete mEmissiveColor;
    delete mTransparency;
    delete mRoughness;
    delete mMetalness;
    delete mIblStrength;
    delete mNormalMapFactor;
    delete mOcclusionMapStrength;
    delete mEmissiveIntensity;
    // fabric / cloth extension
    delete mSheen;
    delete mSheenColor;
    delete mSheenRoughness;
    delete mTransmissionColor;
    delete mTransmissionPower;
    delete mTransmissionThickness;
    delete mDiffuseWrap;
    delete mSpecularAntiAliasing;
    delete mDoubleSided;
    // maps
    delete mBaseColorMap;
    delete mRoughnessMap;
    delete mMetalnessMap;
    delete mNormalMap;
    delete mOcclusionMap;
    delete mEmissiveColorMap;
  }
}

void OmPbrAppearance::downloadAssets() {
  OmBaseNode::downloadAssets();
  if (baseColorMap())
    baseColorMap()->downloadAssets();

  if (roughnessMap())
    roughnessMap()->downloadAssets();

  if (metalnessMap())
    metalnessMap()->downloadAssets();

  if (normalMap())
    normalMap()->downloadAssets();

  if (occlusionMap())
    occlusionMap()->downloadAssets();

  if (emissiveColorMap())
    emissiveColorMap()->downloadAssets();
}

void OmPbrAppearance::preFinalize() {
  OmAbstractAppearance::preFinalize();

  if (baseColorMap())
    baseColorMap()->preFinalize();

  if (roughnessMap())
    roughnessMap()->preFinalize();

  if (metalnessMap())
    metalnessMap()->preFinalize();

  if (normalMap())
    normalMap()->preFinalize();

  if (occlusionMap())
    occlusionMap()->preFinalize();

  if (emissiveColorMap())
    emissiveColorMap()->preFinalize();

  updateBaseColorMap();
  updateRoughnessMap();
  updateMetalnessMap();
  updateNormalMap();
  updateOcclusionMap();
  updateEmissiveColorMap();

  mInitialEmissiveColor = mEmissiveColor->value();
}

void OmPbrAppearance::postFinalize() {
  OmAbstractAppearance::postFinalize();

  if (baseColorMap())
    baseColorMap()->postFinalize();

  if (roughnessMap())
    roughnessMap()->postFinalize();

  if (metalnessMap())
    metalnessMap()->postFinalize();

  if (normalMap())
    normalMap()->postFinalize();

  if (occlusionMap())
    occlusionMap()->postFinalize();

  if (emissiveColorMap())
    emissiveColorMap()->postFinalize();

  connect(mBaseColor, &OmSFColor::changed, this, &OmPbrAppearance::updateBaseColor);
  connect(mBaseColorMap, &OmSFNode::changed, this, &OmPbrAppearance::updateBaseColorMap);
  connect(mTransparency, &OmSFDouble::changed, this, &OmPbrAppearance::updateTransparency);
  connect(mRoughness, &OmSFDouble::changed, this, &OmPbrAppearance::updateRoughness);
  connect(mRoughnessMap, &OmSFNode::changed, this, &OmPbrAppearance::updateRoughnessMap);
  connect(mMetalness, &OmSFDouble::changed, this, &OmPbrAppearance::updateMetalness);
  connect(mMetalnessMap, &OmSFNode::changed, this, &OmPbrAppearance::updateMetalnessMap);
  connect(mIblStrength, &OmSFDouble::changed, this, &OmPbrAppearance::updateIblStrength);
  connect(mNormalMap, &OmSFNode::changed, this, &OmPbrAppearance::updateNormalMap);
  connect(mNormalMapFactor, &OmSFDouble::changed, this, &OmPbrAppearance::updateNormalMapFactor);
  connect(mOcclusionMap, &OmSFNode::changed, this, &OmPbrAppearance::updateOcclusionMap);
  connect(mOcclusionMapStrength, &OmSFDouble::changed, this, &OmPbrAppearance::updateOcclusionMapStrength);
  connect(mEmissiveColor, &OmSFColor::changed, this, &OmPbrAppearance::updateEmissiveColor);
  connect(mEmissiveColorMap, &OmSFNode::changed, this, &OmPbrAppearance::updateEmissiveColorMap);
  connect(mEmissiveIntensity, &OmSFDouble::changed, this, &OmPbrAppearance::updateEmissiveIntensity);

  connect(mSheen, &OmSFDouble::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mSheenColor, &OmSFColor::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mSheenRoughness, &OmSFDouble::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mTransmissionColor, &OmSFColor::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mTransmissionPower, &OmSFDouble::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mTransmissionThickness, &OmSFDouble::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mDiffuseWrap, &OmSFDouble::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mSpecularAntiAliasing, &OmSFDouble::changed, this, &OmPbrAppearance::updateFabricParameters);
  connect(mDoubleSided, &OmSFBool::changed, this, &OmPbrAppearance::updateFabricParameters);

  connect(OmWrenRenderingContext::instance(), &OmWrenRenderingContext::backgroundColorChanged, this,
          &OmPbrAppearance::updateBackgroundColor);

  // D1.4: these connects used to be established lazily inside the (deleted) WREN
  // modifyWrenMaterial(); they keep the `changed()` propagation on background
  // luminosity/cubemap edits alive for the renderer-facing consumers.
  OmBackground *background = OmBackground::firstInstance();
  if (background) {
    connect(background, &OmBackground::luminosityChanged, this, &OmPbrAppearance::updateCubeMap, Qt::UniqueConnection);
    connect(background, &OmBackground::cubemapChanged, this, &OmPbrAppearance::updateCubeMap, Qt::UniqueConnection);
  }

  // we need to do one more background probe if there is no local cubemap
  updateCubeMap();

  if (!OmWorld::instance()->isLoading())
    emit changed();
}

void OmPbrAppearance::reset(const QString &id) {
  OmAbstractAppearance::reset(id);

  if (baseColorMap())
    baseColorMap()->reset(id);

  if (roughnessMap())
    roughnessMap()->reset(id);

  if (metalnessMap())
    metalnessMap()->reset(id);

  if (normalMap())
    normalMap()->reset(id);

  if (occlusionMap())
    occlusionMap()->reset(id);

  if (emissiveColorMap())
    emissiveColorMap()->reset(id);
}

void OmPbrAppearance::setEmissiveColor(const OmRgb &color) {
  mEmissiveColor->setValue(color);
}

void OmPbrAppearance::createWrenObjects() {
  OmAbstractAppearance::createWrenObjects();

  sanitizeFields();

  if (baseColorMap())
    baseColorMap()->createWrenObjects();

  if (roughnessMap())
    roughnessMap()->createWrenObjects();

  if (metalnessMap())
    metalnessMap()->createWrenObjects();

  if (normalMap())
    normalMap()->createWrenObjects();

  if (occlusionMap())
    occlusionMap()->createWrenObjects();

  if (emissiveColorMap())
    emissiveColorMap()->createWrenObjects();
}

OmImageTexture *OmPbrAppearance::baseColorMap() const {
  return dynamic_cast<OmImageTexture *>(mBaseColorMap->value());
}

OmImageTexture *OmPbrAppearance::roughnessMap() const {
  return dynamic_cast<OmImageTexture *>(mRoughnessMap->value());
}

OmImageTexture *OmPbrAppearance::metalnessMap() const {
  return dynamic_cast<OmImageTexture *>(mMetalnessMap->value());
}

OmImageTexture *OmPbrAppearance::normalMap() const {
  return dynamic_cast<OmImageTexture *>(mNormalMap->value());
}

OmImageTexture *OmPbrAppearance::occlusionMap() const {
  return dynamic_cast<OmImageTexture *>(mOcclusionMap->value());
}

OmImageTexture *OmPbrAppearance::emissiveColorMap() const {
  return dynamic_cast<OmImageTexture *>(mEmissiveColorMap->value());
}

bool OmPbrAppearance::isBaseColorTextureLoaded() const {
  return (baseColorMap() && baseColorMap()->hasImage());
}

bool OmPbrAppearance::isRoughnessTextureLoaded() const {
  return (roughnessMap() && roughnessMap()->hasImage());
}

bool OmPbrAppearance::isOcclusionTextureLoaded() const {
  return (occlusionMap() && occlusionMap()->hasImage());
}

OmRgb OmPbrAppearance::baseColor() const {
  return mBaseColor->value();
}

OmRgb OmPbrAppearance::emissiveColor() const {
  return mEmissiveColor->value();
}

double OmPbrAppearance::emissiveIntensity() const {
  return mEmissiveIntensity->value();
}

double OmPbrAppearance::transparency() const {
  return mTransparency->value();
}

double OmPbrAppearance::roughness() const {
  return mRoughness->value();
}

double OmPbrAppearance::iblStrength() const {
  return mIblStrength->value();
}

void OmPbrAppearance::pickColorInBaseColorTexture(OmRgb &pickedColor, const OmVector2 &uv) const {
  OmImageTexture *tex = baseColorMap();
  if (tex) {
    OmVector2 uvTransformed = transformUVCoordinate(uv);
    tex->pickColor(uvTransformed, pickedColor);
  } else
    pickedColor.setValue(1.0, 1.0, 1.0);  // default value
}

void OmPbrAppearance::pickRoughnessInTexture(double *roughness, const OmVector2 &uv) const {
  *roughness = getRedValueInTexture(roughnessMap(), uv);
}

void OmPbrAppearance::pickOcclusionInTexture(double *occlusion, const OmVector2 &uv) const {
  *occlusion = getRedValueInTexture(occlusionMap(), uv);
}

double OmPbrAppearance::getRedValueInTexture(OmImageTexture *texture, const OmVector2 &uv) const {
  if (texture) {
    OmRgb pickedColor;
    OmVector2 uvTransformed = transformUVCoordinate(uv);
    texture->pickColor(uvTransformed, pickedColor);
    return pickedColor.red();
  }
  return 0.0;  // default value
}

void OmPbrAppearance::updateCubeMap() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateBaseColor() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateBackgroundColor() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateBaseColorMap() {
  if (baseColorMap())
    connect(baseColorMap(), &OmImageTexture::changed, this, &OmPbrAppearance::updateBaseColorMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateTransparency() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateRoughness() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mRoughness, 0.0, 1.0, 0.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateRoughnessMap() {
  if (roughnessMap())
    connect(roughnessMap(), &OmImageTexture::changed, this, &OmPbrAppearance::updateRoughnessMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateMetalness() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mMetalness, 0.0, 1.0, 0.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateMetalnessMap() {
  if (metalnessMap())
    connect(metalnessMap(), &OmImageTexture::changed, this, &OmPbrAppearance::updateMetalnessMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateIblStrength() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mIblStrength, 1.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateNormalMap() {
  if (normalMap())
    connect(normalMap(), &OmImageTexture::changed, this, &OmPbrAppearance::updateNormalMap, Qt::UniqueConnection);

  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateNormalMapFactor() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mNormalMapFactor, 1.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateOcclusionMap() {
  if (occlusionMap())
    connect(occlusionMap(), &OmImageTexture::changed, this, &OmPbrAppearance::updateOcclusionMap, Qt::UniqueConnection);
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateOcclusionMapStrength() {
  if (OmFieldChecker::resetDoubleIfNegative(this, mOcclusionMapStrength, 1.0))
    return;
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateEmissiveColor() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateEmissiveColorMap() {
  if (emissiveColorMap())
    connect(emissiveColorMap(), &OmImageTexture::changed, this, &OmPbrAppearance::updateEmissiveColorMap, Qt::UniqueConnection);
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateEmissiveIntensity() {
  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::updateFabricParameters() {
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSheen, 0.0, 1.0, 0.0))
    return;
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSheenRoughness, 0.0, 1.0, 0.3))
    return;
  if (OmFieldChecker::resetDoubleIfNegative(this, mTransmissionPower, 12.0))
    return;
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransmissionThickness, 0.0, 1.0, 0.5))
    return;
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mDiffuseWrap, 0.0, 1.0, 0.0))
    return;
  if (OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSpecularAntiAliasing, 0.0, 1.0, 0.0))
    return;

  if (isPostFinalizedCalled())
    emit changed();
}

void OmPbrAppearance::exportNodeSubNodes(OmWriter &writer) const {
  if (writer.isOmniSim()) {
    OmAbstractAppearance::exportNodeSubNodes(writer);
    return;
  }

  if (writer.isW3d()) {
    if (baseColorMap()) {
      baseColorMap()->setRole("baseColorMap");
      baseColorMap()->write(writer);
    }
    if (roughnessMap()) {
      roughnessMap()->setRole("roughnessMap");
      roughnessMap()->write(writer);
    }
    if (metalnessMap()) {
      metalnessMap()->setRole("metalnessMap");
      metalnessMap()->write(writer);
    }
    if (normalMap()) {
      normalMap()->setRole("normalMap");
      normalMap()->write(writer);
    }
    if (occlusionMap()) {
      occlusionMap()->setRole("occlusionMap");
      occlusionMap()->write(writer);
    }
    if (emissiveColorMap()) {
      emissiveColorMap()->setRole("emissiveColorMap");
      emissiveColorMap()->write(writer);
    }

    if (textureTransform())
      textureTransform()->write(writer);
  }
}

QStringList OmPbrAppearance::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "baseColor"
         << "emissiveColor"
         << "roughness"
         << "metalness"
         << "IBLStrength"
         << "normalMapFactor"
         << "occlusionMapStrength"
         << "emissiveIntensity"
         << "transparency";
  return fields;
}

bool OmPbrAppearance::exportNodeHeader(OmWriter &writer) const {
  if (writer.isUrdf())
    return true;
  return OmAbstractAppearance::exportNodeHeader(writer);
}

void OmPbrAppearance::exportShallowNode(const OmWriter &writer) const {
  if (!writer.isW3d())
    return;

  if (baseColorMap()) {
    baseColorMap()->exportShallowNode(writer);
  }

  if (roughnessMap()) {
    roughnessMap()->exportShallowNode(writer);
  }

  if (metalnessMap()) {
    metalnessMap()->exportShallowNode(writer);
  }
  if (normalMap()) {
    normalMap()->exportShallowNode(writer);
  }
  if (occlusionMap()) {
    occlusionMap()->exportShallowNode(writer);
  }
  if (emissiveColorMap()) {
    emissiveColorMap()->exportShallowNode(writer);
  }
}

void OmPbrAppearance::sanitizeFields() {
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransparency, 0.0, 1.0, 0.0);
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mRoughness, 0.0, 1.0, 0.0);
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mMetalness, 0.0, 1.0, 0.0);
  OmFieldChecker::resetDoubleIfNegative(this, mIblStrength, 1.0);
  OmFieldChecker::resetDoubleIfNegative(this, mNormalMapFactor, 1.0);
  OmFieldChecker::resetDoubleIfNegative(this, mOcclusionMapStrength, 1.0);
  // fabric / cloth extension
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSheen, 0.0, 1.0, 0.0);
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSheenRoughness, 0.0, 1.0, 0.3);
  OmFieldChecker::resetDoubleIfNegative(this, mTransmissionPower, 12.0);
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mTransmissionThickness, 0.0, 1.0, 0.5);
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mDiffuseWrap, 0.0, 1.0, 0.0);
  OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(this, mSpecularAntiAliasing, 0.0, 1.0, 0.0);
}
