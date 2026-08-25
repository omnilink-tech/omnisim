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

#ifndef OM_LIGHT_HPP
#define OM_LIGHT_HPP

//
// Description: abstract base class for all lights
// Inherited by:
//   OmDirectionalLight, OmPointLight, OmSpotLight
//

#include "OmBaseNode.hpp"
#include "OmRgb.hpp"

#include <QtCore/QList>

class OmLight : public OmBaseNode {
  Q_OBJECT

public:
  // destructor
  virtual ~OmLight() override;

  // reimplemented public functions
  void createWrenObjects() override;
  void preFinalize() override;
  void postFinalize() override;

  // field accessors
  bool isOn() const;
  bool castShadows() const;
  bool castLensFlares() const;
  double intensity() const;
  double ambientIntensity() const;
  const OmRgb &color() const;

  const OmRgb &initialColor() const { return mInitialColor; }
  void setColor(const OmRgb &color);

  void toggleOn(bool on);

  // specific functions
  static bool doesAtLeastOneLightCastsShadows() { return numberOfLightsCastingShadows() > 0; }
  static const QList<const OmLight *> &lights() { return cLights; }
  static int numberOfOnLights();

  // The SCENE-GLOBAL ambient light: sum over every ON light of (ambientIntensity x color).
  // The ONE definition of the scene ambient, consumed by the wgpu renderer, which multiplies it
  // by the material's own ambient to shade a legacy Appearance/Material surface.
  // Every light type defaults ambientIntensity to 0, so an ordinary world returns (0,0,0).
  static void sceneAmbientLight(float rgb[3]);

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  // all constructors are reserved for derived classes only
  OmLight(const OmLight &other);
  OmLight(const OmNode &other);
  OmLight(const QString &modelName, OmTokenizer *tokenizer);

  void setAmbientIntensity(double value);

  // user accessible fields
  OmSFColor *mColor;
  OmSFDouble *mIntensity;
  OmSFBool *mOn;
  OmSFDouble *mAmbientIntensity;
  OmSFBool *mCastShadows;
  OmSFBool *mCastLensFlares;

protected slots:
  virtual void updateAmbientIntensity();
  virtual void updateIntensity();
  virtual void updateOn();
  virtual void updateColor();

private:
  static QList<const OmLight *> cLights;
  static int numberOfLightsCastingShadows();

  // other variables
  OmRgb mInitialColor;

  OmLight &operator=(const OmLight &);  // non copyable
  // only derived classes can be cloned
  OmNode *clone() const override = 0;

  void init();

private slots:
  void updateCastShadows();

signals:
  void castLensFlaresChanged();
  void isOnChanged();
  void colorChanged();
  void intensityChanged();
  void locationChanged();
  void directionChanged();
};

#endif
