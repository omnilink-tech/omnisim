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

#ifndef OM_SPOT_LIGHT_HPP
#define OM_SPOT_LIGHT_HPP

#include "OmLight.hpp"

class OmVector3;

class OmSpotLight : public OmLight {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSpotLight(OmTokenizer *tokenizer = NULL);
  OmSpotLight(const OmSpotLight &other);
  explicit OmSpotLight(const OmNode &other);
  virtual ~OmSpotLight() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SPOT_LIGHT; }
  void preFinalize() override;
  void postFinalize() override;

  // specific functions
  const OmVector3 &direction() const;
  double cutOffAngle() const;
  double beamWidth() const;
  double exponent() const;
  double computeAttenuation(double distance) const;
  OmVector3 computeAbsoluteLocation() const;
  OmVector3 computeAbsoluteDirection() const;  // direction field rotated into world space
  // raw attenuation coefficients (constant, linear, quadratic) + cutoff radius, for renderers
  // that evaluate attenuation per-fragment (the wgpu multi-light path)
  const OmVector3 &attenuation() const;
  double radius() const;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected slots:
  void updateAmbientIntensity() override;
  // cppcheck-suppress uselessOverride
  void updateIntensity() override;
  // cppcheck-suppress uselessOverride
  void updateColor() override;

private:
  // user accessible fields
  OmSFVector3 *mAttenuation;
  OmSFVector3 *mLocation;
  OmSFDouble *mRadius;
  OmSFVector3 *mDirection;
  OmSFDouble *mCutOffAngle;
  OmSFDouble *mBeamWidth;

  OmSpotLight &operator=(const OmSpotLight &);  // non copyable
  OmNode *clone() const override { return new OmSpotLight(*this); }
  void init();
  void checkAmbientAndAttenuationExclusivity();

private slots:
  void updateDirection();
  void updateCutOffAngle();
  void updateBeamWidth();
  void updateAttenuation();
  void updateLocation();
  void updateRadius();
};

#endif
