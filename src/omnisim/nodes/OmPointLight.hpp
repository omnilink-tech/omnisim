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

#ifndef OM_POINT_LIGHT_HPP
#define OM_POINT_LIGHT_HPP

#include "OmLight.hpp"

#include <QtCore/QMap>

class OmVector3;

class OmPointLight : public OmLight {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPointLight(OmTokenizer *tokenizer = NULL);
  OmPointLight(const OmPointLight &other);
  explicit OmPointLight(const OmNode &other);
  virtual ~OmPointLight() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_POINT_LIGHT; }
  void preFinalize() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  // specific functions
  double computeAttenuation(double distance) const;
  OmVector3 computeAbsoluteLocation() const;
  // raw attenuation coefficients (constant, linear, quadratic) + cutoff radius, for renderers
  // that evaluate attenuation per-fragment (the wgpu multi-light path)
  const OmVector3 &attenuation() const;
  double radius() const;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected slots:
  void updateAmbientIntensity() override;
  // cppcheck-suppress uselessOverride
  void updateIntensity() override;

private:
  // user accessible fields
  OmSFVector3 *mAttenuation;
  OmSFVector3 *mLocation;
  OmSFDouble *mRadius;

  QMap<QString, OmVector3> mSavedLocation;

  OmPointLight &operator=(const OmPointLight &);  // non copyable
  OmNode *clone() const override { return new OmPointLight(*this); }

  void init();
  void checkAmbientAndAttenuationExclusivity();

private slots:
  void updateAttenuation();
  void updateLocation();
  void updateRadius();
};

#endif
