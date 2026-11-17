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

#ifndef OM_CONE_HPP
#define OM_CONE_HPP

#include "OmGeometry.hpp"

class OmSFDouble;

class OmCone : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCone(OmTokenizer *tokenizer = NULL);
  OmCone(const OmCone &other);
  explicit OmCone(const OmNode &other);
  virtual ~OmCone() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CONE; }
  void postFinalize() override;
  void createWrenObjects() override;
  void createResizeManipulator() override;
  void rescale(const OmVector3 &scale) override;

  // Fields accessors
  double height() const;
  double scaledHeight() const;
  double bottomRadius() const;
  double scaledBottomRadius() const;

  // Fields setters
  void setHeight(double h);
  void setBottomRadius(double r);

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const OmRay &ray) const override;

  // friction
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override;

  // Non-recursive texture mapping
  OmVector2 nonRecursiveTextureSizeFactor() const override { return OmVector2(2, 1); }

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;

private:
  // user accessible fields
  OmSFDouble *mBottomRadius;
  OmSFDouble *mHeight;
  OmSFBool *mSide;
  OmSFBool *mBottom;
  OmSFInt *mSubdivision;

  bool sanitizeFields();

  OmCone &operator=(const OmCone &);  // non copyable
  OmNode *clone() const override { return new OmCone(*this); }
  void init();

  // ray tracing
  // compute collision point and return the distance
  double computeLocalCollisionPoint(OmVector3 &point, const OmRay &ray) const;

private slots:
  void updateBottomRadius();
  void updateHeight();
  void updateSide();
  void updateBottom();
  void updateSubdivision();
};

#endif
