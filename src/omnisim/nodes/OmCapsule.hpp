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

#ifndef OM_CAPSULE_HPP
#define OM_CAPSULE_HPP

#include "OmGeometry.hpp"
#include "OmSFDouble.hpp"

class OmCapsule : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCapsule(OmTokenizer *tokenizer = NULL);
  OmCapsule(const OmCapsule &other);
  explicit OmCapsule(const OmNode &other);
  virtual ~OmCapsule() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CAPSULE; }
  void postFinalize() override;
  void createWrenObjects() override;
  bool createOdeGeom() override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void rescale(const OmVector3 &scale) override;

  // field accessors
  double radius() const { return mRadius->value(); }
  double scaledRadius() const;
  double height() const { return mHeight->value(); }
  double scaledHeight() const;
  int subdivision() const;

  // field setters
  void setRadius(double r) { mRadius->setValue(r); }
  void setHeight(double h) { mHeight->setValue(h); }

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const OmRay &ray) const override;

  // friction
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override;

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;

private:
  // user accessible fields
  OmSFBool *mBottom;
  OmSFDouble *mRadius;
  OmSFDouble *mHeight;
  OmSFBool *mSide;
  OmSFBool *mTop;
  OmSFInt *mSubdivision;

  bool sanitizeFields();

  OmCapsule &operator=(const OmCapsule &);  // non copyable
  OmNode *clone() const override { return new OmCapsule(*this); }
  void init();

  // ODE

  // ray tracing
  double computeLocalCollisionPoint(OmVector3 &point,
                                    const OmRay &ray) const;  // compute the collison point and return the distance

private slots:
  void updateBottom();
  void updateRadius();
  void updateHeight();
  void updateSide();
  void updateTop();
  void updateSubdivision();
};

#endif
