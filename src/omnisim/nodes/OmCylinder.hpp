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

#ifndef OM_CYLINDER_HPP
#define OM_CYLINDER_HPP

#include "OmGeometry.hpp"
#include "OmSFDouble.hpp"

class OmSFInt;

class OmCylinder : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmCylinder(OmTokenizer *tokenizer = NULL);
  OmCylinder(const OmCylinder &other);
  explicit OmCylinder(const OmNode &other);
  virtual ~OmCylinder() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_CYLINDER; }
  void postFinalize() override;
  void createWrenObjects() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void rescale(const OmVector3 &scale) override;

  // field accessors
  double radius() const { return mRadius->value(); }
  double scaledRadius() const;
  double height() const { return mHeight->value(); }
  double scaledHeight() const;

  // field setters
  void setRadius(double r) { mRadius->setValue(r); }
  void setHeight(double h) { mHeight->setValue(h); }

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const OmRay &ray) const override;

  // friction
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override;

  // Non-recursive texture mapping
  OmVector2 nonRecursiveTextureSizeFactor() const override { return OmVector2(2, 2); }

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;

private:
  OmCylinder &operator=(const OmCylinder &);  // non copyable
  OmNode *clone() const override { return new OmCylinder(*this); }
  void init();

  // user accessible fields
  OmSFBool *mBottom;
  OmSFDouble *mHeight;
  OmSFDouble *mRadius;
  OmSFBool *mSide;
  OmSFBool *mTop;
  OmSFInt *mSubdivision;

  bool sanitizeFields();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // ray tracing
  double computeLocalCollisionPoint(OmVector3 &point, int &faceIndex,
                                    const OmRay &ray) const;  // compute collision point and return the distance

private slots:
  void updateBottom();
  void updateRadius();
  void updateHeight();
  void updateSide();
  void updateTop();
  void updateSubdivision();
};

#endif
