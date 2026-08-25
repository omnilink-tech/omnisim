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

#ifndef OM_SPHERE_HPP
#define OM_SPHERE_HPP

#include "OmGeometry.hpp"
#include "OmSFDouble.hpp"

class OmVector3;

class OmSphere : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSphere(OmTokenizer *tokenizer = NULL);
  OmSphere(const OmSphere &other);
  explicit OmSphere(const OmNode &other);
  virtual ~OmSphere() override;

  // field accessors
  double radius() const { return mRadius->value(); }
  double scaledRadius() const;

  // field setters
  void setRadius(double r) { mRadius->setValue(r); }

  // reimplemented functions
  int nodeType() const override { return WB_NODE_SPHERE; }
  void postFinalize() override;
  void createWrenObjects() override;
  dGeomID createOdeGeom(dSpaceID space) override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void rescale(const OmVector3 &scale) override;

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
  OmSphere &operator=(const OmSphere &);  // non copyable
  OmNode *clone() const override { return new OmSphere(*this); }
  void init();

  // user accessible fields
  OmSFDouble *mRadius;
  OmSFInt *mSubdivision;
  OmSFBool *mIco;

  bool sanitizeFields();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;

  // ray tracing
  bool computeCollisionPoint(OmVector3 &point, const OmRay &ray) const;

private slots:
  void updateRadius();
  void updateMesh();
};

#endif
