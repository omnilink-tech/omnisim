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

#ifndef OM_PLANE_HPP
#define OM_PLANE_HPP

#include "OmGeometry.hpp"

class OmPlane : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPlane(OmTokenizer *tokenizer = NULL);
  OmPlane(const OmPlane &other);
  explicit OmPlane(const OmNode &other);
  virtual ~OmPlane() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_PLANE; }
  void postFinalize() override;
  void createWrenObjects() override;
  void createResizeManipulator() override;
  void rescale(const OmVector3 &scale) override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;

  bool createOdeGeom() override;
  void setOdePosition(const OmVector3 &translation) override;
  void setOdeRotation(const OmMatrix3 &matrix) override;

  void updateOdePlanePosition();

  // field accessors
  const OmVector2 &size() const;
  const OmVector2 scaledSize() const;

  // Setters
  void setSize(const OmVector2 &size);
  void setSize(double x, double y);
  void setX(double x);
  void setY(double y);

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
  OmSFVector2 *mSize;

  OmPlane &operator=(const OmPlane &);  // non copyable
  OmNode *clone() const override { return new OmPlane(*this); }
  void init();
  bool sanitizeFields();

  void computePlaneParams(OmVector3 &n, double &d);

  // ray tracing
  bool computeCollisionPoint(OmVector3 &point, const OmRay &ray) const;
  void computeMinMax(const OmVector3 &point, OmVector3 &min, OmVector3 &max) const;

private slots:
  void updateSize();
};

#endif
