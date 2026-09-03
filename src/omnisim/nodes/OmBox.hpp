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

#ifndef OM_BOX_HPP
#define OM_BOX_HPP

#include "OmGeometry.hpp"

class OmSFVector3;

class OmBox : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmBox(OmTokenizer *tokenizer = NULL);
  OmBox(const OmBox &other);
  explicit OmBox(const OmNode &other);
  virtual ~OmBox() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_BOX; }
  void postFinalize() override;
  bool createOdeGeom() override;
  void createWrenObjects() override;
  void createResizeManipulator() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  void rescale(const OmVector3 &scale) override;

  // field accessors
  const OmVector3 &size() const;
  const OmVector3 scaledSize() const;

  // field setters
  void setSize(const OmVector3 &size);
  void setSize(double x, double y, double z);
  void setX(double x);
  void setY(double y);
  void setZ(double z);

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override;
  double computeDistance(const OmRay &ray) const override;

  // friction
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override;

  // Non-recursive texture mapping
  OmVector2 nonRecursiveTextureSizeFactor() const override { return OmVector2(4, 2); }

  static OmVector2 computeTextureCoordinate(const OmVector3 &minBound, const OmVector3 &maxBound, const OmVector3 &point,
                                            bool nonRecursive, int intersectedFace = -1);
  static int findIntersectedFace(const OmVector3 &minBound, const OmVector3 &maxBound, const OmVector3 &intersectionPoint);

  // resize manipulator
  void setResizeManipulatorDimensions() override;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;

  // Fluid

private:
  OmBox &operator=(const OmBox &);  // non copyable
  OmNode *clone() const override { return new OmBox(*this); }
  void init();

  // user accessible fields
  OmSFVector3 *mSize;

  bool sanitizeFields();

  // ODE

  // ray tracing
  // compute collision point and return distance
  double computeLocalCollisionPoint(OmVector3 &point, const OmRay &ray) const;

private slots:
  void updateSize();
};

#endif
