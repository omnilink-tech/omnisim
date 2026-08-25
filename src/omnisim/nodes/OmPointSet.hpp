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

#ifndef OM_POINT_SET_HPP
#define OM_POINT_SET_HPP

#include "OmGeometry.hpp"

class OmCoordinate;
class OmColor;

class OmPointSet : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPointSet(OmTokenizer *tokenizer = NULL);
  OmPointSet(const OmPointSet &other);
  explicit OmPointSet(const OmNode &other);
  virtual ~OmPointSet() override;

  // field accessors
  OmCoordinate *coord() const;
  OmColor *color() const;

  // reimplemented public functions
  virtual int nodeType() const override { return WB_NODE_POINT_SET; }
  virtual void postFinalize() override;
  virtual void createWrenObjects() override;
  virtual void rescale(const OmVector3 &scale) override {}

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override { return false; }
  double computeDistance(const OmRay &ray) const override { return -1; }

  // resize manipulator
  bool hasResizeManipulator() const override { return false; }

  // friction (PointSet never used in a boundingObject)
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override { return OmVector3(0, 0, 0); }

protected:
  // reimplemented protected functions
  bool isShadedGeometryPickable() override { return false; }

private:
  // user accessible fields
  OmSFNode *mCoord;
  OmSFNode *mColor;

  bool sanitizeFields();

  OmPointSet &operator=(const OmPointSet &);  // non copyable
  OmNode *clone() const override { return new OmPointSet(*this); }
  void init();

private slots:
  void updateCoord();
  void updateColor();
};

#endif
