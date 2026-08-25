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

#ifndef OM_INDEXED_LINE_SET_HPP
#define OM_INDEXED_LINE_SET_HPP

#include "OmGeometry.hpp"

class OmCoordinate;

class OmIndexedLineSet : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmIndexedLineSet(OmTokenizer *tokenizer = NULL);
  OmIndexedLineSet(const OmIndexedLineSet &other);
  explicit OmIndexedLineSet(const OmNode &other);
  virtual ~OmIndexedLineSet() override;

  // field accessors
  OmCoordinate *coord() const;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_INDEXED_LINE_SET; }
  void postFinalize() override;
  void createWrenObjects() override;
  void rescale(const OmVector3 &scale) override {}
  void reset(const QString &id) override;

  // ray tracing
  void recomputeBoundingSphere() const override;
  bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const override { return false; }
  double computeDistance(const OmRay &ray) const override { return -1; }

  // resize manipulator
  bool hasResizeManipulator() const override { return false; }

  // friction
  OmVector3 computeFrictionDirection(const OmVector3 &normal) const override;

  QStringList fieldsToSynchronizeWithW3d() const override;

protected:
  // reimplemented protected functions
  bool isShadedGeometryPickable() override { return false; }

private:
  // user accessible fields
  OmSFNode *mCoord;
  OmMFInt *mCoordIndex;

  bool sanitizeFields();

  OmIndexedLineSet &operator=(const OmIndexedLineSet &);  // non copyable
  OmNode *clone() const override { return new OmIndexedLineSet(*this); }
  void init();

  int estimateIndexCount(bool isOutlineMesh = false) const;

private slots:
  void updateCoord();
  void updateCoordIndex();
};

#endif
