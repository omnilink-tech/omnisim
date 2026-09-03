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

#ifndef OM_ELEVATION_GRID_HPP
#define OM_ELEVATION_GRID_HPP

#include "OmGeometry.hpp"
#include "OmMFDouble.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFNode.hpp"

class OmVector3;

class OmElevationGrid : public OmGeometry {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmElevationGrid(OmTokenizer *tokenizer = NULL);
  OmElevationGrid(const OmElevationGrid &other);
  explicit OmElevationGrid(const OmNode &other);
  virtual ~OmElevationGrid() override;

  // field accessors
  // getters
  double xSpacing() const { return mXSpacing->value(); }
  double ySpacing() const { return mYSpacing->value(); }
  int xDimension() const { return mXDimension->value(); }
  int yDimension() const { return mYDimension->value(); }
  // ⚠ This returned `int` until 2026-08-14 while mHeight is an OmMFDouble, so every elevation was
  // TRUNCATED to a whole metre on the way out -- a terrain whose heights are all below 1.0 read back
  // as a flat plane at 0. Its one caller is OmObjectDetection.cpp (the camera-recognition / radar
  // occlusion point set), which therefore saw flat terrain; the Newton heightfield collider added the
  // same day is the second caller and would have been silently flattened too.
  double height(int index) const { return mHeight->item(index); }
  double heightRange() const { return mMaxHeight - mMinHeight; }
  // setters
  void setXspacing(double x) { mXSpacing->setValue(x); }
  void setYspacing(double y) { mYSpacing->setValue(y); }
  void setHeightScaleFactor(double ratio) { mHeight->multiplyAllItems(ratio); }

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_ELEVATION_GRID; }
  void preFinalize() override;
  void postFinalize() override;
  bool createOdeGeom() override;
  void createWrenObjects() override;
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

signals:
  void validElevationGridInserted();

protected:
  bool areSizeFieldsVisibleAndNotRegenerator() const override;

private:
  OmElevationGrid &operator=(const OmElevationGrid &);  // non copyable
  OmNode *clone() const override { return new OmElevationGrid(*this); }
  void init();

  // user accessible fields
  OmMFDouble *mHeight;
  OmSFInt *mXDimension;
  OmSFDouble *mXSpacing;
  OmSFInt *mYDimension;
  OmSFDouble *mYSpacing;
  OmSFDouble *mThickness;

  // other variables
  double mMinHeight;  // min value in "height" field
  double mMaxHeight;  // max value in "height" field
  double *mData;
  void checkHeight();
  double width() const { return mXSpacing->value() * (mXDimension->value() - 1); }
  double depth() const { return mYSpacing->value() * (mYDimension->value() - 1); }
  double height() const { return mMaxHeight - mMinHeight; }

  bool sanitizeFields();

  // ODE
  void applyToOdeData(bool correctSolidMass = true) override;
  bool setOdeHeightfieldData();
  double scaledWidth() const;
  double scaledDepth() const;
  double heightScaleFactor() const { return fabs(absoluteScale().y()); }

  // ray tracing
  double computeLocalCollisionPoint(const OmRay &ray, OmVector3 &localCollisionPoint) const;
  void computeMinMax(const OmVector3 &point, OmVector3 &min, OmVector3 &max) const;

private slots:
  void updateHeight();
  void updateXDimension();
  void updateXSpacing();
  void updateYDimension();
  void updateYSpacing();
  void updateThickness();
};

#endif
