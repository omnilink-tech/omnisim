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

#ifndef OM_GEOMETRY_HPP
#define OM_GEOMETRY_HPP

//
// Description: abstract base class for all geometry primitives
// Inherited by:
//   OmBox, OmSphere, OmPlane, OmCapsule, OmCylinder, OmCone,
//   OmElevationGrid, OmIndexedFaceSet, OmIndexedLineSet and OmMesh
//

#include "OmBaseNode.hpp"
#include "OmMatrix3.hpp"
#include "OmVector2.hpp"

class OmBoundingSphere;
class OmMatrix4;
class OmMatter;
class OmRay;
class OmRgb;
class OmWrenAbstractResizeManipulator;


class OmGeometry : public OmBaseNode {
  Q_OBJECT

public:
  // Destructor
  virtual ~OmGeometry() override;

  // Reimplemented public functions
  void postFinalize() override;
  void createOdeObjects() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  void propagateSelection(bool selected) override;

  virtual bool hasDefaultMaterial() { return false; }
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setPickable(bool pickable);

  // WREN

  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override {
    return QList<const OmBaseNode *>() << this;
  }

  // Called when this geometry enters a boundingObject: validates it as a
  // collider (warns and returns false on a bad size etc.) and prepares any
  // native collider data. The collider itself is registered with Newton from
  // the boundingObject walk in OmSolid::flushPendingNewtonRegistrations.
  virtual bool createOdeGeom();
  virtual void setOdePosition(const OmVector3 &translation);
  virtual void setOdeRotation(const OmMatrix3 &rotation);

  virtual void applyToOdeData(bool correctSolidMass = true) {}  // Resize the ODE dGeom stored in a bounding OmGeometry

  // Ray tracing
  OmBoundingSphere *boundingSphere() const override { return mBoundingSphere; }
  virtual void recomputeBoundingSphere() const = 0;
  virtual bool pickUVCoordinate(OmVector2 &uv, const OmRay &ray, int textureCoordSet = 0) const = 0;
  virtual double computeDistance(const OmRay &ray) const = 0;

  // friction
  virtual OmVector3 computeFrictionDirection(const OmVector3 &normal) const = 0;

  // Non-recursive texture mapping
  virtual OmVector2 nonRecursiveTextureSizeFactor() const { return OmVector2(1, 1); }

  virtual void rescale(const OmVector3 &scale) = 0;
  const OmVector3 absoluteScale() const;
  OmVector3 absolutePosition() const;

  // ODE collision info
  void setColliding();

  // Position and orientation matrix of the upper transform
  OmMatrix4 matrix() const;

  // Scale handles constraints
  int constraintType() const;

  // resize manipulator
  bool hasResizeManipulator() const override { return areSizeFieldsVisibleAndNotRegenerator(); }
  OmWrenAbstractResizeManipulator *resizeManipulator();
  bool isResizeManipulatorAttached() const;
  void attachResizeManipulator() override;
  void detachResizeManipulator() const override;
  void updateResizeHandlesSize() override;
  virtual void createResizeManipulatorIfNeeded();
  virtual void setResizeManipulatorDimensions() {}
  void setUniformConstraintForResizeHandles(bool enabled) override;

  // export
  void exportBoundingObjectToW3d(OmWriter &writer) const override;

  static int maxIndexNumberToCastShadows();
  int triangleCount() const;

  // visibility
  void setTransparent(bool isTransparent);
  bool isTransparent() const { return mIsTransparent; }

signals:
  void changed();
  void wrenObjectsCreated();
  void boundingGeometryRemoved();

public slots:
  void showResizeManipulator(bool enabled) override;

protected:
  bool exportNodeHeader(OmWriter &writer) const override;

  static const float LINE_SCALE_FACTOR;

  // All constructors are reserved for derived classes only
  OmGeometry(const OmGeometry &other);
  OmGeometry(const OmNode &other);
  OmGeometry(const QString &modelName, OmTokenizer *tokenizer);

  // for bounding object representation use a subdivision >= MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION
  // so that it is clear for the user that the ODE object is a real rounded shape and not an approximation as the graphical mesh
  const int MIN_BOUNDING_OBJECT_CIRCLE_SUBDIVISION = 16;


  // Selection
  bool isSelected();
  bool isPickable() { return mPickable; }

  // query flags
  virtual bool isShadedGeometryPickable() { return true; }

  // check if the fields that resize the geometry are visible in the scene tree
  virtual bool areSizeFieldsVisibleAndNotRegenerator() const { return false; }

  // Scaling
  bool mIs90DegreesRotated;  // rotate ElevationGrid by 90 degrees: ODE to FLU rotation
  OmVector3 mLocalOdeGeomOffsetPosition;
  OmBaseNode *transformedGeometry();
  // Fluid

  // Ray tracing
  mutable OmBoundingSphere *mBoundingSphere;

  // Resize handles
  OmWrenAbstractResizeManipulator *mResizeManipulator;  // Set of handles allowing resize by dragging the mouse
  bool mResizeManipulatorInitialized;
  int mResizeConstraint;

private:
  OmGeometry &operator=(const OmGeometry &);  // non copyable
  // Only derived classes can be cloned
  OmNode *clone() const override = 0;

  void init();

  virtual void createResizeManipulator() {}

  // ODE info
  double mCollisionTime;          // milliseconds
  double mPreviousCollisionTime;  // milliseconds

  OmVector3 mOdeOffsetTranslation;
  OmVector3 mOdePositionSet;
  OmMatrix3 mOdeOffsetRotation;

  bool mPickable;
  bool mIsTransparent;

};

#endif
