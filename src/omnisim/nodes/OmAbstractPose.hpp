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

#ifndef OM_ABSTRACT_POSE_HPP
#define OM_ABSTRACT_POSE_HPP

//
// Description: abstract node implementing 'translation', 'rotation' fields
//              functionalities and keeping the WREN::SceneNode up-to-date
//
// Inherited by: OmPose
//

#include <cassert>
#include "OmMatrix3.hpp"
#include "OmMatrix4.hpp"
#include "OmSFDouble.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector3.hpp"

class OmBaseNode;
class OmTranslateRotateManipulator;

class OmAbstractPose {
public:
  virtual ~OmAbstractPose();

  virtual OmBaseNode *baseNode() const { return mBaseNode; }

  // field accessors
  const OmVector3 &translation() const { return mTranslation->value(); }
  const OmRotation &rotation() const { return mRotation->value(); }
  OmSFVector3 *translationFieldValue() const { return mTranslation; }
  OmSFRotation *rotationFieldValue() const { return mRotation; }

  double translationStep() const { return mTranslationStep->value(); }
  double rotationStep() const { return mRotationStep->value(); }

  // setters
  void setTranslationAndRotation(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

  void setTranslationAndRotation(const OmVector3 &v, const OmRotation &r);
  void translate(double tx, double ty, double tz) {
    setTranslation(mTranslation->x() + tx, mTranslation->y() + ty, mTranslation->z() + tz);
  }
  void translate(const OmVector3 &v) { translate(v.x(), v.y(), v.z()); }
  void setTranslation(double tx, double ty, double tz);
  void setTranslationFromOde(double tx, double ty, double tz) { mTranslation->setValueFromOde(tx, ty, tz); }
  void setTranslation(const OmVector3 &v) { setTranslation(v.x(), v.y(), v.z()); }
  void setTranslationFromOde(const OmVector3 &v) { mTranslation->setValueFromOde(v.x(), v.y(), v.z()); }
  void rotate(const OmVector3 &v);
  void setRotation(double x, double y, double z, double angle);
  void setRotation(const OmRotation &r);
  void setRotationFromOde(const OmRotation &r) { mRotation->setValueFromOde(r); }
  void setRotationAngle(double angle);

  // 4x4 transform matrices
  const OmMatrix4 &matrix() const;
  virtual const OmMatrix4 &vrmlMatrix() const;
  OmVector3 xAxis() const { return matrix().xAxis(); }
  OmVector3 yAxis() const { return matrix().yAxis(); }
  OmVector3 zAxis() const { return matrix().zAxis(); }

  // 3x3 absolute rotation matrix
  OmMatrix3 rotationMatrix() const;

  const OmQuaternion &relativeQuaternion() const { return mRelativeQuaternion; }

  // position in 'world' coordinates
  OmVector3 position() const { return matrix().translation(); }

  // translate-rotate manipulator
  OmTranslateRotateManipulator *translateRotateManipulator() const { return mTranslateRotateManipulator; }
  virtual void updateTranslateRotateHandlesSize();
  virtual void attachTranslateRotateManipulator();
  virtual void detachTranslateRotateManipulator();

  // check if translation and rotation field is visible and don't trigger parameter node regeneration
  bool canBeTranslated() const;
  bool canBeRotated() const;
  bool isTranslationFieldVisible() const;
  bool isRotationFieldVisible() const;

  virtual void emitTranslationOrRotationChangedByUser() { assert(false); }

protected:
  void init(OmBaseNode *node);

  // all constructors are reserved for derived classes only
  explicit OmAbstractPose(OmBaseNode *node) { init(node); }

  // in OmTrackWheel fields are created instead of loading them
  OmSFVector3 *mTranslation;
  OmSFRotation *mRotation;
  OmSFDouble *mTranslationStep;
  OmSFDouble *mRotationStep;

  void setMatrixNeedUpdateFlag() const;
  void updateRotation();
  void updateTranslation();
  void updateTranslationAndRotation();

  OmTranslateRotateManipulator *mTranslateRotateManipulator;
  void createTranslateRotateManipulatorIfNeeded();
  bool mTranslateRotateManipulatorInitialized;

  void inline setTranslationAndRotationFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

  // WREN objects and methods
  void deleteWrenObjects();
  OmBaseNode *mBaseNode;
  mutable OmMatrix4 *mMatrix;
  mutable bool mMatrixNeedUpdate;
  mutable OmMatrix4 mVrmlMatrix;
  mutable bool mVrmlMatrixNeedUpdate;
  mutable OmQuaternion mRelativeQuaternion;

private:
  mutable bool mIsTranslationFieldVisible;
  mutable bool mIsRotationFieldVisible;
  mutable bool mIsTranslationFieldVisibleReady;
  mutable bool mIsRotationFieldVisibleReady;
  mutable bool mCanBeTranslated;
  mutable bool mCanBeRotated;
  void updateTranslationFieldVisibility() const;
  void updateRotationFieldVisibility() const;

  virtual void updateMatrix() const;
};

void inline OmAbstractPose::setTranslationAndRotationFromOde(double tx, double ty, double tz, double rx, double ry, double rz,
                                                             double angle) {
  mTranslation->setValueFromOde(tx, ty, tz);
  mRotation->setValueFromOde(rx, ry, rz, angle);
}

#endif
