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

#ifndef OM_POSE_HPP
#define OM_POSE_HPP

//
// Description: a node that defines a 3D coordinate system transformation
//
// Inherited by: OmSolid
//

#include "OmAbstractPose.hpp"
#include "OmGroup.hpp"
#include "OmShape.hpp"

class OmPose : public OmGroup, public OmAbstractPose {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmPose(OmTokenizer *tokenizer = NULL);
  OmPose(const OmPose &other);
  explicit OmPose(const OmNode &other);
  virtual ~OmPose() override;

  // reimplemented functions
  int nodeType() const override { return WB_NODE_POSE; }
  void preFinalize() override;
  void postFinalize() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool isColliding = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setMatrixNeedUpdate() override;
  void connectGeometryField(bool dynamic);
  void reset(const QString &id) override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override {
    return QList<const OmBaseNode *>() << this;
  }

  // accessors to stored fields
  const OmVector3 translationFromFile(const QString &id) const { return mSavedTranslations[id]; }
  const OmRotation rotationFromFile(const QString &id) const { return mSavedRotations[id]; }
  void setTranslationFromFile(const OmVector3 &translation) { mSavedTranslations[stateId()] = translation; }
  void setRotationFromFile(const OmRotation &rotation) { mSavedRotations[stateId()] = rotation; }

  void save(const QString &id) override;

  // update of ODE data stored in geometry() for OmPose lying into a boundingObject
  void applyToOdeData(bool correctMass = true);

  // for a Pose lying into a boundingObject
  void listenToChildrenField();
  inline OmGeometry *geometry() const;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = false) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;

  void enablePoseChangedSignal() const { mPoseChangedSignalEnabled = true; }
  void emitTranslationOrRotationChangedByUser() override;
  OmVector3 translationFrom(const OmNode *fromNode) const;
  OmMatrix3 rotationMatrixFrom(const OmNode *fromNode) const;

  // export
  void exportBoundingObjectToW3d(OmWriter &writer) const override;
  QStringList fieldsToSynchronizeWithW3d() const override;

public slots:
  virtual void updateRotation();
  virtual void updateTranslation();
  virtual void updateTranslationAndRotation();

signals:
  void geometryInPoseInserted();
  void poseChanged();
  void translationOrRotationChangedByUser();

protected:
  // this constructor is reserved for derived classes only
  OmPose(const QString &modelName, OmTokenizer *tokenizer);

  mutable bool mPoseChangedSignalEnabled;

private:
  OmPose &operator=(const OmPose &);  // non copyable
  OmNode *clone() const override { return new OmPose(*this); }
  void init();

  // Positions and orientations storage
  QMap<QString, OmVector3> mSavedTranslations;
  QMap<QString, OmRotation> mSavedRotations;

  void applyToOdeGeomRotation();
  void applyToOdeGeomPosition(bool correctMass = true);
  void applyToOdeMass(OmGeometry *g, dGeomID geom);
  void destroyPreviousOdeGeoms();
  OmShape *shape() const;

private slots:
  void createOdeGeom(int index = -1);
  void createOdeGeomIfNeeded();
  void notifyJerk();
};

inline OmGeometry *OmPose::geometry() const {
  if (childCount() == 0)
    return NULL;

  OmBaseNode *const c = child(0);

  if (c->nodeType() == WB_NODE_SHAPE)
    return static_cast<OmShape *>(c)->geometry();

  return dynamic_cast<OmGeometry *>(c);
}

#endif
