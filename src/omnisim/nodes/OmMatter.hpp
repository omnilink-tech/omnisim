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

#ifndef OM_MATTER_HPP
#define OM_MATTER_HPP

#include "OmPose.hpp"
#include "OmSFBool.hpp"
#include "OmSFString.hpp"

class OmGeometry;

class OmMatter : public OmPose {
  Q_OBJECT

public:
  // constructors and destructor
  virtual ~OmMatter() override;

  // reimplemented public functions
  void createWrenObjects() override;
  void postFinalize() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  // field accessors
  const QString &name() const { return mName->value(); }
  const QString &model() const { return mModel->value(); }
  OmBaseNode *boundingObject() const;

  // ODE objects accessors
  dGeomID odeGeom() const;
  dSpaceID upperSpace() const;  // the smallest ODE space containing the boudingObject, possibly itself
  dSpaceID space() const;       // the largest space containing the boundingObject
  dSpaceID groupSpace() const;  // returns the ODE space associated to a Group if the bounding object is a group

  // collision flags
  bool isColliding() const;
  bool wasColliding() const;
  void setColliding();

  bool boundingObjectHasChanged() const { return mBoundingObjectHasChanged; }
  void setBoundingObjectFlag(bool changed) { mBoundingObjectHasChanged = changed; }
  bool hasAvalidBoundingObject() const { return boundingObject() && boundingObject()->isAValidBoundingObject(); }
  void setBoundingObject(OmNode *boundingObject);

  // for wb_supervisor_simulation_reset_physics()
  virtual void resetPhysics(bool recursive = true) {}
  virtual void pausePhysics(bool resumeAutomatically = false) {}
  virtual void resumePhysics() {}

  // handle artifical moves triggered by the user or a Supervisor
  virtual void jerk(bool resetVelocities = false, bool rootJerk = true) = 0;
  void forwardJerk() override { mNeedToHandleJerk = true; }

  // selection
  void select(bool selected);
  bool isSelected() const { return mSelected; }
  bool isLocked() const { return mLocked->value(); }

  // ODE positioning
  void updateOdeGeomPosition() { updateOdeGeomPosition(odeGeom()); }

  // Always false in practice: the single caller (OmMultimediaStreamingServer) passes false
  // and nothing else writes it, so a matter refreshes its overlays on deselection only.
  static void enableShowMatterCenter(bool enabled) { cShowMatterCenter = enabled; }

signals:
  void matterNameChanged();
  void matterModelChanged();

public slots:
  // recursions through bounding objects for material updates
  virtual void propagateBoundingObjectMaterialUpdate(bool onSelection = false) = 0;
  virtual void updateBoundingObject() = 0;

protected:
  // Abstract class: constructors are reserved for derived classes only
  OmMatter(const OmMatter &other);
  OmMatter(const OmNode &other);
  OmMatter(const QString &modelName, OmTokenizer *tokenizer);

  // Selection-driven overlay hooks: matter itself draws no overlay, OmSolid overrides both.
  virtual void applyVisibilityFlagsToWren(bool selected) {}
  virtual void applyChangesToWren() {}

  OmSFString *mName;
  OmSFNode *mBoundingObject;

  // Bounding Object
  bool isBoundingObjectFinalizationCompleted(OmBaseNode *node);
  bool checkBoundingObject() const;
  bool mBoundingObjectHasChanged;

  dGeomID createOdeGeomFromNode(dSpaceID space, OmBaseNode *node);
  dGeomID createOdeGeomFromBoundingObject(dSpaceID space);
  void updateOdeGeomPosition(dGeomID g);
  bool handleJerkIfNeeded();

  virtual bool isInsertedOdeGeomPositionUpdateRequired() const { return true; }

  // Sleep flag
  void updateSleepFlag();

protected slots:
  virtual void updateLineScale();
  void updateTranslation() override;
  void updateRotation() override;
  virtual void removeBoundingGeometry() {}
  virtual void boundingObjectFinalizationCompleted(OmBaseNode *node);
  virtual void updateName();

private:
  OmMatter &operator=(const OmMatter &);  // non copyable
  void init();

  OmSFString *mModel;
  OmSFString *mDescription;
  OmSFBool *mLocked;

  // Selection
  bool mSelected;

  bool mNeedToHandleJerk;

  void updateOdePlaceableGeomPosition(dGeomID g);
  virtual void handleJerk() = 0;

  // Update static geom position
  void updateOdePlanePosition(dGeomID plane);

  dGeomID createOdeGeomFromGroup(dSpaceID space, OmGroup *group);
  dGeomID createOdeGeomFromGeometry(dSpaceID space, OmGeometry *geometry, bool setOdeData = true);
  dGeomID createOdeGeomFromPose(dSpaceID space, OmPose *pose);
  void disconnectFromBoundingObjectUpdates(const OmNode *node) const;

  virtual void createOdeGeoms() = 0;

  virtual void setGeomMatter(dGeomID g, OmBaseNode *node = NULL) = 0;

  void connectNameUpdates() const;

  static bool cShowMatterCenter;

private slots:
  virtual void createOdeGeomFromInsertedGroupItem(OmBaseNode *node) = 0;
  void createOdeGeomFromInsertedPoseItem();
  void createOdeGeomFromInsertedShapeItem();
  void insertValidGeometryInBoundingObject();
  void updateManipulatorVisibility();
  void updateLocked();
};

#endif
