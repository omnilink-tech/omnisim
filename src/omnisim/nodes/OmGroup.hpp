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

#ifndef OM_GROUP_HPP
#define OM_GROUP_HPP

#include "OmBaseNode.hpp"
#include "OmHiddenKinematicParameters.hpp"
#include "OmMFNode.hpp"

class OmBoundingSphere;

class OmGroup : public OmBaseNode {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmGroup(OmTokenizer *tokenizer = NULL);
  OmGroup(const OmGroup &other);
  explicit OmGroup(const OmNode &other);
  virtual ~OmGroup() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_GROUP; }
  void downloadAssets() override;
  void preFinalize() override;
  void postFinalize() override;
  void createOdeObjects() override;
  void createWrenObjects() override;
  void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) override;
  void setSleepMaterial() override;
  void setScaleNeedUpdate() override;
  bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const override;
  bool isSuitableForInsertionInBoundingObject(bool warning = false) const override;
  bool shallExport() const override;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const override;

  // field accessors
  int childCount() const { return mChildren->size(); }
  OmBaseNode *child(int index) const;
  int nodeIndex(OmNode *child) const;
  const OmMFNode &children() const { return *mChildren; }
  const OmMFNode *childrenField() const { return mChildren; }

  // append at the end
  void addChild(OmNode *child);

  // insert a child at the specified index
  void insertChild(int index, OmNode *child);

  // set a child at the specified index
  void setChild(int index, OmNode *child);

  // remove all children without deleting them
  void clear();

  // remove and delete all children
  void deleteAllChildren();

  // remove and delete all solid children
  virtual void deleteAllSolids();

  // utility forward functions if the group/transform node has no solid ancestor
  // forward jerk notification to children
  virtual void forwardJerk();
  void writeParameters(OmWriter &writer) const override;
  virtual void collectHiddenKinematicParameters(OmHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                                int &counter) const;
  virtual bool resetHiddenKinematicParameters();
  virtual bool restoreHiddenKinematicParameters(const OmHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                                int &counter);
  void readHiddenKinematicParameter(OmField *field) override;

  // selection
  void propagateSelection(bool selected) override;

  // propagate change in segmentation color
  void updateSegmentationColor(const OmRgb &color) override;

  // bounding sphere
  OmBoundingSphere *boundingSphere() const override { return mBoundingSphere; }
  void recomputeBoundingSphere();

  // lazy matrix multiplication system
  void setMatrixNeedUpdate() override;

  // export
  void exportBoundingObjectToW3d(OmWriter &writer) const override;

signals:
  // called after the list of children has changed
  void childrenChanged();
  void topLevelListsUpdateRequested();
  void childAdded(OmBaseNode *child);
  void finalizedChildAdded(OmBaseNode *child);  // emit signal when inserting child after current node is finalized
  void notifyParentSlot(OmBaseNode *child);
  void notifyParentJoint(OmBaseNode *child);
  void childFinalizationHasProgressed(const int progress);  // 0: beginning, 100: end
  void worldLoadingStatusHasChanged(QString status);

protected:
  // this constructor is reserved for derived classes only
  OmGroup(const QString &modelName, OmTokenizer *tokenizer);

  // called when a node is inserted in the children of this group
  // or in the children of the children of this group, etc.
  virtual void descendantNodeInserted(OmBaseNode *decendant);

  // utility fields if the group/transform node has no solid ancestor
  bool mHasNoSolidAncestor;
  OmHiddenKinematicParameters::HiddenKinematicParametersMap mHiddenKinematicParametersMap;

  mutable OmBoundingSphere *mBoundingSphere;

private:
  OmGroup &operator=(const OmGroup &);  // non copyable
  OmNode *clone() const override { return new OmGroup(*this); }
  void init();


  // user accessible fields
  OmMFNode *mChildren;
  int mLoadProgress;

public slots:
  void cancelFinalization();
  void insertChildFromSlotOrJoint(OmBaseNode *decendant);

private slots:
  void insertChildPrivate(int index);
  void monitorChildFinalization(OmBaseNode *child);
};

#endif
