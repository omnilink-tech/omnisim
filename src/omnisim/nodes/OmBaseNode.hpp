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

#ifndef OM_BASE_NODE_HPP
#define OM_BASE_NODE_HPP

//
// Description: abstract base class for all types of nodes in Webots
//
// Inherited by:
//   Directly or indirectly by every node class in the 'nodes' subproject
//

#include "../../../include/controller/c/omnisim/nodes.h"
#include "OmNode.hpp"

#include "OmRgb.hpp"

class OmPose;
class OmTransform;
class OmSolid;
class OmBoundingSphere;
class OmMatrix3;
class OmVector3;


class OmBaseNode : public OmNode {
  Q_OBJECT

public:
  // destructor
  virtual ~OmBaseNode() override;

  virtual void downloadAssets() {}
  // finalize() assumes that the whole world node/field structure is complete
  void finalize();
  virtual void preFinalize() {
    mPreFinalizeCalled = true;
    setCreationCompleted();
  }
  virtual void postFinalize();
  virtual void validateProtoNodes();
  virtual void validateProtoNode() {}
  bool isPreFinalizedCalled() const { return mPreFinalizeCalled; }
  bool isPostFinalizedCalled() const { return mPostFinalizeCalled; }
  void reset(const QString &id) override;

  // for libController
  virtual int nodeType() const = 0;

  // updates material of all OmGeometry descendants lying into a bounding object
  // reimplemented in OmGroup (recurse through all children), OmPose, OmShape and OmGeometry
  virtual void updateCollisionMaterial(bool triggerChange = false, bool onSelection = false) {}
  virtual void setSleepMaterial() {}

  // reimplemented in OmGroup (recurse through all children), OmPropeller, OmShape and OmGeometry
  virtual void propagateSelection(bool selected) {}

  // Method used to cache absolute scale values
  // reimplemented in OmGroup (recurse through all children), OmPose, OmSolid, OmShape and OmIndexedFaceSet
  virtual void setScaleNeedUpdate() {}

  // informs all children that their matrices need to be recomputed (inherited from OmGroup)
  // reimplemented in OmGroup (recurse through all children), OmPose, OmSolid, OmPropeller
  virtual void setMatrixNeedUpdate() {}

  // propagate segmentation color change reimplemented in OmGroup (recurse through all children), OmBasicJoint,
  // OmCadShape, OmShape, OmSlot and OmSolid
  virtual void updateSegmentationColor(const OmRgb &color) {}

  // Wren functions
  virtual void createWrenObjects();
  bool areWrenObjectsInitialized() const { return mWrenObjectsCreatedCalled; }

  // return the closest descendant node(s) with dedicated Wren node (may be the node itself)
  // only OmBillboard, OmGeometry, OmPose, and OmMuscle have a dedicated Wren node
  // used to properly apply Wren settings only to the current/descendant nodes and not to parent and sibling nodes
  virtual QList<const OmBaseNode *> findClosestDescendantNodesWithDedicatedWrenNode() const {
    return QList<const OmBaseNode *>();
  }

  // Ode functions
  virtual void createOdeObjects() { mOdeObjectsCreatedCalled = true; }
  bool areOdeObjectsCreated() const { return mOdeObjectsCreatedCalled; }

  // node as bounding object
  virtual bool isAValidBoundingObject(bool checkOde = false, bool warning = true) const { return false; }
  virtual bool isSuitableForInsertionInBoundingObject(bool warning = false) const { return false; }

  // Cached utility functions for permanent node properties
  bool isInBoundingObject() const;
  OmSolid *upperSolid() const;
  OmSolid *topSolid() const;
  OmPose *upperPose() const;
  OmTransform *upperTransform() const;

  // Cached function that can change if new USE nodes are added
  // return if this node or any of its instances is used in boundingObject
  OmNode::NodeUse nodeUse() const;

  // Ray tracing functions
  virtual OmBoundingSphere *boundingSphere() const { return NULL; }

  // resize/scale manipulator
  virtual bool hasResizeManipulator() const { return false; }
  virtual void attachResizeManipulator() {}
  virtual void detachResizeManipulator() const {}
  virtual void updateResizeHandlesSize() {}
  virtual void setUniformConstraintForResizeHandles(bool enabled) {}

  // only for PROTO instances
  // return the first finalized instance node of a PROTO (multiple finalized instances may exist)
  OmBaseNode *getFirstFinalizedProtoInstance() const;

  QString documentationUrl() const;

signals:
  void visibleHandlesChanged(bool resizeHandlesEnabled);
  void finalizationCompleted(OmBaseNode *node);

public slots:
  virtual void showResizeManipulator(bool enabled) {}

protected:
  bool isUrdfRootLink() const override;
  virtual OmVector3 urdfRotation(const OmMatrix3 &rotationMatrix) const;

  void exportUrdfJoint(OmWriter &writer) const override;

  // constructor:
  // if the tokenizer is NULL, then the node is constructed with the default field values
  // otherwise the field values are read from the tokenizer
  OmBaseNode(const QString &modelName, OmTokenizer *tokenizer);

  // copy constructor to be invoked from the copy constructors of derived classes
  // copies all the field values
  OmBaseNode(const OmBaseNode &other);
  OmBaseNode(const OmNode &other);

  // constructor for shallow nodes, should be used exclusively by the CadShape node
  OmBaseNode(const QString &modelName);

  void defHasChanged() override { finalize(); }
  void useNodesChanged() const override { mNodeUseDirty = true; };

  // semaphore used to cancel the finalization
  bool mFinalizationCanceled;

  bool isInvisibleNode() const;

  bool exportNodeHeader(OmWriter &writer) const override;

private:
  OmBaseNode &operator=(const OmBaseNode &);  // non copyable
  void init();

  // WREN

  // finalize booleans
  bool mPreFinalizeCalled;
  bool mPostFinalizeCalled;
  bool mWrenObjectsCreatedCalled;
  bool mOdeObjectsCreatedCalled;

  // Bounding object info
  // TODO: remove mutable keywords: this breaks the const functions
  //       possible improvement solutions (to investigate):
  //         -> migrate the search/cache code into OmNodeUtilities
  //         -> migrate the search/cache code into not const functions called when setting the parent
  mutable bool mIsInBoundingObject;
  mutable bool mBoundingObjectFirstTimeSearch;
  mutable OmPose *mUpperPose;
  mutable bool mUpperPoseFirstTimeSearch;
  mutable OmTransform *mUpperTransform;
  mutable bool mUpperTransformFirstTimeSearch;
  mutable OmSolid *mUpperSolid;
  mutable bool mUpperSolidFirstTimeSearch;
  mutable OmSolid *mTopSolid;
  mutable bool mTopSolidFirstTimeSearch;
  mutable OmNode::NodeUse mNodeUse;
  mutable bool mNodeUseDirty;
};

#endif
