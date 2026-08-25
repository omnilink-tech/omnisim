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

#include "OmSelection.hpp"

#include "OmNodeUtilities.hpp"
#include "OmSimulationWorld.hpp"
#include "OmSolid.hpp"
#include "OmTransform.hpp"
#include "OmWrenRenderingContext.hpp"

OmSelection *OmSelection::cInstance = NULL;

OmSelection::OmSelection() :
  QObject(),
  mSelectedAbstractPose(NULL),
  mSelectedNode(NULL),
  mResizeHandlesEnabledFromSceneTree(false) {
  assert(cInstance == NULL);
  cInstance = this;
}

OmSelection::~OmSelection() {
  cInstance = NULL;
}

OmSolid *OmSelection::selectedSolid() const {
  return dynamic_cast<OmSolid *>(mSelectedAbstractPose);
}

void OmSelection::selectNodeFromSceneTree(OmBaseNode *node) {
  if (mSelectedNode == node)
    return;
  selectNode(node);
  emit selectionChangedFromSceneTree(mSelectedAbstractPose);
  if (mSelectedAbstractPose)
    updateHandlesScale();
}

void OmSelection::selectPoseFromView3D(OmAbstractPose *p, bool handlesDisabled) {
  OmBaseNode *node = p == NULL ? NULL : p->baseNode();
  if (mSelectedNode == node)
    return;
  selectNode(node, handlesDisabled);
  emit selectionChangedFromView3D(mSelectedAbstractPose);
  if (mSelectedAbstractPose)
    updateHandlesScale();
}

void OmSelection::selectNode(OmBaseNode *n, bool handlesDisabled) {
  const bool poseChanged =
    ((n == NULL) != (mSelectedAbstractPose == NULL)) || (n == NULL || n != mSelectedAbstractPose->baseNode());
  if (mSelectedNode) {
    // unselect previously selected node
    updateMatterSelection(false);

    if (!mSelectedNode->isBeingDeleted()) {
      disconnect(mSelectedNode, &OmBaseNode::destroyed, this, &OmSelection::clear);
      setUniformConstraintForResizeHandles(false);
      mSelectedNode->detachResizeManipulator();

      if (mSelectedAbstractPose && poseChanged) {
        mSelectedAbstractPose->detachTranslateRotateManipulator();
        disconnect(mSelectedAbstractPose->baseNode(), &OmNode::isBeingDestroyed, this, &OmSelection::clear);
      }
    }
  }

  mSelectedNode = n;
  if (poseChanged)
    mSelectedAbstractPose = dynamic_cast<OmAbstractPose *>(mSelectedNode);
  mResizeHandlesEnabledFromSceneTree = false;

  if (mSelectedNode) {
    // select newly selected node
    connect(mSelectedNode, &OmBaseNode::destroyed, this, &OmSelection::clear, Qt::UniqueConnection);
    updateMatterSelection(true);

    if (mSelectedAbstractPose && poseChanged) {
      connect(mSelectedNode, &OmNode::isBeingDestroyed, this, &OmSelection::clear, Qt::UniqueConnection);
      if (!handlesDisabled && !mSelectedNode->isUseNode() &&
          !OmNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractPose->baseNode()))
        mSelectedAbstractPose->attachTranslateRotateManipulator();
    }
  }
}

void OmSelection::updateMatterSelection(bool selected) {
  if (mSelectedNode == NULL)
    return;

  OmMatter *matter = dynamic_cast<OmMatter *>(mSelectedNode);
  if (matter == NULL)
    // show bounding objects of parent OmMatter
    matter = OmNodeUtilities::findUpperMatter(mSelectedNode);

  if (matter != NULL && !matter->isBeingDeleted()) {
    matter->select(selected);
    if (selected) {
      // collision and sleep materials update
      if (!OmWrenRenderingContext::instance()->isOptionalRenderingEnabled(OmWrenRenderingContext::VF_ALL_BOUNDING_OBJECTS))
        matter->propagateBoundingObjectMaterialUpdate(selected);
    }
  }
}

void OmSelection::propagateBoundingObjectMaterialUpdate() {
  if (!mSelectedAbstractPose)
    return;
  OmMatter *const matter = dynamic_cast<OmMatter *>(mSelectedAbstractPose);
  if (matter != NULL)
    matter->propagateBoundingObjectMaterialUpdate();
}

bool OmSelection::isObjectMotionAllowed() const {
  if (!mSelectedAbstractPose)
    return false;

  OmBaseNode *topNode = mSelectedAbstractPose->baseNode();
  const OmAbstractPose *topPose = NULL;
  if (!topNode->isTopLevel()) {
    const OmSolid *solid = OmNodeUtilities::findUppermostSolid(topNode);
    if (solid)
      topPose = dynamic_cast<OmAbstractPose *>(topNode);
    else
      return false;
  } else
    topPose = dynamic_cast<OmAbstractPose *>(topNode);

  return topPose && !OmNodeUtilities::isNodeOrAncestorLocked(topNode) && topPose->isTranslationFieldVisible();
}

void OmSelection::clear() {
  if (mSelectedNode) {
    selectNode(NULL);
    emit selectionChangedFromSceneTree(NULL);
  }
}

void OmSelection::updateHandlesScale() {
  const OmTransform *t = dynamic_cast<const OmTransform *>(mSelectedNode);
  if (t && mResizeHandlesEnabledFromSceneTree && mSelectedNode)
    mSelectedNode->updateResizeHandlesSize();
  if (mSelectedAbstractPose)
    mSelectedAbstractPose->updateTranslateRotateHandlesSize();
}

void OmSelection::showResizeManipulatorFromSceneTree(bool enabled) {
  mSelectedNode->showResizeManipulator(enabled);
  mResizeHandlesEnabledFromSceneTree = enabled;
  emit visibleHandlesChanged();
}

void OmSelection::disableActiveManipulator() {
  if (!mSelectedAbstractPose)
    return;

  mSelectedAbstractPose->detachTranslateRotateManipulator();
  const OmTransform *t = dynamic_cast<const OmTransform *>(mSelectedAbstractPose);
  if (t)
    t->detachResizeManipulator();
}

void OmSelection::restoreActiveManipulator() {
  if (!mSelectedAbstractPose || mSelectedNode->isUseNode() ||
      OmNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractPose->baseNode()))
    return;

  if (mResizeHandlesEnabledFromSceneTree) {
    OmTransform *t = dynamic_cast<OmTransform *>(mSelectedAbstractPose);
    if (t)
      t->attachResizeManipulator();
  } else
    mSelectedAbstractPose->attachTranslateRotateManipulator();
}

bool OmSelection::showResizeManipulatorFromView3D(bool enabled) {
  // only AbstractPose descendants can be selected from the 3d view
  OmTransform *transform = dynamic_cast<OmTransform *>(mSelectedAbstractPose);
  if (!mSelectedAbstractPose || mResizeHandlesEnabledFromSceneTree || !mSelectedNode || mSelectedNode->isUseNode() ||
      (transform && !transform->hasResizeManipulator()) ||
      OmNodeUtilities::isNodeOrAncestorLocked(mSelectedAbstractPose->baseNode()))
    return false;

  if (enabled) {
    mSelectedAbstractPose->detachTranslateRotateManipulator();
    if (transform)
      transform->attachResizeManipulator();
  } else {
    if (transform)
      transform->detachResizeManipulator();
    mSelectedAbstractPose->attachTranslateRotateManipulator();
  }
  emit visibleHandlesChanged();
  return true;
}

void OmSelection::setUniformConstraintForResizeHandles(bool enable) {
  if (mResizeHandlesEnabledFromSceneTree) {
    mSelectedNode->setUniformConstraintForResizeHandles(enable);
    emit visibleHandlesChanged();
  }
}
