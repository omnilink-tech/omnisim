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

#include "OmBaseNode.hpp"
#include "OmBasicJoint.hpp"
#include "OmBoundingSphere.hpp"
#include "OmDictionary.hpp"
#include "OmField.hpp"
#include "OmMatrix3.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmSolid.hpp"
#include "OmStandardPaths.hpp"
#include "OmTemplateManager.hpp"
#include "OmTransform.hpp"
#include "OmVector3.hpp"
#include "OmViewpoint.hpp"
#include "OmWorld.hpp"


OmVector3 OmBaseNode::urdfRotation(const OmMatrix3 &rotationMatrix) const {
  return rotationMatrix.toEulerAnglesZYX();
}

void OmBaseNode::init() {
  mPreFinalizeCalled = false;
  mPostFinalizeCalled = false;
  mWrenObjectsCreatedCalled = false;
  mOdeObjectsCreatedCalled = false;
  mIsInBoundingObject = false;
  mUpperPose = NULL;
  mUpperSolid = NULL;
  mTopSolid = NULL;
  mBoundingObjectFirstTimeSearch = true;
  mUpperPoseFirstTimeSearch = true;
  mUpperTransformFirstTimeSearch = true;
  mUpperSolidFirstTimeSearch = true;
  mTopSolidFirstTimeSearch = true;
  mFinalizationCanceled = false;
  mNodeUse = OmNode::UNKNOWN_USE;
  mNodeUseDirty = true;

  connect(this, &OmNode::parameterChanged, OmNodeOperations::instance(), &OmNodeOperations::updateExternProtoDeclarations);
}

OmBaseNode::OmBaseNode(const QString &modelName, OmTokenizer *tokenizer) :
  OmNode(modelName, OmWorld::instance() ? OmWorld::instance()->fileName() : "", tokenizer) {
  init();
}

OmBaseNode::OmBaseNode(const OmBaseNode &other) : OmNode(other) {
  init();
}

OmBaseNode::OmBaseNode(const OmNode &other) : OmNode(other) {
  init();
}

// special constructor for shallow nodes, it's used by CadShape to instantiate PBRAppearances from an assimp material in
// order to configure the WREN materials. Shallow nodes are invisible but persistent, and due to their incompleteness should not
// be modified or interacted with in any other way other than through the creation and destruction of CadShape nodes
OmBaseNode::OmBaseNode(const QString &modelName) : OmNode(modelName) {
  init();
}

OmBaseNode::~OmBaseNode() {
  if (mPostFinalizeCalled && !defName().isEmpty() && !OmWorld::instance()->isCleaning() && !OmTemplateManager::isRegenerating())
    OmDictionary::instance()->removeNodeFromDictionary(this);
}

void OmBaseNode::finalize() {
  finalizeProtoParametersRedirection();

  if (isProtoParameterNode()) {
    // finalize PROTO parameter node instances of the current node
    QVector<OmNode *> nodeInstances = protoParameterNodeInstances();
    OmBaseNode *baseNodeInstance = NULL;
    foreach (OmNode *nodeInstance, nodeInstances) {
      baseNodeInstance = dynamic_cast<OmBaseNode *>(nodeInstance);
      // recursive call to finalize nested parameter instances
      baseNodeInstance->finalize();
    }
    setFieldsParentNode();
    return;
  }

  if (!isPreFinalizedCalled())
    preFinalize();

  if (!areOdeObjectsCreated() && (OmWorld::instance()->isLoading() || !OmNodeUtilities::isTrackAnimatedGeometry(this)))
    // in case of nodes descending from Track.animatedGeometries field we don't want to create ODE objects
    // these nodes are automatically skipped if a Track or ancestor node is finalized, so we only have to check in case of node
    // insertion
    createOdeObjects();

  // D1.4 (WREN deletion): createWrenObjects() survives as the renderer-object init hook --
  // its per-node overrides keep their NON-WREN side effects (CPU pixel loads, registries,
  // child recursion) and areWrenObjectsInitialized() keeps doubling as the "node is fully
  // set up, caches may latch" flag half the tree reads. It now runs unconditionally: there
  // is no GL/WREN precondition left.
  if (!areWrenObjectsInitialized())
    createWrenObjects();

  if (mFinalizationCanceled)
    return;

  setFieldsParentNode();

  if (!isPostFinalizedCalled())
    postFinalize();

  validateProtoNodes();

  emit finalizationCompleted(this);
}

void OmBaseNode::postFinalize() {
  mPostFinalizeCalled = true;
  connect(this, &OmNode::defUseNameChanged, OmNodeOperations::instance(), &OmNodeOperations::requestUpdateSceneDictionary);
}

void OmBaseNode::validateProtoNodes() {
  QList<OmNode *> nodes = subNodes(true, false, false);
  nodes.prepend(this);

  foreach (OmNode *node, nodes) {
    if (node->isProtoInstance())
      dynamic_cast<OmBaseNode *>(node)->validateProtoNode();
  }
}

void OmBaseNode::reset(const QString &id) {
  OmNode::reset(id);
  OmBoundingSphere *const nodeBoundingSphere = boundingSphere();
  if (nodeBoundingSphere)
    nodeBoundingSphere->resetGlobalCoordinatesUpdateTime();
}

//////////////////////////
// WREN and ODE objects //
//////////////////////////

void OmBaseNode::createWrenObjects() {
  mWrenObjectsCreatedCalled = true;
}

// Utility functions
bool OmBaseNode::isInBoundingObject() const {
  if (mBoundingObjectFirstTimeSearch) {
    mIsInBoundingObject = OmNodeUtilities::isInBoundingObject(this);
    if (areWrenObjectsInitialized())
      mBoundingObjectFirstTimeSearch = false;
  }

  return mIsInBoundingObject;
}

OmNode::NodeUse OmBaseNode::nodeUse() const {
  if (mNodeUseDirty) {
    mNodeUse = OmNodeUtilities::checkNodeUse(this);
    if (areWrenObjectsInitialized())
      mNodeUseDirty = false;
  }

  return mNodeUse;
}

OmPose *OmBaseNode::upperPose() const {
  if (mUpperPoseFirstTimeSearch) {
    mUpperPose = OmNodeUtilities::findUpperPose(this);
    if (areWrenObjectsInitialized())
      mUpperPoseFirstTimeSearch = false;
  }

  return mUpperPose;
}

OmTransform *OmBaseNode::upperTransform() const {
  if (mUpperTransformFirstTimeSearch) {
    mUpperTransform = OmNodeUtilities::findUpperTransform(this);
    if (areWrenObjectsInitialized())
      mUpperTransformFirstTimeSearch = false;
  }

  return mUpperTransform;
}

OmSolid *OmBaseNode::upperSolid() const {
  if (mUpperSolidFirstTimeSearch) {
    mUpperSolid = OmNodeUtilities::findUpperSolid(this);
    if (areWrenObjectsInitialized())
      mUpperSolidFirstTimeSearch = false;
  }

  return mUpperSolid;
}

OmSolid *OmBaseNode::topSolid() const {
  if (mTopSolidFirstTimeSearch) {
    mTopSolid = OmNodeUtilities::findTopSolid(this);
    if (areWrenObjectsInitialized())
      mTopSolidFirstTimeSearch = false;
  }

  return mTopSolid;
}

OmBaseNode *OmBaseNode::getFirstFinalizedProtoInstance() const {
  QList<const OmNode *> nodes;  // stack containing other instances of the proto parameter node
                                // to be used in case of deeply nested PROTOs where the first one could not be finalized
  const OmBaseNode *baseNode = this;
  while (baseNode && !baseNode->isPostFinalizedCalled() && baseNode->isProtoParameterNode()) {
    // if node is a proto parameter node we need to find the corresponding proto parameter node instance
    // if the parameter is used multiple times all the instances are inspected in depth-first search (using the "nodes" list)
    const QVector<OmNode *> nodeInstances = baseNode->protoParameterNodeInstances();
    if (nodeInstances.isEmpty()) {
      if (nodes.isEmpty())
        return NULL;
      baseNode = static_cast<const OmBaseNode *>(nodes.takeFirst());
      continue;
    }
    baseNode = static_cast<const OmBaseNode *>(nodeInstances.at(0));
    for (int i = 0; i < nodeInstances.size(); ++i)
      nodes.append(nodeInstances.at(i));
  }

  return baseNode && baseNode->isPostFinalizedCalled() ? const_cast<OmBaseNode *>(baseNode) : NULL;
}

bool OmBaseNode::isInvisibleNode() const {
  return OmWorld::instance()->viewpoint()->getInvisibleNodes().contains(this);
}

QString OmBaseNode::documentationUrl() const {
  QStringList bookAndPage = documentationBookAndPage(OmNodeUtilities::isRobotTypeName(nodeModelName()));
  if (!bookAndPage.isEmpty())
    return QString("%1/%2/%3.md").arg(OmStandardPaths::omniSimDocsBaseUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);
  return QString();
}

bool OmBaseNode::exportNodeHeader(OmWriter &writer) const {
  if (!writer.isW3d())
    return OmNode::exportNodeHeader(writer);

  writer << "<" << w3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
  if (isInvisibleNode())
    writer << " render=\'false\'";
  QStringList bookAndPage = documentationBookAndPage(OmNodeUtilities::isRobotTypeName(nodeModelName()));
  if (!bookAndPage.isEmpty())
    writer
      << QString(" docUrl=\'%1/%2/%3.md\'").arg(OmStandardPaths::omniSimDocsBaseUrl()).arg(bookAndPage[0]).arg(bookAndPage[1]);

  if (isUseNode() && defNode()) {  // export referred DEF node id
    const OmNode *def = defNode();
    // cppcheck-suppress knownConditionTrueFalse
    if (def && def->isProtoParameterNode())
      def = static_cast<const OmBaseNode *>(def)->getFirstFinalizedProtoInstance();
    assert(def != NULL);
    writer << " USE=\'n" + QString::number(def->uniqueId()) + "\'/>";
    return true;
  }
  return false;
}

bool OmBaseNode::isUrdfRootLink() const {
  if (findSFString("name") || dynamic_cast<OmBasicJoint *>(parentNode()))
    return true;
  return false;
}

void OmBaseNode::exportUrdfJoint(OmWriter &writer) const {
  if (!parentNode() || dynamic_cast<OmBasicJoint *>(parentNode()))
    return;

  OmVector3 translation;
  OmVector3 eulerRotation;
  const OmNode *const upperLinkRoot = findUrdfLinkRoot();
  assert(upperLinkRoot);

  if (dynamic_cast<const OmPose *>(this) && dynamic_cast<const OmPose *>(upperLinkRoot)) {
    const OmPose *const upperLinkRootPose = static_cast<const OmPose *>(this);
    translation = upperLinkRootPose->translationFrom(upperLinkRoot);
    eulerRotation = urdfRotation(upperLinkRootPose->rotationMatrixFrom(upperLinkRoot));
  }

  translation += writer.jointOffset();
  writer.setJointOffset(OmVector3(0.0, 0.0, 0.0));

  writer.increaseIndent();
  writer.indent();
  writer << QString("<joint name=\"%1_%2_joint\" type=\"fixed\">\n").arg(upperLinkRoot->urdfName()).arg(urdfName());

  writer.increaseIndent();
  writer.indent();
  writer << QString("<parent link=\"%1\"/>\n").arg(upperLinkRoot->urdfName());
  writer.indent();
  writer << QString("<child link=\"%1\"/>\n").arg(urdfName());
  writer.indent();
  writer << QString("<origin xyz=\"%1\" rpy=\"%2\"/>\n")
              .arg(translation.toString(OmPrecision::FLOAT_ROUND_6))
              .arg(eulerRotation.toString(OmPrecision::FLOAT_ROUND_6));
  writer.decreaseIndent();

  writer.indent();
  writer << "</joint>\n";
  writer.decreaseIndent();
}
