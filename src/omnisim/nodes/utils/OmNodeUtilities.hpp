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

#ifndef OM_NODE_UTILITIES_HPP
#define OM_NODE_UTILITIES_HPP

//
// Description: utility class allowing to query OmBaseNode instances in their tree context
//              through static functions.
//              For generic node functions please refer to OmVrmlNodeUtilities namespace.
//

#include "OmNode.hpp"
#include "OmNodeModel.hpp"
#include "OmProtoModel.hpp"

#include <QtCore/QList>

class OmBaseNode;
class OmBoundingSphere;
class OmField;
class OmMatter;
class OmRay;
class OmRobot;
class OmShape;
class OmSolid;
class OmPose;
class OmTransform;

class QString;

namespace OmNodeUtilities {

  //////////////////////////
  // Permanent properties //
  //////////////////////////

  void fixBackwardCompatibility(OmNode *node);

  // find the closest OmTransform ancestor
  OmTransform *findUpperTransform(const OmNode *node);

  // find the closest OmPose ancestor
  OmPose *findUpperPose(const OmNode *node);

  // find the closest OmSolid ancestor
  OmSolid *findUpperSolid(const OmNode *node);

  // find the closest OmMatter ancestor
  OmMatter *findUpperMatter(const OmNode *node);

  // find the closest ancestor of specified type
  // searchDegree specifies how many ancestor have to be checked, if lower or equal to 0 all the hierarchy is inspected
  OmNode *findUpperNodeByType(const OmNode *node, int nodeType, int searchDegrees = 0);

  // return if this node contains descendant nodes of the specified types
  bool hasDescendantNodesOfType(const OmNode *node, const QList<int> &nodeTypes);

  // return all the descendant nodes fulfilling the specified type condition
  // typeCondition is a function that checks the type of the node
  // if recursive is set to FALSE children of the descendant node having the specified type are not inspected
  QList<OmNode *> findDescendantNodesOfType(OmNode *node, bool (&typeCondition)(OmBaseNode *), bool recursive);

  // find the uppermost OmPose ancestor (may be the node itself)
  OmPose *findUppermostPose(const OmNode *node);

  // find the uppermost OmSolid ancestor (may be the node itself)
  OmSolid *findUppermostSolid(const OmNode *node);

  // find the uppermost OmMatter ancestor (may be the node itself)
  OmMatter *findUppermostMatter(OmNode *node);

  // find the top node and return it if it is a OmSolid, return NULL otherwise
  OmSolid *findTopSolid(const OmNode *node);

  // find a robot ancestor above the node in the scene tree, return NULL if no robot found
  OmRobot *findRobotAncestor(const OmNode *node);

  // return direct Solid descendant nodes
  // in case of PROTO nodes only internal nodes are checked
  QList<OmSolid *> findSolidDescendants(OmNode *node);

  // is this node located directly or indirectly under a Billboard
  bool isDescendantOfBillboard(const OmNode *node);

  // is this node located directly or indirectly under a Propeller
  bool isDescendantOfPropeller(const OmNode *node);

  // is this node located in the boundingObject field of a Solid
  // use checkNodeUse() to inspect USE nodes and PROTO parameter instances
  bool isInBoundingObject(const OmNode *node);

  // check if node is used in a boundingObject field and/or in the global structure
  OmNode::NodeUse checkNodeUse(const OmNode *n);

  // find the OmMatter ancestor whose boundingObject field contains this node
  OmMatter *findBoundingObjectAncestor(const OmBaseNode *node);

  // is this node a valid USEable node
  bool isAValidUseableNode(const OmNode *node, QString *warning = NULL);

  // return closest OmMatter ancestor that is visible in the scene tree (given node included)
  OmMatter *findUpperVisibleMatter(OmNode *node);

  // is the target field or the target parameter field a template regenerator field
  bool isTemplateRegeneratorField(const OmField *field);

  //////////////////////////////
  // Non-permanent properties //
  //////////////////////////////

  // has this node a robot ancestor
  bool hasARobotAncestor(const OmNode *node);

  // has this node a Robot node descendant
  bool hasARobotDescendant(const OmNode *node);

  // has this node a Device node descendant
  // Connector node often needs to be ignored as it can be passive and inserted in non-robot nodes
  bool hasADeviceDescendant(const OmNode *node, bool ignoreConnector);

  // has this node a Solid node descendant
  bool hasASolidDescendant(const OmNode *node);

  // has this node a Joint node descendant
  bool hasAJointDescendant(const OmNode *node);

  // is this node selected
  bool isSelected(const OmNode *node);

  // is this node or a OmMatter ancestor of the current node locked
  bool isNodeOrAncestorLocked(const OmNode *node);

  // tests node types
  bool isGeometryTypeName(const QString &modelName);
  bool isCollisionDetectedGeometryTypeName(const QString &modelName);
  bool isRobotTypeName(const QString &modelName);
  bool isDeviceTypeName(const QString &modelName);
  bool isSolidDeviceTypeName(const QString &modelName);
  bool isSolidTypeName(const QString &modelName);
  bool isMatterTypeName(const QString &modelName);
  QString slotType(const OmNode *node);

  bool isTrackAnimatedGeometry(const OmNode *node);


  ///////////
  // Other //
  ///////////

  // find intersecting Shape
  const OmShape *findIntersectingShape(const OmRay &ray, double maxDistance, double &distance, double minDistance = 0.0);

  // validate a new inserted node
  // this functions helps handling properly the validation of a Slot node
  // return false if the Slot structure is invalid and insertion should be aborted
  bool validateInsertedNode(OmField *field, const OmNode *newNode, const OmNode *parentNode, bool isInBoundingObject);

  // check if a new node with the given parameters can be inserted in the field 'field' of parent node 'node'
  // in case of PROTO parent node and parameter field,
  // it first retrieve the base field and model and then check the validity
  // type is checked in case of Slot node
  bool isAllowedToInsert(const OmField *const field, const OmNode *node, QString &errorMessage, OmNode::NodeUse nodeUse,
                         const QString &type, const QString &newNodeModelName, const OmNodeModel *newNodeBaseModel,
                         const QStringList &newNodeProtoParentList, bool automaticBoundingObjectCheck = true);
  inline bool isAllowedToInsert(const OmField *const field, const OmNode *node, QString &errorMessage, OmNode::NodeUse nodeUse,
                                const QString &type, const QString &newNodeBaseModelName, const QString &newNodeModelName,
                                const QStringList &newNodeProtoParentList, bool automaticBoundingObjectCheck = true) {
    return isAllowedToInsert(field, node, errorMessage, nodeUse, type, newNodeModelName,
                             OmNodeModel::findModel(newNodeBaseModelName), newNodeProtoParentList,
                             automaticBoundingObjectCheck);
  }
  inline bool isAllowedToInsert(const OmField *const field, const OmNode *node, QString &errorMessage, OmNode::NodeUse nodeUse,
                                const QString &type, const OmNode *newNode, bool automaticBoundingObjectCheck = true) {
    return isAllowedToInsert(field, node, errorMessage, nodeUse, type, newNode->modelName(), newNode->model(),
                             newNode->isProtoInstance() ? newNode->proto()->parentProtoNames() : QStringList(),
                             automaticBoundingObjectCheck);
  }

  // check existing node structure
  bool validateExistingChildNode(const OmField *const field, const OmNode *childNode, const OmNode *node,
                                 bool isInBoundingObject, QString &errorMessage);

  // can srcNode be transformed
  // hasDeviceDescendant expected values: {-1: not computed, 0: doesn't have device descendants, 1: has device descendants)
  enum Answer { SUITABLE, UNSUITABLE, LOOSING_INFO };
  Answer isSuitableForTransform(const OmNode *srcNode, const QString &destModelName, int *hasDeviceDescendantFlag);

  // check if type of two Slot nodes is compatible
  bool isSlotTypeMatch(const QString &firstType, const QString &secondType, QString &errorMessage);

  // return a node's bounding sphere ancestor if it exists (can be the node's own)
  OmBoundingSphere *boundingSphereAncestor(const OmNode *node);

};  // namespace OmNodeUtilities

#endif
