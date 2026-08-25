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

#include "OmNodeUtilities.hpp"

#include "../../../include/controller/c/omnisim/nodes.h"
#include "OmBackground.hpp"
#include "OmBallJoint.hpp"
#include "OmBallJointParameters.hpp"
#include "OmBasicJoint.hpp"
#include "OmBillboard.hpp"
#include "OmBoundingSphere.hpp"
#include "OmBrake.hpp"
#include "OmCamera.hpp"
#include "OmCapsule.hpp"
#include "OmCone.hpp"
#include "OmConnector.hpp"
#include "OmCylinder.hpp"
#include "OmDevice.hpp"
#include "OmElevationGrid.hpp"
#include "OmEmitter.hpp"
#include "OmField.hpp"
#include "OmFog.hpp"
#include "OmHinge2Joint.hpp"
#include "OmJointParameters.hpp"
#include "OmLidar.hpp"
#include "OmLinearMotor.hpp"
#include "OmLog.hpp"
#include "OmLogicalDevice.hpp"
#include "OmMFNode.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeReader.hpp"
#include "OmPen.hpp"
#include "OmPlane.hpp"
#include "OmPositionSensor.hpp"
#include "OmProtoModel.hpp"
#include "OmRadar.hpp"
#include "OmReceiver.hpp"
#include "OmRobot.hpp"
#include "OmSelection.hpp"
#include "OmSimulationState.hpp"
#include "OmSlot.hpp"
#include "OmSolid.hpp"
#include "OmStandardPaths.hpp"
#include "OmTemplateManager.hpp"
#include "OmTokenizer.hpp"
#include "OmTouchSensor.hpp"
#include "OmTrack.hpp"
#include "OmTrackWheel.hpp"
#include "OmTransform.hpp"
#include "OmVersion.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"

#include <QtCore/QQueue>
#include <QtCore/QSet>
#include <QtCore/QStack>
#include <QtCore/QStringList>
#include <cassert>

namespace {
  static void sortNodeListForBackwardCompatibility(QList<OmNode *> &children);
  static QList<OmNode *> getNodeChildrenForBackwardCompatibility(const OmNode *node);
  static QList<OmNode *> getNodeChildrenAndBoundingForBackwardCompatibility(OmNode *node);

  QList<OmNode *> getNodeChildrenForBackwardCompatibility(const OmNode *node) {
    QList<OmNode *> children;
    const OmGroup *const nodeGroup = dynamic_cast<const OmGroup *>(node);
    if (!nodeGroup)
      return children;
    for (int i = 0; i < nodeGroup->childCount(); i++)
      children.append(nodeGroup->child(i));
    return children;
  }

  QList<OmNode *> getNodeChildrenAndBoundingForBackwardCompatibility(OmNode *node) {
    // Make a list of children to be rotated (children, TrackWheel children, bounding object with the group node ignored).
    QList<OmNode *> children = getNodeChildrenForBackwardCompatibility(node);
    QList<OmNode *> newChildren;
    OmNode *boundingObject = static_cast<OmSolid *>(node)->boundingObject();
    for (OmNode *child : children) {
      if (dynamic_cast<OmTrackWheel *>(child))
        newChildren += getNodeChildrenForBackwardCompatibility(child);
      else
        newChildren += child;
    }

    if (!dynamic_cast<OmPose *>(boundingObject) && !dynamic_cast<OmGeometry *>(boundingObject) &&
        !dynamic_cast<OmShape *>(boundingObject) && dynamic_cast<OmGroup *>(boundingObject))
      newChildren += boundingObject->subNodes(false, false);
    else if (boundingObject)
      newChildren.append(boundingObject);

    // Insert the USE nodes in the beginning.
    sortNodeListForBackwardCompatibility(newChildren);

    return newChildren;
  }

  void sortNodeListForBackwardCompatibility(QList<OmNode *> &children) {
    auto getNodeWeight = [](const OmNode *n) {
      // Higher number means higher priority.
      if (dynamic_cast<const OmGeometry *>(n))
        return 3;
      if (n->isDefNode())
        return 0;
      if (n->isUseNode())
        return 2;
      return 1;
    };
    std::sort(children.begin(), children.end(),
              [&getNodeWeight](const OmNode *a, const OmNode *b) -> bool { return getNodeWeight(a) > getNodeWeight(b); });
  }

  bool isAllowedToInsert(const QString &fieldName, const QString &nodeName, const OmNode *node, QString &errorMessage,
                         OmNode::NodeUse nodeUse, const QString &type, bool automaticBoundingObjectCheck = true,
                         bool areSlotAllowed = true) {
    errorMessage = QString();
    const QString defaultErrorMessage =
      QObject::tr("Cannot insert %1 node in '%2' field of %3 node.").arg(nodeName).arg(fieldName).arg(node->nodeModelName());

    if (!areSlotAllowed && nodeName == "Slot") {  // it is not allowed to insert slot->slot-slot
      errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node, because a trio of slot is not allowed.")
                       .arg(nodeName)
                       .arg(fieldName)
                       .arg(node->nodeModelName());
      return false;
    }

    // No robot can be inserted in helix of propellers.
    if (OmNodeUtilities::isRobotTypeName(nodeName) && OmNodeUtilities::isDescendantOfPropeller(node))
      return false;

    if (dynamic_cast<const OmSlot *>(node) && (fieldName == "endPoint")) {  // add something in the endPoint field of a slot
      if (dynamic_cast<const OmSlot *>(node->parentNode())) {  // pair of slots, we can add everything that is allowed in the
                                                               // children field of the parent of first Slot node
        OmNode *parentNode = node->parentNode();
        const OmNode *upperNode = parentNode->parentNode();
        const OmField *upperField = parentNode->parentField(true);
        if (!upperNode || !upperField) {
          assert(false);
          return false;
        }

        return ::isAllowedToInsert(upperField->name(), nodeName, upperNode, errorMessage, nodeUse, type,
                                   automaticBoundingObjectCheck, false);
      }

      // not in a pair => we can only add a Slot of same type
      QString slotType = OmNodeUtilities::slotType(node);
      // if node is a proto and is a parameter of another proto, the pair of slot is not detected
      if (OmNodeReader::current() && node->isProtoInstance() && node->parentNode() && node->parentNode()->isProtoInstance())
        return true;
      if (nodeName != "Slot") {
        errorMessage =
          QObject::tr("Cannot insert %1 node in '%2' field of %3 node: only a slot can be added in the parent slot.")
            .arg(nodeName)
            .arg(fieldName)
            .arg(node->nodeModelName());
        return false;
      }

      bool valid = OmNodeUtilities::isSlotTypeMatch(type, slotType, errorMessage);
      if (!valid)
        errorMessage.prepend(QObject::tr("Cannot insert %1 node in '%2' field of %3 node: ")
                               .arg(nodeName)
                               .arg(fieldName)
                               .arg(node->nodeModelName()));
      return valid;
    }

    const QString &parentModelName = node->nodeModelName();
    bool boundingObjectCase = (nodeUse & OmNode::BOUNDING_OBJECT_USE) || (fieldName == "boundingObject");
    if (automaticBoundingObjectCheck && (nodeUse == OmNode::UNKNOWN_USE)) {
      nodeUse = OmNodeUtilities::checkNodeUse(node);
      boundingObjectCase = boundingObjectCase || (nodeUse & OmNode::BOUNDING_OBJECT_USE);
    }

    const bool childrenField = fieldName == "children";
    const bool isTransformOrTransformDescendant = node->modelName() == "Transform" || OmNodeUtilities::findUpperTransform(node);

    if (childrenField) {
      const bool isInsertingTopLevel = node->isWorldRoot();

      // A robot cannot be a bounding object
      if (!boundingObjectCase && !isTransformOrTransformDescendant && OmNodeUtilities::isRobotTypeName(nodeName) &&
          !OmNodeUtilities::isDescendantOfBillboard(node))
        return true;

      // top level nodes
      bool invalidUseOfTopLevelNode = false;
      if (nodeName == "Fog" || nodeName == "Background") {
        if ((nodeName == "Fog" && OmFog::numberOfFogInstances() != 0) ||
            (nodeName == "Background" && OmBackground::numberOfBackgroundInstances() != 0)) {
          errorMessage = QObject::tr("Cannot insert duplicated %1 node: only one instance is allowed.").arg(nodeName);
          return false;
        } else if (isInsertingTopLevel)
          return true;
        invalidUseOfTopLevelNode = true;

      } else if (nodeName == "WorldInfo" || nodeName == "Viewpoint") {
        if (isInsertingTopLevel) {
          if (OmNodeReader::current())
            return true;

          errorMessage = defaultErrorMessage;
          return false;
        } else
          invalidUseOfTopLevelNode = true;
      }

      if (invalidUseOfTopLevelNode) {
        errorMessage = QObject::tr("%1 node can only be inserted at the top level of the node hierarchy.").arg(nodeName);
        return false;
      }

      if (isInsertingTopLevel) {
        // other nodes that can be inserted at top level
        if (nodeName == "Charger")
          return true;
        if (nodeName == "DirectionalLight")
          return true;
        if (nodeName == "Solid")
          return true;
        if (nodeName == "Group")
          return true;
        if (nodeName == "Pose")
          return true;
        if (nodeName == "Transform")
          return true;
        if (nodeName == "Billboard")
          return true;
        if (nodeName == "Shape")
          return true;
        if (nodeName == "CadShape")
          return true;
        if (nodeName == "PointLight")
          return true;
        if (nodeName == "SpotLight")
          return true;
        if (nodeName == "GranularGroup")
          return true;
        // Cloth is top-level ONLY, and not merely by convention: its particle
        // positions come back from Newton in world space and it renders under
        // the scene root, so an ancestor Pose/Group transform would be ignored
        // rather than applied. Its own translation/rotation fields are the
        // whole placement story (OmCloth.hpp).
        if (nodeName == "Cloth")
          return true;
        // SoftBody is the same node shape as Cloth -- particles, world-space
        // readback, scene-root rendering -- so it is top-level ONLY for exactly
        // the same reason (OmSoftBody.hpp).
        if (nodeName == "SoftBody")
          return true;

        errorMessage = QObject::tr("%1 node cannot be inserted at the top level of the node hierarchy.").arg(nodeName);
        return false;
      }
      if (nodeName == "Slot") {
        if (OmNodeUtilities::isDescendantOfBillboard(node))
          return false;
        return !isTransformOrTransformDescendant && !boundingObjectCase;
      }
    }

    static QStringList *fields = NULL;
    if (!fields) {
      fields = new QStringList();
      *fields << "physics"
              << "Physics"
              << "color"
              << "Color"
              << "contactProperties"
              << "ContactProperties"
              << "coord"
              << "Coordinate"
              << "damping"
              << "Damping"
              << "defaultDamping"
              << "Damping"
              << "focus"
              << "Focus"
              << "jointParameters3"
              << "JointParameters"
              << "jointParameters2"
              << "JointParameters"
              << "lens"
              << "Lens"
              << "lensFlare"
              << "LensFlare"
              << "material"
              << "Material"
              << "normal"
              << "Normal"
              << "recognition"
              << "Recognition"
              << "textureTransform"
              << "TextureTransform"
              << "texCoord"
              << "TextureCoordinate"
              << "zoom"
              << "Zoom";
    }

    for (int i = 0, size = fields->size(); i < size; i += 2) {
      if (fieldName == fields->at(i)) {
        if (nodeName == fields->at(i + 1))
          return true;
        else {
          errorMessage = defaultErrorMessage;
          return false;
        }
      }
    }

    if (fieldName == "appearance") {
      if (nodeName == "Appearance")
        return true;
      else if (nodeName == "PBRAppearance") {
        const OmShape *const shape = dynamic_cast<const OmShape *const>(node);
        if (!shape)
          return true;
        const OmGeometry *const geometry = shape->geometry();
        if (!geometry)
          return true;
        if (geometry->nodeType() == WB_NODE_INDEXED_LINE_SET || geometry->nodeType() == WB_NODE_POINT_SET) {
          errorMessage = QObject::tr("The '%1' node doesn't support 'PBRAppearance' in the 'appearance' field of its parent "
                                     "node, please use 'Appearance' instead.")
                           .arg(geometry->nodeModelName());
          return false;
        }
        return true;
      } else {
        errorMessage = defaultErrorMessage;
        return false;
      }
    }

    if (fieldName == "endPoint") {
      if (OmNodeUtilities::isSolidTypeName(nodeName) || nodeName == "SolidReference")
        return true;
      else if (nodeName == "Slot")
        return true;
    } else if (fieldName == "rotatingHead") {
      if (OmNodeUtilities::isSolidTypeName(nodeName))
        return true;
    } else if (fieldName.endsWith("Helix")) {
      if (OmNodeUtilities::isSolidTypeName(nodeName))
        return true;

    } else if (fieldName == "device") {
      const OmJoint *joint = dynamic_cast<const OmJoint *>(node);
      if (parentModelName.startsWith("Hinge") || parentModelName == "Propeller" || parentModelName == "BallJoint") {
        if ((nodeName == "RotationalMotor" &&
             (OmNodeReader::current() || (joint && joint->motor() == NULL) || parentModelName == "Propeller")) ||
            (nodeName == "PositionSensor" && (OmNodeReader::current() || (joint && joint->positionSensor() == NULL))) ||
            (nodeName == "Brake" && (OmNodeReader::current() || (joint && joint->brake() == NULL))))
          return OmNodeUtilities::hasARobotAncestor(node);

      } else if (parentModelName == "SliderJoint" || parentModelName == "Track") {
        const OmTrack *track = dynamic_cast<const OmTrack *>(node);
        if ((nodeName == "LinearMotor" &&
             (OmNodeReader::current() || (joint && joint->motor() == NULL) || (track && track->motor() == NULL))) ||
            (nodeName == "PositionSensor" && (OmNodeReader::current() || (joint && joint->positionSensor() == NULL) ||
                                              (track && track->positionSensor() == NULL))) ||
            (nodeName == "Brake" &&
             (OmNodeReader::current() || (joint && joint->brake() == NULL) || (track && track->brake() == NULL))))
          return OmNodeUtilities::hasARobotAncestor(node);
      }

    } else if (fieldName == "device2") {
      const OmHinge2Joint *joint = dynamic_cast<const OmHinge2Joint *>(node);
      if ((parentModelName == "Hinge2Joint" || parentModelName == "BallJoint") &&
          ((nodeName == "RotationalMotor" && (OmNodeReader::current() || (joint && joint->motor2() == NULL))) ||
           (nodeName == "PositionSensor" && (OmNodeReader::current() || (joint && joint->positionSensor2() == NULL))) ||
           (nodeName == "Brake" && (OmNodeReader::current() || (joint && joint->brake2() == NULL)))))
        return OmNodeUtilities::hasARobotAncestor(node);

    } else if (fieldName == "device3") {
      const OmBallJoint *joint = dynamic_cast<const OmBallJoint *>(node);
      if (parentModelName == "BallJoint" &&
          ((nodeName == "RotationalMotor" && (OmNodeReader::current() || (joint && joint->motor3() == NULL))) ||
           (nodeName == "PositionSensor" && (OmNodeReader::current() || (joint && joint->positionSensor3() == NULL))) ||
           (nodeName == "Brake" && (OmNodeReader::current() || (joint && joint->brake3() == NULL)))))
        return OmNodeUtilities::hasARobotAncestor(node);

    } else if (fieldName == "jointParameters") {
      if (parentModelName == "HingeJoint") {
        if (nodeName == "HingeJointParameters")
          return true;
      } else if (parentModelName == "SliderJoint") {
        if (nodeName == "JointParameters")
          return true;
      } else if (parentModelName == "Hinge2Joint") {
        if (nodeName == "HingeJointParameters")
          return true;
        if ((nodeName == "Hinge2JointParameters") && OmNodeReader::current())
          return true;  // // DEPRECATED, only for backward compatibility
      } else if (parentModelName == "BallJoint") {
        if (nodeName == "BallJointParameters")
          return true;
      }

    } else if (fieldName == "texture" && parentModelName == "Appearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "baseColorMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "roughnessMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "metalnessMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "normalMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "occlusionMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "emissiveColorMap" && parentModelName == "PBRAppearance") {
      return nodeName == "ImageTexture";

    } else if (fieldName == "cubemap" && parentModelName == "Background") {
      return nodeName == "Cubemap";

    } else if (fieldName == "motor" && parentModelName == "Track") {
      return nodeName == "LinearMotor";

    } else if (fieldName == "muscles" && (parentModelName == "LinearMotor" || parentModelName == "RotationalMotor")) {
      QString invalidParentNode;
      if (OmNodeUtilities::findUpperNodeByType(node, WB_NODE_TRACK, 1))
        invalidParentNode = "Track";

      if (!invalidParentNode.isEmpty()) {
        errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node:: "
                                   "%4 node doesn't support Muscle functionality.")
                         .arg(nodeName)
                         .arg(fieldName)
                         .arg(parentModelName)
                         .arg(invalidParentNode);
        return false;
      }
      return nodeName == "Muscle";

    } else if (fieldName == "animatedGeometry" && parentModelName == "Track") {
      return nodeName == "CadShape" || nodeName == "Shape" || nodeName == "Transform" || nodeName == "Pose" ||
             nodeName == "Group" || nodeName == "Slot";

    } else if (!boundingObjectCase) {
      if (fieldName == "children") {
        if (nodeName == "Group")
          return true;
        if (nodeName == "Pose")
          return true;
        if (nodeName == "Transform")
          return true;
        if (nodeName == "Shape")
          return true;
        if (nodeName == "CadShape")
          return true;

        if (OmNodeUtilities::isDescendantOfBillboard(node))
          // only Group, Pose, Transform, Shape and CadShape allowed
          return false;

        // if the node itself is a Transform or it has a Transform ancestor exists, prohibit the insertion of Solids,
        // Robots, Devices, Propellers, Lights, and Joints
        if (isTransformOrTransformDescendant) {
          if (nodeName == "PointLight" || nodeName == "SpotLight" || nodeName == "DirectionalLight" ||
              OmNodeUtilities::isSolidTypeName(nodeName) || nodeName == "Propeller" || nodeName.endsWith("Joint"))
            return false;
        }

        if (nodeName == "Solid")
          return true;

        if (OmVrmlNodeUtilities::isFieldDescendant(node, "animatedGeometry"))
          // only Group, Pose, Transform, Shape, CadShape and Slot allowed
          return false;

        if ((parentModelName == "TrackWheel") || OmNodeUtilities::findUpperNodeByType(node, WB_NODE_TRACK_WHEEL))
          // only Group, Pose, Transform, Shape, CadShape and Slot allowed
          return false;

        if (nodeName == "PointLight")
          return true;
        if (nodeName == "SpotLight")
          return true;
        if (nodeName == "Propeller")
          return true;
        if (nodeName == "Charger")
          return true;
        if (nodeName == "TrackWheel")
          return parentModelName == "Track";

        if (nodeName == "Connector" || nodeName.endsWith("Joint") || nodeName == "VacuumGripper") {
          if (OmNodeUtilities::isSolidTypeName(parentModelName) || OmNodeUtilities::findUpperSolid(node) != NULL)
            return true;

          errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node that doesn't have a Solid ancestor.")
                           .arg(nodeName)
                           .arg(fieldName)
                           .arg(parentModelName);
        }

        if (OmNodeUtilities::isSolidDeviceTypeName(nodeName)) {
          if (OmNodeUtilities::hasARobotAncestor(node))
            return true;

          errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node that doesn't have a Robot ancestor.")
                           .arg(nodeName)
                           .arg(fieldName)
                           .arg(parentModelName);
        }

      } else if (fieldName == "geometry") {
        if (nodeName == "IndexedLineSet" || nodeName == "PointSet") {
          const OmShape *const shape = dynamic_cast<const OmShape *const>(node);
          if (shape && shape->pbrAppearance()) {
            errorMessage =
              QObject::tr("Can't insert a '%1' node in the 'geometry' field of 'Shape' node if the 'appearance' field "
                          "contains a 'PBRAppearance' node, please use an 'Appearance' node instead.")
                .arg(nodeName);
            return false;
          }
        }
        if (OmNodeUtilities::isGeometryTypeName(nodeName))
          return true;
      }

    } else {  // boundingObject use
      if (fieldName == "boundingObject") {
        if (nodeName == "Shape")
          return true;
        if (nodeName == "Group")
          return true;
        if (nodeName == "Pose")
          return true;
        if (OmNodeUtilities::isCollisionDetectedGeometryTypeName(nodeName))
          return true;

      } else if (childrenField) {
        if (nodeName == "Shape")
          return true;
        if ((nodeName == "Pose") && (parentModelName != "Pose"))
          return true;
        // if the node is also used outside a boundingObject geometries cannot be inserted directly in the children field
        if (!(nodeUse & OmNode::STRUCTURE_USE) && OmNodeUtilities::isCollisionDetectedGeometryTypeName(nodeName))
          return true;

      } else if (fieldName == "geometry") {
        if (OmNodeUtilities::isCollisionDetectedGeometryTypeName(nodeName))
          return true;
      }

      if (OmNodeUtilities::isGeometryTypeName(nodeName)) {
        errorMessage = QObject::tr("%1 geometry node cannot be used in bounding object.").arg(nodeName);
      } else {
        errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node in bounding object.")
                         .arg(nodeName)
                         .arg(fieldName)
                         .arg(parentModelName);
      }

      return false;
    }

    if (errorMessage.isEmpty())
      errorMessage = defaultErrorMessage;

    return false;
  }

  bool isSolidNode(OmBaseNode *node) {
    return dynamic_cast<OmSolid *>(node);
  }

  bool doesFieldRestrictionAcceptNode(const OmField *const field, const QString &nodeModelName, const OmNodeModel *nodeModel,
                                      const QStringList &protoParentList) {
    assert(field->hasRestrictedValues());
    foreach (const OmFieldValueRestriction restriction, field->acceptedValues())
      if (restriction.isNodeAccepted(nodeModelName, nodeModel, protoParentList))
        return true;
    return false;
  }
};  // namespace

OmNode *OmNodeUtilities::findUpperNodeByType(const OmNode *node, int nodeType, int searchDegrees) {
  if (node == NULL)
    return NULL;

  int count = searchDegrees > 0 ? searchDegrees : -1;
  OmBaseNode *n = dynamic_cast<OmBaseNode *>(node->parentNode());
  while (n && count != 0) {
    if (n->nodeType() == nodeType)
      return n;
    n = dynamic_cast<OmBaseNode *>(n->parentNode());
    count--;
  }
  return NULL;
}

bool OmNodeUtilities::hasDescendantNodesOfType(const OmNode *node, const QList<int> &nodeTypes) {
  QList<OmNode *> subNodes = node->subNodes(true);
  if (subNodes.isEmpty())
    return false;
  foreach (const OmNode *n, subNodes) {
    if (nodeTypes.contains(dynamic_cast<const OmBaseNode *>(n)->nodeType()))
      return true;
  }
  return false;
}

OmMatter *OmNodeUtilities::findUpperMatter(const OmNode *node) {
  if (node == NULL)
    return NULL;

  OmNode *n = node->parentNode();

  while (n) {
    OmMatter *const matter = dynamic_cast<OmMatter *>(n);
    if (matter)
      return matter;
    else
      n = n->parentNode();
  }
  return NULL;
}

OmSolid *OmNodeUtilities::findUpperSolid(const OmNode *node) {
  if (node == NULL)
    return NULL;
  OmMatter *upperMatter = findUpperMatter(node);
  // in the case of slot we want to return the parent node of the first slot
  const OmSlot *slot = dynamic_cast<OmSlot *>(upperMatter);
  while (slot) {
    upperMatter = findUpperMatter(upperMatter);
    slot = dynamic_cast<OmSlot *>(upperMatter);
  }
  return dynamic_cast<OmSolid *>(upperMatter);
}

OmPose *OmNodeUtilities::findUppermostPose(const OmNode *node) {
  const OmNode *n = node;
  OmPose *uppermostPose = NULL;
  while (n) {
    const OmPose *pose = dynamic_cast<const OmPose *>(n);
    if (pose)
      uppermostPose = const_cast<OmPose *>(pose);
    n = n->parentNode();
  };
  return uppermostPose;
}

OmSolid *OmNodeUtilities::findUppermostSolid(const OmNode *node) {
  const OmNode *n = node;
  OmSolid *uppermostSolid = NULL;
  while (n) {
    const OmSolid *solid = dynamic_cast<const OmSolid *>(n);
    if (solid)
      uppermostSolid = const_cast<OmSolid *>(solid);
    n = n->parentNode();
  };
  return uppermostSolid;
}

OmMatter *OmNodeUtilities::findUppermostMatter(OmNode *node) {
  const OmNode *n = node;
  OmMatter *uppermostMatter = NULL;
  while (n) {
    const OmMatter *matter = dynamic_cast<const OmMatter *>(n);
    if (matter)
      uppermostMatter = const_cast<OmMatter *>(matter);
    n = n->parentNode();
  };
  return uppermostMatter;
}

OmSolid *OmNodeUtilities::findTopSolid(const OmNode *node) {
  if (node == NULL)
    return NULL;

  const OmNode *n = node;
  const OmNode *parent = n->parentNode();
  OmSolid *topSolid = NULL;
  while (parent) {
    OmSolid *currentSolid = dynamic_cast<OmSolid *>(const_cast<OmNode *>(n));
    if (currentSolid)
      topSolid = currentSolid;

    n = parent;
    parent = n->parentNode();
  }
  return topSolid;
}

OmTransform *OmNodeUtilities::findUpperTransform(const OmNode *node) {
  if (node == NULL)
    return NULL;

  OmNode *n = node->parentNode();
  while (n) {
    OmTransform *const transform = dynamic_cast<OmTransform *>(n);
    if (transform)
      return transform;
    else
      n = n->parentNode();
  }
  return NULL;
}

OmPose *OmNodeUtilities::findUpperPose(const OmNode *node) {
  if (node == NULL)
    return NULL;

  OmNode *n = node->parentNode();
  while (n) {
    OmPose *const pose = dynamic_cast<OmPose *>(n);
    if (pose)
      return pose;
    else
      n = n->parentNode();
  }
  return NULL;
}

bool OmNodeUtilities::hasARobotDescendant(const OmNode *node) {
  const QList<OmNode *> &subNodes = node->subNodes(true);

  foreach (OmNode *const descendantNode, subNodes) {
    if (dynamic_cast<OmRobot *>(descendantNode))
      return true;
  }

  return false;
}

bool OmNodeUtilities::hasADeviceDescendant(const OmNode *node, bool ignoreConnector) {
  const OmGroup *group = dynamic_cast<const OmGroup *>(node);
  if (!group)
    return false;

  const QList<OmNode *> &subNodes = node->subNodes(true);

  foreach (OmNode *const descendantNode, subNodes) {
    if (dynamic_cast<OmDevice *>(descendantNode) && (!ignoreConnector || !dynamic_cast<OmConnector *>(descendantNode)))
      return true;
  }

  return false;
}

bool OmNodeUtilities::hasARobotAncestor(const OmNode *node) {
  const OmRobot *robot = findRobotAncestor(node);

  return robot != NULL;
}

OmRobot *OmNodeUtilities::findRobotAncestor(const OmNode *node) {
  if (!node)
    return NULL;

  while (node) {
    if (isRobotTypeName(node->nodeModelName())) {
      const OmRobot *robot = reinterpret_cast<const OmRobot *>(node);
      return const_cast<OmRobot *>(robot);
    }

    node = node->parentNode();
  }
  return NULL;
}

bool OmNodeUtilities::isDescendantOfBillboard(const OmNode *node) {
  if (node == NULL)
    return false;

  OmNode *n = const_cast<OmNode *>(node);
  while (n && !n->isWorldRoot()) {
    const OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(n);

    if (!baseNode)
      return false;

    if (baseNode->nodeType() == WB_NODE_BILLBOARD)
      return true;

    n = n->parentNode();
  }

  return false;
}

bool OmNodeUtilities::isDescendantOfPropeller(const OmNode *node) {
  if (node == NULL)
    return false;

  OmNode *n = const_cast<OmNode *>(node);
  while (n && !n->isWorldRoot()) {
    const OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(n);

    if (!baseNode)
      return false;

    if (baseNode->nodeType() == WB_NODE_PROPELLER)
      return true;

    n = n->parentNode();
  }

  return false;
}

OmNode::NodeUse OmNodeUtilities::checkNodeUse(const OmNode *n) {
  OmNode::NodeUse nodeUse = OmNode::UNKNOWN_USE;
  if (n->isDefNode()) {
    // check if at least one of the USE node is in bounding object
    foreach (const OmNode *useNode, n->useNodes()) {
      nodeUse = static_cast<OmNode::NodeUse>(nodeUse | checkNodeUse(useNode));
      if (nodeUse == OmNode::BOTH_USE)
        return nodeUse;
    }
  }

  if (n->isProtoParameterNode()) {
    QVector<OmNode *> instances = n->protoParameterNodeInstances();
    // check if at least one of the instances is in bounding object
    foreach (const OmNode *instance, instances) {
      nodeUse = static_cast<OmNode::NodeUse>(nodeUse | checkNodeUse(instance));
      if (nodeUse == OmNode::BOTH_USE)
        return nodeUse;
    }
    return nodeUse;
  }

  const OmNode *const p = n->parentNode();
  if (p) {
    const OmMatter *const m = dynamic_cast<const OmMatter *>(p);
    // cppcheck-suppress knownConditionTrueFalse
    if (m)
      return static_cast<OmNode::NodeUse>(nodeUse |
                                          (m->boundingObject() == n ? OmNode::BOUNDING_OBJECT_USE : OmNode::STRUCTURE_USE));

    return static_cast<OmNode::NodeUse>(nodeUse | checkNodeUse(p));
  }

  return static_cast<OmNode::NodeUse>(nodeUse | OmNode::STRUCTURE_USE);
}

bool OmNodeUtilities::isInBoundingObject(const OmNode *node) {
  const OmNode *const p = node->parentNode();
  if (p) {
    const OmMatter *const m = dynamic_cast<const OmMatter *>(p);
    // cppcheck-suppress knownConditionTrueFalse
    if (m) {
      const OmNode *boundingObject = m->boundingObject();

      while (boundingObject) {
        if (boundingObject == node)
          return true;
        boundingObject = boundingObject->protoParameterNode();
      }
    }

    return isInBoundingObject(p);
  }

  return false;
}

OmMatter *OmNodeUtilities::findBoundingObjectAncestor(const OmBaseNode *node) {
  if (!node || !node->isInBoundingObject())
    return NULL;

  OmNode *ancestor = node->parentNode();
  while (ancestor && !ancestor->isWorldRoot()) {
    OmMatter *solidAncestor = dynamic_cast<OmMatter *>(ancestor);
    if (solidAncestor)
      return solidAncestor;
    ancestor = ancestor->parentNode();
  }

  return NULL;
}

bool OmNodeUtilities::isSelected(const OmNode *node) {
  if (!node)
    return false;

  const OmSolid *const selectedSolid = OmSelection::instance()->selectedSolid();
  if (!selectedSolid)
    return false;

  const OmSolid *const upperSolid = findUpperSolid(node);
  const OmSolid *const topSolid = findTopSolid(upperSolid);
  if (upperSolid == selectedSolid || topSolid == selectedSolid)
    return true;

  return false;
}

void OmNodeUtilities::fixBackwardCompatibility(OmNode *node) {
  // We don't want to apply the fix if the node is already >R2021b
  if (!node)
    return;
  if (node->proto() && node->proto()->fileVersion() > OmVersion(2021, 1, 1))
    return;
  const OmNode *const protoAncestor = OmVrmlNodeUtilities::findRootProtoNode(node);
  if (!node->proto() && protoAncestor && protoAncestor->proto()->fileVersion() > OmVersion(2021, 1, 1))
    return;
  if (node->isWorldRoot() && OmTokenizer::worldFileVersion() > OmVersion(2021, 1, 1))
    return;

  static const QString message(
    QObject::tr("Trying to resolve the backwards compability by adjusting the rotation (strategy %1)."));

  // We want to find nodes until PROTOs.
  QList<OmNode *> candidates;
  QQueue<OmNode *> queue;
  QList<OmNode *> subProtos;
  queue.enqueue(node);
  candidates.append(node);
  while (!queue.isEmpty()) {
    const OmNode *const n = queue.dequeue();
    for (OmNode *child : n->subNodes(false, true, true)) {
      if (!child->proto()) {
        queue.append(child);
        if (!candidates.contains(child))
          candidates.append(child);
      } else
        subProtos.append(child);
    }
  }
  sortNodeListForBackwardCompatibility(candidates);

  // Apply rotations to the candidates.
  for (OmNode *candidate : candidates) {
    // This condition is added to handle dangling pointers.
    // TODO: It is very slow though, we may need to improve it.
    if (!node->subNodes(true, true, true).contains(candidate) && candidate != node)
      continue;
    if (dynamic_cast<OmCamera *>(candidate) || dynamic_cast<OmLidar *>(candidate) || dynamic_cast<OmRadar *>(candidate) ||
        dynamic_cast<OmPen *>(candidate) || dynamic_cast<OmEmitter *>(candidate) || dynamic_cast<OmReceiver *>(candidate) ||
        dynamic_cast<OmConnector *>(candidate) || dynamic_cast<OmTouchSensor *>(candidate) ||
        dynamic_cast<OmViewpoint *>(candidate) || dynamic_cast<OmTrack *>(candidate)) {
      // Rotate devices.
      OmMatrix3 rotationFix(-M_PI_2, 0, M_PI_2);
      if (dynamic_cast<OmPen *>(candidate) || dynamic_cast<OmTrack *>(candidate))
        rotationFix = OmMatrix3(-M_PI_2, 0, 0);
      if (dynamic_cast<OmEmitter *>(candidate) || dynamic_cast<OmReceiver *>(candidate) ||
          dynamic_cast<OmConnector *>(candidate) || dynamic_cast<OmTouchSensor *>(candidate))
        rotationFix = OmMatrix3(-M_PI_2, 0, -M_PI_2);

      // Rotate the viewpoint (exception).
      if (dynamic_cast<OmViewpoint *>(candidate)) {
        candidate->info(message.arg("A1"));
        OmViewpoint *const viewpoint = static_cast<OmViewpoint *>(candidate);
        viewpoint->orientation()->setValue(OmRotation(viewpoint->orientation()->value().toMatrix3() * rotationFix));
        viewpoint->save("__init__");
        continue;
      }
      candidate->info(message.arg("A2"));

      // Rotate the device.
      if (candidate != node) {
        OmPose *const device = static_cast<OmPose *>(candidate);
        device->setRotation(OmRotation(device->rotation().toMatrix3() * rotationFix));
        device->save("__init__");
      }

      QList<OmNode *> children = getNodeChildrenAndBoundingForBackwardCompatibility(candidate);
      for (OmNode *child : children) {
        if (!getNodeChildrenAndBoundingForBackwardCompatibility(candidate).contains(child))
          continue;

        OmPose *childPose = dynamic_cast<OmPose *>(child);
        if (childPose) {
          // Squash poses if possible.
          childPose->setRotation(OmRotation(rotationFix.transposed() * childPose->rotation().toMatrix3()));
          childPose->setTranslation(rotationFix.transposed() * childPose->translation());
          childPose->save("__init__");
        } else {
          if (!getNodeChildrenForBackwardCompatibility(candidate).contains(child)) {
            // Child is a bounding object.
            child->info(message.arg("A2_1"));
            OmField *boundingObjectField = candidate->findField("boundingObject");
            if (!boundingObjectField) {
              // field not found if parent is a PROTO and the field is not exposed
              candidate->warn("Conversion to the new OmniSim world format was unsuccessful, please resolve it manually.");
              continue;
            }
            OmPose *const pose = new OmPose();
            pose->setRotation(OmRotation(rotationFix.transposed()));
            pose->save("__init__");
            OmNode *newNode = child->cloneAndReferenceProtoInstance();
            OmNodeOperations::instance()->initNewNode(pose, candidate, boundingObjectField, -1, false, false);
            OmNodeOperations::instance()->initNewNode(newNode, pose, pose->findField("children"), 0, false, false);
          } else {
            // Child is under the `children` field.
            child->info(message.arg("A2_2"));
            OmField *childrenField = candidate->findField("children");
            if (!childrenField) {
              // field not found if parent is a PROTO and the field is not exposed
              candidate->warn("Conversion to the new OmniSim world format was unsuccessful, please resolve it manually.");
              continue;
            }
            OmPose *const pose = new OmPose();
            pose->setRotation(OmRotation(rotationFix.transposed()));
            pose->save("__init__");
            OmNode *newNode = child->cloneAndReferenceProtoInstance();
            OmNodeOperations::instance()->initNewNode(pose, candidate, childrenField, 0, false, false);
            OmNodeOperations::instance()->deleteNode(child);
            OmNodeOperations::instance()->initNewNode(newNode, pose, pose->findField("children"), 0, false, false);
          }
        }
      }
    } else if (dynamic_cast<OmCylinder *>(candidate) || dynamic_cast<OmCapsule *>(candidate) ||
               dynamic_cast<OmCone *>(candidate) || dynamic_cast<OmPlane *>(candidate) ||
               dynamic_cast<OmElevationGrid *>(candidate)) {
      // Rotate geometries.
      const OmMatrix3 rotationFix(-M_PI_2, 0, 0);
      OmNode *const nodeToRotate = dynamic_cast<OmShape *>(candidate->parentNode()) ? candidate->parentNode() : candidate;
      OmNode *const parent = nodeToRotate->parentNode();
      assert(dynamic_cast<OmGroup *>(parent));

      OmPose *const parentPose = dynamic_cast<OmPose *>(parent);
      if (parentPose && parentPose->subNodes(false, false).size() == 1) {
        // Squash poses if possible.
        candidate->info(message.arg("B1"));
        if (dynamic_cast<OmTrackWheel *>(parentPose->parentNode()))
          continue;
        parentPose->setRotation(OmRotation(parentPose->rotation().toMatrix3() * rotationFix));
        parentPose->save("__init__");
      } else
        candidate->warn("Conversion to the new OmniSim world format was unsuccessful, please resolve it manually.");
    }
  }

  // Convert sub-protos.
  for (OmNode *subProto : subProtos) {
    if (subProto->proto() &&
        (subProto->proto()->path().contains(OmStandardPaths::omniSimHomePath()) ||
         subProto->proto()->name() == "Bc21bCameraProto") &&
        dynamic_cast<OmPose *>(subProto)) {
      // Since we rotated almost all Webots PROTOs we need to rotate them back.
      // The `Bc21bCameraProto.proto` is added for CI tests (the CI tests are not in the same directory as Webots).

      subProto->info(message.arg("C"));
      const OmMatrix3 rotationFix(-M_PI_2, 0, M_PI_2);
      OmPose *const subProtoPose = static_cast<OmPose *>(subProto);
      subProtoPose->setRotation(OmRotation(subProtoPose->rotation().toMatrix3() * rotationFix));
      subProtoPose->save("__init__");
    } else if (!node->isWorldRoot())
      fixBackwardCompatibility(subProto);
  }
}

OmMatter *OmNodeUtilities::findUpperVisibleMatter(OmNode *node) {
  if (!node)
    return NULL;

  QStack<OmNode *> nodeStack;
  OmNode *parent = node;
  // get node sequence from 'node' to the world root
  while (parent && !parent->isWorldRoot()) {
    while (parent->protoParameterNode())
      parent = parent->protoParameterNode();
    nodeStack.push(parent);
    parent = parent->parentNode();
  }

  if (nodeStack.isEmpty())
    return NULL;

  // iterate back through the stack to check the node visibility
  parent = nodeStack.pop();
  OmMatter *visibleMatter = dynamic_cast<OmMatter *>(parent);
  OmNode *n = NULL;
  while (!nodeStack.isEmpty()) {
    n = nodeStack.pop();
    if (parent->isProtoInstance()) {
      if (!parent->isProtoParameterChild(n))
        return visibleMatter;
    }

    OmMatter *matter = dynamic_cast<OmMatter *>(n);
    if (matter) {
      if (matter->isProtoParameterNode()) {
        OmBaseNode *finalizedInstance = matter->getFirstFinalizedProtoInstance();
        if (finalizedInstance)
          visibleMatter = dynamic_cast<OmMatter *>(finalizedInstance);
      } else
        visibleMatter = matter;
    }
    parent = n;
  }

  return visibleMatter;
}

QList<OmSolid *> OmNodeUtilities::findSolidDescendants(OmNode *node) {
  QList<OmSolid *> solidsList;
  QList<OmNode *> list = findDescendantNodesOfType(node, isSolidNode, false);
  for (int i = 0; i < list.size(); ++i)
    solidsList << dynamic_cast<OmSolid *>(list[i]);
  return solidsList;
}

// cppcheck-suppress constParameterPointer
// cppcheck-suppress constParameterReference
QList<OmNode *> OmNodeUtilities::findDescendantNodesOfType(OmNode *node, bool (&typeCondition)(OmBaseNode *), bool recursive) {
  QList<OmNode *> result;
  QQueue<OmNode *> queue;
  QSet<OmNode *> visited;  // avoid repeated linear scans for SolidReference cycles
  queue.enqueue(node);
  OmNode *n = NULL;
  while (!queue.isEmpty()) {
    n = queue.dequeue();
    visited.insert(n);
    if (typeCondition(dynamic_cast<OmBaseNode *>(n))) {
      result.append(n);
      if (!recursive)
        continue;
    }

    const OmGroup *const group = dynamic_cast<OmGroup *>(n);
    if (group) {
      int childCount = group->childCount();
      for (int i = 0; i < childCount; ++i)
        queue.enqueue(group->child(i));
      continue;
    }

    const OmSlot *const slot = dynamic_cast<OmSlot *>(n);
    if (slot) {
      // cppcheck-suppress constVariablePointer
      OmNode *baseEndPoint = slot->endPoint();
      if (baseEndPoint && (!slot->solidReferenceEndPoint() || !visited.contains(baseEndPoint)))
        queue.enqueue(baseEndPoint);
      continue;
    }

    const OmBasicJoint *const joint = dynamic_cast<OmBasicJoint *>(n);
    if (joint) {
      // cppcheck-suppress constVariablePointer
      OmSolid *endPoint = joint->solidEndPoint();
      if (endPoint && (!joint->solidReference() || !visited.contains(endPoint)))
        queue.enqueue(endPoint);
    }
  }
  if (!result.isEmpty() && result.first() == node)
    result.removeFirst();
  return result;
}

bool OmNodeUtilities::isTemplateRegeneratorField(const OmField *field) {
  const OmField *f = field;
  while (f != NULL) {
    if (f->isTemplateRegenerator() ||
        (f->parentNode() && OmTemplateManager::isNodeChangeTriggeringRegeneration(f->parentNode())))
      return true;
    f = f->parameter();
  }
  return false;
}

bool OmNodeUtilities::isNodeOrAncestorLocked(const OmNode *node) {
  const OmNode *n = node;
  while (n && !n->isWorldRoot()) {
    const OmBaseNode *baseNode = dynamic_cast<const OmBaseNode *>(n);
    if (baseNode && baseNode->nodeType() == WB_NODE_BILLBOARD)
      return true;

    const OmMatter *matter = dynamic_cast<const OmMatter *>(n);
    if (matter && matter->isLocked())
      return true;

    n = n->parentNode();
  }

  return false;
}

const OmShape *OmNodeUtilities::findIntersectingShape(const OmRay &ray, double maxDistance, double &distance,
                                                      double minDistance) {
  double timeStep = OmSimulationState::instance()->time();
  const OmGroup *root = OmWorld::instance()->root();
  const int childCount = root->childCount();
  distance = maxDistance;
  const OmShape *shape = NULL;
  OmBoundingSphere *bs;
  for (int i = 0; i < childCount; ++i) {
    bs = root->child(i)->boundingSphere();
    if (bs == NULL)
      continue;
    OmBoundingSphere::IntersectingShape res = bs->computeIntersection(ray, timeStep);
    if (res.shape != NULL && res.distance < distance && res.distance > minDistance) {
      distance = res.distance;
      shape = res.shape;
    }
  }
  return shape;
}

dBodyID OmNodeUtilities::findBodyMerger(const OmNode *node) {
  if (!node)
    return NULL;

  const OmNode *n = node;
  while (n) {
    const OmSolid *s = dynamic_cast<const OmSolid *>(n);
    if (s && s->bodyMerger())
      return s->bodyMerger();
    if (dynamic_cast<const OmBasicJoint *>(n))
      break;
    n = n->parentNode();
  }
  return NULL;
}

bool OmNodeUtilities::isTrackAnimatedGeometry(const OmNode *node) {
  if (node == NULL)
    return false;

  const OmNode *n = node;
  const OmNode *p = n->parentNode();
  while (p) {
    if (dynamic_cast<const OmTrack *>(p) != NULL)
      return (n->parentField() && n->parentField()->model()->name() == "animatedGeometry");
    n = p;
    p = p->parentNode();
  }

  return false;
}

bool OmNodeUtilities::isGeometryTypeName(const QString &modelName) {
  if (modelName == "Cone")
    return true;
  if (modelName == "IndexedLineSet")
    return true;
  if (modelName == "PointSet")
    return true;
  if (isCollisionDetectedGeometryTypeName(modelName))
    return true;
  return false;
}

bool OmNodeUtilities::isCollisionDetectedGeometryTypeName(const QString &modelName) {
  if (modelName == "Box")
    return true;
  if (modelName == "Capsule")
    return true;
  if (modelName == "Cylinder")
    return true;
  if (modelName == "ElevationGrid")
    return true;
  if (modelName == "IndexedFaceSet")
    return true;
  if (modelName == "Mesh")
    return true;
  if (modelName == "Plane")
    return true;
  if (modelName == "Sphere")
    return true;
  return false;
}

bool OmNodeUtilities::isRobotTypeName(const QString &modelName) {
  return modelName == "Robot";
}

bool OmNodeUtilities::isDeviceTypeName(const QString &modelName) {
  if (isSolidDeviceTypeName(modelName))
    return true;
  QStringList deviceTypeName = (QStringList() << "Brake"
                                              << "LinearMotor"
                                              << "PositionSensor"
                                              << "RotationalMotor");
  return deviceTypeName.contains(modelName);
}

bool OmNodeUtilities::isSolidDeviceTypeName(const QString &modelName) {
  QStringList solidDeviceTypeName = (QStringList() << "Accelerometer"
                                                   << "Altimeter"
                                                   << "Camera"
                                                   << "Compass"
                                                   << "Connector"
                                                   << "Display"
                                                   << "DistanceSensor"
                                                   << "Emitter"
                                                   << "GPS"
                                                   << "Gyro"
                                                   << "InertialUnit"
                                                   << "LED"
                                                   << "Lidar"
                                                   << "LightSensor"
                                                   << "Pen"
                                                   << "Radar"
                                                   << "RangeFinder"
                                                   << "Receiver"
                                                   << "Speaker"
                                                   << "TouchSensor"
                                                   << "Track"
                                                   << "VacuumGripper");
  return solidDeviceTypeName.contains(modelName);
}

bool OmNodeUtilities::isSolidTypeName(const QString &modelName) {
  if (modelName == "Solid")
    return true;
  if (modelName == "Charger")
    return true;
  if (OmNodeUtilities::isSolidDeviceTypeName(modelName))
    return true;
  if (isRobotTypeName(modelName))
    return true;

  return false;
}

bool OmNodeUtilities::isMatterTypeName(const QString &modelName) {
  if (OmNodeUtilities::isSolidTypeName(modelName))
    return true;

  return false;
}

bool OmNodeUtilities::isSlotTypeMatch(const QString &firstType, const QString &secondType, QString &errorMessage) {
  if (firstType.isEmpty() || secondType.isEmpty()) {
    // empty type matches any type
    return true;
  } else if (firstType.endsWith('+') || firstType.endsWith('-')) {  // slots with gender
    if (firstType.left(firstType.size() - 1) == secondType.left(secondType.size() - 1)) {
      if (firstType == secondType) {  // the gender is the same => not compatible
        errorMessage = QObject::tr("the two Slot nodes have the same gender.");
        return false;
      } else  // the gender is different => ok
        return true;
    }
  } else if (firstType == secondType)
    return true;

  // type is not the same
  errorMessage = QObject::tr("types '%1' and '%2' are not matching.").arg(firstType).arg(secondType);
  return false;
}

// cppcheck-suppress constParameterPointer
bool OmNodeUtilities::validateInsertedNode(OmField *field, const OmNode *newNode, const OmNode *parentNode,
                                           bool isInBoundingObject) {
  if (newNode == NULL || field == NULL || parentNode == NULL)
    return true;

  // special case: validation of insertion of Slot node
  // normal validation could fail because the new node is not yet inserted in parent node
  const OmSlot *slot = dynamic_cast<const OmSlot *>(newNode);
  if (slot && slot->endPoint() != NULL) {
    const OmSlot *lowerSlot = slot->slotEndPoint();

    // skip couple of Slot nodes and
    // validate if the endPoint node can be inserted in the parent node
    QList<OmField *> fields = field->internalFields();
    if (fields.isEmpty())
      fields.append(field);
    foreach (OmField *internalField, fields) {
      if (internalField->isParameter())
        // recursive call: check only node field names and not parameter names
        validateInsertedNode(internalField, newNode, internalField->parentNode(), isInBoundingObject);
      else {
        OmNode *internalParentNode = internalField->parentNode();

        // check for single or trio of Slot nodes
        const OmSlot *parentSlot = dynamic_cast<const OmSlot *>(internalParentNode);
        QString errorMessage;
        if (parentSlot && lowerSlot)
          errorMessage = QObject::tr("Cannot insert %1 node in '%2' field of %3 node, because a trio of slot is not allowed.");
        else if (!parentSlot && !lowerSlot)
          errorMessage =
            QObject::tr("Cannot insert %1 node in '%2' field of %3 node: only a slot can be added in the parent slot.");

        if (!errorMessage.isEmpty()) {
          internalParentNode->parsingWarn(
            errorMessage.arg(newNode->modelName()).arg(field->name()).arg(parentNode->nodeModelName()));
          return false;
        }

        if (lowerSlot)
          lowerSlot->validate(internalParentNode, internalField, isInBoundingObject);
        else if (dynamic_cast<const OmSlot *>(internalParentNode)) {
          // upper slot
          const OmField *internalParentField = internalParentNode->parentField(true);
          internalParentNode = internalParentNode->parentNode();
          newNode->validate(internalParentNode, internalParentField, isInBoundingObject);
        } else  // invalid structure
          newNode->validate(internalParentNode, internalField, isInBoundingObject);
      }
      return true;
    }
  }

  newNode->validate(NULL, NULL, isInBoundingObject);
  return true;
}

bool OmNodeUtilities::validateExistingChildNode(const OmField *const field, const OmNode *childNode, const OmNode *node,
                                                bool isInBoundingObject, QString &errorMessage) {
  const QString &fieldName = field->name();
  const QString &parentModelName = node->nodeModelName();
  const QString &childModelName = childNode->nodeModelName();

  enum ValidationResultType { NONE = 0, DUPLICATED = 1, ROBOT_ANCESTOR = 2 };
  int result = NONE;
  if (fieldName == "device") {
    const OmJoint *joint = dynamic_cast<const OmJoint *>(node);
    if (parentModelName.startsWith("Hinge") || parentModelName == "BallJoint") {
      if (joint) {
        if (childModelName == "RotationalMotor")
          result = 1 + (static_cast<OmNode *>(joint->motor()) == childNode);
        else if (childModelName == "PositionSensor")
          result = 1 + (static_cast<OmNode *>(joint->positionSensor()) == childNode);
        else if (childModelName == "Brake")
          result = 1 + (static_cast<OmNode *>(joint->brake()) == childNode);
      }
    } else if (parentModelName == "SliderJoint" || parentModelName == "Track") {
      const OmTrack *track = dynamic_cast<const OmTrack *>(node);
      if (childModelName == "LinearMotor")
        result = 1 + (((joint && static_cast<OmNode *>(joint->motor()) == childNode) ||
                       (track && static_cast<OmNode *>(track->motor()) == childNode)));
      else if (childModelName == "PositionSensor")
        result = 1 + (((joint && static_cast<OmNode *>(joint->positionSensor()) == childNode) ||
                       (track && static_cast<OmNode *>(track->positionSensor()) == childNode)));
      else if (childModelName == "Brake")
        result = 1 + (((joint && static_cast<OmNode *>(joint->brake()) == childNode) ||
                       (track && static_cast<OmNode *>(track->brake()) == childNode)));
    } else if (parentModelName == "Propeller" && childModelName == "RotationalMotor")
      result = ROBOT_ANCESTOR;
  } else if (fieldName == "device2") {
    const OmHinge2Joint *joint = dynamic_cast<const OmHinge2Joint *>(node);
    if (joint) {
      if (childModelName == "RotationalMotor")
        result = 1 + (static_cast<OmNode *>(joint->motor2()) == childNode);
      else if (childModelName == "PositionSensor")
        result = 1 + (static_cast<OmNode *>(joint->positionSensor2()) == childNode);
      else if (childModelName == "Brake")
        result = 1 + (static_cast<OmNode *>(joint->brake2()) == childNode);
    }
  } else if (fieldName == "device3") {
    const OmBallJoint *joint = dynamic_cast<const OmBallJoint *>(node);
    if (joint) {
      if (childModelName == "RotationalMotor")
        result = 1 + (static_cast<OmNode *>(joint->motor3()) == childNode);
      else if (childModelName == "PositionSensor")
        result = 1 + (static_cast<OmNode *>(joint->positionSensor3()) == childNode);
      else if (childModelName == "Brake")
        result = 1 + (static_cast<OmNode *>(joint->brake3()) == childNode);
    }
  }
  if (result == ROBOT_ANCESTOR)  // valid if node has a robot ancestor
    return OmNodeUtilities::hasARobotAncestor(node);
  else if (result == DUPLICATED) {  // another device of the same type already exists
    errorMessage = QObject::tr("Only a single %1 node can be inserted in the '%2' field of a %3 node.")
                     .arg(childModelName)
                     .arg(fieldName)
                     .arg(parentModelName);
    return false;
  }

  return ::isAllowedToInsert(fieldName, childModelName, node, errorMessage,
                             isInBoundingObject ? OmNode::BOUNDING_OBJECT_USE : OmNode::STRUCTURE_USE,
                             OmNodeUtilities::slotType(childNode));
}

bool OmNodeUtilities::isAllowedToInsert(const OmField *const field, const OmNode *node, QString &errorMessage,
                                        OmNode::NodeUse nodeUse, const QString &type, const QString &newNodeModelName,
                                        const OmNodeModel *newNodeBaseModel, const QStringList &newNodeProtoParentList,
                                        bool automaticBoundingObjectCheck) {
  if (field->hasRestrictedValues() &&
      !doesFieldRestrictionAcceptNode(field, newNodeModelName, newNodeBaseModel, newNodeProtoParentList))
    return false;
  if (field->isParameter()) {
    foreach (OmField *internalField, field->internalFields()) {
      bool valid;
      if (internalField->isParameter())
        // recursive call: check only node field names and not parameter names
        valid = isAllowedToInsert(internalField, internalField->parentNode(), errorMessage, OmNode::UNKNOWN_USE, type,
                                  newNodeModelName, newNodeBaseModel, newNodeProtoParentList, automaticBoundingObjectCheck);
      else {
        const OmNode *parentNode = internalField->parentNode();
        valid = ::isAllowedToInsert(internalField->name(), newNodeBaseModel->name(), parentNode, errorMessage,
                                    static_cast<const OmBaseNode *>(parentNode)->nodeUse(), type, automaticBoundingObjectCheck);
      }
      if (!valid)
        return false;
    }
    return true;
  } else
    return ::isAllowedToInsert(field->name(), newNodeBaseModel->name(), node, errorMessage, nodeUse, type,
                               automaticBoundingObjectCheck);
}

OmNodeUtilities::Answer OmNodeUtilities::isSuitableForTransform(const OmNode *const srcNode, const QString &destModelName,
                                                                int *hasDeviceDescendantFlag) {
  const QString &srcModelName = srcNode->nodeModelName();

  // cannot transform into same type
  if (srcModelName == destModelName)
    return UNSUITABLE;

  OmNode::NodeUse nodeUse = OmNodeUtilities::checkNodeUse(srcNode);
  if (nodeUse & OmNode::BOUNDING_OBJECT_USE) {
    if (srcModelName == "Group" && destModelName == "Pose")
      return SUITABLE;
    if (srcModelName == "Pose" && destModelName == "Group")
      return LOOSING_INFO;

    return UNSUITABLE;
  }

  if (srcModelName == "Group" || srcModelName == "Pose" || srcModelName == "Transform") {
    Answer ok;
    if (srcModelName == "Transform") {
      const OmTransform *transform = dynamic_cast<const OmTransform *>(srcNode);
      ok = transform && transform->scale() != OmVector3(1, 1, 1) ? LOOSING_INFO : SUITABLE;
    } else
      ok = SUITABLE;

    if (destModelName == "Transform" || destModelName == "Pose" || destModelName == "Solid")
      return ok;

    if (destModelName == "Group") {
      const OmPose *p = dynamic_cast<const OmPose *>(srcNode);
      const bool pose = p && (p->translation() != OmVector3(0, 0, 0) || p->rotation().angle() != 0);
      return pose ? LOOSING_INFO : ok;
    }

    if (srcNode->isTopLevel())
      return (destModelName == "Charger" || isRobotTypeName(destModelName)) ? ok : UNSUITABLE;

    if (isSolidDeviceTypeName(destModelName))
      return hasARobotAncestor(srcNode) ? ok : UNSUITABLE;

    return UNSUITABLE;
  }

  if (destModelName == "Group" || destModelName == "Pose" || destModelName == "Transform") {
    if (isSolidTypeName(srcModelName)) {
      bool hasDevices;
      if (hasDeviceDescendantFlag && *hasDeviceDescendantFlag >= 0)  // read cached value
        hasDevices = *hasDeviceDescendantFlag == 1;
      else {
        hasDevices = hasADeviceDescendant(srcNode, true);
        if (hasDeviceDescendantFlag)
          *hasDeviceDescendantFlag = hasDevices ? 1 : 0;
      }

      if (hasDevices)
        return hasARobotAncestor(srcNode->parentNode()) ? LOOSING_INFO : UNSUITABLE;

      const OmSolid *upperSolid = findUpperSolid(srcNode);
      if (!upperSolid && hasAJointDescendant(srcNode))
        return UNSUITABLE;

      return !upperSolid && hasADeviceDescendant(srcNode, false) ? UNSUITABLE : LOOSING_INFO;
    }

    return UNSUITABLE;
  }

  if (isRobotTypeName(srcModelName)) {
    if (destModelName == "Solid" || destModelName == "Charger" || destModelName == "Connector") {
      bool hasDevices;
      if (hasDeviceDescendantFlag && *hasDeviceDescendantFlag >= 0)  // read cached value
        hasDevices = *hasDeviceDescendantFlag == 1;
      else {
        hasDevices = hasADeviceDescendant(srcNode, true);
        if (hasDeviceDescendantFlag)
          *hasDeviceDescendantFlag = hasDevices ? 1 : 0;
      }
      if (destModelName == "Solid" || destModelName == "Charger")
        return hasDevices ? UNSUITABLE : LOOSING_INFO;
      if (destModelName == "Connector") {
        return (hasDevices || !findUpperSolid(srcNode)) ? UNSUITABLE : LOOSING_INFO;
      }
    }

    return UNSUITABLE;
  }

  if (srcModelName == "Charger")
    return destModelName == "Robot" || destModelName == "Solid" || (destModelName == "Connector" && findUpperSolid(srcNode)) ?
             LOOSING_INFO :
             UNSUITABLE;

  if (isSolidTypeName(srcModelName)) {
    if (srcNode->isTopLevel())
      return (destModelName == "Solid" || isRobotTypeName(destModelName) || destModelName == "Charger") ? SUITABLE : UNSUITABLE;

    if (isSolidDeviceTypeName(srcModelName)) {
      if (destModelName == "Solid")
        return LOOSING_INFO;
      if (isSolidDeviceTypeName(destModelName))
        return (srcModelName != "Connector" || hasARobotAncestor(srcNode)) ? LOOSING_INFO : UNSUITABLE;
    } else if (isSolidDeviceTypeName(destModelName))
      return ((destModelName == "Connector" && findUpperSolid(srcNode)) || hasARobotAncestor(srcNode)) ? SUITABLE : UNSUITABLE;

    return UNSUITABLE;
  }

  return UNSUITABLE;
}

QString OmNodeUtilities::slotType(const OmNode *node) {
  const OmSlot *slot = dynamic_cast<const OmSlot *>(node);
  if (slot)
    return slot->slotType();
  else
    return "";
}

bool OmNodeUtilities::isAValidUseableNode(const OmNode *node, QString *warning) {
  OmNode *const n = const_cast<OmNode *>(node);

  const OmSolid *const solid = dynamic_cast<OmSolid *>(n);
  if (solid) {
    if (warning)
      *warning = QObject::tr("Solid nodes cannot be USEd.");
    return false;
  }

  const OmBillboard *const billboard = dynamic_cast<OmBillboard *>(n);
  if (billboard) {
    if (warning)
      *warning = QObject::tr("Billboard nodes cannot be USEd.");
    return false;
  }

  const OmBasicJoint *const joint = dynamic_cast<OmBasicJoint *>(n);
  if (joint) {
    if (warning)
      *warning = QObject::tr("Joint nodes cannot be USEd.");
    return false;
  }

  const OmJointParameters *const jointParameters = dynamic_cast<OmJointParameters *>(n);
  if (jointParameters) {
    if (warning)
      *warning = QObject::tr("JointParameters nodes cannot be USEd.");
    return false;
  }

  const OmTrackWheel *const trackWheel = dynamic_cast<OmTrackWheel *>(n);
  if (trackWheel) {
    if (warning)
      *warning = QObject::tr("TrackWheel nodes cannot be USEd.");
    return false;
  }

  const OmBallJointParameters *const ballJointParameters = dynamic_cast<OmBallJointParameters *>(n);
  if (ballJointParameters) {
    if (warning)
      *warning = QObject::tr("BallJointParameters nodes cannot be USEd.");
    return false;
  }

  const OmLogicalDevice *const logicalDevice = dynamic_cast<OmLogicalDevice *>(n);
  if (logicalDevice) {
    if (warning)
      *warning = QObject::tr("Device nodes cannot be USEd.");
    return false;
  }

  if (hasASolidDescendant(node)) {
    if (warning)
      *warning = QObject::tr("Nodes with a Solid descendant cannot be USEd.");
    return false;
  }

  if (hasAJointDescendant(node)) {
    if (warning)
      *warning = QObject::tr("Nodes with a Joint descendant cannot be USEd.");
    return false;
  }

  return true;
}

bool OmNodeUtilities::hasASolidDescendant(const OmNode *node) {
  if (node == NULL)
    return false;
  OmNode *const n = const_cast<OmNode *>(node);

  const OmSlot *const slot = dynamic_cast<OmSlot *>(n);
  // cppcheck-suppress knownConditionTrueFalse
  if (slot) {
    OmNode *endPoint = slot->endPoint();
    if (endPoint)
      return dynamic_cast<OmSolid *>(endPoint) || hasASolidDescendant(endPoint);
    return false;
  }

  const OmBasicJoint *const joint = dynamic_cast<OmBasicJoint *>(n);
  if (joint)
    return joint->solidEndPoint() != NULL;

  const OmGroup *const group = dynamic_cast<OmGroup *>(n);
  if (group == NULL)
    return false;

  OmMFNode::Iterator it(group->children());
  while (it.hasNext()) {
    OmNode *const next = it.next();
    if (dynamic_cast<OmSolid *>(next) || hasASolidDescendant(next))
      return true;
  }

  return false;
}

bool OmNodeUtilities::hasAJointDescendant(const OmNode *node) {
  if (node == NULL)
    return false;
  OmNode *const n = const_cast<OmNode *>(node);

  const OmSlot *const slot = dynamic_cast<OmSlot *>(n);
  // cppcheck-suppress knownConditionTrueFalse
  if (slot)
    return hasAJointDescendant(slot->endPoint());

  const OmGroup *const group = dynamic_cast<OmGroup *>(n);
  if (group == NULL)
    return false;

  OmMFNode::Iterator it(group->children());
  while (it.hasNext()) {
    OmNode *const next = it.next();
    if (dynamic_cast<OmBasicJoint *>(next) || hasAJointDescendant(next))
      return true;
  }

  return false;
}

////////////////////////////////////////
// Lookup related to DEF name changes //
////////////////////////////////////////

OmBoundingSphere *OmNodeUtilities::boundingSphereAncestor(const OmNode *node) {
  const OmNode *n = node;
  while (n) {
    const OmBaseNode *currentBaseNode = dynamic_cast<const OmBaseNode *>(n);
    if (currentBaseNode && currentBaseNode->boundingSphere()) {
      currentBaseNode->boundingSphere()->recomputeIfNeeded(false);
      if (!currentBaseNode->boundingSphere()->isEmpty()) {
        return currentBaseNode->boundingSphere();
      }
    }
    if (n->isTopLevel())
      break;
    n = n->parentNode();
  }
  return NULL;
}
