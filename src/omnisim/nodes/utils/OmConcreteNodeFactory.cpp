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

#include "OmConcreteNodeFactory.hpp"

#include "OmAccelerometer.hpp"
#include "OmAltimeter.hpp"
#include "OmAppearance.hpp"
#include "OmBackground.hpp"
#include "OmBallJoint.hpp"
#include "OmBallJointParameters.hpp"
#include "OmBillboard.hpp"
#include "OmBox.hpp"
#include "OmBrake.hpp"
#include "OmCadShape.hpp"
#include "OmCamera.hpp"
#include "OmCapsule.hpp"
#include "OmCharger.hpp"
#include "OmCloth.hpp"
#include "OmColor.hpp"
#include "OmCompass.hpp"
#include "OmCone.hpp"
#include "OmConnector.hpp"
#include "OmContactProperties.hpp"
#include "OmCoordinate.hpp"
#include "OmCylinder.hpp"
#include "OmDamping.hpp"
#include "OmDirectionalLight.hpp"
#include "OmDisplay.hpp"
#include "OmDistanceSensor.hpp"
#include "OmElevationGrid.hpp"
#include "OmEmitter.hpp"
#include "OmFocus.hpp"
#include "OmFog.hpp"
#include "OmGps.hpp"
#include "OmGranularGroup.hpp"
#include "OmGroup.hpp"
#include "OmGyro.hpp"
#include "OmHinge2Joint.hpp"
#include "OmHingeJointParameters.hpp"
#include "OmImageTexture.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmIndexedLineSet.hpp"
#include "OmInertialUnit.hpp"
#include "OmLed.hpp"
#include "OmLens.hpp"
#include "OmLensFlare.hpp"
#include "OmLidar.hpp"
#include "OmLightSensor.hpp"
#include "OmLinearMotor.hpp"
#include "OmLog.hpp"
#include "OmMaterial.hpp"
#include "OmMesh.hpp"
#include "OmMuscle.hpp"
#include "OmNodeModel.hpp"
#include "OmNodeUtilities.hpp"
#include "OmNormal.hpp"
#include "OmPbrAppearance.hpp"
#include "OmPen.hpp"
#include "OmPhysics.hpp"
#include "OmPlane.hpp"
#include "OmPointLight.hpp"
#include "OmPointSet.hpp"
#include "OmPose.hpp"
#include "OmPositionSensor.hpp"
#include "OmPropeller.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmRadar.hpp"
#include "OmRangeFinder.hpp"
#include "OmReceiver.hpp"
#include "OmRecognition.hpp"
#include "OmRobot.hpp"
#include "OmRotationalMotor.hpp"
#include "OmShape.hpp"
#include "OmSliderJoint.hpp"
#include "OmSlot.hpp"
#include "OmSoftBody.hpp"
#include "OmSolid.hpp"
#include "OmSolidReference.hpp"
#include "OmSpeaker.hpp"
#include "OmSphere.hpp"
#include "OmSpotLight.hpp"
#include "OmTemplateManager.hpp"
#include "OmTextureCoordinate.hpp"
#include "OmTextureTransform.hpp"
#include "OmTokenizer.hpp"
#include "OmTouchSensor.hpp"
#include "OmTrack.hpp"
#include "OmTrackWheel.hpp"
#include "OmTransform.hpp"
#include "OmUrl.hpp"
#include "OmVacuumGripper.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"
#include "OmWorldInfo.hpp"
#include "OmZoom.hpp"

#include <QtCore/QStringList>

// this creates and destructs the global instance
OmConcreteNodeFactory OmConcreteNodeFactory::gFactory;

OmNode *OmConcreteNodeFactory::createNode(const QString &modelName, OmTokenizer *tokenizer, OmNode *parentNode,
                                          const QString *protoUrl) {
  if (modelName == "Accelerometer")
    return new OmAccelerometer(tokenizer);
  if (modelName == "Altimeter")
    return new OmAltimeter(tokenizer);
  if (modelName == "Appearance")
    return new OmAppearance(tokenizer);
  if (modelName == "Background")
    return new OmBackground(tokenizer);
  if (modelName == "BallJoint")
    return new OmBallJoint(tokenizer);
  if (modelName == "BallJointParameters")
    return new OmBallJointParameters(tokenizer);
  if (modelName == "Billboard")
    return new OmBillboard(tokenizer);
  if (modelName == "Box")
    return new OmBox(tokenizer);
  if (modelName == "Brake")
    return new OmBrake(tokenizer);
  if (modelName == "Camera")
    return new OmCamera(tokenizer);
  if (modelName == "Capsule")
    return new OmCapsule(tokenizer);
  if (modelName == "Charger")
    return new OmCharger(tokenizer);
  if (modelName == "CadShape")
    return new OmCadShape(tokenizer);
  if (modelName == "Cloth")
    return new OmCloth(tokenizer);
  if (modelName == "Color")
    return new OmColor(tokenizer);
  if (modelName == "Compass")
    return new OmCompass(tokenizer);
  if (modelName == "Cone")
    return new OmCone(tokenizer);
  if (modelName == "Connector")
    return new OmConnector(tokenizer);
  if (modelName == "ContactProperties")
    return new OmContactProperties(tokenizer);
  if (modelName == "Coordinate")
    return new OmCoordinate(tokenizer);
  if (modelName == "Cylinder")
    return new OmCylinder(tokenizer);
  if (modelName == "Damping")
    return new OmDamping(tokenizer);
  if (modelName == "DirectionalLight")
    return new OmDirectionalLight(tokenizer);
  if (modelName == "Display")
    return new OmDisplay(tokenizer);
  if (modelName == "DistanceSensor")
    return new OmDistanceSensor(tokenizer);
  if (modelName == "ElevationGrid")
    return new OmElevationGrid(tokenizer);
  if (modelName == "Emitter")
    return new OmEmitter(tokenizer);
  if (modelName == "Focus")
    return new OmFocus(tokenizer);
  if (modelName == "Fog")
    return new OmFog(tokenizer);
  if (modelName == "GPS")
    return new OmGps(tokenizer);
  if (modelName == "GranularGroup")
    return new OmGranularGroup(tokenizer);
  if (modelName == "Group")
    return new OmGroup(tokenizer);
  if (modelName == "Gyro")
    return new OmGyro(tokenizer);
  if (modelName == "Hinge2Joint")
    return new OmHinge2Joint(tokenizer);
  if (modelName == "Hinge2JointParameters")
    return new OmHingeJointParameters(tokenizer, true);  // DEPRECATED, only for backward compatibility
  if (modelName == "HingeJoint")
    return new OmHingeJoint(tokenizer);
  if (modelName == "HingeJointParameters")
    return new OmHingeJointParameters(tokenizer);
  if (modelName == "ImageTexture")
    return new OmImageTexture(tokenizer);
  if (modelName == "IndexedFaceSet")
    return new OmIndexedFaceSet(tokenizer);
  if (modelName == "IndexedLineSet")
    return new OmIndexedLineSet(tokenizer);
  if (modelName == "InertialUnit")
    return new OmInertialUnit(tokenizer);
  if (modelName == "JointParameters")
    return new OmJointParameters(tokenizer);
  if (modelName == "LED")
    return new OmLed(tokenizer);
  if (modelName == "Lens")
    return new OmLens(tokenizer);
  if (modelName == "LensFlare")
    return new OmLensFlare(tokenizer);
  if (modelName == "Lidar")
    return new OmLidar(tokenizer);
  if (modelName == "LightSensor")
    return new OmLightSensor(tokenizer);
  if (modelName == "LinearMotor")
    return new OmLinearMotor(tokenizer);
  if (modelName == "Mesh")
    return new OmMesh(tokenizer);
  if (modelName == "Material")
    return new OmMaterial(tokenizer);
  if (modelName == "Muscle")
    return new OmMuscle(tokenizer);
  if (modelName == "Normal")
    return new OmNormal(tokenizer);
  if (modelName == "PBRAppearance")
    return new OmPbrAppearance(tokenizer);
  if (modelName == "Pen")
    return new OmPen(tokenizer);
  if (modelName == "Physics")
    return new OmPhysics(tokenizer);
  if (modelName == "Plane")
    return new OmPlane(tokenizer);
  if (modelName == "PointLight")
    return new OmPointLight(tokenizer);
  if (modelName == "PointSet")
    return new OmPointSet(tokenizer);
  if (modelName == "PositionSensor")
    return new OmPositionSensor(tokenizer);
  if (modelName == "Pose")
    return new OmPose(tokenizer);
  if (modelName == "Propeller")
    return new OmPropeller(tokenizer);
  if (modelName == "Radar")
    return new OmRadar(tokenizer);
  if (modelName == "RangeFinder")
    return new OmRangeFinder(tokenizer);
  if (modelName == "Receiver")
    return new OmReceiver(tokenizer);
  if (modelName == "Recognition")
    return new OmRecognition(tokenizer);
  if (modelName == "Robot")
    return new OmRobot(tokenizer);
  if (modelName == "RotationalMotor")
    return new OmRotationalMotor(tokenizer);
  if (modelName == "Shape")
    return new OmShape(tokenizer);
  if (modelName == "SliderJoint")
    return new OmSliderJoint(tokenizer);
  if (modelName == "Slot")
    return new OmSlot(tokenizer);
  if (modelName == "SoftBody")
    return new OmSoftBody(tokenizer);
  if (modelName == "Solid")
    return new OmSolid(tokenizer);
  if (modelName == "SolidReference")
    return new OmSolidReference(tokenizer);
  if (modelName == "Speaker")
    return new OmSpeaker(tokenizer);
  if (modelName == "Sphere")
    return new OmSphere(tokenizer);
  if (modelName == "SpotLight")
    return new OmSpotLight(tokenizer);
  if (modelName == "TextureCoordinate")
    return new OmTextureCoordinate(tokenizer);
  if (modelName == "TextureTransform")
    return new OmTextureTransform(tokenizer);
  if (modelName == "TouchSensor")
    return new OmTouchSensor(tokenizer);
  if (modelName == "Track")
    return new OmTrack(tokenizer);
  if (modelName == "TrackWheel")
    return new OmTrackWheel(tokenizer);
  if (modelName == "Transform") {
    if (OmWorld::instance() && OmWorld::instance()->isLoading())
      return OmVrmlNodeUtilities::transformBackwardCompatibility(tokenizer) ? new OmPose(tokenizer) :
                                                                              new OmTransform(tokenizer);
    return new OmTransform(tokenizer);
  }
  if (modelName == "VacuumGripper")
    return new OmVacuumGripper(tokenizer);
  if (modelName == "Viewpoint")
    return new OmViewpoint(tokenizer);
  if (modelName == "WorldInfo")
    return new OmWorldInfo(tokenizer);
  if (modelName == "Zoom")
    return new OmZoom(tokenizer);

  // look for PROTOs
  OmProtoModel *model;
  const QString &worldPath = OmWorld::instance() ? OmWorld::instance()->fileName() : "";
  if (protoUrl) {
    const QString prefix = OmUrl::computePrefix(*protoUrl);
    model = OmProtoManager::instance()->readModel(*protoUrl, worldPath, prefix);
  } else {
    const QString &parentFilePath = tokenizer->fileName().isEmpty() ? tokenizer->referralFile() : tokenizer->fileName();
    model = OmProtoManager::instance()->findModel(modelName, worldPath, parentFilePath);
  }

  if (!model)
    return NULL;

  // reset global parent that could be changed while parsing the PROTO model
  if (parentNode)
    OmNode::setGlobalParentNode(parentNode);
  OmNode *protoInstance =
    OmNode::createProtoInstance(model, tokenizer, OmWorld::instance() ? OmWorld::instance()->fileName() : "");
  if (protoInstance)
    OmTemplateManager::instance()->subscribe(protoInstance, false);

  OmNodeUtilities::fixBackwardCompatibility(protoInstance);

  return protoInstance;
}

OmNode *OmConcreteNodeFactory::createCopy(const OmNode &original) {
  const QString &modelName = original.nodeModelName();

  if (modelName == "Accelerometer")
    return new OmAccelerometer(original);
  if (modelName == "Altimeter")
    return new OmAltimeter(original);
  if (modelName == "Appearance")
    return new OmAppearance(original);
  if (modelName == "Background")
    return new OmBackground(original);
  if (modelName == "BallJoint")
    return new OmBallJoint(original);
  if (modelName == "BallJointParameters")
    return new OmBallJointParameters(original);
  if (modelName == "Billboard")
    return new OmBillboard(original);
  if (modelName == "Box")
    return new OmBox(original);
  if (modelName == "Brake")
    return new OmBrake(original);
  if (modelName == "CadShape")
    return new OmCadShape(original);
  if (modelName == "Camera")
    return new OmCamera(original);
  if (modelName == "Capsule")
    return new OmCapsule(original);
  if (modelName == "Charger")
    return new OmCharger(original);
  if (modelName == "Cloth")
    return new OmCloth(original);
  if (modelName == "Color")
    return new OmColor(original);
  if (modelName == "Compass")
    return new OmCompass(original);
  if (modelName == "Cone")
    return new OmCone(original);
  if (modelName == "Connector")
    return new OmConnector(original);
  if (modelName == "ContactProperties")
    return new OmContactProperties(original);
  if (modelName == "Coordinate")
    return new OmCoordinate(original);
  if (modelName == "Cylinder")
    return new OmCylinder(original);
  if (modelName == "Damping")
    return new OmDamping(original);
  if (modelName == "DirectionalLight")
    return new OmDirectionalLight(original);
  if (modelName == "Display")
    return new OmDisplay(original);
  if (modelName == "DistanceSensor")
    return new OmDistanceSensor(original);
  if (modelName == "ElevationGrid")
    return new OmElevationGrid(original);
  if (modelName == "Emitter")
    return new OmEmitter(original);
  if (modelName == "Focus")
    return new OmFocus(original);
  if (modelName == "Fog")
    return new OmFog(original);
  if (modelName == "GPS")
    return new OmGps(original);
  if (modelName == "GranularGroup")
    return new OmGranularGroup(original);
  if (modelName == "Group")
    return new OmGroup(original);
  if (modelName == "Gyro")
    return new OmGyro(original);
  if (modelName == "Hinge2Joint")
    return new OmHinge2Joint(original);
  if (modelName == "Hinge2JointParameters")
    return new OmHingeJointParameters(original, true);  // DEPRECATED, only for backward compatibility
  if (modelName == "HingeJoint")
    return new OmHingeJoint(original);
  if (modelName == "HingeJointParameters")
    return new OmHingeJointParameters(original);
  if (modelName == "ImageTexture")
    return new OmImageTexture(original);
  if (modelName == "IndexedFaceSet")
    return new OmIndexedFaceSet(original);
  if (modelName == "IndexedLineSet")
    return new OmIndexedLineSet(original);
  if (modelName == "InertialUnit")
    return new OmInertialUnit(original);
  if (modelName == "JointParameters")
    return new OmJointParameters(original);
  if (modelName == "LED")
    return new OmLed(original);
  if (modelName == "Lens")
    return new OmLens(original);
  if (modelName == "LensFlare")
    return new OmLensFlare(original);
  if (modelName == "Lidar")
    return new OmLidar(original);
  if (modelName == "LightSensor")
    return new OmLightSensor(original);
  if (modelName == "LinearMotor")
    return new OmLinearMotor(original);
  if (modelName == "Material")
    return new OmMaterial(original);
  if (modelName == "Mesh")
    return new OmMesh(original);
  if (modelName == "Muscle")
    return new OmMuscle(original);
  if (modelName == "Normal")
    return new OmNormal(original);
  if (modelName == "PBRAppearance")
    return new OmPbrAppearance(original);
  if (modelName == "Pen")
    return new OmPen(original);
  if (modelName == "Physics")
    return new OmPhysics(original);
  if (modelName == "Plane")
    return new OmPlane(original);
  if (modelName == "PointLight")
    return new OmPointLight(original);
  if (modelName == "PointSet")
    return new OmPointSet(original);
  if (modelName == "PositionSensor")
    return new OmPositionSensor(original);
  if (modelName == "Pose")
    return new OmPose(original);
  if (modelName == "Propeller")
    return new OmPropeller(original);
  if (modelName == "Radar")
    return new OmRadar(original);
  if (modelName == "RangeFinder")
    return new OmRangeFinder(original);
  if (modelName == "Receiver")
    return new OmReceiver(original);
  if (modelName == "Recognition")
    return new OmRecognition(original);
  if (modelName == "Robot")
    return new OmRobot(original);
  if (modelName == "RotationalMotor")
    return new OmRotationalMotor(original);
  if (modelName == "Shape")
    return new OmShape(original);
  if (modelName == "SliderJoint")
    return new OmSliderJoint(original);
  if (modelName == "Slot")
    return new OmSlot(original);
  if (modelName == "SoftBody")
    return new OmSoftBody(original);
  if (modelName == "Solid")
    return new OmSolid(original);
  if (modelName == "SolidReference")
    return new OmSolidReference(original);
  if (modelName == "Speaker")
    return new OmSpeaker(original);
  if (modelName == "Sphere")
    return new OmSphere(original);
  if (modelName == "SpotLight")
    return new OmSpotLight(original);
  if (modelName == "TextureCoordinate")
    return new OmTextureCoordinate(original);
  if (modelName == "TextureTransform")
    return new OmTextureTransform(original);
  if (modelName == "TouchSensor")
    return new OmTouchSensor(original);
  if (modelName == "Track")
    return new OmTrack(original);
  if (modelName == "TrackWheel")
    return new OmTrackWheel(original);
  if (modelName == "Transform")
    return new OmTransform(original);
  if (modelName == "VacuumGripper")
    return new OmVacuumGripper(original);
  if (modelName == "Viewpoint")
    return new OmViewpoint(original);
  if (modelName == "WorldInfo")
    return new OmWorldInfo(original);
  if (modelName == "Zoom")
    return new OmZoom(original);

  return NULL;
}

const QString OmConcreteNodeFactory::slotType(OmNode *node) {
  return OmNodeUtilities::slotType(node);
}

bool OmConcreteNodeFactory::validateExistingChildNode(const OmField *field, const OmNode *childNode, const OmNode *node,
                                                      bool isInBoundingObject, QString &errorMessage) const {
  return OmNodeUtilities::validateExistingChildNode(field, childNode, node, isInBoundingObject, errorMessage);
}
