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

#include "WbSolid.hpp"

#include "WbBox.hpp"
#include "WbCapsule.hpp"
#include "WbCylinder.hpp"
#include "WbDamping.hpp"
#include "WbField.hpp"
#include "WbGeometry.hpp"
#include "WbGroup.hpp"
#include "WbSFString.hpp"
#include "WbImmersionProperties.hpp"
#include "WbIndexedFaceSet.hpp"
#include "WbJoint.hpp"
#include "WbBasicJoint.hpp"
#include "WbHingeJoint.hpp"
#include "WbSliderJoint.hpp"
#include "WbMesh.hpp"
#include "WbTriangleMesh.hpp"
#include "WbTriangleMeshGeometry.hpp"
#include "WbJointParameters.hpp"
#include "WbLog.hpp"
#include "WbMFNode.hpp"
#include "WbMFVector3.hpp"
#include "WbMassChecker.hpp"
#include "WbMathsUtilities.hpp"
#include "WbMatrix3.hpp"
#include "WbMatrix4.hpp"
#include "WbMatter.hpp"
#include "WbMotor.hpp"
#include "WbNodeOperations.hpp"
#include "WbNodeUtilities.hpp"
#include "WbOdeContact.hpp"
#include "WbOdeContext.hpp"
#include "WbOdeGeomData.hpp"
#include "WbNewtonBackend.hpp"
#include "WbPhysics.hpp"
#include "WbPhysicsBackend.hpp"
#include "WbPlane.hpp"
#include "WbPose.hpp"
#include "WbPropeller.hpp"
#include "WbResizeManipulator.hpp"
#include "WbRobot.hpp"
#include "WbRotation.hpp"
#include "WbShape.hpp"
#include "WbSimulationState.hpp"
#include "WbSlot.hpp"
#include "WbSolidMerger.hpp"
#include "WbSolidReference.hpp"
#include "WbSphere.hpp"
#include "WbSupportPolygonRepresentation.hpp"
#include "WbToken.hpp"
#include "WbTokenizer.hpp"
#include "WbVector4.hpp"
#include "WbViewpoint.hpp"
#include "WbVrmlNodeUtilities.hpp"
#include "WbWorld.hpp"
#include "WbWorldInfo.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

#include <wren/config.h>
#include <wren/material.h>
#include <wren/node.h>
#include <wren/renderable.h>
#include <wren/scene.h>
#include <wren/static_mesh.h>
#include <wren/transform.h>

#include <ode/fluid_dynamics/ode_fluid_dynamics.h>

#include <QtCore/QFile>
#include <QtCore/QQueue>
#include <QtCore/QRegularExpression>
#include <QtCore/QStringList>

#include <cstdlib>

using namespace WbHiddenKinematicParameters;
using namespace WbSolidUtilities;
using namespace std;

const double WbSolid::MASS_ZERO_THRESHOLD = 1e-10;
const double REFERENCE_DENSITY = 1000.0;

QList<const WbSolid *> WbSolid::cSolids;

void WbSolid::init() {
  // ODE stuff
  mJoint = NULL;
  mOdeMass = NULL;
  mMassAroundCoM = NULL;
  mReferenceMass = NULL;

  // Webots physics data
  mGlobalMass = 0.0;
  mGlobalVolume = 0.0;
  mGlobalCenterOfMass = WbVector3();
  mCenterOfMass = WbVector3();

  // Flags
  mWasSleeping = false;
  mBoundingObjectHasChanged = false;
  mSelected = false;
  mHasSearchedRobot = false;
  mHasExtractedContactPoints = false;
  mUseInertiaMatrix = false;
  mIsPermanentlyKinematic = false;
  mIsKinematic = false;
  mUpdatedInStep = false;
  mResetPhysicsInStep = false;
  mKinematicWarningPrinted = false;
  mHasDynamicSolidDescendant = false;
  mNameClashResolved = false;

  // Newton dispatcher (cuda-newton-physics-plan.md P3.2): -1 means not
  // registered with WbNewtonBackend. Populated in postFinalize when
  // physicsBackend resolves to an available Newton backend.
  mNewtonBodyIndex = -1;
  mNewtonBodyIsStatic = false;

  // Merger
  mSolidMerger = NULL;
  mMergerIsSet = false;

  // Support polygon representation
  mY = numeric_limits<double>::max();
  mSupportPolygon = WbPolygon();
  mSupportPolygonNeedsUpdate = false;
  mSupportPolygonRepresentationIsEnabled = false;
  mSupportPolygonRepresentation = NULL;

  // Center of mass representation
  mCenterOfMassTransform = NULL;
  mCenterOfMassMaterial = NULL;
  mCenterOfMassMesh = NULL;
  mCenterOfMassRenderable = NULL;

  // Global center of mass representation
  mGlobalCenterOfMassRepresentationIsEnabled = false;
  mGlobalCenterOfMassTransform = NULL;
  mGlobalCenterOfMassMaterial = NULL;
  mGlobalCenterOfMassMesh = NULL;
  mGlobalCenterOfMassRenderable = NULL;

  // Center of buoyancy representation
  mCenterOfBuoyancyRepresentationIsEnabled = false;
  mHasExtractedImmersions = false;
  mCenterOfBuoyancyTransform = NULL;
  mCenterOfBuoyancyMaterial = NULL;
  mCenterOfBuoyancyRenderable = NULL;

  // user fields
  mContactMaterial = findSFString("contactMaterial");
  mPhysicsBackend = findSFString("physicsBackend");
  mImmersionProperties = findMFNode("immersionProperties");
  mBoundingObject = findSFNode("boundingObject");
  mPhysics = findSFNode("physics");
  mRadarCrossSection = findSFDouble("radarCrossSection");
  mRecognitionColors = findMFColor("recognitionColors");

  // hidden fields
  mLinearVelocity = findSFVector3("linearVelocity");
  mAngularVelocity = findSFVector3("angularVelocity");

  if (mLinearVelocity && mAngularVelocity) {
    updateIsLinearVelocityNull();
    updateIsAngularVelocityNull();
    connect(mLinearVelocity, &WbSFVector3::changed, this, &WbSolid::updateIsLinearVelocityNull);
    connect(mAngularVelocity, &WbSFVector3::changed, this, &WbSolid::updateIsAngularVelocityNull);
  }

  mOriginalHiddenKinematicParameters = NULL;
}

WbSolid::WbSolid(WbTokenizer *tokenizer) : WbMatter("Solid", tokenizer) {
  init();
}

WbSolid::WbSolid(const WbSolid &other) : WbMatter(other) {
  init();
}

WbSolid::WbSolid(const WbNode &other) : WbMatter(other) {
  init();
}

WbSolid::WbSolid(const QString &modelName, WbTokenizer *tokenizer) : WbMatter(modelName, tokenizer) {
  init();
}

WbSolid::~WbSolid() {
  if (mRadarCrossSection->value() > 0.0)
    WbWorld::instance()->removeRadarTarget(this);

  if (!mRecognitionColors->isEmpty())
    WbWorld::instance()->removeCameraRecognitionObject(this);

  qDeleteAll(mHiddenKinematicParametersMap);
  mHiddenKinematicParametersMap.clear();

  cSolids.removeAll(this);

  // Cleanup WREN
  if (areWrenObjectsInitialized()) {
    // Center of mass
    wr_node_delete(WR_NODE(mCenterOfMassTransform));
    wr_node_delete(WR_NODE(mCenterOfMassRenderable));
    wr_material_delete(mCenterOfMassMaterial);
    wr_static_mesh_delete(mCenterOfMassMesh);

    // Global center of mass
    wr_node_delete(WR_NODE(mGlobalCenterOfMassTransform));
    wr_node_delete(WR_NODE(mGlobalCenterOfMassRenderable));
    wr_material_delete(mGlobalCenterOfMassMaterial);
    wr_static_mesh_delete(mGlobalCenterOfMassMesh);

    // Center of buoyancy
    wr_node_delete(WR_NODE(mCenterOfBuoyancyTransform));
    wr_node_delete(WR_NODE(mCenterOfBuoyancyRenderable));
    wr_material_delete(mCenterOfBuoyancyMaterial);
  }

  delete mSupportPolygonRepresentation;
  mSupportPolygonRepresentation = NULL;

  if (isSolidMerger())
    delete mSolidMerger.data();

  if (!mSolidMerger.isNull())
    mSolidMerger->removeSolid(this);

  // cleanup ODE
  delete mReferenceMass;
  mReferenceMass = NULL;
  delete mOdeMass;
  mOdeMass = NULL;
  delete mMassAroundCoM;
  mMassAroundCoM = NULL;
  if (mJoint)
    dJointDestroy(mJoint);
  mJoint = NULL;

  // disconnecting descendants
  foreach (const WbSolid *const solid, mSolidChildren)
    disconnect(solid, &WbSolid::destroyed, this, 0);
}

void WbSolid::deleteAllSolids() {
  foreach (WbSolid *const solid, mSolidChildren)
    WbNodeOperations::instance()->deleteNode(solid);
  mSolidChildren.clear();
}

void WbSolid::validateProtoNode() {
  if (isProtoInstance()) {
    bool checkTranslation = !isTranslationFieldVisible();
    bool checkRotation = !isRotationFieldVisible();
    if (!(checkTranslation || checkRotation))
      return;

    foreach (const WbField *parameter, parameters()) {
      if (checkTranslation && parameter->name() == "translation" && parameter->isTemplateRegenerator()) {
        parsingWarn(tr("template regenerator field named 'translation' found. "
                       "It is recommended not to use template statements to update the top Solid 'translation' field"));
        checkTranslation = false;
      } else if (checkRotation && parameter->name() == "rotation" && parameter->isTemplateRegenerator()) {
        parsingWarn(tr("template regenerator field named 'rotation' found. "
                       "It is recommended not to use template statements to update the top Solid 'rotation' field"));
        checkTranslation = false;
      }

      if (!(checkTranslation || checkRotation))
        break;
    }
  }
}

void WbSolid::downloadAssets() {
  WbGroup::downloadAssets();
  if (boundingObject())
    boundingObject()->downloadAssets();
}

void WbSolid::preFinalize() {
  mHasNoSolidAncestor = false;

  cSolids << this;

  updateChildren();

  WbMatter::preFinalize();

  if (physics())
    physics()->preFinalize();
  else
    mIsKinematic = true;

  if (mBoundingObject->value())
    boundingObject()->preFinalize();

  assert(mImmersionProperties);
  WbMFNode::Iterator it(*mImmersionProperties);
  while (it.hasNext()) {
    WbBaseNode *const ip = static_cast<WbImmersionProperties *>(it.next());
    ip->preFinalize();
  }

  setMatrixNeedUpdate();  // force the matrix update after the first ode update

  // needed to be done before createOdeObjects
  // because of the SolidMerger
  mOdeMass = new dMass;  // stores inertia and CoM relative to solid center in the local frame coordinates
  dMassSetZero(mOdeMass);
  mMassAroundCoM = new dMass;  // stores inertia and CoM relative to solid center in the local frame coordinates
  dMassSetZero(mMassAroundCoM);
  mReferenceMass = new dMass;
  dMassSetZero(mReferenceMass);
  mIsPermanentlyKinematic = WbSolidUtilities::isPermanentlyKinematic(this);  // cached because it can't be called in destructor
  if (mIsPermanentlyKinematic)
    mIsKinematic = true;

  // Overwrites loaded values with hidden field and hidden parameter values (translation, rotation, joint position), reads
  // initial velocities
  if (isProtoInstance() && !isNestedProtoNode()) {
    int counter = 0;
    if (!restoreHiddenKinematicParameters(mHiddenKinematicParametersMap, counter)) {
      bool success = resetHiddenKinematicParameters();
      if (success)
        WbLog::instance()->warning(tr("PROTO '%1' changed after this world was saved:\n"
                                      "hidden parameters have been automatically reset.\n\n"
                                      "Please save the current world to get rid of this message.")
                                     .arg(modelName()),
                                   true);
      else
        WbLog::instance()->warning(tr("PROTO '%1' changed after this world was saved:\n"
                                      "hidden parameters cannot be loaded correctly.\n\n"
                                      "Please save the current world to get rid of this message.")
                                     .arg(modelName()),
                                   true);
      qDeleteAll(mHiddenKinematicParametersMap);
      mHiddenKinematicParametersMap.clear();
    }
  }

  if (nodeType() != WB_NODE_TOUCH_SENSOR && nodeType() != WB_NODE_VACUUM_GRIPPER && mBoundingObject->value() &&
      mPhysics->value() == NULL && mJointParents.size() == 0 && upperSolid() && upperSolid()->physics()) {
    // P5 hang fix 2026-05-28: dedupe this warning at module scope.
    // The same warning fires once per URDF-imported visual-only Solid
    // (top_plate_link, top_chassis_link, etc. on Husky; many more on
    // Spot). At 40+ huskies the accumulated WbLog::warning cost stalls
    // world load. Cap at 5 occurrences; emit a single summary at the
    // cap. The warning is genuine-useful for hand-written worlds (one
    // is enough to convey the issue) and a known-benign artifact for
    // URDF imports.
    static int sNullPhysicsWarnCount = 0;
    if (sNullPhysicsWarnCount < 5) {
      ++sNullPhysicsWarnCount;
      parsingWarn(tr("As 'physics' is set to NULL, collisions will have no effect."));
    } else if (sNullPhysicsWarnCount == 5) {
      ++sNullPhysicsWarnCount;
      parsingWarn(tr("As 'physics' is set to NULL, collisions will have no effect. "
                     "(further occurrences in this session suppressed.)"));
    }
  }
}

bool WbSolid::restoreHiddenKinematicParameters(const HiddenKinematicParametersMap &map, int &counter) {
  if (!applyHiddenKinematicParameters(map.value(counter, NULL), true))
    return false;

  ++counter;

  foreach (WbSolid *const solid, mSolidChildren) {
    if (!solid->restoreHiddenKinematicParameters(map, counter))
      return false;
  }

  return true;
}

bool WbSolid::resetHiddenKinematicParameters() {
  foreach (WbSolid *const solid, mSolidChildren) {
    if (!solid->resetHiddenKinematicParameters())
      return false;
  }

  if (mOriginalHiddenKinematicParameters)
    return applyHiddenKinematicParameters(mOriginalHiddenKinematicParameters, false);

  return true;
}

bool WbSolid::applyHiddenKinematicParameters(const HiddenKinematicParameters *hkp, bool backupPrevious) {
  if (!hkp)
    return true;

  WbVector3 *previousT = NULL;
  WbRotation *previousR = NULL;
  WbVector3 *previousL = NULL;
  WbVector3 *previousA = NULL;
  PositionMap *previousP = NULL;

  const WbVector3 *const t = hkp->translation();
  if (t) {
    if (backupPrevious)
      previousT = new WbVector3(translation());
    WbPose::setTranslation(*t);
  }

  const WbRotation *const r = hkp->rotation();
  if (r) {
    if (backupPrevious)
      previousR = new WbRotation(rotation());
    WbPose::setRotation(*r);
  }

  const PositionMap *const m = hkp->positions();
  if (m) {
    if (backupPrevious)
      previousP = new PositionMap();

    const PositionMap::const_iterator end = m->constEnd();
    for (PositionMap::const_iterator i = m->constBegin(); i != end; ++i) {
      const WbVector3 *const p = i.value();
      if (!p)
        return false;
      const int jointIndex = i.key();
      assert(jointIndex < mJointChildren.length());
      WbJoint *const j = dynamic_cast<WbJoint *>(mJointChildren.at(jointIndex));
      if (!j)
        return false;

      if (backupPrevious) {
        WbVector3 v(NAN, NAN, NAN);
        const WbJointParameters *const param1 = j->parameters();
        if (param1)
          v[0] = j->position();
        const WbJointParameters *const param2 = j->parameters2();
        if (param2)
          v[1] = j->position(2);
        const WbJointParameters *const param3 = j->parameters3();
        if (param3)
          v[2] = j->position(3);
        previousP->insert(jointIndex, new WbVector3(v));
      }

      for (int k = 0; k < 3; ++k) {
        const double posk = (*p)[k];
        if (!std::isnan(posk))
          j->setPosition(posk, k + 1);
      }
    }
  }

  const WbVector3 *const l = hkp->linearVelocity();
  if (l) {
    if (backupPrevious)
      previousL = new WbVector3(mLinearVelocity->value());
    mLinearVelocity->setValue(*l);
  }

  const WbVector3 *const a = hkp->angularVelocity();
  if (a) {
    if (backupPrevious)
      previousA = new WbVector3(mAngularVelocity->value());
    mAngularVelocity->setValue(*a);
  }

  if (backupPrevious && (previousT || previousR || previousP || previousL || previousA)) {
    delete mOriginalHiddenKinematicParameters;
    mOriginalHiddenKinematicParameters = new HiddenKinematicParameters(previousT, previousR, previousP, previousL, previousA);
  }

  return true;
}

void WbSolid::postFinalize() {
  delete mOriginalHiddenKinematicParameters;
  mOriginalHiddenKinematicParameters = NULL;

  WbMatter::postFinalize();
  if (physics())
    physics()->postFinalize();

  // P2/P3 of cuda-newton-physics-plan.md: trigger backend resolution at
  // world-load time for any Solid that opts into a non-default backend.
  // Actual registration is deferred to flushPendingNewtonRegistrations()
  // (driven by WbSimulationWorld::step before finalizeWorld) so that
  // matrix() returns correct world coordinates -- at this point the
  // parent transform chain may not yet be computed for nested Solids
  // (e.g. wheels under a HingeJoint).
  //
  // Pure-ODE worlds: this branch is bypassed entirely (string compare
  // against "ode"), so they pay zero overhead.
  if (physicsBackendName() != QStringLiteral("ode"))
    (void)physicsBackend();  // bring up backend if needed; registry call_once

  WbMFNode::Iterator it(*mImmersionProperties);
  while (it.hasNext()) {
    WbBaseNode *const ip = static_cast<WbImmersionProperties *>(it.next());
    ip->postFinalize();
  }

  updateDynamicSolidDescendantFlag();

  connect(mTranslation, &WbSFVector3::changedByUser, this, &WbSolid::resetPhysicsIfRequired);
  connect(mRotation, &WbSFVector3::changedByUser, this, &WbSolid::resetPhysicsIfRequired);

  disconnectFieldNotification(rotationFieldValue());
  disconnectFieldNotification(translationFieldValue());
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbSolid::onSimulationModeChanged);
  connect(WbSimulationState::instance(), &WbSimulationState::renderingStateChanged, this, &WbSolid::onSimulationModeChanged);
  connect(this, &WbSolid::massPropertiesChanged, this, &WbSolid::displayWarning);
  connect(mPhysics, &WbSFNode::changed, this, &WbSolid::updatePhysics);
  connect(mRadarCrossSection, &WbSFDouble::changed, this, &WbSolid::updateRadarCrossSection);
  connect(mRecognitionColors, &WbMFColor::itemChanged, this, &WbSolid::updateRecognitionColors);
  connect(mRecognitionColors, &WbMFColor::itemRemoved, this, &WbSolid::updateRecognitionColors);
  connect(mRecognitionColors, &WbMFColor::itemInserted, this, &WbSolid::updateRecognitionColors);

  if (isTopSolid()) {
    updateGlobalCenterOfMass();
    updateGlobalVolume();
  }

  displayWarning();

  if (mRadarCrossSection->value() > 0.0)
    WbWorld::instance()->addRadarTarget(this);

  if (!mRecognitionColors->isEmpty())
    WbWorld::instance()->addCameraRecognitionObject(this);

  if (protoParameterNode()) {
    const QVector<WbNode *> nodes = protoParameterNode()->protoParameterNodeInstances();
    if (nodes.size() > 1 && nodes.at(0) == this)
      parsingWarn(tr("Solid node defined in PROTO field is used multiple times. "
                     "OmniSim doesn't fully support this because the multiple node instances cannot be identical."));
  }
}

void WbSolid::resolveNameClashIfNeeded(bool automaticallyChange, bool recursive, const QList<WbSolid *> &siblings,
                                       QSet<const QString> *topSolidNameSet) {
  const QString &warningText =
    tr("'name' field value should be unique: '%1' already used by a sibling Solid node.").arg(name());

  if (isProtoParameterNode() || siblings.isEmpty())
    return;

  if (topSolidNameSet && !automaticallyChange) {
    if (topSolidNameSet->contains(name()))
      parsingWarn(warningText);
    else
      topSolidNameSet->insert(name());
  } else {
    QList<int> indices;
    // extract name without index
    QRegularExpression re("(.+)\\(\\d+\\)$");
    QRegularExpressionMatch match = re.match(name());
    QString nameWithoutIndex(name());
    if (match.hasMatch())
      nameWithoutIndex = match.captured(1);

    // loop through sibling nodes
    const WbNode *parameterNode = protoParameterNode();
    while (parameterNode && parameterNode->protoParameterNode())
      parameterNode = parameterNode->protoParameterNode();
    const WbNode *visibleNode = parameterNode ? parameterNode : this;

    bool found = false;
    re.setPattern(QString("%1\\((\\d+)\\)").arg(QRegularExpression::escape(nameWithoutIndex)));
    foreach (const WbSolid *s, siblings) {
      if (!s || s == this)
        continue;

      const bool matchingName = s->name() == name();
      found |= matchingName;
      if (matchingName) {
        if (parameterNode != NULL) {
          // ensure that solid nodes doesn't refer to the same PROTO parameter node
          // otherwise we will loop forever
          const WbNode *otherParameterNode = s->protoParameterNode();
          while (otherParameterNode && otherParameterNode->protoParameterNode())
            otherParameterNode = otherParameterNode->protoParameterNode();
          if (otherParameterNode == parameterNode) {
            visibleNode->parsingWarn(
              warningText +
              tr(" A unique name cannot be automatically generated because the same PROTO parameter is used multiple times."));
            goto recursion;
          }
        }
      }
      match = re.match(s->name());
      if (match.hasMatch()) {
        indices << match.captured(1).toInt();
      }
    }

    if (found) {
      if (automaticallyChange) {
        WbField *nameField = findField("name", true);
        bool isTemplateRegenerator = false;
        while (nameField && !isTemplateRegenerator) {
          isTemplateRegenerator = WbNodeUtilities::isTemplateRegeneratorField(nameField);
          nameField = nameField->parameter();
        }
        if (isTemplateRegenerator)
          visibleNode->parsingWarn(
            warningText +
            tr(" A unique name cannot be automatically generated because 'name' is a template regenerator field."));
        else if (!WbVrmlNodeUtilities::isVisible(findField("name")))
          visibleNode->parsingWarn(warningText);
        else {
          // find first available index
          std::sort(indices.begin(), indices.end());
          int newIndex = 1;
          foreach (int i, indices) {
            if (i != newIndex)
              break;
            newIndex++;
          }
          const QString newName = QString("%1(%2)").arg(nameWithoutIndex).arg(newIndex);
          mNameClashResolved = true;
          mName->setValue(newName);
        }
      } else
        visibleNode->parsingWarn(warningText);
    }
  }

recursion:
  if (recursive) {
    QList<WbSolid *> solidChildrenList = mSolidChildren.toList();
    foreach (WbSolid *s, solidChildrenList)
      s->resolveNameClashIfNeeded(automaticallyChange, recursive, solidChildrenList, NULL);
  }
}

void WbSolid::updateName() {
  if (!mNameClashResolved) {
    const WbSolid *us = upperSolid();
    resolveNameClashIfNeeded(false, false, us ? us->solidChildren().toList() : WbWorld::instance()->topSolids(), NULL);
  } else
    // name field has just been updated in a previous call of resolveNameClashIfNeeded
    mNameClashResolved = false;
  WbMatter::updateName();
}

QString WbSolid::computeUniqueName() const {
  const WbSolid *solid = this;
  QString uniqueName;
  while (true) {
    QString name = solid->name();
    name.replace("\\", "\\\\");  // escape '\'
    name.replace(":", "\\:");    // escape ':'
    uniqueName.prepend(name);
    solid = solid->upperSolid();
    if (solid)
      uniqueName.prepend(":");
    else
      break;
  }
  return uniqueName;
}

WbSolid *WbSolid::findDescendantSolidFromUniqueName(QStringList &names) const {
  const WbSolid *solid = this;
  while (solid && !names.isEmpty()) {
    QString name = names.takeFirst();
    name.replace("\\:", ":");    // revert escape of ':'
    name.replace("\\\\", "\\");  // revert escape of '\'
    WbSolid *nextSolid = NULL;
    foreach (WbSolid *s, solid->mSolidChildren) {
      if (s->name() == name) {
        if (names.isEmpty())
          return s;
        nextSolid = s;
      }
    }
    solid = nextSolid;
  }
  return NULL;
}

WbSolid *WbSolid::findSolidFromUniqueName(const QString &name) {
  // Solid names joined by ':'
  QStringList names = splitUniqueNamesByEscapedPattern(name, ":");
  QString topName = names.takeFirst();
  topName.replace("\\:", ":");    // revert escape of ':'
  topName.replace("\\\\", "\\");  // revert escape of '\'
  foreach (WbSolid *solid, WbWorld::instance()->topSolids()) {
    if (solid->name() == topName)
      return names.isEmpty() ? solid : solid->findDescendantSolidFromUniqueName(names);
  }
  return NULL;
}

QStringList WbSolid::splitUniqueNamesByEscapedPattern(const QString &text, const QString &pattern) {
  QStringList result;
  // To check that the pattern matched and the first character is not escaped, given that
  // the name can end with '\' (i.e. '\\' because was escaped when writing), we have to
  // check that ':' is preceeded by zero or an even number of '\\'
  QRegularExpression re("[^\\\\](\\\\\\\\)*" + pattern);
  QRegularExpressionMatch match = re.match(text);
  int startIndex = 0;
  while (match.hasMatch()) {
    int capturedStart = match.capturedStart() + match.capturedLength() - pattern.size();
    result << text.mid(startIndex, capturedStart - startIndex);
    startIndex = match.capturedEnd();
    match = re.match(text, startIndex);
  }
  result << text.mid(startIndex, text.size() - startIndex);
  return result;
}

/////////////////////////
// Create WREN Objects //
/////////////////////////

void WbSolid::createWrenObjects() {
  // Center of mass representation
  const float centerOfMassMeshVertices[18] = {-0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,  0.0f, -0.5f, 0.0f,
                                              0.0f,  1.0f, 0.0f, 0.0f, 0.0f, -0.5f, 0.0f, 0.0f,  1.0f};
  const float colors[18] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                            0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f};
  mCenterOfMassMesh = wr_static_mesh_line_set_new(6, centerOfMassMeshVertices, colors);
  mCenterOfMassMaterial = wr_phong_material_new();
  wr_phong_material_set_color_per_vertex(mCenterOfMassMaterial, true);
  wr_phong_material_set_transparency(mCenterOfMassMaterial, 0.5f);
  wr_material_set_default_program(mCenterOfMassMaterial, WbWrenShaders::lineSetShader());

  mCenterOfMassRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mCenterOfMassRenderable, false);
  wr_renderable_set_receive_shadows(mCenterOfMassRenderable, false);
  wr_renderable_set_material(mCenterOfMassRenderable, mCenterOfMassMaterial, NULL);
  wr_renderable_set_mesh(mCenterOfMassRenderable, WR_MESH(mCenterOfMassMesh));
  wr_renderable_set_visibility_flags(mCenterOfMassRenderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
  wr_renderable_set_drawing_mode(mCenterOfMassRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_drawing_order(mCenterOfMassRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

  mCenterOfMassTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mCenterOfMassTransform), false);
  wr_transform_attach_child(mCenterOfMassTransform, WR_NODE(mCenterOfMassRenderable));

  // Global center of mass representation
  const float globalCenterOfMassVertices[18] = {
    -1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f,
  };

  mGlobalCenterOfMassMesh = wr_static_mesh_line_set_new(6, globalCenterOfMassVertices, NULL);

  mGlobalCenterOfMassMaterial = wr_phong_material_new();
  const float color[3] = {0.05f, 0.05f, 0.5f};
  wr_phong_material_set_color(mGlobalCenterOfMassMaterial, color);
  wr_phong_material_set_transparency(mGlobalCenterOfMassMaterial, 0.7f);
  wr_material_set_default_program(mGlobalCenterOfMassMaterial, WbWrenShaders::lineSetShader());

  mGlobalCenterOfMassRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mGlobalCenterOfMassRenderable, false);
  wr_renderable_set_receive_shadows(mGlobalCenterOfMassRenderable, false);
  wr_renderable_set_material(mGlobalCenterOfMassRenderable, mGlobalCenterOfMassMaterial, NULL);
  wr_renderable_set_mesh(mGlobalCenterOfMassRenderable, WR_MESH(mGlobalCenterOfMassMesh));
  wr_renderable_set_visibility_flags(mGlobalCenterOfMassRenderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
  wr_renderable_set_drawing_mode(mGlobalCenterOfMassRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_drawing_order(mGlobalCenterOfMassRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

  mGlobalCenterOfMassTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mGlobalCenterOfMassTransform), false);
  wr_transform_attach_child(mGlobalCenterOfMassTransform, WR_NODE(mGlobalCenterOfMassRenderable));

  WbMatter::createWrenObjects();

  wr_transform_attach_child(wrenNode(), WR_NODE(mCenterOfMassTransform));

  // Global center of mass is attached to root node as it is already in absolute coordinates and also it is easier to handle.
  wr_transform_attach_child(wr_scene_get_root(wr_scene_get_instance()), WR_NODE(mGlobalCenterOfMassTransform));

  // Center of buoyancy
  mCenterOfBuoyancyMaterial = wr_phong_material_new();
  const float centerOfBuoyancyColor[3] = {0.5f, 0.5f, 0.8f};
  wr_phong_material_set_color(mCenterOfBuoyancyMaterial, centerOfBuoyancyColor);
  wr_phong_material_set_transparency(mCenterOfBuoyancyMaterial, 0.7f);
  wr_material_set_default_program(mCenterOfBuoyancyMaterial, WbWrenShaders::lineSetShader());
  mCenterOfBuoyancyRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mCenterOfBuoyancyRenderable, false);
  wr_renderable_set_receive_shadows(mCenterOfBuoyancyRenderable, false);
  wr_renderable_set_mesh(mCenterOfBuoyancyRenderable, WR_MESH(mGlobalCenterOfMassMesh));
  wr_renderable_set_material(mCenterOfBuoyancyRenderable, mCenterOfBuoyancyMaterial, NULL);
  wr_renderable_set_drawing_order(mCenterOfBuoyancyRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);
  wr_renderable_set_drawing_mode(mCenterOfBuoyancyRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_visibility_flags(mCenterOfBuoyancyRenderable, WbWrenRenderingContext::VF_INVISIBLE_FROM_CAMERA);
  mCenterOfBuoyancyTransform = wr_transform_new();
  wr_node_set_visible(WR_NODE(mCenterOfBuoyancyTransform), false);
  wr_transform_attach_child(mCenterOfBuoyancyTransform, WR_NODE(mCenterOfBuoyancyRenderable));
  wr_transform_attach_child(wr_scene_get_root(wr_scene_get_instance()), WR_NODE(mCenterOfBuoyancyTransform));

  // Connects signals for further updates
  connect(WbWrenRenderingContext::instance(), &WbWrenRenderingContext::lineScaleChanged, this, &WbSolid::updateLineScale);

  updateLineScale();
  applyChangesToWren();
}

////////////////////////////
//   Create ODE Objects   //
////////////////////////////

void WbSolid::setSolidMerger() {
  if (mIsKinematic) {
    mSolidMerger = NULL;
    return;
  }

  const WbSolid *const us = jointParent() ? NULL : upperSolid();
  const bool inherit = us && us->physics() && name().compare("right wheel", Qt::CaseInsensitive) != 0 &&
                       name().compare("left wheel", Qt::CaseInsensitive) != 0;
  mSolidMerger = inherit ? us->solidMerger() : QPointer<WbSolidMerger>(new WbSolidMerger(this));
}

void WbSolid::setJointParents() {
  // TouchSensor special joint or fixed joint to static environment
  setOdeJointToUpperSolid();

  // new joints
  typedef QList<WbBasicJoint *>::const_iterator LCI;
  LCI end = mJointParents.constEnd();
  for (LCI it = mJointParents.constBegin(); it != end; ++it)
    (*it)->setJoint();
}

void WbSolid::setupSolidMerger() {
  // Detaches the solid if it was previously merged
  if (isSolidMerger()) {
    setJointParents();
    return;
  }

  if (mSolidMerger)
    mSolidMerger->removeSolid(this);

  // Sets the new solid merger
  setSolidMerger();

  dGeomID g = odeGeom();
  if (boundingObject() && g == NULL)
    createOdeGeoms();
  else if (g && mSolidMerger)
    mSolidMerger->addGeomToSpace(g);

  if (mSolidMerger) {
    assert(isDynamic());  // At this point mSolidMerger == NULL if mIsKinematic == false
    if (mOdeMass->mass == 0.0)
      createOdeMass();
    mSolidMerger->appendSolid(this);
    // Recursively assigns the WbSolid body to every non-space ODE dGeom
    g = odeGeom();
    if (g)
      mSolidMerger->attachGeomsToBody(g);
    if (mSolidMerger->isSet())
      mSolidMerger->mergeMass(this, false);
  }
}

// Recursive method that sets solid mergers, creates masses and attaches dGeoms from top to bottom
void WbSolid::setupSolidMergers() {
  setupSolidMerger();
  // Recurses through all first level solid descendants
  foreach (WbSolid *const solid, mSolidChildren)
    solid->setupSolidMergers();

  mMergerIsSet = true;
}

// Recursive method that sets children joints with referenced endpoints
void WbSolid::setJointChildrenWithReferencedEndpoint() {
  foreach (WbBasicJoint *const j, mJointChildren)
    if (j->solidReference()) {
      j->updateEndPoint();
      j->setJoint();
    }

  foreach (WbSolid *const solid, mSolidChildren)
    solid->setJointChildrenWithReferencedEndpoint();
}

void WbSolid::createOdeObjects() {
  if (boundingObject())
    boundingObject()->createOdeObjects();

  if (isTopLevel() || !mMergerIsSet) {  // the second condition is for newly inserted solids only
    setupSolidMergers();                // this recursion sets solid mergers but also creates dGeoms, dMasses
    setBodiesAndJointsToParents();      // this recursion sets bodies positions and joints to parents
    setJointChildrenWithReferencedEndpoint();
  }

  // Recurses through solid descendants
  WbPose::createOdeObjects();
}

// Sets recursively every ODE object which was not set during solid merger settings, i.e. bodies and joints to parents
void WbSolid::setBodiesAndJointsToParents() {
  assert(mMergerIsSet);
  if (isDynamic()) {
    if (isSolidMerger())
      mSolidMerger->setupOdeBody();
    const WbPhysics *const p = physics();
    connect(p, &WbPhysics::massOrDensityChanged, this, &WbSolid::updateOdeMass, Qt::UniqueConnection);
    connect(p, &WbPhysics::massOrDensityChanged, WbMassChecker::instance(), &WbMassChecker::checkMasses, Qt::UniqueConnection);
    connect(p, &WbPhysics::centerOfMassChanged, this, &WbSolid::updateOdeCenterOfMass, Qt::UniqueConnection);
    connect(p, &WbPhysics::inertialPropertiesChanged, this, &WbSolid::updateOdeInertiaMatrix, Qt::UniqueConnection);
    connect(p, &WbPhysics::dampingChanged, this, &WbSolid::updateOdeDamping, Qt::UniqueConnection);
  } else
    updateOdeGeomPosition();  // for kinematic solids

  // Recurses through solid descendants
  foreach (WbSolid *const solid, mSolidChildren)
    solid->setBodiesAndJointsToParents();

  WbBasicJoint *const pj = jointParent();
  if (pj)
    pj->updateAfterParentPhysicsChanged();  // needed also in kinematic mode

  if (isSolidMerger()) {
    // Sets joints
    setOdeJointToUpperSolid();

    // Sets 'new' joints
    if (pj)
      pj->setJoint();
  }
}

// Reset ODE joints (with no position offset) for every solid linked to this one
void WbSolid::resetJointsToLinkedSolids() {
  assert(mMergerIsSet);
  resetJointPositions(true);

  foreach (WbSolid *const solid, mSolidChildren)
    solid->resetJointPositions(true);

  foreach (WbBasicJoint *const j, mJointChildren)
    if (j->solidReference())
      j->resetJointPositions();
}

// Reset ODE joints (with no position offset) for every solid linked to this one or to one of its descendants
void WbSolid::resetJoints() {
  if (isSolidMerger())
    resetJointPositions(true);

  foreach (WbSolid *const solid, mSolidChildren)
    solid->resetJoints();
}

void WbSolid::createOdeGeoms() {
  assert(odeGeom() == NULL);
  dSpaceID space = mSolidMerger ? mSolidMerger->reservedSpace() : WbOdeContext::instance()->space();
  createOdeGeomFromBoundingObject(space);
}

void WbSolid::createOdeGeomFromInsertedGroupItem(WbBaseNode *node) {
  assert(node);

  dSpaceID space = upperSpace();
  assert(space);

  dGeomID insertedGeom = createOdeGeomFromNode(space, node);
  if (!insertedGeom)  // if the inserted node has no Geometry child or it has an indexed face set which is invalid
    return;

  if (isDynamic()) {
    assert(mSolidMerger);
    addMass(node);
    // Attaches the dGeom to merger's body and adjusts the mass of both solid and solid merger
    dGeomSetBody(insertedGeom, mSolidMerger->body());
    adjustOdeMass();
  } else
    updateOdeGeomPosition(insertedGeom);
}

// Methods modifying the mass
void WbSolid::updateTopSolidGlobalMass() const {
  WbSolid *const ts = topSolid();
  if (ts && ts->isPostFinalizedCalled()) {
    ts->updateGlobalCenterOfMass();
    ts->updateGlobalVolume();
  }
}

// Method correcting the ODE dMass of the WbSolid after insertion or deletion of a bounding WbGeometry; it is also called when
// the density and the mass field change
void WbSolid::adjustOdeMass(bool mergeMass) {
  if (mSolidMerger == NULL) {
    updateTopSolidGlobalMass();
    emit massPropertiesChanged();
    return;
  }

  const double currentMass = mReferenceMass->mass;

  updateCenterOfMass();

  if (currentMass <= MASS_ZERO_THRESHOLD) {
    dMassSetZero(mReferenceMass);  // clears possible roundoff errors (float addition/subtraction are not associative)
    if (mergeMass && physics()->mode() == WbPhysics::BOUNDING_OBJECT_BASED) {
      // makes sure ODE is fed with a (non-zero) suitable mass
      // deactivate warning when the mass is temporarily zeroed during a dictionary update
      setDefaultMassSettings(true, !WbNode::cUpdatingDictionary);
      mSolidMerger->mergeMass(this);

      // set mass displayed in the Solid's mass tab
      dMassSetParameters(mMassAroundCoM, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
      mMassAroundCoM->c[0] = mCenterOfMass.x();
      mMassAroundCoM->c[1] = mCenterOfMass.y();
      mMassAroundCoM->c[2] = mCenterOfMass.z();
      updateTopSolidGlobalMass();
      emit massPropertiesChanged();
    }
    return;
  }

  if (mUseInertiaMatrix) {  // true when the two inertia 3D-fields are filled and the CoM is also specified
    setOdeInertiaMatrix();
  } else {
    memcpy(mOdeMass, mReferenceMass, sizeof(dMass));
    const WbPhysics *const p = physics();
    const double fieldMass = p->mass();

    if (fieldMass > 0.0)
      dMassAdjust(mOdeMass, fieldMass);
    else {
      const double fieldDensity = p->density();
      if (fieldDensity >= 0.0)
        dMassAdjust(mOdeMass, (currentMass * fieldDensity) / REFERENCE_DENSITY);
    }

    memcpy(mMassAroundCoM, mOdeMass, sizeof(dMass));

    // Translate the mass to solid's origin
    if (p->centerOfMass().size() == 1) {
      dVector3 t;
      dSubtractVectors3(t, centerOfMass().ptr(), mReferenceMass->c);
      dMassTranslate(mOdeMass, t[0], t[1], t[2]);
    }
  }

  // merge mass and reset body, geoms and joints starting from and pointing to this solid
  if (mergeMass)
    mSolidMerger->mergeMass(this);

  // Adjust the global center of mass
  updateTopSolidGlobalMass();
  updateGraphicalGlobalCenterOfMass();
  emit massPropertiesChanged();
}

void WbSolid::addMassFromInsertedNode(WbBaseNode *node) {  // node is a WbGeometry or a WbPose
  assert(isDynamic() && mSolidMerger);
  addMass(node);
  adjustOdeMass();
}

void WbSolid::subtractOdeMass(const dMass *mass, bool adjustSolidMass) {
  if (mIsKinematic)
    return;

  // Modifies the Solid reference mass (based on boundingObject)
  double m1000 = mReferenceMass->mass;
  double m = mass->mass;
  const double massDifference = m1000 - m;
  if (massDifference <= MASS_ZERO_THRESHOLD)
    dMassSetZero(mReferenceMass);
  else {
    const double r = 1.0 / massDifference;
    m *= r;
    m1000 *= r;
    dAddScaledVectors3(mReferenceMass->c, mReferenceMass->c, mass->c, m1000, -m);
    WbSolidUtilities::subtractInertiaMatrix(mReferenceMass->I, mass->I);
    mReferenceMass->mass = massDifference;
  }

  // Modifies the Solid mass in use
  if (adjustSolidMass) {
    adjustOdeMass();
    awake();
  }
}

void WbSolid::correctOdeMass(const dMass *mass, WbBaseNode *node, bool adjustSolidMass) {
  if (mIsKinematic)
    return;

  subtractOdeMass(mass, false);
  addMass(node);

  if (adjustSolidMass) {  // default case
    adjustOdeMass();
    awake();
  }
}

void WbSolid::removeBoundingGeometry() {
  if (isBeingDeleted())
    return;

  const WbGeometry *const geometry = dynamic_cast<WbGeometry *>(sender());
  const dMass *const dmass = geometry->odeMass();

  if (isDynamic() && dmass->mass > 0.0) {
    // modify the mass only if it is computed by means of the boundingObject
    subtractOdeMass(dmass, physics()->mode() == WbPhysics::BOUNDING_OBJECT_BASED);
  }
}

// This is the default joint creation behavior
// this method is overridden in the WbTouchSensor class
dJointID WbSolid::createJoint(dBodyID body, dBodyID parentBody, dWorldID world) const {
  dJointID odeJoint = dJointCreateFixed(world, 0);

  setJoint(odeJoint, body, parentBody);

  return odeJoint;
}

void WbSolid::setJoint(dJointID joint, dBodyID body, dBodyID parentBody) const {
  dJointAttach(joint, parentBody, body);
  dJointSetFixed(joint);
}

void WbSolid::printKinematicWarningIfNeeded() {
  if (mKinematicWarningPrinted || !mHasDynamicSolidDescendant || !belongsToStaticBasis())
    return;

  mKinematicWarningPrinted = true;
  parsingWarn(tr("This node is controlled in kinematics mode "
                 "but some Solid descendant nodes have physics and won't move along with this node."));
}

WbVector3 WbSolid::relativeLinearVelocity(const WbSolid *parentSolid) const {
  WbVector3 l = isDynamic() ? solidMerger()->solid()->linearVelocity() : linearVelocity();

  const WbSolid *solid = this;
  // if this solid is kinematic we need to add the velocities of the parents
  if (solid->isKinematic()) {
    while (!solid->isTopSolid() && solid != parentSolid) {
      solid = solid->upperSolid();
      l = solid->rotationMatrix() * l;
      l += solid->linearVelocity();
      if (!solid->isKinematic())
        break;
    }
  } else if (parentSolid != NULL) {  // in case of dynamic solid, the velocity is already absolute
    while (!solid->isTopSolid() && solid != parentSolid)
      solid = solid->upperSolid();
    l -= solid->isDynamic() ? solid->solidMerger()->solid()->linearVelocity() : solid->linearVelocity();
  }

  assert(solid == parentSolid || parentSolid == NULL);
  return l;
}

WbVector3 WbSolid::relativeAngularVelocity(const WbSolid *parentSolid) const {
  WbVector3 a = isDynamic() ? solidMerger()->solid()->angularVelocity() : angularVelocity();

  const WbSolid *solid = this;
  // if this solid is kinematic we need to add the velocities of the parents
  if (solid->isKinematic()) {
    while (!solid->isTopSolid() && solid != parentSolid) {
      solid = solid->upperSolid();
      a = solid->rotationMatrix() * a;
      a += solid->angularVelocity();
      if (!solid->isKinematic())
        break;
    }
  } else if (parentSolid != NULL) {  // in case of dynamic solid, the velocity is already absolute
    while (!solid->isTopSolid() && solid != parentSolid)
      solid = solid->upperSolid();
    a -= solid->isDynamic() ? solid->solidMerger()->solid()->angularVelocity() : solid->angularVelocity();
  }

  assert(solid == parentSolid || parentSolid == NULL);
  return a;
}

void WbSolid::setLinearVelocity(const double velocity[3]) {
  mLinearVelocity->setValue(velocity[0], velocity[1], velocity[2]);
  if (isSolidMerger()) {
    // W3.2: a Newton-backed Solid's ODE body is disabled, so the ODE set would be lost -- route to Newton.
    if (!setNewtonBodyVel(velocity, false)) {
      dBodyID b = body();
      WbPhysicsBackendRegistry::odeBackend()->setBodyLinearVel(static_cast<WbBodyHandle>(b), velocity);
    }
  }
  printKinematicWarningIfNeeded();
}

void WbSolid::setAngularVelocity(const double velocity[3]) {
  mAngularVelocity->setValue(velocity[0], velocity[1], velocity[2]);
  if (isSolidMerger()) {
    if (!setNewtonBodyVel(velocity, true)) {
      dBodyID b = body();
      WbPhysicsBackendRegistry::odeBackend()->setBodyAngularVel(static_cast<WbBodyHandle>(b), velocity);
    }
  }
  printKinematicWarningIfNeeded();
}

void WbSolid::updateIsLinearVelocityNull() {
  mIsLinearVelocityNull = mLinearVelocity->value().isNull();
}

void WbSolid::updateIsAngularVelocityNull() {
  mIsAngularVelocityNull = mAngularVelocity->value().isNull();
}

dBodyID WbSolid::bodyMerger() const {
  assert(mIsKinematic || mSolidMerger);
  return isDynamic() ? mSolidMerger->body() : NULL;
}

dBodyID WbSolid::body() const {
  return isSolidMerger() ? mSolidMerger->body() : NULL;
}

void WbSolid::appendJointParent(WbBasicJoint *joint) {
  mJointParents.append(joint);
}

void WbSolid::removeJointParent(WbBasicJoint *joint) {
  mJointParents.removeOne(joint);
}

bool WbSolid::needJointToUpperSolid(const WbSolid *upperSolid) const {
  // create a fixed joint to the static environment only if
  // this node doesn't have any joint ancestor and any dynamic solid ancestor
  return mJointParents.isEmpty() && upperSolid->belongsToStaticBasis();
}

void WbSolid::setOdeJointToUpperSolid() {
  const WbSolid *const us = jointParent() ? NULL : upperSolid();
  const bool b = us && needJointToUpperSolid(us) && isSolidMerger() && (us->isDynamic() || us->belongsToStaticBasis());
  if (mJoint == NULL && b) {
    mJoint = createJoint(body(), us->bodyMerger(), WbOdeContext::instance()->world());
    return;
  }

  if (mJoint == NULL)
    return;

  if (b)
    // if the upper solid has no body, the solid is fixed to the static environment
    setJoint(mJoint, body(), us->bodyMerger());
  else
    // Removes the joint from simulation without destroying it
    setJoint(mJoint, NULL, NULL);
}

void WbSolid::setGeomMatter(dGeomID g, WbBaseNode *node) {
  if (mSolidMerger) {
    dGeomSetBody(g, mSolidMerger->body());
    addMassFromInsertedNode(node);
  }
}
/////////////////////
// Update Methods  //
/////////////////////

// Resets recursively ODE dGeoms positions, dBodies and joints starting from *this
void WbSolid::handleJerk() {
  jerk(false);
  if (!belongsToStaticBasis())
    awake();
  else
    WbWorld::instance()->awake();
}

void WbSolid::updateTranslation() {
  WbMatter::updateTranslation();
  syncNewtonPoseFromFields();

  if ((mGlobalCenterOfMassRepresentationIsEnabled || mSupportPolygonRepresentation) &&
      WbSimulationState::instance()->isPaused()) {
    float position[3];
    computedGlobalCenterOfMass().toFloatArray(position);
    wr_transform_set_position(mGlobalCenterOfMassTransform, position);
  }

  if (!mJointParents.isEmpty() && WbSimulationState::instance()->isPaused())
    emit positionChangedArtificially();
}

void WbSolid::updateRotation() {
  WbMatter::updateRotation();
  syncNewtonPoseFromFields();

  if ((mGlobalCenterOfMassRepresentationIsEnabled || mSupportPolygonRepresentation) &&
      WbSimulationState::instance()->isPaused()) {
    float position[3];
    computedGlobalCenterOfMass().toFloatArray(position);
    wr_transform_set_position(mGlobalCenterOfMassTransform, position);
  }

  if (!mJointParents.isEmpty() && WbSimulationState::instance()->isPaused())
    emit positionChangedArtificially();
}

// Bridge a Supervisor-driven translation/rotation write into Newton's
// body_q so a controller calling reset_episode() actually relocates
// the Newton body. Without this, ODE moves but Newton stays where it
// was, and over many resets the two state machines drift until the
// constraint solver fails.
void WbSolid::syncNewtonPoseFromFields() {
  if (mNewtonBodyIndex < 0)
    return;
  // P8.2: a static (mass=0, pinned) Newton body has no joints and never
  // moves, so there is nothing to sync -- and critically it must NOT
  // call resetJointsToDefaults() below (it's a top-level/root Solid, so
  // it would otherwise clobber the shared articulation every tick, the
  // exact mechanism of the chassis-freeze bug).
  if (mNewtonBodyIsStatic)
    return;
  WbPhysicsBackend *const raw = WbPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  WbNewtonBackend *const newton = static_cast<WbNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;
  const WbVector3 t = matrix().translation();
  const WbQuaternion q = WbRotation(rotationMatrix()).toQuaternion();
  newton->resetBodyPose(mNewtonBodyIndex, t.x(), t.y(), t.z(),
                        q.x(), q.y(), q.z(), q.w());
  // For the ROOT solid only, also reset all joint angles to defaults
  // and re-FK the chain. Doing this once per chassis reset is enough
  // -- the descendant Solids' body_q gets recomputed via FK from the
  // freshly-zeroed joint_q values.
  if (upperPose() == nullptr)
    newton->resetJointsToDefaults();
}

// Creates and updates, or destroys, the ODE dBody according to the existence of a WbPhysics node
void WbSolid::updatePhysics() {
  assert(areOdeObjectsCreated());

  if (isBeingDeleted())
    return;

  // this handles the deleted physics cases, we disable the optional renderings
  // that depend on the solid having a physics node.
  if (!mPhysics->value()) {
    if (isTopSolid())
      showSupportPolygonRepresentation(false);

    showGlobalCenterOfMassRepresentation(false);
    showCenterOfBuoyancyRepresentation(false);
  }

  bool previousKinematic = mIsKinematic;
  mIsKinematic = mPhysics->value() == NULL || mIsPermanentlyKinematic;
  if (mUseInertiaMatrix && mPhysics->value() != NULL)
    mUseInertiaMatrix = false;

  if (mSolidMerger == NULL || mSolidMerger->solid() == this) {
    mMergerIsSet = false;
    delete mSolidMerger.data();
  }

  if (mOdeMass && mPhysics->value())
    dMassSetZero(mOdeMass);  // force recomputing the ODE mass
  setupSolidMergers();
  setBodiesAndJointsToParents();
  setJointChildrenWithReferencedEndpoint();

  adjustOdeMass();
  applyMassCenterToWren();
  refreshPhysicsRepresentation();

  if (previousKinematic != mIsKinematic)
    updateDynamicSolidDescendantFlag();
}

void WbSolid::updateRadarCrossSection() {
  if (mRadarCrossSection->value() > 0.0) {
    if (!WbWorld::instance()->radarTargetSolids().contains(this))
      WbWorld::instance()->addRadarTarget(this);
  } else if (WbWorld::instance()->radarTargetSolids().contains(this))
    WbWorld::instance()->removeRadarTarget(this);
}

void WbSolid::updateRecognitionColors() {
  WbRgb segmentationColor(0.0, 0.0, 0.0);
  if (!mRecognitionColors->isEmpty()) {
    if (!WbWorld::instance()->cameraRecognitionObjects().contains(this))
      WbWorld::instance()->addCameraRecognitionObject(this);
    segmentationColor = mRecognitionColors->item(0);
  } else if (WbWorld::instance()->cameraRecognitionObjects().contains(this))
    WbWorld::instance()->removeCameraRecognitionObject(this);

  // set segmentation color in child nodes
  WbGroup::updateSegmentationColor(segmentationColor);
}

void WbSolid::updateSegmentationColor(const WbRgb &color) {
  // apply segmentation color from parent node if needed
  if (!mRecognitionColors->isEmpty())
    // this node already defines different recognitionColors
    return;

  WbGroup::updateSegmentationColor(color);
}

void WbSolid::updateOdeMass() {
  assert(isDynamic());

  const WbPhysics *const p = physics();
  if (!p->hasApositiveMassOrDensity())
    return;

  p->checkMassAndDensity();

  if (mUseInertiaMatrix && p->mode() != WbPhysics::CUSTOM_INERTIA_MATRIX)
    applyToOdeMass();  // the inertia computation mode has changed
  else
    adjustOdeMass();

  awake();
  refreshPhysicsRepresentation();
}

void WbSolid::setOdeInertiaMatrix() {
  assert(isDynamic() && physics()->mode() == WbPhysics::CUSTOM_INERTIA_MATRIX);
  const WbPhysics *const p = physics();
  mUseInertiaMatrix = true;
  const WbMFVector3 &inertia = p->inertiaMatrix();
  const WbVector3 &v0 = inertia.item(0);
  const WbVector3 &v1 = inertia.item(1);
  dMassSetParameters(mOdeMass, p->mass(), 0.0, 0.0, 0.0, v0.x(), v0.y(), v0.z(), v1.x(), v1.y(), v1.z());

  memcpy(mMassAroundCoM, mOdeMass, sizeof(dMass));
  updateCenterOfMass();
  const WbVector3 &com = centerOfMass();
  dMassTranslate(mOdeMass, com.x(), com.y(), com.z());  // translates inertia matrix to solid frame's origin
  emit massPropertiesChanged();
}

void WbSolid::updateOdeInertiaMatrix() {
  assert(isDynamic() && physics()->mode() == WbPhysics::CUSTOM_INERTIA_MATRIX);

  setOdeInertiaMatrix();
  mSolidMerger->mergeMass(this);
  awake();

  refreshPhysicsRepresentation();
}

void WbSolid::setInertiaMatrixFromBoundingObject() {
  assert(isDynamic() && hasAvalidBoundingObject() && physics()->hasApositiveMassOrDensity());

  // Uses the density or the mass of the Physics node together with the geometries
  // in the boundingObject to compute the inertia matrix of the solid.

  dMass dmass;
  dMassSetZero(&dmass);

  // Adds the masses of all the primitives lying in the bounding object
  WbSolidUtilities::addMass(&dmass, boundingObject(), REFERENCE_DENSITY);
  memcpy(mReferenceMass, &dmass, sizeof(dMass));

  // Computes the inertia matrix around the center of mass of the bounding object
  const dReal *const com = mReferenceMass->c;
  dMassTranslate(&dmass, -com[0], -com[1], -com[2]);

  const WbField *const parameter = findField("physics", true)->parameter();
  WbPhysics *const p = parameter ? static_cast<WbPhysics *>(static_cast<WbSFNode *>(parameter->value())->value()) : physics();

  // Sets the actual total mass to mReferenceMass
  double boundingObjectMass = mReferenceMass->mass;
  if (p->mass() > 0.0)
    boundingObjectMass = p->mass();
  else {
    boundingObjectMass *= p->density() / REFERENCE_DENSITY;
    p->setMass(boundingObjectMass, true);
    p->parsingInfo(tr("'mass' set as bounding object's mass based on 'density'."));
  }

  // Adjust the total according to mass and density fields
  dMassAdjust(mReferenceMass, boundingObjectMass);

  p->setDensity(-1.0, true);

  const double *const I = mReferenceMass->I;
  p->setInertiaMatrix(I[0], I[5], I[10], I[1], I[2], I[6], true);
  p->checkInertiaMatrix(false);

  const double *const c = mReferenceMass->c;
  p->setCenterOfMass(c[0], c[1], c[2], true);
  p->parsingInfo(tr("Bounding object's center of mass inserted."));

  p->updateMode();
  if (parameter)
    physics()->updateMode();

  updateOdeInertiaMatrix();
}

void WbSolid::updateOdeCenterOfMass() {
  assert(isDynamic() && mSolidMerger);
  updateCenterOfMass();

  applyToOdeMass();
  mSolidMerger->setGeomAndBodyPositions(true);  // reset also joints passing through this solid
  awake();

  refreshPhysicsRepresentation();
}

void WbSolid::updateOdeDamping() {
  assert(isDynamic() && mSolidMerger);
  mSolidMerger->setOdeDamping();
  awake();
}

void WbSolid::updateBoundingObject() {
  if (mBoundingObject->value() != NULL) {
    WbBaseNode *node = dynamic_cast<WbBaseNode *>(mBoundingObject->value());
    assert(node);
    if (!isBoundingObjectFinalizationCompleted(node))
      // postpone bounding object update after finalization
      return;

    createOdeGeoms();
    updateOdeGeomPosition();
    dGeomID g = odeGeom();
    if (g && mSolidMerger) {
      mSolidMerger->attachGeomsToBody(g);
      createOdeMass(false);
      mSolidMerger->mergeMass(this);
    }
  }

  mBoundingObjectHasChanged = true;
  refreshPhysicsRepresentation();
}

// Updates of children nodes

void WbSolid::collectSolidChildren(const WbGroup *group, bool connectSignals, QVector<WbSolid *> &solidChildren,
                                   QVector<WbBasicJoint *> &jointChildren, QVector<WbPropeller *> &propellerChildren) {
  const WbMFNode *const ch = group->childrenField();
  if (connectSignals) {
    connect(ch, &WbMFNode::changed, this, &WbSolid::updateChildren, Qt::UniqueConnection);
    connect(group, &WbGroup::finalizedChildAdded, this, &WbSolid::refreshPhysicsRepresentation, Qt::UniqueConnection);
    connect(group, &WbGroup::finalizedChildAdded, this, &WbSolid::updateTopSolidGlobalMass, Qt::UniqueConnection);
  }
  WbMFNode::Iterator it(ch);
  while (it.hasNext()) {
    WbNode *const n = it.next();

    // cppcheck-suppress constVariablePointer
    WbSolid *const solid = dynamic_cast<WbSolid *>(n);
    if (solid) {
      solidChildren.append(solid);
      continue;
    }

    // cppcheck-suppress constVariablePointer
    WbBasicJoint *j = dynamic_cast<WbBasicJoint *>(n);
    if (j) {
      jointChildren.append(j);
      // cppcheck-suppress constVariablePointer
      WbSolid *const ep = j->solidEndPoint();
      if (ep && j->solidReference() == NULL) {
        solidChildren.append(ep);
        continue;
      }
    }

    // cppcheck-suppress constVariablePointer
    WbPropeller *propeller = dynamic_cast<WbPropeller *>(n);
    if (propeller) {
      propellerChildren.append(propeller);
      continue;
    }

    const WbGroup *const groupChild = dynamic_cast<WbGroup *>(n);
    if (groupChild) {
      collectSolidChildren(groupChild, connectSignals, solidChildren, jointChildren, propellerChildren);
      continue;
    }

    const WbSlot *slot = dynamic_cast<WbSlot *>(n);
    if (slot) {
      if (slot->hasEndPoint()) {
        WbSlot *sep = slot->slotEndPoint();
        while (sep) {
          slot = sep;
          sep = slot->slotEndPoint();
        }
        if (slot->solidEndPoint())
          solidChildren.append(slot->solidEndPoint());
        else if (slot->groupEndPoint())
          collectSolidChildren(slot->groupEndPoint(), connectSignals, solidChildren, jointChildren, propellerChildren);
        else {
          j = dynamic_cast<WbBasicJoint *>(slot->endPoint());
          if (j) {
            jointChildren.append(j);
            // cppcheck-suppress constVariablePointer
            WbSolid *const ep = j->solidEndPoint();
            if (ep && j->solidReference() == NULL) {
              solidChildren.append(ep);
              continue;
            }
          }

          propeller = dynamic_cast<WbPropeller *>(slot->endPoint());
          if (propeller) {
            propellerChildren.append(propeller);
            continue;
          }
        }
      }
    }
  }

  if (isPostFinalizedCalled())
    updateDynamicSolidDescendantFlag();
}

void WbSolid::updateDynamicSolidDescendantFlag() {
  mHasDynamicSolidDescendant = false;
  foreach (const WbSolid *s, mSolidChildren) {
    if (!s->isPostFinalizedCalled())
      // postpone flag update after finalization
      return;

    if (s->isDynamic() || s->mHasDynamicSolidDescendant) {
      mHasDynamicSolidDescendant = true;
      break;
    }
  }

  WbSolid *us = upperSolid();
  if (us && us->isPostFinalizedCalled())
    us->updateDynamicSolidDescendantFlag();
}

void WbSolid::updateChildrenAfterJointEndPointChange(WbBaseNode *node) {
  if (node)
    updateChildren();
}

void WbSolid::updateChildren() {
  mSolidChildren.clear();
  mJointChildren.clear();
  mPropellerChildren.clear();
  collectSolidChildren(this, true, mSolidChildren, mJointChildren, mPropellerChildren);

  foreach (WbSolid *const solid, mSolidChildren) {
    connect(solid, &WbSolid::destroyed, this, &WbSolid::updateChildren, Qt::UniqueConnection);
    connect(solid, &WbSolid::destroyed, this, &WbSolid::refreshPhysicsRepresentation, Qt::UniqueConnection);
    connect(solid, &WbSolid::physicsPropertiesChanged, this, &WbSolid::refreshPhysicsRepresentation, Qt::UniqueConnection);
  }
  foreach (WbBasicJoint *const jointChild, mJointChildren)
    connect(jointChild, &WbBasicJoint::endPointChanged, this, &WbSolid::updateChildrenAfterJointEndPointChange,
            Qt::UniqueConnection);
}

bool WbSolid::resetJointPositions(bool allParents) {
  bool b = false;

  setOdeJointToUpperSolid();

  foreach (WbBasicJoint *const j, mJointParents) {
    if (allParents || j->upperSolid()->belongsToStaticBasis())
      b |= j->resetJointPositions();
  }

  return b;
}

void WbSolid::updateGlobalCenterOfMass() {
  mGlobalCenterOfMass.setXyz(0.0, 0.0, 0.0);
  mGlobalMass = 0.0;
  foreach (WbSolid *const solid, mSolidChildren) {
    if (!solid->isPreFinalizedCalled())
      // skip until finalization is completed
      // it could happen in particular in case of multiple instances of PROTO parameter node
      return;

    solid->updateGlobalCenterOfMass();
    const double childGlobalMass = solid->globalMass();
    mGlobalMass += childGlobalMass;
    mGlobalCenterOfMass += childGlobalMass * solid->globalCenterOfMass();
  }

  if (isDynamic()) {
    mGlobalCenterOfMass += mOdeMass->mass * (matrix() * centerOfMass());
    mGlobalMass += mOdeMass->mass;
  }

  if (mGlobalMass > 0.0)
    mGlobalCenterOfMass /= mGlobalMass;
  else
    mGlobalCenterOfMass = position();
}

void WbSolid::updateCenterOfBuoyancy() {
  assert(isDynamic());

  const int size = mListOfImmersions.size();
  double immersedMass = 0.0;
  mCenterOfBuoyancy.setXyz(0.0, 0.0, 0.0);
  for (int i = 0; i < size; ++i) {
    const dImmersionGeom &ig = mListOfImmersions.at(i);
    const double immersedVolume = ig.volume;
    const double *const cob = ig.buoyancyCenter;
    const dReal fluidDensity = dFluidGetDensity(dGeomGetFluid(ig.g2));
    const dReal fluidMass = fluidDensity * immersedVolume;
    mCenterOfBuoyancy += fluidMass * WbVector3(cob[0], cob[1], cob[2]);
    immersedMass += fluidMass;
  }

  if (immersedMass > 0.0)
    mCenterOfBuoyancy /= immersedMass;
}

double WbSolid::averageDensity() const {
  return mGlobalVolume > 0.0 ? mGlobalMass / mGlobalVolume : -1.0;
}

void WbSolid::updateGlobalVolume() {
  double cumulativeVolume = 0.0;

  foreach (WbSolid *const solid, mSolidChildren) {
    if (!solid->isPreFinalizedCalled())
      // skip until finalization is completed
      // it could happen in particular in case of multiple instances of PROTO parameter node
      return;
    solid->updateGlobalVolume();
    cumulativeVolume += solid->globalVolume();
  }

  mGlobalVolume = cumulativeVolume + 0.001 * mReferenceMass->mass;
}

void WbSolid::updateCenterOfMass() {
  assert(isDynamic());

  WbPhysics *const p = physics();
  p->updateMode();
  const int mode = p->mode();

  mCenterOfMass.setXyz(0.0, 0.0, 0.0);

  switch (mode) {
    case WbPhysics::CUSTOM_INERTIA_MATRIX:
      mCenterOfMass = p->centerOfMass().item(0);
      break;
    case WbPhysics::BOUNDING_OBJECT_BASED: {
      if (p->centerOfMass().size() == 1) {
        mCenterOfMass = p->centerOfMass().item(0);
      } else if (mBoundingObject->value() != NULL)
        mCenterOfMass.setXyz(mReferenceMass->c[0], mReferenceMass->c[1], mReferenceMass->c[2]);
      break;
    }
    default:
      assert(mode == WbPhysics::INVALID);
  }

  applyMassCenterToWren();
}

////////////////////
// Apply Methods  //
////////////////////

// Apply to WREN

void WbSolid::applyMassCenterToWren() {
  if (!areWrenObjectsInitialized())
    return;

  if (mIsKinematic) {
    wr_node_set_visible(WR_NODE(mCenterOfMassTransform), false);
    return;
  }

  // if the CoM is (0, 0, 0), it coincides with the axes center, so we hide it in this case
  const bool massCenterVisible = isSelected() && !mCenterOfMass.isNull();
  if (massCenterVisible) {
    float position[3];
    mCenterOfMass.toFloatArray(position);

    wr_transform_set_position(mCenterOfMassTransform, position);
    wr_node_set_visible(WR_NODE(mCenterOfMassTransform), true);
  } else
    wr_node_set_visible(WR_NODE(mCenterOfMassTransform), false);
}

void WbSolid::updateLineScale() {
  WbMatter::updateLineScale();

  const float lineScale = wr_config_get_line_scale() * WbWrenRenderingContext::SOLID_LINE_SCALE_FACTOR;
  const float scale[3] = {lineScale, lineScale, lineScale};

  wr_transform_set_scale(mCenterOfBuoyancyTransform, scale);
  wr_transform_set_scale(mGlobalCenterOfMassTransform, scale);
  wr_transform_set_scale(mCenterOfMassTransform, scale);

  if (mSupportPolygonRepresentation)
    mSupportPolygonRepresentation->setScale(scale);
}

void WbSolid::applyChangesToWren() {
  WbMatter::applyChangesToWren();
  applyMassCenterToWren();
  refreshPhysicsRepresentation();
}

void WbSolid::applyVisibilityFlagsToWren(bool selected) {
  WbMatter::applyVisibilityFlagsToWren(selected);

  if (isDynamic() && !centerOfMass().isNull())
    wr_node_set_visible(WR_NODE(mCenterOfMassTransform), selected);
}

void WbSolid::setDefaultMassSettings(bool applyCenterOfMassTranslation, bool warning) {
  const double fieldMass = physics()->mass();
  if (fieldMass > 0.0) {
    dMassSetParameters(mOdeMass, fieldMass, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    if (warning)
      parsingWarn(
        tr("Undefined inertia matrix: using the identity matrix. Please specify 'boundingObject' or 'inertiaMatrix' values."));
  } else {
    dMassSetParameters(mOdeMass, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    if (warning) {
      if (physics()->density() > 0.0)
        parsingWarn(
          tr("Mass is invalid because 'boundingObject' is not defined. Using default mass properties: mass = 1, inertia "
             "matrix = identity"));
      else
        parsingWarn(
          tr("Mass is invalid: %1. Using default mass properties: mass = 1, inertia matrix = identity").arg(fieldMass));
    }
  }

  if (applyCenterOfMassTranslation)
    dMassTranslate(mOdeMass, mCenterOfMass.x(), mCenterOfMass.y(), mCenterOfMass.z());
}

// Compute the mass and the inertia around solid frame's origin
void WbSolid::createOdeMass(bool reset) {
  assert(isDynamic());

  if (reset) {
    dMassSetZero(mReferenceMass);
    dMassSetZero(mOdeMass);
  }

  // Adds the masses of all the primitives lying in the bounding object
  const WbPhysics *const p = physics();
  const bool customMass = p->mode() == WbPhysics::CUSTOM_INERTIA_MATRIX;
  // needed for average density and average damping
  WbSolidUtilities::addMass(mReferenceMass, boundingObject(), REFERENCE_DENSITY, !customMass);

  // Checks whether there is a valid inertia matrix, and uses it if so
  if (customMass)
    setOdeInertiaMatrix();
  else {
    mUseInertiaMatrix = false;
    // We rule out the case of a boundingObject with no Geometry inside
    if (mReferenceMass->mass <= 0.0)
      setDefaultMassSettings(false);
    else {
      // Uses the density or the mass of the Physics node together with the geometries
      // in the boundingObject to compute the inertia matrix of the solid.
      memcpy(mOdeMass, mReferenceMass, sizeof(dMass));
    }

    updateCenterOfMass();

    assert(mOdeMass->mass > 0.0);

    const double fieldDensity = p->density();
    const double fieldMass = p->mass();

    // Sets the actual total mass
    double actualMass;
    if (fieldMass > 0.0)
      actualMass = fieldMass;
    else if (fieldDensity != REFERENCE_DENSITY)
      actualMass = mOdeMass->mass * fieldDensity / REFERENCE_DENSITY;
    else
      actualMass = mOdeMass->mass;

    // Adjust the total according to mass and density fields
    dMassAdjust(mOdeMass, actualMass);

    memcpy(mMassAroundCoM, mOdeMass, sizeof(dMass));

    // Translate the mass to solid's origin
    if (p->centerOfMass().size() == 1) {
      dVector3 t;
      dSubtractVectors3(t, centerOfMass().ptr(), mReferenceMass->c);
      dMassTranslate(mOdeMass, t[0], t[1], t[2]);
    }

    updateTopSolidGlobalMass();
    emit massPropertiesChanged();
  }
}

double WbSolid::mass() const {
  return mOdeMass->mass;
}

double WbSolid::density() const {
  const double d = isDynamic() ? physics()->density() : -1.0;
  const double v = volume();
  return d >= 0.0 ? d : v > 0.0 ? mOdeMass->mass / v : -1.0;
}

double WbSolid::volume() const {
  return 0.001 * mReferenceMass->mass;
}

const double *WbSolid::inertiaMatrix() const {
  return mMassAroundCoM->I;
}

void WbSolid::applyToOdeMass() {
  assert(isDynamic() && mSolidMerger);
  const WbPhysics *const p = physics();
  if (!p->hasApositiveMassOrDensity())
    return;

  createOdeMass();
  mSolidMerger->mergeMass(this);
}

void WbSolid::updateTransformForPhysicsStep() {
  if (mUpdatedInStep)
    return;

  applyPhysicsTransform();

  QList<WbSolid *> reversedList;
  reversedList << this;
  WbSolid *s = NULL;
  WbNode *p = parentNode();
  while (p != NULL && !p->isWorldRoot()) {
    s = dynamic_cast<WbSolid *>(p);
    if (s != NULL) {
      if (s->mUpdatedInStep)
        break;  // ancestor nodes already updated
      reversedList.prepend(s);
    }
    p = p->parentNode();
  }

  // update transform from root to current node as applyPhysicsTransform uses the upper transform matrix
  QListIterator<WbSolid *> it(reversedList);
  while (it.hasNext()) {
    s = it.next();
    s->applyPhysicsTransform();
    s->mUpdatedInStep = true;
  }
}

void WbSolid::applyPhysicsTransform() {
  dVector3 result;  // VRML translation
  dQuaternion qr;   // VRML rotation

  // get current body rotation
  dBodyID b = body();
  if (!b)
    return;
  dBodyCopyQuaternion(b, qr);

  // update linear and angular velocity
  if (mLinearVelocity && mAngularVelocity) {
    double l[3], a[3];
    WbPhysicsBackend *const odeBackend = WbPhysicsBackendRegistry::odeBackend();
    odeBackend->getBodyLinearVel(static_cast<WbBodyHandle>(b), l);
    odeBackend->getBodyAngularVel(static_cast<WbBodyHandle>(b), a);
    mLinearVelocity->setValue(l[0], l[1], l[2]);
    mAngularVelocity->setValue(a[0], a[1], a[2]);
  }

  // find Solid merger's frame center in world coordinates
  const WbVector3 &com = mSolidMerger->centerOfMass();
  if (com.isNull())
    dBodyCopyPosition(b, result);
  else
    // Solid center != com in this case
    dBodyGetRelPointPos(b, -com.x(), -com.y(), -com.z(), result);
  // RL training in ODE can occasionally diverge to NaN on extreme actions /
  // joint-limit violations. Asserting blocks the whole simulator on a
  // Windows MSVCRT dialog -- in --batch --mode=fast headless training that
  // looks like "controller closed connection" to the trainer. Skip the
  // pose writeback on NaN so the sim stays alive and the trainer's
  // termination logic gets its episode-end on the next step.
  if (std::isnan(result[0]) || std::isnan(result[1]) || std::isnan(result[2]))
    return;
  // printf("new body pos = %f, %f, %f (apply phy.)\n", result[0], result[1], result[2]);
  const WbPose *const up = upperPose();
  if (up) {
    const WbMatrix4 &upm = up->matrix();
    const WbVector3 &prel = upm.pseudoInversed(WbVector3(result));
    result[0] = prel[0];
    result[1] = prel[1];
    result[2] = prel[2];
    // printf("result = %f, %f, %f (apply phy.))\n", result[0], result[1], result[2]);
    // find rotation difference between upper pose and solid child.
    // P1.5 widening: bookend quaternion read flows through dispatcher;
    // dQMultiply stays as local ODE math on a staged dQuaternion (same
    // pattern as WbConnector::rotateBodies).
    const WbQuaternion &q = upm.extractedQuaternion();
    double qBody[4];
    WbPhysicsBackendRegistry::odeBackend()->getBodyQuaternion(static_cast<WbBodyHandle>(b), qBody);
    const dQuaternion qOde = {static_cast<dReal>(qBody[0]), static_cast<dReal>(qBody[1]),
                              static_cast<dReal>(qBody[2]), static_cast<dReal>(qBody[3])};
    dQMultiply1(qr, q.ptr(), qOde);
  }

  const double norm = sqrt(qr[1] * qr[1] + qr[2] * qr[2] + qr[3] * qr[3]);
  if (std::isnan(qr[0]) || std::isnan(norm) || norm == 0.0) {
    setTransformFromOde(result[0], result[1], result[2], 0.0, 0.0, 1.0, 0.0);
    return;
  }

  double angle = 2.0 * atan2(norm, qr[0]);  // in the range [-2 * M_PI, 2 * M_PI]
  if (angle < -M_PI)
    angle += 2.0 * M_PI;
  else if (angle > M_PI)
    angle -= 2.0 * M_PI;

  const double normInv = 1.0 / norm;
  qr[1] *= normInv;
  qr[2] *= normInv;
  qr[3] *= normInv;

  // block signals from WbPose (baseclass): we don't want to update the bodies and the geoms
  // printf("pos = %f, %f, %f\n", result[0], result[1], result[2]);
  setTransformFromOde(result[0], result[1], result[2], qr[1], qr[2], qr[3], angle);
}

//////////////////
// Run Methods  //
//////////////////

// OMNISIM_PROBE_TRAJ root classifier (newton-ode-replacement-plan.md W5): a ROOT is tagged 'A' when it is the
// controlled mechanism -- a WbRobot (panda, husky, spot, a battlebot) -- and 'R' otherwise (a free prop or a
// static structure). The faithful meter splits on this because a loose prop that rolls or settles to a
// different-but-equally-valid rest pose under a different solver is a fidelity FACT (untunable -- more
// substeps EJECT it), not a Newton bug, and reporting only the worst root lets one rolling paint bucket mask
// that the ROBOT tracks ODE exactly. NOTE: "has a joint" is the WRONG proxy -- an articulated PROP (a paint
// bucket with a hinged handle) has a joint yet is not a robot; WbRobot is the semantically correct test.
void WbSolid::postPhysicsStep() {
  // OMNISIM_PROBE_TRAJ (faithful-match meter, newton-ode-replacement-plan.md W5.4): per-step world position
  // of each articulation ROOT, so faithful_check.py can compare a world's Newton vs forced-ODE trajectory
  // (turning the coverage meter's "eligible %" into "faithful %"). Dumped at the same point under both
  // solvers -- the pose here is one physics step behind under BOTH, so the lag cancels in the diff. Exits
  // after OMNISIM_PROBE_TRAJ_MS (default 1000) of sim time. Inert unless the env var is set. The trailing
  // A/R tag (W5) marks articulated robot roots vs rigid free-prop/static roots so the meter can split them.
  if (qEnvironmentVariableIsSet("OMNISIM_PROBE_TRAJ")) {
    bool isRoot = true;
    for (WbNode *n = parentNode(); n != nullptr; n = n->parentNode())
      if (dynamic_cast<const WbSolid *>(n) != nullptr) {
        isRoot = false;
        break;
      }
    // OMNISIM_PROBE_TRAJ_ALL (W6 diagnosis): dump EVERY solid, not just articulation roots, so a robot's
    // per-link / foot positions can be compared Newton-vs-ODE (e.g. localize Spot's leg collapse: feet on
    // the floor + low body => legs not holding the pose; feet below the floor => contact penetration).
    if (isRoot || qEnvironmentVariableIsSet("OMNISIM_PROBE_TRAJ_ALL")) {
      const double tms = WbSimulationState::instance() ? WbSimulationState::instance()->time() : 0.0;
      bool ok = false;
      const double parsed = qEnvironmentVariable("OMNISIM_PROBE_TRAJ_MS").toDouble(&ok);
      if (tms > (ok ? parsed : 1000.0))
        std::_Exit(0);  // first root past the budget exits; every step up to it is fully dumped
      const WbVector3 p = matrix().translation();
      const char tag = (dynamic_cast<const WbRobot *>(this) != nullptr) ? 'A' : 'R';
      QFile tf(qEnvironmentVariable("OMNISIM_PROBE_TRAJ"));
      if (tf.open(QIODevice::Append | QIODevice::Text))
        tf.write(QString("%1\t%2\t%3\t%4\t%5\t%6\n")
                     .arg(tms, 0, 'f', 1)
                     .arg(name())
                     .arg(p.x(), 0, 'f', 6)
                     .arg(p.y(), 0, 'f', 6)
                     .arg(p.z(), 0, 'f', 6)
                     .arg(tag)
                     .toUtf8());
    }
  }
  int i = 0;
  dBodyID odeBody = this->body();

  if (mResetPhysicsInStep) {
    // physics reset from Supervisor: if the solid is also moved from Supervisor in the same step, ODE may overwrite velocities
    // and forces based on the jerk
    resetSingleSolidPhysics();
    if (mSolidMerger)
      mSolidMerger->setBodyArtificiallyDisabled(false);
    mResetPhysicsInStep = false;
  }

  // P3.10f: skip ODE pose readback for non-Newton-registered Solids
  // that descend from a Newton-backed ancestor (e.g. URDF visual-only
  // children like top_chassis, bumpers, top_plate_link inside a
  // newton_husky Robot).
  //
  // Why: ODE is still simulating these bodies independently because we
  // don't suppress ODE for Newton-backed solids. ODE's chassis position
  // drifts from Newton's chassis position over time -- ODE has the real
  // husky chassis approximately stationary while Newton drives it
  // forward. applyPhysicsTransform then writes a parent-local pose
  // computed against ODE's chassis, so the visual children "lag" the
  // Newton-driven chassis (looks like the body mesh is shifted off the
  // wheels). For these children the URDF-defined static offset is the
  // correct relative pose; leaving translation untouched lets the scene
  // tree carry the parent's Newton-driven transform onto them at render
  // time, which is what the user expects to see.
  bool hasNewtonBackedAncestor = false;
  if (mNewtonBodyIndex < 0) {
    for (WbNode *n = parentNode(); n != nullptr; n = n->parentNode()) {
      if (WbSolid *p = dynamic_cast<WbSolid *>(n)) {
        if (p->mNewtonBodyIndex >= 0) {
          hasNewtonBackedAncestor = true;
          break;
        }
      }
    }
  }

  if (odeBody && WbPhysicsBackendRegistry::odeBackend()->isBodyEnabled(static_cast<WbBodyHandle>(odeBody)) == 1 &&
      !hasNewtonBackedAncestor)
    applyPhysicsTransform();

  // P3.2 of cuda-newton-physics-plan.md: read this Solid's pose back
  // from the Newton solver. Runs after ODE's applyPhysicsTransform so
  // -- if there's also an ODE body (Newton bodies that haven't yet
  // suppressed ODE creation) -- Newton's value is the one that ends up
  // in mTranslation. P3.2 deliberately doesn't suppress ODE for
  // Newton bodies; the cross-backend bridge work in P3.3 is what makes
  // ODE's role for Newton-backed solids well-defined. For now Newton's
  // pose just wins the per-step write race, which is enough to verify
  // the readback path end-to-end.
  if (mNewtonBodyIndex >= 0 && !mNewtonBodyIsStatic) {
    // P3.10k: use the registry's Newton backend (not the local
    // physicsBackend() field). When physicsBackend is set on the
    // outer URDFRobot wrapper and *inherited* by leg Solids via
    // effectivePhysicsBackendName, the leg's own physicsBackend
    // field is still default "ode" -- the previous local-only
    // check then skipped the Newton overwrite for every leg, ODE's
    // applyPhysicsTransform wrote junk poses, and the visual meshes
    // flew off the chassis. (Newton-registered Solids are a guaranteed
    // sign that the registry has a working Newton backend, so the
    // availability check below is mostly defensive.)
    WbPhysicsBackend *backend = WbPhysicsBackendRegistry::newtonBackend();
    if (backend != nullptr && backend->isAvailable()) {
      WbNewtonBackend *newton = static_cast<WbNewtonBackend *>(backend);
      double xform[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
      if (newton->isWorldRunning() && newton->getBodyXform(mNewtonBodyIndex, xform) == 0) {
        const WbVector3 worldPos(xform[0], xform[1], xform[2]);
        // Newton's quaternion order is (qx, qy, qz, qw); WbQuaternion takes (w, x, y, z).
        const WbQuaternion worldQuat(xform[6], xform[3], xform[4], xform[5]);

        // Project Newton's world pose into the immediate upperPose's
        // local frame -- the upperPose() is the parent in the SCENE
        // TREE (typically a WbBasicJoint, not the parent Solid). Its
        // matrix() includes the joint's rotation. ODE's
        // applyPhysicsTransform does exactly this projection; if we
        // use solidParent() (skipping the joint), the rendering then
        // applies the joint rotation on top and the visual mesh drifts
        // off the body. That was the "body parts falling off" bug --
        // joint constraint satisfied in Newton, but the rendered pose
        // had the joint rotation stacked twice.
        // P3.10n: use setValueFromOde (emits `changedByOde`, NOT
        // `changed`). setValue would fire WbPose::updateTranslation,
        // and our Newton-aware reset hook listens on updateTranslation
        // -- the resulting feedback loop reset joint_q every step and
        // froze Spot at spawn pose. Mirroring ODE's setTransformFromOde
        // sidesteps the slot entirely. We DO still need to manually
        // invalidate the matrix cache because the slot's other job
        // (setMatrixNeedUpdate) skipped with it; supervisor.getPosition
        // reads a cached world matrix and was reporting stale spawn
        // pose otherwise.
        const WbPose *const up = upperPose();
        if (up == nullptr) {
          mTranslation->setValueFromOde(worldPos);
          mRotation->setValueFromOde(WbRotation(worldQuat));
        } else {
          const WbMatrix4 &upm = up->matrix();
          const WbVector3 localPos = upm.pseudoInversed(worldPos);
          mTranslation->setValueFromOde(localPos);
          // Local rotation = inv(upper_rot) * world_rot.
          const WbQuaternion upQ = upm.extractedQuaternion();
          const WbQuaternion localQ = upQ.conjugated() * worldQuat;
          mRotation->setValueFromOde(WbRotation(localQ));
        }
        setMatrixNeedUpdate();
        // CRITICAL: push the updated translation/rotation to the WREN visual
        // transform. setValueFromOde() above bypasses the `changed` signal
        // (using `changedByOde` to dodge our Newton reset hook), and the
        // `changed` signal is what normally triggers applyTranslationToWren
        // / applyRotationToWren. Without these explicit calls the data
        // layer moves (field + matrix cache + getPosition all advance) but
        // the GPU mesh stays at spawn -- the rendered chassis appears
        // STATIONARY while physics is walking it forward. Verified: with
        // this missing, x>200 in the Scene Tree while the visual robot
        // doesn't budge. Leg joints already refresh WREN via their own
        // joint-node path, which is why you see legs cycling in place.
        applyTranslationToWren();
        applyRotationToWren();
        // Populate the world-frame velocity fields from Newton so the
        // supervisor getVelocity() returns the real velocity (it was 0
        // under Newton because only ODE ever filled these). Newton
        // body_qd = [vx,vy,vz, wx,wy,wz] in WORLD frame, exactly the order
        // getVelocity() expects. The RL policy is a velocity-feedback
        // balancer, so this is what lets a Newton-deployed (and
        // in-OmniSim-trained) policy actually walk instead of going blind.
        double bvel[6] = {0, 0, 0, 0, 0, 0};
        if (mLinearVelocity && mAngularVelocity &&
            newton->getBodyVelocity(mNewtonBodyIndex, bvel) == 0) {
          mLinearVelocity->setValue(bvel[0], bvel[1], bvel[2]);
          mAngularVelocity->setValue(bvel[3], bvel[4], bvel[5]);
        }
      }
    }
  }

  // Warning: do not use foreach here => Qt foreach loop are very inefficient here
  for (i = 0; i < mJointChildren.size(); ++i)
    if (mJointChildren.at(i)->isEnabled())
      mJointChildren.at(i)->postPhysicsStep();

  for (i = 0; i < mSolidChildren.size(); ++i)
    mSolidChildren.at(i)->postPhysicsStep();
}

void WbSolid::prePhysicsStep(double ms) {
  int i = 0;

  if (handleJerkIfNeeded())
    mMovedChildren.clear();
  else if (!mMovedChildren.isEmpty())
    childrenJerk();

  if (mIsKinematic && robot())
    updateOdeGeomPosition();

  // Update position in kinematic mode
  if (mIsKinematic && mLinearVelocity && !mIsLinearVelocityNull)
    translate(mLinearVelocity->value() * (ms / 1000));
  // Update orientation in kinematic mode
  if (mIsKinematic && mAngularVelocity && !mIsAngularVelocityNull)
    rotate(mAngularVelocity->value() * (ms / 1000));

  // Warning: do not use foreach here => Qt foreach loop are very inefficient here
  for (i = 0; i < mJointChildren.size(); ++i)
    if (mJointChildren.at(i)->solidEndPoint())
      mJointChildren.at(i)->prePhysicsStep(ms);

  for (i = 0; i < mSolidChildren.size(); ++i)
    mSolidChildren.at(i)->prePhysicsStep(ms);

  for (i = 0; i < mPropellerChildren.size(); ++i)
    mPropellerChildren.at(i)->prePhysicsStep(ms);

  mUpdatedInStep = false;
}

////////////
// Others //
////////////

// Accessors to relatives

WbBasicJoint *WbSolid::jointParent() const {
  const WbSlot *parentSlot = dynamic_cast<WbSlot *>(parentNode());
  if (parentSlot) {
    const WbSlot *granParentSlot = dynamic_cast<WbSlot *>(parentSlot->parentNode());
    if (granParentSlot)
      return dynamic_cast<WbBasicJoint *>(granParentSlot->parentNode());
  }
  return dynamic_cast<WbBasicJoint *>(parentNode());
}

dBodyID WbSolid::upperSolidBody() const {
  return upperSolid()->bodyMerger();
}

WbSolid *WbSolid::findSolid(const QString &name, const WbSolid *const exception) {
  if (this->name() == name && this != exception)
    return this;

  foreach (WbSolid *const solid, mSolidChildren) {
    WbSolid *const s = solid->findSolid(name, exception);
    if (s && s != exception)
      return s;
  }

  return NULL;
}

WbRobot *WbSolid::robot() const {
  if (!mHasSearchedRobot) {
    mRobot = WbNodeUtilities::findRobotAncestor(this);
    mHasSearchedRobot = true;
  }

  return mRobot;
}

// Returns true if all solid ancestors have no physics
bool WbSolid::belongsToStaticBasis() const {
  const WbSolid *s = this;

  while (s) {
    if (s->isDynamic())
      return false;
    s = s->upperSolid();
  }

  return true;
}

WbPhysics *WbSolid::physics() const {
  return static_cast<WbPhysics *>(mPhysics->value());
}

// Resolves the physicsBackend SFString to a WbPhysicsBackend* via the
// process-wide registry. Worlds that omit the field (or use the default
// "ode") get WbOdeBackend. Worlds that opt into "newton" get WbNewtonBackend
// when it's actually available -- otherwise the registry silently falls
// back to ODE so the world still loads and runs. This matches the
// three-layer compatibility contract from cuda-newton-physics-plan.md.
const QString &WbSolid::physicsBackendName() const {
  // WbSolidDevice subclass models (Accelerometer.wrl, Camera.wrl, GPS.wrl, …)
  // don't declare the `physicsBackend` field, so findSFString returns NULL on
  // their WbSolid base sub-object. Treat that as the "ode" default — devices
  // never participate in backend selection. Without this guard, every world
  // containing any WbSolidDevice subclass crashed in postFinalize.
  static const QString kOde = QStringLiteral("ode");
  if (!mPhysicsBackend)
    return kOde;
  return mPhysicsBackend->value();
}

WbPhysicsBackend *WbSolid::physicsBackend() const {
  const QByteArray name = physicsBackendName().toUtf8();
  return WbPhysicsBackendRegistry::resolve(WbPhysicsBackendKindFromString(name.constData()));
}

WbBodyHandle WbSolid::bodyHandle() const {
  // Newton wins when both backends own a body for this Solid: the ODE
  // body in that case is a collision-only bridge proxy and its
  // dynamic state is whatever the bridge last wrote, not the truth.
  // Newton's body_q is the truth.
  if (mNewtonBodyIndex >= 0)
    return WbNewtonBackend::handleFromIndex(mNewtonBodyIndex);
  if (dBodyID b = bodyMerger())
    return static_cast<WbBodyHandle>(b);
  return nullptr;
}

int WbSolid::effectiveNewtonBodyIndex() const {
  if (mNewtonBodyIndex >= 0)
    return mNewtonBodyIndex;
  if (mSolidMerger && mSolidMerger->solid() != nullptr && mSolidMerger->solid() != this)
    return mSolidMerger->solid()->newtonBodyIndex();
  return -1;
}

QString WbSolid::effectivePhysicsBackendName(bool *downgradedByGate) const {
  if (downgradedByGate != nullptr)
    *downgradedByGate = false;
  static const QString kOde = QStringLiteral("ode");
  static const QString kNewton = QStringLiteral("newton");
  const QString local = physicsBackendName();
  // An EXPLICIT local choice ("ode" or "newton") always wins.
  if (local == kOde || local == kNewton)
    return local;
  // local is "auto" (the Phase-D Solid/Robot default, baa1c104): a robot
  // is one coupled multibody system and MUST resolve to a single solver,
  // so an EXPLICIT backend on any ancestor governs this whole subtree.
  // Walk up for the nearest ancestor Solid that explicitly chose a
  // backend and inherit it.
  //
  // Before this guard, a descendant's *default* "auto" was returned
  // immediately (it's != "ode"), overriding an ancestor URDFRobot that
  // was explicitly set to "ode": the chassis stayed ODE while the
  // imported leg Solids (hip/upper_leg/lower_leg default "auto") each got
  // a Newton body, so hip_y/knee registered as Newton joints inside an
  // otherwise-ODE robot. ODE actuated those joints correctly but their
  // position sensors read Newton's never-advancing seed angle (Newton
  // wasn't stepping that robot) -- the Spot "frozen joint sensor"
  // regression that blinded the residual walker's closed-loop obs.
  for (WbNode *n = parentNode(); n != nullptr; n = n->parentNode()) {
    const WbSolid *p = dynamic_cast<const WbSolid *>(n);
    if (p == nullptr)
      continue;
    const QString a = p->physicsBackendName();
    if (a == kOde || a == kNewton)
      return a;  // explicit ancestor governs the whole articulation
  }
  // No explicit ancestor: consult the world-level default (default-flip-plan.md §3.2) before
  // resolving bare "auto". A world can thus pin every still-"auto" Solid to "ode" (the legacy
  // fallback) or "newton" without editing each node. Inert when unset (empty -> falls through). An
  // explicit local or ancestor backend already returned above, so this never overrides one.
  const WbWorldInfo *const wi = WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
  if (wi != nullptr) {
    const QString &wd = wi->defaultPhysicsBackend();
    if (wd == kOde || wd == kNewton)
      return wd;
  }
  // Still bare "auto": capability gate (default-flip-plan.md §4.1). If this articulation uses a
  // Newton-unsupported feature (non-Hinge/Slider joint = correctness; mesh collision = fidelity), keep
  // it on ODE so it's never silently degraded or solver-mixed. Monotonically safe (ODE handles all).
  // This is the ONE return that is a genuine SILENT Newton->ODE downgrade (the other "ode" returns above
  // are explicit choices), so flag it for the enforcement sweep in flushPendingNewtonRegistrations.
  if (!articulationNewtonCapable()) {
    if (downgradedByGate != nullptr)
      *downgradedByGate = true;
    return kOde;
  }
  // bare "auto", Newton-capable -> resolve it (Newton when available).
  return local;
}

// P3.10j: Walk the boundingObject sub-tree looking for a mesh
// (WbTriangleMeshGeometry subclasses: WbMesh, WbIndexedFaceSet). If
// found, compute the local-frame AABB from the mesh's vertex stream
// and emit the half-extents via the out params. Returns true on hit.
//
// URDFs with mesh collisions (Spot's body/hip/upper-leg use STL meshes,
// not primitives) fall through Newton's primitive narrow-phase
// otherwise; the placeholder-shape fallback in flushPendingNewton-
// Registrations isn't a substitute, it just gets the body floating.
// An AABB box is a coarse approximation but enough for ground contact
// and joint-actuator stability.
static bool computeBoundingObjectMeshAabb(WbBaseNode *bo,
                                          double *hx, double *hy, double *hz,
                                          double *cx, double *cy, double *cz) {
  // Unwrap nested Pose/Group wrappers the same way the primitive path
  // does in flushPendingNewtonRegistrations.
  for (int unwrap = 0; unwrap < 4 && bo != nullptr; ++unwrap) {
    if (dynamic_cast<WbTriangleMeshGeometry *>(bo))
      break;
    if (const WbGroup *g = dynamic_cast<const WbGroup *>(bo)) {
      const WbMFNode &kids = g->children();
      if (kids.size() == 0)
        return false;
      bo = dynamic_cast<WbBaseNode *>(kids.item(0));
      continue;
    }
    return false;
  }
  WbTriangleMeshGeometry *const tmg = dynamic_cast<WbTriangleMeshGeometry *>(bo);
  if (tmg == nullptr)
    return false;
  WbTriangleMesh *const tm = tmg->triangleMesh();
  if (tm == nullptr)
    return false;
  const int n = tm->numberOfVertices();
  if (n <= 0)
    return false;
  const double *v = tm->coordinatesData();
  if (v == nullptr)
    return false;
  double xMin = v[0], xMax = v[0];
  double yMin = v[1], yMax = v[1];
  double zMin = v[2], zMax = v[2];
  for (int i = 1; i < n; ++i) {
    const double x = v[3 * i + 0], y = v[3 * i + 1], z = v[3 * i + 2];
    if (x < xMin) xMin = x; else if (x > xMax) xMax = x;
    if (y < yMin) yMin = y; else if (y > yMax) yMax = y;
    if (z < zMin) zMin = z; else if (z > zMax) zMax = z;
  }
  *hx = 0.5 * (xMax - xMin);
  *hy = 0.5 * (yMax - yMin);
  *hz = 0.5 * (zMax - zMin);
  *cx = 0.5 * (xMin + xMax);
  *cy = 0.5 * (yMin + yMax);
  *cz = 0.5 * (zMin + zMax);
  if (*hx < 1e-4) *hx = 1e-4;
  if (*hy < 1e-4) *hy = 1e-4;
  if (*hz < 1e-4) *hz = 1e-4;
  return true;
}

// §4.1 capability gate — recursive subtree walk: is this Solid + all its joint-connected descendants
// Newton-supported? A non-Hinge/Slider joint (Newton never registers it -> mixed/broken articulation) or
// a mesh boundingObject (Newton only AABB-approximates it; the conservative default prefers exact ODE)
// makes the whole articulation ineligible. No root resolution here -- the caller starts at the root.
// reasonOut (optional): set to the first disqualifier found ("mesh" | "joint") when this returns false,
// so the OMNISIM_PROBE_GATE coverage meter (newton-ode-replacement-plan.md W0.2) can histogram WHY an
// articulation falls back, not just that it did. Left untouched on the capable (true) path.
static bool solidSubtreeNewtonCapable(const WbSolid *s, const char **reasonOut = nullptr) {
  if (s == nullptr)
    return true;
  // Tier B (fidelity): mesh collision geometry. Newton now handles triangle-mesh collision NATIVELY
  // (add_shape_mesh, newton-ode-replacement-plan.md W1) -- it no longer just AABB-approximates -- so a
  // mesh boundingObject no longer forces the articulation onto ODE by default. OMNISIM_NEWTON_MESH_TO_ODE=1
  // restores the old conservative routing (mesh -> ODE) without editing worlds, for any mesh class that
  // proves unfaithful under Newton (the plan's "flip per-shape, cautiously" lever).
  if (qEnvironmentVariableIsSet("OMNISIM_NEWTON_MESH_TO_ODE")) {
    WbBaseNode *const bo = s->boundingObject();
    if (bo != nullptr) {
      double hx, hy, hz, cx, cy, cz;
      if (computeBoundingObjectMeshAabb(bo, &hx, &hy, &hz, &cx, &cy, &cz)) {
        if (reasonOut != nullptr)
          *reasonOut = "mesh";
        return false;
      }
    }
  }
  // Tier A (correctness): a joint to a child that Newton can't register (only Hinge/Slider are).
  for (WbBasicJoint *const j : s->jointChildren()) {
    // EXACT type via nodeType(), NOT dynamic_cast: WbBallJoint + WbHinge2Joint both INHERIT WbHingeJoint,
    // so dynamic_cast<WbHingeJoint*> wrongly accepts them. Newton registers HingeJoint + SliderJoint
    // (revolute/prismatic), Hinge2Joint (a native 2-DoF d6, since W2) and BallJoint (a native 3-DoF
    // spherical joint, since W2.2 -- see WbBasicJoint flush). A motorised ball joint still needs MuJoCo
    // (ball target pos/vel control is XPBD-unsupported upstream), but the passive constraint is admitted here.
    const int nt = j->nodeType();
    if (nt != WB_NODE_HINGE_JOINT && nt != WB_NODE_SLIDER_JOINT && nt != WB_NODE_HINGE_2_JOINT &&
        nt != WB_NODE_BALL_JOINT) {
      if (reasonOut != nullptr)
        *reasonOut = "joint";
      return false;
    }
    // The ARTICULATED child hangs off the joint's ENDPOINT, not solidChildren() (which holds only fixed
    // sub-solids). Recurse there to reach a deeper non-Hinge/Slider joint or mesh — without this, a
    // mixed hinge->ball chain is missed and wrongly admitted to Newton (verified failure).
    if (!solidSubtreeNewtonCapable(j->solidEndPoint(), reasonOut))
      return false;
  }
  // Also recurse fixed sub-solids (Solids nested without an intervening joint).
  for (WbSolid *const c : s->solidChildren())
    if (!solidSubtreeNewtonCapable(c, reasonOut))
      return false;
  return true;
}

bool WbSolid::articulationNewtonCapable(const char **reasonOut) const {
  if (reasonOut != nullptr)
    *reasonOut = "capable";
  // Power-user / A-B-test bypass: force auto->Newton even for unsupported features.
  if (qEnvironmentVariableIsSet("OMNISIM_AUTO_NO_CAPABILITY_GATE"))
    return true;
  // Resolve to the articulation ROOT (topmost Solid ancestor) so every body in one coupled multibody
  // system shares ONE answer -- mixing solvers within an articulation is the Spot frozen-sensor bug.
  const WbSolid *root = this;
  for (WbNode *n = parentNode(); n != nullptr; n = n->parentNode())
    if (const WbSolid *const p = dynamic_cast<const WbSolid *>(n))
      root = p;
  if (root != this)
    return root->articulationNewtonCapable(reasonOut);
  // Root: RECOMPUTE every call (do NOT cache). The first query can come from a joint's postFinalize
  // BEFORE the whole articulation subtree is collected (a child's jointChildren not yet populated) — a
  // stale "capable" cached then would wrongly let a mixed hinge+ball articulation onto Newton (verified:
  // it did). The decisive query is the LATE one in flushPendingNewtonRegistrations, by which point the
  // tree is complete; a fresh walk there sees the deeper non-Hinge/Slider joint and returns ode, so the
  // body registration is skipped. Only called at finalization (joint postFinalize + flush), never
  // per-frame, so an O(articulation) walk per call is fine. mNewtonArticulationCapable is reused purely
  // to log the ODE fallback ONCE (not as a result cache).
  const char *reason = "capable";
  const bool capable = solidSubtreeNewtonCapable(this, &reason);
  if (reasonOut != nullptr)
    *reasonOut = reason;  // "capable" | "mesh" | "joint" -- for the OMNISIM_PROBE_GATE coverage sweep
  if (!capable && mNewtonArticulationCapable != 0)
    WbLog::info(tr("[capability-gate] '%1' uses a Newton-unsupported feature (non-Hinge/Slider joint "
                   "or mesh collision); its \"auto\" physics stays on ODE.")
                  .arg(name()));
  mNewtonArticulationCapable = capable ? 1 : 0;
  return capable;
}

// P3.10d: walk a Solid's descendant tree summing masses of Solids that
// would be filtered as fixed-children. Stops descending into any
// HingeJoint subtree (those are real articulated bodies that get their
// own Newton mass). Returns the leader's own mass + the rolled-up
// fixed-child masses, mirroring ODE's WbSolidMerger combined-inertia
// behaviour. Without this, a URDFRobot wrapper Robot (default m=0.001)
// would dominate the inertia tensor of a 46 kg husky chassis, and the
// 2.6 kg wheels would fling the wrapper around like a kite.
static double rolledUpMass(const WbNode *root) {
  double m = 0.0;
  if (const WbSolid *rootSolid = dynamic_cast<const WbSolid *>(root)) {
    if (rootSolid->physics() != nullptr && rootSolid->physics()->mass() > 0.0)
      m += rootSolid->physics()->mass();
  }
  if (const WbGroup *g = dynamic_cast<const WbGroup *>(root)) {
    const WbMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const WbNode *kid = kids.item(i);
      if (kid == nullptr)
        continue;
      // HingeJoints + their endPoint Solids are independent Newton
      // bodies -- their masses live in their own bodies, NOT rolled up.
      if (dynamic_cast<const WbBasicJoint *>(kid))
        continue;
      m += rolledUpMass(kid);
    }
  }
  return m;
}

// Walk the leader Solid + its FIXED-child descendants (same traversal as
// rolledUpMass, stopping at joint subtrees) and collect the dynamic (mass>0)
// Solids. Used by the composite-inertia rollup below.
static void gatherFixedSolids(const WbNode *root, QList<const WbSolid *> &out) {
  if (const WbSolid *const s = dynamic_cast<const WbSolid *>(root)) {
    if (s->physics() != nullptr && s->physics()->mass() > 0.0)
      out.append(s);
  }
  if (const WbGroup *const g = dynamic_cast<const WbGroup *>(root)) {
    const WbMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const WbNode *const kid = kids.item(i);
      if (kid == nullptr || dynamic_cast<const WbBasicJoint *>(kid))
        continue;
      gatherFixedSolids(kid, out);
    }
  }
}

// OMNISIM_NEWTON_COMPOSITE_INERTIA (opt-in): compose the PHYSICALLY CORRECT
// mass + center-of-mass + inertia over the leader Solid AND its fixed-child
// descendants (parallel-axis theorem), so a merged Newton body carries the true
// composite inertial properties instead of the LEADER LINK's only. The default
// rollup sums MASS but keeps only the leader's inertia/COM -- that made the G1
// torso body (torso+head+... merged) laterally ASYMMETRIC (whole-body CoM ~2cm
// off-centre) and wrong-inertia, so the deploy tipped under SolverMuJoCo while
// Newton's add_urdf (which welds the links with the correct composite) stood.
// Outputs are in the LEADER BODY frame: outCom = composite COM,
// Iout = {ixx, iyy, izz, ixy, ixz, iyz} about that COM. Returns false (caller
// keeps the leader-only values) if there is nothing to compose.
static bool rolledUpComInertia(const WbSolid *leader, double &outMass,
                               WbVector3 &outCom, double Iout[6]) {
  QList<const WbSolid *> bodies;
  gatherFixedSolids(leader, bodies);
  if (bodies.isEmpty())
    return false;
  // Per-body world-frame mass, COM and inertia-about-COM.
  auto worldInertia = [](const WbSolid *D, double &m, WbVector3 &cW, WbMatrix3 &IW) {
    const WbPhysics *const p = D->physics();
    m = p->mass();
    const WbMatrix3 R = D->rotationMatrix();
    const WbVector3 pD = D->matrix().translation();
    WbVector3 cl(0.0, 0.0, 0.0);
    if (p->centerOfMass().size() >= 1)
      cl = p->centerOfMass().item(0);
    cW = pD + R * cl;
    double ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    const WbMFVector3 &im = p->inertiaMatrix();
    if (im.size() >= 1) { const WbVector3 &dd = im.item(0); ixx = dd.x(); iyy = dd.y(); izz = dd.z(); }
    if (im.size() >= 2) { const WbVector3 &oo = im.item(1); ixy = oo.x(); ixz = oo.y(); iyz = oo.z(); }
    const WbMatrix3 Il(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
    IW = R * Il * R.transposed();
  };
  double M = 0.0;
  WbVector3 mc(0.0, 0.0, 0.0);
  for (const WbSolid *const D : bodies) {
    double m; WbVector3 cW; WbMatrix3 IW;
    worldInertia(D, m, cW, IW);
    M += m;
    mc = mc + cW * m;
  }
  if (M <= 0.0)
    return false;
  const WbVector3 cWtot = mc * (1.0 / M);
  double Iw[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (const WbSolid *const D : bodies) {
    double m; WbVector3 cW; WbMatrix3 IW;
    worldInertia(D, m, cW, IW);
    const WbVector3 d = cW - cWtot;
    const double d2 = d.dot(d);
    const double pa[9] = {
      d2 - d.x() * d.x(), -d.x() * d.y(),     -d.x() * d.z(),
      -d.y() * d.x(),     d2 - d.y() * d.y(), -d.y() * d.z(),
      -d.z() * d.x(),     -d.z() * d.y(),     d2 - d.z() * d.z()};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        Iw[r * 3 + c] += IW(r, c) + m * pa[r * 3 + c];
  }
  const WbMatrix3 Rs = leader->rotationMatrix();
  const WbVector3 ps = leader->matrix().translation();
  const WbMatrix3 IWm(Iw[0], Iw[1], Iw[2], Iw[3], Iw[4], Iw[5], Iw[6], Iw[7], Iw[8]);
  const WbMatrix3 Il = Rs.transposed() * IWm * Rs;
  outMass = M;
  outCom = Rs.transposed() * (cWtot - ps);
  Iout[0] = Il(0, 0); Iout[1] = Il(1, 1); Iout[2] = Il(2, 2);
  Iout[3] = Il(0, 1); Iout[4] = Il(0, 2); Iout[5] = Il(1, 2);
  return true;
}

// Phase-D regression guard companion to rolledUpMass(): true iff this
// Solid (or any of its fixed-child descendants -- same traversal as
// rolledUpMass, stopping at joint subtrees) carries a Physics node.
// A Solid with NO Physics anywhere in its fixed-child subtree is static
// scene geometry (floors, walls, sun markers) and, exactly like ODE,
// must NOT get a dynamic body. rolledUpMass alone can't gate this: a
// density-mass Physics node reports mass()<=0 yet is genuinely dynamic,
// so we test for the node's presence, not its mass value.
static bool subtreeHasPhysics(const WbNode *root) {
  if (const WbSolid *const rootSolid = dynamic_cast<const WbSolid *>(root)) {
    if (rootSolid->physics() != nullptr)
      return true;
  }
  if (const WbGroup *const g = dynamic_cast<const WbGroup *>(root)) {
    const WbMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const WbNode *const kid = kids.item(i);
      if (kid == nullptr)
        continue;
      // Joint endpoints are independent Newton bodies registered on
      // their own; their Physics doesn't roll up into this leader.
      if (dynamic_cast<const WbBasicJoint *>(kid))
        continue;
      if (subtreeHasPhysics(kid))
        return true;
    }
  }
  return false;
}

// P8.2: attach a Solid's boundingObject geometry to Newton body `idx`
// as a collision shape. Walks Pose/Group wrappers (accumulating their
// translation offsets, mirroring the URDF collision-<origin> convention)
// down to the inner primitive. Returns a human-readable shape
// description, or empty if no recognizable primitive was found. Shared
// verbatim by the dynamic-body and static-body registration paths so
// both narrow-phase shapes stay identical (cylinder->sphere
// substitution, mesh->AABB approximation, the lot). `boundingObjectValue`
// is the raw `mBoundingObject->value()` node.
// Per-material SOFT contact: a Solid whose contactMaterial matches
// OMNISIM_NEWTON_SOFT_MATERIAL (default "cube") gets a low contact ke so its
// MuJoCo solref is compliant. Needed for loose bin CONTENTS resting on a DYNAMIC
// bin floor: the gripper plowing the dense layer otherwise injects launch energy
// and ejects neighbours. MuJoCo mixes a contact toward the softer geom, so only
// the contents are softened while the bin structure / ground / grasp stay firm.
// Returns the soft ke, or -1 to keep the firm default.
static double newtonSoftKeForMaterial(const WbSFString *contactMaterial) {
  if (contactMaterial == nullptr)
    return -1.0;
  // OPT-IN: no soft ke unless OMNISIM_NEWTON_SOFT_KE is explicitly set, so the
  // default physics is exactly unchanged for every existing world.
  bool ok = false;
  const double v = qgetenv("OMNISIM_NEWTON_SOFT_KE").toDouble(&ok);
  if (!ok || v <= 0.0)
    return -1.0;
  QByteArray softMat = qgetenv("OMNISIM_NEWTON_SOFT_MATERIAL");
  if (softMat.isEmpty())
    softMat = "cube";
  if (contactMaterial->value() != QString::fromUtf8(softMat))
    return -1.0;
  return v;
}

// Register a single primitive geometry as a Newton collision shape on body
// `idx`, offset by `off`. Returns a description or empty if `g` isn't a
// recognized primitive. Shared by the single-shape and compound walkers.
static QString addNewtonPrimitive(WbNewtonBackend *newton, int idx,
                                  const WbBaseNode *g, const WbVector3 &off,
                                  double softKe) {
  if (const WbSphere *sphere = dynamic_cast<const WbSphere *>(g)) {
    newton->addShapeSphere(idx, sphere->radius(), off.x(), off.y(), off.z());
    return QString("sphere r=%1 at (%2,%3,%4)").arg(sphere->radius())
        .arg(off.x()).arg(off.y()).arg(off.z());
  }
  if (const WbBox *box = dynamic_cast<const WbBox *>(g)) {
    const WbVector3 &sz = box->size();
    newton->addShapeBox(idx, sz.x() * 0.5, sz.y() * 0.5, sz.z() * 0.5,
                        off.x(), off.y(), off.z(), softKe);
    return QString("box hx=%1 hy=%2 hz=%3 at (%4,%5,%6)")
        .arg(sz.x() * 0.5).arg(sz.y() * 0.5).arg(sz.z() * 0.5)
        .arg(off.x()).arg(off.y()).arg(off.z());
  }
  if (const WbCylinder *cyl = dynamic_cast<const WbCylinder *>(g)) {
    newton->addShapeSphere(idx, cyl->radius(), off.x(), off.y(), off.z());
    return QString("sphere(was cylinder) r=%1 at (%2,%3,%4)").arg(cyl->radius())
        .arg(off.x()).arg(off.y()).arg(off.z());
  }
  if (const WbCapsule *cap = dynamic_cast<const WbCapsule *>(g)) {
    newton->addShapeCapsule(idx, cap->radius(), cap->height() * 0.5);
    return QString("capsule r=%1 hh=%2").arg(cap->radius()).arg(cap->height() * 0.5);
  }
  return QString();
}

// Compound walker: recurse the WHOLE boundingObject sub-tree and register
// EVERY primitive as its own Newton shape (accumulating Pose/Transform
// translation offsets), so a multi-collider rigid body -- a free dynamic bin
// with floor + walls + a graspable handle on ONE body -- attaches all of its
// colliders, not just the first child of the Group. The default path (below)
// still registers only the first primitive, so this richer behaviour is
// OPT-IN via OMNISIM_NEWTON_COMPOUND_COLLIDERS to keep every existing world's
// physics byte-for-byte unchanged.
static QString registerNewtonShapesRec(WbNewtonBackend *newton, int idx,
                                       const WbBaseNode *bo, WbVector3 off,
                                       double softKe) {
  if (bo == nullptr)
    return QString();
  if (const WbShape *sh = dynamic_cast<const WbShape *>(bo))
    return registerNewtonShapesRec(newton, idx, sh->geometry(), off, softKe);
  // WbPose extends WbGroup: accumulate its translation, then recurse its kids.
  if (const WbPose *p = dynamic_cast<const WbPose *>(bo)) {
    const WbVector3 childOff = off + p->translation();
    QString desc;
    const WbMFNode &kids = p->children();
    for (int i = 0; i < kids.size(); ++i) {
      const QString d = registerNewtonShapesRec(
          newton, idx, dynamic_cast<WbBaseNode *>(kids.item(i)), childOff, softKe);
      if (!d.isEmpty())
        desc += (desc.isEmpty() ? "" : "; ") + d;
    }
    return desc;
  }
  if (const WbGroup *g = dynamic_cast<const WbGroup *>(bo)) {
    QString desc;
    const WbMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const QString d = registerNewtonShapesRec(
          newton, idx, dynamic_cast<WbBaseNode *>(kids.item(i)), off, softKe);
      if (!d.isEmpty())
        desc += (desc.isEmpty() ? "" : "; ") + d;
    }
    return desc;
  }
  return addNewtonPrimitive(newton, idx, bo, off, softKe);
}

static QString attachNewtonShapeFromBoundingObject(WbNewtonBackend *newton, int idx,
                                                   WbBaseNode *boundingObjectValue,
                                                   double softKe = -1.0) {
  // OPT-IN two ways (mirrors newtonStatics / newtonRobotColliders): register
  // every collider in a compound boundingObject (Group of offset primitives on
  // one rigid body) instead of just the first child, via the launch env var OR
  // the per-world WorldInfo.newtonCompoundColliders field -- so a demo world folds
  // the knob into the .wbt and "just works" in the GUI / launcher / headless with
  // no env var. Per-call (NOT static) so a world switched in via the launcher's
  // worldReload reads ITS OWN field; the defaults keep every existing world's
  // physics byte-for-byte unchanged.
  const WbWorldInfo *const wiCompound =
      WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
  const bool compound = !qgetenv("OMNISIM_NEWTON_COMPOUND_COLLIDERS").isEmpty() ||
                        (wiCompound != nullptr && wiCompound->newtonCompoundColliders());
  if (compound) {
    const QString d = registerNewtonShapesRec(newton, idx, boundingObjectValue,
                                              WbVector3(0.0, 0.0, 0.0), softKe);
    if (!d.isEmpty())
      return d;
    // fall through to the single-shape walker if nothing matched
  }
  QString shapeDesc;
  WbBaseNode *bo = boundingObjectValue;
  WbVector3 shapeOffset(0.0, 0.0, 0.0);
  for (int unwrap = 0; unwrap < 4 && bo != nullptr; ++unwrap) {
    if (dynamic_cast<const WbSphere *>(bo) ||
        dynamic_cast<const WbBox *>(bo) ||
        dynamic_cast<const WbCylinder *>(bo) ||
        dynamic_cast<const WbCapsule *>(bo) ||
        dynamic_cast<const WbTriangleMeshGeometry *>(bo))
      break;
    if (const WbShape *sh = dynamic_cast<const WbShape *>(bo)) {
      // `boundingObject Shape { geometry Box {..} }` (and the common
      // `boundingObject USE <visualShape>` idiom) wraps the primitive in
      // a Shape. ODE resolves this natively; Newton's walker previously
      // didn't, fell through to the r=0.12 placeholder sphere, and every
      // such body rested 0.12 m up / got ejected at spawn. Descend into
      // the Shape's geometry and re-test.
      bo = sh->geometry();
      continue;
    }
    if (const WbPose *p = dynamic_cast<const WbPose *>(bo))
      shapeOffset += p->translation();  // WbPose extends WbGroup; falls through
    if (const WbGroup *g = dynamic_cast<const WbGroup *>(bo)) {
      const WbMFNode &kids = g->children();
      if (kids.size() == 0)
        break;
      bo = dynamic_cast<WbBaseNode *>(kids.item(0));
      continue;
    }
    break;
  }
  if (const WbSphere *sphere = dynamic_cast<const WbSphere *>(bo)) {
    const double radius = sphere->radius();
    newton->addShapeSphere(idx, radius, shapeOffset.x(), shapeOffset.y(), shapeOffset.z());
    shapeDesc = QString("sphere r=%1 at (%2,%3,%4)").arg(radius)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
  } else if (const WbBox *box = dynamic_cast<const WbBox *>(bo)) {
    const WbVector3 &sz = box->size();
    newton->addShapeBox(idx, sz.x() * 0.5, sz.y() * 0.5, sz.z() * 0.5,
                        shapeOffset.x(), shapeOffset.y(), shapeOffset.z(), softKe);
    shapeDesc = QString("box hx=%1 hy=%2 hz=%3 at (%4,%5,%6)")
                    .arg(sz.x() * 0.5).arg(sz.y() * 0.5).arg(sz.z() * 0.5)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
  } else if (const WbCylinder *cyl = dynamic_cast<const WbCylinder *>(bo)) {
    // W1.2 (newton-ode-replacement-plan.md): hand the cylinder to Newton as an oriented CAPSULE of the same
    // radius + half-height (addShapeCylinder substitutes a capsule -- native cylinder narrow-phase locks
    // wheels against the ground, probe 7). A capsule's central segment is a true cylinder of the correct
    // rolling radius, so this is a LINE contact of the right width -- strictly more faithful than the old
    // point-contact sphere, and the capsule narrow-phase is robust. The backend rotates the capsule Z->Y
    // (Webots cylinders extend along body-local Y) and applies shapeOffset.
    const double radius = cyl->radius();
    const double halfHeight = cyl->height() * 0.5;
    newton->addShapeCylinder(idx, radius, halfHeight, shapeOffset.x(), shapeOffset.y(), shapeOffset.z());
    shapeDesc = QString("cylinder->capsule r=%1 hh=%2 at (%3,%4,%5)").arg(radius).arg(halfHeight)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
  } else if (const WbCapsule *cap = dynamic_cast<const WbCapsule *>(bo)) {
    const double radius = cap->radius();
    const double halfHeight = cap->height() * 0.5;
    newton->addShapeCapsule(idx, radius, halfHeight);
    shapeDesc = QString("capsule r=%1 hh=%2").arg(radius).arg(halfHeight);
  } else if (dynamic_cast<WbPlane *>(bo) != nullptr) {
    // Infinite ground plane (newton-ode-replacement-plan.md W1.1) -- a Floor's Plane boundingObject. Newton
    // add_shape_plane, local normal +Z (the WbPlane convention); the body's transform orients it. Without
    // this the static floor had NO Newton collider, so Newton-dynamic props fell straight through it (the
    // faithful-check finding). Takes effect when the floor is a Newton static body (OMNISIM_NEWTON_STATICS).
    newton->addShapePlane(idx, shapeOffset.x(), shapeOffset.y(), shapeOffset.z());
    shapeDesc = QString("plane off=(%1,%2,%3)").arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
  } else if (WbTriangleMeshGeometry *const tmg = dynamic_cast<WbTriangleMeshGeometry *>(bo)) {
    // NATIVE triangle-mesh collision (newton-ode-replacement-plan.md W1): hand the mesh's vertices +
    // triangle indices straight to Newton (add_shape_mesh) instead of the old AABB-box approximation. The
    // verts are in the geometry's local frame; shapeOffset places that frame in the body (translation
    // only, matching the primitive shapes). Falls back to the AABB box if the mesh data isn't available.
    WbTriangleMesh *const tm = tmg->triangleMesh();
    if (tm != nullptr && tm->numberOfVertices() > 0 && tm->numberOfTriangles() > 0) {
      newton->addShapeMesh(idx, tm->coordinatesData(), tm->numberOfVertices(), tm->indicesData(),
                           tm->numberOfTriangles(), shapeOffset.x(), shapeOffset.y(), shapeOffset.z());
      shapeDesc = QString("mesh verts=%1 tris=%2 off=(%3,%4,%5)")
                      .arg(tm->numberOfVertices()).arg(tm->numberOfTriangles())
                      .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
    } else {
      double hx = 0.0, hy = 0.0, hz = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
      if (computeBoundingObjectMeshAabb(boundingObjectValue, &hx, &hy, &hz, &cx, &cy, &cz)) {
        newton->addShapeBox(idx, hx, hy, hz, cx, cy, cz);
        shapeDesc = QString("box(mesh AABB fallback)");
      }
    }
  }
  return shapeDesc;
}

// Count the collision primitives the compound walker (registerNewtonShapesRec)
// would register from a boundingObject sub-tree -- i.e. how many Newton shapes a
// COMPOUND dynamic body actually gets. Used to detect a multi-collider free body
// (a movable bin/tote) so it can be given its real geometry inertia instead of the
// single-body Husky mass preset. Mirrors registerNewtonShapesRec's recursion.
static int countNewtonCompoundPrimitives(const WbBaseNode *bo) {
  if (bo == nullptr)
    return 0;
  if (const WbShape *sh = dynamic_cast<const WbShape *>(bo))
    return countNewtonCompoundPrimitives(sh->geometry());
  if (const WbGroup *g = dynamic_cast<const WbGroup *>(bo)) {  // also covers WbPose
    int n = 0;
    const WbMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i)
      n += countNewtonCompoundPrimitives(dynamic_cast<WbBaseNode *>(kids.item(i)));
    return n;
  }
  if (dynamic_cast<const WbSphere *>(bo) || dynamic_cast<const WbBox *>(bo) ||
      dynamic_cast<const WbCylinder *>(bo) || dynamic_cast<const WbCapsule *>(bo))
    return 1;
  return 0;
}

void WbSolid::flushPendingNewtonRegistrations() {
  // Resolve once outside the loop -- isAvailable() check is cheap but
  // we don't need a separate registry hit per Solid.
  WbPhysicsBackend *const raw = WbPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  WbNewtonBackend *const newton = static_cast<WbNewtonBackend *>(raw);

  // Count the bodies THIS flush actually registers. flushPendingNewtonRegistrations()
  // is called every tick (WbSimulationWorld::step), but registration is a one-shot
  // per Solid (mNewtonBodyIndex >= 0 short-circuits below), so on all but the first
  // few ticks of a world this stays 0 -- which is what gates the census log at the
  // bottom of this function. Without it the census re-fired on EVERY tick and
  // flooded the agent-facing controller.log stream on GET /sim/events.
  int registeredThisFlush = 0;

  for (const WbSolid *const cs : cSolids) {
    // Iteration is over const pointers (cSolids is List<const WbSolid*>);
    // we mutate via the const-cast since mNewtonBodyIndex is a private
    // bookkeeping field. Acceptable here because the underlying list
    // entries aren't truly const -- they're just stored that way.
    WbSolid *const s = const_cast<WbSolid *>(cs);
    if (s == nullptr || s->mNewtonBodyIndex >= 0)
      continue;
    // P3.10: pick up inherited backend from any ancestor Solid (e.g.
    // URDFRobot-generated chassis Solids inherit from the outer Robot).
    bool gatedToOde = false;
    if (s->effectivePhysicsBackendName(&gatedToOde) == QStringLiteral("ode")) {
      // Newton enforcement (2026-06-29 default: no silent Newton->ODE). A
      // DELIBERATE ODE opt-out (explicit physicsBackend "ode" on this Solid,
      // an ancestor, or WorldInfo.defaultPhysicsBackend) is fine and stays
      // silent -- gatedToOde is false for those. But a bare-"auto"
      // articulation that the §4.1 capability gate routed to ODE -- because it
      // uses a Newton-unsupported feature -- is a SILENT downgrade: the world
      // reports Newton yet part of it runs ODE. Refuse to do that quietly.
      if (gatedToOde && WbPhysicsBackendRegistry::newtonEnforced()) {
        const char *reason = "capable";
        s->articulationNewtonCapable(&reason);
        WbLog::fatal(
            tr("[newton-enforce] Solid '%1' would silently fall back to ODE: its \"auto\" "
               "articulation uses a Newton-unsupported feature (%2), so the capability gate routed "
               "it to ODE while the rest of the world runs on Newton. Newton enforcement is on (the "
               "default on a Newton-capable build); a silent Newton->ODE downgrade is refused. Fix: "
               "make the feature Newton-compatible, or set the articulation's physicsBackend to "
               "\"ode\" explicitly, or allow the graceful fall-back with OMNISIM_ALLOW_ODE_FALLBACK=1.")
                .arg(s->name(), QString::fromUtf8(reason)));
      }
      continue;
    }

    // P3.10c: skip Solids that ODE would fold into a parent solid via
    // its WbSolidMerger -- i.e. Physics-bearing Solids that hang
    // directly off another Solid without an intervening joint. URDF
    // "fixed" joints, Group/Pose nesting, and the URDFRobot expansion's
    // bumper/top-plate/IMU children all land here. ODE's WbSolidMerger
    // hasn't run yet by flush time so we can't query it directly;
    // instead walk the tree -- if the first WbSolid or WbBasicJoint
    // ancestor is a WbSolid (no intervening joint), this is a fixed
    // child and shouldn't get its own Newton body. WbBasicJoint
    // endpoint Solids (wheels, articulated end-effectors) get their
    // own bodies as expected because the joint is what we hit first
    // walking up.
    bool isFixedChild = false;
    for (WbNode *n = s->parentNode(); n != nullptr; n = n->parentNode()) {
      if (dynamic_cast<WbBasicJoint *>(n))
        break;  // joint ancestor first -- this Solid is a real articulated body
      if (dynamic_cast<WbSolid *>(n)) {
        isFixedChild = true;
        break;
      }
    }
    if (isFixedChild)
      continue;

    // staticBase HUMANOID special case (G1 / H1 / Atlas seated or pinned).
    // For `staticBase TRUE` the URDF importer strips ONLY the *root* link's
    // Physics block (WbUrdfImporter), which is enough for a fixed-base arm
    // (a static-base arm): its base subtree then has no Physics at all, so
    // subtreeHasPhysics(s) is false and it takes the static-weld path below.
    // A floating-base humanoid pelvis, however, usually carries a FIXED-child
    // decoration link that still has an inertial (G1's pelvis_contour_link),
    // so subtreeHasPhysics(s) stays TRUE and the base wrongly takes the
    // dynamic free-root path -- the pelvis then skates around under the arm's
    // reaction torque (the seated arm-mimic base drift). The staticBase intent
    // is unambiguous from the stripped root physics, so detect it directly
    // (an articulated Robot whose own Physics was removed) and route the base
    // through the SAME FIXED-joint weld the fixed-base arms use.
    const bool staticBaseRobot =
        dynamic_cast<const WbRobot *>(s) != nullptr &&
        s->physics() == nullptr &&
        !s->mJointChildren.isEmpty();

    // A Solid with no Physics node anywhere in its fixed-child subtree
    // is STATIC scene geometry (RectangleArena floor, walls,
    // OmniSimSunMarker, obstacles). Before Phase D these were skipped
    // here because their default physicsBackend was "ode"; the
    // "auto" -> "newton" default flip (baa1c104) now lets them through,
    // and they would each receive a spurious *dynamic* Newton body with
    // the 0.25 kg fallback mass below. Because such furniture is a
    // top-level (root) Solid whose pose field still emits `changed` per
    // step, its WbSolid::syncNewtonPoseFromFields bridge fires
    // resetJointsToDefaults() on the SHARED webots_world articulation
    // every tick -- snapping every real robot's joints + bodies back to
    // spawn and freezing them in place (the husky chassis-freeze bug).
    if (!subtreeHasPhysics(s) || staticBaseRobot) {
      // staticBase robot root (a static-base arm, bolted-down manipulator, ...): the URDF
      // importer strips the base link's Physics block for `staticBase TRUE`
      // (the kinematic "bolted to the floor" root, WbUrdfImporter). That
      // leaves this WbRobot wrapper looking like furniture -- and skipped
      // below -- so its base->link1 hinge had no Newton parent body and was
      // dropped (WbBasicJoint flush needs a body on BOTH ends). The arm
      // articulation then had no root and Newton never stepped it: the arm
      // froze at its spawn pose under Newton while actuating fine under ODE.
      // Register the base as a Newton STATIC (mass=0) body so the root hinge
      // attaches; finalize() anchors a static root with a FIXED joint (not a
      // 6-DOF free joint), giving the arm a pinned root to articulate off.
      // Default-on (staticBase arms are common), unlike env-gated scene
      // statics. Only staticBase robots reach here -- a normal robot's base
      // keeps its Physics so subtreeHasPhysics() is true and it takes the
      // dynamic addBody/free-root path below, unchanged.
      if (dynamic_cast<const WbRobot *>(s) != nullptr) {
        // Only an ARTICULATED robot base needs a Newton root body: a joint must
        // hang off it. A physics-less Robot with NO joint children (the
        // draggable OmniSimSunMarker supervisor, a sensor-only beacon, ...) is
        // a kinematic marker, not a robot base -- mJointChildren is exactly the
        // set of joints whose Newton parent body is this Solid, so an empty
        // list means nothing attaches here. Registering such a marker as a
        // static body injects a spurious mass=0 orphan into the "statics"
        // articulation: harmless under XPBD but fatal to SolverMuJoCo, which
        // refuses to compile a body whose mass/inertia is below mjMINVAL.
        if (s->mJointChildren.isEmpty())
          continue;
        if (newton->ensureWorldOpen() != 0)
          return;
        const WbVector3 bt = s->matrix().translation();
        const WbQuaternion bq = WbRotation(s->rotationMatrix()).toQuaternion();
        const int bidx = newton->addStaticBody(bt.x(), bt.y(), bt.z(),
                                               bq.x(), bq.y(), bq.z(), bq.w());
        if (bidx >= 0) {
          s->mNewtonBodyIndex = bidx;
          s->mNewtonBodyIsStatic = true;
          ++registeredThisFlush;
        }
        continue;
      }
      // P8.2 (statics-on-Newton): opt-in via OMNISIM_NEWTON_STATICS.
      // A top-level static collider (has a boundingObject, no joint
      // parent, opted into Newton) registers as a mass=0 STATIC Newton
      // body so it's a collision surface for dynamic Newton bodies,
      // instead of staying ODE-only. mNewtonBodyIsStatic flags it so the
      // per-step pose writeback + syncNewtonPoseFromFields (incl. the
      // articulation-wide resetJointsToDefaults that caused the freeze)
      // are both skipped -- a pinned body has nothing to write back and
      // must never touch the shared articulation. Opt-in two ways
      // (default-flip-plan.md §4.2 N3): the OMNISIM_NEWTON_STATICS env var
      // (launch override) OR the per-world WorldInfo.newtonStatics field (folds
      // the knob into the .wbt so a legged/floored world needs no env var). The
      // default stays "skip" (statics ODE-side) so the husky / head-on /
      // determinism paths are byte-unchanged unless a world opts in.
      {
        const WbWorldInfo *const wiStatics =
            WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
        const bool staticsOn = !qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_STATICS") ||
                               (wiStatics != nullptr && wiStatics->newtonStatics());
        if (!staticsOn)
          continue;
      }
      if (s->upperPose() != nullptr)
        continue;  // only walk from top-level scene statics
      // Colliders may sit on the top-level Solid itself OR on fixed-child
      // Solids inside a PROTO expansion -- Wall.proto, for one, keeps its
      // Box on a nested child Solid and the root has no boundingObject at
      // all (which made every warehouse/maze wall intangible to Newton
      // bodies: the husky drove straight through them). Harvest every
      // boundingObject in the joint-free subtree and register each as its
      // own static body at its world pose.
      QVector<WbSolid *> staticColliders;
      QVector<WbNode *> walk;
      walk.append(s);
      while (!walk.isEmpty()) {
        WbNode *const node = walk.takeLast();
        if (WbSolid *const sol = dynamic_cast<WbSolid *>(node)) {
          if (sol->mBoundingObject != nullptr && sol->mBoundingObject->value() != nullptr)
            staticColliders.append(sol);
        }
        if (const WbGroup *const g = dynamic_cast<const WbGroup *>(node)) {
          const WbMFNode &kids = g->children();
          for (int i = 0; i < kids.size(); ++i) {
            WbNode *const kid = kids.item(i);
            if (kid != nullptr && dynamic_cast<WbBasicJoint *>(kid) == nullptr)
              walk.append(kid);
          }
        }
      }
      if (staticColliders.isEmpty())
        continue;
      WbPhysicsBackend *sback = WbPhysicsBackendRegistry::resolve(WbPhysicsBackendKind::Newton);
      if (sback == nullptr || sback->kind() != WbPhysicsBackendKind::Newton)
        continue;
      if (newton->ensureWorldOpen() != 0)
        return;
      for (WbSolid *const sc : staticColliders) {
        const WbVector3 st = sc->matrix().translation();
        const WbQuaternion sq = WbRotation(sc->rotationMatrix()).toQuaternion();
        const int sidx = newton->addStaticBody(st.x(), st.y(), st.z(),
                                               sq.x(), sq.y(), sq.z(), sq.w());
        if (sidx >= 0) {
          attachNewtonShapeFromBoundingObject(
              newton, sidx, dynamic_cast<WbBaseNode *>(sc->mBoundingObject->value()),
              newtonSoftKeForMaterial(sc->mContactMaterial));
          sc->mNewtonBodyIndex = sidx;
          sc->mNewtonBodyIsStatic = true;
          ++registeredThisFlush;
        }
      }
      continue;
    }

    // Resolve via the local physicsBackend() so the registry's
    // fall-back layer still applies for the rare case of an
    // explicitly-set "newton" being unavailable.
    WbPhysicsBackend *back = s->physicsBackend();
    if (back == nullptr || back->kind() != WbPhysicsBackendKind::Newton) {
      // Local says "ode" but ancestor says "newton". Trust the ancestor.
      back = WbPhysicsBackendRegistry::resolve(WbPhysicsBackendKind::Newton);
      if (back == nullptr || back->kind() != WbPhysicsBackendKind::Newton)
        continue;
    }

    if (newton->ensureWorldOpen() != 0)
      return;

    // World-space pose: matrix() walks the parent transform chain
    // (Solid -> HingeJoint -> Solid -> ... -> root), so wheels under a
    // chassis end up at their actual world position rather than the
    // local SFVec3f field value.
    const WbVector3 t = s->matrix().translation();
    const WbMatrix3 R = s->rotationMatrix();
    const WbQuaternion q = WbRotation(R).toQuaternion();

    // P3.10d: combined mass = own mass + rolled-up fixed-child masses.
    // For URDFRobot wrappers this picks up the inertial_link's mass
    // and any fixed-link decorations (top_chassis, bumpers, rails,
    // imu_link, etc.) so the chassis Newton body has the correct
    // total inertia. Falls back to a sane default of 0.25 kg if the
    // Solid has no Physics + no fixed-child mass at all.
    double mass = rolledUpMass(s);
    if (mass <= 0.0)
      mass = 0.25;

    // P3.10l: read the URDF inertia tensor straight off the Solid's
    // Physics node. WbPhysics::inertiaMatrix() returns 2 vec3s when
    // explicitly set: [(ixx, iyy, izz), (ixy, ixz, iyz)] (Webots'
    // 6-value upper-triangular convention). If the field is empty,
    // pass zeros and let the helper module fall back to the
    // chassis-vs-wheel preset.
    double ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    double cx = 0, cy = 0, cz = 0;
    bool hasCom = false;
    if (const WbPhysics *const phys = s->physics()) {
      const WbMFVector3 &im = phys->inertiaMatrix();
      if (im.size() >= 1) {
        const WbVector3 &diag = im.item(0);
        ixx = diag.x(); iyy = diag.y(); izz = diag.z();
      }
      if (im.size() >= 2) {
        const WbVector3 &off = im.item(1);
        ixy = off.x(); ixz = off.y(); iyz = off.z();
      }
      // OMNISIM_NEWTON_USE_LINK_COM (default off): opt-in true link COM; off =
      // COM at link origin (legacy, every existing Newton robot validated
      // against this). Rebuild-gated. When on, pass the Solid's centerOfMass
      // (WbPhysics, link/body frame) so the Newton body's COM matches the URDF
      // inertial origin -- the inertia tensor above is already about the COM
      // frame, so the pairing is physically correct.
      if (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_USE_LINK_COM")) {
        const WbMFVector3 &com = phys->centerOfMass();
        if (com.size() >= 1) {
          const WbVector3 &c = com.item(0);
          cx = c.x(); cy = c.y(); cz = c.z();
          hasCom = true;
        }
      }
    }

    // OMNISIM_NEWTON_COMPOSITE_INERTIA (opt-in, default OFF): override the
    // leader-only mass/inertia/COM with the PHYSICALLY CORRECT composite over
    // the leader + its fixed-child descendants. The leader-only rollup made the
    // merged G1 torso body laterally asymmetric + wrong-inertia, tipping the
    // deploy stand; the composite matches Newton's add_urdf (which stands).
    if (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_COMPOSITE_INERTIA")) {
      double cMass = 0.0;
      WbVector3 cCom(0.0, 0.0, 0.0);
      double cI[6] = {0, 0, 0, 0, 0, 0};
      if (rolledUpComInertia(s, cMass, cCom, cI) && cMass > 0.0) {
        mass = cMass;
        ixx = cI[0]; iyy = cI[1]; izz = cI[2];
        ixy = cI[3]; ixz = cI[4]; iyz = cI[5];
        cx = cCom.x(); cy = cCom.y(); cz = cCom.z();
        hasCom = true;
      }
    }

    // FIX (compound dynamic-body inertia): a DYNAMIC free body that declares mass
    // but NO inertiaMatrix and whose boundingObject is a COMPOUND of >=2 colliders
    // (a movable bin / tote) would otherwise inherit the Husky-chassis mass preset
    // diag(0.0094m, 0.0167m, 0.0094m) in World.add_body -- wrong magnitude AND
    // DEGENERATE (Ixx == Izz, a tied eigenvalue pair in the vertical X-Z plane).
    // Newton's SolverMuJoCo eig3 resolves that tie to an ARBITRARY rotated
    // body_iquat, so mj_step integrates the free body in a tilted inertial frame
    // and drops the box-box contacts on one body-local half -- parts resting on the
    // bin's +x floor fall straight through. createOdeMass() has already integrated
    // the body's real geometry inertia about the solid ORIGIN into odeMass()->I
    // (the value the Newton path otherwise discards); feed it so add_body takes its
    // existing explicit-tensor branch and the degenerate preset is never reached.
    // Scoped tight: only fires under the compound-collider opt-in, only when no
    // explicit inertiaMatrix was given, and only for a >=2-collider boundingObject
    // -- so single-collider bodies, statics, and every URDF robot link (which ship
    // an explicit inertia tensor) are byte-identical.
    if (ixx <= 0.0 && iyy <= 0.0 && izz <= 0.0 && s->mBoundingObject != nullptr) {
      const WbWorldInfo *const wiCmp =
          WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
      const bool compoundOn =
          !qgetenv("OMNISIM_NEWTON_COMPOUND_COLLIDERS").isEmpty() ||
          (wiCmp != nullptr && wiCmp->newtonCompoundColliders());
      if (compoundOn &&
          countNewtonCompoundPrimitives(
              dynamic_cast<WbBaseNode *>(s->mBoundingObject->value())) >= 2) {
        const dMass *const om = s->odeMass();
        if (om != nullptr && om->mass > 0.0 &&
            om->I[0] > 0.0 && om->I[5] > 0.0 && om->I[10] > 0.0) {
          ixx = om->I[0]; iyy = om->I[5]; izz = om->I[10];
          ixy = om->I[1]; ixz = om->I[2]; iyz = om->I[6];
          // Even the geometry-correct diagonal can be eig3-degenerate: a square bin
          // has Ixx==Iyy, and Izz is the largest so the diagonal is not descending --
          // newton's eig3 then resolves the tie / permutation to a rotated body_iquat
          // that still drops a few contacts on one body-local edge (the residual +y
          // sink). When the tensor is effectively diagonal (negligible products of
          // inertia), force a strictly-distinct DESCENDING diagonal so eig3 returns
          // an identity inertial frame. The principal-axis re-labeling is dynamically
          // irrelevant for these heavy, controller-pinned compound bins -- and the
          // (>=2-collider + compound opt-in) gate reaches no other body.
          if (ixy > -1e-9 && ixy < 1e-9 && ixz > -1e-9 && ixz < 1e-9 &&
              iyz > -1e-9 && iyz < 1e-9) {
            double hi = ixx, mid = iyy, lo = izz;
            if (mid > hi) { const double t = hi; hi = mid; mid = t; }
            if (lo > hi) { const double t = hi; hi = lo; lo = t; }
            if (lo > mid) { const double t = mid; mid = lo; lo = t; }
            if (mid >= hi) mid = hi * 0.99;   // break ties -> strictly descending
            if (lo >= mid) lo = mid * 0.99;
            ixx = hi; iyy = mid; izz = lo;
          }
        }
      }
    }

    QString shapeDesc;
    const int idx = newton->addBody(mass, t.x(), t.y(), t.z(),
                                    q.x(), q.y(), q.z(), q.w(),
                                    ixx, iyy, izz, ixy, ixz, iyz,
                                    hasCom, cx, cy, cz);
    if (idx >= 0) {
      // P3.10i: WbRobot wrappers (URDFRobot expansion produces these)
      // typically have a chassis-envelope bounding box that includes
      // the wheel space -- ground contact via that box short-circuits
      // the wheel rolling motion (chassis sits on the box, wheels
      // don't carry weight). Skip the shape on the wrapper Robot;
      // the descendant wheel Solids' shapes handle ground contact.
      // Mass + inertia (rolled up via P3.10d) stay on the body.
      const bool isRobotWrapper = dynamic_cast<const WbRobot *>(s) != nullptr;
      // P3.10j: Husky's URDFRobot wrapper has a chassis-envelope box
      // that engulfs the wheel space -- using it as the Newton shape
      // pins the chassis on the ground and the wheels lose contact.
      // The default behaviour is therefore "wrapper gets a 1 mm
      // placeholder, real ground contact comes from descendants".
      // Spot (and any quadruped) breaks that assumption: its legs
      // hang below the chassis, so the chassis bounding object SHOULD
      // be the load-bearing shape. Opt-in via env var, keeping the
      // husky-friendly default intact.
      // Opt-in two ways (mirrors newtonStatics): the launch env var OR the
      // per-world WorldInfo.newtonRobotColliders field, so a demo world folds
      // the knob into the .wbt and "just works" in the GUI without an env var.
      const WbWorldInfo *const wiRobotColliders =
          WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
      const bool wrapperUsesOwnShape =
          isRobotWrapper &&
          (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE") ||
           (wiRobotColliders != nullptr && wiRobotColliders->newtonRobotColliders()));
      const bool useBoundingObject = (s->mBoundingObject != nullptr) &&
                                     (!isRobotWrapper || wrapperUsesOwnShape);
      if (useBoundingObject) {
        // P8.2: shape extraction (Pose-unwrap + primitive/mesh handling)
        // is now shared with the static-body path via this helper.
        shapeDesc = attachNewtonShapeFromBoundingObject(
            newton, idx, dynamic_cast<WbBaseNode *>(s->mBoundingObject->value()),
            newtonSoftKeForMaterial(s->mContactMaterial));
      }
      if (shapeDesc.isEmpty() && !isRobotWrapper) {
        newton->addShapeSphere(idx, 0.12);
        shapeDesc = QStringLiteral("sphere r=0.12 (default fallback)");
      }
      if (isRobotWrapper && shapeDesc.isEmpty()) {
        // P3.10i: Newton's joint constraints break if a body has no
        // collision shape at all (the hinges silently decouple). Add
        // a tiny sphere (r=1mm) just to keep the body well-formed for
        // the solver -- physically negligible, doesn't interfere with
        // ground contact via descendant wheels. Only used when the
        // wrapper's own bounding object would short-circuit those.
        newton->addShapeSphere(idx, 0.001);
        shapeDesc = QStringLiteral("sphere r=0.001 (Robot wrapper placeholder)");
      }
      s->mNewtonBodyIndex = idx;
      ++registeredThisFlush;
      // P5 step-1 perf fix 2026-05-28: the per-body "registered solid"
      // WbLog::info fires during step 1 *after* mConsoleLogsPostponed
      // has flipped back to false (per WbMainWindow::restorePerspective
      // → setConsoleLogsPostponed(false)), so the WbLog skip-receivers
      // postponed-short-circuit in WbLog.cpp doesn't apply. Per-call
      // overhead × N bodies stalls scenes past ~20 huskies. Suppressed
      // here; the bounded one-shot "world opened" + the bounded "hinge
      // joint N" logs are still emitted, which is enough for the
      // "Newton physics actually running" diagnostic.

      // P4: stop ODE from simulating this body. Newton owns its dynamics
      // now; ODE iterating across 50+ Newton-backed bodies every Webots
      // tick was the dominant cost in the 10-husky swarm world (sim ran
      // at ~0.4 fps before this). dBodyDisable freezes the body in ODE
      // -- forces don't accumulate, contacts don't fire, position
      // doesn't update. Newton's per-step writeback in postPhysicsStep
      // is what now drives the visible pose.
      if (s->mSolidMerger && s->mSolidMerger->body() != nullptr)
        s->mSolidMerger->setBodyArtificiallyDisabled(true);
    }
  }

  // One-shot registration census: how many Solids actually received a
  // Newton body, split static vs dynamic, with the first few static
  // collider names. The per-body logs are suppressed for perf, so this
  // is the only "are the crates/walls actually colliders?" signal.
  //
  // GUARD (2026-07-11): fire only on a flush that ACTUALLY REGISTERED something.
  // The census was written to be one-shot, but its only condition was "some Solid
  // has a Newton body" -- which is true on every tick once the world is built, and
  // this function runs every tick (WbSimulationWorld::step -> flushPendingNewtonRegistrations).
  // So it re-emitted the identical line at tick rate, and because WbLog::info is
  // relayed to the agent-facing controller.log stream, it drowned GET /sim/events:
  // measured 96.7% of the log stream was this one line, with real controller
  // telemetry silently evicted from the ring buffer (dropped_log climbing without
  // bound). Gating on registeredThisFlush keeps the diagnostic exactly where it is
  // useful -- the tick(s) that build the world, and any LATER tick that registers a
  // newly-spawned Solid (a supervisor-injected robot), which is genuinely worth a
  // line -- and is self-resetting across world loads: a reloaded world constructs
  // fresh WbSolids with mNewtonBodyIndex = -1, so they re-register and re-census.
  // No static/global flag to leak or forget to reset.
  if (registeredThisFlush > 0) {
    int nStatic = 0, nDynamic = 0;
    QStringList staticNames;
    for (const WbSolid *const cs : cSolids) {
      if (cs == nullptr || cs->mNewtonBodyIndex < 0)
        continue;
      if (cs->mNewtonBodyIsStatic) {
        ++nStatic;
        if (staticNames.size() < 8)
          staticNames.append(cs->name());
      } else {
        ++nDynamic;
      }
    }
    if (nStatic > 0 || nDynamic > 0)
      WbLog::info(QString("[WbNewtonBackend] registered %1 dynamic + %2 static "
                          "Newton bodies (+%4 this pass) (statics: %3)")
                      .arg(nDynamic).arg(nStatic).arg(staticNames.join(", ")).arg(registeredThisFlush));
  }

  // Per-world Newton solver choice (WorldInfo.newtonSolver). Plumbed here --
  // after registrations have opened the build-phase world but before
  // WbNewtonBackend::finalizeWorld() builds the solver -- so a world that
  // needs SolverMuJoCo's robust frictional contact (e.g. a friction pinch
  // grasp, which XPBD structurally cannot hold) selects it declaratively in
  // the .wbt instead of via an env var. No-op when nothing registered (world
  // never opened) or when the field is empty/"auto"/"xpbd" (default XPBD).
  if (newton->isWorldOpenForBuild()) {
    const WbWorldInfo *const wi = WbWorld::instance() ? WbWorld::instance()->worldInfo() : nullptr;
    if (wi != nullptr) {
      newton->setSolverPreference(wi->newtonSolver());
      // Fold WorldInfo.newtonSubsteps into the runtime too (N3): a contact-heavy
      // world declares its XPBD sub-steps in the .wbt instead of via an env var
      // (OMNISIM_NEWTON_SUBSTEPS still overrides). No-op for the default (1).
      newton->setNewtonSubsteps(wi->newtonSubsteps());
    }
  }
}

bool WbSolid::isSleeping() const {
  dBodyID b = bodyMerger();
  if (b)
    return WbPhysicsBackendRegistry::odeBackend()->isBodyEnabled(static_cast<WbBodyHandle>(b)) == 0;
  return false;
}

// ODE encapsulated methods. P1.5 widening: both flow through the
// dispatcher; bodyMerger() returns an ODE body so the registry
// resolves to WbOdeBackend.
void WbSolid::addForceAtPosition(const WbVector3 &force, const WbVector3 &position) {
  const double f[3] = {force.x(), force.y(), force.z()};
  const double p[3] = {position.x(), position.y(), position.z()};
  WbPhysicsBackendRegistry::odeBackend()->addBodyForceAtPos(static_cast<WbBodyHandle>(bodyMerger()), f, p);
}

void WbSolid::addTorque(const WbVector3 &torque) {
  const double t[3] = {torque.x(), torque.y(), torque.z()};
  WbPhysicsBackendRegistry::odeBackend()->addBodyTorque(static_cast<WbBodyHandle>(bodyMerger()), t);
}

// W3.1 (newton-ode-replacement-plan.md): route an external force-at-world-position + torque to this Solid's
// NEWTON body if it has one. Newton's body_f is a WORLD-frame wrench about the body's reference point, so a
// force F at world point P becomes (F, (P - bodyOrigin) x F + torque). Returns true if it routed to Newton
// (the caller then skips the ODE path, whose body is disabled for Newton-backed Solids); false -> not
// Newton-backed, caller falls back to ODE. The body origin from the Newton pose readback approximates the
// COM as the torque reference -- exact when the Solid's frame is at its COM.
bool WbSolid::applyExternalForceNewton(const WbVector3 &force, const WbVector3 &worldPos,
                                       const WbVector3 &torque) const {
  if (qEnvironmentVariableIsSet("OMNISIM_DEBUG_FORCE")) {
    QFile df(qEnvironmentVariable("OMNISIM_DEBUG_FORCE"));
    if (df.open(QIODevice::Append | QIODevice::Text))
      df.write(QString("solid=%1 newtonIdx=%2 F=(%3,%4,%5)\n").arg(name()).arg(mNewtonBodyIndex)
                   .arg(force.x(), 0, 'f', 3).arg(force.y(), 0, 'f', 3).arg(force.z(), 0, 'f', 3).toUtf8());
  }
  // Revert lever (W3.1): OMNISIM_NEWTON_NO_EXT_FORCE=1 falls back to the pre-W3.1 behavior (ODE path, which
  // is a no-op on a Newton-backed Solid's disabled body) so the force-injection delta can be A/B'd.
  if (qEnvironmentVariableIsSet("OMNISIM_NEWTON_NO_EXT_FORCE"))
    return false;
  if (mNewtonBodyIndex < 0)
    return false;
  WbPhysicsBackend *const backend = WbPhysicsBackendRegistry::newtonBackend();
  if (backend == nullptr || !backend->isAvailable())
    return false;
  WbNewtonBackend *const newton = static_cast<WbNewtonBackend *>(backend);
  WbVector3 ref = matrix().translation();  // fallback: the Solid's world origin
  double xform[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  if (newton->getBodyXform(mNewtonBodyIndex, xform) == 0)
    ref = WbVector3(xform[0], xform[1], xform[2]);
  const WbVector3 d = worldPos - ref;
  const WbVector3 tau(d.y() * force.z() - d.z() * force.y(),
                      d.z() * force.x() - d.x() * force.z(),
                      d.x() * force.y() - d.y() * force.x());
  return newton->addBodyForce(mNewtonBodyIndex, force.x(), force.y(), force.z(),
                              tau.x() + torque.x(), tau.y() + torque.y(), tau.z() + torque.z()) == 0;
}

// W3.2: route a mid-step velocity set to this Solid's NEWTON body if it has one (angular=false -> linear
// half, true -> angular half of body_qd). Returns true when routed to Newton; false -> not Newton-backed,
// caller falls back to ODE (whose body is disabled for Newton Solids, so the set would otherwise be lost).
bool WbSolid::setNewtonBodyVel(const double v[3], bool angular) const {
  if (mNewtonBodyIndex < 0)
    return false;
  WbPhysicsBackend *const backend = WbPhysicsBackendRegistry::newtonBackend();
  if (backend == nullptr || !backend->isAvailable())
    return false;
  return static_cast<WbNewtonBackend *>(backend)->setBodyVel(mNewtonBodyIndex, v[0], v[1], v[2],
                                                             angular ? 1 : 0) == 0;
}

// Selection management
void WbSolid::propagateSelection(bool selected) {
  if (wrenNode() && mIsPermanentlyKinematic) {
    const WbPropeller *const propeller = dynamic_cast<WbPropeller *>(parentNode());
    if (propeller) {
      const bool active = propeller->helix() == this;
      wr_node_set_visible(WR_NODE(wrenNode()), selected || active);
    }
  }

  select(selected);
  WbMatter::propagateSelection(selected);

  foreach (WbBasicJoint *const j, mJointChildren) {
    if (j->solidReference())
      continue;
    WbSolid *const solid = j->solidEndPoint();
    if (solid)
      solid->propagateSelection(selected);
  }

  WbBaseNode *const bo = boundingObject();
  if (bo)
    bo->propagateSelection(selected);
}

void WbSolid::setMatrixNeedUpdate() {
  WbNode *bo = boundingObject();
  WbGroup *g = dynamic_cast<WbGroup *>(bo);
  if (g)
    g->setMatrixNeedUpdate();

  WbPose::setMatrixNeedUpdate();
}

void WbSolid::reset(const QString &id) {
  WbMatter::reset(id);

  for (int i = 0; i < mImmersionProperties->size(); ++i)
    mImmersionProperties->item(i)->reset(id);
  WbNode *const p = mPhysics->value();
  if (p)
    p->reset(id);

  if (mJointParents.size() == 0) {
    setTranslation(translationFromFile(id));
    setRotation(rotationFromFile(id));
  }
  resetSingleSolidPhysics();
  resetContactPointsAndSupportPolygon();
  resetContactPoints();
  resetImmersions();

  // remove contact joints
  if (isSolidMerger()) {
    dBodyID b = body();
    int jointNumber = dBodyGetNumJoints(b);
    dJointID joints[jointNumber];
    for (int i = 0; i < jointNumber; ++i)
      joints[i] = dBodyGetJoint(b, i);
    for (int i = 0; i < jointNumber; ++i) {
      if (dJointGetType(joints[i]) == dJointTypeContact) {
        dJointAttach(joints[i], 0, 0);
        dJointDestroy(joints[i]);
      }
    }
  }

  int counter = 0;
  restoreHiddenKinematicParameters(mHiddenKinematicParametersMap, counter);

  if (handleJerkIfNeeded())
    mMovedChildren.clear();
  else if (!mMovedChildren.isEmpty())
    childrenJerk();

  mListOfImmersions.clear();
}

void WbSolid::save(const QString &id) {
  WbMatter::save(id);
  if (isTopSolid())
    saveHiddenFieldValues();

  for (int i = 0; i < mImmersionProperties->size(); ++i)
    mImmersionProperties->item(i)->save(id);
  WbNode *const p = mPhysics->value();
  if (p)
    p->save(id);
}

// Recursive reset methods
// It resets the positions of all ODE dGeoms, static or not, based on the current translation and rotation fields
// It also resets the velocities of all dBodies to 0.0
void WbSolid::jerk(bool resetVelocities, bool rootJerk) {
  if (isSolidMerger())
    mSolidMerger->setGeomAndBodyPositions(resetVelocities, mJointParents.size() == 0 && !isTopSolid());
  else
    updateOdeGeomPosition();

  foreach (WbSolid *const solid, mSolidChildren)
    solid->jerk(resetVelocities, false);

  foreach (WbBasicJoint *const j, mJointChildren)
    j->updateOdeWorldCoordinates();

  if (isDynamic() && mJointParents.size() > 0 && rootJerk)
    emit positionChangedArtificially();
}

void WbSolid::notifyChildJerk(WbPose *childNode) {
  const WbNode *node = childNode->parentNode();
  while (node != this && node != NULL) {
    if (mMovedChildren.contains(dynamic_cast<const WbPose *>(node)))
      return;
    node = node->parentNode();
  }

  mMovedChildren.append(childNode);
}

void WbSolid::childrenJerk() {
  updateOdeGeomPosition();
  foreach (const WbPose *childNode, mMovedChildren) {
    QVector<WbSolid *> solidChildrenList;
    QVector<WbBasicJoint *> jointChildrenList;
    QVector<WbPropeller *> propellerChildrenList;
    collectSolidChildren(childNode, false, solidChildrenList, jointChildrenList, propellerChildrenList);

    foreach (WbSolid *const solid, solidChildrenList)
      solid->jerk(false, false);

    foreach (WbBasicJoint *const j, jointChildrenList)
      j->updateOdeWorldCoordinates();
  }
  mMovedChildren.clear();
}

void WbSolid::awake() {
  if (mSolidMerger && !mSolidMerger->isBodyArtificiallyDisabled())
    WbPhysicsBackendRegistry::odeBackend()->setBodyEnabled(static_cast<WbBodyHandle>(mSolidMerger->body()), true);
  else
    WbWorld::instance()->awake();
}

void WbSolid::awakeSolids(WbGroup *group) {
  assert(group);

  WbSolid *const solid = dynamic_cast<WbSolid *>(group);
  if (solid) {
    dBodyID b = solid->body();

    WbPhysicsBackend *const odeBackend = WbPhysicsBackendRegistry::odeBackend();
    if (b && odeBackend->isBodyEnabled(static_cast<WbBodyHandle>(b)) == 1)
      // this body is enabled => all its children are enabled too
      return;

    if (b && solid->solidMerger() && !solid->solidMerger()->isBodyArtificiallyDisabled())
      odeBackend->setBodyEnabled(static_cast<WbBodyHandle>(b), true);
    if (solid->mHasDynamicSolidDescendant) {
      const QVector<WbSolid *> &sl = solid->solidChildren();
      for (int i = 0; i < sl.size(); ++i)
        awakeSolids(sl.at(i));
    }
  } else {  // Handles the case of non-Solid (possibly nested) Groups which are children of the root
    WbMFNode::Iterator it(group->children());
    while (it.hasNext()) {
      WbGroup *const g = dynamic_cast<WbGroup *>(it.next());
      if (g)
        awakeSolids(g);
    }
  }
}

void WbSolid::resetPhysics(bool recursive) {
  resetSingleSolidPhysics();

  // Recurses through all first level solid descendants
  if (recursive) {
    foreach (WbSolid *const solid, mSolidChildren)
      solid->resetPhysics();
  }
}

void WbSolid::resetSingleSolidPhysics() {
  // check for joints and disable all motors
  const int size = mJointChildren.size();
  for (int i = 0; i < size; ++i) {
    WbJoint *const j = dynamic_cast<WbJoint *>(mJointChildren[i]);
    if (j)
      j->resetPhysics();
  }

  mLinearVelocity->setValue(0.0, 0.0, 0.0);
  mAngularVelocity->setValue(0.0, 0.0, 0.0);

  if (isSolidMerger()) {
    dBodyID b = body();
    // P1.5 widening: physics reset (linear+angular vel, force, torque)
    // and wake all flow through the dispatcher.
    WbPhysicsBackend *const odeBackend = WbPhysicsBackendRegistry::odeBackend();
    const double zero[3] = {0.0, 0.0, 0.0};
    odeBackend->setBodyLinearVel(static_cast<WbBodyHandle>(b), zero);
    odeBackend->setBodyAngularVel(static_cast<WbBodyHandle>(b), zero);
    odeBackend->setBodyForce(static_cast<WbBodyHandle>(b), zero);
    odeBackend->setBodyTorque(static_cast<WbBodyHandle>(b), zero);
    if (!mSolidMerger->isBodyArtificiallyDisabled())
      odeBackend->setBodyEnabled(static_cast<WbBodyHandle>(b), true);
  }

  if (mJoint) {
    // P1.6 slice 6: dispatcher-routed zero-on-disable for the Hinge /
    // Slider branches. (The Hinge2 / Ball branches stay direct until
    // slice 7 widens those joint families.) `odeBackend` from the
    // sibling isSolidMerger block above isn't in scope here, so we
    // re-resolve.
    WbPhysicsBackend *const ode = WbPhysicsBackendRegistry::odeBackend();
    switch (dJointGetType(mJoint)) {
      case dJointTypeHinge2: {
        const WbJointHandle h = reinterpret_cast<WbJointHandle>(mJoint);
        ode->setJointHinge2Param(h, 0, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointHinge2Param(h, 0, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        ode->setJointHinge2Param(h, 1, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointHinge2Param(h, 1, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        break;
      }
      case dJointTypeHinge: {
        const WbJointHandle h = reinterpret_cast<WbJointHandle>(mJoint);
        ode->setJointHingeParam(h, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointHingeParam(h, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        break;
      }
      case dJointTypeSlider: {
        const WbJointHandle h = reinterpret_cast<WbJointHandle>(mJoint);
        ode->setJointSliderParam(h, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointSliderParam(h, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        break;
      }
      case dJointTypeBall: {
        const WbJointHandle h = reinterpret_cast<WbJointHandle>(mJoint);
        ode->setJointBallParam(h, 0, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointBallParam(h, 0, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        ode->setJointBallParam(h, 1, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointBallParam(h, 1, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        ode->setJointBallParam(h, 2, WbPhysicsBackend::WB_JP_FMAX, 0.0);
        ode->setJointBallParam(h, 2, WbPhysicsBackend::WB_JP_VELOCITY, 0.0);
        break;
      }
      default:  // only the above joint types are currently implemented in Webots
        break;
    }
  }
}

void WbSolid::pausePhysics(bool resumeAutomatically) {
  if (resumeAutomatically)
    mResetPhysicsInStep = true;

  if (mSolidMerger)
    mSolidMerger->setBodyArtificiallyDisabled(true);

  foreach (WbSolid *const solid, mSolidChildren)
    solid->pausePhysics();
}

void WbSolid::resumePhysics() {
  resetSingleSolidPhysics();
  if (mSolidMerger)
    mSolidMerger->setBodyArtificiallyDisabled(false);

  foreach (WbSolid *const solid, mSolidChildren)
    solid->resumePhysics();
}

///////////////////////////////
// Contact Points Management //
///////////////////////////////

const QVector<WbVector3> &WbSolid::computedContactPoints(bool includeDescendants) {
  extractContactPoints();
  connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this, &WbSolid::resetContactPoints,
          Qt::UniqueConnection);
  return includeDescendants ? mGlobalListOfContactPoints : mListOfContactPoints;
}

const QVector<const WbSolid *> &WbSolid::computedSolidPerContactPoints() {
  extractContactPoints();
  connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this, &WbSolid::resetContactPoints,
          Qt::UniqueConnection);
  return mSolidPerContactPoints;
}

const QVector<double> &WbSolid::computedContactPointDepths(bool includeDescendants) {
  extractContactPoints();
  connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this, &WbSolid::resetContactPoints,
          Qt::UniqueConnection);
  return includeDescendants ? mGlobalListOfContactPointDepths : mListOfContactPointDepths;
}

void WbSolid::extractContactPoints() {
  if (mHasExtractedContactPoints)
    return;

  const WbWorld *const world = WbWorld::instance();
  const QList<WbOdeContact> &fullList = world->odeContacts();
  const int size = fullList.size();

  // W4.2c: for a Newton-backed Solid with native contacts enabled, the native pass below is the SOLE source
  // -- skip the ODE-bridge accumulation entirely. Multi-body verification showed a Robot's disabled ODE
  // proxies STILL collide, so ADDING native on top double-counted (rover: ODE=8 + native=5 -> 13). ODE Solids
  // and the default path are untouched (useNative is false unless this Solid is Newton-backed AND opted in).
  const bool useNative = (mNewtonBodyIndex >= 0
                          && qEnvironmentVariableIsSet("OMNISIM_NEWTON_NATIVE_CONTACTS")
                          && WbPhysicsBackendRegistry::newtonBackend() != nullptr
                          && WbPhysicsBackendRegistry::newtonBackend()->isAvailable());
  if (!useNative)
    for (int i = 0; i < size; ++i) {
    const dContactGeom &cg = fullList.at(i).contactGeom();
    const WbOdeGeomData *const odeGeomData1 = static_cast<WbOdeGeomData *>(dGeomGetData(cg.g1));
    const WbOdeGeomData *const odeGeomData2 = static_cast<WbOdeGeomData *>(dGeomGetData(cg.g2));
    const WbSolid *const s1 = odeGeomData1->solid();
    const WbSolid *const s2 = odeGeomData2->solid();

    if (s1 == this || s2 == this) {
      const double *const pos = cg.pos;
      const WbVector3 v(pos[0], pos[1], pos[2]);
      mListOfContactPoints.append(v);
      mListOfContactPointDepths.append(cg.depth);
    }

    if (s1->topSolid() == this || s2->topSolid() == this) {
      const double *const pos = cg.pos;
      const WbVector3 v(pos[0], pos[1], pos[2]);
      mGlobalListOfContactPoints.append(v);
      mGlobalListOfContactPointDepths.append(cg.depth);
      if (s1->topSolid() == this)
        mSolidPerContactPoints.append(s1);
      else
        mSolidPerContactPoints.append(s2);
      // stores the smallest y-coordinate of all contact points
      const double downProjection = v.dot(world->worldInfo()->upVector());
      if (downProjection < mY)
        mY = downProjection;
    }
  }

  // W4.2 (gated): compare the NATIVE Newton contacts for this Solid against the ODE-bridge list just built
  // from world->odeContacts(), so native/ODE parity can be proven before the source is swapped (W4.2c) and
  // the bridge retired (W4.3). Inert unless OMNISIM_NEWTON_CONTACTS_CMP=<file> is set. ODE and Newton are
  // different narrow-phases, so the match is qualitative (same body pairs, contacts near the same surface),
  // not point-identical.
  if (mNewtonBodyIndex >= 0 && qEnvironmentVariableIsSet("OMNISIM_NEWTON_CONTACTS_CMP")) {
    WbPhysicsBackend *const rawN = WbPhysicsBackendRegistry::newtonBackend();
    if (rawN != nullptr && rawN->isAvailable()) {
      std::vector<WbNewtonContact> nc;
      static_cast<WbNewtonBackend *>(rawN)->getContacts(nc);
      int nativeForThis = 0;
      QString detail;
      for (size_t i = 0; i < nc.size(); ++i) {
        const WbNewtonContact &c = nc[i];
        if (c.bodyA == mNewtonBodyIndex || c.bodyB == mNewtonBodyIndex) {
          ++nativeForThis;
          detail += QString("  native p=(%1,%2,%3) depth=%4 other=%5\n")
                        .arg(c.point[0], 0, 'f', 3).arg(c.point[1], 0, 'f', 3).arg(c.point[2], 0, 'f', 3)
                        .arg(c.depth, 0, 'f', 4)
                        .arg(c.bodyA == mNewtonBodyIndex ? c.bodyB : c.bodyA);
        }
      }
      QFile cf(qEnvironmentVariable("OMNISIM_NEWTON_CONTACTS_CMP"));
      if (cf.open(QIODevice::Append | QIODevice::Text))
        cf.write(QString("solid=%1 newtonIdx=%2 ODE_pts=%3 native_pts=%4\n%5")
                     .arg(name()).arg(mNewtonBodyIndex)
                     .arg(mListOfContactPoints.size()).arg(nativeForThis).arg(detail).toUtf8());
    }
  }

  // W4.2c (opt-in, gated by OMNISIM_NEWTON_NATIVE_CONTACTS): ADD the native Newton contacts for a Newton-backed
  // Solid -- the ODE bridge can't supply them (its disabled proxy body doesn't collide, so the loop above
  // appended nothing for this body; measured ODE_pts=0 vs native_pts=8). Additive + gated, so the default and
  // every ODE world stay byte-identical. Mirrors the ODE two-list logic exactly with native data; dedups the
  // floor's dual static(-1)/body registration by point proximity. This is what lets a pure-Newton world feed
  // the contact-points supervisor API (damage tracker) without the ODE collision pass (W4.3 then drops it).
  if (useNative) {
    WbPhysicsBackend *const rawN = WbPhysicsBackendRegistry::newtonBackend();
    if (rawN != nullptr && rawN->isAvailable()) {
      std::vector<WbNewtonContact> nc;
      static_cast<WbNewtonBackend *>(rawN)->getContacts(nc);
      QHash<int, WbSolid *> bodyToSolid;
      const QList<WbSolid *> allSolids = WbWorld::instance()->findSolids();
      for (int si = 0; si < allSolids.size(); ++si)
        if (allSolids.at(si)->mNewtonBodyIndex >= 0)
          bodyToSolid.insert(allSolids.at(si)->mNewtonBodyIndex, allSolids.at(si));
      const WbVector3 up = world->worldInfo()->upVector();
      for (size_t i = 0; i < nc.size(); ++i) {
        const WbNewtonContact &c = nc[i];
        WbSolid *const sA = (c.bodyA >= 0) ? bodyToSolid.value(c.bodyA, nullptr) : nullptr;  // -1 = static world
        WbSolid *const sB = (c.bodyB >= 0) ? bodyToSolid.value(c.bodyB, nullptr) : nullptr;
        const WbVector3 v(c.point[0], c.point[1], c.point[2]);
        if (sA == this || sB == this) {
          bool dup = false;
          for (int q = 0; q < mListOfContactPoints.size(); ++q)
            if ((mListOfContactPoints.at(q) - v).length() < 1e-4) { dup = true; break; }
          if (!dup) {
            mListOfContactPoints.append(v);
            mListOfContactPointDepths.append(c.depth);
          }
        }
        const bool aInSubtree = (sA != nullptr && sA->topSolid() == this);
        const bool bInSubtree = (sB != nullptr && sB->topSolid() == this);
        if (aInSubtree || bInSubtree) {
          bool dupG = false;
          for (int q = 0; q < mGlobalListOfContactPoints.size(); ++q)
            if ((mGlobalListOfContactPoints.at(q) - v).length() < 1e-4) { dupG = true; break; }
          if (!dupG) {
            mGlobalListOfContactPoints.append(v);
            mGlobalListOfContactPointDepths.append(c.depth);
            mSolidPerContactPoints.append(aInSubtree ? sA : sB);
            const double downProjection = v.dot(up);
            if (downProjection < mY)
              mY = downProjection;
          }
        }
      }
    }
  }

  mHasExtractedContactPoints = true;
}

void WbSolid::extractImmersions() {
  assert(isDynamic());
  if (mHasExtractedImmersions)
    return;

  const WbWorld *const world = WbWorld::instance();
  const QList<dImmersionGeom> &fullList = world->immersionGeoms();
  const int size = fullList.size();

  for (int i = 0; i < size; ++i) {
    const dImmersionGeom &ig = fullList.at(i);
    const WbOdeGeomData *const odeGeomData = static_cast<WbOdeGeomData *>(dGeomGetData(ig.g1));
    const WbSolid *const s = odeGeomData->solid();

    if (s == this)
      mListOfImmersions.append(ig);
  }

  mHasExtractedImmersions = true;
}

// Computes the support polygon of the robot if needed
const WbPolygon &WbSolid::supportPolygon() {
  const WbWorldInfo *const worldInfo = WbWorld::instance()->worldInfo();
  if (!mSupportPolygonNeedsUpdate)
    return mSupportPolygon;

  extractContactPoints();
  const int numberOfContactPoints = mGlobalListOfContactPoints.size();
  const WbVector3 &eastVector = worldInfo->eastVector();
  const WbVector3 &northVector = worldInfo->northVector();
  // Rules out 4 trivial cases
  if (numberOfContactPoints <= 3) {
    assert(mSupportPolygon.size() >= numberOfContactPoints);
    for (int i = 0; i < numberOfContactPoints; ++i) {
      const WbVector3 &v = mGlobalListOfContactPoints.at(i);
      mSupportPolygon[i].setXy(v.dot(northVector), v.dot(eastVector));
    }
    mSupportPolygon.setActualSize(numberOfContactPoints);
    return mSupportPolygon;
  }

  // From now on, the robot has at least 4 contact points
  QVector<WbVector2> listOfProjectedContactPoints(numberOfContactPoints);
  // Projects contact points onto a plane orthogonal to the down direction
  for (int i = 0; i < numberOfContactPoints; ++i) {
    const WbVector3 &v = mGlobalListOfContactPoints.at(i);
    listOfProjectedContactPoints[i].setXy(v.dot(northVector), v.dot(eastVector));
  }

  // Gets the indices of points in the convex hull of the projected contact points
  QVector<int> listOfIndices(numberOfContactPoints);
  const int supportPolygonSize = WbMathsUtilities::twoStepsConvexHull(listOfProjectedContactPoints, listOfIndices);

  // Resizes the support polygon only if the number of vertices has increased
  if (supportPolygonSize > mSupportPolygon.size())
    mSupportPolygon.resize(supportPolygonSize);
  mSupportPolygon.setActualSize(supportPolygonSize);

  // Extracts the support polygon from the projected contact points
  for (int i = 0; i < supportPolygonSize; ++i)
    mSupportPolygon[i] = listOfProjectedContactPoints.at(listOfIndices.at(i));

  // For optimization
  mSupportPolygonNeedsUpdate = false;

  return mSupportPolygon;
}

void WbSolid::deleteSupportPolygonRepresentation() {
  delete mSupportPolygonRepresentation;
  mSupportPolygonRepresentation = NULL;
}

// Creates or destroys the graphical support polygon of the solid according to menu actions: the solid must be a top solid
bool WbSolid::showSupportPolygonRepresentation(bool enabled) {
  if (mIsKinematic || !isTopLevel()) {
    if (enabled)
      info(tr("A top Solid with a non-NULL Physics node has to be chosen rather."));
    return false;
  }
  mSupportPolygonRepresentationIsEnabled = enabled;

  if (enabled) {
    if (!mSupportPolygonRepresentation) {
      mSupportPolygonRepresentation = new WbSupportPolygonRepresentation();
      mSupportPolygonNeedsUpdate = true;
    }
    if (mSupportPolygon.size() < 4)
      // minimum expected size to rule out trivial cases
      mSupportPolygon.resize(4);
    connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
            &WbSolid::refreshSupportPolygonRepresentation, Qt::UniqueConnection);
    connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
            &WbSolid::resetContactPointsAndSupportPolygon, Qt::UniqueConnection);
    mSupportPolygonRepresentation->show(true);
    wr_node_set_visible(WR_NODE(mGlobalCenterOfMassTransform), true);
    refreshSupportPolygonRepresentation();
    updateLineScale();
  } else {
    if (mSupportPolygonRepresentation) {
      disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
                 &WbSolid::refreshSupportPolygonRepresentation);
      disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
                 &WbSolid::resetContactPointsAndSupportPolygon);
      if (WbSimulationState::instance()->isFast() || !WbSimulationState::instance()->isRendering())
        deleteSupportPolygonRepresentation();
      else {
        mSupportPolygonRepresentation->show(false);
        wr_node_set_visible(WR_NODE(mGlobalCenterOfMassTransform), false);
      }
    }
  }

  return true;
}

// Shows or hides the graphical 'global' center of mass of the solid according to menu actions
bool WbSolid::showGlobalCenterOfMassRepresentation(bool enabled) {
  if (mIsKinematic) {
    if (enabled)
      info(tr("A Solid with a non-NULL Physics node must be chosen."));
    return false;
  }
  mGlobalCenterOfMassRepresentationIsEnabled = enabled;

  if (enabled) {
    connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
            &WbSolid::refreshGlobalCenterOfMassRepresentation, Qt::UniqueConnection);
    wr_node_set_visible(WR_NODE(mGlobalCenterOfMassTransform), true);
    refreshGlobalCenterOfMassRepresentation();
  } else {
    disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
               &WbSolid::refreshGlobalCenterOfMassRepresentation);
    wr_node_set_visible(WR_NODE(mGlobalCenterOfMassTransform), false);
  }

  return true;
}

// Shows or hides the graphical center of buoyancy of the solid according to menu actions
bool WbSolid::showCenterOfBuoyancyRepresentation(bool enabled) {
  if (mIsKinematic) {
    if (enabled)
      info(tr("A Solid with a non-NULL Physics node must be chosen."));
    return false;
  }
  mCenterOfBuoyancyRepresentationIsEnabled = enabled;

  if (enabled) {
    connect(WbSimulationState::instance(), &WbSimulationState::physicsStepEnded, this,
            &WbSolid::refreshCenterOfBuoyancyRepresentation, Qt::UniqueConnection);
    refreshCenterOfBuoyancyRepresentation();
  } else {
    disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepEnded, this,
               &WbSolid::refreshCenterOfBuoyancyRepresentation);
    wr_node_set_visible(WR_NODE(mCenterOfBuoyancyTransform), false);
  }

  return true;
}

void WbSolid::refreshPhysicsRepresentation() {
  if (mSupportPolygonRepresentationIsEnabled)
    refreshSupportPolygonRepresentation();
  else if (mGlobalCenterOfMassRepresentationIsEnabled)
    refreshGlobalCenterOfMassRepresentation();

  if (mCenterOfBuoyancyRepresentationIsEnabled)
    refreshCenterOfBuoyancyRepresentation();

  // propagate change to ancestors
  emit physicsPropertiesChanged();
}

// Redraws the support polygon of the solid after each physics step when required by the menu options
void WbSolid::refreshSupportPolygonRepresentation() {
  const WbVector3 &c = computedGlobalCenterOfMass();
  float position[3];
  c.toFloatArray(position);
  wr_transform_set_position(mGlobalCenterOfMassTransform, position);
  const WbWorldInfo *const worldInfo = WbWorld::instance()->worldInfo();
  const WbVector3 b[3] = {worldInfo->northVector(), worldInfo->upVector(), worldInfo->eastVector()};
  const WbPolygon &p = supportPolygon();
  mSupportPolygonRepresentation->draw(p, mY, c, b);
}

// Redraws the global center of mass of the solid after each physics step when required by the menu options
void WbSolid::refreshGlobalCenterOfMassRepresentation() {
  if (mSupportPolygonRepresentationIsEnabled)
    return;

  float position[3];
  computedGlobalCenterOfMass().toFloatArray(position);
  wr_transform_set_position(mGlobalCenterOfMassTransform, position);
}

// Redraws the center of buoyancy of the solid after each physics step when required by the menu options
void WbSolid::refreshCenterOfBuoyancyRepresentation() {
  connect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this, &WbSolid::resetImmersions,
          Qt::UniqueConnection);
  extractImmersions();
  if (mListOfImmersions.size() > 0) {
    updateCenterOfBuoyancy();
    wr_node_set_visible(WR_NODE(mCenterOfBuoyancyTransform), true);
    float position[3];
    mCenterOfBuoyancy.toFloatArray(position);
    wr_transform_set_position(mCenterOfBuoyancyTransform, position);
  } else
    wr_node_set_visible(WR_NODE(mCenterOfBuoyancyTransform), false);
}

unsigned char WbSolid::staticBalance() {
  const WbVector3 &c = computedGlobalCenterOfMass();
  const WbPolygon &p = supportPolygon();
  const WbWorldInfo *const wi = WbWorld::instance()->worldInfo();
  const double globalComX = c.dot(wi->northVector());
  const double globalComZ = c.dot(wi->eastVector());
  const bool stable = p.contains(globalComX, globalComZ);
  return stable;
}

void WbSolid::resetContactPointsAndSupportPolygon() {
  mGlobalListOfContactPoints.resize(0);
  mGlobalListOfContactPointDepths.resize(0);
  mSolidPerContactPoints.resize(0);
  mY = numeric_limits<double>::max();
  mSupportPolygonNeedsUpdate = true;
  mHasExtractedContactPoints = false;
}

void WbSolid::resetContactPoints() {
  mListOfContactPoints.resize(0);
  mGlobalListOfContactPoints.resize(0);
  mListOfContactPointDepths.resize(0);
  mGlobalListOfContactPointDepths.resize(0);
  mSolidPerContactPoints.resize(0);
  mHasExtractedContactPoints = false;
  disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this, &WbSolid::resetContactPoints);
}

void WbSolid::resetImmersions() {
  mListOfImmersions.resize(0);
  mHasExtractedImmersions = false;
  disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this, &WbSolid::resetImmersions);
}

void WbSolid::onSimulationModeChanged() {
  if (WbSimulationState::instance()->isFast() || !WbSimulationState::instance()->isRendering()) {
    if (mSupportPolygonRepresentation && !mSupportPolygonRepresentationIsEnabled) {
      deleteSupportPolygonRepresentation();
      disconnect(WbSimulationState::instance(), &WbSimulationState::physicsStepStarted, this,
                 &WbSolid::refreshSupportPolygonRepresentation);
    }
  }
}

void WbSolid::updateGraphicalGlobalCenterOfMass() {
  if (mIsKinematic)
    return;

  if (mSupportPolygonRepresentationIsEnabled || mGlobalCenterOfMassRepresentationIsEnabled) {
    float globalCenterOfMassRelativePosition[3];
    // Place center of mass relative to its position
    globalCenterOfMass().toFloatArray(globalCenterOfMassRelativePosition);
    wr_transform_set_position(mGlobalCenterOfMassTransform, globalCenterOfMassRelativePosition);
  }
}

void WbSolid::resetPhysicsIfRequired(bool changedFromSupervisor) {
  if (!changedFromSupervisor) {
    // For now, only the position modifications done by the user should reset the physics.
    resetPhysics();
  }

  WbViewpoint *viewpoint = WbWorld::instance()->viewpoint();
  if (viewpoint->followedSolid() == this)
    viewpoint->updateFollowSolidState();
}

// Collision and sleep flags management

void WbSolid::propagateBoundingObjectMaterialUpdate(bool onSelection) {
  // Recurses through all first level solid descendants
  foreach (WbSolid *const solid, mSolidChildren)
    solid->propagateBoundingObjectMaterialUpdate(onSelection);

  WbBaseNode *const bo = boundingObject();
  if (!bo)
    return;

  const bool isAsleep = isSleeping();
  const bool triggerChange = mBoundingObjectHasChanged || onSelection;

  // Update with current collision and sleep flags
  if (triggerChange) {
    if (isAsleep)
      bo->setSleepMaterial();
    else
      bo->updateCollisionMaterial(true, onSelection);

    updateSleepFlag();
    return;
  }

  const bool sleepHasChanged = mWasSleeping != isAsleep;
  mWasSleeping = isAsleep;

  // Optimized update with previous and current flags
  if (isAsleep) {
    if (sleepHasChanged)
      bo->setSleepMaterial();
    updateSleepFlag();
    return;
  } else
    bo->updateCollisionMaterial(sleepHasChanged, onSelection);

  updateSleepFlag();
}

void WbSolid::updateSleepFlag() {
  WbMatter::updateSleepFlag();
}

void WbSolid::displayWarning() {
  dBodyID b = bodyMerger();
  if (b) {
    dMass m;
    dBodyGetMass(b, &m);

    double inertialMatrixDiagonalMin = qMin(m.I[0], qMin(m.I[5], m.I[10]));
    double inertialMatrixDiagonalMax = qMax(m.I[0], qMax(m.I[5], m.I[10]));

    // To reproduce this, just play with a cylinder of h=0.125, r=0.0075
    // http://www.gamedev.net/topic/355556-problem-with-ode-and-small-numbers/

    if (inertialMatrixDiagonalMin > 0.0 &&
        // inertialMatrixDiagonalMax > 0.0 && // this is ensured
        inertialMatrixDiagonalMin < 1.0e-5 &&                         // light object : this threshold is empirical
        inertialMatrixDiagonalMax / inertialMatrixDiagonalMin > 15.0  // oblong object : this threshold is empirical
    )
      parsingWarn(tr("OmniSim has detected that this solid is light and oblong according to its inertia matrix. "
                     "This belongs in the physics edge cases, and can imply weird physical results. "
                     "Increasing the weight of the object or reducing its eccentricity are recommended."));
  }
}

/////////////////////////////////////////////
//  Collecting names of Solid descendants  //
/////////////////////////////////////////////

void WbSolid::collectSolidDescendantNames(QStringList &items, const WbSolid *const solidException) const {
  if (this != solidException)
    items << name();

  // Recurses through all first level solid descendants
  foreach (const WbSolid *const solid, mSolidChildren)
    solid->collectSolidDescendantNames(items, solidException);
}

//////////////////////////////////////////////////////////////////
//  Collecting kinematic hidden parameters of Solid descendants //
//////////////////////////////////////////////////////////////////

void WbSolid::collectHiddenKinematicParameters(HiddenKinematicParametersMap &map, int &counter) const {
  const bool merger = isSolidMerger();
  const WbVector3 *t = NULL;
  const WbRotation *r = NULL;
  const WbVector3 *l = NULL;
  const WbVector3 *a = NULL;
  bool copyTranslation = false;
  bool copyRotation = false;
  WbVector3 translationToBeCopied;
  WbRotation rotationToBeCopied;

  if (mSolidMerger == NULL || merger) {
    // TODO: implement an mIsVisible flag in WbNode for sake of efficiency
    const WbBasicJoint *parentJoint = jointParent();
    if (parentJoint) {
      // remove unquantified ODE effects on the endPoint Solid
      parentJoint->computeEndPointSolidPositionFromParameters(translationToBeCopied, rotationToBeCopied);
      // Note:
      //   This is an exception to the global double precision which is not sufficient here,
      //   because the accumulated error is big in computeEndPointSolidPositionFromParameters().
      //   cf. https://github.com/omichel/webots-dev/issues/6512
      if (!translationToBeCopied.almostEquals(translationFromFile(stateId()),
                                              100000.0 * std::numeric_limits<double>::epsilon()) &&
          !isTranslationFieldVisible())
        copyTranslation = true;
      if (!rotationToBeCopied.almostEquals(rotationFromFile(stateId()), 100000.0 * std::numeric_limits<double>::epsilon()) &&
          !isRotationFieldVisible())
        copyRotation = true;
    } else {
      if (translation() != translationFromFile(stateId()) && !isTranslationFieldVisible())
        t = &translation();
      if (rotation() != rotationFromFile(stateId()) && !isRotationFieldVisible())
        r = &rotation();
    }

    if (merger) {
      const dBodyID b = mSolidMerger->body();
      // P1.5 widening: hidden-field-save velocity reads via dispatcher.
      double lArray[3], aArray[3];
      WbPhysicsBackend *const odeBackend = WbPhysicsBackendRegistry::odeBackend();
      odeBackend->getBodyLinearVel(static_cast<WbBodyHandle>(b), lArray);
      odeBackend->getBodyAngularVel(static_cast<WbBodyHandle>(b), aArray);
      if (!WbMathsUtilities::isZeroVector3(lArray))
        l = new WbVector3(lArray);
      if (!WbMathsUtilities::isZeroVector3(aArray))
        a = new WbVector3(aArray);
    }
  }

  PositionMap positions;
  const int size = mJointChildren.size();
  for (int i = 0; i < size; ++i) {
    const WbJoint *const j = dynamic_cast<WbJoint *>(mJointChildren[i]);
    if (j) {
      WbVector3 v(NAN, NAN, NAN);

      // TODO: implement an mIsVisible flag in WbNode for sake of efficiency
      const WbJointParameters *const p = j->parameters();
      if ((p == NULL || !WbVrmlNodeUtilities::isVisible(p->findField("position"))) && j->position() != j->initialPosition())
        v[0] = j->position();

      if (j->nodeType() == WB_NODE_HINGE_2_JOINT || j->nodeType() == WB_NODE_BALL_JOINT) {
        const WbJointParameters *const p2 = j->parameters2();
        if ((p2 == NULL || !WbVrmlNodeUtilities::isVisible(p2->findField("position"))) &&
            j->position(2) != j->initialPosition(2))
          v[1] = j->position(2);
      }

      if (j->nodeType() == WB_NODE_BALL_JOINT) {
        const WbJointParameters *const p3 = j->parameters3();
        if ((p3 == NULL || !WbVrmlNodeUtilities::isVisible(p3->findField("position"))) &&
            j->position(3) != j->initialPosition(3))
          v[2] = j->position(3);
      }

      if (!std::isnan(v[0]) || !std::isnan(v[1]) || !std::isnan(v[2]))
        positions.insert(i, new WbVector3(v));
    }
  }

  PositionMap *const p = positions.size() > 0 ? new PositionMap(positions) : NULL;

  if (t || r || p || l || a || copyTranslation || copyRotation) {
    HiddenKinematicParameters *hkp = new HiddenKinematicParameters(t, r, p, l, a);
    if (copyTranslation)
      hkp->createTranslation(translationToBeCopied[0], translationToBeCopied[1], translationToBeCopied[2]);
    if (copyRotation)
      hkp->createRotation(rotationToBeCopied[0], rotationToBeCopied[1], rotationToBeCopied[2], rotationToBeCopied[3]);
    map.insert(counter, hkp);
  }

  ++counter;

  // Recurses through all first level solid descendants
  foreach (const WbSolid *const solid, mSolidChildren)
    solid->collectHiddenKinematicParameters(map, counter);
}

///////////////////
// Hidden fields //
///////////////////

void WbSolid::saveHiddenFieldValues() const {
  if (isSolidMerger()) {
    const dBodyID b = mSolidMerger->body();
    // P1.5 widening: velocity reads for hidden-field save via dispatcher.
    double l[3], a[3];
    WbPhysicsBackend *const odeBackend = WbPhysicsBackendRegistry::odeBackend();
    odeBackend->getBodyLinearVel(static_cast<WbBodyHandle>(b), l);
    odeBackend->getBodyAngularVel(static_cast<WbBodyHandle>(b), a);
    mLinearVelocity->setValue(l[0], l[1], l[2]);
    mAngularVelocity->setValue(a[0], a[1], a[2]);
  }
}

////////////////////////
//  Kinematic solids  //
////////////////////////

void WbSolid::enable(bool enabled, bool ode) {
  assert(mIsKinematic);

  wr_node_set_visible(WR_NODE(wrenNode()), enabled);

  if (ode) {
    const dGeomID g = odeGeom();
    if (g) {
      dSpaceID space = WbOdeContext::instance()->space();
      const bool hasSpace = dGeomGetSpace(g) != NULL;
      if (enabled) {
        if (!hasSpace)
          dSpaceAdd(space, g);
      } else if (hasSpace)
        dSpaceRemove(space, g);
    }
  }
}

void WbSolid::exportUrdfShape(WbWriter &writer, const QString &geometry, const WbPose *pose, const WbVector3 &offset) const {
  const QStringList element = QStringList() << "visual"
                                            << "collision";
  for (int j = 0; j < element.size(); ++j) {
    writer.increaseIndent();
    writer.indent();
    writer << QString("<%1>\n").arg(element[j]);
    writer.increaseIndent();
    if (pose != this || !offset.isNull()) {
      WbVector3 translation = pose->translation() + offset;
      WbRotation rotation = pose->rotation();
      writer.indent();
      if (pose == this) {
        rotation = WbRotation(0.0, 1.0, 0.0, 0.0);
        translation = offset;
      }
      writer << QString("<origin xyz=\"%1\" rpy=\"%2\"/>\n")
                  .arg(translation.toString(WbPrecision::FLOAT_ROUND_6))
                  .arg(rotation.toMatrix3().toEulerAnglesZYX().toString(WbPrecision::FLOAT_ROUND_6));
    }
    writer.indent();
    writer << "<geometry>\n";
    writer.increaseIndent();
    writer.indent();
    writer << geometry;
    writer.decreaseIndent();
    writer.indent();
    writer << "</geometry>\n";
    writer.decreaseIndent();
    writer.indent();
    writer << QString("</%1>\n").arg(element[j]);
    writer.decreaseIndent();
  }
}

bool WbSolid::exportNodeHeader(WbWriter &writer) const {
  if (writer.isUrdf()) {
    const bool ret = WbMatter::exportNodeHeader(writer);
    if (!ret) {
      if (boundingObject()) {
        QList<WbNode *> nodes = boundingObject()->subNodes(true);
        for (int i = 0; i < nodes.size(); ++i) {
          const WbNode *node = nodes[i];
          const WbCylinder *cylinder = dynamic_cast<const WbCylinder *>(node);
          const WbBox *box = dynamic_cast<const WbBox *>(node);
          const WbSphere *sphere = dynamic_cast<const WbSphere *>(node);
          const WbCapsule *capsule = dynamic_cast<const WbCapsule *>(node);
          if (box || cylinder || sphere || capsule) {
            const WbPose *pose = WbNodeUtilities::findUpperPose(node);
            QList<std::pair<QString, WbVector3>> geometries;  // string of the geometry and its offset

            if (box) {
              std::pair<QString, WbVector3> pair;
              pair.first = QString("<box size=\"%1 %2 %3\"/>\n").arg(box->size().x()).arg(box->size().y()).arg(box->size().z());
              geometries << pair;
            } else if (cylinder) {
              std::pair<QString, WbVector3> pair;
              pair.first = QString("<cylinder radius=\"%1\" length=\"%2\"/>\n").arg(cylinder->radius()).arg(cylinder->height());
              geometries << pair;
            } else if (capsule) {
              std::pair<QString, WbVector3> pair;
              pair.first = QString("<cylinder radius=\"%1\" length=\"%2\"/>\n").arg(capsule->radius()).arg(capsule->height());
              geometries << pair;
              pair.first = QString("<sphere radius=\"%1\"/>\n").arg(capsule->radius());
              pair.second = WbVector3(0.0, 0.5 * capsule->height(), 0.0);
              if (pose)
                pair.second = pose->rotation().toMatrix3() * pair.second;
              geometries << pair;
              pair.first = QString("<sphere radius=\"%1\"/>\n").arg(capsule->radius());
              pair.second = WbVector3(0.0, -0.5 * capsule->height(), 0.0);
              if (pose)
                pair.second = pose->rotation().toMatrix3() * pair.second;
              geometries << pair;
            } else if (sphere) {
              std::pair<QString, WbVector3> pair;
              pair.first = QString("<sphere radius=\"%1\"/>\n").arg(sphere->radius());
              geometries << pair;
            } else
              assert(false);
            for (int j = 0; j < geometries.size(); ++j)
              exportUrdfShape(writer, geometries[j].first, pose, geometries[j].second + writer.jointOffset());
          }
        }
      }
    }
    return ret;
  }

  return WbMatter::exportNodeHeader(writer);
}

void WbSolid::exportNodeFooter(WbWriter &writer) const {
  if (writer.isW3d() && boundingObject())
    boundingObject()->exportBoundingObjectToW3d(writer);

  WbMatter::exportNodeFooter(writer);
}
