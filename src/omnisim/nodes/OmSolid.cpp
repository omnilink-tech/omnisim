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

#include "OmSolid.hpp"

#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmElevationGrid.hpp"
#include "OmConnector.hpp"
#include "OmCylinder.hpp"
#include "OmDamping.hpp"
#include "OmField.hpp"
#include "OmGeometry.hpp"
#include "OmGroup.hpp"
#include "OmSFString.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmJoint.hpp"
#include "OmBasicJoint.hpp"
#include "OmHingeJoint.hpp"
#include "OmSliderJoint.hpp"
#include "OmMesh.hpp"
#include "OmTriangleMesh.hpp"
#include "OmTriangleMeshGeometry.hpp"
#include "OmJointParameters.hpp"
#include "OmLog.hpp"
#include "OmMFColor.hpp"
#include "OmMFNode.hpp"
#include "OmMFVector3.hpp"
#include "OmMassChecker.hpp"
#include "OmMathsUtilities.hpp"
#include "OmMatrix3.hpp"
#include "OmMatrix4.hpp"
#include "OmMatter.hpp"
#include "OmMotor.hpp"
#include "OmNodeOperations.hpp"
#include "OmNodeUtilities.hpp"
#include "OmNewtonBackend.hpp"
#include "OmPhysics.hpp"
#include "OmPhysicsBackend.hpp"
#include "OmPlane.hpp"
#include "OmPose.hpp"
#include "OmPropeller.hpp"
#include "OmResizeManipulator.hpp"
#include "OmRobot.hpp"
#include "OmRotation.hpp"
#include "OmShape.hpp"
#include "OmSimulationState.hpp"
#include "OmSlot.hpp"
#include "OmSolidMerger.hpp"
#include "OmSolidReference.hpp"
#include "OmSolidUtilities.hpp"
#include "OmSphere.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmTouchSensor.hpp"
#include "OmVacuumGripper.hpp"
#include "OmVector4.hpp"
#include "OmViewpoint.hpp"
#include "OmVrmlNodeUtilities.hpp"
#include "OmWorld.hpp"
#include "OmContactProperties.hpp"
#include "OmWorldInfo.hpp"
#include "OmWrenRenderingContext.hpp"


#include <QtCore/QFile>
#include <QtCore/QQueue>
#include <QtCore/QRegularExpression>

int OmSolid::recognitionColorSize() const {
  return mRecognitionColors->size();
}

const OmRgb &OmSolid::recognitionColor(int index) const {
  return mRecognitionColors->item(index);
}

bool OmSolid::isSolidMerger() const {
  return !mIsKinematic && mSolidMerger && mSolidMerger->solid() == this;
}

QPointer<OmSolidMerger> OmSolid::solidMerger() const {
  return mSolidMerger;
}

#include <QtCore/QStringList>

#include <cstdlib>
#include <cstring>

using namespace OmHiddenKinematicParameters;
using namespace OmSolidUtilities;
using namespace std;

const double OmSolid::MASS_ZERO_THRESHOLD = 1e-10;
const double REFERENCE_DENSITY = 1000.0;

QList<const OmSolid *> OmSolid::cSolids;

// See the header: bumped whenever a Solid could newly become registrable, so
// flushPendingNewtonRegistrations() can skip its whole-scene ancestor walk on
// the ticks where nothing has changed.
int OmSolid::cNewtonFlushGeneration = 0;

void OmSolid::init() {

  // Webots physics data
  mGlobalMass = 0.0;
  mGlobalVolume = 0.0;
  mGlobalCenterOfMass = OmVector3();
  mCenterOfMass = OmVector3();

  // Flags
  mWasSleeping = false;
  mBoundingObjectHasChanged = false;
  mSelected = false;
  mHasSearchedRobot = false;
  mHasExtractedContactPoints = false;
  mUseInertiaMatrix = false;
  mNativeInertiaValid = false;
  mIsPermanentlyKinematic = false;
  mIsKinematic = false;
  mUpdatedInStep = false;
  mResetPhysicsInStep = false;
  mKinematicWarningPrinted = false;
  mHasDynamicSolidDescendant = false;
  mNameClashResolved = false;

  // Newton dispatcher (cuda-newton-physics-plan.md P3.2): -1 means not
  // registered with OmNewtonBackend. Populated in postFinalize when
  // physicsBackend resolves to an available Newton backend.
  mNewtonBodyIndex = -1;
  mNewtonBodyIsStatic = false;
  mNewtonBodyIsKinematic = false;
  mNewtonKinPoseValid = false;
  for (int ki = 0; ki < 7; ++ki)
    mNewtonKinPushedPose[ki] = 0.0;

  // Merger
  mSolidMerger = NULL;
  mMergerIsSet = false;

  // Support polygon representation
  mY = numeric_limits<double>::max();
  mSupportPolygon = OmPolygon();
  mSupportPolygonNeedsUpdate = false;
  mSupportPolygonRepresentationIsEnabled = false;
  mSupportPolygonRepresentation = false;

  // Global center of mass representation
  mGlobalCenterOfMassRepresentationIsEnabled = false;

  // Center of buoyancy representation: flag only. Fluid immersion went with the ODE
  // backend, so nothing is ever immersed and the marker has no position to take.
  mCenterOfBuoyancyRepresentationIsEnabled = false;

  // user fields
  mContactMaterial = findSFString("contactMaterial");
  mPhysicsBackend = findSFString("physicsBackend");
  // May be NULL: derived nodes that do not redeclare Solid's field set (the
  // same situation physicsBackend documents at physicsBackendName()) simply do
  // not have it, and an absent field means "unset", not zero-by-accident.
  mNewtonClothCoupling = findSFInt("newtonClothCoupling");
  mBoundingObject = findSFNode("boundingObject");
  mPhysics = findSFNode("physics");
  mNewtonGravityCompensation = findSFDouble("newtonGravityCompensation");
  mNewtonFriction = findSFDouble("newtonFriction");
  mNewtonFrictionTorsional = findSFDouble("newtonFrictionTorsional");
  mNewtonFrictionRolling = findSFDouble("newtonFrictionRolling");
  mRadarCrossSection = findSFDouble("radarCrossSection");
  mRecognitionColors = findMFColor("recognitionColors");

  // hidden fields
  mLinearVelocity = findSFVector3("linearVelocity");
  mAngularVelocity = findSFVector3("angularVelocity");

  if (mLinearVelocity && mAngularVelocity) {
    updateIsLinearVelocityNull();
    updateIsAngularVelocityNull();
    connect(mLinearVelocity, &OmSFVector3::changed, this, &OmSolid::updateIsLinearVelocityNull);
    connect(mAngularVelocity, &OmSFVector3::changed, this, &OmSolid::updateIsAngularVelocityNull);
  }

  mOriginalHiddenKinematicParameters = NULL;
}

OmSolid::OmSolid(OmTokenizer *tokenizer) : OmMatter("Solid", tokenizer) {
  init();
}

OmSolid::OmSolid(const OmSolid &other) : OmMatter(other) {
  init();
}

OmSolid::OmSolid(const OmNode &other) : OmMatter(other) {
  init();
}

OmSolid::OmSolid(const QString &modelName, OmTokenizer *tokenizer) : OmMatter(modelName, tokenizer) {
  init();
}

OmSolid::~OmSolid() {
  if (mRadarCrossSection->value() > 0.0)
    OmWorld::instance()->removeRadarTarget(this);

  if (!mRecognitionColors->isEmpty())
    OmWorld::instance()->removeCameraRecognitionObject(this);

  qDeleteAll(mHiddenKinematicParametersMap);
  mHiddenKinematicParametersMap.clear();

  cSolids.removeAll(this);

  // A Solid is going away, so anything caching OmSolid* by Newton body index
  // must rebuild. The per-step native-contact snapshot in extractContactPoints()
  // is keyed on (world, backend, time, this generation), and NONE of the first
  // three changes when a node is removed mid-step: POST /scene/delete defaults
  // to settle_steps 0, so the clock does not advance and the snapshot stays
  // "valid" while holding a pointer to this freed object. The next contact query
  // in the same step would then resolve a stale body index to it and dereference
  // it (sA->topSolid()). Newton keeps reporting contacts for a deleted body --
  // there is no remove path from a deleted Solid to the MuJoCo model -- so the
  // stale entry is likely to be HIT, not merely present.
  //
  // Symmetric with the preFinalize() bump for spawns. Bumping only ever
  // invalidates a cache, so the worst case is one extra rebuild.
  bumpNewtonFlushGeneration();

  if (isSolidMerger())
    delete mSolidMerger.data();

  if (!mSolidMerger.isNull())
    mSolidMerger->removeSolid(this);

  // cleanup ODE

  // disconnecting descendants
  foreach (const OmSolid *const solid, mSolidChildren)
    disconnect(solid, &OmSolid::destroyed, this, 0);
}

void OmSolid::deleteAllSolids() {
  foreach (OmSolid *const solid, mSolidChildren)
    OmNodeOperations::instance()->deleteNode(solid);
  mSolidChildren.clear();
}

void OmSolid::validateProtoNode() {
  if (isProtoInstance()) {
    bool checkTranslation = !isTranslationFieldVisible();
    bool checkRotation = !isRotationFieldVisible();
    if (!(checkTranslation || checkRotation))
      return;

    foreach (const OmField *parameter, parameters()) {
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

void OmSolid::downloadAssets() {
  OmGroup::downloadAssets();
  if (boundingObject())
    boundingObject()->downloadAssets();
}

void OmSolid::preFinalize() {
  mHasNoSolidAncestor = false;

  cSolids << this;

  updateChildren();

  OmMatter::preFinalize();

  if (physics())
    physics()->preFinalize();
  else
    mIsKinematic = true;

  if (mBoundingObject->value())
    boundingObject()->preFinalize();

  setMatrixNeedUpdate();  // force the matrix update after the first ode update

  // needed to be done before createOdeObjects
  // because of the SolidMerger
  mIsPermanentlyKinematic = OmSolidUtilities::isPermanentlyKinematic(this);  // cached because it can't be called in destructor
  if (mIsPermanentlyKinematic)
    mIsKinematic = true;

  // Overwrites loaded values with hidden field and hidden parameter values (translation, rotation, joint position), reads
  // initial velocities
  if (isProtoInstance() && !isNestedProtoNode()) {
    int counter = 0;
    if (!restoreHiddenKinematicParameters(mHiddenKinematicParametersMap, counter)) {
      bool success = resetHiddenKinematicParameters();
      if (success)
        OmLog::instance()->warning(tr("PROTO '%1' changed after this world was saved:\n"
                                      "hidden parameters have been automatically reset.\n\n"
                                      "Please save the current world to get rid of this message.")
                                     .arg(modelName()),
                                   true);
      else
        OmLog::instance()->warning(tr("PROTO '%1' changed after this world was saved:\n"
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
    // Spot). At 40+ huskies the accumulated OmLog::warning cost stalls
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

bool OmSolid::restoreHiddenKinematicParameters(const HiddenKinematicParametersMap &map, int &counter) {
  if (!applyHiddenKinematicParameters(map.value(counter, NULL), true))
    return false;

  ++counter;

  foreach (OmSolid *const solid, mSolidChildren) {
    if (!solid->restoreHiddenKinematicParameters(map, counter))
      return false;
  }

  return true;
}

bool OmSolid::resetHiddenKinematicParameters() {
  foreach (OmSolid *const solid, mSolidChildren) {
    if (!solid->resetHiddenKinematicParameters())
      return false;
  }

  if (mOriginalHiddenKinematicParameters)
    return applyHiddenKinematicParameters(mOriginalHiddenKinematicParameters, false);

  return true;
}

bool OmSolid::applyHiddenKinematicParameters(const HiddenKinematicParameters *hkp, bool backupPrevious) {
  if (!hkp)
    return true;

  OmVector3 *previousT = NULL;
  OmRotation *previousR = NULL;
  OmVector3 *previousL = NULL;
  OmVector3 *previousA = NULL;
  PositionMap *previousP = NULL;

  const OmVector3 *const t = hkp->translation();
  if (t) {
    if (backupPrevious)
      previousT = new OmVector3(translation());
    OmPose::setTranslation(*t);
  }

  const OmRotation *const r = hkp->rotation();
  if (r) {
    if (backupPrevious)
      previousR = new OmRotation(rotation());
    OmPose::setRotation(*r);
  }

  const PositionMap *const m = hkp->positions();
  if (m) {
    if (backupPrevious)
      previousP = new PositionMap();

    const PositionMap::const_iterator end = m->constEnd();
    for (PositionMap::const_iterator i = m->constBegin(); i != end; ++i) {
      const OmVector3 *const p = i.value();
      if (!p)
        return false;
      const int jointIndex = i.key();
      assert(jointIndex < mJointChildren.length());
      OmJoint *const j = dynamic_cast<OmJoint *>(mJointChildren.at(jointIndex));
      if (!j)
        return false;

      if (backupPrevious) {
        OmVector3 v(NAN, NAN, NAN);
        const OmJointParameters *const param1 = j->parameters();
        if (param1)
          v[0] = j->position();
        const OmJointParameters *const param2 = j->parameters2();
        if (param2)
          v[1] = j->position(2);
        const OmJointParameters *const param3 = j->parameters3();
        if (param3)
          v[2] = j->position(3);
        previousP->insert(jointIndex, new OmVector3(v));
      }

      for (int k = 0; k < 3; ++k) {
        const double posk = (*p)[k];
        if (!std::isnan(posk))
          j->setPosition(posk, k + 1);
      }
    }
  }

  const OmVector3 *const l = hkp->linearVelocity();
  if (l) {
    if (backupPrevious)
      previousL = new OmVector3(mLinearVelocity->value());
    mLinearVelocity->setValue(*l);
  }

  const OmVector3 *const a = hkp->angularVelocity();
  if (a) {
    if (backupPrevious)
      previousA = new OmVector3(mAngularVelocity->value());
    mAngularVelocity->setValue(*a);
  }

  if (backupPrevious && (previousT || previousR || previousP || previousL || previousA)) {
    delete mOriginalHiddenKinematicParameters;
    mOriginalHiddenKinematicParameters = new HiddenKinematicParameters(previousT, previousR, previousP, previousL, previousA);
  }

  return true;
}

void OmSolid::postFinalize() {
  delete mOriginalHiddenKinematicParameters;
  mOriginalHiddenKinematicParameters = NULL;

  OmMatter::postFinalize();
  if (physics())
    physics()->postFinalize();

  // Trigger backend resolution at world-load time (registry call_once; the
  // Newton runtime is normally already preloaded). Actual registration is
  // deferred to flushPendingNewtonRegistrations() (driven by
  // OmSimulationWorld::step before finalizeWorld) so that matrix() returns
  // correct world coordinates -- at this point the parent transform chain may
  // not yet be computed for nested Solids (e.g. wheels under a HingeJoint).
  (void)physicsBackend();

  // A new Solid exists, so the registration flush must look again -- this is
  // the hook that keeps the flush's skip-gate honest for mid-run spawns
  // (/scene/spawn and every supervisor import land here).
  bumpNewtonFlushGeneration();

  updateDynamicSolidDescendantFlag();

  connect(mTranslation, &OmSFVector3::changedByUser, this, &OmSolid::resetPhysicsIfRequired);
  connect(mRotation, &OmSFVector3::changedByUser, this, &OmSolid::resetPhysicsIfRequired);

  disconnectFieldNotification(rotationFieldValue());
  disconnectFieldNotification(translationFieldValue());
  connect(OmSimulationState::instance(), &OmSimulationState::modeChanged, this, &OmSolid::onSimulationModeChanged);
  connect(OmSimulationState::instance(), &OmSimulationState::renderingStateChanged, this, &OmSolid::onSimulationModeChanged);
  connect(this, &OmSolid::massPropertiesChanged, this, &OmSolid::displayWarning);
  connect(mPhysics, &OmSFNode::changed, this, &OmSolid::updatePhysics);
  connect(mRadarCrossSection, &OmSFDouble::changed, this, &OmSolid::updateRadarCrossSection);
  connect(mRecognitionColors, &OmMFColor::itemChanged, this, &OmSolid::updateRecognitionColors);
  connect(mRecognitionColors, &OmMFColor::itemRemoved, this, &OmSolid::updateRecognitionColors);
  connect(mRecognitionColors, &OmMFColor::itemInserted, this, &OmSolid::updateRecognitionColors);

  if (isTopSolid()) {
    updateGlobalCenterOfMass();
    updateGlobalVolume();
  }

  displayWarning();

  if (mRadarCrossSection->value() > 0.0)
    OmWorld::instance()->addRadarTarget(this);

  if (!mRecognitionColors->isEmpty())
    OmWorld::instance()->addCameraRecognitionObject(this);

  if (protoParameterNode()) {
    const QVector<OmNode *> nodes = protoParameterNode()->protoParameterNodeInstances();
    if (nodes.size() > 1 && nodes.at(0) == this)
      parsingWarn(tr("Solid node defined in PROTO field is used multiple times. "
                     "OmniSim doesn't fully support this because the multiple node instances cannot be identical."));
  }
}

void OmSolid::resolveNameClashIfNeeded(bool automaticallyChange, bool recursive, const QList<OmSolid *> &siblings,
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
    const OmNode *parameterNode = protoParameterNode();
    while (parameterNode && parameterNode->protoParameterNode())
      parameterNode = parameterNode->protoParameterNode();
    const OmNode *visibleNode = parameterNode ? parameterNode : this;

    bool found = false;
    re.setPattern(QString("%1\\((\\d+)\\)").arg(QRegularExpression::escape(nameWithoutIndex)));
    foreach (const OmSolid *s, siblings) {
      if (!s || s == this)
        continue;

      const bool matchingName = s->name() == name();
      found |= matchingName;
      if (matchingName) {
        if (parameterNode != NULL) {
          // ensure that solid nodes doesn't refer to the same PROTO parameter node
          // otherwise we will loop forever
          const OmNode *otherParameterNode = s->protoParameterNode();
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
        OmField *nameField = findField("name", true);
        bool isTemplateRegenerator = false;
        while (nameField && !isTemplateRegenerator) {
          isTemplateRegenerator = OmNodeUtilities::isTemplateRegeneratorField(nameField);
          nameField = nameField->parameter();
        }
        if (isTemplateRegenerator)
          visibleNode->parsingWarn(
            warningText +
            tr(" A unique name cannot be automatically generated because 'name' is a template regenerator field."));
        else if (!OmVrmlNodeUtilities::isVisible(findField("name")))
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
    QList<OmSolid *> solidChildrenList = mSolidChildren.toList();
    foreach (OmSolid *s, solidChildrenList)
      s->resolveNameClashIfNeeded(automaticallyChange, recursive, solidChildrenList, NULL);
  }
}

void OmSolid::updateName() {
  if (!mNameClashResolved) {
    const OmSolid *us = upperSolid();
    resolveNameClashIfNeeded(false, false, us ? us->solidChildren().toList() : OmWorld::instance()->topSolids(), NULL);
  } else
    // name field has just been updated in a previous call of resolveNameClashIfNeeded
    mNameClashResolved = false;
  OmMatter::updateName();
}

QString OmSolid::computeUniqueName() const {
  const OmSolid *solid = this;
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

OmSolid *OmSolid::findDescendantSolidFromUniqueName(QStringList &names) const {
  const OmSolid *solid = this;
  while (solid && !names.isEmpty()) {
    QString name = names.takeFirst();
    name.replace("\\:", ":");    // revert escape of ':'
    name.replace("\\\\", "\\");  // revert escape of '\'
    OmSolid *nextSolid = NULL;
    foreach (OmSolid *s, solid->mSolidChildren) {
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

OmSolid *OmSolid::findSolidFromUniqueName(const QString &name) {
  // Solid names joined by ':'
  QStringList names = splitUniqueNamesByEscapedPattern(name, ":");
  QString topName = names.takeFirst();
  topName.replace("\\:", ":");    // revert escape of ':'
  topName.replace("\\\\", "\\");  // revert escape of '\'
  foreach (OmSolid *solid, OmWorld::instance()->topSolids()) {
    if (solid->name() == topName)
      return names.isEmpty() ? solid : solid->findDescendantSolidFromUniqueName(names);
  }
  return NULL;
}

QStringList OmSolid::splitUniqueNamesByEscapedPattern(const QString &text, const QString &pattern) {
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

void OmSolid::createWrenObjects() {
  // D1.4: the WREN centre-of-mass crosses + support-polygon visuals died with WREN; the wgpu
  // overlay path (OmWgpuView's COM collectors) draws from mCenterOfMass/mGlobalCenterOfMass,
  // which stay maintained below.
  OmMatter::createWrenObjects();
}

////////////////////////////
//   Create ODE Objects   //
////////////////////////////

void OmSolid::setSolidMerger() {
  if (mIsKinematic) {
    mSolidMerger = NULL;
    return;
  }

  const OmSolid *const us = jointParent() ? NULL : upperSolid();
  const bool inherit = us && us->physics() && name().compare("right wheel", Qt::CaseInsensitive) != 0 &&
                       name().compare("left wheel", Qt::CaseInsensitive) != 0;
  mSolidMerger = inherit ? us->solidMerger() : QPointer<OmSolidMerger>(new OmSolidMerger(this));
}

void OmSolid::setJointParents() {

  // new joints
  typedef QList<OmBasicJoint *>::const_iterator LCI;
  LCI end = mJointParents.constEnd();
  for (LCI it = mJointParents.constBegin(); it != end; ++it)
    (*it)->setJoint();
}

void OmSolid::setupSolidMerger() {
  // Detaches the solid if it was previously merged
  if (isSolidMerger()) {
    setJointParents();
    return;
  }

  if (mSolidMerger)
    mSolidMerger->removeSolid(this);

  // Sets the new solid merger
  setSolidMerger();

  if (mSolidMerger) {
    assert(isDynamic());  // At this point mSolidMerger == NULL if mIsKinematic == false
    if (!mNativeInertiaValid)
      createOdeMass();  // computes the native inertia mirror (the Newton inertia feed)
    mSolidMerger->appendSolid(this);
    if (mSolidMerger->isSet())
      mSolidMerger->mergeMass(this, false);
  }
}

// Recursive method that sets solid mergers, creates masses and attaches dGeoms from top to bottom
void OmSolid::setupSolidMergers() {
  setupSolidMerger();
  // Recurses through all first level solid descendants
  foreach (OmSolid *const solid, mSolidChildren)
    solid->setupSolidMergers();

  mMergerIsSet = true;
}

// Recursive method that sets children joints with referenced endpoints
void OmSolid::setJointChildrenWithReferencedEndpoint() {
  foreach (OmBasicJoint *const j, mJointChildren)
    if (j->solidReference()) {
      j->updateEndPoint();
      j->setJoint();
    }

  foreach (OmSolid *const solid, mSolidChildren)
    solid->setJointChildrenWithReferencedEndpoint();
}

void OmSolid::createOdeObjects() {
  if (boundingObject())
    boundingObject()->createOdeObjects();

  if (isTopLevel() || !mMergerIsSet) {  // the second condition is for newly inserted solids only
    setupSolidMergers();                // this recursion sets solid mergers but also creates dGeoms, dMasses
    setBodiesAndJointsToParents();      // this recursion sets bodies positions and joints to parents
    setJointChildrenWithReferencedEndpoint();
  }

  // Recurses through solid descendants
  OmPose::createOdeObjects();
}

// Sets recursively every ODE object which was not set during solid merger settings, i.e. bodies and joints to parents
void OmSolid::setBodiesAndJointsToParents() {
  assert(mMergerIsSet);
  if (isDynamic()) {
    if (isSolidMerger())
      mSolidMerger->setupOdeBody();
    const OmPhysics *const p = physics();
    connect(p, &OmPhysics::massOrDensityChanged, this, &OmSolid::updateOdeMass, Qt::UniqueConnection);
    connect(p, &OmPhysics::massOrDensityChanged, OmMassChecker::instance(), &OmMassChecker::checkMasses, Qt::UniqueConnection);
    connect(p, &OmPhysics::centerOfMassChanged, this, &OmSolid::updateOdeCenterOfMass, Qt::UniqueConnection);
    connect(p, &OmPhysics::inertialPropertiesChanged, this, &OmSolid::updateOdeInertiaMatrix, Qt::UniqueConnection);
    connect(p, &OmPhysics::dampingChanged, this, &OmSolid::updateOdeDamping, Qt::UniqueConnection);
  }

  // Recurses through solid descendants
  foreach (OmSolid *const solid, mSolidChildren)
    solid->setBodiesAndJointsToParents();

  OmBasicJoint *const pj = jointParent();
  if (pj)
    pj->updateAfterParentPhysicsChanged();  // needed also in kinematic mode

  if (isSolidMerger()) {
    if (pj)
      pj->setJoint();
  }
}

// Reset ODE joints (with no position offset) for every solid linked to this one
void OmSolid::resetJointsToLinkedSolids() {
  assert(mMergerIsSet);
  resetJointPositions(true);

  foreach (OmSolid *const solid, mSolidChildren)
    solid->resetJointPositions(true);

  foreach (OmBasicJoint *const j, mJointChildren)
    if (j->solidReference())
      j->resetJointPositions();
}

// Reset ODE joints (with no position offset) for every solid linked to this one or to one of its descendants
void OmSolid::resetJoints() {
  if (isSolidMerger())
    resetJointPositions(true);

  foreach (OmSolid *const solid, mSolidChildren)
    solid->resetJoints();
}

void OmSolid::createOdeGeomFromInsertedGroupItem(OmBaseNode *node) {
  assert(node);

  if (!createOdeGeomFromNode(node))  // if the inserted node has no Geometry child or it has an indexed face set which is invalid
    return;

  if (isDynamic()) {
    assert(mSolidMerger);
    adjustOdeMass();
  }
}

// Methods modifying the mass
void OmSolid::updateTopSolidGlobalMass() const {
  OmSolid *const ts = topSolid();
  if (ts && ts->isPostFinalizedCalled()) {
    ts->updateGlobalCenterOfMass();
    ts->updateGlobalVolume();
  }
}

// Method correcting the ODE dMass of the OmSolid after insertion or deletion of a bounding OmGeometry; it is also called when
// the density and the mass field change
void OmSolid::adjustOdeMass(bool mergeMass) {
  // ODE is gone: there is no dMass to adjust (the native mirror is
  // recomputed by createOdeMass); keep only the scene-level side effects the
  // ON path produces (CoM refresh, global-mass rollup, change signal).
  (void)mergeMass;
  if (mSolidMerger != NULL)
    updateCenterOfMass();
  updateTopSolidGlobalMass();
  emit massPropertiesChanged();
}

void OmSolid::addMassFromInsertedNode(OmBaseNode *node) {  // node is a OmGeometry or a OmPose
  assert(isDynamic() && mSolidMerger);
  adjustOdeMass();
}

void OmSolid::removeBoundingGeometry() {
  if (isBeingDeleted())
    return;

  // ODE is gone: geometry dMasses are never allocated; force the native
  // mirror to be recomputed on the next merger pass instead.
  if (isDynamic())
    mNativeInertiaValid = false;
}

void OmSolid::printKinematicWarningIfNeeded() {
  if (mKinematicWarningPrinted || !mHasDynamicSolidDescendant || !belongsToStaticBasis())
    return;

  mKinematicWarningPrinted = true;
  parsingWarn(tr("This node is controlled in kinematics mode "
                 "but some Solid descendant nodes have physics and won't move along with this node. Either add a Physics node to this Solid so the whole chain is simulated, or remove the descendants' Physics nodes so they follow it kinematically -- see docs/reference/solid.md."));
}

OmVector3 OmSolid::relativeLinearVelocity(const OmSolid *parentSolid) const {
  OmVector3 l = isDynamic() ? solidMerger()->solid()->linearVelocity() : linearVelocity();

  const OmSolid *solid = this;
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

OmVector3 OmSolid::relativeAngularVelocity(const OmSolid *parentSolid) const {
  OmVector3 a = isDynamic() ? solidMerger()->solid()->angularVelocity() : angularVelocity();

  const OmSolid *solid = this;
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

void OmSolid::setLinearVelocity(const double velocity[3]) {
  mLinearVelocity->setValue(velocity[0], velocity[1], velocity[2]);
  if (isSolidMerger()) {
    if (!setNewtonBodyVel(velocity, false)) {
      // FIX 5 (t=0 setVelocity drop): not Newton-routable YET (registration
      // has not run). Cache for replay at Newton registration. Last write wins.
      mPendingNewtonLinVel[0] = velocity[0];
      mPendingNewtonLinVel[1] = velocity[1];
      mPendingNewtonLinVel[2] = velocity[2];
      mPendingNewtonLinVelValid = true;
    }
  }
  printKinematicWarningIfNeeded();
}

void OmSolid::setAngularVelocity(const double velocity[3]) {
  mAngularVelocity->setValue(velocity[0], velocity[1], velocity[2]);
  if (isSolidMerger()) {
    if (!setNewtonBodyVel(velocity, true)) {
      // FIX 5: see setLinearVelocity above.
      mPendingNewtonAngVel[0] = velocity[0];
      mPendingNewtonAngVel[1] = velocity[1];
      mPendingNewtonAngVel[2] = velocity[2];
      mPendingNewtonAngVelValid = true;
    }
  }
  printKinematicWarningIfNeeded();
}

void OmSolid::updateIsLinearVelocityNull() {
  mIsLinearVelocityNull = mLinearVelocity->value().isNull();
}

void OmSolid::updateIsAngularVelocityNull() {
  mIsAngularVelocityNull = mAngularVelocity->value().isNull();
}

void OmSolid::appendJointParent(OmBasicJoint *joint) {
  mJointParents.append(joint);
}

void OmSolid::removeJointParent(OmBasicJoint *joint) {
  mJointParents.removeOne(joint);
}

void OmSolid::setGeomMatter(OmBaseNode *node) {
  if (mSolidMerger)
    addMassFromInsertedNode(node);
}
/////////////////////
// Update Methods  //
/////////////////////

// Resets recursively ODE dGeoms positions, dBodies and joints starting from *this
void OmSolid::handleJerk() {
  jerk(false);
  if (!belongsToStaticBasis())
    awake();
  else
    OmWorld::instance()->awake();
}

void OmSolid::updateTranslation() {
  OmMatter::updateTranslation();
  syncNewtonPoseFromFields();

  if (!mJointParents.isEmpty() && OmSimulationState::instance()->isPaused())
    emit positionChangedArtificially();
}

void OmSolid::updateRotation() {
  OmMatter::updateRotation();
  syncNewtonPoseFromFields();

  if (!mJointParents.isEmpty() && OmSimulationState::instance()->isPaused())
    emit positionChangedArtificially();
}

// Bridge a Supervisor-driven translation/rotation write into Newton's
// body_q so a controller calling reset_episode() actually relocates
// the Newton body. Without this, ODE moves but Newton stays where it
// was, and over many resets the two state machines drift until the
// constraint solver fails.
void OmSolid::syncNewtonPoseFromFields() {
  // OMNISIM_NEWTON_KINEMATIC: a pose write landing on a physics-less
  // (kinematic) subtree is a KINEMATIC MOVE -- a supervisor teleporting a
  // prop, the engine's kinematic joint FK (OmHingeJoint::updatePosition ->
  // setTranslationAndRotation), or velocity integration (prePhysicsStep's
  // translate/rotate). Push it into the Newton mocap bodies (this Solid's
  // own, plus every registered descendant whose world pose moved with it
  // even though its own fields did not change) instead of dropping it.
  // Guarded so a physics-less Solid that nevertheless registered a DYNAMIC
  // Newton body (physics-bearing fixed child rolled up into it) keeps the
  // resetBodyPose path below unchanged.
  if (newtonKinematicNativeEnabled() && mIsKinematic &&
      (mNewtonBodyIndex < 0 || mNewtonBodyIsStatic || mNewtonBodyIsKinematic)) {
    pushNewtonKinematicSubtreePose();
    return;
  }
  if (mNewtonBodyIndex < 0)
    return;
  // P8.2: a static (mass=0, pinned) Newton body has no joints and never
  // moves, so there is nothing to sync -- and critically it must NOT
  // call resetJointsToDefaults() below (it's a top-level/root Solid, so
  // it would otherwise clobber the shared articulation every tick, the
  // exact mechanism of the chassis-freeze bug).
  if (mNewtonBodyIsStatic)
    return;
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;  // refusal lives in OmSimulationWorld::step -- see refuseIfNewtonBroken
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;
  const OmVector3 t = matrix().translation();
  const OmQuaternion q = OmRotation(rotationMatrix()).toQuaternion();
  newton->resetBodyPose(mNewtonBodyIndex, t.x(), t.y(), t.z(),
                        q.x(), q.y(), q.z(), q.w());
  // For the ROOT solid only, also reset all joint angles to defaults
  // and re-FK the chain. Doing this once per chassis reset is enough
  // -- the descendant Solids' body_q gets recomputed via FK from the
  // freshly-zeroed joint_q values.
  if (upperPose() == nullptr)
    newton->resetJointsToDefaults();
}

// OMNISIM_NEWTON_KINEMATIC: push this Solid's engine-computed world pose into
// its Newton mocap body. This is the ONLY write path a kinematic/static body
// takes -- never resetBodyPose (which zeroes velocity and dirties the MuJoCo
// copy-in) and never resetJointsToDefaults (the chassis-freeze mechanism).
// Dirty-tracked so the per-step prePhysicsStep refresh costs one pose compare
// for a body that did not move.
void OmSolid::pushNewtonKinematicPose() {
  if (!newtonKinematicNativeEnabled() || mNewtonBodyIndex < 0 ||
      !(mNewtonBodyIsStatic || mNewtonBodyIsKinematic))
    return;
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);
  if (!newton->isWorldRunning())
    return;
  const OmVector3 t = matrix().translation();
  const OmQuaternion q = OmRotation(rotationMatrix()).toQuaternion();
  const double pose[7] = {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
  if (mNewtonKinPoseValid) {
    bool same = true;
    for (int i = 0; i < 7; ++i)
      if (pose[i] != mNewtonKinPushedPose[i]) {
        same = false;
        break;
      }
    if (same)
      return;
  }
  if (newton->setKinematicPose(mNewtonBodyIndex, pose[0], pose[1], pose[2],
                               pose[3], pose[4], pose[5], pose[6]) == 0) {
    for (int i = 0; i < 7; ++i)
      mNewtonKinPushedPose[i] = pose[i];
    mNewtonKinPoseValid = true;
  }
}

// OMNISIM_NEWTON_KINEMATIC: recursive form -- a field write lands on ONE
// Solid, but the world pose of every descendant moved with it (nested PROTO
// colliders registered on child Solids, deeper links of a kinematic chain)
// while their own fields stayed put. Walks fixed sub-solids AND joint
// endpoints; only static/kinematic-flagged bodies are pushed, so descending
// through a dynamic descendant is harmless (its body is solver-owned).
void OmSolid::pushNewtonKinematicSubtreePose() {
  if (!newtonKinematicNativeEnabled())
    return;
  pushNewtonKinematicPose();
  for (OmSolid *const c : mSolidChildren)
    c->pushNewtonKinematicSubtreePose();
  for (OmBasicJoint *const j : mJointChildren) {
    OmSolid *const ep = j->solidEndPoint();
    if (ep != nullptr)
      ep->pushNewtonKinematicSubtreePose();
  }
}

// Creates and updates, or destroys, the ODE dBody according to the existence of a OmPhysics node
void OmSolid::updatePhysics() {
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

  if (mPhysics->value())
    mNativeInertiaValid = false;  // force recomputing the native mirror
  setupSolidMergers();
  setBodiesAndJointsToParents();
  setJointChildrenWithReferencedEndpoint();

  adjustOdeMass();
  refreshPhysicsRepresentation();

  if (previousKinematic != mIsKinematic)
    updateDynamicSolidDescendantFlag();
}

void OmSolid::updateRadarCrossSection() {
  if (mRadarCrossSection->value() > 0.0) {
    if (!OmWorld::instance()->radarTargetSolids().contains(this))
      OmWorld::instance()->addRadarTarget(this);
  } else if (OmWorld::instance()->radarTargetSolids().contains(this))
    OmWorld::instance()->removeRadarTarget(this);
}

void OmSolid::updateRecognitionColors() {
  OmRgb segmentationColor(0.0, 0.0, 0.0);
  if (!mRecognitionColors->isEmpty()) {
    if (!OmWorld::instance()->cameraRecognitionObjects().contains(this))
      OmWorld::instance()->addCameraRecognitionObject(this);
    segmentationColor = mRecognitionColors->item(0);
  } else if (OmWorld::instance()->cameraRecognitionObjects().contains(this))
    OmWorld::instance()->removeCameraRecognitionObject(this);

  // set segmentation color in child nodes
  OmGroup::updateSegmentationColor(segmentationColor);
}

void OmSolid::updateSegmentationColor(const OmRgb &color) {
  // apply segmentation color from parent node if needed
  if (!mRecognitionColors->isEmpty())
    // this node already defines different recognitionColors
    return;

  OmGroup::updateSegmentationColor(color);
}

void OmSolid::updateOdeMass() {
  assert(isDynamic());

  const OmPhysics *const p = physics();
  if (!p->hasApositiveMassOrDensity())
    return;

  p->checkMassAndDensity();

  if (mUseInertiaMatrix && p->mode() != OmPhysics::CUSTOM_INERTIA_MATRIX)
    applyToOdeMass();  // the inertia computation mode has changed
  else
    adjustOdeMass();

  awake();
  refreshPhysicsRepresentation();
}

void OmSolid::setOdeInertiaMatrix() {
  assert(isDynamic() && physics()->mode() == OmPhysics::CUSTOM_INERTIA_MATRIX);
  const OmPhysics *const p = physics();
  mUseInertiaMatrix = true;
  // ODE is gone: the Newton registration reads the inertiaMatrix /
  // centerOfMass fields directly; only refresh the cached CoM here.
  (void)p;
  updateCenterOfMass();
  emit massPropertiesChanged();
}

void OmSolid::updateOdeInertiaMatrix() {
  assert(isDynamic() && physics()->mode() == OmPhysics::CUSTOM_INERTIA_MATRIX);

  setOdeInertiaMatrix();
  mSolidMerger->mergeMass(this);
  awake();

  refreshPhysicsRepresentation();
}

void OmSolid::setInertiaMatrixFromBoundingObject() {
  // This GUI action ran ODE's dMass integrator, which has been removed. Refuse
  // loudly instead of writing a fabricated matrix.
  OmLog::warning(tr("'%1': computing an inertia matrix from the bounding object is unavailable -- it ran ODE's "
                    "mass integrator, and ODE has been removed. The inertiaMatrix fields were NOT modified.")
                   .arg(name()));
}

void OmSolid::updateOdeCenterOfMass() {
  assert(isDynamic() && mSolidMerger);
  updateCenterOfMass();

  applyToOdeMass();
  mSolidMerger->setGeomAndBodyPositions(true);  // reset also joints passing through this solid
  awake();

  refreshPhysicsRepresentation();
}

void OmSolid::updateOdeDamping() {
  assert(isDynamic() && mSolidMerger);
  mSolidMerger->setOdeDamping();
  awake();
}

void OmSolid::updateBoundingObject() {
  if (mBoundingObject->value() != NULL) {
    OmBaseNode *node = dynamic_cast<OmBaseNode *>(mBoundingObject->value());
    assert(node);
    if (!isBoundingObjectFinalizationCompleted(node))
      // postpone bounding object update after finalization
      return;

  }

  mBoundingObjectHasChanged = true;
  refreshPhysicsRepresentation();
}

// Updates of children nodes

void OmSolid::collectSolidChildren(const OmGroup *group, bool connectSignals, QVector<OmSolid *> &solidChildren,
                                   QVector<OmBasicJoint *> &jointChildren, QVector<OmPropeller *> &propellerChildren) {
  const OmMFNode *const ch = group->childrenField();
  if (connectSignals) {
    connect(ch, &OmMFNode::changed, this, &OmSolid::updateChildren, Qt::UniqueConnection);
    connect(group, &OmGroup::finalizedChildAdded, this, &OmSolid::refreshPhysicsRepresentation, Qt::UniqueConnection);
    connect(group, &OmGroup::finalizedChildAdded, this, &OmSolid::updateTopSolidGlobalMass, Qt::UniqueConnection);
  }
  OmMFNode::Iterator it(ch);
  while (it.hasNext()) {
    OmNode *const n = it.next();

    // cppcheck-suppress constVariablePointer
    OmSolid *const solid = dynamic_cast<OmSolid *>(n);
    if (solid) {
      solidChildren.append(solid);
      continue;
    }

    // cppcheck-suppress constVariablePointer
    OmBasicJoint *j = dynamic_cast<OmBasicJoint *>(n);
    if (j) {
      jointChildren.append(j);
      // cppcheck-suppress constVariablePointer
      OmSolid *const ep = j->solidEndPoint();
      if (ep && j->solidReference() == NULL) {
        solidChildren.append(ep);
        continue;
      }
    }

    // cppcheck-suppress constVariablePointer
    OmPropeller *propeller = dynamic_cast<OmPropeller *>(n);
    if (propeller) {
      propellerChildren.append(propeller);
      continue;
    }

    const OmGroup *const groupChild = dynamic_cast<OmGroup *>(n);
    if (groupChild) {
      collectSolidChildren(groupChild, connectSignals, solidChildren, jointChildren, propellerChildren);
      continue;
    }

    const OmSlot *slot = dynamic_cast<OmSlot *>(n);
    if (slot) {
      if (slot->hasEndPoint()) {
        OmSlot *sep = slot->slotEndPoint();
        while (sep) {
          slot = sep;
          sep = slot->slotEndPoint();
        }
        if (slot->solidEndPoint())
          solidChildren.append(slot->solidEndPoint());
        else if (slot->groupEndPoint())
          collectSolidChildren(slot->groupEndPoint(), connectSignals, solidChildren, jointChildren, propellerChildren);
        else {
          j = dynamic_cast<OmBasicJoint *>(slot->endPoint());
          if (j) {
            jointChildren.append(j);
            // cppcheck-suppress constVariablePointer
            OmSolid *const ep = j->solidEndPoint();
            if (ep && j->solidReference() == NULL) {
              solidChildren.append(ep);
              continue;
            }
          }

          propeller = dynamic_cast<OmPropeller *>(slot->endPoint());
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

void OmSolid::updateDynamicSolidDescendantFlag() {
  mHasDynamicSolidDescendant = false;
  foreach (const OmSolid *s, mSolidChildren) {
    if (!s->isPostFinalizedCalled())
      // postpone flag update after finalization
      return;

    if (s->isDynamic() || s->mHasDynamicSolidDescendant) {
      mHasDynamicSolidDescendant = true;
      break;
    }
  }

  OmSolid *us = upperSolid();
  if (us && us->isPostFinalizedCalled())
    us->updateDynamicSolidDescendantFlag();
}

void OmSolid::updateChildrenAfterJointEndPointChange(OmBaseNode *node) {
  if (node)
    updateChildren();
}

void OmSolid::updateChildren() {
  mSolidChildren.clear();
  mJointChildren.clear();
  mPropellerChildren.clear();
  collectSolidChildren(this, true, mSolidChildren, mJointChildren, mPropellerChildren);

  foreach (OmSolid *const solid, mSolidChildren) {
    connect(solid, &OmSolid::destroyed, this, &OmSolid::updateChildren, Qt::UniqueConnection);
    connect(solid, &OmSolid::destroyed, this, &OmSolid::refreshPhysicsRepresentation, Qt::UniqueConnection);
    connect(solid, &OmSolid::physicsPropertiesChanged, this, &OmSolid::refreshPhysicsRepresentation, Qt::UniqueConnection);
  }
  foreach (OmBasicJoint *const jointChild, mJointChildren)
    connect(jointChild, &OmBasicJoint::endPointChanged, this, &OmSolid::updateChildrenAfterJointEndPointChange,
            Qt::UniqueConnection);
}

bool OmSolid::resetJointPositions(bool allParents) {
  bool b = false;

  foreach (OmBasicJoint *const j, mJointParents) {
    if (allParents || j->upperSolid()->belongsToStaticBasis())
      b |= j->resetJointPositions();
  }

  return b;
}

void OmSolid::updateGlobalCenterOfMass() {
  mGlobalCenterOfMass.setXyz(0.0, 0.0, 0.0);
  mGlobalMass = 0.0;
  foreach (OmSolid *const solid, mSolidChildren) {
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
    const double nativeMass = mass();  // native mirror / declared physics mass (see mass())
    mGlobalCenterOfMass += nativeMass * (matrix() * centerOfMass());
    mGlobalMass += nativeMass;
  }

  if (mGlobalMass > 0.0)
    mGlobalCenterOfMass /= mGlobalMass;
  else
    mGlobalCenterOfMass = position();
}

double OmSolid::averageDensity() const {
  return mGlobalVolume > 0.0 ? mGlobalMass / mGlobalVolume : -1.0;
}

void OmSolid::updateGlobalVolume() {
  double cumulativeVolume = 0.0;

  foreach (OmSolid *const solid, mSolidChildren) {
    if (!solid->isPreFinalizedCalled())
      // skip until finalization is completed
      // it could happen in particular in case of multiple instances of PROTO parameter node
      return;
    solid->updateGlobalVolume();
    cumulativeVolume += solid->globalVolume();
  }

  // ODE is gone: the reference-mass integrator does not run; this
  // Solid's own volume is unknown (counted as 0, see volume()).
  mGlobalVolume = cumulativeVolume;
}

void OmSolid::updateCenterOfMass() {
  assert(isDynamic());

  OmPhysics *const p = physics();
  p->updateMode();
  const int mode = p->mode();

  mCenterOfMass.setXyz(0.0, 0.0, 0.0);

  switch (mode) {
    case OmPhysics::CUSTOM_INERTIA_MATRIX:
      mCenterOfMass = p->centerOfMass().item(0);
      break;
    case OmPhysics::BOUNDING_OBJECT_BASED: {
      if (p->centerOfMass().size() == 1) {
        mCenterOfMass = p->centerOfMass().item(0);
      } else if (mBoundingObject->value() != NULL)
        // the native mirror's CoM equals mReferenceMass->c in this branch (no
        // declared centerOfMass, so createOdeMass never translated it)
        mCenterOfMass.setXyz(mNativeInertiaValid ? mNativeInertia.cx() : 0.0,
                             mNativeInertiaValid ? mNativeInertia.cy() : 0.0,
                             mNativeInertiaValid ? mNativeInertia.cz() : 0.0);
      break;
    }
    default:
      assert(mode == OmPhysics::INVALID);
  }

}

////////////////////
// Apply Methods  //
////////////////////

// Apply to WREN

void OmSolid::applyChangesToWren() {
  OmMatter::applyChangesToWren();
  refreshPhysicsRepresentation();
}

void OmSolid::applyVisibilityFlagsToWren(bool selected) {
  OmMatter::applyVisibilityFlagsToWren(selected);
}

void OmSolid::setDefaultMassSettings(bool applyCenterOfMassTranslation, bool warning) {
  const double fieldMass = physics()->mass();
  if (fieldMass > 0.0) {
    if (warning)
      parsingWarn(
        tr("Undefined inertia matrix: using the identity matrix. Please specify 'boundingObject' or 'inertiaMatrix' values. The Solid has a 'mass' but nothing to derive its inertia from, so it gets 1 kg.m^2 on every axis (far too much for a small part: it will tumble sluggishly). Set a 'boundingObject' (the inertia is derived from it) or declare 'inertiaMatrix' plus 'centerOfMass' -- see docs/reference/physics.md."));
  } else {
    if (warning) {
      if (physics()->density() > 0.0)
        parsingWarn(
          tr("Mass is invalid because 'boundingObject' is not defined. Using default mass properties: mass = 1, inertia "
             "matrix = identity. 'density' needs a 'boundingObject' to integrate over: add one, or set 'mass' (kg) directly -- see docs/reference/physics.md."));
      else
        parsingWarn(
          tr("Mass is invalid: %1. Using default mass properties: mass = 1, inertia matrix = identity. Set Physics.mass to a positive value in kg (or 'density' with a 'boundingObject') -- see docs/reference/physics.md.").arg(fieldMass));
    }
  }

  (void)applyCenterOfMassTranslation;
}

// Compute the mass and the inertia around solid frame's origin
void OmSolid::createOdeMass(bool reset) {
  assert(isDynamic());

  if (reset) {
  }

  // The native mirror is recomputed from scratch below (geometry branch only);
  // an explicit inertiaMatrix never populates it -- the Newton path consumes
  // those fields directly and never reads the geometry-derived tensor.
  mNativeInertiaValid = false;

  // Adds the masses of all the primitives lying in the bounding object
  const OmPhysics *const p = physics();
  const bool customMass = p->mode() == OmPhysics::CUSTOM_INERTIA_MATRIX;
  // needed for average density and average damping

  // Checks whether there is a valid inertia matrix, and uses it if so
  if (customMass)
    setOdeInertiaMatrix();
  else {
    mUseInertiaMatrix = false;

    updateCenterOfMass();

    const double fieldDensity = p->density();
    const double fieldMass = p->mass();

    // Sets the actual total mass
    double actualMass;
    // ODE is gone: re-derived from the native pipeline's own numbers
    // inside the mirror block below (same formula, geometry mass from
    // OmSolidUtilities::addInertia instead of the absent dMass integrator).
    actualMass = 0.0;

    // ODE-free mirror of the exact pipeline above into mNativeInertia -- an
    // INDEPENDENT recomputation from the geometry (OmSolidUtilities::addInertia),
    // not a copy of mOdeMass. This is the Newton feed that survives src/ode
    // deletion; parity vs the dMass oracle is dumped under OMNISIM_DUMP_INERTIA=1
    // and pinned by tests/test_newton_native_inertia_parity.py. Runtime
    // boundingObject edits go through createOdeMass again, so the mirror
    // tracks every recomputation the ODE side makes.
    {
      OmInertia ref;
      OmSolidUtilities::addInertia(&ref, boundingObject(), REFERENCE_DENSITY);
      if (ref.mass() <= 0.0) {
        // setDefaultMassSettings mirror: fieldMass if declared, else 1 kg;
        // identity tensor either way (no COM translate here -- it is applied
        // below from the declared centerOfMass, matching the ODE branch).
        mNativeInertia.setParameters(fieldMass > 0.0 ? fieldMass : 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
      } else
        mNativeInertia = ref;
      // OFF mode: mirror the ODE branch's actualMass formula exactly. The
      // base mass is the geometry-derived reference mass (ref, parity-pinned
      // against the retired dMass integrator), or the default-mass fallback
      // the ODE branch would have installed via setDefaultMassSettings.
      {
        const double baseMass = ref.mass() > 0.0 ? ref.mass() : (fieldMass > 0.0 ? fieldMass : 1.0);
        if (fieldMass > 0.0)
          actualMass = fieldMass;
        else if (fieldDensity != REFERENCE_DENSITY)
          actualMass = baseMass * fieldDensity / REFERENCE_DENSITY;
        else
          actualMass = baseMass;
      }
      mNativeInertia.adjust(actualMass);
      if (p->centerOfMass().size() == 1) {
        const OmVector3 &com = centerOfMass();
        mNativeInertia.translate(com.x() - ref.cx(), com.y() - ref.cy(), com.z() - ref.cz());
      }
      mNativeInertiaValid = true;
      // the earlier updateCenterOfMass() ran before the mirror was valid;
      // refresh it now that the geometry CoM is known.
      updateCenterOfMass();

      // NATIVE-ONLY inertia dump (OMNISIM_DUMP_INERTIA, value-parsed). This used
      // to print the dMass oracle beside the native tensor so the two could be
      // diffed; src/ode is deleted, so the oracle half is gone and the ODE values
      // live frozen in tests/goldens/ode_oracle_goldens.json instead. The dump
      // stays -- unguarded now -- because it is what keeps the native composer
      // pinned to those frozen numbers
      // (tests/test_newton_native_inertia_parity.py). Removing it would retire
      // the only continuous check that the ODE-free mass integrator still agrees
      // with the ODE-era physics.
      static const bool dumpInertia = []() {
        const QString v = QString::fromUtf8(qgetenv("OMNISIM_DUMP_INERTIA")).trimmed().toLower();
        return !v.isEmpty() && v != "0" && v != "false" && v != "off" && v != "no";
      }();
      if (dumpInertia)
        OmLog::info(QString("[inertia-parity] name=%1 mass_native=%2 "
                            "c_native=(%3 %4 %5) I_native=(%6 %7 %8 %9 %10 %11)")
                      .arg(name())
                      .arg(mNativeInertia.mass(), 0, 'g', 17)
                      .arg(mNativeInertia.cx(), 0, 'g', 17)
                      .arg(mNativeInertia.cy(), 0, 'g', 17)
                      .arg(mNativeInertia.cz(), 0, 'g', 17)
                      .arg(mNativeInertia.ixx(), 0, 'g', 17)
                      .arg(mNativeInertia.iyy(), 0, 'g', 17)
                      .arg(mNativeInertia.izz(), 0, 'g', 17)
                      .arg(mNativeInertia.ixy(), 0, 'g', 17)
                      .arg(mNativeInertia.ixz(), 0, 'g', 17)
                      .arg(mNativeInertia.iyz(), 0, 'g', 17));
    }

    updateTopSolidGlobalMass();
    emit massPropertiesChanged();
  }
}

double OmSolid::mass() const {
  // ODE is gone: the native mirror is the same total-mass number the
  // dMass pipeline produced (parity-pinned); a custom-inertia Solid (whose
  // mirror is never populated) reports the declared physics mass.
  if (mNativeInertiaValid)
    return mNativeInertia.mass();
  const OmPhysics *const p = physics();
  return p != NULL && p->mass() > 0.0 ? p->mass() : 0.0;
}

double OmSolid::density() const {
  const double d = isDynamic() ? physics()->density() : -1.0;
  const double v = volume();
  return d >= 0.0 ? d : v > 0.0 ? mass() / v : -1.0;
}

double OmSolid::volume() const {
  return 0.0;  // ODE is gone: the reference-mass integrator does not run (volume unknown, never fabricated)
}

const double *OmSolid::inertiaMatrix() const {
  // The Physics pane is an in-tree caller: it renders this as a row-major
  // 3x3 tensor. Returning NULL after the ODE deletion made selecting its
  // "Mass" tab dereference a null pointer and close OmniSim. Reconstruct the
  // same view from the ODE-free native mirror, or from the authored custom
  // tensor when that mode deliberately bypasses the mirror.
  static thread_local double matrix[9];
  if (mNativeInertiaValid) {
    matrix[0] = mNativeInertia.ixx();
    matrix[1] = mNativeInertia.ixy();
    matrix[2] = mNativeInertia.ixz();
    matrix[3] = mNativeInertia.ixy();
    matrix[4] = mNativeInertia.iyy();
    matrix[5] = mNativeInertia.iyz();
    matrix[6] = mNativeInertia.ixz();
    matrix[7] = mNativeInertia.iyz();
    matrix[8] = mNativeInertia.izz();
    return matrix;
  }

  const OmPhysics *const p = physics();
  if (p != NULL && p->inertiaMatrix().size() >= 1) {
    const OmVector3 &diagonal = p->inertiaMatrix().item(0);
    const OmVector3 offDiagonal = p->inertiaMatrix().size() >= 2 ?
      p->inertiaMatrix().item(1) : OmVector3(0.0, 0.0, 0.0);
    matrix[0] = diagonal.x();
    matrix[1] = offDiagonal.x();
    matrix[2] = offDiagonal.y();
    matrix[3] = offDiagonal.x();
    matrix[4] = diagonal.y();
    matrix[5] = offDiagonal.z();
    matrix[6] = offDiagonal.y();
    matrix[7] = offDiagonal.z();
    matrix[8] = diagonal.z();
    return matrix;
  }

  // Same fallback the mass pipeline uses when neither geometry nor a custom
  // tensor yields usable inertia.
  matrix[0] = 1.0;
  matrix[1] = 0.0;
  matrix[2] = 0.0;
  matrix[3] = 0.0;
  matrix[4] = 1.0;
  matrix[5] = 0.0;
  matrix[6] = 0.0;
  matrix[7] = 0.0;
  matrix[8] = 1.0;
  return matrix;
}

void OmSolid::applyToOdeMass() {
  assert(isDynamic() && mSolidMerger);
  const OmPhysics *const p = physics();
  if (!p->hasApositiveMassOrDensity())
    return;

  createOdeMass();
  mSolidMerger->mergeMass(this);
}

void OmSolid::updateTransformForPhysicsStep() {
  if (mUpdatedInStep)
    return;

  applyPhysicsTransform();

  QList<OmSolid *> reversedList;
  reversedList << this;
  OmSolid *s = NULL;
  OmNode *p = parentNode();
  while (p != NULL && !p->isWorldRoot()) {
    s = dynamic_cast<OmSolid *>(p);
    if (s != NULL) {
      if (s->mUpdatedInStep)
        break;  // ancestor nodes already updated
      reversedList.prepend(s);
    }
    p = p->parentNode();
  }

  // update transform from root to current node as applyPhysicsTransform uses the upper transform matrix
  QListIterator<OmSolid *> it(reversedList);
  while (it.hasNext()) {
    s = it.next();
    s->applyPhysicsTransform();
    s->mUpdatedInStep = true;
  }
}

void OmSolid::applyPhysicsTransform() {
  // ODE is gone: there is never an ODE body to read back (body() is
  // always null -- the ON path would return immediately below); the Newton
  // pose readback lives in postPhysicsStep.
}

//////////////////
// Run Methods  //
//////////////////

// OMNISIM_PROBE_TRAJ root classifier (newton-ode-replacement-plan.md W5): a ROOT is tagged 'A' when it is the
// controlled mechanism -- a OmRobot (panda, husky, spot, a battlebot) -- and 'R' otherwise (a free prop or a
// static structure). The faithful meter splits on this because a loose prop that rolls or settles to a
// different-but-equally-valid rest pose under a different solver is a fidelity FACT (untunable -- more
// substeps EJECT it), not a Newton bug, and reporting only the worst root lets one rolling paint bucket mask
// that the ROBOT tracks ODE exactly. NOTE: "has a joint" is the WRONG proxy -- an articulated PROP (a paint
// bucket with a hinged handle) has a joint yet is not a robot; OmRobot is the semantically correct test.
void OmSolid::postPhysicsStep() {
  // OMNISIM_PROBE_TRAJ (faithful-match meter, newton-ode-replacement-plan.md W5.4): per-step world position
  // of each articulation ROOT, so faithful_check.py can compare a world's Newton vs forced-ODE trajectory
  // (turning the coverage meter's "eligible %" into "faithful %"). Dumped at the same point under both
  // solvers -- the pose here is one physics step behind under BOTH, so the lag cancels in the diff. Exits
  // after OMNISIM_PROBE_TRAJ_MS (default 1000) of sim time. Inert unless the env var is set. The trailing
  // A/R tag (W5) marks articulated robot roots vs rigid free-prop/static roots so the meter can split them.
  // Tier 1a-adjacent (physics-step-cost-optimization-plan.md §Tier 2): these
  // env probes ran once per Solid per step -- a getenv syscall pair per body
  // per tick on a diagnostic that is off in every production run. Latch once;
  // the (never-used) ability to toggle the probe mid-run is what it costs.
  static const bool probeTraj = qEnvironmentVariableIsSet("OMNISIM_PROBE_TRAJ");
  if (probeTraj) {
    bool isRoot = true;
    for (OmNode *n = parentNode(); n != nullptr; n = n->parentNode())
      if (dynamic_cast<const OmSolid *>(n) != nullptr) {
        isRoot = false;
        break;
      }
    // OMNISIM_PROBE_TRAJ_ALL (W6 diagnosis): dump EVERY solid, not just articulation roots, so a robot's
    // per-link / foot positions can be compared Newton-vs-ODE (e.g. localize Spot's leg collapse: feet on
    // the floor + low body => legs not holding the pose; feet below the floor => contact penetration).
    static const bool probeTrajAll = qEnvironmentVariableIsSet("OMNISIM_PROBE_TRAJ_ALL");
    if (isRoot || probeTrajAll) {
      const double tms = OmSimulationState::instance() ? OmSimulationState::instance()->time() : 0.0;
      bool ok = false;
      const double parsed = qEnvironmentVariable("OMNISIM_PROBE_TRAJ_MS").toDouble(&ok);
      if (tms > (ok ? parsed : 1000.0))
        std::_Exit(0);  // first root past the budget exits; every step up to it is fully dumped
      const OmVector3 p = matrix().translation();
      const char tag = (dynamic_cast<const OmRobot *>(this) != nullptr) ? 'A' : 'R';
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

  if (mResetPhysicsInStep) {
    // physics reset from Supervisor: if the solid is also moved from Supervisor in the same step, ODE may overwrite velocities
    // and forces based on the jerk
    resetSingleSolidPhysics();
    if (mSolidMerger)
      mSolidMerger->setBodyArtificiallyDisabled(false);
    mResetPhysicsInStep = false;
    // Tier 1b: an external physics reset intervened -- never let the
    // unchanged-pose compare skip the writeback that follows it.
    mLastNewtonXformValid = false;
  }

  // P3.2 of cuda-newton-physics-plan.md: read this Solid's pose back
  // from the Newton solver. This is the ONLY per-step pose readback now:
  // the ODE-body branch that used to precede it (gated on isBodyEnabled)
  // went with ODE, so a Solid with no Newton registration simply keeps its
  // authored / scene-tree-inherited pose -- which is what the URDF
  // visual-only children (top_chassis, bumpers, top_plate_link inside a
  // newton_husky Robot) wanted anyway.
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
    OmPhysicsBackend *backend = OmPhysicsBackendRegistry::newtonBackend();
    if (backend != nullptr && backend->isAvailable()) {
      OmNewtonBackend *newton = static_cast<OmNewtonBackend *>(backend);
      double xform[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
      if (newton->isWorldRunning() && newton->getBodyXform(mNewtonBodyIndex, xform) == 0) {
        // Tier 1b (physics-step-cost-optimization-plan.md): Newton has no body
        // sleep, so before this check every RESTING body paid the full
        // writeback -- frame projection, two field writes + changedByOde
        // emits, matrix-cache invalidation, two WREN transform sets -- every
        // tick, forever. When the solver hands back a pose BITWISE identical
        // to the one we last wrote, all of that is provably a no-op: skip it.
        // Exact 7-double compare only (an epsilon skip could accumulate
        // visible staleness); velocities below are still refreshed, since
        // they can change while the pose bits do not.
        const bool poseUnchanged =
          mLastNewtonXformValid && std::memcmp(xform, mLastNewtonXform, sizeof(mLastNewtonXform)) == 0;
        if (!poseUnchanged) {
        std::memcpy(mLastNewtonXform, xform, sizeof(mLastNewtonXform));
        mLastNewtonXformValid = true;
        const OmVector3 worldPos(xform[0], xform[1], xform[2]);
        // Newton's quaternion order is (qx, qy, qz, qw); OmQuaternion takes (w, x, y, z).
        const OmQuaternion worldQuat(xform[6], xform[3], xform[4], xform[5]);

        // Project Newton's world pose into the immediate upperPose's
        // local frame -- the upperPose() is the parent in the SCENE
        // TREE (typically a OmBasicJoint, not the parent Solid). Its
        // matrix() includes the joint's rotation. ODE's
        // applyPhysicsTransform does exactly this projection; if we
        // use solidParent() (skipping the joint), the rendering then
        // applies the joint rotation on top and the visual mesh drifts
        // off the body. That was the "body parts falling off" bug --
        // joint constraint satisfied in Newton, but the rendered pose
        // had the joint rotation stacked twice.
        // P3.10n: use setValueFromOde (emits `changedByOde`, NOT
        // `changed`). setValue would fire OmPose::updateTranslation,
        // and our Newton-aware reset hook listens on updateTranslation
        // -- the resulting feedback loop reset joint_q every step and
        // froze Spot at spawn pose. Mirroring ODE's setTransformFromOde
        // sidesteps the slot entirely. We DO still need to manually
        // invalidate the matrix cache because the slot's other job
        // (setMatrixNeedUpdate) skipped with it; supervisor.getPosition
        // reads a cached world matrix and was reporting stale spawn
        // pose otherwise.
        const OmPose *const up = upperPose();
        if (up == nullptr) {
          mTranslation->setValueFromOde(worldPos);
          mRotation->setValueFromOde(OmRotation(worldQuat));
        } else {
          const OmMatrix4 &upm = up->matrix();
          const OmVector3 localPos = upm.pseudoInversed(worldPos);
          mTranslation->setValueFromOde(localPos);
          // Local rotation = inv(upper_rot) * world_rot.
          const OmQuaternion upQ = upm.extractedQuaternion();
          const OmQuaternion localQ = upQ.conjugated() * worldQuat;
          mRotation->setValueFromOde(OmRotation(localQ));
        }
        // DIAGNOSTIC ONLY (physics-step-cost-optimization-plan.md §3b):
        // OMNISIM_NEWTON_SKIP_MATRIX_INVAL=1 drops the subtree invalidation to
        // PRICE it. setMatrixNeedUpdate recurses into every child with no
        // early-out (OmPose.cpp -> OmGroup.cpp, carried through joints by
        // OmBasicJoint), so on a robot the chassis invalidates the whole
        // subtree and then each wheel invalidates its own again, every tick.
        // Ablation rather than a probe because a per-Solid timer would itself
        // be ~15-20% of the quantity being measured, and because this is
        // exactly how OMNISIM_NEWTON_SKIP_WREN killed the rendering theory in
        // one run.
        // ⚠ MEASUREMENT SWITCH, NEVER A SHIPPING MODE: it makes
        // supervisor.getPosition and every sensor read a STALE world matrix.
        static const bool skipMatrixInval =
          qEnvironmentVariableIsSet("OMNISIM_NEWTON_SKIP_MATRIX_INVAL");
        if (!skipMatrixInval)
          setMatrixNeedUpdate();
        // CRITICAL: push the updated translation/rotation to the WREN visual
        // transform. setValueFromOde() above bypasses the `changed` signal
        // (using `changedByOde` to dodge our Newton reset hook), and the
        // `changed` signal is what normally triggers applyTranslationToWren
        // / applyRotationToWren. Without these explicit calls the data
        // D1.4: the WREN transform writeback (and its OMNISIM_NEWTON_SKIP_WREN ablation)
        // died with WREN -- the wgpu refresh reads the pose fields/matrix directly.
        }  // !poseUnchanged (Tier 1b)
        // Populate the world-frame velocity fields from Newton so the
        // supervisor getVelocity() returns the real velocity (it was 0
        // under Newton because only ODE ever filled these). Newton
        // body_qd = [vx,vy,vz, wx,wy,wz] in WORLD frame, exactly the order
        // getVelocity() expects. The RL policy is a velocity-feedback
        // balancer, so this is what lets a Newton-deployed (and
        // in-OmniSim-trained) policy actually walk instead of going blind.
        // DIAGNOSTIC ONLY (same campaign): these sit OUTSIDE the Tier 1b
        // dirty-skip, so they run for every registered dynamic body every
        // tick whether or not the pose moved, and setValue's equality
        // early-out rarely fires for the same reason Tier 1b rarely fires
        // (no body sleep; resting bodies jitter in low bits).
        // ⚠ MEASUREMENT SWITCH: makes supervisor.getVelocity() stale.
        static const bool skipVel =
          qEnvironmentVariableIsSet("OMNISIM_NEWTON_SKIP_VELOCITY");
        double bvel[6] = {0, 0, 0, 0, 0, 0};
        if (!skipVel && mLinearVelocity && mAngularVelocity &&
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

void OmSolid::prePhysicsStep(double ms) {
  int i = 0;

  if (handleJerkIfNeeded())
    mMovedChildren.clear();
  else if (!mMovedChildren.isEmpty())
    childrenJerk();

  if (mIsKinematic && robot()) {
    // OMNISIM_NEWTON_KINEMATIC: the Newton-side analogue of the ODE geom
    // refresh above. A robot-owned kinematic Solid's WORLD pose changes when
    // its ancestors move (the chassis drives, the arm's world placement
    // follows) with no field change of its own -- the field-change hook
    // (syncNewtonPoseFromFields) never fires for that, so refresh here, in
    // the same phase ODE refreshes its geoms. Dirty-tracked: an unmoved body
    // costs one pose compare.
    pushNewtonKinematicPose();
  }

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

OmBasicJoint *OmSolid::jointParent() const {
  const OmSlot *parentSlot = dynamic_cast<OmSlot *>(parentNode());
  if (parentSlot) {
    const OmSlot *granParentSlot = dynamic_cast<OmSlot *>(parentSlot->parentNode());
    if (granParentSlot)
      return dynamic_cast<OmBasicJoint *>(granParentSlot->parentNode());
  }
  return dynamic_cast<OmBasicJoint *>(parentNode());
}

OmSolid *OmSolid::findSolid(const QString &name, const OmSolid *const exception) {
  if (this->name() == name && this != exception)
    return this;

  foreach (OmSolid *const solid, mSolidChildren) {
    OmSolid *const s = solid->findSolid(name, exception);
    if (s && s != exception)
      return s;
  }

  return NULL;
}

OmRobot *OmSolid::robot() const {
  if (!mHasSearchedRobot) {
    mRobot = OmNodeUtilities::findRobotAncestor(this);
    mHasSearchedRobot = true;
  }

  return mRobot;
}

// Returns true if all solid ancestors have no physics
bool OmSolid::belongsToStaticBasis() const {
  const OmSolid *s = this;

  while (s) {
    if (s->isDynamic())
      return false;
    s = s->upperSolid();
  }

  return true;
}

OmPhysics *OmSolid::physics() const {
  return static_cast<OmPhysics *>(mPhysics->value());
}

// Resolves the physicsBackend SFString to a OmPhysicsBackend* via the
// process-wide registry. Every value resolves to OmNewtonBackend, the only
// backend; when the Newton runtime is unavailable the registry still hands out
// that object (inert: nothing registers with it) and logs one error saying the
// world will load and then stand still. Retired values ("ode") are warned about
// once per world in effectivePhysicsBackendName() and run on Newton too.
const QString &OmSolid::physicsBackendName() const {
  // OmSolidDevice subclass models (Accelerometer.wrl, Camera.wrl, GPS.wrl, …)
  // don't declare the `physicsBackend` field, so findSFString returns NULL on
  // their OmSolid base sub-object. Without a guard here, every world containing
  // any OmSolidDevice subclass crashed in postFinalize. A MISSING field is the
  // absence of a choice, so it reads as "auto": the device inherits its
  // articulation's backend, the only correct answer for a node that is part of
  // a robot.
  static const QString kAuto = QStringLiteral("auto");
  if (!mPhysicsBackend)
    return kAuto;
  return mPhysicsBackend->value();
}

OmPhysicsBackend *OmSolid::physicsBackend() const {
  const QByteArray name = physicsBackendName().toUtf8();
  return OmPhysicsBackendRegistry::resolve(OmPhysicsBackendKindFromString(name.constData()));
}

OmBodyHandle OmSolid::bodyHandle() const {
  if (mNewtonBodyIndex >= 0)
    return OmNewtonBackend::handleFromIndex(mNewtonBodyIndex);
  return nullptr;
}

OmBodyHandle OmSolid::carrierBodyHandle() const {
  // See the header comment: a folded device carrier owns no body of its own,
  // so resolve the nearest Newton body up the fold (merger-aware, same walk
  // welds and wrench reads use) and hand back the same handle bodyHandle()
  // would build for it.
  const int idx = nearestNewtonBodyIndex();
  if (idx < 0)
    return nullptr;
  return OmNewtonBackend::handleFromIndex(idx);
}

int OmSolid::effectiveNewtonBodyIndex() const {
  if (mNewtonBodyIndex >= 0)
    return mNewtonBodyIndex;
  if (mSolidMerger && mSolidMerger->solid() != nullptr && mSolidMerger->solid() != this)
    return mSolidMerger->solid()->newtonBodyIndex();
  return -1;
}

int OmSolid::nearestNewtonBodyIndex() const {
  for (const OmSolid *s = this; s != nullptr; s = s->upperSolid()) {
    const int idx = s->effectiveNewtonBodyIndex();
    if (idx >= 0)
      return idx;
  }
  return -1;
}

OmSolid *OmSolid::findSolidByNewtonBodyIndex(int idx) {
  if (idx < 0)
    return nullptr;
  for (const OmSolid *const s : cSolids) {
    if (s != nullptr && s->mNewtonBodyIndex == idx)
      // cSolids stores const pointers as a storage convention only (see the
      // const_cast in flushPendingNewtonRegistrations); the entries are not
      // truly const, and callers (VacuumGripper attach) need mutable access.
      return const_cast<OmSolid *>(s);
  }
  return nullptr;
}

// Retired physicsBackend values. "ode" (and anything else the schema let
// through that is not "newton" / "auto" / "") used to select OmOdeBackend, an
// inert stub: the Solid was registered with NO solver -- no gravity, no
// contact -- while the world loaded clean and only a per-Solid warning said so.
// That trap was closed on 2026-09-02: a retired value now reads as "auto" (the
// Solid runs on Newton like every other) and the world is told ONCE, naming
// the first offender. Keyed on the live OmWorld through a QPointer so a reload
// warns again and a recycled address cannot suppress it.
static bool physicsBackendValueIsRetired(const QString &v) {
  return !v.isEmpty() && v != QStringLiteral("auto") && v != QStringLiteral("newton");
}

static void warnRetiredPhysicsBackendOncePerWorld(const QString &value, const QString &where) {
  static QPointer<const OmWorld> sWarnedWorld;
  const OmWorld *const w = OmWorld::instance();
  if (w != nullptr && sWarnedWorld == w)
    return;
  sWarnedWorld = w;
  OmLog::warning(QObject::tr("physicsBackend \"%1\" is retired; this Solid runs on Newton, the only backend "
                             "(first seen on %2; reported once per world). ODE was removed, and since 2026-09-02 "
                             "a retired value no longer leaves a Solid with no physics. Delete the field or "
                             "write \"newton\".")
                   .arg(value, where),
                 false, OmLog::ODE);
}

bool OmSolid::gatedAwayFromNewton() const {
  bool gated = false;
  effectivePhysicsBackendName(&gated);
  return gated;
}

QString OmSolid::effectivePhysicsBackendName(bool *downgradedByGate) const {
  if (downgradedByGate != nullptr)
    *downgradedByGate = false;
  static const QString kNewton = QStringLiteral("newton");
  static const QString kAuto = QStringLiteral("auto");
  const QString &local = physicsBackendName();
  // An EXPLICIT local "newton" always wins.
  if (local == kNewton)
    return local;
  if (physicsBackendValueIsRetired(local))
    warnRetiredPhysicsBackendOncePerWorld(local, QStringLiteral("Solid '%1'").arg(usefulName()));
  // local is "auto" (the Phase-D Solid/Robot default, baa1c104) or a retired value that reads as
  // "auto": a robot is one coupled multibody system and MUST resolve to a single solver, so an
  // EXPLICIT backend on any ancestor governs this whole subtree. Walk up for the nearest ancestor
  // Solid that explicitly chose "newton" and inherit it. (Historically this walk is what stopped a
  // descendant's default from overriding its URDFRobot's explicit choice -- the Spot "frozen joint
  // sensor" regression, where leg joints registered with one solver inside a robot on another.)
  for (OmNode *n = parentNode(); n != nullptr; n = n->parentNode()) {
    const OmSolid *p = dynamic_cast<const OmSolid *>(n);
    if (p == nullptr)
      continue;
    if (p->physicsBackendName() == kNewton)
      return kNewton;  // explicit ancestor governs the whole articulation
  }
  // No explicit ancestor: consult the world-level default (default-flip-plan.md §3.2) before
  // resolving bare "auto". A world can thus pin every still-"auto" Solid to "newton" without editing
  // each node -- which is also what "auto" means, so the field is inert unless it carries a retired
  // value, which warns once. An explicit local or ancestor backend already returned above.
  const OmWorldInfo *const wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
  if (wi != nullptr) {
    const QString &wd = wi->defaultPhysicsBackend();
    if (wd == kNewton)
      return wd;
    if (physicsBackendValueIsRetired(wd))
      warnRetiredPhysicsBackendOncePerWorld(wd, QStringLiteral("WorldInfo.defaultPhysicsBackend"));
  }
  // Still bare "auto": capability gate (default-flip-plan.md §4.1). If this articulation uses a
  // Newton-unsupported feature (non-Hinge/Slider joint = correctness; a kinematic chain under
  // OMNISIM_NEWTON_KINEMATIC=0), it is NOT registered with Newton -- and since there is no other solver,
  // that is a genuine SILENT omission, so flag it for the enforcement sweep in
  // flushPendingNewtonRegistrations (which fatals under newtonEnforced() unless the reason is a
  // kinematic chain, which the scene tree animates).
  if (!articulationNewtonCapable()) {
    if (downgradedByGate != nullptr)
      *downgradedByGate = true;
    return kAuto;
  }
  // bare "auto", Newton-capable -> Newton.
  return kAuto;
}

// P3.10j: Walk the boundingObject sub-tree looking for a mesh
// (OmTriangleMeshGeometry subclasses: OmMesh, OmIndexedFaceSet). If
// found, compute the local-frame AABB from the mesh's vertex stream
// and emit the half-extents via the out params. Returns true on hit.
//
// URDFs with mesh collisions (Spot's body/hip/upper-leg use STL meshes,
// not primitives) fall through Newton's primitive narrow-phase
// otherwise; the placeholder-shape fallback in flushPendingNewton-
// Registrations isn't a substitute, it just gets the body floating.
// An AABB box is a coarse approximation but enough for ground contact
// and joint-actuator stability.
static bool computeBoundingObjectMeshAabb(OmBaseNode *bo,
                                          double *hx, double *hy, double *hz,
                                          double *cx, double *cy, double *cz) {
  // Unwrap nested Pose/Group wrappers the same way the primitive path
  // does in flushPendingNewtonRegistrations.
  for (int unwrap = 0; unwrap < 4 && bo != nullptr; ++unwrap) {
    if (dynamic_cast<OmTriangleMeshGeometry *>(bo))
      break;
    if (const OmGroup *g = dynamic_cast<const OmGroup *>(bo)) {
      const OmMFNode &kids = g->children();
      if (kids.size() == 0)
        return false;
      bo = dynamic_cast<OmBaseNode *>(kids.item(0));
      continue;
    }
    return false;
  }
  OmTriangleMeshGeometry *const tmg = dynamic_cast<OmTriangleMeshGeometry *>(bo);
  if (tmg == nullptr)
    return false;
  OmTriangleMesh *const tm = tmg->triangleMesh();
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
// OMNISIM_NEWTON_KINEMATIC (value-parsed, DEFAULT OFF) -- kernel blocker #4,
// _scratch/design_kinematic_inertia.md Part 1: native kinematic bodies on
// Newton via MuJoCo MOCAP bodies. When ON, physics-less joint endpoints
// register as engine-driven mocap colliders instead of gating the whole
// articulation to ODE. OFF = byte-identical behavior everywhere this is
// consulted (the gate, the flush, the pose-push hooks).
bool OmSolid::newtonKinematicNativeEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_KINEMATIC")).trimmed().toLower();
    // DEFAULT ON since 2026-08-08: ODE is being deleted, so there is no
    // fallback left to degrade to -- an unset gate must mean the native
    // path, not silence. "0"/"false"/"off"/"no" still opt out.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

static bool solidSubtreeNewtonCapable(const OmSolid *s, const char **reasonOut = nullptr) {
  if (s == nullptr)
    return true;
  // Mesh collision geometry is NOT a disqualifier: Newton handles triangle-mesh collision natively
  // (add_shape_mesh, newton-ode-replacement-plan.md W1; convexified -- see the hard-won rules). The
  // OMNISIM_NEWTON_MESH_TO_ODE=1 lever that used to route mesh-collider articulations "to ODE" was
  // deleted on 2026-09-02: with ODE gone it routed them to no physics at all.
  // Tier A (correctness): a joint to a child that Newton can't register (only Hinge/Slider are).
  for (OmBasicJoint *const j : s->jointChildren()) {
    // EXACT type via nodeType(), NOT dynamic_cast: OmBallJoint + OmHinge2Joint both INHERIT OmHingeJoint,
    // so dynamic_cast<OmHingeJoint*> wrongly accepts them. Newton registers HingeJoint + SliderJoint
    // (revolute/prismatic), Hinge2Joint (a native 2-DoF d6, since W2) and BallJoint (a native 3-DoF
    // spherical joint, since W2.2 -- see OmBasicJoint flush). So neither type disqualifies an
    // articulation, with OR without OMNISIM_NEWTON_BALL_HINGE2.
    //
    // ⚠ WHAT THE FLAG CHANGES IS WHETHER THOSE TWO CAN BE *DRIVEN*, AND THE GATE-OFF CASE IS A SILENT
    // ONE. Admitted-but-passive means a MOTORISED Hinge2Joint / BallJoint on an "auto" articulation
    // resolves to Newton, gets its constraint, and then its motors do nothing at all: setPosition and
    // setVelocity are accepted, no error is logged, and its position sensors read an ODE joint that a
    // Newton-driven world never advances (so they report a frozen 0). That is pre-existing behaviour and
    // this gate does not change it; OMNISIM_NEWTON_BALL_HINGE2=1 is what fixes it, by registering both
    // types with per-axis actuation and reading their angles back from Newton (OmBasicJoint::
    // registerNewtonMultiDof + the postPhysicsStep overrides). Worlds affected today: the motor2 /
    // motor3 / muscle / gyro samples and the api joint worlds.
    // ⚠ THERE IS NO WORKAROUND: a motorised Hinge2Joint / BallJoint with the gate off has NO actuation
    // path at all (there is no second solver to pin it to). Making OMNISIM_NEWTON_BALL_HINGE2 work is
    // the only fix.
    const int nt = j->nodeType();
    if (nt != WB_NODE_HINGE_JOINT && nt != WB_NODE_SLIDER_JOINT && nt != WB_NODE_HINGE_2_JOINT &&
        nt != WB_NODE_BALL_JOINT) {
      if (reasonOut != nullptr)
        *reasonOut = "joint";
      return false;
    }
    // KINEMATIC CHAIN: a joint whose endPoint Solid has NO Physics node is not a dynamics problem at
    // all -- classic Webots animates it kinematically (the scene tree moves, no solver simulates it).
    // Newton has no kinematic-joint mode, so such an endpoint never registers a Newton body and the
    // joint flush FATALs under newton-enforce ("child body resolved to Newton but never registered").
    // Measured on the readiness sweep 2026-08-07: this one shape explains most of the 13 red worlds
    // (ball_joint_reset, gps_speed's 'solid_without_physic', touch_sensor_kinematic, ...). Route the
    // articulation to ODE, which still owns kinematic simulation while it ships; when the kinematic-
    // collision kernel work lands (ode-retirement-campaign.md), this clause is where it takes over.
    // OMNISIM_NEWTON_KINEMATIC lifts this disqualifier: the endpoint then
    // registers as a native KINEMATIC (mocap) body in the flush and the
    // engine keeps animating it exactly as under ODE (the joint is simply
    // never registered with Newton -- see OmBasicJoint::postFinalize).
    if (!OmSolid::newtonKinematicNativeEnabled()) {
      const OmSolid *const ep = j->solidEndPoint();
      if (ep != nullptr && ep->physics() == nullptr) {
        if (reasonOut != nullptr)
          *reasonOut = "kinematic";
        return false;
      }
    }
    // The ARTICULATED child hangs off the joint's ENDPOINT, not solidChildren() (which holds only fixed
    // sub-solids). Recurse there to reach a deeper non-Hinge/Slider joint or mesh — without this, a
    // mixed hinge->ball chain is missed and wrongly admitted to Newton (verified failure).
    if (!solidSubtreeNewtonCapable(j->solidEndPoint(), reasonOut))
      return false;
  }
  // Also recurse fixed sub-solids (Solids nested without an intervening joint).
  for (OmSolid *const c : s->solidChildren())
    if (!solidSubtreeNewtonCapable(c, reasonOut))
      return false;
  return true;
}

bool OmSolid::articulationNewtonCapable(const char **reasonOut) const {
  if (reasonOut != nullptr)
    *reasonOut = "capable";
  // Power-user / A-B-test bypass: force auto->Newton even for unsupported features.
  if (qEnvironmentVariableIsSet("OMNISIM_AUTO_NO_CAPABILITY_GATE"))
    return true;
  // Resolve to the articulation ROOT (topmost Solid ancestor) so every body in one coupled multibody
  // system shares ONE answer -- mixing solvers within an articulation is the Spot frozen-sensor bug.
  const OmSolid *root = this;
  for (OmNode *n = parentNode(); n != nullptr; n = n->parentNode())
    if (const OmSolid *const p = dynamic_cast<const OmSolid *>(n))
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
    *reasonOut = reason;  // "capable" | "mesh" | "joint" | "kinematic" -- for the OMNISIM_PROBE_GATE coverage sweep
  if (!capable && mNewtonArticulationCapable != 0)
    OmLog::info(tr("[capability-gate] '%1' uses a Newton-unsupported feature (%2); its \"auto\" physics "
                   "is not registered with Newton.")
                  .arg(name())
                  .arg(QString::fromUtf8(reason)));
  mNewtonArticulationCapable = capable ? 1 : 0;
  return capable;
}

// OMNISIM_NEWTON_TOUCH_FORCE (value-parsed, DEFAULT OFF): the force-TouchSensor
// un-fold + cfrc_int mount-force readback (_scratch/design_weld_touch.md T1).
// When ON, a force-type TouchSensor with a Physics node is exempted from the
// P3.10c fold: it registers its OWN Newton body (own boundingObject shapes,
// own mass -- the leader stops rolling it up) welded to the leader with a
// FIXED joint, so its mount wrench is readable. OFF = today's registration,
// byte-identical.
static bool newtonTouchForceUnfoldEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_TOUCH_FORCE")).trimmed().toLower();
    // DEFAULT ON since 2026-08-08: ODE is being deleted, so there is no
    // fallback left to degrade to -- an unset gate must mean the native
    // path, not silence. "0"/"false"/"off"/"no" still opt out.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

// OMNISIM_NEWTON_BUMPER (value-parsed, DEFAULT OFF): same un-fold, applied to
// "bumper" TouchSensors. A bumper answers "is anything touching ME", which the
// native contact set can only attribute if the sensor owns a body -- folded, its
// contacts are indistinguishable from its chassis's. Must match the gate in
// OmTouchSensor.cpp byte-for-byte: the read there refuses unless the sensor was
// un-folded here.
static bool newtonBumperUnfoldEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_BUMPER")).trimmed().toLower();
    // DEFAULT ON since 2026-08-08: ODE is being deleted, so there is no
    // fallback left to degrade to -- an unset gate must mean the native
    // path, not silence. "0"/"false"/"off"/"no" still opt out.
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

// True iff this node is a force-type TouchSensor the un-fold applies to.
// Consulted by the fold walk (register it separately), by rolledUpMass /
// gatherFixedSolids (do NOT double-count its mass on the leader -- the 981 N
// canonical value exists precisely because only the mass ABOVE the mount
// presses through it), and by the composite-inertia rollup (design risk:
// OMNISIM_NEWTON_COMPOSITE_INERTIA must skip un-folded children).
static bool isUnfoldedTouchSensor(const OmNode *n) {
  const OmTouchSensor *const ts = dynamic_cast<const OmTouchSensor *>(n);
  if (ts == nullptr)
    return false;
  // deviceType() is a non-const virtual; the cast is const-safe (pure read).
  const bool isBumper = const_cast<OmTouchSensor *>(ts)->deviceType() == OmTouchSensor::BUMPER;
  // Two independent gates over one mechanism: force/force-3d sensors un-fold so
  // their MOUNT WRENCH is readable, bumpers so their CONTACTS are attributable.
  if (isBumper) {
    // A BUMPER'S PRECONDITION IS A COLLIDER, NOT A BODY, so it is spelled with
    // boundingObject and NOT with physics.
    //
    // This gate used to require a Physics node for both types, and that was the
    // whole bumper defect. A bumper is the one device whose canonical authoring
    // has NO Physics node -- upstream needs none (a pad is not a body, and
    // ODE's near-callback set the touch flag straight off the geom), the shipped
    // PROTOs are written that way, and OmTouchSensor::updateType demands physics
    // only for "force"/"force-3d". Requiring it here meant the normal spelling
    // never un-folded, and the fold DROPS the child's boundingObject (see the T1
    // comment at the fold site), so the pad was not a collider either: the parent
    // BODY took the contact instead. Measured on OmniBench lane 4's own scene,
    // whose pad protrudes 10 mm below the chassis:
    //     physics NULL  ->  value 0 over 1000 samples, rest z = 0.64989 (the
    //                       CHASSIS underside on the floor)
    //     physics Physics{mass 0.001}
    //                   ->  value 1, rest z = 0.65984 (the PAD on the floor,
    //                       which is what the declared geometry says)
    // i.e. the sensor was not merely unreadable, the scene's collision geometry
    // was wrong by the pad's protrusion. Un-folding on the boundingObject fixes
    // both halves at one site, because they are one mechanism.
    //
    // A bumper with no boundingObject keeps folding: there is no geometry to
    // attribute a contact to, so a body of its own would be an empty body, and
    // OmTouchSensor's read still says plainly that it cannot answer.
    if (ts->boundingObject() == nullptr)
      return false;
    return newtonBumperUnfoldEnabled();
  }
  // "force"/"force-3d" keep requiring a Physics node, because for them it is a
  // real precondition and not an accident of this fold: the value is the MOUNT
  // WRENCH, which is only defined for a sensor that is an inertial body in its
  // own right. updateType() already warns at parse time when one is missing.
  if (ts->physics() == nullptr)
    return false;
  return newtonTouchForceUnfoldEnabled();
}

// OMNISIM_NEWTON_VACUUM_COLLIDER (value-parsed, DEFAULT ON): un-fold a
// VacuumGripper for the same reason the two TouchSensor gates above exist --
// so the device's OWN boundingObject survives as a collider.
static bool newtonVacuumUnfoldEnabled() {
  static const bool on = []() {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_VACUUM_COLLIDER")).trimmed().toLower();
    if (v.isEmpty())
      return true;
    return !(v == "0" || v == "false" || v == "off" || v == "no");
  }();
  return on;
}

// A VacuumGripper was folded into its merge leader like any other fixed child,
// and the fold DROPS the child's boundingObject (see the T1 comment below:
// "only the leader's own mBoundingObject registers"). The consequences chain:
// the cup never collides, so it never accumulates contact points, so
// attachToSolid() resolves no partner Solid, so createFixedJoint() is reached
// with other == nullptr and engages the weld with body2 < 0 -- which
// OmVacuumGripper documents as "welds it to the WORLD". Turning the vacuum on
// therefore PINS THE ARM IN MID-AIR instead of picking anything up.
//
// Measured on a slider rig (0.5 kg tool, 0.2 kg part, 400 N motor, CPU
// mj_step), control and test in one world:
//   vacuum OFF  tool lifts 0.1407 -> 0.4976 m, part correctly left behind
//   vacuum ON   tool FROZEN at 0.1468 m, part never leaves the floor, and the
//               tool springs free the instant turnOff() is called
// The gripper never appeared in the registration line at all -- "registered 2
// dynamic + 2 static" for a world whose only bodies are the tool and the part.
//
// Same mechanism as the TouchSensor un-fold, so the same fix: give it its own
// body, welded to the leader with a fixed joint, carrying its own shapes.
static bool isUnfoldedVacuumGripper(const OmNode *n) {
  const OmVacuumGripper *const vg = dynamic_cast<const OmVacuumGripper *>(n);
  if (vg == nullptr)
    return false;
  if (vg->boundingObject() == nullptr)
    return false;  // nothing to rescue -- folding it costs nothing
  return newtonVacuumUnfoldEnabled();
}

// Every contact-sensing device whose collider must survive the fold.
static bool isUnfoldedContactDevice(const OmNode *n) {
  return isUnfoldedTouchSensor(n) || isUnfoldedVacuumGripper(n);
}

// P3.10d: walk a Solid's descendant tree summing masses of Solids that
// would be filtered as fixed-children. Stops descending into any
// HingeJoint subtree (those are real articulated bodies that get their
// own Newton mass). Returns the leader's own mass + the rolled-up
// fixed-child masses, mirroring ODE's OmSolidMerger combined-inertia
// behaviour. Without this, a URDFRobot wrapper Robot (default m=0.001)
// would dominate the inertia tensor of a 46 kg husky chassis, and the
// 2.6 kg wheels would fling the wrapper around like a kite.
static double rolledUpMass(const OmNode *root) {
  double m = 0.0;
  if (const OmSolid *rootSolid = dynamic_cast<const OmSolid *>(root)) {
    if (rootSolid->physics() != nullptr && rootSolid->physics()->mass() > 0.0)
      m += rootSolid->physics()->mass();
  }
  if (const OmGroup *g = dynamic_cast<const OmGroup *>(root)) {
    const OmMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const OmNode *kid = kids.item(i);
      if (kid == nullptr)
        continue;
      // HingeJoints + their endPoint Solids are independent Newton
      // bodies -- their masses live in their own bodies, NOT rolled up.
      if (dynamic_cast<const OmBasicJoint *>(kid))
        continue;
      // Un-folded force TouchSensors are independent Newton bodies too
      // (fixed-welded, not merged) -- rolling their mass up as well would
      // double-count it and turn the canonical 981 N mount force into 1962.
      if (isUnfoldedContactDevice(kid))
        continue;
      m += rolledUpMass(kid);
    }
  }
  return m;
}

// Walk the leader Solid + its FIXED-child descendants (same traversal as
// rolledUpMass, stopping at joint subtrees) and collect the dynamic (mass>0)
// Solids. Used by the composite-inertia rollup below.
static void gatherFixedSolids(const OmNode *root, QList<const OmSolid *> &out) {
  if (const OmSolid *const s = dynamic_cast<const OmSolid *>(root)) {
    if (s->physics() != nullptr && s->physics()->mass() > 0.0)
      out.append(s);
  }
  if (const OmGroup *const g = dynamic_cast<const OmGroup *>(root)) {
    const OmMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const OmNode *const kid = kids.item(i);
      if (kid == nullptr || dynamic_cast<const OmBasicJoint *>(kid))
        continue;
      // Un-folded force TouchSensors carry their own Newton body -- the
      // composite-inertia rollup must not absorb them (design risk list).
      if (isUnfoldedContactDevice(kid))
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
static bool rolledUpComInertia(const OmSolid *leader, double &outMass,
                               OmVector3 &outCom, double Iout[6]) {
  QList<const OmSolid *> bodies;
  gatherFixedSolids(leader, bodies);
  if (bodies.isEmpty())
    return false;
  // Per-body world-frame mass, COM and inertia-about-COM.
  auto worldInertia = [](const OmSolid *D, double &m, OmVector3 &cW, OmMatrix3 &IW) {
    const OmPhysics *const p = D->physics();
    m = p->mass();
    const OmMatrix3 R = D->rotationMatrix();
    const OmVector3 pD = D->matrix().translation();
    OmVector3 cl(0.0, 0.0, 0.0);
    if (p->centerOfMass().size() >= 1)
      cl = p->centerOfMass().item(0);
    cW = pD + R * cl;
    double ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    const OmMFVector3 &im = p->inertiaMatrix();
    if (im.size() >= 1) { const OmVector3 &dd = im.item(0); ixx = dd.x(); iyy = dd.y(); izz = dd.z(); }
    if (im.size() >= 2) { const OmVector3 &oo = im.item(1); ixy = oo.x(); ixz = oo.y(); iyz = oo.z(); }
    const OmMatrix3 Il(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
    IW = R * Il * R.transposed();
  };
  double M = 0.0;
  OmVector3 mc(0.0, 0.0, 0.0);
  for (const OmSolid *const D : bodies) {
    double m; OmVector3 cW; OmMatrix3 IW;
    worldInertia(D, m, cW, IW);
    M += m;
    mc = mc + cW * m;
  }
  if (M <= 0.0)
    return false;
  const OmVector3 cWtot = mc * (1.0 / M);
  double Iw[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (const OmSolid *const D : bodies) {
    double m; OmVector3 cW; OmMatrix3 IW;
    worldInertia(D, m, cW, IW);
    const OmVector3 d = cW - cWtot;
    const double d2 = d.dot(d);
    const double pa[9] = {
      d2 - d.x() * d.x(), -d.x() * d.y(),     -d.x() * d.z(),
      -d.y() * d.x(),     d2 - d.y() * d.y(), -d.y() * d.z(),
      -d.z() * d.x(),     -d.z() * d.y(),     d2 - d.z() * d.z()};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        Iw[r * 3 + c] += IW(r, c) + m * pa[r * 3 + c];
  }
  const OmMatrix3 Rs = leader->rotationMatrix();
  const OmVector3 ps = leader->matrix().translation();
  const OmMatrix3 IWm(Iw[0], Iw[1], Iw[2], Iw[3], Iw[4], Iw[5], Iw[6], Iw[7], Iw[8]);
  const OmMatrix3 Il = Rs.transposed() * IWm * Rs;
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
static bool subtreeHasPhysics(const OmNode *root) {
  if (const OmSolid *const rootSolid = dynamic_cast<const OmSolid *>(root)) {
    if (rootSolid->physics() != nullptr)
      return true;
  }
  if (const OmGroup *const g = dynamic_cast<const OmGroup *>(root)) {
    const OmMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const OmNode *const kid = kids.item(i);
      if (kid == nullptr)
        continue;
      // Joint endpoints are independent Newton bodies registered on
      // their own; their Physics doesn't roll up into this leader.
      if (dynamic_cast<const OmBasicJoint *>(kid))
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
static double newtonSoftKeForMaterial(const OmSFString *contactMaterial) {
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
// Does anything BELOW this solid carry a bounding object?
//
// Asked so that a Robot wrapper with no descendant collider can fall back to
// its own shape instead of the r=1mm placeholder. A multi-link robot answers
// true (its wheels/feet are the load-bearing colliders and the wrapper should
// stay out of their way); a single-link one answers false and would otherwise
// have no usable collision geometry at all.
static bool hasDescendantCollider(const OmSolid *s) {
  if (s == nullptr)
    return false;
  const QList<OmSolid *> &kids = s->solidChildren();
  foreach (const OmSolid *child, kids) {
    if (child == nullptr)
      continue;
    if (child->boundingObject() != nullptr)
      return true;
    if (hasDescendantCollider(child))
      return true;
  }
  return false;
}

// The collider's pose in the OWNING BODY's local frame, accumulated down the
// boundingObject's Pose/Transform chain.
//
// ⚠ Both walkers below used to carry a bare OmVector3 and NEVER read
// OmPose::rotation(), so every authored collider rotation was silently dropped.
// addShapeCylinder then invented a fixed -90 deg about X to compensate; the two
// cancelled on wheels (whose Pose carries +90 about X, and a capsule is
// symmetric about its centre) and corrupted every collider that was NOT
// pre-rotated. See the SHAPE XFORM CONVENTION note in OmNewtonBackend.hpp.
struct OmNewtonShapeXform {
  OmVector3 t;
  OmQuaternion q;  // identity by default (OmQuaternion's default ctor)
};

// Compose one authored Pose onto the running frame: X_child = X_running * X_pose.
// The Pose's own translation is expressed in the frame its ANCESTORS establish,
// so it has to be rotated by the running quaternion before being added --
// summing raw translations is only correct while every ancestor is unrotated,
// which is exactly the assumption the old code made implicitly. Handles
// arbitrary nesting (Pose inside Pose inside Group inside Shape).
static OmNewtonShapeXform composeNewtonShapePose(const OmNewtonShapeXform &running,
                                                 const OmPose *p) {
  OmNewtonShapeXform out;
  out.t = running.t + running.q * p->translation();
  out.q = running.q * p->rotation().toQuaternion();
  out.q.normalize();  // guard against drift over a deep chain
  return out;
}

static QString addNewtonPrimitive(OmNewtonBackend *newton, int idx,
                                  const OmBaseNode *g, const OmNewtonShapeXform &x,
                                  double softKe, double solidMu,
                                  double solidMuT, double solidMuR) {
  const OmVector3 &off = x.t;
  // Newton/warp quaternion order is (qx, qy, qz, qw); OmQuaternion stores (w, x, y, z).
  const double qx = x.q.x(), qy = x.q.y(), qz = x.q.z(), qw = x.q.w();
  if (const OmSphere *sphere = dynamic_cast<const OmSphere *>(g)) {
    // No orientation: a sphere is rotation-invariant. It still needs the
    // correctly COMPOSED offset above.
    newton->addShapeSphere(idx, sphere->radius(), off.x(), off.y(), off.z(), solidMu,
                           solidMuT, solidMuR);
    return QString("sphere r=%1 at (%2,%3,%4)").arg(sphere->radius())
        .arg(off.x()).arg(off.y()).arg(off.z());
  }
  if (const OmBox *box = dynamic_cast<const OmBox *>(g)) {
    const OmVector3 &sz = box->size();
    newton->addShapeBox(idx, sz.x() * 0.5, sz.y() * 0.5, sz.z() * 0.5,
                        off.x(), off.y(), off.z(), softKe, qx, qy, qz, qw, solidMu,
                        solidMuT, solidMuR);
    return QString("box hx=%1 hy=%2 hz=%3 at (%4,%5,%6) q=(%7,%8,%9,%10)")
        .arg(sz.x() * 0.5).arg(sz.y() * 0.5).arg(sz.z() * 0.5)
        .arg(off.x()).arg(off.y()).arg(off.z())
        .arg(qx).arg(qy).arg(qz).arg(qw);
  }
  if (const OmCylinder *cyl = dynamic_cast<const OmCylinder *>(g)) {
    // Hand the cylinder over as an oriented CAPSULE of the same radius AND
    // half-height, exactly as the single-shape path below does.
    //
    // ⚠ This walker was written later (621a7ee41) and re-implemented the
    // primitive cases from scratch, keeping the point-contact sphere that
    // attachNewtonShapeFromBoundingObject had ALREADY replaced -- so opting a
    // world into compound colliders silently threw away every cylinder's
    // length and left it colliding at a single point at its centre.
    //
    // Measured: a 0.06 m box parked on a r=0.05 h=0.60 horizontal bar, offset
    // 0.22 m along the bar's own axis, fell straight past it to the ground
    // (z=0.0297) with newtonCompoundColliders TRUE and rested on top of it
    // (z=0.5796 = bar top 0.55 + half the box) with the flag absent. Same
    // world, same bar, one flag. In omniarm6_universal_pick this is why the
    // suction tool -- two cylinders, hence two small spheres with nothing
    // along the shaft -- swept through the parts it was meant to pick.
    const double halfHeight = cyl->height() * 0.5;
    newton->addShapeCylinder(idx, cyl->radius(), halfHeight, off.x(), off.y(), off.z(),
                             qx, qy, qz, qw);
    return QString("cylinder->capsule r=%1 hh=%2 at (%3,%4,%5) q=(%6,%7,%8,%9)")
        .arg(cyl->radius()).arg(halfHeight)
        .arg(off.x()).arg(off.y()).arg(off.z())
        .arg(qx).arg(qy).arg(qz).arg(qw);
  }
  if (const OmCapsule *cap = dynamic_cast<const OmCapsule *>(g)) {
    // Was offset-less AND orientation-less: an authored Capsule inside a Pose
    // collided at the body origin, unrotated, however the .wbt placed it.
    newton->addShapeCapsule(idx, cap->radius(), cap->height() * 0.5,
                            off.x(), off.y(), off.z(), qx, qy, qz, qw);
    return QString("capsule r=%1 hh=%2 at (%3,%4,%5) q=(%6,%7,%8,%9)")
        .arg(cap->radius()).arg(cap->height() * 0.5)
        .arg(off.x()).arg(off.y()).arg(off.z())
        .arg(qx).arg(qy).arg(qz).arg(qw);
  }
  return QString();
}

// THE ONE READ of the compound-collider opt-in. Two independent copies of this
// test used to exist -- the collider walker and the inertia-source branch --
// and both were PRESENCE-gated, i.e. `OMNISIM_NEWTON_COMPOUND_COLLIDERS=0`
// turned the feature ON. That is the exact trap AGENTS.md documents for
// OMNISIM_REQUIRE_NEWTON, and it is inconsistent with every neighbouring knob
// (newtonStatics, newtonRobotColliders, newtonKinematicNative), all of which
// value-parse. Now value-parsed and shared, so the collider choice and the
// inertia choice can never disagree (internal parity plan, item W1.6).
//
// Per-call (NOT static) so a world switched in via the launcher's worldReload
// reads ITS OWN WorldInfo field. Precedence: env ON wins, else the world field.
static bool newtonCompoundCollidersOn() {
  const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_COMPOUND_COLLIDERS")).trimmed().toLower();
  if (!v.isEmpty() && v != "0" && v != "false" && v != "off" && v != "no")
    return true;
  const OmWorldInfo *const wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
  return wi != nullptr && wi->newtonCompoundColliders();
}

// Compound walker: recurse the WHOLE boundingObject sub-tree and register
// EVERY primitive as its own Newton shape (accumulating Pose/Transform
// translation offsets), so a multi-collider rigid body -- a free dynamic bin
// with floor + walls + a graspable handle on ONE body -- attaches all of its
// colliders, not just the first child of the Group. The default path (below)
// still registers only the first primitive, so this richer behaviour is
// OPT-IN via OMNISIM_NEWTON_COMPOUND_COLLIDERS to keep every existing world's
// physics byte-for-byte unchanged.
static QString registerNewtonShapesRec(OmNewtonBackend *newton, int idx,
                                       const OmBaseNode *bo, OmNewtonShapeXform x,
                                       double softKe, double solidMu,
                                       double solidMuT, double solidMuR) {
  if (bo == nullptr)
    return QString();
  if (const OmShape *sh = dynamic_cast<const OmShape *>(bo))
    return registerNewtonShapesRec(newton, idx, sh->geometry(), x, softKe, solidMu, solidMuT, solidMuR);
  // OmPose extends OmGroup: compose its translation AND rotation, then recurse
  // its kids. (OmTransform derives from OmPose, so this arm catches it too.)
  if (const OmPose *p = dynamic_cast<const OmPose *>(bo)) {
    const OmNewtonShapeXform childX = composeNewtonShapePose(x, p);
    QString desc;
    const OmMFNode &kids = p->children();
    for (int i = 0; i < kids.size(); ++i) {
      const QString d = registerNewtonShapesRec(
          newton, idx, dynamic_cast<OmBaseNode *>(kids.item(i)), childX, softKe, solidMu, solidMuT, solidMuR);
      if (!d.isEmpty())
        desc += (desc.isEmpty() ? "" : "; ") + d;
    }
    return desc;
  }
  if (const OmGroup *g = dynamic_cast<const OmGroup *>(bo)) {
    QString desc;
    const OmMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i) {
      const QString d = registerNewtonShapesRec(
          newton, idx, dynamic_cast<OmBaseNode *>(kids.item(i)), x, softKe, solidMu, solidMuT, solidMuR);
      if (!d.isEmpty())
        desc += (desc.isEmpty() ? "" : "; ") + d;
    }
    return desc;
  }
  return addNewtonPrimitive(newton, idx, bo, x, softKe, solidMu, solidMuT, solidMuR);
}

// Per-Solid tangential friction (Solid.newtonFriction). Returns a NEGATIVE
// sentinel when the field is absent or unset, which the runtime reads as
// "inherit WorldInfo.newtonGroundMu" -- so every world that does not declare
// it is byte-for-byte unchanged. Null-checked because derived node types need
// not redeclare Solid's field set.
static double newtonFrictionForSolid(const OmSFDouble *f) {
  return (f == nullptr) ? -1.0 : f->value();
}

static QString attachNewtonShapeFromBoundingObject(OmNewtonBackend *newton, int idx,
                                                   OmBaseNode *boundingObjectValue,
                                                   double softKe = -1.0,
                                                   double solidMu = -1.0,
                                                   double solidMuT = -1.0,
                                                   double solidMuR = -1.0) {
  // OPT-IN two ways (mirrors newtonStatics / newtonRobotColliders): register
  // every collider in a compound boundingObject (Group of offset primitives on
  // one rigid body) instead of just the first child, via the launch env var OR
  // the per-world WorldInfo.newtonCompoundColliders field -- so a demo world folds
  // the knob into the .wbt and "just works" in the GUI / launcher / headless with
  // no env var. Per-call (NOT static) so a world switched in via the launcher's
  // worldReload reads ITS OWN field; the defaults keep every existing world's
  // physics byte-for-byte unchanged.
  const bool compound = newtonCompoundCollidersOn();
  if (compound) {
    const QString d = registerNewtonShapesRec(newton, idx, boundingObjectValue,
                                              OmNewtonShapeXform(), softKe, solidMu,
                                              solidMuT, solidMuR);
    if (!d.isEmpty())
      return d;
    // fall through to the single-shape walker if nothing matched
  }
  QString shapeDesc;
  OmBaseNode *bo = boundingObjectValue;
  OmNewtonShapeXform shapeX;  // identity: no offset, no rotation
  for (int unwrap = 0; unwrap < 4 && bo != nullptr; ++unwrap) {
    if (dynamic_cast<const OmSphere *>(bo) ||
        dynamic_cast<const OmBox *>(bo) ||
        dynamic_cast<const OmCylinder *>(bo) ||
        dynamic_cast<const OmCapsule *>(bo) ||
        dynamic_cast<const OmTriangleMeshGeometry *>(bo))
      break;
    if (const OmShape *sh = dynamic_cast<const OmShape *>(bo)) {
      // `boundingObject Shape { geometry Box {..} }` (and the common
      // `boundingObject USE <visualShape>` idiom) wraps the primitive in
      // a Shape. ODE resolves this natively; Newton's walker previously
      // didn't, fell through to the r=0.12 placeholder sphere, and every
      // such body rested 0.12 m up / got ejected at spawn. Descend into
      // the Shape's geometry and re-test.
      bo = sh->geometry();
      continue;
    }
    if (const OmPose *p = dynamic_cast<const OmPose *>(bo))
      shapeX = composeNewtonShapePose(shapeX, p);  // OmPose extends OmGroup; falls through
    if (const OmGroup *g = dynamic_cast<const OmGroup *>(bo)) {
      const OmMFNode &kids = g->children();
      if (kids.size() == 0)
        break;
      // ⚠ THE SILENT DROP (internal parity plan, item W1.6). Children 1..n-1 of a
      // compound boundingObject are discarded here, so a Group of primitives
      // collides as its FIRST primitive only. A Chair.proto collides as a
      // floating seat slab with no legs; 120 compound objects across 91 files
      // are in this state, and it just bit a shipped sample world. Nothing said
      // so -- the world loaded clean, the body had a collider, and only the
      // geometry was wrong.
      //
      // WARN, do NOT flip the default. WorldInfo.newtonCompoundColliders also
      // selects the INERTIA source further down, so flipping it silently
      // changes the inertia tensor of every dynamic multi-collider body in the
      // tree. Decoupling those two is separate, larger work; until it is done a
      // named warning is the honest half.
      // `!compound` matters: with the opt-in ON this walker is only reached
      // when the recursive registration matched nothing at all, and telling
      // that author to "set newtonCompoundColliders TRUE" would be nonsense.
      if (kids.size() > 1 && !compound) {
        static QSet<int> warnedGroupIds;
        const int gid = g->uniqueId();
        if (!warnedGroupIds.contains(gid)) {
          warnedGroupIds.insert(gid);
          const OmSolid *const owner = g->upperSolid();
          OmLog::warning(
            QObject::tr("The boundingObject of '%1' is a Group of %2 collision shapes, but only the FIRST is "
                        "registered with the physics engine -- the other %3 are silently DROPPED, so this body "
                        "collides as a fraction of its own shape. Set WorldInfo.newtonCompoundColliders TRUE to "
                        "register all of them. (That field also switches this body's inertia source, so expect the "
                        "dynamics to change as well as the collision.)")
              .arg(owner != nullptr ? owner->usefulName() : g->usefulName())
              .arg(kids.size())
              .arg(kids.size() - 1),
            false, OmLog::ODE);
        }
      }
      bo = dynamic_cast<OmBaseNode *>(kids.item(0));
      continue;
    }
    break;
  }
  // Aliases so the shape arms below read unchanged. The quaternion is in
  // newton/warp's (x,y,z,w) order; OmQuaternion stores (w,x,y,z).
  const OmVector3 &shapeOffset = shapeX.t;
  const double sqx = shapeX.q.x(), sqy = shapeX.q.y(), sqz = shapeX.q.z(), sqw = shapeX.q.w();
  if (const OmSphere *sphere = dynamic_cast<const OmSphere *>(bo)) {
    const double radius = sphere->radius();
    // Sphere: rotation-invariant, offset only.
    newton->addShapeSphere(idx, radius, shapeOffset.x(), shapeOffset.y(), shapeOffset.z(), solidMu,
                           solidMuT, solidMuR);
    shapeDesc = QString("sphere r=%1 at (%2,%3,%4)").arg(radius)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
  } else if (const OmBox *box = dynamic_cast<const OmBox *>(bo)) {
    const OmVector3 &sz = box->size();
    newton->addShapeBox(idx, sz.x() * 0.5, sz.y() * 0.5, sz.z() * 0.5,
                        shapeOffset.x(), shapeOffset.y(), shapeOffset.z(), softKe,
                        sqx, sqy, sqz, sqw, solidMu, solidMuT, solidMuR);
    shapeDesc = QString("box hx=%1 hy=%2 hz=%3 at (%4,%5,%6) q=(%7,%8,%9,%10)")
                    .arg(sz.x() * 0.5).arg(sz.y() * 0.5).arg(sz.z() * 0.5)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z())
                    .arg(sqx).arg(sqy).arg(sqz).arg(sqw);
  } else if (const OmCylinder *cyl = dynamic_cast<const OmCylinder *>(bo)) {
    // W1.2 (newton-ode-replacement-plan.md): hand the cylinder to Newton as a CAPSULE of the same
    // radius + half-height (addShapeCylinder substitutes a capsule -- native cylinder narrow-phase locks
    // wheels against the ground, probe 7). A capsule's central segment is a true cylinder of the correct
    // rolling radius, so this is a LINE contact of the right width -- strictly more faithful than the old
    // point-contact sphere, and the capsule narrow-phase is robust.
    //
    // Both an OmniSim Cylinder and a newton capsule are Z-ALIGNED, so the substitution needs NO axis
    // correction -- the only orientation that belongs here is the authored one composed above. (The
    // backend used to invent a fixed -90 deg about X on the false premise that Webots cylinders run
    // along body-local Y; see the SHAPE XFORM CONVENTION note in OmNewtonBackend.hpp.)
    const double radius = cyl->radius();
    const double halfHeight = cyl->height() * 0.5;
    newton->addShapeCylinder(idx, radius, halfHeight, shapeOffset.x(), shapeOffset.y(), shapeOffset.z(),
                             sqx, sqy, sqz, sqw);
    shapeDesc = QString("cylinder->capsule r=%1 hh=%2 at (%3,%4,%5) q=(%6,%7,%8,%9)").arg(radius).arg(halfHeight)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z())
                    .arg(sqx).arg(sqy).arg(sqz).arg(sqw);
  } else if (const OmCapsule *cap = dynamic_cast<const OmCapsule *>(bo)) {
    const double radius = cap->radius();
    const double halfHeight = cap->height() * 0.5;
    // Was offset-less AND orientation-less -- an authored Capsule collided at the body origin,
    // unrotated, wherever the .wbt actually put it.
    newton->addShapeCapsule(idx, radius, halfHeight, shapeOffset.x(), shapeOffset.y(), shapeOffset.z(),
                            sqx, sqy, sqz, sqw);
    shapeDesc = QString("capsule r=%1 hh=%2 at (%3,%4,%5) q=(%6,%7,%8,%9)").arg(radius).arg(halfHeight)
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z())
                    .arg(sqx).arg(sqy).arg(sqz).arg(sqw);
  } else if (dynamic_cast<OmPlane *>(bo) != nullptr) {
    // Infinite ground plane (newton-ode-replacement-plan.md W1.1) -- a Floor's Plane boundingObject. Newton
    // add_shape_plane, local normal +Z (the OmPlane convention); the body's transform orients it. Without
    // this the static floor had NO Newton collider, so Newton-dynamic props fell straight through it (the
    // faithful-check finding). Takes effect when the floor is a Newton static body (OMNISIM_NEWTON_STATICS).
    newton->addShapePlane(idx, shapeOffset.x(), shapeOffset.y(), shapeOffset.z());
    shapeDesc = QString("plane off=(%1,%2,%3)").arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
  } else if (const OmElevationGrid *const eg = dynamic_cast<const OmElevationGrid *>(bo)) {
    // NATIVE heightfield terrain. An ElevationGrid is NOT an OmTriangleMeshGeometry (it derives
    // straight from OmGeometry), so before this arm existed it fell off the end of this chain and
    // registered NO collider whatsoever -- silently, with no warning and exit 0. Measured on
    // tests/benchmarks/omnibench/lane4/worlds/object_elevationgrid_terrain.wbt: a 1 kg sphere
    // dropped on a flat grid reached z = -42.89 m and was STILL ACCELERATING at t = 3 s. (Until
    // 2026-08-12 the implicit z=0 ground plane masked this as a rest at z = 0.0996, which is why a
    // broken terrain collider survived so long.)
    //
    // height[] is handed over in the node's own row-major order (index = y*xDimension + x); the
    // backend converts OmniSim's cell-spacing + dimension-count into newton's half-extents and
    // applies the VRML corner-origin -> newton centre-origin shift.
    const int xDim = eg->xDimension();
    const int yDim = eg->yDimension();
    if (xDim >= 2 && yDim >= 2) {
      QVector<double> heights(xDim * yDim);
      for (int i = 0; i < xDim * yDim; ++i)
        heights[i] = eg->height(i);
      newton->addShapeHeightfield(idx, heights.constData(), xDim, yDim,
                                  eg->xSpacing(), eg->ySpacing(),
                                  shapeOffset.x(), shapeOffset.y(), shapeOffset.z());
      shapeDesc = QString("heightfield %1x%2 spacing=(%3,%4) at (%5,%6,%7)")
                    .arg(xDim).arg(yDim).arg(eg->xSpacing()).arg(eg->ySpacing())
                    .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z());
    } else {
      // Not a warning-free silent drop any more: say which field is wrong.
      OmLog::warning(QString("ElevationGrid boundingObject needs xDimension and yDimension >= 2 to "
                             "be a collider (got %1 x %2); it will not collide. Set both to at least 2 (they count height samples, so 'height' must then hold xDimension*yDimension values) -- see docs/reference/elevationgrid.md.")
                       .arg(xDim).arg(yDim));
    }
  } else if (OmTriangleMeshGeometry *const tmg = dynamic_cast<OmTriangleMeshGeometry *>(bo)) {
    // NATIVE triangle-mesh collision (newton-ode-replacement-plan.md W1): hand the mesh's vertices +
    // triangle indices straight to Newton (add_shape_mesh) instead of the old AABB-box approximation. The
    // verts are in the geometry's local frame; the composed shape xform places that frame in the body
    // -- offset AND rotation, matching the primitive shapes (it was translation-only, which tipped any
    // mesh collision authored inside a rotated Pose, as the URDF importer routinely emits). Falls back
    // to the AABB box if the mesh data isn't available.
    OmTriangleMesh *const tm = tmg->triangleMesh();
    if (tm != nullptr && tm->numberOfVertices() > 0 && tm->numberOfTriangles() > 0) {
      newton->addShapeMesh(idx, tm->coordinatesData(), tm->numberOfVertices(), tm->indicesData(),
                           tm->numberOfTriangles(), shapeOffset.x(), shapeOffset.y(), shapeOffset.z(),
                           sqx, sqy, sqz, sqw);
      shapeDesc = QString("mesh verts=%1 tris=%2 off=(%3,%4,%5) q=(%6,%7,%8,%9)")
                      .arg(tm->numberOfVertices()).arg(tm->numberOfTriangles())
                      .arg(shapeOffset.x()).arg(shapeOffset.y()).arg(shapeOffset.z())
                      .arg(sqx).arg(sqy).arg(sqz).arg(sqw);
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
static int countNewtonCompoundPrimitives(const OmBaseNode *bo) {
  if (bo == nullptr)
    return 0;
  if (const OmShape *sh = dynamic_cast<const OmShape *>(bo))
    return countNewtonCompoundPrimitives(sh->geometry());
  if (const OmGroup *g = dynamic_cast<const OmGroup *>(bo)) {  // also covers OmPose
    int n = 0;
    const OmMFNode &kids = g->children();
    for (int i = 0; i < kids.size(); ++i)
      n += countNewtonCompoundPrimitives(dynamic_cast<OmBaseNode *>(kids.item(i)));
    return n;
  }
  if (dynamic_cast<const OmSphere *>(bo) || dynamic_cast<const OmBox *>(bo) ||
      dynamic_cast<const OmCylinder *>(bo) || dynamic_cast<const OmCapsule *>(bo))
    return 1;
  return 0;
}

// The ONE resolution of "what ground friction did this world ask for" on the
// Newton path, used by both the head-of-flush plumb (which must run before any
// shape is created -- newton copies cfg.mu into shapes at add time) and the
// tail re-assert. Precedence:
//   1. WorldInfo.newtonGroundMu >= 0 -- the native declaration (0 = frictionless).
//   2. WorldInfo.contactProperties.coulombFriction, BRIDGED only when the
//      world EXPLICITLY pinned the Newton backend (defaultPhysicsBackend
//      "newton" -- a world that chose Newton on purpose means its declared
//      friction) or when OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES is set
//      (value-parsed: "0"/"false"/"off" = off). An ordinary "auto" world is
//      NEVER re-frictioned -- 240 live worlds declare coulombFriction and were
//      tuned under the effective mu of 1.0; silently adopting their values
//      would change the physics of every one of them.
//   3. NEGATIVE = unset -> the runtime default (env var, else 1.0).
//      NOT 0: mu=0 is a legal declaration (a frictionless world), and
//      using 0 as the sentinel made it unsayable in a .wbt at all.
// *bridgedOut reports whether case 2 fired, so the "your coulombFriction is
// ignored" warning can stay honest.
static double resolvedNewtonGroundMu(const OmWorldInfo *wi, bool *bridgedOut = nullptr) {
  if (bridgedOut != nullptr)
    *bridgedOut = false;
  if (wi == nullptr)
    return -1.0;
  const double declared = wi->newtonGroundMu();
  if (declared >= 0.0)
    return declared;
  const bool pinned = wi->defaultPhysicsBackend().compare("newton", Qt::CaseInsensitive) == 0;
  const QString optRaw = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES")).trimmed().toLower();
  const bool optIn = !optRaw.isEmpty() && optRaw != "0" && optRaw != "false" && optRaw != "off" && optRaw != "no";
  if (!pinned && !optIn)
    return -1.0;
  for (int i = 0; i < wi->contactPropertiesCount(); ++i) {
    const OmContactProperties *const cp = wi->contactProperties(i);
    if (cp != nullptr && cp->coulombFrictionSize() > 0 && cp->coulombFriction(0) > 0.0) {
      if (bridgedOut != nullptr)
        *bridgedOut = true;
      static bool sBridgeLogged = false;
      if (!sBridgeLogged) {
        sBridgeLogged = true;
        OmLog::info(QObject::tr("[OmNewtonBackend] contactProperties.coulombFriction %1 bridged to Newton ground "
                                "friction (%2). Declare WorldInfo.newtonGroundMu to control this directly.")
                      .arg(cp->coulombFriction(0))
                      .arg(pinned ? "world pins defaultPhysicsBackend \"newton\""
                                  : "OMNISIM_NEWTON_BRIDGE_CONTACT_PROPERTIES"));
      }
      return cp->coulombFriction(0);
    }
  }
  return -1.0;
}

void OmSolid::captureNewtonVelocitiesForRebuild() {
  // W1.7: postPhysicsStep refreshes mLinearVelocity/mAngularVelocity from
  // the solver every tick, so the FIELDS hold the live values. Stash them in
  // the FIX-5 replay slots; re-registration replays them into the fresh
  // Newton world (set_body_vel queues pre-finalize and drains after
  // finalize's closing eval_fk). Dynamic registered bodies only -- statics
  // do not move and kinematic bodies are driven from their fields.
  for (const OmSolid *cs : cSolids) {
    OmSolid *const s = const_cast<OmSolid *>(cs);
    if (s->mNewtonBodyIndex < 0 || s->mNewtonBodyIsStatic || s->mNewtonBodyIsKinematic)
      continue;
    if (s->mLinearVelocity) {
      const OmVector3 &lv = s->mLinearVelocity->value();
      s->mPendingNewtonLinVel[0] = lv.x();
      s->mPendingNewtonLinVel[1] = lv.y();
      s->mPendingNewtonLinVel[2] = lv.z();
      s->mPendingNewtonLinVelValid = true;
    }
    if (s->mAngularVelocity) {
      const OmVector3 &av = s->mAngularVelocity->value();
      s->mPendingNewtonAngVel[0] = av.x();
      s->mPendingNewtonAngVel[1] = av.y();
      s->mPendingNewtonAngVel[2] = av.z();
      s->mPendingNewtonAngVelValid = true;
    }
  }
}

void OmSolid::resetNewtonRegistrationsForRebuild() {
  // W1.7: forget every Newton registration so the next flush re-registers
  // the whole scene into the fresh world. Registration reads the LIVE world
  // transform (matrix().translation() / rotationMatrix()), which the
  // per-tick pose readback keeps current, so a rebuild re-seeds bodies at
  // their current pose -- never the authored one.
  int droppedWelds = 0;
  for (const OmSolid *cs : cSolids) {
    OmSolid *const s = const_cast<OmSolid *>(cs);
    s->mNewtonBodyIndex = -1;
    s->mNewtonBodyIsStatic = false;
    s->mNewtonBodyIsKinematic = false;
    s->mNewtonKinPoseValid = false;
    s->mLastNewtonXformValid = false;
    // Weld slots index the OLD Newton world; the flush's weld-slot sweep
    // re-reserves fresh ones while the rebuilt world is open for build.
    if (OmConnector *const c = dynamic_cast<OmConnector *>(s)) {
      if (c->resetNewtonWeldSlotForRebuild())
        ++droppedWelds;
    } else if (OmVacuumGripper *const v = dynamic_cast<OmVacuumGripper *>(s)) {
      if (v->resetNewtonWeldSlotForRebuild())
        ++droppedWelds;
    }
  }
  if (droppedWelds > 0)
    OmLog::warning(QObject::tr(
      "[OmNewtonBackend] physics rebuild dropped %1 ENGAGED weld(s) (Connector locks / "
      "VacuumGripper grips). Held objects are released; re-lock or re-grip from the "
      "controller after the rebuild. Re-engaging welds across a rebuild is not yet "
      "implemented.").arg(droppedWelds));
}

void OmSolid::flushPendingNewtonRegistrations() {
  // Resolve once outside the loop -- isAvailable() check is cheap but
  // we don't need a separate registry hit per Solid.
  OmPhysicsBackend *const raw = OmPhysicsBackendRegistry::newtonBackend();
  if (raw == nullptr || !raw->isAvailable())
    return;
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(raw);

  // GATE (physics-step-cost-optimization-plan.md §3 item 2). Once the world is
  // running, a full pass that registered NOTHING means every Solid then in the
  // scene has been dispositioned -- registered, or declined for a reason that
  // cannot change without one of the generation bumps above. Re-walking the
  // whole scene graph every tick after that is pure cost (0.097 ms/tick on the
  // 8-Husky world, 11% of the tick).
  //
  // Deliberately NOT gated while the world is still building: the pre-run
  // passes are where registration actually happens, and the contact/coordinate
  // caching below must reach ensureWorldOpen() before the ground plane exists.
  static int sLastCleanGeneration = -1;
  if (newton->isWorldRunning() && sLastCleanGeneration == cNewtonFlushGeneration)
    return;

  // Cache the world's contact/solver declaration BEFORE anything registers.
  // newton's ModelBuilder copies cfg.mu/ke/kd into each shape AT ADD TIME, so
  // the old tail-of-flush plumb reached nothing: a declared
  // WorldInfo.newtonGroundMu 2.0 left the box sliding at the default on the
  // 55-degree ramp comparator, and friction_grasp_minimal's declared
  // mu/ke/kd never functioned (it held on other fields). The world is not
  // open yet, so this call only CACHES (rc -1 expected); ensureWorldOpen()
  // applies the cache right after constructing the world, before the ground
  // plane. The tail plumb below is kept as the finalize-time re-assert.
  {
    const OmWorldInfo *const cwi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
    if (cwi != nullptr) {
      // WorldInfo.coordinateSystem -> the Newton builder's up axis. SAME REASON
      // AND SAME SEAM as the contact params below, one level more fundamental:
      // newton reads builder.up_vector when it composes the implicit ground
      // plane's normal, and setWorldGravity() projects the world's gravity
      // vector onto that same up vector -- so if the axis is not in place before
      // ensureWorldOpen() adds the plane, neither is fixable afterwards.
      //
      // Until 2026-08-08 nothing read this field on the Newton path at all. The
      // 210 NUE (Y-up) worlds in this tree -- 29% of 719, and every one of them
      // on a bare "auto" backend -- therefore ran at gravity ZERO (the
      // projection of (0,-g,0) onto the hardcoded (0,0,1)) with an infinite
      // plane whose normal pointed along their EAST axis, i.e. a vertical wall
      // through the scene. ODE used to mask it as the fall-back backend; a log-only
      // PASS could never see it. The field is passed VERBATIM -- the runtime
      // resolves the axis from where the "U" sits, the same rule
      // OmWorldInfo::updateGravityBasis() uses to build mUpVector.
      newton->setCoordinateSystem(cwi->coordinateSystem().toStdString());
      newton->setContactSolverParams(resolvedNewtonGroundMu(cwi), cwi->newtonContactKe(), cwi->newtonContactKd(),
                                     cwi->newtonIterations(), cwi->newtonLsIterations());
    }
  }

  // Static colliders this flush declined to register (WorldInfo.newtonStatics
  // is FALSE). Collected so the warning below can NAME them -- an intangible
  // floor is otherwise indistinguishable from a working one.
  QStringList skippedStaticNames;

  // Count the bodies THIS flush actually registers. flushPendingNewtonRegistrations()
  // is called every tick (OmSimulationWorld::step), but registration is a one-shot
  // per Solid (mNewtonBodyIndex >= 0 short-circuits below), so on all but the first
  // few ticks of a world this stays 0 -- which is what gates the census log at the
  // bottom of this function. Without it the census re-fired on EVERY tick and
  // flooded the agent-facing controller.log stream on GET /sim/events.
  int registeredThisFlush = 0;

  // Force-type TouchSensors exempted from the P3.10c fold this flush
  // (OMNISIM_NEWTON_TOUCH_FORCE) -- registered in a SECOND pass after the
  // main loop so their merge leader's body index already exists regardless
  // of cSolids ordering.
  QVector<OmSolid *> pendingUnfoldSensors;

  for (const OmSolid *const cs : cSolids) {
    // Iteration is over const pointers (cSolids is List<const OmSolid*>);
    // we mutate via the const-cast since mNewtonBodyIndex is a private
    // bookkeeping field. Acceptable here because the underlying list
    // entries aren't truly const -- they're just stored that way.
    OmSolid *const s = const_cast<OmSolid *>(cs);
    if (s == nullptr || s->mNewtonBodyIndex >= 0)
      continue;
    // Pick up the inherited backend from any ancestor Solid (e.g. URDFRobot-
    // generated chassis Solids inherit from the outer Robot). Every value
    // resolves to Newton -- a retired "ode" pin warns once per world inside
    // effectivePhysicsBackendName() and registers like everything else (until
    // 2026-09-02 it skipped registration here and the Solid had NO physics).
    // The only thing that keeps a Solid out of Newton is the §4.1 capability
    // gate, and that is a SILENT omission the enforcement below has to name.
    bool gatedFromNewton = false;
    s->effectivePhysicsBackendName(&gatedFromNewton);
    if (gatedFromNewton) {
      // Newton enforcement (2026-06-29 default: no silent drop-out). A
      // bare-"auto" articulation that the §4.1 capability gate kept off Newton
      // -- because it uses a Newton-unsupported feature -- would otherwise be
      // simulated by nothing while the world reports Newton.
      if (OmPhysicsBackendRegistry::newtonEnforced()) {
        const char *reason = "capable";
        s->articulationNewtonCapable(&reason);
        if (qstrcmp(reason, "kinematic") == 0) {
          // A KINEMATIC chain (a joint whose endPoint has no Physics node) has
          // no dynamics for Newton to degrade -- classic Webots animates it
          // through the scene tree and ODE's kinematic path, and that is what
          // it gets. This is a WARNING, not a FATAL, because the enforcement's
          // crime is a QUIET hybrid; a named routing is not quiet. It is also
          // temporary by decision: when ODE is deleted, kinematic articulation
          // needs native handling (ode-retirement-campaign.md, kernel blocker
          // #4), and this warning marks every world that needs it.
          static QSet<QString> sWarnedKinematic;
          if (!sWarnedKinematic.contains(s->name())) {
            sWarnedKinematic.insert(s->name());
            OmLog::warning(tr("[capability-gate] Solid '%1' is a KINEMATIC articulation (a joint's endPoint has no "
                              "Physics node): it animates kinematically -- the scene tree moves and no solver "
                              "simulates it -- rather than through Newton dynamics. Give the endPoint a Physics "
                              "node to simulate it dynamically.")
                             .arg(s->name()),
                           false, OmLog::ODE);
          }
        } else {
          OmLog::fatal(
              tr("[newton-enforce] Solid '%1' would silently drop out of the simulation: its \"auto\" "
                 "articulation uses a Newton-unsupported feature (%2), so the capability gate kept it "
                 "off Newton -- and Newton is the only backend, so it would run with no physics at all. "
                 "Fix: make the feature Newton-compatible.")
                  .arg(s->name(), QString::fromUtf8(reason)));
        }
      }
      continue;
    }

    // P3.10c: skip Solids that ODE would fold into a parent solid via
    // its OmSolidMerger -- i.e. Physics-bearing Solids that hang
    // directly off another Solid without an intervening joint. URDF
    // "fixed" joints, Group/Pose nesting, and the URDFRobot expansion's
    // bumper/top-plate/IMU children all land here. ODE's OmSolidMerger
    // hasn't run yet by flush time so we can't query it directly;
    // instead walk the tree -- if the first OmSolid or OmBasicJoint
    // ancestor is a OmSolid (no intervening joint), this is a fixed
    // child and shouldn't get its own Newton body. OmBasicJoint
    // endpoint Solids (wheels, articulated end-effectors) get their
    // own bodies as expected because the joint is what we hit first
    // walking up.
    bool isFixedChild = false;
    for (OmNode *n = s->parentNode(); n != nullptr; n = n->parentNode()) {
      if (dynamic_cast<OmBasicJoint *>(n))
        break;  // joint ancestor first -- this Solid is a real articulated body
      if (dynamic_cast<OmSolid *>(n)) {
        isFixedChild = true;
        break;
      }
    }
    if (isFixedChild) {
      // T1 un-fold (_scratch/design_weld_touch.md): a FORCE-type TouchSensor
      // must NOT be folded -- its "force"/"force-3d" value is the MOUNT-JOINT
      // reaction (the canonical world expects 981 N for a 100 kg robot atop a
      // 100 kg sensor, i.e. only what presses THROUGH the mount), which is
      // only readable if the sensor is its own mjc body welded to the leader
      // (cfrc_int). Folding also DROPS its boundingObject entirely (only the
      // leader's own mBoundingObject registers), so a folded sensor is not
      // even a collider. Deferred to a second pass below so the leader's body
      // exists first. Gated by OMNISIM_NEWTON_TOUCH_FORCE (default OFF).
      if (isUnfoldedContactDevice(s)) {
        pendingUnfoldSensors.append(s);
        continue;
      }
      // ...UNLESS the fold has nowhere to go. Folding assumes the merge
      // leader (the topmost joint-free Solid ancestor) registers a Newton
      // body that absorbs this child's mass and joints. A JOINTED Solid
      // nested under a bare decorative wrapper -- motor2/motor3's pattern:
      //   Solid { children [ Robot { physics ... Hinge2Joint ... } ] }
      // folded into a leader with NO physics anywhere above it, so the
      // leader registered nothing, this Robot registered nothing, and its
      // joints FATALed under newton-enforce ("parent body resolved to
      // Newton but never registered"). If this Solid carries physics AND
      // joints, and no ancestor up the joint-free chain carries physics,
      // it IS the effective dynamic body: let it register itself.
      bool ancestorHasPhysics = false;
      for (OmNode *n = s->parentNode(); n != nullptr; n = n->parentNode()) {
        if (dynamic_cast<OmBasicJoint *>(n))
          break;
        const OmSolid *const as = dynamic_cast<const OmSolid *>(n);
        if (as != nullptr && as->physics() != nullptr) {
          ancestorHasPhysics = true;
          break;
        }
      }
      if (!(s->physics() != nullptr && !s->mJointChildren.isEmpty() && !ancestorHasPhysics))
        continue;
      // fall through: register this jointed, physics-bearing Solid as its own body
    }

    // staticBase HUMANOID special case (G1 / H1 / Atlas seated or pinned).
    // For `staticBase TRUE` the URDF importer strips ONLY the *root* link's
    // Physics block (OmUrdfImporter), which is enough for a fixed-base arm
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
        dynamic_cast<const OmRobot *>(s) != nullptr &&
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
    // step, its OmSolid::syncNewtonPoseFromFields bridge fires
    // resetJointsToDefaults() on the SHARED webots_world articulation
    // every tick -- snapping every real robot's joints + bodies back to
    // spawn and freezing them in place (the husky chassis-freeze bug).
    if (!subtreeHasPhysics(s) || staticBaseRobot) {
      // OMNISIM_NEWTON_KINEMATIC (value-parsed, default OFF) -- kernel
      // blocker #4, _scratch/design_kinematic_inertia.md Part 1. A
      // physics-less JOINT ENDPOINT is animated by the ENGINE
      // (OmMotor::runKinematicControl -> updatePosition writes the scene
      // tree; a Supervisor jointParameters.position write takes the same
      // path); the solver's only job for it is COLLISION. Register every
      // collider in its joint-free subtree as a KINEMATIC (fixed-root ->
      // MuJoCo MOCAP) Newton body at its world pose; the engine pushes pose
      // updates through setKinematicPose (syncNewtonPoseFromFields on field
      // writes + the prePhysicsStep refresh for ancestor motion), and
      // mocap-vs-dynamic contact reproduces ODE's one-way kinematic coupling
      // (pushes/supports dynamics, never pushed back). The joint itself is
      // NEVER registered with Newton (OmBasicJoint::postFinalize skips it),
      // exactly as ODE never created a dJoint for it. Without the flag these
      // articulations are gated to ODE before reaching this loop, so this
      // clause is unreachable and behavior is byte-identical.
      if (OmSolid::newtonKinematicNativeEnabled() && !staticBaseRobot &&
          s->upperPose() != nullptr && s->jointParent() != nullptr && s->physics() == nullptr) {
        QVector<OmSolid *> kinColliders;
        QVector<OmNode *> kwalk;
        kwalk.append(s);
        while (!kwalk.isEmpty()) {
          OmNode *const node = kwalk.takeLast();
          if (OmSolid *const sol = dynamic_cast<OmSolid *>(node)) {
            if (sol->mBoundingObject != nullptr && sol->mBoundingObject->value() != nullptr)
              kinColliders.append(sol);
          }
          if (const OmGroup *const g = dynamic_cast<const OmGroup *>(node)) {
            const OmMFNode &kids = g->children();
            for (int i = 0; i < kids.size(); ++i) {
              OmNode *const kid = kids.item(i);
              if (kid != nullptr && dynamic_cast<OmBasicJoint *>(kid) == nullptr)
                kwalk.append(kid);
            }
          }
        }
        if (kinColliders.isEmpty())
          continue;  // nothing collidable -- pure scene-tree animation suffices
        if (newton->ensureWorldOpen() != 0)
          return;
        for (OmSolid *const kc : kinColliders) {
          const OmVector3 kt = kc->matrix().translation();
          const OmQuaternion kq = OmRotation(kc->rotationMatrix()).toQuaternion();
          const int kidx = newton->addKinematicBody(kt.x(), kt.y(), kt.z(),
                                                    kq.x(), kq.y(), kq.z(), kq.w());
          if (kidx >= 0) {
            attachNewtonShapeFromBoundingObject(
                newton, kidx, dynamic_cast<OmBaseNode *>(kc->mBoundingObject->value()),
                newtonSoftKeForMaterial(kc->mContactMaterial),
                newtonFrictionForSolid(kc->mNewtonFriction),
                newtonFrictionForSolid(kc->mNewtonFrictionTorsional),
                newtonFrictionForSolid(kc->mNewtonFrictionRolling));
            kc->mNewtonBodyIndex = kidx;
            // Static protections (no per-step pose readback, no
            // resetBodyPose/resetJointsToDefaults) PLUS the kinematic
            // pose-push path.
            kc->mNewtonBodyIsStatic = true;
            kc->mNewtonBodyIsKinematic = true;
            ++registeredThisFlush;
          }
        }
        continue;
      }
      // staticBase robot root (a static-base arm, bolted-down manipulator, ...): the URDF
      // importer strips the base link's Physics block for `staticBase TRUE`
      // (the kinematic "bolted to the floor" root, OmUrdfImporter). That
      // leaves this OmRobot wrapper looking like furniture -- and skipped
      // below -- so its base->link1 hinge had no Newton parent body and was
      // dropped (OmBasicJoint flush needs a body on BOTH ends). The arm
      // articulation then had no root and Newton never stepped it: the arm
      // froze at its spawn pose under Newton while actuating fine under ODE.
      // Register the base as a Newton STATIC (mass=0) body so the root hinge
      // attaches; finalize() anchors a static root with a FIXED joint (not a
      // 6-DOF free joint), giving the arm a pinned root to articulate off.
      // Default-on (staticBase arms are common), unlike env-gated scene
      // statics. Only staticBase robots reach here -- a normal robot's base
      // keeps its Physics so subtreeHasPhysics() is true and it takes the
      // dynamic addBody/free-root path below, unchanged.
      if (dynamic_cast<const OmRobot *>(s) != nullptr) {
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
        const OmVector3 bt = s->matrix().translation();
        const OmQuaternion bq = OmRotation(s->rotationMatrix()).toQuaternion();
        const int bidx = newton->addStaticBody(bt.x(), bt.y(), bt.z(),
                                               bq.x(), bq.y(), bq.z(), bq.w());
        if (bidx >= 0) {
          s->mNewtonBodyIndex = bidx;
          s->mNewtonBodyIsStatic = true;
          ++registeredThisFlush;
        }
        continue;
      }
      // P8.2 (statics-on-Newton): ON BY DEFAULT since 2026-08-07. A top-level
      // static collider (has a boundingObject, no joint parent, opted into
      // Newton) registers as a mass=0 STATIC Newton body so it's a collision
      // surface for dynamic Newton bodies, instead of staying ODE-only.
      // mNewtonBodyIsStatic flags it so the per-step pose writeback +
      // syncNewtonPoseFromFields (incl. the articulation-wide
      // resetJointsToDefaults that caused the freeze) are both skipped -- a
      // pinned body has nothing to write back and must never touch the shared
      // articulation.
      //
      // WHY THE DEFAULT FLIPPED. The old default ("skip" -- statics ODE-side,
      // which in a Newton world means they collide with NOTHING) was the
      // quietest defect in the engine: a ball dropped from z=0.9 onto a Box
      // floor whose top is at z=0.55 passed straight through and settled at
      // z=0.0996 on the implicit z=0 plane -- a surface that is not in the
      // file. Nothing warned; the run exited 0. Worlds whose floor happens to
      // sit at z=0 were masked, which is how it survived. 47 worlds and ~30
      // launch scripts had already opted in explicitly, including every
      // lane-1 correctness world and every RL deploy launcher -- the flip
      // aligns the default with what everything measured actually ran.
      //
      // Precedence (pinned by tests/test_newton_static_floor_collides.py):
      //   WorldInfo.newtonStatics TRUE  -> ON, always (that world had statics
      //                                    before the flip too, so the hatch
      //                                    must not take them away)
      //   OMNISIM_NEWTON_STATICS=<val>  -> value-parsed ("0"/"false"/"off" =
      //                                    OFF -- the old presence-gated test
      //                                    made =0 mean ON, the exact inverted
      //                                    -hatch bug class F2 catalogued)
      //   unset                         -> ON (the flip)
      // OMNISIM_NEWTON_STATICS=0 therefore reproduces the pre-flip build
      // tree-wide with one variable, EXCEPT for worlds that declared the
      // field -- which is what makes it an exact revert, not a third state.
      {
        const OmWorldInfo *const wiStatics =
            OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
        bool staticsOn = true;
        if (wiStatics != nullptr && wiStatics->newtonStatics())
          staticsOn = true;
        else {
          const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_STATICS")).trimmed().toLower();
          if (!v.isEmpty())
            staticsOn = !(v == "0" || v == "false" || v == "off" || v == "no");
        }
        if (!staticsOn) {
          // This branch is now reachable only via the OMNISIM_NEWTON_STATICS=0
          // revert hatch. An intangible floor is still not a visible failure
          // (bodies just rest at the wrong height on the implicit plane), so
          // keep NAMING what was skipped even for a deliberate opt-out.
          if (s->boundingObject() != nullptr && s->upperPose() == nullptr)
            skippedStaticNames.append(s->name().isEmpty() ? QString("<unnamed>") : s->name());
          continue;
        }
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
      QVector<OmSolid *> staticColliders;
      QVector<OmNode *> walk;
      walk.append(s);
      while (!walk.isEmpty()) {
        OmNode *const node = walk.takeLast();
        if (OmSolid *const sol = dynamic_cast<OmSolid *>(node)) {
          if (sol->mBoundingObject != nullptr && sol->mBoundingObject->value() != nullptr)
            staticColliders.append(sol);
        }
        if (const OmGroup *const g = dynamic_cast<const OmGroup *>(node)) {
          const OmMFNode &kids = g->children();
          for (int i = 0; i < kids.size(); ++i) {
            OmNode *const kid = kids.item(i);
            if (kid != nullptr && dynamic_cast<OmBasicJoint *>(kid) == nullptr)
              walk.append(kid);
          }
        }
      }
      if (staticColliders.isEmpty())
        continue;
      if (newton->ensureWorldOpen() != 0)
        return;
      for (OmSolid *const sc : staticColliders) {
        const OmVector3 st = sc->matrix().translation();
        const OmQuaternion sq = OmRotation(sc->rotationMatrix()).toQuaternion();
        const int sidx = newton->addStaticBody(st.x(), st.y(), st.z(),
                                               sq.x(), sq.y(), sq.z(), sq.w());
        if (sidx >= 0) {
          attachNewtonShapeFromBoundingObject(
              newton, sidx, dynamic_cast<OmBaseNode *>(sc->mBoundingObject->value()),
              newtonSoftKeForMaterial(sc->mContactMaterial),
              newtonFrictionForSolid(sc->mNewtonFriction),
              newtonFrictionForSolid(sc->mNewtonFrictionTorsional),
              newtonFrictionForSolid(sc->mNewtonFrictionRolling));
          sc->mNewtonBodyIndex = sidx;
          sc->mNewtonBodyIsStatic = true;
          ++registeredThisFlush;
        }
      }
      continue;
    }

    // `newton` (resolved and availability-checked once above the loop) is the
    // backend for every Solid that reaches this point.
    if (newton->ensureWorldOpen() != 0)
      return;

    // World-space pose: matrix() walks the parent transform chain
    // (Solid -> HingeJoint -> Solid -> ... -> root), so wheels under a
    // chassis end up at their actual world position rather than the
    // local SFVec3f field value.
    const OmVector3 t = s->matrix().translation();
    const OmMatrix3 R = s->rotationMatrix();
    const OmQuaternion q = OmRotation(R).toQuaternion();

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
    // Physics node. OmPhysics::inertiaMatrix() returns 2 vec3s when
    // explicitly set: [(ixx, iyy, izz), (ixy, ixz, iyz)] (Webots'
    // 6-value upper-triangular convention). If the field is empty,
    // pass zeros and let the helper module fall back to the
    // chassis-vs-wheel preset.
    double ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    double cx = 0, cy = 0, cz = 0;
    bool hasCom = false;
    if (const OmPhysics *const phys = s->physics()) {
      const OmMFVector3 &im = phys->inertiaMatrix();
      if (im.size() >= 1) {
        const OmVector3 &diag = im.item(0);
        ixx = diag.x(); iyy = diag.y(); izz = diag.z();
      }
      if (im.size() >= 2) {
        const OmVector3 &off = im.item(1);
        ixy = off.x(); ixz = off.y(); iyz = off.z();
      }
      // OMNISIM_NEWTON_USE_LINK_COM (default off): opt-in true link COM; off =
      // COM at link origin (legacy, every existing Newton robot validated
      // against this). Rebuild-gated. When on, pass the Solid's centerOfMass
      // (OmPhysics, link/body frame) so the Newton body's COM matches the URDF
      // inertial origin -- the inertia tensor above is already about the COM
      // frame, so the pairing is physically correct.
      if (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_USE_LINK_COM")) {
        const OmMFVector3 &com = phys->centerOfMass();
        if (com.size() >= 1) {
          const OmVector3 &c = com.item(0);
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
      OmVector3 cCom(0.0, 0.0, 0.0);
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
    if (ixx <= 0.0 && iyy <= 0.0 && izz <= 0.0 && s->mBoundingObject != nullptr &&
        qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_LEGACY_INERTIA_PRESET")) {
      const bool compoundOn = newtonCompoundCollidersOn();   // W1.6: one shared, value-parsed read
      // FIX 2 (omnibench T3): a dynamic Solid with no explicit inertiaMatrix used
      // to fall through to the runtime's hard-coded Husky-wheel preset
      // diag(0.0094m, 0.0167m, 0.0094m) -- a m=1 r=0.1 sphere got Iyy 4.18x too
      // large (rolling accel 47.6% low). createOdeMass() has already integrated
      // the CORRECT tensor from the bounding object into odeMass()->I; feed it
      // whenever OMNISIM_NEWTON_LEGACY_INERTIA_PRESET is unset. OmRobot wrapper
      // bodies are EXCLUDED: their mass is a fixed-child rollup over an envelope
      // boundingObject and every wheeled-robot result was validated against the
      // preset -- they stay byte-identical. URDF links ship an explicit
      // inertiaMatrix (OmUrdfImporter) so robot links never reach this branch.
      // Scale by k = mass / om->mass: rolledUpMass may exceed the Solid's own
      // mass that odeMass() integrated.
      const int nPrims = countNewtonCompoundPrimitives(
          dynamic_cast<OmBaseNode *>(s->mBoundingObject->value()));
      if (dynamic_cast<const OmRobot *>(s) == nullptr && nPrims >= 1) {
        // ODE-RETIREMENT: the geometry-derived tensor now comes from the
        // ODE-free native composer (OmSolidUtilities::addInertia mirrored in
        // createOdeMass), parity-proven against the dMass pipeline it
        // replaces (tests/test_newton_native_inertia_parity.py, <=1e-12 on
        // primitives). OMNISIM_NEWTON_NATIVE_INERTIA=0 USED to revert to the
        // dMass feed while src/ode still shipped (value-parsed; default ON).
        // ODE is gone: the native composer is the ONLY tensor source;
        // OMNISIM_NEWTON_NATIVE_INERTIA=0 has no dMass feed to revert to and
        // is ignored.
        const bool useNative = s->mNativeInertiaValid;
        const double srcMass = useNative ? s->mNativeInertia.mass() : 0.0;
        const double srcIxx = useNative ? s->mNativeInertia.ixx() : 0.0;
        const double srcIyy = useNative ? s->mNativeInertia.iyy() : 0.0;
        const double srcIzz = useNative ? s->mNativeInertia.izz() : 0.0;
        const double srcIxy = useNative ? s->mNativeInertia.ixy() : 0.0;
        const double srcIxz = useNative ? s->mNativeInertia.ixz() : 0.0;
        const double srcIyz = useNative ? s->mNativeInertia.iyz() : 0.0;
        if (srcMass > 0.0 && srcIxx > 0.0 && srcIyy > 0.0 && srcIzz > 0.0) {
          const double k = mass / srcMass;
          ixx = srcIxx * k; iyy = srcIyy * k; izz = srcIzz * k;
          ixy = srcIxy * k; ixz = srcIxz * k; iyz = srcIyz * k;
          // Even the geometry-correct diagonal can be eig3-degenerate: a square bin
          // has Ixx==Iyy, and Izz is the largest so the diagonal is not descending --
          // newton's eig3 then resolves the tie / permutation to a rotated body_iquat
          // that still drops a few contacts on one body-local edge (the residual +y
          // sink). When the tensor is effectively diagonal (negligible products of
          // inertia), force a strictly-distinct DESCENDING diagonal so eig3 returns
          // an identity inertial frame. The principal-axis re-labeling is dynamically
          // irrelevant for these heavy, controller-pinned compound bins.
          // RE-GATED to the compound opt-in + >=2 colliders: exactly-isotropic
          // single-primitive tensors (sphere/cube) must reach add_body as the
          // explicit diagonal with identity iquat, NOT be perturbed.
          if (compoundOn && nPrims >= 2 &&
              ixy > -1e-9 && ixy < 1e-9 && ixz > -1e-9 && ixz < 1e-9 &&
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
    if (idx >= 0 && s->mNewtonGravityCompensation != nullptr)
      // Must precede finalizeWorld(): gravcomp reaches the mjSpec at build time
      // and cannot be patched into mj_model afterwards. A 0 value is a no-op, so
      // a world that does not declare the field is untouched.
      newton->setBodyGravcomp(idx, s->mNewtonGravityCompensation->value());
    if (idx >= 0) {
      // P3.10i: OmRobot wrappers (URDFRobot expansion produces these)
      // typically have a chassis-envelope bounding box that includes
      // the wheel space -- ground contact via that box short-circuits
      // the wheel rolling motion (chassis sits on the box, wheels
      // don't carry weight). Skip the shape on the wrapper Robot;
      // the descendant wheel Solids' shapes handle ground contact.
      // Mass + inertia (rolled up via P3.10d) stay on the body.
      const bool isRobotWrapper = dynamic_cast<const OmRobot *>(s) != nullptr;
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
      const OmWorldInfo *const wiRobotColliders =
          OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
      const bool wrapperUsesOwnShape =
          isRobotWrapper &&
          (!qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_WRAPPER_USES_OWN_SHAPE") ||
           (wiRobotColliders != nullptr && wiRobotColliders->newtonRobotColliders()));
      // ⚠ A wrapper with NOTHING below it to carry contact must use its own
      // shape, opt-in or not. The skip above exists because on a multi-link
      // robot the descendants are the load-bearing colliders (a Husky's wheels
      // hang below its chassis, and a chassis box would short-circuit them) --
      // but a SINGLE-LINK robot has no descendants, so skipping leaves it with
      // the r=1mm placeholder below and it rests on a pinpoint.
      //
      // Measured 2026-08-02, Newton, a 50 mm cube: imported as a single-link
      // URDFRobot it settled at z=0.0506 against a floor whose top is 0.05 --
      // its FRAME on the floor, the box sunk halfway -- where a physically
      // identical native Solid settled correctly at 0.0749. ODE placed both at
      // 0.0749, so this was Newton-only. An agent hit it, spent the time to
      // measure it, and worked around it by rewriting the task's props as
      // native Solids; its note read "a single-link URDF free body rests with
      // its frame, not its box bottom, on the floor".
      const bool wrapperHasNoDescendantCollider = isRobotWrapper && !hasDescendantCollider(s);
      const bool useBoundingObject = (s->mBoundingObject != nullptr) &&
                                     (!isRobotWrapper || wrapperUsesOwnShape || wrapperHasNoDescendantCollider);
      if (useBoundingObject) {
        // P8.2: shape extraction (Pose-unwrap + primitive/mesh handling)
        // is now shared with the static-body path via this helper.
        shapeDesc = attachNewtonShapeFromBoundingObject(
            newton, idx, dynamic_cast<OmBaseNode *>(s->mBoundingObject->value()),
            newtonSoftKeForMaterial(s->mContactMaterial),
            newtonFrictionForSolid(s->mNewtonFriction),
              newtonFrictionForSolid(s->mNewtonFrictionTorsional),
              newtonFrictionForSolid(s->mNewtonFrictionRolling));
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
      // ⚠ SAY SO when the collider is not the geometry that was authored.
      //
      // Every branch above can quietly hand the solver something other than
      // what the scene declares: a body with no resolvable bounding object
      // becomes a 12 cm sphere, a robot wrapper becomes a 1 mm sphere, a mesh
      // becomes its axis-aligned box, a cylinder becomes a capsule. All four
      // change the physics, all four were computed into `shapeDesc`, and
      // `shapeDesc` was then thrown away -- so the run behaved differently
      // from the file and nothing anywhere said why.
      //
      // Measured 2026-08-02: a single-link URDF body silently got the 1 mm
      // placeholder and rested on a pinpoint 24 mm too low. An agent found it
      // by dropping a reference cube beside it and comparing, which is the
      // expensive way to learn something the engine already knew. Warned once
      // per body, naming the solid and the substitution.
      if (!shapeDesc.isEmpty() && (shapeDesc.contains("fallback") || shapeDesc.contains("placeholder") ||
                                   shapeDesc.contains("was cylinder"))) {
        OmLog::warning(QObject::tr("Solid '%1': the physics collider is NOT the authored geometry -- using %2. The "
                                   "body will contact the world at a different size or place than the scene shows. To collide as authored, declare a 'boundingObject' on this Solid (Box, Sphere, Capsule, Cylinder, Plane or a mesh; a Group of several needs WorldInfo.newtonCompoundColliders TRUE) -- a Robot with none gets the 1 mm placeholder by design. See docs/guide/newton-physics-backend.md.")
                         .arg(s->name())
                         .arg(shapeDesc),
                       false, OmLog::ODE);
      }
      s->mNewtonBodyIndex = idx;
      ++registeredThisFlush;
      // FIX 5 (t=0 setVelocity drop): replay a Supervisor velocity set that
      // arrived before this registration (cached by setLinearVelocity /
      // setAngularVelocity). At this point the runtime world is still
      // pre-finalize, so set_body_vel queues the write and finalize()
      // drains it into body_qd / joint_qd after its closing eval_fk.
      if (s->mPendingNewtonLinVelValid) {
        s->setNewtonBodyVel(s->mPendingNewtonLinVel, false);
        s->mPendingNewtonLinVelValid = false;
      }
      if (s->mPendingNewtonAngVelValid) {
        s->setNewtonBodyVel(s->mPendingNewtonAngVel, true);
        s->mPendingNewtonAngVelValid = false;
      }
      // P5 step-1 perf fix 2026-05-28: the per-body "registered solid"
      // OmLog::info fires during step 1 *after* mConsoleLogsPostponed
      // has flipped back to false (per OmMainWindow::restorePerspective
      // → setConsoleLogsPostponed(false)), so the OmLog skip-receivers
      // postponed-short-circuit in OmLog.cpp doesn't apply. Per-call
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
    }
  }

  // ---- PASS 2: force-TouchSensor un-fold (_scratch/design_weld_touch.md T1,
  // OMNISIM_NEWTON_TOUCH_FORCE). Each collected sensor gets its OWN Newton
  // body (own mass -- rolledUpMass above excluded it from the leader -- plus
  // its own boundingObject shapes, which the fold used to DROP) welded to the
  // merge leader's body with a FIXED joint. newton's MuJoCo conversion keeps
  // a FIXED-joint child as a separate welded mjc body with its own geoms and
  // no joint element, so contacts attribute to the sensor and cfrc_int reads
  // its mount wrench (the 981 N semantics). Runs after the main loop so the
  // leader's body exists whatever the cSolids order; a leader that is not
  // Newton-registered (ODE articulation) leaves the sensor untouched -- it is
  // retried next flush and folds nowhere, exactly the pre-change behaviour.
  for (OmSolid *const ts : pendingUnfoldSensors) {
    const OmSolid *const up = ts->upperSolid();
    const int parentBody = (up != nullptr) ? up->nearestNewtonBodyIndex() : -1;
    if (parentBody < 0)
      continue;  // leader not (yet) on Newton -- retry next flush
    if (newton->ensureWorldOpen() != 0)
      return;
    const OmVector3 t = ts->matrix().translation();
    const OmQuaternion q = OmRotation(ts->rotationMatrix()).toQuaternion();
    double mass = ts->physics() != nullptr ? ts->physics()->mass() : 0.0;
    if (mass <= 0.0)
      mass = rolledUpMass(ts);
    if (mass <= 0.0) {
      // A device with NO Physics node contributes no mass in the first place --
      // rolledUpMass only sums Solids that declare one -- so the leader's mass
      // must not change just because the device now owns a body. Give it a token
      // mass instead: it is fixed-welded to the leader and only needs to be a
      // well-formed body, not a heavy one. That covers a VacuumGripper (a
      // suction cup is part of the tool, not a body in its own right; the 0.25 kg
      // fallback would be a 50% mass change on a 0.5 kg tool) and, since the
      // bumper un-fold above stopped requiring physics, a bumper pad -- which is
      // the common case, and where 0.25 kg on each pad of a small robot would be
      // a physics change smuggled in behind a sensor fix.
      // A DECLARED Physics node whose mass resolves <= 0 (density-derived) keeps
      // the old 0.25 kg fallback: the author did ask for a body there.
      mass = (ts->physics() == nullptr || isUnfoldedVacuumGripper(ts)) ? 1.0e-4 : 0.25;
    }
    // Explicit inertia when the Physics node declares one; zeros otherwise
    // let the runtime preset stand -- a welded child's inertia folds into the
    // parent's composite, so the preset is inconsequential for the mount
    // force (pinned by the parity test).
    double ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0;
    if (const OmPhysics *const phys = ts->physics()) {
      const OmMFVector3 &im = phys->inertiaMatrix();
      if (im.size() >= 1) {
        const OmVector3 &diag = im.item(0);
        ixx = diag.x(); iyy = diag.y(); izz = diag.z();
      }
      if (im.size() >= 2) {
        const OmVector3 &off = im.item(1);
        ixy = off.x(); ixz = off.y(); iyz = off.z();
      }
    }
    const int idx = newton->addBody(mass, t.x(), t.y(), t.z(),
                                    q.x(), q.y(), q.z(), q.w(),
                                    ixx, iyy, izz, ixy, ixz, iyz);
    if (idx < 0)
      continue;
    if (ts->mNewtonGravityCompensation != nullptr)
      newton->setBodyGravcomp(idx, ts->mNewtonGravityCompensation->value());
    if (ts->mBoundingObject != nullptr && ts->mBoundingObject->value() != nullptr)
      attachNewtonShapeFromBoundingObject(
          newton, idx, dynamic_cast<OmBaseNode *>(ts->mBoundingObject->value()),
          newtonSoftKeForMaterial(ts->mContactMaterial),
          newtonFrictionForSolid(ts->mNewtonFriction),
              newtonFrictionForSolid(ts->mNewtonFrictionTorsional),
              newtonFrictionForSolid(ts->mNewtonFrictionRolling));
    if (newton->addJointFixed(parentBody, idx) < 0) {
      OmLog::warning(QObject::tr("TouchSensor '%1': un-fold weld to its parent body failed -- the sensor body is "
                                 "registered but unattached. Its force readback will be wrong; report this.")
                       .arg(ts->name()),
                     false, OmLog::ODE);
    }
    ts->mNewtonBodyIndex = idx;
    ++registeredThisFlush;
    // Same ODE hand-off as the main path: Newton owns this body now.
  }

  // ---- Weld-slot sweep (Connector / VacuumGripper, OMNISIM_NEWTON_WELDS).
  // One INACTIVE MuJoCo weld slot per device whose merge leader owns a Newton
  // body, allocated while the world is still open for build -- eq arrays are
  // compile-time sized, so a device without a slot can never lock natively.
  // Idempotent (each device caches its slot id); the devices themselves are
  // OmSolids, so the census list is the complete roster.
  if (newton->isWorldOpenForBuild()) {
    for (const OmSolid *const cs : cSolids) {
      OmSolid *const dev = const_cast<OmSolid *>(cs);
      if (OmConnector *const c = dynamic_cast<OmConnector *>(dev))
        c->ensureNewtonWeldSlot(newton);
      else if (OmVacuumGripper *const v = dynamic_cast<OmVacuumGripper *>(dev))
        v->ensureNewtonWeldSlot(newton);
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
  // this function runs every tick (OmSimulationWorld::step -> flushPendingNewtonRegistrations).
  // So it re-emitted the identical line at tick rate, and because OmLog::info is
  // relayed to the agent-facing controller.log stream, it drowned GET /sim/events:
  // measured 96.7% of the log stream was this one line, with real controller
  // telemetry silently evicted from the ring buffer (dropped_log climbing without
  // bound). Gating on registeredThisFlush keeps the diagnostic exactly where it is
  // useful -- the tick(s) that build the world, and any LATER tick that registers a
  // newly-spawned Solid (a supervisor-injected robot), which is genuinely worth a
  // line -- and is self-resetting across world loads: a reloaded world constructs
  // fresh OmSolids with mNewtonBodyIndex = -1, so they re-register and re-census.
  // No static/global flag to leak or forget to reset.
  if (registeredThisFlush > 0) {
    int nStatic = 0, nDynamic = 0;
    QStringList staticNames;
    for (const OmSolid *const cs : cSolids) {
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
    if (!skippedStaticNames.isEmpty()) {
      static bool sWarnedSkippedStatics = false;
      if (!sWarnedSkippedStatics) {
        sWarnedSkippedStatics = true;
        OmLog::warning(QObject::tr(
                         "%1 static collider(s) in this world are NOT registered with the Newton backend and "
                         "therefore collide with NOTHING: %2. They stay on the ODE side, which simulates none of "
                         "this world's bodies. Anything that looks supported is resting on the implicit ground "
                         "plane at z=0, so a raised floor, table or wall is intangible and bodies pass through it "
                         "and settle at the wrong height. Set WorldInfo.newtonStatics TRUE (or "
                         "OMNISIM_NEWTON_STATICS=1) to register them.")
                         .arg(skippedStaticNames.size())
                         .arg(skippedStaticNames.join(", ")),
                       false, OmLog::ODE);
      }
    }
    if (nStatic > 0 || nDynamic > 0)
      OmLog::info(QString("[OmNewtonBackend] registered %1 dynamic + %2 static "
                          "Newton bodies (+%4 this pass) (statics: %3)")
                      .arg(nDynamic).arg(nStatic).arg(staticNames.join(", ")).arg(registeredThisFlush));

    // ⚠ THE FLOOR MIGHT NOT BE IN THE SCENE. Newton opens every world with an
    // implicit ground plane at z=0, added before any Solid is known and never
    // appearing as a node. A world that declares no static collision surface
    // therefore still runs -- bodies rest on something the file does not
    // contain, `getFromDef` cannot find, and a scene walk cannot report.
    //
    // Measured 2026-08-03: an agent authored a working pick-and-place with no
    // floor at all. It ran, the block sat on the implicit plane, and the
    // support-surface channel came back unanswerable -- so the tier clause
    // that asks what the object was resting on could not be graded, and the
    // cell was filed against OUR scaffolding rather than against the scene.
    // The world was not self-describing and nothing said so.
    //
    // Warned once, when dynamic bodies exist and nothing static does.
    static bool sWarnedImplicitFloor = false;
    if (!sWarnedImplicitFloor && nDynamic > 0 && nStatic == 0) {
      sWarnedImplicitFloor = true;
      OmLog::warning(QObject::tr(
                       "This world declares NO static collision surface, so its bodies are resting on the implicit "
                       "ground plane the physics backend adds at z=0. That plane is not a node: it cannot be found by "
                       "DEF, it does not appear in the scene tree, and anything asking what holds this scene up will "
                       "get no answer. Add a floor Solid with a boundingObject if the world is meant to describe "
                       "itself."),
                     false, OmLog::ODE);
    }
  }

  // Per-world Newton preference (WorldInfo.newtonSolver). Plumbed here --
  // after registrations have opened the build-phase world but before
  // OmNewtonBackend::finalizeWorld() builds the solver. Since XPBD's removal
  // (2026-08-07) the solver is SolverMuJoCo unconditionally; the preference
  // only selects CPU mj_step vs the batched mujoco_warp GPU path. No-op when
  // nothing registered (world never opened).
  if (newton->isWorldOpenForBuild()) {
    const OmWorldInfo *const wi = OmWorld::instance() ? OmWorld::instance()->worldInfo() : nullptr;
    if (wi != nullptr) {
      newton->setSolverPreference(wi->newtonSolver().toStdString());
      // ⚠ newtonSolver "vbd" REMOVES MuJoCo from the world, and the features
      // that go with it go quietly. The runtime enumerates exactly which of
      // THIS world's fields it is dropping -- but it does so through
      // _newton_log, i.e. into .build_tmp/newton_solver.log, which nobody
      // reads unless they already suspect something. Put a pointer where a
      // person will actually see it.
      //
      // INFO rather than WARNING deliberately: the author typed this value, so
      // the SOLVER choice is not a surprise and does not deserve to fail a
      // --fail-on-warning run. What IS surprising is which of their own fields
      // silently stopped meaning anything, and that is the list this points at.
      if (wi->newtonSolver() == "vbd")
        OmLog::info(QObject::tr(
          "WorldInfo.newtonSolver \"vbd\": ONE newton SolverVBD will own every rigid body, joint AND "
          "particle -- there is no MuJoCo in this world. Raycast-backed devices (DistanceSensor, "
          "Receiver, LightSensor, Radar, Camera recognition occlusion), Connector / VacuumGripper "
          "welds, TouchSensor and the MuJoCo contact readback are ALL unavailable here, rigid contact "
          "is AVBD rather than MuJoCo's friction cone, and newton itself IGNORES joint armature, "
          "friction, effort limits, velocity limits and target mode WITHOUT SAYING SO. The engine "
          "lists the ones this world actually uses in .build_tmp/newton_solver.log (grep "
          "'newtonSolver \"vbd\"'). Use \"mujoco+vbd\" instead if this world needs cloth AND any of "
          "the above."));
      // Per-Solid cloth-proxy visibility (Solid.newtonClothCoupling). Sent
      // here rather than at each addBody site for two reasons: every body
      // index exists by now (the four registration paths -- dynamic, static,
      // kinematic and sensor-unfold -- have all run), and the roster is
      // re-sent WHOLE, which is what a rebuild needs. Body indices are only
      // meaningful inside the build that produced them, so the backend
      // deliberately does not cache and replay them the way it does the solver
      // preference.
      //
      // Costs one pass over cSolids per flush and sends NOTHING for a world
      // that leaves every Solid at 0 -- which is what keeps the runtime's
      // roster empty, and an empty roster is what makes such a world
      // bit-identical to one built before this field existed.
      {
        int nCouple = 0, nExclude = 0;
        QStringList coupledNames;
        for (const OmSolid *const cs : cSolids) {
          if (cs == nullptr || cs->mNewtonClothCoupling == nullptr || cs->mNewtonBodyIndex < 0)
            continue;
          const int mode = cs->mNewtonClothCoupling->value();
          if (mode == 0)
            continue;
          if (newton->setClothCoupling(cs->mNewtonBodyIndex, mode) != 0)
            continue;
          if (mode > 0) {
            ++nCouple;
            if (coupledNames.size() < 12)
              coupledNames.append(QStringLiteral("%1(b%2)").arg(cs->name()).arg(cs->mNewtonBodyIndex));
          } else {
            ++nExclude;
          }
        }
        if (nCouple > 0 || nExclude > 0) {
          // The runtime logs the resolved roster as bare newton body INDICES,
          // because that is all it has. This line is what makes those indices
          // resolvable back to nodes an author can find in the scene tree --
          // without it, "proxy roster narrowed to [7, 8]" names nothing.
          OmLog::info(QObject::tr("Solid.newtonClothCoupling: %1 body(ies) declared VISIBLE to the cloth "
                                  "solver%2, %3 declared hidden. %4")
                        .arg(nCouple)
                        .arg(coupledNames.isEmpty() ? QString() : QStringLiteral(" [%1]").arg(coupledNames.join(", ")))
                        .arg(nExclude)
                        .arg(nCouple > 0
                               ? QObject::tr("Because at least one body is declared VISIBLE, the world is in "
                                             "ALLOWLIST mode: every OTHER rigid body is now invisible to the "
                                             "fabric and the cloth will pass straight through it. Floors and "
                                             "tables are ordinary bodies and must be listed to be seen.")
                               : QObject::tr("No body is declared visible, so the world stays in DENYLIST mode: "
                                             "every body except those %1 remains a cloth proxy.")
                                   .arg(nExclude)));
        }
      }
      // Fold WorldInfo.newtonSubsteps into the runtime too (N3): a contact-heavy
      // world declares its XPBD sub-steps in the .wbt instead of via an env var
      // (OMNISIM_NEWTON_SUBSTEPS still overrides). No-op for the default (1).
      newton->setNewtonSubsteps(wi->newtonSubsteps());
      // WorldInfo.newtonCone / newtonImpratio -> SolverMuJoCo contact knobs
      // (per-world; "" / 0 = MuJoCo stock, env vars still override).
      newton->setContactCone(wi->newtonCone().toStdString(), wi->newtonImpratio());
      // WorldInfo.newtonCondim -> mjModel.geom_condim (per-world; 0 = unset,
      // leaving the condim 3 every OmniSim geom carries today). 4 adds the
      // torsional friction a two-finger pinch needs; without it a pinched part
      // spins about the contact normal at zero cost.
      newton->setContactCondim(wi->newtonCondim());
      // WorldInfo.newtonNoslipIterations -> mjOption.noslip_iterations
      // (per-world; 0 = unset, which is MuJoCo's own stock value, so every
      // existing world is byte-identical). It is the friction-only post-solve
      // pass, and what it removes is the tangential drift that lets a pinch
      // creep out of a grasp while its normal force sits exactly where it was
      // commanded -- measured on ladder0 rung 8. CPU mj_step only; mujoco_warp
      // does not implement it and the runtime declines + warns there.
      // ⚠ The runtime can only DECLINE this on the GPU path (mujoco_warp has no
      // noslip field and its put_model raises on a non-zero one), and it records
      // the decline in the solver string the log + the .newton.json sidecar
      // carry. That is machine-readable but easy to miss, and a knob that reads
      // as set and does nothing is exactly how a measurement gets attributed to
      // the wrong thing -- so say it in the log too, where a person will see it.
      // Scoped to the DECLARED solver: a run steered onto warp by
      // OMNISIM_NEWTON_MJWARP instead is caught by the sidecar marker only.
      if (wi->newtonNoslipIterations() > 0 && wi->newtonSolver() == "mujoco_warp")
        OmLog::warning(QObject::tr("WorldInfo.newtonNoslipIterations %1 is IGNORED on newtonSolver "
                                   "\"mujoco_warp\": mujoco_warp does not implement MuJoCo's noslip "
                                   "pass (its Option type has no such field). Use newtonSolver "
                                   "\"mujoco\" (CPU mj_step) if the behaviour you want depends on it.")
                         .arg(wi->newtonNoslipIterations()),
                       false, OmLog::ODE);
      newton->setNoslipIterations(wi->newtonNoslipIterations());
      // WorldInfo.newtonClothSelfContact -> SolverVBD's
      // particle_enable_self_contact (per-world; -1 = the runtime default,
      // which is ON, env var still overrides). Only "vbd" and "mujoco+vbd"
      // build a SolverVBD, so on any other solver the field is inert -- and a
      // knob that reads as set and does nothing is how a measurement gets
      // attributed to the wrong thing, so say so rather than let it pass. The
      // reverse mistake is the expensive one: a deformable-GRASP world that
      // leaves this unset gets the draping default and slips 24x in silence.
      if (wi->newtonClothSelfContact() >= 0 && wi->newtonSolver() != "vbd" && wi->newtonSolver() != "mujoco+vbd")
        OmLog::warning(QObject::tr("WorldInfo.newtonClothSelfContact %1 is IGNORED on newtonSolver %2: "
                                   "only \"vbd\" and \"mujoco+vbd\" build a newton SolverVBD, and particle "
                                   "self-contact is one of its parameters. A world with no particles has "
                                   "nothing to apply it to.")
                         .arg(wi->newtonClothSelfContact())
                         .arg(wi->newtonSolver().isEmpty() ? QString("\"\" (auto)")
                                                           : QString("\"%1\"").arg(wi->newtonSolver())),
                       false, OmLog::ODE);
      newton->setClothSelfContact(wi->newtonClothSelfContact());
      // WorldInfo.newtonNjmax / newtonNconmax -> SolverMuJoCo constraint-row +
      // contact buffer caps (per-world; 0 = the runtime's built-in 256, env
      // vars still override). A fleet world (10 Huskies measure 320 rows) can now
      // declare its own budget instead of needing launch env vars, which a
      // .wbt cannot carry.
      newton->setConstraintBuffers(wi->newtonNjmax(), wi->newtonNconmax());
      // WorldInfo.newtonGroundMu / newtonContactKe / newtonContactKd /
      // newtonIterations / newtonLsIterations -> the contact model and the
      // MuJoCo solver's iteration counts (per-world; 0 = the runtime default,
      // env vars still override). Until 2026-08-02 these five were reachable
      // ONLY from the process environment, so a world file did not describe
      // its own physics: a tuned friction grasp loaded by anyone else got
      // default friction and a soft contact, and did not hold.
      bool cpBridged = false;
      const double resolvedMu = resolvedNewtonGroundMu(wi, &cpBridged);
      newton->setContactSolverParams(resolvedMu, wi->newtonContactKe(), wi->newtonContactKd(),
                                     wi->newtonIterations(), wi->newtonLsIterations());
      // ⚠ WorldInfo.contactProperties is an ODE-path node and is INERT here.
      // Warn once per world when a scene declares a friction it is not going
      // to get: the field looks like the answer, the engine ignores it, and
      // the run then behaves as though friction were 1.0. Measured twice --
      // once in our own findings, and once by an agent that rediscovered it
      // from scratch while trying to make a two-finger grasp hold, having
      // reasonably assumed the declared coulombFriction was in effect.
      //
      // It WARNS rather than honouring the value on purpose. 322 worlds in
      // this tree declare contactProperties, and most were tuned under Newton
      // with the effective mu of 1.0; silently adopting their declared
      // coulombFriction would change the physics of every one of them.
      static bool sContactPropsWarned = false;
      if (!sContactPropsWarned && !cpBridged && wi->contactPropertiesCount() > 0 && wi->newtonGroundMu() < 0.0 &&
          qEnvironmentVariableIsEmpty("OMNISIM_NEWTON_GROUND_MU")) {
        double declared = -1.0;
        for (int i = 0; i < wi->contactPropertiesCount(); ++i) {
          const OmContactProperties *const cp = wi->contactProperties(i);
          if (cp != nullptr && cp->coulombFrictionSize() > 0) {
            declared = cp->coulombFriction(0);
            break;
          }
        }
        if (declared >= 0.0 && qAbs(declared - 1.0) > 1e-9) {
          sContactPropsWarned = true;
          // ⚠ THIS MESSAGE USED TO END "or pin physicsBackend \"ode\" on the
          // Solids you are tuning through contactProperties". Since bdc02139
          // deleted src/ode that advice produces a Solid registered with NO
          // solver -- no gravity, no contact -- so the fix for "my friction is
          // ignored" was a world with no physics. Never restore it.
          OmLog::warning(QObject::tr("WorldInfo.contactProperties declares coulombFriction %1, but this world runs on "
                                     "the Newton backend, which does NOT read that field: the effective friction is "
                                     "1.0. Set WorldInfo.newtonGroundMu %1 to get the friction you asked for. (Do NOT "
                                     "reach for physicsBackend \"ode\": ODE was deleted, and a Solid pinned to it is "
                                     "registered with no solver at all -- no gravity and no contact.)")
                           .arg(declared),
                         false, OmLog::ODE);
        }
      }
      // Internal parity plan, item W1.5: RESTITUTION IS A HARD LIMIT, NOT A GAP.
      // ContactProperties.bounce / bounceVelocity are ODE-path fields whose
      // accessors have zero callers, and unlike coulombFriction there is no
      // newton* field to migrate them to and no prospect of one: MuJoCo has no
      // coefficient of restitution. Our contact defaults map to the stock,
      // critically-damped solref (0.02, 1.0), i.e. e ~= 0, and MuJoCo's own
      // spring-damper contact only behaves at the two ends -- one intermediate
      // configuration rebounded 661 m from a 1 m drop. So say plainly that the
      // field is never read, and do not pretend a knob exists.
      //
      // Separate one-shot from the friction warning above, with its own
      // precondition: it fires only when the author actually WROTE a bounce
      // (see OmContactProperties::bounceIsAuthored), never on the .wrl default
      // of 0.5, which would hit every one of the 322 worlds carrying a
      // ContactProperties node and get muted rather than read.
      static bool sBounceWarned = false;
      if (!sBounceWarned && wi->contactPropertiesCount() > 0) {
        for (int i = 0; i < wi->contactPropertiesCount(); ++i) {
          const OmContactProperties *const cp = wi->contactProperties(i);
          if (cp == nullptr || (!cp->bounceIsAuthored() && !cp->bounceVelocityIsAuthored()))
            continue;
          sBounceWarned = true;
          OmLog::warning(QObject::tr("ContactProperties declares bounce %1 / bounceVelocity %2, and this world runs on "
                                     "the Newton backend, which NEVER READS either field. This is a hard limit, not a "
                                     "missing feature: MuJoCo has no coefficient of restitution, so there is no "
                                     "newton* field to migrate these to. Contacts here are critically damped "
                                     "(solref 0.02 1.0), i.e. effectively inelastic. If a scene needs a bounce, model "
                                     "it in the controller.")
                           .arg(cp->bounce())
                           .arg(cp->bounceVelocity()),
                         false, OmLog::ODE);
          break;
        }
      }
      // WorldInfo.gravity -> Newton (was: library default -9.81 always).
      const OmVector3 &g = wi->gravityVector();
      newton->setWorldGravity(g.x(), g.y(), g.z());
    }
  }

  // A running world whose full pass registered nothing is DISPOSITIONED: mark
  // this generation clean so subsequent ticks skip the whole-scene walk until
  // something bumps the generation. (Registering anything leaves it dirty, so
  // the build-up ticks always re-run.)
  if (newton->isWorldRunning() && registeredThisFlush == 0)
    sLastCleanGeneration = cNewtonFlushGeneration;
}

bool OmSolid::isSleeping() const {
  // Newton has no body auto-disable, so nothing ever sleeps. (This used to ask
  // ODE's dBodyIsEnabled; with ODE gone the answer was constant-false anyway.)
  return false;
}

// Direct body wrenches. ODE removed: OmSolid::addForceAtPosition / addTorque are
// now UNIMPLEMENTED -- the mouse force/torque drag (OmDragSolidEvent) and the
// granular-group coupling (OmGranularGroup) that call them apply nothing.
//
// CORRECTION 2026-08-22. This comment used to continue "The supervisor's own
// add_force/add_torque paths are unaffected: they route through
// OmSolid::applyExternalForceNewton" -- and that was FALSE for two of the three
// verbs from bdc02139 (2026-08-08) until 2026-08-22. Only C_SUPERVISOR_NODE_ADD_TORQUE
// reached applyExternalForceNewton. C_SUPERVISOR_NODE_ADD_FORCE and
// C_SUPERVISOR_NODE_ADD_FORCE_WITH_OFFSET still tested the LEGACY ODE merger body
// (OmSolidMerger::mBody, set NULL in its constructor and assigned nowhere once ODE
// was deleted), found it null, took their "can't be used with a kinematic Solid"
// branch and dropped the wrench -- on bodies Newton had registered as DYNAMIC.
// OmPropeller::prePhysicsStep had the identical dead gate. All three now gate on
// the Newton body handle, which is what add_torque had been doing all along; the
// asymmetry is exactly why the sentence read as true.
//
// Measured after the repair (OmniBench lane 4, machine 9722d23d12a3, CPU mj_step):
// phenomenon.supervisor_external_force broken -> works, 10 N on a 2 kg body giving
// 5.000076 m/s^2 against an analytic 5.0 and a 5 cm offset arm sweeping 9.388 rad.
// The revert hatch OMNISIM_NEWTON_NO_EXT_FORCE (checked inside
// applyExternalForceNewton) was INERT while these gates were dead and is honest
// again: with it set, the three propeller.omniworld helicopters do not move for
// 15,360 steps.
//
// Still true, and still deliberate: applyExternalForceNewton is NOT wired up in the
// two stubs below (see the ODE-sweep report -- deciding whether the drag verbs
// should adopt it is a behaviour change, not a deletion).
void OmSolid::addForceAtPosition(const OmVector3 &force, const OmVector3 &position) {
}

void OmSolid::addTorque(const OmVector3 &torque) {
}

// W3.1 (newton-ode-replacement-plan.md): route an external force-at-world-position + torque to this Solid's
// NEWTON body if it has one. Newton's body_f is a WORLD-frame wrench about the body's reference point, so a
// force F at world point P becomes (F, (P - bodyOrigin) x F + torque). Returns true if it routed to Newton
// (the caller then skips the ODE path, whose body is disabled for Newton-backed Solids); false -> not
// Newton-backed, so the caller has nowhere to route it (there is no ODE) and the force is DROPPED -- the
// supervisor call sites warn in that case. The body origin from the Newton pose readback approximates the
// COM as the torque reference -- exact when the Solid's frame is at its COM.
bool OmSolid::applyExternalForceNewton(const OmVector3 &force, const OmVector3 &worldPos,
                                       const OmVector3 &torque) const {
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
  OmPhysicsBackend *const backend = OmPhysicsBackendRegistry::newtonBackend();
  if (backend == nullptr || !backend->isAvailable())
    return false;
  OmNewtonBackend *const newton = static_cast<OmNewtonBackend *>(backend);
  OmVector3 ref = matrix().translation();  // fallback: the Solid's world origin
  double xform[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  if (newton->getBodyXform(mNewtonBodyIndex, xform) == 0)
    ref = OmVector3(xform[0], xform[1], xform[2]);
  const OmVector3 d = worldPos - ref;
  const OmVector3 tau(d.y() * force.z() - d.z() * force.y(),
                      d.z() * force.x() - d.x() * force.z(),
                      d.x() * force.y() - d.y() * force.x());
  return newton->addBodyForce(mNewtonBodyIndex, force.x(), force.y(), force.z(),
                              tau.x() + torque.x(), tau.y() + torque.y(), tau.z() + torque.z()) == 0;
}

// W3.2: route a mid-step velocity set to this Solid's NEWTON body if it has one (angular=false -> linear
// half, true -> angular half of body_qd). Returns true when routed to Newton; false -> not Newton-backed,
// caller caches the value for replay at Newton registration (FIX 5: a t=0 setVelocity used to be dropped
// outright, because at t=0 the Newton body does not exist yet and there is no other engine to take it).
bool OmSolid::setNewtonBodyVel(const double v[3], bool angular) const {
  if (mNewtonBodyIndex < 0)
    return false;
  OmPhysicsBackend *const backend = OmPhysicsBackendRegistry::newtonBackend();
  if (backend == nullptr || !backend->isAvailable())
    return false;
  // A LINK INSIDE AN ARTICULATION CANNOT HAVE ITS VELOCITY SET THIS WAY, AND
  // MUST NOT PRETEND OTHERWISE. Under MuJoCo a body's velocity is DERIVED from
  // the joint velocities, so a write into body_qd is overwritten by the next
  // state update and the call does nothing at all.
  //
  // Measured against ODE on the same world (free body + hinge child, gravity
  // off, setVelocity then one step):
  //
  //   free body       ODE exact          Newton exact
  //   articulated     ODE (0.150, -0.173, 0.419, ...)   Newton (0, 0, 0, ...)
  //
  // ODE's answer is the requested velocity projected onto the constraint --
  // the write took effect and the joint corrected it. Newton's is a silent
  // no-op, which reads to a caller exactly like "the body really is at rest".
  // Doing nothing is defensible; doing it silently is not.
  if (jointParent() != nullptr) {
    static bool sWarnedArticulatedSetVel = false;
    if (!sWarnedArticulatedSetVel) {
      sWarnedArticulatedSetVel = true;
      OmLog::warning(QObject::tr(
                       "setVelocity() on '%1' has NO EFFECT: it is a link inside an articulation, and this world's "
                       "physics backend derives link velocities from the joint velocities rather than storing them "
                       "per body. The call is ignored, not applied and corrected. Drive the joint (target velocity "
                       "or force) instead, or set the velocity of the articulation's free base.")
                       .arg(name()),
                     false, OmLog::ODE);
    }
    return false;
  }
  return static_cast<OmNewtonBackend *>(backend)->setBodyVel(mNewtonBodyIndex, v[0], v[1], v[2],
                                                             angular ? 1 : 0) == 0;
}

// Selection management
void OmSolid::propagateSelection(bool selected) {
  select(selected);
  OmMatter::propagateSelection(selected);

  foreach (OmBasicJoint *const j, mJointChildren) {
    if (j->solidReference())
      continue;
    OmSolid *const solid = j->solidEndPoint();
    if (solid)
      solid->propagateSelection(selected);
  }

  OmBaseNode *const bo = boundingObject();
  if (bo)
    bo->propagateSelection(selected);
}

void OmSolid::setMatrixNeedUpdate() {
  OmNode *bo = boundingObject();
  OmGroup *g = dynamic_cast<OmGroup *>(bo);
  if (g)
    g->setMatrixNeedUpdate();

  OmPose::setMatrixNeedUpdate();
}

void OmSolid::reset(const QString &id) {
  OmMatter::reset(id);

  OmNode *const p = mPhysics->value();
  if (p)
    p->reset(id);

  if (mJointParents.size() == 0) {
    setTranslation(translationFromFile(id));
    setRotation(rotationFromFile(id));
  }
  resetSingleSolidPhysics();
  resetContactPointsAndSupportPolygon();
  resetContactPoints();

  // remove contact joints

  int counter = 0;
  restoreHiddenKinematicParameters(mHiddenKinematicParametersMap, counter);

  if (handleJerkIfNeeded())
    mMovedChildren.clear();
  else if (!mMovedChildren.isEmpty())
    childrenJerk();

}

void OmSolid::save(const QString &id) {
  OmMatter::save(id);
  if (isTopSolid())
    saveHiddenFieldValues();

  OmNode *const p = mPhysics->value();
  if (p)
    p->save(id);
}

// Recursive reset methods
// It resets the positions of all ODE dGeoms, static or not, based on the current translation and rotation fields
// It also resets the velocities of all dBodies to 0.0
void OmSolid::jerk(bool resetVelocities, bool rootJerk) {
  if (isSolidMerger())
    mSolidMerger->setGeomAndBodyPositions(resetVelocities, mJointParents.size() == 0 && !isTopSolid());

  foreach (OmSolid *const solid, mSolidChildren)
    solid->jerk(resetVelocities, false);

  foreach (OmBasicJoint *const j, mJointChildren)
    j->updateOdeWorldCoordinates();

  if (isDynamic() && mJointParents.size() > 0 && rootJerk)
    emit positionChangedArtificially();
}

void OmSolid::notifyChildJerk(OmPose *childNode) {
  const OmNode *node = childNode->parentNode();
  while (node != this && node != NULL) {
    if (mMovedChildren.contains(dynamic_cast<const OmPose *>(node)))
      return;
    node = node->parentNode();
  }

  mMovedChildren.append(childNode);
}

void OmSolid::childrenJerk() {
  foreach (const OmPose *childNode, mMovedChildren) {
    QVector<OmSolid *> solidChildrenList;
    QVector<OmBasicJoint *> jointChildrenList;
    QVector<OmPropeller *> propellerChildrenList;
    collectSolidChildren(childNode, false, solidChildrenList, jointChildrenList, propellerChildrenList);

    foreach (OmSolid *const solid, solidChildrenList)
      solid->jerk(false, false);

    foreach (OmBasicJoint *const j, jointChildrenList)
      j->updateOdeWorldCoordinates();
  }
  mMovedChildren.clear();
}

void OmSolid::awake() {
  // Newton has no body sleep, so a merged body needs no waking; the world-level
  // awake() is kept for the kinematic / not-yet-merged case it always covered.
  if (!mSolidMerger || mSolidMerger->isBodyArtificiallyDisabled())
    OmWorld::instance()->awake();
}

void OmSolid::awakeSolids(OmGroup *group) {
  // ODE removed: waking the scene was entirely dBodyEnable on the ODE bodies,
  // and Newton has no auto-disable to undo. Kept as a no-op because
  // OmWorld::awake() (and its once-per-step gate) walks the scene through it.
}

void OmSolid::resetPhysics(bool recursive) {
  resetSingleSolidPhysics();

  // Recurses through all first level solid descendants
  if (recursive) {
    foreach (OmSolid *const solid, mSolidChildren)
      solid->resetPhysics();
  }
}

void OmSolid::resetSingleSolidPhysics() {
  // check for joints and disable all motors
  const int size = mJointChildren.size();
  for (int i = 0; i < size; ++i) {
    OmJoint *const j = dynamic_cast<OmJoint *>(mJointChildren[i]);
    if (j)
      j->resetPhysics();
  }

  mLinearVelocity->setValue(0.0, 0.0, 0.0);
  mAngularVelocity->setValue(0.0, 0.0, 0.0);

  // ODE removed: zeroing the SOLVER-side velocity/force/torque of the merged
  // body is now UNIMPLEMENTED -- only the display fields above are cleared, so
  // a Newton body keeps whatever body_qd / body_f it had. (Deliberately not
  // wired to Newton here: that is a behaviour change, not a deletion.)
}

void OmSolid::pausePhysics(bool resumeAutomatically) {
  if (resumeAutomatically)
    mResetPhysicsInStep = true;

  if (mSolidMerger)
    mSolidMerger->setBodyArtificiallyDisabled(true);

  foreach (OmSolid *const solid, mSolidChildren)
    solid->pausePhysics();
}

void OmSolid::resumePhysics() {
  resetSingleSolidPhysics();
  if (mSolidMerger)
    mSolidMerger->setBodyArtificiallyDisabled(false);

  foreach (OmSolid *const solid, mSolidChildren)
    solid->resumePhysics();
}

///////////////////////////////
// Contact Points Management //
///////////////////////////////

const QVector<OmVector3> &OmSolid::computedContactPoints(bool includeDescendants) {
  extractContactPoints();
  connect(OmSimulationState::instance(), &OmSimulationState::physicsStepStarted, this, &OmSolid::resetContactPoints,
          Qt::UniqueConnection);
  return includeDescendants ? mGlobalListOfContactPoints : mListOfContactPoints;
}

const QVector<const OmSolid *> &OmSolid::computedSolidPerContactPoints() {
  extractContactPoints();
  connect(OmSimulationState::instance(), &OmSimulationState::physicsStepStarted, this, &OmSolid::resetContactPoints,
          Qt::UniqueConnection);
  return mSolidPerContactPoints;
}

const QVector<double> &OmSolid::computedContactPointDepths(bool includeDescendants) {
  extractContactPoints();
  connect(OmSimulationState::instance(), &OmSimulationState::physicsStepStarted, this, &OmSolid::resetContactPoints,
          Qt::UniqueConnection);
  return includeDescendants ? mGlobalListOfContactPointDepths : mListOfContactPointDepths;
}

void OmSolid::extractContactPoints() {
  if (mHasExtractedContactPoints)
    return;

  const OmWorld *const world = OmWorld::instance();

  // W4.2c: for a Newton-backed Solid with native contacts enabled, the native pass below is the SOLE source
  // -- skip the ODE-bridge accumulation entirely. Multi-body verification showed a Robot's disabled ODE
  // proxies STILL collide, so ADDING native on top double-counted (rover: ODE=8 + native=5 -> 13). ODE Solids
  // and the default path are untouched (useNative is false unless this Solid is Newton-backed).
  //
  // DEFAULT ON as of 2026-08-07. Two earlier comments fought over this line:
  // one claimed "DEFAULT ON as of 2026-08-03", the other recorded that the
  // 2026-08-03 flip was tried and REVERTED for lack of end-to-end evidence --
  // and the code was the second (presence-gated opt-in, so `=0` turned it ON,
  // the inverted-hatch bug class F2 catalogued). The evidence the revert was
  // waiting for now exists: on a ball demonstrably at rest on a floor,
  // getContactPoints reads 0 with the native path off and >=1 with it on
  // (commit 4f09f9b6 named this "the next measurement";
  // tests/test_newton_contacts_visible_by_default.py pins it, with a
  // geometric rest-height assertion so a zero cannot be excused as "really
  // mid-air"). The cost of the old default was total contact blindness on
  // every Newton world -- 1008 contacts on ODE vs 0 on Newton for one scene;
  // an agent proving a grasp found the query empty while the gripper was
  // visibly holding the block and built a geometric work-around instead.
  //
  // OMNISIM_NEWTON_NATIVE_CONTACTS=0 is the exact-revert hatch (value-parsed:
  // "0"/"false"/"off" = off, anything else = on, unset = on).
  bool nativeContactsOn = true;
  {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_NEWTON_NATIVE_CONTACTS")).trimmed().toLower();
    if (!v.isEmpty())
      nativeContactsOn = !(v == "0" || v == "false" || v == "off" || v == "no");
  }
  const bool useNative = (mNewtonBodyIndex >= 0 && nativeContactsOn
                          && OmPhysicsBackendRegistry::newtonBackend() != nullptr
                          && OmPhysicsBackendRegistry::newtonBackend()->isAvailable());

  // ...and when the revert hatch turned it OFF for a Newton-backed body, SAY
  // that the answer is "cannot know", not "none" -- an empty set from a blind
  // query is indistinguishable from "nothing is touching", and that ambiguity
  // once cost an agent a working grasp proof. Reachable only via
  // OMNISIM_NEWTON_NATIVE_CONTACTS=0 now that the default is on.
  if (mNewtonBodyIndex >= 0 && !useNative) {
    static bool sWarnedNewtonContactBlind = false;
    if (!sWarnedNewtonContactBlind) {
      sWarnedNewtonContactBlind = true;
      OmLog::warning(QObject::tr("Contact queries on Newton-backed Solids return an EMPTY set: the native contact "
                                 "readback is off, so getContactPoints cannot see this backend's contacts and its "
                                 "answer means 'unknown', not 'nothing is touching'. Unset "
                                 "OMNISIM_NEWTON_NATIVE_CONTACTS (or set it to 1) to read them."),
                     false, OmLog::ODE);
    }
  }

  // W4.2c (ON by default since 2026-08-07; OMNISIM_NEWTON_NATIVE_CONTACTS=0 reverts): the native Newton
  // contacts for a Newton-backed Solid -- the ODE bridge can't supply them (its disabled proxy body doesn't
  // collide, so the loop above appended nothing for this body; measured ODE_pts=0 vs native_pts=8). ODE
  // worlds stay byte-identical (useNative requires a Newton body). Mirrors the ODE two-list logic exactly
  // with native data; dedups the floor's dual static(-1)/body registration by point proximity. This is what
  // lets a pure-Newton world feed the contact-points supervisor API (damage tracker) without the ODE
  // collision pass (W4.3 then drops it). ⚠ findSolids() per queried Solid is O(N^2) on a fleet world when
  // something polls contacts every step. The per-step snapshot below keeps the
  // lazy behaviour while paying that scene walk once.
  if (useNative) {
    OmPhysicsBackend *const rawN = OmPhysicsBackendRegistry::newtonBackend();
    if (rawN != nullptr && rawN->isAvailable()) {
      // One native-contact snapshot is shared by EVERY Solid queried during
      // this physics step. The previous path rebuilt findSolids(), the
      // body-index map, and a copy of the complete Newton contact vector once
      // per queried Solid. A supervisor deeply tracking R robots therefore
      // paid O(R * world-solids) scene walks per step before doing any useful
      // filtering. Fleet observability must scale with the world, not with
      // world x query count.
      //
      // Time alone is not a safe key: a hot-reloaded world can return to the
      // same simulation time. World identity and the Solid registration
      // generation close that hole, while a backend pointer change covers a
      // solver reinitialisation inside the same world.
      struct NativeContactSnapshot {
        const OmWorld *world = nullptr;
        const OmPhysicsBackend *backend = nullptr;
        double time = -INFINITY;
        int solidGeneration = -1;
        std::vector<OmNewtonContact> contacts;
        QHash<int, OmSolid *> bodyToSolid;
        QHash<const OmSolid *, QVector<int>> contactsBySolid;
        QHash<const OmSolid *, QVector<int>> contactsByTopSolid;
      };
      static NativeContactSnapshot snapshot;
      const double contactTime = OmSimulationState::instance()->time();
      if (snapshot.world != world || snapshot.backend != rawN || snapshot.time != contactTime ||
          snapshot.solidGeneration != cNewtonFlushGeneration) {
        snapshot.world = world;
        snapshot.backend = rawN;
        snapshot.time = contactTime;
        snapshot.solidGeneration = cNewtonFlushGeneration;
        snapshot.contacts.clear();
        static_cast<OmNewtonBackend *>(rawN)->getContacts(snapshot.contacts);
        snapshot.bodyToSolid.clear();
        snapshot.contactsBySolid.clear();
        snapshot.contactsByTopSolid.clear();
        const QList<OmSolid *> allSolids = world->findSolids();
        snapshot.bodyToSolid.reserve(allSolids.size());
        for (int si = 0; si < allSolids.size(); ++si) {
          OmSolid *const candidate = allSolids.at(si);
          if (candidate->mNewtonBodyIndex >= 0)
            snapshot.bodyToSolid.insert(candidate->mNewtonBodyIndex, candidate);
        }
        // Build both lookup shapes once. Shallow queries need contacts on this
        // exact body; a deep Robot query needs contacts on any articulated body
        // whose topSolid() is that Robot. A contact internal to one Robot is
        // inserted only once in the top-level index.
        for (int ci = 0; ci < static_cast<int>(snapshot.contacts.size()); ++ci) {
          const OmNewtonContact &contact = snapshot.contacts[ci];
          OmSolid *const solidA = contact.bodyA >= 0 ? snapshot.bodyToSolid.value(contact.bodyA, nullptr) : nullptr;
          OmSolid *const solidB = contact.bodyB >= 0 ? snapshot.bodyToSolid.value(contact.bodyB, nullptr) : nullptr;
          if (solidA)
            snapshot.contactsBySolid[solidA].append(ci);
          if (solidB && solidB != solidA)
            snapshot.contactsBySolid[solidB].append(ci);
          const OmSolid *const topA = solidA ? solidA->topSolid() : nullptr;
          const OmSolid *const topB = solidB ? solidB->topSolid() : nullptr;
          if (topA)
            snapshot.contactsByTopSolid[topA].append(ci);
          if (topB && topB != topA)
            snapshot.contactsByTopSolid[topB].append(ci);
        }
      }
      const std::vector<OmNewtonContact> &nc = snapshot.contacts;
      const QHash<int, OmSolid *> &bodyToSolid = snapshot.bodyToSolid;
      const OmVector3 up = world->worldInfo()->upVector();
      const QHash<const OmSolid *, QVector<int>> &contactIndex =
        topSolid() == this ? snapshot.contactsByTopSolid : snapshot.contactsBySolid;
      const auto indexedContacts = contactIndex.constFind(this);
      if (indexedContacts != contactIndex.cend()) {
        for (const int ci : indexedContacts.value()) {
          const OmNewtonContact &c = nc[ci];
          OmSolid *const sA = (c.bodyA >= 0) ? bodyToSolid.value(c.bodyA, nullptr) : nullptr;  // -1 = static world
          OmSolid *const sB = (c.bodyB >= 0) ? bodyToSolid.value(c.bodyB, nullptr) : nullptr;
          const OmVector3 v(c.point[0], c.point[1], c.point[2]);
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
  }

  mHasExtractedContactPoints = true;
}

// Computes the support polygon of the robot if needed
const OmPolygon &OmSolid::supportPolygon() {
  const OmWorldInfo *const worldInfo = OmWorld::instance()->worldInfo();
  if (!mSupportPolygonNeedsUpdate)
    return mSupportPolygon;

  extractContactPoints();
  const int numberOfContactPoints = mGlobalListOfContactPoints.size();
  const OmVector3 &eastVector = worldInfo->eastVector();
  const OmVector3 &northVector = worldInfo->northVector();
  // Rules out 4 trivial cases
  if (numberOfContactPoints <= 3) {
    assert(mSupportPolygon.size() >= numberOfContactPoints);
    for (int i = 0; i < numberOfContactPoints; ++i) {
      const OmVector3 &v = mGlobalListOfContactPoints.at(i);
      mSupportPolygon[i].setXy(v.dot(northVector), v.dot(eastVector));
    }
    mSupportPolygon.setActualSize(numberOfContactPoints);
    return mSupportPolygon;
  }

  // From now on, the robot has at least 4 contact points
  std::vector<OmVector2> listOfProjectedContactPoints(numberOfContactPoints);
  // Projects contact points onto a plane orthogonal to the down direction
  for (int i = 0; i < numberOfContactPoints; ++i) {
    const OmVector3 &v = mGlobalListOfContactPoints.at(i);
    listOfProjectedContactPoints[i].setXy(v.dot(northVector), v.dot(eastVector));
  }

  // Gets the indices of points in the convex hull of the projected contact points
  std::vector<int> listOfIndices(numberOfContactPoints);
  const int supportPolygonSize = OmMathsUtilities::twoStepsConvexHull(listOfProjectedContactPoints, listOfIndices);

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

void OmSolid::deleteSupportPolygonRepresentation() {
  mSupportPolygonRepresentation = false;
}

// D1.4: the WREN support-polygon visual died with WREN (it was already unreachable on the
// shipped wgpu default). The menu state + the wb_supervisor_node_get_static_balance data
// path (supportPolygon()/extractContactPoints()/staticBalance(), C5 frozen ABI) survive.
bool OmSolid::showSupportPolygonRepresentation(bool enabled) {
  if (mIsKinematic || !isTopLevel()) {
    if (enabled)
      info(tr("A top Solid with a non-NULL Physics node has to be chosen rather."));
    return false;
  }
  mSupportPolygonRepresentationIsEnabled = enabled;
  mSupportPolygonRepresentation = enabled;

  if (enabled) {
    if (mSupportPolygon.size() < 4)
      // minimum expected size to rule out trivial cases
      mSupportPolygon.resize(4);
    connect(OmSimulationState::instance(), &OmSimulationState::physicsStepStarted, this,
            &OmSolid::resetContactPointsAndSupportPolygon, Qt::UniqueConnection);
  } else {
    disconnect(OmSimulationState::instance(), &OmSimulationState::physicsStepStarted, this,
               &OmSolid::resetContactPointsAndSupportPolygon);
  }

  return true;
}

// Shows or hides the graphical 'global' center of mass of the solid according to menu actions
bool OmSolid::showGlobalCenterOfMassRepresentation(bool enabled) {
  if (mIsKinematic) {
    if (enabled)
      info(tr("A Solid with a non-NULL Physics node must be chosen."));
    return false;
  }
  mGlobalCenterOfMassRepresentationIsEnabled = enabled;

  // D1.4: the WREN global-COM cross died with WREN; the flag survives for the menu +
  // the wgpu COM overlay collectors.
  return true;
}

// Tracks the menu toggle only: Fluid immersion went with the ODE backend, so no solid is
// ever immersed and there is no marker to draw. The flag is still read back by the menu
// check state (OmView3D) and persisted per node in the perspective file (OmWorld).
bool OmSolid::showCenterOfBuoyancyRepresentation(bool enabled) {
  if (mIsKinematic) {
    if (enabled)
      info(tr("A Solid with a non-NULL Physics node must be chosen."));
    return false;
  }
  mCenterOfBuoyancyRepresentationIsEnabled = enabled;

  return true;
}

void OmSolid::refreshPhysicsRepresentation() {
  if (mSupportPolygonRepresentationIsEnabled)
    refreshSupportPolygonRepresentation();
  else if (mGlobalCenterOfMassRepresentationIsEnabled)
    refreshGlobalCenterOfMassRepresentation();

  // propagate change to ancestors
  emit physicsPropertiesChanged();
}

// D1.4: the per-step WREN redraws died with WREN. The support-polygon DATA still refreshes
// through supportPolygon() on demand (C5).
void OmSolid::refreshSupportPolygonRepresentation() {
}

void OmSolid::refreshGlobalCenterOfMassRepresentation() {
}

unsigned char OmSolid::staticBalance() {
  const OmVector3 &c = computedGlobalCenterOfMass();
  const OmPolygon &p = supportPolygon();
  const OmWorldInfo *const wi = OmWorld::instance()->worldInfo();
  const double globalComX = c.dot(wi->northVector());
  const double globalComZ = c.dot(wi->eastVector());
  const bool stable = p.contains(globalComX, globalComZ);
  return stable;
}

void OmSolid::resetContactPointsAndSupportPolygon() {
  mGlobalListOfContactPoints.resize(0);
  mGlobalListOfContactPointDepths.resize(0);
  mSolidPerContactPoints.resize(0);
  mY = numeric_limits<double>::max();
  mSupportPolygonNeedsUpdate = true;
  mHasExtractedContactPoints = false;
}

void OmSolid::resetContactPoints() {
  mListOfContactPoints.resize(0);
  mGlobalListOfContactPoints.resize(0);
  mListOfContactPointDepths.resize(0);
  mGlobalListOfContactPointDepths.resize(0);
  mSolidPerContactPoints.resize(0);
  mHasExtractedContactPoints = false;
  disconnect(OmSimulationState::instance(), &OmSimulationState::physicsStepStarted, this, &OmSolid::resetContactPoints);
}

void OmSolid::onSimulationModeChanged() {
  if (OmSimulationState::instance()->isFast() || !OmSimulationState::instance()->isRendering()) {
    if (mSupportPolygonRepresentation && !mSupportPolygonRepresentationIsEnabled)
      deleteSupportPolygonRepresentation();
  }
}

void OmSolid::updateGraphicalGlobalCenterOfMass() {
  // D1.4: the WREN global-COM cross is gone; globalCenterOfMass() itself stays live.
}

void OmSolid::resetPhysicsIfRequired(bool changedFromSupervisor) {
  if (!changedFromSupervisor) {
    // For now, only the position modifications done by the user should reset the physics.
    resetPhysics();
  }

  OmViewpoint *viewpoint = OmWorld::instance()->viewpoint();
  if (viewpoint->followedSolid() == this)
    viewpoint->updateFollowSolidState();
}

// Collision and sleep flags management

void OmSolid::propagateBoundingObjectMaterialUpdate(bool onSelection) {
  // Recurses through all first level solid descendants
  foreach (OmSolid *const solid, mSolidChildren)
    solid->propagateBoundingObjectMaterialUpdate(onSelection);

  OmBaseNode *const bo = boundingObject();
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

void OmSolid::updateSleepFlag() {
  OmMatter::updateSleepFlag();
}

void OmSolid::displayWarning() {
}

/////////////////////////////////////////////
//  Collecting names of Solid descendants  //
/////////////////////////////////////////////

void OmSolid::collectSolidDescendantNames(QStringList &items, const OmSolid *const solidException) const {
  if (this != solidException)
    items << name();

  // Recurses through all first level solid descendants
  foreach (const OmSolid *const solid, mSolidChildren)
    solid->collectSolidDescendantNames(items, solidException);
}

//////////////////////////////////////////////////////////////////
//  Collecting kinematic hidden parameters of Solid descendants //
//////////////////////////////////////////////////////////////////

void OmSolid::collectHiddenKinematicParameters(HiddenKinematicParametersMap &map, int &counter) const {
  const bool merger = isSolidMerger();
  const OmVector3 *t = NULL;
  const OmRotation *r = NULL;
  const OmVector3 *l = NULL;
  const OmVector3 *a = NULL;
  bool copyTranslation = false;
  bool copyRotation = false;
  OmVector3 translationToBeCopied;
  OmRotation rotationToBeCopied;

  if (mSolidMerger == NULL || merger) {
    // TODO: implement an mIsVisible flag in OmNode for sake of efficiency
    const OmBasicJoint *parentJoint = jointParent();
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

    // ODE removed: the merged body's velocity read that used to fill `l` / `a`
    // is gone, so hidden-field save of a Solid's linear/angular velocity is now
    // UNIMPLEMENTED (both stay NULL, exactly as the zeroing stub already made
    // them). The per-step Newton readback populates the mLinearVelocity /
    // mAngularVelocity FIELDS; it is not read from here.
  }

  PositionMap positions;
  const int size = mJointChildren.size();
  for (int i = 0; i < size; ++i) {
    const OmJoint *const j = dynamic_cast<OmJoint *>(mJointChildren[i]);
    if (j) {
      OmVector3 v(NAN, NAN, NAN);

      // TODO: implement an mIsVisible flag in OmNode for sake of efficiency
      const OmJointParameters *const p = j->parameters();
      if ((p == NULL || !OmVrmlNodeUtilities::isVisible(p->findField("position"))) && j->position() != j->initialPosition())
        v[0] = j->position();

      if (j->nodeType() == WB_NODE_HINGE_2_JOINT || j->nodeType() == WB_NODE_BALL_JOINT) {
        const OmJointParameters *const p2 = j->parameters2();
        if ((p2 == NULL || !OmVrmlNodeUtilities::isVisible(p2->findField("position"))) &&
            j->position(2) != j->initialPosition(2))
          v[1] = j->position(2);
      }

      if (j->nodeType() == WB_NODE_BALL_JOINT) {
        const OmJointParameters *const p3 = j->parameters3();
        if ((p3 == NULL || !OmVrmlNodeUtilities::isVisible(p3->findField("position"))) &&
            j->position(3) != j->initialPosition(3))
          v[2] = j->position(3);
      }

      if (!std::isnan(v[0]) || !std::isnan(v[1]) || !std::isnan(v[2]))
        positions.insert(i, new OmVector3(v));
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
  foreach (const OmSolid *const solid, mSolidChildren)
    solid->collectHiddenKinematicParameters(map, counter);
}

///////////////////
// Hidden fields //
///////////////////

void OmSolid::saveHiddenFieldValues() const {
  // ODE removed: this refreshed mLinearVelocity / mAngularVelocity from the ODE
  // body before a save, and is now UNIMPLEMENTED. Dropping it is strictly better
  // than keeping it: against the inert stub it overwrote both fields with ZEROS,
  // clobbering the values the per-step Newton readback had just written.
}

////////////////////////
//  Kinematic solids  //
////////////////////////

void OmSolid::enable(bool enabled, bool ode) {
  assert(mIsKinematic);
  // D1.4: the WREN visibility toggle died with WREN (a disabled kinematic solid no longer
  // hides visually; nothing in the tree relied on it under the wgpu default).
  (void)enabled;
  (void)ode;
}

void OmSolid::exportUrdfShape(OmWriter &writer, const QString &geometry, const OmPose *pose, const OmVector3 &offset) const {
  const QStringList element = QStringList() << "visual"
                                            << "collision";
  for (int j = 0; j < element.size(); ++j) {
    writer.increaseIndent();
    writer.indent();
    writer << QString("<%1>\n").arg(element[j]);
    writer.increaseIndent();
    if (pose != this || !offset.isNull()) {
      OmVector3 translation = pose->translation() + offset;
      OmRotation rotation = pose->rotation();
      writer.indent();
      if (pose == this) {
        rotation = OmRotation(0.0, 1.0, 0.0, 0.0);
        translation = offset;
      }
      writer << QString("<origin xyz=\"%1\" rpy=\"%2\"/>\n")
                  .arg(translation.toString(OmPrecision::FLOAT_ROUND_6))
                  .arg(rotation.toMatrix3().toEulerAnglesZYX().toString(OmPrecision::FLOAT_ROUND_6));
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

bool OmSolid::exportNodeHeader(OmWriter &writer) const {
  if (writer.isUrdf()) {
    const bool ret = OmMatter::exportNodeHeader(writer);
    if (!ret) {
      if (boundingObject()) {
        QList<OmNode *> nodes = boundingObject()->subNodes(true);
        for (int i = 0; i < nodes.size(); ++i) {
          const OmNode *node = nodes[i];
          const OmCylinder *cylinder = dynamic_cast<const OmCylinder *>(node);
          const OmBox *box = dynamic_cast<const OmBox *>(node);
          const OmSphere *sphere = dynamic_cast<const OmSphere *>(node);
          const OmCapsule *capsule = dynamic_cast<const OmCapsule *>(node);
          if (box || cylinder || sphere || capsule) {
            const OmPose *pose = OmNodeUtilities::findUpperPose(node);
            QList<std::pair<QString, OmVector3>> geometries;  // string of the geometry and its offset

            if (box) {
              std::pair<QString, OmVector3> pair;
              pair.first = QString("<box size=\"%1 %2 %3\"/>\n").arg(box->size().x()).arg(box->size().y()).arg(box->size().z());
              geometries << pair;
            } else if (cylinder) {
              std::pair<QString, OmVector3> pair;
              pair.first = QString("<cylinder radius=\"%1\" length=\"%2\"/>\n").arg(cylinder->radius()).arg(cylinder->height());
              geometries << pair;
            } else if (capsule) {
              std::pair<QString, OmVector3> pair;
              pair.first = QString("<cylinder radius=\"%1\" length=\"%2\"/>\n").arg(capsule->radius()).arg(capsule->height());
              geometries << pair;
              pair.first = QString("<sphere radius=\"%1\"/>\n").arg(capsule->radius());
              pair.second = OmVector3(0.0, 0.5 * capsule->height(), 0.0);
              if (pose)
                pair.second = pose->rotation().toMatrix3() * pair.second;
              geometries << pair;
              pair.first = QString("<sphere radius=\"%1\"/>\n").arg(capsule->radius());
              pair.second = OmVector3(0.0, -0.5 * capsule->height(), 0.0);
              if (pose)
                pair.second = pose->rotation().toMatrix3() * pair.second;
              geometries << pair;
            } else if (sphere) {
              std::pair<QString, OmVector3> pair;
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

  return OmMatter::exportNodeHeader(writer);
}

void OmSolid::exportNodeFooter(OmWriter &writer) const {
  if (writer.isW3d() && boundingObject())
    boundingObject()->exportBoundingObjectToW3d(writer);

  OmMatter::exportNodeFooter(writer);
}
