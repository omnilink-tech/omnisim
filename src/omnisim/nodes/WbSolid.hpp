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

#ifndef WB_SOLID_HPP
#define WB_SOLID_HPP

#include "WbHiddenKinematicParameters.hpp"
#include "WbMFColor.hpp"
#include "WbMatter.hpp"
#include "WbPhysicsBackend.hpp"
#include "WbPolygon.hpp"
#include "WbSolidMerger.hpp"
#include "WbSolidUtilities.hpp"

#include <QtCore/QList>
#include <QtCore/QPointer>
#include <QtCore/QSet>

class WbBasicJoint;
class WbPhysics;
class WbPropeller;
class WbRobot;
class WbSupportPolygonRepresentation;

struct WrTransform;
struct WrRenderable;
struct WrStaticMesh;
struct WrMaterial;

struct dImmersionGeom;

class WbSolid : public WbMatter {
  Q_OBJECT

public:
  // constructors and destructor
  explicit WbSolid(WbTokenizer *tokenizer = NULL);
  WbSolid(const WbSolid &other);
  explicit WbSolid(const WbNode &other);
  virtual ~WbSolid() override;

  // list of finalized solids
  static const QList<const WbSolid *> &solids() { return cSolids; }

  // P3.7 of cuda-newton-physics-plan.md: walk cSolids and register any
  // that opted into a Newton backend but haven't yet been added to its
  // model. Deferred from postFinalize so matrix() returns the correct
  // world transform (parent chain is fully wired by flush time).
  // Idempotent across calls -- already-registered solids are skipped.
  static void flushPendingNewtonRegistrations();

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_SOLID; }
  void downloadAssets() override;
  void createWrenObjects() override;
  void preFinalize() override;
  void validateProtoNode() override;
  void postFinalize() override;
  void propagateSelection(bool selected) override;
  void saveHiddenFieldValues() const;
  void setMatrixNeedUpdate() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;
  void updateSegmentationColor(const WbRgb &color) override;

  // processing before / after ODE world step
  virtual void prePhysicsStep(double ms);
  virtual void postPhysicsStep();

  void createOdeObjects() override;

  // remove and delete all solid children
  void deleteAllSolids() override;

  // field accessors
  const QString &contactMaterial() const { return mContactMaterial->value(); }
  WbPhysics *physics() const;
  const WbMFNode &immersionProperties() const { return *mImmersionProperties; }
  float radarCrossSection() const { return mRadarCrossSection->value(); }
  // physicsBackend: opt-in selector for GPU-resident dynamics. Returns the
  // backend resolved through WbPhysicsBackendRegistry — silently falls back
  // to ODE when the requested backend is unavailable (e.g. NEWTON=OFF build,
  // no CUDA hardware, missing wheels). The string accessor returns the raw
  // field value for diagnostic logging. Subclass models that don't declare
  // the field (every WbSolidDevice .wrl — Accelerometer, Camera, GPS, …)
  // report the default "ode" so the world loads instead of crashing.
  const QString &physicsBackendName() const;
  WbPhysicsBackend *physicsBackend() const;
  // P3.10: walk up the scene tree returning the first non-"ode" backend
  // value seen on any ancestor Solid, or this Solid's own value if it's
  // already non-default. Lets `physicsBackend "newton"` on a URDFRobot's
  // outer Robot propagate to the chassis + wheel Solids it auto-generates,
  // so a single field on the wrapper opts the entire articulated tree
  // into Newton.
  // downgradedByGate (optional out): set true ONLY when this returns "ode"
  // because the §4.1 capability gate pushed an otherwise-Newton-bound "auto"
  // articulation onto ODE (a SILENT Newton->ODE downgrade) -- NOT when "ode"
  // came from an explicit local / ancestor / world choice. The Newton-
  // enforcement sweep uses it to fatal on a silent downgrade while leaving a
  // deliberate ODE opt-out alone.
  QString effectivePhysicsBackendName(bool *downgradedByGate = nullptr) const;
  // §4.1 capability gate (default-flip-plan.md): is this Solid's whole articulation Newton-capable?
  // False (=> a bare-"auto" articulation resolves to ODE, never silently degraded/solver-mixed) when it
  // uses a Newton-unsupported feature: a non-Hinge/Slider joint (correctness) or a mesh boundingObject
  // (fidelity: Newton only AABB-approximates it). Resolved at the articulation ROOT so every body in one
  // coupled system shares one answer (no mixing — the Spot frozen-sensor failure mode). Cached.
  // Monotonically safe: ODE handles everything; OMNISIM_AUTO_NO_CAPABILITY_GATE bypasses for power use.
  // reasonOut (optional): set to "capable" | "mesh" | "joint" so the OMNISIM_PROBE_GATE coverage meter
  // can histogram WHY an articulation is / isn't Newton-eligible (newton-ode-replacement-plan.md W0.2).
  bool articulationNewtonCapable(const char **reasonOut = nullptr) const;
  // P3.2/P3.7: index in the active Newton model, or -1 if not registered.
  int newtonBodyIndex() const { return mNewtonBodyIndex; }
  // P3.10c: merger-aware accessor. If this Solid is part of a SolidMerger
  // but not the leader, returns the leader's newtonBodyIndex (the body
  // that carries the combined inertia in Newton's model). Otherwise
  // returns this Solid's own index. Used by WbBasicJoint when wiring
  // hinge endpoints so wheels attached to (e.g.) `top_chassis_link` get
  // attached to the chassis-merger leader's body in Newton, mirroring
  // ODE's WbSolidMerger semantics.
  int effectiveNewtonBodyIndex() const;
  // Polymorphic body handle for the dispatcher. Pairs with
  // physicsBackend() so callers can do
  //   solid->physicsBackend()->getBodyPosition(solid->bodyHandle(), pos)
  // without caring whether this Solid lives on ODE or Newton. Returns
  // NULL if the Solid has no body on either backend.
  //
  // Resolution:
  //   - mNewtonBodyIndex >= 0  -> handleFromIndex(mNewtonBodyIndex)
  //   - else if bodyMerger() non-null -> (void*) bodyMerger() (ODE convention)
  //   - else -> NULL
  // Newton wins when both exist because the bridge runs ODE alongside
  // Newton for collision detection only; the dynamic state lives on
  // Newton and is what callers reading pose / velocity want.
  WbBodyHandle bodyHandle() const;
  int recognitionColorSize() const { return mRecognitionColors->size(); }
  const WbRgb &recognitionColor(int index) const { return mRecognitionColors->item(index); }
  // hidden field accessors
  const WbVector3 &linearVelocity() const { return mLinearVelocity->value(); }
  const WbVector3 &angularVelocity() const { return mAngularVelocity->value(); }

  // return the velocity relatively to a parent Solid (or absolute velocity if parentSolid is NULL)
  WbVector3 relativeLinearVelocity(const WbSolid *parentSolid = NULL) const;
  WbVector3 relativeAngularVelocity(const WbSolid *parentSolid = NULL) const;

  void setLinearVelocity(const double velocity[3]);
  void setAngularVelocity(const double velocity[3]);

  // ODE objects accessors
  dJointID joint() const { return mJoint; }
  dBodyID body() const;
  double mass() const;
  const double *inertiaMatrix() const;
  double globalMass() const { return mGlobalMass; }
  double globalVolume() const { return mGlobalVolume; }
  const WbVector3 &globalCenterOfMass() const { return mGlobalCenterOfMass; }
  // global center of mass: adds up the solid itself and all its descendants
  const WbVector3 &computedGlobalCenterOfMass() {
    updateGlobalCenterOfMass();
    return mGlobalCenterOfMass;
  }
  void updateCenterOfMass();  // update the center of mass according to the Physics node specification
  const WbVector3 &centerOfMass() const { return mCenterOfMass; }
  dMass *referenceMass() const { return mReferenceMass; }
  // returns inertia and CoM relative to solid center in the local frame coordinates
  dMass *odeMass() const { return mOdeMass; }

  const WbVector3 &centerOfBuoyancy() const { return mCenterOfBuoyancy; }

  const QVector<WbVector3> &contactPoints(bool includeDescendants = false) const {
    return includeDescendants ? mGlobalListOfContactPoints : mListOfContactPoints;
  }
  const QVector<WbVector3> &computedContactPoints(bool includeDescendants = false);
  const QVector<const WbSolid *> &computedSolidPerContactPoints();
  // Per-contact penetration depth (ODE dContactGeom.depth), aligned 1:1 with
  // computedContactPoints(includeDescendants). Real engine-computed contact
  // magnitude. See physics-contact-impulse-api.md.
  const QVector<double> &computedContactPointDepths(bool includeDescendants = false);

  // Solid merger
  QPointer<WbSolidMerger> solidMerger() const { return mSolidMerger; }
  void setupSolidMerger();
  void setupSolidMergers();
  bool isSolidMerger() const { return !mIsKinematic && mSolidMerger && mSolidMerger->solid() == this; }
  bool mergerIsSet() const { return mMergerIsSet; }
  dBodyID bodyMerger() const;

  // New joints
  void appendJointParent(WbBasicJoint *joint);
  void removeJointParent(WbBasicJoint *joint);

  // set up joints for special nodes:
  // - fixed joint between TouchSensor and parent body
  // - fixed joint between dynamic solid child and kinematic parent body (static environment)
  void setOdeJointToUpperSolid();

  // dynamical state
  bool isDynamic() const { return !mIsKinematic; }
  bool isKinematic() const { return mIsKinematic; }

  // sleeping flags
  bool isSleeping() const;

  // awakening
  void awake();
  static void awakeSolids(WbGroup *group);

  void resetPhysics(bool recursive = true) override;
  // pause/resume physics computation on the current solid and its descandants
  void pausePhysics(bool resumeAutomatically = false) override;
  void resumePhysics() override;

  // handle artifical moves triggered by the user or a Supervisor
  void jerk(bool resetVelocities = true, bool rootJerk = true) override;
  void notifyChildJerk(WbPose *childNode);

  // physics setters
  void addForceAtPosition(const WbVector3 &force, const WbVector3 &position);
  void addTorque(const WbVector3 &torque);
  // W3.1: route a world-frame force-at-position + torque to this Solid's Newton body (if any). Returns true
  // when routed to Newton; false -> not Newton-backed (caller falls back to ODE). See WbSolid.cpp.
  bool applyExternalForceNewton(const WbVector3 &force, const WbVector3 &worldPos,
                                const WbVector3 &torque) const;
  // W3.2: route a mid-step velocity set to this Solid's Newton body (angular=false -> linear, true ->
  // angular). Returns true when routed to Newton; false -> not Newton-backed (caller falls back to ODE).
  bool setNewtonBodyVel(const double v[3], bool angular) const;

  // physics node setter
  void setInertiaMatrixFromBoundingObject();

  // selection
  void enable(bool enabled, bool ode = true);  // remove a kinematic solid from simulation or reintroduce it

  // support polygon representation
  bool supportPolygonRepresentationEnabled() const { return mSupportPolygonRepresentationIsEnabled; }
  bool showSupportPolygonRepresentation(bool enabled);
  unsigned char staticBalance();  // returns zero if unstable, one otherwise

  // global center of mass representation
  bool globalCenterOfMassRepresentationEnabled() const { return mGlobalCenterOfMassRepresentationIsEnabled; }
  bool showGlobalCenterOfMassRepresentation(bool enabled);

  // utilities
  WbRobot *robot() const;
  bool isTopSolid() const { return (this == topSolid()); }
  virtual WbSolid *findSolid(const QString &name, const WbSolid *const exception = NULL);
  bool belongsToStaticBasis() const;  // Returns true if all solid ancestors have no physics

  // lists of WbSolid and WbBasicJoint children
  const QVector<WbSolid *> &solidChildren() const { return mSolidChildren; }
  const QVector<WbBasicJoint *> &jointChildren() const { return mJointChildren; }

  // ODE mass adjustments
  void correctOdeMass(const dMass *mass, WbBaseNode *node, bool adjustSolidMass = true);

  // ODE positioning
  void resetJointsToLinkedSolids();  // reset joint to any linked solid to this one

  // update the node transform matrix based on the newly computed ODE transform matrix
  // it loops through all the ancestor Solid nodes with a body and updates them
  void updateTransformForPhysicsStep();

  // Density
  double volume() const;
  double density() const;
  double averageDensity() const;
  void updateGlobalVolume();

  // Contact points management
  const WbPolygon &supportPolygon();

  // Immersion
  bool showCenterOfBuoyancyRepresentation(bool enabled);
  bool centerOfBuoyancyRepresentationEnabled() const { return mCenterOfBuoyancyRepresentationIsEnabled; }

  // Collecting and restoring info on descendants
  void collectSolidDescendantNames(QStringList &items, const WbSolid *const solidException = NULL) const;
  void collectHiddenKinematicParameters(WbHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                        int &counter) const override;

  // Threshold to handle mass round off errors after resize events
  static const double MASS_ZERO_THRESHOLD;

  WbBasicJoint *jointParent() const;

  // save previous position and orientation
  void savePreviousTransform();

  // print a warning if the current solid is moved in kinematics mode and it
  // has any dynamic Solid descendant node that won't follow it
  void printKinematicWarningIfNeeded();

  // Functions to compute unique name, find a Solid based on the unique name and resolve name clashes
  QString computeUniqueName() const;
  WbSolid *findDescendantSolidFromUniqueName(QStringList &names) const;
  void resolveNameClashIfNeeded(bool automaticallyChange, bool recursive, const QList<WbSolid *> &siblings,
                                QSet<const QString> *topSolidNameSet);
  static WbSolid *findSolidFromUniqueName(const QString &name);
  static QStringList splitUniqueNamesByEscapedPattern(const QString &text, const QString &pattern);

signals:
  void contactPointsRequested();
  void massPropertiesChanged();
  void physicsPropertiesChanged();
  void positionChangedArtificially();

public slots:
  // recursions through solid children with bounding objects for material updates
  void propagateBoundingObjectMaterialUpdate(bool onSelection = false) override;
  void updateGlobalCenterOfMass();
  void updateGraphicalGlobalCenterOfMass();
  void resetPhysicsIfRequired(bool changedFromSupervisor);
  virtual void updateChildren();
  void updateBoundingObject() override;

protected:
  // this constructor is reserved for derived classes only
  WbSolid(const QString &modelName, WbTokenizer *tokenizer);

  // physics accessors
  dBodyID upperSolidBody() const;
  const QList<WbBasicJoint *> &jointParents() const { return mJointParents; }
  void setJointParents();

  // to-be-reimplemented in derived classes
  void updateName() override;
  virtual dJointID createJoint(dBodyID body, dBodyID parentBody, dWorldID world) const;
  // check if a ODE joint is needed between current and upper solid
  // special cases for: TouchSensor
  virtual bool needJointToUpperSolid(const WbSolid *upperSolid) const;

  // Avoids joint destruction, which is safer with respect to physics plugins
  void setJoint(dJointID joint, dBodyID body, dBodyID parentBody) const;

  // Renders the frame axes and the center of mass
  void applyVisibilityFlagsToWren(bool selected) override;
  void applyChangesToWren() override;
  void applyMassCenterToWren();

  // Solid merger, i.e. solid ancestor (possibly the solid itself) that owns the mass, body and dGeoms of this solid..
  virtual void setSolidMerger();
  // Non NULL only if this solid is dynamic and is not related to its dynamic parent by a joint
  QPointer<WbSolidMerger> mSolidMerger;

  bool isInsertedOdeGeomPositionUpdateRequired() const override { return mIsKinematic; }

  // export
  bool exportNodeHeader(WbWriter &writer) const override;
  void exportNodeFooter(WbWriter &writer) const override;

protected slots:
  void updateTranslation() override;
  void updateRotation() override;
  void updateLineScale() override;
  virtual void updateIsLinearVelocityNull();
  virtual void updateIsAngularVelocityNull();

private:
  // Push the Solid's current mTranslation/mRotation into Newton's
  // body_q so a Supervisor field write (e.g. reset_episode()) actually
  // moves the Newton-tracked body. No-op for non-Newton Solids.
  void syncNewtonPoseFromFields();

  WbSolid &operator=(const WbSolid &);  // non copyable
  void init();

  void exportUrdfShape(WbWriter &writer, const QString &geometry, const WbPose *pose, const WbVector3 &offset) const;

  // list of finalized solids
  static QList<const WbSolid *> cSolids;

  // user accessible fields
  WbSFString *mContactMaterial;
  WbSFString *mPhysicsBackend;
  WbSFNode *mPhysics;
  WbMFNode *mImmersionProperties;
  WbSFDouble *mRadarCrossSection;
  WbMFColor *mRecognitionColors;

  // P3.2: index of this Solid's body in the WbNewtonBackend's Newton
  // model, or -1 when not registered (ode-backed or Newton unavailable).
  // Populated in postFinalize, consumed by postPhysicsStep for pose
  // readback.
  int mNewtonBodyIndex;

  // §4.1 capability gate: cached articulation-Newton-capability (-1 unknown, 0 no, 1 yes), computed
  // once at the articulation root and shared by every body in it. mutable: filled by the const
  // articulationNewtonCapable() on first query.
  mutable int mNewtonArticulationCapable = -1;

  // P8.2: true when mNewtonBodyIndex refers to a STATIC (mass=0) Newton
  // body registered via addStaticBody (arena floor / walls / obstacles
  // that opted into Newton but have no Physics node). Static bodies are
  // pinned by the solver and never integrate, so they skip the per-step
  // pose writeback AND the syncNewtonPoseFromFields supervisor bridge
  // (in particular resetJointsToDefaults, which on a root Solid would
  // otherwise clobber the shared articulation every tick).
  bool mNewtonBodyIsStatic;

  // hidden fields
  WbSFVector3 *mLinearVelocity;
  WbSFVector3 *mAngularVelocity;

  bool mIsLinearVelocityNull;
  bool mIsAngularVelocityNull;

  // Clone
  WbNode *clone() const override { return new WbSolid(*this); }

  // Selection
  bool mSelected;

  // ODE
  dJointID mJoint;
  bool mUpdatedInStep;       // used to update Transform coordinated to setup ray collisions (based on pre-physics step values)
  bool mResetPhysicsInStep;  // used to completely reset physics when the solid is also moved in the same step
  void setGeomAndBodyPositions();
  void applyPhysicsTransform();
  void resetJoints();  // reset joint to any linked solid to this one or to one of its descendants
  void setBodiesAndJointsToParents();
  void setJointChildrenWithReferencedEndpoint();
  void updateKinematicPlaceableGeomPosition(dGeomID g);
  bool resetJointPositions(bool allParents = false);
  void handleJerk() override;

  QVector<WbPose *> mMovedChildren;
  void childrenJerk();

  void resetSingleSolidPhysics();

  // set position and orientation from physics integration (in contrast with supervisor or user setup)
  void inline setTransformFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

  // Mass and density
  void addMassFromInsertedNode(WbBaseNode *node);
  bool checkMassAndDensity() const;
  double mAverageDensity;
  dMass *mMassAroundCoM;
  dMass *mOdeMass;         // around Solid frame center
  dMass *mReferenceMass;   // the mass of the solid when the density is uniformly set to 1000 kg/m^3
  bool mUseInertiaMatrix;  // indicates that the WbSolid uses the latest valid inertia matrix field for ODE physics computation
  WbVector3 mCenterOfMass;

  // ODE mass adjustments
  void createOdeMass(bool reset = true);
  // adjust the mass of the WbSolid after insertion or deletion in the boundingObject
  void adjustOdeMass(bool mergeMass = true);
  void addMass(WbBaseNode *node) { WbSolidUtilities::addMass(mReferenceMass, node, 1000); }
  // subtracts the mass of a deleted node in the boundingObject (called from WbGeometry)
  void subtractOdeMass(const dMass *mass, bool adjustSolidMass = true);
  void setDefaultMassSettings(bool applyCenterOfMassTranslation, bool warning = true);

  // Global center of mass variables
  WbVector3 mGlobalCenterOfMass;  // expressed in absolute (world) coordinates; if the total mass is zero and no CoM is
                                  // specified, it defaults to the Solid frame's center
  bool mGlobalCenterOfMassRepresentationIsEnabled;
  double mGlobalVolume;  // volume of the solid and all its descendants
  double mGlobalMass;    // mass of the solid and all its descendants

  // Immersion
  WbVector3 mCenterOfBuoyancy;
  bool mHasExtractedImmersions;
  bool mCenterOfBuoyancyRepresentationIsEnabled;
  void extractImmersions();
  void updateCenterOfBuoyancy();
  QVector<dImmersionGeom> mListOfImmersions;

  // Sleep flag
  bool mWasSleeping;
  virtual void updateSleepFlag();

  // Merger setup flag
  bool mMergerIsSet;
  bool mIsPermanentlyKinematic;

  bool mIsKinematic;

  bool mKinematicWarningPrinted;
  bool mHasDynamicSolidDescendant;

  // Contact points
  QVector<WbVector3> mListOfContactPoints;
  QVector<WbVector3> mGlobalListOfContactPoints;  // includes contacts of Solid descendants
  QVector<double> mListOfContactPointDepths;        // 1:1 with mListOfContactPoints
  QVector<double> mGlobalListOfContactPointDepths;  // 1:1 with mGlobalListOfContactPoints
  // defines the colliding Solid for each for the contact point in mGlobalListOfContactPoints
  QVector<const WbSolid *> mSolidPerContactPoints;
  bool mHasExtractedContactPoints;
  void extractContactPoints();  // populates both local and global lists

  // Support polygon variables

  // used to index the gravity basis vector when projecting contact points on a plan orthogonal to the gravity direction
  enum { X, Y, Z };
  WbSupportPolygonRepresentation *mSupportPolygonRepresentation;
  void deleteSupportPolygonRepresentation();
  mutable WbPolygon mSupportPolygon;
  mutable bool mSupportPolygonNeedsUpdate;
  double mY;  // minimum on the y-coordinate over all contact points
  bool mSupportPolygonRepresentationIsEnabled;

  // Relatives
  mutable bool mHasSearchedRobot;
  QVector<WbSolid *> mSolidChildren;  // either direct children, solid children of a WbGroup appearing in the children list,
                                      // solid endPoints of joint children or solid endPoint of a slot
  QVector<WbBasicJoint *> mJointChildren;
  QVector<WbPropeller *> mPropellerChildren;
  QList<WbBasicJoint *> mJointParents;  //  direct joint parent and joints whose endPoint is this solid
  mutable WbRobot *mRobot;

  // Update of children lists
  void collectSolidChildren(const WbGroup *group, bool connectSignals, QVector<WbSolid *> &solidChildren,
                            QVector<WbBasicJoint *> &jointChildren, QVector<WbPropeller *> &propellerChildren);
  void updateDynamicSolidDescendantFlag();

  void setOdeInertiaMatrix();
  void createOdeGeoms() override;

  // WREN objects
  WrTransform *mCenterOfMassTransform;
  WrRenderable *mCenterOfMassRenderable;
  WrMaterial *mCenterOfMassMaterial;
  WrStaticMesh *mCenterOfMassMesh;

  WrTransform *mGlobalCenterOfMassTransform;
  WrRenderable *mGlobalCenterOfMassRenderable;
  WrMaterial *mGlobalCenterOfMassMaterial;
  WrStaticMesh *mGlobalCenterOfMassMesh;

  WrTransform *mCenterOfBuoyancyTransform;
  WrRenderable *mCenterOfBuoyancyRenderable;
  WrMaterial *mCenterOfBuoyancyMaterial;

  WbHiddenKinematicParameters::HiddenKinematicParameters *mOriginalHiddenKinematicParameters;
  bool applyHiddenKinematicParameters(const WbHiddenKinematicParameters::HiddenKinematicParameters *hkp, bool backupPrevious);
  bool restoreHiddenKinematicParameters(const WbHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                        int &counter) override;
  bool resetHiddenKinematicParameters() override;

  void setGeomMatter(dGeomID g, WbBaseNode *node = NULL) override;

  bool mNameClashResolved;

private slots:
  void updateChildrenAfterJointEndPointChange(WbBaseNode *node);
  void updatePhysics();
  void updateRadarCrossSection();
  void updateRecognitionColors();
  void updateOdeMass();
  void applyToOdeMass();
  void updateOdeInertiaMatrix();
  void updateOdeCenterOfMass();
  void updateOdeDamping();
  void createOdeGeomFromInsertedGroupItem(WbBaseNode *node) override;
  void removeBoundingGeometry() override;
  void updateTopSolidGlobalMass() const;
  // connected to the physicsStepEnded() signal in computedContactPoints() and disconnected when called
  void resetContactPoints();
  void resetContactPointsAndSupportPolygon();  // connected to the physicsStepEnded() signal
  void resetImmersions();
  void refreshPhysicsRepresentation();             // redraws CoM, support polygon and CoB after physics properties change
  void refreshSupportPolygonRepresentation();      // redraws after each physics step
  void refreshGlobalCenterOfMassRepresentation();  // redraws the global CoM after each physics step
  void refreshCenterOfBuoyancyRepresentation();    // redraws the CoB after each physics step
  void onSimulationModeChanged();                  // deletes the support polygon representation if need be
  void displayWarning();
};

void inline WbSolid::setTransformFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle) {
  WbPose::setTranslationAndRotationFromOde(tx, ty, tz, rx, ry, rz, angle);
  WbPose::updateTranslationAndRotation();
}

#endif
