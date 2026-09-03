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

#ifndef OM_SOLID_HPP
#define OM_SOLID_HPP

#include "OmHiddenKinematicParameters.hpp"
#include "OmInertia.hpp"
#include "OmMatter.hpp"
#include "OmPhysicsHandles.hpp"
#include "OmPolygon.hpp"

#include <QtCore/QList>
#include <QtCore/QPointer>
#include <QtCore/QSet>

class OmBasicJoint;
class OmMFColor;
class OmPhysics;
class OmPhysicsBackend;
class OmPropeller;
class OmRgb;
class OmRobot;
class OmSolidMerger;



class OmSolid : public OmMatter {
  Q_OBJECT

public:
  // constructors and destructor
  explicit OmSolid(OmTokenizer *tokenizer = NULL);
  OmSolid(const OmSolid &other);
  explicit OmSolid(const OmNode &other);
  virtual ~OmSolid() override;

  // list of finalized solids
  static const QList<const OmSolid *> &solids() { return cSolids; }

  // P3.7 of cuda-newton-physics-plan.md: walk cSolids and register any
  // that opted into a Newton backend but haven't yet been added to its
  // model. Deferred from postFinalize so matrix() returns the correct
  // world transform (parent chain is fully wired by flush time).
  // Idempotent across calls -- already-registered solids are skipped.
  static void flushPendingNewtonRegistrations();

  // W1.7 mid-run physics rebuild: capture every registered dynamic body's
  // live velocity into the pending-velocity replay slots (the FIX-5 path in
  // flushPendingNewtonRegistrations re-seeds them at re-registration), then
  // forget all Newton registration state so the next flush re-registers the
  // whole scene into a fresh Newton world. Called by
  // OmSimulationWorld::step() when a rebuild was requested; the teardown of
  // the old Newton world happens in the same breath (OmNewtonBackend).
  static void captureNewtonVelocitiesForRebuild();
  static void resetNewtonRegistrationsForRebuild();

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
  void updateSegmentationColor(const OmRgb &color) override;

  // processing before / after ODE world step
  virtual void prePhysicsStep(double ms);
  virtual void postPhysicsStep();

  void createOdeObjects() override;

  // remove and delete all solid children
  void deleteAllSolids() override;

  // field accessors
  const QString &contactMaterial() const { return mContactMaterial->value(); }
  OmPhysics *physics() const;
  float radarCrossSection() const { return mRadarCrossSection->value(); }
  // physicsBackend: the per-Solid selector. Every value resolves to Newton
  // through OmPhysicsBackendRegistry (the only backend); a retired value such
  // as "ode" warns once per world and runs on Newton too. The string accessor
  // returns the raw field value. Subclass models that don't declare the field
  // (every OmSolidDevice .wrl — Accelerometer, Camera, GPS, …) report "auto"
  // so the world loads instead of crashing.
  const QString &physicsBackendName() const;
  OmPhysicsBackend *physicsBackend() const;
  // Returns "newton" when this Solid or an ancestor Solid pins it explicitly
  // (so `physicsBackend "newton"` on a URDFRobot's outer Robot governs the
  // chassis + wheel Solids it auto-generates), otherwise "auto" -- which also
  // means Newton.
  // downgradedByGate (optional out): set true ONLY when the §4.1 capability
  // gate kept an "auto" articulation OFF Newton because it uses an unsupported
  // feature. With no second solver that is a SILENT omission, and the Newton-
  // enforcement sweep fatals on it (except a kinematic chain, which warns).
  QString effectivePhysicsBackendName(bool *downgradedByGate = nullptr) const;
  // Convenience for the joint flush: the downgradedByGate verdict alone.
  bool gatedAwayFromNewton() const;
  // §4.1 capability gate (default-flip-plan.md): is this Solid's whole articulation Newton-capable?
  // False (=> a bare-"auto" articulation resolves to ODE, never silently degraded/solver-mixed) when it
  // uses a Newton-unsupported feature: a non-Hinge/Slider joint (correctness) or a mesh boundingObject
  // (fidelity: Newton only AABB-approximates it). Resolved at the articulation ROOT so every body in one
  // coupled system shares one answer (no mixing — the Spot frozen-sensor failure mode). Cached.
  // Monotonically safe: ODE handles everything; OMNISIM_AUTO_NO_CAPABILITY_GATE bypasses for power use.
  // reasonOut (optional): set to "capable" | "mesh" | "joint" so the OMNISIM_PROBE_GATE coverage meter
  // can histogram WHY an articulation is / isn't Newton-eligible (newton-ode-replacement-plan.md W0.2).
  bool articulationNewtonCapable(const char **reasonOut = nullptr) const;
  // OMNISIM_NEWTON_KINEMATIC (value-parsed: "0"/"false"/"off"/"no" = off,
  // default OFF) -- kernel blocker #4, _scratch/design_kinematic_inertia.md
  // Part 1. When ON, a physics-less (kinematic) Solid gets a NATIVE Newton
  // path instead of gating its articulation to ODE: joint endpoints without a
  // Physics node register as KINEMATIC (MuJoCo mocap) bodies carrying their
  // boundingObject shapes, and every engine-side pose update (motor kinematic
  // control, supervisor writes, velocity integration, ancestor motion) is
  // pushed into the solver via setKinematicPose. OFF = byte-identical
  // behavior (the capability gate still routes kinematic articulations to
  // ODE with the existing warning).
  static bool newtonKinematicNativeEnabled();
  // P3.2/P3.7: index in the active Newton model, or -1 if not registered.
  int newtonBodyIndex() const { return mNewtonBodyIndex; }
  // P3.10c: merger-aware accessor. If this Solid is part of a SolidMerger
  // but not the leader, returns the leader's newtonBodyIndex (the body
  // that carries the combined inertia in Newton's model). Otherwise
  // returns this Solid's own index. Used by OmBasicJoint when wiring
  // hinge endpoints so wheels attached to (e.g.) `top_chassis_link` get
  // attached to the chassis-merger leader's body in Newton, mirroring
  // ODE's OmSolidMerger semantics.
  int effectiveNewtonBodyIndex() const;
  // The first Newton
  // body index found walking THIS Solid and then its ancestor Solids (each
  // merger-aware via effectiveNewtonBodyIndex). Under the P3.10c fold a
  // device Solid (Connector, VacuumGripper, TouchSensor) has no body of its
  // own -- the merge leader's body is what a weld or wrench read must anchor
  // on. -1 when the whole chain owns no Newton body (ODE world / kinematic).
  int nearestNewtonBodyIndex() const;
  // Reverse lookup over the finalized-solid census: the Solid whose
  // mNewtonBodyIndex is `idx`, or NULL. Used by VacuumGripper's native
  // contact-based attach trigger (the ODE odeNearCallback source is dead for
  // Newton bodies) to turn a OmNewtonContact body index back into the Solid
  // it may weld to. O(#solids); called only while a gripper is on and
  // unattached.
  static OmSolid *findSolidByNewtonBodyIndex(int idx);
  // Polymorphic body handle for the dispatcher. Pairs with
  // physicsBackend() so callers can do
  //   solid->physicsBackend()->getBodyPosition(solid->bodyHandle(), pos)
  // without caring whether this Solid lives on ODE or Newton. Returns
  // NULL if the Solid has no body on either backend.
  //
  // Resolution:
  //   - mNewtonBodyIndex >= 0  -> handleFromIndex(mNewtonBodyIndex)
  //   - else -> NULL
  // Newton wins when both exist because the bridge runs ODE alongside
  // Newton for collision detection only; the dynamic state lives on
  // Newton and is what callers reading pose / velocity want.
  OmBodyHandle bodyHandle() const;
  // Fold-aware body handle: bodyHandle() answers only when THIS Solid owns a
  // body, but under the P3.10c fold a device carrier (a nested Solid with no
  // joint between it and its parent -- the URDF importer's IMU emission
  // pattern) owns none: it is rigidly welded to its merge leader / nearest
  // bodied ancestor, whose body is the physically correct read (omega is
  // identical throughout the fold, and point-velocity reads pass the
  // sensor's own world position so the lever arm is right). Resolves via
  // nearestNewtonBodyIndex() -- the same walk welds and wrench reads anchor
  // on -- and returns NULL when the whole chain owns no Newton body.
  OmBodyHandle carrierBodyHandle() const;
  int recognitionColorSize() const;
  const OmRgb &recognitionColor(int index) const;
  // hidden field accessors
  const OmVector3 &linearVelocity() const { return mLinearVelocity->value(); }
  const OmVector3 &angularVelocity() const { return mAngularVelocity->value(); }

  // return the velocity relatively to a parent Solid (or absolute velocity if parentSolid is NULL)
  OmVector3 relativeLinearVelocity(const OmSolid *parentSolid = NULL) const;
  OmVector3 relativeAngularVelocity(const OmSolid *parentSolid = NULL) const;

  void setLinearVelocity(const double velocity[3]);
  void setAngularVelocity(const double velocity[3]);

  double mass() const;
  const double *inertiaMatrix() const;
  double globalMass() const { return mGlobalMass; }
  double globalVolume() const { return mGlobalVolume; }
  const OmVector3 &globalCenterOfMass() const { return mGlobalCenterOfMass; }
  // global center of mass: adds up the solid itself and all its descendants
  const OmVector3 &computedGlobalCenterOfMass() {
    updateGlobalCenterOfMass();
    return mGlobalCenterOfMass;
  }
  void updateCenterOfMass();  // update the center of mass according to the Physics node specification
  const OmVector3 &centerOfMass() const { return mCenterOfMass; }

  const QVector<OmVector3> &contactPoints(bool includeDescendants = false) const {
    return includeDescendants ? mGlobalListOfContactPoints : mListOfContactPoints;
  }
  const QVector<OmVector3> &computedContactPoints(bool includeDescendants = false);
  const QVector<const OmSolid *> &computedSolidPerContactPoints();
  // Per-contact penetration depth (ODE dContactGeom.depth), aligned 1:1 with
  // computedContactPoints(includeDescendants). Real engine-computed contact
  // magnitude. See physics-contact-impulse-api.md.
  const QVector<double> &computedContactPointDepths(bool includeDescendants = false);

  // Solid merger
  QPointer<OmSolidMerger> solidMerger() const;
  void setupSolidMerger();
  void setupSolidMergers();
  bool isSolidMerger() const;
  bool mergerIsSet() const { return mMergerIsSet; }

  // New joints
  void appendJointParent(OmBasicJoint *joint);
  void removeJointParent(OmBasicJoint *joint);

  // set up joints for special nodes:
  // - fixed joint between TouchSensor and parent body
  // - fixed joint between dynamic solid child and kinematic parent body (static environment)

  // dynamical state
  bool isDynamic() const { return !mIsKinematic; }
  bool isKinematic() const { return mIsKinematic; }

  // sleeping flags
  bool isSleeping() const;

  // awakening
  void awake();
  static void awakeSolids(OmGroup *group);

  void resetPhysics(bool recursive = true) override;
  // pause/resume physics computation on the current solid and its descandants
  void pausePhysics(bool resumeAutomatically = false) override;
  void resumePhysics() override;

  // handle artifical moves triggered by the user or a Supervisor
  void jerk(bool resetVelocities = true, bool rootJerk = true) override;
  void notifyChildJerk(OmPose *childNode);

  // physics setters
  void addForceAtPosition(const OmVector3 &force, const OmVector3 &position);
  void addTorque(const OmVector3 &torque);
  // W3.1: route a world-frame force-at-position + torque to this Solid's Newton body (if any). Returns true
  // when routed to Newton; false -> not Newton-backed, so the caller must warn or drop. See OmSolid.cpp.
  bool applyExternalForceNewton(const OmVector3 &force, const OmVector3 &worldPos,
                                const OmVector3 &torque) const;
  // W3.2: route a mid-step velocity set to this Solid's Newton body (angular=false -> linear, true ->
  // angular). Returns true when routed to Newton; false -> not routable YET, caller caches for replay.
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
  OmRobot *robot() const;
  bool isTopSolid() const { return (this == topSolid()); }
  virtual OmSolid *findSolid(const QString &name, const OmSolid *const exception = NULL);
  bool belongsToStaticBasis() const;  // Returns true if all solid ancestors have no physics

  // lists of OmSolid and OmBasicJoint children
  const QVector<OmSolid *> &solidChildren() const { return mSolidChildren; }
  const QVector<OmBasicJoint *> &jointChildren() const { return mJointChildren; }

  // ODE mass adjustments

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
  const OmPolygon &supportPolygon();

  // Immersion: menu-toggle state only, nothing is drawn (no Fluid node exists post-ODE)
  bool showCenterOfBuoyancyRepresentation(bool enabled);
  bool centerOfBuoyancyRepresentationEnabled() const { return mCenterOfBuoyancyRepresentationIsEnabled; }

  // Collecting and restoring info on descendants
  void collectSolidDescendantNames(QStringList &items, const OmSolid *const solidException = NULL) const;
  void collectHiddenKinematicParameters(OmHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                        int &counter) const override;

  // Threshold to handle mass round off errors after resize events
  static const double MASS_ZERO_THRESHOLD;

  OmBasicJoint *jointParent() const;

  // save previous position and orientation
  void savePreviousTransform();

  // print a warning if the current solid is moved in kinematics mode and it
  // has any dynamic Solid descendant node that won't follow it
  void printKinematicWarningIfNeeded();

  // Functions to compute unique name, find a Solid based on the unique name and resolve name clashes
  QString computeUniqueName() const;
  OmSolid *findDescendantSolidFromUniqueName(QStringList &names) const;
  void resolveNameClashIfNeeded(bool automaticallyChange, bool recursive, const QList<OmSolid *> &siblings,
                                QSet<const QString> *topSolidNameSet);
  static OmSolid *findSolidFromUniqueName(const QString &name);
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
  OmSolid(const QString &modelName, OmTokenizer *tokenizer);

  // physics accessors
  const QList<OmBasicJoint *> &jointParents() const { return mJointParents; }
  void setJointParents();

  // to-be-reimplemented in derived classes
  void updateName() override;


  // Renders the frame axes and the center of mass
  void applyVisibilityFlagsToWren(bool selected) override;
  void applyChangesToWren() override;

  // Solid merger, i.e. solid ancestor (possibly the solid itself) that owns the mass, body and dGeoms of this solid..
  virtual void setSolidMerger();
  // Non NULL only if this solid is dynamic and is not related to its dynamic parent by a joint
  QPointer<OmSolidMerger> mSolidMerger;


  // export
  bool exportNodeHeader(OmWriter &writer) const override;
  void exportNodeFooter(OmWriter &writer) const override;

protected slots:
  void updateTranslation() override;
  void updateRotation() override;
  virtual void updateIsLinearVelocityNull();
  virtual void updateIsAngularVelocityNull();

private:
  // Push the Solid's current mTranslation/mRotation into Newton's
  // body_q so a Supervisor field write (e.g. reset_episode()) actually
  // moves the Newton-tracked body. No-op for non-Newton Solids.
  void syncNewtonPoseFromFields();

  // OMNISIM_NEWTON_KINEMATIC: push THIS Solid's current world pose into its
  // Newton kinematic/static (mocap) body via setKinematicPose. Dirty-tracked
  // against the last pushed pose, so an idle body costs one compare per call.
  // No-op when the flag is off, the Solid owns no Newton body, or the body is
  // a dynamic registration.
  void pushNewtonKinematicPose();
  // Same, recursively over the joint-free AND jointed kinematic subtree --
  // a field write lands on ONE Solid but moves the world pose of every
  // descendant (nested PROTO colliders, deeper kinematic chain links), whose
  // own fields never change.
  void pushNewtonKinematicSubtreePose();

  OmSolid &operator=(const OmSolid &);  // non copyable
  void init();

  void exportUrdfShape(OmWriter &writer, const QString &geometry, const OmPose *pose, const OmVector3 &offset) const;

  // list of finalized solids
  static QList<const OmSolid *> cSolids;

  // user accessible fields
  OmSFString *mContactMaterial;
  OmSFString *mPhysicsBackend;
  // Solid.newtonClothCoupling: -1 / 0 / +1 -- hide this body from the cloth
  // solver / leave it to the runtime / show it. NULL on node types that do not
  // declare the field (the same guard physicsBackend needs), so every read
  // must be null-checked rather than dereferenced.
  OmSFInt *mNewtonClothCoupling;
  OmSFNode *mPhysics;
  // Solid.newtonFriction: per-Solid tangential friction. NEGATIVE = unset,
  // inheriting WorldInfo.newtonGroundMu. NULL on node types that do not declare
  // the field, so every read must be null-checked (same guard as newtonClothCoupling).
  // Solid.newtonGravityCompensation: 0..1 fraction of gravity cancelled on this
  // body. NULL on node types that do not declare the field, so null-check.
  OmSFDouble *mNewtonGravityCompensation;
  OmSFDouble *mNewtonFriction;
  // Torsional / rolling friction (MuJoCo geom_friction[1] and [2]). NEGATIVE =
  // unset. Only consulted at WorldInfo.newtonCondim >= 4 / >= 6 respectively.
  OmSFDouble *mNewtonFrictionTorsional;
  OmSFDouble *mNewtonFrictionRolling;
  OmSFDouble *mRadarCrossSection;
  OmMFColor *mRecognitionColors;

  // P3.2: index of this Solid's body in the OmNewtonBackend's Newton
  // model, or -1 when not registered (ode-backed or Newton unavailable).
  // Populated in postFinalize, consumed by postPhysicsStep for pose
  // readback.
  int mNewtonBodyIndex;

  // FIX 5 (t=0 setVelocity drop): a Supervisor setLinearVelocity /
  // setAngularVelocity processed BEFORE this Solid's Newton registration
  // (immediate messages run first in OmSimulationWorld::step) used to land on
  // the dead ODE body and be silently lost. The setters cache the request
  // here when setNewtonBodyVel() cannot route it; the dynamic-registration
  // site in flushPendingNewtonRegistrations replays it once the body exists.
  bool mPendingNewtonLinVelValid = false;
  bool mPendingNewtonAngVelValid = false;
  double mPendingNewtonLinVel[3] = {0.0, 0.0, 0.0};
  double mPendingNewtonAngVel[3] = {0.0, 0.0, 0.0};

  // §4.1 capability gate: cached articulation-Newton-capability (-1 unknown, 0 no, 1 yes), computed
  // once at the articulation root and shared by every body in it. mutable: filled by the const
  // articulationNewtonCapable() on first query.
  mutable int mNewtonArticulationCapable = -1;

  // Registration-flush generation (physics-step-cost-optimization-plan.md §3
  // item 2). flushPendingNewtonRegistrations() runs EVERY tick and, before
  // this gate, re-ran an ancestor walk with dynamic_casts for every Solid that
  // will never register (visual-only children, fixed children, ode-pinned
  // props -- the majority of a robot scene) for the life of the world.
  // Measured cost: 0.097 ms/tick on an 8-Husky world, 11% of the tick.
  //
  // The gate is a generation counter rather than a boolean so that "a pass
  // registered nothing" can be remembered against "the scene has not changed
  // since". Bumped by anything that can make a Solid newly registrable: a new
  // Solid finalizing (the /scene/spawn path goes through postFinalize), and a
  // runtime write to the fields registrability is decided from.
  static int cNewtonFlushGeneration;

public:
  // Invalidate the registration-flush gate. Call from any site that changes
  // whether some Solid could register with Newton.
  static void bumpNewtonFlushGeneration() { ++cNewtonFlushGeneration; }

private:
  // Tier 1b (physics-step-cost-optimization-plan.md): the last Newton world
  // xform this Solid's postPhysicsStep wrote into the scene graph. When the
  // solver returns a BITWISE-identical pose (a genuinely settled body), the
  // whole writeback -- frame projection, two field writes + signal emits,
  // matrix invalidation, two WREN transform sets -- is skipped. Exact
  // compare only: an epsilon skip could accumulate visible staleness.
  // Validity is dropped whenever the writeback is skipped for any other
  // reason (reset, re-registration), so a stale compare can never suppress
  // a real update.
  bool mLastNewtonXformValid = false;
  double mLastNewtonXform[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // P8.2: true when mNewtonBodyIndex refers to a STATIC (mass=0) Newton
  // body registered via addStaticBody (arena floor / walls / obstacles
  // that opted into Newton but have no Physics node). Static bodies are
  // pinned by the solver and never integrate, so they skip the per-step
  // pose writeback AND the syncNewtonPoseFromFields supervisor bridge
  // (in particular resetJointsToDefaults, which on a root Solid would
  // otherwise clobber the shared articulation every tick).
  bool mNewtonBodyIsStatic;

  // OMNISIM_NEWTON_KINEMATIC: true when mNewtonBodyIndex refers to a
  // KINEMATIC (mocap) Newton body registered via addKinematicBody -- a
  // physics-less joint endpoint (or its nested collider) whose pose the
  // ENGINE animates and pushes through setKinematicPose. Like statics it
  // skips the per-step pose writeback and never takes resetBodyPose /
  // resetJointsToDefaults (mNewtonBodyIsStatic is set alongside for those
  // protections); unlike plain statics its pose pushes are expected, not
  // exceptional.
  bool mNewtonBodyIsKinematic;
  // Last pose pushed through setKinematicPose ([x,y,z, qx,qy,qz,qw]), valid
  // when mNewtonKinPoseValid -- the dirty filter that keeps the per-step
  // prePhysicsStep refresh free for bodies that did not move.
  bool mNewtonKinPoseValid;
  double mNewtonKinPushedPose[7];

  // hidden fields
  OmSFVector3 *mLinearVelocity;
  OmSFVector3 *mAngularVelocity;

  bool mIsLinearVelocityNull;
  bool mIsAngularVelocityNull;

  // Clone
  OmNode *clone() const override { return new OmSolid(*this); }

  // Selection
  bool mSelected;

  bool mUpdatedInStep;       // used to update Transform coordinated to setup ray collisions (based on pre-physics step values)
  bool mResetPhysicsInStep;  // used to completely reset physics when the solid is also moved in the same step
  void setGeomAndBodyPositions();
  void applyPhysicsTransform();
  void resetJoints();  // reset joint to any linked solid to this one or to one of its descendants
  void setBodiesAndJointsToParents();
  void setJointChildrenWithReferencedEndpoint();
  bool resetJointPositions(bool allParents = false);
  void handleJerk() override;

  QVector<OmPose *> mMovedChildren;
  void childrenJerk();

  void resetSingleSolidPhysics();

  // set position and orientation from physics integration (in contrast with supervisor or user setup)
  void inline setTransformFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle);

  // Mass and density
  void addMassFromInsertedNode(OmBaseNode *node);
  bool checkMassAndDensity() const;
  double mAverageDensity;
  bool mUseInertiaMatrix;  // indicates that the OmSolid uses the latest valid inertia matrix field for ODE physics computation
  OmVector3 mCenterOfMass;
  // ODE-free mirror of mOdeMass (geometry-derived mass properties about the
  // Solid frame origin), computed by OmSolidUtilities::addInertia -- the
  // Newton feed that survives src/ode deletion. Valid only when
  // mNativeInertiaValid (i.e. after createOdeMass ran the geometry branch).
  OmInertia mNativeInertia;
  bool mNativeInertiaValid;

  // ODE mass adjustments
  void createOdeMass(bool reset = true);
  // adjust the mass of the OmSolid after insertion or deletion in the boundingObject
  void adjustOdeMass(bool mergeMass = true);
  // subtracts the mass of a deleted node in the boundingObject (called from OmGeometry)
  void setDefaultMassSettings(bool applyCenterOfMassTranslation, bool warning = true);

  // Global center of mass variables
  OmVector3 mGlobalCenterOfMass;  // expressed in absolute (world) coordinates; if the total mass is zero and no CoM is
                                  // specified, it defaults to the Solid frame's center
  bool mGlobalCenterOfMassRepresentationIsEnabled;
  double mGlobalVolume;  // volume of the solid and all its descendants
  double mGlobalMass;    // mass of the solid and all its descendants

  // Immersion
  bool mCenterOfBuoyancyRepresentationIsEnabled;

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
  QVector<OmVector3> mListOfContactPoints;
  QVector<OmVector3> mGlobalListOfContactPoints;  // includes contacts of Solid descendants
  QVector<double> mListOfContactPointDepths;        // 1:1 with mListOfContactPoints
  QVector<double> mGlobalListOfContactPointDepths;  // 1:1 with mGlobalListOfContactPoints
  // defines the colliding Solid for each for the contact point in mGlobalListOfContactPoints
  QVector<const OmSolid *> mSolidPerContactPoints;
  bool mHasExtractedContactPoints;
  void extractContactPoints();  // populates both local and global lists

  // Support polygon variables

  // used to index the gravity basis vector when projecting contact points on a plan orthogonal to the gravity direction
  enum { X, Y, Z };
  bool mSupportPolygonRepresentation;  // D1.4: the WREN visual is gone; kept as the "created" flag
  void deleteSupportPolygonRepresentation();
  mutable OmPolygon mSupportPolygon;
  mutable bool mSupportPolygonNeedsUpdate;
  double mY;  // minimum on the y-coordinate over all contact points
  bool mSupportPolygonRepresentationIsEnabled;

  // Relatives
  mutable bool mHasSearchedRobot;
  QVector<OmSolid *> mSolidChildren;  // either direct children, solid children of a OmGroup appearing in the children list,
                                      // solid endPoints of joint children or solid endPoint of a slot
  QVector<OmBasicJoint *> mJointChildren;
  QVector<OmPropeller *> mPropellerChildren;
  QList<OmBasicJoint *> mJointParents;  //  direct joint parent and joints whose endPoint is this solid
  mutable OmRobot *mRobot;

  // Update of children lists
  void collectSolidChildren(const OmGroup *group, bool connectSignals, QVector<OmSolid *> &solidChildren,
                            QVector<OmBasicJoint *> &jointChildren, QVector<OmPropeller *> &propellerChildren);
  void updateDynamicSolidDescendantFlag();

  void setOdeInertiaMatrix();

  // WREN objects


  OmHiddenKinematicParameters::HiddenKinematicParameters *mOriginalHiddenKinematicParameters;
  bool applyHiddenKinematicParameters(const OmHiddenKinematicParameters::HiddenKinematicParameters *hkp, bool backupPrevious);
  bool restoreHiddenKinematicParameters(const OmHiddenKinematicParameters::HiddenKinematicParametersMap &map,
                                        int &counter) override;
  bool resetHiddenKinematicParameters() override;

  void setGeomMatter(OmBaseNode *node = NULL) override;

  bool mNameClashResolved;

private slots:
  void updateChildrenAfterJointEndPointChange(OmBaseNode *node);
  void updatePhysics();
  void updateRadarCrossSection();
  void updateRecognitionColors();
  void updateOdeMass();
  void applyToOdeMass();
  void updateOdeInertiaMatrix();
  void updateOdeCenterOfMass();
  void updateOdeDamping();
  void createOdeGeomFromInsertedGroupItem(OmBaseNode *node) override;
  void removeBoundingGeometry() override;
  void updateTopSolidGlobalMass() const;
  // connected to the physicsStepEnded() signal in computedContactPoints() and disconnected when called
  void resetContactPoints();
  void resetContactPointsAndSupportPolygon();  // connected to the physicsStepEnded() signal
  void refreshPhysicsRepresentation();             // redraws CoM and support polygon after physics properties change
  void refreshSupportPolygonRepresentation();      // redraws after each physics step
  void refreshGlobalCenterOfMassRepresentation();  // redraws the global CoM after each physics step
  void onSimulationModeChanged();                  // deletes the support polygon representation if need be
  void displayWarning();
};

void inline OmSolid::setTransformFromOde(double tx, double ty, double tz, double rx, double ry, double rz, double angle) {
  OmPose::setTranslationAndRotationFromOde(tx, ty, tz, rx, ry, rz, angle);
  OmPose::updateTranslationAndRotation();
}

#endif
