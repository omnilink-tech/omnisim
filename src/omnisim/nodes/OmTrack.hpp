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

//
//  OmTrack.hpp
//

// Implemented node class representing a generic robot tank track

#ifndef OM_TRACK_HPP
#define OM_TRACK_HPP

#include "OmSolid.hpp"

#include <vector>

class OmBaseNode;
class OmBrake;
class OmLinearMotor;
class OmLogicalDevice;
class OmPositionSensor;
class OmSFInt;
class OmSFVector2;
class OmShape;
class OmTextureTransform;
class OmTrackWheel;

class OmTrack : public OmSolid {
  Q_OBJECT
public:
  explicit OmTrack(OmTokenizer *tokenizer = NULL);
  OmTrack(const OmTrack &other);
  explicit OmTrack(const OmNode &other);
  virtual ~OmTrack() override;

  // reimplemented public functions
  int nodeType() const override { return WB_NODE_TRACK; }
  void preFinalize() override;
  void createWrenObjects() override;
  void postFinalize() override;
  void prePhysicsStep(double ms) override;
  void exportNodeSubNodes(OmWriter &writer) const override;
  void setMatrixNeedUpdate() override;
  void reset(const QString &id) override;
  void save(const QString &id) override;

  double contactSurfaceVelocity() const { return mSurfaceVelocity; }
  double position() const { return mMotorPosition; }

  QVector<OmLogicalDevice *> devices() const;
  OmLinearMotor *motor() const;
  OmPositionSensor *positionSensor() const;
  OmBrake *brake() const;

  void animateMesh();

  // ---- P2: the wgpu DRAW SOURCE (docs/developer/wren-deletion-runbook.md) ------------
  //
  // ⚠ A Track is NOT a deformable and must not be ported like one. Its belt is INSTANCED
  // geometry: N copies of a real OmGeometry, and animateMesh() generates NO vertices at
  // all -- it advances mBeltPositions[i]. So the wgpu draw is N extra MODEL MATRICES
  // reusing a mesh the ordinary Shape collect already knows how to acquire, which is the
  // same shape P9's granular port used and nothing like the Cloth vertex stream.
  //
  // ⚠ AND THE ORIGINAL GEOMETRY MUST NOT ALSO BE DRAWN. The belt copies replace it. The
  // wgpu side gets this for free -- `animatedGeometry` is an SFNode field, not a child,
  // so collectWorldDraws' children walk never reaches it -- but that is a property worth
  // knowing before anyone "fixes" the walk to cover SFNode fields.
  //
  // THE REGISTRY: same reasoning as OmMuscle's. The frame listener's track list is a
  // PENDING-REBUILD queue that frameStarted() clears, so it cannot answer "does this world
  // have a Track?".
  static bool anyTracks();
  static const std::vector<OmTrack *> &liveTracks();

  // One entry per (belt element x animated shape) -- exactly the renderables WREN builds.
  struct WgpuBeltDraw {
    OmShape *shape = NULL;        // for the appearance / material
    OmGeometry *geometry = NULL;  // for the mesh + its local scale
    bool castShadows = false;
    OmMatrix4 world;  // trackWorld * beltTRS * geomMatrix -- the mesh's own scale is NOT
                      // folded in; the caller applies it, as the Shape path does.
  };
  // Appends this step's belt placements. Empty (and cheap) for a Track with no
  // `animatedGeometry`, which is most of them -- ConveyorBelt.proto animates its TEXTURE
  // instead and produces nothing here.
  void wgpuBeltDraws(std::vector<WgpuBeltDraw> &out) const;

  // Does this Track scroll its belt TEXTURE (a non-null TextureTransform and a non-zero
  // `textureAnimation`)? The UV affine a draw carries is filled at COLLECT time, and the
  // wgpu main view caches its draw list and refreshes only the model matrices -- so a
  // scrolling texture is frozen unless something tells the cache to re-read it. Static so
  // a world with no Track pays one empty-test.
  static bool anyTextureAnimation();

  // Telemetry only (OMNISIM_WGPU_REPORT): the leading belt element of the first live Track
  // that has one, in the Track's OWN path coordinates. Reported alongside the draw's
  // world-space model translation precisely because those two can disagree: a carrier that
  // is itself moving makes the world number change while the belt is frozen, and a carrier
  // sinking makes it change without bound. This one moves if and only if the BELT advanced,
  // and it is bounded by the closed path. False when no Track has a belt.
  static bool wgpuFirstBeltProbe(double &x, double &y, double &rotation);

protected slots:
  void updateChildren() override;

private:
  OmTrack &operator=(const OmTrack &);  // non copyable
  OmNode *clone() const override { return new OmTrack(*this); }
  void init();

  OmMFNode *mDeviceField;
  OmSFVector2 *mTextureAnimationField;
  OmSFNode *mGeometryField;
  OmSFInt *mGeometriesCountField;

  // internal fields
  OmLinearMotor *mLinearMotor;
  OmBrake *mBrake;
  double mMotorPosition;
  double mSurfaceVelocity;
  dBodyID mBodyID;

  // wheels
  QVector<OmTrackWheel *> mWheelsList;
  void clearWheelsList();

  // texture animation
  OmShape *mShape;
  OmTextureTransform *mTextureTransform;
  QMap<QString, OmVector2> mSavedTextureTransformTranslations;

  // geometries animation
  struct PathSegment {
    PathSegment(const OmVector2 &pointA, const OmVector2 &pointB, double angle, double r, const OmVector2 &c,
                const OmVector2 &direction) :
      startPoint(pointA),
      endPoint(pointB),
      initialRotation(angle),
      radius(r),
      center(c),
      increment(direction) {}
    PathSegment(const OmVector2 &pointA, const OmVector2 &pointB, double angle, const OmVector2 &normalizedDirection) :
      startPoint(pointA),
      endPoint(pointB),
      initialRotation(angle),
      radius(-1),
      center(OmVector2()),
      increment(normalizedDirection) {}
    PathSegment() : initialRotation(M_PI), radius(0.0) {}

    OmVector2 startPoint;
    OmVector2 endPoint;
    double initialRotation;
    double radius;        // round path only
    OmVector2 center;     // round path only
    OmVector2 increment;  // straight path only
  };

  struct BeltPosition {
    BeltPosition(const OmVector2 &pos, double angle, int index) : position(pos), rotation(angle), segmentIndex(index) {}
    BeltPosition() : rotation(0.0), segmentIndex(0) {}

    OmVector2 position;
    double rotation;
    int segmentIndex;
  };

  double mPathLength;
  double mPathStepSize;
  QVector<PathSegment> mPathList;
  BeltPosition mFirstGeometryPosition;
  void computeBeltPath();
  BeltPosition computeNextGeometryPosition(BeltPosition current, double stepSize, bool segmentChanged = false) const;

  // animated mesh
  struct AnimatedObject {
    OmGeometry *geometry;
    // P2: the owning Shape, for the wgpu appearance -- wgpu reads the
    // PBRAppearance / Appearance off the node, so keep the node.
    OmShape *shape;
    bool castShadows;
  };

  double mAnimationStepSize;
  QList<AnimatedObject *> mAnimatedObjectList;
  QList<BeltPosition> mBeltPositions;

  void initAnimatedGeometriesBeltPosition();
  void clearAnimatedGeometries();
  bool findAndConnectAnimatedGeometries(bool connectSignals, QList<OmShape *> *shapeList);
  void exportAnimatedGeometriesMesh(OmWriter &writer) const;

private slots:
  void addDevice(int index);
  void updateDevices();
  void updateShapeNode();
  void updateTextureTransform();
  void updateWheelsList();
  void updateAnimatedGeometriesAfterFinalization(OmBaseNode *node);
  void updateAnimatedGeometries();
  void updateAnimatedGeometriesPath();
  void updateTextureAnimation();
};

#endif
