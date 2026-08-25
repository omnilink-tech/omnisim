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

#include "OmSolidUtilities.hpp"

#include "OmBox.hpp"
#include "OmCapsule.hpp"
#include "OmCylinder.hpp"
#include "OmElevationGrid.hpp"
#include "OmIndexedFaceSet.hpp"
#include "OmInertia.hpp"
#include "OmMatrix3.hpp"
#include "OmMesh.hpp"
#include "OmNodeUtilities.hpp"
#include "OmPose.hpp"
#include "OmShape.hpp"
#include "OmSolid.hpp"
#include "OmSphere.hpp"
#include "OmTriangleMesh.hpp"

#include "OmOdeTypes.hpp"  // opaque handle typedefs only

#include <assert.h>

// The ODE interface does not allow to simply cast a dGeomID in a dSpaceID although dSpace inherits from dGeom; this forces the
// use of a reinterpret_cast Note: this is possible because the internal stucture dxSpace inherits of dxGeom
dSpaceID OmSolidUtilities::dynamicCastInSpaceID(dGeomID g) {
  (void)g;
  return NULL;  // ODE is gone: no collision spaces exist
}

void OmSolidUtilities::subtractInertiaMatrix(double *I, const double *J) {
  I[0] -= J[0];
  I[1] -= J[1];
  I[2] -= J[2];
  I[5] -= J[5];
  I[6] -= J[6];
  I[10] -= J[10];
  I[4] = I[1];
  I[8] = I[2];
  I[9] = I[6];
}

void OmSolidUtilities::setDefaultMass(dMass *m) {
  (void)m;  // ODE is gone: dMass is opaque; OmInertia::setDefault is the native equivalent
}

// The mass is supposed to be homogeneously spread over the body boundingObject
void OmSolidUtilities::addMass(dMass *const mass, OmNode *const node, double density, bool warning) {
  // ODE is gone: the dMass integrator is retired -- addInertia (below,
  // parity-pinned) is the only mass-property source. NOTE (report item): the
  // per-geometry user warnings this pass emitted (invalid radius/dimensions,
  // non-volume trimeshes) are not emitted any more; addInertia is
  // warning-free by design.
  (void)mass;
  (void)node;
  (void)density;
  (void)warning;
}

// ODE-free mirror of addMass over OmInertia: identical traversal, identical
// validity gates (including the bug-compatible "failed trimesh geom is warned
// about but contributes nothing" branch), no warnings (the dMass pass above
// already emitted them). Parity-tested against the dMass pipeline by
// tests/test_newton_native_inertia_parity.py while ODE still ships.
void OmSolidUtilities::addInertia(OmInertia *const inertia, OmNode *const node, double density) {
  if (!node)
    return;

  const OmShape *const shape = dynamic_cast<OmShape *>(node);
  if (shape) {
    if (shape->geometry() == NULL)
      return;
    addInertia(inertia, shape->geometry(), density);
    return;
  }

  OmInertia m;

  // The OmPose case must come before the OmGroup case
  const OmPose *const pose = dynamic_cast<OmPose *>(node);
  if (pose) {
    OmGeometry *g = pose->geometry();
    // no ODE geom exists any more: the geometry's presence is the gate
    // (the ON-side odeGeom() test only encodes "the geometry was buildable")
    if (g)
      addInertia(&m, g, density);
    else
      // invalid geometry
      return;

    // Rotate then translate into the parent frame, exactly as addMass does.
    // dRFromAxisAndAngle normalizes the axis (OmMatrix3::fromAxisAngle does
    // not), so normalize here; a zero axis means identity, as in ODE.
    const OmRotation &r = pose->rotation();
    OmVector3 axis(r.x(), r.y(), r.z());
    const double l = axis.length();
    if (l > 0.0) {
      axis /= l;
      const OmMatrix3 R(axis, r.angle());
      m.rotate(R);
    }
    const OmVector3 t = pose->translation();
    m.translate(t.x(), t.y(), t.z());
    inertia->add(m);
    return;
  }

  // The case of a OmGroup which is NOT a OmPose
  const OmGroup *const group = dynamic_cast<OmGroup *>(node);
  if (group) {
    OmMFNode::Iterator it(group->children());
    while (it.hasNext())
      addInertia(&m, it.next(), density);
    if (m.mass() > 0.0)
      inertia->add(m);
    return;
  }

  OmSphere *const sphere = dynamic_cast<OmSphere *>(node);
  if (sphere) {
    const double radius = sphere->scaledRadius();
    if (radius <= 0.0)
      m.setDefault();
    else
      m.setSphere(density, radius);
    inertia->add(m);
    return;
  }

  OmCylinder *const cylinder = dynamic_cast<OmCylinder *>(node);
  if (cylinder) {
    const double radius = cylinder->scaledRadius();
    const double height = cylinder->scaledHeight();
    if (radius <= 0.0 || height <= 0.0)
      m.setDefault();
    else
      m.setCylinderZ(density, radius, height);
    inertia->add(m);
    return;
  }

  OmCapsule *const capsule = dynamic_cast<OmCapsule *>(node);
  if (capsule) {
    const double radius = capsule->scaledRadius();
    const double height = capsule->scaledHeight();
    if (radius <= 0.0 || height <= 0.0)
      m.setDefault();
    else
      m.setCapsuleZ(density, radius, height);
    inertia->add(m);
    return;
  }

  OmBox *const box = dynamic_cast<OmBox *>(node);
  if (box) {
    const OmVector3 size = box->scaledSize();
    if (size.x() <= 0.0 || size.y() <= 0.0 || size.z() <= 0.0)
      m.setDefault();
    else
      m.setBox(density, size.x(), size.y(), size.z());
    inertia->add(m);
    return;
  }

  OmTriangleMeshGeometry *const tmg = dynamic_cast<OmTriangleMeshGeometry *>(node);
  if (tmg) {
    // Bug-compatible with addMass: when the ODE trimesh geom failed to build,
    // the default mass is SET but never ADDED -- the branch contributes
    // nothing to the total. (Post-ODE-deletion this gate becomes a
    // OmTriangleMesh validity check; the outcome is identical.)
    const OmTriangleMesh *const tm = tmg->triangleMesh();
    const OmVector3 s = tmg->absoluteScale();
    if (tm == NULL || !m.setTrimesh(density, tm, s.x(), s.y(), s.z()))
      m.setDefault();
    inertia->add(m);
    return;
  }

  // ElevationGrid (and anything unhandled) contributes nothing, as in addMass.
}

static bool isSolidForCollect(OmBaseNode *n) {
  return dynamic_cast<OmSolid *>(n) != nullptr;
}

void OmSolidUtilities::collectNewtonBodies(OmNode *const root, QVector<int> &out) {
  if (root == NULL)
    return;
  OmSolid *const rootSolid = dynamic_cast<OmSolid *>(root);
  if (rootSolid != nullptr && rootSolid->newtonBodyIndex() >= 0)
    out.append(rootSolid->newtonBodyIndex());
  // findDescendantNodesOfType descends OmGroup children, OmSlot endPoints AND
  // OmBasicJoint endPoints (with SolidReference loop guards) -- the complete
  // walk the ray consumers need. recursive=true so nested Solids are visited.
  const QList<OmNode *> descendants =
    OmNodeUtilities::findDescendantNodesOfType(const_cast<OmNode *>(root), isSolidForCollect, true);
  foreach (OmNode *const n, descendants) {
    OmSolid *const s = static_cast<OmSolid *>(n);
    if (s->newtonBodyIndex() >= 0)
      out.append(s->newtonBodyIndex());
  }
}

// Recursive method that checks the validity of a boundingObject: USED FOR DEBUG ONLY
// (Note: such check is already done during the contruction process which issues appropriate warnings)
bool OmSolidUtilities::checkBoundingObject(OmNode *const node) {
  if (node == NULL)
    return false;

  const OmPose *const pose = dynamic_cast<OmPose *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (pose) {
    OmNode *child = pose->child(0);
    if (child == NULL) {
      node->parsingWarn(
        QObject::tr("Invalid 'boundingObject' (a Pose has no 'geometry'): the inertia matrix cannot be calculated."));
      return false;
    }

    const OmGeometry *const g = dynamic_cast<OmGeometry *>(child);
    // cppcheck-suppress knownConditionTrueFalse
    if (g)
      return true;

    const OmShape *const shape = dynamic_cast<OmShape *>(child);
    if (shape == NULL || shape->geometry() == NULL) {
      node->parsingWarn(QObject::tr("Invalid 'boundingObject' (a Pose, or a Shape within a Pose, has no 'geometry'): the "
                                    "inertia matrix cannot be calculated."));
      return false;
    }
  }

  const OmShape *const shape = dynamic_cast<OmShape *>(node);
  if (shape && shape->geometry() == NULL) {
    node->parsingWarn(QObject::tr(
      "Invalid 'boundingObject' (a Shape's 'geometry' field is not set): the inertia matrix cannot be calculated."));
    return false;
  }

  const OmGroup *const group = dynamic_cast<OmGroup *>(node);
  if (group) {
    const OmMFNode &children = group->children();
    const int size = children.size();
    if (size == 0) {
      node->parsingWarn(
        QObject::tr("Invalid 'boundingObject' (Group with no child set): the inertia matrix cannot be calculated."));
      return false;
    }
    for (int i = 0; i < size; ++i) {
      if (!checkBoundingObject(children.item(i)))
        return false;
    }
  }

  return true;
}

OmGeometry *OmSolidUtilities::geometry(OmNode *const node) {
  if (!node)
    return NULL;

  OmGeometry *const geometry = dynamic_cast<OmGeometry *>(node);
  // cppcheck-suppress knownConditionTrueFalse
  if (geometry)
    return geometry;

  const OmShape *const shape = dynamic_cast<OmShape *>(node);
  if (shape) {
    if (shape->geometry() == NULL)
      shape->parsingInfo(QObject::tr("Please specify 'geometry' (of Shape placed in 'boundingObject')."));

    return shape->geometry();
  }

  node->parsingWarn(QObject::tr("Ignoring illegal \"%1\" node in 'boundingObject'.").arg(node->usefulName()));
  return NULL;
}

bool OmSolidUtilities::isPermanentlyKinematic(const OmNode *node) {
  const OmBaseNode *const p = node ? static_cast<OmBaseNode *>(node->parentNode()) : NULL;
  return p && p->nodeType() == WB_NODE_PROPELLER;
}
