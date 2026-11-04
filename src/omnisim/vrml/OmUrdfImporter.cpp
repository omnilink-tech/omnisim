// Copyright 2026 OmniLink
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

#include "OmUrdfImporter.hpp"

#include "OmLog.hpp"

#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QRegularExpression>
#include <QtXml/QDomDocument>
#include <QtCore/QFileInfo>
#include <QtCore/QHash>
#include <QtCore/QList>
#include <QtCore/QSet>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QTextStream>

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

// ---------------------------------------------------------------------------
// URDF data model
// ---------------------------------------------------------------------------

struct UrdfOrigin {
  double x = 0.0, y = 0.0, z = 0.0;
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

struct UrdfMaterial {
  QString name;
  double r = 0.5, g = 0.5, b = 0.5, a = 1.0;
  // Optional PBR from a non-standard <omnisim/> child of <material>. URDF can say
  // what colour a part is and nothing else, so every imported robot rendered at a
  // flat roughness 0.5 / metalness 0 and read as plastic. Negative = unset, which
  // keeps those constants byte-for-byte for any URDF that does not opt in.
  double roughness = -1.0, metalness = -1.0;
  // Standard URDF <texture filename="..."/>, resolved like a mesh path; empty =
  // none. Emitted as baseColorMap, and colour alpha < 1 as transparency
  // (2026-09-25): both were parsed-and-dropped, so a textured or see-through
  // part (a carbon frame, a rotor blur disc) needed a hand-patched robot.
  QString texturePath;
};

struct UrdfGeometry {
  QString kind;  // "box", "cylinder", "sphere", "mesh", "unsupported"
  double sx = 0.0, sy = 0.0, sz = 0.0;
  double radius = 0.0;
  double length = 0.0;
  QString detail;          // for mesh: original filename as written in the URDF (used in warnings)
  QString meshPath;        // for mesh: resolved absolute filesystem path, empty if unresolvable
  double meshScaleX = 1.0, meshScaleY = 1.0, meshScaleZ = 1.0;
};

struct UrdfVisual {
  UrdfOrigin origin;
  UrdfGeometry geometry;
  UrdfMaterial material;
  bool hasMaterial = false;
};

struct UrdfCollision {
  UrdfOrigin origin;
  UrdfGeometry geometry;
};

struct UrdfInertial {
  UrdfOrigin origin;
  double mass = 0.0;
  bool present = false;
  // Full inertia tensor from <inertia ixx ixy ixz iyy iyz izz/>. Emitted into
  // OmniSim' Physics.inertiaMatrix (2-line MFVector3: [Ixx Iyy Izz], [Ixy Ixz Iyz])
  // when hasInertiaMatrix is true and the tensor is positive definite.
  double ixx = 0.0, ixy = 0.0, ixz = 0.0;
  double iyy = 0.0, iyz = 0.0, izz = 0.0;
  bool hasInertiaMatrix = false;
  // An <inertia> WAS declared but is not positive definite (an all-zero tensor
  // included), so it is not passed through. The values, for the warning
  // emitLinkPhysics writes once it knows what the fallback kept.
  bool declaredNotPositiveDefinite = false;
  bool declaredNonZeroDiagonal = false;
  QString declaredTensorText;
};

// Webots' updateMode requires the inertia tensor to be positive definite or it
// rejects the custom matrix and falls back to bounding-object derivation. A
// quick analytical 3x3 PD check: all leading principal minors positive.
bool inertiaIsPositiveDefinite(double ixx, double ixy, double ixz,
                               double iyy, double iyz, double izz) {
  if (ixx <= 0.0)
    return false;
  const double m2 = ixx * iyy - ixy * ixy;
  if (m2 <= 0.0)
    return false;
  const double det = ixx * (iyy * izz - iyz * iyz)
                   - ixy * (ixy * izz - iyz * ixz)
                   + ixz * (ixy * iyz - iyy * ixz);
  return det > 0.0;
}

// ---------------------------------------------------------------------------
// Sensor data model
// ---------------------------------------------------------------------------
//
// URDF has no native sensor schema, so we look at Gazebo extension blocks:
//
//   <gazebo reference="imu_link">
//     <sensor name="imu" type="imu">
//       <update_rate>100</update_rate>
//     </sensor>
//   </gazebo>
//
// and the older plugin-only form used by Clearpath URDFs (Jackal):
//
//   <gazebo>
//     <plugin filename="libhector_gazebo_ros_imu.so" name="imu_controller">
//       <bodyName>imu_link</bodyName>
//       <updateRate>50.0</updateRate>
//     </plugin>
//   </gazebo>
//
// The legacy plugin form is mapped onto the same UrdfSensor records so the
// downstream emission code is plugin-agnostic.

enum class UrdfSensorKind {
  Imu,    // -> Accelerometer + Gyro + InertialUnit
  Gps,    // -> GPS
  Camera, // -> Camera (RGB)
  Lidar,  // -> Lidar (planar) or RangeFinder
};

struct UrdfSensor {
  UrdfSensorKind kind = UrdfSensorKind::Imu;
  QString name;       // base name; per-device suffixes (_accel, _gyro, _orientation) added at emit time
  QString linkName;   // link this sensor is attached to (via <gazebo reference="...">)
  double updateRate = 100.0;
  // Camera/Lidar specifics, populated only when relevant
  int width = 320, height = 240;
  double horizontalFov = 1.047;  // ~60 deg default
  double minRange = 0.1, maxRange = 30.0;
  int lidarHorizontalResolution = 360;
  // Multi-layer lidars (Velodyne, Sick LD-MRS): populated from
  // <scan><vertical>...</vertical></scan>. Default to single-layer.
  int lidarNumberOfLayers = 1;
  double lidarVerticalFov = 0.0;
  // Resolved root-relative translation, set when sensors are relocated to
  // emit at Robot root via fixed-joint-chain walking (see emitRobot).
  double translationX = 0.0, translationY = 0.0, translationZ = 0.0;
};

struct UrdfLink {
  QString name;
  QList<UrdfVisual> visuals;
  QList<UrdfCollision> collisions;
  UrdfInertial inertial;
  double gazeboMu = -1.0;  // <gazebo reference="LINK"> <mu1>/<mu2> (the smaller); negative = unset
};

struct UrdfJoint {
  QString name;
  QString type;  // "fixed", "revolute", "continuous", "prismatic"
  QString parent;
  QString child;
  UrdfOrigin origin;
  double ax = 1.0, ay = 0.0, az = 0.0;
  double lower = 0.0, upper = 0.0;
  bool hasLower = false, hasUpper = false;
  double velocity = 10.0;
  bool hasVelocity = false;
  double effort = 0.0;
  bool hasEffort = false;
  double damping = 0.0;
  bool hasDamping = false;
  double friction = 0.0;
  bool hasFriction = false;
  double rest = 0.0;     // OmniSim extension: <rest>X</rest> child of <joint>
  bool hasRest = false;  // overrides the default initial joint position
  // <mimic joint="L" multiplier="m" offset="o">: q = o + m * q_L (2026-10-06).
  QString mimicJoint;
  double mimicMultiplier = 1.0;
  double mimicOffset = 0.0;
};

struct UrdfRobot {
  QString name;
  QHash<QString, UrdfMaterial> materials;
  QHash<QString, UrdfLink> links;
  QStringList linkOrder;
  QList<UrdfJoint> joints;
  QList<UrdfSensor> sensors;
};

// ---------------------------------------------------------------------------
// Parsing helpers
// ---------------------------------------------------------------------------

QList<double> parseDoubles(const QString &text, int expected) {
  QList<double> result;
  if (text.isEmpty())
    return result;
  const QStringList parts = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
  if (parts.size() != expected)
    return result;
  for (const QString &p : parts)
    result.append(p.toDouble());
  return result;
}

bool urdfDebugEnabled() {
  const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_DEBUG")).trimmed().toLower();
  return !value.isEmpty() && value != "0" && value != "false" && value != "off";
}

// URDF inertia tensors USED to be imported only under
// OMNISIM_URDF_USE_INERTIA=1. That default-OFF was an ODE-era workaround: ODE's
// `dMassSetParameters` crashed on small-mass child links (e.g. Jackal wheels at
// mass=0.477 with 0.0013/0.0024/0.0013 inertia), so the importer dropped every
// <inertia> tensor and let the bounding object supply one instead.
//
// ⚠️ WHAT THAT COST, measured 2026-09-10 on a two-link cart-pole URDF: with the
// flag unset the declared tensor NEVER reached the solver. The Newton runtime's
// `add_body` sees ixx=iyy=izz=0 and substitutes its Husky-tuned preset
// (0.0094m, 0.0167m, 0.0094m under 10 kg), so a pole declaring 0.0667 kg·m²
// about its COM was integrated at 0.00334 — and multiplying the URDF tensor by
// 4.5× produced a BYTE-IDENTICAL mjModel. The tensor was not merely ignored, it
// was masked by a number with no relation to the robot.
//
// ODE was deleted (src/ode is gone; `dMassSetParameters` no longer exists), so
// the reason for the OFF default died with it. Newton/MuJoCo consumes the tensor
// directly. Default is now ON; `OMNISIM_URDF_USE_INERTIA=0` restores the
// bounding-object-derived import for a bisect. Value-parsed, as every boolean
// hatch in this engine must be.
bool urdfUseInertiaEnabled() {
  const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_USE_INERTIA")).trimmed().toLower();
  if (value.isEmpty())
    return true;
  return value != "0" && value != "false" && value != "off" && value != "no";
}

// A declared tensor the importer REJECTS (not positive definite -- an all-zero
// one included --, impossible, or vanishingly small) used to take the link's
// declared centre of mass down with it. The bounding-object
// fallback emitted mass + centerOfMass but no inertiaMatrix, and the Newton
// registration passes a COM only alongside an explicit tensor -- so the solver
// stepped the link with its COM at the link origin. Found 2026-10-20 on AgiBot
// G2's body_link4 (declared COM (-0.071, 0, 0.030), stepped at (0, 0, 0)). The
// fallback Physics is now marked DEF URDF_INERTIA_FALLBACK and always carries the
// declared COM; OmSolid's Newton registration keys on the marker, keeps the mass
// and COM, and takes ONLY the rotational inertia from the bounding object, about
// that COM. OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM=0 (value-parsed) restores the
// unmarked block and the COM at the link origin.
bool urdfInertiaFallbackKeepsCom() {
  const QString value =
    QString::fromUtf8(qgetenv("OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM")).trimmed().toLower();
  if (value.isEmpty())
    return true;
  return value != "0" && value != "false" && value != "off" && value != "no";
}

// URDF <inertial><origin rpy="..."/> orients the frame the <inertia> tensor is
// written in; Physics.inertiaMatrix is about the centre of mass but in the
// Solid's (= the link's) own axes. Until 2026-10-15 the importer used only the
// origin's xyz (centerOfMass) and dropped the rpy, so a tensor given in a
// rotated inertial frame reached the solver UNROTATED: mass and COM right, the
// rotational inertia about every axis wrong (spot_description's hips and lower
// legs had their principal moments on the wrong axes. PyBullet and Gazebo rotate;
// plain MuJoCo 3.11's URDF import drops it too). parseLink now re-expresses the tensor in the link
// frame, I_link = R I R^T with R = Rz(yaw) Ry(pitch) Rx(roll), and clears the
// rpy. OMNISIM_URDF_INERTIAL_RPY=0 (value-parsed) restores the old import.
bool urdfInertialRpyEnabled() {
  static const bool enabled = [] {
    const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_INERTIAL_RPY")).trimmed().toLower();
    return value.isEmpty() || (value != "0" && value != "false" && value != "off" && value != "no");
  }();
  return enabled;
}

void rpyToMatrix(double r, double p, double y, double &m00, double &m01, double &m02,
                 double &m10, double &m11, double &m12, double &m20, double &m21, double &m22);

// Re-express a URDF inertia tensor through the rotation `rpy` (URDF fixed-axis
// convention): I' = R I R^T. Products of inertia use the URDF sign convention
// (un-negated integrals), which is the one Physics.inertiaMatrix uses too.
void rotateUrdfInertia(double &ixx, double &ixy, double &ixz, double &iyy, double &iyz, double &izz, double roll,
                       double pitch, double yaw) {
  if (roll == 0.0 && pitch == 0.0 && yaw == 0.0)
    return;
  double R[3][3];
  rpyToMatrix(roll, pitch, yaw, R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]);
  const double I[3][3] = {{ixx, ixy, ixz}, {ixy, iyy, iyz}, {ixz, iyz, izz}};
  double RI[3][3], Out[3][3];
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      RI[r][c] = R[r][0] * I[0][c] + R[r][1] * I[1][c] + R[r][2] * I[2][c];
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      Out[r][c] = RI[r][0] * R[c][0] + RI[r][1] * R[c][1] + RI[r][2] * R[c][2];
  ixx = Out[0][0];
  iyy = Out[1][1];
  izz = Out[2][2];
  // Symmetrize: R I R^T is symmetric in exact arithmetic only.
  ixy = 0.5 * (Out[0][1] + Out[1][0]);
  ixz = 0.5 * (Out[0][2] + Out[2][0]);
  iyz = 0.5 * (Out[1][2] + Out[2][1]);
}

// Principal moments (eigenvalues of the symmetric 3x3 inertia tensor), ascending.
// Closed form (Smith 1961) -- the C++ mirror of `principal_moments()` in
// scripts/dev/urdf_import.py, so the offline preflight and the native importer
// judge a tensor by the same rule.
void urdfPrincipalMoments(double ixx, double ixy, double ixz,
                          double iyy, double iyz, double izz, double out[3]) {
  const double p1 = ixy * ixy + ixz * ixz + iyz * iyz;
  if (p1 == 0.0) {
    out[0] = ixx; out[1] = iyy; out[2] = izz;
  } else {
    const double q = (ixx + iyy + izz) / 3.0;
    const double p2 = (ixx - q) * (ixx - q) + (iyy - q) * (iyy - q) + (izz - q) * (izz - q) + 2.0 * p1;
    const double p = std::sqrt(p2 / 6.0);
    const double b11 = (ixx - q) / p, b22 = (iyy - q) / p, b33 = (izz - q) / p;
    const double b12 = ixy / p, b13 = ixz / p, b23 = iyz / p;
    const double det = b11 * (b22 * b33 - b23 * b23)
                     - b12 * (b12 * b33 - b23 * b13)
                     + b13 * (b12 * b23 - b22 * b13);
    double r = det / 2.0;
    if (r < -1.0) r = -1.0;
    if (r > 1.0) r = 1.0;
    const double phi = std::acos(r) / 3.0;
    const double eig1 = q + 2.0 * p * std::cos(phi);
    const double eig3 = q + 2.0 * p * std::cos(phi + 2.0 * M_PI / 3.0);
    out[0] = eig1; out[1] = 3.0 * q - eig1 - eig3; out[2] = eig3;
  }
  std::sort(out, out + 3);
}

// A rigid body's principal moments must satisfy a + b >= c. A tensor can be
// positive definite and still describe no physical body, and MuJoCo's own
// compiler REJECTS such a body ("inertia must satisfy A + B >= C") -- which,
// now that the tensor actually reaches the solver, would turn a sloppy URDF
// into a hard load failure instead of a silent approximation. Falling back to
// the bounding object (loudly) keeps such a robot loadable.
bool urdfInertiaSatisfiesTriangleInequality(double ixx, double ixy, double ixz,
                                            double iyy, double iyz, double izz,
                                            double moments[3]) {
  urdfPrincipalMoments(ixx, ixy, ixz, iyy, iyz, izz, moments);
  return (moments[0] + moments[1]) >= moments[2] * (1.0 - 1e-9);
}

// URDF gazebo-extension sensors (IMU/GPS/Camera/Lidar) are imported only when
// this is set. Emitting bare InertialUnit/Gyro/Accelerometer/GPS devices into
// a child Solid currently crashes OmniSim' device registration mid-startup
// for reasons not yet diagnosed (suspect: devices need a Pose wrapper or to
// live at Robot root). Default OFF; opt in with OMNISIM_URDF_USE_SENSORS=1.
bool urdfUseSensorsEnabled() {
  const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_USE_SENSORS")).trimmed().toLower();
  return !value.isEmpty() && value != "0" && value != "false" && value != "off";
}

void normalizeVector(double &x, double &y, double &z) {
  const double norm = std::sqrt(x * x + y * y + z * z);
  if (norm < 1e-9) {
    x = 1.0;
    y = 0.0;
    z = 0.0;
    return;
  }
  x /= norm;
  y /= norm;
  z /= norm;
}

void warnUnsupportedGeometry(const QString &linkName, const QString &context, const UrdfGeometry &geometry) {
  if (geometry.kind == "mesh" && geometry.meshPath.isEmpty()) {
    OmLog::warning(QObject::tr("URDF link '%1': %2 mesh '%3' could not be resolved and will be skipped.")
                     .arg(linkName, context, geometry.detail),
                   false, OmLog::PARSING);
    return;
  }

  if (geometry.kind == "unsupported") {
    OmLog::warning(QObject::tr("URDF link '%1': unsupported %2 geometry <%3>; this element will be skipped.")
                     .arg(linkName, context, geometry.detail),
                   false, OmLog::PARSING);
  }
}

// Resolve a URDF <mesh filename="..."> reference to an absolute filesystem path
// with forward slashes, suitable for inlining into a OmniSim Mesh { url "..." }
// node. Returns an empty string if the file cannot be located on disk.
// Supports: package://<pkg>/<rest>, file:///<abs>, file://<abs>, absolute paths,
// and plain relative paths (resolved against the URDF directory).
QString resolveMeshPath(const QString &filenameAttr, const QString &urdfDir) {
  if (filenameAttr.isEmpty())
    return QString();

  auto asForwardAbs = [](const QString &p) -> QString {
    QString abs = QFileInfo(p).absoluteFilePath();
    abs.replace('\\', '/');
    return abs;
  };
  auto existsAbs = [&](const QString &candidate) -> QString {
    if (candidate.isEmpty())
      return QString();
    if (QFileInfo(candidate).isFile())
      return asForwardAbs(candidate);
    return QString();
  };

  QString s = filenameAttr;
  s.replace('\\', '/');

  // file:// URIs
  if (s.startsWith("file:///"))
    return existsAbs(s.mid(7));  // keep leading slash for POSIX
  if (s.startsWith("file://"))
    return existsAbs(s.mid(7));

  // package://<pkg>/<rest>
  if (s.startsWith("package://")) {
    const QString rest = s.mid(10);
    const int slash = rest.indexOf('/');
    if (slash <= 0)
      return QString();
    const QString pkg = rest.left(slash);
    const QString tail = rest.mid(slash + 1);

    // Walk up from the URDF directory. At each ancestor, try:
    //   1) <ancestor>/<pkg>/<tail>   (package is a sibling/descendant)
    //   2) <ancestor>/<tail>         (ancestor itself IS the package)
    QDir dir(urdfDir);
    for (int i = 0; i < 8; ++i) {  // walk up at most 8 levels
      const QString candidate1 = dir.filePath(pkg + "/" + tail);
      QString hit = existsAbs(candidate1);
      if (!hit.isEmpty())
        return hit;
      if (dir.dirName() == pkg) {
        const QString candidate2 = dir.filePath(tail);
        hit = existsAbs(candidate2);
        if (!hit.isEmpty())
          return hit;
      }
      if (!dir.cdUp())
        break;
    }
    // Last resort: strip the prefix and try relative to URDF dir.
    return existsAbs(QDir(urdfDir).filePath(tail));
  }

  // Absolute path (Unix / or Windows drive-letter)
  if (QFileInfo(s).isAbsolute())
    return existsAbs(s);

  // Plain relative path
  return existsAbs(QDir(urdfDir).filePath(s));
}

UrdfOrigin parseOrigin(const QDomElement &elem) {
  UrdfOrigin o;
  if (elem.isNull())
    return o;
  const QList<double> xyz = parseDoubles(elem.attribute("xyz"), 3);
  if (xyz.size() == 3) {
    o.x = xyz[0];
    o.y = xyz[1];
    o.z = xyz[2];
  }
  const QList<double> rpy = parseDoubles(elem.attribute("rpy"), 3);
  if (rpy.size() == 3) {
    o.roll = rpy[0];
    o.pitch = rpy[1];
    o.yaw = rpy[2];
  }
  return o;
}

UrdfGeometry parseGeometry(const QDomElement &elem, const QString &urdfDir) {
  UrdfGeometry g;
  if (elem.isNull())
    return g;
  const QDomElement box = elem.firstChildElement("box");
  if (!box.isNull()) {
    g.kind = "box";
    const QList<double> s = parseDoubles(box.attribute("size"), 3);
    if (s.size() == 3) {
      g.sx = s[0];
      g.sy = s[1];
      g.sz = s[2];
    }
    return g;
  }
  const QDomElement cyl = elem.firstChildElement("cylinder");
  if (!cyl.isNull()) {
    g.kind = "cylinder";
    g.radius = cyl.attribute("radius", "0.05").toDouble();
    g.length = cyl.attribute("length", "0.1").toDouble();
    return g;
  }
  const QDomElement sph = elem.firstChildElement("sphere");
  if (!sph.isNull()) {
    g.kind = "sphere";
    g.radius = sph.attribute("radius", "0.05").toDouble();
    return g;
  }
  const QDomElement mesh = elem.firstChildElement("mesh");
  if (!mesh.isNull()) {
    g.kind = "mesh";
    g.detail = mesh.attribute("filename");
    if (mesh.hasAttribute("scale")) {
      const QList<double> s = parseDoubles(mesh.attribute("scale"), 3);
      if (s.size() == 3) {
        g.meshScaleX = s[0];
        g.meshScaleY = s[1];
        g.meshScaleZ = s[2];
      }
    }
    g.meshPath = resolveMeshPath(g.detail, urdfDir);
    return g;
  }
  const QDomElement firstChild = elem.firstChildElement();
  if (!firstChild.isNull()) {
    g.kind = "unsupported";
    g.detail = firstChild.tagName();
  }
  return g;
}

UrdfMaterial parseMaterial(const QDomElement &elem, QHash<QString, UrdfMaterial> &materials,
                           const QString &urdfDir) {
  UrdfMaterial m;
  m.name = elem.attribute("name");
  const QDomElement texture = elem.firstChildElement("texture");
  if (!texture.isNull()) {
    const QString filename = texture.attribute("filename");
    m.texturePath = resolveMeshPath(filename, urdfDir);
    if (m.texturePath.isEmpty() && !filename.isEmpty())
      OmLog::warning(QObject::tr("URDF material '%1': texture '%2' not found; the part renders untextured.")
                       .arg(m.name)
                       .arg(filename));
  }
  const QDomElement pbr = elem.firstChildElement("omnisim");
  if (!pbr.isNull()) {
    bool ok = false;
    const double rough = pbr.attribute("roughness").toDouble(&ok);
    if (ok)
      m.roughness = qBound(0.0, rough, 1.0);
    ok = false;
    const double metal = pbr.attribute("metalness").toDouble(&ok);
    if (ok)
      m.metalness = qBound(0.0, metal, 1.0);
  }
  const QDomElement color = elem.firstChildElement("color");
  if (!color.isNull()) {
    const QList<double> rgba = parseDoubles(color.attribute("rgba"), 4);
    if (rgba.size() == 4) {
      m.r = rgba[0];
      m.g = rgba[1];
      m.b = rgba[2];
      m.a = rgba[3];
    }
    if (!m.name.isEmpty())
      materials.insert(m.name, m);
    return m;
  }
  if (!m.texturePath.isEmpty()) {
    // A texture with no colour is a complete definition too.
    m.r = m.g = m.b = 1.0;
    if (!m.name.isEmpty())
      materials.insert(m.name, m);
    return m;
  }
  if (!m.name.isEmpty() && materials.contains(m.name))
    return materials.value(m.name);
  return m;
}

UrdfLink parseLink(const QDomElement &elem, QHash<QString, UrdfMaterial> &materials, const QString &urdfDir) {
  UrdfLink link;
  link.name = elem.attribute("name");

  for (QDomElement v = elem.firstChildElement("visual"); !v.isNull(); v = v.nextSiblingElement("visual")) {
    UrdfVisual vis;
    vis.origin = parseOrigin(v.firstChildElement("origin"));
    vis.geometry = parseGeometry(v.firstChildElement("geometry"), urdfDir);
    warnUnsupportedGeometry(link.name, "visual", vis.geometry);
    const QDomElement matEl = v.firstChildElement("material");
    if (!matEl.isNull()) {
      vis.material = parseMaterial(matEl, materials, urdfDir);
      vis.hasMaterial = true;
    }
    link.visuals.append(vis);
  }

  for (QDomElement c = elem.firstChildElement("collision"); !c.isNull(); c = c.nextSiblingElement("collision")) {
    UrdfCollision coll;
    coll.origin = parseOrigin(c.firstChildElement("origin"));
    coll.geometry = parseGeometry(c.firstChildElement("geometry"), urdfDir);
    warnUnsupportedGeometry(link.name, "collision", coll.geometry);
    link.collisions.append(coll);
  }
  // Multiple <collision> elements are combined into a single OmniSim
  // boundingObject Group { children [...] } below in emitLinkPhysics.

  const QDomElement inertEl = elem.firstChildElement("inertial");
  if (!inertEl.isNull()) {
    link.inertial.present = true;
    link.inertial.origin = parseOrigin(inertEl.firstChildElement("origin"));
    const QDomElement massEl = inertEl.firstChildElement("mass");
    if (!massEl.isNull())
      link.inertial.mass = massEl.attribute("value", "0.0").toDouble();
    const QDomElement inertiaEl = inertEl.firstChildElement("inertia");
    if (!inertiaEl.isNull()) {
      const double ixx = inertiaEl.attribute("ixx", "0.0").toDouble();
      const double ixy = inertiaEl.attribute("ixy", "0.0").toDouble();
      const double ixz = inertiaEl.attribute("ixz", "0.0").toDouble();
      const double iyy = inertiaEl.attribute("iyy", "0.0").toDouble();
      const double iyz = inertiaEl.attribute("iyz", "0.0").toDouble();
      const double izz = inertiaEl.attribute("izz", "0.0").toDouble();
      // Only pass the tensor through if it's PD; OmniSim would reject it
      // otherwise and fall back to bounding-object derivation anyway.
      // Skipping silently here keeps the runtime warning surface clean for
      // the common case (Jackal, Husky, TurtleBot3) where every link is PD.
      if (inertiaIsPositiveDefinite(ixx, ixy, ixz, iyy, iyz, izz)) {
        link.inertial.ixx = ixx;
        link.inertial.ixy = ixy;
        link.inertial.ixz = ixz;
        link.inertial.iyy = iyy;
        link.inertial.iyz = iyz;
        link.inertial.izz = izz;
        link.inertial.hasInertiaMatrix = true;
        // The tensor is written in the inertial frame; inertiaMatrix takes it in
        // the link's axes (urdfInertialRpyEnabled). Rotation preserves positive
        // definiteness and the principal moments, so the gate above still holds.
        UrdfOrigin &io = link.inertial.origin;
        if (urdfInertialRpyEnabled() && (io.roll != 0.0 || io.pitch != 0.0 || io.yaw != 0.0)) {
          rotateUrdfInertia(link.inertial.ixx, link.inertial.ixy, link.inertial.ixz, link.inertial.iyy,
                            link.inertial.iyz, link.inertial.izz, io.roll, io.pitch, io.yaw);
          io.roll = io.pitch = io.yaw = 0.0;
        }
      } else {
        // Warned by emitLinkPhysics, which knows what the fallback keeps.
        link.inertial.declaredNotPositiveDefinite = true;
        link.inertial.declaredNonZeroDiagonal = ixx > 0.0 || iyy > 0.0 || izz > 0.0;
        link.inertial.declaredTensorText = QString("ixx=%1 iyy=%2 izz=%3 ixy=%4 ixz=%5 iyz=%6")
                                             .arg(ixx).arg(iyy).arg(izz)
                                             .arg(ixy).arg(ixz).arg(iyz);
      }
    }
  }

  return link;
}

UrdfJoint parseJoint(const QDomElement &elem) {
  UrdfJoint j;
  j.name = elem.attribute("name");
  j.type = elem.attribute("type", "fixed");
  const QDomElement parent = elem.firstChildElement("parent");
  if (!parent.isNull())
    j.parent = parent.attribute("link");
  const QDomElement child = elem.firstChildElement("child");
  if (!child.isNull())
    j.child = child.attribute("link");
  j.origin = parseOrigin(elem.firstChildElement("origin"));
  const QDomElement axisEl = elem.firstChildElement("axis");
  if (!axisEl.isNull()) {
    const QList<double> axis = parseDoubles(axisEl.attribute("xyz"), 3);
    if (axis.size() == 3) {
      j.ax = axis[0];
      j.ay = axis[1];
      j.az = axis[2];
    }
  }
  normalizeVector(j.ax, j.ay, j.az);

  const QDomElement limEl = elem.firstChildElement("limit");
  if (!limEl.isNull()) {
    if (limEl.hasAttribute("lower")) {
      j.lower = limEl.attribute("lower").toDouble();
      j.hasLower = true;
    }
    if (limEl.hasAttribute("upper")) {
      j.upper = limEl.attribute("upper").toDouble();
      j.hasUpper = true;
    }
    if (limEl.hasAttribute("velocity")) {
      j.velocity = limEl.attribute("velocity").toDouble();
      j.hasVelocity = true;
    }
    if (limEl.hasAttribute("effort")) {
      j.effort = limEl.attribute("effort").toDouble();
      j.hasEffort = true;
      // effort 0 imports as maxTorque/maxForce 0: the motor is OFF and the joint
      // moves freely, as SDF reads it. ROS-Industrial descriptions write 0 for
      // "unknown" on joints they drive, so say what happened.
      if (j.effort == 0.0 && (j.type == "revolute" || j.type == "continuous" || j.type == "prismatic"))
        OmLog::warning(QObject::tr("URDF joint '%1': <limit effort=\"0\"> imports as a motor with no torque, so "
                                   "the joint is passive and cannot be driven. If it is meant to be driven "
                                   "(ROS-Industrial descriptions use 0 for \"unknown\"), give it the real effort.")
                         .arg(j.name));
    }
  }

  const QDomElement mimicEl = elem.firstChildElement("mimic");
  if (!mimicEl.isNull() && !mimicEl.attribute("joint").trimmed().isEmpty()) {
    j.mimicJoint = mimicEl.attribute("joint").trimmed();
    if (mimicEl.hasAttribute("multiplier"))
      j.mimicMultiplier = mimicEl.attribute("multiplier").toDouble();
    if (mimicEl.hasAttribute("offset"))
      j.mimicOffset = mimicEl.attribute("offset").toDouble();
  }
  const QDomElement dynamicsEl = elem.firstChildElement("dynamics");
  if (!dynamicsEl.isNull()) {
    if (dynamicsEl.hasAttribute("damping")) {
      j.damping = dynamicsEl.attribute("damping").toDouble();
      j.hasDamping = true;
    }
    if (dynamicsEl.hasAttribute("friction")) {
      j.friction = dynamicsEl.attribute("friction").toDouble();
      j.hasFriction = true;
    }
  }
  // OmniSim extension: <rest>VALUE</rest> child sets the joint's initial
  // position (overrides OmniSim' default-to-midpoint when 0 is outside
  // the limits). Used to widen a joint's range while preserving the
  // spawn pose -- e.g. Spot's hip_y can be widened for self-righting
  // without breaking the policy trained at the old midpoint of 0.30.
  const QDomElement restEl = elem.firstChildElement("rest");
  if (!restEl.isNull()) {
    const QString restText = restEl.text().trimmed();
    bool ok = false;
    const double v = restText.toDouble(&ok);
    if (ok) {
      j.rest = v;
      j.hasRest = true;
    }
  }
  return j;
}

// Parse a modern <gazebo reference="LINK"><sensor> block.
// Returns true if a sensor was emitted; false on unrecognized type.
bool parseGazeboSensorBlock(const QDomElement &sensorEl, const QString &linkName,
                            QList<UrdfSensor> &out) {
  const QString type = sensorEl.attribute("type");
  const QString name = sensorEl.attribute("name");
  if (type.isEmpty() || linkName.isEmpty())
    return false;

  UrdfSensor s;
  s.linkName = linkName;
  s.name = name.isEmpty() ? linkName : name;
  const QDomElement updateEl = sensorEl.firstChildElement("update_rate");
  if (!updateEl.isNull())
    s.updateRate = updateEl.text().toDouble();

  if (type == "imu") {
    s.kind = UrdfSensorKind::Imu;
  } else if (type == "gps") {
    s.kind = UrdfSensorKind::Gps;
  } else if (type == "camera" || type == "depth") {
    s.kind = UrdfSensorKind::Camera;
    const QDomElement cam = sensorEl.firstChildElement("camera");
    if (!cam.isNull()) {
      const QDomElement hfovEl = cam.firstChildElement("horizontal_fov");
      if (!hfovEl.isNull())
        s.horizontalFov = hfovEl.text().toDouble();
      const QDomElement imageEl = cam.firstChildElement("image");
      if (!imageEl.isNull()) {
        const QDomElement w = imageEl.firstChildElement("width");
        const QDomElement h = imageEl.firstChildElement("height");
        if (!w.isNull())
          s.width = w.text().toInt();
        if (!h.isNull())
          s.height = h.text().toInt();
      }
    }
  } else if (type == "ray" || type == "gpu_ray" || type == "lidar" || type == "gpu_lidar") {
    s.kind = UrdfSensorKind::Lidar;
    const QDomElement ray = sensorEl.firstChildElement("ray");
    const QDomElement source = ray.isNull() ? sensorEl.firstChildElement("lidar") : ray;
    if (!source.isNull()) {
      const QDomElement scan = source.firstChildElement("scan");
      if (!scan.isNull()) {
        const QDomElement horizontal = scan.firstChildElement("horizontal");
        if (!horizontal.isNull()) {
          const QDomElement samples = horizontal.firstChildElement("samples");
          const QDomElement minA = horizontal.firstChildElement("min_angle");
          const QDomElement maxA = horizontal.firstChildElement("max_angle");
          if (!samples.isNull())
            s.lidarHorizontalResolution = samples.text().toInt();
          if (!minA.isNull() && !maxA.isNull())
            s.horizontalFov = maxA.text().toDouble() - minA.text().toDouble();
        }
        const QDomElement vertical = scan.firstChildElement("vertical");
        if (!vertical.isNull()) {
          const QDomElement samples = vertical.firstChildElement("samples");
          const QDomElement minA = vertical.firstChildElement("min_angle");
          const QDomElement maxA = vertical.firstChildElement("max_angle");
          if (!samples.isNull())
            s.lidarNumberOfLayers = samples.text().toInt();
          if (!minA.isNull() && !maxA.isNull())
            s.lidarVerticalFov = maxA.text().toDouble() - minA.text().toDouble();
        }
      }
      const QDomElement range = source.firstChildElement("range");
      if (!range.isNull()) {
        const QDomElement minR = range.firstChildElement("min");
        const QDomElement maxR = range.firstChildElement("max");
        if (!minR.isNull())
          s.minRange = minR.text().toDouble();
        if (!maxR.isNull())
          s.maxRange = maxR.text().toDouble();
      }
    }
  } else {
    return false;
  }

  out.append(s);
  return true;
}

// Legacy hector_gazebo plugin -> sensor mapping. URDFs from Clearpath (Jackal,
// Husky upstream) declare sensors only as plugin instantiations rather than
// <sensor> blocks. We recognize a few well-known filenames so those URDFs
// import with their full sensor suite even without edits.
bool parseGazeboPluginAsSensor(const QDomElement &pluginEl, QList<UrdfSensor> &out) {
  const QString filename = pluginEl.attribute("filename");
  if (filename.isEmpty())
    return false;
  const QDomElement bodyEl = pluginEl.firstChildElement("bodyName");
  const QString bodyName = bodyEl.isNull() ? QString() : bodyEl.text().trimmed();
  if (bodyName.isEmpty())
    return false;

  UrdfSensor s;
  s.linkName = bodyName;
  const QDomElement nameEl = pluginEl.firstChildElement("topicName");
  if (!nameEl.isNull()) {
    QString topic = nameEl.text().trimmed();
    if (topic.startsWith('/'))
      topic = topic.mid(1);
    s.name = topic.isEmpty() ? bodyName : topic.replace('/', '_');
  } else {
    s.name = pluginEl.attribute("name", bodyName);
  }
  const QDomElement rateEl = pluginEl.firstChildElement("updateRate");
  if (!rateEl.isNull())
    s.updateRate = rateEl.text().toDouble();

  if (filename.contains("hector_gazebo_ros_imu") || filename.contains("gazebo_ros_imu")) {
    s.kind = UrdfSensorKind::Imu;
  } else if (filename.contains("hector_gazebo_ros_gps") || filename.contains("gazebo_ros_gps")) {
    s.kind = UrdfSensorKind::Gps;
  } else {
    return false;
  }

  out.append(s);
  return true;
}

void parseGazeboExtensions(const QDomElement &robotRoot, UrdfRobot &robot) {
  for (QDomElement g = robotRoot.firstChildElement("gazebo"); !g.isNull();
       g = g.nextSiblingElement("gazebo")) {
    const QString reference = g.attribute("reference");
    // <gazebo reference="LINK"><mu1>/<mu2></gazebo>: the link's contact friction
    // (2026-10-09). Both the text form (<mu1>0.2</mu1>) and the attribute form
    // (<mu1 value="0.5"/>) occur in published URDFs. A link that set it -- a caster
    // meant to roll at 0.2, a drive wheel at 100 -- used to meet the floor at the
    // world's newtonGroundMu. It becomes Solid.newtonFriction, which the runtime lets
    // win the contact against unstated surfaces, as Gazebo's min rule does against its
    // high-friction ground. A direction-dependent pair (mu1 along fdir1, mu2 across)
    // takes the smaller, keeping a caster's free direction free. Value-parsed hatch:
    // OMNISIM_URDF_GAZEBO_FRICTION=0 ignores the tags.
    static const bool gazeboFriction = [] {
      const QString v = QString::fromUtf8(qgetenv("OMNISIM_URDF_GAZEBO_FRICTION")).trimmed().toLower();
      return !(v == "0" || v == "false" || v == "off" || v == "no");
    }();
    if (gazeboFriction && !reference.isEmpty() && robot.links.contains(reference)) {
      double mu = -1.0;
      for (const char *tag : {"mu1", "mu2"}) {
        const QDomElement e = g.firstChildElement(tag);
        if (e.isNull())
          continue;
        QString text = e.text().trimmed();
        if (text.isEmpty())
          text = e.attribute("value").trimmed();
        bool ok = false;
        const double v = text.toDouble(&ok);
        if (ok && v >= 0.0)
          mu = (mu < 0.0) ? v : qMin(mu, v);
      }
      // Gazebo URDFs write "infinite" friction as mu 100 (ODE takes the min with the
      // ground, so it only means "never slip"). MuJoCo's friction cone degenerates at
      // that value: a Neobotix MP-400 with mu1/mu2 100 on its drive wheels spun them at
      // the commanded rate while the body moved 0 m; at 1.5 it drove 0.871 of command
      // with its wheels tracking 0.871. Above kGazeboMuCap the value is imported as the
      // cap, which is already no-slip for a wheel, and the clamp is logged.
      static constexpr double kGazeboMuCap = 2.0;
      if (mu > kGazeboMuCap) {
        OmLog::info(QObject::tr("URDF link '%1': <gazebo> friction %2 imported as %3 (MuJoCo's friction cone "
                                "degenerates at Gazebo's 'never slip' values; %3 does not slip either)")
                      .arg(reference)
                      .arg(mu)
                      .arg(kGazeboMuCap));
        mu = kGazeboMuCap;
      }
      if (mu >= 0.0)
        robot.links[reference].gazeboMu = mu;
    }
    // <gazebo reference="LINK"><sensor>...</sensor></gazebo>
    if (!reference.isEmpty()) {
      for (QDomElement sensor = g.firstChildElement("sensor"); !sensor.isNull();
           sensor = sensor.nextSiblingElement("sensor")) {
        parseGazeboSensorBlock(sensor, reference, robot.sensors);
      }
    }
    // <gazebo><plugin filename="libhector_gazebo_ros_imu.so">...
    for (QDomElement plugin = g.firstChildElement("plugin"); !plugin.isNull();
         plugin = plugin.nextSiblingElement("plugin")) {
      parseGazeboPluginAsSensor(plugin, robot.sensors);
    }
  }
}

bool parseRobotXml(const QByteArray &xml, const QString &sourceLabel, const QString &urdfDir, UrdfRobot &robot) {
  QDomDocument doc;
  QString errorMsg;
  int errorLine = 0;
  if (!doc.setContent(xml, &errorMsg, &errorLine)) {
    OmLog::error(QObject::tr("URDF parse error in '%1' at line %2: %3")
                   .arg(sourceLabel)
                   .arg(errorLine)
                   .arg(errorMsg));
    return false;
  }
  const QDomElement root = doc.documentElement();
  if (root.tagName() != "robot") {
    OmLog::error(QObject::tr("URDF '%1': expected <robot> root, got <%2>")
                   .arg(sourceLabel)
                   .arg(root.tagName()));
    return false;
  }
  robot.name = root.attribute("name", "robot");

  for (QDomElement m = root.firstChildElement("material"); !m.isNull(); m = m.nextSiblingElement("material"))
    parseMaterial(m, robot.materials, urdfDir);

  for (QDomElement l = root.firstChildElement("link"); !l.isNull(); l = l.nextSiblingElement("link")) {
    UrdfLink link = parseLink(l, robot.materials, urdfDir);
    robot.links.insert(link.name, link);
    robot.linkOrder.append(link.name);
  }

  for (QDomElement j = root.firstChildElement("joint"); !j.isNull(); j = j.nextSiblingElement("joint"))
    robot.joints.append(parseJoint(j));

  parseGazeboExtensions(root, robot);

  // Dedupe: keep first occurrence per (kind, linkName, name) tuple. Some URDFs
  // declare a sensor both with <gazebo reference><sensor> and a legacy plugin,
  // and we don't want both copies in the scene.
  QSet<QString> sensorKeys;
  QList<UrdfSensor> deduped;
  for (const UrdfSensor &s : robot.sensors) {
    const QString key = QString("%1|%2|%3").arg(int(s.kind)).arg(s.linkName, s.name);
    if (sensorKeys.contains(key))
      continue;
    sensorKeys.insert(key);
    deduped.append(s);
  }
  robot.sensors = deduped;

  return true;
}

// ---------------------------------------------------------------------------
// VRML emission
// ---------------------------------------------------------------------------

bool isSupportedPrimitive(const QString &kind) {
  return kind == "box" || kind == "cylinder" || kind == "sphere";
}

bool isSupportedGeometry(const UrdfGeometry &g) {
  if (isSupportedPrimitive(g.kind))
    return true;
  if (g.kind == "mesh" && !g.meshPath.isEmpty())
    return true;
  return false;
}

bool meshHasNonUnitScale(const UrdfGeometry &g) {
  return g.kind == "mesh" && (g.meshScaleX != 1.0 || g.meshScaleY != 1.0 || g.meshScaleZ != 1.0);
}

void rpyToMatrix(double r, double p, double y, double &m00, double &m01, double &m02,
                 double &m10, double &m11, double &m12, double &m20, double &m21, double &m22) {
  const double cr = std::cos(r), sr = std::sin(r);
  const double cp = std::cos(p), sp = std::sin(p);
  const double cy = std::cos(y), sy = std::sin(y);

  // R = Rz(y) * Ry(p) * Rx(r)
  m00 = cy * cp;
  m01 = cy * sp * sr - sy * cr;
  m02 = cy * sp * cr + sy * sr;
  m10 = sy * cp;
  m11 = sy * sp * sr + cy * cr;
  m12 = sy * sp * cr - cy * sr;
  m20 = -sp;
  m21 = cp * sr;
  m22 = cp * cr;
}

void rotateVectorByOrigin(const UrdfOrigin &origin, double ix, double iy, double iz, double &ox, double &oy, double &oz) {
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  rpyToMatrix(origin.roll, origin.pitch, origin.yaw, m00, m01, m02, m10, m11, m12, m20, m21, m22);
  ox = m00 * ix + m01 * iy + m02 * iz;
  oy = m10 * ix + m11 * iy + m12 * iz;
  oz = m20 * ix + m21 * iy + m22 * iz;
  normalizeVector(ox, oy, oz);
}

void jointAxisInParentFrame(const UrdfJoint &joint, double &ax, double &ay, double &az) {
  rotateVectorByOrigin(joint.origin, joint.ax, joint.ay, joint.az, ax, ay, az);
}

QString vectorString(double x, double y, double z) {
  return QString("%1 %2 %3").arg(x).arg(y).arg(z);
}

void rpyToAxisAngle(double r, double p, double y, double &ax, double &ay, double &az, double &angle) {
  if (r == 0.0 && p == 0.0 && y == 0.0) {
    ax = 0.0;
    ay = 0.0;
    az = 1.0;
    angle = 0.0;
    return;
  }
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  rpyToMatrix(r, p, y, m00, m01, m02, m10, m11, m12, m20, m21, m22);

  const double trace = m00 + m11 + m22;
  double cosAngle = (trace - 1.0) / 2.0;
  if (cosAngle > 1.0)
    cosAngle = 1.0;
  if (cosAngle < -1.0)
    cosAngle = -1.0;
  angle = std::acos(cosAngle);
  if (std::abs(angle) < 1e-9) {
    ax = 0.0;
    ay = 0.0;
    az = 1.0;
    angle = 0.0;
    return;
  }
  if (std::abs(angle - M_PI) < 1e-6) {
    if (m00 > m11 && m00 > m22) {
      ax = std::sqrt(std::max(0.0, m00 - m11 - m22 + 1.0)) / 2.0;
      ay = ax > 0 ? m01 / (2.0 * ax) : 0.0;
      az = ax > 0 ? m02 / (2.0 * ax) : 0.0;
    } else if (m11 > m22) {
      ay = std::sqrt(std::max(0.0, m11 - m00 - m22 + 1.0)) / 2.0;
      ax = ay > 0 ? m01 / (2.0 * ay) : 0.0;
      az = ay > 0 ? m12 / (2.0 * ay) : 0.0;
    } else {
      az = std::sqrt(std::max(0.0, m22 - m00 - m11 + 1.0)) / 2.0;
      ax = az > 0 ? m02 / (2.0 * az) : 0.0;
      ay = az > 0 ? m12 / (2.0 * az) : 0.0;
    }
    angle = M_PI;
    return;
  }
  const double s = 2.0 * std::sin(angle);
  ax = (m21 - m12) / s;
  ay = (m02 - m20) / s;
  az = (m10 - m01) / s;
}

// URDF rpy of a rotation matrix (the inverse of rpyToMatrix: R = Rz(y) Ry(p) Rx(r)).
void matrixToRpy(double m00, double m10, double m20, double m21, double m22, double m01, double m11,
                 double &r, double &p, double &y) {
  const double sp = std::max(-1.0, std::min(1.0, -m20));
  p = std::asin(sp);
  if (std::abs(sp) < 1.0 - 1e-12) {
    r = std::atan2(m21, m22);
    y = std::atan2(m10, m00);
  } else {
    // Gimbal lock (pitch = +-90 deg): roll and yaw share one axis; put it all in yaw.
    r = 0.0;
    y = std::atan2(-m01, m11);
  }
}

// T_a * T_b for two URDF origins: the frame b is expressed in, followed by b.
UrdfOrigin composeUrdfOrigins(const UrdfOrigin &a, const UrdfOrigin &b) {
  double a00, a01, a02, a10, a11, a12, a20, a21, a22;
  double b00, b01, b02, b10, b11, b12, b20, b21, b22;
  rpyToMatrix(a.roll, a.pitch, a.yaw, a00, a01, a02, a10, a11, a12, a20, a21, a22);
  rpyToMatrix(b.roll, b.pitch, b.yaw, b00, b01, b02, b10, b11, b12, b20, b21, b22);
  UrdfOrigin out;
  out.x = a.x + a00 * b.x + a01 * b.y + a02 * b.z;
  out.y = a.y + a10 * b.x + a11 * b.y + a12 * b.z;
  out.z = a.z + a20 * b.x + a21 * b.y + a22 * b.z;
  const bool aRot = a.roll != 0.0 || a.pitch != 0.0 || a.yaw != 0.0;
  const bool bRot = b.roll != 0.0 || b.pitch != 0.0 || b.yaw != 0.0;
  if (!aRot) {
    out.roll = b.roll; out.pitch = b.pitch; out.yaw = b.yaw;  // exact: no re-extraction round trip
  } else if (!bRot) {
    out.roll = a.roll; out.pitch = a.pitch; out.yaw = a.yaw;
  } else {
    const double c00 = a00 * b00 + a01 * b10 + a02 * b20, c01 = a00 * b01 + a01 * b11 + a02 * b21;
    const double c10 = a10 * b00 + a11 * b10 + a12 * b20, c11 = a10 * b01 + a11 * b11 + a12 * b21;
    const double c20 = a20 * b00 + a21 * b10 + a22 * b20, c21 = a20 * b01 + a21 * b11 + a22 * b21;
    const double c22 = a20 * b02 + a21 * b12 + a22 * b22;
    matrixToRpy(c00, c10, c20, c21, c22, c01, c11, out.roll, out.pitch, out.yaw);
  }
  return out;
}

bool isIdentityUrdfOrigin(const UrdfOrigin &o) {
  return o.x == 0.0 && o.y == 0.0 && o.z == 0.0 && o.roll == 0.0 && o.pitch == 0.0 && o.yaw == 0.0;
}

// OMNISIM_URDF_REROOT_KEEP_OFFSET (value-parsed, default ON since 2026-10-03).
// When the importer re-roots past an empty frame link (base_footprint ->
// base_link, see emitRobot), the fixed joint's offset is KEPT: the Robot's
// frame stays the URDF root's (base_footprint, on the floor) and the promoted
// link's visuals, collisions, inertial and child joints are emitted at that
// offset. Until then the offset was dropped, so every child joint of the
// promoted link moved by it -- a TurtleBot3 Burger's wheel axles landed at
// z 0.023 in the Robot frame instead of 0.033 and the chassis rode the 10 mm
// base_joint offset high relative to its `translation` (Pi Mouse: 1.85 mm).
// =0 restores the dropped offset.
bool urdfRerootKeepOffsetEnabled() {
  const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_REROOT_KEEP_OFFSET")).trimmed().toLower();
  if (value.isEmpty())
    return true;
  return value != "0" && value != "false" && value != "off" && value != "no";
}

QString emitGeometry(const UrdfGeometry &g, const QString &indent) {
  if (g.kind == "box")
    return QString("%1geometry Box { size %2 %3 %4 }\n").arg(indent).arg(g.sx).arg(g.sy).arg(g.sz);
  if (g.kind == "cylinder")
    return QString("%1geometry Cylinder { height %2 radius %3 }\n").arg(indent).arg(g.length).arg(g.radius);
  if (g.kind == "sphere")
    return QString("%1geometry Sphere { radius %2 }\n").arg(indent).arg(g.radius);
  if (g.kind == "mesh" && !g.meshPath.isEmpty())
    return QString("%1geometry Mesh { url \"%2\" }\n").arg(indent, g.meshPath);
  return QString();
}

// Deterministic color from link name - spreads links across a pleasant palette
static void linkFallbackColor(const QString &linkName, double &r, double &g, double &b) {
  // A curated palette of readable, professional-looking colors
  static const double palette[][3] = {
    {0.35, 0.45, 0.65},  // slate blue
    {0.70, 0.40, 0.35},  // terracotta
    {0.40, 0.55, 0.45},  // sage green
    {0.75, 0.65, 0.40},  // ochre
    {0.45, 0.40, 0.60},  // dusty violet
    {0.60, 0.50, 0.40},  // warm taupe
    {0.40, 0.60, 0.65},  // teal
    {0.65, 0.55, 0.50},  // pale brown
  };
  const int paletteSize = sizeof(palette) / sizeof(palette[0]);
  // Hash the link name to pick a palette entry
  uint hash = 0;
  for (QChar c : linkName)
    hash = hash * 31u + c.unicode();
  const int idx = hash % paletteSize;
  r = palette[idx][0];
  g = palette[idx][1];
  b = palette[idx][2];
}

QString emitVisual(const UrdfVisual &v, const QString &indent, const QString &linkName) {
  if (!isSupportedGeometry(v.geometry))
    return QString();
  double ax, ay, az, ang;
  rpyToAxisAngle(v.origin.roll, v.origin.pitch, v.origin.yaw, ax, ay, az, ang);
  const bool hasOrigin = (v.origin.x != 0.0 || v.origin.y != 0.0 || v.origin.z != 0.0 || ang != 0.0);
  const bool needsScale = meshHasNonUnitScale(v.geometry);
  const bool hasTransform = hasOrigin || needsScale;

  QString out;
  QString innerIndent = indent;
  if (hasTransform) {
    // Use Transform (scalable) when the mesh has a non-unit scale, Pose otherwise.
    const QString wrapper = needsScale ? QStringLiteral("Transform") : QStringLiteral("Pose");
    out += indent + wrapper + " {\n";
    out += indent + QString("  translation %1 %2 %3\n").arg(v.origin.x).arg(v.origin.y).arg(v.origin.z);
    if (ang != 0.0)
      out += indent + QString("  rotation %1 %2 %3 %4\n").arg(ax).arg(ay).arg(az).arg(ang);
    if (needsScale)
      out += indent + QString("  scale %1 %2 %3\n").arg(v.geometry.meshScaleX).arg(v.geometry.meshScaleY).arg(v.geometry.meshScaleZ);
    out += indent + "  children [\n";
    innerIndent = indent + "    ";
  }

  out += innerIndent + "Shape {\n";

  // Imported URDF meshes are frequently dense (robot wheel DAEs easily hit
  // 25k+ triangles); OmniSim can't cast shadows above ~21k triangles and
  // would otherwise log a warning per mesh. Disable shadow casting on
  // mesh-backed shapes - a cosmetic tradeoff that silences the warning.
  if (v.geometry.kind == "mesh")
    out += innerIndent + "  castShadows FALSE\n";

  // Always emit an appearance. Use URDF material if available, otherwise
  // fall back to a deterministic palette color based on the link name.
  double r, g, b;
  if (v.hasMaterial) {
    r = v.material.r;
    g = v.material.g;
    b = v.material.b;
  } else {
    linkFallbackColor(linkName, r, g, b);
  }
  out += innerIndent + "  appearance PBRAppearance {\n";
  out += innerIndent + QString("    baseColor %1 %2 %3\n").arg(r).arg(g).arg(b);
  const double rgh = (v.hasMaterial && v.material.roughness >= 0.0) ? v.material.roughness : 0.5;
  const double mtl = (v.hasMaterial && v.material.metalness >= 0.0) ? v.material.metalness : 0.0;
  out += innerIndent + QString("    roughness %1\n").arg(rgh);
  out += innerIndent + QString("    metalness %1\n").arg(mtl);
  if (v.hasMaterial && v.material.a < 1.0)
    out += innerIndent + QString("    transparency %1\n").arg(1.0 - qBound(0.0, v.material.a, 1.0));
  if (v.hasMaterial && !v.material.texturePath.isEmpty())
    out += innerIndent + QString("    baseColorMap ImageTexture { url [ \"%1\" ] }\n").arg(v.material.texturePath);
  // The URDF material name, so a controller can find a part's appearance
  // (e.g. rotor blur discs faded with rotor speed). Omitted when unnamed.
  if (v.hasMaterial && !v.material.name.isEmpty())
    out += innerIndent + QString("    name \"%1\"\n").arg(v.material.name);
  out += innerIndent + "  }\n";

  out += emitGeometry(v.geometry, innerIndent + "  ");
  out += innerIndent + "}\n";

  if (hasTransform) {
    out += indent + "  ]\n";
    out += indent + "}\n";
  }
  return out;
}

// A URDF <collision><mesh scale="..."> is written as Mesh.scale (2026-10-03).
// A boundingObject cannot hold a Transform, so before Mesh had a scale field the
// importer could only warn and load the collider at UNIT scale -- the Neobotix
// MP-400 body (MP-400-BODY.dae, authored in millimetres, scale 0.001) collided
// 1000x too large, and the ROX frame, nanoscan and Argo casters likewise.
// OMNISIM_URDF_COLLISION_MESH_SCALE=0 restores the unit-scale import and its
// warning (value-parsed).
bool urdfCollisionMeshScaleEnabled() {
  const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_COLLISION_MESH_SCALE")).trimmed().toLower();
  if (value.isEmpty())
    return true;
  return value != "0" && value != "false" && value != "off" && value != "no";
}

QString meshScaleField(const UrdfGeometry &g) {
  if (!meshHasNonUnitScale(g) || !urdfCollisionMeshScaleEnabled())
    return QString();
  return QString(" scale %1 %2 %3").arg(g.meshScaleX, 0, 'g', 17).arg(g.meshScaleY, 0, 'g', 17).arg(g.meshScaleZ, 0, 'g', 17);
}

// Emit the raw geometry primitive (Box, Cylinder, Sphere, Mesh) with the given indent.
// Only collision paths call this (visuals go through emitGeometry, whose Transform
// carries the scale), so a mesh's URDF scale rides on Mesh.scale here.
QString emitRawGeometry(const UrdfGeometry &g, const QString &indent) {
  if (g.kind == "box")
    return indent + QString("Box { size %1 %2 %3 }\n").arg(g.sx).arg(g.sy).arg(g.sz);
  if (g.kind == "cylinder")
    return indent + QString("Cylinder { height %1 radius %2 }\n").arg(g.length).arg(g.radius);
  if (g.kind == "sphere")
    return indent + QString("Sphere { radius %1 }\n").arg(g.radius);
  if (g.kind == "mesh" && !g.meshPath.isEmpty())
    return indent + QString("Mesh { url \"%1\"%2 }\n").arg(g.meshPath, meshScaleField(g));
  return QString();
}

// A boundingObject cannot hold a Transform, so the scale travels as Mesh.scale
// (meshScaleField above). Only with that disabled is the mesh emitted at unit
// scale, and then we warn once per link so the substitution is visible.
void warnMeshCollisionScale(const QString &linkName, const UrdfGeometry &g) {
  if (meshHasNonUnitScale(g) && !urdfCollisionMeshScaleEnabled()) {
    OmLog::warning(QObject::tr("URDF link '%1': collision mesh '%2' has scale %3 %4 %5 which cannot be applied "
                               "inside a boundingObject; it will be loaded at unit scale.")
                     .arg(linkName, g.detail)
                     .arg(g.meshScaleX).arg(g.meshScaleY).arg(g.meshScaleZ),
                   false, OmLog::PARSING);
  }
}

// Emit a single collision as either a raw geometry or a Pose-wrapped geometry,
// suitable for placement inside a Group's children list.
QString emitCollisionNode(const UrdfCollision &c, const QString &indent, const QString &linkName) {
  if (!isSupportedGeometry(c.geometry))
    return QString();
  warnMeshCollisionScale(linkName, c.geometry);
  double ax, ay, az, ang;
  rpyToAxisAngle(c.origin.roll, c.origin.pitch, c.origin.yaw, ax, ay, az, ang);
  const bool hasTransform = (c.origin.x != 0.0 || c.origin.y != 0.0 || c.origin.z != 0.0 || ang != 0.0);

  if (!hasTransform)
    return emitRawGeometry(c.geometry, indent);

  QString out = indent + "Pose {\n";
  out += indent + QString("  translation %1 %2 %3\n").arg(c.origin.x).arg(c.origin.y).arg(c.origin.z);
  if (ang != 0.0)
    out += indent + QString("  rotation %1 %2 %3 %4\n").arg(ax).arg(ay).arg(az).arg(ang);
  out += indent + "  children [\n";
  out += emitRawGeometry(c.geometry, indent + "    ");
  out += indent + "  ]\n";
  out += indent + "}\n";
  return out;
}

// Emit a single collision wrapped as a top-level boundingObject (for links
// with exactly one supported collision primitive).
//
// Cylinder geometries are ALWAYS wrapped in a Pose, even when the URDF
// collision origin is identity. OmniSim' bare-Cylinder boundingObject path on
// a Solid that has a non-identity `rotation` field has been observed to leave
// the ODE cylinder geom Z-axis-aligned in the body frame instead of following
// the body rotation -- it shows up as a wheel that physically rotates but
// generates no rolling friction (the cylinder ends up flat-on-floor when it
// should be edge-on-floor). The Pose-wrapped path doesn't have this issue:
// Husky's wheels go through it (they declare a 90deg collision-origin rpy in
// URDF) and roll correctly; TurtleBot3 burger's wheels declare an identity
// collision origin and were stuck. Wrapping unconditionally costs one extra
// node per cylinder and aligns the geom path with the well-tested route.
QString emitBoundingObject(const UrdfCollision &c, const QString &indent, const QString &linkName) {
  if (!isSupportedGeometry(c.geometry))
    return QString();
  warnMeshCollisionScale(linkName, c.geometry);
  double ax, ay, az, ang;
  rpyToAxisAngle(c.origin.roll, c.origin.pitch, c.origin.yaw, ax, ay, az, ang);
  const bool hasTransform = (c.origin.x != 0.0 || c.origin.y != 0.0 || c.origin.z != 0.0 || ang != 0.0);
  const bool forcePoseWrap = c.geometry.kind == "cylinder";

  if (!hasTransform && !forcePoseWrap) {
    if (c.geometry.kind == "box")
      return indent + QString("boundingObject Box { size %1 %2 %3 }\n").arg(c.geometry.sx).arg(c.geometry.sy).arg(c.geometry.sz);
    if (c.geometry.kind == "sphere")
      return indent + QString("boundingObject Sphere { radius %1 }\n").arg(c.geometry.radius);
    if (c.geometry.kind == "mesh")
      return indent + QString("boundingObject Mesh { url \"%1\"%2 }\n").arg(c.geometry.meshPath, meshScaleField(c.geometry));
    return QString();
  }

  QString out = indent + "boundingObject Pose {\n";
  out += indent + QString("  translation %1 %2 %3\n").arg(c.origin.x).arg(c.origin.y).arg(c.origin.z);
  if (ang != 0.0)
    out += indent + QString("  rotation %1 %2 %3 %4\n").arg(ax).arg(ay).arg(az).arg(ang);
  out += indent + "  children [\n";
  out += emitRawGeometry(c.geometry, indent + "    ");
  out += indent + "  ]\n";
  out += indent + "}\n";
  return out;
}

// Emit a boundingObject Group { children [...] } containing multiple collisions.
QString emitBoundingObjectGroup(const QList<UrdfCollision> &collisions, const QString &indent, const QString &linkName) {
  QString out = indent + "boundingObject Group {\n";
  out += indent + "  children [\n";
  for (const UrdfCollision &c : collisions) {
    if (isSupportedGeometry(c.geometry))
      out += emitCollisionNode(c, indent + "    ", linkName);
  }
  out += indent + "  ]\n";
  out += indent + "}\n";
  return out;
}

// Emit a single sensor cluster as a child Solid of the Robot. The carrier
// Solid has its own tiny bounding box + Physics, mirroring the pattern from
// projects/samples/devices/worlds/imu.omniworld. This is the only structure
// observed to load reliably: bare OmSolidDevice nodes as direct children of
// Robot crash at world load when there are two or more siblings (verified by
// bisecting hand-authored test worlds -- one InertialUnit works, two devices
// trip ACCESS_VIOLATION). Wrapping the cluster in a Pose also crashes. The
// stock IMU sample puts them inside a Solid; that loads cleanly.
QString emitOneSensorAtRoot(const UrdfSensor &s, const QString &indent) {
  QString out;
  out += indent + "Solid {\n";
  out += indent + QString("  name \"%1_carrier\"\n").arg(s.name);
  out += indent + QString("  translation %1 %2 %3\n").arg(s.translationX).arg(s.translationY).arg(s.translationZ);
  out += indent + "  children [\n";
  const QString in = indent + "    ";
  switch (s.kind) {
    case UrdfSensorKind::Imu:
      out += in + QString("InertialUnit { name \"%1\" }\n").arg(s.name);
      out += in + QString("Gyro { name \"%1_gyro\" }\n").arg(s.name);
      out += in + QString("Accelerometer { name \"%1_accel\" }\n").arg(s.name);
      break;
    case UrdfSensorKind::Gps:
      out += in + QString("GPS { name \"%1\" }\n").arg(s.name);
      break;
    case UrdfSensorKind::Camera:
      out += in + "Camera {\n";
      out += in + QString("  name \"%1\"\n").arg(s.name);
      out += in + QString("  width %1\n").arg(s.width);
      out += in + QString("  height %1\n").arg(s.height);
      out += in + QString("  fieldOfView %1\n").arg(s.horizontalFov);
      out += in + "}\n";
      break;
    case UrdfSensorKind::Lidar:
      out += in + "Lidar {\n";
      out += in + QString("  name \"%1\"\n").arg(s.name);
      out += in + QString("  horizontalResolution %1\n").arg(s.lidarHorizontalResolution);
      out += in + QString("  fieldOfView %1\n").arg(s.horizontalFov);
      out += in + QString("  minRange %1\n").arg(s.minRange);
      out += in + QString("  maxRange %1\n").arg(s.maxRange);
      // numberOfLayers is ALWAYS written (2026-09-30). The Lidar node's own
      // default is 4 layers over a 0.2 rad vertical fan (Lidar.wrl), so a
      // planar URDF <ray> -- no <vertical> block, lidarNumberOfLayers == 1 --
      // used to arrive as a 4-layer device tilted +-5.7 / +-1.9 deg: measured
      // on the TurtleBot3 LDS-01 (2026-09-30), the bottom layer read the
      // FLOOR at ~1.8 m and the Jackal LMS1xx's at ~4 m, and a consumer
      // picking "the layer with most returns" (omnisim_ros2's sensor_node)
      // took the floor ring for obstacles.
      out += in + QString("  numberOfLayers %1\n").arg(s.lidarNumberOfLayers);
      if (s.lidarNumberOfLayers > 1 && s.lidarVerticalFov > 0.0)
        out += in + QString("  verticalFieldOfView %1\n").arg(s.lidarVerticalFov);
      out += in + "}\n";
      break;
  }
  out += indent + "  ]\n";
  out += indent + "  boundingObject Box { size 0.001 0.001 0.001 }\n";
  out += indent + "  physics Physics { density -1 mass 0.001 }\n";
  out += indent + "}\n";
  return out;
}

QString emitSensorsForLink(const QString &linkName, const QString &indent,
                           const QHash<QString, QList<UrdfSensor>> &sensorsByLink,
                           const UrdfOrigin *linkOffset = nullptr) {
  if (!urdfUseSensorsEnabled())
    return QString();
  if (!sensorsByLink.contains(linkName))
    return QString();
  // Emit carrier Solids inline as children of the link's own Solid. Each
  // carrier has a tiny bbox + Physics; SolidMerger absorbs them into the
  // parent's body since the carrier-to-link relationship is a fixed-joint
  // children edge. Stays at the link's frame, no need to walk back to the
  // root.
  QString out;
  for (const UrdfSensor &s : sensorsByLink.value(linkName)) {
    UrdfSensor local = s;
    // linkOffset: the link's frame in its Solid's frame -- non-null only for a
    // re-rooted Robot link whose dropped-frame offset is kept (emitRobot).
    // Translation only, like every other carrier placement in this file.
    local.translationX = linkOffset ? linkOffset->x : 0.0;
    local.translationY = linkOffset ? linkOffset->y : 0.0;
    local.translationZ = linkOffset ? linkOffset->z : 0.0;
    out += emitOneSensorAtRoot(local, indent);
  }
  return out;
}

QString emitLinkChildren(const UrdfLink &link, const QString &indent,
                        const QHash<QString, QList<UrdfSensor>> &sensorsByLink,
                        const UrdfOrigin *linkOffset = nullptr) {
  QString out;
  for (const UrdfVisual &v : link.visuals)
    out += emitVisual(v, indent, link.name);
  out += emitSensorsForLink(link.name, indent, sensorsByLink, linkOffset);
  return out;
}

// `bodyForFixedSubtree`: the link is the child of a MOVING joint, declares neither
// <inertial> nor <collision>, but links welded below it by fixed joints carry mass
// or collision (computed by emptyLinkCarriesFixedSubtree). It must become a body,
// or nothing is: with no Physics it is a kinematic frame, its joint has no dynamic
// endpoint, and the fixed subtree -- whose merge walk stops at that joint endpoint
// -- registers as a FREE body of its own. Measured 2026-10-06: a 1 kg gripper
// fixed below such a link fell away to z = -11 m while the arm swung on without
// it; on Kinova's Gen3 lite (end_effector_link) the gripper and any tool payload
// vanished from the arm's joint torques. MuJoCo fuses the fixed children into the
// body instead, which is what the composite merge does once this link has a body.
// Solid.newtonFriction from a link's <gazebo> <mu1>/<mu2> (parseGazeboExtensions). Emitted on
// link Solids only: the robot root is a Robot node, which declares no newtonFriction field,
// so writing it there is a parse error that failed the whole load.
static QString emitLinkFriction(const UrdfLink &link, const QString &indent) {
  if (link.gazeboMu < 0.0 || link.collisions.isEmpty())
    return QString();
  return indent + QString("newtonFriction %1\n").arg(link.gazeboMu);
}

QString emitLinkPhysics(const UrdfLink &link, const QString &indent, bool allowSyntheticPhysics = true,
                        const UrdfOrigin *linkOffset = nullptr, bool bodyForFixedSubtree = false) {
  QString out;

  // Collect collisions with emittable geometry (supported primitives or resolved meshes).
  QList<UrdfCollision> supported;
  for (const UrdfCollision &c : link.collisions) {
    if (isSupportedGeometry(c.geometry))
      supported.append(c);
  }

  if (supported.size() == 1) {
    out += emitBoundingObject(supported.first(), indent, link.name);
  } else if (supported.size() > 1) {
    out += emitBoundingObjectGroup(supported, indent, link.name);
  } else if (link.inertial.present || (bodyForFixedSubtree && allowSyntheticPhysics)) {
    // OmniSim needs a boundingObject to compute an inertia matrix; a link with
    // <inertial> but no <collision> would otherwise trigger "Undefined inertia
    // matrix" on every load. Emit a 10 mm placeholder sphere -- big enough
    // that the auto-derived inertia tensor isn't degenerate (1 mm sphere
    // produced ~1e-10 inertia, which appears to leave the merged root body
    // unable to receive friction-induced acceleration on small robots).
    // linkOffset (a re-rooted Robot link, emitRobot): keep the placeholder at
    // the LINK origin, not at the Robot's base_footprint frame on the floor.
    // DEF URDF_INERTIA_PLACEHOLDER (2026-10-06): the engine registers this sphere
    // NON-colliding (OmSolid.cpp quietIfUrdfInertiaPlaceholder). URDF gives a link
    // with no <collision> no contact; the sphere used to collide as a 1 cm marble
    // at every such link origin (LimX HU_D04, TRON1).
    if (linkOffset != nullptr && !isIdentityUrdfOrigin(*linkOffset))
      out += indent + QString("boundingObject Pose { translation %1 %2 %3 children [ DEF URDF_INERTIA_PLACEHOLDER "
                              "Sphere { radius 0.01 } ] }\n")
                        .arg(linkOffset->x).arg(linkOffset->y).arg(linkOffset->z);
    else
      out += indent + "boundingObject DEF URDF_INERTIA_PLACEHOLDER Sphere { radius 0.01 }\n";
  }
  // OmniSim treats a Solid with boundingObject but no Physics as unable to
  // participate in collisions ("collisions will have no effect"). Many ROS
  // URDFs (e.g. Husky's top_plate_link) declare collision geometry without
  // <inertial> because the real mass lives on a sibling. Synthesize a
  // near-zero-mass Physics so contacts register; the descendant inertial
  // distribution still dominates dynamics.
  //
  // BUT: for fixed-joint child links (allowSyntheticPhysics=false), suppress
  // the synthesis. A fixed joint means the child is rigidly part of the
  // parent. Giving it its own dynamic Physics block with a tiny mass leaves
  // the ODE constraint to fight gravity, and the constraint loses — the
  // child drifts out of position (visible on Panda's hand and fingers, on
  // PR2's tucked arms, etc.). When no synthetic Physics is emitted, the
  // child becomes a passive kinematic offset of the parent — exactly the
  // welded-on-rigidly semantics a fixed joint should produce. The cost is
  // that the link's collisions won't generate contacts; for purely visual
  // attachments (gripper hands, sensor mounts, antennas) that's the right
  // trade-off. Links that need real contact dynamics should declare an
  // <inertial> in the URDF or be connected via a non-fixed joint.
  // The same near-zero synthetic mass (1 g) carries an empty moving-joint link
  // whose fixed subtree has mass or collision (bodyForFixedSubtree, above): the
  // subtree's own mass then merges into it, as MuJoCo's URDF fusing does.
  const bool needsSyntheticPhysics =
    allowSyntheticPhysics && !link.inertial.present && (!supported.isEmpty() || bodyForFixedSubtree);
  if (link.inertial.present || needsSyntheticPhysics) {
    const double mass = link.inertial.present ? link.inertial.mass : 0.001;
    QString physicsBody;
    physicsBody += indent + "  density -1\n";
    physicsBody += indent + QString("  mass %1\n").arg(mass);
    // OmniSim refuses the custom inertiaMatrix unless mass > 0 AND a single-line
    // centerOfMass is present. So when we have a URDF tensor, emit the COM even
    // when the URDF origin is the link origin (0 0 0) — otherwise the tensor
    // would be silently ignored.
    //
    // Also clamp out vanishingly-small tensors. The threshold USED to be 1e-4,
    // because ODE's dMassSetParameters was observed to crash below roughly
    // that even on tensors that satisfy positive-definiteness and the triangle
    // inequality. ODE is deleted; MuJoCo's own floor is mjMINVAL (1e-15), so a
    // 1e-4 clamp is no longer a crash guard -- it is a second silent mask on
    // top of the one this file just removed. A 100 g gripper finger's real
    // tensor is ~1e-5 and would have been thrown away. The clamp survives only
    // as a numerical-sanity floor at 1e-9, and dropping a tensor now WARNS
    // instead of doing it quietly.
    constexpr double kMinInertia = 1e-9;
    const bool tensorTooSmall = link.inertial.hasInertiaMatrix
                                && (link.inertial.ixx < kMinInertia
                                    || link.inertial.iyy < kMinInertia
                                    || link.inertial.izz < kMinInertia);
    // A tensor can be positive definite (the gate parseLink already applied)
    // and still describe no physical body. Now that the tensor actually reaches
    // the solver, MuJoCo's compiler would REJECT such a body outright -- so
    // catch it here and fall back to the bounding object, loudly.
    double moments[3] = {0.0, 0.0, 0.0};
    const bool tensorImpossible =
      link.inertial.hasInertiaMatrix && !tensorTooSmall &&
      !urdfInertiaSatisfiesTriangleInequality(link.inertial.ixx, link.inertial.ixy,
                                              link.inertial.ixz, link.inertial.iyy,
                                              link.inertial.iyz, link.inertial.izz, moments);
    // The fallback replaces ONLY the rotational inertia: the declared mass and
    // centre of mass are kept, and the marked block tells the Newton registration
    // to take the tensor from the bounding object ABOUT that centre of mass (see
    // urdfInertiaFallbackKeepsCom).
    // A tensor parseLink already refused as not positive definite (an all-zero one
    // included) is the third rejection, and drops the COM the same way (2026-10-20).
    const bool tensorNotPd = link.inertial.present && link.inertial.declaredNotPositiveDefinite;
    const bool tensorRejected =
      link.inertial.present && urdfUseInertiaEnabled() && mass > 0.0 &&
      ((link.inertial.hasInertiaMatrix && (tensorTooSmall || tensorImpossible)) || tensorNotPd);
    const bool markFallback = tensorRejected && urdfInertiaFallbackKeepsCom();
    // Not rejected (no mass, or OMNISIM_URDF_USE_INERTIA=0): the warning parseLink
    // used to write, and still only for a tensor that is not all zero -- an all-zero
    // tensor on a massless frame is a normal URDF idiom.
    if (tensorNotPd && !tensorRejected && link.inertial.declaredNonZeroDiagonal)
      OmLog::warning(QObject::tr("URDF link '%1': inertia tensor is not positive definite (%2); "
                                 "falling back to bounding-object inertia.")
                       .arg(link.name).arg(link.inertial.declaredTensorText),
                     false, OmLog::PARSING);
    if (tensorRejected) {
      const QString consequence =
        markFallback
          ? QObject::tr("kept the declared mass (%1 kg) and centre of mass (%2 %3 %4); replaced only the "
                        "rotational inertia, with the bounding object's tensor scaled to that mass and taken "
                        "about that centre of mass.")
              .arg(mass).arg(link.inertial.origin.x).arg(link.inertial.origin.y).arg(link.inertial.origin.z)
          : QObject::tr("kept the declared mass (%1 kg); replaced the rotational inertia with the bounding "
                        "object's AND dropped the declared centre of mass (%2 %3 %4) to the link origin "
                        "(OMNISIM_URDF_INERTIA_FALLBACK_KEEP_COM=0).")
              .arg(mass).arg(link.inertial.origin.x).arg(link.inertial.origin.y).arg(link.inertial.origin.z);
      if (tensorNotPd)
        OmLog::warning(QObject::tr("URDF link '%1': inertia tensor is not positive definite (%2); "
                                   "falling back to bounding-object inertia: %3")
                         .arg(link.name).arg(link.inertial.declaredTensorText).arg(consequence),
                       false, OmLog::PARSING);
      else if (tensorTooSmall)
        OmLog::warning(QObject::tr("URDF link '%1': inertia tensor has a principal moment below "
                                   "%2 kg.m^2 (%3 %4 %5); falling back to bounding-object inertia: %6")
                         .arg(link.name).arg(kMinInertia)
                         .arg(link.inertial.ixx).arg(link.inertial.iyy).arg(link.inertial.izz)
                         .arg(consequence));
      else
        OmLog::warning(QObject::tr("URDF link '%1': inertia tensor violates the triangle inequality "
                                   "for principal moments (a+b >= c): principal moments are "
                                   "(%2 %3 %4). No rigid body has this tensor, and MuJoCo would "
                                   "refuse the model; falling back to bounding-object inertia: %5")
                         .arg(link.name).arg(moments[0]).arg(moments[1]).arg(moments[2])
                         .arg(consequence));
    }
    const bool hasUrdfInertia = link.inertial.present && link.inertial.hasInertiaMatrix && mass > 0.0
                                && !tensorTooSmall && !tensorImpossible && urdfUseInertiaEnabled();
    const bool hasComOffset = link.inertial.present &&
                              (link.inertial.origin.x != 0.0 || link.inertial.origin.y != 0.0 ||
                               link.inertial.origin.z != 0.0);
    if (hasUrdfInertia || hasComOffset || markFallback) {
      physicsBody += indent + QString("  centerOfMass [%1 %2 %3]\n")
                                .arg(link.inertial.origin.x)
                                .arg(link.inertial.origin.y)
                                .arg(link.inertial.origin.z);
    }
    if (hasUrdfInertia) {
      // OmniSim inertiaMatrix layout: line 1 = [Ixx Iyy Izz], line 2 = [Ixy Ixz Iyz].
      // Same sign convention as URDF — both store the un-negated integrals.
      physicsBody += indent + QString("  inertiaMatrix [\n");
      physicsBody +=
        indent + QString("    %1 %2 %3\n").arg(link.inertial.ixx).arg(link.inertial.iyy).arg(link.inertial.izz);
      physicsBody +=
        indent + QString("    %1 %2 %3\n").arg(link.inertial.ixy).arg(link.inertial.ixz).arg(link.inertial.iyz);
      physicsBody += indent + QString("  ]\n");
    }
    out += indent + (markFallback ? "physics DEF URDF_INERTIA_FALLBACK Physics {\n" : "physics Physics {\n");
    out += physicsBody;
    out += indent + "}\n";
  }
  return out;
}

// The joint's initial position, when the importer gives it one: the URDF
// <rest> value, else the midpoint of a revolute range that excludes 0, else
// the nearest stop of a prismatic range that excludes 0. ONE rule for both
// the HingeJointParameters `position` line and the endPoint pose emitted by
// emitSolidEndPoint, so the two can never disagree: the endPoint is written
// ALREADY POSED at this angle (Webots semantics -- the endPoint fields are the
// pose AT `position`), and the Newton bridge starts the joint there
// (OmBasicJoint::flushPendingNewtonRegistrations, 2026-09-08). Before that the
// endPoint was written at the URDF zero pose, so the scene, the joint's own
// zero-pose bookkeeping and the first physics step all disagreed.
//
// A <rest> is the author's explicit start angle, so it wins on EVERY revolute
// or continuous joint, whatever its range. Until 2026-10-23 it was read only
// after the range checks below, so a joint whose limits cover a full turn
// (+/-3.14 or wider: the `full revolution` early return), a continuous joint,
// a revolute with incomplete limits, or a range wholly past +/-pi started at
// 0 with nothing logged (Ti5 T170 shoulders at +/-3.14 with <rest>: q_start
// 0.0, while +/-3.13 started at the rest angle). Nothing stops it being
// applied: the endPoint is posed by emitSolidEndPoint, the `position` line
// needs no stops, and the Newton bridge starts the joint there. Prismatic
// joints keep their own rule and do not read <rest>.
bool initialJointPosition(const UrdfJoint &joint, double &q0) {
  q0 = 0.0;
  if (joint.type == "prismatic") {
    if (joint.hasLower && joint.hasUpper && joint.upper > joint.lower &&
        (joint.lower > 0.0 || joint.upper < 0.0)) {
      q0 = joint.lower > 0.0 ? joint.lower : joint.upper;
      return true;
    }
    return false;
  }
  if (joint.type != "revolute" && joint.type != "continuous")
    return false;
  if (joint.hasRest) {
    q0 = joint.rest;
    return true;
  }
  if (joint.type != "revolute" || !joint.hasLower || !joint.hasUpper)
    return false;
  const double kPiLimit = M_PI - 0.01;
  if (joint.lower <= -kPiLimit && joint.upper >= kPiLimit)
    return false;  // full revolution: no stops, no seeded midpoint
  const double lo = std::max(-kPiLimit, std::min(kPiLimit, joint.lower));
  const double hi = std::max(-kPiLimit, std::min(kPiLimit, joint.upper));
  if (hi <= lo)
    return false;
  if (lo > 0.0 || hi < 0.0) {
    q0 = 0.5 * (lo + hi);
    return true;
  }
  return false;
}

// Axis-angle of a rotation matrix (the extraction rpyToAxisAngle does after
// building its matrix, kept separate so a composed matrix can use it).
void matrixToAxisAngle(double m00, double m01, double m02, double m10, double m11, double m12,
                       double m20, double m21, double m22, double &ax, double &ay, double &az, double &angle) {
  const double trace = m00 + m11 + m22;
  double cosAngle = (trace - 1.0) / 2.0;
  if (cosAngle > 1.0)
    cosAngle = 1.0;
  if (cosAngle < -1.0)
    cosAngle = -1.0;
  angle = std::acos(cosAngle);
  if (std::abs(angle) < 1e-9) {
    ax = 0.0;
    ay = 0.0;
    az = 1.0;
    angle = 0.0;
    return;
  }
  if (std::abs(angle - M_PI) < 1e-6) {
    if (m00 > m11 && m00 > m22) {
      ax = std::sqrt(std::max(0.0, m00 - m11 - m22 + 1.0)) / 2.0;
      ay = ax > 0 ? m01 / (2.0 * ax) : 0.0;
      az = ax > 0 ? m02 / (2.0 * ax) : 0.0;
    } else if (m11 > m22) {
      ay = std::sqrt(std::max(0.0, m11 - m00 - m22 + 1.0)) / 2.0;
      ax = ay > 0 ? m01 / (2.0 * ay) : 0.0;
      az = ay > 0 ? m12 / (2.0 * ay) : 0.0;
    } else {
      az = std::sqrt(std::max(0.0, m22 - m00 - m11 + 1.0)) / 2.0;
      ax = az > 0 ? m02 / (2.0 * az) : 0.0;
      ay = az > 0 ? m12 / (2.0 * az) : 0.0;
    }
    angle = M_PI;
    return;
  }
  const double s = 2.0 * std::sin(angle);
  ax = (m21 - m12) / s;
  ay = (m02 - m20) / s;
  az = (m10 - m01) / s;
}

// Rotation matrix of `angle` about the unit axis (ux, uy, uz) -- Rodrigues.
void axisAngleToMatrix(double ux, double uy, double uz, double angle,
                       double &m00, double &m01, double &m02, double &m10, double &m11, double &m12,
                       double &m20, double &m21, double &m22) {
  const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
  m00 = c + ux * ux * t;      m01 = ux * uy * t - uz * s; m02 = ux * uz * t + uy * s;
  m10 = uy * ux * t + uz * s; m11 = c + uy * uy * t;      m12 = uy * uz * t - ux * s;
  m20 = uz * ux * t - uy * s; m21 = uz * uy * t + ux * s; m22 = c + uz * uz * t;
}

void appendJointPhysicsParameters(QString &out, const UrdfJoint &joint, const QString &indent) {
  // PRISMATIC joints: emit the stops in METRES, with none of the angular
  // clamping below -- a linear travel of 0.0425 m has nothing to do with pi.
  //
  // ⚠ This case used to fall through and emit NOTHING, so every URDF-imported
  // prismatic joint reached the solver unlimited. Measured on the Robotiq 2F-85
  // fingers (omniarm6_2f85_grip.urdf declares lower="-0.03" upper="0.0425"): the
  // compiled mjModel carried `limited=0, range=[-1e10, 1e10]`. Combined with
  // the two pads being mutually collision-filtered, nothing stopped the fingers
  // closing straight through each other, and the released finger crept open
  // monotonically over 60k steps instead of stopping at its limit.
  if (joint.type == "prismatic" && joint.hasLower && joint.hasUpper &&
      joint.upper > joint.lower) {
    out += indent + QString("minStop %1\n").arg(joint.lower);
    out += indent + QString("maxStop %1\n").arg(joint.upper);
    double q0 = 0.0;
    if (initialJointPosition(joint, q0)) {
      // Same reasoning as the revolute case: the joint would otherwise start
      // outside its own stops, which the engine warns about on every load.
      out += indent + QString("position %1\n").arg(q0);
    }
    return;
  }
  // Only emit minStop/maxStop for revolute joints. Continuous joints have no
  // limits by definition. OmniSim also requires the stops to be strictly
  // inside (-pi, pi), so clamp with a small margin.
  if (joint.type == "revolute" && joint.hasLower && joint.hasUpper) {
    const double kPiLimit = M_PI - 0.01;  // leave some slack for the <= pi check
    // Skip entirely if the URDF limits already cover a full revolution.
    const bool fullRange = (joint.lower <= -kPiLimit && joint.upper >= kPiLimit);
    // A range that reaches past +/-pi at ONE end only used to be CLAMPED to the
    // representable window, which silently deleted real travel: a KUKA KR6 R900
    // sixx A2 (-190..+45 deg) stopped 10.573 deg short of its own declared
    // limit, a -6.283..+0.785 joint lost 3.151 rad, and 59 joints across the
    // shipped robots were affected -- the DR02 humanoids' shoulders by 40.6 deg
    // each. Nothing was logged and the headless run still PASSed.
    //
    // minStop/maxStop cannot express it, so do what the full-range case already
    // does: skip the stops and let the motor's minPosition/maxPosition below
    // carry the declared range. That is NOT a weaker form of enforcement --
    // both routes arrive at the solver as the same joint limit (verified in the
    // "hinge joint N ... lim=[lo, hi]" backend line), so the joint is held to
    // its authored range either way. No warning for that reason.
    const bool partialRange = !fullRange && (joint.lower < -kPiLimit || joint.upper > kPiLimit);
    if (!fullRange && !partialRange) {
      const double lo = std::max(-kPiLimit, std::min(kPiLimit, joint.lower));
      const double hi = std::max(-kPiLimit, std::min(kPiLimit, joint.upper));
      if (hi > lo) {
        out += indent + QString("minStop %1\n").arg(lo);
        out += indent + QString("maxStop %1\n").arg(hi);
        // OmniSim defaults the initial position to 0. If the clamped range
        // excludes 0, the joint starts outside its stops and OmniSim warns
        // 'maxStop must be >= position' (panda_joint4 is the canonical case:
        // URDF limits [-3.07, -0.07]). Seed the midpoint instead.
        //
        // Also honor an optional `<rest>NUMBER</rest>` child tag inside
        // <joint>: if present, this is the joint's initial position
        // regardless of where 0 falls in the limits. Used by widened
        // URDFs that need a wider range than the original but want to
        // preserve the original spawn pose -- e.g. Spot's hip_y was
        // limit [0.001, 0.60] (spawn midpoint 0.30) and is now widened
        // to [-2.50, 3.00] with <rest>0.30</rest> to keep walking.
        // (The `position` line itself is written below, outside this branch.)
      }
    }
    // initialJointPosition is the single rule shared with emitSolidEndPoint,
    // which writes the endPoint already posed at this angle, so the joint MUST
    // be told it starts there whenever the endPoint was posed -- including the
    // partial-range case, which emits no stops. Until 2026-10-07 this line sat
    // inside the stops branch: a <rest> on a joint reaching past +/-pi (UFACTORY
    // xArm joint3 -3.927..0.192) spawned the link at the rest angle while the
    // joint read 0 there, so every setPosition() landed `rest` radians off and
    // the xArm was driven 20 deg past its own stop. The full-range case is no
    // exception since 2026-10-23: initialJointPosition returns a <rest> there
    // too, and the endPoint is posed at it -- so the line moved out of this
    // branch, below, where a continuous joint reaches it as well.
    if (joint.hasRest && (joint.rest < joint.lower || joint.rest > joint.upper))
      OmLog::warning(QObject::tr("URDF joint '%1': <rest>%2</rest> lies outside its own limits [%3, %4], so "
                                 "the joint starts outside its range. Move the rest angle inside it.")
                       .arg(joint.name)
                       .arg(joint.rest)
                       .arg(joint.lower)
                       .arg(joint.upper));
  }
  // The `position` line, for every revolute or continuous joint the importer
  // gives a start angle. A continuous joint has no stops, but a <rest> on it
  // poses its endPoint all the same, so it needs the line as much as a limited
  // joint does.
  if (joint.type == "revolute" || joint.type == "continuous") {
    double q0 = 0.0;
    if (initialJointPosition(joint, q0))
      out += indent + QString("position %1\n").arg(q0);
  }
  if (joint.hasDamping)
    out += indent + QString("dampingConstant %1\n").arg(joint.damping);
  if (joint.hasFriction)
    out += indent + QString("staticFriction %1\n").arg(joint.friction);
}

void logJointDebugInfo(const UrdfJoint &joint, double parentAx, double parentAy, double parentAz) {
  if (!urdfDebugEnabled())
    return;

  QString message = QObject::tr("URDF_DEBUG joint '%1': parent='%2' child='%3' origin_xyz=[%4] origin_rpy=[%5] "
                                "axis_joint=[%6] axis_parent=[%7]");
  message = message.arg(joint.name);
  message = message.arg(joint.parent);
  message = message.arg(joint.child);
  message = message.arg(vectorString(joint.origin.x, joint.origin.y, joint.origin.z));
  message = message.arg(vectorString(joint.origin.roll, joint.origin.pitch, joint.origin.yaw));
  message = message.arg(vectorString(joint.ax, joint.ay, joint.az));
  message = message.arg(vectorString(parentAx, parentAy, parentAz));
  if (joint.hasLower && joint.hasUpper)
    message += QObject::tr(" limits=[%1,%2]").arg(joint.lower).arg(joint.upper);
  if (joint.hasVelocity)
    message += QObject::tr(" velocity=%1").arg(joint.velocity);
  if (joint.hasDamping)
    message += QObject::tr(" damping=%1").arg(joint.damping);
  if (joint.hasFriction)
    message += QObject::tr(" friction=%1").arg(joint.friction);
  OmLog::info(message, false, OmLog::PARSING);
}

QString emitJoint(const UrdfJoint &j, const QString &indent,
                  const QHash<QString, QList<UrdfJoint>> &jointsByParent,
                  const QHash<QString, UrdfLink> &links,
                  const QHash<QString, QList<UrdfSensor>> &sensorsByLink);
static void appendMimicFields(QString &out, const UrdfJoint &j, const QString &indent);

// TRUE when `link` declares neither <inertial> nor a supported <collision> but a
// link welded below it by fixed joints (any depth) declares either -- the case
// emitLinkPhysics' `bodyForFixedSubtree` exists for. Value-parsed hatch:
// OMNISIM_URDF_EMPTY_LINK_BODY=0 restores the old import (no body; the fixed
// subtree comes loose).
static bool emptyLinkCarriesFixedSubtree(const UrdfLink &link,
                                         const QHash<QString, QList<UrdfJoint>> &jointsByParent,
                                         const QHash<QString, UrdfLink> &links) {
  static const bool enabled = [] {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_URDF_EMPTY_LINK_BODY")).trimmed().toLower();
    return v.isEmpty() || (v != "0" && v != "false" && v != "off" && v != "no");
  }();
  if (!enabled || link.inertial.present)
    return false;
  const auto isPhysical = [](const UrdfLink &l) {
    if (l.inertial.present)
      return true;
    for (const UrdfCollision &c : l.collisions)
      if (isSupportedGeometry(c.geometry))
        return true;
    return false;
  };
  if (isPhysical(link))
    return false;  // has its own collision: the ordinary synthetic-physics path covers it
  QList<QString> stack{link.name};
  QSet<QString> seen;
  while (!stack.isEmpty()) {
    const QString name = stack.takeLast();
    if (seen.contains(name))
      continue;
    seen.insert(name);
    for (const UrdfJoint &cj : jointsByParent.value(name)) {
      if (cj.type != "fixed" || !links.contains(cj.child))
        continue;
      if (isPhysical(links.value(cj.child)))
        return true;
      stack.append(cj.child);
    }
  }
  return false;
}

QString emitSolidEndPoint(const UrdfLink &link, const UrdfJoint &joint, const QString &indent,
                          const QHash<QString, QList<UrdfJoint>> &jointsByParent,
                          const QHash<QString, UrdfLink> &links,
                          const QHash<QString, QList<UrdfSensor>> &sensorsByLink) {
  QString out = indent + "endPoint Solid {\n";
  // The endPoint is written AT the joint's initial position (Webots semantics:
  // the endPoint fields are the pose at `position`, and the joint's zero-pose
  // bookkeeping un-rotates them by it). For a revolute joint that is the URDF
  // child frame rotated about the joint axis (given in that same child frame)
  // by q0; for a prismatic joint it is the child frame translated along the
  // axis by q0. The joint anchor coincides with the child origin in URDF, so
  // a revolute pose keeps the translation.
  double q0 = 0.0;
  const bool posed = initialJointPosition(joint, q0) && q0 != 0.0;
  double m00, m01, m02, m10, m11, m12, m20, m21, m22;
  rpyToMatrix(joint.origin.roll, joint.origin.pitch, joint.origin.yaw, m00, m01, m02, m10, m11, m12, m20, m21, m22);
  double tx = joint.origin.x, ty = joint.origin.y, tz = joint.origin.z;
  if (posed) {
    const double n = std::sqrt(joint.ax * joint.ax + joint.ay * joint.ay + joint.az * joint.az);
    const double ux = n > 0.0 ? joint.ax / n : 0.0, uy = n > 0.0 ? joint.ay / n : 0.0, uz = n > 0.0 ? joint.az / n : 1.0;
    if (joint.type == "prismatic") {
      // R_rpy * (axis * q0), added to the child origin.
      tx += (m00 * ux + m01 * uy + m02 * uz) * q0;
      ty += (m10 * ux + m11 * uy + m12 * uz) * q0;
      tz += (m20 * ux + m21 * uy + m22 * uz) * q0;
    } else {
      // R = R_rpy * R(axis_child, q0)
      double a00, a01, a02, a10, a11, a12, a20, a21, a22;
      axisAngleToMatrix(ux, uy, uz, q0, a00, a01, a02, a10, a11, a12, a20, a21, a22);
      const double r00 = m00 * a00 + m01 * a10 + m02 * a20, r01 = m00 * a01 + m01 * a11 + m02 * a21, r02 = m00 * a02 + m01 * a12 + m02 * a22;
      const double r10 = m10 * a00 + m11 * a10 + m12 * a20, r11 = m10 * a01 + m11 * a11 + m12 * a21, r12 = m10 * a02 + m11 * a12 + m12 * a22;
      const double r20 = m20 * a00 + m21 * a10 + m22 * a20, r21 = m20 * a01 + m21 * a11 + m22 * a21, r22 = m20 * a02 + m21 * a12 + m22 * a22;
      m00 = r00; m01 = r01; m02 = r02; m10 = r10; m11 = r11; m12 = r12; m20 = r20; m21 = r21; m22 = r22;
    }
  }
  out += indent + QString("  translation %1 %2 %3\n").arg(tx).arg(ty).arg(tz);
  double ax, ay, az, ang;
  matrixToAxisAngle(m00, m01, m02, m10, m11, m12, m20, m21, m22, ax, ay, az, ang);
  if (ang != 0.0)
    out += indent + QString("  rotation %1 %2 %3 %4\n").arg(ax).arg(ay).arg(az).arg(ang);
  out += indent + QString("  name \"%1\"\n").arg(link.name);
  out += indent + "  children [\n";
  out += emitLinkChildren(link, indent + "    ", sensorsByLink);
  if (jointsByParent.contains(link.name)) {
    for (const UrdfJoint &cj : jointsByParent.value(link.name))
      out += emitJoint(cj, indent + "    ", jointsByParent, links, sensorsByLink);
  }
  out += indent + "  ]\n";
  out += emitLinkPhysics(link, indent + "  ", true, nullptr,
                         emptyLinkCarriesFixedSubtree(link, jointsByParent, links));
  out += emitLinkFriction(link, indent + "  ");
  out += indent + "}\n";
  return out;
}

QString emitJoint(const UrdfJoint &j, const QString &indent,
                  const QHash<QString, QList<UrdfJoint>> &jointsByParent,
                  const QHash<QString, UrdfLink> &links,
                  const QHash<QString, QList<UrdfSensor>> &sensorsByLink) {
  if (!links.contains(j.child))
    return QString();
  const UrdfLink &child = links.value(j.child);
  double parentAx = j.ax;
  double parentAy = j.ay;
  double parentAz = j.az;
  jointAxisInParentFrame(j, parentAx, parentAy, parentAz);
  logJointDebugInfo(j, parentAx, parentAy, parentAz);

  if (j.type == "fixed") {
    QString out = indent + "Solid {\n";
    out += indent + QString("  translation %1 %2 %3\n").arg(j.origin.x).arg(j.origin.y).arg(j.origin.z);
    double ax, ay, az, ang;
    rpyToAxisAngle(j.origin.roll, j.origin.pitch, j.origin.yaw, ax, ay, az, ang);
    if (ang != 0.0)
      out += indent + QString("  rotation %1 %2 %3 %4\n").arg(ax).arg(ay).arg(az).arg(ang);
    out += indent + QString("  name \"%1\"\n").arg(child.name);
    out += indent + "  children [\n";
    out += emitLinkChildren(child, indent + "    ", sensorsByLink);
    if (jointsByParent.contains(child.name)) {
      for (const UrdfJoint &cj : jointsByParent.value(child.name))
        out += emitJoint(cj, indent + "    ", jointsByParent, links, sensorsByLink);
    }
    out += indent + "  ]\n";
    // allowSyntheticPhysics=false: a fixed joint means the child is
    // rigidly welded to the parent. Suppress the auto-Physics we'd
    // otherwise emit for collision-only links — that synthetic mass
    // makes ODE fight the fixed-joint constraint under gravity and the
    // child drifts out of position (Panda hand falling off the flange).
    out += emitLinkPhysics(child, indent + "  ", /*allowSyntheticPhysics=*/false);
    out += emitLinkFriction(child, indent + "  ");
    out += indent + "}\n";
    return out;
  }

  if (j.type == "revolute" || j.type == "continuous") {
    QString out = indent + "HingeJoint {\n";
    out += indent + "  jointParameters HingeJointParameters {\n";
    out += indent + QString("    axis %1 %2 %3\n").arg(parentAx).arg(parentAy).arg(parentAz);
    out += indent + QString("    anchor %1 %2 %3\n").arg(j.origin.x).arg(j.origin.y).arg(j.origin.z);
    appendJointPhysicsParameters(out, j, indent + "    ");
    out += indent + "  }\n";
    out += indent + "  device [\n";
    out += indent + "    RotationalMotor {\n";
    out += indent + QString("      name \"%1_motor\"\n").arg(j.name);
    out += indent + QString("      maxVelocity %1\n").arg(j.hasVelocity ? j.velocity : 10.0);
    if (j.hasEffort)
      out += indent + QString("      maxTorque %1\n").arg(j.effort);
    // Full-range revolute joints (URDF |limit| >= pi, e.g. manipulator
    // arm joints with +/-2pi range) have their +/-pi-clamped joint stops
    // skipped by appendJointPhysicsParameters (OmniSim requires stops strictly
    // inside (-pi, pi)). Under Newton that left them with NO limit signal, so
    // the ke/kd discriminator (OmBasicJoint::flushPendingNewtonRegistrations)
    // mislabeled them velocity-driven wheels (kp=0) and they could not hold a
    // setpoint. Record the URDF limits as the MOTOR's control min/maxPosition
    // so the discriminator reads them as a position-controlled limb (kp=20).
    // `continuous` joints (true free-spinning wheels) intentionally get none.
    // On the ODE path this only clamps setPosition() to the joint's own +/-2pi
    // range, which a sane controller never commands past -- so existing ODE
    // behavior is unchanged in practice (the joint mechanical stops are still
    // absent, leaving it free to rotate).
    // Emitted for EVERY limited revolute joint since 2026-09-20, not only the
    // full-range ones. Restricting it to full-range left an ordinary joint --
    // anything inside +/-pi, which is most of them -- enforcing its limits
    // through the stops while reporting NONE: Motor.getMinPosition() and
    // getMaxPosition() both returned 0, so an agent or a controller reading the
    // motor to discover the joint's envelope was told it could not move at all.
    // Reported independently by two users the same week, on a KUKA KR6 and a
    // Galbot R1. The stops still do the mechanical enforcement where they can;
    // this is what makes the joint able to DESCRIBE itself, and for the
    // partial-range case above it is also the only thing carrying the limit.
    if (j.type == "revolute" && j.hasLower && j.hasUpper && j.upper > j.lower) {
      out += indent + QString("      minPosition %1\n").arg(j.lower);
      out += indent + QString("      maxPosition %1\n").arg(j.upper);
    }
    appendMimicFields(out, j, indent + "      ");
    out += indent + "    }\n";
    // Pair the motor with a PositionSensor so controllers can read joint
    // angle / velocity (motor.getPositionSensor() returns this). Without it
    // every URDF-imported revolute or continuous joint is write-only.
    out += indent + "    PositionSensor {\n";
    out += indent + QString("      name \"%1_sensor\"\n").arg(j.name);
    out += indent + "    }\n";
    out += indent + "  ]\n";
    out += emitSolidEndPoint(child, j, indent + "  ", jointsByParent, links, sensorsByLink);
    out += indent + "}\n";
    return out;
  }

  if (j.type == "prismatic") {
    QString out = indent + "SliderJoint {\n";
    out += indent + "  jointParameters JointParameters {\n";
    out += indent + QString("    axis %1 %2 %3\n").arg(parentAx).arg(parentAy).arg(parentAz);
    appendJointPhysicsParameters(out, j, indent + "    ");
    out += indent + "  }\n";
    out += indent + "  device [\n";
    out += indent + "    LinearMotor {\n";
    out += indent + QString("      name \"%1_motor\"\n").arg(j.name);
    if (j.hasVelocity)
      out += indent + QString("      maxVelocity %1\n").arg(j.velocity);
    if (j.hasEffort)
      out += indent + QString("      maxForce %1\n").arg(j.effort);
    // The travel as the motor's control range too, as for revolute joints
    // (2026-09-20). Without it LinearMotor.getMinPosition()/getMaxPosition()
    // read 0/0 while the stops enforced the URDF range (Niryo Ned2 and Seeed
    // reBot grippers, 2026-10-04): a controller reading the motor to find the
    // finger's travel was told it could not move. The solver limit is
    // unchanged -- OmBasicJoint reads the motor range first, the stops second,
    // and both now carry the same values.
    if (j.hasLower && j.hasUpper && j.upper > j.lower) {
      out += indent + QString("      minPosition %1\n").arg(j.lower);
      out += indent + QString("      maxPosition %1\n").arg(j.upper);
    }
    appendMimicFields(out, j, indent + "      ");
    out += indent + "    }\n";
    out += indent + "    PositionSensor {\n";
    out += indent + QString("      name \"%1_sensor\"\n").arg(j.name);
    out += indent + "    }\n";
    out += indent + "  ]\n";
    out += emitSolidEndPoint(child, j, indent + "  ", jointsByParent, links, sensorsByLink);
    out += indent + "}\n";
    return out;
  }

  return QString();
}

// URDF <mimic> (2026-10-06). Until then the importer dropped it: a mimic-driven
// gripper (Robotiq 2F-85/2F-140, Franka Hand, Kinova, LimX, Dexmate ...) arrived
// as independent motors, the follower fingers free or held only by their own
// servo, so they drifted apart under asymmetric load. The follower's motor now
// names its leader; the engine enforces q = offset + multiplier * q_leader as a
// MuJoCo joint-equality constraint (the standard MuJoCo encoding of a mimic:
// <equality><joint polycoef="offset multiplier 0 0 0">; MuJoCo 3.11's own URDF
// import drops <mimic> entirely) and registers the follower without a drive. OMNISIM_URDF_MIMIC=0 (value-parsed)
// restores the old import.
static void appendMimicFields(QString &out, const UrdfJoint &j, const QString &indent) {
  static const bool enabled = [] {
    const QString v = QString::fromUtf8(qgetenv("OMNISIM_URDF_MIMIC")).trimmed().toLower();
    return v.isEmpty() || (v != "0" && v != "false" && v != "off" && v != "no");
  }();
  if (!enabled || j.mimicJoint.isEmpty())
    return;
  out += indent + QString("mimicMotor \"%1_motor\"\n").arg(j.mimicJoint);
  out += indent + QString("mimicMultiplier %1\n").arg(j.mimicMultiplier, 0, 'g', 17);
  out += indent + QString("mimicOffset %1\n").arg(j.mimicOffset, 0, 'g', 17);
}

QString emitRobot(const UrdfRobot &robot) {
  // Find root link (one not appearing as the child of any joint)
  QStringList childLinks;
  for (const UrdfJoint &j : robot.joints)
    childLinks.append(j.child);

  QStringList rootCandidates;
  for (const QString &linkName : robot.linkOrder) {
    if (!childLinks.contains(linkName))
      rootCandidates.append(linkName);
  }
  QString rootName = rootCandidates.isEmpty() ? QString() : rootCandidates.first();
  if (rootName.isEmpty()) {
    OmLog::error(QObject::tr("URDF '%1': no root link found").arg(robot.name));
    return QString();
  }
  if (rootCandidates.size() > 1) {
    OmLog::warning(QObject::tr("URDF '%1': multiple root links found (%2). Using '%3'.")
                     .arg(robot.name)
                     .arg(rootCandidates.join(", "))
                     .arg(rootName),
                   false, OmLog::PARSING);
  }

  // Re-anchor at first non-empty child when the URDF root is a pure-frame
  // link (no visuals, no collisions, no inertial) connected to its first
  // child by a fixed joint. ROS URDFs use this pattern with base_footprint
  // as a virtual ground-plane frame, and the synthetic Physics + tiny
  // placeholder bounding object that an empty root forces the importer to
  // emit produced a body that wouldn't accelerate from wheel-friction
  // forces (verified on TurtleBot3 burger: wheels rotated at commanded
  // speed but the chassis stayed at (0,0,0)). Promoting the first real
  // child to the Robot's root puts the chassis body directly under the
  // Robot node, no merger gymnastics required.
  //
  // The skipped fixed joints' offset (`rerootOffset`, composed down the chain)
  // used to be DROPPED here, which moved every child joint of the promoted link
  // by it: on a TurtleBot3 Burger the wheel axles landed at z 0.023 in the Robot
  // frame instead of 0.033 (base_joint is +0.010), so a robot placed at
  // `translation ... 0` spawned with its wheels 10 mm into the floor and its
  // frame then rode 10 mm above the `translation` it was given (Raspberry Pi
  // Mouse: 1.85 mm). The offset is now KEPT (OMNISIM_URDF_REROOT_KEEP_OFFSET,
  // default ON): the Robot frame stays the URDF root's (base_footprint), and
  // the promoted link's visuals, collisions, inertial and child joints are
  // emitted AT the offset, so the composed URDF kinematics are exact.
  bool rerooted = false;
  UrdfOrigin rerootOffset;  // URDF root frame -> promoted link frame
  while (true) {
    const UrdfLink &candidate = robot.links.value(rootName);
    if (candidate.visuals.isEmpty() && candidate.collisions.isEmpty() && !candidate.inertial.present) {
      QList<UrdfJoint> outJoints;
      for (const UrdfJoint &j : robot.joints) {
        if (j.parent == rootName)
          outJoints.append(j);
      }
      if (outJoints.size() == 1 && outJoints.first().type == "fixed") {
        rerootOffset = composeUrdfOrigins(rerootOffset, outJoints.first().origin);
        rootName = outJoints.first().child;
        rerooted = true;
        continue;
      }
    }
    break;
  }
  const bool keepRerootOffset = rerooted && !isIdentityUrdfOrigin(rerootOffset) && urdfRerootKeepOffsetEnabled();
  if (rerooted && urdfDebugEnabled()) {
    OmLog::info(QObject::tr("URDF_DEBUG robot '%1': re-rooted to '%2' (skipping empty frame links), offset "
                            "xyz=[%3] rpy=[%4] %5")
                  .arg(robot.name, rootName)
                  .arg(vectorString(rerootOffset.x, rerootOffset.y, rerootOffset.z))
                  .arg(vectorString(rerootOffset.roll, rerootOffset.pitch, rerootOffset.yaw))
                  .arg(keepRerootOffset ? "kept" : "dropped"),
                false, OmLog::PARSING);
  }

  if (urdfDebugEnabled()) {
    QString message = QObject::tr("URDF_DEBUG robot '%1': root='%2' links=%3 joints=%4");
    message = message.arg(robot.name);
    message = message.arg(rootName);
    message = message.arg(robot.links.size());
    message = message.arg(robot.joints.size());
    OmLog::info(message, false, OmLog::PARSING);
  }

  UrdfLink rootLink = robot.links.value(rootName);

  // OmniSim requires the Robot root to have Physics if any descendant Solid
  // does; otherwise it logs "controlled in kinematics mode but some Solid
  // descendant nodes have physics". This pattern is common in ROS URDFs
  // (e.g. Husky base_link, TurtleBot3 base_footprint) where the root link is
  // a pure frame and the real mass lives on a fixed-jointed child like
  // inertial_link or base_link. Synthesize a Physics block on the root so
  // the robot is treated as dynamic; descendant inertials get merged in by
  // OmniSim' SolidMerger.
  //
  // Use 0.1 kg (not 0.001) and a 0.01 m sphere bounding object. Smaller
  // values (mass=0.001, sphere=0.001) produced bodies whose ODE inertia
  // tensor was so tiny (~1e-10) that the merger appeared to produce a body
  // that received no friction-induced acceleration from the wheels --
  // observed on TurtleBot3 burger where wheels rotated at commanded speed
  // but the chassis stayed at exactly (0,0,0) for the entire run. 0.1/0.01
  // is still small enough that the descendant inertials dominate the merged
  // mass distribution.
  if (!rootLink.inertial.present) {
    bool anyDescendantHasInertial = false;
    for (const QString &linkName : robot.linkOrder) {
      if (linkName == rootName)
        continue;
      if (robot.links.value(linkName).inertial.present) {
        anyDescendantHasInertial = true;
        break;
      }
    }
    if (anyDescendantHasInertial) {
      rootLink.inertial.present = true;
      rootLink.inertial.mass = 0.1;
    }
  }

  QHash<QString, QList<UrdfJoint>> jointsByParent;
  for (const UrdfJoint &j : robot.joints)
    jointsByParent[j.parent].append(j);

  // Keep the re-root offset (see the loop above): express everything the
  // promoted link carries in the URDF root's frame. `rootLink` is already a
  // copy; only the promoted link's own joints are re-expressed -- deeper links
  // hang off them and inherit the shift through the emitted hierarchy.
  if (keepRerootOffset) {
    for (UrdfVisual &v : rootLink.visuals)
      v.origin = composeUrdfOrigins(rerootOffset, v.origin);
    for (UrdfCollision &c : rootLink.collisions)
      c.origin = composeUrdfOrigins(rerootOffset, c.origin);
    if (rootLink.inertial.present) {
      // Only the inertial origin's xyz is emitted (centerOfMass). Its rpy was
      // already folded into the tensor by parseLink (urdfInertialRpyEnabled),
      // so the tensor here is in the promoted link's axes; a rotated offset
      // re-expresses it in the Robot's: I' = R I R^T.
      UrdfOrigin com = rootLink.inertial.origin;
      com.roll = com.pitch = com.yaw = 0.0;
      com = composeUrdfOrigins(rerootOffset, com);
      rootLink.inertial.origin.x = com.x;
      rootLink.inertial.origin.y = com.y;
      rootLink.inertial.origin.z = com.z;
      if (rootLink.inertial.hasInertiaMatrix) {
        UrdfInertial &in = rootLink.inertial;
        rotateUrdfInertia(in.ixx, in.ixy, in.ixz, in.iyy, in.iyz, in.izz, rerootOffset.roll, rerootOffset.pitch,
                          rerootOffset.yaw);
      }
    }
    if (jointsByParent.contains(rootName)) {
      QList<UrdfJoint> &rootJoints = jointsByParent[rootName];
      for (UrdfJoint &j : rootJoints)
        j.origin = composeUrdfOrigins(rerootOffset, j.origin);
    }
  }
  const UrdfOrigin *const rootLinkOffset = keepRerootOffset ? &rerootOffset : nullptr;

  // Walk fixed-joint chain from each link back to the (possibly re-rooted)
  // root and accumulate translations. Used to relocate sensor emission from
  // the deeply-nested link's Solid to the Robot's root, which fixed the
  // load-time crash on device registration (root cause appears to be that
  // device sub-Solids inside a fixed-joint-merged child Solid don't register
  // cleanly with OmniSim' device manager). Non-fixed joints break the chain
  // (we don't try to compute live offsets through hinge joints -- a sensor
  // mounted on a moving arm would need a proper supervisor handle for that).
  auto resolveLinkOffset = [&](const QString &link) -> UrdfOrigin {
    UrdfOrigin acc;  // (0, 0, 0)
    QString cur = link;
    QSet<QString> seen;
    while (cur != rootName && !seen.contains(cur)) {
      seen.insert(cur);
      bool advanced = false;
      for (const UrdfJoint &j : robot.joints) {
        if (j.child != cur)
          continue;
        if (j.type != "fixed")
          return acc;  // chain broken; return accumulated so far (origin-of-arm)
        // Accumulate translation; ignore rpy for now (sensors usually mount
        // with whatever orientation their link sits at, and IMU/GPS use the
        // robot-body frame, not the link frame).
        acc.x += j.origin.x;
        acc.y += j.origin.y;
        acc.z += j.origin.z;
        cur = j.parent;
        advanced = true;
        break;
      }
      if (!advanced)
        break;
    }
    return acc;
  };

  // Sensors all emit at Robot root; their per-sensor translation comes from
  // walking the fixed-joint chain back to the (re-rooted) root.
  QHash<QString, QList<UrdfSensor>> sensorsByLink;  // legacy: still keyed by link
  QList<UrdfSensor> sensorsAtRoot;
  for (const UrdfSensor &s : robot.sensors) {
    if (!robot.links.contains(s.linkName)) {
      OmLog::warning(QObject::tr("URDF '%1': sensor '%2' references unknown link '%3'; skipping.")
                       .arg(robot.name, s.name, s.linkName),
                     false, OmLog::PARSING);
      continue;
    }
    UrdfSensor relocated = s;
    const UrdfOrigin off = resolveLinkOffset(s.linkName);
    relocated.translationX = off.x;
    relocated.translationY = off.y;
    relocated.translationZ = off.z;
    sensorsAtRoot.append(relocated);
    sensorsByLink[s.linkName].append(s);  // kept for the debug log
  }

  if (urdfDebugEnabled() && !robot.sensors.isEmpty()) {
    QStringList lines;
    for (const UrdfSensor &s : robot.sensors) {
      static const char *kindNames[] = {"imu", "gps", "camera", "lidar"};
      lines.append(QString("%1@%2(%3)").arg(s.name, s.linkName, kindNames[int(s.kind)]));
    }
    OmLog::info(QObject::tr("URDF_DEBUG robot '%1': sensors=[%2]")
                  .arg(robot.name, lines.join(", ")),
                false, OmLog::PARSING);
  }

  QString out;
  out += "Robot {\n";
  // Emit a default translation/rotation. The URDFRobot expansion's override
  // injection replaces these in-place instead of inserting new lines -- the
  // insertion path appears to corrupt something OmniSim reads downstream
  // when sensor carrier Solids are present (observed: ACCESS_VIOLATION at
  // world load with the in-memory expansion, identical bytes load fine
  // when written to disk and re-loaded). Replace-in-place doesn't trigger it.
  out += "  translation 0 0 0\n";
  out += "  rotation 0 0 1 0\n";
  out += QString("  name \"%1\"\n").arg(robot.name);
  out += "  children [\n";
  out += emitLinkChildren(rootLink, "    ", sensorsByLink, rootLinkOffset);
  if (jointsByParent.contains(rootName)) {
    for (const UrdfJoint &j : jointsByParent.value(rootName))
      out += emitJoint(j, "    ", jointsByParent, robot.links, sensorsByLink);
  }
  // Sensor carriers now emit inline as children of their attached link's
  // Solid via emitSensorsForLink (called by emitLinkChildren during the
  // recursive tree walk). Root-level emission is no longer needed.
  (void)sensorsAtRoot;
  out += "  ]\n";
  out += emitLinkPhysics(rootLink, "  ", true, rootLinkOffset);
  out += "}\n";
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

QString OmUrdfImporter::convertContent(const QByteArray &xmlContent, const QString &sourceLabel) {
  UrdfRobot robot;
  // When invoked without a file path, resolve mesh references relative to the
  // current working directory. Callers that care about mesh paths should use
  // convertFile() so the URDF's own directory is used.
  const QString urdfDir = QFileInfo(sourceLabel).isFile()
                            ? QFileInfo(sourceLabel).absolutePath()
                            : QDir::currentPath();

  // Strip unexpanded xacro placeholders like ${namespace} that some
  // upstream URDFs ship literally (TurtleBot3's *.urdf files are the
  // canonical example -- they're really xacro fragments distributed as
  // .urdf, and consumers are expected to substitute ${namespace} with
  // either a real prefix or empty string). Detect and drop them so
  // joint and link names parse to clean identifiers; warn once per
  // import so the caller knows their URDF wasn't fully rendered.
  QByteArray processed = xmlContent;
  static const QRegularExpression xacroVar("\\$\\{[^}]*\\}");
  if (xacroVar.match(QString::fromUtf8(processed)).hasMatch()) {
    QString text = QString::fromUtf8(processed);
    text.replace(xacroVar, QString());
    processed = text.toUtf8();
    OmLog::warning(QObject::tr("URDF '%1': stripped unexpanded xacro ${...} "
                               "placeholders; render the xacro source for "
                               "production use.")
                     .arg(sourceLabel),
                   false, OmLog::PARSING);
  }

  if (!parseRobotXml(processed, sourceLabel, urdfDir, robot))
    return QString();
  return emitRobot(robot);
}

QString OmUrdfImporter::convertFile(const QString &urdfPath) {
  QFile file(urdfPath);
  if (!file.open(QIODevice::ReadOnly)) {
    OmLog::error(QObject::tr("Cannot open URDF file '%1': this URDFRobot is skipped entirely (no robot, no devices). URDFRobot.url resolves relative to the world file's directory, so check the path from there, and preflight the file with `python scripts/dev/urdf_import.py --report --strict <urdf>` (docs/developer/urdf-import-debugging.md).").arg(urdfPath));
    return QString();
  }
  const QByteArray content = file.readAll();
  file.close();
  return convertContent(content, urdfPath);
}

QByteArray OmUrdfImporter::expandUrdfRobotBlocks(const QByteArray &worldContents,
                                                  const QString &worldFilePath) {
  QString text = QString::fromUtf8(worldContents);

  // Search for URDFRobot { ... } blocks. We do simple brace counting.
  const QString marker = "URDFRobot";
  int searchStart = 0;

  while (true) {
    const int markerPos = text.indexOf(marker, searchStart);
    if (markerPos < 0)
      break;

    // Make sure this is a standalone identifier (not part of a longer word).
    if (markerPos > 0) {
      const QChar before = text.at(markerPos - 1);
      if (before.isLetterOrNumber() || before == '_') {
        searchStart = markerPos + marker.length();
        continue;
      }
    }
    if (markerPos + marker.length() < text.length()) {
      const QChar after = text.at(markerPos + marker.length());
      if (after.isLetterOrNumber() || after == '_') {
        searchStart = markerPos + marker.length();
        continue;
      }
    }

    // Skip if this match is inside a comment (# until end of line).
    {
      int lineStart = markerPos;
      while (lineStart > 0 && text.at(lineStart - 1) != '\n')
        lineStart--;
      bool inComment = false;
      bool inString = false;
      for (int k = lineStart; k < markerPos; k++) {
        const QChar c = text.at(k);
        if (c == '"')
          inString = !inString;
        else if (c == '#' && !inString) {
          inComment = true;
          break;
        }
      }
      if (inComment) {
        searchStart = markerPos + marker.length();
        continue;
      }
    }

    // Find opening brace
    int p = markerPos + marker.length();
    while (p < text.length() && text.at(p).isSpace())
      p++;
    if (p >= text.length() || text.at(p) != '{') {
      searchStart = markerPos + marker.length();
      continue;
    }

    // Find matching closing brace
    int depth = 1;
    int q = p + 1;
    while (q < text.length() && depth > 0) {
      const QChar ch = text.at(q);
      if (ch == '{')
        depth++;
      else if (ch == '}')
        depth--;
      if (depth == 0)
        break;
      q++;
    }
    if (depth != 0)
      break;  // unmatched braces; let the parser report the error

    // Extract the inside of the URDFRobot block
    const QString inside = text.mid(p + 1, q - p - 1);

    // Parse fields. We support: url, translation, rotation, name, controller,
    // controllerArgs, supervisor, customData, staticBase, physicsBackend.
    QString urlField, translationField, rotationField, nameField, controllerField,
            controllerArgsField, supervisorField, customDataField, staticBaseField,
            physicsBackendField, windowField;

    {
      // Very small field parser: identifier followed by either a quoted string,
      // a brace block, or a list of numbers up to the next identifier or the end.
      int idx = 0;
      while (idx < inside.length()) {
        // Skip whitespace
        while (idx < inside.length() && inside.at(idx).isSpace())
          idx++;
        // Skip comments
        if (idx < inside.length() && inside.at(idx) == '#') {
          while (idx < inside.length() && inside.at(idx) != '\n')
            idx++;
          continue;
        }
        if (idx >= inside.length())
          break;
        // Read identifier
        int idStart = idx;
        while (idx < inside.length() &&
               (inside.at(idx).isLetterOrNumber() || inside.at(idx) == '_'))
          idx++;
        if (idx == idStart) {
          idx++;
          continue;
        }
        const QString fieldName = inside.mid(idStart, idx - idStart);
        // Skip whitespace
        while (idx < inside.length() && inside.at(idx).isSpace())
          idx++;
        if (idx >= inside.length())
          break;

        // Read value
        QString value;
        if (inside.at(idx) == '"') {
          // Quoted string. Backslash-escapes (`\"` and `\\`) are honored
          // so customData payloads containing escaped quotes -- e.g.
          // `customData "{\"wheel_torque\":...}"` -- aren't truncated at
          // the first inner quote. Without this the rest of the field
          // block parses as garbage and the URDF expansion that
          // re-emits customData ends up with mismatched quotes
          // ("Unclosed string literal" errors at the post-expansion
          // line that re-emits the truncated value).
          int valStart = idx;
          idx++;
          while (idx < inside.length() && inside.at(idx) != '"') {
            if (inside.at(idx) == '\\' && idx + 1 < inside.length())
              idx += 2;  // skip escape pair (\" or \\); both consume two chars
            else
              idx++;
          }
          if (idx < inside.length())
            idx++;  // closing quote
          value = inside.mid(valStart, idx - valStart);
        } else {
          // Read until end of line OR until the next known field-name token.
          // The latter matters when the whole URDFRobot block is on a single
          // line (no newlines inside) - without this the parser would eat
          // every subsequent field as part of the current value.
          static const QSet<QString> kFieldNames = {
            "url", "translation", "rotation", "name", "controller",
            "controllerArgs", "supervisor", "customData", "staticBase",
            "physicsBackend"
          };
          int valStart = idx;
          while (idx < inside.length() && inside.at(idx) != '\n') {
            if (inside.at(idx).isSpace() && idx + 1 < inside.length() &&
                inside.at(idx + 1).isLetter()) {
              int peekStart = idx + 1;
              int peekEnd = peekStart;
              while (peekEnd < inside.length() &&
                     (inside.at(peekEnd).isLetterOrNumber() || inside.at(peekEnd) == '_'))
                peekEnd++;
              if (peekEnd > peekStart &&
                  kFieldNames.contains(inside.mid(peekStart, peekEnd - peekStart)))
                break;
            }
            idx++;
          }
          value = inside.mid(valStart, idx - valStart).trimmed();
        }

        if (fieldName == "url")
          urlField = value;
        else if (fieldName == "translation")
          translationField = value;
        else if (fieldName == "rotation")
          rotationField = value;
        else if (fieldName == "name")
          nameField = value;
        else if (fieldName == "controller")
          controllerField = value;
        else if (fieldName == "controllerArgs")
          controllerArgsField = value;
        else if (fieldName == "supervisor")
          supervisorField = value;
        else if (fieldName == "customData")
          customDataField = value;
        else if (fieldName == "staticBase")
          staticBaseField = value;
        else if (fieldName == "physicsBackend")
          physicsBackendField = value;
        else if (fieldName == "window")
          windowField = value;
      }
    }

    if (urlField.isEmpty()) {
      OmLog::error(QObject::tr("URDFRobot block at offset %1 has no 'url' field").arg(markerPos));
      searchStart = q + 1;
      continue;
    }

    // Strip surrounding quotes from url
    if (urlField.startsWith('"') && urlField.endsWith('"'))
      urlField = urlField.mid(1, urlField.length() - 2);

    // Resolve URDF path relative to the world file
    QString urdfPath = urlField;
    if (!QFileInfo(urdfPath).isAbsolute()) {
      const QFileInfo worldInfo(worldFilePath);
      urdfPath = worldInfo.absoluteDir().filePath(urdfPath);
    }

    QString vrml = OmUrdfImporter::convertFile(urdfPath);
    if (vrml.isEmpty()) {
      searchStart = q + 1;
      continue;
    }

    // Inject overrides (translation, rotation, name, controller) into the
    // generated Robot { ... } text by string substitution. The emitter
    // emits default `translation 0 0 0` and `rotation 0 0 1 0` lines, so
    // these patches REPLACE in-place; if the defaults aren't present (e.g.
    // older importer output), fall back to inserting after `Robot {\n`.
    auto patchOrInsert = [&](const QString &fieldName, const QString &value) {
      const QRegularExpression re(QString("\n  %1 [^\n]*\n").arg(fieldName));
      QRegularExpressionMatch match = re.match(vrml);
      if (match.hasMatch()) {
        vrml.replace(match.capturedStart(), match.capturedLength(), QString("\n  %1 %2\n").arg(fieldName, value));
      } else {
        const int robotEnd = vrml.indexOf("Robot {");
        if (robotEnd >= 0) {
          const int insertAt = vrml.indexOf('\n', robotEnd) + 1;
          vrml.insert(insertAt, QString("  %1 %2\n").arg(fieldName, value));
        }
      }
    };
    if (!translationField.isEmpty())
      patchOrInsert("translation", translationField);
    if (!rotationField.isEmpty())
      patchOrInsert("rotation", rotationField);
    if (!nameField.isEmpty()) {
      // Replace only the first 'name "..."' occurrence (the Robot's name).
      const QRegularExpression re("name \"[^\"]*\"");
      QRegularExpressionMatch match = re.match(vrml);
      if (match.hasMatch())
        vrml.replace(match.capturedStart(), match.capturedLength(), QString("name %1").arg(nameField));
    }
    if (!controllerField.isEmpty()) {
      // Replace controller line if present, else insert before final '}'
      const QRegularExpression re("controller \"[^\"]*\"");
      QRegularExpressionMatch match = re.match(vrml);
      if (match.hasMatch())
        vrml.replace(match.capturedStart(), match.capturedLength(), QString("controller %1").arg(controllerField));
      else {
        const int lastBrace = vrml.lastIndexOf('}');
        if (lastBrace >= 0)
          vrml.insert(lastBrace, QString("  controller %1\n").arg(controllerField));
      }
    }
    if (!controllerArgsField.isEmpty()) {
      // controllerArgs is a list value like ["--port" "6080"]. The
      // emitted Robot block from OmUrdfImporter::convertFile() never
      // includes one (the URDF import doesn't know about controller
      // arguments) so always insert before the final '}'. Multi-husky
      // worlds depend on this so each URDFRobot's bridge can take a
      // different --port without env-var gymnastics.
      const int lastBrace = vrml.lastIndexOf('}');
      if (lastBrace >= 0)
        vrml.insert(lastBrace, QString("  controllerArgs %1\n").arg(controllerArgsField));
    }
    if (!supervisorField.isEmpty()) {
      // Inject `supervisor TRUE` (or whatever the user wrote) before the final
      // '}' of the generated Robot block. Without this the imported robot is
      // never a supervisor and getSelf() returns NULL on the controller side.
      const int lastBrace = vrml.lastIndexOf('}');
      if (lastBrace >= 0)
        vrml.insert(lastBrace, QString("  supervisor %1\n").arg(supervisorField));
    }
    if (!customDataField.isEmpty()) {
      const int lastBrace = vrml.lastIndexOf('}');
      if (lastBrace >= 0)
        vrml.insert(lastBrace, QString("  customData %1\n").arg(customDataField));
    }
    // P3.10 of cuda-newton-physics-plan.md: pass physicsBackend through
    // to the generated Robot. The inheritance mechanism in
    // OmSolid::effectivePhysicsBackendName then propagates the field
    // to every wheel/link Solid the URDF expansion generated, so
    //   URDFRobot { url "husky.urdf" physicsBackend "newton" }
    // routes the entire articulated tree through Newton with one field.
    if (!physicsBackendField.isEmpty()) {
      const int lastBrace = vrml.lastIndexOf('}');
      if (lastBrace >= 0)
        vrml.insert(lastBrace, QString("  physicsBackend %1\n").arg(physicsBackendField));
    }
    // Pass through the `window` field so URDFRobot can opt into a custom
    // HTML robot-window plugin (e.g. omnilink_chat for the chat-driven
    // OmniLink demos). Without this the field is silently dropped and the
    // robot falls back to the default <generic> motors/sensors window.
    if (!windowField.isEmpty()) {
      const int lastBrace = vrml.lastIndexOf('}');
      if (lastBrace >= 0)
        vrml.insert(lastBrace, QString("  window %1\n").arg(windowField));
    }

    // staticBase TRUE removes the root Physics block from the emitted Robot.
    // OmniSim reads a Robot with no physics field as a kinematic root, which is
    // exactly the "bolted to the floor" semantics arms like UR5e and Panda
    // expose through the staticBase PROTO field. Without this, even a 2 kg
    // base will skate around the world under arm-joint torque.
    if (staticBaseField.compare("TRUE", Qt::CaseInsensitive) == 0) {
      const QString marker = "\n  physics Physics {";
      const int blockNl = vrml.indexOf(marker);
      if (blockNl >= 0) {
        // The block starts with the two-space indent on the line after \n.
        const int blockStart = blockNl + 1;
        int p = blockStart;
        int depth = 0;
        bool started = false;
        while (p < vrml.length()) {
          const QChar c = vrml.at(p);
          if (c == '{') {
            depth++;
            started = true;
          } else if (c == '}') {
            depth--;
            if (started && depth == 0) {
              p++;  // include the closing brace
              break;
            }
          }
          p++;
        }
        // Eat the trailing newline so we don't leave a blank line behind.
        if (p < vrml.length() && vrml.at(p) == '\n')
          p++;
        vrml.remove(blockStart, p - blockStart);
      }
    }

    // Replace the entire URDFRobot { ... } block with the generated VRML
    text.replace(markerPos, q - markerPos + 1, vrml);
    searchStart = markerPos + vrml.length();

    if (urdfDebugEnabled()) {
      const QString dumpPath = QDir::temp().filePath(
        QString("omnisim_urdf_%1.omniworld.snippet").arg(QFileInfo(urdfPath).baseName()));
      QFile dumpFile(dumpPath);
      if (dumpFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        dumpFile.write(vrml.toUtf8());
        dumpFile.close();
        OmLog::info(QObject::tr("URDF_DEBUG: wrote expanded VRML for '%1' to %2").arg(urdfPath, dumpPath));
      }
    }
  }

  const QByteArray result = text.toUtf8();
  if (urdfDebugEnabled()) {
    const QString dumpPath = QDir::temp().filePath("omnisim_world_postexpand.omniworld");
    QFile dumpFile(dumpPath);
    if (dumpFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
      dumpFile.write(result);
      dumpFile.close();
      OmLog::info(QObject::tr("URDF_DEBUG: wrote post-expansion world to %1").arg(dumpPath));
    }
  }
  return result;
}
