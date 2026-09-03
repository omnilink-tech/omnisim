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

#include <cmath>

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

// URDF inertia tensors are imported only when this is set. Even URDF tensors
// that pass every PD + triangle-inequality check still trip an ODE/Webots
// crash on small-mass child links (e.g. Jackal wheels at mass=0.477 with
// 0.0013/0.0024/0.0013 inertia). Until that's diagnosed, we default to
// bounding-object-derived inertia — historically what the importer did —
// and let advanced users opt back in with OMNISIM_URDF_USE_INERTIA=1.
bool urdfUseInertiaEnabled() {
  const QString value = QString::fromUtf8(qgetenv("OMNISIM_URDF_USE_INERTIA")).trimmed().toLower();
  return !value.isEmpty() && value != "0" && value != "false" && value != "off";
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

UrdfMaterial parseMaterial(const QDomElement &elem, QHash<QString, UrdfMaterial> &materials) {
  UrdfMaterial m;
  m.name = elem.attribute("name");
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
      vis.material = parseMaterial(matEl, materials);
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
      } else if (ixx > 0.0 || iyy > 0.0 || izz > 0.0) {
        OmLog::warning(QObject::tr("URDF link '%1': inertia tensor is not positive definite "
                                   "(ixx=%2 iyy=%3 izz=%4 ixy=%5 ixz=%6 iyz=%7); "
                                   "falling back to bounding-object inertia.")
                         .arg(link.name)
                         .arg(ixx).arg(iyy).arg(izz)
                         .arg(ixy).arg(ixz).arg(iyz),
                       false, OmLog::PARSING);
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
    }
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
    parseMaterial(m, robot.materials);

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
  out += innerIndent + "  }\n";

  out += emitGeometry(v.geometry, innerIndent + "  ");
  out += innerIndent + "}\n";

  if (hasTransform) {
    out += indent + "  ]\n";
    out += indent + "}\n";
  }
  return out;
}

// Emit the raw geometry primitive (Box, Cylinder, Sphere, Mesh) with the given indent.
QString emitRawGeometry(const UrdfGeometry &g, const QString &indent) {
  if (g.kind == "box")
    return indent + QString("Box { size %1 %2 %3 }\n").arg(g.sx).arg(g.sy).arg(g.sz);
  if (g.kind == "cylinder")
    return indent + QString("Cylinder { height %1 radius %2 }\n").arg(g.length).arg(g.radius);
  if (g.kind == "sphere")
    return indent + QString("Sphere { radius %1 }\n").arg(g.radius);
  if (g.kind == "mesh" && !g.meshPath.isEmpty())
    return indent + QString("Mesh { url \"%1\" }\n").arg(g.meshPath);
  return QString();
}

// OmniSim does not honour scale on a Transform inside a boundingObject. If a
// URDF collision mesh has non-unit scale we warn once per link; the mesh is
// still emitted at unit scale so the robot at least has collision volume.
void warnMeshCollisionScale(const QString &linkName, const UrdfGeometry &g) {
  if (meshHasNonUnitScale(g)) {
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
      return indent + QString("boundingObject Mesh { url \"%1\" }\n").arg(c.geometry.meshPath);
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
      if (s.lidarNumberOfLayers > 1) {
        out += in + QString("  numberOfLayers %1\n").arg(s.lidarNumberOfLayers);
        if (s.lidarVerticalFov > 0.0)
          out += in + QString("  verticalFieldOfView %1\n").arg(s.lidarVerticalFov);
      }
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
                           const QHash<QString, QList<UrdfSensor>> &sensorsByLink) {
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
    local.translationX = 0.0;
    local.translationY = 0.0;
    local.translationZ = 0.0;
    out += emitOneSensorAtRoot(local, indent);
  }
  return out;
}

QString emitLinkChildren(const UrdfLink &link, const QString &indent,
                        const QHash<QString, QList<UrdfSensor>> &sensorsByLink) {
  QString out;
  for (const UrdfVisual &v : link.visuals)
    out += emitVisual(v, indent, link.name);
  out += emitSensorsForLink(link.name, indent, sensorsByLink);
  return out;
}

QString emitLinkPhysics(const UrdfLink &link, const QString &indent, bool allowSyntheticPhysics = true) {
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
  } else if (link.inertial.present) {
    // OmniSim needs a boundingObject to compute an inertia matrix; a link with
    // <inertial> but no <collision> would otherwise trigger "Undefined inertia
    // matrix" on every load. Emit a 10 mm placeholder sphere -- big enough
    // that the auto-derived inertia tensor isn't degenerate (1 mm sphere
    // produced ~1e-10 inertia, which appears to leave the merged root body
    // unable to receive friction-induced acceleration on small robots).
    out += indent + "boundingObject Sphere { radius 0.01 }\n";
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
  const bool needsSyntheticPhysics =
    allowSyntheticPhysics && !link.inertial.present && !supported.isEmpty();
  if (link.inertial.present || needsSyntheticPhysics) {
    const double mass = link.inertial.present ? link.inertial.mass : 0.001;
    out += indent + "physics Physics {\n";
    out += indent + "  density -1\n";
    out += indent + QString("  mass %1\n").arg(mass);
    // OmniSim refuses the custom inertiaMatrix unless mass > 0 AND a single-line
    // centerOfMass is present. So when we have a URDF tensor, emit the COM even
    // when the URDF origin is the link origin (0 0 0) — otherwise the tensor
    // would be silently ignored.
    //
    // Also clamp out vanishingly-small tensors: ODE's dMassSetParameters has
    // been observed to crash on inertia values below ~1e-4 even though the
    // tensor satisfies positive-definiteness and the triangle inequality
    // (Jackal wheels at ixx=iyy=0.0013 are right on the threshold; URDFs that
    // declare 1e-9 placeholder inertias for sensor-mount frames are
    // unambiguously past it). Below the threshold we drop the explicit tensor
    // and let OmniSim derive inertia from the bounding object.
    constexpr double kMinInertia = 1e-4;
    const bool tensorTooSmall = link.inertial.hasInertiaMatrix
                                && (link.inertial.ixx < kMinInertia
                                    || link.inertial.iyy < kMinInertia
                                    || link.inertial.izz < kMinInertia);
    const bool hasUrdfInertia = link.inertial.present && link.inertial.hasInertiaMatrix && mass > 0.0
                                && !tensorTooSmall && urdfUseInertiaEnabled();
    const bool hasComOffset = link.inertial.present &&
                              (link.inertial.origin.x != 0.0 || link.inertial.origin.y != 0.0 ||
                               link.inertial.origin.z != 0.0);
    if (hasUrdfInertia || hasComOffset) {
      out += indent + QString("  centerOfMass [%1 %2 %3]\n")
                          .arg(link.inertial.origin.x)
                          .arg(link.inertial.origin.y)
                          .arg(link.inertial.origin.z);
    }
    if (hasUrdfInertia) {
      // OmniSim inertiaMatrix layout: line 1 = [Ixx Iyy Izz], line 2 = [Ixy Ixz Iyz].
      // Same sign convention as URDF — both store the un-negated integrals.
      out += indent + QString("  inertiaMatrix [\n");
      out += indent + QString("    %1 %2 %3\n").arg(link.inertial.ixx).arg(link.inertial.iyy).arg(link.inertial.izz);
      out += indent + QString("    %1 %2 %3\n").arg(link.inertial.ixy).arg(link.inertial.ixz).arg(link.inertial.iyz);
      out += indent + QString("  ]\n");
    }
    out += indent + "}\n";
  }
  return out;
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
    if (joint.lower > 0.0 || joint.upper < 0.0) {
      // Same reasoning as the revolute case: the joint would otherwise start
      // outside its own stops, which the engine warns about on every load.
      out += indent + QString("position %1\n").arg(joint.lower > 0.0 ? joint.lower : joint.upper);
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
    if (!fullRange) {
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
        if (joint.hasRest)
          out += indent + QString("position %1\n").arg(joint.rest);
        else if (lo > 0.0 || hi < 0.0)
          out += indent + QString("position %1\n").arg(0.5 * (lo + hi));
      }
    }
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

QString emitSolidEndPoint(const UrdfLink &link, const UrdfJoint &joint, const QString &indent,
                          const QHash<QString, QList<UrdfJoint>> &jointsByParent,
                          const QHash<QString, UrdfLink> &links,
                          const QHash<QString, QList<UrdfSensor>> &sensorsByLink) {
  QString out = indent + "endPoint Solid {\n";
  out += indent + QString("  translation %1 %2 %3\n").arg(joint.origin.x).arg(joint.origin.y).arg(joint.origin.z);
  double ax, ay, az, ang;
  rpyToAxisAngle(joint.origin.roll, joint.origin.pitch, joint.origin.yaw, ax, ay, az, ang);
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
  out += emitLinkPhysics(link, indent + "  ");
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
    if (j.type == "revolute" && j.hasLower && j.hasUpper) {
      const double kFullRange = M_PI - 0.01;
      if (j.lower <= -kFullRange && j.upper >= kFullRange && j.upper > j.lower) {
        out += indent + QString("      minPosition %1\n").arg(j.lower);
        out += indent + QString("      maxPosition %1\n").arg(j.upper);
      }
    }
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
  // Robot node, no merger gymnastics required. Any fixed-joint offset is
  // dropped -- the new root sits at the original Robot's `translation`,
  // which usually shifts the robot a few cm relative to where the URDF
  // author intended; small price for getting motion to work.
  bool rerooted = false;
  while (true) {
    const UrdfLink &candidate = robot.links.value(rootName);
    if (candidate.visuals.isEmpty() && candidate.collisions.isEmpty() && !candidate.inertial.present) {
      QList<UrdfJoint> outJoints;
      for (const UrdfJoint &j : robot.joints) {
        if (j.parent == rootName)
          outJoints.append(j);
      }
      if (outJoints.size() == 1 && outJoints.first().type == "fixed") {
        rootName = outJoints.first().child;
        rerooted = true;
        continue;
      }
    }
    break;
  }
  if (rerooted && urdfDebugEnabled()) {
    OmLog::info(QObject::tr("URDF_DEBUG robot '%1': re-rooted to '%2' (skipping empty frame links)")
                  .arg(robot.name, rootName),
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
  out += emitLinkChildren(rootLink, "    ", sensorsByLink);
  if (jointsByParent.contains(rootName)) {
    for (const UrdfJoint &j : jointsByParent.value(rootName))
      out += emitJoint(j, "    ", jointsByParent, robot.links, sensorsByLink);
  }
  // Sensor carriers now emit inline as children of their attached link's
  // Solid via emitSensorsForLink (called by emitLinkChildren during the
  // recursive tree walk). Root-level emission is no longer needed.
  (void)sensorsAtRoot;
  out += "  ]\n";
  out += emitLinkPhysics(rootLink, "  ");
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
