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

#include "OmShape.hpp"

#include "OmAppearance.hpp"
#include "OmBoundingSphere.hpp"
#include "OmImageTexture.hpp"
#include "OmMaterial.hpp"
#include "OmNodeUtilities.hpp"
#include "OmOdeContext.hpp"
#include "OmPaintTexture.hpp"
#include "OmPbrAppearance.hpp"
#include "OmPreferences.hpp"
#include "OmRay.hpp"
#include "OmRgb.hpp"
#include "OmSFBool.hpp"
#include "OmSFNode.hpp"

void OmShape::init() {
  mAppearance = findSFNode("appearance");
  mGeometry = findSFNode("geometry");
  mCastShadows = findSFBool("castShadows");
  mIsPickable = findSFBool("isPickable");
}

OmShape::OmShape(OmTokenizer *tokenizer) : OmBaseNode("Shape", tokenizer) {
  init();
}

OmShape::OmShape(const OmShape &other) : OmBaseNode(other) {
  init();
}

OmShape::OmShape(const OmNode &other) : OmBaseNode(other) {
  init();
}

OmShape::~OmShape() {
}

void OmShape::downloadAssets() {
  OmBaseNode::downloadAssets();
  if (abstractAppearance())
    abstractAppearance()->downloadAssets();
  if (geometry())
    geometry()->downloadAssets();
}

void OmShape::preFinalize() {
  OmBaseNode::preFinalize();

  // handle both kinds of appearance nodes
  OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->preFinalize();

  if (geometry())
    geometry()->preFinalize();

  updateAppearance();
  updateGeometry();
}

void OmShape::postFinalize() {
  OmBaseNode::postFinalize();

  // handle both kinds of appearance nodes
  OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->postFinalize();

  if (geometry())
    geometry()->postFinalize();

  connect(mAppearance, &OmSFNode::changed, this, &OmShape::updateAppearance);
  connect(mGeometry, &OmSFNode::changed, this, &OmShape::updateGeometry);
  if (!isInBoundingObject()) {
    connect(mCastShadows, &OmSFBool::changed, this, &OmShape::updateCastShadows);
    updateCastShadows();
    connect(mIsPickable, &OmSFBool::changed, this, &OmShape::updateIsPickable);
    updateIsPickable();
  }
  if (geometry())
    connect(geometry(), &OmGeometry::changed, this, &OmShape::updateGeometryMaterial, Qt::UniqueConnection);

  connect(OmPreferences::instance(), &OmPreferences::changedByUser, this, &OmShape::updateAppearance);
}

void OmShape::reset(const QString &id) {
  OmBaseNode::reset(id);

  // handle both kinds of appearance nodes
  OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->reset(id);

  OmNode *const geometryNode = mGeometry->value();
  if (geometryNode)
    geometryNode->reset(id);
}

OmAppearance *OmShape::appearance() const {
  return dynamic_cast<OmAppearance *>(mAppearance->value());
}

OmPbrAppearance *OmShape::pbrAppearance() const {
  return dynamic_cast<OmPbrAppearance *>(mAppearance->value());
}

OmAbstractAppearance *OmShape::abstractAppearance() const {
  return dynamic_cast<OmAbstractAppearance *>(mAppearance->value());
}

void OmShape::propagateSelection(bool selected) {
  OmGeometry *const g = geometry();
  if (g)
    geometry()->propagateSelection(selected);
}

void OmShape::updateCollisionMaterial(bool triggerChange, bool onSelection) {  // for OmShapes lying into a bounding object only
  OmGeometry *const g = geometry();
  if (g)
    geometry()->updateCollisionMaterial(triggerChange, onSelection);
}

void OmShape::setScaleNeedUpdate() {  // for any OmShape
  OmGeometry *const g = geometry();
  if (g)
    g->setScaleNeedUpdate();
}

void OmShape::setSleepMaterial() {
  OmGeometry *const g = geometry();
  if (g)
    geometry()->setSleepMaterial();
}

void OmShape::updateSegmentationColor(const OmRgb &) {
  // D1.4: the WREN segmentation-material recolour is gone; the tree-wide
  // virtual is kept so ancestor recursion still terminates here cleanly.
}

void OmShape::updateAppearance() {
  if (appearance())
    connect(appearance(), &OmAppearance::changed, this, &OmShape::updateAppearance, Qt::UniqueConnection);
  else if (pbrAppearance())
    connect(pbrAppearance(), &OmPbrAppearance::changed, this, &OmShape::updateAppearance, Qt::UniqueConnection);

  if (areWrenObjectsInitialized())
    applyMaterialToGeometry();
  else if (!isInBoundingObject())
    emit wrenMaterialChanged();
}

void OmShape::updateGeometry() {
  if (isInBoundingObject())
    return;

  const OmGeometry *const g = geometry();
  if (g) {
    connect(g, &OmGeometry::wrenObjectsCreated, this, &OmShape::updateGeometryMaterial, Qt::UniqueConnection);
    connect(g, &OmGeometry::wrenObjectsCreated, this, &OmShape::updateIsPickable, Qt::UniqueConnection);
    connect(g, &OmGeometry::changed, this, &OmShape::updateGeometryMaterial, Qt::UniqueConnection);
    if (isPostFinalizedCalled()) {
      if (g->isPostFinalizedCalled())
        OmBoundingSphere::addSubBoundingSphereToParentNode(this);
      else
        connect(g, &OmBaseNode::finalizationCompleted, this, &OmShape::updateBoundingSphere, Qt::UniqueConnection);
    }
  }
}

void OmShape::updateBoundingSphere(OmBaseNode *subNode) {
  disconnect(subNode, &OmBaseNode::finalizationCompleted, this, &OmShape::updateBoundingSphere);
  OmBoundingSphere::addSubBoundingSphereToParentNode(this);
}

void OmShape::updateCastShadows() {
  assert(!isInBoundingObject());

  // D1.4: the geometry no longer latches a WREN cast-shadows flag; the wgpu
  // draw reads isCastShadowsEnabled() off this node. The signal still fires so
  // consumers (OmTrack's animated belt) can refresh.
  if (geometry())
    emit castShadowsChanged();
}

void OmShape::updateIsPickable() {
  assert(!isInBoundingObject());
  OmGeometry *const g = geometry();
  if (g)
    g->setPickable(mIsPickable->value());
}

void OmShape::setAppearance(OmAppearance *appearance) {
  mAppearance->removeValue();
  mAppearance->setValue(appearance);
}

void OmShape::setPbrAppearance(OmPbrAppearance *appearance) {
  mAppearance->removeValue();
  mAppearance->setValue(appearance);
}

void OmShape::setGeometry(OmGeometry *geometry) {
  mGeometry->removeValue();
  mGeometry->setValue(geometry);
}

void OmShape::updateGeometryMaterial() {
  if (areWrenObjectsInitialized())
    applyMaterialToGeometry();
}

bool OmShape::isCastShadowsEnabled() const {
  return mCastShadows->value();
}

OmBoundingSphere *OmShape::boundingSphere() const {
  const OmGeometry *const g = geometry();
  if (g)
    return g->boundingSphere();

  return NULL;
}

void OmShape::createWrenObjects() {
  OmBaseNode::createWrenObjects();

  // handle both kinds of appearance nodes
  OmBaseNode *baseNode = dynamic_cast<OmBaseNode *>(mAppearance->value());
  if (baseNode)
    baseNode->createWrenObjects();

  OmGeometry *const g = geometry();
  if (g)
    g->createWrenObjects();
}

void OmShape::applyMaterialToGeometry() {
  // D1.4: the WREN material build/bind is gone -- the wgpu path reads the
  // appearance node off this Shape at collect time. What SURVIVES here is the
  // engine-state latching WREN's bind used to piggy-back on: the geometry's
  // fully-transparent flag, and the material-changed signal consumers key on.
  OmGeometry *const g = geometry();
  if (g) {
    if (appearance()) {
      if (appearance()->areWrenObjectsInitialized()) {
        if (appearance()->material() && appearance()->material()->transparency() == 1)
          g->setTransparent(true);
        else
          g->setTransparent(false);
      } else {
        // We need to call setTransparent for default appearance in case a previous transparent appearance was existing and
        // replaced by the default one.
        g->setTransparent(false);
      }
    } else if (pbrAppearance() && g->nodeType() != WB_NODE_POINT_SET) {
      if (pbrAppearance()->areWrenObjectsInitialized()) {
        if (pbrAppearance()->transparency() == 1)
          g->setTransparent(true);
        else
          g->setTransparent(false);
      }
    } else {
      // We need to call setTransparent for default appearance in case a previous transparent appearance was existing and
      // replaced by the default one.
      g->setTransparent(false);
    }

    emit wrenMaterialChanged();
  }
}

// ray cast to a shape: pick the collided color
// using uv mapping, paint color and diffuse color
// could be improved by computing the exact openGL computing including lighting
void OmShape::pickColor(const OmRay &ray, OmRgb &pickedColor, double *roughness, double *occlusion) const {
  const OmAppearance *const app = appearance();
  const OmPbrAppearance *const pbrApp = pbrAppearance();
  const OmGeometry *const geom = geometry();

  OmRgb diffuseColor(1.0f, 1.0f, 1.0f);
  OmRgb textureColor(1.0f, 1.0f, 1.0f);
  OmRgb paintColor(1.0f, 1.0f, 1.0f);
  pickedColor = OmRgb(1.0f, 1.0f, 1.0f);  // default value
  OmVector2 uv(-1, -1);

  if (roughness)
    *roughness = 0.0;
  if (occlusion)
    *occlusion = 0.0;

  if (geom) {
    float paintContribution = 0.0f;

    OmPaintTexture *paintTexture = OmPaintTexture::findPaintTexture(this);
    if (paintTexture) {
      const bool success = geom->pickUVCoordinate(uv, ray, 0);
      if (!success) {
        if (app) {
          pickedColor = app->diffuseColor();

          // apply transparency factor
          if (app->material()) {
            const float alpha = app->material()->transparency();
            pickedColor.setRed(pickedColor.red() * alpha);
            pickedColor.setGreen(pickedColor.green() * alpha);
            pickedColor.setBlue(pickedColor.blue() * alpha);
          }
        } else if (pbrApp) {
          pickedColor = pbrApp->baseColor();

          // apply transparency factor
          const float alpha = pbrApp->transparency();
          pickedColor.setRed(pickedColor.red() * alpha);
          pickedColor.setGreen(pickedColor.green() * alpha);
          pickedColor.setBlue(pickedColor.blue() * alpha);

          if (roughness)
            *roughness = pbrApp->roughness();
        }

        return;
      }
      // retrieve the corresponding color in the paint texture
      paintTexture->pickColor(uv, paintColor, &paintContribution);
    }

    if (app) {
      // diffuse color
      diffuseColor = app->diffuseColor();

      // apply transparency factor
      if (app->material()) {
        const double alpha = 1.0 - app->material()->transparency();
        diffuseColor.setRed(diffuseColor.red() * alpha);
        diffuseColor.setGreen(diffuseColor.green() * alpha);
        diffuseColor.setBlue(diffuseColor.blue() * alpha);
      }

      // texture color
      if (app->isTextureLoaded()) {
        if (uv.x() == -1) {
          bool success = geom->pickUVCoordinate(uv, ray, 0);
          if (!success) {
            pickedColor = diffuseColor;
            return;
          }
        }

        // retrieve the corresponding color in the texture
        app->pickColorInTexture(uv, textureColor);
      }

    } else if (pbrApp) {
      // diffuse color
      diffuseColor = pbrApp->baseColor();

      // apply transparency factor
      const double alpha = 1.0 - pbrApp->transparency();
      diffuseColor.setRed(diffuseColor.red() * alpha);
      diffuseColor.setGreen(diffuseColor.green() * alpha);
      diffuseColor.setBlue(diffuseColor.blue() * alpha);

      if (roughness)
        *roughness = pbrApp->roughness();

      // texture color
      if (pbrApp->isBaseColorTextureLoaded()) {
        if (uv.x() == -1) {
          bool success = geom->pickUVCoordinate(uv, ray, 0);
          if (!success) {
            pickedColor = diffuseColor;
            return;
          }
        }

        // retrieve the corresponding color in the texture
        pbrApp->pickColorInBaseColorTexture(textureColor, uv);
        if (roughness && pbrApp->isRoughnessTextureLoaded())
          pbrApp->pickRoughnessInTexture(roughness, uv);
        if (occlusion && pbrApp->isOcclusionTextureLoaded())
          pbrApp->pickOcclusionInTexture(occlusion, uv);
      }
    } else if (paintTexture) {
      pickedColor = paintColor;
      return;
    } else
      return;  // default value

    // combine colors
    pickedColor.setRed((1.0f - paintContribution) * diffuseColor.red() * textureColor.red() +
                       paintContribution * paintColor.red());
    pickedColor.setGreen((1.0f - paintContribution) * diffuseColor.green() * textureColor.green() +
                         paintContribution * paintColor.green());
    pickedColor.setBlue((1.0f - paintContribution) * diffuseColor.blue() * textureColor.blue() +
                        paintContribution * paintColor.blue());
  }
}

QList<const OmBaseNode *> OmShape::findClosestDescendantNodesWithDedicatedWrenNode() const {
  QList<const OmBaseNode *> list;
  const OmGeometry *const g = geometry();
  if (g)
    list << g;
  return list;
}

///////////////////////////////////////////////////////////
//  ODE related methods for OmShapes in boundingObjects  //
///////////////////////////////////////////////////////////

void OmShape::connectGeometryField() const {
  connect(mGeometry, &OmSFNode::changed, this, &OmShape::createOdeGeom, Qt::UniqueConnection);
}

void OmShape::disconnectGeometryField() const {
  disconnect(mGeometry, &OmSFNode::changed, this, &OmShape::createOdeGeom);
}

void OmShape::createOdeGeom() {
  const OmGeometry *const g = geometry();

  if (g == NULL) {
    parsingInfo(tr("Please specify 'geometry' field (of Shape placed in 'boundingObject')."));
    return;
  }

  emit geometryInShapeInserted();
}

bool OmShape::isSuitableForInsertionInBoundingObject(bool warning) const {
  const OmGeometry *const g = geometry();
  return g ? g->isSuitableForInsertionInBoundingObject(warning) : true;
}

bool OmShape::isAValidBoundingObject(bool checkOde, bool warning) const {
  const OmGeometry *const g = geometry();
  return g && g->isAValidBoundingObject(checkOde, warning);
}

////////////
// Export //
////////////

bool OmShape::exportNodeHeader(OmWriter &writer) const {
  if (writer.isW3d() && isUseNode() && defNode()) {
    writer << "<" << w3dName() << " id=\'n" << QString::number(uniqueId()) << "\'";
    writer << " USE=\'" + QString::number(defNode()->uniqueId()) + "\'></" + w3dName() + ">";
    return true;
  }

  return OmBaseNode::exportNodeHeader(writer);
}

void OmShape::exportBoundingObjectToW3d(OmWriter &writer) const {
  assert(writer.isW3d());

  if (isUseNode() && defNode())
    writer << "<" << w3dName() << " role='boundingObject' USE=\'n" + QString::number(defNode()->uniqueId()) + "\'/>";
  else {
    writer << "<" << w3dName() << " role='boundingObject'"
           << " id=\'n" << QString::number(uniqueId()) << "\'>";
    geometry()->write(writer);
    writer << "</" + w3dName() + ">";
  }
}

QStringList OmShape::fieldsToSynchronizeWithW3d() const {
  QStringList fields;
  fields << "isPickable"
         << "castShadows";
  return fields;
}
