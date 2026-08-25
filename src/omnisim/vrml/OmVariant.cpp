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

#include "OmVariant.hpp"

#include "OmNode.hpp"
#include "OmRgb.hpp"
#include "OmRotation.hpp"
#include "OmVector2.hpp"
#include "OmVector3.hpp"

#include <QtCore/QString>
#include "../../../include/controller/c/omnisim/supervisor.h"

OmVariant::OmVariant() : mType(-1), mOwnsNode(false) {
}

OmVariant::OmVariant(const OmVariant &other) : mType(-1), mOwnsNode(false) {
  setValue(other);
}

OmVariant &OmVariant::operator=(const OmVariant &other) {
  setValue(other);
  return *this;
}

bool OmVariant::operator==(const OmVariant &other) const {
  if (other.type() != mType)
    return false;
  if (mType == -1)
    return true;

  switch (mType) {
    case WB_SF_BOOL:
      return other.toBool() == mBool;
    case WB_SF_INT32:
      return other.toInt() == mInt;
    case WB_SF_FLOAT:
      return other.toDouble() == mDouble;
    case WB_SF_STRING:
      return other.toString() == *mString;
    case WB_SF_VEC2F:
      return other.toVector2() == *mVector2;
    case WB_SF_VEC3F:
      return other.toVector3() == *mVector3;
    case WB_SF_COLOR:
      return other.toColor() == *mColor;
    case WB_SF_ROTATION:
      return other.toRotation() == *mRotation;
    case WB_SF_NODE: {
      if (!mNode || !other.toNode())
        return mNode == other.toNode();
      return *other.toNode() == *mNode;
    }
    default:
      assert(false);
  }
  return false;
}

bool OmVariant::operator!=(const OmVariant &other) const {
  return !(*this == other);
}

OmVariant::OmVariant(bool b) : mType(-1), mOwnsNode(false) {
  OmVariant::setBool(b);
}

OmVariant::OmVariant(int i) : mType(-1), mOwnsNode(false) {
  OmVariant::setInt(i);
}

OmVariant::OmVariant(double d) : mType(-1), mOwnsNode(false) {
  OmVariant::setDouble(d);
}

OmVariant::OmVariant(const QString &s) : mType(-1), mOwnsNode(false) {
  OmVariant::setString(s);
}

OmVariant::OmVariant(const OmVector2 &v) : mType(-1), mOwnsNode(false) {
  OmVariant::setVector2(v);
}

OmVariant::OmVariant(const OmVector3 &v) : mType(-1), mOwnsNode(false) {
  OmVariant::setVector3(v);
}

OmVariant::OmVariant(const OmRgb &c) : mType(-1), mOwnsNode(false) {
  OmVariant::setColor(c);
}

OmVariant::OmVariant(const OmRotation &r) : mType(-1), mOwnsNode(false) {
  OmVariant::setRotation(r);
}

OmVariant::OmVariant(OmNode *n) : mType(-1), mOwnsNode(false) {
  OmVariant::setNode(n);
}

OmVariant::~OmVariant() {
  OmVariant::clear();
}

void OmVariant::clear() {
  switch (mType) {
    case WB_SF_STRING:
      delete mString;
      break;
    case WB_SF_VEC2F:
      delete mVector2;
      break;
    case WB_SF_VEC3F:
      delete mVector3;
      break;
    case WB_SF_COLOR:
      delete mColor;
      break;
    case WB_SF_ROTATION:
      delete mRotation;
      break;
    case WB_SF_NODE:
      if (mNode && mOwnsNode) {
        delete mNode;
        mNode = NULL;
        mOwnsNode = false;
      }
      break;
  }

  mType = -1;
}

const QString OmVariant::toSimplifiedStringRepresentation(OmPrecision::Level level) const {
  switch (mType) {
    case WB_SF_BOOL:
      return mBool ? "TRUE" : "FALSE";
    case WB_SF_INT32:
      return QString::number(mInt);
    case WB_SF_FLOAT:
      return OmPrecision::doubleToString(mDouble, level);
    case WB_SF_STRING:
      return QString("\"%1\"").arg(*mString);
    case WB_SF_VEC2F:
      return mVector2->toString(level);
    case WB_SF_VEC3F:
      return mVector3->toString(level);
    case WB_SF_COLOR:
      return mColor->toString(level);
    case WB_SF_ROTATION:
      return mRotation->toString(level);
    case WB_SF_NODE: {
      if (mNode)
        return mNode->fullName();
      else
        return "NULL";
    }
    default:
      assert(false);
  }
  return "";
}

void OmVariant::setBool(bool b) {
  clear();
  mBool = b;
  mType = WB_SF_BOOL;
}

void OmVariant::setInt(int i) {
  clear();
  mInt = i;
  mType = WB_SF_INT32;
}

void OmVariant::setDouble(double d) {
  clear();
  mDouble = d;
  mType = WB_SF_FLOAT;
}

void OmVariant::setVector2(const OmVector2 &v) {
  clear();
  mVector2 = new OmVector2(v);
  mType = WB_SF_VEC2F;
}

void OmVariant::setVector3(const OmVector3 &v) {
  clear();
  mVector3 = new OmVector3(v);
  mType = WB_SF_VEC3F;
}

void OmVariant::setString(const QString &s) {
  clear();
  mString = new QString(s);
  mType = WB_SF_STRING;
}

void OmVariant::setColor(const OmRgb &c) {
  mColor = new OmRgb(c);
  mType = WB_SF_COLOR;
}

void OmVariant::setRotation(const OmRotation &r) {
  clear();
  mRotation = new OmRotation(r);
  mType = WB_SF_ROTATION;
}

void OmVariant::setNode(OmNode *n, bool persistent) {
  clear();
  if (persistent) {
    // If persistent is true, the variant owns its own clone of the node.
    // This is usefull in case of enumeration for the field model.
    mNode = n ? n->cloneAndReferenceProtoInstance() : NULL;
    if (mNode)
      mOwnsNode = true;
  } else {
    mNode = n;
    if (n)
      connect(n, &QObject::destroyed, this, &OmVariant::clearNode);
  }
  mType = WB_SF_NODE;
}

void OmVariant::setValue(const OmVariant &v) {
  switch (v.type()) {
    case WB_SF_STRING:
      setString(v.toString());
      break;
    case WB_SF_VEC2F:
      setVector2(v.toVector2());
      break;
    case WB_SF_VEC3F:
      setVector3(v.toVector3());
      break;
    case WB_SF_COLOR:
      setColor(v.toColor());
      break;
    case WB_SF_ROTATION:
      setRotation(v.toRotation());
      break;
    case WB_SF_BOOL:
      setBool(v.toBool());
      break;
    case WB_SF_INT32:
      setInt(v.toInt());
      break;
    case WB_SF_FLOAT:
      setDouble(v.toDouble());
      break;
    case WB_SF_NODE:
      setNode(v.toNode());
      break;
  }
}

void OmVariant::clearNode() {
  mNode = NULL;
}
