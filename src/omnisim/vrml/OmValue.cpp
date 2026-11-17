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

#include "OmValue.hpp"
#include <QtCore/QMap>

static QMap<QString, WbFieldType> initgVrmlMap() {
  QMap<QString, WbFieldType> map;
  map.insert("SFBool", WB_SF_BOOL);
  map.insert("SFInt32", WB_SF_INT32);
  map.insert("SFFloat", WB_SF_FLOAT);
  map.insert("SFVec2f", WB_SF_VEC2F);
  map.insert("SFVec3f", WB_SF_VEC3F);
  map.insert("SFRotation", WB_SF_ROTATION);
  map.insert("SFColor", WB_SF_COLOR);
  map.insert("SFString", WB_SF_STRING);
  map.insert("SFNode", WB_SF_NODE);
  map.insert("MFBool", WB_MF_BOOL);
  map.insert("MFInt32", WB_MF_INT32);
  map.insert("MFFloat", WB_MF_FLOAT);
  map.insert("MFVec2f", WB_MF_VEC2F);
  map.insert("MFVec3f", WB_MF_VEC3F);
  map.insert("MFRotation", WB_MF_ROTATION);
  map.insert("MFColor", WB_MF_COLOR);
  map.insert("MFString", WB_MF_STRING);
  map.insert("MFNode", WB_MF_NODE);
  return map;
}

static QMap<QString, WbFieldType> initShortMap() {
  QMap<QString, WbFieldType> map;
  map.insert("Boolean", WB_SF_BOOL);
  map.insert("Integer", WB_SF_INT32);
  map.insert("Float", WB_SF_FLOAT);
  map.insert("Vector2", WB_SF_VEC2F);
  map.insert("Vector3", WB_SF_VEC3F);
  map.insert("Rotation", WB_SF_ROTATION);
  map.insert("Color", WB_SF_COLOR);
  map.insert("String", WB_SF_STRING);
  map.insert("Node", WB_SF_NODE);
  map.insert("MultiBoolean", WB_MF_BOOL);
  map.insert("MultiInt", WB_MF_INT32);
  map.insert("MultiFloat", WB_MF_FLOAT);
  map.insert("MultiVector2", WB_MF_VEC2F);
  map.insert("MultiVector3", WB_MF_VEC3F);
  map.insert("MultiColor", WB_MF_COLOR);
  map.insert("MultiRotation", WB_MF_ROTATION);
  map.insert("MultiString", WB_MF_STRING);
  map.insert("MultiNode", WB_MF_NODE);
  return map;
}

static QMap<QString, WbFieldType> gVrmlMap = initgVrmlMap();
static QMap<QString, WbFieldType> gShortMap = initShortMap();

OmValue::OmValue() {
}

OmValue::~OmValue() {
}

WbFieldType OmValue::singleType() const {
  return OmValue::toSingle(type());
}

bool OmValue::isSingle(WbFieldType type) {
  return (type & WB_MF) == 0;
}

bool OmValue::isMultiple(WbFieldType type) {
  return (type & WB_MF) == WB_MF;
}

WbFieldType OmValue::toSingle(WbFieldType type) {
  return WbFieldType(type & (~WB_MF));
}

QString OmValue::vrmlTypeName() const {
  return typeToVrmlName(type());
}

QString OmValue::shortTypeName() const {
  return typeToShortName(type());
}

WbFieldType OmValue::vrmlNameToType(const QString &vrmlName) {
  return gVrmlMap.value(vrmlName, WB_NO_FIELD);
}

QString OmValue::typeToVrmlName(WbFieldType type) {
  return gVrmlMap.key(type, "Unknown");
}

QString OmValue::typeToShortName(WbFieldType type) {
  return gShortMap.key(type, "Unknown");
}
