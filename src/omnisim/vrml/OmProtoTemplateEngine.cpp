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

#include "OmProtoTemplateEngine.hpp"

#include "OmApplicationInfo.hpp"
#include "OmStandardPaths.hpp"

#include <QtCore/QRegularExpression>
#include "OmField.hpp"
#include "OmMultipleValue.hpp"
#include "OmNode.hpp"
#include "OmNodeModel.hpp"
#include "OmProject.hpp"
#include "OmProtoModel.hpp"
#include "OmRgb.hpp"
#include "OmRotation.hpp"
#include "OmSingleValue.hpp"
#include "OmVariant.hpp"
#include "OmVector2.hpp"
#include "OmVector3.hpp"
#include "OmVersion.hpp"

#include <QtCore/QVector>

#include <cassert>

static QString gCoordinateSystem;

OmProtoTemplateEngine::OmProtoTemplateEngine(const QString &templateContent) : OmTemplateEngine(templateContent) {
}

bool OmProtoTemplateEngine::generate(const QString &logHeaderName, const QVector<OmField *> &parameters,
                                     const QString &protoPath, const QString &worldPath, int id) {
  // generate the final script file from the template script file
  QHash<QString, QString> tags;

  tags["fields"] = "";
  foreach (const OmField *parameter, parameters) {
    if (!parameter->isTemplateRegenerator())  // keep only regenerator fields
      continue;
    const QString &valueString = convertFieldValueToJavaScriptStatement(parameter);
    if (!valueString.isEmpty()) {
      tags["fields"] += QString("%1: {").arg(parameter->name());
      tags["fields"] += QString("value: %1, ").arg(valueString);
      tags["fields"] += QString("defaultValue: %1").arg(convertFieldDefaultValueToJavaScriptStatement(parameter));
    }
    tags["fields"] += "},\n";
  }
  tags["fields"].chop(2);  // remove the last ",\n" if any

#ifdef _WIN32
  tags["context"] = QString("os: 'windows', ");
#endif
#ifdef __linux__
  tags["context"] = QString("os: 'linux', ");
#endif
#ifdef __APPLE__
  tags["context"] = QString("os: 'mac', ");
#endif
  tags["context"] += QString("world: '%1', ").arg(escapeString(worldPath));
  tags["context"] += QString("proto: '%1', ").arg(escapeString(protoPath));
  tags["context"] += QString("webots_home: '%1', ").arg(escapeString(OmStandardPaths::omniSimHomePath()));
  tags["context"] += QString("project_path: '%1', ").arg(escapeString(OmProject::current()->path()));
  tags["context"] += QString("temporary_files_path: '%1', ").arg(escapeString(OmStandardPaths::webotsTmpPath()));
  tags["context"] += QString("id: '%1', ").arg(id);
  tags["context"] += QString("coordinate_system: '%1', ").arg(gCoordinateSystem);
  OmVersion version = OmApplicationInfo::version();
  // for example major = R2018a and revision = 0
  tags["context"] +=
    QString("webots_version: {major: '%1', revision: '%2'}").arg(version.toString(false)).arg(version.revisionNumber());

  return OmTemplateEngine::generate(tags, logHeaderName);
}

void OmProtoTemplateEngine::setCoordinateSystem(const QString &coordinateSystem) {
  gCoordinateSystem = coordinateSystem;
}

const QString &OmProtoTemplateEngine::coordinateSystem() {
  return gCoordinateSystem;
}

QString OmProtoTemplateEngine::convertFieldValueToJavaScriptStatement(const OmField *field) {
  if (field->isSingle()) {
    const OmSingleValue *singleValue = dynamic_cast<const OmSingleValue *>(field->value());
    assert(singleValue);
    const OmVariant &variant = singleValue->variantValue();
    return convertVariantToJavaScriptStatement(variant);
  } else if (field->isMultiple()) {
    const OmMultipleValue *multipleValue = dynamic_cast<const OmMultipleValue *>(field->value());
    assert(multipleValue);
    // multiple values into a JavaScript array
    QString result = "[";
    for (int i = 0; i < multipleValue->size(); ++i) {
      if (i != 0)
        result += ", ";
      const OmVariant &variant = multipleValue->variantValue(i);
      result += convertVariantToJavaScriptStatement(variant);
    }
    result += "]";
    return result;
  }

  assert(false);
  return "";
}

QString OmProtoTemplateEngine::convertFieldDefaultValueToJavaScriptStatement(const OmField *field) {
  if (field->isSingle()) {
    const OmSingleValue *singleValue = dynamic_cast<const OmSingleValue *>(field->defaultValue());
    assert(singleValue);
    const OmVariant &variant = singleValue->variantValue();
    return convertVariantToJavaScriptStatement(variant);
  }

  else if (field->isMultiple()) {
    const OmMultipleValue *multipleValue = dynamic_cast<const OmMultipleValue *>(field->defaultValue());
    assert(multipleValue);
    // multiple values into a JavaScript array
    QString result = "[";
    for (int i = 0; i < multipleValue->size(); ++i) {
      if (i != 0)
        result += ", ";
      const OmVariant &variant = multipleValue->variantValue(i);
      result += convertVariantToJavaScriptStatement(variant);
    }
    result += "]";
    return result;
  }

  assert(false);
  return "";
}

QString OmProtoTemplateEngine::convertVariantToJavaScriptStatement(const OmVariant &variant) {
  switch (variant.type()) {
    case WB_SF_BOOL:
      return QString("%1").arg(variant.toBool() ? "true" : "false");
    case WB_SF_INT32:
      return QString("%1").arg(variant.toInt());
    case WB_SF_FLOAT:
      return QString("%1").arg(variant.toDouble());
    case WB_SF_VEC2F:
      return QString("{x: %1, y: %2}").arg(variant.toVector2().x()).arg(variant.toVector2().y());
    case WB_SF_VEC3F:
      return QString("{x: %1, y: %2, z: %3}")
        .arg(variant.toVector3().x())
        .arg(variant.toVector3().y())
        .arg(variant.toVector3().z());
    case WB_SF_ROTATION:
      return QString("{x: %1, y: %2, z: %3, a: %4}")
        .arg(variant.toRotation().x())
        .arg(variant.toRotation().y())
        .arg(variant.toRotation().z())
        .arg(variant.toRotation().angle());
    case WB_SF_COLOR:
      return QString("{r: %1, g: %2, b: %3}")
        .arg(variant.toColor().red())
        .arg(variant.toColor().green())
        .arg(variant.toColor().blue());
    case WB_SF_STRING: {
      QString string = variant.toString();
      string.replace("\\\"", "\"");  // make sure that if a double
      // quote is already protected we don't 'break' this potection
      string.replace("\"", "\\\"");
      return QString("'%1'").arg(string);
    }
    case WB_SF_NODE: {
      OmNode *node = variant.toNode();
      if (node) {
        // javascript object
        QString nodeString = "{";

        // node_name key
        nodeString += QString("node_name: '%1'").arg(node->modelName());
        nodeString += ", ";

        // fields key: fieldName = fieldValue
        nodeString += "fields: {";
        foreach (const OmField *field, node->fieldsOrParameters()) {
          if (field->name() != "node_name") {
            nodeString += QString("%1: {").arg(field->name());
            nodeString += QString("value: %1, ").arg(convertFieldValueToJavaScriptStatement(field));
            nodeString += QString("defaultValue: %1").arg(convertFieldDefaultValueToJavaScriptStatement(field));
            if (field != node->fieldsOrParameters().last())
              nodeString += "}, ";
            else
              nodeString += "}";
          }
        }
        nodeString += "}";  // end of fields

        nodeString += "}";  // end of nodes

        return nodeString;
      } else
        return "undefined";
    }
    default:
      assert(false);
      return "";
  }
}
