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

#ifndef OM_PROTO_TEMPLATE_ENGINE_HPP
#define OM_PROTO_TEMPLATE_ENGINE_HPP

//
// Description:    template engine
// Responsability: manage file parsing using a template engine given a VRML context

#include <QtCore/QObject>
#include <QtCore/QString>

#include "OmTemplateEngine.hpp"

class OmField;
class OmVariant;

class OmProtoTemplateEngine : public OmTemplateEngine {
  Q_OBJECT

public:
  explicit OmProtoTemplateEngine(const QString &templateContent);
  virtual ~OmProtoTemplateEngine() override {}

  bool generate(const QString &logHeaderName, const QVector<OmField *> &parameters, const QString &protoPath,
                const QString &worldPath, int id);
  static QString convertFieldValueToJavaScriptStatement(const OmField *field);
  static const QString &coordinateSystem();
  static void setCoordinateSystem(const QString &coordinateSystem);

private:
  static QString convertFieldDefaultValueToJavaScriptStatement(const OmField *field);
  static QString convertVariantToJavaScriptStatement(const OmVariant &variant);
};

#endif
