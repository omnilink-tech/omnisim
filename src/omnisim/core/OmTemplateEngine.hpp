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

#ifndef OM_TEMPLATE_ENGINE_HPP
#define OM_TEMPLATE_ENGINE_HPP

//
// Description:    template engine
// Responsability: manage file parsing using a template engine given a VRML context

#include <QtCore/QObject>
#include <QtCore/QString>

// Load-time split of the JavaScript template path (OMNISIM_RELOAD_PROFILE=1), cumulative ns:
// [0] translate template -> JS text, [1] write jsTemplateFilled.js, [2] QJSEngine construction,
// [3] importModule (parse + compile), [4] generateVrml() call. Reported by OmApplication.
struct OmTemplateEngineProfile {
  qint64 ns[5] = {0, 0, 0, 0, 0};
  qint64 bytes = 0;       // filled-template bytes handed to importModule, cumulative
  qint64 resultHash = 0;  // order-sensitive hash of every generated result (A/B proof of identity)
  int calls = 0;
  static OmTemplateEngineProfile &instance();
};

class OmTemplateEngine : public QObject {
  Q_OBJECT

public:
  static const QString &openingToken();
  static const QString &closingToken();

  static QString escapeString(const QString &string);

  explicit OmTemplateEngine(const QString &templateContent);
  virtual ~OmTemplateEngine() {}

  bool generate(QHash<QString, QString> tags, const QString &logHeaderName);

  const QByteArray &result() { return mResult; }

  const QString &error() const { return mError; }

private:
  static void initializeJavaScript();

  bool generateJavascript(QHash<QString, QString> tags, const QString &logHeaderName);

  QString mTemplateContent;
  QString mError;
  QByteArray mResult;
};

#endif
