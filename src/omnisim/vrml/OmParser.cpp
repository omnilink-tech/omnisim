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

#include "OmParser.hpp"

#include "OmApplicationInfo.hpp"
#include "OmFieldModel.hpp"
#include "OmLog.hpp"
#include "OmNodeModel.hpp"
#include "OmProtoManager.hpp"
#include "OmProtoModel.hpp"
#include "OmProtoTemplateEngine.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QRegularExpression>

#include <cassert>

static double cLegacyGravity = 9.81;

double OmParser::legacyGravity() {
  return cLegacyGravity;
}

OmParser::OmParser(OmTokenizer *tokenizer) : mTokenizer(tokenizer), mProto(false) {
}

const QString &OmParser::fileName() const {
  return mTokenizer->fileName();
}

OmParser::~OmParser() {
}

void OmParser::parseDoubles(int n) {
  for (int i = 0; i < n; ++i) {
    if (!nextToken()->isNumeric())
      reportUnexpected(QObject::tr("floating point value"));
  }
}

const QString OmParser::parseUrl() {
  if (!peekToken()->isString())
    reportUnexpected(QObject::tr("string literal"));
  const QString url = nextToken()->toString();
  if (!url.toLower().endsWith(".proto")) {
    mTokenizer->reportError(QObject::tr("Expected URL to end with '.proto' or '.PROTO'"));
    throw 0;
  }

  return url;
}

void OmParser::parseInt() {
  // this is a backwards compatibility fix for pre-8.6 files, where
  // 'filtering' could have boolean values (it now takes integral values)
  QString fieldName(mTokenizer->lastWord());
  OmToken *fieldValue = nextToken();

  if (fieldValue->isBoolean() && fieldName == "filtering" && mTokenizer->fileVersion() < OmVersion(8, 6, 0)) {
    OmLog::warning(QObject::tr("Boolean values for 'ImageTexture.filtering' field are deprecated"
                               " from Webots 8.6 onwards; the value has been converted automatically."
                               " Please update your PROTO and world files accordingly."),
                   false, OmLog::PARSING);

    bool isFilteringOn = fieldValue->toBool();
    if (isFilteringOn)
      fieldValue->setInt(4);
    else
      fieldValue->setInt(0);
  } else if (!fieldValue->isNumeric())
    reportUnexpected(QObject::tr("integer value"));
}

void OmParser::parseBool() {
  if (!nextToken()->isBoolean())
    reportUnexpected(QObject::tr("boolean value"));
}

void OmParser::parseString() {
  if (!nextToken()->isString())
    reportUnexpected(QObject::tr("string literal"));
}

void OmParser::parseSingleFieldValue(WbFieldType type, const QString &worldPath) {
  switch (type) {
    case WB_SF_INT32:
      parseInt();
      break;
    case WB_SF_FLOAT:
      parseDoubles(1);
      break;
    case WB_SF_STRING:
      parseString();
      break;
    case WB_SF_VEC2F:
      parseDoubles(2);
      break;
    case WB_SF_VEC3F:
    case WB_SF_COLOR:
      parseDoubles(3);
      break;
    case WB_SF_ROTATION:
      parseDoubles(4);
      break;
    case WB_SF_BOOL:
      parseBool();
      break;
    case WB_SF_NODE: {
      if (peekWord() == "NULL")
        skipToken();
      else
        parseNode(worldPath);
      break;
    }
    default:
      assert(false);
  }
}

void OmParser::parseFieldValue(WbFieldType type, const QString &worldPath) {
  if (OmValue::isSingle(type))
    parseSingleFieldValue(type, worldPath);
  else {
    const WbFieldType singleType = OmValue::toSingle(type);
    if (peekWord() == "[") {
      skipToken();
      while (peekWord() != "]")
        parseSingleFieldValue(singleType, worldPath);
      skipToken();
    } else
      parseSingleFieldValue(singleType, worldPath);
  }
}

void OmParser::parseFieldAcceptedValues(WbFieldType type, const QString &worldPath) {
  while (nextWord() != '}') {
    mTokenizer->ungetToken();
    OmParser::parseSingleFieldValue(type, worldPath);
    if (nextWord() != '+')
      mTokenizer->ungetToken();
  }
}

void OmParser::parseFieldDeclaration(const QString &worldPath) {
  const OmToken *const token = nextToken();
  if (token->word() != "field" && token->word() != "w3dField" && token->word() != "hiddenField" &&
      token->word() != "deprecatedField" && token->word() != "unconnectedField")
    reportUnexpected(QObject::tr("'field', 'unconnectedField', 'w3dField' or 'hiddenField' keywords"));

  // check field type
  const WbFieldType type = OmValue::vrmlNameToType(nextWord());
  if (!type)
    reportUnexpected(QObject::tr("field type name"));

  if (nextWord() == "{")
    parseFieldAcceptedValues((WbFieldType)(type & (~WB_MF)), worldPath);
  else
    mTokenizer->ungetToken();
  parseIdentifier();
  parseFieldValue(type, worldPath);
}

void OmParser::reportError(const QString &message, const OmToken *token) const {
  mTokenizer->reportError(message, token);
}

void OmParser::reportFileError(const QString &message) const {
  mTokenizer->reportFileError(message);
}

void OmParser::reportUnexpected(const QString &expected) const {
  const OmToken *const found = mTokenizer->lastToken();
  QString foundWord(found->word());
  if (foundWord.split(QRegularExpression("\\s+")).size() == 1)
    foundWord = QString("'%1'").arg(foundWord);

  QString expectedWord(expected);
  if (expectedWord.split(QRegularExpression("\\s+")).size() == 1)
    expectedWord = QString("'%1'").arg(expectedWord);

  mTokenizer->reportError(QObject::tr("Expected %1, found %2").arg(expectedWord, foundWord), found);
  throw 0;
}

bool OmParser::parseWorld(const QString &worldPath, bool (*updateProgress)(int)) {
  mTokenizer->rewind();
  try {
    while (!peekToken()->isEof()) {
      while (peekWord() == "EXTERNPROTO" || peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
        skipExternProto();

      parseNode(worldPath);
      if (!updateProgress(mTokenizer->pos() * 100 / mTokenizer->totalTokensNumber()))
        return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

void OmParser::parseProtoDefinition(const QString &worldPath) {
  parseProtoInterface(worldPath);
  parseExactWord("{");
  parseNode(worldPath);
  parseExactWord("}");
}

bool OmParser::parseObject(const QString &worldPath) {
  mTokenizer->rewind();
  try {
    parseNode(worldPath);
  } catch (...) {
    return false;
  }
  return true;
}

bool OmParser::parseNodeModel() {
  mTokenizer->rewind();
  try {
    parseIdentifier();  // node name
    parseExactWord("{");

    while (peekWord() != "}")
      parseFieldDeclaration("");

    skipToken();  // "}"
  } catch (...) {
    return false;
  }
  return true;
}

OmToken *OmParser::nextToken() {
  return mTokenizer->nextToken();
}

const QString &OmParser::nextWord() {
  return mTokenizer->nextToken()->word();
}

const QString &OmParser::peekWord() const {
  return mTokenizer->peekToken()->word();
}

OmToken *OmParser::peekToken() {
  return mTokenizer->peekToken();
}

void OmParser::skipToken() {
  assert(mTokenizer->hasMoreTokens());
  mTokenizer->nextToken();
}

void OmParser::parseEof() {
  if (nextToken()->isEof())
    return;

  reportUnexpected(QObject::tr("end of file"));
}

const QString &OmParser::parseIdentifier(const QString &expected) {
  const OmToken *token = nextToken();

  if (!token->isIdentifier())
    reportUnexpected(expected);

  return token->word();
}

void OmParser::parseExactWord(const QString &word) {
  if (nextToken()->word() == word)
    return;

  reportUnexpected(word);
}

void OmParser::parseNode(const QString &worldPath) {
  if (peekWord() == "USE") {
    skipToken();
    parseIdentifier();
    return;
  }

  if (peekWord() == "DEF") {
    skipToken();
    parseIdentifier();
  }

  QString nodeName = parseIdentifier(QObject::tr("node or PROTO name"));
  const QString &newNodeName = OmNodeModel::compatibleNodeName(nodeName);
  if (newNodeName != nodeName) {
    reportError(QObject::tr("Deprecated '%1', please use '%2' node instead").arg(nodeName).arg(newNodeName));
    nodeName = newNodeName;
  }

  const OmNodeModel *const nodeModel = OmNodeModel::findModel(nodeName);
  if (nodeModel) {
    parseExactWord("{");
    while (peekWord() != "}")
      parseField(nodeModel, worldPath);
    skipToken();  // "}";
    // if no coordinate system was explicitly set in parseField(), set the default value.
    if (nodeModel->name() == "WorldInfo" && OmProtoTemplateEngine::coordinateSystem().isEmpty()) {
      if (mTokenizer->fileVersion() < OmVersion(2020, 1, 0))  // earlier than R2020b
        OmProtoTemplateEngine::setCoordinateSystem("NUE");
      else
        OmProtoTemplateEngine::setCoordinateSystem("ENU");
    }
    return;
  }

  const QString &referral = mTokenizer->fileName().isEmpty() ? mTokenizer->referralFile() : mTokenizer->fileName();
  const OmProtoModel *const protoModel = OmProtoManager::instance()->findModel(nodeName, worldPath, referral);
  if (protoModel) {
    parseExactWord("{");
    while (peekWord() != "}")
      parseParameter(protoModel, worldPath);
    skipToken();  // "}";
    return;
  }

  reportError(QObject::tr("Skipped unknown '%1' node or PROTO").arg(nodeName));
  mTokenizer->skipNode();
}

void OmParser::parseField(const OmNodeModel *nodeModel, const QString &worldPath) {
  const QString &fieldName = parseIdentifier(QObject::tr("field name or '}'"));
  // we need to set the coordinate system to the OmProtoTemplateEngine early enough to be able to pass the "coordinate_system"
  // as a context dictionary to procedural PROTO parameter nodes that are created before the WorldInfo node.
  if (nodeModel->name() == "WorldInfo") {
    if (mTokenizer->fileVersion() >= OmVersion(2020, 1, 0) && fieldName == "coordinateSystem") {
      QString coordinateSystem = peekWord();
      if (coordinateSystem.at(0) == '"' && coordinateSystem.back() == '"') {
        coordinateSystem = coordinateSystem.mid(1, coordinateSystem.size() - 2);
        OmProtoTemplateEngine::setCoordinateSystem(coordinateSystem);
      }
    } else if (mTokenizer->fileVersion() < OmVersion(2020, 1, 0) && fieldName == "gravity") {
      const double x = nextWord().toDouble();
      const double y = nextWord().toDouble();
      const double z = peekWord().toDouble();
      cLegacyGravity = sqrt(x * x + y * y + z * z);
      reportError(QObject::tr("Found deprecated gravity vector (%1 %2 %3) in WorldInfo, using gravity vector length: %4")
                    .arg(x)
                    .arg(y)
                    .arg(z)
                    .arg(cLegacyGravity));
      mTokenizer->skipField(true);
      return;
    }
  }
  const OmFieldModel *const fieldModel = nodeModel->findFieldModel(fieldName);
  if (!fieldModel) {
    reportError(QObject::tr("Skipped unknown '%1' field in %2 node").arg(fieldName, nodeModel->name()));
    mTokenizer->skipField(true);
    return;
  }

  if (peekWord() == "IS") {
    skipToken();
    if (!mProto)
      reportError(QObject::tr("'IS' keyword is only allowed in .proto files"));

    parseIdentifier(QObject::tr("PROTO field name"));
    return;
  }

  parseFieldValue(fieldModel->type(), worldPath);
}

void OmParser::parseParameter(const OmProtoModel *protoModel, const QString &worldPath) {
  const QString &parameterName = parseIdentifier(QObject::tr("parameter name or '}'"));

  // no check on Webots-generated hidden fields
  if (parameterName == "hidden") {
    mTokenizer->nextToken();
    mTokenizer->skipField();
    return;
  }

  const OmFieldModel *const parameterModel = protoModel->findFieldModel(parameterName);
  if (!parameterModel) {
    reportError(QObject::tr("Skipped unknown '%1' parameter in %2 PROTO").arg(parameterName, protoModel->name()));
    mTokenizer->skipField(true);
    return;
  }

  // allow to pass a PROTO parameter as value of a sub PROTO node parameter
  if (peekWord() == "IS") {
    skipToken();

    parseIdentifier(QObject::tr("PROTO field name"));
    return;
  }

  parseFieldValue(parameterModel->type(), worldPath);
}

bool OmParser::parseProtoInterface(const QString &worldPath) {
  mProto = true;
  try {
    while (peekWord() == "EXTERNPROTO" || peekWord() == "IMPORTABLE")  // consume EXTERNPROTO declarations
      skipExternProto();

    parseExactWord("PROTO");
    parseIdentifier();
    parseExactWord("[");

    while (peekWord() != "]" && mTokenizer->hasMoreTokens())
      parseFieldDeclaration(worldPath);

    parseExactWord("]");

  } catch (...) {
    return false;
  }
  return true;
}

bool OmParser::parseProtoBody(const QString &worldPath) {
  mProto = true;
  try {
    parseNode(worldPath);
    parseEof();
  } catch (...) {
    return false;
  }
  return true;
}

void OmParser::skipProtoDefinition(OmTokenizer *tokenizer) {
  // we should skip instead of parsing the tokens
  // but this is used only for VRML import, and can be optimized later
  OmParser parser(tokenizer);
  parser.parseProtoDefinition("");
}

void OmParser::skipExternProto() {
  if (peekWord() == "IMPORTABLE")
    nextToken();
  if (peekWord() == "EXTERNPROTO")
    nextToken();

  const OmToken *token = nextToken();
  if (!token->isString())
    reportUnexpected("string literal");
}

QStringList OmParser::protoNodeList() {
  const int position = mTokenizer->pos();
  assert(mTokenizer->hasMoreTokens());
  mTokenizer->nextToken();  // consume the first token so that lastWord() is defined

  // in PROTO headers, field restrictions are also defined using "type{ }"
  const QStringList exceptions = {"MFBool",   "SFBool",   "SFColor", "MFColor", "SFFloat",    "MFFloat",
                                  "SFInt32",  "MFInt32",  "SFNode",  "MFNode",  "SFRotation", "MFRotation",
                                  "SFString", "MFString", "SFVec2f", "MFVec2f", "SFVec3f",    "MFVec3f"};

  QStringList protoList;
  while (mTokenizer->hasMoreTokens()) {
    if (mTokenizer->peekWord() == "{" && !protoList.contains(mTokenizer->lastWord()) &&
        !OmNodeModel::isBaseModelName(OmNodeModel::compatibleNodeName(mTokenizer->lastWord())) &&
        !exceptions.contains(mTokenizer->lastWord()))
      protoList << mTokenizer->lastWord();

    mTokenizer->nextToken();
  }

  mTokenizer->seek(position);
  return protoList;
}
