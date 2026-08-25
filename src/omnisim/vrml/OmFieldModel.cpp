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

#include "OmFieldModel.hpp"

#include "OmMFBool.hpp"
#include "OmMFColor.hpp"
#include "OmMFDouble.hpp"
#include "OmMFInt.hpp"
#include "OmMFNode.hpp"
#include "OmMFRotation.hpp"
#include "OmMFString.hpp"
#include "OmMFVector2.hpp"
#include "OmMFVector3.hpp"

#include "OmSFBool.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFNode.hpp"
#include "OmSFRotation.hpp"
#include "OmSFString.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"

#include "OmNode.hpp"
#include "OmToken.hpp"
#include "OmTokenizer.hpp"
#include "OmValue.hpp"
#include "OmWriter.hpp"

#include <cassert>

OmFieldModel::OmFieldModel(OmTokenizer *tokenizer, const QString &worldPath) {
  QString nw(tokenizer->nextWord());
  if (nw != "field" && nw != "w3dField" && nw != "hiddenField" && nw != "hidden" && nw != "deprecatedField" &&
      nw != "unconnectedField") {
    tokenizer->reportError(QObject::tr("Expected field type but found '%2'").arg(nw), tokenizer->lastToken());
    throw 0;
  }

  mIsW3d = nw == "w3dField";
  mIsDeprecated = nw == "deprecatedField";
  mIsHiddenField = mIsDeprecated || nw == "hiddenField";
  mIsHiddenParameter = nw == "hidden";
  mIsUnconnected = nw == "unconnectedField";
  mRefCount = 0;
  mIsTemplateRegenerator = false;

  QString typeName;
  if (mIsHiddenParameter) {
    nw = tokenizer->peekWord();

    if (nw.startsWith("rotation"))
      typeName = "SFRotation";
    else if (nw.startsWith("translation"))
      typeName = "SFVec3f";
    else if (nw.startsWith("position"))  // position or position2
      typeName = "SFFloat";
    else if (nw.startsWith("linearVelocity"))
      typeName = "SFVec3f";
    else if (nw.startsWith("angularVelocity"))
      typeName = "SFVec3f";
    else {
      tokenizer->reportError(QObject::tr("Expected hidden field identifier but found '%2'").arg(nw), tokenizer->lastToken());
      throw 0;
    }
  } else
    typeName = tokenizer->nextWord();

  if (tokenizer->nextWord() == "{") {
    QString singleTypeName = typeName;
    singleTypeName.replace("MF", "SF");
    mAcceptedValues = getAcceptedValues(singleTypeName, tokenizer, worldPath);
  } else
    tokenizer->ungetToken();

  // copy the token, indeed, the pointer reference can be deleted by the tokenizer
  mNameToken = new OmToken(*(tokenizer->nextToken()));
  mName = mNameToken->word();
  mDefaultValue = createValueForVrmlType(typeName, tokenizer, worldPath);
  if (mDefaultValue == NULL) {
    tokenizer->reportError(QObject::tr("Expected VRML97 type but found '%2'").arg(typeName), tokenizer->lastToken());
    throw 0;
  }

  if (hasRestrictedValues()) {
    int refusedIndex;
    bool defaultValueIsValid = true;
    while (!isValueAccepted(mDefaultValue, &refusedIndex)) {
      defaultValueIsValid = false;
      const OmMultipleValue *multipleValue = dynamic_cast<OmMultipleValue *>(mDefaultValue);
      if (multipleValue)
        mAcceptedValues << OmFieldValueRestriction(multipleValue->variantValue(refusedIndex), false);
      else {
        OmSingleValue *singleValue = dynamic_cast<OmSingleValue *>(mDefaultValue);
        assert(singleValue);
        mAcceptedValues << OmFieldValueRestriction(singleValue->variantValue(), false);
      }
    }
    if (!defaultValueIsValid)
      tokenizer->reportError(QObject::tr("The default value of field '%1' is not in the list of accepted values").arg(mName),
                             tokenizer->lastToken());
  }
}

OmFieldModel::~OmFieldModel() {
  delete mDefaultValue;
  delete mNameToken;
}

void OmFieldModel::destroy() {
  assert(mRefCount == 0);
  delete this;
}

void OmFieldModel::ref() const {
  mRefCount++;
}

void OmFieldModel::unref() const {
  mRefCount--;
  if (mRefCount == 0)
    delete this;
}

OmValue *OmFieldModel::createValueForVrmlType(const QString &type, OmTokenizer *tokenizer, const QString &worldPath) {
  if (type == "SFString")
    return new OmSFString(tokenizer, worldPath);
  else if (type == "SFInt32")
    return new OmSFInt(tokenizer, worldPath);
  else if (type == "SFFloat")
    return new OmSFDouble(tokenizer, worldPath);
  else if (type == "SFVec2f")
    return new OmSFVector2(tokenizer, worldPath);
  else if (type == "SFVec3f")
    return new OmSFVector3(tokenizer, worldPath);
  else if (type == "SFColor")
    return new OmSFColor(tokenizer, worldPath);
  else if (type == "SFNode")
    return new OmSFNode(tokenizer, worldPath);
  else if (type == "SFBool")
    return new OmSFBool(tokenizer, worldPath);
  else if (type == "SFRotation")
    return new OmSFRotation(tokenizer, worldPath);
  else if (type == "MFString")
    return new OmMFString(tokenizer, worldPath);
  else if (type == "MFInt32")
    return new OmMFInt(tokenizer, worldPath);
  else if (type == "MFFloat")
    return new OmMFDouble(tokenizer, worldPath);
  else if (type == "MFVec2f")
    return new OmMFVector2(tokenizer, worldPath);
  else if (type == "MFVec3f")
    return new OmMFVector3(tokenizer, worldPath);
  else if (type == "MFColor")
    return new OmMFColor(tokenizer, worldPath);
  else if (type == "MFNode")
    return new OmMFNode(tokenizer, worldPath);
  else if (type == "MFBool")
    return new OmMFBool(tokenizer, worldPath);
  else if (type == "MFRotation")
    return new OmMFRotation(tokenizer, worldPath);
  else
    return NULL;
}

QList<OmFieldValueRestriction> OmFieldModel::getAcceptedValues(const QString &type, OmTokenizer *tokenizer,
                                                               const QString &worldPath) {
  QList<OmFieldValueRestriction> values;
  while (tokenizer->nextWord() != '}') {
    tokenizer->ungetToken();

    const OmSingleValue *singleValue =
      dynamic_cast<const OmSingleValue *>(OmFieldModel::createValueForVrmlType(type, tokenizer, worldPath));
    assert(singleValue);

    bool allowSubtypeMatch = tokenizer->peekWord() == '+';
    if (allowSubtypeMatch)
      tokenizer->nextToken();

    OmFieldValueRestriction restriction(singleValue->variantValue(), allowSubtypeMatch);
    if (type == "SFNode" && restriction.toNode()) {
      // explicit copy of the node to be persistent.
      OmNode *copy = restriction.toNode()->cloneAndReferenceProtoInstance();
      restriction.setNode(copy, true);
      QObject::connect(&restriction, &QObject::destroyed, copy, &QObject::deleteLater);
    }

    values << restriction;
    delete singleValue;
  }
  return values;
}

bool OmFieldModel::isValueAccepted(const OmValue *value, int *refusedIndex) const {
  *refusedIndex = -1;
  if (mAcceptedValues.isEmpty())
    return true;
  const OmMultipleValue *multipleValue = dynamic_cast<const OmMultipleValue *>(value);
  const OmSingleValue *singleValue = dynamic_cast<const OmSingleValue *>(value);
  if (multipleValue) {
    for (int i = 0; i < multipleValue->size(); ++i) {
      bool accepted = false;
      foreach (const OmFieldValueRestriction acceptedVariant, mAcceptedValues) {
        if (acceptedVariant.isVariantAccepted(multipleValue->variantValue(i))) {
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        *refusedIndex = i;
        return false;
      }
    }
    return true;
  } else {
    assert(singleValue);
    foreach (const OmFieldValueRestriction acceptedVariant, mAcceptedValues) {
      if (acceptedVariant.isVariantAccepted(singleValue->variantValue()))
        return true;
    }
    *refusedIndex = 0;
    return false;
  }
}

bool OmFieldModel::isMultiple() const {
  return dynamic_cast<OmMultipleValue *>(mDefaultValue);
}

bool OmFieldModel::isSingle() const {
  return dynamic_cast<OmSingleValue *>(mDefaultValue);
}

void OmFieldModel::write(OmWriter &writer) const {
  writer << "field " << mDefaultValue->vrmlTypeName() << " " << mName << " ";
  mDefaultValue->write(writer);
}
