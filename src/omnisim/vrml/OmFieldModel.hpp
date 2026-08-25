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

#ifndef OM_FIELD_MODEL_HPP
#define OM_FIELD_MODEL_HPP

//
// Description: a class that defines a model for a node's field
//   The model is used in OmNodeModel.
//

#include <QtCore/QString>
#include <OmFieldValueRestriction.hpp>
#include <OmNodeModel.hpp>
#include <OmProtoModel.hpp>
#include <OmValue.hpp>
#include <OmVariant.hpp>

class OmTokenizer;
class OmToken;
class OmWriter;

class OmFieldModel {
public:
  // create from tokenizer
  OmFieldModel(OmTokenizer *tokenizer, const QString &worldPath);

  // field name
  const QString &name() const { return mName; }

  // W3D export
  bool isW3d() const { return mIsW3d; }
  void write(OmWriter &writer) const;

  bool isDeprecated() const { return mIsDeprecated; }

  // Hidden field flag
  bool isHiddenField() const { return mIsHiddenField; }
  bool isHiddenParameter() const { return mIsHiddenParameter; }

  bool isUnconnected() const { return mIsUnconnected; }

  // default value
  OmValue *defaultValue() const { return mDefaultValue; }

  // accepted values
  bool isValueAccepted(const OmValue *value, int *refusedIndex) const;
  bool hasRestrictedValues() const { return !mAcceptedValues.isEmpty(); }
  const QList<OmFieldValueRestriction> &acceptedValues() const { return mAcceptedValues; }

  // field type
  WbFieldType type() const { return mDefaultValue->type(); }
  bool isMultiple() const;
  bool isSingle() const;

  // useful tokens for error reporting
  OmToken *nameToken() const { return mNameToken; }

  // template
  void setTemplateRegenerator(bool isRegenerator) { mIsTemplateRegenerator = isRegenerator; }
  bool isTemplateRegenerator() const { return mIsTemplateRegenerator; }

  // add/remove a reference to this field model from a field, a proto model or a node model instance
  // when the reference count reaches zero (in unref()) the field model is deleted
  void ref() const;
  void unref() const;

  // delete this field model
  // reference count has to be zero
  void destroy();

private:
  OmFieldModel(const OmFieldModel &);             // non constructor-copyable
  OmFieldModel &operator=(const OmFieldModel &);  // non copyable
  ~OmFieldModel();

  QString mName;
  bool mIsW3d;
  bool mIsHiddenField, mIsHiddenParameter;
  bool mIsTemplateRegenerator;
  bool mIsDeprecated;
  bool mIsUnconnected;
  OmValue *mDefaultValue;
  QList<OmFieldValueRestriction> mAcceptedValues;  // TODO: const OmVariant
  OmToken *mNameToken;

  mutable int mRefCount;

  static OmValue *createValueForVrmlType(const QString &type, OmTokenizer *tokenizer, const QString &worldPath);
  static QList<OmFieldValueRestriction> getAcceptedValues(const QString &type, OmTokenizer *tokenizer,
                                                          const QString &worldPath);
};

#endif
