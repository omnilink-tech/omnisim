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

#ifndef OM_FIELD_HPP
#define OM_FIELD_HPP

//
// Description: a node's field
//   Each field contains a OmValue
//

#include <QtCore/QList>
#include <QtCore/QObject>
#include <QtCore/QString>
#include "../../../include/controller/c/omnisim/supervisor.h"

#include "OmFieldModel.hpp"
#include "OmPrecision.hpp"

class OmNode;  // circular dependency: needed by PROTO mechanism for easily retrieving the parent of internal fields
class OmTokenizer;
class OmWriter;
class OmValue;

class OmField : public QObject {
  Q_OBJECT

public:
  // create from a field model
  explicit OmField(const OmFieldModel *model, OmNode *parentNode = NULL);

  // create by copying another field
  explicit OmField(const OmField &other, OmNode *parentNode = NULL);
  virtual ~OmField();

  // the field's model
  const OmFieldModel *model() const { return mModel; }

  // default value
  const OmValue *defaultValue() const;
  virtual bool isDefault() const;
  // reset to default value
  // for SFNode and MFNode only sets value to NULL
  virtual void reset(bool blockValueSignals = false);

  // write in VRML format
  virtual void write(OmWriter &writer) const;
  bool isW3d() const;

  bool isDeprecated() const;

  // read field value
  void readValue(OmTokenizer *tokenizer, const QString &worldPath);

  // optional redirection to a proto parameter
  void setAlias(const QString &alias) { mAlias = alias; }
  const QString &alias() const { return mAlias; }
  void redirectTo(OmField *parameter, bool skipCopy = false);
  OmField *parameter() const { return mParameter; }
  const QList<OmField *> &internalFields() const { return mInternalFields; }
  bool isParameter() const;

  void clearInternalFields() { mInternalFields.clear(); }

  void setParentNode(OmNode *node) { mParentNode = node; }
  OmNode *parentNode() const { return mParentNode; }

  // template
  void setTemplateRegenerator(bool isRegenerator) { mIsTemplateRegenerator = isRegenerator; }
  bool isTemplateRegenerator() const { return mIsTemplateRegenerator; }

  // the field's name
  const QString &name() const;

  // the field's value (this pointer is never NULL)
  OmValue *value() const { return mValue; }

  // set new value (types must match) in multiple steps to update view
  void setValue(const OmValue *otherValue);
  // copy value from another field (the types must be the same) in one step
  void copyValueFrom(const OmField *other);

  // create WREN and ODE objects in USE node fields when the corresponding DEF node has changed
  void defHasChanged();

  // convert to string for the GUI, e.g. "position 0.1 2 3"
  QString toString(OmPrecision::Level level) const;

  // the field's type (shortcuts for OmValue)
  WbFieldType type() const;        // e.g. WB_MF_NODE
  WbFieldType singleType() const;  // e.g. returns WB_SF_NODE for a WB_MF_NODE
  bool isMultiple() const;
  bool isSingle() const;
  bool isHidden() const;
  bool isHiddenParameter() const;

  // accepted values
  bool hasRestrictedValues() const { return mModel->hasRestrictedValues(); }
  const QList<OmFieldValueRestriction> acceptedValues() const { return mModel->acceptedValues(); }

  // enable forwarding signals when the size of MF fields changes
  void listenToValueSizeChanges() const;

  const QString &scope() const { return mScope; }
  void setScope(const QString &value) { mScope = value; }

signals:
  void valueChanged();
  void valueChangedByOde();
  void valueChangedByOmniSim();
  void valueSizeChanged();

protected:
private:
  OmField &operator=(const OmField &);  // non-copyable
  const OmFieldModel *mModel;           // field model (name and default value)
  OmValue *mValue;                      // field value (never NULL)
  bool mWasRead;                        // true if the value was read from file

  // for proto definition only
  OmField *mParameter;  // optional connection to a proto parameter
  QString mAlias;       // IS string
  bool mIsTemplateRegenerator;

  // for proto parameter only
  QList<OmField *> mInternalFields;  // internal fields towards which a parameter is redirecting its value

  // for internal fields only
  OmNode *mParentNode;

  QString mScope;

private slots:
  void parameterChanged();
  void parameterNodeInserted(int index);
  void parameterNodeRemoved(int index);
  void parameterNodeChanged(int index);
  void fieldChanged();
  void fieldChangedByOde();
  void removeInternalField(QObject *field);
  void checkValueIsAccepted();
};

#endif
