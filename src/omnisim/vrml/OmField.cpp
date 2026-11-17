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

#include "OmField.hpp"

#include "OmFieldModel.hpp"
#include "OmLog.hpp"
#include "OmMFBool.hpp"
#include "OmMFColor.hpp"
#include "OmMFDouble.hpp"
#include "OmMFInt.hpp"
#include "OmMFNode.hpp"
#include "OmMFRotation.hpp"
#include "OmMFString.hpp"
#include "OmMFVector2.hpp"
#include "OmMFVector3.hpp"
#include "OmMultipleValue.hpp"
#include "OmNode.hpp"
#include "OmSFDouble.hpp"
#include "OmSFNode.hpp"
#include "OmSFRotation.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"
#include "OmTokenizer.hpp"
#include "OmValue.hpp"
#include "OmWriter.hpp"

#include <cassert>
#include <iostream>

// creates with the default value
OmField::OmField(const OmFieldModel *model, OmNode *parentNode) :
  mModel(model),
  mValue(model->defaultValue()->clone()),
  mWasRead(false),
  mParameter(NULL),
  mIsTemplateRegenerator(model->isTemplateRegenerator()),
  mParentNode(parentNode) {
  mModel->ref();
  if (hasRestrictedValues())
    connect(mValue, &OmValue::changed, this, &OmField::checkValueIsAccepted, Qt::UniqueConnection);
  connect(mValue, &OmValue::changed, this, &OmField::valueChanged, Qt::UniqueConnection);
  connect(mValue, &OmValue::changedByOde, this, &OmField::valueChangedByOde, Qt::UniqueConnection);
  connect(mValue, &OmValue::changedByOmniSim, this, &OmField::valueChangedByOmniSim, Qt::UniqueConnection);
}

OmField::OmField(const OmField &other, OmNode *parentNode) :
  mModel(other.mModel),
  mValue(other.value()->clone()),
  mWasRead(false),
  mParameter(NULL),
  mAlias(other.mAlias),
  mIsTemplateRegenerator(other.mIsTemplateRegenerator),
  mParentNode(parentNode),
  mScope(other.mScope) {
  mModel->ref();
  if (hasRestrictedValues())
    connect(mValue, &OmValue::changed, this, &OmField::checkValueIsAccepted, Qt::UniqueConnection);
  connect(mValue, &OmValue::changed, this, &OmField::valueChanged, Qt::UniqueConnection);
  connect(mValue, &OmValue::changedByOde, this, &OmField::valueChangedByOde, Qt::UniqueConnection);
  connect(mValue, &OmValue::changedByOmniSim, this, &OmField::valueChangedByOmniSim, Qt::UniqueConnection);
}

OmField::~OmField() {
  foreach (OmField *const field, mInternalFields)
    field->mParameter = NULL;
  delete mValue;
  mModel->unref();
}

void OmField::listenToValueSizeChanges() const {
  if (singleType() == WB_SF_NODE) {
    OmSFNode *sfnode = static_cast<OmSFNode *>(mValue);
    connect(sfnode, &OmSFNode::changed, this, &OmField::valueSizeChanged, Qt::UniqueConnection);
    return;
  }
  if (isSingle())
    return;
  const OmMultipleValue *mf = static_cast<OmMultipleValue *>(mValue);
  connect(mf, &OmMultipleValue::itemRemoved, this, &OmField::valueSizeChanged, Qt::UniqueConnection);
  connect(mf, &OmMultipleValue::itemInserted, this, &OmField::valueSizeChanged, Qt::UniqueConnection);
}

const QString &OmField::name() const {
  return mModel->name();
}

bool OmField::isW3d() const {
  return mModel->isW3d();
}

bool OmField::isDeprecated() const {
  return mModel->isDeprecated();
}

// Because of unconnected fields, the only way to definitively check if a field is a parameter is to check its parent node
// If that is not possible, fallback to the old behavior (See #6604 and #6735)
bool OmField::isParameter() const {
  return parentNode() ? parentNode()->isProtoInstance() : !mInternalFields.isEmpty();
}

void OmField::readValue(OmTokenizer *tokenizer, const QString &worldPath) {
  if (mWasRead)
    tokenizer->reportError(tr("Duplicate field value: '%1'").arg(name()));

  mValue->read(tokenizer, worldPath);
  mWasRead = true;
  if (hasRestrictedValues())
    checkValueIsAccepted();
}

void OmField::write(OmWriter &writer) const {
  if (isDefault())
    return;
  if (writer.isW3d())
    writer << " ";
  const bool notAString = type() != WB_SF_STRING;
  writer.writeFieldStart(name(), notAString);
  mValue->write(writer);
  writer.writeFieldEnd(notAString);
}

const OmValue *OmField::defaultValue() const {
  return mModel->defaultValue();
}

bool OmField::isDefault() const {
  return mValue->equals(mModel->defaultValue());
}

void OmField::reset(bool blockValueSignals) {
  if (singleType() == WB_SF_NODE) {
    // clear field and set value to NULL or []
    // but new default node instances have to be created separately
    OmSFNode *sfnode = dynamic_cast<OmSFNode *>(mValue);
    OmMFNode *mfnode = dynamic_cast<OmMFNode *>(mValue);
    if (sfnode) {
      if (blockValueSignals) {
        sfnode->blockSignals(true);
        sfnode->removeValue();
        sfnode->blockSignals(false);
      } else
        sfnode->removeValue();
    } else if (mfnode) {
      // remove all children
      const int n = mfnode->size() - 1;
      if (blockValueSignals)
        mfnode->blockSignals(true);
      for (int i = n; i >= 0; --i)
        mfnode->removeItem(i);
      if (blockValueSignals)
        mfnode->blockSignals(false);
    }
    return;
  }

  setValue(mModel->defaultValue());
}

void OmField::checkValueIsAccepted() {
  int refusedIndex;
  if (!mModel->isValueAccepted(mValue, &refusedIndex)) {
    QString acceptedValuesList = "";
    foreach (const OmFieldValueRestriction acceptedValue, mModel->acceptedValues())
      acceptedValuesList +=
        acceptedValue.toSimplifiedStringRepresentation() + (acceptedValue.allowsSubtypes() ? "+" : "") + ", ";
    acceptedValuesList.chop(2);
    QString error;
    if (isSingle()) {
      error = tr("Invalid '%1' changed to %2. The value should be in the list: {%3}.")
                .arg(name())
                .arg(defaultValue()->toString())
                .arg(acceptedValuesList);
      reset(true);
    } else {
      OmMultipleValue *mvalue = dynamic_cast<OmMultipleValue *>(mValue);
      assert(mvalue);
      error = tr("Invalid '%1' removed from '%2' field. The values should be in the list: {%3}.")
                .arg(mvalue->itemToString(refusedIndex))
                .arg(name())
                .arg(acceptedValuesList);
      mvalue->removeItem(refusedIndex);
    }
    if (parentNode())
      parentNode()->parsingWarn(error);
    else
      OmLog::warning(error, false, OmLog::PARSING);
  }
}

void OmField::setValue(const OmValue *otherValue) {
  OmMultipleValue *mvalue = dynamic_cast<OmMultipleValue *>(mValue);
  if (mvalue) {
    // remove all children
    mvalue->clear();

    // add default children
    switch (mvalue->type()) {
      case WB_MF_NODE: {
        const OmMFNode *const otherField = dynamic_cast<const OmMFNode *>(otherValue);
        OmMFNode *const actualField = dynamic_cast<OmMFNode *>(mvalue);
        OmMFIterator<OmMFNode, OmNode *> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_VEC2F: {
        const OmMFVector2 *const otherField = dynamic_cast<const OmMFVector2 *>(otherValue);
        OmMFVector2 *const actualField = dynamic_cast<OmMFVector2 *>(mvalue);
        OmMFIterator<OmMFVector2, OmVector2> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_VEC3F: {
        const OmMFVector3 *const otherField = dynamic_cast<const OmMFVector3 *>(otherValue);
        OmMFVector3 *const actualField = dynamic_cast<OmMFVector3 *>(mvalue);
        OmMFIterator<OmMFVector3, OmVector3> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_ROTATION: {
        const OmMFRotation *const otherField = dynamic_cast<const OmMFRotation *>(otherValue);
        OmMFRotation *const actualField = dynamic_cast<OmMFRotation *>(mvalue);
        OmMFIterator<OmMFRotation, OmRotation> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_COLOR: {
        const OmMFColor *const otherField = dynamic_cast<const OmMFColor *>(otherValue);
        OmMFColor *const actualField = dynamic_cast<OmMFColor *>(mvalue);
        OmMFIterator<OmMFColor, OmRgb> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_STRING: {
        const OmMFString *const otherField = dynamic_cast<const OmMFString *>(otherValue);
        OmMFString *const actualField = dynamic_cast<OmMFString *>(mvalue);
        OmMFIterator<OmMFString, QString> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_BOOL: {
        const OmMFBool *const otherField = dynamic_cast<const OmMFBool *>(otherValue);
        OmMFBool *const actualField = dynamic_cast<OmMFBool *>(mvalue);
        OmMFIterator<OmMFBool, bool> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_INT32: {
        const OmMFInt *const otherField = dynamic_cast<const OmMFInt *>(otherValue);
        OmMFInt *const actualField = dynamic_cast<OmMFInt *>(mvalue);
        OmMFIterator<OmMFInt, int> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      case WB_MF_FLOAT: {
        const OmMFDouble *const otherField = dynamic_cast<const OmMFDouble *>(otherValue);
        OmMFDouble *const actualField = dynamic_cast<OmMFDouble *>(mvalue);
        OmMFIterator<OmMFDouble, double> it(otherField);
        while (it.hasNext())
          actualField->addItem(it.next());
        break;
      }
      default:
        break;
    }

  } else
    // single value
    mValue->copyFrom(otherValue);
}

QString OmField::toString(OmPrecision::Level level) const {
  return QString("%1 %2").arg(name(), mValue->toString(level));
}

WbFieldType OmField::type() const {
  return mValue->type();
}

WbFieldType OmField::singleType() const {
  return mValue->singleType();
}

bool OmField::isMultiple() const {
  return mModel->isMultiple();
}

bool OmField::isSingle() const {
  return mModel->isSingle();
}

bool OmField::isHidden() const {
  return mModel->isHiddenField();
}

bool OmField::isHiddenParameter() const {
  return mModel->isHiddenParameter();
}

// redirect this node field to a proto parameter
void OmField::redirectTo(OmField *parameter, bool skipCopy) {
  // qDebug() << "redirectTo: " << this << " " << name() << " -> " << parameter << " " << parameter->name();

  if (mParameter == parameter || this == parameter || parameter->mInternalFields.contains(this))
    // skip self and duplicated redirection
    return;

  if (mParameter) {
    // remove previous connections
    const OmMFNode *mfnode = dynamic_cast<OmMFNode *>(mParameter->value());
    if (mfnode) {
      disconnect(mfnode, &OmMFNode::itemInserted, mParameter, &OmField::parameterNodeInserted);
      disconnect(mfnode, &OmMFNode::itemRemoved, mParameter, &OmField::parameterNodeRemoved);
    } else {
      // make sure the field gets updated when the parameter changes, e.g. by Scene Tree or Supervisor, etc.
      if (mParameter->mInternalFields.size() == 1) {
        disconnect(mParameter, &OmField::valueChanged, mParameter, &OmField::parameterChanged);
        disconnect(mParameter->value(), &OmValue::changedByUser, this->value(), &OmValue::changedByUser);
      }
      disconnect(this, &OmField::valueChanged, mParameter, &OmField::fieldChanged);
    }

    // ODE updates
    const QString &fieldName = name();
    if (fieldName == "translation") {
      disconnect(static_cast<OmSFVector3 *>(mValue), &OmSFVector3::changedByOde, mParameter, &OmField::fieldChangedByOde);
      disconnect(static_cast<OmSFVector2 *>(mValue), &OmSFVector2::changedByOmniSim, mParameter, &OmField::fieldChangedByOde);
    } else if (fieldName == "rotation")
      disconnect(static_cast<OmSFRotation *>(mValue), &OmSFRotation::changedByOde, mParameter, &OmField::fieldChangedByOde);
    else if (fieldName == "position")
      disconnect(static_cast<OmSFDouble *>(mValue), &OmSFDouble::changedByOde, mParameter, &OmField::fieldChangedByOde);

    mParameter->mInternalFields.removeAll(this);
  }

  mParameter = parameter;

  assert(mParameter);
  if (!mParameter)
    return;

  // propagate top -> down the template regenerator flag
  if (isTemplateRegenerator())
    parameter->setTemplateRegenerator(true);

  mParameter->mInternalFields.append(this);
  connect(this, &QObject::destroyed, mParameter, &OmField::removeInternalField);

  // copy parameter value to field
  if (!skipCopy)
    mValue->copyFrom(mParameter->value());

  OmMFNode *mfnode = dynamic_cast<OmMFNode *>(mParameter->value());
  if (mfnode) {
    connect(mfnode, &OmMFNode::itemChanged, mParameter, &OmField::parameterNodeChanged, Qt::UniqueConnection);
    connect(mfnode, &OmMFNode::itemInserted, mParameter, &OmField::parameterNodeInserted, Qt::UniqueConnection);
    connect(mfnode, &OmMFNode::itemRemoved, mParameter, &OmField::parameterNodeRemoved, Qt::UniqueConnection);

  } else {
    // make sure the field gets updated when the parameter changes, e.g. by Scene Tree or Supervisor, etc.
    connect(mParameter, &OmField::valueChanged, mParameter, &OmField::parameterChanged, Qt::UniqueConnection);
    connect(mParameter->value(), &OmValue::changedByUser, this->value(), &OmValue::changedByUser, Qt::UniqueConnection);

    // In some case OmniSim modifies the fields directly and not the proto parameters, e.g. changing "translation" or
    // "rotation" fields with the mouse. In these cases we need to propagate the change back to the proto parameters, e.g. in
    // order to update the Scene Tree
    if (!isHidden())
      connect(this, &OmField::valueChanged, mParameter, &OmField::fieldChanged);
  }

  // ODE updates
  const QString &fieldName = name();
  if (fieldName == "translation") {
    connect(static_cast<OmSFVector3 *>(mValue), &OmSFVector3::changedByOde, mParameter, &OmField::fieldChangedByOde);
    connect(static_cast<OmSFVector2 *>(mValue), &OmSFVector2::changedByOmniSim, mParameter, &OmField::fieldChangedByOde);
  } else if (fieldName == "rotation")
    connect(static_cast<OmSFRotation *>(mValue), &OmSFRotation::changedByOde, mParameter, &OmField::fieldChangedByOde);
  else if (fieldName == "position")
    connect(static_cast<OmSFDouble *>(mValue), &OmSFDouble::changedByOde, mParameter, &OmField::fieldChangedByOde);
}

void OmField::removeInternalField(QObject *field) {
  mInternalFields.removeAll(static_cast<OmField *>(field));
}

// propagate change in proto parameter to a node field
void OmField::parameterChanged() {
  OmSFNode *sfnode = dynamic_cast<OmSFNode *>(mValue);
  if (sfnode && sfnode->value()) {
    OmNode *node = sfnode->value();

    OmNode *instance = NULL;
    foreach (OmField *internalField, mInternalFields) {
      OmNode::setGlobalParentNode(internalField->parentNode(), true);
      instance = node->cloneAndReferenceProtoInstance();
      sfnode = dynamic_cast<OmSFNode *>(internalField->value());
      sfnode->setValue(instance);
    }

  } else {
    foreach (OmField *const field, mInternalFields)
      field->copyValueFrom(this);
  }
}

// propagate node insertion to internal fields of parameter
void OmField::parameterNodeInserted(int index) {
  OmMFNode *mfnode = dynamic_cast<OmMFNode *>(mValue);
  OmNode *const node = mfnode->item(index);

  OmNode *instance = NULL;
  foreach (OmField *internalField, mInternalFields) {
    OmNode::setGlobalParentNode(internalField->parentNode(), true);
    instance = node->cloneAndReferenceProtoInstance();
    mfnode = dynamic_cast<OmMFNode *>(internalField->value());
    mfnode->insertItem(index, instance);
  }
}

// propagate node remotion to internal fields of parameter
void OmField::parameterNodeRemoved(int index) {
  OmMFNode *mfnode = NULL;
  foreach (OmField *const field, mInternalFields) {
    mfnode = dynamic_cast<OmMFNode *>(field->value());
    mfnode->removeItem(index);
  }
}

void OmField::parameterNodeChanged(int index) {
  OmMFNode *mfnode = dynamic_cast<OmMFNode *>(mValue);
  OmNode *const node = mfnode->item(index);
  foreach (OmField *const field, mInternalFields) {
    OmNode *instance = node->cloneAndReferenceProtoInstance();
    mfnode = dynamic_cast<OmMFNode *>(field->value());
    mfnode->setItem(index, instance);
  }
}

// propagate change in a node field to a proto parameter
void OmField::fieldChanged() {
  // do not propagate a node change back to the proto parameter otherwise we would loop infinitly
  // because the break condition (node == node) is not fully functional
  if (singleType() != WB_SF_NODE)
    copyValueFrom(static_cast<OmField *>(sender()));
}

void OmField::fieldChangedByOde() {
  // do not propagate a node change back to the proto parameter otherwise we would loop infinitly
  // because the break condition (node == node) is not fully functional
  mValue->blockSignals(true);
  mValue->copyFrom(static_cast<OmValue *>(sender()));
  mValue->blockSignals(false);
  mValue->emitChangedByOde();
}

void OmField::copyValueFrom(const OmField *other) {
  assert(other);
  mValue->copyFrom(other->mValue);
}

void OmField::defHasChanged() {
  mValue->defHasChanged();
}
