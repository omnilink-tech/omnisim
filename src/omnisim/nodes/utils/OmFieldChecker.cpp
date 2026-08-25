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

#include "OmFieldChecker.hpp"

#include "OmBaseNode.hpp"
#include "OmField.hpp"
#include "OmMFColor.hpp"
#include "OmRgb.hpp"
#include "OmSFColor.hpp"
#include "OmSFDouble.hpp"
#include "OmSFInt.hpp"
#include "OmSFVector2.hpp"
#include "OmSFVector3.hpp"
#include "OmValue.hpp"
#include "OmVector2.hpp"
#include "OmVector3.hpp"

/*
In English:

- "x is positive" means "x > 0"
- "x is negative" means "x < 0"

Hence:

- "x is non-negative" means "x >= 0"
- "x is non-positive" means "x <= 0"
*/

bool OmFieldChecker::resetDoubleIfNegative(const OmBaseNode *node, OmSFDouble *value, double defaultValue) {
  if (value->value() < 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be non-negative.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfNonPositive(const OmBaseNode *node, OmSFDouble *value, double defaultValue) {
  if (value->value() <= 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfNegativeAndNotDisabled(const OmBaseNode *node, OmSFDouble *value, double defaultValue,
                                                         double disableValue) {
  if (value->value() < 0 && (value->value() != disableValue)) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or non-negative.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfNonPositiveAndNotDisabled(const OmBaseNode *node, OmSFDouble *value, double defaultValue,
                                                            double disableValue) {
  if (value->value() <= 0 && (value->value() != disableValue)) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or positive.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(const OmBaseNode *node, OmSFDouble *value,
                                                                             double min, double max, double disableValue,
                                                                             double defaultValue) {
  if (value->value() != disableValue && (value->value() < min || value->value() > max)) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in either %3 or in range [%4, %5].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfNotInRangeWithIncludedBounds(const OmBaseNode *node, OmSFDouble *value, double min,
                                                               double max, double defaultValue) {
  if (value->value() < min || value->value() > max) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range [%3, %4].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::clampDoubleToRangeWithIncludedBounds(const OmBaseNode *node, OmSFDouble *value, double min, double max) {
  double defaultValue = value->value();
  if (value->value() < min)
    defaultValue = min;
  else if (value->value() > max)
    defaultValue = max;

  if (defaultValue != value->value()) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range [%3, %4].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfNotInRangeWithExcludedBounds(const OmBaseNode *node, OmSFDouble *value, double min,
                                                               double max, double defaultValue) {
  if (value->value() <= min || value->value() >= max) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range ]%3, %4[.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfLess(const OmBaseNode *node, OmSFDouble *value, double threshold, double defaultValue) {
  if (value->value() < threshold) {
    const OmField *field = findField(node, value);
    node->parsingWarn(
      tr("Invalid '%1' changed to %2. The value should be %3 or greater.").arg(field->name()).arg(defaultValue).arg(threshold));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetDoubleIfGreater(const OmBaseNode *node, OmSFDouble *value, double threshold, double defaultValue) {
  if (value->value() > threshold) {
    const OmField *field = findField(node, value);
    node->parsingWarn(
      tr("Invalid '%1' changed to %2. The value should be %3 or less.").arg(field->name()).arg(defaultValue).arg(threshold));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetIntIfNegative(const OmBaseNode *node, OmSFInt *value, int defaultValue) {
  if (value->value() < 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be non-negative.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetIntIfNonPositive(const OmBaseNode *node, OmSFInt *value, int defaultValue) {
  if (value->value() <= 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.").arg(field->name()).arg(defaultValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetIntIfLess(const OmBaseNode *node, OmSFInt *value, int threshold, int defaultValue) {
  if (value->value() < threshold) {
    const OmField *field = findField(node, value);
    node->parsingWarn(
      tr("Invalid '%1' changed to %2. The value should be %3 or greater.").arg(field->name()).arg(defaultValue).arg(threshold));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetIntIfNotInRangeWithIncludedBounds(const OmBaseNode *node, OmSFInt *value, int min, int max,
                                                            int defaultValue) {
  if (value->value() < min || value->value() > max) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be in range [%3, %4].")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(min)
                        .arg(max));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetIntIfNonPositiveAndNotDisabled(const OmBaseNode *node, OmSFInt *value, int defaultValue,
                                                         int disableValue) {
  if (value->value() <= 0 && value->value() != disableValue) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or positive.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetIntIfNegativeAndNotDisabled(const OmBaseNode *node, OmSFInt *value, int defaultValue,
                                                      int disableValue) {
  if (value->value() < 0 && value->value() != disableValue) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be either %3 or non-negative.")
                        .arg(field->name())
                        .arg(defaultValue)
                        .arg(disableValue));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetVector2IfNonPositive(const OmBaseNode *node, OmSFVector2 *value, const OmVector2 &defaultValue) {
  if (value->x() <= 0 || value->y() <= 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.")
                        .arg(field->name())
                        .arg(defaultValue.toString(OmPrecision::GUI_MEDIUM)));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetVector3IfNegative(const OmBaseNode *node, OmSFVector3 *value, const OmVector3 &defaultValue) {
  if (value->x() < 0 || value->y() < 0 || value->z() < 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be non-negative.")
                        .arg(field->name())
                        .arg(defaultValue.toString(OmPrecision::GUI_MEDIUM)));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetVector3IfNonPositive(const OmBaseNode *node, OmSFVector3 *value, const OmVector3 &defaultValue) {
  if (value->x() <= 0 || value->y() <= 0 || value->z() <= 0) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2. The value should be positive.")
                        .arg(field->name())
                        .arg(defaultValue.toString(OmPrecision::GUI_MEDIUM)));
    value->setValue(defaultValue);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetColorIfInvalid(const OmBaseNode *node, OmSFColor *value) {
  OmRgb rgb = value->value();
  if (rgb.clampValuesIfNeeded()) {
    const OmField *field = findField(node, value);
    node->parsingWarn(tr("Invalid '%1' changed to %2.").arg(field->name()).arg(rgb.toString(OmPrecision::GUI_MEDIUM)));
    value->setValue(rgb);
    return true;
  }
  return false;
}

bool OmFieldChecker::resetMultipleColorIfInvalid(const OmBaseNode *node, OmMFColor *value) {
  bool changed = false;
  int size = value->size();
  const OmField *field = findField(node, value);
  for (int i = 0; i < size; i++) {
    OmRgb rgb = value->item(i);
    if (rgb.clampValuesIfNeeded()) {
      node->parsingWarn(
        tr("Invalid item %1 of '%2' changed to %3.").arg(i).arg(field->name()).arg(rgb.toString(OmPrecision::GUI_MEDIUM)));
      value->setItem(i, rgb);
      changed = true;
    }
  }
  return changed;
}

const OmField *OmFieldChecker::findField(const OmBaseNode *node, OmValue *value) {
  foreach (const OmField *field, node->fields())
    if (field->value() == value)
      return field;
  return NULL;
}
