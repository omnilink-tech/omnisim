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

#ifndef OM_FIELD_CHECKER_HPP
#define OM_FIELD_CHECKER_HPP

class OmBaseNode;
class OmField;
class OmSFColor;
class OmSFDouble;
class OmSFInt;
class OmSFVector2;
class OmSFVector3;
class OmMFColor;
class OmRgb;
class OmValue;
class OmVector2;
class OmVector3;

#include <QtCore/QObject>

// NOTE: The pattern "return if changed" at the OmFieldChecker caller level is difficult to remove.
//       Indeed, I hoped solving that by blocking the signals of the OmValue when setting it to default
//       but this doesn't work in some cases (e.g. PROTOs)
//       Blocking the signals from value->node for a while is required to achieve,
//       and this is complex (and perhaps impossible):
//         http://stackoverflow.com/questions/15633086/qt-block-temporarily-signals-between-2-qobjects

class OmFieldChecker : public QObject {
  Q_OBJECT

public:
  static bool resetDoubleIfNegative(const OmBaseNode *node, OmSFDouble *value, double defaultValue);
  static bool resetDoubleIfNonPositive(const OmBaseNode *node, OmSFDouble *value, double defaultValue);
  static bool resetDoubleIfNotInRangeWithIncludedBounds(const OmBaseNode *node, OmSFDouble *value, double min, double max,
                                                        double defaultValue);
  static bool resetDoubleIfNotInRangeWithExcludedBounds(const OmBaseNode *node, OmSFDouble *value, double min, double max,
                                                        double defaultValue);
  static bool resetDoubleIfLess(const OmBaseNode *node, OmSFDouble *value, double threshold, double defaultValue);
  static bool resetDoubleIfGreater(const OmBaseNode *node, OmSFDouble *value, double threshold, double defaultValue);
  static bool resetDoubleIfNegativeAndNotDisabled(const OmBaseNode *node, OmSFDouble *value, double defaultValue,
                                                  double disableValue);
  static bool resetDoubleIfNonPositiveAndNotDisabled(const OmBaseNode *node, OmSFDouble *value, double defaultValue,
                                                     double disableValue);
  static bool resetDoubleIfNotInRangeWithIncludedBoundsAndNotDisabled(const OmBaseNode *node, OmSFDouble *value, double min,
                                                                      double max, double disableValue, double defaultValue);

  static bool clampDoubleToRangeWithIncludedBounds(const OmBaseNode *node, OmSFDouble *value, double min, double max);

  static bool resetIntIfNegative(const OmBaseNode *node, OmSFInt *value, int defaultValue);
  static bool resetIntIfNonPositive(const OmBaseNode *node, OmSFInt *value, int defaultValue);
  static bool resetIntIfLess(const OmBaseNode *node, OmSFInt *value, int threshold, int defaultValue);
  static bool resetIntIfNotInRangeWithIncludedBounds(const OmBaseNode *node, OmSFInt *value, int min, int max,
                                                     int defaultValue);
  static bool resetIntIfNonPositiveAndNotDisabled(const OmBaseNode *node, OmSFInt *value, int defaultValue, int disableValue);
  static bool resetIntIfNegativeAndNotDisabled(const OmBaseNode *node, OmSFInt *value, int defaultValue, int disableValue);

  static bool resetVector2IfNonPositive(const OmBaseNode *node, OmSFVector2 *value, const OmVector2 &defaultValue);

  static bool resetVector3IfNegative(const OmBaseNode *node, OmSFVector3 *value, const OmVector3 &defaultValue);
  static bool resetVector3IfNonPositive(const OmBaseNode *node, OmSFVector3 *value, const OmVector3 &defaultValue);

  static bool resetColorIfInvalid(const OmBaseNode *node, OmSFColor *value);
  static bool resetMultipleColorIfInvalid(const OmBaseNode *node, OmMFColor *value);

private:
  static const OmField *findField(const OmBaseNode *node, OmValue *value);

  static bool clampRgb(OmRgb &rgb);

  OmFieldChecker() {}
};

#endif
