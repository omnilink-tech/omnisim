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

#ifndef OM_VALUE_HPP
#define OM_VALUE_HPP

//
// Description:
//   OmValue is the abstract base class for any type of value that can be stored in a OmField
//
// Inherited by:
//   OmSingleValue, OmMultipleValue
//

#include <QtCore/QObject>
#include "../../../include/controller/c/omnisim/supervisor.h"

#include "OmPrecision.hpp"

class OmTokenizer;
class OmWriter;

class OmValue : public QObject {
  Q_OBJECT

public:
  virtual ~OmValue();

  // virtual copy constructor
  virtual OmValue *clone() const = 0;

  // read the value
  virtual void read(OmTokenizer *tokenizer, const QString &worldPath) = 0;

  // write the value
  virtual void write(OmWriter &writer) const = 0;

  // virtual comparison and assignment
  virtual bool equals(const OmValue *other) const = 0;
  virtual void copyFrom(const OmValue *other) = 0;
  void emitChangedByOde() { emit changedByOde(); }

  // string for the GUI
  // level is not meaningful in all the subclasses.
  virtual QString toString(OmPrecision::Level level = OmPrecision::DOUBLE_MAX) const = 0;

  // field type as used supervisor functions
  virtual WbFieldType type() const = 0;
  WbFieldType singleType() const;
  bool isSingle() const { return OmValue::isSingle(type()); }
  bool isMultiple() const { return OmValue::isMultiple(type()); }

  // e.g. "SFNode", "SFVec3f", "SFInt32", etc.
  QString vrmlTypeName() const;

  // e.g. "Node", "Vector3", "Int"
  QString shortTypeName() const;

  // static operation for the field type
  static bool isSingle(WbFieldType type);
  static bool isMultiple(WbFieldType type);
  static WbFieldType toSingle(WbFieldType type);
  static WbFieldType vrmlNameToType(const QString &vrmlName);
  static QString typeToVrmlName(WbFieldType type);
  static QString typeToShortName(WbFieldType type);

  virtual void defHasChanged() {}

signals:
  // emitted after the content of the value was changed
  void changed();
  void changedByOde();
  void changedByUser(bool changedFromSupervisor);
  void changedByOmniSim();

protected:
  // abstract class cannot be instantiated
  OmValue();

private:
};

#endif
