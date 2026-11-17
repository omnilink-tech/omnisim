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

#ifndef OM_MF_BOOL_HPP
#define OM_MF_BOOL_HPP

//
// Description: field value that contains a multiple bool
//

#include "OmMultipleValue.hpp"
#include "OmWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class OmMFBool : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmMFIterator<OmMFBool, bool> Iterator;

  OmMFBool() {}
  OmMFBool(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFBool(const OmMFBool &other) : mVector(other.mVector) {}
  virtual ~OmMFBool() override {}
  OmValue *clone() const override { return new OmMFBool(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(OmWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, OmPrecision::DOUBLE_MAX);
  }
  void insertDefaultItem(int index) override;
  OmVariant defaultNewVariant() const override { return OmVariant(true); }
  void removeItem(int index) override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_BOOL; }
  const bool &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, bool b);
  void addItem(const bool &b);
  void insertItem(int index, const bool &b);
  OmMFBool &operator=(const OmMFBool &other);
  bool operator==(const OmMFBool &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<bool> mVector;
};

#endif
