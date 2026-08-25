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

#ifndef OM_MF_VECTOR2_HPP
#define OM_MF_VECTOR2_HPP

//
// Description: field value that contains a multiple OmVector2
//

#include "OmMultipleValue.hpp"
#include "OmVector2.hpp"
#include "OmWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class OmMFVector2 : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmMFIterator<OmMFVector2, OmVector2> Iterator;

  OmMFVector2(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFVector2(const OmMFVector2 &other) : mVector(other.mVector) {}
  virtual ~OmMFVector2() override {}
  OmValue *clone() const override { return new OmMFVector2(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mVector.size(); }
  void clear() override;
  void writeItem(OmWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << itemToString(index, writer.isOmniSim() ? OmPrecision::DOUBLE_MAX : OmPrecision::FLOAT_MAX);
  }
  void insertDefaultItem(int index) override;
  OmVariant defaultNewVariant() const override { return OmVariant(OmVector2()); }
  void removeItem(int index) override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_VEC2F; }
  const OmVector2 &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, const OmVector2 &vec);
  void addItem(const OmVector2 &vec);
  void insertItem(int index, const OmVector2 &vec);
  void mult(double factor);
  OmMFVector2 &operator=(const OmMFVector2 &other);
  bool operator==(const OmMFVector2 &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;

private:
  QVector<OmVector2> mVector;
};

#endif
