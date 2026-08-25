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

#ifndef OM_MF_INT_HPP
#define OM_MF_INT_HPP

//
// Description: field value that contains a multiple integers
//

#include "OmMultipleValue.hpp"
#include "OmWriter.hpp"

#include <QtCore/QVector>

#include <cassert>

class OmMFInt : public OmMultipleValue {
  Q_OBJECT

public:
  typedef OmMFIterator<OmMFInt, int> Iterator;

  OmMFInt() {}
  OmMFInt(OmTokenizer *tokenizer, const QString &worldPath) { read(tokenizer, worldPath); }
  OmMFInt(const OmMFInt &other) : mVector(other.mVector) {}
  virtual ~OmMFInt() override {}
  OmValue *clone() const override { return new OmMFInt(*this); }
  bool equals(const OmValue *other) const override;
  void copyFrom(const OmValue *other) override;
  int size() const override { return mVector.size(); }
  bool isEmpty() const { return mVector.empty(); }
  void clear() override;
  void writeItem(OmWriter &writer, int index) const override {
    assert(index >= 0 && index < size());
    writer << mVector[index];
  }
  void insertDefaultItem(int index) override;
  OmVariant defaultNewVariant() const override { return OmVariant(0); }
  void removeItem(int index) override;
  OmVariant variantValue(int index) const override {
    assert(index >= 0 && index < size());
    return OmVariant(mVector[index]);
  }
  WbFieldType type() const override { return WB_MF_INT32; }
  const int &item(int index) const {
    assert(index >= 0 && index < size());
    return mVector[index];
  }
  void setItem(int index, int value);
  void addItem(int value);
  void insertItem(int index, int value);
  void normalizeIndices();  // indices smaller than -1 are changed to -1
  OmMFInt &operator=(const OmMFInt &other);
  bool operator==(const OmMFInt &other) const { return mVector == other.mVector; }

protected:
  void readAndAddItem(OmTokenizer *tokenizer, const QString &worldPath) override;
  bool smallSeparator(int i) const override;

private:
  QVector<int> mVector;
};

#endif
