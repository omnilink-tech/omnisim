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

#ifndef OM_POLYGON_HPP
#define OM_POLYGON_HPP

//
// Description: 2D Polygon
//

// Deliberately Qt-free (core-evolution-plan.md, Phase Q2): maths/ is future-core code.
// Formerly subclassed QVarLengthArray<OmVector2, 32>; a std::vector with an equivalent
// reserve gives the same growth behavior for the support-polygon use case.
#include <vector>

#include "OmVector2.hpp"

class OmPolygon {
public:
  OmPolygon();
  virtual ~OmPolygon();
  // Container surface (the subset of the former QVarLengthArray API in actual use)
  int size() const { return (int)mVertices.size(); }
  void resize(int size) { mVertices.resize(size); }
  OmVector2 &operator[](int i) { return mVertices[i]; }
  const OmVector2 &operator[](int i) const { return mVertices[i]; }
  const OmVector2 &value(int i) const { return mVertices[i]; }
  const OmVector2 &at(int i) const { return mVertices.at(i); }
  // Accessor
  int actualSize() const {
    // mSize is used to avoid reallocation memory when the number of vertices decreases (we always have: actualSize() <= size())
    return mSize;
  }
  void setActualSize(int size) { mSize = size; }
  bool contains(const OmVector2 &point) const;
  bool contains(double x, double y) const;

private:
  std::vector<OmVector2> mVertices;
  int mSize;  // number of vertices
};

#endif  // OM_POLYGON_HPP
