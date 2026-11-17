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

#ifndef OM_TESSELATOR_HPP
#define OM_TESSELATOR_HPP

//
// Description: helper class used to tesselate
//

#include <QtCore/QList>
#include <QtCore/QString>
#include <QtCore/QVector>

class OmVector3;

class OmTesselator {
public:
  static QString tesselate(const QList<QVector<int>> &indexes, const QList<OmVector3> &vertices, QList<QVector<int>> &results);

private:
  OmTesselator() {}
};

#endif
