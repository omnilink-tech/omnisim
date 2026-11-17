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

#include "OmDataStream.hpp"
#include <QtCore/QDataStream>
#include <QtCore/QIODevice>

OmDataStream &OmDataStream::writeRawData(const char *s, int len) {
  return (OmDataStream &)append(s, len);
}

OmDataStream &OmDataStream::operator<<(qint8 n) {
  return (OmDataStream &)append(n);
}

OmDataStream &OmDataStream::operator<<(qint16 n) {
  *this << quint8(n) << quint8(n >> 8);
  return *this;
}

OmDataStream &OmDataStream::operator<<(qint32 n) {
  *this << quint16(n) << quint16(n >> 16);
  return *this;
}

OmDataStream &OmDataStream::operator<<(qint64 n) {
  *this << quint32(n) << quint32(n >> 32);
  return *this;
}

OmDataStream &OmDataStream::operator<<(bool n) {
  return *this << qint8(n);
}

OmDataStream &OmDataStream::operator<<(float f) {
  return (OmDataStream &)append(reinterpret_cast<const char *>(&f), sizeof(f));
}

OmDataStream &OmDataStream::operator<<(double f) {
  return (OmDataStream &)append(reinterpret_cast<const char *>(&f), sizeof(f));
}

OmDataStream &OmDataStream::operator<<(const char *s) {
  return (OmDataStream &)append(s);
}

void OmDataStream::increaseNbChunks(unsigned short n) {
  unsigned short nbChunks;
  QDataStream ds((QByteArray) * this);
  ds.setByteOrder(QDataStream::LittleEndian);
  ds.device()->seek(0);
  ds >> nbChunks;

  OmDataStream newNbChunks(0);
  unsigned short newNbChunksValue = nbChunks + n;
  newNbChunks << newNbChunksValue;
  replace(0, (int)sizeof(unsigned short), newNbChunks);
}
