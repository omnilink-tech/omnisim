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

#ifndef OM_DATA_STREAM_HPP
#define OM_DATA_STREAM_HPP

#include <QtCore/QByteArray>

class QByteArray;

class OmDataStream : public QByteArray {
public:
  using QByteArray::QByteArray;

  int mSizePtr;
  int mDataSize;

  OmDataStream &writeRawData(const char *s, int len);

  OmDataStream &operator<<(qint8 n);
  OmDataStream &operator<<(quint8 n);
  OmDataStream &operator<<(qint16 n);
  OmDataStream &operator<<(quint16 n);
  OmDataStream &operator<<(qint32 n);
  OmDataStream &operator<<(quint32 n);
  OmDataStream &operator<<(qint64 n);
  OmDataStream &operator<<(quint64 n);

  OmDataStream &operator<<(bool n);
  OmDataStream &operator<<(float f);
  OmDataStream &operator<<(double f);
  OmDataStream &operator<<(const char *s);

  void increaseNbChunks(unsigned short n);
};

inline OmDataStream &OmDataStream::operator<<(quint8 n) {
  return *this << qint8(n);
}
inline OmDataStream &OmDataStream::operator<<(quint16 n) {
  return *this << qint16(n);
}
inline OmDataStream &OmDataStream::operator<<(quint32 n) {
  return *this << qint32(n);
}
inline OmDataStream &OmDataStream::operator<<(quint64 n) {
  return *this << qint64(n);
}

#endif
