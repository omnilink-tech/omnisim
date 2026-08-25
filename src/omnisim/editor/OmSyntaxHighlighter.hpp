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

#ifndef OM_SYNTAX_HIGHLIGHTER_HPP
#define OM_SYNTAX_HIGHLIGHTER_HPP

//
// Description: a class for highlighting search-match text in console output
//

#include <QtCore/QRegularExpression>
#include <QtGui/QSyntaxHighlighter>
#include <QtGui/QTextCharFormat>

class QTextDocument;
class OmLanguage;

class OmSyntaxHighlighter : public QSyntaxHighlighter {
  Q_OBJECT

public:
  static OmSyntaxHighlighter *createForLanguage(OmLanguage *language, QTextDocument *parent,
                                                OmSyntaxHighlighter *previousHighlighter = NULL);

public slots:
  void setSearchTextRule(const QRegularExpression &regularExpression);

protected:
  explicit OmSyntaxHighlighter(QTextDocument *parent);
  void highlightSearchText(const QString &text, int offset = 0);
  void highlightBlock(const QString &text) override;

  QRegularExpression mSearchTextRule;
  QTextCharFormat mSearchTextFormat;
};

#endif
