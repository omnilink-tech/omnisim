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

#include "WbSyntaxHighlighter.hpp"

#include <QtGui/QTextDocument>

WbSyntaxHighlighter *WbSyntaxHighlighter::createForLanguage(WbLanguage *language, QTextDocument *parent,
                                                            WbSyntaxHighlighter *previousHighlighter) {
  Q_UNUSED(language);
  WbSyntaxHighlighter *highlighter = new WbSyntaxHighlighter(parent);
  if (previousHighlighter != NULL) {
    highlighter->setSearchTextRule(previousHighlighter->mSearchTextRule);
    delete previousHighlighter;
  }
  return highlighter;
}

WbSyntaxHighlighter::WbSyntaxHighlighter(QTextDocument *parent) : QSyntaxHighlighter(parent) {
  mSearchTextFormat.setBackground(Qt::gray);
}

void WbSyntaxHighlighter::setSearchTextRule(const QRegularExpression &regularExpression) {
  if (mSearchTextRule == regularExpression)
    return;

  mSearchTextRule = regularExpression;
  rehighlight();
}

void WbSyntaxHighlighter::highlightBlock(const QString &text) {
  highlightSearchText(text);
  setCurrentBlockState(0);
}

void WbSyntaxHighlighter::highlightSearchText(const QString &text, int offset) {
  if (mSearchTextRule.pattern().isEmpty())
    return;

  QRegularExpressionMatch match = mSearchTextRule.match(text);
  for (int index = 0; index <= match.lastCapturedIndex(); ++index)
    setFormat(match.capturedStart() + offset, match.capturedLength(), mSearchTextFormat);
}
