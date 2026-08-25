// Copyright 2026 OmniLink
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

#ifndef OM_WORLD_FILE_FORMAT_HPP
#define OM_WORLD_FILE_FORMAT_HPP

//
// Description: the OmniSim world-file extension policy -- DUAL-READ, SINGLE-WRITE.
//
//   OmniSim reads BOTH `.omniworld` and `.wbt`, indefinitely. A `.wbt` world
//   keeps working forever: there are external users and forks, and nothing in
//   this policy is allowed to break them.
//
//   OmniSim WRITES only `.omniworld`. Save, Save As, the new-world wizard, the
//   project templates and every generator emit the new extension.
//
//   Why a new extension at all: `.wbt` means "OmniSim", and the format has
//   genuinely forked. `URDFRobot`, `Cloth`, the `omnisim://` URL scheme and
//   every `newton*` field are unloadable in Webots, and legacy nodes like
//   `Fluid` / `immersionProperties` now hard-ERROR here. The extension is a
//   CAPABILITY SIGNAL, not a rebrand.
//
//   This mirrors the precedent already set by the file header, which migrated
//   `#VRML_SIM R2025a utf8` -> `#OMNISIM R2025a utf8` while the tokenizer kept
//   accepting both (see OmTokenizer::checkFileHeader).
//
//   EVERY extension test in the engine must go through this header. Do not
//   hand-roll `endsWith(".wbt")` -- that is how half a dual-read policy ships.
//

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QString>
#include <QtCore/QStringList>

namespace OmWorldFileFormat {

  // The one extension OmniSim writes. Includes the leading dot.
  inline const QString &writeExtension() {
    static const QString ext(".omniworld");
    return ext;
  }

  // The legacy extension, still read forever. Includes the leading dot.
  inline const QString &legacyExtension() {
    static const QString ext(".wbt");
    return ext;
  }

  // Every extension accepted on read, preferred first.
  inline const QStringList &readExtensions() {
    static const QStringList exts = QStringList() << writeExtension() << legacyExtension();
    return exts;
  }

  // Suffixes without the leading dot -- for QFileInfo::suffix() comparisons.
  inline const QStringList &readSuffixes() {
    static const QStringList sfx = QStringList() << "omniworld"
                                                 << "wbt";
    return sfx;
  }

  // true if `path` names a world file in either accepted extension.
  inline bool isWorldFile(const QString &path) {
    foreach (const QString &ext, readExtensions()) {
      if (path.endsWith(ext, Qt::CaseInsensitive))
        return true;
    }
    return false;
  }

  // true if `suffix` (no leading dot, e.g. from QFileInfo::suffix()) is a world suffix.
  inline bool isWorldSuffix(const QString &suffix) {
    foreach (const QString &sfx, readSuffixes()) {
      if (QString::compare(suffix, sfx, Qt::CaseInsensitive) == 0)
        return true;
    }
    return false;
  }

  // Directory name filters for QDir::setNameFilters() -- both extensions.
  inline const QStringList &nameFilters() {
    static const QStringList filters = QStringList() << "*.omniworld"
                                                     << "*.wbt";
    return filters;
  }

  // Human-readable filter string for QFileDialog. Not translated here: the
  // caller wraps it in tr() where a translation context exists.
  inline const QString &fileDialogFilter() {
    static const QString filter("World Files (*.omniworld *.OMNIWORLD *.wbt *.WBT)");
    return filter;
  }

  // Drop whichever world extension `path` carries. Returns `path` unchanged
  // when it carries neither, so it is safe to call unconditionally.
  inline QString stripExtension(const QString &path) {
    foreach (const QString &ext, readExtensions()) {
      if (path.endsWith(ext, Qt::CaseInsensitive))
        return path.left(path.length() - ext.length());
    }
    return path;
  }

  // Replace whichever world extension `path` carries with `replacement`
  // (which should include its own leading dot, e.g. ".jpg"). Used for the
  // sibling artefacts -- thumbnails, `_net` dirs -- that shadow a world.
  inline QString replaceExtension(const QString &path, const QString &replacement) {
    return stripExtension(path) + replacement;
  }

  // Force `path` to the write extension: converts a legacy `.wbt` name and
  // appends to an extension-less one. The single funnel for "what do we save as".
  inline QString toWriteExtension(const QString &path) {
    if (path.endsWith(writeExtension(), Qt::CaseInsensitive))
      return path;
    if (path.endsWith(legacyExtension(), Qt::CaseInsensitive))
      return stripExtension(path) + writeExtension();
    return path + writeExtension();
  }

  // Resolve a world path that may be named with either extension: returns the
  // path as given when it exists, otherwise the sibling in the other extension
  // if THAT exists, otherwise the path as given. Lets a stored reference to
  // `foo.wbt` keep resolving after the tree is migrated to `foo.omniworld`
  // (and vice versa) -- recent-files lists, launcher manifests, `--world` args.
  inline QString resolveExisting(const QString &path) {
    if (path.isEmpty() || QFileInfo::exists(path))
      return path;
    foreach (const QString &ext, readExtensions()) {
      if (!path.endsWith(ext, Qt::CaseInsensitive))
        continue;
      const QString stem = path.left(path.length() - ext.length());
      foreach (const QString &alt, readExtensions()) {
        if (alt == ext)
          continue;
        if (QFileInfo::exists(stem + alt))
          return stem + alt;
      }
      break;
    }
    return path;
  }

}  // namespace OmWorldFileFormat

#endif
