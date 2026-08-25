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
// Copyright 2026 OmniLink
// Stable, opt-in precompiled header for OmniSim developer builds.

#ifndef OM_BUILD_PCH_HPP
#define OM_BUILD_PCH_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// ---- QtCore ---------------------------------------------------------------
// The stdlib block above is safe but nearly worthless on its own: MEASURED
// 2026-08-14 (machine 9722d23d12a3) it bought 1.13-1.26x, which is plausibly
// why OMNISIM_USE_PCH was never switched on.  The cost is Qt, not <vector>.
// A 523-line file (nodes/OmCloth.cpp) compiles in 2.6 s even at -O0, because
// almost all of that is header parsing.  Adding these takes the same TUs to:
//
//     nodes/OmCloth.cpp            2578 -> 1264 ms   (2.04x)
//     physics/OmNewtonBackend.cpp  3820 -> 2414 ms   (1.58x)
//     nodes/OmSolid.cpp           11838 -> 8970 ms   (1.32x)
//
// The gain shrinks as a file's own code grows, so it is largest exactly where
// a feature lane lives: new, small translation units.
//
// ⚠ QtCore ONLY, deliberately.  Every module's include set carries
// QT_CORE_INCLUDE (see WB_MATHS_INCLUDE, the narrowest one), so a QtCore PCH
// resolves for every C++ TU in the build.  QtGui / QtWidgets / QtNetwork do
// NOT appear in all of them -- putting those here would make the PCH
// unresolvable for the modules that lack the include path, and GCC's failure
// mode is a SILENT fallback to parsing the header normally, which would then
// pull Qt into TUs that never wanted it and make the build slower rather than
// erroring.  Keep this list inside the intersection.
#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QList>
#include <QtCore/QVector>
#include <QtCore/QMap>
#include <QtCore/QHash>
#include <QtCore/QSet>
#include <QtCore/QFile>
#include <QtCore/QDir>
#include <QtCore/QTextStream>
#include <QtCore/QDataStream>
#include <QtCore/QVariant>
#include <QtCore/QDateTime>

#endif
